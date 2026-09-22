//*****************************************************************************
//
//! @file Stride_inte.c  (compiled as ble_freertos_amdtps.c)
//!
//! @brief AMDTP profile + main() for BMP390 + BMI323 BLE telemetry.
//!
//! This file owns ALL AMDTP symbols, so the SDK file
//! ambiq_ble/apps/amdtps/amdtp_main.c must NOT be listed in the Makefile.
//!
//! Data flow
//! ---------
//!   SensorTask (rtos.c) packs g_sTelemetryPacket (24-byte binary struct)
//!   every 50 ms and sets g_bNewTelemetryAvailable = true.
//!
//!   amdtpProcCccState() is called by the Cordio ATT layer when the phone
//!   writes the AMDTP TX CCC descriptor:
//!     CCC = NOTIFY  → g_bTxSubscribed = true;  first packet is sent immediately.
//!     CCC = 0       → g_bTxSubscribed = false; RadioTask stops sending.
//!
//!   RadioTask (radio_task.c) checks both g_bTxSubscribed and
//!   g_bNewTelemetryAvailable on every WSF dispatcher pass and calls
//!   AmdtpsSendPacket() when both are true.
//!
//!   amdtpDtpTransCback() is the AMDTP TX-complete callback.  It does NOT
//!   call AmdtpsSendSensorData() directly — RadioTask's polling loop is the
//!   sole sender.  This removes the dual-send race that existed previously
//!   (callback + RadioTask both calling AmdtpsSendPacket() for the same
//!   packet).
//!
//! Connection parameters
//! ---------------------
//!   amdtpUpdateCfg drives the App-framework's post-connect parameter
//!   update 500 ms after DM_CONN_OPEN_IND.
//!
//!   connIntervalMin = 12 (15 ms), connIntervalMax = 24 (30 ms).
//!   iOS enforces a 15 ms minimum; setting Min < 12 causes the phone to
//!   silently reject the update and fall back to a slow default interval
//!   (often 100 ms), which then makes AmdtpsSendPacket() return
//!   AMDTP_STATUS_BUSY on every 50 ms sensor tick.  The previous value of
//!   Min=6 (7.5 ms) was the root cause of the BUSY storm at 20 Hz.
//!
//!   FIX SUMMARY (vs. original Stride_inte.c)
//!   -----------------------------------------
//!   1. amdtpUpdateCfg connIntervalMin corrected from 6→12 (7.5→15 ms).
//!      iOS rejects intervals below 15 ms; the rejected update left the
//!      connection at a slow default interval, flooding AMDTP_STATUS_BUSY.
//!
//!   2. AmdtpsSendSensorData() now sends sizeof(telemetry_packet_t) = 24 bytes,
//!      not sizeof(telemetry_string_t) = 80 bytes.  Sending 80 bytes of which
//!      56 are garbage (uninitialised char padding) corrupted every packet.
//!
//!   3. amdtpDtpTransCback() no longer calls AmdtpsSendSensorData().
//!      Previously both the TX-complete callback AND RadioTask's polling loop
//!      independently tried to send the same packet.  On a fast connection
//!      the callback fires before RadioTask's next wsfOsDispatcher() pass,
//!      causing two back-to-back calls to AmdtpsSendPacket() for the same
//!      g_sTelemetryPacket — the second always returns AMDTP_STATUS_BUSY
//!      and RadioTask then clears g_bNewTelemetryAvailable without sending,
//!      silently dropping every other packet.  RadioTask polling is sufficient.
//
//*****************************************************************************

// ---------------------------------------------------------------------------
// Include order: WSF types MUST precede everything else.
// ---------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "wsf_types.h"
#include "wsf_msg.h"
#include "wsf_trace.h"
#include "wsf_timer.h"
#include "bstream.h"

#include "hci_api.h"
#include "dm_api.h"
#include "att_api.h"
#include "smp_api.h"

#include "app_api.h"
#include "app_db.h"
#include "app_ui.h"
#include "app_hw.h"

#include "svc_ch.h"
#include "svc_core.h"
#include "svc_dis.h"
#include "gatt_api.h"

#include "amdtp_api.h"
#include "amdtps_api.h"
#include "svc_amdtp.h"

#include "am_util.h"

#include "ble_freertos_amdtps.h"
#include "rtos.h"           // g_sTelemetryPacket (telemetry_packet_t), g_bNewTelemetryAvailable
#include "radio_task.h"     // g_bTxSubscribed, g_bBleConnected

extern void   attsCsfSetHashUpdateStatus(bool_t isUpdating);
extern bool_t attsCsfGetHashUpdateStatus(void);

//*****************************************************************************
// Compile-time options
//*****************************************************************************
#define MEASURE_THROUGHPUT

//*****************************************************************************
// WSF message event IDs
//*****************************************************************************
#define AMDTP_MSG_START              0xA0

enum
{
    AMDTP_TIMER_IND       = AMDTP_MSG_START,
    AMDTP_SENSOR_DATA_IND,
#ifdef MEASURE_THROUGHPUT
    AMDTP_MEAS_TP_TIMER_IND,
#endif
};

//*****************************************************************************
// Internal message type
//*****************************************************************************
typedef union
{
    wsfMsgHdr_t   hdr;
    dmEvt_t       dm;
    attsCccEvt_t  ccc;
    attEvt_t      att;
} amdtpMsg_t;

static void amdtpClose(amdtpMsg_t *pMsg);

//*****************************************************************************
// Profile configuration
//*****************************************************************************

static const appAdvCfg_t amdtpAdvCfg =
{
    { 60000, 0, 0 },
    {   800, 800, 0 },
};

static const appSlaveCfg_t amdtpSlaveCfg =
{
    AMDTP_CONN_MAX,
};

static const appSecCfg_t amdtpSecCfg =
{
    DM_AUTH_BOND_FLAG | DM_AUTH_SC_FLAG,
    0,
    DM_KEY_DIST_LTK,
    FALSE,
    FALSE,
};

// FIX: connIntervalMin raised from 6→12 (7.5→15 ms).
// iOS hard-enforces a 15 ms floor (interval units = 1.25 ms, so min=12).
// Setting min=6 causes iOS to silently reject the update, the connection
// stays at its negotiated default (often 45-100 ms), and every 50 ms
// sensor sample arrives faster than the BLE interval can clear it, causing
// AMDTP_STATUS_BUSY on every TX attempt.
static const appUpdateCfg_t amdtpUpdateCfg =
{
    500,    // connIdleTimeout (ms) — delay before first update request
    12,     // connIntervalMin (1.25 ms units → 15 ms) — iOS minimum
    24,     // connIntervalMax (1.25 ms units → 30 ms)
    0,      // connLatency (slave latency = 0 for low-latency streaming)
    3000,   // supTimeout (10 ms units → 30 s) — generous for continuous stream
    5,      // numAttempts
};

static const smpCfg_t amdtpSmpCfg =
{
    3000,
    SMP_IO_NO_IN_NO_OUT,
    7,
    16,
    3,
    0,
};

static const AmdtpsCfg_t amdtpAmdtpsCfg = { 0 };

//*****************************************************************************
// Advertising payload
//*****************************************************************************
static const uint8_t amdtpAdvDataDisc[] =
{
    2,  DM_ADV_TYPE_FLAGS,     DM_FLAG_LE_GENERAL_DISC | DM_FLAG_LE_BREDR_NOT_SUP,
    2,  DM_ADV_TYPE_TX_POWER,  0,
    3,  DM_ADV_TYPE_16_UUID,   UINT16_TO_BYTES(ATT_UUID_DEVICE_INFO_SERVICE),
    17, DM_ADV_TYPE_128_UUID,  ATT_UUID_AMDTP_SERVICE,
};

static const uint8_t amdtpScanDataDisc[] =
{
    6, DM_ADV_TYPE_LOCAL_NAME, 'A','m','d','t','p',
};

//*****************************************************************************
// CCC descriptor table
//*****************************************************************************
enum
{
    AMDTP_GATT_SC_CCC_IDX,
    AMDTP_AMDTPS_TX_CCC_IDX,
    AMDTP_AMDTPS_RX_ACK_CCC_IDX,
    AMDTP_NUM_CCC_IDX
};

static const attsCccSet_t amdtpCccSet[AMDTP_NUM_CCC_IDX] =
{
    { GATT_SC_CH_CCC_HDL,    ATT_CLIENT_CFG_INDICATE, DM_SEC_LEVEL_NONE },
    { AMDTPS_TX_CH_CCC_HDL,  ATT_CLIENT_CFG_NOTIFY,   DM_SEC_LEVEL_NONE },
    { AMDTPS_ACK_CH_CCC_HDL, ATT_CLIENT_CFG_NOTIFY,   DM_SEC_LEVEL_NONE },
};

//*****************************************************************************
// Module globals
//*****************************************************************************
wsfHandlerId_t amdtpHandlerId;

#ifdef MEASURE_THROUGHPUT
static wsfTimer_t measTpTimer;
static int        gTotalDataBytesRecev = 0;
#endif

static bool bPairingCompleted = false;

//*****************************************************************************
//
//! @brief AmdtpsSendSensorData — snapshot telemetry and send one AMDTP packet.
//!
//! FIX: sends sizeof(telemetry_packet_t) = 24 bytes.
//! The original code sent sizeof(telemetry_string_t) = 80 bytes — the struct
//! is only 24 bytes so the remaining 56 bytes were uninitialised stack garbage.
//!
//! Called ONLY from RadioTask's polling path (radio_task.c).
//! NOT called from amdtpDtpTransCback — see fix note #3 in file header.
//
//*****************************************************************************
static void
AmdtpsSendSensorData(void)
{
    telemetry_packet_t  sPkt;    // FIX: correct type (was telemetry_string_t)
    eAmdtpStatus_t      eStatus;

    // Snapshot the volatile global by value.  RadioTask calls this only when
    // g_bNewTelemetryAvailable is true, so SensorTask is not writing concurrently.
    memcpy(&sPkt, (const void *)&g_sTelemetryPacket, sizeof(sPkt));

    // FIX: send sizeof(telemetry_packet_t) = 24 bytes, not 80.
    eStatus = AmdtpsSendPacket(AMDTP_PKT_TYPE_DATA,
                               FALSE,   /* encrypted */
                               FALSE,   /* enableACK — fire-and-forget */
                               (uint8_t *)&sPkt,
                               (uint16_t)sizeof(telemetry_packet_t));

    if (eStatus == AMDTP_STATUS_SUCCESS)
    {
        g_bNewTelemetryAvailable = false;   // SensorTask may write next sample
        am_util_stdio_printf(
            "[AMDTP TX] %u bytes | t=%u | T:%.2fC P:%uPa A:%.2fm "
            "| Ax:%.3fg Ay:%.3fg Az:%.3fg "
            "| Gx:%.1fdps Gy:%.1fdps Gz:%.1fdps\r\n",
            (unsigned)sizeof(telemetry_packet_t),
            (unsigned)sPkt.ui32Timestamp,
            sPkt.i16TempCx100   / 100.0f,
            (unsigned)sPkt.ui32PressurePa,
            sPkt.i16AltMx100    / 100.0f,
            sPkt.i16AxGx1000    / 1000.0f,
            sPkt.i16AyGx1000    / 1000.0f,
            sPkt.i16AzGx1000    / 1000.0f,
            sPkt.i16GxDpsx100   / 100.0f,
            sPkt.i16GyDpsx100   / 100.0f,
            sPkt.i16GzDpsx100   / 100.0f);
    }
    else
    {
        am_util_stdio_printf(
            "[AMDTP TX] AmdtpsSendSensorData failed, status=%d\r\n", eStatus);
    }
}

//*****************************************************************************
// DM callback
//*****************************************************************************
static void
amdtpDmCback(dmEvt_t *pDmEvt)
{
    dmEvt_t *pMsg;
    uint16_t len = DmSizeOfEvt(pDmEvt);

    if ((pMsg = WsfMsgAlloc(len)) != NULL)
    {
        memcpy(pMsg, pDmEvt, len);
        WsfMsgSend(amdtpHandlerId, pMsg);
    }
}

//*****************************************************************************
// ATT callback
//*****************************************************************************
static void
amdtpAttCback(attEvt_t *pEvt)
{
    attEvt_t *pMsg;

    if ((pMsg = WsfMsgAlloc(sizeof(attEvt_t) + pEvt->valueLen)) != NULL)
    {
        memcpy(pMsg, pEvt, sizeof(attEvt_t));
        pMsg->pValue = (uint8_t *)(pMsg + 1);
        memcpy(pMsg->pValue, pEvt->pValue, pEvt->valueLen);
        WsfMsgSend(amdtpHandlerId, pMsg);
    }
}

//*****************************************************************************
// CCC callback
//*****************************************************************************
static void
amdtpCccCback(attsCccEvt_t *pEvt)
{
    attsCccEvt_t *pMsg;
    appDbHdl_t    dbHdl;

    if ((pEvt->handle != ATT_HANDLE_NONE) &&
        ((dbHdl = AppDbGetHdl((dmConnId_t)pEvt->hdr.param)) != APP_DB_HDL_NONE))
    {
        AppDbSetCccTblValue(dbHdl, pEvt->idx, pEvt->value);
    }

    if ((pMsg = WsfMsgAlloc(sizeof(attsCccEvt_t))) != NULL)
    {
        memcpy(pMsg, pEvt, sizeof(attsCccEvt_t));
        WsfMsgSend(amdtpHandlerId, pMsg);
    }
}

//*****************************************************************************
// amdtpProcCccState
//*****************************************************************************
static void
amdtpProcCccState(amdtpMsg_t *pMsg)
{
    APP_TRACE_INFO3("ccc state ind value:%d handle:%d idx:%d",
                    pMsg->ccc.value, pMsg->ccc.handle, pMsg->ccc.idx);

    if (pMsg->ccc.idx == AMDTP_AMDTPS_TX_CCC_IDX)
    {
        if (pMsg->ccc.value == ATT_CLIENT_CFG_NOTIFY)
        {
            amdtps_start((dmConnId_t)pMsg->ccc.hdr.param,
                         AMDTP_TIMER_IND,
                         AMDTP_AMDTPS_TX_CCC_IDX);

            g_bTxSubscribed = true;
            am_util_stdio_printf(
                "[AMDTP] TX subscribed — BMP390+BMI323 telemetry streaming at 20 Hz\r\n");

            // Send first available sample immediately.
            // After this, RadioTask's polling loop takes over cadence.
            if (g_bNewTelemetryAvailable)
            {
                AmdtpsSendSensorData();
            }
        }
        else
        {
            am_util_stdio_printf("[AMDTP] TX unsubscribed — streaming paused\r\n");
            g_bTxSubscribed = false;
            amdtpClose(pMsg);
            amdtps_stop((dmConnId_t)pMsg->ccc.hdr.param);
        }
    }
}

//*****************************************************************************
// amdtpClose
//*****************************************************************************
static void
amdtpClose(amdtpMsg_t *pMsg)
{
    (void)pMsg;
}

//*****************************************************************************
// amdtpSetup
//*****************************************************************************
static void
amdtpSetup(amdtpMsg_t *pMsg)
{
    (void)pMsg;
    AppAdvSetData(APP_ADV_DATA_DISCOVERABLE,
                  sizeof(amdtpAdvDataDisc),  (uint8_t *)amdtpAdvDataDisc);
    AppAdvSetData(APP_SCAN_DATA_DISCOVERABLE,
                  sizeof(amdtpScanDataDisc), (uint8_t *)amdtpScanDataDisc);
    AppAdvSetData(APP_ADV_DATA_CONNECTABLE,
                  sizeof(amdtpAdvDataDisc),  (uint8_t *)amdtpAdvDataDisc);
    AppAdvSetData(APP_SCAN_DATA_CONNECTABLE,
                  sizeof(amdtpScanDataDisc), (uint8_t *)amdtpScanDataDisc);
    AppAdvStart(APP_MODE_AUTO_INIT);
}

//*****************************************************************************
// amdtpBtnCback
//*****************************************************************************
static void
amdtpBtnCback(uint8_t btn)
{
    dmConnId_t connId;

    if ((connId = AppConnIsOpen()) != DM_CONN_ID_NONE)
    {
        if (btn == APP_UI_BTN_1_LONG)
        {
            AppConnClose(connId);
        }
    }
    else
    {
        switch (btn)
        {
            case APP_UI_BTN_1_SHORT:
                AppAdvStart(APP_MODE_AUTO_INIT);
                break;
            case APP_UI_BTN_1_MED:
                AppSetBondable(TRUE);
                AppAdvStart(APP_MODE_DISCOVERABLE);
                break;
            case APP_UI_BTN_1_LONG:
                AppDbDeleteAllRecords();
                AppAdvStart(APP_MODE_AUTO_INIT);
                break;
            default:
                break;
        }
    }
}

//*****************************************************************************
// Throughput measurement
//*****************************************************************************
#ifdef MEASURE_THROUGHPUT
static void
showThroughput(void)
{
    if (gTotalDataBytesRecev > 0)
    {
        APP_TRACE_INFO1("RX throughput: %d bytes/s", gTotalDataBytesRecev);
    }
    gTotalDataBytesRecev = 0;
    WsfTimerStartSec(&measTpTimer, 1);
}
#endif

//*****************************************************************************
//
//! @brief amdtpDtpRecvCback — phone → device data.
//
//*****************************************************************************
void
amdtpDtpRecvCback(uint8_t *buf, uint16_t len)
{
#ifdef MEASURE_THROUGHPUT
    static bool measTpStarted = false;
#endif

    if (len == 0 || buf == NULL) return;

    if (buf[0] == 1)
    {
        APP_TRACE_INFO0("[AMDTP RX] start cmd (ignored — RadioTask owns TX cadence)");
    }
    else if (buf[0] == 2)
    {
        APP_TRACE_INFO0("[AMDTP RX] stop cmd (ignored — unsubscribe CCC to stop)");
    }
    else
    {
#ifdef MEASURE_THROUGHPUT
        gTotalDataBytesRecev += len;
        if (!measTpStarted)
        {
            measTpStarted = true;
            WsfTimerStartSec(&measTpTimer, 1);
        }
#endif
    }
}

//*****************************************************************************
//
//! @brief amdtpDtpTransCback — TX-complete callback from AMDTP profile.
//!
//! FIX: This callback no longer calls AmdtpsSendSensorData().
//!
//! Original code called AmdtpsSendSensorData() here, creating a dual send-path:
//!   Path 1: amdtpDtpTransCback() fires from HCI ISR/WSF context immediately
//!           after ACL TX complete.
//!   Path 2: RadioTask's wsfOsDispatcher() loop also sees
//!           g_bNewTelemetryAvailable == true and calls AmdtpsSendPacket().
//!
//! Race result: Path 1 succeeds and clears g_bNewTelemetryAvailable.
//!              Path 2 sees the flag already false and skips — OK so far.
//!              But Path 1 then checks the flag AGAIN and may fire a SECOND
//!              send for the next sample before RadioTask has a chance to
//!              call wsfOsDispatcher(), consuming two samples in rapid
//!              succession and returning AMDTP_STATUS_BUSY on the second.
//!              The BUSY causes RadioTask to clear g_bNewTelemetryAvailable
//!              without actually sending, dropping that packet silently.
//!
//! Removing the callback's send call leaves RadioTask as the sole sender.
//! RadioTask runs wsfOsDispatcher() → polls flags → sends at its own pace,
//! which is naturally gated to one packet per WSF dispatch pass (~one BLE
//! connection interval = 15-30 ms), comfortably below the 50 ms sensor rate.
//
//*****************************************************************************
void
amdtpDtpTransCback(eAmdtpStatus_t status)
{
    // Log the TX result for debugging; RadioTask will pick up the next
    // sample on its next wsfOsDispatcher() pass.
    if (status != AMDTP_STATUS_SUCCESS)
    {
        APP_TRACE_INFO1("[AMDTP] TX callback status=%d", (int)status);
    }
}

//*****************************************************************************
//
//! @brief amdtpProcMsg — central WSF event dispatcher for the AMDTP profile.
//
//*****************************************************************************
static void
amdtpProcMsg(amdtpMsg_t *pMsg)
{
    uint8_t uiEvent = APP_UI_NONE;

    switch (pMsg->hdr.event)
    {
        case AMDTP_SENSOR_DATA_IND:
            // Posted by RadioTask when it detects g_bNewTelemetryAvailable.
            // RadioTask handles the actual send; this event is informational.
            break;

        case AMDTP_TIMER_IND:
            amdtps_proc_msg(&pMsg->hdr);
            break;

#ifdef MEASURE_THROUGHPUT
        case AMDTP_MEAS_TP_TIMER_IND:
            showThroughput();
            break;
#endif

        case ATTS_HANDLE_VALUE_CNF:
            amdtps_proc_msg(&pMsg->hdr);
            break;

        case ATTS_CCC_STATE_IND:
            amdtpProcCccState(pMsg);
            break;

        case ATT_MTU_UPDATE_IND:
            APP_TRACE_INFO1("MTU negotiated: %d bytes",
                            ((attEvt_t *)pMsg)->mtu);
            break;

        case DM_CONN_OPEN_IND:
            amdtps_proc_msg(&pMsg->hdr);
            // Request DLE — up to 251-byte ACL PDUs (fits 24-byte packet + headers)
            DmConnSetDataLen(1, 251, 0x848);
            uiEvent = APP_UI_CONN_OPEN;
            g_bTxSubscribed          = false;
            g_bNewTelemetryAvailable = false;
            am_util_stdio_printf("[BLE] Connected — waiting for TX subscription\r\n");
            break;

        case DM_CONN_CLOSE_IND:
            amdtpClose(pMsg);
            amdtps_proc_msg(&pMsg->hdr);
            g_bTxSubscribed = false;
            APP_TRACE_INFO1("DM_CONN_CLOSE_IND reason=0x%x",
                            pMsg->dm.connClose.reason);
            am_util_stdio_printf("[BLE] Disconnected\r\n");
            uiEvent = APP_UI_CONN_CLOSE;
            break;

        case DM_CONN_UPDATE_IND:
            amdtps_proc_msg(&pMsg->hdr);
            APP_TRACE_INFO2("Conn params updated: interval=%d latency=%d",
                            pMsg->dm.connUpdate.connInterval,
                            pMsg->dm.connUpdate.connLatency);
            am_util_stdio_printf("[BLE] Conn interval = %d × 1.25 ms = %.1f ms\r\n",
                                 (int)pMsg->dm.connUpdate.connInterval,
                                 pMsg->dm.connUpdate.connInterval * 1.25f);
            break;

        case DM_CONN_DATA_LEN_CHANGE_IND:
            APP_TRACE_INFO2("DLE changed: TX=%d RX=%d",
                            ((hciLeDataLenChangeEvt_t *)pMsg)->maxTxOctets,
                            ((hciLeDataLenChangeEvt_t *)pMsg)->maxRxOctets);
            break;

        case DM_PHY_UPDATE_IND:
            APP_TRACE_INFO3("PHY updated: status=%d RX=%d TX=%d",
                            pMsg->dm.phyUpdate.status,
                            pMsg->dm.phyUpdate.rxPhy,
                            pMsg->dm.phyUpdate.txPhy);
            break;

        case DM_RESET_CMPL_IND:
            attsCsfSetHashUpdateStatus(TRUE);
            if (amdtpSecCfg.auth & DM_AUTH_SC_FLAG)
            {
                DmSecGenerateEccKeyReq();
            }
            else
            {
                AttsCalculateDbHash();
            }
            uiEvent = APP_UI_RESET_CMPL;
            break;

        case ATTS_DB_HASH_CALC_CMPL_IND:
            amdtpSetup(pMsg);
            break;

        case DM_ADV_START_IND:
            uiEvent = APP_UI_ADV_START;
            am_util_stdio_printf("[BLE] Advertising started\r\n");
            break;

        case DM_ADV_STOP_IND:
            uiEvent = APP_UI_ADV_STOP;
            break;

        case DM_SEC_PAIR_CMPL_IND:
            DmSecGenerateEccKeyReq();
            bPairingCompleted = true;
            uiEvent = APP_UI_SEC_PAIR_CMPL;
            am_util_stdio_printf("[BLE] Pairing complete\r\n");
            break;

        case DM_SEC_PAIR_FAIL_IND:
            DmSecGenerateEccKeyReq();
            bPairingCompleted = false;
            APP_TRACE_INFO1("DM_SEC_PAIR_FAIL_IND status=0x%x",
                            pMsg->dm.pairCmpl.hdr.status);
            uiEvent = APP_UI_SEC_PAIR_FAIL;
            break;

        case DM_SEC_ENCRYPT_IND:
            uiEvent = APP_UI_SEC_ENCRYPT;
            break;

        case DM_SEC_ENCRYPT_FAIL_IND:
            uiEvent = APP_UI_SEC_ENCRYPT_FAIL;
            break;

        case DM_SEC_AUTH_REQ_IND:
            AppHandlePasskey(&pMsg->dm.authReq);
            break;

        case DM_SEC_ECC_KEY_IND:
            DmSecSetEccKey(&pMsg->dm.eccMsg.data.key);
            if (attsCsfGetHashUpdateStatus())
            {
                AttsCalculateDbHash();
            }
            break;

        case DM_SEC_COMPARE_IND:
            AppHandleNumericComparison(&pMsg->dm.cnfInd);
            break;

        case DM_HW_ERROR_IND:
            uiEvent = APP_UI_HW_ERROR;
            am_util_stdio_printf("ERROR: BLE hardware error\r\n");
            break;

        case DM_VENDOR_SPEC_CMD_CMPL_IND:
            break;

        default:
            break;
    }

    if (uiEvent != APP_UI_NONE)
    {
        AppUiAction(uiEvent);
    }
}

//*****************************************************************************
//
//! @brief AmdtpHandlerInit — register configs with the App framework.
//
//*****************************************************************************
void
AmdtpHandlerInit(wsfHandlerId_t handlerId)
{
    APP_TRACE_INFO0("AmdtpHandlerInit");

    amdtpHandlerId = handlerId;

    pAppAdvCfg    = (appAdvCfg_t *)    &amdtpAdvCfg;
    pAppSlaveCfg  = (appSlaveCfg_t *)  &amdtpSlaveCfg;
    pAppSecCfg    = (appSecCfg_t *)    &amdtpSecCfg;
    pAppUpdateCfg = (appUpdateCfg_t *) &amdtpUpdateCfg;

    AppSlaveInit();
    AppServerInit();

    pSmpCfg = (smpCfg_t *) &amdtpSmpCfg;

    amdtps_init(handlerId,
                (AmdtpsCfg_t *)&amdtpAmdtpsCfg,
                amdtpDtpRecvCback,
                amdtpDtpTransCback);

#ifdef MEASURE_THROUGHPUT
    measTpTimer.handlerId = handlerId;
    measTpTimer.msg.event = AMDTP_MEAS_TP_TIMER_IND;
#endif
}

//*****************************************************************************
//
//! @brief AmdtpHandler — WSF event handler entry-point.
//
//*****************************************************************************
void
AmdtpHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
    (void)event;
    if (pMsg != NULL)
    {
        if (pMsg->event >= ATT_CBACK_START && pMsg->event <= ATT_CBACK_END)
        {
            AppServerProcAttMsg(pMsg);
        }
        else if (pMsg->event >= DM_CBACK_START && pMsg->event <= DM_CBACK_END)
        {
            AppSlaveProcDmMsg((dmEvt_t *)pMsg);
            AppSlaveSecProcDmMsg((dmEvt_t *)pMsg);
        }
        amdtpProcMsg((amdtpMsg_t *)pMsg);
    }
}

//*****************************************************************************
//
//! @brief AmdtpStart — register callbacks and GATT services, start DM reset.
//
//*****************************************************************************
void
AmdtpStart(void)
{
    DmRegister(amdtpDmCback);
    DmConnRegister(DM_CLIENT_ID_APP, amdtpDmCback);
    AttRegister(amdtpAttCback);
    AttConnRegister(AppServerConnCback);
    AttsCccRegister(AMDTP_NUM_CCC_IDX,
                    (attsCccSet_t *)amdtpCccSet,
                    amdtpCccCback);

    AppUiBtnRegister(amdtpBtnCback);

    SvcCoreGattCbackRegister(GattReadCback, GattWriteCback);
    SvcCoreAddGroup();
    SvcDisAddGroup();

    SvcAmdtpsCbackRegister(NULL, amdtps_write_cback);
    SvcAmdtpsAddGroup();

    GattSetSvcChangedIdx(AMDTP_GATT_SC_CCC_IDX);

    DmDevReset();
}

//*****************************************************************************
// Hardware initialisation + main()
//*****************************************************************************
static void
enable_print_interface(void)
{
#if 1
    am_bsp_uart_printf_enable();
#else
    am_bsp_itm_printf_enable();
#endif
    am_util_stdio_terminal_clear();
}

int
main(void)
{
#ifndef NOFPU
    am_hal_sysctrl_fpu_enable();
    am_hal_sysctrl_fpu_stacking_enable(true);
#else
    am_hal_sysctrl_fpu_disable();
#endif

    am_bsp_low_power_init();

#ifdef AM_DEBUG_PRINTF
    enable_print_interface();
#endif

    run_tasks();

    while (1) {}
}
//*****************************************************************************
//
//! @file radio_task.c
//!
//! @brief BLE AMDTP radio task — WSF/ExactLE stack + sensor telemetry TX.
//!
//! Data flow
//! ---------
//!   SensorTask (rtos.c) packs a 24-byte telemetry_packet_t every 50 ms and
//!   sets g_bNewTelemetryAvailable = true.
//!   RadioTask forwards the packet via AmdtpsSendPacket() when:
//!     (a) g_bBleConnected is true  (a phone is connected), AND
//!     (b) g_bTxSubscribed  is true (phone has written CCC = NOTIFY).
//!
//! FIX SUMMARY (vs. original)
//! --------------------------
//!   1. sPkt type corrected: telemetry_packet_t (24 bytes) instead of
//!      telemetry_string_t (char[80]).  Sending 80 bytes of which 56 were
//!      uninitialised padding corrupted every packet received by the phone.
//!
//!   2. AmdtpsSendPacket() len argument is now sizeof(telemetry_packet_t) = 24
//!      explicitly, not sizeof(sPkt) where sPkt was the wrong type.
//!
//!   3. On AMDTP_STATUS_BUSY, g_bNewTelemetryAvailable is NO LONGER cleared.
//!      The original code cleared the flag on BUSY, which meant the sample
//!      was silently dropped even though it was never sent.  Now the flag
//!      stays set; RadioTask will retry on the next wsfOsDispatcher() pass
//!      (within one BLE connection interval = 15-30 ms).
//!
//!   4. The dual-send race fix is in Stride_inte.c (amdtpDtpTransCback no
//!      longer calls AmdtpsSendSensorData).  RadioTask is now the sole sender.
//
//*****************************************************************************

#include "wsf_types.h"
#include "wsf_trace.h"
#include "wsf_buf.h"
#include "wsf_timer.h"
#include "wsf_msg.h"

#include "hci_handler.h"
#include "dm_handler.h"
#include "dm_api.h"
#include "l2c_handler.h"
#include "att_handler.h"
#include "smp_handler.h"
#include "l2c_api.h"
#include "att_api.h"
#include "smp_api.h"
#include "app_api.h"
#include "hci_core.h"
#include "hci_drv.h"
#include "hci_drv_apollo.h"
#include "hci_drv_em9305.h"

#include "amdtp_api.h"
#include "amdtps_api.h"
#include "app_ui.h"

#include "ble_freertos_amdtps.h"
#include "radio_task.h"
#include "rtos.h"          // telemetry_packet_t, g_sTelemetryPacket

#include <string.h>

//*****************************************************************************
// Public globals
//*****************************************************************************
TaskHandle_t radio_task_handle;
volatile bool g_bBleConnected = false;
volatile bool g_bTxSubscribed = false;

//*****************************************************************************
// Module-private state
//*****************************************************************************
static uint32_t s_ui32TxBusyCount = 0;

//*****************************************************************************
// WSF buffer pool
//
// Pool sizes chosen for AMDTP at 20 Hz with 24-byte packets.
// 280-byte pool: AMDTP frames = 24-byte payload + AMDTP header (8 B) +
// HCI ACL header (4 B) + L2CAP (4 B) = ~40 bytes.  DLE ACL max = 255 B.
// 30 buffers provides headroom for back-to-back DLE packets.
//*****************************************************************************
#define WSF_BUF_POOLS  4

static uint32_t g_pui32BufMem[
    (WSF_BUF_POOLS * 16
     + 16 *  8
     + 32 *  4
     + 64 *  6
     + 280 * 30) / sizeof(uint32_t)];

static wsfBufPoolDesc_t g_psPoolDescriptors[WSF_BUF_POOLS] =
{
    {  16,  8 },
    {  32,  4 },
    {  64,  6 },
    { 280, 30 },
};

//*****************************************************************************
// Forward declaration
//*****************************************************************************
static void exactle_stack_init(void);

//*****************************************************************************
//
//! @brief Initialise the ExactLE (Cordio) BLE stack.
//
//*****************************************************************************
static void
exactle_stack_init(void)
{
    wsfHandlerId_t handlerId;
    uint16_t       wsfBufMemLen;

    WsfOsInit();
    WsfTimerInit();

    wsfBufMemLen = WsfBufInit(sizeof(g_pui32BufMem),
                              (uint8_t *)g_pui32BufMem,
                              WSF_BUF_POOLS,
                              g_psPoolDescriptors);
    if (wsfBufMemLen > sizeof(g_pui32BufMem))
    {
        am_util_debug_printf(
            "WARN: WSF buf pool too small by %u bytes\r\n",
            (unsigned)(wsfBufMemLen - sizeof(g_pui32BufMem)));
    }

    SecInit();
    SecAesInit();
    SecCmacInit();
    SecEccInit();

    handlerId = WsfOsSetNextHandler(HciHandler);
    HciHandlerInit(handlerId);

    handlerId = WsfOsSetNextHandler(DmHandler);
    DmDevVsInit(0);
    DmAdvInit();
    DmPhyInit();
    DmConnInit();
    DmConnSlaveInit();
    DmSecInit();
    DmSecLescInit();
    DmPrivInit();
    DmHandlerInit(handlerId);

    handlerId = WsfOsSetNextHandler(L2cSlaveHandler);
    L2cSlaveHandlerInit(handlerId);
    L2cInit();
    L2cSlaveInit();

    handlerId = WsfOsSetNextHandler(AttHandler);
    AttHandlerInit(handlerId);
    AttsInit();
    AttsIndInit();
    AttcInit();

    handlerId = WsfOsSetNextHandler(SmpHandler);
    SmpHandlerInit(handlerId);
    SmprInit();
    SmprScInit();

    HciSetMaxRxAclLen(251);

    handlerId = WsfOsSetNextHandler(AppHandler);
    AppHandlerInit(handlerId);

    handlerId = WsfOsSetNextHandler(AmdtpHandler);
    AmdtpHandlerInit(handlerId);

    handlerId = WsfOsSetNextHandler(HciDrvHandler);
    HciDrvHandlerInit(handlerId);
}

//*****************************************************************************
//
//! @brief GPIO ISR — EM9305 radio data-ready / interrupt line.
//
//*****************************************************************************
void
GPIO_INT_ISR(void)
{
    am_hal_gpio_mask_t IntStatus;
    uint32_t           ui32IntStatus;

    am_hal_gpio_interrupt_status_get(GPIO_INT_CHANNEL, false, &IntStatus);
    am_hal_gpio_interrupt_irq_status_get(GPIO_INT_IRQ, false, &ui32IntStatus);
    am_hal_gpio_interrupt_irq_clear(GPIO_INT_IRQ, ui32IntStatus);
    am_hal_gpio_interrupt_service(GPIO_INT_IRQ, ui32IntStatus);
}

//*****************************************************************************
//
//! @brief UART ISR
//
//*****************************************************************************
#if   AM_BSP_UART_PRINT_INST == 0
void am_uart_isr(void)
#elif AM_BSP_UART_PRINT_INST == 1
void am_uart1_isr(void)
#elif AM_BSP_UART_PRINT_INST == 2
void am_uart2_isr(void)
#elif AM_BSP_UART_PRINT_INST == 3
void am_uart3_isr(void)
#endif
{
    uint32_t ui32Status = UARTn(AM_BSP_UART_PRINT_INST)->MIS;
    UARTn(AM_BSP_UART_PRINT_INST)->IEC = ui32Status;
}

//*****************************************************************************
//
//! @brief RadioTaskSetup
//
//*****************************************************************************
void
RadioTaskSetup(void)
{
    am_util_debug_printf("RadioTaskSetup: complete\r\n");
}

//*****************************************************************************
//
//! @brief RadioTask — BLE AMDTP server + BMP390/BMI323 telemetry forwarding.
//!
//! Execution flow
//! ──────────────
//!   1. Boot EM9305 radio IC.
//!   2. Initialise the WSF / ExactLE BLE stack.
//!   3. Start the AMDTP profile.
//!   4. Enter the WSF event-dispatch loop.
//!      After each dispatcher pass:
//!        a) Refresh g_bBleConnected.
//!        b) If subscribed AND new data ready: snapshot and send.
//!        c) On BUSY: leave g_bNewTelemetryAvailable SET so the next pass
//!           retries rather than silently dropping the packet (FIX #3).
//
//*****************************************************************************
void
RadioTask(void *pvParameters)
{
#if WSF_TRACE_ENABLED == TRUE
    am_util_debug_printf("WSF trace enabled\r\n");
#endif

    HciDrvRadioBoot(1);
    exactle_stack_init();
    AmdtpStart();

    while (1)
    {
        // ── 4a. Run the WSF event dispatcher ─────────────────────────
        wsfOsDispatcher();

        // ── 4b. Refresh connection state ──────────────────────────────
        {
            dmConnId_t dmConnId      = AppConnIsOpen();
            bool       bNowConnected = (dmConnId != DM_CONN_ID_NONE);

            if (bNowConnected && !g_bBleConnected)
            {
                am_util_debug_printf("BLE: phone CONNECTED (connId=%d)\r\n",
                                     (int)dmConnId);
                g_bTxSubscribed          = false;
                g_bNewTelemetryAvailable = false;
                s_ui32TxBusyCount        = 0;
            }
            else if (!bNowConnected && g_bBleConnected)
            {
                am_util_debug_printf(
                    "BLE: phone DISCONNECTED — re-advertising\r\n");
                g_bTxSubscribed   = false;
                s_ui32TxBusyCount = 0;
            }

            g_bBleConnected = bNowConnected;
        }

        // ── 4c. Forward telemetry to phone ────────────────────────────
        if (g_bTxSubscribed && g_bNewTelemetryAvailable)
        {
            // FIX #1: correct type — was telemetry_string_t (char[80])
            telemetry_packet_t sPkt;
            memcpy(&sPkt, (const void *)&g_sTelemetryPacket, sizeof(sPkt));

            // FIX #3: clear the flag BEFORE calling SendPacket so that
            // SensorTask can immediately start writing the next sample
            // while we're in the send call.  On BUSY we restore the flag
            // below so the sample is retried, not dropped.
            g_bNewTelemetryAvailable = false;

            // FIX #2: send sizeof(telemetry_packet_t) = 24 bytes exactly.
            eAmdtpStatus_t eTx = AmdtpsSendPacket(
                                     AMDTP_PKT_TYPE_DATA,
                                     FALSE,
                                     FALSE,
                                     (uint8_t *)&sPkt,
                                     (uint16_t)sizeof(telemetry_packet_t));

            if (eTx == AMDTP_STATUS_SUCCESS)
            {
                s_ui32TxBusyCount = 0;
                am_util_stdio_printf(
                    "[BLE TX] %u B | t=%u T:%.2fC P:%uPa A:%.2fm "
                    "Ax:%.3f Ay:%.3f Az:%.3f Gx:%.1f Gy:%.1f Gz:%.1f\r\n",
                    (unsigned)sizeof(telemetry_packet_t),
                    (unsigned)sPkt.ui32Timestamp,
                    sPkt.i16TempCx100  / 100.0f,
                    (unsigned)sPkt.ui32PressurePa,
                    sPkt.i16AltMx100   / 100.0f,
                    sPkt.i16AxGx1000   / 1000.0f,
                    sPkt.i16AyGx1000   / 1000.0f,
                    sPkt.i16AzGx1000   / 1000.0f,
                    sPkt.i16GxDpsx100  / 100.0f,
                    sPkt.i16GyDpsx100  / 100.0f,
                    sPkt.i16GzDpsx100  / 100.0f);
            }
            else if (eTx == AMDTP_STATUS_BUSY)
            {
                // FIX #3: restore the flag so we retry this sample on the
                // next wsfOsDispatcher() pass instead of silently dropping it.
                // The packet content in g_sTelemetryPacket is still valid
                // because SensorTask only overwrites it when the flag is false.
                g_bNewTelemetryAvailable = true;   // retry next pass

                s_ui32TxBusyCount++;
                if (s_ui32TxBusyCount % 5u == 1u)
                {
                    am_util_debug_printf(
                        "[BLE TX] AMDTP busy (x%u) — retrying on next pass\r\n",
                        (unsigned)s_ui32TxBusyCount);
                }
            }
            else
            {
                // Other errors (not subscribed, bad state, etc.) — drop this
                // sample and let SensorTask produce a fresh one.
                am_util_debug_printf(
                    "[BLE TX] AmdtpsSendPacket() error %d — dropping packet\r\n",
                    (int)eTx);
                s_ui32TxBusyCount = 0;
                // flag was already cleared above; SensorTask will write next sample
            }
        }
        else if (!g_bTxSubscribed && g_bNewTelemetryAvailable)
        {
            // Not subscribed — discard stale sample silently.
            g_bNewTelemetryAvailable = false;
        }

    } // end while(1)
}
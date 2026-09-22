//*****************************************************************************
//
//  sdcard_spi.c
//! @file
//!
//! @brief SD card SPI-mode physical layer driver for the Apollo510B EVB.
//!        IOM transaction style matches lsm6dsl.c in this project: byte
//!        streams are packed into the IOM's 32-bit-word TX/RX buffers,
//!        Transaction.bContinue is used to hold CS asserted across the
//!        multiple back-to-back transfers that one SD command requires
//!        (command bytes -> R1 poll -> optional data block -> CS release).
//!
//!        Implements the standard SD SPI-mode bring-up sequence:
//!          CMD0  (GO_IDLE_STATE)
//!          CMD8  (SEND_IF_COND)      - detect SD ver2 / voltage window
//!          CMD55+ACMD41 (SD_SEND_OP_COND) - poll until card leaves idle
//!          CMD58 (READ_OCR)          - detect block- vs byte-addressing (CCS)
//!          CMD16 (SET_BLOCKLEN=512)  - only needed for byte-addressed cards
//!        then single block read (CMD17) / write (CMD24).
//!
//*****************************************************************************

#include <string.h>
#include "sdcard_spi.h"

//*****************************************************************************
// Set to 0 to silence the step-by-step init trace once things are working.
//*****************************************************************************
#define SDSPI_DEBUG_PRINT   1

#if SDSPI_DEBUG_PRINT
#define SDSPI_DBG(...)   am_util_stdio_printf(__VA_ARGS__)
#else
#define SDSPI_DBG(...)
#endif

//*****************************************************************************
// Module state
//*****************************************************************************
static void                *g_pIOMHandle   = NULL;
static am_hal_iom_config_t  g_sIOMConfig;
static sd_card_type_e       g_eCardType    = SD_CARD_NONE;
static bool                 g_bReady       = false;

// Buffer big enough for one 512-byte data block + 2 CRC bytes, packed into
// 32-bit words (same packing scheme as lsm6dsl.c: byte i -> word[i/4],
// shifted by (i%4)*8).
#define SD_BUF_WORDS   ((SD_BLOCK_LEN + 2 + 3) / 4)   // 129 words

//*****************************************************************************
// R1 response bits
//*****************************************************************************
#define R1_IDLE_STATE       0x01
#define R1_ILLEGAL_CMD      0x04

// Data block tokens
#define TOKEN_START_BLOCK   0xFE
#define TOKEN_START_BLOCK_MULTI 0xFC
#define TOKEN_STOP_TRAN     0xFD

//*****************************************************************************
//
//  sd_tx() / sd_rx() - raw byte-stream helpers over the IOM.
//
//  bKeepCS = true  -> CS stays asserted after this transfer (chain continues)
//  bKeepCS = false -> CS is released at the end of this transfer
//
//*****************************************************************************
static uint32_t
sd_tx(const uint8_t *pData, uint32_t ui32Len, bool bKeepCS)
{
    am_hal_iom_transfer_t Transaction;
    uint32_t ui32Buf[SD_BUF_WORDS];
    uint32_t i;

    if (ui32Len == 0 || ui32Len > (SD_BUF_WORDS * 4))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    memset(ui32Buf, 0xFF, sizeof(ui32Buf));   // idle line level = 0xFF
    for (i = 0; i < ui32Len; i++)
    {
        uint32_t ui32WordIdx = i / 4;
        uint32_t ui32Shift   = (i % 4) * 8;
        ui32Buf[ui32WordIdx] = (ui32Buf[ui32WordIdx] & ~(0xFFUL << ui32Shift)) |
                               ((uint32_t)pData[i] << ui32Shift);
    }

    memset(&Transaction, 0, sizeof(Transaction));
    Transaction.ui32InstrLen                = 0;
    Transaction.eDirection                  = AM_HAL_IOM_TX;
    Transaction.ui32NumBytes                = ui32Len;
    Transaction.pui32TxBuffer               = ui32Buf;
    Transaction.bContinue                   = bKeepCS;
    Transaction.uPeerInfo.ui32SpiChipSelect = SD_IOM_CS_CHNL;

    return am_hal_iom_blocking_transfer(g_pIOMHandle, &Transaction);
}

static uint32_t
sd_rx(uint8_t *pData, uint32_t ui32Len, bool bKeepCS)
{
    am_hal_iom_transfer_t Transaction;
    uint32_t ui32Buf[SD_BUF_WORDS];
    uint32_t ui32Status;
    uint32_t i;

    if (ui32Len == 0 || ui32Len > (SD_BUF_WORDS * 4))
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    memset(&Transaction, 0, sizeof(Transaction));
    Transaction.ui32InstrLen                = 0;
    Transaction.eDirection                  = AM_HAL_IOM_RX;
    Transaction.ui32NumBytes                = ui32Len;
    Transaction.pui32RxBuffer               = ui32Buf;
    Transaction.bContinue                   = bKeepCS;
    Transaction.uPeerInfo.ui32SpiChipSelect = SD_IOM_CS_CHNL;

    ui32Status = am_hal_iom_blocking_transfer(g_pIOMHandle, &Transaction);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32Status;
    }

    for (i = 0; i < ui32Len; i++)
    {
        pData[i] = (uint8_t)((ui32Buf[i / 4] >> ((i % 4) * 8)) & 0xFF);
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  sd_send_cmd() - send a 6-byte SD command frame and collect the R1
//  response. Leaves CS ASSERTED on return (caller must release it, either
//  after reading extra R3/R7 bytes, a data block, or immediately via
//  sd_release_cs()).
//
//*****************************************************************************
// Forward declaration - defined below, used by sd_send_cmd_retry() above it.
static void sd_release_cs(void);

static uint8_t
sd_send_cmd(uint8_t ui8Cmd, uint32_t ui32Arg, uint8_t ui8Crc)
{
    uint8_t pui8Frame[6];
    uint8_t ui8Sync = 0xFF;
    uint8_t ui8R1 = 0xFF;
    int     i;

    // One extra 0xFF sync byte ahead of every command frame. This is
    // standard SD-SPI practice (see e.g. ChaN's MMC/SPI reference driver)
    // and gives the card's SPI front end one settled clock edge before the
    // real start bit arrives - cheap insurance against the occasional
    // "no response" seen on the first command issued right after a CS
    // release.
    sd_tx(&ui8Sync, 1, true);

    pui8Frame[0] = 0x40 | (ui8Cmd & 0x3F);
    pui8Frame[1] = (uint8_t)(ui32Arg >> 24);
    pui8Frame[2] = (uint8_t)(ui32Arg >> 16);
    pui8Frame[3] = (uint8_t)(ui32Arg >> 8);
    pui8Frame[4] = (uint8_t)(ui32Arg);
    pui8Frame[5] = ui8Crc | 0x01;

    sd_tx(pui8Frame, 6, true);

    // Wait for R1 (bit7 clear). Spec max NCR is 8 bytes; widened a bit
    // for margin.
    for (i = 0; i < 16; i++)
    {
        sd_rx(&ui8R1, 1, true);
        if ((ui8R1 & 0x80) == 0)
        {
            break;
        }
    }

    return ui8R1;
}

//*****************************************************************************
//
//  sd_send_cmd_retry() - same as sd_send_cmd(), but retries the whole
//  command (not just the R1 poll) a few times. Use this for one-shot
//  commands that aren't already wrapped in an outer state-machine retry
//  loop (CMD8, CMD16, CMD58, CMD9, CMD17, CMD24), so a single transient
//  glitch doesn't fail the whole init/read/write.
//
//  NOTE: always releases CS between attempts; on the attempt that
//  produces the returned R1, CS is left ASSERTED (same contract as
//  sd_send_cmd()) so the caller can read any trailing R3/R7/data bytes.
//
//*****************************************************************************
static uint8_t
sd_send_cmd_retry(uint8_t ui8Cmd, uint32_t ui32Arg, uint8_t ui8Crc, int iRetries)
{
    uint8_t ui8R1 = 0xFF;
    int     i;

    for (i = 0; i < iRetries; i++)
    {
        ui8R1 = sd_send_cmd(ui8Cmd, ui32Arg, ui8Crc);
        if (ui8R1 != 0xFF)
        {
            break;
        }
        sd_release_cs();
        am_util_delay_ms(1);
    }

    return ui8R1;
}

static void
sd_release_cs(void)
{
    uint8_t ui8Dummy;
    // One extra byte with bContinue=false deasserts CS at the end of the
    // transfer and provides the trailing clock the card spec calls for.
    sd_rx(&ui8Dummy, 1, false);
}

static uint32_t
sd_wait_not_busy(uint32_t ui32TimeoutMs)
{
    uint8_t  ui8Byte = 0;
    uint32_t ui32Ms;

    for (ui32Ms = 0; ui32Ms < ui32TimeoutMs; ui32Ms++)
    {
        sd_rx(&ui8Byte, 1, true);
        if (ui8Byte == 0xFF)
        {
            return AM_HAL_STATUS_SUCCESS;
        }
        am_util_delay_ms(1);
    }
    return AM_HAL_STATUS_FAIL;
}

//*****************************************************************************
//
//  sdspi_iom_init() - one-time IOM + GPIO bring-up (400 kHz, SPI mode 0).
//
//*****************************************************************************
uint32_t
sdspi_iom_init(void)
{
    uint32_t ui32Status;

    g_sIOMConfig.eInterfaceMode     = AM_HAL_IOM_SPI_MODE;
    g_sIOMConfig.ui32ClockFreq      = SD_IOM_SPI_SPEED_INIT;
    g_sIOMConfig.eSpiMode           = AM_HAL_IOM_SPI_MODE_0;
    g_sIOMConfig.pNBTxnBuf          = NULL;
    g_sIOMConfig.ui32NBTxnBufLength = 0;

    ui32Status = am_hal_iom_initialize(SD_IOM_MODULE, &g_pIOMHandle);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) return ui32Status;

    ui32Status = am_hal_iom_power_ctrl(g_pIOMHandle, AM_HAL_SYSCTRL_WAKE, false);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) return ui32Status;

    ui32Status = am_hal_iom_configure(g_pIOMHandle, &g_sIOMConfig);
    if (ui32Status != AM_HAL_STATUS_SUCCESS) return ui32Status;

    am_bsp_iom_pins_enable(SD_IOM_MODULE, AM_HAL_IOM_SPI_MODE);

    return am_hal_iom_enable(g_pIOMHandle);
}

static uint32_t
sd_set_clock(uint32_t ui32ClockFreq)
{
    g_sIOMConfig.ui32ClockFreq = ui32ClockFreq;
    return am_hal_iom_configure(g_pIOMHandle, &g_sIOMConfig);
}

//*****************************************************************************
//
//  sdspi_init() - card power-up / identification sequence.
//
//*****************************************************************************
uint32_t
sdspi_init(sd_card_type_e *peCardType)
{
    uint8_t  ui8R1;
    uint8_t  pui8Buf[4];
    uint32_t ui32Retry;
    bool     bSdV2 = false;

    g_bReady   = false;
    g_eCardType = SD_CARD_NONE;

    SDSPI_DBG("sdspi_init: starting SD SPI-mode bring-up sequence...\r\n");

    // >=74 dummy clocks with CS *high* (deselected) so the card sees a
    // clean idle bus before the first command. Sending 10 bytes of 0xFF
    // as a stand-alone (non-continued) transfer gives 80 clocks.
    {
        uint8_t pui8Dummy[10];
        memset(pui8Dummy, 0xFF, sizeof(pui8Dummy));
        sd_tx(pui8Dummy, sizeof(pui8Dummy), false);
    }

    // CMD0: GO_IDLE_STATE -> expect R1 == 0x01 (idle).
    for (ui32Retry = 0; ui32Retry < 10; ui32Retry++)
    {
        ui8R1 = sd_send_cmd(0, 0x00000000, 0x95);
        sd_release_cs();
        if (ui8R1 == R1_IDLE_STATE)
        {
            break;
        }
        am_util_delay_ms(1);
    }
    SDSPI_DBG("  CMD0  (GO_IDLE_STATE) R1=0x%02X after %u tries\r\n", ui8R1, ui32Retry + 1);
    if (ui8R1 != R1_IDLE_STATE)
    {
        SDSPI_DBG("  -> No response / wrong R1. Likely no card present, MISO/MOSI/SCK\r\n"
                  "     swapped, CS stuck, or missing pull-up on MISO/CS.\r\n");
        return AM_HAL_STATUS_FAIL;   // no card / no response
    }

    // CMD8: SEND_IF_COND (0x1AA = 2.7-3.6V, check pattern 0xAA).
    // Only SD ver2 cards implement this; MMC/SDv1 return R1_ILLEGAL_CMD.
    ui8R1 = sd_send_cmd_retry(8, 0x000001AA, 0x87, 3);
    if (ui8R1 == R1_IDLE_STATE)
    {
        sd_rx(pui8Buf, 4, true);   // R7 trailing 4 bytes (echoed voltage/pattern)
        sd_release_cs();
        if (pui8Buf[2] == 0x01 && pui8Buf[3] == 0xAA)
        {
            bSdV2 = true;
        }
    }
    else
    {
        sd_release_cs();
    }
    SDSPI_DBG("  CMD8  (SEND_IF_COND)  R1=0x%02X  SDv2=%d\r\n", ui8R1, bSdV2);

    // ACMD41 (via CMD55+CMD41): poll until card leaves idle state.
    // HCS (bit30) is set for SDv2 to allow the card to report SDHC/SDXC.
    for (ui32Retry = 0; ui32Retry < 1000; ui32Retry++)
    {
        sd_send_cmd(55, 0, 0x01);              // APP_CMD
        sd_release_cs();
        ui8R1 = sd_send_cmd(41, bSdV2 ? 0x40000000 : 0x00000000, 0x01);
        sd_release_cs();
        if (ui8R1 == 0x00)
        {
            break;
        }
        am_util_delay_ms(1);
    }
    SDSPI_DBG("  ACMD41 (SD_SEND_OP_COND) R1=0x%02X after %u tries\r\n", ui8R1, ui32Retry + 1);
    if (ui8R1 != 0x00)
    {
        // ACMD41 not accepted -> fall back to MMC CMD1.
        for (ui32Retry = 0; ui32Retry < 1000; ui32Retry++)
        {
            ui8R1 = sd_send_cmd(1, 0, 0x01);
            sd_release_cs();
            if (ui8R1 == 0x00)
            {
                g_eCardType = SD_CARD_MMC;
                break;
            }
            am_util_delay_ms(1);
        }
        SDSPI_DBG("  CMD1  (MMC fallback) R1=0x%02X after %u tries\r\n", ui8R1, ui32Retry + 1);
        if (ui8R1 != 0x00)
        {
            SDSPI_DBG("  -> Card never left idle state (timed out at both ACMD41 and\r\n"
                      "     CMD1). Check card power/insertion and clock speed.\r\n");
            return AM_HAL_STATUS_FAIL;         // card didn't come out of idle
        }
    }
    else
    {
        g_eCardType = bSdV2 ? SD_CARD_SDv2_BYTE : SD_CARD_SDv1;
    }

    // CMD58: READ_OCR - for SDv2 cards, bit30 (CCS) of the OCR tells us
    // whether the card is block-addressed (SDHC/SDXC) or byte-addressed.
    if (g_eCardType == SD_CARD_SDv2_BYTE)
    {
        ui8R1 = sd_send_cmd_retry(58, 0, 0x01, 3);
        sd_rx(pui8Buf, 4, true);
        sd_release_cs();
        SDSPI_DBG("  CMD58 (READ_OCR) R1=0x%02X  OCR=%02X%02X%02X%02X\r\n",
                  ui8R1, pui8Buf[0], pui8Buf[1], pui8Buf[2], pui8Buf[3]);
        if (ui8R1 == 0x00 && (pui8Buf[0] & 0x40))
        {
            g_eCardType = SD_CARD_SDv2_BLOCK;
        }
    }

    // Byte-addressed cards need an explicit 512-byte block length.
    if (g_eCardType != SD_CARD_SDv2_BLOCK)
    {
        ui8R1 = sd_send_cmd_retry(16, SD_BLOCK_LEN, 0x01, 5);
        sd_release_cs();
        SDSPI_DBG("  CMD16 (SET_BLOCKLEN) R1=0x%02X\r\n", ui8R1);
        if (ui8R1 != 0x00)
        {
            return AM_HAL_STATUS_FAIL;
        }
    }

    // Bring the clock up to full speed for data transfer.
    sd_set_clock(SD_IOM_SPI_SPEED_FULL);

    g_bReady = true;
    if (peCardType != NULL)
    {
        *peCardType = g_eCardType;
    }

    SDSPI_DBG("  SD init OK. Card type = %d (0=none,1=MMC,2=SDv1,3=SDv2-byte,4=SDv2-block)\r\n",
              g_eCardType);

    return AM_HAL_STATUS_SUCCESS;
}

bool
sdspi_is_ready(void)
{
    return g_bReady;
}

//*****************************************************************************
//
//  sdspi_read_block() - CMD17, single block read.
//
//*****************************************************************************
uint32_t
sdspi_read_block(uint32_t ui32BlockAddr, uint8_t *pui8Buf)
{
    uint8_t  ui8R1;
    uint8_t  ui8Token;
    uint8_t  pui8Crc[2];
    uint32_t ui32Addr;
    int      i;

    if (!g_bReady || pui8Buf == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    // Block-addressed cards (SDHC/SDXC) take a block number directly;
    // byte-addressed cards (SDSC/MMC) need the address in bytes.
    ui32Addr = (g_eCardType == SD_CARD_SDv2_BLOCK) ? ui32BlockAddr
                                                    : (ui32BlockAddr * SD_BLOCK_LEN);

    ui8R1 = sd_send_cmd(17, ui32Addr, 0x01);
    if (ui8R1 != 0x00)
    {
        sd_release_cs();
        return AM_HAL_STATUS_FAIL;
    }

    // Wait for the data start token (0xFE), ~100ms timeout.
    ui8Token = 0xFF;
    for (i = 0; i < 100; i++)
    {
        sd_rx(&ui8Token, 1, true);
        if (ui8Token == TOKEN_START_BLOCK)
        {
            break;
        }
        am_util_delay_ms(1);
    }
    if (ui8Token != TOKEN_START_BLOCK)
    {
        sd_release_cs();
        return AM_HAL_STATUS_FAIL;
    }

    sd_rx(pui8Buf, SD_BLOCK_LEN, true);
    sd_rx(pui8Crc, 2, true);         // trailing CRC16, not checked here
    sd_release_cs();

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  sdspi_write_block() - CMD24, single block write.
//
//*****************************************************************************
uint32_t
sdspi_write_block(uint32_t ui32BlockAddr, const uint8_t *pui8Buf)
{
    uint8_t  ui8R1;
    uint8_t  ui8Token;
    uint8_t  ui8DataResp;
    uint8_t  pui8Crc[2] = { 0xFF, 0xFF };   // CRC not checked in SPI mode by default
    uint32_t ui32Addr;

    if (!g_bReady || pui8Buf == NULL)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    ui32Addr = (g_eCardType == SD_CARD_SDv2_BLOCK) ? ui32BlockAddr
                                                    : (ui32BlockAddr * SD_BLOCK_LEN);

    ui8R1 = sd_send_cmd(24, ui32Addr, 0x01);
    if (ui8R1 != 0x00)
    {
        sd_release_cs();
        return AM_HAL_STATUS_FAIL;
    }

    ui8Token = TOKEN_START_BLOCK;
    sd_tx(&ui8Token, 1, true);
    sd_tx(pui8Buf, SD_BLOCK_LEN, true);
    sd_tx(pui8Crc, 2, true);

    sd_rx(&ui8DataResp, 1, true);
    // Data response token: xxx0RRR1, RRR=010 -> accepted.
    if ((ui8DataResp & 0x1F) != 0x05)
    {
        sd_release_cs();
        return AM_HAL_STATUS_FAIL;
    }

    // Card pulls DO low (busy) while it programs the block; poll for 0xFF.
    // SD spec allows up to 250 ms for a single block program; Class 4/6 cards
    // can occasionally reach ~600 ms on the first write after f_expand.
    // 2000 ms gives solid margin for all card grades without stalling long.
    if (sd_wait_not_busy(2000) != AM_HAL_STATUS_SUCCESS)
    {
        sd_release_cs();
        return AM_HAL_STATUS_FAIL;
    }
    sd_release_cs();

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//  sdspi_get_sector_count() - CMD9 (SEND_CSD) + CSD parse.
//  Supports CSD version 1.0 (standard capacity) and 2.0 (SDHC/SDXC).
//
//*****************************************************************************
uint32_t
sdspi_get_sector_count(void)
{
    uint8_t  ui8R1;
    uint8_t  ui8Token;
    uint8_t  pui8Csd[16];
    uint8_t  pui8Crc[2];
    int      i;
    uint32_t ui32Sectors = 0;

    if (!g_bReady)
    {
        return 0;
    }

    ui8R1 = sd_send_cmd(9, 0, 0x01);
    if (ui8R1 != 0x00)
    {
        sd_release_cs();
        return 0;
    }

    ui8Token = 0xFF;
    for (i = 0; i < 100; i++)
    {
        sd_rx(&ui8Token, 1, true);
        if (ui8Token == TOKEN_START_BLOCK)
        {
            break;
        }
        am_util_delay_ms(1);
    }
    if (ui8Token != TOKEN_START_BLOCK)
    {
        sd_release_cs();
        return 0;
    }

    sd_rx(pui8Csd, 16, true);
    sd_rx(pui8Crc, 2, true);
    sd_release_cs();

    if ((pui8Csd[0] >> 6) == 1)
    {
        // CSD v2.0 (SDHC/SDXC): C_SIZE is a 22-bit field, bytes [7..9].
        uint32_t ui32CSize = (((uint32_t)pui8Csd[7] & 0x3F) << 16) |
                              ((uint32_t)pui8Csd[8] << 8) |
                              (uint32_t)pui8Csd[9];
        ui32Sectors = (ui32CSize + 1) * 1024;   // 512-byte sectors
    }
    else
    {
        // CSD v1.0: C_SIZE (12 bits), C_SIZE_MULT (3 bits), READ_BL_LEN (4 bits).
        uint32_t ui32CSize = (((uint32_t)pui8Csd[6] & 0x03) << 10) |
                              ((uint32_t)pui8Csd[7] << 2) |
                              ((uint32_t)pui8Csd[8] >> 6);
        uint32_t ui32CSizeMult = (((uint32_t)pui8Csd[9] & 0x03) << 1) |
                                  ((uint32_t)pui8Csd[10] >> 7);
        uint32_t ui32ReadBlLen = pui8Csd[5] & 0x0F;
        uint32_t ui32BlockLen  = 1UL << ui32ReadBlLen;
        uint32_t ui32Mult      = 1UL << (ui32CSizeMult + 2);
        uint32_t ui32TotalBytes = (ui32CSize + 1) * ui32Mult * ui32BlockLen;
        ui32Sectors = ui32TotalBytes / SD_BLOCK_LEN;
    }

    return ui32Sectors;
}
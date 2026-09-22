//*****************************************************************************
//
//  sdcard_spi.h
//! @file
//!
//! @brief Low-level SD card driver, SPI mode, for the Apollo510B EVB.
//!        Target: AmbiqSuite SDK 5.1.0, IOM SPI (same transaction style as
//!        SD-card-only SPI transactions - am_hal_iom_blocking_transfer,
//!        bContinue used to hold CS low across a multi-phase command).
//!
//! Wiring (IOM0 - LSM6DSL uses IOM1 I2C, no pin conflict with IOM0 SPI):
//!   SCK  -> AM_BSP_GPIO_IOM0_SCK
//!   MOSI -> AM_BSP_GPIO_IOM0_MOSI  -> SD DI
//!   MISO -> AM_BSP_GPIO_IOM0_MISO  -> SD DO
//!   CS   -> AM_BSP_GPIO_IOM0_CS (hardware CS0)  -> SD CS / DAT3
//!   VDD  -> 3.3 V
//!   GND  -> GND
//!
//! Check am_bsp_pins.c for the exact physical pin numbers on your board.
//!
//! This file only implements the SD *physical layer* (card init, single
//! block read/write) that a FatFs diskio.c glue layer needs. It does not
//! know anything about FAT, files, or directories - that is FatFs's job.
//!
//*****************************************************************************

#ifndef SDCARD_SPI_H
#define SDCARD_SPI_H

#include <stdint.h>
#include <stdbool.h>
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#ifdef __cplusplus
extern "C" {
#endif

//*****************************************************************************
// Configuration - change these if the card is wired to a different IOM/CS.
//*****************************************************************************
#define SD_IOM_MODULE          0
#define SD_IOM_CS_CHNL         AM_BSP_IOM0_CS_CHNL

// Card init must happen at <=400 kHz per the SD spec. Bump to full speed
// (default 8 MHz, safe for most breakout boards/cables) after init.
#define SD_IOM_SPI_SPEED_INIT   AM_HAL_IOM_400KHZ
#define SD_IOM_SPI_SPEED_FULL   AM_HAL_IOM_8MHZ

#define SD_BLOCK_LEN            512U

//*****************************************************************************
// Card type flags, returned by sdspi_init() via *peCardType.
//*****************************************************************************
typedef enum
{
    SD_CARD_NONE = 0,
    SD_CARD_MMC,          // MMCv3 (byte addressed)
    SD_CARD_SDv1,         // SD ver1 (byte addressed)
    SD_CARD_SDv2_BYTE,    // SD ver2, standard capacity (byte addressed)
    SD_CARD_SDv2_BLOCK,   // SD ver2, high/extended capacity (block addressed, SDHC/SDXC)
} sd_card_type_e;

//*****************************************************************************
// API
//*****************************************************************************

//
// One-time IOM + GPIO bring-up. Call once at startup before sdspi_init().
//
uint32_t sdspi_iom_init(void);

//
// SD card power-up / identification sequence (CMD0/CMD8/ACMD41/CMD58/CMD16).
// Leaves the IOM clocked at SD_IOM_SPI_SPEED_FULL on success.
//
uint32_t sdspi_init(sd_card_type_e *peCardType);

//
// Read one 512-byte block. ui32BlockAddr is a BLOCK number (0,1,2,...),
// not a byte offset - sdspi_read_block() converts to a byte address
// internally for byte-addressed (SDv1/MMC) cards.
//
uint32_t sdspi_read_block(uint32_t ui32BlockAddr, uint8_t *pui8Buf);

//
// Write one 512-byte block.
//
uint32_t sdspi_write_block(uint32_t ui32BlockAddr, const uint8_t *pui8Buf);

//
// CMD9 (SEND_CSD) + parse -> total capacity in 512-byte sectors.
// Returns 0 on failure.
//
uint32_t sdspi_get_sector_count(void);

//
// True once sdspi_init() has completed successfully.
//
bool sdspi_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif // SDCARD_SPI_H
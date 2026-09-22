/*-----------------------------------------------------------------------*/
/* Low level disk I/O module - SD card, SPI mode, Apollo510B             */
/* Glue layer between FatFs and sdcard_spi.c                             */
/*-----------------------------------------------------------------------*/

#include "ff.h"			/* Obtains integer types */
#include "diskio.h"		/* Declarations of disk functions */
#include "sdcard_spi.h"	/* SD-over-SPI physical layer */
#include "am_util.h"	/* am_util_stdio_printf() for init diagnostics */

/* Definitions of physical drive number for each drive */
#define DEV_SD		0	/* Map the SD card to physical drive 0 */

static sd_card_type_e g_eCardType = SD_CARD_NONE;

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	if (pdrv != DEV_SD)
	{
		return STA_NOINIT;
	}
	return sdspi_is_ready() ? 0 : STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	uint32_t ui32Status;

	if (pdrv != DEV_SD)
	{
		return STA_NOINIT;
	}

	/* IOM/GPIO bring-up only needs to happen once; harmless to call again. */
	ui32Status = sdspi_iom_init();
	if (ui32Status != AM_HAL_STATUS_SUCCESS)
	{
		am_util_stdio_printf("disk_initialize: sdspi_iom_init() failed, status=0x%08X\r\n",
		                     ui32Status);
		return STA_NOINIT;
	}

	ui32Status = sdspi_init(&g_eCardType);
	if (ui32Status != AM_HAL_STATUS_SUCCESS)
	{
		am_util_stdio_printf("disk_initialize: sdspi_init() failed, status=0x%08X\r\n",
		                     ui32Status);
		return STA_NOINIT;
	}

	return 0;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	LBA_t sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	if (pdrv != DEV_SD || !sdspi_is_ready())
	{
		return RES_NOTRDY;
	}

	for (UINT i = 0; i < count; i++)
	{
		if (sdspi_read_block((uint32_t)sector + i, buff + (i * 512)) != AM_HAL_STATUS_SUCCESS)
		{
			return RES_ERROR;
		}
	}

	return RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	LBA_t sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
	if (pdrv != DEV_SD || !sdspi_is_ready())
	{
		return RES_NOTRDY;
	}

	for (UINT i = 0; i < count; i++)
	{
		if (sdspi_write_block((uint32_t)sector + i, buff + (i * 512)) != AM_HAL_STATUS_SUCCESS)
		{
			return RES_ERROR;
		}
	}

	return RES_OK;
}

#endif


/*-----------------------------------------------------------------------*/
/* Control Functions                                                     */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	if (pdrv != DEV_SD)
	{
		return RES_PARERR;
	}

	switch (cmd)
	{
		case CTRL_SYNC:
			/* Single-block blocking transfers -> nothing pending to flush. */
			return RES_OK;

		case GET_SECTOR_COUNT:
		{
			uint32_t ui32Count = sdspi_get_sector_count();
			if (ui32Count == 0)
			{
				return RES_ERROR;
			}
			*(LBA_t *)buff = ui32Count;
			return RES_OK;
		}

		case GET_SECTOR_SIZE:
			*(WORD *)buff = SD_BLOCK_LEN;
			return RES_OK;

		case GET_BLOCK_SIZE:
			/* Erase block size in sectors - 1 is a safe generic default. */
			*(DWORD *)buff = 1;
			return RES_OK;

		default:
			return RES_PARERR;
	}
}

/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/
DWORD get_fattime (void)
{
  return 0;
}
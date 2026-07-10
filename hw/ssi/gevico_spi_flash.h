/*
 * G233 SPI NOR Flash emulation (W25X16 / W25X32)
 *
 * Copyright (c) 2025 Chao Liu <chao.liu@yeah.net>
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Emulates Winbond SPI NOR Flash devices for the G233 SPI controller.
 * Supports: JEDEC ID, RDSR, WREN/WRDI, SE, PP, READ.
 */

#ifndef HW_SSI_GEVICO_SPI_FLASH_H
#define HW_SSI_GEVICO_SPI_FLASH_H

#include "qemu/osdep.h"

#define FLASH_SR_BUSY   0x01
#define FLASH_SR_WEL    0x02

#define FLASH_PAGE_SIZE   256
#define FLASH_SECTOR_SIZE 4096

/* Flash commands */
#define FLASH_CMD_JEDEC_ID      0x9F
#define FLASH_CMD_READ_STATUS   0x05
#define FLASH_CMD_WRITE_ENABLE  0x06
#define FLASH_CMD_WRITE_DISABLE 0x04
#define FLASH_CMD_SECTOR_ERASE  0x20
#define FLASH_CMD_PAGE_PROGRAM  0x02
#define FLASH_CMD_READ_DATA     0x03

typedef enum {
    FLASH_IDLE,
    FLASH_JEDEC_ID,
    FLASH_RDSR,
    FLASH_SE_ADDR,       /* receiving 3 address bytes for sector erase */
    FLASH_PP_ADDR,       /* receiving 3 address bytes for page program */
    FLASH_PP_DATA,       /* receiving page program data bytes */
    FLASH_READ_ADDR,     /* receiving 3 address bytes for read */
    FLASH_READ_DATA,     /* sending read data bytes */
} FlashCmdState;

typedef struct GEVICOSPIFlash {
    uint8_t *storage;       /* flash data array */
    uint32_t size;          /* total size in bytes */
    uint32_t jedec_id;      /* 24-bit JEDEC ID */

    FlashCmdState state;    /* current command state */
    uint8_t  cmd;           /* current command byte */
    uint32_t addr;          /* address being built/referenced */
    uint8_t  addr_shift;    /* how many address bytes remaining */
    uint32_t data_count;    /* how many data bytes to transfer */
    uint32_t data_pos;      /* position within flash for read/program */
    uint8_t  status;        /* status register (BUSY, WEL) */
    bool     write_enable;  /* WEL latch */
} GEVICOSPIFlash;

void gevico_spi_flash_init(GEVICOSPIFlash *f, uint32_t size_mb,
                            uint32_t jedec_id);
void gevico_spi_flash_reset(GEVICOSPIFlash *f);
uint8_t gevico_spi_flash_transfer(GEVICOSPIFlash *f, uint8_t tx);
void gevico_spi_flash_cs_deassert(GEVICOSPIFlash *f);

#endif

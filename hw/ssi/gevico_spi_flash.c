/*
 * G233 SPI NOR Flash emulation (W25X16 / W25X32)
 *
 * Copyright (c) 2025 Chao Liu <chao.liu@yeah.net>
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * State-machine based emulation of Winbond SPI NOR Flash.
 * Each call to gevico_spi_flash_transfer() sends one byte (MOSI) and
 * returns one byte (MISO).
 */

#include "qemu/osdep.h"
#include "hw/ssi/gevico_spi_flash.h"

void gevico_spi_flash_init(GEVICOSPIFlash *f, uint32_t size_mb,
                            uint32_t jedec_id)
{
    f->size = size_mb * 1024 * 1024;
    f->jedec_id = jedec_id;
    f->storage = g_malloc0(f->size);
    f->state = FLASH_IDLE;
    f->status = 0;
    f->write_enable = false;
}

void gevico_spi_flash_reset(GEVICOSPIFlash *f)
{
    f->state = FLASH_IDLE;
    f->cmd = 0;
    f->addr = 0;
    f->addr_shift = 0;
    f->data_count = 0;
    f->data_pos = 0;
    f->status = 0;
    f->write_enable = false;
}

/* Helper: build a 24-bit address from 3 sequential bytes (MSB first) */
static void flash_addr_byte(GEVICOSPIFlash *f, uint8_t byte)
{
    f->addr = (f->addr << 8) | byte;
    f->addr_shift--;
}

/* Return the response byte based on current state and input byte.
 * State transitions happen on each call. */
uint8_t gevico_spi_flash_transfer(GEVICOSPIFlash *f, uint8_t tx)
{
    uint8_t rx = 0;

    switch (f->state) {
    case FLASH_IDLE:
        f->cmd = tx;
        switch (tx) {
        case FLASH_CMD_JEDEC_ID:
            f->state = FLASH_JEDEC_ID;
            f->data_count = 3;
            f->data_pos = 0;
            /* first response byte available on next transfer */
            rx = 0;
            break;
        case FLASH_CMD_READ_STATUS:
            f->state = FLASH_RDSR;
            break;
        case FLASH_CMD_WRITE_ENABLE:
            f->write_enable = true;
            f->status |= FLASH_SR_WEL;
            f->state = FLASH_IDLE;
            break;
        case FLASH_CMD_WRITE_DISABLE:
            f->write_enable = false;
            f->status &= ~FLASH_SR_WEL;
            f->state = FLASH_IDLE;
            break;
        case FLASH_CMD_SECTOR_ERASE:
            f->state = FLASH_SE_ADDR;
            f->addr = 0;
            f->addr_shift = 3;
            break;
        case FLASH_CMD_PAGE_PROGRAM:
            f->state = FLASH_PP_ADDR;
            f->addr = 0;
            f->addr_shift = 3;
            break;
        case FLASH_CMD_READ_DATA:
            f->state = FLASH_READ_ADDR;
            f->addr = 0;
            f->addr_shift = 3;
            break;
        default:
            /* unknown command — stay idle */
            break;
        }
        break;

    case FLASH_JEDEC_ID:
        /* Return next JEDEC ID byte */
        switch (f->data_pos) {
        case 0:
            rx = (f->jedec_id >> 16) & 0xFF;
            break;
        case 1:
            rx = (f->jedec_id >> 8) & 0xFF;
            break;
        case 2:
            rx = f->jedec_id & 0xFF;
            break;
        default:
            rx = 0;
            break;
        }
        f->data_pos++;
        if (f->data_pos >= 3) {
            f->state = FLASH_IDLE;
        }
        break;

    case FLASH_RDSR:
        rx = f->status;
        f->state = FLASH_IDLE;
        break;

    case FLASH_SE_ADDR:
        flash_addr_byte(f, tx);
        if (f->addr_shift == 0) {
            /* Erase the 4KB sector */
            if (f->write_enable && f->addr < f->size) {
                uint32_t sector_base = f->addr & ~(FLASH_SECTOR_SIZE - 1);
                memset(f->storage + sector_base, 0xFF, FLASH_SECTOR_SIZE);
                f->write_enable = false;
                f->status &= ~FLASH_SR_WEL;
            }
            f->state = FLASH_IDLE;
        }
        break;

    case FLASH_PP_ADDR:
        flash_addr_byte(f, tx);
        if (f->addr_shift == 0) {
            f->state = FLASH_PP_DATA;
            f->data_count = 0;
            f->data_pos = f->addr & (FLASH_PAGE_SIZE - 1);
            /* Align to page start for tracking */
            f->addr &= ~(FLASH_PAGE_SIZE - 1);
        }
        break;

    case FLASH_PP_DATA:
        /* Program one byte (only if write enabled) */
        if (f->write_enable && (f->addr + f->data_pos) < f->size) {
            /* Flash can only clear bits (1→0); erase sets to 0xFF first */
            f->storage[f->addr + f->data_pos] &= tx;
        }
        f->data_pos++;
        f->data_count++;
        /* Stop at page boundary (256 bytes) */
        if (f->data_count >= FLASH_PAGE_SIZE) {
            f->write_enable = false;
            f->status &= ~FLASH_SR_WEL;
            f->state = FLASH_IDLE;
        }
        break;

    case FLASH_READ_ADDR:
        flash_addr_byte(f, tx);
        if (f->addr_shift == 0) {
            f->state = FLASH_READ_DATA;
            f->data_pos = f->addr;
        }
        break;

    case FLASH_READ_DATA:
        if (f->data_pos < f->size) {
            rx = f->storage[f->data_pos];
        } else {
            rx = 0xFF;
        }
        f->data_pos++;
        /* Continue reading until CS de-asserted */
        break;
    }

    return rx;
}

/* Called when CS is de-asserted.  Terminates any in-progress command
 * and commits pending operations (e.g. page program). */
void gevico_spi_flash_cs_deassert(GEVICOSPIFlash *f)
{
    switch (f->state) {
    case FLASH_PP_DATA:
        /* Page program terminated by CS↑: commit and clear WEL */
        f->write_enable = false;
        f->status &= ~FLASH_SR_WEL;
        break;
    case FLASH_SE_ADDR:
        /* Incomplete address — abort erase */
        break;
    default:
        break;
    }
    f->state = FLASH_IDLE;
}

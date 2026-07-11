/*
 * G233 Rust SPI Controller — C-side declaration for the Rust device.
 *
 * Copyright (c) 2025 HUST OpenAtom Open Source Club.
 * Author(s): Chen Miao <chenmiao@openatom.club>
 * Author(s): Chao Liu <chao.liu@openatom.club>
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_G233_SPI_H
#define HW_G233_SPI_H

#include "hw/core/sysbus.h"

/**
 * g233_rspi_create:
 * @addr: MMIO base address for the device registers
 *
 * Create, realize, and map the G233 Rust SPI controller.  The device
 * includes a built-in AT25 SPI EEPROM (256 bytes) on CS0.
 *
 * Returns: pointer to the realized DeviceState
 */
DeviceState *g233_rspi_create(hwaddr addr);

#endif /* HW_G233_SPI_H */

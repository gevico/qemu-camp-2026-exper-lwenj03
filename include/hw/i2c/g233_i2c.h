/*
 * G233 I2C GPIO Controller — C-side declaration for the Rust device.
 *
 * Copyright (c) 2025 HUST OpenAtom Open Source Club.
 * Author(s): Chen Miao <chenmiao@openatom.club>
 * Author(s): Chao Liu <chao.liu@openatom.club>
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_G233_I2C_H
#define HW_G233_I2C_H

#include "hw/core/sysbus.h"

/**
 * g233_i2c_gpio_create:
 * @addr: MMIO base address for the device registers
 *
 * Create, realize, and map the G233 I2C GPIO controller.  The device
 * includes an on-chip I2C bus with a built-in AT24C02 EEPROM at slave
 * address 0x50.
 *
 * Returns: pointer to the realized DeviceState
 */
DeviceState *g233_i2c_gpio_create(hwaddr addr);

#endif /* HW_G233_I2C_H */

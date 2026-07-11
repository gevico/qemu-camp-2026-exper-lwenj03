// Copyright 2025 HUST OpenAtom Open Source Club.
// Author(s): Chen Miao <chenmiao@openatom.club>
// Author(s): Chao Liu <chao.liu@openatom.club>
// SPDX-License-Identifier: GPL-2.0-or-later

//! G233 I2C GPIO Controller — Rust SysBus device.
//!
//! This crate provides a QEMU device model for the G233 SoC's I2C
//! controller at MMIO base address `0x10013000`.  It wraps the
//! pure-Rust I2C bus abstraction (`i2c_rs`) and includes a built-in
//! AT24C02 EEPROM at slave address 0x50.

mod device;

use hwcore::DeviceState;
use qom::ObjectClassMethods;
use qom::ObjectDeref;
use system::SysBusDeviceMethods;
use util::ResultExt;

use device::G233I2CGpioState;

/// Create and realize the G233 I2C GPIO controller at the given base
/// address.  Called from C board code (`hw/riscv/g233.c`).
///
/// # Safety
///
/// Must only be called during board initialization while the BQL is held.
#[no_mangle]
pub unsafe extern "C" fn g233_i2c_gpio_create(addr: u64) -> *mut DeviceState {
    let dev = G233I2CGpioState::new();
    dev.sysbus_realize().unwrap_fatal();
    dev.mmio_map(0, addr);
    dev.as_mut_ptr()
}

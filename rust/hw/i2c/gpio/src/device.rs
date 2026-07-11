// Copyright 2025 HUST OpenAtom Open Source Club.
// Author(s): Chen Miao <chenmiao@openatom.club>
// Author(s): Chao Liu <chao.liu@openatom.club>
// SPDX-License-Identifier: GPL-2.0-or-later

//! G233 I2C GPIO Controller Device
//!
//! This is a simple I2C controller with MMIO registers that wraps the
//! pure-Rust [`I2CBus`](i2c_rs::I2CBus) and drives a built-in AT24C02
//! EEPROM at address 0x50.
//!
//! Register map (base 0x10013000):
//!   0x00  I2C_CTRL     — bit0: EN, bit1: START, bit2: STOP, bit3: RW
//!   0x04  I2C_STATUS   — bit0: BUSY, bit1: ACK, bit2: DONE
//!   0x08  I2C_ADDR     — 7-bit slave address
//!   0x0C  I2C_DATA     — data register
//!   0x10  I2C_PRESCALE — clock prescaler

use std::ffi::CStr;

use bql::prelude::*;
use common::prelude::*;
use hwcore::prelude::*;
use i2c_rs::{I2CBus, I2CEvent, I2CSlave};
use qom::prelude::*;
use system::prelude::*;

// ─── Register offsets ──────────────────────────────────────────────────

const REG_CTRL: hwaddr = 0x00;
const REG_STATUS: hwaddr = 0x04;
const REG_ADDR: hwaddr = 0x08;
const REG_DATA: hwaddr = 0x0C;
const REG_PRESCALE: hwaddr = 0x10;

// ─── CTRL register bits ─────────────────────────────────────────────────

const CTRL_EN: u32 = 1 << 0;
const CTRL_START: u32 = 1 << 1;
const CTRL_STOP: u32 = 1 << 2;
const CTRL_RW: u32 = 1 << 3; // 0 = write (master→slave), 1 = read (slave→master)

// ─── STATUS register bits ───────────────────────────────────────────────

const STATUS_BUSY: u32 = 1 << 0;
const STATUS_ACK: u32 = 1 << 1;
const STATUS_DONE: u32 = 1 << 2;

// ─── AT24C02 EEPROM (256 bytes, 8-byte page size) ──────────────────────

/// AT24C02: 256-byte I2C EEPROM with 8-byte page write granularity.
///
/// Protocol:
/// - Write: master sends memory address byte, then data bytes
/// - Read:  master sends memory address byte in a write transaction,
///          then a repeated START in read mode, then clocks out bytes
/// - Page write: sequential writes wrap within the current 8-byte page
struct At24c02 {
    addr: u8,
    mem: [u8; 256],
    cur: u8,
    haveaddr: bool,
}

impl At24c02 {
    fn new(addr: u8) -> Self {
        Self {
            addr,
            mem: [0xFF; 256],
            cur: 0,
            haveaddr: false,
        }
    }
}

impl I2CSlave for At24c02 {
    fn address(&self) -> u8 {
        self.addr
    }

    fn event(&mut self, event: I2CEvent) -> i32 {
        match event {
            I2CEvent::StartSend | I2CEvent::StartRecv => {
                self.haveaddr = false;
            }
            _ => {}
        }
        0
    }

    fn send(&mut self, data: u8) -> i32 {
        if !self.haveaddr {
            // First byte after START in write mode is the memory address
            self.cur = data;
            self.haveaddr = true;
        } else {
            // Subsequent bytes are data
            self.mem[self.cur as usize] = data;
            // Wrap within 8-byte page boundary
            self.cur = (self.cur & 0xF8) | ((self.cur + 1) & 0x07);
        }
        0
    }

    fn recv(&mut self) -> u8 {
        let val = self.mem[self.cur as usize];
        self.cur = self.cur.wrapping_add(1);
        val
    }
}

// ─── Inner state (register file + I2C bus) ─────────────────────────────

/// All mutable state of the I2C GPIO controller, protected by the BQL.
pub struct I2CGpioInner {
    ctrl: u32,
    status: u32,
    addr: u32,
    data: u32,
    prescale: u32,
    bus: I2CBus,
}

impl I2CGpioInner {
    fn new() -> Self {
        let mut bus = I2CBus::new();
        bus.attach(Box::new(At24c02::new(0x50)));
        Self {
            ctrl: 0,
            status: 0,
            addr: 0,
            data: 0,
            prescale: 0,
            bus,
        }
    }
}

// ─── QOM SysBus device ──────────────────────────────────────────────────

/// The G233 I2C GPIO controller SysBus device.
///
/// Exposes five 32-bit MMIO registers.  Internally wraps a pure-Rust
/// [`I2CBus`] with a built-in AT24C02 EEPROM at slave address 0x50.
#[repr(C)]
#[derive(qom::Object, hwcore::Device)]
pub struct G233I2CGpioState {
    pub parent_obj: ParentField<SysBusDevice>,
    pub iomem: MemoryRegion,
    pub inner: BqlRefCell<I2CGpioInner>,
}

qom_isa!(G233I2CGpioState: SysBusDevice, DeviceState, Object);

unsafe impl ObjectType for G233I2CGpioState {
    type Class = <SysBusDevice as ObjectType>::Class;
    const TYPE_NAME: &'static CStr = c"g233-i2c-gpio";
}

impl ObjectImpl for G233I2CGpioState {
    type ParentType = SysBusDevice;

    const INSTANCE_INIT: Option<unsafe fn(ParentInit<Self>)> = Some(Self::init);
    const INSTANCE_POST_INIT: Option<fn(&Self)> = Some(Self::post_init);
    const CLASS_INIT: fn(&mut Self::Class) = Self::Class::class_init::<Self>;
}

impl DeviceImpl for G233I2CGpioState {}

impl ResettablePhasesImpl for G233I2CGpioState {}

impl SysBusDeviceImpl for G233I2CGpioState {}

// ─── MMIO implementation ────────────────────────────────────────────────

impl G233I2CGpioState {
    /// Device instance initialization (called once by QOM).
    unsafe fn init(mut this: ParentInit<Self>) {
        static I2C_OPS: MemoryRegionOps<G233I2CGpioState> =
            MemoryRegionOpsBuilder::<G233I2CGpioState>::new()
                .read(&G233I2CGpioState::read)
                .write(&G233I2CGpioState::write)
                .little_endian()
                .impl_sizes(4, 4)
                .build();

        MemoryRegion::init_io(
            &mut uninit_field_mut!(*this, iomem),
            &I2C_OPS,
            "g233-i2c-gpio",
            0x1000,
        );

        uninit_field_mut!(*this, inner).write(BqlRefCell::new(I2CGpioInner::new()));
    }

    /// Post-initialization: expose the MMIO region to the board.
    fn post_init(&self) {
        self.init_mmio(&self.iomem);
    }

    /// MMIO read callback.  All registers are 32-bit little-endian.
    fn read(&self, offset: hwaddr, _size: u32) -> u64 {
        let inner = self.inner.borrow();
        match offset {
            REG_CTRL => inner.ctrl as u64,
            REG_STATUS => inner.status as u64,
            REG_ADDR => inner.addr as u64,
            REG_DATA => inner.data as u64,
            REG_PRESCALE => inner.prescale as u64,
            _ => 0,
        }
    }

    /// MMIO write dispatcher.
    fn write(&self, offset: hwaddr, value: u64, _size: u32) {
        let val = value as u32;
        match offset {
            REG_CTRL => self.write_ctrl(val),
            REG_ADDR => self.inner.borrow_mut().addr = val,
            REG_DATA => self.inner.borrow_mut().data = val,
            REG_PRESCALE => self.inner.borrow_mut().prescale = val,
            _ => {}
        }
    }

    /// Handle a write to the CTRL register.
    ///
    /// The CTRL register is a command register:
    ///   EN | START  → I2C START condition
    ///   EN | STOP   → I2C STOP condition
    ///   EN          → write data byte (DATA register → slave)
    ///   EN | RW     → read data byte (slave → DATA register)
    ///
    /// All I2C operations complete synchronously.  DONE is set
    /// immediately; the test polling loop will see it on the first read.
    fn write_ctrl(&self, val: u32) {
        let mut inner = self.inner.borrow_mut();
        inner.ctrl = val;

        // A write without EN is a no-op.
        if val & CTRL_EN == 0 {
            return;
        }

        if val & CTRL_START != 0 {
            // ── START condition ──────────────────────────────────────
            let is_recv = (val & CTRL_RW) != 0;
            let addr = inner.addr as u8;
            let result = inner.bus.start_transfer(addr, is_recv);
            if result == 0 {
                inner.status |= STATUS_ACK;
            } else {
                inner.status &= !STATUS_ACK;
            }
            inner.status |= STATUS_BUSY;
            inner.status |= STATUS_DONE;
        } else if val & CTRL_STOP != 0 {
            // ── STOP condition ───────────────────────────────────────
            inner.bus.end_transfer();
            inner.status &= !STATUS_BUSY;
            inner.status |= STATUS_DONE;
        } else if val & CTRL_RW != 0 {
            // ── Read (slave → master) ────────────────────────────────
            let byte = inner.bus.recv();
            inner.data = byte as u32;
            inner.status |= STATUS_DONE;
        } else {
            // ── Write (master → slave) ───────────────────────────────
            let data = inner.data as u8;
            let result = inner.bus.send(data);
            if result == 0 {
                inner.status |= STATUS_ACK;
            } else {
                inner.status &= !STATUS_ACK;
            }
            inner.status |= STATUS_DONE;
        }
    }
}

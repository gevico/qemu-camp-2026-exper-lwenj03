// Copyright 2025 HUST OpenAtom Open Source Club.
// Author(s): Chen Miao <chenmiao@openatom.club>
// Author(s): Chao Liu <chao.liu@openatom.club>
// SPDX-License-Identifier: GPL-2.0-or-later

//! G233 Rust SPI Controller Device
//!
//! A simple SPI master controller with MMIO registers.  Internally
//! drives a built-in AT25 SPI EEPROM (256 bytes) on CS0.
//!
//! Register map (base 0x10019000):
//!   0x00  RSPI_CR1  — bit0: SPE (enable), bit2: MSTR (master)
//!   0x04  RSPI_SR   — bit0: RXNE, bit1: TXE, bit4: OVERRUN
//!   0x08  RSPI_DR   — data register (write=TX, read=RX)
//!   0x0C  RSPI_CS   — chip select (0 or 1)

#![allow(dead_code)]

use std::ffi::CStr;

use bql::prelude::*;
use common::prelude::*;
use hwcore::prelude::*;
use qom::prelude::*;
use system::prelude::*;

// ─── Register offsets ──────────────────────────────────────────────────

const REG_CR1: hwaddr = 0x00;
const REG_SR: hwaddr = 0x04;
const REG_DR: hwaddr = 0x08;
const REG_CS: hwaddr = 0x0C;

// ─── CR1 bits ───────────────────────────────────────────────────────────

const CR1_SPE: u32 = 1 << 0;
const CR1_MSTR: u32 = 1 << 2;

// ─── SR bits ────────────────────────────────────────────────────────────

const SR_RXNE: u32 = 1 << 0;
const SR_TXE: u32 = 1 << 1;
const SR_OVERRUN: u32 = 1 << 4;

// ─── AT25 SPI EEPROM commands ──────────────────────────────────────────

const CMD_WREN: u8 = 0x06;
const CMD_RDSR: u8 = 0x05;
const CMD_READ: u8 = 0x03;
const CMD_WRITE: u8 = 0x02;

// Status register bits
const AT25_SR_WIP: u8 = 1 << 0;
const AT25_SR_WEL: u8 = 1 << 1;

// ─── AT25 SPI EEPROM (256 bytes) ───────────────────────────────────────

/// AT25 SPI EEPROM state machine states.
#[derive(Debug, Clone, Copy, PartialEq)]
enum At25State {
    Idle,
    ReadStatus,
    ReadAddr,
    ReadData,
    WriteAddr,
    WriteData,
}

/// AT25 SPI EEPROM: 256-byte SPI flash with status register.
struct At25Eeprom {
    mem: [u8; 256],
    sr: u8,         // status register (bit0=WIP, bit1=WEL)
    state: At25State,
    addr: u8,       // current address pointer
    write_busy: u32, // remaining write-busy counter (decrements on clock)
}

impl At25Eeprom {
    fn new() -> Self {
        Self {
            mem: [0xFF; 256],
            sr: 0,
            state: At25State::Idle,
            addr: 0,
            write_busy: 0,
        }
    }

    /// Process a byte received from the SPI master.
    /// Returns the byte to send back (full-duplex SPI).
    fn transfer(&mut self, tx: u8) -> u8 {
        if self.write_busy > 0 {
            self.write_busy -= 1;
            // During write cycle, device is busy — return 0x00
            return 0x00;
        }

        // If a known command byte arrives while the state machine is in a
        // data-transfer state, treat it as a new transaction (simulating
        // CS deassertion/reassertion between transactions).
        if self.state != At25State::Idle && self.state != At25State::ReadStatus
            && (tx == CMD_WREN || tx == CMD_RDSR || tx == CMD_READ || tx == CMD_WRITE)
        {
            self.state = At25State::Idle;
        }

        match self.state {
            At25State::Idle => {
                match tx {
                    CMD_WREN => {
                        self.sr |= AT25_SR_WEL;
                        self.state = At25State::Idle;
                    }
                    CMD_RDSR => {
                        self.state = At25State::ReadStatus;
                    }
                    CMD_READ => {
                        self.state = At25State::ReadAddr;
                    }
                    CMD_WRITE => {
                        self.state = At25State::WriteAddr;
                    }
                    _ => {
                        self.state = At25State::Idle;
                    }
                }
                0x00
            }
            At25State::ReadStatus => {
                self.state = At25State::Idle;
                self.sr
            }
            At25State::ReadAddr => {
                self.addr = tx;
                self.state = At25State::ReadData;
                0x00
            }
            At25State::ReadData => {
                let val = self.mem[self.addr as usize];
                self.addr = self.addr.wrapping_add(1);
                val
            }
            At25State::WriteAddr => {
                self.addr = tx;
                self.state = At25State::WriteData;
                0x00
            }
            At25State::WriteData => {
                self.mem[self.addr as usize] = tx;
                self.addr = self.addr.wrapping_add(1);
                // Clear WEL after actual write data
                self.sr &= !AT25_SR_WEL;
                0x00
            }
        }
    }

    /// Advance the write-busy counter (called on each CS-related event).
    fn tick(&mut self) {
        if self.write_busy > 0 {
            self.write_busy -= 1;
        }
    }

    /// Reset the state machine (called on CS deassertion or controller disable).
    fn reset_state(&mut self) {
        self.state = At25State::Idle;
    }
}

// ─── Internal controller state ─────────────────────────────────────────

/// All mutable state of the SPI controller, protected by the BQL.
pub struct RspiInner {
    cr1: u32,
    sr: u32,
    dr: u32,
    cs: u32,
    flash: At25Eeprom,
}

impl RspiInner {
    fn new() -> Self {
        let mut inner = Self {
            cr1: 0,
            sr: 0,
            dr: 0,
            cs: 0,
            flash: At25Eeprom::new(),
        };
        // TXE is set only when SPI is enabled
        inner.update_sr();
        inner
    }

    /// Recompute SR flags based on controller state.
    fn update_sr(&mut self) {
        if self.is_enabled() {
            // When enabled, TX buffer is always ready (full-duplex)
            self.sr |= SR_TXE;
        } else {
            self.sr = 0;
        }
    }

    fn is_enabled(&self) -> bool {
        (self.cr1 & CR1_SPE) != 0
    }
}

// ─── QOM SysBus device ──────────────────────────────────────────────────

/// The G233 Rust SPI controller SysBus device.
#[repr(C)]
#[derive(qom::Object, hwcore::Device)]
pub struct G233RspiState {
    pub parent_obj: ParentField<SysBusDevice>,
    pub iomem: MemoryRegion,
    pub inner: BqlRefCell<RspiInner>,
}

qom_isa!(G233RspiState: SysBusDevice, DeviceState, Object);

unsafe impl ObjectType for G233RspiState {
    type Class = <SysBusDevice as ObjectType>::Class;
    const TYPE_NAME: &'static CStr = c"g233-rspi";
}

impl ObjectImpl for G233RspiState {
    type ParentType = SysBusDevice;

    const INSTANCE_INIT: Option<unsafe fn(ParentInit<Self>)> = Some(Self::init);
    const INSTANCE_POST_INIT: Option<fn(&Self)> = Some(Self::post_init);
    const CLASS_INIT: fn(&mut Self::Class) = Self::Class::class_init::<Self>;
}

impl DeviceImpl for G233RspiState {}

impl ResettablePhasesImpl for G233RspiState {}

impl SysBusDeviceImpl for G233RspiState {}

// ─── MMIO implementation ────────────────────────────────────────────────

impl G233RspiState {
    unsafe fn init(mut this: ParentInit<Self>) {
        static RSPI_OPS: MemoryRegionOps<G233RspiState> =
            MemoryRegionOpsBuilder::<G233RspiState>::new()
                .read(&G233RspiState::read)
                .write(&G233RspiState::write)
                .little_endian()
                .impl_sizes(4, 4)
                .build();

        MemoryRegion::init_io(
            &mut uninit_field_mut!(*this, iomem),
            &RSPI_OPS,
            "g233-rspi",
            0x1000,
        );

        uninit_field_mut!(*this, inner).write(BqlRefCell::new(RspiInner::new()));
    }

    fn post_init(&self) {
        self.init_mmio(&self.iomem);
    }

    /// MMIO read callback.  Reading DR clears RXNE.
    fn read(&self, offset: hwaddr, _size: u32) -> u64 {
        if offset == REG_DR {
            let mut inner = self.inner.borrow_mut();
            let val = inner.dr as u64;
            // Reading DR clears RXNE and reloads TXE
            inner.sr &= !SR_RXNE;
            inner.sr |= SR_TXE;
            return val;
        }
        let inner = self.inner.borrow();
        match offset {
            REG_CR1 => inner.cr1 as u64,
            REG_SR => inner.sr as u64,
            REG_DR => inner.dr as u64,
            REG_CS => inner.cs as u64,
            _ => 0,
        }
    }

    /// MMIO write callback.
    fn write(&self, offset: hwaddr, value: u64, _size: u32) {
        let val = value as u32;
        match offset {
            REG_CR1 => self.write_cr1(val),
            REG_DR => self.write_dr(val),
            REG_CS => {
                let mut inner = self.inner.borrow_mut();
                inner.cs = val;
                // Reset flash state machine on CS change
                inner.flash.reset_state();
            }
            _ => {}
        }
    }

    /// Handle CR1 write (SPI enable/disable, master mode).
    fn write_cr1(&self, val: u32) {
        let mut inner = self.inner.borrow_mut();
        inner.cr1 = val;
        if val & CR1_SPE != 0 {
            // Enable SPI — set TXE
            inner.sr |= SR_TXE;
        } else {
            // Disable SPI — clear all status
            inner.sr = 0;
            inner.flash.reset_state();
        }
    }

    /// Handle DR write (trigger SPI transfer with the flash).
    fn write_dr(&self, val: u32) {
        let mut inner = self.inner.borrow_mut();

        // Check overrun: writing DR while RXNE is still set
        if inner.sr & SR_RXNE != 0 {
            inner.sr |= SR_OVERRUN;
        }

        let tx_byte = val as u8;

        // Perform the SPI transfer with the flash
        // SPI is full-duplex: send a byte and receive one simultaneously
        let rx_byte = inner.flash.transfer(tx_byte);

        // Update data register with received byte
        inner.dr = rx_byte as u32;

        // Clear TXE (data has been written to shift register)
        inner.sr &= !SR_TXE;

        // Set RXNE (received data is available)
        inner.sr |= SR_RXNE;

        // Set TXE again (ready for next byte — full-duplex)
        inner.sr |= SR_TXE;

        // Advance flash write-busy counter
        inner.flash.tick();
    }
}

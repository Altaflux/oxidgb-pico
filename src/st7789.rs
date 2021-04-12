//! A ST7789 interface, designed for Pimoroni's Pico display.

use crate::rp2040::gpio::{gpio_set_function, GPIOFunc};
use crate::rp2040::pac;
use crate::rp2040::pac::SIO;
use crate::rp2040::spi::{SPIInstance, SPI};

use cortex_m::delay::Delay;
use cortex_m::prelude::*;

pub struct ST7789<T: SPIInstance> {
    spi: SPI<T>,
    cs_gpio: u32,
    dc_gpio: u32,
}

bitflags! {
    /// SPI commands that can be used when talking to the device.
    struct ST7789Registers: u8 {
        const SWRESET = 0x01;
        const TEON = 0x35;
        const MADCTL = 0x36;
        const COLMOD = 0x3A;
        const GCTRL = 0xB7;
        const VCOMS = 0xBB;
        const LCMCTRL = 0xC0;
        const VDVVRHEN = 0xC2;
        const VRHS = 0xC3;
        const VDVS = 0xC4;
        const FRCTRL2 = 0xC6;
        const PWRCTRL1 = 0xD0;
        const FRMCTR1 = 0xB1;
        const FRMCTR2 = 0xB2;
        const GMCTRP1 = 0xE0;
        const GMCTRN1 = 0xE1;
        const INVOFF = 0x20;
        const SLPOUT = 0x11;
        const DISPON = 0x29;
        const GAMSET = 0x26;
        const DISPOFF = 0x28;
        const RAMWR = 0x2C;
        const INVON = 0x21;
        const CASET = 0x2A;
        const RASET = 0x2B;
    }
}

bitflags! {
    /// Bitflags for the MadCTl register.
    struct MadCTL: u8 {
        const ROW_ORDER   = 0b10000000;
        const COL_ORDER   = 0b01000000;
        const SWAP_XY     = 0b00100000;  // AKA "MV"
        const SCAN_ORDER  = 0b00010000;
        const RGB         = 0b00001000;
        const HORIZ_ORDER = 0b00000100;
    }
}

impl<T: SPIInstance> ST7789<T> {
    /// Writes a single command to the device in a blocking fashion.
    fn command(&mut self, sio: &SIO, command: ST7789Registers, data: Option<&[u8]>) {
        // Ported from https://github.com/pimoroni/pimoroni-pico/blob/main/drivers/st7789/st7789.cpp:
        // Write 0 (low) to the CS and DC pins
        sio.gpio_out_clr
            .modify(|_, x| unsafe { x.bits((1 << self.cs_gpio) | (1 << self.dc_gpio)) });

        self.spi.write_blocking(&[command.bits as _]);

        if let Some(data) = data {
            // Data mode
            sio.gpio_out_set
                .modify(|_, x| unsafe { x.bits(1 << self.dc_gpio) });

            self.spi.write_blocking(data);
        }

        sio.gpio_out_set
            .modify(|_, x| unsafe { x.bits(1 << self.cs_gpio) });
    }

    /// Writes a single command to the device in a blocking fashion (with 16-bit data).
    fn command_16(&mut self, sio: &SIO, command: ST7789Registers, data: Option<&[u16]>) {
        // Ported from https://github.com/pimoroni/pimoroni-pico/blob/main/drivers/st7789/st7789.cpp:
        // Write 0 (low) to the CS and DC pins
        sio.gpio_out_clr
            .modify(|_, x| unsafe { x.bits((1 << self.cs_gpio) | (1 << self.dc_gpio)) });

        self.spi.write_blocking_16(&[command.bits as _]);

        if let Some(data) = data {
            // Data mode
            sio.gpio_out_set
                .modify(|_, x| unsafe { x.bits(1 << self.dc_gpio) });

            self.spi.write_blocking_16(data);
        }

        sio.gpio_out_set
            .modify(|_, x| unsafe { x.bits(1 << self.cs_gpio) });
    }

    /// Updates the framebuffer on the device in a blocking fashion.
    pub fn update_framebuffer(&mut self, sio: &SIO, data: &[u16]) {
        self.command_16(sio, ST7789Registers::RAMWR, Some(data))
    }

    pub fn new(
        sys_timer: &mut Delay,
        pads: &pac::PADS_BANK0,
        io: &pac::IO_BANK0,
        sio: &SIO,
        dc: u32,
        cs: u32,
        sck: u32,
        mosi: u32,
        spi: SPI<T>,
    ) -> Self {
        gpio_set_function(pads, io, dc, GPIOFunc::SIO);
        sio.gpio_oe_set.modify(|_, x| unsafe { x.bits(1 << dc) });

        gpio_set_function(pads, io, cs, GPIOFunc::SIO);
        sio.gpio_oe_set.modify(|_, x| unsafe { x.bits(1 << cs) });

        gpio_set_function(pads, io, sck, GPIOFunc::SPI);
        gpio_set_function(pads, io, mosi, GPIOFunc::SPI);

        let mut display = ST7789 {
            spi,
            cs_gpio: cs,
            dc_gpio: dc,
        };

        // Auto init sequence
        display.command(sio, ST7789Registers::SWRESET, None);

        sys_timer.delay_ms(150);

        display.command(sio, ST7789Registers::TEON, Some(&[0x00]));
        display.command(sio, ST7789Registers::COLMOD, Some(&[0b101]));

        display.command(sio, ST7789Registers::INVON, None);
        display.command(sio, ST7789Registers::SLPOUT, None);
        display.command(sio, ST7789Registers::DISPON, None);

        sys_timer.delay_ms(50);

        let mut ca_set: [u8; 4] = [0; 4];
        ca_set[0..2].copy_from_slice(&40u16.to_be_bytes());
        ca_set[2..4].copy_from_slice(&279u16.to_be_bytes());
        let mut ra_set: [u8; 4] = [0; 4];
        ra_set[0..2].copy_from_slice(&53u16.to_be_bytes());
        ra_set[2..4].copy_from_slice(&187u16.to_be_bytes());

        let mad_ctl = MadCTL::COL_ORDER | MadCTL::SWAP_XY | MadCTL::SCAN_ORDER;

        display.command(sio, ST7789Registers::CASET, Some(&ca_set));
        display.command(sio, ST7789Registers::RASET, Some(&ra_set));
        display.command(sio, ST7789Registers::MADCTL, Some(&[mad_ctl.bits]));

        display
    }
}

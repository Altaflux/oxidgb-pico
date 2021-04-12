//! SPI handling structures.
//! Trait structure based off https://github.com/jannic/rp-microcontroller-rs/blob/master/boards/rp-pico/examples/uart/uart.rs

use core::ops::Deref;

use super::pac;

/// The kind of SPI interface being used.
pub enum SPIInterface {
    SPI0,
    SPI1,
}

/// Used as a generic interface across SPI interfaces.
pub trait SPIInstance: Deref<Target = pac::spi0::RegisterBlock> {
    const SPIKIND: SPIInterface;
}

/// A container for an SPI interface, providing a high-level interface to the device.
pub struct SPI<T: SPIInstance> {
    inner: T,
    data_bit_size: u8,
}

impl<T: SPIInstance> SPI<T> {
    /// Returns if the device is currently writable (i.e the outgoing buffer is available).
    #[inline(always)]
    fn is_writable(&self) -> bool {
        self.inner.sspsr.read().tnf().bit()
    }

    /// Returns if the device is currently readable (i.e data is available).
    #[inline(always)]
    fn is_readable(&self) -> bool {
        self.inner.sspsr.read().rne().bit()
    }

    /// Returns if the device is currently busy.
    #[inline(always)]
    fn is_busy(&self) -> bool {
        self.inner.sspsr.read().bsy().bit()
    }

    /// Writes 8-bit data to the interface in a blocking fashion.
    #[link_section = ".data"]
    #[inline(never)]
    pub fn write_blocking(&mut self, data: &[u8]) {
        if self.data_bit_size != 8 {
            self.set_8_bit_size();
        }

        // As per https://github.com/raspberrypi/pico-sdk src/rp2_common/hardware_spi/spi.c:

        // Write to TX FIFO whilst ignoring RX, then clean up afterward. When RX
        // is full, PL022 inhibits RX pushes, and sets a sticky flag on
        // push-on-full, but continues shifting. Safe if SSPIMSC_RORIM is not set.
        for data_byte in data {
            while !self.is_writable() {}

            self.inner
                .sspdr
                .write_with_zero(|x| unsafe { x.bits(*data_byte as _) });
        }

        // Drain RX FIFO, then wait for shifting to finish (which may be *after*
        // TX FIFO drains), then drain RX FIFO again
        while self.is_readable() {
            let _ = self.inner.sspdr.read();
        }

        while self.is_busy() {}

        while self.is_readable() {
            let _ = self.inner.sspdr.read();
        }

        // Don't leave overrun flag set
        self.inner.sspicr.write_with_zero(|x| x.roric().set_bit());
    }

    /// Writes 16-bit data to the interface in a blocking fashion.
    #[link_section = ".data"]
    #[inline(never)]
    pub fn write_blocking_16(&mut self, data: &[u16]) {
        if self.data_bit_size != 16 {
            self.set_16_bit_size();
        }

        // As per https://github.com/raspberrypi/pico-sdk src/rp2_common/hardware_spi/spi.c:

        // Write to TX FIFO whilst ignoring RX, then clean up afterward. When RX
        // is full, PL022 inhibits RX pushes, and sets a sticky flag on
        // push-on-full, but continues shifting. Safe if SSPIMSC_RORIM is not set.
        for data_byte in data {
            while !self.is_writable() {}

            self.inner
                .sspdr
                .write_with_zero(|x| unsafe { x.bits(*data_byte as _) });
        }

        // Drain RX FIFO, then wait for shifting to finish (which may be *after*
        // TX FIFO drains), then drain RX FIFO again
        while self.is_readable() {
            let _ = self.inner.sspdr.read();
        }

        while self.is_busy() {}

        while self.is_readable() {
            let _ = self.inner.sspdr.read();
        }

        // Don't leave overrun flag set
        self.inner.sspicr.write_with_zero(|x| x.roric().set_bit());
    }

    /// Puts this entire SPI instance into a reset state. Configurations will need to be
    /// reapplied in this case.
    fn reset(&mut self, resets: &pac::resets::RegisterBlock) {
        match T::SPIKIND {
            SPIInterface::SPI0 => resets.reset.modify(|_, x| x.spi0().set_bit()),
            SPIInterface::SPI1 => resets.reset.modify(|_, x| x.spi1().set_bit()),
        }
    }

    /// Takes the SPI device out of a reset status and waits for it to come back up.
    fn unreset(&mut self, resets: &pac::resets::RegisterBlock) {
        match T::SPIKIND {
            SPIInterface::SPI0 => resets.reset.modify(|_, x| x.spi0().clear_bit()),
            SPIInterface::SPI1 => resets.reset.modify(|_, x| x.spi1().clear_bit()),
        }

        while match T::SPIKIND {
            SPIInterface::SPI0 => resets.reset_done.read().spi0().bit_is_clear(),
            SPIInterface::SPI1 => resets.reset_done.read().spi1().bit_is_clear(),
        } {}
    }

    /// Manually sets the device's format.
    fn set_format(&mut self, data_bits: u8, cpol: bool, cpha: bool) {
        self.inner
            .sspcr0
            .modify(|_, x| unsafe { x.dss().bits(data_bits - 1).spo().bit(cpol).sph().bit(cpha) });

        self.data_bit_size = data_bits;
    }

    /// Shortcut for setting the SPI device into a byte transmission mode.
    fn set_8_bit_size(&mut self) {
        self.set_format(8, false, false)
    }

    /// Shortcut for setting the SPI device into a half-word transmission mode.
    fn set_16_bit_size(&mut self) {
        self.set_format(16, false, false)
    }

    /// Configures the baud-rate of the device to a new value.
    pub fn set_baudrate(&mut self, baud_rate: u32) {
        // TODO: Actually implement this - don't assume the clock speed
        let freq_in = 125 * 1000 * 1000; /*clock_get_hz(clk_peri);;*/

        let mut prescale_result = 0;
        for prescale in (2..=256).step_by(2) {
            prescale_result = prescale;

            if freq_in < (prescale as u64 + 2) * 256 * (baud_rate as u64) {
                break;
            }
        }

        let mut postdiv_result = 0;
        for postdiv in (2..=256).rev() {
            postdiv_result = postdiv;

            if freq_in / (prescale_result * (postdiv_result - 1)) > baud_rate as u64 {
                break;
            }
        }

        self.inner
            .sspcpsr
            .write_with_zero(|x| unsafe { x.bits(prescale_result as u32) });
        self.inner
            .sspcr0
            .write(|x| unsafe { x.scr().bits((postdiv_result - 1) as u8) });
    }

    /// Creates a new SPI instance, using the specified interface.
    pub fn new(instance: T, reset_interface: &pac::RESETS, default_speed: u32) -> Self {
        let mut instance = SPI {
            inner: instance,
            data_bit_size: 0,
        };

        instance.reset(reset_interface);
        instance.unreset(reset_interface);

        instance.set_baudrate(default_speed);
        instance.set_8_bit_size();
        // Always enable DREQ signals -- harmless if DMA is not listening
        instance
            .inner
            .sspdmacr
            .modify(|_, x| x.rxdmae().set_bit().txdmae().set_bit());
        instance.set_8_bit_size();

        // Finally enable the SPI
        instance.inner.sspcr1.modify(|_, x| x.sse().set_bit());

        instance
    }
}

impl SPIInstance for pac::SPI0 {
    const SPIKIND: SPIInterface = SPIInterface::SPI0;
}

impl SPIInstance for pac::SPI1 {
    const SPIKIND: SPIInterface = SPIInterface::SPI1;
}

//! An interface to the core/peripherals of the RP2040 chip.

pub use rp2040_pac as pac;

use core::ops::Not;

mod clocks;
pub mod gpio;
pub mod pwm;
pub mod spi;

/// Handle peripheral resets so the chip is usable.
pub fn setup_chip(p: &mut rp2040_pac::Peripherals, do_overclock: bool, overclock_multiplier: u32) {
    // Now reset all the peripherals, except QSPI and XIP (we're using those
    // to execute from external flash!)

    const RESETS_RESET_BITS: u32 = 0x01ffffff;
    const RESETS_RESET_USBCTRL_BITS: u32 = 0x01000000;
    const RESETS_RESET_UART1_BITS: u32 = 0x00800000;
    const RESETS_RESET_UART0_BITS: u32 = 0x00400000;
    const RESETS_RESET_SPI1_BITS: u32 = 0x00020000;
    const RESETS_RESET_SPI0_BITS: u32 = 0x00010000;
    const RESETS_RESET_RTC_BITS: u32 = 0x00008000;
    const RESETS_RESET_ADC_BITS: u32 = 0x00000001;

    const RESETS_RESET_IO_QSPI_BITS: u32 = 0x00000040;
    const RESETS_RESET_PADS_QSPI_BITS: u32 = 0x00000200;
    const RESETS_RESET_PLL_USB_BITS: u32 = 0x00002000;
    const RESETS_RESET_PLL_SYS_BITS: u32 = 0x00001000;

    p.RESETS.reset.write_with_zero(|w| unsafe {
        w.bits(
            (RESETS_RESET_IO_QSPI_BITS
                | RESETS_RESET_PADS_QSPI_BITS
                | RESETS_RESET_PLL_USB_BITS
                | RESETS_RESET_PLL_SYS_BITS)
                .not(),
        )
    });

    // We want to take everything out of reset, except these peripherals:
    //
    // * ADC
    // * RTC
    // * SPI0
    // * SPI1
    // * UART0
    // * UART1
    // * USBCTRL
    //
    // These must stay in reset until the clocks are sorted out.
    const PERIPHERALS_TO_UNRESET: u32 = RESETS_RESET_BITS
        & !(RESETS_RESET_ADC_BITS
            | RESETS_RESET_RTC_BITS
            | RESETS_RESET_SPI0_BITS
            | RESETS_RESET_SPI1_BITS
            | RESETS_RESET_UART0_BITS
            | RESETS_RESET_UART1_BITS
            | RESETS_RESET_USBCTRL_BITS);

    // Write 0 to the reset field to take it out of reset
    p.RESETS
        .reset
        .modify(|r, w| unsafe { w.bits(r.bits() & PERIPHERALS_TO_UNRESET.not()) });

    while (!p.RESETS.reset_done.read().bits() & PERIPHERALS_TO_UNRESET) != 0 {}

    clocks::init_clocks(p, do_overclock, overclock_multiplier);

    // Clear the rest now the clocks are running
    p.RESETS.reset.modify(|_r, w| {
        w.spi0()
            .clear_bit()
            .spi1()
            .clear_bit()
            .adc()
            .clear_bit()
            .rtc()
            .clear_bit()
            .uart0()
            .clear_bit()
            .uart1()
            .clear_bit()
            .usbctrl()
            .clear_bit()
    });

    const PERIPHERALS_TO_UNRESET_PERIPHERALS: u32 = RESETS_RESET_ADC_BITS
        | RESETS_RESET_RTC_BITS
        | RESETS_RESET_SPI0_BITS
        | RESETS_RESET_SPI1_BITS
        | RESETS_RESET_UART0_BITS
        | RESETS_RESET_UART1_BITS
        | RESETS_RESET_USBCTRL_BITS;

    while (!p.RESETS.reset_done.read().bits() & PERIPHERALS_TO_UNRESET_PERIPHERALS) != 0 {}
}

//! GPIO helper functions.

use crate::rp2040::pac;

bitflags! {
    pub struct GPIOFunc: u32 {
        const XIP = 0;
        const SPI = 1;
        const UART = 2;
        const I2C = 3;
        const PWM = 4;
        const SIO = 5;
        const PIO0 = 6;
        const PIO1 = 7;
        const GPCK = 8;
        const USB = 9;
        const NULL = 0xf;
    }
}

/// Sets the kind of the mentioned GPIO pin (if this PIN can support this function).
pub fn gpio_set_function(
    pads: &pac::PADS_BANK0,
    io: &pac::IO_BANK0,
    gpio_id: u32,
    function: GPIOFunc,
) {
    pads.gpio[gpio_id as usize].modify(|_, w| w.ie().set_bit().od().clear_bit());
    io.gpio[gpio_id as usize]
        .gpio_ctrl
        .write_with_zero(|x| unsafe { x.funcsel().bits(function.bits as _) });
}

//! Provides high-level structures for the PicoDisplay.

use crate::rp2040::gpio::{gpio_set_function, GPIOFunc};
use crate::rp2040::pac;
use crate::rp2040::pac::SIO;
use crate::rp2040::pwm::{
    pwm_gpio_to_slice_num, pwm_init, pwm_set_gpio_level, pwm_set_wrap, PWMConfig,
};
use crate::rp2040::spi::SPI;

use crate::st7789::ST7789;
use cortex_m::delay::Delay;

pub struct PicoDisplay {
    st7789: ST7789<pac::SPI0>,
    bl_gpio: u32,
    width: usize,
    height: usize,
}

impl PicoDisplay {
    /// Returns the width of this display instance.
    pub fn get_width(&self) -> usize {
        self.width
    }

    /// Returns the height of this display instance.
    pub fn get_height(&self) -> usize {
        self.height
    }

    /// Configures the screen's backlight as a factor between 0 and 255.
    pub fn set_display_backlight(&mut self, pwm: &pac::PWM, value: u8) {
        // Ported from https://github.com/pimoroni/pimoroni-pico/blob/main/drivers/st7789/st7789.cpp:
        // gamma correct the provided 0-255 brightness value onto a
        // 0-65535 range for the pwm counter
        let gamma = 2.032;
        let value = (value as f32) / 255.0f32;
        let value = unsafe { core::intrinsics::powf32(value, gamma) };
        let value = (value * 65535.0f32 + 0.5f32) as u16;

        pwm_set_gpio_level(pwm, self.bl_gpio, value);
    }

    /// Draws the screen using the specified buffer.
    pub fn draw(&mut self, sio: &SIO, buffer: &[u16]) {
        assert_eq!(buffer.len(), self.width * self.height);
        self.st7789.update_framebuffer(sio, buffer);
    }

    /// Creates a new customised PicoDisplay instance.
    pub fn new(
        sys_timer: &mut Delay,
        pwm: &pac::PWM,
        pads: &pac::PADS_BANK0,
        io: &pac::IO_BANK0,
        sio: &SIO,
        resets: &pac::RESETS,
        spi: pac::SPI0,
        cs: u32,
        dc: u32,
        bl: u32,
        sck: u32,
        mosi: u32,
        display_width: usize,
        display_height: usize,
    ) -> Self {
        let spi = SPI::new(spi, resets, 64 * 1024 * 1024);

        // Backlight
        let config = PWMConfig::default();
        pwm_set_wrap(pwm, pwm_gpio_to_slice_num(bl as _), 65535);
        pwm_init(pwm, pwm_gpio_to_slice_num(bl as _), &config, true);
        gpio_set_function(pads, io, bl, GPIOFunc::PWM);

        PicoDisplay {
            st7789: ST7789::new(sys_timer, pads, io, sio, dc, cs, sck, mosi, spi),
            bl_gpio: bl,
            width: display_width,
            height: display_height,
        }
    }

    /// Configures a screen based on the default Pimoroni PicoDisplay.
    pub fn default_pimoroni(
        sys_timer: &mut Delay,
        pwm: &pac::PWM,
        pads: &pac::PADS_BANK0,
        io: &pac::IO_BANK0,
        sio: &SIO,
        resets: &pac::RESETS,
        spi: pac::SPI0,
    ) -> Self {
        const CS: u32 = 17;
        const DC: u32 = 16;
        const BL: u32 = 20;
        const SCK: u32 = 18;
        const MOSI: u32 = 19;

        const DISPLAY_WIDTH: usize = 240;
        const DISPLAY_HEIGHT: usize = 135;

        Self::new(
            sys_timer,
            pwm,
            pads,
            io,
            sio,
            resets,
            spi,
            CS,
            DC,
            BL,
            SCK,
            MOSI,
            DISPLAY_WIDTH,
            DISPLAY_HEIGHT,
        )
    }
}

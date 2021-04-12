//! PWM helper functions.

use crate::rp2040::pac;

use core::ops::Not;

use paste::paste;

pub struct PWMConfig {
    csr: u32,
    div: u32,
    top: u32,
}

/*

#define PWM_CH0_CSR_PH_CORRECT_RESET  _u(0x0)
#define PWM_CH0_CSR_PH_CORRECT_BITS   _u(0x00000002)
#define PWM_CH0_CSR_PH_CORRECT_MSB    _u(1)
#define PWM_CH0_CSR_PH_CORRECT_LSB    _u(1)
#define PWM_CH0_CSR_PH_CORRECT_ACCESS "RW"
 */

const PWM_CH0_CSR_PH_CORRECT_BITS: u32 = 0x00000002;
const PWM_CH0_CSR_PH_CORRECT_LSB: u32 = 1;

const PWM_CH1_DIV_INT_LSB: u32 = 4;

const PWM_CH0_CSR_DIVMODE_BITS: u32 = 0x00000030;
const PWM_CH0_CSR_DIVMODE_LSB: u32 = 4;

const PWM_CH0_CSR_A_INV_BITS: u32 = 0x00000004;
const PWM_CH0_CSR_A_INV_LSB: u32 = 2;
const PWM_CH0_CSR_B_INV_BITS: u32 = 0x00000008;
const PWM_CH0_CSR_B_INV_LSB: u32 = 3;

const PWM_CH0_CSR_EN_LSB: u32 = 0;

const PWM_CH0_CC_RESET: u32 = 0;

const PWM_CH0_CTR_RESET: u32 = 0;

bitflags! {
    /// The kinds of division modes that the PWM can use.
    pub struct PWMClkdivMode: u32 {
        const PWM_DIV_FREE_RUNNING = 0;
        const PWM_DIV_B_HIGH = 1;
        const PWM_DIV_B_RISING = 2;
        const PWM_DIV_B_FALLING = 3;
    }
}

impl PWMConfig {
    pub fn set_phase_correct(&mut self, value: bool) {
        self.csr = (self.csr & PWM_CH0_CSR_PH_CORRECT_BITS.not())
            | (if value { 1 } else { 0 } << PWM_CH0_CSR_PH_CORRECT_LSB);
    }

    pub fn set_clkdiv_int(&mut self, value: u32) {
        self.div = value << PWM_CH1_DIV_INT_LSB;
    }

    pub fn set_clkdiv_mode(&mut self, value: PWMClkdivMode) {
        self.csr =
            (self.csr & PWM_CH0_CSR_DIVMODE_BITS.not()) | (value.bits << PWM_CH0_CSR_DIVMODE_LSB);
    }

    pub fn set_output_polarity(&mut self, a: bool, b: bool) {
        self.csr = (self.csr & (PWM_CH0_CSR_A_INV_BITS | PWM_CH0_CSR_B_INV_BITS).not())
            | (if a { 1 } else { 0 } << PWM_CH0_CSR_A_INV_LSB)
            | (if b { 1 } else { 0 } << PWM_CH0_CSR_B_INV_LSB);
    }

    pub fn set_wrap(&mut self, value: u32) {
        self.top = value;
    }
}

impl Default for PWMConfig {
    fn default() -> Self {
        let mut config = PWMConfig {
            csr: 0,
            div: 0,
            top: 0,
        };

        config.set_phase_correct(false);
        config.set_clkdiv_int(1);
        config.set_clkdiv_mode(PWMClkdivMode::PWM_DIV_FREE_RUNNING);
        config.set_output_polarity(false, false);
        config.set_wrap(0xffff);

        config
    }
}

pub fn pwm_gpio_to_slice_num(gpio: u32) -> u32 {
    (gpio >> 1) & 7
}

pub fn pwm_gpio_to_channel(gpio: u32) -> u32 {
    gpio & 1
}

macro_rules! pwm_caller {
    ($func_name:ident, $pac:ident, $match_value:expr, $call:ident, $value:expr) => {
        paste! {
            match $match_value {
                0 => $pac.[<ch0_ $func_name>].$call($value),
                1 => $pac.[<ch1_ $func_name>].$call($value),
                2 => $pac.[<ch2_ $func_name>].$call($value),
                3 => $pac.[<ch3_ $func_name>].$call($value),
                4 => $pac.[<ch4_ $func_name>].$call($value),
                5 => $pac.[<ch5_ $func_name>].$call($value),
                6 => $pac.[<ch6_ $func_name>].$call($value),
                7 => $pac.[<ch7_ $func_name>].$call($value),
                _ => unreachable!()
            }
        }
    };
}

pub fn pwm_set_wrap(pac: &pac::PWM, slice_num: u32, wrap: u16) {
    pwm_caller!(top, pac, slice_num, write_with_zero, |x| unsafe {
        x.bits(wrap as _)
    });
}

pub fn pwm_set_gpio_level(pac: &pac::PWM, gpio: u32, level: u16) {
    let slice_num = pwm_gpio_to_slice_num(gpio);
    let channel_num = pwm_gpio_to_channel(gpio);

    let pwm_shift = if channel_num != 0 { 16 } else { 0 };

    let level = (level as u32) << pwm_shift;

    let mask = if channel_num != 0 {
        0xFFFF0000u32
    } else {
        0x0000FFFFu32
    };

    pwm_caller!(cc, pac, slice_num, write, |x| unsafe {
        x.bits(level & mask)
    });
}

pub fn pwm_init(pac: &pac::PWM, slice: u32, config: &PWMConfig, start: bool) {
    pwm_caller!(csr, pac, slice, write_with_zero, |x| unsafe { x.bits(0) });
    pwm_caller!(ctr, pac, slice, write_with_zero, |x| unsafe {
        x.bits(PWM_CH0_CTR_RESET)
    });
    pwm_caller!(cc, pac, slice, write_with_zero, |x| unsafe {
        x.bits(PWM_CH0_CC_RESET)
    });
    pwm_caller!(top, pac, slice, write_with_zero, |x| unsafe {
        x.bits(config.top)
    });
    pwm_caller!(div, pac, slice, write_with_zero, |x| unsafe {
        x.bits(config.div)
    });
    pwm_caller!(csr, pac, slice, write_with_zero, |x| unsafe {
        x.bits(config.csr | if start { 1 << PWM_CH0_CSR_EN_LSB } else { 0 })
    });
}

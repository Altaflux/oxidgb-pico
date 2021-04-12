//! Performs basic clock initialisation, as based on the Pico-SDK:
//! https://github.com/raspberrypi/pico-sdk

use crate::rp2040::pac;

use paste::paste;

use core::ops::Not;

/// Pulls the PLLs into a RESET state.
fn reset_clocks(pac: &pac::Peripherals) {
    pac.RESETS
        .reset
        .modify(|_, x| x.pll_sys().set_bit().pll_usb().set_bit());
}

/// Unresets the clocks and waits for them to be ready.
fn unreset_clocks(pac: &pac::Peripherals) {
    pac.RESETS
        .reset
        .modify(|_, x| x.pll_sys().clear_bit().pll_usb().clear_bit());

    loop {
        let value = pac.RESETS.reset_done.read();
        if value.pll_sys().bit_is_set() && value.pll_usb().bit_is_set() {
            break;
        }
    }
}

const MHZ: u32 = 1000000;
const XOSC_MHZ: u32 = 12;

/// Setup the XOSC.
fn xosc_init(pac: &pac::Peripherals) {
    // Assumes 1-15 MHz input
    pac.XOSC.ctrl.write_with_zero(|x| x.freq_range()._1_15mhz());

    // Set xosc startup delay
    let startup_delay = (((12 * MHZ) / 1000) + 128) / 256;

    pac.XOSC
        .startup
        .write_with_zero(|x| unsafe { x.bits(startup_delay) });

    // Set the enable bit now that we have set freq range and startup delay
    pac.XOSC.ctrl.modify(|_, x| x.enable().enable());

    // Wait for XOSC to be stable
    while pac.XOSC.status.read().stable().bit_is_clear() {}
}

enum PLL {
    System,
    USB,
}

macro_rules! pll_call {
    ($pac: ident, $chosen_device:expr, $device:ident, $method:ident) => {
        match $chosen_device {
            PLL::System => $pac.PLL_SYS.$device.$method(),
            PLL::USB => $pac.PLL_USB.$device.$method(),
        }
    };
    ($pac: ident, $chosen_device:expr, $device:ident, $method:ident, $value:expr) => {
        match $chosen_device {
            PLL::System => $pac.PLL_SYS.$device.$method($value),
            PLL::USB => $pac.PLL_USB.$device.$method($value),
        }
    };
}

/// Initialises a PLL - note that a reset should have occurred before this.
fn pll_init(
    pac: &pac::Peripherals,
    pll: PLL,
    refdiv: u32,
    vco_freq: u32,
    post_div1: u8,
    post_div2: u8,
) {
    pll_call!(pac, &pll, pwr, write, |x| unsafe { x.bits(0xffffffff) });
    pll_call!(pac, &pll, fbdiv_int, write_with_zero, |x| x);

    let ref_mhz = XOSC_MHZ / refdiv;
    let fbdiv = vco_freq / (ref_mhz * MHZ);

    pll_call!(pac, &pll, fbdiv_int, write_with_zero, |x| unsafe {
        x.bits(fbdiv)
    });
    pll_call!(pac, &pll, pwr, modify, |_, x| x
        .pd()
        .clear_bit()
        .vcopd()
        .clear_bit());

    while pll_call!(pac, &pll, cs, read).lock().bit_is_clear() {}

    pll_call!(pac, &pll, prim, write_with_zero, |x| unsafe {
        x.postdiv1().bits(post_div1).postdiv2().bits(post_div2)
    });

    pll_call!(pac, &pll, pwr, modify, |_, x| x.postdivpd().clear_bit());
}

enum ClockIndex {
    Reference,
    System,
    USB,
    ADC,
    RTC,
    Peripheral,
}

impl ClockIndex {
    fn has_glitchless_mux(&self) -> bool {
        match self {
            ClockIndex::Reference => true,
            ClockIndex::System => true,
            _ => false,
        }
    }
}

macro_rules! clock_caller {
    ($pac:ident, $clock_name:expr, $func_name:ident, $call:ident) => {
        paste! {
            match $clock_name {
                ClockIndex::Reference => $pac.CLOCKS.[<clk_ref_ $func_name>].$call().bits(),
                ClockIndex::System => $pac.CLOCKS.[<clk_sys_ $func_name>].$call().bits(),
                ClockIndex::USB => $pac.CLOCKS.[<clk_usb_ $func_name>].$call().bits(),
                ClockIndex::ADC => $pac.CLOCKS.[<clk_adc_ $func_name>].$call().bits(),
                ClockIndex::RTC => $pac.CLOCKS.[<clk_rtc_ $func_name>].$call().bits(),
                ClockIndex::Peripheral => $pac.CLOCKS.[<clk_peri_ $func_name>].$call().bits()
            }
        }
    };
    ($pac:ident, $clock_name:expr, $func_name:ident, $call:ident, $value:expr) => {
        paste! {
            match $clock_name {
                ClockIndex::Reference => $pac.CLOCKS.[<clk_ref_ $func_name>].$call($value),
                ClockIndex::System => $pac.CLOCKS.[<clk_sys_ $func_name>].$call($value),
                ClockIndex::USB => $pac.CLOCKS.[<clk_usb_ $func_name>].$call($value),
                ClockIndex::ADC => $pac.CLOCKS.[<clk_adc_ $func_name>].$call($value),
                ClockIndex::RTC => $pac.CLOCKS.[<clk_rtc_ $func_name>].$call($value),
                ClockIndex::Peripheral => $pac.CLOCKS.[<clk_peri_ $func_name>].$call($value)
            }
        }
    };
}

macro_rules! clock_caller_not_peripheral {
    ($pac:ident, $clock_name:expr, $func_name:ident, $call:ident) => {
        paste! {
            match $clock_name {
                ClockIndex::Reference => $pac.CLOCKS.[<clk_ref_ $func_name>].$call().bits(),
                ClockIndex::System => $pac.CLOCKS.[<clk_sys_ $func_name>].$call().bits(),
                ClockIndex::USB => $pac.CLOCKS.[<clk_usb_ $func_name>].$call().bits(),
                ClockIndex::ADC => $pac.CLOCKS.[<clk_adc_ $func_name>].$call().bits(),
                ClockIndex::RTC => $pac.CLOCKS.[<clk_rtc_ $func_name>].$call().bits(),
                ClockIndex::Peripheral => 0
            }
        }
    };
    ($pac:ident, $clock_name:expr, $func_name:ident, $call:ident, $value:expr) => {
        paste! {
            match $clock_name {
                ClockIndex::Reference => $pac.CLOCKS.[<clk_ref_ $func_name>].$call($value),
                ClockIndex::System => $pac.CLOCKS.[<clk_sys_ $func_name>].$call($value),
                ClockIndex::USB => $pac.CLOCKS.[<clk_usb_ $func_name>].$call($value),
                ClockIndex::ADC => $pac.CLOCKS.[<clk_adc_ $func_name>].$call($value),
                ClockIndex::RTC => $pac.CLOCKS.[<clk_rtc_ $func_name>].$call($value),
                ClockIndex::Peripheral => {}
            }
        }
    };
}

/// Configures a particular clock - note that clocks should have been reset before this.
fn clock_configure(
    pac: &pac::Peripherals,
    clock: ClockIndex,
    src: u8,
    auxsrc: u8,
    src_freq: u32,
    freq: u32,
) -> bool {
    if freq > src_freq {
        return false;
    }

    let div = ((src_freq as u64) << 8) / freq as u64;

    if div > clock_caller_not_peripheral!(pac, &clock, div, read) as u64 {
        clock_caller_not_peripheral!(pac, &clock, div, write_with_zero, |x| unsafe {
            x.bits(div as u32)
        });
    }

    const CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX: u8 = 1;

    const CLOCKS_CLK_GPOUT0_CTRL_ENABLE_BITS: u32 = 0x00000800;

    // Avoid turning off the main CPU reference clock if we are modifying that
    if clock.has_glitchless_mux() && src == CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX {
        clock_caller!(pac, &clock, ctrl, modify, |r, x| unsafe {
            x.bits(r.bits() & CLOCKS_CLK_GPOUT0_CTRL_ENABLE_BITS.not())
        });

        while (clock_caller!(pac, &clock, selected, read) & 1) == 0 {}
    } else {
        clock_caller!(pac, &clock, ctrl, write_with_zero, |x| unsafe { x.bits(0) });
        // We don't store clock information - just loop for a little while
        // and then move on
        cortex_m::asm::delay(100 * 3);
    }

    clock_caller!(pac, &clock, ctrl, modify, |_, x| unsafe {
        x.auxsrc().bits(auxsrc)
    });

    if clock.has_glitchless_mux() {
        const CLOCKS_CLK_REF_CTRL_SRC_BITS: u32 = 0x03;
        const CLOCKS_CLK_REF_CTRL_SRC_LSB: u32 = 0;

        clock_caller!(pac, &clock, ctrl, modify, |r, x| unsafe {
            x.bits(
                (r.bits() & CLOCKS_CLK_REF_CTRL_SRC_BITS.not())
                    | (((src as u32) << CLOCKS_CLK_REF_CTRL_SRC_LSB)
                        & CLOCKS_CLK_REF_CTRL_SRC_BITS),
            )
        });

        while (clock_caller!(pac, &clock, selected, read) & (1 << src)) == 0 {}
    }

    clock_caller!(pac, &clock, ctrl, modify, |r, x| unsafe {
        x.bits(r.bits() | CLOCKS_CLK_GPOUT0_CTRL_ENABLE_BITS)
    });

    clock_caller_not_peripheral!(pac, &clock, div, write_with_zero, |x| unsafe {
        x.bits(div as u32)
    });

    true
}

/// Initialises all clocks and PLLs to their default/stock state.
/// See `main.rs` for a description of the overclocking interface.
pub(crate) fn init_clocks(pac: &pac::Peripherals, do_overclock: bool, overclock_multiplier: u32) {
    // From https://github.com/raspberrypi/pico-sdk/blob/02b98e5476abc1788bb07be54aead72220c7fdd1/src/rp2_common/hardware_clocks/clocks.c

    let overclock_profile = if do_overclock {
        overclock_multiplier
    } else {
        // Default, stock multiplier for the PLL.
        15
    };

    // TODO: Watchdog?

    // Disable resus that may be enabled from previous software
    pac.CLOCKS
        .clk_sys_resus_ctrl
        .write_with_zero(|x| x.enable().clear_bit());

    // Enable the XOSC
    xosc_init(pac);

    // Before we touch PLLs, switch sys and ref cleanly away from their aux sources.
    // Note that src == 1 is the aux here!
    pac.CLOCKS.clk_sys_ctrl.modify(|_, x| x.src().clear_bit());
    while pac.CLOCKS.clk_sys_selected.read().bits() != 0x1 {}
    pac.CLOCKS
        .clk_ref_ctrl
        .modify(|_, x| unsafe { x.src().bits(0) });
    while pac.CLOCKS.clk_ref_selected.read().bits() != 0x1 {}

    reset_clocks(pac);
    unreset_clocks(pac);

    // Init plls
    pll_init(pac, PLL::System, 1, overclock_profile * 100 * MHZ, 6, 2);
    pll_init(pac, PLL::USB, 1, 480 * MHZ, 5, 2);

    const CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC: u8 = 0x2;

    const CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS: u8 = 0x0;

    const CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX: u8 = 0x1;

    const CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS: u8 = 0x0;
    const CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB: u8 = 0x0;
    const CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB: u8 = 0x0;
    const CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB: u8 = 0x0;

    // Configure clocks
    clock_configure(
        pac,
        ClockIndex::Reference,
        CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC,
        0,
        12 * MHZ,
        12 * MHZ,
    );

    clock_configure(
        pac,
        ClockIndex::System,
        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
        125 * MHZ,
        125 * MHZ,
    );

    clock_configure(
        pac,
        ClockIndex::USB,
        0,
        CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
        48 * MHZ,
        48 * MHZ,
    );

    clock_configure(
        pac,
        ClockIndex::ADC,
        0,
        CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
        48 * MHZ,
        48 * MHZ,
    );

    clock_configure(
        pac,
        ClockIndex::RTC,
        0,
        CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
        48 * MHZ,
        46875,
    );

    clock_configure(
        pac,
        ClockIndex::Peripheral,
        0,
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
        125 * MHZ,
        125 * MHZ,
    );
}

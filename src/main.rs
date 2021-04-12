//! A GameBoy emulator for the Raspberry Pi Pico...

#![no_std]
#![no_main]
#![feature(core_intrinsics)]
#![feature(default_alloc_error_handler)]

// ==========================================
// CONFIGURATION SECTION
// ==========================================

/// Overclocking switch - disabling this will only use stock clocks for talking to
/// peripherals.
///
/// While this should be safe (this doesn't tweak voltage, just the core PLL), I obviously
/// am **not liable** for breaking your Pico.
const DO_OVERCLOCK: bool = true;
/// The PLL multiplier for any overclock applied.
/// Stock is a multiplier of 15 (which is what is applied when `DO_OVERCLOCK` is set to false).
const OVERCLOCK_MULTIPLIER: u32 = 30;
/// Path to your game data, relative to the `src/` directory.
const GAME_DATA: &'static [u8] = include_bytes!("../pokemon.gb");

// ==========================================
// RUNTIME SECTION
// ==========================================

#[macro_use]
extern crate bitflags;

extern crate rp2040_panic_usb_boot;

#[macro_use]
extern crate alloc;

mod pico_display;
mod rp2040;
mod st7789;

use cortex_m_rt::entry;

use alloc_cortex_m::CortexMHeap;
use cortex_m::prelude::*;

use core::cmp::min;

use embedded_graphics::fonts::Text;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::style::{PrimitiveStyle, TextStyle};

use oxidgb_core::cpu::CPU;
use oxidgb_core::gpu::PITCH;
use oxidgb_core::mem::GBMemory;
use oxidgb_core::rom::GameROM;

use crate::pico_display::PicoDisplay;

#[link_section = ".boot_loader"]
#[used]
pub static BOOT_LOADER: [u8; 256] = rp2040_boot2::BOOT_LOADER;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

/// High-level object designed to store an RGB565 buffer and provide a
/// stable interface to it regardless of what craziness is going on under
/// the hood.
struct PicoScreen<'a> {
    framebuffer: &'a mut [u16],
    width: usize,
    height: usize,
}

// Interface for embedded-graphics.
impl<'a> DrawTarget<Rgb565> for PicoScreen<'a> {
    type Error = core::convert::Infallible;

    fn draw_pixel(&mut self, pixel: Pixel<Rgb565>) -> Result<(), Self::Error> {
        let Pixel(Point { x, y }, color) = pixel;
        if !(0..self.width).contains(&(x as usize)) || !(0..self.height).contains(&(y as usize)) {
            return Ok(());
        }

        self.framebuffer[y as usize * self.width + x as usize] = color.into_storage();

        Ok(())
    }

    fn size(&self) -> Size {
        Size::new(self.width as u32, self.height as u32)
    }
}

#[no_mangle]
#[entry]
fn main() -> ! {
    // Setup the allocator with a conservative heap.
    let start = cortex_m_rt::heap_start() as usize;
    let size = 200 * 1024; // in bytes
    unsafe { ALLOCATOR.init(start, size) }

    let mut pac = rp2040_pac::Peripherals::take().unwrap();
    let cp = rp2040_pac::CorePeripherals::take().unwrap();

    // Before we touch any peripherals, we need to setup clocks and so forth. Do this now:
    rp2040::setup_chip(&mut pac, DO_OVERCLOCK, OVERCLOCK_MULTIPLIER);

    let mut delay = cortex_m::delay::Delay::new(cp.SYST, 125_000_000);

    let mut display = PicoDisplay::default_pimoroni(
        &mut delay,
        &pac.PWM,
        &pac.PADS_BANK0,
        &pac.IO_BANK0,
        &pac.SIO,
        &pac.RESETS,
        pac.SPI0,
    );

    display.set_display_backlight(&pac.PWM, 100);

    let rom = GameROM::build(GAME_DATA);

    let mut framebuffer = vec![0u16; display.get_width() * display.get_height()];

    let mut screen = PicoScreen {
        framebuffer: &mut framebuffer,
        width: display.get_width(),
        height: display.get_height(),
    };

    // Draw a splash screen:
    let fill = PrimitiveStyle::with_fill(Rgb565::new(50, 50, 180));
    let text_style = TextStyle::new(profont::ProFont24Point, Rgb565::WHITE);
    let small_text_style = TextStyle::new(profont::ProFont10Point, Rgb565::WHITE);

    let y_offset = screen.height as i32 / 2;

    // Text background:
    Rectangle::new(
        Point::new(0, y_offset - 18),
        Point::new(screen.width as _, y_offset + 18),
    )
    .into_styled(fill)
    .draw(&mut screen)
    .unwrap();

    let text = "Oxidgb";
    let width = text.len() as i32 * 16;
    Text::new(
        text,
        Point::new(
            screen.width as i32 / 2 - width / 2,
            screen.height as i32 / 2 - 18,
        ),
    )
    .into_styled(text_style)
    .draw(&mut screen)
    .unwrap();

    let text = format!("Booting \"{}\"...", rom.get_cart_name());
    let width = text.len() as i32 * 7;
    Text::new(
        &text,
        Point::new(
            screen.width as i32 / 2 - width / 2,
            screen.height as i32 / 2 + 50,
        ),
    )
    .into_styled(small_text_style)
    .draw(&mut screen)
    .unwrap();

    display.draw(&pac.SIO, &screen.framebuffer);

    // Build memory
    let memory = GBMemory::build(rom);

    // Build CPU
    let mut cpu = CPU::build(memory);

    // Update the screen now everything is ready:
    let text = format!("Used RAM: {}, free: {}", ALLOCATOR.used(), ALLOCATOR.free());
    let width = text.len() as i32 * 7;
    Text::new(
        &text,
        Point::new(
            screen.width as i32 / 2 - width / 2,
            screen.height as i32 / 2 - 65,
        ),
    )
    .into_styled(small_text_style)
    .draw(&mut screen)
    .unwrap();

    let text = format!(
        "Used flash: {}",
        (GAME_DATA.as_ptr() as usize) + GAME_DATA.len() - 0x10000000
    );
    let width = text.len() as i32 * 7;
    Text::new(
        &text,
        Point::new(
            screen.width as i32 / 2 - width / 2,
            screen.height as i32 / 2 - 55,
        ),
    )
    .into_styled(small_text_style)
    .draw(&mut screen)
    .unwrap();

    display.draw(&pac.SIO, &screen.framebuffer);

    // We use the same buffer for the gameboy screen - clear it now:
    embedded_graphics::DrawTarget::clear(&mut screen, Rgb565::BLACK).unwrap();

    delay.delay_ms(2000);

    loop {
        cpu.run();

        let x_max = min(display.get_width(), 160);
        let y_max = min(display.get_height(), 144);

        let y_offset = (display.get_height() - y_max) / 2;
        let x_offset = (display.get_width() - x_max) / 2;

        // TODO: This isn't particularly efficient - change how the runtime works?
        //       (generic framebuffer interface?)
        for y in 0..y_max {
            for x in 0..x_max {
                let offset = (y * 160 + x) * PITCH;

                let packed_rgb = ((cpu.mem.gpu.pixel_data[offset] as u16 & 0b11111000) << 8)
                    | ((cpu.mem.gpu.pixel_data[offset + 1] as u16 & 0b11111100) << 3)
                    | (cpu.mem.gpu.pixel_data[offset + 2] as u16 >> 3);
                screen.framebuffer[(y + y_offset) * display.get_width() + (x + x_offset)] =
                    packed_rgb;
            }
        }

        // TODO: Get DMA working for this
        display.draw(&pac.SIO, &screen.framebuffer);
    }
}

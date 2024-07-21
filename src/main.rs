#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod wifi;

use display_interface_parallel_gpio::{Generic8BitBus, PGPIO8BitInterface};
use embassy_executor::Spawner;
use embassy_net::{Config, Stack, StackResources};
use embassy_time::{Duration, Timer};
use embedded_graphics::framebuffer::Framebuffer;
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::pixelcolor::raw::BigEndian;
use embedded_graphics::pixelcolor::{BinaryColor, Rgb565};
use embedded_graphics::prelude::*;
use embedded_graphics::text::{Alignment, Text};
use embedded_graphics::{draw_target::DrawTarget, prelude::RgbColor};
use esp_backtrace as _;
use esp_hal::timer::{ErasedTimer, OneShotTimer, PeriodicTimer};
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{Io, Level, Output},
    peripherals::Peripherals,
    prelude::*,
    rtc_cntl::Rtc,
    system::SystemControl,
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_wifi::initialize;
use esp_wifi::wifi::{WifiDevice, WifiStaDevice};
use mipidsi::options::{ColorInversion, ColorOrder, Orientation, Rotation};
use mipidsi::{models::ST7789, Builder};

extern crate alloc;

use core::mem::MaybeUninit;

use crate::wifi::wifi_task;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[main]
async fn main(spawner: Spawner) -> ! {
    init_heap();

    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.LPWR, None);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks, None);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks, None);
    let mut wdt1 = timer_group1.wdt;
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    esp_println::logger::init_logger_from_env();

    let timer = PeriodicTimer::new(timer_group1.timer0.into());

    // Set GPIO4 as an output, and set its state high initially.
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut lcd_power_on = Output::new(io.pins.gpio15, Level::High);
    lcd_power_on.set_high();
    let _button = io.pins.gpio21;

    // Initialize the Delay peripheral, and use it to toggle the LED state in a
    // loop.
    let mut delay = Delay::new(&clocks);

    println!("init display");

    let rst = Output::new(io.pins.gpio5, Level::Low);
    let dc = Output::new(io.pins.gpio7, Level::Low);
    let wr = Output::new(io.pins.gpio8, Level::Low);
    let _cs = Output::new(io.pins.gpio6, Level::Low);
    let _re = Output::new(io.pins.gpio9, Level::High);
    let _backlight = Output::new(io.pins.gpio38, Level::High);

    let d0 = Output::new(io.pins.gpio39, Level::Low);
    let d1 = Output::new(io.pins.gpio40, Level::Low);
    let d2 = Output::new(io.pins.gpio41, Level::Low);
    let d3 = Output::new(io.pins.gpio42, Level::Low);
    let d4 = Output::new(io.pins.gpio45, Level::Low);
    let d5 = Output::new(io.pins.gpio46, Level::Low);
    let d6 = Output::new(io.pins.gpio47, Level::Low);
    let d7 = Output::new(io.pins.gpio48, Level::Low);

    let bus = Generic8BitBus::new((d0, d1, d2, d3, d4, d5, d6, d7));

    let display_interface = PGPIO8BitInterface::new(bus, dc, wr);

    let mut display = Builder::new(ST7789, display_interface)
        .reset_pin(rst)
        .display_size(170, 320)
        .color_order(ColorOrder::Rgb)
        .invert_colors(ColorInversion::Inverted)
        .orientation(Orientation::new().rotate(Rotation::Deg90))
        .init(&mut delay)
        .unwrap();
    display.clear(Rgb565::RED).unwrap();
    println!("display init ok");

    println!("init embassy");

    let timer0 = OneShotTimer::new(timer_group0.timer0.into());
    let timers = [timer0];
    let timers = mk_static!([OneShotTimer<ErasedTimer>; 1], timers);

    esp_hal_embassy::init(&clocks, timers);

    println!("init wifi");

    let init = initialize(
        esp_wifi::EspWifiInitFor::Wifi,
        timer,
        esp_hal::rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
        &clocks,
    )
    .unwrap();

    let wifi = peripherals.WIFI;
    let (wifi_interface, controller) =
        esp_wifi::wifi::new_with_mode(&init, wifi, WifiStaDevice).unwrap();
    let config = Config::dhcpv4(Default::default());

    let seed = 1234;

    println!("init network stack");

    let stack = &*mk_static!(
        Stack<WifiDevice<'_, WifiStaDevice>>,
        Stack::new(
            wifi_interface,
            config,
            mk_static!(StackResources<3>, StackResources::<3>::new()),
            seed
        )
    );

    spawner.spawn(wifi::connection(controller)).ok();
    spawner.spawn(wifi_task(stack)).ok();

    loop {
        if stack.is_link_up() {
            break;
        }
        println!("Waiting for IP...");
        Timer::after(Duration::from_millis(500)).await;
    }

    let mut fb = Framebuffer::<
        Rgb565,
        _,
        BigEndian,
        320,
        170,
        { embedded_graphics::framebuffer::buffer_size::<Rgb565>(320, 170) },
    >::new();
    fb.clear(Rgb565::WHITE).unwrap();

    let gif = tinygif::Gif::from_slice(include_bytes!("../ferris.gif")).unwrap();

    //     Text::with_alignment(
    //         &"Test Text",
    //         Point::new(100, 40),
    //         MonoTextStyleBuilder::new()
    //             .background_color(Rgb565::BLACK)
    //             .text_color(Rgb565::CSS_BISQUE)
    //             .font(&FONT_10X20)
    //             .build(),
    //         Alignment::Center,
    //     )
    //     .draw(&mut display)
    //     .unwrap();
    //     delay.delay_ms(1000u32);

    loop {
        for frame in gif.frames() {
            frame.draw(&mut fb.translated(Point::new(0, 10))).unwrap();

            fb.as_image().draw(&mut display).unwrap();
        }
    }
}

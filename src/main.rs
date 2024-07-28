#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod wifi;

use alloc::sync::Arc;
use core::fmt::Write;
use display_interface_parallel_gpio::{Generic8BitBus, PGPIO8BitInterface};
use embassy_executor::Spawner;
use embassy_net::tcp::client::{TcpClient, TcpClientState};
use embassy_net::{dns::DnsSocket, Config, Stack, StackResources};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Ticker, Timer};
use embedded_graphics::mono_font::iso_8859_7::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::pixelcolor::Rgb565;
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
use mipidsi::Display;
use mipidsi::{models::ST7789, Builder};
use reqwless::client::HttpClient;
use reqwless::request::Method;

extern crate alloc;

use core::borrow::BorrowMut;
use core::mem::MaybeUninit;

use crate::wifi::wifi_task;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

type SharedData = Arc<Mutex<NoopRawMutex, Option<Data>>>;

type DI = PGPIO8BitInterface<
    Generic8BitBus<
        Output<'static, esp_hal::gpio::GpioPin<39>>,
        Output<'static, esp_hal::gpio::GpioPin<40>>,
        Output<'static, esp_hal::gpio::GpioPin<41>>,
        Output<'static, esp_hal::gpio::GpioPin<42>>,
        Output<'static, esp_hal::gpio::GpioPin<45>>,
        Output<'static, esp_hal::gpio::GpioPin<46>>,
        Output<'static, esp_hal::gpio::GpioPin<47>>,
        Output<'static, esp_hal::gpio::GpioPin<48>>,
    >,
    Output<'static, esp_hal::gpio::GpioPin<7>>,
    Output<'static, esp_hal::gpio::GpioPin<8>>,
>;

type DISPLAY = Display<DI, ST7789, Output<'static, esp_hal::gpio::GpioPin<5>>>;

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
async fn main(spawner: Spawner) {
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

    let mut display: DISPLAY = Builder::new(ST7789, display_interface)
        .reset_pin(rst)
        .display_size(170, 320)
        .display_offset(35, 0)
        .color_order(ColorOrder::Rgb)
        .invert_colors(ColorInversion::Inverted)
        .orientation(Orientation::new().rotate(Rotation::Deg90))
        .init(&mut delay)
        .unwrap();

    display.clear(Rgb565::RED).unwrap();
    println!("display init ok");

    display.clear(Rgb565::BLUE).unwrap();
    let data: SharedData = Arc::new(Mutex::new(None));

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
    spawner.spawn(load_data(stack, data.clone())).ok();
    spawner.spawn(render_task(data.clone(), display)).ok();

    // let mut fb = Framebuffer::<
    //     Rgb565,
    //     _,
    //     BigEndian,
    //     320,
    //     170,
    //     { embedded_graphics::framebuffer::buffer_size::<Rgb565>(320, 170) },
    // >::new();
    // fb.clear(Rgb565::WHITE).unwrap();

    // let gif = tinygif::Gif::from_slice(include_bytes!("../ferris.gif")).unwrap();

    // delay.delay_ms(1000u32);

    //     loop {
    //         for frame in gif.frames() {
    //             frame.draw(&mut fb.translated(Point::new(0, 10))).unwrap();

    //             fb.as_image().draw(&mut display).unwrap();
    //         }
    //     }
}
use alloc::{
    string::{String, ToString},
    vec::Vec,
};
use serde::{Deserialize, Serialize};

#[derive(Deserialize, Serialize, Debug, Clone)]
pub struct Data {
    datetime: String,
    wind_direction: String,
    wind_status: String,
    wind_speed: f64,
    wave_direction: Option<String>,
    wave_period: Option<i32>,
    wave_height: Option<f64>,
    spot_name: String,
    air_temperature: i32,
}

#[embassy_executor::task]
async fn load_data(
    stack: &'static Stack<WifiDevice<'static, WifiStaDevice>>,
    shared_data: SharedData,
) {
    let mut rx_buffer = [0; 2048];
    let client_state = TcpClientState::<1, 2048, 2048>::new();
    let tcp_client = TcpClient::new(stack, &client_state);
    let dns = DnsSocket::new(stack);

    let mut ticker = Ticker::every(Duration::from_secs(10));

    loop {
        ticker.next().await;

        println!("Fetching weather data");

        stack.wait_config_up().await;

        loop {
            if let Some(config) = stack.config_v4() {
                println!("Got IP: {}", config.address);
                break;
            }
            Timer::after(Duration::from_millis(500)).await;
        }

        let mut http_client = HttpClient::new(&tcp_client, &dns);
        let mut request = http_client
            .request(Method::GET, "http://192.168.1.10:3000")
            .await
            .unwrap();

        let response = request.send(&mut rx_buffer).await.unwrap();

        if let Ok(body) = response.body().read_to_end().await {
            let data: Data = serde_json::from_slice(body).unwrap();
            println!("Just Received: {:?}", data);

            let mut shared_data_lock = shared_data.lock().await;
            let shared_data_ref = shared_data_lock.borrow_mut();

            **shared_data_ref = Some(data);
        }
    }
}

#[embassy_executor::task]
async fn render_task(shared_data: SharedData, mut display: DISPLAY) {
    let mut ticker = Ticker::every(Duration::from_millis(10));

    let style_0 = MonoTextStyleBuilder::new()
        // .background_color(Rgb565::BLACK)
        .text_color(Rgb565::CSS_BISQUE)
        .font(&FONT_10X20)
        .build();

    loop {
        ticker.next().await;

        if let Some(data) = shared_data.lock().await.clone() {
            display.clear(Rgb565::BLUE).unwrap();
            println!("Data do display: {:?}", data);

            let mut spot_name = heapless::String::<40>::new();
            write!(spot_name, "Spot Name: {}", data.spot_name).unwrap();

            Text::with_alignment(&spot_name, Point::new(20, 20), style_0, Alignment::Left)
                .draw(&mut display)
                .unwrap();

            let mut wind_speed = heapless::String::<40>::new();
            write!(wind_speed, "Wind Speed: {}kts", data.wind_speed).unwrap();

            let mut air_temperature = heapless::String::<30>::new();
            write!(air_temperature, "Air Temperature: {}°C", data.air_temperature).unwrap();

            let mut wave_height = heapless::String::<20>::new();
            write!(
                wave_height,
                "Wave Height: {}m",
                data.wave_height.unwrap_or(0.0)
            )
            .unwrap();

            Text::with_alignment(&wind_speed, Point::new(20, 40), style_0, Alignment::Left)
                .draw(&mut display)
                .unwrap();

            Text::with_alignment(&wave_height, Point::new(20, 60), style_0, Alignment::Left)
                .draw(&mut display)
                .unwrap();

            Text::with_alignment(
                &air_temperature,
                Point::new(20, 80),
                style_0,
                Alignment::Left,
            )
            .draw(&mut display)
            .unwrap();

            Text::with_alignment(
                &data.datetime,
                Point::new(40, 120),
                style_0,
                Alignment::Left,
            )
            .draw(&mut display)
            .unwrap();

            break;
        }

        Text::with_alignment(
            "NO DATA",
            Point::new(320 / 2, 170 / 2),
            style_0,
            Alignment::Center,
        )
        .draw(&mut display)
        .unwrap();
    }
}

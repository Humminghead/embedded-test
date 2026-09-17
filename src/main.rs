#![no_std]
#![no_main]

use crate::radio::si47xx::{self, is_bus_cts, PowerUpArg};

use core::fmt::Write;
use defmt::{error, info};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::peripherals::I2C2;
use embassy_stm32::{bind_interrupts, dma, peripherals};
use embassy_time::Timer;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use embedded_hal::digital::OutputPin;
use embedded_hal_bus::{i2c::AtomicDevice, util::AtomicCell};
use heapless::String;
use panic_probe as _;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
mod radio;

bind_interrupts!(
    struct Irqs {
        I2C2_EV => embassy_stm32::i2c::EventInterruptHandler<I2C2>;
        I2C2_ER => embassy_stm32::i2c::ErrorInterruptHandler<I2C2>;
        DMA1_CHANNEL4 => dma::InterruptHandler<peripherals::DMA1_CH4>;
        DMA1_CHANNEL5 => dma::InterruptHandler<peripherals::DMA1_CH5>;
    }
);

static I2C_ADDR_SEN_1: u8 = 0b0110_0011; // 0x63 (SEN pin high)

static BLINK_LONG: (i32, i32) = (500, 500);
static BLINK_SHORT: (i32, i32) = (250, 250);

static CODE_RESET_ERR: [(i32, i32); 2] = [BLINK_SHORT, BLINK_SHORT]; // 2
static CODE_IIC_CTS_TIMEOUT_ERR: [(i32, i32); 2] = [BLINK_LONG, BLINK_SHORT]; //11
static CODE_IIC_INVALID_ARG_ERR: [(i32, i32); 3] = [BLINK_LONG, BLINK_SHORT, BLINK_SHORT]; //12
static CODE_IIC_STCINT_ERR: [(i32, i32); 4] = [BLINK_LONG, BLINK_SHORT, BLINK_SHORT, BLINK_SHORT]; //13
static CODE_FM_NO_STATION_FOUND: [(i32, i32); 2] = [BLINK_LONG, BLINK_LONG]; //20

static RESTART_TIME_SEC: u64 = 2;

// ARG1 of POWER_UP in FM receive mode.
// A 32.768 kHz crystal is populated on RCLK/GPO3 => XOSCEN = 1.
// AN332 page 65 note: for Si474x it says "Set to 0" but that note only applies
// when an *external* clock source feeds RCLK; with a passive crystal XOSCEN must be 1.
const POWER_UP_ARG1_FM: u8 =
    (PowerUpArg::CTSIEN.bits() | PowerUpArg::GPO2OEN.bits() | PowerUpArg::XOSCEN.bits()) as u8;

// FM band, in 10 kHz units. 8750 = 87.5 MHz, 10800 = 108.0 MHz.
const FM_BAND_LOW: u16 = 8750;
const FM_BAND_HIGH: u16 = 10800;
const FM_STEP_10KHZ: u16 = 10; // 100 kHz

// RSSI threshold (dBuV) to declare a valid station and stop the scan.
// Default value is 20 dBµV.
// AN332 page 58 (FM_SEEK_TUNE_RSSI_TRESHOLD)
const FM_RSSI_LOCK_THRESHOLD: u8 = 20;

// FM_TUNE_FREQ: tSTC ≈ 60–80 ms on FMRX 4.0 (AN332 Table 49).
// 40 attempts × 5 ms = 200 ms budget.
const STC_POLL_MS: u64 = 5;
const STC_MAX_ATTEMPTS: u32 = 40;

async fn flash_signal<P: OutputPin>(pin: &mut P, signal: &[(i32, i32)]) {
    for &(h_ms, l_ms) in signal {
        let _ = pin.set_high();
        Timer::after_millis(h_ms as u64).await;
        let _ = pin.set_low();
        Timer::after_millis(l_ms as u64).await;
    }
    let _ = pin.set_high();
}

async fn reset_i2c_device<P: OutputPin>(pin: &mut P) -> bool {
    if pin.set_low().is_err() {
        return false;
    }
    Timer::after_millis(250).await;
    if pin.set_high().is_err() {
        return false;
    }
    Timer::after_millis(250).await;
    true
}

async fn display_reset<P: OutputPin>(pin: &mut P) -> Result<(), P::Error> {
    pin.set_low()?;
    Timer::after_millis(250).await;

    pin.set_high()
}

async fn error_loop<P: OutputPin>(pin: &mut P, sig: &[(i32, i32)]) -> ! {
    loop {
        Timer::after_secs(RESTART_TIME_SEC).await;
        flash_signal(pin, sig).await;
    }
}

#[embassy_executor::main]
async fn main(_s: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let mut dev_rst_pin = Output::new(p.PB1, Level::Low, Speed::Low);
    let mut display_rst_pin = Output::new(p.PB0, Level::Low, Speed::Low);
    let mut led_pin = Output::new(p.PC13, Level::High, Speed::Low);

    // Create I2C bus
    let i2c = I2c::new(
        p.I2C2,
        p.PB10,
        p.PB11,
        p.DMA1_CH4,
        p.DMA1_CH5,
        Irqs,
        Default::default(),
    );
    let i2c_bus = AtomicCell::new(i2c);

    // Give the radio a view of the bus
    let radio_bus = AtomicDevice::new(&i2c_bus);
    let mut device = si47xx::FmReceiver::new(radio_bus, I2C_ADDR_SEN_1);

    // Give the display a view of the bus
    let display_bus = AtomicDevice::new(&i2c_bus);
    let interface = I2CDisplayInterface::new(display_bus);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    let text_style: embedded_graphics::mono_font::MonoTextStyle<'_, BinaryColor> =
        MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(BinaryColor::On)
            .build();

    // Init the display
    display_reset(&mut display_rst_pin).await;
    Timer::after_millis(250).await;

    match display.init() {
        Ok(_) => {}
        Err(_e) => {
            error_loop(
                &mut led_pin,
                [BLINK_SHORT, BLINK_SHORT, BLINK_SHORT].as_slice(),
            )
            .await;
        }
    }

    Text::with_baseline(
        "PowerUP the radio!",
        Point::new(0, 16),
        text_style,
        Baseline::Top,
    )
    .draw(&mut display)
    .unwrap();

    display.flush().unwrap();

    // Reset the device
    if !reset_i2c_device(&mut dev_rst_pin).await {
        error_loop(&mut led_pin, &CODE_RESET_ERR[..]).await;
    }

    // Power up in FM receive mode with analog audio out
    let status = match device
        .power_up(POWER_UP_ARG1_FM, si47xx::OptMode::AnalogAudio)
        .await
    {
        Ok(s) => s,
        Err(_) => error_loop(&mut led_pin, &CODE_IIC_INVALID_ARG_ERR[..]).await,
    };

    if si47xx::is_bus_error(status.bits()) {
        error_loop(&mut led_pin, &CODE_IIC_INVALID_ARG_ERR[..]).await;
    }

    // tCTS for POWER_UP is 110 ms; wait a bit extra for the crystal to settle (XOSCEN = 1).
    Timer::after_millis(500).await;

    // Wait for CTS
    loop {
        let s = match device.get_int_status().await {
            Ok(s) => s,
            Err(_) => {
                Timer::after_micros(300).await;
                continue;
            }
        };
        if is_bus_cts(s.bits()) {
            break;
        }
        Timer::after_micros(300).await;
    }

    // Print chip revision
    match device.get_rev_info().await {
        Ok(rev) => info!(
            "PN=0x{:02X} FW={}.{} CMP={}.{} CHIP=0x{:02X}",
            rev.pn, rev.fw_major, rev.fw_minor, rev.cmp_major, rev.cmp_minor, rev.chiprev
        ),
        Err(_) => error!("GET_REV failed"),
    }

    // Enable STC + CTS + ERR interrupts in GPO_IEN
    if let Ok(res) = device
        .set_property(
            si47xx::ReceiverProperties::GpoIen as u16,
            (si47xx::GpoIen::STC_IEN | si47xx::GpoIen::CTS_IEN | si47xx::GpoIen::ERR_IEN).bits(),
        )
        .await
    {
        if !is_bus_cts(res.bits()) {
            error!("GPO_IEN did not return CTS");
            error_loop(&mut led_pin, &CODE_IIC_CTS_TIMEOUT_ERR[..]).await;
        }
    }

    // Soft-mute max attenuation = 10 dB
    let _ = device
        .set_property(
            si47xx::ReceiverProperties::FmSoftMuteMaxAttenuation as u16,
            10 as u16,
        )
        .await;

    // Scan the FM band for a station above the RSSI threshold
    let mut locked: Option<(u16, u8, u8)> = None; // (freq_10khz, rssi, snr)
    let mut tune_freq = FM_BAND_LOW;

    while tune_freq <= FM_BAND_HIGH {
        // Issue tune
        if device.set_tune_freq(tune_freq).await.is_err() {
            error!("FM_TUNE_FREQ i2c error @ {}", tune_freq);
            tune_freq += FM_STEP_10KHZ;
            continue;
        }

        // Wait for STCINT
        let mut attempts = 0u32;
        let mut line: String<32> = String::new();
        loop {
            let s = match device.get_int_status().await {
                Ok(s) => s,
                Err(_) => break,
            };
            if s.contains(si47xx::ReceiverStatus::STCINT) {
                break;
            }
            attempts += 1;
            if attempts > STC_MAX_ATTEMPTS {
                error!("STCINT timeout @ {}", tune_freq);
                // error_loop(&mut led_pin, &CODE_IIC_STCINT_ERR).await;
                break;
            }
            Timer::after_millis(STC_POLL_MS).await;
        }

        // Read status (also clears STCINT via INTACK)
        if let Ok(resp) = device.fm_tune_status(true).await {
            let freq = ((resp[2] as u16) << 8) | resp[3] as u16;
            let valid = (resp[1] & 0x01) != 0;
            let rssi = resp[4];
            let snr = resp[5];

            info!(
                "READFREQ={} RSSI={} SNR={} VALID={}",
                freq, rssi, snr, valid
            );

            write!(
                &mut line,
                "{}.{}kHz {}dB {}dB {}",
                freq / 100,
                freq - ((freq / 100) * 100),
                rssi,
                snr,
                if valid { "OK" } else { "--" }
            )
            .unwrap();

            display.clear_buffer();
            Text::with_baseline(&line, Point::new(0, 16), text_style, Baseline::Top)
                .draw(&mut display)
                .unwrap();
            display.flush().unwrap();
            line.clear();

            if valid && rssi >= FM_RSSI_LOCK_THRESHOLD {
                info!(
                    "Locked on {} kHz (RSSI={} dBuV, SNR={} dB)",
                    freq as u32 * 10,
                    rssi,
                    snr
                );
                locked = Some((freq, rssi, snr));
                break;
            }
        }

        tune_freq += FM_STEP_10KHZ;
    }

    // Read RSQ on the locked channel, if any
    match locked {
        Some((freq, _, _)) => {
            if let Ok(resp) = device.fm_rsq_status(true).await {
                info!(
                    "RSQ @ {} kHz: RSSI={} dBuV SNR={} dB MULT={} STBLEND={}",
                    freq as u32 * 10,
                    resp[4],
                    resp[5],
                    resp[6],
                    resp[7]
                );
            }
        }
        None => {
            error!(
                "No station above {} dBuV found in 87.5 - 108.0 MHz",
                FM_RSSI_LOCK_THRESHOLD
            );

            error_loop(&mut led_pin, &CODE_FM_NO_STATION_FOUND[..]).await;
        }
    }

    Timer::after_secs(10).await;
    let _ = device.power_down().await;

    display.clear_buffer();
    Text::with_baseline(
        "Radio is off!",
        Point::new(0, 16),
        text_style,
        Baseline::Top,
    )
    .draw(&mut display)
    .unwrap();
    display.flush().unwrap();

    loop {
        flash_signal(&mut led_pin, &[BLINK_LONG][..]).await;
    }
}

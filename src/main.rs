#![no_std]
#![no_main]

use crate::radio::si47xx::{self, is_bus_cts, FmRsqIntSource, GpoIen, PowerUpArg};

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
    mono_font::{ascii::FONT_6X10, MonoTextStyle, MonoTextStyleBuilder},
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

// AN332 Table 52: POWER_UP ARG2 = 0x05 => analog audio, FM receive
const POWER_UP_ARG2_FM: u8 = 0x05;

// FM_TUNE_FREQ: tSTC ≈ 60–80 ms on FMRX 4.0 (AN332 Table 49).
// 40 attempts × 5 ms = 200 ms budget.
const STC_POLL_MS: u64 = 100;
const STC_MAX_ATTEMPTS: u32 = 30;

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

/// Poll GET_INT_STATUS until STCINT is set or timeout.
async fn wait_for_stc<I2C, E>(
    dev: &mut si47xx::FmReceiver<I2C>,
) -> Result<(), si47xx::ReceiverError<E>>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
{
    for _ in 0..STC_MAX_ATTEMPTS {
        let s = dev.get_int_status().await?;
        if s.contains(si47xx::ReceiverStatus::STCINT) {
            return Ok(());
        }
        Timer::after_millis(STC_POLL_MS).await;
    }
    Err(si47xx::ReceiverError::CtsTimeout)
}

/// Send a SET_PROPERTY command and wait for CTS to be set.
async fn set_property_and_wait<I2C, E>(
    dev: &mut si47xx::FmReceiver<I2C>,
    prop: si47xx::ReceiverProperties,
    value: u16,
) -> Result<(), si47xx::ReceiverError<E>>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
{
    dev.set_property(prop as u16, value).await?;

    // Wait for CTS, up to ~100 ms
    for _ in 0..100 {
        let status = dev.get_int_status().await?;

        if status.contains(si47xx::ReceiverStatus::ERR) {
            error!("Si4743 ERR while setting property 0x{:04X}", prop as u16);
            return Err(si47xx::ReceiverError::InvalidArg);
        }

        if status.contains(si47xx::ReceiverStatus::CTS) {
            return Ok(());
        }
        Timer::after_millis(1).await;
    }
    Err(si47xx::ReceiverError::CtsTimeout)
}

/// Perform a seek and return (freq_10kHz, valid, rssi, snr).
async fn seek_fm_station<I2C, E>(
    dev: &mut si47xx::FmReceiver<I2C>,
    wrap: bool,
) -> Option<(u16, bool, u8, u8)>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
{
    if dev
        .fm_seek_start(si47xx::SeekDirection::Up, wrap)
        .await
        .is_err()
    {
        let _ = dev.fm_seek_cancel().await;
        return None;
    }

    if wait_for_stc(dev).await.is_err() {
        return None;
    }

    match dev.fm_tune_status(true).await {
        Ok(resp) => Some((
            ((resp[2] as u16) << 8) | resp[3] as u16, // READFREQ
            (resp[1] & 0x01) != 0,                    // VALID
            resp[4],                                  // RSSI
            resp[5],                                  // SNR
        )),
        Err(_) => None,
    }
}

fn print_yellow_message(
    line: &str,
    display: &mut impl DrawTarget<Color = BinaryColor>,
    text_style: MonoTextStyle<'_, BinaryColor>,
) {
    Text::with_baseline(&line, Point::new(0, 0), text_style, Baseline::Top)
        .draw(display)
        .ok();
}

fn print_blue_message(
    line: &str,
    display: &mut impl DrawTarget<Color = BinaryColor>,
    text_style: MonoTextStyle<'_, BinaryColor>,
) {
    Text::with_baseline(&line, Point::new(0, 16), text_style, Baseline::Top)
        .draw(display)
        .ok();
}

fn print_fm_info(
    freq: u16,
    valid: bool,
    rssi: u8,
    snr: u8,
    display: &mut impl DrawTarget<Color = BinaryColor>,
    text_style: MonoTextStyle<'_, BinaryColor>,
) {
    let mut line: String<32> = String::new();

    write!(
        &mut line,
        "{}.{} MHz RSSI={} \nSNR={} {}",
        freq / 100,
        freq % 100,
        rssi,
        snr,
        if valid { "OK" } else { "--" }
    )
    .ok();

    Text::with_baseline(&line, Point::new(0, 16), text_style, Baseline::Top)
        .draw(display)
        .ok();
}

#[embassy_executor::main]
async fn main(_s: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let mut dev_rst_pin = Output::new(p.PB1, Level::Low, Speed::Low);
    let mut display_rst_pin = Output::new(p.PB0, Level::Low, Speed::Low);
    let mut led_pin = Output::new(p.PC13, Level::High, Speed::Low);

    let i2c = I2c::new(
        p.I2C2,
        p.PA9,
        p.PA10,
        p.DMA1_CH4,
        p.DMA1_CH5,
        Irqs,
        Default::default(),
    );
    let i2c_bus = AtomicCell::new(i2c);

    let radio_bus = AtomicDevice::new(&i2c_bus);
    let mut device = si47xx::FmReceiver::new(radio_bus, I2C_ADDR_SEN_1);

    let display_bus = AtomicDevice::new(&i2c_bus);
    let interface = I2CDisplayInterface::new(display_bus);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    let text_style: MonoTextStyle<'_, BinaryColor> = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    display_reset(&mut display_rst_pin).await;
    Timer::after_millis(250).await;

    if display.init().is_err() {
        error_loop(&mut led_pin, &[BLINK_SHORT, BLINK_SHORT, BLINK_SHORT]).await;
    }

    print_yellow_message("Power up the radio", &mut display, text_style);
    display.flush().unwrap();

    // --- Hardware reset ---
    if !reset_i2c_device(&mut dev_rst_pin).await {
        error_loop(&mut led_pin, &CODE_RESET_ERR).await;
    }

    // --- POWER_UP (analog audio, FM receive) ---
    // AN332 Table 52: 0x01 0xC0 0x05
    match device
        .power_up(POWER_UP_ARG1_FM, si47xx::OptMode::AnalogAudio)
        .await
    {
        Ok(s) if !si47xx::is_bus_error(s.bits()) => {}
        _ => error_loop(&mut led_pin, &CODE_IIC_INVALID_ARG_ERR).await,
    }

    Timer::after_millis(500).await;

    // Wait for CTS after POWER_UP
    loop {
        match device.get_int_status().await {
            Ok(s) if is_bus_cts(s.bits()) => break,
            _ => Timer::after_micros(300).await,
        }
    }

    // --- GET_REV (AN332 Table 52: 0x10) ---
    match device.get_rev_info().await {
        Ok(rev) => {
            info!(
                "PN=0x{:02X} FW={}.{} CMP={}.{} CHIP=0x{:02X}",
                rev.pn, rev.fw_major, rev.fw_minor, rev.cmp_major, rev.cmp_minor, rev.chiprev
            );

            let mut line: String<32> = String::new();
            write!(
                &mut line,
                "PN=0x{:02X} \nFW={}.{} \nCMP={}.{} \nCHIP=0x{:02X}",
                rev.pn, rev.fw_major, rev.fw_minor, rev.cmp_major, rev.cmp_minor, rev.chiprev
            )
            .ok();
            print_blue_message(&line, &mut display, text_style);
            display.flush().unwrap();
        }
        Err(_) => error!("GET_REV failed"),
    }

    // Show chip info
    Timer::after_secs(3).await;
    display.clear_buffer();

    // ================================================================
    // AN332 Table 52 — FM/RDS Receiver Configuration Sequence
    // ================================================================

    // 1. GPO_IEN — enable STC, ERR, CTS, RSQ interrupts
    let gpo_ien =
        GpoIen::STC_IEN | GpoIen::RDS_IEN | GpoIen::RSQ_IEN | GpoIen::ERR_IEN | GpoIen::CTS_IEN;
    if set_property_and_wait(
        &mut device,
        si47xx::ReceiverProperties::GpoIen,
        gpo_ien.bits(),
    )
    .await
    .is_err()
    {
        error_loop(&mut led_pin, &CODE_IIC_CTS_TIMEOUT_ERR).await;
    }

    // 2. REFCLK_FREQ — 32.768 kHz crystal => 32768 Hz
    if set_property_and_wait(&mut device, si47xx::ReceiverProperties::RefclkFreq, 32768)
        .await
        .is_err()
    {
        error_loop(&mut led_pin, &CODE_IIC_CTS_TIMEOUT_ERR).await;
    }

    // 3. REFCLK_PRESCALE — divide by 1 (RCLK = 32.768 kHz directly)
    if set_property_and_wait(&mut device, si47xx::ReceiverProperties::RefclkPrescale, 1)
        .await
        .is_err()
    {
        error_loop(&mut led_pin, &CODE_IIC_CTS_TIMEOUT_ERR).await;
    }

    // 4. RX_VOLUME — output volume = 63 (max)
    if set_property_and_wait(&mut device, si47xx::ReceiverProperties::RxVolume, 63)
        .await
        .is_err()
    {
        error_loop(&mut led_pin, &CODE_IIC_CTS_TIMEOUT_ERR).await;
    }

    // 5. FM_DEEMPHASIS — 50 µs (Europe)
    if set_property_and_wait(
        &mut device,
        si47xx::ReceiverProperties::FmDeemphasis,
        0x0001,
    )
    .await
    .is_err()
    {
        error_loop(&mut led_pin, &CODE_IIC_CTS_TIMEOUT_ERR).await;
    }

    // 6. RX_HARD_MUTE — enable L and R audio outputs (unmute)
    if set_property_and_wait(&mut device, si47xx::ReceiverProperties::RxHardMute, 0x0000)
        .await
        .is_err()
    {
        error_loop(&mut led_pin, &CODE_IIC_CTS_TIMEOUT_ERR).await;
    }

    // 7. FM_BLEND_RSSI_STEREO_THRESHOLD — 49 dBµV
    if set_property_and_wait(
        &mut device,
        si47xx::ReceiverProperties::FmBlendRssiStereoThreshold,
        49,
    )
    .await
    .is_err()
    {
        error_loop(&mut led_pin, &CODE_IIC_CTS_TIMEOUT_ERR).await;
    }

    // 8. FM_BLEND_RSSI_MONO_THRESHOLD — 30 dBµV
    if set_property_and_wait(
        &mut device,
        si47xx::ReceiverProperties::FmBlendRssiMonoThreshold,
        30,
    )
    .await
    .is_err()
    {
        error_loop(&mut led_pin, &CODE_IIC_CTS_TIMEOUT_ERR).await;
    }

    // 9. FM_MAX_TUNE_ERROR — 40 kHz
    if set_property_and_wait(&mut device, si47xx::ReceiverProperties::FmMaxTuneError, 20)
        .await
        .is_err()
    {
        error_loop(&mut led_pin, &CODE_IIC_CTS_TIMEOUT_ERR).await;
    }

    // 10. FM_RSQ_INT_SOURCE — enable blend, SNR hi/lo, RSSI hi/lo interrupts
    let rsq_src = FmRsqIntSource::BLEND_IEN
        | FmRsqIntSource::SNR_HI_IEN
        | FmRsqIntSource::SNR_LO_IEN
        | FmRsqIntSource::RSSI_HI_IEN
        | FmRsqIntSource::RSSI_LO_IEN;
    if set_property_and_wait(
        &mut device,
        si47xx::ReceiverProperties::FmRsqIntSource,
        rsq_src.bits(),
    )
    .await
    .is_err()
    {
        error_loop(&mut led_pin, &CODE_IIC_CTS_TIMEOUT_ERR).await;
    }

    // 11. FM_RSQ_SNR_HI_THRESHOLD — 30 dB
    if set_property_and_wait(
        &mut device,
        si47xx::ReceiverProperties::FmRsqSnrHiThreshold,
        30,
    )
    .await
    .is_err()
    {
        error_loop(&mut led_pin, &CODE_IIC_CTS_TIMEOUT_ERR).await;
    }

    // 12. FM_RSQ_SNR_LO_THRESHOLD — 6 dB
    if set_property_and_wait(
        &mut device,
        si47xx::ReceiverProperties::FmRsqSnrLoThreshold,
        6,
    )
    .await
    .is_err()
    {
        error_loop(&mut led_pin, &CODE_IIC_CTS_TIMEOUT_ERR).await;
    }

    // 13. FM_RSQ_RSSI_HI_THRESHOLD — 50 dBµV
    if set_property_and_wait(
        &mut device,
        si47xx::ReceiverProperties::FmRsqRssiHiThreshold,
        50,
    )
    .await
    .is_err()
    {
        error_loop(&mut led_pin, &CODE_IIC_CTS_TIMEOUT_ERR).await;
    }

    // 14. FM_RSQ_RSSI_LO_THRESHOLD — 24 dBµV
    if set_property_and_wait(
        &mut device,
        si47xx::ReceiverProperties::FmRsqRssiLoThreshold,
        24,
    )
    .await
    .is_err()
    {
        error_loop(&mut led_pin, &CODE_IIC_CTS_TIMEOUT_ERR).await;
    }

    // 15. FM_RSQ_BLEND_THRESHOLD — pilot = 1, threshold = 50% (0x0032)
    if set_property_and_wait(
        &mut device,
        si47xx::ReceiverProperties::FmRsqBlendThreshold,
        0x0032,
    )
    .await
    .is_err()
    {
        error_loop(&mut led_pin, &CODE_IIC_CTS_TIMEOUT_ERR).await;
    }

    // 16. FM_SOFT_MUTE_MAX_ATTENUATION — 10 dB
    if set_property_and_wait(
        &mut device,
        si47xx::ReceiverProperties::FmSoftMuteMaxAttenuation,
        10,
    )
    .await
    .is_err()
    {
        error_loop(&mut led_pin, &CODE_IIC_CTS_TIMEOUT_ERR).await;
    }

    // 17. FM_SOFT_MUTE_SNR_THRESHOLD — 6 dB
    if set_property_and_wait(
        &mut device,
        si47xx::ReceiverProperties::FmSoftMuteSnrThreshold,
        6,
    )
    .await
    .is_err()
    {
        error_loop(&mut led_pin, &CODE_IIC_CTS_TIMEOUT_ERR).await;
    }

    // 18. FM_SEEK_BAND_BOTTOM — 87.50 MHz
    if set_property_and_wait(
        &mut device,
        si47xx::ReceiverProperties::FmSeekBandBottom,
        8750,
    )
    .await
    .is_err()
    {
        error_loop(&mut led_pin, &CODE_IIC_CTS_TIMEOUT_ERR).await;
    }

    // 19. FM_SEEK_BAND_TOP — 107.9 MHz
    if set_property_and_wait(
        &mut device,
        si47xx::ReceiverProperties::FmSeekBandTop,
        10790,
    )
    .await
    .is_err()
    {
        error_loop(&mut led_pin, &CODE_IIC_CTS_TIMEOUT_ERR).await;
    }

    // 20. FM_SEEK_FREQ_SPACING — 100 kHz for EU
    if set_property_and_wait(
        &mut device,
        si47xx::ReceiverProperties::FmSeekFreqSpacing,
        100,
    )
    .await
    .is_err()
    {
        error_loop(&mut led_pin, &CODE_IIC_CTS_TIMEOUT_ERR).await;
    }

    // 21. FM_SEEK_TUNE_SNR_THRESHOLD — 6 dB (default: 3 dB)
    if set_property_and_wait(
        &mut device,
        si47xx::ReceiverProperties::FmSeekTuneSnrThreshold,
        3,
    )
    .await
    .is_err()
    {
        error_loop(&mut led_pin, &CODE_IIC_CTS_TIMEOUT_ERR).await;
    }

    // 22. FM_SEEK_TUNE_RSSI_THRESHOLD — 10 dBµV (default: 20 dBµV)
    if set_property_and_wait(
        &mut device,
        si47xx::ReceiverProperties::FmSeekTuneRssiThreshold,
        20,
    )
    .await
    .is_err()
    {
        error_loop(&mut led_pin, &CODE_IIC_CTS_TIMEOUT_ERR).await;
    }

    info!("Si4743 configuration complete");

    // ================================================================
    // Main loop: seek for a station, display, repeat
    // ================================================================
    let mut wait_time = 2;
    loop {
        display.clear_buffer();
        print_yellow_message("Seeking...", &mut display, text_style);
        display.flush().unwrap();

        match seek_fm_station(&mut device, true).await {
            Some((freq, valid, rssi, snr)) => {
                info!(
                    "Found station: {}.{} MHz RSSI={} SNR={}",
                    freq / 100,
                    freq % 100,
                    rssi,
                    snr
                );
                display.clear_buffer();
                print_fm_info(freq, valid, rssi, snr, &mut display, text_style);
                display.flush().unwrap();
                wait_time = 10;
            }
            None => {
                error!("No station found");
                flash_signal(&mut led_pin, &CODE_FM_NO_STATION_FOUND).await;
                display.clear_buffer();
                print_yellow_message("No stations found!", &mut display, text_style);
                display.flush().unwrap();
                wait_time = 2;
            }
        }

        Timer::after_secs(wait_time).await;
    }
    /*
    device.set_tune_freq(9410).await.unwrap();

    loop {
        let s = device.get_int_status().await.unwrap();
        Timer::after_millis(200).await;
        if s.contains(si47xx::ReceiverStatus::STCINT) {
            break;
        }
        Timer::after_millis(100).await;
    }
    display.clear_buffer();

    let mut line: String<32> = String::new();
    let resp = device.fm_tune_status(true).await.unwrap();

    let valid = (resp[1] & 0x01) != 0;
    let afcrl = (resp[1] & 0x02) as u8;

    let freq = ((resp[2] as u16) << 8) | resp[3] as u16;
    let rssi = resp[4];
    let snr = resp[5];

    write!(
        &mut line,
        "{}.{} MHz RSSI={} \nSNR={} AFCRL={} \nVALID={}",
        freq / 100,
        freq % 100,
        rssi,
        snr,
        afcrl,
        if valid { "OK" } else { "NO" }
    )
    .ok();

    print_blue_message(&line, &mut display, text_style);
    display.flush().unwrap();
    loop {
        Timer::after_secs(1).await;
    }*/
}

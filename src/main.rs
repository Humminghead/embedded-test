#![no_std]
#![no_main]

use crate::radio::si47xx::{self, is_bus_cts, PowerUpArg};

use defmt::{error, info};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::peripherals::I2C2;
use embassy_time::Timer;
use embedded_hal::digital::OutputPin;
use panic_probe as _;
mod radio;

bind_interrupts!(
    struct Irqs {
        I2C2_EV => embassy_stm32::i2c::EventInterruptHandler<I2C2>;
        I2C2_ER => embassy_stm32::i2c::ErrorInterruptHandler<I2C2>;
    }
);

static I2C_ADDR_SEN_1: u8 = 0b01100011; // 0x63

static BLINK_LONG: (i32, i32) = (500, 500);
static BLINK_SHORT: (i32, i32) = (250, 250);

static CODE_RESET_ERR: [(i32, i32); 2] = [BLINK_SHORT, BLINK_SHORT];
static CODE_IIC_CTS_TIMEOUT_ERR: [(i32, i32); 2] = [BLINK_LONG, BLINK_SHORT];
static CODE_IIC_INVALID_ARG_ERR: [(i32, i32); 3] = [BLINK_LONG, BLINK_SHORT, BLINK_SHORT];

static RESTART_TIME_SEC: u64 = 2;

// ARG1 for POWER_UP in FM receive mode:
// CTSIEN | GPO2OEN | FUNC[3:0] = 0b1_1_0000
const POWER_UP_ARG1_FM: u8 = (PowerUpArg::CTSIEN.bits() | PowerUpArg::GPO2OEN.bits() | PowerUpArg::XOSCEN.bits()) as u8;

// ARG2 OPMODE = 0b0000_0101 analog audio outputs (LOUT/ROUT)
const POWER_UP_ARG2_ANALOG: u8 = 0b0000_0101;

// Target FM frequency: FM_TUNE_FREQ expects 10 kHz units.
// 103.4 MHz -> 10340 (0x2864)
const FM_FREQ_10KHZ: u16 = 10050;

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

async fn error_loop<P: OutputPin>(pin: &mut P, sig: &[(i32, i32)]) {
    loop {
        Timer::after_secs(RESTART_TIME_SEC).await;
        flash_signal(pin, sig).await;
    }
}

async fn check_cts<P: OutputPin>(pin: &mut P, value: u8) {
    if !is_bus_cts(value) {
        error_loop(pin, &CODE_IIC_CTS_TIMEOUT_ERR).await;
    }
}
#[embassy_executor::main]
async fn main(_s: Spawner) {
    let mut cnt = 0;
    let p = embassy_stm32::init(Default::default());

    let mut dev_rst_pin = Output::new(p.PB1, Level::Low, Speed::Low);
    let mut led_pin = Output::new(p.PC13, Level::High, Speed::Low);

    let i2c = I2c::new_no_dma(p.I2C2, p.PB10, p.PB11, Irqs, Default::default());
    let mut device = si47xx::FmReceiver::new(i2c, I2C_ADDR_SEN_1);

    // Reset the device
    if !reset_i2c_device(&mut dev_rst_pin).await {
        error_loop(&mut led_pin, &CODE_RESET_ERR).await;
    }

    // Power up in FM receive mode with analog audio out
    let status = device
        .power_up(POWER_UP_ARG1_FM, si47xx::OptMode::AnalogAudio)
        .await
        .unwrap();

    if si47xx::is_bus_error(status.bits()) {
        error_loop(&mut led_pin, &CODE_IIC_INVALID_ARG_ERR).await;
    }

    Timer::after_millis(500).await; // settle crystal before polling CTS

    // Wait for CTS
    loop {
        let s = device.get_int_status().await.unwrap();
        if si47xx::wait_cts(&s).await {
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

    error!("{}", cnt);
    cnt += 1;

    // Enable CTS + STC + ERR interrupts in GPO_IEN
    {
        let res = device
            .set_property(
                si47xx::ReceiverProperties::GpoIen as u16,
                (si47xx::GpoIen::STC_IEN | si47xx::GpoIen::CTS_IEN | si47xx::GpoIen::ERR_IEN)
                    .bits(),
            )
            .await
            .unwrap();
        check_cts(&mut led_pin, res.bits()).await;
    }

    error!("{}", cnt);
    cnt += 1;

    // Optional: configure soft mute / blend / channel filter defaults
    // For example, set soft mute attenuation to 10 dB:
    {
        let _ = device
            .set_property(
                si47xx::ReceiverProperties::FmSoftMuteMaxAttenuation as u16,
                0x000A,
            )
            .await;
    }

    error!("{}", cnt);
    cnt += 1;

    // Tune to FM frequency
    // ... after the GPO_IEN and soft-mute setup ...

    let mut tune_freq: u16 = 8750;
    while tune_freq <= 10800 {
        error!("Tune freq {} ({} kHz)", tune_freq, tune_freq as u32 * 10);

        device.set_tune_freq(tune_freq).await.unwrap();

        // tSTC for FM_TUNE_FREQ is ~60-80 ms on FMRX 4.0; give it plenty of margin.
        let mut attempts = 0u32;
        loop {
            let s = device.get_int_status().await.unwrap();
            if s.contains(si47xx::ReceiverStatus::STCINT) {
                break;
            }
            attempts += 1;
            if attempts > 40 {
                // ~200 ms worst case
                error!("STCINT timeout @ {}", tune_freq);
                break;
            }
            Timer::after_millis(5).await; // <-- was micros(10)
        }

        // Read + clear STCINT, then sample RSSI/SNR
        match device.fm_tune_status(true).await {
            Ok(resp) => {
                let freq = ((resp[2] as u16) << 8) | resp[3] as u16;
                info!(
                    "READFREQ={} RSSI={} SNR={} VALID={}",
                    freq,
                    resp[4],
                    resp[5],
                    resp[1] & 0x01
                );
            }
            Err(_) => error!("FM_TUNE_STATUS failed @ {}", tune_freq),
        }

        tune_freq += 10; // 100 kHz step
    }

    error!("{}", cnt);
    cnt += 1;

    // Read tune status (clears STCINT via INTACK)
    match device.fm_tune_status(true).await {
        Ok(resp) => {
            let freq = ((resp[2] as u16) << 8) | resp[3] as u16;
            let rssi = resp[4];
            let snr = resp[5];
            info!(
                "Tuned to {} (10 kHz) RSSI={} dBuV SNR={} dB",
                freq, rssi, snr
            );
        }
        Err(_) => error!("FM_TUNE_STATUS failed"),
    }

    error!("{}", cnt);
    cnt += 1;

    // Read RSQ status once
    match device.fm_rsq_status(true).await {
        Ok(resp) => {
            let rssi = resp[4];
            let snr = resp[5];
            info!("RSQ: RSSI={} dBuV SNR={} dB", rssi, snr);
        }
        Err(_) => error!("FM_RSQ_STATUS failed"),
    }

    error!("{}", cnt);
    cnt += 1;

    // Success blink pattern
    flash_signal(&mut led_pin, &[BLINK_LONG, BLINK_LONG, BLINK_LONG]).await;

    Timer::after_secs(30).await;
    let _ = device.power_down().await;

    loop {
        Timer::after_secs(1).await;
        flash_signal(&mut led_pin, &[BLINK_LONG, BLINK_SHORT]).await;
    }
}

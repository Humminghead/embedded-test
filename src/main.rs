#![no_std]
#![no_main]

use crate::radio::si47xx::{self, PowerUpArg, ReceiverError};

use defmt::{error, info};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::{Config, I2c};
use embassy_stm32::peripherals::I2C2;
use embassy_time::Timer;
use embedded_hal::digital::OutputPin; // or embedded_hal::digital::v2::OutputPin
use panic_probe as _;
mod radio;

bind_interrupts!(
    struct Irqs {
        I2C2_EV => embassy_stm32::i2c::EventInterruptHandler<I2C2>;
        I2C2_ER => embassy_stm32::i2c::ErrorInterruptHandler<I2C2>;
    }
);

// Device address
static I2C_ADDR_SEN_0: u8 = 0b00010001; // 0x11
static I2C_ADDR_SEN_1: u8 = 0b01100011; // 0x63

// Led flash time
static BLINK_LONG: (i32, i32) = (500, 500);
static BLINK_SHORT: (i32, i32) = (250, 250);

// Led error codes
static CODE_RESET_ERR: [(i32, i32); 2] = [BLINK_SHORT, BLINK_SHORT]; // 2
static CODE_CHIP_POWER_UP_ERR: [(i32, i32); 3] = [BLINK_SHORT, BLINK_SHORT, BLINK_SHORT]; // 3
static CODE_IIC_CTS_TIMEOUT_ERR: [(i32, i32); 2] = [BLINK_LONG, BLINK_SHORT]; // 11
static CODE_IIC_INVALID_ARG_ERR: [(i32, i32); 3] = [BLINK_LONG, BLINK_SHORT, BLINK_SHORT]; // 12

// Time values
static RESTART_TIME_SEC: u64 = 2;

// Power up options
const POWER_UP_FLAGS: u8 =
    (PowerUpArg::CTSIEN.bits() | PowerUpArg::GPO2OEN.bits() | PowerUpArg::XOSCEN.bits()) as u8;

/* Flashes the led according a signal pattern */
async fn flash_singnal<P: OutputPin>(pin: &mut P, singnal: &[(i32, i32)]) {
    for &(h_ms, l_ms) in singnal {
        let _ = pin.set_high();
        Timer::after_millis(h_ms as u64).await;
        let _ = pin.set_low();
        Timer::after_millis(l_ms as u64).await;
    }

    let _ = pin.set_high();
}

/* Reset the I2C device with the 'pin' */
async fn reset_i2c_device<P: OutputPin>(pin: &mut P) -> bool {
    let mut result = pin.set_low();
    if result.is_err() {
        return false;
    }
    Timer::after_millis(250).await;
    result = pin.set_high();
    if result.is_err() {
        return false;
    }
    Timer::after_millis(250).await;

    true
}

async fn error_loop<P: OutputPin>(pin: &mut P, sig: &[(i32, i32)]) {
    loop {
        Timer::after_secs(RESTART_TIME_SEC).await;
        flash_singnal(pin, sig).await;
    }
}

#[embassy_executor::main]
async fn main(_s: Spawner) {
    // Create mcu's peripherial
    let p = embassy_stm32::init(Default::default());

    // Create rst pin for control of the reset of the SI device
    let mut dev_rst_pin = Output::new(p.PB1, Level::Low, Speed::Low);

    // Create led pin for device state monitoring
    let mut led_pin = Output::new(p.PC13, Level::High, Speed::Low);

    // Create radio device
    let i2c = I2c::new_no_dma(p.I2C2, p.PB10, p.PB11, Irqs, Default::default());
    let mut device = si47xx::Receiver::new(i2c, I2C_ADDR_SEN_1);

    // Reset the device
    if !reset_i2c_device(&mut dev_rst_pin).await {
        error_loop(&mut led_pin, &CODE_RESET_ERR).await;
    }

    let err = device.power_up(POWER_UP_FLAGS, si47xx::OptMode::AnalogAudio).await;

    if err == Err(ReceiverError::CtsTimeout) {
        loop {
            if device.poll_int_status().await.is_ok() {
                break;
            }
            flash_singnal(&mut led_pin, &CODE_IIC_CTS_TIMEOUT_ERR).await;
        }
    } else if err == Err(ReceiverError::InvalidArg) {
        error_loop(&mut led_pin, &CODE_IIC_INVALID_ARG_ERR).await;
    } else {
        error!("other");
        error_loop(&mut led_pin, &CODE_CHIP_POWER_UP_ERR).await;
    }

    info!("Chip revision: {}", device.get_rev_info().await.unwrap());

    Timer::after_secs(2).await;
    let _ = device.power_down().await;

    loop {
        Timer::after_secs(1).await;
        //flash_singnal(&mut led_pin, &[BLINK_LONG, BLINK_LONG, BLINK_LONG]).await;
    }
}

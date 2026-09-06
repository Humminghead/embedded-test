#![no_std]
#![no_main]

use core::result;

use crate::radio::si47xx;

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::pac::GPIOB;
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_sync::signal;
use embassy_time::Timer;
use embedded_hal::digital::{ErrorType, OutputPin}; // or embedded_hal::digital::v2::OutputPin
use panic_probe as _;

mod radio;

bind_interrupts!(
    struct Irqs {
        // USB_LP_CAN1_RX0 => embassy_stm32::usb::InterruptHandler<peripherals::USB>;
    }
);

// Device address
static I2C_ADDR: u8 = 0x63;

// Led error codes
static CODE_RESET_ERR: [(i32, i32); 2] = [(125, 125), (125, 125)]; // 2
static CODE_CHIP_POWER_UP_ERR: [(i32, i32); 3] = [(125, 125), (125, 125), (125, 125)]; // 3

// Time values
static RESTART_TIME_SEC: u64 = 2;

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
    let i2c = I2c::new_blocking(p.I2C2, p.PB10, p.PB11, Default::default());
    let mut device = si47xx::Receiver::new(i2c, I2C_ADDR);

    // Reset the device
    if !reset_i2c_device(&mut dev_rst_pin).await {
        error_loop(&mut led_pin, &CODE_RESET_ERR).await;
    }

    if device.power_up(si47xx::OptMode::AnalogAudio).await.is_err() {
        error_loop(&mut led_pin, &CODE_CHIP_POWER_UP_ERR).await;
    }

    // let info = device.get_rev_info().await;

    // Timer::after_secs(2).await;
    // let _ = device.power_down().await;

    loop {
        Timer::after_secs(1).await;
    }
}

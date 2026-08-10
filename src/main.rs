#![no_std]
#![no_main]

use crate::radio::si47xx;

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::i2c::I2c;
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_time::Timer;
use panic_probe as _;

mod radio;

bind_interrupts!(struct Irqs {
    // USB_LP_CAN1_RX0 => embassy_stm32::usb::InterruptHandler<peripherals::USB>;
});

#[embassy_executor::main]
async fn main(_s: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let i2c = I2c::new_blocking(p.I2C2, p.PB10, p.PB11, Default::default());
    let mut device = si47xx::Receiver::new(i2c, 0x63);

    let ok = device.power_up(si47xx::OptMode::AnalogAudio).await;

    if !ok.is_err() {
        info!("Device started!");
    }

    let info = device.get_rev_info().await;

    Timer::after_secs(5).await;
    let _ = device.power_down().await;

    loop {
        Timer::after_secs(1).await;
    }
}

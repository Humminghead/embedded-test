#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
// use embassy_time::Timer;
use embassy_stm32::{bind_interrupts, peripherals};
use panic_probe as _;

bind_interrupts!(struct Irqs {
    USB_LP_CAN1_RX0 => embassy_stm32::usb::InterruptHandler<peripherals::USB>;
});

#[embassy_executor::main]
async fn main(_s: Spawner) {
    info!("Hello World!");
    loop {
        // Timer::after_secs(1).await;
    }
}

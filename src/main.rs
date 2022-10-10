#![no_std]
#![no_main]

use panic_halt as _;

use ads1x1x::{channel, Ads1x1x, SlaveAddr};
use arduino_hal::{hal::wdt, prelude::*};
use nb::block;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let mut watchdog = wdt::Wdt::new(dp.WDT, &dp.CPU.mcusr);
    let pins = arduino_hal::pins!(dp);

    let mut led = pins.d13.into_output();
    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);

    let i2c = arduino_hal::I2c::new(
        dp.TWI,
        pins.a4.into_pull_up_input(),
        pins.a5.into_pull_up_input(),
        50000,
    );
    let adc_addr = SlaveAddr::default();
    let mut adc = Ads1x1x::new_ads1013(i2c, adc_addr);

    ufmt::uwriteln!(&mut serial, "Hello from Arduino!\r").ok();
    watchdog.start(wdt::Timeout::Ms2000).unwrap();

    loop {
        led.toggle();
        let meas = block!(adc.read(&mut channel::DifferentialA0A1)).unwrap();
        ufmt::uwriteln!(&mut serial, "ADC read: {}", meas).ok();
        arduino_hal::delay_ms(500);
        watchdog.feed();
    }
}

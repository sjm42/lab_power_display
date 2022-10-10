#![no_std]
#![no_main]

use panic_halt as _;

use ads1x1x::{channel, Ads1x1x, FullScaleRange, SlaveAddr};
use arduino_hal::{hal::wdt, prelude::*, spi};
use embedded_hal::spi::MODE_0;

use max7219::*;
use nb::block;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let mut watchdog = wdt::Wdt::new(dp.WDT, &dp.CPU.mcusr);
    let pins = arduino_hal::pins!(dp);

    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);

    let i2c = arduino_hal::I2c::new(
        dp.TWI,
        pins.a4.into_pull_up_input(),
        pins.a5.into_pull_up_input(),
        50000,
    );
    let adc_addr = SlaveAddr::default();
    let mut adc = Ads1x1x::new_ads1015(i2c, adc_addr);
    adc.disable_comparator().unwrap();

    /*
    let data = pins.d8.into_output();
    let sck = pins.d9.into_output();
    let cs = pins.d10.into_output();
    let disp = MAX7219::from_pins(displays, data, cs, sck);
    */

    // SPI pins:
    // CS/SS = d10
    // MOSI = d11
    // MISO = d12
    // SCLK = d13

    let spi_c = spi::Settings {
        data_order: spi::DataOrder::MostSignificantFirst,
        clock: spi::SerialClockRate::OscfOver64,
        mode: MODE_0,
    };

    let (spi, _) = arduino_hal::Spi::new(
        dp.SPI,
        pins.d13.into_output(),
        pins.d11.into_output(),
        pins.d12.into_pull_up_input(),
        pins.d10.into_output(),
        spi_c,
    );
    let mut disp = MAX7219::from_spi_cs(2, spi, pins.d9.into_output()).unwrap();
    disp.power_on().unwrap();
    disp.set_intensity(0, 1).unwrap();
    disp.set_intensity(1, 1).unwrap();
    disp.clear_display(0).unwrap();
    disp.clear_display(1).unwrap();

    ufmt::uwrite!(&mut serial, "Hello from Arduino!\r\n").ok();
    watchdog.start(wdt::Timeout::Ms2000).unwrap();

    // We use gain 16 for input 0-1 and gain 1 for input 2-3

    let mut buf: [u8; 8] = [b' '; 8];
    loop {
        adc.set_full_scale_range(FullScaleRange::Within4_096V)
            .unwrap();
        let volt = block!(adc.read(&mut channel::DifferentialA0A1)).unwrap();
        ufmt::uwrite!(&mut serial, "ADC volt: {}\r\n", volt).ok();

        buf[0] = b'U';
        i16_disp(volt, &mut buf[2..=7]);
        disp.write_str(0, &buf, 0b00000000).unwrap();
        /*
        ufmt::uwrite!(&mut serial, "U: \"{}\"\r\n", unsafe {
            core::str::from_utf8_unchecked(&buf)
        })
        .ok();
        */

        arduino_hal::delay_ms(500);
        watchdog.feed();

        adc.set_full_scale_range(FullScaleRange::Within0_256V)
            .unwrap();
        let amps = block!(adc.read(&mut channel::DifferentialA2A3)).unwrap();
        ufmt::uwrite!(&mut serial, "ADC amps: {}\r\n", amps).ok();

        buf[0] = b'A';
        i16_disp(amps, &mut buf[2..=7]);
        disp.write_str(1, &buf, 0b00000000).unwrap();
        /*
        ufmt::uwrite!(&mut serial, "A: \"{}\"\r\n", unsafe {
            core::str::from_utf8_unchecked(&buf)
        })
        .ok();
        */

        arduino_hal::delay_ms(500);
        watchdog.feed();
        ufmt::uwrite!(&mut serial, "\r\n").ok();
    }
}

// print the given 16bit integer into the buffer given
fn i16_disp(val: i16, buf: &mut [u8]) {
    if buf.len() < 6 {
        return;
    }
    let uval = val.unsigned_abs();
    buf[0] = if val.is_negative() { b'-' } else { b' ' };
    buf[1] = 0x30 + (((uval / 10000) % 10) as u8);
    buf[2] = 0x30 + (((uval / 1000) % 10) as u8);
    buf[3] = 0x30 + (((uval / 100) % 10) as u8);
    buf[4] = 0x30 + (((uval / 10) % 10) as u8);
    buf[5] = 0x30 + ((uval % 10) as u8);
}

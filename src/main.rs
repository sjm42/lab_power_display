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

    // SPI pins:
    // CS/SS = d10 (dumy, not used - we actually use d9 for MAX7219)
    // MOSI = d11
    // MISO = d12
    // SCLK = d13

    let spi_c = spi::Settings {
        data_order: spi::DataOrder::MostSignificantFirst,
        clock: spi::SerialClockRate::OscfOver2,
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
    let mut disp = MAX7219::from_spi_cs(1, spi, pins.d9.into_output()).unwrap();
    disp.power_on().unwrap();
    disp.set_intensity(0, 1).unwrap();
    disp.clear_display(0).unwrap();

    ufmt::uwrite!(&mut serial, "Lab power display\r\n").ok();
    watchdog.start(wdt::Timeout::Ms2000).unwrap();

    let mut buf: [u8; 8] = [b' '; 8];
    loop {
        // use gain 16 for input 0-1 (current)
        adc.set_full_scale_range(FullScaleRange::Within0_256V)
            .unwrap();
        let amps = block!(adc.read(&mut channel::DifferentialA0A1)).unwrap();

        #[cfg(feature = "calibrate")]
        {
            ufmt::uwrite!(&mut serial, "ADC amps: {}\r\n", amps).ok();

            buf[0] = b'A';
            i16_disp(amps, &mut buf);
            disp.write_str(0, &buf, 0b00000000).unwrap();
            ufmt::uwrite!(&mut serial, "{}\r\n", unsafe {
                core::str::from_utf8_unchecked(&buf)
            })
            .ok();
            arduino_hal::delay_ms(500);
        }

        watchdog.feed();

        // use gain 1 for input 2-3 (voltage)
        adc.set_full_scale_range(FullScaleRange::Within4_096V)
            .unwrap();
        let volt = block!(adc.read(&mut channel::DifferentialA2A3)).unwrap();

        #[cfg(feature = "calibrate")]
        {
            ufmt::uwrite!(&mut serial, "ADC amps: {}\r\n\r\n", volt).ok();

            buf[0] = b'U';
            i16_disp(volt, &mut buf);
            disp.write_str(0, &buf, 0b00000000).unwrap();
            ufmt::uwrite!(&mut serial, "{}\r\n", unsafe {
                core::str::from_utf8_unchecked(&buf)
            })
            .ok();

            arduino_hal::delay_ms(500);
        }

        watchdog.feed();
    }
}

// print the given 16bit integer into the buffer given
fn i16_disp(val: i16, buf: &mut [u8]) {
    if buf.len() < 8 {
        return;
    }
    let uval = val.unsigned_abs();
    buf[1] = if val.is_negative() { b'-' } else { b' ' };
    buf[2] = 0x30 + (((uval / 10000) % 10) as u8);
    buf[3] = 0x30 + (((uval / 1000) % 10) as u8);
    buf[5] = 0x30 + (((uval / 100) % 10) as u8);
    buf[6] = 0x30 + (((uval / 10) % 10) as u8);
    buf[7] = 0x30 + ((uval % 10) as u8);
}

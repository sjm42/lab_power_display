// lab_power_display - main.rs

#![no_std]
#![no_main]

use ads1x1x::{channel, Ads1x1x, FullScaleRange, SlaveAddr};
use arduino_hal::{hal::wdt, prelude::*, spi};
use embedded_hal::spi::MODE_0;
use max7219::*;
use nb::block;
use ufmt::uWrite;

use panic_halt as _;

const N_AVG: i32 = 16;

// ************************************************
// Our calibration data, based on real measurements

const AMP_OFF: i16 = 70;

const CAL_V: [(i16, f32); 13] = [
    (0, 0.000),
    (773, 1.082),
    (1452, 2.020),
    (2543, 3.523),
    (3624, 5.012),
    (6516, 9.017),
    (8717, 12.050),
    (10897, 15.030),
    (14508, 20.000),
    (18186, 25.080),
    (21790, 30.030),
    (22585, 31.120),
    (24175, 33.360),
];

const CAL_I: [(i16, f32); 9] = [
    (0, 0.0),
    (544, 0.100),
    (2160, 0.500),
    (4195, 1.000),
    (6448, 1.500),
    (8376, 2.000),
    (10834, 2.500),
    (12990, 3.000),
    (15150, 3.500),
];

// *** end of calibration data
// ***************************

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut watchdog = wdt::Wdt::new(dp.WDT, &dp.CPU.mcusr);
    watchdog.start(wdt::Timeout::Ms2000).unwrap();

    // NOTE: calibration mode is triggered with jumper in d3+d4
    pins.d3.into_output().set_low();
    let d4 = pins.d4.into_pull_up_input();
    // leave it to settle for a short time while we do other stuff...

    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);
    ufmt::uwrite!(&mut serial, "Lab power display\r\n").ok();

    // Display buffer for our 8 digit 7seg
    let mut buf: [u8; 8] = [b'-'; 8];

    // *** Initialize SPI and MAX7219
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
    disp.write_str(0, &buf, 0b11111111).unwrap();

    ufmt::uwrite!(&mut serial, "SPI and MAX7219 init done.\r\n").ok();

    // *** Initializa i2c and ADS1115

    let i2c = arduino_hal::I2c::new(
        dp.TWI,
        pins.a4.into_pull_up_input(),
        pins.a5.into_pull_up_input(),
        50000,
    );
    let adc_addr = SlaveAddr::default();
    let mut adc = Ads1x1x::new_ads1115(i2c, adc_addr);
    adc.disable_comparator().unwrap();

    ufmt::uwrite!(&mut serial, "I2C and ADS1115 init done.\r\n").ok();

    // Check if the jumper is present
    let calibration = d4.is_low();
    let production = !calibration;

    loop {
        // use gain 16 for input 0-1 (current)
        adc.set_full_scale_range(FullScaleRange::Within0_256V)
            .unwrap();
        arduino_hal::delay_ms(5);
        let mut sum: i32 = 0;
        for _ in 0..N_AVG {
            sum += block!(adc.read(&mut channel::DifferentialA0A1)).unwrap() as i32;
            arduino_hal::delay_ms(1);
        }
        watchdog.feed();
        let adc_amps = (sum / N_AVG) as i16;

        if calibration {
            ufmt::uwrite!(&mut serial, "ADC amps: {}\r\n", adc_amps).ok();
            buf[0] = b'A';
            i16_disp(adc_amps, &mut buf);
            disp.set_intensity(0, 8).unwrap();
            disp.write_str(0, &buf, 0b00000000).unwrap();
            /*
                ufmt::uwrite!(&mut serial, "{}\r\n", unsafe {
                    core::str::from_utf8_unchecked(&buf)
                })
                .ok();
            */
            arduino_hal::delay_ms(1000);
            watchdog.feed();
        }

        // use gain 1 for input 2-3 (voltage)
        adc.set_full_scale_range(FullScaleRange::Within4_096V)
            .unwrap();
        arduino_hal::delay_ms(5);
        sum = 0;
        for _ in 0..N_AVG {
            sum += block!(adc.read(&mut channel::DifferentialA2A3)).unwrap() as i32;
            arduino_hal::delay_ms(1);
        }
        watchdog.feed();
        let adc_volt = (sum / N_AVG) as i16;

        if calibration {
            ufmt::uwrite!(&mut serial, "ADC volt: {}\r\n\r\n", adc_volt).ok();
            buf[0] = b'U';
            i16_disp(adc_volt, &mut buf);
            disp.set_intensity(0, 8).unwrap();
            disp.write_str(0, &buf, 0b00000000).unwrap();
            /*
                ufmt::uwrite!(&mut serial, "{}\r\n", unsafe {
                    core::str::from_utf8_unchecked(&buf)
                })
                .ok();
            */
            arduino_hal::delay_ms(1000);
            watchdog.feed();
        }

        if production {
            // ufmt::uwrite!(&mut serial, "*** volt interpolate()\r\n").ok();
            let volt_f = interpolate_f(&mut serial, adc_volt, &CAL_V);
            /*
            ufmt::uwrite!(
                &mut serial,
                "volt_f(*1000): {}\r\n",
                (volt_f * 1000.0) as i32
            )
            .ok();
            */

            // ufmt::uwrite!(&mut serial, "*** amps interpolate()\r\n").ok();
            let amps_f = interpolate_f(&mut serial, adc_amps - AMP_OFF, &CAL_I);
            /*
            ufmt::uwrite!(
                &mut serial,
                "amps_f(*1000): {}\r\n",
                (amps_f * 1000.0) as i32
            )
            .ok();
            */

            volt_amps_disp(volt_f, amps_f, &mut buf);
            disp.set_intensity(0, 8).unwrap();
            disp.write_str(0, &buf, 0b01000100).unwrap();
        }
    }
}

fn i16_disp(val: i16, buf: &mut [u8]) {
    if buf.len() < 8 {
        return;
    }
    let uval = val.unsigned_abs();
    buf[1] = if val.is_negative() { b'-' } else { b' ' };
    buf[2] = b'0' + ((uval / 10000) % 10) as u8;
    buf[3] = b'0' + ((uval / 1000) % 10) as u8;
    buf[5] = b'0' + ((uval / 100) % 10) as u8;
    buf[6] = b'0' + ((uval / 10) % 10) as u8;
    buf[7] = b'0' + (uval % 10) as u8;
}

fn volt_amps_disp(volt: f32, amps: f32, buf: &mut [u8]) {
    if buf.len() < 8 {
        return;
    }
    let volt_u = (volt * 100.0).clamp(0.0, 9999.0) as u16;
    let amps_u = (amps * 100.0).clamp(0.0, 999.0) as u16;
    let v10 = ((volt_u / 1000) % 10) as u8;
    buf[0] = if v10 > 0 { b'0' + v10 } else { b' ' };
    buf[1] = b'0' + ((volt_u / 100) % 10) as u8;
    buf[2] = b'0' + ((volt_u / 10) % 10) as u8;
    buf[3] = b'0' + (volt_u % 10) as u8;
    buf[4] = b' ';
    buf[5] = b'0' + ((amps_u / 100) % 10) as u8;
    buf[6] = b'0' + ((amps_u / 10) % 10) as u8;
    buf[7] = b'0' + (amps_u % 10) as u8;
}

fn interpolate_f(_p: &mut impl uWrite, mut x: i16, calibr: &[(i16, f32)]) -> f32 {
    x = if x < 0 { 0 } else { x };
    // ufmt::uwrite!(p, "x: {}\r\n", x).ok();
    for i in 0..calibr.len() {
        if x < calibr[i].0 {
            let floor_x = calibr[i - 1].0;
            // ufmt::uwrite!(p, "floor_x: {}\r\n", floor_x).ok();
            let floor_f = calibr[i - 1].1;
            // ufmt::uwrite!(p, "floor_f(*1000): {}\r\n", (floor_f * 1000.0) as i32).ok();
            let step_x = calibr[i].0 - floor_x;
            // ufmt::uwrite!(p, "step_x: {}\r\n", step_x).ok();
            let step_f = calibr[i].1 - floor_f;
            // ufmt::uwrite!(p, "step_f(*1000): {}\r\n", (step_f * 1000.0) as i32).ok();
            let delta_x = x - floor_x;
            // ufmt::uwrite!(p, "delta_x: {}\r\n", delta_x).ok();

            return floor_f + (delta_x as f32 / step_x as f32) * step_f;
        }
    }
    0.0
}

// EOF

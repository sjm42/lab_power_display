#![no_std]
#![no_main]

use ads1x1x::{channel, Ads1x1x, FullScaleRange, SlaveAddr};
use arduino_hal::{hal::wdt, prelude::*, spi};
use core::ops::*;
use embedded_hal::spi::MODE_0;
use max7219::*;
use nb::block;
use panic_halt as _;

const N_AVG: i32 = 16;

// ************************************************
// Our calibration data, based on real measurements

const CAL_V: [(i16, f32); 9] = [
    (3, 0.000),
    (1132, 1.490),
    (4322, 5.570),
    (8061, 10.370),
    (12863, 16.450),
    (17448, 22.300),
    (22028, 28.100),
    (24725, 31.500),
    (27422, 34.900),
];

const CAL_I: [(i16, f32); 9] = [
    (469, 0.0982),
    (889, 0.1983),
    (2152, 0.4985),
    (4264, 0.9982),
    (6404, 1.4987),
    (8561, 1.9989),
    (10780, 2.4994),
    (13040, 2.9993),
    (15300, 3.4992),
];

const OFF_I: [(f32, i16); 9] = [
    (0.0, 40),
    (2.0, 57),
    (5.0, 66),
    (10.0, 80),
    (15.0, 93),
    (20.0, 103),
    (25.0, 114),
    (31.0, 130),
    (37.0, 146),
];

// *** end of calibration data
// ***************************

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let mut watchdog = wdt::Wdt::new(dp.WDT, &dp.CPU.mcusr);
    watchdog.start(wdt::Timeout::Ms2000).unwrap();

    // NOTE: calibration mode is triggered with jumper in d4+d5
    let pins = arduino_hal::pins!(dp);
    let d4 = pins.d4.into_pull_up_input();
    let mut d5 = pins.d5.into_output();
    d5.set_low();

    let mut serial = arduino_hal::default_serial!(dp, pins, 57600);
    ufmt::uwrite!(&mut serial, "Lab power display\r\n").ok();

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
    disp.set_intensity(0, 8).unwrap();
    disp.clear_display(0).unwrap();

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

    let mut buf: [u8; 8] = [b' '; 8];
    loop {
        // use gain 16 for input 0-1 (current)
        adc.set_full_scale_range(FullScaleRange::Within0_256V)
            .unwrap();

        let mut sum: i32 = 0;
        for _ in 0..N_AVG {
            sum += block!(adc.read(&mut channel::DifferentialA0A1)).unwrap() as i32;
        }
        watchdog.feed();
        let adc_amps = (sum / N_AVG) as i16;

        if calibration {
            // ufmt::uwrite!(&mut serial, "ADC amps: {}\r\n", amps).ok();

            buf[0] = b'A';
            i16_disp(adc_amps, &mut buf);
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
        sum = 0;
        for _ in 0..N_AVG {
            sum += block!(adc.read(&mut channel::DifferentialA2A3)).unwrap() as i32;
        }
        watchdog.feed();
        let adc_volt = (sum / N_AVG) as i16;

        if calibration {
            // ufmt::uwrite!(&mut serial, "ADC volt: {}\r\n\r\n", volt).ok();

            buf[0] = b'U';
            i16_disp(adc_volt, &mut buf);
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
            let volt_f = interpolate_f(adc_volt, &CAL_V);
            let amp_off = interpolate_i(volt_f, &OFF_I);
            let amps_f = interpolate_f(adc_amps - amp_off, &CAL_I);
            volt_amps_disp(volt_f, amps_f, &mut buf);
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

fn interpolate_i(x: f32, calibr: &[(f32, i16)]) -> i16 {
    for i in 0..calibr.len() {
        let x_floor = calibr[i].0;
        if x > x_floor {
            let f_floor = calibr[i].1;
            let step_x = calibr[i + 1].0 - x_floor;
            let step_f = calibr[i + 1].1 - f_floor;
            let delta_x = x - x_floor;

            return f_floor + ((((delta_x / step_x) * step_f as f32) + 0.5) as i16);
        }
    }
    0
}

// here we have some trait & generics porn
fn interpolate_f<N1, N2>(x: N1, calibr: &[(N1, N2)]) -> N2
where
    N1: Copy + Add<Output = N1> + Sub<Output = N1> + Ord + Into<N2>,
    N2: Copy + Default + Add<Output = N2> + Sub<Output = N2> + Div<Output = N2> + Mul<Output = N2>,
{
    for i in 0..calibr.len() {
        let x_floor = calibr[i].0;
        if x > x_floor {
            let f_floor = calibr[i].1;
            let step_x = calibr[i + 1].0 - x_floor;
            let step_f = calibr[i + 1].1 - f_floor;
            let delta_x = x - x_floor;

            return f_floor + (delta_x.into() / step_x.into()) * step_f;
        }
    }
    N2::default()
}

// EOF

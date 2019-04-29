//! Platform-agnostic ADXL343 accelerometer driver which uses I2C via
//! [embedded-hal] and implements the [`Accelerometer` trait][trait]
//! from the `accelerometer` crate.
//!
//! <https://www.bosch-sensortec.com/bst/products/all_products/bmi160>
//!
//! > The BMI160 is a small, low power, low noise 16-bit inertial measurement unit designed
//! > for use in mobile applications like augmented reality or indoor navigation which require
//! > highly accurate, real-time sensor data.
//! >
//! > In full operation mode, with both the accelerometer and gyroscope enabled, the current
//! > consumption is typically 950 μA, enabling always-on applications in battery driven devices.
//! > It is available in a compact 14-pin 2.5 x 3.0 x 0.8 mm³ LGA package.
//! [embedded-hal]: https://docs.rs/embedded-hal
//! [trait]: https://docs.rs/accelerometer/latest/accelerometer/trait.Accelerometer.html

#![no_std]
#![deny(
 //   warnings,
    missing_docs,
    trivial_casts,
    trivial_numeric_casts,
    unsafe_code,
    unused_import_braces,
    unused_qualifications
)]
#![forbid(unsafe_code)]

extern crate embedded_hal as hal;

mod register;
use self::register::Register;

use embedded_hal::blocking::i2c::{Write, WriteRead};

/// BMI1160 I2C address.
/// Assumes ALT address pin low
pub const ADDRESS: u8 = 0x68;
//pub const ADDRESS:u8 = 0x69;

/// BMI160 driver
pub struct Bmi160<I2C> {
    /// Underlying I2C device
    i2c: I2C,
}

impl<I2C, E> Bmi160<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
{
    /// Create a new BMI160 driver from the given I2C peripheral
    ///
    /// Default
    pub fn new(i2c: I2C) -> Result<Self, E> {
        let bmi160 = Bmi160 { i2c };
        Ok(bmi160)
    }

    /// Get the chip ID
    pub fn get_chip_id(&mut self) -> Result<u8, E> {
        let input = [Register::CHIP_ID.addr()];
        let mut output = [0u8];
        self.i2c.write_read(ADDRESS, &input, &mut output)?;
        Ok(output[0])
    }

    /// Read The Data (Mag, Gyro, RHALL, Accel) from the Data Register
    pub fn read_data(&mut self) -> Result<Data, E> {
        let mut buffer = [0u8, 20];
        self.i2c.write_read(ADDRESS, &[Register::CMD.addr()], &mut buffer)?;
        Ok(Data::new_from_buffer(&mut buffer))
    }

    /// Resets and restarts the device.
    pub fn soft_reset(&mut self) -> Result<(), E> {
        Ok(())
    }

    /// Write to the given register
    // TODO: make this an internal API after enough functionality is wrapped
    pub fn write_register(&mut self, register: Register, value: u8) -> Result<(), E> {
        debug_assert!(!register.read_only(), "can't write to read-only register");
        self.i2c.write(ADDRESS, &[register.addr(), value])?;
        Ok(())
    }

    /// Write to a given register, then read the result
    // TODO: make this an internal API after enough functionality is wrapped
    pub fn write_read_register(&mut self, register: Register, buffer: &mut [u8]) -> Result<(), E> {
        self.i2c.write_read(ADDRESS, &[register.addr()], buffer)
    }
}

/// Raw Data Struct for the XYZ data returned from reading
/// the data register. The individual XYZ contain both
/// u8 for LSB and MSB.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(C)]
pub struct DataXYZRaw {
    /// X LSB
    pub x_lsb: u8,

    /// X MSB
    x_msb: u8,

    /// Y LSB
    y_lsb: u8,

    /// Y MSB
    y_msb: u8,

    /// Z LSB
    z_lsb: u8,

    /// Z MSB
    z_msb: u8,
}

/// The Raw Data structure returned from reading the 
/// data register.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(C)]
pub struct Data {
    /// Magnatometer XYZ Raw Data
    pub mag: DataXYZRaw,

    /// RHALL LSB Data
    pub rhall_lsb: u8,

    /// RHALL MSB Data
    pub rhall_msb: u8,

    /// Gyroscope XYZ Raw Data
    pub gyro: DataXYZRaw,

    /// Accelerometer XYZ Raw Data
    pub accel: DataXYZRaw,
}

impl Data {
    /// Returns a new Data struct from the data buffer returned
    /// from the Data register.
    pub fn new_from_buffer(buffer: &mut [u8]) -> Self {
        Data {
            mag:  DataXYZRaw {
                x_lsb: buffer[0],
                x_msb: buffer[1],
                y_lsb: buffer[2],
                y_msb: buffer[3],
                z_lsb: buffer[4],
                z_msb: buffer[5],
            },

            rhall_lsb: buffer[6],
            rhall_msb: buffer[7],

            gyro: DataXYZRaw {
                x_lsb: buffer[8],
                x_msb: buffer[9],
                y_lsb: buffer[10],
                y_msb: buffer[11],
                z_lsb: buffer[12],
                z_msb: buffer[13],
            },

            accel: DataXYZRaw {
                x_lsb: buffer[14],
                x_msb: buffer[15],
                y_lsb: buffer[16],
                y_msb: buffer[17],
                z_lsb: buffer[18],
                z_msb: buffer[19],
            }, 
        }
    }
}


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
}

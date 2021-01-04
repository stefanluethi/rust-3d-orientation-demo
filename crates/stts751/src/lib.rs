#![no_std]

/// Driver for the STTS751 I2C Temperature Sensor
/// based on the lsm6dso crate

mod register;
use register::*;

mod error;
use error::Error as TemperatureError;

use embedded_hal::blocking::i2c::{Write, WriteRead};
use core::fmt::{Debug};
use core::convert::{TryInto};

/// Temperature errors, generic around another error type `E` representing
/// an (optional) cause of this error.
#[derive(Debug)]
pub enum Error<E> {
    /// I²C bus error
    I2C(E),

    /// Invalid data rate selection
    InvalidDataRate,

    /// Invalid operating mode selection
    InvalidMode,

    /// Invalid full-scale selection
    InvalidRange,

    /// Attempted to write to a read-only register
    WriteToReadOnly,

    /// Invalid address provided
    WrongAddress,
}


/// `STTS751` driver (only basic functionality)
pub struct Stts751<I2C> {
    /// Underlying I2C interface
    i2c: I2C,
}

#[allow(dead_code)]
impl<I2C, E> Stts751<I2C>
    where
        I2C: WriteRead<Error = E> + Write<Error = E>,
        E: Debug,
{
    /// Create a new STTS751 driver from the given I2C peripheral. Default is
    /// 10bit, 1Hz, Continuous.
    pub fn new(i2c: I2C) -> Result<Self, Error<E>> {
        let stts751 = Stts751 {
            i2c,
        };

        // we'll just use the devices default settings for this

        Ok(stts751)
    }

    /// Modify a register's value. Read the current value of the register,
    /// update the value with the provided function, and set the register to
    /// the return value.
    fn modify_register<F>(&mut self, register: Register, f: F) -> Result<(), Error<E>>
        where
            F: FnOnce(u8) -> u8,
    {
        let value = self.read_register(register)?;

        self.write_register(register, f(value))
    }

    /// Clear the given bits in the given register.
    fn register_clear_bits(&mut self, reg: Register, bits: u8) -> Result<(), Error<E>> {
        self.modify_register(reg, |v| v & !bits)
    }

    /// Set the given bits in the given register.
    fn register_set_bits(&mut self, reg: Register, bits: u8) -> Result<(), Error<E>> {
        self.modify_register(reg, |v| v | bits)
    }

    /// Set or clear the given given bits in the given register, depending on
    /// the value of `set`.
    fn register_xset_bits(&mut self, reg: Register, bits: u8, set: bool) -> Result<(), Error<E>> {
        if set {
            self.register_set_bits(reg, bits)
        } else {
            self.register_clear_bits(reg, bits)
        }
    }

    /// Read the magnetic field values for all 3 axis
    fn read_temp_bytes(&mut self) -> Result<[u8; 2], Error<E>> {
        let mut data = [0u8; 2];

        data[0] = self.read_register(Register::TEMPERATURE_LOW)?;
        data[1] = self.read_register(Register::TEMPERATURE_HIGH)?;

        Ok(data)
    }

    /// Write a byte to the given register.
    fn write_register(&mut self, register: Register, value: u8) -> Result<(), Error<E>> {
        if register.read_only() {
            return Err(Error::WriteToReadOnly);
        }

        self.i2c
            .write(DEVICE_ADDRESS, &[register.addr(), value])
            .map_err(Error::I2C)
    }

    /// Read a byte from the given register.
    fn read_register(&mut self, register: Register) -> Result<u8, Error<E>> {
        let mut data = [0];

        self.i2c
            .write_read(DEVICE_ADDRESS, &[register.addr()], &mut data)
            .map_err(Error::I2C)
            .and(Ok(data[0]))
    }
}

pub trait Temperature {
    type Error: Debug;
    fn temp_norm(&mut self) -> Result<f32, TemperatureError<Self::Error>>;
}
impl<I2C, E> Temperature for Stts751<I2C>
    where
        I2C: WriteRead<Error = E> + Write<Error = E>,
        E: Debug,
{
    type Error = Error<E>;

    /// Get normalized ±gauss reading from the magnetometer.
    fn temp_norm(&mut self) -> Result<f32, TemperatureError<Self::Error>> {
        // factory calibrated sensitivity
        let scale = 1.0/256.0;

        let temp_raw = self.temp_raw()?;
        let temp = temp_raw as f32 * scale;

        Ok(temp)
    }
}


pub trait RawTemperature<I> {
    type Error: Debug;
    fn temp_raw(&mut self) -> Result<I, TemperatureError<Self::Error>>;
}
impl<I2C, E> RawTemperature<i16> for Stts751<I2C>
    where
        I2C: WriteRead<Error = E> + Write<Error = E>,
        E: Debug,
{
    type Error = Error<E>;

    /// Get raw magnetic field data from the magnetometer.
    fn temp_raw(&mut self) -> Result<i16, TemperatureError<Self::Error>> {
        let temp_bytes = self.read_temp_bytes()?;

        let temp = i16::from_le_bytes(temp_bytes.try_into().unwrap());

        Ok(temp)
    }
}

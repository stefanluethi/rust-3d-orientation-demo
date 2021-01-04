#![no_std]

/// Driver for the LIS2MDL I2C Magnetometer Sensor
/// based on the lsm6dso crate

mod register;
use register::*;

mod error;
use error::Error as MagnetometerError;

use embedded_hal::blocking::i2c::{Write, WriteRead};
use core::fmt::{Debug};
use core::convert::{TryInto, TryFrom};
use micromath::vector::{F32x3, I16x3, Vector};

/// Magnetometer errors, generic around another error type `E` representing
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


/// `LIS2MDL` driver (only basic functionality)
pub struct Lis2mdl<I2C> {
    /// Underlying I2C interface
    i2c: I2C,
}

#[allow(dead_code)]
impl<I2C, E> Lis2mdl<I2C>
    where
        I2C: WriteRead<Error = E> + Write<Error = E>,
        E: Debug,
{
    /// Create a new LIS3MDL driver from the given I2C peripheral. Default is
    /// 50Hz Continuous.
    pub fn new(i2c: I2C) -> Result<Self, Error<E>> {
        let mut lis2mdl = Lis2mdl{
            i2c,
        };

        // if lis2mdl.get_device_id()? != DEVICE_ID {
        //     return Err(Error::WrongAddress);
        // }

        // Enable Block Data Update
        lis2mdl.register_set_bits(Register::CFG_REG_C, BDU)?;

        // Set Output Data Rate
        //lis2mdl.set_data_rate(DataRate::Hz_50);

        // Set sensor mode
        //lis2mdl.register_set_bits(Register::CFG_REG_A, OFF_CANC);

        // Enable temperature compensation
        //lis2mdl.register_set_bits(Register::CFG_REG_A, COMP_TEMP_EN);

        /* Set device in continuous mode */
        //lis2mdl.set_mode(Mode::Continuous);

        Ok(lis2mdl)
    }

    /// `WHO_AM_I` register.
    pub fn get_device_id(&mut self) -> Result<u8, Error<E>> {
        self.read_register(Register::WHO_AM_I)
    }

    /// Operating mode selection.
    pub fn set_mode(&mut self, mode: Mode) -> Result<(), Error<E>> {
        self.register_set_bits(Register::CFG_REG_A, mode.bits())?;
        self.register_clear_bits(Register::CFG_REG_A, (!mode.bits()) & 0x03)?;
        Ok(())
    }

    /// Output data rate selection
    pub fn set_data_rate(&mut self, data_rate: DataRate) -> Result<(), Error<E>> {
        self.register_set_bits(Register::CFG_REG_A, data_rate.bits() << 2)?;
        self.register_clear_bits(Register::CFG_REG_A, (!data_rate.bits() & 0x03) << 2)?;
        Ok(())
    }

    /// Read the current data selection rate.
    pub fn get_datarate(&mut self) -> Result<DataRate, Error<E>> {
        let ctrl = self.read_register(Register::CFG_REG_A)?;
        let odr = (ctrl & 0b1100) >> 2;

        DataRate::try_from(odr).map_err(|_| Error::InvalidDataRate)
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
    fn read_mag_bytes(&mut self) -> Result<[u8; 6], Error<E>> {
        let mut data = [0u8; 6];

        self.i2c
            .write_read(DEVICE_ADDRESS, &[Register::OUTX_L_REG.addr()], &mut data)
            .map_err(Error::I2C)
            .and(Ok(data))
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

pub trait Magnetometer {
    type Error: Debug;
    fn mag_norm(&mut self) -> Result<F32x3, MagnetometerError<Self::Error>>;
    fn sample_rate(&mut self) -> Result<f32, MagnetometerError<Self::Error>>;
}
impl<I2C, E> Magnetometer for Lis2mdl<I2C>
    where
        I2C: WriteRead<Error = E> + Write<Error = E>,
        E: Debug,
{
    type Error = Error<E>;

    /// Get normalized ±gauss reading from the magnetometer.
    fn mag_norm(&mut self) -> Result<F32x3, MagnetometerError<Self::Error>> {
        // factory calibrated sensitivity
        let scale = 0.0015;

        let mag_raw = self.mag_raw()?;
        let x = mag_raw.x as f32 * scale;
        let y = mag_raw.y as f32 * scale;
        let z = mag_raw.z as f32 * scale;

        Ok(F32x3::new(x, y, z))
    }

    /// Get the sample rate of the magnetometer data.
    fn sample_rate(&mut self) -> Result<f32, MagnetometerError<Self::Error>> {
        Ok(self.get_datarate()?.sample_rate())
    }
}


pub trait RawMagnetometer<V: Vector> {
    type Error: Debug;
    fn mag_raw(&mut self) -> Result<V, MagnetometerError<Self::Error>>;
}
impl<I2C, E> RawMagnetometer<I16x3> for Lis2mdl<I2C>
    where
        I2C: WriteRead<Error = E> + Write<Error = E>,
        E: Debug,
{
    type Error = Error<E>;

    /// Get raw magnetic field data from the magnetometer.
    fn mag_raw(&mut self) -> Result<I16x3, MagnetometerError<Self::Error>> {
        let mag_bytes = self.read_mag_bytes()?;

        let x = i16::from_le_bytes(mag_bytes[0..2].try_into().unwrap());
        let y = i16::from_le_bytes(mag_bytes[2..4].try_into().unwrap());
        let z = i16::from_le_bytes(mag_bytes[4..6].try_into().unwrap());

        Ok(I16x3::new(x, y, z))
    }
}

#![no_std]

/// Driver for the LSM6DSO I2C Acceleration & Gyro Sensor
/// based on https://github.com/BenBergman/lis3dh-rs

use core::convert::{TryFrom, TryInto};
use core::fmt::{Debug};

pub use accelerometer;
use accelerometer::error::Error as AccelerometerError;
use accelerometer::vector::{F32x3, I16x3, Vector};
pub use accelerometer::{Accelerometer, RawAccelerometer};
use embedded_hal::blocking::i2c::{Write, WriteRead};

mod register;
pub use register::*;

/// Accelerometer errors, generic around another error type `E` representing
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

/// `LSM6DSO` driver.
pub struct Lsm6dso<I2C> {
    /// Underlying I²C device
    i2c: I2C,

    /// Current I²C slave address
    address: u8,
}

#[allow(dead_code)]
impl<I2C, E> Lsm6dso<I2C>
    where
        I2C: WriteRead<Error = E> + Write<Error = E>,
        E: Debug,
{
    /// Create a new LSM6DSO driver from the given I2C peripheral. Default is
    /// Hz_400 HighResolution.
    pub fn new(i2c: I2C, address: SlaveAddr) -> Result<Self, Error<E>> {
        let mut lsm6dso = Lsm6dso {
            i2c,
            address: address.addr(),
        };

        if lsm6dso.get_device_id()? != DEVICE_ID {
            return Err(Error::WrongAddress);
        }

        //lsm6dso.register_set_bits(Register::CTRL3_C, SW_RESET);
        //todo wait for reset

        // Disable I3C interface
        lsm6dso.register_clear_bits(Register::CTRL9_XL, 0b000_0010)?; // I3C_DISABLE

        // Block data update
        lsm6dso.register_set_bits(Register::CTRL3_C, BDU)?;

        // Enable accelerometer
        lsm6dso.set_acc_mode(AccMode::HighPerformance)?;
        lsm6dso.set_acc_datarate(AccDataRate::Hz_416)?;

        // Enable Gyroscope
        lsm6dso.set_gyro_datarate(GyroDataRate::Hz_416)?;

        Ok(lsm6dso)
    }

    /// `WHO_AM_I` register.
    pub fn get_device_id(&mut self) -> Result<u8, Error<E>> {
        self.read_register(Register::WHO_AM_I)
    }

    /// Operating mode selection.
    pub fn set_acc_mode(&mut self, mode: AccMode) -> Result<(), Error<E>> {
        match mode {
            AccMode::Normal => {
                self.register_clear_bits(Register::CTRL6_C, 0b0001_0000)?;
            }
            AccMode::HighPerformance => {
                self.register_set_bits(Register::CTRL6_C, 0b0001_0000)?;
            }
        }

        Ok(())
    }

    /// Read the current operating mode
    pub fn get_acc_mode(&mut self) -> Result<AccMode, Error<E>> {
        let ctrl = self.read_register(Register::CTRL6_C)?;

        let is_hp_set = (ctrl >> 4) & 0x01 != 0;

        let mode = match is_hp_set {
            true => AccMode::HighPerformance,
            false => AccMode::Normal,
        };

        Ok(mode)
    }

    /// Data rate selection.
    pub fn set_acc_datarate(&mut self, datarate: AccDataRate) -> Result<(), Error<E>> {
        self.modify_register(Register::CTRL1_XL, |mut ctrl| {
            ctrl &= !ODR_MASK;
            // Write in new output data rate
            ctrl |= datarate.bits() << 4;

            ctrl
        })
    }

    /// Read the current data selection rate.
    pub fn get_acc_datarate(&mut self) -> Result<AccDataRate, Error<E>> {
        let ctrl = self.read_register(Register::CTRL1_XL)?;
        let odr = (ctrl & ODR_MASK) >> 4;

        AccDataRate::try_from(odr).map_err(|_| Error::InvalidDataRate)
    }

    /// Data rate selection.
    pub fn set_gyro_datarate(&mut self, datarate: GyroDataRate) -> Result<(), Error<E>> {
        self.modify_register(Register::CTRL2_G, |mut ctrl| {
            ctrl &= !ODR_MASK;
            // Write in new output data rate
            ctrl |= datarate.bits() << 4;

            ctrl
        })
    }

    /// Read the current data selection rate.
    pub fn get_gyro_datarate(&mut self) -> Result<GyroDataRate, Error<E>> {
        let ctrl = self.read_register(Register::CTRL2_G)?;
        let odr = (ctrl & ODR_MASK) >> 4;

        GyroDataRate::try_from(odr).map_err(|_| Error::InvalidDataRate)
    }

    /// Full-scale selection.
    pub fn set_acc_range(&mut self, range: AccRange) -> Result<(), Error<E>> {
        self.modify_register(Register::CTRL1_XL, |mut ctrl| {
            // Mask off lowest 4 bits
            ctrl &= !FS_MASK;
            // Write in new full-scale
            ctrl |= range.bits() << 1;

            ctrl
        })
    }

    /// Read the current full-scale.
    pub fn get_acc_range(&mut self) -> Result<AccRange, Error<E>> {
        let ctrl = self.read_register(Register::CTRL1_XL)?;
        let fs = (ctrl & FS_MASK) >> 1;

        AccRange::try_from(fs).map_err(|_| Error::InvalidRange)
    }

    /// Full-scale selection.
    pub fn set_gyro_range(&mut self, range: GyroRange) -> Result<(), Error<E>> {
        self.modify_register(Register::CTRL2_G, |mut ctrl| {
            // Mask off lowest 4 bits
            ctrl &= !0b0000_1110;
            // Write in new full-scale
            ctrl |= range.bits() << 1;

            ctrl
        })
    }

    /// Read the current full-scale.
    pub fn get_gyro_range(&mut self) -> Result<GyroRange, Error<E>> {
        let ctrl = self.read_register(Register::CTRL2_G)?;
        let fs = (ctrl & 0b0000_1110) >> 1;

        GyroRange::try_from(fs).map_err(|_| Error::InvalidRange)
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

    /// Read from the registers for each of the 3 axes.
    fn read_accel_bytes(&mut self) -> Result<[u8; 6], Error<E>> {
        let mut data = [0u8; 6];

        self.i2c
            .write_read(self.address, &[Register::OUTX_L_A.addr()], &mut data)
            .map_err(Error::I2C)
            .and(Ok(data))
    }

    /// Read from the registers for each of the 3 axes.
    fn read_gyro_bytes(&mut self) -> Result<[u8; 6], Error<E>> {
        let mut data = [0u8; 6];

        self.i2c
            .write_read(self.address, &[Register::OUTX_L_G.addr()], &mut data)
            .map_err(Error::I2C)
            .and(Ok(data))
    }

    /// Write a byte to the given register.
    fn write_register(&mut self, register: Register, value: u8) -> Result<(), Error<E>> {
        if register.read_only() {
            return Err(Error::WriteToReadOnly);
        }

        self.i2c
            .write(self.address, &[register.addr(), value])
            .map_err(Error::I2C)
    }

    /// Read a byte from the given register.
    fn read_register(&mut self, register: Register) -> Result<u8, Error<E>> {
        let mut data = [0];

        self.i2c
            .write_read(self.address, &[register.addr()], &mut data)
            .map_err(Error::I2C)
            .and(Ok(data[0]))
    }
}

//------------------------------------------------------------------------------
impl<I2C, E> Accelerometer for Lsm6dso<I2C>
    where
        I2C: WriteRead<Error = E> + Write<Error = E>,
        E: Debug,
{
    type Error = Error<E>;

    /// Get normalized ±g reading from the accelerometer. You should be reading
    /// based on data ready interrupt or if reading in a tight loop you should
    /// waiting for `is_data_ready`.
    fn accel_norm(&mut self) -> Result<F32x3, AccelerometerError<Self::Error>> {
        // The official driver from ST was used as a reference.
        // https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/master/lsm6dso_STdC
        let range = self.get_acc_range()?;

        // factory calibrated sensitivity
        let scale = match range {
            AccRange::G2 => 0.000061,
            AccRange::G4 => 0.000122,
            AccRange::G8 => 0.000244,
            AccRange::G16 => 0.000488,
        };

        let acc_raw = self.accel_raw()?;
        let x = acc_raw.x as f32 * scale;
        let y = acc_raw.y as f32 * scale;
        let z = acc_raw.z as f32 * scale;

        Ok(F32x3::new(x, y, z))
    }

    /// Get the sample rate of the accelerometer data.
    fn sample_rate(&mut self) -> Result<f32, AccelerometerError<Self::Error>> {
        Ok(self.get_acc_datarate()?.sample_rate())
    }
}

impl<I2C, E> RawAccelerometer<I16x3> for Lsm6dso<I2C>
    where
        I2C: WriteRead<Error = E> + Write<Error = E>,
        E: Debug,
{
    type Error = Error<E>;

    /// Get raw acceleration data from the accelerometer. You should be reading
    /// based on data ready interrupt or if reading in a tight loop you should
    /// waiting for `is_data_ready`.
    fn accel_raw(&mut self) -> Result<I16x3, AccelerometerError<Self::Error>> {
        let accel_bytes = self.read_accel_bytes()?;

        let x = i16::from_le_bytes(accel_bytes[0..2].try_into().unwrap());
        let y = i16::from_le_bytes(accel_bytes[2..4].try_into().unwrap());
        let z = i16::from_le_bytes(accel_bytes[4..6].try_into().unwrap());

        Ok(I16x3::new(x, y, z))
    }
}

//------------------------------------------------------------------------------
pub trait Gyro {
    type Error: Debug;
    fn gyro_norm(&mut self) -> Result<F32x3, AccelerometerError<Self::Error>>;
    fn sample_rate(&mut self) -> Result<f32, AccelerometerError<Self::Error>>;
}
impl<I2C, E> Gyro for Lsm6dso<I2C>
    where
        I2C: WriteRead<Error = E> + Write<Error = E>,
        E: Debug,
{
    type Error = Error<E>;

    /// Get normalized ±gauss reading from the magnetometer.
    fn gyro_norm(&mut self) -> Result<F32x3, AccelerometerError<Self::Error>> {
        let range = self.get_gyro_range()?;

        // factory calibrated sensitivity
        let scale = match range {
            GyroRange::DPS125 => 0.004375,
            GyroRange::DPS250 => 0.00875,
            GyroRange::DPS500 => 0.0175,
            GyroRange::DPS1000 => 0.035,
            GyroRange::DPS2000 => 0.070,
        };

        let gyro_raw = self.gyro_raw()?;
        let x = gyro_raw.x as f32 * scale;
        let y = gyro_raw.y as f32 * scale;
        let z = gyro_raw.z as f32 * scale;

        Ok(F32x3::new(x, y, z))
    }

    /// Get the sample rate of the accelerometer data.
    fn sample_rate(&mut self) -> Result<f32, AccelerometerError<Self::Error>> {
        Ok(self.get_acc_datarate()?.sample_rate())
    }
}


pub trait RawGyro<V: Vector> {
    type Error: Debug;
    fn gyro_raw(&mut self) -> Result<V, AccelerometerError<Self::Error>>;
}
impl<I2C, E> RawGyro<I16x3> for Lsm6dso<I2C>
    where
        I2C: WriteRead<Error = E> + Write<Error = E>,
        E: Debug,
{
    type Error = Error<E>;

    /// Get raw magnetic field data from the magnetometer.
    fn gyro_raw(&mut self) -> Result<I16x3, AccelerometerError<Self::Error>> {
        let gyro_bytes = self.read_gyro_bytes()?;

        let x = i16::from_le_bytes(gyro_bytes[0..2].try_into().unwrap());
        let y = i16::from_le_bytes(gyro_bytes[2..4].try_into().unwrap());
        let z = i16::from_le_bytes(gyro_bytes[4..6].try_into().unwrap());

        Ok(I16x3::new(x, y, z))
    }
}
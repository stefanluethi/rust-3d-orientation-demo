//! based on https://github.com/BenBergman/lis3dh-rs

#![no_std]

use core::convert::{TryFrom, TryInto};
use core::fmt::{Debug};

pub use accelerometer;
use accelerometer::error::Error as AccelerometerError;
use accelerometer::vector::{F32x3, I16x3};
pub use accelerometer::{Accelerometer, RawAccelerometer};
use embedded_hal::blocking::i2c::{Write, WriteRead};

mod register;
use register::*;
pub use register::{DataRate, DataStatus, Mode, Range, SlaveAddr};

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
        //lsm6dso.register_set_bits(Register::CTRL3_C, BDU);

        lsm6dso.set_mode(Mode::HighPerformance)?;

        lsm6dso.set_datarate(DataRate::Hz_416)?;

        // Enable ADCs.
        //lsm6dso.write_register(Register::TEMP_CFG, ADC_EN)?;

        Ok(lsm6dso)
    }

    /// `WHO_AM_I` register.
    pub fn get_device_id(&mut self) -> Result<u8, Error<E>> {
        self.read_register(Register::WHO_AM_I)
    }

    /// X,Y,Z-axis enable. todo obsolete?
    /// `CTRL_REG1`: `Xen`, `Yen`, `Zen`
    // fn enable_axis(&mut self, (x, y, z): (bool, bool, bool)) -> Result<(), Error<E>> {
    //     self.modify_register(Register::CTRL1, |mut ctrl1| {
    //         ctrl1 &= !(X_EN | Y_EN | Z_EN); // disable all axes
    //
    //         ctrl1 |= if x { X_EN } else { 0 };
    //         ctrl1 |= if y { Y_EN } else { 0 };
    //         ctrl1 |= if z { Z_EN } else { 0 };
    //
    //         ctrl1
    //     })
    // }

    /// Operating mode selection.
    pub fn set_mode(&mut self, mode: Mode) -> Result<(), Error<E>> {
        match mode {
            Mode::Normal => {
                self.register_clear_bits(Register::CTRL6_C, 0b0001_0000)?;
            }
            Mode::HighPerformance => {
                self.register_set_bits(Register::CTRL6_C, 0b0001_0000)?;
            }
        }

        Ok(())
    }

    /// Read the current operating mode
    pub fn get_mode(&mut self) -> Result<Mode, Error<E>> {
        let ctrl = self.read_register(Register::CTRL6_C)?;

        let is_hp_set = (ctrl >> 4) & 0x01 != 0;



        let mode = match is_hp_set {
            true => Mode::HighPerformance,
            false => Mode::Normal,
            _ => return Err(Error::InvalidMode),
        };

        Ok(mode)
    }

    /// Data rate selection.
    pub fn set_datarate(&mut self, datarate: DataRate) -> Result<(), Error<E>> {
        self.modify_register(Register::CTRL1_XL, |mut ctrl| {
            ctrl &= !ODR_MASK;
            // Write in new output data rate
            ctrl |= datarate.bits() << 4;

            ctrl
        })
    }

    /// Read the current data selection rate.
    pub fn get_datarate(&mut self) -> Result<DataRate, Error<E>> {
        let ctrl = self.read_register(Register::CTRL1_XL)?;
        let odr = (ctrl & ODR_MASK) >> 4;

        DataRate::try_from(odr).map_err(|_| Error::InvalidDataRate)
    }

    /// Full-scale selection.
    pub fn set_range(&mut self, range: Range) -> Result<(), Error<E>> {
        self.modify_register(Register::CTRL1_XL, |mut ctrl| {
            // Mask off lowest 4 bits
            ctrl &= !FS_MASK;
            // Write in new full-scale
            ctrl |= range.bits() << 1;

            ctrl
        })
    }

    /// Read the current full-scale.
    pub fn get_range(&mut self) -> Result<Range, Error<E>> {
        let ctrl = self.read_register(Register::CTRL6_C)?;
        let fs = (ctrl & FS_MASK) >> 1;

        Range::try_from(fs).map_err(|_| Error::InvalidRange)
    }

    /// Set `REFERENCE` register. todo
    // pub fn set_ref(&mut self, reference: u8) -> Result<(), Error<E>> {
    //     self.write_register(Register::REFERENCE, reference)
    // }

    /// Read the `REFERENCE` register. todo
    // pub fn get_ref(&mut self) -> Result<u8, Error<E>> {
    //     self.read_register(Register::REFERENCE)
    // }

    /// Accelerometer data-available status. todo
    // pub fn get_status(&mut self) -> Result<DataStatus, Error<E>> {
    //     let stat = self.read_register(Register::STATUS)?;
    //
    //     Ok(DataStatus {
    //         zyxor: (stat & ZYXOR) != 0,
    //         xyzor: ((stat & XOR) != 0, (stat & YOR) != 0, (stat & ZOR) != 0),
    //         zyxda: (stat & ZYXDA) != 0,
    //         xyzda: ((stat & XDA) != 0, (stat & YDA) != 0, (stat & ZDA) != 0),
    //     })
    // }

    /// Convenience function for `STATUS_REG` to confirm all three X, Y and
    /// Z-axis have new data available for reading by accel_raw and associated
    /// function calls.
    // pub fn is_data_ready(&mut self) -> Result<bool, Error<E>> {
    //     let value = self.get_status()?;
    //
    //     Ok(value.zyxda)
    // }

    /// Temperature sensor enable. todo
    /// `TEMP_CGF_REG`: `TEMP_EN`, the BDU bit in `CTRL_REG4` is also set.
    // pub fn enable_temp(&mut self, enable: bool) -> Result<(), Error<E>> {
    //     self.register_xset_bits(Register::TEMP_CFG, ADC_EN & TEMP_EN, enable)?;
    //
    //     // enable block data update (required for temp reading)
    //     if enable {
    //         self.register_xset_bits(Register::CTRL4, BDU, true)?;
    //     }
    //
    //     Ok(())
    // }

    /// Raw temperature sensor data as `i16`. The temperature sensor __must__
    /// be enabled via `enable_temp` prior to reading. todo
    // pub fn get_temp_out(&mut self) -> Result<i16, Error<E>> {
    //     let out_l = self.read_register(Register::OUT_ADC3_L)?;
    //     let out_h = self.read_register(Register::OUT_ADC3_H)?;
    //
    //     Ok(i16::from_le_bytes([out_l, out_h]))
    // }

    /// Temperature sensor data converted to `f32`. Output is in degree
    /// celsius. The temperature sensor __must__ be enabled via `enable_temp`
    /// prior to reading. todo
    // pub fn get_temp_outf(&mut self) -> Result<f32, Error<E>> {
    //     let temp_out = self.get_temp_out()?;
    //
    //     Ok(temp_out as f32 / 256.0 + 25.0)
    // }

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
        let range = self.get_range()?;

        // factory calibrated sensitivity
        let scale = match range {
            Range::G2 => 0.000061,
            Range::G4 => 0.000122,
            Range::G8 => 0.000244,
            Range::G16 => 0.000488,
        };

        let acc_raw = self.accel_raw()?;
        let x = acc_raw.x as f32 * scale;
        let y = acc_raw.y as f32 * scale;
        let z = acc_raw.z as f32 * scale;

        Ok(F32x3::new(x, y, z))
    }

    /// Get the sample rate of the accelerometer data.
    fn sample_rate(&mut self) -> Result<f32, AccelerometerError<Self::Error>> {
        Ok(self.get_datarate()?.sample_rate())
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
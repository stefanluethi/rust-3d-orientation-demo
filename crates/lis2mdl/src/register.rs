use num_enum::TryFromPrimitive;

/// Enumerate all device registers.
#[allow(dead_code, non_camel_case_types)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Register {
    OFFSET_X_REG_L = 0x45,
    OFFSET_X_REG_H = 0x46,
    OFFSET_Y_REG_L = 0x47,
    OFFSET_Y_REG_H = 0x48,
    OFFSET_Z_REG_L = 0x49,
    OFFSET_Z_REG_H = 0x4A,
    WHO_AM_I = 0x4F,
    CFG_REG_A = 0x60,
    CFG_REG_B = 0x61,
    CFG_REG_C = 0x62,
    INT_CRTL_REG = 0x63,
    INT_SOURCE_REG = 0x64,
    INT_THS_L_REG = 0x65,
    INT_THS_H_REG = 0x66,
    STATUS_REG = 0x67,
    OUTX_L_REG = 0x68,
    OUTX_H_REG = 0x69,
    OUTY_L_REG = 0x6A,
    OUTY_H_REG = 0x6B,
    OUTZ_L_REG = 0x6C,
    OUTZ_H_REG = 0x6D,
    TEMP_OUT_L_REG = 0x6E,
    TEMP_OUT_H_REG = 0x6F,
}


impl Register {
    /// Get register address
    pub fn addr(self) -> u8 {
        self as u8
    }

    /// Is the register read-only?
    pub fn read_only(self) -> bool {
        match self {
            Register::WHO_AM_I
            | Register::INT_SOURCE_REG
            | Register::STATUS_REG
            | Register::OUTX_L_REG
            | Register::OUTX_H_REG
            | Register::OUTY_L_REG
            | Register::OUTY_H_REG
            | Register::OUTZ_L_REG
            | Register::OUTZ_H_REG
            | Register::TEMP_OUT_L_REG
            | Register::TEMP_OUT_H_REG => true,
            _ => false,
        }
    }
}

/// Output data rate.
#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum DataRate {
    /// 100Hz
    Hz_100 = 0b11,

    /// 50Hz
    Hz_50 = 0b10,

    /// 20Hz
    Hz_20 = 0b01,

    /// 10Hz
    Hz_10 = 0b00,
}

impl DataRate {
    pub fn bits(self) -> u8 {
        self as u8
    }

    pub fn sample_rate(self) -> f32 {
        match self {
            DataRate::Hz_100 => 100.0,
            DataRate::Hz_50 => 50.0,
            DataRate::Hz_20 => 20.0,
            DataRate::Hz_10 => 10.0,
        }
    }
}

/// Operating mode. todo
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Mode {
    /// Continuous mode
    Continuous = 0b00,

    /// Single shot mode
    Single = 0b01,

    /// Idle mode
    Idle = 0b11,
}

impl Mode {
    pub fn bits(self) -> u8 {
        self as u8
    }
}


// slave address
pub const DEVICE_ADDRESS: u8 = 0x1E;

// === WHO_AMI_I ===

/// `WHO_AM_I` device identification register
pub const DEVICE_ID: u8 = 0x3C;

// some registers we need for this demonstration
// todo: replace with a type safe register model

// CFG_REG_A
pub const COMP_TEMP_EN: u8 = 0b1000_0000;
// CFG_REG_B
pub const OFF_CANC: u8 = 0b0000_0010;
// CFG_REG_C
pub const BDU: u8 = 0b0010_0000;




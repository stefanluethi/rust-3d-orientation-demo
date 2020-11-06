use num_enum::TryFromPrimitive;

/// Possible I²C slave addresses.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum SlaveAddr {
    /// Default slave address (`0x6A`)
    Default = 0x6A,

    /// Alternate slave address (`0x6B`)
    Alternate = 0x6B,
}

impl SlaveAddr {
    pub fn addr(self) -> u8 {
        return self as u8;
    }
}

/// Enumerate all device registers.
#[allow(dead_code, non_camel_case_types)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Register {
    FUNC_CFG_ACCESS = 0x01,
    PIN_CTRL = 0x02,
    FIFO_CTRL1 = 0x07,
    FIFO_CTRL2 = 0x08,
    FIFO_CTRL3 = 0x09,
    FIFO_CTRL4 = 0x0A,
    COUNTER_BDR_REG1 = 0x0B,
    COUNTER_BDR_REG2 = 0x0C,
    WHO_AM_I = 0x0F,
    CTRL1_XL = 0x10,
    CTRL2_G = 0x11,
    CTRL3_C = 0x12,
    CTRL4_C = 0x13,
    CTRL5_C = 0x14,
    CTRL6_C = 0x15,
    CTRL7_G = 0x16,
    CTRL8_XL = 0x17,
    CTRL9_XL = 0x18,
    CTRL10_C = 0x19,
    ALL_INT_SRC = 0x1A,
    WAKE_UP_SRC = 0x1B,
    TAP_SRC = 0x1C,
    D6D_SRC = 0x1D,
    STATUS_REG = 0x1E,
    OUT_TEMP_L = 0x20,
    OUT_TEMP_H = 0x21,
    OUTX_L_G = 0x22,
    OUTX_H_G = 0x23,
    OUTY_L_G = 0x24,
    OUTY_H_G = 0x25,
    OUTZ_L_G = 0x26,
    OUTZ_H_G = 0x27,
    OUTX_L_A = 0x28,
    OUTX_H_A = 0x29,
    OUTY_L_A = 0x2A,
    OUTY_H_A = 0x2B,
    OUTZ_L_A = 0x2C,
    OUTZ_H_A = 0x2D,
    EMB_FUNC_STATUS_MAINPAGE = 0x35,
    FSM_STATUS_A_MAINPAGE = 0x36,
    FSM_STATUS_B_MAINPAGE = 0x37,
    STATUS_MASTER_MAINPAGE = 0x39,
    FIFO_STATUS1 = 0x3A,
    FIFO_STATUS2 = 0x3B,
    TIME_LATENCY = 0x3C,
    TIMESTAMP0 = 0x40,
    TIMESTAMP1 = 0x41,
    TIMESTAMP2 = 0x042,
    TIMESTAMP3 = 0x43,
    TAP_CFG0 = 0x56,
    TAP_CFG1 = 0x57,
    TAP_CFG2 = 0x58,
    TAP_THS_6D = 0x59,
    INT_DUR2 = 0x5A,
    WAKE_UP_THS = 0x5B,
    WAKE_UP_DUR = 0x5C,
    FREE_FALL = 0x5D,
    MD1_CFG = 0x5E,
    MD2_CFG = 0x5F,
    I3C_BUS_AVB = 0x62,
    INTERNAL_FREQ_FINE = 0x63,
    INT_OIS = 0x6F,
    CTRL1_OIS = 0x70,
    CTRL2_OIS = 0x71,
    CTRL3_OIS = 0x72,
    X_OFS_USR = 0x73,
    Y_OFS_USR = 0x74,
    Z_OFS_USR = 0x75,
    FIFO_DATA_OUT_TAG = 0x78,
    FIFO_DATA_OUT_X_L = 0x79,
    FIFO_DATA_OUT_X_H = 0x7A,
    FIFO_DATA_OUT_Y_L = 0x7B,
    FIFO_DATA_OUT_Y_H = 0x7C,
    FIFO_DATA_OUT_Z_L = 0x7D,
    FIFO_DATA_OUT_Z_H = 0x7E,
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
            | Register::ALL_INT_SRC
            | Register::WAKE_UP_SRC
            | Register::TAP_SRC
            | Register::D6D_SRC
            | Register::STATUS_REG
            | Register::OUT_TEMP_L
            | Register::OUT_TEMP_H
            | Register::OUTX_L_G
            | Register::OUTX_H_G
            | Register::OUTY_L_G
            | Register::OUTY_H_G
            | Register::OUTZ_L_G
            | Register::OUTZ_H_G
            | Register::OUTX_L_A
            | Register::OUTX_H_A
            | Register::OUTY_L_A
            | Register::OUTY_H_A
            | Register::OUTZ_L_A
            | Register::OUTZ_H_A
            | Register::EMB_FUNC_STATUS_MAINPAGE
            | Register::FSM_STATUS_A_MAINPAGE
            | Register::FSM_STATUS_B_MAINPAGE
            | Register::STATUS_MASTER_MAINPAGE
            | Register::FIFO_STATUS1
            | Register::FIFO_STATUS2
            | Register::TIMESTAMP0
            | Register::TIMESTAMP1
            | Register::TIMESTAMP2
            | Register::TIMESTAMP3
            | Register::INTERNAL_FREQ_FINE
            | Register::INT_OIS
            | Register::CTRL1_OIS
            | Register::CTRL2_OIS
            | Register::CTRL3_OIS
            | Register::FIFO_DATA_OUT_TAG
            | Register::FIFO_DATA_OUT_X_L
            | Register::FIFO_DATA_OUT_X_H
            | Register::FIFO_DATA_OUT_Y_L
            | Register::FIFO_DATA_OUT_Y_H
            | Register::FIFO_DATA_OUT_Z_L
            | Register::FIFO_DATA_OUT_Z_H => true,
            _ => false,
        }
    }
}

/// Full-scale selection.
#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum AccRange {
    /// ±16g
    G16 = 0b01,
    /// ±8g
    G8 = 0b11,
    /// ±4g
    G4 = 0b10,
    /// ±2g (Default)
    G2 = 0b00,
}

impl AccRange {
    pub fn bits(self) -> u8 {
        self as u8
    }
}

/// Output data rate.
#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum AccDataRate {
    /// 6.66kHz
    Hz_6660 = 0b1010,
    /// 3.33kHz
    Hz_3330 = 0b1001,
    /// 1.66kHz
    Hz_1660 = 0b1000,
    /// 833Hz
    Hz_833 = 0b0111,
    /// 416Hz (Default)
    Hz_416 = 0b0110,
    /// 208Hz
    Hz_208 = 0b0101,
    /// 104Hz
    Hz_104 = 0b0100,
    /// 52Hz
    Hz_52 = 0b0011,
    /// 26Hz
    Hz_26 = 0b0010,
    /// 12.5Hz
    Hz_12_5 = 0b0001,
    /// 1.6Hz
    Hz_1_6 = 0b1011,
    /// Power down
    PowerDown = 0b0000,
}

impl AccDataRate {
    pub fn bits(self) -> u8 {
        self as u8
    }

    pub fn sample_rate(self) -> f32 {
        match self {
            AccDataRate::Hz_6660 => 6660.0,
            AccDataRate::Hz_3330 => 3330.0,
            AccDataRate::Hz_1660 => 1660.0,
            AccDataRate::Hz_833 => 833.0,
            AccDataRate::Hz_416 => 416.0,
            AccDataRate::Hz_208 => 208.0,
            AccDataRate::Hz_104 => 104.0,
            AccDataRate::Hz_52 => 52.0,
            AccDataRate::Hz_26 => 26.0,
            AccDataRate::Hz_12_5 => 12.5,
            AccDataRate::Hz_1_6 => 1.6,
            AccDataRate::PowerDown => 0.0,
        }
    }
}


#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum GyroRange {
    /// ±2kDPS
    DPS2000 = 0b110,
    /// ±1kDPS
    DPS1000 = 0b100,
    /// ±500DPS
    DPS500 = 0b010,
    /// ±250DPS (Default)
    DPS250 = 0b000,
    /// ±125DPS (Default)
    DPS125 = 0b001,
}

impl GyroRange {
    pub fn bits(self) -> u8 {
        self as u8
    }
}
#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum GyroDataRate {
    /// 6.66kHz
    Hz_6660 = 0b1010,
    /// 3.33kHz
    Hz_3330 = 0b1001,
    /// 1.66kHz
    Hz_1660 = 0b1000,
    /// 833Hz
    Hz_833 = 0b0111,
    /// 416Hz (Default)
    Hz_416 = 0b0110,
    /// 208Hz
    Hz_208 = 0b0101,
    /// 104Hz
    Hz_104 = 0b0100,
    /// 52Hz
    Hz_52 = 0b0011,
    /// 26Hz
    Hz_26 = 0b0010,
    /// 12.5Hz
    Hz_12_5 = 0b0001,
    /// 1.6Hz
    Hz_6_5 = 0b1011,
    /// Power down
    PowerDown = 0b0000,
}

impl GyroDataRate {
    pub fn bits(self) -> u8 {
        self as u8
    }

    pub fn sample_rate(self) -> f32 {
        match self {
            GyroDataRate::Hz_6660 => 6660.0,
            GyroDataRate::Hz_3330 => 3330.0,
            GyroDataRate::Hz_1660 => 1660.0,
            GyroDataRate::Hz_833 => 833.0,
            GyroDataRate::Hz_416 => 416.0,
            GyroDataRate::Hz_208 => 208.0,
            GyroDataRate::Hz_104 => 104.0,
            GyroDataRate::Hz_52 => 52.0,
            GyroDataRate::Hz_26 => 26.0,
            GyroDataRate::Hz_12_5 => 12.5,
            GyroDataRate::Hz_6_5 => 6.5,
            GyroDataRate::PowerDown => 0.0,
        }
    }
}

/// Data status structure. Decoded from the `STATUS_REG` register. todo
///
/// `STATUS_REG` has the following bit fields:
///   * `ZYXOR` - X, Y and Z-axis data overrun
///   * `ZOR` - Z-axis data overrun
///   * `YOR` - Y-axis data overrun
///   * `XOR` - X-axis data overrun
///   * `ZYXDA` - X, Y and Z-axis new data available
///   * `ZDA` - Z-axis new data available
///   * `YDA` Y-axis new data available
///   * `XDA` X-axis new data available
///
/// This struct splits the fields into more convenient groups:
///  * `zyxor` -> `ZYXOR`
///  * `xyzor` -> (`XOR`, `YOR`, `ZOR`)
///  * `zyxda` -> `ZYXDA`
///  * `xyzda` -> (`XDA`, `YDA`, `ZDA`)
#[derive(Debug)]
pub struct DataStatus {
    /// ZYXOR bit
    pub zyxor: bool,

    /// (XOR, YOR, ZOR) bits
    pub xyzor: (bool, bool, bool),

    /// ZYXDA bit
    pub zyxda: bool,

    /// (XDA, YDA, ZDA) bits
    pub xyzda: (bool, bool, bool),
}

/// Operating mode. todo
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum AccMode {
    /// High-performance mode
    HighPerformance,

    /// Normal mode
    Normal,
}

// === WHO_AMI_I (0Fh) ===

/// `WHO_AM_I` device identification register
pub const DEVICE_ID: u8 = 0x6C;

// === TEMP_CFG_REG (1Fh) === todo

pub const ADC_EN: u8 = 0b1000_0000;
pub const TEMP_EN: u8 = 0b0100_0000;

// === CTRL1_XL (10h) ===

pub const ODR_MASK: u8 = 0b1111_0000;
pub const FS_MASK: u8 = 0b0000_1100;
pub const Z_EN: u8 = 0b0000_0100;


// === CTRL3_C (12h) ===

pub const BOOT: u8 = 0b1000_0000;
pub const BDU: u8 = 0b0100_0000;
pub const H_LACTIVE: u8 = 0b0010_0000;
pub const PP_OD: u8 = 0b0001_0000;
pub const SIM: u8 = 0b0000_1000;
pub const IF_INC: u8 = 0b0000_0100;
pub const SW_RESET: u8 = 0b0000_0001;

// === STATUS_REG (27h) === todo

pub const ZYXOR: u8 = 0b1000_0000;
pub const ZOR: u8 = 0b0100_0000;
pub const YOR: u8 = 0b0010_0000;
pub const XOR: u8 = 0b0001_0000;
pub const ZYXDA: u8 = 0b0000_1000;
pub const ZDA: u8 = 0b0000_0100;
pub const YDA: u8 = 0b0000_0010;
pub const XDA: u8 = 0b0000_0001;
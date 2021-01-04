/// Enumerate all device registers.
#[allow(dead_code, non_camel_case_types)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum Register {
    TEMPERATURE_HIGH = 0x00,
    STATUS = 0x01,
    TEMPERATURE_LOW = 0x02,
    CONFIGURATION = 0x03,
    CONVERSION_RATE = 0x04,
    TEMPERATURE_HIGH_LIMIT_HIGH = 0x05,
    TEMPERATURE_HIGH_LIMIT_LOW = 0x06,
    TEMPERATURE_LOW_LIMIT_HIGH = 0x07,
    TEMPERATURE_LOW_LIMIT_LOW = 0x08,
    ONE_SHOT = 0x0F,
    THERM_LIMIT = 0x20,
    THERM_HYSTERESIS = 0x21,
    SMBUS_TIMEOUT = 0x22,
    PRODUCT_ID = 0xFD,
    MANUFACTURER_ID = 0xFE,
    REVISION_ID = 0xFF,
}

#[allow(dead_code)]
impl Register {
    /// Get register address
    pub fn addr(self) -> u8 {
        self as u8
    }

    /// Is the register read-only?
    pub fn read_only(self) -> bool {
        match self {
            Register::TEMPERATURE_HIGH
            | Register::STATUS
            | Register::TEMPERATURE_LOW
            | Register::PRODUCT_ID
            | Register::MANUFACTURER_ID
            | Register::REVISION_ID => true,
            _ => false,
        }
    }
}

// slave address
pub const DEVICE_ADDRESS: u8 = 0x4A;

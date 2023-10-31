//! The address width for all pipes in the nRF24L01 setup

use super::RegisterValue;

/// The address width of the rx/tx address field
#[derive(Debug, Clone, Copy)]
pub enum AddressWidth {
    // 3 byte address width
    A3Bytes = 0x01,
    // 4 byte address width
    A4Bytes = 0x10,
    // 5 byte address width
    A5Bytes = 0x11,
}

impl AddressWidth {
    /// Get the u8 value of the address width
    pub const fn as_u8(&self) -> u8 {
        match self {
            Self::A3Bytes => 3u8,
            Self::A4Bytes => 4u8,
            Self::A5Bytes => 5u8,
        }
    }
}

impl RegisterValue for AddressWidth {
    fn register_value(&self, address: u8) -> u8 {
        if address == 0x03 {
            match self {
                Self::A3Bytes => 0b0000_0001,
                Self::A4Bytes => 0b0000_0010,
                Self::A5Bytes => 0b0000_0011,
            }
        } else {
            return 0u8;
        }
    }
}
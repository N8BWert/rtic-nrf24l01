//! Power Amplifier level for the nRF24l01

use super::RegisterValue;

use defmt::Format;

/// The power amplifier configuration
/// 
/// Output Power | Current Consumption
/// ----------------------------------
/// 0dBm         | 11.3mA
/// ----------------------------------
/// -6dBm        | 9.0mA
/// ----------------------------------
/// -12dBm       | 7.5mA
/// ----------------------------------
/// -18dBm       | 7.0mA
/// ----------------------------------
#[derive(Debug, Clone, Copy, Format)]
pub enum PowerAmplifier {
    // Maximum Power Amplifier Level (0dBm)
    PAMax,
    // -6dBm
    PAHigh,
    // -12dBm
    PAMedium,
    // Minimum Power Amplifier Level (-18dBm)
    PALow,
}

impl RegisterValue for PowerAmplifier {
    fn register_value(&self, address: u8) -> u8 {
        if address == 0x06 {
            match self {
                Self::PAMax => 0b0000_0110,
                Self::PAHigh => 0b0000_0100,
                Self::PAMedium => 0b0000_0010,
                Self::PALow => 0b0000_0000,
            }
        } else {
            return 0u8;
        }
    }
}
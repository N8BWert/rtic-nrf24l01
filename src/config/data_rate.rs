//! The data rate of the module

use crate::register::Register;

use super::RegisterValue;

use defmt::Format;

/// The datarate of transmission
/// 
/// Using lower air rates gives better receiver sensitivity.
/// 
/// Using higher air rates gives lower current consumption and reduced probability
/// of on-air collisions.
#[derive(Debug, Clone, Copy, Format)]
pub enum DataRate {
    // 250 kbps
    R250kb,
    // 1 Mbps
    R1Mb,
    // 2Mbps
    R2Mb,
}

impl RegisterValue for DataRate {
    fn register_mask(&self, register: Register) -> u8 {
        if register == Register::RfSetup {
            !(0b0010_1000)
        } else {
            0
        }
    }

    fn register_value(&self, register: Register) -> u8 {
        if register == Register::RfSetup {
            match self {
                Self::R250kb => 0b0010_0000,
                Self::R1Mb => 0b0000_00000,
                Self::R2Mb => 0b00000_1000,
            }
        } else {
            0
        }
    } 
}
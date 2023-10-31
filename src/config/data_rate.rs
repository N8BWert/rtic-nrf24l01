//! The data rate of the module

use super::RegisterValue;

/// The datarate of transmission
/// 
/// Using lower air rates gives better receiver sensitivity.
/// 
/// Using higher air rates gives lower current consumption and reduced probability
/// of on-air collisions.
#[derive(Debug, Clone, Copy)]
pub enum DataRate {
    // 250 kbps
    R250kb,
    // 1 Mbps
    R1Mb,
    // 2Mbps
    R2Mb,
}

impl RegisterValue for DataRate {
    fn register_value(&self, address: u8) -> u8 {
        if address == 0x06 {
            match self {
                Self::R250kb => 0b0010_0000,
                Self::R1Mb => 0b0000_0000,
                Self::R2Mb => 0b0000_1000,
            }
        } else {
            return 0u8;
        }
    }
}
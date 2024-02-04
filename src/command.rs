//! Commands that can be sent to the NRF24L01 Module.
//! 
//! Never use these commands on their own and instead use the library functions

use crate::register::Register;

/// The various SPI commands that can be issued to the NRF24L01 module
pub enum Command {
    ReadRegister(Register),
    WriteRegister(Register),
    RegisterMask,
    Activate,
    ReadRxPayload,
    WriteTxPayload,
    FlushTx,
    FlushRx,
    ReuseTxPayload,
    ReadRxPayloadWidth,
    WriteAcknowledgePayload(u8),
    WriteTxPayloadNoAck,
    Nop,
}

impl Command {
    // Parse the opcode of command from the command itself
    pub fn opcode(&self) -> u8 {
        match self {
            Self::ReadRegister(register) => 0x00 | *register as u8,
            Self::WriteRegister(register) => 0x20 | (*register as u8),
            Self::RegisterMask => 0x1F,
            Self::Activate => 0x50,
            Self::ReadRxPayload => 0b0110_0001,
            Self::WriteTxPayload => 0b1010_0000,
            Self::FlushTx => 0b1110_0001,
            Self::FlushRx => 0b1110_0010,
            Self::ReuseTxPayload => 0b1110_0011,
            Self::ReadRxPayloadWidth => 0b0110_0000,
            Self::WriteAcknowledgePayload(pipe) => 0b1010_1000 | pipe,
            Self::WriteTxPayloadNoAck => 0b1011_0000,
            Self::Nop => 0xFF,
        }
    }
}
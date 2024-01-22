//! Commands that can be sent to the NRF24L01 Module.
//! 
//! Never use these commands on their own and instead use the library functions

/// The various SPI commands that can be issued to the NRF24L01 module
pub enum Command {
    ReadRegister(u8),
    WriteRegister(u8),
    ReadRxPayload,
    WriteTxPayload,
    FlushTx,
    FlushRx,
    ReuseTxPayload,
    Activate,
    ReadRxPayloadWidth,
    WriteAcknowledgePayload(u8),
    Nop,
    RegisterMask,
}

impl Command {
    // Parse the opcode of command from the command itself
    pub fn opcode(&self) -> u8 {
        match self {
            Self::ReadRegister(register) => 0x00 | register,
            Self::WriteRegister(register) => 0x20 | register,
            Self::ReadRxPayload => 0x61,
            Self::WriteTxPayload => 0xA0,
            Self::FlushTx => 0xE1,
            Self::FlushRx => 0xE2,
            Self::ReuseTxPayload => 0xE3,
            Self::Activate => 0x50,
            Self::ReadRxPayloadWidth => 0x60,
            Self::WriteAcknowledgePayload(payload) => 0xA8 | payload,
            Self::Nop => 0xFF,
            Self::RegisterMask => 0x1F,
        }
    }
}
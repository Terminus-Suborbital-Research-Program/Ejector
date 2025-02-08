use bincode::{
    config::standard,
    decode_from_slice, encode_into_slice,
    error::{DecodeError, EncodeError},
    Decode, Encode,
};
use embedded_io::{Read, Write};

use heapless::Vec;

type Buffer = Vec<u8, 256>;

use bin_packets::packets::ApplicationPacket;

#[derive(Debug, Clone, Copy, Encode, Decode)]
#[allow(non_camel_case_types)]
pub enum LinkLayerPayload {
    Payload(ApplicationPacket),
    ACK,
    NACK,
    NODATA,
}

#[derive(Debug, Clone, Copy, Encode, Decode, Hash)]
pub enum Device {
    Ejector,
    Deployer,
    Icarus,
    Atmega,
    Pi,
    Tester,
}

#[derive(Debug, Clone, Copy, Encode, Decode)]
pub struct LinkPacket {
    pub from_device: Device,
    pub to_device: Device,
    pub route_through: Option<Device>,
    pub payload: LinkLayerPayload,
    pub checksum: Option<u32>,
}

impl LinkPacket {
    // FNV1A hash
    pub fn checksum(&self) -> u32 {
        let mut hash: u32 = 0x811c9dc5;
        let fnv_prime: u32 = 16777619;

        hash ^= self.from_device as u32;
        hash = hash.wrapping_mul(fnv_prime);
        hash ^= self.to_device as u32;
        hash = hash.wrapping_mul(fnv_prime);
        if let Some(route) = self.route_through {
            hash ^= route as u32;
            hash = hash.wrapping_mul(fnv_prime);
        }

        let mut buffer = [0u8; 256];
        let bytes = bincode::encode_into_slice(&self.payload, &mut buffer, standard()).unwrap();
        for byte in &buffer[0..bytes] {
            hash ^= *byte as u32;
            hash = hash.wrapping_mul(fnv_prime);
        }

        hash
    }

    // Set the checksum field to the correct value
    pub fn set_checksum(&mut self) {
        self.checksum = Some(self.checksum());
    }

    // Verify the checksum field is correct
    pub fn verify_checksum(&self) -> bool {
        match self.checksum {
            Some(checksum) => checksum == self.checksum(),
            None => false,
        }
    }
}

impl Default for LinkPacket {
    fn default() -> Self {
        Self {
            from_device: Device::Tester,
            to_device: Device::Tester,
            route_through: None,
            payload: LinkLayerPayload::NODATA,
            checksum: None,
        }
    }
}

// Device to act as a link layer connection, from a embedded_hal::Read/Write/ReadReady/WriteReady
// to a bincode::Reader/Writer
pub struct LinkLayerDevice<D> {
    pub device: D,
    pub me: Device,
    buffer: Buffer,
}

impl<D> LinkLayerDevice<D> {
    pub fn new(device: D, me: Device) -> Self {
        Self {
            device,
            me,
            buffer: Buffer::new(),
        }
    }

    pub fn construct_packet(&self, packload: ApplicationPacket, to: Device) -> LinkPacket {
        let mut packet = LinkPacket {
            from_device: self.me,
            to_device: to,
            route_through: None,
            payload: LinkLayerPayload::Payload(packload),
            checksum: None,
        };

        packet.set_checksum();

        packet
    }
}

// When we can write, we can encode packets to the underlying device
impl<D> LinkLayerDevice<D>
where
    D: Write,
{
    pub fn write_link_packet(&mut self, packet: LinkPacket) -> Result<(), EncodeError> {
        let mut slice = [0u8; 128];
        let written = encode_into_slice(packet, &mut slice, standard())?;
        self.device.write(&slice[..written]).ok();
        Ok(())
    }
}

// When we can read, we can decode packets from the underlying device
impl<D> LinkLayerDevice<D>
where
    D: Read,
{
    pub fn read_link_packet(&mut self) -> Option<LinkPacket> {
        let config = standard();

        // Read in to the buffer
        let mut slice = [0u8; 128];
        let read = self.device.read(&mut slice).ok();
        if let Some(read) = read {
            for i in 0..read {
                self.buffer.push(slice[i]).ok();
            }
        } else {
            return None;
        }

        loop {
            // Try to decode a packet from the current buffer
            match decode_from_slice(&self.buffer, config) {
                Ok((packet, read_bytes)) => {
                    for _ in 0..read_bytes {
                        self.buffer.remove(0); // Innefficient, but it works
                    }
                    return Some(packet);
                }

                Err(e) => match e {
                    // If we're out of data, then return none
                    DecodeError::UnexpectedEnd { .. } => {
                        return None;
                    }

                    _ => {
                        // Pop off the first byte and try again
                        self.buffer.remove(0);
                    }
                },
            }

            if self.buffer.len() == 0 {
                return None;
            }
        }
    }
}

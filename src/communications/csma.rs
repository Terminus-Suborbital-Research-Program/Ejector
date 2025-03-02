use bincode::{
    config::standard, decode_from_slice, encode_into_slice, error::DecodeError, Decode, Encode,
};
use defmt::{info, Format};
use embedded_io::{Read, ReadReady, Write, WriteReady};

const MAX_PACKET_SIZE: usize = 256;
const RX_BUFFER_CAPACITY: usize = 512;

pub struct CADevice<T, D, const N: usize>
where
    D: Read + Write + ReadReady + WriteReady,
{
    device: D,
    tx_queue: heapless::spsc::Queue<T, N>,
    rx_queue: heapless::spsc::Queue<T, N>,
    rx_buffer: heapless::Vec<u8, RX_BUFFER_CAPACITY>, // Internal byte accumulator.
    // at which a transmission can be attempted.
    backoff_until: u64,
    rng: Lcg,
}

impl<T, D, const N: usize> CADevice<T, D, N>
where
    D: Read + Write + ReadReady + WriteReady,
    T: Encode + Decode + Format,
{
    pub fn new(device: D, rng: Lcg) -> Self {
        Self {
            device,
            tx_queue: heapless::spsc::Queue::new(),
            rx_queue: heapless::spsc::Queue::new(),
            rx_buffer: heapless::Vec::new(),
            backoff_until: 0,
            rng,
        }
    }

    pub fn enqueue(&mut self, packet: T) -> Result<(), ()> {
        self.tx_queue.enqueue(packet).map_err(|_| ())
    }

    pub fn dequeue(&mut self) -> Option<T> {
        self.rx_queue.dequeue()
    }

    pub fn update_rx(&mut self) {
        if let Ok(true) = self.device.read_ready() {
            let mut buf = [0u8; MAX_PACKET_SIZE];
            if let Ok(n) = self.device.read(&mut buf) {
                if n > 0 {
                    let _ = self.rx_buffer.extend_from_slice(&buf[..n]);
                }
            }
        }

        while !self.rx_buffer.is_empty() {
            match decode_from_slice::<T, _>(&self.rx_buffer, standard()) {
                Ok((packet, consumed)) => {
                    for _ in 0..consumed {
                        self.rx_buffer.remove(0);
                    }
                    let _ = self.rx_queue.enqueue(packet);
                }
                Err(DecodeError::UnexpectedEnd { .. }) => break,
                Err(_) => {
                    // On other decode errors, drop one byte and try again.
                    self.rx_buffer.remove(0);
                }
            }
        }
    }

    /// Update for transmitting data.
    /// `current_time` is the current time in ms from your monotonic timer.
    /// Returns Some(delay_ms) if still in backoff, allowing the caller to sleep.
    pub fn update_tx(&mut self, current_time: u64) -> Option<u64> {
        // Check if we are still within our backoff period.
        if current_time < self.backoff_until {
            return Some(self.backoff_until - current_time);
        }

        if self.tx_queue.peek().is_some() {
            // If channel busy, back off.
            if let Ok(true) = self.device.read_ready() {
                // Generate a random delay between 1 and 10 ms.
                let delay = self.rng.gen_range(1, 10) as u64;
                self.backoff_until = current_time + delay;
                return Some(delay);
            }

            // Channel is clear: dequeue and transmit.
            if let Some(packet) = self.tx_queue.dequeue() {
                defmt::debug!("Transmitting packet {:?}", packet);
                let mut buf = [0u8; MAX_PACKET_SIZE];
                if let Ok(n) = encode_into_slice(packet, &mut buf, standard()) {
                    let _ = self.device.write_all(&buf[..n]);
                }
            }
        }
        None
    }
}

/// Linear Congruential Generator, for random backoff.
pub struct Lcg {
    seed: u32,
}

impl Lcg {
    /// Creates a new LCG with the given seed.
    pub fn new(seed: u32) -> Self {
        // Avoid a zero seed since it can stall some LCG implementations.
        Self {
            seed: if seed == 0 { 1 } else { seed },
        }
    }

    /// Generates the next random number.
    pub fn next(&mut self) -> u32 {
        // Common LCG parameters:
        // multiplier: 1664525, increment: 1013904223.
        // These values are from Numerical Recipes.
        self.seed = self.seed.wrapping_mul(1664525).wrapping_add(1013904223);
        self.seed
    }

    /// Generates a random number in the inclusive range [low, high].
    ///
    /// # Panics
    /// Panics if `high` is less than `low`.
    pub fn gen_range(&mut self, low: u32, high: u32) -> u32 {
        debug_assert!(high >= low, "high must be greater than or equal to low");
        let range = high - low + 1;
        low + (self.next() % range)
    }
}

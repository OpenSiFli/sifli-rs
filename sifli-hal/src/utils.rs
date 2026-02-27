#![allow(unused)]

use embassy_time::{Duration, Instant};
use portable_atomic::{AtomicU32, AtomicU64, AtomicU8, Ordering};

/// A thread-safe bit flag structure based on AtomicU8.
pub(crate) struct BitFlags8 {
    flags: AtomicU8,
}

impl BitFlags8 {
    /// Creates new bit flags with an initial value.
    pub(crate) fn new(initial_value: u8) -> Self {
        Self {
            flags: AtomicU8::new(initial_value),
        }
    }

    /// Sets a specific bit by its index (0-7).
    pub(crate) fn set_bit(&self, idx: u8) {
        assert!(idx < 8, "Index out of bounds for BitFlags8");
        self.flags.fetch_or(1 << idx, Ordering::SeqCst);
    }

    /// Clears a specific bit by its index (0-7).
    pub(crate) fn clear_bit(&self, idx: u8) {
        assert!(idx < 8, "Index out of bounds for BitFlags8");
        self.flags.fetch_and(!(1 << idx), Ordering::SeqCst);
    }

    /// Sets multiple bits using a bitmask.
    pub(crate) fn set_bits(&self, bits: u8) {
        self.flags.fetch_or(bits, Ordering::SeqCst);
    }

    /// Clears multiple bits using a bitmask.
    pub(crate) fn clear_bits(&self, bits: u8) {
        self.flags.fetch_and(!bits, Ordering::SeqCst);
    }

    /// Checks if a specific bit is set.
    pub(crate) fn is_bit_set(&self, idx: u8) -> bool {
        assert!(idx < 8, "Index out of bounds for BitFlags8");
        (self.flags.load(Ordering::SeqCst) & (1 << idx)) != 0
    }

    /// Gets the current value of the flags.
    pub(crate) fn get(&self) -> u8 {
        self.flags.load(Ordering::SeqCst)
    }
}

/// A thread-safe bit flag structure based on AtomicU32.
pub(crate) struct BitFlags32 {
    flags: AtomicU32,
}

impl BitFlags32 {
    /// Creates new bit flags with an initial value.
    pub(crate) fn new(initial_value: u32) -> Self {
        Self {
            flags: AtomicU32::new(initial_value),
        }
    }

    /// Sets a specific bit by its index (0-31).
    pub(crate) fn set_bit(&self, idx: u8) {
        assert!(idx < 32, "Index out of bounds for BitFlags32");
        self.flags.fetch_or(1 << idx, Ordering::SeqCst);
    }

    /// Clears a specific bit by its index (0-31).
    pub(crate) fn clear_bit(&self, idx: u8) {
        assert!(idx < 32, "Index out of bounds for BitFlags32");
        self.flags.fetch_and(!(1 << idx), Ordering::SeqCst);
    }

    /// Sets multiple bits using a bitmask.
    pub(crate) fn set_bits(&self, bits: u32) {
        self.flags.fetch_or(bits, Ordering::SeqCst);
    }

    /// Clears multiple bits using a bitmask.
    pub(crate) fn clear_bits(&self, bits: u32) {
        self.flags.fetch_and(!bits, Ordering::SeqCst);
    }

    /// Checks if a specific bit is set.
    pub(crate) fn is_bit_set(&self, idx: u8) -> bool {
        assert!(idx < 32, "Index out of bounds for BitFlags32");
        (self.flags.load(Ordering::SeqCst) & (1 << idx)) != 0
    }

    /// Gets the current value of the flags.
    pub(crate) fn get(&self) -> u32 {
        self.flags.load(Ordering::SeqCst)
    }
}

/// A thread-safe bit flag structure based on AtomicU64.
pub(crate) struct BitFlags64 {
    flags: AtomicU64,
}

impl BitFlags64 {
    /// Creates new bit flags with an initial value.
    pub(crate) fn new(initial_value: u64) -> Self {
        Self {
            flags: AtomicU64::new(initial_value),
        }
    }

    /// Sets a specific bit by its index (0-63).
    pub(crate) fn set_bit(&self, idx: u8) {
        assert!(idx < 64, "Index out of bounds for BitFlags64");
        self.flags.fetch_or(1 << idx, Ordering::SeqCst);
    }

    /// Clears a specific bit by its index (0-63).
    pub(crate) fn clear_bit(&self, idx: u8) {
        assert!(idx < 64, "Index out of bounds for BitFlags64");
        self.flags.fetch_and(!(1 << idx), Ordering::SeqCst);
    }

    /// Sets multiple bits using a bitmask.
    pub(crate) fn set_bits(&self, bits: u64) {
        self.flags.fetch_or(bits, Ordering::SeqCst);
    }

    /// Clears multiple bits using a bitmask.
    pub(crate) fn clear_bits(&self, bits: u64) {
        self.flags.fetch_and(!bits, Ordering::SeqCst);
    }

    /// Checks if a specific bit is set.
    pub(crate) fn is_bit_set(&self, idx: u8) -> bool {
        assert!(idx < 64, "Index out of bounds for BitFlags64");
        (self.flags.load(Ordering::SeqCst) & (1 << idx)) != 0
    }

    /// Gets the current value of the flags.
    pub(crate) fn get(&self) -> u64 {
        self.flags.load(Ordering::SeqCst)
    }
}

pub(crate) struct BitIter(pub(crate) u32);

impl Iterator for BitIter {
    type Item = u8;

    fn next(&mut self) -> Option<Self::Item> {
        match self.0.trailing_zeros() {
            32 => None,
            b => {
                self.0 &= !(1 << b);
                Some(b as _)
            }
        }
    }
}

pub(crate) struct BitIter64(pub(crate) u64);

impl Iterator for BitIter64 {
    type Item = u8;

    fn next(&mut self) -> Option<Self::Item> {
        match self.0.trailing_zeros() {
            64 => None,
            b => {
                self.0 &= !(1 << b);
                Some(b as _)
            }
        }
    }
}

/// Blocks until a condition becomes false or a timeout is reached.
#[inline]
pub fn blocking_wait_timeout<F>(mut condition: F, timeout: Duration) -> Result<(), ()>
where
    F: FnMut() -> bool,
{
    let start = Instant::now();

    while condition() {
        if start.elapsed() > timeout {
            return Err(());
        }
        // core::hint::spin_loop();
    }

    Ok(())
}

/// Blocks until a condition becomes false or a timeout in milliseconds is reached.
#[inline]
pub fn blocking_wait_timeout_ms<F>(mut condition: F, timeout_ms: u64) -> Result<(), ()>
where
    F: FnMut() -> bool,
{
    blocking_wait_timeout(&mut condition, Duration::from_millis(timeout_ms))
}

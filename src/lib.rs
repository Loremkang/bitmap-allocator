#![no_std]
#![feature(asm)]

use bit_field::BitField;
use core::ops::Range;

/// Allocator of a bitmap, able to allocate / free bits.
pub trait BitAlloc: Default {
    /// The bitmap has a total of CAP bits, numbered from 0 to CAP-1 inclusively.
    const CAP: usize;

    /// Allocate a free bit.
    fn alloc(&mut self) -> Option<usize>;

    /// Free an allocated bit.
    fn dealloc(&mut self, key: usize);

    /// Mark bits in the range as unallocated (available)
    fn insert(&mut self, range: Range<usize>);

    /// Reverse of insert
    fn remove(&mut self, range: Range<usize>);

    /// Whether there are free bits remaining
    fn any(&mut self) -> bool;

    /// Foundation of insert and remove. "bit" is only 0xffff or 0. Lazy tagging: apply to oneself when pushing down.
    fn set(&mut self, range: Range<usize>, bit: u16);

    /// Add tag for delayed update. Directly modify data for leaves.
    fn tag(&mut self, bit: u16);

    /// Whether a specific bit is free
    fn test(&mut self, key: usize) -> bool;
}

/// A bitmap of 256 bits
pub type BitAlloc256 = BitAllocCascade16<BitAlloc16>;
/// A bitmap of 4K bits
pub type BitAlloc4K = BitAllocCascade16<BitAlloc256>;
/// A bitmap of 64K bits
pub type BitAlloc64K = BitAllocCascade16<BitAlloc4K>;
/// A bitmap of 1M bits
pub type BitAlloc1M = BitAllocCascade16<BitAlloc64K>;
/// A bitmap of 16M bits
pub type BitAlloc16M = BitAllocCascade16<BitAlloc1M>;
/// A bitmap of 256M bits
pub type BitAlloc256M = BitAllocCascade16<BitAlloc16M>;

/// Implement the bit allocator by segment tree algorithm.
#[derive(Default)]
pub struct BitAllocCascade16<T: BitAlloc> {
    bit_tag: u16, // save the (pushdown) tag on each segment
    tagging: bool, // mark if the tag is active
    bitset: u16, // for each bit, 1 indicates available, 0 indicates inavailable
    sub: [T; 16],
}

impl<T: BitAlloc> BitAlloc for BitAllocCascade16<T> {
    const CAP: usize = T::CAP * 16;

    fn alloc(&mut self) -> Option<usize> {
        self.apply_tags();
        if self.any() {
            let i = log2(self.bitset);
            let res = self.sub[i].alloc().unwrap() + i * T::CAP;
            self.bitset.set_bit(i, self.sub[i].any());
            Some(res)
        } else {
            None
        }
    }
    fn dealloc(&mut self, key: usize) {
        self.apply_tags();
        let i = key / T::CAP;
        self.sub[i].dealloc(key % T::CAP);
        self.bitset.set_bit(i, true);
    }
    fn insert(&mut self, range: Range<usize>) {
        self.set(range, 0xffff);
    }
    fn remove(&mut self, range: Range<usize>) {
        self.set(range, 0);
    }
    fn set(&mut self, range: Range<usize>, bit: u16) {
        let Range { start, end } = range;
        assert!(start <= end);
        assert!(end <= Self::CAP);
        self.apply_tags();
        if end - start == Self::CAP { // fully cover this segment
            self.tag(bit);
        } else {
            for i in start / T::CAP..=(end - 1) / T::CAP {
                let begin = if start / T::CAP == i {
                    start % T::CAP
                } else {
                    0
                };
                let end = if end / T::CAP == i {
                    end % T::CAP
                } else {
                    T::CAP
                };
                self.sub[i].set(begin..end, bit);
                self.bitset.set_bit(i, self.sub[i].any());
            }
        }
    }
    fn tag(&mut self, bit: u16) {
        self.tagging = true;
        self.bit_tag = bit;
    }
    fn any(&mut self) -> bool {
        self.apply_tags();
        self.bitset != 0
    }
    fn test(&mut self, key: usize) -> bool {
        self.apply_tags();
        self.sub[key / T::CAP].test(key % T::CAP)
    }
}

impl<T: BitAlloc> BitAllocCascade16<T> {
    /// Apply tags on oneself. May propogate to leaves, so we have "tag" function.
    fn apply_tags(&mut self) {
        if self.tagging == true {
            self.bitset.set_bits(0..16, self.bit_tag);
            for i in 0..16 {
                self.sub[i].tag(self.bit_tag);
            }
            self.tagging = false;
        }
    }
}

/// A bitmap consisting of only 16 bits.
/// BitAlloc16 acts as the leaf (except the leaf bits of course) nodes
/// in the segment trees.
#[derive(Default)]
pub struct BitAlloc16(u16);

impl BitAlloc for BitAlloc16 {
    const CAP: usize = 16;

    fn alloc(&mut self) -> Option<usize> {
        if self.any() {
            let i = log2(self.0);
            self.0.set_bit(i, false);
            Some(i)
        } else {
            None
        }
    }
    fn dealloc(&mut self, key: usize) {
        assert!(!self.test(key));
        self.0.set_bit(key, true);
    }
    fn insert(&mut self, range: Range<usize>) {
        self.set(range, 0xffff);
    }
    fn remove(&mut self, range: Range<usize>) {
        self.set(range, 0);
    }
    fn set(&mut self, range: Range<usize>, bit: u16) {
        self.0.set_bits(range.clone(), bit.get_bits(range));
    }
    fn tag(&mut self, bit: u16) {
        self.set(0..16, bit);
    }
    fn any(&mut self) -> bool {
        self.0 != 0
    }
    fn test(&mut self, key: usize) -> bool {
        self.0.get_bit(key)
    }
}

#[inline(always)]
#[cfg(target_arch = "x86_64")]
fn log2(x: u16) -> usize {
    assert_ne!(x, 0);
    let pos: u16;
    unsafe { asm!("bsrw $1, $0" :"=r"(pos) :"r"(x) : :"volatile") };
    pos as usize
}

#[inline(always)]
#[cfg(not(target_arch = "x86_64"))]
fn log2(x: u16) -> usize {
    log2_naive(x)
}

#[cfg(not(target_arch = "x86_64"))]
#[inline(always)]
fn log2_naive(mut x: u16) -> usize {
    //a naive implement
    assert_ne!(x, 0);
    let mut pos = -1;
    while x != 0 {
        pos += 1;
        x >>= 1;
    }
    pos as usize
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(not(target_arch = "x86_64"))]
    #[test]
    fn log2_() {
        for x in 1..=0xffff {
            assert_eq!(log2(x), log2_naive(x), "log2 failed: {}", x);
        }
    }

    #[test]
    fn bitalloc16() {
        let mut ba = BitAlloc16::default();
        assert_eq!(BitAlloc16::CAP, 16);
        ba.insert(0..16);
        for i in 0..16 {
            assert_eq!(ba.test(i), true);
        }
        ba.remove(8..14);
        assert_eq!(ba.alloc(), Some(15));
        assert_eq!(ba.alloc(), Some(14));
        assert_eq!(ba.alloc(), Some(7));
        ba.dealloc(14);
        ba.dealloc(15);
        ba.dealloc(7);

        for _ in 0..10 {
            assert!(ba.alloc().is_some());
        }
        assert!(!ba.any());
        assert!(ba.alloc().is_none());
    }

    #[test]
    fn bitalloc4k() {
        let mut ba = BitAlloc4K::default();
        assert_eq!(BitAlloc4K::CAP, 4096);
        ba.insert(0..4096);
        for i in 0..4096 {
            assert_eq!(ba.test(i), true);
        }
        ba.remove(8..4094);
        for i in 0..4096 {
            assert_eq!(ba.test(i), i < 8 || i >= 4094);
        }
        assert_eq!(ba.alloc(), Some(4095));
        assert_eq!(ba.alloc(), Some(4094));
        assert_eq!(ba.alloc(), Some(7));
        ba.dealloc(4095);
        ba.dealloc(4094);
        ba.dealloc(7);

        for _ in 0..10 {
            assert!(ba.alloc().is_some());
        }
        assert!(ba.alloc().is_none());
    }
}

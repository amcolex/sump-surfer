//! Direct /dev/mem access for hardware register manipulation
//!
//! Provides safe(r) wrappers around mmap for accessing FPGA registers
//! and BRAM from userspace.

use std::fs::OpenOptions;
use std::io;
use std::os::unix::io::AsRawFd;

/// Memory-mapped region for hardware access
pub struct DevMem {
    ptr: *mut u8,
    size: usize,
    base_addr: usize,
}

// Safety: DevMem only provides &self methods that use volatile reads/writes
// The underlying memory is hardware registers, not shared with other threads
unsafe impl Send for DevMem {}
unsafe impl Sync for DevMem {}

impl DevMem {
    /// Map a physical memory region
    ///
    /// # Arguments
    /// * `base_addr` - Physical base address (must be page-aligned for best results)
    /// * `size` - Size of region to map
    ///
    /// # Safety
    /// Caller must ensure the address range is valid for the hardware
    pub fn new(base_addr: usize, size: usize) -> io::Result<Self> {
        let fd = OpenOptions::new()
            .read(true)
            .write(true)
            .open("/dev/mem")?;

        let page_size = unsafe { libc::sysconf(libc::_SC_PAGESIZE) as usize };
        let page_offset = base_addr % page_size;
        let map_base = base_addr - page_offset;
        let map_size = size + page_offset;

        let ptr = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                map_size,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                fd.as_raw_fd(),
                map_base as libc::off_t,
            )
        };

        if ptr == libc::MAP_FAILED {
            return Err(io::Error::last_os_error());
        }

        // Adjust pointer to account for page alignment
        let adjusted_ptr = unsafe { (ptr as *mut u8).add(page_offset) };

        Ok(Self {
            ptr: adjusted_ptr,
            size,
            base_addr,
        })
    }

    /// Read a 32-bit word at byte offset
    #[inline]
    pub fn read32(&self, offset: usize) -> Option<u32> {
        if offset + 4 > self.size {
            return None;
        }
        Some(unsafe {
            std::ptr::read_volatile(self.ptr.add(offset) as *const u32)
        })
    }

    /// Write a 32-bit word at byte offset
    #[inline]
    pub fn write32(&self, offset: usize, value: u32) -> bool {
        if offset + 4 > self.size {
            return false;
        }
        unsafe {
            std::ptr::write_volatile(self.ptr.add(offset) as *mut u32, value);
        }
        true
    }

    /// Get the base address
    #[allow(dead_code)]
    pub fn base_addr(&self) -> usize {
        self.base_addr
    }

    /// Get the mapped size
    #[allow(dead_code)]
    pub fn size(&self) -> usize {
        self.size
    }
}

impl Drop for DevMem {
    fn drop(&mut self) {
        let page_size = unsafe { libc::sysconf(libc::_SC_PAGESIZE) as usize };
        let page_offset = self.base_addr % page_size;
        let map_ptr = unsafe { self.ptr.sub(page_offset) };
        let map_size = self.size + page_offset;

        unsafe {
            libc::munmap(map_ptr as *mut libc::c_void, map_size);
        }
    }
}

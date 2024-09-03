// SPDX-License-Identifier: GPL-2.0

//! I2c kernel interface
//!
//! C header: [`include/linux/i2c.h`, 'include/linux/clk.h']


/// i2c func
pub mod functionality;
/// i2c msg
pub mod msg;
/// i2c timing
pub mod timing;
/// aw_apb_i2c register

use core::{
    ffi::c_void,
    marker::PhantomData,
};

use crate::{
    bindings,
    device,
    str::CStr,
    error::{Result, to_result, from_result},
    types::ForeignOwnable,
    ThisModule,
    prelude::{EINVAL,Vec, PinInit},
    pr_info,
    types::Opaque,
};

use macros::vtable;

/// message cannot have length of 0
pub const I2C_AQ_NO_ZERO_LEN_READ : u64 = 0b100000;
/// message cannot have length of 0
pub const I2C_AQ_NO_ZERO_LEN_WRITE : u64 = 0b1000000;
/// message cannot have length of 0
pub const I2C_AQ_NO_ZERO_LEN:u64 = I2C_AQ_NO_ZERO_LEN_READ | I2C_AQ_NO_ZERO_LEN_WRITE;

/// Determine whether the I2C device is a slave
pub fn i2c_detect_slave_mode(dev: &device::Device) -> bool {
    // SAFETY: call ffi and ptr is valid
    unsafe{bindings::i2c_detect_slave_mode(dev.ptr)}
}

/// Wraps the kernel's pointer of `struct i2c_msg`.
///
/// # Invariants
///
/// The pointer `I2cMsg::ptr` is non-null and valid.
pub struct I2cMsg {
    ptr: *mut bindings::i2c_msg,
}

// SAFETY: `msg` only holds a pointer to a C i2c_msg, which is safe to be used from any thread.
unsafe impl Send for I2cMsg {}

// SAFETY: `Device` only holds a pointer to a C i2c_msg, references to which are safe to be used
// from any thread.
unsafe impl Sync for I2cMsg {}


impl I2cMsg {
    /// Creates a new device instance.
    ///
    /// # Safety
    ///
    /// Callers must ensure that `ptr` is valid, non-null, and has a non-zero reference count.
    pub unsafe fn from_ptr(ptr: *mut bindings::i2c_msg) -> Self {
        // INVARIANT: The safety requirements satisfy all but one invariant, which is that `self`
        // owns a reference. This is satisfied by the call to `get_device` above.
        Self { ptr }
    }

    /// From Linux msgs to user msg array
    pub fn into_array<T, F> (&self, num: usize, trans: F) -> Result<Vec<T>>
    where F: Fn(&mut bindings::i2c_msg)->T 
    {
        let mut v = Vec::new(); 
        let mut ptr = self.ptr;
        for _i in 0..num {
            let msg = unsafe { &mut *ptr};
            v.try_push(trans(msg))?;
            // SAFETY: Belive for linux param is OK
            unsafe { ptr = ptr.add(1);}
        }
        Ok(v)
    }
}

/// A i2cAdapter.
#[vtable]
pub trait I2cAlgo {
    /// Context data associated with the gpio chip.
    ///
    /// It determines the type of the context data passed to each of the methods of the trait.
    type Data: ForeignOwnable + Sync + Send;

    /// master xfer 
    fn master_xfer(_data: <Self::Data as ForeignOwnable>::Borrowed<'_>, 
        _msg: &I2cMsg, _msg_len: usize) -> Result<i32>;
    /// functonality
    fn functionality(_data: <Self::Data as ForeignOwnable>::Borrowed<'_>) -> u32;
}

/// Linux completion wrapper
///
/// Wraps the kernel's C `struct i2c_adapter`.
///
#[repr(transparent)]
pub struct I2cAdapter<T:I2cAlgo> {
    adapter: Opaque<bindings::i2c_adapter>,
    _phantom: PhantomData<T>,
}

unsafe impl<T:I2cAlgo> Sync for I2cAdapter<T> {}
unsafe impl<T:I2cAlgo> Send for I2cAdapter<T> {}

impl<T: I2cAlgo> I2cAdapter<T> {
    /// Creates a new [`I2cAdapter`] initialiser but does not register it yet.
    pub fn new (name: &'static CStr,
        owner: &'static  ThisModule, 
        parent: *mut bindings::device,
        quirks: *const bindings::i2c_adapter_quirks,
        algo: *mut bindings::i2c_algorithm,
        of_node: *mut bindings::device_node,
        ) -> impl PinInit<Self> {
        unsafe {
            kernel::init::pin_init_from_closure(move |slot| {
                let slot = Self::raw_get(slot);
                // SAFETY: adapter init as zero
                core::ptr::write_bytes(slot, 0, 1);
                let bytes = name.as_bytes_with_nul();
                for (i, &byte) in bytes.iter().enumerate() {
                    (*slot).name[i] = byte as core::ffi::c_char;
                }

                if T::HAS_MASTER_XFER {
                    (*algo).master_xfer = Some(maxster_xfer_callback::<T>);
                }

                if T::HAS_FUNCTIONALITY {
                    (*algo).functionality = Some(functionality_callback::<T>);
                }

                (*slot).algo = algo;

                (*slot).quirks = quirks;
                (*slot).dev.of_node = of_node;
                (*slot).owner = owner.0;
                (*slot).dev.parent = parent;

                (*slot).retries = 3;
                (*slot).nr = -1;
                (*slot).class = bindings::I2C_CLASS_DEPRECATED;
                Ok(())
            })
        }
    }

    /// Get a pointer to the inner `completion`.
    ///
    /// # Safety
    ///
    /// The provided pointer must not be dangling and must be properly aligned. (But the memory
    /// need not be initialized.)
    #[inline]
    unsafe fn raw_get(ptr: *const Self) -> *mut bindings::i2c_adapter {
        // SAFETY: The caller promises that the pointer is aligned and not dangling.
        //
        // A pointer cast would also be ok due to `#[repr(transparent)]`. We use `addr_of!` so that
        // the compiler does not complain that the `work` field is unused.
        unsafe { Opaque::raw_get(core::ptr::addr_of!((*ptr).adapter)) }
    }

    /// FFI API i2c_add_numbered_adapter
    pub fn add_numbered_adapter(&self, data: T::Data) -> Result {
        let ptr = self.adapter.get();
        let data_pointer = <T::Data as ForeignOwnable>::into_foreign(data) as *mut c_void;
        unsafe {
            (*ptr).dev.driver_data = data_pointer;
            to_result(bindings::i2c_add_numbered_adapter(ptr))?;
        }
        Ok(())
    }
}
impl<T:I2cAlgo> Drop for I2cAdapter<T> {
    fn drop(&mut self) {
        pr_info!("i2c adapter dropped");
    }
}

unsafe extern "C" fn maxster_xfer_callback<T: I2cAlgo>(
    adapter: *mut bindings::i2c_adapter,
    msgs: *mut bindings::i2c_msg,
    num: core::ffi::c_int,
) -> core::ffi::c_int {
    from_result ( || {
        let data = unsafe {T::Data::borrow((*adapter).dev.driver_data) };
        if num.is_negative() {
            return Err(EINVAL);
        }
        unsafe {T::master_xfer(data, &I2cMsg::from_ptr(msgs), num as usize)}
    })
}

unsafe extern "C" fn functionality_callback<T: I2cAlgo>(
    adapter: *mut bindings::i2c_adapter,
) -> bindings::u32_ {
        let data = unsafe {T::Data::borrow((*adapter).dev.driver_data)};
        T::functionality(data)
}

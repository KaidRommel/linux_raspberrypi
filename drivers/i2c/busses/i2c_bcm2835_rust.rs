// SPDX-License-Identifier: GPL-2.0

//! Rust i2c driver.
use core::cmp::max;
use core::f32::consts;
use core::{ffi, result};
use core::result::Result::Ok;

use kernel::device::RawDevice;
use kernel::{error, prelude::*};
use kernel::{platform, module_platform_driver, of, driver};
use kernel::{define_of_id_table, device, driver_of_id_table, module_of_id_table};
use kernel::bindings;
use kernel::sync::Arc;
use kernel::error::{to_result, Result, from_err_ptr};
use kernel::completion;
use kernel::clk::Clk;
use kernel::clk_provider::{self, ClkInitData};
use kernel::str::{CStr, CString};
use kernel::c_str;
use kernel::container_of;
use kernel::irq;

module_platform_driver! {
    type: I2cBcm2835,
    name: "I2cBcm2835",
    author: "Rust for Linux Contributors",
    license: "GPL",
    initcall: "arch",
}

struct I2cBcm2835;

// todo 常量映射
// 分频器寄存器的偏移量
const BCM2835_I2C_DIV: usize = 0x14;
const BCM2835_I2C_DEL: usize = 0x18;
const BCM2835_I2C_CLKT: usize = 0x1c;

pub const BCM2835_I2C_FEDL_SHIFT: u32 = 16;
pub const BCM2835_I2C_REDL_SHIFT: u32 = 0;

const BCM2835_I2C_CDIV_MIN: u32 = 0x0002;
const BCM2835_I2C_CDIV_MAX: u32 = 0xFFFE;

static CLK_TOUT_MS: u32 = 35;

fn clk_bcm2835_i2c_calc_divider (rate: u64, parent_rate: u64) -> Result<u32> {
    let mut divider:u32 = parent_rate.div_ceil(rate) as u32;
    if (divider & 0x1) != 0 {
        divider += 1;
    }
    if (divider < BCM2835_I2C_CDIV_MIN) || (divider > BCM2835_I2C_CDIV_MAX) {
        return Err(EINVAL);
    }
    Ok(divider)
}

fn clk_bcm2835_i2c_set_rate(hw: &clk_provider::ClkHw, rate: u64, parent_rate: u64) -> Result {
    pr_info!("i2c set rate!");

    let clk_bcm2835_i2c = to_clk_bcm2835_i2c(hw);

    let divider = clk_bcm2835_i2c_calc_divider(rate, parent_rate)?;

    clk_bcm2835_i2c.bcm2835_i2c_writel(BCM2835_I2C_DIV, divider);

    /*
    * Number of core clocks to wait after falling edge before
    * outputting the next data bit.  Note that both FEDL and REDL
    * can't be greater than CDIV/2.
    */
    let fedl = max(divider / 16, 1);

    /*
    * Number of core clocks to wait after rising edge before
    * sampling the next incoming data bit.
    */
    let redl = max(divider / 4, 1);

    clk_bcm2835_i2c.bcm2835_i2c_writel(BCM2835_I2C_DEL, 
        ((fedl << BCM2835_I2C_FEDL_SHIFT) | (redl << BCM2835_I2C_REDL_SHIFT)) as u32);
    
    
    let clk_tout ={
        if rate as u32 > (0xffff * 1000 / CLK_TOUT_MS) {
            0xffff
        } else {
            CLK_TOUT_MS * rate as u32 / 1000
        }
    };
    
    clk_bcm2835_i2c.bcm2835_i2c_writel(BCM2835_I2C_CLKT, clk_tout);

    Ok(())
}

fn clk_bcm2835_i2c_round_rate(hw: &clk_provider::ClkHw, rate: u64, parent_rate: &mut u64) -> i64 {
    pr_info!("i2c round rate!");
    
    if let Ok(divider) = clk_bcm2835_i2c_calc_divider(rate, *parent_rate) {
        let divider: u64 = divider as u64;
        return parent_rate.div_ceil(divider) as i64;
    } else {
        return 0;
    }
}

fn clk_bcm2835_i2c_recalc_rate(hw: &clk_provider::ClkHw, parent_rate: u64) -> u64 {
    pr_info!("i2c recalc rate!");
    
    let clk_bcm2835_i2c = to_clk_bcm2835_i2c(hw);

    let divider = clk_bcm2835_i2c.bcm2835_i2c_readl(BCM2835_I2C_DIV) as u64;

    parent_rate.div_ceil(divider)
}


struct I2cClkOps;
#[vtable]
impl clk_provider::ClkOps for I2cClkOps{
    fn set_rate(hw: &clk_provider::ClkHw, rate: u64, parent_rate: u64) -> Result {
        pr_info!("i2c set rate!");
        clk_bcm2835_i2c_set_rate(hw, rate, parent_rate)
    }

    fn round_rate(hw: &clk_provider::ClkHw, rate: u64, parent_rate: &mut u64) -> i64 {
        pr_info!("i2c round rate!");
        clk_bcm2835_i2c_round_rate(hw, rate, parent_rate)
    }

    fn recalc_rate(hw: &clk_provider::ClkHw, parent_rate: u64) -> u64 {
        pr_info!("i2c recalc rate!");
        clk_bcm2835_i2c_recalc_rate(hw, parent_rate)
    }
}

// 返回包含 hw 的 ClkBcm2835I2c 类型对象
fn to_clk_bcm2835_i2c(hw: &clk_provider::ClkHw) -> &ClkBcm2835I2c {
    unsafe {
        &*container_of!(hw, ClkBcm2835I2c, clk_hw)
    }
}
struct ClkBcm2835I2c {
    clk_hw: clk_provider::ClkHw,
    // CHECK
    // i2c_dev: &'s I2cBcm2835Data
    reg_base: *mut u8,
}

impl ClkBcm2835I2c {
    fn from_raw<'a>(ptr: *mut Self) -> &'a mut Self {
        let ptr = ptr.cast::<Self>();
        unsafe { &mut *ptr }
    }

    // 将值写入分频器寄存器
    pub(crate) fn bcm2835_i2c_writel(&self, reg: usize, val: u32) {
        let addr = self.reg_base.wrapping_add(reg);
        unsafe { bindings::writel(val as _, addr as _); }
    }
    
    // 读取分频器寄存器的值
    pub(crate) fn bcm2835_i2c_readl(&self, reg: usize) -> u32 {
        let reg_base = self.reg_base;
        pr_info!("reg_base: {:?}",reg_base);
        let addr = reg_base.wrapping_add(reg);
        unsafe { bindings::readl(addr as _)}
    }

}

mod i2c_func {
    use kernel::bindings;
    use kernel::types::Opaque;

    struct I2cMsg {
        ptr: *mut bindings::i2c_msg,
    }

    
    
    pub struct I2cAdapter<T:I2cAlgo> {
        adapter: Opaque<bindings::i2c_adapter>,
        _phantom: PhantomData<T>,
    }

    impl I2cAdapter {
        fn i2c_get_apadata(&self) {

        }
    }

}

struct I2cBcm2835Algo;

#[vtable]
impl I2cBcm2835Algo {

}

// CHECK
unsafe fn i2c_kzalloc<T>(device: &device::Device) -> Result<*mut T>{
    let size = core::mem::size_of::<T>();
    let ptr = unsafe {
        bindings::devm_kmalloc(device.raw_device(), size, bindings::GFP_KERNEL)
    };
    if ptr.is_null() {
        return Err(ENOMEM);
    }
    Ok(ptr as *mut T)
}

struct I2cBcm2835IrqHandler;

impl irq::Handler for I2cBcm2835IrqHandler {
    type Data = Arc<I2cBcm2835Data>;

    fn handle_irq(data: ArcBorrow<'_, DwI2cData>) -> irq::Return {
        
        irq::Return::None
    }
}





fn of_property_read_u32(device: &device::Device, propname: &'static CStr, val: *mut u32) -> Result {
    to_result(unsafe {
        let np = (*device.raw_device()).of_node;
        bindings::of_property_read_variable_u32_array(np, propname.as_char_ptr(), val, 1, 0)
    })
}

fn clk_set_rate_exclusive(bus_clk: &Clk, bus_clk_rate: u32) -> Result {
    to_result(unsafe{
        bindings::clk_set_rate_exclusive(bus_clk.as_ptr(), bus_clk_rate.into())
    })
}

fn clk_prepare_enable(bus_clk: &Clk) -> Result {
    unsafe {
        let mut ret = bindings::clk_prepare(bus_clk.as_ptr());
        if ret == 0{
            return Ok(());
        }
        ret = bindings::clk_enable(bus_clk.as_ptr());
        if ret == 0 {
            bindings::clk_unprepare(bus_clk.as_ptr())
        }
        to_result(ret)
    }
}

// fn request_irq(irq: i32, )
struct I2cBcm2835Data {
    dev: device::Device,
    reg_base: *mut u8,
    irq: irq::Registration::<I2cBcm2835IrqHandler>,
    // i2c_adapter: I2cBcm2835Adapter,
    // completion: completion::Completion,
}

impl I2cBcm2835Data {
    // Create I2cBcm2835Data from raw ptr
    pub(crate) fn new(
        dev: device::Device, 
        reg_base: *mut u8, 
        irq: u32
    ) -> Arc<Self> {

    }
    
    unsafe fn from_raw<'a>(ptr: *mut Self) -> &'a mut Self {
        let ptr = ptr.cast::<Self>();
        unsafe {&mut *ptr}  
    }
    
    // CHECK
    unsafe fn as_ptr(&self) -> *const Self {
        self as *const Self
    }
    
    // 将值写入分频器寄存器
    pub(crate) fn bcm2835_i2c_writel(&self, reg: usize, val: u32) {
        let addr = self.reg_base.wrapping_add(reg);
        unsafe { bindings::writel(val as _, addr as _); }
    }
    
    // 读取分频器寄存器的值
    pub(crate) fn bcm2835_i2c_readl(&self, reg: usize) -> u32 {
        let addr = self.reg_base.wrapping_add(reg);
        unsafe { bindings::readl(addr as _) }
    }

    pub(crate) fn bcm2835_i2c_register_div<'a>(dev: &'a device::Device, mclk: &'a Clk, reg_base: *mut u8) -> Result<&'a mut Clk> {
        let mclk_name = unsafe {
            CStr::from_char_ptr(bindings::__clk_get_name(mclk.as_ptr()))
        };
        
        let name = CString::try_from_fmt(fmt!("{}_div", dev.name()))?;
        let parent_names = [mclk_name.as_char_ptr()];

        let clk_bcm2835_i2c = unsafe {
            let raw_ptr = i2c_kzalloc::<ClkBcm2835I2c>(dev)?;
            let clk_bcm2835_i2c = ClkBcm2835I2c::from_raw(raw_ptr);
            let clk_init_data = ClkInitData::new()
                .name_config(&name, &parent_names)
                .set_ops::<I2cClkOps>()
                .set_flags(0);
            clk_bcm2835_i2c.clk_hw.set_init_data(&clk_init_data);
            // CHECK
            // clk_bcm2835_i2c.i2c_dev = self;
            clk_bcm2835_i2c.reg_base = reg_base;
            clk_bcm2835_i2c.clk_hw.register_clkdev(c_str!("div"), dev.name())?;
            clk_bcm2835_i2c
        };
        
        let raw_ptr = unsafe {
            from_err_ptr(bindings::devm_clk_register(dev.raw_device(), clk_bcm2835_i2c.clk_hw.as_mut_ptr()))?
        }; 
        Ok(Clk::from_raw(raw_ptr))
    }

    pub(crate) fn request_irq(irq: u32, data: Box<u32>) -> Result<irq::Registration<Example>> {
        irq::Registration::try_new(irq, data, irq::flags::SHARED, fmt!("example_{irq}"))
    }
}

// CHECK
unsafe impl Send for I2cBcm2835Data {}
unsafe impl Sync for I2cBcm2835Data {}

impl driver::DeviceRemoval for I2cBcm2835Data {
    fn device_remove(&self){
        pr_info!("i2c device data drop");
        // todo
    }
}

struct I2cBcm2835IdInfo {}


define_of_id_table! {MY_ID_TABLE, (), [
    (of::DeviceId::Compatible(b"brcm,bcm2711-i2c"), None),
    (of::DeviceId::Compatible(b"brcm,bcm2835-i2c"), None),
    (of::DeviceId::Compatible(b"snps,designware-i2c"), None),
]}

module_of_id_table!(MOD_TABLE, MY_ID_TABLE);

impl platform::Driver for I2cBcm2835 {

    type Data = Arc<I2cBcm2835Data>;
    
    //type IdInfo = I2cBcm2835IdInfo;

    driver_of_id_table!(MY_ID_TABLE);

    fn probe(dev: &mut platform::Device, _id_info: Option<&Self::IdInfo>) -> Result<Self::Data> {
        dev_info!(dev,"BCM2835 i2c bus device ({}) driver probe.\n",dev.name());

        let device= device::Device::from_dev(dev);

        let reg_base = dev.ioremap_resource(0)?;
        pr_info!("reg base: {:?}", reg_base);
        
        let mclk = device.clk_get()?;
        let bus_clk = I2cBcm2835Data::bcm2835_i2c_register_div(&device, mclk, reg_base)?;

        let mut bus_clk_rate: u32 = 0;
        // CHECK

        if let Err(_) = of_property_read_u32(&device, c_str!("clock-frequency"), &mut bus_clk_rate) {
            bus_clk_rate = bindings::I2C_MAX_STANDARD_MODE_FREQ;
            dev_err!(dev,"Could not read clock-frequency property\n",);
        }

        if let Err(_) = clk_set_rate_exclusive(bus_clk, bus_clk_rate) {
            dev_err!(dev,"Could not set clock frequency\n",);
        }
        
        if let Err(_) = clk_prepare_enable(bus_clk) {
            dev_err!(dev,"Couldn't prepare clock\n",);
        }


        let irq = dev.irq_resource(0)?;

        let device_data = Arc::try_new(I2cBcm2835Data{
            dev: device,
            reg_base: reg_base,
        })?;

        dev_info!(dev,"BCM2835 i2c bus device ({}) driver probe END.\n",dev.name());
        Ok(device_data)
    }
    fn remove(_data: &Self::Data) -> Result {
        // TODO: remove i2c driver
        Ok(())
    }
}
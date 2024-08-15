// SPDX-License-Identifier: GPL-2.0

//! Rust i2c driver.
use core::result::Result::Ok;

use kernel::prelude::*;
use kernel::{platform, module_platform_driver, of, driver};
use kernel::{define_of_id_table, device, driver_of_id_table, module_of_id_table};
use kernel::bindings;
use kernel::sync::Arc;
use kernel::error::Result;
use kernel::completion;
use kernel::clk::{self, Clk};
use kernel::clk_provider;
use kernel::str::{CStr, CString};
use kernel::types::Opaque;

module_platform_driver! {
    type: I2cBcm2835,
    name: "I2cBcm2835",
    author: "Rust for Linux Contributors",
    license: "GPL",
    initcall: "arch",
}

struct I2cBcm2835;

// todo 常量映射
const BCM2835_I2C_CDIV_MIN: u32 = 0x0002;
const BCM2835_I2C_CDIV_MAX: u32 = 0xFFFE;

fn clk_bcm2835_i2c_calc_divider (rate: u64, parent_rate: &mut u64) -> Result<u32> {
    let mut divider:u32 = parent_rate.div_ceil(rate) as u32;
    if (divider & 0x1) != 0 {
        divider += 1;
    }
    if (divider < BCM2835_I2C_CDIV_MIN) || (divider > BCM2835_I2C_CDIV_MAX) {
        return Err(EINVAL);
    }
    Ok(divider)
}

struct I2cClkOps;

#[vtable]
impl clk_provider::ClkOps for I2cClkOps{
    fn set_rate(hw: &clk_provider::ClkHw, rate: u64, parent_rate: u64) -> Result {
        // todo
        Err(ENOTSUPP)
    }

    fn round_rate(hw: &clk_provider::ClkHw, rate: u64, parent_rate: &mut u64) -> i64 {
        if let Ok(divider) = clk_bcm2835_i2c_calc_divider(rate, parent_rate) {
            let divider: u64 = divider as u64;
            return parent_rate.div_ceil(divider) as i64;
        } else {
            return 0;
        }
    }

    fn recalc_rate(hw: &clk_provider::ClkHw, parent_rate: u64) -> u64 {
        // todo
        0
    }
}

struct ClkBcm2835I2c {
    clk_hw: clk_provider::ClkHw
    // CHECK
    // i2c_dev: & Bcm2835I2cDev
}

impl ClkBcm2835I2c {

}

struct I2cBcm2835Adapter {}

struct I2cBcm2835Data {
    // dev: device::Device,
    // reg_base: *mut u8,
    // irq: i32,
    // i2c_adapter: I2cBcm2835Adapter,
    // completion: completion::Completion,
}

impl I2cBcm2835Data {
    // pub fn bcm2835_i2c_register_div(&mut self, mclk: &Clk) -> Result<&mut Clk> {
    pub(crate) fn bcm2835_i2c_register_div(&mut self, mclk: &Clk) {
        let mclk_name = unsafe {
            CStr::from_char_ptr(bindings::__clk_get_name(mclk.as_ptr()))
        };
        
        //let name = CString::try_from_fmt(fmt!("{}_div", self.dev.name))?;
        let parent_names = [mclk_name.as_char_ptr()];


        let clk_init_data = clk_provider::ClkInitData::new();

        // todo


    }
}

impl driver::DeviceRemoval for I2cBcm2835Data {
    fn device_remove(&self){
        pr_info!("I2cBcm2835_Rust: i2c device data drop");
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
        pr_info!("I2cBcm2835_Rust: i2c device probe!");

        let device = device::Device::from_dev(dev);

        let reg_base = dev.ioremap_resource(0)?;
        
        let mclk = device.clk_get()?;

        // todo
        //let bus_clk = 

        let device_data = Arc::try_new(I2cBcm2835Data {})?;
        Ok(device_data)
    }
    fn remove(_data: &Self::Data) -> Result {
        // TODO: remove i2c driver
        Ok(())
    }
}
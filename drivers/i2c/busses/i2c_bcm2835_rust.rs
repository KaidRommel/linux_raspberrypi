// SPDX-License-Identifier: GPL-2.0

//! Rust i2c driver.
use kernel::{platform, module_platform_driver, of, driver, sync::Arc};
use kernel::{define_of_id_table, module_of_id_table, driver_of_id_table};
use kernel::error::Result;
use kernel::prelude::*;

use kernel::{define_id_array,driver_id_table};

module_platform_driver! {
    type: I2cBcm2835,
    name: "I2cBcm2835",
    author: "Rust for Linux Contributors",
    license: "GPL",
    initcall: "arch",
}

struct I2cBcm2835Data {
    
}

impl driver::DeviceRemoval for I2cBcm2835Data {
    fn device_remove(&self){
        pr_info!("I2cBcm2835_Rust: i2c device data drop");
    }
}
struct I2cBcm2835IdInfo {}
struct I2cBcm2835;

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

    fn probe(_dev: &mut platform::Device, _id_info: Option<&Self::IdInfo>) -> Result<Self::Data> {
        pr_info!("I2cBcm2835_Rust: i2c device probe!");
        let device_data = Arc::try_new(I2cBcm2835Data {}).unwrap();
        Ok(device_data)
    }
    fn remove(_data: &Self::Data) -> Result {
        // TODO: remove i2c driver
        Ok(())
    }
}
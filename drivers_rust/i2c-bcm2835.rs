// SPDX-License-Identifier: GPL-2.0

//! Rust i2c driver.
use kernel::{platform, module_driver, module_platform_driver, of, pr_info};
use kernel::{define_of_id_table, module_of_id_table, driver_of_id_table,module_id_table};

module_platform_driver! {
    type: I2cBcm2835,
    name: "i2c_bcm2835",
    author: "Rust for Linux Contributors",
    license: "GPL",
}

struct I2cBcm2835;

define_of_id_table! {MY_ID_TABLE, (), [
    (of::DeviceId::Compatible(b"brcm,bcm2711-i2c"), None),
    (of::DeviceId::Compatible(b"brcm,bcm2835-i2c"), None),
]}

module_of_id_table!(MOD_TABLE, MY_ID_TABLE);

impl platform::Driver for I2cBcm2835 {
    driver_of_id_table!(MY_ID_TABLE);
    fn probe(_dev: &mut platform::Device, _id_info: Option<&Self::IdInfo>) -> Result {
        pr_info!("i2c probe!");
        Ok(())
    }
    fn remove(_data: &Self::Data) -> Result {
        // TODO: remove i2c driver
        Ok(())
    }
}
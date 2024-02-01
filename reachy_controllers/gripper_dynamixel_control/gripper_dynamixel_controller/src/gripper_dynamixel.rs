use std::time::Duration;

use cache_cache::Cache;
use log::info;
use rustypot::{device::mx, DynamixelSerialIO};
use serde::{Deserialize, Serialize};
use serialport::TTYPort;

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

pub struct GripperDynamixel {
    serial_port: Box<TTYPort>,
    io: DynamixelSerialIO,
    id: u8,

    target_position: Cache<u8, f64>,
    torque_on: Cache<u8, bool>,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct GripperDynamixelConfig {
    pub serial_port_name: String,
    pub id: u8,
}

impl GripperDynamixel {
    pub fn new(serial_port_name: &str, id: u8) -> Result<Self> {
        let mut controller = GripperDynamixel {
            serial_port: Box::new(
                serialport::new(serial_port_name, 2_000_000)
                    .timeout(Duration::from_millis(10))
                    .open_native()?,
            ),
            io: DynamixelSerialIO::v1(),
            id,
            target_position: Cache::keep_last(),
            torque_on: Cache::keep_last(),
        };

        controller.serial_port.set_exclusive(false)?;

        info!("GripperDynamixel {} {} initialized", serial_port_name, id);

        Ok(controller)
    }

    pub fn with_config(configfile: &str) -> Result<Self> {
        let f = std::fs::File::open(configfile)?;
        let config: GripperDynamixelConfig = serde_yaml::from_reader(f)?;

        GripperDynamixel::new(config.serial_port_name.as_str(), config.id)
    }

    /// Checks if the torque is on.
    pub fn is_torque_on(&mut self) -> Result<bool> {
        self.torque_on.entry(self.id).or_try_insert_with(|_| {
            mx::read_torque_enable(&self.io, self.serial_port.as_mut(), self.id)
                .map(|torque| torque != 0)
        })
    }
    /// Enables the torque on all two motors.
    pub fn enable_torque(&mut self) -> Result<()> {
        self.set_torque(true)
    }
    /// Disables the torque on all two motors.
    pub fn disable_torque(&mut self) -> Result<()> {
        self.set_torque(false)
    }
    fn set_torque(&mut self, torque: bool) -> Result<()> {
        let current_torque = self.is_torque_on()?;

        if torque != current_torque {
            mx::write_torque_enable(&self.io, self.serial_port.as_mut(), self.id, torque as u8)?;

            self.torque_on.insert(self.id, torque);
        }

        Ok(())
    }

    pub fn get_current_position(&mut self) -> Result<f64> {
        Ok(mx::conv::dxl_pos_to_radians(mx::read_present_position(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
        )?))
    }

    pub fn get_current_target_position(&mut self) -> Result<f64> {
        self.target_position.entry(self.id).or_try_insert_with(|_| {
            Ok(mx::conv::dxl_pos_to_radians(mx::read_goal_position(
                &self.io,
                self.serial_port.as_mut(),
                self.id,
            )?))
        })
    }

    pub fn set_target_position(&mut self, target_position: f64) -> Result<()> {
        let current_target = self.get_current_target_position()?;

        if current_target != target_position {
            mx::write_goal_position(
                &self.io,
                self.serial_port.as_mut(),
                self.id,
                mx::conv::radians_to_dxl_pos(target_position),
            )?;

            self.target_position.insert(self.id, target_position);
        }

        Ok(())
    }
}

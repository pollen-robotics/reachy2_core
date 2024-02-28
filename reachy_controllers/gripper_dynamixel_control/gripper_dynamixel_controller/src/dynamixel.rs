use std::time::Duration;

use cache_cache::Cache;
use log::info;
use motor_toolbox_rs::{MissingResisterErrror, MotorsController, RawMotorsIO, PID};
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
            io: DynamixelSerialIO::v1().with_post_delay(Duration::from_millis(1)),
            id,
            target_position: Cache::keep_last(),
            torque_on: Cache::keep_last(),
        };

        controller.serial_port.set_exclusive(false)?;

        info!("GripperDynamixel {} {} initialized", serial_port_name, id);

        Ok(controller)
    }

    pub fn with_config(config: GripperDynamixelConfig) -> Result<Self> {
        GripperDynamixel::new(config.serial_port_name.as_str(), config.id)
    }
}

impl MotorsController<1> for GripperDynamixel {
    fn io(&mut self) -> &mut dyn motor_toolbox_rs::RawMotorsIO<1> {
        self
    }

    fn offsets(&self) -> [Option<f64>; 1] {
        [None]
    }

    fn reduction(&self) -> [Option<f64>; 1] {
        [None]
    }

    fn limits(&self) -> [Option<motor_toolbox_rs::Limit>; 1] {
        [None]
    }
}

impl RawMotorsIO<1> for GripperDynamixel {
    /// Checks if the torque is on.
    fn is_torque_on(&mut self) -> Result<[bool; 1]> {
        self.torque_on
            .entry(self.id)
            .or_try_insert_with(|_| {
                mx::read_torque_enable(&self.io, self.serial_port.as_mut(), self.id)
                    .map(|torque| torque != 0)
            })
            .map(|x| [x])
    }
    fn set_torque(&mut self, torque: [bool; 1]) -> Result<()> {
        let current_torque = RawMotorsIO::is_torque_on(self)?;

        if torque != current_torque {
            mx::write_torque_enable(
                &self.io,
                self.serial_port.as_mut(),
                self.id,
                torque[0] as u8,
            )?;

            self.torque_on.insert(self.id, torque[0]);
        }

        Ok(())
    }

    fn get_current_position(&mut self) -> Result<[f64; 1]> {
        Ok([mx::conv::dxl_pos_to_radians(mx::read_present_position(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
        )?)])
    }

    fn get_current_torque(&mut self) -> Result<[f64; 1]> {
        Err(MissingResisterErrror("current_torque".to_string()).into())
    }

    fn get_current_velocity(&mut self) -> Result<[f64; 1]> {
        Err(MissingResisterErrror("current_velocity".to_string()).into())
    }

    fn get_target_position(&mut self) -> Result<[f64; 1]> {
        self.target_position
            .entry(self.id)
            .or_try_insert_with(|_| {
                Ok(mx::conv::dxl_pos_to_radians(mx::read_goal_position(
                    &self.io,
                    self.serial_port.as_mut(),
                    self.id,
                )?))
            })
            .map(|x| [x])
    }

    fn set_target_position(&mut self, target_position: [f64; 1]) -> Result<()> {
        let current_target = RawMotorsIO::get_target_position(self)?;

        if current_target != target_position {
            mx::write_goal_position(
                &self.io,
                self.serial_port.as_mut(),
                self.id,
                mx::conv::radians_to_dxl_pos(target_position[0]),
            )?;

            self.target_position.insert(self.id, target_position[0]);
        }

        Ok(())
    }

    fn get_velocity_limit(&mut self) -> Result<[f64; 1]> {
        Err(MissingResisterErrror("velocity_limit".to_string()).into())
    }
    fn set_velocity_limit(&mut self, _velocity_limit: [f64; 1]) -> Result<()> {
        Err(MissingResisterErrror("velocity_limit".to_string()).into())
    }
    fn get_torque_limit(&mut self) -> Result<[f64; 1]> {
        Err(MissingResisterErrror("torque_limit".to_string()).into())
    }
    fn set_torque_limit(&mut self, _torque_limit: [f64; 1]) -> Result<()> {
        Err(MissingResisterErrror("torque_limit".to_string()).into())
    }
    fn get_pid_gains(&mut self) -> Result<[PID; 1]> {
        Err(MissingResisterErrror("pid_gains".to_string()).into())
    }
    fn set_pid_gains(&mut self, _pid_gains: [PID; 1]) -> Result<()> {
        Err(MissingResisterErrror("pid_gains".to_string()).into())
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn parse_dynamixel_config_file() {
        let f = std::fs::File::open("./config/dynamixel.yaml").unwrap();

        let config: Result<crate::GripperConfig, _> = serde_yaml::from_reader(f);
        assert!(config.is_ok());

        let config = config.unwrap();

        if let crate::GripperIOConfig::DynamixelSerialIO(c) = config.io {
            assert_eq!(c.serial_port_name, "/dev/left_gripper");
            assert_eq!(c.id, 23);
        } else {
            panic!("Wrong config type");
        }
    }
}

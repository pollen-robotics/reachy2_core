use cache_cache::Cache;
use log::info;
use motor_toolbox_rs::{MissingRegisterErrror, MotorsController, RawMotorsIO, PID};
use rustypot::{device::xm, DynamixelSerialIO};

use serialport::TTYPort;
use std::time::Duration;

// use crate::Dynamixel;
use crate::DynamixelConfigInfo;

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

pub struct XMDynamixel {
    serial_port: Box<TTYPort>,
    io: DynamixelSerialIO,
    id: u8,

    target_position: Cache<u8, f64>,

    torque_on: Cache<u8, bool>,

    velocity_limit: Cache<u8, f64>,
    torque_limit: Cache<u8, f64>,
    control_mode: Cache<u8, u8>,
}

impl XMDynamixel {
    pub fn new(serial_port_name: &str, id: u8, mode: u8, current_limit: f64) -> Result<Self> {
        let mut controller = XMDynamixel {
            serial_port: Box::new(
                serialport::new(serial_port_name, 2_000_000)
                    .timeout(Duration::from_millis(10))
                    .open_native()?,
            ),
            io: DynamixelSerialIO::v2().with_post_delay(Duration::from_millis(1)),
            id,
            target_position: Cache::keep_last(),
            torque_on: Cache::keep_last(),
            velocity_limit: Cache::keep_last(),
            torque_limit: Cache::keep_last(),
            control_mode: Cache::keep_last(),
        };

        controller.serial_port.set_exclusive(false)?; //To be able to share the serial port

        motor_toolbox_rs::MotorsController::set_control_mode(&mut controller, [mode])?; //control mode 3=standard position control 5=torque based position control
        let mode = motor_toolbox_rs::MotorsController::get_control_mode(&mut controller)?;
        info!(
            "XMDynamixel {} {} initialized, operating mode: {}",
            serial_port_name, id, mode[0]
        );

        motor_toolbox_rs::MotorsController::set_torque_limit(&mut controller, [current_limit])?;
        let tlimit = motor_toolbox_rs::MotorsController::get_torque_limit(&mut controller)?;
        info!(
            "XMDynamixel {} {} initialized, torque_limit: {}",
            serial_port_name, id, tlimit[0]
        );

        motor_toolbox_rs::MotorsController::set_target_torque(&mut controller, [current_limit])?;
        let ttorque = motor_toolbox_rs::MotorsController::get_target_torque(&mut controller)?;
        info!(
            "XMDynamixel {} {} initialized, target_torque: {}",
            serial_port_name, id, ttorque[0]
        );

        Ok(controller)
    }

    pub fn with_config(config: DynamixelConfigInfo) -> Result<Self> {
        XMDynamixel::new(
            config.serial_port_name.as_str(),
            config.id,
            config.mode,
            config.current_limit,
        )
    }
}

impl MotorsController<1> for XMDynamixel {
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

impl RawMotorsIO<1> for XMDynamixel {
    /// Checks if the torque is on.
    fn is_torque_on(&mut self) -> Result<[bool; 1]> {
        self.torque_on
            .entry(self.id)
            .or_try_insert_with(|_| {
                xm::read_torque_enable(&self.io, self.serial_port.as_mut(), self.id)
                    .map(|torque| torque != 0)
            })
            .map(|x| [x])
    }
    fn set_torque(&mut self, torque: [bool; 1]) -> Result<()> {
        let current_torque = RawMotorsIO::is_torque_on(self)?;

        if torque != current_torque {
            xm::write_torque_enable(
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
        Ok([xm::conv::dxl_pos_to_radians(xm::read_present_position(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
        )?) as f64])
    }

    fn get_current_torque(&mut self) -> Result<[f64; 1]> {
        Ok([xm::conv::dxl_current_to_ma(xm::read_present_current(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
        )?) as f64])
    }

    fn get_current_velocity(&mut self) -> Result<[f64; 1]> {
        Ok([xm::conv::dxl_vel_to_rpm(xm::read_present_velocity(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
        )?) as f64])
    }

    fn get_target_position(&mut self) -> Result<[f64; 1]> {
        self.target_position
            .entry(self.id)
            .or_try_insert_with(|_| {
                Ok(xm::conv::dxl_pos_to_radians(xm::read_goal_position(
                    &self.io,
                    self.serial_port.as_mut(),
                    self.id,
                )?) as f64)
            })
            .map(|x| [x])
    }

    fn set_target_position(&mut self, target_position: [f64; 1]) -> Result<()> {
        let current_target = RawMotorsIO::get_target_position(self)?;

        if current_target != target_position {
            xm::write_goal_position(
                &self.io,
                self.serial_port.as_mut(),
                self.id,
                xm::conv::radians_to_dxl_pos(target_position[0] as f32),
            )?;

            self.target_position.insert(self.id, target_position[0]);
        }

        Ok(())
    }

    fn get_velocity_limit(&mut self) -> Result<[f64; 1]> {
        self.velocity_limit
            .entry(self.id)
            .or_try_insert_with(|_| {
                Ok(xm::conv::dxl_vel_to_rpm(xm::read_velocity_limit(
                    &self.io,
                    self.serial_port.as_mut(),
                    self.id,
                )? as i32) as f64)
            })
            .map(|x| [x])
    }
    fn set_velocity_limit(&mut self, velocity_limit: [f64; 1]) -> Result<()> {
        let current_velocity_limit = RawMotorsIO::get_velocity_limit(self)?;

        if current_velocity_limit != velocity_limit {
            xm::write_velocity_limit(
                &self.io,
                self.serial_port.as_mut(),
                self.id,
                xm::conv::rpm_to_dxl_vel(velocity_limit[0] as f32) as u32,
            )?;

            self.velocity_limit.insert(self.id, velocity_limit[0]);
        }

        Ok(())
    }

    fn get_torque_limit(&mut self) -> Result<[f64; 1]> {
        self.torque_limit
            .entry(self.id)
            .or_try_insert_with(|_| {
                Ok((xm::conv::dxl_current_to_ma(xm::read_torque_limit(
                    &self.io,
                    self.serial_port.as_mut(),
                    self.id,
                )? as i16) as f32
                    / 1000.0) as f64)
            })
            .map(|x| [x])
    }

    fn set_torque_limit(&mut self, torque_limit: [f64; 1]) -> Result<()> {
        let current_torque_limit = RawMotorsIO::get_torque_limit(self)?;

        if current_torque_limit != torque_limit {
            xm::write_current_limit(
                &self.io,
                self.serial_port.as_mut(),
                self.id,
                xm::conv::ma_to_dxl_current((torque_limit[0] * 1000.0) as f32) as u16,
            )?;

            self.torque_limit.insert(self.id, torque_limit[0]);
        }

        Ok(())
    }

    /// Set the target current (A)
    fn set_target_torque(&mut self, target: [f64; 1]) -> Result<()> {
        xm::write_goal_current(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
            xm::conv::ma_to_dxl_current((target[0] * 1000.0) as f32) as i16,
        )?;
        Ok(())
    }
    /// Get the target current (A)
    fn get_target_torque(&mut self) -> Result<[f64; 1]> {
        let cur = xm::conv::dxl_current_to_ma(xm::read_goal_current(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
        )?) / 1000.0;
        Ok([cur as f64])
    }

    /// Set the target velocity (rad/s)
    fn set_target_velocity(&mut self, target: [f64; 1]) -> Result<()> {
        xm::write_goal_velocity(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
            xm::conv::rpm_to_dxl_vel((target[0] * 9.5492999999998) as f32) as i32,
        )?;
        Ok(())
    }
    /// Get the target velocity (rad/s)
    fn get_target_velocity(&mut self) -> Result<[f64; 1]> {
        let cur = xm::conv::dxl_vel_to_rpm(xm::read_goal_velocity(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
        )?) / 9.5492999999998;
        Ok([cur as f64])
    }

    /// Get the current control mode
    fn get_control_mode(&mut self) -> Result<[u8; 1]> {
        self.control_mode
            .entry(self.id)
            .or_try_insert_with(|_| {
                Ok((xm::read_operating_mode(&self.io, self.serial_port.as_mut(), self.id)?))
            })
            .map(|x| [x])

        // Ok([xm::read_operating_mode(
        //     &self.io,
        //     self.serial_port.as_mut(),
        //     self.id,
        // )?])
    }

    /// Get the temperature (°C)
    fn get_motors_temperature(&mut self) -> Result<[f64; 1]> {
        let temp = xm::conv::dxl_to_temperature(xm::read_present_temperature(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
        )?);
        Ok([temp as f64])
    }

    /// Set the current control mode
    fn set_control_mode(&mut self, mode: [u8; 1]) -> Result<()> {
        let ctrl_mode = RawMotorsIO::get_control_mode(self)?;

        if ctrl_mode != mode {
            xm::write_operating_mode(&self.io, self.serial_port.as_mut(), self.id, mode[0])?;

            self.control_mode.insert(self.id, mode[0]);
        }

        // xm::write_operating_mode(&self.io, self.serial_port.as_mut(), self.id, mode[0])?;
        Ok(())
    }

    fn get_pid_gains(&mut self) -> Result<[PID; 1]> {
        Err(MissingRegisterErrror("pid_gains".to_string()).into())
    }
    fn set_pid_gains(&mut self, _pid_gains: [PID; 1]) -> Result<()> {
        Err(MissingRegisterErrror("pid_gains".to_string()).into())
    }

    fn set_target_position_fb(&mut self, _: [f64; 1]) -> Result<[f64; 1]> {
        Err(MissingRegisterErrror("target_position_fb".to_string()).into())
    }
    fn get_axis_sensors(&mut self) -> Result<[f64; 1]> {
        Err(MissingRegisterErrror("axis_sensor".to_string()).into())
    }

    fn get_board_state(&mut self) -> Result<u8> {
        Err(MissingRegisterErrror("board_state".to_string()).into())
    }
    fn set_board_state(&mut self, _: u8) -> Result<()> {
        Err(MissingRegisterErrror("board_state".to_string()).into())
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn parse_dynamixel_config_file() {
        let f = std::fs::File::open("./config/dynamixel.yaml").unwrap();

        let config: Result<crate::XMConfig, _> = serde_yaml::from_reader(f);
        assert!(config.is_ok());

        let config = config.unwrap();

        if let crate::DxlIOConfig::DynamixelSerialIO(c) = config.io {
            assert_eq!(c.serial_port_name, "/dev/left_antenna");
            assert_eq!(c.id, 23);
        } else {
            panic!("Wrong config type");
        }
    }
}

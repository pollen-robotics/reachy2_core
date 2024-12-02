use cache_cache::Cache;
use log::error;
use log::info;
use log::warn;

use motor_toolbox_rs::{MissingRegisterErrror, MotorsController, RawMotorsIO, PID};
use rustypot::{device::xl330, DynamixelSerialIO};
use serialport::TTYPort;
use std::time::Duration;

// use crate::Dynamixel;
use crate::DynamixelConfigInfo;

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

pub struct XL330Dynamixel {
    serial_port: Box<TTYPort>,
    io: DynamixelSerialIO,
    id: u8,

    target_position: Cache<u8, f64>,

    torque_on: Cache<u8, bool>,

    velocity_limit: Cache<u8, f64>,
    torque_limit: Cache<u8, f64>,
    control_mode: Cache<u8, u8>,
}

impl XL330Dynamixel {
    pub fn new(serial_port_name: &str, id: u8, mode: u8, current_limit: f64) -> Result<Self> {
        let mut controller = XL330Dynamixel {
            serial_port: Box::new(
                serialport::new(serial_port_name, 2_000_000)
                    .timeout(Duration::from_millis(10))
                    .open_native()?,
            ),
            io: DynamixelSerialIO::v2().with_post_delay(Duration::from_millis(2)),
            id,
            target_position: Cache::keep_last(),
            torque_on: Cache::keep_last(),
            velocity_limit: Cache::keep_last(),
            torque_limit: Cache::keep_last(),
            control_mode: Cache::keep_last(),
        };

        controller.serial_port.set_exclusive(false)?; //To be able to share the serial port

        let torque_on = motor_toolbox_rs::MotorsController::is_torque_on(&mut controller)?;

        if torque_on[0] {
            motor_toolbox_rs::MotorsController::set_control_mode(&mut controller, [mode])?; //control mode 3=standard position control 5=torque based position control. Looks like torque should be off to change it?
            let rmode = motor_toolbox_rs::MotorsController::get_control_mode(&mut controller)?;
            info!(
                "XL330Dynamixel {} {} initialized, operating mode: {} (asked: {})",
                serial_port_name, id, rmode[0], mode
            );

            motor_toolbox_rs::MotorsController::set_torque_limit(&mut controller, [current_limit])?;
            let tlimit = motor_toolbox_rs::MotorsController::get_torque_limit(&mut controller)?;
            info!(
                "XL330Dynamixel {} {} initialized, torque_limit: {} (asked: {})",
                serial_port_name, id, tlimit[0], current_limit
            );
        } else {
            warn!(
                "XL330Dynamixel {} {} torque is off, cannot set control mode",
                serial_port_name, id
            );
            warn!(
                "XL330Dynamixel {} {} torque is off, cannot set torque_limit",
                serial_port_name, id
            );
        }

        motor_toolbox_rs::MotorsController::set_target_torque(&mut controller, [current_limit])?;
        let ttorque = motor_toolbox_rs::MotorsController::get_target_torque(&mut controller)?;
        info!(
            "XL330Dynamixel {} {} initialized, target_torque: {}",
            serial_port_name, id, ttorque[0]
        );

        Ok(controller)
    }

    pub fn with_config(config: DynamixelConfigInfo) -> Result<Self> {
        XL330Dynamixel::new(
            config.serial_port_name.as_str(),
            config.id,
            config.mode,
            config.current_limit,
        )
    }
}

impl MotorsController<1> for XL330Dynamixel {
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

impl RawMotorsIO<1> for XL330Dynamixel {
    /// Checks if the torque is on.
    fn is_torque_on(&mut self) -> Result<[bool; 1]> {
        self.torque_on
            .entry(self.id)
            .or_try_insert_with(|_| {
                xl330::read_torque_enable(&self.io, self.serial_port.as_mut(), self.id)
                    .map(|torque| torque != 0)
            })
            .map(|x| [x])
    }
    fn set_torque(&mut self, torque: [bool; 1]) -> Result<()> {
        let current_torque = RawMotorsIO::is_torque_on(self)?;

        if torque != current_torque {
            xl330::write_torque_enable(
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
        Ok([
            xl330::conv::dxl_pos_to_radians(xl330::read_present_position(
                &self.io,
                self.serial_port.as_mut(),
                self.id,
            )?) as f64,
        ])
    }

    // it is in fact the current in A
    fn get_current_torque(&mut self) -> Result<[f64; 1]> {
        Ok([xl330::conv::dxl_current_to_ma(xl330::read_present_current(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
        )? as i16) as f64
            / 1000.0])
    }

    // rad/s
    fn get_current_velocity(&mut self) -> Result<[f64; 1]> {
        Ok([xl330::conv::dxl_vel_to_rpm(xl330::read_present_velocity(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
        )?) as f64
            * 0.10472])
    }

    fn get_target_position(&mut self) -> Result<[f64; 1]> {
        self.target_position
            .entry(self.id)
            .or_try_insert_with(|_| {
                Ok(xl330::conv::dxl_pos_to_radians(xl330::read_goal_position(
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
            xl330::write_goal_position(
                &self.io,
                self.serial_port.as_mut(),
                self.id,
                xl330::conv::radians_to_dxl_pos(target_position[0] as f32),
            )?;

            self.target_position.insert(self.id, target_position[0]);
        }

        Ok(())
    }

    //rad/s
    fn get_velocity_limit(&mut self) -> Result<[f64; 1]> {
        self.velocity_limit
            .entry(self.id)
            .or_try_insert_with(|_| {
                Ok(xl330::conv::dxl_vel_to_rpm(xl330::read_velocity_limit(
                    &self.io,
                    self.serial_port.as_mut(),
                    self.id,
                )? as i32) as f64
                    * 0.10472)
            })
            .map(|x| [x])
    }

    fn set_velocity_limit(&mut self, velocity_limit: [f64; 1]) -> Result<()> {
        let current_velocity_limit = RawMotorsIO::get_velocity_limit(self)?;

        if current_velocity_limit != velocity_limit {
            xl330::write_velocity_limit(
                &self.io,
                self.serial_port.as_mut(),
                self.id,
                (xl330::conv::rpm_to_dxl_vel(velocity_limit[0] as f32) as f32 / 0.10472) as u32,
            )?;

            self.velocity_limit.insert(self.id, velocity_limit[0]);
        }

        Ok(())
    }

    //in A (ok it is not really in Nm)
    fn get_torque_limit(&mut self) -> Result<[f64; 1]> {
        self.torque_limit
            .entry(self.id)
            .or_try_insert_with(|_| {
                Ok((xl330::conv::dxl_current_to_ma(xl330::read_torque_limit(
                    &self.io,
                    self.serial_port.as_mut(),
                    self.id,
                )? as i16) as f32
                    / 1000.0) as f64)
            })
            .map(|x| [x])
    }

    //in A (ok it is not really in Nm)
    fn set_torque_limit(&mut self, torque_limit: [f64; 1]) -> Result<()> {
        let current_torque_limit = RawMotorsIO::get_torque_limit(self)?;

        if current_torque_limit != torque_limit {
            xl330::write_current_limit(
                &self.io,
                self.serial_port.as_mut(),
                self.id,
                xl330::conv::ma_to_dxl_current((torque_limit[0] * 1000.0) as f32) as u16,
            )?;

            self.torque_limit.insert(self.id, torque_limit[0]);
        }

        Ok(())
    }

    /// Set the target current (A)
    fn set_target_torque(&mut self, target: [f64; 1]) -> Result<()> {
        xl330::write_goal_current(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
            xl330::conv::ma_to_dxl_current((target[0] * 1000.0) as f32) as i16,
        )?;
        Ok(())
    }
    /// Get the target current (A)
    fn get_target_torque(&mut self) -> Result<[f64; 1]> {
        let cur = xl330::conv::dxl_current_to_ma(xl330::read_goal_current(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
        )? as i16)
            / 1000.0;
        Ok([cur as f64])
    }

    /// Set the target velocity (rad/s)
    fn set_target_velocity(&mut self, target: [f64; 1]) -> Result<()> {
        xl330::write_goal_velocity(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
            xl330::conv::rpm_to_dxl_vel((target[0] * 9.5492999999998) as f32) as i32,
        )?;
        Ok(())
    }
    /// Get the target velocity (rad/s)
    fn get_target_velocity(&mut self) -> Result<[f64; 1]> {
        let cur = xl330::conv::dxl_vel_to_rpm(xl330::read_goal_velocity(
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
                Ok((xl330::read_operating_mode(&self.io, self.serial_port.as_mut(), self.id)?))
            })
            .map(|x| [x])

        // Ok([xl330::read_operating_mode(
        //     &self.io,
        //     self.serial_port.as_mut(),
        //     self.id,
        // )?])
    }

    /// Get the temperature (°C)
    fn get_motors_temperature(&mut self) -> Result<[f64; 1]> {
        let temp = xl330::conv::dxl_to_temperature(xl330::read_present_temperature(
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
            xl330::write_operating_mode(&self.io, self.serial_port.as_mut(), self.id, mode[0])?;

            self.control_mode.insert(self.id, mode[0]);
        }

        // xl330::write_operating_mode(&self.io, self.serial_port.as_mut(), self.id, mode[0])?;
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

        let config: Result<crate::XL330Config, _> = serde_yaml::from_reader(f);
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

use motor_toolbox_rs::{FakeMotorsController, MotorsController, Result};
use serde::{Deserialize, Serialize};
mod background_controller;
mod controller;
use background_controller::BackgroundDynamixelController;
use controller::ForegroundDynamixelController;

mod xl330;
mod xm;

#[derive(Debug, Deserialize, Serialize)]
pub enum DxlModel {
    XL330,
    XM,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct DynamixelConfigInfo {
    pub serial_port_name: String,
    pub id: u8,
    pub model: DxlModel,
    pub mode: u8,
    pub current_limit: f64,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct DxlConfig {
    pub io: DxlIOConfig,
}

#[derive(Debug, Deserialize, Serialize)]
pub enum DxlIOConfig {
    DynamixelSerialIO(DynamixelConfigInfo),
    FakeIO(FakeConfig),
}

#[derive(Debug, Deserialize, Serialize)]
pub struct FakeConfig {}

pub struct DynamixelJoint {
    inner: Box<dyn MotorsController<1> + Send>,
}

pub enum Dynamixel2Joints {
    Fg(ForegroundDynamixelController),
    Bg(BackgroundDynamixelController),
}

#[derive(Debug, Deserialize, Serialize)]
pub struct JointsConfig {
    pub left_config: DxlConfig,
    pub right_config: DxlConfig,
    pub background: String,
}

impl DynamixelJoint {
    pub fn with_config(config: DxlConfig) -> Result<Self> {
        let inner: Box<dyn MotorsController<1> + Send> = match config.io {
            DxlIOConfig::DynamixelSerialIO(config) => match config.model {
                DxlModel::XL330 => Box::new(xl330::XL330Dynamixel::with_config(config)?),
                DxlModel::XM => Box::new(xm::XMDynamixel::with_config(config)?),
            },
            DxlIOConfig::FakeIO(_) => Box::new(FakeMotorsController::new()),
        };
        Ok(DynamixelJoint { inner: inner })
    }
    pub fn with_config_file(config_file: &str) -> Result<Self> {
        let f = std::fs::File::open(config_file)?;
        let config: DxlConfig = serde_yaml::from_reader(f)?;
        DynamixelJoint::with_config(config)
    }
}

impl DynamixelJoint {
    /// Check if the torque is ON or OFF
    pub fn is_torque_on(&mut self) -> Result<bool> {
        let torques = self.inner.is_torque_on()?;
        Ok(torques[0])
    }
    /// Enable the torque
    ///
    /// # Arguments
    /// * reset_target: if true, the target position will be reset to the current position
    pub fn enable_torque(&mut self, reset_target: bool) -> Result<()> {
        if !self.is_torque_on()? && reset_target {
            let thetas = self.inner.get_current_position()?;
            self.inner.set_target_position(thetas)?;
        }
        self.inner.set_torque([true])
    }

    /// turn on/off the torque. The second bool in the tuple is the "reset target position argument"
    pub fn set_torque(&mut self, torques: (bool, bool)) -> Result<()> {
        if torques.0 && torques.1 {
            let thetas = self.inner.get_current_position()?;
            self.inner.set_target_position(thetas)?;
        }
        self.inner.set_torque([torques.0])
    }

    /// Disable the torque
    pub fn disable_torque(&mut self) -> Result<()> {
        self.inner.set_torque([false])
    }

    /// Get the present position (in rads)
    pub fn get_current_orientation(&mut self) -> Result<f64> {
        let pos = self.inner.get_current_position()?;
        Ok(pos[0])
    }

    /// Get the present velocity (in rads)
    pub fn get_current_velocity(&mut self) -> Result<f64> {
        let vel = self.inner.get_current_velocity()?;
        Ok(vel[0] * 0.10472)
    }

    /// Get the present current (in A)
    pub fn get_current_torque(&mut self) -> Result<f64> {
        let cur = self.inner.get_current_torque()?;
        Ok(cur[0] * 0.001)
    }

    /// Set the target torque (in A)
    pub fn set_target_torque(&mut self, current: f64) -> Result<()> {
        self.inner.set_target_torque([current])
    }

    /// Get the target torque (in A)
    pub fn get_target_torque(&mut self) -> Result<f64> {
        let torque = self.inner.get_target_torque()?;
        Ok(torque[0])
    }

    /// Set the target velocity (in rad/s)
    pub fn set_target_velocity(&mut self, velocity: f64) -> Result<()> {
        self.inner.set_target_velocity([velocity])
    }

    /// Get the target velocity (in rad/s)
    pub fn get_target_velocity(&mut self) -> Result<f64> {
        let vel = self.inner.get_target_velocity()?;
        Ok(vel[0])
    }

    /// Set the control mode (position, velocity, torque based position...)
    pub fn set_control_mode(&mut self, mode: u8) -> Result<()> {
        self.inner.set_control_mode([mode])
    }

    /// Get the control mode
    pub fn get_control_mode(&mut self) -> Result<u8> {
        let mode = self.inner.get_control_mode()?;
        Ok(mode[0])
    }

    /// Get the target position (in rads)
    pub fn get_target_orientation(&mut self) -> Result<f64> {
        let pos = self.inner.get_target_position()?;
        Ok(pos[0])
    }
    /// Set the target position (in rads)
    pub fn set_target_orientation(&mut self, target: f64) -> Result<()> {
        self.inner.set_target_position([target])
    }

    /// Get the velocity limit of each raw motor (in radians/s)
    pub fn get_raw_motors_velocity_limit(&mut self) -> Result<f64> {
        let vel = self.inner.get_velocity_limit()?;
        Ok(vel[0])
    }
    /// Set the velocity limit of each raw motor (in radians/s)

    pub fn set_raw_motors_velocity_limit(&mut self, velocity_limit: f64) -> Result<()> {
        self.inner.set_velocity_limit([velocity_limit])
    }
    /// Get the torque limit of each raw motor (in Nm)

    pub fn get_raw_motors_torque_limit(&mut self) -> Result<f64> {
        let torque = self.inner.get_torque_limit()?;
        Ok(torque[0])
    }
    /// Set the torque limit of each raw motor (in Nm)
    pub fn set_raw_motors_torque_limit(&mut self, torque_limit: f64) -> Result<()> {
        self.inner.set_torque_limit([torque_limit])
    }

    fn set_target_position_fb(&mut self, pos: f64) -> Result<f64> {
        let pos = self.inner.set_target_position_fb([pos])?;
        Ok(pos[0])
    }
    fn get_axis_sensors(&mut self) -> Result<f64> {
        let axis = self.inner.get_axis_sensors()?;
        Ok(axis[0])
    }

    pub fn get_motors_temperature(&mut self) -> Result<f64> {
        let temp = self.inner.get_motors_temperature()?;
        Ok(temp[0])
    }

    fn get_board_state(&mut self) -> Result<u8> {
        let state = self.inner.get_board_state()?;
        Ok(state)
    }
    fn set_board_state(&mut self, state: u8) -> Result<()> {
        self.inner.set_board_state(state)
    }
}

impl Dynamixel2Joints {
    pub fn with_config(config: JointsConfig) -> Result<Dynamixel2Joints> {
        if config.background == "true" {
            Ok(Dynamixel2Joints::Bg(
                BackgroundDynamixelController::with_config(
                    config.left_config,
                    config.right_config,
                )?,
            ))
        } else {
            Ok(Dynamixel2Joints::Fg(
                ForegroundDynamixelController::with_config(
                    config.left_config,
                    config.right_config,
                )?,
            ))
        }
    }
    pub fn with_config_file(config_file: &str) -> Result<Dynamixel2Joints> {
        let f = std::fs::File::open(config_file)?;
        let config: JointsConfig = serde_yaml::from_reader(f)?;
        Dynamixel2Joints::with_config(config)
    }

    /// turn on/off the torque. The second bool in the tuple is the "reset target position argument"
    pub fn set_torque(&mut self, torques: [(bool, bool); 2]) -> Result<()> {
        match self {
            Dynamixel2Joints::Bg(c) => c.set_torque(torques),
            Dynamixel2Joints::Fg(c) => c.set_torque(torques),
        }
    }

    pub fn is_torque_on(&mut self) -> Result<[Option<bool>; 2]> {
        match self {
            Dynamixel2Joints::Bg(c) => c.is_torque_on(),
            Dynamixel2Joints::Fg(c) => c.is_torque_on(),
        }
    }

    pub fn get_current_position(&mut self) -> Result<[Option<f64>; 2]> {
        match self {
            Dynamixel2Joints::Fg(c) => c.get_current_position(),
            Dynamixel2Joints::Bg(c) => c.get_current_position(),
        }
    }

    pub fn set_target_position(&mut self, target: [f64; 2]) -> Result<()> {
        match self {
            Dynamixel2Joints::Fg(c) => c.set_target_position(target),
            Dynamixel2Joints::Bg(c) => c.set_target_position(target),
        }
    }

    pub fn get_target_position(&mut self) -> Result<[Option<f64>; 2]> {
        match self {
            Dynamixel2Joints::Fg(c) => c.get_target_position(),
            Dynamixel2Joints::Bg(c) => c.get_target_position(),
        }
    }

    pub fn get_current_velocity(&mut self) -> Result<[Option<f64>; 2]> {
        match self {
            Dynamixel2Joints::Fg(c) => c.get_current_velocity(),
            Dynamixel2Joints::Bg(c) => c.get_current_velocity(),
        }
    }

    pub fn get_current_torque(&mut self) -> Result<[Option<f64>; 2]> {
        match self {
            Dynamixel2Joints::Fg(c) => c.get_current_torque(),
            Dynamixel2Joints::Bg(c) => c.get_current_torque(),
        }
    }

    pub fn set_target_torque(&mut self, target: [f64; 2]) -> Result<()> {
        match self {
            Dynamixel2Joints::Fg(c) => c.set_target_torque(target),
            Dynamixel2Joints::Bg(c) => c.set_target_torque(target),
        }
    }

    pub fn get_target_torque(&mut self) -> Result<[Option<f64>; 2]> {
        match self {
            Dynamixel2Joints::Fg(c) => c.get_target_torque(),
            Dynamixel2Joints::Bg(c) => c.get_target_torque(),
        }
    }

    pub fn set_control_mode(&mut self, mode: [u8; 2]) -> Result<()> {
        match self {
            Dynamixel2Joints::Fg(c) => c.set_control_mode(mode),
            Dynamixel2Joints::Bg(c) => c.set_control_mode(mode),
        }
    }
    pub fn get_control_mode(&mut self) -> Result<[Option<u8>; 2]> {
        match self {
            Dynamixel2Joints::Fg(c) => c.get_control_mode(),
            Dynamixel2Joints::Bg(c) => c.get_control_mode(),
        }
    }

    pub fn get_raw_motors_torque_limit(&mut self) -> Result<[Option<f64>; 2]> {
        match self {
            Dynamixel2Joints::Fg(c) => c.get_raw_motors_torque_limit(),
            Dynamixel2Joints::Bg(c) => c.get_raw_motors_torque_limit(),
        }
    }

    pub fn get_raw_motors_velocity_limit(&mut self) -> Result<[Option<f64>; 2]> {
        match self {
            Dynamixel2Joints::Fg(c) => c.get_raw_motors_velocity_limit(),
            Dynamixel2Joints::Bg(c) => c.get_raw_motors_velocity_limit(),
        }
    }

    pub fn set_raw_motors_torque_limit(&mut self, limit: [f64; 2]) -> Result<()> {
        match self {
            Dynamixel2Joints::Fg(c) => c.set_raw_motors_torque_limit(limit),
            Dynamixel2Joints::Bg(c) => c.set_raw_motors_torque_limit(limit),
        }
    }

    pub fn set_raw_motors_velocity_limit(&mut self, limit: [f64; 2]) -> Result<()> {
        match self {
            Dynamixel2Joints::Fg(c) => c.set_raw_motors_velocity_limit(limit),
            Dynamixel2Joints::Bg(c) => c.set_raw_motors_velocity_limit(limit),
        }
    }

    pub fn get_motors_temperature(&mut self) -> Result<[Option<f64>; 2]> {
        match self {
            Dynamixel2Joints::Fg(c) => c.get_motors_temperature(),
            Dynamixel2Joints::Bg(c) => c.get_motors_temperature(),
        }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn parse_fake_config_file() {
        let f = std::fs::File::open("./config/fake.yaml").unwrap();

        let config: Result<crate::AntennaConfig, _> = serde_yaml::from_reader(f);
        assert!(config.is_ok());

        let config = config.unwrap();

        if let crate::AntennaIOConfig::FakeIO(_) = config.io {
        } else {
            panic!("Wrong config type");
        }
    }
}

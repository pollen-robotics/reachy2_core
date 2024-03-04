use dynamixel::{GripperDynamixel, GripperDynamixelConfig};
use motor_toolbox_rs::{FakeMotorsController, MotorsController, Result};
use serde::{Deserialize, Serialize};

mod dynamixel;

#[derive(Debug, Deserialize, Serialize)]
pub struct GripperConfig {
    pub io: GripperIOConfig,
}

#[derive(Debug, Deserialize, Serialize)]
pub enum GripperIOConfig {
    DynamixelSerialIO(GripperDynamixelConfig),
    FakeIO(GripperFakeConfig),
}

#[derive(Debug, Deserialize, Serialize)]
pub struct GripperFakeConfig {}

pub struct Gripper {
    inner: Box<dyn MotorsController<1> + Send>,
}

impl Gripper {
    pub fn with_config(config: GripperConfig) -> Result<Self> {
        let inner: Box<dyn MotorsController<1> + Send> = match config.io {
            GripperIOConfig::DynamixelSerialIO(config) => {
                Box::new(GripperDynamixel::with_config(config)?)
            }
            GripperIOConfig::FakeIO(_) => Box::new(FakeMotorsController::new()),
        };
        Ok(Gripper { inner })
    }
    pub fn with_config_file(config_file: &str) -> Result<Self> {
        let f = std::fs::File::open(config_file)?;
        let config: GripperConfig = serde_yaml::from_reader(f)?;
        Gripper::with_config(config)
    }
}

impl Gripper {
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
    /// Disable the torque
    pub fn disable_torque(&mut self) -> Result<()> {
        self.inner.set_torque([false])
    }

    /// Get the current position (in rads)
    pub fn get_current_orientation(&mut self) -> Result<f64> {
        let pos = self.inner.get_current_position()?;
        Ok(pos[0])
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

    /// Get the velocity limit of each raw motor [motor_a, motor_b] (in radians/s)
    /// caution: this is the raw value used by the motors used inside the actuator, not a limit in orbita2d orientation!
    pub fn get_raw_motors_velocity_limit(&mut self) -> Result<f64> {
        let vel = self.inner.get_velocity_limit()?;
        Ok(vel[0])
    }
    /// Set the velocity limit of each raw motor [motor_a, motor_b] (in radians/s)
    /// caution: this is the raw value used by the motors used inside the actuator, not a limit in orbita2d orientation!
    pub fn set_raw_motors_velocity_limit(&mut self, velocity_limit: f64) -> Result<()> {
        self.inner.set_velocity_limit([velocity_limit])
    }
    /// Get the torque limit of each raw motor [motor_a, motor_b] (in Nm)
    /// caution: this is the raw value used by the motors used inside the actuator, not a limit in orbita2d orientation!
    pub fn get_raw_motors_torque_limit(&mut self) -> Result<f64> {
        let torque = self.inner.get_torque_limit()?;
        Ok(torque[0])
    }
    /// Set the torque limit of each raw motor [motor_a, motor_b] (in Nm)
    /// caution: this is the raw value used by the motors used inside the actuator, not a limit in orbita2d orientation!
    pub fn set_raw_motors_torque_limit(&mut self, torque_limit: f64) -> Result<()> {
        self.inner.set_torque_limit([torque_limit])
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn parse_fake_config_file() {
        let f = std::fs::File::open("./config/fake.yaml").unwrap();

        let config: Result<crate::GripperConfig, _> = serde_yaml::from_reader(f);
        assert!(config.is_ok());

        let config = config.unwrap();

        if let crate::GripperIOConfig::FakeIO(_) = config.io {
        } else {
            panic!("Wrong config type");
        }
    }
}

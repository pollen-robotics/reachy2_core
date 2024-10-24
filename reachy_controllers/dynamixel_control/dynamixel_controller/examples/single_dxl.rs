use dynamixel_controller::DynamixelJoint;
use std::{error::Error, thread, time::Duration};

fn main() -> Result<(), Box<dyn Error>> {
    let mut dxljoint = DynamixelJoint::with_config_file("config/single_xl330.yaml")?;
    println!("dxl torque_on: {:?}", dxljoint.is_torque_on()?);
    dxljoint.enable_torque(true)?;
    println!("dxl torque_on: {:?}", dxljoint.is_torque_on()?);

    thread::sleep(Duration::from_millis(100));
    dxljoint.set_target_orientation(0.0)?;
    thread::sleep(Duration::from_millis(1000));
    println!("dxl pos: {:?}", dxljoint.get_current_orientation()?);
    dxljoint.set_target_orientation(20.0_f64.to_radians())?;
    thread::sleep(Duration::from_millis(1000));
    println!("dxl pos: {:?}", dxljoint.get_current_orientation()?);
    dxljoint.set_target_orientation(0.0)?;
    thread::sleep(Duration::from_millis(1000));
    println!("dxl pos: {:?}", dxljoint.get_current_orientation()?);
    dxljoint.disable_torque()?;
    println!("dxl torque_on: {:?}", dxljoint.is_torque_on()?);

    Ok(())
}

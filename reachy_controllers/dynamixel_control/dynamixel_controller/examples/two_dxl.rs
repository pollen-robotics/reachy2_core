use dynamixel_controller::Dynamixel2Joints;
use std::{error::Error, thread, time::Duration};

fn main() -> Result<(), Box<dyn Error>> {
    let mut dxljoint = Dynamixel2Joints::with_config_file("config/two_xl330.yaml")?;
    println!("dxl torque_on: {:?}", dxljoint.is_torque_on()?);
    dxljoint.set_torque([(true, true), (true, true)])?;
    thread::sleep(Duration::from_millis(100));
    println!("dxl torque_on: {:?}", dxljoint.is_torque_on()?);

    dxljoint.set_target_position([0.0, 0.0])?;
    thread::sleep(Duration::from_millis(1000));
    let pos = dxljoint.get_current_position()?;
    println!("dxl pos: left {:?} right {:?}", pos[0], pos[1]);

    dxljoint.set_target_position([20.0_f64.to_radians(), 20.0_f64.to_radians()])?;
    thread::sleep(Duration::from_millis(1000));
    let pos = dxljoint.get_current_position()?;
    println!("dxl pos: left {:?} right {:?}", pos[0], pos[1]);

    dxljoint.set_target_position([0.0, 0.0])?;
    thread::sleep(Duration::from_millis(1000));
    let pos = dxljoint.get_current_position()?;
    println!("dxl pos: left {:?} right {:?}", pos[0], pos[1]);

    dxljoint.set_torque([(false, false), (false, false)])?;
    thread::sleep(Duration::from_millis(100));
    println!("dxl torque_on: {:?}", dxljoint.is_torque_on()?);

    Ok(())
}

#include "smart_gripper_dynamixel_system_hwi/smart_gripper_dynamixel_system_hwi.hpp"

#include "smart_gripper_dynamixel_hwi.h"

#include <cmath>
#include <string>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"

std::vector<float> parse_string_as_vec(std::string s) {
  std::string delimiter = ",";

  std::vector<float> v;

  size_t pos = 0;
  std::string token;
  while ((pos = s.find(delimiter)) != std::string::npos) {
    token = s.substr(0, pos);
    v.push_back(std::stof(token));
    s.erase(0, pos + delimiter.length());
  }
  v.push_back(std::stof(s));

  return v;
}


namespace smart_gripper_dynamixel_system_hwi
{
CallbackReturn
SmartGripperDynamixelSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  long unsigned int nb_joints_expected = 1;
  if (info.joints.size() != nb_joints_expected)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("SmartGripperDynamixelSystem"),
      "Incorrect number of joints, expected %ld, got \"%s\"", nb_joints_expected,
       std::to_string(info.joints.size()).c_str()
    );
    return CallbackReturn::ERROR;
  }


  const char *serial_port;
  uint8_t id;

  for (auto const& params : info.hardware_parameters)
  {
    if (params.first == "gripper_port") {
      serial_port = params.second.c_str();
    }
    else if (params.first == "gripper_id") {
      id = (uint8_t)std::stoi(params.second.c_str());
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("SmartGripperDynamixelSystem"),
    "Trying to connect on serial port \"%s\"",
    serial_port
  );

  this->uid = smart_gripper_dynamixel_hwi_init(
    serial_port,
    id
  );

  clock_ = rclcpp::Clock();
  RCLCPP_INFO(
    rclcpp::get_logger("SmartGripperDynamixelSystem"),
    "System \"%s\" init!", info_.name.c_str()
  );

  return CallbackReturn::SUCCESS;
}

CallbackReturn
SmartGripperDynamixelSystem::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set some default values

  for (int i=0; i < 1; i++) {
    hw_states_position_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_states_velocity_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_states_effort_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_states_temperature_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_states_torque_limit_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_states_speed_limit_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_states_torque_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_states_p_gain_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_states_i_gain_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_states_d_gain_[i] = std::numeric_limits<double>::quiet_NaN();
  }

  // TODO: make sure there is no error here!
  smart_gripper_dynamixel_hwi_get_goal_position(this->uid, hw_commands_position_);
//   smart_gripper_dynamixel_hwi_get_max_speed(this->uid, hw_commands_speed_limit_);
  // smart_gripper_dynamixel_hwi_get_max_torque(this->uid, hw_commands_torque_limit_);
  smart_gripper_dynamixel_hwi_is_torque_on(this->uid, hw_commands_torque_);
//   smart_gripper_dynamixel_hwi_get_pid(this->uid, hw_commands_p_gain_, hw_commands_i_gain_, hw_commands_d_gain_);

  last_timestamp_ = clock_.now();

  RCLCPP_INFO(
    rclcpp::get_logger("SmartGripperDynamixelSystem"),
    "System \"%s\" successfully started!", info_.name.c_str()
  );
  return CallbackReturn::SUCCESS;
}

CallbackReturn
SmartGripperDynamixelSystem::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("SmartGripperDynamixelSystem"),
    "System \"%s\" successfully deactivated!", info_.name.c_str()
  );
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
SmartGripperDynamixelSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (std::size_t i = 0; i < 1; i++)
  {
    auto joint = info_.joints[i];

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, hardware_interface::HW_IF_EFFORT, &hw_states_effort_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, "temperature", &hw_states_temperature_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, "torque_limit", &hw_states_torque_limit_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, "speed_limit", &hw_states_speed_limit_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, "torque", &hw_states_torque_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, "p_gain", &hw_states_p_gain_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, "i_gain", &hw_states_i_gain_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, "d_gain", &hw_states_d_gain_[i]));

    RCLCPP_INFO(
      rclcpp::get_logger("SmartGripperDynamixelSystem"),
      "export state interface (%s) \"%s\"!", info_.name.c_str(), joint.name.c_str()
      );
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
SmartGripperDynamixelSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (std::size_t i = 0; i < 1; i++)
  {
    auto joint = info_.joints[i];

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint.name, hardware_interface::HW_IF_POSITION, &hw_commands_position_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint.name, "speed_limit", &hw_commands_speed_limit_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint.name, "torque_limit", &hw_commands_torque_limit_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint.name, "torque", &hw_commands_torque_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint.name, "p_gain", &hw_commands_p_gain_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint.name, "i_gain", &hw_commands_i_gain_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint.name, "d_gain", &hw_commands_d_gain_[i]));

    RCLCPP_INFO(
      rclcpp::get_logger("SmartGripperDynamixelSystem"),
      "export command interface (%s) \"%s\"!", info_.name.c_str(), joint.name.c_str()
      );
  }

  return command_interfaces;
}

hardware_interface::return_type
SmartGripperDynamixelSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  current_timestamp = clock_.now();
  rclcpp::Duration duration = current_timestamp - last_timestamp_;
  last_timestamp_ = current_timestamp;


  if (smart_gripper_dynamixel_hwi_get_position(this->uid, hw_states_position_) != 0) {
    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("SmartGripperDynamixelSystem"),
      clock_,
      LOG_THROTTLE_DURATION,
      "(%s) READ POSITION ERROR!", info_.name.c_str()
    );
  }



    // if (smart_gripper_dynamixel_hwi_get_orientation_velocity_load(this->uid, hw_states_position_, hw_states_velocity_, hw_states_effort_) != 0) {
  //   RCLCPP_INFO_THROTTLE(
  //     rclcpp::get_logger("SmartGripperDynamixelSystem"),
  //     clock_,
  //     LOG_THROTTLE_DURATION,
  //     "(%s) READ ORIENTATION/VELOCITY/EFFORT ERROR!", info_.name.c_str()
  //   );
  // }

  // if (smart_gripper_dynamixel_hwi_get_temperature(this->uid, hw_states_temperature_) != 0) {
  //     RCLCPP_INFO_THROTTLE(
  //       rclcpp::get_logger("SmartGripperDynamixelSystem"),
  //       clock_,
  //       LOG_THROTTLE_DURATION,
  //       "(%s) READ TEMPERATURE ERROR!", info_.name.c_str()
  //     );
  // }

  // if (smart_gripper_dynamixel_hwi_get_max_torque(this->uid, hw_states_torque_limit_) != 0) {
  //   RCLCPP_INFO_THROTTLE(
  //     rclcpp::get_logger("SmartGripperDynamixelSystem"),
  //     clock_,
  //     LOG_THROTTLE_DURATION,
  //     "(%s) READ TORQUE LIMIT ERROR!", info_.name.c_str()
  //   );
  // }
//   if (smart_gripper_dynamixel_hwi_get_max_speed(this->uid, hw_states_speed_limit_) != 0) {
//     RCLCPP_INFO_THROTTLE(
//       rclcpp::get_logger("SmartGripperDynamixelSystem"),
//       clock_,
//       LOG_THROTTLE_DURATION,
//       "(%s) READ SPEED LIMIT ERROR!", info_.name.c_str()
//     );
//   }
  if (smart_gripper_dynamixel_hwi_is_torque_on(this->uid, hw_states_torque_) != 0) {
      RCLCPP_INFO_THROTTLE(
        rclcpp::get_logger("SmartGripperDynamixelSystem"),
        clock_,
        LOG_THROTTLE_DURATION,
        "(%s) READ TORQUE (ON/OFF) ERROR!", info_.name.c_str()
      );
  }
//   if (smart_gripper_dynamixel_hwi_get_pid(this->uid, hw_states_p_gain_, hw_states_i_gain_, hw_states_d_gain_) != 0) {
//     RCLCPP_INFO_THROTTLE(
//       rclcpp::get_logger("SmartGripperDynamixelSystem"),
//       clock_,
//       LOG_THROTTLE_DURATION,
//       "(%s) READ PID ERROR!", info_.name.c_str()
//     );
//   }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
SmartGripperDynamixelSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (smart_gripper_dynamixel_hwi_set_target_position(
    this->uid, 
    *hw_commands_position_
  ) != 0) {
    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("SmartGripperDynamixelSystem"),
      clock_,
      LOG_THROTTLE_DURATION,
      "(%s) WRITE POSITION LIMIT ERROR!", info_.name.c_str()
    );
  }

  // if (smart_gripper_dynamixel_hwi_set_target_orientation_max_speed_max_torque(
  //   this->uid, 
  //   hw_commands_position_,
  //   hw_commands_speed_limit_,
  //   hw_commands_torque_limit_
  // ) != 0) {
  //   RCLCPP_INFO_THROTTLE(
  //     rclcpp::get_logger("SmartGripperDynamixelSystem"),
  //     clock_,
  //     LOG_THROTTLE_DURATION,
  //     "(%s) WRITE ORIENTATION/SPEED LIMIT/TORQUE LIMIT ERROR!", info_.name.c_str()
  //   );
  // }
  
  
  if (smart_gripper_dynamixel_hwi_set_torque(this->uid, *hw_commands_torque_) != 0) {
    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("SmartGripperDynamixelSystem"),
      clock_,
      LOG_THROTTLE_DURATION,
      "(%s) WRITE TORQUE ERROR!", info_.name.c_str()
    );
  }
//   if (smart_gripper_dynamixel_hwi_set_pid(this->uid, hw_commands_p_gain_, hw_commands_i_gain_, hw_commands_d_gain_) != 0) {
//     RCLCPP_INFO_THROTTLE(
//       rclcpp::get_logger("SmartGripperDynamixelSystem"),
//       clock_,
//       LOG_THROTTLE_DURATION,
//       "(%s) WRITE PID ERROR!", info_.name.c_str()
//     );
//   }

  return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  smart_gripper_dynamixel_system_hwi::SmartGripperDynamixelSystem,
  hardware_interface::SystemInterface)

#include "dynamixel_system_hwi/dynamixel_system_hwi.hpp"

#include "dynamixel_controller.h"

#include <cmath>
#include <string>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"



namespace dynamixel_system_hwi
{
CallbackReturn
DynamixelSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  long unsigned int nb_joints_expected = 2; //TODO Check
  if (info.joints.size() != nb_joints_expected)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("DynamixelSystem"),
      "Incorrect number of joints, expected %ld, got \"%s\"", nb_joints_expected,
       std::to_string(info.joints.size()).c_str()
    );
    return CallbackReturn::ERROR;
  }

  /*
  //TODO GPIO
  const int nb_gpios_expected = 8; //for each motor: ctrl mode, torque_limit, velocity_limit, temperature

  if (info.gpios.size() != nb_gpios_expected)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("DynamixelSystem"),
      "Got \"%s\" gpios", std::to_string(info.gpios.size()).c_str());
    return CallbackReturn::ERROR;
  }
*/

  const char *config_file = "";


  for (auto const& params : info.hardware_parameters)
  {
    if (params.first == "dxl_config_file") {
      config_file = params.second.c_str();
    }
    else{
      RCLCPP_ERROR(
        rclcpp::get_logger("DynamixelSystem"),
        "Unknown parameter \"%s\"", params.first.c_str()
      );
      return CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("DynamixelSystem"),
    "Trying to connect to dxl (2 motors) with config \"%s\"", config_file
  );

  int err = dynamixel_2joints_from_config(&(this->uid),config_file);

  if (err != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("DynamixelSystem"),
      "Failed to init dxl_controller with config file: \"%s\"",
      config_file
    );
    return CallbackReturn::ERROR;
  }

  clock_ = rclcpp::Clock();
  RCLCPP_INFO(
    rclcpp::get_logger("DynamixelSystem"),
    "System \"%s\" init!", info_.name.c_str()
  );

  return CallbackReturn::SUCCESS;
}

CallbackReturn
DynamixelSystem::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set some default values

  for (int i=0; i < 2; i++) {
    hw_states_position_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_states_velocity_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_states_effort_[i] = std::numeric_limits<double>::quiet_NaN();

    //GPIO
    hw_states_temperature_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_states_torque_limit_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_states_speed_limit_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_states_torque_[i] = std::numeric_limits<double>::quiet_NaN();

    //future stuffs
    hw_states_p_gain_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_states_i_gain_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_states_d_gain_[i] = std::numeric_limits<double>::quiet_NaN();
  }

  // TODO: make sure there is no error here!
  // dynamixel_2joints_get_target_position(this->uid, &hw_commands_position_);



  last_timestamp_ = clock_.now();

  RCLCPP_INFO(
    rclcpp::get_logger("DynamixelSystem"),
    "System \"%s\" successfully started!", info_.name.c_str()
  );
  return CallbackReturn::SUCCESS;
}

CallbackReturn
DynamixelSystem::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("DynamixelSystem"),
    "System \"%s\" successfully deactivated!", info_.name.c_str()
  );
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DynamixelSystem::export_state_interfaces()
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


    //TODO GPIO
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
      rclcpp::get_logger("DynamixelSystem"),
      "export state interface (%s) \"%s\"!", info_.name.c_str(), joint.name.c_str()
      );
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DynamixelSystem::export_command_interfaces()
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
      rclcpp::get_logger("DynamixelSystem"),
      "export command interface (%s) \"%s\"!", info_.name.c_str(), joint.name.c_str()
      );
  }

  return command_interfaces;
}

hardware_interface::return_type
DynamixelSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  current_timestamp = clock_.now();
  rclcpp::Duration duration = current_timestamp - last_timestamp_;
  last_timestamp_ = current_timestamp;


  if (dynamixel_2joints_get_current_position(this->uid, &hw_states_position_) != 0) {
    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("DynamixelSystem"),
      clock_,
      LOG_THROTTLE_DURATION,
      "(%s) READ POSITION ERROR!", info_.name.c_str()
    );
  }

  if (dynamixel_2joints_get_current_velocity(this->uid, &hw_states_velocity_) != 0) {
    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("DynamixelSystem"),
      clock_,
      LOG_THROTTLE_DURATION,
      "(%s) READ VELOCITY ERROR!", info_.name.c_str()
    );
  }

  if (dynamixel_2joints_get_current_torque(this->uid, &hw_states_effort_) != 0) {
    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("DynamixelSystem"),
      clock_,
      LOG_THROTTLE_DURATION,
      "(%s) READ TORQUE ERROR!", info_.name.c_str()
    );
  }

  if (dynamixel_2joints_get_raw_motors_torque_limit(this->uid, &hw_states_torque_limit_) != 0) {
    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("DynamixelSystem"),
      clock_,
      LOG_THROTTLE_DURATION,
      "(%s) READ TORQUE LIMIT ERROR!", info_.name.c_str()
    );
  }

  if (dynamixel_2joints_get_raw_motors_velocity_limit(this->uid, &hw_states_speed_limit_) != 0) {
    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("DynamixelSystem"),
      clock_,
      LOG_THROTTLE_DURATION,
      "(%s) READ SPEED LIMIT ERROR!", info_.name.c_str()
    );
  }


  if (dynamixel_2joints_get_motors_temperature(this->uid, &hw_states_temperature_) != 0) {
    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("DynamixelSystem"),
      clock_,
      LOG_THROTTLE_DURATION,
      "(%s) READ TEMPERATURE ERROR!", info_.name.c_str()
    );
  }

  bool torque_on[2] = {false,false};

  if (dynamixel_2joints_is_torque_on(this->uid, &torque_on) != 0) {
      RCLCPP_INFO_THROTTLE(
        rclcpp::get_logger("DynamixelSystem"),
        clock_,
        LOG_THROTTLE_DURATION,
        "(%s) READ TORQUE (ON/OFF) ERROR!", info_.name.c_str()
      );
  }
  else{
    for (int i=0; i < 2; i++) {
      hw_states_torque_[i] = torque_on[i] ? 1.0 : 0.0;
    }
  }



  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
DynamixelSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (dynamixel_2joints_set_target_position(
    this->uid,
    &hw_commands_position_
  ) != 0) {
    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("DynamixelSystem"),
      clock_,
      LOG_THROTTLE_DURATION,
      "(%s) WRITE POSITION LIMIT ERROR!", info_.name.c_str()
    );
  }

  // if (gripper_dynamixel_hwi_set_target_orientation_max_speed_max_torque(
  //   this->uid,
  //   hw_commands_position_,
  //   hw_commands_speed_limit_,
  //   hw_commands_torque_limit_
  // ) != 0) {
  //   RCLCPP_INFO_THROTTLE(
  //     rclcpp::get_logger("GripperDynamixelSystem"),
  //     clock_,
  //     LOG_THROTTLE_DURATION,
  //     "(%s) WRITE ORIENTATION/SPEED LIMIT/TORQUE LIMIT ERROR!", info_.name.c_str()
  //   );
  // }


  bool torque_on[2] = {hw_commands_torque_[0] != 0.0, hw_commands_torque_[1] != 0.0};

  if (dynamixel_2joints_set_torque(this->uid, &torque_on) != 0) {
    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("DynamixelSystem"),
      clock_,
      LOG_THROTTLE_DURATION,
      "(%s) WRITE TORQUE ERROR!", info_.name.c_str()
    );
  }

  return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dynamixel_system_hwi::DynamixelSystem,
  hardware_interface::SystemInterface)

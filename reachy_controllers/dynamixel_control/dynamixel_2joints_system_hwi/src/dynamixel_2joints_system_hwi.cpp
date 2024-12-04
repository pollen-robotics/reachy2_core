#include "dynamixel_2joints_system_hwi/dynamixel_2joints_system_hwi.hpp"

#include "dynamixel_controller.h"

#include <cmath>
#include <string>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"



namespace dynamixel_2joints_system_hwi
{
CallbackReturn
DynamixelSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  long unsigned int nb_joints_expected = 2;

   for (std::size_t i = 0; i < info_.joints.size(); i++)
   {
     auto joint = info_.joints[i];
    RCLCPP_INFO(
      rclcpp::get_logger("DynamixelSystem"),
      "Got joint: \"%s\" ", joint.name.c_str());
   }

  if (info.joints.size() != nb_joints_expected)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("DynamixelSystem"),
      "Incorrect number of joints, expected %ld, got \"%s\"", nb_joints_expected,
       std::to_string(info.joints.size()).c_str()
    );
    return CallbackReturn::ERROR;
  }


  //GPIO
  const int nb_gpios_expected = 4; //ie: antenna_left, antenna_right, antenna_raw_motor_left, antenna_raw_motor_right
  for (std::size_t i = 0; i < info_.gpios.size(); i++)
  {
    auto gpio = info_.gpios[i];
    RCLCPP_INFO(
      rclcpp::get_logger("DynamixelSystem"),
      "Got GPIO: \"%s\" ", gpio.name.c_str());

  }
  if (info.gpios.size() != nb_gpios_expected)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("DynamixelSystem"),
      "Got \"%s\" gpios", std::to_string(info.gpios.size()).c_str());
    return CallbackReturn::ERROR;
  }


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
    hw_states_current_[i] = std::numeric_limits<double>::quiet_NaN();


    //future stuffs
    hw_states_p_gain_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_states_i_gain_[i] = std::numeric_limits<double>::quiet_NaN();
    hw_states_d_gain_[i] = std::numeric_limits<double>::quiet_NaN();
  }

  // TODO: make sure there is no error here!
  // Initialize states and commands




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


  if (dynamixel_2joints_get_target_torque(this->uid, &hw_states_current_) != 0) {
    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("DynamixelSystem"),
      clock_,
      LOG_THROTTLE_DURATION,
      "(%s) READ TARGET CURRENT ERROR!", info_.name.c_str()
    );
  }


  uint8_t mode[2] = {255,255};

  if (dynamixel_2joints_get_control_mode(this->uid, &mode) != 0) {
    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("DynamixelSystem"),
      clock_,
      LOG_THROTTLE_DURATION,
      "(%s) READ CONTROL MODE ERROR!", info_.name.c_str()
    );
  }
  hw_states_mode_[0] = static_cast<double>(mode[0]);
  hw_states_mode_[1] = static_cast<double>(mode[1]);


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



  hw_commands_position_[0]=hw_states_position_[0];
  hw_commands_position_[1]=hw_states_position_[1];

  hw_commands_speed_limit_[0]=hw_states_speed_limit_[0];
  hw_commands_speed_limit_[1]=hw_states_speed_limit_[1];

  hw_commands_torque_limit_[0]=hw_states_torque_limit_[0];
  hw_commands_torque_limit_[1]=hw_states_torque_limit_[1];

  hw_commands_torque_[0]=hw_states_torque_[0];
  hw_commands_torque_[1]=hw_states_torque_[1];

  // hw_commands_p_gain_[2];
  // hw_commands_i_gain_[2];
  // hw_commands_d_gain_[2];
  hw_commands_mode_[0]=hw_states_mode_[0];
  hw_commands_mode_[1]=hw_states_mode_[1];

  hw_commands_current_[0]=hw_states_current_[0];
  hw_commands_current_[1]=hw_states_current_[1];




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

  for (std::size_t i = 0; i < 2; i++)
  {
    auto joint = info_.joints[i];

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.name, hardware_interface::HW_IF_EFFORT, &hw_states_effort_[i]));

    RCLCPP_INFO(
      rclcpp::get_logger("DynamixelSystem"),
      "export state interface (%s) \"%s\"!", info_.name.c_str(), joint.name.c_str()
      );
  }


  //GPIOS
      // motor index (not corresponding to the GPIO index)
  // Here we have 2 motors but 4 GPIOs (left/torque, right/torque, left_raw_motor/ and right_raw_motor/)
    size_t motor_index = 0;
    size_t raw_motor_index = 0;

    RCLCPP_INFO(
      rclcpp::get_logger("DynamixelSystem"),
      "export state interface (GPIO) (%s)", info_.name.c_str());

    // be careful GPIO index != motor or actuator index
    for (std::size_t i = 0; i < info_.gpios.size(); i++)
    {
      auto gpio = info_.gpios[i];

      if (gpio.name == (info_.name+"_left").c_str() || gpio.name == (info_.name+"_right").c_str()|| gpio.name.compare(std::string("l_hand"))==0  || gpio.name.compare(std::string("r_hand" ))==0) //for compatibility...
      {

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            gpio.name, "torque", &hw_states_torque_[motor_index]));
        motor_index++;

        // RCLCPP_INFO(
        //     rclcpp::get_logger("DynamixelSystem"),
        //     "export state interface (GPIO torque) (%s) \"%s\"!", info_.name.c_str(), gpio.name.c_str());

          RCLCPP_INFO(
            rclcpp::get_logger("DynamixelSystem"), "Added %s/%s",
            info_.gpios.at(i).name.c_str(), "torque");

      }
      else if (gpio.name.find("raw_motor") != std::string::npos)
      {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            gpio.name, "temperature", &hw_states_temperature_[raw_motor_index]));
        RCLCPP_INFO(
            rclcpp::get_logger("DynamixelSystem"), "Added %s/%s",
            info_.gpios.at(i).name.c_str(), "temperature");

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            gpio.name, "torque_limit", &hw_states_torque_limit_[raw_motor_index]));

        RCLCPP_INFO(
            rclcpp::get_logger("DynamixelSystem"), "Added %s/%s",
            info_.gpios.at(i).name.c_str(), "torque_limit");

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            gpio.name, "speed_limit", &hw_states_speed_limit_[raw_motor_index]));
        RCLCPP_INFO(
            rclcpp::get_logger("DynamixelSystem"), "Added %s/%s",
            info_.gpios.at(i).name.c_str(), "speed_limit");

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            gpio.name, "p_gain", &hw_states_p_gain_[raw_motor_index]));
        RCLCPP_INFO(
            rclcpp::get_logger("DynamixelSystem"), "Added %s/%s",
            info_.gpios.at(i).name.c_str(), "p_gain");

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            gpio.name, "i_gain", &hw_states_i_gain_[raw_motor_index]));
        RCLCPP_INFO(
            rclcpp::get_logger("DynamixelSystem"), "Added %s/%s",
            info_.gpios.at(i).name.c_str(), "i_gain");

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            gpio.name, "d_gain", &hw_states_d_gain_[raw_motor_index]));
        RCLCPP_INFO(
            rclcpp::get_logger("DynamixelSystem"), "Added %s/%s",
            info_.gpios.at(i).name.c_str(), "d_gain");

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            gpio.name, "mode", &hw_states_mode_[raw_motor_index]));
        RCLCPP_INFO(
            rclcpp::get_logger("DynamixelSystem"), "Added %s/%s",
            info_.gpios.at(i).name.c_str(), "mode");


        state_interfaces.emplace_back(hardware_interface::StateInterface(
            gpio.name, "current", &hw_states_current_[raw_motor_index]));

        RCLCPP_INFO(
            rclcpp::get_logger("DynamixelSystem"), "Added %s/%s",
            info_.gpios.at(i).name.c_str(), "current");


        RCLCPP_INFO(
            rclcpp::get_logger("DynamixelSystem"),
            "export state interface (GPIO) (%s) \"%s\"!", info_.name.c_str(), gpio.name.c_str());

        // next motor
        raw_motor_index++;

      }
      else
      {
        RCLCPP_WARN(
            rclcpp::get_logger("DynamixelSystem"),
            "Unknown state interface (GPIO) (%s) \"%s\"!", info_.name.c_str(), gpio.name.c_str());
      }

    }
    RCLCPP_INFO(
      rclcpp::get_logger("DynamixelSystem"),
      "export state interface (GPIO) (%s) done", info_.name.c_str());


    if (motor_index != 2 || raw_motor_index != 2)
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("DynamixelSystem"),
          "DynamixelHWI: Number of motors not correct: expected 2 found %ld! Or number of raw_motors not correct: expected 2 found %ld! Stopping operation!", motor_index, raw_motor_index);
      std::abort();
    }



  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DynamixelSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (std::size_t i = 0; i < 2; i++)
  {
    auto joint = info_.joints[i];

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint.name, hardware_interface::HW_IF_POSITION, &hw_commands_position_[i]));

    RCLCPP_INFO(
      rclcpp::get_logger("DynamixelSystem"),
      "export command interface (%s) \"%s\"!", info_.name.c_str(), joint.name.c_str()
      );
  }

    // motor index (not corresponding to the GPIO index)
    size_t raw_motor_index = 0;
    size_t motor_index = 0;

    RCLCPP_INFO(
      rclcpp::get_logger("DynamixelSystem"),
      "export command interface (GPIO) (%s) size: %ld", info_.name.c_str(), info_.gpios.size());

    // be careful GPIO index != motor or actuator index
    for (std::size_t i = 0; i < info_.gpios.size(); i++)
    {
      auto gpio = info_.gpios[i];


      // if (gpio.name == info_.name.c_str())
      if (gpio.name == (info_.name+"_left").c_str() || gpio.name == (info_.name+"_right").c_str()|| gpio.name.compare(std::string("l_hand"))==0  || gpio.name.compare(std::string("r_hand" ))==0) //for compatibility...
      {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            gpio.name, "torque", &hw_commands_torque_[motor_index]));

        RCLCPP_INFO(
            rclcpp::get_logger("DynamixelSystem"), "Added %s/%s",
            info_.gpios.at(i).name.c_str(), "torque");
        motor_index++;

        // RCLCPP_INFO(
        //     rclcpp::get_logger("DynamixelSystem"),
        //     "export command interface (GPIO) (%s) \"%s\"!", info_.name.c_str(), gpio.name.c_str());
      }
      else if (gpio.name.find("raw_motor") != std::string::npos)
      {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            gpio.name, "speed_limit", &hw_commands_speed_limit_[raw_motor_index]));
        RCLCPP_INFO(
          rclcpp::get_logger("DynamixelSystem"), "Added %s/%s",
          info_.gpios.at(i).name.c_str(), "speed_limit");

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            gpio.name, "torque_limit", &hw_commands_torque_limit_[raw_motor_index]));
        RCLCPP_INFO(
            rclcpp::get_logger("DynamixelSystem"), "Added %s/%s",
            info_.gpios.at(i).name.c_str(), "torque_limit");

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            gpio.name, "p_gain", &hw_commands_p_gain_[raw_motor_index]));
        RCLCPP_INFO(
            rclcpp::get_logger("DynamixelSystem"), "Added %s/%s",
            info_.gpios.at(i).name.c_str(), "p_gain");

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            gpio.name, "i_gain", &hw_commands_i_gain_[raw_motor_index]));
        RCLCPP_INFO(
            rclcpp::get_logger("DynamixelSystem"), "Added %s/%s",
            info_.gpios.at(i).name.c_str(), "i_gain");

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            gpio.name, "d_gain", &hw_commands_d_gain_[raw_motor_index]));
        RCLCPP_INFO(
            rclcpp::get_logger("DynamixelSystem"), "Added %s/%s",
            info_.gpios.at(i).name.c_str(), "d_gain");

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            gpio.name, "mode", &hw_commands_mode_[raw_motor_index]));
        RCLCPP_INFO(
            rclcpp::get_logger("DynamixelSystem"), "Added %s/%s",
            info_.gpios.at(i).name.c_str(), "mode");

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            gpio.name, "current", &hw_commands_current_[raw_motor_index]));
        RCLCPP_INFO(
            rclcpp::get_logger("DynamixelSystem"), "Added %s/%s",
            info_.gpios.at(i).name.c_str(), "current");


        RCLCPP_INFO(
            rclcpp::get_logger("DynamixelSystem"),
            "export command interface (GPIO) (%s) \"%s\"!", info_.name.c_str(), gpio.name.c_str());

        // next motor
        raw_motor_index++;

      }
      else
      {
        RCLCPP_WARN(
            rclcpp::get_logger("DynamixelSystem"),
            "Unknown command interface (GPIO) (%s) \"%s\"!", info_.name.c_str(), gpio.name.c_str());
      }

    }

    RCLCPP_INFO(
      rclcpp::get_logger("DynamixelSystem"),
      "export command interface (GPIO) (%s) done", info_.name.c_str());

    if (motor_index != 2 || raw_motor_index != 2)
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("DynamixelSystem"),
          "DynamixelHWI: Number of motors not correct: expected 2 found %ld! Or number of raw_motors not correct: expected 2 found %ld! Stopping operation!", motor_index, raw_motor_index);
      std::abort();
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

  if (dynamixel_2joints_get_target_torque(this->uid, &hw_states_current_) != 0) {
    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("DynamixelSystem"),
      clock_,
      LOG_THROTTLE_DURATION,
      "(%s) READ TARGET CURRENT ERROR!", info_.name.c_str()
    );
  }

  uint8_t mode[2] = {255,255};

  if (dynamixel_2joints_get_control_mode(this->uid, &mode) != 0) {
    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("DynamixelSystem"),
      clock_,
      LOG_THROTTLE_DURATION,
      "(%s) READ CONTROL MODE ERROR!", info_.name.c_str()
    );
  }
  hw_states_mode_[0] = static_cast<double>(mode[0]);
  hw_states_mode_[1] = static_cast<double>(mode[1]);


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


  bool torque_on[2] = {hw_commands_torque_[0] != 0.0, hw_commands_torque_[1] != 0.0};

  if (dynamixel_2joints_set_torque(this->uid, &torque_on) != 0) {
    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("DynamixelSystem"),
      clock_,
      LOG_THROTTLE_DURATION,
      "(%s) WRITE TORQUE ERROR!", info_.name.c_str()
    );
  }

  if (dynamixel_2joints_set_target_position(
    this->uid,
    &hw_commands_position_
  ) != 0) {
    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("DynamixelSystem"),
      clock_,
      LOG_THROTTLE_DURATION,
      "(%s) WRITE TARGET POSITION ERROR!", info_.name.c_str()
    );
  }


  uint8_t mode[2] = {static_cast<uint8_t>(hw_commands_mode_[0]), static_cast<uint8_t>(hw_commands_mode_[1])};

  if (dynamixel_2joints_set_control_mode(
    this->uid,
    &mode
  ) != 0) {
    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("DynamixelSystem"),
      clock_,
      LOG_THROTTLE_DURATION,
      "(%s) WRITE CONTROL MODE ERROR!", info_.name.c_str()
    );
  }

    if (dynamixel_2joints_set_raw_motors_velocity_limit(
    this->uid,
    &hw_commands_speed_limit_
  ) != 0) {
    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("DynamixelSystem"),
      clock_,
      LOG_THROTTLE_DURATION,
      "(%s) WRITE VELOCITY LIMIT ERROR!", info_.name.c_str()
    );
    }

    if (dynamixel_2joints_set_raw_motors_torque_limit(
    this->uid,
    &hw_commands_torque_limit_
  ) != 0) {
    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("DynamixelSystem"),
      clock_,
      LOG_THROTTLE_DURATION,
      "(%s) WRITE TORQUE LIMIT ERROR!", info_.name.c_str()
    );
  }



    if (dynamixel_2joints_set_target_torque(
          this->uid,
          &hw_commands_current_
          ) != 0) {
      RCLCPP_INFO_THROTTLE(
        rclcpp::get_logger("DynamixelSystem"),
        clock_,
        LOG_THROTTLE_DURATION,
      "(%s) WRITE TARGET CURRENT ERROR!", info_.name.c_str()
        );
    }



  return hardware_interface::return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dynamixel_2joints_system_hwi::DynamixelSystem,
  hardware_interface::SystemInterface)

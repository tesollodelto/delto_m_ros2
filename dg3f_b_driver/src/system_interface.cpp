// Copyright 2025 TESOLLO
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the TESOLLO nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "dg3f_b_driver/system_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dg3f_b_driver
{

hardware_interface::SystemInterface::CallbackReturn SystemInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{  
  if (hardware_interface::SystemInterface::CallbackReturn::SUCCESS !=
    hardware_interface::SystemInterface::on_init(info))
  {
    return CallbackReturn::ERROR;
  }

  positions_.resize(info_.joints.size(), 0.0);
  velocities_.resize(info_.joints.size(), 0.0);
  efforts_.resize(info_.joints.size(), 0.0);
  effort_commands_.resize(info_.joints.size(), 0.0);
  current_.resize(info_.joints.size(), 0.0);
  temperature_.resize(info_.joints.size(), 0.0);
  
  // Initialize current control variables
  current_limit_flag_.resize(info_.joints.size(), 0);
  current_integral_.resize(info_.joints.size(), 0.0);
  
  // Initialize connection status
  connection_status_ = 0.0;  // Start as disconnected
  is_connected_.store(false);

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_ERROR(
        rclcpp::get_logger("SystemInterface"),
        "Joint '%s' needs a command interface.", joint.name.c_str());

      return CallbackReturn::ERROR;
    }

    if (!(joint.state_interfaces[0].name ==
      hardware_interface::HW_IF_POSITION ||
      joint.state_interfaces[1].name ==
      hardware_interface::HW_IF_VELOCITY ||
      joint.state_interfaces[2].name == hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("SystemInterface"),
        "Joint '%s' needs the following state interfaces in this "
        "order: %s, %s",
        joint.name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  // Set default values
  delto_ip_ = "169.254.186.72";  // Default IP
  delto_port_ = 502;             // Default port
  fingertip_sensor_ = false;     // Default value
  io_ = false;                   // Default value
  model_ = 0x3F01;               // Default model for dg3f_b

  // Safely get parameters with defaults
  if (info.hardware_parameters.find("delto_ip") !=
      info.hardware_parameters.end()) {
    delto_ip_ = info.hardware_parameters.at("delto_ip");
  } else {
    RCLCPP_WARN(rclcpp::get_logger("SystemInterface"),
                "Parameter 'delto_ip' not found, using default: %s",
                delto_ip_.c_str());
  }

  if (info.hardware_parameters.find("delto_port") !=
      info.hardware_parameters.end()) {
    try {
      delto_port_ = std::stoi(info.hardware_parameters.at("delto_port"));
    } catch (...) {
      RCLCPP_WARN(rclcpp::get_logger("SystemInterface"),
                  "Invalid port parameter, using default: %d", delto_port_);
    }
  } else {
    RCLCPP_WARN(rclcpp::get_logger("SystemInterface"),
                "Parameter 'delto_port' not found, using default: %d",
                delto_port_);
  }

  RCLCPP_INFO(rclcpp::get_logger("SystemInterface"), "Delto model: 0x%X", model_);
  
  // Check fingertip parameter
  if (info.hardware_parameters.find("fingertip_sensor") !=
      info.hardware_parameters.end()) {
    fingertip_sensor_ =
        info.hardware_parameters.at("fingertip_sensor") == "true";
  }

  // Check IO parameter
  if (info.hardware_parameters.find("IO") != info.hardware_parameters.end()) {
    io_ = info.hardware_parameters.at("IO") == "true";
  }

  delto_client_ = std::make_unique<DeltoTCP::Communication>(
    delto_ip_, delto_port_, model_, fingertip_sensor_, io_);
  
  // Try to connect with proper error handling like dg5f_driver
  try {
    RCLCPP_INFO(rclcpp::get_logger("SystemInterface"), "Attempting to connect to %s:%d", delto_ip_.c_str(), delto_port_);
    delto_client_->Connect();
    is_connected_.store(true);
    connection_status_ = 1.0;
    RCLCPP_INFO(rclcpp::get_logger("SystemInterface"), "Successfully connected to device at %s:%d", delto_ip_.c_str(), delto_port_);
  } catch (...) {
    RCLCPP_WARN(rclcpp::get_logger("SystemInterface"),
                "Connect Failed.");
    is_connected_.store(false);
    connection_status_ = 0.0;
    return CallbackReturn::FAILURE;
  }
    // m_init_thread_ = std::thread(&SystemInterface::init, this);
  // m_init_thread_.detach();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
SystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(
    info_.joints.size() *
    2 + 1);                         // position, velocity + connection_status

  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &positions_[i]));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &velocities_[i]));

    std::cout << "export_state_interfaces: " << info_.joints[i].name << std::endl;
  }
  
  // Add connection status state interface
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      "system", "connection_status", &connection_status_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
SystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
        &effort_commands_[i]));
    
    std::cout << "export_command_interfaces: " << info_.joints[i].name << "/effort" << std::endl;
  }

  std::cout << "Total command interfaces exported: " << command_interfaces.size() << std::endl;
  return command_interfaces;
}

SystemInterface::return_type SystemInterface::prepare_command_mode_switch(
  [[maybe_unused]] const std::vector<std::string> & start_interfaces,
  [[maybe_unused]] const std::vector<std::string> & stop_interfaces)
{
  return return_type::OK;
}

SystemInterface::CallbackReturn SystemInterface::on_activate(
  [[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("SystemInterface"), "Starting DELTO dg3f_b_driver ...");

  // Connection is already established in on_init(), just check status
  if (is_connected_.load()) {
    RCLCPP_INFO(rclcpp::get_logger("SystemInterface"), "DELTO dg3f_b_driver started successfully!");
    return CallbackReturn::SUCCESS;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("SystemInterface"), "Device not connected, cannot activate");
    return CallbackReturn::ERROR;
  }
}

hardware_interface::SystemInterface::CallbackReturn
SystemInterface::on_deactivate(
  [[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("SystemInterface"), "Deactivating DELTO dg3f_b_driver ...");

  try {
    if (delto_client_) {
      delto_client_->Disconnect();
    }
    is_connected_.store(false);
    connection_status_ = 0.0;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("SystemInterface"), "Exception during deactivation: %s", e.what());
  }

  RCLCPP_INFO(rclcpp::get_logger("SystemInterface"), "DELTO dg3f_b_driver deactivated");
  
  return CallbackReturn::SUCCESS;
}

hardware_interface::SystemInterface::CallbackReturn
SystemInterface::on_shutdown(
  [[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("SystemInterface"), "Shutting down DELTO dg3f_b_driver ...");
  
  try {
    if (delto_client_) {
      delto_client_->Disconnect();
    }
    is_connected_.store(false);
    connection_status_ = 0.0;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("SystemInterface"), "Exception during shutdown: %s", e.what());
  }
  
  RCLCPP_INFO(rclcpp::get_logger("SystemInterface"), "DELTO dg3f_b_driver stopped");
  return CallbackReturn::SUCCESS;
}


SystemInterface::return_type SystemInterface::read(
  [[maybe_unused]] const rclcpp::Time & time, const rclcpp::Duration & period)
{
  try {
    if (!delto_client_) {
      RCLCPP_ERROR(rclcpp::get_logger("SystemInterface"), "Client is not initialized");
      is_connected_.store(false);
      connection_status_ = 0.0;
      return return_type::ERROR;
    }

    if (!is_connected_.load()) {
      connection_status_ = 0.0;
      return return_type::OK;
    }

    DeltoReceivedData received_data;
    try {
      received_data = delto_client_->GetData();
      // Update connection status to indicate successful communication
      is_connected_.store(true);
      connection_status_ = 1.0;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(rclcpp::get_logger("SystemInterface"), "Failed to read data: %s", e.what());
      is_connected_.store(false);
      connection_status_ = 0.0;
      return return_type::ERROR;
    }

    // Simplified data assignment like dg3f_m_driver
    if (received_data.joint.size() > 0) {
      positions_ = received_data.joint;
    }
    if (received_data.velocity.size() > 0) {
      velocities_ = received_data.velocity;  
    }
    if (received_data.current.size() > 0) {
      current_ = received_data.current;
      efforts_ = current_;
    }


    return return_type::OK;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("SystemInterface"), "Unexpected error in read: %s", e.what());
    is_connected_.store(false);
    connection_status_ = 0.0;
    return return_type::ERROR;
  }
}

SystemInterface::return_type SystemInterface::write(
  [[maybe_unused]] const rclcpp::Time & time,
  [[maybe_unused]] const rclcpp::Duration & period)
{
// Check if we're connected before trying to write
  if (!is_connected_.load()) {
    return return_type::ERROR;
  }

  std::vector<double> filter_effort_commands(effort_commands_.size());
  std::vector<double> duty(effort_commands_.size());
  std::vector<int> int_duty(effort_commands_.size());
  std::vector<int> current_mA(effort_commands_.size());

  for (size_t i = 0; i < effort_commands_.size(); ++i) {
    current_mA[i] = static_cast<int>(current_[i]);
  }
  
  try {
    filter_effort_commands = delto_gripper_helper::CurrentControl(
        effort_commands_.size(), current_mA, effort_commands_,
        current_limit_flag_, current_integral_);

    duty = delto_gripper_helper::ConvertDuty(effort_commands_.size(),
                                             filter_effort_commands);

    for (size_t i = 0; i < effort_commands_.size(); ++i) {
      int_duty[i] = static_cast<int>(duty[i] * 10);
      int_duty[i] = std::clamp(int_duty[i], -1000, 1000);
    }

    // for (auto & i : int_duty) {
    //   i *= -1;
    // }

    delto_client_->SendDuty(int_duty);
    
    // Update connection status - if we reach here, write was successful
    is_connected_.store(true);
    connection_status_ = 1.0;
    
  } catch (const std::exception & e) {
    std::cerr << "Failed to write data: " << e.what() << std::endl;
    // Update connection status on error
    is_connected_.store(false);
    connection_status_ = 0.0;
    return return_type::ERROR;
  }
  return return_type::OK;
}
}  // namespace dg3f_b_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  dg3f_b_driver::SystemInterface,
  hardware_interface::SystemInterface)

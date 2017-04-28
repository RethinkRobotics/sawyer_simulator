/***************************************************************************
* Copyright (c) 2013-2017, Rethink Robotics Inc.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
**************************************************************************/

// Overload the default plugin
#include <sawyer_gazebo/sawyer_gazebo_ros_control_plugin.h>
#include <controller_manager_msgs/SwitchController.h>

namespace sawyer_gazebo_plugin {

void SawyerGazeboRosControlPlugin::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf) {
  GazeboRosControlPlugin::Load(parent, sdf);
  SawyerGazeboRosControlPlugin::initAssembly(model_nh_);
  speed_ratio_sub_ = model_nh_.subscribe<std_msgs::Float64>("/robot/limb/right/set_speed_ratio", 1, &SawyerGazeboRosControlPlugin::speedRatioCallback, this);
  joint_command_timeout_sub_ = model_nh_.subscribe<std_msgs::Float64>("/robot/limb/right/joint_command_timeout", 1, &SawyerGazeboRosControlPlugin::jointCommandTimeoutCallback, this);
  joint_command_sub_ = model_nh_.subscribe<intera_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1, &SawyerGazeboRosControlPlugin::jointCommandCallback, this);
}

void SawyerGazeboRosControlPlugin::initAssembly(ros::NodeHandle& nh) {
    //Default values for the assembly state
    std::shared_ptr<intera_core_msgs::AssemblyState> assembly_state(new intera_core_msgs::AssemblyState());
    assembly_state->ready = true;  // true if enabled
    assembly_state->enabled = true;  // true if enabled
    assembly_state->stopped = false; // true if stopped -- e-stop asserted
    assembly_state->error = false;  // true if a component of the assembly has an error
    assembly_state->lowVoltage = false;  // true if the robot entered lowVoltage mode
    assembly_state->estop_button = intera_core_msgs::AssemblyState::ESTOP_BUTTON_UNPRESSED;  // button status
    assembly_state->estop_source = intera_core_msgs::AssemblyState::ESTOP_SOURCE_NONE;  // If stopped is
    is_enabled_ = assembly_state->enabled;
    is_stopped_ = assembly_state->stopped;
    assembly_state_buffer_.set(assembly_state);
    assembly_state_pub_ = nh.advertise<intera_core_msgs::AssemblyState>("state", 1);
    assembly_state_timer_ = nh.createTimer(100, &SawyerGazeboRosControlPlugin::updateAssembly, this);// 100Hz
    assembly_enable_sub_ = nh.subscribe("set_super_enable", 100, &SawyerGazeboRosControlPlugin::callbackEnableAssembly, this);
    assembly_stop_sub_ =  nh.subscribe("set_super_stop", 100, &SawyerGazeboRosControlPlugin::callbackStopAssembly, this);
    assembly_reset_sub_ = nh.subscribe("set_super_reset", 100, &SawyerGazeboRosControlPlugin::callbackResetAssembly, this);
}

void SawyerGazeboRosControlPlugin::updateAssembly(const ros::TimerEvent& e){
    std::shared_ptr<const intera_core_msgs::AssemblyState> assembly_state;
    assembly_state_buffer_.get(assembly_state);
    is_enabled_ = assembly_state->enabled;
    is_stopped_ = assembly_state->stopped;
    GazeboRosControlPlugin::e_stop_active_ = !is_enabled_ || is_stopped_;
    assembly_state_pub_.publish(*assembly_state);
}

void SawyerGazeboRosControlPlugin::callbackEnableAssembly(const std_msgs::Bool &msg) {
  std::shared_ptr<intera_core_msgs::AssemblyState> assembly_state(new intera_core_msgs::AssemblyState());
  if (msg.data) {
    assembly_state->enabled = true;
  }
  else {
    assembly_state->enabled = false;
  }
  assembly_state->ready = true;
  assembly_state->stopped = false;
  assembly_state->error = false;
  assembly_state->lowVoltage = false;
  assembly_state->estop_button = intera_core_msgs::AssemblyState::ESTOP_BUTTON_UNPRESSED;
  assembly_state->estop_source = intera_core_msgs::AssemblyState::ESTOP_SOURCE_NONE;
  assembly_state_buffer_.set(assembly_state);
}

 /**
  * Method to stop the robot and capture the source of the stop
  */
void SawyerGazeboRosControlPlugin::callbackStopAssembly(const std_msgs::Empty &msg) {
  std::shared_ptr<intera_core_msgs::AssemblyState> assembly_state(new intera_core_msgs::AssemblyState());
  assembly_state->enabled = false;
  assembly_state->ready = false;
  assembly_state->stopped = true;
  assembly_state->error = false;
  assembly_state->lowVoltage = false;
  assembly_state->estop_button = intera_core_msgs::AssemblyState::ESTOP_BUTTON_UNPRESSED;
  assembly_state->estop_source = intera_core_msgs::AssemblyState::ESTOP_SOURCE_UNKNOWN;
  assembly_state_buffer_.set(assembly_state);
}

 /**
  * Method resets all the values to False and 0s
  */
void SawyerGazeboRosControlPlugin::callbackResetAssembly(const std_msgs::Empty &msg) {
  std::shared_ptr<intera_core_msgs::AssemblyState> assembly_state(new intera_core_msgs::AssemblyState());
  assembly_state->enabled = false;
  assembly_state->ready = true;
  assembly_state->stopped = false;
  assembly_state->error = false;
  assembly_state->estop_button = intera_core_msgs::AssemblyState::ESTOP_BUTTON_UNPRESSED;
  assembly_state->estop_source = intera_core_msgs::AssemblyState::ESTOP_SOURCE_NONE;
  assembly_state_buffer_.set(assembly_state);
}

void SawyerGazeboRosControlPlugin::speedRatioCallback(const std_msgs::Float64 msg) {
  ROS_INFO_STREAM_NAMED("ros_control_plugin", "Data: " << msg.data);
}

void SawyerGazeboRosControlPlugin::jointCommandTimeoutCallback(const std_msgs::Float64 msg) {
  ROS_INFO_STREAM_NAMED("sawyer_control_plugin", "Joint command timeout: " << msg.data);
}

void SawyerGazeboRosControlPlugin::jointCommandCallback(const intera_core_msgs::JointCommandConstPtr& msg) {
  // lock out other thread(s) which are getting called back via ros.
  std::lock_guard<std::mutex> guard(mtx_);

  std::vector<std::string> start_controllers;
  std::vector<std::string> stop_controllers;
  std::string start, stop;
  std::string side = "right";

  switch (msg->mode)
  {
    case intera_core_msgs::JointCommand::POSITION_MODE:
      start_controllers.push_back(side + "_joint_position_controller");
      stop_controllers.push_back(side + "_joint_velocity_controller");
      stop_controllers.push_back(side + "_joint_effort_controller");
      start = side + "_joint_position_controller";
      stop = side + "_joint_velocity_controller and " + side + "_joint_effort_controller";
      break;
    case intera_core_msgs::JointCommand::VELOCITY_MODE:
      start_controllers.push_back(side + "_joint_velocity_controller");
      stop_controllers.push_back(side + "_joint_position_controller");
      stop_controllers.push_back(side + "_joint_effort_controller");
      start = side + "_joint_velocity_controller";
      stop = side + "_joint_position_controller and " + side + "_joint_effort_controller";
      break;
    case intera_core_msgs::JointCommand::TORQUE_MODE:
      start_controllers.push_back(side + "_joint_effort_controller");
      stop_controllers.push_back(side + "_joint_position_controller");
      stop_controllers.push_back(side + "_joint_velocity_controller");
      start = side + "_joint_effort_controller";
      stop = side + "_joint_velocity_controller and " + side + "_joint_position_controller";
      break;
    default:
      ROS_ERROR_STREAM_NAMED("ros_control_plugin", "Unknown command mode " << msg->mode);
      return;
  }
  // Checks if we have already disabled the controllers
  /** \brief Switch multiple controllers simultaneously.
   *
   * \param start_controllers A vector of controller names to be started
   * \param stop_controllers A vector of controller names to be stopped
   * \param strictness How important it is that the requested controllers are
   * started and stopped.  The levels are defined in the
   * controller_manager_msgs/SwitchControllers service as either \c BEST_EFFORT
   * or \c STRICT.  \c BEST_EFFORT means that \ref switchController can still
   * succeed if a non-existant controller is requested to be stopped or started.
   */
  if (!controller_manager_->switchController(start_controllers, stop_controllers,
                                             controller_manager_msgs::SwitchController::Request::BEST_EFFORT))
  {
    ROS_ERROR_STREAM_NAMED("ros_control_plugin", "Failed to switch controllers");
  }
  else
  {
    //is_disabled = false;
    ROS_INFO_NAMED("ros_control_plugin", "Robot is enabled");
    ROS_DEBUG_STREAM_NAMED("ros_control_plugin", "Controller " << start << " started and " << stop << " stopped.");
    ROS_INFO_NAMED("ros_control_plugin", "Gravity compensation was turned on by mode command callback");
  }
}
// register the plugin with gazebo
GZ_REGISTER_MODEL_PLUGIN(SawyerGazeboRosControlPlugin);
}

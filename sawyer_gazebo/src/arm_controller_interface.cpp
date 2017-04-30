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

#include <sawyer_gazebo/arm_controller_interface.h>
#include <controller_manager_msgs/SwitchController.h>

namespace sawyer_gazebo {

void ArmControllerInterface::init(ros::NodeHandle& nh, boost::shared_ptr<controller_manager::ControllerManager> controller_manager) {
  controller_manager_ = controller_manager;
  speed_ratio_sub_ = nh.subscribe("limb/right/set_speed_ratio", 1, &ArmControllerInterface::speedRatioCallback, this);
  joint_command_timeout_sub_ = nh.subscribe("limb/right/joint_command_timeout", 1, &ArmControllerInterface::jointCommandTimeoutCallback, this);
  joint_command_sub_ = nh.subscribe("limb/right/joint_command", 1, &ArmControllerInterface::jointCommandCallback, this);
}

void ArmControllerInterface::speedRatioCallback(const std_msgs::Float64 msg) {
  ROS_INFO_STREAM_NAMED("ros_control_plugin", "Data: " << msg.data);
}

void ArmControllerInterface::jointCommandTimeoutCallback(const std_msgs::Float64 msg) {
  ROS_INFO_STREAM_NAMED("sawyer_control_plugin", "Joint command timeout: " << msg.data);
}

void ArmControllerInterface::jointCommandCallback(const intera_core_msgs::JointCommandConstPtr& msg) {
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
}

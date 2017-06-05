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

void ArmControllerInterface::init(ros::NodeHandle& nh, std::string side,
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager) {
  current_mode_ = -1;
  side_ = side;
  controller_manager_ = controller_manager;
  speed_ratio_sub_ = nh.subscribe("limb/"+side_+"/set_speed_ratio", 1,
                       &ArmControllerInterface::speedRatioCallback, this);
  joint_command_timeout_sub_ = nh.subscribe("limb/"+side_+"/joint_command_timeout", 1,
                       &ArmControllerInterface::jointCommandTimeoutCallback, this);
  joint_command_sub_ = nh.subscribe("limb/"+side_+"/joint_command", 1,
                       &ArmControllerInterface::jointCommandCallback, this);
}

void ArmControllerInterface::speedRatioCallback(const std_msgs::Float64 msg) {
  ROS_INFO_STREAM_NAMED("ros_control_plugin", "Data: " << msg.data);
}

void ArmControllerInterface::jointCommandTimeoutCallback(const std_msgs::Float64 msg) {
  ROS_INFO_STREAM_NAMED("sawyer_control_plugin", "Joint command timeout: " << msg.data);
}

std::string ArmControllerInterface::getControllerString(std::string mode_str){
    std::ostringstream ss;
    ss << side_ <<"_joint_"<<mode_str<<"_controller";
    return ss.str();
}

void ArmControllerInterface::jointCommandCallback(const intera_core_msgs::JointCommandConstPtr& msg) {
  // lock out other thread(s) which are getting called back via ros.
  std::lock_guard<std::mutex> guard(mtx_);

  std::vector<std::string> start_controllers;
  std::vector<std::string> stop_controllers;
  if(current_mode_ != msg->mode)
  {
    switch (msg->mode)
    {
      case intera_core_msgs::JointCommand::POSITION_MODE:
      case intera_core_msgs::JointCommand::TRAJECTORY_MODE:
        start_controllers.push_back(getControllerString("position"));
        start_controllers.push_back(getControllerString("gravity"));
        stop_controllers.push_back(getControllerString("velocity"));
        stop_controllers.push_back(getControllerString("effort"));
        break;
      case intera_core_msgs::JointCommand::VELOCITY_MODE:
        start_controllers.push_back(getControllerString("velocity"));
        start_controllers.push_back(getControllerString("gravity"));
        stop_controllers.push_back(getControllerString("position"));
        stop_controllers.push_back(getControllerString("effort"));
        break;
      case intera_core_msgs::JointCommand::TORQUE_MODE:
        start_controllers.push_back(getControllerString("effort"));
        start_controllers.push_back(getControllerString("gravity"));
        stop_controllers.push_back(getControllerString("position"));
        stop_controllers.push_back(getControllerString("velocity"));
        break;
      default:
        ROS_ERROR_STREAM_NAMED("ros_control_plugin", "Unknown command mode " << msg->mode);
        return;
    }
    if (!controller_manager_->switchController(start_controllers, stop_controllers,
                              controller_manager_msgs::SwitchController::Request::BEST_EFFORT))
    {
      ROS_ERROR_STREAM_NAMED("ros_control_plugin", "Failed to switch controllers");
    }
    else
    {
      current_mode_ = msg->mode;
      //is_disabled = false;
      //ROS_INFO_NAMED("ros_control_plugin", "Robot is enabled");
      ROS_INFO_STREAM_NAMED("ros_control_plugin", "Controller " << start_controllers[0]
                            << " started and " << stop_controllers[0] + " and " + stop_controllers[1] << " stopped.");
      //ROS_INFO_NAMED("ros_control_plugin", "Gravity compensation was turned on by mode command callback");
    }
  }

}
}

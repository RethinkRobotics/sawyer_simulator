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

#include <sawyer_sim_controllers/sawyer_position_controller.h>
#include <pluginlib/class_list_macros.h>

namespace sawyer_sim_controllers {
  bool SawyerPositionController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n){
    if(!sawyer_sim_controllers::JointGroupPositionController::init(hw, n)) {
      return false;
    } else {
      std::string topic_name;
      if (n.getParam("topic_joint_command", topic_name)) {
        ros::NodeHandle nh("~");
        sub_joint_command_ = nh.subscribe(topic_name, 1, &SawyerPositionController::jointCommandCB, this);
      } else {
        sub_joint_command_ = n.subscribe("joint_command", 1, &SawyerPositionController::jointCommandCB, this);
      }
      std::string topic_speed_ratio;
      if (n.getParam("topic_set_speed_ratio", topic_speed_ratio)) {
        ros::NodeHandle nh("~");
        sub_speed_ratio_ = nh.subscribe(topic_speed_ratio, 1, &SawyerPositionController::speedRatioCallback, this);
      } else {
        sub_speed_ratio_ = n.subscribe("set_speed_ratio", 1, &SawyerPositionController::speedRatioCallback, this);
      }
      std::shared_ptr<std_msgs::Float64> speed_ratio(new std_msgs::Float64());
      speed_ratio->data = 0.3; // Default to 30% max urdf speed
      speed_ratio_buffer_.set(speed_ratio);
    }
    return true;
  }

  void SawyerPositionController::speedRatioCallback(const std_msgs::Float64 msg) {
    std::shared_ptr<std_msgs::Float64> speed_ratio(new std_msgs::Float64());
    if(msg.data > 1){
      speed_ratio->data = 1;
    }
    else if(msg.data < 0){
      speed_ratio->data = 0;
    }
    else{
      speed_ratio->data = msg.data;
    }
    speed_ratio_buffer_.set(speed_ratio);
    ROS_INFO_STREAM_NAMED("Speed Ratio Callback", "received " << msg.data);
  }

  void SawyerPositionController::jointCommandCB(const intera_core_msgs::JointCommandConstPtr& msg) {
    // TODO: Guard against out of range, bad joints, and invalid commands
    std::vector<Command> commands;
    std::shared_ptr<const std_msgs::Float64> speed_ratio;
    speed_ratio_buffer_.get(speed_ratio);
    for (int i = 0; i < msg->names.size(); i++) {
      Command cmd = Command();
      cmd.name_ = msg->names[i];
      cmd.position_ = msg->position[i];
      if(msg->mode == intera_core_msgs::JointCommand::POSITION_MODE && speed_ratio){
        // joint_velocity = speed_ratio * max joint velocity from urdf * direction of joint motion
        cmd.velocity_ = speed_ratio->data * position_controllers_[cmd.name_]->joint_urdf_->limits->velocity *
                          (cmd.position_ - position_controllers_[cmd.name_]->joint_.getPosition());
        cmd.has_velocity_=true;
      }
      else if(msg->mode == intera_core_msgs::JointCommand::TRAJECTORY_MODE &&
              msg->position.size() == msg->velocity.size()){
          cmd.velocity_ = msg->velocity[i];
          cmd.has_velocity_=true;
      }
      commands.push_back(cmd);
    }
    position_command_buffer_.writeFromNonRT(commands);
    new_command_ = true;
  }

}

PLUGINLIB_EXPORT_CLASS(sawyer_sim_controllers::SawyerPositionController, controller_interface::ControllerBase)

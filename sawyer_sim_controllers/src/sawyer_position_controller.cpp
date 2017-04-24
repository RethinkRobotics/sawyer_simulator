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
      if (n.getParam("topic", topic_name)) {
        ros::NodeHandle nh("~");
        sub_joint_command_ = nh.subscribe(topic_name, 1, &SawyerPositionController::jointCommandCB, this);
      } else {
        sub_joint_command_ = n.subscribe("joint_command", 1, &SawyerPositionController::jointCommandCB, this);
      }
    }
    return true;
  }

  void SawyerPositionController::jointCommandCB(const intera_core_msgs::JointCommandConstPtr& msg) {
    std::vector<Command> commands;

    for (int i = 0; i < msg->names.size(); i++) {
      Command cmd = Command();
      cmd.name_ = msg->names[i];
      cmd.position_ = msg->position[i];
      commands.push_back(cmd);
    }
    position_command_buffer_.writeFromNonRT(commands);
    new_command_ = true;
  }

}

PLUGINLIB_EXPORT_CLASS(sawyer_sim_controllers::SawyerPositionController, controller_interface::ControllerBase)
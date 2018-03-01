/***************************************************************************
* Copyright (c) 2013-2018, Rethink Robotics Inc.
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

#include <sawyer_sim_controllers/sawyer_gravity_controller.h>
#include <pluginlib/class_list_macros.h>

namespace sawyer_sim_controllers {

  bool SawyerGravityController::init(sawyer_hardware_interface::SharedJointInterface* hw, ros::NodeHandle &n){
    // TODO: use constant, don't hardcode ctrl_subtype ("GRAVITY_COMPENSATION")
    if(!sawyer_sim_controllers::JointArrayController<sawyer_effort_controllers::JointEffortController>::init(hw, n, "GRAVITY_COMPENSATION")) {
      return false;
    } else {
      std::string topic_name;
      if (n.getParam("topic", topic_name)) {
        ros::NodeHandle nh("~");
        sub_joint_command_ = nh.subscribe(topic_name, 1, &SawyerGravityController::gravityCommandCB, this);
      } else {
        sub_joint_command_ = n.subscribe("gravity_command", 1, &SawyerGravityController::gravityCommandCB, this);
      }
    }
    return true;
  }

  void SawyerGravityController::gravityCommandCB(const intera_core_msgs::SEAJointStateConstPtr& msg) {

      std::vector<Command> commands;
      if (msg->name.size() != msg->gravity_only.size()) {
        ROS_ERROR_STREAM_NAMED(JOINT_ARRAY_CONTROLLER_NAME, "Gravity commands size does not match joints size");
      }
      for (int i = 0; i < msg->name.size(); i++) {
        Command cmd = Command();
        cmd.name_ = msg->name[i];
        cmd.effort_ = msg->gravity_only[i];
        commands.push_back(cmd);
      }
      command_buffer_.writeFromNonRT(commands);
      new_command_ = true;
  }

  void SawyerGravityController::setCommands() {
    // set the new commands for each controller
    std::vector<Command> command = *(command_buffer_.readFromRT());
    for (auto it = command.begin(); it != command.end(); it++) {
      controllers_[it->name_]->command_buffer_.writeFromNonRT(it->effort_);
    }
  }
}

PLUGINLIB_EXPORT_CLASS(sawyer_sim_controllers::SawyerGravityController, controller_interface::ControllerBase)

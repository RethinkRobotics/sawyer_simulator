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

#include <sawyer_sim_controllers/joint_group_position_controller.h>
#include <pluginlib/class_list_macros.h>

namespace sawyer_sim_controllers {
  JointGroupPositionController::JointGroupPositionController() : new_command_(true) {}
  JointGroupPositionController::~JointGroupPositionController() {};

  bool JointGroupPositionController::init(hardware_interface::EffortJointInterface* robot, ros::NodeHandle& nh) {
    // Get joint sub-controllers
    XmlRpc::XmlRpcValue xml_struct;
    if (!nh.getParam("joints", xml_struct))
    {
      ROS_ERROR_NAMED("position", "No 'joints' parameter in controller (namespace '%s')", nh.getNamespace().c_str());
      return false;
    }

    // Make sure it's a struct
    if (xml_struct.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_ERROR_NAMED("position", "The 'joints' parameter is not a struct (namespace '%s')", nh.getNamespace().c_str());
      return false;
    }

    // Get number of joints
    n_joints_ = xml_struct.size();
    ROS_INFO_STREAM_NAMED("position", "Initializing JointGroupPositionController with " << n_joints_ << " joints.");

    int i = 0;  // track the joint id
    for (XmlRpc::XmlRpcValue::iterator joint_it = xml_struct.begin(); joint_it != xml_struct.end(); ++joint_it)
    {
      // Get joint controller
      if (joint_it->second.getType() != XmlRpc::XmlRpcValue::TypeStruct)
      {
        ROS_ERROR_NAMED("position", "The 'joints/joint_controller' parameter is not a struct (namespace '%s')",
                        nh.getNamespace().c_str());
        return false;
      }

      // Get joint controller name
      std::string joint_controller_name = joint_it->first;

      // Get the joint name
      std::string joint_name = joint_it->second["joint"];

      // Get the joint-namespace nodehandle
      {
        ros::NodeHandle joint_nh(nh, "joints/" + joint_controller_name);
        ROS_INFO_STREAM_NAMED("init", "Loading sub-controller '" << joint_controller_name
                                                                 << "', Namespace: " << joint_nh.getNamespace());

        position_controllers_[joint_name].reset(new effort_controllers::JointPositionController());
        position_controllers_[joint_name]->init(robot, joint_nh);

      }  // end of joint-namespaces

      // increment joint i
      ++i;
    }

    return true;
  }

  void JointGroupPositionController::starting(const ros::Time& time) {
    for (JointPositionControllerMap::iterator it = position_controllers_.begin(); it != position_controllers_.end(); it++) {
      it->second->starting(time);
    }
  }

  void JointGroupPositionController::stopping(const ros::Time& time) {

  }

  void JointGroupPositionController::update(const ros::Time& time, const ros::Duration& period) {
    // first check if there are new commands
    if (new_command_) {
      // assume we will succeed at this command
      new_command_ = false;

      // set the new commands for each controller
      std::vector<Command> command = *(position_command_buffer_.readFromRT());
      for (std::vector<Command>::iterator it = command.begin(); it != command.end(); it++) {
        if(it->has_velocity_){
          position_controllers_[it->name_]->setCommand(it->position_, it->velocity_);
        }
        else{
          position_controllers_[it->name_]->setCommand(it->position_);
        }
      }
    }

    // always update all controllers
    for (JointPositionControllerMap::iterator it = position_controllers_.begin(); it != position_controllers_.end(); it++) {
      it->second->update(time, period);
    }
  }

} // namespace
PLUGINLIB_EXPORT_CLASS(sawyer_sim_controllers::JointGroupPositionController, controller_interface::ControllerBase)

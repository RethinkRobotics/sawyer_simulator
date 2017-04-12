/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

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
          position_controllers_[it->name_]->setCommand(it->position_);
        }
      }

      // always update all controllers
      for (JointPositionControllerMap::iterator it = position_controllers_.begin(); it != position_controllers_.end(); it++) {
        it->second->update(time, period);
      }
    }

} // namespace
PLUGINLIB_EXPORT_CLASS(sawyer_sim_controllers::JointGroupPositionController, controller_interface::ControllerBase)

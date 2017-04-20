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

// Overload the default plugin
#include <gazebo_ros_control/gazebo_ros_control_plugin.h>
#include <controller_manager_msgs/SwitchController.h>

#include <intera_core_msgs/JointCommand.h>
#include <std_msgs/Float64.h>

namespace sawyer_gazebo_plugin
{
class SawyerGazeboRosControlPlugin : public gazebo_ros_control::GazeboRosControlPlugin
{
private:
    // mutex for re-entrant calls to modeCommandCallback
    std::mutex mtx_;
    ros::Subscriber speed_ratio_sub_;
    ros::Subscriber joint_command_timeout_sub_;
    ros::Subscriber joint_command_sub_;

public:
    void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf) {
      GazeboRosControlPlugin::Load(parent, sdf);

      speed_ratio_sub_ = model_nh_.subscribe<std_msgs::Float64>("/robot/limb/right/set_speed_ratio", 1, &SawyerGazeboRosControlPlugin::speedRatioCallback, this);
      joint_command_timeout_sub_ = model_nh_.subscribe<std_msgs::Float64>("/robot/limb/right/joint_command_timeout", 1, &SawyerGazeboRosControlPlugin::jointCommandTimeoutCallback, this);
      joint_command_sub_ = model_nh_.subscribe<intera_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1, &SawyerGazeboRosControlPlugin::jointCommandCallback, this);
    }

    void speedRatioCallback(const std_msgs::Float64 msg) {
      ROS_INFO_STREAM_NAMED("ros_control_plugin", "Data: " << msg.data);
    }

    void jointCommandTimeoutCallback(const std_msgs::Float64 msg) {
      ROS_INFO_STREAM_NAMED("sawyer_control_plugin", "Joint command timeout: " << msg.data);
    }

    void jointCommandCallback(const intera_core_msgs::JointCommandConstPtr& msg) {
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
};

// register the plugin with gazebo
GZ_REGISTER_MODEL_PLUGIN(SawyerGazeboRosControlPlugin);
}

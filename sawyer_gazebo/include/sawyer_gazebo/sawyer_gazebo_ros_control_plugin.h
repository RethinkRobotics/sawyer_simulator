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
#ifndef _GAZEBO_ROS_CONTROL___SAWYER_ROBOT_HW_SIM_H_
#define _GAZEBO_ROS_CONTROL___SAWYER_ROBOT_HW_SIM_H_
// Overload the default plugin
#include <gazebo_ros_control/gazebo_ros_control_plugin.h>

#include <realtime_tools/realtime_box.h>

#include <intera_core_msgs/JointCommand.h>
#include <intera_core_msgs/AssemblyState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>


namespace sawyer_gazebo_plugin {
  class SawyerGazeboRosControlPlugin : public gazebo_ros_control::GazeboRosControlPlugin
  {
  private:
    // mutex for re-entrant calls to modeCommandCallback
    std::mutex mtx_;
    ros::Subscriber speed_ratio_sub_;
    ros::Subscriber joint_command_timeout_sub_;
    ros::Subscriber joint_command_sub_;

    bool is_enabled_;
    bool is_stopped_;
    ros::Publisher assembly_state_pub_;
    ros::Subscriber assembly_enable_sub_;
    ros::Subscriber assembly_stop_sub_;
    ros::Subscriber assembly_reset_sub_;
    ros::Timer assembly_state_timer_;
    realtime_tools::RealtimeBox< std::shared_ptr<const intera_core_msgs::AssemblyState> > assembly_state_buffer_;
  protected:
    void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);

    void speedRatioCallback(const std_msgs::Float64 msg);
    void jointCommandTimeoutCallback(const std_msgs::Float64 msg);
    void jointCommandCallback(const intera_core_msgs::JointCommandConstPtr& msg);

    /**
     * Method to stop the robot and capture the source of the stop
     */
    void initAssembly(ros::NodeHandle& nh);
    void updateAssembly(const ros::TimerEvent& e);
    void callbackEnableAssembly(const std_msgs::Bool &msg);
    void callbackStopAssembly(const std_msgs::Empty &msg);
    void callbackResetAssembly(const std_msgs::Empty &msg);

  };
}
#endif // #ifndef __GAZEBO_ROS_CONTROL_PLUGIN_SAWYER_ROBOT_HW_SIM_H_

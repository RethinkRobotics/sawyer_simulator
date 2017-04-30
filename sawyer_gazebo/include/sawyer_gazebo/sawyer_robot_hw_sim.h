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

#ifndef _SAWYER_GAZEBO___SAWYER_ROBOT_HW_SIM_H_
#define _SAWYER_GAZEBO___SAWYER_ROBOT_HW_SIM_H_

// gazebo_ros_control
#include <gazebo_ros_control/default_robot_hw_sim.h>


namespace sawyer_gazebo
{

class SawyerRobotHWSim : public gazebo_ros_control::DefaultRobotHWSim
{
public:

  virtual bool initSim(
    const std::string& robot_namespace,
    ros::NodeHandle model_nh,
    gazebo::physics::ModelPtr parent_model,
    const urdf::Model *const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions);

  virtual void eStopActive(const bool active);

protected:
  std::vector<double> joint_damping_;
};

typedef std::shared_ptr<SawyerRobotHWSim> SawyerRobotHWSimPtr;

}

#endif // #ifndef __SAWYER_GAZEBO_PLUGIN_DEFAULT_ROBOT_HW_SIM_H_

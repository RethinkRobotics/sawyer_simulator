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

#include <sawyer_gazebo/sawyer_robot_hw_sim.h>

namespace sawyer_gazebo
{

bool SawyerRobotHWSim::initSim(
  const std::string& robot_namespace,
  ros::NodeHandle model_nh,
  gazebo::physics::ModelPtr parent_model,
  const urdf::Model *const urdf_model,
  std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  bool ret = gazebo_ros_control::DefaultRobotHWSim::initSim(robot_namespace, model_nh, parent_model, urdf_model, transmissions);
  joint_damping_.resize(gazebo_ros_control::DefaultRobotHWSim::n_dof_);
  for(std::size_t i = 0; i != gazebo_ros_control::DefaultRobotHWSim::sim_joints_.size(); ++i)
  {
    joint_damping_[i] = gazebo_ros_control::DefaultRobotHWSim::sim_joints_[i]->GetDamping(0);
  }
  return ret;
}

void SawyerRobotHWSim::eStopActive(const bool active)
{
  gazebo_ros_control::DefaultRobotHWSim::eStopActive(active);
  for(std::size_t i = 0; i != gazebo_ros_control::DefaultRobotHWSim::sim_joints_.size(); ++i)
  {
    gazebo_ros_control::DefaultRobotHWSim::sim_joints_[i]->SetDamping(0, (active ? 100000 : joint_damping_[i]));
  }
}

}

PLUGINLIB_EXPORT_CLASS(sawyer_gazebo::SawyerRobotHWSim, gazebo_ros_control::RobotHWSim)

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

#ifndef SAWYER_GRAVITY_CONTROLLER_H
#define SAWYER_GRAVITY_CONTROLLER_H

#include <sawyer_sim_controllers/joint_array_controller.h>
#include <std_msgs/Empty.h>
#include <realtime_tools/realtime_box.h>
#include <sawyer_sim_controllers/sawyer_joint_effort_controller.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <sensor_msgs/JointState.h>

#include <kdl/jntarray.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

namespace sawyer_sim_controllers
{
  class SawyerGravityController : public sawyer_sim_controllers::JointArrayController<sawyer_effort_controllers::JointEffortController>
  {
  public:
    SawyerGravityController() : sawyer_sim_controllers::JointArrayController<sawyer_effort_controllers::JointEffortController>() { };

    virtual ~SawyerGravityController() {sub_joint_command_.shutdown();}
    virtual bool init(sawyer_hardware_interface::SharedJointInterface* hw, ros::NodeHandle &n) override;
    void setCommands();

  private:
    struct LastVelocity
    {
      ros::Time time;
      KDL::JntArray velocity;
    };
    realtime_tools::RealtimeBox< std::shared_ptr<const LastVelocity > > box_last_velocity_;
    realtime_tools::RealtimeBox< std::shared_ptr<const ros::Time > > box_disable_time_;
    ros::Subscriber sub_joint_command_;
    ros::Subscriber sub_gravity_disable_;
    ros::Duration gravity_disable_timeout_;

    KDL::Chain chain_;
    std::vector<std::string> joint_names_;
    std::unique_ptr<KDL::ChainIdSolver_RNE> inverse_dynamics_solver_;

    void gravityDisableCB(const std_msgs::Empty& msg);
    bool computeInverseDynamicsTorques(const KDL::JntArray& jnt_pos,
                                       const KDL::JntArray& jnt_vel,
                                       const KDL::JntArray& jnt_accel,
                                       KDL::JntArray& jnt_torques);
    void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);
    bool parseParams(const ros::NodeHandle& nh);
    void jointStateToKDL(const sensor_msgs::JointState& joint_state, const std::vector<std::string>& joint_names,
                          KDL::JntArray& jnt_pos, KDL::JntArray& jnt_vel, KDL::JntArray& jnt_eff);
  };
}

#endif

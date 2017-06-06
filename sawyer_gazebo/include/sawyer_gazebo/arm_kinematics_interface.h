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
#ifndef _SAWYER_GAZEBO___ARM_KINEMATICS_INTERFACE_H_
#define _SAWYER_GAZEBO___ARM_KINEMATICS_INTERFACE_H_

#include <map>

#include <ros/ros.h>
#include <realtime_tools/realtime_box.h>
#include <intera_core_msgs/JointCommand.h>
#include <intera_core_msgs/SolvePositionFK.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_listener.h>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

#include <kdl_parser/kdl_parser.hpp>

namespace sawyer_gazebo {
  class ArmKinematicsInterface
  {
  public:
    bool init(ros::NodeHandle& nh, std::string side);

  private:
    std::string side_, root_name_, tip_name_;
    KDL::Tree tree_;
    class Kinematics
    {
    public:
        KDL::Chain chain;
        KDL::ChainFkSolverPos_recursive* fk_pos_solver;
        KDL::ChainFkSolverVel_recursive* fk_vel_solver;
        KDL::ChainIdSolver_RNE*         gravity_solver;
        //KDL::ChainIkSolverVel_pinv* ik_solver_vel;
        //KDL::ChainIkSolverPos_NR_JL* ik_solver_pos;
    };
    std::map<std::string, Kinematics> kinematic_chain_map_;
    tf::TransformListener tf_listener_;

    realtime_tools::RealtimeBox< std::shared_ptr<const intera_core_msgs::JointCommand> > joint_command_buffer_;
    realtime_tools::RealtimeBox< std::shared_ptr<const sensor_msgs::JointState> > joint_state_buffer_;

    ros::Subscriber joint_command_sub_;
    ros::Subscriber joint_state_sub_;

    ros::Publisher endpoint_state_pub_;
    ros::Publisher gravity_torques_pub_;

    ros::ServiceServer ik_service_;
    ros::ServiceServer fk_service_;

    ros::Timer update_timer_;

  protected:
    bool createKinematicChain(std::string tip_name);

    void update(const ros::TimerEvent& e);

    void jointCommandCallback(const intera_core_msgs::JointCommandConstPtr& msg);
    void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);

    void publishGravityTorques();
    void publishEndpointState();

    bool parseParams(const ros::NodeHandle& nh);

    bool FKService(intera_core_msgs::SolvePositionFK::Request& req,
                   intera_core_msgs::SolvePositionFK::Response& res);
    /* Method to calculate the FK for the required joint configuration
     *  @returns true if successful
     */
    bool computePositionFK(const Kinematics& kin, const KDL::JntArray& jnt_vel, geometry_msgs::Pose& result);

    bool computeVelocityFK(const Kinematics& kin, const KDL::JntArrayVel& jnt_vel, geometry_msgs::Twist& result);

    void jointStateToKDL(const sensor_msgs::JointState& joint_configuration, const Kinematics& kin,
                         KDL::JntArray& jnt_pos, KDL::JntArray& jnt_vel, KDL::JntArray& jnt_eff);
    /* Method to calculate the IK for the required end pose
     *  @returns true if successful
     */
    /*bool getPositionIK(const geometry_msgs::PoseStamped& pose_stamp, const sensor_msgs::JointState& seed,
                       sensor_msgs::JointState* result);*/

  };
}
#endif // #ifndef __SAWYER_GAZEBO__ARM_KINEMATICS_INTERFACE_H_

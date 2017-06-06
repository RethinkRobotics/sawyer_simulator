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

#include <sawyer_gazebo/arm_kinematics_interface.h>
#include <intera_core_msgs/SEAJointState.h>
#include <intera_core_msgs/EndpointState.h>

#include <urdf/model.h>

#include <kdl/jntarray.hpp>

#include <tf_conversions/tf_kdl.h>

namespace sawyer_gazebo {

bool ArmKinematicsInterface::init(ros::NodeHandle& nh, std::string side) {
  side_ = side;
  if(!parseParams(nh)){
    return false;
  }
  // Init Solvers to default endpoint
  if(!createKinematicChain(tip_name_)){
    return false;
  }
  gravity_torques_pub_ = nh.advertise<intera_core_msgs::SEAJointState>("limb/"+side_+"/gravity_compensation_torques", 1);
  endpoint_state_pub_ = nh.advertise<intera_core_msgs::EndpointState>("limb/"+side_+"/endpoint_state", 1);
  joint_state_sub_ = nh.subscribe("joint_states", 1,
                       &ArmKinematicsInterface::jointStateCallback, this);
  joint_command_sub_ = nh.subscribe("limb/"+side_+"/joint_command", 1,
                       &ArmKinematicsInterface::jointCommandCallback, this);
  fk_service_ = nh.advertiseService("/ExternalTools/"+side_+"/PositionKinematicsNode/FKService", &ArmKinematicsInterface::FKService, this);
  update_timer_ = nh.createTimer(100, &ArmKinematicsInterface::update, this);// 100Hz
  return true;
}

bool ArmKinematicsInterface::createKinematicChain(std::string tip_name){
  Kinematics* kin_ptr = new Kinematics();
  if(!tree_.getChain(root_name_, tip_name, kin_ptr->chain)) {
    ROS_FATAL_NAMED("kinematics", "Couldn't find chain %s to %s", root_name_.c_str(), tip_name.c_str());
    return false;
  }
  KDL::Vector gravity = KDL::Vector(0.0, 0.0, -9.8);
  kin_ptr->gravity_solver = new KDL::ChainIdSolver_RNE(kin_ptr->chain, gravity);
  kin_ptr->fk_pos_solver = new KDL::ChainFkSolverPos_recursive(kin_ptr->chain);
  kin_ptr->fk_vel_solver = new KDL::ChainFkSolverVel_recursive(kin_ptr->chain);
  //ik_solver_vel_ = new KDL::ChainIkSolverVel_pinv(kin.chain);
  kinematic_chain_map_.insert(std::make_pair(tip_name, *kin_ptr));
  return true;;
}

bool ArmKinematicsInterface::parseParams(const ros::NodeHandle& nh) {
  urdf::Model robot_model;
  std::string urdf_xml;
  ROS_DEBUG_NAMED("kinematics", "Reading xml file from parameter server");
  if (!nh.getParam("/robot_description", urdf_xml))
  {
    ROS_FATAL_NAMED("kinematics", "Could not load the xml from parameter server: %s", urdf_xml.c_str());
    return false;
  }
  if (!nh.getParam("root_name", root_name_))
  {
    ROS_FATAL_NAMED("kinematics", "No tip name for gravity found on parameter server");
    return false;
  }
  if (!nh.getParam("tip_name", tip_name_))
  {
    ROS_FATAL_NAMED("kinematics", "No tip name for gravity found on parameter server");
    return false;
  }
  robot_model.initString(urdf_xml);
  if (!kdl_parser::treeFromUrdfModel(robot_model, tree_)) {
    ROS_FATAL_NAMED("kinematics", "Failed to extract kdl tree from xml robot description.");
    return false;
  }
  return true;
}

void ArmKinematicsInterface::update(const ros::TimerEvent& e) {
  //std::shared_ptr<const sensor_msgs::JointState> joint_state;
  //joint_state_buffer_.get(joint_state);
  publishEndpointState();
}

void ArmKinematicsInterface::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg) {
  std::shared_ptr<sensor_msgs::JointState> joint_state(new sensor_msgs::JointState(*msg));
  joint_state_buffer_.set(joint_state);
}

void ArmKinematicsInterface::jointCommandCallback(const intera_core_msgs::JointCommandConstPtr& msg) {
  std::shared_ptr<intera_core_msgs::JointCommand> joint_command(new intera_core_msgs::JointCommand(*msg));
  joint_command_buffer_.set(joint_command);
}

void ArmKinematicsInterface::jointStateToKDL(const sensor_msgs::JointState& joint_state, const Kinematics& kin,
                                             KDL::JntArray& jnt_pos, KDL::JntArray& jnt_vel, KDL::JntArray& jnt_eff){
    size_t num_jnts = kin.chain.getNrOfJoints();
    size_t num_segs = kin.chain.getNrOfSegments();
    jnt_pos.resize(num_jnts);
    jnt_vel.resize(num_jnts);
    jnt_eff.resize(num_jnts);
    size_t jnt_idx = 0;
    for (size_t seg_idx = 0; seg_idx < num_segs; seg_idx++)
    {
      KDL::Joint jnt = kin.chain.getSegment(seg_idx).getJoint();
      if(jnt.getTypeName() == "None" || jnt.getTypeName() == "Unknown")
        continue;
      std::string chain_joint_name = kin.chain.getSegment(seg_idx).getJoint().getName();
      for (size_t msg_idx = 0; msg_idx < joint_state.name.size(); msg_idx++)
      {
        if (joint_state.name[msg_idx] == chain_joint_name){
          if(jnt_idx < joint_state.position.size())
            jnt_pos(jnt_idx) = joint_state.position[msg_idx];
          if(jnt_idx < joint_state.velocity.size())
            jnt_vel(jnt_idx) = joint_state.velocity[msg_idx];
          if(jnt_idx < joint_state.effort.size())
            jnt_eff(jnt_idx) = joint_state.effort[msg_idx];
          jnt_idx++;
          break;
        }
      }
    }
}

bool ArmKinematicsInterface::FKService(intera_core_msgs::SolvePositionFK::Request& req,
                                       intera_core_msgs::SolvePositionFK::Response& res){
  for(size_t i=0; i<req.configuration.size(); i++) {
    geometry_msgs::PoseStamped pose_stamp;
    res.inCollision.push_back(false);
    res.isValid.push_back(false);
    res.pose_stamp.push_back(geometry_msgs::PoseStamped());
    res.pose_stamp[i].header.stamp = ros::Time::now();
    // Try to find the kinematic chain, if not, create it
    if(kinematic_chain_map_.find(req.tip_names[i]) == kinematic_chain_map_.end()){
      if(!createKinematicChain(req.tip_names[i])){
        // If chain cannot be created, leave isValid false and move on
        continue;
      }
    }
    KDL::JntArray jnt_pos, jnt_vel, jnt_eff;
    jointStateToKDL(req.configuration[i], kinematic_chain_map_[req.tip_names[i]], jnt_pos, jnt_vel, jnt_eff);
    if(computePositionFK(kinematic_chain_map_[req.tip_names[i]], jnt_pos, res.pose_stamp[i].pose)){
      res.isValid[i]=true;
    }
  }
  return true;
}

bool ArmKinematicsInterface::computePositionFK(const Kinematics& kin,
                                               const KDL::JntArray& jnt_pos,
                                               geometry_msgs::Pose& result)
{
  KDL::Frame p_out;
  size_t num_segs = kin.chain.getNrOfSegments();
  if (kin.fk_pos_solver->JntToCart(jnt_pos, p_out, num_segs) < 0)
  {
    return false;
  }
  tf::poseKDLToMsg(p_out, result);
  return true;
}

bool ArmKinematicsInterface::computeVelocityFK(const Kinematics& kin,
                                               const KDL::JntArrayVel& jnt_vel,
                                               geometry_msgs::Twist& result)
{
  KDL::FrameVel v_out;
  int num_segs = kin.chain.getNrOfSegments();
  if (kin.fk_vel_solver->JntToCart(jnt_vel, v_out, num_segs) < 0)
  {
    return false;
  }
  tf::twistKDLToMsg(v_out.GetTwist(), result);
  return true;
}

void ArmKinematicsInterface::publishEndpointState() {
    intera_core_msgs::EndpointState endpoint_state;
    std::shared_ptr<const sensor_msgs::JointState> joint_state;
    joint_state_buffer_.get(joint_state);
    if(joint_state.get())
    {
      KDL::JntArray jnt_pos, jnt_vel, jnt_eff;
      jointStateToKDL(*joint_state.get(), kinematic_chain_map_[tip_name_], jnt_pos, jnt_vel, jnt_eff);
      endpoint_state.header.stamp = ros::Time::now();
      endpoint_state.valid = true;
      if(!computePositionFK(kinematic_chain_map_[tip_name_], jnt_pos, endpoint_state.pose)){
        endpoint_state.valid &= false;
      }
      KDL::JntArrayVel jnt_array_vel(jnt_pos, jnt_vel);
      if(!computeVelocityFK(kinematic_chain_map_[tip_name_], jnt_array_vel, endpoint_state.twist)){
        endpoint_state.valid &= false;
      }
      /*
      if(!computeEffortFK(tip_name_, *joint_state.get(), endpoint_state.wrench)){ // TODO
        endpoint_state.valid &= false;
      }
      */
      endpoint_state_pub_.publish(endpoint_state);
    }
}

}

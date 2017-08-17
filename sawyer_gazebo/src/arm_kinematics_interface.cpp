/***************************************************************************
* Copyright (c) 2017, Rethink Robotics Inc.
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

#include <memory>

#include <intera_core_msgs/SEAJointState.h>
#include <intera_core_msgs/EndpointState.h>
#include <intera_core_msgs/EndpointStates.h>
#include <geometry_msgs/PoseStamped.h>

#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>
#include <urdf/model.h>

namespace sawyer_gazebo
{

bool ArmKinematicsInterface::init(ros::NodeHandle& nh, std::string side)
{
  side_ = side;
  if (!parseParams(nh))
  {
    return false;
  }
  // Init Solvers to default endpoint
  if (!createKinematicChain(tip_name_))
  {
    return false;
  }
  // Init Solvers to default hand camera
  if (!createKinematicChain(hand_camera_name_))
  {
    return false;
  }
  gravity_torques_seq_ = 0;
  endpoint_state_seq_ = 0;
  gravity_torques_pub_ = nh.advertise<intera_core_msgs::SEAJointState>(
                            "limb/"+side_+"/gravity_compensation_torques", 1);
  endpoint_state_pub_ = nh.advertise<intera_core_msgs::EndpointState>(
                            "limb/"+side_+"/endpoint_state", 1);
  tip_state_pub_ = nh.advertise<intera_core_msgs::EndpointStates>(
                            "limb/"+side_+"/tip_states", 1);
  joint_state_sub_ = nh.subscribe("joint_states", 1,
                       &ArmKinematicsInterface::jointStateCallback, this);
  joint_command_sub_ = nh.subscribe("limb/"+side_+"/joint_command", 1,
                       &ArmKinematicsInterface::jointCommandCallback, this);
  fk_service_ = nh.advertiseService(
                    "/ExternalTools/"+side_+"/PositionKinematicsNode/FKService",
                    &ArmKinematicsInterface::servicePositionFK, this);
  ik_service_ = nh.advertiseService(
                  "/ExternalTools/"+side_+"/PositionKinematicsNode/IKService",
                  &ArmKinematicsInterface::servicePositionIK, this);
  // Update at 100Hz
  update_timer_ = nh.createTimer(100, &ArmKinematicsInterface::update, this);
  return true;
}

bool ArmKinematicsInterface::createKinematicChain(std::string tip_name)
{
  if(kinematic_chain_map_.find(tip_name) != kinematic_chain_map_.end())
  {
    ROS_WARN_NAMED("kinematics", "Kinematic chain from %s to %s already exists!",
                   root_name_.c_str(), tip_name.c_str());
    return false;
  }
  Kinematics kin;
  if (!tree_.getChain(root_name_, tip_name, kin.chain))
  {
    ROS_ERROR_NAMED("kinematics", "Couldn't find chain %s to %s",
                    root_name_.c_str(), tip_name.c_str());
    return false;
  }
  kin.gravity_solver = std::make_unique<KDL::ChainIdSolver_RNE>(kin.chain, KDL::Vector(0.0, 0.0, -9.8));
  kin.fk_pos_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(kin.chain);
  kin.fk_vel_solver = std::make_unique<KDL::ChainFkSolverVel_recursive>(kin.chain);
  kinematic_chain_map_.insert(std::make_pair(tip_name, std::move(kin)));
  return true;
}

bool ArmKinematicsInterface::parseParams(const ros::NodeHandle& nh)
{
  urdf::Model robot_model;
  std::string urdf_xml;
  ROS_DEBUG_NAMED("kinematics", "Reading xml file from parameter server");
  if (!nh.getParam("/robot_description", urdf_xml))
  {
    ROS_FATAL_NAMED("kinematics",
        "Could not load the xml from parameter server: %s", urdf_xml.c_str());
    return false;
  }
  if (!nh.getParam("root_name", root_name_))
  {
    ROS_FATAL_NAMED("kinematics",
        "No tip name for gravity found on parameter server");
    return false;
  }
  if (!nh.getParam("tip_name", tip_name_))
  {
    ROS_FATAL_NAMED("kinematics",
        "No tip name found on parameter server");
    return false;
  }
  if (!nh.getParam("hand_camera_name", hand_camera_name_))
  {
    ROS_FATAL_NAMED("kinematics",
        "No hand camera name found on parameter server");
    return false;
  }
  robot_model.initString(urdf_xml);
  if (!kdl_parser::treeFromUrdfModel(robot_model, tree_))
  {
    ROS_FATAL_NAMED("kinematics",
        "Failed to extract kdl tree from xml robot description.");
    return false;
  }
  return true;
}

void ArmKinematicsInterface::update(const ros::TimerEvent& e)
{
  publishEndpointState();
  publishGravityTorques();
}

void ArmKinematicsInterface::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  auto joint_state = std::make_shared<sensor_msgs::JointState>(*msg);
  joint_state_buffer_.set(joint_state);  // FIXME - what happens when different joint states messages are received?
}

void ArmKinematicsInterface::jointCommandCallback(const intera_core_msgs::JointCommandConstPtr& msg)
{
  auto joint_command = std::make_shared<intera_core_msgs::JointCommand>(*msg);
  joint_command_buffer_.set(joint_command);
}

void ArmKinematicsInterface::publishGravityTorques()
{
  std::shared_ptr<const sensor_msgs::JointState> joint_state;
  joint_state_buffer_.get(joint_state);
  if (joint_state.get())
  {
    intera_core_msgs::SEAJointState gravity_torques;
    auto j_state = *joint_state.get();
    auto num_jnts = kinematic_chain_map_[tip_name_].chain.getNrOfJoints();
    gravity_torques.name.resize(num_jnts);
    gravity_torques.actual_position.resize(num_jnts);
    gravity_torques.actual_velocity.resize(num_jnts);
    gravity_torques.actual_effort.resize(num_jnts);
    gravity_torques.gravity_only.resize(num_jnts);
    // FIXME(imcmahon): Populate below fields
    gravity_torques.commanded_position.resize(num_jnts);
    gravity_torques.commanded_velocity.resize(num_jnts);
    gravity_torques.commanded_acceleration.resize(num_jnts);
    gravity_torques.commanded_effort.resize(num_jnts);
    gravity_torques.gravity_model_effort.resize(num_jnts);
    gravity_torques.interaction_torque.resize(num_jnts);
    gravity_torques.hysteresis_model_effort.resize(num_jnts);
    gravity_torques.crosstalk_model_effort.resize(num_jnts);
    // FIXME(imcmahon): Populate above fields
    KDL::JntArray jnt_pos, jnt_vel, jnt_eff, jnt_torques;
    jointStateToKDL(j_state, kinematic_chain_map_[tip_name_].chain, jnt_pos, jnt_vel, jnt_eff, gravity_torques.name);
    computeGravityFK(kinematic_chain_map_[tip_name_], jnt_pos, jnt_vel, jnt_torques);
    for (size_t jnt_idx = 0; jnt_idx < num_jnts; jnt_idx++)
    {
      gravity_torques.actual_position[jnt_idx] = jnt_pos(jnt_idx);
      gravity_torques.actual_velocity[jnt_idx] = jnt_vel(jnt_idx);
      gravity_torques.actual_effort[jnt_idx] = jnt_eff(jnt_idx);
      gravity_torques.gravity_only[jnt_idx] = jnt_torques(jnt_idx);
    }
    gravity_torques.header.frame_id = root_name_;
    gravity_torques_seq_++;
    gravity_torques.header.seq = gravity_torques_seq_;
    gravity_torques.header.stamp = ros::Time::now();
    gravity_torques_pub_.publish(gravity_torques);
  }
}

void ArmKinematicsInterface::jointStateToKDL(const sensor_msgs::JointState& joint_state, const KDL::Chain& chain,
                                             KDL::JntArray& jnt_pos, KDL::JntArray& jnt_vel, KDL::JntArray& jnt_eff,
                                             std::vector<std::string>& jnt_names)
{
  auto num_jnts = chain.getNrOfJoints();
  auto num_segs = chain.getNrOfSegments();
  auto num_msg = joint_state.name.size();
  jnt_pos.resize(num_jnts);
  jnt_vel.resize(num_jnts);
  jnt_eff.resize(num_jnts);
  jnt_names.resize(num_jnts);
  decltype(num_jnts) jnt_idx = 0;
  for (decltype(num_segs) seg_idx = 0; seg_idx < num_segs; seg_idx++)
  {
    const auto& jnt = chain.getSegment(seg_idx).getJoint();
    if (jnt.getTypeName() == "None" || jnt.getTypeName() == "Unknown")
      continue;
    jnt_names[jnt_idx] = chain.getSegment(seg_idx).getJoint().getName();
    for (decltype(num_msg) msg_idx = 0; msg_idx < num_msg; msg_idx++)
    {
      if (jnt_idx < num_jnts && joint_state.name[msg_idx] == jnt_names[jnt_idx])
      {
        if (jnt_idx < joint_state.position.size())
          jnt_pos(jnt_idx) = joint_state.position[msg_idx];
        if (jnt_idx < joint_state.velocity.size())
          jnt_vel(jnt_idx) = joint_state.velocity[msg_idx];
        if (jnt_idx < joint_state.effort.size())
          jnt_eff(jnt_idx) = joint_state.effort[msg_idx];
        jnt_idx++;
        break;
      }
    }
  }
}

bool ArmKinematicsInterface::servicePositionIK(intera_core_msgs::SolvePositionIK::Request& req,
                                       intera_core_msgs::SolvePositionIK::Response& res)
{ /*
  auto req_size = req.pose_stamp.size();
  res.joints.resize(req_size, sensor_msgs::JointState());
  res.result_type.resize(req_size, intera_core_msgs::SolvePositionIK::IK_FAILED);
  for (size_t i = 0; i < req_size; i++)
  {
    res.joints[i].header.stamp = ros::Time::now();
    // Try to find the kinematic chain, if not, create it
    if (kinematic_chain_map_.find(req.tip_names[i]) == kinematic_chain_map_.end() &&
        !createKinematicChain(req.tip_names[i]))
    {
        // If chain is not found and cannot be created, leave isValid false and move on
        res.result_type = intera_core_msgs::SolvePositionIK::IK_ENDPOINT_DOES_NOT_EXIST;
        continue;
    }
    if (req.use_nullspace_goal.size() > i && req.use_nullspace_goal[i] ){
      // use req.nullspace_goal
    }
    KDL::JntArray jnt_pos, jnt_vel, jnt_eff;
    std::vector<std::string> jnt_names;
    jointStateToKDL(req.configuration[i], kinematic_chain_map_[req.tip_names[i]].chain, jnt_pos, jnt_vel, jnt_eff, jnt_names);
    if (computePositionFK(kinematic_chain_map_[req.tip_names[i]], jnt_pos, res.pose_stamp[i].pose))
    {
      res.isValid[i] = true;
    }
}*/
  return true;
}

bool ArmKinematicsInterface::servicePositionFK(intera_core_msgs::SolvePositionFK::Request& req,
                                       intera_core_msgs::SolvePositionFK::Response& res)
{
  auto req_size = req.configuration.size();
  res.inCollision.resize(req_size, false);
  res.isValid.resize(req_size, false);
  res.pose_stamp.resize(req_size, geometry_msgs::PoseStamped());
  for (size_t i = 0; i < req_size; i++)
  {
    res.pose_stamp[i].header.stamp = ros::Time::now();
    // Try to find the kinematic chain, if not, create it
    if (kinematic_chain_map_.find(req.tip_names[i]) == kinematic_chain_map_.end() &&
        !createKinematicChain(req.tip_names[i]))
    {
        // If chain is not found and cannot be created, leave isValid false and move on
        continue;
    }
    KDL::JntArray jnt_pos, jnt_vel, jnt_eff;
    std::vector<std::string> jnt_names;
    jointStateToKDL(req.configuration[i], kinematic_chain_map_[req.tip_names[i]].chain, jnt_pos, jnt_vel, jnt_eff, jnt_names);
    if (computePositionFK(kinematic_chain_map_[req.tip_names[i]], jnt_pos, res.pose_stamp[i].pose))
    {
      res.isValid[i] = true;
    }
  }
  return true;
}

bool ArmKinematicsInterface::computeGravityFK(const Kinematics& kin,
                                              const KDL::JntArray& jnt_pos,
                                              const KDL::JntArray& jnt_vel,
                                              KDL::JntArray& jnt_torques)
{
  KDL::JntArray jnt_accel;
  jnt_accel.resize(kin.chain.getNrOfJoints());

  KDL::JntArray zero(kin.chain.getNrOfJoints());
  std::vector<KDL::Wrench> f_ext(kin.chain.getNrOfSegments(), KDL::Wrench::Zero());
  jnt_torques.resize(kin.chain.getNrOfJoints());

  if (kin.gravity_solver->CartToJnt(jnt_pos, zero, zero, f_ext, jnt_torques) < 0)
  {
    return false;
  }
  return true;
}

bool ArmKinematicsInterface::computePositionFK(const Kinematics& kin,
                                               const KDL::JntArray& jnt_pos,
                                               geometry_msgs::Pose& result)
{
  KDL::Frame p_out;
  if (kin.fk_pos_solver->JntToCart(jnt_pos, p_out, kin.chain.getNrOfSegments()) < 0)
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
  if (kin.fk_vel_solver->JntToCart(jnt_vel, v_out, kin.chain.getNrOfSegments()) < 0)
  {
    return false;
  }
  tf::twistKDLToMsg(v_out.GetTwist(), result);
  return true;
}
/* TODO(imcmahon): once ChainFDSolverTau is upstreamed
bool ArmKinematicsInterface::computeEffortFK(const Kinematics& kin,
                                             const KDL::JntArray& jnt_pos,
                                             const KDL::JntArray& jnt_eff,
                                             geometry_msgs::Wrench& result)
{
  KDL::Wrench wrench;
  if (kin.fk_eff_solver->JntToCart(jnt_pos, jnt_eff, wrench) < 0)
  {
    return false;
  }
  tf::wrenchKDLToMsg(wrench, result);
  return true;
} */

void ArmKinematicsInterface::publishEndpointState()
{
  std::shared_ptr<const sensor_msgs::JointState> joint_state;
  joint_state_buffer_.get(joint_state);
  if (joint_state.get())
  {
    intera_core_msgs::EndpointStates endpoint_states;
    const auto frames = {tip_name_, hand_camera_name_};
    for(const auto& frame : frames)
    {
      intera_core_msgs::EndpointState endpoint_state;
      endpoint_state.valid = true;
      KDL::JntArray jnt_pos, jnt_vel, jnt_eff;
      std::vector<std::string> jnt_names;
      jointStateToKDL(*joint_state.get(), kinematic_chain_map_[frame].chain, jnt_pos, jnt_vel, jnt_eff, jnt_names);
      if (!computePositionFK(kinematic_chain_map_[frame], jnt_pos, endpoint_state.pose))
      {
        endpoint_state.valid &= false;
      }
      KDL::JntArrayVel jnt_array_vel(jnt_pos, jnt_vel);
      if (!computeVelocityFK(kinematic_chain_map_[frame], jnt_array_vel, endpoint_state.twist))
      {
        endpoint_state.valid &= false;
      }
      /* TODO(imcmahon) once ChainFDSolverTau is upstreamed
      if(!computeEffortFK(kinematic_chain_map_[tip_name_], jnt_pos, jnt_eff, endpoint_state.wrench)){
        endpoint_state.valid &= false;
      }
      */
      endpoint_states.names.push_back(frame);
      endpoint_states.states.push_back(endpoint_state);
      if(frame == tip_name_)
      {
        endpoint_state.header.frame_id = root_name_;
        endpoint_state.header.stamp = ros::Time::now();
        endpoint_state_pub_.publish(endpoint_state);
      }
    }
    endpoint_states.header.frame_id = root_name_;
    endpoint_states.header.stamp = ros::Time::now();
    tip_state_pub_.publish(endpoint_states);
  }
}

}  // namespace sawyer_gazebo

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

#include <sawyer_sim_controllers/sawyer_gravity_controller.h>
#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <urdf/model.h>

namespace sawyer_sim_controllers {

  bool SawyerGravityController::init(sawyer_hardware_interface::SharedJointInterface* hw, ros::NodeHandle &n){
    // TODO: use constant, don't hardcode ctrl_subtype ("GRAVITY_COMPENSATION")
    if(!sawyer_sim_controllers::JointArrayController<sawyer_effort_controllers::JointEffortController>::init(hw, n, "GRAVITY_COMPENSATION")) {
      return false;
    }
    std::string command_topic;
    if (n.getParam("command_topic", command_topic)) {
      ros::NodeHandle nh("~");
      sub_joint_command_ = nh.subscribe(command_topic, 1, &SawyerGravityController::jointStateCallback, this);
    } else {
      sub_joint_command_ = n.subscribe("gravity_command", 1, &SawyerGravityController::jointStateCallback, this);
    }
    std::string disable_topic;
    if (n.getParam("disable_topic", disable_topic)) {
      ros::NodeHandle nh("~");
      sub_gravity_disable_ = nh.subscribe(disable_topic, 1, &SawyerGravityController::gravityDisableCB, this);
    } else {
      sub_gravity_disable_ = n.subscribe("gravity_disable", 1, &SawyerGravityController::gravityDisableCB, this);
    }

    if(!parseParams(n))
      return false;
    inverse_dynamics_solver_ = std::make_unique<KDL::ChainIdSolver_RNE>(chain_, KDL::Vector(0.0, 0.0, -9.8));

    // In order to disable the gravity compensation torques,
    // an empty message should be published at a frequency greater than (1/disable_timeout) Hz
    double disable_timeout;
    n.param<double>("disable_timeout", disable_timeout, 0.2);
    gravity_disable_timeout_ = ros::Duration(disable_timeout);

    KDL::JntArray zero(chain_.getNrOfJoints());
    auto p_current_velocity = std::make_shared<LastVelocity>();
    p_current_velocity->time = ros::Time::now();
    p_current_velocity->velocity = zero;
    box_last_velocity_.set(p_current_velocity);

    return true;
  }

  bool SawyerGravityController::parseParams(const ros::NodeHandle& nh)
  {
    std::string urdf_xml, root_name, tip_name;
    ROS_DEBUG_NAMED("GRAVITY_CONTROLLER", "Reading xml file from parameter server");
    if (!nh.getParam("/robot_description", urdf_xml))
    {
      ROS_FATAL_NAMED("GRAVITY_CONTROLLER",
          "Could not load the xml from parameter server: %s", urdf_xml.c_str());
      return false;
    }
    if (!nh.getParam("links/root", root_name))
    {
      ROS_FATAL_NAMED("GRAVITY_CONTROLLER",
          "No root name for Kinematic Chain found on parameter server");
      return false;
    }
    if (!nh.getParam("links/tip", tip_name))
    {
      ROS_FATAL_NAMED("GRAVITY_CONTROLLER",
          "No tip name for Kinematic Chain found on parameter server");
      return false;
    }
    urdf::Model robot_model;
    KDL::Tree tree;
    robot_model.initString(urdf_xml);
    if (!kdl_parser::treeFromUrdfModel(robot_model, tree))
    {
      ROS_FATAL_NAMED("GRAVITY_CONTROLLER",
          "Failed to extract kdl tree from xml robot description.");
      return false;
    }
    if (!tree.getChain(root_name, tip_name, chain_))
    {
      ROS_ERROR_NAMED("GRAVITY_CONTROLLER", "Couldn't find chain %s to %s",
                      root_name.c_str(), tip_name.c_str());
      return false;
    }
    // Save off Joint Names
    for (size_t seg_idx = 0; seg_idx < chain_.getNrOfSegments(); seg_idx++)
    {
      const auto& jnt = chain_.getSegment(seg_idx).getJoint();
      if (jnt.getTypeName() == "None" || jnt.getTypeName() == "Unknown")
        continue;
      joint_names_.push_back(chain_.getSegment(seg_idx).getJoint().getName());
    }
    return true;
  }

  void SawyerGravityController::jointStateToKDL(const sensor_msgs::JointState& joint_state, const std::vector<std::string>& joint_names,
                                                KDL::JntArray& jnt_pos, KDL::JntArray& jnt_vel, KDL::JntArray& jnt_eff)
  {
    auto num_jnts = joint_names.size();
    auto num_msg = joint_state.name.size();
    // Check to see if there are any values before allocating space
    if (!joint_state.position.empty())
      jnt_pos.resize(num_jnts);
    if (!joint_state.velocity.empty())
      jnt_vel.resize(num_jnts);
    if (!joint_state.effort.empty())
      jnt_eff.resize(num_jnts);
    for(size_t jnt_idx = 0; jnt_idx < num_jnts; jnt_idx++)
    {
      for (size_t msg_idx = 0; msg_idx < num_msg; msg_idx++)
      {
        if (joint_state.name[msg_idx] == joint_names[jnt_idx])
        {
          if (msg_idx < joint_state.position.size())
            jnt_pos(jnt_idx) = joint_state.position[msg_idx];
          if (msg_idx < joint_state.velocity.size())
            jnt_vel(jnt_idx) = joint_state.velocity[msg_idx];
          if (msg_idx < joint_state.effort.size())
            jnt_eff(jnt_idx) = joint_state.effort[msg_idx];
          break;
        }
      }
    }
  }

  void SawyerGravityController::gravityDisableCB(const std_msgs::Empty& msg) {
      auto p_disable_msg_time = std::make_shared<ros::Time>(ros::Time::now());
      box_disable_time_.set(p_disable_msg_time);
      ROS_INFO_STREAM_THROTTLE(60, "Gravity compensation torques are disabled...");
  }

  bool SawyerGravityController::computeInverseDynamicsTorques(const KDL::JntArray& jnt_pos,
                                                      const KDL::JntArray& jnt_vel,
                                                      const KDL::JntArray& jnt_accel,
                                                      KDL::JntArray& jnt_torques)
  {
    std::vector<KDL::Wrench> f_ext(chain_.getNrOfSegments(), KDL::Wrench::Zero());
    jnt_torques.resize(chain_.getNrOfJoints());
    return !(inverse_dynamics_solver_->CartToJnt(jnt_pos, jnt_vel, jnt_accel, f_ext, jnt_torques) < 0);
  }

  void SawyerGravityController::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
  { // FIXME: what happens when the joint state msg doesn't contain all required joints? or different order?
    ros::Time current_time = ros::Time::now();
    KDL::JntArray jnt_pos, jnt_vel, jnt_effort, jnt_accel, jnt_id_torque;
    std::shared_ptr<const LastVelocity> p_last_velocity;
    box_last_velocity_.get(p_last_velocity);
    jointStateToKDL(*msg, joint_names_, jnt_pos, jnt_vel, jnt_effort);
    jnt_accel.resize(joint_names_.size());
    double dt = (current_time - p_last_velocity->time).toSec();
    for (int i = 0; i < joint_names_.size(); i++) {
      jnt_accel(i) = (jnt_vel(i) - p_last_velocity->velocity(i)) / dt;
    }
    if(computeInverseDynamicsTorques(jnt_pos, jnt_vel, jnt_accel, jnt_id_torque)){
      std::vector<Command> commands;
      std::shared_ptr<const ros::Time>  p_disable_msg_time;
      box_disable_time_.get(p_disable_msg_time);
      bool enable_gravity = !p_disable_msg_time || ((ros::Time::now() - *p_disable_msg_time.get()) > gravity_disable_timeout_);
      for (int i = 0; i < joint_names_.size(); i++) {
        Command cmd = Command();
        cmd.name_ = joint_names_[i];
        cmd.effort_ = enable_gravity ? jnt_id_torque(i) : 0.0;
        commands.push_back(cmd);
      }
      auto p_current_velocity = std::make_shared<LastVelocity>();
      p_current_velocity->time = current_time;
      p_current_velocity->velocity = jnt_vel;
      box_last_velocity_.set(p_current_velocity);
      command_buffer_.writeFromNonRT(commands);
      new_command_ = true;
    }
  }

  void SawyerGravityController::setCommands()
  {
    // set the new commands for each controller
    std::vector<Command> command = *(command_buffer_.readFromRT());
    for (auto it = command.begin(); it != command.end(); it++) {
      controllers_[it->name_]->command_buffer_.writeFromNonRT(it->effort_);
    }
  }
}
PLUGINLIB_EXPORT_CLASS(sawyer_sim_controllers::SawyerGravityController, controller_interface::ControllerBase)

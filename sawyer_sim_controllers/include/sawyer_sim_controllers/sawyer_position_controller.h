#ifndef SAWYER_POSITION_CONTROLLER_H
#define SAWYER_POSITION_CONTROLLER_H

#include <sawyer_sim_controllers/joint_group_position_controller.h>
#include <intera_core_msgs/JointCommand.h>
#include <ros/node_handle.h>

#include <control_toolbox/pid.h>

namespace sawyer_sim_controllers
{
    class SawyerPositionController : public sawyer_sim_controllers::JointGroupPositionController
    {
    public:
        virtual ~SawyerPositionController() {sub_joint_command_.shutdown();}
        virtual bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n);

    private:
        ros::Subscriber sub_joint_command_;

        void jointCommandCB(const intera_core_msgs::JointCommandConstPtr& msg);
    };
}

#endif
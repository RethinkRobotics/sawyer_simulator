
#include <sawyer_sim_controllers/sawyer_position_controller.h>
#include <pluginlib/class_list_macros.h>

namespace sawyer_sim_controllers {
    bool SawyerPositionController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n){
        ROS_INFO_STREAM_NAMED("sawyercontroller", "initializing position controller");
        if(!sawyer_sim_controllers::JointGroupPositionController::init(hw, n)) {
            ROS_INFO_STREAM_NAMED("sawyercontroller", "failed parent init");
            return false;
        } else {
            ROS_INFO_STREAM_NAMED("sawyercontroller", "about to subscribed");
            // TODO what is the correct topic name / type combination
            // this topic is subscribed in parent class too as array. should change name.
          std::string topic_name;
          if (n.getParam("topic", topic_name)) {
            ros::NodeHandle nh("~");
            sub_joint_command_ = nh.subscribe(topic_name, 1, &SawyerPositionController::jointCommandCB, this);
          } else {
            sub_joint_command_ = n.subscribe("joint_command", 1, &SawyerPositionController::jointCommandCB, this);
          }



        }
        ROS_INFO_STREAM_NAMED("sawyercontroller", "i guess init was all good");
        return true;
    }

    void SawyerPositionController::jointCommandCB(const intera_core_msgs::JointCommandConstPtr& msg) {
        ROS_INFO_STREAM_NAMED("sawyercontroller", "Got joint command");
        std::vector<Command> commands;

        for (int i = 0; i < msg->names.size(); i++) {
          ROS_INFO_STREAM("Joint " << msg->names[i] << " going to " << msg->position[i]);
          Command cmd = Command();
          cmd.name_ = msg->names[i];
          cmd.position_ = msg->position[i];
          commands.push_back(cmd);
        }
        position_command_buffer_.writeFromNonRT(commands);
        new_command_ = true;
        ROS_INFO_STREAM("jointCommandCb new command: " << new_command_);

    }

}

PLUGINLIB_EXPORT_CLASS(sawyer_sim_controllers::SawyerPositionController, controller_interface::ControllerBase)

#include <effort_controllers/joint_position_controller.h>

namespace sawyer_sim_controllers {
    typedef std::map<std::string, boost::shared_ptr<effort_controllers::JointPositionController> > JointPositionControllerMap;

    class JointGroupPositionController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
        JointGroupPositionController();
        ~JointGroupPositionController();

        /**
         * Store joint commands in a generic format that is compatible with JointPositionController
         */
        struct Command {
            double position_;
            double velocity_;
            bool has_velocity_;
            std::string name_;
        };

        bool init(hardware_interface::EffortJointInterface* robot, ros::NodeHandle& nh);
        void starting(const ros::Time& time);
        void stopping(const ros::Time& time);
        void update(const ros::Time& time, const ros::Duration& period);

        size_t n_joints_;
        bool new_command_;

        realtime_tools::RealtimeBuffer<std::vector<Command> > position_command_buffer_;
        JointPositionControllerMap position_controllers_;
    };
}
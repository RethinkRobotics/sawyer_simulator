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
5.3.0 (2018-8-14)
---------------------------------
- Updates the how the RobotAssemblyState message is handled for ESTOP messages

5.2.0 Patch (2018-4-16)
---------------------------------
- Patch release
- Fixes Gravity Compensation Torque Calculation
- Adds Coriolis and Inertial torques to gravity_model_effort
- Adds new URDF Xacro Args (removable pedestal, fixed pose in Gazebo)
- Adds new roslaunch args for mobile Sawyer possibilities
- Retuned Position and Velocity controllers for more stable operation

5.2.0 (2018-3-14)
---------------------------------
- Initial release of Sawyer-Gazebo integration
- Adds controllers (Position, Velocity, Torque) to move robot
- Adds Gravity Compensation Controller
- Adds Kinematics (forward, inverse, gravity) Services
- Adds Head and Electric Gripper controllers
- Adds ESTOP emulation, along with simulated hardware brakes
- Adds Camera, basic IO, and Head Display emulation
- Adds Pick and Place example (as displayed in README gif)


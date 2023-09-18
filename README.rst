sawyer_simulator
================

Gazebo simulation with emulated interfaces for the Sawyer Research Robot. Please follow the tutorial_ on the Rethink Robotics Wiki to get started with Sawyer in Gazebo.

.. _tutorial: https://support.rethinkrobotics.com/support/solutions/articles/80000980381-gazebo-tutorial

.. image:: sawyer_gazebo/etc/sawyer_gazebo.gif

Code & Tickets
--------------

+-----------------+----------------------------------------------------------------------------------+
| Documentation   | https://support.rethinkrobotics.com/support/home                                 |
+-----------------+----------------------------------------------------------------------------------+
| Issues          | https://github.com/RethinkRobotics/sawyer_simulator/issues                       |
+-----------------+----------------------------------------------------------------------------------+
| Contributions   | https://github.com/RethinkRobotics/sawyer_simulator/blob/master/CONTRIBUTING.md  |
+-----------------+----------------------------------------------------------------------------------+

sawyer_simulator Repository Overview
------------------------------------

::

     .
     |
     +-- sawyer_simulator/              sawyer_simulator metapackage
     |
     +-- sawyer_gazebo/                 Gazebo interface for the Sawyer that loads the models into simulation
     |
     +-- sawyer_hardware_interface/     This emulates the hardware interfaces of Sawyer
     |
     +-- sawyer_sim_controllers/        Controller plugins for Sawyer
     |
     +-- sawyer_sim_examples/           Examples specific to Sawyer in Simulation
     |                                  (use intera_examples for examples that will work both in
     |                                   simulation AND the real Sawyer robot)

Other Sawyer Repositories
-------------------------
+------------------+-----------------------------------------------------+
| intera_sdk       | https://github.com/RethinkRobotics/intera_sdk       |
+------------------+-----------------------------------------------------+
| intera_commom    | https://github.com/RethinkRobotics/intera_common    |
+------------------+-----------------------------------------------------+
| sawyer_robot     | https://github.com/RethinkRobotics/sawyer_robot     |
+------------------+-----------------------------------------------------+
| sawyer_moveit    | https://github.com/RethinkRobotics/sawyer_moveit    |
+------------------+-----------------------------------------------------+

Latest Release Information
--------------------------

https://support.rethinkrobotics.com/support/solutions/folders/80000687452

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pal_robotiq_controller_configuration_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.0.14 (2023-01-10)
-------------------

0.0.13 (2022-09-29)
-------------------

0.0.12 (2022-09-23)
-------------------

0.0.11 (2022-05-04)
-------------------
* Merge branch 'almi' into 'master'
  Update the version of the mimicjoint + update the grippers for grasping
  See merge request robots/pal_robotiq_gripper!8
* Update the version of the mimicjoint + update the center of mass and inertia of the robotiq 85 gripper + adapt the force and collision in gazebo to grasp objects
* Contributors: saikishor, thomaspeyrucain

0.0.10 (2022-03-23)
-------------------

0.0.9 (2022-03-21)
------------------

0.0.8 (2022-03-18)
------------------
* Merge branch 'add_robotiq_epick_gripper' into 'master'
  Add robotiq_epick_gripper urdf
  See merge request robots/pal_robotiq_gripper!5
* Add epick grasping service and is_grasped topic
* Contributors: saikishor, thomaspeyrucain

0.0.7 (2021-11-18)
------------------
* Merge branch 'robotiq_fixes' into 'master'
  Robotiq fixes
  See merge request robots/pal_robotiq_gripper!4
* added missing roboticsgroup_gazebo_plugins gazebo plugin dependency
* Contributors: Sai Kishor Kothakota, saikishor

0.0.6 (2021-11-09)
------------------

0.0.5 (2021-09-07)
------------------

0.0.4 (2021-05-04)
------------------

0.0.3 (2021-04-21)
------------------
* more cleanup and fix the missing config folder in the install rules
* Contributors: Sai Kishor Kothakota

0.0.2 (2021-04-21)
------------------
* fixed some minor package configurations and licenses
* Contributors: Sai Kishor Kothakota

0.0.1 (2021-04-21)
------------------
* start grasp controller by default
* load PIDs based on the suffix type
* move the controller configuration to have a single package and choose based on the model
* Contributors: Sai Kishor Kothakota

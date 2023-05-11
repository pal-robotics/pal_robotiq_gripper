^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pal_robotiq_140_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.19 (2023-05-11)
-------------------
* Merge branch 'add-grasping-frames-raw' into 'master'
  Add grasping frames raw
  See merge request robots/pal_robotiq_gripper!17
* Fix the typos
* Modified the grasping frames xacro file path
* Added xacro file for grasping frames
* Contributors: Sai Kishor Kothakota, sergiacosta

0.0.18 (2023-03-23)
-------------------
* Merge branch 'mew-grasping-frames' into 'master'
  New grasping frames
  See merge request robots/pal_robotiq_gripper!15
* Replaced links by grasping frames macro
* Added Z and Y grasping frames
* Contributors: narcismiguel, sergiacosta

0.0.17 (2023-02-27)
-------------------

0.0.16 (2023-02-15)
-------------------
* Merge branch 'add_is_grasped_sim' into 'master'
  Add is_grasped topic and fix grasping service in simulation
  See merge request robots/pal_robotiq_gripper!11
* comment + fusion launchfiles + modify max contact
* Apply suggestion for naming and syntax
* Update robotiq 140 urdf to increase effort and change grasping frame joint distance
* Increase the attach steps to prevent the segfault
* Add fake grasp to 140 + comment the fake grasp
* Contributors: saikishor, thomaspeyrucain

0.0.15 (2023-01-12)
-------------------

0.0.14 (2023-01-10)
-------------------

0.0.13 (2022-09-29)
-------------------

0.0.12 (2022-09-23)
-------------------
* Merge branch 'add-grasping-frame' into 'master'
  Add grasping frame
  See merge request robots/pal_robotiq_gripper!9
* Add grasping frame
* Contributors: David ter Kuile, Jordan Palacios

0.0.11 (2022-05-04)
-------------------
* Merge branch 'almi' into 'master'
  Update the version of the mimicjoint + update the grippers for grasping
  See merge request robots/pal_robotiq_gripper!8
* Fix aborted error for robotiq 2f 140
* Update the version of the mimicjoint + update the center of mass and inertia of the robotiq 85 gripper + adapt the force and collision in gazebo to grasp objects
* Contributors: saikishor, thomaspeyrucain

0.0.10 (2022-03-23)
-------------------

0.0.9 (2022-03-21)
------------------

0.0.8 (2022-03-18)
------------------

0.0.7 (2021-11-18)
------------------

0.0.6 (2021-11-09)
------------------

0.0.5 (2021-09-07)
------------------

0.0.4 (2021-05-04)
------------------
* fix the transmission naming
* Contributors: Sai Kishor Kothakota

0.0.3 (2021-04-21)
------------------

0.0.2 (2021-04-21)
------------------
* fixed some minor package configurations and licenses
* Contributors: Sai Kishor Kothakota

0.0.1 (2021-04-21)
------------------
* Cleanup CMakeLists
* fix the black material not unique issue
* Update the gazebo material types
* update the coupling link info into the main macros for 140 model
* Fix the URDF of 140 model the mimic configuration
* added URDF of robotiq 140
* Contributors: Sai Kishor Kothakota

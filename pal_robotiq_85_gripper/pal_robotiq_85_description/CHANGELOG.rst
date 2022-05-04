^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pal_robotiq_85_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge branch 'almi' into 'master'
  Update the version of the mimicjoint + update the grippers for grasping
  See merge request robots/pal_robotiq_gripper!8
* Tweak damping parameter because of ABORTED error with play_motion
* Fix gazebo.launch for robotiq-2f-85
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
* Update inertia parameters of base link of robotiq-2f-85
* Update the gazebo material types
* update the base link visual STL
* fix the issue with the inner_finger link on wrong limits
* Update inertias of the links to a proper functional one
* more fixes+
* Update URDF model with new meshes and some changes to mimic joints naming
* Update URDF of robotiq 85
* added black material color to the coupling visual
* Move the coupling link into the macro and update links
* Update URDF from ROS Industrial repo
* updates files to use right and left naming
* added the first version of robotiq 85 gripper
* Contributors: Sai Kishor Kothakota

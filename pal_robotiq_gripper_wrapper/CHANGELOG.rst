^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pal_robotiq_gripper_wrapper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.19 (2023-05-11)
-------------------

0.0.18 (2023-03-23)
-------------------

0.0.17 (2023-02-27)
-------------------
* Merge branch 'fix_status_human' into 'master'
  Update script for python 3
  See merge request robots/pal_robotiq_gripper!16
* Fix for python
* Update script for python 3
* Contributors: Jordan Palacios, thomaspeyrucain

0.0.16 (2023-02-15)
-------------------
* Merge branch 'add_is_grasped_sim' into 'master'
  Add is_grasped topic and fix grasping service in simulation
  See merge request robots/pal_robotiq_gripper!11
* Merge branch 'release-service' into 'add_is_grasped_sim'
  Release service
  See merge request robots/pal_robotiq_gripper!14
* Fix typo
* Remove simulation arg and suffix. Delete simulation only class and use use_sim_time only
* Remove redundant lines
* add release service for epick gripper
* Add release service and ensure abstract class is compatible with python2 and 3
* create base class for gripperGrasp
* comment + fusion launchfiles + modify max contact
* Change is_grasped topic name for the epick gripper to be consistent
* Add fake grasp to 140 + comment the fake grasp
* Change the is_grasped topic name
* Add python script to the include directory
* Improve is_grasped topic in simulation
* Add is_grasped topic and fix grasping service in simulation
* Contributors: David ter Kuile, saikishor, thomaspeyrucain

0.0.15 (2023-01-12)
-------------------

0.0.14 (2023-01-10)
-------------------

0.0.13 (2022-09-29)
-------------------
* Merge branch 'fix_grasping' into 'master'
  Fix grasping
  See merge request robots/pal_robotiq_gripper!10
* Remove unnecessary loginfo + fix syntax
* Fix grasping if no object is detected
* Contributors: saikishor, thomaspeyrucain

0.0.12 (2022-09-23)
-------------------

0.0.11 (2022-05-04)
-------------------

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
* Change config files name
* Change joint name and configuration
* Add config files and take in consideration the epick gripper in the grasping
* Merge branch 'object-detection' into 'master'
  Object detection
  See merge request robots/pal_robotiq_gripper!3
* Fix grasping + add intelligence in the dynamic parameter + fix format
* Update default parameters
* Adding pressure dynamic param and removing unused param + improvements
  Removing max_position_error and rate dynamic param
  Change the default param of the timeout and closin with optimized one
  Add dynamic_reconfigure as a new dynamic_parameter
  Improve when the gripper detects a close to stay to True when moving
  to the optimised position
  Add is_grasped topic for Tiago dual on <gripper>_motor/is_grasped
* Change print to rospy.loginfo
* Fusion the two classes
* Update grasping service
* Remove bool to avoid confusion
* Fix bool and Bool
* Create topic is grasped
* feat: enable for tiago dual
* fix: report real value and not one hardcoded
* Contributors: daniellopez, saikishor, thomaspeyrucain

0.0.7 (2021-11-18)
------------------
* Merge branch 'robotiq_fixes' into 'master'
  Robotiq fixes
  See merge request robots/pal_robotiq_gripper!4
* Fix the bugs in the gripper_grasping file and added compatibility to TIAGo++
* added gripper_motor_name argument to the config files
* added missing pal_robotiq_gripper_wrapper_msgs dependency
* Contributors: Sai Kishor Kothakota, saikishor

0.0.6 (2021-11-09)
------------------
* Merge branch 'object-detection' into 'master'
  object-detection
  See merge request robots/pal_robotiq_gripper!2
* chore: MR changes printing better log info
* fix: add GripperStatus msg instead of reusing JointState msg
* feat: GripperStatus msg
* feat: publish position in mm
* fix: gripper_status_human
* WIP: feat: gripper grasp status
* Contributors: Jordan Palacios, daniellopez

0.0.5 (2021-09-07)
------------------
* Merge branch 'grasp_fix' into 'master'
  Fix grasping service
  See merge request robots/pal_robotiq_gripper!1
* Fix grasping service
* Contributors: saikishor, thomaspeyrucain

0.0.4 (2021-05-04)
------------------

0.0.3 (2021-04-21)
------------------
* more cleanup and fix the missing config folder in the install rules
* Contributors: Sai Kishor Kothakota

0.0.2 (2021-04-21)
------------------

0.0.1 (2021-04-21)
------------------
* added pal_robotiq_gripper_wrapper initial commit
* Contributors: Sai Kishor Kothakota

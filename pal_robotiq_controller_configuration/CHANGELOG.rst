^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pal_robotiq_controller_configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.0 (2024-07-31)
------------------

2.0.3 (2024-05-06)
------------------

2.0.2 (2024-03-25)
------------------

2.0.1 (2024-03-06)
------------------

2.0.0 (2024-01-17)
------------------
* updating license to apache
* linters
* fix config yaml file error
* changing the type of the controllers according to ros2
* deleting the installation of the home_gripper script
* adapting pal_robotiq_controller_configuration package to ros 2
* adapting launch files into the new launch files structure
* deleting unused home_gripper script
* adding myself as maintainer
* home_gripper ported but still need to check if necessary
* controller configuration package ported
* Cmake changes
* Contributors: Aina Irisarri

0.0.19 (2023-05-11)
-------------------

0.0.18 (2023-03-23)
-------------------

0.0.17 (2023-02-27)
-------------------

0.0.16 (2023-02-15)
-------------------
* Merge branch 'add_is_grasped_sim' into 'master'
  Add is_grasped topic and fix grasping service in simulation
  See merge request robots/pal_robotiq_gripper!11
* Merge branch 'release-service' into 'add_is_grasped_sim'
  Release service
  See merge request robots/pal_robotiq_gripper!14
* refactor simulation launch file
* Remove simulation arg and suffix. Delete simulation only class and use use_sim_time only
* comment + fusion launchfiles + modify max contact
* Contributors: David ter Kuile, saikishor, thomaspeyrucain

0.0.15 (2023-01-12)
-------------------

0.0.14 (2023-01-10)
-------------------

0.0.13 (2022-09-29)
-------------------

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

0.0.7 (2021-11-18)
------------------

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
* move the controller configuration to have a single package and choose based on the model
* Contributors: Sai Kishor Kothakota

# <center> DFRL</center>


## Introduction

This project provides an adaptive quadruped motor controller, which is based on CPGs and a novel biologically-inspired reflex mechanism with online, called distributed-force-feedback-based reflex with online learning (DFRL).  The DFRL can automatically adjust joint command offsets, produced by central pattern generators (CPG) or DMP originally, for locomotion of quadruped robots, thereby making the robots with appropriate body posture to trot stably on various terrains, including uphill, downhill and complex slopes.

The DFRL is modular and independent of specific CPG format (e.g., SO (2)-based CPG and DMP) and can directly be employed on different sized and weighted quadruped robots. To verify this point, this project implements the DFRL on both the SO(2) CPG -based control and DMP-based control. Experimental results show that the DFRL can automatically adjust the command offsets produced by SO(2) CPG-based control or DMP-based control. 

## Framework

The project is organized by three sub-folders including **controllers**, **projects**, and **vrep-simulation**. 

- **controllers** consists of the code of the control methods, including dfrl.cpp and dfrl.h
- **project** contains the main.cpp of the project, and manage the software
- **vrep-simulation** stores the simulation model which is based on VREP.  It has two quadruped robots: Lilibot and Laikago



## The CPG-based control code is in:

- controllers/genesis/genesis-ann-library/synapticPlasticityCpg.cpp
- controllers/genesis/genesis-ann-lrbrary/synapticPlasticityCpg.h

## The DMP-based control code is in:

- projects/genesis/catkin_ws/src/stbot_dfrl/scripts/stbot_dmp_node.py

## The DFRL code is in:

- controllers/genesis/genesis-ann-library/dfrl.cpp

- controllers/genesis/genesis-ann-library/dfrl.h

## The implementation of the project
### Install necessary software on Ubuntu 18.04 or later version.
- The v4_0_0 or the latest version of the CoppeliaSim is necessary to run the simulation. The CoppeliaSim provides an platform to execute the simulated Lilibot. The CoppeliaSim can be download in here https://www.coppeliarobotics.com/.
- The vortex physical engine is necessary to execute dynamical computation of the simulation. The software and its activation can be seen in this link: https://www.cm-labs.com/vortex-studio/software/vortex-studio-academic-access/

### Steps to run the simulation
- Open a terminal to run command: roscore

- Start the CoppeliaSim and open the simulated Lilibot model at vrep_simulation/lilibot/lilibot-V5-P4-E2.ttt of this project. 

- Click the run button at the toolbox of the CoppeliaSim.

- After click the button, the simulation is running. Simultaneously, the adaptive quadruped motor control (default is SO(2) CPG-based control with the DFRL) is executed to control the simulated Lilibot. In the control, the CPGs-based control module outputs basic rhythmic commands while the DFRL module produce signals to adjust the offsets of the commands, thereby maintaining the robot with proper posture according to the GRF distribution.

- The way to switch the CPG-based control and the DMP-based control is to set the start script at **projects/genesis/catkin_ws/src/stbot_dfrl/start_lilibot_vrep**. These two control methods (CPG and DMP) are excuted by the roslaunch file, stbot_dfrl.launch and stbot_dmp.launch.




## Reference

T. Sun, Z. Dai, and P. Manoonpong, Distributed-force-feeback-based reflex with online learning for adaptive quadruped motor control, Neural Networks, 2021.

If you have any questions/doubts  about how to implement this project on your computer, you are welcomed to raise issues and email to me. My email address is suntao.hn@gmail.com

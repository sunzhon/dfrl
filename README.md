# dfrl


## Introduction

This project provides a novel biologically-inspired reflex mechanism with online, called distributed-force-feedback-based reflex with online learning (DFRL), which is for adaptive quadruped motor control. The DFRL can automatically adjust joint command offsets, produced by central pattern generators (CPG) originally, for locomotion of quadruped robots, thereby making the robots with appropriate body posture to trot stably on various terrains, including uphill, downhill and complex slopes.

The DFRL is modular and independent of specific CPG format and can directly be employed on different sized and weighted quadruped robots.

## Framework

The project is organized by five sub-folders including **controllers**, **projects**, **robots**, **utils**, and **vrep-simulation**.

-**controllers** consists of the code of the control methods
-**project** contains the main.cpp of the project
-**utils** contain the fundamental library codes, i.e., artificial neural network codes.
-**vrep-simulation** stores the simulation model which is based on VREP.





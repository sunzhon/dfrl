# <center> DFRL</center>


## Introduction

This project provides an adaptive quadruped motor controller, which is based on CPGs and a novel biologically-inspired reflex mechanism with online, called distributed-force-feedback-based reflex with online learning (DFRL).  The DFRL can automatically adjust joint command offsets, produced by central pattern generators (CPG) originally, for locomotion of quadruped robots, thereby making the robots with appropriate body posture to trot stably on various terrains, including uphill, downhill and complex slopes.

The DFRL is modular and independent of specific CPG format and can directly be employed on different sized and weighted quadruped robots.

## Framework

The project is organized by five sub-folders including **controllers**, **projects**, **utils**, and **vrep-simulation**. 

- **controllers** consists of the code of the control methods, including dfrl.cpp and dfrl.h
- **project** contains the main.cpp of the project, and manage the software 
- **utils** contain the fundamental library codes, i.e., artificial neural network codes.
- **vrep-simulation** stores the simulation model which is based on VREP.  It has two quadruped robots: Lilibot and Laikago



## The CPG-based control code is in:

- controllers/genesis/genesis-ann-library/synapticPlasticityCpg.cpp
- controllers/genesis/genesis-ann-lrbrary/synapticPlasticityCpg.h



## The DFRL code is in:

- controllers/genesis/genesis-ann-library/dfrl.cpp

- controllers/genesis/genesis-ann-library/dfrl.h





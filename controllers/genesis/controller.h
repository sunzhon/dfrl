//
// Created by mat on 12/30/17.
//

#ifndef PENDULUM_CONTROLLER_H
#define PENDULUM_CONTROLLER_H

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include<pthread.h>
#include<errno.h>
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <std_msgs/Int32.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/Float32MultiArray.h"
#include "rosClass.h"
#include "modularNeuroController.h"
#include "Log.h"
//#include <utils/sim_robots/stbot/stdogbot/dogbotsensormotordefinition.h>
using namespace std;
using namespace stcontroller;
class RosClass;
class ModularNeuroController;
class Log;

namespace stcontroller{
class Controller {
public:
    Controller(int argc,char* argv[]);
    ~ Controller();
    bool run();
private:
    std::vector<float> motorValue;
    std::vector<float> sensorValue;
    std::vector<float> controlParameters;
    std::vector<float> neuroNetworkOutput;
	
    float forceLimit = 10000;
	uint8_t leg_num;
	uint8_t motor_num;
	uint8_t pose_num;
	uint8_t sensor_num;
	uint8_t param_num;

	unsigned int files_num;
    std::vector<std::string> files;

    ModularNeuroController * neuroController;    		
    RosClass * ros;
    Log* log;
private:
    pthread_t tid;
    static void *paramServiceThread(void *arg);
    void paramService();
};

}
#endif //PENDULUM_CONTROLLER_H

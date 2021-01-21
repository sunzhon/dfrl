/*
Author: sun tao
Email: suntao.hn@qq.com
Date: 2018-04-17
*/

#include "controller.h"
bool save_data = true;
using namespace stcontroller;
namespace stcontroller{

    Controller::Controller(int argc, char* argv[]){
        //1) create simRos
        ros = new RosClass(argc, argv);
        leg_num=ros->leg_num;
        motor_num=ros->motor_num;
        pose_num=ros->pose_num;
        sensor_num=ros->sensor_num;
        param_num=ros->param_num;
        //2) create modularNeuralController
        const struct stcontroller::ModularNeuroControllerConf  myControllerConf(leg_num, motor_num, pose_num, sensor_num, param_num);
        neuroController = new ModularNeuroController(myControllerConf);    
        //4) create log files
        std::string str[]={"controlfile_CPGs", "controlfile_commands", "sensorfile_joints", "parameterfile_rosparameters", "controlfile_modules"};
        std::vector<std::string> strfiles(str,str+sizeof(str)/sizeof(str[0]));
        files=strfiles;
        files_num=files.size();
        log= new Log(files);
        //3) inital motor and sensor
        motorValue.resize(motor_num);
        sensorValue.resize(sensor_num);
        controlParameters.resize(param_num);
        //4) add parameter thread
        if(pthread_create(&tid, NULL, paramServiceThread, (void *)this) != 0){
            perror("create thread fail in controller!\n");
        }

        ros->getParameters(controlParameters);
        neuroController->setParameters(controlParameters);
    }



    Controller::~Controller() {
        delete ros;
        delete neuroController;
        delete log;
    }

    void Controller::paramService(){
        if(neuroController->conf.stsetCPGS.State){
            neuroController->getParameters(controlParameters);
            ros->setParameters(controlParameters);
            neuroController->conf.stsetCPGS.State=false;
            cout<<"##----------------INFO----------------##"<<endl;
            cout<<"Controller update the control parameters to ROS parameter server"<<endl;
            cout<<"----------------------------------------"<<endl;
        }
        // controller get parameters from ROS parameter serve
        ros->getParameters(controlParameters);
        neuroController->setParameters(controlParameters);
    }

    void *Controller::paramServiceThread(void * arg){
        Controller * ptr = (Controller*) arg;
        while(!ptr->ros->terminate){
            ptr->paramService();
            sleep(1);
        }
        pthread_exit(0);
    }

    bool Controller::run() {
        if(ros::ok())
        {
            if(ros->terminate)
                return false;
            //1) read sensor value from robot for stbot controller
            ros->getSensorValue(sensorValue);
            //2) update the controller
            neuroController->step(&sensorValue[0],sensorValue.size(),&motorValue[0],motorValue.size());
            //3) Send motor position control value to robot 
            ros->setMotorValue(motorValue);
            //4) publish and plot NeuroNetwork to V-REP
            neuroController->getOutData(neuroNetworkOutput);
            ros->plotNeuroNetwork(neuroNetworkOutput);
            

            //5) save data to log
            if(save_data){
                vector<float> cpg_data;
                vector<float> commands_data;
                vector<float> sensory_data;
                vector<float> parameters_data;
                vector<float> modules_data;
                neuroController->storedData(cpg_data, commands_data, sensory_data, parameters_data, modules_data);

                log->saveData(files.at(0),cpg_data);
                log->saveData(files.at(1),commands_data);
                log->saveData(files.at(2),sensory_data);
                log->saveData(files.at(3),parameters_data);
                log->saveData(files.at(4),modules_data);
                log->step();
            }
            //8) sleep
            ros->rosSleep();
            return true;
        }

        return false;
    }

}



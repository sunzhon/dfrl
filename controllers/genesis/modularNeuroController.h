#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <assert.h>
#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <time.h>
#include "learning-tool/lowPassfilter.h"
#include "learning-tool/movingAverage.h"
#include "genesis-ann-library/manipulation.h"
#include "modularNeural.h"

namespace stcontroller{
    enum MOTIONSATGE{// motion stage
        SELF_ORGANIZATION_STAGE =1,
        PHASE_STABLE_STAGE =2
    };

    struct ModularNeuroControllerConf{
        /*
         *This conf is for set or store the configariotn of the neural control modules.
         It store the outputs of submodules of the neural control
         It set the parameters of the submodules of the neural control
         It includes all submodules of the modular neural control
         */
        //1) add parameter
        //1.0) the number of legs 
        unsigned short leg_num;
        unsigned short pose_num;
        unsigned short sensor_num;
        unsigned short motor_num;
        unsigned short param_num;

        //1.1) input of whole net
        std::vector<parameter> stJ1Input;
        std::vector<parameter> stPsnInput;
        std::vector<parameter> stVrnHipInput;
        std::vector<parameter> stVrnKneeInput;

        //1.2) MI of CPG,beta of PCPG, two parameters and CPGSType and weather or not to set SP
        parameter stCPGMi;
        parameter stCPGPGain;
        parameter stCPGPThreshold;
        parameter stPCPGBeta;
        parameter stCPGSType;
        struct{
            parameter Type;
            bool State;    
        } stsetCPGS;
        //2.3) set the MN bias
        vector<parameter> stMNBias1;
        vector<parameter> stMNBias2;
        vector<parameter> stMNBias3;

        //2) add Inspectvalue
        vector<parameter> stCPGN0;
        vector<parameter> stCPGN1;
        //2.1) plot of PCPG
        vector<parameter> stPCPGN0;
        vector<parameter> stPCPGN1;

        //2.2) plot of PSN
        vector<parameter> stPSN10;
        vector<parameter> stPSN11;

        //2.3) plot of VRN
        vector<parameter> stVRNHip;
        vector<parameter> stVRNKnee;
        //2.4) plot of VRN
        vector<parameter> stPMN0;
        vector<parameter> stPMN1;
        vector<parameter> stPMN2;

        //2.10) plot of gamma of CPG of DL
        vector<parameter> stPAOut0;
        vector<parameter> stPAOut1;
        // FFM of DL
        vector<parameter> stFAOut;
        //2.11) plot of Attitude Reflex
        vector<parameter> stReflexOutN0;
        vector<parameter> stReflexOutN1;
        vector<parameter> stReflexOutN2;
        //2.13) plot the np_grf
        vector<parameter> stNPOut;
        //2.15) the gait term of CPG
        vector<parameter> stACIOut0;
        vector<parameter> stACIOut1;
        //2.16) the adaptive sensory feedback gain
        vector<parameter> stAFGOut;
        //2.17) the forward model of GRF
        vector<parameter> stFMOut;

        //4) initial function for parameter
        ModularNeuroControllerConf(unsigned int leg_num, unsigned int motor_num, unsigned int pose_num, unsigned int sensor_num, unsigned int param_num){
            this->leg_num =leg_num;
            this->motor_num =motor_num;
            this->sensor_num =sensor_num;
            this->param_num =param_num;
            this->pose_num = pose_num;

            stJ1Input.resize(leg_num);
            stPsnInput.resize(leg_num);
            stVrnHipInput.resize(leg_num);
            stVrnKneeInput.resize(leg_num);
            stMNBias1.resize(leg_num);
            stMNBias2.resize(leg_num);
            stMNBias3.resize(leg_num);

            stCPGN0.resize(leg_num);
            stCPGN1.resize(leg_num);
            stPCPGN0.resize(leg_num);
            stPCPGN1.resize(leg_num);
            stPSN10.resize(leg_num);
            stPSN11.resize(leg_num);
            stVRNHip.resize(leg_num);
            stVRNKnee.resize(leg_num);
            stPMN0.resize(leg_num);
            stPMN1.resize(leg_num);
            stPMN2.resize(leg_num);

            stPAOut0.resize(leg_num);
            stPAOut1.resize(leg_num);
            stFAOut.resize(leg_num);
            stAFGOut.resize(leg_num);
            stFMOut.resize(leg_num);
            stACIOut0.resize(leg_num);//adaptive control input--adaptive neural communication mechanism
            stACIOut1.resize(leg_num);
            stNPOut.resize(leg_num);
            stReflexOutN0.resize(leg_num);
            stReflexOutN1.resize(leg_num);
            stReflexOutN2.resize(leg_num);

            for(unsigned int i=0;i<leg_num;i++){
                stJ1Input.at(i)=0.0;
                stPsnInput.at(i)=0.0;
                stVrnHipInput.at(i)=0.04;
                stVrnKneeInput.at(i)=0.04;

                stMNBias1.at(i)=0.0;
                stMNBias2.at(i)=0.0;
                stMNBias3.at(i)=0.0;
            }

            stCPGSType=5.0;//get CPGSType
            stCPGMi=0.0;//0.075;//CPG
            stCPGPGain=0.0;
            stCPGPThreshold=1000;
            stPCPGBeta=0.0;//PCPG
            stsetCPGS.Type=stCPGSType;// set CPGSType
            stsetCPGS.State=false;// set CPGSType

        }
    };


    class ModularNeuroController {

        private:
            //utility function to draw outputs of the neurons

            void updateData();
            void updateGUI();
            bool debug_mode;//调试模式和工作模式

        public:
            //class constructor
            ModularNeuroController( const ModularNeuroControllerConf& c );
            ModularNeuroController(int lilDogtype,bool mCPGs,bool mMuscleModelisEnabled);
            void initialize(int lilDogtype,bool mCPGs,bool mMuscleModelisEnabled);

            //class destructor
            virtual ~ModularNeuroController();

            virtual void init(int sensornumber, int motornumber);

            parameter sigmoid(parameter num){
                return 1.0 / (1.0 + exp(-num));
            }

            //perform one step
            virtual void step(const parameter*, int number_sensors, parameter*, int number_motors);
            void addInspectable();
            void addParameters();

            void getParameters(std::vector<float>& param)const;
            void setParameters(const std::vector<float>& param);
            void getOutData(std::vector<float>& data);
            /// performs one step without learning. Calulates motor commands from sensor inputs.

            /***** STOREABLE ****/
            /** stores the controller values to a given file. */
            void storedData(std::vector<float>& cpg_data, std::vector<float>& commands_data, std::vector<float>& sensory_data,
            std::vector<float>& parameters_data, std::vector<float>& modules_data);
            /** loads the controller values from a given file. */
            //virtual bool restore(FILE* f);
            //virtual bool store(FILE* f) const;
            //virtual parameter getGaitDiagram(parameter GRF);


        private:
            ModularNeural *mnc;
        private:
            CPGSTYPE CPGSType;//get CPGType from outside of controller

        protected:

            unsigned short numbersensors, numbermotors;
            //paramkey name;
            unsigned long int t;
        public:
            // external contorl parameters
            ModularNeuroControllerConf conf;
            // inputs and outputs of serves of every joints
            std::vector<parameter> x;
            std::vector<parameter> y;

            //low pass filter
            std::vector<lowPass_filter *> filterPosition;
            std::vector<lowPass_filter *> filterVelocity;
            std::vector<lowPass_filter *> filterCurrent;
            std::vector<lowPass_filter *> filterVoltage;
            std::vector<lowPass_filter *> filterPose;
            std::vector<lowPass_filter *> filterGRF;


            // post processing value of sensor value
            std::vector<parameter > JointPosition;
            std::vector<parameter > JointVelocity;
            std::vector<parameter > JointCurrent;
            std::vector<parameter > JointVoltage;
            std::vector<parameter > Pose;
            std::vector<parameter > GRForce;
    };

}
#endif

/***
Data:2017/12/19
Descriptions: the main interface of controller
Authors: sun tao
 *****/
#include "modularNeuroController.h"
using namespace matrix;
using namespace std;
bool TestIntergrative=false;
vector<float> Exp_GRF;
MOTIONSATGE MotionStage=SELF_ORGANIZATION_STAGE;// motion stage 
namespace stcontroller {
    ModularNeuroController::ModularNeuroController(const ModularNeuroControllerConf& c): conf(c) {

            //1.1) initial filter of sensory feedback
            filterGRF.resize(conf.leg_num);
            filterPose.resize(conf.pose_num); // 3 orientation and 3 position
            filterPosition.resize(conf.motor_num);
            filterVelocity.resize(conf.motor_num);
            filterCurrent.resize(conf.motor_num);
            filterVoltage.resize(conf.motor_num);
            //1.2) initial sensor value store
            GRForce.resize(conf.leg_num);//Ground reaction force
            Pose.resize(conf.pose_num); //3 orentation and 3 position dimensions
            JointPosition.resize(conf.motor_num); //12 joints
            JointVelocity.resize(conf.motor_num); //12 joints
            JointCurrent.resize(conf.motor_num); //12 joints
            JointVoltage.resize(conf.motor_num); //12 joints
            //1.3) initial ModularNeural
            initialize(2, true, false);
            init(conf.sensor_num,conf.motor_num);
            debug_mode = false;
            //1.4) new filter ,low pass filter to clean the feedback signals of the different legs,cut off frequency has been set to 0.3,
            //setting it to a frequency bigger than 0.4 results in too much sensitive adaptive oscillator's response
            for (unsigned int i = 0; i < filterGRF.size(); i++)
                filterGRF.at(i) = new lowPass_filter(0.6);
            for (unsigned int i = 0; i < filterPose.size(); i++)
                filterPose.at(i) = new lowPass_filter(0.4);
            for (unsigned int i = 0; i < filterPosition.size(); i++)
                filterPosition.at(i) = new lowPass_filter(0.4);
            for (unsigned int i = 0; i < filterVelocity.size(); i++)
                filterVelocity.at(i) = new lowPass_filter(0.4);
            for (unsigned int i = 0; i < filterCurrent.size(); i++)
                filterCurrent.at(i) = new lowPass_filter(0.7);
            for (unsigned int i = 0; i < filterVoltage.size(); i++)
                filterVoltage.at(i) = new lowPass_filter(0.7);

        }
    ModularNeuroController::~ModularNeuroController() {
        //delete mnc
        delete mnc;// modular neural control
        // delete filters
        for(std::vector<lowPass_filter *>::iterator it= filterGRF.begin();it!=filterGRF.end();it++ )
            delete *it;
        filterGRF.clear();
        for(std::vector<lowPass_filter *>::iterator it= filterPose.begin();it!=filterPose.end();it++ )
            delete *it;
        filterPose.clear();
        for(std::vector<lowPass_filter *>::iterator it= filterPosition.begin();it!=filterPosition.end();it++ )
            delete *it;
        filterPosition.clear();
        for(std::vector<lowPass_filter *>::iterator it= filterVelocity.begin();it!=filterVelocity.end();it++ )
            delete *it;
        filterVelocity.clear();
        for(std::vector<lowPass_filter *>::iterator it= filterCurrent.begin();it!=filterCurrent.end();it++ )
            delete *it;
        filterCurrent.clear();
        for(std::vector<lowPass_filter *>::iterator it= filterVoltage.begin();it!=filterVoltage.end();it++ )
            delete *it;
        filterVoltage.clear();
    }

    void ModularNeuroController::initialize(int aAMOSversion, bool mCPGs,
            bool mMuscleModelisEnabled) {
        t = 0;
        //the second variable corresponds to the number of cpgs to create,i.e. fourlegs=4
        mnc = new ModularNeural(conf.leg_num);
        std::cout<<"#------------------------------------------------#"<<std::endl;
        std::cout<<"#--Initial Modulaer Neural network successfuly !-#"<<std::endl;
        std::cout<<"#------------------------------------------------#"<<std::endl;
    }

    void ModularNeuroController::init(int sensornumber, int motornumber) {
        numbersensors = sensornumber;
        numbermotors = motornumber;
        x.resize(sensornumber);
        y.resize(motornumber);

    }

/*
    void ModularNeuroController::setParameters(const std::vector<float>& param){
        assert(param.size()==conf.param_num);
        conf.stCPGMi=param[0];
        conf.stPCPGBeta=param[1];
        conf.stCPGSType=param[2];
        for(unsigned int i=0;i<conf.leg_num;i++){
            conf.stPsnInput.at(i)=param[6*i+3];
            conf.stVrnHipInput.at(i)=param[6*i+1+3];
            conf.stVrnKneeInput.at(i)=param[6*i+2+3];

            conf.stMNBias1.at(i)=param[6*i+3+3];
            conf.stMNBias2.at(i)=param[6*i+4+3];
            conf.stMNBias3.at(i)=param[6*i+5+3];
        }

    }

    void ModularNeuroController::getParameters(std::vector<float>& param)const{
        assert(param.size()==conf.param_num);
        param[0] = conf.stCPGMi;
        param[1] = conf.stPCPGBeta;
        param[2] = conf.stCPGSType;

        for(unsigned int i=0;i<conf.leg_num;i++){
            param[6*i+3]=conf.stPsnInput.at(i);
            param[6*i+1+3]=conf.stVrnHipInput.at(i);
            param[6*i+2+3]=conf.stVrnKneeInput.at(i);

            param[6*i+3+3]=conf.stMNBias1.at(i);
            param[6*i+3+4]=conf.stMNBias2.at(i);
            param[6*i+3+5]=conf.stMNBias3.at(i);
        }

    }

*/

    void ModularNeuroController::setParameters(const std::vector<float>& param){
        assert(param.size()==conf.param_num);
        conf.stCPGSType=param[0];
        conf.stCPGMi=param[1];
        conf.stCPGPGain=param[2];
        conf.stCPGPThreshold=param[3];
        conf.stPCPGBeta=param[4];
        for(unsigned int i=0;i<conf.leg_num;i++){
            conf.stPsnInput.at(i)=param[6*i+5];
            conf.stVrnHipInput.at(i)=param[6*i+6];
            conf.stVrnKneeInput.at(i)=param[6*i+7];

            conf.stMNBias1.at(i)=param[6*i + 8]; 
            conf.stMNBias2.at(i)=param[6*i + 9]; 
            conf.stMNBias3.at(i)=param[6*i + 10];
        }
    }   

    void ModularNeuroController::getParameters(std::vector<float>& param)const{
        assert(param.size()==conf.param_num);
        param[0] = conf.stCPGSType;
        param[1] = conf.stCPGMi;
        param[2] = conf.stCPGPGain;
        param[3] = conf.stCPGPThreshold;
        param[4] = conf.stPCPGBeta;

        for(unsigned int i=0;i<conf.leg_num;i++){
            param[6*i+5]=conf.stPsnInput.at(i);
            param[6*i+6]=conf.stVrnHipInput.at(i);
            param[6*i+7]=conf.stVrnKneeInput.at(i);

            param[6*i+8]=conf.stMNBias1.at(i);
            param[6*i+9]=conf.stMNBias2.at(i);
            param[6*i+10]=conf.stMNBias3.at(i);
        }
    }  



    //implement controller here
    void ModularNeuroController::step(const parameter* x_, int number_sensors, parameter* y_, int number_motors) {
        assert(number_sensors == numbersensors);
        assert(number_motors == numbermotors);
        //0) Sensor inputs/scaling  ----------------
        for (unsigned int i = 0; i < x.size(); i++) {
            x.at(i) = x_[i];
        }
        //0.1) filter sensor value
        //0.11) filter joint angle feedback
        for (unsigned int i = 0; i < filterPosition.size(); i++){
            JointPosition.at(i) = filterPosition.at(i)->update(x.at(i));//S1R0_as
            JointVelocity.at(i) = filterVelocity.at(i)->update(x.at(i+conf.motor_num));
            JointCurrent.at(i) = filterCurrent.at(i)->update(x.at(i+2*conf.motor_num));
            JointVoltage.at(i) = filterVoltage.at(i)->update(x.at(i+3*conf.motor_num));
        }
        //0.12) filter Pose
        for (unsigned int i = 0; i < filterPose.size(); i++)
            Pose.at(i) = filterPose.at(i)->update(x.at(i+4*conf.motor_num));
        //0.13) filter ground reaction force
        for(unsigned int i = 0; i < filterGRF.size(); i++){
            GRForce.at(i) = filterGRF.at(i)->update(x.at(i+4*conf.motor_num + conf.pose_num));//filtering of the feedback signal,force signal
        }
        //-------------------------------------------------------------------------//
        //0.15) get the CPGSType from ros param
        CPGSType=(CPGSTYPE)conf.stCPGSType; 
        //1) set the simulation parameters
        //1.1) set the CPGs' parameters
        for (unsigned int i = 0; i < conf.leg_num; i++) {
            mnc->setCpgMi(i, conf.stCPGMi);//fix MI when cpgstype is SP
            mnc->setPCPGbeta(i, conf.stPCPGBeta);
            mnc->setMNBias(i,conf.stMNBias1.at(i),conf.stMNBias2.at(i),conf.stMNBias3.at(i));
        }
        for(unsigned int i =0;i<conf.leg_num;i++)
            mnc->setInputNeuronInput(i,conf.stJ1Input.at(i),conf.stPsnInput.at(i), conf.stVrnHipInput.at(i), conf.stVrnKneeInput.at(i));

        //2) set sensory feedback
        mnc->setAttituteInput(Pose);//update the attitude input
        for (unsigned int i = 0; i < conf.leg_num; i++) {
            mnc->setJointSensorAngle(i, JointPosition.at(3 * i),
                    JointPosition.at(3 * i + 1), JointPosition.at(3 * i + 2));
            mnc->setFootSensorForce(i,GRForce.at(i));//GRF feedback
        }
        //3) update steps after all sensory information are update
        for (unsigned int i = 0; i < conf.leg_num; i++) {
            mnc->step(i,CPGSType);
        }
        //4) control the joints
        for(unsigned int i=0;i<y.size();i++)
            y_[i] = mnc->getPMNOutput(i);
        //5) update the inspectvalue
        updateData();
        //6) update the time
        t++;
    }

    void ModularNeuroController::updateData() {
        /*
        utility to draw outputs of mnc
        */
        for(unsigned int i=0;i<conf.leg_num;i++){
            //1) CPG
            conf.stCPGN0.at(i) =mnc->getCpgOut0(i);
            conf.stCPGN1.at(i) =mnc->getCpgOut1(i);
            //2) PCPG
            conf.stPCPGN0.at(i) = mnc->getPcpgOutput(i, 0);
            conf.stPCPGN1.at(i) = mnc->getPcpgOutput(i, 1);
            //3) PSN
            conf.stPSN10.at(i) = mnc->getPsnOutput(i, 10);
            conf.stPSN11.at(i) = mnc->getPsnOutput(i, 11);
            //4) VRN
            conf.stVRNHip.at(i) = mnc->getHipVrnOutput(i);
            conf.stVRNKnee.at(i) = mnc->getKneeVrnOutput(i);
            //5) Reflex of a leg
            conf.stReflexOutN0.at(i)= mnc->getDFRLOutput(0);
            conf.stReflexOutN1.at(i)= mnc->getDFRLOutput(1);
            conf.stReflexOutN2.at(i)= mnc->getDFRLOutput(2);
            //6) PMN
            conf.stPMN0.at(i) = mnc->getPmnOutput(i, 0);
            conf.stPMN1.at(i) = mnc->getPmnOutput(i, 1);
            conf.stPMN2.at(i) = mnc->getPmnOutput(i, 2);
            //8) AFG adaptive feedback gain
            conf.stAFGOut.at(i) = 0.03;
            //9) FA frequency adaptation
            conf.stFAOut.at(i) = 0.0;
            //10) sensory feedback term in CPG
            conf.stPAOut0.at(i) = mnc->getSFOutput(i,0);
            conf.stPAOut1.at(i) = mnc->getSFOutput(i,1);
            //11) NP for ground reaction force
            conf.stNPOut.at(i) = mnc->getNPOutput(i);
            //12) adaptive control input term in CPG
            conf.stACIOut0.at(i)=mnc->getACIOutput(i,0);// the gait term in CPG
            conf.stACIOut1.at(i)=mnc->getACIOutput(i,1);// the gait term in CPG
            //13) forward model of AFG
            conf.stFMOut.at(i)=0;

        }
    }

    void ModularNeuroController::getOutData(std::vector<float>& data){
        /**
         * get out data for online display in Vrep simulation 
         */

        data.clear();
        for(uint8_t i=0;i<conf.leg_num;i++){
            //1) CPG
            data.push_back(conf.stCPGN0.at(i));//0
            data.push_back(conf.stCPGN1.at(i));//1
            //2) PCPG
            data.push_back(conf.stPCPGN0.at(i));//2
            data.push_back(conf.stPCPGN1.at(i));//3
            //3) PSN
            data.push_back(conf.stPSN10.at(i));//4
            data.push_back(conf.stPSN11.at(i));//5
            //4) VRN
            data.push_back(conf.stVRNHip.at(i));//6
            data.push_back(conf.stVRNKnee.at(i));//7
            //5) Reflex
            data.push_back(conf.stReflexOutN0.at(i));//8
            data.push_back(conf.stReflexOutN1.at(i));//9
            data.push_back(conf.stReflexOutN2.at(i));//10
            //7) PMN
            data.push_back(conf.stPMN0.at(i));//11
            data.push_back(conf.stPMN1.at(i));//12
            data.push_back(conf.stPMN2.at(i));//13
            //9) gait term in CPG
            data.push_back(conf.stACIOut0.at(i));//14
            data.push_back(conf.stACIOut1.at(i));//15
            //10)adaptive sensory feedback
            data.push_back(conf.stAFGOut.at(i));//16
            //11) frequency adaptation
            data.push_back(conf.stFAOut.at(i));//17
            //12) phase adaptation--sensory feedback term
            data.push_back(conf.stPAOut0.at(i));//18
            data.push_back(conf.stPAOut1.at(i));//19
            //13) formard model
            data.push_back(conf.stFMOut.at(i));//20
        }
        //10) phase and its stability
        data.push_back(GRForce.at(0));//84--RF
        data.push_back(GRForce.at(1));//85--RH
        data.push_back(GRForce.at(2));//86--LF
        data.push_back(GRForce.at(3));//87--LH
    }


    void ModularNeuroController::storedData(std::vector<float>& cpg_data, std::vector<float>& commands_data, std::vector<float>& sensory_data, std::vector<float>& parameters_data, std::vector<float>& modules_data) {
/**
 * Store data in files
 */
        //1)  CPG
        cpg_data.clear();
        for(uint8_t i=0;i<conf.leg_num;i++){
            //1) CPG
            cpg_data.push_back(conf.stCPGN0.at(i));
            cpg_data.push_back(conf.stCPGN1.at(i));
        }
        //2) commands
        commands_data.clear();
        for(uint8_t i=0;i<conf.leg_num;i++){
            commands_data.push_back(conf.stPMN0.at(i));//12
            commands_data.push_back(conf.stPMN1.at(i));//13
            commands_data.push_back(conf.stPMN2.at(i));//14
        }
        //3) sensory feedback
        sensory_data.clear();
        //joint feedback
        for(unsigned int idx=0; idx<JointPosition.size(); idx++) 
            sensory_data.push_back(JointPosition.at(idx));///117-128
        for(unsigned int idx=0; idx<JointCurrent.size(); idx++) 
            sensory_data.push_back(JointVelocity.at(idx));///117-128
        for(unsigned int idx=0; idx<JointCurrent.size(); idx++) 
            sensory_data.push_back(JointCurrent.at(idx));///117-128
        for(unsigned int idx=0; idx<JointVoltage.size(); idx++) 
            sensory_data.push_back(JointVoltage.at(idx));///129-140
        // oirentation
        for(unsigned int idx=0;idx<Pose.size();idx++)
            sensory_data.push_back(Pose.at(idx));
        // GRF
        for(uint8_t i=0;i<conf.leg_num;i++){
            sensory_data.push_back(GRForce.at(i));//11
        }
        //4) ros server parameters
        parameters_data.clear();
        parameters_data.push_back(conf.stCPGMi);
        parameters_data.push_back(conf.stPCPGBeta);
        parameters_data.push_back(conf.stCPGSType);
        for(unsigned int i=0;i<conf.leg_num;i++){
            parameters_data.push_back(conf.stPsnInput.at(i));
            parameters_data.push_back(conf.stVrnHipInput.at(i));
            parameters_data.push_back(conf.stVrnKneeInput.at(i));

            parameters_data.push_back(conf.stMNBias1.at(i));
            parameters_data.push_back(conf.stMNBias2.at(i));
            parameters_data.push_back(conf.stMNBias3.at(i));
        }
        //5) modules
        modules_data.clear();
        modules_data.push_back(mnc->getDFRLplasticWeight(0));
        modules_data.push_back(mnc->getDFRLplasticWeight(1));
        modules_data.push_back(mnc->getDFRLOutput(0));
        modules_data.push_back(mnc->getDFRLOutput(1));
        modules_data.push_back(mnc->getDFRLOutput(2));
    }

}

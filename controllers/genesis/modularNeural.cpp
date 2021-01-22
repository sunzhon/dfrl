#include "modularNeural.h"
ModularNeural::ModularNeural(unsigned int nCPGs) {
    //1) initial
    this->n_CPGs = nCPGs;
    mLINs.resize(nCPGs);
    mCPGs.resize(nCPGs);
    mPCPGs.resize(nCPGs);
    mPSNs.resize(nCPGs);
    hip_mVRNs.resize(nCPGs);
    knee_mVRNs.resize(nCPGs);
    mPMNs.resize(nCPGs);
    mNPs.resize(nCPGs);
    reflex.resize(nCPGs);//in order to attitude control
    grf.resize(nCPGs);
    np_grf.resize(nCPGs);
    jaf.resize(nCPGs);//joint angle feedback
    jmc.resize(nCPGs);//joint motor command
    obd.resize(nCPGs);// obstacle detect
    // 2) single module for whole robot
    // Distributed force feedback based reflex with oneline leraning DFRL
        dffbReflex = new DFRL(0);
    //3) new the spinal cord  neural network
    //********CPGs+PSNs+VRNs*******//
    for (unsigned int i = 0; i < nCPGs; i++) {
        //2.0) initial
        jaf.at(i).resize(3);//3 joints
        jmc.at(i).resize(3);//3 joints
        reflex.at(i).resize(3);//3 joints to response for the reflexs
        //2.1) input neurons
        mLINs.at(i) = new stcontroller::LIN();
        //2.2) CPGs
        mCPGs.at(i) = new SPCPG(i,nCPGs);
        //2.3) PCPGs
        mPCPGs.at(i) = new DFPCPG();
        w(mPCPGs.at(i)->getNeuron(0), mCPGs.at(i)->getNeuron(0), 1.0);//PCPG0<-->hip joint
        w(mPCPGs.at(i)->getNeuron(1), mCPGs.at(i)->getNeuron(1), 1.0);//PCPG1<-->knee joint
        //2.4) PSNs
        mPSNs.at(i) = new PSN();
        //connection of input I1 with PSN
        w(mPSNs.at(i)->getNeuron(0), mLINs.at(i)->getNeuron(1), -1.0);
        w(mPSNs.at(i)->getNeuron(1), mLINs.at(i)->getNeuron(1), 1.0);

        //2.5) PCPG to PSN
        w(mPSNs.at(i)->getNeuron(2), mPCPGs.at(i)->getNeuron(0), 1.0);
        w(mPSNs.at(i)->getNeuron(5), mPCPGs.at(i)->getNeuron(0), 1.0);

        w(mPSNs.at(i)->getNeuron(3), mPCPGs.at(i)->getNeuron(1), 1.0);
        w(mPSNs.at(i)->getNeuron(4), mPCPGs.at(i)->getNeuron(1), 1.0);

        //2.6) VRNs
        hip_mVRNs.at(i) = new VRN();
        //PSN to hip VRN ,suntao add this
        w(hip_mVRNs.at(i)->getNeuron(0), mPSNs.at(i)->getNeuron(11), 1.75);
        w(hip_mVRNs.at(i)->getNeuron(1), mLINs.at(i)->getNeuron(2), 5);

        knee_mVRNs.at(i) = new VRN();
        //PSN to knee VRN ,suntao add this
        w(knee_mVRNs.at(i)->getNeuron(0), mPSNs.at(i)->getNeuron(10), 1.75);
        w(knee_mVRNs.at(i)->getNeuron(1), mLINs.at(i)->getNeuron(3), 5);
        
        //2.7) PMN
        mPMNs.at(i)=new PMN(3);//each leg have three activity joint
        w(mPMNs.at(i)->getNeuron(1),hip_mVRNs.at(i)->getNeuron(6),1.0);
        w(mPMNs.at(i)->getNeuron(2),knee_mVRNs.at(i)->getNeuron(6),1.0);
        
        //2.8) --Distributed-force-feedback-based reflex with online learing 
        w(mPMNs.at(i)->getNeuron(1),dffbReflex->getNeuron(0),1.0);// 1.0 DistributedForce feedback reflex
        w(mPMNs.at(i)->getNeuron(2),dffbReflex->getNeuron(1),1.0);// 1.0 DistributedForce feedback reflex

        //2.9) Neural preprocessing for fliter grf signals
        mNPs.at(i)=new NP();
        mNPs.at(i)->setUp(20.0,7.2,-6.0);
    }
}

ModularNeural::~ModularNeural(){
    for(std::vector<LIN *>::iterator it= mLINs.begin();it!=mLINs.end();it++ )
        delete *it;
    mLINs.clear();
    for(std::vector<SPCPG *>::iterator it= mCPGs.begin();it!=mCPGs.end();it++ )
        delete *it;
    mCPGs.clear();
    for(std::vector<DFPCPG *>::iterator it= mPCPGs.begin();it!=mPCPGs.end();it++ )
        delete *it;
    mPCPGs.clear();
    for(std::vector<PSN *>::iterator it= mPSNs.begin();it!=mPSNs.end();it++ )
        delete *it;
    mPSNs.clear();
    for(std::vector<VRN *>::iterator it= hip_mVRNs.begin();it!=hip_mVRNs.end();it++ )
        delete *it;
    hip_mVRNs.clear();
    for(std::vector<VRN *>::iterator it= knee_mVRNs.begin();it!=knee_mVRNs.end();it++ )
        delete *it;
    knee_mVRNs.clear();
    for(std::vector<PMN *>::iterator it= mPMNs.begin();it!=mPMNs.end();it++ )
        delete *it;
    mPMNs.clear();
    for(std::vector<NP *>::iterator it= mNPs.begin();it!=mNPs.end();it++ )
        delete *it;
    mNPs.clear();
    delete dffbReflex;
}

void ModularNeural::step(unsigned int ID,CPGSTYPE CPGSType) {
    assert(ID < n_CPGs );
    //0) update and set the input neuron and preprocessing neurons
    //0.1) update input neuron
    mLINs.at(ID)->step();
    //0.2) update preprocessing neuron
    mNPs.at(ID)->setInput(grf.at(ID));
    mNPs.at(ID)->step();
    np_grf.at(ID) =mNPs.at(ID)->getOutput();
    //1) update ffReflex
    if(ID==0){//single modules
        dffbReflex->setInput(grf);
        dffbReflex->step();
    }
    //2) update CPGs
    mCPGs.at(ID)->setInput(grf.at(ID)/*GRF*/, np_grf.at(ID)/*filter GRF with NP*/, ori/*body psoture*/, 0.03/*sensory feedback gain*/, getmCPGsOutputs(n_CPGs)/*CPGs ouputs*/, CPGSType, cmatrix/*connection matrix*/); //force feedback to this
    mCPGs.at(ID)->step();//Hide
    //3) update PCPG
    mPCPGs.at(ID)->step();
    // 4) update PSN
    mPSNs.at(ID)->updateActivities();
    mPSNs.at(ID)->updateOutputs();
    // 5) update VRN
    hip_mVRNs.at(ID)->updateActivities();
    hip_mVRNs.at(ID)->updateOutputs();
    knee_mVRNs.at(ID)->updateActivities();
    knee_mVRNs.at(ID)->updateOutputs();
    //6) update motor neuron
    mPMNs.at(ID)->step();	//6) update the CPG feedback
    //6.1) get the joint motor command from PMN
    jmc.at(ID).at(0)=mPMNs.at(ID)->getOutput(0);//jmc=joint motor command
    jmc.at(ID).at(1)=mPMNs.at(ID)->getOutput(1);
    jmc.at(ID).at(2)=mPMNs.at(ID)->getOutput(2);
}

// ----------------------- Input Neurons
void ModularNeural::setInputNeuronInput(unsigned int ID, float j1, float psn, float hipVrn,
        float kneeVrn) {
    assert(ID <n_CPGs);
    setInput(mLINs.at(ID)->getNeuron(0), j1);
    setInput(mLINs.at(ID)->getNeuron(1), psn);
    setInput(mLINs.at(ID)->getNeuron(2), hipVrn);
    setInput(mLINs.at(ID)->getNeuron(3), kneeVrn);
}


float ModularNeural::getInputNeuronInput(unsigned int ID, unsigned int index)const{
    assert(ID <n_CPGs);
    assert(index<3);
    return getInput(mLINs.at(ID)->getNeuron(index));
}
float ModularNeural::getInputNeuronOutput(unsigned int ID,int index)const{
    assert(ID <n_CPGs);
    assert(index<3);
    return getOutput(mLINs.at(ID)->getNeuron(index));
}
void ModularNeural::setFootSensorForce(unsigned int ID,float _grf){
    assert(ID <n_CPGs);
    grf.at(ID)=_grf;
}
void ModularNeural::setObstacleSensorForce(unsigned int ID,float _obd){
    assert(ID <n_CPGs);
    obd.at(ID)=_obd;
}
void ModularNeural::setJointSensorAngle(unsigned int ID,float _j1,float _j2,float _j3){
    // update mi input neuron
    assert(ID <n_CPGs);
    jaf.at(ID).at(0)=_j1;
    jaf.at(ID).at(1)=_j2;
    jaf.at(ID).at(2)=_j3;

}

void ModularNeural::updateJointMotor(unsigned int ID){
    assert(ID <n_CPGs);
    for(unsigned int i=0;i<3;i++)
        jmc.at(ID).at(i)=mPMNs.at(ID)->getOutput(i);
}

// -------------------CPG
float ModularNeural::getCpgOut0(unsigned int ID)const{
    assert(ID <n_CPGs);
    return mCPGs.at(ID)->getOutput(0);
}
float ModularNeural::getCpgOut1(unsigned int ID)const{
    assert(ID <n_CPGs);
    return mCPGs.at(ID)->getOutput(1);
}

void ModularNeural::setCpgMi(unsigned int ID, float mi){
    assert(ID < n_CPGs);
    mCPGs.at(ID)->setMi(mi);
}

matrixD ModularNeural::getmCPGsOutputs(unsigned int nCPGs){
    std::vector<std::vector<float>> activities;
    activities.resize(nCPGs);
    for(unsigned int i = 0;i < nCPGs;i++){
        activities.at(i).resize(2);
        for(unsigned int j = 0;j < 2;j++)
            activities.at(i).at(j) = mCPGs.at(i)->getOutput(j);//getActivity(j);
    }
    return activities;
}


//-------------------PSN
float ModularNeural::getPsnOutput(unsigned int ID, int index)const{
    assert(ID <n_CPGs);
    return mPSNs.at(ID)->getOutput(index);
}
//-------------------VRN
float ModularNeural::getHipVrnOutput(unsigned int ID)const{
    assert(ID <n_CPGs);
    return hip_mVRNs.at(ID)->getOutput(6);
}
float ModularNeural::getKneeVrnOutput(unsigned int ID)const{
    assert(ID <n_CPGs);
    return knee_mVRNs.at(ID)->getOutput(6);
}
//-------------------PCPG
float ModularNeural::getPcpgOutput(unsigned int ID, int index)const{
    assert(ID <n_CPGs);
    return mPCPGs.at(ID)->getOutput(index);
}

void ModularNeural::setPCPGbeta(unsigned int ID, float _beta){
    assert(ID <n_CPGs);
    mPCPGs.at(ID)->setBeta(_beta);
}

//----------------------------PMN
float ModularNeural::getPmnOutput(unsigned int ID,int index)const{
    assert(ID <n_CPGs);
    return mPMNs.at(ID)->getOutput(index);
}
float ModularNeural::getPMNOutput(unsigned int index)const{
    assert(index < 3*n_CPGs);
    return jmc.at(index/3).at(index%3);

}

void ModularNeural::setMNBias(unsigned int ID,float b1, float b2,float b3){
    assert(ID < n_CPGs);
    mPMNs.at(ID)->setBias(0,b1);
    mPMNs.at(ID)->setBias(1,b2);
    mPMNs.at(ID)->setBias(2,b3);

}


//-----------------------flexor and Attitude reflex---------------------------//
void ModularNeural::setAttituteInput(vector<float> ori){
    this->ori=ori;
}
//------------------------- get mNPs
float ModularNeural::getNPOutput(unsigned int ID)const{
    assert(ID<n_CPGs);
    return mNPs.at(ID)->getOutput();
}

//------------------------- get SF, snesory feedback term 
float ModularNeural::getSFOutput(unsigned int ID,unsigned int index)const{
    assert(ID<n_CPGs);
    return mCPGs.at(ID)->getSFOutput(index);
}

//------------------------- get ACI, adaptive control input term
float ModularNeural::getACIOutput(unsigned int ID,unsigned int index)const{
    assert(ID<n_CPGs);
    return mCPGs.at(ID)->getACIOutput(index);
}

void ModularNeural::setACIMatrix(Matrix cmatrix){
    this->cmatrix =cmatrix;
}

//-------------------------get Distributed Force feedback-based reflex with online learning (DFRL)
float ModularNeural::getDFRLplasticWeight(unsigned int index)const{
    assert(index<2);
    return dffbReflex->getPlasticWeight(index);
}

float ModularNeural::getDFRLOutput(unsigned int index)const{
    assert(index<3);
    return dffbReflex->getOutput(index);
}

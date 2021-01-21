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
    mAFGs.resize(nCPGs);
    mFAs.resize(nCPGs);
    mVestiReflexs.resize(nCPGs);
    mTouchReflexs.resize(nCPGs);
    mFlexorReflexs.resize(nCPGs);
    mExtensorReflexs.resize(nCPGs);
    mLocalReflexs.resize(nCPGs);
    mNPs.resize(nCPGs);
    mFTNNs.resize(nCPGs);
    mIkineNNs.resize(nCPGs);
    reflex.resize(nCPGs);//in order to attitude control
    grf.resize(nCPGs);
    np_grf.resize(nCPGs);
    jaf.resize(nCPGs);//joint angle feedback
    jmc.resize(nCPGs);//joint motor command
    obd.resize(nCPGs);// obstacle detect
    mDelays.resize(nCPGs);//delay the cpg signals
    esnFMs.resize(nCPGs);
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
        
        //2.6-1) mFTNNs,foot trajectory
        mFTNNs.at(i) = new FTNN(i);
        //VRN to FTNN,suntao add this
        w(mFTNNs.at(i)->getNeuron(0),hip_mVRNs.at(i)->getNeuron(6),1.0);
        w(mFTNNs.at(i)->getNeuron(1),knee_mVRNs.at(i)->getNeuron(6),1.0);
        //2.6-2) mIkineNNs , inverse kinematics
        mIkineNNs.at(i) = new IkineNN();
        //FTNN to IkineNN,suntao add this
        w(mIkineNNs.at(i)->getNeuron(0),mFTNNs.at(i)->getNeuron(0),1.0);
        w(mIkineNNs.at(i)->getNeuron(1),mFTNNs.at(i)->getNeuron(1),1.0);

        /*Reflexes*/

        //2.8.0) new attitude reflex model to replace the mATFs
        mVestiReflexs.at(i)=new stcontroller::VestibularReflex(i);
        //2.8.1) flexor reflex to negotiate obstacle
        mFlexorReflexs.at(i)=new stcontroller::FlexorReflex();
        //2.8.2) extensor reflex to negotiate obstacle
        mExtensorReflexs.at(i)=new stcontroller::ExtensorReflex(i);
        //2.8.3) local reflex including searching and elevation reflexes to negotiate obstacle
        mLocalReflexs.at(i)=new stcontroller::LocalReflex(i);
        //2.8.4) touch reflex for adjusting the amplitudes of the knee joints
        mTouchReflexs.at(i)=new stcontroller::TouchReflex(i);

        //2.9) PMN
        mPMNs.at(i)=new PMN(3);//each leg have three activity joint
        w(mPMNs.at(i)->getNeuron(1),hip_mVRNs.at(i)->getNeuron(6),1.0);
        w(mPMNs.at(i)->getNeuron(2),knee_mVRNs.at(i)->getNeuron(6),1.0);
        //--vestibular reflexes
        //w(mPMNs.at(i)->getNeuron(1),mVestiReflexs.at(i)->getNeuron(1),1.0);//1.0 Vestibular        
        //w(mPMNs.at(i)->getNeuron(2),mVestiReflexs.at(i)->getNeuron(2),1.0);//1.0 Vestibular
        //--Distributed force feedback based reflex with esn and learing 
        w(mPMNs.at(i)->getNeuron(1),dffbReflex->getNeuron(0),1.0);// 1.0 DistributedForce feedback reflex
        w(mPMNs.at(i)->getNeuron(2),dffbReflex->getNeuron(1),1.0);// 1.0 DistributedForce feedback reflex


        //2.10) adaptation
        //adaptive feedback gain
        mAFGs.at(i)=new AdaptiveFeedbackGain(i);
        //frequency adaptation
        mFAs.at(i)=new FrequencyAdaptation(i);
        //2.11) Neural preprocessing for fliter grf signals
        mNPs.at(i)=new NP();
        mNPs.at(i)->setUp(20.0,7.2,-6.0);

        //2.12) Forward model based ESN
        esnFMs.at(i)= new ESNForwardmodel(i);
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
    for(std::vector<AdaptiveFeedbackGain *>::iterator it= mAFGs.begin();it!=mAFGs.end();it++ )
        delete *it;
    mAFGs.clear();
    for(std::vector<FrequencyAdaptation *>::iterator it= mFAs.begin();it!=mFAs.end();it++ )
        delete *it;
    mFAs.clear();
    for(std::vector<stcontroller::VestibularReflex *>::iterator it= mVestiReflexs.begin();it!=mVestiReflexs.end();it++ )
        delete *it;
    mVestiReflexs.clear();
    for(std::vector<NP *>::iterator it= mNPs.begin();it!=mNPs.end();it++ )
        delete *it;
    mNPs.clear();
    for(std::vector<stcontroller::FlexorReflex *>::iterator it= mFlexorReflexs.begin();it!=mFlexorReflexs.end();it++ )
        delete *it;
    mFlexorReflexs.clear();
    for(std::vector<stcontroller::ExtensorReflex *>::iterator it= mExtensorReflexs.begin();it!=mExtensorReflexs.end();it++ )
        delete *it;
    mExtensorReflexs.clear();
    for(std::vector<stcontroller::LocalReflex *>::iterator it= mLocalReflexs.begin();it!=mLocalReflexs.end();it++ )
        delete *it;
    mLocalReflexs.clear();
    for(std::vector<FTNN *>::iterator it= mFTNNs.begin();it!=mFTNNs.end();it++ )
        delete *it;
    mFTNNs.clear();
    for(std::vector<IkineNN *>::iterator it= mIkineNNs.begin();it!=mIkineNNs.end();it++ )
        delete *it;
    mIkineNNs.clear();
    for(std::vector<stcontroller::TouchReflex*>::iterator it= mTouchReflexs.begin();it!=mTouchReflexs.end();it++ )
        delete *it;
    mTouchReflexs.clear();
    for(std::vector<ESNForwardmodel*>::iterator it= esnFMs.begin();it!=esnFMs.end();it++ )
        delete *it;
    esnFMs.clear();
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
    //1) update reflex
    //1.1) update vestibular reflex
        mVestiReflexs.at(ID)->setInput(ori,np_grf.at(ID));
        mVestiReflexs.at(ID)->step();
    //1.2) update flexor reflex
    //mFlexorReflexs.at(ID)->setInput(obd.at(ID));
    //mFlexorReflexs.at(ID)->step();
    //1.3) update extensor reflex
    //mExtensorReflexs.at(ID)->setInput(mFlexorReflexs.at(0)->getOutput(2),mFlexorReflexs.at(1)->getOutput(2),mFlexorReflexs.at(2)->getOutput(2),mFlexorReflexs.at(3)->getOutput(2),np_grf.at(ID));
    //mExtensorReflexs.at(ID)->step();
    //1.4) update Local reflex
    //mLocalReflexs.at(ID)->setInput(np_grf.at(ID),jmc.at(ID).at(2));
    //mLocalReflexs.at(ID)->step();
    //1.6) update Touch reflex
    //mTouchReflexs.at(ID)->setInput(np_grf.at(ID),mCPGs.at(ID)->getOutput(0));// np_grf. np_grf is like a bool, use CPG out, not jmc, since jmc has skip due to reflexes
    //mTouchReflexs.at(ID)->step();
    //1.5) update ffReflex
    if(ID==0){//single modules
        dffbReflex->setInput(grf);
        dffbReflex->step();
    }
    //1.7) sum reflex command
    reflex.at(ID).at(0)= mVestiReflexs.at(ID)->getOutput(0);
    reflex.at(ID).at(1)= mVestiReflexs.at(ID)->getOutput(1)+ 0.0 * mFlexorReflexs.at(ID)->getOutput(1) + 0.0*mExtensorReflexs.at(ID)->getOutput(0);
    reflex.at(ID).at(2)= mVestiReflexs.at(ID)->getOutput(2)+ 0.0 * mFlexorReflexs.at(ID)->getOutput(2) + 0.0*mExtensorReflexs.at(ID)->getOutput(1);

    //2) update CPGs
    //mCPGs.at(ID)->setMi(mFAs.at(0)->getOutput());// dou shiyong RF de MI
    mCPGs.at(ID)->setInput(grf.at(ID), np_grf.at(ID), ori, mAFGs.at(ID)->getOutput(), getmCPGsOutputs(n_CPGs), CPGSType, cmatrix); //force feedback to this
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
    //5-1) update FTNN
    mFTNNs.at(ID)->step();
    //5-2) update IkineNN
    mIkineNNs.at(ID)->step();
    //5-3) Combine reflexes into CPGs 
    //reflexesSwitch(ID);
    //6) update motor neuron
    mPMNs.at(ID)->step();	//6) update the CPG feedback
    //6.1) get the joint motor command from PMN
    jmc.at(ID).at(0)=mPMNs.at(ID)->getOutput(0);//jmc=joint motor command
    jmc.at(ID).at(1)=mPMNs.at(ID)->getOutput(1);
    jmc.at(ID).at(2)=mPMNs.at(ID)->getOutput(2);
    //7) sensory adaptation
    mAFGs.at(ID)->setInput(np_grf.at(ID),jmc.at(ID).at(2));
    mAFGs.at(ID)->step();//input x,d or,x,y
    //8) frequency adaptation
    mFAs.at(ID)->setInput(jmc.at(ID).at(1),jaf.at(ID).at(1));
    mFAs.at(ID)->step();//input x,d or,x,y

    //9) Forwardd model for predict the joint feedback based on ESN
    //esnFMs.at(ID)->setInputDataSet(jmc.at(ID),jaf.at(ID));
    //esnFMs.at(ID)->step();
    //esnFMs.at(ID)->getOutput(0);

}


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
void ModularNeural::setACIMatrix(Matrix cmatrix){
    this->cmatrix =cmatrix;
}
void ModularNeural::setCpgMi(unsigned int ID, float mi){
    assert(ID < n_CPGs);
    mCPGs.at(ID)->setMi(mi);
}
void ModularNeural::setMNBias(unsigned int ID,float b1, float b2,float b3){
    assert(ID < n_CPGs);
    mPMNs.at(ID)->setBias(0,b1);
    mPMNs.at(ID)->setBias(1,b2);
    mPMNs.at(ID)->setBias(2,b3);

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
void ModularNeural::setPCPGbeta(unsigned int ID, float _beta){
    assert(ID <n_CPGs);
    mPCPGs.at(ID)->setBeta(_beta);
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
//----------------------------PMN
float ModularNeural::getPmnOutput(unsigned int ID,int index)const{
    assert(ID <n_CPGs);
    return mPMNs.at(ID)->getOutput(index);
}
float ModularNeural::getPMNOutput(unsigned int index)const{
    assert(index < 3*n_CPGs);
    return jmc.at(index/3).at(index%3);

}
// ----------------Adaptive feedback gains
float ModularNeural::getAFGOutput(unsigned int ID)const{
    assert(ID <n_CPGs);
    return  mAFGs.at(ID)->getOutput();
}
float ModularNeural::getAFGfmOutput(unsigned int ID)const{
    assert(ID <n_CPGs);
    return  mAFGs.at(ID)->getFMOutput();
}


// -------------------------------Frequency adaptation
float ModularNeural::getFAOutput(unsigned int ID)const{
    assert(ID <n_CPGs);
    return mFAs.at(ID)->getOutput();
}
//-----------------------flexor and Attitude reflex---------------------------//
void ModularNeural::setAttituteInput(vector<float> ori){
    this->ori=ori;
}
float ModularNeural::getReflexes(unsigned int ID,unsigned int joint)const{
    assert(ID <n_CPGs);
    assert(joint < 3);
    return reflex.at(ID).at(joint);
}

float ModularNeural::getVestibularReflexOutput(unsigned int ID,int index)const{
    assert(ID<n_CPGs);
    return mVestiReflexs.at(ID)->getOutput(index);
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
//-------------------------get Distributed Force feedback-based reflex with online learning (DFRL)
float ModularNeural::getDFRLplasticWeight(unsigned int index)const{
    assert(index<4);
    return dffbReflex->getPlasticWeight(index);
}

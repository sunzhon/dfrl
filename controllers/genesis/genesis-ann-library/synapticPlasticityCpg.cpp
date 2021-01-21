#include "synapticPlasticityCpg.h"
using namespace std;
SPCPG::SPCPG(unsigned int ID,unsigned int nCPGs){ 
    this->ID = ID;
    this->nCPGs =nCPGs;
    setNeuronNumber(2);// CPG is tanhFunction

    MI=0.0;//modification
    updateWeights();
    b(0,0.01);
    b(1,0.01);

    setOutput(0,0.01);
    setOutput(1,0.01);

    a_t.resize(2);//all of those is vector type
    a_t1.resize(2);
    o_t.resize(2);//all of those is vector type
    // input feedback
    //----external terms
    //- connection term-
    aci = new ACI(ID,nCPGs);
    //- sensory feedback term--
    sf = new SF();
    //----internal terms
    //--phase reset term ----//
    prs =new PhaseReset();
    //--phase inhibition term --//
    pib =new PhaseInhibition();
    //--vestibular response term --//
    ves =new Vestibular(ID);
}

SPCPG::~SPCPG(){
    delete aci;
    delete sf;
    delete prs;
    delete pib;
    delete ves;
}

void SPCPG::setInput(parameter grf, parameter np_grf, vector<parameter> ori, parameter gain, matrixD signals, CPGSTYPE CPGType, Matrix cmatrix){
    //1) sensory adaptation, agrf adaptive ground reaction force
    sf_gain=gain;
    sf_np_grf=np_grf;
    
    //2) Adaptive neural communication
   if(CPGType==REFLEX)
        setMi(0.0);
    aci_signals=signals;
    aci_CPGType=CPGType;
    aci_cmatrix=cmatrix;

    //3) vestibular 
    ves_ori=ori;
    ves_grf=grf;

    //4) phase reset
    prs_np_grf=np_grf;

    //5) phase inhibition
    pib_grf=grf;
}


void SPCPG::setMi(parameter mi){
    if(mi<1.0)// aviod to destroy robot since its frequecny is too high
        MI=mi;
    if(mi==0.0){// initialize CPG when MI euqal to original point ,mi==0.0
        MI=0.0;
        ANN::setActivity(0,0.0);
        ANN::setActivity(1,0.0);
        b(0,0.01);
        b(1,0.01);
        setOutput(0,0.01);
        setOutput(1,0.01);
    }
}

void SPCPG::updateWeights(){
    w(0,0,1.4);
    w(0,1,+0.18+MI);
    w(1,0,-0.18-MI);
    w(1,1,1.4);
}
void SPCPG::updateActivities(){
    ANN::updateActivities();
    for(unsigned int i=0;i<2;i++){
        a_t.at(i) = getActivity(getNeuron(i));
        o_t.at(i) = getOutput(getNeuron(i));
    }
    //----defining connection and sensory force feeddback--//
    aci->setInput(aci_signals);
    aci->setDelta(aci_CPGType,aci_cmatrix);
    aci->step();
    
    sf->setInput(sf_gain,sf_np_grf,o_t);
    sf->step();

    ves->setInput(ves_ori,ves_grf,a_t);
    ves->step();
    //-----phasse reset and phase inhibition----//
    prs->setInput(a_t,prs_np_grf);
    prs->step();

    pib->setInput(pib_grf);
    pib->step();

    // the sum of these terms
    for(unsigned int i=0;i<2;i++){
        a_t1.at(i) = a_t.at(i) - 0.0*sf->getOutput(i) + aci->getOutput(i) - 0.0*ves->getOutput(i) + 0.0*prs->getOutput(i) + 0.0*pib->getOutput(i);
        setActivity(i, a_t1.at(i));
    }
}

void SPCPG::updateOutputs(){
    ANN::updateOutputs();
}
void SPCPG::step(){
    updateWeights();
    updateActivities();
    updateOutputs();
}

parameter SPCPG::getSFOutput(unsigned int index)const{
    assert(index<2);
    return sf->getOutput(index);
}
parameter SPCPG::getACIOutput(unsigned int index)const{
    assert(index<2);
    return aci->getOutput(index);
}
//-------------------------sensory feedback function class ------//
SF::SF(){
    output.resize(2);
    o_t.resize(2);
    gamma = 0.0;
    grf = 0.0;
}

void SF::setInput(parameter gamma,parameter grf,vectorD o_t){
    this->gamma=gamma;
    this->grf =grf;
    this->o_t =o_t;
}

void SF::step(){
    output.at(0)= gamma*grf*cos(o_t.at(0));
    output.at(1)= gamma*grf*sin(o_t.at(1));
}

parameter SF::getOutput(unsigned int index)const{
    assert(index<2);
    return output.at(index);
}

//---------------------ACI Class------------------------//
ACI::ACI(unsigned int ID,unsigned int nCPGs){
    this->ID=ID;
    this->nCPGs =nCPGs;
    output.resize(2);
    cCPGat.resize(nCPGs);
    for(unsigned int i=0;i<nCPGs;i++)
        cCPGat.at(i).resize(2);
    delta.resize(nCPGs);
    for(unsigned int i=0;i<nCPGs;i++)
        delta.at(i).resize(nCPGs);
    K.resize(nCPGs);
    for(unsigned int i=0;i<nCPGs;i++)
        K.at(i).resize(nCPGs);
    C = 0.00;//0.01;
    initK(0.00);
    initDelta();
}

void ACI::setInput(matrixD signals){
    for(unsigned int i=0;i<signals.size();i++)
        for(unsigned int j=0;j<signals.at(i).size();j++)
            cCPGat.at(i).at(j) = signals.at(i).at(j);
}

void ACI::step(){
    for(unsigned int i=0; i<2;i++){
        output.at(i) =0.0;//initial
        for(unsigned int j=0;j<nCPGs;j++){//the number pf CPGs
            output.at(i) -= K.at(ID).at(j)*(C*(1-cos(cCPGat.at(ID).at(i)-cCPGat.at(j).at(i)-delta.at(ID).at(j)))+sin(cCPGat.at(ID).at(i)-cCPGat.at(j).at(i) - delta.at(ID).at(j)));
        }
    }
}

parameter ACI::getOutput(unsigned int index)const{
    assert(index<2);
    return output.at(index);
}

void ACI::initK(parameter constant){
    for(unsigned int i=0;i<nCPGs;i++)
        for(unsigned int j=0;j<nCPGs;j++)
            K.at(i).at(j) = constant;
}

void ACI::initDelta(parameter constant){
    for(unsigned int i=0;i<nCPGs;i++)
        for(unsigned int j=0;j<nCPGs;j++)
            delta.at(i).at(j) = constant;

}

void ACI::setK(){
    K.at(0).at(0) =0.00;
    K.at(0).at(1) =0.01;
    K.at(0).at(2) =0.00;
    K.at(0).at(3) =0.00;

    K.at(1).at(0) =0.00;
    K.at(1).at(1) =0.00;
    K.at(1).at(2) =0.01;
    K.at(1).at(3) =0.00;

    K.at(2).at(0) =0.01;
    K.at(2).at(1) =0.00;
    K.at(2).at(2) =0.00;
    K.at(2).at(3) =0.01;

    K.at(3).at(0) =0.00;
    K.at(3).at(1) =0.00;
    K.at(3).at(2) =0.00;
    K.at(3).at(3) =0.00;

}

void ACI::setDelta(CPGSTYPE CPGSType, Matrix cmatrix){
    switch(CPGSType){
        case GALLOP:
            // gallop output
            delta.at(0).at(0) = 0.0;
            delta.at(0).at(1) = -M_PI;
            delta.at(0).at(2) = 0.0;
            delta.at(0).at(3) = -M_PI;

            delta.at(1).at(0) = M_PI;
            delta.at(1).at(1) = 0.0;
            delta.at(1).at(2) = M_PI;
            delta.at(1).at(3) = 0.0;

            delta.at(2).at(0) = 0.0;
            delta.at(2).at(1) = -M_PI;
            delta.at(2).at(2) = 0.0;
            delta.at(2).at(3) = -M_PI;

            delta.at(3).at(0) = M_PI;
            delta.at(3).at(1) = 0.0;
            delta.at(3).at(2) = M_PI;
            delta.at(3).at(3) = 0.0;
            initK(0.01);
            break;
        case PACE:
            // pace output
            delta.at(0).at(0) = 0.0;
            delta.at(0).at(1) = 0.0;
            delta.at(0).at(2) = -M_PI;
            delta.at(0).at(3) = -M_PI;

            delta.at(1).at(0) = 0.0;
            delta.at(1).at(1) = 0.0;
            delta.at(1).at(2) = -M_PI;
            delta.at(1).at(3) = -M_PI;

            delta.at(2).at(0) = M_PI;
            delta.at(2).at(1) = M_PI;
            delta.at(2).at(2) = 0.0;
            delta.at(2).at(3) = 0.0;

            delta.at(3).at(0) = M_PI;
            delta.at(3).at(1) = M_PI;
            delta.at(3).at(2) = 0.0;
            delta.at(3).at(3) = 0.0;
            initK(0.01);
            break;
        case WALK_L:
            // a walk_d output
            delta.at(0).at(0) = 0.0;
            delta.at(0).at(1) = -0.5*M_PI;
            delta.at(0).at(2) = -M_PI;
            delta.at(0).at(3) = -1.5*M_PI;

            delta.at(1).at(0) = 0.5*M_PI;
            delta.at(1).at(1) = 0.0;
            delta.at(1).at(2) = -0.5*M_PI;
            delta.at(1).at(3) = -M_PI;

            delta.at(2).at(0) = M_PI;
            delta.at(2).at(1) = 0.5*M_PI;
            delta.at(2).at(2) = 0.0;
            delta.at(2).at(3) = -0.5*M_PI;

            delta.at(3).at(0) = 1.5*M_PI;
            delta.at(3).at(1) = M_PI;
            delta.at(3).at(2) = 0.5*M_PI;
            delta.at(3).at(3) = 0.0;
            initK(0.01);
            break;
        case WALK_D:
            // a walk_l output
            delta.at(0).at(0) = 0.0;
            delta.at(0).at(1) = -1.5*M_PI;
            delta.at(0).at(2) = -M_PI;
            delta.at(0).at(3) = -0.5*M_PI;

            delta.at(1).at(0) = 1.5*M_PI;
            delta.at(1).at(1) = 0.0;
            delta.at(1).at(2) = 0.5*M_PI;
            delta.at(1).at(3) = M_PI;

            delta.at(2).at(0) = M_PI;
            delta.at(2).at(1) = -0.5*M_PI;
            delta.at(2).at(2) = 0.0;
            delta.at(2).at(3) = 0.5*M_PI;

            delta.at(3).at(0) = 0.5*M_PI;
            delta.at(3).at(1) = -M_PI;
            delta.at(3).at(2) = -0.5*M_PI;
            delta.at(3).at(3) = 0.0;
            initK(0.01);
            break;
        case TROT:
            // trot output
            delta.at(0).at(0) = 0.0;
            delta.at(0).at(1) = -M_PI;
            delta.at(0).at(2) = -M_PI;
            delta.at(0).at(3) = 0.0;

            delta.at(1).at(0) = M_PI;
            delta.at(1).at(1) = 0.0;
            delta.at(1).at(2) = 0.0;
            delta.at(1).at(3) = M_PI;

            delta.at(2).at(0) = M_PI;
            delta.at(2).at(1) = 0.0;
            delta.at(2).at(2) = 0.0;
            delta.at(2).at(3) = M_PI;

            delta.at(3).at(0) = 0.0;
            delta.at(3).at(1) = -M_PI;
            delta.at(3).at(2) = -M_PI;
            delta.at(3).at(3) = 0.0;
            initK(0.01);
            break;
        case DECOUPLED:
            initK(0.0);
            break;
        case REFLEX:
            initK(0.0);
            break;
        case SP:
            assert(cmatrix.getM()==delta.size());
            for(unsigned int i=0;i<delta.size();i++)
                for(unsigned int j=0;j<delta.at(i).size();j++)
                    delta.at(i).at(j)=cmatrix.val(i,j);
            initK(0.04);
            break;
        default:
            perror("CPGSType don't define in cpg.cpp:334");
    }

}
//--------------------PhaseReset class----------------//
PhaseReset::PhaseReset(){
    Diracs =0.0;
    a_t.resize(2);
    a_t_old.resize(2);
    grf = 0.0;
    grf_old = 0.0;
    reset.resize(2);
}
void PhaseReset::setInput(vectorD a_t,parameter grf){
    assert(a_t.size()==2);
    this->a_t =  a_t;
    this->grf = grf;
}
void PhaseReset::step(){
    /* version I:
     * This version can work like GRF modulation to produce a stable gaits, 
     * but it is not the right phase reset. For the improved one, please see version II in nexts.
     */

    /*
    if((grf > 0.2)&&(grf_old <= 0.2)){ //0.2 is vital important, it casued the reset moments
        Diracs =1.0;
    }else{
        Diracs =0.0;
    }
    reset.at(0) = ( 1.0 - a_t.at(0))*Diracs;
    reset.at(1) = - a_t.at(1)*Diracs;
    grf_old = grf;
    */

    /*Version II:
     * The phase reset, it has seseral key points:
     * 1) If the actual touch mement is ahead of the expected one, then reset the phase, let the CPG oscillation return move
     * 2) Find the activity value when the direction swithing, just opposite the sign of the activity of O2.
     * 3)O1 used to control hip joint, whiel knee joint was activated by O1 joint. 
     * 4) The degree of the knee joint movement is decieded by GRF value.
     */
    if((grf > 0.2)&&(grf_old <= 0.2)){// Indicate the actual touch moment
        if(a_t.at(0)>a_t_old.at(0)){// if signal is still increase for hip2 joint forward movement.
            Diracs =1.0;
        }else{
           Diracs =0.0;
        }
    }else{
        Diracs =0.0;
    }
    reset.at(0) = (1-a_t.at(0))*Diracs;
    reset.at(1) = - 1.0*a_t.at(1)*Diracs;// maybe use hieracy CPG phase process
    grf_old = grf;

    a_t_old.at(0)=a_t.at(0);
    a_t_old.at(0)=a_t.at(0);

}
parameter PhaseReset::getOutput(unsigned int index)const{
    assert(index < 2);
    return reset.at(index);
}

//-----------PhaseInhibition class-----//
PhaseInhibition::PhaseInhibition(){
    Ch=0.06;
    grf=0.0;
    Tduration =0.0;
    inhibition.resize(2);
}
void PhaseInhibition::setInput(parameter grf){
    this->grf =grf;
}
void PhaseInhibition::step(){
    if(grf >0.3)
        Tduration = 1.0;
    else{
        Tduration = 0.0;
    }
    inhibition.at(0)=0.0;
    inhibition.at(1)=-Ch*grf*Tduration;
}

parameter PhaseInhibition::getOutput(unsigned int index)const{
    assert(index<2);
    return inhibition.at(index);
}

//-------------------------vestibular response class ------//
Vestibular::Vestibular(uint8_t leg){
    assert(leg>=0);
    ves.resize(2);
    roll = 0.0;
    pitch = 0.0;
    grf = 0.0;
    switch(leg){
        case 0:
        case 1:
            roll_gain= 50;
            break;
        case 2:
        case 3:
            roll_gain= -50;
            break;
        default:
            perror("leg number is wrong in vestibular response");
    }

    switch(leg){
        case 0:
        case 2:
            pitch_gain= 50;
            break;
        case 1:
        case 3:
            pitch_gain= -50;
            break;
        default:
            perror("leg number is wrong in vestibular response");
    }

}
void Vestibular::setInput(vector<parameter> ori, parameter grf,vectorD a_t){
    this->roll = ori.at(0);
    this->pitch = ori.at(1);
    this->grf = grf;
    this->a_t = a_t;
}
void Vestibular::step(){
    ves.at(0)= 0.2*tanh(roll_gain*roll+ pitch_gain*pitch)*cos(a_t.at(0));
    ves.at(1)= 0.2*tanh(roll_gain*roll+ pitch_gain*pitch)*sin(a_t.at(1));
    //ves.at(0)= (roll_gain*roll+ pitch_gain*pitch)*cos(a_t.at(0))*grf;
    //ves.at(1)= (roll_gain*roll+ pitch_gain*pitch)*sin(a_t.at(1))*grf;
    //ves.at(0)= (0.0*roll_gain*roll*(grf>0.01 ? 0.0:1.0 )+ 0.0*pitch_gain*pitch*(grf>0.02 ? 1.0:0.0 ))*cos(a_t.at(0));
    //ves.at(1)= (0.0*roll_gain*roll*(grf>0.01 ? 0.0:1.0 )+ 0.0*pitch_gain*pitch*(grf>0.02 ? 1.0:0.0 ))*sin(a_t.at(1));
}
parameter Vestibular::getOutput(unsigned int index)const{
    assert(index<2);
    return ves.at(index);
}

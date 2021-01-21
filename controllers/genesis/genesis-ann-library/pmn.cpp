#include "pmn.h"


PMN::PMN(unsigned int _JointNum)
{
    //setDefaultTransferFunction(identityFunction());
    //setAllTransferFunctions(identityFunction());
    setNeuronNumber(_JointNum);
    JointNum=_JointNum;


    delta1=0.001;
    delta1_old=0.0;
    o1=0.0;
    o1_old=0.0;
    output1=0.0;
    delta2=0.001;
    delta2_old=0.0;
    o2=0.0;
    o2_old=0.0;
    output2=0.0;
    grf=0.0;

}

void PMN::setInputGRF(double grf){
    this->grf=grf;
}

void PMN::updateOutputs(){
    double output2_upLimit=0.55;
    double output2_downLimit=-0.3;
    o2=ANN::getOutput(2);
    o1=ANN::getOutput(1);
    delta1=o1-o1_old;
    delta2=o2-o2_old;


    if(grf>0.05){
        if(delta1>0.0){
            if(delta2>0.0)
                output2+= 1.2*delta2;// right or fast
            if(delta2<0.0)
                output2= output2; //reverse or stop
        }
        if(delta1<0.0){
            if(delta2>0.0)
                output2= output2;
            if(delta2<0.0)
                output2+=0.2*delta2;
        }
    }else{//swing
        if(delta1>0.0){
            if(delta2>0)
                output2+=0.2*delta2;
            if(delta2<0)
                output2=output2;
        }
        if(delta1<0.0){
            if(delta2>0)
                output2=output2;
            if(delta2<0)
                output2+=1.2*delta2;
        }
    }

    output2 = output2 > output2_upLimit ? output2_upLimit : (output2 < output2_downLimit ? output2_downLimit : output2);
    delta1_old=delta1;
    o1_old=o1;
    delta2_old=delta2;
    o2_old=o2;

    ANN::setOutput(2,output2);
}

void PMN::step(){
    updateWeights();
    updateActivities();
    ANN::updateOutputs();
    //PMN::updateOutputs();
}


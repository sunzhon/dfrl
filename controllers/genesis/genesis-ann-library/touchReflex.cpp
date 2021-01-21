/*
 * touchReflex.cpp
 *  Created on: 2020-02-03
 *      Author: suntao
 */

#include "touchReflex.h"
using namespace stcontroller;
namespace stcontroller {

//-----------------TouchReflex class -----------------------//
TouchReflex::TouchReflex(unsigned int leg){
	ID =leg;
	ffm = new ForceForwardmodel(1.0);
	dil = new DILearn();
    dil->setParameters(0.65,0.1,0.01,0.85,0.01,0.001);

    setNeuronNumber(1);
    setTransferFunction(0,identityFunction());
    activate=false;
}
TouchReflex::~TouchReflex(){
    delete ffm;
    delete dil;
}
void TouchReflex::setInput(double grf, double hip_jmc) {
    ffm->setInput(hip_jmc);
    this->grf=grf;
}
void TouchReflex::step(){
    // Just when robot was put on the ground, and joint are move,then start this reflexes
    if((grf>0.1)&&(ffm->getOutput()>0.1))
        activate=true;

    ffm->step();
    if(activate){
        dil->setInput(ffm->getOutput()/10.0,grf/10.0);// 1.6 is the standard value of the Touch, the value was got by experiments
        dil->step();
    }
    ANN::setOutput(0,1.0 + dil->getOutput());
    if(ID==0){
        //printf("grf:%0.2f\t exp:%0.2f\n", grf,ffm->getOutput());
        //printf("err:%0.2f\n", grf-ffm->getOutput());
        //printf("out:%0.2f\n", ANN::getOutput(0));
    }
}

double TouchReflex::getOutput(){
    return ANN::getOutput(0);
}

} /* namespace stcontroller */

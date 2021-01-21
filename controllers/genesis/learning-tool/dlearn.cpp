/*
 * dLearn.cpp
 *
 *  Created on: Dec 8, 2017
 *      Author: suntao
 */

#include "dlearn.h"
/*
 * This class implement adaptive sensory feedback strength to CPG and adaptive bias of motor neurons by
 * the difference  between the contact sensory and expected contact value .
 *
 *
 * */
//-----------------Force forward model---------------//
ForceForwardmodel::ForceForwardmodel(float _alpha,float _gama) {
	setNeuronNumber(1);
	setTransferFunction(0,identityFunction());
	gama = _gama;
	alpha = _alpha;
	output_old = 0.0;
	activity_old = 0.0;
	input = 0.0;
	output = 0.0;
    activity = 0.0;
}

ForceForwardmodel::~ForceForwardmodel() {

}
void ForceForwardmodel::setAlpha(float _alpha) {
	alpha = _alpha;
}
void ForceForwardmodel::setInput(float _input) {
	input = _input;
	ANN::setInput(0,input);
}
void ForceForwardmodel::step() {
	ANN::updateActivities();
    activity=ANN::getActivity(0);

	if (activity <= activity_old)
		G = 1.0;
	else
		G = 0.0;
	output = alpha * (gama * G+ (1 -gama)* output_old); //-0.8*F+0.8;
    activity_old=activity;
	output_old = output;
	ANN::setOutput(0,output);
}
float ForceForwardmodel::getOutput() const {
	return ANN::getOutput(0);
}
float ForceForwardmodel::getInput() const {
	return ANN::getInput(0);
}

//------------Dual-learning ----------------------------------------//

DLearn::DLearn() {

	// TODO Auto-generated constructor stub
	//parammeters
	As = 0.95;
	Af = 0.65;
	Bs = 0.018;
	Bf = 0.0225;
	Kf_old = 0.0;
	Ks_old = 0.0;

    //parameters
    error=0.0;
    expected_value = 0.0;
    actual_value = 0.0;
    lowerThreshold = 0.0;
    upperThreshold = 1.0;
    controlOutput = 0.0;
}

void DLearn::setInput(float expected_value,float actual_value) {
	this->expected_value = expected_value;
	this->actual_value = actual_value;
}

void DLearn::step(){
	//0) calculate the error
	error = actual_value - expected_value;
    //1) learn to produce adaptive weight for sensory feedback strength
    Kf = 0.7*(Af * Kf_old + Bf * error);
    Ks = 0.7*(As * Ks_old + Bs * error);
    Kf= Kf>0.15 ? 0.15:(Kf<-0.15 ? -0.15:Kf);
    Ks= Ks>0.15 ? 0.15:(Ks<-0.15 ? -0.15:Ks);
    gamma = Kf + Ks;	//4.5*(Kf+Ks)+0.045;
    Kf_old = Kf;
    Ks_old = Ks;
    
    if(gamma <= lowerThreshold){
        controlOutput = lowerThreshold;
    }
    else if(gamma >=upperThreshold){
        controlOutput = upperThreshold;
    }else{

        controlOutput = gamma;
    }

}

float DLearn::getOutput() {
    return controlOutput;
}


void DLearn::setParameters(float Af, float Bf, float As, float Bs ){

    this->Af=Af;
    this->Bf=Bf;

    this->As=As;
    this->Bs=Bs;
}

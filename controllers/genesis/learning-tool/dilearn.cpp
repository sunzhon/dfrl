/*
 * diLearn.cpp
 *
 *  Created on: June 22, 2019
 *      Author: suntao
 */

#include "dilearn.h"
/*
 * This class implement adaptive sensory feedback strength to CPG and adaptive initialOutput of motor neurons by
 * the difference  between the contact sensory and expected contact value .
 *
 *
 * */
//-----------------Angle forward model---------------//
AngleForwardmodel::AngleForwardmodel(float alpha) {
	setNeuronNumber(1);
	setTransferFunction(0,identityFunction());
	this->alpha = alpha;
	input = 0.0;
	output = 0.0;
}

AngleForwardmodel::~AngleForwardmodel() {

}

void AngleForwardmodel::setAlpha(float alpha) {
	this->alpha = alpha;
}

void AngleForwardmodel::setInput(float input) {
	this->input = input;
	ANN::setInput(0,alpha*input);
}

void AngleForwardmodel::step() {
	ANN::updateActivities();
	ANN::updateOutputs();
}

float AngleForwardmodel::getOutput() const {
	return ANN::getOutput(0);
}


//------------Dual-learning ----------------------------------------//

DILearn::DILearn() {

	// TODO Auto-generated constructor stub
	//weight
	Af = 0.98;//0.87;//1.0;//0.6;
	Bf = 0.15;//.3;
    Cf = 0.0004;//0.0002;//0.015;
	As = 0.997;//0.997;//1.0;//0.9;
	Bs = 0.03;//0.1;
    Cs = 0.0002;//0.000015;

	Kf_old = 0.0;
	Ks_old = 0.0;

    //parameters
    error = 0.0;
    integralError = 0.0;
    actualValue = 0.0;
    expectedValue = 0.0;
    // 
    output = Kf_old + Ks_old;
    uplimit=5.0;
    downlimit=-5.0;
}

void DILearn::step(){
    //0) calculate the error
    error = actualValue-expectedValue;
    //1) learn to produce adaptive weight for sensory feedback strength
    if((error > 0.0)&&(output > uplimit)){// no update
        ;
    }else if((error < 0.0)&&(output < downlimit)){// no update
        ;
    }else{
        integralError += error;
        Kf = Af * Kf_old + Bf * error +Cf * integralError;
        Ks = As * Ks_old + Bs * error +Cs * integralError;
        output = Kf + Ks;
        Kf_old = Kf;
        Ks_old = Ks;
    }

}

void DILearn::setInput(float expectedValue,float actualValue) {
    this->expectedValue = expectedValue;
    this->actualValue = actualValue;
}

float DILearn::getOutput() {
    return output;
}

void DILearn::setParameters(float Af, float Bf, float Cf, float As, float Bs, float Cs){

    this->Af=Af;
    this->Bf=Bf;
    this->Cf=Cf;

    this->As=As;
    this->Bs=Bs;
    this->Cs=Cs;
}

void DILearn::setLimit(float uplimit, float downlimit){
    this->uplimit=uplimit;
    this->downlimit=downlimit;

}

float DILearn::getUpLimit(){
    return uplimit;
}
float DILearn::getDownLimit(){
    return downlimit;
}

//------------ Adaptive Dual-learning ----------------------------------------//
//Depending on the error to choose dual, faster or slower learner

ADILearn::ADILearn() {

	// TODO Auto-generated constructor stub
	//weight
	Af = 0.98;//0.87;//1.0;//0.6;
	Bf = 0.15;//.3;
    Cf = 0.0004;//0.0002;//0.015;
	As = 0.997;//0.997;//1.0;//0.9;
	Bs = 0.03;//0.1;
    Cs = 0.0002;//0.000015;

	Kf_old = 0.0;
	Ks_old = 0.0;

    //parameters
    error = 0.0;
    error_old=0.0;
    error_old_old=0.0;
    error_delta=0.0;
    error_threshold=0.15;
    integralError = 0.0;
    actualValue = 0.0;
    expectedValue = 0.0;
    // 
    output = Kf_old + Ks_old;
    uplimit=5.0;
    downlimit=-5.0;
}

void ADILearn::step(){
    //0) calculate the error
    error = actualValue-expectedValue;
    error_delta=error-error_old;

    //1) learn to produce adaptive weight for sensory feedback strength
    if((error > 0.0)&&(output >= uplimit)){// no update, touch uplimit and will be over, so stop
        ;
    }else if((error < 0.0)&&(output <= downlimit)){// no update
        ;
    }else{// among the output limit
        // adaptive learner
        if((error*error_delta>0.0)&&(abs(error)>error_threshold)){// if error augment and bigger than threshold
            Kf_old = Kf;
            Ks_old = Ks;
            integralError += error;
            Kf = Af * Kf_old + Bf * error +Cf * integralError;
            Ks = As * Ks_old + Bs * error +Cs * integralError;
            output = Kf + Ks;
        }else{
            //PID control
            integralError += error;
            Ks = As*Ks_old + Bs*error + Cs*integralError;
            output=Ks+Kf;
        }
    }
    error_old_old=error_old;
    error_old=error;
}

void ADILearn::setInput(float expectedValue,float actualValue) {
    this->expectedValue = expectedValue;
    this->actualValue = actualValue;
}

float ADILearn::getOutput() {
    return output;
}

void ADILearn::setParameters(float Af, float Bf, float Cf, float As, float Bs, float Cs){

    this->Af=Af;
    this->Bf=Bf;
    this->Cf=Cf;

    this->As=As;
    this->Bs=Bs;
    this->Cs=Cs;
}

void ADILearn::setLimit(float uplimit, float downlimit){
    this->uplimit=uplimit;
    this->downlimit=downlimit;

}


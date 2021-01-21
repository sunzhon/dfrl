/*
 * sensoryAdaptation.cpp
 *
 *  Created on: June 22, 2019
 *      Author: suntao
 */

#include "adaptiveFeedbackGain.h"
/*
 * This class implement adaptive sensory feedback strength to CPG and adaptive bias of motor neurons by
 * the difference  between the contact sensory and expected contact value .
 *
 *
 * */
//------------phase adaptation ----------------------------------------//

AdaptiveFeedbackGain::AdaptiveFeedbackGain(unsigned int id) {

	// TODO Auto-generated constructor stub
    this->ID = id;
	// force fwardmodel
	FFM =new ForceForwardmodel(0.95);
    //dual-rate learning
    DL = new DLearn();
    DL->setParameters(0.65,0.0225,0.95,0.018);
    output = 0.0;
}

AdaptiveFeedbackGain::~AdaptiveFeedbackGain() {
	// TODO Auto-generated destructor stub
	delete FFM;
    delete DL;
}
void AdaptiveFeedbackGain::setInput(float grf, float joint_motor){
    FFM->setInput(joint_motor);
    this->grf=grf;
    }
void AdaptiveFeedbackGain::step(){
	//-1) force forward model
	FFM->step();
    expectedValue = FFM->getOutput();
    actualValue = grf;

    DL->setInput(expectedValue,actualValue);
    DL->step();

    output = DL->getOutput();
}
float AdaptiveFeedbackGain::getOutput(){
    return output;
}
float AdaptiveFeedbackGain::getFMOutput(){
    return expectedValue;
}
float AdaptiveFeedbackGain::getAGRF(){
    return actualValue;
}


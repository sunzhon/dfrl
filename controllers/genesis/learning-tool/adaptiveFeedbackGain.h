/*
 * phaseAdaptation.h
 *
 *  Created on: June 22, 2019
 *      Author: suntao
 */

#ifndef PHASEADAPTATION_H_
#define PHASEADAPTATION_H_
#include "dlearn.h"

class AdaptiveFeedbackGain {
public:
	AdaptiveFeedbackGain(unsigned int id);
	virtual ~AdaptiveFeedbackGain();
	void step();
	void setInput(float actual_force,float joint_motor);
	float getOutput();
	float getFMOutput();
	float getAGRF();
private:
    unsigned int ID;
	//model
	//learn
    ForceForwardmodel * FFM;
    DLearn * DL;

	float joint_motor;
	float grf;
    float output;// adaptive ground reaction force
    float expectedValue,actualValue;
};

#endif /* PHASEADAPTATION_H_ */

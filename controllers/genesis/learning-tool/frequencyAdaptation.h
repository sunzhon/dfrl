
/*
 * frequencyAdaptation.h
 *
 *  Created on: June 22, 2019
 *      Author: suntao
 */

#ifndef FREQUENCYADAPTATION_H_
#define FREQUENCYADAPTATION_H_
#include "dilearn.h"
#include "postProcessing.h"
#include "dualIntegralLearner.h"
#include "study-tool/plotData.h"
#include "motionPhaseStability.h"
class FrequencyAdaptation {
public:
	FrequencyAdaptation(unsigned int id);
	virtual ~FrequencyAdaptation();
	void step();
	void setInput(float jcm,float jaf);
	float getOutput();
private:
	//model
	//learn
    unsigned int ID;
    AngleForwardmodel * AFM;
    DILearn * DIL;
    //dualIntegralLearner * DIL;
    postProcessing *PPE;//expected signals
    postProcessing *PPA;//actual signals

	float jcm;
    std::vector<float> data;
	float jaf;

    CircleQueue<float> *expected;
    CircleQueue<float> *actual;

    float expectedValue, actualValue;
    float output;
};

#endif /* FREQUENCYADPTATION_H_ */

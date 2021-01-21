
/*
 * frequencyAdaptationBasedVes.h
 *
 *  Created on: DEc 1, 2019
 *      Author: suntao
 */

#ifndef FREQUENCYADAPTATIONBASEDVES_H_
#define FREQUENCYADAPTATIONBASEDVES_H_
#include "dilearn.h"
#include "postProcessing.h"
#include "dualIntegralLearner.h"
#include "study-tool/plotData.h"
#include "motionPhaseStability.h"
class FrequencyAdaptationBasedVes {
public:
	FrequencyAdaptationBasedVes();
	virtual ~FrequencyAdaptationBasedVes();
	void step();
	void setInput(float jcm,float jaf);
	float getOutput();
private:
	//model
	//learn
    DILearn * DIL;
    //dualIntegralLearner * DIL;
    postProcessing *PPOriAlpha;//actual signals
    postProcessing *PPOriBeta;//actual signals

	float alpha,beta;

    float expectedValue, actualAlphaAmplitudeValue, actualBetaAmplitudeValue;
    float output;
    int runTime =0;
};

#endif /* FREQUENCYADPTATIONBASEDVES_H_ */

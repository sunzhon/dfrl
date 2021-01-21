/*
 * frequencyAdaptationBasedVes.cpp
 *
 *  Created on: June 22, 2019
 *      Author: suntao
 */

#include "frequencyAdaptationBasedVes.h"
#include <math.h>
/*
 * This class is used to achieve adaptive frequency for CPGs through dual-rate intergrative learning
 * using body attitude angles
 *
 * */

//plotData a("1",plotSize);
//------------frequency adaptation ----------------------------------------//
FrequencyAdaptationBasedVes::FrequencyAdaptationBasedVes() {
    //dual-rate learning
    DIL = new DILearn();

    // post processing 
    PPOriAlpha= new postProcessing();
    PPOriBeta = new postProcessing();

    //lower filter, when the frequency of input signals increase, the amplitude of the output signals become smaller. Here the filter for PPE is to filter noise, for PPA, there are two functions, one is to filter noise, another is to equaivaent a motor tranfer which work like a lower filter for commands
    PPOriAlpha->setBeta(0.8);// for simulation is 0.4, for real is 0.3 because the noise is bigger in real
    PPOriBeta->setBeta(0.8);// for simulation is 0.4, for real is 0.3 because the noise is bigger in real

    expectedValue = 0.0;
    actualAlphaAmplitudeValue = 0.0;
    actualBetaAmplitudeValue = 0.0;
    output = 0.0;
}

FrequencyAdaptationBasedVes::~FrequencyAdaptationBasedVes() {
    // TODO Auto-generated destructor stub
    delete DIL;
    delete PPOriAlpha;
    delete PPOriBeta;
}

void FrequencyAdaptationBasedVes::setInput(float alpha, float beta){
    this->alpha=alpha;//alpha along x direction(forward way) of the ori
    this->beta=beta;//beta along y direction(sideway) of the ori
}

void FrequencyAdaptationBasedVes::step(){

    actualAlphaAmplitudeValue = PPOriAlpha->calculateLPFAmplitude(alpha);
    actualBetaAmplitudeValue = PPOriBeta->calculateLPFAmplitude(beta);
    expectedValue = 0.008;

    if(runTime>=100)
    {
        DIL->setInput(expectedValue,actualBetaAmplitudeValue);
        DIL->step();
    }else
        runTime++;
    // Bias the output
    output = DIL->getOutput();// output*0.7+ 0.3*DIL->getOutput();
    //if(ID==0)
    //    printf("ID:%d -> E:%.5f A:%.5f O:%.6f\n",ID,expectedValue,actualValue,output);
}

float FrequencyAdaptationBasedVes::getOutput(){
    return output;
}

/*
 * frequencyAdaptation.cpp
 *
 *  Created on: June 22, 2019
 *      Author: suntao
 */

#include "frequencyAdaptation.h"
#include <math.h>
/*
 * This class is used to achieve adaptive frequency for CPGs through dual-rate intergrative learning
 *
 *
 * */
int plotSize=3;
float bias = -0.04;
int runTime=0;

//plotData a("1",plotSize);
//------------frequency adaptation ----------------------------------------//
FrequencyAdaptation::FrequencyAdaptation(unsigned int id) {
    this->ID =id;
    // TODO Auto-generated constructor stub
    // force fwardmodel
    AFM =new AngleForwardmodel(1.0);
    //dual-rate learning
    DIL = new DILearn();

    // post processing 
    PPE = new postProcessing();
    PPA = new postProcessing();

    //lower filter, when the frequency of input signals increase, the amplitude of the output signals become smaller. Here the filter for PPE is to filter noise, for PPA, there are two functions, one is to filter noise, another is to equaivaent a motor tranfer which work like a lower filter for commands
    PPA->setBeta(0.25);// for simulation is 0.4, for real is 0.3 because the noise is bigger in real
    PPE->setBeta(0.8);

    //expected = new CircleQueue<float>(0.0,2);
    //actual = new CircleQueue<float>(0.0,80);

    expectedValue = 0.0;
    actualValue = 0.0;
    output = 0.0;
}

FrequencyAdaptation::~FrequencyAdaptation() {
    // TODO Auto-generated destructor stub
    delete AFM;
    delete DIL;
    delete PPE;
    delete PPA;
}

void FrequencyAdaptation::setInput(float jcm, float jaf){
    AFM->setInput(jcm);
    this->jaf=jaf;//joint actual feedback
}

void FrequencyAdaptation::step(){
    // force forward model
    AFM->step();
    //expected->write(PPE->calculateLPFAmplitude(AFM->getOutput())+bias);
    //actual->write(PPA->calculateLPFAmplitude(jaf));
    //expected->step();
    //actual->step();
    //expectedValue=((float)((int)(expected->getMax()*1000)))/1000.0;
    //actualValue=((float)((int)(actual->getMax()*1000)))/1000.0;

    expectedValue = PPE->calculateLPFAmplitude(AFM->getOutput())+bias;
    actualValue  = PPA->calculateLPFAmplitude(jaf);
    expectedValue=((float)((int)round(expectedValue*10000.0)))/10000.0;
    actualValue=((float)((int)round(actualValue*10000.0)))/10000.0;

    if(runTime>=2400)
    {
        DIL->setInput(expectedValue,actualValue);
        DIL->step();
    }else
        runTime++;
    // Bias the output
    output = output*0.7+ 0.3*DIL->getOutput();
    //if(ID==0)
    //    printf("ID:%d -> E:%.5f A:%.5f O:%.6f\n",ID,expectedValue,actualValue,output);
}

float FrequencyAdaptation::getOutput(){
    return output;
}

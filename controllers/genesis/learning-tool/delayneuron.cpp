/*
 * delayneuron.cpp
 *
 *  Created on: 2018-3-3
 *      Author: suntao
 */

#include "delayneuron.h"
using namespace std;
namespace stcontroller {

DelayNeuron::DelayNeuron(vector<uint16_t> delay) {
	// TODO Auto-generated constructor stub
	setAllTransferFunctions(identityFunction());
    uint8_t number=delay.size();
    setNeuronNumber(number);
    delay_time.resize(number);
    mDelays.resize(number);
    for(uint8_t i=0;i<number;i++){
    delay_time.at(i)=delay.at(i);
	mDelays.at(i) = new STDelayline(delay_time.at(i)*2);
    }
}

DelayNeuron::~DelayNeuron() {
    // TODO Auto-generated destructor stub
    for(std::vector<STDelayline *>::iterator it= mDelays.begin();it!=mDelays.end();it++ )
        delete *it;
    mDelays.clear();
}
void DelayNeuron::updateOutputs(){
   // ANN::updateOutputs();
   for(uint8_t i=0;i<mDelays.size();i++ ){
    mDelays.at(i)->Write(ANN::getActivity(i));
    mDelays.at(i)->Step();
    ANN::setOutput(i,mDelays.at(i)->Read(delay_time.at(i)));    
   }

}

} /* namespace stcontroller */

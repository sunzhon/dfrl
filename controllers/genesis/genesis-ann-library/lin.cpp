/*
 * lin.cpp
 *
 *  Created on: Oct 24, 2017
 *      Author: suntao
 */

#include "lin.h"

namespace stcontroller {

LIN::LIN() {
	// TODO Auto-generated constructor stub
	 setDefaultTransferFunction(identityFunction());//addNeuron will use this transfer function
	 setAllTransferFunctions(identityFunction());
	 setNeuronNumber(4);

}
void LIN::step(){
    updateWeights();
    updateActivities();
    updateOutputs();
}
LIN::~LIN() {
	// TODO Auto-generated destructor stub
}

}

/*
 * delayneuron.h
 *
 *  Created on: 2018-3-3
 *      Author: suntao
 */

#ifndef DELAYNEURON_H_
#define DELAYNEURON_H_

#include <utils/ann-framework/ann.h>
#include "stdelayline.h"
using namespace std;
namespace stcontroller {

class DelayNeuron: public ANN{

public:
	DelayNeuron(vector<uint16_t> delay);
	virtual ~DelayNeuron();
    void updateOutputs();
private:
	std::vector<STDelayline *> mDelays;
    std::vector<uint16_t> delay_time;
};

} /* namespace stcontroller */

#endif /* DELAYNEURON_H_ */

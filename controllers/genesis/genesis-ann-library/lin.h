/*
 * lin.h
 *This is a Input neurons group of locomotion neural control network
 *  Created on: Oct 24, 2017
 *      Author: suntao
 */

#ifndef LIN_H_
#define LIN_H_

#include "utils/ann-framework/ann.h"

namespace stcontroller {

class LIN: public ANN {
public:
	LIN();
    void step();
	virtual ~LIN();
};

}

#endif /* LIN_H_ */

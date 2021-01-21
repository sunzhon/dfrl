/*
 * ExtensorReflex.h
 *
 *  Created on: 2018-3-9
 *      Author: suntao
		email:suntao.hn@qq.com
 */

#ifndef EXTENSORREFLEX_H_
#define EXTENSORREFLEX_H_

#include <utils/ann-framework/ann.h>
#include <utils/ann-library/vrn.h>
//#include <utils/ann-library/ExtensorReflex.h>
#include <stdio.h>
#include <assert.h>
//#include <shark/Data/Csv.h>
//#include <shark/Algorithms/Trainers/LDA.h>
//#include <shark/ObjectiveFunctions/Loss/ZeroOneLoss.h>
#include <iostream>

using namespace std;
namespace stcontroller {
class ExtensorReflex: public ANN {
public:
	ExtensorReflex(int _leg);
	virtual ~ExtensorReflex();
	void setInput(double _leg1, double _leg2, double _leg3, double _leg4,double _grf);
	void step();
	double getOutput(unsigned int index);

private:
	VRN *vrn;
	double grf,leg1,leg2,leg3,leg4;
	unsigned int legId;
};

} /* namespace stcontroller */

#endif /* EXTENSORREFLEX_H_ */

#ifndef DFPCPG_H_
#define DFPCPG_H_

#include "utils/ann-framework/ann.h"
#include <assert.h> 
class DFPCPG: public ANN {
public:

	DFPCPG();
    void step();
	void updateOutputs();
	void setBeta(double _beta);
	void setInput(int index, double value);
private:
	std::vector<double> set;
	std::vector<double> countup;
	std::vector<double> countdown;

	std::vector<double> pcpg_step;
	std::vector<double> setold;
	std::vector<double> countupold;
	std::vector<double> countdownold;
	std::vector<double> diffset;
	std::vector<double> deltaxdown;
	std::vector<double> deltaxup;
	std::vector<double> xup;
	std::vector<double> xdown;

	std::vector<double> yup;
	std::vector<double> ydown;
	std::vector<double> pcpg_output;
	std::vector<double> beta;// control the duty factor
};

#endif /* PCPG_H_ */

#ifndef FTNN_H_
#define FTNN_H_

#include <math.h>
#include <vector>
#include "utils/ann-framework/ann.h"

using namespace std;

class FTNN: public ANN{

public:
    FTNN(unsigned int ID);
    void step();

private:
    unsigned int ID;
    vector<double> ftnn_step;
    vector<double> ftnn_stepold;
	std::vector<int> set;
	std::vector<int> countup;
	std::vector<int> countdown;
	std::vector<int> setold;
	std::vector<int> countupold;
	std::vector<int> countdownold;
	std::vector<int> diffset;
	std::vector<int> deltaxdown;
	std::vector<int> deltaxup;
	std::vector<int> xup;
	std::vector<int> xdown;

	std::vector<float> ftnn_output;
    std::vector<float> input_max;
    std::vector<float> input_min;

    std::vector<string> phase;

    float t;
    float T;//period time
    float H;
    float S;
    float df;
    float Ty;
    float Px,Py;
    float Px0,Py0;
    float Px_max, Px_min,Py_max,Py_min;
};




#endif

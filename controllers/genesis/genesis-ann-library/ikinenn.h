#ifndef IKINENN_H_
#define IKINENN_H_

#include <math.h>
#include <vector>
#include "utils/ann-framework/ann.h"

using namespace std;

class IkineNN: public ANN{

public:
    IkineNN();
    void step();

private:
    vector<float> input;
    vector<float> theta;
	std::vector<float> theta_init;
	std::vector<float> link;
};





#endif

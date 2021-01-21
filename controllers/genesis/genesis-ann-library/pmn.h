#ifndef PMN_H_
#define PMN_H_

#include "utils/ann-framework/ann.h"
//#include <ode_robots/amosiisensormotordefinition.h>


class PMN : public ANN {
public:
    PMN(unsigned int _JointNum);
    void step();
    void updateOutputs();
    void setInputGRF(double grf);
private:
    unsigned int JointNum;
    double o1, o1_old, delta1, delta1_old;
    double o2, o2_old, delta2, delta2_old;
    double grf;
    double output1, output2;
};


#endif /* PMN_H_ */

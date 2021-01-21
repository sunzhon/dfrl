
#ifndef NP_H_
#define NP_H_


#include "utils/ann-framework/ann.h"
#include <string.h>
#include <stdlib.h>
#include <iostream>
class NP:public ANN {
public:

	NP(std::string _transfer= "logistic",bool learning=false);
	double getOutput();
	double getWr();
	void step();
	void stepLearning(double _target);
	void setInput(double _input);
	void startLearning();
	void setUp(double _Wi,double _Wr,double _Bias);
private:
	unsigned int count;
	double nu,delta, deltaWi,deltaWr,deltaB;
	double Wi,Wr,Bias;
	double input,output,output_old,target,Err;
	bool learning_state;
	string transfer;//the type fo np
};

#endif /* PCPG_H_ */

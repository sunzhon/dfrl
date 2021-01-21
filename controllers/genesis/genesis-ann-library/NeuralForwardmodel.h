#ifndef _NEURALFORWARDMODEL_H_
#define _NEURALFORWARDMODEL_H_ 


#include <utils/ann-framework/ann.h>
#include "learning-tool/dlearn.h"
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <assert.h>

class ForceForwardmodel;

class NeuralForwardmodel:public ANN {
public:

	NeuralForwardmodel(std::string _transfer= "logistic",bool learning=false);
	~NeuralForwardmodel();
	double getOutput();
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
	ForceForwardmodel *FFM;
};


#endif


#ifndef FLEXORREFLEX_H_
#define FLEXORREFLEX_H_

#include <utils/ann-framework/ann.h>
#include "learning-tool/dlearn.h"
#include "neuralpreprocessing.h"
#include "NeuralForwardmodel.h"
#include <assert.h>
#include <stdio.h>
#include <iostream>
#include <string>
class NP;
class NeuralForwardmodel;
class ForceForwardmodel;

namespace stcontroller{
class SwF: public ANN{
	public:
			SwF();
			virtual ~SwF();
			void step();
			void setInput(double _input);
			double getOutput();
	private:
		double output,input;
		unsigned int SwCount,SwCount_old,set,set_old,diffset;
		unsigned int deltaSw;
		ForceForwardmodel * FFM;
};
//------------------FlexorRefelx class-------------//
class FlexorReflex: public ANN{
public:
	FlexorReflex();
	virtual ~FlexorReflex();
	void setInput(double _obd);
	void step();
	double getOutput(unsigned int id);
private:

	double obd,jointMotor;
	NP * obstacleDetectNp;
	NP * delayhipNp;
	NP *delaykneeNp;
};

}
#endif

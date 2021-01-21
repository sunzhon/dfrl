#ifndef LOCALREFLEX_H_
#define LOCALREFLEX_H_

#include <ann.h>
#include "study-tool/plotData.h"
namespace stcontroller {
class NeuralForwardModel: public ANN{
public:
	NeuralForwardModel();
	void step();
	void setInput(double _input);
	double getOutput();
private:
	double Wi,Wr,Bias;
	double input;

};

//--------***SearchingReflex***----------------//
class SearchingReflex: public ANN{
public:
	SearchingReflex(unsigned int ID);
	~SearchingReflex();
	void step();
	void setInput(double _actualGrf,double _jmc);
	double getOutput();

private:
	unsigned int ID;
	NeuralForwardModel * forwardmodel;
	double threshold;
	double targetGrf,actualGrf,jmc,error;
};
//--------***ElevationReflex***----------------//
class ElevationReflex: public ANN{
public:
	ElevationReflex(unsigned int ID);
	~ElevationReflex();
	void step();
	void setInput(double _actualGrf,double _jmc);
	double getOutput();

private:
	unsigned int ID;
	NeuralForwardModel * forwardmodel;
	double threshold;
	double targetGrf,actualGrf,jmc,error;
};
//--------***LocalReflex***----------------//
class LocalReflex: public ANN{
public:
	LocalReflex(unsigned int ID);
	~LocalReflex();
	void step();
	void setInput(double _actualGrf,double _jmc);
	double getOutput(unsigned int index);

private:
	SearchingReflex * searching;
	ElevationReflex * elevation;
};


}

#endif

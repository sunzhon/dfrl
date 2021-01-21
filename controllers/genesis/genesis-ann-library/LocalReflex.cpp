#include "LocalReflex.h"

namespace stcontroller{
//----------forward model---------------//
NeuralForwardModel::NeuralForwardModel(){
	Wr = 2.5;
	Wi = 8;
	Bias =0;
	setNeuronNumber(2);
	setTransferFunction(1,logisticFunction());
	w(0,0,Wr);
	b(0,Bias);
	w(1,0,10.0);

}
void NeuralForwardModel::setInput(double _input){
	input =_input;
}
void NeuralForwardModel::step(){
	ANN::setInput(0,Wi*input);
	ANN::step();
}
double NeuralForwardModel::getOutput(){
	return ANN::getOutput(1);
}
//--------** LocalReflex***--------------------//

LocalReflex::LocalReflex(unsigned int _ID){
	searching = new SearchingReflex(_ID);
	elevation = new ElevationReflex(_ID);
}

LocalReflex::~LocalReflex(){
	delete searching;
	delete elevation;
}
void LocalReflex::setInput(double _jmc,double _actualGrf){

	searching->setInput(_jmc,_actualGrf);
	elevation->setInput(_jmc,_actualGrf);

}
void LocalReflex::step(){
	searching->step();
	elevation->step();
}
double LocalReflex::getOutput(unsigned int index){
	if(index == 0)	
		return searching->getOutput();
	else
		return elevation->getOutput();
}
//--------***SearchingReflex***----------------//
SearchingReflex::SearchingReflex(unsigned int _ID){
	ID = _ID;
	//1) adaptive neural forward model to predict the expected grf
	forwardmodel = new NeuralForwardModel();
	//2) searching network
	threshold =0.01;
	setNeuronNumber(1);
	setTransferFunction(0,identityFunction());
	
}

SearchingReflex::~SearchingReflex(){
	delete forwardmodel;
}

void SearchingReflex::setInput(double _actualGrf,double _jmc){//jmc == joint motor commands
	actualGrf =_actualGrf;
	jmc =_jmc;
}

void SearchingReflex::step(){
	forwardmodel->setInput(jmc);
	forwardmodel->step();
	targetGrf = forwardmodel->getOutput();
	error = targetGrf - actualGrf;
	ANN::setInput(0,targetGrf*error);//when targetGrf ==0, then the input also equal to zeros
	ANN::updateActivities();
	ANN::updateOutputs();
	ANN::setOutput(0,targetGrf*ANN::getOutput(0));// when targetGrf ==1, it is on
	
}

double SearchingReflex::getOutput(){
	return ANN::getOutput(0);
}

//-------------***elevation reflex*****--------------------//

ElevationReflex::ElevationReflex(unsigned int _ID){
	ID = _ID;
	//1) adaptive neural forward model to predict the expected grf
	forwardmodel = new NeuralForwardModel();
	//2) searching network
	threshold =0.01;
	setNeuronNumber(1);
	setTransferFunction(0,identityFunction());
}

ElevationReflex::~ElevationReflex(){
	delete forwardmodel;
}

void ElevationReflex::setInput(double _jmc,double _actualGrf){
	jmc=_jmc;
	actualGrf =_actualGrf;
}

void ElevationReflex::step(){
	forwardmodel->setInput(jmc);
	forwardmodel->step();
	targetGrf = forwardmodel->getOutput();
	error = targetGrf - actualGrf;
	ANN::setInput(0,(targetGrf-1)*error);// swing phase is enable
	ANN::updateActivities();
	ANN::updateOutputs();
	ANN::setOutput(0,(targetGrf-1)*ANN::getOutput(0));// stance phase is zeros
	
}

double ElevationReflex::getOutput(){
	return ANN::getOutput(0);

}

}

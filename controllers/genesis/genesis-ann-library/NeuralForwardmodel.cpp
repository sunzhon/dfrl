/* Neural forward model
 * Author: suntao
 * Date: 2019
 * Description: Neural forward model, recurrent neural network as a forward model for prediction sensory feedback
 *
 *
 */
#include "NeuralForwardmodel.h"

NeuralForwardmodel::NeuralForwardmodel(std::string _transfer,bool learning):count(0),nu(0.01),input(0.0),target(0.0)
{
	learning_state=learning;
	transfer = _transfer;
	setNeuronNumber(2);
	setTransferFunction(1,identityFunction());
	FFM = new ForceForwardmodel(1.0);

	if(!transfer.compare("logistic")){
		setTransferFunction(0,logisticFunction());
		Wi = 10.0;
		Wr=7.2;
		Bias=-6.0;
	}
	else{
		setTransferFunction(0,tanhFunction());
		Wi = 10.0;
		Wr=2.5;
		Bias=0.0;
	}
	if(learning_state==true){
		output=1.0;
		Err=0.0;

		Wi=0.0;
		Wr=0.0;
		Bias=0.0;
	}
		w(0,0,Wr);//7.2 --8.6
		b(0,Bias);
	w(FFM->getNeuron(0),getNeuron(0),-1.0);
	w(getNeuron(1),FFM->getNeuron(0),1.0);
}
NeuralForwardmodel::~NeuralForwardmodel(){
	delete FFM;
}
void NeuralForwardmodel::step(){
	ANN::setInput(0,Wi*input);
	w(0,0,Wr);
	b(0,Bias);
	ANN::step();
	FFM->step();
	output = FFM->getOutput();//ANN::getOutput(0);//FFM->getOutput();
}

void NeuralForwardmodel::stepLearning(double _target){
	
	target=_target;
	output_old=output;
	NeuralForwardmodel::step();
	if(learning_state==true){
			count+=1;
			if(count%200==0){
				Err=0.0;
			}
			Err += 1/2.0*(target-output)*(target-output);
			delta =nu*(target-output)*(1-output*output);
			deltaWr = delta*output_old;
			deltaWi = delta*input;
			deltaB = delta;
			Wr+=deltaWr;
			Wi+=deltaWi;
			Bias +=deltaB;
			if(count>1400){
				learning_state = false;
				std::cout<<"Learning finisied"<<std::endl;
			}
			std::cout<<"Count: "<<count<<std::endl;
			std::cout<<"ERR: "<<Err<<std::endl;
			std::cout<<"Wr: "<<Wr<<std::endl;
			std::cout<<"Wi: "<<Wi<<std::endl;
			std::cout<<"Bias: "<<Bias<<std::endl;
	}
}
void NeuralForwardmodel::setUp(double _Wi,double _Wr,double _Bias){
	Bias=_Bias;
	Wi=_Wi;
	Wr=_Wr;
	learning_state=false;// close the learning 
}
double NeuralForwardmodel::getOutput()
{
	return ANN::getOutput(1);
}

void NeuralForwardmodel::setInput(double _input){
	input = _input;
}

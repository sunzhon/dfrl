/***
Author:suntao
Date:2018
Description: It is a ann with hypersis properties
 It can be used to as a switcher, when input > for example 0.4, then the output is around 1.0, otherwise, the output is around zero. 

****/

#include "neuralpreprocessing.h"

NP::NP(std::string _transfer,bool learning):count(0),nu(0.01),input(0.0),target(0.0)
{
	transfer = _transfer;
	learning_state=learning;
	setNeuronNumber(1);
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
}
void NP::step(){
	ANN::setInput(0,Wi*input);
	w(0,0,Wr);
	b(0,Bias);
	ANN::step();
}

void NP::stepLearning(double _target){
	
	target=_target;
	output_old=output;
	NP::step();
	output=ANN::getOutput(0);

	// learning step
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
			if((Err < 1.0)&&(count%200==199)){
				learning_state = false;
				std::cout<<"Learning finisied"<<std::endl;
			}
				std::cout<<"ERR: "<<Err<<std::endl;
				std::cout<<"Wr: "<<Wr<<std::endl;
				std::cout<<"Wi: "<<Wi<<std::endl;
				std::cout<<"Bias: "<<Bias<<std::endl;
	}
}
void NP::setUp(double _Wi,double _Wr,double _Bias){
	Bias=_Bias;
	Wi=_Wi;
	Wr=_Wr;
	learning_state=false;// close the learning 
}
double NP::getOutput()
{
   return ANN::getOutput(0);
}

double NP::getWr(){
	return Err;
}
void NP::setInput(double _input){
	input = _input;
}

# include "FlexorReflex.h"

namespace stcontroller{

// the first period of swing phase, I calculate it from hip joint motion
SwF::SwF(){
	setNeuronNumber(1);
	FFM =new ForceForwardmodel(1.0);
	w(getNeuron(0),FFM->getNeuron(0),1.0);

	deltaSw=0;
	set=0;
	set_old=0;
	
	SwCount = 0;
	SwCount_old = 0;
	diffset = 0;
	output=0.0;
	input=0.0;
}
SwF::~SwF(){
	delete FFM;
}
void SwF::setInput(double _input){
	input=_input;// The input shuold be the hip joint motor command
	FFM->setInput(input);
}

double  SwF::getOutput(){
	return ANN::getOutput(0);
}

void SwF::step(){
	FFM->step();
	updateActivities();
	set_old=set;
	SwCount_old=SwCount;
	if(getActivity(0) < 0.1){
		SwCount+=1;
		set = 0;
	}else{
		SwCount = 0;
		set = 1;
	}
	diffset=set-set_old;
	if (SwCount== 0 && diffset== 1 && set == 1)
		deltaSw= SwCount_old;

	if((set==0)&&(SwCount<deltaSw/2))
		setOutput(0,1.0);//the first period of swing phase
	else
		setOutput(0,0.0);	
}

//---------------FlexorReflex class---------------------------------//
FlexorReflex::FlexorReflex(){

	obstacleDetectNp= new NP();
	delayhipNp= new NP();
	delaykneeNp= new NP();
	obstacleDetectNp->setUp(-8.0,8.0,-6.0);
	delayhipNp->setUp(6.0,7.8,-6.0);
	delaykneeNp->setUp(6.0,9.0,-6.0);
	obd=0.0;
	setNeuronNumber(3);
	setTransferFunction(0,identityFunction());
	setTransferFunction(1,identityFunction());
	setTransferFunction(2,identityFunction());
	w(delayhipNp->getNeuron(0),obstacleDetectNp->getNeuron(0),6.0);//unuseful
	w(delaykneeNp->getNeuron(0),obstacleDetectNp->getNeuron(0),6.0);//unuseful

	w(getNeuron(0),obstacleDetectNp->getNeuron(0),1.0);//unuseful
	w(getNeuron(1),delayhipNp->getNeuron(0),-1.0);
	w(getNeuron(2),delaykneeNp->getNeuron(0),+1.6);

}

void FlexorReflex::setInput(double _obd){
	obd=_obd;
}

void FlexorReflex::step(){
	
	obstacleDetectNp->setInput(obd);
	obstacleDetectNp->step();
	delayhipNp->step();
	delaykneeNp->step();
	ANN::step();
}
double FlexorReflex::getOutput(unsigned int id){
	assert(id < getNeuronNumber());
	if(id==0)
		return obstacleDetectNp->getOutput();
	else if(id==1)
		return ANN::getOutput(1);
	else
		return ANN::getOutput(2);
}

FlexorReflex::~FlexorReflex(){
	delete obstacleDetectNp;
	delete delayhipNp;
	delete delaykneeNp;
}

}

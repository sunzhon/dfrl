/***************************************************************************
 *   Copyright (C) 2012 by Dennis Goldschmidt                              *
 *    goldschmidt@physik3.gwdg.de                                          *
 *                                                                         *
 * This is a general forward model!                                        *
 *                                                                         *
 **************************************************************************/

#include "forwardmodel.h"
#include "../ann-framework/ann.h"
#include <vector>
using namespace std;

/******************************************************************************
 *     Construct forward model
 *****************************************************************************/

class ForwardModelANN: public ANN {
public:

	ForwardModelANN() {

		setNeuronNumber(6);
		//setTransferFunction(2, identityFunction());
		std::vector<double> w_w;
		std::vector<double> b_b;
		w_w.resize(9);
		b_b.resize(6);

		for (int i = 0; i < 9; i++)
			w_w.at(i) = 1.0;
		for (int i = 0; i < 6; i++)
			b_b.at(i) = 0.05;
		// synaptic weights
		w(2, 0, w_w.at(0));
		w(2, 1, w_w.at(1));

		w(3, 0, w_w.at(2));
		w(3, 1, w_w.at(3));

		w(4, 0, w_w.at(4));
		w(4, 1, w_w.at(5));

		w(5, 2, w_w.at(6));
		w(5, 3, w_w.at(7));
		w(5, 4, w_w.at(8));
		for (int i = 0; i < 6; i++)
			b(i, b_b.at(i));
	}
};

Forwardmodel::Forwardmodel(bool _learn_switch) { //Hier ANN generieren

	FModelANN = new ForwardModelANN();
	w20 = 0.01;
	w21 = 0.01;
	w30 = 0.01;
	w31 = 0.01;
	w40 = 0.01;
	w41 = 0.01;
	w52 = 0.01;
	w53 = 0.01;
	w54 = 0.01;
	counter = 0;
	learn_switch=_learn_switch;
	gama=0.799;

	if (learn_switch) // learning
	{
		learn_rate = 0.8;
	} else { // no learning
		learn_rate = 0.0;
	}

}

Forwardmodel::~Forwardmodel() {
	// delete neuronal net
	delete FModelANN;
}

double Forwardmodel::step(double motor_input2, double motor_input3,
		double sensor_input) {

	FModelANN->setInput(0, motor_input2);
	FModelANN->setInput(1, motor_input3);
	FModelANN->step();
	output = FModelANN->getOutput(5);
	if(output>output_old)
					G=0;
				else
					G=1;
	outputfinal=gama*G+(1-gama)*outputfinal_old;
	learn_error=sensor_input - outputfinal;
	cost_error = 0.5 * learn_error*learn_error;

	if ((cost_error < 0.05)&&(counter>1000))
		learn_switch = false;

	if (learn_switch ) {
		counter = counter + 1;
		w52 = w52
				- learn_rate * learn_error * outputfinal * (1 - outputfinal)
						* FModelANN->getOutput(2);
		w53 = w53
				- learn_rate * learn_error * outputfinal * (1 - outputfinal)
						* FModelANN->getOutput(3);
		w54 = w54
				- learn_rate * learn_error * outputfinal * (1 - outputfinal)
						* FModelANN->getOutput(4);
		//------------------
		S_delta = (outputfinal - FModelANN->getOutput(2) * w52) * w52
				* (1 - (FModelANN->getOutput(2)) * (FModelANN->getOutput(2)));
		w20 = w20
				- learn_rate * FModelANN->getOutput(2)
						* (1 - FModelANN->getOutput(2)) * (S_delta);

		S_delta = (outputfinal - FModelANN->getOutput(2) * w53) * w53
				* (1 - (FModelANN->getOutput(3)) * (FModelANN->getOutput(3)));
		w30 = w30
				- learn_rate * FModelANN->getOutput(3)
						* (1 - FModelANN->getOutput(3)) * (S_delta);

		S_delta = (outputfinal - FModelANN->getOutput(4) * w54) * w54
				* (1 - (FModelANN->getOutput(4)) * (FModelANN->getOutput(4)));
		w40 = w40
				- learn_rate * FModelANN->getOutput(4)
						* (1 - FModelANN->getOutput(4)) * (S_delta);
		//-------------------
		S_delta = (outputfinal - FModelANN->getOutput(2) * w52) * w52
				* (1 - (FModelANN->getOutput(2)) * (FModelANN->getOutput(2)));
		w21 = w21
				- learn_rate * FModelANN->getOutput(2)
						* (1 - FModelANN->getOutput(2)) * (S_delta);

		S_delta = (outputfinal - FModelANN->getOutput(2) * w53) * w53
				* (1 - (FModelANN->getOutput(3)) * (FModelANN->getOutput(3)));
		w31 = w31
				- learn_rate * FModelANN->getOutput(3)
						* (1 - FModelANN->getOutput(3)) * (S_delta);

		S_delta = (outputfinal - FModelANN->getOutput(4) * w54) * w54
				* (1 - (FModelANN->getOutput(4)) * (FModelANN->getOutput(4)));
		w41 = w41
				- learn_rate * FModelANN->getOutput(4)
						* (1 - FModelANN->getOutput(4)) * (S_delta);
	}
		//------------------------
		FModelANN->setWeight(2, 0, w20);
		FModelANN->setWeight(3, 0, w30);
		FModelANN->setWeight(4, 0, w40);

		FModelANN->setWeight(2, 1, w21);
		FModelANN->setWeight(3, 1, w31);
		FModelANN->setWeight(4, 1, w41);

		FModelANN->setWeight(5, 2, w52);
		FModelANN->setWeight(5, 3, w53);
		FModelANN->setWeight(5, 4, w54);

	return outputfinal;
}

double Forwardmodel::getOutputfinal()const{
	return outputfinal;
}
double Forwardmodel::getLearnerror()const{
	return learn_error;
}
double Forwardmodel::getW20()const{
	return w52;
}
double Forwardmodel::getCounter()const{
	return counter;
}

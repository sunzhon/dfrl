/*
 * legdelay.cpp
 *
 *  Created on: Oct 25, 2017
 *      Author: suntao
 */

#include "legdelay.h"

LegDelay::LegDelay(int tau) {
	// TODO Auto-generated constructor stub
	LDL.resize(3);
	this->tau=tau;
	for(std::vector<STDelayline *>::iterator itr=LDL.begin();itr!=LDL.end();itr++)
	*itr = new STDelayline(this->tau + 1/*size of delayline buffer*/);


}

LegDelay::~LegDelay() {
	// TODO Auto-generated destructor stub
}
void LegDelay::read(double & j1,double & j2,double & j3){
	j1=LDL.at(0)->Read(tau);
	j2=LDL.at(1)->Read(tau);
	j3=LDL.at(2)->Read(tau);
}

void LegDelay::write(double j1,double j2,double j3){
	LDL.at(0)->Write(j1);
	LDL.at(1)->Write(j2);
	LDL.at(2)->Write(j3);
}
void LegDelay::step(){
	LDL.at(0)->Step();
	LDL.at(1)->Step();
	LDL.at(2)->Step();

}

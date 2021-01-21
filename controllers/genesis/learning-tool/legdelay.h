/*
 * legdelay.h
 *
 *  Created on: Oct 25, 2017
 *      Author: suntao
 */
#include <learning-tool/stdelayline.h>
#include <vector>

class Delayline;

#ifndef LEGDELAY_H_
#define LEGDELAY_H_

class LegDelay {
public:
	LegDelay(int tau);
	void read(double &j1,double &j2,double &j3);
	void write(double j1,double j2,double j3);
	void step();
	virtual ~LegDelay();

private:
	std::vector<STDelayline *> LDL;//leg delay line
	int tau;
};

#endif /* LEGDELAY_H_ */

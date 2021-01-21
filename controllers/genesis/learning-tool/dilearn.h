/*
 * dilearn.h
 *
 *  Created on: Juane 23, 2019
 *      Author: suntao
 */

#ifndef DILEARN_H_
#define DILEARN_H_
#include <utils/ann-framework/ann.h>

#define TREND_UP true
#define TREND_DOWN false

class AngleForwardmodel: public ANN{
private:
	float alpha;
	float input;//motor input
	float output;//expected_force;
public:
	AngleForwardmodel(float alpha=0.9);

	virtual ~AngleForwardmodel();
	void setInput(float input);
	void step();
	float getOutput()const;
	void setAlpha(float alpha);

};


class DILearn {
public:
	DILearn();
	void step();
	void setInput(float expectedValue,float actualValue);
	float getOutput();
    void setParameters(float Af, float Bf, float Cf, float As, float Bs, float Cs);
    void setLimit(float uplimit, float downlimit);
    float getUpLimit();
    float getDownLimit();
private:
	//model
	//learn
	float Af;
	float As;
	float Bf;
	float Bs;
    float Cf;
    float Cs;

	float Ks;
	float Ks_old;
	float Kf;
	float Kf_old;
	float error;
	float integralError;
    float output;

	float expectedValue;
    float actualValue;

    float uplimit;
    float downlimit;

};

// adaptive DILearn
class ADILearn {
public:
	ADILearn();
	void step();
	void setInput(float expectedValue,float actualValue);
	float getOutput();
    void setParameters(float Af, float Bf, float Cf, float As, float Bs, float Cs);
    void setLimit(float uplimit, float downlimit);

private:
	//model
	//learn
	float Af;
	float As;
	float Bf;
	float Bs;
    float Cf;
    float Cs;

	float Ks;
	float Ks_old;
	float Kf;
	float Kf_old;
	float error;
    float error_old;
    float error_old_old;
    float error_delta;
    float error_threshold;
	float integralError;
    float output;

	float expectedValue;
    float actualValue;

    float uplimit;
    float downlimit;

};

#endif /* DILEARN_H_ */

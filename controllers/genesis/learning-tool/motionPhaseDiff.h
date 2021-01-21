#ifndef PHASESHIFT_H_
#define PHASESHIFT_H_
#include <vector>
#include <utility>
#include <math.h>
#include <time.h>
#include <assert.h>
#include <iostream>
#include <queue>
#include <map>
#include <algorithm>
#include "lowPassfilter.h"
#include "matrix.h"
using namespace matrix;
using namespace std;
typedef std::pair<clock_t,float> pcpgsig;
typedef std::pair<unsigned int long,clock_t>  peakpoint;
typedef std::map<unsigned int long,clock_t> peakmap;
typedef	std::vector<std::vector<float> > CMatrix;
//-------------------QueueVessel------------------------//
template<typename T>
class QueueVessel{
public:
	QueueVessel(T x, int size);
	void step();
	T getOutput();
	void setInput(T x);
	bool getState();
private:
	void Write(T x);
	T Read(int delay);
	int mod(int x,int m);
	void  updateExtreme();		
	void DeWrite(float x);
	float DeRead(int delay);
private:
	std::vector<T> buffer;
	std::vector<float> derivation;
	int steps;
	T extreme;
	bool capturestate;
};
//-------------------CapturePeak------------------------//
class CapturePeak{
public:
	CapturePeak();
	~CapturePeak();
	void setInput(pcpgsig sig);
	void step();
	peakmap getOutput();
private:
	pcpgsig signal;
	QueueVessel<pcpgsig> *qv;
	peakmap peak;
	unsigned long peakcount;
};
//-------------------CapturePeriod---------------------//
class CapturePeriod{
public:
	CapturePeriod();
	~CapturePeriod();
	void setInput(pcpgsig sig);
	void step();
	float getOutput();
private:
	CapturePeak * sig_capturepeak;
	peakmap sigpeak;
	float period;
};
//-------------------PhaseShift------------------------//
class PhaseShift{
public:
	PhaseShift();
	~PhaseShift();
	void setInput(pcpgsig sig1,pcpgsig sig2);
	void step();
	float getOutput();
private:
	CapturePeak * sig1cap;
	CapturePeak * sig2cap;
	peakmap sig1peak;
	peakmap sig2peak;
	std::vector<unsigned int> dis;// the distance between peaks in different signals
	float phaseshift;
	lowPass_filter *filter;

};
//-----------------MotionPhaseDiff-----------------//
class MotionPhaseDiff{
public:
	MotionPhaseDiff(unsigned int nCPGs);
	~MotionPhaseDiff();
	void setInput(unsigned int index, pcpgsig sig);
	void step();
	Matrix getOutput();

private:
	Matrix cmatrix;
	std::vector<pcpgsig> signals;
	std::vector<std::vector<PhaseShift *>> phase_shift;

};
//-------------------PhaseShiftV2------------------------//
class PhaseShiftV2{
public:
	PhaseShiftV2();
	~PhaseShiftV2();
	void setInput(peakmap peak1, peakmap peak2);
	void step();
	float getOutput();
private:
	peakmap sig1peak;
	peakmap sig2peak;
	float phaseshift;
	lowPass_filter *filter;
};

//-----------------MotionPhaseDiff-----------------//
class MotionPhaseDiffV2{
public:
	MotionPhaseDiffV2(unsigned int nCPGs);
	~MotionPhaseDiffV2();
	void setInput(unsigned int index, pcpgsig sig);
	void step();
	Matrix getOutput();

private:
	Matrix cmatrix;
	std::vector<pcpgsig> signals;
	std::vector<CapturePeak *> peak_capture;
	std::vector<std::vector<PhaseShiftV2 *>> phase_shift;

};

#endif

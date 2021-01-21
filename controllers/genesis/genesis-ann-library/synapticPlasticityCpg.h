#ifndef _SPCPG_H
#define _SPCPG_H
//#include <selforg/matrix.h>
#include <vector>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <iostream>
#include <fstream>
#include "utils/ann-framework/ann.h"
#include "utils/ann-framework/neuron.h"
#include "learning-tool/stdelayline.h"
#include "learning-tool/matrix.h"
using namespace matrix;
typedef float parameter;
typedef std::vector<parameter> vectorD;
typedef std::vector<std::vector<parameter>> matrixD;
//typedef std::vector<std::vector<parameter>> Matrix;

enum CPGSTYPE{
//0-5,CCPGs,6-DCPGs
	WALK_D = 0,
	WALK_L = 1,
	PACE = 2,
	GALLOP = 3,
	TROT = 4,
	DECOUPLED =5,
	SP =6,
	REFLEX =7
};
//------------sensory feedback function----//
class SF{
public:
	SF();
	void setInput(parameter gamma,parameter grf,vectorD a_t);
	void step();
	parameter getOutput(unsigned int index)const;
private:
	vectorD o_t;
	vectorD output;
	parameter grf;// grf
    parameter gamma;// adaptive feedback gain
};

//------------ACI class------------//
class ACI{
public:
	ACI(unsigned int ID,unsigned int nCPGs);
	void setInput(matrixD signals);
	void step();
	parameter getOutput(unsigned int index)const;
	void initDelta(parameter constant=0.0);
	void initK(parameter constant);
	void setK();
	void setDelta(CPGSTYPE CPGSType, Matrix cmatrix);
private:
	unsigned int ID;
	unsigned int nCPGs;
	vectorD output;
	matrixD cCPGat;
	matrixD delta;
	matrixD K;
	parameter C;
};
//-------------PhaseReset class---------//
class PhaseReset{
public:
	PhaseReset();
	void setInput(vectorD a_t,parameter grf);
	void step();
	parameter getOutput(unsigned int index)const;
private:
	vectorD a_t;
	vectorD a_t_old;
	vectorD reset;
	parameter grf;
	parameter grf_old;
	parameter Diracs;
};
//-----------PhaseInhibition class-----//
class PhaseInhibition{
public:
	PhaseInhibition();
	void setInput(parameter grf);
	void step();
	parameter getOutput(unsigned int index)const;
private:
	parameter Ch;
	parameter grf;
	parameter Tduration;
	vectorD inhibition;
};

//-------------Vestibular class-------------//
class Vestibular{
    public: 
        Vestibular(uint8_t leg);
        void setInput(vector<parameter> ori, parameter grf, vectorD a_t);
        void step();
        parameter getOutput(unsigned int index) const;
    private:
        parameter roll,pitch;
        parameter grf;
        parameter roll_gain,pitch_gain;
        vector<parameter> ves;
        vectorD a_t;
};
//-------------synaptic plasticity CPG class---------------//
class SPCPG :public ANN
{
    private:
        unsigned int ID;//identity of CPG
        unsigned int nCPGs;//the number of CPG
        parameter MI;
		vectorD a_t,a_t1;//activity of neuron
		vectorD o_t;//output of neuron
		//----- adaptive neural communication using adaptive control input------//
		ACI *aci;
		//-----adaptive physical communication using sensory feedback term---//
		SF * sf;
		// ----phase reset ------//
		PhaseReset * prs;
		// ---phase inhibition ----//
		PhaseInhibition *pib;	
		// ---vestibular response ----//
		Vestibular *ves;	
public:

		SPCPG(unsigned int ID,unsigned int nCPGs);
		virtual ~SPCPG();
		void updateWeights();
		void updateActivities();
		void updateOutputs();
        void step();
		parameter getOut0();
		parameter getOut1();
		parameter getACIOutput(unsigned int index)const;
		parameter getSFOutput(unsigned int index)const;
	
		void setMi(parameter mi);
        void setInput(parameter grf, parameter np_grf, vector<parameter> ori, parameter gain, matrixD signals, CPGSTYPE CPGType, Matrix cmatrix);
private:
    matrixD aci_signals;
    CPGSTYPE aci_CPGType;
    Matrix aci_cmatrix;

    parameter sf_np_grf;
    parameter sf_gain;

    vector<parameter> ves_ori;
    parameter ves_grf;

    parameter prs_np_grf;

    parameter pib_grf;
};
#endif

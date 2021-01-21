#ifndef MODULARNEURAL_H_
#define MODULARNEURAL_H_
//#include <selforg/controller_misc.h>
//#include <selforg/configurable.h>
//#include <selforg/types.h>
//#include <selforg/matrix.h>

#include <assert.h>
#include <time.h>
#include <cmath>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <iostream>
#include <stdio.h>
#include "genesis-ann-library/synapticPlasticityCpg.h"
#include "genesis-ann-library/doubleFunctionPcpg.h"
#include "genesis-ann-library/lin.h"
#include "genesis-ann-library/pmn.h"
#include "genesis-ann-library/neuralpreprocessing.h"
#include "genesis-ann-library/vestibularReflex.h"
#include "genesis-ann-library/dfrl.h"
#include "genesis-ann-library/touchReflex.h"
#include "genesis-ann-library/FlexorReflex.h"
#include "genesis-ann-library/ExtensorReflex.h"
#include "genesis-ann-library/LocalReflex.h"
#include "genesis-ann-library/ftnn.h"
#include "genesis-ann-library/ikinenn.h"
#include "genesis-ann-library/esnForwardmodel.h"

#include "learning-tool/delayneuron.h"
#include "learning-tool/dlearn.h"
#include "learning-tool/adaptiveFeedbackGain.h"
#include "learning-tool/frequencyAdaptation.h"

#include "utils/ann-framework/neuron.h"
#include "utils/ann-framework/synapse.h"
#include "utils/ann-framework/ann.h"

#include "utils/ann-library/vrn.h"
#include "utils/ann-library/psn.h"

using namespace matrix;
using namespace stcontroller;
//前置声明，在该类的定义之前，声明该类，告诉后面有这样一个类，在该类实现之前只能定义该类的指针，不能定义对象，只有
//在该类定义之后，才能定义该类的对象.  告诉编译器CPG,..是类类型。
class SPCPG;
class DFPCPG;
class VRN;
class PSN;
class NP;// neural preprocessing
class AdaptiveFeedbackGain;
class FrequencyAdaptation;
class AttitudeFeedback;
class BpNet;
class ForceForwardmodel;
class FlexorReflex;
class ExtensorReflex;
class LocalReflex;
class TouchReflex;
class ConnectionMatrix;
class PredefinedGait;
class FTNN;
class IkineNN;
//class DFRL;// DFFB reflex with online learning
//class DelayNeuron;

class ModularNeural : public ANN{
			public:
					ModularNeural(unsigned int nCPGs);
					virtual	~ModularNeural();
			// external input
			void setFootSensorForce(unsigned int ID,float _grf = 0.0);
			void setObstacleSensorForce(unsigned int ID,float _obd);
			void setJointSensorAngle(unsigned int ID,float _j1,float _j2,float _j3);
			void setCpgsType(CPGSTYPE CPGSType);
			void setACIMatrix(Matrix cmatrix);
			// step
			void step(unsigned int ID, CPGSTYPE CPGSType);
			
			void updateJointMotor(unsigned int ID);
			void updateMotorlimit(unsigned int ID);
			void updateAttitudeControl();
			//LIN
			float getInputNeuronOutput(unsigned int ID,int index)const;
			float getInputNeuronInput(unsigned int ID,unsigned int index)const;
			void setInputNeuronInput(unsigned int ID,float j1, float psn, float hipVrn,float kneeVrn); //left,right,forward,backward
			// CPG
			float getCpgOut0(unsigned int ID)const;
			float getCpgOut1(unsigned int ID)const;
			float getCpgFrequency(unsigned int ID)const;
			void setCpgMi(unsigned int ID,float mi);
			void setCPGGamma(unsigned int ID,float _g1);
			matrixD getmCPGsOutputs(unsigned int nCPGs);
			//PCPG
			float getPcpgOutput(unsigned int ID,int index)const;
			void setPCPGbeta(unsigned int ID,float _beta);
			//PSN
			float getPsnOutput(unsigned int ID,int index)const;
			//VRN
			float getHipVrnOutput(unsigned int ID)const;
			float getKneeVrnOutput(unsigned int ID)const;
			//PMN
			float getPmnOutput(unsigned int ID,int neuron_index)const;
			float getPMNOutput(unsigned int index)const;
			void setMNBias(unsigned int ID,float b1,float b2,float b3);
			//sensory feedback term in CPG
			float getSFOutput(unsigned int ID,unsigned int index)const;
            //adaptive controil input term in CPG
			float getACIOutput(unsigned int ID, unsigned int index)const;
            //frequency adaptation
			float getFAOutput(unsigned int ID)const;
            //adaptive feedback gain
			float getAFGOutput(unsigned int ID)const;
            //sensory adaptation of forward model
			float getAFGfmOutput(unsigned int ID)const;

			//ATF
			float getReflexes(unsigned int ID,unsigned int joint)const;// get attitute feedback output accumulation
			void setAttituteInput(vector<float> ori);
			//Vestibular Reflex
			float getVestibularReflexOutput(unsigned int ID, int index)const;
			// NeuralProcess work like a filter
			float getNPOutput(unsigned int ID)const;

            // distributed force feedback based (DFFB) reflex with online learning (DFRL)
            float getDFRLplasticWeight(unsigned int index)const;

            // reflexes switch, for choose and add reflexes into PMN of CPGs control
            void reflexesSwitch(unsigned int ID);
			private:
				std::vector<stcontroller::LIN*> mLINs;//输入神经元组,input neurons
				std::vector<SPCPG*> mCPGs;// CPGs
				std::vector<DFPCPG*> mPCPGs;//PCPGs
				std::vector<PSN*> mPSNs;// PMN
				std::vector<VRN*> hip_mVRNs;//sun tao  add this
				std::vector<VRN*> knee_mVRNs;// to regulate knee joint signal
				std::vector<DelayNeuron*> mDelays;
				std::vector<AdaptiveFeedbackGain *>mAFGs;// adaptive feedback gain
				std::vector<FrequencyAdaptation *>mFAs;//pinglv shiyingxing
				std::vector<PMN *> mPMNs;//运动神经元
				std::vector<stcontroller::VestibularReflex *> mVestiReflexs;//姿态控制
				std::vector<stcontroller::TouchReflex *> mTouchReflexs;// GRF distribution reflexes
				std::vector<stcontroller::FlexorReflex *> mFlexorReflexs;//flexor reflex to negotiate obstacle
				std::vector<stcontroller::ExtensorReflex *> mExtensorReflexs;//flexor reflex to negotiate obstacle
				std::vector<stcontroller::LocalReflex *> mLocalReflexs;//flexor reflex to negotiate obstacle
				std::vector<NP *> mNPs;//neural preprocessing for grf
                std::vector<FTNN *> mFTNNs;// foot trajectory neural network
                std::vector<IkineNN *> mIkineNNs;// foot trajectory neural network

                DFRL * dffbReflex;
				unsigned int n_CPGs;
				int tau,tau_l;
				float osc_couple0,osc_couple1;
				int previousID;
				float beta;
                float pitch_old=0.0;
                float pitch_changes=0.0;
                unsigned int pitch_steps=0;
				std::vector<std::vector<float>> reflex;//4legs,12 joints
                vector<float> ori;//机身姿态
				std::vector<float > grf;//actual足底力
				std::vector<float > np_grf;//neural preprocessing 足底力
				std::vector<std::vector<float> > jaf;//joint angle feedback
				std::vector<std::vector<float> > jmc;//joint motor command
				std::vector<float > obd;//obstacle detect
				Matrix cmatrix;			

                std::vector<ESNForwardmodel*> esnFMs;// ESN forward models
};
#endif

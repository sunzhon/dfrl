/*
 * dfrl.h
 *
 *  Created on: 2020-05-15
 *      Author: suntao
 *
 *  Description: It is to implement distributed force feedback-based reflex with online learning mechanism (DFRL)
 */

#ifndef DFRL_H_
#define DFRL_H_

#include <utils/ann-framework/ann.h>
#include <utils/ann-library/vrn.h>
#include <stdio.h>
#include <assert.h>
#include <iostream>
#include "learning-tool/lowPassfilter.h"
#include "learning-tool/dilearn.h"
#include "GRFsDistribution.h"


class lowPass_filter;
class GRFsDistribution;
class DILearn;

using namespace std;
namespace stcontroller {
    //------------------Optimizer for calculate the control action -----------//
    class Optimizer{
        public:
            Optimizer();
            ~Optimizer();
            void setInput(float current_error);
            void step();
            float getOutput();
        private:
            DILearn * dil;
            float output;
            float current_error, previous_error;
    };

    //-------------------Distribution force feedback-based (DFFB) reflex classes------------------//
    class ReflexInputLayer: public ANN{
        public:
            ReflexInputLayer(){
                setDefaultTransferFunction(identityFunction());
                setNeuronNumber(2);
                // This indicates the desired GRFs distribution (\gamma)
                b(0, -1.1);
                b(1, -1.1);
                w(1,0,-1.0);
            }

    };

    class ReflexHiddenLayer: public ANN{
        public:
            ReflexHiddenLayer(){
                setDefaultTransferFunction(identityFunction());        
                setNeuronNumber(1);
                w(0,0,1.0);
            }

    };

    class ReflexOutputLayer: public ANN{
        public:
            ReflexOutputLayer(){
                setDefaultTransferFunction(identityFunction());
                setNeuronNumber(2);
            }

    };


    //-------------------Distribution-force-feedback-based reflex with oneline learning (DFRL) class------------------//
    class DFRL : public ANN{
        public:
            DFRL(unsigned int ID);
            ~DFRL();
            void setInput(std::vector<float> grf);
            void step();
            float getOutput(unsigned int index);
            float getPlasticWeight(unsigned int index);
        private:
            //feedback GRFs distribution
            unsigned int ID;
            GRFsDistribution *GRFs_distribution;
            float feedback;
            float control_1, control_2;//for hip2 and knee joint of reflex outputs

            //optimization
            Optimizer *optimizer;
            float optimize_variable;


            //DFFB reflex neural network with three layers
            ReflexInputLayer* input_layer;
            ReflexHiddenLayer* hidden_layer;
            ReflexOutputLayer* output_layer;
            float w_1,w_2;

    };

} /* namespace stcontroller */

#endif /* DFRL_H_ */

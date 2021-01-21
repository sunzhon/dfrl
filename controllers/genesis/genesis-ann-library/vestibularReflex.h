/*
 * VestibularReflexNoStep.h
 *
 *  Created on: 2018-3-9
 *      Author: suntao
 */

#ifndef ATTIREFLEX_H_
#define ATTIREFLEX_H_

#include <utils/ann-framework/ann.h>
#include <utils/ann-library/vrn.h>
#include <stdio.h>
#include <assert.h>
#include <iostream>

using namespace std;
namespace stcontroller {
class VestibularReflex;
class VestibularReflexNeuralNetwork;
    enum{
        FR=0,
        RR=1,
        FL=2,
        RL=3
    };


    //-------VestibularReflexWithStep class--------------------//
    class VestibularReflex: public ANN{
        public:
            VestibularReflex(unsigned int ID);
            virtual ~VestibularReflex();
            void setInput(vector<float> ori, float grf);
            void updateOutputs();
            void step();
            float getOutput(unsigned int index);
        private:
            unsigned int ID;//identity
            unsigned int jointNumber;
            VestibularReflexNeuralNetwork* reflexNeuralNetwork;
            float Wori_r, Wori_p;
    };


    //------------------VestibularReflexNeuralNetwork class ----------------------------//
    class VestibularReflexNeuralNetwork: public ANN{
        public:
            VestibularReflexNeuralNetwork(unsigned int ID);
            virtual ~VestibularReflexNeuralNetwork();
            void setInput(float rollPitch);
            void step();
            float getOutput(unsigned int index);
        private:
            unsigned int ID;//identity
    };


} /* namespace stcontroller */

#endif /* ATTIREFLEX_H_ */

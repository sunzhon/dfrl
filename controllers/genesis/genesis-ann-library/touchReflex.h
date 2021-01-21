/*
 * touchReflex.h
 *
 *  Created on: 2020-02-03
 *      Author: suntao
 */

#ifndef TOUCHREFLEX_H_
#define TOUCHREFLEX_H_

#include <utils/ann-framework/ann.h>
#include <utils/ann-library/vrn.h>
#include <stdio.h>
#include <assert.h>
#include <iostream>
#include "neuralpreprocessing.h"
#include "learning-tool/lowPassfilter.h"
#include "learning-tool/dlearn.h"
#include "learning-tool/dilearn.h"

//using namespace shark;
class ForceForwardmodel;
class DILearn;
using namespace std;
namespace stcontroller {


    //------------------Touch Reflex class ----------------------------//
    class TouchReflex: public ANN{
        public:
            TouchReflex(unsigned int leg);
            virtual ~TouchReflex();
            void setInput(double grf, double hip_jmc);
            void step();
            double getOutput();
        private:

            unsigned int ID;//identity
            ForceForwardmodel * ffm;
            DILearn* dil;
            double grf;
            bool activate;
    };

} /* namespace stcontroller */

#endif /* TOUCHREFLEX_H_ */

/*
 * COGReflex.h
 *
 *  Created on: 2020-02-01
 *      Author: suntao
 */

#ifndef GRFSDISTRIBUTION_H_
#define GRFSDISTRIBUTION_H_

#include <utils/ann-framework/ann.h>
#include <utils/ann-library/vrn.h>
#include <stdio.h>
#include <assert.h>
#include <iostream>
#include "neuralpreprocessing.h"
#include "learning-tool/lowPassfilter.h"
#include "learning-tool/movingAverage.h"
#include "learning-tool/dilearn.h"

//using namespace shark;
class NP;
class lowPass_filter;
using namespace std;
namespace stcontroller {

    //-------------------COG distribution class------------------//
    class GRFsDistribution{
        public:
            GRFsDistribution(bool leftRight=false);// defualt to calculate the hind and front side
            ~GRFsDistribution();
            void setInput(vector<float> input);
            void step();
            float getOutput();
        private:
            lowPass_filter *NP_oneSideLegsGRF;
            lowPass_filter *NP_anotherSideLegsGRF;
            MovingAverage *NP_movingAverage;
            lowPass_filter *filterCOG;
            float output;
            float one_side_leg_grf;
            float another_side_leg_grf;
            bool leftRight;

    };

} /* namespace stcontroller */

#endif /* GRFSDISTRIBUTION_H_ */

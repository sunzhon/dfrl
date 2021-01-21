/*
 * manipulation.h
 *
 *  Created on: 2020-02-13
 *      Author: suntao
 */

#ifndef MANIPULATION_H_
#define MANIPULATION_H_

#include <utils/ann-framework/ann.h>
#include <utils/ann-library/vrn.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <iostream>
#include "learning-tool/lowPassfilter.h"
#include "learning-tool/dilearn.h"
#include "learning-tool/movingAverage.h"
#include "learning-tool/motionPhaseStability.h"

//using namespace shark;
class lowPass_filter;
class DILearn;
class MovingAverage;
using namespace std;
namespace stcontroller {

    //-------------------Manipulation class------------------//
    class Manipulation{
        public:
            Manipulation();
            ~Manipulation();
            void setInput(double expected_speed, double expected_direction, double actual_speed,double actual_direction,double pitch,double hip_jmc,double knee_jmc);
            void step();
            double getHipVRNOutput(unsigned int index);
            double getKneeVRNOutput(unsigned int index);
            double getPSNOutput(unsigned int index);
            double getMIOutput(unsigned int index);
        private:
            lowPass_filter *speed_filter;
            MovingAverage *direction_filter;
            lowPass_filter *pitch_filter;
            double mi,psn,hip_vrn,knee_vrn;
            double expected_direction,expected_speed,actual_direction,actual_speed;
            double pitch;// body picth
            DILearn* mi_dil;
            DILearn* vrn_dil;

            CircleQueue<double>* hip_jmc_bias;
            CircleQueue<double>* knee_jmc_bias;

    };
} /* namespace stcontroller */

#endif /* MANIPULATION_H_ */

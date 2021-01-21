/*
 * manipulation.cpp
 *  Created on: 2020-02-13
 *      Author: suntao
 */

#include "manipulation.h"
using namespace stcontroller;
namespace stcontroller {

//----------------Manipulation class -------------------------------//
Manipulation::Manipulation(){

    speed_filter=new lowPass_filter();
    direction_filter=new MovingAverage(1,60);

    mi_dil=new DILearn();
    mi_dil->setParameters(0.65,0.045,0.0005,0.85,0.015,0.00005);
    mi_dil->setLimit(0.4,0.0);

    vrn_dil=new DILearn();
    vrn_dil->setParameters(0.635,0.032,0.000005,0.85,0.010,0.000001);
    vrn_dil->setLimit(0.015,-0.015);

    pitch_filter=new lowPass_filter();

    hip_jmc_bias = new CircleQueue<double>(0.0,60);
    knee_jmc_bias = new CircleQueue<double>(0.0,60);

    mi=0.0;
    psn=0.0;
    hip_vrn=0.0;
    knee_vrn=0.0;
    expected_direction = 0.0;
    actual_direction = 0.0;
    expected_speed =0.0;
    actual_speed = 0.0;
    pitch=0.0;
}

Manipulation::~Manipulation(){
    delete speed_filter;
    delete direction_filter;
    delete mi_dil;
    delete vrn_dil;
    delete hip_jmc_bias;
    delete knee_jmc_bias;
}

double Manipulation::getHipVRNOutput(unsigned int index){
    assert(index<4);
    switch(index){
        case 0:
        case 1:
            return -hip_vrn;
        case 2:
        case 3:
            return hip_vrn;
        default:
            perror("mainpulation classs has error\n");
            return -1;

    }
}

double Manipulation::getKneeVRNOutput(unsigned int index){
    assert(index<4);
    switch(index){
        case 0:
        case 1:
            return -knee_vrn;
        case 2:
        case 3:
            return knee_vrn;
        default:
            perror("mainpulation classs has error\n");
            return -1;

    }
}

double Manipulation::getMIOutput(unsigned int index){
    assert(index<4);
            return mi;
}

double Manipulation::getPSNOutput(unsigned int index){
    assert(index<4);
            return psn;
}

void Manipulation::setInput(double expected_speed, double expected_direction, double actual_speed, double actual_direction,double pitch, double hip_jmc, double knee_jmc){

    assert(abs(expected_direction)<3.14/2);
    this->expected_speed=expected_speed;
    this->expected_direction=expected_direction;
    this->actual_speed=speed_filter->update(actual_speed);
    this->actual_direction=direction_filter->update(actual_direction);
    this->pitch=pitch_filter->update(pitch);// just on body is level, then adjust direction

    hip_jmc_bias->write(hip_jmc);
    knee_jmc_bias->write(knee_jmc);
}

void Manipulation::step(){
    //if(abs(pitch)<0.6){// just when body is level to adjust walking direction, since the step length is decided bt hip joint amplitudes in this case
        mi_dil->setInput(expected_speed,actual_speed);
        mi_dil->step();
        mi=mi_dil->getOutput();

        vrn_dil->setInput(expected_direction,actual_direction);
        vrn_dil->step();
        hip_vrn=(0.76+hip_jmc_bias->getMedian())*vrn_dil->getOutput();
        knee_vrn=(0.53-knee_jmc_bias->getMedian())*vrn_dil->getOutput();
        hip_jmc_bias->step();
        knee_jmc_bias->step();


        if(expected_speed>=0.0)
            psn=0.0;
        else
            psn=1.0;
    //}else{

      //  hip_vrn=0.0;
      //  knee_vrn=0.0;
   // }
    //printf("exp:%0.3f\t,act:%0.3f\t,vrnH:%0.3f,vrnK:%0.3f\n",expected_direction,actual_direction,hip_vrn,knee_vrn);
    //printf("hip_bias:%0.3f\t knee_bias:%0.3f\n",hip_jmc_bias->getMedian(),knee_jmc_bias->getMedian());

}


} /* namespace stcontroller */

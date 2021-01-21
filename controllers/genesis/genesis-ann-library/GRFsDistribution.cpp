/*
 * GRFsDistribution.cpp
 *  Created on: 2020-02-01
 *      Author: suntao
 */

#include "GRFsDistribution.h"
using namespace stcontroller;
namespace stcontroller {

    //----------------GRFsDistribution class -------------------------------//
    GRFsDistribution::GRFsDistribution(bool leftRight){
        /**
         * @descrption: Calculate the forces (GRFs) distribution between front and hind legs or left and right legs
         * @param: defualt is to calculate front and hind sides
         * There are four objects created. 
         */
        this->leftRight=leftRight;

        NP_oneSideLegsGRF=new lowPass_filter(0.3);
        NP_anotherSideLegsGRF=new lowPass_filter(0.3);
        NP_movingAverage=new MovingAverage(1,60);//60
        filterCOG=new lowPass_filter(0.3);
    }

    GRFsDistribution::~GRFsDistribution(){
        delete NP_oneSideLegsGRF;
        delete NP_anotherSideLegsGRF;
        delete NP_movingAverage;
        delete filterCOG;
    }

    float GRFsDistribution::getOutput(){
        return output;
    }

    void GRFsDistribution::setInput(vector<float> input){
        /**
         * @param: input, GRF (ground reaction forces) of all legs
         * @description: depeingding on leftRight ( a bool parameter) to set which sides are used to compare, default is front and hind sides
         * @return: void.
         */ 
        assert(input.size()==4);// four legs, please set it as the leg numbers
        if(leftRight){// between left and right side
            one_side_leg_grf=input.at(0)+input.at(1);   //right side
            another_side_leg_grf=input.at(2)+input.at(3);   //left side
        }else{
            one_side_leg_grf=input.at(0)+input.at(2);   //front side
            another_side_leg_grf=input.at(1)+input.at(3);   //right side
        }
    }

    void GRFsDistribution::step(){
        float temp=0.0;
        if((one_side_leg_grf>0.1)&&(another_side_leg_grf>0.1)){//this means the feet should be in stance phase
            temp = NP_oneSideLegsGRF->update(one_side_leg_grf)/NP_anotherSideLegsGRF->update(another_side_leg_grf);
        }
        //moving average filer and a low pass filter
        output=filterCOG->update(NP_movingAverage->update(temp));
    }
} /* namespace stcontroller */

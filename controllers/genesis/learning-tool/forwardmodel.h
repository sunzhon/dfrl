/*
 * forwardmodel.h
 *  Created on: 1 3, 2018
 *      Author: suntao
 */

#ifndef FORWARDMODEL_H_
#define FORWARDMODEL_H_

#include <vector>
#include <iostream>
#include <cstdlib>
#include "utils/ann-framework/ann.h"
using namespace std;


class ForwardModelANN;


class Forwardmodel {
public:
    Forwardmodel( bool _learn_witch);
    virtual ~Forwardmodel();
    double getOutputfinal()const;
    double getLearnerror()const;
    double getW20()const;
    double getCounter()const;
    double step(double motor_input2,double motor_input3, double sensor_input);




private:

    double learn_rate;
    bool learn_switch;
    double learn_error;
    double cost_error;
    double outputfinal;

    double outputfinal_old;
    double output,output_old;
    double gama,G;

    double S_delta;
    double counter;

    double w20;
    double w21;

    double w30;
    double w31;

    double w40;
    double w41;

    double w52;
    double w53;
    double w54;

  private:
    ForwardModelANN* FModelANN;
};

#endif /* FORWARDMODEL_H_ */

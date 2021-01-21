#ifndef ESNFORWARDMODEL_H_
#define ESNFORWARDMODEL_H_
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include "networkmatrix.h"


class ESNetwork;

class ESNForwardmodel{
    public:
        ESNForwardmodel(unsigned int ID=0, std::string dir="./", unsigned int  n_input=1, unsigned int n_output=1, unsigned int n_hidden=250, bool feedback=false, bool feeding_input=false, float leak=0.0);
        ~ESNForwardmodel();
        void stepLearning();
        void stepPredict();
        void setDataSet(std::vector<float> inputs,std::vector<float> targets);
        float getOutput(unsigned int idx);
        void loadModel(unsigned int id, const std::string& dir);
        void saveModel(unsigned int id, const std::string& dir);
        void setParameters(unsigned int transfer_func_out=0/*linear*/, unsigned int transfer_func_hidden=2/*tanh*/, unsigned int esn_type=2/*stand esn learning*/, unsigned int input_sparsity=0/*input connect to all hidden neurons*/, float input_range=1.0/* [-1.0,1.0]*/, unsigned int  learn_mode=1/*RLS*/, bool load_weight=false/*not load learned weight*/, float noise_range=0.001, bool RCneuronNoise=false/*constant fixed bias*/);

        bool getLearned();

    private:
        std::string dir;
        ESNetwork * ESN;
        unsigned int n_input;
        unsigned int n_output;
        unsigned int n_hidden;
        float * ESinput;
        float * ESoutput;
        float * ESTrainOutput;
        unsigned int ID;
        unsigned int global_count;
        unsigned int learning_episodes;
        bool learn;       
};

#endif

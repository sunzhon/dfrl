/*
 * Author: suntao
 * Date: 2020-3-4
 * Description: This is a echo state network designed for forward model for predicting sensory using commands
 *
 */
#include "esnForwardmodel.h"

ESNForwardmodel::ESNForwardmodel(unsigned int ID, std::string dir, unsigned int  n_input, unsigned int n_output, unsigned int n_hidden, bool feedback, bool feeding_input, float leak){
    // ESN ID
    this->ID=ID;
    // path that store ESN
    this->dir=dir;
    // dimensions of the ESN
    this->n_input=n_input;
    this->n_output=n_output;
    this->n_hidden=n_hidden;
    //Setting ENS parameters
    ESN = new ESNetwork(n_input/*no. input*/,n_output /*no. output*/,n_hidden /*rc hidden neurons*/,
            feedback /*feedback*/, feeding_input /*feeding input to output*/, leak /*leak = 0.0*/, false/*IP*/);


    ESN->autocorr = pow(10,3); // set as high as possible, default = 1
    ESN->generate_random_weights(50 /*10% sparsity = 90% connectivity */, 0.95 /*1.2-1.5 = chaotics*/);
    //Create ESN input vector
    ESinput = new float[n_input];
    ESoutput = new float[n_output];

    //Create ESN target output vector
    ESTrainOutput = new float[n_output];

    //Initial values of input and target output
    for(unsigned int i = 0; i < n_input; i++)
    {
        ESinput[i] = 0.0;
    }
    for(unsigned int i = 0; i< n_output; i++)
    {
        ESTrainOutput[i] = 0.0;
    }
    learn=true;
    learning_episodes=500;
}

void ESNForwardmodel::setParameters(unsigned int transfer_func_out, unsigned int transfer_func_hidden, unsigned int esn_type, unsigned int input_sparsity, float input_range, unsigned int  learn_mode, bool load_weight, float noise_range, bool RCneuronNoise){

    ESN->outnonlinearity = transfer_func_out; // 0 = linear, 1 = sigmoid, 2 = tanh: transfer function of an output neuron

    ESN->nonlinearity = transfer_func_hidden; // 0 = linear, 1 = sigmoid, 2 = tanh: transfer function of all hidden neurons

    ESN->withRL = esn_type; // 2 = stand ESN learning, 1 = RL with TD learning

    ESN->InputSparsity = input_sparsity; // if 0 = input connects to all hidden neurons, if 100 = input does not connect to hidden neurons


    ESN->InputWeightRange = input_range; // scaling of input to hidden neurons, default 0.15 means [-0.15, +0.15]

    ESN->LearnMode = learn_mode;//RLS = 1. LMS =2

    ESN->Loadweight = load_weight; // true = loading learned weights

    ESN->NoiseRange = noise_range; //

    ESN->RCneuronNoise = RCneuronNoise; // false = constant fixed bias, true = changing noise bias every time
}

ESNForwardmodel::~ESNForwardmodel(){
    //----- ESN objects garbage collection ---- //
    delete []ESN;
    delete []ESinput;
    delete []ESoutput;
    delete []ESTrainOutput;
}

void ESNForwardmodel::setDataSet(std::vector<float> inputs, std::vector<float> targets){
    for(unsigned int idx=0;idx<n_input;idx++)
        ESinput[idx] = inputs.at(idx);// Input

    for(unsigned int idx=0;idx<n_output;idx++)
        ESTrainOutput[idx]= targets.at(idx); //Training output (target function)
}

void ESNForwardmodel::stepLearning(){
    if(global_count == learning_episodes){
    // whether to learning initialization
        learn=false;
        //std::cout << "initial learn finish, save training"<<std::endl;
        //1.2) save training
        saveModel(ID,dir);
    }
    //1.1) training or running depeding on learn arg
    ESN->setInput(ESinput, n_input/* no. input*/);
    //printf("esn---:%0.2f, %0.2f\n",ESinput[0], ESTrainOutput[0]);
    ESN->takeStep(ESTrainOutput /*targets*/, 0.9 /*learning rate 0.9*/, 1 /*no td = 1 else td_error*/, true/* true= learn, false = not learning learn_critic*/);

    global_count+=1;
}

void ESNForwardmodel::stepPredict(){
    //1.1) training or running depeding on learn arg
    ESN->setInput(ESinput, n_input/* no. input*/);
    //printf("esn---:%0.2f, %0.2f\n",ESinput[0], ESTrainOutput[0]);
    ESN->takeStep(ESTrainOutput /*targets*/, 0.9 /*learning rate 0.9*/, 1 /*no td = 1 else td_error*/, false/* true= learn, false = not learning learn_critic*/);
    for(unsigned int idx=0;idx<n_output;idx++)
        ESoutput[idx] = ESN->outputs->val(idx, 0);
}


float ESNForwardmodel::getOutput(unsigned int idx){
    return ESoutput[idx];
}

void ESNForwardmodel::loadModel(unsigned int id, const std::string& dir){
    // load model from a file
    ESN->readParametersFromFile(id, dir);
    ESN->readStartweightsFromFile(id, dir);
    ESN->readEndweightsFromFile(id, dir);
    ESN->readInnerweightsFromFile(id, dir);
    ESN->readNoiseFromFile(id, dir);
}

void ESNForwardmodel::saveModel(unsigned int id, const std::string & dir){
    ESN->writeParametersToFile(id, dir);
    ESN->writeStartweightsToFile(id, dir);
    ESN->writeEndweightsToFile(id, dir);
    ESN->writeInnerweightsToFile(id, dir);
    ESN->writeNoiseToFile(id, dir);
}

bool ESNForwardmodel::getLearned(){
    return !learn;
}

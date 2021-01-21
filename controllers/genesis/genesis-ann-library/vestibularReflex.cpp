/*
 * vestibularReflex.cpp
 * 
 * The algorith got inspiration from vestibular reflex mechanism of animals
 *  Created on: 2018-3-9
 *  Modified on: 2020-5-1
 *      Author: suntao
 */

#include "vestibularReflex.h"
using namespace stcontroller;
namespace stcontroller {

    //-----------------VestibularReflex class -----------------------//
    VestibularReflex::VestibularReflex(unsigned int ID){
        // Corrdinatiion among four legs
        this->ID =ID;
        float temp1=75;// 75 roll
        float temp2=70;// pitch  
        switch (ID) {
            case FR:
                Wori_r = temp1;
                Wori_p = temp2;
                break;
            case RR:
                Wori_r = temp1;
                Wori_p = -temp2;
                break;
            case FL:
                Wori_r = -temp1;
                Wori_p = temp2;
                break;
            case RL:
                Wori_r = -temp1;
                Wori_p = -temp2;
                break;
            default:
                perror("leg number is error in VestibularReflexNoStep");
        }
        // neural network of vestibular reflexes
        reflexNeuralNetwork = new VestibularReflexNeuralNetwork(ID);

        jointNumber=3; // joint numbers of a leg
        setNeuronNumber(jointNumber);
        for(unsigned int idx=0;idx<jointNumber;idx++)
            setTransferFunction(idx,identityFunction());
    }

    VestibularReflex::~VestibularReflex(){
        delete reflexNeuralNetwork;
    }

    void VestibularReflex::setInput(vector<float> ori, float grf) {
        assert(ori.size()>=3);
        float rollPitchInput = Wori_r * ori.at(0)/*roll*/ + Wori_p * ori.at(1) /*pitch*/;
        reflexNeuralNetwork->setInput(rollPitchInput);// set input of its first neuron
    }
    
    void VestibularReflex::step(){
        ANN::updateActivities();
        VestibularReflex::updateOutputs();
    }
    void VestibularReflex::updateOutputs(){
        reflexNeuralNetwork->step();
        ANN::setOutput(0,reflexNeuralNetwork->getOutput(4));
        ANN::setOutput(1,0.65*reflexNeuralNetwork->getOutput(5));// 0.3 is a gain
        ANN::setOutput(2,0.5*reflexNeuralNetwork->getOutput(6));//0.5 is a gain , depends on l_1 and l_2
    }

    float VestibularReflex::getOutput(unsigned int index){
        assert(index<jointNumber);
        return ANN::getOutput(index);
    }


    //-----------------VestibularReflexNeuralNetwork class -----------------------//
    VestibularReflexNeuralNetwork::VestibularReflexNeuralNetwork(unsigned int ID) {
        /**
         *
         * Description: filter and process orientation signals and output 
         * @param: ID is the identification of the object
         */
        this->ID=ID;
        //construct the nerual network of reflexes

        setNeuronNumber(7);
        setTransferFunction(0,identityFunction());
        setTransferFunction(1,logisticFunction());
        setTransferFunction(2,logisticFunction());
        setTransferFunction(3,identityFunction());
        setTransferFunction(4,identityFunction());
        setTransferFunction(5,identityFunction());
        setTransferFunction(6,identityFunction());

        // neuron 0-3, filte the inputs
        w(1,0,1.0);
        w(2,0,1.0);
        w(3,1,1.0);
        w(3,2,1.0);
        b(1,+4.0);
        b(2,-4.0);
        b(3,-1.0);

        //connect to three output neurons    
        w(4,3,0.0); // hip 1 joint Elevation and Depression
        w(5,3,0.011); // 0.011 hip 2 joint Froward and Backward
        w(6,3,-0.015); // -0.015 knee joint Extension and Flexsion
        //output neurons has a recurrent
        w(5,5,0.999);
        w(6,6,0.999);

    }

    VestibularReflexNeuralNetwork::~VestibularReflexNeuralNetwork() {
        // TODO Auto-generated destructor stub
    }

    void VestibularReflexNeuralNetwork::setInput(float rollPitch) {
        ANN::setInput(0,rollPitch);
    }

    void VestibularReflexNeuralNetwork::step() {
        //1) update
        ANN::updateWeights();
        ANN::updateActivities();
        ANN::updateOutputs();
    }

    float VestibularReflexNeuralNetwork::getOutput(unsigned int index) {
        /**
         * @param index is the output neuron index
         * @return the output of the index output neuron 4,5,6
         */
        assert(index>=4);
        return ANN::getOutput(index);//0--4; 1--5; 2--6
    }



} 

/* namespace stcontroller */

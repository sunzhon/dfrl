#include "dfrl.h"

using namespace stcontroller;

bool Dropped=false;
bool OPENDIL=false;

namespace stcontroller{

    DFRL::DFRL(unsigned int ID){
        this->ID=ID;

        //1) reflex neural network---DFFB reflex neural network
        input_layer= new ReflexInputLayer();
        hidden_layer = new ReflexHiddenLayer();
        output_layer = new ReflexOutputLayer();
        // neural interace to output side
        setNeuronNumber(2);
        ANN::setDefaultTransferFunction(identityFunction());

        // plastic weights
        w_1 = 0.003;//0.003;//0.0021;//0.006
        w_2 = 0.0032;//0.0032;//0.0022;//0.006

        w(hidden_layer->getNeuron(0),input_layer->getNeuron(0),w_1);
        w(hidden_layer->getNeuron(0),input_layer->getNeuron(1),w_2);

        w(output_layer->getNeuron(0), hidden_layer->getNeuron(0), 2.0); // hip2 0.72*2
        w(output_layer->getNeuron(1), hidden_layer->getNeuron(0), 1.0); // knee 0.44*2


        // feedback compuate COG gamma
        GRFs_distribution = new GRFsDistribution();//beweent front hind legs

        // optimer outputs manipulation variable, DIL
        optimizer= new Optimizer();
        feedback=0.0;
        optimize_variable=0.0;
        control_1=0.0;
        control_2=0.0;
    }

    DFRL::~DFRL(){
        delete GRFs_distribution;
        delete optimizer;

        // delete neural network of reflex computation
        delete input_layer;
        delete hidden_layer;
        delete output_layer;
    }

    void DFRL::step(){
        //1) calculate GRF distribution
        GRFs_distribution->step();
        feedback=GRFs_distribution->getOutput();
        if(feedback>0.0)
            Dropped=true;
        if(Dropped){// To make sure whether the robot is dropped on the ground
            //1) update the input layer
            input_layer->setInput(0,feedback);
            input_layer->setInput(1,feedback);
            input_layer->step();
            optimizer->setInput(input_layer->getOutput(0));

            //2) update plastic synapses by DIL
            if(OPENDIL){// open the DIL in the second step after the Dropped is true
                optimizer->step();
                optimize_variable=optimizer->getOutput();
                w(hidden_layer->getNeuron(0),input_layer->getNeuron(0), optimize_variable+w_1);
                w(hidden_layer->getNeuron(0),input_layer->getNeuron(1), optimize_variable+w_2);
            }
            OPENDIL=true;
            //3) update the hidden layer
            hidden_layer->step();

            //4) update the output layer
            output_layer->step();

            //5 get output
            control_1 = output_layer->getOutput(0);
            control_2 = output_layer->getOutput(1);
            //neural interface to outside
            ANN::setOutput(0,control_1);
            ANN::setOutput(1,control_2);
        }
       //printf("GRFs distribution: %0.3f, reflex:%0.3f, error:%0.3f\n", feedback, control_1, input_layer->getOutput(0));

    }

    void DFRL::setInput(std::vector<float> grf){
        assert(grf.size()==4); //the number of legs equal 4
        GRFs_distribution->setInput(grf);
    }

    float DFRL::getPlasticWeight(unsigned int index){
        assert(index<2);
        if(index==0)
            return getWeight(hidden_layer->getNeuron(0),input_layer->getNeuron(0));
        else if(index==1)
            return getWeight(hidden_layer->getNeuron(0),input_layer->getNeuron(1));
        else{
            perror("DFRL output index error");
            return 0;
        }
    }

    float DFRL::getOutput(unsigned int index){
        assert(index<3);
        switch(index){
            case 0:
                return 0;
            case 1:
                return control_1;
            case 2:
                return control_2;
            default:
                perror("DFRL output index error");
                return -1;

        }
    }


    /************************Optimization****************************************/
    Optimizer::Optimizer(){
        output =0.0;
        current_error = 0.0;
        previous_error = 0.0;
        // dual integral learn (DIL)
        dil= new DILearn();
        dil->setParameters(0.01, 0.05, 0.001,     0.1, 0.01, 0.0001); //
        dil->setLimit(.01,-.01);
    }

    Optimizer::~Optimizer(){
        delete dil;

    }

    void Optimizer::setInput(float current_error){
        this->previous_error = this->current_error;
        this->current_error = current_error;
    }

    void Optimizer::step(){
        dil->setInput(abs(previous_error) /*expected*/,abs(current_error)/*actual*/);
        dil->step();
        output = dil->getOutput();// incrase the weight if has error
    }

    float Optimizer::getOutput(){
        // return control actiion as as control variable
        return output;
    }


} /* namespace stcontroller */

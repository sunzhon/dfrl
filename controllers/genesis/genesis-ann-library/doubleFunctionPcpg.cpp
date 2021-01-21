#include "doubleFunctionPcpg.h"
//孙涛 add  pcpg_type

DFPCPG::DFPCPG() :
    set(2), countup(2), countdown(2) {

        /*******************************************************************************
         *  MODULE 2 CPG POST-PROCESSING
         *******************************************************************************/
        pcpg_step.resize(2);
        setold.resize(2);
        countupold.resize(2);
        countdownold.resize(2);
        diffset.resize(2);
        // zhenliang 
        deltaxdown.resize(2);
        deltaxup.resize(2);
        deltaxup.at(0)=100;//chu shi hua
        deltaxup.at(1)=100;
        deltaxdown.at(0)=100;
        deltaxdown.at(1)=100;
        xup.resize(2);
        xdown.resize(2);

        yup.resize(2);
        ydown.resize(2);

        pcpg_output.resize(2);

        setNeuronNumber(2);
        beta.resize(2);//0.85;

    }
void DFPCPG::updateOutputs() {
    /*
    //CPG post processing
    for (unsigned int i=0; i < pcpg_output.size(); i++) {
        pcpg_step.at(i) = getActivity(i);
        setold.at(i) = set.at(i);
        countupold.at(i) = countup.at(i);
        countdownold.at(i) = countdown.at(i);

        //1) Linear threshold transfer function neuron 1 , or called step function neuron//

        if (pcpg_step.at(i) >= beta.at(i)) //Intuitively select high level
            set.at(i) = 1.0;//1.0 
        else
            set.at(i) = -1.0;//-1.0

        diffset.at(i) = set.at(i) - setold.at(i); // double

        //2) Count how many step of Swing
        if (set.at(i) == 1.0) {
            countup.at(i) = countup.at(i) + 1.0; //Delta x0 up
            countdown.at(i) = 0.0;
        }
        // Count how many step of Stance
        else if (set.at(i) == -1.0) {
            countdown.at(i) = countdown.at(i) + 1.0; //Delta x0 down
            countup.at(i) = 0.0;
        }

        //3) Memorized the total steps of swing and stance
        if (countup.at(i) == 0.0 && diffset.at(i) == -2.0 && set.at(i) == -1.0)
            deltaxup.at(i) = countupold.at(i);

        if (countdown.at(i) == 0.0 && diffset.at(i) == 2.0 && set.at(i) == 1.0)
            deltaxdown.at(i) = countdownold.at(i);

        //4) Comput y up and down !!!!
        //xup.at(i) = countup.at(i);
        //xdown.at(i) = countdown.at(i);
        
       //yup.at(i) = ((2. / deltaxup.at(i)) * xup.at(i)) - 1;
        //ydown.at(i) = ((-2. / deltaxdown.at(i)) * xdown.at(i)) + 1;
       // if(set.at(i)>=0)
       // pcpg_output.at(i)=yup.at(i);
       // else
       // pcpg_output.at(i)=ydown.at(i);

        if(set.at(i)>=0.0){
        yup.at(i) = (1.0-ydown.at(i))/deltaxup.at(i) * xup.at(i) + ydown.at(i);
        pcpg_output.at(i)=yup.at(i);
        }else{
        ydown.at(i) = (-1-yup.at(i))/deltaxdown.at(i) * xdown.at(i) + yup.at(i);
        pcpg_output.at(i)=ydown.at(i);
        }

    //5) Combine y up and down !!!!
    //Limit upper and lower boundary
    pcpg_output.at(i) = (pcpg_output.at(i) > 1.0) ? 1.0 : ((pcpg_output.at(i) < -1.0) ? -1.0 : pcpg_output.at(i));
    //CPG post processing
    setOutput(i, pcpg_output.at(i));
    }
    */
    ANN::updateOutputs();
}
void DFPCPG::step(){
    ANN::updateActivities();
    updateOutputs();
}
void DFPCPG::setBeta(double _beta){
    beta.at(0)=_beta;
    beta.at(1)=_beta+0.05;
}

void DFPCPG::setInput(int index,double value){
    assert(index<2);
}

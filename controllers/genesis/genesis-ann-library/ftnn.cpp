/**
Author:suntao
Date: 2018-11
Description: It is used to create foot trajectory using neural network

**/
#include "ftnn.h"
FTNN::FTNN(unsigned int ID): set(2), countup(2), countdown(2){

    this->ID =ID;
	ftnn_step.resize(2);
	ftnn_stepold.resize(2);

	setold.resize(2);
	countupold.resize(2);
	countdownold.resize(2);
	diffset.resize(2);
	// zhenliang 
	deltaxdown.resize(2);
	deltaxup.resize(2);

	xup.resize(2);
	xdown.resize(2);

	input_max.resize(2);
	input_min.resize(2);
	ftnn_output.resize(2);
    phase.resize(2);

	setNeuronNumber(2);

    phase.at(0) = "swing";
    phase.at(1) = "swing";
    Px0 = 0.01;
    Py0 = -0.12;
    Px_max = 0.08;
    Px_min =-0.08;
    Py_max =-0.06;
    Py_min =-0.15;
    t=0;
}


void FTNN::step(){
//1) get parameters
    updateActivities();
	for (unsigned int i=0; i < ftnn_output.size(); i++) {

		setold.at(i) = set.at(i);
		countupold.at(i) = countup.at(i);
		countdownold.at(i) = countdown.at(i);

        ftnn_stepold.at(i) =ftnn_step.at(i);
		ftnn_step.at(i) = getActivity(i);

		//1.1) Linear threshold transfer function neuron 1 , or called step function neuron//
		/********************************************************/

		if (ftnn_step.at(i) >= ftnn_stepold.at(i)) ////////////////////Intuitively select
			set.at(i) = 1;//1.0 
		else
			set.at(i) = -1;//-1.0

		diffset.at(i) = set.at(i) - setold.at(i); // 

		//1.2) Count how many step of Swing and Stance
		/********************************************************/

		if (set.at(i) == 1) {
			countup.at(i) = countup.at(i) + 1; //Delta x0 up
			countdown.at(i) = 0;
            phase.at(i) = "swing";
		}
		// Count how many step of Stance
		else if (set.at(i) == -1) {
			countdown.at(i) = countdown.at(i) + 1; //Delta x0 down
			countup.at(i) = 0;
            phase.at(i) = "stance";
		}

		//1.3) Memorized the total steps of swing and stance
		/********************************************************/
		if (countup.at(i) == 0 && diffset.at(i) == -2 && set.at(i) == -1){//end up ,start to down
			deltaxup.at(i) = countupold.at(i);
            input_max.at(i) = ftnn_stepold.at(i);
        }

		if (countdown.at(i) == 0 && diffset.at(i) == 2 && set.at(i) == 1){//end down ,start to up
			deltaxdown.at(i) = countdownold.at(i);
            input_min.at(i) = ftnn_stepold.at(i);
        }

		//1.4) Comput y up and down !!!!
		/********************************************************/
		xup.at(i) = countup.at(i);
		xdown.at(i) = countdown.at(i);
	}
//2) set parameters
    
    Ty= deltaxup.at(0);
    T = deltaxup.at(0) + deltaxdown.at(0);
    
      
    if(phase.at(0)=="swing")
        t = xup.at(0);//0 hip, 1 knee
    if(phase.at(0)=="stance")
        t = deltaxup.at(0)+xdown.at(0);

    H = (input_max.at(1) -input_min.at(1))/15.0;//1.0 is a scale factor
    S = (input_max.at(0) -input_min.at(0))/-10.0;//5.0 is a scale factor
/*
    if(t<100)
        t++;
    else
        t=0;
    H=0.02;
    S= - 0.04;
    T=100;
    Ty=T/2;
*/
//3) calculate foot trajecory
    //3.1) cycloid foot trajecoty
    if(t<Ty){
           Px=S*(t/Ty-1.0/2.0/M_PI*sin(2.0*M_PI*t/Ty))+Px0-S/2.0;
        
        if(t<Ty/2.0)
            Py=2.0*H*(t/Ty-1.0/4.0/M_PI*sin(4.0*M_PI*t/Ty)) +Py0;
        else
            Py=2.0*H*(1.0-t/Ty+1.0/4.0/M_PI*sin(4.0*M_PI*t/Ty)) +Py0;
    }
    else{
        Px= Px0+S/2.0 -S*((t-Ty)/(T-Ty)-1.0/2.0/M_PI*sin((2.0*M_PI*(t-Ty))/(T-Ty)));
        Py= Py0;
    }
/*  //3.2) polymoid foot trajectory
    if(t<Ty){
        Px=S*(-16.0*t*t*t/(T*T*T)+12.0*t*t/(T*T) -t/T-1.0/4.0)+Px0 +S/2.0;
        Py=H*(-128.0*(t*t*t)/(T*T*T) + 48.0*t*t/(T*T))+Py0;
    }else{
        Px=S*(-t/T-1.0/4.0)+Px0+S/2.0;
        Py=Py0;
    }
*/
//4) set output and limit
    ftnn_output.at(0) = Px > Px_max ? Px_max :( Px < Px_min ? Px_min : Px);
    ftnn_output.at(1) = Py > Py_max ? Py_max :( Py < Py_min ? Py_min : Py);

	setOutput(0, ftnn_output.at(0));
	setOutput(1, ftnn_output.at(1));
}


#include "ikinenn.h"

IkineNN::IkineNN(){
input.resize(2);
theta.resize(2);
theta_init.resize(2);
link.resize(2);

link.at(0) = 0.07;//L1
link.at(1) = 0.085;//L2
theta_init.at(0) = 1.055;//43.0/180.0*M_PI;//1.055;//0.0;//-0.845;//-0.763;
theta_init.at(1) = -92.0/180.0*M_PI;//-1.61;//0.0;//-1.71;//-1.7;

setNeuronNumber(2);
}

void IkineNN::step(){
    updateActivities();
    for(unsigned int i=0;i<2;i++)
        input.at(i) = getActivity(i);
  
    float square1 = input.at(0)*input.at(0)+ input.at(1)*input.at(1);//
    float square2 = link.at(0)*link.at(0)+ link.at(1)*link.at(1);
    float divide = (square1-square2)/(2.0*link.at(0)*link.at(1));
    theta.at(1) = acos(divide);

    float temp1 = -link.at(0)-link.at(1)*cos(theta.at(1));
    float temp2 = link.at(1)*sin(theta.at(1));
    float temp3 = input.at(0)/sqrt(temp1*temp1+temp2*temp2);
    theta.at(0) = asin(temp3)-atan(temp2/temp1);


    theta.at(0) = -theta.at(0) + theta_init.at(0);
    theta.at(1) = theta.at(1) + theta_init.at(1);

//    theta.at(0) = theta.at(0) > M_PI ? theta.at(0)-M_PI : (theta.at(0) < -M_PI ? theta.at(0)+M_PI : theta.at(0));
//    theta.at(1) = theta.at(1) > M_PI ? theta.at(1)-M_PI : (theta.at(1) < -M_PI ? theta.at(1)+M_PI : theta.at(1));

    setOutput(0,theta.at(0));
    setOutput(1,theta.at(1));
}

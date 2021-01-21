#include "movingAverage.h"

MovingAverage::MovingAverage(int level,int length){
    this->level=level;
    assert(level<3);
    switch(level){
        case 0:
            buf1= new CircleQueue<float>(0.0,length);
            break;
        case 1:

            buf1 = new CircleQueue<float>(0.0,length);
            buf2 = new CircleQueue<float>(0.0,length/2);
            break;
        case 2:
            buf1 = new CircleQueue<float>(0.0,length);
            buf2 = new CircleQueue<float>(0.0,length/2);
            buf3 = new CircleQueue<float>(0.0,length/4);
            break;
        default:
            perror("moving average level is wrong");
            exit(-1);
    }

    output1 = 0.0;
    output2 = 0.0;
    output3 = 0.0;
    output = 0.0;


}

MovingAverage::~MovingAverage(){
    switch(level){
        case 0:
            delete buf1;
            break;
        case 1:

            delete buf1;
            delete buf2;
            break;
        case 2:
            delete buf1;
            delete buf2;
            delete buf3;
            break;
        default:
            perror("moving average level is wrong");
            exit(-1);
    }

}

void MovingAverage::setInput(float input){
    buf1->write(input);
}

void MovingAverage::step(){

    switch(level){
        case 0:
            output1=buf1->getAverage();
            buf1->step();
            break;
        case 1:
            output1=buf1->getAverage();
            buf1->step();

            buf2->write(output1);
            output2=buf2->getAverage();
            buf2->step();
            break;
        case 2:
            output1=buf1->getAverage();
            buf1->step();

            buf2->write(output1);
            output2=buf2->getAverage();
            buf2->step();

            buf3->write(output2);
            output3=buf3->getAverage();
            buf3->step();
            break;
        default:
            perror("moving average level is wrong");
            exit(-1);
    }


}

float MovingAverage::getOutput(){
    switch(level){
        case 0:
            output = output1;
            break;
        case 1:
            output = output2;
            break;
        case 2:
            output = output3;
            break;
        default:
            perror("moving average level is wrong");
            exit(-1);
    }
    return output;

}

float MovingAverage::update(float input){
    setInput(input);
    step();
    getOutput();
    return output;

}

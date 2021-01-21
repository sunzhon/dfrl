#ifndef MOVINGAVERAGE_H_
#define MOVINGAVERAGE_H_
#include "motionPhaseStability.h"

class MovingAverage{


    public:
        MovingAverage(int level=0,int length=50);
        ~MovingAverage();
        void setInput(float input);
        void step();
        float getOutput();
        float update(float input);
    private:
        CircleQueue<float> *buf1;
        CircleQueue<float> *buf2;
        CircleQueue<float> *buf3;
        float output1,output2,output3,output;

        int level;


};

#endif

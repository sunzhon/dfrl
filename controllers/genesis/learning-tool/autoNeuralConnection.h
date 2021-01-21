#ifndef MOTIONESTIMATE_H_
#define MOTIONESTIMATE_H_
#include "motionPhaseDiff.h"
#include "motionPhaseStability.h"

class AutoNeuralConnection{
public:
    AutoNeuralConnection(int leg_num);
    ~AutoNeuralConnection();
    void setInput(unsigned int index, pcpgsig sig);
    Matrix getPhaseDiff();
    float getPhaseStability();
    bool getANCtrigger();
    void step();
private:
    MotionPhaseDiffV2* mpd;
    MotionPhaseStability* mps;
    ANCtrigger* anctrigger;
};

#endif

#include "autoNeuralConnection.h"

AutoNeuralConnection::AutoNeuralConnection(int leg_num){
    mpd = new MotionPhaseDiffV2(leg_num);
    mps = new MotionPhaseStability(mpd->getOutput());
    anctrigger = new ANCtrigger();
}
AutoNeuralConnection::~AutoNeuralConnection(){
    delete mpd;
    delete mps;
    delete anctrigger;
}
void AutoNeuralConnection::step(){
    mpd->step();
    mps->setInput(mpd->getOutput());
    mps->step();
    anctrigger->setInput(mps->getOutput());
    anctrigger->step();
}
void AutoNeuralConnection::setInput(unsigned int index, pcpgsig sig){
    mpd->setInput(index,sig);
}
Matrix AutoNeuralConnection::getPhaseDiff(){
    return mpd->getOutput();
}
float AutoNeuralConnection::getPhaseStability(){
    return mps->getOutput();
}
bool AutoNeuralConnection::getANCtrigger(){
    return anctrigger->getOutput();
}

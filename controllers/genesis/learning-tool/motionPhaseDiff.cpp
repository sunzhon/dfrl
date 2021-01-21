#include "motionPhaseDiff.h"

//-------------QueueVessel------------------//
// This is a circle queue vessel, and capture the extrem point in this vessel for slope signals or steps signals
template<typename T>
QueueVessel<T>::QueueVessel(T x,int size){
    assert(size>5);// the capability of the Queuevessel cannot be too small
    buffer.resize(size);
    derivation.resize(size);
    steps =0;
    extreme = x;
    capturestate =false;
}
template<typename T>
void QueueVessel<T>::Write(T x){
    buffer.at(steps)= x;
}
template<typename T>
void QueueVessel<T>::DeWrite(float x){
    derivation.at(steps)= x;
}
template<typename T>
T QueueVessel<T>::Read(int delay){
    T y;
    y = buffer.at(mod(steps-delay,buffer.size()));
    return y;
}
template<typename T>
float QueueVessel<T>::DeRead(int delay){
    float y;
    y = derivation.at(mod(steps-delay,derivation.size()));
    return y;
}
template<typename T>
int QueueVessel<T>::mod(int x,int m){
    int r = x%m;
    return r<0 ? r+m:r;
}
template<typename T>
void QueueVessel<T>::setInput(T x){
    Write(x);
}
template<typename T>
void QueueVessel<T>::step(){
    updateExtreme();
    steps++;
    if(steps% buffer.size()==0){
        steps=0;
    }
}
template<typename T>
void QueueVessel<T>::updateExtreme(){
    DeWrite(Read(0).second-Read(1).second);//calculate the derivation
    //--------for slope signals
    if((DeRead(4)>0)&&(DeRead(3)>0)&&(DeRead(2)>0)
            &&(DeRead(1)<=0)&&(DeRead(0)<=0)){//calculate the peak point
        extreme = Read(2);
        capturestate =true;
    }else{
        capturestate =false;
    }
    /*
    //--------for step signals
    if((DeRead(4)==0)&&(DeRead(3)==0)&&(DeRead(2)>0)
    &&(DeRead(1)==0)&&(DeRead(0)==0)){//calculate the peak point
    extreme = Read(2);
    capturestate =true;
    }else{
    capturestate =false;
    }
    */
}
template<typename T>
T QueueVessel<T>::getOutput(){
    return extreme;
}
template<typename T>
bool QueueVessel<T>::getState(){
    return capturestate;
}

//-------------Capture the time of peak points of pcpg signals------------------//
CapturePeak::CapturePeak(){
    pcpgsig signal(0,0.0);
    peakcount =0;
    peak.insert({0,0});
    qv = new QueueVessel<pcpgsig>(pcpgsig(0,0.0),10);
}
CapturePeak::~CapturePeak(){
    delete qv;
}
void CapturePeak::setInput(pcpgsig sig){
    signal =sig;
}
void CapturePeak::step(){
    pcpgsig temp(0,0.0);
    //1) capture the peak point of signal
    qv->setInput(signal);
    qv->step();
    if(qv->getState()){
        temp = qv->getOutput();
        peakcount=peakcount+1;
        peak.insert(peakpoint(peakcount,temp.first));
    }
    //2) keep the peak map less than 5
    if(peak.size()>5){
        peakmap::iterator iter =peak.begin();
        peak.erase(iter);
    }
}
peakmap CapturePeak::getOutput(){
    return peak;
}
//------------CapturePeriod---------------//
//---- not use till to now in this issue
CapturePeriod::CapturePeriod(){
    sig_capturepeak= new CapturePeak();
}
CapturePeriod::~CapturePeriod(){
    delete sig_capturepeak;
}
void CapturePeriod::setInput(pcpgsig sig){
    sig_capturepeak->setInput(sig);
}
void CapturePeriod::step(){
    sig_capturepeak->step();
    sigpeak = sig_capturepeak->getOutput();
    peakmap::reverse_iterator iter =sigpeak.rbegin();
    period = iter->second - sigpeak[iter->first - 1];
}
float CapturePeriod::getOutput(){
    return period;
}

//------------PhaseShift------------------//
//------ calculate the phase sift bwteeen two pcpg siglas
PhaseShift::PhaseShift(){
    sig1cap= new CapturePeak();
    sig2cap= new CapturePeak();
    phaseshift =0.0;//-2*PI~2*PI
    filter = new lowPass_filter(0.15);
}
PhaseShift::~PhaseShift(){
    delete sig1cap;
    delete sig2cap;
    delete filter;
}
void PhaseShift::setInput(pcpgsig sig1,pcpgsig sig2){
    sig1cap->setInput(sig1);
    sig2cap->setInput(sig2);
}
void PhaseShift::step(){
    //0) get peaks
    sig1cap->step();
    sig2cap->step();
    sig1peak = sig1cap->getOutput();
    sig2peak = sig2cap->getOutput();
    peakmap::reverse_iterator iter1 =sig1peak.rbegin();
    peakmap::reverse_iterator iter2 =sig2peak.rbegin();
    //1) calcualte the period of now 
    int period;
    period=iter1->second - (++iter1)->second;
    //2) calculate the old peak point number of the newest peak point of two peaksmap
    unsigned int temp;
    iter2 =sig2peak.rbegin();
    if(iter1->first <= iter2->first){
        temp = iter1->first;
    }else{
        temp = iter2->first;
    }
    //3) calculate the phaseshift
    if((sig1peak.count(temp) !=1)||(sig2peak.count(temp)!=1)){// count() return whether it has temp
        std::cout<<" no element in peakmap for calculating the phase shift:"<<temp<<std::endl;
        phaseshift = 0.0;
    }else{
        if(period==0){
            phaseshift =0.0;
        }else{
            phaseshift = (sig1peak[temp]-sig2peak[temp])*2.0*M_PI/(float)period;//11.624;
        }
    }

    //4) calibrate the range,let it into [-pi,pi]
    if(phaseshift<0.0){
        phaseshift = -1.0*acos(cos(phaseshift));
    }else{
        phaseshift = acos(cos(phaseshift));
    }
    //5) filter
    phaseshift = filter->update(phaseshift);	
}

float PhaseShift::getOutput(){
    return phaseshift;
}

//----------MotionPhaseDiff-------------//
//----------V1 use a cascade with hierarchy process
MotionPhaseDiff::MotionPhaseDiff(unsigned int nCPGs){//the size of legs
    //1) initialize the connection matrix
    cmatrix.set(nCPGs,nCPGs);
    //2) initialize the signals form pcpg
    signals.resize(nCPGs);
    for(unsigned int i=0;i<nCPGs;i++)
        signals.at(i)= pcpgsig(0,0.0);
    //3) initialize the phaseshift
    phase_shift.resize(nCPGs);
    for(unsigned int i=0;i<nCPGs;i++)
        phase_shift.at(i).resize(nCPGs);
    for(unsigned int i=0;i<nCPGs;i++)
        for(unsigned int j=0;j<nCPGs;j++)
            phase_shift.at(i).at(j) = new PhaseShift();
}

MotionPhaseDiff::~MotionPhaseDiff(){
    for(std::vector<std::vector<PhaseShift *>>::iterator it=phase_shift.begin();it!=phase_shift.end();it++)
        for(std::vector<PhaseShift *>::iterator itj=it->begin();itj!= it->end();itj++)
            delete *itj;
}

void MotionPhaseDiff::setInput(unsigned int index, pcpgsig sig){
    assert(index <signals.size());
    signals.at(index)=sig;
}

void MotionPhaseDiff::step(){
    for(unsigned int i=0;i<phase_shift.size();i++)
        for(unsigned int j=0;j<phase_shift.at(i).size();j++){
            phase_shift.at(i).at(j)->setInput(signals.at(i),signals.at(j));
            phase_shift.at(i).at(j)->step();
            cmatrix.val(i,j)= phase_shift.at(i).at(j)->getOutput();
        }
    //set the matrix ,let the front right leg is a standard
    if(cmatrix.val(0,1)>0.0)
        cmatrix = cmatrix*-1.0;
}

Matrix MotionPhaseDiff::getOutput(){
    return cmatrix;
}
//---------------------------------------------------------------//



//-------------------------------------------------------------//
//------------PhaseShiftV2------------------//
//------ calculate the phase sift bwteeen two peakmaps, compared to V1, this 
// has less computation consume
PhaseShiftV2::PhaseShiftV2(){
    phaseshift =0.0;//-*PI~*PI
    filter = new lowPass_filter(0.2);
}
PhaseShiftV2::~PhaseShiftV2(){
    delete filter;
}
void PhaseShiftV2::setInput(peakmap peak1,peakmap peak2){
    sig1peak =peak1;
    sig2peak =peak2;
}
void PhaseShiftV2::step(){
    //1) calcualte the period of now 
    int period1, period2, period;
    peakmap::reverse_iterator iter1 =sig1peak.rbegin();
    peakmap::reverse_iterator iter2 =sig2peak.rbegin();
    period1=iter1->second - (++iter1)->second;
    period2=iter2->second - (++iter2)->second;
    period= (period1+period2)/2.0;
    //2) calculate the old peak point of the newest peak point of two peaksmap
    iter1 =sig1peak.rbegin();
    iter2 =sig2peak.rbegin();


    //3) calculate the phaseshift
    /* version 1
     *not stable
       if(iter2->second<iter1->second){
       if(period==0){
       phaseshift = 0.0;
       cout<<"phase shift is zero:"<<period<<endl;
       } 
       phaseshift = (iter1->second-iter2->second)*2.0*M_PI/(float)period;//11.624;
       }else{
       if((++iter2)->second<iter1->second){
       if(period==0){
       phaseshift = 0.0;
       cout<<"phase shift is zero:"<<period<<endl;
       } 
       phaseshift = (iter1->second-iter2->second)*2.0*M_PI/(float)period;//11.624;
       }else{
       if((++iter2)->second<iter1->second){
       if(period==0){
       phaseshift = 0.0;
       cout<<"phase shift is zero:"<<period<<endl;
       } 
       phaseshift = (iter1->second-iter2->second)*2.0*M_PI/(float)period;//11.624;
       }
       }
       }
       */
    /*
     * 
     // version 2
     // use the peakcount to decied the matched peaks., but if the long time cannot get 
     // peakpoint, then this method is not work
     unsigned int temp;
     if(iter1->first <= iter2->first){
     temp = iter1->first;
     }else{
     temp = iter2->first;
     }
     if((sig1peak.count(temp) !=1)||(sig2peak.count(temp)!=1)){// tongshi you di jihao
       std::cout<<" no element in peakmap for calculating the phase shift:"<<temp<<std::endl;
       phaseshift =0.0;
       }else{
       if(period==0){
       phaseshift =0.0;
       cout<<"1phase shift is zero:"<<period<<endl;

       }else{
       phaseshift = (sig1peak[temp]-sig2peak[temp])*2.0*M_PI/(float)period;//11.624;
       cout<<"2phase shift is zero:"<<period<<" temp:"<<temp<<endl;
       }
       }
       */
    // version 3 
    // the iter1 is frame, and compare to the one of iter2, that is the bigest one but smaller than iter1
    for(unsigned int i=0;i<sig2peak.size();i++){
        if(iter2->second<iter1->second)
            break;
        else
            iter2++;
    }
    if(period==0)
        phaseshift=0.0;
    else
        phaseshift = (iter1->second-iter2->second)*2.0*M_PI/(float)period;//11.624;


    //4) calibrate the range,let it into [-pi,pi]
    if(phaseshift<0.0){
        phaseshift = -1.0*acos(cos(phaseshift));
    }else{
        phaseshift = acos(cos(phaseshift));
    }
    //5) filter
    phaseshift = filter->update(phaseshift);	
}

float PhaseShiftV2::getOutput(){
    return phaseshift;
}
//----------MotionPhaseDiffV2-------------//
//----------V2 use a serial without hierarchy process--------//
MotionPhaseDiffV2::MotionPhaseDiffV2(unsigned int nCPGs){//the size of legs
    //1) initialize the connection matrix
    cmatrix.set(nCPGs,nCPGs);
    //2) initialize the signals form pcpg
    signals.resize(nCPGs);
    for(unsigned int i=0;i<nCPGs;i++)
        signals.at(i)= pcpgsig(0,0.0);
    //3) initialize the capturePeak
    peak_capture.resize(nCPGs);
    for(uint16_t i=0;i<nCPGs;i++)
        peak_capture.at(i)= new CapturePeak();

    //3) initialize the phaseshift
    phase_shift.resize(nCPGs);
    for(unsigned int i=0;i<nCPGs;i++)
        phase_shift.at(i).resize(nCPGs);
    for(unsigned int i=0;i<nCPGs;i++)
        for(unsigned int j=0;j<nCPGs;j++)
            phase_shift.at(i).at(j) = new PhaseShiftV2();
}

MotionPhaseDiffV2::~MotionPhaseDiffV2(){
    for(std::vector<std::vector<PhaseShiftV2 *>>::iterator it=phase_shift.begin();it!=phase_shift.end();it++)
        for(std::vector<PhaseShiftV2 *>::iterator itj=it->begin();itj!= it->end();itj++)
            delete *itj;
    for(std::vector<CapturePeak *>::iterator it=peak_capture.begin();it!= peak_capture.end();it++)
        delete *it;
}

void MotionPhaseDiffV2::setInput(unsigned int index, pcpgsig sig){
    assert(index <signals.size());
    signals.at(index)=sig;
}

void MotionPhaseDiffV2::step(){
    //1) calculate the peak point map of all signals
    for(uint8_t i=0;i < peak_capture.size();i++){
        peak_capture.at(i)->setInput(signals.at(i));
        peak_capture.at(i)->step();
    }
    for(unsigned int i=0;i<phase_shift.size();i++)
        for(unsigned int j=0;j<phase_shift.at(i).size();j++){
            phase_shift.at(i).at(j)->setInput(peak_capture.at(i)->getOutput(),peak_capture.at(j)->getOutput());
            phase_shift.at(i).at(j)->step();
            cmatrix.val(i,j)= phase_shift.at(i).at(j)->getOutput();
        }
    //set the matrix ,let the front right leg is a standard
    if(cmatrix.val(0,1)>0.0)
        cmatrix = cmatrix*-1.0;
}

Matrix MotionPhaseDiffV2::getOutput(){
    return cmatrix;
}

/*  Author :sun tao
**  Data: 9-10-2018
**  email: suntao.hn@qq.com**/
#ifndef PHASESTABILITY_H_
#define PHASESTABILITY_H_
#include <vector>
#include "matrix.h"
#include <math.h>
using namespace matrix;
using namespace std;

/***** Circle Queue class************/

template<typename T>
class CircleQueue{
public:
     CircleQueue(T x, int size);
     void step();
     void write(T x);
     T read(int delay);
     int mod(int x,int m);
     T getSum();
     T getAverage();
     T getMedian();
     std::pair<int,T> getMax();
     std::pair<int,T> getMin();
private:
     std::vector<T> buffer;
     int steps;
};

template<typename T>
CircleQueue<T>::CircleQueue(T x,int size){
	buffer.resize(size);
	steps =0;
}
template<typename T>
void CircleQueue<T>::write(T x){
	buffer.at(steps)= x;
}
template<typename T>
T CircleQueue<T>::read(int delay){
	T y;
	y = buffer.at(mod(steps-delay,buffer.size()));
	return y;
}
template<typename T>
int CircleQueue<T>::mod(int x,int m){
	int r = x%m;
	return r<0 ? r+m:r;
}
template<typename T>
void CircleQueue<T>::step(){
	steps++;
	if(steps% buffer.size()==0){
		steps=0;
	}
}
template<typename T>
T CircleQueue<T>::getSum(){
    T sum=buffer.at(0)*0.0;
    for(uint16_t i=0;i<buffer.size();i++)
         sum+=read(i);
    return sum;
}
template<typename T>
T CircleQueue<T>::getAverage(){
    T average= buffer.at(0)*0.0;
    average=getSum()*(1.0/buffer.size());
    return average;

}

template<typename T>
T CircleQueue<T>::getMedian(){
    T median= buffer.at(0)*0.0;
    median=(getMax().second+getMin().second)*0.5;
    return median;

}
template<typename T>
std::pair<int,T> CircleQueue<T>::getMax(){
    T max= read(0);
    int maxID;
    for(unsigned int index=1;index<buffer.size();index++)
        if(read(index)>=max){
            max=read(index);
            maxID=index;
        }
    return std::pair<int,T>(maxID,max);
}

template<typename T>
std::pair<int,T> CircleQueue<T>::getMin(){
    T min= read(0);
    int minID;
    for(unsigned int index=1;index<buffer.size();index++)
        if(read(index)<=min){
            min=read(index);
            minID=index;
        }
    return std::pair<int,T>(minID,min);
}
/*****MotionPhaseStability class***********/
class MotionPhaseStability{
public:
    MotionPhaseStability(Matrix matrix);
    ~MotionPhaseStability();
    void setInput(Matrix matrix);
    void step();
    float getOutput();
    void init();
private:
    CircleQueue<Matrix> *sampleQueue;
    CircleQueue<float> *varianceQueue;
    float stability;
    unsigned int size;
    unsigned int dim;
    Matrix m_trot;
};



/***** ANCtrigger class***********/
// This class is used to determine when to open ANC depending on the Motionphase Stability
// The output of the class is the kappa in P2 paper
class ANCtrigger{
public:
    ANCtrigger();
    ~ANCtrigger();
    void setInput(float stability);
    void step();
    bool getOutput();
    void init();
private:
    CircleQueue<float> *stabilityQueue;
    std::pair<int, float> peakStability;
    float stabilityUpThreshold, stabilityLowThreshold;
    bool kappa= false;
    int size;

};


#endif

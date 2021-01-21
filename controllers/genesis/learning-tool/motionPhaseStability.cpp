#include "motionPhaseStability.h"
using namespace matrix;
using namespace std;

/***** phase stability class ******/
MotionPhaseStability::MotionPhaseStability(Matrix matrix){
    size =60;// for distance set size=20
    sampleQueue = new CircleQueue<Matrix>(matrix,size);
    dim = matrix.getM();
    double temp[] = {0.0,-M_PI,-M_PI,0.0, M_PI,0.0,0.0,M_PI, M_PI,0.0,0.0,M_PI, 0.0,-M_PI,-M_PI,0.0};
    m_trot.set(dim,dim,temp);
    init();

}
MotionPhaseStability::~MotionPhaseStability(){
    delete sampleQueue;
}
void MotionPhaseStability::setInput(Matrix matrix){
    sampleQueue->write(matrix);
}

void MotionPhaseStability::step(){
    sampleQueue->step();

    /*
    //---------** virance method--- diatance between previous average matrix **---------//
    Matrix m_sum(dim,dim),m_average(dim,dim),m_diff(dim,dim),m_squre(dim,dim);
    float distance_sum, distance_average;
    //1) calculate average
    for(uint16_t i=150;i<size;i++)  //before size/2 time
        m_sum+=sampleQueue->read(i);
    m_average=m_sum*(1.0/100);
    //2) calculate variance
    for(unsigned int i =0;i<50;i++){//size
        m_diff = sampleQueue->read(i)-m_average;
        for(uint8_t j=0;j<m_diff.getM();j++)
            for(uint8_t k=0;k<m_diff.getN();k++)
                m_squre.val(j,k) = m_diff.val(j,k)*m_diff.val(j,k);
        // m_squre_sum += m_squre;
        distance_sum += sqrt(m_squre.elementSum());
    }
    //3) calculate the sum of variances
    distance_average=distance_sum*(1.0/50);
    //4) calcualte the stability from sum of variances
    stability = distance_average;
    */
    
    //---------** virance method--- diatance between current average matrix **---------//
    Matrix m_sum(dim,dim),m_average(dim,dim),m_diff(dim,dim),m_squre(dim,dim);
    float squre_sum, distance_average;
    //1) calculate average
    for(uint16_t i=0;i<size;i++)  //before size/2 time
        m_sum+=sampleQueue->read(i);
    m_average=m_sum*(1.0/size);
    //2) calculate variance
    for(unsigned int i =0;i<size;i++){//size
        m_diff = sampleQueue->read(i)-m_average;
        squre_sum+=m_diff.norm_sqr();
    }
    //3) calculate the sum of variances
    distance_average=squre_sum*(1.0/size);
    //4) calcualte the stability from sum of variances
    stability = distance_average;

    /*
    //---------** Distance method--- distance between trot gait **---------//
    Matrix m_diff(dim,dim),m_squre(dim,dim);
    float distance_sum,distance_average;
    //1) calculate distance
    for(uint8_t i=0;i<size;i++){
        m_diff=sampleQueue->read(i)-m_trot;
        for(uint8_t j=0;j<m_diff.getM();j++)
            for(uint8_t k=0;k<m_diff.getN();k++)
                m_squre.val(j,k) = m_diff.val(j,k)*m_diff.val(j,k);
        distance_sum += sqrt(m_squre.elementSum());
    }
    distance_average=1.0/size*distance_sum;
    //2) calcualte the stability from sum of variances
    stability = distance_average;
    */

}

float MotionPhaseStability::getOutput(){
    return stability;
}

void MotionPhaseStability::init(){
    Matrix matrix(dim,dim);
    for(uint16_t i=0;i<size;i++){
        sampleQueue->write(matrix);
        sampleQueue->step();
    }
}

/*********************ANC trigger class**********************/
// This is for activating ANC to function depending on the stability profile when 
// a stable interlimb corrdination emerges
//
ANCtrigger::ANCtrigger(){
    size=400;// control ros rate  = 50
    stabilityQueue = new CircleQueue<float>(0.0,size);
    peakStability = std::pair<int,float> (0,0.0);
    stabilityUpThreshold = 0.8;
    stabilityLowThreshold = 0.4;
}

ANCtrigger::~ANCtrigger(){
    delete stabilityQueue;
}
void ANCtrigger::step(){
    if(!kappa){// if no stable
        // acquire the trend of the stability increase or decreas
        if(stabilityQueue->read(0)<stabilityLowThreshold){// 1) should be small
            peakStability=stabilityQueue->getMax();
            if(peakStability.second>=stabilityUpThreshold){
                //2) peak point should be at the latter part of the queue
                if((peakStability.first<(int)(size*5/6))&&(peakStability.first>(int)(size/2))){
                    kappa=true;
                    cout<<"sss "<<stabilityQueue->read(0)<<endl;
                }
            }
        }
    }else{// if already stable
        if(stabilityQueue->read(0)>stabilityLowThreshold){
            kappa=false;
            cout<<stabilityQueue->read(0)<<" kappa is false"<<endl;
            
        }
    }
}

void ANCtrigger::setInput(float stability){
    stabilityQueue->step();
    stabilityQueue->write(stability);
}

bool ANCtrigger::getOutput(){
    return kappa;
}

//
// Created by mat on 3/4/18.
//

#ifndef MAIN_CONTROLLER_POSTPROCESSING_H
#define MAIN_CONTROLLER_POSTPROCESSING_H

#include <vector>
#include <cmath>
#include <iostream>

class postProcessing {
public:
    postProcessing();
    double getLPFSignal();
    double getAmplitudeSignal();
    double calculateAmplitudeSignal(double);
    double calculateLPFSignal(double);
    void setBeta(double);
    double calculateLPFAmplitude(double);
    double getTimeBetweenZeroDerivative();

private:
    // Calculate amplitude parameters
    void calculateAmplitude(double);

    float sigDot = 0, sigPrimeOld = 0, signalOld = 0;
    int timeSinceZeroDerivative = 0;
    int timeBetweenZeroDerivative = 0;
    bool lastZeroDerivative = false;
    std::vector<float> zeroDerivative = {0,0};
    float amplitude = 0;

    // Low Pass Filter Parameters
    void lowPassFiltering(double);

    float LPFSignal = 0, LPFbeta = 0.8;
};


#endif //MAIN_CONTROLLER_POSTPROCESSING_H

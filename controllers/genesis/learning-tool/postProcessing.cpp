//
// Created by mat on 3/4/18.
//

#include "postProcessing.h"

postProcessing::postProcessing() = default;

void postProcessing::lowPassFiltering(double signal) {
    // Calculate the amplitudes
    LPFSignal = LPFSignal - (LPFbeta * (LPFSignal - signal));
}

void postProcessing::calculateAmplitude(double signal) {
    // Calculate the derivative
    sigDot = (signal - signalOld)/0.0125; // 80Hz = 0.0125s period

    // See if the derivative changes sign
    if ((((sigDot >= 0) ^ (sigPrimeOld < 0)) == 0) && timeSinceZeroDerivative  >1) {
        // If it changes sign then it is a local max- or minimum
        if (sigDot > 0 && !lastZeroDerivative) {//mini
            zeroDerivative[0] = signalOld;
            lastZeroDerivative = true;
        } else if (sigDot < 0 && lastZeroDerivative){//max
            zeroDerivative[1] = signalOld;
            lastZeroDerivative = false;
        }

        // Subtract local minimum from maximum to get amplitude
        amplitude = std::fabs(zeroDerivative[0]-zeroDerivative[1])/2;

        if(timeSinceZeroDerivative != timeBetweenZeroDerivative)
            timeBetweenZeroDerivative = (timeSinceZeroDerivative+timeBetweenZeroDerivative)/2;

        timeSinceZeroDerivative = 0;
    }
    else {
        timeSinceZeroDerivative++;
    }

    // Advance
    sigPrimeOld = sigDot;
    signalOld = signal;
}

double postProcessing::calculateLPFAmplitude(double signal){
    lowPassFiltering(signal);
    calculateAmplitude(LPFSignal);
    return amplitude;
}

double postProcessing::calculateLPFSignal(double signal){
    lowPassFiltering(signal);
    return LPFSignal;
}
double postProcessing::calculateAmplitudeSignal(double signal){
    calculateAmplitude(LPFSignal);
    return amplitude;
}

void postProcessing::setBeta(double beta) {
    LPFbeta=beta;
}

double postProcessing::getLPFSignal() {
    return LPFSignal;
}

double postProcessing::getAmplitudeSignal() {
    return amplitude;
}

double postProcessing::getTimeBetweenZeroDerivative() {
    return timeBetweenZeroDerivative;
}

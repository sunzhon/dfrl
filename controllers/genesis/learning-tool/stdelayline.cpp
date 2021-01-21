/*
 * delayline.cpp
 *
 *  Created on: Apr 10, 2012
 *      Author: degoldschmidt
 */
using namespace std;
#include "stdelayline.h"

STDelayline::STDelayline(int size) {
  buffer.resize(size);
  step = 0;
}

void STDelayline::Write(double input) {
  // write delayed values
  buffer.at(step) = input; //write
}

double STDelayline::Read(int delay) {
  // read delayed values
  double y;
  y = buffer.at(mod(step-delay,buffer.size())); //read
  return y;
}

void STDelayline::Step() {
  // step
  step++;
  if (step % buffer.size() == 0) {
    step = 0;
  }
}

void STDelayline::Reset() {
  // reset buffer
  int newsize = buffer.size();
  buffer.clear();
  buffer.resize(newsize);
}

int STDelayline::mod(int x, int m) {
       int r = x%m;
       return r<0 ? r+m : r;
}



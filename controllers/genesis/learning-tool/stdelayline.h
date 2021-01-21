/*
 * delayline.h
 *  Created on: Apr 10, 2012
 *      Author: degoldschmidt
 *     NOTE: Sun Tao apply this into stbot project
 */

#ifndef STDELAYLINE_H_
#define STDELAYLINE_H_

#include <vector>
#include <iostream>
#include <cstdlib>
using namespace std;

class STDelayline {
public:
    STDelayline(int size);
    double Read(int delay);
    void Write(double input);
    void Step();
    void Reset();
    static int mod(int x, int m);
    vector<double> buffer;
    int step;

private:



};


#endif /* DELAYLINE_H_ */

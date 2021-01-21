/*
 * dlearn.h
 *
 *  Created on: Dec 8, 2017
 *      Author: suntao
 */

#ifndef DLEARN_H_
#define DLEARN_H_
#include <utils/ann-framework/ann.h>

#define TREND_UP true
#define TREND_DOWN false

class ForceForwardmodel: public ANN{
    private:
        float gama,alpha,G;
        float input,activity,activity_old;//motor input
        float output,output_old;//expected_force;
    public:
        ForceForwardmodel(float _alpha,float _gama = 0.99);

        virtual ~ForceForwardmodel();
        void setInput(float _input);
        void step();
        float getOutput()const;
        float getInput()const;
        void setAlpha(float _alpha);

};


class DLearn {
    public:
        DLearn();
        void step();
        void setInput(float expexted_force,float actual_force);
        float getOutput();
        void setParameters(float Af, float Bf, float As, float Bs );
    private:
        //model
        //learn
        float Af;
        float As;
        float Bf;
        float Bs;

        float Ks;
        float Ks_old;
        float Kf;
        float Kf_old;
        float error;
        float gamma;

        float expected_value;
        float actual_value;

        float lowerThreshold,upperThreshold;
        float controlOutput;
};

#endif /* DLEARN_H_ */

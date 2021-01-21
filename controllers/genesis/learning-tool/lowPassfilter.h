/*
 * lowPassfilter.h
 *
 *  Created on: Feb 11, 2015
 *      Author: giuliano
 */

#ifndef LOWPASSFILTER_H_
#define LOWPASSFILTER_H_

class lowPass_filter {
public:
	lowPass_filter();
	lowPass_filter(float weight);
	float update(float value);
	void set_current_value(float value);
	virtual ~lowPass_filter();

private:
	float w;
	float current_value;
};

#endif /* CONTROLLERS_RUNBOTII_DACBOT_LOWPASSFILTER_H_ */







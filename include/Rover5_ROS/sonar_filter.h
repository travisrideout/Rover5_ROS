/*
 * sonar_filter.h
 *
 *  Created on: Mar 6, 2015
 *      Author: travis
 */

#ifndef INCLUDE_ROVER5_ROS_SONAR_FILTER_H_
#define INCLUDE_ROVER5_ROS_SONAR_FILTER_H_

#include <iostream>

class Sonar_Filter
{
public:
	Sonar_Filter();
	int AveragingFilter(int);
	int IIR(int);
	~Sonar_Filter();

private:
	int in0, in1, in2, in3, in4, in5, in6;
	int out0, out1, out2, out3, out4, out5, out6;
	double a1, a2, b0, b1, b2;
	int iir_max;

};

#endif /* INCLUDE_ROVER5_ROS_SONAR_FILTER_H_ */

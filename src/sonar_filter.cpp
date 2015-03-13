/*
 * sonar_filter.cpp
 *
 *  Created on: Mar 6, 2015
 *      Author: travis
 */

#include <Rover5_ROS/sonar_filter.h>

Sonar_Filter::Sonar_Filter():
	in0(0),in1(0),in2(0),in3(0),in4(0),in5(0),in6(0),
	out0(0),out1(0),out2(0),out3(0),out4(0),out5(0),out6(0),
	a1(1.01),
	a2(-0.01),
	b0(-0.4),
	b1(0.05),
	b2(0.375),
	iir_max(1550)
	{
}

int Sonar_Filter::AveragingFilter(int range){
	if((range-out0)>50){
		in0=in1+1;
	}else if((range-out0)<-50&&out0>0){
		in0=in1-1;
	}else{
		in0=range;
	}
	out0=(in0+in1+in2+in3+in4+in5+in6+out1+out2+out3+out4+out5+out6)/13;
	in1 = in0;
	in2 = in1;
	in3 = in2;
	in4 = in3;
	in5 = in4;
	in6 = in5;
	out1 = out0;
	out2 = out1;
	out3 = out2;
	out4 = out3;
	out5 = out4;
	out6 = out5;
	return out0;
}

int Sonar_Filter::IIR(int range){
	in0 = range-out0;
	out0 = out1*a1+out2*a2+in0*b0+in1*b1*in2*b2;
	if (out0>iir_max){
		out0=iir_max;
	}
	out1 = out0;
	out2 = out1;
	in1 = in0;
	in2 = in1;
	return out0;
}

Sonar_Filter::~Sonar_Filter(){
	std::cout << "Sonar_Filter object destroyed" << std::endl;
}



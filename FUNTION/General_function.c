#include "math.h"




// 参数：x 是要限制的数，a 是下限，b 是上限
float constrain(float x, float a, float b) 
{
  if (x < a) {
    return a;
  } else if (x > b) {
    return b;
  } else {
    return x;
  }
}


float tamp_task(float input,float step,float ref_angle)
{
	if(fabs(ref_angle-input)<step) 
		return ref_angle;
	else 
	{
		if(input<ref_angle)
			input+=step;

		else if(input>ref_angle)
			input-=step;
	}
	
	return input;
}
































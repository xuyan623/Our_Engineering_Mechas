#include "filter.h"

float Fir_hanning_20_30Hz [21] = {0.00101817116370487,0.00462495540223689,0.0113796960211338,0.0213781754455494,0.0341842713704715,0.0488492120624196,0.0640193484263798,0.0781175705972818,0.0895699818913486,0.0970407392926174,0.0996357566537127,0.0970407392926174,0.0895699818913486,0.0781175705972818,0.0640193484263798,0.0488492120624196,0.0341842713704715,0.0213781754455494,0.0113796960211338,0.00462495540223689,0.00101817116370487};
float Fir_hanning_10_30Hz [11] = {0.00985251478137758,0.0388911109987124,0.0811783408490355,0.125485355638645,0.158920832476061,0.171343690512335,0.158920832476061,0.125485355638645,0.0811783408490355,0.0388911109987124,0.00985251478137758};
float Fir_hanning_8_30Hz [9] = {0.0176764665739234,0.0667464118866970,0.130306913693273,0.183326950056021,0.203886515580170,0.183326950056021,0.130306913693273,0.0667464118866970,0.0176764665739230};
float Fir_hanning_6_30Hz [7] = {0.0351213616040696,0.123573015205195,0.214756255944446,0.253098734492580,0.214756255944446,0.123573015205195,0.0351213616040696};
float Fir_hanning_4_30Hz [5] = {0.0819367487921516,0.250242733165992,0.335641036083713,0.250242733165992,0.0819367487921516};
float Fir_hanning_2_30Hz [3] = {0.249258903175929,0.501482193648142,0.249258903175929};
float Filter0(float data , float* windows)
{
	static float data_input[ORDER+1] = {0};
	static u8 i = 0;
	float output = 0;
	for(i=ORDER;i>0;i--)
	{
		data_input[i] = data_input[i-1];
	}
	data_input[0] = data;
	for(i=0;i<ORDER;i++)
	{
		output += data_input[i]*windows[i];
	}
	return output;
}

float Filter1(float data , float* windows)
{
	static float data_input[ORDER+1] = {0};
	static u8 i = 0;
	float output = 0;
	for(i=ORDER;i>0;i--)
	{
		data_input[i] = data_input[i-1];
	}
	data_input[0] = data;
	for(i=0;i<ORDER;i++)
	{
		output += data_input[i]*windows[i];
	}
	return output;
}

float Filter2(float data , float* windows)
{
	static float data_input[ORDER+1] = {0};
	static u8 i = 0;
	float output = 0;
	for(i=ORDER;i>0;i--)
	{
		data_input[i] = data_input[i-1];
	}
	data_input[0] = data;
	for(i=0;i<ORDER;i++)
	{
		output += data_input[i]*windows[i];
	}
	return output;
}

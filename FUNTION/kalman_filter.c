/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
/** @file kalman_filter.c
 *  @version 1.0
 *  @date Apr 2018
 *
 *  @brief kalman filter realization
 *
 *  @copyright 2018 DJI RoboMaster. All rights reserved.
 *
 */
 
#include "kalman_filter.h"

//float xhat_data[2], xhatminus_data[2], z_data[2],Pminus_data[4], K_data[4];
//float P_data[4] = {2, 0, 0, 2};
//float AT_data[4], HT_data[4];
//float A_data[4] = {1, 0.001, 0, 1};
//float H_data[4] = {1, 0, 0, 1};
//float Q_data[4] = {1, 0, 0, 1};
//float R_data[4] = {2000, 0, 0, 5000};


float matrix_value1;
float matrix_value2;

void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
{
  mat_init(&F->xhat,2,1,(float *)I->xhat_data);
  mat_init(&F->xhatminus,2,1,(float *)I->xhatminus_data);
  mat_init(&F->z,2,1,(float *)I->z_data);
  mat_init(&F->A,2,2,(float *)I->A_data);
  mat_init(&F->H,2,2,(float *)I->H_data);
  mat_init(&F->Q,2,2,(float *)I->Q_data);
  mat_init(&F->R,2,2,(float *)I->R_data);
  mat_init(&F->P,2,2,(float *)I->P_data);
  mat_init(&F->Pminus,2,2,(float *)I->Pminus_data);
  mat_init(&F->K,2,2,(float *)I->K_data);
  mat_init(&F->AT,2,2,(float *)I->AT_data);
  mat_trans(&F->A, &F->AT);
  mat_init(&F->HT,2,2,(float *)I->HT_data);
  mat_trans(&F->H, &F->HT);
//  matrix_value2 = F->A.pData[1];
}


/********************************************************
				 Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
         R:测量噪声，R增大，动态响应变慢，收敛稳定性变好
				 F->z.pData  输入预测矩阵
				 P(k) 最优值对应的协方差	
				 Q用于求先验估计方差xhatminus，后验估计方差xhat
				 R用于求卡尔曼增益K
*********************************************************/
float *kalman_filter_calc(kalman_filter_t *F, float angle, float speed)
{
  float TEMP_data[4] = {0, 0, 0, 0};
  float TEMP_data21[2] = {0, 0};
  mat TEMP,TEMP21;				//定义mat浮点矩阵结构类型TEMP,TEMP21

	
	//mat_init	将浮点矩阵的参数初始化，确定行列，并将参数（矩阵数组TEMP_data【4】）赋值给第一个参数TEMP
  mat_init(&TEMP,2,2,(float *)TEMP_data);	//指向TEMP矩阵，2行2列，矩阵数组为TEMP_data[4]
  mat_init(&TEMP21,2,1,(float *)TEMP_data21);

	/*F->z.pData 预测矩阵 */
  F->z.pData[0] = angle;		
  F->z.pData[1] = speed;

	/*求先验估计xhatminus*/
  //1. xhat'(k)= A xhat(k-1)
  mat_mult(&F->A, &F->xhat, &F->xhatminus);		//浮点矩阵乘法	先验估计xhatminus = A * xhat(k-1)

	/*求先验方差估计Pminus*/
  //2. P'(k) = A P(k-1) AT + Q
  mat_mult(&F->A, &F->P, &F->Pminus);					//							先验方差估计Pminus = A * P(k-1) 
  mat_mult(&F->Pminus, &F->AT, &TEMP);				//							TEMP2*2矩阵 = 先验方差估计Pminus * AT
  mat_add(&TEMP, &F->Q, &F->Pminus);					//浮点矩阵加法	先验方差估计Pminus = TEMP + 过程噪声Q

	/*求Kalman增益K*/
  //3. K(k) = P'(k) HT / (H P'(k) HT + R)
  mat_mult(&F->H, &F->Pminus, &F->K);					//							Kalman增益K = H * P'(k)
  mat_mult(&F->K, &F->HT, &TEMP);							//							TEMP = Kalman增益K * HT
  mat_add(&TEMP, &F->R, &F->K);								//							Kalman增益K = TEMP + 测量噪声R

  mat_inv(&F->K, &F->P);											//矩阵的逆			对矩阵K求逆得到P
  mat_mult(&F->Pminus, &F->HT, &TEMP);				//							TEMP = P'(k) * HT
  mat_mult(&TEMP, &F->P, &F->K);							//							Kalman增益K = TEMP * P

	/*求后验估计xhat*/
  //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
  mat_mult(&F->H, &F->xhatminus, &TEMP21);		//							TEMP21 = H * 先验估计xhat'(k)
  mat_sub(&F->z, &TEMP21, &F->xhat);					//							后验估计xhat = z(k) - TEMP21
  mat_mult(&F->K, &F->xhat, &TEMP21);					//							TEMP21 = K(k) * xhat
  mat_add(&F->xhatminus, &TEMP21, &F->xhat);	//							后验估计xhat = xhatminus + TEMP21

	/*求后验估计方差P*/
  //5. P(k) = (1-K(k)H)P'(k)
  mat_mult(&F->K, &F->H, &F->P);							//							K * H
  mat_sub(&F->Q, &F->P, &TEMP);								//							Q - K * H				(Q = 1)
  mat_mult(&TEMP, &F->Pminus, &F->P);					//							P = (Q - K * H)	* Pminus
	
  matrix_value1 = F->xhat.pData[0];
  matrix_value2 = F->xhat.pData[1];

  F->filtered_value[1] = F->xhat.pData[1];
  return F->filtered_value;
}


#ifdef IIR_FILTER
double NUM[7] = 
{
  0.000938328292536,
 -0.005375225952906,
  0.01306656496967,
 -0.01725922344534,
  0.01306656496967,
 -0.005375225952906,
  0.000938328292536
};
double DEN[7] = 
{
   1,
  -5.733703351811,
  13.70376013927,
 -17.47505866491,
  12.53981348666,
  -4.800968865471,
   0.7661573674342
};

typedef struct
{
  double raw_value;
  double xbuf[7];
  double ybuf[7];
  double filtered_value;
} iir_filter_t;

iir_filter_t yaw_msg_t, pit_msg_t;


double iir_filter(iir_filter_t *F)
{
  int i;
  for(i = 6; i > 0; i--)
  {
    F->xbuf[i] = F->xbuf[i-1];
    F->ybuf[i] = F->ybuf[i-1];
  }
  
  F->xbuf[0] = F->raw_value;
  F->ybuf[0] = NUM[0] * F->xbuf[0];
  
  for(i = 1; i < 7; i++)
  {
    F->ybuf[0] = F->ybuf[0] + NUM[i] * F->xbuf[i] - DEN[i] * F->ybuf[i];
  }
  
  F->filtered_value = F->ybuf[0];
  
  return F->filtered_value;
}
#endif

float low_pass_filter(float input, float prev_output, float alpha)
{
    return alpha * input + (1 - alpha) * prev_output;
}


void LADRC_TD(LADRC_NUM *LADRC_TYPE1, float Expect)
{
	float fh = -LADRC_TYPE1->r * LADRC_TYPE1->r * (LADRC_TYPE1->v1 - Expect) - 2 * LADRC_TYPE1->r * LADRC_TYPE1->v2;		//fh = r*r*(v1-in)-2r*v2
	LADRC_TYPE1->v1 += LADRC_TYPE1->v2 * LADRC_TYPE1->h;                                                                //v1 = v1+v2*h
	LADRC_TYPE1->v2 += fh * LADRC_TYPE1->h;                                                                             //v2 = v2+fh*h
}

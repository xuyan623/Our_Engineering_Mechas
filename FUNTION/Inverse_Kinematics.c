#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// =========================================================================
// 矩阵运算辅助函数
// =========================================================================

// 3x3 矩阵乘法: C = A * B
void mat_mul_3x3(double A[3][3], double B[3][3], double C[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            C[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

// 3x3 矩阵转置: AT = A'
void mat_transpose_3x3(double A[3][3], double AT[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            AT[j][i] = A[i][j];
        }
    }
}

// =========================================================================
// 核心逆解函数
// =========================================================================
// 返回值：找到的有效解的数量
// T_target: 4x4 目标齐次变换矩阵
// q_sols: 用于存储输出解的二维数组 (最多支持返回多个解，目前按你的逻辑返回1组)
// =========================================================================
int custom_arm_ik(double T_target[4][4], double q_sols[][6]) {
    // --- 1. 定义机械臂物理参数 ---
    double L1 = 0.275; 
    double L2 = 0.15; 
    double L3 = 0.173; 
    double L4 = 0.055;
    
    // 核心 DH 参数提取
    double d3 = 0.176; 
    double d4 = L2 + L3; 
    double d6 = L4;      
    
    int sol_count = 0; // 记录解的个数
    
    // --- 2. 提取目标位置与姿态 ---
    double R_target[3][3];
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            R_target[i][j] = T_target[i][j];
        }
    }
    double P_target[3] = {T_target[0][3], T_target[1][3], T_target[2][3]};
    
    // --- 3. 求解手腕中心 (Wrist Center, WC) 位置 ---
    // P_wc = P_target - d6 * R_target * [0; 0; 1] (即 R_target 的第三列乘以 d6)
    double P_wc[3];
    P_wc[0] = P_target[0] - d6 * R_target[0][2];
    P_wc[1] = P_target[1] - d6 * R_target[1][2];
    P_wc[2] = P_target[2] - d6 * R_target[2][2];
    
    double xw = P_wc[0], yw = P_wc[1], zw = P_wc[2];
    
    // --- 4. 求解关节 1 (Yaw) ---
    double r_xy = sqrt(xw*xw + yw*yw);
    if (r_xy < fabs(d3)) {
        // printf("Error: 目标位置不可达，手腕中心距离 Z0 轴太近。\n");
        return 0; // 返回 0 表示无解
    }
    
    double phi = atan2(yw, xw);
    double alpha = asin(d3 / r_xy);
    
    // 注：你的 MATLAB 代码这里只保留了一组解 (th1 = phi + alpha)
    // 若要全解，可扩展另一组: phi - alpha + M_PI
    double th1 = phi + alpha;
    
    // --- 5. 求解关节 2 和 3 ---
    double x1 = xw * cos(th1) + yw * sin(th1);
    double z1 = zw; 
    
    double D = (L1*L1 + d4*d4 - (x1*x1 + z1*z1)) / (2 * L1 * d4);
    if (fabs(D) > 1.0) {
        return 0; // 无解，跳过
    }
    
    // 注：你的 MATLAB 代码这里只保留了一组解 th3_up
    double th3 = atan2(D, sqrt(1.0 - D*D)); 
    
    double A = L1 - d4 * sin(th3);
    double B = d4 * cos(th3);
    double th2 = atan2(A * z1 - B * x1, A * x1 + B * z1);
    
    // --- 6. 求解姿态关节 (手腕 4, 5, 6) ---
    double c1 = cos(th1), s1 = sin(th1);
    double c2 = cos(th2), s2 = sin(th2);
    double c3 = cos(th3), s3 = sin(th3);
    
    double R0_1[3][3] = { {c1, -s1, 0}, {s1, c1, 0}, {0, 0, 1} };
    double R1_2[3][3] = { {c2, -s2, 0}, {0, 0, -1}, {s2, c2, 0} };
    double R2_3[3][3] = { {c3, -s3, 0}, {s3, c3, 0}, {0, 0, 1} };
    
    double R0_2[3][3], R0_3[3][3], R0_3_T[3][3], R3_6[3][3];
    
    // R0_3 = R0_1 * R1_2 * R2_3
    mat_mul_3x3(R0_1, R1_2, R0_2);
    mat_mul_3x3(R0_2, R2_3, R0_3);
    
    // R3_6 = R0_3' * R_target
    mat_transpose_3x3(R0_3, R0_3_T);
    mat_mul_3x3(R0_3_T, R_target, R3_6);
    
    // c5 = R3_6(2, 3) -> C语言中索引为 [1][2]
    double c5 = R3_6[1][2];
    
    // 注：你的 MATLAB 代码这里只保留了一组解 th5_neg
    double th5 = atan2(-sqrt(1.0 - c5*c5), c5);
    
    double th4, th6;
    if (fabs(sin(th5)) > 1e-6) {
        // th4 = atan2(R3_6(3, 3)/sin(th5), -R3_6(1, 3)/sin(th5))
        th4 = atan2(R3_6[2][2]/sin(th5), -R3_6[0][2]/sin(th5));
        // th6 = atan2(-R3_6(2, 2)/sin(th5), R3_6(2, 1)/sin(th5))
        th6 = atan2(-R3_6[1][1]/sin(th5), R3_6[1][0]/sin(th5));
    } else {
        // 万向节死锁 (Singularity) 处理
        th4 = 0; 
        if (c5 > 0) {
            th6 = atan2(-R3_6[2][0], R3_6[0][0]);
        } else {     
            th6 = -atan2(R3_6[2][0], -R3_6[0][0]);
        }
    }
    
    // 保存并归一化解 (-pi 到 pi)
    double q_raw[6] = {th1, th2, th3, th4, th5, th6};
    for(int i=0; i<6; i++) {
        q_sols[sol_count][i] = atan2(sin(q_raw[i]), cos(q_raw[i]));
    }
    sol_count++;
    
    return sol_count;
}


    

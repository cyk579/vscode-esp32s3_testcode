#ifndef _ENCODER_H_
#define _ENCODER_H_

// 初始化三个电机的编码器
void encoder_init(void);

// 获取三个轮子的平滑速度（只读，不影响后台采样）
void encoder_get_speeds(float *speed_a, float *speed_b, float *speed_d);

// 获取三个轮子的原始速度（未平滑）
void encoder_get_raw_speeds(float *speed_a, float *speed_b, float *speed_d);

#endif
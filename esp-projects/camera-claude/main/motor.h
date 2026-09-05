#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <stdint.h>

// 初始化电机外设
void motor_init(void);

// 控制各个电机 (dir: 1正转, -1反转, 0停止 | speed: 0~8191)
void set_motor_A(int dir, uint32_t speed);
void set_motor_B(int dir, uint32_t speed);
void set_motor_D(int dir, uint32_t speed);

#endif
#ifndef _PID_H_
#define _PID_H_

#include <stdint.h>

typedef struct {
	int direction;
	uint32_t speed;
} pid_motor_state_t;

// 初始化循迹 PID 参数和内部状态
void pid_init(void);

// 根据循迹误差计算修正量，并更新三路电机
void pid_update(float error);

// 获取最近一次发送给三个电机的方向和速度
void pid_get_motor_states(pid_motor_state_t *motor_A,
						  pid_motor_state_t *motor_B,
						  pid_motor_state_t *motor_D);

// 停止三路电机
void pid_stop(void);

// 手动控制三路电机功率
void pid_manual_control(int power_A, int power_B, int power_D);

#endif

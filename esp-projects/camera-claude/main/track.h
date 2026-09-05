#ifndef _TRACK_H_
#define _TRACK_H_

// 初始化红外循迹模块
void track_init(void);

// 读取四路传感器状态：0 为低电平，1 为高电平
// 传感器定义：0 为黑色，1 为白色或自然光
void track_get_sensor_states(int *out1, int *out2, int *out3, int *out4);

// 判断最近一次采样是否为全白丢线状态
int track_is_line_lost(void);

// 判断是否回到黑线中心区域
int track_is_centered(void);

// 判断四路红外是否全为 0（0000 状态）
int track_is_all_zero(void);

// 获取当前的加权平均偏差
// 返回值：0 (居中), 正数 (偏左，需要右转修正), 负数 (偏右，需要左转修正)
float get_track_error(void);

#endif


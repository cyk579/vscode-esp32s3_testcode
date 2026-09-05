#ifndef _ULTRASONIC_H_
#define _ULTRASONIC_H_

// 初始化超声波传感器
void ultrasonic_init(void);

// 获取前方障碍物距离 (单位: 厘米)
float ultrasonic_get_distance_cm(void);

#endif
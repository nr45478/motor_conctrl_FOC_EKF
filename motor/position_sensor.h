/**********************************
 * 高精度位置传感器模块头文件
 **********************************/
#ifndef __POSITION_SENSOR_H_
#define __POSITION_SENSOR_H_

#include "main.h"

// 位置获取方法选择
typedef enum {
    POS_METHOD_HALL_ONLY = 0,      // 仅使用霍尔（60°精度）
    POS_METHOD_HALL_INTERP = 1,    // 霍尔+速度插值（推荐）
    POS_METHOD_EKF = 2             // EKF估算（无感算法）
} POSITION_METHOD;

// 位置传感器配置
typedef struct {
    POSITION_METHOD method;         // 当前使用的位置获取方法
    real32_T interpolated_angle;    // 插值后的连续角度
    real32_T last_hall_angle;       // 上次霍尔角度
    uint8_t hall_changed_flag;      // 霍尔状态变化标志
	  uint8_t last_hall_state;       // ✅ 新增：用于判断霍尔跳变
	    // ===== 新增：解包用 =====
    float elec_prev_wrap;   // 上一次的电角(0~2π)
    float elec_unwrap;      // 连续电角(不回零，一直累加)
    float mech_unwrap;      // 连续机械角 = elec_unwrap / POLE_PAIRS
} POSITION_SENSOR_DEF;

// 全局变量
extern POSITION_SENSOR_DEF Position_Sensor;
float wrapToPi(float x);
float wrapTo2Pi(float x);
// 函数声明
void position_sensor_init(POSITION_METHOD method);
real32_T get_high_precision_position(void);
void position_sensor_update(void);  // 在中断中调用

#endif
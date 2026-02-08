/**********************************
 * 控制模式管理模块头文件
 **********************************/
#ifndef __CONTROL_MODE_H_
#define __CONTROL_MODE_H_

#include "main.h"

// 控制模式枚举
typedef enum {
    MODE_SPEED = 0,        // 速度控制模式（原有）
    MODE_POSITION = 1      // 位置控制模式（新增）
} CONTROL_MODE_ENUM;

// 控制模式管理结构体
typedef struct {
    CONTROL_MODE_ENUM mode;        // 当前模式
    CONTROL_MODE_ENUM last_mode;   // 上一次模式
    uint8_t mode_switch_request;   // 模式切换请求标志
    uint8_t mode_init_flag;        // 模式初始化完成标志
} CONTROL_MODE_DEF;

// 全局变量
extern CONTROL_MODE_DEF Control_Mode;

// 函数声明
void control_mode_init(void);
void switch_to_speed_mode(void);
void switch_to_position_mode(void);
void control_mode_handle(void);  // 在主循环中调用

#endif


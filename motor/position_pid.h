/**********************************
 * 位置环PID控制器头文件
 **********************************/
#ifndef __POSITION_PID_H_
#define __POSITION_PID_H_

#include "main.h"

// 适配50Hz（314rad/s）的参数修改
#define POSITION_PI_P        18.0F     // 从0.5提升到5.0（增强纠偏力度）
#define POSITION_PI_I        0.0F     // 保留0.1，避免积分饱和
#define POSITION_PI_D        0.0F     // 仍关闭微分
// 核心修改：速度限幅从50rad/s→350rad/s（50Hz×2π≈314rad/s，留余量）
#define POSITION_PI_UP_LIMIT   50.0F  
#define POSITION_PI_LOW_LIMIT  -50.0F
#define POSITION_PID_PERIOD  0.001F  // 保留1ms周期

// 结构体/全局变量/函数声明 保留不变
typedef struct {
    real32_T P_Gain;        
    real32_T I_Gain;        
    real32_T D_Gain;        
    real32_T B_Gain;        
    real32_T Max_Output;    
    real32_T Min_Output;    
    real32_T I_Sum;         
    real32_T Last_Error;    
} POSITION_PID_DEF;

extern real32_T Position_Ref;      
extern real32_T Position_Fdk;      
extern real32_T Position_Pid_Out;  
extern POSITION_PID_DEF Position_Pid;

void Position_Pid_Calc(real32_T ref_temp, real32_T fdb_temp, 
                       real32_T* out_temp, POSITION_PID_DEF* pid_temp);
void position_pid_initialize(void);
real32_T position_error_calc(real32_T ref, real32_T fdb);

#endif


/**********************************
 * 位置环PID控制器实现
 **********************************/
#include "position_pid.h"
#include "speed_pid.h"  // 需要访问Speed_Ref
#include "hall_sensor.h"
// 全局变量定义
real32_T Position_Ref = 0.0F;      // 位置给定
real32_T Position_Fdk = 0.0F;      // 位置反馈
real32_T Position_Pid_Out = 0.0F;  // 位置PID输出
POSITION_PID_DEF Position_Pid;
#define POSITION_DAMPING     15.0F    // 阻尼系数，需要调试
static inline float wrapToPi(float x){
    while(x >  PI) x -= 2.0f*PI;
    while(x <= -PI) x += 2.0f*PI;
    return x;
}

/**
 * @brief  位置误差计算（处理角度环绕）
 * @note   解决0°和360°跳变问题，计算最短路径误差
 * @param  ref: 目标位置 (rad)
 * @param  fdb: 反馈位置 (rad)
 * @retval 误差值 (rad)，范围[-π, π]
 */
//real32_T position_error_calc(real32_T ref, real32_T fdb)
//{
//    real32_T error = ref - fdb;
//    
//    // 处理角度环绕：确保误差在[-π, π]范围内
//    while (error > PI) {
//        error -= 2.0F * PI;
//    }
//    while (error < -PI) {
//        error += 2.0F * PI;
//    }
//    
//    return error;
//}

/**
 * @brief  位置环PID计算
 * @note   输入位置给定和反馈，输出速度指令
 * @param  ref_temp: 位置给定 (rad)
 * @param  fdb_temp: 位置反馈 (rad)
 * @param  out_temp: 输出速度指令 (rad/s)
 * @param  pid_temp: PID参数结构体
 * @retval None
 */
//real32_T error;
//void Position_Pid_Calc(real32_T ref_temp, real32_T fdb_temp, 
//                       real32_T* out_temp, POSITION_PID_DEF* pid_temp)
//{

//    real32_T temp;
//    real32_T d_term = 0.0F;
//    
//    // 1. 计算位置误差（处理角度环绕）
//    error = position_error_calc(ref_temp, fdb_temp);
//    if(error>5.0)error = 5.0;
//	  if(error<-5.0) error = -5.0;
//    // 2. 微分项计算（可选，初期可设D_Gain=0）
//    if (pid_temp->D_Gain > 0.0001F) {
//        d_term = pid_temp->D_Gain * (error - pid_temp->Last_Error) 
//                 / POSITION_PID_PERIOD;
//        pid_temp->Last_Error = error;
//    }
//    
//    // 3. PID计算：P + I + D
//    temp = pid_temp->P_Gain * error + pid_temp->I_Sum + d_term;
//    
//    // 4. 输出限幅
//    if (temp > pid_temp->Max_Output) {
//        *out_temp = pid_temp->Max_Output;
//    } else if (temp < pid_temp->Min_Output) {
//        *out_temp = pid_temp->Min_Output;
//    } else {
//        *out_temp = temp;
//    }
//    
//    // 5. 积分项更新（带抗饱和）
//    // 只有当输出未饱和时才累积积分
//    pid_temp->I_Sum += ((*out_temp - temp) * pid_temp->B_Gain 
//                        + pid_temp->I_Gain * error) * POSITION_PID_PERIOD;
//    
//    // 6. 积分限幅（防止积分过大）
//    if (pid_temp->I_Sum > pid_temp->Max_Output) {
//        pid_temp->I_Sum = pid_temp->Max_Output;
//    } else if (pid_temp->I_Sum < pid_temp->Min_Output) {
//        pid_temp->I_Sum = pid_temp->Min_Output;
//    }
//}
// 先定义全局变量：用于位置反馈滤波（平滑霍尔跳变）
float Position_Fdb_Filtered = 0.0F;
// 滤波系数（0.1~0.3，越小越平滑，先设0.2）
#define POS_FILTER_ALPHA  0.2F
real32_T error = 0.0;
// 角度单位转换：度→弧度
#define DEG_TO_RAD(deg) ((deg) * 3.1415926F / 180.0F)
// 位置误差计算（单独封装，处理环绕+限幅）
real32_T position_error_calc(real32_T ref, real32_T fdb)
{
    error = ref - fdb;
    
    // 处理角度环绕
    while (error > PI) error -= 2.0F * PI;
    while (error < -PI) error += 2.0F * PI;
    
    // 限幅±30°（半个扇区，合理范围）
    // 30° = π/6 ≈ 0.524 rad
    const real32_T MAX_ERROR = 3.14F;
    if (error > MAX_ERROR) error = MAX_ERROR;
    else if (error < -MAX_ERROR) error = -MAX_ERROR;
    
    return error;
}

void Position_Pid_Calc(real32_T ref_temp, real32_T fdb_temp, 
                       real32_T* out_temp, POSITION_PID_DEF* pid_temp)
{
//    real32_T error, temp;
//    
//    // 滤波
//    Position_Fdb_Filtered += POS_FILTER_ALPHA * wrapToPi(fdb_temp - Position_Fdb_Filtered);
//    
//    // 计算误差
//    error = position_error_calc(ref_temp, Position_Fdb_Filtered);
//    
//    // 【关键修改】PI控制，但积分要限幅
//    // P项
//    temp = pid_temp->P_Gain * error;
//    
//    // I项（只在小误差时累积，积分分离）
//    if (fabsf(error) < 0.3F) {  // 误差小于0.3rad才累积积分
//        pid_temp->I_Sum += pid_temp->I_Gain * error * POSITION_PID_PERIOD;
//    }
//    
//    // 【关键】积分限幅，设为最大输出的30%~50%
//    const real32_T I_LIMIT = pid_temp->Max_Output * 0.3F;
//    if (pid_temp->I_Sum > I_LIMIT) pid_temp->I_Sum = I_LIMIT;
//    else if (pid_temp->I_Sum < -I_LIMIT) pid_temp->I_Sum = -I_LIMIT;
//    
//    // P + I
//    temp += pid_temp->I_Sum;
    real32_T error, temp;
    real32_T damping_term;
    
    // 滤波
    Position_Fdb_Filtered += POS_FILTER_ALPHA * wrapToPi(fdb_temp - Position_Fdb_Filtered);
    
    // 计算误差
    error = position_error_calc(ref_temp, Position_Fdb_Filtered);
    
    // P项
    temp = pid_temp->P_Gain * error;
    
    // 【关键】阻尼项：使用实际速度（hall_speed或mech_unwrap的导数）
    // 负号表示阻尼方向与速度相反
    damping_term = -POSITION_DAMPING * hall_speed;
    
    // I项（积分分离）
    if (fabsf(error) < 0.3F) {
        pid_temp->I_Sum += pid_temp->I_Gain * error * POSITION_PID_PERIOD;
    }
    
    // 积分限幅
    const real32_T I_LIMIT = pid_temp->Max_Output * 0.3F;
    if (pid_temp->I_Sum > I_LIMIT) pid_temp->I_Sum = I_LIMIT;
    else if (pid_temp->I_Sum < -I_LIMIT) pid_temp->I_Sum = -I_LIMIT;
    
    // P + I + 阻尼
    temp += pid_temp->I_Sum + damping_term;
    
    // 输出限幅
    if (temp > pid_temp->Max_Output) {
        *out_temp = pid_temp->Max_Output;
        // 抗饱和：输出饱和时，阻止积分继续增长
        if (error > 0) pid_temp->I_Sum -= pid_temp->I_Gain * error * POSITION_PID_PERIOD;
    } else if (temp < pid_temp->Min_Output) {
        *out_temp = pid_temp->Min_Output;
        if (error < 0) pid_temp->I_Sum -= pid_temp->I_Gain * error * POSITION_PID_PERIOD;
    } else {
        *out_temp = temp;
    }
    
    pid_temp->Last_Error = error;
}
/**
 * @brief  位置环PID初始化
 * @note   设置PID参数，清空状态变量
 * @retval None
 */
void position_pid_initialize(void)
{
    // PID参数配置
    Position_Pid.P_Gain = POSITION_PI_P;
    Position_Pid.I_Gain = POSITION_PI_I;
    Position_Pid.D_Gain = POSITION_PI_D;
    Position_Pid.B_Gain = 0.015F;  // 抗饱和增益（可与速度环保持一致）
    Position_Pid.Max_Output = POSITION_PI_UP_LIMIT;
    Position_Pid.Min_Output = POSITION_PI_LOW_LIMIT;
    
    // 状态变量清零
    Position_Pid.I_Sum = 0.0F;
    Position_Pid.Last_Error = 0.0F;
    
    // 初始位置设定为当前位置（防止启动冲击）
    Position_Ref = 0.0F;
    Position_Fdk = 0.0F;
    Position_Pid_Out = 0.0F;
}
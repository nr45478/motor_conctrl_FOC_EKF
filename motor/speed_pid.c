/**********************************
            
**********************************/
#include "speed_pid.h"


#define SPEED_PID_PERIOD 0.001F

real32_T SPEED_PI_I = 5.0F;
real32_T SPEED_PI_KB = 0.015F;
real32_T SPEED_PI_LOW_LIMIT = -5.0F;
real32_T SPEED_PI_P = 0.003F;
real32_T SPEED_PI_UP_LIMIT = 5.0F;

                   
real32_T Speed_Ref;       //�ٶȲο�          
real32_T Speed_Fdk;        //�ٶȷ���          
real32_T Speed_Pid_Out;    //�ٶ�PID�����Ҳ����Q��������Ĳο�             

SPEED_PID_DEF Speed_Pid;

/**
  * @brief  速度环PI控制器计算函数（增量式PI+积分分离+限幅保护）
  * @note   适用于STM32 FOC电机控制，输入为Hz，内部转换为rad/s计算，输出为电流环参考值
  * @param  ref_temp: 速度参考值 (单位：Hz，外部输入的目标转速)
  * @param  fdb_temp: 速度反馈值 (单位：rad/s，霍尔/编码器计算的实际转速)
  * @param  out_temp: PI输出值指针 (输出：电流环参考值，需限幅在Max/Min_Output之间)
  * @param  current_pid_temp: PI参数结构体指针 (包含Kp/Ki/B_Gain/积分值/输出限幅等)
  * @retval 无
  */
void Speed_Pid_Calc(real32_T ref_temp,real32_T fdb_temp,real32_T* out_temp,SPEED_PID_DEF* current_pid_temp)
{
  // 局部变量定义
  real32_T error;        // 速度误差值（rad/s）
  real32_T temp;         // PI计算中间值（未限幅）

  // 1. 计算速度误差：参考值转rad/s - 反馈值（统一单位为rad/s）
  // 6.28318548 = 2*π，将外部输入的Hz（转/秒）转换为rad/s（弧度/秒）
  // 例：1Hz = 2π rad/s → 对应电机每秒转1圈，角速度为2π弧度
  error = 6.28318548F * ref_temp - fdb_temp;            

  // 2. 比例项(P) + 积分项(I) 初步计算（未限幅）
  // current_pid_temp->P_Gain: 比例系数Kp
  // current_pid_temp->I_Sum: 积分累计值（历史积分和）
  temp = (error + current_pid_temp->I_Sum) * current_pid_temp->P_Gain;

  // 3. 输出限幅保护：防止PI输出超过电流环最大/最小驱动能力
  // Max_Output: 电流环最大输出（如电机最大相电流对应的参考值）
  // Min_Output: 电流环最小输出（反向最大电流）
  if (temp > current_pid_temp->Max_Output) {
    *out_temp = current_pid_temp->Max_Output;  // 超过上限，限幅为最大值
  } else if (temp < current_pid_temp->Min_Output) {
    *out_temp = current_pid_temp->Min_Output;  // 低于下限，限幅为最小值
  } else {
    *out_temp = temp;                          // 正常范围，输出原始计算值
  }

  // 4. 积分项更新（增量式积分 + 积分分离（B_Gain） + 积分限幅隐含在输出限幅）
  // 核心逻辑：仅当输出未限幅时，积分才累加；限幅时积分停止，防止积分饱和
  // (*out_temp - temp): 限幅偏差，限幅时为负数/正数，正常时为0
  // current_pid_temp->B_Gain: 积分分离系数（通常为0~1，0=完全积分分离，1=无分离）
  // current_pid_temp->I_Gain: 积分系数Ki
  // SPEED_PID_PERIOD: PI计算周期（秒，如100Hz计算则为0.01s）
  current_pid_temp->I_Sum += ((*out_temp - temp) * current_pid_temp->B_Gain + current_pid_temp->I_Gain* error) * SPEED_PID_PERIOD;
}


void speed_pid_initialize(void)
{
  Speed_Pid.P_Gain = SPEED_PI_P;
  Speed_Pid.I_Gain = SPEED_PI_I;
  Speed_Pid.B_Gain = SPEED_PI_KB;
  Speed_Pid.Max_Output = SPEED_PI_UP_LIMIT;
  Speed_Pid.Min_Output = SPEED_PI_LOW_LIMIT;
  Speed_Pid.I_Sum = 0.0f;
}



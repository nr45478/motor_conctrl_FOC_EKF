/**********************************
 * 控制模式管理模块实现
 **********************************/
#include "control_mode.h"
#include "position_pid.h"
#include "speed_pid.h"
#include "position_sensor.h"

// 全局变量
CONTROL_MODE_DEF Control_Mode;

/**
 * @brief  控制模式初始化
 * @note   默认启动为速度模式
 * @retval None
 */
void control_mode_init(void)
{
    Control_Mode.mode = MODE_SPEED;  // 默认速度模式
    Control_Mode.last_mode = MODE_SPEED;
    Control_Mode.mode_switch_request = 0;
    Control_Mode.mode_init_flag = 0;
}

/**
 * @brief  切换到速度模式
 * @note   保存当前位置，清空位置环积分
 * @retval None
 */
//	void switch_to_speed_mode(void)
//	{
//		// 1. 清空位置环积分（防止冲击）
//		Position_Pid.I_Sum = 0.0F;
//		Position_Pid.Last_Error = 0.0F;
//		
//		// 2. 保持当前速度（平滑切换）
//		// Speed_Ref 保持不变
//		
//		// 3. 切换模式标志
//		Control_Mode.mode = MODE_SPEED;
//		Control_Mode.mode_init_flag = 1;
//		
//		// 调试信息（可选）
//		// printf("Switched to SPEED mode\r\n");
//	}
//void switch_to_speed_mode(void)
//{
//    // 1. 清空位置环状态
//    Position_Pid.I_Sum = 0.0F;
//    Position_Pid.Last_Error = 0.0F;
//    
//    // 2. 【重要】将当前速度保持为速度给定（平滑切换）
//    Speed_Ref = Speed_Fdk / (2.0F * PI);  // 转换为Hz
//    
//    // 3. 切换模式标志
//    Control_Mode.mode = MODE_SPEED;
//    Control_Mode.mode_init_flag = 1;
//}
// 1. 修复 switch_to_speed_mode（Speed_Ref=带符号的Hz）
void switch_to_speed_mode(void)
{
    Position_Pid.I_Sum = 0.0F;
    Position_Pid.Last_Error = 0.0F;
    
    // Speed_Fdk(rad/s) → 带符号的Hz（保留方向）
    Speed_Ref = Speed_Fdk / 6.28318548F;  // 比如Speed_Fdk=157rad/s → 25Hz（正向）
    
    Speed_Pid.I_Sum = Speed_Pid_Out / Speed_Pid.P_Gain;
    Control_Mode.mode = MODE_SPEED;
    Control_Mode.mode_init_flag = 1;
}
/**
 * @brief  切换到位置模式
 * @note   初始化位置给定为当前位置，防止突变
 * @retval None
 */
//void switch_to_position_mode(void)
//{
//    // 1. 读取当前位置
//    Position_Fdk = get_high_precision_position();
//    
//    // 2. 设置位置给定=当前位置（无冲击切换）
//    Position_Ref = Position_Fdk;
//    
//    // 3. 清空位置环状态
//    Position_Pid.I_Sum = 0.0F;
//    Position_Pid.Last_Error = 0.0F;
//    
//    // 4. 清空速度环积分（继承位置环输出）
//    Speed_Pid.I_Sum = 0.0F;
//    
//    // 5. 切换模式标志
//    Control_Mode.mode = MODE_POSITION;
//    Control_Mode.mode_init_flag = 1;
//    
//    // 调试信息（可选）
//    // printf("Switched to POSITION mode, Current pos: %.2f rad\r\n", Position_Fdk);
//}
	void switch_to_position_mode(void)
	{
			// 1. 读取当前位置作为初始位置给定
//			Position_Fdk = get_high_precision_position();
			Position_Ref = Position_Fdk;
			
			// 2. 清空位置环状态
			Position_Pid.I_Sum = 0.0F;
			Position_Pid.Last_Error = 0.0F;
			
			// 3. 【重要】将当前速度环输出预加载到速度环积分项（平滑切换）
			Speed_Pid.I_Sum = Speed_Pid_Out / Speed_Pid.P_Gain;
			
			// 4. 切换模式标志
			Control_Mode.mode = MODE_POSITION;
			Control_Mode.mode_init_flag = 1;
	}
/**
 * @brief  控制模式处理函数
 * @note   在主循环或低频任务中调用，处理模式切换
 * @retval None
 */
void control_mode_handle(void)
{
    // 检测模式切换请求
    if (Control_Mode.mode_switch_request) {
        Control_Mode.mode_switch_request = 0;  // 清除标志
        
        // 执行模式切换
        if (Control_Mode.mode == MODE_SPEED) {
            switch_to_position_mode();
        } else {
            switch_to_speed_mode();
        }
        
        Control_Mode.last_mode = Control_Mode.mode;
    }
}

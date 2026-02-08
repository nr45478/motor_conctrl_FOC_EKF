///**********************************
//      
//**********************************/
//#include "main.h"
//#include "low_task.h"
//#include "adc.h"
//#include "control_mode.h"  // 需要引入控制模式头文件
//#include "position_pid.h"
//#include "position_sensor.h"
//uint16_t key2_cnt = 0;        // 新增：按键2计数器
//uint8_t key2_press_flag = 0;  // 新增：按键2长按检测标志
//u16 hz_100_cnt = 0;
//uint8_t motor_start_stop = 0;
//uint8_t motor_start_stop_pre = 1;

//uint16_t key1_cnt;
//uint8_t key1_press_flag = 0;

///**
// * @brief  电机启动函数
// * @note   调用此函数后，电机将完成初始化并进入运行状态
// * @param  无
// * @retval 无
// */
//void motor_start(void)
//{
//  // 1. 置位GPIOC的9号引脚（通常用于控制驱动使能/电源使能，拉高=打开驱动）
//  GPIO_SetBits(GPIOC,GPIO_Pin_9);
//  
//  // 2. FOC算法初始化（重置FOC核心参数：Clark/Park变换、SVPWM、电流采样等）
//  foc_algorithm_initialize();
//  
//  // 3. 设置初始速度给定值：电机方向(motor_direction) × 25rad/s
//  //    motor_direction=1→正向25rad/s，=-1→反向25rad/s，25rad/s≈238rpm（25*60/(2π)）
//  Speed_Ref = motor_direction * (25.0F);//初始速度给定（正反转由motor_direction决定）
//  
//  // 4. 速度闭环标志置0（表示暂未进入速度闭环，先开环启动/寻相，避免闭环过早介入导致抖动）
//  speed_close_loop_flag=0;
//  
//  // 5. 初始化q轴参考电流为0（启动瞬间无电流输出，避免电机突冲）
//  Iq_ref=0.0f;

//  // 6. 设置霍尔角度补偿增量（用于低速/启动阶段的霍尔角度精细化补偿，提升启动精度）
//  hall_angle_add=0.0005f;
//  
//  // 7. 初始化霍尔速度为0（清除上次运行的速度残留，保证启动时速度检测从零开始）
//  hall_speed = 0.0f;
//  
//  // 8. 使能PWM输出（TIM定时器的PWM通道，驱动DRV8301等功率器件输出电压）
//  TIM_CtrlPWMOutputs(PWM_TIM,ENABLE);
//  
//  // 9. 设置电机运行显示标志为1（OLED屏会检测此标志，显示"Motor:run"状态）
//  motor_run_display_flag = 1;
//}
//void motor_stop(void)
//{
//  GPIO_ResetBits(GPIOC,GPIO_Pin_9);
//  TIM_CtrlPWMOutputs(PWM_TIM,DISABLE);
//  motor_run_display_flag = 0;
//}


////// 底层控制任务函数（电机启停、按键控制等核心逻辑）
////void low_control_task(void)
////{
////    // 当偏移标志位为2时（偏移校准完成/到位状态）
////    if(get_offset_flag == 2)
////    {
////        // 检测电机启停状态是否发生变化（防止重复执行启停操作）
////        if(motor_start_stop_pre!=motor_start_stop)
////        {
////            // 更新电机启停状态前值（保存当前状态作为下次对比基准）
////            motor_start_stop_pre=motor_start_stop;
////            
////            // 如果电机启停指令为1（启动）
////            if(motor_start_stop == 1)
////            {
////                motor_start();  // 执行电机启动函数
////            }
////            else  // 电机启停指令为0（停止）
////            {
////                motor_stop();   // 执行电机停止函数
////            }     
////        }
////    }
////	
////    // 检测按键1按下标志位（key1_flag=1表示按键1被按下）
////    if(key1_flag==1)
////    {
////        key1_press_flag = 1;  // 设置按键1长按检测标志位
////    }
////    
////    // 如果按键1长按检测标志位已置位（进入按键1长按/短按判断逻辑）
////    if(key1_press_flag)
////    {
////        key1_cnt++;  // 按键1按下计时计数器自增（用于判断长按时间）
////        
////        // 短按判断：计时未到100且按键已释放（key1_flag=0表示释放）
////        if(key1_cnt<100 && key1_flag == 0)
////        {	
////            // 短按逻辑：切换电机启停状态
////            if(motor_start_stop==0)  // 当前是停止状态
////            {
////                motor_start_stop=1;  // 设置为启动状态
////            }
////            else  // 当前是启动状态
////            {
////                motor_start_stop=0;  // 设置为停止状态
////            }	
////            
////            // 重置按键1相关标志和计数器（清除本次按键状态）
////            key1_flag=0;
////            key1_cnt = 0;
////            key1_press_flag = 0;
////        }
////        // 长按判断：计时超过100（达到长按阈值）
////        else if(key1_cnt>100)
////        {
////            motor_stop();              // 先停止电机
////            motor_direction = -motor_direction;  // 反转电机方向
////            motor_start();	            // 重新启动电机（以新方向运行）	
////            
////            // 重置按键1相关标志和计数器（清除本次按键状态）
////            key1_cnt = 0;
////            key1_flag = 0;
////            key1_press_flag = 0;
////        }
////    } 
////	
////    // 检测按键2按下标志位（key2_flag=1表示按键2被按下）
////    if(key2_flag==1)
////    {
////        display_flag=1;  // 设置显示更新标志位（用于刷新速度/状态显示）
////        
////        // 电机正方向运行时（direction≠-1表示正方向）
////        if(motor_direction!= -1.0f)	
////        {
////            // 速度参考值大于25时（下限保护）
////            if(Speed_Ref>25.0f)
////                Speed_Ref-=5.0f;  // 速度参考值减5（降低速度）
////        }
////        else  // 电机反方向运行时
////        {
////            // 速度参考值小于-25时（下限保护）
////            if(Speed_Ref<-25.0f)
////                Speed_Ref+=5.0f;  // 速度参考值加5（降低反向速度）		
////        }

////        key2_flag=0;  // 清除按键2按下标志（防止重复执行）
////    }
////	
////    // 检测按键3按下标志位（key3_flag=1表示按键3被按下）
////    if(key3_flag==1)
////    {
////        // 电机正方向运行时
////        if(motor_direction!= -1.0f)	
////        {
////            Speed_Ref+=5.0f;  // 速度参考值加5（提高速度）
////            // 上限保护：速度参考值超过200时限制为200
////            if(Speed_Ref>200.0f)
////                Speed_Ref=200.0f;
////        }
////        else  // 电机反方向运行时
////        {
////            Speed_Ref-=4.0f;  // 速度参考值减4（提高反向速度）
////            // 上限保护：速度参考值低于-200时限制为-200
////            if(Speed_Ref<-200.0f)
////                Speed_Ref=-200.0f;
////        }   
////        
////        key3_flag=0;  // 清除按键3按下标志（防止重复执行）		
////    }
////}

//// 底层控制任务函数（电机启停、按键控制等核心逻辑）
////void low_control_task(void)
////{
////    // 当偏移标志位为2时（偏移校准完成/到位状态）
////    if(get_offset_flag == 2)
////    {
////        // 检测电机启停状态是否发生变化（防止重复执行启停操作）
////        if(motor_start_stop_pre != motor_start_stop)
////        {
////            // 更新电机启停状态前值（保存当前状态作为下次对比基准）
////            motor_start_stop_pre = motor_start_stop;
////            
////            // 如果电机启停指令为1（启动）
////            if(motor_start_stop == 1)
////            {
////                motor_start();  // 执行电机启动函数
////            }
////            else  // 电机启停指令为0（停止）
////            {
////                motor_stop();   // 执行电机停止函数
////            }     
////        }
////    }
////	
////    // ========== 按键1处理：启停/换向 ==========
////    // 检测按键1按下标志位（key1_flag=1表示按键1被按下）
////    if(key1_flag == 1)
////    {
////        key1_press_flag = 1;  // 设置按键1长按检测标志位
////    }
////    
////    // 如果按键1长按检测标志位已置位（进入按键1长按/短按判断逻辑）
////    if(key1_press_flag)
////    {
////        key1_cnt++;  // 按键1按下计时计数器自增（用于判断长按时间）
////        
////        // 短按判断：计时未到100且按键已释放（key1_flag=0表示释放）
////        if(key1_cnt < 100 && key1_flag == 0)
////        {	
////            // 短按逻辑：切换电机启停状态
////            if(motor_start_stop == 0)  // 当前是停止状态
////            {
////                motor_start_stop = 1;  // 设置为启动状态
////            }
////            else  // 当前是启动状态
////            {
////                motor_start_stop = 0;  // 设置为停止状态
////            }	
////            
////            // 重置按键1相关标志和计数器（清除本次按键状态）
////            key1_flag = 0;
////            key1_cnt = 0;
////            key1_press_flag = 0;
////        }
////        // 长按判断：计时超过100（达到长按阈值）
////        else if(key1_cnt > 100)
////        {
////            motor_stop();              // 先停止电机
////            motor_direction = -motor_direction;  // 反转电机方向
////            motor_start();	            // 重新启动电机（以新方向运行）	
////            
////            // 重置按键1相关标志和计数器（清除本次按键状态）
////            key1_cnt = 0;
////            key1_flag = 0;
////            key1_press_flag = 0;
////        }
////    } 
////	
////    // ========== 按键2处理：降速/切换位置模式 ==========
////    // 检测按键2按下标志位（key2_flag=1表示按键2被按下）
////    if(key2_flag == 1)
////    {
////        key2_press_flag = 1;  // 标记按键2被按下
////    }
////    
////    // 按键2长短按判断逻辑
////    if(key2_press_flag)
////    {
////        key2_cnt++;  // 计时
////        
////        // 短按判断：计时未到100且按键已释放
////        if(key2_cnt < 100 && key2_flag == 0)
////        {
////            // 短按逻辑：降速（仅在速度模式下生效）
////            if(Control_Mode.mode == MODE_SPEED)
////            {
////                display_flag = 1;  // 设置显示更新标志位
////                
////                // 电机正方向运行时（direction≠-1表示正方向）
////                if(motor_direction != -1.0f)	
////                {
////                    // 速度参考值大于25时（下限保护）
////                    if(Speed_Ref > 25.0f)
////                        Speed_Ref -= 5.0f;  // 速度参考值减5（降低速度）
////                }
////                else  // 电机反方向运行时
////                {
////                    // 速度参考值小于-25时（下限保护）
////                    if(Speed_Ref < -25.0f)
////                        Speed_Ref += 5.0f;  // 速度参考值加5（降低反向速度）		
////                }
////            }
////            
////            // 重置按键2状态
////            key2_cnt = 0;
////            key2_flag = 0;
////            key2_press_flag = 0;
////        }
////        // 长按判断：计时超过100
////        else if(key2_cnt > 100)
////        {
////            // 长按逻辑：切换到位置模式
////            Control_Mode.mode_switch_request = 1;
////            display_flag = 1;
////            
////            // 重置按键2状态
////            key2_cnt = 0;
////            key2_flag = 0;
////            key2_press_flag = 0;
////        }
////    }
////	
////    // ========== 按键3处理：升速 ==========
////    // 检测按键3按下标志位（key3_flag=1表示按键3被按下）
////    if(key3_flag == 1)
////    {
////        // 电机正方向运行时
////        if(motor_direction != -1.0f)	
////        {
////            Speed_Ref += 5.0f;  // 速度参考值加5（提高速度）
////            // 上限保护：速度参考值超过200时限制为200
////            if(Speed_Ref > 200.0f)
////                Speed_Ref = 200.0f;
////        }
////        else  // 电机反方向运行时
////        {
////            Speed_Ref -= 4.0f;  // 速度参考值减4（提高反向速度）
////            // 上限保护：速度参考值低于-200时限制为-200
////            if(Speed_Ref < -200.0f)
////                Speed_Ref = -200.0f;
////        }   
////        
////        key3_flag = 0;  // 清除按键3按下标志（防止重复执行）		
////    }
////}
//void low_control_task(void)
//{
//	   extern real32_T Position_Ref;      // 位置给定
//    // 当偏移标志位为2时（偏移校准完成/到位状态）
//    if(get_offset_flag == 2)
//    {
//        // 检测电机启停状态是否发生变化（防止重复执行启停操作）
//        if(motor_start_stop_pre != motor_start_stop)
//        {
//            // 更新电机启停状态前值（保存当前状态作为下次对比基准）
//            motor_start_stop_pre = motor_start_stop;
//            
//            // 如果电机启停指令为1（启动）
//            if(motor_start_stop == 1)
//            {
//                motor_start();  // 执行电机启动函数
//            }
//            else  // 电机启停指令为0（停止）
//            {
//                motor_stop();   // 执行电机停止函数
//            }     
//        }
//    }
//    
//    // ========== 【新增】按键2+3组合：切换显示页面 ==========
//    // 检测按键2和按键3同时按下
//    if(key2_flag == 1 && key3_flag == 1)
//    {
//        // 循环切换显示页面 0 → 1 → 2 → 3 → 0...
//        display_flag++;
//        if(display_flag > 5)
//            display_flag = 0;
//        
//        oled_switch_page(display_flag);
//        
//        // 清除按键标志，防止触发其他功能
//        key2_flag = 0;
//        key3_flag = 0;
//        key2_press_flag = 0;
//        key2_cnt = 0;
//        
//        // 等待按键释放（防止连续切换）
//        // 注意：这里简单处理，实际应该在中断中检测释放
//        return;  // 直接返回，不再处理单独按键
//    }
//	
//    // ========== 按键1处理：启停/换向 ==========
//    if(key1_flag == 1)
//    {
//        key1_press_flag = 1;
//    }
//    
//    if(key1_press_flag)
//    {
//        key1_cnt++;
//        
//        // 短按：启停
//        if(key1_cnt < 100 && key1_flag == 0)
//        {	
//            if(motor_start_stop == 0)
//            {
//                motor_start_stop = 1;
//            }
//            else
//            {
//                motor_start_stop = 0;
//            }	
//            
//            key1_flag = 0;
//            key1_cnt = 0;
//            key1_press_flag = 0;
//        }
//        // 长按：换向
//        else if(key1_cnt > 100)
//        {
//            motor_stop();
//            motor_direction = -motor_direction;
//            motor_start();
//            
//            key1_cnt = 0;
//            key1_flag = 0;
//            key1_press_flag = 0;
//        }
//    } 
//	
//    // ========== 按键2处理：降速/切换模式 ==========
//    if(key2_flag == 1)
//    {
//        key2_press_flag = 1;
//    }
//    
//    if(key2_press_flag)
//    {
//        key2_cnt++;
//        
//        // 短按：降速（仅速度模式）
//        if(key2_cnt < 100 && key2_flag == 0)
//        {
//            if(Control_Mode.mode == MODE_SPEED)
//            {
//                if(motor_direction != -1.0f)	
//                {
//                    if(Speed_Ref > 25.0f)
//                        Speed_Ref -= 5.0f;
//                }
//                else
//                {
//                    if(Speed_Ref < -25.0f)
//                        Speed_Ref += 5.0f;		
//                }
//            }
//            
//            key2_cnt = 0;
//            key2_flag = 0;
//            key2_press_flag = 0;
//        }
//        // 长按：切换控制模式
//        else if(key2_cnt > 100)
//        {
//            Control_Mode.mode_switch_request = 1;
//            
//            key2_cnt = 0;
//            key2_flag = 0;
//            key2_press_flag = 0;
//        }
//    }
//	
//    // ========== 按键3处理：升速/增加位置 ==========
//if(key3_flag == 1)
//{
//    if(Control_Mode.mode == MODE_SPEED)
//    {
//        // 速度模式：加速
//        if(motor_direction != -1.0f)	
//        {
//            Speed_Ref += 5.0f;
//            if(Speed_Ref > 200.0f)
//                Speed_Ref = 200.0f;
//        }
//        else
//        {
//            Speed_Ref -= 4.0f;
//            if(Speed_Ref < -200.0f)
//                Speed_Ref = -200.0f;
//        }
//    }
//    else  // MODE_POSITION
//    {
//        // 【修复】位置模式：增加位置给定
//        Position_Ref += 0.1F;  // 每次增加0.1弧度（约5.7度）
//        
//        // 角度规范化（保持在0~2π范围）
//        if(Position_Ref >= 2.0F * PI)
//            Position_Ref -= 2.0F * PI;
//    }
//    
//    key3_flag = 0;
//}
//}

////void SysTick_Handler(void)
////{
////  //rtspeed_ref=20.0F;
////  if(drv8301_init_ok_flag==1)
////  {
////     drv8301_protection();
////  }
////  Speed_Pid_Calc(Speed_Ref,Speed_Fdk,&Speed_Pid_Out,&Speed_Pid);
////  hz_100_cnt++;
////  if(hz_100_cnt==10)
////  {
////    //communication_handle();
////    low_control_task();
////    TimingDelay_Decrement();
////    hz_100_cnt=0; 
////  }
////  
////}
///**
// * @brief  位置环PID计算
// * @note   输入位置给定和反馈，输出速度指令
// * @param  ref_temp: 位置给定 (rad)
// * @param  fdb_temp: 位置反馈 (rad)
// * @param  out_temp: 输出速度指令 (rad/s)
// * @param  pid_temp: PID参数结构体
// * @retval None

// */
//extern real32_T Position_Ref;      // 位置给定
//extern real32_T Position_Fdk;      // 位置反馈
//extern real32_T Position_Pid_Out;  // 位置PID输出
//struct  Position_Pid;
//static uint8_t position_mode_init = 0;
////void SysTick_Handler(void)
////{
////  if(drv8301_init_ok_flag == 1)
////  {
////     drv8301_protection();
////  }
////  static uint8_t speed_cnt = 0;
////  //speed_cnt++;
////  if(Control_Mode.mode == MODE_SPEED)
////  {
////    Speed_Pid_Calc(Speed_Ref, Speed_Fdk, &Speed_Pid_Out, &Speed_Pid);
////  }
////	else
////	{
////	  Position_Pid_Calc(Position_Ref,Position_Fdk,&Position_Pid_Out,&Position_Pid);
////	}
////  hz_100_cnt++;
////  if(hz_100_cnt == 10)
////  {
////    low_control_task();
////    TimingDelay_Decrement();
////    hz_100_cnt = 0; 
////  }
////}
//volatile uint32_t g_ms = 0;
//extern  uint32_t hall_last_edge_ms;
//extern float Position_Fdb_Filtered;
//#include "position_sensor.h"
//void SysTick_Handler(void)
//{
//	g_ms++;
//		// 停转/低速超时保护：比如 50ms 没有霍尔边沿 => 认为速度为0，停止角度积分
//	if((g_ms - hall_last_edge_ms) > 500)
//	{
//			hall_speed = 0.0f;
//			hall_angle_add = 0.0f;
//	}
//  if(drv8301_init_ok_flag == 1)
//  {
//     drv8301_protection();
//  }
//  
//  // ========== 三环串级控制 ==========
////  if(speed_close_loop_flag == 2)  // 仅在启动完成后执行闭环控制
//	if(motor_start_stop==1 && get_offset_flag==2)
//  {
//		Position_Fdk = wrapTo2Pi(Position_Sensor.mech_unwrap);
//    if(Control_Mode.mode == MODE_POSITION)
//    {
//			
//      // 位置环计算（输出是 rad/s）
//		  if(g_ms%10 == 0)
//			{	
//								// 【新增】第一次进入位置模式时，同步目标位置

//				if(position_mode_init == 0)
//				{
//						Position_Ref = wrapTo2Pi(Position_Sensor.mech_unwrap);   // 目标=当前机械角表盘
//						Position_Fdb_Filtered = Position_Ref;          // 滤波器也同步
//						position_mode_init = 1;
//				}
//				
//			Position_Pid_Calc(Position_Ref, Position_Fdk, &Position_Pid_Out, &Position_Pid);
//			}
//      // 关键：rad/s → Hz，适配函数要求
//      float speed_ref_hz = Position_Pid_Out / (2.0f * PI);
//      Speed_Pid_Calc(speed_ref_hz, Speed_Fdk, &Speed_Pid_Out, &Speed_Pid);
//    }
//    else  // MODE_SPEED
//    {

//			// 【新增】退出位置模式时重置标志
//			position_mode_init = 0;
//      // 速度模式：速度环 -> 电流环
//      // Speed_Ref 由按键设定，直接计算速度环
//      Speed_Pid_Calc(Speed_Ref, Speed_Fdk, &Speed_Pid_Out, &Speed_Pid);
//    }
//  }
//  
//  // ========== 低频任务调度 (100Hz) ==========
//  hz_100_cnt++;
//  if(hz_100_cnt == 10)
//  {
//    low_control_task();
//    TimingDelay_Decrement();
//    hz_100_cnt = 0; 
//  }
//}
/**********************************
      
**********************************/
#include "main.h"
#include "low_task.h"
#include "adc.h"

volatile uint32_t g_ms = 0;
u16 hz_100_cnt = 0;
uint8_t motor_start_stop = 0;
uint8_t motor_start_stop_pre = 1;

uint16_t key1_cnt;
uint8_t key1_press_flag = 0;

void motor_start(void)
{
  GPIO_SetBits(GPIOC,GPIO_Pin_9);
  foc_algorithm_initialize();
  Speed_Ref=motor_direction*25.0F;//Ǵ֯ת̙
  speed_close_loop_flag=0;
  Iq_ref=0.0f;

  hall_angle_add=0.0005f;
  hall_speed = 0.0f;
  TIM_CtrlPWMOutputs(PWM_TIM,ENABLE);
  
  motor_run_display_flag = 1;
}
void motor_stop(void)
{
  GPIO_ResetBits(GPIOC,GPIO_Pin_9);
  TIM_CtrlPWMOutputs(PWM_TIM,DISABLE);
  motor_run_display_flag = 0;
}


void low_control_task(void)
{
	if(get_offset_flag == 2)
  {
    if(motor_start_stop_pre!=motor_start_stop)
    {
      motor_start_stop_pre=motor_start_stop;
      if(motor_start_stop == 1)
      {
        motor_start();
      }
      else
      {
        motor_stop();
      }     
    }
  }
	
	
  if(key1_flag==1)
  {
		key1_press_flag = 1;
  }
	if(key1_press_flag){
		key1_cnt++;
		if(key1_cnt<100 && key1_flag == 0)
		{	
			if(motor_start_stop==0)
			{
				motor_start_stop=1;
			}
			else
			{
				motor_start_stop=0;
			}	
			key1_flag=0;
			key1_cnt = 0;
			key1_press_flag = 0;
		}else if(key1_cnt>100){
			motor_stop();
			motor_direction = -motor_direction;
			motor_start();	
			key1_cnt = 0;
			key1_flag = 0;
			key1_press_flag = 0;
		}
	} 
	
  if(key2_flag==1)
  {
		display_flag=1;//ДʾՋѐӎ˽
		if(motor_direction!= -1.0f)	{
			if(Speed_Ref>25.0f)//خС̙׈
				Speed_Ref-=5.0f;//ҽ޸
		}else{
			if(Speed_Ref<-25.0f)//خС̙׈
				Speed_Ref+=5.0f;//ҽ޸		
		}
 
    key2_flag=0;
  }
	
  if(key3_flag==1)
  {
		if(motor_direction!= -1.0f)	
		{
		    Speed_Ref+=5.0f;//ҽ޸
			  if(Speed_Ref>200.0f)//خճ̙׈ 50
			    Speed_Ref=200.0f;
		}else{
				Speed_Ref-=4.0f;//ҽ޸
				if(Speed_Ref<-200.0f)//خճ̙׈ 50
					Speed_Ref=-200.0f;
		}   
		key3_flag=0;		
  }
}





void SysTick_Handler(void)
{
  //rtspeed_ref=20.0F;
  if(drv8301_init_ok_flag==1)
  {
     drv8301_protection();
  }
  Speed_Pid_Calc(Speed_Ref,Speed_Fdk,&Speed_Pid_Out,&Speed_Pid);
  hz_100_cnt++;
  if(hz_100_cnt==10)
  {
    //communication_handle();
    low_control_task();
    TimingDelay_Decrement();
    hz_100_cnt=0; 
  }
  
}

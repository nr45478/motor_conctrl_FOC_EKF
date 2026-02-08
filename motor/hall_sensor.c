///**********************************
//           
//**********************************/
//#include "main.h"
//#include "hall_sensor.h"
//u8 hall_read_temp;

//float hall_angle;
//float hall_angle_add;
//float hall_speed;
//volatile uint8_t hall_state = 0;
//volatile uint8_t hall_sector = 0xFF;
//volatile uint8_t hall_sector_prev = 0xFF;
//volatile int8_t  hall_dir = +1;         // +1 正向, -1 反向
//volatile uint32_t hall_last_edge_ms = 0; // 记录最近一次霍尔边沿时间(毫秒)
//// 你需要一个全局毫秒计数 g_ms（可在 SysTick 里 ++）
//extern volatile uint32_t g_ms;
//static uint8_t hall_state_last_valid = 0xFF;  // 上次有效状态
//static uint32_t hall_debounce_ms = 0;         // 消抖计时
//#define HALL_DEBOUNCE_TIME  2                 // 消抖时间 2ms
//static uint8_t hall_state_to_sector(uint8_t s)
//{
//    switch(s)
//    {
//        case 0x05: return 0;
//        case 0x04: return 1;
//        case 0x06: return 2;
//        case 0x02: return 3;
//        case 0x03: return 4;
//        case 0x01: return 5;
//        default:   return 0xFF;
//    }
//}
////void TIM2_IRQHandler(void)
////{
////  float temp;
////  if(TIM_GetFlagStatus(HALL_TIM,TIM_FLAG_CC1)==SET)
////  {
////    temp = (float)(TIM_GetCapture1(HALL_TIM));
////    hall_angle_add = (float)HALL_ANGLE_FACTOR/(float)(temp);
////    hall_speed = (float)HALL_SPEED_FACTOR/(float)(temp);
////    hall_read_temp = GPIO_ReadInputDataBit(HALL_CH3_GPIO_PORT,HALL_CH3_PIN);
////    hall_read_temp |= GPIO_ReadInputDataBit(HALL_CH2_GPIO_PORT,HALL_CH2_PIN)<<1;
////    hall_read_temp |= GPIO_ReadInputDataBit(HALL_CH1_GPIO_PORT,HALL_CH1_PIN)<<2;

////    if(hall_read_temp==0x05)
////    {
////      hall_angle = 0.0f+PHASE_SHIFT_ANGLE;
////    }
////    else if(hall_read_temp==0x04)
////    {
////      hall_angle = (PI/3.0f)+PHASE_SHIFT_ANGLE;
////    }
////    else if(hall_read_temp==0x06)
////    {
////      hall_angle = (PI*2.0f/3.0f)+PHASE_SHIFT_ANGLE;
////    }
////    else if(hall_read_temp==0x02)
////    {
////      hall_angle = PI+PHASE_SHIFT_ANGLE;
////    }
////    else if(hall_read_temp==0x03)
////    {
////      hall_angle = (PI*4.0f/3.0f)+PHASE_SHIFT_ANGLE;
////    }
////    else if(hall_read_temp==0x01)
////    {
////      hall_angle = (PI*5.0f/3.0f)+PHASE_SHIFT_ANGLE;
////    }
////    if(hall_angle<0.0f)
////    {
////      hall_angle += 2.0f*PI;
////    }
////    else if(hall_angle>(2.0f*PI))
////    {
////      hall_angle -= 2.0f*PI;
////    }
////    
////    TIM_ClearFlag(HALL_TIM,TIM_FLAG_CC1);
////  }
////  
////}
//volatile uint32_t hall_edge_cnt = 0;
//volatile uint32_t hall_last_edge_us;

//volatile uint32_t hall_sector_dt_ticks;  // 上一扇区耗时
////	void TIM2_IRQHandler(void)
////{
////    float temp;

////    if(TIM_GetFlagStatus(HALL_TIM, TIM_FLAG_CC1) == SET)
////    {
////			
////        // 读霍尔状态
////        uint8_t new_state = 0;
////        new_state  = GPIO_ReadInputDataBit(HALL_CH3_GPIO_PORT, HALL_CH3_PIN);
////        new_state |= GPIO_ReadInputDataBit(HALL_CH2_GPIO_PORT, HALL_CH2_PIN) << 1;
////        new_state |= GPIO_ReadInputDataBit(HALL_CH1_GPIO_PORT, HALL_CH1_PIN) << 2;
////        
////        // 【关键】消抖判断：状态必须真正变化，且距离上次有效跳变超过消抖时间
////        if(new_state == hall_state || (g_ms - hall_debounce_ms) < HALL_DEBOUNCE_TIME)
////        {
////            // 抖动，忽略本次中断
////            TIM_ClearFlag(HALL_TIM, TIM_FLAG_CC1);
////            return;
////        }
////        
////        // 有效跳变，更新消抖计时
////        hall_debounce_ms = g_ms;
////        hall_state = new_state;
////        
////        temp = (float)TIM_GetCapture1(HALL_TIM);

////        // 状态->扇区
////        hall_sector = hall_state_to_sector(hall_state);

////        // 计算方向
////        if(hall_sector != 0xFF && hall_sector_prev != 0xFF && hall_sector != hall_sector_prev)
////        {
////            if(hall_sector == (uint8_t)((hall_sector_prev + 1) % 6))      
////                hall_dir = +1;
////            else if(hall_sector == (uint8_t)((hall_sector_prev + 5) % 6)) 
////                hall_dir = -1;
////        }
////        hall_sector_prev = hall_sector;

////        // 计算速度和角度增量
////        hall_angle_add = hall_dir * ((float)HALL_ANGLE_FACTOR / temp);
////        hall_speed     = hall_dir * ((float)HALL_SPEED_FACTOR / temp);

////        // 扇区基准角
////        if(hall_sector != 0xFF)
////        {
////            hall_angle = (hall_sector * (PI/3.0f)) + PHASE_SHIFT_ANGLE;
////            if(hall_angle < 0.0f) hall_angle += 2.0f * PI;
////            else if(hall_angle > 2.0f*PI) hall_angle -= 2.0f * PI;
////        }
////        
////        hall_edge_cnt++;
////        hall_last_edge_ms = g_ms;

////        TIM_ClearFlag(HALL_TIM, TIM_FLAG_CC1);
////    }
////}
//void TIM2_IRQHandler(void)
//{
//    if (TIM_GetFlagStatus(HALL_TIM, TIM_FLAG_CC1) != SET)
//        return;

//    uint8_t new_state = 0;
//    new_state  = GPIO_ReadInputDataBit(HALL_CH3_GPIO_PORT, HALL_CH3_PIN);
//    new_state |= GPIO_ReadInputDataBit(HALL_CH2_GPIO_PORT, HALL_CH2_PIN) << 1;
//    new_state |= GPIO_ReadInputDataBit(HALL_CH1_GPIO_PORT, HALL_CH1_PIN) << 2;

//    if (new_state == hall_state || (g_ms - hall_debounce_ms) < HALL_DEBOUNCE_TIME)
//    {
//        TIM_ClearFlag(HALL_TIM, TIM_FLAG_CC1);
//        return;
//    }

//    hall_debounce_ms = g_ms;
//    hall_state = new_state;

//    // ✅ 保存扇区耗时
//    uint32_t cap = (uint32_t)TIM_GetCapture1(HALL_TIM);
//    if (cap > 0) hall_sector_dt_ticks = cap;

//    hall_sector = hall_state_to_sector(hall_state);

//    // 方向判定
//    if (hall_sector != 0xFF && hall_sector_prev != 0xFF && hall_sector != hall_sector_prev)
//    {
//        int8_t diff = (int8_t)hall_sector - (int8_t)hall_sector_prev;
//        if (diff == 1 || diff == -5)
//            hall_dir = +1;
//        else if (diff == -1 || diff == 5)
//            hall_dir = -1;
//    }
//    hall_sector_prev = hall_sector;

//    // 扇区基准角
//    if (hall_sector != 0xFF)
//    {
//        hall_angle = hall_sector * (PI / 3.0f) + PHASE_SHIFT_ANGLE;
//        if (hall_angle < 0.0f)       hall_angle += 2.0f * PI;
//        if (hall_angle > 2.0f * PI)  hall_angle -= 2.0f * PI;
//    }

//    // 速度（保留给其他地方用）
//    hall_speed = hall_dir * ((float)HALL_SPEED_FACTOR / (float)cap);

//    hall_edge_cnt++;
//    hall_last_edge_ms = g_ms;

//    TIM_ClearFlag(HALL_TIM, TIM_FLAG_CC1);
//}
/**********************************
           
**********************************/
#include "main.h"
#include "hall_sensor.h"
u8 hall_read_temp;

float hall_angle;
float hall_angle_add;
float hall_speed;
u8 hall_read_temp;
volatile uint8_t hall_state = 0;
volatile uint8_t hall_sector = 0xFF;
volatile uint8_t hall_sector_prev = 0xFF;
volatile int8_t  hall_dir = +1;         // +1 正向, -1 反向
volatile uint32_t hall_last_edge_ms = 0; // 记录最近一次霍尔边沿时间(毫秒)
// 你需要一个全局毫秒计数 g_ms（可在 SysTick 里 ++）
extern volatile uint32_t g_ms;
static uint8_t hall_state_last_valid = 0xFF;  // 上次有效状态
static uint32_t hall_debounce_ms = 0;         // 消抖计时
volatile uint32_t hall_sector_dt_ticks;  // 上一扇区耗时
#define HALL_DEBOUNCE_TIME  2                 // 消抖时间 2ms
void TIM2_IRQHandler(void)
{
  float temp;
  if(TIM_GetFlagStatus(HALL_TIM,TIM_FLAG_CC1)==SET)
  {
    temp = (float)(TIM_GetCapture1(HALL_TIM));
    hall_angle_add = (float)HALL_ANGLE_FACTOR/(float)(temp);
    hall_speed = (float)HALL_SPEED_FACTOR/(float)(temp);
    hall_read_temp = GPIO_ReadInputDataBit(HALL_CH3_GPIO_PORT,HALL_CH3_PIN);
    hall_read_temp |= GPIO_ReadInputDataBit(HALL_CH2_GPIO_PORT,HALL_CH2_PIN)<<1;
    hall_read_temp |= GPIO_ReadInputDataBit(HALL_CH1_GPIO_PORT,HALL_CH1_PIN)<<2;

    if(hall_read_temp==0x05)
    {
      hall_angle = 0.0f+PHASE_SHIFT_ANGLE;
    }
    else if(hall_read_temp==0x04)
    {
      hall_angle = (PI/3.0f)+PHASE_SHIFT_ANGLE;
    }
    else if(hall_read_temp==0x06)
    {
      hall_angle = (PI*2.0f/3.0f)+PHASE_SHIFT_ANGLE;
    }
    else if(hall_read_temp==0x02)
    {
      hall_angle = PI+PHASE_SHIFT_ANGLE;
    }
    else if(hall_read_temp==0x03)
    {
      hall_angle = (PI*4.0f/3.0f)+PHASE_SHIFT_ANGLE;
    }
    else if(hall_read_temp==0x01)
    {
      hall_angle = (PI*5.0f/3.0f)+PHASE_SHIFT_ANGLE;
    }
    if(hall_angle<0.0f)
    {
      hall_angle += 2.0f*PI;
    }
    else if(hall_angle>(2.0f*PI))
    {
      hall_angle -= 2.0f*PI;
    }
    
    TIM_ClearFlag(HALL_TIM,TIM_FLAG_CC1);
  }
  
}

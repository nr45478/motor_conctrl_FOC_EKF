/**********************************
 * 高精度位置传感器模块实现
 **********************************/
#include "position_sensor.h"
#include "hall_sensor.h"  // 使用霍尔传感器数据
#include "foc_algorithm.h"  // 访问EKF输出
extern volatile uint8_t hall_state;
// 全局变量
POSITION_SENSOR_DEF Position_Sensor;

// 插值更新周期（与PWM中断一致，假设10kHz）
#define INTERP_UPDATE_PERIOD 0.0001F  // 100us

/**
 * @brief  位置传感器初始化
 * @param  method: 位置获取方法
 * @retval None
 */
//void position_sensor_init(POSITION_METHOD method)
//{
//    Position_Sensor.method = method;
//    Position_Sensor.interpolated_angle = 0.0F;
//    Position_Sensor.last_hall_angle = hall_angle;
//    Position_Sensor.hall_changed_flag = 0;
//}
void position_sensor_init(POSITION_METHOD method)
{
    Position_Sensor.method = method;
    Position_Sensor.interpolated_angle = hall_angle; // 用当前扇区角初始化
    Position_Sensor.last_hall_angle = 0.0f;          // 这个字段你可以不用了
    Position_Sensor.hall_changed_flag = 0;
    Position_Sensor.last_hall_state = hall_state;    // 你需要在结构体里加这个字段
		Position_Sensor.elec_prev_wrap = Position_Sensor.interpolated_angle;
		Position_Sensor.elec_unwrap    = Position_Sensor.interpolated_angle;
		Position_Sensor.mech_unwrap    = Position_Sensor.elec_unwrap / 3.0;
}

/**
 * @brief  获取高精度位置（主函数）
 * @note   根据配置的方法返回位置
 * @retval 当前转子位置 (rad)，范围[0, 2π]
 */
real32_T get_high_precision_position(void)
{
    real32_T position = 0.0F;
    
    switch (POS_METHOD_HALL_INTERP) {
        case POS_METHOD_HALL_ONLY:
            // 方法1：直接使用霍尔角度（60°精度）
            position = hall_angle;
            break;
            
        case POS_METHOD_HALL_INTERP:
            // 方法2：霍尔+速度插值（推荐）
            position = Position_Sensor.interpolated_angle;
//				    position = hall_angle;
            break;
            
        case POS_METHOD_EKF:
            // 方法3：使用EKF估算的角度
            position = FOC_Output.EKF[3];  // xD[3]是转子角度
            break;
            
        default:
            position = hall_angle;
            break;
    }
    
    // 角度规范化到[0, 2π]
    while (position < 0.0F) {
        position += 2.0F * PI;
    }
    while (position >= 2.0F * PI) {
        position -= 2.0F * PI;
    }
    
    return position;
}

/**
 * @brief  位置传感器更新（在PWM中断中调用）
 * @note   实现速度积分插值
 * @retval None
 */
//void position_sensor_update(void)
//{
//    // 仅在插值模式下执行
//    if (Position_Sensor.method != POS_METHOD_HALL_INTERP) {
//        return;
//    }
//    
//    // 检测霍尔状态是否变化
//    if (hall_angle != Position_Sensor.last_hall_angle) {
//        // 霍尔跳变 → 校正插值角度
//        Position_Sensor.interpolated_angle = hall_angle;
//        Position_Sensor.last_hall_angle = hall_angle;
//        Position_Sensor.hall_changed_flag = 1;
//    } else {
//        // 霍尔未跳变 → 速度积分
//        // 使用当前转速累加角度
////        Position_Sensor.interpolated_angle += hall_speed * INTERP_UPDATE_PERIOD;
//			Position_Sensor.interpolated_angle += (hall_speed * 2.0f * PI) * INTERP_UPDATE_PERIOD;
//        
//        // 角度规范化
//        if (Position_Sensor.interpolated_angle >= 2.0F * PI) {
//            Position_Sensor.interpolated_angle -= 2.0F * PI;
//        } else if (Position_Sensor.interpolated_angle < 0.0F) {
//            Position_Sensor.interpolated_angle += 2.0F * PI;
//        }
//    }
//}
//void position_sensor_update(void)
//{
//    if(Position_Sensor.method != POS_METHOD_HALL_INTERP) return;

//    if(hall_state != Position_Sensor.last_hall_state)
//    {
//        Position_Sensor.interpolated_angle = hall_angle; // 用扇区基准角校正
//        Position_Sensor.last_hall_state = hall_state;
//    }
//    else
//    {
//        Position_Sensor.interpolated_angle += (hall_speed * 2.0f * PI) * INTERP_UPDATE_PERIOD;
//        if(Position_Sensor.interpolated_angle >= 2.0f*PI) Position_Sensor.interpolated_angle -= 2.0f*PI;
//        else if(Position_Sensor.interpolated_angle < 0.0f) Position_Sensor.interpolated_angle += 2.0f*PI;
//    }
//}
float wrapToPi(float x){
    while(x >  PI) x -= 2.0f*PI;
    while(x <= -PI) x += 2.0f*PI;
    return x;
}

float wrapTo2Pi(float x){
    while(x < 0.0f) x 	+= 2.0f*PI;
    while(x >= 2.0f*PI) x -= 2.0f*PI;
    return x;
}
#define POLE_PAIRS 3

//void position_sensor_update(void)
//{
//    if(Position_Sensor.method != POS_METHOD_HALL_INTERP) return;

//    // 霍尔跳变时校正
//    if(hall_state != Position_Sensor.last_hall_state)
//    {
////        Position_Sensor.interpolated_angle = hall_angle;
//				Position_Sensor.interpolated_angle += wrapToPi(hall_angle - Position_Sensor.interpolated_angle);
//				Position_Sensor.interpolated_angle = wrapTo2Pi(Position_Sensor.interpolated_angle);
//        Position_Sensor.last_hall_state = hall_state;
//    }
//    else
//    {
//        // 【重要】静止或低速时，不要积分，保持当前角度
//        // 否则会漂移
//        if(fabsf(hall_speed) > 1.0F)  // 阈值1 rad/s
//        {
//            Position_Sensor.interpolated_angle += hall_speed * 2.0f * PI * 0.0001F;
//            
//            if(Position_Sensor.interpolated_angle >= 2.0F*PI) 
//                Position_Sensor.interpolated_angle -= 2.0F*PI;
//            else if(Position_Sensor.interpolated_angle < 0.0F) 
//                Position_Sensor.interpolated_angle += 2.0F*PI;
//        }
//        // else: 静止时保持interpolated_angle不变
//    }
//		float elec_wrap = Position_Sensor.interpolated_angle; // 0..2π

//		Position_Sensor.elec_unwrap += wrapToPi(elec_wrap - Position_Sensor.elec_prev_wrap);
//		Position_Sensor.elec_prev_wrap = elec_wrap;

//		// 连续机械角
//		Position_Sensor.mech_unwrap = Position_Sensor.elec_unwrap / (float)POLE_PAIRS;
//}
#define SECTOR_RAD (PI / 3.0f)
extern volatile  uint32_t hall_last_edge_ms;
extern volatile  uint32_t g_ms;
extern volatile  int8_t  hall_dir;
extern volatile  uint32_t hall_sector_dt_ticks;
void position_sensor_update(void)
{
    if (Position_Sensor.method != POS_METHOD_HALL_INTERP) return;

    // 没超时才做相位插值
    if ((g_ms - hall_last_edge_ms) <= 50)
    {
        float phase = 0.0f;
        if (hall_sector_dt_ticks > 0)
        {
            uint32_t elapsed = TIM_GetCounter(HALL_TIM);
            phase = (float)elapsed / (float)hall_sector_dt_ticks;
            if (phase < 0.0f) phase = 0.0f;
            if (phase > 1.2f) phase = 1.2f;
        }

        Position_Sensor.interpolated_angle = wrapTo2Pi(hall_angle + (float)hall_dir * phase * SECTOR_RAD);
    }
    // else: 超时，保持 interpolated_angle 不变

    // 角度展开（始终执行）
    float elec_wrap = Position_Sensor.interpolated_angle;
    Position_Sensor.elec_unwrap += wrapToPi(elec_wrap - Position_Sensor.elec_prev_wrap);
    Position_Sensor.elec_prev_wrap = elec_wrap;
    Position_Sensor.mech_unwrap = Position_Sensor.elec_unwrap / (float)POLE_PAIRS;
}
/**********************************
 * OLED显示面板 
 **********************************/
#include "main.h"
#include <math.h>   // fabsf
#include "oled_display.h"
#include "drv8301.h"
#include "control_mode.h"
#include "position_pid.h"
#include "position_sensor.h"
// 原有宏定义保留
#ifndef PI
#define PI 3.1415926f
#endif
#define RAD2DEG (57.2957795f)
#define KEY_SWITCH_PAGE_TIME 200   // 长按2秒切换（10ms任务周期）

// 原有全局变量保留
u8 usb_open_display_flag;
u8 data_upload_display_flag;
u8 motor_run_display_flag;
u8 motor_run_display_flag_pre = 1;
u8 display_static_flag;
u8 init_dispaly_flag;
u8 display_index;
u8 display_index_key;
u8 display_flag = 0;
u8 clear_display_flag = 0;
u8 drv8301_fault_flag = 0;
uint16_t drv8301_reg_read1[4];

// 新增：四值显示专用标志
static CONTROL_MODE_ENUM last_display_mode = MODE_SPEED;
static u8 key_switch_page_flag = 0;    // 页面切换触发标志
static u32 key_switch_page_cnt = 0;    // 页面切换计时计数器
u8 KEY1_KEY2_COMBO_FLAG = 0;           // 组合键标志

// 外部变量声明（需在其他文件定义）
extern CONTROL_MODE_DEF Control_Mode;       // 控制模式
extern float Position_Ref;                  // 位置给定
extern float Position_Fdk;                  // 位置反馈
extern float Speed_Ref;                     // 速度给定（rad/s）
extern float Speed_Fdk;                     // 速度反馈（rad/s）
extern float hall_speed;                    // 霍尔速度（Hz）
extern float EKF_Hz;                        // EKF估算速度（Hz）
extern FOC_INPUT_DEF FOC_Input;             // FOC输入
extern float motor_direction;               // 电机方向
extern volatile uint32_t hall_edge_cnt;     // 霍尔边沿计数
// 按键状态
extern u8 key1_flag;                        // 按键1状态（0=未按，1=按下）
extern u8 key2_flag;                        // 按键2状态（0=未按，1=按下）

/**
 * @brief  【核心新增】四值显示页面（显示：霍尔Hz、EKFHz、位置偏差、Iq电流）
 */
void display_page_four_values(void)
{
    // 模式/页面切换时清屏重绘（避免残留）
    if(last_display_mode != Control_Mode.mode || clear_display_flag == 0)
    {
        OLED_Clear();
        last_display_mode = Control_Mode.mode;
        clear_display_flag = 1;
        
        // 绘制静态文本（仅切换时绘制一次，提升刷新率）
        OLED_ShowString(0, 0, "4-Values");          // 页面标题
        OLED_ShowString(0, 1, "h:");                // 霍尔速度(Hz)
        OLED_ShowString(0, 2, "e:");                // EKF速度(Hz)
        OLED_ShowString(0, 3, "Err:");              // 位置偏差(rad)
        OLED_ShowString(0, 4, "Iq: .");             // Q轴电流(A)
        OLED_ShowString(11*8, 0, Control_Mode.mode == MODE_POSITION ? "POS" : "SPD"); // 当前模式
    }

    // 显示电机运行状态（RUN/STOP）
    if(motor_run_display_flag_pre != motor_run_display_flag)
    {
        OLED_ShowString(8*8, 0, motor_run_display_flag ? "RUN " : "STOP");
        motor_run_display_flag_pre = motor_run_display_flag;
    }

    // ---------- 1. 霍尔速度（h: Hz，绝对值显示） ----------
    float h_hz = fabsf(hall_speed);
    h_hz = (h_hz > 9999) ? 9999 : h_hz; // 限制范围避免乱码
    OLED_ShowNum(2*8, 1, (u32)h_hz, 4, 16);
    OLED_ShowString(6*8, 1, "Hz");

    // ---------- 2. EKF速度（e: Hz，绝对值显示） ----------
    float e_hz = fabsf(EKF_Hz);
    e_hz = (e_hz > 9999) ? 9999 : e_hz;
    OLED_ShowNum(2*8, 2, (u32)e_hz, 4, 16);
    OLED_ShowString(6*8, 2, "Hz");

    // ---------- 3. 位置偏差（Err: rad，带正负） ----------
    float pos_err = Position_Ref - Position_Fdk;
    // 限制显示范围（±99.99rad）
    pos_err = (pos_err > 99.99) ? 99.99 : (pos_err < -99.99 ? -99.99 : pos_err);
    
    if(pos_err >= 0)
    {
        OLED_ShowString(4*8, 3, "+");
        OLED_ShowNum(5*8, 3, (u32)pos_err, 2, 8);                  // 整数位
        OLED_ShowString(7*8, 3, ".");                               // 小数点
        OLED_ShowNum(8*8, 3, (u32)(pos_err * 100) % 100, 2, 8);    // 小数位
    }
    else
    {
        float err_abs = -pos_err;
        OLED_ShowString(4*8, 3, "-");
        OLED_ShowNum(5*8, 3, (u32)err_abs, 2, 8);
        OLED_ShowString(7*8, 3, ".");
        OLED_ShowNum(8*8, 3, (u32)(err_abs * 100) % 100, 2, 8);
    }
    OLED_ShowString(10*8, 3, "rad");

    // ---------- 4. Q轴电流（Iq: A，绝对值显示） ----------
    float iq_abs = fabsf(FOC_Input.Iq_ref);
    iq_abs = (iq_abs > 9.99) ? 9.99 : iq_abs; // 限制0-9.99A
    OLED_ShowNum(4*8, 4, (u32)iq_abs, 1, 16);                 // 整数位
    OLED_ShowNum(6*8, 4, (u32)(iq_abs * 100) % 100, 2, 8);    // 小数位
    OLED_ShowString(8*8, 4, "A");
}

/**
 * @brief  Logo页面
 */
void display_page_logo(void)
{
    OLED_DrawBMP(0, 0, 128, 8, Logo);
}

/**
 * @brief  速度模式页面
 */
void display_page_speed_mode(void)
{
    if(last_display_mode != MODE_SPEED)
    {
        OLED_Clear();
        last_display_mode = MODE_SPEED;
        OLED_ShowString(0, 0, "Mode:SPEED");
        OLED_ShowString(0, 2, "Spd:");
        OLED_ShowString(0, 4, "Fdk:");
        OLED_ShowString(0, 6, "Iq: .");
    }

    if(motor_run_display_flag_pre != motor_run_display_flag)
    {
        OLED_ShowString(11*8, 0, motor_run_display_flag ? "RUN " : "STOP");
        motor_run_display_flag_pre = motor_run_display_flag;
    }

    // 速度给定（RPM）
    float speed_ref_rpm = Speed_Ref * 60.0f / 6.28318548f;
    speed_ref_rpm = (speed_ref_rpm > 9999) ? 9999 : (speed_ref_rpm < -9999 ? -9999 : speed_ref_rpm);
    OLED_ShowString(5*8, 2, speed_ref_rpm >= 0 ? "+" : "-");
    OLED_ShowNum(6*8, 2, (u32)fabsf(speed_ref_rpm), 4, 16);
    OLED_ShowString(10*8, 2, "rpm");

    // 速度反馈（RPM）
    float speed_fdk_rpm = Speed_Fdk * 60.0f / 6.28318548f;
    speed_fdk_rpm = (speed_fdk_rpm > 9999) ? 9999 : (speed_fdk_rpm < -9999 ? -9999 : speed_fdk_rpm);
    OLED_ShowString(5*8, 4, speed_fdk_rpm >= 0 ? "+" : "-");
    OLED_ShowNum(6*8, 4, (u32)fabsf(speed_fdk_rpm), 4, 16);
    OLED_ShowString(10*8, 4, "rpm");

    // Q轴电流
    float iq_abs = fabsf(FOC_Input.Iq_ref);
    iq_abs = (iq_abs > 9.99) ? 9.99 : iq_abs;
    OLED_ShowNum(4*8, 6, (u32)iq_abs, 1, 16);
    OLED_ShowNum(6*8, 6, (u32)(iq_abs * 100) % 100, 2, 8);
    OLED_ShowString(8*8, 6, "A");
}

/**
 * @brief  位置模式页面
 */
extern real32_T error;
extern real32_T Position_Pid_Out;
void display_page_position_mode(void)
{
    if(last_display_mode != MODE_POSITION)
    {
        OLED_Clear();
        last_display_mode = MODE_POSITION;
        // 静态标签
        OLED_ShowString(0, 0, "F:");//1.16
        OLED_ShowString(64, 0, "O:");//2.62
        OLED_ShowString(0, 2, "Err:");//0.52
        OLED_ShowString(0, 4, "hall:");//1.05
        OLED_ShowString(0, 6, "intp:");//1.16
    }

    if(motor_run_display_flag_pre != motor_run_display_flag)
    {
        OLED_ShowString(112, 0, motor_run_display_flag ? "R" : "S");
        motor_run_display_flag_pre = motor_run_display_flag;
    }

    // 先清空区域再写，用空格覆盖
    // Position_Fdk: 第0行，从16像素开始，占6个字符宽度
    OLED_ShowString(16, 0, "      ");  // 6个空格清旧值
    OLED_ShowFloat(16, 0, Position_Fdk, 2);  // 只显示2位小数，如"3.14"
    
    // Position_Pid_Out: 第0行，从80像素开始
    OLED_ShowString(80, 0, "      ");  // 6个空格
    OLED_ShowFloat(80, 0, Position_Pid_Out, 2);
    
    // error
    OLED_ShowString(40, 2, "      ");
    OLED_ShowFloat(40, 2, hall_speed, 2);
    
    // hall_angle
    OLED_ShowString(48, 4, "      ");
    OLED_ShowFloat(48, 4, hall_angle, 2);
    
    // interpolated_angle
    OLED_ShowString(48, 6, "      ");
    OLED_ShowFloat(48, 6, Position_Sensor.interpolated_angle, 2);
}

/**
 * @brief  调试页面
 */
void display_page_debug(void)
{
//    if(clear_display_flag == 0)
//    {
//        OLED_Clear();
//        OLED_ShowString(0, 2, "hall_f:");
//        OLED_ShowString(0, 4, "ekf_f:");
//        OLED_ShowString(0, 6, "Iq_ref: .");
//        clear_display_flag = 1;
//    }

//    if(motor_run_display_flag_pre != motor_run_display_flag)
//    {
//        OLED_ShowString(0, 0, motor_run_display_flag ? "Motor:run " : "Motor:stop");
//        motor_run_display_flag_pre = motor_run_display_flag;
//    }

//    hall_speed = (hall_speed > 999) ? 999 : (hall_speed < 0 ? 0 : hall_speed);
//    EKF_Hz = (EKF_Hz > 999) ? 999 : (EKF_Hz < 0 ? 0 : EKF_Hz);
//    OLED_ShowNum(7*8, 2, (u32)hall_speed, 3, 16);
//    OLED_ShowNum(7*8, 4, (u32)EKF_Hz, 3, 16);
//    OLED_ShowNum(7*8, 6, (u32)FOC_Input.Iq_ref, 1, 16);
//    OLED_ShowNum(9*8, 6, (u32)(FOC_Input.Iq_ref * 100), 2, 8);
OLED_ShowString(0, 0, "R:");  // Ref
OLED_ShowFloat(16, 0, Position_Ref, 2);
OLED_ShowString(0, 4, "F:");  // Fdk  
OLED_ShowFloat(16, 4, Position_Fdk, 2);
}

/**
 * @brief  原有故障页面（保留）
 */
void display_page_fault(void)
{
    OLED_DrawBMP(0, 0, 128, 8, fault);
}

/**
 * @brief  按键切换检测（保留原有逻辑）
 */
void key_switch_page_detect(void)
{
    if(key1_flag == 1 && key2_flag == 1)
    {
        KEY1_KEY2_COMBO_FLAG = 1;
        key_switch_page_cnt++;

        if(key_switch_page_cnt >= KEY_SWITCH_PAGE_TIME)
        {
            key_switch_page_flag = 1;
            oled_switch_page((display_flag + 1) % 5); // 新增四值页面后，页面数改为5
            key_switch_page_cnt = 0;
            KEY1_KEY2_COMBO_FLAG = 0;
        }
    }
    else
    {
        KEY1_KEY2_COMBO_FLAG = 0;
        key_switch_page_cnt = 0;
        key_switch_page_flag = 0;
    }
}

/**
 * @brief  OLED主处理函数（新增四值页面分支）
 */
void oled_display_handle(void)
{
    key_switch_page_detect();

    // 故障优先显示
    if(drv8301_fault_flag != 0)
    {
        display_page_fault();
        return;
    }

    // 页面切换逻辑（新增4号页面：四值显示）
    switch(display_flag)
    {
        case 0:  // Logo页面
            display_page_logo();
            clear_display_flag = 0;
            break;
            
        case 1:  // 调试信息页面
            display_page_debug();
            break;
            
        case 2:  // 原始页面
            oled_display();
            break;
            
        case 3:  // 自动模式页面（速度/位置）
            display_page_position_mode(); // 复用原有位置模式页面（兼容速度模式）
            break;
            
        case 4:  // 【新增】四值显示页面
            display_page_four_values();
            break;
            
        default:
            display_page_logo();
            break;
    }
}

/**
 * @brief  页面切换函数（适配新增页面）
 */
void oled_switch_page(u8 page)
{
    if(page != display_flag)
    {
        display_flag = page;
        clear_display_flag = 0;
        last_display_mode = MODE_SPEED;
        OLED_Clear();
    }
}
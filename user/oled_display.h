#ifndef __OLED_DISPLAY_H_
#define __OLED_DISPLAY_H_

/******************************************
 * 头文件包含（确保依赖的类型/宏能正确识别）
 ******************************************/
#include "stdint.h"  // 标准整数类型（u8/u32等）

/******************************************
 * 宏定义（页面编号枚举，替代魔法数字）
 ******************************************/
// OLED显示页面编号（与oled_switch_page函数配套使用）
typedef enum
{
    OLED_PAGE_LOGO        = 0,    // Logo开机页面
    OLED_PAGE_DEBUG       = 1,    // 调试信息页面（hall_f/ekf_f/Iq_ref）
    OLED_PAGE_ORIGINAL    = 2,    // 原始oled_display()页面
    OLED_PAGE_AUTO_MODE   = 3 ,    // 自动模式页面（速度/位置模式自动切换）
	 OLED_PAGE_AUTO_MODE2    = 4 
} OLED_PAGE_ENUM;

/******************************************
 * 全局变量声明（extern）
 * 说明：仅声明需要跨文件访问的变量，私有变量不对外暴露
 ******************************************/
// USB/数据上传显示标志
extern uint8_t usb_open_display_flag;
extern uint8_t data_upload_display_flag;

// 电机运行状态显示标志
extern uint8_t motor_run_display_flag;

// 显示控制相关标志
extern uint8_t display_static_flag;    // 静态显示标志（是否刷新静态文本）
extern uint8_t init_dispaly_flag;      // 初始化显示标志
extern uint8_t display_index_key;      // 按键切换显示索引
extern uint8_t display_cnt;            // 显示计数（防抖/延时）
extern uint8_t display_flag;           // 当前显示页面（对应OLED_PAGE_ENUM）
extern uint8_t drv8301_fault_flag;     // DRV8301故障显示标志（0=无故障，非0=故障）

/******************************************
 * 函数声明（按功能分类，补充详细注释）
 ******************************************/

/**
 * @brief  OLED显示主处理函数
 * @note   需在主循环/显示任务中周期调用（建议10~50ms）
 * @param  无
 * @retval 无
 */
extern void oled_display_handle(void);

/**
 * @brief  切换OLED显示页面
 * @note   可由按键/外部事件触发调用
 * @param  page: 目标页面编号（参考OLED_PAGE_ENUM枚举）
 * @retval 无
 */
void oled_switch_page(uint8_t page);

/**
 * @brief  显示调试信息页面
 * @note   显示hall_f/ekf_f/Iq_ref等调试参数
 * @param  无
 * @retval 无
 */
void display_page_debug(void);

/**
 * @brief  显示位置模式页面
 * @note   显示位置给定/反馈、速度反馈、q轴电流等参数
 * @param  无
 * @retval 无
 */
void display_page_position_mode(void);

/**
 * @brief  显示速度模式页面
 * @note   显示速度给定/反馈、q轴电流等参数
 * @param  无
 * @retval 无
 */
void display_page_speed_mode(void);

/**
 * @brief  显示Logo开机页面
 * @note   显示开机位图/Logo
 * @param  无
 * @retval 无
 */
void display_page_logo(void);
void display_page_four_values(void);
#endif /* __OLED_DISPLAY_H_ */



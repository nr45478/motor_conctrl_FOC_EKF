#ifndef __LOW_TASK_H_
#define __LOW_TASK_H_









extern uint8_t motor_start_stop;
extern uint8_t key1_press_flag;
void motor_start(void);
void motor_stop(void);
void hall_angle_update_loop(void);
void low_control_task(void);
#endif

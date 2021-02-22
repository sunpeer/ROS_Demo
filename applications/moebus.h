#ifndef __MOEBUS_H__
#define __MOEBUS_H__

#include <rtthread.h>
#include <board.h>
#include "math_def.h"

#ifdef __cplusplus
extern "C"{
#endif


//根据面包板的那个版本的接线图来确定以下的对应关系

#define LF_IN1 GET_PIN(D,2)
#define LF_IN2 GET_PIN(C,12)
#define LF_PWM GET_PIN(C,9)
#define LF_CHANNEL 4

#define RF_IN1 GET_PIN(B,5)  
#define RF_IN2 GET_PIN(B,4)  
#define RF_PWM GET_PIN(C,8)
#define RF_CHANNEL 3 

#define LR_IN1 GET_PIN(C,5)  
#define LR_IN2 GET_PIN(C,4)  
#define LR_PWM GET_PIN(C,7)
#define LR_CHANNEL 2

#define RR_IN1 GET_PIN(B,1)  
#define RR_IN2 GET_PIN(B,0)  
#define RR_PWM GET_PIN(C,6)
#define RR_CHANNEL 1


#define PI                  3.1415926f

#define WHEEL_DIAMETER 		0.080			//wheel's diameter in meters
#define a_PARAMETER         104.0f    /*(0.054f)  mm*/
#define b_PARAMETER         82.5f   /*(0.0818f)   mm*/
#define SDRM_PARAM          0.75/PI    /*(300.25f)      动向速度转换比率 */
#define BASE_WIDTH 			a_PARAMETER+b_PARAMETER

#define PWM_CYCLE(Hz)       (1000000000UL / Hz)                //PWM周期
#define PWM_PERIOD          PWM_CYCLE(80000)


#define MAX_RPM 			100	//轮子的转速
#define PWM_MAX_PER         11250                 	//占空比最大值90%
#define PWM_MIN_PER         625                     //占空比最小值5%

#define PWM_NAME "pwm8"
#define PULSE_PER_ROTATE 1560
#define Kp 20
#define Ki 10
#define MAX_PWM 11250

typedef struct velocities
{
    float linear_x;
    float linear_y;
    float angular_z;
}velocities_t;


void Motor_Init(void);
void set_spd(float x_vel, float y_vel, float z_rota);
void update_car(void);
velocities_t get_velocities();

#ifdef __cplusplus
}
#endif

#endif
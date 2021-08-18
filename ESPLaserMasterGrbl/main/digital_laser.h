/*
 * digital_laser.h
 *
 *  Created on: 2021年7月19日
 *      Author: c
 */

#ifndef MAIN_DIGITAL_LASER_H_
#define MAIN_DIGITAL_LASER_H_

#include <ctype.h>
#include "board.h"

typedef enum{
  COMM_TICK_ALIVE                   =0,
  COMM_AIM_CONTROL                    ,
  COMM_FOCUS_DISTANCE                 ,
  COMM_GET_TEMPRATURE                 ,
  COMM_FAN_FRE_CONTROL                ,
  COMM_FAN_DUTY_CONTROL               ,
  COMM_LASER_PWM_FREQUENT             ,
  COMM_LASER_PWM_DUTY                 ,
  COMM_FIRE_DETECT                    ,
  COMM_GET_LASER_AGE                  ,
  COMM_GET_MCU_SN                     ,
  COMM_GET_LASER_VENDOR               ,
  COMM_GET_PRODUCT_DATE               ,
  COMM_GET_HARDWARE_VERSION           ,
  COMM_GET_SOFTWARE_VERSION           ,
}regNum;


void laser_init(void);
uint8_t laser_write(uint8_t* data, uint16_t len);
uint8_t laser_read(uint8_t* data, uint16_t len);
void digital_laser_test(void);
uint8_t laser_set_value(regNum reg_num, uint32_t value);
uint32_t laser_get_value(regNum reg_num);
//void laser_auto_focus(void* arg);
void laser_auto_focus_task_create(void);
void laser_pwm_duty_enqueue(int value);
void laser_enter_isr(void);
void laser_exit_isr(void);
void laser_task_create(void);
#endif /* MAIN_DIGITAL_LASER_H_ */

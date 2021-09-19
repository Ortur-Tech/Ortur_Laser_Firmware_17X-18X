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

#define LASER_WRITE 1
#define LASER_READ 0

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
  COMM_GET_BTN_STATE				  ,
  COMM_GET_VL6180_CONNECTED			  ,
  COMM_GET_NTC_CONNECTED			  ,
  COMM_GET_LASER_STATE				  ,
  COMM_LASER_PWM_DUTY_255			  ,
  COMM_LASER_PWM_DUTY_511			  ,
  COMM_LASER_PWM_DUTY_767			  ,
  COMM_LASER_PWM_DUTY_1000			  ,
}LaserRegNum;


typedef struct {
	uint32_t laser_duty;
	uint32_t laser_fre;
	uint32_t laser_precision;
	uint32_t distance;
	uint32_t temperture;
	uint32_t fan_fre;
	uint32_t fan_duty;
	uint32_t fire;
	uint32_t key;
	uint32_t is_vl6180_connected;
}LaserInfo;

extern LaserInfo laser_info;

void laser_init(void);
uint8_t laser_write(uint8_t* data, uint16_t len);
uint8_t laser_read(uint8_t* data, uint16_t len);
void digital_laser_test(void);
uint8_t laser_set_value(LaserRegNum reg_num, uint32_t value);
uint32_t laser_get_value(LaserRegNum reg_num);
//void laser_auto_focus(void* arg);
void laser_auto_focus_cycle(void);
void laser_auto_focus_set(uint8_t flag);
void laser_pwm_duty_enqueue(int value);
void laser_read_write_enqueue(uint8_t rw_flag, LaserRegNum reg, int value);
uint8_t laser_probe_state(void);

void laser_enter_isr(void);
void laser_exit_isr(void);
void laser_task_create(void);
void laser_keep_active(void);

#endif /* MAIN_DIGITAL_LASER_H_ */

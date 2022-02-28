/*
 * accelDetection.h
 *
 *  Created on: 2021��5��20��
 *      Author: c
 */
#include "board.h"
#include "driver.h"

#ifndef MAIN_ACCELDETECTION_H_
#define MAIN_ACCELDETECTION_H_


#define USE_GSENSOR_FIFO_MODE 1

#define SC7A20_ADDR	  0X18 //0x30

#define SC7A20_DEVICE 0X11
#define OTHER_DEVICE  0X03
#define BMA250_DEVICE 0X03
#define BMA253_DEVICE 0XFA

void Gsensor_Init();
void accel_detection();
void accel_detection_limit();
void gsensor_info_report(void);
esp_err_t i2c_master_init(void);

#endif /* MAIN_ACCELDETECTION_H_ */

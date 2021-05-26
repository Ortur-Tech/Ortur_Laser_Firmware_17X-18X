/*
 * accelDetection.h
 *
 *  Created on: 2021Äê5ÔÂ20ÈÕ
 *      Author: c
 */

#ifndef MAIN_ACCELDETECTION_H_
#define MAIN_ACCELDETECTION_H_

#define SC7A20_ADDR	  0X18 //0x30
#define SC7A20_DEVICE 0X11
#define OTHER_DEVICE  0X03

void Gsensor_Init();
void accel_detection();
void accel_detection_limit();

#endif /* MAIN_ACCELDETECTION_H_ */

/*
 * digital_laser.h
 *
 *  Created on: 2021年7月19日
 *      Author: c
 */

#ifndef MAIN_DIGITAL_LASER_H_
#define MAIN_DIGITAL_LASER_H_

#include "board.h"

void laser_init(void);
uint8_t laser_write(uint8_t* data, uint16_t len);
uint8_t laser_read(uint8_t* data, uint16_t len);
void digital_laser_test(void);

#endif /* MAIN_DIGITAL_LASER_H_ */

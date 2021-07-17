/*
 * serial_iap.h
 *
 *  Created on: 2021年1月22日
 *      Author: c
 */

#ifndef SERIAL_IAP_H_
#define SERIAL_IAP_H_

#include "board.h"

#define ORTUR_LASER_MODE
#define USE_SERIAL_IAP  1

#define ENTER_APP    	1
#define ENTER_MSC_IAP	2

#define MAX_BUF_SIZE 1024*4

#define HEADER_LOW  0X7a
#define HEADER_HIGH 0x55

#define PROTOCOL_VERSION 0X01

#define HEADER_POS 0
#define HEADER_LEN 2
#define COMM_NUM_POS 2
#define COMM_NUM_LEN 2
#define CMD_POS  4
#define DATA_LEN_POS 5
#define DATA_LEN_LEN 2
#define DATA_POS 7


#if defined(ORTUR_LASER_MODE)
#define GFU_CMD_IDENT       "ORTULASE" // [ORTU]R [LASE]R   通讯身份标识
#elif defined(ORTUR_CNC_MODE)
#define GFU_CMD_IDENT       "ORTUAUFE" // [ORTU]R [AUFE]RO  通讯身份标识
#endif

uint8_t serial_DataHandle(void);

void rec_TimeCheck(void);

void rec_SerialData(uint8_t data);

void serial_DataInit(void);
void serial_Iap(void);
void serial_set(uint8_t flag);




#endif /* SERIAL_IAP_H_ */

/*
 * board.h
 *
 *  Created on: 2021年5月20日
 *      Author: c
 */

#ifndef MAIN_BOARD_H_
#define MAIN_BOARD_H_

#include "stdio.h"
#include "config.h"


/*******************硬件版本定义 BEGIN*******************/
#define BOARD_UNKOWN 		0
#define OLM_ESP_V1X			1


#define BOARD_VERSION 		OLM_ESP_V1X
/*******************硬件版本定义 END*******************/


/*******************机型定义 BEGIN*******************/
#define BOARD_UNKOWN 		0
#define CNC_AUFERO			1
#define OLM_2_PRO			2
#define OLM_PRO				3



#define MACHINE_TYPE 		OLM_2_PRO
/*******************机型定义 END*******************/



#define DEBUG_LEVEL 0x00//LOG_INFO//LOG_ERROR
/*调试等级*/
typedef enum{
    LOG_DEBUG=1,
    LOG_INFO=2,
    LOG_WARN=4,
    LOG_ERROR=8,
    LOG_FATAL=16,
	LOG_IMP_INFO=32,
}LogLevel;


#define mprintf(level,format,...)  ((level&DEBUG_LEVEL)?printf(format,##__VA_ARGS__):((void)0U))

typedef enum{
 SYSTEM_WAIT_FOR_POWERON = 0,
 SYSTEM_KEY_PRESS_FOR_POWERON = 1,
 SYSTEM_POWERON = 2,
 SYSTEM_KEY_PRESS_FOR_POWEROFF = 3,
 SYSTEM_POWEROFF = 4,
}system_status;


#if ENABLE_POWER_SUPPLY_CHECK
#define get_SystemPowerStatus() (gpio_get_level(POWER_CHECK_PIN) ? 1:0) //PA13
#else
#define get_SystemPowerStatus() (1)
#endif
uint8_t get_PowerKeyStatus(void);
void soft_delay_us(uint32_t nus);
void soft_delay_ms(uint32_t nms);
void Check_Rst_Source(void);

void poweron_CmdSet(uint8_t cmd);
void key_func(uint8_t m);
void HAL_Delay(uint32_t ms);
uint32_t HAL_GetTick(void);
void creat_ExtFuncTask(void );

#endif /* MAIN_BOARD_H_ */

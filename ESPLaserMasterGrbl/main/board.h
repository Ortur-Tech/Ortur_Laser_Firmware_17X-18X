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
/************************硬件版本对应的机型*********************************
// *    BOARD_VERSION                    MACHINE_TYPE                         软件版本
// *  OLM_ESP_PRO_V1X  --- AUFERO1---AUFERO2---OLM2S2---OLM2_PRO_S2             V18X
// *                                             ||
// *  S2_MAX_V10                               OLM2(S0)                         V18X
// *
// *  OLM_ESP_V1X      --- OLM2_PRO_S1（S1机型）                                 V17X
 ****************************************************************************/


/*******************硬件版本定义 BEGIN*******************/
#define BOARD_UNKOWN 		0
#define OLM_ESP_V1X			1	//曾老板的主板
#define OLM_ESP_PRO_V1X     2   //我们自己的主板
#define OCM_ESP_PRO_V1X     3   //我们自己的主板


/*******************硬件版本定义 END*******************/


/*******************机型定义 BEGIN*******************/
// 三段型号命名 , 主版本xx , 次版本xx , 小修订xx
// Ortur 10
#define MACHINE_UNKOWN 		0
#define OLM	     			101000
#define OLM_PRO				100100
#define OLM2				100200   // OLM2 S0 方框机400*430
#define OLM2_S2             100202
#define OLM2_PRO		    102000
#define OLM2_PRO_S1		    102001
#define OLM2_PRO_S2		    102002
#define OLM3	     		100300

// Aufero 20
#define AUFERO_CNC			200000
#define AUFERO_1			200100	//悬臂机
#define AUFERO_2			200200	//方框机器390*390
#define AUFERO_3			200300
#define AUFERO_4			200400	//方框机器370*400


#define MACHINE_TYPE 	    AUFERO_1
/*******************机型定义 END*******************/

#if (MACHINE_TYPE == OLM2)
#define ORTUR_HW_NAME 		"S2_MAX_V1.0"
#define BOARD_VERSION 		 OLM_ESP_PRO_V1X
#elif (MACHINE_TYPE == OLM2_S2) || (MACHINE_TYPE == OLM2_PRO_S2) || (MACHINE_TYPE == AUFERO_1) || (MACHINE_TYPE == AUFERO_2)
#define ORTUR_HW_NAME 		"OLM_ESP_PRO_V1.2"
#define BOARD_VERSION 		 OLM_ESP_PRO_V1X
#elif (MACHINE_TYPE == OLM2_PRO_S1)
#define ORTUR_HW_NAME 		"OLM_ESP_V1X"
#define BOARD_VERSION 		 OLM_ESP_V1X
#elif
#error "unknown machine type!!!"
#endif

#define DEBUG_LEVEL LOG_ERROR
/*调试等级*/
typedef enum{
    LOG_DEBUG = 1,
    LOG_INFO = 2,
    LOG_WARN = 4,
    LOG_ERROR = 8,
    LOG_FATAL = 16,
	LOG_IMP_INFO = 32,
	LOG_TEMP = 64,
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
void HAL_TickInit(void);
uint32_t HAL_GetTick(void);
void creat_ExtFuncTask(void );

void Usb_ForceReset(void);
void disable_rom_code_console(void);

#endif /* MAIN_BOARD_H_ */

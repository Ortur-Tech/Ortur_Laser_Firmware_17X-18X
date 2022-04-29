/*
 * wireless_powerjack.h
 *
 *  Created on: 2022年4月22日
 *      Author: leadiffer
 */

#ifndef MAIN_EXTERN_COMPONENTS_WIRELESS_POWERSTRIP_H_
#define MAIN_EXTERN_COMPONENTS_WIRELESS_POWERSTRIP_H_
#include "stdint.h"
#include "esp_err.h"

#ifdef WIRELESS_STRIP_ENABLE
#define WIRELESS_STRIP_ADDR_DEFAULT		0x11111
#define WIRELESS_STRIP_TYPE_DEFAULT		0x0
#define STRIP_RANDOM_ADDR_DEFAULT		0

#define WIRELESS_STRIP_PIN			15				//light pin =gpio15
#define	TRUE_H_DELAY_US				750
#define TRUE_L_DELAY_US				250
#define	FALSE_H_DELAY_US			250
#define FALSE_L_DELAY_US			750

#define TX_BUFFER_MAX_LEN			50
#define SEND_REPEAT_NUM				7
#define TX_PERIOD_INVERT_MS			24
#define TX_WAIT_DONE_MS				100
#define PAIRING_REPEAT_TIMES		6

typedef struct
{
	uint32_t counter_clk_hz;
	uint8_t tx_buff[TX_BUFFER_MAX_LEN];
	uint8_t buff_len;
	uint16_t	h_1_ticks;
	uint16_t	l_1_ticks;
	uint16_t	h_0_ticks;
	uint16_t	l_0_ticks;
	uint32_t	addr;

}rmt_private_data_t;

typedef struct {
	uint32_t addr;
	uint8_t type;
}strip_settings_t;

typedef struct{
	uint8_t on;
	uint8_t off;
}strip_code_t;

typedef enum{
	KEY_NUM_1 = 1,
	KEY_NUM_2,
	KEY_NUM_3,
	KEY_NUM_4,
	KEY_NUM_5,
	KEY_NUM_6,
	KEY_NUM_7,
	KEY_NUM_8,
	KEY_NUM_9,
}key_num_n;

typedef enum{
	STRIP_CMD_M20	= 20,	//key1 on
	STRIP_CMD_M21,			//key1 off
	STRIP_CMD_M22,			//key2 on
	STRIP_CMD_M23,			//key2 off
	STRIP_CMD_M24,			//key3 on
	STRIP_CMD_M25,			//key3 off
	STRIP_CMD_M26,			//key4 on
	STRIP_CMD_M27,			//key4 off
	STRIP_CMD_M28,			//pairing
	STRIP_CMD_DIY_CMD,	//自定义数据
}cmd_code_n;

typedef struct{
	uint16_t cmd;
	uint8_t key;
}cmd_flag_t;


extern rmt_private_data_t	rmt_data;
extern cmd_flag_t	cmd_flag;

void wireless_strip_init(void);
void send_cmd(uint32_t addr, uint8_t key);
esp_err_t strip_fill_buffer_send(uint8_t *buff, uint8_t len, uint32_t timeout_ms);
void key_control(uint8_t key_num, uint8_t value);
void control_pairing(void);
void execute_cmd_task(void);

#endif

#endif /* MAIN_EXTERN_COMPONENTS_WIRELESS_POWERSTRIP_H_ */

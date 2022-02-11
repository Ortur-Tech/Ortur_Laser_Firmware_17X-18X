#ifndef _I2C_IAP_H_
#define _I2C_IAP_H_

#include "driver.h"

/****I2C IAP 流程**/
#define  I2C_IAP_CONFIRM_INFO  0X00
#define  I2C_IAP_RST           0X01
#define  I2C_IAP_START         0X02
#define  I2C_IAP_SET           0X03
#define  I2C_IAP_SEND          0X04
#define  I2C_IAP_EXE           0X05

/****命令类型**/
#define  I2C_IAP_REQ_CMD 		  0X03
#define  I2C_IAP_SET_CMD  		  0X04
#define  I2C_IAP_SENDDATA_CMD 	  0X05
#define  I2C_IAP_UPDATE_CMD       0X06

typedef enum{
	iap_nexts_step = 0,
	iap_exe_success ,
	iap_exe_failed,
	iap_current_step = 0xff
}iap_steps;

typedef enum{
	i2c_iap_req = 0,
	i2c_iap_set,
	i2c_iap_send,
	i2c_iap_exe
}iap_cmd_num;


typedef enum{
	wmcu = 1,
	w_ext_spiflash,
	w_ext_eeprom
}file_type;

typedef enum{
	rstdev= 1,
	enterUdisk,
	jumpapp
}update_cmd;

typedef struct{
	uint32_t filelen;
	file_type ftype;
	uint32_t  crc32;
	uint32_t  write_addr;
}file_info;

typedef struct {
	char reqdata[8];
	file_info finfo;
	uint8_t subdata[512];
	update_cmd updatecmd;
}iap_data_send;



typedef enum{
	ok = 0,
}update_resp_status;

typedef enum{
	revok = 0,
	resend,
	writedefeated,
	firmwareRevSuccess
   }data_resp_status;
typedef struct{
  data_resp_status status;
  uint16_t  cnt;
}data_resp;

typedef enum{
	setok = 0,
	unsupport
}set_resp_status;

/*命令响应结构体*/
typedef struct {
	uint8_t cmdlen;
	uint8_t datlen;
	char reqdata[8];
	set_resp_status setstatus;
	data_resp   datastatus;
	update_resp_status updatestatus;
}i2c_iap_resp;

/*升级包数据结构体*/
typedef struct {
	uint8_t  cmdid;
	uint16_t datlen;
	iap_data_send datasend;
	uint16_t crc16;
}i2c_iap_msg;



iap_steps i2c_iap();
#endif

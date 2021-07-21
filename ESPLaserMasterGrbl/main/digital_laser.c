/*
 * digital_laser.c
 *
 *  Created on: 2021年7月19日
 *      Author: c
 */

#include "digital_laser.h"
#include "driver/i2c.h"
#include <string.h>

#define LASER_IIC_ADDR 0X21
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


typedef struct {
	uint8_t reg;
	uint8_t len;
}reg_des;

reg_des regs[20] = {0};

uint8_t laser_init_flag = 0;


void laser_init(void)
{
	regs[COMM_TICK_ALIVE 		   ].reg = 0XC0; regs[COMM_TICK_ALIVE 			].len = 1;
	regs[COMM_AIM_CONTROL 		   ].reg = 0XC1; regs[COMM_AIM_CONTROL 			].len = 1;
	regs[COMM_FOCUS_DISTANCE 	   ].reg = 0XC2; regs[COMM_FOCUS_DISTANCE 		].len = 2;
	regs[COMM_GET_TEMPRATURE	   ].reg = 0XC3; regs[COMM_GET_TEMPRATURE		].len = 2;
	regs[COMM_FAN_FRE_CONTROL      ].reg = 0XC4; regs[COMM_FAN_FRE_CONTROL      ].len = 3;
	regs[COMM_FAN_DUTY_CONTROL     ].reg = 0XC5; regs[COMM_FAN_DUTY_CONTROL     ].len = 1;
	regs[COMM_LASER_PWM_FREQUENT   ].reg = 0XC6; regs[COMM_LASER_PWM_FREQUENT   ].len = 3;
	regs[COMM_LASER_PWM_DUTY       ].reg = 0XC7; regs[COMM_LASER_PWM_DUTY       ].len = 1;
	regs[COMM_FIRE_DETECT          ].reg = 0XC8; regs[COMM_FIRE_DETECT          ].len = 2;
	regs[COMM_GET_LASER_AGE        ].reg = 0XC9; regs[COMM_GET_LASER_AGE        ].len = 4;
	regs[COMM_GET_MCU_SN           ].reg = 0XCA; regs[COMM_GET_MCU_SN           ].len = 12;
	regs[COMM_GET_LASER_VENDOR     ].reg = 0XCB; regs[COMM_GET_LASER_VENDOR     ].len = 1;
	regs[COMM_GET_PRODUCT_DATE     ].reg = 0XCC; regs[COMM_GET_PRODUCT_DATE     ].len = 3;
	regs[COMM_GET_HARDWARE_VERSION ].reg = 0XCD; regs[COMM_GET_HARDWARE_VERSION ].len = 2;
	regs[COMM_GET_SOFTWARE_VERSION ].reg = 0XCE; regs[COMM_GET_SOFTWARE_VERSION ].len = 2;

	laser_init_flag = 1;

}
uint8_t laser_write(uint8_t* data, uint16_t len)
{
	if(laser_init_flag == 1)
	{
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (LASER_IIC_ADDR << 1) | I2C_MASTER_WRITE, 1);
		i2c_master_write(cmd, data, len, 1);
		i2c_master_stop(cmd);
		esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
		i2c_cmd_link_delete(cmd);
		if (ret != ESP_OK)
		{
			mprintf(LOG_ERROR,"IIC write error:%d.\r\n",ret);
			return ret;
		}
		return ret;
	}
	return 0;
}


uint8_t laser_protocol_write(regNum reg_num, uint8_t* data, uint16_t len)
{
	uint8_t buf[256] = {0};
	uint8_t check_sum = 0;
	uint16_t i = 0, j = 0;
	buf[i++] = regs[reg_num].reg;
	buf[i++] = len;
	memcpy(&buf[i], data, len);
	i = i + len;
	for(j = 0; j < i; j++)
	{
		check_sum = check_sum + buf[j];
	}
	buf[i++] = ~check_sum;

	laser_write(buf, i);
	return 0;
}
/**
 * 读数字激光器寄存器
 */
uint8_t laser_protocol_read(regNum reg_num, uint8_t* buf)
{
	//laser_protocol_write(reg_num, buf, 0);
	//laser_read(buf,regs[reg_num].len);
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (LASER_IIC_ADDR << 1) | I2C_MASTER_WRITE, 1);
	i2c_master_write_byte(cmd, regs[reg_num].reg, 1);	//寄存器
	i2c_master_write_byte(cmd, 0, 1);					//数据长度，读时为0
	i2c_master_write_byte(cmd, ~regs[reg_num].reg, 1);	//check sum
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (LASER_IIC_ADDR << 1) | I2C_MASTER_READ, 1);
	for(int i = 0; i < regs[reg_num].len + 3; i++)
	{
		if(i == (regs[reg_num].len -1))
		{
			i2c_master_read_byte(cmd, &buf[i], I2C_MASTER_NACK);
		}
		else
		{
			i2c_master_read_byte(cmd, &buf[i], I2C_MASTER_ACK);
		}

	}
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 300 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if (ret != ESP_OK) {
			mprintf(LOG_ERROR,"IIC read error:%d.\r\n",ret);
			return ret;
		}
	return 0;
}
/*
 * 用来设置一些整形数据类型的寄存器
 */
uint8_t laser_set_value(regNum reg_num, uint32_t value)
{
	uint8_t buf[10] = {0};
	uint8_t i = 0, j = 0;
	for(i = regs[reg_num].len; i > 0;i--)
	{
		buf[j++] = value >> (8 * (i -1));
	}
	laser_protocol_write(regs[reg_num].reg, buf, regs[reg_num].len);
	return 0;
}



void digital_laser_test(void)
{
	uint8_t buf[256] = {0};
	while(1)
	{
		laser_protocol_read(COMM_FOCUS_DISTANCE,buf);
		printf("current distance:%d.\r\n", (buf[2]<< 8) | buf[3]);
		HAL_Delay(1000);
	}
}

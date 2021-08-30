/*
 * digital_laser.c
 *
 *  Created on: 2021年7月19日
 *      Author: c
 */

#include "digital_laser.h"
#include "driver/i2c.h"
#include <string.h>
#include "grbl/system.h"
#include "grbl/report.h"
#include "driver.h"
#include "unistd.h"
#include "software_i2c.h"

#define LASER_IIC_ADDR 0X21


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
	regs[COMM_LASER_PWM_DUTY       ].reg = 0XC7; regs[COMM_LASER_PWM_DUTY       ].len = 2;
	regs[COMM_FIRE_DETECT          ].reg = 0XC8; regs[COMM_FIRE_DETECT          ].len = 2;
	regs[COMM_GET_LASER_AGE        ].reg = 0XC9; regs[COMM_GET_LASER_AGE        ].len = 4;
	regs[COMM_GET_MCU_SN           ].reg = 0XCA; regs[COMM_GET_MCU_SN           ].len = 12;
	regs[COMM_GET_LASER_VENDOR     ].reg = 0XCB; regs[COMM_GET_LASER_VENDOR     ].len = 1;
	regs[COMM_GET_PRODUCT_DATE     ].reg = 0XCC; regs[COMM_GET_PRODUCT_DATE     ].len = 3;
	regs[COMM_GET_HARDWARE_VERSION ].reg = 0XCD; regs[COMM_GET_HARDWARE_VERSION ].len = 2;
	regs[COMM_GET_SOFTWARE_VERSION ].reg = 0XCE; regs[COMM_GET_SOFTWARE_VERSION ].len = 2;

	regs[COMM_GET_BTN_STATE        ].reg = 0XCF; regs[COMM_GET_BTN_STATE        ].len = 1;
	regs[COMM_GET_VL6180_CONNECTED ].reg = 0XD0; regs[COMM_GET_VL6180_CONNECTED ].len = 1;
	regs[COMM_GET_NTC_CONNECTED    ].reg = 0XD1; regs[COMM_GET_NTC_CONNECTED    ].len = 1;
	regs[COMM_GET_LASER_STATE      ].reg = 0XD2; regs[COMM_GET_LASER_STATE      ].len = 1;

	regs[COMM_LASER_PWM_DUTY_255   ].reg = 0xE0; regs[COMM_LASER_PWM_DUTY_255	].len = 1;
	regs[COMM_LASER_PWM_DUTY_511   ].reg = 0xE1; regs[COMM_LASER_PWM_DUTY_511	].len = 1;
	regs[COMM_LASER_PWM_DUTY_767   ].reg = 0xE2; regs[COMM_LASER_PWM_DUTY_767	].len = 1;
	regs[COMM_LASER_PWM_DUTY_1000  ].reg = 0xE3; regs[COMM_LASER_PWM_DUTY_1000  ].len = 1;

	laser_init_flag = 1;
	/*创建激光功率设置传输任务*/
	laser_task_create();
}
uint8_t laser_write(uint8_t* data, uint16_t len)
{
	if(laser_init_flag == 1)
	{
#if USE_SOFTWARE_IIC
		sw_i2c_master_start();
		sw_i2c_master_write_byte((LASER_IIC_ADDR << 1) | I2C_MASTER_WRITE);
		sw_i2c_master_write(data, len);
		sw_i2c_master_stop();
#else
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (LASER_IIC_ADDR << 1) | I2C_MASTER_WRITE, 1);
		i2c_master_write(cmd, data, len, 1);
		i2c_master_stop(cmd);
		esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 300 / portTICK_RATE_MS);
		i2c_cmd_link_delete(cmd);
		if (ret != ESP_OK)
		{
			mprintf(LOG_ERROR,"IIC write error:%d.\r\n",ret);
			return ret;
		}
		return ret;
#endif
	}
	return 0;
}


uint8_t laser_protocol_write(LaserRegNum reg_num, uint8_t* data, uint16_t len)
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
uint8_t laser_protocol_read(LaserRegNum reg_num, uint8_t* buf)
{
#if USE_SOFTWARE_IIC
	sw_i2c_master_start();
	sw_i2c_master_write_byte((LASER_IIC_ADDR << 1) | I2C_MASTER_WRITE);
	sw_i2c_master_write_byte(regs[reg_num].reg);
	sw_i2c_master_write_byte(0);
	sw_i2c_master_write_byte(~regs[reg_num].reg);
	sw_i2c_master_stop();

	sw_i2c_master_start();
	sw_i2c_master_write_byte((LASER_IIC_ADDR << 1) | I2C_MASTER_READ);
	for(int i = 0; i < regs[reg_num].len + 3; i++)
	{
		if(i == (regs[reg_num].len + 2))
		{
			sw_i2c_master_read_byte(&buf[i], I2C_MASTER_NACK);
		}
		else
		{
			sw_i2c_master_read_byte(&buf[i], I2C_MASTER_ACK);
		}
	}
	sw_i2c_master_stop();
#else
	//laser_protocol_write(reg_num, buf, 0);
	//laser_read(buf,regs[reg_num].len);
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (LASER_IIC_ADDR << 1) | I2C_MASTER_WRITE, 1);
	i2c_master_write_byte(cmd, regs[reg_num].reg, 1);	//寄存器
	i2c_master_write_byte(cmd, 0, 1);					//数据长度，读时为0
	i2c_master_write_byte(cmd, ~regs[reg_num].reg, 1);	//check sum
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 300 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (LASER_IIC_ADDR << 1) | I2C_MASTER_READ, 1);
	for(int i = 0; i < regs[reg_num].len + 3; i++)
	{
		if(i == (regs[reg_num].len + 2))
		{
			i2c_master_read_byte(cmd, &buf[i], I2C_MASTER_NACK);
		}
		else
		{
			i2c_master_read_byte(cmd, &buf[i], I2C_MASTER_ACK);
		}
	}
	i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 300 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if (ret != ESP_OK) {
			mprintf(LOG_ERROR,"IIC read error:%d.\r\n",ret);
			return ret;
		}
#endif
	return 0;
}

uint32_t laser_power = 0;
/*
 * 用来设置一些整形数据类型的寄存器
 */
uint8_t laser_set_value(LaserRegNum reg_num, uint32_t value)
{
	uint8_t buf[10] = {0};
	uint8_t i = 0, j = 0;
	if(reg_num == COMM_LASER_PWM_DUTY)
	{
//		if(laser_power == value)
//		{
//			return 0;
//		}
		laser_power = value;

		/*简化协议只传3个字节，地址，寄存器，值*/
		buf[0] = 0xE0 + (value / 0x100);
		buf[1] = value % 0x100;
		laser_write(buf, 2);
		return 0;
	}
	//portENTER_CRITICAL(&mux2);
	for(i = regs[reg_num].len; i > 0;i--)
	{
		buf[j++] = value >> (8 * (i -1));
	}
	laser_protocol_write(reg_num, buf, regs[reg_num].len);
	//portEXIT_CRITICAL(&mux2);
	return 0;
}
/*
 * 用来获取一些整形数据类型的寄存器
 */
uint32_t laser_get_value(LaserRegNum reg_num)
{
	uint8_t buf[200] = {0};
	uint32_t value = 0;

	if(reg_num == COMM_LASER_PWM_DUTY)
	{
		return laser_power;
	}
	//portENTER_CRITICAL(&mux2);

	laser_protocol_read(reg_num,buf);
//	if(reg_num == COMM_GET_TEMPRATURE)
//	{
//		printf("%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\r\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7],buf[8]);
//
//	}
	for(int i = 0; i < regs[reg_num].len; i++)
	{
		if(i > 3) break;
		value = (value << 8) + buf[i + 2];
	}

	//portEXIT_CRITICAL(&mux2);
	//printf("current distance:%d.\r\n", (buf[2]<< 8) | buf[3]);
	return value;
}
/*
 * 返回对焦状态
 */
uint8_t laser_probe_state(void)
{
	laser_read_write_enqueue(LASER_READ, COMM_FOCUS_DISTANCE,0);
	if(laser_info.distance > settings.laser_focal_length)
	{
		return (laser_info.distance - settings.laser_focal_length) < 1 ? 1 : 0;
	}
	else
	{
		return (settings.laser_focal_length - laser_info.distance) < 1 ? 1 : 0;
	}
}

#define LASER_FOCUS_TIMES 10    	//循环对焦次数
#define LASER_FOCUS_LIMIT 1			//对焦运行最大误差mm
#define LASER_FOCUS_VALID_TIMES 3	//连续对焦OK次数
uint8_t laser_focus_status = 0;
void laser_auto_focus(void* arg)
{
	uint32_t distance = 0;
	uint32_t focus_ok_cnt = 0;
	uint32_t diff_value = 0;
	if(settings.laser_focal_length > 0)
	{
		if(laser_focus_status == 0)
		{
			for(int i = 0; i < LASER_FOCUS_TIMES; i++)
			{
				HAL_Delay(500);
				distance = laser_get_value(COMM_FOCUS_DISTANCE);
				diff_value = distance > settings.laser_focal_length ? distance - settings.laser_focal_length : settings.laser_focal_length - distance;
				printf("current distance:%d:%d.\r\n", distance,diff_value);
				if(diff_value <= LASER_FOCUS_LIMIT)
				{
					focus_ok_cnt++;
					if(focus_ok_cnt > LASER_FOCUS_VALID_TIMES)
					{
						//对焦ok
						//return ;
					}
				}
				else
				{
					char cmd[20] ={0};
					if(distance > settings.laser_focal_length)
					{
//						portENTER_CRITICAL(&mux2);
//						/*电机向下运动*/
//						gpio_set_level(Z_DIRECTION_PIN,1);
//						gpio_set_level(STEPPERS_DISABLE_PIN,0);
//						for(int i = 0; i < diff_value * 200 * 8; i++)
//						{
//							usleep(4);
//							gpio_set_level(Z_STEP_PIN,1);
//							usleep(4);
//							gpio_set_level(Z_STEP_PIN,0);
//
//						}
//
//						gpio_set_level(STEPPERS_DISABLE_PIN,1);
//						portENTER_CRITICAL(&mux2);
						sprintf(cmd,"G91Z%d",diff_value);
						report_status_message(gc_execute_block(cmd, NULL));
						HAL_Delay(6000);
					}
					else
					{
						/*电机向上运动*/
//						gpio_set_level(Z_DIRECTION_PIN,0);
//						gpio_set_level(STEPPERS_DISABLE_PIN,0);
//
//						for(int i = 0; i < diff_value * 200 * 8; i++)
//						{
//							gpio_set_level(Z_STEP_PIN,1);
//							usleep(20);
//							gpio_set_level(Z_STEP_PIN,0);
//							usleep(20);
//						}
//
//						gpio_set_level(STEPPERS_DISABLE_PIN,1);
						sprintf(cmd,"G91Z-%d",diff_value);
						report_status_message(gc_execute_block(cmd, NULL));
						HAL_Delay(6000);
					}
					printf("%s",cmd);
				}
			}
		}
	}
	while(1)
	{
		HAL_Delay(1000);
	}
	return;
}

static uint8_t ucParameterFocus;
TaskHandle_t auto_focus_task_handle = NULL;


void laser_auto_focus_task_create(void)
{
	xTaskCreate( laser_auto_focus, "extended_FuncTask", 2048, &ucParameterFocus, 6, &auto_focus_task_handle );
	configASSERT( auto_focus_task_handle );
}

void digital_laser_test(void)
{
	uint32_t j = 0;
	while(1)
	{
		j = j + 100;
		if(j > 1000) j = 0;
		laser_set_value(COMM_LASER_PWM_DUTY,j);
		HAL_Delay(10);

		//int distance = laser_get_value(COMM_FOCUS_DISTANCE);
		//printf("current distance:%d.\r\n", distance);
		//HAL_Delay(1000);
	}
}

#define PWM_DUTY_QUEUE_LENGTH 50
#define LASER_TASK_PRIORITY  10

/*在中断标志*/
uint8_t isr_flag = 0;
/*pwm占空比队列*/
QueueHandle_t pwmDutyQueue;

typedef struct {
	LaserRegNum reg_num; //寄存器
	uint8_t rw_flag; //读写标志位
	uint32_t vlaue;  //写入值
}LaserRW_Node;

/*
 * breif:进入中断标志位，用于判断激光pwm值入队时需要调用的函数
 */
void laser_enter_isr(void)
{
	isr_flag++;
}
void laser_exit_isr(void)
{
	if(isr_flag > 0)
		isr_flag--;
}
/*
 * breif:占空比发送任务
 * */
void laser_task(void* arg)
{
	LaserRW_Node laser_cmd = {0};
	BaseType_t xStatus;
	pwmDutyQueue = xQueueCreate( PWM_DUTY_QUEUE_LENGTH, sizeof(LaserRW_Node));  //创建一个消息队列
	for(;;)
	{
		xStatus = xQueueReceive(pwmDutyQueue,&laser_cmd,0xffffff);
		if(xStatus == pdPASS)
		{
			if(laser_cmd.rw_flag == LASER_WRITE)
			{
				laser_set_value(laser_cmd.reg_num, laser_cmd.vlaue);
			}
			else
			{
				switch (laser_cmd.reg_num)
				{
					case COMM_FOCUS_DISTANCE:
					{
						laser_info.distance = laser_get_value(laser_cmd.reg_num);
						break;
					}
					case COMM_GET_TEMPRATURE:
					{
						laser_info.temperture = laser_get_value(laser_cmd.reg_num);
						break;
					}
					case COMM_FIRE_DETECT:
					{
						laser_info.fire = laser_get_value(laser_cmd.reg_num);
						break;
					}
					case COMM_FAN_DUTY_CONTROL:
					{
						laser_info.fan_duty = laser_get_value(laser_cmd.reg_num);
						break;
					}
					case COMM_GET_BTN_STATE:
					{
						laser_info.key = laser_get_value(laser_cmd.reg_num);
						break;
					}
					default:
					{
						laser_get_value(laser_cmd.reg_num);
						break;
					}
				}

			}

		}
	}
}
/*
 * breif:创建占空比发送任务
 */
void laser_task_create(void)
{
	xTaskCreate(laser_task, "laser_task", 2048, NULL, LASER_TASK_PRIORITY, NULL );
}
/*
 * breif:激光占空比入队
 */
void laser_pwm_duty_enqueue(int value)
{
	laser_read_write_enqueue(LASER_WRITE, COMM_LASER_PWM_DUTY, value);
}

/*
 * breif:数字激光器读写入队
 */
void laser_read_write_enqueue(uint8_t rw_flag, LaserRegNum reg, int value)
{
	const TickType_t xTicksToWait = pdMS_TO_TICKS(0);
	LaserRW_Node laser_cmd = {0};
	laser_cmd.rw_flag = rw_flag;
	laser_cmd.reg_num = reg;
	laser_cmd.vlaue = value;
	if(isr_flag)
	{
		xQueueSendToBackFromISR(pwmDutyQueue, &laser_cmd, xTicksToWait);
	}
	else
	{
		xQueueSendToBack(pwmDutyQueue, &laser_cmd, xTicksToWait);
	}
}

LaserInfo laser_info = {0};
/*
 * breif:数字激光心跳
 */
void laser_keep_active(void)
{
	static uint32_t time = 0;

	if(sys.state == STATE_CYCLE)
	{
		if((HAL_GetTick() - time) > 3000)
		{
			time = HAL_GetTick();
			laser_set_value(COMM_TICK_ALIVE, 0);
		}
	}
	else
	{
		if((HAL_GetTick() - time) > 500)
		{
			time = HAL_GetTick();
			laser_info.distance = laser_get_value(COMM_FOCUS_DISTANCE);
			laser_info.temperture = laser_get_value(COMM_GET_TEMPRATURE);
			laser_info.fire = laser_get_value(COMM_FIRE_DETECT);
			laser_info.fan_duty = laser_get_value(COMM_FAN_DUTY_CONTROL);
			laser_info.laser_duty = laser_get_value(COMM_LASER_PWM_DUTY);
			laser_info.laser_fre = laser_get_value(COMM_LASER_PWM_FREQUENT);
			laser_info.key = laser_get_value(COMM_GET_BTN_STATE);
			laser_info.is_vl6180_connected = laser_get_value(COMM_GET_VL6180_CONNECTED);

			/*读取PWM频率*/
			laser_info.laser_fre = laser_get_value(COMM_LASER_PWM_FREQUENT);
			/**/
			if(laser_info.laser_fre != settings.spindle.pwm_freq)
			{
				laser_set_value(COMM_TICK_ALIVE, 0);
			}
		}
	}


}

#define FORWARD_DISTANCE 5
uint8_t auto_focus_flag = 0;

void laser_auto_focus_set(uint8_t flag)
{
	auto_focus_flag = flag;
}

void laser_auto_focus_cycle(void)
{
//	"$HZ\r\n"
//	"G21\r\n"
//	"G91\r\n"
//	/*读到达焦距的大致长度*/
//	"G38.2Z-50F100\r\n"
//	"G0Z-5\r\n"
//	"G38.2Z10F100\r\n"
//	"G0Z+5\r\n"
//	"G38.2Z-10F100\r\n"
//	 "G90\r\n"
//	 "G10 L20 P1 Z0\r\n"
	if(auto_focus_flag == 0) return;
	auto_focus_flag = 0;

	char line[257] = {0};
	uint32_t dis = 0;

	memcpy(line,"$HZ\0",3);
    report_status_message(system_execute_line(line));

	grbl.report.status_message(gc_execute_block("G21\0",NULL));
	grbl.report.status_message(gc_execute_block("G91\0",NULL));
	/*获取当前距离*/
	laser_info.distance = laser_get_value(COMM_FOCUS_DISTANCE);
	if(laser_info.distance < settings.laser_focal_length)
	{
		/*当前距离比焦距还小*/
		hal.stream.write_all("[auto focus fail. The carving is too close to the lens.]");
		return ;
	}
	dis = laser_info.distance - settings.laser_focal_length + FORWARD_DISTANCE ;
	/*先快速移动2/3的距离*/


	sprintf(line, "G38.2Z-%dF600", dis);
	grbl.report.status_message(gc_execute_block(line,NULL));
	grbl.report.status_message(gc_execute_block("G0Z-5\0",NULL));
	grbl.report.status_message(gc_execute_block("G38.2Z15F100\0",NULL));
	grbl.report.status_message(gc_execute_block("G0Z5\0",NULL));
	HAL_Delay(100);
	grbl.report.status_message(gc_execute_block("G38.2Z-15F100\0",NULL));
	grbl.report.status_message(gc_execute_block("G90\0",NULL));
	grbl.report.status_message(gc_execute_block("G10 L20 P1 Z0\0",NULL));


}

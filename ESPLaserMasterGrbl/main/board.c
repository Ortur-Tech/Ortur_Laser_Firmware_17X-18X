
#include "driver.h"
#include "board.h"
#include <unistd.h>
#include "esp32-hal-uart.h"
#include "accelDetection.h"

#define TIMER_DIVIDER         (80)  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds



void Usb_ForceReset(void)
{
	gpio_config_t gpioConfig = {
			.pin_bit_mask = ((uint64_t)1 << GPIO_NUM_20),
			.mode = GPIO_MODE_OUTPUT,
			.pull_up_en = GPIO_PULLDOWN_DISABLE,
			.pull_down_en = GPIO_PULLDOWN_DISABLE,
			.intr_type = GPIO_INTR_DISABLE
		};

	gpio_config(&gpioConfig);

	gpio_set_level(GPIO_NUM_20, 0);
}

/*��չ��������*/
void extended_FuncTask( void * pvParameters )
{
	for( ;; )
	{
#if ENABLE_ACCELERATION_DETECT
		/*���ٶȼ��*/
		accel_detection_limit();
#endif
		/*��ƽ��ֵ*/
		fire_GetAverageValue();
		// Task code goes here.
		vTaskDelay(10/portTICK_PERIOD_MS);
	}
}

static uint8_t ucParameterToPass;
TaskHandle_t extendedFuncTaskHandle = NULL;

// Function that creates a task.
void creat_ExtFuncTask( void )
{

	 // Create the task, storing the handle.  Note that the passed parameter ucParameterToPass
	 // must exist for the lifetime of the task, so in this case is declared static.  If it was just an
	 // an automatic stack variable it might no longer exist, or at least have been corrupted, by the time
	 // the new task attempts to access it.
	xTaskCreate( extended_FuncTask, "extended_FuncTask", 2048, &ucParameterToPass, 3, &extendedFuncTaskHandle );
    configASSERT( extendedFuncTaskHandle );
}

/*
 * board.c
 *
 *  Created on: 2021��5��20��
 *      Author: c
 */
void HAL_Delay(uint32_t ms)
{
	vTaskDelay(ms/portTICK_PERIOD_MS);
}
uint32_t hal_tick_timer = 0;

static bool IRAM_ATTR HAL_TickInc(void *args)
{
	hal_tick_timer++;
#if ENABLE_ACCELERATION_DETECT
	/*���ٶȼ��*/
	//accel_detection_limit();
#endif
	/*����״̬����*/
	fire_Alarm();

#ifdef DELAY_OFF_SPINDLE
	/*�ӳٹؼ���ɢ�ȷ���*/
	spindle_calculate_heat();
	spindle_delay_stop();
#endif

#if USB_SERIAL_CDC
    //if(usbPlugIn)
    {
        //setUsbPlugIn(usbPlugIn - 1);
        //if(!usbPlugIn)
        //	printf("USB Plug Out.\n\r");
    }
#endif

	return 1;
}
/*1ms��ʱ��*/
void HAL_TickInit(void)
{
	/* Select and initialize basic parameters of the timer */
	    timer_config_t config = {
	        .divider = TIMER_DIVIDER,
	        .counter_dir = TIMER_COUNT_UP,
	        .counter_en = TIMER_PAUSE,
	        .alarm_en = TIMER_ALARM_EN,
	        .auto_reload = 1000,
	    }; // default clock source is APB
	    timer_init(TIMER_GROUP_1, TIMER_1, &config);

	    /* Timer's counter will initially start from value below.
		   Also, if auto_reload is set, this value will be automatically reload on alarm */
		timer_set_counter_value(TIMER_GROUP_1, TIMER_1, 0);

		timer_set_alarm_value(TIMER_GROUP_1, TIMER_1, 1000);
		timer_enable_intr(TIMER_GROUP_1, TIMER_1);

		timer_isr_callback_add(TIMER_GROUP_1, TIMER_1, HAL_TickInc, NULL, 0);
		timer_start(TIMER_GROUP_1, TIMER_1);
}



uint32_t HAL_GetTick(void)
{
	return hal_tick_timer;
}
/**
 * @brief ����Ƭ����ƵΪ72�׺���ʱ�ȽϾ�׼�������ʱ
 * @param nus��λ΢��
 */
void soft_delay_us(uint32_t nus)
{
	usleep(nus);
}
/**
 * @brief soft_delay_ms
 * @param nms��λ����
 */
void soft_delay_ms(uint32_t nms)
{
	while(nms--)
	{
		soft_delay_us(1000);
	}
}
/*��������δ����������½�����Ļ�ĸ�λָ��*/
void UsartInit(void)
{

}


#define KEY_PRESS_POWERON_TIME 1000 //3S����
#define KEY_PRESS_POWEROFF_TIME 4000 //5S�ػ�
#define REPORT_STAUTS_INTERVAL_TIME  200  //200MS�ϱ�һ��

uint8_t key_power_status = 0;

uint8_t get_PowerKeyStatus(void)
{
	return key_power_status ;
}
/*
 *   ��һ���ֱ�ʾ����״̬���ڶ�����ʾ24v�ⲿ����״̬--�ȴ�����--<PS:0,0>���¿�������--<PS:1>����--<PS:2>���¹ػ�����--<PS:3>�ػ��ɹ�--<PS:4>
 *
 */
void report_KeyPowerSupplyStatus(system_status s,uint8_t mode)
{
	static uint32_t report_time = 0;
	char str[30] = {0};
	key_power_status = s;
	sprintf(str,"<Sleep|PS:%d,%d>\r\n",s,get_SystemPowerStatus());
	if(mode)
	{
		serialWriteS(str);
	}
	else
	{
		if((HAL_GetTick() - report_time) > REPORT_STAUTS_INTERVAL_TIME)
		{
			report_time = HAL_GetTick();
			serialWriteS(str);
		}
	}

}

uint8_t poweron_cmd = 0;

void poweron_CmdSet(uint8_t cmd)
{
	poweron_cmd = cmd;
}
/*
 * 1������		0���ػ�
 */
void key_func(uint8_t m)
{
	static uint32_t press_start_time = 0;
	static uint32_t led_flash_time = 0;
	static uint8_t key_release_flag = 0;

	static uint8_t report_onece = 0;
	if(m)
	{
		Usb_ForceReset();
	    HAL_TickInit();
	    led_Init();
	    serialInit();
		power_KeyInit();
		while(1)
		{
			/*���������*/
			if(poweron_cmd == CMD_POWERON)
			{
				/*�����ϱ�����״̬*/
				report_KeyPowerSupplyStatus(SYSTEM_POWERON,1);
				key_release_flag = 1;
				power_LedOn();
				break;
			}
			/*TODO:���������Ҫι��*/
			if(power_KeyDown() == 0)
			{
				/*����ϱ���������״̬*/
				report_KeyPowerSupplyStatus(SYSTEM_KEY_PRESS_FOR_POWERON,0);
				if((HAL_GetTick()-press_start_time) > KEY_PRESS_POWERON_TIME)
				{
					/*�����ϱ�����״̬*/
					report_KeyPowerSupplyStatus(SYSTEM_POWERON,1);
					key_release_flag = 1;
					power_LedOn();
					break;
				}
				if((HAL_GetTick()-led_flash_time)>100)
				{
					power_LedToggle();
					led_flash_time = HAL_GetTick();
				}
			}
			else
			{
				power_LedOff();
				/*����ϱ��ػ�״̬*/
				report_KeyPowerSupplyStatus(SYSTEM_WAIT_FOR_POWERON,0);
				press_start_time = HAL_GetTick();
			}

		}
	}
	else
	{
		/*���������ͷŰ������ܽ��йػ��ж�*/
		if(key_release_flag == 1)
		{
			if(power_KeyDown() == 0)
			{
				return;
			}
			else
			{
				key_release_flag = 0;
			}
		}

		if(power_KeyDown() == 0)
		{
			if(report_onece == 0)
			{
				/*����ϱ���������״̬*/
				report_KeyPowerSupplyStatus(SYSTEM_KEY_PRESS_FOR_POWEROFF,0);
			}

			if((HAL_GetTick()-press_start_time) > KEY_PRESS_POWEROFF_TIME)
			{

				if(report_onece == 0)
				{
					report_onece = 1;
					/*�����ϱ��ػ��ɹ�*/
					report_KeyPowerSupplyStatus(SYSTEM_POWEROFF,1);

				}
				power_LedOff();
				while(power_KeyDown() == 0)
				{
					report_KeyPowerSupplyStatus(SYSTEM_POWEROFF,0);
					/*TODO:��Ҫι��*/
				}
				power_LedOff();
				HAL_Delay(5);
				/*�����λ*/
				esp_restart();
				while(1);
			}
			else
			{
				if((HAL_GetTick()-led_flash_time)>100)
				{
					power_LedToggle();
					led_flash_time = HAL_GetTick();
				}
			}

		}
		else
		{
			key_power_status = SYSTEM_POWERON;
			press_start_time = HAL_GetTick();
		}
	}
}




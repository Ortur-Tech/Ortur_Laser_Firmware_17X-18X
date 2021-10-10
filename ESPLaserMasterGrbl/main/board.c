
#include "driver.h"
#include "board.h"
#include <unistd.h>
#include "esp32-hal-uart.h"
#include "accelDetection.h"
#include "serial_iap.h"
#include "driver/uart.h"
#include "digital_laser.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#define USE_FREERTOS_TIMER 1

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

	HAL_Delay(20);
}

/*扩展功能任务1.加速度检测 2.火焰ad值获取*/
void extended_FuncTask( void * pvParameters )
{
	vTaskDelay(1000/portTICK_PERIOD_MS);
	/*在这里开电源避免干扰*/
	//power_CtrOn();
	for( ;; )
	{
		/*根据是否有供电自动开关，避免插usb供电时打火*/
		power_auto_ctrl();

		vTaskDelay(10/portTICK_PERIOD_MS);

#if ENABLE_ACCELERATION_DETECT
		/*加速度检测*/
		accel_detection_limit();
#endif
		/*统计激光工作时长*/
		laser_on_time_count();
		/*不移动激光检查*/
		movement_laseron_check();
#if ENABLE_FIRE_CHECK
		/*都平均值*/
		fire_GetAverageValue();

		fire_Check();
#endif

#ifdef DELAY_OFF_SPINDLE
		spindle_calculate_heat();
		spindle_delay_stop();
#endif

#if ENABLE_DIGITAL_LASER
		laser_keep_active();
#endif
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
 *  Created on: 2021年5月20日
 *      Author: c
 */
void HAL_Delay(uint32_t ms)
{
	vTaskDelay(ms/portTICK_PERIOD_MS);
}
uint32_t hal_tick_timer = 0;

#if USE_FREERTOS_TIMER
void HAL_TickInc(TimerHandle_t xTimer)
#else
static bool IRAM_ATTR HAL_TickInc(void *args)
#endif
{
	laser_enter_isr();

	hal_tick_timer++;

#if ENABLE_FIRE_CHECK
	/*报警状态处理*/
	fire_Alarm();
#endif

#ifdef DELAY_OFF_SPINDLE
	/*延迟关激光散热风扇*/
	spindle_calculate_heat();
	spindle_delay_stop();
#endif
	rec_TimeCheck();
#if USB_SERIAL_CDC
    //if(usbPlugIn)
    {
        //setUsbPlugIn(usbPlugIn - 1);
        //if(!usbPlugIn)
        //	printf("USB Plug Out.\n\r");
    }
#endif

    laser_exit_isr();
#if !USE_FREERTOS_TIMER
	return 1;
#endif
}



#define TICK_TIMER_GROUP TIMER_GROUP_1
#define TICK_TIMER_INDX TIMER_1
/*1ms定时器*/
void HAL_TickInit(void)
{
#if USE_FREERTOS_TIMER
	int i = 5 ;
	TimerHandle_t tickTimer = NULL;
	tickTimer = xTimerCreate("user tick timer.", 1000, pdTRUE, (void*)i, HAL_TickInc);
	xTimerStart(tickTimer, 0);
#else
	/* Select and initialize basic parameters of the timer */
	    timer_config_t config = {
	        .divider = TIMER_DIVIDER,
	        .counter_dir = TIMER_COUNT_UP,
	        .counter_en = TIMER_PAUSE,
	        .alarm_en = TIMER_ALARM_EN,
	        .auto_reload = 1000,
	    }; // default clock source is APB
	    timer_init(TICK_TIMER_GROUP, TICK_TIMER_INDX, &config);

	    /* Timer's counter will initially start from value below.
		   Also, if auto_reload is set, this value will be automatically reload on alarm */
		timer_set_counter_value(TICK_TIMER_GROUP, TICK_TIMER_INDX, 0);

		timer_set_alarm_value(TICK_TIMER_GROUP, TICK_TIMER_INDX, 1000);
		timer_enable_intr(TICK_TIMER_GROUP, TICK_TIMER_INDX);

		timer_isr_callback_add(TICK_TIMER_GROUP, TICK_TIMER_INDX, HAL_TickInc, NULL, 0);
		timer_start(TICK_TIMER_GROUP, TICK_TIMER_INDX);
#endif
}



uint32_t HAL_GetTick(void)
{
#if USE_FREERTOS_TIMER
	return xTaskGetTickCount();
#else
	return hal_tick_timer;
#endif
}
/**
 * @brief 当单片机主频为72兆赫兹时比较精准的软件延时
 * @param nus单位微秒
 */
void soft_delay_us(uint32_t nus)
{
	usleep(nus);
}
/**
 * @brief soft_delay_ms
 * @param nms单位毫秒
 */
void soft_delay_ms(uint32_t nms)
{
	while(nms--)
	{
		soft_delay_us(1000);
	}
}
/*仅用于在未开机的情况下接收屏幕的复位指令*/
void UsartInit(void)
{

}


#define KEY_PRESS_POWERON_TIME 1000 //3S开机
#define KEY_PRESS_POWEROFF_TIME 4000 //5S关机
#define REPORT_STAUTS_INTERVAL_TIME  200  //200MS上报一次

uint8_t key_power_status = 0;

uint8_t get_PowerKeyStatus(void)
{
	return key_power_status ;
}
/*
 *   第一个字表示开关状态，第二个表示24v外部供电状态--等待开机--<PS:0,0>按下开机按键--<PS:1>开机--<PS:2>按下关机按键--<PS:3>关机成功--<PS:4>
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
 * 1：开机		0：关机
 */
void key_func(uint8_t m)
{
	static uint32_t press_start_time = 0;
	static uint32_t led_flash_time = 0;
	static uint8_t key_release_flag = 0;

	static uint8_t report_onece = 0;
	if(m)
	{
		//Usb_ForceReset();
	    HAL_TickInit();
	    led_Init();
	    serialInit();
		power_KeyInit();
		//single_uart_init(X_AXIS);
		while(1)
		{
			/*接收命令开机*/
			if(poweron_cmd == CMD_POWERON)
			{
				/*立即上报开机状态*/
				report_KeyPowerSupplyStatus(SYSTEM_POWERON,1);
				key_release_flag = 1;
				power_LedOn();
				break;
			}
			/*TODO:这里可能需要喂狗*/
			if(power_KeyDown())
			{
				/*间隔上报按键按下状态*/
				report_KeyPowerSupplyStatus(SYSTEM_KEY_PRESS_FOR_POWERON,0);
				if((HAL_GetTick()-press_start_time) > KEY_PRESS_POWERON_TIME)
				{
					/*立即上报开机状态*/
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
				/*间隔上报关机状态*/
				report_KeyPowerSupplyStatus(SYSTEM_WAIT_FOR_POWERON,0);
				press_start_time = HAL_GetTick();
			}

		}
	}
	else
	{
		/*开机后需释放按键才能进行关机判断*/
		if(key_release_flag == 1)
		{
			if(power_KeyDown())
			{
				return;
			}
			else
			{
				key_release_flag = 0;
			}
		}

		if(power_KeyDown())
		{
			if(report_onece == 0)
			{
				/*间隔上报按键按下状态*/
				report_KeyPowerSupplyStatus(SYSTEM_KEY_PRESS_FOR_POWEROFF,0);
			}

			if((HAL_GetTick()-press_start_time) > KEY_PRESS_POWEROFF_TIME)
			{

				if(report_onece == 0)
				{
					report_onece = 1;
					/*立即上报关机成功*/
					report_KeyPowerSupplyStatus(SYSTEM_POWEROFF,1);

				}
				power_LedOff();
				while(power_KeyDown())
				{
					report_KeyPowerSupplyStatus(SYSTEM_POWEROFF,0);
					/*TODO:需要喂狗*/
				}
				power_LedOff();
				/*关屏幕电源*/
				power_CtrOff();
				HAL_Delay(5);
				/*软件复位*/
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

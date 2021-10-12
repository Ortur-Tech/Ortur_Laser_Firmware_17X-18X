/*
 * debug_probe.c
 *
 *  Created on: 2021年10月10日
 *      Author: c
 */

#define DEBUG_PROBE_NUM 10
#define DEBUG_PEOBE_ACTIVE 1
#define DEBUG_PEOBE_DEACTIVE 0

#include "debug_probe.h"

#ifdef USE_DEBUG_PROBE

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"

debug_probe dp_list[DEBUG_PROBE_NUM] = {0};

debug_probe * dp_stack_top = NULL;
debug_probe * last_dp_stack_top = NULL;

portMUX_TYPE mux_dp_stack = portMUX_INITIALIZER_UNLOCKED;

// 预设调试探针
debug_probe * grbl_task_dp = NULL;
debug_probe * grbl_safe_func_dp = NULL;
debug_probe * grbl_stepper_isr_dp = NULL;
debug_probe * usb_cdc_isr_dp = NULL;
debug_probe * uart_isr_dp = NULL;

static void active_debug_probe(debug_probe* dp)
{
	if(dp && dp->probe >0 && dp->probe <46)
		gpio_set_level(dp->probe, DEBUG_PEOBE_ACTIVE );
}

static void deactive_debug_probe(debug_probe* dp)
{
	if(dp && dp->probe >0 && dp->probe <46)
		gpio_set_level(dp->probe, DEBUG_PEOBE_DEACTIVE );
}


void init_debug_probe()
{
	int i = 0;
	for(i = 0; i < DEBUG_PROBE_NUM; i++)
	{
		delete_debug_probe(&dp_list[i]);
	}
}

debug_probe* create_debug_probe(int task, uint32_t probe)
{
	int i = 0;
	debug_probe* new_dp = NULL;
	debug_probe* other_dp = NULL;

	if(task == -1 ||probe == -1)
	{
		return NULL;
	}

	for(i = 0; i < DEBUG_PROBE_NUM; i++)
	{
		if(dp_list[i].task == -1)
		{
			new_dp = &dp_list[i];

			new_dp->task = task;
			new_dp->probe = probe;

			gpio_config_t gpioConfig = {
				.pin_bit_mask = ((uint64_t)1 << probe),
				.mode = GPIO_MODE_OUTPUT,
				.pull_up_en = GPIO_PULLUP_ENABLE,
				.pull_down_en = GPIO_PULLDOWN_DISABLE,
				.intr_type = GPIO_INTR_DISABLE
			};

			gpio_config(&gpioConfig);
			gpio_set_level(probe,DEBUG_PEOBE_DEACTIVE);


			break;
		}
	}

	return new_dp;
}

void delete_debug_probe(debug_probe* dp)
{
	dp->level = -1;
	dp->probe = -1;
	dp->task = -1;
	dp->next_probe = NULL;
	dp->pre_probe = NULL;
}

void _push_debug_probe(debug_probe* dp)
{
	if(dp_stack_top != dp && dp->pre_probe == NULL)
	{
		last_dp_stack_top = dp_stack_top;
		dp_stack_top = dp;

		if(dp_stack_top)
		{
			dp_stack_top->level = last_dp_stack_top ? last_dp_stack_top->level + 1 : 0;
			dp_stack_top->next_probe = NULL;
			dp_stack_top->pre_probe = last_dp_stack_top;
		}

		if(last_dp_stack_top)
		{
			last_dp_stack_top->next_probe = dp_stack_top;
		}

		deactive_debug_probe(last_dp_stack_top);
		active_debug_probe(dp_stack_top);
	}
}

void push_debug_probe(debug_probe* dp)
{
	//portENTER_CRITICAL(&mux_dp_stack);

	_push_debug_probe(dp);

	//portEXIT_CRITICAL(&mux_dp_stack);
}

void push_debug_probe_isr(debug_probe* dp)
{
	//portENTER_CRITICAL_ISR(&mux_dp_stack);

	_push_debug_probe(dp);

	//portEXIT_CRITICAL_ISR(&mux_dp_stack);
}

debug_probe* _pop_debug_probe()
{

	last_dp_stack_top = dp_stack_top;
	if(last_dp_stack_top)
	{
		last_dp_stack_top->level = -1;
		dp_stack_top = last_dp_stack_top->pre_probe;
	}

	deactive_debug_probe(last_dp_stack_top);
	active_debug_probe(dp_stack_top);

	return last_dp_stack_top;
}

debug_probe* pop_debug_probe()
{
	//portENTER_CRITICAL(&mux_dp_stack);

	last_dp_stack_top = _pop_debug_probe();

	//portEXIT_CRITICAL(&mux_dp_stack);

	return last_dp_stack_top;

}

debug_probe* pop_debug_probe_isr()
{
	//portENTER_CRITICAL_ISR(&mux_dp_stack);

	last_dp_stack_top = _pop_debug_probe();

	//portENTER_CRITICAL_ISR(&mux_dp_stack);

	return last_dp_stack_top;
}

int debug_probe_stack_level()
{
	return dp_stack_top ? dp_stack_top->level : -1;
}

#endif

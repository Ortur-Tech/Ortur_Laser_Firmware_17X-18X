/*
 * debug_probe.h
 *
 *  Created on: 2021年10月10日
 *      Author: c
 */

#ifndef MAIN_DEBUG_PROBE_H_
#define MAIN_DEBUG_PROBE_H_

#define USE_DEBUG_PROBE 1

#ifdef USE_DEBUG_PROBE

#include "stdint.h"

typedef struct
{
	int task;
	int level;
	int probe;
	void* next_probe;
	void* pre_probe;
}debug_probe;

extern debug_probe * dp_stack_top;

extern debug_probe * grbl_task_dp;
extern debug_probe * grbl_safe_func_dp;
extern debug_probe * grbl_stepper_isr_dp;
extern debug_probe * usb_cdc_isr_dp;
extern debug_probe * uart_isr_dp;

void init_debug_probe();

debug_probe* create_debug_probe(int task, uint32_t probe);

void delete_debug_probe(debug_probe* dp);

void push_debug_probe(debug_probe* dp);
void push_debug_probe_isr(debug_probe* dp);

debug_probe* pop_debug_probe();
debug_probe* pop_debug_probe_isr();

int debug_probe_stack_level();

#endif


#endif /* MAIN_DEBUG_PROBE_H_ */

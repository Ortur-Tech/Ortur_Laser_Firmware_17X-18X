/*
 * usb_serial.h
 *
 *  Created on: 2021��5��20��
 *      Author: c
 */

#ifndef MAIN_USB_SERIAL_H_
#define MAIN_USB_SERIAL_H_
#include "board.h"
#include <stdbool.h>


void usb_SerialInit(void);
void setUsbCDCConnected(uint8_t value);
void usbInit (void);
uint8_t isUsbPlugIn(void);
uint8_t isUsbCDCConnected(void);
void usbReset(void);
int16_t usbGetC(void);
void usbWriteS(const char *s);
uint16_t usbRxFree (void);
void usbRxFlush(void);
void usbRxCancel(void);
void usbBufferInput (uint8_t *data, uint32_t length);
bool usbSuspendInput (bool suspend);
uint8_t isUsbPlugIn(void);
extern uint8_t usbPlugIn;
extern uint8_t usbCDCConnected;

#endif /* MAIN_USB_SERIAL_H_ */

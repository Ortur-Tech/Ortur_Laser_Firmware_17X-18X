/*
 * usb_serial.c
 *
 *  Created on: 2021��5��20��
 *      Author: c
 */


#include "usb_serial.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "esp_log.h"

#include "grbl/grbl.h"
#include "esp32-hal-uart.h"
#include "tusb_msc_disk.h"
#include "digital_laser.h"

static const char *TAG = "usb";
static uint8_t buf[CONFIG_USB_CDC_RX_BUFSIZE + 1];

void usb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    /* initialization */
    size_t rx_size = 0;
    laser_enter_isr();
    /* read */
    esp_err_t ret = tinyusb_cdcacm_read(itf, buf, CONFIG_USB_CDC_RX_BUFSIZE, &rx_size);
    if (ret == ESP_OK)
    {
		usbBufferInput(buf, rx_size);
		if(!isUsbCDCConnected())
		{
			setUsbCDCConnected(1);//收到USB数据证明VCP已经被连接
		}
    } else {
        ESP_LOGE(TAG, "Read error");
    }
    laser_exit_isr();

//    /* initialization */
//    size_t rx_size = 0;
//
//    /* read */
//    esp_err_t ret = tinyusb_cdcacm_read(itf, buf, CONFIG_USB_CDC_RX_BUFSIZE, &rx_size);
//    if (ret == ESP_OK) {
//        buf[rx_size] = '\0';
//        ESP_LOGI(TAG, "Got data (%d bytes): %s", rx_size, buf);
//    } else {
//        ESP_LOGE(TAG, "Read error");
//    }
//
//    /* write back */
//    tinyusb_cdcacm_write_queue(itf, buf, rx_size);
//    tinyusb_cdcacm_write_flush(itf, 0);
}

void usb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    int dtr = event->line_state_changed_data.dtr;
    int rst = event->line_state_changed_data.rts;
    ESP_LOGI(TAG, "Line state changed! dtr:%d, rst:%d", dtr, rst);
    //printf("Line state changed! dtr:%d, rst:%d", dtr, rst);
}
void usb_cdc_line_coding_changed_callback(int itf, cdcacm_event_t *event)
{
	uint32_t baud = event->line_coding_changed_data.p_line_coding->bit_rate;
	setUsbCDCConnected(1);
}
void usb_SerialInit(void)
{
	usbInit();

//	tinyusb_config_t tusb_cfg = {
//	    .descriptor = NULL,         //Uses default descriptor specified in Menuconfig
//	    .string_descriptor = NULL,  //Uses default string specified in Menuconfig
//	    .external_phy = false,
//	};
//	ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

	usb_msc_spiffs_init();

	tinyusb_config_cdcacm_t amc_cfg = {
		.usb_dev = TINYUSB_USBDEV_0,
		.cdc_port = TINYUSB_CDC_ACM_0,
		.rx_unread_buf_sz = 64,
		.callback_rx = &usb_cdc_rx_callback, // the first way to register a callback
		.callback_rx_wanted_char = NULL,
		.callback_line_state_changed = NULL,
		.callback_line_coding_changed = usb_cdc_line_coding_changed_callback,
	};

	ESP_ERROR_CHECK(tusb_cdc_acm_init(&amc_cfg));
	/* the second way to register a callback */
	ESP_ERROR_CHECK(tinyusb_cdcacm_register_callback(
					TINYUSB_CDC_ACM_0,
					CDC_EVENT_LINE_STATE_CHANGED,
					&usb_cdc_line_state_changed_callback));
	ESP_LOGI(TAG, "USB initialization DONE");
}

uint8_t usbPlugIn = 0;
uint8_t usbCDCConnected = 0;
static stream_rx_buffer_t rxbuf = {0}, rxbackup;
static stream_block_tx_buffer_t txbuf = {0};
static char txdata2[BLOCK_TX_BUFFER_SIZE]; // Secondary TX buffer (for double buffering)
static bool use_tx2data = false;

void usbInit (void)
{
    txbuf.s = txbuf.data;
    txbuf.max_length = BLOCK_TX_BUFFER_SIZE;
}

uint8_t isUsbPlugIn(void)
{
	//return usbCDCConnected;
	return tud_connected()&&!tud_suspended();
}

// Is usb cable plug in
void setUsbPlugIn(uint8_t value)
{
//	usbPlugIn = value;
//	if(usbPlugIn == 0)
//		usbCDCConnected = 0;
}

//Is usb cdc connected
uint8_t isUsbCDCConnected(void)
{
	//return usbPlugIn && usbCDCConnected;
	return usbCDCConnected;
}

void setUsbCDCConnected(uint8_t value)
{
	//printf("setUsbCDCConnected:%d.\r\n",value);
	usbCDCConnected = value;
}

//
// Returns number of free characters in the input buffer
//
uint16_t usbRxFree (void)
{
    uint16_t tail = rxbuf.tail, head = rxbuf.head;
    return RX_BUFFER_SIZE - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the input buffer
//
void usbRxFlush (void)
{
    rxbuf.tail = rxbuf.head;
// Flush output buffer too.
    txbuf.s = use_tx2data ? txdata2 : txbuf.data;
    txbuf.length = 0;
}

//
// Flushes and adds a CAN character to the input buffer
//
void usbRxCancel (void)
{
    rxbuf.data[rxbuf.head] = ASCII_CAN;
    rxbuf.tail = rxbuf.head;
    rxbuf.head = (rxbuf.tail + 1) & (RX_BUFFER_SIZE - 1);
}

//
// Writes current buffer to the USB output stream, swaps buffers
//
static inline bool usb_write (void)
{
    static uint8_t dummy = 0;
    uint32_t start_write_time = HAL_GetTick();
    #define MAX_WRITE_TIME 500 //500ms

    txbuf.s = use_tx2data ? txdata2 : txbuf.data;

    if(txbuf.length == 0) return false;

    /* write back */
    tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, (uint8_t*)txbuf.s, txbuf.length);
    esp_err_t err = tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 10);

    while(isUsbCDCConnected() && err == ESP_FAIL)
    {
    	err = tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 10);

        if(!hal.stream_blocking_callback())
            return false;
        if(!isUsbCDCConnected())
        	return false;
        //Avoid death waiting
        if(start_write_time + MAX_WRITE_TIME < HAL_GetTick())
        {
        	printf("usb_write timeout.\r\n");
        	setUsbCDCConnected(0);
        	break;
        }
    }

    use_tx2data = !use_tx2data;
    txbuf.s = use_tx2data ? txdata2 : txbuf.data;
    txbuf.length = 0;

    return true;
}

//
// Writes a null terminated string to the USB output stream, blocks if buffer full
// Buffers string up to EOL (LF) before transmitting
//

void usbWriteS(const char *s)
{
    size_t length = strlen(s);

    if((length + txbuf.length) > txbuf.max_length) {
        if(!usb_write())
            return;
    }

    memcpy(txbuf.s, s, length);
    txbuf.length += length;
    txbuf.s += length;

    if(s[length - 1] == ASCII_LF) {
        if(!usb_write())
            return;
    }
}

//
// usbGetC - returns -1 if no data available
//
int16_t usbGetC (void)
{
    uint16_t bptr = rxbuf.tail;

    if(bptr == rxbuf.head)
        return -1; // no data available else EOF

    char data = rxbuf.data[bptr++];             // Get next character, increment tmp pointer
    rxbuf.tail = bptr & (RX_BUFFER_SIZE - 1);   // and update pointer

    return (int16_t)data;
}

// "dummy" version of serialGetC
static int16_t usbGetNull (void)
{
    return -1;
}

bool usbSuspendInput (bool suspend)
{
    if(suspend)
        hal.stream.read = usbGetNull;
    else if(rxbuf.backup)
        memcpy(&rxbuf, &rxbackup, sizeof(stream_rx_buffer_t));

    return rxbuf.tail != rxbuf.head;
}

void usbBufferInput (uint8_t *data, uint32_t length)
{
    while(length--) {

        uint_fast16_t next_head = (rxbuf.head + 1)  & (RX_BUFFER_SIZE - 1); // Get and increment buffer pointer

        if(rxbuf.tail == next_head) {                                       // If buffer full
            rxbuf.overflow = 1;                                             // flag overflow
        } else {
            if(*data == CMD_TOOL_ACK && !rxbuf.backup) {

                memcpy(&rxbackup, &rxbuf, sizeof(stream_rx_buffer_t));
                rxbuf.backup = true;
                rxbuf.tail = rxbuf.head;
                hal.stream.read = usbGetC; // restore normal input

            } else if(sys.ready && hal.stream.enqueue_realtime_command && !hal.stream.enqueue_realtime_command(*data)) {        // Check and strip realtime commands,
                rxbuf.data[rxbuf.head] = *data;                             // if not add data to buffer
                rxbuf.head = next_head;                                     // and update pointer
            }
        }
        data++;                                                             // next
    }
}





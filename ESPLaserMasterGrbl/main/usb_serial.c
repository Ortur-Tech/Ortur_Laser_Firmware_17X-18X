/*
 * usb_serial.c
 *
 *  Created on: 2021Äê5ÔÂ20ÈÕ
 *      Author: c
 */


#include "usb_serial.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "esp_log.h"
static const char *TAG = "usb";
static uint8_t buf[CONFIG_USB_CDC_RX_BUFSIZE + 1];

void usb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    /* initialization */
    size_t rx_size = 0;

    /* read */
    esp_err_t ret = tinyusb_cdcacm_read(itf, buf, CONFIG_USB_CDC_RX_BUFSIZE, &rx_size);
    if (ret == ESP_OK) {
        buf[rx_size] = '\0';
        ESP_LOGI(TAG, "Got data (%d bytes): %s", rx_size, buf);
    } else {
        ESP_LOGE(TAG, "Read error");
    }

    /* write back */
    tinyusb_cdcacm_write_queue(itf, buf, rx_size);
    tinyusb_cdcacm_write_flush(itf, 0);
}

void usb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    int dtr = event->line_state_changed_data.dtr;
    int rst = event->line_state_changed_data.rts;
    ESP_LOGI(TAG, "Line state changed! dtr:%d, rst:%d", dtr, rst);
}

void usb_SerialInit(void)
{
	tinyusb_config_t tusb_cfg = {
	    .descriptor = NULL,         //Uses default descriptor specified in Menuconfig
	    .string_descriptor = NULL,  //Uses default string specified in Menuconfig
	    .external_phy = false,
	};
	ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

	tinyusb_config_cdcacm_t amc_cfg = {
		.usb_dev = TINYUSB_USBDEV_0,
		.cdc_port = TINYUSB_CDC_ACM_0,
		.rx_unread_buf_sz = 64,
		.callback_rx = &usb_cdc_rx_callback, // the first way to register a callback
		.callback_rx_wanted_char = NULL,
		.callback_line_state_changed = NULL,
		.callback_line_coding_changed = NULL
	};

	ESP_ERROR_CHECK(tusb_cdc_acm_init(&amc_cfg));
	/* the second way to register a callback */
	ESP_ERROR_CHECK(tinyusb_cdcacm_register_callback(
					TINYUSB_CDC_ACM_0,
					CDC_EVENT_LINE_STATE_CHANGED,
					&usb_cdc_line_state_changed_callback));
	ESP_LOGI(TAG, "USB initialization DONE");
}








/*
 * tusb_msc_disk.h
 *
 *  Created on: 2021年6月2日
 *      Author: c
 */

#ifndef ESP_IDF_COMPONENTS_TINYUSB_ADDITIONS_INCLUDE_TUSB_MSC_DISK_H_
#define ESP_IDF_COMPONENTS_TINYUSB_ADDITIONS_INCLUDE_TUSB_MSC_DISK_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "../../../../app_update/include/esp_ota_ops.h"
#include "../../../../spi_flash/include/esp_partition.h"


#define CFG_EXAMPLE_MSC_READONLY

extern const esp_partition_t *target_partition;;

void app_iap( void );

void usb_msc_spiffs_init(void);










#ifdef __cplusplus
}
#endif



#endif /* ESP_IDF_COMPONENTS_TINYUSB_ADDITIONS_INCLUDE_TUSB_MSC_DISK_H_ */

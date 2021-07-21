/*
 * tusb_msc_disk.c
 *
 *  Created on: 2021年6月2日
 *      Author: c
 */

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "tusb.h"
#include "tinyusb.h"
#include "tusb_msc_disk.h"
#include "../../../../app_update/include/esp_ota_ops.h"
#include "../../../../spi_flash/include/esp_partition.h"
#include "esp_log.h"
#include "board.h"
#include "my_machine_map.h"
#include "driver.h"

#if MACHINE_TYPE == OLM_2_PRO
#if BOARD_VERSION == OLM_ESP_PRO_V1X
#define OTA_FILE_NAME "ESP_OLM2_PRO_"
#elif BOARD_VERSION == OLM_ESP_V1X
#define OTA_FILE_NAME "OLM2_PRO_"
#endif
#endif

//#include "msc_device.h"
#define MSC_OTA		0
#define MSC_SPIFFS	1

const esp_partition_t *target_partition;
uint8_t *msc_disk = NULL;
static bool idf_flash;
static uint32_t _lba = 0;
static long old_millis;
static uint32_t _offset = 0;
uint8_t usb_msc_init_flag = MSC_SPIFFS;

/**
 * Task used as workaround to detect when update has been finished
 */
static void ticker_task(void *p) {
	while (1) {
		if (HAL_GetTick() - old_millis > 1000) {
			//board_led_state(STATE_WRITING_FINISHED);
			esp_err_t err = esp_ota_set_boot_partition(target_partition);
			if (err)
				mprintf(LOG_ERROR, "BOOT ERR => %x [%d]", err, _offset);

			//board_led_state(10);
			esp_restart();
		}
		HAL_Delay(100);
	}
}

// Some MCU doesn't have enough 8KB SRAM to store the whole disk
// We will use Flash as read-only disk with board that has
// CFG_EXAMPLE_MSC_READONLY defined

enum {
	DISK_BLOCK_NUM = 2 * 50, // 8KB is the smallest size that windows allow to mount
	DISK_BLOCK_SIZE = 512
};

//--------------------------------------------------------------------+
// LUN 0
//--------------------------------------------------------------------+
#define README0_CONTENTS "Copy ORTUR Laser Master Firmware to here.More info to visit www.ortur.net "

#ifdef CFG_EXAMPLE_MSC_READONLY
const
#endif
uint8_t msc_disk0[DISK_BLOCK_NUM][DISK_BLOCK_SIZE] = {
//------------- Block0: Boot Sector -------------//
// byte_per_sector    = DISK_BLOCK_SIZE; fat12_sector_num_16  = DISK_BLOCK_NUM;
// sector_per_cluster = 1; reserved_sectors = 1;
// fat_num            = 1; fat12_root_entry_num = 16;
// sector_per_fat     = 1; sector_per_track = 1; head_num = 1; hidden_sectors = 0;
// drive_number       = 0x80; media_type = 0xf8; extended_boot_signature = 0x29;
// filesystem_type    = "FAT12   "; volume_serial_number = 0x1234; volume_label = "TinyUSB MSC";
// FAT magic code at offset 510-511
		{ 0xEB, 0x3C, 0x90, 0x4D, 0x53, 0x44, 0x4F, 0x53, 0x35, 0x2E, 0x30,
				0x00, 0x02, 0x10, 0x01, 0x00, 0x01, 0x10, 0x00, 0x10, 0x00,
				0xF8, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x29, 0x34, 0x12,
				0x56, 0x78, 'O', 'R', 'T', 'U', 'R', ' ', 'L', 'a', 's', 'e',
				'r', 0x46, 0x41, 0x54, 0x31, 0x32, 0x20, 0x20, 0x20, 0x00, 0x00,

				// Zero up to 2 last bytes of FAT magic code
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x00, 0x00, 0x55, 0xAA },

		//------------- Block1: FAT12 Table -------------//
		{ 0xF8, 0xFF, 0xFF, 0xFF, 0x0F // // first 2 entries must be F8FF, third entry is cluster end of readme file
		},

		//------------- Block2: Root Directory -------------//
		{
				// first entry is volume label
				'O', 'R', 'T', 'U', 'R', ' ', 'L', 'a', 's', 'e', 'r', 0x08,
				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				0x4F, 0x6D, 0x65, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
				// second entry is readme file
				'R', 'E', 'A', 'D', 'M', 'E', ' ', ' ', 'T', 'X', 'T', 0x20,
				0x00, 0xC6, 0x52, 0x6D, 0x65, 0x43, 0x65, 0x43, 0x00, 0x00,
				0x88, 0x6D, 0x65, 0x43,                       // date and time
				0x02, 0x00,                                   // sector
				sizeof(README0_CONTENTS) - 1, 0x00, 0x00, 0x00 // readme's files size (4 Bytes)
		},

		//------------- Block3: Readme Content -------------//
		README0_CONTENTS };

void check_running_partition(void) {
	const esp_partition_t *_part_ota0 = esp_ota_get_running_partition();

	mprintf(LOG_INFO, "TYPE => %d", _part_ota0->type);
	mprintf(LOG_INFO, "SUBTYPE => %02x", _part_ota0->subtype);
	mprintf(LOG_INFO, "ADDRESS => %x", _part_ota0->address);
	mprintf(LOG_INFO, "SIZE => %x", _part_ota0->size);
	mprintf(LOG_INFO, "label => %s", _part_ota0->label);
}

/*
 *
 */
/*0:ota 盘 1:siffs盘*/
bool init_disk(uint8_t pan) {
	if (pan == MSC_OTA) {
		esp_partition_t *current_partition;
		check_running_partition();
		current_partition = esp_ota_get_running_partition();
		if (memcmp(current_partition->label, "ota_0", 5) == 0) {
			target_partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP,
					ESP_PARTITION_SUBTYPE_ANY, "ota_1");
		} else {
			target_partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP,
					ESP_PARTITION_SUBTYPE_ANY, "ota_0");
		}
		msc_disk = (uint8_t*) heap_caps_calloc(1,
				DISK_BLOCK_SIZE * DISK_BLOCK_NUM, MALLOC_CAP_32BIT);
		if (msc_disk == NULL)
			return false;
		memcpy(msc_disk, msc_disk0, sizeof(msc_disk0));
		msc_disk[20] =
				(uint8_t) (target_partition->size / DISK_BLOCK_SIZE >> 8);
		msc_disk[19] = (uint8_t) (target_partition->size / DISK_BLOCK_SIZE
				& 0xff);

	} else {
		target_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
				ESP_PARTITION_SUBTYPE_ANY, "storage");
//		msc_disk = (uint8_t*)heap_caps_calloc(1, DISK_BLOCK_SIZE * DISK_BLOCK_NUM, MALLOC_CAP_32BIT);
//		if(msc_disk == NULL) return false;
//		memcpy(msc_disk, msc_disk0, sizeof(msc_disk0));
//		msc_disk[20] = (uint8_t)(target_partition->size / DISK_BLOCK_SIZE >> 8);
//		msc_disk[19] = (uint8_t)(target_partition->size / DISK_BLOCK_SIZE & 0xff);
	}

	usb_msc_init_flag = pan;
	//board_led_state(STATE_BOOTLOADER_STARTED);
	return true;
}

// Invoked to determine max LUN
uint8_t tud_msc_get_maxlun_cb(void) {
	return 1; // dual LUN
}

// Invoked when received SCSI_CMD_INQUIRY
// Application fill vendor id, product id and revision with string up to 8, 16, 4 characters respectively
void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8],
		uint8_t product_id[16], uint8_t product_rev[4]) {

	(void) lun; // use same ID for both LUNs

	const char vid[] = "ORTUR_USB";
	const char pid[] = "Mass Storage";
	const char rev[] = "1.0";

	memcpy(vendor_id, vid, strlen(vid));
	memcpy(product_id, pid, strlen(pid));
	memcpy(product_rev, rev, strlen(rev));
}

// Invoked when received Test Unit Ready command.
// return true allowing host to read/write this LUN e.g SD card inserted
bool tud_msc_test_unit_ready_cb(uint8_t lun) {

	(void) lun;

	return true; // RAM disk is always ready
}

// Invoked when received SCSI_CMD_READ_CAPACITY_10 and SCSI_CMD_READ_FORMAT_CAPACITY to determine the disk size
// Application update block count and block size
void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count,
		uint16_t *block_size) {

#ifndef CFG_EXAMPLE_MSC_READONLY
  (void) lun;

  *block_count = DISK_BLOCK_NUM;
  *block_size  = DISK_BLOCK_SIZE;
#else
	if (usb_msc_init_flag == MSC_OTA) {
		(void) lun;
		if (target_partition == NULL) {
			*block_count = DISK_BLOCK_NUM;
			*block_size = DISK_BLOCK_SIZE;
		} else {
			*block_count = target_partition->size / DISK_BLOCK_SIZE;
			*block_size = DISK_BLOCK_SIZE;
			mprintf(LOG_INFO, "block_count:%x block_size:%x.\r\n", *block_count,
					*block_size);
		}
	} else {
		*block_count = 0;
		*block_size = 0;
	}

#endif
}

// Invoked when received Start Stop Unit command
// - Start = 0 : stopped power mode, if load_eject = 1 : unload disk storage
// - Start = 1 : active mode, if load_eject = 1 : load disk storage
bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start,
		bool load_eject) {

	(void) lun;
	(void) power_condition;

	if (load_eject) {
		if (start) {
			// load disk storage
		} else {
			// unload disk storage
		}
	}

	return true;
}

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and return number of copied bytes.
int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
		void *buffer, uint32_t bufsize) {
	if (usb_msc_init_flag != MSC_OTA)
		return bufsize;

#ifndef CFG_EXAMPLE_MSC_READONLY
  uint8_t const* addr = (lun ? msc_disk1[lba] : msc_disk0[lba]) + offset;
  memcpy(buffer, addr, bufsize);
#else
	(void) lun;
	if (target_partition == NULL || lba < 50) // first 50 lba is RAM disk, 50+ its ota partition
			{
		uint8_t *addr = &msc_disk[lba * 512] + offset;
		memcpy(buffer, addr, bufsize);

	} else {
		esp_partition_read(target_partition, (lba * 512) + offset - 512, buffer,
				bufsize);
	}
#endif
	return bufsize;
}
typedef __attribute__((packed))  struct {
	uint8_t DIR_Name[11];          // File Name
	uint8_t DIR_Attr;              // File Attribute
	uint8_t DIR_NTRes;             // Reserved
	uint8_t DIR_CreateTime_Tenth;  // Component of the file creation time
	uint16_t DIR_CreateTime;        // Component of the file creation time
	uint16_t DIR_CreateDate;        // Component of the file creation date
	uint16_t DIR_LastAccessDate;    // Last Access date
	uint16_t DIR_ClusHigh;          // High word of first data cluster
	uint16_t DIR_WriteTime;         // Last modification write time
	uint16_t DIR_WriteDate;         // Last modification write date
	uint16_t DIR_ClusLow;           // Low word of first data cluster
	uint32_t DIR_FileSize;          // Filesize
} FAT_DIR_t;

uint8_t longFileName[250] = {0};
uint8_t nameLen = 0;
uint8_t fileNameMatchFlag = 0;
/*获取文件名*/
uint8_t get_long_file_name(FAT_DIR_t *pFile)
{
	FAT_DIR_t* p = pFile;
	uint8_t name[32] = {0};
	uint32_t i = 0;
	uint8_t* ptr;

	nameLen = 0;
	do{
		p--;
		ptr = (uint8_t*)p;
		memcpy(name,&ptr[1],10);
		memcpy(&name[10],&ptr[14],12);
		memcpy(&name[22],&ptr[28],4);

		mprintf(LOG_TEMP,"NAME:");
		for(i = 0; i < 26; i++)
		{
			mprintf(LOG_TEMP,"%02x",name[i]);
		}
		for(i = 0; i < 26; i+=2)
		{
//			if(((name[i] <= 'Z') && (name[i] >= 'A')) ||
//					((name[i] <= 'z') && (name[i] >= 'a')) ||
//					((name[i] <= '9') && (name[i] >= '0')) ||(name[i] == '.'))
			if(name[i] != 0)
			{
				longFileName[nameLen++] = name[i];
				if(nameLen > 100)
				{
					mprintf(LOG_TEMP,"FILE NAME:%.*s.\r\n",nameLen,longFileName);
					return nameLen;
				}
			}
			else
			{
				mprintf(LOG_TEMP,"FILE NAME:%.*s.\r\n",nameLen,longFileName);
				return nameLen;
			}
		}

	}while(!(ptr[0] & (1 << 6)));//是否是最后一个目录项
	mprintf(LOG_TEMP,"FILE NAME:%.*s.\r\n",nameLen,longFileName);
	return nameLen;
}

FAT_DIR_t FileAttr = { .DIR_FileSize = 0, };

uint32_t FAT_RootDirWriteRequest(uint32_t FAT_LBA, uint32_t offset, uint8_t *data, uint32_t len) {
	FAT_DIR_t *pFile = (FAT_DIR_t*) data;
	uint32_t index = 2;

	pFile++; // Skip Root Dir
	pFile++; // Skip Status File

	while ((pFile->DIR_Attr != 0x20) && (index++ < 512)) {
		pFile++;
	}

	// Find it
	if (index <= 512)
	{
#if DEBUG_LEVEL
    	mprintf(LOG_INFO,"-----------------------------------------------\r\n");
    	Usart_SendData(data,len);
#endif
		memcpy(&FileAttr, pFile, 32);
		FileAttr.DIR_WriteTime = 0;
		FileAttr.DIR_WriteDate = 0;
		mprintf(LOG_TEMP,"file name:%s.size:%d.\r\n", FileAttr.DIR_Name, FileAttr.DIR_FileSize);

		if(memcmp(FileAttr.DIR_Name,OTA_FILE_NAME,strlen(OTA_FILE_NAME)) == 0)
		{
			/*固件名匹配*/
			fileNameMatchFlag = 1;
		}
		if(get_long_file_name(pFile))
		{
			if(memcmp(longFileName,OTA_FILE_NAME,strlen(OTA_FILE_NAME)) == 0)
			{
				mprintf(LOG_TEMP,"update file found.\r\n");
				/*固件名匹配*/
				fileNameMatchFlag = 1;
			}
			else
			{
				mprintf(LOG_TEMP,"not update file.\r\n");
			}
		}

	}
	return len;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and return number of written bytes
int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
		uint8_t *buffer, uint32_t bufsize) {
	if (usb_msc_init_flag != MSC_OTA)
		return bufsize;
#ifndef CFG_EXAMPLE_MSC_READONLY
  uint8_t* addr = (lun ? msc_disk1[lba] : msc_disk0[lba])  + offset;
  memcpy(addr, buffer, bufsize);
#else
	esp_err_t err = 0;
	(void) lun;
	if (buffer[0] == 0xe9 && !idf_flash && fileNameMatchFlag == 1) { // we presume that we are having beginning of esp32 binary file when we see magic number at beginning of buffer
		//ESP_LOGE("", "start flash");
		//
		//board_led_state(STATE_WRITING_STARTED);

		idf_flash = true;
		_lba = lba;
		esp_partition_erase_range(target_partition, 0x0,
				target_partition->size);
		old_millis = HAL_GetTick();
		xTaskCreate(ticker_task, "tT", 3 * 1024, NULL, 1, NULL);
	}

	if (!idf_flash) {
		FAT_RootDirWriteRequest(lba, offset, buffer, bufsize);
		mprintf(LOG_TEMP, "write lba:%d offset:%d,len:%d.\r\n", lba, offset, bufsize);
		for (int i = 0; i < bufsize; i++) {
			mprintf(LOG_TEMP, "%02x ", buffer[i]);
		}
		uint8_t *addr = &msc_disk[lba * 512] + offset;
		memcpy(addr, buffer, bufsize);
	} else {
		if (lba < _lba) {
			// ignore LBA that is lower than start update LBA, it is most likely FAT update
			return bufsize;
		}
		mprintf(LOG_TEMP, "write offset:%d,len:%d.to flash.\r\n", _offset,
				bufsize);
		power_LedToggle();
		comm_LedToggle();
		err = esp_partition_write(target_partition, _offset, buffer, bufsize);
		_offset += bufsize;
	}
	old_millis = HAL_GetTick();

#endif

	return bufsize;
}
#if CONFIG_USB_MSC_ENABLED
// Callback invoked when received an SCSI command not in built-in list below
// - READ_CAPACITY10, READ_FORMAT_CAPACITY, INQUIRY, MODE_SENSE6, REQUEST_SENSE
// - READ10 and WRITE10 has their own callbacks
int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void *buffer,
		uint16_t bufsize) {
	if (usb_msc_init_flag != MSC_OTA)
		return 0;
	// read10 & write10 has their own callback and MUST not be handled here

	void const *response = NULL;
	uint16_t resplen = 0;

	// most scsi handled is input
	bool in_xfer = true;

	switch (scsi_cmd[0]) {
	case SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
		// Host is about to read/write etc ... better not to disconnect disk
		resplen = 0;
		break;

	case SCSI_CMD_START_STOP_UNIT:
		// Host try to eject/safe remove/poweroff us. We could safely disconnect with disk storage, or go into lower power
		/* scsi_start_stop_unit_t const * start_stop = (scsi_start_stop_unit_t const *) scsi_cmd;
		 // Start bit = 0 : low power mode, if load_eject = 1 : unmount disk storage as well
		 // Start bit = 1 : Ready mode, if load_eject = 1 : mount disk storage
		 start_stop->start;
		 start_stop->load_eject;
		 */
		resplen = 0;
		break;

	default:
		// Set Sense = Invalid Command Operation
		tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);

		// negative means error -> tinyusb could stall and/or response with failed status
		resplen = -1;
		break;
	}

	// return resplen must not larger than bufsize
	if (resplen > bufsize)
		resplen = bufsize;

	if (response && (resplen > 0)) {
		if (in_xfer) {
			memcpy(buffer, response, resplen);
		} else {
			// SCSI output
		}
	}

	return resplen;
}

#endif

int key_Status(void) {
	if (power_KeyDown()) {
		vTaskDelay(10 / portTICK_PERIOD_MS);
		if (power_KeyDown()) {
			return 0;
		}
	}
	return 1;
}

#define LED_FLASH_TIME 350 //MS

void led_flash(void *pvParameters)
{
	while(1)
	{
		power_LedToggle();
		comm_LedToggle();
		vTaskDelay(LED_FLASH_TIME / portTICK_PERIOD_MS);
	}
}

static uint8_t led_ucParameter;
TaskHandle_t ledflash_TaskHandle = NULL;

void usb_MscTask(void *pvParameters) {
	tinyusb_config_t tusb_cfg = { .descriptor = NULL, //Uses default descriptor specified in Menuconfig
			.string_descriptor = NULL, //Uses default string specified in Menuconfig
			.external_phy = false, };
	msc_cdc_separate_init(!INIT_CDC_ONLY);
	ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
	tusb_init();
	xTaskCreate( led_flash, "led_flash", 512, &led_ucParameter, 1, &ledflash_TaskHandle );
	mprintf(LOG_TEMP, "creat usb msc task.\r\n");
	while (1)
	{
		tud_task(); // tinyusb device task
		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
}

#define ENABLE_OTA_KEY_COMPATIBLE_MODE 1

void ota_key_init(void)
{
#if ENABLE_OTA_KEY_COMPATIBLE_MODE
	gpio_config_t gpioConfig = {
			.pin_bit_mask = ((uint64_t)1 << GPIO_NUM_46) | ((uint64_t)1 << GPIO_NUM_41),
			.mode = GPIO_MODE_INPUT,
			.pull_up_en = GPIO_PULLUP_ENABLE,
			.pull_down_en = GPIO_PULLDOWN_DISABLE,
			.intr_type = GPIO_INTR_DISABLE
		};
	gpio_config(&gpioConfig);
#else
	power_KeyInit();
#endif
}
/*0:按下 1：没按下*/
uint8_t ota_key_status(void)
{
#if ENABLE_OTA_KEY_COMPATIBLE_MODE
	if ((gpio_get_level(GPIO_NUM_46) == 1) || (gpio_get_level(GPIO_NUM_41) == 0))
	{
		vTaskDelay(10 / portTICK_PERIOD_MS);
		if ((gpio_get_level(GPIO_NUM_46) == 1) || (gpio_get_level(GPIO_NUM_41) == 0))
		{
			return 0;
		}
	}
	return 1;
#else
	return key_Status();
#endif
}

static uint8_t ucParameter;
TaskHandle_t usbMscTaskHandle = NULL;

void app_iap(void)
{
	ota_key_init();
	if (ota_key_status() == 0)
	{
		if (init_disk(MSC_OTA) == true)
		{
			led_Init();
			usb_MscTask(NULL);
		}
	}
	led_Init();
}

void usb_msc_spiffs_init(void) {
	tinyusb_config_t tusb_cfg = { .descriptor = NULL, //Uses default descriptor specified in Menuconfig
			.string_descriptor = NULL, //Uses default string specified in Menuconfig
			.external_phy = false, };

	ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
//	if(init_disk(MSC_SPIFFS) == true)
//	{
//		 mprintf(LOG_TEMP,"creat usb msc app task.\r\n");
//		/*创建一个任务让msc正常跑*/
//		xTaskCreate( usb_MscTask, "usb_MscTask", 2048, &ucParameter, 2, &usbMscTaskHandle );
//		configASSERT( usbMscTaskHandle );
//	}
}


/*
 * mac_encrypt.c
 *
 *  Created on: 2021年12月22日
 *      Author: c
 */
#include "mbedtls/md5.h"
#include "mbedtls/aes.h"
#include "mac_encrypt.h"
#include "string.h"
#include "driver.h"
#include "board.h"

/*密钥组合 = 机器型号 + 机器版本 + 发售年月，不能超过16个字节，不足16字节的补0*/

#if MACHINE_TYPE == AUFERO_1
#define MACHINE_MODE_ABBR 		"AL1"
#define MACHINE_VERSION_ABBR	"01"
#define MACHINE_YEAR			21
#define MACHINE_MONTH			12

#elif MACHINE_TYPE == AUFERO_2
#define MACHINE_MODE_ABBR 		"AL2"
#define MACHINE_VERSION_ABBR	"01"
#define MACHINE_YEAR			21
#define MACHINE_MONTH			12

#elif MACHINE_TYPE == OLM2_PRO_S1
#define MACHINE_MODE_ABBR 		"OLM2PRO"
#define MACHINE_VERSION_ABBR	"01"
#define MACHINE_YEAR			21
#define MACHINE_MONTH			8

#elif MACHINE_TYPE == OLM2_PRO_S2
#define MACHINE_MODE_ABBR 		"OLM2PRO"
#define MACHINE_VERSION_ABBR	"02"
#define MACHINE_YEAR			21
#define MACHINE_MONTH			10
#elif  MACHINE_TYPE == OLM2_PRO_S2_MAX
#define MACHINE_MODE_ABBR 		"OLM2PRO MAX"
#define MACHINE_VERSION_ABBR	"03"
#define MACHINE_YEAR			22
#define MACHINE_MONTH			03
#elif MACHINE_TYPE == OLM2_S2
#define MACHINE_MODE_ABBR 		"OLM2"
#define MACHINE_VERSION_ABBR	"02"
#define MACHINE_YEAR			21
#define MACHINE_MONTH			11

#elif MACHINE_TYPE == OLM2
#define MACHINE_MODE_ABBR 		"OLM2"
#define MACHINE_VERSION_ABBR	"01"
#define MACHINE_YEAR			21
#define MACHINE_MONTH			10
#else
#error "machine type error!!!"
#endif



#define AES_KEY_BITS 128
/*每个字节所在的位置0 ~ 15每个值必须不同*/
uint8_t mac_byte_index[6] = {7, 5, 2, 10, 4, 15};
uint8_t aes_key[20] = "ortur laser master";

/*密钥规则*/
int mac_aes_key_init(void)
{
	int i = 0;
	memcpy(&aes_key[i], MACHINE_MODE_ABBR, strlen(MACHINE_MODE_ABBR));
	i = i + strlen(MACHINE_MODE_ABBR);
	memcpy(&aes_key[i], MACHINE_VERSION_ABBR, strlen(MACHINE_VERSION_ABBR));
	i = i + strlen(MACHINE_VERSION_ABBR);
	aes_key[i++] =  MACHINE_YEAR;
	aes_key[i++] =  MACHINE_MONTH;
	for(; i < 16; i++)
	{
		aes_key[i] = 0;
	}
	return  i;
}

int mac_aes_encrypt( unsigned char data[16], unsigned char aes_data[16])
{
	int ret = 0;
	mbedtls_aes_context ctx;
	mbedtls_aes_init(&ctx);
	ret = mbedtls_aes_setkey_enc( &ctx, aes_key, AES_KEY_BITS );

	ret = mbedtls_aes_crypt_ecb( &ctx, 1, data, aes_data );
	return ret;
}
int mac_aes_decrypt( unsigned char data[16], unsigned char aes_data[16])
{
	int ret = 0;
	mbedtls_aes_context ctx;
	mbedtls_aes_init(&ctx);
	ret = mbedtls_aes_setkey_dec( &ctx, aes_key, AES_KEY_BITS );
	ret = mbedtls_aes_crypt_ecb( &ctx, 0, aes_data, data);
	return ret;
}
int mac_encrypt(unsigned char aes_data[16])
{
	int ret = 0;
	unsigned char mac[6] = {0};
	unsigned char mac_data[16] = {0};
	memset(mac_data, 0, 16);
	memset(aes_data, 0, 16);
	mac_aes_key_init();
	/*获取mac*/
	esp_read_mac(mac, ESP_MAC_WIFI_STA);
	//printf("mac_encrypt:%02X,%02X,%02X,%02X,%02X,%02X.\r\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	/*md5加密一次*/
	mbedtls_md5_ret(mac, 6, mac_data);
	/*把数据按规则放入到数组内*/
	mac_data[mac_byte_index[0]] = mac[0];
	mac_data[mac_byte_index[1]] = mac[1];
	mac_data[mac_byte_index[2]] = mac[2];
	mac_data[mac_byte_index[3]] = mac[3];
	mac_data[mac_byte_index[4]] = mac[4];
	mac_data[mac_byte_index[5]] = mac[5];
	/*进行AES加密*/
	ret = mac_aes_encrypt(mac_data, aes_data);
	return ret;
}

int mac_decrypt(unsigned char mac[6],unsigned char aes_data[16])
{
	int ret = 0;
	unsigned char mac_data[16] = {0};
	memset(mac_data, 0, 16);
	mac_aes_key_init();
	/*解密*/
	ret = mac_aes_decrypt(mac_data, aes_data);
	/*把mac数据按规则取出*/
	 mac[0]= mac_data[mac_byte_index[0]];
	 mac[1]= mac_data[mac_byte_index[1]];
	 mac[2]= mac_data[mac_byte_index[2]];
	 mac[3]= mac_data[mac_byte_index[3]];
	 mac[4]= mac_data[mac_byte_index[4]];
	 mac[5]= mac_data[mac_byte_index[5]];

	 //printf("mac_decrypt:%02X,%02X,%02X,%02X,%02X,%02X.\r\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	 return ret;
}

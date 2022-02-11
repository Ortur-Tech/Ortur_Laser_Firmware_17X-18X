/*
 * mac_encrypt.h
 *
 *  Created on: 2021年12月22日
 *      Author: c
 */

#ifndef MAIN_MAC_ENCRYPT_H_
#define MAIN_MAC_ENCRYPT_H_

int mac_encrypt(unsigned char aes_data[16]);
int mac_decrypt(unsigned char mac[6],unsigned char aes_data[16]);

#endif /* MAIN_MAC_ENCRYPT_H_ */

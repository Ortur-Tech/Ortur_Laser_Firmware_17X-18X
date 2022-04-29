#include "wireless_powerstrip.h"
#include "esp_log.h"
#include "esp_attr.h"
#include <stdlib.h>
#include <string.h>
#include "driver/rmt.h"
#include "grbl/settings.h"
#include "grbl/hal.h"
#include "stdio.h"

#if WIRELESS_STRIP_ENABLE


#define STRIP_XE_338	0xa0002000
#define STRIP_HE_SEN	0xa0002001

#define CURRENT_STRIP_NUM	STRIP_XE_338

//#if (CURRENT_STRIP_NUM==STRIP_XE_338)
static const strip_code_t control_code2[] ={
		{3, 	8},	//the first switch code = {on,  off}
		{5, 	6}, //the second switch code = {on,  off}
		{9, 	4}, //the 3th switch code = {on,  off}
		{1, 	2}, //the 4th switch code = {on,  off}
};
//#elif (CURRENT_STRIP_NUM==STRIP_HE_SEN)
static const strip_code_t control_code1[] ={
		{3, 	3},	//the first switch code = {on,  off}
		{1, 	1}, //the second switch code = {on,  off}
		{2, 	2}, //the 3th switch code = {on,  off}
		{9, 	9}, //the 4th switch code = {on,  off}
		{4, 	4}, //the 4th switch code = {on,  off}
};
//#endif

#define WIRELESS_STRIP_DEBUG


#define STRIP_RMT_TX_CHANNEL	RMT_CHANNEL_3

#if WIRELESS_STRIP_PIN < 32
	#define GPIO_SET_JACK_HIGH(gpio_num) GPIO.out_w1ts = ((uint32_t)1 << gpio_num);
	#define GPIO_SET_JACK_LOW(gpio_num) GPIO.out_w1tc = ((uint32_t)1 << gpio_num);
#else
	#define GPIO_SET_JACK_HIGH(gpio_num) GPIO.out1_w1ts.data = ((uint32_t)1 << (gpio_num - 32));
	#define GPIO_SET_JACK_LOW(gpio_num) GPIO.out1_w1tc.data = ((uint32_t)1 << (gpio_num - 32));
#endif

rmt_private_data_t	rmt_data;
cmd_flag_t	cmd_flag;

static void strip_rmt_tx_init(void);
static void strip_generate_random_addr(void);

/*
 * brief:Initialize strip
 */
void wireless_strip_init(void)
{
	memset(&cmd_flag, 0, sizeof(cmd_flag_t));
	memset(&rmt_data, 0, sizeof(rmt_private_data_t));

	strip_rmt_tx_init();
    rmt_get_counter_clock(STRIP_RMT_TX_CHANNEL, &rmt_data.counter_clk_hz);

    // us -> ticks
    float ticks_per_us = (float)rmt_data.counter_clk_hz / 1e6;
    rmt_data.h_1_ticks = ticks_per_us * TRUE_H_DELAY_US;
    rmt_data.l_1_ticks = ticks_per_us * TRUE_L_DELAY_US;
    rmt_data.h_0_ticks = ticks_per_us * FALSE_H_DELAY_US;
    rmt_data.l_0_ticks = ticks_per_us * FALSE_L_DELAY_US;

    strip_generate_random_addr();
}

static void strip_generate_random_addr(void)
{
	if(settings.random_addr==0)
	{
		settings.random_addr = esp_random() & 0x000FFFFF;

#ifdef WIRELESS_STRIP_DEBUG
	    uint8_t rebuff[50]={0};
	    sprintf((char *)rebuff, "random=%#x. \n", settings.random_addr);
	    hal.stream.write_all((char *)rebuff);
#endif
		write_global_settings();
	}
	rmt_data.addr = settings.random_addr;
}

/*
 * brief:控制某个插孔的开关
 * param:插孔编号 1, 2, 3, 4...
 * param:bool值，开=1 关=0
 */
void execute_cmd_task(void)
{
	if(cmd_flag.cmd!=0)
	{
		switch(cmd_flag.cmd)
		{
			case STRIP_CMD_M20:
			{
				key_control(KEY_NUM_1, true);
			}
			break;
			case STRIP_CMD_M21:
			{
				key_control(KEY_NUM_1, false);
			}
			break;
			case STRIP_CMD_M22:
			{
				key_control(KEY_NUM_2, true);
			}
			break;
			case STRIP_CMD_M23:
			{
				key_control(KEY_NUM_2, false);
			}
			break;
			case STRIP_CMD_M24:
			{
				key_control(KEY_NUM_3, true);
			}
			break;
			case STRIP_CMD_M25:
			{
				key_control(KEY_NUM_3, false);
			}
			break;
			case STRIP_CMD_M26:
			{
				key_control(KEY_NUM_4, true);
			}
			break;
			case STRIP_CMD_M27:
			{
				key_control(KEY_NUM_4, false);
			}
			break;
			case STRIP_CMD_M28:
			{
				control_pairing();
			}
			break;
			case STRIP_CMD_DIY_CMD:
			{
				send_cmd(settings.strip.addr, cmd_flag.key);
			}
			break;
		}
		memset(&cmd_flag, 0, sizeof(cmd_flag_t));
	}
}


/*
 * brief:控制某个插孔的开关
 * param:插孔编号 1, 2, 3, 4...
 * param:bool值，开=1 关=0
 */
void key_control(uint8_t key_num, uint8_t value)
{
	strip_code_t *ptr;
	uint8_t get_key=0;
	if(key_num<1||key_num>10)
		return;
	switch(settings.strip.type)
	{
		case 0:
			ptr = control_code1;
			if(key_num>5)
				return;
		break;
		case 1:
			ptr = control_code2;
			if(key_num>4)
				return;
		break;
		default:
			ptr = control_code1;
			if(key_num>5)
				return;
		break;
	}
	get_key = (value==1?ptr[key_num-1].on:ptr[key_num-1].off);
	send_cmd(settings.strip.addr, get_key);
}

/*
 * brief:控制排插对码
 */
void control_pairing(void)
{
	strip_code_t *ptr;
	uint8_t i;
	uint8_t get_key1=0,get_key2=0,get_key3=0,get_key4=0,get_key5=0;
	switch(settings.strip.type)
	{
		case 0:
		{
			ptr = control_code1;
			get_key1 = ptr[0].on;
			get_key5 = ptr[4].on;
			for(i=0;i<PAIRING_REPEAT_TIMES;i++)
			{
				send_cmd(settings.strip.addr, get_key1);
				vTaskDelay(pdMS_TO_TICKS(500));
			}
			send_cmd(settings.strip.addr, get_key5);
		}
		break;
		case 1:
			ptr = control_code2;
			get_key1 = ptr[0].on;
			for(i=0;i<PAIRING_REPEAT_TIMES;i++)
			{
				send_cmd(settings.strip.addr, get_key1);
				vTaskDelay(pdMS_TO_TICKS(500));
			}
		break;
		default:
			ptr = control_code1;
		break;
	}
}

/*
 * brief:发送遥控码值 且重复发4遍
 * param:地址 有效位为低20bit
 * param:key值 有效位为低4bit
 */
void send_cmd(uint32_t addr, uint8_t key)
{
	if(addr==0x11111)
	{
		addr = rmt_data.addr;
	}
	uint32_t cmd = ((addr<<4)&0xFFFFFFF0) + key;
	uint8_t buff[TX_BUFFER_MAX_LEN]={0};
	uint8_t i,len;
	buff[0]=(cmd>>16)&0xff;
	buff[1]=(cmd>>8)&0xff;
	buff[2]=cmd&0xff;
	len=3;

#ifdef WIRELESS_STRIP_DEBUG
    uint8_t rebuff[50]={0};
    sprintf((char *)rebuff, "cmd=%#x %#x %#x. \n", buff[0], buff[1], buff[2]);
    hal.stream.write_all((char *)rebuff);
#endif

//	for(i=0; i<SEND_REPEAT_NUM; i++)
	{
		strip_fill_buffer_send(buff, len, TX_WAIT_DONE_MS);
//		vTaskDelay(pdMS_TO_TICKS(TX_PERIOD_INVERT_MS));
	}
}



/**
 * @brief Conver data to RMT format.
 *
 * @param[in] src: source data, to converted to RMT format
 * @param[in] dest: place where to store the convert result
 * @param[in] src_size: size of source data
 * @param[in] wanted_num: number of RMT items that want to get
 * @param[out] translated_size: number of source data that got converted
 * @param[out] item_num: number of RMT items which are converted from source data
 */
static void IRAM_ATTR data_rmt_adapter(const void *src, rmt_item32_t *dest, size_t src_size,
        size_t wanted_num, size_t *translated_size, size_t *item_num)
{
    if (src == NULL || dest == NULL) {
        *translated_size = 0;
        *item_num = 0;
        return;
    }
    const rmt_item32_t bit0 = {{{ rmt_data.h_0_ticks, 1, rmt_data.l_0_ticks, 0 }}}; //Logical 0
    const rmt_item32_t bit1 = {{{ rmt_data.h_1_ticks, 1, rmt_data.l_1_ticks, 0 }}}; //Logical 1
    size_t size = 0;
    size_t num = 0;
    uint8_t *psrc = (uint8_t *)src;
    rmt_item32_t *pdest = dest;
    while (size < src_size && num < wanted_num) {
        for (int i = 0; i < 8; i++) {
            // MSB first
            if (*psrc & (1 << (7 - i))) {
                pdest->val =  bit1.val;
            } else {
                pdest->val =  bit0.val;
            }
            num++;
            pdest++;
        }
        size++;
        psrc++;
    }
    *translated_size = size;
    *item_num = num;
}

/*
 * Initialize the RMT Tx channel
 */
static void strip_rmt_tx_init(void)
{
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(WIRELESS_STRIP_PIN, STRIP_RMT_TX_CHANNEL);
    config.clk_div =80;
    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

//    STRIP_CHECK(rmt_get_counter_clock(config.channel, &rmt_data.counter_clk_hz) == ESP_OK,
//            "get rmt counter clock failed");

    // set 433mhz data to rmt adapter
    rmt_translator_init(config.channel, data_rmt_adapter);
}

/*
 * clear tx_buffer, fill buffer, send buffer
 */
esp_err_t strip_fill_buffer_send(uint8_t *buff, uint8_t len, uint32_t timeout_ms)
{
#if 1
	uint8_t index,i ;
	if(buff==NULL||len<=0)
		return ESP_FAIL;
	const rmt_item32_t sync_data= {{{250,1,7750,0}}};
     //Write zero to turn off all leds
    memset(rmt_data.tx_buff, 0, TX_BUFFER_MAX_LEN);
    for(index=0;index<len;index++)
    {
    	rmt_data.tx_buff[index]=buff[index];
    }
    rmt_data.buff_len=len;
    for(i=0; i<SEND_REPEAT_NUM; i++)
    {
		rmt_write_items(STRIP_RMT_TX_CHANNEL, &sync_data, 1, true);
		rmt_write_sample(STRIP_RMT_TX_CHANNEL, rmt_data.tx_buff, rmt_data.buff_len, true);
		rmt_wait_tx_done(STRIP_RMT_TX_CHANNEL, pdMS_TO_TICKS(timeout_ms));
    }
	rmt_write_items(STRIP_RMT_TX_CHANNEL, &sync_data, 1, true);
    return 0;
#else
	rmt_get_counter_clock(STRIP_RMT_TX_CHANNEL, &rmt_data.counter_clk_hz);
    uint8_t rebuff[50]={0};
    sprintf((char *)rebuff, "clk=%d. \n", rmt_data.counter_clk_hz);
    hal.stream.write_all((char *)rebuff);
	rmt_item32_t txbuff[26]={
			{{{250,1,7750,0}}},

			{{{750,1,250,0}}},
			{{{750,1,250,0}}},
			{{{250,1,750,0}}},
			{{{750,1,250,0}}},

			{{{250,1,750,0}}},
			{{{750,1,250,0}}},
			{{{750,1,250,0}}},
			{{{250,1,750,0}}},

			{{{750,1,250,0}}},
			{{{750,1,250,0}}},
			{{{250,1,750,0}}},
			{{{750,1,250,0}}},

			{{{250,1,750,0}}},
			{{{750,1,250,0}}},
			{{{750,1,250,0}}},
			{{{750,1,250,0}}},

			{{{250,1,750,0}}},
			{{{750,1,250,0}}},
			{{{250,1,750,0}}},
			{{{250,1,750,0}}},

			{{{250,1,750,0}}},
			{{{250,1,750,0}}},
			{{{750,1,250,0}}},
			{{{750,1,250,0}}},

			{{{250,1,7750,0}}},
	};
	rmt_write_items(STRIP_RMT_TX_CHANNEL, txbuff, 26, true);
    return 0;
#endif

}


#endif




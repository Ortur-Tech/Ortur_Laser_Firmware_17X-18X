#include "serial_iap.h"
#include "string.h"
#include "driver\uart.h"
#include "tusb_msc_disk.h"
#include "driver.h"
#include "esp32-hal-uart.h"

#define FLASH_START_ADDR 0
uint32_t UART_UPDATE_WAIT_TIME = 500;

#define REC_TIMEOUT 10


typedef struct {
	uint8_t DataBuf[MAX_BUF_SIZE];
	int DataLen;
	uint8_t DataRecFlag;
	uint32_t recStartTime;
}SerialData;


SerialData sd;

portMUX_TYPE mux1 = portMUX_INITIALIZER_UNLOCKED;

void init_target_partition(void)
{
	const esp_partition_t *current_partition;

	current_partition = esp_ota_get_running_partition();

	if (memcmp(current_partition->label, "ota_0", 5) == 0)
	{
		target_partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP,
				ESP_PARTITION_SUBTYPE_ANY, "ota_1");
	}
	else
	{
		target_partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP,
				ESP_PARTITION_SUBTYPE_ANY, "ota_0");
	}
	printf("ota target partition %s.\r\n",target_partition->label);
	portENTER_CRITICAL(&mux1);
	/*擦除*/
	esp_partition_erase_range(target_partition, 0x0, target_partition->size);
	portEXIT_CRITICAL(&mux1);
}

/*串口数据初始化*/
void serial_DataInit(void)
{
	//memset(sd.DataBuf,0,MAX_BUF_SIZE);
	sd.DataLen=0;
	sd.DataRecFlag=0;
}
/*串口数据接收*/
void rec_SerialData(uint8_t data)
{
	if(sd.DataRecFlag!=0) return;
	sd.DataBuf[sd.DataLen++]=data;
	sd.recStartTime=HAL_GetTick();
}
/*串口数据超时检查*/
void rec_TimeCheck(void)
{
	if(sd.DataRecFlag!=0)return;
	if(sd.DataLen==0)return;

	if((HAL_GetTick()-sd.recStartTime)>REC_TIMEOUT)
		sd.DataRecFlag=1;
}
/*计算modbus crc效验*/
unsigned short CRC16_MODBUS(unsigned char *puchMsg, unsigned int usDataLen)
{
   ushort i, j, tmp, CRC16;

	CRC16 = 0xFFFF;             //CRC寄存器初始值
	for (i = 0; i < usDataLen; i++)
	{
		CRC16 ^= puchMsg[i];
		for (j = 0; j < 8; j++)
		{
			tmp = (ushort)(CRC16 & 0x0001);
			CRC16 >>= 1;
			if (tmp == 1)
			{
				CRC16 ^= 0xA001;    //异或多项式
			}
		}
	}
//	cmd[i++] = (byte) (CRC16 & 0x00FF);
//	cmd[i++] = (byte) ((CRC16 & 0xFF00)>>8);
	return CRC16;
}

/*合并16bit数据*/
uint16_t Mg(uint8_t h,uint8_t l)
{
	return ((h<<8)+l);
}

/*串口协议数据响应*/
void reply(uint8_t comm_num,uint8_t cmd,uint8_t* data,uint16_t len)
{
	uint8_t buf[100]={0};
	uint16_t crc=0;

	uint16_t i=0;
	/*协议头*/
	buf[i++]=HEADER_HIGH;
	buf[i++]=HEADER_LOW;
	/*协议版本*/
	buf[i++]=PROTOCOL_VERSION;
	/*通讯编号*/
	buf[i++]=comm_num;
	/*命令码*/
	buf[i++]=cmd;
	/*数据长度*/
	buf[i++]=(len>>8);
	buf[i++]=(len);
	/*数据*/
	memcpy(&buf[i],data,len);
	i=i+len;
	/*crc效验*/
	crc=CRC16_MODBUS(buf,len+7);
	buf[i++]=(crc>>8);
	buf[i++]=(crc);
	/*发送数据*/
	serialWriteData(buf,i);

}

uint8_t mymemcmp(uint8_t* src,uint8_t* data,uint16_t len)
{
	uint16_t i = 0;
	for(i=0;i<len;i++)
	{
		if(src[i]!=data[i])
		{
			return 1;
		}
	}
	return 0;
}

/*升级结构体*/
typedef struct{
	/*固件大小*/
	uint32_t size;
	/*固件类型*/
	uint8_t type;
	/*固件CRC32*/
	uint32_t crc32;
	/*固件烧录地址*/
	uint32_t fileAddr;
}FileInfo;

FileInfo App;

uint8_t tempBuf[MAX_BUF_SIZE]={0};
/*串口数据处理*/
uint8_t serial_DataHandle(void)
{
	uint8_t programCnt = 0;
	volatile uint8_t res = 0;
	static uint32_t pre_packge_num=0;
	uint32_t packge_num = 0;
	static uint32_t write_cnt = 0;
	uint16_t check_crc = 0;
	uint8_t data[10] = {0};
	uint8_t comm_num=0;


	if(sd.DataRecFlag == 1)
	{
		/*判断数据长度*/
		if(sd.DataLen<9)
		{
			serial_DataInit();
			return 0;
		}
		/*判断帧长度*/
		if(sd.DataLen!=(Mg(sd.DataBuf[5],sd.DataBuf[6])+9))
		{
			serial_DataInit();
			return 0;
		}
		/*判断帧头*/
		if((sd.DataBuf[0]!=0x55)||(sd.DataBuf[1]!=0x7a))
		{
			serial_DataInit();
			return 0;
		}
		check_crc=CRC16_MODBUS(sd.DataBuf,sd.DataLen-2);
		/*判断数据效验*/
		if(Mg(sd.DataBuf[sd.DataLen-2],sd.DataBuf[sd.DataLen-1])==check_crc)
		{
			/*通讯编号*/
			comm_num=sd.DataBuf[3];

			switch(sd.DataBuf[4])
			{
				case 0x03:
				{
					mprintf(LOG_INFO,"Communication command received.\r\n");
					serial_DataInit();
					pre_packge_num=0;
					/*初始化目标升级区*/
					init_target_partition();
					reply(comm_num,0x03,(uint8_t*)GFU_CMD_IDENT,8);
					res=1;
					break;
				}
				case 0x04:
				{
					/*进入升级模式后延长退出时间*/
					UART_UPDATE_WAIT_TIME = 3000;
					/*解析一些数据*/
					App.size=sd.DataBuf[DATA_POS+3]+((uint32_t)sd.DataBuf[DATA_POS+2]<<8)+((uint32_t)sd.DataBuf[DATA_POS+1]<<16)+((uint32_t)sd.DataBuf[DATA_POS]<<24);
					App.type=sd.DataBuf[DATA_POS+4];
					App.crc32=sd.DataBuf[DATA_POS+8]+((uint32_t)sd.DataBuf[DATA_POS+7]<<8)+((uint32_t)sd.DataBuf[DATA_POS+6]<<16)+((uint32_t)sd.DataBuf[DATA_POS+5]<<24);
					//App.fileAddr=sd.DataBuf[DATA_POS+12]+((uint32_t)sd.DataBuf[DATA_POS+11]<<8)+((uint32_t)sd.DataBuf[DATA_POS+10]<<16)+((uint32_t)sd.DataBuf[DATA_POS+9]<<24);
					App.fileAddr = FLASH_START_ADDR;
					mprintf(LOG_INFO,"Write data size:%d Write data type:%d Write data crc32:%d Write data addr:%d。\r\n",App.size,App.type,App.crc32,App.fileAddr);

					mprintf(LOG_INFO,"Setup upgrade command received.\r\n");
					data[0]=0;
					serial_DataInit();


					reply(comm_num,0x04,data,1);
					write_cnt=0;
					res=1;
					break;
				}
				case 0x05:
				{
					if(App.type == 1)
					{
						packge_num=Mg(sd.DataBuf[7],sd.DataBuf[8]);
						/*数据包发送错误，请求重复指定数据包*/
						if(packge_num!=pre_packge_num)
						{
							data[0]=1;
							data[1]=pre_packge_num>>8;
							data[2]=pre_packge_num;
							serial_DataInit();
							reply(comm_num,0x05,data,3);
							return 1;
						}
						/*写入flash，最多10次写入效验*/
						while(res == 0)
						{
							programCnt++;
							portENTER_CRITICAL(&mux1);
							esp_partition_write(target_partition, App.fileAddr+write_cnt,&sd.DataBuf[9],Mg(sd.DataBuf[5],sd.DataBuf[6])-2);//更新FLASH代码
							esp_partition_read(target_partition,App.fileAddr+write_cnt,tempBuf,Mg(sd.DataBuf[5],sd.DataBuf[6])-2);
							portEXIT_CRITICAL(&mux1);
							if(0 == mymemcmp(&sd.DataBuf[9],tempBuf,Mg(sd.DataBuf[5],sd.DataBuf[6])-2))
							{
								res = 1;
								pre_packge_num++;
								programCnt=0;
							}
							/*写入flash失败*/
							if(programCnt>10)
							{
								data[0]=2;
								data[1]=packge_num>>8;
								data[2]=packge_num;
								serial_DataInit();
								reply(comm_num,0x05,data,3);
								return 1;
							}
						}
						write_cnt=write_cnt+Mg(sd.DataBuf[5],sd.DataBuf[6])-2;
						mprintf(LOG_IMP_INFO,"Upgrade total file size=%d.\r\nWritten data=%d.\r\n",App.size,write_cnt);
						if(App.size==write_cnt)
						{
							/*数据已经写完，效验CRC32*/
							//if(crc32()==App.crc32)
							{
								mprintf(LOG_IMP_INFO,"The new firmware has been updated...\r\n");
							}
							pre_packge_num=0;
							data[0]=3;//数据接收完成
						}
						else
						{
							data[0]=0;//数据接收ok
						}

						data[1]=packge_num>>8;
						data[2]=packge_num;
						serial_DataInit();
						reply(comm_num,0x05,data,3);
						if(data[0] == 3)
						{
							esp_err_t err = esp_ota_set_boot_partition(target_partition);
							if (err)
							{
								mprintf(LOG_ERROR, "BOOT ERR => %x.\r\n", err);
							}
							esp_restart();
						}
					}
					break;
				}
				case 0x06:
				{
					/*重启*/
					if(sd.DataBuf[DATA_POS]==1)
					{
						data[0]=0;
						serial_DataInit();
						reply(comm_num,0x06,data,1);
						esp_restart();
						res=0;
					}/*进入U盘模式*/
					else if(sd.DataBuf[DATA_POS]==2)
					{
						data[0]=0;
						serial_DataInit();
						reply(comm_num,0x06,data,1);
						res=2;
					}/*跳转到app*/
					else if(sd.DataBuf[DATA_POS]==3)
					{
						data[0]=0;
						serial_DataInit();
						reply(comm_num,0x06,data,1);
						res=0;
						HAL_Delay(10);
						esp_restart();
						//Jump(FLASH_START_ADDR);
					}

					break;
				}
				default:
				{
					res=0;
					break;
				}
			}
		}

	}
	return res;
}

static uint8_t serial_iap_flag = 0;
/**/
void serial_iap_set(uint8_t flag)
{
	serial_iap_flag = flag;
}

uint8_t serial_iap_get(void)
{
	return serial_iap_flag ;
}

void serial_Iap(void)
{
	static uint64_t comm_time=0;
	uint8_t ret = 0;
	if(serial_iap_flag == 1)
	{
		comm_time = HAL_GetTick();
		while(1)
		{
			/*串口数据处理函数*/
			ret = serial_DataHandle();
			if(1 == ret)
			{
				power_LedToggle();
				comm_LedToggle();
				/*更新处理时间*/
				comm_time = HAL_GetTick();
			}
			else if(ret == 2)
			{
				/*退出串口IAP模式，并进入到U盘升级模式*/
				return ;
			}

			/*时间限制500ms超时时间*/
			if((HAL_GetTick() - comm_time) > UART_UPDATE_WAIT_TIME)
			{
				//Jump(FLASH_START_ADDR);
				return ;
			}
		}

	}
}


/*计 算*/








































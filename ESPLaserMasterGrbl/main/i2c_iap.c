/*
 * i2c_iap.c
 *  Created on: 2021年12月18日
 *  Author: horsa
 */
#include "i2c_iap.h"
#include "digital_laser.h"
#include "string.h"

#define digitalaserAddr     0X21
#define POLY                0x1021
i2c_iap_msg  I2CIAPREQ[6] ={0};
i2c_iap_resp I2CIAPRESP[6] ={0};


void laser_iap_init(void)
{
	I2CIAPREQ[i2c_iap_req ].cmdid = I2C_IAP_REQ_CMD;	   I2CIAPREQ[i2c_iap_req ].datlen = 8;
	I2CIAPREQ[i2c_iap_set ].cmdid = I2C_IAP_SET_CMD;       I2CIAPREQ[i2c_iap_set ].datlen = 0x0d;
	I2CIAPREQ[i2c_iap_send].cmdid = I2C_IAP_SENDDATA_CMD;  I2CIAPREQ[i2c_iap_send].datlen = 242;
	I2CIAPREQ[i2c_iap_exe ].cmdid = I2C_IAP_UPDATE_CMD;    I2CIAPREQ[i2c_iap_exe ].datlen = 1;

	I2CIAPRESP[i2c_iap_req ].cmdlen = 17; I2CIAPRESP[i2c_iap_req ].datlen = 8;
	I2CIAPRESP[i2c_iap_set ].cmdlen = 10; I2CIAPRESP[i2c_iap_set ].datlen = 1; I2CIAPREQ[i2c_iap_set].datasend.finfo.ftype = wmcu; I2CIAPRESP[i2c_iap_set].setstatus = unsupport;
	I2CIAPRESP[i2c_iap_send].cmdlen = 12; I2CIAPRESP[i2c_iap_send].datlen = 3;
	I2CIAPRESP[i2c_iap_exe ].cmdlen = 10; I2CIAPRESP[i2c_iap_send].datlen = 1; I2CIAPREQ[i2c_iap_exe].datasend.updatecmd   = jumpapp;
}
/*
 *  brief：得到数字激光头回复数据
 */
esp_err_t i2c_iap_get_resp(iap_cmd_num cmdnum,uint8_t* buf)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (digitalaserAddr << 1) | I2C_MASTER_READ, 1);

	for(int i = 0; i < I2CIAPRESP[cmdnum].cmdlen ; i++)
	{
		if(i == (I2CIAPRESP[cmdnum].cmdlen-1))
		{
			i2c_master_read_byte(cmd, &buf[i], I2C_MASTER_NACK);
		}
		else
		{
			i2c_master_read_byte(cmd, &buf[i], I2C_MASTER_ACK);
		}
	}
//	for(int i = 0; i < I2CIAPRESP[cmdnum].cmdlen; i++)
//	{
//		i2c_master_read_byte(cmd, &buf[i], I2C_MASTER_ACK);
//	}
	i2c_master_stop(cmd);
	esp_err_t  ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 300 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if (ret != ESP_OK) {
			mprintf(LOG_ERROR,"IIC read error:%d.\r\n",ret);
			return ret;
		}
	return 0;
}

/**************
 * brief：解析数字激光头回复的数据
 * 557A||||||
 * 数据头2byte|协议版本1byte	|通讯编号1byte|功能码1byte|数据长度2byte|数据内容	|CRC162byte
 **************/
esp_err_t iap_communication_resp_analyse(iap_cmd_num cmdnum)
{

	uint8_t resp[20] = {0};
	uint8_t i = 0;
	switch(cmdnum)
	    {
	       case i2c_iap_req:

	    	   i2c_iap_get_resp(i2c_iap_req,resp);

	    	   for(i = 0;i < I2CIAPRESP[i2c_iap_req ].datlen;i++)
	    	   {
	    		   I2CIAPRESP[i2c_iap_req].reqdata[i] = resp[i+7];
	    	   }
	    	   break;
	       case i2c_iap_set:
			   i2c_iap_get_resp(i2c_iap_set,resp);
			   I2CIAPRESP[i2c_iap_set].setstatus = resp[7];
			   break;
	       case i2c_iap_send:
			   i2c_iap_get_resp(i2c_iap_send,resp);
			   I2CIAPRESP[i2c_iap_send].datastatus.status = resp[7];
			   I2CIAPRESP[i2c_iap_send].datastatus.cnt = (resp[8] << 8)|resp[9];
			   break;
	       case i2c_iap_exe:
			   i2c_iap_get_resp(i2c_iap_exe,resp);
			   I2CIAPRESP[i2c_iap_exe].updatestatus= resp[7];
			   mprintf(LOG_TEMP,"resp[7]:%d\r\n",resp[7]);
			   break;
	       default:
	    	   break;
	    }
   return 0;
}

/****
 *brief:向数字激光头发送数据包
 *param：发送的命令编号
 *return：发送数据状态 0：继续发数据 1：数据ok 2：数据错误
 ***/
extern uint16_t getiapsubdata(uint8_t* iapdata,const uint16_t currcnt);
extern unsigned short CRC16_MODBUS(unsigned char *puchMsg, unsigned int usDataLen);
uint8_t IAP_COMMUNICATION_REQ(iap_cmd_num cmdnum)
{

	static  uint8_t  msgnum = 0;
	static  uint16_t sendedcnt = 0;
	static  uint8_t   RESEND = 0;
	uint8_t buf[600] = {0};
	uint16_t i = 0,j = 0,pos = 0,subcnt = 0;
	uint16_t CRC16 = 0;
	uint8_t extdata = 0;

	buf[i++] = 0x55;
	buf[i++] = 0x7a;
	buf[i++] = 1;

	switch(cmdnum)
    {
       case i2c_iap_req:
    	   mprintf(LOG_TEMP,"GET IAP REQ START\r\n");
    	   buf[i++] = msgnum;
    	   buf[i++] = I2CIAPREQ[i2c_iap_req].cmdid;
    	   buf[i++] = I2CIAPREQ[i2c_iap_req].datlen >> 8;
    	   buf[i++] = I2CIAPREQ[i2c_iap_req].datlen;
    	   buf[i++] = 1;
    	   buf[i++] = 9;
    	   buf[i++] = 9;
    	   buf[i++] = 3;
    	   buf[i++] = 0;
    	   buf[i++] = 8;
    	   buf[i++] = 0;
    	   buf[i++] = 8;
//    	   for(j=0;j<I2CIAPREQ[i2c_iap_req].datlen;j++)
//    	   {
//    		   buf[i+j] = j;
//    	   }
//    	   pos = i+j;
    	   pos = i;
    	   CRC16 = CRC16_MODBUS(buf,pos) ;
    	   buf[pos++] = (CRC16 >> 8) & 0xff;
    	   buf[pos++]   =  CRC16 & 0xff;
    	   laser_write(buf,pos);//发数据
    	   //发完通讯请求后 发出请求回复命令包
    	   iap_communication_resp_analyse(i2c_iap_req);
    	   for(i = 0;i < I2CIAPRESP[i2c_iap_req ].datlen;i++)
    	   {
    		   mprintf(LOG_TEMP,"reqdata:%d",I2CIAPRESP[i2c_iap_req].reqdata[i]);
    	   }
    	   for(i = 0;i < I2CIAPRESP[i2c_iap_req ].datlen;i++)
		   {
			   if(I2CIAPRESP[i2c_iap_req].reqdata[i] != buf[i+7])
			   {
				   //返回内容不相同则继续发
				   msgnum++;
		    	   mprintf(LOG_TEMP,"GET IAP REQ CHECK FIELED\r\n");
		    	   return 0;
			   }
		   }
    	   mprintf(LOG_TEMP,"GET IAP REQ END\r\n");
    	   break;
       case i2c_iap_set: //发送升级文件大小、校验等信息
    	   buf[i++] = msgnum;
		   buf[i++] = I2CIAPREQ[i2c_iap_set].cmdid;
		   buf[i++] = I2CIAPREQ[i2c_iap_set].datlen >> 8;
		   buf[i++] = I2CIAPREQ[i2c_iap_set].datlen;

		   buf[i++] = I2CIAPREQ[i2c_iap_set].datasend.finfo.filelen >> 24;
		   buf[i++] = I2CIAPREQ[i2c_iap_set].datasend.finfo.filelen >> 16;
		   buf[i++] = I2CIAPREQ[i2c_iap_set].datasend.finfo.filelen >> 8;
		   buf[i++] = I2CIAPREQ[i2c_iap_set].datasend.finfo.filelen;

		   buf[i++] = I2CIAPREQ[i2c_iap_set].datasend.finfo.ftype;

		   buf[i++] = I2CIAPREQ[i2c_iap_set].datasend.finfo.crc32 >> 24;
		   buf[i++] = I2CIAPREQ[i2c_iap_set].datasend.finfo.crc32 >> 16;
		   buf[i++] = I2CIAPREQ[i2c_iap_set].datasend.finfo.crc32 >> 8;
		   buf[i++] = I2CIAPREQ[i2c_iap_set].datasend.finfo.crc32;

		   buf[i++] = I2CIAPREQ[i2c_iap_set].datasend.finfo.write_addr >> 24;
		   buf[i++] = I2CIAPREQ[i2c_iap_set].datasend.finfo.write_addr >> 16;
		   buf[i++] = I2CIAPREQ[i2c_iap_set].datasend.finfo.write_addr >> 8;
		   buf[i++] = I2CIAPREQ[i2c_iap_set].datasend.finfo.write_addr;
		   CRC16 = CRC16_MODBUS(buf,i) ;
		   buf[i++] = (CRC16 >> 8) & 0xff;
		   buf[i++]   =  CRC16 & 0xff;
		   pos = i;
		   laser_write(buf,pos);
		   //发出请求回复命令包
		   iap_communication_resp_analyse(i2c_iap_set);
		  if(I2CIAPRESP[i2c_iap_set].setstatus) //不支持升级
		  {
			  msgnum++;
			  return 2;
		  }
		   break;
       case i2c_iap_send:
//    	   //计算分包大小
//		   if(I2CIAPREQ[i2c_iap_set].datasend.finfo.filelen % (I2CIAPREQ[i2c_iap_send].datlen-2))
//		   {
//			   subcnt = I2CIAPREQ[i2c_iap_set].datasend.finfo.filelen / (I2CIAPREQ[i2c_iap_send].datlen-2);
//		   }
//		   else
//		   {
//			   subcnt =	I2CIAPREQ[i2c_iap_set].datasend.finfo.filelen / (I2CIAPREQ[i2c_iap_send].datlen-2) + 1;
//		   }
//		   mprintf(LOG_TEMP,"subcnt:%d=filelen:%d/datlen:%d\r\n",subcnt,I2CIAPREQ[i2c_iap_set].datasend.finfo.filelen,I2CIAPREQ[i2c_iap_send].datlen);
    	   if( (subcnt = getiapsubdata(I2CIAPREQ[i2c_iap_send].datasend.subdata,sendedcnt))!= 0)
    	   {
    		   if(subcnt % 8 != 0)
    		   {
    			   extdata = 8 - (subcnt % 8);
    			   subcnt+=extdata;
    		   }
			   buf[i++] = msgnum;
			   buf[i++] = I2CIAPREQ[i2c_iap_send].cmdid;
			   buf[i++] = (subcnt+2) >> 8;
			   buf[i++] = (subcnt+2)& 0xff;
			   buf[i++] = sendedcnt >> 8;
			   buf[i++] = sendedcnt;

			   memcpy(&buf[i],I2CIAPREQ[i2c_iap_send].datasend.subdata,subcnt - extdata);//I2CIAPREQ[i2c_iap_send].datasend.subdata
			   i = i + subcnt - extdata;
			   if(extdata) //如果最后一包数据不能被8整除则补上0xff
			   {
				  for(j=0;j < extdata;j++)
				  {
					  buf[i++] = 0xff;
				  }
			   }
			   CRC16 = CRC16_MODBUS(buf,i) ;
			   buf[i++] = (CRC16 >> 8) & 0xff;
			   buf[i++] =  CRC16 & 0xff;
			   laser_write(buf,i);
			   iap_communication_resp_analyse(i2c_iap_send);
			   //根据获取到的分包数据发送状态 判断是否重发
			   mprintf(LOG_TEMP,"subdata[0]:%x,subdata[1]:%x\r\n",I2CIAPREQ[i2c_iap_send].datasend.subdata[0],I2CIAPREQ[i2c_iap_send].datasend.subdata[1]);
			   // mprintf(LOG_TEMP,"datastatus.cnt:%d,cnt:%d\r\n",I2CIAPRESP[i2c_iap_send].datastatus.cnt,sendedcnt);
			   if(I2CIAPRESP[i2c_iap_send].datastatus.status == revok && I2CIAPRESP[i2c_iap_send].datastatus.cnt == sendedcnt)
			   {
				   sendedcnt++;
			   }
			   else if(I2CIAPRESP[i2c_iap_send].datastatus.status == firmwareRevSuccess)
			   {
				   msgnum++;
				   mprintf(LOG_TEMP,"SEND DATA FINISH.\r\n");
				   return 1;
			   }
			   else if((I2CIAPRESP[i2c_iap_send].datastatus.status == resend || I2CIAPRESP[i2c_iap_send].datastatus.status == writedefeated))
			   {
				   //重发此包数据
				   mprintf(LOG_TEMP,"RESEND CURRENT DATA PACKAGE.\r\n");
				   if(RESEND++ > 5 )
				   {
					   RESEND = 0;
					   return 2;//数据错误
				   }
			   }
			   else
			   {
				   return 2;//数据错误
			   }
			   msgnum++;
			   return 0;
    	   }
    	   else
    	   {
    		   return 1; //升级包数据全部发送完成
    	   }
    	   break;
       case i2c_iap_exe:
    	   buf[i++] = msgnum;
		   buf[i++] = I2CIAPREQ[i2c_iap_exe].cmdid;
		   buf[i++] = I2CIAPREQ[i2c_iap_exe].datlen >> 8;
		   buf[i++] = I2CIAPREQ[i2c_iap_exe].datlen;
		   buf[i++] = I2CIAPREQ[i2c_iap_exe].datasend.updatecmd;
		   CRC16 = CRC16_MODBUS(buf,i);
		   buf[i++] = (CRC16 >> 8) & 0xff;
		   buf[i++]   =  CRC16 & 0xff;
		   pos = i;
		   laser_write(buf,pos);
		   iap_communication_resp_analyse(i2c_iap_exe);
		   if(!I2CIAPRESP[i2c_iap_exe].updatestatus) //执行升级成功
		   {
			   return 1;
		   }
		   else
		   {
			   return 2;
		   }
		   break;
       default:
    	   break;
    }
	msgnum++;
   return 1;
}
/*
 *brief:开始数字数字激光器升级流程
 *return:升级状态 0xff：继续当前流程 0：下一个流程 1:升级完成 2：错误
 */
iap_steps i2c_iap(void)
{
	static uint8_t event = I2C_IAP_CONFIRM_INFO;
	static uint8_t reqcnt = 0;
	static uint32_t waittime = 0;
    uint8_t stat = 0;
	switch(event)
	{
	    case I2C_IAP_CONFIRM_INFO:
	    	//确认数字激光头连接以及厂家正确

	    	mprintf(LOG_TEMP, "CONFIRM_INFO,ALIVE:%d,VENDOR:%d\r\n",laser_get_value(COMM_TICK_ALIVE),laser_get_value(COMM_GET_LASER_VENDOR));

	    	if(1 == laser_get_value(COMM_TICK_ALIVE) && 1 == laser_get_value(COMM_GET_LASER_VENDOR))
	    	{
	    		mprintf(LOG_TEMP, "CONFIRM_INFO OK...\r\n");
	    		event = I2C_IAP_RST;
	    		return iap_nexts_step;
	    	}
	    	else if(laser_get_value(COMM_TICK_ALIVE) == 0||laser_get_value(COMM_TICK_ALIVE) == 0xffffffff)
	    	{
	    		//如果通讯失败则强制重启 进入升级流程
	    		gpio_set_level(SPINDLE_ENABLE_PIN, settings.spindle.invert.on ? 1 : 0); //关闭激光头供电
	    		waittime = HAL_GetTick();
	    		event = I2C_IAP_START;
	    		return iap_nexts_step;
	    	}
	    	else
	    	{
	    		mprintf(LOG_TEMP, "CONFIRM_INFO ERROR\r\n");
	    		return iap_exe_failed;
	    	}
	    	break;
	    case I2C_IAP_RST:
			//重启数字激光头
	    	mprintf(LOG_TEMP, "Rst start...\r\n");
	    	waittime = HAL_GetTick() - 50;
		    laser_set_value(COMM_IAP_RST,1);
		    event = I2C_IAP_START;
		    return iap_nexts_step;
		case I2C_IAP_START:
			if((HAL_GetTick() - waittime) > 10)
			{
				gpio_set_level(SPINDLE_ENABLE_PIN, settings.spindle.invert.on ? 0 : 1); //开启激光头供电
				if(IAP_COMMUNICATION_REQ(i2c_iap_req))
				{
					event = I2C_IAP_SET;
					waittime = HAL_GetTick();
					return iap_nexts_step;
				}
				if(reqcnt++ > 5)
				{
					reqcnt = 0;
					return iap_exe_failed;
				}
			}
			break;
		case I2C_IAP_SET:
			if(IAP_COMMUNICATION_REQ(i2c_iap_set)&&!I2CIAPRESP[i2c_iap_set].setstatus)
			{
				event = I2C_IAP_SEND;
				return iap_nexts_step;
			}
			break;
		case I2C_IAP_SEND:
			//等数字激光头擦除flash，防止写入失败
			if (HAL_GetTick() - waittime > 3000)
			{
				mprintf(LOG_TEMP, "SEND DATA start...\r\n");
				stat = IAP_COMMUNICATION_REQ(i2c_iap_send);
				if(1 == stat)
				{
					event = I2C_IAP_EXE;
				}
				else if(2 == stat) //数据错误
				{
					event = 0xff;
					return iap_exe_failed;
				}
				else if(0 == stat)
				{
					return iap_current_step;
				}
				return iap_nexts_step;
			}
			break;
		case I2C_IAP_EXE: //执行升级
			mprintf(LOG_TEMP, "IAP EXE UPDATE START...\r\n");
			stat = IAP_COMMUNICATION_REQ(i2c_iap_exe);
			if(1 == stat)
			{
				return iap_exe_success;//升级成功
			}
			else //升级失败
			{
				event = 0xff;
				return iap_exe_failed;
			}
			break;
		default:
			return iap_exe_failed;
			break;
	}
	return iap_current_step;
}

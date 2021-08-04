/*
 * accelDetection.c
 *
 *  Created on: 2021年5月20日
 *      Author: c
 */
#include "accelDetection.h"
#include "board.h"
#include "driver.h"
#include "driver/i2c.h"
#include "grbl/grbl.h"
#include "grbl/state_machine.h"
#include "my_machine_map.h"
#include "digital_laser.h"
#include "software_i2c.h"


#ifndef ABS
  #define ABS(x) ((x)<0?-(x):(x))
#endif

int16_t accel_x,accel_y, accel_z;
int16_t accel_x_old,accel_y_old,accel_z_old;
uint16_t accel_slope;
uint8_t shake_detected = 0; //是否检测到震动
#define  accel_check_interval_ms 100 //100毫秒检测一次
uint32_t last_accel_check_ms = 0;
uint32_t detection_count = 0; //开始检测
int16_t gsensor_extern_scale = 1;

uint8_t Check_BMA250_ID(void);
void BMA250_Get_Acceleration(short *gx, short *gy, short *gz);

#define BMA250_Addr          0X18 //(0x18<<1)    //SD0 Low  0x18;    SD0 High 0x19;

#define BMP_CHIPID           0x00
#define BMP_ACC_X_LSB        0x02
#define BMP_ACC_X_MSB        0x03
#define BMP_ACC_Y_LSB        0x04
#define BMP_ACC_Y_MSB        0x05
#define BMP_ACC_Z_LSB        0x06
#define BMP_ACC_Z_MSB        0x07

#define BMP_GRANGE           0x0F	   // g Range
#define BMP_BWD              0x10	   // Bandwidth
#define BMP_PM               0x11	   // Power modes
#define BMP_SCR              0x13	   // Special Control Register
#define BMP_RESET            0x14	   // Soft reset register writing 0xB6 causes reset
#define BMP_ISR1             0x16	   // Interrupt settings register 1
#define BMP_ISR2             0x17	   // Interrupt settings register 2
#define BMP_IMR1             0x19	   // Interrupt mapping register 1
#define BMP_IMR2             0x1A	   // Interrupt mapping register 2
#define BMP_IMR3             0x1B	   // Interrupt mapping register 3

#define BMP_AS_RANGE 2
#define BMP_AS_BANDWIDTH 8
#define BMP_AS_SLEEPPHASE 2

uint8_t GsensorDeviceType=0;    //!<gsensor芯片类型



/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
#if USE_SOFTWARE_IIC
	return sw_i2c_init(IIC_SDA_PIN, IIC_SCL_PIN);
#else
    int i2c_master_port = I2C_NUM_0;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = IIC_SDA_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = IIC_SCL_PIN,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
    	mprintf(LOG_ERROR,"IIC Init error:%d.\r\n",err);

        return err;
    }
    err = i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
		mprintf(LOG_ERROR,"IIC i2c_driver_install error:%d.\r\n",err);
		return err;
	}
    return err;
#endif
}

esp_err_t Write_One_Byte_iicaddr(uint8_t addr, uint8_t reg, uint8_t data)
{
#if USE_SOFTWARE_IIC
	esp_err_t err;
	err = sw_i2c_master_start();
	err = sw_i2c_master_write_byte((addr << 1) | I2C_MASTER_WRITE);
	err = sw_i2c_master_write_byte(reg);
	err = sw_i2c_master_write_byte(data);
	err = sw_i2c_master_stop();
	return err;
#else
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, 1);
	i2c_master_write_byte(cmd, reg, 1);
	i2c_master_write_byte(cmd, data, 1);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if (ret != ESP_OK) {
		mprintf(LOG_ERROR,"IIC write error:%d.\r\n",ret);
		return ret;
	}
	return ret;
#endif
}
uint8_t Read_One_Byte(uint8_t addr, uint8_t reg)
{
#if USE_SOFTWARE_IIC
	esp_err_t err;
	uint8_t data = 0;
	err = sw_i2c_master_start();
	err = sw_i2c_master_write_byte((addr << 1) | I2C_MASTER_WRITE);
	err = sw_i2c_master_write_byte(reg);
	err = sw_i2c_master_start();
	err = sw_i2c_master_write_byte((addr << 1) | I2C_MASTER_READ);
	err = sw_i2c_master_read_byte(&data, 1);
	err = sw_i2c_master_stop();
	return data;
#else
	uint8_t data = 0;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, 1);
	i2c_master_write_byte(cmd, reg, 1);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, 1);
	i2c_master_read_byte(cmd, &data, 1);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 300 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if (ret != ESP_OK) {
			mprintf(LOG_ERROR,"IIC read error:%d.\r\n",ret);
			return ret;
		}
	return data;
#endif
}
/**
 * @brief BMA250_Init
 * @param
 */
void BMA250_Init(void)
{
    uint8_t bGRange = 0;                                  // g Range;
    uint8_t bBwd = 0;                                     // Bandwidth
    uint8_t bSleep = 0;                                   // Sleep phase

// Configure sensor and start to sample data
#if (BMP_AS_RANGE == 2)
	bGRange = 0x03;
#elif (BMP_AS_RANGE == 4)
	bGRange = 0x05;
#elif (BMP_AS_RANGE == 8)
	bGRange = 0x08;
#elif (BMP_AS_RANGE == 16)
	bGRange = 0x0C;
#else
//	#error "Measurement range not supported"
#endif

#if (BMP_AS_BANDWIDTH == 8)
	bBwd = 0x08;
#elif (BMP_AS_BANDWIDTH == 16)
	bBwd = 0x09;
#elif (BMP_AS_BANDWIDTH == 31)
	bBwd = 0x0A;
#elif (BMP_AS_BANDWIDTH == 63)
	bBwd = 0x0B;
#elif (BMP_AS_BANDWIDTH == 125)
	bBwd = 0x0C;
#elif (BMP_AS_BANDWIDTH == 250)
	bBwd = 0x0D;
#elif (BMP_AS_BANDWIDTH == 500)
	bBwd = 0x0E;
#elif (BMP_AS_BANDWIDTH == 1000)
	bBwd = 0x0F;
#else
//	#error "Sample rate not supported"
#endif

#if (BMP_AS_SLEEPPHASE == 1)
	bSleep = 0x4C;
#elif (BMP_AS_SLEEPPHASE == 2)
	bSleep = 0x4E;
#elif (BMP_AS_SLEEPPHASE == 4)
	bSleep = 0x50;
#elif (BMP_AS_SLEEPPHASE == 6)
	bSleep = 0x52;
#elif (BMP_AS_SLEEPPHASE == 10)
	bSleep = 0x54;
#elif (BMP_AS_SLEEPPHASE == 25)
	bSleep = 0x56;
#elif (BMP_AS_SLEEPPHASE == 50)
	bSleep = 0x58;
#else
//	#error "Sleep phase duration not supported"
#endif


	// write sensor configuration
	Write_One_Byte_iicaddr(BMA250_Addr, BMP_GRANGE, bGRange);  // Set measurement range
	Write_One_Byte_iicaddr(BMA250_Addr, BMP_BWD, bBwd);        // Set filter bandwidth
	Write_One_Byte_iicaddr(BMA250_Addr, BMP_PM, bSleep);       // Set filter bandwidth


#ifndef BMP_AS_FILTERING
	Write_One_Byte_iicaddr(BMA250_Addr, BMP_SCR, 0x80);        // acquire unfiltered acceleration data
#endif

}

/**
 * @brief Sc7a20_Init 初始化 sc7a20
 */
void Sc7a20_Init(void)
{

	Write_One_Byte_iicaddr(SC7A20_ADDR, 0x20, 0x47);
	Write_One_Byte_iicaddr(SC7A20_ADDR, 0x23, 0x88);

}
/**
 * @brief Get_GsensorType
 * @return 返回芯片id
 */
uint8_t Get_GsensorType(void)
{
	if(SC7A20_DEVICE== Read_One_Byte(SC7A20_ADDR, 0xf))
	{
		return SC7A20_DEVICE;
	}
	else
	{
		return OTHER_DEVICE;
	}
}

/**
 * @brief Sc7a20_Get_Acceleration得到加速度值(原始值)
 * @param gx x轴加速度的原始读数(带符号)
 * @param gy y轴加速度的原始读数(带符号)
 * @param gz z轴加速度的原始读数(带符号)
 */
void Sc7a20_Get_Acceleration(short *gx, short *gy, short *gz)
{
	uint16_t x_l8,x_h8,y_l8,y_h8,z_l8,z_h8;

	x_l8 = Read_One_Byte(SC7A20_ADDR, 0x28);
	x_h8 = Read_One_Byte(SC7A20_ADDR, 0x29);
	y_l8 = Read_One_Byte(SC7A20_ADDR, 0x2A);
	y_h8 = Read_One_Byte(SC7A20_ADDR, 0x2B);
	z_l8 = Read_One_Byte(SC7A20_ADDR, 0x2C);
	z_h8 = Read_One_Byte(SC7A20_ADDR, 0x2D);

    *gx=(short)((x_h8<<8)|x_l8);
    *gy=(short)((y_h8<<8)|y_l8);
    *gz=(short)((z_h8<<8)|z_l8);

    *gx=(*gx)>>6;
    *gy=(*gy)>>6;
    *gz=(*gz)>>6;

	//mprintf(LOG_INFO,"xValue:%d. yValue:%d. zValue:%d.\r\n",*gx,*gy,*gz);
	return ;
}

uint8_t iic_init_flag = 0 ;
/**
 * @brief Gsensor_Init
 * @author Cc
 */
void Gsensor_Init(void)
{
	i2c_master_init();
	laser_init();
	//digital_laser_test();
	//laser_auto_focus();
	if(GsensorDeviceType == 0)
	{
		GsensorDeviceType = Get_GsensorType();
	}
	if(GsensorDeviceType == SC7A20_DEVICE)
	{
		Sc7a20_Init();
		gsensor_extern_scale = 5;
	}
	else
	{
		GsensorDeviceType = Check_BMA250_ID();
		if((GsensorDeviceType != 0xff) && (GsensorDeviceType != 0x00))
		{
			BMA250_Init();
		}
		gsensor_extern_scale = 1;
	}

	if((GsensorDeviceType != SC7A20_DEVICE) &&
			(GsensorDeviceType != BMA250_DEVICE) &&
			(GsensorDeviceType != BMA253_DEVICE))
	{
		GsensorDeviceType = 0;
	}
	iic_init_flag = 1;
}

/**
 * @brief Check_BMA250_ID
 * @return 芯片id
 */
uint8_t Check_BMA250_ID(void)
{
    return (Read_One_Byte(BMA250_Addr, 0x00));
}

/**
 * @brief BMA250_Get_Acceleration 得到加速度值(原始值)
 * @param gx 陀螺仪x,y,z轴的原始读数(带符号)
 * @param gy
 * @param gz
 */
void BMA250_Get_Acceleration(short *gx, short *gy, short *gz)
{
	uint16_t x_l8,x_h8,y_l8,y_h8,z_l8,z_h8;

    x_l8 = Read_One_Byte(BMA250_Addr, BMP_ACC_X_LSB);
    x_h8 = Read_One_Byte(BMA250_Addr, BMP_ACC_X_MSB);
    y_l8 = Read_One_Byte(BMA250_Addr, BMP_ACC_Y_LSB);
    y_h8 = Read_One_Byte(BMA250_Addr, BMP_ACC_Y_MSB);
    z_l8 = Read_One_Byte(BMA250_Addr, BMP_ACC_Z_LSB);
    z_h8 = Read_One_Byte(BMA250_Addr, BMP_ACC_Z_MSB);

    *gx=(short)((x_h8<<8)|x_l8);
    *gy=(short)((y_h8<<8)|y_l8);
    *gz=(short)((z_h8<<8)|z_l8);

    *gx=(*gx)>>6;
    *gy=(*gy)>>6;
    *gz=(*gz)>>6;

}

/**
 * @brief Get_Acceleration
 * @param devAddr 设备地址
 * @param firstAddr 读取首地址
 * @param gx x轴加速度value
 * @param gy y轴加速度value
 * @param gz z轴加速度值
 */
void Get_Acceleration(uint8_t devAddr ,uint8_t firstAddr,short *gx, short *gy, short *gz)
{
	uint16_t x_l8,x_h8,y_l8,y_h8,z_l8,z_h8;

    /*读加速度值*/
    x_l8 = Read_One_Byte(devAddr, firstAddr);
    x_h8 = Read_One_Byte(devAddr, firstAddr+1);
    y_l8 = Read_One_Byte(devAddr, firstAddr+2);
    y_h8 = Read_One_Byte(devAddr, firstAddr+3);
    z_l8 = Read_One_Byte(devAddr, firstAddr+4);
    z_h8 = Read_One_Byte(devAddr, firstAddr+5);

    /*取高10bit*/
    *gx=(short)((x_h8<<8)|x_l8);
    *gy=(short)((y_h8<<8)|y_l8);
    *gz=(short)((z_h8<<8)|z_l8);

	*gx=(*gx)>>6;
	*gy=(*gy)>>6;
	*gz=(*gz)>>6;
}

portMUX_TYPE mux2 = portMUX_INITIALIZER_UNLOCKED;

#if ENABLE_ACCELERATION_DETECT
//初始化并读取加速度计数据
/**
 * @brief 检测加速度是否超限制
 */
void accel_detection()
{
	// 3.91mg
#if USE_SOFTWARE_IIC
	portENTER_CRITICAL(&mux2);
#endif
	if(GsensorDeviceType == SC7A20_DEVICE)
		Get_Acceleration(SC7A20_ADDR, 0X28,&accel_x,&accel_y,&accel_z);
	else
		Get_Acceleration(BMA250_Addr, BMP_ACC_X_LSB,&accel_x,&accel_y,&accel_z);
#if USE_SOFTWARE_IIC
	portEXIT_CRITICAL(&mux2);
#endif
	mprintf(LOG_INFO,"xValue:%d. yValue:%d. zValue:%d.\r\n",accel_x,accel_y,accel_z);

	//计算加速度斜率(加加速度)突变
	//如果加速度突变,就表明有较大的震动或位移,这样雕刻会错位,无法继续
	//至少记录2次值以上加速度值
	if(detection_count > 2)
	{
		//计算加速度变化,雕刻时,不允许加速度突变
		// 3.91mg   g==10m/s
        #define G2MS(g) (g*10) // g 转 mm/s
        #define MG2G(mg) (mg*1000/391) // mg 转 g
        #define GS_K  1000  * 10 / 391 / accel_check_interval_ms //计算斜率的常数K
		accel_slope = ( ABS(accel_x - accel_x_old) + ABS(accel_y - accel_y_old) + ABS(accel_z - accel_z_old) ) * GS_K * gsensor_extern_scale;
		mprintf(LOG_INFO,"accel_slope:%d.\r\n",accel_slope);

		if( accel_slope >= settings.accel_sensitivity )
		{
			mprintf(LOG_INFO,"xValue:%d. yValue:%d. zValue:%d.\r\n",accel_x_old,accel_y_old,accel_z_old);
			mprintf(LOG_INFO,"xValue:%d. yValue:%d. zValue:%d.\r\n",accel_x,accel_y,accel_z);
			shake_detected = 1;
		}
	}

	if(shake_detected)
	{
		shake_detected = 0;

		//进入警告状态,终止雕刻
		//if(sys.trust_state == STATE_CYCLE)
		if(sys.state == STATE_CYCLE)  //---TOCHECK
		{
			switch(GsensorDeviceType)
			{
			case SC7A20_DEVICE :
				hal.stream.write_all("[SC7A20:(");
				break;
			case BMA250_DEVICE :
				hal.stream.write_all("[BMA250:(");
				break;
			case BMA253_DEVICE :
				hal.stream.write_all("[BMA253:(");
				break;
			default:
				hal.stream.write_all("[UKNW:(");
			}
			hal.stream.write(appendbuf(1,uitoa((uint32_t)accel_slope)));
			hal.stream.write_all(") Shock and Movement detected! Check TroubleShooting Section in User Manual.]"ASCII_EOL);
			sys.state = STATE_ALARM;
			sys.abort = 1;
			spindle_set_speed(0);
			spindle_off_directly();
		}
	}

	//计算均值
	accel_x_old = accel_x;
	accel_y_old = accel_y;
	accel_z_old = accel_z;

	++detection_count;//统计检测次数
}

/**
 * @brief
 *  限制加速度检测的频率100ms读取一次
 */
void accel_detection_limit()
{
  static uint8_t recursion = 0;

  if(GsensorDeviceType == 0) return;

  if(!iic_init_flag) return;

  if(!recursion)//防止递归调用
  {
	recursion++;

	//雕刻运行时才检测
	//if(sys.trust_state == STATE_CYCLE)
	if(sys.state == STATE_CYCLE)
	{
		//定时检查,避免速度太频繁
		if(last_accel_check_ms + accel_check_interval_ms <= HAL_GetTick())
		{
			accel_detection();
			last_accel_check_ms = HAL_GetTick();
		}
	}
	else
	{
		detection_count = 0;
	}

	recursion--;
  }
}

#endif


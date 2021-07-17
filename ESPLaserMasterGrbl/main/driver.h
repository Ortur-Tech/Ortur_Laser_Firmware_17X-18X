/*
  driver.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for ESP32

  Part of GrblHAL

  Copyright (c) 2018-2020 Terje Io
  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __DRIVER_H__
#define __DRIVER_H__

#ifndef OVERRIDE_MY_MACHINE
//
// Set options from my_machine.h
//
#include "my_machine.h"

#if NETWORKING_ENABLE
#define WIFI_ENABLE 1
#endif

#if WEBUI_ENABLE
#error "WebUI is not available in this setup!"
#endif
//
#else
//
// options for cmake (idf.py)
//
#ifdef CNC_BOOSTERPACK
#define BOARD_CNC_BOOSTERPACK  1
#else
// NOTE: Only one board may be enabled!
// If none is enabled pin mappings from generic_map.h will be used
//#define BOARD_BDRING_V3P5
//#define BOARD_BDRING_V4
//#define BOARD_BDRING_I2S6A // NOT production ready!
#endif

//
// Set options from CMakeLists.txt
//
#ifdef WEBUI_ENABLE
#undef WEBUI_ENABLE
#define WEBUI_ENABLE 1
#endif

#ifdef AUTH_ENABLE
#undef AUTH_ENABLE
#define AUTH_ENABLE 1
#endif

#ifdef SDCARD_ENABLE
#undef SDCARD_ENABLE
#define SDCARD_ENABLE 1
#endif

#ifdef BLUETOOTH_ENABLE
#undef BLUETOOTH_ENABLE
#define BLUETOOTH_ENABLE 1
#endif

#ifdef MPG_MODE_ENABLE
#undef MPG_MODE_ENABLE
#define MPG_MODE_ENABLE 1
#endif

#ifdef KEYPAD_ENABLE
#undef KEYPAD_ENABLE
#define KEYPAD_ENABLE 1
#endif

#ifdef TRINAMIC_ENABLE
#undef TRINAMIC_ENABLE
#define TRINAMIC_ENABLE 1
#define TRINAMIC_I2C    1
#endif

#ifdef NETWORKING_ENABLE
#define WIFI_ENABLE      1
#define HTTP_ENABLE      0
#define TELNET_ENABLE    1
#define WEBSOCKET_ENABLE 1
#define NETWORK_TELNET_PORT     23
#define NETWORK_HTTP_PORT       80
#define NETWORK_WEBSOCKET_PORT  81
#endif

#if WEBUI_ENABLE
#undef HTTP_ENABLE
#define HTTP_ENABLE 1
#endif

#define EEPROM_ENABLE 0

#endif

#include "soc/rtc.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "driver/ledc.h"
#include "driver/rmt.h"
#include "driver/i2c.h"

#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "grbl/hal.h"

static const DRAM_ATTR float FZERO = 0.0f;

#define PWM_RAMPED       0 // Ramped spindle PWM.
#define PROBE_ENABLE     1 // Probe input
#define PROBE_ISR        0 // Catch probe state change by interrupt TODO: needs verification!
#define TRINAMIC_DEV     0 // Development mode, adds a few M-codes to aid debugging. Do not enable in production code

// DO NOT change settings here!

#ifndef IOEXPAND_ENABLE
#define IOEXPAND_ENABLE 0 // I2C IO expander for some output signals.
#endif

#ifndef WIFI_SOFTAP
#define WIFI_SOFTAP      0
#endif

#ifndef KEYPAD_ENABLE
#define KEYPAD_ENABLE    0
#endif

#ifndef NETWORKING_ENABLE
#define WIFI_ENABLE      0
#define HTTP_ENABLE      0
#define TELNET_ENABLE    0
#define WEBSOCKET_ENABLE 0
#endif

#ifndef BLUETOOTH_ENABLE
#define BLUETOOTH_ENABLE 0
#endif

#ifndef AUTH_ENABLE
#define AUTH_ENABLE      0
#endif

#ifndef SDCARD_ENABLE
#define SDCARD_ENABLE    0
#endif

#ifndef WEBUI_ENABLE
#define WEBUI_ENABLE     0
#endif

#ifndef TRINAMIC_ENABLE
#define TRINAMIC_ENABLE  0
#define TRINAMIC_I2C     0
#endif

// end configuration

#if !WIFI_ENABLE
  #if HTTP_ENABLE || TELNET_ENABLE || WEBSOCKET_ENABLE
  #error "Networking protocols requires networking enabled!"
  #endif // WIFI_ENABLE
#else

#if !NETWORK_PARAMETERS_OK

// WiFi Station (STA) settings
#define NETWORK_HOSTNAME    "Grbl"
#define NETWORK_IPMODE      1 // 0 = static, 1 = DHCP, 2 = AutoIP
#define NETWORK_IP          "192.168.5.1"
#define NETWORK_GATEWAY     "192.168.5.1"
#define NETWORK_MASK        "255.255.255.0"

// WiFi Access Point (AP) settings
#if WIFI_SOFTAP
#define NETWORK_AP_HOSTNAME "GrblAP"
#define NETWORK_AP_IP       "192.168.5.1"
#define NETWORK_AP_GATEWAY  "192.168.5.1"
#define NETWORK_AP_MASK     "255.255.255.0"
#define WIFI_AP_SSID        "GRBL"
#define WIFI_AP_PASSWORD    "GrblPassword" // Minimum 8 characters, or blank for open
#define WIFI_MODE WiFiMode_AP; // OPTION: WiFiMode_APSTA
#else
#define WIFI_MODE WiFiMode_STA; // Do not change!
#endif

#if NETWORK_IPMODE < 0 || NETWORK_IPMODE > 2
#error "Invalid IP mode selected!"
#endif

#if NETWORK_IPMODE == 0 && WIFI_SOFTAP
#error "Cannot use static IP for station when soft AP is enabled!"
#endif

#endif // !NETWORK_PARAMETERS_OK
#endif // WIFI_ENABLE

#if BLUETOOTH_ENABLE
#define BLUETOOTH_DEVICE    "GRBL"
#define BLUETOOTH_SERVICE   "GRBL Serial Port" // Minimum 8 characters, or blank for open
#endif

// End configuration

#if TRINAMIC_ENABLE
#include "tmc2130/trinamic.h"
#endif

#ifdef SPINDLE_HUANYANG
#include "spindle/huanyang.h"
#endif

typedef struct
{
    grbl_wifi_mode_t mode;
    wifi_sta_settings_t sta;
    wifi_ap_settings_t ap;
//  network_settings_t network;
    password_t admin_password;
    password_t user_password;
} wifi_settings_t;

typedef struct {
    uint8_t action;
    void *params;
} i2c_task_t;

// End configuration

#ifdef BOARD_CNC_BOOSTERPACK
  #include "cnc_boosterpack_map.h"
#elif defined(BOARD_BDRING_V4)
  #include "bdring_v4_map.h"
#elif defined(BOARD_BDRING_V3P5)
  #include "bdring_v3.5_map.h"
#elif defined(BOARD_BDRING_I2S6A)
  #include "bdring_i2s_6_axis_map.h"
#else // default board - NOTE: NOT FINAL VERSION!
  #include "my_machine_map.h"
#endif

#ifndef GRBL_ESP32
#error "Add #define GRBL_ESP32 in grbl/config.h or update your CMakeLists.txt to the latest version!"
#endif

#ifdef I2C_PORT
extern QueueHandle_t i2cQueue;
extern SemaphoreHandle_t i2cBusy;
#elif IOEXPAND_ENABLE || KEYPAD_ENABLE || EEPROM_ENABLE || (TRINAMIC_ENABLE && TRINAMIC_I2C)
#error "I2C port not available!"
#endif

#if MPG_MODE_ENABLE || MODBUS_ENABLE
#define SERIAL2_ENABLE 1
#endif

#if MPG_MODE_ENABLE
  #ifndef MPG_ENABLE_PIN
  #error "MPG_ENABLE_PIN must be defined when MPG mode is enabled!"
  #endif
  #ifndef UART2_RX_PIN
  #error "UART2_RX_PIN must be defined when MPG mode is enabled!"
  #endif
#endif

void selectStream (stream_type_t stream);

extern portMUX_TYPE mux ;

#ifdef DELAY_OFF_SPINDLE

  #define MAX_SPINDLE_FAN_TIME (2*60*1000) //最大的风扇冷却时间
  #define MIN_SPINDLE_FAN_TIME (30*1000)   //最小的风扇冷却时间

  //假定散热和发热都是线性关系
  //激光器全工作超过30分钟风扇延时工作120秒，确保激光器冷却
  #define FAN_HEAT_DISSIPATION_PER_SECOND  400  //风扇辅助每秒散热量
  #define AIR_HEAT_DISSIPATION_PER_SECOND  100  //自然空冷每秒散热量
  #define LASER_CALORIFIC_PER_SECOND      1000  //激光每秒发热量

  //最大发热量
  #define MAX_SPINDLE_HEAT   ( MAX_SPINDLE_FAN_TIME / 1000 * FAN_HEAT_DISSIPATION_PER_SECOND)

  /* 主轴/激光 是否关闭的标识 */
  extern uint8_t spindle_disable_by_grbl;
  /* 主轴/激光 被关闭的时间 */
  extern uint32_t spindle_disabled_time;
  /* 主轴/激光 累积热量*/
  extern uint32_t spindle_cumulative_heat;
  /* 主轴/激光 是否挂起的标识 */
  extern uint8_t spindle_suspend_flag;
  /* 主轴/激光 风扇延时时间*/
  extern uint32_t spindle_fan_delay_time;

  /**
   * @brief spindle_suspend_flag_set 设置挂起状态，用于挂起恢复是开激光使能
   * @param status
   */
  inline void spindle_suspend_flag_set(uint8_t status)
  {
  	spindle_suspend_flag = status;
  }

  /**
   * @brief is_spindle_suspend_flag_set 读取挂起状态
   * @return
   */
  inline uint8_t is_spindle_suspend_flag_set(void)
  {
  	return spindle_suspend_flag;
  }

  /**
   * @brief grbl 层面关激光供电
   * @param status
   */
  void spindle_disable_by_grbl_set(uint8_t status);

  /**
   * @brief delay_stop_spindle
   * @return 1: 允许关激光 0：需要延时关激光
   */
  uint8_t spindle_delay_stop(void);

  /**
   * @brief 计算主轴热量累积
   */
  void spindle_calculate_heat();

#endif

uint8_t IsMainPowrIn(void);
void Main_PowerCheckReport(uint8_t mode);
void Main_PowerCheck(void);
void spindle_off (void);
bool driver_init (void);
/*激光是否打开*/
uint8_t is_SpindleOpen(void);
/*获取激光pwm功率，*/
uint16_t laser_GetPower(void);
void light_SetState(uint8_t s);

uint8_t fan_GetSpeed(void);
void fan_PwmSet(uint8_t duty);
void beep_PwmSet(uint8_t duty);

void fire_Alarm(void);
void fire_Check(void);
void fire_AlarmStateSet(uint8_t state);
void fire_GetAverageValue(void);
void fire_InfoReport(void);
uint32_t fire_GetEvnValue(void);
uint32_t fire_GetCurrentValue(void);
void fire_CheckTempEnable(void);
void fire_CheckTempDisable(void);

uint8_t light_GetBrightness(void);
uint8_t fan_GetSpeed(void);
void spindle_off_directly(void);
void mcu_reboot();
void spindle_set_speed (uint_fast16_t pwm_value);
void power_KeyInit(void);
uint8_t power_KeyDown(void);
void led_Init(void);
void power_LedToggle(void);
void power_LedOn(void);
void power_LedOff(void);
void comm_LedToggle(void);
void comm_LedOn(void);
void comm_LedOff(void);
void power_LedAlarm(void);
void reset_report(void);

void spindle_reset(void);

uint32_t power_GetVotage(void);
uint32_t power_GetCurrent(void);

void power_CtrlInit(void);
void power_CtrOff(void);
void power_CtrOn(void);

void system_UpdateAutoPoweroffTime(void);
void system_AutoPowerOff(void);
void movement_laseron_check(void);

void laser_use_time_save(void);
void laser_on_time_count(void);

#endif // __DRIVER_H__

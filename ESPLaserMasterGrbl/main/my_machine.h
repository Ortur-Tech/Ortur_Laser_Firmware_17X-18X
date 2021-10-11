/*
  my_machine.h - configuration for ESP32 processos

  NOTE: only in use if not compiling with cmake (idf.py)

  Part of GrblHAL

  Copyright (c) 2020 Terje Io

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
#include "board.h"

#define PLACE_ORIGIN "China"

#define PRODUCER_NAME "ORTUR"

#define FW_AUTHOR "ORTUR"

#if BOARD_VERSION == OLM_ESP_V1X
#define ORTUR_FW_VERSION_NUM 173
#define ORTUR_FW_VERSION "173"
#elif BOARD_VERSION == OLM_ESP_PRO_V1X
#define ORTUR_FW_VERSION_NUM 184
#define ORTUR_FW_VERSION "184"
#elif BOARD_VERSION == OCM_ESP_PRO_V1X
#define ORTUR_FW_VERSION_NUM 300
#define ORTUR_FW_VERSION "300"
#else
#define ORTUR_FW_VERSION_NUM 900
#define ORTUR_FW_VERSION "900"
#endif

#define ORTUR_FW_NAME "OLF " ORTUR_FW_VERSION


#if BOARD_VERSION == OLM_ESP_V1X

#if (MACHINE_TYPE == OLM_2_PRO)
#define ORTUR_MODEL_NAME "Ortur Laser Master 2 Pro S1"
#elif (MACHINE_TYPE == AUFERO_4)
#define ORTUR_MODEL_NAME "Ortur Aufero 4 S1"
#elif (MACHINE_TYPE == AUFERO_1)
#define ORTUR_MODEL_NAME "Ortur Aufero 1 S1"
#elif (MACHINE_TYPE == OLM_PRO)
#define ORTUR_MODEL_NAME "Ortur Laser Master Pro S1"
#elif
#define ORTUR_MODEL_NAME "Unkown Machine S1"
#endif

#elif BOARD_VERSION == OLM_ESP_PRO_V1X

#if (MACHINE_TYPE == OLM_2_PRO)
#define ORTUR_MODEL_NAME "Ortur Laser Master 2 Pro S2"
#elif (MACHINE_TYPE == AUFERO_4)
#define ORTUR_MODEL_NAME "Ortur Aufero 4 S2"
#elif (MACHINE_TYPE == AUFERO_1)
#define ORTUR_MODEL_NAME "Ortur Aufero 1 S2"
#elif (MACHINE_TYPE == OLM_PRO)
#define ORTUR_MODEL_NAME "Ortur Laser Master Pro S2"
#elif
#define ORTUR_MODEL_NAME "Unkown Machine S2"
#endif

#elif BOARD_VERSION == OCM_ESP_PRO_V1X
#if (MACHINE_TYPE == AUFERO_CNC)
#define ORTUR_MODEL_NAME "Ortur Aufero CNC S2"
#else
#error "错误的机型！！！"
#endif
#else
#error "错误的硬件版本！！！"
#endif

// NOTE: Only one board may be enabled!
// If none is enabled pin mappings from generic_map.h will be used
//#define BOARD_BDRING_V3P5
//#define BOARD_BDRING_V4
//#define BOARD_BDRING_I2S6A // NOT production ready!

// Configuration
// Uncomment to enable, for some a value > 1 may be assigned, if so the default value is shown.

//#define NETWORKING_ENABLE  1 // WiFi streaming. Requires networking plugin.
#if NETWORKING_ENABLE
//#define WIFI_SOFTAP        1 // Use Soft AP mode for WiFi.
//#define WEBUI_ENABLE       0 // Not yet available, do not change.
#endif
//#define SDCARD_ENABLE      1 // Run gcode programs from SD card, requires sdcard plugin.
//#define BLUETOOTH_ENABLE   1 // Enable Bloetooht streaming.
//#define MPG_MODE_ENABLE    1 // Enable MPG mode (secondary serial port)
//#define EEPROM_ENABLE      1 // I2C EEPROM support. Set to 1 for 24LC16(2K), 2 for larger sizes. Requires eeprom plugin.
//#define EEPROM_IS_FRAM     1 // Uncomment when EEPROM is enabled and chip is FRAM, this to remove write delay.

#if NETWORKING_ENABLE
#define NETWORK_PARAMETERS_OK   1
#define TELNET_ENABLE           1 // Telnet daemon - requires Ethernet streaming enabled.
#define WEBSOCKET_ENABLE        1 // Websocket daemon - requires Ethernet streaming enabled.
#define NETWORK_HOSTNAME        "Grbl"
#define NETWORK_IPMODE          1 // 0 = static, 1 = DHCP, 2 = AutoIP
#define NETWORK_IP              "192.168.5.1"
#define NETWORK_GATEWAY         "192.168.5.1"
#define NETWORK_MASK            "255.255.255.0"
#define NETWORK_TELNET_PORT     23
#define NETWORK_WEBSOCKET_PORT  81
#define NETWORK_HTTP_PORT       80

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
#define WIFI_SOFTAP 0
#define WIFI_MODE WiFiMode_STA; // Do not change!
#endif

#endif

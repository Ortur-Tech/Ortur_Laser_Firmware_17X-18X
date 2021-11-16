/*
  settings.h - non-volatile storage configuration handling

  Part of GrblHAL

  Copyright (c) 2017-2020 Terje Io
  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
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

#ifndef _SETTINGS_H_
#define _SETTINGS_H_

#include "config.h"
#include "system.h"
#include "my_machine.h"

// Version of the persistent storage data. Will be used to migrate existing data from older versions of Grbl
// when firmware is upgraded. Always stored in byte 0 of non-volatile storage
#define SETTINGS_VERSION ORTUR_FW_VERSION_NUM  // NOTE: Check settings_reset() when moving to next version.


// Define axis settings numbering scheme. Starts at Setting_AxisSettingsBase, every INCREMENT, over N_SETTINGS.
#ifdef ENABLE_BACKLASH_COMPENSATION
#define AXIS_N_SETTINGS          6
#else
#define AXIS_N_SETTINGS          4
#endif
#define AXIS_SETTINGS_INCREMENT  10 // Must be greater than the number of axis settings TODO: change to 100 to allow for a logical wider range of parameters?

// Define encoder settings numbering scheme. Starts at Setting_EncoderSettingsBase, every INCREMENT, over N_SETTINGS.
// Not referenced by the core.
#define ENCODER_N_SETTINGS_MAX 5 // NOTE: This is the maximum number of encoders allowed.
#define ENCODER_SETTINGS_INCREMENT 10

typedef enum {
    Setting_PulseMicroseconds = 0,
    Setting_StepperIdleLockTime = 1,
    Setting_StepInvertMask = 2,
    Setting_DirInvertMask = 3,
    Setting_InvertStepperEnable = 4,
    Setting_LimitPinsInvertMask = 5,
    Setting_InvertProbePin = 6,
    Setting_StatusReportMask = 10,
    Setting_JunctionDeviation = 11,
    Setting_ArcTolerance = 12,
    Setting_ReportInches = 13,
    Setting_ControlInvertMask = 14,
    Setting_CoolantInvertMask = 15,
    Setting_SpindleInvertMask = 16,
    Setting_ControlPullUpDisableMask = 17,
    Setting_LimitPullUpDisableMask = 18,
    Setting_ProbePullUpDisable = 19,
    Setting_SoftLimitsEnable = 20,
    Setting_HardLimitsEnable = 21,
    Setting_HomingEnable = 22,
    Setting_HomingDirMask = 23,
    Setting_HomingFeedRate = 24,
    Setting_HomingSeekRate = 25,
    Setting_HomingDebounceDelay = 26,
    Setting_HomingPulloff = 27,
    Setting_G73Retract = 28,
    Setting_PulseDelayMicroseconds = 29,
    Setting_RpmMax = 30,
    Setting_RpmMin = 31,
    Setting_Mode = 32,
    Setting_PWMFreq = 33,
    Setting_PWMOffValue = 34,
    Setting_PWMMinValue = 35,
    Setting_PWMMaxValue = 36,
    Setting_StepperDeenergizeMask = 37,
    Setting_SpindlePPR = 38,
    Setting_EnableLegacyRTCommands = 39,
    Setting_JogSoftLimited = 40,
    Setting_ParkingEnable = 41,
    Setting_ParkingAxis = 42,

    Setting_HomingLocateCycles = 43,
    Setting_HomingCycle_1 = 44,
    Setting_HomingCycle_2 = 45,
    Setting_HomingCycle_3 = 46,
    Setting_HomingCycle_4 = 47,
    Setting_HomingCycle_5 = 48,
    Setting_HomingCycle_6 = 49,
// Optional driver implemented settings for jogging
    Setting_JogStepSpeed = 50,
    Setting_JogSlowSpeed = 51,
    Setting_JogFastSpeed = 52,
    Setting_JogStepDistance = 53,
    Setting_JogSlowDistance = 54,
    Setting_JogFastDistance = 55,
//
    Setting_ParkingPulloutIncrement = 56,
    Setting_ParkingPulloutRate = 57,
    Setting_ParkingTarget = 58,
    Setting_ParkingFastRate = 59,

    Setting_RestoreOverrides = 60,
    Setting_IgnoreDoorWhenIdle = 61,
    Setting_SleepEnable = 62,
    Setting_HoldActions = 63,
    Setting_ForceInitAlarm = 64,
    Setting_ProbingFeedOverride = 65,
// Optional driver implemented settings for piecewise linear spindle PWM algorithm
    Setting_LinearSpindlePiece1 = 66,

	Setting_LinearSpindlePiece2 = 67,
	Setting_LinearSpindlePiece3 = 68,
	Setting_LinearSpindlePiece4 = 69,

//

// Optional driver implemented settings for additional streams
    Setting_NetworkServices = 70,
    Setting_BlueToothDeviceName = 71,
    Setting_BlueToothServiceName = 72,
    Setting_WifiMode = 73,
    Setting_WiFi_STA_SSID = 74,
    Setting_WiFi_STA_Password = 75,
    Setting_WiFi_AP_SSID = 76,
    Setting_WiFi_AP_Password = 77,
    Setting_Wifi_AP_Country = 78,
    Setting_Wifi_AP_Channel = 79,
//

// Optional settings for closed loop spindle speed control
    Setting_SpindlePGain = 80,
    Setting_SpindleIGain = 81,
    Setting_SpindleDGain = 82,
    Setting_SpindleDeadband = 83,
    Setting_SpindleMaxError = 84,
    Setting_SpindleIMaxError = 85,
    Setting_SpindleDMaxError = 86,

// Optional settings for closed loop spindle synchronized motion
    Setting_PositionPGain = 90,
    Setting_PositionIGain = 91,
    Setting_PositionDGain = 92,
    Setting_PositionDeadband = 93,
    Setting_PositionMaxError = 94,
    Setting_PositionIMaxError = 95,
    Setting_PositionDMaxError = 96,
//

    Setting_AxisSettingsBase = 100, // NOTE: Reserving settings values >= 100 for axis settings. Up to 255.
    Setting_AxisSettingsMax = 255,

// Optional driver implemented settings
    Setting_TrinamicDriver = 256,
    Setting_TrinamicHoming = 257,

/*******************************添加参数BEGIN***************************************************/
	Setting_FireLogEnable = 259,
	Setting_FireAlarmDeltaThreshold = 260,
	Setting_FireAlarmThreshold = 261,
#if ENABLE_ACCELERATION_DETECT
	Setting_AccelerateThreshold = 262,
#endif
	Setting_AutoPowerOffTime = 263,
	Setting_LaserUsedTime = 264,
	Setting_LaserFocalLength = 265,//焦距
	Setting_IICRate = 266,			//IIC通讯速率
	Setting_EnableDigitalLaserMode = 267,//IIC激光头控制模式
	Setting_EnableEcho = 268,				//使能echo
	Setting_PowerLogEnable = 269,			//电源调试信息使能
	Setting_UartBaudrate = 270,			//串口baud
#if ENABLE_HOMING_FORCE_SET_ORIGIN_OFFSET
	Setting_OriginOffsetX = 271,
	Setting_OriginOffsetY = 272,
	Setting_OriginOffsetZ = 273,
#endif
/******************************添加参数END************************************************/

    // Normally used for Ethernet or WiFi Station
    Setting_Hostname = 300,
    Setting_IpMode = 301,
    Setting_IpAddress = 302,
    Setting_Gateway = 303,
    Setting_NetMask = 304,
    Setting_TelnetPort = 305,
    Setting_HttpPort = 306,
    Setting_WebSocketPort = 307,

    // Normally used for WiFi Access Point
    Setting_Hostname2 = 310,
    Setting_IpMode2 = 311,
    Setting_IpAddress2 = 312,
    Setting_Gateway2 = 313,
    Setting_NetMask2 = 314,
    Setting_TelnetPort2 = 315,
    Setting_HttpPort2 = 316,
    Setting_WebSocketPort2 = 317,

    Setting_Hostname3 = 320,
    Setting_IpMode3 = 321,
    Setting_IpAddress3 = 322,
    Setting_Gateway3 = 323,
    Setting_NetMask3 = 324,
    Setting_TelnetPort3 = 325,
    Setting_HttpPort3 = 326,
    Setting_WebSocketPort3 = 327,

    Setting_AdminPassword = 330,
    Setting_UserPassword = 331,

    Setting_SpindleAtSpeedTolerance = 340,
    Setting_ToolChangeMode = 341,
    Setting_ToolChangeProbingDistance = 342,
    Setting_ToolChangeFeedRate = 343,
    Setting_ToolChangeSeekRate = 344,

    Setting_THC_Mode = 350,
    Setting_THC_Delay = 351,
    Setting_THC_Threshold = 352,
    Setting_THC_PGain = 353,
    Setting_THC_IGain = 354,
    Setting_THC_DGain = 355,
    Setting_THC_VADThreshold = 356,
    Setting_THC_VoidOverride = 357,
    Setting_Arc_FailTimeout = 358,
    Setting_Arc_RetryDelay = 359,
    Setting_Arc_MaxRetries = 360,
    Setting_Arc_VoltageScale = 361,
    Setting_Arc_VoltageOffset = 362,
    Setting_Arc_HeightPerVolt = 363,
    Setting_Arc_OkHighVoltage = 364,
    Setting_Arc_OkLowVoltage = 365,

    Settings_IoPort_InvertIn  = 370,
    Settings_IoPort_Pullup_Disable = 371,
    Settings_IoPort_InvertOut = 372,
    Settings_IoPort_OD_Enable = 373,

    Setting_EncoderSettingsBase = 400, // NOTE: Reserving settings values >= 400 for encoder settings. Up to 449.
    Setting_EncoderSettingsMax = 449,

    // Reserved for user plugins - do NOT use for public plugins
    Setting_UserDefined_0 = 450,
    Setting_UserDefined_1 = 451,
    Setting_UserDefined_2 = 452,
    Setting_UserDefined_3 = 453,
    Setting_UserDefined_4 = 454,
    Setting_UserDefined_5 = 455,
    Setting_UserDefined_6 = 456,
    Setting_UserDefined_7 = 457,
    Setting_UserDefined_8 = 458,
    Setting_UserDefined_9 = 459,

    Setting_SettingsMax
//
} setting_type_t;

typedef enum {
    AxisSetting_StepsPerMM = 0,
    AxisSetting_MaxRate = 1,
    AxisSetting_Acceleration = 2,
    AxisSetting_MaxTravel = 3,
    AxisSetting_StepperCurrent = 4,
    AxisSetting_MicroSteps = 5,
    AxisSetting_Backlash = 6
    /*
    AxisSetting_P_Gain = 7,
    AxisSetting_I_Gain = 8,
    AxisSetting_D_Gain = 9,
    AxisSetting_I_MaxError = 10
    */
} axis_setting_type_t;

typedef union {
    uint8_t mask;
    struct {
        uint8_t defaults          :1,
                parameters        :1,
                startup_lines     :1,
                build_info        :1,
                driver_parameters :1,
                unassigned        :3;

    };
} settings_restore_t;

extern const settings_restore_t settings_all;

typedef char stored_line_t[MAX_STORED_LINE_LENGTH];

typedef union {
    uint16_t value;
    struct {
        uint16_t report_inches                :1,
                 restore_overrides            :1,
                 safety_door_ignore_when_idle :1,
                 sleep_enable                 :1,
                 disable_laser_during_hold    :1,
                 force_initialization_alarm   :1,
                 legacy_rt_commands           :1,
                 restore_after_feed_hold      :1,
                 unassigned                   :8;
    };
} settingflags_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t invert_probe_pin         :1,
                disable_probe_pullup     :1,
                invert_connected_pin     :1,
                disable_connected_pullup :1,
                allow_feed_override      :1,
                enable_protection        :1,
                unassigned               :2;
    };
} probeflags_t;

typedef union {
    uint16_t mask;
    struct {
        uint16_t machine_position   :1,
                 buffer_state       :1,
                 line_numbers       :1,
                 feed_speed         :1,
                 pin_state          :1,
                 work_coord_offset  :1,
                 overrides          :1,
                 probe_coordinates  :1,
                 sync_on_wco_change :1,
                 parser_state       :1,
                 alarm_substate     :1,
                 run_substate       :1,
                 unassigned         :4;
    };
} reportmask_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t enabled                 :1,
                deactivate_upon_init    :1,
                enable_override_control :1,
                unassigned              :5;
    };
} parking_setting_flags_t;

typedef struct {
    parking_setting_flags_t flags;
    uint8_t axis;               // Define which axis that performs the parking motion
    float target;               // Parking axis target. In mm, as machine coordinate [-max_travel,0].
    float rate;                 // Parking fast rate after pull-out in mm/min.
    float pullout_rate;         // Pull-out/plunge slow feed rate in mm/min.
    float pullout_increment;    // Spindle pull-out and plunge distance in mm. Incremental distance.
} parking_settings_t;

typedef struct {
    float p_gain;
    float i_gain;
    float d_gain;
    float p_max_error;
    float i_max_error;
    float d_max_error;
    float deadband;
    float max_error;
} pid_values_t;

typedef struct {
    float rpm_max;
    float rpm_min;
    float pwm_freq;
    float pwm_period;
    float pwm_off_value;
    float pwm_min_value;
    float pwm_max_value;
    float at_speed_tolerance;
    pwm_piece_t pwm_piece[SPINDLE_NPWM_PIECES];
    pid_values_t pid;
    uint16_t ppr; // Spindle encoder pulses per revolution
    spindle_state_t invert;
    bool disable_with_zero_speed;
} spindle_settings_t;

typedef struct {
    pid_values_t pid;
} position_pid_t; // Used for synchronized motion

typedef union {
    uint8_t value;
    struct {
        uint8_t enabled              :1,
                single_axis_commands :1,
                init_lock            :1,
                force_set_origin     :1,
                manual               :1,
                unassigned           :3;
    };
} homing_settings_flags_t;

typedef struct {
    float fail_length_percent; // DUAL_AXIS_HOMING_FAIL_AXIS_LENGTH_PERCENT
    float fail_distance_max;   // DUAL_AXIS_HOMING_FAIL_DISTANCE_MAX
    float fail_distance_min;   // DUAL_AXIS_HOMING_FAIL_DISTANCE_MIN
} homing_dual_axis_t;

typedef struct {
    float feed_rate;
    float seek_rate;
    float pulloff;
    axes_signals_t dir_mask;
    uint8_t locate_cycles;
    uint16_t debounce_delay;
    homing_settings_flags_t flags;
    axes_signals_t cycle[N_AXIS];
//    homing_dual_axis_t dual_axis;
} homing_settings_t;

typedef struct {
    axes_signals_t step_invert;
    axes_signals_t dir_invert;
    axes_signals_t enable_invert;
    axes_signals_t deenergize;
    float pulse_microseconds;
    float pulse_delay_microseconds;
    uint16_t idle_lock_time; // If value = 255, steppers do not disable.
} stepper_settings_t;

typedef struct {
    float steps_per_mm;
    float max_rate;
    float acceleration;
    float max_travel;
#ifdef ENABLE_BACKLASH_COMPENSATION
    float backlash;
#endif
} axis_settings_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t hard_enabled     :1,
                soft_enabled     :1,
                check_at_init    :1,
                jog_soft_limited :1,
                two_switches     :1,
                unassigned       :3;
    };
} limit_settings_flags_t;

typedef struct {
    limit_settings_flags_t flags;
    axes_signals_t invert;
    axes_signals_t disable_pullup;
} limit_settings_t;

typedef union {
    uint8_t value;
    uint8_t mask;
    struct {
        uint8_t bit0 :1,
                bit1 :1,
                bit2 :1,
                bit3 :1,
                bit4 :1,
                bit5 :1,
                bit6 :1,
                bit7 :1;
    };
} ioport_bus_t;

typedef struct {
    ioport_bus_t invert_in;
    ioport_bus_t pullup_disable_in;
    ioport_bus_t invert_out;
    ioport_bus_t od_enable_out;
} ioport_signals_t;

typedef enum {
    Mode_Standard = 0,
    Mode_Laser,
    Mode_Lathe
} machine_mode_t;

typedef enum {
    ToolChange_Disabled = 0,
    ToolChange_Manual,
    ToolChange_Manual_G59_3,
    ToolChange_SemiAutomatic,
    ToolChange_Ignore
} toolchange_mode_t;

typedef struct {
    float feed_rate;
    float seek_rate;
    float probing_distance;
    toolchange_mode_t mode;
} tool_change_settings_t;

// Global persistent settings (Stored from byte persistent storage_ADDR_GLOBAL onwards)
typedef struct {
    // Settings struct version
    uint32_t version;
    float junction_deviation;
    float arc_tolerance;
    float g73_retract;
    machine_mode_t mode;
    tool_change_settings_t tool_change;
    axis_settings_t axis[N_AXIS];
    control_signals_t control_invert;
    control_signals_t control_disable_pullup;
    coolant_state_t coolant_invert;
    spindle_settings_t spindle;
    stepper_settings_t steppers;
    reportmask_t status_report; // Mask to indicate desired report data.
    settingflags_t flags;       // Contains default boolean settings
    probeflags_t probe;
    homing_settings_t homing;
    limit_settings_t limits;
    parking_settings_t parking;
    position_pid_t position;    // Used for synchronized motion
    ioport_signals_t ioport;
    uint8_t fire_log_enable;
    uint32_t fire_alarm_delta_threshold;   	//比较阈值
    uint32_t fire_alarm_time_threshold;    	//次数阈值
    uint32_t laser_used_time;				//激光器使用时长
#if ENABLE_HOMING_FORCE_SET_ORIGIN_OFFSET
    int16_t origin_offset_x;
    int16_t origin_offset_y;
    int16_t origin_offset_z;
#endif
#if ENABLE_ACCELERATION_DETECT
    uint16_t accel_sensitivity;
#endif
    uint16_t sys_auto_poweroff_time;
    uint16_t laser_focal_length;  			//焦距
    uint16_t laser_control_mode;
    uint16_t iic_rate;						//数字激光通信速率，单位KHZ
    uint8_t echo_enable;
    uint8_t power_log_enable;				//打印电源电压，电流信息
    uint32_t uart_baudrate;					//串口波特率
} settings_t;

extern settings_t settings;

// Initialize the configuration subsystem (load settings from persistent storage)
void settings_init();

// Helper function to clear and restore persistent storage defaults
void settings_restore(settings_restore_t restore_flags);

// A helper method to set new settings from command line
status_code_t settings_store_global_setting(setting_type_t setting, char *svalue);

// Writes the protocol line variable as a startup line in persistent storage
void settings_write_startup_line(uint8_t idx, char *line);

// Reads an persistent storage startup line to the protocol line variable
bool settings_read_startup_line(uint8_t idx, char *line);

// Writes build info user-defined string
void settings_write_build_info(char *line);

// Reads build info user-defined string
bool settings_read_build_info(char *line);

// Writes selected coordinate data to persistent storage
void settings_write_coord_data(coord_system_id_t id, float (*coord_data)[N_AXIS]);

// Reads selected coordinate data from persistent storage
bool settings_read_coord_data(coord_system_id_t id, float (*coord_data)[N_AXIS]);

// Writes selected tool data to persistent storage
bool settings_write_tool_data (tool_data_t *tool_data);

// Read selected tool data from persistent storage
bool settings_read_tool_data (uint32_t tool, tool_data_t *tool_data);

void write_global_settings ();//添加函数声明

/*恢复出厂设置*/
void coord_data_restore(void);
#endif

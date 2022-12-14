/*
  system.h - Header for system level commands and real-time processes

  Part of GrblHAL

  Copyright (c) 2017-2020 Terje Io
  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC

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

#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#include "gcode.h"
#include "probe.h"

// Define system executor bit map. Used internally by realtime protocol as realtime command flags,
// which notifies the main program to execute the specified realtime command asynchronously.
// NOTE: The system executor uses an unsigned 16-bit volatile variable (16 flag limit.) The default
// flags are always false, so the realtime protocol only needs to check for a non-zero value to
// know when there is a realtime command to execute.
#define EXEC_STATUS_REPORT  bit(0)
#define EXEC_CYCLE_START    bit(1)
#define EXEC_CYCLE_COMPLETE bit(2)
#define EXEC_FEED_HOLD      bit(3)
#define EXEC_STOP           bit(4)
#define EXEC_RESET          bit(5)
#define EXEC_SAFETY_DOOR    bit(6)
#define EXEC_MOTION_CANCEL  bit(7)
#define EXEC_SLEEP          bit(8)
#define EXEC_TOOL_CHANGE    bit(9)
#define EXEC_PID_REPORT     bit(10)
#define EXEC_GCODE_REPORT   bit(11)
#define EXEC_TLO_REPORT     bit(12)
#define EXEC_RT_COMMAND     bit(13)

// Define system state bit map. The state variable primarily tracks the individual functions
// of Grbl to manage each without overlapping. It is also used as a messaging flag for
// critical events.
// NOTE: flags are mutually exclusive, bit map allows testing for multiple states (except STATE_IDLE) in a single statement
#define STATE_IDLE          0      // Must be zero. No flags.
#define STATE_ALARM         bit(0) // In alarm state. Locks out all g-code processes. Allows settings access.
#define STATE_CHECK_MODE    bit(1) // G-code check mode. Locks out planner and motion only.
#define STATE_HOMING        bit(2) // Performing homing cycle
#define STATE_CYCLE         bit(3) // Cycle is running or motions are being executed.
#define STATE_HOLD          bit(4) // Active feed hold
#define STATE_JOG           bit(5) // Jogging mode.
#define STATE_SAFETY_DOOR   bit(6) // Safety door is ajar. Feed holds and de-energizes system.
#define STATE_SLEEP         bit(7) // Sleep state.
#define STATE_ESTOP         bit(8) // EStop mode, reports and is mainly handled similar to alarm state
#define STATE_TOOL_CHANGE   bit(9) // Manual tool change, similar to STATE_HOLD - but stops spindle and allows jogging.

// Define Grbl feedback message codes. Valid values (0-255).
typedef enum {
    Message_None = 0, // Reserved, do not change value
    Message_CriticalEvent = 1,
    Message_AlarmLock = 2,
    Message_AlarmUnlock = 3,
    Message_Enabled = 4,
    Message_Disabled = 5,
    Message_SafetyDoorAjar = 6,
    Message_CheckLimits = 7,
    Message_ProgramEnd = 8,
    Message_RestoreDefaults = 9,
    Message_SpindleRestore = 10,
    Message_SleepMode = 11,
    Message_EStop = 12,
    Message_HomingCycleRequired = 13,
    Message_CycleStartToRerun = 14,
    Message_ReferenceTLOEstablished = 15,

	/*????????????????????????????????????12V?????????*/
	Message_PowerSupplied = 16,
	Message_NoPowerSupply = 17,

    Message_NextMessage // Next unassigned message number
} message_code_t;

// Alarm executor codes. Valid values (1-255). Zero is reserved.
typedef enum {
    Alarm_None = 0,
    Alarm_HardLimit = 1,
    Alarm_SoftLimit = 2,
    Alarm_AbortCycle = 3,
    Alarm_ProbeFailInitial = 4,
    Alarm_ProbeFailContact = 5,
    Alarm_HomingFailReset = 6,
    Alarm_HomingFailDoor = 7,
    Alarm_FailPulloff = 8,
    Alarm_HomingFailApproach = 9,
    Alarm_EStop = 10,
    Alarm_HomingRequried = 11,
    Alarm_LimitsEngaged = 12,
    Alarm_ProbeProtect = 13,
    Alarm_Spindle = 14,
    Alarm_HomingFailAutoSquaringApproach = 15
} alarm_code_t;

typedef enum {
    Parking_DoorClosed = 0,
    Parking_DoorAjar,
    Parking_Retracting,
    Parking_Cancel,
    Parking_Resuming
} parking_state_t;

typedef enum {
    Hold_NotHolding = 0,
    Hold_Complete = 1,
    Hold_Pending = 2
} hold_state_t;

// Define step segment generator state flags.
typedef union {
    uint8_t flags;
    struct {
        uint8_t end_motion         :1,
                execute_hold       :1,
                execute_sys_motion :1,
                update_spindle_rpm :1,
                unassigned         :4;
    };
} step_control_t;


typedef union {
    uint16_t value;
    uint16_t mask;
    struct {
        uint16_t reset              :1,
                 feed_hold          :1,
                 cycle_start        :1,
                 safety_door_ajar   :1,
                 block_delete       :1,
                 stop_disable       :1, // M1
                 e_stop             :1,
                 probe_disconnected :1,
                 motor_fault        :1,
                 motor_warning      :1,
                 unassigned         :4,
                 probe_triggered    :1, // used for probe protection
                 deasserted         :1; // this flag is set if signals are deasserted. Note: do NOT pass on to Grbl control_interrupt_handler if set.
    };
} control_signals_t;


// Define spindle stop override control states.
typedef union {
    uint8_t value;
    struct {
        uint8_t enabled       :1,
                initiate      :1,
                restore       :1,
                restore_cycle :1,
                unassigned    :4;
    };
} spindle_stop_t;

#ifdef PID_LOG

typedef struct {
    uint_fast16_t idx;
    float setpoint;
    float t_sample;
    float target[PID_LOG];
    float actual[PID_LOG];
} pid_data_t;

#endif

typedef union {
    uint16_t value;
    struct {
        uint16_t mpg_mode      :1, // MPG mode changed.
                 scaling       :1, // Scaling (G50/G51) changed.
                 homed         :1, // Homed state changed.
                 xmode         :1, // Lathe radius/diameter mode changed.
                 spindle       :1, // Spindle state changed.
                 coolant       :1, // Coolant state changed.
                 overrides     :1, // Overrides changed.
                 tool          :1, // Tool changed.
                 wco           :1, // Add work coordinates.
                 gwco          :1, // Add work coordinate.
                 tool_offset   :1, // Tool offsets changed.
                 pwm           :1, // Add PWM information (optional: to be added by driver).
                 motor         :1, // Add motor information (optional: to be added by driver).
                 encoder       :1, // Add encoder information (optional: to be added by driver).
                 tlo_reference :1, // Tool length offset reference changed
                 all           :1; // Set when CMD_STATUS_REPORT_ALL is requested, may be used by user code
    };
} report_tracking_flags_t;

typedef struct {
    uint8_t feed_rate;              // Feed rate override value in percent
    uint8_t rapid_rate;             // Rapids override value in percent
    uint8_t spindle_rpm;            // Spindle speed override value in percent
    spindle_stop_t spindle_stop;    // Tracks spindle stop override states
    gc_override_flags_t control;    // Tracks override control states.
} overrides_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t mpg_mode              :1, // MPG mode flag. Set when switched to secondary input stream. (unused for now)
                probe_succeeded       :1, // Tracks if last probing cycle was successful.
                soft_limit            :1, // Tracks soft limit errors for the state machine.
                exit                  :1, // System exit flag. Used in combination with abort to terminate main loop.
                block_delete_enabled  :1, // Set to true to enable block delete
                feed_hold_pending     :1,
                delay_overrides       :1,
                optional_stop_disable :1; // Set to true to disable M1 (optional stop), via realtime command
    };
} system_flags_t;

// Define global system variables
typedef struct {
    uint_fast16_t state;                // Tracks the current system state of Grbl.
                                        // NOTE: Setting the state variable directly is NOT allowed! Use the set_state() function!
    bool ready;                         // System ready flag.
    bool abort;                         // System abort flag. Forces exit back to main loop for reset.
    bool cancel;                        // System cancel flag.
    bool suspend;                       // System suspend state flag.
    volatile bool steppers_deenergize;  // Set to true to deenergize stepperes
    bool mpg_mode;                      // To be moved to system_flags_t
    axes_signals_t tlo_reference_set;   // Axes with tool length reference offset set
    int32_t tlo_reference[N_AXIS];      // Tool length reference offset
    alarm_code_t alarm_pending;         // Delayed alarm, currently used for probe protection
    system_flags_t flags;               // Assorted state flags
    step_control_t step_control;        // Governs the step segment generator depending on system state.
    axes_signals_t homing_axis_lock;    // Locks axes when limits engage. Used as an axis motion mask in the stepper ISR.
    axes_signals_t homing;              // Axes with homing enabled.
    axes_signals_t homed;               // Indicates which axes has been homed.
    overrides_t override;               // Override values & states
    report_tracking_flags_t report;     // Tracks when to add data to status reports.
    parking_state_t parking_state;      // Tracks parking state
    hold_state_t holding_state;         // Tracks holding state
    float home_position[N_AXIS];        // Home position for homed axes
    float spindle_rpm;
#ifdef PID_LOG
    pid_data_t pid_log;
#endif
} system_t;

extern system_t sys;

// NOTE: These position variables may need to be declared as volatiles, if problems arise.
extern int32_t sys_position[N_AXIS];      // Real-time machine (aka home) position vector in steps.
extern int32_t sys_probe_position[N_AXIS]; // Last probe position in machine coordinates and steps.

extern volatile probing_state_t sys_probing_state; // Probing state value.  Used to coordinate the probing cycle with stepper ISR.
extern volatile uint_fast16_t sys_rt_exec_state;   // Global realtime executor bitflag variable for state management. See EXEC bitmasks.
extern volatile uint_fast16_t sys_rt_exec_alarm;   // Global realtimeate val executor bitflag variable for setting various alarms.

// Executes an internal system command, defined as a string starting with a '$'
status_code_t system_execute_line(char *line);

// Execute the startup script lines stored in non-volatile storage upon initialization
void system_execute_startup(char *line);

void system_flag_wco_change();

// Returns machine position of axis 'idx'. Must be sent a 'step' array.
//float system_convert_axis_steps_to_mpos(int32_t *steps, uint_fast8_t idx);

// Updates a machine 'position' array based on the 'step' array sent.
void system_convert_array_steps_to_mpos(float *position, int32_t *steps);

// Checks and reports if target array exceeds machine travel limits.
bool system_check_travel_limits(float *target);

// Checks and limit jog commands to within machine travel limits.
void system_apply_jog_limits (float *target);

// Special handlers for setting and clearing Grbl's real-time execution flags.
#define system_set_exec_state_flag(mask) hal.set_bits_atomic(&sys_rt_exec_state, (mask))
#define system_clear_exec_state_flag(mask) hal.clear_bits_atomic(&sys_rt_exec_state, (mask))
#define system_clear_exec_states() hal.set_value_atomic(&sys_rt_exec_state, 0)
#define system_set_exec_alarm(code) hal.set_value_atomic(&sys_rt_exec_alarm, (uint_fast16_t)(code))
#define system_clear_exec_alarm() hal.set_value_atomic(&sys_rt_exec_alarm, 0)

void control_interrupt_handler (control_signals_t signals);

#endif

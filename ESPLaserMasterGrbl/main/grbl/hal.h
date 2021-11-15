/*
  hal.h - HAL (Hardware Abstraction Layer) entry points structures and capabilities type

  Part of GrblHAL

  Copyright (c) 2016-2020 Terje Io

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

#ifndef _HAL_H_
#define _HAL_H_

#include "grbl.h"
#include "gcode.h"
#include "system.h"
#include "coolant_control.h"
#include "spindle_control.h"
#include "stepper.h"
#include "nvs.h"
#include "stream.h"
#include "probe.h"
#include "plugins.h"
#include "settings.h"
#include "report.h"

#define HAL_VERSION 7

// driver capabilities, to be set by driver in driver_init(), flags may be cleared after to switch off option
typedef union {
    uint32_t value;
    struct {
        uint32_t mist_control              :1,
                 variable_spindle          :1,
                 safety_door               :1,
                 spindle_dir               :1,
                 software_debounce         :1,
                 step_pulse_delay          :1,
                 limits_pull_up            :1,
                 control_pull_up           :1,
                 probe_pull_up             :1,
                 amass_level               :2, // 0...3
                 program_stop              :1,
                 block_delete              :1,
                 e_stop                    :1,
                 spindle_at_speed          :1,
                 laser_ppi_mode            :1,
                 spindle_sync              :1,
                 sd_card                   :1,
                 bluetooth                 :1,
                 ethernet                  :1,
                 wifi                      :1,
                 spindle_pwm_invert        :1,
                 spindle_pid               :1,
                 mpg_mode                  :1,
                 spindle_pwm_linearization :1,
                 probe_connected           :1,
                 atc                       :1,
                 no_gcode_message_handling :1,
                 unassigned                :4;
    };
} driver_cap_t;

typedef void (*driver_reset_ptr)(void);

// I/O stream

typedef void (*stream_write_ptr)(const char *s);
typedef bool (*enqueue_realtime_command_ptr)(char data);

typedef struct {
    stream_type_t type;
    bool switchable;  // Indicates whether the IO steam can be switched.
    uint16_t (*get_rx_buffer_available)(void);
//    bool (*stream_write)(char c);
    stream_write_ptr write;     // write string to current I/O stream only.
    stream_write_ptr write_all; // write string to all active output streams.
    int16_t (*read)(void);
    void (*write_buffer_size)(void);
    void (*reset_read_buffer)(void);
    void (*cancel_read_buffer)(void);
    bool (*suspend_read)(bool await);
    enqueue_realtime_command_ptr enqueue_realtime_command; // NOTE: set by grbl at startup.
} io_stream_t;

typedef struct {
    uint8_t num_digital_in;
    uint8_t num_digital_out;
    uint8_t num_analog_in;
    uint8_t num_analog_out;
    void (*digital_out)(uint8_t port, bool on);
    bool (*analog_out)(uint8_t port, float value);
    int32_t (*wait_on_input)(bool digital, uint8_t port, wait_mode_t wait_mode, float timeout);
} io_port_t;

// Spindle

typedef void (*spindle_set_state_ptr)(spindle_state_t state, float rpm);
typedef spindle_state_t (*spindle_get_state_ptr)(void);
#ifdef SPINDLE_PWM_DIRECT
typedef uint_fast16_t (*spindle_get_pwm_ptr)(float rpm);
typedef void (*spindle_update_pwm_ptr)(uint_fast16_t pwm);
#else
typedef void (*spindle_update_rpm_ptr)(float rpm);
#endif
typedef spindle_data_t (*spindle_get_data_ptr)(spindle_data_request_t request);
typedef void (*spindle_reset_data_ptr)(void);
typedef void (*spindle_pulse_on_ptr)(uint_fast16_t pulse_length);

typedef struct {
    spindle_set_state_ptr set_state;
    spindle_get_state_ptr get_state;
#ifdef SPINDLE_PWM_DIRECT
    spindle_get_pwm_ptr get_pwm;
    spindle_update_pwm_ptr update_pwm;
#else
    spindle_update_rpm_ptr update_rpm;
#endif
    // Optional entry points:
    spindle_get_data_ptr get_data;
    spindle_reset_data_ptr reset_data;
    spindle_pulse_on_ptr pulse_on;
} spindle_ptrs_t;

// Coolant

typedef void (*coolant_set_state_ptr)(coolant_state_t mode);
typedef coolant_state_t (*coolant_get_state_ptr)(void);

typedef struct {
    coolant_set_state_ptr set_state;
    coolant_get_state_ptr get_state;
} coolant_ptrs_t;

// Limits

typedef void (*limits_enable_ptr)(bool on, bool homing);
typedef axes_signals_t (*limits_get_state_ptr)(void);
typedef void (*limit_interrupt_callback_ptr)(axes_signals_t state);

typedef struct {
    limits_enable_ptr enable;
    limits_get_state_ptr get_state;
    limit_interrupt_callback_ptr interrupt_callback; // set up by core before driver_init() is called.
} limits_ptrs_t;

// Control signals

typedef control_signals_t (*control_signals_get_state_ptr)(void);
typedef void (*control_signals_callback_ptr)(control_signals_t signals);

typedef struct {
    control_signals_get_state_ptr get_state;
    control_signals_callback_ptr interrupt_callback; // set up by core before driver_init() is called.
} control_signals_ptrs_t;

// Steppers

typedef void (*stepper_wake_up_ptr)(void);
typedef void (*stepper_go_idle_ptr)(bool clear_signals);
typedef void (*stepper_enable_ptr)(axes_signals_t enable);
typedef void (*stepper_disable_motors_ptr)(axes_signals_t axes, squaring_mode_t mode);
typedef void (*stepper_cycles_per_tick_ptr)(uint32_t cycles_per_tick);
typedef void (*stepper_pulse_start_ptr)(stepper_t *stepper);
typedef void (*stepper_output_step_ptr)(axes_signals_t step_outbits, axes_signals_t dir_outbits);
typedef axes_signals_t (*stepper_get_auto_squared_ptr)(void);
typedef void (*stepper_interrupt_callback_ptr)(void);

typedef struct {
    stepper_wake_up_ptr wake_up;
    stepper_go_idle_ptr go_idle;
    stepper_enable_ptr enable;
    stepper_disable_motors_ptr disable_motors;
    stepper_cycles_per_tick_ptr cycles_per_tick;
    stepper_pulse_start_ptr pulse_start;
    stepper_interrupt_callback_ptr interrupt_callback; // set up by core before driver_init() is called.
    // Optional entry points:
    stepper_get_auto_squared_ptr get_auto_squared;
    stepper_output_step_ptr output_step;
} stepper_ptrs_t;

// Driver/plugin settings (optional)

typedef status_code_t (*driver_setting_ptr)(setting_type_t setting, float value, char *svalue);
typedef void (*settings_changed_ptr)(settings_t *settings);
typedef void (*driver_settings_report_ptr)(setting_type_t setting_type);
typedef void (*driver_axis_settings_report_ptr)(axis_setting_type_t setting_type, uint8_t axis_idx);
typedef void (*driver_settings_load_ptr)(void);
typedef void (*driver_settings_restore_ptr)(void);

typedef struct {
    uint32_t nvs_address;
    driver_setting_ptr set;
    settings_changed_ptr changed;
    driver_settings_report_ptr report;
    driver_axis_settings_report_ptr axis_report;
    driver_settings_load_ptr load;
    driver_settings_restore_ptr restore;
} driver_setting_ptrs_t;

// Probe (optional)

typedef probe_state_t (*probe_get_state_ptr)(void);
typedef void (*probe_configure_ptr)(bool is_probe_away, bool probing);
typedef void (*probe_connected_toggle_ptr)(void);

typedef struct {
    probe_configure_ptr configure;
    probe_get_state_ptr get_state;
    probe_connected_toggle_ptr connected_toggle;
} probe_ptrs_t;

typedef void (*tool_select_ptr)(tool_data_t *tool, bool next);
typedef status_code_t (*tool_change_ptr)(parser_state_t *gc_state);

typedef struct {
    tool_select_ptr select;
    tool_change_ptr change;
} tool_ptrs_t;

// User M-codes (optional)

typedef user_mcode_t (*user_mcode_check_ptr)(user_mcode_t mcode);
typedef status_code_t (*user_mcode_validate_ptr)(parser_block_t *gc_block, uint32_t *value_words);
typedef void (*user_mcode_execute_ptr)(uint_fast16_t state, parser_block_t *gc_block);

typedef struct {
    user_mcode_check_ptr check;
    user_mcode_validate_ptr validate;
    user_mcode_execute_ptr execute;
} user_mcode_ptrs_t;

// Encoder (optional)

typedef void (*encoder_on_event_ptr)(encoder_t *encoder, int32_t position);
typedef void (*encoder_reset_ptr)(uint_fast8_t id);

typedef struct {
    encoder_on_event_ptr on_event;
    encoder_reset_ptr reset;
} encoder_ptrs_t;

//

// main HAL structure

typedef struct {
    uint32_t version;
    char *info;
    char *driver_version;
    char *driver_options;
    char *board;
    uint32_t f_step_timer;
    uint32_t rx_buffer_size;

    void (*watchdog_feed)(void);
    bool (*driver_setup)(settings_t *settings);
    void (*delay_ms)(uint32_t ms, void (*callback)(void));
    void (*set_bits_atomic)(volatile uint_fast16_t *value, uint_fast16_t bits);
    uint_fast16_t (*clear_bits_atomic)(volatile uint_fast16_t *value, uint_fast16_t bits);
    uint_fast16_t (*set_value_atomic)(volatile uint_fast16_t *value, uint_fast16_t bits);
    void (*irq_enable)(void);
    void (*irq_disable)(void);

    limits_ptrs_t limits;
    coolant_ptrs_t coolant;
    spindle_ptrs_t spindle;
    stepper_ptrs_t stepper;
    control_signals_ptrs_t control;
    io_stream_t stream;
    settings_changed_ptr settings_changed;

 //
 // optional entry points, may be unassigned (null)
 //
    bool (*driver_release)(void);
    bool (*get_position)(int32_t (*position)[N_AXIS]);
    uint32_t (*get_elapsed_ticks)(void);
    void (*pallet_shuttle)(void);
    void (*reboot)(void);
#ifdef DEBUGOUT
    void (*debug_out)(bool on);
#endif

    probe_ptrs_t probe;
    user_mcode_ptrs_t user_mcode;
    driver_reset_ptr driver_reset;
    driver_setting_ptrs_t driver_settings;
    tool_ptrs_t tool;
    encoder_ptrs_t encoder;
    nvs_io_t nvs;
    io_port_t port;

    bool (*stream_blocking_callback)(void); // set up by core before driver_init() is called.

    // driver capabilities flags
    driver_cap_t driver_cap;

} grbl_hal_t;

// TODO: move the following structs to grbl.h?

/* TODO: add to grbl pointers so that a different formatting (xml, json etc) of reports may be implemented by driver?
typedef struct {
    status_code_t (*report_status_message)(status_code_t status_code);
    alarm_code_t (*report_alarm_message)(alarm_code_t alarm_code);
    message_code_t (*report_feedback_message)(message_code_t message_code);
    void (*report_init_message)(void);
    void (*report_grbl_help)(void);
    void (*report_grbl_settings)(void);
    void (*report_echo_line_received)(char *line);
    void (*report_realtime_status)(void);
    void (*report_probe_parameters)(void);
    void (*report_ngc_parameters)(void);
    void (*report_gcode_modes)(void);
    void (*report_startup_line)(uint8_t n, char *line);
    void (*report_execute_startup_message)(char *line, status_code_t status_code);
} grbl_report_t;
*/

// Report entry points set by core at reset.

typedef status_code_t (*status_message_ptr)(status_code_t status_code);
typedef message_code_t (*feedback_message_ptr)(message_code_t message_code);

typedef struct {
    status_message_ptr status_message;
    feedback_message_ptr feedback_message;
} report_t;

// Core event handler and other entry points.
// Most of the event handlers defaults to NULL, a few is set up to call a dummy handler for simpler code.

typedef void (*on_state_change_ptr)(uint_fast16_t state);
typedef void (*on_probe_completed_ptr)(void);
typedef void (*on_program_completed_ptr)(program_flow_t program_flow);
typedef void (*on_execute_realtime_ptr)(uint_fast16_t state);
typedef void (*on_unknown_accessory_override_ptr)(uint8_t cmd);
typedef void (*on_report_options_ptr)(void);
typedef void (*on_realtime_report_ptr)(stream_write_ptr stream_write, report_tracking_flags_t report);
typedef void (*on_unknown_feedback_message_ptr)(stream_write_ptr stream_write);
typedef bool (*on_laser_ppi_enable_ptr)(uint_fast16_t ppi, uint_fast16_t pulse_length);
typedef status_code_t (*on_unknown_sys_command_ptr)(uint_fast16_t state, char *line, char *lcline); // return Status_Unhandled.
typedef status_code_t (*on_user_command_ptr)(char *line);

typedef struct {
    // report entry points set by core at reset.
    report_t report;
    // grbl core events - may be subscribed to by drivers or by the core.
    on_state_change_ptr on_state_change;
    on_probe_completed_ptr on_probe_completed;
    on_program_completed_ptr on_program_completed;
    on_execute_realtime_ptr on_execute_realtime;
    on_unknown_accessory_override_ptr on_unknown_accessory_override;
    on_report_options_ptr on_report_options;
    on_realtime_report_ptr on_realtime_report;
    on_unknown_feedback_message_ptr on_unknown_feedback_message;
    on_unknown_sys_command_ptr on_unknown_sys_command; // return Status_Unhandled if not handled.
    on_user_command_ptr on_user_command;
    on_laser_ppi_enable_ptr on_laser_ppi_enable;
    // core entry points - set up by core before driver_init() is called.
    bool (*protocol_enqueue_gcode)(char *data);
} grbl_t;

extern grbl_t grbl;
extern grbl_hal_t hal;
extern bool driver_init (void);

#endif

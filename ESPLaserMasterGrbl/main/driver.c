/*
  driver.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for ESP32

  Part of grblHAL

  Copyright (c) 2018-2021 Terje Io

  Some parts
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "grbl/limits.h"

#include "driver.h"
#include "esp32-hal-uart.h"
#include "nvs.h"
#include "grbl/protocol.h"
#include "esp_log.h"
#include "driver/ledc.h"
//#include "grbl_esp32_if/grbl_esp32_if.h"

#include "board.h"
#include "grbl/state_machine.h"
#include "usb_serial.h"
#include "my_machine_map.h"
#include "accelDetection.h"
#include "esp_adc_cal.h"
#include "driver/adc.h"

#ifdef USE_I2S_OUT
#include "i2s_out.h"
#endif

#if WIFI_ENABLE
#include "wifi.h"
#endif

#if WEBUI_ENABLE
#include "webui/response.h"
#endif

#if TELNET_ENABLE
#include "networking/TCPStream.h"
#endif

#if WEBSOCKET_ENABLE
#include "networking/WsStream.h"
#endif

#if BLUETOOTH_ENABLE
#include "bluetooth.h"
#endif

#if SDCARD_ENABLE
#include "sdcard/sdcard.h"
#include "esp_vfs_fat.h"
#endif

#if KEYPAD_ENABLE
#include "keypad/keypad.h"
#endif

#if IOEXPAND_ENABLE
#include "ioexpand.h"
#endif

#if EEPROM_ENABLE
#include "eeprom/eeprom.h"
#endif

#if I2C_ENABLE
#include "i2c.h"
#endif

#ifndef VFD_SPINDLE
static uint32_t pwm_max_value;
static bool pwmEnabled = false;
static spindle_pwm_t spindle_pwm;
#else
#undef SPINDLE_RPM_CONTROLLED
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

// prescale step counter to 20Mhz
#define STEPPER_DRIVER_PRESCALER 4

#if PWM_RAMPED

#define SPINDLE_RAMP_STEP_INCR 20 // timer compare register change per ramp step
#define SPINDLE_RAMP_STEP_TIME 2  // ms

typedef struct {
    volatile uint32_t ms_cfg;
    volatile uint32_t ms_count;
    uint32_t pwm_current;
    uint32_t pwm_target;
    uint32_t pwm_step;
} pwm_ramp_t;

static pwm_ramp_t pwm_ramp;
#endif

typedef enum {
    Input_Probe = 0,
    Input_Reset,
    Input_FeedHold,
    Input_CycleStart,
    Input_SafetyDoor,
    Input_ModeSelect,
    Input_LimitX,
    Input_LimitX_Max,
    Input_LimitY,
    Input_LimitY_Max,
    Input_LimitZ,
    Input_LimitZ_Max,
    Input_LimitA,
    Input_LimitA_Max,
    Input_LimitB,
    Input_LimitB_Max,
    Input_LimitC,
    Input_LimitC_Max,
    Input_KeypadStrobe
} input_t;

typedef struct {
    input_t id;
    uint8_t pin;
    uint8_t group;
    uint32_t mask;
    uint8_t offset;
    bool invert;
    volatile bool active;
    volatile bool debounce;
} state_signal_t;


#if MPG_MODE_ENABLE
static io_stream_t prev_stream = {0};
#endif

const io_stream_t serial_stream = {
    .type = StreamType_Serial,
    .read = serialRead,
    .write = serialWriteS,
    .write_all = serialWriteS,
    .get_rx_buffer_available = serialRXFree,
    .reset_read_buffer = serialFlush,
    .cancel_read_buffer = serialCancel,
    .suspend_read = serialSuspendInput,
    .enqueue_realtime_command = protocol_enqueue_realtime_command
};

#if WIFI_ENABLE

static network_services_t services = {0};

void tcpStreamWriteS (const char *data)
{
#if TELNET_ENABLE
    if(services.telnet)
        TCPStreamWriteS(data);
#endif
#if WEBSOCKET_ENABLE
    if(services.websocket)
        WsStreamWriteS(data);
#endif
    serialWriteS(data);
}

#if TELNET_ENABLE
const io_stream_t telnet_stream = {
    .type = StreamType_Telnet,
    .read = TCPStreamGetC,
    .write = TCPStreamWriteS,
    .write_all = tcpStreamWriteS,
    .get_rx_buffer_available = TCPStreamRxFree,
    .reset_read_buffer = TCPStreamRxFlush,
    .cancel_read_buffer = TCPStreamRxCancel,
    .suspend_read = TCPStreamSuspendInput,
    .enqueue_realtime_command = protocol_enqueue_realtime_command
};
#endif

#if WEBSOCKET_ENABLE
const io_stream_t websocket_stream = {
    .type = StreamType_WebSocket,
    .read = WsStreamGetC,
    .write = WsStreamWriteS,
    .write_all = tcpStreamWriteS,
    .get_rx_buffer_available = WsStreamRxFree,
    .reset_read_buffer = WsStreamRxFlush,
    .cancel_read_buffer = WsStreamRxCancel,
    .suspend_read = WsStreamSuspendInput,
    .enqueue_realtime_command = protocol_enqueue_realtime_command,
};
#endif

#endif // WIFI_ENABLE

#if BLUETOOTH_ENABLE
void btStreamWriteS (const char *data)
{
    BTStreamWriteS(data);
    serialWriteS(data);
}

const io_stream_t bluetooth_stream = {
    .type = StreamType_Bluetooth,
    .read = BTStreamGetC,
    .write = BTStreamWriteS,
    .write_all = btStreamWriteS,
    .get_rx_buffer_available = BTStreamRXFree,
    .reset_read_buffer = BTStreamFlush,
    .cancel_read_buffer = BTStreamCancel,
    .suspend_read = serialSuspendInput,
    .enqueue_realtime_command = protocol_enqueue_realtime_command
};

#endif

#define INPUT_GROUP_CONTROL (1 << 0)
#define INPUT_GROUP_PROBE   (1 << 1)
#define INPUT_GROUP_LIMIT   (1 << 2)
#define INPUT_GROUP_KEYPAD  (1 << 3)
#define INPUT_GROUP_MPG     (1 << 4)

state_signal_t inputpin[] = {
#ifdef RESET_PIN
    { .id = Input_Reset,        .pin = RESET_PIN,       .group = INPUT_GROUP_CONTROL },
#endif
#ifdef FEED_HOLD_PIN
    { .id = Input_FeedHold,     .pin = FEED_HOLD_PIN,   .group = INPUT_GROUP_CONTROL },
#endif
#ifdef CYCLE_START_PIN
    { .id = Input_CycleStart,   .pin = CYCLE_START_PIN, .group = INPUT_GROUP_CONTROL },
#endif
#ifdef SAFETY_DOOR_PIN
    { .id = Input_SafetyDoor,   .pin = SAFETY_DOOR_PIN, .group = INPUT_GROUP_CONTROL },
#endif
#ifdef PROBE_PIN
    { .id = Input_Probe,        .pin = PROBE_PIN,       .group = INPUT_GROUP_PROBE },
#endif
    { .id = Input_LimitX,       .pin = X_LIMIT_PIN,     .group = INPUT_GROUP_LIMIT },
    { .id = Input_LimitY,       .pin = Y_LIMIT_PIN,     .group = INPUT_GROUP_LIMIT },
    { .id = Input_LimitZ,       .pin = Z_LIMIT_PIN,     .group = INPUT_GROUP_LIMIT }
#ifdef A_LIMIT_PIN
  , { .id = Input_LimitA,       .pin = A_LIMIT_PIN,     .group = INPUT_GROUP_LIMIT }
#endif
#ifdef B_LIMIT_PIN
  , { .id = Input_LimitB,       .pin = B_LIMIT_PIN,     .group = INPUT_GROUP_LIMIT }
#endif
#ifdef C_LIMIT_PIN
  , { .id = Input_LimitC,       .pin = C_LIMIT_PIN,     .group = INPUT_GROUP_LIMIT }
#endif
#if MPG_MODE_ENABLE
  , { .id = Input_ModeSelect,   .pin = MPG_ENABLE_PIN,  .group = INPUT_GROUP_MPG }
#endif
#if KEYPAD_ENABLE
  , { .id = Input_KeypadStrobe, .pin = KEYPAD_STROBE_PIN, .group = INPUT_GROUP_KEYPAD }
#endif
};

gpio_num_t outputpin[] =
{
#ifdef STEPPERS_DISABLE_PIN
    STEPPERS_DISABLE_PIN,
#endif
#if defined(SPINDLE_ENABLE_PIN) && SPINDLE_ENABLE_PIN != IOEXPAND
    SPINDLE_ENABLE_PIN,
#endif
#if defined(SPINDLE_DIRECTION_PIN) && SPINDLE_DIRECTION_PIN != IOEXPAND
    SPINDLE_DIRECTION_PIN,
#endif
#if defined(COOLANT_FLOOD_PIN) && COOLANT_FLOOD_PIN != IOEXPAND
    COOLANT_FLOOD_PIN,
#endif
#if defined(COOLANT_MIST_PIN) && COOLANT_MIST_PIN != IOEXPAND
    COOLANT_MIST_PIN,
#endif
    X_DIRECTION_PIN,
    Y_DIRECTION_PIN,
    Z_DIRECTION_PIN
};

static volatile uint32_t ms_count = 1; // NOTE: initial value 1 is for "resetting" systick timer
static bool IOInitDone = false;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
#if PROBE_ENABLE
static probe_state_t probe = {
    .connected = On
};
#endif

#ifdef USE_I2S_OUT
#define DIGITAL_IN(pin) i2s_out_state(pin)
#define DIGITAL_OUT(pin, state) i2s_out_write(pin, state)
uint32_t i2s_step_length = I2S_OUT_USEC_PER_PULSE, i2s_step_samples = 1;
#else
#define DIGITAL_IN(pin) gpio_get_level(pin)
#define DIGITAL_OUT(pin, state) gpio_set_level(pin, state)
#endif

#if IOEXPAND_ENABLE
static ioexpand_t iopins = {0};
#endif

#ifndef VFD_SPINDLE



static ledc_timer_config_t ledTimerConfig = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_10_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = 5000
};

static ledc_channel_config_t ledConfig = {
    .gpio_num = SPINDLEPWMPIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,  /*!< LEDC channel duty, the range of duty setting is [0, (2**duty_resolution)] */
    .hpoint = 0
};

#endif

#if MODBUS_ENABLE
static modbus_stream_t modbus_stream = {0};
#endif

// Interrupt handler prototypes
static void gpio_isr (void *arg);
static void stepper_driver_isr (void *arg);

static TimerHandle_t xDelayTimer = NULL, debounceTimer = NULL;

static void activateStream (const io_stream_t *stream)
{
#if MPG_MODE_ENABLE
    if(hal.stream.type == StreamType_MPG) {
        hal.stream.write_all = stream->write_all;
        if(prev_stream.reset_read_buffer != NULL)
            prev_stream.reset_read_buffer();
        memcpy(&prev_stream, stream, sizeof(io_stream_t));
    } else
#endif
        memcpy(&hal.stream, stream, sizeof(io_stream_t));
}

void selectStream (stream_type_t stream)
{
    static stream_type_t active_stream = StreamType_Serial;

    switch(stream) {

#if BLUETOOTH_ENABLE
        case StreamType_Bluetooth:
            activateStream(&bluetooth_stream);
//            services.bluetooth = On;
            break;
#endif

#if TELNET_ENABLE
        case StreamType_Telnet:
            hal.stream.write_all("[MSG:TELNET STREAM ACTIVE]\r\n");
            activateStream(&telnet_stream);
            services.telnet = On;
            break;
#endif

#if WEBSOCKET_ENABLE
        case StreamType_WebSocket:
            hal.stream.write_all("[MSG:WEBSOCKET STREAM ACTIVE]\r\n");
            activateStream(&websocket_stream);
            services.websocket = On;
            break;
#endif

        case StreamType_Serial:
            activateStream(&serial_stream);
#if WIFI_ENABLE
            services.mask = 0;
#endif
            if(active_stream != StreamType_Serial)
                hal.stream.write_all("[MSG:SERIAL STREAM ACTIVE]\r\n");
            break;

        default:
            break;
    }

    active_stream = stream;
}

void initRMT (settings_t *settings)
{
    rmt_item32_t rmtItem[2];

    rmt_config_t rmtConfig = {
        .rmt_mode = RMT_MODE_TX,
        .clk_div = 20,
        .mem_block_num = 1,
        .tx_config.loop_en = false,
        .tx_config.carrier_en = false,
        .tx_config.carrier_freq_hz = 0,
        .tx_config.carrier_duty_percent = 50,
        .tx_config.carrier_level = RMT_CARRIER_LEVEL_LOW,
        .tx_config.idle_output_en = true
    };

    rmtItem[0].duration0 = (uint32_t)(settings->steppers.pulse_delay_microseconds > 0.0f ? 4.0f * settings->steppers.pulse_delay_microseconds : 1.0f);
    rmtItem[0].duration1 = (uint32_t)(4.0f * settings->steppers.pulse_microseconds);
    rmtItem[1].duration0 = 0;
    rmtItem[1].duration1 = 0;

    uint32_t channel;
    for(channel = 0; channel < N_AXIS; channel++) {

        rmtConfig.channel = channel;

        switch(channel) {
            case 0:
                rmtConfig.tx_config.idle_level = settings->steppers.step_invert.x;
                rmtConfig.gpio_num = X_STEP_PIN;
                break;
            case 1:
                rmtConfig.tx_config.idle_level = settings->steppers.step_invert.y;
                rmtConfig.gpio_num = Y_STEP_PIN;
                break;
            case 2:
                rmtConfig.tx_config.idle_level = settings->steppers.step_invert.z;
                rmtConfig.gpio_num = Z_STEP_PIN;
                break;
        }
        rmtItem[0].level0 = rmtConfig.tx_config.idle_level;
        rmtItem[0].level1 = !rmtConfig.tx_config.idle_level;
        rmt_config(&rmtConfig);
        rmt_fill_tx_items(rmtConfig.channel, &rmtItem[0], 2, 0);
    }
}

void vTimerCallback (TimerHandle_t xTimer)
{
    void (*callback)(void) = (void (*)(void))pvTimerGetTimerID(xTimer);

    if(callback)
        callback();

    xTimerDelete(xDelayTimer, 3);
    xDelayTimer = NULL;
}

IRAM_ATTR static void driver_delay_ms (uint32_t ms, void (*callback)(void))
{
    if(callback) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if(xDelayTimer) {
            xTimerDelete(xDelayTimer, 3);
            xDelayTimer = NULL;
        }
        xDelayTimer = xTimerCreate("msDelay", pdMS_TO_TICKS(ms), pdFALSE, callback, vTimerCallback);
        xTimerStartFromISR(xDelayTimer, &xHigherPriorityTaskWoken);
        if(xHigherPriorityTaskWoken)
            portYIELD_FROM_ISR();
    } else {
        if(xDelayTimer) {
            xTimerDelete(xDelayTimer, 3);
            xDelayTimer = NULL;
        }
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
}

#ifdef DEBUGOUT
static void debug_out (bool enable)
{
    gpio_set_level(STEPPERS_DISABLE_PIN, enable);
}
#endif

// Set stepper direction output pins
// NOTE: see note for set_step_outputs()
inline IRAM_ATTR static void set_dir_outputs (axes_signals_t dir_outbits)
{
    dir_outbits.value ^= settings.steppers.dir_invert.mask;
    DIGITAL_OUT(X_DIRECTION_PIN, dir_outbits.x);
    DIGITAL_OUT(Y_DIRECTION_PIN, dir_outbits.y);
    DIGITAL_OUT(Z_DIRECTION_PIN, dir_outbits.z);
#ifdef A_AXIS
    DIGITAL_OUT(A_DIRECTION_PIN, dir_outbits.a);
#endif
#ifdef B_AXIS
    DIGITAL_OUT(B_DIRECTION_PIN, dir_outbits.b);
#endif
#ifdef C_AXIS
    DIGITAL_OUT(C_DIRECTION_PIN, dir_outbits.c);
#endif
}

// Enable/disable steppers
static void stepperEnable (axes_signals_t enable)
{
    enable.mask ^= settings.steppers.enable_invert.mask;

#if TRINAMIC_ENABLE && TRINAMIC_I2C
    axes_signals_t tmc_enable = trinamic_stepper_enable(enable);
 #if !CNC_BOOSTERPACK // Trinamic BoosterPack does not support mixed drivers
  #if IOEXPAND_ENABLE
    if(!tmc_enable.x)
        iopins.stepper_enable_x = enable.x;
    if(!tmc_enable.y)
        iopins.stepper_enable_y = enable.y;
    if(!tmc_enable.z)
        iopins.stepper_enable_z = enable.z;
  #endif
 #endif
#elif IOEXPAND_ENABLE // TODO: read from expander?
    iopins.stepper_enable_x = enable.x;
    iopins.stepper_enable_y = enable.y;
    iopins.stepper_enable_z = enable.z;
    ioexpand_out(iopins);
#elif defined(STEPPERS_DISABLE_PIN)
    DIGITAL_OUT(STEPPERS_DISABLE_PIN, enable.x);
#else
    DIGITAL_OUT(X_DISABLE_PIN, enable.x);
    DIGITAL_OUT(Y_DISABLE_PIN, enable.y);
    DIGITAL_OUT(Z_DISABLE_PIN, enable.z);
#ifdef A_AXIS
    DIGITAL_OUT(A_DISABLE_PIN, enable.a);
#endif
#ifdef B_AXIS
    DIGITAL_OUT(B_DISABLE_PIN, enable.b);
#endif
#ifdef C_AXIS
    DIGITAL_OUT(C_DISABLE_PIN, enable.c);
#endif
#endif
}

#ifdef USE_I2S_OUT

// Set stepper pulse output pins
inline __attribute__((always_inline)) IRAM_ATTR static void i2s_set_step_outputs (axes_signals_t step_outbits)
{
    step_outbits.value ^= settings.steppers.step_invert.mask;
    DIGITAL_OUT(X_STEP_PIN, step_outbits.x);
    DIGITAL_OUT(Y_STEP_PIN, step_outbits.y);
    DIGITAL_OUT(Z_STEP_PIN, step_outbits.z);
#ifdef A_AXIS
    DIGITAL_OUT(A_STEP_PIN, step_outbits.a);
#endif
#ifdef B_AXIS
    DIGITAL_OUT(B_STEP_PIN, step_outbits.b);
#endif
#ifdef C_AXIS
    DIGITAL_OUT(C_STEP_PIN, step_outbits.c);
#endif
}

IRAM_ATTR static void I2S_stepperGoIdle (bool clear_signals)
{
    if(clear_signals) {
        i2s_set_step_outputs((axes_signals_t){0});
        set_dir_outputs((axes_signals_t){0});
        i2s_out_reset();
    }

    i2s_out_set_passthrough();
}

IRAM_ATTR static void I2S_stepperCyclesPerTick (uint32_t cycles_per_tick)
{
    i2s_out_set_pulse_period(cycles_per_tick);
}

// Sets stepper direction and pulse pins and starts a step pulse
IRAM_ATTR static void I2S_stepperPulseStart (stepper_t *stepper)
{
    if(stepper->dir_change)
        set_dir_outputs(stepper->dir_outbits);

    if(stepper->step_outbits.value) {
        i2s_set_step_outputs(stepper->step_outbits);
        i2s_out_push_sample(i2s_step_samples);
        i2s_set_step_outputs((axes_signals_t){0});
    }
}

// Starts stepper driver ISR timer and forces a stepper driver interrupt callback
static void I2S_stepperWakeUp (void)
{
    // Enable stepper drivers.
    stepperEnable((axes_signals_t){AXES_BITMASK});
    i2s_out_set_stepping();
}

#else

// Set stepper pulse output pins
inline IRAM_ATTR static void set_step_outputs (axes_signals_t step_outbits)
{
    if(step_outbits.x) {
        RMT.conf_ch[0].conf1.mem_rd_rst = 1;
        RMT.conf_ch[0].conf1.tx_start = 1;
    }

    if(step_outbits.y) {
        RMT.conf_ch[1].conf1.mem_rd_rst = 1;
        RMT.conf_ch[1].conf1.tx_start = 1;
    }

    if(step_outbits.z) {
        RMT.conf_ch[2].conf1.mem_rd_rst = 1;
        RMT.conf_ch[2].conf1.tx_start = 1;
    }
}

#endif

// Starts stepper driver ISR timer and forces a stepper driver interrupt callback
static void stepperWakeUp (void)
{
    // Enable stepper drivers.
    stepperEnable((axes_signals_t){AXES_BITMASK});

    timer_set_counter_value(STEP_TIMER_GROUP, STEP_TIMER_INDEX, 0x00000000ULL);
//  timer_set_alarm_value(STEP_TIMER_GROUP, STEP_TIMER_INDEX, 5000ULL);
    TIMERG0.hw_timer[STEP_TIMER_INDEX].alarm_high = 0;
    TIMERG0.hw_timer[STEP_TIMER_INDEX].alarm_low = 5000UL;

    timer_start(STEP_TIMER_GROUP, STEP_TIMER_INDEX);
    TIMERG0.hw_timer[STEP_TIMER_INDEX].config.alarm_en = TIMER_ALARM_EN;
}

// Disables stepper driver interrupts
IRAM_ATTR static void stepperGoIdle (bool clear_signals)
{
    timer_pause(STEP_TIMER_GROUP, STEP_TIMER_INDEX);

    if(clear_signals) {
#ifdef USE_I2S_OUT
        i2s_set_step_outputs((axes_signals_t){0});
#else
        set_step_outputs((axes_signals_t){0});
#endif
        set_dir_outputs((axes_signals_t){0});
    }
}

// Sets up stepper driver interrupt timeout
IRAM_ATTR static void stepperCyclesPerTick (uint32_t cycles_per_tick)
{
// Limit min steps/s to about 2 (hal.f_step_timer @ 20MHz)
#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    TIMERG0.hw_timer[STEP_TIMER_INDEX].alarm_low = cycles_per_tick < (1UL << 18) ? cycles_per_tick : (1UL << 18) - 1UL;
#else
    TIMERG0.hw_timer[STEP_TIMER_INDEX].alarm_low = cycles_per_tick < (1UL << 23) ? cycles_per_tick : (1UL << 23) - 1UL;
#endif
}

// Sets stepper direction and pulse pins and starts a step pulse
IRAM_ATTR static void stepperPulseStart (stepper_t *stepper)
{
    if(stepper->dir_change)
        set_dir_outputs(stepper->dir_outbits);

    if(stepper->step_outbits.value) {
#ifdef USE_I2S_OUT
        uint64_t step_pulse_start_time = esp_timer_get_time();
        i2s_set_step_outputs(stepper->step_outbits);
        while (esp_timer_get_time() - step_pulse_start_time < i2s_step_length) {
            __asm__ __volatile__ ("nop");  // spin here until time to turn off step
        }
        i2s_set_step_outputs((axes_signals_t){0});
#else
        set_step_outputs(stepper->step_outbits);
#endif
    }
}

#ifdef USE_I2S_OUT

static void i2s_set_streaming_mode (bool stream)
{
    if(!stream && hal.stepper.wake_up == I2S_stepperWakeUp) {
        i2s_out_set_passthrough();
        i2s_out_delay();
    }

    if(stream) {
        hal.stepper.wake_up = I2S_stepperWakeUp;
        hal.stepper.go_idle = I2S_stepperGoIdle;
        hal.stepper.cycles_per_tick = I2S_stepperCyclesPerTick;
        hal.stepper.pulse_start = I2S_stepperPulseStart;
    } else {
        hal.stepper.wake_up = stepperWakeUp;
        hal.stepper.go_idle = stepperGoIdle;
        hal.stepper.cycles_per_tick = stepperCyclesPerTick;
        hal.stepper.pulse_start = stepperPulseStart;
    }
}

#endif

// Enable/disable limit pins interrupt
static void limitsEnable (bool on, bool homing)
{
#ifdef USE_I2S_OUT
    i2s_set_streaming_mode(!homing);
#endif

    uint32_t i = sizeof(inputpin) / sizeof(state_signal_t);
    do {
        if(inputpin[--i].group == INPUT_GROUP_LIMIT)
            gpio_set_intr_type(inputpin[i].pin, on ? (inputpin[i].invert ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE) : GPIO_INTR_DISABLE);
    } while(i);

#if TRINAMIC_ENABLE
    trinamic_homing(homing);
#endif
}

// Returns limit state as an axes_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
inline IRAM_ATTR static limit_signals_t limitsGetState()
{
    limit_signals_t signals = {0};

    signals.min.x = gpio_get_level(X_LIMIT_PIN);
    signals.min.y = gpio_get_level(Y_LIMIT_PIN);
    signals.min.z = gpio_get_level(Z_LIMIT_PIN);
#ifdef A_LIMIT_PIN
    signals.min.a = gpio_get_level(A_LIMIT_PIN);
#endif
#ifdef B_LIMIT_PIN
    signals.min.b = gpio_get_level(B_LIMIT_PIN);
#endif
#ifdef C_LIMIT_PIN
    signals.min.c = gpio_get_level(C_LIMIT_PIN);
#endif

    if (settings.limits.invert.value)
        signals.min.value ^= settings.limits.invert.value;

    return signals;
}

// Returns system state as a control_signals_t variable.
// Each bitfield bit indicates a control signal, where triggered is 1 and not triggered is 0.
inline IRAM_ATTR static control_signals_t systemGetState (void)
{
    control_signals_t signals;

    signals.value = settings.control_invert.value;

#ifdef RESET_PIN
    signals.reset = gpio_get_level(RESET_PIN);
#endif
#ifdef FEED_HOLD_PIN
    signals.feed_hold = gpio_get_level(FEED_HOLD_PIN);
#endif
#ifdef CYCLE_START_PIN
    signals.cycle_start = gpio_get_level(CYCLE_START_PIN);
#endif
#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
    signals.safety_door_ajar = gpio_get_level(SAFETY_DOOR_PIN);
#endif

    if(settings.control_invert.value)
        signals.value ^= settings.control_invert.value;

    return signals;
}

#ifdef PROBE_PIN

// Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
static void probeConfigure(bool is_probe_away, bool probing)
{
#ifdef USE_I2S_OUT
    i2s_set_streaming_mode(!probing);
#endif

    probe.triggered = Off;
    probe.is_probing = probing;
    probe.inverted = is_probe_away ? !settings.probe.invert_probe_pin : settings.probe.invert_probe_pin;

#if PROBE_ISR
    gpio_set_intr_type(inputpin[INPUT_PROBE].pin, probe_invert ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE);
    inputpin[INPUT_PROBE].active = false;
#endif
}

// Returns the probe connected and triggered pin states.
probe_state_t probeGetState (void)
{
    probe_state_t state = {0};

    state.connected = probe.connected;

#if PROBE_ISR
    // TODO: verify!
    inputpin[INPUT_PROBE].active = inputpin[INPUT_PROBE].active || ((uint8_t)gpio_get_level(PROBE_PIN) ^ probe.inverted);
    state.triggered = inputpin[INPUT_PROBE].active;
#else
    state.triggered = (uint8_t)gpio_get_level(PROBE_PIN) ^ probe.inverted;
#endif

    return state;
}

#endif

#ifndef VFD_SPINDLE

// Static spindle (off, on cw & on ccw)
IRAM_ATTR inline void spindle_off (void)
{
#if IOEXPAND_ENABLE
    iopins.spindle_on = settings.spindle.invert.on ? On : Off;
    ioexpand_out(iopins);
#else
    gpio_set_level(SPINDLE_ENABLE_PIN, settings.spindle.invert.on ? 1 : 0);
#endif
}

IRAM_ATTR inline static void spindle_on (void)
{
#if IOEXPAND_ENABLE
    iopins.spindle_on = settings.spindle.invert.on ? Off : On;
    ioexpand_out(iopins);
#else
    gpio_set_level(SPINDLE_ENABLE_PIN, settings.spindle.invert.on ? 0 : 1);
#endif
}

IRAM_ATTR inline static void spindle_dir (bool ccw)
{
    if(hal.driver_cap.spindle_dir) {
#if IOEXPAND_ENABLE
        iopins.spindle_dir = (ccw ^ settings.spindle.invert.ccw) ? On : Off;
        ioexpand_out(iopins);
#elif defined(SPINDLE_DIRECTION_PIN)
        gpio_set_level(SPINDLE_DIRECTION_PIN, (ccw ^ settings.spindle.invert.ccw) ? 1 : 0);
#endif
    }
}

// Start or stop spindle
IRAM_ATTR static void spindleSetState (spindle_state_t state, float rpm)
{
    if (!state.on)
        spindle_off();
    else {
        spindle_dir(state.ccw);
        spindle_on();
    }
}

uint8_t esp32s2_read_output_pin(uint32_t num)
{
	if(num < 32)
	{
		return GPIO.out & (1 << num) ? 1 : 0;
	}
	else
	{
		return (GPIO.out1.data & (1 << (num - 32))) ? 1 : 0;
	}
	return 0;
}

uint8_t is_SpindleEnable(void)
{

	return esp32s2_read_output_pin(SPINDLE_ENABLE_PIN) ? 0:1 ;
}
/*激光是否打开*/
uint8_t is_SpindleOpen(void)
{
	uint32_t duty = 0;
	duty = ledc_get_duty(ledConfig.speed_mode, ledConfig.channel);
	duty = settings.spindle.invert.pwm ? pwm_max_value - duty : duty;
	return duty && is_SpindleEnable();
}
/*获取激光pwm功率，*/
uint16_t laser_GetPower(void)
{
	uint32_t duty = 0;
	duty = ledc_get_duty(ledConfig.speed_mode, ledConfig.channel);
	duty = settings.spindle.invert.pwm ? pwm_max_value - duty : duty;
	return duty * 1000 / spindle_pwm.period ;
}
// Variable spindle control functions

// Sets spindle speed
IRAM_ATTR void spindle_set_speed (uint_fast16_t pwm_value)
{
    if (pwm_value == spindle_pwm.off_value) {
        if(settings.spindle.flags.pwm_action == SpindleAction_DisableWithZeroSPeed)
            spindle_off();
#if PWM_RAMPED
        pwm_ramp.pwm_target = pwm_value;
        ledc_set_fade_step_and_start(ledConfig.speed_mode, ledConfig.channel, pwm_ramp.pwm_target, 1, 4, LEDC_FADE_NO_WAIT);
#else
        if(spindle_pwm.always_on) {
            ledc_set_duty(ledConfig.speed_mode, ledConfig.channel, spindle_pwm.off_value);
            ledc_update_duty(ledConfig.speed_mode, ledConfig.channel);
        } else
            ledc_stop(ledConfig.speed_mode, ledConfig.channel, settings.spindle.invert.pwm ? 1 : 0);
#endif
        pwmEnabled = false;
     } else {
#if PWM_RAMPED
         pwm_ramp.pwm_target = pwm_value;
         ledc_set_fade_step_and_start(ledConfig.speed_mode, ledConfig.channel, pwm_ramp.pwm_target, 1, 4, LEDC_FADE_NO_WAIT);
#else
         ledc_set_duty(ledConfig.speed_mode, ledConfig.channel, settings.spindle.invert.pwm ? pwm_max_value - pwm_value : pwm_value);
         ledc_update_duty(ledConfig.speed_mode, ledConfig.channel);
#endif
        if(!pwmEnabled) {
            spindle_on();
            pwmEnabled = true;
        }
    }
}

#ifdef SPINDLE_PWM_DIRECT

static uint_fast16_t spindleGetPWM (float rpm)
{
    return spindle_compute_pwm_value(&spindle_pwm, rpm, false);
}

#else // Only enable if (when?) ESP IDF supports FPU access in ISR !!

IRAM_ATTR static void spindleUpdateRPM (float rpm)
{
    spindle_set_speed(spindle_compute_pwm_value(&spindle_pwm, rpm, false));
}

#endif

// Start or stop spindle, variable version

IRAM_ATTR void __attribute__ ((noinline)) _setSpeed (spindle_state_t state, float rpm)
{
    spindle_dir(state.ccw);
    spindle_set_speed(spindle_compute_pwm_value(&spindle_pwm, rpm, false));
}

IRAM_ATTR static void spindleSetStateVariable (spindle_state_t state, float rpm)
{
    if (!state.on || memcmp(&rpm, &FZERO, sizeof(float)) == 0) { // rpm == 0.0f cannot be used, causes intermittent panic on soft reset!
        spindle_set_speed(spindle_pwm.off_value);
        spindle_off();
    } else
        _setSpeed(state, rpm);
}

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (void)
{
    spindle_state_t state = {0};

#if IOEXPAND_ENABLE // TODO: read from expander?
    state.on = iopins.spindle_on;
    state.ccw = hal.driver_cap.spindle_dir && iopins.spindle_dir;
#else
    state.on = gpio_get_level(SPINDLE_ENABLE_PIN) != 0;
  #if defined(SPINDLE_DIRECTION_PIN)
    state.ccw = hal.driver_cap.spindle_dir && gpio_get_level(SPINDLE_DIRECTION_PIN) != 0;
  #endif
#endif
    state.value ^= settings.spindle.invert.mask;
    state.on |= pwmEnabled;

#if PWM_RAMPED
    state.at_speed = ledc_get_duty(ledConfig.speed_mode, ledConfig.channel) == pwm_ramp.pwm_target;
#endif
    return state;
}

// end spindle code

#endif

// Start/stop coolant (and mist if enabled)
IRAM_ATTR static void coolantSetState (coolant_state_t mode)
{
    mode.value ^= settings.coolant_invert.mask;
#if IOEXPAND_ENABLE
    iopins.flood_on = mode.flood;
    iopins.mist_on = mode.mist;
    ioexpand_out(iopins);
#else
  #ifdef COOLANT_FLOOD_PIN
    gpio_set_level(COOLANT_FLOOD_PIN, mode.flood ? 1 : 0);
  #endif
  #ifdef COOLANT_MIST_PIN
    gpio_set_level(COOLANT_MIST_PIN, mode.mist ? 1 : 0);
  #endif
#endif
}

// Returns coolant state in a coolant_state_t variable
static coolant_state_t coolantGetState (void)
{
    coolant_state_t state = {0};

#if IOEXPAND_ENABLE // TODO: read from expander?
    state.flood = iopins.flood_on;
    state.mist = iopins.mist_on;
#else
  #ifdef COOLANT_FLOOD_PIN
    state.flood = gpio_get_level(COOLANT_FLOOD_PIN);
  #endif
  #ifdef COOLANT_MIST_PIN
    state.mist  = gpio_get_level(COOLANT_MIST_PIN);
  #endif
#endif

    state.value ^= settings.coolant_invert.mask;

    return state;
}

// Helper functions for setting/clearing/inverting individual bits atomically (uninterruptable)
IRAM_ATTR static void bitsSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    portENTER_CRITICAL(&mux);
    *ptr |= bits;
    portEXIT_CRITICAL(&mux);
}

IRAM_ATTR static uint_fast16_t bitsClearAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    portENTER_CRITICAL(&mux);
    uint_fast16_t prev = *ptr;
    *ptr &= ~bits;
    portEXIT_CRITICAL(&mux);
    return prev;
}

IRAM_ATTR static uint_fast16_t valueSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t value)
{
    portENTER_CRITICAL(&mux);
    uint_fast16_t prev = *ptr;
    *ptr = value;
    portEXIT_CRITICAL(&mux);
    return prev;
}

static void enable_irq (void)
{
    portEXIT_CRITICAL(&mux);
}

static void disable_irq (void)
{
    portENTER_CRITICAL(&mux);
}

#if MPG_MODE_ENABLE

IRAM_ATTR static void modeSelect (bool mpg_mode)
{
    // Deny entering MPG mode if busy
    if(mpg_mode == sys.mpg_mode || (mpg_mode && (gc_state.file_run || !(sys.state == STATE_IDLE || (sys.state & (STATE_ALARM|STATE_ESTOP)))))) {
        hal.stream.enqueue_realtime_command(CMD_STATUS_REPORT_ALL);
        return;
    }

    serialSelect(mpg_mode);

    if(mpg_mode) {
        memcpy(&prev_stream, &hal.stream, sizeof(io_stream_t));
        hal.stream.type = StreamType_MPG;
        hal.stream.read = serial2Read;
        hal.stream.write = serial_stream.write;
        hal.stream.get_rx_buffer_available = serial2RXFree;
        hal.stream.reset_read_buffer = serial2Flush;
        hal.stream.cancel_read_buffer = serial2Cancel;
        hal.stream.suspend_read = serial2SuspendInput;
    } else if(hal.stream.read != NULL)
        memcpy(&hal.stream, &prev_stream, sizeof(io_stream_t));

    hal.stream.reset_read_buffer();

    sys.mpg_mode = mpg_mode;
    sys.report.mpg_mode = On;

    // Force a realtime status report, all reports when MPG mode active
    hal.stream.enqueue_realtime_command(mpg_mode ? CMD_STATUS_REPORT_ALL : CMD_STATUS_REPORT);
}

IRAM_ATTR static void modeChange(void)
{
    modeSelect(!gpio_get_level(MPG_ENABLE_PIN));
}

IRAM_ATTR static void modeEnable (void)
{
    if(sys.mpg_mode == gpio_get_level(MPG_ENABLE_PIN))
        modeSelect(true);
}

#endif

void debounceTimerCallback (TimerHandle_t xTimer)
{
      uint8_t grp = 0;

      uint32_t i = sizeof(inputpin) / sizeof(state_signal_t);
      do {
          i--;
          if(inputpin[i].debounce && inputpin[i].active) {
              inputpin[i].active = false; //gpio_get_level(inputpin[i].pin) == (inputpin[i].invert ? 0 : 1);
              grp |= inputpin[i].group;
          }
      } while(i);

//    printf("Debounce %d %d\n", grp, limitsGetState().value);

      if(grp & INPUT_GROUP_LIMIT)
            hal.limits.interrupt_callback(limitsGetState());

      if(grp & INPUT_GROUP_CONTROL)
          hal.control.interrupt_callback(systemGetState());
}


// Configures perhipherals when settings are initialized or changed
static void settings_changed (settings_t *settings)
{

#ifndef VFD_SPINDLE

    if((hal.driver_cap.variable_spindle = settings->spindle.rpm_max > settings->spindle.rpm_min)) {

        if(ledTimerConfig.freq_hz != (uint32_t)settings->spindle.pwm_freq) {
            ledTimerConfig.freq_hz = (uint32_t)settings->spindle.pwm_freq;
            if(ledTimerConfig.freq_hz <= 100) {
                if(ledTimerConfig.duty_resolution != LEDC_TIMER_14_BIT) {
                    ledTimerConfig.duty_resolution = LEDC_TIMER_14_BIT;
                    ledc_timer_config(&ledTimerConfig);
                }
            } else if(ledTimerConfig.duty_resolution != LEDC_TIMER_10_BIT) {
                ledTimerConfig.duty_resolution = LEDC_TIMER_10_BIT;
                ledc_timer_config(&ledTimerConfig);
            }
        }

        pwm_max_value = (1UL << ledTimerConfig.duty_resolution) - 1;

        spindle_pwm.period = (uint32_t)(80000000UL / settings->spindle.pwm_freq);
        if(settings->spindle.pwm_off_value == 0.0f)
            spindle_pwm.off_value = settings->spindle.invert.pwm ? pwm_max_value : 0;
        else {
            spindle_pwm.off_value = (uint32_t)(pwm_max_value * settings->spindle.pwm_off_value / 100.0f);
            if(settings->spindle.invert.pwm)
                spindle_pwm.off_value = pwm_max_value - spindle_pwm.off_value;
        }
        spindle_pwm.min_value = (uint32_t)(pwm_max_value * settings->spindle.pwm_min_value / 100.0f);
        spindle_pwm.max_value = (uint32_t)(pwm_max_value * settings->spindle.pwm_max_value / 100.0f) + (settings->spindle.invert.pwm ? -1 : 1);
        spindle_pwm.pwm_gradient = (float)(spindle_pwm.max_value - spindle_pwm.min_value) / (settings->spindle.rpm_max - settings->spindle.rpm_min);
        spindle_pwm.always_on = settings->spindle.pwm_off_value != 0.0f;

        ledc_set_freq(ledTimerConfig.speed_mode, ledTimerConfig.timer_num, ledTimerConfig.freq_hz);
    }

#endif

    if(IOInitDone) {

      #ifndef VFD_SPINDLE
        hal.spindle.set_state = hal.driver_cap.variable_spindle ? spindleSetStateVariable : spindleSetState;
      #endif

#if WIFI_ENABLE

        static bool wifi_ok = false;

        if(!wifi_ok)
            wifi_ok = wifi_start();

        // TODO: start/stop services...
#endif

#if BLUETOOTH_ENABLE
        static bool bluetooth_ok = false;
        if(!bluetooth_ok)
            bluetooth_ok = bluetooth_start();
        // else report error?
#endif

        stepperEnable(settings->steppers.deenergize);

        /*********************
         * Step pulse config *
         *********************/

#ifdef USE_I2S_OUT
        i2s_step_length = (uint32_t)(settings->steppers.pulse_microseconds);
        if(i2s_step_length < I2S_OUT_USEC_PER_PULSE)
            i2s_step_length = I2S_OUT_USEC_PER_PULSE;
        i2s_step_samples = i2s_step_length / I2S_OUT_USEC_PER_PULSE; // round up?
#else
        initRMT(settings);
#endif

        /****************************************
         *  Control, limit & probe pins config  *
         ****************************************/

        bool pullup = true;
        control_signals_t control_fei;
        gpio_config_t config;

        control_fei.mask = settings->control_disable_pullup.mask ^ settings->control_invert.mask;

        axes_signals_t limit_fei;
        limit_fei.mask = settings->limits.disable_pullup.mask ^ settings->limits.invert.mask;

        uint32_t i = sizeof(inputpin) / sizeof(state_signal_t);

        do {

            config.intr_type = GPIO_INTR_DISABLE;

            switch(inputpin[--i].id) {

                case Input_Reset:
                    pullup = !settings->control_disable_pullup.reset;
                    inputpin[i].invert = control_fei.reset;
                    config.intr_type = inputpin[i].invert ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE;
                    break;

                case Input_FeedHold:
                    pullup = !settings->control_disable_pullup.feed_hold;
                    inputpin[i].invert = control_fei.feed_hold;
                    config.intr_type = inputpin[i].invert ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE;
                    break;

                case Input_CycleStart:
                    pullup = !settings->control_disable_pullup.cycle_start;
                    inputpin[i].invert = control_fei.cycle_start;
                    config.intr_type = inputpin[i].invert ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE;
                    break;

                case Input_SafetyDoor:
                    pullup = !settings->control_disable_pullup.safety_door_ajar;
                    inputpin[i].invert = control_fei.safety_door_ajar;
                    config.intr_type = inputpin[i].invert ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE;
                    break;
#ifdef PROBE_PIN
                case Input_Probe:
                    pullup = hal.driver_cap.probe_pull_up;
                    inputpin[i].invert = false;
                    break;
#endif
                case Input_LimitX:
                    pullup = !settings->limits.disable_pullup.x;
                    inputpin[i].invert = limit_fei.x;
                    break;

                case Input_LimitY:
                    pullup = !settings->limits.disable_pullup.y;
                    inputpin[i].invert = limit_fei.y;
                    break;

                case Input_LimitZ:
                    pullup = !settings->limits.disable_pullup.z;
                    inputpin[i].invert = limit_fei.z;
                    break;
#ifdef A_LIMIT_PIN
                case Input_LimitA:
                    pullup = !settings->limits.disable_pullup.a;
                    inputpin[i].invert = limit_fei.a;
                    break;
#endif
#ifdef B_LIMIT_PIN
                case Input_LimitB:
                    pullup = !settings->limits.disable_pullup.b;
                    inputpin[i].invert = limit_fei.b;
                    break;
#endif
#ifdef C_LIMIT_PIN
                case Input_LimitC:
                    pullup = !settings->limits.disable_pullup.c;
                    inputpin[i].invert = limit_fei.c;
                    break;
#endif
#if MPG_MODE_ENABLE
                case Input_ModeSelect:
                    pullup = true;
                    inputpin[i].invert = false;
                    config.intr_type = GPIO_INTR_ANYEDGE;
                    break;
#endif
#if KEYPAD_ENABLE
                case Input_KeypadStrobe:
                    pullup = true;
                    inputpin[i].invert = false;
                    config.intr_type = GPIO_INTR_ANYEDGE;
                    break;
#endif
                default:
                    break;

            }

            if(inputpin[i].pin != 0xFF) {

                gpio_intr_disable(inputpin[i].pin);

                config.pin_bit_mask = 1ULL << inputpin[i].pin;
                config.mode = GPIO_MODE_INPUT;
                config.pull_up_en = pullup && inputpin[i].pin < 34 ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
                config.pull_down_en = pullup || inputpin[i].pin >= 34 ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE;

                inputpin[i].offset = config.pin_bit_mask > (1ULL << 31) ? 1 : 0;
                inputpin[i].mask = inputpin[i].offset == 0 ? (uint32_t)config.pin_bit_mask : (uint32_t)(config.pin_bit_mask >> 32);

    //            printf("IN %d - %d - %d : %x\n", inputpin[i].pin,  inputpin[i].offset, inputpin[i].mask, inputpin[i].invert);

                gpio_config(&config);

                inputpin[i].active   = gpio_get_level(inputpin[i].pin) == (inputpin[i].invert ? 0 : 1);
                inputpin[i].debounce = hal.driver_cap.software_debounce && !(inputpin[i].group == INPUT_GROUP_PROBE || inputpin[i].group == INPUT_GROUP_KEYPAD || inputpin[i].group == INPUT_GROUP_MPG);
            }
            //test_function();
        } while(i);

#if MPG_MODE_ENABLE
        if(hal.driver_cap.mpg_mode)
            // Delay mode enable a bit so grbl can finish startup and MPG controller can check ready status
            hal.delay_ms(50, modeEnable);
#endif
    }
}

#if WIFI_ENABLE
static void reportConnection (bool newopt)
{
    if(!newopt && (services.telnet || services.websocket)) {
        hal.stream.write("[NETCON:");
        hal.stream.write(services.telnet ? "Telnet" : "Websocket");
        hal.stream.write("]" ASCII_EOL);
    }
}
#endif

void light_Init(void)
{
	 gpio_config_t gpioConfig = {
	        .pin_bit_mask = ((uint64_t)1 << LIGHT_PIN),
	        .mode = GPIO_MODE_OUTPUT,
	        .pull_up_en = GPIO_PULLUP_ENABLE,
	        .pull_down_en = GPIO_PULLDOWN_DISABLE,
	        .intr_type = GPIO_INTR_DISABLE
	    };

	gpio_config(&gpioConfig);
	gpio_set_level(LIGHT_PIN,0);
}

uint8_t light_brightness = 100;
void light_SetState(uint8_t s)
{
	if(s)
	{
		light_brightness = 100;
		gpio_set_level(LIGHT_PIN,1);
	}
	else
	{
		light_brightness = 0;
		gpio_set_level(LIGHT_PIN,0);
	}
}
uint8_t light_GetBrightness(void)
{
	return light_brightness;
}

void beep_Init(void)
{
	 gpio_config_t gpioConfig = {
	        .pin_bit_mask = ((uint64_t)1 << BEEP_PIN),
	        .mode = GPIO_MODE_OUTPUT,
	        .pull_up_en = GPIO_PULLUP_ENABLE,
	        .pull_down_en = GPIO_PULLDOWN_DISABLE,
	        .intr_type = GPIO_INTR_DISABLE
	    };

	gpio_config(&gpioConfig);
	gpio_set_level(BEEP_PIN,0);
}

void beep_PwmSet(uint8_t duty)
{
	if(duty)
	{
		gpio_set_level(BEEP_PIN,1);
	}
	else
	{
		gpio_set_level(BEEP_PIN,0);
	}
}


static void check_efuse(void)
{
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }

}
static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
static esp_adc_cal_characteristics_t *adc_chars;
void fire_CheckInit(void)
{
	//check_efuse();
    //Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_13);
    adc1_config_channel_atten(ADC_CHANNEL_1, ADC_ATTEN_DB_11);
    //Characterize ADC
    //adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    //esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_13, 1100, adc_chars);
    //print_char_val_type(val_type);

}

static uint8_t pre_state = 0;
static uint8_t fire_alarm_state = 0;
static uint8_t fire_triggle_when_cycle = 0;
static uint8_t fire_temp_enable_flag = 1;
#define FIRE_CHECK_DEBOUNCE_TIME 10	//MS
//#define FIRE_CHECK_MAX_CNT 	10	//次
#define FIRE_CHECK_MAX_TIME 1000 //MS

void fire_CheckTempEnable(void)
{
	fire_temp_enable_flag = 1;
}

void fire_CheckTempDisable(void)
{
	fire_temp_enable_flag = 0;
}

/*是否获取到环境ad值，在未获取到该值前不能使用火焰检测*/
uint8_t fire_evn_flag = 0;
uint16_t fire_evn_value = 0;
uint16_t sensor_data_head = 0;
uint16_t sensor_data_tail = 0;

#define EVN_CAL_VALUE_TIME  	3000
#define EVN_GET_INTERVAL 		50	 //每50ms采集一次数据
#define EVN_MAX_RATE_OF_CHANGE 	20   //最大认为是环境波动的变化值
#define EVN_MAX_VALUE 			4000 //当低于3000时不能进行火焰检测
#define EVN_MAX_DATA_NUM 		(1000*5/EVN_GET_INTERVAL) //10S每50ms采集一次数据
uint16_t sensor_data[EVN_MAX_DATA_NUM] = {0};

/*中位值平均滤波+滑动滤波 算法获取环境值*/
void fire_UpdateEnvironmentValue1(void)
{
	/*获取环境值时的突变值限制*/
#define FIRE_MUTATION_VALUE_LIMIT 500

	uint8_t str[50] = {0};
	uint32_t i  = 0 ;
	uint32_t next = sensor_data_tail +1;
	uint32_t sum_value = 0;
	uint32_t avg_value = 0;
	uint32_t max_value = 0;
	uint32_t min_value = 0xffff;
	uint32_t err_cnt = 0;
	static uint32_t cal_value_timer = 0 ;
	/*每3s计算一次环境值*/
	if((HAL_GetTick() - cal_value_timer) > EVN_CAL_VALUE_TIME)
	{
		cal_value_timer = HAL_GetTick();

		if(next >= EVN_MAX_DATA_NUM) next = 0;
		/*数据满了开始计算*/
		if(sensor_data_head != next) return;

		for(i = 0; i< EVN_MAX_DATA_NUM; i++)
		{
			/*获取最大值*/
			if(max_value < sensor_data[i]) max_value = sensor_data[i];
			/*获取最小值*/
			if(min_value > sensor_data[i]) min_value = sensor_data[i];
			sum_value = sum_value + sensor_data[i];
		}
		/*去掉最大值和最小值*/
		sum_value = sum_value - max_value - min_value;
		/*求总体平均值*/
		avg_value = sum_value / (EVN_MAX_DATA_NUM -2);

//		/*剔除掉相对于平均值的误差值--限幅*/
//		for(i = 0; i< EVN_MAX_DATA_NUM; i++)
//		{
//			if(abs((int)sensor_data[i] - (int)avg_value) > FIRE_MUTATION_VALUE_LIMIT)
//			{
//				err_cnt ++;
//				sum_value = sum_value - sensor_data[i];
//			}
//		}
//		/*求最终平均值*/
//		avg_value = sum_value / (EVN_MAX_DATA_NUM - err_cnt);


		fire_evn_flag = 1;
		fire_evn_value = avg_value;
	}
}

/*获取环境ad value 连续10秒内的平滑曲线即斜率小于一个固定值*/
void fire_UpdateEnvironmentValue(void)
{
	uint8_t str[50] = {0};
	uint32_t i  = 0 ;
	uint32_t next = sensor_data_tail +1;
	static uint32_t cal_value_timer = 0 ;
	/*每3s计算一次环境值*/
	if((HAL_GetTick() - cal_value_timer) > EVN_CAL_VALUE_TIME)
	{
		cal_value_timer = HAL_GetTick();

		if(next >= EVN_MAX_DATA_NUM) next = 0;
		/*数据满了开始计算*/
		if(sensor_data_head != next) return;

		for(i = 0; i< EVN_MAX_DATA_NUM-1; i++)
		{
			if(abs((int)sensor_data[i] - (int)sensor_data[i+1]) > EVN_MAX_RATE_OF_CHANGE)
			{
				/*丢弃前面的值*/
				sensor_data_head = i+1;
				return ;
			}
		}
		fire_evn_flag = 1;
		fire_evn_value = sensor_data[sensor_data_tail];
	}
}
static uint32_t fire_catch_cnt = 0;		/*统计检测到火焰的次数*/
/*在中断tick中断中调用*/
void fire_GetAverageValue(void)
{
	uint8_t str[100] = {0};
	static uint32_t get_value_timer = 0 ;
	uint16_t next = 0;
	if(hal.stream.write_all == NULL)return ;
	/*每50ms获取一个数据*/
	if((HAL_GetTick() - get_value_timer) > EVN_GET_INTERVAL)
	{
		get_value_timer = HAL_GetTick();

		next = sensor_data_tail + 1;
		if(next >= EVN_MAX_DATA_NUM)
			next = 0;
		sensor_data[sensor_data_tail] = adc1_get_raw((adc1_channel_t)ADC_CHANNEL_1);

		if(settings.fire_log_enable)
		{
			sprintf((char*)str,"[MSG:Fire Sensor,ENV:%d,CURRENT:%d,COUNT:%d.]\r\n",fire_evn_value,sensor_data[sensor_data_tail],fire_catch_cnt);
			hal.stream.write_all((char*)str);
		}

		sensor_data_tail = next;
		if(next == sensor_data_head)
		{
			if((sensor_data_head + 1) >= EVN_MAX_DATA_NUM)
			{
				sensor_data_head = 0;
			}
			else
			{
				sensor_data_head++;
			}

		}
	}
	fire_UpdateEnvironmentValue();
}

/*计算相邻两点的斜率*/
float curve_GetSmoothness(uint16_t* data,uint32_t len)
{
	uint32_t i = 0;
	/*两个数据之间的间隔是50ms--10s 200个数据样本*/
	float sensor_data_slope[EVN_MAX_DATA_NUM-1] = {0};
	for(i = 0; i< (EVN_MAX_DATA_NUM - 1); i++)
	{
		sensor_data_slope[i] = ((float)sensor_data[i+1] - (float)sensor_data[i]) / EVN_GET_INTERVAL;
	}

	return sensor_data_slope[0];
}


void fire_InfoReport(void)
{
	char str[100] = {0};
	if(fire_temp_enable_flag == 0)
	{
		hal.stream.write_all("[MSG:Flame detector Inactive temporarily]" ASCII_EOL);
	}
	else
	{
		if(fire_evn_flag == 0)
		{
			hal.stream.write_all("[MSG:Detecting ambient infrared]" ASCII_EOL);
		}
		else
		{
			if(fire_evn_value < EVN_MAX_VALUE)
			{
				hal.stream.write_all("[MSG: Flame detector Inactive. Luminosity too high]" ASCII_EOL);
			}
			else
			{
				sprintf(str,"[MSG: Flame detector active,Ambient infrared value:%d]\r\n",fire_evn_value);
				hal.stream.write_all(str);
			}

		}
	}
}

uint32_t fire_GetEvnValue(void)
{
	return fire_evn_value;
}

uint32_t fire_GetCurrentValue(void)
{
	return adc1_get_raw((adc1_channel_t)ADC_CHANNEL_1);
}


/*
 * 检测1s内的触发次数
 */
void fire_Check(void)
{
	static uint32_t check_timer = 0; //消抖计时
	uint16_t average_value = 0;

	uint32_t limit_min_value = 0;

	control_signals_t signalss = {0};
	/*临时关闭,硬件复位后使能*/
	if(fire_temp_enable_flag == 0)
	{
		return;
	}
	/*火焰检测功能关闭*/
	if(settings.fire_alarm_time_threshold == 0)
	{
		return;
	}
	/*已经处于报警状态，则不再进行火焰检测*/
	if(fire_alarm_state == 1)
	{
		return;
	}
	/*还未得到环境值*/
	if(fire_evn_flag == 0)
	{
		return;
	}
	/*环境值过小*/
	if(fire_evn_value < EVN_MAX_VALUE)
	{
		return ;
	}

	if((HAL_GetTick() - check_timer) > FIRE_CHECK_DEBOUNCE_TIME)
	{
		check_timer = HAL_GetTick();

		/*根据不同的环境光强度下计算触发值*/
		#define K_VALUE 0.2
		#define B_VALUE (-500)
		/**/
		limit_min_value = fire_evn_value * K_VALUE + B_VALUE;
		limit_min_value = 50;

		average_value = adc1_get_raw((adc1_channel_t)ADC_CHANNEL_1);
		if((fire_evn_value > average_value) && (fire_evn_value - average_value > limit_min_value))
		{
			fire_catch_cnt++;
		}
		else
		{
			if(fire_catch_cnt)
				fire_catch_cnt --;
		}
	}

	if(fire_catch_cnt > settings.fire_alarm_time_threshold)
	{
		beep_PwmSet(100);

		/*利用hold功能实现暂停打印功能*/
		if(state_get() == STATE_CYCLE)
		{
			fire_triggle_when_cycle = 1;
			signalss.feed_hold = 1;
			hal.control.interrupt_callback(signalss);
		}

		spindle_off_directly();

		/*触发火焰报警后需重新获取环境值*/
		fire_evn_flag = 0;
//		fan_PwmSet(0);//关风扇
		hal.stream.write_all("[MSG: Flame Alarm! If Safe, press Power Button to Resume]" ASCII_EOL);
//		sys.state = STATE_ALARM;
//		sys.abort = 1;
		fire_alarm_state = 1;
		/*修复这个问题： 紧急停止在关闭状态下再触发火焰后无法关闭蜂鸣器*/
		pre_state = 1;

	}
	else/*环境光线太强*/
	{

	}
}


/*使能或消除火焰报警状态*/
void fire_AlarmStateSet(uint8_t state)
{
	/*每次关报警的时候都需要重置获取环境的标志位,以避免重复触发*/
	if((fire_alarm_state == 1) && (state == 0))
	{
		if(fire_triggle_when_cycle == 1)
		{
			hal.stream.write_all("[MSG:Flame Detection OverRide. Resuming Engrave.]" ASCII_EOL);
			fire_triggle_when_cycle = 0;
		}
		else
		{
			hal.stream.write_all("[MSG:Flame detection alarm cancelled.]" ASCII_EOL);
		}
		fire_evn_flag = 0;
	}
	fire_alarm_state = state;
}

void fire_Alarm(void)
{

	static uint32_t flash_timer = 0;

	if(fire_alarm_state)
	{
		/*声光报警*/
		if((HAL_GetTick()-flash_timer)>500)
		{
			//HAL_GPIO_TogglePin(LIGHT_PWM_PORT, LIGHT_PWM_BIT);
			//light_Toggle();
			flash_timer = HAL_GetTick();
		}
		pre_state = 1;
	}
	else
	{
		/*避免多次关闭报警动作*/
		if(pre_state == 1)
		{
			pre_state = 0;
			/*关闭报警*/
			light_SetState(1);
			beep_PwmSet(0);
		}
	}

}
/*LED*/
void led_Init(void)
{
	 gpio_config_t gpioConfig = {
	        .pin_bit_mask = ((uint64_t)1 << POWER_LED_PIN) | ((uint64_t)1 << COMM_LED_PIN),
	        .mode = GPIO_MODE_OUTPUT,
	        .pull_up_en = GPIO_PULLUP_ENABLE,
	        .pull_down_en = GPIO_PULLDOWN_DISABLE,
	        .intr_type = GPIO_INTR_DISABLE
	    };

	gpio_config(&gpioConfig);
}

void power_LedAlarm(void)
{
	static uint32_t flash_time = 0;
	if((state_get() == STATE_ALARM) && (gpio_get_level(POWER_KEY_PIN) != 0))
	{
		if((HAL_GetTick() - flash_time) > 500)
		{
			flash_time = HAL_GetTick();
			power_LedToggle();
		}
	}
	else if(gpio_get_level(POWER_KEY_PIN) != 0)
	{
		gpio_set_level(POWER_LED_PIN, 1);
	}
}
void power_LedToggle(void)
{
	static uint8_t status = 0;
	status = !status;
	gpio_set_level(POWER_LED_PIN,status);
}
void power_LedOn(void)
{
	gpio_set_level(POWER_LED_PIN,1);
}
void power_LedOff(void)
{
	gpio_set_level(POWER_LED_PIN,0);
}
void comm_LedToggle(void)
{
	static uint8_t status = 0;
	status = !status;
	gpio_set_level(COMM_LED_PIN,status);
}
void comm_LedOn(void)
{
	gpio_set_level(COMM_LED_PIN,1);
}
void comm_LedOff(void)
{
	gpio_set_level(COMM_LED_PIN,0);
}
/*KEY*/
void power_KeyInit(void)
{
	gpio_config_t gpioConfig = {
			.pin_bit_mask = ((uint64_t)1 << POWER_KEY_PIN),
			.mode = GPIO_MODE_INPUT,
			.pull_up_en = GPIO_PULLUP_ENABLE,
			.pull_down_en = GPIO_PULLDOWN_DISABLE,
			.intr_type = GPIO_INTR_DISABLE
		};

	gpio_config(&gpioConfig);
}

uint8_t power_KeyDown(void)
{
	return gpio_get_level(POWER_KEY_PIN);
}

static uint32_t auto_poweroff_time = 0;

/*自动关机功能*/
void system_UpdateAutoPoweroffTime(void)
{
	auto_poweroff_time = HAL_GetTick();
}
void system_AutoPowerOff(void)
{
	if(settings.sys_auto_poweroff_time == 0) return;
	/*usb,uart长时间没有输入数据则自动关机*/
	if((HAL_GetTick() - auto_poweroff_time) > (1000 * 60 * settings.sys_auto_poweroff_time))
	{
		hal.stream.write_all(" [MSG: Power saving Mode enabled. Ortur powering off]"ASCII_EOL);
		HAL_Delay(5);
		esp_restart();
	}
}
/*电源检测*/
uint8_t report_power_flag = 0;//是否报告电源状态
static uint8_t last_power_flag=0;//变化前电源状态

#define IsMainPowrBitSet() (gpio_get_level(POWER_CHECK_PIN) ? 1:0)

void Main_PowerCheckInit(void)
{
	gpio_config_t gpioConfig = {
			.pin_bit_mask = ((uint64_t)1 << POWER_CHECK_PIN),
			.mode = GPIO_MODE_INPUT,
			.pull_up_en = GPIO_PULLUP_ENABLE,
			.pull_down_en = GPIO_PULLDOWN_DISABLE,
			.intr_type = GPIO_INTR_DISABLE
		};

	gpio_config(&gpioConfig);
}


/**
 *掉电去抖
 */
uint8_t IsMainPowrIn(void)
{
	if(IsMainPowrBitSet())
	{
		HAL_Delay(10);
		if(IsMainPowrBitSet())
		{
			last_power_flag=1;
			return 1;
		}
	}
	return 0;
}
/*0:check power 1:report power*/
void Main_PowerCheckReport(uint8_t mode)
{
	/*报告状态*/
	if(mode)
	{
		if(report_power_flag)
		{
		  report_feedback_message(report_power_flag);
		  report_power_flag=0;
		}
	}
	else /*检测状态*/
	{
		if(!IsMainPowrIn())
		 {
			report_power_flag = Message_NoPowerSupply;
			 //report_feedback_message(MESSAGE_MAIN_POWER_OFF);
		 }
	}
}
/**
 * 检测主电源状态
 */
void Main_PowerCheck(void)
{
	if(last_power_flag==0)
	{
		if(IsMainPowrIn())
		{
			report_feedback_message(Message_PowerSupplied);
		}
	}
	else
	{
		/*掉电去抖*/
		if(!IsMainPowrBitSet())
		{
			HAL_Delay(10);
			if(!IsMainPowrBitSet())
			{
				last_power_flag=0;
				report_feedback_message(Message_NoPowerSupply);
			}
		}
	}
}

// Initializes MCU peripherals for Grbl use
static bool driver_setup (settings_t *settings)
{
    /******************
     *  Stepper init  *
     ******************/

    timer_config_t timerConfig = {
        .divider     = STEPPER_DRIVER_PRESCALER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en  = TIMER_PAUSE,
        .alarm_en    = TIMER_ALARM_EN,
        .intr_type   = TIMER_INTR_LEVEL,
        .auto_reload = true
    };

    timer_init(STEP_TIMER_GROUP, STEP_TIMER_INDEX, &timerConfig);
    timer_set_counter_value(STEP_TIMER_GROUP, STEP_TIMER_INDEX, 0ULL);
    timer_isr_register(STEP_TIMER_GROUP, STEP_TIMER_INDEX, stepper_driver_isr, 0, ESP_INTR_FLAG_IRAM, NULL);
    timer_enable_intr(STEP_TIMER_GROUP, STEP_TIMER_INDEX);

    /********************
     *  Output signals  *
     ********************/


    uint32_t idx;
    for(idx = 0; idx < N_AXIS; idx++)
        rmt_set_source_clk(idx, RMT_BASECLK_APB);

    uint64_t mask = 0; // this is insane...
    idx = sizeof(outputpin) / sizeof(gpio_num_t);
    do {
        mask |= ((uint64_t)1ULL << outputpin[--idx]);
    } while(idx);

    gpio_config_t gpioConfig = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&gpioConfig);

#if MPG_MODE_ENABLE
ccc
    /************************
     *  MPG mode (pre)init  *
     ************************/

    // Set as output low (until boot is complete)
    gpioConfig.pin_bit_mask = ((uint64_t)1ULL << MPG_ENABLE_PIN);
    gpio_config(&gpioConfig);
    gpio_set_level(MPG_ENABLE_PIN, 0);

    serial2Init(BAUD_RATE);
#endif

   /****************************
    *  Software debounce init  *
    ****************************/

    if(hal.driver_cap.software_debounce)
        debounceTimer = xTimerCreate("debounce", pdMS_TO_TICKS(32), pdFALSE, NULL, debounceTimerCallback);

    /******************************************
     *  Control, limit & probe pins dir init  *
     ******************************************/

    gpio_isr_register(gpio_isr, NULL, (int)ESP_INTR_FLAG_IRAM, NULL);

#ifndef VFD_SPINDLE

    /******************
    *  Spindle init  *
    ******************/

#if PWM_RAMPED
    ledc_fade_func_install(ESP_INTR_FLAG_IRAM);
#endif
    ledConfig.speed_mode = ledTimerConfig.speed_mode;
    ledc_timer_config(&ledTimerConfig);
    ledc_channel_config(&ledConfig);

    /**/

#endif

#if SDCARD_ENABLE

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5
//        .allocation_unit_size = 16 * 1024
    };

    slot_config.gpio_miso = PIN_NUM_MISO;
    slot_config.gpio_mosi = PIN_NUM_MOSI;
    slot_config.gpio_sck  = PIN_NUM_CLK;
    slot_config.gpio_cs   = PIN_NUM_CS;

    host.max_freq_khz = 20000; //SDMMC_FREQ_DEFAULT; //SDMMC_FREQ_PROBING; 19000;

    sdmmc_card_t* card;
    esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

    sdcard_init();
#endif

#if IOEXPAND_ENABLE
    ioexpand_init();
#endif

#if WEBUI_ENABLE
    webui_init();
#endif

#if ENABLE_POWER_SUPPLY_CHECK
    /*主电源供电检测*/
    Main_PowerCheckInit();
#endif
    /*照明初始化*/
    light_Init();
    /*蜂鸣器初始化*/
    beep_Init();
    /*火焰检测初始化*/
    fire_CheckInit();
#if ENABLE_ACCELERATION_DETECT
    /*加速度传感器初始化*/
    Gsensor_Init();
#endif
  // Set defaults

    IOInitDone = settings->version == 19;

    hal.settings_changed(settings);

    hal.stepper.go_idle(true);

    return IOInitDone;
}
void test_function(void)
{
	//while(1)
	 {
		//printf("hello world.\r\n");
		//vTaskDelay(1000 / portTICK_PERIOD_MS);
	 }
}
// Initialize HAL pointers, setup serial comms and enable EEPROM
// NOTE: Grbl is not yet configured (from EEPROM data), driver_setup() will be called when done
bool driver_init (void)
{
    // Enable EEPROM and serial port here for Grbl to be able to configure itself and report any errors

    serialInit();

    usb_SerialInit();

    hal.info = "ESP32 S2";
    hal.driver_version = "210314";
#ifdef BOARD_NAME
    hal.board = BOARD_NAME;
#endif
    hal.driver_setup = driver_setup;
    hal.f_step_timer = rtc_clk_apb_freq_get() / STEPPER_DRIVER_PRESCALER; // 20 MHz
    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.delay_ms = driver_delay_ms;
    hal.settings_changed = settings_changed;

#ifndef USE_I2S_OUT
    hal.stepper.wake_up = stepperWakeUp;
    hal.stepper.go_idle = stepperGoIdle;
    hal.stepper.enable = stepperEnable;
    hal.stepper.cycles_per_tick = stepperCyclesPerTick;
    hal.stepper.pulse_start = stepperPulseStart;
#else
    hal.stepper.wake_up = I2S_stepperWakeUp;
    hal.stepper.go_idle = I2S_stepperGoIdle;
    hal.stepper.enable = stepperEnable;
    hal.stepper.cycles_per_tick = I2S_stepperCyclesPerTick;
    hal.stepper.pulse_start = I2S_stepperPulseStart;
    i2s_out_init();
    i2s_out_set_pulse_callback(hal.stepper.interrupt_callback);
#endif

    hal.limits.enable = limitsEnable;
    hal.limits.get_state = limitsGetState;

    hal.coolant.set_state = coolantSetState;
    hal.coolant.get_state = coolantGetState;

#ifdef PROBE_PIN
    hal.probe.get_state = probeGetState;
    hal.probe.configure = probeConfigure;
#endif

#ifndef VFD_SPINDLE
    hal.spindle.set_state = spindleSetState;
    hal.spindle.get_state = spindleGetState;
  #ifdef SPINDLE_PWM_DIRECT
    hal.spindle.get_pwm = spindleGetPWM;
    hal.spindle.update_pwm = spindle_set_speed;
  #else
    hal.spindle.update_rpm = spindleUpdateRPM; // NOTE: fails in laser mode as ESP32 does not handle FPU access in ISRs!
  #endif
#endif

    hal.control.get_state = systemGetState;


    selectStream(StreamType_Serial);

#if I2C_ENABLE
    I2CInit();
#endif

#if EEPROM_ENABLE
    i2c_eeprom_init();
#else
    if(nvsInit()) {
        hal.nvs.type = NVS_Flash;
        hal.nvs.memcpy_from_flash = nvsRead;
        hal.nvs.memcpy_to_flash = nvsWrite;
    } else
        hal.nvs.type = NVS_None;
#endif

//    hal.reboot = esp_restart; crashes the MCU...
    hal.irq_enable = enable_irq;
    hal.irq_disable = disable_irq;
    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;
    hal.get_elapsed_ticks = xTaskGetTickCountFromISR;

#ifdef DEBUGOUT
    hal.debug_out = debug_out;
#endif

#if WIFI_ENABLE
    grbl.on_report_options = reportConnection;
#endif

  // driver capabilities, used for announcing and negotiating (with Grbl) driver functionality

#ifndef VFD_SPINDLE
  #if IOEXPAND_ENABLE || defined(SPINDLE_DIRECTION_PIN)
    hal.driver_cap.spindle_dir = On;
  #endif
    hal.driver_cap.variable_spindle = On;
    hal.driver_cap.spindle_pwm_invert = On;
  #if PWM_RAMPED
    hal.driver_cap.spindle_at_speed = On;
  #endif
  #if IOEXPAND_ENABLE || defined(COOLANT_MIST_PIN)
    hal.driver_cap.mist_control = On;
  #endif
#endif
    hal.driver_cap.software_debounce = On;
    hal.driver_cap.step_pulse_delay = On;
    hal.driver_cap.amass_level = 3;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;
    hal.driver_cap.probe_pull_up = On;
#ifdef SAFETY_DOOR_PIN
    hal.signals_cap.safety_door_ajar = On;
#endif
#if MPG_MODE_ENABLE
    hal.driver_cap.mpg_mode = On;
#endif

#if MODBUS_ENABLE

    modbus_stream.write = serial2Write;
    modbus_stream.read = serial2Read;
    modbus_stream.flush_rx_buffer = serial2Flush;
    modbus_stream.flush_tx_buffer = serial2Flush;
    modbus_stream.get_rx_buffer_count = serial2Available;
    modbus_stream.get_tx_buffer_count = serial2txCount;
    modbus_stream.set_baud_rate = serial2SetBaudRate;

    bool modbus = modbus_init(&modbus_stream);

#if SPINDLE_HUANYANG > 0
    if(modbus)
        huanyang_init(&modbus_stream);
#endif

#endif

#if WIFI_ENABLE
    wifi_init();
#endif

#if BLUETOOTH_ENABLE
    bluetooth_init();
#endif

#if TRINAMIC_ENABLE
    trinamic_init();
#endif

#if KEYPAD_ENABLE
    keypad_init();
#endif

//    grbl_esp32_if_init();

    //my_plugin_init();
    //test_function();
    // no need to move version check before init - compiler will fail any mismatch for existing entries
    return hal.version == 8;
}

/* interrupt handlers */

// Main stepper driver
IRAM_ATTR static void stepper_driver_isr (void *arg)
{
    TIMERG0.int_clr.t0 = 1;
    TIMERG0.hw_timer[STEP_TIMER_INDEX].config.alarm_en = TIMER_ALARM_EN;

    hal.stepper.interrupt_callback();
}

  //GPIO intr process
IRAM_ATTR static void gpio_isr (void *arg)
{
  bool debounce = false;
  uint8_t grp = 0;
  uint32_t intr_status[2];
  intr_status[0] = READ_PERI_REG(GPIO_STATUS_REG);          // get interrupt status for GPIO0-31
  intr_status[1] = READ_PERI_REG(GPIO_STATUS1_REG);         // get interrupt status for GPIO32-39
  SET_PERI_REG_MASK(GPIO_STATUS_W1TC_REG, intr_status[0]);  // clear intr for gpio0-gpio31
  SET_PERI_REG_MASK(GPIO_STATUS1_W1TC_REG, intr_status[1]); // clear intr for gpio32-39

  uint32_t i = sizeof(inputpin) / sizeof(state_signal_t);
  do {
      i--;
      if(intr_status[inputpin[i].offset] & inputpin[i].mask) {
          inputpin[i].active = true;
          if(inputpin[i].debounce)
              debounce = true;
          else
              grp |= inputpin[i].group;
      }
  } while(i);

  if(debounce) {
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      xTimerStartFromISR(debounceTimer, &xHigherPriorityTaskWoken);
  }

  if(grp & INPUT_GROUP_LIMIT)
      hal.limits.interrupt_callback(limitsGetState());

  if(grp & INPUT_GROUP_CONTROL)
      hal.control.interrupt_callback(systemGetState());

#if MPG_MODE_ENABLE

  static bool mpg_mutex = false;

  if((grp & INPUT_GROUP_MPG) && !mpg_mutex) {
      mpg_mutex = true;
      modeChange();
     // hal.delay_ms(50, modeChange); // causes intermittent panic... stacked calls due to debounce?
      mpg_mutex = false;
  }
#endif

#if KEYPAD_ENABLE
  if(grp & INPUT_GROUP_KEYPAD)
      keypad_keyclick_handler(gpio_get_level(KEYPAD_STROBE_PIN));
#endif
}


#ifndef _MAX
  #define _MAX(a,b) ((a)>(b)?(a):(b))
#endif

#ifndef _MIN
  #define _MIN(a,b) ((a)<(b)?(a):(b))
#endif

  /* 主轴/激光 是否关闭的标识 */
  uint8_t spindle_disable_by_grbl = 0;
  /* 主轴/激光 被关闭的时间 */
  uint32_t spindle_disabled_time = 0;
  /* 主轴/激光 累积热量*/
  uint32_t spindle_cumulative_heat = 0;
  /* 主轴/激光 是否挂起的标识 */
  uint8_t spindle_suspend_flag = 0;
  /* 主轴/激光 风扇延时时间*/
  uint32_t spindle_fan_delay_time = 0;

/*用于关闭激光*/
void spindle_reset(void)
{
	spindle_set_speed(0);
	spindle_off_directly();
}
void spindle_off_directly(void)
{
	/*关主轴供电*/
	gpio_set_level(SPINDLE_ENABLE_PIN, settings.spindle.invert.on ? 1 : 0);

}


  uint8_t spindle_delay_stop(void)
  {
  	if(spindle_disable_by_grbl)
  	{
  		if(( HAL_GetTick() - spindle_disabled_time ) > spindle_fan_delay_time)
  		{
  			spindle_disable_by_grbl = 0;
  			spindle_cumulative_heat = 0;
  			/*关主轴供电*/
  			gpio_set_level(SPINDLE_ENABLE_PIN, settings.spindle.invert.on ? 1 : 0);
  		}
  		else
  			return 0;
  	}

  	return 1;
  }

  void spindle_disable_by_grbl_set(uint8_t status)
  {
	spindle_disable_by_grbl = status;
	spindle_disabled_time = HAL_GetTick();
	if(status)
	{
		spindle_fan_delay_time = (spindle_cumulative_heat / FAN_HEAT_DISSIPATION_PER_SECOND) * 1000;
		spindle_fan_delay_time = _MAX(spindle_fan_delay_time,MIN_SPINDLE_FAN_TIME);
		spindle_fan_delay_time = _MIN(MAX_SPINDLE_FAN_TIME,spindle_fan_delay_time);
	}
  }

void spindle_calculate_heat()
{
  static uint32_t last_time = 0;
  static uint32_t equivalent_power = 0;
  if(HAL_GetTick() - last_time >= 1000)
  {
	  //电源开启
	  if(is_SpindleEnable())
	  {
		  //计算每秒产生的热量和丧失的热量
		  if(spindle_cumulative_heat < MAX_SPINDLE_HEAT)
			  spindle_cumulative_heat += equivalent_power / 1000;
		  if(spindle_cumulative_heat >= FAN_HEAT_DISSIPATION_PER_SECOND)
			  spindle_cumulative_heat -= FAN_HEAT_DISSIPATION_PER_SECOND ;
	  }
	  else
	  {
		  if(spindle_cumulative_heat >= AIR_HEAT_DISSIPATION_PER_SECOND)
			  spindle_cumulative_heat -= AIR_HEAT_DISSIPATION_PER_SECOND ;
	  }
	  equivalent_power = 0;
	  last_time = HAL_GetTick();
  }
  else
  {
	  equivalent_power += sys.spindle_rpm;
  }
}

/*该函数在系统定时器中调用用于紧急按钮消抖
 *
 * "Emergency switch Engaged"
"Emergency switch Cleared"
 * */

/*记录之前的reset状态*/
static uint32_t pre_reset_flag = 0;

void reset_report(void)
{
	static uint32_t release_reset_timer = 0;


	if((systemGetState().reset == 1) && (pre_reset_flag == 0))
	{
		hal.stream.write_all("[MSG:Emergency switch Engaged!]" ASCII_EOL);
		pre_reset_flag = 1;
		release_reset_timer = HAL_GetTick();
	}

	if(pre_reset_flag == 1)
	{
		if(systemGetState().reset == 0)
		{
			if((HAL_GetTick() - release_reset_timer) > 200)
			{
				hal.stream.write_all("[MSG:Emergency switch Cleared.]" ASCII_EOL);
				pre_reset_flag = 0;
			}
		}
		else
		{
			release_reset_timer = HAL_GetTick();
		}
	}
}

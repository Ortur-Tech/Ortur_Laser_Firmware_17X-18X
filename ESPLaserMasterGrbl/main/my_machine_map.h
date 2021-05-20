/*

*/

#define BOARD_NAME "ESPLaser"

// timer definitions
#define STEP_TIMER_GROUP TIMER_GROUP_0
#define STEP_TIMER_INDEX TIMER_0

#if SDCARD_ENABLE

// Pin mapping when using SPI mode.
// With this mapping, SD card can be used both in SPI and 1-line SD mode.
// Note that a pull-up on CS line is required in SD mode.
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

#endif // SDCARD_ENABLE

// Define step pulse output pins.
#define X_STEP_PIN  GPIO_NUM_33
#define Y_STEP_PIN  GPIO_NUM_36
#define Z_STEP_PIN  GPIO_NUM_2

// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define X_DIRECTION_PIN GPIO_NUM_34
#define Y_DIRECTION_PIN GPIO_NUM_4
#define Z_DIRECTION_PIN GPIO_NUM_1

// Define stepper driver enable/disable output pin(s).
#define STEPPERS_DISABLE_PIN    GPIO_NUM_21

#define ENABLE_JTAG 1

// Define homing/hard limit switch input pins and limit interrupt vectors.
#if ENABLE_JTAG
#define X_LIMIT_PIN GPIO_NUM_35
#else
#define X_LIMIT_PIN GPIO_NUM_42
#endif
#define Y_LIMIT_PIN GPIO_NUM_45
#define Z_LIMIT_PIN GPIO_NUM_46

// Define spindle enable and spindle direction output pins.

#ifndef VFD_SPINDLE
#define SPINDLE_ENABLE_PIN  GPIO_NUM_7
#define SPINDLEPWMPIN       GPIO_NUM_10
#endif

// Define flood and mist coolant enable output pins.

#define COOLANT_FLOOD_PIN   GPIO_NUM_16
#ifndef VFD_SPINDLE
#define COOLANT_MIST_PIN    GPIO_NUM_16
#endif

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#define RESET_PIN       GPIO_NUM_38

#if ENABLE_JTAG
#define FEED_HOLD_PIN   GPIO_NUM_35
#define CYCLE_START_PIN GPIO_NUM_35
#else
#define FEED_HOLD_PIN   GPIO_NUM_40
#define CYCLE_START_PIN GPIO_NUM_41
#endif

#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
#define SAFETY_DOOR_PIN GPIO_NUM_35
#endif

// Define probe switch input pin.
#if PROBE_ENABLE
#define PROBE_PIN    GPIO_NUM_39
#endif

#if MODBUS_ENABLE
#define UART2_RX_PIN            GPIO_NUM_22
#define UART2_TX_PIN            GPIO_NUM_21
#if RS485_DIR_ENABLE
#define MODBUS_DIRECTION_PIN    GPIO_NUM_2
#endif
#endif


#if KEYPAD_ENABLE
#error No free pins for keypad!
#endif

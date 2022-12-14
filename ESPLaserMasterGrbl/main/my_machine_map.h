/*

*/

#ifndef _MY_MACHINE_MAP_H_
#define _MY_MACHINE_MAP_H_



#include "board.h"

#if (BOARD_VERSION == OLM_ESP_V1X)

#define BOARD_NAME "OLM ESP Board"

// timer definitions
#define STEP_TIMER_GROUP TIMER_GROUP_1
#define STEP_TIMER_INDEX TIMER_0
#define STEP_TIMER TIMERG1

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
#define X_STEP_PIN  GPIO_NUM_12
#define Y_STEP_PIN  GPIO_NUM_14
#define Z_STEP_PIN  GPIO_NUM_35
#define STEP_MASK       (1ULL << X_STEP_PIN|1ULL << Y_STEP_PIN|1ULL << Z_STEP_PIN) // All step bits

// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define X_DIRECTION_PIN GPIO_NUM_11
#define Y_DIRECTION_PIN GPIO_NUM_13
#define Z_DIRECTION_PIN GPIO_NUM_34
#define DIRECTION_MASK      (1ULL << X_DIRECTION_PIN|1ULL << Y_DIRECTION_PIN|1ULL << Z_DIRECTION_PIN) // All direction bits


// Define stepper driver enable/disable output pin(s).
#define STEPPERS_DISABLE_PIN    GPIO_NUM_21
#define STEPPERS_DISABLE_MASK   (1ULL << STEPPERS_DISABLE_PIN)


#define ENABLE_JTAG 0

// Define homing/hard limit switch input pins and limit interrupt vectors.

#define X_LIMIT_PIN GPIO_NUM_5
#define Y_LIMIT_PIN GPIO_NUM_4
#define Z_LIMIT_PIN GPIO_NUM_3
#define LIMIT_MASK      (1ULL << X_LIMIT_PIN|1ULL << Y_LIMIT_PIN|1ULL << Z_LIMIT_PIN) // All limit bits


// Define spindle enable and spindle direction output pins.

#ifndef VFD_SPINDLE
#define SPINDLE_ENABLE_PIN  GPIO_NUM_16
#define SPINDLE_MASK        (1ULL << SPINDLE_ENABLE_PIN)
#define SPINDLEPWMPIN       GPIO_NUM_17
#endif

// Define flood and mist coolant enable output pins.

#define COOLANT_FLOOD_PIN   GPIO_NUM_7
#ifndef VFD_SPINDLE
#define COOLANT_MIST_PIN    GPIO_NUM_8
#define COOLANT_MASK        (1ULL << COOLANT_FLOOD_PIN|1ULL << COOLANT_MIST_PIN)
#endif

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#if ENABLE_JTAG
#define RESET_PIN       GPIO_NUM_45
#define FEED_HOLD_PIN   GPIO_NUM_45
#define CYCLE_START_PIN GPIO_NUM_45
#else
#define RESET_PIN       GPIO_NUM_40
#define FEED_HOLD_PIN   GPIO_NUM_0
#define CYCLE_START_PIN GPIO_NUM_41 //GPIO_NUM_39
#endif

#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
#define SAFETY_DOOR_PIN GPIO_NUM_42
#endif
#define CONTROL_MASK        (1ULL << RESET_PIN|1ULL << FEED_HOLD_PIN|1ULL << CYCLE_START_PIN)

// Define probe switch input pin.
#if PROBE_ENABLE
#define PROBE_PIN    GPIO_NUM_6
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

/*????????????*/
#define POWER_KEY_PIN 			GPIO_NUM_41
/*???????????????LED*/
#define POWER_LED_PIN			GPIO_NUM_33
/*USB??????LED*/
#define COMM_LED_PIN			GPIO_NUM_45
/*????????????*/
#define FIRE_CHECK_PIN			GPIO_NUM_2
#define FIRE_ADC_CHANNEL		ADC_CHANNEL_1
/*????????????*/
#define POWER_CHECK_PIN			GPIO_NUM_18
#define POWER_CHECK_CHANNEL		ADC2_CHANNEL_7
/*??????Light*/
#define LIGHT_PIN				GPIO_NUM_1
/*?????????beep*/
#define BEEP_PIN				GPIO_NUM_38
/*IIC SCL*/
#define IIC_SCL_PIN				GPIO_NUM_10
/*IIC SDA*/
#define IIC_SDA_PIN				GPIO_NUM_9

#elif (BOARD_VERSION == OLM_ESP_PRO_V1X)

#define BOARD_NAME "OLM ESP PRO Board"

#define KEY_PREES_DOWM_LEVEL_HIGH 1

// timer definitions
#define STEP_TIMER_GROUP TIMER_GROUP_1
#define STEP_TIMER_INDEX TIMER_0
#define STEP_TIMER TIMERG1

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
#define X_STEP_PIN  GPIO_NUM_21
#define Y_STEP_PIN  GPIO_NUM_2
#define Z_STEP_PIN  GPIO_NUM_1
#define STEP_MASK       (1ULL << X_STEP_PIN|1ULL << Y_STEP_PIN|1ULL << Z_STEP_PIN) // All step bits

// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define X_DIRECTION_PIN GPIO_NUM_7
#define Y_DIRECTION_PIN GPIO_NUM_3
#define Z_DIRECTION_PIN GPIO_NUM_0
#define DIRECTION_MASK      (1ULL << X_DIRECTION_PIN|1ULL << Y_DIRECTION_PIN|1ULL << Z_DIRECTION_PIN) // All direction bits


// Define stepper driver enable/disable output pin(s).
#define STEPPERS_DISABLE_PIN    GPIO_NUM_4
#define STEPPERS_DISABLE_MASK   (1ULL << STEPPERS_DISABLE_PIN)


#define ENABLE_JTAG 0

// Define homing/hard limit switch input pins and limit interrupt vectors.

#define X_LIMIT_PIN GPIO_NUM_8
#define Y_LIMIT_PIN GPIO_NUM_45
#define Z_LIMIT_PIN GPIO_NUM_38
#define LIMIT_MASK      (1ULL << X_LIMIT_PIN|1ULL << Y_LIMIT_PIN|1ULL << Z_LIMIT_PIN) // All limit bits


// Define spindle enable and spindle direction output pins.

#ifndef VFD_SPINDLE
#define SPINDLE_ENABLE_PIN  GPIO_NUM_18
#define SPINDLE_MASK        (1ULL << SPINDLE_ENABLE_PIN)
#define SPINDLEPWMPIN       GPIO_NUM_17
#endif

// Define flood and mist coolant enable output pins.

#define COOLANT_FLOOD_PIN   GPIO_NUM_13
#ifndef VFD_SPINDLE
#define COOLANT_MIST_PIN    GPIO_NUM_13
#define COOLANT_MASK        (1ULL << COOLANT_FLOOD_PIN|1ULL << COOLANT_MIST_PIN)
#endif

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.

#define RESET_PIN       GPIO_NUM_35
#define FEED_HOLD_PIN   GPIO_NUM_34
#define CYCLE_START_PIN GPIO_NUM_46 //GPIO_NUM_33


#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
#define SAFETY_DOOR_PIN GPIO_NUM_42
#endif
#define CONTROL_MASK        (1ULL << RESET_PIN|1ULL << FEED_HOLD_PIN|1ULL << CYCLE_START_PIN)

// Define probe switch input pin.
#if PROBE_ENABLE
#define PROBE_PIN    GPIO_NUM_36
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

/*????????????*/
#define POWER_KEY_PIN 			GPIO_NUM_46
/*???????????????LED*/
#define POWER_LED_PIN			GPIO_NUM_10

#if ENABLE_JTAG
/*USB??????LED*/
#define COMM_LED_PIN			GPIO_NUM_10
#else
/*USB??????LED*/
#define COMM_LED_PIN			GPIO_NUM_41
#endif

/*????????????*/
#define FIRE_CHECK_PIN			GPIO_NUM_9
#define FIRE_ADC_CHANNEL		ADC_CHANNEL_8
/*????????????*/
#define POWER_CHECK_PIN			GPIO_NUM_5
#define POWER_CHECK_CHANNEL		ADC_CHANNEL_4
/*??????Light*/
#define LIGHT_PIN				GPIO_NUM_15
/*?????????beep*/
#define BEEP_PIN				GPIO_NUM_16
/*IIC SCL*/
#define IIC_SCL_PIN				GPIO_NUM_11
/*IIC SDA*/
#define IIC_SDA_PIN				GPIO_NUM_12
/*????????????*/
#define PWR_CTR_PIN				GPIO_NUM_14
/*????????????*/
#define POWER_CURRENT_PIN		GPIO_NUM_6
#define POWER_CURRENT_CHANNEL   ADC_CHANNEL_5

/*x driver uart*/
#define X_DRIVER_UART_PIN		GPIO_NUM_42
#define Y_DRIVER_UART_PIN		GPIO_NUM_39
#define Z_DRIVER_UART_PIN		GPIO_NUM_40
#define UNUSE_PIN				GPIO_NUM_36

#elif (BOARD_VERSION == OCM_ESP_PRO_V1X)

#define BOARD_NAME "OCM ESP PRO Board"

#define KEY_PREES_DOWM_LEVEL_HIGH 1

// timer definitions
#define STEP_TIMER_GROUP TIMER_GROUP_1
#define STEP_TIMER_INDEX TIMER_0
#define STEP_TIMER TIMERG1

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
#define X_STEP_PIN  GPIO_NUM_1
#define Y_STEP_PIN  GPIO_NUM_3
#define Z_STEP_PIN  GPIO_NUM_5
#define STEP_MASK       (1ULL << X_STEP_PIN|1ULL << Y_STEP_PIN|1ULL << Z_STEP_PIN) // All step bits

// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define X_DIRECTION_PIN GPIO_NUM_2
#define Y_DIRECTION_PIN GPIO_NUM_4
#define Z_DIRECTION_PIN GPIO_NUM_6
#define DIRECTION_MASK      (1ULL << X_DIRECTION_PIN|1ULL << Y_DIRECTION_PIN|1ULL << Z_DIRECTION_PIN) // All direction bits


// Define stepper driver enable/disable output pin(s).
#define STEPPERS_DISABLE_PIN    GPIO_NUM_0
#define STEPPERS_DISABLE_MASK   (1ULL << STEPPERS_DISABLE_PIN)


#define ENABLE_JTAG 0

// Define homing/hard limit switch input pins and limit interrupt vectors.

#define X_LIMIT_PIN GPIO_NUM_21
#define Y_LIMIT_PIN GPIO_NUM_45
#define Z_LIMIT_PIN GPIO_NUM_41
#define LIMIT_MASK      (1ULL << X_LIMIT_PIN|1ULL << Y_LIMIT_PIN|1ULL << Z_LIMIT_PIN) // All limit bits


// Define spindle enable and spindle direction output pins.

#ifndef VFD_SPINDLE
#define SPINDLE_ENABLE_PIN  GPIO_NUM_13//GPIO_NUM_34
#define SPINDLE_MASK        (1ULL << SPINDLE_ENABLE_PIN)
#define SPINDLEPWMPIN       GPIO_NUM_33
#endif

// Define flood and mist coolant enable output pins.

#define COOLANT_FLOOD_PIN   GPIO_NUM_38
#ifndef VFD_SPINDLE
#define COOLANT_MIST_PIN    GPIO_NUM_37
#define COOLANT_MASK        (1ULL << COOLANT_FLOOD_PIN|1ULL << COOLANT_MIST_PIN)
#endif

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.

#define RESET_PIN       GPIO_NUM_17
#define FEED_HOLD_PIN   GPIO_NUM_16
#define CYCLE_START_PIN GPIO_NUM_15


#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
#define SAFETY_DOOR_PIN GPIO_NUM_42
#endif
#define CONTROL_MASK        (1ULL << RESET_PIN|1ULL << FEED_HOLD_PIN|1ULL << CYCLE_START_PIN)

// Define probe switch input pin.
#if PROBE_ENABLE
#define PROBE_PIN    GPIO_NUM_18
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

/*????????????*/
#define POWER_KEY_PIN 			GPIO_NUM_46
/*???????????????LED*/
#define POWER_LED_PIN			GPIO_NUM_39

/*USB??????LED*/
#define COMM_LED_PIN			GPIO_NUM_40

/*????????????*/
#define POWER_CHECK_PIN			GPIO_NUM_9
#define POWER_CHECK_CHANNEL		ADC_CHANNEL_8

/*IIC SCL*/
#define IIC_SCL_PIN				GPIO_NUM_12
/*IIC SDA*/
#define IIC_SDA_PIN				GPIO_NUM_11
/*????????????*/
#define PWR_CTR_PIN				GPIO_NUM_35
/*????????????*/
#define POWER_CURRENT_PIN		GPIO_NUM_10
#define POWER_CURRENT_CHANNEL   ADC_CHANNEL_9


#elif

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


#endif


#endif

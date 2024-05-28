// ------------------------------------------------------------------------------------------------------- //
// Pendulum on a cart
// ------------------------------------------------------------------------------------------------------- //

// Version: 1.0
// Author: Renan Duarte
// E-mail: duarte.renan@hotmail.com
// Date:   13/05/2024
// Device: CORTEX M4F TM4C123GH6PM Tiva C Launchpad

// ------------------------------------------------------------------------------------------------------- //
// Predefined Symbols:
// ------------------------------------------------------------------------------------------------------- //

// PART_TM4C123GH6PM - To use pin_map.h
// TARGET_IS_TM4C129_RA0 - To use the include files in ROM of the Tiva device

// ------------------------------------------------------------------------------------------------------- //
// Pin map
// ------------------------------------------------------------------------------------------------------- //

// Encoder theta
//              PD6     A
//              PD7     B

// Encoder X
//              PC5     A
//              PC6     B

// Stepper
//              PB5     Step
//              PB0     Direction
//              PB1     Enable

// Limit Switches
//              PE2     -X
//              PE3     +X

// LCD
//              PA2     SCLK
//              PA3     DC
//              PA4     SCE
//              PA5     DN
//              PA6     BLK

// Buttons
//              PF4     BTN1
//              PF0     BTN2

// UART
//              PA0     RX
//              PA1     TX

// Extras
//              PF1     RGB_R
//              PF2     RGB_B
//              PF3     RGB_G

// ------------------------------------------------------------------------------------------------------- //

#ifndef MAIN_HPP_
#define MAIN_HPP_

#ifdef __cplusplus
extern "C"
{
#endif

// ------------------------------------------------------------------------------------------------------- //
// Includes
// ------------------------------------------------------------------------------------------------------- //

// Standard libraries
#include <stdint.h>
#include <stdbool.h>
#include "stdio.h"
#include "stdlib.h"

// TivaC hardware related libraries
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_timer.h"

// TivaC driver libraries
#include "driverlib/adc.h"
#include "driverlib/aes.h"
#include "driverlib/can.h"
#include "driverlib/comp.h"
#include "driverlib/crc.h"
#include "driverlib/des.h"
#include "driverlib/eeprom.h"
#include "driverlib/emac.h"
#include "driverlib/epi.h"
#include "driverlib/flash.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/hibernate.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "driverlib/lcd.h"
#include "driverlib/mpu.h"
#include "driverlib/onewire.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/qei.h"
#include "driverlib/rom.h"
#include "driverlib/shamd5.h"
#include "driverlib/ssi.h"
#include "driverlib/sw_crc.h"
#include "driverlib/sysctl.h"
#include "driverlib/sysexc.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "driverlib/usb.h"
#include "driverlib/watchdog.h"

// ------------------------------------------------------------------------------------------------------- //
// Application defines
// ------------------------------------------------------------------------------------------------------- //

// Firmware info
#define DEVICE_FW_NAME          "InvPend LQR"
#define DEVICE_FW_VERSION       "1.0"
#define DEVICE_FW_AUTHOR        "Renan Duarte"

// PI
#define PI                      3.1415926535898F

// Application timming
#define TIMER_FREQUENCY         1000
#define CONTROL_LOOP_FREQUENCY  200

// LCD
#define LCD_REFRESH_FREQUENCY   10

// Buttons
#define BUTTON_SCAN_FREQUENCY   1000
#define BUTTON_SCAN_INTERVAL    (1000/BUTTON_SCAN_FREQUENCY)
#define BUTTON_DEAD_TIME        10
#define BUTTON_WINDOW           250
#define BUTTON_LONG_TIMEOUT     1000

// UART
#define UART_REFRESH_FREQUENCY  10
#define UART_BAUD_RATE          115200

// Encoder - Theta
#define ENCODER_T_FREQUENCY     200                     // Encoder scan frequency
#define ENCODER_T_PPR           4000                    // Encoder maximum counter value

// Encoder - X
#define ENCODER_X_FREQUENCY     200                     // Encoder scan frequency
#define ENCODER_X_PPR           83300                   // Encoder maximum counter value

// RGB LED
#define RGB_PWM_FREQ            1000
#define RGB_REFRESH_FREQUENCY   10




// Stepper
#define STEPPER_PPS_MIN         20                                      // Minimum velocity (pulses per second)
#define STEPPER_PPS_MAX         200000                                  // Maximum velocity (pulses per second)
#define STEPPER_ACC_MIN         -10                                     // Minimum acceleration (m/s^2)
#define STEPPER_ACC_MAX         10                                      // Maximum acceleration (m/s^2)
#define STEPPER_PPS_CAL         15000                                   // Calibration velocity (pulses per second)
#define STEPPER_STEPS_REV       200                                     // Steps per revolution
#define STEPPER_MICROSTEPS      32                                      // Pulses per step
#define STEPPER_PPR             (STEPPER_STEPS_REV*STEPPER_MICROSTEPS)  // Pulses per revolution
#define STEPPER_PD              0.013157248F                            // Pulley pitch diameter (meters) - 0.01273 nominal
#define STEPPER_KV_PPS          (STEPPER_PPR/(PI*STEPPER_PD))           // Relation between PPS and m/s

// X-axis
#define X_VALUE_TOTAL_M         0.4055F                 // Maximum travel distance in m
#define X_VALUE_ABS_M           (X_VALUE_TOTAL_M/2)     // Axis limits in m (-X_VALUE_ABS_M to X_VALUE_ABS_M)

// Theta-axis
#define T_VALUE_MAX_RAD         (2*PI)                  // Maximum angle in radians
#define T_VALUE_ABS_MAX         PI                      // Axis limits in radians (-T_VALUE_ABS_MAX to T_VALUE_ABS_MAX)

// ------------------------------------------------------------------------------------------------------- //
// Hardware defines
// ------------------------------------------------------------------------------------------------------- //

// LCD
#define LCD_SSI_PERIPH          SYSCTL_PERIPH_SSI0
#define LCD_SCLK_PERIPH         SYSCTL_PERIPH_GPIOA
#define LCD_DN_PERIPH           SYSCTL_PERIPH_GPIOA
#define LCD_SCE_PERIPH          SYSCTL_PERIPH_GPIOA
#define LCD_DC_PERIPH           SYSCTL_PERIPH_GPIOA
#define LCD_BKL_PERIPH          SYSCTL_PERIPH_GPIOA
#define LCD_SSI_BASE            SSI0_BASE
#define LCD_SCLK_BASE           GPIO_PORTA_BASE
#define LCD_DN_BASE             GPIO_PORTA_BASE
#define LCD_SCE_BASE            GPIO_PORTA_BASE
#define LCD_DC_BASE             GPIO_PORTA_BASE
#define LCD_BKL_BASE            GPIO_PORTA_BASE
#define LCD_SCLK_CFG            GPIO_PA2_SSI0CLK
#define LCD_DN_CFG              GPIO_PA5_SSI0TX
#define LCD_SCLK_PIN            GPIO_PIN_2
#define LCD_DN_PIN              GPIO_PIN_5
#define LCD_SCE_PIN             GPIO_PIN_4
#define LCD_DC_PIN              GPIO_PIN_3
#define LCD_BKL_PIN             GPIO_PIN_6

// Buttons
#define BUTTON_GPIO_PERIPH      SYSCTL_PERIPH_GPIOF
#define BUTTON_GPIO_BASE        GPIO_PORTF_BASE
#define BUTTON_PIN_1            GPIO_PIN_4
#define BUTTON_PIN_2            GPIO_PIN_0

// UART defines
#define UART_UART_PERIPH        SYSCTL_PERIPH_UART0
#define UART_GPIO_PERIPH        SYSCTL_PERIPH_GPIOA
#define UART_UART_BASE          UART0_BASE
#define UART_GPIO_BASE          GPIO_PORTA_BASE
#define UART_RX_CFG             GPIO_PA0_U0RX
#define UART_TX_CFG             GPIO_PA1_U0TX
#define UART_RX_PIN             GPIO_PIN_0
#define UART_TX_PIN             GPIO_PIN_1
#define UART_INTERRUPT          INT_UART0
#define UART_CFG                UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE | UART_CONFIG_WLEN_8

// Encoder defines - Theta
#define ENCODER_T_QEI_PERIPH    SYSCTL_PERIPH_QEI0
#define ENCODER_T_GPIO_PERIPH   SYSCTL_PERIPH_GPIOD
#define ENCODER_T_QEI_BASE      QEI0_BASE
#define ENCODER_T_GPIO_BASE     GPIO_PORTD_BASE
#define ENCODER_T_A_CFG         GPIO_PD6_PHA0
#define ENCODER_T_B_CFG         GPIO_PD7_PHB0
#define ENCODER_T_A_PIN         GPIO_PIN_6
#define ENCODER_T_B_PIN         GPIO_PIN_7
#define ENCODER_T_CFG           QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_QUADRATURE | QEI_CONFIG_SWAP
#define ENCODER_T_INTERRUPT     QEI_INTTIMER

// Encoder defines - X
#define ENCODER_X_QEI_PERIPH    SYSCTL_PERIPH_QEI1
#define ENCODER_X_GPIO_PERIPH   SYSCTL_PERIPH_GPIOC
#define ENCODER_X_QEI_BASE      QEI1_BASE
#define ENCODER_X_GPIO_BASE     GPIO_PORTC_BASE
#define ENCODER_X_A_CFG         GPIO_PC5_PHA1
#define ENCODER_X_B_CFG         GPIO_PC6_PHB1
#define ENCODER_X_A_PIN         GPIO_PIN_5
#define ENCODER_X_B_PIN         GPIO_PIN_6
#define ENCODER_X_CFG           QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_QUADRATURE | QEI_CONFIG_SWAP
#define ENCODER_X_INTERRUPT     QEI_INTTIMER

// RGB LED
#define RGB_PWM_PERIPH          SYSCTL_PERIPH_PWM1;
#define RGB_GPIO_PERIPH         SYSCTL_PERIPH_GPIOF;
#define RGB_PWM_BASE            PWM1_BASE;
#define RGB_GPIO_BASE           GPIO_PORTF_BASE;
#define RGB_PWM_R_GEN           PWM_GEN_2
#define RGB_PWM_G_GEN           PWM_GEN_3
#define RGB_PWM_B_GEN           PWM_GEN_3
#define RGB_PWM_R_OUT           PWM_OUT_5
#define RGB_PWM_G_OUT           PWM_OUT_7
#define RGB_PWM_B_OUT           PWM_OUT_6
#define RGB_PWM_R_OUT_BIT       PWM_OUT_5_BIT
#define RGB_PWM_G_OUT_BIT       PWM_OUT_7_BIT
#define RGB_PWM_B_OUT_BIT       PWM_OUT_6_BIT
#define RGB_PIN_R_CFG           GPIO_PF1_M1PWM5
#define RGB_PIN_G_CFG           GPIO_PF3_M1PWM7
#define RGB_PIN_B_CFG           GPIO_PF2_M1PWM6
#define RGB_R_PIN               GPIO_PIN_1
#define RGB_G_PIN               GPIO_PIN_3
#define RGB_B_PIN               GPIO_PIN_2
#define RGB_PWM_INT             INT_PWM1_2
#define RGB_PWM_INT_GEN         PWM_INT_GEN_2
#define RGB_PWM_CFG             PWM_GEN_MODE_DOWN | PWM_GEN_MODE_DBG_RUN











// Stepper
#define STEPPER_PWM_PERIPH      SYSCTL_PERIPH_PWM0
#define STEPPER_PWM_BASE        PWM0_BASE
#define STEPPER_PWM_GEN         PWM_GEN_1
#define STEPPER_PWM_OUT         PWM_OUT_3
#define STEPPER_PWM_OUT_BIT     PWM_OUT_3_BIT
#define STEPPER_INT_PWM         PWM_INT_GEN_1
#define STEPPER_INT             INT_PWM0_1
#define STEPPER_GPIO_PERIPH     SYSCTL_PERIPH_GPIOB
#define STEPPER_GPIO_BASE       GPIO_PORTB_BASE
#define STEPPER_GPIO_CFG        GPIO_PB5_M0PWM3
#define STEPPER_PIN_STEP        GPIO_PIN_5
#define STEPPER_PIN_DIR         GPIO_PIN_0
#define STEPPER_PIN_EN          GPIO_PIN_1
#define STEPPER_PWM_DIV_CFG     SYSCTL_PWMDIV_64
#define STEPPER_PWM_DIV         64

// Limit switches
#define SWITCH_GPIO_PERIPH      SYSCTL_PERIPH_GPIOE
#define SWITCH_GPIO_BASE        GPIO_PORTE_BASE
#define SWITCH_PIN_START        GPIO_PIN_2
#define SWITCH_PIN_END          GPIO_PIN_3
#define SWITCH_INT_GPIO_START   GPIO_INT_PIN_2
#define SWITCH_INT_GPIO_END     GPIO_INT_PIN_3
#define SWITCH_INT              INT_GPIOE

// ------------------------------------------------------------------------------------------------------- //
// Enumerations
// ------------------------------------------------------------------------------------------------------- //

// Calibration status flags
typedef enum
{
    CAL_DONE,                   // Calibration is done
    CAL_PENDING,                // Calibration is pending
    CAL_RUNNING,                // Calibration is running
    sizeof_calibration_status_t // Do not change.
} calibration_status_t;

// ------------------------------------------------------------------------------------------------------- //
// Structs
// ------------------------------------------------------------------------------------------------------- //

// Cart variables
typedef struct
{
    float Pos;  // Position (mm)
    float Vel;  // Velocity (mm/s)
} cart_t;

// Cart variables -  Default values
#define cart_t_default { \
    .Pos = 0, \
    .Vel = 0, \
}

// Pendulum variables
typedef struct
{
    float Pos;  // Position (degrees)
    float Vel;  // Velocity (degrees/s)
} pendulum_t;

// Pendulum variables -  Default values
#define pendulum_t_default { \
    .Pos = PI, \
    .Vel = 0, \
}

// Stepper variables
typedef struct
{
    bool Enabled;       // Enabled status
    bool Dir;           // Direction (1 = forward, 0 = backward)
    uint32_t TargetPPS; // Target velocity (pulses per second)
    uint32_t CurrentPPS;// Target velocity (pulses per second)
    int32_t DeltaPPS;   // PPS increment in every timer loop (pulses)
    bool AccEnabled;    // Acceleration enabled status
    float AccValue;     // Acceleration (m/s^2)
    uint32_t PwmPeriod; // PWM period (clock ticks)
    uint32_t PwmClock;  // PWM clock (Hz)
} stepper_t;

// Stepper variables - Default values
#define stepper_t_default { \
    .Enabled = false, \
    .Dir = false, \
    .TargetPPS = STEPPER_PPS_MIN, \
    .CurrentPPS = STEPPER_PPS_MIN, \
    .DeltaPPS = 0, \
    .AccEnabled = false, \
    .PwmPeriod = 0, \
    .PwmClock = 0, \
}

// Calibration variables
typedef struct
{
    calibration_status_t Status;    // Status
    uint8_t Progress;               // Progress (0 to 100)
    int32_t Offset;                 // Encoder counter offset
    uint32_t Max;                   // Maximum encoder counter value
} calibration_t;

// LQR controller variables
typedef struct
{
    float Ref_Shaddow;      // Position setpoint (shaddow value)
    float Ref[5];           // Setpoint
    float States[5];        // State values (sample k)
    float E_now[5];         // Error (sample k)
    float K[5];             // Gain matrix
    float U_nxt;            // Control action (sample k + 1)
} lqr_t;

// LQR controller variables - Default values
#define lqr_t_default { \
    .Ref_Shaddow = 0, \
    .Ref = {0, 0, 0, 0, 0}, \
    .States = {0, 0, 0, 0, 0}, \
    .E_now = {0, 0, 0, 0, 0}, \
    .K = {-5, -5, -35, -5, 2}, \
    .U_nxt = 0, \
}

// Device firmware info struct
typedef struct
{
    char Name[12];          // Firmware name - 11 bytes + \0
    char Version[5];        // Firmware version - 4 bytes + \0
    char Timestamp[22];     // Firmware timestamp - 21 bytes + \0
    char Author[15];        // Firmware author
} firmware_info_t;

// Device firmware info struct - Default values
#define firmware_info_t_default { \
    .Name = DEVICE_FW_NAME, \
    .Version = DEVICE_FW_VERSION, \
    .Timestamp = __DATE__ " " __TIME__ "\0" \
    .Author = DEVICE_FW_AUTHOR, \
}

// ------------------------------------------------------------------------------------------------------- //
// Structs
// ------------------------------------------------------------------------------------------------------- //


// ------------------------------------------------------------------------------------------------------- //

#ifdef __cplusplus
}
#endif

#endif

// ------------------------------------------------------------------------------------------------------- //

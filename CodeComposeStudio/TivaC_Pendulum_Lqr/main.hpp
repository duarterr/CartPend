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

// LCD functions
#include "Lcd_TivaC.hpp"

// Button functions
#include "Button_TivaC.hpp"

// UART functions
#include "Uart_TivaC.hpp"

// Encoder functions
#include "Encoder_TivaC.hpp"

// RGB LED functions
#include "Rgb_TivaC.hpp"

// Stepper functions
#include "Stepper_TivaC.hpp"

// Auxiliary functions
#include "Aux_Functions.hpp"

// LQR functions
#include "Lqr_TivaC.hpp"

// PID functions
#include "Pid_TivaC.hpp"

// Lead functions
#include "Lead_TivaC.hpp"

// Standard libraries
#include <stdint.h>

// TivaC hardware related libraries
#include "inc/hw_memmap.h"

// TivaC driver libraries
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/qei.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/watchdog.h"

// ------------------------------------------------------------------------------------------------------- //
// Constants
// ------------------------------------------------------------------------------------------------------- //

#define PI                      3.1415926535898F       // Value of Pi

// ------------------------------------------------------------------------------------------------------- //
// Application defines
// ------------------------------------------------------------------------------------------------------- //

// Firmware info
#define DEVICE_FW_NAME          "InvPend LQR"          // Firmware name
#define DEVICE_FW_VERSION       "1.0"                  // Firmware version
#define DEVICE_FW_AUTHOR        "Renan Duarte"         // Firmware author

// Application timming
#define TIMER_FREQUENCY         1000                   // Systick timer frequency in Hz
#define CONTROL_LOOP_FREQUENCY  200                    // Control loop frequency in Hz

// LCD
#define LCD_REFRESH_FREQUENCY   10                     // LCD refresh frequency in Hz

// Buttons
#define BUTTON_SCAN_FREQUENCY   1000                   // Button scan frequency in Hz
#define BUTTON_SCAN_INTERVAL    (1000/BUTTON_SCAN_FREQUENCY) // Button scan interval in ms
#define BUTTON_DEAD_TIME        10                     // Button dead time in ms
#define BUTTON_WINDOW           250                    // Button window time in ms
#define BUTTON_LONG_TIMEOUT     1000                   // Button long press timeout in ms

// UART
#define UART_REFRESH_FREQUENCY  50                     // UART refresh frequency in Hz
#define UART_BAUD_RATE          115200                 // UART baud rate
#define UART_MODE               (UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE | UART_CONFIG_WLEN_8) // UART mode configuration

// Encoder - Theta
#define ENCODER_T_FREQUENCY     200                     // Encoder scan frequency
#define ENCODER_T_PPR           4000                    // Encoder maximum counter value
#define ENCODER_T_CAL_CYCLES    10                      // Oscillations required for calibration

// Encoder - X
#define ENCODER_X_FREQUENCY     200                     // Encoder scan frequency
#define ENCODER_X_PPR           83300                   // Encoder maximum counter value

// RGB LED
#define RGB_PWM_FREQ            1000                   // RGB LED PWM frequency in Hz
#define RGB_REFRESH_FREQUENCY   10                     // RGB LED refresh frequency in Hz

// Stepper
#define STEPPER_STEPS_REV       200                                     // Number of steps per revolution
#define STEPPER_MICROSTEPS      32                                      // Number of pulses per step
#define STEPPER_PPR             (STEPPER_STEPS_REV*STEPPER_MICROSTEPS)  // Total pulses per revolution
#define STEPPER_PD              0.0136F                                 // Pulley pitch diameter (m) - 0.01273 nominal
#define STEPPER_KV              (STEPPER_PPR/(PI*STEPPER_PD))           // Conversion factor between PPS and m/s
#define STEPPER_PPS_MAX         200000                                  // Maximum velocity (pulses per second)
#define STEPPER_VEL_MAX         ((float)STEPPER_PPS_MAX/STEPPER_KV)     // Maximum velocity (m/s)
#define STEPPER_ACC_MAX         10                                      // Maximum acceleration (m/s^2)
#define STEPPER_REFRESH_FREQ    1000                                    // Velocity control frequency

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
#define LCD_SSI_PERIPH          SYSCTL_PERIPH_SSI0     // LCD SSI peripheral
#define LCD_SCLK_PERIPH         SYSCTL_PERIPH_GPIOA    // LCD SCLK GPIO peripheral
#define LCD_DN_PERIPH           SYSCTL_PERIPH_GPIOA    // LCD DN GPIO peripheral
#define LCD_SCE_PERIPH          SYSCTL_PERIPH_GPIOA    // LCD SCE GPIO peripheral
#define LCD_DC_PERIPH           SYSCTL_PERIPH_GPIOA    // LCD DC GPIO peripheral
#define LCD_BKL_PERIPH          SYSCTL_PERIPH_GPIOA    // LCD BKL GPIO peripheral
#define LCD_SSI_BASE            SSI0_BASE              // Base address for SSI0
#define LCD_SCLK_BASE           GPIO_PORTA_BASE        // Base address for SCLK GPIO port
#define LCD_DN_BASE             GPIO_PORTA_BASE        // Base address for DN GPIO port
#define LCD_SCE_BASE            GPIO_PORTA_BASE        // Base address for SCE GPIO port
#define LCD_DC_BASE             GPIO_PORTA_BASE        // Base address for DC GPIO port
#define LCD_BKL_BASE            GPIO_PORTA_BASE        // Base address for BKL GPIO port
#define LCD_SCLK_CFG            GPIO_PA2_SSI0CLK       // Pin mux configuration for SCLK
#define LCD_DN_CFG              GPIO_PA5_SSI0TX        // Pin mux configuration for DN
#define LCD_SCLK_PIN            GPIO_PIN_2             // SCLK pin
#define LCD_DN_PIN              GPIO_PIN_5             // DN pin
#define LCD_SCE_PIN             GPIO_PIN_4             // EN pin
#define LCD_DC_PIN              GPIO_PIN_3             // DC pin
#define LCD_BKL_PIN             GPIO_PIN_6             // BKL pin

// Buttons
#define BUTTON_GPIO_PERIPH      SYSCTL_PERIPH_GPIOF    // Button GPIO peripheral
#define BUTTON_GPIO_BASE        GPIO_PORTF_BASE        // Base address for Button GPIO port
#define BUTTON_PIN_1            GPIO_PIN_4             // Button 1 pin
#define BUTTON_PIN_2            GPIO_PIN_0             // Button 2 pin

// UART defines
#define UART_UART_PERIPH        SYSCTL_PERIPH_UART0    // UART peripheral
#define UART_GPIO_PERIPH        SYSCTL_PERIPH_GPIOA    // UART GPIO peripheral
#define UART_UART_BASE          UART0_BASE             // Base address for UART0
#define UART_GPIO_BASE          GPIO_PORTA_BASE        // Base address for UART GPIO port
#define UART_RX_CFG             GPIO_PA0_U0RX          // Pin mux configuration for UART RX
#define UART_TX_CFG             GPIO_PA1_U0TX          // Pin mux configuration for UART TX
#define UART_RX_PIN             GPIO_PIN_0             // UART RX pin
#define UART_TX_PIN             GPIO_PIN_1             // UART TX pin

// Encoder defines - Theta
#define ENCODER_T_QEI_PERIPH    SYSCTL_PERIPH_QEI0     // QEI peripheral for Theta encoder
#define ENCODER_T_GPIO_PERIPH   SYSCTL_PERIPH_GPIOD    // GPIO peripheral for Theta encoder
#define ENCODER_T_QEI_BASE      QEI0_BASE              // Base address for Theta QEI module
#define ENCODER_T_GPIO_BASE     GPIO_PORTD_BASE        // Base address for Theta GPIO port
#define ENCODER_T_A_CFG         GPIO_PD6_PHA0          // Pin mux configuration for Theta encoder Phase A
#define ENCODER_T_B_CFG         GPIO_PD7_PHB0          // Pin mux configuration for Theta encoder Phase B
#define ENCODER_T_A_PIN         GPIO_PIN_6             // Theta encoder Phase A pin
#define ENCODER_T_B_PIN         GPIO_PIN_7             // Theta encoder Phase B pin
#define ENCODER_T_CFG           QEI_CONFIG_SWAP        // Theta encoder configuration

// Encoder defines - X
#define ENCODER_X_QEI_PERIPH    SYSCTL_PERIPH_QEI1     // QEI peripheral for X encoder
#define ENCODER_X_GPIO_PERIPH   SYSCTL_PERIPH_GPIOC    // GPIO peripheral for X encoder
#define ENCODER_X_QEI_BASE      QEI1_BASE              // Base address for X QEI module
#define ENCODER_X_GPIO_BASE     GPIO_PORTC_BASE        // Base address for X GPIO port
#define ENCODER_X_A_CFG         GPIO_PC5_PHA1          // Pin mux configuration for X encoder Phase A
#define ENCODER_X_B_CFG         GPIO_PC6_PHB1          // Pin mux configuration for X encoder Phase B
#define ENCODER_X_A_PIN         GPIO_PIN_5             // X encoder Phase A pin
#define ENCODER_X_B_PIN         GPIO_PIN_6             // X encoder Phase B pin
#define ENCODER_X_CFG           QEI_CONFIG_SWAP        // X encoder configuration

// RGB LED
#define RGB_PWM_PERIPH          SYSCTL_PERIPH_PWM1      // RGB PWM peripheral
#define RGB_GPIO_PERIPH         SYSCTL_PERIPH_GPIOF     // RGB GPIO peripheral
#define RGB_TIMER_PERIPH        SYSCTL_PERIPH_WTIMER1   // Peripheral for timer module
#define RGB_PWM_BASE            PWM1_BASE               // RGB PWM base
#define RGB_GPIO_BASE           GPIO_PORTF_BASE         // RGB GPIO base
#define RGB_TIMER_BASE          WTIMER1_BASE            // Base address for timer module
#define RGB_PWM_R_GEN           PWM_GEN_2               // PWM generator for Red
#define RGB_PWM_G_GEN           PWM_GEN_3               // PWM generator for Green
#define RGB_PWM_B_GEN           PWM_GEN_3               // PWM generator for Blue
#define RGB_PWM_R_OUT           PWM_OUT_5               // PWM output for Red
#define RGB_PWM_G_OUT           PWM_OUT_7               // PWM output for Green
#define RGB_PWM_B_OUT           PWM_OUT_6               // PWM output for Blue
#define RGB_PWM_R_OUT_BIT       PWM_OUT_5_BIT           // PWM output bit for Red
#define RGB_PWM_G_OUT_BIT       PWM_OUT_7_BIT           // PWM output bit for Green
#define RGB_PWM_B_OUT_BIT       PWM_OUT_6_BIT           // PWM output bit for Blue
#define RGB_PIN_R_CFG           GPIO_PF1_M1PWM5         // Pin mux configuration for Red
#define RGB_PIN_G_CFG           GPIO_PF3_M1PWM7         // Pin mux configuration for Green
#define RGB_PIN_B_CFG           GPIO_PF2_M1PWM6         // Pin mux configuration for Blue
#define RGB_R_PIN               GPIO_PIN_1              // Red LED pin
#define RGB_G_PIN               GPIO_PIN_3              // Green LED pin
#define RGB_B_PIN               GPIO_PIN_2              // Blue LED pin

// Stepper
#define STEPPER_PWM_PERIPH      SYSCTL_PERIPH_PWM0      // Peripheral for PWM module
#define STEPPER_PWM_BASE        PWM0_BASE               // Base address for PWM module
#define STEPPER_PWM_GEN         PWM_GEN_1               // PWM generator
#define STEPPER_PWM_OUT         PWM_OUT_3               // PWM output
#define STEPPER_PWM_OUT_BIT     PWM_OUT_3_BIT           // PWM output bit
#define STEPPER_STEP_PERIPH     SYSCTL_PERIPH_GPIOB     // Peripheral for step pin
#define STEPPER_STEP_BASE       GPIO_PORTB_BASE         // Base address for step pin
#define STEPPER_STEP_PIN        GPIO_PIN_5              // Step pin number
#define STEPPER_STEP_PIN_MUX    GPIO_PB5_M0PWM3         // Step pin mux configuration
#define STEPPER_DIR_PERIPH      SYSCTL_PERIPH_GPIOB     // Peripheral for direction pin
#define STEPPER_DIR_BASE        GPIO_PORTB_BASE         // Base address for direction pin
#define STEPPER_DIR_PIN         GPIO_PIN_0              // Direction pin number
#define STEPPER_EN_PERIPH       SYSCTL_PERIPH_GPIOB     // Peripheral for enable pin
#define STEPPER_EN_BASE         GPIO_PORTB_BASE         // Base address for enable pin
#define STEPPER_EN_PIN          GPIO_PIN_1              // Enable pin number
#define STEPPER_LIMSTART_PERIPH SYSCTL_PERIPH_GPIOE     // Peripheral for start limit switch
#define STEPPER_LIMSTART_BASE   GPIO_PORTE_BASE         // Base address for start limit switch
#define STEPPER_LIMSTART_PIN    GPIO_PIN_2              // Start limit switch pin number
#define STEPPER_LIMEND_PERIPH   SYSCTL_PERIPH_GPIOE     // Peripheral for end limit switch
#define STEPPER_LIMEND_BASE     GPIO_PORTE_BASE         // Base address for end limit switch
#define STEPPER_LIMEND_PIN      GPIO_PIN_3              // End limit switch pin number
#define STEPPER_TIMER_PERIPH    SYSCTL_PERIPH_WTIMER0   // Peripheral for timer module
#define STEPPER_TIMER_BASE      WTIMER0_BASE            // Base address for timer module

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

// Control modes
typedef enum
{
    CONTROL_OFF,            // No control
    CONTROL_LQR,            // LQR control
    CONTROL_PID,            // PID control
    CONTROL_LEAD,           // Lead control
    sizeof_control_mode_t   // Do not change.
} control_mode_t;

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

// Calibration variables
typedef struct
{
    calibration_status_t Status;    // Status
    uint8_t Progress;               // Progress (0 to 100)
    int32_t Offset;                 // Encoder counter offset
    uint32_t Max;                   // Maximum encoder counter value
} calibration_t;

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
// Configuration variables - Constants
// ------------------------------------------------------------------------------------------------------- //

// Define LCD configuration parameters
const lcd_config_t Lcd_Config = {
    .Ssi = {
        .Periph = LCD_SSI_PERIPH,
        .Base = LCD_SSI_BASE
    },
    .Sclk = {
        .Periph = LCD_SCLK_PERIPH,
        .Base = LCD_SCLK_BASE,
        .PinMux = LCD_SCLK_CFG,
        .Pin = LCD_SCLK_PIN
    },
    .Dn = {
        .Periph = LCD_DN_PERIPH,
        .Base = LCD_DN_BASE,
        .PinMux = LCD_DN_CFG,
        .Pin = LCD_DN_PIN
    },
    .Sce = {
        .Periph = LCD_SCE_PERIPH,
        .Base = LCD_SCE_BASE,
        .Pin = LCD_SCE_PIN
    },
    .Dc = {
        .Periph = LCD_DC_PERIPH,
        .Base = LCD_DC_BASE,
        .Pin = LCD_DC_PIN
    },
    .Bkl = {
        .Periph = LCD_BKL_PERIPH,
        .Base = LCD_BKL_BASE,
        .Pin = LCD_BKL_PIN
    }
};

// Define RGB LED configuration parameters
const rgb_config_t Led_Config = {
    .Periph = {
        .Pwm = RGB_PWM_PERIPH,
        .Gpio = RGB_GPIO_PERIPH,
        .Timer = RGB_TIMER_PERIPH
    },
    .Base = {
        .Pwm = RGB_PWM_BASE,
        .Gpio = RGB_GPIO_BASE,
        .Timer = RGB_TIMER_BASE
    },
    .Gen = {
        .R = RGB_PWM_R_GEN,
        .G = RGB_PWM_G_GEN,
        .B = RGB_PWM_B_GEN
    },
    .Out = {
        .R = RGB_PWM_R_OUT,
        .G = RGB_PWM_G_OUT,
        .B = RGB_PWM_B_OUT
    },
    .OutBit = {
        .R = RGB_PWM_R_OUT_BIT,
        .G = RGB_PWM_G_OUT_BIT,
        .B = RGB_PWM_B_OUT_BIT
    },
    .PinMux = {
        .R = RGB_PIN_R_CFG,
        .G = RGB_PIN_G_CFG,
        .B = RGB_PIN_B_CFG
    },
    .Pin = {
        .R = RGB_R_PIN,
        .G = RGB_G_PIN,
        .B = RGB_B_PIN
    },
    .Params = {
        .PwmFrequency = RGB_PWM_FREQ
    }
};

// Define button 1 parameters
const button_config_t Button1_Config = {
    .Hardware = {
        .Periph = BUTTON_GPIO_PERIPH,
        .Base = BUTTON_GPIO_BASE,
        .Pin = BUTTON_PIN_1
    },
    .Params = {
        .Interval = BUTTON_SCAN_INTERVAL,
        .DeadTime = BUTTON_DEAD_TIME,
        .Window = BUTTON_WINDOW,
        .LongClickTimeout = BUTTON_LONG_TIMEOUT
    }
};

// Define button 2 parameters
const button_config_t Button2_Config = {
    .Hardware = {
        .Periph = BUTTON_GPIO_PERIPH,
        .Base = BUTTON_GPIO_BASE,
        .Pin = BUTTON_PIN_2
    },
    .Params = {
        .Interval = BUTTON_SCAN_INTERVAL,
        .DeadTime = BUTTON_DEAD_TIME,
        .Window = BUTTON_WINDOW,
        .LongClickTimeout = BUTTON_LONG_TIMEOUT
    }
};

// Define UART parameters
const uart_config_t Uart_Config = {
    .Hardware = {
        .PeriphUART = UART_UART_PERIPH,
        .PeriphGPIO = UART_GPIO_PERIPH,
        .BaseUART = UART_UART_BASE,
        .BaseGPIO = UART_GPIO_BASE,
        .PinMuxRX = UART_RX_CFG,
        .PinMuxTX = UART_TX_CFG,
        .PinRX = UART_RX_PIN,
        .PinTX = UART_TX_PIN
    },
    .Params = {
        .BaudRate = UART_BAUD_RATE,
        .Mode = UART_MODE
    }
};

// Define encoder parameters - Theta
const encoder_config_t EncoderT_Config = {
    .Hardware = {
        .PeriphQEI = ENCODER_T_QEI_PERIPH,
        .PeriphGPIO = ENCODER_T_GPIO_PERIPH,
        .BaseQEI = ENCODER_T_QEI_BASE,
        .BaseGPIO = ENCODER_T_GPIO_BASE,
        .PinMuxA = ENCODER_T_A_CFG,
        .PinMuxB = ENCODER_T_B_CFG,
        .PinA = ENCODER_T_A_PIN,
        .PinB = ENCODER_T_B_PIN,
        .Config = ENCODER_T_CFG
    },
    .Params = {
        .PPR = ENCODER_T_PPR,
        .ScanFreq = ENCODER_T_FREQUENCY
    }
};

// Define encoder parameters - X
const encoder_config_t EncoderX_Config = {
    .Hardware = {
        .PeriphQEI = ENCODER_X_QEI_PERIPH,
        .PeriphGPIO = ENCODER_X_GPIO_PERIPH,
        .BaseQEI = ENCODER_X_QEI_BASE,
        .BaseGPIO = ENCODER_X_GPIO_BASE,
        .PinMuxA = ENCODER_X_A_CFG,
        .PinMuxB = ENCODER_X_B_CFG,
        .PinA = ENCODER_X_A_PIN,
        .PinB = ENCODER_X_B_PIN,
        .Config = ENCODER_X_CFG
    },
    .Params = {
        .PPR = ENCODER_X_PPR,
        .ScanFreq = ENCODER_X_FREQUENCY
    }
};

// Define stepper motor configuration parameters
const stepper_config_t Stepper_Config = {
    .Pwm = {
        .Periph = STEPPER_PWM_PERIPH,
        .Base = STEPPER_PWM_BASE,
        .Gen = STEPPER_PWM_GEN,
        .Out = STEPPER_PWM_OUT,
        .OutBit = STEPPER_PWM_OUT_BIT
    },
    .Step = {
        .Periph = STEPPER_STEP_PERIPH,
        .Base = STEPPER_STEP_BASE,
        .Pin = STEPPER_STEP_PIN,
        .PinMux = STEPPER_STEP_PIN_MUX
    },
    .Dir = {
        .Periph = STEPPER_DIR_PERIPH,
        .Base = STEPPER_DIR_BASE,
        .Pin = STEPPER_DIR_PIN
    },
    .En = {
        .Periph = STEPPER_EN_PERIPH,
        .Base = STEPPER_EN_BASE,
        .Pin = STEPPER_EN_PIN
    },
    .LimStart = {
        .Periph = STEPPER_LIMSTART_PERIPH,
        .Base = STEPPER_LIMSTART_BASE,
        .Pin = STEPPER_LIMSTART_PIN
    },
    .LimEnd = {
        .Periph = STEPPER_LIMEND_PERIPH,
        .Base = STEPPER_LIMEND_BASE,
        .Pin = STEPPER_LIMEND_PIN
    },
    .Params = {
        .VelMax = STEPPER_VEL_MAX,
        .AccMax = STEPPER_ACC_MAX,
        .Kv = STEPPER_KV,
        .VelUpdateFrequency = TIMER_FREQUENCY
    },
    .Timer = {
        .Periph = STEPPER_TIMER_PERIPH,
        .Base = STEPPER_TIMER_BASE
    }
};

// ------------------------------------------------------------------------------------------------------- //

#ifdef __cplusplus
}
#endif

#endif

// ------------------------------------------------------------------------------------------------------- //

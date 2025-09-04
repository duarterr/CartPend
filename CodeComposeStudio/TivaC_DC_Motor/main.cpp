// ------------------------------------------------------------------------------------------------------- //
// DC Motor Simple Test
// ------------------------------------------------------------------------------------------------------- //

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

#include "DCMotor_TivaC.hpp"

// ------------------------------------------------------------------------------------------------------- //
// DC Motor Configuration
// ------------------------------------------------------------------------------------------------------- //

const dcmotor_config_t DCMotor_Config = {
    .Pwm = {
        .Periph = SYSCTL_PERIPH_PWM0,
        .Base = PWM0_BASE,
        .Gen = PWM_GEN_1,
        .Out = PWM_OUT_3,
        .OutBit = PWM_OUT_3_BIT
    },
    .PwmPin = {
        .Periph = SYSCTL_PERIPH_GPIOB,
        .Base = GPIO_PORTB_BASE,
        .Pin = GPIO_PIN_5,
        .PinMux = GPIO_PB5_M0PWM3
    },
    .EnableCW = {
        .Periph = SYSCTL_PERIPH_GPIOB,
        .Base = GPIO_PORTB_BASE,
        .Pin = GPIO_PIN_1
    },
    .EnableCCW = {
        .Periph = SYSCTL_PERIPH_GPIOB,
        .Base = GPIO_PORTB_BASE,
        .Pin = GPIO_PIN_0
    },
    .LimStart = {
        .Periph = SYSCTL_PERIPH_GPIOE,
        .Base = GPIO_PORTE_BASE,
        .Pin = GPIO_PIN_2
    },
    .LimEnd = {
        .Periph = SYSCTL_PERIPH_GPIOE,
        .Base = GPIO_PORTE_BASE,
        .Pin = GPIO_PIN_3
    },
    .Params = {
        .AbsVelMin = 0.0f,
        .AbsVelMax = 1.0f,
    }
};

// ------------------------------------------------------------------------------------------------------- //
// Main
// ------------------------------------------------------------------------------------------------------- //

// Debug test variables - modify these in debugger
volatile bool enable = false;      // Set to true to enable motor
volatile float vel = 0.0f;         // Set velocity (-1.0 to 1.0)


void main()
{
    // Configure clock - 80 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // Enable interrupts
    IntMasterEnable();

    // Initialize DC Motor
    DCMotor Motor;
    Motor.Init(&DCMotor_Config);

    // State tracking
    static bool last_enable = false;
    static float last_vel = 0.0f;

    // Simple debug test loop
    while (1)
    {
        // Check if enable state changed
        if (enable != last_enable)
        {
            last_enable = enable;

            if (enable)
            {
                // Motor was enabled - set the current velocity
                Motor.SetVelocity(vel);
                last_vel = vel;
            }
            else
            {
                // Motor was disabled - stop it
                Motor.Stop();
            }
        }

        // Check if velocity changed (only if motor is enabled)
        else if (enable && (vel != last_vel))
        {
            last_vel = vel;
            Motor.SetVelocity(vel);
        }

        // Small delay to prevent excessive CPU usage
        SysCtlDelay(800000);    // ~100ms delay at 80MHz
    }
}

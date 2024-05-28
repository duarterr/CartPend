// ------------------------------------------------------------------------------------------------------- //

// RGB LED library
// Version: 1.0
// Author: Renan Duarte
// E-mail: duarte.renan@hotmail.com
// Date:   20/05/2024

// ------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------- //
// Includes
// ------------------------------------------------------------------------------------------------------- //

// RGB defines and macros
#include "Rgb_TivaC.hpp"

// Standard libraries
#include <stdint.h>

// Auxiliary functions
#include <Aux_Functions.hpp>

// TivaC device defines and macros
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"

// ------------------------------------------------------------------------------------------------------- //
// Functions definitions
// ------------------------------------------------------------------------------------------------------- //

// Name:        _InitHardware
// Description: Starts device peripherals
// Arguments:   None
// Returns:     None

void Rgb::_InitHardware()
{
    // Enable peripheral clocks
    SysCtlPeripheralEnable(_Config.Periph.Pwm);
    SysCtlPeripheralEnable(_Config.Periph.Gpio);

    // Power up delay
    SysCtlDelay(10);

    // Unlock used pins (has no effect if pin is not protected by the GPIOCR register
    GPIOUnlockPin(_Config.Base.Gpio, _Config.Pin.R | _Config.Pin.G | _Config.Pin.B);

    // Configure pins as PWM outputs
    GPIOPinTypePWM (_Config.Base.Gpio, _Config.Pin.R | _Config.Pin.G | _Config.Pin.B);
    GPIOPinConfigure (_Config.PinMux.R);
    GPIOPinConfigure (_Config.PinMux.G);
    GPIOPinConfigure (_Config.PinMux.B);

    // Configure PWM options
    PWMGenConfigure (_Config.Base.Pwm, _Config.Gen.R, _Config.Params.PwmMode);
    PWMGenConfigure (_Config.Base.Pwm, _Config.Gen.G, _Config.Params.PwmMode);
    PWMGenConfigure (_Config.Base.Pwm, _Config.Gen.B, _Config.Params.PwmMode);

    // Set the PWM frequency
    _SetPwmFrequency (_Config.Params.PwmFrequency);

    // Set initial color - Off
    SetColor (_NewColor, 0);

    // Interrupt on zero count
    PWMGenIntTrigEnable(_Config.Base.Pwm, _Config.Gen.R, PWM_INT_CNT_ZERO);

    // Register interrupt handler
    PWMGenIntRegister(_Config.Base.Pwm, _Config.Gen.R, _Config.Int.Callback);

    // Enable interrupt
    PWMIntEnable(_Config.Base.Pwm, _Config.Int.Gen);
    IntEnable(_Config.Int.Interrupt);

    // Turn on the output pins
    PWMOutputState (_Config.Base.Pwm, _Config.OutBit.R | _Config.OutBit.G | _Config.OutBit.B , true);

    // Enable the PWM generators
    PWMGenEnable (_Config.Base.Pwm, _Config.Gen.R);
    PWMGenEnable (_Config.Base.Pwm, _Config.Gen.G);
    PWMGenEnable (_Config.Base.Pwm, _Config.Gen.B);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        _SetPwmFrequency
// Description: Changes PWM modules frequency
// Arguments:   Frequency
// Returns:     None

void Rgb::_SetPwmFrequency (uint32_t Frequency)
{
    // Set the period (expressed in clock ticks)
    _PwmPeriod = (_GetPwmClock()/Frequency) - 1;
    PWMGenPeriodSet (_Config.Base.Pwm, _Config.Gen.R, _PwmPeriod);
    PWMGenPeriodSet (_Config.Base.Pwm, _Config.Gen.G, _PwmPeriod);
    PWMGenPeriodSet (_Config.Base.Pwm, _Config.Gen.B, _PwmPeriod);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        _SetPwmDuty
// Description: Changes PWM modules duty cycle
// Arguments:   DutyR, DutyG, DutyB - Duty cycles - 0 to _PwmPeriod
// Returns:     None

void Rgb::_SetPwmDuty (uint16_t DutyR, uint16_t DutyG, uint16_t DutyB)
{
    PWMPulseWidthSet (_Config.Base.Pwm, _Config.Out.R , DutyR);
    PWMPulseWidthSet (_Config.Base.Pwm, _Config.Out.G , DutyG);
    PWMPulseWidthSet (_Config.Base.Pwm, _Config.Out.B , DutyB);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        _GetPwmClock
// Description: Gets PWM module clock
// Arguments:   None
// Returns:     Clock in Hz

uint32_t Rgb::_GetPwmClock()
{
    uint8_t ClockShifts = 0;

    switch (SysCtlPWMClockGet())
    {
        case SYSCTL_PWMDIV_1:
            ClockShifts = 0;
            break;

        case SYSCTL_PWMDIV_2:
            ClockShifts = 1;
            break;

        case SYSCTL_PWMDIV_4:
            ClockShifts = 2;
            break;

        case SYSCTL_PWMDIV_8:
            ClockShifts = 3;
            break;

        case SYSCTL_PWMDIV_16:
            ClockShifts = 4;
            break;

        case SYSCTL_PWMDIV_32:
            ClockShifts = 5;
            break;

        case SYSCTL_PWMDIV_64:
            ClockShifts = 6;
            break;

        default:
            break;
    }

    return SysCtlClockGet() >> ClockShifts;
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        _ColorsAreEqual
// Description: Check if two colors are equal
// Arguments:   Color1 - First color - rgb_color_t value
//              Color2 - Second color - rgb_color_t value
// Returns:     True if equal. False otherwise

bool Rgb::_ColorsAreEqual(rgb_color_t Color1, rgb_color_t Color2)
{
    return (Color1.R == Color2.R && Color1.G == Color2.G && Color1.B == Color2.B);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        _CalculateStepSkip
// Description: Determine the number of steps to skip during the fade
// Arguments:   NewValue - First value
//              CurrentValue - Second value
//              FadeSteps - Number of fade steps
// Returns:     True if equal. False otherwise

uint16_t Rgb::_CalculateStepSkip(uint16_t NewValue, uint16_t CurrentValue, uint16_t FadeSteps)
{
    uint16_t Diff = abs((int16_t)NewValue - CurrentValue);

    if (Diff == 0)
        return 0;

    else if (Diff >= FadeSteps)
        return 1;

    else
        return (float)FadeSteps / Diff + 0.5;
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        Rgb
// Description: Constructor of the class with no arguments
// Arguments:   None
// Returns:     None

Rgb::Rgb()
{

}

// ------------------------------------------------------------------------------------------------------- //

// Name:        Rgb
// Description: Constructor of the class with rgb_config_t struct as argument
// Arguments:   Config - rgb_config_t struct
// Returns:     None

Rgb::Rgb(rgb_config_t *Config) : Rgb()
{
    Init(Config);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        Init
// Description: Starts device peripherals and application state machine
// Arguments:   Config - rgb_config_t struct
// Returns:     None

void Rgb::Init(rgb_config_t *Config)
{
    // Get rgb_config_t object parameters and store in a "private" variable
    memcpy(&_Config, Config, sizeof(rgb_config_t));

    //  Initialize hardware
    _InitHardware();
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        SetColor
// Description: Sets the RGB LED color
// Arguments:   Color - New color - rgb_color_t structure (0x00 to 0xFF values)
//              FadeTime - Fade time to new color (ms)
// Returns:     None

void Rgb::SetColor(rgb_color_t Color, uint16_t FadeTime)
{
    _NewColor.R = Color.R;
    _NewColor.G = Color.G;
    _NewColor.B = Color.B;

    _FadeSteps = (FadeTime != 0) * (FadeTime*1000) / _Config.Params.PwmFrequency;
}

// Name:        GetColor
// Description: Gets the current LED color
// Arguments:   Buffer - Pointer rgb_color_t structure where values will be saved (0x00 to 0xFF values)
// Returns:     None

void Rgb::GetColor(rgb_color_t *Buffer)
{
    Buffer->R = _CurrentColor.R;
    Buffer->G = _CurrentColor.G;
    Buffer->B = _CurrentColor.B;
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        PwmIsr
// Description: PWM interrupt service routine
// Arguments:   None
// Returns:     None

void Rgb::PwmIsr ()
{
    static bool _IsFading = false;
    static uint16_t _StepSkipR = 0;
    static uint16_t _StepSkipG = 0;
    static uint16_t _StepSkipB = 0;

    // Make sure PWM frequency is right (may change if PWM clock is divided externally)
    _SetPwmFrequency (_Config.Params.PwmFrequency);

    // Fade to new color
    if (_FadeSteps != 0 && !_ColorsAreEqual(_CurrentColor, _NewColor))
    {
        // Not already fading
        if (!_IsFading)
        {
            // Reset variables
            _StepCounter = 0;
            _IsFading = true;

            // Calculate number of steps to skip for each color component
            _StepSkipR = _CalculateStepSkip(_NewColor.R, _CurrentColor.R, _FadeSteps);
            _StepSkipG = _CalculateStepSkip(_NewColor.G, _CurrentColor.G, _FadeSteps);
            _StepSkipB = _CalculateStepSkip(_NewColor.B, _CurrentColor.B, _FadeSteps);
        }

        // Increase step counter
        _StepCounter++;

        // Change color components
        if ((_StepCounter % _StepSkipR) == 0)
            _CurrentColor.R += (_NewColor.R > _CurrentColor.R ? 1 : -1);
        if ((_StepCounter % _StepSkipG) == 0)
            _CurrentColor.G += (_NewColor.G > _CurrentColor.G ? 1 : -1);
        if ((_StepCounter % _StepSkipB) == 0)
            _CurrentColor.B += (_NewColor.B > _CurrentColor.B ? 1 : -1);

        // Fade is complete
        if (_StepCounter >= _FadeSteps)
        {
            _CurrentColor = _NewColor;
            _IsFading = false;
        }
    }

    // No fade
    else
    {
        _CurrentColor = _NewColor;
        _IsFading = false;
    }

    // Define and apply duty cycles
    uint16_t DutyR = (uint16_t)Aux::Map(_CurrentColor.R, 0, 255, 1, _PwmPeriod);
    uint16_t DutyG = (uint16_t)Aux::Map(_CurrentColor.G, 0, 255, 1, _PwmPeriod);
    uint16_t DutyB = (uint16_t)Aux::Map(_CurrentColor.B, 0, 255, 1, _PwmPeriod);

    _SetPwmDuty(DutyR, DutyG, DutyB);

    // Clear interrupt flag
    PWMGenIntClear(_Config.Base.Pwm, _Config.Gen.R, PWM_INT_CNT_ZERO);
}

// ------------------------------------------------------------------------------------------------------- //

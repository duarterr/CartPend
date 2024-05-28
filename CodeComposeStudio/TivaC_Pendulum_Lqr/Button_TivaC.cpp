// ------------------------------------------------------------------------------------------------------- //

// Button interface with support to single or multiple short and long clicks
// Version: 1.0
// Author: Renan Duarte
// E-mail: duarte.renan@hotmail.com
// Date:   13/05/2024

// ------------------------------------------------------------------------------------------------------- //
// Behaviour
// ------------------------------------------------------------------------------------------------------- //

//  It is assumed that the button is active low (connected to GND when pressed). Internal pullup resistor
//  is used.
//  The ScanEvent function should be called every "Interval" ms. It will return true if an event was
//  detected. The "EventData" variable will contain the event (from button_event_code_t) and the counter
//  value (number of times the event occurred)

// ------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------- //
// Includes
// ------------------------------------------------------------------------------------------------------- //

// Button defines and macros
#include "Button_TivaC.hpp"

// Standard libraries
#include <stdint.h>

// TivaC device defines and macros
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"

// ------------------------------------------------------------------------------------------------------- //
// Functions definitions
// ------------------------------------------------------------------------------------------------------- //

// Name:        _InitHardware
// Description: Starts device peripherals
// Arguments:   None
// Returns:     None

void Button::_InitHardware()
{
    // Enable peripheral clock
    SysCtlPeripheralEnable(_Config.Hardware.Periph);

    // Power up delay
    SysCtlDelay(10);

    // Unlock used pins (has no effect if pin is not protected by the GPIOCR register
    GPIOUnlockPin(_Config.Hardware.Base, _Config.Hardware.Pin);

    // Configure pins as input
    GPIOPinTypeGPIOInput (_Config.Hardware.Base, _Config.Hardware.Pin);

    // Enable pull-up resistor
    GPIOPadConfigSet (_Config.Hardware.Base, _Config.Hardware.Pin, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        _Pressed
// Description: Checks if the button is pressed
// Arguments:   None
// Returns:     True if button is pressed. False otherwise

bool Button::_Pressed()
{
    return (GPIOPinRead (_Config.Hardware.Base, _Config.Hardware.Pin) == 0);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        _ResetVariables
// Description: Reset the internal Button_Scan_Event function variables
// Arguments:   Pointers to the variables to be reseted
// Returns:     None

void Button::_ResetVariables()
{
    _Scan.TimeCounter = 0;
    _Scan.ShortCounter = 0;
    _Scan.LongCounter = 0;
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        Button
// Description: Constructor of the class with no arguments
// Arguments:   None
// Returns:     None

Button::Button()
{
    _Scan = button_scan_t_default;
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        Button
// Description: Constructor of the class with button_config_t struct as argument
// Arguments:   Config - button_config_t struct
// Returns:     None

Button::Button(button_config_t *Config) : Button()
{
    Init(Config);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        Init
// Description: Starts device peripherals and application state machine
// Arguments:   Config - button_config_t struct
// Returns:     None

void Button::Init(button_config_t *Config)
{
    // Get button_config_t object parameters and store in a "private" variable
    memcpy(&_Config, Config, sizeof(button_config_t));

    //  Initialize hardware
    _InitHardware();
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        ScanEvent
// Description: Scans the button state and detect events
// Arguments:   true if an event was detected. False otherwise
// Returns:     EventData - button_event_data_t struct to receive event data

bool Button::ScanEvent (button_event_data_t *EventData)
{
    // Auxiliary variables
    bool EventFlag = false;

    // Read button status
    bool ButtonPressed = _Pressed();

    // Increase time counter
    _Scan.TimeCounter += _Config.Params.Interval;

    // Button state machine
    switch (_Scan.State)
    {
        // Waiting for button click
        case BUTTON_INIT:
            // Detected click
            if (ButtonPressed)
            {
                // Save state
                _Scan.State = BUTTON_DOWN;

                // Reset variables
                _ResetVariables();
            }
            break;

        // Button is pressed
        case BUTTON_DOWN:
            // Button released after debounce time
            if ((!ButtonPressed) && (_Scan.TimeCounter > _Config.Params.DeadTime))
            {
                // Save state
                _Scan.State = BUTTON_UP;
                _Scan.TimeCounter = 0;
            }

            // Button long pressed
            else if ((ButtonPressed) && (_Scan.TimeCounter > _Config.Params.LongClickTimeout))
            {
                // Increase long click counter
                _Scan.LongCounter++;

                // Configure event data
                EventData->EventCode =  BUTTON_LONG_CLICK_TICK;
                EventData->Counter = _Scan.LongCounter;
                EventFlag = true;

                // Save state
                _Scan.State = BUTTON_HELD;

                // Reset variables
                _Scan.TimeCounter = 0;
            }

            break;

        // Button released
        case BUTTON_UP:
            // Button released after debounce time
            if (_Scan.TimeCounter >= _Config.Params.DeadTime)
            {
                // Increase click counter
                _Scan.ShortCounter++;

                // Save state
                _Scan.State = BUTTON_COUNT;
            }

            break;

        // Debounce time is over, count clicks
        case BUTTON_COUNT:
            // Detected click
            if (ButtonPressed)
            {
                // Save state
                _Scan.State = BUTTON_DOWN;

                // Reset variables
                _Scan.TimeCounter = 0;
            }

            // Detection window is over, handle short click count
            else if (_Scan.TimeCounter > _Config.Params.Window)
            {
                // Configure event data
                EventData->EventCode = BUTTON_SHORT_CLICK;
                EventData->Counter = _Scan.ShortCounter;
                EventFlag = true;

                // Save state
                _Scan.State = BUTTON_INIT;

                // Reset variables
                _ResetVariables();
            }

            break;

        // Button is held down
        case BUTTON_HELD:
            // Button released, handle long click count
            if (!ButtonPressed)
            {
                // Configure event data
                EventData->EventCode = BUTTON_LONG_CLICK;
                EventData->Counter = _Scan.LongCounter;
                EventFlag = true;

                // Save state
                _Scan.State = BUTTON_INIT;

                // Reset variables
                _ResetVariables();
            }

            // Button still long pressed
            else if ((ButtonPressed) && (_Scan.TimeCounter > _Config.Params.LongClickTimeout))
            {
                // Increase long click counter
                _Scan.LongCounter++;

                // Configure event data
                EventData->EventCode =  BUTTON_LONG_CLICK_TICK;
                EventData->Counter = _Scan.LongCounter;
                EventFlag = true;

                // Save state
                _Scan.State = BUTTON_HELD;

                // Reset variables
                _Scan.TimeCounter = 0;
            }

            break;

        // Unknown state - Reset state machine
        default:
            // Save state
            _Scan.State = BUTTON_INIT;

            // Reset variables
            _ResetVariables();

            break;
    }

    return EventFlag;
}

// ------------------------------------------------------------------------------------------------------- //

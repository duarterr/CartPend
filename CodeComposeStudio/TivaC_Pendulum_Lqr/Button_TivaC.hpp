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

#ifndef BUTTON_TIVAC_H_
#define BUTTON_TIVAC_H_

#ifdef __cplusplus
extern "C"
{
#endif

// ------------------------------------------------------------------------------------------------------- //
// Includes
// ------------------------------------------------------------------------------------------------------- //

// Standard libraries
#include <stdint.h>

// ------------------------------------------------------------------------------------------------------- //
// Program enumerations
// ------------------------------------------------------------------------------------------------------- //

// Button states
typedef enum
{
    BUTTON_INIT,                    // Initial state
    BUTTON_DOWN,                    // Button pressed
    BUTTON_UP,                      // Button released
    BUTTON_COUNT,                   // Click count state
    BUTTON_HELD,                    // Button held down
} button_state_t;

// Button events
typedef enum
{
    BUTTON_SHORT_CLICK,             // Button short click (one or more short clicks)
    BUTTON_LONG_CLICK,              // Button long click (one or more long clicks)
    BUTTON_LONG_CLICK_TICK,         // Button long click tick (every long click timeout has passed)
} button_event_code_t;

// ------------------------------------------------------------------------------------------------------- //
// Structs
// ------------------------------------------------------------------------------------------------------- //

// Hardware configuration structure
typedef struct
{
    uint32_t Periph;                // GPIO peripheral
    uint32_t Base;                  // GPIO base
    uint32_t Pin;                   // GPIO pin
} button_hardware_t;

// Button parameters structure
typedef struct
{
    uint8_t Interval;               // Polling interval (ms)
    uint8_t DeadTime;               // Button debouncing dead time (ms)
    uint16_t Window;                // Button click detection window (ms)
    uint16_t LongClickTimeout;      // Button long click timeout (ms)
} button_params_t;

// Button configuration structure
typedef struct
{
    button_hardware_t Hardware;     // Hardware struct
    button_params_t Params;         // Parameters struct
} button_config_t;

// Button event data structure
typedef struct
{
    button_event_code_t EventCode;  // Event code flag - button_event_code_t
    uint8_t Counter;                // Counter (short clicks or long clicks)
} button_event_data_t;

// Button scan variables data structure
typedef struct
{
    uint16_t TimeCounter;           // Time counter (ms)
    uint8_t ShortCounter;           // Short clicks counter
    uint8_t LongCounter;            // Long clicks counter
    button_state_t State;           // Button state
} button_scan_t;

// Button scan variables data structure - Default values
#define button_scan_t_default { \
    .TimeCounter = 0, \
    .ShortCounter = 0, \
    .LongCounter = 0, \
    .State = BUTTON_INIT, \
}

// ------------------------------------------------------------------------------------------------------- //
// Class prototype
// ------------------------------------------------------------------------------------------------------- //

class Button
{
    // --------------------------------------------------------------------------------------------------- //
    // Private members
    // --------------------------------------------------------------------------------------------------- //

    private:

        // Button configuration object
        button_config_t _Config;

        // Button scan variables
        button_scan_t _Scan = button_scan_t_default;

        // Name:        _InitHardware
        // Description: Starts device peripherals
        // Arguments:   None
        // Returns:     None
        void _InitHardware();

        // Name:        _Pressed
        // Description: Checks if the button is pressed
        // Arguments:   None
        // Returns:     True if button is pressed. False otherwise
        bool _Pressed();

        // Name:        _ResetVariables
        // Description: Reset the internal Button_Scan_Event function variables
        // Arguments:   Pointers to the variables to be reseted
        // Returns:     None
        void _ResetVariables();

    // --------------------------------------------------------------------------------------------------- //
    // Public members
    // --------------------------------------------------------------------------------------------------- //

    public:

        // Name:        Button
        // Description: Constructor of the class with no arguments
        // Arguments:   None
        // Returns:     None
        Button();

        // Name:        Button
        // Description: Constructor of the class with button_config_t struct as argument
        // Arguments:   Config - button_config_t struct
        // Returns:     None
        Button(button_config_t *Config);

        // Name:        Init
        // Description: Starts device peripherals and application state machine
        // Arguments:   Config - button_config_t struct
        // Returns:     None
        void Init(button_config_t *Config);

        // Name:        ScanEvent
        // Description: Scans the button state and detect events
        // Arguments:   true if an event was detected. False otherwise
        // Returns:     Event_Data - button_event_data_t struct to receive event data
        bool ScanEvent(button_event_data_t *EventData);
};

// ------------------------------------------------------------------------------------------------------- //

#ifdef __cplusplus
}
#endif

#endif

// ------------------------------------------------------------------------------------------------------- //

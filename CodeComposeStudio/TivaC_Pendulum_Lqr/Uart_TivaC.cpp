// ------------------------------------------------------------------------------------------------------- //

// UART library for TivaC devices
// Version: 1.0
// Author: Renan Duarte
// E-mail: duarte.renan@hotmail.com
// Date:   13/05/2024

// ------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------- //
// Includes
// ------------------------------------------------------------------------------------------------------- //

// Button defines and macros
#include "Uart_TivaC.hpp"

// Standard libraries
#include <stdint.h>

// TivaC device defines and macros
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

// ------------------------------------------------------------------------------------------------------- //
// Functions definitions
// ------------------------------------------------------------------------------------------------------- //

// Name:        _InitHardware
// Description: Starts device peripherals
// Arguments:   None
// Returns:     None

void Uart::_InitHardware()
{
    // Enable peripheral clocks
    SysCtlPeripheralEnable(_Config.Hardware.PeriphUART);
    SysCtlPeripheralEnable(_Config.Hardware.PeriphGPIO);

    // Unlock used pins (has no effect if pin is not protected by the GPIOCR register
    GPIOUnlockPin(_Config.Hardware.BaseGPIO, _Config.Hardware.PinRX | _Config.Hardware.PinTX);

    // Configure GPIO Pins for UART mode
    GPIOPinConfigure(_Config.Hardware.PinMuxRX);
    GPIOPinConfigure(_Config.Hardware.PinMuxTX);
    GPIOPinTypeUART(_Config.Hardware.BaseGPIO, _Config.Hardware.PinRX | _Config.Hardware.PinTX);

    // Configure UART
    UARTConfigSetExpClk(_Config.Hardware.BaseUART, SysCtlClockGet(), _Config.Params.BaudRate, _Config.Hardware.Config);

    // Register interrupt handler
    UARTIntRegister(_Config.Hardware.BaseUART, _Config.Hardware.Callback);

    // Enable interrupt on RX and RX timeout
    UARTIntEnable(_Config.Hardware.BaseUART, UART_INT_RX | UART_INT_RT);

    // Enable interrupt
    IntEnable(_Config.Hardware.Interrupt);

    // Enable the UART module
    UARTEnable(_Config.Hardware.BaseUART);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        _BufferPutByte
// Description: Puts a byte in the UART buffer
// Arguments:   Byte - Byte to be put
// Returns:     None

void Uart::_BufferPutByte (uint8_t Byte)
{
    // Put char in FIFO buffer - Wait until there is space
    UARTCharPut(_Config.Hardware.BaseUART, Byte);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        Uart
// Description: Constructor of the class with no arguments
// Arguments:   None
// Returns:     None

Uart::Uart()
{

}

// ------------------------------------------------------------------------------------------------------- //

// Name:        Uart
// Description: Constructor of the class with uart_config_t struct as argument
// Arguments:   Config - uart_config_t struct
// Returns:     None

Uart::Uart(uart_config_t *Config) : Uart()
{
    Init(Config);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        Init
// Description: Starts device peripherals and variables
// Arguments:   Config - uart_config_t struct
// Returns:     None

void Uart::Init(uart_config_t *Config)
{
    // Get uart_config_t object parameters and store in a private variable
    memcpy(&_Config, Config, sizeof(uart_config_t));

    //  Initialize hardware
    _InitHardware();
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        SendString
// Description: Sends a string to the UART
// Arguments:   String - Pointer to the string to be sent
// Returns:     None

void Uart::SendString (const char *String)
{
    // Send all chars of the string
    while (*String)
        _BufferPutByte(*String++);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        ReceiveIsr
// Description: UART RX interrupt service routine
// Arguments:   None
// Returns:     None

void Uart::ReceiveIsr ()
{
    // Get the interrupt status
    uint32_t Status = UARTIntStatus(_Config.Hardware.BaseUART, true);

    // Clear the asserted interrupts.
    UARTIntClear(_Config.Hardware.BaseUART, Status);

    // Loop while there are characters in the receive FIFO
    while(UARTCharsAvail(_Config.Hardware.BaseUART))
    {
        // Read the next character from the UART
        char Rx = UARTCharGetNonBlocking(_Config.Hardware.BaseUART);
    }
}

// ------------------------------------------------------------------------------------------------------- //

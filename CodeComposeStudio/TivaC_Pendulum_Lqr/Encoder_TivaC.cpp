// ------------------------------------------------------------------------------------------------------- //

// Encoder library for TivaC devices
// Version: 1.0
// Author: Renan Duarte
// E-mail: duarte.renan@hotmail.com
// Date:   14/05/2024

// ------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------- //
// Includes
// ------------------------------------------------------------------------------------------------------- //

// Button defines and macros
#include "Encoder_TivaC.hpp"

// Standard libraries
#include <stdint.h>

// TivaC device defines and macros
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/qei.h"

// ------------------------------------------------------------------------------------------------------- //
// Functions definitions
// ------------------------------------------------------------------------------------------------------- //

// Name:        _InitHardware
// Description: Starts device peripherals
// Arguments:   None
// Returns:     None

void Encoder::_InitHardware()
{
    // Enable peripheral clocks
    SysCtlPeripheralEnable (_Config.Hardware.PeriphQEI);
    SysCtlPeripheralEnable (_Config.Hardware.PeriphGPIO);

    // Power up delay
    SysCtlDelay (10);

    // Unlock used pins (has no effect if pin is not protected by the GPIOCR register
    GPIOUnlockPin(_Config.Hardware.BaseGPIO, _Config.Hardware.PinA | _Config.Hardware.PinB);

    // Configure pins as A and B inputs
    GPIOPinTypeQEI(_Config.Hardware.BaseGPIO, _Config.Hardware.PinA | _Config.Hardware.PinB);
    GPIOPinConfigure(_Config.Hardware.PinMuxA);
    GPIOPinConfigure(_Config.Hardware.PinMuxB);

    // Configure the QEI with defined config and PPR
    QEIConfigure(_Config.Hardware.BaseQEI, _Config.Hardware.Config, _Config.Params.PPR);

    // Configure velocity calculation
    QEIVelocityConfigure(_Config.Hardware.BaseQEI, QEI_VELDIV_1, SysCtlClockGet() / _Config.Params.ScanFreq);
    QEIVelocityEnable(_Config.Hardware.BaseQEI);

    // Register interrupt handler for velocity timer expiration
    QEIIntRegister(_Config.Hardware.BaseQEI, _Config.Hardware.Callback);

    // Enable QEI interrupt
    QEIIntEnable(_Config.Hardware.BaseQEI, _Config.Hardware.Interrupt);

    // Enable the QEI
    QEIEnable(_Config.Hardware.BaseQEI);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        Encoder
// Description: Constructor of the class with no arguments
// Arguments:   None
// Returns:     None

Encoder::Encoder()
{

}

// ------------------------------------------------------------------------------------------------------- //

// Name:        Encoder
// Description: Constructor of the class with encoder_config_t struct as argument
// Arguments:   Config - encoder_config_t struct
// Returns:     None

Encoder::Encoder(encoder_config_t *Config) : Encoder()
{
    Init(Config);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        Init
// Description: Starts device peripherals and variables
// Arguments:   Config - encoder_config_t struct
// Returns:     None

void Encoder::Init(encoder_config_t *Config)
{
    // Get encoder_config_t object parameters and store in a private variable
    memcpy(&_Config, Config, sizeof(encoder_config_t));

    //  Initialize hardware
    _InitHardware();
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        GetData
// Description: Gets all encoder data
// Arguments:   Encoder - encoder_data_t struct to receive data
// Returns:     None

void Encoder::GetData (encoder_data_t *Data)
{
    Data->Pos = _Data.Pos;
    Data->Vel = _Data.Vel;
    Data->Dir = _Data.Dir;
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        GetPos
// Description: Gets encoder position
// Arguments:   None
// Returns:     Encoder position read in last scan

uint32_t Encoder::GetPos ()
{
     return _Data.Pos;
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        SetPos
// Description: Sets encoder position
// Arguments:   New encoder position
// Returns:     None

void Encoder::SetPos (uint32_t Pos)
{
    // Set the position reading of the encoder
     QEIPositionSet(_Config.Hardware.BaseQEI, Pos);
     _Data.Pos =  Pos;
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        GetVel
// Description: Gets encoder velocity
// Arguments:   None
// Returns:     Encoder velocity read in last scan

uint32_t Encoder::GetVel ()
{
    return _Data.Vel;
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        GetDir
// Description: Gets encoder direction
// Arguments:   None
// Returns:     Encoder direction read in last scan

int32_t Encoder::GetDir ()
{
    return _Data.Dir;
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        TimerIsr
// Description: Velocity timer interrupt service routine
// Arguments:   None
// Returns:     None

void Encoder::TimerIsr ()
{
    // Clear the interrupt that is generated
    QEIIntClear(_Config.Hardware.BaseQEI, QEIIntStatus(_Config.Hardware.BaseQEI, true));

    // Get the position reading of the encoder
    _Data.Pos = QEIPositionGet(_Config.Hardware.BaseQEI);

    // Get the number of quadrature ticks since last call
    _Data.Vel = QEIVelocityGet(_Config.Hardware.BaseQEI);

    // Get the direction reading of the encoder
    _Data.Dir = QEIDirectionGet(_Config.Hardware.BaseQEI);
}

// ------------------------------------------------------------------------------------------------------- //


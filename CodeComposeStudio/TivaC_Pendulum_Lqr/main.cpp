// ------------------------------------------------------------------------------------------------------- //
// Pendulum on a cart
// ------------------------------------------------------------------------------------------------------- //

// Version: 1.0
// Author: Renan Duarte
// E-mail: duarte.renan@hotmail.com
// Date:   13/05/2024
// Device: CORTEX M4F TM4C123GH6PM Tiva C Launchpad

// ------------------------------------------------------------------------------------------------------- //
// Includes
// ------------------------------------------------------------------------------------------------------- //

// Application defines
#include "main.hpp"

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

// Auxiliary functions
#include "Aux_Functions.hpp"

// ------------------------------------------------------------------------------------------------------- //
// Functions Prototypes
// ------------------------------------------------------------------------------------------------------- //

// Start stepper PWM generator
void _StepperStartPwm (void);

// Stop stepper PWM generator
void _StepperStopPwm (void);

// Set stepper PWM frequency and duty
void _StepperChangePWM (int32_t NewPPS);


// Move stepper with speed ramp - Constant acceleration
// Direction is determined based on current status and acceleration value
void StepperMoveAccel (uint32_t FinalVelocity, float Acceleration);

// Start stepper at constant speed - No velocity control
void StepperMoveNoAccel (bool Direction, uint32_t Velocity);

// Stop stepper
void StepperStop (void);

// Stepper velocity control
void StepperUpdateMovement (void);

// Stepper stall detection
void StepperDetectStall (void);

// Check if movement is possible in a given direction
bool StepperCanMove(bool Direction);



// Calculate LQR control action value and stores in Lqr struct
void LqrCompute (void);




// Send device info via UART
void DeviceUpdateUart(void);

// LCD update
void DeviceUpdateLcd(void);

// RGB LED update
void DeviceUpdateRgb(void);

// Button 1 event handler
void DeviceUpdateButton1 (button_event_data_t *EventData);

// Button 2 event handler
void DeviceUpdateButton2 (button_event_data_t *EventData);

// SysTick interrupt service routine
void IsrSysTick (void);

// ------------------------------------------------------------------------------------------------------- //
// Variables
// ------------------------------------------------------------------------------------------------------- //

// Display object - From Lcd_TivaC class
Lcd Display;

// Button objects - From Button_TivaC class
Button Button1, Button2;

// UART object - From Uart_TivaC class
Uart Serial;

// Encoder objects - From Encoder_TivaC class
Encoder EncoderT, EncoderX;

// RGB LED object - From Rgb_TivaC class
Rgb Led;


// CPU clock - Hz
uint32_t cpuClock = 0;

// Systick interrupt counter
volatile uint32_t SysTickCounter = 0;

// Cart-related variables
volatile cart_t Cart = cart_t_default;

// Pendulum-related variables
volatile pendulum_t Pendulum = pendulum_t_default;

// Stepper-related variables
volatile stepper_t Stepper = stepper_t_default;

// Calibration variables
volatile calibration_t CalT = {.Status = CAL_PENDING, .Progress = 0, .Offset = 0, .Max = ENCODER_T_PPR};
volatile calibration_t CalX = {.Status = CAL_PENDING, .Progress = 0, .Offset = 1000, .Max = ENCODER_X_PPR};

// LQR
lqr_t Lqr = lqr_t_default;

uint32_t PPS = 50000;
float Accel = 0.1;

// ------------------------------------------------------------------------------------------------------- //
// Calculate LQR control action value and stores in Lqr struct
// ------------------------------------------------------------------------------------------------------- //

void LqrCompute (void)
{
    char Idx = 0;

    // Define current state vector
    Lqr.States[0] = Cart.Pos;
    Lqr.States[1] = Cart.Vel;
    Lqr.States[2] = Pendulum.Pos;
    Lqr.States[3] = Pendulum.Vel;
    Lqr.States[4] = Lqr.Ref[0] - Cart.Pos;

    // Calculate state error
    for (Idx = 0; Idx < 5; Idx++)
        Lqr.E_now[Idx] = Lqr.Ref[Idx] - Lqr.States[Idx];

    // Calculate U_nxt
    Lqr.U_nxt = 0;

    for (Idx = 0; Idx < 5; Idx++)
        Lqr.U_nxt += Lqr.K[Idx]*Lqr.E_now[Idx];
}

/* -------------------------------------------------------------------------------------------------------------------- */
// Set stepper direction
/* -------------------------------------------------------------------------------------------------------------------- */

void StepperSetDirection (bool NewDirection)
{
    Stepper.Dir = NewDirection;
    GPIOPinWrite (STEPPER_GPIO_BASE, STEPPER_PIN_DIR, NewDirection * 0xFF);
}

/* -------------------------------------------------------------------------------------------------------------------- */
// Set stepper enable/disable status
/* -------------------------------------------------------------------------------------------------------------------- */

void StepperSetEnable (bool NewEnable)
{
    Stepper.Enabled = NewEnable;
    GPIOPinWrite (STEPPER_GPIO_BASE, STEPPER_PIN_EN, !NewEnable * 0xFF);
}

// ------------------------------------------------------------------------------------------------------- //
// Check if movement is possible in a given direction
// ------------------------------------------------------------------------------------------------------- //

bool StepperCanMove(bool Direction)
{
    // One of the limit switches is pressed and stepper is trying to move in the same direction
    return !((GPIOPinRead(SWITCH_GPIO_BASE, SWITCH_PIN_START) && (Direction == 0))
            || (GPIOPinRead(SWITCH_GPIO_BASE, SWITCH_PIN_END) && (Direction == 1)));
}

// ------------------------------------------------------------------------------------------------------- //
// Stepper stall detection
// ------------------------------------------------------------------------------------------------------- //

void StepperDetectStall (void)
{
    if (Stepper.Enabled)
    {
        static uint32_t LastEncoderCounter = 0;

        // Cart moved too little and stepper has PWM
        if ((LastEncoderCounter == EncoderX.GetPos()) && (Stepper.CurrentPPS == Stepper.TargetPPS))
            StepperStop();

        LastEncoderCounter = EncoderX.GetPos();
    }
}

// ------------------------------------------------------------------------------------------------------- //
// Stepper stall detection
// ------------------------------------------------------------------------------------------------------- //

void _StepperChangePWM (void)
{
    // Desired PWM frequency is high enough for high speed PWM clock
    if ((Stepper.CurrentPPS > 3000) && (Stepper.PwmClock != cpuClock))
    {
        // Disable PWM clock division
        SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
        Stepper.PwmClock = cpuClock;
    }

    // Desired PWM frequency is too low for current PWM clock
    else if ((Stepper.CurrentPPS < 2000) && (Stepper.PwmClock == cpuClock))
    {
        // Reduce PWM clock
        SysCtlPWMClockSet(STEPPER_PWM_DIV_CFG);
        Stepper.PwmClock = cpuClock / STEPPER_PWM_DIV;
    }

    // Define PWM period (expressed in clock ticks)
    Stepper.PwmPeriod = (Stepper.PwmClock/Stepper.CurrentPPS) - 1;

    // Change PWM frequency and duty cycle (only if necessary)
    if (Stepper.PwmPeriod != PWMGenPeriodGet (STEPPER_PWM_BASE, STEPPER_PWM_GEN))
    {
        PWMGenPeriodSet (STEPPER_PWM_BASE, STEPPER_PWM_GEN, Stepper.PwmPeriod);
        PWMPulseWidthSet (STEPPER_PWM_BASE, STEPPER_PWM_OUT, Stepper.PwmPeriod >> 1);
    }
}

// ------------------------------------------------------------------------------------------------------- //
// Stepper velocity control
// ------------------------------------------------------------------------------------------------------- //

void StepperUpdateMovement (void)
{
    int32_t NewPPS = 0;

    // Stepper is disabled - Shouldn't enter here
    if (!Stepper.Enabled)
    {
        // Stop movement
        StepperStop();
        return;
    }

    // Acceleration is on
    if (Stepper.AccEnabled)
    {
        // Negative acceleration
        if (Stepper.DeltaPPS < 0)
        {
            // Is going forwards
            if (Stepper.Dir == 1)
            {
                // Decrease PPS
                NewPPS = Stepper.CurrentPPS + Stepper.DeltaPPS;

                // Reached minimum speed
                if (Stepper.CurrentPPS <= STEPPER_PPS_MIN)
                    // Invert direction
                    StepperSetDirection(!Stepper.Dir);

                // Reset speed to avoid going bellow minimum
                if (NewPPS < STEPPER_PPS_MIN)
                    NewPPS = STEPPER_PPS_MIN;
            }

            // Is going backwards
            else if (Stepper.Dir == 0)
            {
                // Increase PPS
                NewPPS = Stepper.CurrentPPS - Stepper.DeltaPPS;

                // Reached target speed
                if (NewPPS > Stepper.TargetPPS)
                    NewPPS = Stepper.TargetPPS;
            }
        }

        // Positive acceleration
        else if (Stepper.DeltaPPS > 0)
        {
            // Is going forwards
            if (Stepper.Dir == 1)
            {
                // Increase PPS
                NewPPS = Stepper.CurrentPPS + Stepper.DeltaPPS;

                // Reached target speed
                if (NewPPS > Stepper.TargetPPS)
                    NewPPS = Stepper.TargetPPS;
            }

            // Is going backwards
            else if (Stepper.Dir == 0)
            {
                // Decrease PPS
                NewPPS = Stepper.CurrentPPS - Stepper.DeltaPPS;

                // Reached minimum speed
                if (Stepper.CurrentPPS <= STEPPER_PPS_MIN)
                    // Invert direction
                    StepperSetDirection(!Stepper.Dir);

                // Reset speed to avoid going bellow minimum
                if (NewPPS < STEPPER_PPS_MIN)
                    NewPPS = STEPPER_PPS_MIN;
            }
        }

        // No acceleration
        else
            NewPPS = Stepper.CurrentPPS;
    }

    // Acceleration is off
    else if (NewPPS != Stepper.TargetPPS)
        NewPPS = Stepper.TargetPPS;

    // Cart is close to end limits - Reduce velocity
    if (((Cart.Pos > 0.19) && (Stepper.Dir == 1)) || ((Cart.Pos < -0.19) && (Stepper.Dir == 0)))
    {
        // Limit speed
        NewPPS = STEPPER_PPS_CAL;
    }

    // New velocity is non zero
    if (NewPPS != 0)
    {
        // PWM frequency needs to be changed
        Stepper.CurrentPPS = NewPPS;

        _StepperChangePWM ();
    }
}

// ------------------------------------------------------------------------------------------------------- //
// Move stepper with speed ramp - Constant acceleration
// Direction is determined based on current status and acceleration value
// ------------------------------------------------------------------------------------------------------- //

void StepperMoveAccel (uint32_t FinalVelocity, float Acceleration)
{
    // Get current direction
    bool Direction = Stepper.Dir;

    // Stepper is stopped
    if (!Stepper.Enabled)
    {
        // Start going forward
        if (Acceleration > 0)
            Direction = 1;

        // Start going backward
        else if (Acceleration < 0)
            Direction = 0;

        // Define initial speed
        Stepper.CurrentPPS = STEPPER_PPS_MIN;

        _StepperChangePWM ();
    }

    // Cannot move in desired direction
    if (!StepperCanMove(Direction))
    {
        StepperStop();
        return;
    }

    // Velocity saturation
    if ((CalX.Status != CAL_DONE) && (FinalVelocity > STEPPER_PPS_CAL))
        FinalVelocity = STEPPER_PPS_CAL;
    else if (FinalVelocity > STEPPER_PPS_MAX)
        FinalVelocity = STEPPER_PPS_MAX;
    else if(FinalVelocity < STEPPER_PPS_MIN)
        FinalVelocity = STEPPER_PPS_MIN;

    // Acceleration saturation
    if (Acceleration > STEPPER_ACC_MAX)
        Acceleration = STEPPER_ACC_MAX;
    else if (Acceleration < STEPPER_ACC_MIN)
        Acceleration = STEPPER_ACC_MIN;

    // Set direction and enable pins
    StepperSetDirection(Direction);
    StepperSetEnable(true);

    // Set velocity and acceleration
    Stepper.TargetPPS = FinalVelocity;
    Stepper.AccValue = Acceleration;
    Stepper.AccEnabled = Acceleration == 0 ? 0 : 1;
    Stepper.DeltaPPS = Stepper.AccValue*STEPPER_KV_PPS/TIMER_FREQUENCY;

    // TODO: Is there a better way?
    if (Stepper.DeltaPPS == 0)
        return;

    // Start PWM
    _StepperStartPwm ();
}

// ------------------------------------------------------------------------------------------------------- //
// Start stepper at constant speed - No velocity control
// ------------------------------------------------------------------------------------------------------- //

void StepperMoveNoAccel (bool Direction, uint32_t Velocity)
{
    // Cannot move in desired direction
    if (!StepperCanMove(Direction))
    {
        StepperStop();
        return;
    }

    // Velocity saturation
    if (Velocity == 0)
    {
        // No need to go beyond
        StepperStop();
        return;
    }
    else if (Velocity > STEPPER_PPS_MAX)
        Velocity = STEPPER_PPS_MAX;
    else if(Velocity < STEPPER_PPS_MIN)
        Velocity = STEPPER_PPS_MIN;

    // Direction change with motor enabled. Stop it first
    if (Stepper.Enabled && (Direction != Stepper.Dir))
        StepperStop ();

    // Set direction and enable pins
    StepperSetDirection(Direction);
    StepperSetEnable(true);

    // Set velocity and acceleration
    Stepper.TargetPPS = Velocity;
    Stepper.AccValue = 0;
    Stepper.AccEnabled = 0;

    // Start PWM
    _StepperStartPwm ();
}

/* -------------------------------------------------------------------------------------------------------------------- */
// Start stepper PWM generator
/* -------------------------------------------------------------------------------------------------------------------- */

void _StepperStartPwm (void)
{
    // Turn on the output pins
    PWMOutputState (STEPPER_PWM_BASE, STEPPER_PWM_OUT_BIT , true);

    // Start PWM
    PWMGenEnable (STEPPER_PWM_BASE, STEPPER_PWM_GEN);
}

/* -------------------------------------------------------------------------------------------------------------------- */
// Stop stepper PWM generator
/* -------------------------------------------------------------------------------------------------------------------- */

void _StepperStopPwm (void)
{
    // Stop PWM
    PWMGenDisable (STEPPER_PWM_BASE, STEPPER_PWM_GEN);

    // Clear interrupt flag
    PWMGenIntClear(STEPPER_PWM_BASE, STEPPER_PWM_GEN, PWM_INT_CNT_ZERO);

    // Turn off the output pins
    PWMOutputState (STEPPER_PWM_BASE, STEPPER_PWM_OUT_BIT , false);
}

// ------------------------------------------------------------------------------------------------------- //
// Stop stepper
// ------------------------------------------------------------------------------------------------------- //

void StepperStop (void)
{
    // Reset enable pin
    StepperSetEnable(false);

    // Stop PWM
    _StepperStopPwm ();

    // Reset velocity and acceleration
    Stepper.CurrentPPS = STEPPER_PPS_MIN;
    Stepper.AccValue = 0;
    Stepper.DeltaPPS = 0;
}

// ------------------------------------------------------------------------------------------------------- //
// Stepper PWM zero count interrupt service routine
// ------------------------------------------------------------------------------------------------------- //

void IsrPwmZero(void)
{
    // Cannot move anymore in current direction
    if (!StepperCanMove(Stepper.Dir))
    {
        StepperStop();
        return;
    }

    // Clear interrupt flag
    PWMGenIntClear(STEPPER_PWM_BASE, STEPPER_PWM_GEN, PWM_INT_CNT_ZERO);
}

// ------------------------------------------------------------------------------------------------------- //
// Update cart state based on encoder readings
// ------------------------------------------------------------------------------------------------------- //

void CartUpdateState (void)
{
    // Calculate cart absolute position - Remove 1000 added during calibration
    Cart.Pos = ((float)X_VALUE_TOTAL_M/CalX.Max)*((int32_t)EncoderX.GetPos() - CalX.Offset) - X_VALUE_ABS_M;

    // Calculate cart velocity
    Cart.Vel = (((X_VALUE_TOTAL_M * EncoderX.GetVel()) / CalX.Max) * ENCODER_X_FREQUENCY) * EncoderX.GetDir();
}

// ------------------------------------------------------------------------------------------------------- //
// Calibrate cart X position limits
// ------------------------------------------------------------------------------------------------------- //

void CartCalibratePos (void)
{
    CalX.Progress = 0;
    CalX.Status = CAL_RUNNING;

    // Start stepper
    //StepperMoveAccel (STEPPER_PPS_CAL, -0.1);
    StepperMoveNoAccel (0, STEPPER_PPS_CAL);

    // Move until home switch triggers
    while (Stepper.Enabled);

    // Set encoder counter - Added offset because cart can move past limit switch trigger point
    EncoderX.SetPos(CalX.Offset);

    // Start stepper
    //StepperMoveAccel (STEPPER_PPS_CAL, 0.1);
    StepperMoveNoAccel (1, STEPPER_PPS_CAL);

    // Move until end switch triggers
    while (Stepper.Enabled)
        CalX.Progress = (uint32_t)(EncoderX.GetPos() * 200)/ENCODER_X_PPR;

    // Define encoder maximum counter value - Without offset
    CalX.Max = EncoderX.GetPos() - CalX.Offset;

    CalX.Status = CAL_DONE;
}

// ------------------------------------------------------------------------------------------------------- //
// Update pendulum state based on encoder readings
// ------------------------------------------------------------------------------------------------------- //

void PendulumUpdateState (void)
{
    // Calculate pendulum absolute position
    Pendulum.Pos = ((float)T_VALUE_MAX_RAD/CalT.Max)*((int32_t)EncoderT.GetPos() - CalT.Offset) - T_VALUE_ABS_MAX;

    // Calculate pendulum velocity
    Pendulum.Vel = (((T_VALUE_MAX_RAD * EncoderT.GetVel()) / CalT.Max) * ENCODER_T_FREQUENCY) * EncoderT.GetDir();
}

// ------------------------------------------------------------------------------------------------------- //
// Calibrate pendulum angle
// ------------------------------------------------------------------------------------------------------- //

void PendulumCalibrateAngle (void)
{
    CalT.Progress = 0;
    CalT.Status = CAL_RUNNING;

    uint32_t PositionLast = 5000;
    uint32_t Counter = 0;

    // Wait for pendulum to stabilise in the downward position
    while (Counter < 100)
    {
        if (EncoderT.GetVel() != 0)
            Counter = 0;
        else if (PositionLast == EncoderT.GetPos())
            Counter++;
        else
            Counter = 0;

        CalT.Progress = Counter;

        PositionLast = EncoderT.GetPos();
        SysCtlDelay(100000);
    }

    // Set encoder counter
    EncoderT.SetPos(0);

    // Define encoder maximum counter value
    CalT.Max = ENCODER_T_PPR;

    CalT.Status = CAL_DONE;
}

// ------------------------------------------------------------------------------------------------------- //
// Send device info via UART
// ------------------------------------------------------------------------------------------------------- //

void DeviceUpdateUart(void)
{
    // Send data only if axis are calibrated
    if ((CalX.Status == CAL_DONE) && (CalT.Status == CAL_DONE))
    {
        char Aux_String [50];

        ltoa (SysTickCounter, Aux_String, 10);
        Serial.SendString (Aux_String);
        Serial.SendString (";");

        Aux::F2Str(Stepper.AccValue, Aux_String, 6);
        Serial.SendString (Aux_String);
        Serial.SendString (";");

        if (Stepper.Dir == 0)
            ltoa (-1.0 * Stepper.CurrentPPS * Stepper.Enabled, Aux_String, 10);
        else
            ltoa (Stepper.CurrentPPS * Stepper.Enabled, Aux_String, 10);

        Serial.SendString (Aux_String);
        Serial.SendString (";");

        Aux::F2Str (Cart.Pos, Aux_String, 6);
        Serial.SendString (Aux_String);
        Serial.SendString (";");

        Aux::F2Str (Cart.Vel, Aux_String, 6);
        Serial.SendString (Aux_String);
        Serial.SendString (";");

        Aux::F2Str (Pendulum.Pos, Aux_String, 6);
        Serial.SendString (Aux_String);
        Serial.SendString (";");

        Aux::F2Str (Pendulum.Vel, Aux_String, 6);
        Serial.SendString (Aux_String);
        Serial.SendString ("\n");
    }
}

// ------------------------------------------------------------------------------------------------------- //
// LCD update
// ------------------------------------------------------------------------------------------------------- //

void DeviceUpdateLcd(void)
{
    Display.ClearAll();
    Display.DrawFilledRectangle (0, 0, 83, 8, LCD_PIXEL_ON);


    // Calibration mode
    if ((CalX.Status == CAL_RUNNING) || (CalT.Status == CAL_RUNNING))
    {
        Display.DrawRectangle (0, 8, 83, 16, LCD_PIXEL_ON);
        Display.DrawRectangle (0, 38, 83, 47, LCD_PIXEL_ON);

        Display.Goto(0, 9);
        Display.WriteString("CALIBRATION", LCD_FONT_SMALL, LCD_PIXEL_XOR);

        if (CalX.Status == CAL_RUNNING)
        {
            Display.Goto(1, 24);
            Display.WriteString("X-axis", LCD_FONT_SMALL, LCD_PIXEL_XOR);

            if (CalX.Progress != 0)
            {
                Display.Goto(3, 0);
                Display.WriteString("Calibrating...", LCD_FONT_SMALL, LCD_PIXEL_ON);

                Display.DrawFilledRectangle(0, 38, (CalX.Progress * PCD8544_COLUMNS / 100), 47, LCD_PIXEL_ON);
            }

            else
            {
                Display.Goto(3, 15);
                Display.WriteString("Homing...", LCD_FONT_SMALL, LCD_PIXEL_ON);
            }
        }

        else if (CalT.Status == CAL_RUNNING)
        {
            Display.Goto(1, 12);
            Display.WriteString("Theta-axis", LCD_FONT_SMALL, LCD_PIXEL_XOR);

            Display.Goto(3, 0);
            Display.WriteString("Calibrating...", LCD_FONT_SMALL, LCD_PIXEL_ON);

            if (CalT.Progress != 0)
                Display.DrawFilledRectangle(0, 38, (CalT.Progress * PCD8544_COLUMNS / 100), 47, LCD_PIXEL_ON);
        }
    }

    // Run mode
    else
    {
        Display.Goto(0, 18);
        Display.WriteString("LQR MODE", LCD_FONT_SMALL, LCD_PIXEL_XOR);

        float NumberToWrite = 0;

        Display.Goto(2, 0);
        Display.WriteString("xR:", LCD_FONT_SMALL, LCD_PIXEL_ON);
        NumberToWrite = Lqr.Ref[0];
        if (NumberToWrite >= 0)
            Display.WriteChar(' ', LCD_FONT_SMALL, LCD_PIXEL_ON);
        Display.WriteFloat(NumberToWrite, 6, LCD_FONT_SMALL, LCD_PIXEL_ON);
        Display.Goto(Display.GetBank(), PCD8544_COLUMNS - 6);
        Display.WriteString("m", LCD_FONT_SMALL, LCD_PIXEL_ON);

        Display.Goto(3, 0);
        Display.WriteString("xP:", LCD_FONT_SMALL, LCD_PIXEL_ON);
        NumberToWrite = Cart.Pos;
        if (NumberToWrite >= 0)
            Display.WriteChar(' ', LCD_FONT_SMALL, LCD_PIXEL_ON);
        Display.WriteFloat(NumberToWrite, 6, LCD_FONT_SMALL, LCD_PIXEL_ON);
        Display.Goto(Display.GetBank(), PCD8544_COLUMNS - 6);
        Display.WriteString("m", LCD_FONT_SMALL, LCD_PIXEL_ON);

        Display.Goto(4, 0);
        Display.WriteString("xV:", LCD_FONT_SMALL, LCD_PIXEL_ON);
        NumberToWrite = Cart.Vel;
        if (NumberToWrite >= 0)
            Display.WriteChar(' ', LCD_FONT_SMALL, LCD_PIXEL_ON);
        Display.WriteFloat(NumberToWrite, 4, LCD_FONT_SMALL, LCD_PIXEL_ON);
        Display.Goto(Display.GetBank(), PCD8544_COLUMNS - 18);
        Display.WriteString("m/s", LCD_FONT_SMALL, LCD_PIXEL_ON);

        Display.Goto(5, 0);
        Display.WriteString("tP:", LCD_FONT_SMALL, LCD_PIXEL_ON);
        NumberToWrite = Pendulum.Pos;
        if (NumberToWrite >= 0)
            Display.WriteChar(' ', LCD_FONT_SMALL, LCD_PIXEL_ON);
        Display.WriteFloat(NumberToWrite, 4, LCD_FONT_SMALL, LCD_PIXEL_ON);
        Display.Goto(Display.GetBank(), PCD8544_COLUMNS - 18);
        Display.WriteString("rad", LCD_FONT_SMALL, LCD_PIXEL_ON);

        // Rail
        Display.DrawLine(0, 13, 83, 13, LCD_PIXEL_ON);
        Display.DrawLine(0, 11, 0, 15, LCD_PIXEL_ON);
        Display.DrawLine(83, 11, 83, 15, LCD_PIXEL_ON);

        // Cart
        uint8_t xP = (uint8_t)Aux::Map (Cart.Pos, -X_VALUE_ABS_M, X_VALUE_ABS_M, 0, (PCD8544_COLUMNS - 1));
        Display.DrawFilledRectangle(xP - 2, 11, xP + 2, 15, LCD_PIXEL_ON);

        // Shaddow reference mark
        xP = (uint8_t)Aux::Map (Lqr.Ref_Shaddow, -X_VALUE_ABS_M, X_VALUE_ABS_M, 0, (PCD8544_COLUMNS - 1));
        Display.DrawPixel(xP, 12, LCD_PIXEL_XOR);
        Display.DrawPixel(xP, 13, LCD_PIXEL_OFF);

        // Reference mark
        xP = (uint8_t)Aux::Map (Lqr.Ref[0], -X_VALUE_ABS_M, X_VALUE_ABS_M, 0, (PCD8544_COLUMNS - 1));
        Display.DrawPixel(xP, 14, LCD_PIXEL_XOR);
        Display.DrawPixel(xP, 13, LCD_PIXEL_OFF);
    }

    // Commit changes
    Display.Commit();
}

// ------------------------------------------------------------------------------------------------------- //
// RGB update
// ------------------------------------------------------------------------------------------------------- //

void DeviceUpdateRgb(void)
{
    // Calibration mode
    if ((CalX.Status == CAL_RUNNING) || (CalT.Status == CAL_RUNNING))
        Led.SetColor(RGB_RED, 0);

    // Run mode
    else
    {
        if (fabs(Lqr.E_now[0]) < 0.001)
            Led.SetColor(RGB_GREEN, 0);
        else
            Led.SetColor(RGB_YELLOW, 0);
    }
}

// ------------------------------------------------------------------------------------------------------- //
// Button 1 event handler
// ------------------------------------------------------------------------------------------------------- //

void DeviceUpdateButton1 (button_event_data_t *EventData)
{
     // Button short clicks
     if (EventData->EventCode == BUTTON_SHORT_CLICK)
     {
         // Single click
         if (EventData->Counter == 1)
         {
             Lqr.Ref_Shaddow -= 0.025;

             if (Lqr.Ref_Shaddow < -0.15)
                 Lqr.Ref_Shaddow = -0.15;
         }

         // Double click
         else if (EventData->Counter == 2)
             Lqr.Ref[0] = Lqr.Ref_Shaddow;
     }
}

// ------------------------------------------------------------------------------------------------------- //
// Button 2 event handler
// ------------------------------------------------------------------------------------------------------- //

void DeviceUpdateButton2 (button_event_data_t *EventData)
{
     // Button short clicks
     if (EventData->EventCode == BUTTON_SHORT_CLICK)
     {
         // Single click
         if (EventData->Counter == 1)
         {
             Lqr.Ref_Shaddow += 0.025;

             if (Lqr.Ref_Shaddow > 0.15)
                 Lqr.Ref_Shaddow = 0.15;
         }

         // Double click
         else if (EventData->Counter == 2)
             Lqr.Ref[0] = Lqr.Ref_Shaddow;
     }
}

// ------------------------------------------------------------------------------------------------------- //
// SysTick interrupt service routine
// ------------------------------------------------------------------------------------------------------- //

void IsrSysTick (void)
{
    // Pendulum state update
    if ((SysTickCounter % (TIMER_FREQUENCY / ENCODER_T_FREQUENCY)) == 0)
        PendulumUpdateState ();

    // Cart state update
    if ((SysTickCounter % (TIMER_FREQUENCY / ENCODER_X_FREQUENCY)) == 0)
    {
        CartUpdateState ();

        // Check for stalls - Done here because we need updated cart data
        StepperDetectStall ();
    }

    // Control loop
    if ((CalX.Status == CAL_DONE) && (CalT.Status == CAL_DONE) &&  ((SysTickCounter % (TIMER_FREQUENCY / CONTROL_LOOP_FREQUENCY)) == 0))
    {
        // Code for control loops
        if (fabs(Pendulum.Pos) < 0.2)
        {
            LqrCompute ();

            if (Lqr.U_nxt > STEPPER_ACC_MAX)
                Lqr.U_nxt = STEPPER_ACC_MAX;
            else if (Lqr.U_nxt < STEPPER_ACC_MIN)
                Lqr.U_nxt = STEPPER_ACC_MIN;

            StepperMoveAccel(STEPPER_PPS_MAX, Lqr.U_nxt);
        }

        else
            StepperStop();
    }

    // UART update
    if ((SysTickCounter % (TIMER_FREQUENCY / UART_REFRESH_FREQUENCY)) == 0)
        DeviceUpdateUart ();

    // LCD update
    if ((SysTickCounter % (TIMER_FREQUENCY / LCD_REFRESH_FREQUENCY)) == 0)
        DeviceUpdateLcd();

    // RGB update
    if ((SysTickCounter % (TIMER_FREQUENCY / RGB_REFRESH_FREQUENCY)) == 0)
        DeviceUpdateRgb();

    // PWM update
    StepperUpdateMovement();

    // Button 1 scan
    button_event_data_t EventData;
    if (Button1.ScanEvent (&EventData))
        DeviceUpdateButton1(&EventData);

    // Button 2 scan
    if (Button2.ScanEvent (&EventData))
        DeviceUpdateButton2(&EventData);

    // Increase counter (or reset back to zero)
    SysTickCounter = SysTickCounter == 4294967295 ? 0 : (SysTickCounter + 1);
}

// ------------------------------------------------------------------------------------------------------- //
// Main function
// ------------------------------------------------------------------------------------------------------- //

void main (void)
{
    // --------------------------------------------------------------------------------------------------- //
    // CPU settings
    // --------------------------------------------------------------------------------------------------- //

    // Configure clock - 80 MHz - 200 MHz (PLL) / 2.5
    SysCtlClockSet (SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // Get CPU clock
    cpuClock = SysCtlClockGet();

    // Master interrupt enable API for all interrupts
    IntMasterEnable();

    // Enable lazy stacking for interrupt handlers
    FPUEnable();
    FPULazyStackingEnable();

    // --------------------------------------------------------------------------------------------------- //
    // LCD configuration
    // --------------------------------------------------------------------------------------------------- //

    // Define LCD configuration parameters
    lcd_config_t Lcd_Config;
    Lcd_Config.Periph.Ssi = LCD_SSI_PERIPH;
    Lcd_Config.Periph.Sclk = LCD_SCLK_PERIPH;
    Lcd_Config.Periph.Dn = LCD_DN_PERIPH;
    Lcd_Config.Periph.Sce = LCD_SCE_PERIPH;
    Lcd_Config.Periph.Dc = LCD_DC_PERIPH;
    Lcd_Config.Periph.Bkl = LCD_BKL_PERIPH;
    Lcd_Config.Base.Ssi = LCD_SSI_BASE;
    Lcd_Config.Base.Sclk = LCD_SCLK_BASE;
    Lcd_Config.Base.Dn = LCD_DN_BASE;
    Lcd_Config.Base.Sce = LCD_SCE_BASE;
    Lcd_Config.Base.Dc = LCD_DC_BASE;
    Lcd_Config.Base.Bkl = LCD_BKL_BASE;
    Lcd_Config.PinMux.Sclk = LCD_SCLK_CFG;
    Lcd_Config.PinMux.Dn = LCD_DN_CFG;
    Lcd_Config.Pin.Sclk = LCD_SCLK_PIN;
    Lcd_Config.Pin.Dn = LCD_DN_PIN;
    Lcd_Config.Pin.Sce = LCD_SCE_PIN;
    Lcd_Config.Pin.Dc = LCD_DC_PIN;
    Lcd_Config.Pin.Bkl = LCD_BKL_PIN;

    // Initialize LCD display
    Display.Init (&Lcd_Config);

    // Turn on backlight
    Display.Backlight(LCD_BKL_ON);

    // --------------------------------------------------------------------------------------------------- //
    // Buttons configuration
    // --------------------------------------------------------------------------------------------------- //

    // Define button 1 parameters
    button_config_t Button_Config;
    Button_Config.Hardware.Periph = BUTTON_GPIO_PERIPH;
    Button_Config.Hardware.Base = BUTTON_GPIO_BASE;
    Button_Config.Hardware.Pin = BUTTON_PIN_1;
    Button_Config.Params.Interval = BUTTON_SCAN_INTERVAL;
    Button_Config.Params.DeadTime = BUTTON_DEAD_TIME;
    Button_Config.Params.Window = BUTTON_WINDOW;
    Button_Config.Params.LongClickTimeout = BUTTON_LONG_TIMEOUT;

    // Initialize button 1
    Button1.Init(&Button_Config);

    // Define button 2 parameters
    Button_Config.Hardware.Pin = BUTTON_PIN_2;

    // Initialize button 2
    Button2.Init(&Button_Config);

    // --------------------------------------------------------------------------------------------------- //
    // Configure UART
    // --------------------------------------------------------------------------------------------------- //

    // Define UART parameters
    uart_config_t Uart_Config;
    Uart_Config.Hardware.PeriphUART = UART_UART_PERIPH;
    Uart_Config.Hardware.PeriphGPIO = UART_GPIO_PERIPH;
    Uart_Config.Hardware.BaseUART = UART_UART_BASE;
    Uart_Config.Hardware.BaseGPIO = UART_GPIO_BASE;
    Uart_Config.Hardware.PinMuxRX = UART_RX_CFG;
    Uart_Config.Hardware.PinMuxTX = UART_TX_CFG;
    Uart_Config.Hardware.PinRX = UART_RX_PIN;
    Uart_Config.Hardware.PinTX = UART_TX_PIN;
    Uart_Config.Hardware.Config = UART_CFG;
    Uart_Config.Hardware.Interrupt = UART_INTERRUPT;
    Uart_Config.Hardware.Callback = []() { Serial.ReceiveIsr(); };
    Uart_Config.Params.BaudRate = UART_BAUD_RATE;

    // Initialize UART
    Serial.Init(&Uart_Config);

    // --------------------------------------------------------------------------------------------------- //
    // Configure encoders
    // --------------------------------------------------------------------------------------------------- //

    // Define encoder parameters - Theta
    encoder_config_t Encoder_Config;
    Encoder_Config.Hardware.PeriphQEI = ENCODER_T_QEI_PERIPH;
    Encoder_Config.Hardware.PeriphGPIO = ENCODER_T_GPIO_PERIPH;
    Encoder_Config.Hardware.BaseQEI = ENCODER_T_QEI_BASE;
    Encoder_Config.Hardware.BaseGPIO = ENCODER_T_GPIO_BASE;
    Encoder_Config.Hardware.PinMuxA = ENCODER_T_A_CFG;
    Encoder_Config.Hardware.PinMuxB = ENCODER_T_B_CFG;
    Encoder_Config.Hardware.PinA = ENCODER_T_A_PIN;
    Encoder_Config.Hardware.PinB = ENCODER_T_B_PIN;
    Encoder_Config.Hardware.Config = ENCODER_T_CFG;
    Encoder_Config.Hardware.Interrupt = ENCODER_T_INTERRUPT;
    Encoder_Config.Hardware.Callback = [](){EncoderT.TimerIsr();};
    Encoder_Config.Params.PPR = ENCODER_T_PPR;
    Encoder_Config.Params.ScanFreq = ENCODER_T_FREQUENCY;

    // Initialize encoder
    EncoderT.Init(&Encoder_Config);

    // Define encoder parameters - X
    Encoder_Config.Hardware.PeriphQEI = ENCODER_X_QEI_PERIPH;
    Encoder_Config.Hardware.PeriphGPIO = ENCODER_X_GPIO_PERIPH;
    Encoder_Config.Hardware.BaseQEI = ENCODER_X_QEI_BASE;
    Encoder_Config.Hardware.BaseGPIO = ENCODER_X_GPIO_BASE;
    Encoder_Config.Hardware.PinMuxA = ENCODER_X_A_CFG;
    Encoder_Config.Hardware.PinMuxB = ENCODER_X_B_CFG;
    Encoder_Config.Hardware.PinA = ENCODER_X_A_PIN;
    Encoder_Config.Hardware.PinB = ENCODER_X_B_PIN;
    Encoder_Config.Hardware.Config = ENCODER_X_CFG;
    Encoder_Config.Hardware.Interrupt = ENCODER_X_INTERRUPT;
    Encoder_Config.Hardware.Callback = [](){EncoderX.TimerIsr();};
    Encoder_Config.Params.PPR = ENCODER_X_PPR;
    Encoder_Config.Params.ScanFreq = ENCODER_X_FREQUENCY;

    // Initialize encoder
    EncoderX.Init(&Encoder_Config);

    /* ---------------------------------------------------------------------------------------------------------------- */
    // RGB LED configuration
    /* ---------------------------------------------------------------------------------------------------------------- */

    // Define RGB LED configuration parameters
    rgb_config_t Led_Config;
    Led_Config.Periph.Pwm = RGB_PWM_PERIPH;
    Led_Config.Periph.Gpio = RGB_GPIO_PERIPH;
    Led_Config.Base.Pwm = RGB_PWM_BASE;
    Led_Config.Base.Gpio = RGB_GPIO_BASE;
    Led_Config.Gen.R = RGB_PWM_R_GEN;
    Led_Config.Gen.G = RGB_PWM_G_GEN;
    Led_Config.Gen.B = RGB_PWM_B_GEN;
    Led_Config.Out.R = RGB_PWM_R_OUT;
    Led_Config.Out.G = RGB_PWM_G_OUT;
    Led_Config.Out.B = RGB_PWM_B_OUT;
    Led_Config.OutBit.R = RGB_PWM_R_OUT_BIT;
    Led_Config.OutBit.G = RGB_PWM_G_OUT_BIT;
    Led_Config.OutBit.B = RGB_PWM_B_OUT_BIT;
    Led_Config.PinMux.R = RGB_PIN_R_CFG;
    Led_Config.PinMux.G = RGB_PIN_G_CFG;
    Led_Config.PinMux.B = RGB_PIN_B_CFG;
    Led_Config.Pin.R = RGB_R_PIN;
    Led_Config.Pin.G = RGB_G_PIN;
    Led_Config.Pin.B = RGB_B_PIN;
    Led_Config.Int.Interrupt = RGB_PWM_INT;
    Led_Config.Int.Gen = RGB_PWM_INT_GEN;
    Led_Config.Int.Callback = [](){Led.PwmIsr();};
    Led_Config.Params.PwmMode = RGB_PWM_CFG;
    Led_Config.Params.PwmFrequency = RGB_PWM_FREQ;

    // Initialize RGB LED
    Led.Init (&Led_Config);

    // --------------------------------------------------------------------------------------------------- //
    // Configure stepper GPIOs and PWM generator
    // --------------------------------------------------------------------------------------------------- //

    // Enable peripheral clocks
    SysCtlPeripheralEnable (STEPPER_GPIO_PERIPH);
    SysCtlPeripheralEnable (SWITCH_GPIO_PERIPH);
    SysCtlPeripheralEnable (STEPPER_PWM_PERIPH);

    // Power up delay
    SysCtlDelay (10);

    // Unlock used pins (has no effect if pin is not protected by the GPIOCR register
    GPIOUnlockPin(STEPPER_GPIO_BASE, STEPPER_PIN_DIR | STEPPER_PIN_EN);
    GPIOUnlockPin(SWITCH_GPIO_BASE, SWITCH_PIN_START | SWITCH_PIN_END);

    // Configure step pin as PWM output
    GPIOPinTypePWM (STEPPER_GPIO_BASE, STEPPER_PIN_STEP);
    GPIOPinConfigure (STEPPER_GPIO_CFG);

    // Configure DIR and EN pins as outputs
    GPIOPinTypeGPIOOutput (STEPPER_GPIO_BASE, STEPPER_PIN_DIR | STEPPER_PIN_EN);

    // Configure pins - 8mA drive with slew rate control and push-pull mode
    GPIOPadConfigSet (STEPPER_GPIO_BASE, STEPPER_PIN_DIR | STEPPER_PIN_EN, GPIO_STRENGTH_8MA_SC, GPIO_PIN_TYPE_STD);

    // Set initial output states
    StepperSetDirection (Stepper.Dir);
    StepperSetEnable(Stepper.Enabled);

    // Configure switch pins as inputs
    GPIOPinTypeGPIOInput (SWITCH_GPIO_BASE, SWITCH_PIN_START | SWITCH_PIN_END);

    // Enable pull-up resistor
    GPIOPadConfigSet (SWITCH_GPIO_BASE, SWITCH_PIN_START | SWITCH_PIN_END, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // Define initial PWM clock
    SysCtlPWMClockSet(STEPPER_PWM_DIV_CFG);
    Stepper.PwmClock = cpuClock / STEPPER_PWM_DIV;

    // Configure PWM options
    PWMGenConfigure (STEPPER_PWM_BASE, STEPPER_PWM_GEN, PWM_GEN_MODE_DOWN);

    // Set PWM frequency and duty
    //_StepperChangePWM (STEPPER_PPS_MIN);

    // Turn on the output pins
    //PWMOutputState (STEPPER_PWM_BASE, STEPPER_PWM_OUT_BIT , true);

    // Interrupt on zero count
    PWMGenIntTrigEnable(STEPPER_PWM_BASE, STEPPER_PWM_GEN, PWM_INT_CNT_ZERO);

    // Register interrupt handler - PWMGenIntRegister doesn't work for STEPPER_INT for some reason???
    IntRegister(STEPPER_INT, &IsrPwmZero);
    //PWMGenIntRegister(STEPPER_PWM_BASE, STEPPER_PWM_GEN, &IsrPwmZero);

    // Enable interrupt
    PWMIntEnable(STEPPER_PWM_BASE, STEPPER_INT_PWM);
    IntEnable(STEPPER_INT);

    // --------------------------------------------------------------------------------------------------- //
    // Configure Systick timer
    // --------------------------------------------------------------------------------------------------- //

    // Set timer period
    SysTickPeriodSet ((cpuClock/TIMER_FREQUENCY) - 1);

    // Register interrupt handler
    SysTickIntRegister (&IsrSysTick);

    // Enable timer interrupt
    SysTickIntEnable ();

    // --------------------------------------------------------------------------------------------------- //
    // Display info on LCD for a while
    // --------------------------------------------------------------------------------------------------- //

    Display.WriteString(DEVICE_FW_NAME, LCD_FONT_SMALL, LCD_PIXEL_ON);
    Display.Goto(1, 0);
    Display.WriteString("Version: ", LCD_FONT_SMALL, LCD_PIXEL_ON);
    Display.WriteString(DEVICE_FW_VERSION, LCD_FONT_SMALL, LCD_PIXEL_ON);
    Display.Goto(2, 0);
    Display.WriteString(__DATE__ "\0", LCD_FONT_SMALL, LCD_PIXEL_ON);
    Display.Goto(3, 0);
    Display.WriteString(__TIME__ "\0", LCD_FONT_SMALL, LCD_PIXEL_ON);
    Display.Goto(5, 0);
    Display.WriteString(DEVICE_FW_AUTHOR, LCD_FONT_SMALL, LCD_PIXEL_ON);
    Display.Commit();

    SysCtlDelay(30000000);

    // --------------------------------------------------------------------------------------------------- //
    // Start application state machine
    // --------------------------------------------------------------------------------------------------- //

    // Start timer
    SysTickEnable ();

    // Calibrate cart X position limits
    CartCalibratePos ();

    // Calibrate pendulum angle
    PendulumCalibrateAngle ();

    // --------------------------------------------------------------------------------------------------- //
    // Main loop
    // --------------------------------------------------------------------------------------------------- //

    while (1)
    {
//        // Start stepper
//        StepperMoveAccel (PPS, -Accel);
//
//        // Move until home switch triggers
//        while (Stepper.Enabled);
//
//        // Start stepper
//        StepperMoveAccel (PPS, Accel);
//
//        // Move until home switch triggers
//        while (Stepper.Enabled);
    }
}

// ------------------------------------------------------------------------------------------------------- //
// End of code
// ------------------------------------------------------------------------------------------------------- //

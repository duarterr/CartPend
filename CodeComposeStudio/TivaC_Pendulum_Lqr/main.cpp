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

// ------------------------------------------------------------------------------------------------------- //
// Functions Prototypes
// ------------------------------------------------------------------------------------------------------- //

// Send device info via UART
void DeviceUpdateUart(void);

// LCD update
void DeviceUpdateLcd(void);

// RGB LED update
void DeviceUpdateRgb(void);

// Button 1 event handler
void DeviceUpdateButton1 (void);

// Button 2 event handler
void DeviceUpdateButton2 (void);

// SysTick interrupt service routine
void IsrSysTick (void);

// ------------------------------------------------------------------------------------------------------- //
// Objects
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

// Stepper object - From Stepper_TivaC class
Stepper Stepper;

// LQR object - From LQR_TivaC class
Lqr LQR;

// ------------------------------------------------------------------------------------------------------- //
// Variables
// ------------------------------------------------------------------------------------------------------- //

// Cart-related variables
volatile cart_t Cart = cart_t_default;

// Pendulum-related variables
volatile pendulum_t Pendulum = pendulum_t_default;

// Calibration variables
volatile calibration_t CalT = {.Status = CAL_PENDING, .Progress = 0, .Offset = 0, .Max = ENCODER_T_PPR};
volatile calibration_t CalX = {.Status = CAL_PENDING, .Progress = 0, .Offset = 1000, .Max = ENCODER_X_PPR};

volatile float PosRefShaddow = 0;


// Global counters
volatile uint32_t SysTickCounter = 0;
volatile uint32_t ControlCounter = 0;
volatile uint32_t UartCounter = 0;
volatile uint32_t LcdCounter = 0;
volatile uint32_t RgbCounter = 0;
volatile uint32_t ButtonCounter = 0;
volatile uint32_t EncoderTCounter = 0;
volatile uint32_t EncoderXCounter = 0;

// Intervals
const uint32_t ControlInterval = TIMER_FREQUENCY / CONTROL_LOOP_FREQUENCY;
const uint32_t UartInterval = TIMER_FREQUENCY / UART_REFRESH_FREQUENCY;
const uint32_t LcdInterval = TIMER_FREQUENCY / LCD_REFRESH_FREQUENCY;
const uint32_t RgbInterval = TIMER_FREQUENCY / RGB_REFRESH_FREQUENCY;
const uint32_t ButtonInterval = TIMER_FREQUENCY / BUTTON_SCAN_FREQUENCY;
const uint32_t EncoderTInterval = TIMER_FREQUENCY / ENCODER_T_FREQUENCY;
const uint32_t EncoderXInterval = TIMER_FREQUENCY / ENCODER_X_FREQUENCY;

// TODO: REMOVE
float Vel = 0.1;
float Acc = 0.1;
stepper_status_t StepperStatus = stepper_status_t_default;

// ------------------------------------------------------------------------------------------------------- //
// Update cart state based on encoder readings
// ------------------------------------------------------------------------------------------------------- //

void CartUpdateState (void)
{
    // Calculate cart absolute position - Remove offset added during calibration
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

    // Start stepper - Backwards direction
    Stepper.Move (-0.2, 1);

    // Move until home switch triggers
    while (Stepper.GetEnabled());

    // Set encoder counter - Add offset because cart can move past limit switch trigger point
    EncoderX.SetPos(CalX.Offset);

    // Start stepper - Forward direction
    Stepper.Move (0.2, 1);

    // Move until end switch triggers
    while (Stepper.GetEnabled())
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

    // Wait until X encoder updates
    float LastSysTick = SysTickCounter;
    while (SysTickCounter - LastSysTick < EncoderXInterval);

    // Start stepper - Backwards direction
    Stepper.Move (-0.5, 1);

    // Move to X-axis 0
    while (Cart.Pos > 0);
    Stepper.Stop();

    // Aux variables
    uint32_t TimeEven[10] = {};
    uint32_t TimeOdd[10] = {};
    uint32_t PosEven[10] = {};
    uint32_t PosOdd[10] = {};
    float SlopeEven, OffsetEven, SlopeOdd, OffsetOdd;
    int32_t Offset = 0;


    // Set reference to a known value
    EncoderT.SetPos(ENCODER_T_PPR/2);

    // Wait for angle to be increasing
    while (EncoderT.GetDir() != 1);

    int32_t DirectionLast = EncoderT.GetDir();

    for (uint8_t Idx = 0; Idx < ENCODER_T_CAL_CYCLES; Idx++)
    {
        // Wait for direction change
        while (EncoderT.GetDir() == DirectionLast);
        DirectionLast = EncoderT.GetDir();

        // Save time and position
        PosEven[Idx] = EncoderT.GetPos();;
        TimeEven[Idx] = SysTickCounter;

        // Increase progress
        CalT.Progress += (50/ENCODER_T_CAL_CYCLES);

        // Wait for direction change
        while (EncoderT.GetDir() == DirectionLast);
        DirectionLast = EncoderT.GetDir();

        // Save time and position
        PosOdd[Idx] = EncoderT.GetPos();;
        TimeOdd[Idx] = SysTickCounter;

        // Increase progress
        CalT.Progress += (50/ENCODER_T_CAL_CYCLES);
    }

    // Calculate the slopes and offsets for both curves
    Aux::LinearInterpolation(TimeEven, PosEven, ENCODER_T_CAL_CYCLES, &SlopeEven, &OffsetEven);
    Aux::LinearInterpolation(TimeOdd, PosOdd, ENCODER_T_CAL_CYCLES, &SlopeOdd, &OffsetOdd);

    // Get the time where the curves will intersect
    float TimeCross = (OffsetOdd - OffsetEven)/(SlopeEven - SlopeOdd);

    // Calculate offset of theta encoder in pulses (rounding)
    Offset = ENCODER_T_PPR/2 - (SlopeEven * TimeCross + OffsetEven + 0.5);

    // Set new reference - Down position is ENCODER_T_PPR/2
    EncoderT.SetPos(EncoderT.GetPos() + Offset + ENCODER_T_PPR/2);

    // Define encoder maximum counter value
    CalT.Max = ENCODER_T_PPR;

    CalT.Progress = 100;
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

        float AccNow = 0;
        if (Stepper.GetTargetVel() > Stepper.GetCurrentVel())
            AccNow = Stepper.GetCurrentAcc();
        else if (Stepper.GetTargetVel() < Stepper.GetCurrentVel())
            AccNow = -Stepper.GetCurrentAcc();
        Aux::F2Str(AccNow, Aux_String, 6);

        Serial.SendString (Aux_String);
        Serial.SendString (";");

        ltoa ((Stepper.GetDir() == 1 ? Stepper.GetPwmFrequency() : -Stepper.GetPwmFrequency()), Aux_String, 10);

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
        NumberToWrite = LQR.GetReference(0);
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
        xP = (uint8_t)Aux::Map (PosRefShaddow, -X_VALUE_ABS_M, X_VALUE_ABS_M, 0, (PCD8544_COLUMNS - 1));
        Display.DrawPixel(xP, 12, LCD_PIXEL_XOR);
        Display.DrawPixel(xP, 13, LCD_PIXEL_OFF);

        // Reference mark
        xP = (uint8_t)Aux::Map (LQR.GetReference(0), -X_VALUE_ABS_M, X_VALUE_ABS_M, 0, (PCD8544_COLUMNS - 1));
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
        if (Aux::FastFabs(LQR.GetReference(0) - LQR.GetState(0)) < 0.001)
            Led.SetColor(RGB_GREEN, 50);
        else
            Led.SetColor(RGB_YELLOW, 50);
    }
}

// ------------------------------------------------------------------------------------------------------- //
// Button 1 event handler
// ------------------------------------------------------------------------------------------------------- //

void DeviceUpdateButton1 (void)
{
    button_event_data_t EventData;

    if (Button1.ScanEvent (&EventData))
    {
        // Button short clicks
        if (EventData.EventCode == BUTTON_SHORT_CLICK)
        {
            // Single click
            if (EventData.Counter == 1)
            {
                PosRefShaddow -= 0.025;

                if (PosRefShaddow < -0.15)
                PosRefShaddow = -0.15;
            }

            // Double click
            else if (EventData.Counter == 2)
                LQR.SetReference(0, PosRefShaddow);
        }
    }
}

// ------------------------------------------------------------------------------------------------------- //
// Button 2 event handler
// ------------------------------------------------------------------------------------------------------- //

void DeviceUpdateButton2 (void)
{
    button_event_data_t EventData;

    if (Button2.ScanEvent (&EventData))
    {
        // Button short clicks
        if (EventData.EventCode == BUTTON_SHORT_CLICK)
        {
            // Single click
            if (EventData.Counter == 1)
            {
                PosRefShaddow += 0.025;

                if (PosRefShaddow > 0.15)
                    PosRefShaddow = 0.15;
            }

            // Double click
            else if (EventData.Counter == 2)
                LQR.SetReference(0, PosRefShaddow);
        }
    }
}

// ------------------------------------------------------------------------------------------------------- //
// SysTick interrupt service routine
// ------------------------------------------------------------------------------------------------------- //

void IsrSysTick (void)
{
    // Pendulum state update
    if (++EncoderTCounter >= EncoderTInterval)
    {
        PendulumUpdateState();
        EncoderTCounter = 0;
    }

    // Cart state update
    if (++EncoderXCounter >= EncoderXInterval)
    {
        CartUpdateState();

        // Check for stalls - Done here because we need updated cart data
        if (Stepper.CheckForStall(EncoderX.GetPos()))
            Stepper.Stop();

        EncoderXCounter = 0;
    }

    // Control loop
    if ((++ControlCounter >= ControlInterval) && ((CalX.Status == CAL_DONE) && (CalT.Status == CAL_DONE)))
    {
        // Code for control loops
        if (Aux::FastFabs(Pendulum.Pos) < 0.2)
        {
            LQR.SetState(0, Cart.Pos);
            LQR.SetState(1, Cart.Vel);
            LQR.SetState(2, Pendulum.Pos);
            LQR.SetState(3, Pendulum.Vel);

            float Unxt = LQR.Compute ();

            if (Unxt > STEPPER_ACC_MAX)
                Unxt = STEPPER_ACC_MAX;
            else if (Unxt < -STEPPER_ACC_MAX)
                Unxt = -STEPPER_ACC_MAX;

            float FinalVelocity = (Unxt < 0 ? -STEPPER_VEL_MAX : STEPPER_VEL_MAX);
            float Acceleration = Aux::FastFabs(Unxt);

            Stepper.Move(FinalVelocity, Acceleration);
        }

        else
            Stepper.Stop();

        ControlCounter = 0;
    }

    // UART update
    if (++UartCounter >= UartInterval)
    {
        DeviceUpdateUart();
        UartCounter = 0;
    }

    // LCD update
    if (++LcdCounter >= LcdInterval)
    {
        DeviceUpdateLcd();
        LcdCounter = 0;
    }

    // RGB update
    if (++RgbCounter >= RgbInterval)
    {
        DeviceUpdateRgb();
        RgbCounter = 0;
    }

    // Buttons update
    if (++ButtonCounter >= ButtonInterval)
    {
        DeviceUpdateButton1();
        DeviceUpdateButton2();

        ButtonCounter = 0;
    }

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

    // Master interrupt enable API for all interrupts
    IntMasterEnable();

    // Enable lazy stacking for interrupt handlers
    FPUEnable();
    FPULazyStackingEnable();

    // --------------------------------------------------------------------------------------------------- //
    // LCD configuration
    // --------------------------------------------------------------------------------------------------- //

    // Initialize LCD display
    Display.Init (&Lcd_Config);

    // Turn on backlight
    Display.Backlight(LCD_BKL_ON);

    // --------------------------------------------------------------------------------------------------- //
    // Buttons configuration
    // --------------------------------------------------------------------------------------------------- //

    // Initialize button 1
    Button1.Init(&Button1_Config);

    // Initialize button 2
    Button2.Init(&Button2_Config);

    // --------------------------------------------------------------------------------------------------- //
    // Configure UART
    // --------------------------------------------------------------------------------------------------- //

    // Initialize UART
    Serial.Init(&Uart_Config);

    // --------------------------------------------------------------------------------------------------- //
    // Configure encoders
    // --------------------------------------------------------------------------------------------------- //

    // Initialize encoder - Theta-axis
    EncoderT.Init(&EncoderT_Config);

    // Initialize encoder - X-axis
    EncoderX.Init(&EncoderX_Config);

    // --------------------------------------------------------------------------------------------------- //
    // RGB LED configuration
    // --------------------------------------------------------------------------------------------------- //

    // Initialize RGB LED
    Led.Init (&Led_Config);

    // --------------------------------------------------------------------------------------------------- //
    // Configure stepper
    // --------------------------------------------------------------------------------------------------- //

    // Initialize stepper
    Stepper.Init (&Stepper_Config);

    // --------------------------------------------------------------------------------------------------- //
    // Configure LQR
    // --------------------------------------------------------------------------------------------------- //

    // Define LQR gains and references
    float Gains[4] = {-7.071068, -8.212938, -49.663068, -10.818448};
    float Refs[4] = {0};

    // Initialize LQR controller
    LQR.Init (Gains, Refs, 4);

    // --------------------------------------------------------------------------------------------------- //
    // Configure Systick timer
    // --------------------------------------------------------------------------------------------------- //

    // Set timer period
    SysTickPeriodSet ((SysCtlClockGet()/TIMER_FREQUENCY) - 1);

    // Register interrupt handler
    SysTickIntRegister (IsrSysTick);

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

//    // Set encoder counter - Add offset because cart can move past limit switch trigger point
//    EncoderX.SetPos(CalX.Max/2 + CalX.Offset);
//    SysTickCounter = 0;
//    while (SysTickCounter < 1000);
//
//    Stepper.Move (STEPPER_VEL_MAX, 1);
//
//    SysTickCounter = 0;
//    while (SysTickCounter < 500);
//
//    Stepper.Move (-STEPPER_VEL_MAX, 1);
//
//    SysTickCounter = 0;
//    while (SysTickCounter < 1000);
//
//    Stepper.Move (0, 2);

    while (1)
    {
//        // Start stepper
//        Stepper.Move (-Vel, Acc);
//
//        // Move until home switch triggers
//        while (Stepper.GetEnabled())
//            Stepper.GetStatus(&StepperStatus);
//
//        // Start stepper
//        Stepper.Move (Vel, Acc);
//
//        // Move until home switch triggers
//        while (Stepper.GetEnabled())
//            Stepper.GetStatus(&StepperStatus);
    }
}

// ------------------------------------------------------------------------------------------------------- //
// End of code
// ------------------------------------------------------------------------------------------------------- //

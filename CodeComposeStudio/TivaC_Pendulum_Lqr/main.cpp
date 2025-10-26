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

// Update cart state based on encoder readings
void CartUpdateState ();

// Update pendulum state based on encoder readings
void PendulumUpdateState ();

// Move cart to home position (-X_VALUE_ABS_M)
void CalibrationCartGoHome ();

// Calibrate cart X position limits
void CalibrationCartPos ();

// Calibrate cart velocity constant - Must be called with the pendulum detached
void CalibrationCartVel ();

// Calibrate motor velocity constant - Must be called with the pendulum detached
void CalibrationMotorVel ();

// Calibrate pendulum angle
void CalibrationPendulumPos ();

// UART RX callback
void UartRxCallback(char received_char);

// Process received UART command
void UartProcessCommand();

// Send device info via UART
void UartSendStatus();

// LCD update
void DeviceUpdateLcd();

// RGB LED update
void DeviceUpdateRgb();

// Button 1 event handler
void DeviceUpdateButton1 ();

// Button 2 event handler
void DeviceUpdateButton2 ();

// Control loop
void DeviceUpdateControl();

// Apply PosRefShaddow to controllers
void ControlSetPosReference (void);

// SysTick interrupt service routine
void IsrSysTick ();

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

// Cart PID controller object - From Pid_TivaC class
Pid ControllerCartPid;

// Cart lead controller object - From Lead_TivaC class
Lead ControllerCartLead;

// Cart state feedback controller object - From StateFeedback_TivaC class
StateFeedback ControllerCartSF;

// Pendulum state feedback controller object - From StateFeedback_TivaC class
StateFeedback ControllerPendSF;

// Full state LQR controller object - From StateFeedback_TivaC class
StateFeedback ControllerFullLqr;

// ------------------------------------------------------------------------------------------------------- //
// Variables
// ------------------------------------------------------------------------------------------------------- //

// Cart-related variables
volatile cart_t Cart = cart_t_default;

// Pendulum-related variables
volatile pendulum_t Pendulum = pendulum_t_default;

// Calibration variables
volatile calibration_t CalT = {.Status = CAL_PENDING, .Progress = 0, .Offset = 0, .Max = ENCODER_T_PPR, .Kv = 1};
volatile calibration_t CalX = {.Status = CAL_PENDING, .Progress = 0, .Offset = 1000, .Max = ENCODER_X_MAX, .Kv = ENCODER_X_KV};

// Control mode
volatile control_mode_t ControlMode = CONTROL_OFF;

// Cart position reference (only user-changeable state) - Shadow value
volatile float PosRefShadow = 0;

// UART RX variables
volatile char UartRxBuffer[64] = {0};
volatile uint8_t UartRxIndex = 0;
volatile bool UartCommandReady = false;

// Global counter
volatile uint32_t SysTickCounter = 0;

// ------------------------------------------------------------------------------------------------------- //
// Update cart state based on encoder readings
// ------------------------------------------------------------------------------------------------------- //

void CartUpdateState ()
{
    // Calculate cart absolute position - Remove offset added during calibration
    Cart.Pos = ((float)X_VALUE_TOTAL_M/CalX.Max)*((int32_t)EncoderX.GetPos() - CalX.Offset) - X_VALUE_ABS_M;

    // Calculate cart velocity
    Cart.Vel = (EncoderX.GetVel() / CalX.Kv) * EncoderX.GetDir();
}

// ------------------------------------------------------------------------------------------------------- //
// Update pendulum state based on encoder readings
// ------------------------------------------------------------------------------------------------------- //

void PendulumUpdateState ()
{
    // Calculate pendulum absolute position
    Pendulum.Pos = ((float)T_VALUE_MAX_RAD/CalT.Max)*((int32_t)EncoderT.GetPos() - CalT.Offset) - T_VALUE_ABS_MAX;

    // Calculate pendulum velocity
    float raw_vel = (((T_VALUE_MAX_RAD * EncoderT.GetVel()) / CalT.Max) * ENCODER_T_FREQUENCY) * EncoderT.GetDir();

    const float alpha_lpf = 1;
    Pendulum.Vel = alpha_lpf * raw_vel + (1 - alpha_lpf) * Pendulum.Vel;
}

// ------------------------------------------------------------------------------------------------------- //
// Move cart to home position (-X_VALUE_ABS_M)
// ------------------------------------------------------------------------------------------------------- //

void CalibrationCartGoHome ()
{
    // Set calibration variables
    CalX.Progress = 0;
    CalX.Status = CAL_RUNNING;

    // Start stepper - Backwards direction
    Stepper.Move (-STEPPER_VEL_CAL, 0.25);

    // Move until home switch triggers
    while (Stepper.GetEnable());

    // Set encoder counter - Add offset because cart can move past limit switch trigger point
    EncoderX.SetPos(CalX.Offset);

    // Set calibration flag
    CalX.Status = CAL_DONE;
}

// ------------------------------------------------------------------------------------------------------- //
// Calibrate cart X position limits
// ------------------------------------------------------------------------------------------------------- //

void CalibrationCartPos ()
{
    // Set calibration variables
    CalX.Progress = 0;
    CalX.Status = CAL_RUNNING;

    // Start stepper - Backwards direction
    Stepper.Move (-STEPPER_VEL_CAL, 0.25);

    // Move until home switch triggers
    while (Stepper.GetEnable());

    // Set encoder counter - Add offset because cart can move past limit switch trigger point
    EncoderX.SetPos(CalX.Offset);

    // Start stepper - Forward direction
    Stepper.Move (STEPPER_VEL_CAL, 0.25);

    // Move until end switch triggers
    while (Stepper.GetEnable())
        CalX.Progress = (uint32_t)(EncoderX.GetPos() * 100)/ENCODER_X_MAX;

    // Define encoder maximum counter value - Without offset
    CalX.Max = EncoderX.GetPos() - CalX.Offset;

    // Set calibration flag
    CalX.Status = CAL_DONE;
}

// ------------------------------------------------------------------------------------------------------- //
// Calibrate cart velocity constant - Must be called with the pendulum detached
// ------------------------------------------------------------------------------------------------------- //

void CalibrationCartVel ()
{
    const uint8_t NUM_ITERATIONS = 3;  // Number of calibration runs
    const float VEL_MULTIPLIERS[] = {0.25, 0.625, 1};  // Different velocities to test

    float KvEncoder[NUM_ITERATIONS] = {};

    for (uint8_t IterationCounter = 0; IterationCounter < NUM_ITERATIONS; IterationCounter++)
    {
        // Initial homing - Forward direction to end switch
        Stepper.Move (STEPPER_VEL_CAL, 0.25);
        while (Stepper.GetEnable());

        // Start stepper - Backward direction
        Stepper.Move (-VEL_MULTIPLIERS[IterationCounter], STEPPER_ACC_MAX);

        // Wait for speed to stabilise
        while (Stepper.GetCurrentVel() != Stepper.GetTargetVel());

        // Capture start values
        uint32_t PosStart = EncoderX.GetPos();
        uint32_t TickStart = SysTickCounter;
        uint32_t EncoderVelSum = 0;
        uint32_t SampleCounter = 0;

        // Collect samples until 90% of course
        while ((EncoderX.GetPos() > (CalX.Max / 10)) && (Stepper.GetEnable()))
        {
            EncoderVelSum += EncoderX.GetVel();
            SampleCounter++;
        }

        // Capture end values
        uint32_t PosEnd = EncoderX.GetPos();
        uint32_t TickEnd = SysTickCounter;

        // Stop motor
        Stepper.Move (0, STEPPER_ACC_MAX);
        while (Stepper.GetCurrentVel() != 0);

        // Calculate Kv for backward run
        if (SampleCounter > 0)
        {
            float AverageEncoderVel = (float)EncoderVelSum / SampleCounter;
            float DeltaPos = (float)(PosStart - PosEnd) * X_VALUE_TOTAL_M / CalX.Max;
            float DeltaT = (float)(TickEnd - TickStart) / TIMER_FREQUENCY;
            float RealVel = DeltaPos / DeltaT;
            KvEncoder[IterationCounter] = AverageEncoderVel / RealVel;
        }

        // Move slowly to backward end switch
        Stepper.Move (-STEPPER_VEL_CAL, 0.25);
        while (Stepper.GetEnable());

        // Start stepper - Forward direction
        Stepper.Move (VEL_MULTIPLIERS[IterationCounter], STEPPER_ACC_MAX);

        // Wait for speed to stabilise
        while (Stepper.GetCurrentVel() != Stepper.GetTargetVel());

        // Capture start values
        PosStart = EncoderX.GetPos();
        TickStart = SysTickCounter;
        EncoderVelSum = 0;
        SampleCounter = 0;

        // Collect samples until 90% of course
        while ((EncoderX.GetPos() < (CalX.Max * 0.9)) && (Stepper.GetEnable()))
        {
            EncoderVelSum += EncoderX.GetVel();
            SampleCounter++;
        }

        // Capture end values
        PosEnd = EncoderX.GetPos();
        TickEnd = SysTickCounter;

        // Stop motor
        Stepper.Move (0, STEPPER_ACC_MAX);
        while (Stepper.GetCurrentVel() != 0);

        // Calculate Kv for forward run
        if (SampleCounter > 0)
        {
            float AverageEncoderVel = (float)EncoderVelSum / SampleCounter;
            float DeltaPos = (float)(PosEnd - PosStart) * X_VALUE_TOTAL_M / CalX.Max;
            float DeltaT = (float)(TickEnd - TickStart) / TIMER_FREQUENCY;
            float RealVel = DeltaPos / DeltaT;
            KvEncoder[IterationCounter] = AverageEncoderVel / RealVel;
        }
    }

    // Calculate average Kv
    float KvEncoderSum = 0;
    for (uint8_t IterationCounter = 0; IterationCounter < NUM_ITERATIONS; IterationCounter++)
        KvEncoderSum += KvEncoder[IterationCounter];

    CalX.Kv = KvEncoderSum / NUM_ITERATIONS;
}

// ------------------------------------------------------------------------------------------------------- //
// Calibrate motor velocity constant - Must be called with the pendulum detached
// ------------------------------------------------------------------------------------------------------- //

void CalibrationMotorVel ()
{
    const uint8_t NUM_ITERATIONS = 3;
    const float VEL_MULTIPLIERS[] = {0.25, 0.625, 1};
    float StepperKv[NUM_ITERATIONS * 2] = {};
    uint8_t KvCounter = 0;

    for (uint8_t IterationCounter = 0; IterationCounter < NUM_ITERATIONS; IterationCounter++)
    {
        // Initial homing - Forward direction to end switch
        Stepper.Move (STEPPER_VEL_CAL, 0.25);
        while (Stepper.GetEnable());

        // Start stepper - Backward direction at test velocity
        Stepper.Move (-VEL_MULTIPLIERS[IterationCounter], STEPPER_ACC_MAX);

        // Wait for speed to stabilise
        while (Stepper.GetCurrentVel() != Stepper.GetTargetVel());

        uint32_t EncoderVelSum = 0;
        uint64_t PwmSum = 0;
        uint32_t SampleCounter = 0;

        // Collect samples until 90% of course
        while ((EncoderX.GetPos() > (CalX.Max / 10)) && (Stepper.GetEnable()))
        {
            EncoderVelSum += EncoderX.GetVel();
            PwmSum += Stepper.GetPwmFrequency();
            SampleCounter++;
        }

        // Stop motor
        Stepper.Move (0, STEPPER_ACC_MAX);
        while (Stepper.GetCurrentVel() != 0);

        // Calculate Kv for backward run
        if (SampleCounter > 0)
        {
            float AverageVel = ((float)EncoderVelSum / SampleCounter) / CalX.Kv;
            float AveragePwm = (float)PwmSum / SampleCounter;
            StepperKv[KvCounter++] = AveragePwm / AverageVel;
        }

        // Move slowly to backward end switch
        Stepper.Move (-STEPPER_VEL_CAL, 0.25);
        while (Stepper.GetEnable());

        // Start stepper - Forward direction at test velocity
        Stepper.Move (VEL_MULTIPLIERS[IterationCounter], STEPPER_ACC_MAX);

        // Wait for speed to stabilise
        while (Stepper.GetCurrentVel() != Stepper.GetTargetVel());

        EncoderVelSum = 0;
        PwmSum = 0;
        SampleCounter = 0;

        // Collect samples until 90% of course
        while ((EncoderX.GetPos() < (CalX.Max * 0.9)) && (Stepper.GetEnable()))
        {
            EncoderVelSum += EncoderX.GetVel();
            PwmSum += Stepper.GetPwmFrequency();
            SampleCounter++;
        }

        // Stop motor
        Stepper.Move (0, STEPPER_ACC_MAX);
        while (Stepper.GetCurrentVel() != 0);

        // Calculate Kv for forward run
        if (SampleCounter > 0)
        {
            float AverageVel = ((float)EncoderVelSum / SampleCounter) / CalX.Kv;
            float AveragePwm = (float)PwmSum / SampleCounter;
            StepperKv[KvCounter++] = AveragePwm / AverageVel;
        }
    }

    // Calculate average Kv
    float StepperKvSum = 0;
    for (uint8_t Counter = 0; Counter < KvCounter; Counter++)
        StepperKvSum += StepperKv[Counter];
    float KvMotor = StepperKvSum / KvCounter;

    // Update motor Kv parameter
    Stepper.SetKv(KvMotor);
}

// ------------------------------------------------------------------------------------------------------- //
// Calibrate pendulum angle
// ------------------------------------------------------------------------------------------------------- //

void CalibrationPendulumPos ()
{
    CalT.Progress = 0;
    CalT.Status = CAL_RUNNING;

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
// UART RX callback
// ------------------------------------------------------------------------------------------------------- //

void UartRxCallback(char received_char)
{
    // Detect end of command
    if (received_char == '\n' || received_char == '\r')
    {
        if (UartRxIndex > 0)
        {
            UartRxBuffer[UartRxIndex] = '\0';
            UartCommandReady = true;
        }
    }

    // Add char to buffer
    else if (UartRxIndex < sizeof(UartRxBuffer) - 1)
        UartRxBuffer[UartRxIndex++] = received_char;

    // Overflow
    else
        UartRxIndex = 0;
}

// ------------------------------------------------------------------------------------------------------- //
// Process received UART command
// ------------------------------------------------------------------------------------------------------- //

void UartProcessCommand()
{
    if (!UartCommandReady)
        return;

    char* cmd = (char*)UartRxBuffer;

    // Header
    Serial.SendString("[RESPONSE]");

    // Control mode
    if (strncmp(cmd, "MODE:", 5) == 0)
    {
        int mode = atoi(cmd + 5);
        if (mode >= 0 && mode < sizeof_control_mode_t)
        {
            ControlMode = (control_mode_t)mode;

            Serial.SendString("OK: MODE_CHANGED\n");
        }
        else
            Serial.SendString("ERROR: INVALID_MODE\n");
    }

    // Shadow reference
    else if (strncmp(cmd, "SREF:", 5) == 0)
    {
        float ref = atof(cmd + 5);
        if (ref >= -0.2 && ref <= 0.2)
        {
            PosRefShadow = ref;
            Serial.SendString("OK: SHADOW_REF_SET\n");
        }
        else
            Serial.SendString("ERROR: REF_OUT_OF_RANGE\n");
    }

    // Apply shadow reference
    else if (strncmp(cmd, "AREF", 4) == 0)
    {
        ControlSetPosReference();
        Serial.SendString("OK: REF_APPLIED\n");
    }

    else
        Serial.SendString("ERROR: UNKNOWN_COMMAND\n");

    // Clear buffer
    UartCommandReady = false;
    UartRxIndex = 0;
    memset((void*)UartRxBuffer, 0, sizeof(UartRxBuffer));
}

// ------------------------------------------------------------------------------------------------------- //
// Send device info via UART
// ------------------------------------------------------------------------------------------------------- //

void UartSendStatus()
{
    char Aux_String[50];

    // Header
    Serial.SendString("[STATUS]");

    // Timestamp
    ltoa(SysTickCounter, Aux_String, 10);
    Serial.SendString(Aux_String);
    Serial.SendString(";");

    // Control mode
    ltoa((int32_t)ControlMode, Aux_String, 10);
    Serial.SendString(Aux_String);
    Serial.SendString(";");

    // Calibration status
    ltoa((int32_t)CalX.Status, Aux_String, 10);
    Serial.SendString(Aux_String);
    Serial.SendString(";");

    ltoa((int32_t)CalT.Status, Aux_String, 10);
    Serial.SendString(Aux_String);
    Serial.SendString(";");

    // Motor target velocity
    Aux::F2Str(Stepper.GetTargetVel(), Aux_String, 6);
    Serial.SendString(Aux_String);
    Serial.SendString(";");

    // Motor acceleration
    float AccNow = 0;
    if (Stepper.GetTargetVel() > Stepper.GetCurrentVel())
        AccNow = Stepper.GetCurrentAcc();
    else if (Stepper.GetTargetVel() < Stepper.GetCurrentVel())
        AccNow = -Stepper.GetCurrentAcc();
    Aux::F2Str(AccNow, Aux_String, 6);
    Serial.SendString(Aux_String);
    Serial.SendString(";");

    // PWM frequency
    ltoa((Stepper.GetDir() == 1 ? Stepper.GetPwmFrequency() : -Stepper.GetPwmFrequency()), Aux_String, 10);
    Serial.SendString(Aux_String);
    Serial.SendString(";");

    // Cart position
    Aux::F2Str(Cart.Pos, Aux_String, 6);
    Serial.SendString(Aux_String);
    Serial.SendString(";");

    // Cart velocity
    Aux::F2Str(Cart.Vel, Aux_String, 6);
    Serial.SendString(Aux_String);
    Serial.SendString(";");

    // Pendulum position
    Aux::F2Str(Pendulum.Pos, Aux_String, 6);
    Serial.SendString(Aux_String);
    Serial.SendString(";");

    // Pendulum velocity
    Aux::F2Str(Pendulum.Vel, Aux_String, 6);
    Serial.SendString(Aux_String);
    Serial.SendString(";");

    // Position reference
    Aux::F2Str(ControllerFullLqr.GetReference(0), Aux_String, 6);
    Serial.SendString(Aux_String);
    Serial.SendString(";");

    // Shadow reference
    Aux::F2Str(PosRefShadow, Aux_String, 6);
    Serial.SendString(Aux_String);

    Serial.SendString("\n");
}

// ------------------------------------------------------------------------------------------------------- //
// LCD update
// ------------------------------------------------------------------------------------------------------- //

void DeviceUpdateLcd()
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
        if (ControlMode == CONTROL_OFF)
        {
            Display.Goto(0, 9);
            Display.WriteString("CONTROL OFF", LCD_FONT_SMALL, LCD_PIXEL_XOR);
        }

        else if (ControlMode == CONTROL_CART_PID)
        {
            Display.Goto(0, 18);
            Display.WriteString("CART PID", LCD_FONT_SMALL, LCD_PIXEL_XOR);
        }

        else if (ControlMode == CONTROL_CART_LEAD)
        {
            Display.Goto(0, 15);
            Display.WriteString("CART LEAD", LCD_FONT_SMALL, LCD_PIXEL_XOR);
        }

        else if (ControlMode == CONTROL_CART_SF)
        {
            Display.Goto(0, 21);
            Display.WriteString("CART SS", LCD_FONT_SMALL, LCD_PIXEL_XOR);
        }

        else if (ControlMode == CONTROL_PEND_SF)
        {
            Display.Goto(0, 21);
            Display.WriteString("PEND SF", LCD_FONT_SMALL, LCD_PIXEL_XOR);
        }

        else if (ControlMode == CONTROL_FULL_LQR)
        {
            Display.Goto(0, 18);
            Display.WriteString("FULL LQR", LCD_FONT_SMALL, LCD_PIXEL_XOR);
        }


        float NumberToWrite = 0;

        Display.Goto(2, 0);
        Display.WriteString("xR:", LCD_FONT_SMALL, LCD_PIXEL_ON);
        NumberToWrite = ControllerFullLqr.GetReference(0);
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

        // Shadow reference mark
        xP = (uint8_t)Aux::Map (PosRefShadow, -X_VALUE_ABS_M, X_VALUE_ABS_M, 0, (PCD8544_COLUMNS - 1));
        Display.DrawPixel(xP, 12, LCD_PIXEL_XOR);
        Display.DrawPixel(xP, 13, LCD_PIXEL_OFF);

        // Reference mark
        xP = (uint8_t)Aux::Map (ControllerFullLqr.GetReference(0), -X_VALUE_ABS_M, X_VALUE_ABS_M, 0, (PCD8544_COLUMNS - 1));
        Display.DrawPixel(xP, 14, LCD_PIXEL_XOR);
        Display.DrawPixel(xP, 13, LCD_PIXEL_OFF);
    }

    // Commit changes
    Display.Commit();
}

// ------------------------------------------------------------------------------------------------------- //
// RGB update
// ------------------------------------------------------------------------------------------------------- //

void DeviceUpdateRgb()
{
    // Calibration mode
    if ((CalX.Status == CAL_RUNNING) || (CalT.Status == CAL_RUNNING))
        Led.SetColor(RGB_RED, 0);

    // Run mode
    else
    {
        if (Aux::FastFabs(ControllerFullLqr.GetReference(0) - Cart.Pos) < 0.001)
            Led.SetColor(RGB_GREEN, 50);
        else
            Led.SetColor(RGB_YELLOW, 50);
    }
}

// ------------------------------------------------------------------------------------------------------- //
// Button 1 event handler
// ------------------------------------------------------------------------------------------------------- //

void DeviceUpdateButton1 ()
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
                PosRefShadow -= 0.025;

                if (PosRefShadow < -0.2)
                PosRefShadow = -0.2;
            }

            // Double click
            else if (EventData.Counter == 2)
                ControlSetPosReference();
        }

        // Button long clicks
        else if (EventData.EventCode == BUTTON_LONG_CLICK)
        {
            // Single click
            if (EventData.Counter == 1)
            {
                if (ControlMode == CONTROL_OFF)
                    ControlMode = (control_mode_t)(sizeof_control_mode_t - 1);
                else
                    ControlMode = (control_mode_t)(ControlMode - 1);
            }
        }

        // Button long click ticks
        else if (EventData.EventCode == BUTTON_LONG_CLICK_TICK)
        {
            Led.SetColor(RGB_WHITE, 0);
        }
    }
}

// ------------------------------------------------------------------------------------------------------- //
// Button 2 event handler
// ------------------------------------------------------------------------------------------------------- //

void DeviceUpdateButton2 ()
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
                PosRefShadow += 0.025;

                if (PosRefShadow > 0.2)
                    PosRefShadow = 0.2;
            }

            // Double click
            else if (EventData.Counter == 2)
                ControlSetPosReference();
        }

        // Button long clicks
        else if (EventData.EventCode == BUTTON_LONG_CLICK)
        {
            // Single click
            if (EventData.Counter == 1)
            {
                if (ControlMode == (sizeof_control_mode_t - 1))
                    ControlMode = CONTROL_OFF;

                else
                    ControlMode = (control_mode_t)(ControlMode + 1);
            }
        }

        // Button long click ticks
        else if (EventData.EventCode == BUTTON_LONG_CLICK_TICK)
        {
            Led.SetColor(RGB_WHITE, 0);
        }
    }
}

// ------------------------------------------------------------------------------------------------------- //
// Control loop
// ------------------------------------------------------------------------------------------------------- //

void DeviceUpdateControl()
{
    static uint16_t StoppedCounter = 0;

    // Control is on
    if (ControlMode != CONTROL_OFF)
    {
        float Unow = 0;

        // Cart PID controller
        if (ControlMode == CONTROL_CART_PID)
            Unow = ControllerCartPid.Compute(Cart.Pos);

        // Cart Lead controller
        else if (ControlMode == CONTROL_CART_LEAD)
            Unow = ControllerCartLead.Compute(Cart.Pos);

        // Cart State feedback controller
        else if (ControlMode == CONTROL_CART_SF)
        {

            ControllerCartSF.SetState(0, Cart.Pos);
            ControllerCartSF.SetState(1, Cart.Vel);

            Unow = ControllerCartSF.Compute();
        }

        // Pendulum State feedback controller
        else if (ControlMode == CONTROL_PEND_SF)
        {

            ControllerPendSF.SetState(0, Pendulum.Pos);
            ControllerPendSF.SetState(1, Pendulum.Vel);

            Unow = ControllerPendSF.Compute();
        }
 

        // Full LQR controller
        else if (ControlMode == CONTROL_FULL_LQR)
        {
            ControllerFullLqr.SetState(0, Cart.Pos);
            ControllerFullLqr.SetState(1, Cart.Vel);
            ControllerFullLqr.SetState(2, Pendulum.Pos);
            ControllerFullLqr.SetState(3, Pendulum.Vel);

            Unow = ControllerFullLqr.Compute();
        }

        // Acceleration control - Cart state feedback
        if (ControlMode == CONTROL_CART_SF)
        {
            float FinalVelocity = (Unow < 0 ? -STEPPER_VEL_MAX : STEPPER_VEL_MAX);
            float Acceleration = Aux::FastFabs(Unow);

            Stepper.Move(FinalVelocity, Acceleration);

            StoppedCounter = 0;
        }

        // Acceleration control - Pendulum state feedback
        if ((ControlMode == CONTROL_PEND_SF) && (Aux::FastFabs(Pendulum.Pos) < 0.1) && (CalT.Status == CAL_DONE))
        {
            float FinalVelocity = (Unow < 0 ? -STEPPER_VEL_MAX : STEPPER_VEL_MAX);
            float Acceleration = Aux::FastFabs(Unow);

            Stepper.Move(FinalVelocity, Acceleration);

            StoppedCounter = 0;
        }


        // Acceleration control - Full LQR
        else if ((ControlMode == CONTROL_FULL_LQR) && (Aux::FastFabs(Pendulum.Pos) < 0.1) && (CalT.Status == CAL_DONE))
        {
            float FinalVelocity = (Unow < 0 ? -STEPPER_VEL_MAX : STEPPER_VEL_MAX);
            float Acceleration = Aux::FastFabs(Unow);

            Stepper.Move(FinalVelocity, Acceleration);

            StoppedCounter = 0;
        }

        // Velocity control - Other controllers
        else if ((Aux::FastFabs(Unow) >= Stepper.GetMinVel()) && (ControlMode != CONTROL_PEND_SF) && (ControlMode != CONTROL_FULL_LQR))
        {
            Stepper.Move(Unow, STEPPER_ACC_MAX);
            StoppedCounter = 0;
        }

        else
            StoppedCounter++;
    }

    // Control is off
    else
        StoppedCounter++;

    // Cart is stopped for more than 1 second. Turn coils off to avoid heating
    if ((StoppedCounter > 5*CONTROL_LOOP_FREQUENCY) && (Stepper.GetEnable()))
    {
        // TODO: Maybe put the driver to sleep avoids vibrations
        Stepper.Stop();
        StoppedCounter = 0;
    }
}

// ------------------------------------------------------------------------------------------------------- //
// Apply PosRefShaddow to controllers
// ------------------------------------------------------------------------------------------------------- //

void ControlSetPosReference (void)
{
    ControllerCartPid.SetReference(PosRefShadow);
    ControllerCartLead.SetReference(PosRefShadow);
    ControllerCartSF.SetReference(0, PosRefShadow);

    ControllerFullLqr.SetReference(0, PosRefShadow);
}

// ------------------------------------------------------------------------------------------------------- //
// SysTick interrupt service routine
// ------------------------------------------------------------------------------------------------------- //

void IsrSysTick ()
{
    static uint32_t ControlCounter = 0;
    static uint32_t UartCounter = 0;
    static uint32_t LcdCounter = 0;
    static uint32_t RgbCounter = 0;
    static uint32_t ButtonCounter = 0;
    static uint32_t EncoderTCounter = 0;
    static uint32_t EncoderXCounter = 0;
    static uint32_t CmdCounter = 0;

    // Pendulum state update
    if (++EncoderTCounter >= ENCODER_T_INTERVAL)
    {
        PendulumUpdateState();
        EncoderTCounter = 0;
    }

    // Cart state update
    if (++EncoderXCounter >= ENCODER_X_INTERVAL)
    {
        CartUpdateState();

        // Check for stalls - Done here because we need updated cart data
        if (Stepper.CheckForStall(Cart.Vel, 0.1, ENCODER_X_FREQUENCY))
            Stepper.Stop();

        EncoderXCounter = 0;
    }

    // Control loop
    if ((++ControlCounter >= CONTROL_INTERVAL) && (CalX.Status == CAL_DONE))
    {
        DeviceUpdateControl();
        ControlCounter = 0;
    }

    // UART update
    if ((++UartCounter >= UART_TX_INTERVAL) && ((CalX.Status == CAL_DONE) && (CalT.Status == CAL_DONE)))
    {
        UartSendStatus();
        UartCounter = 0;
    }

    // Check for UART commands
    if ((++CmdCounter >= UART_TX_INTERVAL) && ((CalX.Status == CAL_DONE) && (CalT.Status == CAL_DONE)))
    {
        UartProcessCommand();
        CmdCounter = 0;
    }

    // LCD update
    if (++LcdCounter >= LCD_INTERVAL)
    {
        DeviceUpdateLcd();
        LcdCounter = 0;
    }

    // RGB update
    if (++RgbCounter >= RGB_INTERVAL)
    {
        DeviceUpdateRgb();
        RgbCounter = 0;
    }

    // Buttons update
    if (++ButtonCounter >= BUTTON_INTERVAL)
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

void main ()
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

    // Rotate image
    //Display.Rotate(LCD_ROT_ON);

    // Turn on backlight
    Display.Backlight(LCD_BKL_ON);

    // --------------------------------------------------------------------------------------------------- //
    // Buttons configuration
    // --------------------------------------------------------------------------------------------------- //

    // Reverse buttons if LCD is rotated
    if (Display.Rotate(LCD_ROT_GET) == LCD_ROT_ON)
    {
        Button1_Config.Hardware.Pin = BUTTON_PIN_2;
        Button2_Config.Hardware.Pin = BUTTON_PIN_1;
    }

    // Initialize button 1
    Button1.Init(&Button1_Config);

    // Initialize button 2
    Button2.Init(&Button2_Config);

    // --------------------------------------------------------------------------------------------------- //
    // Configure UART
    // --------------------------------------------------------------------------------------------------- //

    // Initialize UART
    Serial.Init(&Uart_Config);
    Serial.SetRxCallback(UartRxCallback);

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
    // Configure cart PID controller
    // --------------------------------------------------------------------------------------------------- //

    // Define PID gains, reference and limits
    ControllerCartPid.SetGains(5, 0, 1);
    ControllerCartPid.SetReference(0);
    ControllerCartPid.SetLimits(-STEPPER_VEL_MAX, STEPPER_VEL_MAX);

    // --------------------------------------------------------------------------------------------------- //
    // Configure cart lead controller
    // --------------------------------------------------------------------------------------------------- //

    // Define lead gains, reference and limits
    ControllerCartLead.SetGains(-0.0030615, 0.0108285255, -0.9948);
    ControllerCartLead.SetReference(0);
    ControllerCartLead.SetLimits(-STEPPER_VEL_MAX, STEPPER_VEL_MAX);

    // --------------------------------------------------------------------------------------------------- //
    // Configure cart state feedback controller
    // --------------------------------------------------------------------------------------------------- //

    // Define state feedback gains and reference
    float CartGains[2] = {37.875340, 20.927861};
    float CartRefs[2] = {0, 0};

    // Initialize Cart State Feedback controller
    ControllerCartSF.Init (CartGains, CartRefs, 2, -STEPPER_ACC_MAX, STEPPER_ACC_MAX);

    // --------------------------------------------------------------------------------------------------- //
    // Configure pendulum state feedback controller
    // --------------------------------------------------------------------------------------------------- //

    // Define state feedback gains and reference
    float PendGains[2] = {-67.631036, -7.567172};
    float PendRefs[2] = {0, 0};

    // Initialize Pend State Feedback controller
    ControllerPendSF.Init (PendGains, PendRefs, 2, -STEPPER_ACC_MAX, STEPPER_ACC_MAX);

    // --------------------------------------------------------------------------------------------------- //
    // Configure LQR controller
    // --------------------------------------------------------------------------------------------------- //

    // Define LQR gains and references

    float CartPendGains[4] = {-2, -10, -40, -20};
    //float CartPendGains[4] = {-7, -5, -50, -10};
    float CartPendRefs[4] = {0};

    // Initialize LQR controller
    ControllerFullLqr.Init (CartPendGains, CartPendRefs, 4, -STEPPER_ACC_MAX, STEPPER_ACC_MAX);

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

    Display.DrawFilledRectangle (0, 0, 83, 8, LCD_PIXEL_ON);
    Display.DrawRectangle (0, 8, 83, 16, LCD_PIXEL_ON);

    Display.Goto(0, 18);
    Display.WriteString(DEVICE_FW_NAME, LCD_FONT_SMALL, LCD_PIXEL_XOR);
    Display.Goto(1, 6);
    Display.WriteString(DEVICE_FW_AUTHOR, LCD_FONT_SMALL, LCD_PIXEL_ON);

    Display.Goto(3, 0);
    Display.WriteString("Version: ", LCD_FONT_SMALL, LCD_PIXEL_ON);
    Display.WriteString(DEVICE_FW_VERSION, LCD_FONT_SMALL, LCD_PIXEL_ON);

    Display.Goto(4, 0);
    Display.WriteString(__DATE__ "\0", LCD_FONT_SMALL, LCD_PIXEL_ON);
    Display.Goto(5, 0);
    Display.WriteString(__TIME__ "\0", LCD_FONT_SMALL, LCD_PIXEL_ON);

    Display.Commit();

    // --------------------------------------------------------------------------------------------------- //
    // Send firmware info to serial port
    // --------------------------------------------------------------------------------------------------- //

    Serial.SendString("[INFO]");
    Serial.SendString (DEVICE_FW_NAME);
    Serial.SendString (" - Version: ");
    Serial.SendString (DEVICE_FW_VERSION);
    Serial.SendString (" - Date: ");
    Serial.SendString (__DATE__ "\0");
    Serial.SendString (" " __TIME__ "\0");
    Serial.SendString (" - Author: ");
    Serial.SendString (DEVICE_FW_AUTHOR);
    Serial.SendString ("\n");

    SysCtlDelay(30000000);

    // --------------------------------------------------------------------------------------------------- //
    // Start application state machine
    // --------------------------------------------------------------------------------------------------- //

    // Start timer
    SysTickEnable ();

    // Home cart
    CalibrationCartGoHome ();

    // Calibrate cart X position limits
    //CalibrationCartPos ();

    // Calibrate cart velocity constant - Must be called with the pendulum detached
    //CalibrationCartVel();

    // Calibrate motor velocity constant - Must be called with the pendulum detached
    //CalibrationMotorVel();

    // Use controller to go to X = 0
    ControlMode = CONTROL_CART_PID;
    while (Aux::FastFabs(Cart.Pos) > 0.0001);
    ControlMode = CONTROL_OFF;

    // Calibrate pendulum angle
    CalibrationPendulumPos ();

    // --------------------------------------------------------------------------------------------------- //
    // Main loop
    // --------------------------------------------------------------------------------------------------- //

    SysCtlDelay(10000000);
    while (1)
    {
        static float MoveVel = STEPPER_VEL_CAL;

        // Start stepper - Backwards direction
        Stepper.Move (-MoveVel, STEPPER_ACC_MAX);

        while ((Cart.Pos > -0.05) && Stepper.GetEnable());

        Stepper.Move (0, STEPPER_ACC_MAX);
        while (Stepper.GetCurrentVel () != 0);

        // Start stepper
        Stepper.Move (MoveVel, STEPPER_ACC_MAX);

        while ((Cart.Pos < 0.05) && Stepper.GetEnable());

        Stepper.Move (0, STEPPER_ACC_MAX);
        while (Stepper.GetCurrentVel () != 0);

        MoveVel += 0.25;
        if (MoveVel > STEPPER_VEL_MAX)
            //MoveVel = STEPPER_VEL_CAL;
            while (1);
    }
}

// ------------------------------------------------------------------------------------------------------- //
// End of code
// ------------------------------------------------------------------------------------------------------- //

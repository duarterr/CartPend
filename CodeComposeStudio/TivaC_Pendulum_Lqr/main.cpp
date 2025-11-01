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

// Move cart to specific position - Must be called AFTER CalibrationCartGoHome
void CalibrationCartGoPosition (float Position);

// Calibrate cart X position limits
void CalibrationCartPos ();

// Calibrate cart Kv - Must be called AFTER CalibrationCartPos
void CalibrationCartVel ();

// Calibrate motor Kv - Must be called AFTER CalibrationCartPos
void CalibrationMotorVel ();

// Calibrate pendulum angle
void CalibrationPendulumPos ();

// UART RX callback
void UartRxCallback(char received_char);

// Process received UART command
void UartProcessCommand();

// Send firmware info to serial port
void UartSendInfo();

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

// Get PosRef from controllers
float ControlGetPosReference (void);

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

// JSON parser object - From Json_Parser class
JsonParser Parser;

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
StateFeedback ControllerCartLqr;

// Pendulum state feedback controller object - From StateFeedback_TivaC class
StateFeedback ControllerPendSF;

// Cart and pendulum full state feedback controller object - From StateFeedback_TivaC class
StateFeedback ControllerCartPendLqr;

// ------------------------------------------------------------------------------------------------------- //
// Variables
// ------------------------------------------------------------------------------------------------------- //

// Cart-related variables
volatile cart_t CartRaw = cart_t_default;
volatile cart_t CartFiltered = cart_t_default;

// Pendulum-related variables
volatile pendulum_t PendulumRaw = pendulum_t_default;
volatile pendulum_t PendulumFiltered = pendulum_t_default;

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

// Calibration data
volatile calibration_master_t Cal = calibration_master_t_default;

// ------------------------------------------------------------------------------------------------------- //
// Update cart state based on encoder readings
// ------------------------------------------------------------------------------------------------------- //

void CartUpdateState()
{
    // Cached scale factors to avoid unnecessary recalculations
    static float PosScale = 0.0f;       // Position scaling factor [m/count]
    static float VelScale = 0.0f;       // Velocity scaling factor [m/s per raw unit]
    static uint32_t LastCalMax = 0;     // Last known calibration max value
    static float LastCalKv = 0.0f;      // Last known velocity calibration constant

    // Check if calibration parameters have changed
    if ((Cal.CartPos.Max != LastCalMax) || (Cal.CartPos.Kv != LastCalKv))
    {
        // Recalculate scaling factors based on current calibration
        PosScale = (float)X_VALUE_TOTAL_M / Cal.CartPos.Max;
        VelScale = 1.0f / Cal.CartPos.Kv;
        LastCalMax = Cal.CartPos.Max;
        LastCalKv = Cal.CartPos.Kv;
    }

    // Compute cart position
    CartRaw.Pos = (PosScale * ((int32_t)EncoderX.GetPos() - Cal.CartPos.Offset)) - X_VALUE_ABS_M;

    // Compute cart velocity
    CartRaw.Vel = (VelScale * EncoderX.GetVel()) * EncoderX.GetDir();

    // Moving average filter - Decimation from ENCODER_T_FREQUENCY to CONTROL_LOOP_FREQUENCY

    // Calculate decimation factor (ENCODER_X_FREQUENCY / CONTROL_LOOP_FREQUENCY)
    static const uint8_t DECIMATION_FACTOR = ENCODER_X_FREQUENCY / CONTROL_LOOP_FREQUENCY;
    static const float INV_DECIMATION = 1.0f / DECIMATION_FACTOR;

    static uint8_t SampleCounter = 0;
    static float PosSum = 0.0f;
    static float VelSum = 0.0f;

    // Accumulate position and velocity
    PosSum += CartRaw.Pos;
    VelSum += CartRaw.Vel;

    // Every DECIMATION_FACTOR samples, compute average
    if (++SampleCounter >= DECIMATION_FACTOR)
    {
        CartFiltered.Pos = PosSum * INV_DECIMATION;
        CartFiltered.Vel = VelSum * INV_DECIMATION;

        // Reset accumulators
        PosSum = 0.0f;
        VelSum = 0.0f;
        SampleCounter = 0;
    }
}

// ------------------------------------------------------------------------------------------------------- //
// Update pendulum state based on encoder readings
// ------------------------------------------------------------------------------------------------------- //

void PendulumUpdateState()
{
    // Cached scale factors to avoid unnecessary recalculations
    static float PosScale = 0.0f;       // Position scaling factor [rad/count]
    static float VelScale = 0.0f;       // Velocity scaling factor [rad/s per raw unit]
    static float LastCalMax = 0.0f;     // Last known calibration max value

    // Check if calibration parameters have changed
    if (Cal.Theta.Max != LastCalMax)
    {
        // Recalculate scaling factors based on current calibration
        PosScale = (float)T_VALUE_MAX_RAD / Cal.Theta.Max;
        VelScale = PosScale * ENCODER_T_FREQUENCY;
        LastCalMax = Cal.Theta.Max;
    }

    // Position
    PendulumRaw.Pos = (PosScale * ((int32_t)EncoderT.GetPos() - Cal.Theta.Offset)) - T_VALUE_ABS_MAX;

    // Velocity
    PendulumRaw.Vel = (VelScale * EncoderT.GetVel()) * EncoderT.GetDir();


    // Moving average filter - Decimation from ENCODER_T_FREQUENCY to CONTROL_LOOP_FREQUENCY

    // Calculate decimation factor
    static const uint8_t DECIMATION_FACTOR = ENCODER_T_FREQUENCY / CONTROL_LOOP_FREQUENCY;
    static const float INV_DECIMATION = 1.0f / DECIMATION_FACTOR;

    static uint8_t SampleCounter = 0;
    static float PosSum = 0.0f;
    static float VelSum = 0.0f;
    static float LastUnwrappedPos = 0.0f;
    static bool FirstSample = true;

    // Unwrap current position to handle angle wrapping
    float UnwrappedPos = PendulumRaw.Pos;

    if (!FirstSample)
    {
        float diff = UnwrappedPos - LastUnwrappedPos;

        // Correct wrap-around discontinuities
        if (diff > M_PI)
            UnwrappedPos -= T_VALUE_MAX_RAD;
        else if (diff < -M_PI)
            UnwrappedPos += T_VALUE_MAX_RAD;
    }
    else
        FirstSample = false;

    LastUnwrappedPos = UnwrappedPos;

    // Accumulate unwrapped values
    PosSum += UnwrappedPos;
    VelSum += PendulumRaw.Vel;

    // Every DECIMATION_FACTOR samples, compute average and wrap back
    if (++SampleCounter >= DECIMATION_FACTOR)
    {
        float PosAvg = PosSum * INV_DECIMATION;

        // Wrap back to [-π, π]
        while (PosAvg > M_PI) PosAvg -= T_VALUE_MAX_RAD;
        while (PosAvg < -M_PI) PosAvg += T_VALUE_MAX_RAD;

        PendulumFiltered.Pos = PosAvg;
        PendulumFiltered.Vel = VelSum * INV_DECIMATION;

        // Reset accumulators
        PosSum = 0.0f;
        VelSum = 0.0f;
        SampleCounter = 0;
    }
}

// ------------------------------------------------------------------------------------------------------- //
// Update flags function
// ------------------------------------------------------------------------------------------------------- //

void CalibrationUpdateFlags()
{
    Cal.AllDone = (Cal.Theta.Status == CAL_DONE) && (Cal.CartPos.Status == CAL_DONE);
    Cal.AnyRunning = (Cal.Theta.Status == CAL_RUNNING) || (Cal.CartPos.Status == CAL_RUNNING) ||
                     (Cal.CartVel.Status == CAL_RUNNING) || (Cal.MotorVel.Status == CAL_RUNNING);
}

// ------------------------------------------------------------------------------------------------------- //
// Move cart to home position (-X_VALUE_ABS_M)
// ------------------------------------------------------------------------------------------------------- //

void CalibrationCartGoHome ()
{
    // Set calibration variables
    Cal.CartPos.Status = CAL_RUNNING;
    Cal.CartPos.Progress = 0;
    CalibrationUpdateFlags();

    // Start stepper - Backwards direction
    Stepper.Move (-STEPPER_VEL_CAL, 0.25);

    // Move until home switch triggers
    while (Stepper.GetEnable());

    // Set encoder counter - Add offset because cart can move past limit switch trigger point
    EncoderX.SetPos(Cal.CartPos.Offset);

    // Set calibration variables
    Cal.CartPos.Status = CAL_DONE;
    Cal.CartPos.Progress = 100;
    CalibrationUpdateFlags();
}

// ------------------------------------------------------------------------------------------------------- //
// Move cart to specific position - Must be called AFTER CalibrationCartGoHome
// ------------------------------------------------------------------------------------------------------- //

void CalibrationCartGoPosition (float Position)
{
    // Set calibration variables
    Cal.CartPos.Status = CAL_RUNNING;
    Cal.CartPos.Progress = 0;
    CalibrationUpdateFlags();

    float StartPos = CartFiltered.Pos;
    float DeltaPos = CartFiltered.Pos - Position;
    float TotalDistance = Aux::FastFabs(DeltaPos);

    // Start stepper
    if (DeltaPos > 0)
    {
        Stepper.Move(-STEPPER_VEL_CAL, 0.25);
        while ((CartFiltered.Pos > Position) && Stepper.GetEnable())
        {
            float Traveled = Aux::FastFabs(CartFiltered.Pos - StartPos);
            Cal.CartPos.Progress = (Traveled / TotalDistance) * 100;
        }
    }
    else if (DeltaPos < 0)
    {
        Stepper.Move(STEPPER_VEL_CAL, 0.25);
        while ((CartFiltered.Pos < Position) && Stepper.GetEnable())
        {
            float Traveled = Aux::FastFabs(CartFiltered.Pos - StartPos);
            Cal.CartPos.Progress = (Traveled / TotalDistance) * 100;
        }
    }

    // Stop stepper
    Stepper.Stop();

    // Set calibration variables
    Cal.CartPos.Status = CAL_DONE;
    Cal.CartPos.Progress = 100;
    CalibrationUpdateFlags();
}

// ------------------------------------------------------------------------------------------------------- //
// Calibrate cart X position limits
// ------------------------------------------------------------------------------------------------------- //

void CalibrationCartPos ()
{
    // Set calibration variables
    Cal.CartPos.Status = CAL_RUNNING;
    Cal.CartPos.Progress = 0;
    CalibrationUpdateFlags();

    // Start stepper - Backwards direction
    Stepper.Move (-STEPPER_VEL_CAL, 0.25);

    // Move until home switch triggers
    while (Stepper.GetEnable());

    // Set encoder counter - Add offset because cart can move past limit switch trigger point
    EncoderX.SetPos(Cal.CartPos.Offset);

    // Start stepper - Forward direction
    Stepper.Move (STEPPER_VEL_CAL, 0.25);

    // Move until end switch triggers
    while (Stepper.GetEnable())
        Cal.CartPos.Progress = (uint32_t)(EncoderX.GetPos() * 100)/ENCODER_X_MAX;

    // Define encoder maximum counter value - Without offset
    Cal.CartPos.Max = EncoderX.GetPos() - Cal.CartPos.Offset;

    // Set calibration variables
    Cal.CartPos.Status = CAL_DONE;
    CalibrationUpdateFlags();
}

// ------------------------------------------------------------------------------------------------------- //
// Calibrate cart Kv - Must be called with the pendulum detached and AFTER CalibrationCartPos
// ------------------------------------------------------------------------------------------------------- //

void CalibrationCartVel ()
{
    float KvEncoder[CAL_KV_TOTAL_STEPS] = {};

    // Set calibration variables
    Cal.CartVel.Status = CAL_RUNNING;
    Cal.CartVel.Progress = 0;
    CalibrationUpdateFlags();

    for (uint8_t IterationCounter = 0; IterationCounter < CAL_KV_ITERATIONS; IterationCounter++)
    {
        // Initial homing - Forward direction to end switch
        Stepper.Move (STEPPER_VEL_CAL, 0.25);
        while (Stepper.GetEnable());

        // Start stepper - Backward direction
        Stepper.Move (-CAL_KV_VELOCITIES[IterationCounter], STEPPER_ACC_MAX);

        // Wait for speed to stabilise
        while (Stepper.GetCurrentVel() != Stepper.GetTargetVel());

        // Capture start values
        uint32_t PosStart = EncoderX.GetPos();
        uint32_t TickStart = SysTickCounter;
        uint32_t EncoderVelSum = 0;
        uint32_t SampleCounter = 0;

        // Collect samples until end position is reached
        while ((CartFiltered.Pos > -CAL_KV_END_POSITIONS[IterationCounter]) && (Stepper.GetEnable()))
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
            float DeltaPos = (float)(PosStart - PosEnd) * X_VALUE_TOTAL_M / Cal.CartPos.Max;
            float DeltaT = (float)(TickEnd - TickStart) / TIMER_FREQUENCY;
            float RealVel = DeltaPos / DeltaT;
            KvEncoder[IterationCounter * 2] = AverageEncoderVel / RealVel;
        }

        Cal.CartVel.Progress = (uint8_t)(((IterationCounter * 2 + 1) * 100) / CAL_KV_TOTAL_STEPS);

        // Move slowly to backward end switch
        Stepper.Move (-STEPPER_VEL_CAL, 0.25);
        while (Stepper.GetEnable());

        // Start stepper - Forward direction
        Stepper.Move (CAL_KV_VELOCITIES[IterationCounter], STEPPER_ACC_MAX);

        // Wait for speed to stabilise
        while (Stepper.GetCurrentVel() != Stepper.GetTargetVel());

        // Capture start values
        PosStart = EncoderX.GetPos();
        TickStart = SysTickCounter;
        EncoderVelSum = 0;
        SampleCounter = 0;

        // Collect samples until end position is reached
        while ((CartFiltered.Pos < CAL_KV_END_POSITIONS[IterationCounter]) && (Stepper.GetEnable()))
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
            float DeltaPos = (float)(PosEnd - PosStart) * X_VALUE_TOTAL_M / Cal.CartPos.Max;
            float DeltaT = (float)(TickEnd - TickStart) / TIMER_FREQUENCY;
            float RealVel = DeltaPos / DeltaT;
            KvEncoder[IterationCounter * 2 + 1] = AverageEncoderVel / RealVel;
        }

        Cal.CartVel.Progress = (uint8_t)(((IterationCounter * 2 + 2) * 100) / CAL_KV_TOTAL_STEPS);
    }

    // Calculate average Kv
    float KvEncoderSum = 0;
    for (uint8_t StepCounter = 0; StepCounter < CAL_KV_TOTAL_STEPS; StepCounter++)
        KvEncoderSum += KvEncoder[StepCounter];
    Cal.CartVel.Kv = KvEncoderSum / CAL_KV_TOTAL_STEPS;
    Cal.CartPos.Kv = Cal.CartVel.Kv;

    // Set calibration variables
    Cal.CartVel.Status = CAL_DONE;
    Cal.CartVel.Progress = 100;
    CalibrationUpdateFlags();
}

// ------------------------------------------------------------------------------------------------------- //
// Calibrate motor Kv - Must be called with the pendulum detached and AFTER CalibrationCartPos
// ------------------------------------------------------------------------------------------------------- //

void CalibrationMotorVel ()
{
    float StepperKv[CAL_KV_TOTAL_STEPS] = {};

    // Set calibration variables
    Cal.MotorVel.Status = CAL_RUNNING;
    Cal.MotorVel.Progress = 0;
    CalibrationUpdateFlags();

    for (uint8_t IterationCounter = 0; IterationCounter < CAL_KV_ITERATIONS; IterationCounter++)
    {
        // Initial homing - Forward direction to end switch
        Stepper.Move (STEPPER_VEL_CAL, 0.25);
        while (Stepper.GetEnable());

        // Start stepper - Backward direction at test velocity
        Stepper.Move (-CAL_KV_VELOCITIES[IterationCounter], STEPPER_ACC_MAX);

        // Wait for speed to stabilise
        while (Stepper.GetCurrentVel() != Stepper.GetTargetVel());

        uint32_t EncoderVelSum = 0;
        uint64_t PwmSum = 0;
        uint32_t SampleCounter = 0;

        // Collect samples until end position is reached
        while ((CartFiltered.Pos > -CAL_KV_END_POSITIONS[IterationCounter]) && (Stepper.GetEnable()))
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
            float AverageVel = ((float)EncoderVelSum / SampleCounter) / Cal.CartPos.Kv;
            float AveragePwm = (float)PwmSum / SampleCounter;
            StepperKv[IterationCounter * 2] = AveragePwm / AverageVel;
        }

        Cal.MotorVel.Progress = (uint8_t)(((IterationCounter * 2 + 1) * 100) / CAL_KV_TOTAL_STEPS);

        // Move slowly to backward end switch
        Stepper.Move (-STEPPER_VEL_CAL, 0.25);
        while (Stepper.GetEnable());

        // Start stepper - Forward direction at test velocity
        Stepper.Move (CAL_KV_VELOCITIES[IterationCounter], STEPPER_ACC_MAX);

        // Wait for speed to stabilise
        while (Stepper.GetCurrentVel() != Stepper.GetTargetVel());

        EncoderVelSum = 0;
        PwmSum = 0;
        SampleCounter = 0;

        // Collect samples until end position is reached
        while ((CartFiltered.Pos < CAL_KV_END_POSITIONS[IterationCounter]) && (Stepper.GetEnable()))
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
            float AverageVel = ((float)EncoderVelSum / SampleCounter) / Cal.CartPos.Kv;
            float AveragePwm = (float)PwmSum / SampleCounter;
            StepperKv[IterationCounter * 2 + 1] = AveragePwm / AverageVel;
        }

        Cal.MotorVel.Progress = (uint8_t)(((IterationCounter * 2 + 2) * 100) / CAL_KV_TOTAL_STEPS);
    }

    // Calculate average Kv
    float StepperKvSum = 0;
    for (uint8_t StepCounter = 0; StepCounter < CAL_KV_TOTAL_STEPS; StepCounter++)
        StepperKvSum += StepperKv[StepCounter];
    Cal.MotorVel.Kv = StepperKvSum / CAL_KV_TOTAL_STEPS;

    // Update motor Kv parameter
    Stepper.SetKv(Cal.MotorVel.Kv);

    // Set calibration variables
    Cal.MotorVel.Status = CAL_DONE;
    Cal.MotorVel.Progress = 100;
    CalibrationUpdateFlags();
}

// ------------------------------------------------------------------------------------------------------- //
// Calibrate pendulum angle
// ------------------------------------------------------------------------------------------------------- //

void CalibrationPendulumPos ()
{
    // Aux variables
    uint32_t TimeEven[10] = {};
    uint32_t TimeOdd[10] = {};
    uint32_t PosEven[10] = {};
    uint32_t PosOdd[10] = {};
    float SlopeEven, OffsetEven, SlopeOdd, OffsetOdd;
    int32_t Offset = 0;

    // Set calibration variables
    Cal.Theta.Progress = 0;
    Cal.Theta.Status = CAL_RUNNING;
    CalibrationUpdateFlags();

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
        Cal.Theta.Progress += (50/ENCODER_T_CAL_CYCLES);

        // Wait for direction change
        while (EncoderT.GetDir() == DirectionLast);
        DirectionLast = EncoderT.GetDir();

        // Save time and position
        PosOdd[Idx] = EncoderT.GetPos();;
        TimeOdd[Idx] = SysTickCounter;

        // Increase progress
        Cal.Theta.Progress += (50/ENCODER_T_CAL_CYCLES);
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
    Cal.Theta.Max = ENCODER_T_PPR;

    // Set calibration variables
    Cal.Theta.Status = CAL_DONE;
    Cal.Theta.Progress = 100;
    CalibrationUpdateFlags();
}

// ------------------------------------------------------------------------------------------------------- //
// UART RX callback
// ------------------------------------------------------------------------------------------------------- //

void UartRxCallback(char received_char)
{
    // Only store character if buffer not full and not processing
    if (!UartCommandReady && UartRxIndex < sizeof(UartRxBuffer) - 1)
    {
        UartRxBuffer[UartRxIndex++] = received_char;

        // Check for complete command
        if (received_char == '}' && UartRxBuffer[0] == '{')
        {
            UartRxBuffer[UartRxIndex] = '\0';
            UartCommandReady = true;
        }
        else if (received_char == '{')
        {
            // New command starting - reset
            UartRxIndex = 1;
            UartRxBuffer[0] = '{';
        }
    }
}

// ------------------------------------------------------------------------------------------------------- //
// Process received UART command - JSON FORMAT
// ------------------------------------------------------------------------------------------------------- //

void UartProcessCommand()
{
    if (!UartCommandReady)
        return;

    char* cmd_str = (char*)UartRxBuffer;
    static char json_response[256];
    static JsonBuilder response(json_response, sizeof(json_response));

    // Reset builder for new response
    response.reset();

    // Parse received JSON
    Parser.setJson(cmd_str);

    // Start response object
    response.startObject();

    // Check if JSON is valid
    if (!Parser.isValid())
    {
        response.addString("STATUS", "ERROR");
        response.addString("MSG", "INVALID_JSON");
    }

    else
    {
        // Extract command field
        char command[32];
        if (!Parser.getString("CMD", command, sizeof(command)))
        {
            response.addString("STATUS", "ERROR");
            response.addString("MSG", "MISSING_CMD_FIELD");
        }
        // MODE command - Change control mode
        else if (strcmp(command, "MODE") == 0)
        {
            int32_t mode;
            if (Parser.getInt("VALUE", &mode) && mode >= 0 && mode < sizeof_control_mode_t)
            {
                ControlMode = (control_mode_t)mode;
                response.addString("STATUS", "OK");
                response.addString("MSG", "MODE_CHANGED");
                response.addInt("VALUE", mode);
            }
            else
            {
                response.addString("STATUS", "ERROR");
                response.addString("MSG", "INVALID_MODE");
            }
        }
        // SREF command - Set shadow reference
        else if (strcmp(command, "SREF") == 0)
        {
            float ref;
            if (Parser.getFloat("VALUE", &ref) && ref >= -0.2 && ref <= 0.2)
            {
                PosRefShadow = ref;
                response.addString("STATUS", "OK");
                response.addString("MSG", "SREF_SET");
                response.addFloat("VALUE", ref, 6);
            }
            else
            {
                response.addString("STATUS", "ERROR");
                response.addString("MSG", "SREF_OUT_OF_RANGE");
            }
        }
        // AREF command - Apply reference
        else if (strcmp(command, "AREF") == 0)
        {
            ControlSetPosReference();
            response.addString("STATUS", "OK");
            response.addString("MSG", "REF_APPLIED");
            response.addFloat("VALUE", ControlGetPosReference(), 6);
        }
        // Unknown command
        else
        {
            response.addString("STATUS", "ERROR");
            response.addString("MSG", "UNKNOWN_COMMAND");
            response.addString("CMD", command);
        }
    }

    // End response object and send
    response.endObject();
    Serial.SendString(response.getBuffer());
    Serial.SendString("\n");

    // Cleanup - Clear buffer and reset flag
    UartCommandReady = false;
    UartRxIndex = 0;
    memset((void*)UartRxBuffer, 0, sizeof(UartRxBuffer));
}

// ------------------------------------------------------------------------------------------------------- //
// Send firmware info to serial port
// ------------------------------------------------------------------------------------------------------- //

void UartSendInfo()
{
    char json_buffer[256];
    JsonBuilder json(json_buffer, sizeof(json_buffer));

    json.startObject();
    json.addString("TYPE", "INFO");
    json.addString("NAME", DEVICE_FW_NAME);
    json.addString("VERSION", DEVICE_FW_VERSION);
    json.addString("AUTHOR", DEVICE_FW_AUTHOR);
    json.addString("DATE", __DATE__);
    json.addString("TIME", __TIME__);
    json.endObject();

    Serial.SendString(json.getBuffer());
    Serial.SendString("\n");
}

// ------------------------------------------------------------------------------------------------------- //
// Send device status via UART
// ------------------------------------------------------------------------------------------------------- //

void UartSendStatus()
{
    static char json_buffer[256];
    JsonBuilder json(json_buffer, sizeof(json_buffer));

    json.startObject();

    // Timestamp
    json.addUInt("TS", SysTickCounter);

    // Control mode
    json.addInt("MODE", (int32_t)ControlMode);

    // Motor data
    json.addFloat("MTV", Stepper.GetTargetVel(), 4);
    json.addFloat("MCV", Stepper.GetCurrentVel(), 4);
    json.addInt("MPWM", (Stepper.GetDir() == 1 ? Stepper.GetPwmFrequency() : -Stepper.GetPwmFrequency()));

    // Cart data
    json.addFloat("XP", CartRaw.Pos, 6);
    json.addFloat("XV", CartRaw.Vel, 6);
    json.addFloat("XPF", CartFiltered.Pos, 6);
    json.addFloat("XVF", CartFiltered.Vel, 6);

    // Pendulum data
    json.addFloat("PP", PendulumRaw.Pos, 6);
    json.addFloat("PV", PendulumRaw.Vel, 6);
    json.addFloat("PPF", PendulumFiltered.Pos, 6);
    json.addFloat("PVF", PendulumFiltered.Vel, 6);

    // References
    json.addFloat("RP", ControlGetPosReference(), 6);
    json.addFloat("RS", PosRefShadow, 6);

    json.endObject();

    // Send JSON
    Serial.SendString(json.getBuffer());
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
    if (Cal.AnyRunning)
    {
        Display.DrawRectangle (0, 8, 83, 16, LCD_PIXEL_ON);
        Display.DrawRectangle (0, 38, 83, 47, LCD_PIXEL_ON);

        Display.Goto(0, 9);
        Display.WriteString("CALIBRATION", LCD_FONT_SMALL, LCD_PIXEL_XOR);

        if (Cal.CartPos.Status == CAL_RUNNING)
        {
            Display.Goto(1, 24);
            Display.WriteString("X-axis", LCD_FONT_SMALL, LCD_PIXEL_XOR);

            if (Cal.CartPos.Progress != 0)
            {
                Display.Goto(3, 0);
                Display.WriteString("Calibrating...", LCD_FONT_SMALL, LCD_PIXEL_ON);

                Display.DrawFilledRectangle(0, 38, (Cal.CartPos.Progress * PCD8544_COLUMNS / 100), 47, LCD_PIXEL_ON);
            }
            else
            {
                Display.Goto(3, 15);
                Display.WriteString("Homing...", LCD_FONT_SMALL, LCD_PIXEL_ON);
            }
        }

        else if (Cal.Theta.Status == CAL_RUNNING)
        {
            Display.Goto(1, 12);
            Display.WriteString("Theta-axis", LCD_FONT_SMALL, LCD_PIXEL_XOR);

            Display.Goto(3, 0);
            Display.WriteString("Calibrating...", LCD_FONT_SMALL, LCD_PIXEL_ON);

            if (Cal.Theta.Progress != 0)
                Display.DrawFilledRectangle(0, 38, (Cal.Theta.Progress * PCD8544_COLUMNS / 100), 47, LCD_PIXEL_ON);
        }

        else if (Cal.CartVel.Status == CAL_RUNNING)
        {
            Display.Goto(1, 18);
            Display.WriteString("Cart Vel", LCD_FONT_SMALL, LCD_PIXEL_XOR);

            Display.Goto(3, 0);
            Display.WriteString("Calibrating...", LCD_FONT_SMALL, LCD_PIXEL_ON);

            if (Cal.CartVel.Progress != 0)
                Display.DrawFilledRectangle(0, 38, (Cal.CartVel.Progress * PCD8544_COLUMNS / 100), 47, LCD_PIXEL_ON);
        }

        else if (Cal.MotorVel.Status == CAL_RUNNING)
        {
            Display.Goto(1, 15);
            Display.WriteString("Motor Vel", LCD_FONT_SMALL, LCD_PIXEL_XOR);

            Display.Goto(3, 0);
            Display.WriteString("Calibrating...", LCD_FONT_SMALL, LCD_PIXEL_ON);

            if (Cal.MotorVel.Progress != 0)
                Display.DrawFilledRectangle(0, 38, (Cal.MotorVel.Progress * PCD8544_COLUMNS / 100), 47, LCD_PIXEL_ON);
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

        else if (ControlMode == CONTROL_CART_LQR)
        {
            Display.Goto(0, 18);
            Display.WriteString("CART LQR", LCD_FONT_SMALL, LCD_PIXEL_XOR);
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
        NumberToWrite = ControlGetPosReference();
        if (NumberToWrite >= 0)
            Display.WriteChar(' ', LCD_FONT_SMALL, LCD_PIXEL_ON);
        Display.WriteFloat(NumberToWrite, 6, LCD_FONT_SMALL, LCD_PIXEL_ON);
        Display.Goto(Display.GetBank(), PCD8544_COLUMNS - 6);
        Display.WriteString("m", LCD_FONT_SMALL, LCD_PIXEL_ON);

        Display.Goto(3, 0);
        Display.WriteString("xP:", LCD_FONT_SMALL, LCD_PIXEL_ON);
        NumberToWrite = CartFiltered.Pos;
        if (NumberToWrite >= 0)
            Display.WriteChar(' ', LCD_FONT_SMALL, LCD_PIXEL_ON);
        Display.WriteFloat(NumberToWrite, 6, LCD_FONT_SMALL, LCD_PIXEL_ON);
        Display.Goto(Display.GetBank(), PCD8544_COLUMNS - 6);
        Display.WriteString("m", LCD_FONT_SMALL, LCD_PIXEL_ON);

        Display.Goto(4, 0);
        Display.WriteString("xV:", LCD_FONT_SMALL, LCD_PIXEL_ON);
        NumberToWrite = CartFiltered.Vel;
        if (NumberToWrite >= 0)
            Display.WriteChar(' ', LCD_FONT_SMALL, LCD_PIXEL_ON);
        Display.WriteFloat(NumberToWrite, 4, LCD_FONT_SMALL, LCD_PIXEL_ON);
        Display.Goto(Display.GetBank(), PCD8544_COLUMNS - 18);
        Display.WriteString("m/s", LCD_FONT_SMALL, LCD_PIXEL_ON);

        Display.Goto(5, 0);
        Display.WriteString("tP:", LCD_FONT_SMALL, LCD_PIXEL_ON);
        NumberToWrite = PendulumFiltered.Pos;
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
        uint8_t xP = (uint8_t)Aux::Map (CartFiltered.Pos, -X_VALUE_ABS_M, X_VALUE_ABS_M, 0, (PCD8544_COLUMNS - 1));
        Display.DrawFilledRectangle(xP - 2, 11, xP + 2, 15, LCD_PIXEL_ON);

        // Shadow reference mark
        xP = (uint8_t)Aux::Map (PosRefShadow, -X_VALUE_ABS_M, X_VALUE_ABS_M, 0, (PCD8544_COLUMNS - 1));
        Display.DrawPixel(xP, 12, LCD_PIXEL_XOR);
        Display.DrawPixel(xP, 13, LCD_PIXEL_OFF);

        // Reference mark
        xP = (uint8_t)Aux::Map (ControlGetPosReference(), -X_VALUE_ABS_M, X_VALUE_ABS_M, 0, (PCD8544_COLUMNS - 1));
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
    if (Cal.AnyRunning)
        Led.SetColor(RGB_RED, 0);

    // Run mode
    else
    {
        if (Aux::FastFabs(ControlGetPosReference() - CartFiltered.Pos) < 0.001)
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
    // Counter to detect when the system has been stopped for too long
    static uint16_t StoppedCounter = 0;

    // Number of control loop iterations equivalent to 1 second
    static const uint16_t STOP_THRESHOLD = 1000 / CONTROL_LOOP_FREQUENCY;

    float Unow = 0.0f;    // Current control output
    bool CanMove = false; // Indicates whether the actuator should move

    // Only process control if the mode is not OFF
    if (ControlMode != CONTROL_OFF)
    {
        // Switch statement is faster than multiple if-else chains
        switch (ControlMode)
        {
            // Cart position PID controller
            case CONTROL_CART_PID:
                Unow = ControllerCartPid.Compute(CartFiltered.Pos);
                CanMove = (Aux::FastFabs(Unow) >= Stepper.GetMinVel());
                break;

            // Cart position lead controller
            case CONTROL_CART_LEAD:
                Unow = ControllerCartLead.Compute(CartFiltered.Pos);
                CanMove = (Aux::FastFabs(Unow) >= Stepper.GetMinVel());
                break;

            // State-feedback controller for cart only
            case CONTROL_CART_LQR:
                ControllerCartLqr.SetState(0, CartFiltered.Pos);
                ControllerCartLqr.SetState(1, CartFiltered.Vel);
                Unow = ControllerCartLqr.Compute();
                CanMove = (Aux::FastFabs(Unow) >= Stepper.GetMinVel());
                break;

            // State-feedback controller for pendulum
            case CONTROL_PEND_SF:
                ControllerPendSF.SetState(0, PendulumFiltered.Pos);
                ControllerPendSF.SetState(1, PendulumFiltered.Vel);
                Unow = ControllerPendSF.Compute();
                CanMove = (Cal.Theta.Status == CAL_DONE)             // Only if calibration done
                          && (Aux::FastFabs(PendulumFiltered.Pos) < 0.1f)   // Pendulum near upright
                          && (Aux::FastFabs(Unow) >= Stepper.GetMinVel());
                break;

            // State-feedback controller for cart and pendulum states
            case CONTROL_FULL_LQR:
                ControllerCartPendLqr.SetState(0, CartFiltered.Pos);
                ControllerCartPendLqr.SetState(1, CartFiltered.Vel);
                ControllerCartPendLqr.SetState(2, PendulumFiltered.Pos);
                ControllerCartPendLqr.SetState(3, PendulumFiltered.Vel);
                Unow = ControllerCartPendLqr.Compute();
                CanMove = (Cal.Theta.Status == CAL_DONE)             // Calibration must be completed
                          && (Aux::FastFabs(PendulumFiltered.Pos) < 0.1f)   // Pendulum within ±0.1 rad
                          && (Aux::FastFabs(Unow) >= Stepper.GetMinVel());
                break;

            // Unknown or disabled control mode
            default:
                CanMove = false;
                break;
        }

        // Apply control action if movement is allowed
        if (CanMove)
        {
            Stepper.Move(Unow, STEPPER_ACC_MAX);
            StoppedCounter = 0;
            return;
        }
    }

    // If we reach this point: control is OFF or movement not allowed
    StoppedCounter++;

    // Disable motor if it has been stopped for more than 1 second
    if ((StoppedCounter > STOP_THRESHOLD) && Stepper.GetEnable())
    {
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
    ControllerCartLqr.SetReference(0, PosRefShadow);

    ControllerCartPendLqr.SetReference(0, PosRefShadow);
}

// ------------------------------------------------------------------------------------------------------- //
// Get PosRef from controllers
// ------------------------------------------------------------------------------------------------------- //

float ControlGetPosReference (void)
{
    return ControllerCartPendLqr.GetReference(0);
}

// ------------------------------------------------------------------------------------------------------- //
// SysTick interrupt service routine
// ------------------------------------------------------------------------------------------------------- //

void IsrSysTick ()
{
    static uint32_t EncoderTCounter = 0;
    static uint32_t EncoderXCounter = 0;
    static uint32_t ControlCounter = 0;
    static uint32_t LcdCounter = 0;
    static uint32_t RgbCounter = 0;
    static uint32_t ButtonCounter = 0;

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
        if (Stepper.CheckForStall(CartFiltered.Vel, 0.1, ENCODER_X_FREQUENCY))
            Stepper.Stop();

        EncoderXCounter = 0;
    }

    // Control loop
    if ((++ControlCounter >= CONTROL_INTERVAL) && (Cal.CartPos.Status == CAL_DONE))
    {
        DeviceUpdateControl();
        ControlCounter = 0;
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
    ControllerCartLead.SetGains(-0.11427, 0.569293140000000, -0.9323);
    ControllerCartLead.SetReference(0);
    ControllerCartLead.SetLimits(-STEPPER_VEL_MAX, STEPPER_VEL_MAX);

    // --------------------------------------------------------------------------------------------------- //
    // Configure cart state feedback controller
    // --------------------------------------------------------------------------------------------------- //

    // Define state feedback gains and reference
    float CartGains[2] = {4.291716, 0.049396};
    float CartFFGains[2] = {4.291716, 0};
    float CartRefs[2] = {0, 0};

    // Initialize state feedback controller
    ControllerCartLqr.Init (CartGains, CartFFGains, CartRefs, 2, -STEPPER_VEL_MAX, STEPPER_VEL_MAX);

    // --------------------------------------------------------------------------------------------------- //
    // Configure pendulum state feedback controller
    // --------------------------------------------------------------------------------------------------- //

    // Define state feedback gains and reference
    float PendGains[2] = {-67.631036, -7.567172};
    float PendFFGains[2] = {-67.631036, 0};
    float PendRefs[2] = {0, 0};

    // Initialize state feedback controller
    ControllerPendSF.Init (PendGains, PendFFGains, PendRefs, 2, -STEPPER_VEL_MAX, STEPPER_VEL_MAX);

    // --------------------------------------------------------------------------------------------------- //
    // Configure cart and pendulum state-feedback controller
    // --------------------------------------------------------------------------------------------------- //

    // Define state feedback gains and reference
    float CartPendGains[4] = {-0.097815, -2.024568, -4.826780, -1.102804};
    float CartPendFFGains[4] = {-0.097815, 0, 0, 0};
    float CartPendRefs[4] = {0};

    // Initialize state feedback controller
    ControllerCartPendLqr.Init (CartPendGains, CartPendFFGains, CartPendRefs, 4, -STEPPER_VEL_MAX, STEPPER_VEL_MAX);

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

    UartSendInfo ();

    SysCtlDelay(30000000);

    // --------------------------------------------------------------------------------------------------- //
    // Start application state machine
    // --------------------------------------------------------------------------------------------------- //

    // Start timer - Critical functions are handled here
    SysTickEnable ();

    // --------------------------------------------------------------------------------------------------- //
    // Calibrations - CalibrationCartGoHome and CalibrationPendulumPos are mandatory
    // --------------------------------------------------------------------------------------------------- //

    // Home cart
    CalibrationCartGoHome ();

    // Calibrate cart X position limits
    CalibrationCartPos ();

    // Calibrate cart velocity constant
    CalibrationCartVel();

    // Calibrate motor velocity constant
    CalibrationMotorVel();

    // Go to x = 0 so pendulum has some momentum
    CalibrationCartGoPosition (0);

    // Calibrate pendulum angle
    CalibrationPendulumPos ();

    // --------------------------------------------------------------------------------------------------- //
    // Main loop - Non critical functions
    // --------------------------------------------------------------------------------------------------- //

    while (1)
    {
        static uint32_t LastUartTime = 0;
        static uint32_t LastCmdTime = 0;

        uint32_t CurrentTime = SysTickCounter;

        // UART update
        if ((CurrentTime - LastUartTime) >= UART_TX_INTERVAL)
        {
            UartSendStatus();
            LastUartTime = CurrentTime;
        }

        // Check for UART commands
        if ((CurrentTime - LastCmdTime) >= UART_TX_INTERVAL)
        {
            UartProcessCommand();
            LastCmdTime = CurrentTime;
        }
    }
}

// ------------------------------------------------------------------------------------------------------- //
// End of code
// ------------------------------------------------------------------------------------------------------- //

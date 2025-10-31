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

// Move cart to specific position (Requires cart position to be calibrated)
void CalibrationCartGoPosition (float Position);

// Calibrate cart X position limits
void CalibrationCartPos ();

// Calibrate cart Kv - Must be called with the pendulum detached and AFTER CalibrationCartPos
void CalibrationCartVel ();

// Calibrate motor Kv - Must be called with the pendulum detached and AFTER CalibrationCartPos
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
volatile cart_t CartFiltered = cart_t_default;

// Pendulum-related variables
volatile pendulum_t Pendulum = pendulum_t_default;
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


uint8_t Start = 0;

// ------------------------------------------------------------------------------------------------------- //
// Update cart state based on encoder readings
// ------------------------------------------------------------------------------------------------------- //

void CartUpdateState()
{
    // Calculate cart absolute position - Remove offset added during calibration
    Cart.Pos = ((float)X_VALUE_TOTAL_M/Cal.CartPos.Max)*((int32_t)EncoderX.GetPos() - Cal.CartPos.Offset) - X_VALUE_ABS_M;

    // Calculate cart velocity
    Cart.Vel = (EncoderX.GetVel() / Cal.CartPos.Kv) * EncoderX.GetDir();

    // Filtered velocity
    static float CartPosLast = 0;
    static bool FirstRun = true;

    if (FirstRun)
    {
        CartPosLast = Cart.Pos;
        CartFiltered.Pos = Cart.Pos;
        CartFiltered.Vel = 0;
        FirstRun = false;
    }
    else
    {
        CartFiltered.Pos = Cart.Pos;
        CartFiltered.Vel = (CartFiltered.Pos - CartPosLast) * ENCODER_X_FREQUENCY;
        CartPosLast = CartFiltered.Pos;
    }
}

// ------------------------------------------------------------------------------------------------------- //
// Update pendulum state based on encoder readings
// ------------------------------------------------------------------------------------------------------- //

void PendulumUpdateState()
{
    // Calculate pendulum absolute position
    Pendulum.Pos = ((float)T_VALUE_MAX_RAD/Cal.Theta.Max)*((int32_t)EncoderT.GetPos() - Cal.Theta.Offset) - T_VALUE_ABS_MAX;

    // Calculate pendulum velocity
    Pendulum.Vel = (((T_VALUE_MAX_RAD * EncoderT.GetVel()) / Cal.Theta.Max) * ENCODER_T_FREQUENCY) * EncoderT.GetDir();

    // Filtered velocity using unwrapped angle difference
    static float PendulumPosLast = 0;
    static bool FirstRun = true;

    if (FirstRun)
    {
        PendulumPosLast = Pendulum.Pos;
        PendulumFiltered.Pos = Pendulum.Pos;
        PendulumFiltered.Vel = 0;
        FirstRun = false;
    }
    else
    {
        // Calculate angular difference with unwrapping
        float diff = Pendulum.Pos - PendulumPosLast;

        // Unwrap the angle difference (shortest path)
        while (diff > M_PI) diff -= 2.0f * M_PI;
        while (diff < -M_PI) diff += 2.0f * M_PI;

        // Calculate filtered velocity from unwrapped difference
        PendulumFiltered.Vel = diff * ENCODER_T_FREQUENCY;

        // Position filter (optional - same as raw for angles)
        PendulumFiltered.Pos = Pendulum.Pos;
    }

    PendulumPosLast = Pendulum.Pos;
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
// Move cart to specific position (Requires cart position to be calibrated)
// ------------------------------------------------------------------------------------------------------- //

void CalibrationCartGoPosition (float Position)
{
    // Set calibration variables
    Cal.CartPos.Status = CAL_RUNNING;
    Cal.CartPos.Progress = 0;
    CalibrationUpdateFlags();

    float StartPos = Cart.Pos;
    float DeltaPos = Cart.Pos - Position;
    float TotalDistance = Aux::FastFabs(DeltaPos);

    // Start stepper
    if (DeltaPos > 0)
    {
        Stepper.Move(-STEPPER_VEL_CAL, 0.25);
        while ((Cart.Pos > Position) && Stepper.GetEnable())
        {
            float Traveled = Aux::FastFabs(Cart.Pos - StartPos);
            Cal.CartPos.Progress = (Traveled / TotalDistance) * 100;
        }
    }
    else if (DeltaPos < 0)
    {
        Stepper.Move(STEPPER_VEL_CAL, 0.25);
        while ((Cart.Pos < Position) && Stepper.GetEnable())
        {
            float Traveled = Aux::FastFabs(Cart.Pos - StartPos);
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
        while ((Cart.Pos > -CAL_KV_END_POSITIONS[IterationCounter]) && (Stepper.GetEnable()))
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
        while ((Cart.Pos < CAL_KV_END_POSITIONS[IterationCounter]) && (Stepper.GetEnable()))
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
        while ((Cart.Pos > -CAL_KV_END_POSITIONS[IterationCounter]) && (Stepper.GetEnable()))
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
        while ((Cart.Pos < CAL_KV_END_POSITIONS[IterationCounter]) && (Stepper.GetEnable()))
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
        response.addString("status", "error");
        response.addString("msg", "invalid_json");
    }

    else
    {
        // Extract command field
        char command[32];
        if (!Parser.getString("cmd", command, sizeof(command)))
        {
            response.addString("status", "error");
            response.addString("msg", "missing_cmd_field");
        }
        // MODE command - Change control mode
        else if (strcmp(command, "MODE") == 0)
        {
            int32_t mode;
            if (Parser.getInt("value", &mode) && mode >= 0 && mode < sizeof_control_mode_t)
            {
                ControlMode = (control_mode_t)mode;
                response.addString("status", "ok");
                response.addString("msg", "mode_changed");
                response.addInt("value", mode);
            }
            else
            {
                response.addString("status", "error");
                response.addString("msg", "invalid_mode");
            }
        }
        // SREF command - Set shadow reference
        else if (strcmp(command, "SREF") == 0)
        {
            float ref;
            if (Parser.getFloat("value", &ref) && ref >= -0.2 && ref <= 0.2)
            {
                PosRefShadow = ref;
                response.addString("status", "ok");
                response.addString("msg", "shadow_ref_set");
                response.addFloat("value", ref, 6);
            }
            else
            {
                response.addString("status", "error");
                response.addString("msg", "ref_out_of_range");
            }
        }
        // AREF command - Apply reference
        else if (strcmp(command, "AREF") == 0)
        {
            ControlSetPosReference();
            response.addString("status", "ok");
            response.addString("msg", "reference_applied");
            response.addFloat("value", ControllerFullLqr.GetReference(0), 6);
        }
        // Unknown command
        else
        {
            response.addString("status", "error");
            response.addString("msg", "unknown_command");
            response.addString("cmd", command);
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
    json.addString("type", "INFO");
    json.addString("name", DEVICE_FW_NAME);
    json.addString("version", DEVICE_FW_VERSION);
    json.addString("author", DEVICE_FW_AUTHOR);
    json.addString("date", __DATE__);
    json.addString("time", __TIME__);
    json.endObject();

    Serial.SendString(json.getBuffer());
    Serial.SendString("\n");
}

// ------------------------------------------------------------------------------------------------------- //
// Send device status via UART
// ------------------------------------------------------------------------------------------------------- //

void UartSendStatus()
{
    static char json_buffer[512];
    JsonBuilder json(json_buffer, sizeof(json_buffer));

    json.startObject();

    // Timestamp
    json.addUInt("ts", SysTickCounter);

    // Control mode
    json.addInt("mode", (int32_t)ControlMode);

    // Calibration status
    json.addInt("cal_x_status", (int32_t)Cal.CartPos.Status);
    json.addInt("cal_t_status", (int32_t)Cal.Theta.Status);

    // Motor data
    json.addFloat("mot_target_vel", Stepper.GetTargetVel(), 4);
    json.addFloat("mot_current_vel", Stepper.GetCurrentVel(), 4);

    float AccNow = 0;
    if (Stepper.GetTargetVel() > Stepper.GetCurrentVel())
        AccNow = Stepper.GetCurrentAcc();
    else if (Stepper.GetTargetVel() < Stepper.GetCurrentVel())
        AccNow = -Stepper.GetCurrentAcc();
    json.addFloat("mot_acc", AccNow, 4);

    json.addInt("mot_pwm", (Stepper.GetDir() == 1 ? Stepper.GetPwmFrequency() : -Stepper.GetPwmFrequency()));

    // Cart data
    json.addFloat("cart_pos", Cart.Pos, 6);
    json.addFloat("cart_vel", Cart.Vel, 6);
    json.addFloat("cart_pos_filt", CartFiltered.Pos, 6);
    json.addFloat("cart_vel_filt", CartFiltered.Vel, 6);

    // Pendulum data
    json.addFloat("pend_pos", Pendulum.Pos, 6);
    json.addFloat("pend_vel", Pendulum.Vel, 6);
    json.addFloat("pend_pos_filt", PendulumFiltered.Pos, 6);
    json.addFloat("pend_vel_filt", PendulumFiltered.Vel, 6);

    // References
    json.addFloat("ref_pos", ControllerFullLqr.GetReference(0), 6);
    json.addFloat("ref_shadow", PosRefShadow, 6);

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
    if ((Cal.CartPos.Status == CAL_RUNNING) || (Cal.Theta.Status == CAL_RUNNING))
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
        bool CanMove = false;

        // Cart PID controller
        if (ControlMode == CONTROL_CART_PID)
            Unow = ControllerCartPid.Compute(CartFiltered.Pos);

        // Cart Lead controller
        else if (ControlMode == CONTROL_CART_LEAD)
            Unow = ControllerCartLead.Compute(CartFiltered.Pos);

        // Cart State feedback controller
        else if (ControlMode == CONTROL_CART_SF)
        {
            ControllerCartSF.SetState(0, CartFiltered.Pos);
            ControllerCartSF.SetState(1, CartFiltered.Vel);
            Unow = ControllerCartSF.Compute();
        }

        // Pendulum State feedback controller
        else if (ControlMode == CONTROL_PEND_SF)
        {
            ControllerPendSF.SetState(0, PendulumFiltered.Pos);
            ControllerPendSF.SetState(1, PendulumFiltered.Vel);
            Unow = ControllerPendSF.Compute();
        }

        // Full LQR controller
        else if (ControlMode == CONTROL_FULL_LQR)
        {
            ControllerFullLqr.SetState(0, CartFiltered.Pos);
            ControllerFullLqr.SetState(1, CartFiltered.Vel);
            ControllerFullLqr.SetState(2, PendulumFiltered.Pos);
            ControllerFullLqr.SetState(3, PendulumFiltered.Vel);
            Unow = ControllerFullLqr.Compute();
        }


        // Check if can apply Unow
        if ((ControlMode == CONTROL_PEND_SF) || (ControlMode == CONTROL_FULL_LQR))
        {
            // Pendulum related controllers
            CanMove = (Cal.Theta.Status == CAL_DONE)
                      && (Aux::FastFabs(Pendulum.Pos) < 0.1f)
                      && (Aux::FastFabs(Unow) >= Stepper.GetMinVel());
        }

        // Other controllers
        else
            CanMove = (Aux::FastFabs(Unow) >= Stepper.GetMinVel());

        // Apply control action
        if (CanMove)
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
    if ((StoppedCounter > (1000/CONTROL_LOOP_FREQUENCY)) && (Stepper.GetEnable()))
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
    if ((++ControlCounter >= CONTROL_INTERVAL) && (Cal.CartPos.Status == CAL_DONE))
    {
        DeviceUpdateControl();
        ControlCounter = 0;
    }

    // UART update
    if ((++UartCounter >= UART_TX_INTERVAL) && ((Cal.CartPos.Status == CAL_DONE) && (Cal.Theta.Status == CAL_DONE)))
    {
        UartSendStatus();
        UartCounter = 0;
    }

    // Check for UART commands
    if ((++CmdCounter >= UART_TX_INTERVAL) && ((Cal.CartPos.Status == CAL_DONE) && (Cal.Theta.Status == CAL_DONE)))
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
    float CartGains[2] = {4.426132, 0.050531};
    float CartFFGains[2] = {4.426132, 0};
    float CartRefs[2] = {0, 0};

    // Initialize Cart State Feedback controller
    ControllerCartSF.Init (CartGains, CartFFGains, CartRefs, 2, -STEPPER_ACC_MAX, STEPPER_ACC_MAX);

    // --------------------------------------------------------------------------------------------------- //
    // Configure pendulum state feedback controller
    // --------------------------------------------------------------------------------------------------- //

    // Define state feedback gains and reference
    float PendGains[2] = {-67.631036, -7.567172};
    float PendFFGains[2] = {-67.631036, 0};
    float PendRefs[2] = {0, 0};

    // Initialize Pend State Feedback controller
    ControllerPendSF.Init (PendGains, PendFFGains, PendRefs, 2, -STEPPER_ACC_MAX, STEPPER_ACC_MAX);

    // --------------------------------------------------------------------------------------------------- //
    // Configure LQR controller
    // --------------------------------------------------------------------------------------------------- //

    // Define LQR gains and references
    float CartPendGains[4] = {-0.097815, -2.024568, -4.826780, -1.102804};
    float CartPendFFGains[4] = {-0.097815, 0, 0, 0};
    float CartPendRefs[4] = {0};

    // Initialize LQR controller
    ControllerFullLqr.Init (CartPendGains, CartPendFFGains, CartPendRefs, 4, -STEPPER_ACC_MAX, STEPPER_ACC_MAX);

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

    // Start timer
    SysTickEnable ();

    // --------------------------------------------------------------------------------------------------- //
    // Calibrations - CalibrationCartGoHome and CalibrationPendulumPos are mandatory
    // --------------------------------------------------------------------------------------------------- //

    // Home cart
    CalibrationCartGoHome ();

//    // Calibrate cart X position limits
//    CalibrationCartPos ();
//
//    // Calibrate cart velocity constant - Must be called with the pendulum detached
//    CalibrationCartVel();
//
//    // Calibrate motor velocity constant - Must be called with the pendulum detached
//    CalibrationMotorVel();
//
//    // Go to x = 0 so pendulum has some momentum
//    CalibrationCartGoPosition (0);

    // Calibrate pendulum angle
    CalibrationPendulumPos ();

    // --------------------------------------------------------------------------------------------------- //
    // Main loop
    // --------------------------------------------------------------------------------------------------- //


    while (Start == 0);

    while (1)
    {
        static float MoveVel = STEPPER_VEL_CAL;

        // Start stepper - Backwards direction
        Stepper.Move (-MoveVel, STEPPER_ACC_MAX);

        while ((Cart.Pos > -0.075) && Stepper.GetEnable());

        Stepper.Move (0, STEPPER_ACC_MAX);
        while (Stepper.GetCurrentVel () != 0);

        SysCtlDelay(10000000);

        // Start stepper
        Stepper.Move (MoveVel, STEPPER_ACC_MAX);

        while ((Cart.Pos < 0.075) && Stepper.GetEnable());

        Stepper.Move (0, STEPPER_ACC_MAX);
        while (Stepper.GetCurrentVel () != 0);

        SysCtlDelay(10000000);

        MoveVel += 0.25;
        if (MoveVel > 1)
            MoveVel = STEPPER_VEL_CAL;
    }
}

// ------------------------------------------------------------------------------------------------------- //
// End of code
// ------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------- //

// Nokia 5110 LCD functions - SPI mode
// Version: 2.0
// Author: Renan Duarte
// E-mail: duarte.renan@hotmail.com
// Date:   11/05/2024

// ------------------------------------------------------------------------------------------------------- //
// Connections
// ------------------------------------------------------------------------------------------------------- //

// +---------------------+
// | TivaC    | Display  |
// |----------|----------|
// | SPIxCLK  | SCLK     |
// | SPIxSIMO | DN       |
// | GPIOxx   | DC       |
// | GPIOxx   | SCE      |
// | GPIOxx   | BKL      |
// | RST      | RST      |
// +---------------------+

// ------------------------------------------------------------------------------------------------------- //
// Behaviour
// ------------------------------------------------------------------------------------------------------- //

//  Any changes in the LCD screen will only be performed after the Commit function is called. All other
//  functions such as GoTo, Write or Draw only change the _Buffer in the microcontroller's memory.

// ------------------------------------------------------------------------------------------------------- //
// Coordinate system
// ------------------------------------------------------------------------------------------------------- //

// Text functions (Write*, Goto, Clear*) operate based on Bank and Column values (B vs X coordinates)
// Drawing functions (Draw*) operate based on pixel coordinates values (X vs Y coordinates)

// (0,0)
// +-------------------+
// |       Bank 0      |
// |       Bank 1      |
// |        ...        |
// | PCD8544_BANKS - 2 |
// | PCD8544_BANKS - 1 |
// +-------------------+
//                     (PCD8544_COLUMNS - 1, PCD8544_ROWS - 1)

// ------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------- //
// Includes
// ------------------------------------------------------------------------------------------------------- //

// LCD5110 defines and macros
#include "Lcd_TivaC.hpp"

// Standard libraries
#include <stdint.h>
#include "stdlib.h"

// Auxiliary functions
#include <Aux_Functions.hpp>

// TivaC device defines and macros
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/ssi.h"

// ------------------------------------------------------------------------------------------------------- //
// Private functions
// ------------------------------------------------------------------------------------------------------- //

// Name:        _SetSelect
// Description: Controls the SCE pin
// Arguments:   Select - true to select LCD, false to deselect
// Returns:     None

void Lcd::_SetSelect (bool Select)
{
    if (Select)
        GPIOPinWrite (_Config.Base.Sce, _Config.Pin.Sce, ~_Config.Pin.Sce);
    else
        GPIOPinWrite (_Config.Base.Sce, _Config.Pin.Sce, _Config.Pin.Sce);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        _SetDc
// Description: Controls the DC pin
// Arguments:   Mode - lcd_mode_t value
// Returns:     None

void Lcd::_SetDc (lcd_mode_t Mode)
{
    if (Mode == LCD_COMMAND)
        GPIOPinWrite (_Config.Base.Dc, _Config.Pin.Dc, ~_Config.Pin.Dc);
    else
        GPIOPinWrite (_Config.Base.Dc, _Config.Pin.Dc, _Config.Pin.Dc);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        _SetBkl
// Description: Controls the backlight pin
// Arguments:   Bkl - lcd_backlight_t value
// Returns:     None

void Lcd::_SetBkl (lcd_backlight_t Bkl)
{
    if (Bkl == LCD_BKL_OFF)
        GPIOPinWrite (_Config.Base.Bkl, _Config.Pin.Bkl, ~_Config.Pin.Bkl);
    else
        GPIOPinWrite (_Config.Base.Bkl, _Config.Pin.Bkl, _Config.Pin.Bkl);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        _LoadSpiBuffer
// Description: Loads a byte into the SPI buffer
// Arguments:   Byte - Byte to be loades
// Returns:     None

void Lcd::_LoadSpiBuffer (uint8_t Byte)
{
    SSIDataPut (_Config.Base.Ssi, Byte);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        _SpiIsBusy
// Description: Gets SPI bus status
// Arguments:   None
// Returns:     SPI busy status

bool Lcd::_SpiIsBusy ()
{
    return SSIBusy(_Config.Base.Ssi);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        _InitHardware
// Description: Starts the TivaC device peripherals required to this application
// Arguments:   Config - lcd_config_t struct
// Returns:     None

void Lcd::_InitHardware (lcd_config_t *Config)
{
    // Get lcd_config_t object parameters and store in a "private" variable
    memcpy(&_Config, Config, sizeof(lcd_config_t));

    // Enable peripherals
    SysCtlPeripheralEnable (_Config.Periph.Ssi);
    SysCtlPeripheralEnable (_Config.Periph.Sclk);
    SysCtlPeripheralEnable (_Config.Periph.Dn);
    SysCtlPeripheralEnable (_Config.Periph.Sce);
    SysCtlPeripheralEnable (_Config.Periph.Dc);
    SysCtlPeripheralEnable (_Config.Periph.Bkl);

    // Configure SSI pins
    GPIOUnlockPin(_Config.Base.Dn, _Config.Pin.Dn);
    GPIOPinConfigure (_Config.PinMux.Dn);
    GPIOPinTypeSSI (_Config.Base.Dn, _Config.Pin.Dn);
    GPIOUnlockPin(_Config.Base.Sclk, _Config.Pin.Sclk);
    GPIOPinConfigure (_Config.PinMux.Sclk);
    GPIOPinTypeSSI (_Config.Base.Sclk, _Config.Pin.Sclk);

    // Configure SSI
    SSIConfigSetExpClk (_Config.Base.Ssi, SysCtlClockGet(), SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, 3500000, 8);

    // Configure GPIO pins
    GPIOUnlockPin(_Config.Base.Dc, _Config.Pin.Dc);
    GPIOPinTypeGPIOOutput (_Config.Base.Dc, _Config.Pin.Dc);
    GPIOUnlockPin(_Config.Base.Sce, _Config.Pin.Sce);
    GPIOPinTypeGPIOOutput (_Config.Base.Sce, _Config.Pin.Sce);
    GPIOUnlockPin(_Config.Base.Bkl, _Config.Pin.Bkl);
    GPIOPinTypeGPIOOutput (_Config.Base.Bkl, _Config.Pin.Bkl);
    _SetBkl(LCD_BKL_OFF);

    // Enable SSI
    SSIEnable (_Config.Base.Ssi);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        _SendByte
// Description: Sends a byte to the LCD controller
// Arguments:   Mode - lcd_mode_t value
//              Byte - Byte to be sent
// Returns:     None

void Lcd::_SendByte (lcd_mode_t Mode, uint8_t Byte)
{
    // Byte is data
    if (Mode == LCD_DATA)
        _SetDc(LCD_DATA);

    // Byte is command
    else
        _SetDc(LCD_COMMAND);

    // Enable display
    _SetSelect(true);

    // Write byte in SPI buffer
    _LoadSpiBuffer(Byte);

    // Wait until complete
    while(_SpiIsBusy());

    // Disable Display
    _SetSelect(false);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        _BufferPutByte
// Description: Puts a byte in the local buffer at current cursor position and advance cursor
// Arguments:   Byte - Byte to be put
// Returns:     None

void Lcd::_BufferPutByte (uint8_t Byte)
{
    // Save data in buffer
    _Buffer [GetBank()][GetColumn()] = Byte;

    // Define current position
    if (_Cursor_Column == PCD8544_COLUMNS - 1)
    {
        _Cursor_Column = 0;

        if (_Cursor_Bank == PCD8544_BANKS - 1)
            _Cursor_Bank = 0;

        else
            _Cursor_Bank++;
    }

    else
        _Cursor_Column++;
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        _AdjustByte
// Description: Adjusts byte according to desired pixel mode
// Arguments:   ByteToSend - Byte to be written in buffer
//              ByteInBuffer - Byte already in buffer
//              Mode - Pixel mode - lcd_pixel_mode_t value
// Returns:     None

uint8_t Lcd::_AdjustByte(uint8_t ByteToSend, uint8_t ByteInBuffer, lcd_pixel_mode_t Mode)
{
    if (Mode == LCD_PIXEL_ON)
        ByteToSend |= ByteInBuffer;
    else if (Mode == LCD_PIXEL_XOR)
        ByteToSend ^= ByteInBuffer;
    else if (Mode == LCD_PIXEL_OFF)
        ByteToSend = ByteInBuffer & ~ByteToSend;

    return ByteToSend;
}

// ------------------------------------------------------------------------------------------------------- //
// Public functions
// ------------------------------------------------------------------------------------------------------- //

// Name:        Lcd
// Description: Constructor of the class with no arguments
// Arguments:   None
// Returns:     None

Lcd::Lcd()
{

}

// ------------------------------------------------------------------------------------------------------- //

// Name:        Lcd
// Description: Constructor of the class with lcd_config_t struct as argument
// Arguments:   Config - lcd_config_t struct
// Returns:     None

Lcd::Lcd(lcd_config_t *Config) : Lcd()
{
    Init(Config);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        Init
// Description: Starts the device peripherals, configures the LCD controller and clears display RAM
// Arguments:   Config - lcd_config_t struct
// Returns:     None

void Lcd::Init (lcd_config_t *Config)
{
    // Configure peripherals
    _InitHardware (Config);

    // Send Startup commands
    _SendByte (LCD_COMMAND, PCD8544_FUNCTIONSET | PCD8544_EXTENDEDINSTRUCTION);
    _SendByte (LCD_COMMAND, PCD8544_SETVOP | 0x3F);
    _SendByte (LCD_COMMAND, PCD8544_SETTEMP | 0x01);
    _SendByte (LCD_COMMAND, PCD8544_SETBIAS | 0x03);
    _SendByte (LCD_COMMAND, PCD8544_FUNCTIONSET);
    _SendByte (LCD_COMMAND, PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYNORMAL);

    // Enable LCD
    Powerdown(LCD_PD_ON);

    // Clear all LCD
    ClearAll ();
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        Commit
// Description: Copy the local buffer to the LCD ram
// Arguments:   None
// Returns:     None

void Lcd::Commit ()
{
    // Variables
    uint8_t Bank = 0;
    uint8_t Column = 0;

    // Go to bank 0, column 0
    _SendByte (LCD_COMMAND, PCD8544_SETBANKADDR);
    _SendByte (LCD_COMMAND, PCD8544_SETCOLUMNADDR);

    // Send all bytes
    for (Bank = 0; Bank < PCD8544_BANKS; Bank++)
    {
        for (Column = 0; Column < PCD8544_COLUMNS; Column++)
            _SendByte (LCD_DATA, _Buffer[Bank][Column]);
    }
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        ClearRange
// Description: Clears a range of columns and sets the cursor at the start of the range
// Arguments:   Bank - Starting bank of the range
//              Column - Starting column of the range
//              Length - Number of columns to be clear
// Returns:     None

void Lcd::ClearRange (uint8_t Bank, uint8_t Column, uint16_t Length)
{
    // Variables
    uint16_t Counter = 0;

    // Go to bank and column
    Goto (Bank, Column);

    // Put blank columns
    for (Counter = 0; Counter < Length; Counter++)
        _BufferPutByte (0x00);

    // Return cursor
    Goto (Bank, Column);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        ClearBank
// Description: Clears a bank of the LCD and sets the cursor at the column 0 of this bank
// Arguments:   Bank - Bank to be clear
// Returns:     None

void Lcd::ClearBank (uint8_t Bank)
{
    ClearRange (Bank, 0, PCD8544_COLUMNS);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        ClearAll
// Description: Clears the LCD and sets the cursor at bank 0, column 0
// Arguments:   None
// Returns:     None

void Lcd::ClearAll ()
{
    ClearRange (0, 0, PCD8544_MAXBYTES);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        Goto
// Description: Sets the LCD cursor position (bank and column)
// Arguments:   Bank - Desired bank
//              Column - Desired column
// Returns:     None

void Lcd::Goto (uint8_t Bank, uint8_t Column)
{
    _Cursor_Bank = Bank;
    _Cursor_Column = Column;
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        GetBank
// Description: Gets the current bank of the LCD cursor
// Arguments:   None
// Returns:     Cursor bank

uint8_t Lcd::GetBank ()
{
    return _Cursor_Bank;
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        GetColumn
// Description: Gets the current column of the LCD cursor
// Arguments:   None
// Returns:     Cursor column

uint8_t Lcd::GetColumn ()
{
    return _Cursor_Column;
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        Powerdown
// Description: Sets or gets LCD powerdown mode
// Arguments:   Active - lcd_powerdown_t value. LCD_PD_GET returns current powerdown status
// Returns:     Current powerdown status - lcd_powerdown_t value

lcd_powerdown_t Lcd::Powerdown (lcd_powerdown_t Active)
{
    // LCD off
    if (Active == LCD_PD_OFF)
    {
        _SendByte (LCD_COMMAND, PCD8544_FUNCTIONSET | PCD8544_POWERDOWN);
        _Active_Status = LCD_PD_OFF;
    }

    // LCD on
    else if (Active == LCD_PD_ON)
    {
        _SendByte (LCD_COMMAND, PCD8544_FUNCTIONSET & ~PCD8544_POWERDOWN);
        _Active_Status = LCD_PD_ON;
    }

    // Return status flag
    return _Active_Status;
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        Backlight
// Description: Sets or gets the backlight status
// Arguments:   Bkl - lcd_backlight_t value. LCD_BKL_GET returns backlight status
// Returns:     Backlight status - lcd_backlight_t value

lcd_backlight_t Lcd::Backlight (lcd_backlight_t Bkl)
{
    // Backlight off
    if (Bkl == LCD_BKL_OFF)
    {
        // Turn backlight off
        _SetBkl(LCD_BKL_OFF);

        // Update status flag
        _Backlight_Status = LCD_BKL_OFF;

        // Return status flag
        return _Backlight_Status;
    }

    // Backlight on
    else if (Bkl == LCD_BKL_ON)
    {
        // Turn backlight on
        _SetBkl(LCD_BKL_ON);

        // Update status flag
        _Backlight_Status = LCD_BKL_ON;

        // Return status flag
        return _Backlight_Status;
    }

    else
        // Return status flag
        return _Backlight_Status;
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        Invert
// Description: Sets or gets inverted mode status
// Arguments:   Inv - lcd_inv_t value. LCD_INV_GET returns inverted status
// Returns:     Inverted mode status - lcd_inv_t value

lcd_inv_t Lcd::Invert (lcd_inv_t Inv)
{
    // Normal mode
    if (Inv == LCD_INV_OFF)
    {
        // Send command to LCD
        _SendByte (LCD_COMMAND, PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYNORMAL);

        // Update status flag
        _Invert_Status = LCD_INV_OFF;

        // Return status flag
        return _Invert_Status;
    }

    // Inverted mode
    else if (Inv == LCD_INV_ON)
    {
        // Send command to LCD
        _SendByte (LCD_COMMAND, PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYINVERTED);

        // Update status flag
        _Invert_Status = LCD_INV_ON;

        // Return status flag
        return _Invert_Status;
    }

    else
        // Return status flag
        return _Invert_Status;
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        WriteChar
// Description: Writes a 6x8 px char on the display starting at the current cursor position
// Arguments:   Char - Char to be sent
//              Font - lcd_font_t value
//              Mode - Pixel mode - lcd_pixel_mode_t value
// Returns:     None

void Lcd::WriteChar (char Char, lcd_font_t Font, lcd_pixel_mode_t Mode)
{
    // Variables
    uint8_t Counter = 0;

    // Adjust char position in font table
    Char -= 0x20;

    // Send the 5 columns of char plus an empty column
    for (Counter = 0; Counter < 6; Counter++)
    {
        // Get byte in display buffer
        uint8_t ByteInBuffer = _Buffer[GetBank()][GetColumn()];

        // Get byte to be sent according to font (6th byte is empty)
        uint8_t ByteToSend = 0;

        if (Counter != 5)
        {
            if (Font == LCD_FONT_DEFAULT)
                ByteToSend = Lcd_Font_0 [Char][Counter];
            else if (Font == LCD_FONT_SMALL)
                ByteToSend = Lcd_Font_1 [Char][Counter];
        }

        // Adjust byte according to desired pixel mode
        ByteToSend = _AdjustByte (ByteToSend, ByteInBuffer, Mode);

        // Put byte in buffer
        _BufferPutByte (ByteToSend);
    }
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        WriteString
// Description: Writes a string on the display starting at the current cursor position using 6x8 px chars
// Arguments:   String - Pointer to the string to be sent
//              Font - lcd_font_t value
//              Mode - Pixel mode - lcd_pixel_mode_t value
// Returns:     None

void Lcd::WriteString (const char *String, lcd_font_t Font, lcd_pixel_mode_t Mode)
{
    // Send all chars of the string
    while (*String)
        WriteChar (*String++, Font, Mode);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        WriteInt
// Description: Writes a int32_t number on the display starting at the current cursor position using 6x8 px chars
// Arguments:   Number - Number to be sent
//              Font - lcd_font_t value
//              Mode - Pixel mode - lcd_pixel_mode_t value
// Returns:     None

void Lcd::WriteInt (int32_t Number, lcd_font_t Font, lcd_pixel_mode_t Mode)
{
    // Variables
    char String[10];

    // Convert int to string
    Aux::L2Str (Number, String);

    // Send string
    WriteString (String, Font, Mode);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        WriteFloat
// Description: Writes a float number on the display starting at the current cursor position using 6x8 px chars
// Arguments:   Number - Number to be sent
//              DecPlaces - Number of decimal places to be shown
//              Font - lcd_font_t value
//              Mode - Pixel mode - lcd_pixel_mode_t value
// Returns:     None

void Lcd::WriteFloat (float Number, uint8_t DecPlaces, lcd_font_t Font, lcd_pixel_mode_t Mode)
{
    // Variables
    char String[10];

    // Convert float to string
    Aux::F2Str (Number, String, DecPlaces);

    // Send String
    WriteString (String, Font, Mode);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        WriteCharBig
// Description: Writes a 10x16 char on the display starting at the current cursor position
//              Only supports chars in the Lcd_FontBig charset
// Arguments:   Char - Char to be sent
//              Mode - Pixel mode - lcd_pixel_mode_t value
// Returns:     None

void Lcd::WriteCharBig (char Char, lcd_pixel_mode_t Mode)
{
    // Variables
    uint8_t Counter = 0;
    unsigned char Column = GetColumn ();
    unsigned char Bank = GetBank ();

    // Adjust char position in font table
    Char -= 0x30;

    // Send columns of the char
    for (Counter = 0; Counter < 18; Counter++)
    {
        // Go to next bank
        if (Counter == 9)
            Goto (Bank + 1, Column);

        // Get byte in display buffer
        uint8_t ByteInBuffer = _Buffer[GetBank()][GetColumn()];

        // Get byte to be sent according to font
        uint8_t ByteToSend = Lcd_FontBig [Char][Counter];

        // Adjust byte according to desired pixel mode
        ByteToSend = _AdjustByte (ByteToSend, ByteInBuffer, Mode);

        // Put byte in buffer
        _BufferPutByte (ByteToSend);
    }

    // Relocate cursor - Leave blank column
    Goto (Bank, Column + 10);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        WriteIntBig
// Description: Writes a int32_t number on the display starting at the current cursor position using 10x16 px chars
// Arguments:   Number - Number to be sent
//              Mode - Pixel mode - lcd_pixel_mode_t value
// Returns:     None

void Lcd::WriteIntBig (int32_t Number, lcd_pixel_mode_t Mode)
{
    // Variables
    char String[10];
    uint8_t Idx = 0;

    // Convert int to String
    Aux::L2Str (Number, String);

    // Adjust "-" signal if negative
    if (String[0] == '-')
        String[0] = 0x3A;

    // Send all chars of the string
    while (String[Idx] != '\0')
        WriteCharBig (String[Idx++], Mode);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        WriteFloatBig
// Description: Writes a float number on the display starting at the current cursor position using 10x16 px chars
//              Does not support Inf or NaN numbers
// Arguments:   Number - Number to be sent
//              DecPlaces - Number of decimal places to be shown
//              Mode - Pixel mode - lcd_pixel_mode_t value
// Returns:     None

void Lcd::WriteFloatBig (float Number, uint8_t DecPlaces, lcd_pixel_mode_t Mode)
{
    // Variables
    char String[10];
    uint8_t Idx = 0;

    // Convert float to string
    Aux::F2Str (Number, String, DecPlaces);

    // Adjust "-" signal if negative
    if (String[0] == '-')
        String[0] = 0x3A;

    // Send string
    while (String[Idx] != '\0')
    {
        // Adjust "." position in font table
        if (String[Idx] == '.')
            String[Idx] = 0x3B;

        // Send string char
        WriteCharBig (String[Idx++], Mode);
    }
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        DrawBitmap
// Description: Draws a bitmap on the display starting at the current cursor position
// Arguments:   Bitmap - Pointer to the array
//              Lenght - Lenght of the array (0 to PCD8544_MAXBYTES bytes)
//              Mode - Pixel mode - lcd_pixel_mode_t value
// Returns:     None

void Lcd::DrawBitmap (const uint8_t *Bitmap, uint16_t Length, lcd_pixel_mode_t Mode)
{
    // Put all bytes in buffer
    for (uint16_t Counter = 0; Counter < Length; Counter++)
    {
        // Get byte to send and advance pointer
        uint8_t ByteToSend = Bitmap[Counter];

        // Get byte in display buffer
        uint8_t ByteInBuffer = _Buffer[GetBank()][GetColumn()];

        // Adjust byte according to desired pixel mode
        ByteToSend = _AdjustByte (ByteToSend, ByteInBuffer, Mode);

        // Put byte in buffer
        _BufferPutByte (ByteToSend);
    }
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        DrawPixel
// Description: Draws a single pixel on the display
// Arguments:   X - Column of the pixel (0 to PCD8544_COLUMNS)
//              Y - Row of the pixel (0 to PCD8544_ROWS)
//              Mode - Pixel mode - lcd_pixel_mode_t value
// Returns:     None

void Lcd::DrawPixel (uint8_t X, uint8_t Y, lcd_pixel_mode_t Mode)
{
    // Check if desired position is within LCD limits
    if (X >= PCD8544_COLUMNS || Y >= PCD8544_ROWS)
        return;

    // Variables
    unsigned char Bank = Y / 8;

    // Set buffer according to desired pixel mode
    if (Mode == LCD_PIXEL_ON)
        _Buffer[Bank][X] |= (1 << (Y % 8));

    else if (Mode == LCD_PIXEL_XOR)
        _Buffer[Bank][X] ^= (1 << (Y % 8));

    else if (Mode == LCD_PIXEL_OFF)
        _Buffer[Bank][X] &= ~(1 << (Y % 8));

    // Set cursor position
    Goto (Bank, X);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        DrawLine
// Description: Draws a line between two points on the display using DDA algorithm
// Arguments:   Xi, Yi - Absolute pixel coordinates for line origin
//              Xf, Yf - Absolute pixel coordinates for line end
//              Mode - Pixel mode - lcd_pixel_mode_t value
// Returns:     None

void Lcd::DrawLine (uint8_t Xi, uint8_t Yi, uint8_t Xf, uint8_t Yf, lcd_pixel_mode_t Mode)
{
    // https://www.geeksforgeeks.org/dda-line-generation-algorithm-computer-graphics/
    // Variables
    int8_t DeltaX = (int8_t)Xf - Xi;
    int8_t DeltaY = (int8_t)Yf - Yi;
    int16_t Steps = abs(DeltaX) > abs(DeltaY) ? abs(DeltaX) : abs(DeltaY);

    // Calculate increment in x & y for each step
    float Xinc = DeltaX / (float)Steps;
    float Yinc = DeltaY / (float)Steps;

    // Starting point
    float X = Xi;
    float Y = Yi;

    // Draw each pixel
    while (Steps >= 0)
    {
        // Draw pixel at (X, Y)
        DrawPixel (X, Y, Mode);

        // Increment X and Y
        X += Xinc;
        Y += Yinc;

        // Reduce step counter
        Steps--;
    }
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        DrawRectangle
// Description: Draws a rectangle between two diagonal points on the display
// Arguments:   Xi, Yi - Absolute pixel coordinates for the first point
//              Xf, Yf - Absolute pixel coordinates for end point
//              Mode - Pixel mode - lcd_pixel_mode_t value
// Returns:     None

void Lcd::DrawRectangle (uint8_t Xi, uint8_t Yi, uint8_t Xf, uint8_t Yf, lcd_pixel_mode_t Mode)
{
    DrawLine (Xi, Yi, Xf, Yi, Mode);
    DrawLine (Xi, Yi + 1, Xi, Yf - 1, Mode);
    DrawLine (Xi, Yf, Xf, Yf, Mode);
    DrawLine (Xf, Yi + 1, Xf, Yf - 1, Mode);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        DrawFilledRectangle
// Description: Draws a filled rectangle between two diagonal points on the display
// Arguments:   Xi, Yi - Absolute pixel coordinates for the first point
//              Xf, Yf - Absolute pixel coordinates for end point
//              Mode - Pixel mode - lcd_pixel_mode_t value
// Returns:     None

void Lcd::DrawFilledRectangle (uint8_t Xi, uint8_t Yi, uint8_t Xf, uint8_t Yf, lcd_pixel_mode_t Mode)
{
    // Variables
    uint8_t Counter = 0;

    for (Counter = Yi; Counter <= Yf; Counter++)
        DrawLine (Xi, Counter, Xf, Counter, Mode);
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        DrawCircle
// Description: Draw a circle using Bresenham algorithm
// Arguments:   Xc, Yc - Absolute pixel coordinates for the center of the circle
//              Radius - Radius of the circle in pixels
//              Mode - Pixel mode - lcd_pixel_mode_t value
// Returns:     None

void Lcd::DrawCircle (uint8_t Xc, uint8_t Yc, uint8_t Radius, lcd_pixel_mode_t Mode)
{
    // https://en.wikipedia.org/wiki/Midpoint_circle_algorithm
    uint8_t Xchange = 1;
    int8_t  Ychange = 3 - (2 * Radius);
    int8_t F = 1 - Radius;
    uint8_t X = 0;
    uint8_t Y = Radius;

    DrawPixel (Xc, Yc + Radius, Mode);
    DrawPixel (Xc, Yc - Radius, Mode);
    DrawPixel (Xc + Radius, Yc, Mode);
    DrawPixel (Xc - Radius, Yc, Mode);

    while (X < Y)
    {
        if (F >= 0)
        {
            Y--;
            Ychange += 2;
            F += Ychange;
        }

        X++;
        Xchange += 2;
        F += Xchange;

        DrawPixel ((Xc + X), (Yc + Y), Mode);
        DrawPixel ((Xc - X), (Yc + Y), Mode);
        DrawPixel ((Xc + X), (Yc - Y), Mode);
        DrawPixel ((Xc - X), (Yc - Y), Mode);

        DrawPixel ((Xc + Y), (Yc + X), Mode);
        DrawPixel ((Xc - Y), (Yc + X), Mode);
        DrawPixel ((Xc + Y), (Yc - X), Mode);
        DrawPixel ((Xc - Y), (Yc - X), Mode);
    }
}

// ------------------------------------------------------------------------------------------------------- //

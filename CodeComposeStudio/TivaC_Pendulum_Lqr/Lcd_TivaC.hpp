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

#ifndef LCD_TIVAC_H_
#define LCD_TIVAC_H_

#ifdef __cplusplus
extern "C"
{
#endif

// ------------------------------------------------------------------------------------------------------- //
// Includes
// ------------------------------------------------------------------------------------------------------- //

// Standard libraries
#include <stdint.h>
#include "stdlib.h"

// ------------------------------------------------------------------------------------------------------- //
// PCD8544 LCD controller definitions - Do not modify
// ------------------------------------------------------------------------------------------------------- //

// Instruction set
#define PCD8544_FUNCTIONSET                 0x20
#define PCD8544_DISPLAYCONTROL              0x08
#define PCD8544_SETBANKADDR                 0x40
#define PCD8544_SETCOLUMNADDR               0x80
#define PCD8544_COLUMNS                     84
#define PCD8544_ROWS                        48
#define PCD8544_BANKS                       6
#define PCD8544_MAXBYTES                    PCD8544_COLUMNS * PCD8544_BANKS

// Function set
#define PCD8544_POWERDOWN                   0x04
#define PCD8544_ENTRYMODE                   0x02
#define PCD8544_EXTENDEDINSTRUCTION         0x01

// Display control
#define PCD8544_DISPLAYBLANK                0x00
#define PCD8544_DISPLAYALLON                0x01
#define PCD8544_DISPLAYNORMAL               0x04
#define PCD8544_DISPLAYINVERTED             0x05

// Temperature control
#define PCD8544_SETTEMP                     0x04

// Bias system
#define PCD8544_SETBIAS                     0x10

// Set Vop
#define PCD8544_SETVOP                      0x80

// ------------------------------------------------------------------------------------------------------- //
// Program enumerations
// ------------------------------------------------------------------------------------------------------- //

// LCD powerdown settings
typedef enum
{
    LCD_PD_OFF,
    LCD_PD_ON,
    LCD_PD_GET,
} lcd_powerdown_t;

// LCD font settings
typedef enum
{
    LCD_FONT_DEFAULT,
    LCD_FONT_SMALL,
    LCD_FONT_GET,
} lcd_font_t;

// LCD backlight settings
typedef enum
{
    LCD_BKL_OFF,
    LCD_BKL_ON,
    LCD_BKL_GET,
} lcd_backlight_t;

// LCD inverted mode settings
typedef enum
{
    LCD_INV_OFF,
    LCD_INV_ON,
    LCD_INV_GET,
} lcd_inv_t;

// Pixel drawing settings
typedef enum
{
    LCD_PIXEL_OFF,
    LCD_PIXEL_ON,
    LCD_PIXEL_XOR,
} lcd_pixel_mode_t;

// Mode definitions
typedef enum
{
    LCD_COMMAND,
    LCD_DATA
} lcd_mode_t;

// ------------------------------------------------------------------------------------------------------- //
// Structs
// ------------------------------------------------------------------------------------------------------- //

// Peripherals
typedef struct
{
    uint32_t Ssi;
    uint32_t Sclk;
    uint32_t Dn;
    uint32_t Sce;
    uint32_t Dc;
    uint32_t Bkl;
} lcd_periph_t;

// Bases
typedef struct
{
    uint32_t Ssi;
    uint32_t Sclk;
    uint32_t Dn;
    uint32_t Sce;
    uint32_t Dc;
    uint32_t Bkl;
} lcd_base_t;

// Pin mux configs
typedef struct
{
    uint32_t Sclk;
    uint32_t Dn;
} lcd_pin_mux_t;

// Display Pins
typedef struct
{
    uint32_t Sclk;
    uint32_t Dn;
    uint32_t Sce;
    uint32_t Dc;
    uint32_t Bkl;
} lcd_pin_t;

// Configuration structure
typedef struct
{
    lcd_periph_t Periph;
    lcd_base_t Base;
    lcd_pin_mux_t PinMux;
    lcd_pin_t Pin;
} lcd_config_t;

// ------------------------------------------------------------------------------------------------------- //
// Class prototype
// ------------------------------------------------------------------------------------------------------- //

class Lcd
{
    // --------------------------------------------------------------------------------------------------- //
    // Private members
    // --------------------------------------------------------------------------------------------------- //

    private:

        // Configuration structure
        lcd_config_t _Config;

        // Current column of the cursor
        uint8_t _Cursor_Column = 0;

        // Current bank of the cursor
        uint8_t  _Cursor_Bank = 0;

        // LCD powerdown status
        lcd_powerdown_t _Active_Status = LCD_PD_OFF;

        // Inverted mode status flag - Non inverted mode at startup
        lcd_inv_t _Invert_Status = LCD_INV_OFF;

        // LCD backlight status flag - Backlight off at startup
        lcd_backlight_t _Backlight_Status = LCD_BKL_OFF;

        // LCD buffer - Identical copy of the LCD ram
        uint8_t _Buffer [PCD8544_BANKS][PCD8544_COLUMNS] = {{0}, {0}};

        // ----------------------------------------------------------------------------------------------- //

        // Name:        _SetSelect
        // Description: Controls the SCE pin
        // Arguments:   Select - true to select LCD, false to deselect
        // Returns:     None
        void _SetSelect (bool Select);

        // Name:        _SetDc
        // Description: Controls the DC pin
        // Arguments:   Mode - lcd_mode_t value
        // Returns:     None
        void _SetDc (lcd_mode_t Mode);

        // Name:        _SetBkl
        // Description: Controls the backlight pin
        // Arguments:   Bkl - lcd_backlight_t value
        // Returns:     None
        void _SetBkl (lcd_backlight_t Bkl);

        // Name:        _LoadSpiBuffer
        // Description: Loads a byte into the SPI buffer
        // Arguments:   Byte - Byte to be loades
        // Returns:     None
        void _LoadSpiBuffer (uint8_t Byte);

        // Name:        _SpiIsBusy
        // Description: Gets SPI bus status
        // Arguments:   None
        // Returns:     SPI busy status
        bool _SpiIsBusy ();

        // Name:        _InitHardware
        // Description: Starts the TivaC device peripherals required to this application
        // Arguments:   Config - lcd_config_t struct
        // Returns:     None
        void _InitHardware (lcd_config_t *Config);

        // Name:        _SendByte
        // Description: Sends a byte to the LCD controller
        // Arguments:   Mode - lcd_mode_t value
        //              Byte - Byte to be sent
        // Returns:     None
        void _SendByte (lcd_mode_t Mode, uint8_t Byte);

        // Name:        _BufferPutByte
        // Description: Puts a byte in the local buffer at current cursor position and advance cursor
        // Arguments:   Byte - Byte to be put
        // Returns:     None
        void _BufferPutByte (uint8_t Byte);

        // Name:        _AdjustByte
        // Description: Adjusts byte according to desired pixel mode
        // Arguments:   ByteToSend - Byte to be written in buffer
        //              ByteInBuffer - Byte already in buffer
        //              Mode - Pixel mode - lcd_pixel_mode_t value
        // Returns:     Adjusted ByteToSend
        uint8_t _AdjustByte(uint8_t ByteToSend, uint8_t ByteInBuffer, lcd_pixel_mode_t Mode);

    // --------------------------------------------------------------------------------------------------- //
    // Public members
    // --------------------------------------------------------------------------------------------------- //

    public:

        // Name:        Lcd
        // Description: Constructor of the class with no arguments
        // Arguments:   None
        // Returns:     None
        Lcd();

        // Name:        Lcd
        // Description: Constructor of the class with lcd_config_t struct as argument
        // Arguments:   Config - lcd_config_t struct
        // Returns:     None
        Lcd(lcd_config_t *Config);

        // Name:        Init
        // Description: Starts the device peripherals, configures the LCD controller and clears display RAM
        // Arguments:   Config - lcd_config_t struct
        // Returns:     None
        void Init (lcd_config_t *Config);

        // Name:        Commit
        // Description: Copy the local buffer to the LCD ram
        // Arguments:   None
        // Returns:     None
        void Commit ();

        // ----------------------------------------------------------------------------------------------- //

        // Name:        ClearRange
        // Description: Clears a range of columns and sets the cursor at the start of the range
        // Arguments:   Bank - Starting bank of the range
        //              Column - Starting column of the range
        //              Length - Number of columns to be clear
        // Returns:     None
        void ClearRange (uint8_t Bank, uint8_t Column, uint16_t Length);

        // Name:        ClearBank
        // Description: Clears a bank of the LCD and sets the cursor at the column 0 of this bank
        // Arguments:   Bank - Bank to be clear
        // Returns:     None
        void ClearBank (uint8_t Bank);

        // Name:        ClearAll
        // Description: Clears the LCD and sets the cursor at bank 0, column 0
        // Arguments:   None
        // Returns:     None
        void ClearAll ();

        // ----------------------------------------------------------------------------------------------- //

        // Name:        Goto
        // Description: Sets the LCD cursor position (bank and column)
        // Arguments:   Bank - Desired bank
        //              Column - Desired column
        // Returns:     None
        void Goto (uint8_t Bank, uint8_t Column);

        // Name:        GetBank
        // Description: Gets the current bank of the LCD cursor
        // Arguments:   None
        // Returns:     Cursor bank
        uint8_t GetBank ();

        // Name:        GetColumn
        // Description: Gets the current column of the LCD cursor
        // Arguments:   None
        // Returns:     Cursor column
        uint8_t GetColumn ();

        // ----------------------------------------------------------------------------------------------- //

        // Name:        Powerdown
        // Description: Sets or gets LCD powerdown mode
        // Arguments:   Active - lcd_powerdown_t value. LCD_PD_GET returns current powerdown status
        // Returns:     Current powerdown status - lcd_powerdown_t value
        lcd_powerdown_t Powerdown (lcd_powerdown_t Active);

        // Name:        Backlight
        // Description: Sets or gets the backlight status
        // Arguments:   Bkl - lcd_backlight_t value. LCD_BKL_GET returns backlight status
        // Returns:     Backlight status - lcd_backlight_t value
        lcd_backlight_t Backlight (lcd_backlight_t Bkl);

        // Name:        Invert
        // Description: Sets or gets inverted mode status
        // Arguments:   Inv - lcd_inv_t value. LCD_INV_GET returns inverted status
        // Returns:     Inverted mode status - lcd_inv_t value
        lcd_inv_t Invert (lcd_inv_t Inv);

        // ----------------------------------------------------------------------------------------------- //

        // Name:        WriteChar
        // Description: Writes a 6x8 px char on the display starting at the current cursor position
        //              Only supports chars in the Lcd_FontBig charset
        // Arguments:   Char - Char to be sent
        //              Font - lcd_font_t value
        //              Mode - Pixel mode - lcd_pixel_mode_t value
        // Returns:     None
        void WriteChar (char Char, lcd_font_t Font, lcd_pixel_mode_t Mode);

        // Name:        WriteString
        // Arguments:   String - Pointer to the string to be sent
        //              Font - lcd_font_t value
        //              Mode - Pixel mode - lcd_pixel_mode_t value
        // Returns:     None
        void WriteString (const char *String, lcd_font_t Font, lcd_pixel_mode_t Mode);

        // Name:        WriteInt
        // Description: Writes a int32_t number on the display starting at the current cursor position using 6x8 px chars
        // Arguments:   Number - Number to be sent
        //              Font - lcd_font_t value
        //              Mode - Pixel mode - lcd_pixel_mode_t value
        // Returns:     None
        void WriteInt (int32_t Number, lcd_font_t Font, lcd_pixel_mode_t Mode);

        // Name:        WriteFloat
        // Description: Writes a float number on the display starting at the current cursor position using 6x8 px chars
        // Arguments:   Number - Number to be sent
        //              DecPlaces - Number of decimal places to be shown
        //              Font - lcd_font_t value
        //              Mode - Pixel mode - lcd_pixel_mode_t value
        // Returns:     None
        void WriteFloat (float Number, uint8_t DecPlaces, lcd_font_t Font, lcd_pixel_mode_t Mode);

        // ----------------------------------------------------------------------------------------------- //

        // Name:        WriteCharBig
        // Description: Writes a 10x16 char on the display starting at the current cursor position
        //              Only supports chars in the Lcd_FontBig charset
        // Arguments:   Char - Char to be sent
        //              Mode - Pixel mode - lcd_pixel_mode_t value
        // Returns:     None
        void WriteCharBig (char Char, lcd_pixel_mode_t Mode);

        // Name:        WriteIntBig
        // Description: Writes a int32_t number on the display starting at the current cursor position using 10x16 px chars
        // Arguments:   Number - Number to be sent
        //              Mode - Pixel mode - lcd_pixel_mode_t value
        // Returns:     None
        void WriteIntBig (int32_t Number, lcd_pixel_mode_t Mode);

        // Name:        WriteFloatBig
        // Description: Writes a float number on the display starting at the current cursor position using 10x16 px chars
        //              Does not support Inf or NaN numbers
        // Arguments:   Number - Number to be sent
        //              DecPlaces - Number of decimal places to be shown
        //              Mode - Pixel mode - lcd_pixel_mode_t value
        // Returns:     None
        void WriteFloatBig (float Number, uint8_t DecPlaces, lcd_pixel_mode_t Mode);

        // ----------------------------------------------------------------------------------------------- //

        // Name:        DrawBitmap
        // Description: Draws a bitmap on the display starting at the current cursor position
        // Arguments:   Bitmap - Pointer to the array
        //              Lenght - Lenght of the array (0 to PCD8544_MAXBYTES bytes)
        //              Mode - Pixel mode - lcd_pixel_mode_t value
        // Returns:     None
        void DrawBitmap (const uint8_t *Bitmap, uint16_t Length, lcd_pixel_mode_t Mode);

        // Name:        DrawPixel
        // Description: Draws a single pixel on the display
        // Arguments:   X - Column of the pixel (0 to PCD8544_COLUMNS)
        //              Y - Row of the pixel (0 to PCD8544_ROWS)
        //              Mode - Pixel mode - lcd_pixel_mode_t value
        // Returns:     None
        void DrawPixel (uint8_t X, uint8_t Y, lcd_pixel_mode_t Mode);

        // Name:        DrawLine
        // Description: Draws a line between two points on the display using DDA algorithm
        // Arguments:   Xi, Yi - Absolute pixel coordinates for line origin
        //              Xf, Yf - Absolute pixel coordinates for line end
        //              Mode - Pixel mode - lcd_pixel_mode_t value
        // Returns:     None
        void DrawLine (uint8_t Xi, uint8_t Yi, uint8_t Xf, uint8_t Yf, lcd_pixel_mode_t Mode);

        // Name:        DrawRectangle
        // Description: Draws a rectangle between two diagonal points on the display
        // Arguments:   Xi, Yi - Absolute pixel coordinates for the first point
        //              Xf, Yf - Absolute pixel coordinates for end point
        //              Mode - Pixel mode - lcd_pixel_mode_t value
        // Returns:     None
        void DrawRectangle (uint8_t Xi, uint8_t Yi, uint8_t Xf, uint8_t Yf, lcd_pixel_mode_t Px_Mode);

        // Name:        DrawFilledRectangle
        // Description: Draws a filled rectangle between two diagonal points on the display
        // Arguments:   Xi, Yi - Absolute pixel coordinates for the first point
        //              Xf, Yf - Absolute pixel coordinates for end point
        //              Mode - Pixel mode - lcd_pixel_mode_t value
        // Returns:     None
        void DrawFilledRectangle (uint8_t Xi, uint8_t Yi, uint8_t Xf, uint8_t Yf, lcd_pixel_mode_t Px_Mode);

        // Name:        DrawCircle
        // Description: Draw a circle using Bresenham algorithm
        // Arguments:   Xc, Yc - Absolute pixel coordinates for the center of the circle
        //              Radius - Radius of the circle in pixels
        //              Px_Mode - Pixel mode - Can be either LCD_PIXEL_OFF, LCD_PIXEL_ON or LCD_PIXEL_XOR
        // Returns:     None
        void DrawCircle (uint8_t Xc, uint8_t Yc, uint8_t Radius, lcd_pixel_mode_t Mode);
};

// ------------------------------------------------------------------------------------------------------- //
// PCD8544 LCD controller fonts - Font 0 - Regular characters - 5x8 px
// ------------------------------------------------------------------------------------------------------- //

static const char Lcd_Font_0 [][5] =
{
        {0x00, 0x00, 0x00, 0x00, 0x00}, // 0x20
        {0x00, 0x00, 0x5f, 0x00, 0x00}, // 0x21 !
        {0x00, 0x07, 0x00, 0x07, 0x00}, // 0x22 "
        {0x14, 0x7f, 0x14, 0x7f, 0x14}, // 0x23 #
        {0x24, 0x2a, 0x7f, 0x2a, 0x12}, // 0x24 $
        {0x23, 0x13, 0x08, 0x64, 0x62}, // 0x25 %
        {0x36, 0x49, 0x55, 0x22, 0x50}, // 0x26 &
        {0x00, 0x05, 0x03, 0x00, 0x00}, // 0x27 '
        {0x00, 0x1c, 0x22, 0x41, 0x00}, // 0x28 (
        {0x00, 0x41, 0x22, 0x1c, 0x00}, // 0x29 )
        {0x14, 0x08, 0x3e, 0x08, 0x14}, // 0x2a *
        {0x08, 0x08, 0x3e, 0x08, 0x08}, // 0x2b +
        {0x00, 0x50, 0x30, 0x00, 0x00}, // 0x2c ,
        {0x08, 0x08, 0x08, 0x08, 0x08}, // 0x2d -
        {0x00, 0x60, 0x60, 0x00, 0x00}, // 0x2e .
        {0x20, 0x10, 0x08, 0x04, 0x02}, // 0x2f /
        {0x3e, 0x51, 0x49, 0x45, 0x3e}, // 0x30 0
        {0x00, 0x42, 0x7f, 0x40, 0x00}, // 0x31 1
        {0x42, 0x61, 0x51, 0x49, 0x46}, // 0x32 2
        {0x21, 0x41, 0x45, 0x4b, 0x31}, // 0x33 3
        {0x18, 0x14, 0x12, 0x7f, 0x10}, // 0x34 4
        {0x27, 0x45, 0x45, 0x45, 0x39}, // 0x35 5
        {0x3c, 0x4a, 0x49, 0x49, 0x30}, // 0x36 6
        {0x01, 0x71, 0x09, 0x05, 0x03}, // 0x37 7
        {0x36, 0x49, 0x49, 0x49, 0x36}, // 0x38 8
        {0x06, 0x49, 0x49, 0x29, 0x1e}, // 0x39 9
        {0x00, 0x36, 0x36, 0x00, 0x00}, // 0x3a :
        {0x00, 0x56, 0x36, 0x00, 0x00}, // 0x3b ;
        {0x08, 0x14, 0x22, 0x41, 0x00}, // 0x3c <
        {0x14, 0x14, 0x14, 0x14, 0x14}, // 0x3d =
        {0x00, 0x41, 0x22, 0x14, 0x08}, // 0x3e >
        {0x02, 0x01, 0x51, 0x09, 0x06}, // 0x3f ?
        {0x32, 0x49, 0x79, 0x41, 0x3e}, // 0x40 @
        {0x7e, 0x11, 0x11, 0x11, 0x7e}, // 0x41 A
        {0x7f, 0x49, 0x49, 0x49, 0x36}, // 0x42 B
        {0x3e, 0x41, 0x41, 0x41, 0x22}, // 0x43 C
        {0x7f, 0x41, 0x41, 0x22, 0x1c}, // 0x44 D
        {0x7f, 0x49, 0x49, 0x49, 0x41}, // 0x45 E
        {0x7f, 0x09, 0x09, 0x09, 0x01}, // 0x46 F
        {0x3e, 0x41, 0x49, 0x49, 0x7a}, // 0x47 G
        {0x7f, 0x08, 0x08, 0x08, 0x7f}, // 0x48 H
        {0x00, 0x41, 0x7f, 0x41, 0x00}, // 0x49 I
        {0x20, 0x40, 0x41, 0x3f, 0x01}, // 0x4a J
        {0x7f, 0x08, 0x14, 0x22, 0x41}, // 0x4b K
        {0x7f, 0x40, 0x40, 0x40, 0x40}, // 0x4c L
        {0x7f, 0x02, 0x0c, 0x02, 0x7f}, // 0x4d M
        {0x7f, 0x04, 0x08, 0x10, 0x7f}, // 0x4e N
        {0x3e, 0x41, 0x41, 0x41, 0x3e}, // 0x4f O
        {0x7f, 0x09, 0x09, 0x09, 0x06}, // 0x50 P
        {0x3e, 0x41, 0x51, 0x21, 0x5e}, // 0x51 Q
        {0x7f, 0x09, 0x19, 0x29, 0x46}, // 0x52 R
        {0x46, 0x49, 0x49, 0x49, 0x31}, // 0x53 S
        {0x01, 0x01, 0x7f, 0x01, 0x01}, // 0x54 T
        {0x3f, 0x40, 0x40, 0x40, 0x3f}, // 0x55 U
        {0x1f, 0x20, 0x40, 0x20, 0x1f}, // 0x56 V
        {0x3f, 0x40, 0x38, 0x40, 0x3f}, // 0x57 W
        {0x63, 0x14, 0x08, 0x14, 0x63}, // 0x58 X
        {0x07, 0x08, 0x70, 0x08, 0x07}, // 0x59 Y
        {0x61, 0x51, 0x49, 0x45, 0x43}, // 0x5a Z
        {0x00, 0x7f, 0x41, 0x41, 0x00}, // 0x5b [
        {0x02, 0x04, 0x08, 0x10, 0x20}, // 0x5c Backslash
        {0x00, 0x41, 0x41, 0x7f, 0x00}, // 0x5d ]
        {0x04, 0x02, 0x01, 0x02, 0x04}, // 0x5e ^
        {0x40, 0x40, 0x40, 0x40, 0x40}, // 0x5f _
        {0x00, 0x01, 0x02, 0x04, 0x00}, // 0x60 `
        {0x20, 0x54, 0x54, 0x54, 0x78}, // 0x61 a
        {0x7f, 0x48, 0x44, 0x44, 0x38}, // 0x62 b
        {0x38, 0x44, 0x44, 0x44, 0x20}, // 0x63 c
        {0x38, 0x44, 0x44, 0x48, 0x7f}, // 0x64 d
        {0x38, 0x54, 0x54, 0x54, 0x18}, // 0x65 e
        {0x08, 0x7e, 0x09, 0x01, 0x02}, // 0x66 f
        {0x0c, 0x52, 0x52, 0x52, 0x3e}, // 0x67 g
        {0x7f, 0x08, 0x04, 0x04, 0x78}, // 0x68 h
        {0x00, 0x44, 0x7d, 0x40, 0x00}, // 0x69 i
        {0x20, 0x40, 0x44, 0x3d, 0x00}, // 0x6a j
        {0x7f, 0x10, 0x28, 0x44, 0x00}, // 0x6b k
        {0x00, 0x41, 0x7f, 0x40, 0x00}, // 0x6c l
        {0x7c, 0x04, 0x18, 0x04, 0x78}, // 0x6d m
        {0x7c, 0x08, 0x04, 0x04, 0x78}, // 0x6e n
        {0x38, 0x44, 0x44, 0x44, 0x38}, // 0x6f o
        {0x7c, 0x14, 0x14, 0x14, 0x08}, // 0x70 p
        {0x08, 0x14, 0x14, 0x18, 0x7c}, // 0x71 q
        {0x7c, 0x08, 0x04, 0x04, 0x08}, // 0x72 r
        {0x48, 0x54, 0x54, 0x54, 0x20}, // 0x73 s
        {0x04, 0x3f, 0x44, 0x40, 0x20}, // 0x74 t
        {0x3c, 0x40, 0x40, 0x20, 0x7c}, // 0x75 u
        {0x1c, 0x20, 0x40, 0x20, 0x1c}, // 0x76 v
        {0x3c, 0x40, 0x30, 0x40, 0x3c}, // 0x77 w
        {0x44, 0x28, 0x10, 0x28, 0x44}, // 0x78 x
        {0x0c, 0x50, 0x50, 0x50, 0x3c}, // 0x79 y
        {0x44, 0x64, 0x54, 0x4c, 0x44}, // 0x7a z
        {0x00, 0x08, 0x36, 0x41, 0x00}, // 0x7b {
        {0x00, 0x00, 0x7f, 0x00, 0x00}, // 0x7c |
        {0x00, 0x41, 0x36, 0x08, 0x00}, // 0x7d }
        {0x10, 0x08, 0x08, 0x10, 0x08}, // 0x7e ~
        {0x00, 0x06, 0x09, 0x09, 0x06}, // 0x7f DEL
};

// ------------------------------------------------------------------------------------------------------- //
// PCD8544 LCD controller fonts - Font 1 - Small characters - 5x8 px
// ------------------------------------------------------------------------------------------------------- //

static const char Lcd_Font_1 [][5] =
{
        {0x00, 0x00, 0x00, 0x00, 0x00}, // 0x20
        {0x00, 0x00, 0x5C, 0x00, 0x00}, // 0x21 !
        {0x00, 0x0C, 0x00, 0x0C, 0x00}, // 0x22 "
        {0x28, 0x7C, 0x28, 0x7C, 0x28}, // 0x23 #
        {0x48, 0x54, 0x7C, 0x54, 0x24}, // 0x24 $
        {0x4C, 0x24, 0x10, 0x48, 0x64}, // 0x25 %
        {0x28, 0x54, 0x58, 0x20, 0x50}, // 0x26 &
        {0x00, 0x00, 0x0C, 0x00, 0x00}, // 0x27 '
        {0x00, 0x38, 0x44, 0x00, 0x00}, // 0x28 (
        {0x00, 0x00, 0x44, 0x38, 0x00}, // 0x29 )
        {0x10, 0x54, 0x38, 0x54, 0x10}, // 0x2a *
        {0x10, 0x10, 0x7C, 0x10, 0x10}, // 0x2b +
        {0x00, 0x20, 0x60, 0x00, 0x00}, // 0x2c ,
        {0x10, 0x10, 0x10, 0x10, 0x10}, // 0x2d -
        {0x00, 0x60, 0x60, 0x00, 0x00}, // 0x2e .
        {0x40, 0x20, 0x10, 0x08, 0x04}, // 0x2f /
        {0x38, 0x64, 0x54, 0x4C, 0x38}, // 0x30 0
        {0x00, 0x48, 0x7C, 0x40, 0x00}, // 0x31 1
        {0x48, 0x64, 0x54, 0x54, 0x48}, // 0x32 2
        {0x44, 0x54, 0x54, 0x54, 0x28}, // 0x33 3
        {0x30, 0x28, 0x24, 0x7C, 0x20}, // 0x34 4
        {0x5C, 0x54, 0x54, 0x54, 0x24}, // 0x35 5
        {0x38, 0x54, 0x54, 0x54, 0x20}, // 0x36 6
        {0x04, 0x04, 0x64, 0x14, 0x0C}, // 0x37 7
        {0x28, 0x54, 0x54, 0x54, 0x28}, // 0x38 8
        {0x08, 0x54, 0x54, 0x54, 0x38}, // 0x39 9
        {0x00, 0x6C, 0x6C, 0x00, 0x00}, // 0x3a :
        {0x00, 0x2C, 0x6C, 0x00, 0x00}, // 0x3b ;
        {0x00, 0x10, 0x28, 0x44, 0x00}, // 0x3c <
        {0x28, 0x28, 0x28, 0x28, 0x28}, // 0x3d =
        {0x00, 0x44, 0x28, 0x10, 0x00}, // 0x3e >
        {0x08, 0x04, 0x44, 0x14, 0x08}, // 0x3f ?
        {0x38, 0x44, 0x74, 0x54, 0x58}, // 0x40 @
        {0x78, 0x14, 0x14, 0x14, 0x78}, // 0x41 A
        {0x7C, 0x54, 0x54, 0x54, 0x28}, // 0x42 B
        {0x38, 0x44, 0x44, 0x44, 0x28}, // 0x43 C
        {0x7C, 0x44, 0x44, 0x44, 0x38}, // 0x44 D
        {0x7C, 0x54, 0x54, 0x54, 0x44}, // 0x45 E
        {0x7C, 0x14, 0x14, 0x14, 0x04}, // 0x46 F
        {0x38, 0x44, 0x44, 0x54, 0x70}, // 0x47 G
        {0x7C, 0x10, 0x10, 0x10, 0x7C}, // 0x48 H
        {0x00, 0x44, 0x7C, 0x44, 0x00}, // 0x49 I
        {0x20, 0x40, 0x40, 0x40, 0x7C}, // 0x4a J
        {0x7C, 0x10, 0x10, 0x28, 0x44}, // 0x4b K
        {0x7C, 0x40, 0x40, 0x40, 0x40}, // 0x4c L
        {0x7C, 0x08, 0x10, 0x08, 0x7C}, // 0x4d M
        {0x7C, 0x08, 0x10, 0x20, 0x7C}, // 0x4e N
        {0x38, 0x44, 0x44, 0x44, 0x38}, // 0x4f O
        {0x7C, 0x14, 0x14, 0x14, 0x08}, // 0x50 P
        {0x38, 0x44, 0x54, 0x24, 0x58}, // 0x51 Q
        {0x7C, 0x14, 0x14, 0x34, 0x48}, // 0x52 R
        {0x48, 0x54, 0x54, 0x54, 0x24}, // 0x53 S
        {0x04, 0x04, 0x7C, 0x04, 0x04}, // 0x54 T
        {0x3C, 0x40, 0x40, 0x40, 0x3C}, // 0x55 U
        {0x1C, 0x20, 0x40, 0x20, 0x1C}, // 0x56 V
        {0x7C, 0x20, 0x10, 0x20, 0x7C}, // 0x57 W
        {0x44, 0x28, 0x10, 0x28, 0x44}, // 0x58 X
        {0x04, 0x08, 0x70, 0x08, 0x04}, // 0x59 Y
        {0x44, 0x64, 0x54, 0x4C, 0x44}, // 0x5a Z
        {0x00, 0x7C, 0x44, 0x44, 0x00}, // 0x5b [
        {0x04, 0x08, 0x10, 0x20, 0x40}, // 0x5c Backslash
        {0x00, 0x44, 0x44, 0x7C, 0x00}, // 0x5d ]
        {0x10, 0x08, 0x04, 0x08, 0x10}, // 0x5e ^
        {0x40, 0x40, 0x40, 0x40, 0x40}, // 0x5f _
        {0x00, 0x0C, 0x10, 0x00, 0x00}, // 0x60 `
        {0x30, 0x48, 0x48, 0x48, 0x70}, // 0x61 a
        {0x7C, 0x48, 0x48, 0x48, 0x30}, // 0x62 b
        {0x30, 0x48, 0x48, 0x48, 0x48}, // 0x63 c
        {0x30, 0x48, 0x48, 0x48, 0x7C}, // 0x64 d
        {0x30, 0x48, 0x58, 0x58, 0x50}, // 0x65 e
        {0x20, 0x78, 0x24, 0x08, 0x00}, // 0x66 f
        {0x10, 0xA8, 0xA8, 0xA8, 0x70}, // 0x67 g
        {0x7C, 0x08, 0x08, 0x08, 0x70}, // 0x68 h
        {0x00, 0x50, 0x74, 0x40, 0x00}, // 0x69 i
        {0x40, 0x80, 0x90, 0x74, 0x00}, // 0x6a j
        {0x7C, 0x10, 0x30, 0x48, 0x00}, // 0x6b k
        {0x00, 0x44, 0x7C, 0x40, 0x00}, // 0x6c l
        {0x78, 0x08, 0x70, 0x08, 0x70}, // 0x6d m
        {0x78, 0x08, 0x08, 0x08, 0x70}, // 0x6e n
        {0x30, 0x48, 0x48, 0x48, 0x30}, // 0x6f o
        {0xF8, 0x48, 0x48, 0x48, 0x30}, // 0x70 p
        {0x30, 0x48, 0x48, 0x48, 0xF8}, // 0x71 q
        {0x78, 0x10, 0x08, 0x08, 0x08}, // 0x72 r
        {0x50, 0x58, 0x58, 0x58, 0x28}, // 0x73 s
        {0x08, 0x3C, 0x48, 0x40, 0x20}, // 0x74 t
        {0x38, 0x40, 0x40, 0x40, 0x78}, // 0x75 u
        {0x18, 0x20, 0x40, 0x20, 0x18}, // 0x76 v
        {0x38, 0x40, 0x38, 0x40, 0x38}, // 0x77 w
        {0x50, 0x48, 0x30, 0x48, 0x28}, // 0x78 x
        {0x18, 0xA0, 0xA0, 0xA0, 0x78}, // 0x79 y
        {0x48, 0x68, 0x68, 0x58, 0x48}, // 0x7a z
        {0x00, 0x10, 0x38, 0x44, 0x44}, // 0x7b {
        {0x00, 0x00, 0x7C, 0x00, 0x00}, // 0x7c |
        {0x44, 0x44, 0x38, 0x10, 0x00}, // 0x7d }
        {0x10, 0x08, 0x10, 0x20, 0x10}, // 0x7e ~
        {0x00, 0x70, 0x50, 0x70, 0x00}, // 0x7f DEL
};

// ------------------------------------------------------------------------------------------------------- //
// PCD8544 LCD controller fonts - Font 2 - Big numbers - 9x16 px
// ------------------------------------------------------------------------------------------------------- //

static const char Lcd_FontBig [][18] =
{
        {0xF0, 0xF8, 0x0C, 0x04, 0x04, 0x04, 0x0C, 0xF8, 0xF0,
        0x0F, 0x1F, 0x30, 0x20, 0x20, 0x20, 0x30, 0x1F, 0x0F}, // 0x30 0

        {0x00, 0x00, 0x10, 0x10, 0xFC, 0xFC, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x20, 0x20, 0x3F, 0x3F, 0x20, 0x20, 0x00}, // 0x31 1

        {0x18, 0x1C, 0x04, 0x04, 0x04, 0x04, 0x8C, 0xF8, 0x70,
        0x20, 0x30, 0x38, 0x2C, 0x26, 0x23, 0x21, 0x20, 0x20}, // 0x32 2

        {0x18, 0x1C, 0x04, 0x84, 0x84, 0x84, 0xCC, 0x78, 0x30,
        0x18, 0x38, 0x20, 0x20, 0x20, 0x20, 0x31, 0x1F, 0x0E}, // 0x33 3

        {0x00, 0x80, 0x40, 0x20, 0x10, 0x08, 0xFC, 0xFC, 0x00,
        0x03, 0x02, 0x02, 0x02, 0x02, 0x02, 0x3F, 0x3F, 0x02}, // 0x34 4

        {0x00, 0x7C, 0x7C, 0x44, 0x44, 0x44, 0xC4, 0x84, 0x04,
        0x18, 0x38, 0x20, 0x20, 0x20, 0x20, 0x30, 0x1F, 0x0F}, // 0x35 5

        {0xE0, 0xF0, 0x58, 0x4C, 0x44, 0x44, 0xC4, 0x84, 0x00,
        0x0F, 0x1F, 0x30, 0x20, 0x20, 0x20, 0x30, 0x1F, 0x0F}, // 0x36 6

        {0x04, 0x04, 0x04, 0x04, 0x04, 0xC4, 0xF4, 0x3C, 0x0C,
        0x00, 0x00, 0x30, 0x3C, 0x0F, 0x03, 0x00, 0x00, 0x00}, // 0x37 7

        {0x30, 0x78, 0xCC, 0x84, 0x84, 0x84, 0xCC, 0x78, 0x30,
        0x0E, 0x1F, 0x31, 0x20, 0x20, 0x20, 0x31, 0x1F, 0x0E}, // 0x38 8

        {0xF0, 0xF8, 0x0C, 0x04, 0x04, 0x04, 0x0C, 0xF8, 0xF0,
        0x00, 0x21, 0x23, 0x22, 0x22, 0x32, 0x1A, 0x0F, 0x07}, // 0x39 9

        {0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00,
        0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00}, // 0x3A -

        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x38, 0x38, 0x38, 0x00, 0x00, 0x00}, // 0x3B .
};

// ------------------------------------------------------------------------------------------------------- //

#ifdef __cplusplus
}
#endif

#endif

// ------------------------------------------------------------------------------------------------------- //

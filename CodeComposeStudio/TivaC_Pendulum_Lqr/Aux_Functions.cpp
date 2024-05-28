// ------------------------------------------------------------------------------------------------------- //

// Auxiliary functions
// Version: 1.0
// Author: Renan Duarte
// E-mail: duarte.renan@hotmail.com
// Date:   14/05/2024

// ------------------------------------------------------------------------------------------------------- //

// ------------------------------------------------------------------------------------------------------- //
// Includes
// ------------------------------------------------------------------------------------------------------- //

// Defines and macros
#include "Aux_Functions.hpp"
#include <stdint.h>
#include <math.h>

// ------------------------------------------------------------------------------------------------------- //
// Functions definitions
// ------------------------------------------------------------------------------------------------------- //

// Name:        Map
// Description: Maps a value from one range to another
// Arguments:   Value - Value to be mapped
//              InMin - Minimum value of the input range
//              InMax - Maximum value of the input range
//              OutMin - Minimum value of the output range
//              OutMax - Maximum value of the input range
// Returns:     Scaled value

float Aux::Map(float Value, float InMin, float InMax, float OutMin, float OutMax)
{
    // Constrain value
    if (Value >= InMax)
        return OutMax;
    else if (Value <= InMin)
        return OutMin;

    // Scale value
    return (Value - InMin) * (OutMax - OutMin) / (InMax - InMin) + OutMin;
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        Strcpy
// Description: Copy one string to another
// Arguments:   Destination - Destination string
//              Source - Source string
// Returns:     Pointer to the beginning of the destination string

char* Aux::Strcpy(char* Destination, const char* Source)
{
    // Save the original destination pointer
    char* original_dest = Destination;

    // Copy characters from source to destination until null terminator is encountered
    while (*Source != '\0')
        *Destination++ = *Source++;

    // Null-terminate the destination string
    *Destination = '\0';

    // Return a pointer to the beginning of the destination string
    return original_dest;
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        StrReverse
// Description: Reverse all characters in the string between the given pointers
// Arguments:   Begin - Pointer to the beginning of the string
//              End - Pointer to the end of the string
// Returns:     None

void Aux::StrReverse(char* Begin, char* End)
{
    while (Begin < End)
    {
        char C = *Begin;
        *Begin++ = *End;
        *End-- = C;
    }
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        L2Str
// Description: Converts a given integer to a string representation
//              If the integer is negative, the string includes a negative sign
// Arguments:   Number - The integer to be converted (int32_t format)
//              String - Pointer to the output buffer where the string representation will be stored
// Returns:     The number of characters written to the string buffer

uint8_t Aux::L2Str(int32_t Number, char* String)
{
    char* Pointer = String;

    // Handle negative numbers
    if (Number < 0)
    {
        *Pointer++ = '-';
        Number = -Number;
    }

    // Convert digits to characters from the end of the string
    do
    {
        *Pointer++ = (Number % 10) + '0';
        Number = Number / 10;
    } while (Number);

    // Reverse the string (excluding the negative sign if present)
    Aux::StrReverse(String + (*String == '-'), Pointer - 1);

    // Null-terminate the string
    *Pointer = '\0';

    // Return the number of digits in the string
    return Pointer - String;
}

// ------------------------------------------------------------------------------------------------------- //

// Name:        F2Str
// Description: Converts a float number to a string with a specified number of decimal places
//              Number will be rounded at the last specified decimal place
// Arguments:   Number - The float number to be converted
//              String - Pointer to the output buffer where the string representation will be stored
//              DecPlaces - The number of decimal places in the resulting string
// Returns:     The number of characters written to the string buffer

uint8_t Aux::F2Str(float Number, char* String, uint8_t DecPlaces)
{
    // Variables
    uint8_t Length = 0;

    // Check if number is infinity (positive or negative)
    if(isinf(Number))
    {
        Aux::Strcpy(String, "Inf");
        return 3;
    }

    // Check if number is NaN
    else if (isnan(Number))
    {
        Aux::Strcpy(String, "NaN");
        return 3;
    }

    // Define rounding factor
    float RoundingFactor = (Number < 0 ? -0.5 : 0.5);
    for (uint8_t Counter = 0; Counter < DecPlaces; Counter++)
        RoundingFactor *= 0.1;

    // Perform rounding
    Number += RoundingFactor;

    // Check signal
    if (Number < 0)
    {
        String[Length++] = '-';
        Number *= -1;
    }

    // Calculate dot location
    uint8_t DotLocation = 0;
    while (Number >= 10)
    {
        Number /= 10;
        DotLocation++;
    }

    // Convert float to string
    uint8_t Iterations = DotLocation + DecPlaces;
    for (uint8_t Counter = 0; Counter <= Iterations; Counter++)
    {
        // Convert current digit to char
        String[Length++] = '0' + (uint8_t)Number;

        // Insert dot if necessary
        if (DotLocation-- == 0)
            String[Length++] = '.';

        // Remove current digit
        Number -= (uint8_t)Number;
        Number *= 10;
    }

    // Null-terminate the string
    String[Length] = '\0';

    // Return the number of digits in the string
    return Length;
}

// ------------------------------------------------------------------------------------------------------- //

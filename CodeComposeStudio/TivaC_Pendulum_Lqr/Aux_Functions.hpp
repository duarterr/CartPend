// ------------------------------------------------------------------------------------------------------- //

// UART library for TivaC devices
// Version: 1.0
// Author: Renan Duarte
// E-mail: duarte.renan@hotmail.com
// Date:   13/05/2024

// ------------------------------------------------------------------------------------------------------- //

#ifndef AUX_FUNCTIONS_H_
#define AUX_FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

// ------------------------------------------------------------------------------------------------------- //
// Includes
// ------------------------------------------------------------------------------------------------------- //

// Standard libraries
#include <stdint.h>

// ------------------------------------------------------------------------------------------------------- //
// Class prototype
// ------------------------------------------------------------------------------------------------------- //

class Aux
{
    // --------------------------------------------------------------------------------------------------- //
    // Public members
    // --------------------------------------------------------------------------------------------------- //

    public:

        // Name:        Map
        // Description: Maps a value from one range to another
        // Arguments:   Value - Value to be mapped
        //              InMin - Minimum value of the input range
        //              InMax - Maximum value of the input range
        //              OutMin - Minimum value of the output range
        //              OutMax - Maximum value of the input range
        // Returns:     Scaled value
        static float Map(float Value, float InMin, float InMax, float OutMin, float OutMax);

        // Name:        Strcpy
        // Description: Copy one string to another
        // Arguments:   Destination - Destination string
        //              Source - Source string
        // Returns:     Pointer to the beginning of the destination string
        static char* Strcpy(char* Destination, const char* Source);

        // Name:        StrReverse
        // Description: Reverse all characters in the string between the given pointers
        // Arguments:   Begin - Pointer to the beginning of the string
        //              End - Pointer to the end of the string
        // Returns:     None
        static void StrReverse(char* Begin, char* End);

        // Name:        L2Str
        // Description: Converts a given integer to a string representation
        //              If the integer is negative, the string includes a negative sign
        // Arguments:   Number - The integer to be converted (int32_t format)
        //              String - Pointer to the output buffer where the string representation will be stored
        // Returns:     The number of characters written to the string buffer
        static uint8_t L2Str(int32_t Number, char* String);

        // Name:        F2Str
        // Description: Converts a float number to a string with a specified number of decimal places
        //              Number will be rounded at the last specified decimal place
        // Arguments:   Number - The float number to be converted
        //              String - Pointer to the output buffer where the string representation will be stored
        //              DecPlaces - The number of decimal places in the resulting string
        // Returns:     The number of characters written to the string buffer
        static uint8_t F2Str(float Number, char* String, uint8_t DecPlaces);
};

// ------------------------------------------------------------------------------------------------------- //

#ifdef __cplusplus
}
#endif

#endif

// ------------------------------------------------------------------------------------------------------- //

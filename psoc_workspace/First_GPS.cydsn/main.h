/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
/*******************************************************************************
* File Name: main.h
*
*******************************************************************************/

#ifndef MAIN_H
#define MAIN_H
    
#include <project.h>
#include <stdio.h>

#define bool uint8_t
#define false 0
#define true 1
#define LOW 0
#define HIGH 1
	
typedef enum
{
Initialising = 0, ///< Initialising. Initial default value until init() is called
Sleep,            ///< Hardware is in low power sleep mode (if supported)
Idle,             ///< Idle
Tx,               ///< Transmitting messages
Rx,
U                ///< Receiving messages
} Mode;


// Helper functions:
uint32_t millis();					// Function to return runtime in milliseconds

#endif /* MAIN_H */

/* [] END OF FILE */


#include "main.h"
#include "USBTTY.h"
#include <USBUART.h>
#include <stdarg.h>
#include <stdio.h>

uint8 oldDTR = 0;					// Var to remember state of DTR


/* Routine to do "printf" to USB UART */
/* NB: !!! Buffer will overflow if result has more chars than msg array! */
uint8 uprintf(const char* fmt, ...) {
	char msg[256];
	uint8 result;
	va_list a_list;
	va_start( a_list, fmt );
	result = vsprintf(msg, fmt, a_list);
	USBPutString(msg);
	return result;
}

/* Routine to put a string out to the USB UART when the device is ready */
void USBPutString(char* s) {
	if(USBDTR()) {							// Ignore output request when no host terminal is present
		while( !USBUART_CDCIsReady() ) ;	// Otherwise, wait for host connection to be ready
		USBUART_PutString(s);				// Send the string
	}
}

/* Routine to put a char out to the USB UART when the device is ready */
void USBPutChar(char c) {
	if(USBDTR()) {							// Ignore output request when no host terminal is present
		while( !USBUART_CDCIsReady() ) ;	// Otherwise, wait for host connection to be ready
		USBUART_PutChar(c);					// Send the char
	}
}

/* Routine to get a char from the USB UART when one is ready */
char USBGetChar() {
	uint16 count;
	int16 limit = 1000;						// Prevent endless looping on zero-length packets
	uint8 c;
	do {
		while( !USBUART_DataIsReady() ) ;	// Wait for characters to be ready
//		CyDelay(2LU);    					// Workaround for old bug where characters may take some time to appear in Serial.read!
		count = USBUART_GetData(&c, 1);
	} while (count == 0 && --limit > 0);	// Allow count to be zero, but only for a limited time
	USBPutChar((char)c);					// Echo the char
//	LCD_Position(1u, 12u);					// Put char to LCD for debugging
//	sprintf(msg, "%c:%02hhx", c, c);		// for DEBUG
//	LCD_PrintString(msg);					// for DEBUG
	return (char)c;
}

/* Function to get the next input char from the host and convert it to upper case if necessary */
char USBGetUpperCaseChar() {
	char c;
	c = USBGetChar();						// Get one character (command) from host
	if(c >= 'a' && c <= 'z') {				// If lower case char
		c += 'A' - 'a';						// Convert to upper case 
	}
	return c;
}

/*
	Routine to get a numeric argument from the USB serial input
	Reads to the end of the input line or until it sees a 'term' character (from argument).
	Ignores spaces and newline characters.
	Returns zero if no digits are present.
	Interprets number as hexadecimal if preceded by 'X' or '0X'.
	Prints an error message if unrecognized characters are present.
	Crudely handles backspace or DEL (0x7F) by removing value of preceding digit and erasing it from the screen.
	'term' argument is a terminator character; (e.g. ',' to separate numbers with commas).  Null (0) arg -> no term.
	Sets high bit (bit 31) of result if non-null term is requested and found.
*/
unsigned long USBGetNum(char term) {
	unsigned long result = 0;
	char dig;
	bool hex = false;
	do {
		dig = USBGetUpperCaseChar();
	} while(dig == '0' || dig == ' ');		// Get 1st nonzero, nonspace char
	if (dig == 'X') {						// 'x' means hexadecimal
		hex = true;
		dig = USBGetUpperCaseChar();		// Get char following 'X'
	}
	while (dig != term && (dig != '\r')) {	// Get digits until we see a terminal or line end (CR)
		if(dig >= '0' && dig <= '9') {
			result = ((hex?16ul:10ul) * result) + (dig - '0');
		} else if(hex && dig >= 'A' && dig <= 'F') {
			result = 16ul * result + (dig - 'A') + 10;
		} else if(dig != 10 && dig != ' ') {	// Ignore LF and Space characters
			if(dig == '\b' || dig == 0x7f) {	// Backspace or DEL?
				result = result / (hex?16ul:10ul);	// Crude delete of preceding digit
				uprintf(" \b");					// Erase deleted digit from screen
				if(result == 0) return result;	// Return if we've deleted the whole value
			} else {
				uprintf("\n***Number error: char=%hc[%hXh] ignored\n", dig, dig);	// Otherwise, not a digit, report error
				USBClearInput();
			}
		}
		dig = USBGetUpperCaseChar();
	}
	if(dig == term && term != 0) {
		result = result | (1<<31);				// Indicate we stopped on the terminal char
	}
	return result;
}

/* Routine to clear the USB input buffer when it gets out of sync due to errors */
void USBClearInput() {
	uint8 bufr[16];
	while( USBUART_DataIsReady() ) {
		USBUART_GetData(&bufr[0], sizeof(bufr));
	}
}

/* Routine to get status of Data Terminal Ready (DTR) from USBUART */
uint8 USBDTR() {
	bool DTR;
	DTR = (USBUART_GetLineControl() & USBUART_LINE_CONTROL_DTR);
	oldDTR = DTR;
	return DTR;
}

uint8 OldUSBDTR() {
	return oldDTR;
}

uint8 SetUSBDTR(uint8 newDTR) {
	oldDTR = newDTR;
	return oldDTR;
}

/* [] END OF FILE */

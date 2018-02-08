#ifndef USBTTY_H
#define USBTTY_H
	
#include <cytypes.h>

/* Ansi function definitions */
uint8 uprintf(const char*, ...);
void USBPutString(char*);
void USBPutChar(char);
char USBGetChar();
char USBGetUpperCaseChar();
unsigned long USBGetNum(char term);
void USBClearInput();
uint8 USBDTR();
uint8 OldUSBDTR();
uint8 SetUSBDTR(uint8);

#endif
/* [] END OF FILE */

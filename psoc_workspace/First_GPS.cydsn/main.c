/* ========================================
 *
 * Copyright Eric Peters, 2017
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF Eric Peters.
 *
 * ========================================
*/

#include "main.h"
//#include "inttypes.h"
#include "CYASK.h"
#include "USBTTY.h"

// Includes for hardware components, interrupts, and pins:
#include "USBUART.h"
#include "VDAC8_P.h"			// Voltage DAC to create chirp signal (0-1 V p-to-p)
#include "VDAC8_M.h"			// Voltage DAC to create chirp signal (0-1 V p-to-p)
#include "PGA_P.h"				// Amplifiers with gain = 4 to increase output voltage
#include "TransducerDrive_P.h"	// Opamp follower used to drive the piezo speaker
#include "PGA_M.h"				// Amplifiers with gain = 4 to increase output voltage
#include "TransducerDrive_M.h"	// Opamp follower used to drive the piezo speaker
#include "PseudoRandom.h"		// Hardware pseudo-random number generator
#include "BitTime.h"			// Timer interrupt for radio bit sampling time and millis function
#include "radioRxPin.h"			// Receiver data pin
#include "radioTxPin.h"			// Transmitter data pin
#include "radioPttPin.h"		// "Push-to-talk" pin (if needed, enables transmitter when set high)
#include "ledPin.h"				// On-board LED used to display debugging info

CY_ISR_PROTO(BitTimeISR);		// Declare radio-sample and millis interrupt routine
CY_ISR_PROTO(ChirpSampleISR);	// Declare audio-chirp-sample interrupt routine

void chirp(const uint16_t f0, const uint16_t f1, const uint32_t len);	// Routine to generate a sine chirp signal
int16_t sine(const uint16_t x);	// Sine routine using lookup table
uint16 prs();					// Read and advance state of pseudo random sequence

// One quarter of a sine wave in 513 10-bit values (0-1023):
// Note: sineTbl is stored in Flash memory
const int16_t sineTbl[] = {		
0,3,6,9,13,16,19,22,25,28,31,35,38,41,44,47,50,53,56,60,63,66,69,72,75,78,82,
85,88,91,94,97,100,103,107,110,113,116,119,122,125,128,131,135,138,141,144,
147,150,153,156,159,163,166,169,172,175,178,181,184,187,190,193,196,200,203,
206,209,212,215,218,221,224,227,230,233,236,239,242,246,249,252,255,258,261,
264,267,270,273,276,279,282,285,288,291,294,297,300,303,306,309,312,315,318,
321,324,327,330,333,336,339,342,345,348,351,353,356,359,362,365,368,371,374,
377,380,383,386,389,391,394,397,400,403,406,409,412,415,417,420,423,426,429,
432,435,437,440,443,446,449,452,454,457,460,463,466,468,471,474,477,479,482,
485,488,491,493,496,499,502,504,507,510,512,515,518,521,523,526,529,531,534,
537,539,542,545,547,550,553,555,558,560,563,566,568,571,574,576,579,581,584,
586,589,592,594,597,599,602,604,607,609,612,614,617,619,622,624,627,629,632,
634,637,639,642,644,647,649,651,654,656,659,661,663,666,668,671,673,675,678,
680,682,685,687,689,692,694,696,699,701,703,705,708,710,712,714,717,719,721,
723,726,728,730,732,734,737,739,741,743,745,747,750,752,754,756,758,760,762,
764,766,768,771,773,775,777,779,781,783,785,787,789,791,793,795,797,799,801,
803,805,806,808,810,812,814,816,818,820,822,824,825,827,829,831,833,835,836,
838,840,842,844,845,847,849,851,852,854,856,858,859,861,863,864,866,868,869,
871,873,874,876,877,879,881,882,884,885,887,889,890,892,893,895,896,898,899,
901,902,904,905,907,908,909,911,912,914,915,917,918,919,921,922,923,925,926,
927,929,930,931,933,934,935,937,938,939,940,941,943,944,945,946,948,949,950,
951,952,953,954,956,957,958,959,960,961,962,963,964,965,966,967,968,969,970,
971,972,973,974,975,976,977,978,979,980,981,982,983,983,984,985,986,987,988,
988,989,990,991,992,992,993,994,995,995,996,997,997,998,999,999,1000,1001,1001,
1002,1003,1003,1004,1005,1005,1006,1006,1007,1007,1008,1008,1009,1010,1010,
1011,1011,1011,1012,1012,1013,1013,1014,1014,1015,1015,1015,1016,1016,1016,
1017,1017,1017,1018,1018,1018,1019,1019,1019,1019,1020,1020,1020,1020,1021,
1021,1021,1021,1021,1022,1022,1022,1022,1022,1022,1022,1023,1023,1023,1023,
1023,1023,1023,1023,1023,1023,1023
};

const uint32_t sample_rate = 62500;
// Parameters to create "chirp" output (rising or falling frequency sine wave):
// The 1st variable (sample_x65536) represents time along the sine wave (i.e. generates argument to the "sine" function)
// The 2nd variable (sample_incr_x65536) is how much to increment the first variable after each sample is generated.  Directly proportional to generated frequency
// The 3rd variable (sample_incr_incr_x65536) is how much to increment the second variable after each sample.  Causes frequency to slowly change (chirp)
// The 4th variable is the length of the total chirp in samples
// The 5th variable is the number of samples remaining before we are done generating this chirp (used by interrupt routine to know when to stop)
uint32_t sample_x65536;				// Sample position (i.e. "time") in ticks in sine wave, multiplied by 65536 (to create 16 fraction bits)
uint32_t sample_incr_x65536;		// Number of table positions to increase "sample" each tick, mult. by 65536 (for 16 fraction bits)
int32_t  sample_incr_incr_x65536;	// Amount to change (+/-) sample_incr_x65536 each tick (2nd derivitive of "sample" position), causes chirp
uint32_t sample_count_max;			// Number of samples in complete ping (at sample_rate)
uint32_t sample_count;				// Number of samples remaining in current chirp (at sample_rate)

// Parameters describing the physical chirp (starting frequency, ending frequency, length in time, and volume level):
uint16_t f0 = 10000;				// Starting frequency of chirp in Hz
uint16_t f1 = 20000;				// Ending frequency of chirp in Hz
uint32_t length = 1000;				// Length of timed operations in milliseconds (e.g. beep)

	
// Transmit definitions
char msg[] = "Hello World  !";	// Message to be transmitted on radio (note space for sequence digit)

Mode mode = Idle;

int32_t lastIntCnt = 0;

int main()
{
	uint32_t temp;
	uint16_t i = 0;
	char c;
	char digit = '0';
	uint8_t led = 0;

	// Radio receive definitions
	uint16_t badCnt, lastBadCnt = 0;		// Number of "bad messages" received (e.g. with bad CRC)
	uint8_t buf[15];						// Buffer for received messages
    uint8_t buflen = sizeof(buf);
    buf[buflen-1] = 0;						// Place null terminator into receive buffer
	
	msg[sizeof(msg)-1] = 0;					// Put null terminator char into transmit msg buffer
	
	BitTime_StartEx(BitTimeISR);			// Set up the 16 kHz timer interrupt
	ChirpSample_StartEx(ChirpSampleISR);	// Set up the 60 kHz audio sample interrupt
	
	// Init the Amplitude Shift Keying (ASK) radio interface for Cypress PSoC
	// Configure the radio interface:
	//		(2000 b/s, rcv pin read func, tx pin write func, ptt pin write func):
	// Note: rx pin 65 is port 2.3; tx pin 66 is port 2.4; and ptt pin 68 is port 2.5 on CY8CKIT-059
	//		led pin 63 is port 2.1, used for debugging display
	CYASK(2000, radioRxPin_Read, radioTxPin_Write, radioPttPin_Write);
	
    // Enable global interrupts
    CyGlobalIntEnable;
	
	// Startup built-in hardware devices
	VDAC8_P_Start();
	VDAC8_M_Start();
	PGA_P_Start();
	PGA_M_Start();
	TransducerDrive_P_Start();
	TransducerDrive_M_Start();
	PseudoRandom_Start();
	
	USBUART_Start(0, USBUART_DWR_POWER_OPERATION);	// Debug-interface (to host computer)
	while(USBUART_GetConfiguration()==0);			// Wait for USB to enumerate us
	USBUART_CDC_Init();								// Initialize the Host-to-PSoC connection
	
	USBPutString("Starting up!\n");
    if (CYASKinit())								// Init the radio routines
        USBPutString("Init success");
    else
        USBPutString("Init failed");
	
	mode = Idle;									// Begin in idle mode

    for(;;)
    {
		if(USBUART_DataIsReady()) {
			c = USBGetUpperCaseChar();
			if (c == 'T')							// "Transmit"
			{
				USBPutString("\nTransmit Mode:\n");
				CYASKsend((uint8_t *)"Xmt starting", 13);	// Send startup message on radio
				digit = '0';
				mode = Tx;
			}
			else if (c == 'R')						// "Receive"
			{
				USBPutString("\nReceive Mode:\n");
				mode = Rx;
			}
			else if (c == 'I')						// "Idle"
			{
				USBPutString("\nIdle Mode\n");
				mode = Idle;
			}
			else if (c == 'B')						// "Beep"
			{
				temp = USBGetNum(0);				// Get requested frequency in Hz
				PseudoRandom_WriteSeed(temp);		// Also use parameter to set PRS seed (temp code)
				while (temp < 100 || temp > 31000) {
					USBPutString("\nBeep:\nEnter frequency in Hz [100..31000]: ");
					temp = USBGetNum(0);
				};
				uprintf("\nBeep (%d Hz)\n", temp);
				chirp((uint16_t)temp, (uint16_t)temp, length);	// Generate beep (freq. in Hz, length in millis)
				mode = Idle;
			}
			else if (c == 'C')						// "Chirp"
			{
				uprintf("\nChirp (%d Hz to %d Hz)\n", f0, f1);
				chirp(f0, f1, length);				// Generate beep (freq. in Hz, length in millis)
				mode = Idle;
			}
			else if (c == 'F')						// "Frequency"
			{
				c = USBGetUpperCaseChar();
				if (c != 'A' && c != 'B') {
					uprintf("\n'F' command must specify 'A' or 'B' frequency to set.  Try again.\n\n");
					while (c != '\n') c = USBGetUpperCaseChar();	// Flush line input
				} else {
					temp = USBGetNum(0);
					if (temp == 0) {
						uprintf("\nFA = %d\nFB = %d\n\n", f0, f1);
					} else {
						if (c == 'A')
							f0 = temp;
						else // (c == 'B')
							f1 = temp;
						uprintf("\nFrequency %c set to %d Hz.\n", c, temp);
					}
				}
			}
			else if (c == 'L')						// "Length"
			{
				uint32_t oldLength = length;
				length = USBGetNum(0);
				if (length == 0) length = oldLength;
				uprintf("\nLength set to %d milliseconds.\n", length);
			}
			else if (c == 'X')						// "Tests"
			{
				// Pseudo Random Generator test
				temp = USBGetNum(0);				// Get requested seed
				PseudoRandom_WriteSeed(temp);		// Use parameter to set PRS seed 
				uprintf("\n%6d", (prs()-32768) / (1<<7));
				uprintf("\n%6d", (prs()-32768) / (1<<7));
				uprintf("\n%6d", (prs()-32768) / (1<<7));
				uprintf("\n%6d", (prs()-32768) / (1<<7));
				uprintf("\n%6d\n", (prs()-32768) / (1<<7));
			}
			else if (c == 'Q')						// "Quit"
			{
				USBPutString("\nQuitting\n");
				return 0;
			}
			else if (c == 'V')						// "Voltage"
			{
				temp = USBGetNum(0);
				while (temp > 5000) {
					USBPutString("\nVoltage:\nEnter Voltage in mV [0..4000]: ");
					temp = USBGetNum(0);
				};
				uprintf("\nVoltage set to %d.%03dV\n", temp/1000, temp%1000);
				temp = temp/16;					// Convert to 0..250 range
				if (temp>255) temp = 255;		// Limit for VDACs is 255
				VDAC8_P_SetValue(temp);			// Set plus-side voltage (0-1.020V)
				VDAC8_M_SetValue(temp);			// Set minus-side voltage
			}
			else if(c==' ' || c=='\n')			// Ignore 
			{
			}
			else USBPutString("\nUnknown command ignored.\n\n");
		}
		if(mode == Tx) {
			msg[12] = digit++;					// Create rotating digit 0 to 9
			if (digit > '9') digit = '0';
			CYASKsend((uint8_t *)msg, strlen(msg));
			CYASKwaitPacketSent();
			uprintf("%s\n", msg);
			CyDelay(500);
		}
		else if(mode == Rx) {
		    badCnt = CYASKrxBad();
		    if(badCnt != lastBadCnt)
		    {
				lastBadCnt = badCnt;
				uprintf("* * * * * * * BAD RCV COUNT * * * * * * *   --->  %d\n", badCnt);
				for (i=0; i<4; i++) {
					led = !led;
					ledPin_Write(led);			// Double blink the LED
					CyDelay(100);
				}
		    }
		    if (CYASKrecv(buf, &buflen)) 		// Non-blocking: Get message from radio if available
		    {
				// Message with a good checksum received, report it.
				uprintf("Message: %s\n", (char*)buf);
				led = !led;
				ledPin_Write(led);				// Toggle the LED
		    }			
		}
    }
}

// Beep generator
//
// Produce sine signal and send it to the transducer DACs as 10-bit values
void chirp(const uint16_t f0, const uint16_t f1, const uint32_t len)	// Generate a test beep frequency f and len milliseconds
{
	sample_count_max = sample_rate * len / 1000;	// Total number of samples in beep
	sample_x65536 = 0;
	// Do the following 2 lines of arithmetic carefully to avoid overflows while maintaining significant bits:
	sample_incr_x65536 = ((65536*f0) / sample_rate) * 2048;
	sample_incr_incr_x65536 = ((((65536*f1) / sample_rate) * 2048) - sample_incr_x65536) / sample_count_max;
	uprintf("\nsample_count_max = %lu\nsample_incr_x65536 = %lu (/65536 = %lu)\n"\
		"sample_incr_incr_x65536 = %ld (/65536 = %ld)\n\n", sample_count_max, sample_incr_x65536,
		sample_incr_x65536 / 65536, sample_incr_incr_x65536, sample_incr_incr_x65536 / 65536);
	sample_count = sample_count_max;				// Begin producing samples in interrupt routine
}

// Function to update pseudo-random number generator
uint16 prs() {
	for(int i = 16; i>0; i--)
		PseudoRandom_Step();
	return PseudoRandom_Read();
}

// Function to calculate signed integer 11-bit sin(x), using quarter-wave table "sineTbl[]"
// Argument x is in units of 1/2048 of a cycle (i.e. x=2048 corresponds to 2*Pi);
// Thus, this sine(x) function calculates int(1023 * sin(2*pi*x/2048))
// This function looks only at bits 0 thru 10 (i.e. 11 bits) of the x argument and ignores the rest,
// because the sinewave repeats: (time: 0 <= mod(x,2048) < 2048) 
// This function returns a signed value ranging from -1023 to 1023, where
// "0" represents zero volts, "-1023" is max negative voltage, and "+1023" is max positive voltage
int16_t sine(const uint16_t x) {
	int16_t s;
	uint16_t b = x & 0b111111111;	// b runs from 0 to 511 (1/4 of sine wave -> 0 to Pi/2)
	if((x & 1<<9) != 0)				// If 2nd or 4th quadrant (these quadrants are "mirror images" of 1st and 3rd quadrants)
		b = 512 - b;				// then b runs from 512 down to 1 (this is where b needs >=9 bits and sineTbl[] needs 513 entries!)
	s = sineTbl[b];					// Look up sine curve in table; result in range 0 thru 1023 inclusive (10 bits)
	if((x & 1<<10) != 0)			// If 3rd or 4th quadrant,
		s = -s; 					// then sine is negative (needs >=11 bits)
	return s;						// Return value -1023 to 1023 (i.e. an 11-bit signed value)
}

// Audio Sample Interrupt
//
// Interrupt to generate each audio sample (62.5 kHz sample rate)
CY_ISR(ChirpSampleISR)
{
	int16_t sin, s;
	if (sample_count > 0) {				// Any samples to produce?
		sample_count--;					// Count this sample
//		sin = sine((uint16_t)(sample_x65536 >> 16))/4;	// Compute 9-bit sine wave value for this sample (-255..255)
		sin = (prs()-32768) / (1<<7);	// Compute 9-bit pseudo-random audio sample 	(temp)
		s = (sin/2) + 128;				// "128" is middle voltage = "0 Volts" ("Offset binary") (N.B. s>=0)
		if (s==1) s = 0;				// Special case when 8-bit sine == -255
		VDAC8_P_SetValue(s);			// Set plus-side voltage (0-1.020V)
		VDAC8_M_SetValue(s - sin);		// Set minus-side voltage (diff P-M is instantaneous value of sine)
		sample_x65536 += sample_incr_x65536;			// Use sample increment to compute next position in sine table
		sample_incr_x65536 += sample_incr_incr_x65536;	// Use sample increment increment (2nd deriv.) to create chirp
	} else {
		VDAC8_P_SetValue(128);			// Set plus-side voltage to "zero"
		VDAC8_M_SetValue(128);			// Set minus-side voltage to "zero" (diff P-M is value of output)
	}
}


// Timer Interrupt
//
// Definitions to generate bit sample timer (16 kHz for 2 kbps @ 8 samples/bit) and 
// millisecond timer (counts milliseconds since starting)
//

#define milliDivisor 16							// Number of timer interrupts per millisecond
uint32_t millisCounter = 0;						// Runtime in milliseconds (rolls over in ~49 days)
static uint16_t downCounter = milliDivisor;		// Count down to each millisecond

uint32_t millis()
{
	return millisCounter;
}

CY_ISR(BitTimeISR)
{
	CYASKhandleTimerInterrupt();
	
	if (--downCounter == 0)
	{
		downCounter = milliDivisor;
		millisCounter++;
	}
}
/* [] END OF FILE */
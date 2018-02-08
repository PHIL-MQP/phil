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

// One quarter of a sine wave in 2048 10-bit values (0-2047):
// Note: sineTbl is stored in Flash memory
const int16_t sineTbl[] = {
128,128,128,129,129,129,130,130,131,131,131,132,132,133,133,133,134,134,135,135,135,136,136,136,137,
137,138,138,138,139,139,140,140,140,141,141,142,142,142,143,143,143,144,144,145,145,145,146,146,147,
147,147,148,148,149,149,149,150,150,150,151,151,152,152,152,153,153,154,154,154,155,155,155,156,156,
157,157,157,158,158,158,159,159,160,160,160,161,161,162,162,162,163,163,163,164,164,165,165,165,166,
166,166,167,167,167,168,168,169,169,169,170,170,170,171,171,172,172,172,173,173,173,174,174,174,175,
175,176,176,176,177,177,177,178,178,178,179,179,180,180,180,181,181,181,182,182,182,183,183,183,184,
184,184,185,185,186,186,186,187,187,187,188,188,188,189,189,189,190,190,190,191,191,191,192,192,192,
193,193,193,194,194,194,195,195,195,196,196,196,197,197,197,198,198,198,199,199,199,200,200,200,201,
201,201,202,202,202,203,203,203,203,204,204,204,205,205,205,206,206,206,207,207,207,207,208,208,208,
209,209,209,210,210,210,210,211,211,211,212,212,212,213,213,213,213,214,214,214,215,215,215,215,216,
216,216,217,217,217,217,218,218,218,218,219,219,219,220,220,220,220,221,221,221,221,222,222,222,222,
223,223,223,224,224,224,224,225,225,225,225,226,226,226,226,227,227,227,227,228,228,228,228,228,229,
229,229,229,230,230,230,230,231,231,231,231,232,232,232,232,232,233,233,233,233,234,234,234,234,234,
235,235,235,235,235,236,236,236,236,236,237,237,237,237,237,238,238,238,238,238,239,239,239,239,239,
240,240,240,240,240,240,241,241,241,241,241,242,242,242,242,242,242,243,243,243,243,243,243,244,244,
244,244,244,244,245,245,245,245,245,245,245,246,246,246,246,246,246,246,247,247,247,247,247,247,247,
248,248,248,248,248,248,248,248,249,249,249,249,249,249,249,249,250,250,250,250,250,250,250,250,250,
250,251,251,251,251,251,251,251,251,251,251,252,252,252,252,252,252,252,252,252,252,252,252,253,253,
253,253,253,253,253,253,253,253,253,253,253,253,254,254,254,254,254,254,254,254,254,254,254,254,254,
254,254,254,254,254,254,254,254,254,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,254,254,254,254,254,254,254,254,254,
254,254,254,254,254,254,254,254,254,254,254,254,254,253,253,253,253,253,253,253,253,253,253,253,253,
253,253,252,252,252,252,252,252,252,252,252,252,252,252,251,251,251,251,251,251,251,251,251,251,250,
250,250,250,250,250,250,250,250,250,249,249,249,249,249,249,249,249,248,248,248,248,248,248,248,248,
247,247,247,247,247,247,247,246,246,246,246,246,246,246,245,245,245,245,245,245,245,244,244,244,244,
244,244,243,243,243,243,243,243,242,242,242,242,242,242,241,241,241,241,241,240,240,240,240,240,240,
239,239,239,239,239,238,238,238,238,238,237,237,237,237,237,236,236,236,236,236,235,235,235,235,235,
234,234,234,234,234,233,233,233,233,232,232,232,232,232,231,231,231,231,230,230,230,230,229,229,229,
229,228,228,228,228,228,227,227,227,227,226,226,226,226,225,225,225,225,224,224,224,224,223,223,223,
222,222,222,222,221,221,221,221,220,220,220,220,219,219,219,218,218,218,218,217,217,217,217,216,216,
216,215,215,215,215,214,214,214,213,213,213,213,212,212,212,211,211,211,210,210,210,210,209,209,209,
208,208,208,207,207,207,207,206,206,206,205,205,205,204,204,204,203,203,203,203,202,202,202,201,201,
201,200,200,200,199,199,199,198,198,198,197,197,197,196,196,196,195,195,195,194,194,194,193,193,193,
192,192,192,191,191,191,190,190,190,189,189,189,188,188,188,187,187,187,186,186,186,185,185,184,184,
184,183,183,183,182,182,182,181,181,181,180,180,180,179,179,178,178,178,177,177,177,176,176,176,175,
175,174,174,174,173,173,173,172,172,172,171,171,170,170,170,169,169,169,168,168,167,167,167,166,166,
166,165,165,165,164,164,163,163,163,162,162,162,161,161,160,160,160,159,159,158,158,158,157,157,157,
156,156,155,155,155,154,154,154,153,153,152,152,152,151,151,150,150,150,149,149,149,148,148,147,147,
147,146,146,145,145,145,144,144,143,143,143,142,142,142,141,141,140,140,140,139,139,138,138,138,137,
137,136,136,136,135,135,135,134,134,133,133,133,132,132,131,131,131,130,130,129,129,129,128,128,128,
127,127,126,126,126,125,125,124,124,124,123,123,122,122,122,121,121,120,120,120,119,119,119,118,118,
117,117,117,116,116,115,115,115,114,114,113,113,113,112,112,112,111,111,110,110,110,109,109,108,108,
108,107,107,106,106,106,105,105,105,104,104,103,103,103,102,102,101,101,101,100,100,100,99,99,98,
98,98,97,97,97,96,96,95,95,95,94,94,93,93,93,92,92,92,91,91,90,90,90,89,89,
89,88,88,88,87,87,86,86,86,85,85,85,84,84,83,83,83,82,82,82,81,81,81,80,80,
79,79,79,78,78,78,77,77,77,76,76,75,75,75,74,74,74,73,73,73,72,72,72,71,71,
71,70,70,69,69,69,68,68,68,67,67,67,66,66,66,65,65,65,64,64,64,63,63,63,62,
62,62,61,61,61,60,60,60,59,59,59,58,58,58,57,57,57,56,56,56,55,55,55,54,54,
54,53,53,53,52,52,52,52,51,51,51,50,50,50,49,49,49,48,48,48,48,47,47,47,46,
46,46,45,45,45,45,44,44,44,43,43,43,42,42,42,42,41,41,41,40,40,40,40,39,39,
39,38,38,38,38,37,37,37,37,36,36,36,35,35,35,35,34,34,34,34,33,33,33,33,32,
32,32,31,31,31,31,30,30,30,30,29,29,29,29,28,28,28,28,27,27,27,27,27,26,26,
26,26,25,25,25,25,24,24,24,24,23,23,23,23,23,22,22,22,22,21,21,21,21,21,20,
20,20,20,20,19,19,19,19,19,18,18,18,18,18,17,17,17,17,17,16,16,16,16,16,15,
15,15,15,15,15,14,14,14,14,14,13,13,13,13,13,13,12,12,12,12,12,12,11,11,11,
11,11,11,10,10,10,10,10,10,10,9,9,9,9,9,9,9,8,8,8,8,8,8,8,7,
7,7,7,7,7,7,7,6,6,6,6,6,6,6,6,5,5,5,5,5,5,5,5,5,5,
4,4,4,4,4,4,4,4,4,4,3,3,3,3,3,3,3,3,3,3,3,3,2,2,2,
2,2,2,2,2,2,2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,
1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,2,2,2,
2,3,3,3,3,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,4,4,5,5,
5,5,5,5,5,5,5,5,6,6,6,6,6,6,6,6,7,7,7,7,7,7,7,7,8,
8,8,8,8,8,8,9,9,9,9,9,9,9,10,10,10,10,10,10,10,11,11,11,11,11,
11,12,12,12,12,12,12,13,13,13,13,13,13,14,14,14,14,14,15,15,15,15,15,15,16,
16,16,16,16,17,17,17,17,17,18,18,18,18,18,19,19,19,19,19,20,20,20,20,20,21,
21,21,21,21,22,22,22,22,23,23,23,23,23,24,24,24,24,25,25,25,25,26,26,26,26,
27,27,27,27,27,28,28,28,28,29,29,29,29,30,30,30,30,31,31,31,31,32,32,32,33,
33,33,33,34,34,34,34,35,35,35,35,36,36,36,37,37,37,37,38,38,38,38,39,39,39,
40,40,40,40,41,41,41,42,42,42,42,43,43,43,44,44,44,45,45,45,45,46,46,46,47,
47,47,48,48,48,48,49,49,49,50,50,50,51,51,51,52,52,52,52,53,53,53,54,54,54,
55,55,55,56,56,56,57,57,57,58,58,58,59,59,59,60,60,60,61,61,61,62,62,62,63,
63,63,64,64,64,65,65,65,66,66,66,67,67,67,68,68,68,69,69,69,70,70,71,71,71,
72,72,72,73,73,73,74,74,74,75,75,75,76,76,77,77,77,78,78,78,79,79,79,80,80,
81,81,81,82,82,82,83,83,83,84,84,85,85,85,86,86,86,87,87,88,88,88,89,89,89,
90,90,90,91,91,92,92,92,93,93,93,94,94,95,95,95,96,96,97,97,97,98,98,98,99,
99,100,100,100,101,101,101,102,102,103,103,103,104,104,105,105,105,106,106,106,107,107,108,108,108,
109,109,110,110,110,111,111,112,112,112,113,113,113,114,114,115,115,115,116,116,117,117,117,118,118,
119,119,119,120,120,120,121,121,122,122,122,123,123,124,124,124,125,125,126,126,126,127,127,128,
};

const uint32_t sample_rate = 62500;

// Parameters to create "chirp" output (rising or falling frequency sine wave):
// The 1st variable (sample_x65536) represents time along the sine wave (i.e. generates argument to the "sine" function)
// The 2nd variable (sample_incr_x65536) is how much to increment the first variable after each sample is generated.  Directly proportional to generated frequency
// The 3rd variable (sample_incr_incr_x65536) is how much to increment the second variable after each sample.  Causes frequency to slowly change (chirp)
// The 4th variable is the length of the total chirp in samples
// The 5th variable is the number of samples remaining before we are done generating this chirp (used by interrupt routine to know when to stop)
static float sin_index = 0;
static float sample_incr = 0;		// Number of table positions to increase "sample" each tick, mult. by 65536 (for 16 fraction bits)
static float  sample_incr_incr = 0;	// Amount to change (+/-) sample_incr_x65536 each tick (2nd derivitive of "sample" position), causes chirp
uint32_t sample_count_max = 0;			// Number of samples in complete ping (at sample_rate)
uint32_t sample_count = 0;				// Number of samples remaining in current chirp (at sample_rate)

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

	USBPutString("Starting up!\r");
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
				USBPutString("\rTransmit Mode:\r");
				CYASKsend((uint8_t *)"Xmt starting", 13);	// Send startup message on radio
				digit = '0';
				mode = Tx;
			}
			else if (c == 'R')						// "Receive"
			{
				USBPutString("\rReceive Mode:\r");
				mode = Rx;
			}
			else if (c == 'I')						// "Idle"
			{
				USBPutString("\rIdle Mode\r");
				mode = Idle;
			}
			else if (c == 'B')						// "Beep"
			{
				temp = USBGetNum(0);				// Get requested frequency in Hz
				PseudoRandom_WriteSeed(temp);		// Also use parameter to set PRS seed (temp code)
				while (temp < 100 || temp > 31000) {
					USBPutString("\rBeep:\rEnter frequency in Hz [100..31000]: ");
					temp = USBGetNum(0);
				};
				uprintf("\rBeep (%d Hz)\r", temp);
				chirp((uint16_t)temp, (uint16_t)temp, length);	// Generate beep (freq. in Hz, length in millis)
				mode = Idle;
			}
			else if (c == 'C')						// "Chirp"
			{
				uprintf("\rChirp (%d Hz to %d Hz)\r", f0, f1);
				chirp(f0, f1, length);				// Generate beep (freq. in Hz, length in millis)
				mode = Idle;
			}
			else if (c == 'F')						// "Frequency"
			{
				c = USBGetUpperCaseChar();
				if (c != 'A' && c != 'B') {
					uprintf("\r'F' command must specify 'A' or 'B' frequency to set.  Try again.\r\r");
					while (c != '\r') c = USBGetUpperCaseChar();	// Flush line input
				} else {
					temp = USBGetNum(0);
					if (temp == 0) {
						uprintf("\rFA = %d\rFB = %d\r\r", f0, f1);
					} else {
						if (c == 'A')
							f0 = temp;
						else // (c == 'B')
							f1 = temp;
						uprintf("\rFrequency %c set to %d Hz.\r", c, temp);
					}
				}
			}
			else if (c == 'L')						// "Length"
			{
				uint32_t oldLength = length;
				length = USBGetNum(0);
				if (length == 0) length = oldLength;
				uprintf("\rLength set to %d milliseconds.\r", length);
			}
			else if (c == 'X')						// "Tests"
			{
				// Pseudo Random Generator test
				temp = USBGetNum(0);				// Get requested seed
				PseudoRandom_WriteSeed(temp);		// Use parameter to set PRS seed
				uprintf("\r%6d", (prs()-32768) / (1<<7));
				uprintf("\r%6d", (prs()-32768) / (1<<7));
				uprintf("\r%6d", (prs()-32768) / (1<<7));
				uprintf("\r%6d", (prs()-32768) / (1<<7));
				uprintf("\r%6d\r", (prs()-32768) / (1<<7));
			}
			else if (c == 'Q')						// "Quit"
			{
				USBPutString("\rQuitting\r");
				return 0;
			}
			else if (c == 'V')						// "Voltage"
			{
				temp = USBGetNum(0);
				while (temp > 5000) {
					USBPutString("\rVoltage:\rEnter Voltage in mV [0..4000]: ");
					temp = USBGetNum(0);
				};
				uprintf("\rVoltage set to %d.%03dV\r", temp/1000, temp%1000);
				temp = temp/16;					// Convert to 0..250 range
				if (temp>255) temp = 255;		// Limit for VDACs is 255
				VDAC8_P_SetValue(temp);			// Set plus-side voltage (0-1.020V)
				VDAC8_M_SetValue(temp);			// Set minus-side voltage
			}
			else if(c==' ' || c=='\r')			// Ignore
			{
			}
			else USBPutString("\rUnknown command ignored.\r\r");
		}
		if(mode == Tx) {
			msg[12] = digit++;					// Create rotating digit 0 to 9
			if (digit > '9') digit = '0';
			CYASKsend((uint8_t *)msg, strlen(msg));
			CYASKwaitPacketSent();
			uprintf("%s\r", msg);
			CyDelay(500);
		}
		else if(mode == Rx) {
		    badCnt = CYASKrxBad();
		    if(badCnt != lastBadCnt)
		    {
				lastBadCnt = badCnt;
				uprintf("* * * * * * * BAD RCV COUNT * * * * * * *   --->  %d\r", badCnt);
				for (i=0; i<4; i++) {
					led = !led;
					ledPin_Write(led);			// Double blink the LED
					CyDelay(100);
				}
		    }
		    if (CYASKrecv(buf, &buflen)) 		// Non-blocking: Get message from radio if available
		    {
				// Message with a good checksum received, report it.
				uprintf("Message: %s\r", (char*)buf);
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
	sin_index = 0;
    sample_incr = 60;
    sample_incr_incr = 0.1;

    sample_count_max = 10 * 2048; // 10 for number of waves, 2048 samples per wave
    sample_count = sample_count_max;
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
	if (sample_count > 0) {				// Any samples to produce?
		sample_count--;					// Count this sample

        uint8_t sin_value = sineTbl[(uint16_t)sin_index];
		VDAC8_P_SetValue(sin_value);			// Set plus-side voltage (0-1.020V)
		VDAC8_M_SetValue(255 - sin_value);		// Set minus-side voltage (diff P-M is instantaneous value of sine)

        if (sin_index + sample_incr > 2047) {
            sin_index = sin_index + sample_incr - 2048;
        }
        else {
            sin_index += sample_incr;			// Use sample increment to compute next position in sine table
        }

		sample_incr += sample_incr_incr;	// Use sample increment increment (2nd deriv.) to create chirp

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

// CYASK.h
//
// Driver to modulate and demodulate conventional ASK (Amplitude Shift Keying) digital radio packets
// for Cypress PSoC processors (4 and 5LP).
//

#ifndef CYASK_h
#define CYASK_h

#include "main.h"

// Maximum message length (including the headers, byte count and FCS) we are willing to support
// This is pretty arbitrary
#define RH_ASK_MAX_PAYLOAD_LEN 67

// The length of the headers we add (To, From, Id, Flags)
// The headers are inside the payload and are therefore protected by the FCS
#define RH_ASK_HEADER_LEN 4

// This is the maximum message length that can be supported by this library. 
// Can be pre-defined to a smaller size (to save SRAM) prior to including this header
// Here we allow for 1 byte message length, 4 bytes headers, user data and 2 bytes of FCS
#ifndef RH_ASK_MAX_MESSAGE_LEN
 #define RH_ASK_MAX_MESSAGE_LEN (RH_ASK_MAX_PAYLOAD_LEN - RH_ASK_HEADER_LEN - 3)
#endif

#if !defined(RH_ASK_RX_SAMPLES_PER_BIT)
/// Number of samples per bit
 #define RH_ASK_RX_SAMPLES_PER_BIT 8
#endif //RH_ASK_RX_SAMPLES_PER_BIT  

/// The size of the receiver ramp. Ramp wraps modulo this number
#define RH_ASK_RX_RAMP_LEN 160

// Ramp adjustment parameters
// Standard is if a transition occurs before RH_ASK_RAMP_TRANSITION (80) in the ramp,
// the ramp is retarded by adding RH_ASK_RAMP_INC_RETARD (11)
// else by adding RH_ASK_RAMP_INC_ADVANCE (29)
// If there is no transition it is adjusted by RH_ASK_RAMP_INC (20)
/// Internal ramp adjustment parameter
#define RH_ASK_RAMP_INC (RH_ASK_RX_RAMP_LEN/RH_ASK_RX_SAMPLES_PER_BIT)
/// Internal ramp adjustment parameter
#define RH_ASK_RAMP_TRANSITION RH_ASK_RX_RAMP_LEN/2
/// Internal ramp adjustment parameter
#define RH_ASK_RAMP_ADJUST 9
/// Internal ramp adjustment parameter
#define RH_ASK_RAMP_INC_RETARD (RH_ASK_RAMP_INC-RH_ASK_RAMP_ADJUST)
/// Internal ramp adjustment parameter
#define RH_ASK_RAMP_INC_ADVANCE (RH_ASK_RAMP_INC+RH_ASK_RAMP_ADJUST)

/// Outgoing message bits grouped as 6-bit words
/// 36 alternating 1/0 bits, followed by 12 bits of start symbol (together called the preamble)
/// Followed immediately by the 4-6 bit encoded byte count, 
/// message buffer and 2 byte FCS
/// Each byte from the byte count on is translated into 2x6-bit words
/// Caution, each symbol is transmitted LSBit first, 
/// but each byte is transmitted high nybble first
/// This is the number of 6 bit nibbles in the preamble
#define RH_ASK_PREAMBLE_LEN 8
	
// Official version numbers are maintained automatically by Makefile:
#define RH_VERSION_MAJOR 1
#define RH_VERSION_MINOR 41

////////////////////////////////////////////////////
////////////////////////////////////////////////////
// Try to be compatible with systems that support yield() and multitasking
// instead of spin-loops
// Recent Arduino IDE or Teensy 3 has yield()
#if (RH_PLATFORM == RH_PLATFORM_ARDUINO && ARDUINO >= 155 && !defined(RH_PLATFORM_ATTINY)) || (TEENSYDUINO && defined(__MK20DX128__))
 #define YIELD yield();
#else
 #define YIELD
#endif


// These defs sometimes cause trouble
#undef abs
#undef round
#undef double

// This is the address that indicates a broadcast
#define RH_BROADCAST_ADDRESS 0xff


/////////////////////////////////////////////////////////////////////
/// \class RH_ASK RH_ASK.h <RH_ASK.h>
/// \brief Driver to send and receive unaddressed, unreliable datagrams via inexpensive ASK (Amplitude Shift Keying) or 
/// OOK (On Off Keying) RF transceivers.
///
/// The message format and software technology is based on our earlier VirtualWire library 
/// (http://www.airspayce.com/mikem/arduino/VirtualWire), with which it is compatible.
/// See http://www.airspayce.com/mikem/arduino/VirtualWire.pdf for more details. 
/// VirtualWire is now obsolete and unsupported and is replaced by this library.
///
/// RH_ASK is a Driver for Arduino, Maple and others that provides features to send short
/// messages, without addressing, retransmit or acknowledgment, a bit like UDP
/// over wireless, using ASK (amplitude shift keying). Supports a number of
/// inexpensive radio transmitters and receivers. All that is required is
/// transmit data, receive data and (for transmitters, optionally) a PTT
/// transmitter enable. Can also be used over various analog connections (not just a data radio), 
/// such as the audio channel of an A/V sender, or long TTL lines.
///
/// It is intended to be compatible with the RF Monolithics (www.rfm.com)
/// Virtual Wire protocol, but this has not been tested.
///
/// Does not use the Arduino UART. Messages are sent with a training preamble,
/// message length and checksum. Messages are sent with 4-to-6 bit encoding
/// for good DC balance, and a CRC checksum for message integrity.
///
/// But why not just use a UART connected directly to the
/// transmitter/receiver? As discussed in the RFM documentation, ASK receivers
/// require a burst of training pulses to synchronize the transmitter and
/// receiver, and also requires good balance between 0s and 1s in the message
/// stream in order to maintain the DC balance of the message. UARTs do not
/// provide these. They work a bit with ASK wireless, but not as well as this
/// code.
///
/// \par Theory of operation
///
/// See ASH Transceiver Software Designer's Guide of 2002.08.07
///   http://www.rfm.com/products/apnotes/tr_swg05.pdf
///
/// http://web.engr.oregonstate.edu/~moon/research/files/cas2_mar_07_dpll.pdf while not directly relevant 
/// is also interesting.
/// \par Implementation Details
///
/// Messages of up to RH_ASK_MAX_PAYLOAD_LEN (67) bytes can be sent
/// Each message is transmitted as:
///
/// - 36 bit training preamble consisting of 0-1 bit pairs
/// - 12 bit start symbol 0xb38
/// - 1 byte of message length byte count (4 to 30), count includes byte count and FCS bytes
/// - n message bytes (uincluding 4 bytes of header), maximum n is RH_ASK_MAX_MESSAGE_LEN + 4 (64)
/// - 2 bytes FCS, sent low byte-hi byte
///
/// Everything after the start symbol is encoded 4 to 6 bits, Therefore a byte in the message
/// is encoded as 2x6 bit symbols, sent hi nybble, low nybble. Each symbol is sent LSBit
/// first. The message may consist of any binary digits.
/// 
/// The Arduino Diecimila clock rate is 16MHz => 62.5ns/cycle.
/// For an RF bit rate of 2000 bps, need 500microsec bit period.
/// The ramp requires 8 samples per bit period, so need 62.5microsec per sample => interrupt tick is 62.5microsec.
///
/// The maximum packet length consists of
/// (6 + 2 + RH_ASK_MAX_MESSAGE_LEN*2) * 6 = 768 bits = 0.384 secs (at 2000 bps).
/// where RH_ASK_MAX_MESSAGE_LEN is RH_ASK_MAX_PAYLOAD_LEN - 7 (= 60).
/// The code consists of an ISR interrupt handler. Most of the work is done in the interrupt
/// handler for both transmit and receive, but some is done from the user level. Expensive
/// functions like CRC computations are always done in the user level.
/// Caution: VirtualWire takes over Arduino Timer1, and this will affect the PWM capabilities of the 
/// digital pins 9 and 10.
///
/// \par Supported Hardware
///
/// A range of communications
/// hardware is supported. The ones listed below are available in common retail
/// outlets in Australia and other countries for under $10 per unit. Many
/// other modules may also work with this software. 
///
/// Runs on a wide range of Arduino processors using Arduino IDE 1.0 or later.
/// Also runs on on Energia
/// with MSP430G2553 / G2452 and Arduino with ATMega328 (courtesy Yannick DEVOS - XV4Y), 
/// but untested by us. It also runs on Teensy 3.0 (courtesy of Paul
/// Stoffregen), but untested by us. Also compiles and runs on ATtiny85 in
/// Arduino environment, courtesy r4z0r7o3. Also compiles on maple-ide-v0.0.12,
/// and runs on Maple, flymaple 1.1 etc. Runs on ATmega8/168 (Arduino Diecimila,
/// Uno etc), ATmega328 and can run on almost any other AVR8 platform,
/// without relying on the Arduino framework, by properly configuring the
/// library editing the RH_ASK.h header file for describing the access
/// to IO pins and for setting up the timer.
///
/// - Receivers
///  - RX-B1 (433.92MHz) (also known as ST-RX04-ASK)
///  - RFM83C from HopeRF http://www.hoperfusa.com/details.jsp?pid=126
/// - Transmitters: 
///  - TX-C1 (433.92MHz)
///  - RFM85 from HopeRF http://www.hoperfusa.com/details.jsp?pid=127
/// - Transceivers
///  - DR3100 (433.92MHz)
///
/// \par Connecting to Arduino
///
/// Most transmitters can be connected to Arduino like this:

/// \code
/// Arduino                         Transmitter
///  GND------------------------------GND
///  D12------------------------------Data
///  5V-------------------------------VCC
/// \endcode
///
/// Most receivers can be connected to Arduino like this:
/// \code
/// Arduino                         Receiver
///  GND------------------------------GND
///  D11------------------------------Data
///  5V-------------------------------VCC
///                                   SHUT (not connected)
///                                   WAKEB (not connected)
///                                   GND |
///                                   ANT |- connect to your antenna syetem
/// \endcode
///
/// RH_ASK works with ATTiny85, using Arduino 1.0.5 and tinycore from
/// https://code.google.com/p/arduino-tiny/downloads/detail?name=arduino-tiny-0100-0018.zip
/// Tested with the examples ask_transmitter and ask_receiver on ATTiny85.
/// Caution: The RAM memory requirements on an ATTiny85 are *very* tight. Even the bare bones
/// ask_transmitter sketch barely fits in eh RAM available on the ATTiny85. Its unlikely to work on 
/// smaller ATTinys such as the ATTiny45 etc. If you have wierd behaviour, consider
/// reducing the size of RH_ASK_MAX_PAYLOAD_LEN to the minimum you can work with.
/// Caution: the default internal clock speed on an ATTiny85 is 1MHz. You MUST set the internal clock speed
/// to 8MHz. You can do this with Arduino IDE, tineycore and ArduinoISP by setting the board type to "ATtiny85@8MHz',
/// setting theProgrammer to 'Arduino as ISP' and selecting Tools->Burn Bootloader. This does not actually burn a
/// bootloader into the tiny, it just changes the fuses so the chip runs at 8MHz. 
/// If you run the chip at 1MHz, you will get RK_ASK speeds 1/8th of the expected.
///
/// Initialise RH_ASK for ATTiny85 like this:
/// // #include <SPI.h> // comment this out, not needed
/// RH_ASK driver(2000, 4, 3); // 200bps, TX on D3 (pin 2), RX on D4 (pin 3)
/// then:
/// Connect D3 (pin 2) as the output to the transmitter
/// Connect D4 (pin 3) as the input from the receiver.
/// 
///
/// For testing purposes you can connect 2 Arduino RH_ASK instances directly, by
/// connecting pin 12 of one to 11 of the other and vice versa, like this for a duplex connection:
///
/// \code
/// Arduino 1         wires         Arduino 1
///  D11-----------------------------D12
///  D12-----------------------------D11
///  GND-----------------------------GND
/// \endcode
///
/// You can also connect 2 RH_ASK instances over a suitable analog
/// transmitter/receiver, such as the audio channel of an A/V transmitter/receiver. You may need
/// buffers at each end of the connection to convert the 0-5V digital output to a suitable analog voltage.
///
/// Measured power output from RFM85 at 5V was 18dBm.
///
/// \par Timers
/// The CYASK driver uses a timer-driven interrupt to generate 8 interrupts per bit period.

// Configure the CYASK driver:
	/// \param[in] speed The desired bit rate in bits per second
    /// \param[in] rxPin The pin that is used to get data from the receiver
    /// \param[in] txPin The pin that is used to send data to the transmitter
    /// \param[in] pttPin The pin that is connected to the transmitter controller. It will be set HIGH to enable the transmitter (unless pttInverted is true).
void CYASK(uint16_t speed, uint8_t (*rxPin)(), void (*txPin)(bool), void (*pttPin)(bool));

// Initialise the Driver transport hardware and software.
    /// Make sure the Driver is properly configured before calling init().
    /// \return true if initialisation succeeded.
bool CYASKinit();

// Tests whether a new message is available
    /// from the Driver. 
    /// On most drivers, this will also put the Driver into RHModeRx mode until
    /// a message is actually received bythe transport, when it wil be returned to RHModeIdle.
    /// This can be called multiple times in a timeout loop
    /// \return true if a new, complete, error-free uncollected message is available to be retreived by recv()
bool    CYASKavailable();

// See if the receiver has gotten a good message, and return it if so:
    /// Turns the receiver on if it not already on.
    /// If there is a valid message available, copy it to buf and return true
    /// else return false.
    /// If a message is copied, *len is set to the length (Caution, 0 length messages are permitted).
    /// You should be sure to call this function frequently enough to not miss any messages
    /// It is recommended that you call it in your main loop.
    /// \param[in] buf Location to copy the received message
    /// \param[in,out] len Pointer to available space in buf. Set to the actual number of octets copied.
    /// \return true if a valid message was copied to buf
bool    CYASKrecv(uint8_t* buf, uint8_t* len);

// Transmit a message as soon as the transmitter is ready (may block):
    /// Waits until any previous transmit packet is finished being transmitted with waitPacketSent().
    /// Then loads a message into the transmitter and starts the transmitter. Note that a message length
    /// of 0 is NOT permitted. 
    /// \param[in] data Array of data to be sent
    /// \param[in] len Number of bytes of data to send (> 0)
    /// \return true if the message length was valid and it was correctly queued for transmit
bool    CYASKsend(const uint8_t* data, uint8_t len);

// Wait (block) until the transmitter is finished sending:
void	CYASKwaitPacketSent();

// Returns the maximum message length 
    /// available in this Driver.
    /// \return The maximum legal message length
uint8_t CYASKmaxMessageLength();

    /// If current mode is Rx or Tx changes it to Idle. If the transmitter or receiver is running, 
    /// disables them.
void    CYASKsetModeIdle();

    /// If current mode is Tx or Idle, changes it to Rx. 
    /// Starts the receiver in the RF69.
void    CYASKsetModeRx();

    /// If current mode is Rx or Idle, changes it to Rx. F
    /// Starts the transmitter in the RF69.
void    CYASKsetModeTx();

//protected:
    /// Read the rxPin in a platform dependent way, taking into account whether it is inverted or not
bool    CYASKreadRx();

    /// Write the txPin in a platform dependent way
void    CYASKwriteTx(bool value);

    /// Write the PttPin in a platform dependent way, taking into account whether it is inverted or not
void    CYASKwritePtt(bool value);

    /// Translates a 6 bit symbol to its 4 bit plaintext equivalent
uint8_t CYASKsymbol_6to4(uint8_t symbol);

    /// The receiver handler function, called a 8 times the bit rate
void    CYASKreceiveTimer();

    /// The transmitter handler function, called a 8 times the bit rate 
void    CYASKtransmitTimer();

    /// Check whether the latest received message is complete and uncorrupted
    /// We should always check the FCS at user level, not interrupt level
    /// since it is slow
void    CYASKvalidateRxBuf();

void	CYASKhandleTimerInterrupt();

    /// Configure bit rate in bits per second
uint16_t        _speed;

    /// The configured receiver pin
uint8_t         (*_rxPin)();

    /// The configured transmitter pin
void	         (*_txPin)(bool);

    /// The configured transmitter enable pin
void	         (*_pttPin)(bool);

    /// True if the sense of the rxPin is to be inverted
bool            _rxInverted;

    /// True if the sense of the pttPin is to be inverted
bool            _pttInverted;

/// These are the different values that can be adopted by the _mode variable and 
/// returned by the mode() member function,
    typedef enum
    {
	RHModeInitialising = 0, ///< Transport is initialising. Initial default value until init() is called..
	RHModeSleep,            ///< Transport hardware is in low power sleep mode (if supported)
	RHModeIdle,             ///< Transport is idle.
	RHModeTx,               ///< Transport is in the process of transmitting a message.
	RHModeRx                ///< Transport is in the process of receiving a message.
    } CYASKMode;

// Used in the interrupt handlers
    /// Buf is filled but not validated
volatile bool   _rxBufFull;

    /// Buf is full and valid
volatile bool   _rxBufValid;

    /// Last digital input from the rx data pin
volatile bool   _rxLastSample;

    /// This is the integrate and dump integral. If there are <5 0 samples in the PLL cycle
    /// the bit is declared a 0, else a 1
volatile uint8_t _rxIntegrator;

    /// PLL ramp, varies between 0 and RH_ASK_RX_RAMP_LEN-1 (159) over 
    /// RH_ASK_RX_SAMPLES_PER_BIT (8) samples per nominal bit time. 
    /// When the PLL is synchronised, bit transitions happen at about the
    /// 0 mark. 
volatile uint8_t _rxPllRamp;

    /// Flag indicates if we have seen the start symbol of a new message and are
    /// in the processes of reading and decoding it
volatile uint8_t _rxActive;

    /// Last 12 bits received, so we can look for the start symbol
volatile uint16_t _rxBits;

    /// How many bits of message we have received. Ranges from 0 to 12
volatile uint8_t _rxBitCount;
    
    /// The incoming message buffer
uint8_t _rxBuf[RH_ASK_MAX_PAYLOAD_LEN];
    
    /// The incoming message expected length
volatile uint8_t _rxCount;
    
    /// The incoming message buffer length received so far
volatile uint8_t _rxBufLen;

    /// Index of the next symbol to send. Ranges from 0 to vw_tx_len
uint8_t _txIndex;

    /// Bit number of next bit to send
uint8_t _txBit;

    /// Sample number for the transmitter. Runs 0 to 7 during one bit interval
uint8_t _txSample;

    /// The transmitter buffer in _symbols_ not data octets
uint8_t _txBuf[(RH_ASK_MAX_PAYLOAD_LEN * 2) + RH_ASK_PREAMBLE_LEN];

    /// Number of symbols in _txBuf to be sent;
uint8_t _txBufLen;

    /// The current transport operating mode
volatile CYASKMode     _mode;

    /// This node id
uint8_t             _thisAddress;
    
    /// Whether the transport is in promiscuous mode
bool                _promiscuous;

    /// TO header in the last received mesasge
volatile uint8_t    _rxHeaderTo;

    /// FROM header in the last received mesasge
volatile uint8_t    _rxHeaderFrom;

    /// ID header in the last received mesasge
volatile uint8_t    _rxHeaderId;

    /// FLAGS header in the last received mesasge
volatile uint8_t    _rxHeaderFlags;

    /// TO header to send in all messages
uint8_t             _txHeaderTo;

    /// FROM header to send in all messages
uint8_t             _txHeaderFrom;

    /// ID header to send in all messages
uint8_t             _txHeaderId;

    /// FLAGS header to send in all messages
uint8_t             _txHeaderFlags;

    /// The value of the last received RSSI value, in some transport specific units
volatile int8_t     _lastRssi;

    /// Count of the number of bad messages (eg bad checksum etc) received
volatile uint16_t   _rxBad;

    /// Count of the number of successfully transmitted messages
volatile uint16_t   _rxGood;

    /// Count of the number of good messages (correct checksum etc) received
volatile uint16_t   _txGood;

uint16_t			CYASKrxBad();

#endif

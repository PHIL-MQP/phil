// CYASK.cpp
//
// Copyright (C) 2017 Eric Peters
// $Id: CYASK.cpp,v 1.00 2017/07/20 06:04:26 eric Exp $

#include "main.h"
#include "CYASK.h"
#include "CRC.h"

#include "radioRxPin.h"
#include "radioTxPin.h"
#include "radioPttPin.h"


// Interrupt handler uses this to find the most recently initialised instance of this driver
//static RH_CYASK* thisCYASKDriver;

// 4 bit to 6 bit symbol converter table
// Used to convert the high and low nybbles of the transmitted data
// into 6 bit symbols for transmission. Each 6-bit symbol has 3 1s and 3 0s 
// with at most 3 consecutive identical bits
static uint8_t symbols[] =
{
    0xd,  0xe,  0x13, 0x15, 0x16, 0x19, 0x1a, 0x1c, 
    0x23, 0x25, 0x26, 0x29, 0x2a, 0x2c, 0x32, 0x34
};

// This is the value of the start symbol after 6-bit conversion and nybble swapping
#define RH_ASK_START_SYMBOL 0xb38

// Configure, but not initialize, the CYASK (Cypress Amplitude Shift Keying) module:
// @parm speed		Radio bit rate
// @parm rxPin		Pointer to routine to read the receive pin
// @parm txPin		Pointer to routine to write the transmit pin
// @parm pttPin		Pointer to routine to write the push-to-talk pin (turns on transmitter when sending)

void CYASK(uint16_t speed, uint8_t (*rxPin)(), void (*txPin)(bool), void (*pttPin)(bool))
{
    _mode = RHModeInitialising;
    _thisAddress = RH_BROADCAST_ADDRESS;
    _txHeaderTo = RH_BROADCAST_ADDRESS;
    _txHeaderFrom = RH_BROADCAST_ADDRESS;
    _txHeaderId = 0;
    _txHeaderFlags = 0;
    _rxBad = 0;
    _rxGood = 0;
    _txGood = 0;
    _speed = speed;
    _rxPin = rxPin;
    _txPin = txPin;
    _pttPin = pttPin;


    // Initialise the first 8 nibbles of the tx buffer to be the standard
    // preamble. We will append messages after that. 0x38, 0x2c is the start symbol before
    // 6-bit conversion to RH_ASK_START_SYMBOL
    uint8_t preamble[RH_ASK_PREAMBLE_LEN] = {0x2a, 0x2a, 0x2a, 0x2a, 0x2a, 0x2a, 0x38, 0x2c};
    memcpy(_txBuf, preamble, sizeof(preamble));
}

bool CYASKinit()
{
    // Ready to go
    CYASKsetModeIdle();

    return true;
}

void CYASKsetModeIdle()
{
    if (_mode != RHModeIdle)
    {
		// Disable the transmitter hardware
		CYASKwritePtt(LOW);
		CYASKwriteTx(LOW);
		_mode = RHModeIdle;
    }
}

void CYASKsetModeRx()
{
    if (_mode != RHModeRx)
    {
		// Disable the transmitter hardware
		CYASKwritePtt(LOW);
		CYASKwriteTx(LOW);
		_mode = RHModeRx;
    }
}

void CYASKsetModeTx()
{
    if (_mode != RHModeTx)
    {
		// PRepare state varibles for a new transmission
		_txIndex = 0;
		_txBit = 0;
		_txSample = 0;

		// Enable the transmitter hardware
		CYASKwritePtt(HIGH);

		_mode = RHModeTx;
    }
}

// Call this often
bool CYASKavailable()
{
    if (_mode == RHModeTx)
		return false;
    CYASKsetModeRx();
    if (_rxBufFull)
    {
		CYASKvalidateRxBuf();
		_rxBufFull= false;
    }
    return _rxBufValid;
}

bool CYASKrecv(uint8_t* buf, uint8_t* len)
{
    if (!CYASKavailable())
		return false;

    if (buf && len)
    {
		// Skip the length and 4 headers that are at the beginning of the rxBuf
		// and drop the trailing 2 bytes of FCS
		uint8_t message_len = _rxBufLen-RH_ASK_HEADER_LEN - 3;
		if (*len > message_len)
		    *len = message_len;
		memcpy(buf, _rxBuf+RH_ASK_HEADER_LEN+1, *len);
    }
    _rxBufValid = false; // Got the most recent message, delete it
//    printBuffer("recv:", buf, *len);
    return true;
}

// Caution: this may block
bool CYASKsend(const uint8_t* data, uint8_t len)
{
    uint8_t i;
    uint16_t index = 0;
    uint16_t crc = 0xffff;
    uint8_t *p = _txBuf + RH_ASK_PREAMBLE_LEN; // start of the message area
    uint8_t count = len + 3 + RH_ASK_HEADER_LEN; // Added byte count and FCS and headers to get total number of bytes

    if (len > RH_ASK_MAX_MESSAGE_LEN)
	return false;

    // Wait for transmitter to become available
    CYASKwaitPacketSent();

    // Encode the message length
    crc = RHcrc_ccitt_update(crc, count);
    p[index++] = symbols[count >> 4];
    p[index++] = symbols[count & 0xf];

    // Encode the headers
    crc = RHcrc_ccitt_update(crc, _txHeaderTo);
    p[index++] = symbols[_txHeaderTo >> 4];
    p[index++] = symbols[_txHeaderTo & 0xf];
    crc = RHcrc_ccitt_update(crc, _txHeaderFrom);
    p[index++] = symbols[_txHeaderFrom >> 4];
    p[index++] = symbols[_txHeaderFrom & 0xf];
    crc = RHcrc_ccitt_update(crc, _txHeaderId);
    p[index++] = symbols[_txHeaderId >> 4];
    p[index++] = symbols[_txHeaderId & 0xf];
    crc = RHcrc_ccitt_update(crc, _txHeaderFlags);
    p[index++] = symbols[_txHeaderFlags >> 4];
    p[index++] = symbols[_txHeaderFlags & 0xf];

    // Encode the message into 6 bit symbols. Each byte is converted into 
    // 2 6-bit symbols, high nybble first, low nybble second
    for (i = 0; i < len; i++)
    {
	crc = RHcrc_ccitt_update(crc, data[i]);
	p[index++] = symbols[data[i] >> 4];
	p[index++] = symbols[data[i] & 0xf];
    }

    // Append the fcs, 16 bits before encoding (4 6-bit symbols after encoding)
    // Caution: VW expects the _ones_complement_ of the CCITT CRC-16 as the FCS
    // VW sends FCS as low byte then hi byte
    crc = ~crc;
    p[index++] = symbols[(crc >> 4)  & 0xf];
    p[index++] = symbols[crc & 0xf];
    p[index++] = symbols[(crc >> 12) & 0xf];
    p[index++] = symbols[(crc >> 8)  & 0xf];

    // Total number of 6-bit symbols to send
    _txBufLen = index + RH_ASK_PREAMBLE_LEN;

    // Start the low level interrupt handler sending symbols
    CYASKsetModeTx();

    return true;
}

// Read the RX data input pin, taking into account platform type and inversion.
bool CYASKreadRx()
{
	return _rxPin();
}

// Write the TX output pin, taking into account platform type.
void CYASKwriteTx(bool value)
{
	_txPin(value!=0);
}

// Write the PTT output pin, taking into account platform type and inversion.
void CYASKwritePtt(bool value)
{
	_pttPin(value!=0);
}

// Convert a 6 bit encoded symbol into its 4 bit decoded equivalent
uint8_t CYASKsymbol_6to4(uint8_t symbol)
{
    uint8_t i;
    uint8_t count;
    
    // Linear search :-( Could have a 64 byte reverse lookup table?
    // There is a little speedup here courtesy Ralph Doncaster:
    // The shortcut works because bit 5 of the symbol is 1 for the last 8
    // symbols, and it is 0 for the first 8.
    // So we only have to search half the table
    for (i = (symbol>>2) & 8, count=8; count-- ; i++)
	if (symbol == symbols[i]) return i;

    return 0; // Not found
}

// Check whether the latest received message is complete and uncorrupted
// We should always check the FCS at user level, not interrupt level
// since it is slow
void CYASKvalidateRxBuf()
{
    uint16_t crc = 0xffff;
    // The CRC covers the byte count, headers and user data
    for (uint8_t i = 0; i < _rxBufLen; i++)
	crc = RHcrc_ccitt_update(crc, _rxBuf[i]);
    if (crc != 0xf0b8) // CRC when buffer and expected CRC are CRC'd
    {
	// Reject and drop the message
	_rxBad++;
	_rxBufValid = false;
	return;
    }

    // Extract the 4 headers that follow the message length
    _rxHeaderTo    = _rxBuf[1];
    _rxHeaderFrom  = _rxBuf[2];
    _rxHeaderId    = _rxBuf[3];
    _rxHeaderFlags = _rxBuf[4];
    if (_promiscuous ||
	_rxHeaderTo == _thisAddress ||
	_rxHeaderTo == RH_BROADCAST_ADDRESS)
    {
	_rxGood++;
	_rxBufValid = true;
    }
}

void CYASKreceiveTimer()
{
    bool rxSample = CYASKreadRx();

    // Integrate each sample
    if (rxSample)
	_rxIntegrator++;

    if (rxSample != _rxLastSample)
    {
	// Transition, advance if ramp > 80, retard if < 80
	_rxPllRamp += ((_rxPllRamp < RH_ASK_RAMP_TRANSITION) 
			   ? RH_ASK_RAMP_INC_RETARD 
			   : RH_ASK_RAMP_INC_ADVANCE);
	_rxLastSample = rxSample;
    }
    else
    {
	// No transition
	// Advance ramp by standard 20 (== 160/8 samples)
	_rxPllRamp += RH_ASK_RAMP_INC;
    }
    if (_rxPllRamp >= RH_ASK_RX_RAMP_LEN)
    {
	// Add this to the 12th bit of _rxBits, LSB first
	// The last 12 bits are kept
	_rxBits >>= 1;

	// Check the integrator to see how many samples in this cycle were high.
	// If < 5 out of 8, then its declared a 0 bit, else a 1;
	if (_rxIntegrator >= 5)
	    _rxBits |= 0x800;

	_rxPllRamp -= RH_ASK_RX_RAMP_LEN;
	_rxIntegrator = 0; // Clear the integral for the next cycle

	if (_rxActive)
	{
	    // We have the start symbol and now we are collecting message bits,
	    // 6 per symbol, each which has to be decoded to 4 bits
	    if (++_rxBitCount >= 12)
	    {
		// Have 12 bits of encoded message == 1 byte encoded
		// Decode as 2 lots of 6 bits into 2 lots of 4 bits
		// The 6 lsbits are the high nybble
		uint8_t this_byte = 
		    (CYASKsymbol_6to4(_rxBits & 0x3f)) << 4 
		    | CYASKsymbol_6to4(_rxBits >> 6);

		// The first decoded byte is the byte count of the following message
		// the count includes the byte count and the 2 trailing FCS bytes
		// REVISIT: may also include the ACK flag at 0x40
		if (_rxBufLen == 0)
		{
		    // The first byte is the byte count
		    // Check it for sensibility. It cant be less than 7, since it
		    // includes the byte count itself, the 4 byte header and the 2 byte FCS
		    _rxCount = this_byte;
		    if (_rxCount < 7 || _rxCount > RH_ASK_MAX_PAYLOAD_LEN)
		    {
			// Stupid message length, drop the whole thing
			_rxActive = false;
			_rxBad++;
                        return;
		    }
		}
		_rxBuf[_rxBufLen++] = this_byte;

		if (_rxBufLen >= _rxCount)
		{
		    // Got all the bytes now
		    _rxActive = false;
		    _rxBufFull = true;
		    CYASKsetModeIdle();
		}
		_rxBitCount = 0;
	    }
	}
	// Not in a message, see if we have a start symbol
	else if (_rxBits == RH_ASK_START_SYMBOL)
	{
	    // Have start symbol, start collecting message
	    _rxActive = true;
	    _rxBitCount = 0;
	    _rxBufLen = 0;
	}
    }
}

void CYASKtransmitTimer()
{
    if (_txSample++ == 0)
    {
	// Send next bit
	// Symbols are sent LSB first
	// Finished sending the whole message? (after waiting one bit period 
	// since the last bit)
	if (_txIndex >= _txBufLen)
	{
	    CYASKsetModeIdle();
	    _txGood++;
	}
	else
	{
	    CYASKwriteTx(_txBuf[_txIndex] & (1 << _txBit++));
	    if (_txBit >= 6)
	    {
		_txBit = 0;
		_txIndex++;
	    }
	}
    }
	
    if (_txSample > 7)
	_txSample = 0;
}

void CYASKhandleTimerInterrupt()
{
    if (_mode == RHModeRx)
		CYASKreceiveTimer();	// Receiving
    else if (_mode == RHModeTx)
        CYASKtransmitTimer();	// Transmitting
}


// Blocks until a valid message is received
void CYASKwaitAvailable()
{
    while (!CYASKavailable())
	YIELD;
}

// Blocks until a valid message is received or timeout expires
// Return true if there is a message available
// Works correctly even on millis() rollover
bool CYASKwaitAvailableTimeout(uint16_t timeout)
{
    uint32_t starttime = millis();
    while ((millis() - starttime) < timeout)
    {
        if (CYASKavailable())
           return true;
		YIELD;
    }
    return false;
}

void CYASKwaitPacketSent()
{
    while (_mode == RHModeTx)
		YIELD; // Wait for any previous transmit to finish
    return;
}

bool CYASKwaitPacketSentTimeout(uint16_t timeout)
{
    uint32_t starttime = millis();
    while ((millis() - starttime) < timeout)
    {
        if (_mode != RHModeTx) // Any previous transmit finished?
           return true;
		YIELD;
    }
    return false;
}

void CYASKsetPromiscuous(bool promiscuous)
{
    _promiscuous = promiscuous;
}

void CYASKsetThisAddress(uint8_t address)
{
    _thisAddress = address;
}

void CYASKsetHeaderTo(uint8_t to)
{
    _txHeaderTo = to;
}

void CYASKsetHeaderFrom(uint8_t from)
{
    _txHeaderFrom = from;
}

void CYASKsetHeaderId(uint8_t id)
{
    _txHeaderId = id;
}

void CYASKsetHeaderFlags(uint8_t set, uint8_t clear)
{
    _txHeaderFlags &= ~clear;
    _txHeaderFlags |= set;
}

uint8_t CYASKheaderTo()
{
    return _rxHeaderTo;
}

uint8_t CYASKheaderFrom()
{
    return _rxHeaderFrom;
}

uint8_t CYASKheaderId()
{
    return _rxHeaderId;
}

uint8_t CYASKheaderFlags()
{
    return _rxHeaderFlags;
}

int8_t CYASKlastRssi()
{
    return _lastRssi;
}

CYASKMode  CYASKmode()
{
    return _mode;
}

void  CYASKsetMode(CYASKMode mode)
{
    _mode = mode;
}

bool  CYASKsleep()
{
    return false;
}

// Diagnostic help
void CYASKprintBuffer(const char* prompt, const uint8_t* buf, uint8_t len)
{
#ifdef RH_HAVE_SERIAL
    uint8_t i;

    Serial.println(prompt);
    for (i = 0; i < len; i++)
    {
	if (i % 16 == 15)
	    Serial.println(buf[i], HEX);
	else
	{
	    Serial.print(buf[i], HEX);
	    Serial.print(' ');
	}
    }
    Serial.println("");
#endif
}

uint16_t CYASKrxBad()
{
    return _rxBad;
}

uint16_t CYASKrxGood()
{
    return _rxGood;
}

uint16_t CYASKtxGood()
{
    return _txGood;
}


// CRC.h
//
// Definitions for CRC routines.
//
// These routines originally derived from Arduino source code. See RHCRC.cpp
// for copyright information
// $Id: CRC.h,v 1.0 2017/07/24 09:14:13 ericp Exp $

#ifndef RHCRC_h
#define RHCRC_h

#include "CYASK.h"

extern uint16_t RHcrc_ccitt_update (uint16_t crc, uint8_t data);

#endif

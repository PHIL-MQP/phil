/* Copyright (c) 2002, 2003, 2004  Marek Michalkiewicz
   Copyright (c) 2005, 2007 Joerg Wunsch
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. */

//	Port to Energia / MPS430 by Yannick DEVOS XV4Y - (c) 2013
//	http://xv4y.radioclub.asia/
//	

// Adapted to RadioHead use by Mike McCauley 2014
// This is to prevent name collisions with other similar library functions
// and to provide a consistent API among all processors
//

// Adapted to Cypress PSoC by Eric Peters 2017
// Cypress does not support C++, so needed to convert to plain C.
// It is for use with "First_GPS", a radio/sonic location finding service for
// First Robotics competitions.

/* $Id: CRC.c,v 1.0 2017/07/24 09:14:13 ericp Exp $ */

#include "CRC.h"

uint16_t RHcrc_ccitt_update (uint16_t crc, uint8_t data)
{
    data ^= LO8 (crc);
    data ^= data << 4;
    
    return ((((uint16_t)data << 8) | HI8 (crc)) ^ (uint8_t)(data >> 4) 
	    ^ ((uint16_t)data << 3));
}



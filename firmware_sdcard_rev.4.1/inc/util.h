/*
 * Copyright (c) 2013, M.Naruoka (fenrir)
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 * 
 * - Redistributions of source code must retain the above copyright notice, 
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, 
 *   this list of conditions and the following disclaimer in the documentation 
 *   and/or other materials provided with the distribution.
 * - Neither the name of the naruoka.org nor the names of its contributors 
 *   may be used to endorse or promote products derived from this software 
 *   without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS 
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#ifndef __UTIL_H__
#define __UTIL_H__

//#include "main.h"
//#include "type.h"
#include "stdint.h"

void wait_8n6clk(unsigned char i);
void _wait_us(unsigned int count);
void wait_ms(unsigned int count);

#define wait_us(n) { \
  ((n) <= 100) ? wait_8n6clk((n) * 5) : _wait_us((n) - 1); \
}

#ifdef ENDIAN_SWAP_BY_FUNC
uint32_t swap_u32(uint32_t dw);
uint16_t swap_u16(uint16_t w);
#else
#define swap_u32(dw) \
( (uint32_t)(((uint32_t)(dw) & 0x000000FF) << 24) \
  | (((uint32_t)(dw) & 0x0000FF00) << 8) \
  | (((uint32_t)(dw) & 0x00FF0000) >> 8) \
  | (((uint32_t)(dw) & 0xFF000000) >> 24) \
)
#define swap_u16(w) \
( (uint16_t)(((uint16_t)(w) & 0x00FF) << 8) \
  | (((uint16_t)(w) & 0xFF00) >> 8) \
)
#endif


#if (defined(__SDCC) || defined(SDCC))
#define le_u32(dw) (dw)
#define le_u16(w) (w)
#define be_u32(dw) swap_u32(dw)
#define be_u16(w) swap_u16(w)
#define _nop_() { \
  __asm \
    nop \
  __endasm; \
}
#else
#define le_u32(dw)
#define le_u16(w)
#define be_u32(dw)
#define be_u16(w)
#endif

#define min(a,b) (((a)<(b))?(a):(b))
#define max(a,b) (((a)>(b))?(a):(b))

uint16_t crc16(uint8_t *buf, uint8_t size, uint16_t crc);

#endif /* __UTIL_H__ */


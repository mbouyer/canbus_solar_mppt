/*
 * Copyright (c) 2022 Manuel Bouyer
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#define I2C_INIT { \
	RC4I2C = 0x41; /* I2C fast slew-rate, no pull-up, i2c threshold */ \
	RC3I2C = 0x41; /* I2C fast slew-rate, no pull-up, i2c threshold */ \
	RC3PPS = 0x37; /* SCL */ \
	RC4PPS = 0x38; /* SDA */ \
	I2C1SCLPPS = 0x13; /* RC3 */ \
	I2C1SDAPPS = 0x14; /* RC4 */ \
	ODCONCbits.ODCC3 = 1; \
	ODCONCbits.ODCC4 = 1; \
	LATCbits.LATC3 = 1; \
	LATCbits.LATC4 = 1; \
	TRISCbits.TRISC3 = 0; \
	TRISCbits.TRISC4 = 0; \
	I2C1CON0 = 0x04; /* On, 7 bits I2C host mode */ \
	I2C1CON1 = 0; \
	I2C1CON2 = 0; /* address buffer enabled */ \
	I2C1CLK = 3; /* clock = mfintosc  (500khz) */ \
	I2C1BAUD = 0; /* prescale = 1 */ \
	I2C1CON0bits.EN = 1; \
    }

char i2c_readreg(const uint8_t address, uint8_t reg, uint8_t *data, uint8_t size);
char i2c_readreg_be(const uint8_t address, uint8_t reg, uint8_t *data, uint8_t size);
char i2c_writereg(const uint8_t address, uint8_t reg, uint8_t *data, uint8_t size);
char i2c_writereg_be(const uint8_t address, uint8_t reg, uint8_t *data, uint8_t size);
char i2c_writecmd(const uint8_t address, uint8_t reg);

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

typedef __volatile enum {
	I2C_INPROGRESS = 0,
	I2C_COMPLETE,
	I2C_ERROR
} i2c_return_t;

void i2c_init(void);

void i2c_readreg(const uint8_t address, uint8_t reg, uint8_t *data, uint8_t size, i2c_return_t *r);
void i2c_readreg_be(const uint8_t address, uint8_t reg, uint8_t *data, uint8_t size, i2c_return_t *r);
void i2c_writereg(const uint8_t address, uint8_t reg, uint8_t *data, uint8_t size, i2c_return_t *r);
void i2c_writereg_be(const uint8_t address, uint8_t reg, uint8_t *data, uint8_t size, i2c_return_t *r);
void i2c_writecmd(const uint8_t address, uint8_t reg, i2c_return_t *r);
void i2c_writereg_dma(const uint8_t address, uint8_t reg, uint8_t *data, uint16_t size, i2c_return_t *r);

void i2c_abort(void);

static char
i2c_wait(volatile i2c_return_t *r)
{
	int i2c_wait_count = 0;
	while (*r == I2C_INPROGRESS) {
		CLRWDT();
		i2c_wait_count++;
		if (i2c_wait_count == 30000) {
			i2c_abort();
			return 1;
		}
	};
	if (*r == I2C_COMPLETE)
		return 0;
	return 1;
}

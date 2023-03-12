/* $Id: serial.c,v 1.2 2017/06/05 11:00:19 bouyer Exp $ */
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

#include <xc.h>
#include <stdio.h>
#include <serial.h>

char uart_txbuf[UART_BUFSIZE];
unsigned char uart_txbuf_prod;
volatile unsigned char uart_txbuf_cons;

void
usart_putchar (char c)
{
#if 1
	unsigned char new_uart_txbuf_prod = (uart_txbuf_prod + 1) & UART_BUFSIZE_MASK;

again:
        while (new_uart_txbuf_prod == uart_txbuf_cons) {
		PIE4bits.U1TXIE = 1; /* ensure we'll make progress */
	}
	uart_txbuf[uart_txbuf_prod] = c;
	uart_txbuf_prod = new_uart_txbuf_prod;
	PIE4bits.U1TXIE = 1;
	if (c == '\n') {
		c = '\r';
		new_uart_txbuf_prod = (uart_txbuf_prod + 1) & UART_BUFSIZE_MASK;
		goto again;
	}
#else
	char d = c;
again:
	if  (!PIR4bits.U1TXIF) {
		goto again;
	}
	U1TXB = d;
	if (d == '\n') {
		d = '\r';
		goto again;
	}
#endif
}

void __interrupt(__irq(U1TX), __low_priority, base(IVECT_BASE))
irql_uart1(void)
{
	if (PIE4bits.U1TXIE && PIR4bits.U1TXIF) {
		if (uart_txbuf_prod == uart_txbuf_cons) {
			PIE4bits.U1TXIE = 0; /* buffer empty */
		} else {
			/* Place char in TXREG - this starts transmition */
			U1TXB = uart_txbuf[uart_txbuf_cons];
			uart_txbuf_cons = (uart_txbuf_cons + 1) & UART_BUFSIZE_MASK;
		}
	}
}

int
getchar(void)
{
	char c;
	while (!PIR4bits.U1RXIF); /* wait for a char */
	c = U1RXB;
	if (U1ERRIRbits.RXFOIF) {
		U1ERRIRbits.RXFOIF = 0;
	}
	return c;
}

/*
 * Copyright (c) 2021 Manuel Bouyer
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

/*
 * thermistor constants array for NT06104F3950B1H (beta=3950)
 * It has to be in its own file because of sdcc bug 
 */

#include <xc.h> 
#include "ntc_tab.h"
const struct temp_val const temps[] = {
	{
		.temp = 35315,
		.val = 3634, /* R = 1270 Ohm */
	},
	{
		.temp = 34815,
		.val = 3564, /* R = 1492 Ohm */
	},
	{
		.temp = 34315,
		.val = 3482, /* R = 1760 Ohm */
	},
	{
		.temp = 33815,
		.val = 3389, /* R = 2086 Ohm */
	},
	{
		.temp = 33315,
		.val = 3280, /* R = 2486 Ohm */
	},
	{
		.temp = 32815,
		.val = 3156, /* R = 2978 Ohm */
	},
	{
		.temp = 32315,
		.val = 3014, /* R = 3588 Ohm */
	},
	{
		.temp = 31815,
		.val = 2855, /* R = 4348 Ohm */
	},
	{
		.temp = 31315,
		.val = 2676, /* R = 5301 Ohm */
	},
	{
		.temp = 30815,
		.val = 2482, /* R = 6506 Ohm */
	},
	{
		.temp = 30315,
		.val = 2271, /* R = 8037 Ohm */
	},
	{
		.temp = 29815,
		.val = 2048, /* R = 10000 Ohm */
	},
	{
		.temp = 29315,
		.val = 1818, /* R = 12535 Ohm */
	},
	{
		.temp = 28815,
		.val = 1585, /* R = 15837 Ohm */
	},
	{
		.temp = 28315,
		.val = 1357, /* R = 20175 Ohm */
	},
	{
		.temp = 27815,
		.val = 1140, /* R = 25925 Ohm */
	},
	{
		.temp = 27315,
		.val = 939, /* R = 33621 Ohm */
	},
	{
		.temp = 26815,
		.val = 758, /* R = 44026 Ohm */
	},
	{
		.temp = 26315,
		.val = 600, /* R = 58245 Ohm */
	},
	{
		.temp = 25815,
		.val = 466, /* R = 77898 Ohm */
	},
	{
		.temp = 25315,
		.val = 355, /* R = 105385 Ohm */
	},
	{
		.temp = 0,
		.val = 0, /* OEL */
	}
};

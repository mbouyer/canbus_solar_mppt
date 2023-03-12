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
 * thermistor constants array for ND06P00103K
 * It has to be in its own file because of sdcc bug 
 */

#include <xc.h> 
#include "ntc_tab.h"
const struct temp_val const temps[] = {
	{
		.temp = 33315,
		.val = 3340, /* R = 2261 Ohm */
	},
	{
		.temp = 32815,
		.val = 3214, /* R = 2742 Ohm */
	},
	{
		.temp = 32315,
		.val = 3069, /* R = 3345 Ohm */
	},
	{
		.temp = 31815,
		.val = 2903, /* R = 4108 Ohm */
	},
	{
		.temp = 31315,
		.val = 2716, /* R = 5076 Ohm */
	},
	{
		.temp = 30815,
		.val = 2510, /* R = 6317 Ohm */
	},
	{
		.temp = 30315,
		.val = 2285, /* R = 7918 Ohm */
	},
	{
		.temp = 29815,
		.val = 2048, /* R = 10000 Ohm */
	},
	{
		.temp = 29315,
		.val = 1802, /* R = 12730 Ohm */
	},
	{
		.temp = 28815,
		.val = 1554, /* R = 16343 Ohm */
	},
	{
		.temp = 28315,
		.val = 1314, /* R = 21166 Ohm */
	},
	{
		.temp = 27815,
		.val = 1087, /* R = 27669 Ohm */
	},
	{
		.temp = 27315,
		.val = 880, /* R = 36526 Ohm */
	},
	{
		.temp = 26815,
		.val = 697, /* R = 48720 Ohm */
	},
	{
		.temp = 26315,
		.val = 541, /* R = 65701 Ohm */
	},
	{
		.temp = 25815,
		.val = 411, /* R = 89633 Ohm */
	},
	{
		.temp = 25315,
		.val = 306, /* R = 123791 Ohm */
	},
	{
		.temp = 0,
		.val = 0, /* OEL */
	}
};

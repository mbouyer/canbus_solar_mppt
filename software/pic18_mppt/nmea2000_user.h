/* $Id: nmea2000_user.h,v 1.2 2017/07/21 20:23:47 bouyer Exp $ */
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

#define NMEA2000_USER_ADDRESS 128

extern unsigned long nmea2000_user_id;

#define NEMA2000_USE_FAST_FRAME

#define NMEA2000_USER_ID nmea2000_user_id /* unique number */
#define NMEA2000_USER_MANUF 0x7feUL /* manufacturer code */
#define NMEA2000_USER_DEVICE_INSTANCE 0
#define NMEA2000_USER_DEVICE_FUNCTION 142
#define NMEA2000_USER_DEVICE_CLASS 35
#define NMEA2000_USER_INDUSTRY_GROUP 4
#define NMEA2000_USER_SYSTEM_INSTANCE 0

/* setup for 250Kbs, 1Tq = 1/10Mhz */
#define NBTCFGU_uval 26 /* Tseg1 = 27 Tq */
#define NBTCFGH_uval 11 /* Tseg2 = 12 Tq */
#define NBTCFGL_uval 10 /* Swj = 11Tq */

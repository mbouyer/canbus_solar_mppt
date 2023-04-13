/* $Id: main.c,v 1.36 2019/03/12 19:24:19 bouyer Exp $ */
/*
 * Copyright (c) 2023 Manuel Bouyer
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
#include <string.h> 
#include <stdlib.h> 
#include <nmea2000.h>
#include <nmea2000_pgn.h>
#include <raddeg.h>
#include "serial.h"
#include "i2c.h"
#include "ntc_tab.h"
#include "pac195x.h"
#include "font5x8.h"
#include "font10x16.h"
#include "icons16x16.h"

typedef unsigned char u_char;
typedef unsigned int  u_int;
typedef unsigned long u_long;

static u_char default_src;

u_int devid, revid; 

u_long nmea2000_user_id; 

static u_char sid;

static struct nmea2000_msg msg;
static u_char nmea2000_data[NMEA2000_DATA_FASTLENGTH];

u_int timer0_read(void);

#define TIMER0_5MS 48
#define TIMER0_1MS 10

#define NCANOK  PORTCbits.RC2

#define PWM_OFF   LATCbits.LATC1
#define PWM_OK    PORTCbits.RC0
#define PWM_MID   LATCbits.LATC5 /* high-side resistor for 3-state PWM output */
#define PWM_OUT   LATCbits.LATC6 /* PWM output */
#define PAC_NDOWN LATCbits.LATC7

#define OLED_RSTN LATAbits.LATA4

static void
batt1_en(char on)
{
	CLCSELECT = 2;
	CLCnPOLbits.G2POL = on;
}

static void
batt2_en(char on)
{
	CLCSELECT = 3;
	CLCnPOLbits.G2POL = on;
}

static char counter_10hz;
static char counter_1hz;
static uint16_t seconds;
static volatile union softintrs {
	struct softintrs_bits {
		char int_10hz : 1;	/* 0.1s timer */
		char int_btn_down : 1;	/* button release->press */
		char int_btn_up : 1;	/* button press->release */
		char int_adcc : 1;	/* A/D convertion complete */
		char int_pacavg : 1;	/* pac average values updated */
		char int_pacacc : 1;	/* pac accum values updated */
	} bits;
	char byte;
} softintrs;

static uint16_t a2d_acc;

static uint16_t board_temp;

static enum {
	BTN_IDLE = 0,
	BTN_DOWN,
	BTN_DOWN_1,
	BTN_DOWN_1_P,
	BTN_DOWN_2,
	BTN_DOWN_2_P,
	BTN_UP
} btn_state;


#define PAC_I2C_ADDR 0x2e

static volatile i2c_return_t i2c_return;

uint16_t batt_v[4];
int16_t batt_i[4];

static uint32_t voltages_acc[4];
pac_ctrl_t pac_ctrl;

/* for journal */
static int64_t l600_current_acc[4];
static __uint24 l600_current_count;
static uint32_t l600_voltages_acc[4];

/* PAC 195x state machine */

static struct pac_i2c_io {
	enum {
		PAC_FREE = 0,
		PAC_WRITE, /* general register write */
		PAC_READ, /* general register read */
		PAC_READ_ACCUM, /* read accumulator values */
		PAC_READ_AVG, /* read recent average values */
	} pac_type;
	u_char pac_reg; /* register address */
	void *pac_data; /* pointer to data for read/write */
	u_char pac_datalen;
} pac_i2c_io;

#define PAC_WRITEREG(reg, data) {\
	pac_i2c_io.pac_type = PAC_WRITE;\
	pac_i2c_io.pac_reg = (reg);\
	pac_i2c_io.pac_data = &(data);\
	pac_i2c_io.pac_datalen = sizeof(data);\
	pac_i2c_state = PAC_I2C_WAIT;\
	}

#define PAC_READREG(reg, data) {\
	pac_i2c_io.pac_type = PAC_READ;\
	pac_i2c_io.pac_reg = (reg);\
	pac_i2c_io.pac_data = &(data);\
	pac_i2c_io.pac_datalen = sizeof(data);\
	pac_i2c_state = PAC_I2C_WAIT;\
	}

#define PAC_WRITECMD(reg) {\
	pac_i2c_io.pac_type = PAC_WRITE;\
	pac_i2c_io.pac_reg = (reg);\
	pac_i2c_io.pac_data = NULL;\
	pac_i2c_io.pac_datalen = 0;\
	pac_i2c_state = PAC_I2C_WAIT;\
	}

/*
 * buffer to read accumulator count and all channel accumulators at once
 * as we're using readreg_be, the register order is inverted
 */
static struct { 
	u_char accum[3*7]; 
	pac_acccnt_t acc_count;
} _read_accum;

/*
 * buffer to read voltage/current values at once 
 * as we're using readreg_be, the register order is inverted
 * Disabled channels are skipped, so only 3 entries
 */
static struct {
	int16_t batt_i[3];
	uint16_t batt_v[3];
} _read_voltcur;

static enum {
	PAC_I2C_IDLE,
	PAC_I2C_ERROR, /* got error */
	PAC_I2C_COMPLETE, /* transaction complete */
	PAC_I2C_WAIT, /* waiting for i2c bus */
	PAC_I2C_WRITE,/* sending data */
	PAC_I2C_READ, /* reading data */
} pac_i2c_state;

static void
pac_i2c_exec(void)
{
	switch(pac_i2c_state) {
	case PAC_I2C_IDLE:
	case PAC_I2C_COMPLETE:
	case PAC_I2C_ERROR:
		return;
	case PAC_I2C_WAIT:
		switch(pac_i2c_io.pac_type) {
		case PAC_WRITE:
			i2c_writereg_be(PAC_I2C_ADDR, pac_i2c_io.pac_reg,
			    pac_i2c_io.pac_data, pac_i2c_io.pac_datalen,
			    &i2c_return);
			pac_i2c_state = PAC_I2C_WRITE;
			return;
		case PAC_READ:
			i2c_readreg_be(PAC_I2C_ADDR, pac_i2c_io.pac_reg,
			    pac_i2c_io.pac_data, pac_i2c_io.pac_datalen,
			    &i2c_return);
			pac_i2c_state = PAC_I2C_READ;
			return;
		case PAC_READ_ACCUM:
			i2c_readreg_be(PAC_I2C_ADDR, PAC_ACCCNT,
			    (void *)&_read_accum, sizeof(_read_accum), &i2c_return);
			pac_i2c_state = PAC_I2C_READ;
			return;
		case PAC_READ_AVG:
			i2c_readreg_be(PAC_I2C_ADDR, PAC_VBUS1_AVG,
			    (void *)&_read_voltcur, sizeof(_read_voltcur), &i2c_return);
			pac_i2c_state = PAC_I2C_READ;
			return;
		default:
			printf("pac_i2c_exec PAC_I2C_WAIT bad type %d\n",
			    pac_i2c_io.pac_type);
		}
		return;
	case PAC_I2C_WRITE:
	case PAC_I2C_READ:
		switch(i2c_return) {
		case I2C_INPROGRESS:
			return;
		case I2C_COMPLETE:
			pac_i2c_state = PAC_I2C_COMPLETE;
			return;
		case I2C_ERROR:
			/* XXX */
			printf("pac command %d %d fail\n", 
			    pac_i2c_io.pac_type, pac_i2c_io.pac_reg);
			pac_i2c_state = PAC_I2C_ERROR;
			return;
		}
	}
}

static char
pac_i2c_flush(void)
{
	while (pac_i2c_state != PAC_I2C_IDLE) {
		pac_i2c_exec();
		if (pac_i2c_state == PAC_I2C_ERROR) {
			printf("pac_i2c_flush: error %d state %d io_state %d\n",
			    i2c_return, pac_i2c_state, pac_i2c_io.pac_type);
			return 0;
		}
		if (pac_i2c_state == PAC_I2C_COMPLETE)
			pac_i2c_state = PAC_I2C_IDLE;
		i2c_wait(&i2c_return); /* will deal with timeout */
	}
	pac_i2c_io.pac_type = PAC_FREE;
	return 1;
}

/*
 * journal data structure (record every 10mn)
 * For current, in mA: 18 bits, including sign
 * voltage, max 20.47V needs 11 bits
 * temperature -40C to 60C: 8 bits (K - 233)
 * valid: 1bit
 * instance: 2 bits
 =>  total 40 bits, or 5 bytes
 51 entries per block of 256 bytes: 1 bytes free (for block flags)
 In 32k flash, 128 blocks -> 6528 entries, or 272 hours (11 days)
   with 4 channels active
 */

union log_entry {
	uint8_t data[5];
	struct {
		uint8_t temp;
		uint8_t u_low; /* low bits of voltage */
		uint16_t i_low; /* low bits current */
		uint8_t u_high : 3; /* high bits of voltage */
		uint8_t nvalid : 1; /* 0 = entry valid */
		uint8_t i_high: 2; /* high bits of current */
		uint8_t instance: 2; /* batt instance */
	} s;
};

#define LOG_ENTRIES 51

struct log_block {
	union log_entry b_entry[LOG_ENTRIES];
	uint8_t b_flags;
#define B_FILL_STAT 0x03
#define B_FILL_FREE 0x03
#define B_FILL_PART 0x01
#define B_FILL_FULL 0x00
#define B_FILL_GEN  0xfc
};

#define LOG_BLOCKS ((uint8_t)128)
#define LOG_BLOCKS_MASK (LOG_BLOCKS - 1)

extern const struct log_block battlog[LOG_BLOCKS] __at(0x18000);

extern struct log_block curlog __at(0x3700);

/* current log entry (to be updated) */
uint8_t log_cblk;
uint8_t log_centry;
uint8_t log_gen;

/* log requests/replies */
uint8_t logreq_len;
uint8_t logreq_id; /* current id for fast frame */

union __packed {
	uint8_t _data[233 + 8];
	struct private_log_request rq;
	struct private_log_reply rp;
	struct private_log_error er;
	struct private_log_reset rst;
} private_log_cmd;
static u_char fastid;

static inline void
utolog(uint16_t u, union log_entry *e)
{
	e->s.u_low = u & 0xff;
	e->s.u_high = (u >> 8) & 0x7;
}

static inline uint16_t
logtou(union log_entry *e)
{
	uint16_t u;

	u = e->s.u_low;
	u |= (uint16_t)(e->s.u_high & 0x7) << 8;
	return u;
}

static inline void
itolog(int64_t i, union log_entry *e)
{
	e->s.i_low = i & 0xffff;
	e->s.i_high = (i >> 16) & 0x3;
}

static inline int32_t
logtoi(union log_entry *e)
{
	int32_t i;
	i = e->s.i_low;
	i |= (uint32_t)(e->s.i_high & 0x3) << 16;
	if (e->s.i_high & 0x2) {
		i |= 0xfffe0000;
	}
	return i;
}

static void
page_erase(const void *p)
{
	uint8_t err = 0;
	__uint24 addr = (__uint24)p;

	printf("erase 0x%lx\n", (uint32_t)addr);

	NVMADR = addr;
	NVMCON1bits.CMD = 0x06;
	INTCON0bits.GIE = 0;

	NVMLOCK = 0x55;
	NVMLOCK = 0xAA;
	NVMCON0bits.GO = 1;

	while (NVMCON0bits.GO)
		; /* wait */

	if (NVMCON1bits.WRERR) {
		err++;
	}
	NVMCON1bits.CMD = 0;
	INTCON0bits.GIE = 1;
	if (err) {
		printf("erase 0x%lx failed\n", (uint32_t)addr);
	}
}

static void
page_read(const void *p)
{
	uint8_t err = 0;
	__uint24 addr = (__uint24)p;

	printf("read 0x%lx\n", (uint32_t)addr);

	NVMADR = addr;
	NVMCON1bits.CMD = 0x02;
	NVMCON0bits.GO = 1;

	while (NVMCON0bits.GO)
		; /* wait */

	NVMCON1bits.CMD = 0;
}

static void
page_write(const void *p)
{
	uint8_t err = 0;
	__uint24 addr = (__uint24)p;

	printf("write 0x%lx\n", (uint32_t)addr);

	NVMADR = addr;
	NVMCON1bits.CMD = 0x05;
	INTCON0bits.GIE = 0;

	NVMLOCK = 0x55;
	NVMLOCK = 0xAA;
	NVMCON0bits.GO = 1;

	while (NVMCON0bits.GO)
		; /* wait */

	if (NVMCON1bits.WRERR) {
		err++;
	}
	NVMCON1bits.CMD = 0;
	INTCON0bits.GIE = 1;
	if (err) {
		printf("write 0x%lx failed\n", (uint32_t)addr);
	}
}

static void
next_log_entry(void)
{
	printf("write log entry %d/%d\n", log_cblk, log_centry);
	/* mark entry as valid */
	curlog.b_entry[log_centry].s.nvalid = 0;
	curlog.b_flags = log_gen | B_FILL_PART;
	log_centry++;
	if (log_centry == LOG_ENTRIES) {
		/* next block */
		/* write current block */
		curlog.b_flags = log_gen | B_FILL_FULL;
		page_write(&battlog[log_cblk]);
		/* point to next block */
		log_cblk = (log_cblk + 1) & LOG_BLOCKS_MASK;
		log_centry = 0;
		if (log_cblk == 0) /* rollover; update gen number */
			log_gen += 0x4;
		/* erase and load new block */
		page_erase(&battlog[log_cblk]);
		page_read(&battlog[log_cblk]);
	} else {
		/* update current block */
		page_write(&battlog[log_cblk]);
	}
}

static void
log_erase(void)
{
	for (uint8_t c = 0; c < LOG_BLOCKS; c++) {
		page_erase(&battlog[c]);
	}
	log_cblk = log_centry = log_gen = 0;
}

static void
update_log(void)
{
	char c;
	double v;
	int32_t v_i;

	for (c = 0; c < 4; c++) {
		printf("log entry %d/%d ", log_cblk, log_centry);
		if ((pac_ctrl.ctrl_chan_dis & (8 >> c)) != 0)
			continue;
		/* batt_i = acc_value * 0.00075 * 1000 */
		v = (double)l600_current_acc[c] * 0.75 / l600_current_count;
		/* adjust with calibration data */
		switch(c) {
		case 0:
			v = v * 0.988689144013892;
			break;
		case 1:
			v = v * 1.00930129713152;
			break;
		case 2:
			v = v * 4.06331342566096;
			break;
		}
		v_i = (int32_t)(v + 0.5);
		printf(" %d %4.4fA %ld", c, v / 1000, v_i);
		itolog(v_i, &curlog.b_entry[log_centry]);
		/* volt = vbus * 0.000488 */
		/* batt_v = voltages_acc * 0.000488 * 100 / 6000; */
		/* adjust by 0.99955132 from calibration data */
		v = (double)l600_voltages_acc[c] * 0.000008129684;
		v_i = (int32_t)(v + 0.5);
		printf(" %4.3fV %ld", v / 100, v_i);
		utolog((uint16_t)v_i, &curlog.b_entry[log_centry]);
		curlog.b_entry[log_centry].s.instance = c;
#if 0
		XXX
		if (board_temp == 0xffff) {
			curlog.b_entry[log_centry].s.temp = 0xff;
		} else {
			curlog.b_entry[log_centry].s.temp =
			    (uint8_t)((board_temp - 23300) / 100);
		}
#endif
		next_log_entry();
		l600_current_acc[c] = 0;
		l600_voltages_acc[c] = 0;
	}
	l600_current_count = 0;
}

static void
send_log_block(uint8_t sid, uint8_t page)
{
	uint8_t c, i, j, r = 0;

	printf("send page %d\n", page);

	c = 0;
	while (c < LOG_ENTRIES && r == 0) {
		fastid = (fastid + 1) & 0x7;
		msg.id.id = 0;
		msg.id.iso_pg = (PRIVATE_LOG >> 8) & 0xff;
		msg.id.daddr = rid.saddr;
		msg.id.priority = NMEA2000_PRIORITY_ACK;
		msg.dlc = sizeof(struct private_log_reply);
		private_log_cmd.rp.cmd = PRIVATE_LOG_REPLY;
		msg.data = (void *)&private_log_cmd.rp;
		private_log_cmd.rp.sid = sid;
		private_log_cmd.rp.idx =
		   ((uint16_t)(battlog[page].b_flags & B_FILL_GEN) << 8) | page;
		for (i = 0; c < LOG_ENTRIES; c++) {
			if (battlog[page].b_entry[c].s.nvalid == 1) {
				printf("page %d entry %d !valid\n", page, c);
				private_log_cmd.rp.idx |= 0x100;
				r++;
				break;
			}
			for (j = 0; j < sizeof(union log_entry); j++) {
				private_log_cmd.rp.data[i] =
				    battlog[page].b_entry[c].data[j];
				i++; msg.dlc++;
			}
			if (msg.dlc >= (NMEA2000_DATA_FASTLENGTH - sizeof(union log_entry))) {
				c++;
				break;
			}
		}
		if (c == LOG_ENTRIES)
			private_log_cmd.rp.idx |= 0x100;
		printf("send fast len %d/%d, %d entries\n",
		    msg.dlc, i, c);
		if (! nmea2000_send_fast_frame(&msg, fastid))
			printf("send PRIVATE_LOG_REPLY failed\n");
	}
}

static void
send_log_error(uint8_t sid, uint8_t code)
{
	fastid = (fastid + 1) & 0x7;
	msg.id.id = 0;
	msg.id.iso_pg = (PRIVATE_LOG >> 8) & 0xff;
	msg.id.daddr = rid.saddr;
	msg.id.priority = NMEA2000_PRIORITY_ACK;
	msg.dlc = sizeof(struct private_log_error);
	msg.data = (void *)&private_log_cmd.er;
	private_log_cmd.er.cmd = PRIVATE_LOG_ERROR;
	private_log_cmd.er.sid = sid;
	private_log_cmd.er.error = code;
	if (! nmea2000_send_fast_frame(&msg, fastid))
		printf("send PRIVATE_LOG_ERROR failed\n");
}

static void
handle_log_request(uint8_t cmd) {
	uint8_t gen = (private_log_cmd.rq.idx & 0xff00) >> 8;
	uint8_t page = private_log_cmd.rq.idx & 0xff;
	uint8_t sid = private_log_cmd.rq.sid;
	uint8_t i;
	printf("log request sid %d gen %d page %d\n", sid, gen, page);

	if (cmd == PRIVATE_LOG_REQUEST_FIRST) {
		/* look for first log entry - usually next page */
		for (i = 0, page = (log_cblk + 1) & LOG_BLOCKS_MASK;
		    i < LOG_BLOCKS; 
		     page = (page + 1) & LOG_BLOCKS_MASK, i++) {
			if ((battlog[page].b_flags & B_FILL_STAT) !=
			    B_FILL_FREE)
				break;
		}
		if (i == LOG_BLOCKS) {
			/* all entries free (e.g. just after a reset) */
			send_log_error(sid, PRIVATE_LOG_ERROR_NOTFOUND);
			return;
		}
		send_log_block(sid, page);
		return;
	}
	/* look for gen/page */
	if ((battlog[page].b_flags & B_FILL_GEN) != gen ||
	    (battlog[page].b_flags & B_FILL_STAT) == B_FILL_FREE) {
		send_log_error(sid, PRIVATE_LOG_ERROR_NOTFOUND);
		return;
	}
	if (cmd == PRIVATE_LOG_REQUEST) {
		/* just send this page */
		send_log_block(sid, page);
		return;
	}
	/* send next page, if there is one */
	if (page == log_cblk) {
		/* this is the last page */
		send_log_error(sid, PRIVATE_LOG_ERROR_LAST);
		return;
	}
	page = (page + 1) & LOG_BLOCKS_MASK;
	if ((battlog[page].b_flags & B_FILL_STAT) == B_FILL_FREE) {
		/* next page is free, assume previous was the last */
		send_log_error(sid, PRIVATE_LOG_ERROR_LAST);
		return;
	}
	send_log_block(sid, page);
}

static void
adctotemp(u_char c)
{
	char i;
	for (i = 1; temps[i].val != 0; i++) {
		if (a2d_acc > temps[i].val) {
			board_temp = temps[i - 1].temp -
			((temps[i - 1].temp - temps[i].temp)  /
		         (temps[i - 1].val - temps[i].val)  * 
			 (temps[i - 1].val - a2d_acc));
			return;
		}
	} 
	board_temp = 0xffff;
}

static void
send_batt_status(char c)
{
	if (nmea2000_status != NMEA2000_S_OK)
		return;

	struct nmea2000_battery_status_data *data = (void *)&nmea2000_data[0];

	if ((pac_ctrl.ctrl_chan_dis & (8 >> c)) != 0)
		return;

	PGN2ID(NMEA2000_BATTERY_STATUS, msg.id);
	msg.id.priority = NMEA2000_PRIORITY_INFO;
	msg.dlc = sizeof(struct nmea2000_battery_status_data);
	msg.data = &nmea2000_data[0];
	data->voltage = (int16_t)batt_v[c];
	data->current = batt_i[c];
	data->temp = board_temp;
	data->sid = sid;
	data->instance = c;
	if (! nmea2000_send_single_frame(&msg))
		printf("send NMEA2000_BATTERY_STATUS failed\n");
}

#if 0

static void
send_dc_status(void)
{
	struct nmea2000_dc_status_data *data = (void *)&nmea2000_data[0];
	static u_char fastid;

	if (input_volt == 0xffff)
		return;

	fastid = (fastid + 1) & 0x7;
	printf("power voltage %d.%03dV\n",
	    (int)(input_volt / 1000), (int)(input_volt % 1000));

	PGN2ID(NMEA2000_DC_STATUS, msg.id);
	msg.id.priority = NMEA2000_PRIORITY_INFO;
	msg.dlc = sizeof(struct nmea2000_dc_status_data);
	msg.data = &nmea2000_data[0];
	data->sid = sid;
	data->instance = 0;
	switch(power_status) {
	case UNKOWN:
		return;
	case OFF:
		data->type = DCSTAT_TYPE_BATT;
		if (batt_v > 1240) {
			data->soc = 100 - ((u_long)time_on_batt * 100UL / 7200UL);
		} else {
			data->soc = 50 - (1240 - batt_v) * 50 / (1240 - 1100);
		}
		data->soh = 0xff;
		data->timeremain = 0xffff; /* XXX compute */
		data->ripple = 0xffff;
		break;
	default:
		/* assume operating on mains power */
		data->type = DCSTAT_TYPE_CONV;
		data->soc = 0xff;
		data->soh =
		    ((u_long)input_volt * 100UL + 6000UL) / 12000;
		data->timeremain = 0xffff;
		data->ripple = 0xffff;
		break;
	}
	if (! nmea2000_send_fast_frame(&msg, fastid))
		printf("send NMEA2000_DC_STATUS failed\n");
}

static void
send_charger_status()
{
	struct nmea2000_charger_status_data *data = (void *)&nmea2000_data[0];

	PGN2ID(NMEA2000_CHARGER_STATUS, msg.id);
	msg.id.priority = NMEA2000_PRIORITY_INFO;
	msg.dlc = sizeof(struct nmea2000_charger_status_data);
	msg.data = &nmea2000_data[0];
	data->instance = 0;
	data->batt_instance = 0;
	data->op_state = charger_status;
	data->mode = CHARGER_MODE_STANDALONE;
	data->enable = 1;
	data->eq_pending = 0;
	data->eq_time_remain = 0;
	if (! nmea2000_send_single_frame(&msg))
		printf("send NMEA2000_CHARGER_STATUS failed\n");
}
#endif

void
user_handle_iso_request(u_long pgn)
{
	printf("ISO_REQUEST for %ld from %d\n", pgn, rid.saddr);
	switch(pgn) {
	case NMEA2000_BATTERY_STATUS:
		for (char c = 0; c < 4; c++) {
			if ((pac_ctrl.ctrl_chan_dis & (8 >> c)) == 0)
				send_batt_status(c);
		}
		break;
#if 0
	case NMEA2000_DC_STATUS:
		send_dc_status();
		break;
	case NMEA2000_CHARGER_STATUS:
		send_charger_status();
		break;
#endif
	}
}

void
user_receive()
{
	u_long pgn;

	pgn = ((u_long)rid.page << 16) | ((u_long)rid.iso_pg << 8);
	if (rid.iso_pg > 239)
		pgn |= rid.daddr;

	switch(pgn) {
	case PRIVATE_LOG:
	    {
		u_char idx = (rdata[0] & FASTPACKET_IDX_MASK);
		u_char id =  (rdata[0] & FASTPACKET_ID_MASK);
		char i, j;

		if (idx == 0) {
			/* new head packet */
			logreq_id = id;
			logreq_len = rdata[1];
			for (i = 0; i < 6 && logreq_len > 0; i++) {  
				private_log_cmd._data[i] = rdata[i+2];
				logreq_len--;
			}       
		} else if (id == logreq_id) {
			j = 1;
			/* i = 6 + (idx - 1) * 7 : i = idx * 7 - 1 */   
			for (i = idx * 7 - 1, j = 1;
			    i < sizeof(private_log_cmd) && j < 8 &&
			        logreq_len > 0;
			    i++, j++) {
				private_log_cmd._data[i] = rdata[j];  
				logreq_len--;
			}
		}

		if (logreq_len == 0) {
			switch(private_log_cmd.rq.cmd) {
			case PRIVATE_LOG_REQUEST:
			case PRIVATE_LOG_REQUEST_FIRST:
			case PRIVATE_LOG_REQUEST_NEXT:
				handle_log_request(private_log_cmd.rq.cmd);
				break;
			case PRIVATE_LOG_RESET:
				printf("log reset from %d ", rid.saddr);
				if (rid.daddr != nmea2000_addr) {
					printf("ignored, wrong daddr %d\n",
					    rid.daddr);
				} else if (private_log_cmd.rst.magic != 
				    PRIVATE_LOG_RESET_MAGIC) {
					printf("ignored, wrong magic 0x%x\n",
					    private_log_cmd.rst.magic);
				} else {
					log_erase();
					printf("done\n");
				}
				break;
			default:
				printf("wrong log cmd %d from %d\n",
				    private_log_cmd.rq.cmd, rid.saddr);
			}
		}
		break;
	    }
	}
}

void
putch(char c)
{
        if (PORTBbits.RB7) {
		usart_putchar(c);
	}
}

static union pacops_pending {
	struct pacops_bits {
		char refresh : 1;	/* send a refresh command */
		char refresh_v : 1;	/* send a refresh_v command */
		char read_values : 1;	/* read average values */
		char read_accum : 1;	/* read accumulator values */
	} bits;
	char byte;
} pacops_pending;

u_int pac_refresh_time;
u_char pac_refresh_valid;

static void
pac_command_complete(void)
{
	switch(pac_i2c_io.pac_type) {
	case PAC_WRITE:
		if (pac_i2c_io.pac_reg == PAC_REFRESH ||
		    pac_i2c_io.pac_reg == PAC_REFRESH_V) {
			pac_refresh_time = timer0_read();
			pac_refresh_valid = 1;
		}
		break;
	case PAC_READ_AVG:
		softintrs.bits.int_pacavg = 1;
		break;
	case PAC_READ_ACCUM:
		softintrs.bits.int_pacacc = 1;
		break;
	default:
		printf("unknown pac_complete %d\n", pac_i2c_io.pac_type);
		break;
	}
	pac_i2c_io.pac_type = PAC_FREE;
	pac_i2c_state = PAC_I2C_IDLE;
}

static void
pac_command_schedule(void)
{
	if (pac_i2c_state != PAC_I2C_IDLE)
		return;

	if (pacops_pending.bits.refresh) {
		pacops_pending.bits.refresh_v = 0;
		pacops_pending.bits.refresh = 0;
		pac_refresh_valid = 0;
		PAC_WRITECMD(PAC_REFRESH);
		return;
	}
	if (pacops_pending.bits.refresh_v) {
		pacops_pending.bits.refresh_v = 0;
		PAC_WRITECMD(PAC_REFRESH_V);
		pac_refresh_valid = 0;
		return;
	}
	if ((pacops_pending.bits.read_values ||
	    pacops_pending.bits.read_accum) && pac_refresh_valid) {
		if ((timer0_read() - pac_refresh_time) < TIMER0_1MS)
			return;
		if (pacops_pending.bits.read_values) {
			pacops_pending.bits.read_values = 0;
			pac_i2c_io.pac_type = PAC_READ_AVG;
			softintrs.bits.int_pacavg = 0;
		} else {
			pacops_pending.bits.read_accum = 0;
			pac_i2c_io.pac_type = PAC_READ_ACCUM;
			softintrs.bits.int_pacacc = 0;
		}
		pac_i2c_state = PAC_I2C_WAIT;
		return;
	}
}

#if 0
static void
read_pac_channel(void)
{
	char c;
	static pac_acccnt_t pac_acccnt;
	int64_t acc_value;
	double v;

	i2c_readreg_be(PAC_I2C_ADDR, PAC_ACCCNT,
	    (void *)&pac_acccnt, sizeof(pac_acccnt), &i2c_return);
	if (i2c_wait(&i2c_return) != 0) {
		printf("read pac_acccnt fail\n");
		pac_acccnt.acccnt_count = 0;
		return;
	} 
	printf("\n %d count %lu", NCANOK, pac_acccnt.acccnt_count);
	l600_current_count += pac_acccnt.acccnt_count;

	for (c = 0; c < 4; c++) {
		if ((pac_ctrl.ctrl_chan_dis & (8 >> c)) != 0)
			continue;

		acc_value = 0;
		i2c_readreg_be(PAC_I2C_ADDR, PAC_ACCV1 + c,
		    (void *)&acc_value, 7, &i2c_return);
		if (i2c_wait(&i2c_return) != 0) {
			printf("read acc_value[%d] fail\n", c);
			continue;
		}
		if (acc_value & 0x0080000000000000) {
			/* adjust negative value */
			acc_value |= 0xff00000000000000;
		}
		l600_current_acc[c] += acc_value;
		/* batt_i = acc_value * 0.00075 * 100 */
		v = (double)acc_value * 0.075 / pac_acccnt.acccnt_count;
		/* adjust with calibration data */
		switch(c) {
		case 0:
			v = v * 0.988689144013892;
			break;
		case 1:
			v = v * 1.00930129713152;
			break;
		case 2:
			v = v * 4.06331342566096;
			break;
		}
		batt_i[c] = (int16_t)(v + 0.5);
		printf(" %d %4.4fA", c, v / 100);
		/* volt = vbus * 0.000488 */
		/* batt_v = voltages_acc * 0.000488 * 100 / 10; */
		/* adjust by 0.99955132 from calibration data */
		v = (double)voltages_acc[c] * 0.0048778104;
		batt_v[c] = (int16_t)(v + 0.5);
		printf(" %4.3fV", v / 100);
	}
}
#endif

#define OLED_ADDR 0x78 // 0b01111000
#define OLED_DISPLAY_SIZE 1024 /* 128 * 64 / 8 */
#define OLED_DISPLAY_W 128
#define OLED_DISPLAY_H (64 / 8)

#define DISPLAY_FONTSMALL_W     6

char oled_displaybuf[OLED_DISPLAY_W / DISPLAY_FONTSMALL_W];
static u_char oled_col;
static u_char oled_line;

char oled_error; /* is display in error state ? */

u_char bright;

/* i2c OLED specific */

#define OLED_I2C_DATABUFSZ	256
#define OLED_I2C_CTRLBUFSZ	16
#define OLED_NBUFS		4

static struct oled_i2c_buf_s {
	u_char oled_databuf[OLED_I2C_DATABUFSZ];
	u_char oled_ctrlbuf[OLED_I2C_CTRLBUFSZ];
	uint16_t  oled_datalen;
	u_char oled_ctrllen;
	enum {
		OLED_CTRL_FREE,
		OLED_CTRL_CLEAR,
		OLED_CTRL_CMD,
		OLED_CTRL_DISPLAY,
	} oled_type;
} oled_i2c_buf[OLED_NBUFS];

static enum {
	OLED_I2C_IDLE,
	OLED_I2C_ERROR, /* got error */
	OLED_I2C_COMPLETE, /* transaction complete */
	OLED_I2C_WAIT, /* waiting for i2c bus */
	OLED_I2C_CMD, /* command sent */
	OLED_I2C_DATA, /* data sent */
} oled_i2c_state;

static u_char oled_i2c_prod;
static u_char oled_i2c_cons;

static void
_oled_i2c_exec(void)
{
	struct oled_i2c_buf_s *oled_s;

	switch(oled_i2c_state) {
	case OLED_I2C_IDLE:
	case OLED_I2C_COMPLETE:
	case OLED_I2C_ERROR:
		return;
	case OLED_I2C_WAIT:
		while (oled_i2c_buf[oled_i2c_cons].oled_type == OLED_CTRL_FREE) {
			oled_i2c_cons = (oled_i2c_cons + 1) & (OLED_NBUFS - 1);
			if (oled_i2c_cons == oled_i2c_prod &&
			    oled_i2c_buf[oled_i2c_cons].oled_type == OLED_CTRL_FREE) {
				return;
			}
		}
		oled_s = &oled_i2c_buf[oled_i2c_cons];
		i2c_writereg(OLED_ADDR, 0, &oled_s->oled_ctrlbuf[0],
		    oled_s->oled_ctrllen, &i2c_return);
		oled_i2c_state = OLED_I2C_CMD;
		return;
	case OLED_I2C_CMD:
		oled_s = &oled_i2c_buf[oled_i2c_cons];
		switch(i2c_return) {
		case I2C_INPROGRESS:
			return;
		case I2C_ERROR:
			if (!oled_error)
				printf("oled_i2c_exec CMD fail (%d)\n",
				    oled_s->oled_type);
			oled_i2c_state = OLED_I2C_ERROR;
			oled_error = 1;
			return;
		case I2C_COMPLETE:
			oled_error = 0;
			break;
		}
		switch(oled_s->oled_type) {
		case OLED_CTRL_DISPLAY:
			i2c_writereg_dma(OLED_ADDR, 0x40, &oled_s->oled_databuf[0],
			    oled_s->oled_datalen, &i2c_return);
			oled_i2c_state = OLED_I2C_DATA;
			return;
		case OLED_CTRL_CMD:
			oled_i2c_state = OLED_I2C_COMPLETE;
			return;
		case OLED_CTRL_CLEAR:
			break;
		default:
			printf("oled_i2c_exec: OLED_I2C_CMD cons %d cons_state %d\n",
			    oled_i2c_cons,
			    oled_i2c_buf[oled_i2c_cons].oled_type);
			return;
		}
		/* FALLTHROUGH */
	case OLED_I2C_DATA:
		oled_s = &oled_i2c_buf[oled_i2c_cons];
		switch(i2c_return) {
		case I2C_INPROGRESS:
			return;
		case I2C_ERROR:
			if (!oled_error)
				printf("oled_i2c_exec DATA fail (%d)\n",
				    oled_s->oled_type);
			oled_i2c_state = OLED_I2C_ERROR;
			oled_error = 1;
			return;
		case I2C_COMPLETE:
			break;
		}
		switch(oled_s->oled_type) {
		case OLED_CTRL_DISPLAY:
			oled_i2c_state = OLED_I2C_COMPLETE;
			return;
		case OLED_CTRL_CLEAR:
			if (oled_s->oled_datalen != 0) {
				i2c_writereg_dma(OLED_ADDR, 0x40,
				    &oled_s->oled_databuf[0],
				    OLED_I2C_DATABUFSZ, &i2c_return);
				oled_s->oled_datalen -= OLED_I2C_DATABUFSZ;
				oled_i2c_state = OLED_I2C_DATA;
			} else {
				oled_i2c_state = OLED_I2C_COMPLETE;
			}
			return;
		default:
			break;
		}
	default:
		printf("oled_i2c_exec: state %d cons %d cons_state %d\n",
		    oled_i2c_state, oled_i2c_cons,
		    oled_i2c_buf[oled_i2c_cons].oled_type);
	}
}

static void
oled_i2c_exec(void)
{
	_oled_i2c_exec();
	if (oled_i2c_state == OLED_I2C_COMPLETE) {
		oled_i2c_buf[oled_i2c_cons].oled_type = OLED_CTRL_FREE;
		oled_i2c_state = OLED_I2C_IDLE;
	}
	if (oled_i2c_state == OLED_I2C_IDLE &&
	    oled_i2c_cons != oled_i2c_prod) {
		/* schedule next command */
		oled_i2c_cons = (oled_i2c_cons + 1) & (OLED_NBUFS - 1);
		oled_i2c_state = OLED_I2C_WAIT;
	}
}

static char
oled_i2c_flush(void)
{
	while (oled_i2c_state != OLED_I2C_IDLE) {
		oled_i2c_exec();
		if (oled_i2c_state == OLED_I2C_ERROR) {
			printf("oled_i2c_flush: error %d state %d cons %d cons_state %d\n",
			    i2c_return, oled_i2c_state, oled_i2c_cons,
			    oled_i2c_buf[oled_i2c_cons].oled_type);
			oled_i2c_state = OLED_I2C_IDLE;
			oled_i2c_buf[oled_i2c_cons].oled_type = OLED_CTRL_FREE;
			oled_i2c_exec();
		} else {
			i2c_wait(&i2c_return); /* will deal with timeout */
		}
	}
	return 1;
}
			

#define OLED_CTRL(s, c) \
        {(s)->oled_ctrlbuf[(s)->oled_ctrllen++] = (c);}

static struct oled_i2c_buf_s *
oled_i2c_reset(void)
{
	/* select next free slot */
	while (oled_i2c_buf[oled_i2c_prod].oled_type != OLED_CTRL_FREE) {
		u_char new_i2c_prod;
		new_i2c_prod = (oled_i2c_prod + 1) & (OLED_NBUFS - 1);
		if (new_i2c_prod == oled_i2c_cons) {
			/* no free slot */
			return NULL;
		}
		oled_i2c_prod = new_i2c_prod;
	}
	oled_i2c_buf[oled_i2c_prod].oled_datalen = 0;
	oled_i2c_buf[oled_i2c_prod].oled_ctrllen = 0;
	return &oled_i2c_buf[oled_i2c_prod];
}
#define OLED_CTRL_WRITE {oled_s->oled_type = OLED_CTRL_CMD; oled_i2c_state = OLED_I2C_WAIT; oled_i2c_flush(); oled_i2c_reset();}
	
static void
displaybuf_small(void)
{
	const u_char *font;
	char *cp;
	u_char len;
	u_char i;
	struct oled_i2c_buf_s *oled_s;

	if ((oled_s = oled_i2c_reset()) == NULL)
		return;

	len = 0;
	for (cp = oled_displaybuf; *cp != '\0'; cp++) {
		if (*cp == '\n' && len == 0) {
			len = (u_char)oled_s->oled_datalen;
			continue;
		}
		font = get_font5x8(*cp);
		for (i = 0; i < 5; i++) {
			oled_s->oled_databuf[oled_s->oled_datalen++] = font[i];
		}
		oled_s->oled_databuf[oled_s->oled_datalen++] = 0;
	}
	if (len == 0)
		len = (u_char)oled_s->oled_datalen;
	/* set column start/end */
	OLED_CTRL(oled_s, 0x00); OLED_CTRL(oled_s, 0x10);  /* reset column start */
	OLED_CTRL(oled_s, 0x21); OLED_CTRL(oled_s, oled_col);
	    OLED_CTRL(oled_s, oled_col + len - 1); /*column start/end */
	/* set page address */
	OLED_CTRL(oled_s, (oled_line & 0x07 ) | 0xb0 );
	oled_s->oled_type = OLED_CTRL_DISPLAY;
	if (oled_i2c_state == OLED_I2C_IDLE)
		oled_i2c_state = OLED_I2C_WAIT;
}

static void
displaybuf_medium(void)
{
	const u_char *font;
	char *cp;
	u_char i, n;
	struct oled_i2c_buf_s *oled_s;

	if ((oled_s = oled_i2c_reset()) == NULL)
		return;
	/* unfortunably we can't build the two lines in a single loop */
	for (n = 0, cp = oled_displaybuf; *cp != '\0'; cp++) {
		font = get_font10x16(*cp);
		for (i = 0; i < 10; i++, n++) {
			oled_s->oled_databuf[n] = font[i * 2];
		}
	}
	for (cp = oled_displaybuf; *cp != '\0'; cp++) {
		font = get_font10x16(*cp);
		for (i = 0; i < 10; i++, n++) {
			oled_s->oled_databuf[n] = font[i * 2 + 1];
		}
	}
	oled_s->oled_datalen = n;
	OLED_CTRL(oled_s, 0x00); OLED_CTRL(oled_s, 0x10);  /* reset column start */
	OLED_CTRL(oled_s, 0x21); OLED_CTRL(oled_s, oled_col); OLED_CTRL(oled_s, oled_col + n / 2 - 1); /*column start/end */
	/* set page address */
	OLED_CTRL(oled_s, (oled_line & 0x07 ) | 0xb0 );
	oled_s->oled_type = OLED_CTRL_DISPLAY;
	if (oled_i2c_state == OLED_I2C_IDLE)
		oled_i2c_state = OLED_I2C_WAIT;
}

static void
displaybuf_icon(char ic)
{
	const u_char *icon;
	char *cp;
	u_char i;
	struct oled_i2c_buf_s *oled_s;

	if ((oled_s = oled_i2c_reset()) == NULL)
		return;
	icon = get_icons16x16(ic);
	for (i = 0; i < 16; i++) {
		oled_s->oled_databuf[i] = icon[i * 2];
		oled_s->oled_databuf[i + 16] = icon[i * 2 + 1];
	}
	oled_s->oled_datalen = 32;
	OLED_CTRL(oled_s, 0x00); OLED_CTRL(oled_s, 0x10);  /* reset column start */
	OLED_CTRL(oled_s, 0x21); OLED_CTRL(oled_s, oled_col); OLED_CTRL(oled_s, oled_col + 16 - 1); /*column start/end */
	/* set page address */
	OLED_CTRL(oled_s, (oled_line & 0x07 ) | 0xb0 );
	oled_s->oled_type = OLED_CTRL_DISPLAY;
	if (oled_i2c_state == OLED_I2C_IDLE)
		oled_i2c_state = OLED_I2C_WAIT;
}

static void
dislplay_battstat(u_char b)
{
	double amps = (double)batt_i[b] / 100;
	double volts = (double)batt_v[b] / 100;
	if (amps > 9.99 || amps < 0) {
		if (volts < 10.0) {
			sprintf(oled_displaybuf,
			    " %1.01fV\n-.--A", volts);
		} else {
			sprintf(oled_displaybuf,
			    "%2.01fV\n-.--A", volts);
		}
	} else {
		if (volts < 10.0) {
			sprintf(oled_displaybuf,
			    " %1.01fV\n%1.02fA", volts, amps);
		} else {
			sprintf(oled_displaybuf,
			    "%2.01fV\n%1.02fA", volts, amps);
		}
	}
	displaybuf_small();
}

int
main(void)
{
	char c;
	uint16_t l;
	uint8_t i2cr;
	pac_accumcfg_t pac_accumcfg;
	pac_neg_pwr_fsr_t pac_neg_pwr_fsr;
	static u_int poll_count;
	uint16_t t0;
	static int32_t voltages_acc_cur[4];
	uint8_t new_boot;
	struct oled_i2c_buf_s *oled_s;

	default_src = 0;
	devid = 0;
	revid = 0;
	nmea2000_user_id = 0;
	/* read devid and user IDs */
	TBLPTR = 0x200000;
	asm("tblrd*+;");
	asm("movff TABLAT, _nmea2000_user_id;");
	asm("tblrd*+;");
	asm("movff TABLAT, _nmea2000_user_id + 1;");
	TBLPTR = 0x3ffffc;
	asm("tblrd*+;");
	asm("movff TABLAT, _revid;");
	asm("tblrd*+;");
	asm("movff TABLAT, _revid + 1;");
	asm("tblrd*+;");
	asm("movff TABLAT, _devid;");
	asm("tblrd*+;");
	asm("movff TABLAT, _devid + 1;");

	/* disable unused modules */
	PMD0 = 0x3a; /* keep clock, FVR, and IOC */
#ifdef USE_TIMER2
	PMD1 = 0xfa; /* keep timer0/timer2 */
	PMD2 = 0x03; /* keep can module */
#else
	PMD1 = 0xfe; /* keep timer0 */
	PMD2 = 0x02; /* keep can module, TU16A */
#endif
	PMD3 = 0xd9; /* keep ADC, CM1, CM2 */
	PMD4 = 0xff;
	PMD5 = 0xff;
	PMD6 = 0xf6; /* keep UART1 and I2C */
	PMD7 = 0xf3; /* keep CLC3 and CLC4 */
	PMD8 = 0xfe; /* keep DMA1 */

	ANSELC = 0;
	ANSELB = 0;
	ANSELA = 0x2F; /* RA0, RA1, RA2, RA3 and RA5 analog */

	/* CANRX on RB3 */
	CANRXPPS = 0x0B;
	/* CANTX on RB2 */
	LATBbits.LATB2 = 1; /* output value when idle */
	TRISBbits.TRISB2 = 0;
	RB2PPS = 0x46;

	LATA = 0;
	OLED_RSTN = 0;
	PAC_NDOWN = 0;
	TRISAbits.TRISA4 = 0; /* RA4/OLED_RSTN as outpout */
	TRISCbits.TRISC7 = 0; /* RC7/NDOWN output */

	/* configure watchdog timer for 2s */
	WDTCON0 = 0x16;
	WDTCON1 = 0x07;

	/* configure sleep mode: PRI_IDLE */
	CPUDOZE = 0x80;

	/*
	 * configure priotities memory access priorities
	 * ISR > DMA1 > main
	 */
	PRLOCK = 0x55;
	PRLOCK = 0xAA;
	PRLOCKbits.PRLOCKED = 0;
	ISRPR = 0;
	MAINPR = 4;
	DMA1PR = 3;
	PRLOCK = 0x55;
	PRLOCK = 0xAA;
	PRLOCKbits.PRLOCKED = 1;


	softintrs.byte = 0;
	counter_10hz = 25;
	counter_1hz = 10;
	seconds = 0;

	for (c = 0; c < 4; c++) {
		voltages_acc_cur[c] = 0;
		l600_current_acc[c] = 0;
		l600_voltages_acc[c] = 0;
	}
	l600_current_count = 0;

	IPR1 = 0;
	IPR2 = 0;
	IPR3 = 0;
	IPR4 = 0;
	IPR5 = 0;
	IPR6 = 0;
	IPR7 = 0;
	IPR8 = 0;
	IPR9 = 0;
	IPR10 = 0;
	IPR11 = 0;
	IPR12 = 0;
	IPR13 = 0;
	IPR14 = 0;
	IPR15 = 0;
	INTCON0 = 0;
	INTCON1 = 0;
	INTCON0bits.IPEN=1; /* enable interrupt priority */

	USART_INIT(0);

	/* configure timer0 as free-running counter at 9.765625Khz * 4 */
	T0CON0 = 0x0;
	T0CON0bits.MD16 = 1; /* 16 bits */
	T0CON1 = 0x48; /* 01001000 Fosc/4, sync, 1/256 prescale */
	PIR3bits.TMR0IF = 0;
	PIE3bits.TMR0IE = 0; /* no interrupt */
	T0CON0bits.T0EN = 1;

#ifdef USE_TIMER2
	/* configure timer2 for 250Hz interrupt */
	T2CON = 0x44; /* b01000100: postscaller 1/5, prescaler 1/16 */
	T2PR = 125; /* 250hz output */
	T2CLKCON = 0x01; /* Fosc / 4 */
	T2CONbits.TMR2ON = 1;
	PIR3bits.TMR2IF = 0;
	IPR3bits.TMR2IP = 1; /* high priority interrupt */
	PIE3bits.TMR2IE = 1;
#else
	/* configure UTMR for 10Hz interrupt */
	TU16ACON0 = 0x04; /* period match IE */
	TU16ACON1 = 0x00;
	TU16AHLT = 0x00; /* can't use hardware reset because of HW bug */
	TU16APS = 249; /* prescaler = 250 -> 40000 Hz */
	TU16APRH = (4000 >> 8) & 0xff;
	TU16APRL = 4000 & 0xff;
	TU16ACLK = 0x2; /* Fosc */
	TUCHAIN = 0;
	TU16ACON0bits.ON = 1;
	TU16ACON1bits.CLR = 1;
	IPR0bits.TU16AIP = 1; /* high priority interrupt */
	PIE0bits.TU16AIE = 1;
#endif

	i2c_init();

	INTCON0bits.GIEH=1;  /* enable high-priority interrupts */   
	INTCON0bits.GIEL=1; /* enable low-priority interrrupts */   

	printf("hello user_id 0x%lx devid 0x%x revid 0x%x\n", nmea2000_user_id, devid, revid);

	printf("n2k_init\n");
	nmea2000_init();
	poll_count = timer0_read();

	/* enable watchdog */
	WDTCON0bits.SEN = 1;

	/* set up ADC */
	ADCON0 = 0x4; /* right-justified */
	ADCLK = 7; /* Fosc/16 */
	/* context 0: PWM_v */
	ADCTX = 0;
	ADCON1 = 0; /* no cap */
	ADCON2 = 0; /* basic mode */
	ADCON3 = 0x80; /* SOI */
	ADREF = 2;  /* vref = vref+ (RA3) */
	ADPCH = 0; /* RA0 */
	ADACQH = 0;
	ADACQL = 20; /* 20Tad Aq */
	/* context 1: solar_v */
	ADCTX = 1;
	ADCON1 = 0; /* no cap */
	ADCON2 = 0; /* basic mode */
	ADCON3 = 0x80; /* SOI */
	ADREF = 2;  /* vref = vref+ (RA3) */
	ADPCH = 2; /* RA2 */
	ADACQH = 0;
	ADACQL = 20; /* 20Tad Aq */
	/* XXX setup thresholds */
	/* context 2: NTC */
	ADCTX = 2;
	ADCON1 = 0; /* no cap */
	ADCON2 = 0; /* basic mode */
	ADCON3 = 0x80; /* SOI */
	ADREF = 0;  /* vref = VDD */
	ADPCH = 5; /* RA5 */
	ADACQH = 0;
	ADACQL = 20; /* 20Tad Aq */
	/* XXX setup thresholds */
	/* context 3: buttons */
	ADCTX = 3;
	ADCON1 = 0; /* no cap */
	ADCON2 = 0; /* basic mode */
	ADCON3 = 0; /* basic mode */
	ADREF = 0;  /* vref = VDD */
	ADPCH = 1; /* RA1 */
	ADACQH = 0;
	ADACQL = 20; /* 20Tad Aq */

	ADCON0bits.ADON = 1;
	PIR1bits.ADIF = 0;
	PIE1bits.ADIE = 0;

	printf("display init");
	oled_i2c_prod = oled_i2c_cons = 0;
	oled_error = 0;
	oled_i2c_state = OLED_I2C_IDLE;
	for (c = 0; c < OLED_NBUFS; c++) {
		oled_i2c_buf[c].oled_type = OLED_CTRL_FREE;
	}
	/* setup OLED */
	oled_s = &oled_i2c_buf[oled_i2c_prod];
	for (l = 0; l < 1000; l++)    
		CLRWDT();
	OLED_RSTN = 1;
	for (l = 0; l < 1000; l++)    
		CLRWDT();
	printf(" 1");

	/* command lock */
	oled_i2c_reset();
	OLED_CTRL(oled_s, 0xfd); OLED_CTRL(oled_s, 0x12);
	OLED_CTRL_WRITE;
	CLRWDT();

	/* display off */
	OLED_CTRL(oled_s, 0xae);
	OLED_CTRL_WRITE;
	CLRWDT();

	/* set freq */
	OLED_CTRL(oled_s, 0xd5); OLED_CTRL(oled_s, 0xa0);
	OLED_CTRL_WRITE;
	CLRWDT();
	/* set multiplexer */
	OLED_CTRL(oled_s, 0xa8); OLED_CTRL(oled_s, 0x3f);
	OLED_CTRL_WRITE;
	CLRWDT();

	/* display offset */
	OLED_CTRL(oled_s, 0xd3); OLED_CTRL(oled_s, 0x00);
	/* start line */
	OLED_CTRL(oled_s, 0x40);
	/* segment remap */
	OLED_CTRL(oled_s, 0xa0);
	/* com output scan direction */
	OLED_CTRL(oled_s, 0xc0);
	/* com pin hardware config */
	OLED_CTRL(oled_s, 0xda); OLED_CTRL(oled_s, 0x12);
	/* current control */
	// OLED_CTRL(oled_s, 0x81); OLED_CTRL(oled_s, 0xdf);
	OLED_CTRL(oled_s, 0x81); OLED_CTRL(oled_s, bright);
	/* pre-charge period */
	OLED_CTRL(oled_s, 0xd9); OLED_CTRL(oled_s, 0x82);
	/* vcomh deselect level */
	OLED_CTRL(oled_s, 0xdb); OLED_CTRL(oled_s, 0x34);
	/* entire display on/off */
	OLED_CTRL(oled_s, 0xa4);
	/* normal/inverse */
	OLED_CTRL(oled_s, 0xa6);
	OLED_CTRL_WRITE;
	CLRWDT();
	/* clear RAM */
	OLED_CTRL(oled_s, 0x20); OLED_CTRL(oled_s, 0x00); /* horizontal addressing mode */
	OLED_CTRL(oled_s, 0x21); OLED_CTRL(oled_s, 0x00); OLED_CTRL(oled_s, 0x7f); /*column start/end */
	OLED_CTRL(oled_s, 0x22); OLED_CTRL(oled_s, 0x00); OLED_CTRL(oled_s, 0x07); /*page start/end */
	OLED_CTRL(oled_s, 0x00); OLED_CTRL(oled_s, 0x10);  /* column start */
	OLED_CTRL_WRITE;
	CLRWDT();
	memset(oled_s->oled_databuf, 0, OLED_I2C_DATABUFSZ);
	printf(" buf size %d", (int)OLED_I2C_DATABUFSZ);
	oled_s->oled_databuf[0] = 0x1;
	oled_s->oled_datalen = OLED_DISPLAY_SIZE;
	OLED_CTRL(oled_s, 0x00); OLED_CTRL(oled_s, 0x10);  /* reset column start */
	OLED_CTRL(oled_s, 0x21); OLED_CTRL(oled_s, 0); OLED_CTRL(oled_s, 0x7f); /*column start/end */
	OLED_CTRL(oled_s, 0xb0); /* page start */
	oled_s->oled_type = OLED_CTRL_CLEAR;
	oled_i2c_state = OLED_I2C_WAIT;
	oled_i2c_flush();

	/* set display on */
	oled_i2c_reset();
	OLED_CTRL(oled_s, 0xaf);
	OLED_CTRL_WRITE;
	CLRWDT();
	printf(" done\n");

	PAC_NDOWN = 1;
	pacops_pending.byte = 0;
	/* wait 50ms for pac1953 to be up */
	for (c = 0; c < 50; c++) {
		t0 = timer0_read();
		while (timer0_read() - t0 < TIMER0_1MS) {
			; /* wait */
		}
	}

#ifdef USELOG
#if 0
	log_erase();
#endif

	/* look for current block */
	printf("blocks");
	for (c = 0; c < LOG_BLOCKS; c++) {
		printf(" 0x%x", battlog[c].b_flags);
		if ((battlog[c].b_flags & B_FILL_STAT) == B_FILL_PART) {
			/* found it */
			log_cblk = c;
			log_gen = battlog[c].b_flags & B_FILL_GEN;
			break;
		}
		if ((battlog[c].b_flags & B_FILL_STAT) == B_FILL_FULL) {
			uint8_t c2 = (c + 1) & LOG_BLOCKS_MASK;
			if ((battlog[c2].b_flags & B_FILL_STAT)
			    == B_FILL_FREE) {
				/* stopped just at boundary */
				log_cblk = c2;
				log_gen = battlog[c].b_flags & B_FILL_GEN;
				if (log_cblk == 0)
					log_gen += 0x4;
				break;
			}
		}
	}
	printf("\n");
	if (c == LOG_BLOCKS) {
		/* log empty, start a 0 */
		printf("log empty\n");
		log_cblk = 0;
		log_gen = 0;
	}
	/* look for first free entry in block */
	for (c = 0; c < LOG_ENTRIES; c++) {
		if (battlog[log_cblk].b_entry[c].s.nvalid == 1) {
			/* first free entry */
			log_centry = c;
			break;
		}
	}
	if (c == LOG_ENTRIES) {
		/* log corrupted, reset */
		printf("non-full block with no free entry: reset log\n");
		log_erase();
	}
	page_read(&battlog[log_cblk]);

	memset(&curlog.b_entry[log_centry], 0, sizeof(union log_entry));

	next_log_entry();
#endif /* USELOG */

again:
	c = 0;
	PAC_READREG(PAC_PRODUCT, i2cr);
	if (pac_i2c_flush() == 0)
		c++;
	else 
		printf("product id 0x%x ", i2cr);

	PAC_READREG(PAC_MANUF, i2cr);
	if (pac_i2c_flush() == 0)
		c++;
	else
		printf("manuf id 0x%x ", i2cr);

	PAC_READREG(PAC_REV, i2cr);
	if (pac_i2c_flush() == 0)
		c++;
	else
		printf("rev id 0x%x\n", i2cr);

	PAC_READREG(PAC_CTRL_ACT, pac_ctrl);
	if (pac_i2c_flush() == 0)
		c++;
	else {
		printf("CTRL_ACT al1 %d al2 %d mode %d dis %d\n",
		    pac_ctrl.ctrl_alert1,
		    pac_ctrl.ctrl_alert2,
		    pac_ctrl.ctrl_mode,
		    pac_ctrl.ctrl_chan_dis);
	}

	pac_ctrl.ctrl_alert1 = pac_ctrl.ctrl_alert2 = CTRL_ALERT_ALERT;
	pac_ctrl.ctrl_mode = CTRL_MODE_1024;
	pac_ctrl.ctrl_chan_dis = 0x1;

	PAC_WRITEREG(PAC_CTRL, pac_ctrl);
	if (pac_i2c_flush() == 0) {
		printf("wr PAC_CTRL fail\n");
		c++;
	}

	pac_accumcfg.accumcfg_acc1 = ACCUMCFG_VSENSE;
	pac_accumcfg.accumcfg_acc2 = ACCUMCFG_VSENSE;
	pac_accumcfg.accumcfg_acc3 = ACCUMCFG_VSENSE;
	pac_accumcfg.accumcfg_acc4 = ACCUMCFG_VSENSE;
	PAC_WRITEREG(PAC_ACCUMCFG, pac_accumcfg);
	if (pac_i2c_flush() == 0) {
		printf("wr PAC_ACCUMCFG fail\n");
		c++;
	}

	pac_neg_pwr_fsr.pwrfsr_vs1 = PAC_PWRSFR_BHALF;
	pac_neg_pwr_fsr.pwrfsr_vs2 = PAC_PWRSFR_BHALF;
	pac_neg_pwr_fsr.pwrfsr_vs3 = PAC_PWRSFR_BHALF;
	pac_neg_pwr_fsr.pwrfsr_vs4 = PAC_PWRSFR_BHALF;

	pac_neg_pwr_fsr.pwrfsr_vb1 = PAC_PWRSFR_UFSR;
	pac_neg_pwr_fsr.pwrfsr_vb2 = PAC_PWRSFR_UFSR;
	pac_neg_pwr_fsr.pwrfsr_vb3 = PAC_PWRSFR_UFSR;
	pac_neg_pwr_fsr.pwrfsr_vb4 = PAC_PWRSFR_UFSR;
	PAC_WRITEREG(PAC_NEG_PWR_FSR, pac_neg_pwr_fsr);
	if (pac_i2c_flush() == 0) {
		printf("wr PAC_NEG_PWR_FSR fail\n");
		c++;
	}

	PAC_WRITECMD(PAC_REFRESH);
	if (pac_i2c_flush() == 0) {
		printf("cmd PAC_REFRESH fail\n");
		c++;
	}
	if (c != 0)
		goto again;

	/* set up some of our input/output */
	PWM_OFF = 1;
	TRISCbits.TRISC1 = 0;

	PWM_MID = 0;
	TRISCbits.TRISC5 = 0;
	PWM_OUT = 0;
	TRISCbits.TRISC6 = 0;

	TRISCbits.TRISC0 = 1; /* PWM_OK input */


	LATBbits.LATB4 = 1; /* BATT1_ON */
	LATBbits.LATB5 = 1; /* BATT2_ON */
	TRISBbits.TRISB4 = 0;
	TRISBbits.TRISB5 = 0;

#if 0

	/*
	 * set up CLC for BATT1/BATT2 outputs:
	 * On only if
	 *   - PWM_OFF == 0
	 *   - PWM_OK == 1
	 *   - PWM_MID == 1
	 *   - the other batt is off
	 *   - we want it on
	 */

	/* CLC inputs - carefull with what port is allowed on which CLCIN */
	CLCIN0PPS = 0x10; /* RC0 PWM_OK */
	CLCIN1PPS = 0x11; /* RC1 PWM_OFF */
	CLCIN2PPS = 0x0c; /* RB4 BATT1_ON */
	CLCIN3PPS = 0x0d; /* RB5 BATT2_ON */
	CLCIN4PPS = 0x15; /* RC5 PWM_MID */

	/* setup CLC3 for BATT1_ON */
	CLCSELECT = 2;
	CLCnCON = 0x02; /* 4 input and */
	CLCnPOL = 0x0d; /* 2 noninverted; 1, 3 & 4 inverted */
	CLCnSEL0 = 0; /* CLCIN0PPS = PWM_OK */
	CLCnSEL1 = 1; /* CLCIN1PPS = PWM_OFF */
	CLCnSEL2 = 3; /* CLCIN3PPS = BATT2_ON */
	CLCnSEL3 = 4; /* CLCIN4PPS = PWM_MID */

	CLCnGLS0 = 0x69; /* b01101001 (!PWM_MID|BATT2_ON|PWM_OFF|!PWM_OK) */
	CLCnGLS1 = 0;
	CLCnGLS2 = 0;
	CLCnGLS3 = 0;
	CLCnCONbits.EN = 1;
	RB4PPS = 0x03; /* BATT1_ON = CLC3OUT */

	/* setup CLC4 for BATT2_ON */
	CLCSELECT = 3;
	CLCnCON = 0x02; /* 4 input and */
	CLCnPOL = 0x0d; /* 2 noninverted; 1, 3 & 4 inverted */
	CLCnSEL0 = 0; /* CLCIN0PPS = PWM_OK */
	CLCnSEL1 = 1; /* CLCIN1PPS = PWM_OFF */
	CLCnSEL2 = 2; /* CLCIN2PPS = BATT1_ON */
	CLCnSEL3 = 4; /* CLCIN4PPS = PWM_MID */

	CLCnGLS0 = 0x69; /* b01101001 (!PWM_MID|BATT1_ON|PWM_OFF|!PWM_OK) */
	CLCnGLS1 = 0;
	CLCnGLS2 = 0;
	CLCnGLS3 = 0;
	CLCnCONbits.EN = 1;
	RB5PPS = 0x04; /* BATT2_ON = CLC4OUT */
#endif

	oled_col = 20;
	oled_line = 5;
	sprintf(oled_displaybuf, "hello");
	displaybuf_medium();
	oled_i2c_flush();

	for (c = 0; c < 4; c++) {
		oled_col = 20 * c;
		oled_line = 1;
		displaybuf_icon(c);
	}
	oled_i2c_flush();

	btn_state = BTN_IDLE;

	FVRCON = 0x8c; /* FVR on, CDAFVR 4.096v */

	CM1CON0 = 0x42; /* invert polarity, hysteresis */
	CM1CON1 = 0x03; /* interrupt on both edges */
	CM1NCH = 0x01; /* CH1IN1- */
	CM1PCH = 0x06; /* FVR */
	CM1CON0bits.EN = 1;
	PIR1bits.C1IF = 0; /* clear any pending interrupt */
	PIE1bits.C1IE = 1; /* enable */

	/* wait 1s, clear display and start operations */
	for (c = 0; c < 100; c++) {
		t0 = timer0_read();
		while (timer0_read() - t0 < (TIMER0_1MS * 10)) {
			CLRWDT();
		}
	}

	/* clear display and put static infos */
	oled_s = oled_i2c_reset();
	memset(oled_s->oled_databuf, 0, OLED_I2C_DATABUFSZ);
	oled_s->oled_datalen = OLED_DISPLAY_SIZE;
	OLED_CTRL(oled_s, 0x00); OLED_CTRL(oled_s, 0x10);  /* reset column start */
	OLED_CTRL(oled_s, 0x21); OLED_CTRL(oled_s, 0); OLED_CTRL(oled_s, 0x7f); /*column start/end */
	OLED_CTRL(oled_s, 0xb0); /* page start */
	oled_s->oled_type = OLED_CTRL_CLEAR;
	oled_i2c_state = OLED_I2C_WAIT;
	oled_i2c_flush();

	oled_col = 0;
	oled_line = 0;
	displaybuf_icon(ICON_SUN);
	oled_i2c_flush();

	oled_col = 0;
	oled_line = 3;
	displaybuf_icon(ICON_LIGHT);
	oled_i2c_flush();

	oled_col = 0;
	oled_line = 6;
	displaybuf_icon(ICON_ENGINE);
	oled_i2c_flush();

	PWM_OFF = 0;
	PWM_MID = 1;
	PWM_OUT = 0;

	printf("enter loop\n");

	while (1) {
		CLRWDT();
		if (C1INTLbits.RXIF) {
			nmea2000_receive();
		}
		if (NCANOK) {
			nmea2000_status = NMEA2000_S_ABORT;
			poll_count = timer0_read();;
		} else if (nmea2000_status != NMEA2000_S_OK) {
			uint16_t ticks, tmrv;

			tmrv = timer0_read();
			ticks = tmrv - poll_count;
			if (ticks > TIMER0_5MS) {
				poll_count = tmrv;
				nmea2000_poll(
				    (u_char)(ticks / TIMER0_1MS));
			}
			if (nmea2000_status == NMEA2000_S_OK) {
				printf("new addr %d\n", nmea2000_addr);
			}
		}

#ifdef USEADC
		if (PIR1bits.ADIF) {
			PIR1bits.ADIF = 0;
			a2d_acc = ((u_int)ADRESH << 8) | ADRESL;
			switch (ADPCH) {
			case 0:
				/* channel 0: NTC */
				adctotemp(0);
				ADCON0bits.ADON = 0;
				ADPCH = 1;
				ADCON0bits.ADON = 1;
				break;
			case 1:
				/* channel 1: NTC */
				adctotemp(1);
				ADCON0bits.ADON = 0;
				ADPCH = 4;
				ADCON0bits.ADON = 1;
				break;
			case 4:
				/* channel 4: NTC */
				adctotemp(2);
				ADCON0bits.ADON = 0;
				ADPCH = 0;
				/* don't set ADON, will do on next second */
				break;
			default:
				printf("unknown channel 0x%x\n", ADCON0);
			}
		}
#endif

		if (softintrs.bits.int_btn_down) {
			softintrs.bits.int_btn_down = 0;
			if (btn_state == BTN_IDLE) {
				btn_state = BTN_DOWN;
				/* get btn value */
				/* XXX context */
				ADCTX = 3;
				PIR1bits.ADIF = 0;
				PIE1bits.ADIE = 1;
				ADCON0bits.ADON = 1;
			}
		}
		if (softintrs.bits.int_btn_up) {
			switch(btn_state) {
			case BTN_DOWN_1_P:
			case BTN_DOWN_2_P:
				softintrs.bits.int_btn_up = 0;
				/* this is a up event */
				btn_state = BTN_UP;
				break;
			BTN_DOWN:
			case BTN_DOWN_1:
			case BTN_DOWN_2:
				/* not ready for an UP event yet, wait */
				break;
			default:
				/* transient event */
				softintrs.bits.int_btn_up = 0;
				btn_state = BTN_IDLE;
				break;
			}
		}
		if (softintrs.bits.int_adcc) {
			if (btn_state == BTN_DOWN) {
				if (ADRES > 0x600 && ADRES < 0x900) {
					btn_state = BTN_DOWN_2;
				} else if (ADRES > 0x400 && ADRES < 0x600) {
					btn_state = BTN_DOWN_1;
				} else {
					/* transient event */
					btn_state = BTN_IDLE;
				}
			} else {
				btn_state = BTN_IDLE;
			}
			// printf("ADC complete 0x%x\n", ADRES);
			softintrs.bits.int_adcc = 0;
			PIR1bits.ADIF = 0;
			PIE1bits.ADIE = 0;
		}

		switch (btn_state) {
		case BTN_DOWN_1:
			oled_col = 60;
			oled_line = 4;
			sprintf(oled_displaybuf, "  B1 ");
			displaybuf_small();
			printf("B1 B 0x%x C 0x%x CLC 0x%x\n", LATB, PORTB, TRISB);
			btn_state = BTN_DOWN_1_P;
			LATBbits.LATB4 = 1;
			batt1_en(1);
			break;
		case BTN_DOWN_2:
			oled_col = 60;
			oled_line = 4;
			sprintf(oled_displaybuf, "  B2 ");
			displaybuf_small();
			printf("B2 B 0x%x C 0x%x CLC 0x%x\n", PORTB, ODCONB, TRISB);
			btn_state = BTN_DOWN_2_P;
			batt2_en(1);
			break;
		case BTN_UP:
			oled_col = 60;
			oled_line = 4;
			sprintf(oled_displaybuf, "  up ");
			displaybuf_small();
			printf("up B 0x%x C 0x%x CLC 0x%x\n", PORTB, PORTC, CLCDATA);
			btn_state = BTN_IDLE;
			LATBbits.LATB4 = 0;
			batt1_en(0);
			batt2_en(0);
			break;
		default:
			break;
		}

		if (softintrs.bits.int_10hz) {
			softintrs.bits.int_10hz = 0;
			counter_1hz--;
			if (counter_1hz == 2) {
				double amps;
				/* update display every seconds */
				/* solar */
				oled_col = 20;
				oled_line = 0;
				dislplay_battstat(2);
				/* batt1 (service) */
				oled_col = 20;
				oled_line = 3;
				dislplay_battstat(1);
				/* batt2 (engine) */
				oled_col = 20;
				oled_line = 6;
				dislplay_battstat(0);
			}

			if (counter_1hz == 1)
				pacops_pending.bits.refresh = 1;
			else if (counter_1hz & 1)
				pacops_pending.bits.refresh_v = 1;
			else
				pacops_pending.bits.read_values = 1;

			if (counter_1hz == 0) {
				counter_1hz = 10;
				pacops_pending.bits.read_accum = 1;
				seconds++;
				if (seconds == 600) {
					// USELOG update_log();
					seconds = 0;
				}
				// USEADC  ADCON0bits.ADON = 1; /* start a new cycle */
			}
			if (ADCON0bits.ADON)
				ADCON0bits.GO = 1;
			if (!NCANOK && nmea2000_status == NMEA2000_S_OK) {
				uint16_t ticks, tmrv;

				tmrv = timer0_read();
				ticks = tmrv - poll_count;
				if (ticks > TIMER0_5MS) {
					poll_count = tmrv;
					nmea2000_poll(
					   (u_char)(ticks / TIMER0_1MS));
				}
				if (nmea2000_status != NMEA2000_S_OK) {
					printf("lost CAN bus %d\n",
					    ticks);
				}
			}
		}
		if (softintrs.bits.int_pacacc) {
			int64_t acc_value;
			double v;
			char *acc_bytes = (void *)&acc_value;
			softintrs.bits.int_pacacc = 0;
			// printf("acc %ld", _read_accum.acc_count.acccnt_count);
			l600_current_count += _read_accum.acc_count.acccnt_count;
			for (c = 0; c < 3; c++) {
				acc_bytes[7] = 0;
				for (char b = 0; b < 7; b++) {
					acc_bytes[b] =
					    _read_accum.accum[c * 7 + b];
				}
				if (acc_value & 0x0080000000000000) {
					/* adjust negative value */
					acc_value |= 0xff00000000000000;
				}
				l600_current_acc[2 - c] += acc_value;
				/* batt_i = acc_value * 0.00075 * 100 */
				v = (double)acc_value * 0.075 / _read_accum.acc_count.acccnt_count;
		                // printf(" %4.4fA", v / 100);

			}
			// printf("\n");
				
			// printf("avg");
			for (c = 0; c < 3; c++) {
				/* i = acc_value * 0.00075 */
				/* batt_i = acc_value * 0.00075 * 100 */
				v = (double)_read_voltcur.batt_i[c] * 0.075;
				batt_i[2 - c] = (int16_t)(v + 0.5);
				// printf(" %4.4fA", v / 100);
				/* volt = vbus * 0.000488 */
				/* batt_v = vbus * 0.000488 * 100 */      
				v = (double)_read_voltcur.batt_v[c] * 0.0488;
				batt_v[2 - c] = (uint16_t)(v + 0.5);
				// printf(" %4.4fV", v / 100);
			}
			// printf("\n");
		}
		if (softintrs.bits.int_pacavg) {
			softintrs.bits.int_pacavg = 0;
		}
		pac_command_schedule();
		while (i2c_return != I2C_INPROGRESS) {
			if (pac_i2c_state < PAC_I2C_WAIT &&
			    oled_i2c_state < OLED_I2C_WAIT) {
				break;
			}

			/*
			 * pac has higher priority
			 */
			 switch(oled_i2c_state) {
			 case OLED_I2C_CMD:
			 case OLED_I2C_DATA:
				oled_i2c_exec();
				break;
			 case OLED_I2C_WAIT:
				if (pac_i2c_state < PAC_I2C_WAIT)
					oled_i2c_exec();
				break;
			default:
				break;
			}
			if (oled_i2c_state == OLED_I2C_ERROR) {
				if (!oled_error)
					printf("oled i2c error\n"); // XXX
				oled_error = 1;
				oled_i2c_state = OLED_I2C_IDLE;
				oled_i2c_buf[oled_i2c_cons].oled_type = OLED_CTRL_FREE;
			}
			 switch(pac_i2c_state) {
			 case PAC_I2C_READ:
			 case PAC_I2C_WRITE:
				pac_i2c_exec();
				break;
			 case PAC_I2C_WAIT:
				if (oled_i2c_state <= OLED_I2C_WAIT)
					pac_i2c_exec();
				break;
			default:
				break;
			}
			if (pac_i2c_state == PAC_I2C_ERROR) {
				printf("pac i2c error\n"); // XXX
				pac_i2c_state = PAC_I2C_IDLE;
				pac_i2c_io.pac_type = PAC_FREE;
			} else if (pac_i2c_state == PAC_I2C_COMPLETE) {
				pac_command_complete();
				pac_command_schedule();
			}
		}

		if (PIR4bits.U1RXIF && (U1RXB == 'r'))
			break;
		if (softintrs.byte == 0)
			SLEEP();
		if (default_src != 0) {
			printf("default handler called for 0x%x\n",
			    default_src);
		}
	}
	while ((c = (char)getchar()) != 'r') {
		printf("resumed %u\n", timer0_read());
		goto again;
	}
	WDTCON0bits.SEN = 0;
	printf("returning\n");
	while (!PIR4bits.U1TXIF) {
		; /* wait */ 
	}
	RESET();
	return 0;
}

u_int
timer0_read(void)
{
	u_int value;

	/* timer0_value = TMR0L | (TMR0H << 8), reading TMR0L first */
	di();
	asm("movff TMR0L, timer0_read@value");
	asm("movff TMR0H, timer0_read@value+1");
	ei();
	return value;
}

#ifdef USE_TIMER2
void __interrupt(__irq(TMR2), __high_priority, base(IVECT_BASE))
irqh_timer2(void)
{
	PIR3bits.TMR2IF = 0;
	if (--counter_10hz == 0) {
		counter_10hz = 25;
		softintrs.bits.int_10hz = 1;
	}			
}
#else
void __interrupt(__irq(TU16A), __high_priority, base(IVECT_BASE))
irqh_tu16a(void)
{
	TU16ACON1bits.CLR = 1;
	TU16ACON1bits.PRIF = 0;
	softintrs.bits.int_10hz = 1;
}
#endif

void __interrupt(__irq(CM1), __low_priority, base(IVECT_BASE))
irqh_cm1(void)
{
	if (CM1CON0bits.OUT)
		softintrs.bits.int_btn_down = 1;
	else 
		softintrs.bits.int_btn_up = 1;
	PIR1bits.C1IF = 0;
}

void __interrupt(__irq(AD), __low_priority, base(IVECT_BASE))
irqh_adcc(void)
{
	PIE1bits.ADIE = 0;
	softintrs.bits.int_adcc = 1;
}

void __interrupt(__irq(default), __low_priority, base(IVECT_BASE))
irqh_default(void)
{
	default_src = 0xff;
}

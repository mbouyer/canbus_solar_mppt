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
#include "vers.h"

typedef unsigned char u_char;
typedef unsigned int  u_int;
typedef unsigned long u_long;

static u_char default_src;

u_int devid, revid; 

u_long nmea2000_user_id; 

uint8_t pac_pid, pac_mid, pac_rid;

static u_char sid;

static struct nmea2000_msg msg;
static u_char nmea2000_data[NMEA2000_DATA_FASTLENGTH];

u_int timer0_read(void);

#define TIMER0_1MS    10
#define TIMER0_5MS    49
#define TIMER0_20MS   195
#define TIMER0_100MS  977

#define NCANOK  PORTCbits.RC2

#define PWM_OFF   LATCbits.LATC1
#define PWM_OK    PORTCbits.RC0
#define PWM_MID   LATCbits.LATC5 /* high-side resistor for 3-state PWM output */
#define PWM_OUT   LATCbits.LATC6 /* PWM output */
#define PAC_NDOWN LATCbits.LATC7

#define OLED_RSTN LATAbits.LATA4

typedef enum {
	BATT_NONE = 0,
	BATT_1 = 1, /* engine */
	BATT_2 = 2, /* service */
} batt_t;

static void
batt_en(batt_t b)
{
	switch(b) {
	case BATT_NONE:
		CLCSELECT = 2;
		CLCnPOLbits.G2POL = 0;
		CLCSELECT = 3;
		CLCnPOLbits.G2POL = 0;
		break;
	case BATT_1:
		CLCSELECT = 2;
		CLCnPOLbits.G2POL = 1;
		CLCSELECT = 3;
		CLCnPOLbits.G2POL = 0;
		break;
	case BATT_2:
		CLCSELECT = 2;
		CLCnPOLbits.G2POL = 0;
		CLCSELECT = 3;
		CLCnPOLbits.G2POL = 1;
		break;
	}
}

static batt_t active_batt;

static char counter_100hz;
static char counter_100hz;
static char counter_10hz;
static uint16_t seconds;
static volatile union softintrs {
	struct softintrs_bits {
		char int_100hz : 1;	/* 0.01s timer */
		char int_btn_down : 1;	/* button release->press */
		char int_btn_up : 1;	/* button press->release */
		char int_adcc : 1;	/* A/D convertion complete */
	} bits;
	char byte;
} softintrs;

static union time_events {
	struct tmev_bits {
		char ev_100hz : 1;	/* 0.01s timer */
		char ev_10hz : 1;	/* 0.1s timer */
		char ev_1hz : 1;	/* 1s timer */
	} bits;
	char byte;
} time_events;

static uint16_t ad_pwm, target_ad_pwm;
static uint16_t ad_solar;
static uint16_t ad_temp;

static uint16_t board_temp;

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

/*
 * buffer to read voltage/current values at once 
 * as we're using readreg_be, the register order is inverted
 * Disabled channels are skipped, so only 3 entries
 * index:
 * _read_voltcur.batt_i[0] = SOLAR = batt_i[2]
 * _read_voltcur.batt_i[1] = BATT_2 = -batt_i[1]
 * _read_voltcur.batt_i[2] = BATT_1 = -batt_i[0]
 *
 * _read_voltcur.batt_v[0] = SOLAR = batt_v[2]
 * _read_voltcur.batt_v[1] = BATT_2 = batt_v[1]
 * _read_voltcur.batt_v[2] = BATT_1 = batt_v[0]
 */
static struct {
	int16_t batt_i[3];
	uint16_t batt_v[3];
} _read_voltcur;

/* computed amps and volt values */
uint16_t batt_v[3];
int16_t batt_i[3];

/*
 * Sliding average values for display.
 * because we switch batteries about once per second, but it may be
 * delayed so take some margin.
 * 100 values per second; we average 4 values and keep 64 averaged values
 * This gives us 256 values, which should be more than enough.
 */
uint16_t batt_v_a[3]; /* 3200 * 4 = 12800, it fits */
int16_t  batt_i_a[3]; /* 1000 * 4 = 4000, it fits */
uint8_t batt_a_count;
#define BATT_A_NCOUNT 4

#define BATT_S_NCOUNT 64
uint8_t batt_s_idx;
uint8_t batt_s_size; /* number of samples in last cycle */
uint8_t batt_s_count; /* samples counter for current cycle */
uint16_t batt_v_s[BATT_S_NCOUNT][3];
int16_t  batt_i_s[BATT_S_NCOUNT][3];
/* keep around display values */
uint16_t batt_v_d[3];
int16_t  batt_i_d[3];

static uint32_t voltages_acc[3];
pac_ctrl_t pac_ctrl;

static union pac_events {
	struct pacevts_bits {
		char pacavg_rdy : 1;	/* _read_voltcur updated */
		char pacavg_rdy_chrg : 1;/* _read_voltcur updated for chrg */
		char pacacc_rdy : 1;	/* _read_accum updated */
		char bvalues_updated : 1; /* batt_i[] & batt_v[] updated */
	} bits;
	char byte;
} pac_events;

/*
 * PWM fsm:
 * DOWN: everything off (PWM_OFF = 1, PWM_MID = PWM_OUT = 0), wait for power
	 up order
 * TURNON: PWM_OFF = 1, PWM_MID = PWM_OUT = 0
 * ON: PWM_OFF = 0, waiting for PWM_OK
 * OK: PWM_OK asserted, turn on PWM_MID, set up PWM registers and 
 *     connect PWM_OUT to PWM
 * IDLE: hardware ready, waiting for PWM values
 * RUNNING: providing PWM signal
 * GOIDLE: prepare for off: PWM_OUT = 0
 * BATTOFF: prepare for off: BATT1_OFF, bATT2_OFF
 * DISCHARGE: prepare for off: PWM_OUT = 0, PWM_MID = 0,
 */
static enum {
	PWMF_DOWN = 0,
	PWMF_TURNON,
	PWMF_ON,
	PWMF_OK,
	PWMF_IDLE,
	PWMF_RUNNING,
	PWMF_GOIDLE,
	PWMF_BATTOFF,
	PWMF_DISCHARGE,
} pwm_fsm;

static volatile union pwm_events {
	struct pwm_evtbits {
		char goon : 1;	/* turn PWM on */
		char gooff : 1; /* turn PWM off */
	} bits;
	char byte;
} pwm_events;

static enum {
	PWME_NOERROR = 0,
	PWME_PWMV,
	PWME_PWMOK,
	PWME_OVERTEMP,
	PWME_BATTCUR,
} pwm_error;

#define TEMP_MAX ((uint16_t)27315 + 7000) /* 70 deg celius */
#define TEMP_RESTART ((uint16_t)TEMP_MAX - 1000) /* 10 deg below TEMP_MAX */

uint16_t pwm_time;
uint8_t pwme_time;

uint8_t pwm_duty_c; /* duty cycle in % * 2 */
#define PWM_DUTY_MAX 198

static inline void
pwmf_goidle()
{
	pwm_events.bits.gooff = 0;
	PWM1CONbits.EN = 0;
	CM1CON0bits.EN = 0;
	PIR1bits.C1IF = 0;
	PIE1bits.C1IE = 0;
	PIR0bits.IOCIF = 0;
	IOCCFbits.IOCCF0 = 0;
	PIE0bits.IOCIE = 0;
	pwm_time = timer0_read();
	pwm_fsm = PWMF_GOIDLE;
}

static inline void
pwm_runfsm()
{
	switch(pwm_fsm) {
	case PWMF_DOWN:
		PWM_OFF = 1;
		PWM_MID = 0;
		PWM_OUT = 0;
		if (pwm_error != PWME_NOERROR)
			return;
		if (pwm_events.bits.goon) {
			pwm_events.bits.goon = 0;
			pwm_time = timer0_read();
			pwm_fsm = PWMF_TURNON;
			CM1CON0bits.EN = 1; /* need to turn on early */
		}
		break;
	case PWMF_TURNON:
		if ((timer0_read() - pwm_time) > TIMER0_100MS) {
			PWM_OFF = 0;
			pwm_time = timer0_read();
			pwm_fsm = PWMF_ON;
		}
		break;
	case PWMF_ON:
		if (PWM_OK) {
			pwm_fsm = PWMF_OK;
		}
		if ((timer0_read() - pwm_time) > TIMER0_100MS) {
			/* timeout, retry */
			printf("PWM_OK tout\n");
			PWM_OFF = 1;
			pwm_time = timer0_read();
			pwm_fsm = PWMF_TURNON;
		}
		break;
	case PWMF_OK:
		PWM_MID = 1;
		PWM1PR = 512; /* pwm at 125Khz */
		PWM1S1P1 = 0; /* default to off */
		PWM1CONbits.EN = 1;
		PIR1bits.C1IF = 0;
		PIE1bits.C1IE = 1;
		PIR0bits.IOCIF = 0;
		IOCCFbits.IOCCF0 = 0;
		PIE0bits.IOCIE = 1;
		pwm_fsm = PWMF_IDLE;
		break;
	case PWMF_IDLE:
		if (pwm_events.bits.gooff) {
			pwmf_goidle();
		}
		break;
	case PWMF_RUNNING:
		if (pwm_events.bits.gooff) {
			pwmf_goidle();
		}
		break;
	case PWMF_GOIDLE:
		if ((timer0_read() - pwm_time) > TIMER0_20MS) {
			printf("PWM OFF CON 0x%x PR 0x%x P1 0x%x B 0x%x C 0x%x\n", 
			    PWM1CON,
			    PWM1PR,
			    PWM1S1P1,
			    PORTB,
			    PORTC);
			batt_en(0);
			pwm_time = timer0_read();
			pwm_fsm = PWMF_BATTOFF;
		}
		break;
	case PWMF_BATTOFF:
		if ((timer0_read() - pwm_time) > TIMER0_5MS) {
			PWM_MID = 0;
			pwm_time = timer0_read();
			pwm_fsm = PWMF_DISCHARGE;
		}
		break;
	case PWMF_DISCHARGE:
		if ((timer0_read() - pwm_time) > TIMER0_20MS) {
			PWM_OFF = 1;
			pwm_fsm = PWMF_DOWN;
		}
		break;
	}
}

static void
pwm_set_duty()
{
	uint32_t v;
	/*
	 * off time needs to be at last 500ns (the NCP5901B's ZCD blanking
	 * timer is 250ns
	 */
#define PWM_MIN_OFF 32

	/* experiments show we can't go above 98% */
	if (pwm_duty_c > 240) /* assume overflow */
		pwm_duty_c = 0;
	else if (pwm_duty_c > 198)
		pwm_duty_c = 198;

	/* first try running at 125Khz */
	v = (uint32_t)512 * (uint32_t)pwm_duty_c / 200;
	// printf("v 0x%lx ", v);
	if (512 - v >= PWM_MIN_OFF) {
		PWM1PR = 512;
		PWM1S1P1 = (uint16_t)v;
	} else {
		/* duty cycle too large, decrease PWM frequency */
		/* PR = PWM_MIN_OFF / (1 - pwm_duty_c / 200) */
		/* PR = PWM_MIN_OFF * 200 / (200 - pwm_duty_c) */
		PWM1PR = (PWM_MIN_OFF * 200) / (200 - pwm_duty_c);
		PWM1S1P1 = PWM1PR - PWM_MIN_OFF;
	}
	// printf("PWM1PR 0x%x S1P1 0x%x\n", PWM1PR, PWM1S1P1);
	PWM1CONbits.LD = 1;
}

typedef enum {
	CHRG_DOWN = 0,
	CHRG_PWMUP,
	CHRG_PRECHARGE,
	CHRG_RERAMPUP,
	CHRG_RAMPUP,
	CHRG_MPPT,
	CHRG_CV,
	CHRG_BSWITCH,
	CHRG_BSWITCH_WAIT,
	CHRG_GODOWN
} chrg_fsm_t;

static chrg_fsm_t chrg_fsm;

static volatile union chrg_events {
	struct chrg_evtbits {
		char goon : 1;	/* turn charger on */
		char gooff : 1; /* turn charger off */
		char battswitch : 1; /* cause a battery switch */
	} bits;
	char byte;
} chrg_events;

static uint16_t chrg_previous_current;
static uint16_t chrg_current_accum;
static uint8_t chrg_accum_cnt;
static int8_t chrg_duty_change;
static uint8_t chrg_volt_target_cnt;
static uint8_t chrg_batt_grace; /* grace cycles after batt switch in CV mode */

struct chrg_param {
	uint8_t chrgp_pwm; /* pwm value */
	int16_t chrgp_iout; /* intensity output */
};

typedef enum {
	BATTS_NONE = 0, /* uninitialised */
	BATTS_BULK, /* charge at max current or bulk voltage */
	BATTS_FLOAT, /* charge at float voltage */
	BATTS_STANDBY, /* do not charge */
	BATTS_ERR, /* error state */
} batt_state_t;

typedef struct {
	uint16_t bp_bulk_voltage_limit;
	uint16_t bp_bulk_voltage; 
	uint16_t bp_float_voltage;
} batt_params_t;

static batt_params_t bparams[2];

typedef struct {
	batt_state_t bc_stat;
	uint16_t bc_cv; /* target voltage */
	chrg_fsm_t bc_chrg_fsm; /* chrg state to resume at */
	struct chrg_param bc_chrg; /* current chrg state */
	struct chrg_param bc_r_chrg; /* best chrg state during rampup */
	u_int bc_sw_time; /* timer0() at last on switch */
	u_int bc_sw_duration; /* number of ticks it should be on */
	u_int bc_rp_time; /* number of seconds in MPPT state */
} batt_context_t;

#define MPPT_RAMPUP_PERIOD 600 /* redo a ramup every 10mn */

static batt_context_t battctx[2];
#define active_bidx (active_batt - BATT_1)
#define active_battctx (battctx[active_bidx])
#define active_battv (batt_v[active_bidx])
#define active_batti (batt_i[active_bidx])

#define BATT_MINV 80 /* if battery below 8V don't start chrg */

static void
check_batt_status()
{
	for (char c = 0; c < 2; c++) {
		uint8_t _v = (uint8_t)(batt_v[c] / 10);
		if (_v <= BATT_MINV) {
			battctx[c].bc_stat = BATTS_NONE;
			continue;
		}
		if (_v > bparams[c].bp_bulk_voltage_limit) {
			if (battctx[c].bc_stat == BATTS_NONE) {
				battctx[c].bc_stat = BATTS_FLOAT;
				battctx[c].bc_cv = bparams[c].bp_float_voltage;
				battctx[c].bc_sw_time = timer0_read();
				battctx[c].bc_chrg_fsm = CHRG_RERAMPUP;
			}
		} else {
			if (battctx[c].bc_stat == BATTS_NONE ||
			    battctx[c].bc_stat == BATTS_STANDBY) {
				battctx[c].bc_stat = BATTS_BULK;
				battctx[c].bc_cv = bparams[c].bp_bulk_voltage;
				battctx[c].bc_sw_time = timer0_read();
				battctx[c].bc_chrg_fsm = CHRG_RERAMPUP;
			}
		}
	}
}

static char
check_batt_active(char b)
{
	if (battctx[b].bc_stat == BATTS_BULK ||
	    battctx[b].bc_stat == BATTS_FLOAT)
		return 1;
	else
		return 0;
}

static char
batt_switch_active()
{
	switch(active_batt) {
	case BATT_1:
		if (check_batt_active(BATT_2 - BATT_1)) {
			active_batt = BATT_2;
			return 1;
		}
		break;
	case BATT_2:
		if (check_batt_active(BATT_1 - BATT_1)) {
			active_batt = BATT_1;
			return 1;
		}
		break;
	default:
		break;
	}
	return 0;
}

static void
schedule_batt_switch()
{
	/*
	 * check if a battery switch is needed:
	 * first see if a switch is due
	 */
	if (chrg_events.bits.battswitch)
		return; /* already pending */

	if (active_battctx.bc_sw_duration != 0 &&
	    timer0_read() - active_battctx.bc_sw_time <
	    active_battctx.bc_sw_duration)
		return; /* not yet */

	/* see if the other battery is also scheduled - and switch now if so */
	switch (active_batt) {
	case BATT_1:
		if (battctx[BATT_2 - BATT_1].bc_sw_duration != 0) {
			active_battctx.bc_sw_duration = 0;
			chrg_events.bits.battswitch = 1;
			return;
		}
		break;
	case BATT_2:
		if (battctx[BATT_1 - BATT_1].bc_sw_duration != 0) {
			active_battctx.bc_sw_duration = 0;
			chrg_events.bits.battswitch = 1;
			return;
		}
		break;
	default:
		break;
	}
	
	/* starting a  new cycle */
	/*
	 * compute weight  of each battery: if one is cc and one is cv,
	 * give 90/10% of time.
	 * if both are in the same state, give 50/50%
	 * One time slot is 1s, we compute the number of 1/10s allocated
	 * to active battery.
	 */
	uint8_t bw[2], sum, c;
	uint16_t time;
	sum = 0;
	for (c = 0; c < 2; c++) {
		bw[c] = 0;
		switch(battctx[c].bc_stat) {
		case BATTS_BULK:
		case BATTS_FLOAT:
			switch(battctx[c].bc_chrg_fsm) {
			case CHRG_MPPT:
			case CHRG_RERAMPUP:
			case CHRG_RAMPUP:
				bw[c] = 90;
				break;
			case CHRG_CV:
				/* don't schedule if battery already too high*/
				if (battctx[c].bc_cv <= batt_v[c] + 1 ||
				    c == active_bidx)
					bw[c] = 10;
				break;
			default:
				break;
			}
		default:
			break;
		}
		sum += bw[c];
	}
	if (sum == 0) {
		/* nothing active, turn off if needed */
		if (chrg_fsm != CHRG_DOWN)
			chrg_events.bits.gooff = 1;
		return;
	}

	sum = sum / 10;
	for (c = 0; c < 2; c++) {
		time = bw[c] / sum;
		battctx[c].bc_sw_duration = time * TIMER0_100MS;
	}

	if (chrg_fsm == CHRG_DOWN) {
		/*
		 * ready to start ? needs solar higher than batteries by 1V,
		 * if batteries are present
		 */
		chrg_events.bits.goon = 1;
		if (check_batt_active(0) && batt_v[2] < batt_v[0] + 100)
			chrg_events.bits.goon = 0;
		if (check_batt_active(1) && batt_v[2] < batt_v[1] + 100)
			chrg_events.bits.goon = 0;
		return;
	}

	if (time != 0 && time < 10) {
		/* both batteries are active, need to switch */
		chrg_events.bits.battswitch = 1;
	} else {
		/* don't switch but update time */
		active_battctx.bc_sw_time = timer0_read();
	}
	/* update stats for display */
	batt_s_size = batt_s_count;
	batt_s_count = 0;
}

uint8_t _mppt_debug;

static void
chrg_mppt_compute()
{
#if 0
	char c;
	c = (chrg_current_accum >> 12) & 0x0f;
	if (c > 9)
		usart_putchar('A' - 10 + c);
	else
		usart_putchar('0' + c);
	c = (chrg_current_accum >> 8) & 0x0f;
	if (c > 9)
		usart_putchar('A' - 10 + c);
	else
		usart_putchar('0' + c);
	c = (chrg_current_accum >> 4) & 0x0f;
	if (c > 9)
		usart_putchar('A' - 10 + c);
	else
		usart_putchar('0' + c);
	c = (chrg_current_accum >> 0) & 0x0f;
	if (c > 9)
		usart_putchar('A' - 10 + c);
	else
		usart_putchar('0' + c);
	usart_putchar('_');
	c = (pwm_duty_c >> 4) & 0x0f;
	if (c > 9)
		usart_putchar('A' - 10 + c);
	else
		usart_putchar('0' + c);
	c = (pwm_duty_c >> 0) & 0x0f;
	if (c > 9)
		usart_putchar('A' - 10 + c);
	else
		usart_putchar('0' + c);
	usart_putchar(' ');
#endif
	if (chrg_previous_current > chrg_current_accum){
		/* wrong move */
		if (chrg_duty_change < 0)
			chrg_duty_change = 8;
		else
			chrg_duty_change = -8;
	} else {
		/*
		 * right move; try a bit faster
		 *  but only every 8 update cycle
		 * in the right direction
		 */
		if (chrg_duty_change < 0) {
			chrg_duty_change += -1;
			if (chrg_duty_change < -64)
				chrg_duty_change = -64;
		} else {
			chrg_duty_change += 1;
			if (chrg_duty_change > 64)
				chrg_duty_change = 64;
		}
	}

	chrg_previous_current = chrg_current_accum;
	pwm_duty_c += (chrg_duty_change >> 3);
	pwm_set_duty();
	chrg_accum_cnt = 4;
	chrg_current_accum = 0;

	if (active_battctx.bc_rp_time >= MPPT_RAMPUP_PERIOD - 4) {
		/* record values after they have stabilized */
		active_battctx.bc_r_chrg.chrgp_pwm = pwm_duty_c;
		active_battctx.bc_r_chrg.chrgp_iout =
		    -_read_voltcur.batt_i[2 - active_bidx];
	} else if (timer0_read() - active_battctx.bc_sw_time > TIMER0_5MS * 4) {
		/* check if we need to re-do a rampup */
		if (active_battctx.bc_rp_time == 0) {
			printf("reramp time\n");
			chrg_fsm = CHRG_RERAMPUP;
		}
		int16_t dtdiff =
		    ((int16_t)pwm_duty_c - active_battctx.bc_r_chrg.chrgp_pwm);
		if (abs(dtdiff) > 20) { /* 10% move */
			printf("reramp duty %d\n", dtdiff);
			chrg_fsm = CHRG_RERAMPUP;
		}
		/* cdiff = active_battctx.bc_r_chrg.chrgp_iout - battcur */
		int32_t cdiff =
		    (active_battctx.bc_r_chrg.chrgp_iout + _read_voltcur.batt_i[2 - active_bidx]);
		cdiff = cdiff * 100 / active_battctx.bc_r_chrg.chrgp_iout;
		if (abs((int)cdiff) > 10) { /* 10% move */
			printf("reramp I %d (%x %x)\n",
			    (int)cdiff,
			    active_battctx.bc_r_chrg.chrgp_iout,
			    -_read_voltcur.batt_i[2 - active_bidx]);
			chrg_fsm = CHRG_RERAMPUP;
		}

		if ((_mppt_debug++ % 128) == 0) {
			printf("bc_r_i %x curr %x df %d\n",
			    active_battctx.bc_r_chrg.chrgp_iout,
			    -_read_voltcur.batt_i[2 - active_bidx],
			    (int)cdiff);
		}
	}
}

static void
chrg_runfsm()
{
	if (pwm_error != 0)
		chrg_events.bits.gooff = 1;
	switch(chrg_fsm) {
	case CHRG_DOWN:
		chrg_events.bits.battswitch = 0;
		battctx[0].bc_sw_duration = 0;
		battctx[1].bc_sw_duration = 0;
		batt_s_count = batt_s_size = 0;
		if (chrg_events.bits.goon) {
			chrg_events.bits.goon = 0;
			chrg_events.bits.gooff = 0;
			pwm_events.bits.gooff = 0;
			pwm_events.bits.goon = 1;
			chrg_fsm = CHRG_PWMUP;
		}
		break;

	case CHRG_PWMUP:
		if (chrg_events.bits.gooff) {
			chrg_fsm = CHRG_GODOWN;
		} else if (pwm_fsm == PWMF_IDLE && active_batt != BATT_NONE &&
		    pac_events.bits.bvalues_updated) {
			pac_events.bits.bvalues_updated = 0;
			if (active_battctx.bc_stat == BATTS_BULK ||
			    active_battctx.bc_stat == BATTS_FLOAT) {
				/*
				 * Vad = (active_battv / 100) / (157 / 22 + 1)
				 * Vad = active_battv / 813.63636
				 * Vad = ad_pwm / 4096 * 2.048
				 * ad_pwm = Vad * 2000
				 * ad_pwm = active_battv / 813.63636 * 2000
				 */

				target_ad_pwm = (uint16_t)(
				    (uint32_t)active_battv * 2000 / 814);
				pwm_fsm = PWMF_RUNNING;
				pwm_duty_c = 40; /* start at 20% */
				pwm_set_duty();
				printf("PWM RUN CON 0x%x PR 0x%x P1 0x%x act %d tgt %x\n", 
				    PWM1CON,
				    PWM1PR,
				    PWM1S1P1,
				    active_batt,
				    target_ad_pwm);
				/* abuse bc_sw_time for precharge timing */
				active_battctx.bc_sw_time = timer0_read();
				chrg_fsm = CHRG_PRECHARGE;
				pac_events.bits.pacavg_rdy_chrg = 0;
			} else {
				if (active_batt == BATT_1)
					active_batt = BATT_2;
				else
					active_batt = BATT_1;
			}
		}
		break;

	case CHRG_PRECHARGE:
		/*
		 * output capacitor precharge: slowly increase pwm_duty_c
		 * until we reach target voltage.
		 * spice shows that it takes 150µs at 80% duty to reach 12.5V
		 * with max solar power
		 */
		if (chrg_events.bits.gooff) {
			chrg_fsm = CHRG_GODOWN;
		} else {
			if (ad_pwm >= target_ad_pwm) {
				printf("pre ad %x duty %d\n", ad_pwm, pwm_duty_c);
				batt_en(active_batt);
				active_battctx.bc_sw_time = timer0_read();
				pwm_duty_c = 0;
				pwm_set_duty();
				chrg_fsm = CHRG_RERAMPUP;
				pac_events.bits.bvalues_updated = 0;
			} else if ((timer0_read() - active_battctx.bc_sw_time) >
			    TIMER0_1MS) {
				pwm_duty_c += 5;
				if (pwm_duty_c > PWM_DUTY_MAX)
					pwm_duty_c = PWM_DUTY_MAX;
				pwm_set_duty();
			}
		}
		break;

	case CHRG_RERAMPUP:
		if (chrg_events.bits.gooff) {
			chrg_fsm = CHRG_GODOWN;
		} else if (pac_events.bits.bvalues_updated) {
			pac_events.bits.bvalues_updated = 0;
			pwm_duty_c = 40; /* start at 20% */
			pwm_set_duty();
			active_battctx.bc_r_chrg.chrgp_iout = -1;
			chrg_fsm = CHRG_RAMPUP;
		}
		break;

	case CHRG_RAMPUP:
		if (pac_events.bits.bvalues_updated) {
			pac_events.bits.bvalues_updated = 0;
			if ((active_battv / 10) > active_battctx.bc_cv) {
				/* if we're at target voltage go to CV mode */
				chrg_volt_target_cnt = 0;
				chrg_fsm = CHRG_CV;
				chrg_batt_grace = 0;
			}
		}
		if (pac_events.bits.pacavg_rdy_chrg &&
		    chrg_fsm == CHRG_RAMPUP) {
			char i;

			/* battery current is inverted */
			int16_t battcur =
			    -_read_voltcur.batt_i[2 - active_bidx];
			if (active_battctx.bc_r_chrg.chrgp_iout < battcur) {
				active_battctx.bc_r_chrg.chrgp_iout = battcur;
				active_battctx.bc_r_chrg.chrgp_pwm = pwm_duty_c;
			}
			if (pwm_duty_c == PWM_DUTY_MAX) {
				/* scan done */
				printf("b %d end scan curr %d pwm %d\n",
				    active_batt, 
				    active_battctx.bc_r_chrg.chrgp_iout,
				    active_battctx.bc_r_chrg.chrgp_pwm);
				pwm_duty_c = active_battctx.bc_r_chrg.chrgp_pwm;
				pwm_set_duty();
				chrg_current_accum = 0;
				chrg_accum_cnt = 4;
				chrg_duty_change = 8;
				active_battctx.bc_rp_time = MPPT_RAMPUP_PERIOD;
				chrg_fsm = CHRG_MPPT;
			} else {
				pwm_duty_c += 5;
				if (pwm_duty_c > PWM_DUTY_MAX)
					pwm_duty_c = PWM_DUTY_MAX;
				pwm_set_duty();
			}
		}
		if (chrg_events.bits.gooff) {
			chrg_fsm = CHRG_GODOWN;
		}
		break;
			
	case CHRG_MPPT:
		if (pac_events.bits.pacavg_rdy_chrg) {
			chrg_current_accum +=
			    (uint16_t)(-_read_voltcur.batt_i[2 - active_bidx]);
			chrg_accum_cnt--;
			if (chrg_accum_cnt == 0) {
				chrg_mppt_compute();
			}
		}
		if (pac_events.bits.bvalues_updated) {
			pac_events.bits.bvalues_updated = 0;
			if ((active_battv / 10) > active_battctx.bc_cv + 1) {
				chrg_volt_target_cnt++;
				/*
				 * if we're above target + 0.1 for more than
				 * 0.4s, or above target + 0.5, switch to
				 * CV mode.
				 */
				if (chrg_volt_target_cnt > 40 || /* 0.4s */
				    (active_battv / 10) > active_battctx.bc_cv + 5) {
					chrg_volt_target_cnt = 0;
					chrg_fsm = CHRG_CV;
					chrg_batt_grace = 0;
				}
			} else {
				chrg_volt_target_cnt = 0;
			}
		}
		if (chrg_events.bits.battswitch &&
		    chrg_current_accum == 0 && chrg_volt_target_cnt == 0 &&
		    chrg_fsm == CHRG_MPPT) {
			active_battctx.bc_chrg_fsm = chrg_fsm;
			active_battctx.bc_chrg.chrgp_pwm = pwm_duty_c;
			active_battctx.bc_chrg.chrgp_iout =
			    chrg_previous_current / 4;
			chrg_fsm = CHRG_BSWITCH;
		}
		if (chrg_events.bits.gooff) {
			chrg_fsm = CHRG_GODOWN;
		}
		break;
		
	case CHRG_CV:
		if (chrg_events.bits.gooff) {
			chrg_fsm = CHRG_GODOWN;
		} else if (pac_events.bits.bvalues_updated) {
			uint8_t _v = (uint8_t)(active_battv / 10);
			pac_events.bits.bvalues_updated = 0;
			/* wait for grace period before checking for mode sw */
			if (chrg_batt_grace)
				chrg_batt_grace--;
			if (chrg_batt_grace == 0 &&
			    _v < active_battctx.bc_cv - 1) {
				chrg_volt_target_cnt++;
				/*
				 * if we're below target - 0.1 for more than
				 * 0.1s, or below target - 0.5, switch to
				 * MPPT mode
				 */
				if (chrg_volt_target_cnt > 10 || /* 0.1s */
				    _v < active_battctx.bc_cv - 5) {
					pwm_duty_c = 20; /* start at 10% */
					pwm_set_duty();
					active_battctx.bc_r_chrg.chrgp_iout = -1;
					chrg_fsm = CHRG_RERAMPUP;
					chrg_volt_target_cnt = 0;
				}
			} else {
				chrg_volt_target_cnt = 0;
				if (chrg_events.bits.battswitch) {
					active_battctx.bc_chrg_fsm = chrg_fsm;
					active_battctx.bc_chrg.chrgp_pwm = pwm_duty_c;
					chrg_fsm = CHRG_BSWITCH;
				}
			}
			if (_v > active_battctx.bc_cv) 
				pwm_duty_c--;
			else if (_v < active_battctx.bc_cv) 
				pwm_duty_c++;
			pwm_set_duty();
		}
		break;

	case CHRG_BSWITCH:
		chrg_events.bits.battswitch = 0;
		if (chrg_events.bits.gooff) {
			chrg_fsm = CHRG_GODOWN;
		} else if (batt_switch_active()) {
			pwm_duty_c = 0;
			pwm_set_duty();
			batt_en(0);
			switch(pwm_fsm) {
			case PWMF_RUNNING:
			case PWMF_IDLE:
				pwm_fsm = PWMF_IDLE;
				active_battctx.bc_sw_time = timer0_read();
				chrg_fsm = CHRG_BSWITCH_WAIT;
				break;
			default:
				chrg_fsm = CHRG_GODOWN;
				break;
			}
		} else {
			/* back to previous state */
			chrg_fsm = active_battctx.bc_chrg_fsm;
			active_battctx.bc_sw_time = timer0_read();
		}
		break;
	case CHRG_BSWITCH_WAIT:
		if (chrg_events.bits.gooff) {
			chrg_fsm = CHRG_GODOWN;
		} else if ((timer0_read() - active_battctx.bc_sw_time) >
		    TIMER0_1MS) {
			batt_en(active_batt);
			chrg_volt_target_cnt = 0;
			chrg_current_accum = 0;
			chrg_accum_cnt = 4;
			chrg_duty_change = 8;
			chrg_fsm = active_battctx.bc_chrg_fsm;
			pwm_duty_c = active_battctx.bc_chrg.chrgp_pwm;
			chrg_previous_current =
			    (uint16_t)active_battctx.bc_chrg.chrgp_iout * 4;
			pwm_set_duty();
			pwm_fsm = PWMF_RUNNING;
			/*
			 * wait 4 read cycles before updating PWM
			 * (we have minimum 10 read cycles before switching)
			 */
			chrg_batt_grace = 4;
		}
		break;
	case CHRG_GODOWN:
		if (chrg_events.bits.gooff) {
			chrg_events.bits.gooff = 0;
			chrg_events.bits.goon = 0;
			pwm_events.bits.gooff = 1;
		}
		if (pwm_fsm == PWMF_DOWN)
			chrg_fsm = CHRG_DOWN;
		break;
	}
	pac_events.bits.pacavg_rdy_chrg = 0;
}

#define PAC_I2C_ADDR 0x2e

static volatile i2c_return_t i2c_return;

/* for journal */
static int64_t l600_current_acc[4];
static __uint24 l600_current_count;
static uint32_t l600_voltages_acc[4];

/* check negative current on battery */
static uint8_t negative_current_count;

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
		if (ad_temp > temps[i].val) {
			board_temp = temps[i - 1].temp -
			((temps[i - 1].temp - temps[i].temp)  /
		         (temps[i - 1].val - temps[i].val)  * 
			 (temps[i - 1].val - ad_temp));
			return;
		}
	} 
	board_temp = 0xffff;
}

static void
send_controller_status(char b)
{
	if (nmea2000_status != NMEA2000_S_OK)
		return;

	struct nmea2000_load_controller_data *data = (void *)&nmea2000_data[0];

	if (b < BATT_1 || b > BATT_2)
		return;
	char bidx = b - BATT_1;
	if (battctx[bidx].bc_stat == BATTS_NONE)
		return;

	PGN2ID(NMEA2000_LOAD_CONTROLLER_STATE, msg.id);
	msg.id.priority = NMEA2000_PRIORITY_INFO;
	msg.dlc = sizeof(struct nmea2000_load_controller_data);
	msg.data = &nmea2000_data[0];
	data->sid = sid;
	data->conn = b;
	data->state = battctx[bidx].bc_stat;
	if (check_batt_active(bidx)) {
		if (b == active_batt) {
			data->op_status = chrg_fsm;
			data->pwm_duty = pwm_duty_c;
		} else {
			data->op_status = battctx[bidx].bc_chrg_fsm;
			data->pwm_duty =  battctx[bidx].bc_chrg.chrgp_pwm;
		}
	} else {
		data->op_status = 0;
		data->pwm_duty =  0;
	}
	data->status = 0;
	data->ton = 0;
	data->toff = 0;
	if (! nmea2000_send_single_frame(&msg))
		printf("send NMEA2000_LOAD_CONTROLLER_STATE failed\n");
}

static void
send_dc_voltage_current(char b)
{
	if (nmea2000_status != NMEA2000_S_OK)
		return;

	struct nmea2000_dc_voltage_current_data *data = (void *)&nmea2000_data[0];
	char bidx;

	switch(b) {
	case 0:
		bidx = 2;
		break;
	case 1:
		bidx = 0;
		break;
	case 2:
		bidx = 1;
		break;
	default:
		return;
	}

	PGN2ID(NMEA2000_DC_VOLTAGE_CURRENT, msg.id);
	msg.id.priority = NMEA2000_PRIORITY_INFO;
	msg.dlc = sizeof(struct nmea2000_dc_voltage_current_data);
	msg.data = &nmea2000_data[0];
	data->sid = sid;
	data->conn = b;
	data->voltage = batt_v_d[bidx];
	__int24 *_ip = (__int24*)(data->current);
	*_ip = batt_i_d[bidx];
	data->reserved = 0;
	if (! nmea2000_send_single_frame(&msg))
		printf("send NMEA2000_DC_VOLTAGE_CURRENT failed\n");
}

static void
send_temperature()
{
	if (nmea2000_status != NMEA2000_S_OK)
		return;

	if (board_temp == 0xffff)
		return;

	struct nmea2000_temp *data = (void *)&nmea2000_data[0];

	PGN2ID(NMEA2000_TEMP, msg.id);
	msg.id.priority = NMEA2000_PRIORITY_INFO;
	msg.dlc = sizeof(struct nmea2000_temp);
	msg.data = &nmea2000_data[0];
	data->sid = sid;
	data->instance = 0;
	data->source = ENV_TSOURCE_INSIDE;
	__uint24 *_tp = (__uint24*)(data->temp);
	*_tp = (__uint24)board_temp * 10;
	data->settemp = TEMP_MAX / 10;
	if (! nmea2000_send_single_frame(&msg))
		printf("send NMEA2000_TEMP failed\n");
}

#if 0
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
	case NMEA2000_DC_VOLTAGE_CURRENT:
		for (char c = 0; c < 3; c++) {
			send_dc_voltage_current(c);
		}
		break;
	case NMEA2000_LOAD_CONTROLLER_STATE:
		for (char c = BATT_1; c <= BATT_2; c++) {
			send_controller_status(c);
		}
		break;
	case NMEA2000_TEMP:
		send_temperature();
		break;
#if 0
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
#ifdef N2K_PRINTF
	nmea2000_putchar(c);
#endif
}

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
		pac_events.bits.pacavg_rdy = 1;
		pac_events.bits.pacavg_rdy_chrg = 1;
		break;
	case PAC_READ_ACCUM:
		pac_events.bits.pacacc_rdy = 1;
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
			pac_events.bits.pacavg_rdy = 0;
			pac_events.bits.pacavg_rdy_chrg = 0;
		} else {
			pacops_pending.bits.read_accum = 0;
			pac_i2c_io.pac_type = PAC_READ_ACCUM;
			pac_events.bits.pacacc_rdy = 0;
		}
		pac_i2c_state = PAC_I2C_WAIT;
		return;
	}
}

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
	
static uint8_t
displaybuf_small(char invert)
{
	const u_char *font;
	char *cp;
	u_char len;
	u_char i;
	struct oled_i2c_buf_s *oled_s;
	u_char inv;

	if (invert)
		inv = 0xff;
	else
		inv = 0;

	if ((oled_s = oled_i2c_reset()) == NULL)
		return 0;

	len = 0;
	for (cp = oled_displaybuf; *cp != '\0'; cp++) {
		if (*cp == '\n' && len == 0) {
			len = (u_char)oled_s->oled_datalen;
			continue;
		}
		font = get_font5x8(*cp);
		for (i = 0; i < 5; i++) {
			oled_s->oled_databuf[oled_s->oled_datalen++] = font[i] ^ inv;
		}
		oled_s->oled_databuf[oled_s->oled_datalen++] = inv;
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
	return 1;
}

static uint8_t
displaybuf_medium(char invert)
{
	const u_char *font;
	char *cp;
	u_char i, n;
	struct oled_i2c_buf_s *oled_s;
	u_char inv;

	if (invert)
		inv = 0xff;
	else
		inv = 0x00;

	if ((oled_s = oled_i2c_reset()) == NULL)
		return 0;
	/* unfortunably we can't build the two lines in a single loop */
	for (n = 0, cp = oled_displaybuf; *cp != '\0'; cp++) {
		font = get_font10x16(*cp);
		for (i = 0; i < 10; i++, n++) {
			oled_s->oled_databuf[n] = font[i * 2] ^ inv;
		}
	}
	for (cp = oled_displaybuf; *cp != '\0'; cp++) {
		font = get_font10x16(*cp);
		for (i = 0; i < 10; i++, n++) {
			oled_s->oled_databuf[n] = font[i * 2 + 1] ^ inv;
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
	return 1;
}

static uint8_t
displaybuf_icon(char ic)
{
	const u_char *icon;
	char *cp;
	u_char i;
	struct oled_i2c_buf_s *oled_s;

	if ((oled_s = oled_i2c_reset()) == NULL)
		return 0;
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
	return 1;
}

static uint8_t
display_clear()
{
	struct oled_i2c_buf_s *oled_s;
	if ((oled_s = oled_i2c_reset()) == NULL)
		return 0;
	memset(oled_s->oled_databuf, 0, OLED_I2C_DATABUFSZ);
	oled_s->oled_datalen = OLED_DISPLAY_SIZE;
	OLED_CTRL(oled_s, 0x00); OLED_CTRL(oled_s, 0x10);  /* reset column start */
	OLED_CTRL(oled_s, 0x21); OLED_CTRL(oled_s, 0); OLED_CTRL(oled_s, 0x7f); /*column start/end */
	OLED_CTRL(oled_s, 0xb0); /* page start */
	oled_s->oled_type = OLED_CTRL_CLEAR;
	if (oled_i2c_state == OLED_I2C_IDLE)
		oled_i2c_state = OLED_I2C_WAIT;
	return 1;
}

/* display management */

typedef enum {
	PAGE_BATTSTAT,
	PAGE_BATTSTAT_DETAILS,
	PAGE_BATTSTAT_DEBUG,
	PAGE_BATTLIST,
	PAGE_BATTLIST_STATE,
} page_t;

static page_t active_page;

/*
 * page status:
 *  0: update state
 *  > 0: init state, with multiple init steps\
 * set to 1 on page switch
 */
uint8_t page_status; 

static void new_page(page_t);

static void
battstat2buf_small(u_char b)
{
	/*can't use double for voltage because %2.1f doens't work as expected*/
	uint16_t _v = batt_v_d[b];
	double amps = (double)batt_i_d[b] / 100;

	if (b < 2 && !check_batt_active(b)) {
		sprintf(oled_displaybuf,
		    "%2d.%1dV\n     ", _v / 100, (_v % 100) / 10);
	} else if (amps > 9.99 || amps < 0) {
		sprintf(oled_displaybuf,
		    "%2d.%1dV\n-.--A", _v / 100, (_v % 100) / 10);
	} else {
		sprintf(oled_displaybuf,
		    "%2d.%1dV\n%1.02fA", _v / 100, (_v % 100) / 10, amps);
	}
	displaybuf_small(0);
}

static void
display_battstat_icons(char solar)
{
	/* clear display and put static infos */
	switch(page_status) {
	case 1:
		if (display_clear() == 0)
			return;
		page_status++;
		/* fallthrough */
	case 2:
		if (solar) {
			oled_col = 0;
			oled_line = 0;
			if (displaybuf_icon(ICON_SUN) == 0)
				return;
		}
		page_status++;
		/* fallthrough */
	case 3:
		oled_col = 0;
		oled_line = 3;
		if (displaybuf_icon(ICON_LIGHT) == 0)
			return;
		page_status++;
		/* fallthrough */
	case 4:
		oled_col = 0;
		oled_line = 6;
		if (displaybuf_icon(ICON_ENGINE) == 0)
			return;
		page_status++;
		/* fallthrough */
	default:
		/* set page_status to 0 in calling page */
		break;
	}
	return;
}

/* to be called on ev_10hz event */
static void
display_battstat_small()
{
	if (page_status != 0) {
		display_battstat_icons(1);
		return;
	}
	if ((counter_10hz & 1) == 1) {
		double amps;
		/* update display 5 times per second */
		/* solar */
		oled_col = 20;
		oled_line = 0;
		battstat2buf_small(2);
		/* batt2 (service) */
		oled_col = 20;
		oled_line = 3;
		battstat2buf_small(1);
		/* batt1 (engine) */
		oled_col = 20;
		oled_line = 6;
		battstat2buf_small(0);
	}
}

static void
display_battstat_debug()
{
	display_battstat_small();
	if (page_status != 0) {
		if (page_status == 5) {
			page_status = 0;
		}
		return;
	}
	/*
	 * once per second display adc output and temperature 
	 * choose a counter_10hz value that doesn't conflict with
	 * display_battstat_small();
	 */
	if (counter_10hz == 2) {
		oled_col = 60;
		oled_line = 1;
		sprintf(oled_displaybuf, "%4x", ad_pwm);
		displaybuf_small(0);
		oled_col = 60;
		oled_line = 2;
		sprintf(oled_displaybuf, "%4x", ad_solar);
		displaybuf_small(0);
		oled_col = 60;
		oled_line = 3;
		sprintf(oled_displaybuf, "%2.2f%c", (float)board_temp / 100.0 - 273.15, 20);
		displaybuf_small(0);
	}
	/*
	 * once per second display battery status
	 * choose a counter_10hz value that doesn't conflict with
	 * display_battstat_small();
	 */
	if (counter_10hz == 4) {
		oled_col = 54;
		oled_line = 4;
		sprintf(oled_displaybuf, "%1x", battctx[1].bc_stat);
		displaybuf_small(0);
		oled_col = 54;
		oled_line = 7;
		sprintf(oled_displaybuf, "%1x", battctx[0].bc_stat);
		displaybuf_small(0);
	}
	/* display charger and pwm states 10 times per second */
	oled_col = 54;
	oled_line = 5;
	sprintf(oled_displaybuf, "%1x %1x %1x %2x %1x", pwm_fsm, pwm_error, chrg_fsm, pwm_duty_c, active_batt);
	displaybuf_small(0);
}

static void
displaybuf_battstat(char b)
{
	switch(battctx[b].bc_stat) {
	case BATTS_NONE:
		sprintf(oled_displaybuf, "none   ");
		break;
	case BATTS_BULK:
		sprintf(oled_displaybuf, "bulk   ");
		break;
	case BATTS_FLOAT:
		sprintf(oled_displaybuf, "float  ");
		break;
	case BATTS_STANDBY:
		sprintf(oled_displaybuf, "standby");
		break;
	case BATTS_ERR:
		sprintf(oled_displaybuf, "error  ");
		break;
	}
}

static void
display_battstat_details()
{
	display_battstat_small();
	if (page_status != 0) {
		if (page_status == 5) {
			page_status = 0;
		}
		return;
	}
	/*
	 * once per second display CAN status and temperature 
	 * choose a counter_10hz value that doesn't conflict with
	 * display_battstat_small();
	 */
	if (counter_10hz == 2) {
		oled_col = 60;
		oled_line = 0;
		if (nmea2000_status == NMEA2000_S_OK) {
			sprintf(oled_displaybuf, "a %3d", nmea2000_addr);
		} else {
			sprintf(oled_displaybuf, "a ---");
		}
		displaybuf_small(0);
		oled_col = 60;
		oled_line = 1;
		sprintf(oled_displaybuf, "%2.1f%c", (float)board_temp / 100.0 - 273.15, 20);
		displaybuf_small(0);
	}
	/*
	 * once per second display battery status
	 * choose a counter_10hz value that doesn't conflict with
	 * display_battstat_small();
	 */
	if (counter_10hz == 4) {
		for (uint8_t c = 0; c < 2; c++) {
			oled_col = 54;
			oled_line = (c == 0) ? 7 : 4;
			displaybuf_battstat(c);
			displaybuf_small(0);
		}
	}
}


static void
battstat2buf(u_char b)
{
	/*can't use double for voltage because %2.1f doens't work as expected*/
	uint16_t _v = batt_v_d[b];
	double amps = (double)batt_i_d[b] / 100;

	if (b < 2 && !check_batt_active(b)) {
		sprintf(oled_displaybuf,
		    "%2d.%1d     ", _v / 100, (_v % 100) / 10);
	} else if (amps > 9.99 || amps < 0) {
		sprintf(oled_displaybuf,
		    "%2d.%1d -.--", _v / 100, (_v % 100) / 10);
	} else {
		sprintf(oled_displaybuf,
		    "%2d.%1d %1.02f", _v / 100, (_v % 100) / 10, amps);
	}
	displaybuf_medium(0);
}

static void
display_battstat_init()
{
	/* clear display and put static infos */
	switch(page_status) {
	case 1:
	case 2:
	case 3:
	case 4:
		display_battstat_icons(1);
		return;
	case 5:
		oled_col = 40; /* 20 + 2 * 10 */
		oled_line = 2;
		sprintf(oled_displaybuf, "V       A");
		if (displaybuf_small(0) == 0)
			return;
		/* fallthrough */
	default:
		page_status = 0;
	}
	return;
}

static void
display_battstat()
{
	if (page_status != 0) {
		display_battstat_init();
		return;
	}
	if ((counter_10hz & 1) == 1) {
		double amps;
		/* update display 5 times per second */
		/* solar */
		oled_col = 20;
		oled_line = 0;
		battstat2buf(2);
		/* batt2 (service) */
		oled_col = 20;
		oled_line = 3;
		battstat2buf(1);
		/* batt1 (engine) */
		oled_col = 20;
		oled_line = 6;
		battstat2buf(0);
	}
}

static batt_t battlist_selected;

static void
display_battlist()
{
	switch(page_status) {
	case 0:
		return;
	case 1:
	case 2:
	case 3:
	case 4:
		display_battstat_icons(0);
		/* fallthrough */
	case 5:
		oled_col = 22;
		oled_line = 3;
		sprintf(oled_displaybuf, "B2");
		if (displaybuf_medium(battlist_selected == BATT_2) == 0)
			return;
		page_status++;
		/* fallthrough */
	case 6:
		oled_col = 22;
		oled_line = 6;
		sprintf(oled_displaybuf, "B1");
		if (displaybuf_medium(battlist_selected == BATT_1) == 0)
			return;
		page_status++;
		/* fallthrough */
	case 7:
		oled_col = 42;
		oled_line = 3;
		displaybuf_battstat(BATT_2 - BATT_1);
		if (displaybuf_medium(battlist_selected == BATT_2 &&
		    active_page == PAGE_BATTLIST_STATE) == 0)
			return;
		page_status++;
		/* fallthrough */
	case 8:
		oled_col = 42;
		oled_line = 6;
		displaybuf_battstat(BATT_1 - BATT_1);
		if (displaybuf_medium(battlist_selected == BATT_1 &&
		    active_page == PAGE_BATTLIST_STATE) == 0)
			return;
		page_status = 0;
		/* fallthrough */
	default:
		return;
	}
}

static void
battlist_next()
{
	if(battlist_selected == BATT_2)
		battlist_selected = BATT_1;
	else
		battlist_selected = BATT_2;
	page_status = 5; /* refresh display */
}

static void
battlist_select()
{
	new_page(PAGE_BATTLIST_STATE);
}

static void
battstate_next()
{
	char c = battlist_selected - BATT_1;
	switch(battctx[c].bc_stat) {
	case BATTS_BULK:
		battctx[c].bc_stat = BATTS_FLOAT;
		battctx[c].bc_cv = bparams[c].bp_float_voltage;
		battctx[c].bc_sw_time = timer0_read();
		battctx[c].bc_chrg_fsm = CHRG_RERAMPUP;
		break;
	case BATTS_FLOAT:
		battctx[c].bc_stat = BATTS_STANDBY;
		break;
	case BATTS_STANDBY:
		battctx[c].bc_stat = BATTS_BULK;
		battctx[c].bc_cv = bparams[c].bp_bulk_voltage;
		battctx[c].bc_sw_time = timer0_read();
		battctx[c].bc_chrg_fsm = CHRG_RERAMPUP;
		break;
	default:
		break;
	}
	page_status = 7; /* refresh display */
}

static void
battstate_select()
{
	new_page(PAGE_BATTSTAT_DETAILS);
}

static void
display_page()
{
	switch(active_page) {
	case PAGE_BATTSTAT:
		display_battstat();
		break;
	case PAGE_BATTSTAT_DETAILS:
		display_battstat_details();
		break;
	case PAGE_BATTSTAT_DEBUG:
		display_battstat_debug();
		break;
	case PAGE_BATTLIST:
	case PAGE_BATTLIST_STATE:
		display_battlist();
		break;
	}
}

static void
new_page(page_t p)
{
	page_status = 1;
	active_page = p;
	display_page();
}

static void
next_page()
{
	switch(active_page) {
	case PAGE_BATTSTAT:
		new_page(PAGE_BATTSTAT_DETAILS);
		break;
	case PAGE_BATTSTAT_DETAILS:
		new_page(PAGE_BATTSTAT_DEBUG);
		break;
	case PAGE_BATTSTAT_DEBUG:
		new_page(PAGE_BATTSTAT);
		break;
	default:
		break;
	}
}

static void
previous_page()
{
	switch(active_page) {
	case PAGE_BATTSTAT:
		new_page(PAGE_BATTSTAT_DEBUG);
		break;
	case PAGE_BATTSTAT_DEBUG:
		new_page(PAGE_BATTSTAT_DETAILS);
		break;
	case PAGE_BATTSTAT_DETAILS:
		new_page(PAGE_BATTSTAT);
		break;
	default:
		break;
	}
}

/* button FSM */
static enum {
	BTN_IDLE = 0,
	BTN_DOWN,
	BTN_DOWN_1,
	BTN_DOWN_1_P,
	BTN_DOWN_2,
	BTN_DOWN_2_P,
	BTN_UP
} btn_state;

u_int btn_time; /* key press time */
#define BTN_LONG_PRESS (TIMER0_100MS * 5)

static void
btn1_short()
{
	switch(active_page) {
	case PAGE_BATTSTAT:
	case PAGE_BATTSTAT_DETAILS:
	case PAGE_BATTSTAT_DEBUG:
		previous_page();
		break;
	case PAGE_BATTLIST:
		battlist_select();
		break;
	case PAGE_BATTLIST_STATE:
		battstate_select();
	}
}

static void
btn1_long()
{
	printf("B1 long %d\n", active_page);
	switch(active_page) {
	case PAGE_BATTSTAT:
	case PAGE_BATTSTAT_DETAILS:
	case PAGE_BATTSTAT_DEBUG:
		battlist_selected = BATT_2;
		new_page(PAGE_BATTLIST);
		break;
	case PAGE_BATTLIST:
	case PAGE_BATTLIST_STATE:
		new_page(PAGE_BATTSTAT_DETAILS);
	default:
		break;
	}
}

static void
btn2_short()
{
	switch(active_page) {
	case PAGE_BATTSTAT:
	case PAGE_BATTSTAT_DETAILS:
	case PAGE_BATTSTAT_DEBUG:
		next_page();
		break;
	case PAGE_BATTLIST:
		battlist_next();
		break;
	case PAGE_BATTLIST_STATE:
		battstate_next();
		break;
	}
}

static void
btn2_long()
{
	switch(active_page) {
	case PAGE_BATTSTAT:
		break;
	case PAGE_BATTSTAT_DETAILS:
		break;
	case PAGE_BATTSTAT_DEBUG:
		break;
	default:
		break;
	}
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
	PMD5 = 0xef; /* keep PWM1 */
	PMD6 = 0xf6; /* keep UART1 and I2C */
	PMD7 = 0xf2; /* keep CLC1, CLC3 and CLC4 */
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
	counter_100hz = 10;
	counter_100hz = 10;
	counter_10hz = 10;
	seconds = 0;
	time_events.byte = 0;

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

	/* configure timer0 as free-running counter at 9.765625Khz */
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
	/* configure UTMR for 100Hz interrupt */
	TU16ACON0 = 0x04; /* period match IE */
	TU16ACON1 = 0x00;
	TU16AHLT = 0x00; /* can't use hardware reset because of HW bug */
	TU16APS = 249; /* prescaler = 250 -> 40000 Hz */
	TU16APRH = (400 >> 8) & 0xff;
	TU16APRL = 400 & 0xff;
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

	printf("solar_mppt %d.%d %s", MAJOR, MINOR, buildstr);
	printf("hello user_id 0x%lx devid 0x%x revid 0x%x\n", nmea2000_user_id, devid, revid);

	printf("n2k_init\n");
	nmea2000_init();
	poll_count = timer0_read();

	/* enable watchdog */
	WDTCON0bits.SEN = 1;

	/* set up ADC */
	ADCON0 = 0x4; /* right-justified */
	ADCLK = 7; /* Fosc/16 */
	/* context 0: solar_v */
	ADCTX = 0;
	ADCON1 = 0; /* no cap */
	ADCON2 = 0; /* basic mode */
	ADCON3 = 0x08; /* SOI */
	ADREF = 2;  /* vref = vref+ (RA3) */
	ADPCH = 2; /* RA2 */
	ADACQH = 0;
	ADACQL = 20; /* 20Tad Aq */
	/* context 1: PWM_v */
	ADCTX = 1;
	ADCON1 = 0; /* no cap */
	ADCON2 = 0; /* basic mode */
	ADCON3 = 0x08; /* SOI */
	ADREF = 2;  /* vref = vref+ (RA3) */
	ADPCH = 0; /* RA0 */
	ADACQH = 0;
	ADACQL = 20; /* 20Tad Aq */
	/* XXX setup thresholds */
	/* context 2: NTC */
	ADCTX = 2;
	ADCON1 = 0; /* no cap */
	ADCON2 = 0; /* basic mode */
	ADCON3 = 0x0f; /* SOI, always interrupt */
	ADREF = 0;  /* vref = VDD */
	ADPCH = 5; /* RA5 */
	ADACQH = 0;
	ADACQL = 20; /* 20Tad Aq */

	/* XXX setup thresholds */
	/* context 3: buttons */
	ADCTX = 3;
	ADCON1 = 0; /* no cap */
	ADCON2 = 0; /* basic mode */
	ADCON3 = 0x0f; /* SOI, always interrupt */
	ADREF = 0;  /* vref = VDD */
	ADPCH = 1; /* RA1 */
	ADACQH = 0;
	ADACQL = 20; /* 20Tad Aq */

	ADCSEL1 = 0x80; /* CHEN */
	ADCSEL2 = 0x80;
	ADCSEL3 = 0xc0; /* CHEN | SSI */
	ADCSEL4 = 0x00;

	PIR2bits.ADCH1IF = 0;
	PIR2bits.ADCH2IF = 0;
	PIR2bits.ADCH3IF = 0;
	PIR2bits.ADCH4IF = 0;
	PIE2bits.ADCH1IE = 0;
	PIE2bits.ADCH2IE = 0;
	PIE2bits.ADCH3IE = 0;
	PIE2bits.ADCH4IE = 0;

	ADCON0bits.ADON = 1;

	ad_pwm = ad_solar = ad_temp = 0xffff;

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

	oled_col = 20;
	oled_line = 1;
	sprintf(oled_displaybuf, "solar_mppt %d.%d", MAJOR, MINOR);
	displaybuf_small(0);
	oled_i2c_flush();

	oled_col = 20;
	oled_line = 2;
	sprintf(oled_displaybuf, "%s", buildstr);
	displaybuf_small(0);
	oled_i2c_flush();

	oled_col = 20;
	oled_line = 3;
	sprintf(oled_displaybuf, "N2K user ID %lu", nmea2000_user_id);
	displaybuf_small(0);
	oled_i2c_flush();

	PAC_NDOWN = 1;
	pacops_pending.byte = 0;
	pac_events.byte = 0;
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
	else {
		printf("product id 0x%x ", i2cr);
		pac_pid = i2cr;
	}

	PAC_READREG(PAC_MANUF, i2cr);
	if (pac_i2c_flush() == 0)
		c++;
	else {
		printf("manuf id 0x%x ", i2cr);
		pac_mid = i2cr;
	}

	PAC_READREG(PAC_REV, i2cr);
	if (pac_i2c_flush() == 0)
		c++;
	else {
		printf("rev id 0x%x\n", i2cr);
		pac_rid = i2cr;
	}

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

	oled_col = 10;
	oled_line = 5;
	sprintf(oled_displaybuf, "pac 0x%x.0x%x.0x%x",
	    pac_pid, pac_mid, pac_rid);
	displaybuf_small(0);
	oled_i2c_flush();

	/* set up some of our input/output */
	PWM_OFF = 1;
	TRISCbits.TRISC1 = 0;

	PWM_MID = 0;
	TRISCbits.TRISC5 = 0;
	PWM_OUT = 0;
	TRISCbits.TRISC6 = 0;

	TRISCbits.TRISC0 = 1; /* PWM_OK input */


	LATBbits.LATB4 = 0; /* BATT1_ON */
	LATBbits.LATB5 = 0; /* BATT2_ON */
	TRISBbits.TRISB4 = 0;
	TRISBbits.TRISB5 = 0;

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

	/* setup CM1 for PWM output voltage alarm */
	CM1CON0 = 0x02; /* hysteresis */
	CM1CON1 = 0x01; /* interrupt falling edge */
	CM1NCH = 0x00; /* CH1IN0- = RA0 */
	CM1PCH = 0x01; /* CH1IN1+ = vref */
	CM1CON0bits.EN = 0; /* enable when PWM is running */
	PIR1bits.C1IF = 0; /* clear any pending interrupt */
	PIE1bits.C1IE = 0; /* enable when PWM is running */

	/* setup IOC to monitor PWM_OK (RC0) going low */
	IOCAP = IOCBP = IOCCP = IOCEP = 0;
	IOCAN = IOCBN = IOCEN = 0;
	IOCCN = 1;
	PIE0bits.IOCIE = 0; /* enable when PWM is running */

	/* setup CLC1 for PWM reset: (!PWM_OK | !CM1OUT) */
	CLCSELECT = 0;
	CLCnCON = 0x02; /* 4 input and */
	CLCnPOL = 0x0e; /* 1 noninverted; 2, 3 & 4 inverted */
	CLCnSEL0 = 0; /* CLCIN0PPS = PWM_OK */
	CLCnSEL1 = 45; /* CM1OUT */
	CLCnSEL2 = 0; /* CLCIN0PPS = PWM_OK */
	CLCnSEL3 = 0; /* CLCIN0PPS = PWM_OK */

	CLCnGLS0 = 0x05; /* b00000101 (!PWM_OK| !CM1OUT) */
	CLCnGLS1 = 0;
	CLCnGLS2 = 0;
	CLCnGLS3 = 0;
	CLCnCONbits.EN = 1;

	/* setup PWM */
	OSCFRQ = 0x08; /* HFINTOSC at 64Mhz */
	PWM1ERS = 0x0a; /* CLC1_OUT */
	PWM1LDS = 0; /* no autoload */
	PWM1CLK = 0x03; /* use HFINTOSSC: minimum off time = 32 */
	PWM1CPRE = 0; /* no prescale */
	PWM1PR = 512; /* pwm at 125Khz */
	PWM1GIE = 0;
	PWM1CON = 0; /* no enable yet */
	PWM1S1CFG = 0; /* left-aligned mode */
	PWM1S1P1 = 0; /* default to off */
	PWM1S1P2 = 0;
	RC6PPS = 0x18; /* PWM1S1P1_OUT */

	pwm_fsm = PWMF_DOWN;
	pwm_events.byte = 0;
	pwm_error = PWME_NOERROR;

	chrg_fsm = CHRG_DOWN;
	chrg_events.byte = 0;

	/* XXX from eeprom ? */
	bparams[BATT_1 - BATT_1].bp_bulk_voltage_limit = 125;
	bparams[BATT_1 - BATT_1].bp_bulk_voltage = 140;
	bparams[BATT_1 - BATT_1].bp_float_voltage = 135;
	bparams[BATT_2 - BATT_1].bp_bulk_voltage_limit = 125;
	bparams[BATT_2 - BATT_1].bp_bulk_voltage = 140;
	bparams[BATT_2 - BATT_1].bp_float_voltage = 135;

	active_batt = BATT_1;
	for (c = 0; c < 2; c++) {
		battctx[c].bc_stat = BATTS_NONE;
	}

	btn_state = BTN_IDLE;

	FVRCON = 0x8c; /* FVR on, CDAFVR 4.096v */

	/* CM2 for button interrupt */
	CM2CON0 = 0x42; /* invert polarity, hysteresis */
	CM2CON1 = 0x03; /* interrupt on both edges */
	CM2NCH = 0x01; /* CH1IN1- */
	CM2PCH = 0x06; /* FVR */
	CM2CON0bits.EN = 1;
	PIR14bits.C2IF = 0; /* clear any pending interrupt */
	PIE14bits.C2IE = 1; /* enable */

	/* wait 1s, clear display and start operations */
	for (c = 0; c < 100; c++) {
		t0 = timer0_read();
		while (timer0_read() - t0 < (TIMER0_1MS * 10)) {
			CLRWDT();
		}
	}

	/* clear display */
	display_clear();
	oled_i2c_flush();
	page_status = 1;
	active_page = 0;

	PIR2bits.ADCH3IF = 0;
	PIE2bits.ADCH3IE = 1;
	ADCTX = 0;
	ADCON0bits.CSEN = 1;
	ADCON0bits.GO = 1;

	memset(batt_v_s, 0, sizeof(batt_v_s));
	memset(batt_i_s, 0, sizeof(batt_i_s));
	batt_s_idx = 0;
	batt_s_size = 0;
	batt_a_count = BATT_A_NCOUNT;
	printf("enter loop\n");

	while (1) {
		time_events.byte = 0;
		if (softintrs.bits.int_100hz) {
			softintrs.bits.int_100hz = 0;
			time_events.bits.ev_100hz = 1;
			counter_100hz--;
			if (counter_100hz == 0) {
				counter_100hz = 10;
				time_events.bits.ev_10hz = 1;
				counter_10hz--;
				if (counter_10hz == 0) {
					counter_10hz = 10;
					time_events.bits.ev_1hz = 1;
				}
			}
		}
				
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

		if (time_events.bits.ev_1hz) {
			adctotemp(0);
			if (board_temp > TEMP_MAX &&
			    chrg_fsm != CHRG_DOWN) {
				pwm_error = PWME_OVERTEMP;
				pwm_events.bits.gooff = 1;
			}
			for (c = 0; c < 2; c++) {
				if (battctx[c].bc_rp_time != 0)
					battctx[c].bc_rp_time--;
			}
			for (char c = 0; c < 3; c++) {
				send_dc_voltage_current(c);
			}
			for (char c = BATT_1; c <= BATT_2; c++) {
				send_controller_status(c);
			}
			send_temperature();
			SIDINC(sid);
		}

		chrg_runfsm();
		pwm_runfsm();

		if (time_events.bits.ev_10hz &&
		    chrg_fsm == CHRG_DOWN && pwm_error != PWME_NOERROR) {
			if (pwm_error == PWME_OVERTEMP) {
				/* restart when below TEMP_RESTART */
				if (board_temp < TEMP_RESTART) {
					pwm_error = PWME_NOERROR;
				}
			} else {
				pwme_time++;
				if (pwme_time >= 20) {
					pwm_error = PWME_NOERROR;
				}
			}
		}

		PIE14bits.C2IE = 0;
		if (softintrs.bits.int_btn_down) {
			softintrs.bits.int_btn_down = 0;
			if (btn_state == BTN_IDLE) {
				btn_state = BTN_DOWN;
				btn_time = timer0_read();
			}
		}
		if (softintrs.bits.int_btn_up) {
			switch(btn_state) {
			case BTN_DOWN_1_P:
				softintrs.bits.int_btn_up = 0;
				btn1_short();
				/* this is a up event */
				btn_state = BTN_UP;
				break;
			case BTN_DOWN_2_P:
				softintrs.bits.int_btn_up = 0;
				btn2_short();
				/* this is a up event */
				btn_state = BTN_UP;
				break;
			case BTN_DOWN:
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
		PIE14bits.C2IE = 1;

		if (softintrs.bits.int_adcc) {
			switch(ADCTX) {
			case 3:
				PIR2bits.ADCH4IF = 0;
				PIE2bits.ADCH4IE = 0;
				if (btn_state == BTN_DOWN) {
					if (ADRES > 0x600 && ADRES < 0x900) {
						btn_state = BTN_DOWN_2;
					} else if (ADRES > 0x400 &&
						   ADRES < 0x600) {
						btn_state = BTN_DOWN_1;
					} else {
						/* transient event */
						btn_state = BTN_IDLE;
					}
				} else {
					btn_state = BTN_IDLE;
				}
				break;
			default:
				PIR2bits.ADCH3IF = 0;
				ADCON0bits.CSEN = 0;
				/* save channel values */
				ADCTX = 0;
				ad_solar = ADRES;
				ADCTX = 1;
				ad_pwm = ADRES;
				ADCTX = 2;
				if (ADRES == 0 || ADRES == 0xfff)
					ad_temp = 0xffff;
				else if (ad_temp == 0 || ad_temp == 0xffff)
					ad_temp = ADRES;
				else
					ad_temp = (ad_temp + ADRES) / 2;
				break;
			}
			softintrs.bits.int_adcc = 0;
			if (btn_state != BTN_DOWN) {
				PIR2bits.ADCH3IF = 0;
				PIE2bits.ADCH3IE = 1;
				ADCTX = 0;
				ADCON0bits.CSEN = 1;
				ADCON0bits.GO = 1;
			}
		}

		switch (btn_state) {
		case BTN_DOWN:
			/* get btn value */
			if (!ADCON0bits.GO) {
				ADCON0bits.CSEN = 0;
				ADCTX = 3;
				PIR2bits.ADCH4IF = 0;
				PIE2bits.ADCH4IE = 1;
				ADCON0bits.GO = 1;
			}
			break;
		case BTN_DOWN_1:
			printf("B1\n");
			btn_state = BTN_DOWN_1_P;
			break;
		case BTN_DOWN_1_P:
			if ((timer0_read() - btn_time) > BTN_LONG_PRESS) {
				btn1_long();
				/* this is a up event */
				btn_state = BTN_UP;
			}
			break;
		case BTN_DOWN_2:
			printf("B2\n");
			btn_state = BTN_DOWN_2_P;
			break;
		case BTN_DOWN_2_P:
			if ((timer0_read() - btn_time) > BTN_LONG_PRESS) {
				btn2_long();
				/* this is a up event */
				btn_state = BTN_UP;
			}
			break;
		case BTN_UP:
			printf("up\n");
			btn_state = BTN_IDLE;
			break;
		default:
			break;
		}

		if (time_events.bits.ev_100hz) {
			/* read PAC values every 10ms, but also accum every s */
			if (counter_100hz == 1 && counter_10hz == 1) {
				pacops_pending.bits.refresh = 1;
				pacops_pending.bits.read_accum = 1;
			} else {
				pacops_pending.bits.refresh_v = 1;
			}
			pacops_pending.bits.read_values = 1;
		}
		if (time_events.bits.ev_10hz) {
			display_page();
		
			if (time_events.bits.ev_1hz) {
				seconds++;
				/* every 10s print states */
				if ((seconds % 10) == 0) {
					uint16_t _v = batt_v_d[2];
					double amps = (double)batt_i_d[2] / 100;
					printf("So %2d.%1dV %1.02fA",
					    _v / 100, (_v % 100) / 10, amps);
					_v = batt_v_d[1];
					amps = (double)batt_i_d[1] / 100;
					printf(" Se %2d.%1d %1.02fA 0x%x",
					    _v / 100, (_v % 100) / 10, amps,
					    battctx[1].bc_stat);
					_v = batt_v_d[0];
					amps = (double)batt_i_d[0] / 100;
					printf(" Mo %2d.%1d %1.02fA 0x%x",
					    _v / 100, (_v % 100) / 10, amps,
					    battctx[0].bc_stat);
					printf(" %2.2f%c",
					    (float)board_temp / 100.0 - 273.15,
					    20);
					printf("\n");
				}
				if (seconds == 600) {
					// USELOG update_log();
					seconds = 0;
				}
				// USEADC  ADCON0bits.ADON = 1; /* start a new cycle */
			}
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
		if (pac_events.bits.pacacc_rdy) {
			int64_t acc_value;
			double v;
			char *acc_bytes = (void *)&acc_value;
			pac_events.bits.pacacc_rdy = 0;
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
		}
		if (pac_events.bits.pacavg_rdy) {
			double v;
			pac_events.bits.pacavg_rdy = 0;
			// printf("avg");
			batt_a_count--;
			if (batt_a_count == 0) {
				batt_s_idx++;
				batt_s_idx = batt_s_idx & (BATT_S_NCOUNT - 1);
				batt_s_count++;
				if (batt_s_count > BATT_S_NCOUNT)
					batt_s_count = BATT_S_NCOUNT;
			}
			for (c = 0; c < 3; c++) {
				uint8_t bi = 2  - c;
				/* i = acc_value * 0.00075 */
				/* batt_i = acc_value * 0.00075 * 100 */
				v = (double)_read_voltcur.batt_i[c] * 0.075;
				if (c != 0) {
					/* batt1/batt2 are inverted */
					batt_i[bi] = (int16_t)(-v + 0.5);
				} else {
					batt_i[bi] = (int16_t)(v + 0.5);
				}
				batt_i_a[bi] += batt_i[bi];

				// printf(" %4.4fA", v / 100);
				/* volt = vbus * 0.000488 */
				/* batt_v = vbus * 0.000488 * 100 */      
				v = (double)_read_voltcur.batt_v[c] * 0.0488;
				batt_v[bi] = (uint16_t)(v + 0.5);
				// printf(" %4.4fV", v / 100);
				batt_v_a[bi] += batt_v[bi];

				if (batt_a_count == 0) {
					int16_t  _i;
					uint16_t _v;
					_i = batt_i_a[bi] / BATT_A_NCOUNT;
					batt_i_a[bi] = 0;
					batt_i_s[batt_s_idx][bi] = _i;
					_v = batt_v_a[bi] / BATT_A_NCOUNT;
					batt_v_a[bi] = 0;
					batt_v_s[batt_s_idx][bi] = _v;
				}
			}
			if (batt_a_count == 0) {
				batt_a_count = BATT_A_NCOUNT;
				/*
				 * update display values. If batt_s_size is 0
				 * (the charger is not running, or just
				 * started), use the last value.
				 */
				uint8_t idx, bi;
				for (bi = 0; bi < 3; bi++) {
					batt_v_d[bi] = batt_v_s[batt_s_idx][bi];
					batt_i_d[bi] = batt_i_s[batt_s_idx][bi];
				}
				for (idx = batt_s_idx - 1, c = batt_s_size;
				     c > 1; idx--, c--) {
					idx = idx & (BATT_S_NCOUNT - 1);
					for (bi = 0; bi < 3; bi++) {
						batt_v_d[bi]+=batt_v_s[idx][bi];
						batt_i_d[bi]+=batt_i_s[idx][bi];
					}
				}
				if (batt_s_size > 1) {
					for (bi = 0; bi < 3; bi++) {
						batt_v_d[bi] /= batt_s_size;
						batt_i_d[bi] /= batt_s_size;
					}
				}
			}
			// printf("\n");
			if (pwm_error == PWME_NOERROR) {
				check_batt_status();
			}
			pac_events.bits.bvalues_updated = 1;
			if (pwm_fsm == PWMF_RUNNING) {
				switch(active_batt) {
				case BATT_1:
					c = 2;
					break;
				case BATT_2:
					c = 1;
					break;
				default:
					c = 0; /* shouldn't happen */
					break;
				}
				/* batt current is inverted */
				if (_read_voltcur.batt_i[c] <= 0)
					negative_current_count = 0;
				else {
					/*
					 * count proportional to negative
					 * current, and abort immediately if
					 * more than 200mA (266)
					 */
					negative_current_count +=
					    (_read_voltcur.batt_i[c] + 25) / 26;
					printf("battneg %d %d %d\n", negative_current_count, active_batt, _read_voltcur.batt_i[c]);
				}
				if (negative_current_count >= 10) {
					pwm_error = PWME_BATTCUR;
					batt_en(BATT_NONE);
					pwm_events.bits.gooff = 1;
					pwme_time = 0;
				}
			} else {
				negative_current_count = 0;
			}
			if (pwm_error == PWME_NOERROR)
				schedule_batt_switch();
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
	if (--counter_250hz == 0) {
		counter_250hz = 25;
		softintrs.bits.int_10hz = 1;
	}			
}
#else
void __interrupt(__irq(TU16A), __high_priority, base(IVECT_BASE))
irqh_tu16a(void)
{
	TU16ACON1bits.CLR = 1;
	TU16ACON1bits.PRIF = 0;
	softintrs.bits.int_100hz = 1;
}
#endif

void __interrupt(__irq(IOC), __low_priority, base(IVECT_BASE))
irqh_ioc(void)
{
	PIR0bits.IOCIF = 0;
	IOCCFbits.IOCCF0 = 0;
	PIE0bits.IOCIE = 0;
	pwm_events.bits.gooff = 1;
	pwm_error = PWME_PWMOK;
	pwme_time = 0;
}

void __interrupt(__irq(CM1), __low_priority, base(IVECT_BASE))
irqh_cm1(void)
{
	PIR1bits.C1IF = 0;
	PIE1bits.C1IE = 0;
	pwm_events.bits.gooff = 1;
	pwm_error = PWME_PWMV;
	pwme_time = 0;
}

void __interrupt(__irq(CM2), __low_priority, base(IVECT_BASE))
irqh_cm2(void)
{
	if (CM2CON0bits.OUT)
		softintrs.bits.int_btn_down = 1;
	else 
		softintrs.bits.int_btn_up = 1;
	PIR14bits.C2IF = 0;
}

void __interrupt(__irq(ADCH3), __low_priority, base(IVECT_BASE))
irqh_adcc3(void)
{
	PIE2bits.ADCH3IE = 0;
	softintrs.bits.int_adcc = 1;
}

void __interrupt(__irq(ADCH4), __low_priority, base(IVECT_BASE))
irqh_adcc4(void)
{
	PIE2bits.ADCH4IE = 0;
	softintrs.bits.int_adcc = 1;
}

void __interrupt(__irq(default), __low_priority, base(IVECT_BASE))
irqh_default(void)
{
	default_src = 0xff;
}

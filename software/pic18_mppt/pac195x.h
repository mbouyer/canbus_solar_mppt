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

/* registers addresses and data structure */

/*
 * data comes in MSB first, so in a 2 bytes structure we have bits 8-15
 * followeed by bits 0-7
 */
#define PAC_REFRESH	0x00 /* command only */

#define PAC_CTRL	0x01
typedef struct {
	uint8_t	ctrl_pad	: 4;
	uint8_t	ctrl_chan_dis	: 4;
	uint8_t	ctrl_alert1	: 2;
	uint8_t	ctrl_alert2	: 2;
	uint8_t	ctrl_mode	: 4;
} pac_ctrl_t;

#define CTRL_MODE_ADAP_1024	0x0
#define CTRL_MODE_1024		0x4
#define CTRL_MODE_8		0x7
#define CTRL_MODE_SINGLE	0x8
#define CTRL_MODE_SINGLE_8X	0x9
#define CTRL_MODE_ADAP_SLEEP	0xf

#define CTRL_ALERT_ALERT	0x0
#define CTRL_ALERT_GPIO_IN	0x1
#define CTRL_ALERT_GPIO_OUT	0x2
#define CTRL_ALERT_SLOW		0x3

#define PAC_ACCCNT	0x02
typedef struct {
	uint32_t acccnt_count;
} pac_acccnt_t;

#define PAC_ACCV1	0x03
#define PAC_ACCV2	0x04
#define PAC_ACCV3	0x05
#define PAC_ACCV4	0x06
typedef struct {
	uint8_t accv_value[7];
} pac_accv_t;

#define PAC_VBUS1	0x07
#define PAC_VBUS2	0x08
#define PAC_VBUS3	0x09
#define PAC_VBUS4	0x0a
typedef union {
	uint16_t vbus_u;
	int16_t  vbus_s;
} pac_vbus_t;

#define PAC_VSENSE1	0x0b
#define PAC_VSENSE2	0x0c
#define PAC_VSENSE3	0x0d
#define PAC_VSENSE4	0x0e
typedef union {
	uint16_t vsense_u;
	int16_t  vsense_s;
} pac_vsense_t;

#define PAC_VBUS1_AVG	0x0f
#define PAC_VBUS2_AVG	0x10
#define PAC_VBUS3_AVG	0x11
#define PAC_VBUS4_AVG	0x12
/* use pac_vbus */

#define PAC_VSENSE1_AVG	0x13
#define PAC_VSENSE2_AVG	0x14
#define PAC_VSENSE3_AVG	0x15
#define PAC_VSENSE4_AVG	0x16
/* use pac_vsense */

#define PAC_VPOWER1	0x17
#define PAC_VPOWER2	0x18
#define PAC_VPOWER3	0x19
#define PAC_VPOWER4	0x1a
typedef union {
	uint32_t vpower_u; /* needs >> 2 */
	int32_t  vpower_s; /* needs >> 2 */
} pac_vpower_t;

#define PAC_SMBUS	0x1c
typedef struct {
	uint8_t	smbus_hs	:1;
	uint8_t	smbus_noskip	:1;
	uint8_t	smbus_bytecount	:1;
	uint8_t	smbus_timeout	:1;
	uint8_t	smbus_por	:1;
	uint8_t	smbus_alert	:1;
	uint8_t	smbus_data1	:1;
	uint8_t	smbus_data2	:1;
} pac_smbus_t;

#define PAC_NEG_PWR_FSR	0x1d
typedef struct {
	uint8_t pwrfsr_vb4	:2;
	uint8_t pwrfsr_vb3	:2;
	uint8_t pwrfsr_vb2	:2;
	uint8_t pwrfsr_vb1	:2;
	uint8_t pwrfsr_vs4	:2;
	uint8_t pwrfsr_vs3	:2;
	uint8_t pwrfsr_vs2	:2;
	uint8_t pwrfsr_vs1	:2;
} pac_neg_pwr_fsr_t;
#define PAC_PWRSFR_UFSR		0x0
#define PAC_PWRSFR_BFSR		0x1
#define PAC_PWRSFR_BHALF	0x2

#define PAC_REFRESH_G	0x1e
/* command only */

#define PAC_REFRESH_V	0x1f
/* command only */

#define PAC_SLOW	0x20
typedef struct {
	uint8_t	slow_pad	:1;
	uint8_t	slow_r_v_fall	:1;
	uint8_t	slow_r_fall	:1;
	uint8_t	slow_r_v_rise	:1;
	uint8_t	slow_r_rise	:1;
	uint8_t	slow_hl		:1;
	uint8_t	slow_lh		:1;
	uint8_t	slow_slow	:1;
} pac_slow_t;

#define PAC_CTRL_ACT	0x21
/* uses pac_ctrl */

#define PAC_NEG_PWR_FSR_ACT	0x22
/* uses pac_neg_pwr_fsr */

#define PAC_CRTL_LAT	0x23
/* uses pac_ctrl */

#define PAC_NEG_PWR_FSR_LAT	0x24
/* uses pac_neg_pwr_fsr */

#define PAC_ACCUMCFG	0x25
typedef struct {
	uint8_t	accumcfg_acc4	:2;
	uint8_t	accumcfg_acc3	:2;
	uint8_t	accumcfg_acc2	:2;
	uint8_t	accumcfg_acc1	:2;
} pac_accumcfg_t;

#define ACCUMCFG_POWER	0x0
#define ACCUMCFG_VSENSE	0x1
#define ACCUMCFG_VBUS	0x2

#define PAC_ALERTST	0x26
typedef struct {
	uint8_t alertst_ch4uc	:1;
	uint8_t alertst_ch3uc	:1;
	uint8_t alertst_ch2uc	:1;
	uint8_t alertst_ch1uc	:1;
	uint8_t alertst_ch4oc	:1;
	uint8_t alertst_ch3oc	:1;
	uint8_t alertst_ch2oc	:1;
	uint8_t alertst_ch1oc	:1;
	uint8_t alertst_ch4uv	:1;
	uint8_t alertst_ch3uv	:1;
	uint8_t alertst_ch2uv	:1;
	uint8_t alertst_ch1uv	:1;
	uint8_t alertst_ch4ov	:1;
	uint8_t alertst_ch3ov	:1;
	uint8_t alertst_ch2ov	:1;
	uint8_t alertst_ch1ov	:1;
	uint8_t alertst_pad	:2;
	uint8_t alertst_count	:1;
	uint8_t alertst_ovf	:1;
	uint8_t alertst_ch4op	:1;
	uint8_t alertst_ch3op	:1;
	uint8_t alertst_ch2op	:1;
	uint8_t alertst_ch1op	:1;
} pac_alertst_t;

#define PAC_ALERT1	0x27
#define PAC_ALERT2	0x28
typedef struct {
	uint8_t	alert_ch4uc	:1;
	uint8_t	alert_ch3uc	:1;
	uint8_t	alert_ch2uc	:1;
	uint8_t	alert_ch1uc	:1;
	uint8_t	alert_ch4oc	:1;
	uint8_t	alert_ch3oc	:1;
	uint8_t	alert_ch2oc	:1;
	uint8_t	alert_ch1oc	:1;
	uint8_t	alert_ch4uv	:1;
	uint8_t	alert_ch3uv	:1;
	uint8_t	alert_ch2uv	:1;
	uint8_t	alert_ch1uv	:1;
	uint8_t	alert_ch4ov	:1;
	uint8_t	alert_ch3ov	:1;
	uint8_t	alert_ch2ov	:1;
	uint8_t	alert_ch1ov	:1;
	uint8_t	alert_pad	:1;
	uint8_t	alert_cc	:1;
	uint8_t	alert_count	:1;
	uint8_t	alert_ovf	:1;
	uint8_t	alert_ch4op	:1;
	uint8_t	alert_ch3op	:1;
	uint8_t	alert_ch2op	:1;
	uint8_t	alert_ch1op	:1;
} pac_alert_t;

#define PAC_ACC_LIMIT	0x29
typedef struct {
	uint8_t	acc_limit_ch4	:2;
	uint8_t	acc_limit_ch3	:2;
	uint8_t	acc_limit_ch2	:2;
	uint8_t	acc_limit_ch1	:2;
	uint8_t	acc_limit_pad	:6;
	uint8_t	acc_limit_count	:2;
} pac_acc_limit_t;

#define PAC_OCLIMIT1	0x30
#define PAC_OCLIMIT2	0x31
#define PAC_OCLIMIT3	0x32
#define PAC_OCLIMIT4	0x33
typedef struct {
	int16_t	limit;
} pac_limit_t;

#define PAC_UCLIMIT1	0x34
#define PAC_UCLIMIT2	0x35
#define PAC_UCLIMIT3	0x36
#define PAC_UCLIMIT4	0x37
/* use pac_limit */

#define PAC_OPLIMIT1	0x38
#define PAC_OPLIMIT2	0x39
#define PAC_OPLIMIT3	0x3a
#define PAC_OPLIMIT4	0x3b
typedef struct {
	__int24	oplimit;
} pac_oplimit_t;

#define PAC_OVLIMIT1	0x3c
#define PAC_OVLIMIT2	0x3d
#define PAC_OVLIMIT3	0x3e
#define PAC_OVLIMIT4	0x3f
/* use pac_limit */

#define PAC_UVLIMIT1	0x40
#define PAC_UVLIMIT2	0x41
#define PAC_UVLIMIT3	0x42
#define PAC_UVLIMIT4	0x43
/* use pac_limit */

#define PAC_OCLIMIT_SMPL 0x44
#define PAC_UCLIMIT_SMPL 0x45
#define PAC_OPLIMIT_SMPL 0x46
#define PAC_OVLIMIT_SMPL 0x47
#define PAC_UVLIMIT_SMPL 0x48
typedef struct {
	uint8_t limit_smpl4	:2;
	uint8_t limit_smpl3	:2;
	uint8_t limit_smpl2	:2;
	uint8_t limit_smpl1	:2;
} pac_limit_smpl_t;

#define PAC_ALERT_EN	0x49
/* uses pac_alert */

#define PAC_ACCUMCFG_ACT 0x4a
#define PAC_ACCUMCFG_LAT 0x4b
/* uses pac_accumcfg */

#define PAC_PRODUCT	0xfd
#define PAC_MANUF	0xfe
#define PAC_REV		0xff

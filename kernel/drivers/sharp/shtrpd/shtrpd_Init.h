/* drivers/sharp/shtrpd/shtrpd_Init.h  (Trackpad driver)
 *
 * Copyright (C) 2011 SHARP CORPORATION
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define PRODUCT_NUMBER			40
#define PROJECT_NUMBER			10
#define VERSION_NUMBER			56

#define CONTROLSETTINGS0_VAL	0x00
#define CONTROLSETTINGS1_VAL	0x99
#define CONTROLSETTINGS2_VAL	0x00
#define CONTROLSETTINGS3_VAL	0x02

#define PROXTHRESHOLD_VAL		75
#define TOUCHMULTIPLIER_VAL		20
#define TOUCHSHIFTER_VAL		7
#define PMPROXTHRESHOLD_VAL		10
#define SNAPTHRESHOLD_VAL		310
#define PROXTHRESHOLD2_VAL		25
#define TOUCHMULTIPLIER2_VAL	25
#define TOUCHSHIFTER2_VAL		7
#define INTOUCHMULTIPLIER_VAL		16

#define ATITARGET_VAL			900
#define ATIC_VAL				1
#define ATITARGET2_VAL			600
#define ATIC2_VAL				0

#define FILTERSETTINGS0_VAL		0x00
#define TOUCHDAMPING_VAL		128
#define PMCOUNTDAMPING_VAL		16
#define LPPMCOUNTDAMPING_VAL	128

#define RESEEDTIME_VAL			255
#define COMMSTIMEOUT_VAL		100
#define MODETIME_VAL			8
#define LPTIME_VAL				5
#define SLEEPTIME_VAL			3

#define TOTALRXS_VAL			9
#define TOTALTXS_VAL			15
#define TRACKPADRXS_VAL			9
#define TRACKPADTXS_VAL			15
#define PMSETUP0_VAL			0x10
#define TXHIGH_VAL				0x7F
#define TXLOW_VAL				0xFF

#define PROXSETTINGS0_VAL		0x24
#define PROXSETTINGS1_VAL		0x70
#define PROXSETTINGS2_VAL		0x00
#define PROXSETTINGS3_VAL		0x43

#define ACTIVECHANNELS0_VAL		0x3FF
#define ACTIVECHANNELS1_VAL		0x3FF
#define ACTIVECHANNELS2_VAL		0x3FF
#define ACTIVECHANNELS3_VAL		0x3FF
#define ACTIVECHANNELS4_VAL		0x3FF
#define ACTIVECHANNELS5_VAL		0x3FF
#define ACTIVECHANNELS6_VAL		0x3FF
#define ACTIVECHANNELS7_VAL		0x3FF
#define ACTIVECHANNELS8_VAL		0x3FF
#define ACTIVECHANNELS9_VAL		0x3FF
#define ACTIVECHANNELS10_VAL	0x3FF
#define ACTIVECHANNELS11_VAL	0x3FF
#define ACTIVECHANNELS12_VAL	0x3FF
#define ACTIVECHANNELS13_VAL	0x3FF
#define ACTIVECHANNELS14_VAL	0x3FF

#define PROXDB_VAL				0x44
#define TOUCHSNAPDB_VAL			0x44
#define TOUCHSNAPDB2_VAL			0x22

#define PMATITARGET_VAL			5500
#define PMATIC_VAL				28

#define TEMP_NON_DETECT_AVG    16
#define TEMP_MIN_DELTA         2
#define TEMP_LTA_ADJ_INTERVAL  2

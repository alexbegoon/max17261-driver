/*
 *  @file:  max17261.h
 *  @brief: MAX17261 Driver
 *
 *  Copyright (C) 2019 Libre Space Foundation (https://libre.space)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#ifndef MAX17261_H_
#define MAX17261_H_

#include <stdint.h>

#define MAX17261_ADDRESS			(0x6C)
#define	MAX17261_RB_THRESHOLD		(3)
#define R_SENSE 					(0.01)

#define CAPACITY_MULTIPLIER				((5e-3) / R_SENSE)
#define CURRENT_MULTIPLIER  			((1.5625e-3) / R_SENSE)
#define CURRENT_MULTIPLIER_MINMAX 	 	(0.4 / R_SENSE)
#define VOLTAGE_MULTIPLIER_V  			(7.8125e-2) //refer to row "Voltage"
#define TIME_MULTIPLIER_MIN  			(5.625 / 60.0) //Least Significant Bit= 5.625 seconds, 60 converts it to minutes.

#define	MAX17261_AvgCurrent 	(0x0B) //The AvgCurrent register reports an average of Current register readings
#define	MAX17261_AvgVCell  		(0x19) //The AvgVCell register reports an average of the VCell register readings.
#define	MAX17261_AvgTA  		(0x16)
#define MAX17261_Config			(0x1D)
#define MAX17261_Config2		(0xBB)
#define	MAX17261_Current  		(0x0A) //Voltage between the CSP and CSN pins, and would need to convert to current
#define	MAX17261_DesignCap  	(0x18) //Capacity of battery inserted, not typically used for user requested capacity
#define MAX17261_DieTemp		(0x34)
#define MAX17261_FStat			(0x3D)
#define MAX17261_HibCfg			(0xBA)
#define MAX17261_MaxMinVolt		(0x1B)
#define MAX17261_MaxMinCurr		(0x1C)
#define MAX17261_MaxMinTemp		(0x1A)
#define MAX17261_ModelCFG		(0xDB)
#define	MAX17261_RepCAP  		(0x05) //Reported Capacity. Refer to page 23 and 13 of AN6358
#define	MAX17261_FullRepCAP  	(0x10) //Reported Full Capacity
#define	MAX17261_RepSOC  		(0x06) //The Reported State of Charge of connected battery. Refer to AN6358 page 23 and 13
#define MAX17261_SoftWakeup		(0x60)
#define	MAX17261_Status			(0x00) //Maintains all flags related to alert thresholds and battery insertion or removal.
#define	MAX17261_Temp			(0x08)
#define	MAX17261_TimeToEmpty	(0x11) //How long before battery is empty (in ms). Refer to page 24 and 13 of AN6358
#define	MAX17261_VCell  		(0x09) //VCell reports the voltage measured between BATT and CSP.
#define MAX17261_TTE			(0x11)
#define MAX17261_IChgTerm		(0x1E)
#define MAX17261_VEmpty			(0x3A)

//#define MAX17261_BIT_Tsel	(1)
//#define MAX17261_BIT_SS		(0)
//#define MAX17261_BIT_ETHERM	(1)
//
//#define MAX17261_Config_Val (MAX17261_BIT_Tsel << 15) |(MAX17261_BIT_SS << 14) | (MAX17261_BIT_ETHERM << 5)

typedef uint8_t (*max17261_write)(uint8_t reg, uint16_t value);
typedef uint8_t (*max17261_read)(uint8_t reg, uint16_t *value);
typedef uint8_t (*max17261_delay)(uint32_t period);

struct max17261_conf {
#ifndef MAX17261_USE_WEAK
	max17261_read read;
	max17261_write write;
	max17261_write write_verify;
	max17261_delay delay_ms;
#endif
	uint16_t HibCFG;
	uint16_t DesignCap;		//!< Design capacity in mAh
	uint16_t IchgTerm;		//!< Charge termination current in mA
	uint16_t VEmpty;		//!< Empty voltage values
	uint16_t ChargeVoltage; //!< Charge voltage in mV
	uint8_t	 force_init;	//!< Force initialization
	uint8_t	 R100;			//!< Thermistor value setting. 0 = 10k, 1 = 100k
};

uint8_t
max17261_init(struct max17261_conf *conf);
uint16_t
max17261_get_reported_capacity(struct max17261_conf *conf);
uint16_t
max17261_get_voltage(struct max17261_conf *conf);
uint16_t
max17261_get_average_voltage(struct max17261_conf *conf);
int16_t
max17261_get_current(struct max17261_conf *conf);
uint8_t
max17261_get_SOC(struct max17261_conf *conf);
void
max17261_reset_minmax_voltage(struct max17261_conf *conf);
void
max17261_get_minmax_voltage(struct max17261_conf *conf, uint16_t *min,
                            uint16_t *max);
void
max17261_set_reported_capacity(struct max17261_conf *conf, uint16_t capacity);
uint16_t
max17261_get_reported_capacity(struct max17261_conf *conf);
void
max17261_set_full_reported_capacity(struct max17261_conf *conf,
                                    uint16_t capacity);
uint16_t
max17261_get_full_reported_capacity(struct max17261_conf *conf);
uint16_t
max17261_get_design_capacity(struct max17261_conf *conf);
void
max17261_set_design_capacity(struct max17261_conf *conf, uint16_t capacity);
int16_t
max17261_get_average_current(struct max17261_conf *conf);
void
max17261_reset_minmax_current(struct max17261_conf *conf);
void
max17261_get_minmax_current(struct max17261_conf *conf, int16_t *min,
                            int16_t *max);
int8_t
max17261_get_temperature(struct max17261_conf *conf);
int8_t
max17261_get_average_temperature(struct max17261_conf *conf);
int8_t
max17261_get_die_temperature(struct max17261_conf *conf);
uint16_t
max17261_get_TTE(struct max17261_conf *conf);
uint8_t
max17261_read_word(struct max17261_conf *conf, uint8_t reg, uint16_t *value);
uint8_t
max17261_write_word(struct max17261_conf *conf, uint8_t reg, uint16_t value);
uint8_t
max17261_write_verify(struct max17261_conf *conf, uint8_t reg, uint16_t value);
uint8_t
max17261_delay_ms(struct max17261_conf *conf, uint32_t period);
#endif

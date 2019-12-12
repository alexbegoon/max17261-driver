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

#define MAX17261_ADDRESS	0x6C
#define	MAX17261_RB_THRESHOLD	3

float r_sense = 0.01;
#define CAPACITY_MULTIPLIER_mAH = (5e-3)/r_sense; //refer to row "Capacity"
#define CURRENT_MULTIPLIER_mV = (1.5625e-3)/r_sense; //refer to row "Current"
#define VOLTAGE_MULTIPLIER_V = 7.8125e-5; //refer to row "Voltage"
#define TIME_MULTIPLIER_MIN = 5.625/60.0; //Least Significant Bit= 5.625 seconds, 3600 converts it to Hours. refer to AN6358 pg 13 figure 1.3 in row "Time"
#define PERCENTAGE_MULTIPLIER = 1.0/256.0; //refer to row "Percentage"

enum MAX17261_RegAddr {
	Status = 0x00, //Maintains all flags related to alert thresholds and battery insertion or removal.
	VCell = 0x09, //VCell reports the voltage measured between BATT and CSP.
	AvgVCell = 0x19, //The AvgVCell register reports an average of the VCell register readings.
	Current = 0x0A, //Voltage between the CSP and CSN pins, and would need to convert to current
	AvgCurrent = 0x0B, //The AvgCurrent register reports an average of Current register readings
	RepSOC = 0x06, //The Reported State of Charge of connected battery. Refer to AN6358 page 23 and 13
	RepCAP = 0x05, //Reported Capacity. Refer to page 23 and 13 of AN6358.
	TimeToEmpty = 0x11, //How long before battery is empty (in ms). Refer to page 24 and 13 of AN6358
	DesignCap = 0x18, //Capacity of battery inserted, not typically used for user requested capacity
};

typedef uint8_t (*max17261_write)(uint8_t reg, uint16_t value);
typedef uint8_t (*max17261_read)(uint8_t reg, uint16_t *value);
typedef uint8_t (*max17261_delay)(uint32_t period);

struct max17261_conf {
	max17261_read read;
	max17261_write write;
	max17261_write write_verify;
	max17261_delay delay_ms;
	uint16_t HibCFG;
	uint16_t DesignCap;
	uint16_t IchgTerm;
	uint16_t VEmpty;
	uint16_t ChargeVoltage; // charge voltage in millivolts
};


#endif

/*
 *  @file:  max17261.c
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
#include "max17261.h"

uint8_t
max17261_init(struct max17261_conf *conf)
{
	uint8_t ret = 0;
	uint16_t value;

	// check for power on reset
	ret = conf->read(MAX17261_Status, &value);
	if ((value & 0x0002) != 0) {
		// Delay until FSTAT.DNR bit == 0
		ret |= conf->read(MAX17261_FStat, &value);
		while (value & 1) {
			conf->delay_ms(10);
			ret |= conf->read(MAX17261_FStat, &value);
		}
		// Initialize Configuration
		ret |= conf->read(MAX17261_HibCfg, &conf->HibCFG); //Store original HibCFG value
		ret |= conf->write(MAX17261_SoftWakeup, 0x90); // Exit Hibernate Mode step 1
		ret |= conf->write(MAX17261_HibCfg, 0x0); // Exit Hibernate Mode step 2
		ret |= conf->write(MAX17261_SoftWakeup, 0x0); // Exit Hibernate Mode step 3

		ret |= conf->write(MAX17261_DesignCap, conf->DesignCap);
		ret |= conf->write(MAX17261_DesignCap, conf->IchgTerm);
		ret |= conf->write(MAX17261_DesignCap, conf->VEmpty);

		if (conf->ChargeVoltage > 4.275)
			ret |= conf->write(MAX17261_ModelCFG, 0x8400) ;   // Write ModelCFG
		else
			ret |= conf->write(MAX17261_ModelCFG, 0x8000) ;   // Write ModelCFG
		//Poll ModelCFG.Refresh(highest bit),
		ret |= conf->read(0xDB, &value);
		while (value & 0x8000) { //do not continue until ModelCFG.Refresh==0
			conf->delay_ms(10);
			ret |= conf->read(MAX17261_ModelCFG, &value);
		}

		ret |= conf->write(MAX17261_HibCfg,
		                   conf->HibCFG) ;   // Restore Original HibCFG value
		// Initialization complete
		ret |= conf->read(MAX17261_Status, &value); //Read Status
		ret |= conf->write_verify(MAX17261_Status, value
		                          && 0xFFFD); //Write and Verify Status with POR bit Cleared
		ret |= conf->write(MAX17261_Config, 0x2210);
	}

	return ret;
}

uint8_t
max17261_get_SOC(struct max17261_conf *conf)
{
	uint16_t value;
	max17261_init(conf);
	conf->read(MAX17261_RepSOC, &value);
	return (uint8_t)(value >> 8);
}

uint16_t
max17261_get_reported_capacity(struct max17261_conf *conf)
{
	uint16_t value;
	max17261_init(conf);
	conf->read(MAX17261_RepCAP, &value);
	return value;
}

void
max17261_set_design_capacity(struct max17261_conf *conf, uint16_t capacity)
{
	conf->write(MAX17261_DesignCap, capacity);
}

uint16_t
max17261_get_design_capacity(struct max17261_conf *conf)
{
	uint16_t value;
	max17261_init(conf);
	conf->read(MAX17261_DesignCap, &value);
	return value;
}

uint16_t
max17261_get_voltage(struct max17261_conf *conf)
{
	uint16_t value;
	max17261_init(conf);
	conf->read(MAX17261_VCell, &value);
	value *= VOLTAGE_MULTIPLIER_V;
	return value;
}

uint16_t
max17261_get_average_voltage(struct max17261_conf *conf)
{
	uint16_t value;
	max17261_init(conf);
	conf->read(MAX17261_AvgVCell, &value);
	value *= VOLTAGE_MULTIPLIER_V;
	return value;
}

void
max17261_get_minmax_voltage(struct max17261_conf *conf, uint16_t *min,
                            uint16_t *max)
{
	uint16_t value;
	max17261_init(conf);
	conf->read(MAX17261_MaxMinVolt, &value);
	*min = (value & 0xFF) * 20;
	*max = ((value >> 8) & 0xFF) * 20;
}

int16_t
max17261_get_current(struct max17261_conf *conf)
{
	int16_t value;
	max17261_init(conf);
	conf->read(MAX17261_Current, (uint16_t *) &value);
	value = value * CURRENT_MULTIPLIER;
	return value;
}

int16_t
max17261_get_average_current(struct max17261_conf *conf)
{
	int16_t value;
	max17261_init(conf);
	conf->read(MAX17261_AvgCurrent, (uint16_t *) &value);
	value = value * CURRENT_MULTIPLIER;
	return value;
}

void
max17261_get_minmax_current(struct max17261_conf *conf, int16_t *min,
                            int16_t *max)
{
	uint16_t value;
	max17261_init(conf);
	conf->read(MAX17261_MaxMinCurr, &value);
	*min = ((int8_t)(value & 0xFF)) * CURRENT_MULTIPLIER_MINMAX;
	*max = ((int8_t)(value >> 8)) * CURRENT_MULTIPLIER_MINMAX;
}

int8_t
max17261_get_die_temperature(struct max17261_conf *conf)
{
	int16_t value;
	max17261_init(conf);
	conf->read(MAX17261_DieTemp, (uint16_t *) &value);
	value >>= 8;
	return value;
}

int8_t
max17261_get_temperature(struct max17261_conf *conf)
{
	int16_t value;
	max17261_init(conf);
	conf->read(MAX17261_Temp, (uint16_t *) &value);
	value >>= 8;
	return value;
}

int8_t
max17261_get_average_temperature(struct max17261_conf *conf)
{
	int16_t value;
	max17261_init(conf);
	conf->read(MAX17261_AvgTA, (uint16_t *) &value);
	value >>= 8;
	return value;
}

void
max17261_get_minmax_temperature(struct max17261_conf *conf, int8_t *min,
                                int8_t *max)
{
	uint16_t value;
	max17261_init(conf);
	conf->read(MAX17261_MaxMinTemp, &value);
	*min = (value & 0xFF) * CURRENT_MULTIPLIER_MINMAX;
	*max = ((value >> 8) & 0xFF) * CURRENT_MULTIPLIER_MINMAX;
}

uint16_t
max17261_get_TTE(struct max17261_conf *conf)
{
	uint16_t value;
	max17261_init(conf);
	conf->read(MAX17261_TTE, &value);
	return value * TIME_MULTIPLIER_MIN;
}

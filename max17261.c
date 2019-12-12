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
	uint8_t ret;
	uint16_t value;

	// check for power on reset
	ret = conf->read(Status, &value) & 0x0002;
	if (value != 0) {
		// Delay until FSTAT.DNR bit == 0
		ret = conf->read(0x3D, &value);
		while (value & 1) {
			conf->delay_ms(10);
			ret = conf->read(0x3D, &value);
		}
		// Initialize Configuration
		conf->read(0xBA, &conf->HibCFG); //Store original HibCFG value
		conf->write(0x60, 0x90); // Exit Hibernate Mode step 1
		conf->write(0xBA, 0x0); // Exit Hibernate Mode step 2
		conf->write(0x60, 0x0); // Exit Hibernate Mode step 3

		if (conf->ChargeVoltage > 4.275)
			conf->write(0xDB, 0x8400) ;   // Write ModelCFG
		else
			conf->write(0xDB, 0x8000) ;   // Write ModelCFG
		//Poll ModelCFG.Refresh(highest bit),
		conf->read(0xDB, &value);
		while (value & 0x8000) { //do not continue until ModelCFG.Refresh==0
			conf->delay_ms(10);
			conf->read(0xDB, &value);
		}

		conf->write(0xBA, conf->HibCFG) ;   // Restore Original HibCFG value
		// Initialization complete
		ret = conf->read(Status, &value); //Read Status
		conf->write_verify(Status, value
		                   && 0xFFFD); //Write and Verify Status with POR bit Cleared

	}

	return 0;
}

uint16_t
max17261_get_state_of_charge(struct max17261_conf *conf)
{
	uint16_t value;
	max17261_init(conf);
	conf->read(RepSOC, &value);
	return value;
}

uint16_t
max17261_get_reported_capacity(struct max17261_conf *conf)
{
	uint16_t value;
	max17261_init(conf);
	conf->read(RepCAP, &value);
	return value;
}

void max17261_set_design_capacity(uint16_t capacity) {

}

void max17261_get_design_capacity() {

}

/*float max17261_get_instantaneous_current()
{
   	int16_t current_raw = readReg16Bit(Current);
	return current_raw * current_multiplier_mV;
}

float max17261_get_instantaneous_voltage()
{
   	uint16_t voltage_raw = readReg16Bit(VCell);
	return voltage_raw * voltage_multiplier_V;
}

float max17261_get_time_to_empty()
{
	uint16_t TTE_raw = readReg16Bit(TimeToEmpty);
	return TTE_raw * time_multiplier_Hours;
}*/


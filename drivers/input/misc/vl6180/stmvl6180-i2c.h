/*
 *  stmvl6180-i2c.h - Linux kernel modules for STM VL6180 FlightSense TOF sensor
 *
 *  Copyright (c) 2017, Fuzhou Rockchip Electronics Co., Ltd
 *  Copyright (c) 2015 STMicroelectronics Imaging Division
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */
/*
 * Defines
 */
#ifndef STMVL6180_I2C_H
#define STMVL6180_I2C_H

struct i2c_data {
	struct i2c_client *client;
	struct regulator *vana;
	uint8_t power_up;
};

int stmvl6180_init_i2c(void);
void stmvl6180_exit_i2c(void *i2c_object);
int stmvl6180_power_up_i2c(void *i2c_object, unsigned int *preset_flag);
int stmvl6180_power_down_i2c(void *i2c_object);

#endif /* STMVL6180_I2C_H */

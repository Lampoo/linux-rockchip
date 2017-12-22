/*
 *  vl6180x_port_i2c.c- Linux kernel modules for STM VL6180 FlightSense TOF sensor
 *
 *  Copyright (c) 2017, Fuzhou Rockchip Electronics Co., Ltd
 *  Copyright (c) 2015 STMicroelectronics Imaging Division.
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

#include "vl6180x_i2c.h"
#include <linux/i2c.h>
#include <linux/module.h>
#include "stmvl6180-i2c.h"

#define I2C_M_WR			0x00
static struct i2c_client *pclient;

void i2c_setclient(void *client)
{
	pclient = (struct i2c_client *)client;
}

void *i2c_getclient(void)
{
	return (void *)pclient;
}

/** int  VL6180x_I2CWrite(VL6180xDev_t dev, void *buff, uint8_t len);
 * @brief       Write data buffer to VL6180x device via i2c
 * @param dev   The device to write to
 * @param buff  The data buffer
 * @param len   The length of the transaction in byte
 * @return      0 on success
 */
int VL6180x_I2CWrite(VL6180xDev_t dev, uint8_t *buff, uint8_t len)
{
	int err = 0;
	struct i2c_msg msg[1];

	msg[0].addr = pclient->addr;
	msg[0].flags = I2C_M_WR;
	msg[0].buf = buff;
	msg[0].len = len;

	err = i2c_transfer(pclient->adapter, msg, 1); /* return the actual messages transfer */
	if (err != 1) {
		pr_err("%s: i2c_transfer err:%d, addr:0x%x, reg:0x%x\n", __func__, err, pclient->addr, (buff[0] << 8 | buff[1]));
		return -1;
	}

	return 0;
}

/** int VL6180x_I2CRead(VL6180xDev_t dev, void *buff, uint8_t len);
 * @brief       Read data buffer from VL6180x device via i2c
 * @param dev   The device to read from
 * @param buff  The data buffer to fill
 * @param len   The length of the transaction in byte
 * @return      transaction status
 */
int VL6180x_I2CRead(VL6180xDev_t dev, uint8_t *buff, uint8_t len)
{
	int err = 0;
	struct i2c_msg msg[1];

	msg[0].addr = pclient->addr;
	msg[0].flags = I2C_M_RD | pclient->flags;
	msg[0].buf = buff;
	msg[0].len = len;

	err = i2c_transfer(pclient->adapter, &msg[0], 1); /* return the actual mesage transfer */
	if (err != 1) {
		pr_err("%s: Read i2c_transfer err:%d, addr:0x%x\n", __func__, err, pclient->addr);
		return -1;
	}

	return 0;
}

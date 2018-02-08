/* drivers/input/sensors/accel/lis2ds_acc.c
 *
 * Copyright (C) 2018 Fuzhou Rockchip Electronics Co., Ltd.
 * Author: Dayao Ji<jdy@rock-chips.com>
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
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/of_gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/sensor-dev.h>
#include <linux/bitops.h>

#define LIS2DS_WHO_AM_I						(0x0F)

/* full scale setting - register & mask */
#define LIS2DS_CTRL_REG1					(0x20)
#define LIS2DS_CTRL_REG2					(0x21)
#define LIS2DS_CTRL_REG3					(0x22)
#define LIS2DS_CTRL_REG4					(0x23)
#define LIS2DS_CTRL_REG5					(0x24)
#define LIS2DS_OUT_T						(0x26)
#define LIS2DS_STATUS_REG					(0x27)
#define LIS2DS_OUT_X_L						(0x28)
#define LIS2DS_OUT_X_H						(0x29)
#define LIS2DS_OUT_Y_L						(0x2a)
#define LIS2DS_OUT_Y_H						(0x2b)
#define LIS2DS_OUT_Z_L						(0x2c)
#define LIS2DS_OUT_Z_H						(0x2d)
#define LIS2DS_FIFO_CTRL_REG				(0x25)
#define LIS2DS_TAP_6D_THS					(0x31)
#define LIS2DS_INT_DUR						(0x32)
#define LIS2DS_WAKE_UP_THS					(0x33)
#define LIS2DS_WAKE_UP_DUR					(0x34)
#define LIS2DS_FREE_FALL					(0x35)
#define LIS2DS_STATUS_DUP					(0x36)
#define LIS2DS_WAKE_UP_SRC					(0x37)
#define LIS2DS_TAP_SRC						(0x38)
#define LIS2DS_6D_SRC						(0x39)

#define LIS2DS_DEVID						(0x43)
#define LIS2DS_ACC_DISABLE					(0x01)

#define LIS2DS_RANGE						(2000000)

#define LIS2DS_PRECISION					(16)
#define LIS2DS_BOUNDARY						(0x1 << (LIS2DS_PRECISION - 1))
#define LIS2DS_GRAVITY_STEP					(LIS2DS_RANGE / LIS2DS_BOUNDARY)

#define LIS2DS_CTRL1_ODR_0HZ				(0x00)
#define LIS2DS_CTRL1_ODR_12_5HZ				(0x01)
#define LIS2DS_CTRL1_ODR_25HZ				(0x02)
#define LIS2DS_CTRL1_ODR_50HZ				(0x03)
#define LIS2DS_CTRL1_ODR_100HZ				(0x04)
#define LIS2DS_CTRL1_ODR_200HZ				(0x05)
#define LIS2DS_CTRL1_ODR_400HZ				(0x06)
#define LIS2DS_CTRL1_ODR_800HZ				(0x07)

#define LIS2DS_CTRL1_FS_2G					(0x00)
#define LIS2DS_CTRL1_FS_4G					(0x02)
#define LIS2DS_CTRL1_FS_8G					(0x03)
#define LIS2DS_CTRL1_FS_16G					(0x01)

#define LIS2DS12_CTRL1_MASK_FS				(0x0C)
#define LIS2DS12_CTRL1_MASK_ODR				(0xF0)

#define LIS2DS12_INT_ACTIVE_MASK			(0x02)
#define LIS2DS12_INT2_ON_INT1_MASK			(0x20)

#define LIS2DS12_EN							(0x01)
#define LIS2DS12_DIS						(0x00)

struct sensor_reg_data {
	char reg;
	char data;
};

/****************operate according to sensor chip:start************/
static int sensor_active(struct i2c_client *client, int enable, int rate)
{
	struct sensor_private_data *sensor = (struct sensor_private_data *)i2c_get_clientdata(client);
	int result = 0;
	int new_data = 0x00, old_data = 0x00;

	sensor->ops->ctrl_data = sensor_read_reg(client, sensor->ops->ctrl_reg);
	old_data = sensor->ops->ctrl_data;

	if (enable) {
		new_data = ((old_data & (~LIS2DS12_CTRL1_MASK_ODR)) | ((LIS2DS_CTRL1_ODR_100HZ << __ffs(LIS2DS12_CTRL1_MASK_ODR)) & LIS2DS12_CTRL1_MASK_ODR));
	} else {
		new_data = ((old_data & (~LIS2DS12_CTRL1_MASK_ODR)) | ((LIS2DS_CTRL1_ODR_0HZ << __ffs(LIS2DS12_CTRL1_MASK_ODR)) & LIS2DS12_CTRL1_MASK_ODR));
	}

	sensor->ops->ctrl_data = new_data;

	DBG("%s:reg=0x%x,reg_ctrl=0x%x,enable=%d\n", __func__, sensor->ops->ctrl_reg, sensor->ops->ctrl_data, enable);
	result = sensor_write_reg(client, sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
	if (result) {
		printk(KERN_ERR "%s:fail to active sensor result=%d ctrl_data=%d \n", __func__, result, sensor->ops->ctrl_data);
	} else {
		if (sensor->pdata->irq_enable) {
			int status = 0;

			/*INT1_DRDY*/
			status = sensor_read_reg(client, LIS2DS_CTRL_REG4);
			status |= 0x01;
			result = sensor_write_reg(client, LIS2DS_CTRL_REG4, status);
			if (result) {
				printk(KERN_ERR "%s:line=%d,error\n", __func__, __LINE__);
				return result;
			}
		}
	}

	return result;
}

static int sensor_init(struct i2c_client *client)
{
	struct sensor_private_data *sensor = (struct sensor_private_data *)i2c_get_clientdata(client);
	int result = 0;
	int status;

	result = sensor->ops->active(client, 0, 0);
	if (result) {
		printk(KERN_ERR "%s:line=%d,error\n", __func__, __LINE__);
		return result;
	}

	sensor->status_cur = SENSOR_OFF;

	if (sensor->pdata->irq_enable) {
		/*INT1_DRDY*/
		status = sensor_read_reg(client, LIS2DS_CTRL_REG4);
		status |= 0x01;
		result = sensor_write_reg(client, LIS2DS_CTRL_REG4, status);
		if (result) {
			printk(KERN_ERR "%s:line=%d,error\n", __func__, __LINE__);
			return result;
		}
	}

	return result;
}

static int sensor_convert_data(struct i2c_client *client, char high_byte, char low_byte)
{
	s64 result;
	struct sensor_private_data *sensor = (struct sensor_private_data *)i2c_get_clientdata(client);

	switch (sensor->devid) {
	case LIS2DS_DEVID:
		result = ((int)high_byte << 8) | (int)low_byte;
		if (result < LIS2DS_BOUNDARY)
			result = result * LIS2DS_GRAVITY_STEP;
		else
			result = ~(((~result & (0x7fff >> (16 - LIS2DS_PRECISION))) + 1) * LIS2DS_GRAVITY_STEP) + 1;
		break;

	default:
		printk(KERN_ERR "%s: devid wasn't set correctly\n", __func__);
		return -EFAULT;
    }

    return (int)result;
}

static int gsensor_report_value(struct i2c_client *client, struct sensor_axis *axis)
{
	struct sensor_private_data *sensor = (struct sensor_private_data *)i2c_get_clientdata(client);

	input_report_abs(sensor->input_dev, ABS_X, axis->x);
	input_report_abs(sensor->input_dev, ABS_Y, axis->y);
	input_report_abs(sensor->input_dev, ABS_Z, axis->z);
	input_sync(sensor->input_dev);
	DBG("Gsensor x==%d  y==%d z==%d\n", axis->x, axis->y, axis->z);

	return 0;
}

#define GSENSOR_MIN  10
static int sensor_report_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor = (struct sensor_private_data *)i2c_get_clientdata(client);
	struct sensor_platform_data *pdata = sensor->pdata;
	int ret = 0;
	int x, y, z;
	struct sensor_axis axis;
	char buffer[6] = {0};
	char value = 0;

	if (sensor->ops->read_len < 6) {
		printk(KERN_ERR "%s:lenth is error,len=%d\n", __func__, sensor->ops->read_len);
		return -1;
	}

	memset(buffer, 0, 6);

	value = sensor_read_reg(client, LIS2DS_STATUS_REG);
	if ((value & 0x0f) == 0) {
		printk(KERN_ERR "%s:line=%d,value=0x%x,data is not ready\n", __func__, __LINE__, value);
		return -1;
	}

	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	do {
		*buffer = sensor->ops->read_reg;
		ret = sensor_rx_data(client, buffer, sensor->ops->read_len);
		if (ret < 0)
			return ret;
	} while (0);

	x = sensor_convert_data(sensor->client, buffer[1], buffer[0]);
	y = sensor_convert_data(sensor->client, buffer[3], buffer[2]);
	z = sensor_convert_data(sensor->client, buffer[5], buffer[4]);

	axis.x = (pdata->orientation[0]) * x + (pdata->orientation[1]) * y + (pdata->orientation[2]) * z;
	axis.y = (pdata->orientation[3]) * x + (pdata->orientation[4]) * y + (pdata->orientation[5]) * z;
	axis.z = (pdata->orientation[6]) * x + (pdata->orientation[7]) * y + (pdata->orientation[8]) * z;

	DBG("%s: axis = %d  %d  %d \n", __func__, axis.x, axis.y, axis.z);

	if ((abs(sensor->axis.x - axis.x) > GSENSOR_MIN) || (abs(sensor->axis.y - axis.y) > GSENSOR_MIN) || (abs(sensor->axis.z - axis.z) > GSENSOR_MIN)) {
		gsensor_report_value(client, &axis);

		mutex_lock(&(sensor->data_mutex));
		sensor->axis = axis;
		mutex_unlock(&(sensor->data_mutex));
	}

	if ((sensor->pdata->irq_enable) && (sensor->ops->int_status_reg >= 0)) {
		value = sensor_read_reg(client, sensor->ops->int_status_reg);
		DBG("%s:sensor int status :0x%x\n", __func__, value);
	}

	return ret;
}

struct sensor_operate gsensor_lis2ds_ops = {
	.name				= "lis2ds_acc",
	.type				= SENSOR_TYPE_ACCEL,
	.id_i2c				= ACCEL_ID_LIS2DS,
	.read_reg			= (LIS2DS_OUT_X_L | 0x80),
	.read_len			= 6,
	.id_reg				= LIS2DS_WHO_AM_I,
	.id_data			= LIS2DS_DEVID,
	.precision			= LIS2DS_PRECISION,
	.ctrl_reg			= LIS2DS_CTRL_REG1,
	.int_status_reg		= SENSOR_UNKNOW_DATA,
	.range				= {-LIS2DS_RANGE, LIS2DS_RANGE},
	.trig				= IRQF_TRIGGER_HIGH | IRQF_ONESHOT | IRQF_SHARED,
	.active				= sensor_active,
	.init				= sensor_init,
	.report				= sensor_report_value,
};

/****************operate according to sensor chip:end************/

//function name should not be changed
static struct sensor_operate *gsensor_get_ops(void)
{
	return &gsensor_lis2ds_ops;
}

static int __init gsensor_lis2ds_init(void)
{
	struct sensor_operate *ops = gsensor_get_ops();
	int result = 0;
	int type = ops->type;

	result = sensor_register_slave(type, NULL, NULL, gsensor_get_ops);

	return result;
}

static void __exit gsensor_lis2ds_exit(void)
{
	struct sensor_operate *ops = gsensor_get_ops();
	int type = ops->type;

	sensor_unregister_slave(type, NULL, NULL, gsensor_get_ops);
}

module_init(gsensor_lis2ds_init);
module_exit(gsensor_lis2ds_exit);

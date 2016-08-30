/*
 * Copyright (C) 2015 Fuzhou Rockchip Electronics Co., Ltd
 * author: chenhengming chm@rock-chips.com
 *	   Alpha Lin, alpha.lin@rock-chips.com
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

#ifndef __ROCKCHIP_MPP_SERVICE_H
#define __ROCKCHIP_MPP_SERVICE_H

#include <linux/ioctl.h>    /* needed for the _IOW etc stuff used later */

/*
 * Ioctl definitions
 */

/* Use 'l' as magic number */
#define MPP_IOC_MAGIC			'l'

#define MPP_IOC_SET_CLIENT_TYPE		_IOW(MPP_IOC_MAGIC, 1, u32)
#define MPP_IOC_GET_HW_FUSE_STATUS	_IOW(MPP_IOC_MAGIC, 2, u32)

#define MPP_IOC_SET_REG			_IOW(MPP_IOC_MAGIC, 3, u32)
#define MPP_IOC_GET_REG			_IOW(MPP_IOC_MAGIC, 4, u32)

#define MPP_IOC_PROBE_IOMMU_STATUS	_IOR(MPP_IOC_MAGIC, 5, u32)

#endif


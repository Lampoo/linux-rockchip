/*
 *  drivers/mfd/rt5025-debug.c
 *  Driver foo Richtek RT5025 PMIC Debug
 *
 *  Copyright (C) 2013 Richtek Electronics
 *  cy_huang <cy_huang@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/string.h>

#include <linux/mfd/rt5025.h>

struct rt5025_debug_info {
	struct i2c_client *i2c;
};

static struct i2c_client *client;
static struct dentry *debugfs_rt_dent;
static struct dentry *debugfs_peek;
static struct dentry *debugfs_poke;

static unsigned char read_data;

static int reg_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static int get_parameters(char *buf, long int *param1, int num_of_par)
{
	char *token;
	int base, cnt;

	token = strsep(&buf, " ");

	for (cnt = 0; cnt < num_of_par; cnt++) {
		if (token != NULL) {
			if ((token[1] == 'x') || (token[1] == 'X'))
				base = 16;
			else
				base = 10;

			if (strict_strtoul(token, base, &param1[cnt]) != 0)
				return -EINVAL;

			token = strsep(&buf, " ");
			}
		else
			return -EINVAL;
	}
	return 0;
}

static ssize_t reg_debug_read(struct file *filp, char __user *ubuf,
				size_t count, loff_t *ppos)
{
	char lbuf[8];
	
	snprintf(lbuf, sizeof(lbuf), "0x%x\n", read_data);
	return simple_read_from_buffer(ubuf, count, ppos, lbuf, strlen(lbuf));
}

static ssize_t reg_debug_write(struct file *filp,
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	char *access_str = filp->private_data;
	char lbuf[32];
	int rc;
	long int param[5];

	if (cnt > sizeof(lbuf) - 1)
		return -EINVAL;

	rc = copy_from_user(lbuf, ubuf, cnt);
	if (rc)
		return -EFAULT;

	lbuf[cnt] = '\0';

	if (!strcmp(access_str, "poke")) {
		/* write */
		rc = get_parameters(lbuf, param, 2);
		if ((param[0] <= 0xFF) && (param[1] <= 0xFF) && (rc == 0))
		{
			rt5025_reg_write(client, param[0], (unsigned char)param[1]);
		}
		else
			rc = -EINVAL;
	} else if (!strcmp(access_str, "peek")) {
		/* read */
		rc = get_parameters(lbuf, param, 1);
		if ((param[0] <= 0xFF) && (rc == 0))
		{
			read_data = rt5025_reg_read(client, param[0]);
		}
		else
			rc = -EINVAL;
	}

	if (rc == 0)
		rc = cnt;

	return rc;
}

static const struct file_operations reg_debug_ops = {
	.open = reg_debug_open,
	.write = reg_debug_write,
	.read = reg_debug_read
};

static int __devinit rt5025_debug_probe(struct platform_device *pdev)
{
	struct rt5025_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct rt5025_debug_info *di;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	di->i2c = chip->i2c;

	RTINFO("add debugfs for core RT5025");
	client = chip->i2c;
	debugfs_rt_dent = debugfs_create_dir("rt5025_dbg", 0);
	if (!IS_ERR(debugfs_rt_dent)) {
		debugfs_peek = debugfs_create_file("peek",
		S_IFREG | S_IRUGO, debugfs_rt_dent,
		(void *) "peek", &reg_debug_ops);

		debugfs_poke = debugfs_create_file("poke",
		S_IFREG | S_IRUGO, debugfs_rt_dent,
		(void *) "poke", &reg_debug_ops);
	}

	platform_set_drvdata(pdev, di);

	return 0;
}

static int __devexit rt5025_debug_remove(struct platform_device *pdev)
{
	struct rt5025_debug_info *di = platform_get_drvdata(pdev);

	if (!IS_ERR(debugfs_rt_dent))
		debugfs_remove_recursive(debugfs_rt_dent);

	kfree(di);
	return 0;
}

static struct platform_driver rt5025_debug_driver = 
{
	.driver = {
		.name = RT5025_DEVICE_NAME "-debug",
		.owner = THIS_MODULE,
	},
	.probe = rt5025_debug_probe,
	.remove = __devexit_p(rt5025_debug_remove),
};

static int __init rt5025_debug_init(void)
{
	return platform_driver_register(&rt5025_debug_driver);
}
subsys_initcall_sync(rt5025_debug_init);

static void __exit rt5025_debug_exit(void)
{
	platform_driver_unregister(&rt5025_debug_driver);
}
module_exit(rt5025_debug_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("CY Huang <cy_huang@richtek.com");
MODULE_DESCRIPTION("Debug driver for RT5025");
MODULE_ALIAS("platform:" RT5025_DEVICE_NAME "-debug");
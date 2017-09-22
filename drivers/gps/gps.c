#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>

#define GPS_DEV_IOC_MAGIC   'g'
#define GPS_OPEN  _IO(GPS_DEV_IOC_MAGIC, 1)
#define GPS_CLOSE _IO(GPS_DEV_IOC_MAGIC, 2)

static int gps_pwr_major;
static int gps_pwr_gpio = -1;
static struct class *gps_pwr_class;
static struct device *gps_pwr_device;

static int gps_pwr_open(struct inode *inode, struct file *file)
{
	if (!gpio_is_valid(gps_pwr_gpio))
		return -1;

	if (gpio_request(gps_pwr_gpio, "GPS,pwr_en"))
		goto err;

	return 0;

err:
	gpio_free(gps_pwr_gpio);
	return -ENODEV;
}

static long gps_pwr_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	switch (cmd) {
	case GPS_OPEN:
		gpio_set_value(gps_pwr_gpio, 1);
		if (gpio_direction_output(gps_pwr_gpio, 1))
			return -1;

		break;

	case GPS_CLOSE:
		if (gpio_direction_output(gps_pwr_gpio, 0))
			return -1;

		gpio_set_value(gps_pwr_gpio, 0);
		break;

	default:
		return -1;
	}

	return 0;
}

static int gps_pwr_release(struct inode *inode, struct file *file)
{
	gpio_direction_output(gps_pwr_gpio, 0);
	gpio_free(gps_pwr_gpio);
	return 0;
}

static const struct file_operations gps_pwr_fops = {
	.owner = THIS_MODULE,
	.open  = gps_pwr_open,
	.release = gps_pwr_release,
	.unlocked_ioctl = gps_pwr_ioctl,
};

static int gps_pwr_probe(struct platform_device *pdev)
{
	enum of_gpio_flags flags;
	struct device_node *GPS_pwr_node = pdev->dev.of_node;

	gps_pwr_gpio = of_get_named_gpio_flags(GPS_pwr_node,
					       "GPS,pwr_en",
					       0,
					       &flags);

	if (gps_pwr_gpio < 0)
		return -1;

	gps_pwr_major = register_chrdev(0, "GPS_pwr", &gps_pwr_fops);
	gps_pwr_class = class_create(THIS_MODULE, "GPS_pwr");
	gps_pwr_device = device_create(gps_pwr_class,
				       NULL, MKDEV(gps_pwr_major, 0),
				       NULL, "GPS");

	return 0;
}

static int gps_pwr_remove(struct platform_device *pdev)
{
	gpio_direction_output(gps_pwr_gpio, 0);
	gpio_free(gps_pwr_gpio);
	unregister_chrdev(gps_pwr_major, "gps_pwr");
	device_unregister(gps_pwr_device);
	class_destroy(gps_pwr_class);

	return 0;
}

static const struct of_device_id of_gps_power_match[] = {
	{ .compatible = "gps_pwr", },
	{},
};

static struct platform_driver gps_gpio_driver = {
	.probe = gps_pwr_probe,
	.remove = gps_pwr_remove,
	.driver = {
		.name = "gps_pwr",
		.owner = THIS_MODULE,
		.of_match_table = of_gps_power_match,
	},
};

module_platform_driver(gps_gpio_driver);

MODULE_AUTHOR("Rockchip TF Chen");
MODULE_DESCRIPTION("GPS power enable");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:gps_pwr");

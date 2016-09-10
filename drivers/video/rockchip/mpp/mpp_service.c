/**
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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/clk.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/mfd/syscon.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>

#include <linux/rockchip_ion.h>
#include <linux/rockchip/cpu.h>
#include <linux/rockchip/iomap.h>

#include <linux/rockchip/grf.h>

#if defined(CONFIG_ROCKCHIP_IOMMU) & defined(CONFIG_ION_ROCKCHIP)
#define CONFIG_VCODEC_MMU
#endif

#include "mpp_dev_common.h"

#include "mpp_service.h"

int debug;
module_param(debug, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "bit switch for mpp_service debug information");

#define VCODEC_CLOCK_ENABLE	1

struct mpp_request {
	u32 *req;
	u32 size;
};

#ifdef CONFIG_COMPAT
struct compat_mpp_request {
	compat_uptr_t req;
	u32 size;
};
#endif

/* debugfs root directory for all mpp device.*/
static struct dentry *parent;

#ifdef CONFIG_DEBUG_FS
static int mpp_debugfs_init(void);
static void mpp_debugfs_exit(void);
static struct dentry *mpp_debugfs_create_device_dir(
		char *dirname, struct dentry *parent);
static int debug_mpp_open(struct inode *inode, struct file *file);

static const struct file_operations debug_mpp_fops = {
	.open = debug_mpp_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif

#define MPP_TIMEOUT_DELAY		(2 * HZ) /* 2s */

static void ctx_deinit(struct rockchip_mpp_dev *data, struct mpp_ctx *reg);
static void mpp_service_session_clear(struct rockchip_mpp_dev *data,
				      struct mpp_session *session)
{
	struct mpp_ctx *reg, *n;

	list_for_each_entry_safe(reg, n, &session->waiting, session_link) {
		ctx_deinit(data, reg);
	}
	list_for_each_entry_safe(reg, n, &session->running, session_link) {
		ctx_deinit(data, reg);
	}
	list_for_each_entry_safe(reg, n, &session->done, session_link) {
		ctx_deinit(data, reg);
	}
}

static void mpp_service_power_off(struct mpp_service *pservice)
{
}

static void mpp_service_power_on(struct mpp_service *pservice)
{
}

static struct mpp_ctx *ctx_init(struct rockchip_mpp_dev *mpp,
				struct mpp_session *session,
				void __user *src, u32 size)
{
	struct mpp_service *pservice = mpp->srv;
	struct mpp_ctx *ctx;

	mpp_debug_enter();

	if (mpp->ops->init)
		ctx = mpp->ops->init(mpp, src, size);
	else
		ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);

	if (NULL == ctx) {
		mpp_err("error: kmalloc fail in reg_init\n");
		return NULL;
	}

	ctx->session = session;

	INIT_LIST_HEAD(&ctx->session_link);
	INIT_LIST_HEAD(&ctx->status_link);

	mutex_lock(&pservice->lock);
	list_add_tail(&ctx->session_link, &session->waiting);
	list_add_tail(&ctx->status_link, &pservice->pending);
	mutex_unlock(&pservice->lock);

	mpp_debug_leave();

	return ctx;
}

static void ctx_deinit(struct rockchip_mpp_dev *mpp, struct mpp_ctx *ctx)
{
	struct mpp_service *pservice = mpp->srv;
	struct mpp_mem_region *mem_region = NULL, *n;

	list_del_init(&ctx->session_link);
	list_del_init(&ctx->status_link);

	/* release memory region attach to this registers table. */
	list_for_each_entry_safe(mem_region, n,
				 &ctx->mem_region_list, reg_lnk) {
		ion_free(pservice->ion_client, mem_region->hdl);
		list_del_init(&mem_region->reg_lnk);
		kfree(mem_region);
	}

	kfree(ctx);
}

static void rockchip_mpp_run(struct rockchip_mpp_dev *mpp)
{
	struct mpp_service *pservice = mpp->srv;
	struct mpp_ctx *ctx = list_entry(pservice->pending.next,
					 struct mpp_ctx, status_link);

	mpp_debug_enter();

	set_bit(HW_RUNNING, &mpp->state);

	list_del_init(&ctx->status_link);
	pservice->current_ctx = ctx;

	mpp_time_record(ctx);
	if (mpp->ops->run)
		mpp->ops->run(mpp);

	mpp_debug_leave();
}

static void rockchip_mpp_try_run(struct rockchip_mpp_dev *mpp)
{
	struct mpp_service *pservice = mpp->srv;

	mpp_debug_enter();

	if (!list_empty(&pservice->pending)) {
		if (mpp->ops->prepare) {
			mpp_service_power_on(pservice);
			mpp->ops->prepare(mpp);
		}

		if (!test_bit(HW_RUNNING, &mpp->state))
			rockchip_mpp_run(mpp);
	}

	mpp_debug_leave();
}

static int rockchip_mpp_result(struct rockchip_mpp_dev *mpp,
			       struct mpp_ctx *ctx, u32 __user *dst)
{
	mpp_debug_enter();

	if (mpp->ops->result)
		mpp->ops->result(mpp, ctx, dst);

	ctx_deinit(mpp, ctx);

	mpp_debug_leave();
	return 0;
}

static int mpp_service_wait_result(struct mpp_session *session,
				   struct rockchip_mpp_dev *mpp,
				   u32 __user *req)
{
	struct mpp_service *pservice = mpp->srv;
	struct mpp_ctx *reg;
	int ret;

	ret = wait_event_timeout(session->wait,
				 !list_empty(&session->done),
				 MPP_TIMEOUT_DELAY);

	if (!list_empty(&session->done)) {
		if (ret < 0)
			mpp_err("warning: pid %d wait task error ret %d\n",
				session->pid, ret);
		ret = 0;
	} else {
		if (unlikely(ret < 0)) {
			mpp_err("error: pid %d wait task ret %d\n",
				session->pid, ret);
		} else if (ret == 0) {
			mpp_err("error: pid %d wait %d task done timeout\n",
				session->pid,
				atomic_read(&session->task_running));
			ret = -ETIMEDOUT;
		}
	}

	if (ret < 0) {
		mutex_lock(&pservice->lock);
		if (mpp->ops->reset)
			mpp->ops->reset(mpp);
		mutex_unlock(&pservice->lock);
		return ret;
	}

	mutex_lock(&pservice->lock);
	reg = list_entry(session->done.next,
			 struct mpp_ctx, session_link);
	rockchip_mpp_result(mpp, reg, req);
	mutex_unlock(&pservice->lock);

	return 0;
}

static long mpp_service_ioctl(struct file *filp, unsigned int cmd,
			      unsigned long arg)
{
	struct rockchip_mpp_dev *mpp =
		container_of(filp->f_dentry->d_inode->i_cdev,
			     struct rockchip_mpp_dev, cdev);
	struct mpp_service *pservice = mpp->srv;
	struct mpp_session *session = (struct mpp_session *)filp->private_data;

	mpp_debug_enter();
	if (NULL == session)
		return -EINVAL;

	switch (cmd) {
	case MPP_IOC_SET_CLIENT_TYPE:
		break;
	case MPP_IOC_SET_REG: {
		struct mpp_request req;
		struct mpp_ctx *reg;

		mpp_debug(DEBUG_IOCTL, "pid %d set reg\n", session->pid);
		if (copy_from_user(&req, (void __user *)arg,
				   sizeof(struct mpp_request))) {
			mpp_err("error: set reg copy_from_user failed\n");
			return -EFAULT;
		}
		reg = ctx_init(mpp, session, (void __user *)req.req, req.size);
		if (NULL == reg) {
			return -EFAULT;
		} else {
			mutex_lock(&pservice->lock);
			rockchip_mpp_try_run(mpp);
			mutex_unlock(&pservice->lock);
		}
	} break;
	case MPP_IOC_GET_REG: {
		struct mpp_request req;

		mpp_debug(DEBUG_IOCTL, "pid %d get reg\n",
			  session->pid);
		if (copy_from_user(&req, (void __user *)arg,
				   sizeof(struct mpp_request))) {
			mpp_err("error: get reg copy_from_user failed\n");
			return -EFAULT;
		}

		return mpp_service_wait_result(session, mpp, req.req);
	} break;
	case MPP_IOC_PROBE_IOMMU_STATUS: {
		int iommu_enable = 1;

		mpp_debug(DEBUG_IOCTL, "VPU_IOC_PROBE_IOMMU_STATUS\n");

		if (copy_to_user((void __user *)arg,
				 &iommu_enable, sizeof(int))) {
			mpp_err("error: iommu status copy_to_user failed\n");
			return -EFAULT;
		}
	} break;
	default: {
		if (mpp->ops->ioctl)
			return mpp->ops->ioctl(filp, cmd, arg);
		else
			mpp_err("unknow mpp service ioctl cmd %x\n", cmd);
	} break;
	}

	mpp_debug_leave();
	return 0;
}

#ifdef CONFIG_COMPAT
static long compat_mpp_service_ioctl(struct file *filp, unsigned int cmd,
				     unsigned long arg)
{
	struct rockchip_mpp_dev *data =
		container_of(filp->f_dentry->d_inode->i_cdev,
			     struct rockchip_mpp_dev, cdev);
	struct mpp_service *pservice = data->srv;
	struct mpp_session *session = (struct mpp_session *)filp->private_data;

	mpp_debug_enter();
	mpp_debug(3, "cmd %x, MPP_IOC_SET_CLIENT_TYPE %x\n", cmd,
		  (u32)MPP_IOC_SET_CLIENT_TYPE);
	if (NULL == session)
		return -EINVAL;

	switch (cmd) {
	case MPP_IOC_SET_CLIENT_TYPE:
		break;
	case MPP_IOC_SET_REG: {
		struct compat_mpp_request req;
		struct mpp_ctx *reg;

		mpp_debug(DEBUG_IOCTL, "compat set reg\n");
		if (copy_from_user(&req, compat_ptr((compat_uptr_t)arg),
				   sizeof(struct compat_mpp_request))) {
			mpp_err("compat set_reg copy_from_user failed\n");
			return -EFAULT;
		}
		reg = ctx_init(data, session,
			       compat_ptr((compat_uptr_t)req.req), req.size);
		if (NULL == reg) {
			return -EFAULT;
		} else {
			mutex_lock(&pservice->lock);
			rockchip_mpp_try_run(data);
			mutex_unlock(&pservice->lock);
		}
	} break;
	case MPP_IOC_GET_REG: {
		struct compat_mpp_request req;

		mpp_debug(DEBUG_IOCTL, "compat get reg\n");
		if (copy_from_user(&req, compat_ptr((compat_uptr_t)arg),
				   sizeof(struct compat_mpp_request))) {
			mpp_err("compat get reg copy_from_user failed\n");
			return -EFAULT;
		}

		return mpp_service_wait_result(session, data,
			compat_ptr((compat_uptr_t)req.req));
	} break;
	case MPP_IOC_PROBE_IOMMU_STATUS: {
		int iommu_enable = 1;

		mpp_debug(DEBUG_IOCTL, "COMPAT_VPU_IOC_PROBE_IOMMU_STATUS\n");

		if (copy_to_user(compat_ptr((compat_uptr_t)arg),
				 &iommu_enable, sizeof(int))) {
			mpp_err("error: VPU_IOC_PROBE_IOMMU_STATUS failed\n");
			return -EFAULT;
		}
	} break;
	default: {
		mpp_err("error: unknow mpp service ioctl cmd %x\n", cmd);
	} break;
	}
	mpp_debug_leave();
	return 0;
}
#endif

static int mpp_service_open(struct inode *inode, struct file *filp)
{
	struct rockchip_mpp_dev *data = container_of(
			inode->i_cdev, struct rockchip_mpp_dev, cdev);
	struct mpp_service *pservice = data->srv;
	struct mpp_session *session = kmalloc(sizeof(*session), GFP_KERNEL);

	mpp_debug_enter();

	if (NULL == session) {
		mpp_err("error: unable to allocate memory for mpp_session.");
		return -ENOMEM;
	}

	session->pid	= current->pid;
	INIT_LIST_HEAD(&session->waiting);
	INIT_LIST_HEAD(&session->running);
	INIT_LIST_HEAD(&session->done);
	INIT_LIST_HEAD(&session->list_session);
	init_waitqueue_head(&session->wait);
	atomic_set(&session->task_running, 0);
	mutex_lock(&pservice->lock);
	list_add_tail(&session->list_session, &pservice->session);
	filp->private_data = (void *)session;
	mutex_unlock(&pservice->lock);

	pr_debug("dev opened\n");
	mpp_debug_leave();
	return nonseekable_open(inode, filp);
}

static int mpp_service_release(struct inode *inode, struct file *filp)
{
	struct rockchip_mpp_dev *mpp = container_of(
			inode->i_cdev, struct rockchip_mpp_dev, cdev);
	struct mpp_service *pservice = mpp->srv;
	int task_running;
	struct mpp_session *session = (struct mpp_session *)filp->private_data;

	mpp_debug_enter();
	if (NULL == session)
		return -EINVAL;

	task_running = atomic_read(&session->task_running);
	if (task_running) {
		pr_err("session %d still has %d task running when closing\n",
		       session->pid, task_running);
		msleep(50);
	}
	wake_up(&session->wait);

	mutex_lock(&pservice->lock);
	/* remove this filp from the asynchronusly notified filp's */
	list_del_init(&session->list_session);
	mpp_service_session_clear(mpp, session);
	kfree(session);
	filp->private_data = NULL;
	mutex_unlock(&pservice->lock);

	pr_debug("dev closed\n");
	mpp_debug_leave();
	return 0;
}

static const struct file_operations mpp_service_fops = {
	.unlocked_ioctl = mpp_service_ioctl,
	.open		= mpp_service_open,
	.release	= mpp_service_release,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = compat_mpp_service_ioctl,
#endif
};

static irqreturn_t mpp_irq(int irq, void *dev_id)
{
	struct rockchip_mpp_dev *mpp = (struct rockchip_mpp_dev *)dev_id;

	int ret = -1;

	if (mpp->ops->irq)
		ret = mpp->ops->irq(mpp);

	if (ret < 0)
		return IRQ_NONE;
	else
		return IRQ_WAKE_THREAD;
}

static irqreturn_t mpp_isr(int irq, void *dev_id)
{
	struct rockchip_mpp_dev *data = (struct rockchip_mpp_dev *)dev_id;
	struct mpp_service *pservice = data->srv;
	struct mpp_ctx *ctx;

	ctx = pservice->current_ctx;
	if (IS_ERR_OR_NULL(ctx)) {
		mpp_err("no current context present\n");
		return IRQ_HANDLED;
	}

	mpp_time_diff(ctx);

	mutex_lock(&pservice->lock);

	list_del_init(&ctx->status_link);
	list_add_tail(&ctx->status_link, &pservice->done);

	list_del_init(&ctx->session_link);
	list_add_tail(&ctx->session_link, &ctx->session->done);

	if (data->ops->done)
		data->ops->done(data);

	clear_bit(HW_RUNNING, &data->state);
	pservice->current_ctx = NULL;

	wake_up(&ctx->session->wait);

	rockchip_mpp_try_run(data);

	mutex_unlock(&pservice->lock);

	return IRQ_HANDLED;
}

#if defined(CONFIG_OF)
static const struct of_device_id mpp_service_dt_ids[] = {
	{.compatible = "rockchip,rkvenc", .data = &rkvenc_variant, },
	{},
};
#endif

static int mpp_dev_probe(struct platform_device *pdev,
			 struct mpp_service *pservice)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	char *name = (char *)dev_name(dev);
	struct device_node *np = pdev->dev.of_node;
	struct rockchip_mpp_dev *mpp = NULL;
	enum VCODEC_RUNNING_MODE mode;
	const struct of_device_id *match;
	const struct rockchip_mpp_dev_variant *variant;

	pr_info("probe device %s\n", dev_name(dev));

	of_property_read_u32(np, "dev_mode", (u32 *)&mode);

	match = of_match_node(mpp_service_dt_ids, dev->of_node);
	variant = match->data;

	mpp = devm_kzalloc(dev, variant->data_len, GFP_KERNEL);
	mpp->srv = pservice;
	mpp->dev = dev;
	mpp->state = 0;
	mpp->variant = variant;

	mpp->variant->hw_probe(mpp);

	of_property_read_string(np, "name", (const char **)&name);
	mpp->mode = mode;

	mpp->reg_base = pservice->reg_base;

	mpp->irq = platform_get_irq(pdev, 0);
	if (mpp->irq > 0) {
		ret = devm_request_threaded_irq(dev, mpp->irq,
						mpp_irq, mpp_isr,
						IRQF_SHARED, dev_name(dev),
						(void *)mpp);
		if (ret) {
			dev_err(dev, "error: can't request vepu irq %d\n",
				mpp->irq);
			goto err;
		}
	} else {
		dev_info(dev, "No interrupt resource found\n");
	}

	dev_info(dev, "resource ready, register device\n");
	/* create device node */
	ret = alloc_chrdev_region(&mpp->dev_t, 0, 1, name);
	if (ret) {
		dev_err(dev, "alloc dev_t failed\n");
		goto err;
	}

	cdev_init(&mpp->cdev, &mpp_service_fops);

	mpp->cdev.owner = THIS_MODULE;
	mpp->cdev.ops = &mpp_service_fops;

	ret = cdev_add(&mpp->cdev, mpp->dev_t, 1);

	if (ret) {
		dev_err(dev, "add dev_t failed\n");
		goto err;
	}

	mpp->cls = class_create(THIS_MODULE, name);

	if (IS_ERR(mpp->cls)) {
		ret = PTR_ERR(mpp->cls);
		dev_err(dev, "class_create err:%d\n", ret);
		goto err;
	}

	mpp->child_dev = device_create(mpp->cls, dev,
		mpp->dev_t, NULL, name);

	platform_set_drvdata(pdev, mpp);

	INIT_LIST_HEAD(&mpp->lnk_service);
	list_add_tail(&mpp->lnk_service, &pservice->subdev_list);

#ifdef CONFIG_DEBUG_FS
	mpp->debugfs_dir = mpp_debugfs_create_device_dir(name, parent);
	if (!IS_ERR_OR_NULL(mpp->debugfs_dir))
		mpp->debugfs_file_regs =
			debugfs_create_file("regs", 0664, mpp->debugfs_dir,
					    mpp, &debug_mpp_fops);
	else
		mpp_err("create debugfs dir %s failed\n", name);
#endif
	return 0;
err:
	if (mpp->child_dev) {
		device_destroy(mpp->cls, mpp->dev_t);
		cdev_del(&mpp->cdev);
		unregister_chrdev_region(mpp->dev_t, 1);
	}

	if (mpp->cls)
		class_destroy(mpp->cls);
	return -1;
}

static void mpp_dev_remove(struct rockchip_mpp_dev *data)
{
	struct mpp_service *pservice = data->srv;

	data->ops->remove(data);

	mutex_lock(&pservice->lock);
	cancel_delayed_work_sync(&pservice->power_off_work);
	if (pservice->hw_ops->power_off)
		pservice->hw_ops->power_off(pservice);
	mutex_unlock(&pservice->lock);

	device_destroy(data->cls, data->dev_t);
	class_destroy(data->cls);
	cdev_del(&data->cdev);
	unregister_chrdev_region(data->dev_t, 1);

#ifdef CONFIG_DEBUG_FS
	if (!IS_ERR_OR_NULL(data->debugfs_dir))
		debugfs_remove_recursive(data->debugfs_dir);
#endif
}

static void mpp_read_property(struct device_node *np,
			      struct mpp_service *pservice)
{
	pservice->mode_bit = 0;
	pservice->mode_ctrl = 0;
	pservice->subcnt = 0;
	pservice->grf_base = NULL;

	of_property_read_u32(np, "subcnt", &pservice->subcnt);

	if (pservice->subcnt > 1) {
		of_property_read_u32(np, "mode_bit", &pservice->mode_bit);
		of_property_read_u32(np, "mode_ctrl", &pservice->mode_ctrl);
	}
#ifdef CONFIG_MFD_SYSCON
	pservice->grf = syscon_regmap_lookup_by_phandle(np, "rockchip,grf");
	if (IS_ERR_OR_NULL(pservice->grf)) {
		pservice->grf = NULL;
#ifdef CONFIG_ARM
		pservice->grf_base = RK_GRF_VIRT;
#else
		mpp_err("can't find mpp grf property\n");
		return;
#endif
	}
#else
#ifdef CONFIG_ARM
	pservice->grf_base = RK_GRF_VIRT;
#else
	mpp_err("can't find mpp grf property\n");
	return;
#endif
#endif
}

static void mpp_init_drvdata(struct mpp_service *pservice)
{
	pservice->curr_mode = -1;

	wake_lock_init(&pservice->wake_lock, WAKE_LOCK_SUSPEND, "mpp");
	INIT_LIST_HEAD(&pservice->pending);
	mutex_init(&pservice->lock);

	INIT_LIST_HEAD(&pservice->done);
	INIT_LIST_HEAD(&pservice->session);
	INIT_LIST_HEAD(&pservice->subdev_list);

	atomic_set(&pservice->enabled,       0);
	atomic_set(&pservice->power_on_cnt,  0);
	atomic_set(&pservice->power_off_cnt, 0);
	atomic_set(&pservice->reset_request, 0);

	pservice->last.tv64 = 0;
	pservice->ion_client = rockchip_ion_client_create("mpp");
	if (IS_ERR(pservice->ion_client)) {
		mpp_err("failed to create ion client for mpp ret %ld\n",
			PTR_ERR(pservice->ion_client));
	} else {
		mpp_debug(DEBUG_IOMMU, "mpp ion client create success!\n");
	}

	pservice->current_ctx = NULL;
}

static int mpp_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res = NULL;
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	struct mpp_service *pservice =
		devm_kzalloc(dev, sizeof(*pservice), GFP_KERNEL);

	dev_info(dev, "%s enter\n", __func__);

	pservice->dev = dev;

	mpp_read_property(np, pservice);

	mpp_init_drvdata(pservice);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	pservice->reg_base = devm_ioremap_resource(pservice->dev, res);
	if (IS_ERR(pservice->reg_base)) {
		mpp_err("ioremap registers base failed\n");
		ret = PTR_ERR(pservice->reg_base);
		goto err;
	}

	mpp_dev_probe(pdev, pservice);

	pr_info("init success\n");

	return 0;

err:
	pr_info("init failed\n");
	if (pservice->hw_ops->power_off)
		pservice->hw_ops->power_off(pservice);
	wake_lock_destroy(&pservice->wake_lock);

	return ret;
}

static int mpp_remove(struct platform_device *pdev)
{
	struct rockchip_mpp_dev *data = platform_get_drvdata(pdev);

	mpp_dev_remove(data);
	return 0;
}

static struct platform_driver mpp_driver = {
	.probe = mpp_probe,
	.remove = mpp_remove,
	.driver = {
		.name = "mpp",
		.owner = THIS_MODULE,
#if defined(CONFIG_OF)
		.of_match_table = of_match_ptr(mpp_service_dt_ids),
#endif
	},
};

static int __init mpp_service_init(void)
{
	int ret = platform_driver_register(&mpp_driver);

	if (ret) {
		mpp_err("Platform device register failed (%d).\n", ret);
		return ret;
	}

#ifdef CONFIG_DEBUG_FS
	mpp_debugfs_init();
#endif

	return ret;
}

static void __exit mpp_service_exit(void)
{
#ifdef CONFIG_DEBUG_FS
	mpp_debugfs_exit();
#endif

	platform_driver_unregister(&mpp_driver);
}

module_init(mpp_service_init);
module_exit(mpp_service_exit);
MODULE_LICENSE("GPL");

#ifdef CONFIG_DEBUG_FS
#include <linux/seq_file.h>

static int mpp_debugfs_init(void)
{
	parent = debugfs_create_dir("mpp", NULL);
	if (!parent)
		return -1;

	return 0;
}

static void mpp_debugfs_exit(void)
{
	debugfs_remove(parent);
}

static struct dentry *mpp_debugfs_create_device_dir(
		char *dirname, struct dentry *parent)
{
	return debugfs_create_dir(dirname, parent);
}

static int debug_mpp_show(struct seq_file *s, void *unused)
{
	struct rockchip_mpp_dev *data = s->private;
	struct mpp_service *pservice = data->srv;

	mpp_service_power_on(pservice);
	mutex_lock(&pservice->lock);

	mutex_unlock(&pservice->lock);
	mpp_service_power_off(pservice);

	return 0;
}

static int debug_mpp_open(struct inode *inode, struct file *file)
{
	return single_open(file, debug_mpp_show, inode->i_private);
}

#endif


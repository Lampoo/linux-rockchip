/**
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd
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

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_platform.h>

#include "mpp_dev_common.h"
#include "mpp_service.h"

#define MPP_POWER_OFF_DELAY		(4 * HZ)

inline void mpp_srv_lock(struct mpp_service *pservice)
{
	mutex_lock(&pservice->lock);
}
EXPORT_SYMBOL(mpp_srv_lock);

inline void mpp_srv_unlock(struct mpp_service *pservice)
{
	mutex_unlock(&pservice->lock);
}
EXPORT_SYMBOL(mpp_srv_unlock);

/* service queue schedule */
inline void mpp_srv_pending_lock(struct mpp_service *pservice,
			    struct mpp_ctx *ctx)
{	
	mpp_srv_lock(pservice);

	INIT_LIST_HEAD(&ctx->status_link);
	list_add_tail(&ctx->status_link, &pservice->pending);

	mpp_srv_unlock(pservice);
}
EXPORT_SYMBOL(mpp_srv_pending_lock);

void mpp_srv_run(struct mpp_service *pservice)
{
	struct mpp_ctx *ctx = mpp_srv_get_pending_ctx(pservice);

	list_del_init(&ctx->status_link);
	pservice->current_dev = ctx->mpp;
	pservice->current_ctx = ctx;
	mpp_srv_power_on(pservice);
	atomic_add(1, &pservice->total_running);
}
EXPORT_SYMBOL(mpp_srv_run);

inline void mpp_srv_done(struct mpp_service *pservice)
{
	struct mpp_ctx *ctx = pservice->current_ctx;

	list_del_init(&ctx->session_link);
	list_add_tail(&ctx->session_link, &ctx->session->done);

	list_del_init(&ctx->status_link);
	list_add_tail(&ctx->status_link, &pservice->done);

	wake_up(&ctx->session->wait);
	pservice->current_ctx = NULL;
	atomic_sub(1, &pservice->total_running);
}
EXPORT_SYMBOL(mpp_srv_done);

inline void mpp_srv_err_hdl_lock(struct mpp_service *pservice)
{
	mpp_srv_lock(pservice);
	atomic_sub(1, &pservice->total_running);
	mpp_srv_unlock(pservice);
}
EXPORT_SYMBOL(mpp_srv_err_hdl_lock);

inline struct mpp_ctx *mpp_srv_get_pending_ctx(struct mpp_service *pservice)
{
	return list_entry(pservice->pending.next, struct mpp_ctx, status_link);
}
EXPORT_SYMBOL(mpp_srv_get_pending_ctx);

inline struct mpp_ctx *mpp_srv_get_current_ctx(struct mpp_service *pservice)
{
	return pservice->current_ctx;
}
EXPORT_SYMBOL(mpp_srv_get_current_ctx);

inline struct mpp_ctx *mpp_srv_get_done_ctx(struct mpp_session *session)
{
	return list_entry(session->done.next, struct mpp_ctx, session_link);
}
EXPORT_SYMBOL(mpp_srv_get_done_ctx);

inline bool mpp_srv_pending_is_empty(struct mpp_service *pservice)
{
	return !!list_empty(&pservice->pending);
}
EXPORT_SYMBOL(mpp_srv_pending_is_empty);

void mpp_srv_attach(struct mpp_service *pservice, struct list_head *elem)
{
	INIT_LIST_HEAD(elem);
	list_add_tail(elem, &pservice->subdev_list);
	pservice->dev_cnt++;
}
EXPORT_SYMBOL(mpp_srv_attach);

void mpp_srv_detach(struct mpp_service *pservice, struct list_head *elem)
{
	list_del_init(elem);
	pservice->dev_cnt--;
}
EXPORT_SYMBOL(mpp_srv_detach);

void mpp_srv_power_off(struct mpp_service *pservice)
{
	struct rockchip_mpp_dev *mpp = pservice->current_dev;
	int total_running;
	int ret = atomic_add_unless(&pservice->enabled, -1, 0);

	if (!ret)
		return;

	total_running = atomic_read(&pservice->total_running);
	if (total_running) {
		pr_alert("alert: power off when %d task running!!\n",
			 total_running);
		mdelay(50);
		pr_alert("alert: delay 50 ms for running task\n");
	}

	pr_info("%s: power off...", dev_name(mpp->dev));

	mpp->variant->power_off(mpp);

	atomic_add(1, &pservice->power_off_cnt);
	wake_unlock(&pservice->wake_lock);
	pr_info("done\n");
}
EXPORT_SYMBOL(mpp_srv_power_off);

static inline void mpp_queue_power_off_work(struct mpp_service *pservice)
{
	queue_delayed_work(system_nrt_wq, &pservice->power_off_work,
			   MPP_POWER_OFF_DELAY);
}

void mpp_srv_power_off_work(struct work_struct *work_s)
{
	struct delayed_work *dlwork = container_of(work_s,
			struct delayed_work, work);
	struct mpp_service *pservice =
		container_of(dlwork,
			     struct mpp_service, power_off_work);

	if (mutex_trylock(&pservice->lock)) {
		mpp_srv_power_off(pservice);
		mutex_unlock(&pservice->lock);
	} else {
		/* Come back later if the device is busy... */
		mpp_queue_power_off_work(pservice);
	}
}

void mpp_srv_power_on(struct mpp_service *pservice)
{
	int ret;
	ktime_t now = ktime_get();
	struct rockchip_mpp_dev *mpp = pservice->current_dev;

	if (ktime_to_ns(ktime_sub(now, pservice->last)) > NSEC_PER_SEC) {
		cancel_delayed_work_sync(&pservice->power_off_work);
		mpp_queue_power_off_work(pservice);
		pservice->last = now;
	}
	ret = atomic_add_unless(&pservice->enabled, 1, 1);
	if (!ret)
		return;

	pr_info("%s: power on\n", dev_name(mpp->dev));

	mpp->variant->power_on(mpp);

	atomic_add(1, &pservice->power_on_cnt);
	wake_lock(&pservice->wake_lock);
}
EXPORT_SYMBOL(mpp_srv_power_on);

bool mpp_srv_is_power_on(struct mpp_service *pservice)
{
	return !!atomic_read(&pservice->enabled);
}
EXPORT_SYMBOL(mpp_srv_is_power_on);

static void mpp_init_drvdata(struct mpp_service *pservice)
{
	wake_lock_init(&pservice->wake_lock, WAKE_LOCK_SUSPEND, "mpp");
	INIT_LIST_HEAD(&pservice->pending);
	mutex_init(&pservice->lock);

	INIT_LIST_HEAD(&pservice->done);
	INIT_LIST_HEAD(&pservice->session);
	INIT_LIST_HEAD(&pservice->subdev_list);

	atomic_set(&pservice->total_running, 0);
	atomic_set(&pservice->enabled, 0);
	atomic_set(&pservice->power_on_cnt, 0);
	atomic_set(&pservice->power_off_cnt, 0);
	INIT_DELAYED_WORK(&pservice->power_off_work, mpp_srv_power_off_work);

	pservice->last.tv64 = 0;

	pservice->current_ctx = NULL;
}

#if defined(CONFIG_OF)
static const struct of_device_id mpp_service_dt_ids[] = {
	{.compatible = "rockchip,mpp_service", },
	{},
};
#endif

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

	mpp_init_drvdata(pservice);

	if (of_property_read_bool(np, "reg")) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

		pservice->reg_base = devm_ioremap_resource(pservice->dev, res);
		if (IS_ERR(pservice->reg_base)) {
			mpp_err("ioremap registers base failed\n");
			ret = PTR_ERR(pservice->reg_base);
			pservice->reg_base = 0;
		}
	} else {
		pservice->reg_base = 0;
	}

	pservice->cls = class_create(THIS_MODULE, dev_name(dev));

	if (IS_ERR(pservice->cls)) {
		ret = PTR_ERR(pservice->cls);
		dev_err(dev, "class_create err:%d\n", ret);
		return -1;
	}

	platform_set_drvdata(pdev, pservice);
	dev_info(dev, "init success\n");

	return 0;
}

static int mpp_remove(struct platform_device *pdev)
{
	struct mpp_service *pservice = platform_get_drvdata(pdev);

	class_destroy(pservice->cls);
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

	return ret;
}

subsys_initcall(mpp_service_init);
MODULE_LICENSE("GPL");


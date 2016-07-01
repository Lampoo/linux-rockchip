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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/rockchip_ion.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <linux/rockchip/pmu.h>
#include <linux/rockchip/cru.h>

#include "mpp_dev_common.h"

static int vcodec_bufid_to_iova(struct rockchip_mpp_dev *data, const u8 *tbl,
				int size, u32 *reg, struct mpp_ctx *ctx)
{
	struct mpp_service *pservice = data->srv;
	struct ion_handle *hdl;
	int ret = 0;
	struct mpp_mem_region *mem_region;
	int i;
	int offset = 0;

	if (tbl == NULL || size <= 0) {
		dev_err(pservice->dev, "input arguments invalidate\n");
		return -1;
	}

	for (i = 0; i < size; i++) {
		int usr_fd = reg[tbl[i]] & 0x3FF;

		mpp_debug(DEBUG_IOMMU, "reg[%03d] fd = %d\n", tbl[i], usr_fd);

		/* if userspace do not set the fd at this register, skip */
		if (usr_fd == 0)
			continue;

		offset = reg[tbl[i]] >> 10;

		mpp_debug(DEBUG_IOMMU, "pos %3d fd %3d offset %10d\n",
			  tbl[i], usr_fd, offset);

		hdl = ion_import_dma_buf(pservice->ion_client, usr_fd);
		if (IS_ERR(hdl)) {
			dev_err(pservice->dev,
				"import dma-buf from fd %d failed, reg[%d]\n",
				usr_fd, tbl[i]);
			return PTR_ERR(hdl);
		}

		mem_region = kzalloc(sizeof(*mem_region), GFP_KERNEL);

		if (!mem_region) {
			ion_free(pservice->ion_client, hdl);
			return -1;
		}

		mem_region->hdl = hdl;
		mem_region->reg_idx = tbl[i];

		if (data->mmu_dev)
			ret = ion_map_iommu(data->dev,
					    pservice->ion_client,
					    mem_region->hdl,
					    &mem_region->iova,
					    &mem_region->len);
		else
			ret = ion_phys(pservice->ion_client,
				       mem_region->hdl,
				       (ion_phys_addr_t *)&mem_region->iova,
				       (size_t *)&mem_region->len);

		if (ret < 0) {
			dev_err(pservice->dev, "reg %d fd %d ion map iommu failed\n",
				tbl[i], usr_fd);
			kfree(mem_region);
			ion_free(pservice->ion_client, hdl);
			return ret;
		}

		reg[tbl[i]] = mem_region->iova + offset;
		INIT_LIST_HEAD(&mem_region->reg_lnk);
		list_add_tail(&mem_region->reg_lnk, &ctx->mem_region_list);
	}

	return 0;
}

int mpp_reg_address_translate(struct rockchip_mpp_dev *data,
			      u32 *reg,
			      struct mpp_ctx *ctx,
			      const struct mpp_trans_info *info)
{
	const u8 *tbl = info->table;
	int size = info->count;

	return vcodec_bufid_to_iova(data, tbl, size, reg, ctx);
}

void translate_extra_info(struct mpp_ctx *ctx,
			  struct extra_info_for_iommu *ext_inf,
			  u32 *reg)
{
	if (ext_inf != NULL /*&& ext_inf->magic == EXTRA_INFO_MAGIC*/) {
		int i;

		for (i = 0; i < ext_inf->cnt; i++) {
			mpp_debug(DEBUG_IOMMU, "reg[%d] + offset %d\n",
				  ext_inf->elem[i].index,
				  ext_inf->elem[i].offset);
			reg[ext_inf->elem[i].index] +=
				ext_inf->elem[i].offset;
		}
	}
}

void dump_reg(u32 *regs, int count)
{
	int i;

	pr_info("dumping vpu_device registers:");

	for (i = 0; i < count; i++)
		pr_info("reg[%02d]: %08x\n", i, readl_relaxed(regs + i));
}

void dump_reg_mem(u32 *regs, int count)
{
	int i;

	pr_info("Dumping mpp_service registers:\n");

	for (i = 0; i < count; i++)
		pr_info("reg[%03d]: %08x\n", i, regs[i]);
}

void vcodec_enter_mode(struct rockchip_mpp_dev *data)
{
	int bits;
	u32 raw = 0;
	struct mpp_service *pservice = data->srv;
	struct rockchip_mpp_dev *subdata, *n;

	if (pservice->subcnt < 2) {
		if (data->mmu_dev && !test_bit(MMU_ACTIVATED, &data->state)) {
			set_bit(MMU_ACTIVATED, &data->state);
			if (atomic_read(&pservice->enabled))
				rockchip_iovmm_activate(data->dev);
			else
				BUG_ON(!atomic_read(&pservice->enabled));
		}
		return;
	}

	if (pservice->curr_mode == data->mode)
		return;

	mpp_debug(DEBUG_IOMMU, "vcodec enter mode %d\n", data->mode);
	list_for_each_entry_safe(subdata, n,
				 &pservice->subdev_list, lnk_service) {
		if (data != subdata && subdata->mmu_dev &&
		    test_bit(MMU_ACTIVATED, &subdata->state)) {
			clear_bit(MMU_ACTIVATED, &subdata->state);
			rockchip_iovmm_deactivate(subdata->dev);
		}
	}

	bits = 1 << pservice->mode_bit;
#ifdef CONFIG_MFD_SYSCON
	if (pservice->grf) {
		regmap_read(pservice->grf, pservice->mode_ctrl, &raw);

		if (data->mode == VCODEC_RUNNING_MODE_HEVC)
			regmap_write(pservice->grf, pservice->mode_ctrl,
				     raw | bits | (bits << 16));
		else
			regmap_write(pservice->grf, pservice->mode_ctrl,
				     (raw & (~bits)) | (bits << 16));
	} else if (pservice->grf_base) {
		u32 *grf_base = pservice->grf_base;

		raw = readl_relaxed(grf_base + pservice->mode_ctrl / 4);
		if (data->mode == VCODEC_RUNNING_MODE_HEVC)
			writel_relaxed(raw | bits | (bits << 16),
				       grf_base + pservice->mode_ctrl / 4);
		else
			writel_relaxed((raw & (~bits)) | (bits << 16),
				       grf_base + pservice->mode_ctrl / 4);
	} else {
		mpp_err("no grf resource defined, switch decoder failed\n");
		return;
	}
#else
	if (pservice->grf_base) {
		u32 *grf_base = pservice->grf_base;

		raw = readl_relaxed(grf_base + pservice->mode_ctrl / 4);
		if (data->mode == VCODEC_RUNNING_MODE_HEVC)
			writel_relaxed(raw | bits | (bits << 16),
				       grf_base + pservice->mode_ctrl / 4);
		else
			writel_relaxed((raw & (~bits)) | (bits << 16),
				       grf_base + pservice->mode_ctrl / 4);
	} else {
		mpp_err("no grf resource define, switch decoder failed\n");
		return;
	}
#endif
	if (data->mmu_dev && !test_bit(MMU_ACTIVATED, &data->state)) {
		set_bit(MMU_ACTIVATED, &data->state);
		if (atomic_read(&pservice->enabled))
			rockchip_iovmm_activate(data->dev);
		else
			BUG_ON(!atomic_read(&pservice->enabled));
	}

	pservice->prev_mode = pservice->curr_mode;
	pservice->curr_mode = data->mode;
}

void vcodec_exit_mode(struct rockchip_mpp_dev *data)
{
	if (data->mmu_dev && test_bit(MMU_ACTIVATED, &data->state)) {
		clear_bit(MMU_ACTIVATED, &data->state);
		rockchip_iovmm_deactivate(data->dev);
		data->srv->curr_mode = VCODEC_RUNNING_MODE_NONE;
	}
}

int mpp_dev_common_ctx_init(struct rockchip_mpp_dev *mpp, struct mpp_ctx *cfg)
{
	INIT_LIST_HEAD(&cfg->mem_region_list);

	return 0;
}

/* device operation fucntions */
struct mpp_ctx *vcodec_common_init(struct rockchip_mpp_dev *data,
				   void __user *src, u32 size)
{
	return 0;
}

int rockchip_mpp_common_try_run(struct rockchip_mpp_dev *mpp)
{
	return 0;
}

int vcodec_common_run(struct rockchip_mpp_dev *data)
{
	return 0;
}

int vcodec_common_done(struct rockchip_mpp_dev *data)
{
	return 0;
}

int vcodec_common_irq(struct rockchip_mpp_dev *data)
{
	return 0;
}

int rockchip_mpp_common_reset(struct rockchip_mpp_dev *mpp)
{
	return 0;
}

int rockchip_mpp_common_result(struct rockchip_mpp_dev *mpp,
			       struct mpp_ctx *ctx,
			       u32 __user *dst)
{
	return 0;
}


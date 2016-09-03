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
#include <linux/compiler.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/rockchip_ion.h>

#include "mpp_dev_common.h"
#include "mpp_dev_rkvenc.h"

#define LINK_TABLE_START	12
#define LINK_TABLE_LEN		128

#define RKVENC_ENC_START		0x004
#define     RKVENC_LKT_NUM(x)			(((x) & 0xff) << 0)
#define	    RKVENC_CMD(x)			(((x) & 0x3) << 8)
#define	    RKVENC_CLK_GATE_EN			BIT(16)
#define RKVENC_SAFE_CLR			0x008
#define RKVENC_LKT_ADDR			0x00c
#define RKVENC_INT_EN			0x010
#define     RKVENC_INT_EN_TIMEOUT		BIT(8)
#define RKVENC_INT_MSK			0x014
#define     RKVENC_INT_MSK_TIMEOUT		BIT(8)
#define RKVENC_INT_CLR			0x018
#define RKVENC_INT_STATUS		0x01c
#define     RKVENC_ONE_FRAME_FINISH		BIT(0)
#define     RKVENC_LINK_TABLE_FINISH		BIT(1)
#define     RKVENC_SAFE_CLEAR_FINISH		BIT(2)
#define     RKVENC_ONE_SLICE_FINISH		BIT(3)
#define     RKVENC_BIT_STREAM_OVERFLOW		BIT(4)
#define     RKVENC_AXI_WRITE_FIFO_FULL		BIT(5)
#define     RKVENC_AXI_WRITE_CHANNEL_ERROR	BIT(6)
#define     RKVENC_AXI_READ_CHANNEL_ERROR	BIT(7)
#define     RKVENC_TIMEOUT_ERROR		BIT(8)
#define RKVENC_ENC_PIC			0x034
#define     RKVENC_ENC_PIC_NODE_INT_EN		BIT(31)
#define RKVENC_ENC_WDG			0x038
#define     RKVENC_PPLN_ENC_LMT(x)		(((x) & 0xff) << 0)
#define RKVENC_OSD_CFG			0x1c0
#define     RKVENC_OSD_CLK_SEL_BIT		BIT(16)
#define RKVENC_STATUS(i)		(0x210 + (4 * i))
#define RKVENC_BSL_STATUS		0x210
#define     RKVENC_BITSTREAM_LENGTH(x)		((x) & 0x7FFFFFF)
#define RKVENC_LKT_STATUS		0x224
#define     RKVENC_LKT_STATUS_FNUM_ENC(x)	(((x) >> 0) & 0xff)
#define     RKVENC_LKT_STATUS_FNUM_CFG(x)	(((x) >> 8) & 0xff)
#define     RKVENC_LKT_STATUS_FNUM_INT(x)	(((x) >> 16) & 0xff)
#define RKVENC_OSD_PLT(i)		(0x400 + (4 * i))
#define RKVENC_OSD_PLT_LEN		256

/*
 * file handle translate information
 */
static const char trans_tbl_rkvenc[] = {
	70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86,
	124, 125, 126, 127, 128, 129, 130, 131
};

const struct mpp_trans_info trans_rkvenc[1] = {
	[0] = {
		.count = sizeof(trans_tbl_rkvenc),
		.table = trans_tbl_rkvenc,
	},
};

struct mpp_ctx *rockchip_mpp_rkvenc_init(struct rockchip_mpp_dev *data,
					 void __user *src, u32 size)
{
	struct rkvenc_ctx *ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	int i;

	mpp_debug_enter();

	if (ctx == NULL)
		return NULL;

	mpp_dev_common_ctx_init(data, &ctx->ictx);

	ctx->mode = RKVENC_MODE_LINKTABLE_FIX;

	size *= 4; /* convert to size in byte */

	WARN_ON(size > sizeof(ctx->cfg));

	size = size > sizeof(ctx->cfg) ? sizeof(ctx->cfg) : size;

	if (copy_from_user(&ctx->cfg, src, size)) {
		mpp_err("error: copy_from_user failed in reg_init\n");
		kfree(ctx);
		return NULL;
	}

	ctx->mode = ctx->cfg.mode;
	if (ctx->mode >= RKVENC_MODE_NUM || ctx->mode == RKVENC_MODE_NONE) {
		mpp_err("Invalid rkvenc running mode %d\n", (int)ctx->mode);
		kfree(ctx);
		return NULL;
	} else if (ctx->mode == RKVENC_MODE_ONEFRAME && ctx->cfg.tbl_num > 1) {
		mpp_err("Configuration miss match, ignore redundant cfg\n");
		ctx->cfg.tbl_num = 1;
	}

	mpp_debug(DEBUG_SET_REG, "tbl num %u, mode %u\n",
		  ctx->cfg.tbl_num, ctx->cfg.mode);

	for (i = 0; i < ctx->cfg.tbl_num; i++) {
		if (0 > mpp_reg_address_translate(data,
						  ctx->cfg.elem[i].reg,
						  &ctx->ictx,
						  &trans_rkvenc[0])) {
			mpp_err("error: translate reg address failed.\n");

			if (unlikely(debug & DEBUG_DUMP_ERR_REG))
				dump_reg_mem(ctx->cfg.elem[i].reg,
					     ctx->cfg.elem[i].reg_num);

			kfree(ctx);

			return NULL;
		}

		mpp_debug(DEBUG_SET_REG, "extra info cnt %u, magic %08x",
			  ctx->cfg.elem[i].ext_inf.cnt,
			  ctx->cfg.elem[i].ext_inf.magic);

		translate_extra_info(&ctx->ictx, &ctx->cfg.elem[i].ext_inf,
				     ctx->cfg.elem[i].reg);
	}

	mpp_debug_leave();

	return &ctx->ictx;
}

void rockchip_mpp_rkvenc_deinit(struct rockchip_mpp_dev *mpp)
{
}

static void status_check(struct rockchip_mpp_dev *mpp)
{
	u32 lkt_status;
	u32 fnum_int;
	u32 fnum_cfg;
	u32 fnum_enc;

	do {
		lkt_status = mpp_read(mpp, RKVENC_LKT_STATUS);
		fnum_int = RKVENC_LKT_STATUS_FNUM_INT(lkt_status);
		fnum_cfg = RKVENC_LKT_STATUS_FNUM_CFG(lkt_status);
		fnum_enc = RKVENC_LKT_STATUS_FNUM_ENC(lkt_status);

		mpp_debug(DEBUG_GET_REG, "frm_num int %u, cfg %u, enc %u\n",
			  fnum_int, fnum_cfg, fnum_enc);

		mdelay(2);
	} while (fnum_int < 2);
}

int rockchip_mpp_rkvenc_prepare(struct rockchip_mpp_dev *mpp)
{
	struct mpp_service *pservice = mpp->srv;
	struct mpp_ctx *ictx;
	struct rkvenc_ctx *ctx_curr;
	struct rkvenc_ctx *ctx_ready;
	struct rockchip_rkvenc_dev *enc =
		container_of(mpp, struct rockchip_rkvenc_dev, dev);

	u32 reg = 0;

	mpp_debug_enter();

	ictx = list_entry(pservice->pending.next,
			  struct mpp_ctx, status_link);
	ctx_curr =
		container_of(pservice->current_ctx, struct rkvenc_ctx, ictx);
	ctx_ready =
		container_of(ictx, struct rkvenc_ctx, ictx);

	if (test_bit(HW_RUNNING, &mpp->state) &&
	    !(ctx_curr && ctx_curr->mode == RKVENC_MODE_LINKTABLE_UPDATE &&
	      ctx_ready->mode == RKVENC_MODE_LINKTABLE_UPDATE))
		return 0;

	if (ctx_curr) {
		u32 lkt_status = mpp_read(mpp, RKVENC_LKT_STATUS);
		u32 fnum_int = RKVENC_LKT_STATUS_FNUM_INT(lkt_status);
		u32 fnum_cfg = RKVENC_LKT_STATUS_FNUM_CFG(lkt_status);
		u32 fnum_enc = RKVENC_LKT_STATUS_FNUM_ENC(lkt_status);
		u8 *cpu_addr = (u8 *)enc->lkt_cpu_addr +
			fnum_cfg * LINK_TABLE_LEN * 4;

		mpp_debug(DEBUG_GET_REG, "frame number %u, %u, %u\n",
			  fnum_int, fnum_cfg, fnum_enc);

		memcpy(cpu_addr, &ctx_ready->cfg.elem[0].reg[LINK_TABLE_START],
		       LINK_TABLE_LEN * 4);

		reg = RKVENC_CLK_GATE_EN |
			RKVENC_CMD(ctx_curr->mode) |
			RKVENC_LKT_NUM(1);
		mpp_write_relaxed(mpp, reg, RKVENC_ENC_START);

		set_bit(HW_RUNNING, &mpp->state);

		list_del_init(&ictx->status_link);
		/* TODO deinit ctx */

		enc->lkt_index++;

		status_check(mpp);
	}

	mpp_debug_leave();

	return 0;
}

int rockchip_mpp_rkvenc_run(struct rockchip_mpp_dev *mpp)
{
	struct mpp_service *pservice = mpp->srv;
	struct rkvenc_ctx *ctx =
		container_of(pservice->current_ctx, struct rkvenc_ctx, ictx);
	struct rockchip_rkvenc_dev *enc =
		container_of(mpp, struct rockchip_rkvenc_dev, dev);
	u32 reg;
	int i;

	mpp_debug_enter();

	switch (ctx->mode) {
	case RKVENC_MODE_ONEFRAME: {
		u32 *src = ctx->cfg.elem[0].reg;

		for (i = (LINK_TABLE_START + LINK_TABLE_LEN) - 1; i > 0; i--)
			mpp_write_relaxed(mpp, src[i], i * 4);

		dsb(sy);

		break;
	}
	case RKVENC_MODE_LINKTABLE_FIX:
	case RKVENC_MODE_LINKTABLE_UPDATE: {
		for (i = 0; i < ctx->cfg.tbl_num; i++) {
			u32 *src = ctx->cfg.elem[i].reg;

			memcpy(enc->lkt_cpu_addr + i * LINK_TABLE_LEN * 4,
			       &src[LINK_TABLE_START], LINK_TABLE_LEN * 4);
		}

		mpp_write_relaxed(mpp, enc->lkt_dma_addr, RKVENC_LKT_ADDR);
		mpp_write_relaxed(mpp, 0xffffffff, RKVENC_INT_EN);

		reg = RKVENC_LKT_NUM(ctx->cfg.tbl_num) |
			RKVENC_CMD(RKVENC_MODE_LINKTABLE_FIX) |
			RKVENC_CLK_GATE_EN;

		mpp_write_relaxed(mpp, reg, RKVENC_ENC_START);

		break;
	}
	default:
		break;
	}

	mpp_debug_leave();

	return 0;
}

int rockchip_mpp_rkvenc_done(struct rockchip_mpp_dev *mpp)
{
	struct mpp_ctx *ictx = mpp->srv->current_ctx;
	struct rkvenc_ctx *ctx = container_of(ictx, struct rkvenc_ctx, ictx);
	struct rockchip_rkvenc_dev *enc =
		container_of(mpp, struct rockchip_rkvenc_dev, dev);
	struct rkvenc_result *result;
	int i;

	mpp_debug_enter();

	if (IS_ERR_OR_NULL(ictx)) {
		mpp_err("Invaidate context to save result\n");
		return -1;
	}

	result = &ctx->result;
	switch (ctx->mode) {
	case RKVENC_MODE_ONEFRAME:
		result->tbl_num = 1;
		result->elem[0].status = enc->irq_status;
		for (i = 0; i < sizeof(result->elem[0].result) / 4; i++)
			result->elem[0].result[i] =
				mpp_read(mpp, RKVENC_STATUS(i));
		break;
	case RKVENC_MODE_LINKTABLE_FIX:
	case RKVENC_MODE_LINKTABLE_UPDATE: {
		u32 lkt_status = mpp_read(mpp, RKVENC_LKT_STATUS);
		u32 fnum_int = RKVENC_LKT_STATUS_FNUM_INT(lkt_status);
		u32 fnum_cfg = RKVENC_LKT_STATUS_FNUM_CFG(lkt_status);
		u32 fnum_enc = RKVENC_LKT_STATUS_FNUM_ENC(lkt_status);

		u32 *lkt_cpu_addr = (u32 *)enc->lkt_cpu_addr;

		if (unlikely(debug & DEBUG_DUMP_ERR_REG))
			dump_reg_mem(lkt_cpu_addr, LINK_TABLE_LEN);

		result->tbl_num = fnum_int;
		for (i = 0; i < fnum_int; i++) {
			result->elem[i].status = enc->irq_status;
			memcpy(result->elem[i].result,
			       &lkt_cpu_addr[i * LINK_TABLE_LEN + 120],
			       sizeof(result->elem[i].result));
			mpp_debug(DEBUG_GET_REG, "stream length %u\n",
				  result->elem[i].result[0]);
		}
		mpp_debug(DEBUG_GET_REG, "frame number %u, %u, %u\n",
			  fnum_int, fnum_cfg, fnum_enc);
		break;
	}
	default:
		break;
	}

	mpp_debug_leave();

	return 0;
}

int rockchip_mpp_rkvenc_irq(struct rockchip_mpp_dev *mpp)
{
	struct rockchip_rkvenc_dev *enc =
		container_of(mpp, struct rockchip_rkvenc_dev, dev);
	u32 irq_status = mpp_read(mpp, RKVENC_INT_STATUS);

	mpp_debug_enter();

	if (irq_status) {
		mpp_write_relaxed(mpp, 0xffffffff, RKVENC_INT_CLR);

		mpp_debug(DEBUG_IRQ_STATUS, "interrupt status %08x\n",
			  irq_status);

		enc->irq_status = irq_status;

		mpp_debug_leave();

		return 0;
	} else {
		return -1;
	}
}

int rockchip_mpp_rkvenc_reset(struct rockchip_mpp_dev *mpp)
{
	struct mpp_service *pservice = mpp->srv;
	struct mpp_mem_region *mem_region = NULL, *n;

	struct mpp_ctx *ctx = pservice->current_ctx;

	/* release memory region attach to this registers table. */
	list_for_each_entry_safe(mem_region, n,
				 &ctx->mem_region_list, reg_lnk) {
		ion_free(pservice->ion_client, mem_region->hdl);
		list_del_init(&mem_region->reg_lnk);
		kfree(mem_region);
	}

	kfree(ctx);

	return 0;
}

int rockchip_mpp_rkvenc_result(struct rockchip_mpp_dev *mpp,
			       struct mpp_ctx *ictx, u32 __user *dst)
{
	struct rkvenc_ctx *ctx = container_of(ictx, struct rkvenc_ctx, ictx);
	struct rkvenc_result *result = &ctx->result;
	unsigned long tbl_size = sizeof(result->tbl_num) +
		sizeof(result->elem[0]) * result->tbl_num;

	switch (ctx->mode) {
	case RKVENC_MODE_ONEFRAME:
	case RKVENC_MODE_LINKTABLE_FIX:
	case RKVENC_MODE_LINKTABLE_UPDATE: {
		if (copy_to_user(dst, &ctx->result, tbl_size)) {
			mpp_err("copy result to user failed\n");
			return -1;
		}
		break;
	}
	default:
		mpp_err("invalid context mode %d\n", (int)ctx->mode);
		return -1;
	}

	return 0;
}

void rockchip_mpp_rkvenc_remove(struct rockchip_mpp_dev *mpp)
{
	struct rockchip_rkvenc_dev *enc =
		container_of(mpp, struct rockchip_rkvenc_dev, dev);
	struct mpp_service *pservice = mpp->srv;

	ion_unmap_kernel(pservice->ion_client, enc->lkt_hdl);
	ion_free(pservice->ion_client, enc->lkt_hdl);
}

static void clear_link_table_list(struct rockchip_rkvenc_dev *enc,
				  struct rkvenc_session *session)
{
	struct link_table_elem *elem, *n;
	struct mpp_service *pservice = enc->dev.srv;

	list_for_each_entry_safe(elem, n,
				 &session->lkt_free_list, list) {
		ion_unmap_kernel(pservice->ion_client, elem->lkt_hdl);
		ion_free(pservice->ion_client, elem->lkt_hdl);
		list_del_init(&elem->list);
		kfree(elem);
	}

	list_for_each_entry_safe(elem, n,
				 &session->lkt_used_list, list) {
		ion_unmap_kernel(pservice->ion_client, elem->lkt_hdl);
		ion_free(pservice->ion_client, elem->lkt_hdl);
		list_del_init(&elem->list);
		kfree(elem);
	}
}

static int rockchip_mpp_rkvenc_open(struct mpp_session **isession,
				    struct rockchip_mpp_dev *mpp)
{
	struct rkvenc_session *session =
		kmalloc(sizeof(*session), GFP_KERNEL);

	if (session == NULL) {
		mpp_err("allocate session failed\n");
		return -1;
	}

	*isession = &session->session;

	return 0;
}

static int rockchip_mpp_rkvenc_release(struct mpp_session *isession,
				       struct rockchip_mpp_dev *mpp)
{
	struct rockchip_rkvenc_dev *enc =
		container_of(mpp, struct rockchip_rkvenc_dev, dev);
	struct rkvenc_session *session =
		container_of(isession, struct rkvenc_session, session);

	clear_link_table_list(enc, session);

	kfree(session);

	return 0;
}

struct mpp_dev_ops rkvenc_ops = {
	.remove = rockchip_mpp_rkvenc_remove,
	.open = rockchip_mpp_rkvenc_open,
	.release = rockchip_mpp_rkvenc_release,
	.init = rockchip_mpp_rkvenc_init,
	.prepare = rockchip_mpp_rkvenc_prepare,
	.run = rockchip_mpp_rkvenc_run,
	.done = rockchip_mpp_rkvenc_done,
	.irq = rockchip_mpp_rkvenc_irq,
	.result = rockchip_mpp_rkvenc_result,
};

int rockchip_mpp_rkvenc_probe(struct rockchip_mpp_dev *mpp)
{
	struct rockchip_rkvenc_dev *enc =
		container_of(mpp, struct rockchip_rkvenc_dev, dev);
	ion_phys_addr_t *dma_addr = &enc->lkt_dma_addr;
	int ret;
	size_t tmp;
	struct mpp_service *pservice = mpp->srv;

	enc->dev.ops = &rkvenc_ops;

	enc->lkt_hdl = ion_alloc(pservice->ion_client,
				 LINK_TABLE_LEN * 4 * 256, 4096,
				 ION_HEAP(ION_CMA_HEAP_ID),
				 0);
	if (enc->lkt_hdl == NULL) {
		dev_err(mpp->dev, "allocate link table buffer failure\n");
		return -1;
	}

	ret = ion_phys(pservice->ion_client, enc->lkt_hdl,
		       dma_addr, &tmp);
	enc->lkt_cpu_addr =
		ion_map_kernel(pservice->ion_client, enc->lkt_hdl);

	enc->aclk = devm_clk_get(mpp->dev, "aclk_vcodec");
	if (IS_ERR_OR_NULL(enc->aclk)) {
		dev_err(mpp->dev, "failed on clk_get aclk\n");
		goto fail;
	}

	enc->hclk = devm_clk_get(mpp->dev, "hclk_vcodec");
	if (IS_ERR_OR_NULL(enc->hclk)) {
		dev_err(mpp->dev, "failed on clk_get hclk\n");
		goto fail;
	}

	enc->core = devm_clk_get(mpp->dev, "clk_core");
	if (IS_ERR_OR_NULL(enc->core)) {
		dev_err(mpp->dev, "failed on clk_get core\n");
		goto fail;
	}

	clk_prepare_enable(enc->aclk);
	clk_prepare_enable(enc->hclk);
	clk_prepare_enable(enc->core);

	return 0;

fail:
	ion_free(pservice->ion_client, enc->lkt_hdl);

	return -1;
}

const struct rockchip_mpp_dev_variant rkvenc_variant = {
	.data_len = sizeof(struct rockchip_rkvenc_dev),
	.hw_probe = rockchip_mpp_rkvenc_probe,
};
EXPORT_SYMBOL(rkvenc_variant);


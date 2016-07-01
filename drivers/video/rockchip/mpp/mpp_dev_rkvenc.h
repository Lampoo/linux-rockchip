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

#ifndef __ROCKCHIP_MPP_DEV_RKVENC_H
#define __ROCKCHIP_MPP_DEV_RKVENC_H

struct rkvenc_config_elem {
	u32 reg_num;
	u32 reg[140];
	struct extra_info_for_iommu ext_inf;
};

struct rkvenc_config {
	u32 mode;
	u32 tbl_num;
	struct rkvenc_config_elem elem[10];
};

struct rkvenc_result_elem {
	u32 status;
	u32 result[11];
};

struct rkvenc_result {
	u32 tbl_num;
	struct rkvenc_result_elem elem[10];
};

struct rkvenc_ctx {
	struct mpp_ctx ictx;
	/* link table hardware access address */
	//unsigned long lkt_dma_addr;
	//void *lkt_cpu_addr;
	//unsigned long lkt_size;
	//u32 lkt_index;
	//struct ion_handle *lkt_hdl;
	enum RKVENC_MODE mode;

	struct rkvenc_config cfg;

	/* store status read from hw, oneframe mode used only */
	struct rkvenc_result result;
};

struct rockchip_rkvenc_dev {
	struct rockchip_mpp_dev dev;
	unsigned long lkt_dma_addr;
	struct ion_handle *lkt_hdl;
	void *lkt_cpu_addr;
	u32 lkt_index;
	u32 irq_status;
	struct clk *aclk;
	struct clk *hclk;
	struct clk *core;
};

struct link_table_elem {
	unsigned long lkt_dma_addr;
	struct ion_handle *lkt_hdl;
	void *lkt_cpu_addr;
	u32 lkt_index;
	struct list_head list;
};

struct rkvenc_session {
	struct mpp_session session;

	struct list_head lkt_free_list;
	struct list_head lkt_used_list;
};

#endif

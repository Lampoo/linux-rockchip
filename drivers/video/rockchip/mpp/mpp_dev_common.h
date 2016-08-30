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

#ifndef __ROCKCHIP_MPP_DEV_COMMON_H
#define __ROCKCHIP_MPP_DEV_COMMON_H

#include <linux/cdev.h>
#include <linux/dma-buf.h>
#include <linux/rockchip-iovmm.h>
#include <linux/types.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>

extern int debug;

/*
 * debug flag usage:
 * +------+-------------------+
 * | 8bit |      24bit        |
 * +------+-------------------+
 *  0~23 bit is for different information type
 * 24~31 bit is for information print format
 */

#define DEBUG_POWER				0x00000001
#define DEBUG_CLOCK				0x00000002
#define DEBUG_IRQ_STATUS			0x00000004
#define DEBUG_IOMMU				0x00000008
#define DEBUG_IOCTL				0x00000010
#define DEBUG_FUNCTION				0x00000020
#define DEBUG_REGISTER				0x00000040
#define DEBUG_EXTRA_INFO			0x00000080
#define DEBUG_TIMING				0x00000100
#define DEBUG_TASK_INFO				0x00000200
#define DEBUG_DUMP_ERR_REG			0x00000400

#define DEBUG_SET_REG				0x00001000
#define DEBUG_GET_REG				0x00002000
#define DEBUG_PPS_FILL				0x00004000
#define DEBUG_IRQ_CHECK				0x00008000
#define DEBUG_CACHE_32B				0x00010000

#define DEBUG_RESET				0x00100000

#define PRINT_FUNCTION				0x80000000
#define PRINT_LINE				0x40000000

#define DEBUG
#ifdef DEBUG
#define mpp_debug_func(type, fmt, args...)			\
	do {							\
		if (unlikely(debug & type)) {			\
			pr_info("%s:%d: " fmt,			\
				 __func__, __LINE__, ##args);	\
		}						\
	} while (0)
#define mpp_debug(type, fmt, args...)				\
	do {							\
		if (unlikely(debug & type)) {			\
			pr_info(fmt, ##args);			\
		}						\
	} while (0)
#else
#define mpp_debug_func(level, fmt, args...)
#define mpp_debug(level, fmt, args...)
#endif

#define mpp_debug_enter() mpp_debug_func(DEBUG_FUNCTION, "enter\n")
#define mpp_debug_leave() mpp_debug_func(DEBUG_FUNCTION, "leave\n")

#define mpp_err(fmt, args...)				\
		pr_err("%s:%d: " fmt, __func__, __LINE__, ##args)

struct mpp_trans_info {
	const int count;
	const char * const table;
};

enum RKVENC_MODE {
	RKVENC_MODE_NONE,
	RKVENC_MODE_ONEFRAME,
	RKVENC_MODE_LINKTABLE_FIX,
	RKVENC_MODE_LINKTABLE_UPDATE,
	RKVENC_MODE_NUM
};

struct rockchip_mpp_dev;
struct mpp_service;
struct mpp_ctx;

/**
 * struct for process session which connect to vpu
 *
 * @author ChenHengming (2011-5-3)
 */
struct mpp_session {
	/* a linked list of data so we can access them for debugging */
	struct list_head list_session;
	/* a linked list of register data waiting for process */
	struct list_head waiting;
	/* a linked list of register data in processing */
	struct list_head running;
	/* a linked list of register data processed */
	struct list_head done;
	wait_queue_head_t wait;
	pid_t pid;
	atomic_t task_running;
};

struct mpp_mem_region {
	struct list_head srv_lnk;
	struct list_head reg_lnk;
	struct list_head session_lnk;
	unsigned long iova;	/* virtual address for iommu */
	unsigned long len;
	u32 reg_idx;
	struct ion_handle *hdl;
};

/**
 * struct for process register set
 *
 * @author ChenHengming (2011-5-4)
 */
struct mpp_ctx {
	struct mpp_session *session;

	/* link to vpu service session */
	struct list_head session_link;
	/* link to register set list */
	struct list_head status_link;

	struct list_head mem_region_list;

	/* record hw start time */
	struct timeval start;
};

enum VCODEC_RUNNING_MODE {
	VCODEC_RUNNING_MODE_NONE = -1,
	VCODEC_RUNNING_MODE_VPU,
	VCODEC_RUNNING_MODE_HEVC,
	VCODEC_RUNNING_MODE_RKVDEC,
	VCODEC_RUNNING_MODE_RKVENC
};

enum vcodec_device_id {
	VCODEC_DEVICE_ID_VPU,
	VCODEC_DEVICE_ID_HEVC,
	VCODEC_DEVICE_ID_COMBO,
	VCODEC_DEVICE_ID_RKVDEC,
	VCODEC_DEVICE_ID_RKVENC,
	VCODEC_DEVICE_ID_BUTT
};

enum vpu_ctx_state {
	MMU_ACTIVATED	= BIT(0),
	HW_RUNNING	= BIT(1)
};

struct extra_info_elem {
	u32 index;
	u32 offset;
};

#define EXTRA_INFO_MAGIC	0x4C4A46

struct extra_info_for_iommu {
	u32 magic;
	u32 cnt;
	struct extra_info_elem elem[20];
};

struct rockchip_mpp_dev_variant {
	u32 data_len;

	int (*hw_probe)(struct rockchip_mpp_dev *mpp);
	void (*power_on)(struct rockchip_mpp_dev *mpp);
	void (*power_off)(struct rockchip_mpp_dev *mpp);
};

struct rockchip_mpp_dev {
	struct mpp_dev_ops *ops;

	struct delayed_work power_off_work;

	struct cdev cdev;
	dev_t dev_t;
	struct class *cls;
	struct device *child_dev;

	int irq;
	struct mpp_service *srv;

	void __iomem *reg_base;
	enum VCODEC_RUNNING_MODE mode;
	struct list_head lnk_service;

	struct device *dev;

	unsigned long state;

	const struct rockchip_mpp_dev_variant *variant;

#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs_dir;
	struct dentry *debugfs_file_regs;
#endif

	struct device *mmu_dev;
};

struct mpp_service {
	struct wake_lock wake_lock;
	ktime_t last; /* record previous power-on time */
	/* vpu service structure global lock */
	struct mutex lock;
	/* link to link_reg in struct vpu_reg */
	struct list_head pending;
	/* link to link_reg in struct vpu_reg */
	struct list_head done;
	/* link to list_session in struct vpu_session */
	struct list_head session;
	atomic_t total_running;
	atomic_t enabled;
	atomic_t power_on_cnt;
	atomic_t power_off_cnt;
	struct mpp_ctx *current_ctx;

	struct device *dev;

	atomic_t reset_request;
	struct ion_client *ion_client;

	enum VCODEC_RUNNING_MODE curr_mode;
	u32 prev_mode;

	u32 mode_bit;
	u32 mode_ctrl;
	u32 *reg_base;
	struct regmap *grf;
	u32 *grf_base;

	u32 subcnt;
	struct list_head subdev_list;
};

/* Ops for common vcodec hardware */
struct mpp_ctx *vcodec_common_init(struct rockchip_mpp_dev *data,
				   void __user *src, u32 size);
int rockchip_mpp_common_try_run(struct rockchip_mpp_dev *data);
int vcodec_common_run(struct rockchip_mpp_dev *data);
int vcodec_common_done(struct rockchip_mpp_dev *data);
int vcodec_common_irq(struct rockchip_mpp_dev *data);
int rockchip_mpp_common_reset(struct rockchip_mpp_dev *mpp);
int rockchip_mpp_common_result(struct rockchip_mpp_dev *mpp,
			       struct mpp_ctx *ctx,
			       u32 __user *dst);
int vcodec_common_get_clk(struct mpp_service *pservice);

/* Ops for rkvenc */
void rockchip_mpp_rkvenc_remove(struct rockchip_mpp_dev *data);
struct mpp_ctx *rockchip_mpp_rkvenc_init(struct rockchip_mpp_dev *data,
					 void __user *src, u32 size);
int rockchip_mpp_rkvenc_prepare(struct rockchip_mpp_dev *data);
int rockchip_mpp_rkvenc_run(struct rockchip_mpp_dev *data);
int rockchip_mpp_rkvenc_done(struct rockchip_mpp_dev *data);
int rockchip_mpp_rkvenc_result(struct rockchip_mpp_dev *mpp,
			       struct mpp_ctx *ctx,
			       u32 __user *dst);
int rockchip_mpp_rkvenc_irq(struct rockchip_mpp_dev *data);

/**
 * struct mpp_dev_ops - hardware mode specific operations
 *
 * @init	Prepare for registers file for specific hardware.
 * @prepare	Check HW status for determining run next task or not.
 * @run		Start a single {en,de}coding run. Set registers to hardware.
 * @reset	Reset the HW.
 * @done	Read back processing results and additional data from hardware.
 * @result	Read status from HW.
 * @deinit	Release the resource allocate during init.
 * @ioctl	ioctl for special HW besides the service common ioctl.
 * @irq		interrupt service for specific hardware.
 * @open	special operation define when session open.
 * @release	Release the resource allocated when special hw session open.
 */
struct mpp_dev_ops {
	void (*remove)(struct rockchip_mpp_dev *mpp);
	struct mpp_ctx *(*init)(struct rockchip_mpp_dev *data,
				void __user *src, u32 size);
	int (*prepare)(struct rockchip_mpp_dev *data);
	int (*run)(struct rockchip_mpp_dev *data);
	int (*done)(struct rockchip_mpp_dev *data);
	int (*irq)(struct rockchip_mpp_dev *data);
	int (*reset)(struct rockchip_mpp_dev *mpp);
	int (*result)(struct rockchip_mpp_dev *mpp, struct mpp_ctx *ctx,
		      u32 __user *dst);
	void (*deinit)(struct rockchip_mpp_dev *mpp);
	long (*ioctl)(struct file *filp, unsigned int cmd, unsigned long arg);
	int (*open)(struct mpp_session **session,
		    struct rockchip_mpp_dev *mpp);
	int (*release)(struct mpp_session *session,
		       struct rockchip_mpp_dev *mpp);
};

void dump_reg(u32 *regs, int count);
void dump_reg_mem(u32 *regs, int count);
int mpp_reg_address_translate(struct rockchip_mpp_dev *data,
			      u32 *reg,
			      struct mpp_ctx *ctx,
			      const struct mpp_trans_info *info);
void translate_extra_info(struct mpp_ctx *ctx,
			  struct extra_info_for_iommu *ext_inf,
			  u32 *reg);

int mpp_dev_common_ctx_init(struct rockchip_mpp_dev *mpp, struct mpp_ctx *cfg);

static inline void mpp_write_relaxed(struct rockchip_mpp_dev *mpp,
				     u32 val, u32 reg)
{
	mpp_debug(DEBUG_SET_REG, "MARK: set reg[%03d]: %08x\n", reg / 4, val);
	writel_relaxed(val, mpp->reg_base + reg);
}

static inline u32 mpp_read(struct rockchip_mpp_dev *mpp, u32 reg)
{
	u32 val = readl(mpp->reg_base + reg);

	mpp_debug(DEBUG_GET_REG, "MARK: get reg[%03d] 0x%x: %08x\n", reg / 4,
		  reg, val);
	return val;
}

static inline void mpp_time_record(struct mpp_ctx *ctx)
{
	if (unlikely(debug & DEBUG_TIMING) && ctx)
		do_gettimeofday(&ctx->start);
}

static inline void mpp_time_diff(struct mpp_ctx *ctx)
{
	struct timeval end;

	do_gettimeofday(&end);
	mpp_debug(DEBUG_TIMING, "consume: %ld us\n",
		  (end.tv_sec  - ctx->start.tv_sec)  * 1000000 +
		  (end.tv_usec - ctx->start.tv_usec));
}

extern const struct rockchip_mpp_dev_variant rkvenc_variant;

#endif

# SPDX-License-Identifier: GPL-2.0
obj-y += common.o
obj-y += cpu.o
obj-$(CONFIG_CPU_RV110X) += rv1108.o
obj-$(CONFIG_CPU_RK3036) += rk3036.o
obj-$(CONFIG_CPU_RK312X) += rk312x.o
obj-$(CONFIG_CPU_RK312X) += rk3126b.o
obj-$(CONFIG_CPU_RK3188) += rk3188.o
obj-$(CONFIG_CPU_RK3288) += rk3288.o
obj-$(CONFIG_CPU_RK322X) += rk322x.o
obj-$(CONFIG_ARM_PSCI) += psci.o
obj-$(CONFIG_ARM_PSCI) += psci_ddr.o
obj-$(CONFIG_ARM_PSCI) += ddr_rk312x.o
ifneq ($(CONFIG_ARM_TRUSTZONE),y)
obj-y += ddr_freq.o
endif
obj-y += efuse.o
obj-y += pvtm.o
obj-y += rk_system_status.o
obj-$(CONFIG_PM) += rockchip_pm.o
ifeq ($(CONFIG_SUSPEND),y)
obj-y += sleep.o
obj-$(CONFIG_CPU_RK312X) += rk312x_sleep.o
obj-$(CONFIG_CPU_RV110X) += rv1108_sleep.o
endif
obj-$(CONFIG_RK_FPGA) += fpga.o
obj-$(CONFIG_CPU_IDLE) += cpuidle.o
obj-$(CONFIG_SMP) += platsmp.o
obj-$(CONFIG_HOTPLUG_CPU) += hotplug.o
obj-$(CONFIG_FIQ_DEBUGGER) += rk_fiq_debugger.o
obj-$(CONFIG_RK_LAST_LOG) += last_log.o
obj-$(CONFIG_DVFS) += dvfs.o
obj-$(CONFIG_RK_VCODEC) += vcodec_service.o
obj-$(CONFIG_RK_PL330_DMA_TEST) += dma_memcpy_test.o
obj-$(CONFIG_RK_SLT_DMA) += slt_dma.o
obj-$(CONFIG_BLOCK_RKNAND) += rknandbase.o
obj-$(CONFIG_DDR_TEST) += ddr_test.o
obj-$(CONFIG_RK_PM_TESTS) += rk_pm_tests/
obj-$(CONFIG_ROCK_CHIP_SOC_CAMERA) += rk_camera.o
obj-$(CONFIG_RK_DSP) += dsp/

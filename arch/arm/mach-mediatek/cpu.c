// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 MediaTek Inc.
 */

#include <asm/io.h>
#include <cpu_func.h>
#include <dm.h>
#include <init.h>
#include <wdt.h>
#include <dm/uclass-internal.h>
#include <linux/arm-smccc.h>
#include <linux/types.h>

#define MTK_SIP_PLAT_BINFO ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_64, \
					      ARM_SMCCC_OWNER_SIP, 0x529)

int arch_cpu_init(void)
{
#ifdef CONFIG_TARGET_MT6877
	/* WDT disable: prevent preloader's WDT from firing during u-boot init. */
	writel(0x22000000, (void __iomem *)0x10007000);
#endif

	icache_enable();

	return 0;
}

/* board_early_init_f no longer needed (bisection moved forward); removed */

void enable_caches(void)
{
#ifdef CONFIG_TARGET_MT6877
	/*
	 * v28: dcache_enable is disabled for MT6877. dcache_enable sets up
	 * MMU page tables (reserved by arm_reserve_mmu during board_init_f)
	 * and turns on MMU+dcache. In our setup the page-table memory lands
	 * in DRAM that the preloader has NOT mapped for the CPU, so the
	 * first access to the table hangs. v27 confirmed this with silent
	 * hang between initr_caches entry and exit.
	 *
	 * With dcache off, u-boot runs with only I-cache enabled (set in
	 * start.S). Slower but fine for fastboot/eMMC-flash usage.
	 */
	(void)0;
#else
	/* Enable D-cache. I-cache is already enabled in start.S */
	dcache_enable();
#endif
}

/**
 * mediatek_sip_part_name - get the part name
 *
 * Retrieve the part name of platform description.
 *
 * This only applicable to SoCs that support SIP plat binfo SMC call.
 *
 * Returns: the part name or 0 if error or no part name
 */
u32 mediatek_sip_part_name(void)
{
	if (CONFIG_IS_ENABLED(TARGET_MT8188) || CONFIG_IS_ENABLED(TARGET_MT8189) ||
	    CONFIG_IS_ENABLED(TARGET_MT8195) || CONFIG_IS_ENABLED(TARGET_MT8365)) {
		struct arm_smccc_res res __maybe_unused;

		arm_smccc_smc(MTK_SIP_PLAT_BINFO, 0, 0, 0, 0, 0, 0, 0, &res);
		if (res.a0)
			return 0;

		return res.a1;
	}

	return 0;
}

/**
 * mediatek_sip_segment_name - get the segment name
 *
 * Retrieve the segment name of platform description.
 *
 * This only applicable to SoCs that support SIP plat binfo SMC call.
 *
 * Returns: the segment name or 0 if error or no segment name
 */
u32 mediatek_sip_segment_name(void)
{
	if (CONFIG_IS_ENABLED(TARGET_MT8188) || CONFIG_IS_ENABLED(TARGET_MT8189) ||
	    CONFIG_IS_ENABLED(TARGET_MT8195) || CONFIG_IS_ENABLED(TARGET_MT8365)) {
		struct arm_smccc_res res __maybe_unused;

		arm_smccc_smc(MTK_SIP_PLAT_BINFO, 1, 0, 0, 0, 0, 0, 0, &res);
		if (res.a0)
			return 0;

		return res.a1;
	}

	return 0;
}

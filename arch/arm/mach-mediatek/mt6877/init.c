// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2026 theloop pmOS port
 *
 * MediaTek MT6877 (Dimensity 930) SoC init glue for u-boot.
 * Forked from arch/arm/mach-mediatek/mt8195/init.c — MT8195 and MT6877 share
 * the Cortex-A78+A55 cluster layout and MediaTek boot-chain conventions, so
 * the early SoC bringup code is nearly identical and only the identifying
 * string differs. DRAM base / MMIO layout come from Kconfig defaults
 * (MTK_MEM_MAP_DDR_BASE_PHY=0x40000000, DDR_SIZE=0x200000000).
 */

#include <asm/io.h>
#include <asm/system.h>
#include <dm/uclass.h>
#include <linux/kernel.h>
#include <linux/sizes.h>
#include <wdt.h>

#ifdef CONFIG_ARM64
#include <asm/armv8/mmu.h>
#else
#include <asm/psci.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

int dram_init(void)
{
	/*
	 * v19: override the DT-based memory size detection and hardcode a
	 * small safe-known-mapped DRAM range. Our DT claims 2 GiB but v18's
	 * silent hang (board_init not reached) strongly suggests relocation
	 * fails because gd->relocaddr lands in DRAM the preloader hasn't
	 * initialized.
	 *
	 * Preloader DEFINITELY has 0x40000000..0x50000000 (256 MiB) mapped
	 * — LK is loaded to 0x48200000 and its code runs from there, so the
	 * DRAM below+above it must be usable. Declare only that window; u-boot
	 * will relocate itself within it (typically to near top of ram_size,
	 * so ~0x4FF00000). Well below any possibly-unmapped region.
	 */
	gd->ram_base = 0x40000000;
	gd->ram_size = 0x10000000;	/* 256 MiB */
	return 0;
}

int board_init(void)
{
	return 0;
}

phys_size_t get_effective_memsize(void)
{
#ifdef CONFIG_ARM64
	/* Cap gd->ram_top at 4 GiB so 32-bit DMA peripherals (eMMC, USB) land
	 * within their addressable range. MT6877 physical DRAM extends above
	 * 4 GiB but those banks are for the kernel/userspace, not u-boot. */
	return min(SZ_4G - gd->ram_base, gd->ram_size);
#else
	/* AArch32 ulong is 32-bit so we can't even address above 4 GiB. Just
	 * use whatever DRAM size the DT declared (2 GiB in our DT, fits). */
	return gd->ram_size;
#endif
}

int mtk_soc_early_init(void)
{
	/*
	 * Disable MT6877 hardware watchdog IMMEDIATELY. Preloader leaves
	 * WDT armed with ~5-10s timeout; if any driver probe during
	 * u-boot init takes longer, WDT fires and the device resets back
	 * to BROM. With WDT off, a silently-hung driver probe leaves the
	 * device in a stable hung state so we can distinguish "u-boot
	 * runs but a probe hangs" from "u-boot crashes early".
	 *
	 * WDT_MODE @ 0x10007000. Key = 0x22 (upper 8 bits).
	 * WDT_EN = bit 0. Writing 0x22000000 (key + disable) turns it off.
	 * Verified working in flash 2: this exact write stopped the 7-8s
	 * watchdog-reset cycle.
	 */
	writel(0x22000000, (void __iomem *)0x10007000);

	return 0;
}

void reset_cpu(void)
{
#ifdef CONFIG_PSCI_RESET
	psci_system_reset();
#elif defined(CONFIG_WDT)
	struct udevice *wdt;

	uclass_first_device(UCLASS_WDT, &wdt);
	if (wdt)
		wdt_expire_now(wdt, 0);
#else
	/*
	 * Poke the MT6877 WDT directly to force a reset. With CONFIG_WDT
	 * disabled (so u-boot doesn't auto-start the watchdog during DM
	 * probing, which was masking panic-reset-cycles as 13s bootloops),
	 * we need a manual reset path here. Write WDT_SWRST = 0x1209 at
	 * WDT_BASE + 0x14 to trigger immediate reset.
	 */
	writel(0x1209, (void __iomem *)(0x10007000 + 0x14));
#endif
}

int print_cpuinfo(void)
{
	printf("CPU:   MediaTek MT6877 (Dimensity 930)\n");
	return 0;
}

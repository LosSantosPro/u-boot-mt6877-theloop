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
#include <env.h>
#include <fdtdec.h>
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
	 * Read ram_base + ram_size from the memory@40000000 DT node (2 GiB).
	 * theloop silicon is 4 GiB LPDDR4x, kernel/pmOS sees ~3.5 GiB after
	 * ATF/TEE/modem reservations; 2 GiB is plenty for u-boot's own use
	 * and get_effective_memsize() caps ram_top at SZ_4G - ram_base below.
	 *
	 * v19..v124 had this hardcoded to a defensive 256 MiB window because
	 * silent hangs before UART came up made it impossible to tell where
	 * relocation was landing. Now that the console is live and relocation
	 * demonstrably works, trust the DT like sibling MT8188/MT8195 do.
	 */
	return fdtdec_setup_mem_size_base();
}

int board_init(void)
{
	return 0;
}

/*
 * Probe MT6877 efuse for a per-die unique ID to use as fastboot serial#.
 * MT8791/MT6877 shares IP generation with MT8188 (efuse@11f20000) and
 * sits close to MT8183/MT8791 (efuse@11f10000). We don't have a
 * confirmed base for MT6877 yet so try both; first one that returns
 * sane (non-zero, non-0xffffffff) data wins. Efuse is auto-latched by
 * preloader so no clock/controller init is needed from here.
 *
 * Note: the first few words at offset 0 are control registers on most
 * MTK efuse controllers; the per-die data bank starts at a product-
 * specific offset. Without a datasheet we pull words 2+3 (offsets 8,
 * 0xc) which tend to hold non-control data across MTK generations.
 * Result is a 16-char hex string "W2W3" to loosely match the look of
 * stock SNs like "GNFMGBD3PCA000205" without pretending it's the same.
 */
static const char *theloop_probe_efuse_serial(char *buf, size_t buflen)
{
	static const u32 bases[] = { 0x11f20000, 0x11f10000 };
	u32 w2, w3;
	int i;

	for (i = 0; i < ARRAY_SIZE(bases); i++) {
		w2 = readl((void __iomem *)(uintptr_t)(bases[i] + 0x8));
		w3 = readl((void __iomem *)(uintptr_t)(bases[i] + 0xc));
		printf("theloop: efuse@%08x +8/+c = 0x%08x 0x%08x\n",
		       bases[i], w2, w3);
		if (w2 == 0 || w2 == 0xffffffff)
			continue;
		snprintf(buf, buflen, "%08X%08X", w2, w3);
		return buf;
	}
	return NULL;
}

int board_late_init(void)
{
	const char *sn;
	char snbuf[17];

	/*
	 * Populate env vars that fastboot getvar reads:
	 *   board    -> getvar "product"    (tb8791p1_64 matches stock LK)
	 *   platform -> getvar "platform"   (SoC codename)
	 *   serial#  -> getvar "serialno" and USB descriptor iSerialNumber
	 *
	 * Must live in board_late_init (not board_init) because env is not
	 * finalized until after board_init returns; values set earlier get
	 * wiped by env_relocate / env default-loading.
	 *
	 * serial# is derived from the SoC efuse per-die ID. Stock LK shows
	 * the factory-written SN from the nvram partition (format like
	 * GNFMGBD3PCA000205), which we cannot match exactly without parsing
	 * the SN1 blob. The efuse-derived hex string is still per-device
	 * and deterministic, so `fastboot devices` can disambiguate units.
	 */
	env_set("board", "tb8791p1_64");
	env_set("platform", "MT6877");

	sn = theloop_probe_efuse_serial(snbuf, sizeof(snbuf));
	if (sn) {
		env_set("serial#", sn);
		printf("theloop: serial# = %s\n", sn);
	} else {
		printf("theloop: efuse returned no usable serial; leaving unset\n");
	}

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

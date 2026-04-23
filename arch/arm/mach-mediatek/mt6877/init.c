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
 * Probe candidate MTK efuse base addresses to find MT6877's actual
 * efuse layout. Without a datasheet or UART reliably available, we
 * expose the readings via two fastboot getvar variables (efuse1 /
 * efuse2) so the host can query them. Each var carries two bases
 * worth of readings (words at +0x8 and +0xc) packed into the
 * 60-char fastboot response budget.
 *
 * Format per entry: "XX=WORD8WORDC" where XX is the top 8 bits of the
 * base address (e.g. "20" = 0x11f20000). Entries joined with ",".
 * Example: "20=1234567890ABCDEF,10=00000000FFFFFFFF" - base 0x11f20000
 * read 0x12345678 at +8 and 0x90ABCDEF at +c.
 *
 * Candidate bases (from other MTK DTSIs in upstream):
 *   0x11f20000 - MT8188, MT7981b
 *   0x11f10000 - MT8183 (MT8791 may inherit)
 *   0x11c10000 - MT8192, MT8195
 *   0x11cb0000 - MT8186
 * Efuse is auto-latched by preloader; no clock setup needed.
 */
struct efuse_probe {
	u32 base;
	u8  tag;	/* top byte of base for compact display */
};

static const struct efuse_probe efuse_set1[] = {
	{ 0x11f20000, 0x20 },	/* MT8188 family */
	{ 0x11f10000, 0x10 },	/* MT8183 family */
};
static const struct efuse_probe efuse_set2[] = {
	{ 0x11c10000, 0xc1 },	/* MT8192/8195 family */
	{ 0x11cb0000, 0xcb },	/* MT8186 family */
};

static void theloop_probe_and_pack(const struct efuse_probe *set, int n,
				   char *envname)
{
	char buf[64];
	int pos = 0;
	u32 w8, wc;
	int i;

	buf[0] = '\0';
	for (i = 0; i < n; i++) {
		w8 = readl((void __iomem *)(uintptr_t)(set[i].base + 0x8));
		wc = readl((void __iomem *)(uintptr_t)(set[i].base + 0xc));
		pos += snprintf(buf + pos, sizeof(buf) - pos,
				"%s%02X=%08X%08X",
				i ? "," : "", set[i].tag, w8, wc);
		if (pos >= sizeof(buf) - 1)
			break;
	}
	env_set(envname, buf);
}

int board_late_init(void)
{
	/*
	 * Populate env vars that fastboot getvar reads:
	 *   board    -> getvar "product"    (tb8791p1_64 matches stock LK)
	 *   platform -> getvar "platform"   (SoC codename)
	 *   efuse1   -> getvar "efuse1"     (debug: efuse probe set 1)
	 *   efuse2   -> getvar "efuse2"     (debug: efuse probe set 2)
	 *
	 * Must live in board_late_init (not board_init) because env is not
	 * finalized until board_init returns; values set earlier get wiped
	 * by env_relocate.
	 *
	 * serial# deliberately not set yet: we need the efuse1/efuse2
	 * readings to know which base address is correct for MT6877.
	 * Once that's confirmed, a follow-up build will set serial# to
	 * a hex string derived from the correct efuse data bank.
	 */
	env_set("board", "tb8791p1_64");
	env_set("platform", "MT6877");

	theloop_probe_and_pack(efuse_set1, ARRAY_SIZE(efuse_set1), "efuse1");
	theloop_probe_and_pack(efuse_set2, ARRAY_SIZE(efuse_set2), "efuse2");

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

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
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/sizes.h>
#include <power/regulator.h>
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
 * Phase 1 display bringup: power the LCM rails and pulse panel reset.
 *
 * MT6877 panel (from stock LK disassembly):
 *   IC:     HX8399-A (LK reports hx8399a for this SKU; preloader LCM
 *           name string shows hx8399c)
 *   GPIOs:  108 = LCM reset (active low)
 *           140 = LCM bias / control 1 (initial low)
 *           141 = LCM bias / control 2 (initial low)
 *   Rails:  VDDI  = MT6359P ldo_vio18 (1.8V)
 *           VMCH  = MT6359P ldo_vio28 (2.8V - closest analogue)
 *
 * MT6877 inherits the MT8188 pinctrl IP: five iocfg bases
 * (iocfg0 at 0x10005000, iocfg_rm 0x11c00000, iocfg_lt 0x11e10000,
 * iocfg_lm 0x11e20000, iocfg_rt 0x11ea0000), all referenced in stock
 * lk_a.bin. For GPIO direction + data-out pins 0..177 all live in
 * iocfg0, so a minimal direct-MMIO helper is enough for Phase 1
 * without pulling in the 1345-line MT8188 pinctrl driver wholesale.
 *
 * Register layout per MT8188 pinctrl (same IP block):
 *   iocfg0 + 0x300 + (pin/8)*0x10, bits (pin%8)*4, 4 bits   : mode
 *   iocfg0 + 0x000 + (pin/32)*0x10, bit (pin%32), 1 bit     : direction
 *   iocfg0 + 0x100 + (pin/32)*0x10, bit (pin%32), 1 bit     : data-out
 */
#define MT6877_IOCFG0_BASE	0x10005000

static void theloop_gpio_set_output(int pin, int value)
{
	void __iomem *iocfg0 = (void __iomem *)(uintptr_t)MT6877_IOCFG0_BASE;
	u32 reg, bit, val;

	/* Select GPIO function (mode 0) for this pin. 4 bits/pin. */
	reg = 0x300 + (pin / 8) * 0x10;
	bit = (pin % 8) * 4;
	val = readl(iocfg0 + reg);
	val &= ~(0xfU << bit);
	writel(val, iocfg0 + reg);

	/* Direction = output (1). 32 pins per register. */
	reg = (pin / 32) * 0x10;
	bit = pin % 32;
	val = readl(iocfg0 + reg);
	val |= (1U << bit);
	writel(val, iocfg0 + reg);

	/* Data-out value. */
	reg = 0x100 + (pin / 32) * 0x10;
	val = readl(iocfg0 + reg);
	if (value)
		val |= (1U << bit);
	else
		val &= ~(1U << bit);
	writel(val, iocfg0 + reg);
}

/*
 * Display subsystem power-on. Translated from Linux kernel's
 * drivers/soc/mediatek/mtk-scpsys.c + mtk-scpsys-mt6877.c.
 *
 * MT6877 SCPSYS (=SPM) is at 0x10006000. DISP power domain control
 * register is at ctl_offs=0x0E48. Status ACK bits are BIT(30) for
 * PWR_ON and BIT(31) for PWR_ON_2ND (in the ctl register itself,
 * not a separate status register).
 *
 * Bus protection for DISP is in infracfg at 0x1020e000 with three
 * masks covering steps 1, 2_0, and 2_1 (see scp_domain_data_mt6877
 * DIS0_PROT_STEP*_MASK).
 *
 * After this routine, MMSYS sub-modules (DSI@14013000, OVL@14005000,
 * etc.) become readable. Before this, they all read as 0 because the
 * display power domain is off and the bus protection bridge stops
 * the transaction.
 */
#define SPM_BASE		0x10006000
#define INFRACFG_AO_BASE	0x1020e000
#define MMSYS_BASE		0x14000000

#define DISP_PWR_CTL		(SPM_BASE + 0x0E48)
#define IFR_BP_CLR		(INFRACFG_AO_BASE + 0x02D8)
#define IFR_BP_STA		(INFRACFG_AO_BASE + 0x02D0)

#define PWR_RST_B_BIT		BIT(0)
#define PWR_ISO_BIT		BIT(1)
#define PWR_ON_BIT		BIT(2)
#define PWR_ON_2ND_BIT		BIT(3)
#define PWR_CLK_DIS_BIT		BIT(4)
#define PWR_SRAM_PDN_BIT	BIT(8)
#define PWR_SRAM_ACK_BIT	BIT(12)
#define PWR_ACK			BIT(30)
#define PWR_ACK_2ND		BIT(31)

/* From DIS0_PROT_STEP*_MASK in mtk-scpsys-mt6877.c */
#define DIS0_PROT_STEP1_0_MASK	(BIT(0) | BIT(2) | BIT(10) | BIT(12) | \
				 BIT(14) | BIT(16) | BIT(24) | BIT(26))
#define DIS0_PROT_STEP2_0_MASK	BIT(6)
#define DIS0_PROT_STEP2_1_MASK	(BIT(1) | BIT(3) | BIT(15) | BIT(17) | \
				 BIT(25) | BIT(27))

static int theloop_disp_domain_power_on(void)
{
	void __iomem *ctl = (void __iomem *)(uintptr_t)DISP_PWR_CTL;
	u32 val;
	int retries;

	val = readl(ctl);

	/* 1. Assert PWR_ON, wait for PWR_ACK */
	val |= PWR_ON_BIT;
	writel(val, ctl);
	for (retries = 1000; retries; retries--) {
		if (readl(ctl) & PWR_ACK)
			break;
		udelay(1);
	}
	if (!retries) {
		printf("theloop: DISP PWR_ON ack timeout\n");
		return -1;
	}

	udelay(50);

	/* 2. Assert PWR_ON_2ND, wait for PWR_ACK_2ND */
	val |= PWR_ON_2ND_BIT;
	writel(val, ctl);
	for (retries = 1000; retries; retries--) {
		if (readl(ctl) & PWR_ACK_2ND)
			break;
		udelay(1);
	}
	if (!retries) {
		printf("theloop: DISP PWR_ON_2ND ack timeout\n");
		return -2;
	}

	/* 3. De-assert PWR_CLK_DIS, PWR_ISO, assert PWR_RST_B */
	val &= ~PWR_CLK_DIS_BIT;
	writel(val, ctl);
	val &= ~PWR_ISO_BIT;
	writel(val, ctl);
	val |= PWR_RST_B_BIT;
	writel(val, ctl);

	/* 4. SRAM wake: clear PDN bit, wait for ACK clear */
	val &= ~PWR_SRAM_PDN_BIT;
	writel(val, ctl);
	for (retries = 1000; retries; retries--) {
		if (!(readl(ctl) & PWR_SRAM_ACK_BIT))
			break;
		udelay(1);
	}
	if (!retries) {
		printf("theloop: DISP SRAM wake timeout\n");
		return -3;
	}

	/* 5. Release bus protection (infracfg) in 3 steps. Each write
	 * clears the corresponding mask in the bus protect register. */
	writel(DIS0_PROT_STEP1_0_MASK,
	       (void __iomem *)(uintptr_t)IFR_BP_CLR);
	writel(DIS0_PROT_STEP2_0_MASK,
	       (void __iomem *)(uintptr_t)IFR_BP_CLR);
	writel(DIS0_PROT_STEP2_1_MASK,
	       (void __iomem *)(uintptr_t)IFR_BP_CLR);

	/* 6. Ungate MMSYS sub-module clocks. CG_CON0 at MMSYS+0x100
	 * (sta), 0x104 (set), 0x108 (clr). CG_CON1 at +0x1a0/a4/a8.
	 * Write the CLR regs with all-1s to enable every gate. */
	writel(0xffffffff, (void __iomem *)(uintptr_t)(MMSYS_BASE + 0x108));
	writel(0xffffffff, (void __iomem *)(uintptr_t)(MMSYS_BASE + 0x1a8));

	printf("theloop: DISP domain powered on, MMSYS clocks ungated\n");
	return 0;
}

static void theloop_panel_power_on(void)
{
	struct udevice *reg;
	int ret;

	/*
	 * Enable VDDI (1.8V display I/O) via MT6359P ldo_vio18. Usually
	 * already on from preloader state, but stock LK re-enables to be
	 * defensive.
	 */
	ret = uclass_get_device_by_name(UCLASS_REGULATOR, "ldo_vio18", &reg);
	if (!ret)
		regulator_set_enable(reg, true);
	else
		printf("theloop: ldo_vio18 not found (%d)\n", ret);

	/*
	 * Enable VMCH equivalent (2.8V LCM analogue) via ldo_vio28. The
	 * MT6359P doesn't have a dedicated "vmch" LDO; stock boards use
	 * vio28 as the 2.8V display supply.
	 */
	ret = uclass_get_device_by_name(UCLASS_REGULATOR, "ldo_vio28", &reg);
	if (!ret)
		regulator_set_enable(reg, true);
	else
		printf("theloop: ldo_vio28 not found (%d)\n", ret);

	mdelay(5);	/* rails settle */

	/* Bias / control: low per stock LK lcm_init */
	theloop_gpio_set_output(140, 0);
	theloop_gpio_set_output(141, 0);
	mdelay(5);

	/*
	 * Reset pulse on GPIO 108. HX8399 datasheet: RESX must be held
	 * low >= 10us, panel ready 120ms after release. Go conservative:
	 * 1ms low, 150ms wait after release.
	 */
	theloop_gpio_set_output(108, 1);
	mdelay(1);
	theloop_gpio_set_output(108, 0);
	mdelay(1);
	theloop_gpio_set_output(108, 1);
	mdelay(150);

	printf("theloop: panel rails + reset sequence done\n");
}

int board_late_init(void)
{
	/*
	 * Populate env vars that fastboot getvar reads:
	 *   board    -> getvar "product"    (tb8791p1_64 matches stock LK)
	 *   platform -> getvar "platform"   (SoC codename)
	 *
	 * Must live in board_late_init (not board_init) because env is not
	 * finalized until board_init returns; values set earlier get wiped
	 * by env_relocate.
	 *
	 * serial# is intentionally not set. Stock MTK LK reads the SN1 blob
	 * from the nvram partition (that is how a provisioned device shows
	 * e.g. "GNFMGBD3PCA000205"); replicating that would require MMC raw
	 * reads + SN1 format parsing from u-boot. Since pmOS userspace gets
	 * the real serial from the kernel nvram driver anyway, leaving
	 * serialno as "Value not set" at the fastboot stage is fine.
	 */
	env_set("board", "tb8791p1_64");
	env_set("platform", "MT6877");

	/*
	 * Display Phase 2 (power domain): turn on the MT6877 DISP
	 * power domain via SCPSYS and ungate MMSYS clocks. Without
	 * this, every display MMIO register (DSI @ 0x14013000, OVL @
	 * 0x14005000, etc.) reads as zero because the bus protection
	 * bridge blocks transactions.
	 */
	theloop_disp_domain_power_on();

	/*
	 * Display Phase 1: panel rails + reset. This only powers the
	 * panel and releases it from reset; no DSI output yet. Visible
	 * success:
	 *   - rail voltage present at panel connector (multimeter check)
	 *   - GPIO 108 data-out bit = 1 (reset released, panel idle)
	 *   - GPIO 140/141 data-out bits = 0
	 *
	 * Verify from fastboot without physical probing:
	 *   fastboot oem "run:md.l 0x10005130 1"  do for pins 96-127
	 *   fastboot oem "run:md.l 0x10005140 1"  do for pins 128-159
	 * Expect:
	 *   @0x10005130 bit 12 (pin 108) = 1
	 *   @0x10005140 bit 12 (pin 140) = 0
	 *   @0x10005140 bit 13 (pin 141) = 0
	 * Direction regs (0x10005030 / 0x10005040) should have the same
	 * bits set, marking those pins as outputs.
	 */
	theloop_panel_power_on();

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

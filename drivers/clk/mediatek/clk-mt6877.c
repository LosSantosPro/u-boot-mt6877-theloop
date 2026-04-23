// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT6877 (Dimensity 930) u-boot clock driver, minimum subset.
 *
 * This is the real MT6877 driver to replace the MT8188 stand-in that the
 * current u-boot scaffold binds to. Covers only what u-boot needs to boot
 * USB gadget / fastboot + eMMC / MSDC + (optional) UART. That's ~12 leaf
 * clocks out of MT6877's ~200, so the driver stays small.
 *
 * Scope:
 *   apmixedsys   USBPLL, MSDCPLL, MAINPLL, UNIVPLL
 *   topckgen     UART_SEL, USB_TOP_SEL, SSUSB_XHCI_SEL,
 *                MSDC50_0_SEL, MSDC50_0_HCLK_SEL, plus fixed divs
 *                chain to back each mux's parent options.
 *   infracfg_ao  UART0, MSDC0, MSDC0_SRC, MSDC0_AES, APDMA,
 *                SSUSB, SSUSB_XHCI gates.
 *
 * Out of scope for u-boot (but real on MT6877): MMPLL/ADSPPLL/TVDPLL/APLL*,
 * GPU MFG* clocks, image/video/camera/audio subsystems. These stay
 * unpopulated and u-boot will reject requests for clock IDs in those
 * ranges with -EINVAL, which is fine; no u-boot driver asks for them.
 *
 * Data sources (ground truth):
 *   kernel  drivers/clk/mediatek/clk-mt6877.c
 *   kernel  include/dt-bindings/clock/mt6877-clk.h
 *   kernel  arch/arm64/boot/dts/mediatek/mt6877.dts (for MMIO + DT refs)
 *
 * Format / framework borrowed from u-boot drivers/clk/mediatek/clk-mt8188.c.
 *
 *
 * HOW TO ACTIVATE THIS FILE:
 *   1. mv this file into u-boot tree:
 *        cp clk-mt6877.c.draft \
 *           u-boot/drivers/clk/mediatek/clk-mt6877.c
 *   2. Swap the Makefile line from
 *        obj-$(CONFIG_TARGET_MT6877) += clk-mt8188.o
 *      to
 *        obj-$(CONFIG_TARGET_MT6877) += clk-mt6877.o
 *   3. Add u-boot-side dt-bindings header
 *        include/dt-bindings/clock/mediatek,mt6877-clk.h
 *      (subset of kernel's include/dt-bindings/clock/mt6877-clk.h).
 *   4. Fix mt6877-theloop.dts to reference the right clock controllers
 *      (see PORT-PLAN-clk-mt6877.md, Phase A).
 */

#include <clk-uclass.h>
#include <dm.h>
#include <log.h>
#include <asm/arch-mediatek/reset.h>
#include <asm/io.h>
#include <dt-bindings/clock/mediatek,mt6877-clk.h>
#include <linux/bitops.h>

#include "clk-mtk.h"

#define MT6877_PLL_FMAX		(3800UL * MHZ)
#define MT6877_PLL_FMIN		(1500UL * MHZ)

/*
 * PLL() macro: expands each PLL() invocation into a fully-initialised
 * `struct mtk_pll_data`. Same shape as MT8188's macro, only the FMIN/FMAX
 * constants differ. `rst_bar_mask = BIT(23)` matters only when
 * CLK_PLL_HAVE_RST_BAR is set in flags (i.e. MAINPLL/UNIVPLL); for other
 * PLLs the field is ignored.
 */
#define PLL(_id, _reg, _pwr_reg, _en_mask, _flags, _pcwbits, _pd_reg,	\
	    _pd_shift, _pcw_reg, _pcw_shift) {				\
		.id = _id,						\
		.reg = _reg,						\
		.pwr_reg = _pwr_reg,					\
		.en_mask = _en_mask,					\
		.rst_bar_mask = BIT(23),				\
		.fmin = MT6877_PLL_FMIN,				\
		.fmax = MT6877_PLL_FMAX,				\
		.flags = _flags,					\
		.pcwbits = _pcwbits,					\
		.pcwibits = 8,						\
		.pd_reg = _pd_reg,					\
		.pd_shift = _pd_shift,					\
		.pcw_reg = _pcw_reg,					\
		.pcw_shift = _pcw_shift,				\
	}

/* ========= TOPCKGEN MUX REG OFFSETS (from kernel clk-mt6877.c) ========= */
#define CLK_CFG_UPDATE			0x0004
#define CLK_CFG_UPDATE1			0x0008

#define CLK_CFG_6			0x0070
#define CLK_CFG_6_SET			0x0074
#define CLK_CFG_6_CLR			0x0078
#define CLK_CFG_7			0x0080
#define CLK_CFG_7_SET			0x0084
#define CLK_CFG_7_CLR			0x0088
#define CLK_CFG_9			0x00A0
#define CLK_CFG_9_SET			0x00A4
#define CLK_CFG_9_CLR			0x00A8

/* Update-bit positions within CLK_CFG_UPDATE / UPDATE1 */
#define TOP_MUX_UART_SHIFT		24
#define TOP_MUX_MSDC50_0_HCLK_SHIFT	26
#define TOP_MUX_MSDC50_0_SHIFT		27
#define TOP_MUX_USB_TOP_SHIFT		5	/* in UPDATE1 */
#define TOP_MUX_SSUSB_XHCI_SHIFT	6	/* in UPDATE1 */

/* ========= APMIXED PLL REG OFFSETS (from kernel clk-mt6877.c) ========= */
#define MAINPLL_CON0			0x350
#define UNIVPLL_CON0			0x308
#define MSDCPLL_CON0			0x360
#define USBPLL_CON0			0x318

/* CON1/CON3 offsets are CON0 + 0x4 / + 0xC for each PLL. */
#define MAINPLL_CON1			(MAINPLL_CON0 + 0x4)
#define MAINPLL_CON3			(MAINPLL_CON0 + 0xC)
#define UNIVPLL_CON1			(UNIVPLL_CON0 + 0x4)
#define UNIVPLL_CON3			(UNIVPLL_CON0 + 0xC)
#define MSDCPLL_CON1			(MSDCPLL_CON0 + 0x4)
#define MSDCPLL_CON3			(MSDCPLL_CON0 + 0xC)
#define USBPLL_CON1			(USBPLL_CON0 + 0x4)
#define USBPLL_CON3			(USBPLL_CON0 + 0xC)

/* ========= EXTERNAL CLOCKS ========= */
/*
 * MT6877 only has a 26 MHz root on u-boot's critical path. 32 KHz RTC and
 * ultra-low-power oscillator exist but no u-boot consumer references them.
 */
#define CLK_EXT_CLK26M		0

static const ulong ext_clock_rates[] = {
	[CLK_EXT_CLK26M] = 26000000,
};

/* ========= APMIXED PLLs ========= */
/*
 * u-boot PLL(id, reg, pwr_reg, en_mask, flags, pcwbits,
 *           pd_reg, pd_shift, pcw_reg, pcw_shift).
 *
 * Kernel mt6877 driver uses:
 *   PLL(id, "name", CON0/*base*, CON0, BIT(0)/*en*,
 *       CON3/*pwr*, rstb_flags, rstb_mask,
 *       CON1, 24 /*pd*,
 *       22/*pcwbits*,
 *       CON1, 0, 22 /*pcw*)
 *
 * Translation: reg = CON0, pwr_reg = CON3, pd_reg = CON1 pd_shift = 24,
 * pcw_reg = CON1 pcw_shift = 0, pcwbits = 22.
 *
 * MAINPLL and UNIVPLL have RST_BAR bit at 23 per kernel driver.
 */
/*
 * en_mask = BIT(0) for every MT6877 PLL (verified against kernel
 * clk-mt6877.c). Earlier revisions here had en_mask = 0 for MSDCPLL /
 * USBPLL and 0xff000000 for MAINPLL / UNIVPLL; both values fail
 * to actually toggle the PLL enable bit, because u-boot's
 * mtk_apmixedsys_enable does `CON0 |= en_mask` and with the wrong mask
 * the write is a no-op. MAINPLL / UNIVPLL still worked because preloader
 * leaves them on (the kernel marks them PLL_AO), but USBPLL is NOT
 * preloader-enabled, so the SSUSB block had no ref_ck and all MAC reads
 * returned 0, which manifested as CAP_EPINFO = 0 and zero endpoints.
 */
static const struct mtk_pll_data apmixed_plls[] = {
	PLL(CLK_APMIXED_MAINPLL, MAINPLL_CON0, MAINPLL_CON3,
	    BIT(0), CLK_PLL_HAVE_RST_BAR, 22,
	    MAINPLL_CON1, 24, MAINPLL_CON1, 0),
	PLL(CLK_APMIXED_UNIVPLL, UNIVPLL_CON0, UNIVPLL_CON3,
	    BIT(0), CLK_PLL_HAVE_RST_BAR, 22,
	    UNIVPLL_CON1, 24, UNIVPLL_CON1, 0),
	PLL(CLK_APMIXED_MSDCPLL, MSDCPLL_CON0, MSDCPLL_CON3,
	    BIT(0), 0, 22,
	    MSDCPLL_CON1, 24, MSDCPLL_CON1, 0),
	PLL(CLK_APMIXED_USBPLL, USBPLL_CON0, USBPLL_CON3,
	    BIT(0), 0, 22,
	    USBPLL_CON1, 24, USBPLL_CON1, 0),
};

/*
 * ID remap for apmixedsys. MT6877's CLK_APMIXED_* IDs are sparse
 * (3, 4, 5, 12) but the u-boot framework indexes tables with dense
 * offsets. id_offs_map compresses sparse IDs to dense indices matching
 * apmixed_plls[] element order.
 */
static const int mt6877_id_apmixed_offs_map[CLK_APMIXED_NR_CLK] = {
	[0 ... CLK_APMIXED_NR_CLK - 1] = -1,
	[CLK_APMIXED_MAINPLL] = 0,
	[CLK_APMIXED_UNIVPLL] = 1,
	[CLK_APMIXED_MSDCPLL] = 2,
	[CLK_APMIXED_USBPLL]  = 3,
};

static const struct mtk_clk_tree mt6877_apmixedsys_clk_tree = {
	.pll_parent = EXT_PARENT(CLK_EXT_CLK26M),
	.ext_clk_rates = ext_clock_rates,
	.num_ext_clks = ARRAY_SIZE(ext_clock_rates),
	.id_offs_map = mt6877_id_apmixed_offs_map,
	.id_offs_map_size = ARRAY_SIZE(mt6877_id_apmixed_offs_map),
	.plls = apmixed_plls,
	.num_plls = ARRAY_SIZE(apmixed_plls),
};

/* ========= TOPCKGEN: fixed dividers only what muxes reference ========= */
/*
 * PLL -> TOP-level 1:1 handoff
 *   CLK_APMIXED_MAINPLL   -> CLK_TOP_MAINPLL (implicit via FACTOR)
 *
 * Then MAINPLL / UNIVPLL / MSDCPLL / OSC get chopped into dN and dN_dM
 * dividers that the muxes pick from. We only declare the dividers whose
 * outputs feed the five muxes we keep:
 *   uart_sel needs:  univpll_d6_d8
 *   msdc50_0_sel:    msdcpll_ck, msdcpll_d2, univpll_d4_d4,
 *                    mainpll_d6_d2, univpll_d4_d2
 *   msdc50_0_hclk_sel:  mainpll_d4_d2, mainpll_d6_d2
 *   usb_top_sel:     univpll_d5_d4, univpll_d6_d4, univpll_d5_d2
 *   ssusb_xhci_sel:  univpll_d5_d4, univpll_d6_d4, univpll_d5_d2
 */
static const struct mtk_fixed_factor top_fixed_divs[] = {
	/* mainpll family */
	FACTOR(CLK_TOP_MAINPLL_D4,    CLK_APMIXED_MAINPLL, 1, 4, CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_MAINPLL_D4_D2, CLK_TOP_MAINPLL_D4,  1, 2, CLK_PARENT_TOPCKGEN),
	FACTOR(CLK_TOP_MAINPLL_D6,    CLK_APMIXED_MAINPLL, 1, 6, CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_MAINPLL_D6_D2, CLK_TOP_MAINPLL_D6,  1, 2, CLK_PARENT_TOPCKGEN),

	/* univpll family */
	FACTOR(CLK_TOP_UNIVPLL_D4,    CLK_APMIXED_UNIVPLL, 1, 4, CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_UNIVPLL_D4_D2, CLK_TOP_UNIVPLL_D4,  1, 2, CLK_PARENT_TOPCKGEN),
	FACTOR(CLK_TOP_UNIVPLL_D4_D4, CLK_TOP_UNIVPLL_D4,  1, 4, CLK_PARENT_TOPCKGEN),
	FACTOR(CLK_TOP_UNIVPLL_D5,    CLK_APMIXED_UNIVPLL, 1, 5, CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_UNIVPLL_D5_D2, CLK_TOP_UNIVPLL_D5,  1, 2, CLK_PARENT_TOPCKGEN),
	FACTOR(CLK_TOP_UNIVPLL_D5_D4, CLK_TOP_UNIVPLL_D5,  1, 4, CLK_PARENT_TOPCKGEN),
	FACTOR(CLK_TOP_UNIVPLL_D6,    CLK_APMIXED_UNIVPLL, 1, 6, CLK_PARENT_APMIXED),
	FACTOR(CLK_TOP_UNIVPLL_D6_D4, CLK_TOP_UNIVPLL_D6,  1, 4, CLK_PARENT_TOPCKGEN),
	FACTOR(CLK_TOP_UNIVPLL_D6_D8, CLK_TOP_UNIVPLL_D6,  1, 8, CLK_PARENT_TOPCKGEN),

	/* msdcpll family */
	FACTOR(CLK_TOP_MSDCPLL_D2,    CLK_APMIXED_MSDCPLL, 1, 2, CLK_PARENT_APMIXED),
};

/* ========= TOPCKGEN: mux parent arrays ========= */
/*
 * Order in each array is authoritative. It's the bit-value written into
 * CLK_CFG_N[width]. Must match kernel driver exactly. "tck_26m_mx9_ck" in
 * the kernel is the same as our CLK_EXT_CLK26M in u-boot view.
 */
static const struct mtk_parent uart_parents[] = {
	EXT_PARENT(CLK_EXT_CLK26M),
	TOP_PARENT(CLK_TOP_UNIVPLL_D6_D8),
};

static const struct mtk_parent msdc5hclk_parents[] = {
	EXT_PARENT(CLK_EXT_CLK26M),
	TOP_PARENT(CLK_TOP_MAINPLL_D4_D2),
	TOP_PARENT(CLK_TOP_MAINPLL_D6_D2),
};

static const struct mtk_parent msdc50_0_parents[] = {
	EXT_PARENT(CLK_EXT_CLK26M),
	APMIXED_PARENT(CLK_APMIXED_MSDCPLL),
	TOP_PARENT(CLK_TOP_MSDCPLL_D2),
	TOP_PARENT(CLK_TOP_UNIVPLL_D4_D4),
	TOP_PARENT(CLK_TOP_MAINPLL_D6_D2),
	TOP_PARENT(CLK_TOP_UNIVPLL_D4_D2),
};

static const struct mtk_parent usb_parents[] = {
	EXT_PARENT(CLK_EXT_CLK26M),
	TOP_PARENT(CLK_TOP_UNIVPLL_D5_D4),
	TOP_PARENT(CLK_TOP_UNIVPLL_D6_D4),
	TOP_PARENT(CLK_TOP_UNIVPLL_D5_D2),
};

static const struct mtk_parent ssusb_xhci_parents[] = {
	EXT_PARENT(CLK_EXT_CLK26M),
	TOP_PARENT(CLK_TOP_UNIVPLL_D5_D4),
	TOP_PARENT(CLK_TOP_UNIVPLL_D6_D4),
	TOP_PARENT(CLK_TOP_UNIVPLL_D5_D2),
};

/* ========= TOPCKGEN: muxes ========= */
/*
 * Format:
 *   MUX_CLR_SET_UPD_FLAGS(id, parents, mux_reg, mux_set_reg, mux_clr_reg,
 *                         mux_shift, mux_width, gate_shift /* unused here,
 *                         upd_reg, upd_shift, flags)
 *
 * Kernel MUX_CLR_SET_UPD args: (id, name, parents, CLK_CFG_N, CLK_CFG_N_SET,
 *                               CLK_CFG_N_CLR, lsb, width, upd_ofs, upd_shift)
 */
static const struct mtk_composite top_muxes[] = {
	/* CLK_CFG_6 */
	MUX_CLR_SET_UPD_FLAGS(CLK_TOP_UART_SEL, uart_parents,
		CLK_CFG_6, CLK_CFG_6_SET, CLK_CFG_6_CLR,
		0, 1, -1, CLK_CFG_UPDATE, TOP_MUX_UART_SHIFT,
		CLK_MUX_SETCLR_UPD),
	MUX_CLR_SET_UPD_FLAGS(CLK_TOP_MSDC50_0_HCLK_SEL, msdc5hclk_parents,
		CLK_CFG_6, CLK_CFG_6_SET, CLK_CFG_6_CLR,
		16, 2, -1, CLK_CFG_UPDATE, TOP_MUX_MSDC50_0_HCLK_SHIFT,
		CLK_MUX_SETCLR_UPD),
	MUX_CLR_SET_UPD_FLAGS(CLK_TOP_MSDC50_0_SEL, msdc50_0_parents,
		CLK_CFG_6, CLK_CFG_6_SET, CLK_CFG_6_CLR,
		24, 3, -1, CLK_CFG_UPDATE, TOP_MUX_MSDC50_0_SHIFT,
		CLK_MUX_SETCLR_UPD),
	/* CLK_CFG_9 */
	MUX_CLR_SET_UPD_FLAGS(CLK_TOP_USB_TOP_SEL, usb_parents,
		CLK_CFG_9, CLK_CFG_9_SET, CLK_CFG_9_CLR,
		0, 2, -1, CLK_CFG_UPDATE1, TOP_MUX_USB_TOP_SHIFT,
		CLK_MUX_SETCLR_UPD),
	MUX_CLR_SET_UPD_FLAGS(CLK_TOP_SSUSB_XHCI_SEL, ssusb_xhci_parents,
		CLK_CFG_9, CLK_CFG_9_SET, CLK_CFG_9_CLR,
		8, 2, -1, CLK_CFG_UPDATE1, TOP_MUX_SSUSB_XHCI_SHIFT,
		CLK_MUX_SETCLR_UPD),
};

/*
 * ID remap for topckgen. Array indices must match element order within
 * top_fixed_divs[] (14 entries at mapped indices 0..13) and top_muxes[]
 * (5 entries at mapped indices 14..18).
 *
 * Unmapped indices (default -1) return -ENOENT from clk_get; that's the
 * correct behaviour for clock IDs u-boot doesn't care about.
 */
static const int mt6877_id_top_offs_map[CLK_TOP_NR_CLK] = {
	[0 ... CLK_TOP_NR_CLK - 1] = -1,
	/* fdivs, order matches top_fixed_divs[] */
	[CLK_TOP_MAINPLL_D4]      = 0,
	[CLK_TOP_MAINPLL_D4_D2]   = 1,
	[CLK_TOP_MAINPLL_D6]      = 2,
	[CLK_TOP_MAINPLL_D6_D2]   = 3,
	[CLK_TOP_UNIVPLL_D4]      = 4,
	[CLK_TOP_UNIVPLL_D4_D2]   = 5,
	[CLK_TOP_UNIVPLL_D4_D4]   = 6,
	[CLK_TOP_UNIVPLL_D5]      = 7,
	[CLK_TOP_UNIVPLL_D5_D2]   = 8,
	[CLK_TOP_UNIVPLL_D5_D4]   = 9,
	[CLK_TOP_UNIVPLL_D6]      = 10,
	[CLK_TOP_UNIVPLL_D6_D4]   = 11,
	[CLK_TOP_UNIVPLL_D6_D8]   = 12,
	[CLK_TOP_MSDCPLL_D2]      = 13,
	/* muxes, order matches top_muxes[] */
	[CLK_TOP_UART_SEL]        = 14,
	[CLK_TOP_MSDC50_0_HCLK_SEL] = 15,
	[CLK_TOP_MSDC50_0_SEL]    = 16,
	[CLK_TOP_USB_TOP_SEL]     = 17,
	[CLK_TOP_SSUSB_XHCI_SEL]  = 18,
};

static const struct mtk_clk_tree mt6877_topckgen_clk_tree = {
	.ext_clk_rates = ext_clock_rates,
	.num_ext_clks = ARRAY_SIZE(ext_clock_rates),
	.id_offs_map = mt6877_id_top_offs_map,
	.id_offs_map_size = ARRAY_SIZE(mt6877_id_top_offs_map),
	.fdivs_offs = 0,
	.muxes_offs = 14,
	.fdivs = top_fixed_divs,
	.num_fdivs = ARRAY_SIZE(top_fixed_divs),
	.muxes = top_muxes,
	.num_muxes = ARRAY_SIZE(top_muxes),
};

/* ========= INFRACFG_AO: gate banks ========= */
/*
 * Verified identical to MT8188 at the register-frame level (same MTK
 * infracfg_ao IP). We only declare bank 0, 1, 2 since those are where
 * our gates live.
 */
static const struct mtk_gate_regs ifrao0_cg_regs = {
	.set_ofs = 0x80,
	.clr_ofs = 0x84,
	.sta_ofs = 0x90,
};

static const struct mtk_gate_regs ifrao1_cg_regs = {
	.set_ofs = 0x88,
	.clr_ofs = 0x8c,
	.sta_ofs = 0x94,
};

static const struct mtk_gate_regs ifrao2_cg_regs = {
	.set_ofs = 0xa4,
	.clr_ofs = 0xa8,
	.sta_ofs = 0xac,
};

#define GATE_IFRAO0(_id, _parent, _shift) {			\
		.id = _id,					\
		.parent = _parent,				\
		.regs = &ifrao0_cg_regs,			\
		.shift = _shift,				\
		.flags = CLK_GATE_SETCLR | CLK_PARENT_TOPCKGEN,	\
	}

#define GATE_IFRAO1(_id, _parent, _shift) {			\
		.id = _id,					\
		.parent = _parent,				\
		.regs = &ifrao1_cg_regs,			\
		.shift = _shift,				\
		.flags = CLK_GATE_SETCLR | CLK_PARENT_TOPCKGEN,	\
	}

#define GATE_IFRAO2(_id, _parent, _shift) {			\
		.id = _id,					\
		.parent = _parent,				\
		.regs = &ifrao2_cg_regs,			\
		.shift = _shift,				\
		.flags = CLK_GATE_SETCLR | CLK_PARENT_TOPCKGEN,	\
	}

/* ========= INFRACFG_AO: gate list ========= */
/*
 * Shift numbers from kernel clk-mt6877.c (lines ~2008, ~2023, ~2052):
 *   bank 0:  UART0=15
 *   bank 1:  MSDC0=3 (via "axi_ck" parent), MSDC0_SRC=6 (msdc50_0_ck),
 *            MSDC0_AES=17 (msdc50_0_ck)
 *   bank 2:  SSUSB=1 (fusb_ck -> usb_top_sel), APDMA=18 (axi_ck),
 *            SSUSB_XHCI=31 (fssusb_xhci_ck -> ssusb_xhci_sel)
 */
/*
 * Parent IDs here only matter for clk_get_rate() back-chaining. The gate
 * enable path only writes set_ofs[shift]. Kernel uses "axi_ck" as the
 * parent for MSDC0 and APDMA gates; we'd need to also bring AXI_SEL mux
 * + AXI factor into the tree to let rate lookups walk that chain. For the
 * fastboot-only path nothing calls clk_get_rate on these gates, so for
 * the initial port we point MSDC0/APDMA at MSDC50_0_SEL and USB_TOP_SEL
 * respectively. Lets rate lookups at least return a non-bogus value if
 * we ever start polling, and keeps the tree self-contained. Revisit when
 * we port the AXI mux.
 */
static const struct mtk_gate infracfg_ao_clks[] = {
	/* IFRAO0: PMIC_TMR bit 0, PMIC_AP bit 1, UART0 bit 15 */
	GATE_IFRAO0(CLK_IFRAO_PMIC_TMR,   CLK_TOP_UART_SEL,          0),
	GATE_IFRAO0(CLK_IFRAO_PMIC_AP,    CLK_TOP_UART_SEL,          1),
	GATE_IFRAO0(CLK_IFRAO_UART0,      CLK_TOP_UART_SEL,         15),
	/* IFRAO1 */
	GATE_IFRAO1(CLK_IFRAO_MSDC0,      CLK_TOP_MSDC50_0_HCLK_SEL, 3),
	GATE_IFRAO1(CLK_IFRAO_MSDC0_SRC,  CLK_TOP_MSDC50_0_SEL,      6),
	GATE_IFRAO1(CLK_IFRAO_MSDC0_AES,  CLK_TOP_MSDC50_0_SEL,     17),
	/* IFRAO2 */
	GATE_IFRAO2(CLK_IFRAO_SSUSB,      CLK_TOP_USB_TOP_SEL,       1),
	GATE_IFRAO2(CLK_IFRAO_APDMA,      CLK_TOP_USB_TOP_SEL,      18),
	GATE_IFRAO2(CLK_IFRAO_SSUSB_XHCI, CLK_TOP_SSUSB_XHCI_SEL,   31),
};

/*
 * ID remap for infracfg_ao gates. Order must match infracfg_ao_clks[].
 */
static const int mt6877_id_infra_ao_offs_map[CLK_IFRAO_NR_CLK] = {
	[0 ... CLK_IFRAO_NR_CLK - 1] = -1,
	[CLK_IFRAO_PMIC_TMR]   = 0,
	[CLK_IFRAO_PMIC_AP]    = 1,
	[CLK_IFRAO_UART0]      = 2,
	[CLK_IFRAO_MSDC0]      = 3,
	[CLK_IFRAO_MSDC0_SRC]  = 4,
	[CLK_IFRAO_MSDC0_AES]  = 5,
	[CLK_IFRAO_SSUSB]      = 6,
	[CLK_IFRAO_APDMA]      = 7,
	[CLK_IFRAO_SSUSB_XHCI] = 8,
};

static const struct mtk_clk_tree mt6877_infracfg_ao_clk_tree = {
	.ext_clk_rates = ext_clock_rates,
	.num_ext_clks = ARRAY_SIZE(ext_clock_rates),
	.id_offs_map = mt6877_id_infra_ao_offs_map,
	.id_offs_map_size = ARRAY_SIZE(mt6877_id_infra_ao_offs_map),
	.gates_offs = 0,
};

/* ========= Probe / match ========= */

static int mt6877_apmixedsys_probe(struct udevice *dev)
{
	return mtk_common_clk_init(dev, &mt6877_apmixedsys_clk_tree);
}

static int mt6877_topckgen_probe(struct udevice *dev)
{
	return mtk_common_clk_init(dev, &mt6877_topckgen_clk_tree);
}

static int mt6877_infracfg_ao_probe(struct udevice *dev)
{
	return mtk_common_clk_gate_init(dev, &mt6877_infracfg_ao_clk_tree,
					infracfg_ao_clks,
					ARRAY_SIZE(infracfg_ao_clks), 0);
}

/*
 * MT6877 kernel DT uses "mediatek,mt6877-apmixedsys" (+ "syscon"),
 * "mediatek,mt6877-topckgen" (+ "syscon"),
 * "mediatek,mt6877-infracfg_ao" (+ "syscon"). Note the underscore in
 * infracfg_ao. Kernel source uses that form consistently, so we match it.
 */
static const struct udevice_id mt6877_apmixed_compat[] = {
	{ .compatible = "mediatek,mt6877-apmixedsys", },
	{ }
};

static const struct udevice_id mt6877_topckgen_compat[] = {
	{ .compatible = "mediatek,mt6877-topckgen", },
	{ }
};

static const struct udevice_id mt6877_infracfg_ao_compat[] = {
	{ .compatible = "mediatek,mt6877-infracfg_ao", },
	{ }
};

/*
 * U_BOOT_DRIVER symbol names must be `mtk_clk_apmixedsys` /
 * `mtk_clk_topckgen` / `mtk_clk_infracfg_ao` verbatim. The framework
 * (drivers/clk/mediatek/clk-mtk.c) does `DM_DRIVER_GET(mtk_clk_topckgen)`
 * and `DM_DRIVER_GET(mtk_clk_apmixedsys)` to locate these by name when
 * walking the parent chain during rate lookups. Since only one MTK
 * target is built at a time, generic names don't collide.
 */
U_BOOT_DRIVER(mtk_clk_apmixedsys) = {
	.name = "mt6877-apmixedsys",
	.id = UCLASS_CLK,
	.of_match = mt6877_apmixed_compat,
	.probe = mt6877_apmixedsys_probe,
	.priv_auto = sizeof(struct mtk_clk_priv),
	.ops = &mtk_clk_apmixedsys_ops,
	.flags = DM_FLAG_PRE_RELOC,
};

U_BOOT_DRIVER(mtk_clk_topckgen) = {
	.name = "mt6877-topckgen",
	.id = UCLASS_CLK,
	.of_match = mt6877_topckgen_compat,
	.probe = mt6877_topckgen_probe,
	.priv_auto = sizeof(struct mtk_clk_priv),
	.ops = &mtk_clk_topckgen_ops,
	.flags = DM_FLAG_PRE_RELOC,
};

U_BOOT_DRIVER(mtk_clk_infracfg_ao) = {
	.name = "mt6877-infracfg-ao",
	.id = UCLASS_CLK,
	.of_match = mt6877_infracfg_ao_compat,
	.probe = mt6877_infracfg_ao_probe,
	.priv_auto = sizeof(struct mtk_cg_priv),
	.ops = &mtk_clk_gate_ops,
	.flags = DM_FLAG_PRE_RELOC,
};

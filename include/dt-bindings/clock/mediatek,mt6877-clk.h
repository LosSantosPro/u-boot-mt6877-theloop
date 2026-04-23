/* SPDX-License-Identifier: GPL-2.0 */
/*
 * U-boot-side subset of kernel include/dt-bindings/clock/mt6877-clk.h
 *
 * Only the clock IDs the u-boot fastboot + MSDC + (optional) UART path
 * actually references. Kept numerically identical to kernel so DT source
 * can later be unified with upstream.
 *
 * Drop this file at:
 *   u-boot/include/dt-bindings/clock/mediatek,mt6877-clk.h
 * Then `#include <dt-bindings/clock/mediatek,mt6877-clk.h>` from the
 * u-boot clk-mt6877.c driver and from mt6877-theloop.dts.
 */

#ifndef _DT_BINDINGS_CLK_MEDIATEK_MT6877_H
#define _DT_BINDINGS_CLK_MEDIATEK_MT6877_H

/* TOPCKGEN: fixed dividers we use as mux parents */
#define CLK_TOP_MAINPLL_D4		2
#define CLK_TOP_MAINPLL_D4_D2		3
#define CLK_TOP_MAINPLL_D6		11
#define CLK_TOP_MAINPLL_D6_D2		12
#define CLK_TOP_UNIVPLL			20
#define CLK_TOP_UNIVPLL_D4		21
#define CLK_TOP_UNIVPLL_D4_D2		22
#define CLK_TOP_UNIVPLL_D4_D4		23
#define CLK_TOP_UNIVPLL_D5		25
#define CLK_TOP_UNIVPLL_D5_D2		26
#define CLK_TOP_UNIVPLL_D5_D4		27
#define CLK_TOP_UNIVPLL_D6		28
#define CLK_TOP_UNIVPLL_D6_D4		30
#define CLK_TOP_UNIVPLL_D6_D8		31
#define CLK_TOP_MSDCPLL			63
#define CLK_TOP_MSDCPLL_D2		64

/* TOPCKGEN: muxes */
#define CLK_TOP_AXI_SEL			122
#define CLK_TOP_UART_SEL		143
#define CLK_TOP_MSDC50_0_HCLK_SEL	145
#define CLK_TOP_MSDC50_0_SEL		146
#define CLK_TOP_USB_TOP_SEL		154
#define CLK_TOP_SSUSB_XHCI_SEL		155

/* INFRACFG_AO: leaf gates u-boot enables */
#define CLK_IFRAO_PMIC_TMR		0
#define CLK_IFRAO_PMIC_AP		1
#define CLK_IFRAO_UART0			15
#define CLK_IFRAO_MSDC0			22
#define CLK_IFRAO_MSDC0_SRC		24
#define CLK_IFRAO_MSDC0_AES		31
#define CLK_IFRAO_SSUSB			36
#define CLK_IFRAO_APDMA			45
#define CLK_IFRAO_SSUSB_XHCI		51

/* APMIXEDSYS: PLLs */
#define CLK_APMIXED_MAINPLL		3
#define CLK_APMIXED_UNIVPLL		4
#define CLK_APMIXED_MSDCPLL		5
#define CLK_APMIXED_USBPLL		12
#define CLK_APMIXED_NR_CLK		13

/*
 * NR_CLK values for each controller. Used to size the id_offs_map arrays
 * in clk-mt6877.c. Match kernel mt6877-clk.h even though u-boot only
 * populates a sparse subset of the IDs.
 */
#define CLK_TOP_NR_CLK			206
#define CLK_IFRAO_NR_CLK		63

#endif /* _DT_BINDINGS_CLK_MEDIATEK_MT6877_H */

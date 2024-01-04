/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Clock identifiers generated from MCUX configuration data for LPC55S69 */


#define LPC_CLOCK_PLL(div, mult, pdec, selr, seli, selp) \
			(((div & 0xFF) << 24) | ((mult & 0xFFFF) << 8) | \
			((pdec & 0xFF))) \
			(((selr & 0xF) << 20) | ((seli & 0x3F) << 13) | \
			((selp & 0x1F) << 8))
#define LPC_CLOCK_PLL_DIV(pll) ((pll >> 56) & 0xFF)
#define LPC_CLOCK_PLL_MULT(pll) ((pll >> 40) & 0xFFFF)
#define LPC_CLOCK_PLL_PDEC(pll) ((pll >> 32) & 0xFF)
#define LPC_CLOCK_PLL_SELR(pll) ((pll >> 20) & 0xF)
#define LPC_CLOCK_PLL_SELI(pll) ((pll >> 13) & 0x3F)
#define LPC_CLOCK_PLL_SELP(pll) ((pll >> 8) & 0x1F)

/* clock_selects */
#define MAINCLKSELA_FRO_12M 0
#define MAINCLKSELA_CLK_IN_EN 1
#define MAINCLKSELA_FRO_1M 2
#define MAINCLKSELA_FRO_HF 3
#define RTCOSC32KSEL_FRO_32K 0
#define RTCOSC32KSEL_XTAL32K 1
#define PLL0CLKSEL_FRO_12M 0
#define PLL0CLKSEL_CLK_IN_EN 1
#define PLL0CLKSEL_FRO_1M 2
#define PLL0CLKSEL_RTCOSC32KSEL 3
#define PLL0CLKSEL_NO_CLOCK 7
#define PLL0_DIRECTO_PLL0_PDEC 0
#define PLL0_DIRECTO_PLL0 1
#define PLL0_BYPASS_PLL0_DIRECTO 0
#define PLL0_BYPASS_PLL0CLKSEL 1
#define PLL1CLKSEL_FRO_12M 0
#define PLL1CLKSEL_CLK_IN_EN 1
#define PLL1CLKSEL_FRO_1M 2
#define PLL1CLKSEL_RTCOSC32KSEL 3
#define PLL1CLKSEL_NO_CLOCK 7
#define PLL1_DIRECTO_PLL1_PDEC 0
#define PLL1_DIRECTO_PLL1 1
#define PLL1_BYPASS_PLL1_DIRECTO 0
#define PLL1_BYPASS_PLL1CLKSEL 1
#define MAINCLKSELB_MAINCLKSELA 0
#define MAINCLKSELB_PLL0_BYPASS 1
#define MAINCLKSELB_PLL1_BYPASS 2
#define MAINCLKSELB_RTCOSC32KSEL 3
#define TRACECLKSEL_TRACECLKDIV 0
#define TRACECLKSEL_FRO_1M 1
#define TRACECLKSEL_RTCOSC32KSEL 2
#define TRACECLKSEL_NO_CLOCK 7
#define SYSTICKCLKSEL0_SYSTICKCLKDIV0 0
#define SYSTICKCLKSEL0_FRO_1M 1
#define SYSTICKCLKSEL0_RTCOSC32KSEL 2
#define SYSTICKCLKSEL0_NO_CLOCK 7
#define SYSTICKCLKSEL1_SYSTICKCLKDIV1 0
#define SYSTICKCLKSEL1_FRO_1M 1
#define SYSTICKCLKSEL1_RTCOSC32KSEL 2
#define SYSTICKCLKSEL1_NO_CLOCK 7
#define ADCCLKSEL_MAINCLKSELB 0
#define ADCCLKSEL_PLL0_BYPASS 1
#define ADCCLKSEL_FRO_HF 2
#define ADCCLKSEL_NO_CLOCK 7
#define USB0CLKSEL_MAINCLKSELB 0
#define USB0CLKSEL_PLL0_BYPASS 1
#define USB0CLKSEL_NO_CLOCK 7
#define USB0CLKSEL_FRO_HF 3
#define USB0CLKSEL_PLL1_BYPASS 5
#define MCLKCLKSEL_FRO_HF 0
#define MCLKCLKSEL_PLL0_BYPASS 1
#define MCLKCLKSEL_NO_CLOCK 7
#define SCTCLKSEL_MAINCLKSELB 0
#define SCTCLKSEL_PLL0_BYPASS 1
#define SCTCLKSEL_CLK_IN_EN 2
#define SCTCLKSEL_FRO_HF 3
#define SCTCLKSEL_NO_CLOCK 7
#define SCTCLKSEL_MCLK_IN 5
#define CLKOUTSEL_MAINCLKSELB 0
#define CLKOUTSEL_PLL0_BYPASS 1
#define CLKOUTSEL_CLK_IN_EN 2
#define CLKOUTSEL_FRO_HF 3
#define CLKOUTSEL_FRO_1M 4
#define CLKOUTSEL_PLL1_BYPASS 5
#define CLKOUTSEL_RTCOSC32KSEL 6
#define CLKOUTSEL_NO_CLOCK 7
#define SDIOCLKSEL_MAINCLKSELB 0
#define SDIOCLKSEL_PLL0_BYPASS 1
#define SDIOCLKSEL_NO_CLOCK 7
#define SDIOCLKSEL_FRO_HF 3
#define SDIOCLKSEL_PLL1_BYPASS 5
#define CTIMERCLKSEL0_MAINCLKSELB 0
#define CTIMERCLKSEL0_PLL0_BYPASS 1
#define CTIMERCLKSEL0_NO_CLOCK 7
#define CTIMERCLKSEL0_FRO_HF 3
#define CTIMERCLKSEL0_FRO_1M 4
#define CTIMERCLKSEL0_MCLK_IN 5
#define CTIMERCLKSEL0_RTCOSC32KSEL 6
#define CTIMERCLKSEL1_MAINCLKSELB 0
#define CTIMERCLKSEL1_PLL0_BYPASS 1
#define CTIMERCLKSEL1_NO_CLOCK 7
#define CTIMERCLKSEL1_FRO_HF 3
#define CTIMERCLKSEL1_FRO_1M 4
#define CTIMERCLKSEL1_MCLK_IN 5
#define CTIMERCLKSEL1_RTCOSC32KSEL 6
#define CTIMERCLKSEL2_MAINCLKSELB 0
#define CTIMERCLKSEL2_PLL0_BYPASS 1
#define CTIMERCLKSEL2_NO_CLOCK 7
#define CTIMERCLKSEL2_FRO_HF 3
#define CTIMERCLKSEL2_FRO_1M 4
#define CTIMERCLKSEL2_MCLK_IN 5
#define CTIMERCLKSEL2_RTCOSC32KSEL 6
#define CTIMERCLKSEL3_MAINCLKSELB 0
#define CTIMERCLKSEL3_PLL0_BYPASS 1
#define CTIMERCLKSEL3_NO_CLOCK 7
#define CTIMERCLKSEL3_FRO_HF 3
#define CTIMERCLKSEL3_FRO_1M 4
#define CTIMERCLKSEL3_MCLK_IN 5
#define CTIMERCLKSEL3_RTCOSC32KSEL 6
#define CTIMERCLKSEL4_MAINCLKSELB 0
#define CTIMERCLKSEL4_PLL0_BYPASS 1
#define CTIMERCLKSEL4_NO_CLOCK 7
#define CTIMERCLKSEL4_FRO_HF 3
#define CTIMERCLKSEL4_FRO_1M 4
#define CTIMERCLKSEL4_MCLK_IN 5
#define CTIMERCLKSEL4_RTCOSC32KSEL 6
#define FCCLKSEL0_MAINCLKSELB 0
#define FCCLKSEL0_PLL0DIV 1
#define FCCLKSEL0_FRO_12M 2
#define FCCLKSEL0_FROHFDIV 3
#define FCCLKSEL0_FRO_1M 4
#define FCCLKSEL0_MCLK_IN 5
#define FCCLKSEL0_RTCOSC32KSEL 6
#define FCCLKSEL0_NO_CLOCK 7
#define FCCLKSEL1_MAINCLKSELB 0
#define FCCLKSEL1_PLL0DIV 1
#define FCCLKSEL1_FRO_12M 2
#define FCCLKSEL1_FROHFDIV 3
#define FCCLKSEL1_FRO_1M 4
#define FCCLKSEL1_MCLK_IN 5
#define FCCLKSEL1_RTCOSC32KSEL 6
#define FCCLKSEL1_NO_CLOCK 7
#define FCCLKSEL2_MAINCLKSELB 0
#define FCCLKSEL2_PLL0DIV 1
#define FCCLKSEL2_FRO_12M 2
#define FCCLKSEL2_FROHFDIV 3
#define FCCLKSEL2_FRO_1M 4
#define FCCLKSEL2_MCLK_IN 5
#define FCCLKSEL2_RTCOSC32KSEL 6
#define FCCLKSEL2_NO_CLOCK 7
#define FCCLKSEL3_MAINCLKSELB 0
#define FCCLKSEL3_PLL0DIV 1
#define FCCLKSEL3_FRO_12M 2
#define FCCLKSEL3_FROHFDIV 3
#define FCCLKSEL3_FRO_1M 4
#define FCCLKSEL3_MCLK_IN 5
#define FCCLKSEL3_RTCOSC32KSEL 6
#define FCCLKSEL3_NO_CLOCK 7
#define FCCLKSEL4_MAINCLKSELB 0
#define FCCLKSEL4_PLL0DIV 1
#define FCCLKSEL4_FRO_12M 2
#define FCCLKSEL4_FROHFDIV 3
#define FCCLKSEL4_FRO_1M 4
#define FCCLKSEL4_MCLK_IN 5
#define FCCLKSEL4_RTCOSC32KSEL 6
#define FCCLKSEL4_NO_CLOCK 7
#define FCCLKSEL5_MAINCLKSELB 0
#define FCCLKSEL5_PLL0DIV 1
#define FCCLKSEL5_FRO_12M 2
#define FCCLKSEL5_FROHFDIV 3
#define FCCLKSEL5_FRO_1M 4
#define FCCLKSEL5_MCLK_IN 5
#define FCCLKSEL5_RTCOSC32KSEL 6
#define FCCLKSEL5_NO_CLOCK 7
#define FCCLKSEL6_MAINCLKSELB 0
#define FCCLKSEL6_PLL0DIV 1
#define FCCLKSEL6_FRO_12M 2
#define FCCLKSEL6_FROHFDIV 3
#define FCCLKSEL6_FRO_1M 4
#define FCCLKSEL6_MCLK_IN 5
#define FCCLKSEL6_RTCOSC32KSEL 6
#define FCCLKSEL6_NO_CLOCK 7
#define FCCLKSEL7_MAINCLKSELB 0
#define FCCLKSEL7_PLL0DIV 1
#define FCCLKSEL7_FRO_12M 2
#define FCCLKSEL7_FROHFDIV 3
#define FCCLKSEL7_FRO_1M 4
#define FCCLKSEL7_MCLK_IN 5
#define FCCLKSEL7_RTCOSC32KSEL 6
#define FCCLKSEL7_NO_CLOCK 7
#define HSLSPICLKSEL_MAINCLKSELB 0
#define HSLSPICLKSEL_PLL0DIV 1
#define HSLSPICLKSEL_FRO_12M 2
#define HSLSPICLKSEL_FROHFDIV 3
#define HSLSPICLKSEL_FRO_1M 4
#define HSLSPICLKSEL_NO_CLOCK 7
#define HSLSPICLKSEL_RTCOSC32KSEL 6

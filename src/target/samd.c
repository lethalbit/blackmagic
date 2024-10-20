/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2014  Richard Meadows <richardeoin>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * This file implements Atmel SAM D target specific functions for
 * detecting the device, providing the XML memory map and Flash memory
 * programming.
 *
 * Tested with
 * * SAMD09D14A (rev B)
 * * SAMD20E17A (rev C)
 * * SAMD20J18A (rev B)
 * * SAMD21J18A (rev B)
 * * SAML21J17B (rev B)
 * * SAMC21N18A (rev E)
 * * PIC32CM1216MC00048 (rev B)
 */

/*
 * Refer to the SAM D20 Datasheet:
 * http://www.atmel.com/Images/Atmel-42129-SAM-D20_Datasheet.pdf
 * particularly sections 12. DSU and 20. NVMCTRL
 */

#include "general.h"
#include "target.h"
#include "target_internal.h"
#include "cortexm.h"
#include "spi.h"

#define SAMD_FLASH_BANK_BASE 0x00000000U
#define SAMD_FLASH_BANK_SIZE 0x00004000U

#define SAMD_SRAM_BASE 0x20000000U
#define SAMD_SRAM_SIZE 0x00001000U

#define SAMD_SQUISHY_FLASH_BASE 0x10000000U
#define SAMD_SQUISHY_FLASH_SIZE 0x04000000U // 64Mib

// SAMD Power Management
#define SAMD_PM_BASE     0x40000400U
#define SAMD_PM_CTRL     (SAMD_PM_BASE + 0x000U)
#define SAMD_PM_SLEEP    (SAMD_PM_BASE + 0x001U)
#define SAMD_PM_EXTCTRL  (SAMD_PM_BASE + 0x002U)
#define SAMD_PM_CPUSEL   (SAMD_PM_BASE + 0x008U)
#define SAMD_PM_APBASEL  (SAMD_PM_BASE + 0x009U)
#define SAMD_PM_APBBSEL  (SAMD_PM_BASE + 0x00aU)
#define SAMD_PM_APBCSEL  (SAMD_PM_BASE + 0x00cU)
#define SAMD_PM_AHBMASK  (SAMD_PM_BASE + 0x014U)
#define SAMD_PM_APBAMASK (SAMD_PM_BASE + 0x018U)
#define SAMD_PM_APBBMASK (SAMD_PM_BASE + 0x01CU)
#define SAMD_PM_APBCMASK (SAMD_PM_BASE + 0x020U)
#define SAMD_PM_INTENCLR (SAMD_PM_BASE + 0x034U)
#define SAMD_PM_INTENSET (SAMD_PM_BASE + 0x035U)
#define SAMD_PM_INTFLAG  (SAMD_PM_BASE + 0x036U)
#define SAMD_PM_RCAUSE   (SAMD_PM_BASE + 0x038U)

#define SAMD_PM_CTRL_CFDEN   (1U << 2U)
#define SAMD_PM_CTRL_BKUPCLK (1U << 2U)

#define SAMD_PM_SLEEP_IDLE_MASK (0x3f << 0U)
#define SAMD_PM_SLEEP_IDLE_CPU  (0x0U << 0U)
#define SAMD_PM_SLEEP_IDLE_AHB  (0x1U << 0U)
#define SAMD_PM_SLEEP_IDLE_APB  (0x2U << 0U)

#define SAMD_PM_EXTCTRL_SETDIS (1U << 0U)

#define SAMD_PM_CPUSEL_CPUDIV_MASK   (0x7U << 0U)
#define SAMD_PM_CPUSEL_CPUDIV_DIV1   (0x0U << 0U)
#define SAMD_PM_CPUSEL_CPUDIV_DIV2   (0x1U << 0U)
#define SAMD_PM_CPUSEL_CPUDIV_DIV4   (0x2U << 0U)
#define SAMD_PM_CPUSEL_CPUDIV_DIV8   (0x3U << 0U)
#define SAMD_PM_CPUSEL_CPUDIV_DIV16  (0x4U << 0U)
#define SAMD_PM_CPUSEL_CPUDIV_DIV32  (0x5U << 0U)
#define SAMD_PM_CPUSEL_CPUDIV_DIV64  (0x6U << 0U)
#define SAMD_PM_CPUSEL_CPUDIV_DIV128 (0x7U << 0U)

#define SAMD_PM_APBASEL_APBADIV_MASK   (0x7U << 0U)
#define SAMD_PM_APBASEL_APBADIV_DIV1   (0x0U << 0U)
#define SAMD_PM_APBASEL_APBADIV_DIV2   (0x1U << 0U)
#define SAMD_PM_APBASEL_APBADIV_DIV4   (0x2U << 0U)
#define SAMD_PM_APBASEL_APBADIV_DIV8   (0x3U << 0U)
#define SAMD_PM_APBASEL_APBADIV_DIV16  (0x4U << 0U)
#define SAMD_PM_APBASEL_APBADIV_DIV32  (0x5U << 0U)
#define SAMD_PM_APBASEL_APBADIV_DIV64  (0x6U << 0U)
#define SAMD_PM_APBASEL_APBADIV_DIV128 (0x7U << 0U)

#define SAMD_PM_APBBSEL_APBADIV_MASK   (0x7U << 0U)
#define SAMD_PM_APBBSEL_APBADIV_DIV1   (0x0U << 0U)
#define SAMD_PM_APBBSEL_APBADIV_DIV2   (0x1U << 0U)
#define SAMD_PM_APBBSEL_APBADIV_DIV4   (0x2U << 0U)
#define SAMD_PM_APBBSEL_APBADIV_DIV8   (0x3U << 0U)
#define SAMD_PM_APBBSEL_APBADIV_DIV16  (0x4U << 0U)
#define SAMD_PM_APBBSEL_APBADIV_DIV32  (0x5U << 0U)
#define SAMD_PM_APBBSEL_APBADIV_DIV64  (0x6U << 0U)
#define SAMD_PM_APBBSEL_APBADIV_DIV128 (0x7U << 0U)

#define SAMD_PM_APBCSEL_APBADIV_MASK   (0x7U << 0U)
#define SAMD_PM_APBCSEL_APBADIV_DIV1   (0x0U << 0U)
#define SAMD_PM_APBCSEL_APBADIV_DIV2   (0x1U << 0U)
#define SAMD_PM_APBCSEL_APBADIV_DIV4   (0x2U << 0U)
#define SAMD_PM_APBCSEL_APBADIV_DIV8   (0x3U << 0U)
#define SAMD_PM_APBCSEL_APBADIV_DIV16  (0x4U << 0U)
#define SAMD_PM_APBCSEL_APBADIV_DIV32  (0x5U << 0U)
#define SAMD_PM_APBCSEL_APBADIV_DIV64  (0x6U << 0U)
#define SAMD_PM_APBCSEL_APBADIV_DIV128 (0x7U << 0U)

#define SAMD_PM_AHBMASK_HPB0    (1U << 0U)
#define SAMD_PM_AHBMASK_HPB1    (1U << 1U)
#define SAMD_PM_AHBMASK_HPB2    (1U << 2U)
#define SAMD_PM_AHBMASK_DSU     (1U << 3U)
#define SAMD_PM_AHBMASK_NVMCTRL (1U << 4U)
#define SAMD_PM_AHBMASK_DMAC    (1U << 5U)

#define SAMD_PM_APBAMASK_PAC0    (1U << 0U)
#define SAMD_PM_APBAMASK_PM      (1U << 1U)
#define SAMD_PM_APBAMASK_SYSCTRL (1U << 2U)
#define SAMD_PM_APBAMASK_GCLK    (1U << 3U)
#define SAMD_PM_APBAMASK_WDT     (1U << 4U)
#define SAMD_PM_APBAMASK_RTC     (1U << 5U)
#define SAMD_PM_APBAMASK_EIC     (1U << 6U)

#define SAMD_PM_APBBMASK_PAC1   (1U << 0U)
#define SAMD_PM_APBBMASK_DSU    (1U << 1U)
#define SAMD_PM_APBBMASK_NVMTRL (1U << 2U)
#define SAMD_PM_APBBMASK_PORT   (1U << 3U)
#define SAMD_PM_APBBMASK_DMAC   (1U << 4U)

#define SAMD_PM_APBCMASK_PAC2    (1U << 0U)
#define SAMD_PM_APBCMASK_EVSYS   (1U << 1U)
#define SAMD_PM_APBCMASK_SERCOM0 (1U << 2U)
#define SAMD_PM_APBCMASK_SERCOM1 (1U << 3U)
#define SAMD_PM_APBCMASK_TC1     (1U << 6U)
#define SAMD_PM_APBCMASK_TC2     (1U << 7U)
#define SAMD_PM_APBCMASK_ADC     (1U << 8U)

#define SAMD_PM_INTENCLR_CKRDY (1U << 0U)
#define SAMD_PM_INTENCLR_CFD   (1U << 1U)

#define SAMD_PM_INTENSET_CKRDY (1U << 0U)
#define SAMD_PM_INTENSET_CFD   (1U << 1U)

#define SAMD_PM_INTFLAG_CKRDY (1U << 0U)
#define SAMD_PM_INTFLAG_CFD   (1U << 1U)

#define SAMD_PM_RCAUSE_POR   (1U << 0U)
#define SAMD_PM_RCAUSE_BOD12 (1U << 1U)
#define SAMD_PM_RCAUSE_BOD33 (1U << 2U)
#define SAMD_PM_RCAUSE_EXT   (1U << 4U)
#define SAMD_PM_RCAUSE_WDT   (1U << 5U)
#define SAMD_PM_RCAUSE_SYST  (1U << 6U)

// SAMD Global Clocking
#define SAMD_GCLK_BASE    0x40000C00
#define SAMD_GCLK_CTRL    (SAMD_GCLK_BASE + 0x000U)
#define SAMD_GCLK_STATUS  (SAMD_GCLK_BASE + 0x001U)
#define SAMD_GCLK_CLKCTRL (SAMD_GCLK_BASE + 0x002U)
#define SAMD_GCLK_GENCTRL (SAMD_GCLK_BASE + 0x004U)
#define SAMD_GCLK_GENDIV  (SAMD_GCLK_BASE + 0x008U)

#define SAMD_GCLK_CTRL_SWRST (1U << 0U)

#define SAMD_GCLK_STATUS_SYNCBUSY (1U << 7U)

#define SAMD_GCLK_CLKCTRL_ID_MASK                 (0x3fU << 0U)
#define SAMD_GCLK_CLKCTRL_ID_GCLK_DFLL48M_REF     (0x00U << 0U)
#define SAMD_GCLK_CLKCTRL_ID_GCLK_DPLL            (0x01U << 0U)
#define SAMD_GCLK_CLKCTRL_ID_GCLK_DPLL_32K        (0x02U << 0U)
#define SAMD_GCLK_CLKCTRL_ID_GCLK_WDT             (0x03U << 0U)
#define SAMD_GCLK_CLKCTRL_ID_GCLK_RTC             (0x04U << 0U)
#define SAMD_GCLK_CLKCTRL_ID_GCLK_EIC             (0x05U << 0U)
#define SAMD_GCLK_CLKCTRL_ID_GCLK_EVSYS_CHANNEL_0 (0x07U << 0U)
#define SAMD_GCLK_CLKCTRL_ID_GCLK_EVSYS_CHANNEL_1 (0x08U << 0U)
#define SAMD_GCLK_CLKCTRL_ID_GCLK_EVSYS_CHANNEL_2 (0x09U << 0U)
#define SAMD_GCLK_CLKCTRL_ID_GCLK_EVSYS_CHANNEL_3 (0x0aU << 0U)
#define SAMD_GCLK_CLKCTRL_ID_GCLK_EVSYS_CHANNEL_4 (0x0bU << 0U)
#define SAMD_GCLK_CLKCTRL_ID_GCLK_EVSYS_CHANNEL_5 (0x0cU << 0U)
#define SAMD_GCLK_CLKCTRL_ID_GCLK_SERCOMx_SLOW    (0x0dU << 0U)
#define SAMD_GCLK_CLKCTRL_ID_GCLK_SERCOM0_CORE    (0x0eU << 0U)
#define SAMD_GCLK_CLKCTRL_ID_GCLK_SERCOM1_CORE    (0x0fU << 0U)
#define SAMD_GCLK_CLKCTRL_ID_GCLK_TC2             (0x12U << 0U)
#define SAMD_GCLK_CLKCTRL_ID_GCLK_ADC             (0x13U << 0U)

#define SAMD_GCLK_CLKCTRL_GEN_MASK  (0x0fU << 8U)
#define SAMD_GCLK_CLKCTRL_GEN_GCLK0 (0x00U << 8U)
#define SAMD_GCLK_CLKCTRL_GEN_GCLK1 (0x01U << 8U)
#define SAMD_GCLK_CLKCTRL_GEN_GCLK2 (0x02U << 8U)
#define SAMD_GCLK_CLKCTRL_GEN_GCLK3 (0x03U << 8U)
#define SAMD_GCLK_CLKCTRL_GEN_GCLK4 (0x04U << 8U)
#define SAMD_GCLK_CLKCTRL_GEN_GCLK5 (0x05U << 8U)

#define SAMD_GCLK_CLKCTRL_CLKEN   (1U << 14U)
#define SAMD_GCLK_CLKCTRL_WRTLOCK (1U << 15U)

#define SAMD_GCLK_GENCTRL_ID_MASK    (0xfU << 0U)
#define SAMD_GCLK_GENCTRL_ID_GCLKEN0 (0x0U << 0U)
#define SAMD_GCLK_GENCTRL_ID_GCLKEN1 (0x1U << 0U)
#define SAMD_GCLK_GENCTRL_ID_GCLKEN2 (0x2U << 0U)
#define SAMD_GCLK_GENCTRL_ID_GCLKEN3 (0x3U << 0U)
#define SAMD_GCLK_GENCTRL_ID_GCLKEN4 (0x4U << 0U)
#define SAMD_GCLK_GENCTRL_ID_GCLKEN5 (0x5U << 0U)

#define SAMD_GCLK_GENCTRL_SRC_MASK      (0x1fU << 8U)
#define SAMD_GCLK_GENCTRL_SRC_XOSC      (0x00U << 8U)
#define SAMD_GCLK_GENCTRL_SRC_GCLKIN    (0x01U << 8U)
#define SAMD_GCLK_GENCTRL_SRC_GCLKGEN1  (0x02U << 8U)
#define SAMD_GCLK_GENCTRL_SRC_OSCULP32K (0x03U << 8U)
#define SAMD_GCLK_GENCTRL_SRC_OSC32K    (0x04U << 8U)
#define SAMD_GCLK_GENCTRL_SRC_XOSC32K   (0x05U << 8U)
#define SAMD_GCLK_GENCTRL_SRC_OSC8M     (0x06U << 8U)
#define SAMD_GCLK_GENCTRL_SRC_DFLL48M   (0x07U << 8U)
#define SAMD_GCLK_GENCTRL_SRC_FDPLL96M  (0x08U << 8U)

#define SAMD_GCLK_GENCTRL_GENEN   (1U << 16U)
#define SAMD_GCLK_GENCTRL_IDC     (1U << 17U)
#define SAMD_GCLK_GENCTRL_OOV     (1U << 18U)
#define SAMD_GCLK_GENCTRL_OE      (1U << 19U)
#define SAMD_GCLK_GENCTRL_DIVSEL  (1U << 20U)
#define SAMD_GCLK_GENCTRL_RUNSTBY (1U << 21U)

#define SAMD_GCLK_GENDIV_ID_MASK    (0xfU << 0U)
#define SAMD_GCLK_GENDIV_ID_GCLKEN0 (0x0U << 0U)
#define SAMD_GCLK_GENDIV_ID_GCLKEN1 (0x1U << 0U)
#define SAMD_GCLK_GENDIV_ID_GCLKEN2 (0x2U << 0U)
#define SAMD_GCLK_GENDIV_ID_GCLKEN3 (0x3U << 0U)
#define SAMD_GCLK_GENDIV_ID_GCLKEN4 (0x4U << 0U)
#define SAMD_GCLK_GENDIV_ID_GCLKEN5 (0x5U << 0U)

#define SAMD_GCLK_GENDIV_DIV_MASK (0xffffU << 8U)

// SAMD SERCOMs
#define SAMD_SERCOM0_BASE 0x42000800U
#define SAMD_SERCOM1_BASE 0x42000C00U

#define SAMD_SERCOMx_CTRLA(base)    (base + 0x000U)
#define SAMD_SERCOMx_CTRLB(base)    (base + 0x004U)
#define SAMD_SERCOMx_BAUD(base)     (base + 0x00cU)
#define SAMD_SERCOMx_INTENCLR(base) (base + 0x014U)
#define SAMD_SERCOMx_INTENSET(base) (base + 0x016U)
#define SAMD_SERCOMx_INTFLAG(base)  (base + 0x018U)
#define SAMD_SERCOMx_STATUS(base)   (base + 0x01aU)
#define SAMD_SERCOMx_SYNCBUSY(base) (base + 0x01cU)
#define SAMD_SERCOMx_ADDR(base)     (base + 0x024U)
#define SAMD_SERCOMx_DATA(base)     (base + 0x028U)
#define SAMD_SERCOMx_DBGCTRL(base)  (base + 0x030U)

#define SAMD_SERCOMx_CTRLA_SWRST           (1U << 0U)
#define SAMD_SERCOMx_CTRLA_ENABLE          (1U << 1U)
#define SAMD_SERCOMx_CTRLA_MODE_MASK       (0x7U << 2U)
#define SAMD_SERCOMx_CTRLA_MODE_PERIPHERAL (0x2U << 2U)
#define SAMD_SERCOMx_CTRLA_MODE_CONTROLLER (0x3U << 2U)
#define SAMD_SERCOMx_CTRLA_RUNSTDBY        (1U << 7U)
#define SAMD_SERCOMx_CTRLA_IBON            (1U << 8U)
#define SAMD_SERCOMx_CTRLA_DOPO_MASK       (0x3U << 16U)
#define SAMD_SERCOMx_CTRLA_DOPO_0          (0x0U << 16U) // PAD[0] = DO, PAD[1] = SCK, PAD[2] = Peripheral_SS
#define SAMD_SERCOMx_CTRLA_DOPO_1          (0x1U << 16U) // PAD[2] = DO, PAD[3] = SCK, PAD[1] = Peripheral_SS
#define SAMD_SERCOMx_CTRLA_DOPO_2          (0x2U << 16U) // PAD[3] = DO, PAD[1] = SCK, PAD[2] = Peripheral_SS
#define SAMD_SERCOMx_CTRLA_DOPO_3          (0x3U << 16U) // PAD[0] = DO, PAD[3] = SCK, PAD[1] = Peripheral_SS
#define SAMD_SERCOMx_CTRLA_DIPO_MASK       (0x3U << 20U)
#define SAMD_SERCOMx_CTRLA_DIPO_0          (0x0U << 20U) // PAD[0] = DI
#define SAMD_SERCOMx_CTRLA_DIPO_1          (0x1U << 20U) // PAD[1] = DI
#define SAMD_SERCOMx_CTRLA_DIPO_2          (0x2U << 20U) // PAD[2] = DI
#define SAMD_SERCOMx_CTRLA_DIPO_3          (0x3U << 20U) // PAD[3] = DI
#define SAMD_SERCOMx_CTRLA_FORM_MASK       (0xfU << 24U)
#define SAMD_SERCOMx_CTRLA_FORM_SPI        (0x0U << 24U) // SPI Frame
#define SAMD_SERCOMx_CTRLA_FORM_SPI_ADDR   (0x2U << 24U) // SPI Frame w/ Addr
#define SAMD_SERCOMx_CTRLA_CPHA            (1U << 28)    // Clock Phase: 0 Leading, 1 Trailing
#define SAMD_SERCOMx_CTRLA_CPOL            (1U << 29)    // Clock Polarity: 0 Low, 1 High
#define SAMD_SERCOMx_CTRLA_DORD            (1U << 30)    // Data Order: 0 MSB, 1 LSB

#define SAMD_SERCOMx_CTRLB_CHSIZE_MASK    (0x7U << 0U)
#define SAMD_SERCOMx_CTRLB_CHSIZE_8BIT    (0x0U << 0U)
#define SAMD_SERCOMx_CTRLB_CHSIZE_9BIT    (0x1U << 0U)
#define SAMD_SERCOMx_CTRLB_PLOADEN        (1U << 6U)
#define SAMD_SERCOMx_CTRLB_SSDE           (1U << 9U)
#define SAMD_SERCOMx_CTRLB_MSSEN          (1U << 13U)
#define SAMD_SERCOMx_CTRLB_AMODE_MASK     (0x3U << 14U)
#define SAMD_SERCOMx_CTRLB_AMODE_ADDRMASK (0x0U << 14U)
#define SAMD_SERCOMx_CTRLB_AMODE_2ADDRS   (0x1U << 14U)
#define SAMD_SERCOMx_CTRLB_AMODE_RANGE    (0x2U << 14U)
#define SAMD_SERCOMx_CTRLB_RXEN           (1U << 17U)

#define SAMD_SERCOMx_INTCLR_DRE   (1U << 0U)
#define SAMD_SERCOMx_INTCLR_TXC   (1U << 1U)
#define SAMD_SERCOMx_INTCLR_RXC   (1U << 2U)
#define SAMD_SERCOMx_INTCLR_SSL   (1U << 3U)
#define SAMD_SERCOMx_INTCLR_ERROR (1U << 7U)

#define SAMD_SERCOMx_INTSET_DRE   (1U << 0U)
#define SAMD_SERCOMx_INTSET_TXC   (1U << 1U)
#define SAMD_SERCOMx_INTSET_RXC   (1U << 2U)
#define SAMD_SERCOMx_INTSET_SSL   (1U << 3U)
#define SAMD_SERCOMx_INTSET_ERROR (1U << 7U)

#define SAMD_SERCOMx_INTFLAG_DRE   (1U << 0U)
#define SAMD_SERCOMx_INTFLAG_TXC   (1U << 1U)
#define SAMD_SERCOMx_INTFLAG_RXC   (1U << 2U)
#define SAMD_SERCOMx_INTFLAG_SSL   (1U << 3U)
#define SAMD_SERCOMx_INTFLAG_ERROR (1U << 7U)

#define SAMD_SERCOMx_STATUS_BUFOVF (1U << 2U)

#define SAMD_SERCOMx_SYNCBUSY_SWRST  (1U << 0U)
#define SAMD_SERCOMx_SYNCBUSY_ENABLE (1U << 1U)
#define SAMD_SERCOMx_SYNCBUSY_CTRLB  (1U << 2U)

#define SAMD_SERCOMx_ADDR_ADDR_MASK     (0xfU << 0U)
#define SAMD_SERCOMx_ADDR_ADDRMASK_MASK (0xfU << 16U)

#define SAMD_SERCOMx_DATA_DATA_MASK (0x10U << 0U)

#define SAMD_SERCOMx_DBGCTRL_DBSTOP (1U << 0U) // 1 Freeze BAUD when DBG, 0 Don't

// SAMD Ports

#define SAMD_PORTx_BASE           0x41004400U
#define SAMD_PORT_A               0x00U
#define SAMD_PORT_B               0x80U
#define SAMD_PORTx_DIR(port)      (SAMD_PORTx_BASE + port + 0x000U)
#define SAMD_PORTx_DIRCLR(port)   (SAMD_PORTx_BASE + port + 0x004U)
#define SAMD_PORTx_DIRSET(port)   (SAMD_PORTx_BASE + port + 0x008U)
#define SAMD_PORTx_DIRTGL(port)   (SAMD_PORTx_BASE + port + 0x00cU)
#define SAMD_PORTx_OUT(port)      (SAMD_PORTx_BASE + port + 0x010U)
#define SAMD_PORTx_OUTCLR(port)   (SAMD_PORTx_BASE + port + 0x014U)
#define SAMD_PORTx_OUTSET(port)   (SAMD_PORTx_BASE + port + 0x018U)
#define SAMD_PORTx_OUTTGL(port)   (SAMD_PORTx_BASE + port + 0x01cU)
#define SAMD_PORTx_IN(port)       (SAMD_PORTx_BASE + port + 0x020U)
#define SAMD_PORTx_CTRL(port)     (SAMD_PORTx_BASE + port + 0x024U)
#define SAMD_PORTx_WRCONFIG(port) (SAMD_PORTx_BASE + port + 0x028U)
#define SAMD_PORTx_PMUX0(port)    (SAMD_PORTx_BASE + port + 0x030U)
#define SAMD_PORTx_PMUX1(port)    (SAMD_PORTx_BASE + port + 0x031U)
#define SAMD_PORTx_PMUX2(port)    (SAMD_PORTx_BASE + port + 0x032U)
#define SAMD_PORTx_PMUX3(port)    (SAMD_PORTx_BASE + port + 0x033U)
#define SAMD_PORTx_PMUX4(port)    (SAMD_PORTx_BASE + port + 0x034U)
#define SAMD_PORTx_PMUX5(port)    (SAMD_PORTx_BASE + port + 0x035U)
#define SAMD_PORTx_PMUX6(port)    (SAMD_PORTx_BASE + port + 0x036U)
#define SAMD_PORTx_PMUX7(port)    (SAMD_PORTx_BASE + port + 0x037U)
#define SAMD_PORTx_PMUX8(port)    (SAMD_PORTx_BASE + port + 0x038U)
#define SAMD_PORTx_PMUX9(port)    (SAMD_PORTx_BASE + port + 0x039U)
#define SAMD_PORTx_PMUX10(port)   (SAMD_PORTx_BASE + port + 0x03aU)
#define SAMD_PORTx_PMUX11(port)   (SAMD_PORTx_BASE + port + 0x03bU)
#define SAMD_PORTx_PMUX12(port)   (SAMD_PORTx_BASE + port + 0x03cU)
#define SAMD_PORTx_PMUX13(port)   (SAMD_PORTx_BASE + port + 0x03dU)
#define SAMD_PORTx_PMUX14(port)   (SAMD_PORTx_BASE + port + 0x03eU)
#define SAMD_PORTx_PMUX15(port)   (SAMD_PORTx_BASE + port + 0x03fU)
#define SAMD_PORTx_PINCFG0(port)  (SAMD_PORTx_BASE + port + 0x040U)
#define SAMD_PORTx_PINCFG1(port)  (SAMD_PORTx_BASE + port + 0x041U)
#define SAMD_PORTx_PINCFG2(port)  (SAMD_PORTx_BASE + port + 0x042U)
#define SAMD_PORTx_PINCFG3(port)  (SAMD_PORTx_BASE + port + 0x043U)
#define SAMD_PORTx_PINCFG4(port)  (SAMD_PORTx_BASE + port + 0x044U)
#define SAMD_PORTx_PINCFG5(port)  (SAMD_PORTx_BASE + port + 0x045U)
#define SAMD_PORTx_PINCFG6(port)  (SAMD_PORTx_BASE + port + 0x046U)
#define SAMD_PORTx_PINCFG7(port)  (SAMD_PORTx_BASE + port + 0x047U)
#define SAMD_PORTx_PINCFG8(port)  (SAMD_PORTx_BASE + port + 0x048U)
#define SAMD_PORTx_PINCFG9(port)  (SAMD_PORTx_BASE + port + 0x049U)
#define SAMD_PORTx_PINCFG10(port) (SAMD_PORTx_BASE + port + 0x04aU)
#define SAMD_PORTx_PINCFG11(port) (SAMD_PORTx_BASE + port + 0x04bU)
#define SAMD_PORTx_PINCFG12(port) (SAMD_PORTx_BASE + port + 0x04cU)
#define SAMD_PORTx_PINCFG13(port) (SAMD_PORTx_BASE + port + 0x04dU)
#define SAMD_PORTx_PINCFG14(port) (SAMD_PORTx_BASE + port + 0x04eU)
#define SAMD_PORTx_PINCFG15(port) (SAMD_PORTx_BASE + port + 0x04fU)
#define SAMD_PORTx_PINCFG16(port) (SAMD_PORTx_BASE + port + 0x050U)
#define SAMD_PORTx_PINCFG17(port) (SAMD_PORTx_BASE + port + 0x051U)
#define SAMD_PORTx_PINCFG18(port) (SAMD_PORTx_BASE + port + 0x052U)
#define SAMD_PORTx_PINCFG19(port) (SAMD_PORTx_BASE + port + 0x053U)
#define SAMD_PORTx_PINCFG20(port) (SAMD_PORTx_BASE + port + 0x054U)
#define SAMD_PORTx_PINCFG21(port) (SAMD_PORTx_BASE + port + 0x055U)
#define SAMD_PORTx_PINCFG22(port) (SAMD_PORTx_BASE + port + 0x056U)
#define SAMD_PORTx_PINCFG23(port) (SAMD_PORTx_BASE + port + 0x057U)
#define SAMD_PORTx_PINCFG24(port) (SAMD_PORTx_BASE + port + 0x058U)
#define SAMD_PORTx_PINCFG25(port) (SAMD_PORTx_BASE + port + 0x059U)
#define SAMD_PORTx_PINCFG26(port) (SAMD_PORTx_BASE + port + 0x05aU)
#define SAMD_PORTx_PINCFG27(port) (SAMD_PORTx_BASE + port + 0x05bU)
#define SAMD_PORTx_PINCFG28(port) (SAMD_PORTx_BASE + port + 0x05cU)
#define SAMD_PORTx_PINCFG29(port) (SAMD_PORTx_BASE + port + 0x05dU)
#define SAMD_PORTx_PINCFG30(port) (SAMD_PORTx_BASE + port + 0x05eU)
#define SAMD_PORTx_PINCFG31(port) (SAMD_PORTx_BASE + port + 0x05fU)

#define SAMD_PORTx_WRCONFIG_PINMASK_MASK (0xffffU << 0U)
#define SAMD_PORTx_WRCONFIG_PMUXEN       (1U << 16U)
#define SAMD_PORTx_WRCONFIG_INEN         (1U << 17U)
#define SAMD_PORTx_WRCONFIG_PULLEN       (1U << 18U)
#define SAMD_PORTx_WRCONFIG_DRVSTR       (1U << 22U)
#define SAMD_PORTx_WRCONFIG_PMUX_MASK    (0xfU << 24U)
#define SAMD_PORTx_WRCONFIG_WRPMUX       (1U << 28U)
#define SAMD_PORTx_WRCONFIG_WRPINCFG     (1U << 30U)
#define SAMD_PORTx_WRCONFIG_HWSEL        (1U << 31U) // 0 Lower 16 Pins, 1 Upper 16 Pins

#define SAMD_PORTx_PMUX_PMUXE_MASK   (0xfU << 0U)
#define SAMD_PORTx_PMUX_PMUXE_FUNC_A (0x0U << 0U)
#define SAMD_PORTx_PMUX_PMUXE_FUNC_B (0x1U << 0U)
#define SAMD_PORTx_PMUX_PMUXE_FUNC_C (0x2U << 0U)
#define SAMD_PORTx_PMUX_PMUXE_FUNC_D (0x3U << 0U)
#define SAMD_PORTx_PMUX_PMUXE_FUNC_E (0x4U << 0U)
#define SAMD_PORTx_PMUX_PMUXE_FUNC_F (0x5U << 0U)
#define SAMD_PORTx_PMUX_PMUXE_FUNC_G (0x6U << 0U)
#define SAMD_PORTx_PMUX_PMUXE_FUNC_H (0x7U << 0U)
#define SAMD_PORTx_PMUX_PMUXO_MASK   (0xfU << 4U)
#define SAMD_PORTx_PMUX_PMUXO_FUNC_A (0x0U << 4U)
#define SAMD_PORTx_PMUX_PMUXO_FUNC_B (0x1U << 4U)
#define SAMD_PORTx_PMUX_PMUXO_FUNC_C (0x2U << 4U)
#define SAMD_PORTx_PMUX_PMUXO_FUNC_D (0x3U << 4U)
#define SAMD_PORTx_PMUX_PMUXO_FUNC_E (0x4U << 4U)
#define SAMD_PORTx_PMUX_PMUXO_FUNC_F (0x5U << 4U)
#define SAMD_PORTx_PMUX_PMUXO_FUNC_G (0x6U << 4U)
#define SAMD_PORTx_PMUX_PMUXO_FUNC_H (0x7U << 4U)

#define SAMD_PORTx_PINCFG_PMUXEN (1U << 0U)
#define SAMD_PORTx_PINCFG_INEN   (1U << 1U)
#define SAMD_PORTx_PINCFG_PULLEN (1U << 2U)
#define SAMD_PORTx_PINCFG_DRVSTR (1U << 6U)

// SAMD Pins

#define SAMD_PIN(num) (1U << num)
#define SAMD_PIN0     SAMD_PIN(0U)
#define SAMD_PIN1     SAMD_PIN(1U)
#define SAMD_PIN2     SAMD_PIN(2U)
#define SAMD_PIN3     SAMD_PIN(3U)
#define SAMD_PIN4     SAMD_PIN(4U)
#define SAMD_PIN5     SAMD_PIN(5U)
#define SAMD_PIN6     SAMD_PIN(6U)
#define SAMD_PIN7     SAMD_PIN(7U)
#define SAMD_PIN8     SAMD_PIN(8U)
#define SAMD_PIN9     SAMD_PIN(9U)
#define SAMD_PIN10    SAMD_PIN(10U)
#define SAMD_PIN11    SAMD_PIN(11U)
#define SAMD_PIN12    SAMD_PIN(12U)
#define SAMD_PIN13    SAMD_PIN(13U)
#define SAMD_PIN14    SAMD_PIN(14U)
#define SAMD_PIN15    SAMD_PIN(15U)
#define SAMD_PIN16    SAMD_PIN(16U)
#define SAMD_PIN17    SAMD_PIN(17U)
#define SAMD_PIN18    SAMD_PIN(18U)
#define SAMD_PIN19    SAMD_PIN(19U)
#define SAMD_PIN20    SAMD_PIN(20U)
#define SAMD_PIN21    SAMD_PIN(21U)
#define SAMD_PIN22    SAMD_PIN(22U)
#define SAMD_PIN23    SAMD_PIN(23U)
#define SAMD_PIN24    SAMD_PIN(24U)
#define SAMD_PIN25    SAMD_PIN(25U)
#define SAMD_PIN26    SAMD_PIN(26U)
#define SAMD_PIN27    SAMD_PIN(27U)
#define SAMD_PIN28    SAMD_PIN(28U)
#define SAMD_PIN29    SAMD_PIN(29U)
#define SAMD_PIN30    SAMD_PIN(30U)
#define SAMD_PIN31    SAMD_PIN(31U)

static bool samd_flash_erase(target_flash_s *f, target_addr_t addr, size_t len);
static bool samd_flash_write(target_flash_s *f, target_addr_t dest, const void *src, size_t len);

static void samd_spi_init(target_s *const target, const target_addr32_t sercom_base);
static void samd_spi_read(target_s *const target, const uint16_t command, const target_addr_t address,
	void *const buffer, const size_t length);
static void samd_spi_write(
	target_s *target, uint16_t command, target_addr_t address, const void *buffer, size_t length);
static void samd_spi_run_cmd(target_s *target, uint16_t command, target_addr_t address);
/* NB: This is not marked static on purpose as it's used by samx5x.c. */
bool samd_mass_erase(target_s *t, platform_timeout_s *print_progess);

static void samd_mem_read(target_s *const target, void *const dest, const target_addr64_t src, const size_t len);

static bool samd_cmd_lock_flash(target_s *t, int argc, const char **argv);
static bool samd_cmd_unlock_flash(target_s *t, int argc, const char **argv);
static bool samd_cmd_unlock_bootprot(target_s *t, int argc, const char **argv);
static bool samd_cmd_lock_bootprot(target_s *t, int argc, const char **argv);
static bool samd_cmd_read_userrow(target_s *t, int argc, const char **argv);
static bool samd_cmd_serial(target_s *t, int argc, const char **argv);
static bool samd_cmd_mbist(target_s *t, int argc, const char **argv);
static bool samd_cmd_ssb(target_s *t, int argc, const char **argv);

const command_s samd_cmd_list[] = {
	{"lock_flash", samd_cmd_lock_flash, "Locks flash against spurious commands"},
	{"unlock_flash", samd_cmd_unlock_flash, "Unlocks flash"},
	{"lock_bootprot", samd_cmd_lock_bootprot, "Lock the boot protections to maximum"},
	{"unlock_bootprot", samd_cmd_unlock_bootprot, "Unlock the boot protections to minimum"},
	{"user_row", samd_cmd_read_userrow, "Prints user row from flash"},
	{"serial", samd_cmd_serial, "Prints serial number"},
	{"mbist", samd_cmd_mbist, "Runs the built-in memory test"},
	{"set_security_bit", samd_cmd_ssb, "Sets the Security Bit"},
	{NULL, NULL, NULL},
};

/* Non-Volatile Memory Controller (NVMC) Parameters */
#define SAMD_ROW_SIZE  256U
#define SAMD_PAGE_SIZE 64U

/* -------------------------------------------------------------------------- */
/* Non-Volatile Memory Controller (NVMC) Registers */
/* -------------------------------------------------------------------------- */

#define SAMD_NVMC         0x41004000U
#define SAMD_NVMC_CTRLA   (SAMD_NVMC + 0x00U)
#define SAMD_NVMC_CTRLB   (SAMD_NVMC + 0x04U)
#define SAMD_NVMC_PARAM   (SAMD_NVMC + 0x08U)
#define SAMD_NVMC_INTFLAG (SAMD_NVMC + 0x14U)
#define SAMD_NVMC_STATUS  (SAMD_NVMC + 0x18U)
#define SAMD_NVMC_ADDRESS (SAMD_NVMC + 0x1cU)

/* Control A Register (CTRLA) */
#define SAMD_CTRLA_CMD_KEY             0xa500U
#define SAMD_CTRLA_CMD_ERASEROW        0x0002U
#define SAMD_CTRLA_CMD_WRITEPAGE       0x0004U
#define SAMD_CTRLA_CMD_ERASEAUXROW     0x0005U
#define SAMD_CTRLA_CMD_WRITEAUXPAGE    0x0006U
#define SAMD_CTRLA_CMD_LOCK            0x0040U
#define SAMD_CTRLA_CMD_UNLOCK          0x0041U
#define SAMD_CTRLA_CMD_PAGEBUFFERCLEAR 0x0044U
#define SAMD_CTRLA_CMD_SSB             0x0045U
#define SAMD_CTRLA_CMD_INVALL          0x0046U

/* Interrupt Flag Register (INTFLAG) */
#define SAMD_NVMC_READY (1U << 0U)

/* Non-Volatile Memory Calibration and Auxiliary Registers */
#define SAMD_NVM_USER_ROW_LOW  0x00804000U
#define SAMD_NVM_USER_ROW_HIGH 0x00804004U
#define SAMD_NVM_CALIBRATION   0x00806020U
#define SAMD_NVM_SERIAL(n)     (0x0080a00cU + (0x30U * (((n) + 3U) / 4U)) + ((n) * 4U))

/* -------------------------------------------------------------------------- */
/* Device Service Unit (DSU) Registers */
/* -------------------------------------------------------------------------- */

#define SAMD_DSU            0x41002000U
#define SAMD_DSU_EXT_ACCESS (SAMD_DSU + 0x100U)
#define SAMD_DSU_CTRLSTAT   (SAMD_DSU_EXT_ACCESS + 0x000U)
#define SAMD_DSU_ADDRESS    (SAMD_DSU_EXT_ACCESS + 0x004U)
#define SAMD_DSU_LENGTH     (SAMD_DSU_EXT_ACCESS + 0x008U)
#define SAMD_DSU_DID        (SAMD_DSU_EXT_ACCESS + 0x018U)

/* Control and Status Register (CTRLSTAT) */
#define SAMD_CTRL_CHIP_ERASE (1U << 4U)
#define SAMD_CTRL_MBIST      (1U << 3U)
#define SAMD_CTRL_CRC        (1U << 2U)
#define SAMD_STATUSA_PERR    (1U << 12U)
#define SAMD_STATUSA_FAIL    (1U << 11U)
#define SAMD_STATUSA_BERR    (1U << 10U)
#define SAMD_STATUSA_CRSTEXT (1U << 9U)
#define SAMD_STATUSA_DONE    (1U << 8U)
#define SAMD_STATUSB_PROT    (1U << 16U)

/* Device Identification Register (DID) */
#define SAMD_DID_MASK          0xfe380000U
#define SAMD_DID_CONST_VALUE   0x10000000U
#define SAMD_DID_DEVSEL_MASK   0xffU
#define SAMD_DID_DEVSEL_POS    0U
#define SAMD_DID_REVISION_MASK 0x0fU
#define SAMD_DID_REVISION_POS  8U
#define SAMD_DID_SERIES_MASK   0x3fU
#define SAMD_DID_SERIES_POS    16U
#define SAMD_DID_FAMILY_MASK   0x1fU
#define SAMD_DID_FAMILY_POS    23U

#define ID_SAMD 0xcd0U

/* Family parts */
typedef struct samd_part {
	uint8_t devsel;
	char pin;
	uint8_t mem;
	uint8_t variant;
} samd_part_s;

static const samd_part_s samd_d21_parts[] = {
	{0x00, 'J', 18, 'A'}, /* SAMD21J18A */
	{0x01, 'J', 17, 'A'}, /* SAMD21J17A */
	{0x02, 'J', 16, 'A'}, /* SAMD21J16A */
	{0x03, 'J', 15, 'A'}, /* SAMD21J15A */
	{0x05, 'G', 18, 'A'}, /* SAMD21G18A */
	{0x06, 'G', 17, 'A'}, /* SAMD21G17A */
	{0x07, 'G', 16, 'A'}, /* SAMD21G16A */
	{0x08, 'G', 15, 'A'}, /* SAMD21G15A */
	{0x0a, 'E', 18, 'A'}, /* SAMD21E18A */
	{0x0b, 'E', 17, 'A'}, /* SAMD21E17A */
	{0x0c, 'E', 16, 'A'}, /* SAMD21E16A */
	{0x0d, 'E', 15, 'A'}, /* SAMD21E15A */
	{0x0f, 'G', 18, 'A'}, /* SAMD21G18A (WLCSP) */
	{0x10, 'G', 17, 'A'}, /* SAMD21G17A (WLCSP) */
	{0x20, 'J', 16, 'B'}, /* SAMD21J16B */
	{0x21, 'J', 15, 'B'}, /* SAMD21J15B */
	{0x23, 'G', 16, 'B'}, /* SAMD21G16B */
	{0x24, 'G', 15, 'B'}, /* SAMD21G15B */
	{0x26, 'E', 16, 'B'}, /* SAMD21E16B */
	{0x27, 'E', 15, 'B'}, /* SAMD21E15B */
	{0x55, 'E', 16, 'B'}, /* SAMD21E16B (WLCSP) */
	{0x56, 'E', 15, 'B'}, /* SAMD21E15B (WLCSP) */
	{0x62, 'E', 16, 'C'}, /* SAMD21E16C (WLCSP) */
	{0x63, 'E', 15, 'C'}, /* SAMD21E15C (WLCSP) */
	{0xff, 0, 0, 0},      /* Sentinel entry */
};

static const samd_part_s samd_c21_parts[] = {
	{0x00, 'J', 18, 'A'}, /* SAMC21J18A */
	{0x01, 'J', 17, 'A'}, /* SAMC21J17A */
	{0x02, 'J', 16, 'A'}, /* SAMC21J16A */
	{0x03, 'J', 15, 'A'}, /* SAMC21J15A */
	{0x05, 'G', 18, 'A'}, /* SAMC21G18A */
	{0x06, 'G', 17, 'A'}, /* SAMC21G17A */
	{0x07, 'G', 16, 'A'}, /* SAMC21G16A */
	{0x08, 'G', 15, 'A'}, /* SAMC21G15A */
	{0x0a, 'E', 18, 'A'}, /* SAMC21E18A */
	{0x0b, 'E', 17, 'A'}, /* SAMC21E17A */
	{0x0c, 'E', 16, 'A'}, /* SAMC21E16A */
	{0x0d, 'E', 15, 'A'}, /* SAMC21E15A */
	{0x20, 'N', 18, 'A'}, /* SAMC21N18A */
	{0x21, 'N', 17, 'A'}, /* SAMC21N17A */
	{0xff, 0, 0, 0},      /* Sentinel entry */
};

static const samd_part_s samd_l21_parts[] = {
	{0x00, 'J', 18, 'A'}, /* SAML21J18A */
	{0x01, 'J', 17, 'A'}, /* SAML21J17A */
	{0x02, 'J', 16, 'A'}, /* SAML21J16A */
	{0x05, 'G', 18, 'A'}, /* SAML21G18A */
	{0x06, 'G', 17, 'A'}, /* SAML21G17A */
	{0x07, 'G', 16, 'A'}, /* SAML21G16A */
	{0x0a, 'E', 18, 'A'}, /* SAML21E18A */
	{0x0b, 'E', 17, 'A'}, /* SAML21E17A */
	{0x0c, 'E', 16, 'A'}, /* SAML21E16A */
	{0x0d, 'E', 15, 'A'}, /* SAML21E15A */
	{0x0f, 'J', 18, 'B'}, /* SAML21J18B */
	{0x10, 'J', 17, 'B'}, /* SAML21J17B */
	{0x11, 'J', 16, 'B'}, /* SAML21J16B */
	{0x14, 'G', 18, 'B'}, /* SAML21G18B */
	{0x15, 'G', 17, 'B'}, /* SAML21G17B */
	{0x16, 'G', 16, 'B'}, /* SAML21G16B */
	{0x19, 'E', 18, 'B'}, /* SAML21E18B */
	{0x1a, 'E', 17, 'B'}, /* SAML21E17B */
	{0x1b, 'E', 16, 'B'}, /* SAML21E16B */
	{0x1c, 'E', 15, 'B'}, /* SAML21E15B */
	{0xff, 0, 0, 0},      /* Sentinel entry */
};

static const samd_part_s samd_l22_parts[] = {
	{0x00, 'N', 18, 'A'}, /* SAML22N18 */
	{0x01, 'N', 17, 'A'}, /* SAML22N17 */
	{0x02, 'N', 16, 'A'}, /* SAML22N16 */
	{0x05, 'J', 18, 'A'}, /* SAML22J18 */
	{0x06, 'J', 17, 'A'}, /* SAML22J17 */
	{0x07, 'J', 16, 'A'}, /* SAML22J16 */
	{0x0a, 'G', 18, 'A'}, /* SAML22G18 */
	{0x0b, 'G', 17, 'A'}, /* SAML22G17 */
	{0x0c, 'G', 16, 'A'}, /* SAML22G16 */
	{0xff, 0, 0, 0},      /* Sentinel entry */
};

/*
 * Overloads the default cortexm reset function with a version that
 * removes the target from extended reset where required.
 */
void samd_reset(target_s *t)
{
	/*
	 * nRST is not asserted here as it appears to reset the adiv5
	 * logic, meaning that subsequent adiv5_* calls PLATFORM_FATAL_ERROR.
	 *
	 * This is ok as normally you can just connect the debugger and go,
	 * but if that's not possible (protection or SWCLK being used for
	 * something else) then having SWCLK low on reset should get you
	 * debug access (cold-plugging). TODO: Confirm this
	 *
	 * See the SAM D20 datasheet §12.6 Debug Operation for more details.
	 *
	 * jtagtap_nrst(true);
	 * jtagtap_nrst(false);
	 *
	 * XXX: Should this actually call cortexm_reset()?
	 */

	/* Read DHCSR here to clear S_RESET_ST bit before reset */
	target_mem32_read32(t, CORTEXM_DHCSR);

	/*
	 * Request System Reset from NVIC: nRST doesn't work correctly
	 * This could be VECTRESET: 0x05fa0001 (reset only core)
	 *          or SYSRESETREQ: 0x05fa0004 (system reset)
	 */
	target_mem32_write32(t, CORTEXM_AIRCR, CORTEXM_AIRCR_VECTKEY | CORTEXM_AIRCR_SYSRESETREQ);

	/* Exit extended reset */
	if (target_mem32_read32(t, SAMD_DSU_CTRLSTAT) & SAMD_STATUSA_CRSTEXT)
		/* Write bit to clear from extended reset */
		target_mem32_write32(t, SAMD_DSU_CTRLSTAT, SAMD_STATUSA_CRSTEXT);

	/* Poll for release from reset */
	while (target_mem32_read32(t, CORTEXM_DHCSR) & CORTEXM_DHCSR_S_RESET_ST)
		continue;

	/* Reset DFSR flags and clear any target errors */
	target_mem32_write32(t, CORTEXM_DFSR, CORTEXM_DFSR_RESETALL);
}

/*
 * Overloads the default cortexm detached function with a version that
 * removes the target from extended reset where required.
 *
 * Only required for SAM D20 _Revision B_ Silicon
 */
static void samd20_revB_detach(target_s *t)
{
	cortexm_detach(t);

	/* Exit extended reset */
	if (target_mem32_read32(t, SAMD_DSU_CTRLSTAT) & SAMD_STATUSA_CRSTEXT)
		/* Write bit to clear from extended reset */
		target_mem32_write32(t, SAMD_DSU_CTRLSTAT, SAMD_STATUSA_CRSTEXT);
}

/*
 * Overloads the default cortexm halt_resume function with a version
 * that removes the target from extended reset where required.
 *
 * Only required for SAM D20 _Revision B_ Silicon
 */
static void samd20_revB_halt_resume(target_s *t, bool step)
{
	cortexm_halt_resume(t, step);

	/* Exit extended reset */
	if (target_mem32_read32(t, SAMD_DSU_CTRLSTAT) & SAMD_STATUSA_CRSTEXT)
		/* Write bit to clear from extended reset */
		target_mem32_write32(t, SAMD_DSU_CTRLSTAT, SAMD_STATUSA_CRSTEXT);
}

/*
 * Release the target from extended reset before running the normal cortexm_attach routine.
 * This prevents tripping up over errata ref 9905
 *
 * Only required for SAM D11 silicon.
 */
static bool samd11_attach(target_s *t)
{
	/* Exit extended reset */
	if (target_mem32_read32(t, SAMD_DSU_CTRLSTAT) & SAMD_STATUSA_CRSTEXT)
		/* Write bit to clear from extended reset */
		target_mem32_write32(t, SAMD_DSU_CTRLSTAT, SAMD_STATUSA_CRSTEXT);

	return cortexm_attach(t);
}

/*
 * Overload the default cortexm attach for when the samd is protected.
 *
 * If the samd is protected then the default cortexm attach will
 * fail as the S_HALT bit in the DHCSR will never go high. This
 * function allows users to attach on a temporary basis so they can
 * rescue the device.
 */
bool samd_protected_attach(target_s *t)
{
	tc_printf(t, "Attached in protected mode, please issue 'monitor erase_mass' to regain chip access\n");
	/* Patch back in the normal cortexm attach for next time */
	t->attach = cortexm_attach;

	/* Allow attach this time */
	return true;
}

/*
 * Use the DSU Device Identification Register to populate a struct
 * describing the SAM D device.
 */
typedef struct samd_descr {
	char family;
	uint8_t series;
	char revision;
	char pin;
	uint32_t ram_size;
	uint32_t flash_size;
	uint8_t mem;
	char variant;
	char package[3];
} samd_descr_s;

samd_descr_s samd_parse_device_id(uint32_t did)
{
	samd_descr_s samd = {0};
	const samd_part_s *parts = samd_d21_parts;
	samd.ram_size = 0x8000;
	samd.flash_size = 0x40000;

	/* Family */
	const uint8_t family = (did >> SAMD_DID_FAMILY_POS) & SAMD_DID_FAMILY_MASK;
	switch (family) {
	case 0:
		samd.family = 'D';
		break;
	case 1:
		samd.family = 'L';
		parts = samd_l21_parts;
		break;
	case 2:
		samd.family = 'C';
		break;
	}
	/* Series */
	const uint8_t series = (did >> SAMD_DID_SERIES_POS) & SAMD_DID_SERIES_MASK;
	switch (series) {
	case 0:
		samd.series = 20;
		break;
	case 1:
		if (family == 2)
			parts = samd_c21_parts;
		samd.series = 21;
		break;
	case 2:
		if (family == 1) {
			samd.series = 22;
			parts = samd_l22_parts;
		} else
			samd.series = 10;
		break;
	case 3:
		samd.series = 11;
		break;
	case 4:
		samd.series = 9;
		break;
	case 7:
		/* PIC32CM MC00 */
		samd.series = 7;
		break;
	default:
		samd.series = 0;
		break;
	}
	/* Revision */
	const uint8_t revision = (did >> SAMD_DID_REVISION_POS) & SAMD_DID_REVISION_MASK;
	samd.revision = (char)('A' + revision);

	const uint8_t devsel = (did >> SAMD_DID_DEVSEL_POS) & SAMD_DID_DEVSEL_MASK;
	switch (samd.series) {
	case 20U: /* SAM D20 */
		switch (devsel / 5U) {
		case 0U:
			samd.pin = 'J';
			break;
		case 1U:
			samd.pin = 'G';
			break;
		case 2U:
			samd.pin = 'E';
			break;
		default:
			samd.pin = 'u';
			break;
		}
		samd.mem = 18U - (devsel % 5U);
		samd.variant = 'A';
		break;
	case 21U: /* SAM D21/L21 */
	case 22U: /* SAM L22 */
		for (size_t i = 0; parts[i].devsel != 0xffU; ++i) {
			if (parts[i].devsel == devsel) {
				samd.pin = parts[i].pin;
				samd.mem = parts[i].mem;
				samd.variant = parts[i].variant;
				break;
			}
		}
		break;
	case 10U: /* SAM D10 */
	case 11U: /* SAM D11 */
		switch (devsel / 3U) {
		case 0U:
			samd.package[0] = 'M';
			break;
		case 1U:
			samd.package[0] = 'S';
			samd.package[1] = 'S';
			break;
		}
		samd.pin = 'D';
		samd.mem = 14U - (devsel % 3U);
		samd.variant = 'A';
		break;
	case 9U: /* SAM D09 */
		samd.ram_size = 4096;
		switch (devsel) {
		case 0U:
			samd.pin = 'D';
			samd.mem = 14;
			samd.flash_size = 16384;
			samd.package[0] = 'M';
			break;
		case 7U:
			samd.pin = 'C';
			samd.mem = 13;
			samd.flash_size = 8192;
			break;
		}
		samd.variant = 'A';
		break;
	case 7U: /* PIC32CM MC00 */
		if (devsel & 0x1) {
			/* PIC32CM6408MC000xx */
			samd.flash_size = 65536;
			samd.ram_size = 8192;
			samd.mem = 8;
		} else {
			/* PIC32CM1216MC000xx */
			samd.flash_size = 131072;
			samd.ram_size = 16384;
			samd.mem = 16;
		}
		/* PIC32CMxxxxMC000(32|48) */
		samd.pin = (devsel & 0x6) ? 48 : 32;
	}

	return samd;
}

static void samd_add_flash(target_s *t, uint32_t addr, size_t length)
{
	target_flash_s *f = calloc(1, sizeof(*f));
	if (!f) { /* calloc failed: heap exhaustion */
		DEBUG_ERROR("calloc: failed in %s\n", __func__);
		return;
	}

	f->start = addr;
	f->length = length;
	f->blocksize = SAMD_ROW_SIZE;
	f->erase = samd_flash_erase;
	f->write = samd_flash_write;
	f->writesize = SAMD_PAGE_SIZE;
	target_add_flash(t, f);
}

#define SAMD_VARIANT_STR_LENGTH 60U

typedef struct samd_priv {
	char samd_variant_string[SAMD_VARIANT_STR_LENGTH];
} samd_priv_s;

bool samd_probe(target_s *t)
{
	/* Check the part number is the SAMD part number */
	if (t->part_id != ID_SAMD)
		return false;

	/* Read the Device ID */
	const uint32_t did = target_mem32_read32(t, SAMD_DSU_DID);

	/* If the Device ID matches */
	if ((did & SAMD_DID_MASK) != SAMD_DID_CONST_VALUE)
		return false;

	samd_priv_s *priv_storage = calloc(1, sizeof(*priv_storage));
	t->target_storage = priv_storage;

	const uint32_t ctrlstat = target_mem32_read32(t, SAMD_DSU_CTRLSTAT);
	const samd_descr_s samd = samd_parse_device_id(did);

	/* Protected? */
	const bool protected = (ctrlstat & SAMD_STATUSB_PROT);

	if (samd.series == 7) {
		snprintf(priv_storage->samd_variant_string, SAMD_VARIANT_STR_LENGTH,
			"Microchip PIC32CM%02u%02uMC000%02u (rev %c)%s", samd.mem > 8U ? 12U : 64U, samd.mem, samd.pin,
			samd.revision, protected ? " protected" : "");
	} else {
		snprintf(priv_storage->samd_variant_string, SAMD_VARIANT_STR_LENGTH, "Atmel SAM%c%02d%c%d%c%s (rev %c)%s",
			samd.family, samd.series, samd.pin, samd.mem, samd.variant, samd.package, samd.revision,
			protected ? " protected" : "");
	}

	/* Setup Target */
	t->driver = priv_storage->samd_variant_string;
	t->reset = samd_reset;
	t->mass_erase = samd_mass_erase;
	t->mem_read = samd_mem_read;

	if (samd.series == 20 && samd.revision == 'B') {
		/*
		 * These functions check for an extended reset.
		 * Appears to be related to Errata 35.4.1 ref 12015
		 */
		t->detach = samd20_revB_detach;
		t->halt_resume = samd20_revB_halt_resume;
	} else if (samd.series == 11) {
		/*
		 * Attach routine that checks for an extended reset and releases it.
		 * This works around Errata 38.2.5 ref 9905
		 */
		t->attach = samd11_attach;
	}

	if (protected) {
		/*
		 * Overload the default cortexm attach
		 * for when the samd is protected.
		 * This function allows users to
		 * attach on a temporary basis so they
		 * can rescue the device.
		 */
		t->attach = samd_protected_attach;
	}

	samd_spi_init(t, SAMD_SERCOM0_BASE);

	target_add_ram32(t, SAMD_SRAM_BASE, samd.ram_size);
	samd_add_flash(t, SAMD_FLASH_BANK_BASE, samd.flash_size);
	bmp_spi_add_flash(
		t, SAMD_SQUISHY_FLASH_BASE, SAMD_SQUISHY_FLASH_SIZE, samd_spi_read, samd_spi_write, samd_spi_run_cmd);

	target_add_commands(t, samd_cmd_list, "SAMD");

	/* If we're not in reset here */
	if (!platform_nrst_get_val()) {
		/* We'll have to release the target from
		 * extended reset to make attach possible */
		if (target_mem32_read32(t, SAMD_DSU_CTRLSTAT) & SAMD_STATUSA_CRSTEXT)
			/* Write bit to clear from extended reset */
			target_mem32_write32(t, SAMD_DSU_CTRLSTAT, SAMD_STATUSA_CRSTEXT);
	}

	return true;
}

/* Temporary (until next reset) flash memory locking */
static void samd_lock_current_address(target_s *t)
{
	/* Issue the lock command */
	target_mem32_write32(t, SAMD_NVMC_CTRLA, SAMD_CTRLA_CMD_KEY | SAMD_CTRLA_CMD_LOCK);
}

/* Temporary (until next reset) flash memory unlocking */
static void samd_unlock_current_address(target_s *t)
{
	/* Issue the unlock command */
	target_mem32_write32(t, SAMD_NVMC_CTRLA, SAMD_CTRLA_CMD_KEY | SAMD_CTRLA_CMD_UNLOCK);
}

static bool samd_wait_nvm_ready(target_s *t)
{
	/* Poll for NVM Ready */
	while ((target_mem32_read32(t, SAMD_NVMC_INTFLAG) & SAMD_NVMC_READY) == 0) {
		if (target_check_error(t))
			return false;
	}
	return true;
}

static bool samd_wait_dsu_ready(target_s *const t, uint32_t *const result, platform_timeout_s *const print_progress)
{
	uint32_t status = 0;
	while ((status & (SAMD_STATUSA_DONE | SAMD_STATUSA_PERR | SAMD_STATUSA_FAIL)) == 0) {
		status = target_mem32_read32(t, SAMD_DSU_CTRLSTAT);
		if (target_check_error(t))
			return false;
		if (print_progress)
			target_print_progress(print_progress);
	}
	*result = status;
	return true;
}

/* Erase flash row by row */
static bool samd_flash_erase(target_flash_s *const f, const target_addr_t addr, const size_t len)
{
	target_s *t = f->t;
	for (size_t offset = 0; offset < len; offset += f->blocksize) {
		/*
		 * Write address of first word in row to erase it
		 * Must be shifted right for 16-bit address, see Datasheet §20.8.8 Address
		 */
		target_mem32_write32(t, SAMD_NVMC_ADDRESS, (addr + offset) >> 1U);

		/* Unlock */
		samd_unlock_current_address(t);

		/* Issue the erase command */
		target_mem32_write32(t, SAMD_NVMC_CTRLA, SAMD_CTRLA_CMD_KEY | SAMD_CTRLA_CMD_ERASEROW);
		if (!samd_wait_nvm_ready(t))
			return false;

		/* Lock */
		samd_lock_current_address(t);
	}

	return true;
}

/*
 * Write flash page by page
 */
static bool samd_flash_write(target_flash_s *f, target_addr_t dest, const void *src, size_t len)
{
	target_s *t = f->t;

	/* Write within a single page. This may be part or all of the page */
	target_mem32_write(t, dest, src, len);

	/* Unlock */
	samd_unlock_current_address(t);

	/* Issue the write page command */
	target_mem32_write32(t, SAMD_NVMC_CTRLA, SAMD_CTRLA_CMD_KEY | SAMD_CTRLA_CMD_WRITEPAGE);
	if (!samd_wait_nvm_ready(t))
		return false;

	/* Lock */
	samd_lock_current_address(t);

	return true;
}

static void samd_pin_setup(
	target_s *const target, const uint8_t port, const uint8_t pin, const uint8_t cfg, const uint8_t mux)
{
	const target_addr32_t port_pinmux_taddr = SAMD_PORTx_PMUX0(port) + (pin >> 1U);

	// Shift depending if we're dealing with an odd pin
	const size_t pinmux_shift = (pin & 1U) << 2U;
	// Get only the half of the pin mux config we're interested in preserving
	uint8_t pin_mux = target_mem32_read8(target, port_pinmux_taddr) & ~(SAMD_PORTx_PMUX_PMUXE_MASK << pinmux_shift);
	// Set the new mux configuration
	pin_mux |= (mux & SAMD_PORTx_PMUX_PMUXE_MASK) << pinmux_shift;
	target_mem32_write8(target, port_pinmux_taddr, pin_mux);

	// Set the pin configuration
	target_mem32_write8(target, SAMD_PORTx_PINCFG0(port) + pin, cfg);
}

static void samd_setup_sercom(target_s *const target, const target_addr32_t sercom_base)
{
	if (sercom_base == SAMD_SERCOM0_BASE) {
		samd_pin_setup(target, SAMD_PORT_A, 4U, SAMD_PORTx_PINCFG_DRVSTR, SAMD_PORTx_PMUX_PMUXE_FUNC_C);
		samd_pin_setup(
			target, SAMD_PORT_A, 5U, SAMD_PORTx_PINCFG_DRVSTR | SAMD_PORTx_PINCFG_PMUXEN, SAMD_PORTx_PMUX_PMUXE_FUNC_D);
		samd_pin_setup(
			target, SAMD_PORT_A, 6U, SAMD_PORTx_PINCFG_DRVSTR | SAMD_PORTx_PINCFG_PMUXEN, SAMD_PORTx_PMUX_PMUXE_FUNC_C);
		samd_pin_setup(target, SAMD_PORT_A, 7U,
			SAMD_PORTx_PINCFG_DRVSTR | SAMD_PORTx_PINCFG_PMUXEN | SAMD_PORTx_PINCFG_INEN, SAMD_PORTx_PMUX_PMUXE_FUNC_D);

		target_mem32_write32(target, SAMD_PORTx_OUTSET(SAMD_PORT_A), SAMD_PIN4);
		target_mem32_write32(target, SAMD_PORTx_DIRSET(SAMD_PORT_A), SAMD_PIN4 | SAMD_PIN5 | SAMD_PIN6);
		target_mem32_write32(target, SAMD_PORTx_DIRCLR(SAMD_PORT_A), SAMD_PIN7);
	} else {
	}
}

static void samd_spi_init(target_s *const target, const target_addr32_t sercom_base)
{
	// Check if the SERCOM is enabled, disable if so
	uint32_t ctrla = target_mem32_read32(target, SAMD_SERCOMx_CTRLA(sercom_base));
	if (ctrla & SAMD_SERCOMx_CTRLA_ENABLE) {
		target_mem32_write32(target, SAMD_SERCOMx_CTRLA(sercom_base), ctrla & ~SAMD_SERCOMx_CTRLA_ENABLE);
		while (target_mem32_read32(target, SAMD_SERCOMx_SYNCBUSY(sercom_base)) & SAMD_SERCOMx_SYNCBUSY_ENABLE)
			continue;
	}

	// Setup the SERCOMx pin configuration
	samd_setup_sercom(target, sercom_base);

	// Set us as an SPI controller
	if (sercom_base == SAMD_SERCOM0_BASE) {
		ctrla = SAMD_SERCOMx_CTRLA_MODE_CONTROLLER |
			// Set CPOL to 0 and CPHA to 1, setting SCK idle low, sample on trailing edge
			SAMD_SERCOMx_CTRLA_CPHA |
			// Pure data frame format (ignores the addr)
			SAMD_SERCOMx_CTRLA_FORM_SPI |
			// Set up the SERCOM Pinout: PAD[0] = COPI; PAD[1] = CLK; PAD[2] = CS; PAD[3] = CIPO
			SAMD_SERCOMx_CTRLA_DOPO_0 | SAMD_SERCOMx_CTRLA_DIPO_1 |
			// Set to LSB-first
			SAMD_SERCOMx_CTRLA_DORD;

		// Wiggle the bits
		target_mem32_write32(target, SAMD_SERCOMx_CTRLA(sercom_base), ctrla);

		// Set the character size to 8-bits, enable receve mode
		target_mem32_write32(
			target, SAMD_SERCOMx_CTRLB(sercom_base), SAMD_SERCOMx_CTRLB_CHSIZE_8BIT | SAMD_SERCOMx_CTRLB_RXEN);

		// Enable the BAUD generation even though we've brainslugged the core
		target_mem32_write8(target, SAMD_SERCOMx_DBGCTRL(sercom_base), 0U);

		// Assume 32MHz in 16MHz flash clock
		// baud = (32MHz / (2 * 16MHz)) - 1 = 0
		target_mem32_write8(target, SAMD_SERCOMx_BAUD(sercom_base), 0U);

		// Enable the SERCOM and wait for things to go green
		target_mem32_write32(target, SAMD_SERCOMx_CTRLA(sercom_base), ctrla | SAMD_SERCOMx_CTRLA_ENABLE);
		while (target_mem32_read32(target, SAMD_SERCOMx_SYNCBUSY(sercom_base)) & SAMD_SERCOMx_SYNCBUSY_ENABLE)
			continue;
	} else {
	}
}

static uint8_t samd_spi_xfer(target_s *const target, const target_addr32_t sercom_base, const uint8_t data)
{
	target_mem32_write8(target, SAMD_SERCOMx_DATA(sercom_base), data);
	return target_mem32_read8(target, SAMD_SERCOMx_DATA(sercom_base));
}

static void samd_spi_setup_xfer(target_s *const target, const uint16_t command, const target_addr32_t address)
{
	target_mem32_write32(target, SAMD_PORTx_OUTCLR(SAMD_PORT_A), SAMD_PIN4);

	/* Set up the instruction */
	const uint8_t opcode = command & SPI_FLASH_OPCODE_MASK;
	samd_spi_xfer(target, SAMD_SERCOM0_BASE, opcode);

	if ((command & SPI_FLASH_OPCODE_MODE_MASK) == SPI_FLASH_OPCODE_3B_ADDR) {
		/* For each byte sent here, we have to manually clean up from the controller with a read */
		samd_spi_xfer(target, SAMD_SERCOM0_BASE, (address >> 16U) & 0xffU);
		samd_spi_xfer(target, SAMD_SERCOM0_BASE, (address >> 8U) & 0xffU);
		samd_spi_xfer(target, SAMD_SERCOM0_BASE, address & 0xffU);
	}

	const size_t inter_length = (command & SPI_FLASH_DUMMY_MASK) >> SPI_FLASH_DUMMY_SHIFT;
	for (size_t i = 0; i < inter_length; ++i)
		/* For each byte sent here, we have to manually clean up from the controller with a read */
		samd_spi_xfer(target, SAMD_SERCOM0_BASE, 0);
}

static void samd_spi_read(target_s *const target, const uint16_t command, const target_addr_t address,
	void *const buffer, const size_t length)
{
	samd_spi_setup_xfer(target, command, address);

	uint8_t *const data = (uint8_t *const)buffer;
	for (size_t i = 0; i < length; ++i)
		data[i] = samd_spi_xfer(target, SAMD_SERCOM0_BASE, 0);

	target_mem32_write32(target, SAMD_PORTx_OUTSET(SAMD_PORT_A), SAMD_PIN4);
}

static void samd_spi_write(target_s *const target, const uint16_t command, const target_addr_t address,
	const void *const buffer, const size_t length)
{
	samd_spi_setup_xfer(target, command, address);

	const uint8_t *const data = (const uint8_t *const)buffer;
	for (size_t i = 0; i < length; ++i)
		samd_spi_xfer(target, SAMD_SERCOM0_BASE, data[i]);

	target_mem32_write32(target, SAMD_PORTx_OUTSET(SAMD_PORT_A), SAMD_PIN4);
}

static void samd_spi_run_cmd(target_s *const target, const uint16_t command, const target_addr_t address)
{
	samd_spi_setup_xfer(target, command, address);
	target_mem32_write32(target, SAMD_PORTx_OUTSET(SAMD_PORT_A), SAMD_PIN4);
}

static void samd_mem_read(target_s *const target, void *const dest, const target_addr64_t src, const size_t len)
{
	if (src >= SAMD_SQUISHY_FLASH_BASE && src < SAMD_SQUISHY_FLASH_BASE + SAMD_SQUISHY_FLASH_SIZE)
		samd_spi_read(target, SPI_FLASH_OPCODE_3B_ADDR | SPI_FLASH_DUMMY_LEN(0U) | SPI_FLASH_OPCODE(0x03U),
			src - SAMD_SQUISHY_FLASH_BASE, dest, len);
	else
		cortexm_mem_read(target, dest, src, len);
}

/* Uses the Device Service Unit to erase the entire flash */
bool samd_mass_erase(target_s *const t, platform_timeout_s *const print_progess)
{
	/* Clear the DSU status bits */
	target_mem32_write32(t, SAMD_DSU_CTRLSTAT, SAMD_STATUSA_DONE | SAMD_STATUSA_PERR | SAMD_STATUSA_FAIL);

	/* Erase all */
	target_mem32_write32(t, SAMD_DSU_CTRLSTAT, SAMD_CTRL_CHIP_ERASE);

	uint32_t status = 0;
	if (!samd_wait_dsu_ready(t, &status, print_progess))
		return false;

	/* Test the protection error bit in Status A */
	if (status & SAMD_STATUSA_PERR) {
		tc_printf(t, "Erase failed due to a protection error.\n");
		return true;
	}

	/* Test the fail bit in Status A */
	return !(status & SAMD_STATUSA_FAIL);
}

/*
 * Sets the NVM region lock bits in the User Row. This value is read
 * at startup as the default value for the lock bits, and hence does
 * not take effect until a reset.
 *
 * 0x0000 = Lock, 0xffff = Unlock (default)
 */
static bool samd_set_flashlock(target_s *t, uint16_t value, const char **argv)
{
	(void)argv;
	uint32_t high = target_mem32_read32(t, SAMD_NVM_USER_ROW_HIGH);
	uint32_t low = target_mem32_read32(t, SAMD_NVM_USER_ROW_LOW);

	/* Write address of a word in the row to erase it */
	/* Must be shifted right for 16-bit address, see Datasheet §20.8.8 Address */
	target_mem32_write32(t, SAMD_NVMC_ADDRESS, SAMD_NVM_USER_ROW_LOW >> 1U);

	/* Issue the erase command */
	target_mem32_write32(t, SAMD_NVMC_CTRLA, SAMD_CTRLA_CMD_KEY | SAMD_CTRLA_CMD_ERASEAUXROW);
	if (!samd_wait_nvm_ready(t))
		return false;

	/* Modify the high byte of the user row */
	high = (high & 0x0000ffffU) | ((value << 16U) & 0xffff0000U);

	/* Write back */
	target_mem32_write32(t, SAMD_NVM_USER_ROW_LOW, low);
	target_mem32_write32(t, SAMD_NVM_USER_ROW_HIGH, high);

	/* Issue the page write command */
	target_mem32_write32(t, SAMD_NVMC_CTRLA, SAMD_CTRLA_CMD_KEY | SAMD_CTRLA_CMD_WRITEAUXPAGE);

	return true;
}

static bool parse_unsigned(const char *str, uint32_t *val)
{
	char *end = NULL;
	unsigned long num;

	num = strtoul(str, &end, 0);
	if (end == NULL || end == str)
		return false;

	*val = (uint32_t)num;
	return true;
}

static bool samd_cmd_lock_flash(target_s *t, int argc, const char **argv)
{
	if (argc > 2) {
		tc_printf(t, "usage: monitor lock_flash [number]\n");
		return false;
	}
	if (argc == 2) {
		uint32_t val = 0;
		if (!parse_unsigned(argv[1], &val)) {
			tc_printf(t, "number must be either decimal or 0x prefixed hexadecimal\n");
			return false;
		}

		if (val > 0xffffU) {
			tc_printf(t, "number must be between 0 and 65535\n");
			return false;
		}

		return samd_set_flashlock(t, (uint16_t)val, NULL);
	}
	return samd_set_flashlock(t, 0x0000, NULL);
}

static bool samd_cmd_unlock_flash(target_s *t, int argc, const char **argv)
{
	(void)argc;
	(void)argv;
	return samd_set_flashlock(t, 0xffff, NULL);
}

static bool samd_set_bootprot(target_s *t, uint16_t value, const char **argv)
{
	(void)argv;
	const uint32_t high = target_mem32_read32(t, SAMD_NVM_USER_ROW_HIGH);
	uint32_t low = target_mem32_read32(t, SAMD_NVM_USER_ROW_LOW);

	/*
	 * Write address of a word in the row to erase it
	 * Must be shifted right for 16-bit address, see Datasheet §20.8.8 Address
	 */
	target_mem32_write32(t, SAMD_NVMC_ADDRESS, SAMD_NVM_USER_ROW_LOW >> 1U);

	/* Issue the erase command */
	target_mem32_write32(t, SAMD_NVMC_CTRLA, SAMD_CTRLA_CMD_KEY | SAMD_CTRLA_CMD_ERASEAUXROW);
	if (!samd_wait_nvm_ready(t))
		return false;

	/* Modify the low word of the user row */
	low = (low & 0xfffffff8U) | ((value << 0U) & 0x00000007U);

	/* Write back */
	target_mem32_write32(t, SAMD_NVM_USER_ROW_LOW, low);
	target_mem32_write32(t, SAMD_NVM_USER_ROW_HIGH, high);

	/* Issue the page write command */
	target_mem32_write32(t, SAMD_NVMC_CTRLA, SAMD_CTRLA_CMD_KEY | SAMD_CTRLA_CMD_WRITEAUXPAGE);
	return true;
}

static bool samd_cmd_lock_bootprot(target_s *t, int argc, const char **argv)
{
	/* Locks first 0x7 .. 0, 0x6 .. 512, 0x5 .. 1024, ..., 0x0 .. 32768 bytes of flash*/
	if (argc > 2) {
		tc_printf(t, "usage: monitor lock_bootprot [number]\n");
		return false;
	}
	if (argc == 2) {
		uint32_t val = 0;
		if (!parse_unsigned(argv[1], &val)) {
			tc_printf(t, "number must be either decimal or 0x prefixed hexadecimal\n");
			return false;
		}

		if (val > 7U) {
			tc_printf(t, "number must be between 0 and 7\n");
			return false;
		}

		return samd_set_bootprot(t, (uint16_t)val, NULL);
	}
	return samd_set_bootprot(t, 0, NULL);
}

static bool samd_cmd_unlock_bootprot(target_s *t, int argc, const char **argv)
{
	(void)argc;
	(void)argv;
	return samd_set_bootprot(t, 7, NULL);
}

static bool samd_cmd_read_userrow(target_s *t, int argc, const char **argv)
{
	(void)argc;
	(void)argv;
	tc_printf(t, "User Row: 0x%08" PRIx32 "%08" PRIx32 "\n", target_mem32_read32(t, SAMD_NVM_USER_ROW_HIGH),
		target_mem32_read32(t, SAMD_NVM_USER_ROW_LOW));

	return true;
}

/* Reads the 128-bit serial number from the NVM */
static bool samd_cmd_serial(target_s *t, int argc, const char **argv)
{
	(void)argc;
	(void)argv;
	tc_printf(t, "Serial Number: 0x");

	for (size_t i = 0; i < 4U; ++i)
		tc_printf(t, "%08" PRIx32 "", target_mem32_read32(t, SAMD_NVM_SERIAL(i)));
	tc_printf(t, "\n");
	return true;
}

/* Returns the size (in bytes) of the current SAM D20's flash memory. */
static uint32_t samd_flash_size(target_s *t)
{
	/* Read the Device ID */
	const uint32_t did = target_mem32_read32(t, SAMD_DSU_DID);
	/* Mask off the device select bits */
	const uint8_t devsel = did & SAMD_DID_DEVSEL_MASK;
	/* Shift the maximum flash size (256KB) down as appropriate */
	return (0x40000U >> (devsel % 5U));
}

/* Runs the Memory Built In Self Test (MBIST) */
static bool samd_cmd_mbist(target_s *t, int argc, const char **argv)
{
	(void)argc;
	(void)argv;
	/* Write the memory parameters to the DSU */
	target_mem32_write32(t, SAMD_DSU_ADDRESS, 0);
	target_mem32_write32(t, SAMD_DSU_LENGTH, samd_flash_size(t));

	/* Clear the fail bit */
	target_mem32_write32(t, SAMD_DSU_CTRLSTAT, SAMD_STATUSA_FAIL);

	/* Write the MBIST command */
	target_mem32_write32(t, SAMD_DSU_CTRLSTAT, SAMD_CTRL_MBIST);

	uint32_t status = 0;
	if (!samd_wait_dsu_ready(t, &status, NULL))
		return false;

	/* Test the protection error bit in Status A */
	if (status & SAMD_STATUSA_PERR) {
		tc_printf(t, "MBIST not run due to protection error.\n");
		return true;
	}

	/* Test the fail bit in Status A */
	if (status & SAMD_STATUSA_FAIL)
		tc_printf(t, "MBIST Fail @ 0x%08" PRIx32 "\n", target_mem32_read32(t, SAMD_DSU_ADDRESS));
	else
		tc_printf(t, "MBIST Passed!\n");
	return true;
}

/*
 * Sets the security bit
 */
static bool samd_cmd_ssb(target_s *t, int argc, const char **argv)
{
	(void)argc;
	(void)argv;
	/* Issue the ssb command */
	target_mem32_write32(t, SAMD_NVMC_CTRLA, SAMD_CTRLA_CMD_KEY | SAMD_CTRLA_CMD_SSB);
	if (!samd_wait_nvm_ready(t))
		return false;

	tc_printf(t, "Security bit set!\nScan again, attach and issue 'monitor erase_mass' to reset.\n");

	target_reset(t);
	return true;
}

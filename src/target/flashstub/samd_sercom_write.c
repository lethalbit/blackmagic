/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2024 Black Sphere Technologies Ltd.
 * Written by Rachel Mant <git@dragonmux.network>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <stddef.h>

#define SAMD_SERCOM0_BASE 0x42000800U

#define SAMD_SERCOMx_INTFLAG_DRE (1U << 0U)
#define SAMD_SERCOMx_INTFLAG_TXC (1U << 1U)
#define SAMD_SERCOMx_INTFLAG_RXC (1U << 2U)

#define SAMD_PORTA_BASE 0x41004400U

#define SAMD_PIN(num) (1U << (num))
#define SAMD_PIN4     SAMD_PIN(4U)

#define SPI_FLASH_OPCODE_MASK      0x00ffU
#define SPI_FLASH_OPCODE(x)        ((x) & SPI_FLASH_OPCODE_MASK)
#define SPI_FLASH_DUMMY_MASK       0x0700U
#define SPI_FLASH_DUMMY_SHIFT      8U
#define SPI_FLASH_DUMMY_LEN(x)     (((x) << SPI_FLASH_DUMMY_SHIFT) & SPI_FLASH_DUMMY_MASK)
#define SPI_FLASH_OPCODE_MODE_MASK 0x0800U
#define SPI_FLASH_OPCODE_ONLY      (0U << 11U)
#define SPI_FLASH_OPCODE_3B_ADDR   (1U << 11U)
#define SPI_FLASH_DATA_MASK        0x1000U
#define SPI_FLASH_DATA_SHIFT       12U
#define SPI_FLASH_DATA_IN          (0U << SPI_FLASH_DATA_SHIFT)
#define SPI_FLASH_DATA_OUT         (1U << SPI_FLASH_DATA_SHIFT)

/* SPI Flash opcodes used */
#define SPI_FLASH_CMD_PAGE_PROGRAM 0x02U
#define SPI_FLASH_CMD_READ_STATUS  0x05U
#define SPI_FLASH_CMD_WRITE_ENABLE 0x06U

/* SPI Flash status register bit definitions */
#define SPI_FLASH_STATUS_BUSY          0x01U
#define SPI_FLASH_STATUS_WRITE_ENABLED 0x02U

typedef struct sercom {
	volatile uint32_t ctrla;
	volatile uint32_t ctrlb;
	uint32_t reserved0;
	volatile uint8_t baud;
	uint8_t reserved1[7U];
	volatile uint8_t itr_enable_clear;
	uint8_t reserved2;
	volatile uint8_t itr_enable_set;
	uint8_t reserved3;
	volatile uint8_t itr_flags;
	uint8_t reserved4;
	volatile uint16_t status;
	volatile uint32_t sync_busy;
	uint32_t reserved5;
	volatile uint16_t addr;
	volatile uint16_t addr_mask;
	volatile uint16_t data;
	uint16_t reserved6[3];
	volatile uint8_t debug_ctrl;
} sercom_s;

static sercom_s *const sercom0 = (sercom_s *)SAMD_SERCOM0_BASE;

typedef struct port {
	volatile uint32_t dir;
	volatile uint32_t dirclr;
	volatile uint32_t dirset;
	volatile uint32_t dirtgl;
	volatile uint32_t out;
	volatile uint32_t outclr;
	volatile uint32_t outset;
	volatile uint32_t outtgl;
	volatile uint32_t in;
	volatile uint32_t ctrl;
	volatile uint32_t wrconfig;
	const uint32_t reserved1;
	volatile uint8_t pmux[16];
	volatile uint8_t pcfg[32];
} port_s;

static port_s *const porta = (port_s *)SAMD_PORTA_BASE;

void __attribute__((naked, used, section(".entry"))) samd_spi_write_stub(
	const uint16_t command, const uint32_t address, const uint8_t *const data, const uint32_t length)
{
	/* Create a stack for sanity */
	__asm__("ldr r4, =#0x20001000\n"
			"mov sp, r4\n"
			"bl samd_spi_write\n"
			"bkpt #1\n");
}

static uint8_t samd_spi_xfer(const uint8_t data)
{
	while (!(sercom0->itr_flags & SAMD_SERCOMx_INTFLAG_DRE))
		continue;
	sercom0->data = data;
	while (!(sercom0->itr_flags & SAMD_SERCOMx_INTFLAG_RXC))
		continue;
	return sercom0->data;
}

static void samd_spi_setup_xfer(const uint16_t command, const uint32_t address)
{
	porta->outclr = SAMD_PIN4;

	/* Set up the instruction */
	const uint8_t opcode = command & SPI_FLASH_OPCODE_MASK;
	samd_spi_xfer(opcode);

	if ((command & SPI_FLASH_OPCODE_MODE_MASK) == SPI_FLASH_OPCODE_3B_ADDR) {
		/* For each byte sent here, we have to manually clean up from the controller with a read */
		samd_spi_xfer((address >> 16U) & 0xffU);
		samd_spi_xfer((address >> 8U) & 0xffU);
		samd_spi_xfer(address & 0xffU);
	}

	const size_t inter_length = (command & SPI_FLASH_DUMMY_MASK) >> SPI_FLASH_DUMMY_SHIFT;
	for (size_t i = 0; i < inter_length; ++i)
		/* For each byte sent here, we have to manually clean up from the controller with a read */
		samd_spi_xfer(0);
}

static void __attribute__((used, section(".entry"))) samd_spi_write(
	const uint16_t command, const uint32_t address, const uint8_t *const data, const uint32_t length)
{
	samd_spi_setup_xfer(command, address);

	for (size_t i = 0; i < length; ++i)
		samd_spi_xfer(data[i]);

	porta->outset = SAMD_PIN4;
}

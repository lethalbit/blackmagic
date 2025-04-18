/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2023-2024 1BitSquared <info@1bitsquared.com>
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

#ifndef TARGET_RISCV_DEBUG_H
#define TARGET_RISCV_DEBUG_H

#include <stdint.h>
#include <stdbool.h>
#include "target.h"
#include "adiv5.h"

typedef enum riscv_debug_version {
	RISCV_DEBUG_UNKNOWN,
	RISCV_DEBUG_UNIMPL,
	RISCV_DEBUG_0_11,
	RISCV_DEBUG_0_13,
	RISCV_DEBUG_1_0,
} riscv_debug_version_e;

/* This enum describes the Hart status (eg after a CSR read/write) */
typedef enum riscv_hart_status {
	/* The Hart is in a good state */
	RISCV_HART_NO_ERROR = 0,
	/* The Hart was busy when the status was read */
	RISCV_HART_BUSY = 1,
	/* The operation requested of the Hart was not supported */
	RISCV_HART_NOT_SUPP = 2,
	/* An exception occurred on the Hart while running the operation */
	RISCV_HART_EXCEPTION = 3,
	/* The Hart is in the wrong state for the requested operation */
	RISCV_HART_WRONG_STATE = 4,
	/* The operation triggered a Hart bus error (bad alignment, access size, or timeout) */
	RISCV_HART_BUS_ERROR = 5,
	/* The operation failed for other (unknown) reasons */
	RISCV_HART_OTHER = 7,
} riscv_hart_status_e;

/* This enum describes the current state of a trigger in the TM */
typedef enum riscv_trigger_state {
	RISCV_TRIGGER_MODE_UNUSED = 0x00000000U,
	RISCV_TRIGGER_MODE_BREAKPOINT = 0x00010000U,
	RISCV_TRIGGER_MODE_WATCHPOINT = 0x00020000U,
} riscv_trigger_state_e;

/* The size bits are 22:21 + 17:16, but the upper 2 are only valid on rv64 */
typedef enum riscv_match_size {
	RV_MATCH_SIZE_8_BIT = 0x00010000U,
	RV_MATCH_SIZE_16_BIT = 0x00020000U,
	RV_MATCH_SIZE_32_BIT = 0x00030000U,
	RV_MATCH_SIZE_48_BIT = 0x00200000U,
	RV_MATCH_SIZE_64_BIT = 0x00210000U,
	RV_MATCH_SIZE_80_BIT = 0x00220000U,
	RV_MATCH_SIZE_96_BIT = 0x00230000U,
	RV_MATCH_SIZE_112_BIT = 0x00400000U,
	RV_MATCH_SIZE_128_BIT = 0x00410000U,
} riscv_match_size_e;

/* These defines specify Hart-specific information such as which memory access style to use */
#define RV_HART_FLAG_ACCESS_WIDTH_MASK  0x0fU
#define RV_HART_FLAG_ACCESS_WIDTH_8BIT  (1U << 0U)
#define RV_HART_FLAG_ACCESS_WIDTH_16BIT (1U << 1U)
#define RV_HART_FLAG_ACCESS_WIDTH_32BIT (1U << 2U)
#define RV_HART_FLAG_ACCESS_WIDTH_64BIT (1U << 3U)
#define RV_HART_FLAG_MEMORY_MASK        (1U << 4U)
#define RV_HART_FLAG_MEMORY_ABSTRACT    (0U << 4U)
#define RV_HART_FLAG_MEMORY_SYSBUS      (1U << 4U)
#define RV_HART_FLAG_DATA_GPR_ONLY      (1U << 5U) /* Hart supports Abstract Data commands for GPRs only */

typedef struct riscv_dmi riscv_dmi_s;

/* This structure represents a version-agnostic Debug Module Interface on a RISC-V device */
struct riscv_dmi {
	uint32_t ref_count;

	uint16_t designer_code;
	riscv_debug_version_e version;

	uint8_t dev_index;
	uint8_t idle_cycles;
	uint8_t address_width;
	uint8_t fault;

	void (*prepare)(target_s *target);
	void (*quiesce)(target_s *target);
	bool (*read)(riscv_dmi_s *dmi, uint32_t address, uint32_t *value);
	bool (*write)(riscv_dmi_s *dmi, uint32_t address, uint32_t value);
};

/* This structure represent a DMI bus that is accessed via an ADI AP */
typedef struct riscv_dmi_ap {
	riscv_dmi_s dmi;
	adiv5_access_port_s *ap;
} riscv_dmi_ap_s;

/* This represents a specific Debug Module on the DMI bus */
typedef struct riscv_dm {
	uint32_t ref_count;

	riscv_dmi_s *dmi_bus;
	uint32_t base;
	riscv_debug_version_e version;
} riscv_dm_s;

#define RV_TRIGGERS_MAX 8U

/* This represents a specific Hart on a DM */
typedef struct riscv_hart {
	riscv_dm_s *dbg_module;
	uint32_t hart_idx;
	uint32_t hartsel;
	uint8_t access_width;
	uint8_t address_width;
	uint8_t flags;
	uint8_t progbuf_size;
	riscv_hart_status_e status;

	uint32_t extensions;
	uint32_t vendorid;
	uint32_t archid;
	uint32_t implid;
	uint32_t hartid;

	char isa_name[32U];

	uint32_t triggers;
	uint32_t trigger_uses[RV_TRIGGERS_MAX];
} riscv_hart_s;

#define RV_STATUS_VERSION_MASK 0x0000000fU

#define RV_DM_DATA0             0x04U
#define RV_DM_DATA1             0x05U
#define RV_DM_DATA2             0x06U
#define RV_DM_DATA3             0x07U
#define RV_DM_ABST_CTRLSTATUS   0x16U
#define RV_DM_ABST_COMMAND      0x17U
#define RV_DM_SYSBUS_CTRLSTATUS 0x38U
#define RV_DM_SYSBUS_ADDR0      0x39U
#define RV_DM_SYSBUS_ADDR1      0x3aU
#define RV_DM_SYSBUS_DATA0      0x3cU
#define RV_DM_SYSBUS_DATA1      0x3dU

#define RV_DM_ABST_CMD_ACCESS_REG 0x00000000U
#define RV_DM_ABST_CMD_ACCESS_MEM 0x02000000U

#define RV_ABST_READ          (0U << 16U)
#define RV_ABST_WRITE         (1U << 16U)
#define RV_REG_XFER           (1U << 17U)
#define RV_ABST_POSTEXEC      (1U << 18U)
#define RV_REG_ACCESS_32_BIT  (2U << 20U)
#define RV_REG_ACCESS_64_BIT  (3U << 20U)
#define RV_REG_ACCESS_128_BIT (4U << 20U)

#define RV_MEM_ACCESS_8_BIT   0x0U
#define RV_MEM_ACCESS_16_BIT  0x1U
#define RV_MEM_ACCESS_32_BIT  0x2U
#define RV_MEM_ACCESS_64_BIT  0x3U
#define RV_MEM_ACCESS_128_BIT 0x4U

#define RV_ABST_MEM_ADDR_POST_INC 0x00080000U
#define RV_ABST_MEM_ACCESS_SHIFT  20U

#define RV_SYSBUS_MEM_ADDR_POST_INC 0x00010000U
#define RV_SYSBUS_MEM_READ_ON_ADDR  0x00100000U
#define RV_SYSBUS_MEM_READ_ON_DATA  0x00008000U
#define RV_SYSBUS_STATUS_BUSY       0x00200000U
#define RV_SYSBUS_MEM_ACCESS_SHIFT  17U

/* dpc -> Debug Program Counter */
#define RV_DPC 0x7b1U
/* The GPR base defines the starting register space address for the CPU state registers */
#define RV_GPR_BASE 0x1000U
/* The FP base defines the starting register space address for the floating point registers */
#define RV_FP_BASE 0x1020U

/**
 * The MXL (Machine XLEN) field encodes the native base integer ISA width
 * 
 * The RISC-V Machine ISA register is MXLEN bits wide so the MXL offset is not fixed
 * To work around this we convert the register to it's canonical 32-bit form internally
 */
#define RV_ISA_MXL_SHIFT 30U                        /* misa Machine XLEN field shift (for 32-bit misa) */
#define RV_ISA_MXL_MASK  (0x3U << RV_ISA_MXL_SHIFT) /* misa Machine XLEN field mask (for 32-bit misa) */
#define RV_ISA_MXL_32    0x1U                       /* misa Machine XLEN field value for 32-bit ISA */
#define RV_ISA_MXL_64    0x2U                       /* misa Machine XLEN field value for 64-bit ISA */
#define RV_ISA_MXL_128   0x3U                       /* misa Machine XLEN field value for 128-bit ISA */

/* 
 * The Extensions field encodes the presence of standard extensions, with a single bit per alphabet letter
 * (bit 0 encodes presence of extension “A” through to bit 25 which encodes “Z”)
 * 
 * This list is taken from the RISC-V Instruction Set Manual v2.2
 * 
 * The list order is the canonical representation order in the ISA subset string
 */
#define RV_ISA_EXTENSIONS_MASK 0x03ffffffU /* misa extensions field mask */

/* Base ISA */
#define RV_ISA_EXT_INTEGER  (1U << 8U) /* 'I': RV32I/64I/128I integer base ISA */
#define RV_ISA_EXT_EMBEDDED (1U << 4U) /* 'E': RV32E reduced integer base ISA (Embedded) */

/* Standard general-purpose ISA */
#define RV_ISA_EXT_MUL_DIV_INT  (1U << 12U) /* 'M': Integer multiplication and division */
#define RV_ISA_EXT_ATOMIC       (1U << 0U)  /* 'A': Atomic instructions */
#define RV_ISA_EXT_SINGLE_FLOAT (1U << 5U)  /* 'F': Single-precision floating-point */
#define RV_ISA_EXT_DOUBLE_FLOAT (1U << 3U)  /* 'D': Double-precision floating-point */

/* 'G' standard general-purpose ISA abbreviation, representing 'IMAFD' */
#define RV_ISA_EXT_GENERAL_PURPOSE                                                               \
	(RV_ISA_EXT_INTEGER | RV_ISA_EXT_MUL_DIV_INT | RV_ISA_EXT_ATOMIC | RV_ISA_EXT_SINGLE_FLOAT | \
		RV_ISA_EXT_DOUBLE_FLOAT)

/* Standard Unprivileged Extensions */
#define RV_ISA_EXT_QUAD_FLOAT      (1U << 16U) /* 'Q': Quad-precision floating-point */
#define RV_ISA_EXT_DECIMAL_FLOAT   (1U << 11U) /* 'L': Decimal floating-point */
#define RV_ISA_EXT_COMPRESSED      (1U << 2U)  /* 'C': 16-bit compressed instructions */
#define RV_ISA_EXT_BIT_MANIP       (1U << 1U)  /* 'B': Bit manipulation */
#define RV_ISA_EXT_DYNAMIC_LANG    (1U << 9U)  /* 'J': Dynamic languages  */
#define RV_ISA_EXT_TRANSACT_MEM    (1U << 19U) /* 'T': Transactional memory */
#define RV_ISA_EXT_PACKED_SIMD     (1U << 15U) /* 'P': Packed-SIMD */
#define RV_ISA_EXT_VECTOR          (1U << 21U) /* 'V': Vector extensions */
#define RV_ISA_EXT_USER_INTERRUPTS (1U << 13U) /* 'N': User-level interrupts */

#define RV_TRIGGER_SUPPORT_MASK       0x0000fffeU
#define RV_TRIGGER_MODE_MASK          0xffff0000U
#define RV_TRIGGER_SUPPORT_BREAKWATCH 0x00000004U

/*
 * The CSR number when requested by GDB is shifted by RV_CSR_GDB_OFFSET so they cannot collide with
 * the GPRs. As a result, we have to subtract RV_CSR_GDB_OFFSET from the value received from GDB.
 */
#define RV_CSR_GDB_OFFSET 128U
#define RV_CSR_STATUS     0x300U
#define RV_CSR_MISA       0x301U
#define RV_CSR_MIE        0x304U
#define RV_CSR_MTVEC      0x305U
#define RV_CSR_MSCRATCH   0x340U
#define RV_CSR_MEPC       0x341U
#define RV_CSR_MCAUSE     0x342U
#define RV_CSR_MTVAL      0x343U
#define RV_CSR_MIP        0x344U

/*
 * These two lines are about allowing GDB to access FPU registers through fake registers offset by
 * RV_FPU_GDB_OFFSET for the normal FPU registers and RV_FPU_GDB_CSR_OFFSET for FPU related CSRs
 */
#define RV_FPU_GDB_OFFSET     33
#define RV_FPU_GDB_CSR_OFFSET 66

/* JTAG DTM function declarations */
#ifdef CONFIG_RISCV
void riscv_jtag_dtm_handler(uint8_t dev_index);
void riscv_adi_dtm_handler(adiv5_access_port_s *ap);
#endif
bool riscv_jtag_dmi_read(riscv_dmi_s *dmi, uint32_t address, uint32_t *value);
bool riscv_jtag_dmi_write(riscv_dmi_s *dmi, uint32_t address, uint32_t value);

void riscv_dmi_init(riscv_dmi_s *dmi);
riscv_hart_s *riscv_hart_struct(target_s *target);

#if CONFIG_BMDA == 1
/* BMDA interposition functions for DP setup */
void bmda_riscv_jtag_dtm_init(riscv_dmi_s *dmi);
#endif

bool riscv_dm_read(riscv_dm_s *dbg_module, uint8_t address, uint32_t *value);
bool riscv_dm_write(riscv_dm_s *dbg_module, uint8_t address, uint32_t value);
bool riscv_command_wait_complete(riscv_hart_s *hart);
bool riscv_csr_read(riscv_hart_s *hart, uint16_t reg, void *data);
bool riscv_csr_write(riscv_hart_s *hart, uint16_t reg, const void *data);
riscv_match_size_e riscv_breakwatch_match_size(size_t size);
bool riscv_config_trigger(
	riscv_hart_s *hart, uint32_t trigger, riscv_trigger_state_e mode, const void *config, const void *address);

bool riscv_attach(target_s *target);
void riscv_detach(target_s *target);

uint8_t riscv_mem_access_width(const riscv_hart_s *hart, target_addr_t address, size_t length);
void riscv32_unpack_data(void *dest, uint32_t data, uint8_t access_width);
uint32_t riscv32_pack_data(const void *src, uint8_t access_width);

void riscv32_mem_read(target_s *target, void *dest, target_addr64_t src, size_t len);
void riscv32_mem_write(target_s *target, target_addr64_t dest, const void *src, size_t len);

#endif /*TARGET_RISCV_DEBUG_H*/

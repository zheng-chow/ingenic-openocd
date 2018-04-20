/***************************************************************************
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by David T.L. Wong                                 *
 *                                                                         *
 *   Copyright (C) 2011 by Drasko DRASKOVIC                                *
 *   drasko.draskovic@gmail.com                                            *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_MIPS32_H
#define OPENOCD_TARGET_MIPS32_H

#include "target.h"
#include "mips32_pracc.h"

#define MIPS32_COMMON_MAGIC     0xB320B320

/* Cache flush type*/
#define ALL                     0
#define INSTNOWB                1
#define DATA                    2
#define ALLNOWB                 3
#define DATANOWB                4
#define L2                      5
#define L2NOWB                  6

/* Mips reg name*/
#define zero    0
#define AT      1
#define v0      2
#define v1      3
#define a0      4
#define a1      5
#define a2      6
#define a3      7
#define t0      8
#define t1      9
#define t2      10
#define t3      11
#define t4      12
#define t5      13
#define t6      14
#define t7      15
#define s0      16
#define s1      17
#define s2      18
#define s3      19
#define s4      20
#define s5      21
#define s6      22
#define s7      23
#define s8      30              /* == fp */
#define t8      24
#define t9      25
#define k0      26
#define k1      27
#define sp      29
#define fp      30
#define ra      31

/**
 * Memory segments (32bit kernel mode addresses)
 * These are the traditional names used in the 32-bit universe.
 */
#define KUSEG			0x00000000
#define KSEG0			0x80000000
#define KSEG1			0xa0000000
#define KSEG2			0xc0000000
#define KSEG3			0xe0000000

/** Returns the kernel segment base of a given address */
#define KSEGX(a)		((a) & 0xe0000000)

/** CP0 CONFIG0 regites fields */
#define MIPS32_CONFIG0_KU_SHIFT 25
#define MIPS32_CONFIG0_KU_MASK (0x7 << MIPS32_CONFIG0_KU_SHIFT)

#define MIPS32_CONFIG0_K0_SHIFT 0
#define MIPS32_CONFIG0_K0_MASK (0x7 << MIPS32_CONFIG0_K0_SHIFT)

#define MIPS32_CONFIG0_K23_SHIFT 28
#define MIPS32_CONFIG0_K23_MASK (0x7 << MIPS32_CONFIG0_K23_SHIFT)

#define MIPS32_CONFIG0_AR_SHIFT 10
#define MIPS32_CONFIG0_AR_MASK (0x7 << MIPS32_CONFIG0_AR_SHIFT)

/** CP0 CONFIG1 regites fields */
#define MIPS32_CONFIG1_IS_SHIFT 22
#define MIPS32_CONFIG1_IS_MASK (0x7 << MIPS32_CONFIG1_IS_SHIFT)

#define MIPS32_CONFIG1_IL_SHIFT 19
#define MIPS32_CONFIG1_IL_MASK (0x7 << MIPS32_CONFIG1_IL_SHIFT)

#define MIPS32_CONFIG1_IA_SHIFT 16
#define MIPS32_CONFIG1_IA_MASK (0x7 << MIPS32_CONFIG1_IA_SHIFT)

#define MIPS32_CONFIG1_DS_SHIFT 13
#define MIPS32_CONFIG1_DS_MASK (0x7 << MIPS32_CONFIG1_DS_SHIFT)

#define MIPS32_CONFIG1_DL_SHIFT 10
#define MIPS32_CONFIG1_DL_MASK (0x7 << MIPS32_CONFIG1_DL_SHIFT)

#define MIPS32_CONFIG1_DA_SHIFT 7
#define MIPS32_CONFIG1_DA_MASK (0x7 << MIPS32_CONFIG1_DA_SHIFT)

#define MIPS32_CONFIG1_FP_SHIFT 0
#define MIPS32_CONFIG1_FP_MASK (0x1 << MIPS32_CONFIG1_FP_SHIFT)

/** CP0 CONFIG2 regites fields */
#define MIPS32_CONFIG2_SS_SHIFT 8
#define MIPS32_CONFIG2_SS_MASK (0xf << MIPS32_CONFIG1_SS_SHIFT)

#define MIPS32_CONFIG2_SL_SHIFT 4
#define MIPS32_CONFIG2_SL_MASK (0xf << MIPS32_CONFIG1_SL_SHIFT)

#define MIPS32_CONFIG2_SA_SHIFT 0
#define MIPS32_CONFIG2_SA_MASK (0xf << MIPS32_CONFIG1_SA_SHIFT)

/** CP0 CONFIG3 regites fields */
#define MIPS32_CONFIG3_ISA_SHIFT	14
#define MIPS32_CONFIG3_ISA_MASK		(3 << MIPS32_CONFIG3_ISA_SHIFT)

/** CP0 STATUS regites fields */
#define MIPS32_STATUS_CU3_SHIFT		31
#define MIPS32_STATUS_CU3_MASK		(1 << MIPS32_STATUS_CU3_SHIFT)

#define MIPS32_STATUS_CU2_SHIFT		30
#define MIPS32_STATUS_CU2_MASK		(1 << MIPS32_STATUS_CU2_SHIFT)

#define MIPS32_STATUS_CU1_SHIFT		29
#define MIPS32_STATUS_CU1_MASK		(1 << MIPS32_STATUS_CU1_SHIFT)

#define MIPS32_STATUS_CU0_SHIFT		28
#define MIPS32_STATUS_CU0_MASK		(1 << MIPS32_STATUS_CU0_SHIFT)

/*
 * MIPS32 Coprocessor 0 register numbers
 */
#define C0_INDEX                0
#define C0_INX                  0
#define C0_RANDOM               1
#define C0_RAND                 1
#define C0_ENTRYLO0             2
#define C0_TLBLO0               2
#define C0_ENTRYLO1             3
#define C0_TLBLO1               3
#define C0_CONTEXT              4
#define C0_CTXT                 4
#define C0_PAGEMASK             5
#define C0_PAGEGRAIN            (5, 1)
#define C0_WIRED                6
#define C0_HWRENA               7
#define C0_BADVADDR             8
#define C0_VADDR                8
#define C0_COUNT                9
#define C0_ENTRYHI              10
#define C0_TLBHI                10
#define C0_GUESTCTL1            10
#define C0_COMPARE              11
#define C0_STATUS               12
#define C0_SR                   12
#define C0_INTCTL               (12, 1)
#define C0_SRSCTL               (12, 2)
#define C0_SRSMAP               (12, 3)
#define C0_CAUSE                13
#define C0_CR                   13
#define C0_EPC                  14
#define C0_PRID                 15
#define C0_EBASE                (15, 1)
#define C0_CONFIG               16
#define C0_CONFIG0              (16, 0)
#define C0_CONFIG1              (16, 1)
#define C0_CONFIG2              (16, 2)
#define C0_CONFIG3              (16, 3)
#define C0_LLADDR               17
#define C0_WATCHLO              18
#define C0_WATCHHI              19
#define C0_DEBUG                23
#define C0_DEPC                 24
#define C0_PERFCNT              25
#define C0_ERRCTL               26
#define C0_CACHEERR             27
#define C0_TAGLO                28
#define C0_ITAGLO               28
#define C0_DTAGLO               (28, 2)
#define C0_TAGLO2               (28, 4)
#define C0_DATALO               (28, 1)
#define C0_IDATALO              (28, 1)
#define C0_DDATALO              (28, 3)
#define C0_DATALO2              (28, 5)
#define C0_TAGHI                29
#define C0_ITAGHI               29
#define C0_DATAHI               (29, 1)
#define C0_ERRPC                30
#define C0_DESAVE               31

/*
 * Cache operations
 */
#define Index_Invalidate_I                               0x00            /* 0           0 */
#define Index_Writeback_Inv_D                            0x01            /* 0           1 */
#define Index_Writeback_Inv_T                            0x02            /* 0           2 */
#define Index_Writeback_Inv_S                            0x03            /* 0           3 */
#define Index_Load_Tag_I                                 0x04            /* 1           0 */
#define Index_Load_Tag_D                                 0x05            /* 1           1 */
#define Index_Load_Tag_T                                 0x06            /* 1           2 */
#define Index_Load_Tag_S                                 0x07            /* 1           3 */
#define Index_Store_Tag_I                                0x08            /* 2           0 */
#define Index_Store_Tag_D                                0x09            /* 2           1 */
#define Index_Store_Tag_T                                0x0A            /* 2           2 */
#define Index_Store_Tag_S                                0x0B            /* 2           3 */
#define Hit_Invalidate_I                                 0x10            /* 4           0 */
#define Hit_Invalidate_D                                 0x11            /* 4           1 */
#define Hit_Invalidate_T                                 0x12            /* 4           2 */
#define Hit_Invalidate_S                                 0x13            /* 4           3 */
#define Fill_I                                           0x14            /* 5           0 */
#define Hit_Writeback_Inv_D                              0x15            /* 5           1 */
#define Hit_Writeback_Inv_T                              0x16            /* 5           2 */
#define Hit_Writeback_Inv_S                              0x17            /* 5           3 */
#define Hit_Writeback_D                                  0x19            /* 6           1 */
#define Hit_Writeback_T                                  0x1A            /* 6           1 */
#define Hit_Writeback_S                                  0x1B            /* 6           3 */
#define Fetch_Lock_I                                     0x1C            /* 7           0 */
#define Fetch_Lock_D                                     0x1D            /* 7           1 */


#define MIPS32_ARCH_REL1 0x0
#define MIPS32_ARCH_REL2 0x1

#define MIPS32_SCAN_DELAY_LEGACY_MODE 2000000

#define MIPS_CORE_M4K  		0x1000
#define MIPS_CORE_INGENIC	0x40000000





/* offsets into mips32 core register cache */
enum {
        MIPS32_PC = 37,
        MIPS32_F0 = 38,
        MIPS32_FCSR = 70,
        MIPS32_FIR = 71,
        MIPS32NUMCOREREGS
};

enum mips_fp {
         FP_NOT_IMP = 0,
         FP_IMP = 1,
};

enum mips32_isa_mode {
	MIPS32_ISA_MIPS32 = 0,
	MIPS32_ISA_MIPS16E = 1,
	MIPS32_ISA_MMIPS32 = 3,
};

enum mips32_isa_imp {
	MIPS32_ONLY = 0,
	MMIPS32_ONLY = 1,
	MIPS32_MIPS16 = 2,
	MIPS32_MMIPS32 = 3,
};


enum mips32_core_type {
	CORE_TYPE_UNKNOWN		=	0,
	MIPS_M4K	  			=	0x0008 | MIPS_CORE_M4K,
	MIPS_INGENIC_XBURST1 	=	0x0000 | MIPS_CORE_INGENIC,
	MIPS_INGENIC_XBURST2 	=	0x0001 | MIPS_CORE_INGENIC,
};

struct mips32_comparator {
	int used;
	uint32_t bp_value;
	uint32_t reg_address;
};

struct mips32_common {
	uint32_t common_magic;
	void *arch_info;
	struct reg_cache *core_cache;
	struct mips_ejtag ejtag_info;
	uint32_t core_regs[MIPS32NUMCOREREGS];
	enum mips32_isa_mode isa_mode;
	enum mips32_isa_imp isa_imp;
	enum mips_fp fp_imp;

	/* working area for fastdata access */
	struct working_area *fast_data_area;

	int bp_scanned;
	int num_inst_bpoints;
	int num_data_bpoints;
	int num_inst_bpoints_avail;
	int num_data_bpoints_avail;
	struct mips32_comparator *inst_break_list;
	struct mips32_comparator *data_break_list;

	/* register cache to processor synchronization */
	int (*read_core_reg)(struct target *target, unsigned int num);
	int (*write_core_reg)(struct target *target, unsigned int num);
};

static inline struct mips32_common *
target_to_mips32(struct target *target)
{
	return target->arch_info;
}

struct mips32_core_reg {
	uint32_t num;
	struct target *target;
	struct mips32_common *mips32_common;
};

struct mips32_algorithm {
	int common_magic;
	enum mips32_isa_mode isa_mode;
};




#define MIPS32_OP_ADDU	0x21u
#define MIPS32_OP_ADDIU	0x09u
#define MIPS32_OP_ANDI	0x0Cu
#define MIPS32_OP_BEQ	0x04u
#define MIPS32_OP_BGTZ	0x07u
#define MIPS32_OP_BNE	0x05u
#define MIPS32_OP_ADDI	0x08u
#define MIPS32_OP_AND	0x24u
#define MIPS32_OP_CACHE	0x2Fu
#define MIPS32_OP_COP0	0x10u
#define MIPS32_OP_COP1	0x11u
#define MIPS32_OP_J	0x02u
#define MIPS32_OP_JR	0x08u
#define MIPS32_OP_LUI	0x0Fu
#define MIPS32_OP_LW	0x23u
#define MIPS32_OP_LB	0x20u
#define MIPS32_OP_LBU	0x24u
#define MIPS32_OP_LHU	0x25u
#define MIPS32_OP_MFHI	0x10u
#define MIPS32_OP_MTHI	0x11u
#define MIPS32_OP_MFLO	0x12u
#define MIPS32_OP_MTLO	0x13u
#define MIPS32_OP_MUL   0x02u
#define MIPS32_OP_RDHWR	0x3Bu
#define MIPS32_OP_SB	0x28u
#define MIPS32_OP_SH	0x29u
#define MIPS32_OP_SW	0x2Bu
#define MIPS32_OP_ORI	0x0Du
#define MIPS32_OP_XORI	0x0Eu
#define MIPS32_OP_XOR	0x26u
#define MIPS32_OP_SLTU	0x2Bu
#define MIPS32_OP_SRL	0x03u
#define MIPS32_OP_SYNCI	0x1Fu
#define MIPS32_OP_SLL	0x00u
#define MIPS32_OP_SLLV  0x04u
#define MIPS32_OP_SLTI	0x0Au
#define MIPS32_OP_MOVN	0x0Bu

#define MIPS32_OP_REGIMM	0x01u
#define MIPS32_OP_SDBBP	0x3Fu
#define MIPS32_OP_SPECIAL	0x00u
#define MIPS32_OP_SPECIAL2	0x07u
#define MIPS32_OP_SPECIAL3	0x1Fu

#define MIPS32_COP0_MF	0x00u
#define MIPS32_COP0_MT	0x04u
#define MIPS32_COP1_MF	0x00u
#define MIPS32_COP1_MT	0x04u
#define MIPS32_COP1_CF	0x02u
#define MIPS32_COP1_CT	0x06u

#define MIPS32_R_INST(opcode, rs, rt, rd, shamt, funct) \
	(((opcode) << 26) | ((rs) << 21) | ((rt) << 16) | ((rd) << 11) | ((shamt) << 6) | (funct))
#define MIPS32_I_INST(opcode, rs, rt, immd) \
	(((opcode) << 26) | ((rs) << 21) | ((rt) << 16) | (immd))
#define MIPS32_J_INST(opcode, addr)	(((opcode) << 26) | (addr))

#define MIPS32_ISA_NOP				0
#define MIPS32_ISA_ADDI(tar, src, val)		MIPS32_I_INST(MIPS32_OP_ADDI, src, tar, val)
#define MIPS32_ISA_ADDIU(tar, src, val)		MIPS32_I_INST(MIPS32_OP_ADDIU, src, tar, val)
#define MIPS32_ISA_ADDU(dst, src, tar)		MIPS32_R_INST(MIPS32_OP_SPECIAL, src, tar, dst, 0, MIPS32_OP_ADDU)
#define MIPS32_ISA_AND(dst, src, tar)		MIPS32_R_INST(0, src, tar, dst, 0, MIPS32_OP_AND)
#define MIPS32_ISA_ANDI(tar, src, val)		MIPS32_I_INST(MIPS32_OP_ANDI, src, tar, val)

#define MIPS32_ISA_B(off)			MIPS32_ISA_BEQ(0, 0, off)
#define MIPS32_ISA_BEQ(src, tar, off)		MIPS32_I_INST(MIPS32_OP_BEQ, src, tar, off)
#define MIPS32_ISA_BGTZ(reg, off)		MIPS32_I_INST(MIPS32_OP_BGTZ, reg, 0, off)
#define MIPS32_ISA_BNE(src, tar, off)		MIPS32_I_INST(MIPS32_OP_BNE, src, tar, off)
#define MIPS32_ISA_CACHE(op, off, base)		MIPS32_I_INST(MIPS32_OP_CACHE, base, op, off)
#define MIPS32_ISA_J(tar)			MIPS32_J_INST(MIPS32_OP_J, (0x0FFFFFFFu & (tar)) >> 2)
#define MIPS32_ISA_JR(reg)			MIPS32_R_INST(0, reg, 0, 0, 0, MIPS32_OP_JR)

#define MIPS32_ISA_LB(reg, off, base)		MIPS32_I_INST(MIPS32_OP_LB, base, reg, off)
#define MIPS32_ISA_LBU(reg, off, base)		MIPS32_I_INST(MIPS32_OP_LBU, base, reg, off)
#define MIPS32_ISA_LHU(reg, off, base)		MIPS32_I_INST(MIPS32_OP_LHU, base, reg, off)
#define MIPS32_ISA_LUI(reg, val)		MIPS32_I_INST(MIPS32_OP_LUI, 0, reg, val)
#define MIPS32_ISA_LW(reg, off, base)		MIPS32_I_INST(MIPS32_OP_LW, base, reg, off)

#define MIPS32_ISA_MFC0(gpr, cpr, sel)		MIPS32_R_INST(MIPS32_OP_COP0, MIPS32_COP0_MF, gpr, cpr, 0, sel)
#define MIPS32_ISA_MTC0(gpr, cpr, sel)		MIPS32_R_INST(MIPS32_OP_COP0, MIPS32_COP0_MT, gpr, cpr, 0, sel)
#define MIPS32_ISA_MFC1(rt, fs)                 MIPS32_R_INST(MIPS32_OP_COP1, MIPS32_COP1_MF, rt, fs, 0, 0)
#define MIPS32_ISA_MTC1(rt, fs)                 MIPS32_R_INST(MIPS32_OP_COP1, MIPS32_COP1_MT, rt, fs, 0, 0)
#define MIPS32_ISA_CFC1(rt, fs)                 MIPS32_R_INST(MIPS32_OP_COP1, MIPS32_COP1_CF, rt, fs, 0, 0)
#define MIPS32_ISA_CTC1(rt, fs)                 MIPS32_R_INST(MIPS32_OP_COP1, MIPS32_COP1_CT, rt, fs, 0, 0)
#define MIPS32_ISA_MFLO(reg)			MIPS32_R_INST(0, 0, 0, reg, 0, MIPS32_OP_MFLO)
#define MIPS32_ISA_MFHI(reg)			MIPS32_R_INST(0, 0, 0, reg, 0, MIPS32_OP_MFHI)
#define MIPS32_ISA_MTLO(reg)			MIPS32_R_INST(0, reg, 0, 0, 0, MIPS32_OP_MTLO)
#define MIPS32_ISA_MTHI(reg)			MIPS32_R_INST(0, reg, 0, 0, 0, MIPS32_OP_MTHI)
#define MIPS32_ISA_MUL(dst, src, t)             MIPS32_R_INST(28, src, t, dst, 0, MIPS32_OP_MUL)

#define MIPS32_ISA_MOVN(dst, src, tar)		MIPS32_R_INST(MIPS32_OP_SPECIAL, src, tar, dst, 0, MIPS32_OP_MOVN)
#define MIPS32_ISA_ORI(tar, src, val)		MIPS32_I_INST(MIPS32_OP_ORI, src, tar, val)
#define MIPS32_ISA_RDHWR(tar, dst)		MIPS32_R_INST(MIPS32_OP_SPECIAL3, 0, tar, dst, 0, MIPS32_OP_RDHWR)
#define MIPS32_ISA_SB(reg, off, base)		MIPS32_I_INST(MIPS32_OP_SB, base, reg, off)
#define MIPS32_ISA_SH(reg, off, base)		MIPS32_I_INST(MIPS32_OP_SH, base, reg, off)
#define MIPS32_ISA_SW(reg, off, base)		MIPS32_I_INST(MIPS32_OP_SW, base, reg, off)

#define MIPS32_ISA_SLL(dst, src, sa)		MIPS32_R_INST(MIPS32_OP_SPECIAL, 0, src, dst, sa, MIPS32_OP_SLL)
#define MIPS32_ISA_SLLV(dst, tar, src)          MIPS32_R_INST(MIPS32_OP_SPECIAL, src, tar, dst, 0, MIPS32_OP_SLLV)
#define MIPS32_ISA_SLTI(tar, src, val)		MIPS32_I_INST(MIPS32_OP_SLTI, src, tar, val)
#define MIPS32_ISA_SLTU(dst, src, tar)		MIPS32_R_INST(MIPS32_OP_SPECIAL, src, tar, dst, 0, MIPS32_OP_SLTU)
#define MIPS32_ISA_SRL(reg, src, off)		MIPS32_R_INST(0, 0, src, reg, off, MIPS32_OP_SRL)
#define MIPS32_ISA_SYNC				0xFu
#define MIPS32_ISA_SYNCI(off, base)		MIPS32_I_INST(MIPS32_OP_REGIMM, base, MIPS32_OP_SYNCI, off)

#define MIPS32_ISA_XOR(reg, val1, val2)		MIPS32_R_INST(0, val1, val2, reg, 0, MIPS32_OP_XOR)
#define MIPS32_ISA_XORI(tar, src, val)		MIPS32_I_INST(MIPS32_OP_XORI, src, tar, val)

#define MIPS32_ISA_SYNCI_STEP		0x1	/* reg num od address step size to be used with synci instruction */

/**
 * Cache operations definitions
 * Operation field is 5 bits long :
 * 1) bits 1..0 hold cache type
 * 2) bits 4..2 hold operation code
 */
#define MIPS32_CACHE_D_HIT_WRITEBACK ((0x1 << 0) | (0x6 << 2))
#define MIPS32_CACHE_I_HIT_INVALIDATE ((0x0 << 0) | (0x4 << 2))

/* ejtag specific instructions */
#define MIPS32_ISA_DRET				0x4200001Fu
/* MIPS32_ISA_J_INST(MIPS32_ISA_OP_SPECIAL2, MIPS32_ISA_OP_SDBBP) */
#define MIPS32_ISA_SDBBP			0x7000003Fu
#define MIPS16_ISA_SDBBP			0xE801u

/*MICRO MIPS INSTRUCTIONS, see doc MD00582 */
#define POOL32A					0X00u
#define POOL32AXf				0x3Cu
#define POOL32B					0x08u
#define POOL32I					0x10u
#define MMIPS32_OP_ADDI			0x04u
#define MMIPS32_OP_ADDIU		0x0Cu
#define MMIPS32_OP_ADDU			0x150u
#define MMIPS32_OP_AND			0x250u
#define MMIPS32_OP_ANDI			0x34u
#define MMIPS32_OP_BEQ			0x25u
#define MMIPS32_OP_BGTZ			0x06u
#define MMIPS32_OP_BNE			0x2Du
#define MMIPS32_OP_CACHE		0x06u
#define MMIPS32_OP_J			0x35u
#define MMIPS32_OP_JALR			0x03Cu
#define MMIPS32_OP_LB			0x07u
#define MMIPS32_OP_LBU			0x05u
#define MMIPS32_OP_LHU			0x0Du
#define MMIPS32_OP_LUI			0x0Du
#define MMIPS32_OP_LW			0x3Fu
#define MMIPS32_OP_MFC0			0x03u
#define MMIPS32_OP_MTC0			0x0Bu
#define MMIPS32_OP_MFLO			0x075u
#define MMIPS32_OP_MFHI			0x035u
#define MMIPS32_OP_MTLO			0x0F5u
#define MMIPS32_OP_MTHI			0x0B5u
#define MMIPS32_OP_MOVN			0x018u
#define MMIPS32_OP_ORI			0x14u
#define MMIPS32_OP_RDHWR		0x1ACu
#define MMIPS32_OP_SB			0x06u
#define MMIPS32_OP_SH			0x0Eu
#define MMIPS32_OP_SW			0x3Eu
#define MMIPS32_OP_SLTU			0x390u
#define MMIPS32_OP_SLL			0x000u
#define MMIPS32_OP_SLTI			0x24u
#define MMIPS32_OP_SRL			0x040u
#define MMIPS32_OP_SYNCI		0x10u
#define MMIPS32_OP_XOR			0x310u
#define MMIPS32_OP_XORI			0x1Cu

#define MMIPS32_ADDI(tar, src, val)		MIPS32_I_INST(MMIPS32_OP_ADDI, tar, src, val)
#define MMIPS32_ADDIU(tar, src, val)		MIPS32_I_INST(MMIPS32_OP_ADDIU, tar, src, val)
#define MMIPS32_ADDU(dst, src, tar)		MIPS32_R_INST(POOL32A, tar, src, dst, 0, MMIPS32_OP_ADDU)
#define MMIPS32_AND(dst, src, tar)		MIPS32_R_INST(POOL32A, tar, src, dst, 0, MMIPS32_OP_AND)
#define MMIPS32_ANDI(tar, src, val)		MIPS32_I_INST(MMIPS32_OP_ANDI, tar, src, val)

#define MMIPS32_B(off)				MMIPS32_BEQ(0, 0, off)
#define MMIPS32_BEQ(src, tar, off)		MIPS32_I_INST(MMIPS32_OP_BEQ, tar, src, off)
#define MMIPS32_BGTZ(reg, off)			MIPS32_I_INST(POOL32I, MMIPS32_OP_BGTZ, reg, off)
#define MMIPS32_BNE(src, tar, off)		MIPS32_I_INST(MMIPS32_OP_BNE, tar, src, off)
#define MMIPS32_CACHE(op, off, base)		MIPS32_R_INST(POOL32B, op, base, MMIPS32_OP_CACHE << 1, 0, off)

#define MMIPS32_J(tar)				MIPS32_J_INST(MMIPS32_OP_J, ((0x07FFFFFFu & ((tar) >> 1))))
#define MMIPS32_JR(reg)				MIPS32_R_INST(POOL32A, 0, reg, 0, MMIPS32_OP_JALR, POOL32AXf)
#define MMIPS32_LB(reg, off, base)		MIPS32_I_INST(MMIPS32_OP_LB, reg, base, off)
#define MMIPS32_LBU(reg, off, base)		MIPS32_I_INST(MMIPS32_OP_LBU, reg, base, off)
#define MMIPS32_LHU(reg, off, base)		MIPS32_I_INST(MMIPS32_OP_LHU, reg, base, off)
#define MMIPS32_LUI(reg, val)			MIPS32_I_INST(POOL32I, MMIPS32_OP_LUI, reg, val)
#define MMIPS32_LW(reg, off, base)		MIPS32_I_INST(MMIPS32_OP_LW, reg, base, off)

#define MMIPS32_MFC0(gpr, cpr, sel)		MIPS32_R_INST(POOL32A, gpr, cpr, sel, MMIPS32_OP_MFC0, POOL32AXf)
#define MMIPS32_MFLO(reg)			MIPS32_R_INST(POOL32A, 0, reg, 0, MMIPS32_OP_MFLO, POOL32AXf)
#define MMIPS32_MFHI(reg)			MIPS32_R_INST(POOL32A, 0, reg, 0, MMIPS32_OP_MFHI, POOL32AXf)
#define MMIPS32_MTC0(gpr, cpr, sel)		MIPS32_R_INST(POOL32A, gpr, cpr, sel, MMIPS32_OP_MTC0, POOL32AXf)
#define MMIPS32_MTLO(reg)			MIPS32_R_INST(POOL32A, 0, reg, 0, MMIPS32_OP_MTLO, POOL32AXf)
#define MMIPS32_MTHI(reg)			MIPS32_R_INST(POOL32A, 0, reg, 0, MMIPS32_OP_MTHI, POOL32AXf)

#define MMIPS32_MOVN(dst, src, tar)		MIPS32_R_INST(POOL32A, tar, src, dst, 0, MMIPS32_OP_MOVN)
#define MMIPS32_NOP				0
#define MMIPS32_ORI(tar, src, val)		MIPS32_I_INST(MMIPS32_OP_ORI, tar, src, val)
#define MMIPS32_RDHWR(tar, dst)			MIPS32_R_INST(POOL32A, dst, tar, 0, MMIPS32_OP_RDHWR, POOL32AXf)
#define MMIPS32_SB(reg, off, base)		MIPS32_I_INST(MMIPS32_OP_SB, reg, base, off)
#define MMIPS32_SH(reg, off, base)		MIPS32_I_INST(MMIPS32_OP_SH, reg, base, off)
#define MMIPS32_SW(reg, off, base)		MIPS32_I_INST(MMIPS32_OP_SW, reg, base, off)

#define MMIPS32_SRL(reg, src, off)		MIPS32_R_INST(POOL32A, reg, src, off, 0, MMIPS32_OP_SRL)
#define MMIPS32_SLTU(dst, src, tar)		MIPS32_R_INST(POOL32A, tar, src, dst, 0, MMIPS32_OP_SLTU)
#define MMIPS32_SYNCI(off, base)		MIPS32_I_INST(POOL32I, MMIPS32_OP_SYNCI, base, off)
#define MMIPS32_SLL(dst, src, sa)		MIPS32_R_INST(POOL32A, dst, src, sa, 0, MMIPS32_OP_SLL)
#define MMIPS32_SLTI(tar, src, val)		MIPS32_I_INST(MMIPS32_OP_SLTI, tar, src, val)
#define MMIPS32_SYNC				0x00001A7Cu /* MIPS32_R_INST(POOL32A, 0, 0, 0, 0x1ADu, POOL32AXf) */

#define MMIPS32_XOR(reg, val1, val2)		MIPS32_R_INST(POOL32A, val1, val2, reg, 0, MMIPS32_OP_XOR)
#define MMIPS32_XORI(tar, src, val)		MIPS32_I_INST(MMIPS32_OP_XORI, tar, src, val)

#define MMIPS32_SYNCI_STEP	0x1u	/* reg num od address step size to be used with synci instruction */


/* ejtag specific instructions */
#define MMIPS32_DRET			0x0000E37Cu	/* MIPS32_R_INST(POOL32A, 0, 0, 0, 0x38D, POOL32AXf) */
#define MMIPS32_SDBBP			0x0000DB7Cu	/* MIPS32_R_INST(POOL32A, 0, 0, 0, 0x1BD, POOL32AXf) */
#define MMIPS16_SDBBP			0x46C0u		/* POOL16C instr */

/* instruction code with isa selection */
#define MIPS32_NOP				0	/* same for both isa's */
#define MIPS32_ADDI(isa, tar, src, val)		(isa ? MMIPS32_ADDI(tar, src, val) : MIPS32_ISA_ADDI(tar, src, val))
#define MIPS32_ADDIU(isa, tar, src, val)	(isa ? MMIPS32_ADDIU(tar, src, val) : MIPS32_ISA_ADDIU(tar, src, val))
#define MIPS32_ADDU(isa, dst, src, tar)		(isa ? MMIPS32_ADDU(dst, src, tar) : MIPS32_ISA_ADDU(dst, src, tar))
#define MIPS32_AND(isa, dst, src, tar)		(isa ? MMIPS32_AND(dst, src, tar) : MIPS32_ISA_AND(dst, src, tar))
#define MIPS32_ANDI(isa, tar, src, val)		(isa ? MMIPS32_ANDI(tar, src, val) : MIPS32_ISA_ANDI(tar, src, val))

#define MIPS32_B(isa, off)			(isa ? MMIPS32_B(off) : MIPS32_ISA_B(off))
#define MIPS32_BEQ(isa, src, tar, off)		(isa ? MMIPS32_BEQ(src, tar, off) : MIPS32_ISA_BEQ(src, tar, off))
#define MIPS32_BGTZ(isa, reg, off)		(isa ? MMIPS32_BGTZ(reg, off) : MIPS32_ISA_BGTZ(reg, off))
#define MIPS32_BNE(isa, src, tar, off)		(isa ? MMIPS32_BNE(src, tar, off) : MIPS32_ISA_BNE(src, tar, off))
#define MIPS32_CACHE(isa, op, off, base)	(isa ? MMIPS32_CACHE(op, off, base) : MIPS32_ISA_CACHE(op, off, base))

#define MIPS32_J(isa, tar)			(isa ? MMIPS32_J(tar) : MIPS32_ISA_J(tar))
#define MIPS32_JR(isa, reg)			(isa ? MMIPS32_JR(reg) : MIPS32_ISA_JR(reg))
#define MIPS32_LB(isa, reg, off, base)		(isa ? MMIPS32_LB(reg, off, base) : MIPS32_ISA_LB(reg, off, base))
#define MIPS32_LBU(isa, reg, off, base)		(isa ? MMIPS32_LBU(reg, off, base) : MIPS32_ISA_LBU(reg, off, base))
#define MIPS32_LHU(isa, reg, off, base)		(isa ? MMIPS32_LHU(reg, off, base) : MIPS32_ISA_LHU(reg, off, base))
#define MIPS32_LW(isa, reg, off, base)		(isa ? MMIPS32_LW(reg, off, base) : MIPS32_ISA_LW(reg, off, base))
#define MIPS32_LUI(isa, reg, val)		(isa ? MMIPS32_LUI(reg, val) : MIPS32_ISA_LUI(reg, val))

#define MIPS32_MFC0(isa, gpr, cpr, sel)		(isa ? MMIPS32_MFC0(gpr, cpr, sel) : MIPS32_ISA_MFC0(gpr, cpr, sel))
#define MIPS32_MTC0(isa, gpr, cpr, sel)		(isa ? MMIPS32_MTC0(gpr, cpr, sel) : MIPS32_ISA_MTC0(gpr, cpr, sel))
#define MIPS32_MFLO(isa, reg)			(isa ? MMIPS32_MFLO(reg) : MIPS32_ISA_MFLO(reg))
#define MIPS32_MFHI(isa, reg)			(isa ? MMIPS32_MFHI(reg) : MIPS32_ISA_MFHI(reg))
#define MIPS32_MTLO(isa, reg)			(isa ? MMIPS32_MTLO(reg) : MIPS32_ISA_MTLO(reg))
#define MIPS32_MTHI(isa, reg)			(isa ? MMIPS32_MTHI(reg) : MIPS32_ISA_MTHI(reg))

#define MIPS32_MOVN(isa, dst, src, tar)		(isa ? MMIPS32_MOVN(dst, src, tar) : MIPS32_ISA_MOVN(dst, src, tar))
#define MIPS32_ORI(isa, tar, src, val)		(isa ? MMIPS32_ORI(tar, src, val) : MIPS32_ISA_ORI(tar, src, val))
#define MIPS32_RDHWR(isa, tar, dst)		(isa ? MMIPS32_RDHWR(tar, dst) : MIPS32_ISA_RDHWR(tar, dst))
#define MIPS32_SB(isa, reg, off, base)		(isa ? MMIPS32_SB(reg, off, base) : MIPS32_ISA_SB(reg, off, base))
#define MIPS32_SH(isa, reg, off, base)		(isa ? MMIPS32_SH(reg, off, base) : MIPS32_ISA_SH(reg, off, base))
#define MIPS32_SW(isa, reg, off, base)		(isa ? MMIPS32_SW(reg, off, base) : MIPS32_ISA_SW(reg, off, base))

#define MIPS32_SLL(isa, dst, src, sa)		(isa ? MMIPS32_SLL(dst, src, sa) : MIPS32_ISA_SLL(dst, src, sa))
#define MIPS32_SLTI(isa, tar, src, val)		(isa ? MMIPS32_SLTI(tar, src, val) : MIPS32_ISA_SLTI(tar, src, val))
#define MIPS32_SLTU(isa, dst, src, tar)		(isa ? MMIPS32_SLTU(dst, src, tar) : MIPS32_ISA_SLTU(dst, src, tar))
#define MIPS32_SRL(isa, reg, src, off)		(isa ? MMIPS32_SRL(reg, src, off) : MIPS32_ISA_SRL(reg, src, off))

#define MIPS32_SYNCI(isa, off, base)		(isa ? MMIPS32_SYNCI(off, base) : MIPS32_ISA_SYNCI(off, base))
#define MIPS32_SYNC(isa)			(isa ? MMIPS32_SYNC : MIPS32_ISA_SYNC)
#define MIPS32_XOR(isa, reg, val1, val2)	(isa ? MMIPS32_XOR(reg, val1, val2) : MIPS32_ISA_XOR(reg, val1, val2))
#define MIPS32_XORI(isa, tar, src, val)		(isa ? MMIPS32_XORI(tar, src, val) : MIPS32_ISA_XORI(tar, src, val))

#define MIPS32_SYNCI_STEP			0x1

/* ejtag specific instructions */
#define MIPS32_DRET(isa)			(isa ? MMIPS32_DRET : MIPS32_ISA_DRET)
#define MIPS32_SDBBP(isa)			(isa ? MMIPS32_SDBBP : MIPS32_ISA_SDBBP)

#define MIPS16_SDBBP(isa)			(isa ? MMIPS16_SDBBP : MIPS16_ISA_SDBBP)

extern const struct command_registration mips32_command_handlers[];

int mips32_arch_state(struct target *target);

int mips32_init_arch_info(struct target *target,
		struct mips32_common *mips32, struct jtag_tap *tap);

int mips32_restore_context(struct target *target);
int mips32_save_context(struct target *target);

struct reg_cache *mips32_build_reg_cache(struct target *target);

int mips32_run_algorithm(struct target *target,
		int num_mem_params, struct mem_param *mem_params,
		int num_reg_params, struct reg_param *reg_params,
		target_addr_t entry_point, target_addr_t exit_point,
		int timeout_ms, void *arch_info);

int mips32_configure_break_unit(struct target *target);

int mips32_enable_interrupts(struct target *target, int enable);

int mips32_examine(struct target *target);

int mips32_read_config_regs(struct target *target);

int mips32_register_commands(struct command_context *cmd_ctx);

int mips32_get_gdb_reg_list(struct target *target,
		struct reg **reg_list[], int *reg_list_size,
		enum target_register_class reg_class);
int mips32_checksum_memory(struct target *target, target_addr_t address,
		uint32_t count, uint32_t *checksum);
int mips32_blank_check_memory(struct target *target,
		target_addr_t address, uint32_t count, uint32_t *blank, uint8_t erased_value);

#endif /* OPENOCD_TARGET_MIPS32_H */

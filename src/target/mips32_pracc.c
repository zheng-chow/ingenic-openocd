/***************************************************************************
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by David T.L. Wong                                 *
 *                                                                         *
 *   Copyright (C) 2009 by David N. Claffey <dnclaffey@gmail.com>          *
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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

/*
 * This version has optimized assembly routines for 32 bit operations:
 * - read word
 * - write word
 * - write array of words
 *
 * One thing to be aware of is that the MIPS32 cpu will execute the
 * instruction after a branch instruction (one delay slot).
 *
 * For example:
 *  LW $2, ($5 +10)
 *  B foo
 *  LW $1, ($2 +100)
 *
 * The LW $1, ($2 +100) instruction is also executed. If this is
 * not wanted a NOP can be inserted:
 *
 *  LW $2, ($5 +10)
 *  B foo
 *  NOP
 *  LW $1, ($2 +100)
 *
 * or the code can be changed to:
 *
 *  B foo
 *  LW $2, ($5 +10)
 *  LW $1, ($2 +100)
 *
 * The original code contained NOPs. I have removed these and moved
 * the branches.
 *
 * These changes result in a 35% speed increase when programming an
 * external flash.
 *
 * More improvement could be gained if the registers do no need
 * to be preserved but in that case the routines should be aware
 * OpenOCD is used as a flash programmer or as a debug tool.
 *
 * Nico Coesel
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/time_support.h>

#include "mips32.h"
#include "mips32_pracc.h"

struct mips32_pracc_context {
	uint32_t *local_oparam;
	int num_oparam;
	const uint32_t *code;
	int code_len;
	uint32_t stack[32];
	int stack_offset;
	struct mips_ejtag *ejtag_info;
};

static int wait_for_pracc_rw(struct mips_ejtag *ejtag_info, uint32_t *ctrl)
{
	uint32_t ejtag_ctrl;
	long long then = timeval_ms();

	/* wait for the PrAcc to become "1" */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);

	while (1) {
		ejtag_ctrl = ejtag_info->ejtag_ctrl;
		int retval = mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
		if (retval != ERROR_OK) {
			LOG_DEBUG("mips_ejtag_drscan_32 Failed");
			return retval;
		}

		if (ejtag_ctrl & EJTAG_CTRL_PRACC)
			break;

		int timeout = timeval_ms() - then;
		if (timeout > 1000) {
			LOG_DEBUG("Timeout: No memory access in progress!");
			return ERROR_JTAG_DEVICE_ERROR;
		}
	}

	*ctrl = ejtag_ctrl;
	return ERROR_OK;
}

static int try_wait_for_pracc_rw(struct mips_ejtag *ejtag_info, uint32_t *ctrl)
{
    uint32_t ejtag_ctrl;

    /* wait for the PrAcc to become "1" */
    mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);

    ejtag_ctrl = ejtag_info->ejtag_ctrl;
    int retval = mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
    if (retval != ERROR_OK) {
        LOG_DEBUG("mips_ejtag_drscan_32 Failed");
        return retval;
    }

    *ctrl = ejtag_ctrl;
    return ERROR_OK;
}

static int mips32_pracc_try_read_ctrl_addr(struct mips_ejtag *ejtag_info)
{
    int retval = try_wait_for_pracc_rw(ejtag_info, &ejtag_info->pa_ctrl);
    if (retval != ERROR_OK) {
        LOG_DEBUG("try_wait_for_pracc_rw failed");
        return retval;
    }

    mips_ejtag_set_instr(ejtag_info, EJTAG_INST_ADDRESS);
    ejtag_info->pa_addr = 0;
    retval = mips_ejtag_drscan_32(ejtag_info, &ejtag_info->pa_addr);

    return retval;
}

/* Shift in control and address for a new processor access, save them in ejtag_info */
static int mips32_pracc_read_ctrl_addr(struct mips_ejtag *ejtag_info)
{
	int retval = wait_for_pracc_rw(ejtag_info, &ejtag_info->pa_ctrl);
	if (retval != ERROR_OK) {
		LOG_DEBUG("wait_for_pracc_rw failed");
		return retval;
	}

	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_ADDRESS);
	ejtag_info->pa_addr = 0;
	retval = mips_ejtag_drscan_32(ejtag_info, &ejtag_info->pa_addr);

	return retval;
}

/* Finish processor access */
static int mips32_pracc_finish(struct mips_ejtag *ejtag_info)
{
	uint32_t ctrl = ejtag_info->ejtag_ctrl & ~EJTAG_CTRL_PRACC;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
	mips_ejtag_drscan_32_out(ejtag_info, ctrl);

	return jtag_execute_queue();
}

int mips32_pracc_clean_text_jump(struct mips_ejtag *ejtag_info)
{
	uint32_t jt_code = MIPS32_J((0x0FFFFFFF & MIPS32_PRACC_TEXT) >> 2);
	int retval;

	/* do 3 0/nops to clean pipeline before a jump to pracc text, NOP in delay slot */
	for (int i = 0; i != 5; i++) {
		/* Wait for pracc */
		retval = wait_for_pracc_rw(ejtag_info, &ejtag_info->pa_ctrl);
		if (retval != ERROR_OK)
			return retval;

		/* Data or instruction out */
		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
		uint32_t data = (i == 3) ? jt_code : MIPS32_NOP;
		mips_ejtag_drscan_32_out(ejtag_info, data);

		/* finish pa */
		retval = mips32_pracc_finish(ejtag_info);
		if (retval != ERROR_OK)
			return retval;
	}

	if (ejtag_info->mode != 0)	/* done, queued mode won't work with lexra cores */
		return ERROR_OK;

	retval = mips32_pracc_read_ctrl_addr(ejtag_info);
	if (retval != ERROR_OK)
		return retval;

	if (ejtag_info->pa_addr != MIPS32_PRACC_TEXT) {			/* LEXRA/BMIPS ?, shift out another NOP */
//		LOG_INFO ("Clean text - LEXRA/BMIPS");
		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
		mips_ejtag_drscan_32_out(ejtag_info, MIPS32_NOP);
		retval = mips32_pracc_finish(ejtag_info);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

int mips32_pracc_exec(struct mips_ejtag *ejtag_info, struct pracc_queue_info *ctx, uint32_t *param_out)
{
	int code_count = 0;
	uint32_t abandoned_count = 0;
	int store_pending = 0;		/* increases with every store instruction at dmseg, decreases with every store pa */
	uint32_t max_store_addr = 0;	/* for store pa address testing */
	uint32_t instr = 0;
	bool final_check = 0;		/* set to 1 if in final checks after function code shifted out */
	int index;
	uint32_t data = 0;
	uint32_t wait_dret_cnt = 0;

	while (1) {
		(void)mips32_pracc_read_ctrl_addr(ejtag_info);		/* update current pa info: control and address */
		if (ejtag_info->pa_ctrl & EJTAG_CTRL_PRNW) {						/* write/store access */
			/* Check for pending store from a previous store instruction at dmseg */
			if (store_pending == 0) {
				LOG_DEBUG("unexpected write at address %" PRIx32, ejtag_info->pa_addr);
				return ERROR_JTAG_DEVICE_ERROR;
			} else if (ejtag_info->pa_addr < MIPS32_PRACC_PARAM_OUT || ejtag_info->pa_addr > max_store_addr) {
				LOG_DEBUG("writing at unexpected address %" PRIx32, ejtag_info->pa_addr);
				return ERROR_JTAG_DEVICE_ERROR;
			}
			mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
			(void)mips_ejtag_drscan_32(ejtag_info, &data);
			/* store data at param out, address based offset */
			param_out[(ejtag_info->pa_addr - MIPS32_PRACC_PARAM_OUT) / 4] = data;
			store_pending--;
		} else {					/* read/fetch access */
			if ((code_count != 0) && (ejtag_info->pa_addr == MIPS32_PRACC_TEXT) && (final_check == 0)) {
				final_check = 1;
				code_count = 0;
			}
			if (!final_check) {			/* executing function code */
				index = (ejtag_info->pa_addr - MIPS32_PRACC_TEXT) / 4;
	                        if ((code_count == 0) && (ejtag_info->pa_addr != MIPS32_PRACC_TEXT)) {
        	                        LOG_DEBUG("reading at unexpected address 0x%08x, expected %x", ejtag_info->pa_addr, MIPS32_PRACC_TEXT);
					return ERROR_JTAG_DEVICE_ERROR;
                        	}
				if (index < ctx->code_count) {
					instr = ctx->pracc_list[index];
					/* check for store instruction at dmseg */
                                	uint32_t store_addr = ctx->pracc_list[ctx->max_code + index];
                                	if (store_addr != 0) {
                                        	if (store_addr > max_store_addr)
                                                	max_store_addr = store_addr;
                                        	store_pending++;
                                	}
				} else {/*for fix IFU prefetch*/
					instr = MIPS32_NOP;
					abandoned_count++;
				}
				code_count++;
				/*if (code_count > PRACC_MAX_EXEC_CODE_COUNT) {
					LOG_DEBUG("max exec code count is %d", PRACC_MAX_EXEC_CODE_COUNT);
					return ERROR_JTAG_DEVICE_ERROR;
				}*/
			} else {/* final check after function code shifted out */
				if (store_pending == 0) {
					if (ejtag_info->pa_addr != MIPS32_PRACC_TEXT) {
                        			instr = MIPS32_B(NEG16(code_count + 1));
						do {
                                        		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
                                        		mips_ejtag_drscan_32_out(ejtag_info, instr);
                                       			(void)mips32_pracc_finish(ejtag_info);
                                			instr = MIPS32_NOP;
							(void)mips32_pracc_read_ctrl_addr(ejtag_info);
                                		} while(ejtag_info->pa_addr != MIPS32_PRACC_TEXT);
                        		}
					return ERROR_OK;
				} else { // for fix LSU store delay
					instr = MIPS32_NOP;
					abandoned_count++;
					code_count++;
				}
			}
			if (abandoned_count > 256) {
                        	LOG_DEBUG("execution abandoned, store pending: %d", store_pending);
                		return ERROR_JTAG_DEVICE_ERROR;
                	}
			/* Send instruction out */
			mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
			mips_ejtag_drscan_32_out(ejtag_info, instr);
		}
		/* finish processor access, let the processor eat! */
		(void)mips32_pracc_finish(ejtag_info);

		if (instr == MIPS32_DRET) {/* after leaving debug mode and make sure the DRET finish */
            		while(1) {
				(void)mips32_pracc_try_read_ctrl_addr(ejtag_info);/* update current pa info: control and address */
				if (((ejtag_info->pa_ctrl & EJTAG_CTRL_BRKST) == 0) ||
            		            ((ejtag_info->pa_ctrl & EJTAG_CTRL_PRACC) && (ejtag_info->pa_addr == MIPS32_PRACC_TEXT))) {
					return ERROR_OK;
				} else if ((ejtag_info->pa_addr != MIPS32_PRACC_TEXT) && (ejtag_info->pa_ctrl & EJTAG_CTRL_PRACC)) {
            		        	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
            		        	mips_ejtag_drscan_32_out(ejtag_info, MIPS32_NOP);
            		        	(void)mips32_pracc_finish(ejtag_info);
				}
            		        wait_dret_cnt++;
            		        if (wait_dret_cnt > 64) {
					LOG_DEBUG("mips32_pracc_finish failed");
            		            	return ERROR_FAIL;
            		        }
            		}
		}
	}

	return ERROR_FAIL;
}

inline void pracc_queue_init(struct pracc_queue_info *ctx)
{
	ctx->retval = ERROR_OK;
	ctx->code_count = 0;
	ctx->store_count = 0;

	ctx->pracc_list = malloc(2 * ctx->max_code * sizeof(uint32_t));
	if (ctx->pracc_list == NULL) {
		LOG_ERROR("Out of memory");
		ctx->retval = ERROR_FAIL;
	}

	ctx->expected_list = malloc(2 * ctx->max_code * sizeof(uint32_t));
	if (ctx->pracc_list == NULL) {
		LOG_ERROR("Out of memory");
		ctx->retval = ERROR_FAIL;
	}
}

unsigned int dump = 0;
inline void pracc_add(struct pracc_queue_info *ctx, uint32_t addr, uint32_t instr)
{
	ctx->pracc_list[ctx->max_code + ctx->code_count] = addr;
	ctx->pracc_list[ctx->code_count] = instr;

	ctx->expected_list[ctx->code_count] = ctx->code_count;

	if (dump == 1)
		LOG_DEBUG("addr:0x%8.8x, data:0x%8.8x, num:%d", addr, instr, ctx->code_count);

	ctx->code_count++;
	
	if (addr)
		ctx->store_count++;
}

inline void pracc_queue_free(struct pracc_queue_info *ctx)
{
	//if (ctx->code_count > ctx->max_code)	/* Only for internal check, will be erased */
	//	LOG_ERROR("Internal error, code count: %d > max code: %d", ctx->code_count, ctx->max_code);
	if (ctx->pracc_list != NULL)
		free(ctx->pracc_list);
	if (ctx->expected_list != NULL)
		free(ctx->expected_list);
}

int mips32_pracc_queue_exec(struct mips_ejtag *ejtag_info, struct pracc_queue_info *ctx, uint32_t *buf)
{
        if (ejtag_info->mode == 0) {
                return mips32_pracc_exec(ejtag_info, ctx, buf);
        }

	union scan_in {
		uint8_t scan_96[12];
		struct {
			uint8_t ctrl[4];
			uint8_t data[4];
			uint8_t addr[4];
		} scan_32;

	} *scan_in = malloc(sizeof(union scan_in) * (ctx->code_count + ctx->store_count));
	if (scan_in == NULL) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	unsigned num_clocks = ((uint64_t)(ejtag_info->scan_delay) * jtag_get_speed_khz() + 500000) / 1000000;

	uint32_t ejtag_ctrl = ejtag_info->ejtag_ctrl & ~EJTAG_CTRL_PRACC;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_ALL);

	int scan_count = 0;
	for (int i = 0; i != 2 * ctx->code_count; i++) {
		uint32_t data = 0;
		if (i & 1u) {			/* Check store address from previous instruction, if not the first */
			if (i < 2 || 0 == ctx->pracc_list[ctx->max_code + (i / 2) - 1])
				continue;
		} else
			data = ctx->pracc_list[i / 2];

		jtag_add_clocks(num_clocks);
		mips_ejtag_add_scan_96(ejtag_info, ejtag_ctrl, data, scan_in[scan_count++].scan_96);
	}

	int retval = jtag_execute_queue();		/* execute queued scans */
	if (retval != ERROR_OK) {
		LOG_INFO ("jtag_execute_queue");
		goto exit;
	}

	uint32_t fetch_addr = MIPS32_PRACC_TEXT;		/* start address */
	scan_count = 0;
	for (int i = 0; i != 2 * ctx->code_count; i++) {				/* verify every pracc access */
		uint32_t store_addr = 0;
		if (i & 1u) {			/* Read store addres from previous instruction, if not the first */
			store_addr = ctx->pracc_list[ctx->max_code + (i / 2) - 1];
			if (i < 2 || 0 == store_addr)
				continue;
		}

		ejtag_ctrl = buf_get_u32(scan_in[scan_count].scan_32.ctrl, 0, 32);
		if (!(ejtag_ctrl & EJTAG_CTRL_PRACC)) {
			LOG_ERROR("Error: access not pending count: %d", scan_count);
			LOG_ERROR("Ejtag control: 0x%x -- store_addr: 0x%x",ejtag_ctrl, store_addr);
			retval = ERROR_FAIL;
			goto exit;
		}

		uint32_t addr = buf_get_u32(scan_in[scan_count].scan_32.addr, 0, 32);
                
		if (store_addr != 0) {
			if (!(ejtag_ctrl & EJTAG_CTRL_PRNW)) {
				LOG_ERROR("Not a store/write access(addr:%x), count: %d", addr, scan_count);
				retval = ERROR_FAIL;
				goto exit;
			}
			if (addr != store_addr) {
				LOG_ERROR("Store address mismatch, read: %" PRIx32 " expected: %" PRIx32 " count: %d",
						addr, store_addr, scan_count);
				retval = ERROR_FAIL;
				goto exit;
			}
			int buf_index = (addr - MIPS32_PRACC_PARAM_OUT) / 4;
			buf[buf_index] = buf_get_u32(scan_in[scan_count].scan_32.data, 0, 32);

		} else {
			if (ejtag_ctrl & EJTAG_CTRL_PRNW) {
				LOG_ERROR("Not a fetch/read access, count: %d addr: %x", scan_count, addr);
				retval = ERROR_FAIL;
				goto exit;
			}
			if (addr != fetch_addr) {
				LOG_ERROR("Fetch addr mismatch, read: %" PRIx32 " expected: %" PRIx32 " count: %d",
					  addr, fetch_addr, scan_count);
				retval = ERROR_FAIL;
				goto exit;
			}
			fetch_addr += 4;
		}
		scan_count++;
	}
exit:
	free(scan_in);

	if (retval != ERROR_OK) {
		int tmp_retval = mips32_pracc_clean_text_jump(ejtag_info);
		if (tmp_retval != ERROR_OK) {
			LOG_DEBUG("mips32_pracc_clean_text_jump failed");
		}
	}

	return retval;
}

int mips32_pracc_read_u32(struct mips_ejtag *ejtag_info, uint32_t addr, uint32_t *buf)
{
	struct pracc_queue_info ctx = {.max_code = 9};
	pracc_queue_init(&ctx);
	if (ctx.retval != ERROR_OK)
		goto exit;

	pracc_add(&ctx, 0, MIPS32_LUI(15, PRACC_UPPER_BASE_ADDR));			/* $15 = MIPS32_PRACC_BASE_ADDR */
	pracc_add(&ctx, 0, MIPS32_LUI(8, UPPER16((addr + 0x8000))));		/* load  $8 with modified upper address */
	pracc_add(&ctx, 0, MIPS32_LW(8, LOWER16(addr), 8));				/* lw $8, LOWER16(addr)($8) */
	pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT,
				MIPS32_SW(8, PRACC_OUT_OFFSET, 15));			/* sw $8,PRACC_OUT_OFFSET($15) */
	pracc_add(&ctx, 0, MIPS32_LUI(8, UPPER16(ejtag_info->reg8)));		/* restore upper 16 of $8 */
	pracc_add(&ctx, 0, MIPS32_ORI(8, 8, LOWER16(ejtag_info->reg8)));		/* restore lower 16 of $8 */

	pracc_add(&ctx, 0, MIPS32_SYNC);
	pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));					/* jump to start */
	pracc_add(&ctx, 0, MIPS32_MFC0(15, 31, 0));					/* move COP0 DeSave to $15 */

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, buf);

exit:
	pracc_queue_free(&ctx);
	return ctx.retval;
}

int mips32_pracc_read_mem(struct mips_ejtag *ejtag_info, uint32_t addr, int size, int count, void *buf, int cputype)
{
	if (count == 1 && size == 4)
		return mips32_pracc_read_u32(ejtag_info, addr, (uint32_t *)buf);

	uint32_t *data = NULL;
	struct pracc_queue_info ctx = {.max_code = 256 * 3 + 9 + 1};	/* alloc memory for the worst case */

	pracc_queue_init(&ctx);
	if (ctx.retval != ERROR_OK)
		goto exit;

	if (size != 4) {
		data = malloc(256 * sizeof(uint32_t));
		if (data == NULL) {
			LOG_ERROR("Out of memory");
			goto exit;
		}
	}

	uint32_t *buf32 = buf;
	uint16_t *buf16 = buf;
	uint8_t *buf8 = buf;

	while (count) {
		ctx.code_count = 0;
		ctx.store_count = 0;
		int this_round_count = (count > 256) ? 256 : count;
		uint32_t last_upper_base_addr = UPPER16((addr + 0x8000));

		pracc_add(&ctx, 0, MIPS32_LUI(15, PRACC_UPPER_BASE_ADDR));			/* $15 = MIPS32_PRACC_BASE_ADDR */
		pracc_add(&ctx, 0, MIPS32_LUI(9, last_upper_base_addr));		/* load the upper memory address in $9 */

		for (int i = 0; i != this_round_count; i++) {			/* Main code loop */
			uint32_t upper_base_addr = UPPER16((addr + 0x8000));
			if (last_upper_base_addr != upper_base_addr) {			/* if needed, change upper address in $9 */
				pracc_add(&ctx, 0, MIPS32_LUI(9, upper_base_addr));
				last_upper_base_addr = upper_base_addr;
			}

			if (size == 4)
				pracc_add(&ctx, 0, MIPS32_LW(8, LOWER16(addr), 9));		/* load from memory to $8 */
			else if (size == 2) {
				pracc_add(&ctx, 0, MIPS32_LHU(8, LOWER16(addr), 9));
//				LOG_INFO ("LH addr: %x size: %x cputype: 0x%x", addr, size, cputype);
				if (cputype == MIPS_CP0_mAPTIV_uP)
					pracc_add(&ctx, 0, MIPS32_NOP);						/* nop - Added to allow read to complete before store. Possible hardware bug */
			}
			else {
				pracc_add(&ctx, 0, MIPS32_LBU(8, LOWER16(addr), 9));
//				LOG_INFO ("LB addr: %x size: %x cputype: 0x%x", addr, size, cputype);
				if (cputype == MIPS_CP0_mAPTIV_uP)
					pracc_add(&ctx, 0, MIPS32_NOP);						/* nop - Added to allow read to complete before store. Possible hardware bug */
			}

			pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + i * 4,
					  MIPS32_SW(8, PRACC_OUT_OFFSET + i * 4, 15));		/* store $8 at param out */
			addr += size;
		}
		pracc_add(&ctx, 0, MIPS32_LUI(8, UPPER16(ejtag_info->reg8)));		/* restore upper 16 bits of reg 8 */
		pracc_add(&ctx, 0, MIPS32_ORI(8, 8, LOWER16(ejtag_info->reg8)));	/* restore lower 16 bits of reg 8 */
		pracc_add(&ctx, 0, MIPS32_LUI(9, UPPER16(ejtag_info->reg9)));		/* restore upper 16 bits of reg 9 */
		pracc_add(&ctx, 0, MIPS32_ORI(9, 9, LOWER16(ejtag_info->reg9)));	/* restore lower 16 bits of reg 9 */
		pracc_add(&ctx, 0, MIPS32_SYNC);
		pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));				/* jump to start */
		pracc_add(&ctx, 0, MIPS32_MFC0(15, 31, 0));					/* restore $15 from DeSave */

		if (size == 4) {
			ctx.retval = mips32_pracc_exec(ejtag_info, &ctx, buf32);
			if (ctx.retval != ERROR_OK){
				LOG_DEBUG("mips32_pracc_exec failed");
				goto exit;
			}
			buf32 += this_round_count;
		} else {
			ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, data);

			if (ctx.retval != ERROR_OK){
				LOG_DEBUG("mips32_pracc_queue_exec failed");
				goto exit;
			}

			uint32_t *data_p = data;
			for (int i = 0; i != this_round_count; i++) {
				if (size == 2)
					*buf16++ = *data_p++;
				else
					*buf8++ = *data_p++;
			}
		}
		count -= this_round_count;
	}
exit:
	pracc_queue_free(&ctx);
	if (data != NULL)
		free(data);
	return ctx.retval;
}

int mips32_pracc_cp0_read(struct mips_ejtag *ejtag_info, uint32_t *val, uint32_t cp0_reg, uint32_t cp0_sel)
{
	struct pracc_queue_info ctx = {.max_code = 8};

	pracc_queue_init(&ctx);
	if (ctx.retval != ERROR_OK)
		goto exit;
	pracc_add(&ctx, 0, MIPS32_LUI(15, PRACC_UPPER_BASE_ADDR));			/* $15 = MIPS32_PRACC_BASE_ADDR */
	pracc_add(&ctx, 0, MIPS32_MFC0(8, 0, 0) | (cp0_reg << 11) | cp0_sel);	/* move COP0 [cp0_reg select] to $8 */
	pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT,
				MIPS32_SW(8, PRACC_OUT_OFFSET, 15));			/* store $8 to pracc_out */
	pracc_add(&ctx, 0, MIPS32_MFC0(15, 31, 0));					/* move COP0 DeSave to $15 */
	pracc_add(&ctx, 0, MIPS32_LUI(8, UPPER16(ejtag_info->reg8)));		/* restore upper 16 bits  of $8 */
	pracc_add(&ctx, 0, MIPS32_SYNC);
	pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));					/* jump to start */
	pracc_add(&ctx, 0, MIPS32_ORI(8, 8, LOWER16(ejtag_info->reg8)));		/* restore lower 16 bits of $8 */

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, val);
exit:
	pracc_queue_free(&ctx);
	return ctx.retval;

	/**
	 * Note that our input parametes cp0_reg and cp0_sel
	 * are numbers (not gprs) which make part of mfc0 instruction opcode.
	 *
	 * These are not fix, but can be different for each mips32_cp0_read() function call,
	 * and that is why we must insert them directly into opcode,
	 * i.e. we can not pass it on EJTAG microprogram stack (via param_in),
	 * and put them into the gprs later from MIPS32_PRACC_STACK
	 * because mfc0 do not use gpr as a parameter for the cp0_reg and select part,
	 * but plain (immediate) number.
	 *
	 * MIPS32_MTC0 is implemented via MIPS32_R_INST macro.
	 * In order to insert our parameters, we must change rd and funct fields.
	 *
	 * code[2] |= (cp0_reg << 11) | cp0_sel;   change rd and funct of MIPS32_R_INST macro
	 **/
}

int mips32_pracc_cp0_write(struct mips_ejtag *ejtag_info, uint32_t val, uint32_t cp0_reg, uint32_t cp0_sel)
{
	struct pracc_queue_info ctx = {.max_code = 6};

	pracc_queue_init(&ctx);
	if (ctx.retval != ERROR_OK)
		goto exit;

	pracc_add(&ctx, 0, MIPS32_LUI(15, UPPER16(val)));				/* Load val to $15 */
	pracc_add(&ctx, 0, MIPS32_ORI(15, 15, LOWER16(val)));

	pracc_add(&ctx, 0, MIPS32_MTC0(15, 0, 0) | (cp0_reg << 11) | cp0_sel);	/* write cp0 reg / sel */

	pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));					/* jump to start */
	pracc_add(&ctx, 0, MIPS32_MFC0(15, 31, 0));					/* move COP0 DeSave to $15 */

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL);
exit:
	pracc_queue_free(&ctx);
	return ctx.retval;

	/**
	 * Note that MIPS32_MTC0 macro is implemented via MIPS32_R_INST macro.
	 * In order to insert our parameters, we must change rd and funct fields.
	 * code[3] |= (cp0_reg << 11) | cp0_sel;   change rd and funct fields of MIPS32_R_INST macro
	 **/
}

/**
 * \b mips32_pracc_sync_cache
 *
 * Synchronize Caches to Make Instruction Writes Effective
 * (ref. doc. MIPS32 Architecture For Programmers Volume II: The MIPS32 Instruction Set,
 *  Document Number: MD00086, Revision 2.00, June 9, 2003)
 *
 * When the instruction stream is written, the SYNCI instruction should be used
 * in conjunction with other instructions to make the newly-written instructions effective.
 *
 * Explanation :
 * A program that loads another program into memory is actually writing the D- side cache.
 * The instructions it has loaded can't be executed until they reach the I-cache.
 *
 * After the instructions have been written, the loader should arrange
 * to write back any containing D-cache line and invalidate any locations
 * already in the I-cache.
 *
 * If the cache coherency attribute (CCA) is set to zero, it's a write through cache, there is no need
 * to write back.
 *
 * In the latest MIPS32/64 CPUs, MIPS provides the synci instruction,
 * which does the whole job for a cache-line-sized chunk of the memory you just loaded:
 * That is, it arranges a D-cache write-back (if CCA = 3) and an I-cache invalidate.
 *
 * The line size is obtained with the rdhwr SYNCI_Step in release 2 or from cp0 config 1 register in release 1.
 */
static int mips32_pracc_synchronize_cache(struct mips_ejtag *ejtag_info,
					 uint32_t start_addr, uint32_t end_addr, int cached, enum mips32_isa_version rel)
{
	struct pracc_queue_info ctx = {.max_code = 256 * 2 + 5};
	pracc_queue_init(&ctx);
	if (ctx.retval != ERROR_OK)
		goto exit;

	/** Find cache line size in bytes */
	uint32_t clsiz;
	if (rel == MIPS32_ISA_RELEASE2) {	/* Release 2 (rel = 1) */
		pracc_add(&ctx, 0, MIPS32_LUI(15, PRACC_UPPER_BASE_ADDR));			/* $15 = MIPS32_PRACC_BASE_ADDR */

		pracc_add(&ctx, 0, MIPS32_RDHWR(8, MIPS32_SYNCI_STEP));			/* load synci_step value to $8 */

		pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT,
				MIPS32_SW(8, PRACC_OUT_OFFSET, 15));			/* store $8 to pracc_out */

		pracc_add(&ctx, 0, MIPS32_LUI(8, UPPER16(ejtag_info->reg8)));			/* restore upper 16 bits  of $8 */
		pracc_add(&ctx, 0, MIPS32_ORI(8, 8, LOWER16(ejtag_info->reg8)));		/* restore lower 16 bits of $8 */
		pracc_add(&ctx, 0, MIPS32_SYNC);
		pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));					/* jump to start */
		pracc_add(&ctx, 0, MIPS32_MFC0(15, 31, 0));					/* move COP0 DeSave to $15 */

		ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, &clsiz);
		if (ctx.retval != ERROR_OK)
			goto exit;

	} else {			/* Release 1 (rel = 0) */
		uint32_t conf;
		ctx.retval = mips32_pracc_cp0_read(ejtag_info, &conf, 16, 1);
		if (ctx.retval != ERROR_OK)
			goto exit;

		uint32_t dl = (conf & MIPS32_CONFIG1_DL_MASK) >> MIPS32_CONFIG1_DL_SHIFT;

		/* dl encoding : dl=1 => 4 bytes, dl=2 => 8 bytes, etc... max dl=6 => 128 bytes cache line size */
		clsiz = 0x2 << dl;
		if (dl == 0)
			clsiz = 0;
	}

	if (clsiz == 0)
		goto exit;  /* Nothing to do */

	/* make sure clsiz is power of 2 */
	if (clsiz & (clsiz - 1)) {
		LOG_DEBUG("clsiz must be power of 2");
		ctx.retval = ERROR_FAIL;
		goto exit;
	}

	/* make sure start_addr and end_addr have the same offset inside de cache line */
	start_addr |= clsiz - 1;
	end_addr |= clsiz - 1;

	ctx.code_count = 0;
	int count = 0;
	uint32_t last_upper_base_addr = UPPER16((start_addr + 0x8000));

	pracc_add(&ctx, 0, MIPS32_LUI(15, last_upper_base_addr));		/* load upper memory base address to $15 */

	while (start_addr <= end_addr) {						/* main loop */
		uint32_t upper_base_addr = UPPER16((start_addr + 0x8000));
		if (last_upper_base_addr != upper_base_addr) {				/* if needed, change upper address in $15 */
			pracc_add(&ctx, 0, MIPS32_LUI(15, upper_base_addr));
			last_upper_base_addr = upper_base_addr;
		}
		if ((rel == MIPS32_ISA_RELEASE2) || (rel == MIPS32_ISA_RELEASE2_INGENIC_FORBID_RDHWR))
			pracc_add(&ctx, 0, MIPS32_SYNCI(LOWER16(start_addr), 15));		/* synci instruction, offset($15) */

		else {
			if (cached == 3)
				pracc_add(&ctx, 0, MIPS32_CACHE(MIPS32_CACHE_D_HIT_WRITEBACK,
							LOWER16(start_addr), 15));		/* cache Hit_Writeback_D, offset($15) */

			pracc_add(&ctx, 0, MIPS32_CACHE(MIPS32_CACHE_I_HIT_INVALIDATE,
							LOWER16(start_addr), 15));		/* cache Hit_Invalidate_I, offset($15) */
		}
		start_addr += clsiz;
		count++;
		if (count == 256 && start_addr <= end_addr) {				/* more ?, then execute code list */
			pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));		/* jump to start */
			pracc_add(&ctx, 0, MIPS32_NOP);						/* nop in delay slot */

			ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL);
			if (ctx.retval != ERROR_OK)
				goto exit;

			ctx.code_count = 0;
			count = 0;
		}
	}
	pracc_add(&ctx, 0, MIPS32_SYNC);
	pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));					/* jump to start */
	pracc_add(&ctx, 0, MIPS32_MFC0(15, 31, 0));					/* restore $15 from DeSave*/

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL);
exit:
	pracc_queue_free(&ctx);
	return ctx.retval;
}

static int mips32_pracc_write_mem_generic(struct mips_ejtag *ejtag_info,
		uint32_t addr, int size, int count, const void *buf)
{
	struct pracc_queue_info ctx = {.max_code = 128 * 3 + 5 + 1};	/* alloc memory for the worst case */

	pracc_queue_init(&ctx);
	if (ctx.retval != ERROR_OK)
		goto exit;

	const uint32_t *buf32 = buf;
	const uint16_t *buf16 = buf;
	const uint8_t *buf8 = buf;

	while (count) {
		ctx.code_count = 0;
		ctx.store_count = 0;
		int this_round_count = (count > 128) ? 128 : count;
		uint32_t last_upper_base_addr = UPPER16((addr + 0x8000));

		pracc_add(&ctx, 0, MIPS32_LUI(15, last_upper_base_addr));		/* load $15 with memory base address */

		for (int i = 0; i != this_round_count; i++) {
			uint32_t upper_base_addr = UPPER16((addr + 0x8000));
			if (last_upper_base_addr != upper_base_addr) {
				pracc_add(&ctx, 0, MIPS32_LUI(15, upper_base_addr));	/* if needed, change upper address in $15*/
				last_upper_base_addr = upper_base_addr;
			}

			if (size == 4) {			/* for word writes check if one half word is 0 and load it accordingly */
				if (LOWER16(*buf32) == 0)
					pracc_add(&ctx, 0, MIPS32_LUI(8, UPPER16(*buf32)));		/* load only upper value */
				else if (UPPER16(*buf32) == 0)
						pracc_add(&ctx, 0, MIPS32_ORI(8, 0, LOWER16(*buf32)));	/* load only lower */
				else {
					pracc_add(&ctx, 0, MIPS32_LUI(8, UPPER16(*buf32)));		/* load upper and lower */
					pracc_add(&ctx, 0, MIPS32_ORI(8, 8, LOWER16(*buf32)));
				}

				pracc_add(&ctx, 0, MIPS32_SW(8, LOWER16(addr), 15));		/* store word to memory */
				buf32++;

			} else if (size == 2) {
				pracc_add(&ctx, 0, MIPS32_ORI(8, 0, *buf16));		/* load lower value */
				pracc_add(&ctx, 0, MIPS32_SH(8, LOWER16(addr), 15));	/* store half word to memory */
				buf16++;

			} else {
				pracc_add(&ctx, 0, MIPS32_ORI(8, 0, *buf8));		/* load lower value */
				pracc_add(&ctx, 0, MIPS32_SB(8, LOWER16(addr), 15));	/* store byte to memory */
				buf8++;
			}
			addr += size;
		}

		pracc_add(&ctx, 0, MIPS32_LUI(8, UPPER16(ejtag_info->reg8)));		/* restore upper 16 bits of reg 8 */
		pracc_add(&ctx, 0, MIPS32_ORI(8, 8, LOWER16(ejtag_info->reg8)));	/* restore lower 16 bits of reg 8 */

		pracc_add(&ctx, 0, MIPS32_SYNC);                                        /* if any DRSEG or DMSEG stores were made, */
		                                                                        /* some cores might leave one or more uncommitted without a SYNC here */
		pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));				/* jump to start */
		pracc_add(&ctx, 0, MIPS32_MFC0(15, 31, 0));				/* restore $15 from DeSave */

		ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL);
		if (ctx.retval != ERROR_OK) {
			LOG_ERROR("mips32_pracc_exec failed");
			goto exit;
		}

		count -= this_round_count;
	}
exit:
	pracc_queue_free(&ctx);
	return ctx.retval;
}

static int mips32_pracc_write_mem_series(struct mips_ejtag *ejtag_info, uint32_t addr, int size, int count, const void *buf)
{
	LOG_DEBUG("mips32_pracc_write_mem_series");
	static uint32_t CC_COPY = 13;
	uint32_t copy_to_mem[] = {
		MIPS32_LUI(15, MIPS32_PRACC_TEXT >> 16),
		MIPS32_ORI(15, 15, MIPS32_PRACC_TEXT & 0xFFFF),
		MIPS32_LUI(8, (count - CC_COPY) >> 16),
		MIPS32_ORI(8, 8, (count - CC_COPY) & 0xFFFF),
		MIPS32_LUI(9, (addr + (CC_COPY * 4)) >> 16),
		MIPS32_ORI(9, 9, (addr + (CC_COPY * 4)) & 0xFFFF),
		MIPS32_LW(10, 0, 15),
		MIPS32_SW(10, 0, 9),
		MIPS32_ADDIU(9, 9, 4),
		MIPS32_BGTZ(8, NEG16(4)),
		MIPS32_ADDIU(8, 8, -1),
		MIPS32_JR(15),
		MIPS32_NOP,
		MIPS32_NOP,
		MIPS32_NOP,
		MIPS32_NOP
        };

	(void)mips32_pracc_write_mem_generic(ejtag_info, addr, 4, CC_COPY, copy_to_mem);
	if ((KSEGX(addr) == KSEG1) || ((addr >= 0xff200000) && (addr <= 0xff3fffff)) == 0) {
		(void)mips32_pracc_synchronize_cache(ejtag_info, addr, (addr + (CC_COPY * 4)), 3, MIPS32_ISA_RELEASE2_INGENIC_FORBID_RDHWR);
	}

	struct pracc_queue_info start_ctx = {.max_code = 8};
        pracc_queue_init(&start_ctx);
        if (start_ctx.retval != ERROR_OK) {
        	pracc_queue_free(&start_ctx);
        	return start_ctx.retval;
	}
        pracc_add(&start_ctx, 0, MIPS32_LUI(15, UPPER16(addr)));
        pracc_add(&start_ctx, 0, MIPS32_ORI(15, 15, LOWER16(addr)));
        pracc_add(&start_ctx, 0, MIPS32_JR(15));
        pracc_add(&start_ctx, 0, MIPS32_NOP);
        start_ctx.retval = mips32_pracc_queue_exec(ejtag_info, &start_ctx, NULL);
        pracc_queue_free(&start_ctx);

	int code_count = 0;
	uint32_t ejtag_ctrl;
	uint32_t redo_count = 0;

	while(1) {
                mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
		uint32_t data = *((uint32_t *)buf + code_count + CC_COPY);
                mips_ejtag_drscan_32_out(ejtag_info, data);
		(void)jtag_execute_queue();

        	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
REDO:		ejtag_ctrl = ejtag_info->ejtag_ctrl & ~EJTAG_CTRL_PRACC;
        	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
		if ((ejtag_ctrl & EJTAG_CTRL_PRACC) != 0) {
			if (code_count == (count - (int)CC_COPY)) {
				break;
			}
			code_count++;
		} else {
			redo_count++;
			LOG_DEBUG("REDO");
			if (redo_count > 64) break;
			else goto REDO;
		}
	}

	(void)mips32_pracc_write_mem_generic(ejtag_info, addr, 4, CC_COPY, buf);

	return ERROR_OK;
}

int mips32_pracc_write_mem(struct mips_ejtag *ejtag_info, uint32_t addr, int size, int count, const void *buf)
{
	int retval;
	LOG_DEBUG("address: 0x%8.8" PRIx32 ", size: 0x%8.8" PRIx32 ", count: 0x%8.8" PRIx32 "",
 			addr, size, count);

	if ((count < 64) || (size != 4)) {
		retval = mips32_pracc_write_mem_generic(ejtag_info, addr, size, count, buf);
	} else if (size == 4) {
		retval = mips32_pracc_write_mem_series(ejtag_info, addr, size, count, buf);
	} else {
		LOG_DEBUG("mips32_pracc_write_mem error");
		retval = ERROR_FAIL;
	}
	if (retval != ERROR_OK) {
		return retval;
	}

	/**
	 * If we are in the cacheable region and cache is activated,
	 * we must clean D$ (if Cache Coherency Attribute is set to 3) + invalidate I$ after we did the write,
	 * so that changes do not continue to live only in D$ (if CCA = 3), but to be
	 * replicated in I$ also (maybe we wrote the istructions)
	 */
	if ((KSEGX(addr) == KSEG1) || ((addr >= 0xff200000) && (addr <= 0xff3fffff))) // TODO:The Ingenic cpu ejtag has itself accelerate mode
		return retval; /*Nothing to do*/

	int cached = 0;

	uint32_t conf = 0;
	uint32_t prid;
	uint32_t config1;
	uint32_t cpuType;

	if ((retval = mips32_pracc_cp0_read(ejtag_info, &conf, 16, 0))!= ERROR_OK)
		return retval;

	/* Read PRID registers */
	if ((retval = mips32_pracc_cp0_read(ejtag_info, &prid, 15, 0)) != ERROR_OK)
		return retval;

	/* Read Config1 registers */
	if ((retval = mips32_pracc_cp0_read(ejtag_info, &config1, 16, 1))!= ERROR_OK)
		return retval;

	switch (KSEGX(addr)) {
		case KUSEG:
			cached = (conf & MIPS32_CONFIG0_KU_MASK) >> MIPS32_CONFIG0_KU_SHIFT;
			break;
		case KSEG0:
			cached = (conf & MIPS32_CONFIG0_K0_MASK) >> MIPS32_CONFIG0_K0_SHIFT;
			break;
		case KSEG2:
		case KSEG3:
			cached = (conf & MIPS32_CONFIG0_K23_MASK) >> MIPS32_CONFIG0_K23_SHIFT;
			break;
		default:
			/* what ? */
			break;
	}

	/*
	 * Check cachablitiy bits coherency algorithm
	 * is the region cacheable or uncached.
	 * If cacheable we have to synchronize the cache
	 */

	/* Get core type */
	cpuType = DetermineCpuTypeFromPrid(prid, conf, config1);

	/* Write back cache or write through cache */
	if ((cpuType == MIPS_INTERAPTIV          ) ||
	    (cpuType == MIPS_INTERAPTIV_CM       ) ||
	    (cpuType == MIPS_INGENIC_XBURST1     ) ||
	    (cpuType == MIPS_INGENIC_XBURST2     )) {
		if ((cached == CCA_IAPTIV_CWBE) || (cached == CCA_IAPTIV_CWB) || (cached == CCA_WB)) {
			uint32_t start_addr = addr;
			uint32_t end_addr = addr + count * size;
			enum mips32_isa_version rel = (enum mips32_isa_version)((conf & MIPS32_CONFIG0_AR_MASK) >> MIPS32_CONFIG0_AR_SHIFT);
			if (rel > MIPS32_ISA_RELEASE2) {
				LOG_DEBUG("Unknown release in cache code");
				return ERROR_FAIL;
			}
			/* Ingenic Xburst1 is not support RDHWR, so we can not use it get CACHE_LINE_SIZE in mips32_pracc_synchronize_cache 
                Ingenic Xburst2 is support RDHWR, but we don't need to use it for more robustness */
		    if ((cpuType == MIPS_INGENIC_XBURST1) || (cpuType == MIPS_INGENIC_XBURST2)) rel = MIPS32_ISA_RELEASE2_INGENIC_FORBID_RDHWR;
			retval = mips32_pracc_synchronize_cache(ejtag_info, start_addr, end_addr, cached, rel);
		}
	}
	else {
		if (cached == 3 || cached == 0) {		/* Write back cache or write through cache */
			uint32_t start_addr = addr;
			uint32_t end_addr = addr + count * size;
			uint32_t rel = (conf & MIPS32_CONFIG0_AR_MASK) >> MIPS32_CONFIG0_AR_SHIFT;
			if (rel > 1) {
				LOG_DEBUG("Unknown release in cache code");
				return ERROR_FAIL;
			}
			retval = mips32_pracc_synchronize_cache(ejtag_info, start_addr, end_addr, cached, rel);
		}
	}

	return retval;
}

int mips32_pracc_invalidate_cache(struct target *target, struct mips_ejtag *ejtag_info, int cache)
{
	static uint32_t inv_inst_cache[] = {

		/* Determine how big the I$ is */
		MIPS32_MFC0(t7, 16, 1),						/* C0_Config1 */  	
		MIPS32_ADDIU(t1, t7, zero),
		MIPS32_SRL(t1, t7,CFG1_ISSHIFT),
		MIPS32_ANDI(t1, t1, 0x7),
		MIPS32_ADDIU(t0, zero, 64),				    /* li t0, 64 */
		MIPS32_SLLV(t1, t0, t1),				    /* I$ Sets per way */

		MIPS32_SRL(t7, t7,CFG1_IASHIFT),
		MIPS32_ANDI(t7, t7, 0x7),
		MIPS32_ADDIU(t7, t7, 1),
		MIPS32_MUL(t1, t1, t7),					    /* Total number of sets */

		/* Clear TagLo/TagHi registers */
		MIPS32_MTC0(zero, C0_ITAGLO, 0),		    /* C0_ITagLo */
		MIPS32_MTC0(zero, C0_ITAGHI, 0),		    /* C0_ITagHi */
		MIPS32_MFC0(t7, 16, 1),						/* Re-read C0_Config1 */

		/* Isolate I$ Line Size */
		MIPS32_ADDIU(t0, zero, 2),				    /* li a2, 2 */
		MIPS32_SRL(t7, t7,CFG1_ILSHIFT),
		MIPS32_ANDI(t7, t7, 0x7),

		MIPS32_SLLV(t7, t0, t7),					/* Now have true I$ line size in bytes */
		MIPS32_LUI(t0, 0x8000),					    /* Get a KSeg0 address for cacheops */

		MIPS32_CACHE(Index_Store_Tag_I, 0, t0),
		MIPS32_ADDI(t1, t1,NEG16(1)),				/* Decrement set counter */
		MIPS32_BNE(t1, zero, NEG16(3)),
		MIPS32_ADD(t0, t0, t7),
	};

	uint32_t inv_data_cache[] = {

        	MIPS32_MFC0(t7, 16, 1),						/* read C0_Config1 */

		MIPS32_SRL (t1, t7, CFG1_DSSHIFT),			/* extract DS */
		MIPS32_ANDI(t1, t1, 0x7),
		MIPS32_ADDIU(t0, zero, 64),				    /* li t0, 64 */
		MIPS32_SLLV(t1, t0, t1),					/* D$ Sets per way */

		MIPS32_SRL(t7, t7, CFG1_DASHIFT),			/* extract DA */
		MIPS32_ANDI(t7, t7, 0x7),
		MIPS32_ADDIU(t7, t7, 1),
		MIPS32_MUL(t1, t1, t7),					    /* Total number of sets */

		/* Clear TagLo/TagHi registers */
		MIPS32_MTC0(zero, C0_TAGLO, 0),				/* write C0_TagLo */
		MIPS32_MTC0(zero, C0_TAGHI, 0),				/* write C0_TagHi */
		MIPS32_MTC0(zero, C0_TAGLO, 2),				/* write C0_DTagLo */
		MIPS32_MTC0(zero, C0_TAGHI, 2),				/* write C0_DTagHi */

		/* Isolate D$ Line Size */
		MIPS32_MFC0(t7, 16, 1),						/* Re-read C0_Config1 */
		MIPS32_ADDIU(t0, zero, 2),				    /* li a2, 2 */

		MIPS32_SRL(t7, t7, CFG1_DLSHIFT),			/* extract DL */
		MIPS32_ANDI(t7, t7, 0x7),

		MIPS32_SLLV(t7, t0, t7),					/* Now have true I$ line size in bytes */

		MIPS32_LUI(t0, 0x8000)					    /* Get a KSeg0 address for cacheops */
	};

	uint32_t inv_L2_cache[] = {

        	MIPS32_MFC0(t7, 16, 2),						/* read C0_Config2 */

		MIPS32_SRL (t1, t7, CFG2_SSSHIFT),			/* extract SS */
		MIPS32_ANDI(t1, t1, 0xf),
		MIPS32_ADDIU(t0, zero, 64),				    /* li t0, 64 */
		MIPS32_SLLV(t1, t0, t1),					/* D$ Sets per way */

		MIPS32_SRL(t7, t7, CFG2_SASHIFT),			/* extract DA */
		MIPS32_ANDI(t7, t7, 0xf),
		MIPS32_ADDIU(t7, t7, 1),
		MIPS32_MUL(t1, t1, t7),					    /* Total number of sets */

		/* Clear TagLo/TagHi registers */
		MIPS32_MTC0(zero, C0_TAGLO, 0),				/* write C0_TagLo */
		MIPS32_MTC0(zero, C0_TAGHI, 0),				/* write C0_TagHi */
		MIPS32_MTC0(zero, C0_TAGLO, 2),				/* write C0_DTagLo */
		MIPS32_MTC0(zero, C0_TAGHI, 2),				/* write C0_DTagHi */

		/* Isolate D$ Line Size */
		MIPS32_MFC0(t7, 16, 2),						/* Re-read C0_Config1 */
		MIPS32_ADDIU(t0, zero, 2),				    /* li a2, 2 */

		MIPS32_SRL(t7, t7, CFG2_SLSHIFT),			/* extract DL */
		MIPS32_ANDI(t7, t7, 0xf),

		MIPS32_SLLV(t7, t0, t7),					/* Now have true I$ line size in bytes */

		MIPS32_LUI(t0, 0x8000)					    /* Get a KSeg0 address for cacheops */
	};
	uint32_t done[] = {
		MIPS32_LUI(t7, UPPER16(MIPS32_PRACC_TEXT)),
		MIPS32_ORI(t7, t7, LOWER16(MIPS32_PRACC_TEXT)),
		MIPS32_JR(t7),							/* jr start */
		MIPS32_MFC0(t7, 31, 0)						/* move COP0 DeSave to $15 */
	};

	struct pracc_queue_info ctx = {.max_code = 26+20};	/* alloc memory for the worst case */

	uint32_t conf;
	uint32_t bpl;

	pracc_queue_init(&ctx);
	if (ctx.retval != ERROR_OK) {
		LOG_ERROR("pracc_queue_init failed");
		goto exit;
	}

	/* Read Config1 Register to retrieve cache info */
	if (cache == INSTNOWB || cache == DATA || cache == DATANOWB) {
		/* Read Config1 Register to retrieve cache info */
		mips32_pracc_cp0_read(ejtag_info, &conf, 16, 1);
	} else if (cache == L2 || cache == L2NOWB){
		mips32_pracc_cp0_read(ejtag_info, &conf, 16, 2);
	}

	switch (cache) {
		case INSTNOWB:
			/* Extract cache line size */
			bpl	 = (conf >> CFG1_ILSHIFT) & 7; /* bit 21:19 */

			/* Core configured with Instruction cache */
			if (bpl == 0) {
				LOG_USER("no instructure cache configured");
				ctx.retval = ERROR_OK;
				goto exit;
			}

			for (unsigned i = 0; i < ARRAY_SIZE(inv_inst_cache); i++)
				pracc_add(&ctx, 0, inv_inst_cache[i]);

			break;

		case DATA:
		case DATANOWB:
			/* Extract cache line size */
			bpl	 = (conf >> CFG1_DLSHIFT) & 7; /* bit 12:10 */

			/* Core configured with Instruction cache */
			if (bpl == 0) {
				LOG_USER("no data cache configured");
				ctx.retval = ERROR_OK;
				goto exit;
 			}

			/* Write exit code */
			for (unsigned i = 0; i < ARRAY_SIZE(inv_data_cache); i++)
				pracc_add(&ctx, 0, inv_data_cache[i]);

			if (cache == DATA)
				pracc_add(&ctx, 0, MIPS32_CACHE(Index_Writeback_Inv_D, 0, t0));
			else {
				if ((cache == ALLNOWB) || (cache == DATANOWB))
					pracc_add(&ctx, 0, MIPS32_CACHE(Index_Store_Tag_D, 0, t0));
			}

			pracc_add(&ctx, 0, MIPS32_ADDI(t1, t1,NEG16(1)));				// Decrement set counter
			pracc_add(&ctx, 0, MIPS32_BNE(t1, zero, NEG16(3)));
			pracc_add(&ctx, 0, MIPS32_ADD(t0, t0, t7));
			break;

		case L2:
		case L2NOWB:
			/* Extract cache line size */
			bpl	 = (conf >> CFG2_SLSHIFT) & 15; /* bit 7:4 */

			/* Core configured with L2 cache */
			if (bpl == 0) {
				LOG_USER("no L2 cache configured");
				ctx.retval = ERROR_OK;
				goto exit;
 			}

			/* Write exit code */
			for (unsigned i = 0; i < ARRAY_SIZE(inv_L2_cache); i++)
				pracc_add(&ctx, 0, inv_L2_cache[i]);

			if (cache == L2)
				pracc_add(&ctx, 0, MIPS32_CACHE(Index_Writeback_Inv_S, 0, t0));
			else {
				if ((cache == ALLNOWB) || (cache == L2NOWB))
					pracc_add(&ctx, 0, MIPS32_CACHE(Index_Store_Tag_S, 0, t0));
			}

			pracc_add(&ctx, 0, MIPS32_ADDI(t1, t1,NEG16(1)));				// Decrement set counter
			pracc_add(&ctx, 0, MIPS32_BNE(t1, zero, NEG16(3)));
			pracc_add(&ctx, 0, MIPS32_ADD(t0, t0, t7));
			break;

	}

	/* Write exit code */
	for (unsigned i = 0; i < ARRAY_SIZE(done); i++)
		pracc_add(&ctx, 0, done[i]);

	/* Start code execution */
	//ctx.code_count = 0; /* Disable pracc access verification due BNZ instruction */
	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL);
	if (ctx.retval != ERROR_OK)
		LOG_DEBUG("mips32_pracc_queue_exec failed - ctx.retval: %d", ctx.retval);

exit:
	pracc_queue_free(&ctx);
	return ctx.retval;
}

int mips32_pracc_write_regs(struct mips_ejtag *ejtag_info, uint32_t *regs)
{
	LOG_DEBUG("mips32_pracc_write_regs");
	static const uint32_t cp0_write_code[] = {
		MIPS32_MTC0(1, 12, 0),							/* move $1 to status */
		MIPS32_MTLO(1),									/* move $1 to lo */
		MIPS32_MTHI(1),									/* move $1 to hi */
		MIPS32_MTC0(1, 8, 0),							/* move $1 to badvaddr */
		MIPS32_MTC0(1, 13, 0),							/* move $1 to cause*/
		MIPS32_MTC0(1, 24, 0),							/* move $1 to depc (pc) */
		MIPS32_MTC0(1, 10, 4),							/* move $1 to guestClt1 (pc) */
	};
	const int cp0_write_code_len = ARRAY_SIZE(cp0_write_code);
	const int GPR_BEGIN = 2;
	const int GPR_END = 32;
	const int MAX_INST_GPR_1 = 3;
	const int MAX_INST_GPR_2_THRU_31 = (GPR_END-GPR_BEGIN) * 2;
	const int MAX_INST_CP0 = cp0_write_code_len * 3;
	const int MAX_INST_EPILOGUE = 1;  /* currently just a branch */

	struct pracc_queue_info ctx = {.max_code = MAX_INST_GPR_1 + MAX_INST_GPR_2_THRU_31 + MAX_INST_CP0 + MAX_INST_EPILOGUE};

	pracc_queue_init(&ctx);
	if (ctx.retval != ERROR_OK)
		goto exit;

	/* load registers 2 to 31 with lui and ori instructions, check if some instructions can be saved */
	for (int i = GPR_BEGIN; i < GPR_END; i++) {
		if (LOWER16((regs[i])) == 0)					/* if lower half word is 0, lui instruction only */
			pracc_add(&ctx, 0, MIPS32_LUI(i, UPPER16((regs[i]))));
		else if (UPPER16((regs[i])) == 0)					/* if upper half word is 0, ori with $0 only*/
			pracc_add(&ctx, 0, MIPS32_ORI(i, 0, LOWER16((regs[i]))));
		else {									/* default, load with lui and ori instructions */
			pracc_add(&ctx, 0, MIPS32_LUI(i, UPPER16((regs[i]))));
			pracc_add(&ctx, 0, MIPS32_ORI(i, i, LOWER16((regs[i]))));
		}
	}

	for (int i = GPR_END; i != GPR_END+cp0_write_code_len; i++) {
		pracc_add(&ctx, 0, MIPS32_LUI(1, UPPER16((regs[i]))));		/* load CPO value in $1, with lui and ori */
		pracc_add(&ctx, 0, MIPS32_ORI(1, 1, LOWER16((regs[i]))));
		pracc_add(&ctx, 0, cp0_write_code[i-GPR_END]);					/* write value from $1 to CPO register */
	}

	pracc_add(&ctx, 0, MIPS32_MTC0(15, 31, 0));				/* load $15 in DeSave */
	pracc_add(&ctx, 0, MIPS32_LUI(1, UPPER16((regs[1]))));			/* load upper half word in $1 */
	pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));					/* jump to start */
	pracc_add(&ctx, 0, MIPS32_ORI(1, 1, LOWER16((regs[1]))));		/* load lower half word in $1 */

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL);

	ejtag_info->reg8  = regs[8];
	ejtag_info->reg9  = regs[9];
	ejtag_info->reg10 = regs[10];
	ejtag_info->reg11 = regs[11];
exit:
	pracc_queue_free(&ctx);
	return ctx.retval;
}

int mips32_pracc_write_fpu_regs(struct mips_ejtag *ejtag_info, uint32_t *regs)
{

	LOG_DEBUG("mips32_pracc_write_fpu_regs");
	struct pracc_queue_info ctx = {.max_code = 110};

	pracc_queue_init(&ctx);
	if (ctx.retval != ERROR_OK)
		goto exit;

	/* load f0..f31 */
	for (int i = MIPS32_F0; i < MIPS32_FCSR; i++) {
		if (LOWER16((regs[i-MIPS32_F0])) == 0)					/* if lower half word is 0, lui instruction only */
			pracc_add(&ctx, 0, MIPS32_LUI(8, UPPER16((regs[i-MIPS32_F0]))));
		else if (UPPER16((regs[i-MIPS32_F0])) == 0)					/* if upper half word is 0, ori with $0 only*/
			pracc_add(&ctx, 0, MIPS32_ORI(8, 0, LOWER16((regs[i-MIPS32_F0]))));
		else {									/* default, load with lui and ori instructions */
			pracc_add(&ctx, 0, MIPS32_LUI(8, UPPER16((regs[i-MIPS32_F0]))));
			pracc_add(&ctx, 0, MIPS32_ORI(8, 8, LOWER16((regs[i-MIPS32_F0]))));
		}

		pracc_add(&ctx, 0, MIPS32_MTC1(8, (i)));

	}

	/* fcsr */
	if (LOWER16((regs[MIPS32_FCSR-MIPS32_F0])) == 0)					/* if lower half word is 0, lui instruction only */
		pracc_add(&ctx, 0, MIPS32_LUI(8, UPPER16((regs[MIPS32_FCSR-MIPS32_F0]))));
	else if (UPPER16((regs[MIPS32_FCSR-MIPS32_F0])) == 0)					/* if upper half word is 0, ori with $0 only*/
		pracc_add(&ctx, 0, MIPS32_ORI(8, 8, LOWER16((regs[MIPS32_FCSR-MIPS32_F0]))));
	else {									/* default, load with lui and ori instructions */
		pracc_add(&ctx, 0, MIPS32_LUI(8, UPPER16((regs[MIPS32_FCSR-MIPS32_F0]))));
		pracc_add(&ctx, 0, MIPS32_ORI(8, 8, LOWER16((regs[MIPS32_FCSR-MIPS32_F0]))));
	}

	pracc_add(&ctx, 0, MIPS32_CTC1(8, 31));

	/* fir */
	if (LOWER16((regs[MIPS32_FIR])) == 0)					/* if lower half word is 0, lui instruction only */
		pracc_add(&ctx, 0, MIPS32_LUI(8, UPPER16((regs[MIPS32_FIR-MIPS32_F0]))));
	else if (UPPER16((regs[MIPS32_FIR])) == 0)					/* if upper half word is 0, ori with $0 only*/
		pracc_add(&ctx, 0, MIPS32_ORI(8, 8, LOWER16((regs[MIPS32_FIR-MIPS32_F0]))));
	else {									/* default, load with lui and ori instructions */
		pracc_add(&ctx, 0, MIPS32_LUI(8, UPPER16((regs[MIPS32_FIR-MIPS32_F0]))));
		pracc_add(&ctx, 0, MIPS32_ORI(8, 8, LOWER16((regs[MIPS32_FIR-MIPS32_F0]))));
	}

	pracc_add(&ctx, 0, MIPS32_CTC1(8, 0));

	pracc_add(&ctx, 0, MIPS32_MTC0(15, 31, 0));				/* load $15 in DeSave */
	pracc_add(&ctx, 0, MIPS32_LUI(1, UPPER16((regs[1]))));			/* load upper half word in $1 */
	pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));					/* jump to start */
	pracc_add(&ctx, 0, MIPS32_ORI(1, 1, LOWER16((regs[1]))));		/* load lower half word in $1 */

exit:
	pracc_queue_free(&ctx);
	return ctx.retval;
}

int mips32_pracc_read_regs(struct mips_ejtag *ejtag_info, uint32_t *regs)
{
	static int cp0_read_code[] = {
		MIPS32_MFC0(8, 12, 0),							/* move status to $8 */
		MIPS32_MFLO(8),									/* move lo to $8 */
		MIPS32_MFHI(8),									/* move hi to $8 */
		MIPS32_MFC0(8, 8, 0),							/* move badvaddr to $8 */
		MIPS32_MFC0(8, 13, 0),							/* move cause to $8 */
		MIPS32_MFC0(8, 24, 0),							/* move depc (pc) to $8 */
		MIPS32_MFC0(8, 10, 4),							/* move guestClt1  to $8 */
	};

	struct pracc_queue_info ctx = {.max_code = 52};

	pracc_queue_init(&ctx);
	if (ctx.retval != ERROR_OK)
		goto exit;

	pracc_add(&ctx, 0, MIPS32_MTC0(1, 31, 0));						/* move $1 to COP0 DeSave */
	pracc_add(&ctx, 0, MIPS32_LUI(1, PRACC_UPPER_BASE_ADDR));				/* $1 = MIP32_PRACC_BASE_ADDR */

	for (int i = 2; i != 32; i++)					/* store GPR's 2 to 31 */
		pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + (i * 4),
				  MIPS32_SW(i, PRACC_OUT_OFFSET + (i * 4), 1));

	for (int i = 0; i != ARRAY_SIZE(cp0_read_code); i++) {
		pracc_add(&ctx, 0, cp0_read_code[i]);				/* load COP0 needed registers to $8 */
		pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + (i + 32) * 4,			/* store $8 at PARAM OUT */
				  MIPS32_SW(8, PRACC_OUT_OFFSET + (i + 32) * 4, 1));
	}
	pracc_add(&ctx, 0, MIPS32_MFC0(8, 31, 0));					/* move DeSave to $8, reg1 value */
	pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + 4,					/* store reg1 value from $8 to param out */
			  MIPS32_SW(8, PRACC_OUT_OFFSET + 4, 1));

	pracc_add(&ctx, 0, MIPS32_MFC0(1, 31, 0));					/* move COP0 DeSave to $1, restore reg1 */
	pracc_add(&ctx, 0, MIPS32_SYNC);
	pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));					/* jump to start */
	pracc_add(&ctx, 0, MIPS32_MTC0(15, 31, 0));					/* load $15 in DeSave */

	if (ejtag_info->mode == 0)
		ctx.store_count++;	/* Needed by legacy code, due to offset from reg0 */

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, regs);

	ejtag_info->reg8  = regs[8];	/* reg8 is saved but not restored, next called function should restore it */
	ejtag_info->reg9  = regs[9];
	ejtag_info->reg10 = regs[10];
	ejtag_info->reg11 = regs[11];
exit:
	pracc_queue_free(&ctx);
	return ctx.retval;
}

int mips32_pracc_read_tlb_entry(struct mips_ejtag *ejtag_info, uint32_t *data, uint32_t index)
{
	struct pracc_queue_info ctx = {.max_code = 49};

	pracc_queue_init(&ctx);
	if (ctx.retval != ERROR_OK)
		goto exit;

	static int tlb_read_code[] = {
		MIPS32_MFC0(8, 2, 0),			/* Read C0_ENTRYLO0 */
		MIPS32_MFC0(8, 3, 0),			/* Read C0_ENTRYLO1 */
		MIPS32_MFC0(8, 10, 0),			/* Read C0_ENTRYHI */
		MIPS32_MFC0(8, 5, 0)			/* Read C0_PAGEMASK */
	};

	pracc_add(&ctx, 0, MIPS32_LUI(15, PRACC_UPPER_BASE_ADDR)); /* $15 = MIP32_PRACC_BASE_ADDR */
	pracc_add(&ctx, 0, MIPS32_LUI(8, UPPER16(index)));		   /* Load TLB Index to $8 */
	pracc_add(&ctx, 0, MIPS32_ORI(8, 8, LOWER16(index)));

	pracc_add(&ctx, 0, MIPS32_MTC0(8, 0, 0));				 /* write C0_Index */
	pracc_add(&ctx, 0, MIPS32_TLBR());						 /* Read TLB entry specified by Index */


	for (uint32_t i = 0; i <ARRAY_SIZE(tlb_read_code); i++) {
		pracc_add(&ctx, 0, tlb_read_code[i]);
		pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + (i * 4),
				  MIPS32_SW(8, PRACC_OUT_OFFSET + (i * 4), 15));
	}

	pracc_add(&ctx, 0, MIPS32_LUI(8, UPPER16(ejtag_info->reg8)));		/* restore upper 16 of $8 */
	pracc_add(&ctx, 0, MIPS32_ORI(8, 8, LOWER16(ejtag_info->reg8)));	/* restore lower 16 of $8 */

	pracc_add(&ctx, 0, MIPS32_SYNC);
	pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));	/* jump to start */
	pracc_add(&ctx, 0, MIPS32_MTC0(15, 31, 0));					/* load $15 in DeSave */

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, data);
exit:
	pracc_queue_free(&ctx);
	return ctx.retval;
}

int mips32_pracc_read_fpu_regs(struct mips_ejtag *ejtag_info, uint32_t *regs)
{
	LOG_DEBUG("mips32_pracc_read_fpu_regs");
	int i;
	struct pracc_queue_info ctx = {.max_code = 80};

	pracc_queue_init(&ctx);
	if (ctx.retval != ERROR_OK)
		goto exit;

	pracc_add(&ctx, 0, MIPS32_MTC0(1, 31, 0));						/* move $1 to COP0 DeSave */
	pracc_add(&ctx, 0, MIPS32_LUI(1, PRACC_UPPER_BASE_ADDR));		/* $1 = MIP32_PRACC_BASE_ADDR */

	for (i = 38; i != 70; i++) {
		pracc_add(&ctx, 0, MIPS32_MFC1(8, (i-38)));						/* load FP registers to $8 */
		pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + ((i-38) * 4),		/* store $8 at PARAM OUT */
				  MIPS32_SW(8, PRACC_OUT_OFFSET + ((i-38) * 4), 1));
	}

	pracc_add(&ctx, 0, MIPS32_CFC1(8, 31));						/* load FCSR registers to $8 */
	pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + ((i-38) * 4),		/* store $8 at PARAM OUT */
			  MIPS32_SW(8, PRACC_OUT_OFFSET + ((i-38) * 4), 1));

	pracc_add(&ctx, 0, MIPS32_CFC1(8, 0));						/* load FIR registers to $8 */
	pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + ((i-38) + 1) * 4,		/* store $8 at PARAM OUT */
			  MIPS32_SW(8, PRACC_OUT_OFFSET + ((i-38) + 1) * 4, 1));

	pracc_add(&ctx, 0, MIPS32_MFC0(1, 31, 0));					/* move COP0 DeSave to $1, restore reg1 */
	pracc_add(&ctx, 0, MIPS32_LUI(8, UPPER16(ejtag_info->reg8)));		/* restore upper 16 of $8 */
	pracc_add(&ctx, 0, MIPS32_ORI(8, 8, LOWER16(ejtag_info->reg8)));	/* restore lower 16 of $8 */

	pracc_add(&ctx, 0, MIPS32_SYNC);
	pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));	/* jump to start */
 
	pracc_add(&ctx, 0, MIPS32_MTC0(15, 31, 0));					/* load $15 in DeSave */

	if (ejtag_info->mode == 0)
		ctx.store_count++;	/* Needed by legacy code, due to offset from reg0 */

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, regs);
exit:
	pracc_queue_free(&ctx);
	return ctx.retval;
}

int mips32_pracc_read_msa_regs(struct mips_ejtag *ejtag_info, uint32_t *regs)
{
	LOG_DEBUG("mips32_pracc_read_msa_regs");
	int i;
	struct pracc_queue_info ctx = {.max_code = 80};

	pracc_queue_init(&ctx);
	if (ctx.retval != ERROR_OK)
		goto exit;

	pracc_add(&ctx, 0, MIPS32_MTC0(1, 31, 0));						/* move $1 to COP0 DeSave */
	pracc_add(&ctx, 0, MIPS32_LUI(1, PRACC_UPPER_BASE_ADDR));		/* $1 = MIP32_PRACC_BASE_ADDR */

	for (i = 74; i != 106; i++) {
		//pracc_add(&ctx, 0, MIPS32_MFC1(8, (i-74)));						/* load FP registers to $8 */
		//pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + ((i-74) * 4),		/* store $8 at PARAM OUT */
//				  MIPS32_SW(8, PRACC_OUT_OFFSET + ((i-74) * 4), 1));
	}

//	pracc_add(&ctx, 0, MIPS32_CFC1(8, 31));						/* load FCSR registers to $8 */
//	pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + ((i-38) * 4),		/* store $8 at PARAM OUT */
//			  MIPS32_SW(8, PRACC_OUT_OFFSET + ((i-38) * 4), 1));
//
//	pracc_add(&ctx, 0, MIPS32_CFC1(8, 0));						/* load FIR registers to $8 */
//	pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + ((i-38) + 1) * 4,		/* store $8 at PARAM OUT */
//			  MIPS32_SW(8, PRACC_OUT_OFFSET + ((i-38) + 1) * 4, 1));

	pracc_add(&ctx, 0, MIPS32_MFC0(1, 31, 0));					/* move COP0 DeSave to $1, restore reg1 */
	pracc_add(&ctx, 0, MIPS32_LUI(8, UPPER16(ejtag_info->reg8)));		/* restore upper 16 of $8 */
	pracc_add(&ctx, 0, MIPS32_ORI(8, 8, LOWER16(ejtag_info->reg8)));	/* restore lower 16 of $8 */

	pracc_add(&ctx, 0, MIPS32_SYNC);
	pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));	/* jump to start */
 
	pracc_add(&ctx, 0, MIPS32_MTC0(15, 31, 0));					/* load $15 in DeSave */

	if (ejtag_info->mode == 0)
		ctx.store_count++;	/* Needed by legacy code, due to offset from reg0 */

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, regs);
exit:
	pracc_queue_free(&ctx);
	return ctx.retval;
}

int mips32_pracc_read_dsp_regs(struct mips_ejtag *ejtag_info, uint32_t *val, uint32_t regs)
{
	struct pracc_queue_info ctx = {.max_code = 48};
	static uint32_t dsp_read_code[] = {
		MIPS32_MFHI (t0),		/* mfhi t0 ($ac0) OPCODE - 0x00004010 */
		MIPS32_DSP_MFHI (t0, 1),	/* mfhi	t0,$ac1 - OPCODE - 0x00204010 */
		MIPS32_DSP_MFHI (t0,2), /* mfhi	t0,$ac2 - OPCODE - 0x00404010 */
		MIPS32_DSP_MFHI (t0,3), /* mfhi	t0,$ac3 - OPCODE - 0x00604010*/
		MIPS32_MFLO (t0),		/* mflo t0 ($ac0) OPCODE - 0x00004012 */
		MIPS32_DSP_MFLO (t0,1), /* mflo	t0,$ac1 - OPCODE - 0x00204012 */
		MIPS32_DSP_MFLO (t0,2), /* mflo	t0,$ac2 - OPCODE - 0x00404012 */
		MIPS32_DSP_MFLO (t0,3),	/* mflo	t0,$ac3 - OPCODE - 0x00604012 */
		MIPS32_DSP_RDDSP (t0,0x3F), /* OPCODE - 0x7c3f44b8 */
	};

	/* check status register to determine if dsp register access is enabled */
	/* Get status register so it can be restored later */

	/* Init context queue */
	pracc_queue_init(&ctx);
	if (ctx.retval != ERROR_OK)
		goto exit;

	/* Save Status Register */
	pracc_add(&ctx, 0, MIPS32_MFC0(9, 12, 0));					/* move status to $9 (t1) 2*/

	/* Read it again in order to modify it */
	pracc_add(&ctx, 0, MIPS32_MFC0(8, 12, 0));					/* move status to $0 (t0) 3*/

	/* Enable access to DSP registers by setting MX bit in status register */
	pracc_add(&ctx, 0, MIPS32_LUI(15, UPPER16(MIPS32_DSP_ENABLE)));		/* $15 = MIPS32_PRACC_STACK 4/5/6*/
	pracc_add(&ctx, 0, MIPS32_ORI(15, 15, LOWER16(MIPS32_DSP_ENABLE)));
	pracc_add(&ctx, 0, MIPS32_OR(8, 8, 15));
	pracc_add(&ctx, 0, MIPS32_MTC0(8, 12, 0));					/* Enable DSP - update status registers 7*/

	pracc_add(&ctx, 0, dsp_read_code[regs]);					/* move AC or Control to $8 (t0) 8*/
	pracc_add(&ctx, 0, MIPS32_MTC0(9, 12, 0));					/* Restore status registers to previous setting 9*/

	pracc_add(&ctx, 0, MIPS32_LUI(15, PRACC_UPPER_BASE_ADDR));	/* $15 = MIPS32_PRACC_BASE_ADDR 1*/
	pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT, MIPS32_SW(8, PRACC_OUT_OFFSET, 15));	/* store $8 to pracc_out 10*/

	pracc_add(&ctx, 0, MIPS32_MFC0(15, 31, 0));							/* move COP0 DeSave to $15 11*/
	pracc_add(&ctx, 0, MIPS32_LUI(8, UPPER16(ejtag_info->reg8)));		/* restore upper 16 of $8 12*/
	pracc_add(&ctx, 0, MIPS32_ORI(8, 8, LOWER16(ejtag_info->reg8)));	/* restore lower 16 of $8 13*/

	pracc_add(&ctx, 0, MIPS32_LUI(9, UPPER16(ejtag_info->reg9)));		/* restore upper 16 of $9 14*/
	pracc_add(&ctx, 0, MIPS32_SYNC);
	pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));		/* jump to start 18*/
	pracc_add(&ctx, 0, MIPS32_ORI(9, 9, LOWER16(ejtag_info->reg9)));	/* restore lower 16 of $9 15*/

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, val);
exit:
	pracc_queue_free(&ctx);
	return ctx.retval;
}

int mips32_pracc_write_dsp_regs(struct mips_ejtag *ejtag_info, uint32_t val, uint32_t regs)
{
	struct pracc_queue_info ctx = {.max_code = 48};
	static uint32_t dsp_write_code[] = {
		MIPS32_MTHI (t0),		/* OPCODE - 0x01000011 */
		MIPS32_DSP_MTHI (t0,1),	/* OPCODE - 0x01000811 */
		MIPS32_DSP_MTHI (t0,2),	/* OPCODE - 0x01001011 */
		MIPS32_DSP_MTHI (t0,3),	/* OPCODE - 0x01001811 */
		MIPS32_MTLO (t0),		/* OPCODE - 0x01000013 */
		MIPS32_DSP_MTLO (t0,1), /* OPCODE - 0x01000813 */
		MIPS32_DSP_MTLO (t0,2),	/* OPCODE - 0x01001013 */
		MIPS32_DSP_MTLO (t0,3),	/* OPCODE - 0x01001813 */
		MIPS32_DSP_WRDSP (t0,0x1F),
	};

	/* Init context queue */
	pracc_queue_init(&ctx);
	if (ctx.retval != ERROR_OK)
		goto exit;

	/* Save Status Register */
	pracc_add(&ctx, 0, MIPS32_MFC0(9, 12, 0));					/* move status to $9 (t1) */

	/* Read it again in order to modify it */
	pracc_add(&ctx, 0, MIPS32_MFC0(8, 12, 0));					/* move status to $0 (t0) */

	/* Enable access to DSP registers by setting MX bit in status register */
	pracc_add(&ctx, 0, MIPS32_LUI(15, UPPER16(MIPS32_DSP_ENABLE)));		/* $15 = MIPS32_PRACC_STACK */
	pracc_add(&ctx, 0, MIPS32_ORI(15, 15, LOWER16(MIPS32_DSP_ENABLE)));
	pracc_add(&ctx, 0, MIPS32_OR(8, 8, 15));
	pracc_add(&ctx, 0, MIPS32_MTC0(8, 12, 0));					/* Enable DSP - update status registers */

	pracc_add(&ctx, 0, MIPS32_LUI(8, UPPER16(val)));			/* Load val to $8 (t0) */
	pracc_add(&ctx, 0, MIPS32_ORI(8, 8, LOWER16(val)));

	pracc_add(&ctx, 0, dsp_write_code[regs]);					 /* move AC or Control to $8 (t0) */

	pracc_add(&ctx, 0, MIPS32_NOP);								/* nop */
	pracc_add(&ctx, 0, MIPS32_MTC0(9, 12, 0));					/* Restore status registers to previous setting */
	pracc_add(&ctx, 0, MIPS32_NOP);								/* nop */

	pracc_add(&ctx, 0, MIPS32_MFC0(15, 31, 0));							/* move COP0 DeSave to $15 */
	pracc_add(&ctx, 0, MIPS32_LUI(8, UPPER16(ejtag_info->reg8)));		/* restore upper 16 of $8 */
	pracc_add(&ctx, 0, MIPS32_ORI(8, 8, LOWER16(ejtag_info->reg8)));	/* restore lower 16 of $8 */

	pracc_add(&ctx, 0, MIPS32_LUI(9, UPPER16(ejtag_info->reg9)));		/* restore upper 16 of $9 */

	pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));		/* jump to start */
	pracc_add(&ctx, 0, MIPS32_ORI(9, 9, LOWER16(ejtag_info->reg9)));	/* restore lower 16 of $9 */

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL);
exit:
	pracc_queue_free(&ctx);
	return ctx.retval;
}

/* fastdata upload/download requires an initialized working area
 * to load the download code; it should not be called otherwise
 * fetch order from the fastdata area
 * 1. start addr
 * 2. end addr
 * 3. data ...
 */
int mips32_pracc_fastdata_xfer(struct mips_ejtag *ejtag_info, struct working_area *source,
		int write_t, uint32_t addr, int count, uint32_t *buf)
{
	uint32_t handler_code[] = {
		/* caution when editing, table is modified below */
		/* r15 points to the start of this code */
		MIPS32_SW(8, MIPS32_FASTDATA_HANDLER_SIZE - 4, 15),
		MIPS32_SW(9, MIPS32_FASTDATA_HANDLER_SIZE - 8, 15),
		MIPS32_SW(10, MIPS32_FASTDATA_HANDLER_SIZE - 12, 15),
		MIPS32_SW(11, MIPS32_FASTDATA_HANDLER_SIZE - 16, 15),
		/* start of fastdata area in t0 */
		MIPS32_LUI(8, UPPER16(MIPS32_PRACC_FASTDATA_AREA)),
		MIPS32_ORI(8, 8, LOWER16(MIPS32_PRACC_FASTDATA_AREA)),
		MIPS32_LW(9, 0, 8),								/* start addr in t1 */
		MIPS32_LW(10, 0, 8),							/* end addr to t2 */
														/* loop: */
		/* 8 */ MIPS32_LW(11, 0, 0),					/* lw t3,[t8 | r9] */
		/* 9 */ MIPS32_SW(11, 0, 0),					/* sw t3,[r9 | r8] */
		MIPS32_BNE(10, 9, NEG16(3)),					/* bne $t2,t1,loop */
		MIPS32_ADDI(9, 9, 4),							/* addi t1,t1,4 */

		MIPS32_LW(8, MIPS32_FASTDATA_HANDLER_SIZE - 4, 15),
		MIPS32_LW(9, MIPS32_FASTDATA_HANDLER_SIZE - 8, 15),
		MIPS32_LW(10, MIPS32_FASTDATA_HANDLER_SIZE - 12, 15),
		MIPS32_LW(11, MIPS32_FASTDATA_HANDLER_SIZE - 16, 15),

		MIPS32_LUI(15, UPPER16(MIPS32_PRACC_TEXT)),
		MIPS32_ORI(15, 15, LOWER16(MIPS32_PRACC_TEXT)),
		MIPS32_JR(15),								/* jr start */
		MIPS32_MFC0(15, 31, 0),						/* move COP0 DeSave to $15 */
	};

	uint32_t jmp_code[] = {
		/* 0 */ MIPS32_LUI(15, 0),		/* addr of working area added below */
		/* 1 */ MIPS32_ORI(15, 15, 0),	/* addr of working area added below */
		MIPS32_JR(15),					/* jump to ram program */
		MIPS32_NOP,
	};

	int retval, i;
	uint32_t val, ejtag_ctrl, address;

	if (source->size < MIPS32_FASTDATA_HANDLER_SIZE) {
		LOG_DEBUG("source->size (%x) < MIPS32_FASTDATA_HANDLER_SIZE", source->size);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (write_t) {
		handler_code[8] = MIPS32_LW(11, 0, 8);	/* load data from probe at fastdata area */
		handler_code[9] = MIPS32_SW(11, 0, 9);	/* store data to RAM @ r9 */
	} else {
		handler_code[8] = MIPS32_LW(11, 0, 9);	/* load data from RAM @ r9 */
		handler_code[9] = MIPS32_SW(11, 0, 8);	/* store data to probe at fastdata area */
	}

	/* write program into RAM */
	if (write_t != ejtag_info->fast_access_save) {
		mips32_pracc_write_mem(ejtag_info, source->address, 4, ARRAY_SIZE(handler_code), handler_code);

		/* save previous operation to speed to any consecutive read/writes */
		ejtag_info->fast_access_save = write_t;
	}

	LOG_DEBUG("%s using 0x%.8" PRIx32 " for write handler", __func__, source->address);

	jmp_code[0] |= UPPER16(source->address);
	jmp_code[1] |= LOWER16(source->address);

	for (i = 0; i < (int) ARRAY_SIZE(jmp_code); i++) {
		retval = wait_for_pracc_rw(ejtag_info, &ejtag_ctrl);
		if (retval != ERROR_OK) {
			LOG_DEBUG("wait_for_pracc_rw failed - retval = %d", retval);
			return retval;
		}

		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
		mips_ejtag_drscan_32_out(ejtag_info, jmp_code[i]);

		/* Clear the access pending bit (let the processor eat!) */
		ejtag_ctrl = ejtag_info->ejtag_ctrl & ~EJTAG_CTRL_PRACC;
		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
		mips_ejtag_drscan_32_out(ejtag_info, ejtag_ctrl);
	}

	/* wait PrAcc pending bit for FASTDATA write */
	retval = wait_for_pracc_rw(ejtag_info, &ejtag_ctrl);
	if (retval != ERROR_OK) {
		LOG_DEBUG("wait_for_pracc_rw failed - retval: %d", retval);
		return retval;
	}

	/* next fetch to dmseg should be in FASTDATA_AREA, check */
	address = 0;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_ADDRESS);
	retval = mips_ejtag_drscan_32(ejtag_info, &address);
	if (retval != ERROR_OK) {
		LOG_DEBUG("mips_ejtag_drscan_32 failed - retval: %d", retval);
		return retval;
	}

	if (address != MIPS32_PRACC_FASTDATA_AREA) {
		LOG_DEBUG("address != MIPS32_PRACC_FASTDATA_AREA (0x%8.8x) - 0x%8.8x", MIPS32_PRACC_FASTDATA_AREA, address);
		return ERROR_FAIL;
	}

	/* Send the load start address */
	val = addr;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_FASTDATA);
	mips_ejtag_fastdata_scan(ejtag_info, 1, &val);

	retval = wait_for_pracc_rw(ejtag_info, &ejtag_ctrl);
	if (retval != ERROR_OK)
		return retval;

	/* Send the load end address */
	val = addr + (count - 1) * 4;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_FASTDATA);
	mips_ejtag_fastdata_scan(ejtag_info, 1, &val);

	unsigned num_clocks = 0;	/* like in legacy code */
	if (ejtag_info->mode != 0)
		num_clocks = ((uint64_t)(ejtag_info->scan_delay) * jtag_get_speed_khz() + 500000) / 1000000;

	for (i = 0; i < count; i++) {
		jtag_add_clocks(num_clocks);
		retval = mips_ejtag_fastdata_scan(ejtag_info, write_t, buf++);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("fastdata load failed");
		return retval;
	}

	retval = wait_for_pracc_rw(ejtag_info, &ejtag_ctrl);
	if (retval != ERROR_OK)
		return retval;

	address = 0;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_ADDRESS);
	retval = mips_ejtag_drscan_32(ejtag_info, &address);
	if (retval != ERROR_OK)
		return retval;

	if (address != MIPS32_PRACC_TEXT)
		LOG_ERROR("mini program did not return to start");

	return retval;
}

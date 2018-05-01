/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2009 SoftPLC Corporation                                *
 *       http://softplc.com                                                *
 *   dick@softplc.com                                                      *
 *                                                                         *
 *   Copyright (C) 2009 Zachary T Welch                                    *
 *   zw@superlucidity.net                                                  *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/jtag.h>
#include "commands.h"

struct cmd_queue_page {
	struct cmd_queue_page *next;
	void *address;
	size_t used;
};

#define CMD_QUEUE_PAGE_SIZE (1024 * 1024)
static struct cmd_queue_page *cmd_queue_pages;
static struct cmd_queue_page *cmd_queue_pages_tail;

struct jtag_command *jtag_command_queue;
static struct jtag_command **next_command_pointer = &jtag_command_queue;

void jtag_queue_command(struct jtag_command *cmd)
{
	/* this command goes on the end, so ensure the queue terminates */
	cmd->next = NULL;

	struct jtag_command **last_cmd = next_command_pointer;
	assert(NULL != last_cmd);
	assert(NULL == *last_cmd);
	*last_cmd = cmd;

	/* store location where the next command pointer will be stored */
	next_command_pointer = &cmd->next;
}

void *cmd_queue_alloc(size_t size)
{
	struct cmd_queue_page **p_page = &cmd_queue_pages;
	int offset;
	uint8_t *t;

	/*
	 * WARNING:
	 *    We align/round the *SIZE* per below
	 *    so that all pointers returned by
	 *    this function are reasonably well
	 *    aligned.
	 *
	 * If we did not, then an "odd-length" request would cause the
	 * *next* allocation to be at an *odd* address, and because
	 * this function has the same type of api as malloc() - we
	 * must also return pointers that have the same type of
	 * alignment.
	 *
	 * What I do not/have is a reasonable portable means
	 * to align by...
	 *
	 * The solution here, is based on these suggestions.
	 * http://gcc.gnu.org/ml/gcc-help/2008-12/msg00041.html
	 *
	 */
	union worse_case_align {
		int i;
		long l;
		float f;
		void *v;
	};
#define ALIGN_SIZE  (sizeof(union worse_case_align))

	/* The alignment process. */
	size = (size + ALIGN_SIZE - 1) & (~(ALIGN_SIZE - 1));
	/* Done... */

	if (*p_page) {
		p_page = &cmd_queue_pages_tail;
		if (CMD_QUEUE_PAGE_SIZE - (*p_page)->used < size)
			p_page = &((*p_page)->next);
	}

	if (!*p_page) {
		*p_page = malloc(sizeof(struct cmd_queue_page));
		(*p_page)->used = 0;
		size_t alloc_size = (size < CMD_QUEUE_PAGE_SIZE) ?
					CMD_QUEUE_PAGE_SIZE : size;
		(*p_page)->address = malloc(alloc_size);
		(*p_page)->next = NULL;
		cmd_queue_pages_tail = *p_page;
	}

	offset = (*p_page)->used;
	(*p_page)->used += size;

	t = (*p_page)->address;
	return t + offset;
}

static void cmd_queue_free(void)
{
	struct cmd_queue_page *page = cmd_queue_pages;

	while (page) {
		struct cmd_queue_page *last = page;
		free(page->address);
		page = page->next;
		free(last);
	}

	cmd_queue_pages = NULL;
	cmd_queue_pages_tail = NULL;
}

void jtag_command_queue_reset(void)
{
	cmd_queue_free();

	jtag_command_queue = NULL;
	next_command_pointer = &jtag_command_queue;
}

enum scan_type jtag_scan_type(const struct scan_command *cmd)
{
	int type = 0;

	if (cmd->fields[0].in_value)
		type |= SCAN_IN;
	if (cmd->fields[0].out_value)
		type |= SCAN_OUT;

	return type;
}

int jtag_build_buffer(const struct scan_command *cmd, uint8_t **buffer)
{
	int bit_count = 0;

	bit_count = cmd->fields[0].num_bits;
	*buffer = calloc(1, DIV_ROUND_UP(bit_count, 8));

		if (cmd->fields[0].out_value) {
			buf_set_buf(cmd->fields[0].out_value, 0, *buffer,
					0, cmd->fields[0].num_bits);
		}

	return bit_count;
}

int jtag_read_buffer(uint8_t *buffer, const struct scan_command *cmd)
{
	/* if neither in_value nor in_handler
	 * are specified we don't have to examine this field
	 */
	if (cmd->fields[0].in_value) {
		int num_bits = cmd->fields[0].num_bits;
		uint8_t *captured = buf_set_buf(buffer, 0,
				malloc(DIV_ROUND_UP(num_bits, 8)), 0, num_bits);

		if (cmd->fields[0].in_value)
			buf_cpy(captured, cmd->fields[0].in_value, num_bits);

		free(captured);
	}

	return ERROR_OK;
}

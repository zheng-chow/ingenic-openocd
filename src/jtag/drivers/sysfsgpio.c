/***************************************************************************
 *   Copyright (C) 2012 by Creative Product Design, marc @ cpdesign.com.au *
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

/* 2014-12: Addition of the SWD protocol support is based on the initial work
 * on bcm2835gpio.c by Paul Fertser and modifications by Jean-Christian de Rivaz. */

/**
 * @file
 * This driver implements a bitbang jtag interface using gpio lines via
 * sysfs.
 * The aim of this driver implementation is use system GPIOs but avoid the
 * need for a additional kernel driver.
 * (Note memory mapped IO is another option, however it doesn't mix well with
 * the kernel gpiolib driver - which makes sense I guess.)
 *
 * A gpio is required for tck, tms, tdi and tdo. One or both of srst and trst
 * must be also be specified. The required jtag gpios are specified via the
 * sysfsgpio_jtag_nums command or the relevant sysfsgpio_XXX_num commang.
 * The srst and trst gpios are set via the sysfsgpio_srst_num and
 * sysfsgpio_trst_num respectively. GPIO numbering follows the kernel
 * convention of starting from 0.
 *
 * The gpios should not be in use by another entity, and must not be requested
 * by a kernel driver without also being exported by it (otherwise they can't
 * be exported by sysfs).
 *
 * The sysfs gpio interface can only manipulate one gpio at a time, so the
 * bitbang write handler remembers the last state for tck, tms, tdi to avoid
 * superfluous writes.
 * For speed the sysfs "value" entry is opened at init and held open.
 * This results in considerable gains over open-write-close (45s vs 900s)
 *
 * Further work could address:
 *  -srst and trst open drain/ push pull
 *  -configurable active high/low for srst & trst
 */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include "bitbang.h"

#include <sys/mman.h>

#define GPIO_BASE	(0x10010000) /* GPIO controller */

#define PDPIN		(*(pio_base+0x300/4))
#define PDINT		(*(pio_base+0x310/4))
#define PDINTS		(*(pio_base+0x314/4))
#define PDINTC		(*(pio_base+0x318/4))
#define PDMSK		(*(pio_base+0x320/4))
#define PDMSKS		(*(pio_base+0x324/4))
#define PDMSKC		(*(pio_base+0x328/4))
#define PDPAT1		(*(pio_base+0x330/4))
#define PDPAT1S		(*(pio_base+0x334/4))
#define PDPAT1C		(*(pio_base+0x338/4))
#define PDPAT0		(*(pio_base+0x340/4))
#define PDPAT0S		(*(pio_base+0x344/4))
#define PDPAT0C		(*(pio_base+0x348/4))

#define PZINTS		(*(pio_base+0x714/4))
#define PZINTC		(*(pio_base+0x718/4))
#define PZMSKS		(*(pio_base+0x724/4))
#define PZMSKC		(*(pio_base+0x728/4))
#define PZPAT1S		(*(pio_base+0x734/4))
#define PZPAT1C		(*(pio_base+0x738/4))
#define PZPAT0S		(*(pio_base+0x744/4))
#define PZPAT0C		(*(pio_base+0x748/4))
#define PZGID2LD	(*(pio_base+0x7f0/4))

static int dev_mem_fd;
static volatile uint32_t *pio_base;

static int sysfsgpio_read(void);
static void sysfsgpio_write(int tck, int tms, int tdi);
static void sysfsgpio_reset(int trst, int srst);

static int sysfsgpio_init(void);
static int sysfsgpio_quit(void);

/* gpio numbers for each gpio. Negative values are invalid */
static int tck_gpio = -1;
static int tms_gpio = -1;
static int tdi_gpio = -1;
static int tdo_gpio = -1;
static int trst_gpio = -1;
static int srst_gpio = -1;

/* Transition delay coefficients */
static int speed_coeff = 113714;
static int speed_offset = 28;
static int port_status;
static unsigned int jtag_delay;

static int sysfsgpio_read(void)							//mod
{										//mod
	return !!(PDPIN & 1<<tdo_gpio);						//mod
}										//mod

static void sysfsgpio_write(int tck, int tms, int tdi)				//mod
{										//mod
	//uint32_t set = tck<<tck_gpio | tms<<tms_gpio | tdi<<tdi_gpio;		//mod
	//uint32_t clear = !tck<<tck_gpio | !tms<<tms_gpio | !tdi<<tdi_gpio;	//mod
	port_status = port_status & ~(1<<tck_gpio | 1<<tms_gpio | 1<<tdi_gpio) | tck<<tck_gpio | tms<<tms_gpio | tdi<<tdi_gpio;

	//PDPAT0S = set;							//mod
	//PDPAT0C = clear;							//mod
	PDPAT0 = port_status;

	for (unsigned int i = 0; i < jtag_delay; i++)				//mod
		asm volatile ("");						//mod
}										//mod

/* (1) assert or (0) deassert reset lines */
static void sysfsgpio_reset(int trst, int srst)					//mod
{										//mod
	uint32_t set = 0;							//mod
	uint32_t clear = 0;							//mod
										//mod
	if (trst_gpio > 0) {							//mod
		//set |= !trst<<trst_gpio;					//mod
		//clear |= trst<<trst_gpio;					//mod
		port_status = port_status & ~(1<<trst_gpio) | !trst<<trst_gpio;
	}									//mod

	if (srst_gpio > 0) {							//mod
		//set |= !srst<<srst_gpio;					//mod
		//clear |= srst<<srst_gpio;					//mod
		port_status = port_status & ~(1<<srst_gpio) | !srst<<srst_gpio;
	}									//mod

	//PDPAT0S = set;							//mod
	//PDPAT0C = clear;							//mod
	PDPAT0 = port_status;
}										//mod

static int sysfsgpio_khz(int khz, int *jtag_speed)				//mod
{										//mod
	if (!khz) {								//mod
		LOG_DEBUG("RCLK not supported");				//mod
		return ERROR_FAIL;						//mod
	}									//mod
	*jtag_speed = speed_coeff/khz - speed_offset;				//mod
	if (*jtag_speed < 0)							//mod
		*jtag_speed = 0;						//mod
	return ERROR_OK;							//mod
}										//mod

static int sysfsgpio_speed_div(int speed, int *khz)				//mod
{										//mod
	*khz = speed_coeff/(speed + speed_offset);				//mod
	return ERROR_OK;							//mod
}										//mod

static int sysfsgpio_speed(int speed)						//mod
{										//mod
	jtag_delay = speed;							//mod
	printf("jtag_delay = %08X\n",jtag_delay);
	return ERROR_OK;							//mod
}										//mod

static int is_gpio_valid(int gpio)						//mod
{										//mod
	return gpio >= 0 && gpio <= 32;						//mod
}										//mod

COMMAND_HANDLER(sysfsgpio_handle_jtag_gpionums)					//mod
{										//mod
	if (CMD_ARGC == 4) {							//mod
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tck_gpio);		//mod
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], tms_gpio);		//mod
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[2], tdi_gpio);		//mod
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[3], tdo_gpio);		//mod
	} else if (CMD_ARGC != 0) {						//mod
		return ERROR_COMMAND_SYNTAX_ERROR;				//mod
	}									//mod

	command_print(CMD_CTX,							//mod
			"GPIO config: tck = %d, tms = %d, tdi = %d, tdi = %d",	//mod
			tck_gpio, tms_gpio, tdi_gpio, tdo_gpio);		//mod

	return ERROR_OK;							//mod
}										//mod

COMMAND_HANDLER(sysfsgpio_handle_jtag_gpionum_tck)				//mod
{										//mod
	if (CMD_ARGC == 1)							//mod
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tck_gpio);		//mod

	command_print(CMD_CTX, "SysfsGPIO num: tck = %d", tck_gpio);		//mod
	return ERROR_OK;							//mod
}										//mod

COMMAND_HANDLER(sysfsgpio_handle_jtag_gpionum_tms)				//mod
{										//mod
	if (CMD_ARGC == 1)							//mod
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tms_gpio);		//mod

	command_print(CMD_CTX, "SysfsGPIO num: tms = %d", tms_gpio);		//mod
	return ERROR_OK;							//mod
}										//mod

COMMAND_HANDLER(sysfsgpio_handle_jtag_gpionum_tdo)				//mod
{										//mod
	if (CMD_ARGC == 1)							//mod
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tdo_gpio);		//mod

	command_print(CMD_CTX, "SysfsGPIO num: tdo = %d", tdo_gpio);		//mod
	return ERROR_OK;							//mod
}										//mod

COMMAND_HANDLER(sysfsgpio_handle_jtag_gpionum_tdi)				//mod
{										//mod
	if (CMD_ARGC == 1)							//mod
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tdi_gpio);		//mod

	command_print(CMD_CTX, "SysfsGPIO num: tdi = %d", tdi_gpio);		//mod
	return ERROR_OK;							//mod
}										//mod

COMMAND_HANDLER(sysfsgpio_handle_jtag_gpionum_srst)				//mod
{										//mod
	if (CMD_ARGC == 1)							//mod
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], srst_gpio);		//mod

	command_print(CMD_CTX, "SysfsGPIO num: srst = %d", srst_gpio);		//mod
	return ERROR_OK;							//mod
}										//mod

COMMAND_HANDLER(sysfsgpio_handle_jtag_gpionum_trst)				//mod
{										//mod
	if (CMD_ARGC == 1)							//mod
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], trst_gpio);		//mod

	command_print(CMD_CTX, "SysfsGPIO num: trst = %d", trst_gpio);		//mod
	return ERROR_OK;							//mod
}										//mod

COMMAND_HANDLER(sysfsgpio_handle_speed_coeffs)					//mod
{										//mod
	if (CMD_ARGC == 2) {							//mod
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], speed_coeff);		//mod
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], speed_offset);		//mod
	}									//mod
	return ERROR_OK;							//mod
}										//mod

static const struct command_registration sysfsgpio_command_handlers[] = {	//mod
	{									//mod
		.name = "sysfsgpio_jtag_nums",					//mod
		.handler = &sysfsgpio_handle_jtag_gpionums,			//mod
		.mode = COMMAND_CONFIG,						//mod
		.help = "gpio numbers for tck, tms, tdi, tdo. (in that order)",	//mod
		.usage = "(tck tms tdi tdo)* ",					//mod
	},									//mod
	{									//mod
		.name = "sysfsgpio_tck_num",					//mod
		.handler = &sysfsgpio_handle_jtag_gpionum_tck,			//mod
		.mode = COMMAND_CONFIG,						//mod
		.help = "gpio number for tck.",					//mod
	},									//mod
	{									//mod
		.name = "sysfsgpio_tms_num",					//mod
		.handler = &sysfsgpio_handle_jtag_gpionum_tms,			//mod
		.mode = COMMAND_CONFIG,						//mod
		.help = "gpio number for tms.",					//mod
	},									//mod
	{									//mod
		.name = "sysfsgpio_tdo_num",					//mod
		.handler = &sysfsgpio_handle_jtag_gpionum_tdo,			//mod
		.mode = COMMAND_CONFIG,						//mod
		.help = "gpio number for tdo.",					//mod
	},									//mod
	{									//mod
		.name = "sysfsgpio_tdi_num",					//mod
		.handler = &sysfsgpio_handle_jtag_gpionum_tdi,			//mod
		.mode = COMMAND_CONFIG,						//mod
		.help = "gpio number for tdi.",					//mod
	},									//mod
	{									//mod
		.name = "sysfsgpio_srst_num",					//mod
		.handler = &sysfsgpio_handle_jtag_gpionum_srst,			//mod
		.mode = COMMAND_CONFIG,						//mod
		.help = "gpio number for srst.",				//mod
	},									//mod
	{									//mod
		.name = "sysfsgpio_trst_num",					//mod
		.handler = &sysfsgpio_handle_jtag_gpionum_trst,			//mod
		.mode = COMMAND_CONFIG,						//mod
		.help = "gpio number for trst.",				//mod
	},									//mod
	{									//mod
		.name = "sysfsgpio_speed_coeffs",				//mod
		.handler = &sysfsgpio_handle_speed_coeffs,			//mod
		.mode = COMMAND_CONFIG,						//mod
		.help = "SPEED_COEFF and SPEED_OFFSET for delay calculations.",	//mod
	},									//mod
	COMMAND_REGISTRATION_DONE						//mod
};										//mod

static struct bitbang_interface sysfsgpio_bitbang = {				//mod
	.read = sysfsgpio_read,							//mod
	.write = sysfsgpio_write,						//mod
	.reset = sysfsgpio_reset,						//mod
	.blink = NULL								//mod
};										//mod

struct jtag_interface sysfsgpio_interface = {					//mod
	.name = "sysfsgpio",							//mod
	.supported = DEBUG_CAP_TMS_SEQ,						//mod
	.execute_queue = bitbang_execute_queue,					//mod
	.transports = jtag_only,						//mod
	.speed = sysfsgpio_speed,						//mod
	.khz = sysfsgpio_khz,							//mod
	.speed_div = sysfsgpio_speed_div,					//mod
	.commands = sysfsgpio_command_handlers,					//mod
	.init = sysfsgpio_init,							//mod
	.quit = sysfsgpio_quit,							//mod
};										//mod

static int sysfsgpio_init(void)							//mod
{
	bitbang_interface = &sysfsgpio_bitbang;					//mod

	if (!is_gpio_valid(tdo_gpio) || !is_gpio_valid(tdi_gpio) ||		//mod
		!is_gpio_valid(tck_gpio) || !is_gpio_valid(tms_gpio) ||		//mod
		(trst_gpio != -1 && !is_gpio_valid(trst_gpio)) ||		//mod
		(srst_gpio != -1 && !is_gpio_valid(srst_gpio)))			//mod
		return ERROR_JTAG_INIT_FAILED;					//mod

	dev_mem_fd = open("/dev/mem", O_RDWR | O_SYNC);				//mod
	if (dev_mem_fd < 0) {							//mod
		perror("open");							//mod
		return ERROR_JTAG_INIT_FAILED;					//mod
	}									//mod

	pio_base = mmap(NULL, sysconf(_SC_PAGE_SIZE), PROT_READ | PROT_WRITE,	//mod
				MAP_SHARED, dev_mem_fd, 0x10010000);		//mod

	if (pio_base == MAP_FAILED) {						//mod
		perror("mmap");							//mod
		close(dev_mem_fd);						//mod
		return ERROR_JTAG_INIT_FAILED;					//mod
	}									//mod

	PZINTC = 1<<tdo_gpio | 1<<tdi_gpio | 1<<tck_gpio | 1<<tms_gpio;		//mod
	PZMSKS = 1<<tdo_gpio | 1<<tdi_gpio | 1<<tck_gpio | 1<<tms_gpio;		//mod
	PZPAT1S = 1<<tdo_gpio;							//mod
	PZPAT1C = 1<<tdi_gpio | 1<<tck_gpio | 1<<tms_gpio;			//mod
	PZPAT0S = 1<<tms_gpio;							//mod
	PZPAT0C = 1<<tdi_gpio | 1<<tck_gpio;					//mod
	PZGID2LD = 0x3;								//mod

	if (trst_gpio != -1) {							//mod
		PZINTC = 1 << trst_gpio;					//mod
		PZMSKS = 1 << trst_gpio;					//mod
		PZPAT1C = 1 << trst_gpio;					//mod
		PZPAT0S = 1 << trst_gpio;					//mod
		PZGID2LD = 0x3;							//mod
	}									//mod
	if (srst_gpio != -1) {							//mod
		PZINTC = 1 << srst_gpio;					//mod
		PZMSKS = 1 << srst_gpio;					//mod
		PZPAT1C = 1 << srst_gpio;					//mod
		PZPAT0S = 1 << srst_gpio;					//mod
		PZGID2LD = 0x3;							//mod
	}									//mod

	port_status = PDPAT0;

	LOG_INFO("GPIO JTAG bitbang driver");					//mod

	printf("port_status = %08X\n",port_status);

	return ERROR_OK;							//mod
}										//mod

static int sysfsgpio_quit(void)							//mod
{										//mod
	PZINTC = 1<<tdo_gpio | 1<<tdi_gpio | 1<<tck_gpio | 1<<tms_gpio;		//mod
	PZMSKS = 1<<tdo_gpio | 1<<tdi_gpio | 1<<tck_gpio | 1<<tms_gpio;		//mod
	PZPAT1S = 1<<tdo_gpio;							//mod
	PZPAT1C = 1<<tdi_gpio | 1<<tck_gpio | 1<<tms_gpio;			//mod
	PZPAT0C = 1<<tdi_gpio | 1<<tck_gpio | 1<<tms_gpio;			//mod
	PZGID2LD = 0x3;								//mod

	if (trst_gpio != -1) {							//mod
		PZINTC = 1 << trst_gpio;					//mod
		PZMSKS = 1 << trst_gpio;					//mod
		PZPAT1C = 1 << trst_gpio;					//mod
		PZPAT0C = 1 << trst_gpio;					//mod
		PZGID2LD = 0x3;							//mod
	}									//mod
	if (srst_gpio != -1) {							//mod
		PZINTC = 1 << srst_gpio;					//mod
		PZMSKS = 1 << srst_gpio;					//mod
		PZPAT1C = 1 << srst_gpio;					//mod
		PZPAT0C = 1 << srst_gpio;					//mod
		PZGID2LD = 0x3;							//mod
	}									//mod

	return ERROR_OK;							//mod
}										//mod


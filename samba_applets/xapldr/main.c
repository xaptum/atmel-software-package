/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2016, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "applet.h"

#include "board.h"
#include "chip.h"
#include "trace.h"

#include "pin_defs.h"
#include "serial/console.h"
#include "mm/cache.h"
#include "gpio/pio.h"
#include "spi/spid.h"
#include "peripherals/bus.h"
#include "peripherals/wdt.h"
#include "peripherals/pmc.h"
#include "gpio/pio.h"

#include <assert.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "serial/usart.h"
#include "serial/usartd.h"
#include "pin_defs.h"
#include "extram/ddram.h"

/*----------------------------------------------------------------------------
 *        Local type definitions
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

#define BLOCK_SIZE                    512u

#define DDR3L_MEM_SIZE                (256 * 1024 * 1024)

/*----------------------------------------------------------------------------
 *         Local variables
 *----------------------------------------------------------------------------*/

static uint8_t* buffer;

static uint32_t buffer_size;

static uint32_t mem_size;

/*----------------------------------------------------------------------------
 *         Local functions
 *----------------------------------------------------------------------------*/

static bool init_extram_from_preset(uint32_t preset)
{
	enum _ddram_devices device;
	struct _mpddrc_desc desc;

	switch (preset) {
#ifdef CONFIG_HAVE_DDR2_MT47H128M8
	case 0:
		trace_warning_wp("Preset 0 (4 x MT47H128M8)\r\n");
		device = MT47H128M8;
		break;
#endif
#ifdef CONFIG_HAVE_DDR2_MT47H64M16
	case 1:
		trace_warning_wp("Preset 1 (MT47H64M16)\r\n");
		device = MT47H64M16;
		break;
#endif
#ifdef CONFIG_HAVE_DDR2_MT47H128M16
	case 2:
		trace_warning_wp("Preset 2 (2 x MT47H128M16)\r\n");
		device = MT47H128M16;
		break;
#endif
#ifdef CONFIG_HAVE_LPDDR2_MT42L128M16
	case 3:
		trace_warning_wp("Preset 3 (2 x MT42L128M16)\r\n");
		device = MT42L128M16;
		break;
#endif
#ifdef CONFIG_HAVE_DDR3_MT41K128M16
	case 4:
		trace_warning_wp("Preset 4 (1 x MT41K128M16)\r\n");
		device = MT41K128M16;
		break;
#endif
#ifdef CONFIG_HAVE_LPDDR3_EDF8164A3MA
	case 5:
		trace_warning_wp("Preset 5 (EDF8164A3MA)\r\n");
		device = EDF8164A3MA;
		break;
#endif
#ifdef CONFIG_HAVE_SDRAM_IS42S16100E
	case 6:
		trace_warning_wp("Preset 6 (IS42S16100E)\r\n");
		device = IS42S16100E;
		break;
#endif
#ifdef CONFIG_HAVE_SDRAM_W981216BH
	case 7:
		trace_warning_wp("Preset 7 (W981216BH)\r\n");
		device = W981216BH;
		break;
#endif
#ifdef CONFIG_HAVE_DDR2_W971G16SG
	case 8:
		trace_warning_wp("Preset 8 (W971G16SG)\r\n");
		device = W971G16SG;
		break;
#endif
#ifdef CONFIG_HAVE_DDR2_W972GG6KB
	case 9:
		trace_warning_wp("Preset 9 (W972GG6KB)\r\n");
		device = W972GG6KB;
		break;
#endif
#ifdef CONFIG_HAVE_SDRAM_AS4C16M16SA
	case 10:
		trace_warning_wp("Preset 10 (AS4C16M16SA)\r\n");
		device = AS4C16M16SA;
		break;
#endif
	default:
		trace_error("Unsupported DRAM preset (%u).\r\n",
				(unsigned)preset);
		return false;
	}

	board_cfg_matrix_for_ddr();
	ddram_init_descriptor(&desc, device);
	ddram_configure(&desc);
	return true;
}

static bool check_extram(void)
{
	volatile uint32_t *extram = (volatile uint32_t *)DDR_CS_ADDR;
	int i;

	if (!pmc_is_system_clock_enabled(PMC_SYSTEM_CLOCK_DDR))
		return false;

	for (i = 0; i < 1024; i++)
		extram[i] = (i & 0xff) * 0x01010101;

	for (i = 0; i < 1024; i++)
		if (extram[i] != (i & 0xff) * 0x01010101)
			return false;

	return true;
}

static uint32_t handle_cmd_initialize(uint32_t cmd, uint32_t *mailbox)
{
	union initialize_mailbox *mbx = (union initialize_mailbox*)mailbox;

	assert(cmd == APPLET_CMD_INITIALIZE);

	if (!applet_set_init_params(mbx))
		return APPLET_FAIL;

	trace_warning_wp("\r\nApplet 'XapLoader' from "
			"softpack " SOFTPACK_VERSION ".\r\n");

	/* configure USART pins */
	/* pio_configure(pins_usart, ARRAY_SIZE(pins_usart)); */

    init_extram_from_preset(4);

    if (!check_extram())
    {
        trace_error("Init DDR3L fails!\r\n");
		return APPLET_FAIL;
    }

    mem_size = DDR3L_MEM_SIZE;
    buffer = applet_buffer;
	buffer_size = applet_buffer_size & ~(BLOCK_SIZE - 1);
	if (buffer_size == 0) {
		trace_error("Not enough memory for buffer\r\n");
		return APPLET_FAIL;
	}

	mbx->out.buf_addr      = (uint32_t)buffer;
	mbx->out.buf_size      = buffer_size;
	mbx->out.page_size     = BLOCK_SIZE;
	mbx->out.mem_size      = mem_size;
	mbx->out.erase_support = 0;
	mbx->out.nand_header   = 0;

	trace_warning_wp("DDR3L device initialization successful\n\r");
	trace_warning_wp("Buffer Address: 0x%lx\r\n", mbx->out.buf_addr);
	trace_warning_wp("Buffer Size: %ld bytes\r\n", mbx->out.buf_size);
	trace_warning_wp("Page Size: %ld bytes\r\n", mbx->out.page_size);
	trace_warning_wp("Memory Size: %ld pages\r\n", mbx->out.mem_size);

	return APPLET_SUCCESS;
}

static uint32_t handle_cmd_read_info(uint32_t cmd, uint32_t *mailbox)
{
	/* 'read info' uses the same mailbox output as 'initialize' */
	union initialize_mailbox *mbx = (union initialize_mailbox*)mailbox;

	assert(cmd == APPLET_CMD_READ_INFO);

	mbx->out.buf_addr = (uint32_t)buffer;
	mbx->out.buf_size = buffer_size;
	mbx->out.page_size = BLOCK_SIZE;
	mbx->out.mem_size = mem_size;
	mbx->out.erase_support = 0;
	mbx->out.nand_header = 0;

	return APPLET_SUCCESS;
}

static uint32_t handle_cmd_write_pages(uint32_t cmd, uint32_t *mailbox)
{
	union read_write_erase_pages_mailbox *mbx =
		(union read_write_erase_pages_mailbox*)mailbox;
	uint32_t offset = mbx->in.offset;
	uint32_t length = mbx->in.length;
    void *pdst;

	assert(cmd == APPLET_CMD_WRITE_PAGES);

	/* check that requested size does not overflow buffer */
	if ((length * BLOCK_SIZE) > buffer_size) {
		trace_error("Buffer overflow\r\n");
		return APPLET_FAIL;
	}

	/* check that requested offset/size does not overflow memory */
	if (offset + length > mem_size) {
		trace_error("Memory overflow\r\n");
		return APPLET_FAIL;
	}

    pdst = (void *)((offset * BLOCK_SIZE));
    memcpy(pdst, buffer, length * BLOCK_SIZE);

	trace_info_wp("Wrote %u bytes at absolute addr 0x%08x\r\n",
			(unsigned)(mbx->in.length * BLOCK_SIZE),
			(unsigned)(mbx->in.offset * BLOCK_SIZE));
	mbx->out.pages = mbx->in.length;

	return APPLET_SUCCESS;
}

static uint32_t handle_cmd_read_pages(uint32_t cmd, uint32_t *mailbox)
{
	union read_write_erase_pages_mailbox *mbx =
		(union read_write_erase_pages_mailbox*)mailbox;
	uint32_t offset = mbx->in.offset;
	uint32_t length = mbx->in.length;
    void *psrc;

	assert(cmd == APPLET_CMD_READ_PAGES);

	/* check that requested size does not overflow buffer */
	if ((length * BLOCK_SIZE) > buffer_size) {
		trace_error("Buffer overflow\r\n");
		return APPLET_FAIL;
	}

	/* check that requested offset/size does not overflow memory */
	if (offset + length > mem_size) {
		trace_error("Memory overflow\r\n");
		return APPLET_FAIL;
	}

    psrc = (void *)((offset * BLOCK_SIZE));
    memcpy(buffer, psrc, length * BLOCK_SIZE);

	trace_info_wp("Read %u bytes at absolute address 0x%08x\r\n",
			(unsigned)(mbx->in.length * BLOCK_SIZE),
			(unsigned)(mbx->in.offset * BLOCK_SIZE));
	mbx->out.pages = mbx->in.length;

	return APPLET_SUCCESS;
}


/*----------------------------------------------------------------------------
 *         Commands list
 *----------------------------------------------------------------------------*/

const struct applet_command applet_commands[] = {
	{ APPLET_CMD_INITIALIZE, handle_cmd_initialize },
	{ APPLET_CMD_READ_INFO, handle_cmd_read_info },
	{ APPLET_CMD_WRITE_PAGES, handle_cmd_write_pages },
	{ APPLET_CMD_READ_PAGES, handle_cmd_read_pages },
	{ 0, NULL }
};

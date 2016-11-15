/* ----------------------------------------------------------------------------
 *         SAM Software Package License
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

/**
 * \file
 *
 * Interface for Synchronous Serial (SSC) controller.
 *
 */

#ifndef _SSC_H
#define _SSC_H

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include <stdint.h>

#include "chip.h"
#include "dma/dma.h"
#include "io.h"
#include "mutex.h"

/*------------------------------------------------------------------------------
 *        Types
 *----------------------------------------------------------------------------*/

struct _ssc_desc;

typedef void (*ssc_callback_t)(struct _ssc_desc* desc, void* args);
/**
 * Configuration setting structure.
 */

struct _ssc_desc {
	Ssc *addr;
	/* Master Clock */
	uint32_t bit_rate;
	/* Sample Frequency (fs) Ratio */
	uint32_t sample_rate;
	/* Number of bits of the slot */
 	uint8_t  slot_length;
	/* Number of slot per frame */
 	uint8_t  slot_num;
	/* Transmit Clock Selection */
	bool tx_cfg_cks_tk;
	/* Receive Clock Selection */
	bool rx_cfg_cks_rk;
	/* Transmit Start Selection */
	uint16_t tx_start_selection;
	/* Receive Start Selection */
	uint16_t rx_start_selection;

	struct {
		mutex_t mutex;

		struct _buffer buffer;
		uint16_t transferred;
		ssc_callback_t callback;
		void*   cb_args;
	} rx, tx;

	struct {
		struct {
			struct dma_channel *channel;
			struct dma_xfer_cfg cfg;
		} rx;
		struct {
			struct dma_channel *channel;
			struct dma_xfer_cfg cfg;
		} tx;
	} dma;
};

enum _ssc_buf_attr {
	SSC_BUF_ATTR_WRITE = 0x01,
	SSC_BUF_ATTR_READ  = 0x02,
};

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Configures a SSC peripheral.If the divided clock is not used, the master
 * clock frequency can be set to 0.
 * \note The emitter and transmitter are disabled by this function.
 * \param desc  Pointer to an SSC instance.
 */
extern void ssc_configure(struct _ssc_desc* desc);

/**
 * \brief Configures the transmitter of a SSC peripheral.
 * \param desc  Pointer to an SSC instance.
 * \param tcmr Transmit Clock Mode Register value.
 * \param tfmr Transmit Frame Mode Register value.
 */
extern void ssc_configure_transmitter(struct _ssc_desc* desc, uint32_t tcmr,
					uint32_t tfmr);
/**
 * \brief Configures the receiver of a SSC peripheral.
 * \param desc  Pointer to an SSC instance.
 * \param rcmr Receive Clock Mode Register value.
 * \param rfmr Receive Frame Mode Register value.
 */
extern void ssc_configure_receiver(struct _ssc_desc* desc, uint32_t rcmr,
					uint32_t rfmr);
/**
 * \brief Enables the transmitter of a SSC peripheral.
 * \param desc  Pointer to an SSC instance.
 */
extern void ssc_enable_transmitter(struct _ssc_desc* desc);

/**
 * \brief Disables the transmitter of a SSC peripheral.
 * \param desc  Pointer to an SSC instance.
 */
extern void ssc_disable_transmitter(struct _ssc_desc* desc);

/**
 * \brief Enables the receiver of a SSC peripheral.
 * \param desc  Pointer to an SSC instance.
 */
extern void ssc_enable_receiver(struct _ssc_desc* desc);

/**
 * \brief Disables the receiver of a SSC peripheral.
 * \param desc  Pointer to an SSC instance.
 */
extern void ssc_disable_receiver(struct _ssc_desc* desc);

/**
 * \brief Enables one or more interrupt sources of a SSC peripheral.
 * \param desc  Pointer to an SSC instance.
 * \param sources Bitwise OR of selected interrupt sources.
 */
extern void ssc_enable_interrupts(struct _ssc_desc* desc, uint32_t sources);

/**
 * \brief Disables one or more interrupt sources of a SSC peripheral.
 * \param desc  Pointer to an SSC instance.
 * \param sources Bitwise OR of selected interrupt sources.
 */
extern void ssc_disable_interrupts(struct _ssc_desc* desc, uint32_t sources);

/**
 * \brief Sends one data frame through a SSC peripheral. If another frame is currently
 * being sent, this function waits for the previous transfer to complete.
 * \param desc  Pointer to an SSC instance.
 * \param frame Data frame to send.
 */
extern void ssc_write(struct _ssc_desc* desc, uint32_t frame);

/**
 * \brief Waits until one frame is received on a SSC peripheral, and returns it.
 * \param desc  Pointer to an SSC instance.
 */
extern uint32_t ssc_read(struct _ssc_desc* desc);

/**
 * \brief Return 1 if one frame is received, 0 otherwise.
 * \param desc  Pointer to an SSC instance.
 */
extern bool ssc_is_rx_ready(struct _ssc_desc* desc);

extern int ssc_transfer(struct _ssc_desc* desc, struct _buffer* buf, ssc_callback_t cb, void* user_args);

extern bool ssc_dma_tx_transfer_is_done(struct _ssc_desc* desc);

extern void ssc_dma_tx_stop(struct _ssc_desc* desc);

extern void ssc_dma_tx_set_callback(struct _ssc_desc* desc, ssc_callback_t cb, void* arg);

extern bool ssc_dma_rx_transfer_is_done(struct _ssc_desc* desc);

extern void ssc_dma_rx_stop(struct _ssc_desc* desc);

extern void ssc_dma_rx_set_callback(struct _ssc_desc* desc, ssc_callback_t cb, void* arg);

#endif /* #ifndef _SSC_H */
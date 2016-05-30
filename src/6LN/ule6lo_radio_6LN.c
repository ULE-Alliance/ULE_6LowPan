/*
 * Copyright (c) 2016, ULE Alliance.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the ULE 6LoWPAN library.
 *
 */


/****************************************************************************
 *                               Include files
 ****************************************************************************/
#include "ule6lo_radio_6LN.h"
#include "../../LinkLayerAbs/6LN/lla_6LN.h"
#include "ule6loLLI_6LN.h"
#include "ule6loTestIn_6LN.h"
#include <netstack.h>
#include <packetbuf.h>

/****************************************************************************
 *                              Macro definitions
 ****************************************************************************/

/****************************************************************************
 *                     Enumerations/Type definitions/Structs
 ****************************************************************************/

/****************************************************************************
 *                            Global variables/const
 ****************************************************************************/


/****************************************************************************
 *                            Local variables/const
 ****************************************************************************/

/****************************************************************************
 *                          Local Function prototypes
 ****************************************************************************/

/****************************************************************************
 *                                Implementation
 ****************************************************************************/
static int
init(void)
{
	return 0;
}

static int
prepare(const void *payload, unsigned short payload_len)
{
	return 1;
}


static int
transmit(unsigned short transmit_len)
{
	return RADIO_TX_OK;
}

static int
send(const void *payload, unsigned short payload_len)
{
	prepare(payload, payload_len);
	ule6loLLI_send(payload, payload_len);
	ule6lo_tx_package_counter++;
	if(ule6lo_txHook!=NULL) {
	  ule6lo_txHook(payload, payload_len);
	}
	return transmit(payload_len);
}

static int
radio_read(void *buf, unsigned short buf_len)
{
	return 0;
}

static int
channel_clear(void)
{
	return 1;
}

static int
receiving_packet(void)
{
	return 0;
}

static int
pending_packet(void)
{
	return 0;
}

static int
on(void)
{
	return 0;
}

static int
off(void)
{
	return 0;
}

static radio_result_t
get_value(radio_param_t param, radio_value_t *value)
{
	return RADIO_RESULT_NOT_SUPPORTED;
}

static radio_result_t
set_value(radio_param_t param, radio_value_t value)
{
	return RADIO_RESULT_NOT_SUPPORTED;
}

static radio_result_t
get_object(radio_param_t param, void *dest, size_t size)
{
	return RADIO_RESULT_NOT_SUPPORTED;
}

static radio_result_t
set_object(radio_param_t param, const void *src, size_t size)
{
	return RADIO_RESULT_NOT_SUPPORTED;
}

const struct radio_driver ule6lo_radio_driver =
{
		init,
		prepare,
		transmit,
		send,
		radio_read,
		channel_clear,
		receiving_packet,
		pending_packet,
		on,
		off,
		get_value,
		set_value,
		get_object,
		set_object
};


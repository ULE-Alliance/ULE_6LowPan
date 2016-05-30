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
#include "ule6loLLI_6LN.h"
#include <netstack.h>
#include <packetbuf.h>
#include "../../DebugFunctions/DebugFunctions.h"
#include "ule6loTestIn_6LN.h"
#include "ule6lo_ipei_defines.h"
#include "ule6loGI_6LN.h"
#include "lla_6LN.h"
#include <stdio.h>
/****************************************************************************
*                              Macro definitions
****************************************************************************/

/****************************************************************************
*                     Enumerations/Type definitions/Structs
****************************************************************************/

/****************************************************************************
*                            Global variables/const
****************************************************************************/
uint32_t ule6lo_rx_package_counter=0;
uint32_t ule6lo_tx_package_counter=0;

/****************************************************************************
*                            Local variables/const
****************************************************************************/

/****************************************************************************
*                          Local Function prototypes
****************************************************************************/

/****************************************************************************
*                                Implementation
****************************************************************************/

ule6lo_status_t ule6loLLI_init(const ule6lo_IPEI_t *IPEIAddr) {
  if(ule6loGI_getStatus() != STATUS_NOT_READY) {
    initialize_network_stack(IPEIAddr);
    return STATUS_SUCCESS;
  }
  else {
    printf("ERROR: Please ensure ule6loGI_init() has been called before establishing connection\n");
    return STATUS_ERROR;
  }
}


ule6lo_status_t ule6loLLI_receive(const uint8_t* data, uint16_t	dataLength, const ule6lo_IPEI_t *ULEAddr) {
	ule6lo_rx_package_counter++;
	if(ule6lo_rxHook!=NULL) {
		ule6lo_rxHook(data,dataLength);
	}
	if(ULEAddr) {
       UIP_WRITE_RECEIVED_IPEI(ULEAddr);
    }
	packetbuf_copyfrom(data, dataLength);
	//packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER, (rimeaddr_t *) &rimeaddr_node_addr);
	//packetbuf_set_addr(PACKETBUF_ADDR_SENDER, (rimeaddr_t *) &pp_lladr);
	NETSTACK_RDC.input();
	return STATUS_SUCCESS;
}

void ule6loLLI_connected(ule6lo_status_t status) {
  ule6loGI_setStatus(status);
}

/**
 * Sends a empty string at the dect layer. It must send data before it can receive any data.
 */
void ule6loLLI_check(void) {
  ule6loLLI_send(NULL,0);
}

void ule6loLLI_delivered(ule6lo_status_t status) {
}

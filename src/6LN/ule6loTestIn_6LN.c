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
#include "ule6loTestIn_6LN.h"
#include <stdio.h>
#include "net/ipv6/uip-ds6.h"
#include "ule6loLLI_6LN.h"
#include "ule6loGI_6LN.h"
#include "net/ipv6/uip-ds6-nbr.h"
#include "../../tools/DebugFunctions/DebugFunctions.h"

/****************************************************************************
*                              Macro definitions
****************************************************************************/

/****************************************************************************
*                     Enumerations/Type definitions/Structs
****************************************************************************/

/****************************************************************************
*                            Global variables/const
****************************************************************************/
ule6lo_hook_t ule6lo_txHook = NULL;
ule6lo_hook_t ule6lo_rxHook = NULL;

/****************************************************************************
*                            Local variables/const
****************************************************************************/

/****************************************************************************
*                          Local Function prototypes
****************************************************************************/

/****************************************************************************
*                                Implementation
****************************************************************************/
ule6lo_status_t ule6loTestIn_init(void) {
  ule6lo_txHook = NULL;
  ule6lo_rxHook = NULL;
  ule6lo_rx_package_counter=0;
  ule6lo_tx_package_counter=0;
	return STATUS_SUCCESS;
}

ule6lo_status_t ule6loTestIn_deinit(void) {
  ule6lo_txHook = NULL;
  ule6lo_rxHook = NULL;
  ule6lo_rx_package_counter=0;
  ule6lo_tx_package_counter=0;
	return STATUS_SUCCESS;
}

ule6lo_status_t ule6loTestIn_reset(void) {
  ule6loTestIn_deinit();
  initialize_network_stack(NULL);
  printIPAddresses();
  printIPV6Neighbourlist();

	return STATUS_SUCCESS;
}

uint16_t ule6loTestIn_getNbListSize(void) {
	uip_ds6_nbr_t *nbr;
	uint16_t num=0;

	for(nbr = nbr_table_head(ds6_neighbors);
			nbr != NULL;
			nbr = nbr_table_next(ds6_neighbors, nbr)) {
		num++;
	}
	return num;
}

ule6lo_status_t ule6loTestIn_getNbList(uint16_t	index, ule6lo_nbr_t* nBListItem) {
	uip_ds6_nbr_t *nbr;
	uint8_t num=0;
	uint8_t i;

	for(nbr = nbr_table_head(ds6_neighbors);
			nbr != NULL;
			nbr = nbr_table_next(ds6_neighbors, nbr)) {

		if(num ==index) {
			//Located requested neighbor entry
			memcpy(&nBListItem->lladdr,&nbr->ipei,sizeof(ule6lo_IPEI_t));
			for(i=0;i<8;i++) {
				nBListItem->paddr.u16[i]=nbr->ipaddr.u16[i];
			}
			return STATUS_SUCCESS;
		}
		num++;
	}
	return STATUS_ERROR;
}

uint32_t ule6loTestIn_getnofSentPacket(void) {
	return ule6lo_tx_package_counter;
}

uint32_t ule6loTestIn_getnofReceivedPacket(void) {
	return ule6lo_rx_package_counter;
}

void ule6loTestIn_regRxHook(ule6lo_hook_t rxHook) {
	ule6lo_rxHook = rxHook;
}

void ule6loTestIn_regTxHook(ule6lo_hook_t txHook) {
	ule6lo_txHook = txHook;
}


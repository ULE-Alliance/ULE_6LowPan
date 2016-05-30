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
#include "ule6loNI_6LBR.h"
#include "eth-drv.h"
#include "ule6loTestIn_6LBR.h"

#include "net/ipv6/uip-icmp6.h"
#include "net/ipv6/uip-ds6.h"

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
/* Payload length of ICMPv6 echo requests used to measure RSSI with def rt */
#define ECHO_REQ_PAYLOAD_LEN   20
/****************************************************************************
 *                                Implementation
 ****************************************************************************/

ule6lo_status_t ule6loNI_receive(const uint8_t*	data, uint16_t dataLength) {
  ule6lo_border_router_rx_package_counter++;
  if(ule6lo_border_router_rxHook!=NULL) {
    ule6lo_border_router_rxHook(data,dataLength);
  }

  eth_drv_input(data,dataLength);
	return STATUS_SUCCESS;
}
void
ule6loNI_echoRequest(void)
{
  if(uip_ds6_get_global(ADDR_PREFERRED) == NULL) {
    return;
  }
  uip_icmp6_send(uip_ds6_defrt_choose(), ICMP6_ECHO_REQUEST, 0,
                 ECHO_REQ_PAYLOAD_LEN);
}

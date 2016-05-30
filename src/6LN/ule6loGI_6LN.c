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
#include "ule6loGI_6LN.h"
#include "string.h"
#include "linkaddr.h"
#include <stdio.h>
#include <sys/process.h>
#include <sys/ctimer.h>
#include <sys/etimer.h>
#include <sys/rtimer.h>
#include <netstack.h>
#include <net/queuebuf.h>
#include <ip/tcpip.h>
#include <ipv6/uip-ds6.h>

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
//FF FE inserted in middle to accomodate with the EUI-64 standard.
//The modification is done inside contiki.
//static uint8_t mac_address[] = {0x3c,0x97,0x0E,0xFF,0xFE,0x01,0x02,0x03};
static uint8_t mac_address[] = {0x3c,0x97,0x0E,0x00,0x00,0x01};
static ule6lo_status_t stack_status = STATUS_NOT_READY;

/****************************************************************************
 *                          Local Function prototypes
 ****************************************************************************/

/****************************************************************************
 *                                Implementation
 ****************************************************************************/

static void
set_rime_addr(void)
{
	linkaddr_t addr;
	uint8_t i;

	memset(&addr, 0, sizeof(linkaddr_t));
#if NETSTACK_CONF_WITH_IPV6
	memcpy(&uip_lladdr.addr, mac_address, sizeof(uip_lladdr.addr));
	memcpy(addr.u8, mac_address, sizeof(addr.u8));
#endif

	linkaddr_set_node_addr(&addr);
	printf("Rime started with address ");
	for(i = 0; i < sizeof(addr.u8) - 1; i++) {
		printf("%02X.", addr.u8[i]);
	}
	printf("%02X\n", addr.u8[i]);
}

void initialize_network_stack(const ule6lo_IPEI_t *IPEIAddr) {
  uint8_t i;
  process_init();
  process_start(&etimer_process, NULL);
  ctimer_init();
  rtimer_init();
  set_rime_addr();
  netstack_init();

  //queuebuf_init();
  if( IPEIAddr != NULL){
    printf("Size of uip_ipeiaddr.addr = %i\n", (int)(sizeof(uip_ipeiaddr.addr)));
    uip_ipeiaddr.addr[0] = 0x00;
    memcpy(&uip_ipeiaddr.addr[1], IPEIAddr, sizeof(ule6lo_IPEI_t));
  }
  process_start(&tcpip_process, NULL);
  stack_status =STATUS_SUCCESS;
}

ule6lo_status_t ule6loGI_init(void) {
  if(stack_status != STATUS_NOT_READY) {
    printf("State was not as expected, please don't modify state before initializing library\n");
    return STATUS_ERROR;
  }
	stack_status = STATUS_NOT_CONNECTED;
	return stack_status;
}

void ule6loGI_setStatus(ule6lo_status_t status) {
  stack_status = status;
}

ule6lo_status_t	ule6loGI_getStatus(void) {
	return stack_status;
}

ule6lo_status_t ule6loGI_getIp6addr(ule6lo_ipType_t ipType, ule6lo_ip6addr_t* ipAddr, ule6lo_ipMode_t mode) {
  ule6lo_status_t status = STATUS_NO_DATA;
  uip_ds6_addr_t *uip_addr = NULL;
  uint8_t index=0;

  switch(ipType) {
  case IP_ADDRESS_LINK_LOCAL:
    uip_addr = uip_ds6_get_link_local((int8_t)mode);
    break;
  case IP_ADDRESS_GLOBAL:
    uip_addr = uip_ds6_get_global((int8_t)mode);
    break;
  default:
    break;
  }

  if(uip_addr !=NULL) {
    for(index=0;index<16;index++) {
      ipAddr->u8[index]=uip_addr->ipaddr.u8[index];
    }
    status = STATUS_SUCCESS;
  }

  return status;
}

void ule6loGI_setMacAddress(ule6lo_macAddr_t* addr) {
	for(uint8_t i=0;i<sizeof(ule6lo_macAddr_t);i++) {
		mac_address[i]=addr->u8[i];
	}
}

/**
 * Add a multicast address to the list.
 */
ule6lo_status_t ule6loGI_addMulticastAddr(ule6lo_ip6addr_t* ipaddress) {
  ule6lo_status_t status = STATUS_NO_DEVICE;
  if( uip_ds6_maddr_add((uip_ipaddr_t*)ipaddress)){
    status = STATUS_SUCCESS;
  }
  return status;
}

/**
 * Remove a multicast address from the list.
 */
ule6lo_status_t ule6loGI_removeMulticastAddr(ule6lo_ip6addr_t* ipaddress) {
  ule6lo_status_t status = STATUS_NO_DEVICE;
#if UIP_CONF_MLD
  uip_ds6_maddr_t *locmaddr;
  if(uip_ds6_list_loop
      ((uip_ds6_element_t *)uip_ds6_if.maddr_list, UIP_DS6_MADDR_NB,
       sizeof(uip_ds6_maddr_t), (void*)ipaddress, 128,
       (uip_ds6_element_t **)&locmaddr) == FOUND) {
    locmaddr->isused = 0;
    locmaddr->report_count = 0;
    stimer_set(&(locmaddr->report_timeout), 0);
    uip_ds6_maddr_rm(locmaddr);
    memset(&locmaddr->ipaddr, 0x00, sizeof(uip_ipaddr_t));
    status = STATUS_SUCCESS;
  }
#endif
  return status;
}

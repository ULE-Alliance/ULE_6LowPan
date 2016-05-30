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

#include "DebugFunctions.h"
#include "net/ipv6/uip-ds6.h"
#include "contiki-conf.h"
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/if.h>
#ifdef LBR_ROLE
#include <ule6loGI_6LBR.h>
#include <ule6loTestIn_6LBR.h>
#else
#include <ule6loGI_6LN.h>
#include <ule6loTestIn_6LN.h>
#endif
#include "net/ip/uip-debug.h"
#include "net/ipv6/uip-ds6-nbr.h"

#if FILTERED_BORDER_ROUTER
void printPrefixListRouter() {
	printf("Printing Router prefix list with space for %i elements.\n",UIP_DS6_PREFIX_NB);
	printf("---------------------.......................\n");
	uip_ds6_prefix_t* locprefix;
	for(locprefix = uip_ds6_prefix_list;
			locprefix < uip_ds6_prefix_list+UIP_DS6_PREFIX_NB;
			locprefix++) {
		uip_debug_ipaddr_print(&locprefix->ipaddr);
		printf(", length=%i, isused=%i, advertise=%i, vlifetime=%i, plifetime=%i, l_a_reserved=%02X\n",locprefix->length,locprefix->isused, locprefix->advertise,locprefix->vlifetime,locprefix->plifetime,locprefix->l_a_reserved);
	}
	printf("End Router prefex list---------------------\n");
}

uint8_t isused;
  uip_ipaddr_t ipaddr;
  uint8_t length;
  uint8_t advertise;
  uint32_t vlifetime;
  uint32_t plifetime;
  uint8_t l_a_reserved; /**< on-link and autonomous flags + 6 reserved bits */

void printPrefixListNonRouter() {
	printf("Printing Non Router prefix list with space for %i elements.\n",UIP_DS6_PREFIX_NB);
		printf("------------------------.......................\n");
		uip_ds6_prefix_non_router_t* locprefix;
		for(locprefix = uip_ds6_prefix_list_non_router;
				locprefix < uip_ds6_prefix_list_non_router+UIP_DS6_PREFIX_NB;
				locprefix++) {
			uip_debug_ipaddr_print(&locprefix->ipaddr);
			printf(", length=%i, isused=%i, isinfinite=%i\n",locprefix->length,locprefix->isused, locprefix->isinfinite);
		}
		printf("End Non Router prefex list---------------------\n");
}
#endif

void printIPV6Neighbourlist(void)  {
	uip_ds6_nbr_t *nbr;
	int num;

	num = 0;
	printf("---NEIGHBOURS:\n");
	for(nbr = nbr_table_head(ds6_neighbors);
			nbr != NULL;
			nbr = nbr_table_next(ds6_neighbors, nbr)) {
		printf("IP ");
		for(num=0;num<8;num++) {
			printf(":%02X%02X",nbr->ipaddr.u8[num*2],nbr->ipaddr.u8[num*2+1]);
		}

		printf(" ,MAC ");
		const uip_lladdr_t * mac = uip_ds6_nbr_get_ll(nbr);
		for(num=0;num<6;num++) {
			printf(":%02X",mac->addr[num]);
		}
		printf(" , ");
		uip_debug_ipei_print(&nbr->ipei);

		printf(" , state=%i",nbr->state);

		printf(" , isRouter=%i\n",nbr->isrouter);
#if (LBR_ROLE & UIP_CONF_MLD)
		printf("MADDR ");
		
		for(int i = 0; i < UIP_DS6_MADDR_NBR_MAX; i++){
		    //for(num=0;num<8;num++) {
		      printf("Index: %d",nbr->maddr_list_index[i]);
		    //}
		    printf("\n");
		}
    
#endif
	}
	printf("---END NEIGHBOURS\n");
	printf("MADDR \n");
	for(int i = 0; i < UIP_DS6_MADDR_NB; i++){
  if( uip_ds6_if.maddr_list[i].isused){
    for(num=0;num<8;num++) {
      printf(":%02X%02X",uip_ds6_if.maddr_list[i].ipaddr.u8[num*2],uip_ds6_if.maddr_list[i].ipaddr.u8[num*2+1]);
    }
    printf("\n");
  }
}
}

void printIPAddresses(void) {
	uip_ds6_addr_t *locaddr;
	int i;
	printf("---My IP addressses are:\n");
	for(locaddr = uip_ds6_if.addr_list;
			locaddr < uip_ds6_if.addr_list + UIP_DS6_ADDR_NB; locaddr++) {
		if(locaddr->isused) {
			for(i = 0; i < 7; ++i) {
				printf("%02x%02x:", locaddr->ipaddr.u8[i * 2],
						locaddr->ipaddr.u8[i * 2 + 1]);
			}

			printf("%02x%02x\n", locaddr->ipaddr.u8[14], locaddr->ipaddr.u8[15]);
		}
	}
	printf("---Done\n");
}

void printLinkLocalAddress(void) {
	ule6lo_ip6addr_t ip;
	int i;
	printf("Link-local IPv6 address ");
	{
		if(ule6loGI_getIp6addr(IP_ADDRESS_LINK_LOCAL,&ip,IP_ADDRESS_ANY) == STATUS_SUCCESS) {
			for(i = 0; i < 7; ++i) {
				printf("%02x%02x:", ip.u8[i * 2],ip.u8[i * 2 + 1]);
			}
			printf("%02x%02x\n", ip.u8[14], ip.u8[15]);
		}
		else {
			printf(" not set\n");
		}
	}
}

void printGloballAddress(void) {
	ule6lo_ip6addr_t ip;
	int i;
	printf("Global IPv6 address ");
	{
		if(ule6loGI_getIp6addr(IP_ADDRESS_GLOBAL,&ip,IP_ADDRESS_ANY) == STATUS_SUCCESS) {
			for(i = 0; i < 7; ++i) {
				printf("%02x%02x:", ip.u8[i * 2],ip.u8[i * 2 + 1]);
			}
			printf("%02x%02x\n", ip.u8[14], ip.u8[15]);
		}
		else {
			printf(" not set\n");
		}
	}
}

void setFixedIP(const char* prefix) {
	uip_ds6_addr_t *lladdr;
	uip_ipaddr_t ipaddr;
	unsigned long vlifetime;
	uint8_t type;

	int i;
	lladdr = uip_ds6_get_link_local(-1);


		  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);
}

int readPCMacAddress(void)
{
//#if (TEST == 1)
    int rawsendsd = socket(PF_INET6, SOCK_DGRAM, 0);
/*#else
    // Open raw socket using IEEE802.3 protocol "IPv6 over bluebook"
    rawsendsd = socket(PF_PACKET, SOCK_DGRAM, htons(ETH_P_IPV6));
    if (rawsendsd < 0)
        dieSysError("socket(PF_PACKET)\n");
#endif*/

    struct ifreq ifReq;
    memset(&ifReq, 0, sizeof(ifReq));
    strncpy(ifReq.ifr_name, "eth0", sizeof(ifReq.ifr_name));

    // Get the interface MAC address
    if (ioctl(rawsendsd, SIOCGIFHWADDR, &ifReq))
        //dieSysError("ioctl(sendsd, SIOCGIFHWADDR)\n");
    	printf("Cant get mac address");

    // Save the interface HW address for later use when sending over this socket
     //memcpy(ethMacAddr.u8, ifReq.ifr_hwaddr.sa_data, 6);
    printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
    		ifReq.ifr_hwaddr.sa_data[0],
    		ifReq.ifr_hwaddr.sa_data[1],
    		ifReq.ifr_hwaddr.sa_data[2],
    		ifReq.ifr_hwaddr.sa_data[3],
    		ifReq.ifr_hwaddr.sa_data[4],
    		ifReq.ifr_hwaddr.sa_data[5]
    		                         );

    // Get the index of the "eth0" network interface
    /*if (ioctl(rawsendsd, SIOCGIFINDEX, &ifReq))
        dieSysError("ioctl(sendsd, SIOCGIFINDEX)\n");

    // When sending packets it is enough to specify sll_family, sll_addr, sll_halen, sll_ifindex
    // The other fields should be 0.
    // On reception, sll_hatype and sll_pkttype are set
    ethSendSockAddr.sll_family = AF_PACKET;
    ethSendSockAddr.sll_protocol = htons(ETH_P_IPV6);
    ethSendSockAddr.sll_ifindex = ifReq.ifr_ifindex;
    ethSendSockAddr.sll_halen = 6;*/

    return 0;
}

void printContikiConfigurationInfo(void) {
#if NETSTACK_CONF_WITH_IPV6
#if UIP_CONF_IPV6_RPL
	printf(CONTIKI_VERSION_STRING " started with IPV6, RPL\n");
#else
	printf(CONTIKI_VERSION_STRING " started with IPV6\n");
#endif
#else
	printf(CONTIKI_VERSION_STRING " started\n");
#endif

#ifdef WITH_6LOWPAN
	printf("WITH_6LOWPAN = %i\n",WITH_6LOWPAN );
#else
	printf("WITH_6LOWPAN undefined\n");
#endif

#ifdef SICSLOWPAN_COMPRESSION
	printf("SICSLOWPAN_COMPRESSION = %i\n",SICSLOWPAN_COMPRESSION);
#else
	printf("SICSLOWPAN_COMPRESSION undefined\n");
#endif

printf("MAC %s RDC %s NETWORK %s\n", NETSTACK_MAC.name, NETSTACK_RDC.name, NETSTACK_NETWORK.name);
}


void test_rx(const uint8_t *data, uint16_t dataLength) {
	printf("rx_hook:Received %i bytes : ",dataLength);
	for(int i = 0; i < dataLength; ++i) {

		printf("%02X", data[i]);
	}
	printf("%s","\n");
}

void test_tx(const uint8_t *data, uint16_t dataLength) {
	printf("tx_hook:Sending %i bytes : ",dataLength);
	for(int i = 0; i < dataLength; ++i) {

		printf("%02X", data[i]);
	}
	printf("%s","\n");
}

void test_br_rx(const uint8_t *data, uint16_t dataLength) {
  printf("rx_br_hook:Received %i bytes : ",dataLength);
  for(int i = 0; i < dataLength; ++i) {

    printf("%02X", data[i]);
  }
  printf("%s","\n");
}

void test_br_tx(const uint8_t *data, uint16_t dataLength) {
  printf("tx__br_hook:Sending %i bytes : ",dataLength);
  for(int i = 0; i < dataLength; ++i) {

    printf("%02X", data[i]);
  }
  printf("%s","\n");
}

/*
 * Copyright (c) 2013, CETIC.
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
 */

/**
 * \file
 *         Packet Filter for 6LBR.
 *         Enables dual-interfaces (IEEE802.15.4 and Ethernet) under the
 *         single-interface uIP stack.
 *         More information: 
 *           https://github.com/cetic/6lbr/wiki/Implementation-Details
 * \author
 *         6LBR Team <6lbr@cetic.be>
 */

#define LOG6LBR_MODULE "PF"

#include "contiki-net.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ipv6/uip-nd6.h"
#include "string.h"

#include "cetic-6lbr.h"
#include "log-6lbr.h"

#include "eth-drv.h"
#include "ip/tcpip.h"
#include "ip/uip.h"

#include "ule6lo_ipei_defines.h"

extern const linkaddr_t linkaddr_null;

static int eth_output(const uip_lladdr_t * src, const uip_lladdr_t * dest);
static uint8_t ethernet_has_fcs =0;

/*---------------------------------------------------------------------------*/


static outputfunc_t wireless_outputfunc;
static inputfunc_t tcpip_inputfunc;

#define BUF ((struct uip_eth_hdr *)&ll_header[0])

#define UIP_IP_BUF ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UIP_ICMP_BUF                      ((struct uip_icmp_hdr *)&uip_buf[uip_l2_l3_hdr_len])
#define UIP_ND6_NS_BUF            ((uip_nd6_ns *)&uip_buf[uip_l2_l3_icmp_hdr_len])
#define UIP_ND6_NA_BUF            ((uip_nd6_na *)&uip_buf[uip_l2_l3_icmp_hdr_len])
#define UIP_UDP_BUF                        ((struct uip_udp_hdr *)&uip_buf[UIP_LLH_LEN + UIP_IPH_LEN])
#define UIP_ICMP_PAYLOAD ((unsigned char *)&uip_buf[uip_l2_l3_icmp_hdr_len])

#define IS_BROADCAST_ADDR(a) ((a)==NULL || linkaddr_cmp((linkaddr_t *)(a), &linkaddr_null) != 0)

/*---------------------------------------------------------------------------*/

static void
send_to_uip(void)
{
	if(tcpip_inputfunc != NULL) {
		tcpip_inputfunc();
	} else {
		LOG6LBR_WARN("No input function set\n");
	}
}
/*---------------------------------------------------------------------------*/

static void
wireless_input(void)
{
	LOG6LBR_PRINTF(PACKET, PF_IN, "wireless_input: processing frame\n");
	send_to_uip();
}

uint8_t
wireless_output(const uip_lladdr_t * src, const uip_lladdr_t * dest)
{
	int ret;

	//Packet filtering
	//----------------
	if(uip_len == 0) {
		LOG6LBR_ERROR("wireless_output: uip_len = 0\n");
		return 0;
	}
	if(dest && linkaddr_cmp((linkaddr_t *) & dest,
			(linkaddr_t *) & wsn_mac_addr)) {
		LOG6LBR_ERROR("wireless_output: sending to self\n");
		return 0;
	}

	//Packet sending
	//--------------
	if(wireless_outputfunc != NULL) {
		LOG6LBR_PRINTF(PACKET, PF_OUT, "wireless_output: sending packet\n");
		ret = wireless_outputfunc(dest);
	} else {
		ret = 0;
	}
	return ret;
}

/*---------------------------------------------------------------------------*/

void
eth_input(void)
{
	int processFrame = 0;

	//Packet type filtering
	//---------------------
	//Keep only IPv6 traffic
	if(BUF->type != UIP_HTONS(UIP_ETHTYPE_IPV6)) {
		LOG6LBR_PRINTF(PACKET, PF_IN, "eth_input: Dropping packet type=0x%04x\n", uip_ntohs(BUF->type));
		uip_len = 0;
		return;
	}
	if((UIP_IP_BUF->len[0] << 8) + UIP_IP_BUF->len[1] + 40 < uip_len) {
#if CONTIKI_TARGET_NATIVE
		if(ethernet_has_fcs) {
			uip_len = (UIP_IP_BUF->len[0] << 8) + UIP_IP_BUF->len[1] + 40;
		} else
#endif
		{
			LOG6LBR_PRINTF(PACKET, PF_IN, "eth_input: packet size different than reported in IPv6 header, %d vs %d\n", uip_len, (UIP_IP_BUF->len[0] << 8) + UIP_IP_BUF->len[1] + 40);
		}
	} else if((UIP_IP_BUF->len[0] << 8) + UIP_IP_BUF->len[1] + 40 > uip_len) {
		LOG6LBR_PRINTF(PACKET, PF_IN, "eth_input: packet shorter than reported in IPv6 header, %d vs %d\n", uip_len, (UIP_IP_BUF->len[0] << 8) + UIP_IP_BUF->len[1] + 40);
		uip_len = 0;
		return;
	}

	//Packet source Filtering
	//-----------------------
	/* IPv6 uses 33-33-xx-xx-xx-xx prefix for multicast ND stuff */
	if((BUF->dest.addr[0] == 0x33) && (BUF->dest.addr[1] == 0x33)) {
		processFrame = 1;
	} else if((BUF->dest.addr[0] == 0xFF)
			&& (BUF->dest.addr[1] == 0xFF)
			&& (BUF->dest.addr[2] == 0xFF)
			&& (BUF->dest.addr[3] == 0xFF)
			&& (BUF->dest.addr[4] == 0xFF)
			&& (BUF->dest.addr[5] == 0xFF)) {
		/* IPv6 does not use broadcast addresses, hence this should not happen */
		LOG6LBR_PRINTF(PACKET, PF_IN, "eth_input: Dropping broadcast packet\n");
		uip_len = 0;
		return;
	}

	//Destination filtering
	//---------------------
	if(memcmp((uint8_t *) & eth_mac_addr, BUF->dest.addr, 6) == 0) {
		processFrame = 1;
	} else {
		if(!processFrame) {
			// Also search our neighbor list to see if we should accept it
			if(uip_ds6_nbr_ll_lookup((const uip_lladdr_t*)BUF->dest.addr)) {
				processFrame = 1;
			}
		}
	}

	//Handle packet
	//-------------
	if(processFrame) {
		LOG6LBR_PRINTF(PACKET, PF_IN, "eth_input: Processing frame\n");
		send_to_uip();
	} else {
		printf("eth_input: Dropping frame, not for us or any known node\n");
		//Drop packet
		uip_len = 0;
	}
}

static int
eth_output(const uip_lladdr_t * src, const uip_lladdr_t * dest)
{
	//Packet filtering
	//----------------
	if(uip_len == 0) {
		LOG6LBR_ERROR("eth_output: uip_len = 0\n");
		return 0;
	}

	if(dest && linkaddr_cmp((linkaddr_t *) & dest,
			(linkaddr_t *) & eth_mac64_addr)) {
		LOG6LBR_ERROR("ethernet_output: sending to self\n");
		return 0;
	}

	//Create packet header
	//--------------------
	//Packet type
	BUF->type = uip_htons(UIP_ETHTYPE_IPV6);

	//Destination address
	if(IS_BROADCAST_ADDR(dest)) {
		BUF->dest.addr[0] = 0x33;
		BUF->dest.addr[1] = 0x33;
		BUF->dest.addr[2] = UIP_IP_BUF->destipaddr.u8[12];
		BUF->dest.addr[3] = UIP_IP_BUF->destipaddr.u8[13];
		BUF->dest.addr[4] = UIP_IP_BUF->destipaddr.u8[14];
		BUF->dest.addr[5] = UIP_IP_BUF->destipaddr.u8[15];
	} else {
		memcpy(BUF->dest.addr, dest, 6);
	}

	//Source address
	if ( src != NULL ) {
		memcpy(BUF->src.addr, src, 6);
	} else {
		memcpy(BUF->src.addr, eth_mac_addr, 6);
	}
	//Sending packet
	//--------------
	LOG6LBR_PRINTF(PACKET, PF_OUT, "eth_output: Sending packet to Ethernet\n");
	eth_drv_send();

	return 1;
}

/*---------------------------------------------------------------------------*/

#if FILTERED_BORDER_ROUTER

static uint8_t
bridge_output(const uip_lladdr_t * dest)
{
	uint8_t isEthOutput = UIP_TRANSMITTED_INTERFACE_TYPE & UIP_ETH_INTERFACE_MASK;
	uint8_t isWirelessOutput = UIP_TRANSMITTED_INTERFACE_TYPE & UIP_LN_INTERFACE_MASK;

	//Filter WSN vs Ethernet segment traffic
	if(isEthOutput) {
		eth_output(NULL, dest);
	}
	if( isWirelessOutput) {
		wireless_output(NULL, dest);
	}
	return 0;
}
#endif

/*---------------------------------------------------------------------------*/

void
packet_filter_init(void)
{
	wireless_outputfunc = tcpip_get_outputfunc();
	tcpip_set_outputfunc(bridge_output);

	tcpip_inputfunc = tcpip_get_inputfunc();

	tcpip_set_inputfunc(wireless_input);
}
/*---------------------------------------------------------------------------*/

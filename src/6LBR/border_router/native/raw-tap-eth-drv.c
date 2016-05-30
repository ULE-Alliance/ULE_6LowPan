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
 * \author
 *         6LBR Team <6lbr@cetic.be>
 */

#define LOG6LBR_MODULE "ETH"

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ip/uip.h"
#include "string.h"

#include "log-6lbr.h"
#include "cetic-6lbr.h"
#include "eth-drv.h"
#include "raw-tap-dev.h"
#include "packet-filter.h"
#include "ule6lo_ipei_defines.h"
#include "ule6loTestIn_6LBR.h"

#define UIP_IP_BUF ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

PROCESS(eth_drv_process, "RAW/TAP Ethernet Driver");

//#if UIP_CONF_LLH_LEN == 0
uint8_t ll_header[ETHERNET_LLH_LEN];
//#endif

uint32_t ule6lo_border_router_rx_package_counter =0;
uint32_t ule6lo_border_router_tx_package_counter =0;

/*---------------------------------------------------------------------------*/

static unsigned char tmp_tap_buf[ETHERNET_LLH_LEN + UIP_BUFSIZE - UIP_LLH_LEN];
void
eth_drv_send(void)
{
  //Should remove ll_header
  memcpy(tmp_tap_buf, ll_header, ETHERNET_LLH_LEN);
  memcpy(tmp_tap_buf + ETHERNET_LLH_LEN, uip_buf+UIP_LLH_LEN, uip_len);

  LOG6LBR_PRINTF(PACKET, ETH_OUT, "write: %d\n", uip_len + ETHERNET_LLH_LEN);
  if (LOG6LBR_COND(DUMP, ETH_OUT)) {
    int i;
#if WIRESHARK_IMPORT_FORMAT
    printf("0000");
    for(i = 0; i < uip_len + ETHERNET_LLH_LEN; i++)
      printf(" %02x", tmp_tap_buf[i]);
#else
    printf("         ");
    for(i = 0; i < uip_len + ETHERNET_LLH_LEN; i++) {
      printf("%02x", tmp_tap_buf[i]);
      if((i & 3) == 3)
        printf(" ");
      if((i & 15) == 15)
        printf("\n         ");
    }
#endif
    printf("\n");
  }

  ule6loNI_send(tmp_tap_buf, uip_len + ETHERNET_LLH_LEN);
  ule6lo_border_router_tx_package_counter++;
    if(ule6lo_border_router_txHook!=NULL) {
      ule6lo_border_router_txHook(tmp_tap_buf, uip_len + ETHERNET_LLH_LEN);
    }
}

void
eth_drv_input(const uint8_t*  data, uint16_t dataLength)
{
  //Extract the received information
  uip_len = dataLength - ETHERNET_LLH_LEN;
  memcpy(ll_header, data, ETHERNET_LLH_LEN);
  memcpy(UIP_IP_BUF, data + ETHERNET_LLH_LEN, uip_len);
  //There is no ipei when coming from the ethernet.
  UIP_WRITE_RECEIVED_IPEI_UNKNOWN();
  *UIP_RECEIVED_FLAGS=UIP_ETH_INTERFACE_MASK;

  //Print debug if needed.
  LOG6LBR_PRINTF(PACKET, ETH_IN, "read: %d\n", uip_len + ETHERNET_LLH_LEN);
  if (LOG6LBR_COND(DUMP, ETH_IN)) {
    int i;
#if WIRESHARK_IMPORT_FORMAT
    printf("0000");
    for(i = 0; i < ETHERNET_LLH_LEN; i++)
      printf(" %02x", ll_header[i]);
    for(i = 0; i < uip_len; i++)
      printf(" %02x", UIP_IP_BUF[i]);
#else
    printf("         ");
    for(i = 0; i < uip_len + ETHERNET_LLH_LEN; i++) {
      if ( i < ETHERNET_LLH_LEN ) {
        printf("%02x", ll_header[i]);
      } else {
        printf("%02x", ((uint8_t*)(UIP_IP_BUF))[i - ETHERNET_LLH_LEN]);
      }
      if((i & 3) == 3)
        printf(" ");
      if((i & 15) == 15)
        printf("\n         ");
    }
#endif
    printf("\n");
  }

  //Process the data
  eth_input();
}

void
eth_drv_exit(void)
{
}

void
eth_drv_init()
{
  LOG6LBR_INFO("RAW/TAP init\n");

  ule6loNI_init();
}

/*---------------------------------------------------------------------------*/

PROCESS_THREAD(eth_drv_process, ev, data)
{
	PROCESS_BEGIN();

	eth_drv_init();
	static struct etimer et;
	//slip_reboot();
	while(!mac_set) {
		etimer_set(&et, CLOCK_SECOND);
		//slip_request_mac();
		printf("Waiting for MAC address\n");
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
	}

	//We must create our own Ethernet MAC address
	memcpy(eth_mac_addr, &wsn_mac_addr, 6);
	LOG6LBR_ETHADDR(INFO, &eth_mac_addr, "Eth MAC address : ");
	eth_mac_addr_ready = 1;
	ethernet_ready = 1;

	PROCESS_END();
}

/*---------------------------------------------------------------------------*/

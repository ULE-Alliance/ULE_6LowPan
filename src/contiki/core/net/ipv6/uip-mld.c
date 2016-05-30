/**
 * \addtogroup uip6
 * @{
 */

/**
 * \file
 *         MLDv1 multicast registration handling (RFC 2710)
 * \author Phoebe Buckheister <phoebe.buckheister@itwm.fhg.de> 
 */

/*
 * Copyright (c) 2014, Fraunhofer ITWM
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "net/ipv6/uip-mld.h"

#if UIP_CONF_MLD

#include "net/ipv6/uip-ds6.h"
#include "net/ipv6/uip-icmp6.h"
#include "net/ip/tcpip.h"
#include "lib/random.h"
#include "net/ipv6/uip-ds6-nbr.h"

#define DEBUG DEBUG_FULL
#include "net/ip/uip-debug.h"

#define UIP_IP_BUF                ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
#define UIP_ICMP_BUF            ((struct uip_icmp_hdr *)&uip_buf[uip_l2_l3_hdr_len])
#define UIP_ICMP6_ERROR_BUF  ((struct uip_icmp6_error *)&uip_buf[uip_l2_l3_icmp_hdr_len])
#define UIP_ICMP6_MLD_BUF  ((struct uip_icmp6_mld1 *)&uip_buf[uip_l2_l3_icmp_hdr_len])

struct etimer uip_mld_timer_periodic;

static void sendMldReports(uint16_t max_delay);

static inline void
mld_report_later(uip_ds6_maddr_t * addr, uint16_t timeout)
{
  uint16_t when = 0;
  if( timeout != 0 ){
    when = random_rand() % timeout;
  }

  PRINTF("Report in %is:", when);
  PRINT6ADDR(&addr->ipaddr);
  PRINTF("\n");
  stimer_set(&addr->report_timeout, when);
}
#define UIP_HBHO_LEN 2
#define UIP_PADN_LEN 2
#define UIP_RTR_ALERT_LEN 4
/*---------------------------------------------------------------------------*/
static void
send_mldv1_packet(uip_ip6addr_t * maddr, uint8_t mld_type)
{
  /* IP header */
  /* MLD requires hoplimits to be 1 and source addresses to be link-local.
   * Since routers must send queries from link-local addresses, a link local
   * source be selected.
   * The destination IP must be the multicast group, though, and source address selection
   * will choose a routable address (if available) for multicast groups that are themselves
   * routable. Thus, select the source address before filling the destination.
   **/
  UIP_IP_BUF->ttl = 1;
  uip_ds6_select_src(&UIP_IP_BUF->srcipaddr, &UIP_IP_BUF->destipaddr);
  /* If the selected source is ::, the MLD packet would be invalid. */
  if(uip_is_addr_unspecified(&UIP_IP_BUF->destipaddr)) {
    return;
  }

  if(mld_type == ICMP6_ML_REPORT) {
    uip_ipaddr_copy(&UIP_IP_BUF->destipaddr, maddr);
  } 
  else{
    uip_create_linklocal_allrouters_mcast(&UIP_IP_BUF->destipaddr);
  }

  UIP_IP_BUF->proto = UIP_PROTO_HBHO;
  uip_len = UIP_LLH_LEN + UIP_IPH_LEN;

  ((uip_hbho_hdr *) & uip_buf[uip_len])->next = UIP_PROTO_ICMP6;
  /* we need only pad with two bytes, so the PadN header is sufficient */
  /* also, len is in units of eight octets, excluding the first. */
  ((uip_hbho_hdr *) & uip_buf[uip_len])->len =
    (UIP_HBHO_LEN + UIP_RTR_ALERT_LEN + UIP_PADN_LEN) / 8 - 1;
  uip_len += UIP_HBHO_LEN;//2

  ((uip_ext_hdr_rtr_alert_tlv *) & uip_buf[uip_len])->tag =
    UIP_EXT_HDR_OPT_RTR_ALERT;
  ((uip_ext_hdr_rtr_alert_tlv *) & uip_buf[uip_len])->len = 2;  /* data length of value field */
  ((uip_ext_hdr_rtr_alert_tlv *) & uip_buf[uip_len])->value = 0;        /* MLD message */
  uip_len += UIP_RTR_ALERT_LEN;//4

  ((uip_ext_hdr_padn_tlv *) & uip_buf[uip_len])->tag = UIP_EXT_HDR_OPT_PADN;
  ((uip_ext_hdr_padn_tlv *) & uip_buf[uip_len])->len = 0;       /* no data bytes following */
  uip_len += UIP_PADN_LEN;//2

  uip_len += UIP_ICMPH_LEN;

  uip_ext_len = UIP_HBHO_LEN + UIP_RTR_ALERT_LEN + UIP_PADN_LEN;
  uip_len += UIP_ICMP6_MLD1_LEN;

  UIP_IP_BUF->len[0] = ((uip_len - UIP_IPH_LEN) >> 8);
  UIP_IP_BUF->len[1] = ((uip_len - UIP_IPH_LEN) & 0xff);
  UIP_ICMP_BUF->type = mld_type;
  UIP_ICMP_BUF->icode = 0;

  UIP_ICMP6_MLD_BUF->maximum_delay = 0;
  UIP_ICMP6_MLD_BUF->reserved = 0;
  uip_ipaddr_copy(&UIP_ICMP6_MLD_BUF->address, maddr);

  UIP_ICMP_BUF->icmpchksum = 0;
  UIP_ICMP_BUF->icmpchksum = ~uip_icmp6chksum();

  tcpip_ipv6_output();
  UIP_STAT(++uip_stat.icmp.sent);
}

/*---------------------------------------------------------------------------*/
void
uip_icmp6_mldv1_schedule_report(uip_ds6_maddr_t * addr)
{
#if LN_ROLE
  //no need to send reports when we are not connetced to LBR..
  if( uip_ds6_nbr_num() != 0)
#endif
  {
    addr->report_count = MLD_INITIAL_REPORT_COUNT;
    stimer_set(&addr->report_timeout, 0);
    etimer_set(&uip_mld_timer_periodic, CLOCK_SECOND / 4);
  }
}

/**
 * Send reports for all multicast addresses in node.
 */
void
uip_mldv1_schedule_report_all_addr(void){
  uint8_t m;
  uip_ds6_maddr_t *addr;
  uint8_t newmaddr = 0;
  for(m = 0; m < UIP_DS6_MADDR_NB; m++) {
      addr = &uip_ds6_if.maddr_list[m];
      if(addr->isused) {
        addr->report_count = MLD_INITIAL_REPORT_COUNT;
        stimer_set(&addr->report_timeout, 0);
        newmaddr = 1;
      }
  }
  if( newmaddr ){
    etimer_set(&uip_mld_timer_periodic, CLOCK_SECOND / 4);
  }
}
/*---------------------------------------------------------------------------*/
void
uip_icmp6_mldv1_report(uip_ip6addr_t * addr)
{
  if(uip_is_addr_linklocal_allnodes_mcast(addr)) {
    PRINTF("Not sending MLDv1 report for FF02::1\n");
    return;
  }

  PRINTF("Sending MLDv1 report for");
  PRINT6ADDR(addr);
  PRINTF("\n");
#if LBR_ROLE
  //do not send reports to ln at lbr
  *UIP_TRANSMITTED_FLAGS = UIP_INTERFACE_TYPE_ETH;
#endif
  send_mldv1_packet(addr, ICMP6_ML_REPORT);
}

/*---------------------------------------------------------------------------*/
/*ONLY TO TESTS QURIES. Not needed in our setup*/
void
uip_icmp6_mldv1_query(uip_ip6addr_t * addr)
{
  PRINTF("Sending MLDv1 query for");
  PRINT6ADDR(addr);
  PRINTF("\n");

  send_mldv1_packet(addr, ICMP6_ML_QUERY);
}

/*---------------------------------------------------------------------------*/
void
uip_icmp6_mldv1_done(uip_ip6addr_t * addr)
{
  if(uip_is_addr_linklocal_allnodes_mcast(addr)) {
    PRINTF("Not sending MLDv1 done for FF02::1\n");
    return;
  }

  PRINTF("Sending MLDv1 done for");
  PRINT6ADDR(addr);
  PRINTF("\n");

  send_mldv1_packet(addr, ICMP6_ML_DONE);
}

/*---------------------------------------------------------------------------*/
void
uip_icmp6_ml_query_input(void)
{
  uip_ds6_maddr_t *addr;
  uint16_t max_delay;

  /*
   * Send an MLDv1 report packet for every multicast address known to be ours.
   */
  PRINTF("Received MLD query from");
  PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
  PRINTF("for");
  PRINT6ADDR(&UIP_ICMP6_MLD_BUF->address);
  PRINTF("Query address");
  PRINT6ADDR(&UIP_ICMP6_MLD_BUF->address);
  PRINTF("\n");

#if (LBR_ROLE)
  max_delay = uip_ntohs(UIP_ICMP6_MLD_BUF->maximum_delay);

  if(uip_ext_len == 0) {
    PRINTF("MLD packet without hop-by-hop header received\n");
    return;
  } else {
    if(!uip_is_addr_linklocal_allnodes_mcast(&UIP_ICMP6_MLD_BUF->address) && !uip_is_addr_unspecified(&UIP_ICMP6_MLD_BUF->address)
       && uip_ds6_is_my_maddr(&UIP_ICMP6_MLD_BUF->address)) {
      addr = uip_ds6_maddr_lookup(&UIP_ICMP6_MLD_BUF->address);
      PRINTF("specified\n");
      if( addr != NULL){
        addr->report_count = 1;
        mld_report_later(addr, max_delay / 1000);
      }
    } else if(uip_is_addr_unspecified(&UIP_ICMP6_MLD_BUF->address)) {
      PRINTF("unspecified\n");
      sendMldReports(max_delay);
    }
  }

  etimer_set(&uip_mld_timer_periodic, CLOCK_SECOND / 4);
#endif
  uip_len = 0;
}
/**
 * Send mld reports to the network for all addresses. Only once for each address.
 */
static void sendMldReports(uint16_t max_delay){
#if (LBR_ROLE)
  for( uint16_t i = 0; i < UIP_DS6_MADDR_NBS; i++){
    if(  uip_ds6_if.maddr_list[i].isused){
        uip_ds6_if.maddr_list[i].report_count = 1;
        mld_report_later(&uip_ds6_if.maddr_list[i], max_delay / 1000);
    }
  }
#endif
}
/**
 * Handle done reports from nodes.
 */
void
uip_icmp6_ml_done_input(void)
{
  uip_ds6_maddr_t *addr;
  uip_ds6_nbr_t *nbr;
  PRINTF("Received MLD done report from");
  PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
  PRINTF("for");
  PRINT6ADDR(&UIP_IP_BUF->destipaddr);
  PRINTF("Remove address");
  PRINT6ADDR(&UIP_ICMP6_MLD_BUF->address);
  PRINTF("\n");
  //remove from neighbour list
  if( (nbr = uip_ds6_nbr_lookup(&UIP_IP_BUF->srcipaddr)) != NULL) {
    uip_ds6_nbr_maddr_rm(nbr, UIP_ICMP6_MLD_BUF);
  }
  uip_len = 0;
}
/*---------------------------------------------------------------------------*/
void
uip_icmp6_ml_report_input(void)
{
  uip_ds6_maddr_t *addr;
  uip_ds6_nbr_t *nbr;
  PRINTF("Received MLD report from");
  PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
  PRINTF("for");
  PRINT6ADDR(&UIP_IP_BUF->destipaddr);
  PRINTF("\n");

  if(uip_ext_len == 0) {
    PRINTF("MLD packet without hop-by-hop header received\n");
  } 
#if LN_ROLE
  else if(uip_ds6_is_my_maddr(&UIP_ICMP6_MLD_BUF->address)) {
    addr = uip_ds6_maddr_lookup(&UIP_ICMP6_MLD_BUF->address);
    if(addr->report_count > 0)
      addr->report_count--;
    PRINTF("IS my maddr- report count!! %d\n", addr->report_count);
  }
#endif
  else if( (nbr = uip_ds6_nbr_lookup(&UIP_IP_BUF->srcipaddr)) != NULL) {
    uip_ds6_nbr_maddr_add(nbr, UIP_ICMP6_MLD_BUF);
  }
  uip_len = 0;

}

/*---------------------------------------------------------------------------*/
void
uip_mld_periodic(void)
{
  uint16_t m;
  uip_ds6_maddr_t *addr;
  bool more = false;

  for(m = 0; m < UIP_DS6_MADDR_NB; m++) {
    addr = &uip_ds6_if.maddr_list[m];
    if(addr->isused && addr->report_count) {
      if(stimer_expired(&addr->report_timeout)) {
        uip_icmp6_mldv1_report(&addr->ipaddr);
        if(--addr->report_count) {
          if(addr->report_timeout.interval == 0){
            mld_report_later(addr, UIP_IP6_MLD_REPORT_INTERVAL);
          }
          stimer_restart(&addr->report_timeout);
        }
      }
      more = true;
    }
  }
  if(more)
    etimer_set(&uip_mld_timer_periodic, CLOCK_SECOND / 4);

}

/** @} */

#endif
/*
 * Copyright (c) 2013, Swedish Institute of Computer Science.
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
 *
 */

/**
 * \addtogroup uip6
 * @{
 */

/**
 * \file
 *    IPv6 Neighbor cache (link-layer/IPv6 address mapping)
 * \author Mathilde Durvy <mdurvy@cisco.com>
 * \author Julien Abeille <jabeille@cisco.com>
 * \author Simon Duquennoy <simonduq@sics.se>
 *
 */

#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include "lib/list.h"
#include "net/linkaddr.h"
#include "net/packetbuf.h"
#include "net/ipv6/uip-ds6-nbr.h"
#include "net/ipv6/uip-ds6.h"
#include "ule6lo_ipei_defines.h"
#include "net/ipv6/sicslowpan.h"
#include "net/ipv6/uip-icmp6.h"
#if UIP_CONF_MLD
#include "net/ipv6/uip-mld.h"
#endif

#define DEBUG DEBUG_FULL
#include "net/ip/uip-debug.h"

#ifdef UIP_CONF_DS6_NEIGHBOR_STATE_CHANGED
#define NEIGHBOR_STATE_CHANGED(n) UIP_CONF_DS6_NEIGHBOR_STATE_CHANGED(n)
void NEIGHBOR_STATE_CHANGED(uip_ds6_nbr_t *n);
#else
#define NEIGHBOR_STATE_CHANGED(n)
#endif /* UIP_DS6_CONF_NEIGHBOR_STATE_CHANGED */

#ifdef UIP_CONF_DS6_LINK_NEIGHBOR_CALLBACK
#define LINK_NEIGHBOR_CALLBACK(addr, status, numtx) UIP_CONF_DS6_LINK_NEIGHBOR_CALLBACK(addr, status, numtx)
void LINK_NEIGHBOR_CALLBACK(const linkaddr_t *addr, int status, int numtx);
#else
#define LINK_NEIGHBOR_CALLBACK(addr, status, numtx)
#endif /* UIP_CONF_DS6_LINK_NEIGHBOR_CALLBACK */

NBR_TABLE_GLOBAL(uip_ds6_nbr_t, ds6_neighbors);

/*---------------------------------------------------------------------------*/
void
uip_ds6_neighbors_init(void)
{
  nbr_table_register(ds6_neighbors, (nbr_table_callback *)uip_ds6_nbr_rm);
}
/*---------------------------------------------------------------------------*/
#if FILTERED_BORDER_ROUTER
uip_ds6_nbr_t *
uip_ds6_nbr_add(const uip_ipaddr_t *ipaddr, const uip_lladdr_t *lladdr,
                uint8_t isrouter, uint8_t state, const ule6lo_IPEI_t *ipei, uint8_t interface_type)
#else
uip_ds6_nbr_t *
uip_ds6_nbr_add(const uip_ipaddr_t *ipaddr, const uip_lladdr_t *lladdr,
                uint8_t isrouter, uint8_t state, const ule6lo_IPEI_t *ipei)
#endif
{
  uint8_t updateCurrent = 1;
#if (UIP_CONF_MULTIPLE_PREFIXES & LBR_ROLE)
  int index;
  uip_ds6_nbr_t *item;
  if((item = uip_ds6_nbr_ll_lookup(lladdr)) != NULL) {
    //add new prefix if not the same entry..
    if( uip_ipaddr_cmp(&item->ipaddr, ipaddr) || state != NBR_REGISTERED || item->state != NBR_REGISTERED){
      updateCurrent = 1;
    }
    else{
      //update with new prefix list
      uint8_t i;
      uip_Prefix_ip6addr_t emptyprefix;

      updateCurrent = 0;
      memset( &emptyprefix, 0x00, sizeof(emptyprefix));
      for( i = 0; i < UIP_DS6_MULTIPLE_PREFIXES_MAX; i++){
        if( memcmp(&item->m_prefix[i], &emptyprefix ,sizeof(uip_Prefix_ip6addr_t) ) == 0){
          memcpy(&item->m_prefix[i], ipaddr, sizeof(uip_Prefix_ip6addr_t));
          PRINTF("Prefix  added\n");
         // PRINTA("IPEI address = %02X:%02X:%02X:%02X:%02X\n",tem->m_prefix[0],ipei->id[1],ipei->id[2],ipei->id[3],ipei->id[4]);
          break;
        }
      }
      if( i == UIP_DS6_MULTIPLE_PREFIXES_MAX){
        PRINTF("No space for prefixes");
      }
      return item;
    }
  }
#endif
  if( updateCurrent ){
    uip_ds6_nbr_t *nbr = nbr_table_add_lladdr(ds6_neighbors, (linkaddr_t*)lladdr);
    if(nbr) {
      if(ipei!=NULL) {
        memcpy(&nbr->ipei,ipei,sizeof(ule6lo_IPEI_t));
      }
      nbr->state = state;
  #if FILTERED_BORDER_ROUTER
      nbr->interface_type = interface_type;
      if( interface_type == UIP_INTERFACE_TYPE_DECT ){
          stimer_set(&nbr->reachable, UIP_NBR_TENTATIVE_TIME); 
      }else{
        stimer_set(&nbr->reachable, 0);
      }
  #else
      //set a tentative time to let it receive a na..
      stimer_set(&nbr->reachable, UIP_NBR_TENTATIVE_TIME);
  #endif
      uip_ipaddr_copy(&nbr->ipaddr, ipaddr);
      nbr->isrouter = isrouter;
      
    #if UIP_CONF_IPV6_QUEUE_PKT
      uip_packetqueue_new(&nbr->packethandle);
    #endif /* UIP_CONF_IPV6_QUEUE_PKT */
      /* timers are set separately, for now we put them in expired state */
      stimer_set(&nbr->sendns, 0);
      nbr->nscount = 0;
  #if (UIP_CONF_MLD & LBR_ROLE)
  #if UIP_DS6_MADDR_NBR_MAX > 0
      memset(nbr->maddr_list_index, 0xFF, sizeof(nbr->maddr_list_index));
  #endif
  #endif
#if (UIP_CONF_MULTIPLE_PREFIXES & LBR_ROLE)
      memset(nbr->m_prefix, 0x00, sizeof(nbr->m_prefix));// no need to set the first prefix, as this is included in the ipaddr..
#endif
      PRINTF("Adding neighbor with ip addr ");
      PRINT6ADDR(ipaddr);
      PRINTF(" link addr ");
      PRINTLLADDR(lladdr);
      PRINTF(" state %u\n", state);
      NEIGHBOR_STATE_CHANGED(nbr);
    
      return nbr;
    } else {
      PRINTF("uip_ds6_nbr_add drop ip addr ");
      PRINT6ADDR(ipaddr);
      PRINTF(" link addr (%p) ", lladdr);
      PRINTLLADDR(lladdr);
      PRINTF(" state %u\n", state);
      return NULL;
    }
  }
}

/*---------------------------------------------------------------------------*/
void
uip_ds6_nbr_rm(uip_ds6_nbr_t *nbr)
{
  if(nbr != NULL) {
#if UIP_CONF_IPV6_QUEUE_PKT
    uip_packetqueue_free(&nbr->packethandle);
#endif /* UIP_CONF_IPV6_QUEUE_PKT */
    NEIGHBOR_STATE_CHANGED(nbr);
    nbr_table_remove(ds6_neighbors, nbr);
  }
  return;
}

/*---------------------------------------------------------------------------*/
const uip_ipaddr_t *
uip_ds6_nbr_get_ipaddr(const uip_ds6_nbr_t *nbr)
{
  return (nbr != NULL) ? &nbr->ipaddr : NULL;
}

/*---------------------------------------------------------------------------*/
const uip_lladdr_t *
uip_ds6_nbr_get_ll(const uip_ds6_nbr_t *nbr)
{
  return (const uip_lladdr_t *)nbr_table_get_lladdr(ds6_neighbors, nbr);
}

/*---------------------------------------------------------------------------*/
int
uip_ds6_nbr_num(void)
{
  uip_ds6_nbr_t *nbr;
  int num;

  num = 0;
  for(nbr = nbr_table_head(ds6_neighbors);
      nbr != NULL;
      nbr = nbr_table_next(ds6_neighbors, nbr)) {
    num++;
  }
  return num;
}
/*---------------------------------------------------------------------------*/
uip_ds6_nbr_t *
uip_ds6_nbr_lookup(const uip_ipaddr_t *ipaddr)
{
#if (UIP_CONF_MULTIPLE_PREFIXES & LBR_ROLE)
  uip_ds6_nbr_t *nbr = nbr_table_head(ds6_neighbors);
  if(ipaddr != NULL) {
    uip_ipaddr_t ipaddrPrefix;
    while(nbr != NULL) {
      if(uip_ipaddr_cmp(&nbr->ipaddr, ipaddr)) {
        return nbr;
      }else if( nbr->isrouter == 0){
        //check all prefixes for neighbor 
        memcpy(&ipaddrPrefix.u8[8], &nbr->ipaddr.u8[8], 8);//copy postfix.
        for ( uint8_t i = 0; i < UIP_DS6_MULTIPLE_PREFIXES_MAX; i++){
          memcpy(&ipaddrPrefix.u8[0], &nbr->m_prefix[i], 8);//copy prefix.
          if(uip_ipaddr_cmp(&ipaddrPrefix, ipaddr)) {
            PRINTF("Other prefix found\n");
            return nbr;
          }
        }
        //and check link local
        if( uip_is_addr_linklocal(ipaddr)){
          memcpy(&ipaddrPrefix.u8[0], ipaddr, 8);//copy prefix.
          memcpy(ipaddrPrefix.u8 + 9, &nbr->ipei, 2);
          ipaddrPrefix.u8[11] = 0xff;
          ipaddrPrefix.u8[12] = 0xfe;
          memcpy(ipaddrPrefix.u8 + 13, (uint8_t *) &nbr->ipei.id[2], 3);
          ipaddrPrefix.u8[8] = 0x02;
          if(uip_ipaddr_cmp(&ipaddrPrefix, ipaddr)) {
            PRINTF("Link local found\n");
            return nbr;
          }
        }
      }
      nbr = nbr_table_next(ds6_neighbors, nbr);
    }
  }
#else
  uip_ds6_nbr_t *nbr = nbr_table_head(ds6_neighbors);
    if(ipaddr != NULL) {
      uip_ipaddr_t ipaddrPrefix;
      while(nbr != NULL) {
        if(uip_ipaddr_cmp(&nbr->ipaddr, ipaddr)) {
          return nbr;
        }else if( nbr->isrouter == 0){
          // check link local
          memcpy(&ipaddrPrefix.u8[8], &nbr->ipaddr.u8[8], 8);//copy postfix.
          if( uip_is_addr_linklocal(ipaddr)){
            memcpy(&ipaddrPrefix.u8[0], ipaddr, 8);//copy prefix.
            memcpy(ipaddrPrefix.u8 + 9, &nbr->ipei, 2);
            ipaddrPrefix.u8[11] = 0xff;
            ipaddrPrefix.u8[12] = 0xfe;
            memcpy(ipaddrPrefix.u8 + 13, (uint8_t *) &nbr->ipei.id[2], 3);
            ipaddrPrefix.u8[8] = 0x02;
            if(uip_ipaddr_cmp(&ipaddrPrefix, ipaddr)) {
              PRINTF("Link local found\n");
              return nbr;
            }
          }
        }
        nbr = nbr_table_next(ds6_neighbors, nbr);
      }
    }
#endif
  return NULL;
}

#if (FILTERED_BORDER_ROUTER & UIP_CONF_MLD)
uip_ds6_nbr_t *
uip_ds6_nbr_lookup_solicited_mc(const uip_ipaddr_t *ipaddr)
{
	uip_ds6_nbr_t *nbr = nbr_table_head(ds6_neighbors);
	if(ipaddr != NULL && uip_is_addr_solicited_node(ipaddr)) {
		while(nbr != NULL) {
			if(ipaddr->u8[13] == nbr->ipaddr.u8[13] && ipaddr->u16[7] == nbr->ipaddr.u16[7]) {
				return nbr;
			}
			nbr = nbr_table_next(ds6_neighbors, nbr);
		}
	}
	return NULL;
}

uint16_t 
uip_ds6_nbr_lookup_mc_in_list(const uip_ipaddr_t *ipaddr)
{
#if UIP_CONF_MLD
  for( uint16_t index = 0; index < UIP_DS6_MADDR_NB; index++){
    if(uip_ds6_if.maddr_list[index].isused == 1 && uip_ipaddr_cmp(&uip_ds6_if.maddr_list[index].ipaddr, ipaddr)){
      return index;
    }
  }
#endif
  return MLD_INVALID_INDEX;
}
uint16_t 
uip_ds6_nbr_lookup_index_in_all_nbrlist(uint16_t index)
{
#if UIP_CONF_MLD
  if(index != MLD_INVALID_INDEX ) {
    uip_ds6_nbr_t *nbr = nbr_table_head(ds6_neighbors);
    while(nbr != NULL) {
      for( uint8_t i = 0; i < UIP_DS6_MADDR_NBR_MAX; i++){
        if(nbr->maddr_list_index[i] == index ){
          return index;
        }
      }
      nbr = nbr_table_next(ds6_neighbors, nbr);
    }
  }
#endif
  return MLD_INVALID_INDEX;
}
uint8_t 
uip_ds6_nbr_forward_mc_from_list(uint16_t index, uip_ds6_nbr_t *nbr)
{
#if UIP_CONF_MLD
  if(index != MLD_INVALID_INDEX) {
    if(nbr != NULL) {
      for( uint8_t i = 0; i < UIP_DS6_MADDR_NBR_MAX; i++){
        if(nbr->maddr_list_index[i] == index){
          return 1;
        }
      }
    }
  }
#endif
  return 0;
}
#endif

/*---------------------------------------------------------------------------*/
uip_ds6_nbr_t *
uip_ds6_nbr_ll_lookup(const uip_lladdr_t *lladdr)
{
  return nbr_table_get_from_lladdr(ds6_neighbors, (linkaddr_t*)lladdr);
}

/*---------------------------------------------------------------------------*/
void
uip_ds6_nbr_maddr_add(uip_ds6_nbr_t *nbr, uip_icmp6_mld1* maddr)
{
#if (UIP_CONF_MLD & LBR_ROLE)
  uip_ds6_maddr_t *ipaddr;
  if(nbr != NULL && maddr != NULL) {
    uint16_t index = uip_ds6_nbr_lookup_mc_in_list(&maddr->address);
    if( index == MLD_INVALID_INDEX){
      //add it to common list
      uip_ds6_maddr_t* locaddr = uip_ds6_maddr_add(&maddr->address);
      if( locaddr != NULL){
        // now it should be added
        index = uip_ds6_nbr_lookup_mc_in_list(&maddr->address);
      }
      
      PRINTF("New index %d\n", index);
    }
    if( index != MLD_INVALID_INDEX){
      for( uint8_t i = 0; i < UIP_DS6_MADDR_NBR_MAX; i++){
        if(  nbr->maddr_list_index[i] == index ){
          //do not add it again.
          break;
        }
        else if( nbr->maddr_list_index[i] == MLD_INVALID_INDEX)
        {
          PRINTF("Adding multicast addr from report to nbr with index %d", index);
          PRINT6ADDR( &maddr->address);
          PRINTF("\n");
          nbr->maddr_list_index[i] = index;
          break;
        }
      }
    }
  }
#endif
}

void
uip_ds6_nbr_maddr_rm(uip_ds6_nbr_t *nbr, uip_icmp6_mld1* maddr)
{
#if (UIP_CONF_MLD & LBR_ROLE)
  uip_ds6_maddr_t *ipaddr;
  if(nbr != NULL && maddr != NULL) {
    uint16_t index = uip_ds6_nbr_lookup_mc_in_list(&maddr->address);
    if( index != MLD_INVALID_INDEX){
      for( uint8_t i = 0; i < UIP_DS6_MADDR_NBR_MAX; i++){
         if( nbr->maddr_list_index[i] == index){
           nbr->maddr_list_index[i] = MLD_INVALID_INDEX;
           break;
         }
      }
      if( uip_ds6_nbr_lookup_index_in_all_nbrlist(index) == MLD_INVALID_INDEX){
        //send done to the network, since this was the last node listening to that address.
        uip_icmp6_mldv1_done(&maddr->address);
        //remove from list
        memset(&uip_ds6_if.maddr_list[index], 0x00, sizeof(uip_ds6_if.maddr_list[index]));
      }
      PRINTF("Removed multicast addr from done report");
      PRINT6ADDR( &maddr->address);
      PRINTF("\n");
    }
  }
#endif
}

uip_ds6_nbr_t *
uip_ds6_nbr_ipei_lookup(const ule6lo_IPEI_t *ipei)
{
	uip_ds6_nbr_t *nbr = nbr_table_head(ds6_neighbors);
	if(ipei != NULL) {
		while(nbr != NULL) {
			if(memcmp(&nbr->ipei, ipei,sizeof(ule6lo_IPEI_t))==0) {
				return nbr;
			}
			nbr = nbr_table_next(ds6_neighbors, nbr);
		}
	}
	return NULL;
}

/*---------------------------------------------------------------------------*/
uip_ipaddr_t *
uip_ds6_nbr_ipaddr_from_lladdr(const uip_lladdr_t *lladdr)
{
  uip_ds6_nbr_t *nbr = uip_ds6_nbr_ll_lookup(lladdr);
  return nbr ? &nbr->ipaddr : NULL;
}

/*---------------------------------------------------------------------------*/
const uip_lladdr_t *
uip_ds6_nbr_lladdr_from_ipaddr(const uip_ipaddr_t *ipaddr)
{
  uip_ds6_nbr_t *nbr = uip_ds6_nbr_lookup(ipaddr);
  return nbr ? uip_ds6_nbr_get_ll(nbr) : NULL;
}

const ule6lo_IPEI_t *uip_ds6_nbr_ipei_from_ipaddr(const uip_ipaddr_t *ipaddr) {
  uip_ds6_nbr_t *nbr = uip_ds6_nbr_lookup(ipaddr);
  return nbr ? &nbr->ipei : NULL;
}

/*---------------------------------------------------------------------------*/
void
uip_ds6_link_neighbor_callback(int status, int numtx)
{
  const linkaddr_t *dest = packetbuf_addr(PACKETBUF_ADDR_RECEIVER);
  if(linkaddr_cmp(dest, &linkaddr_null)) {
    return;
  }

  LINK_NEIGHBOR_CALLBACK(dest, status, numtx);

#if UIP_DS6_LL_NUD
  /* From RFC4861, page 72, last paragraph of section 7.3.3:
   *
   * 	"In some cases, link-specific information may indicate that a path to
   * 	a neighbor has failed (e.g., the resetting of a virtual circuit). In
   * 	such cases, link-specific information may be used to purge Neighbor
   * 	Cache entries before the Neighbor Unreachability Detection would do
   * 	so. However, link-specific information MUST NOT be used to confirm
   * 	the reachability of a neighbor; such information does not provide
   * 	end-to-end confirmation between neighboring IP layers."
   *
   * However, we assume that receiving a link layer ack ensures the delivery
   * of the transmitted packed to the IP stack of the neighbour. This is a 
   * fair assumption and allows battery powered nodes save some battery by 
   * not re-testing the state of a neighbour periodically if it 
   * acknowledges link packets. */
  if(status == MAC_TX_OK) {
    uip_ds6_nbr_t *nbr;
    nbr = uip_ds6_nbr_ll_lookup((uip_lladdr_t *)dest);
    if(nbr != NULL && nbr->state != NBR_INCOMPLETE) {
      nbr->state = NBR_REACHABLE;
      stimer_set(&nbr->reachable, UIP_ND6_REACHABLE_TIME / 1000);
      PRINTF("uip-ds6-neighbor : received a link layer ACK : ");
      PRINTLLADDR((uip_lladdr_t *)dest);
      PRINTF(" is reachable.\n");
    }
  }
#endif /* UIP_DS6_LL_NUD */

}
/*---------------------------------------------------------------------------*/
void
uip_ds6_neighbor_periodic(void)
{
  /* Periodic processing on neighbors */

  uip_ipaddr_t* router;
  uip_ipaddr_t source;
  uip_ds6_nbr_t *nbr = nbr_table_head(ds6_neighbors);
  while(nbr != NULL) {
    switch(nbr->state) {
    case NBR_REACHABLE:
      if(stimer_expired(&nbr->reachable)) {
#if UIP_CONF_IPV6_RPL
        /* when a neighbor leave it's REACHABLE state and is a default router,
           instead of going to STALE state it enters DELAY state in order to
           force a NUD on it. Otherwise, if there is no upward traffic, the
           node never knows if the default router is still reachable. This
           mimics the 6LoWPAN-ND behavior.
         */
        if(uip_ds6_defrt_lookup(&nbr->ipaddr) != NULL) {
          PRINTF("REACHABLE: defrt moving to DELAY (");
          PRINT6ADDR(&nbr->ipaddr);
          PRINTF(")\n");
          nbr->state = NBR_DELAY;
          stimer_set(&nbr->reachable, UIP_ND6_DELAY_FIRST_PROBE_TIME);
          nbr->nscount = 0;
        } else {
          PRINTF("REACHABLE: moving to STALE (");
          PRINT6ADDR(&nbr->ipaddr);
          PRINTF(")\n");
          nbr->state = NBR_STALE;
        }
#else /* UIP_CONF_IPV6_RPL */
        PRINTF("REACHABLE: moving to STALE (");
        PRINT6ADDR(&nbr->ipaddr);
        PRINTF(")\n");
        nbr->state = NBR_STALE;
#endif /* UIP_CONF_IPV6_RPL */
      }
#if LN_ROLE
        if(nbr->nscount >= UIP_ND6_MAX_UNICAST_SOLICIT) {
          uip_ds6_defrt_t *locdefrt;
          PRINTF("NS SEND END\n");
          if((locdefrt = uip_ds6_defrt_lookup(&nbr->ipaddr)) != NULL) {
            if (!locdefrt->isinfinite) {
              uip_ds6_defrt_rm(locdefrt);
            }
          }
          uip_ds6_nbr_rm(nbr);
        }
        else if(stimer_expired(&nbr->sendns) && uip_len==0) {
          router= uip_ds6_defrt_choose();
          if(router !=NULL) {
            uip_ds6_select_src(&source,router);
#if (UIP_CONF_MULTIPLE_PREFIXES)
            uip_ds6_addr_t *locaddr;
              nbr->nscount++;
             for(locaddr = uip_ds6_if.addr_list;
                  locaddr < uip_ds6_if.addr_list + UIP_DS6_ADDR_NB; locaddr++) {
                if(locaddr->isused && ( locaddr->state == IP_ADDRESS_GLOBAL)
                   && !(uip_is_addr_linklocal(&locaddr->ipaddr))) {
                    uip_nd6_ns_output(&locaddr->ipaddr,router,router);
                    tcpip_ipv6_output_with_arg(1);
                }
              }
#else
              uip_nd6_ns_output(&source,router,router);
#endif
              stimer_set(&nbr->sendns, uip_ds6_if.retrans_timer / 1000);
          }
        }
        if(stimer_remaining(&nbr->reachable) < (nbr->reachable.interval / RS_EARLY_WARNING_DIVIDER) ) {
          // Close to expiring, trigger request for update
          uip_ds6_start_rs_sending(0);
        }
        break;
#endif /* LN_ROLE */
      break;
#if LBR_ROLE
    case NBR_REGISTERED:
      if(nbr->interface_type == UIP_INTERFACE_TYPE_DECT)
      {
        if(stimer_expired(&nbr->reachable)) {
          PRINTF("Neighbor lost 1\n");
          uip_ds6_nbr_rm(nbr);
        }
      }
      break;
    case NBR_STALE:
#if !FILTERED_BORDER_ROUTER
      if(stimer_expired(&nbr->reachable)) {
        PRINTF("Neighbor lost 2\n");
        uip_ds6_nbr_rm(nbr);
      }
#endif
      break;
#endif /* LBR_ROLE */
#if LN_ROLE
    case NBR_STALE:
        
        if(stimer_expired(&nbr->reachable)) {
            PRINTF("Neighbor lost ln\n");
            //this should be okay since we only should have 1 router.. 
            if( nbr->isrouter ){
                resetAddrContexts();
            }
            uip_ds6_nbr_rm(nbr);
        }
        else if(stimer_expired(&nbr->sendns) && uip_len==0) {
          router= uip_ds6_defrt_choose();
          if(router !=NULL) {
            uip_ds6_select_src(&source,router);
#if (UIP_CONF_MULTIPLE_PREFIXES)
             uip_ds6_addr_t *locaddr;
              nbr->nscount++;
              PRINTF("NBR_STALE: count %d\n", nbr->nscount);
              for(locaddr = uip_ds6_if.addr_list;
                  locaddr < uip_ds6_if.addr_list + UIP_DS6_ADDR_NB; locaddr++) {
                if(locaddr->isused && ( locaddr->state == IP_ADDRESS_GLOBAL)
                   && !(uip_is_addr_linklocal(&locaddr->ipaddr))) {
                     uip_nd6_ns_output(&locaddr->ipaddr,router,router);
                     tcpip_ipv6_output_with_arg(1);
                }
              }
#else
              uip_nd6_ns_output(&source,router,router);
#endif
              stimer_set(&nbr->sendns, uip_ds6_if.retrans_timer / 1000);
          }
        }
        break;
#endif /* LN_ROLE */
#if UIP_ND6_SEND_NA
    case NBR_TENTATIVE:
      //we are in incomplete state until we receive a nS from the dect node. 
      //But we do not want to send a ns our self..
      //We use the reachable timer to check if we should be removed from the nbr list.
      if(stimer_expired(&nbr->reachable)) {
          uip_ds6_nbr_rm(nbr);
      }
      break;
    case NBR_INCOMPLETE:
      if(nbr->nscount >= UIP_ND6_MAX_MULTICAST_SOLICIT) {
        uip_ds6_nbr_rm(nbr);
      } else if(stimer_expired(&nbr->sendns) && (uip_len == 0)) {
        nbr->nscount++;
        PRINTF("NBR_INCOMPLETE: NS %u\n", nbr->nscount);
        uip_nd6_ns_output(NULL, NULL, &nbr->ipaddr);
        stimer_set(&nbr->sendns, uip_ds6_if.retrans_timer / 1000);
      }
      break;
    case NBR_DELAY:
      if(stimer_expired(&nbr->reachable)) {
        nbr->state = NBR_PROBE;
        nbr->nscount = 0;
        PRINTF("DELAY: moving to PROBE\n");
        stimer_set(&nbr->sendns, 0);
      }
      break;
    case NBR_PROBE:
      if(nbr->nscount >= UIP_ND6_MAX_UNICAST_SOLICIT) {
        uip_ds6_defrt_t *locdefrt;
        PRINTF("PROBE END\n");
        if((locdefrt = uip_ds6_defrt_lookup(&nbr->ipaddr)) != NULL) {
          if (!locdefrt->isinfinite) {
            uip_ds6_defrt_rm(locdefrt);
          }
        }
        uip_ds6_nbr_rm(nbr);
      } else if(stimer_expired(&nbr->sendns) && (uip_len == 0)) {
        nbr->nscount++;
        PRINTF("PROBE: NS %u\n", nbr->nscount);
        uip_nd6_ns_output(NULL, &nbr->ipaddr, &nbr->ipaddr);
        stimer_set(&nbr->sendns, uip_ds6_if.retrans_timer / 1000);
      }
      break;
#endif /* UIP_ND6_SEND_NA */
    default:
      break;
    }
    nbr = nbr_table_next(ds6_neighbors, nbr);
  }
}
/*---------------------------------------------------------------------------*/
uip_ds6_nbr_t *
uip_ds6_get_least_lifetime_neighbor(void)
{
  uip_ds6_nbr_t *nbr = nbr_table_head(ds6_neighbors);
  uip_ds6_nbr_t *nbr_expiring = NULL;
  while(nbr != NULL) {
    if(nbr_expiring != NULL) {
      clock_time_t curr = stimer_remaining(&nbr->reachable);
      if(curr < stimer_remaining(&nbr->reachable)) {
        nbr_expiring = nbr;
      }
    } else {
      nbr_expiring = nbr;
    }
    nbr = nbr_table_next(ds6_neighbors, nbr);
  }
  return nbr_expiring;
}
/*---------------------------------------------------------------------------*/
/** @} */

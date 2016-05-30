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

#ifndef UIP_DS6_NEIGHBOR_H_
#define UIP_DS6_NEIGHBOR_H_

#include "net/ip/uip.h"
#include "net/nbr-table.h"
#include "sys/stimer.h"
#include "net/ipv6/uip-ds6.h"
#include "net/nbr-table.h"

#include "net/ipv6/uip-icmp6.h"

#if UIP_CONF_IPV6_QUEUE_PKT
#include "net/ip/uip-packetqueue.h"
#endif                          /*UIP_CONF_QUEUE_PKT */

#ifdef LBR_ROLE
#include "ule6lo_types_6LBR.h"
#else
#include "ule6lo_types_6LN.h"
#endif

/*--------------------------------------------------*/
/** \brief Possible states for the nbr cache entries */
#define  NBR_INCOMPLETE 0
#define  NBR_REACHABLE 1
#define  NBR_STALE 2
#define  NBR_DELAY 3
#define  NBR_PROBE 4
#define  NBR_TENTATIVE 5
#define  NBR_REGISTERED 6

NBR_TABLE_DECLARE(ds6_neighbors);

/** \brief An entry in the nbr cache */
typedef struct uip_ds6_nbr {
  uip_ipaddr_t ipaddr;
  struct stimer reachable;
  struct stimer sendns;
  uint8_t nscount;
  uint8_t isrouter;
  uint8_t state;
  uint16_t link_metric;
  ule6lo_IPEI_t ipei;
#if FILTERED_BORDER_ROUTER
  uint8_t interface_type;	//0 = dect, 1 = eth, 0xFF = unknown.
#endif
#if UIP_CONF_IPV6_QUEUE_PKT
  struct uip_packetqueue_handle packethandle;
#define UIP_DS6_NBR_PACKET_LIFETIME CLOCK_SECOND * 4
#endif                          /*UIP_CONF_QUEUE_PKT */
#if (UIP_CONF_MLD & LBR_ROLE)
  uint16_t maddr_list_index[UIP_DS6_MADDR_NBR_MAX];
#endif
#if (UIP_CONF_MULTIPLE_PREFIXES & LBR_ROLE)
  uip_Prefix_ip6addr_t m_prefix[UIP_DS6_MULTIPLE_PREFIXES_MAX];
#endif
} uip_ds6_nbr_t;

void uip_ds6_neighbors_init(void);

/** \brief Neighbor Cache basic routines */
#if FILTERED_BORDER_ROUTER
uip_ds6_nbr_t *uip_ds6_nbr_add(const uip_ipaddr_t *ipaddr, const uip_lladdr_t *lladdr,
                               uint8_t isrouter, uint8_t state, const ule6lo_IPEI_t *ipei, uint8_t interface_type);
#else
uip_ds6_nbr_t *uip_ds6_nbr_add(const uip_ipaddr_t *ipaddr, const uip_lladdr_t *lladdr,
                               uint8_t isrouter, uint8_t state, const ule6lo_IPEI_t *ipei);
#endif
void uip_ds6_nbr_rm(uip_ds6_nbr_t *nbr);
const uip_lladdr_t *uip_ds6_nbr_get_ll(const uip_ds6_nbr_t *nbr);
const uip_ipaddr_t *uip_ds6_nbr_get_ipaddr(const uip_ds6_nbr_t *nbr);
uip_ds6_nbr_t *uip_ds6_nbr_lookup(const uip_ipaddr_t *ipaddr);
uip_ds6_nbr_t *uip_ds6_nbr_ll_lookup(const uip_lladdr_t *lladdr);
uip_ds6_nbr_t *uip_ds6_nbr_ipei_lookup(const ule6lo_IPEI_t *ipei);
uip_ipaddr_t *uip_ds6_nbr_ipaddr_from_lladdr(const uip_lladdr_t *lladdr);
const uip_lladdr_t *uip_ds6_nbr_lladdr_from_ipaddr(const uip_ipaddr_t *ipaddr);
const ule6lo_IPEI_t *uip_ds6_nbr_ipei_from_ipaddr(const uip_ipaddr_t *ipaddr);
void uip_ds6_link_neighbor_callback(int status, int numtx);
void uip_ds6_neighbor_periodic(void);
int uip_ds6_nbr_num(void);
uint16_t uip_ds6_nbr_lookup_mc_in_list(const uip_ipaddr_t *ipaddr);
uint8_t uip_ds6_nbr_forward_mc_from_list(uint16_t index, uip_ds6_nbr_t *nbr);
uint16_t uip_ds6_nbr_lookup_index_in_all_nbrlist(uint16_t index);
void uip_ds6_nbr_maddr_add(uip_ds6_nbr_t *nbr, uip_icmp6_mld1* maddr);
void uip_ds6_nbr_maddr_rm(uip_ds6_nbr_t *nbr, uip_icmp6_mld1* maddr);
#if FILTERED_BORDER_ROUTER
uip_ds6_nbr_t * uip_ds6_nbr_lookup_solicited_mc(const uip_ipaddr_t *ipaddr);
#endif
/**
 * \brief
 *    This searches inside the neighbor table for the neighbor that is about to
 *    expire the next.
 *
 * \return
 *    A reference to the neighbor about to expire the next or NULL if
 *    table is empty.
 */
uip_ds6_nbr_t *uip_ds6_get_least_lifetime_neighbor(void);

#endif /* UIP_DS6_NEIGHBOR_H_ */
/** @} */

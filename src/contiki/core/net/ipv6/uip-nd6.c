/*
 * Copyright (C) 1995, 1996, 1997, and 1998 WIDE Project.
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
 * 3. Neither the name of the project nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE PROJECT AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE PROJECT OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
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
 */

/**
 * \addtogroup uip6
 * @{
 */

/**
 * \file
 *    Neighbor discovery (RFC 4861)
 * \author Mathilde Durvy <mdurvy@cisco.com>
 * \author Julien Abeille <jabeille@cisco.com>
 */

#include <string.h>
#include "net/ipv6/uip-icmp6.h"
#include "net/ipv6/uip-nd6.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ipv6/uip-mld.h"
#include "net/ipv6/uip-ds6-nbr.h"
#include "net/ip/uip-nameserver.h"
#include "lib/random.h"
#if FILTERED_BORDER_ROUTER
#include "cetic-6lbr.h"
#endif
#include <ipv6/sicslowpan.h>
#include "ule6lo_ipei_defines.h"


/*------------------------------------------------------------------*/

#define DEBUG DEBUG_FULL
#include "net/ip/uip-debug.h"

#if UIP_LOGGING
#include <stdio.h>
void uip_log(char *msg);

#define UIP_LOG(m) uip_log(m)
#else
#define UIP_LOG(m)
#endif /* UIP_LOGGING == 1 */

/*------------------------------------------------------------------*/
/** @{ */
/** \name Pointers to the header structures.
 *  All pointers except UIP_IP_BUF depend on uip_ext_len, which at
 *  packet reception, is the total length of the extension headers.
 *  
 *  The pointer to ND6 options header also depends on nd6_opt_offset,
 *  which we set in each function.
 *
 *  Care should be taken when manipulating these buffers about the
 *  value of these length variables
 */

#define UIP_IP_BUF                ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])  /**< Pointer to IP header */
#define UIP_ICMP_BUF            ((struct uip_icmp_hdr *)&uip_buf[uip_l2_l3_hdr_len])  /**< Pointer to ICMP header*/
/**@{  Pointers to messages just after icmp header */
#define UIP_ND6_RS_BUF            ((uip_nd6_rs *)&uip_buf[uip_l2_l3_icmp_hdr_len])
#define UIP_ND6_RA_BUF            ((uip_nd6_ra *)&uip_buf[uip_l2_l3_icmp_hdr_len])
#define UIP_ND6_NS_BUF            ((uip_nd6_ns *)&uip_buf[uip_l2_l3_icmp_hdr_len])
#define UIP_ND6_NA_BUF            ((uip_nd6_na *)&uip_buf[uip_l2_l3_icmp_hdr_len])
/** @} */
/** Pointer to ND option */
#define UIP_ND6_OPT_HDR_BUF  ((uip_nd6_opt_hdr *)&uip_buf[uip_l2_l3_icmp_hdr_len + nd6_opt_offset])
#define UIP_ND6_OPT_PREFIX_BUF ((uip_nd6_opt_prefix_info *)&uip_buf[uip_l2_l3_icmp_hdr_len + nd6_opt_offset])
#define UIP_ND6_OPT_MTU_BUF ((uip_nd6_opt_mtu *)&uip_buf[uip_l2_l3_icmp_hdr_len + nd6_opt_offset])
#define UIP_ND6_OPT_RDNSS_BUF ((uip_nd6_opt_dns *)&uip_buf[uip_l2_l3_icmp_hdr_len + nd6_opt_offset])
#define UIP_ND6_OPT_6CO_BUF ((uip_nd6_opt_6CO *)&uip_buf[uip_l2_l3_icmp_hdr_len + nd6_opt_offset])
/** @} */

#if UIP_ND6_SEND_NA || UIP_ND6_SEND_RA || !UIP_CONF_ROUTER
static uint8_t nd6_opt_offset;                     /** Offset from the end of the icmpv6 header to the option in uip_buf*/
static uint8_t *nd6_opt_llao;   /**  Pointer to llao option in uip_buf */
static uip_ds6_nbr_t *nbr; /**  Pointer to a nbr cache entry*/
static uip_ds6_defrt_t *defrt; /**  Pointer to a router list entry */
static uip_ds6_addr_t *addr; /**  Pointer to an interface address */
static uip_nd6_opt_address_reg* nd6_opt_aro; /** Pointer to ARO option in uip_buf */
#endif /* UIP_ND6_SEND_NA || UIP_ND6_SEND_RA || !UIP_CONF_ROUTER */

#if !UIP_CONF_ROUTER  || FILTERED_BORDER_ROUTER          // TBD see if we move it to ra_input
static uip_nd6_opt_prefix_info *nd6_opt_prefix_info; /**  Pointer to prefix information option in uip_buf */
static uip_nd6_opt_6CO *nd6_opt_6CO;
static uip_ipaddr_t ipaddr;
#endif
#if (!UIP_CONF_ROUTER || UIP_ND6_SEND_RA)
static uip_ds6_prefix_t *prefix; /**  Pointer to a prefix list entry */
#endif

#if FILTERED_BORDER_ROUTER
static uip_ds6_prefix_non_router_t *prefix_non_router; /**  Pointer to a prefix list entry */
#endif



#if UIP_CONF_DS6_ROUTE_INFORMATION || FILTERED_BORDER_ROUTER
#define UIP_ND6_OPT_ROUTE_BUF ((uip_nd6_opt_route_info *)&uip_buf[uip_l2_l3_icmp_hdr_len + nd6_opt_offset])
#endif

#if UIP_ND6_SEND_NA || UIP_ND6_SEND_RA || !UIP_CONF_ROUTER
/*------------------------------------------------------------------*/
/* Copy link-layer address from LLAO option to a word-aligned uip_lladdr_t */
static void
extract_lladdr_aligned(uip_lladdr_t *dest) {
  if(dest != NULL && nd6_opt_llao != NULL) {
    memcpy(dest, &nd6_opt_llao[UIP_ND6_OPT_DATA_OFFSET], UIP_LLADDR_LEN);
  }
}
#endif /* UIP_ND6_SEND_NA || UIP_ND6_SEND_RA || !UIP_CONF_ROUTER */
/*------------------------------------------------------------------*/
static void
create_address_reg(uint8_t *buffer, uint8_t* source, uint8_t status, uint16_t lifetime) {
  uip_nd6_opt_address_reg* address_reg = (uip_nd6_opt_address_reg*)buffer;
  address_reg->type = UIP_ND6_OPT_ADDRESS_REG;
  address_reg->len = 2;
  address_reg->status=status;
  address_reg->reserved1=0;
  address_reg->reserved=0;
  address_reg->lifetime = lifetime;

  memcpy(address_reg->eui64,source,8);
  address_reg->eui64[0] ^= 0x02;
}

/* create a llao */ 
static void
create_llao(uint8_t *llao, uint8_t type) {
  llao[UIP_ND6_OPT_TYPE_OFFSET] = type;
  llao[UIP_ND6_OPT_LEN_OFFSET] = UIP_ND6_OPT_LLAO_LEN >> 3;
  memcpy(&llao[UIP_ND6_OPT_DATA_OFFSET], &uip_lladdr, UIP_LLADDR_LEN);
  /* padding on some */
  memset(&llao[UIP_ND6_OPT_DATA_OFFSET + UIP_LLADDR_LEN], 0,
         UIP_ND6_OPT_LLAO_LEN - 2 - UIP_LLADDR_LEN);
}


static void
create_llao_with_mac(uint8_t *llao, uint8_t type, const uip_lladdr_t* mac) {
  llao[UIP_ND6_OPT_TYPE_OFFSET] = type;
  llao[UIP_ND6_OPT_LEN_OFFSET] = UIP_ND6_OPT_LLAO_LEN >> 3;
  memcpy(&llao[UIP_ND6_OPT_DATA_OFFSET], mac, UIP_LLADDR_LEN);
  /* padding on some */
  memset(&llao[UIP_ND6_OPT_DATA_OFFSET + UIP_LLADDR_LEN], 0,
         UIP_ND6_OPT_LLAO_LEN - 2 - UIP_LLADDR_LEN);
}

/*------------------------------------------------------------------*/

#if UIP_ND6_SEND_NA
static void
ns_input(void)
{
  uint8_t flags;
#if LBR_ROLE
  uint8_t aro_return_status = ARO_SUCCESS;
  uint8_t aro_eui64[8];
  uint8_t include_aro=0;
#endif
  uip_ds6_nbr_t* neighbor=NULL;

  PRINTF("Received NS from ");
  PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
  PRINTF(" to ");
  PRINT6ADDR(&UIP_IP_BUF->destipaddr);
  PRINTF(" with target address");
  PRINT6ADDR((uip_ipaddr_t *) (&UIP_ND6_NS_BUF->tgtipaddr));
  PRINTF("\n");
  UIP_STAT(++uip_stat.nd6.recv);

#if UIP_CONF_IPV6_CHECKS
  if((UIP_IP_BUF->ttl != UIP_ND6_HOP_LIMIT) ||
     (uip_is_addr_mcast(&UIP_ND6_NS_BUF->tgtipaddr)) ||
     (UIP_ICMP_BUF->icode != 0)) {
    PRINTF("NS received is bad\n");
    goto discard;
  }
#endif /* UIP_CONF_IPV6_CHECKS */

  /* Options processing */
  // Start by getting pointers to all options
  nd6_opt_llao = NULL;
  nd6_opt_aro = NULL;
  nd6_opt_offset = UIP_ND6_NS_LEN;
  while(uip_l3_icmp_hdr_len + nd6_opt_offset < uip_len) {
#if UIP_CONF_IPV6_CHECKS
    if(UIP_ND6_OPT_HDR_BUF->len == 0) {
      PRINTF("NS received is bad\n");
      goto discard;
    }
#endif /* UIP_CONF_IPV6_CHECKS */
    switch (UIP_ND6_OPT_HDR_BUF->type) {
    case UIP_ND6_OPT_SLLAO:
    	//printf("UIP_ND6_OPT_SLLAO option included\n");
      nd6_opt_llao = &uip_buf[uip_l2_l3_icmp_hdr_len + nd6_opt_offset];
      break;
#if LBR_ROLE
    case UIP_ND6_OPT_ADDRESS_REG:
      nd6_opt_aro = (uip_nd6_opt_address_reg*) &uip_buf[uip_l2_l3_icmp_hdr_len + nd6_opt_offset];
      break;
#endif
    default:
      PRINTF("ND option not supported in NS");
      break;
    }
    nd6_opt_offset += (UIP_ND6_OPT_HDR_BUF->len << 3);
  }

  // Process received options
#if LBR_ROLE
  if(nd6_opt_aro!=NULL) {
    if(nd6_opt_aro->len!=2) {
      PRINTF("NS ARO opt len is bad\n");
      goto discard;
    }
    if(nd6_opt_aro->status!=0) {
      PRINTF("NS ARO opt status is bad\n");
      goto discard;
    }
    if(nd6_opt_llao!=NULL && !uip_is_addr_unspecified(&UIP_IP_BUF->srcipaddr)) {
      // Handle the ARO
      PRINTF("NS ARO opt Processing\n");
      nbr = uip_ds6_nbr_lookup(&UIP_IP_BUF->srcipaddr);
      memcpy(&aro_eui64[0],nd6_opt_aro->eui64,sizeof(aro_eui64));
      aro_eui64[0] ^= 0x02;
      if(nbr == NULL) {
        // Not a duplicate
        aro_return_status = ARO_SUCCESS;
        include_aro=1;
      }
      else {
        //Possibly a duplicate, check EUI
        const uip_lladdr_t * mac = uip_ds6_nbr_get_ll(nbr);
        uip_ipaddr_t ipaddr_l;
        uip_ds6_set_addr_iid(&ipaddr_l, (uip_lladdr_t*)mac);
        if(memcmp(&ipaddr_l.u8[8],&aro_eui64[0],sizeof(aro_eui64))==0) {
          //Not a duplicate, eui matches
          aro_return_status = ARO_SUCCESS;
          include_aro=1;
        }
        else {
          uip_ipaddr_t aro_return_destination;
          //Duplicate detected, do not modify neighbor list
          PRINTF("NS ARO opt duplicate detected\n");
          uip_ipaddr_copy(&UIP_IP_BUF->srcipaddr, &UIP_ND6_NS_BUF->tgtipaddr);
          memcpy(&aro_return_destination.u8[8],nd6_opt_aro->eui64,sizeof(aro_eui64));
          uip_create_linklocal_prefix(&aro_return_destination);
          aro_return_destination.u8[8] ^= 0x02;
          uip_ipaddr_copy(&UIP_IP_BUF->destipaddr, &aro_return_destination);
          flags = UIP_ND6_NA_FLAG_SOLICITED | UIP_ND6_NA_FLAG_OVERRIDE;
          aro_return_status = ARO_DUPLICATE;
          include_aro =1;
          goto create_na;
        }
      }
    }
    //else process as ARO option was not present
  }
#endif

  if(nd6_opt_llao!=NULL) {
#if UIP_CONF_IPV6_CHECKS
    /* There must be NO option in a DAD NS */
    if(uip_is_addr_unspecified(&UIP_IP_BUF->srcipaddr)) {
      PRINTF("NS received is bad\n");
      goto discard;
    } else {
#endif /*UIP_CONF_IPV6_CHECKS */
      nbr = uip_ds6_nbr_lookup(&UIP_IP_BUF->srcipaddr);
      if(nbr == NULL) {
        //printf("nbr==null\n");
        uip_lladdr_t lladdr_aligned;
        extract_lladdr_aligned(&lladdr_aligned);
#if FILTERED_BORDER_ROUTER
        
        if(UIP_RECEIVED_INTERFACE_TYPE == UIP_INTERFACE_TYPE_DECT){
          nbr = uip_ds6_nbr_add(&UIP_IP_BUF->srcipaddr, &lladdr_aligned, 0, NBR_REGISTERED,UIP_IPEI_RECEIVED, UIP_RECEIVED_INTERFACE_TYPE);
        }
        else{
          nbr = uip_ds6_nbr_add(&UIP_IP_BUF->srcipaddr, &lladdr_aligned, 0, NBR_STALE,UIP_IPEI_RECEIVED, UIP_RECEIVED_INTERFACE_TYPE);
        }
#else
        nbr = uip_ds6_nbr_add(&UIP_IP_BUF->srcipaddr, &lladdr_aligned, 0, NBR_STALE,UIP_IPEI_RECEIVED);
#endif
#if LBR_ROLE
        if(nd6_opt_aro!=NULL && nbr == NULL) {
          //Not enough space in neighbor list, reply with error
          PRINTF("NS ARO opt neighbor list full\n");
          uip_ipaddr_t aro_return_destination;
          memcpy(&aro_return_destination.u8[8],nd6_opt_aro->eui64,sizeof(aro_eui64));
          uip_create_linklocal_prefix(&aro_return_destination);
          aro_return_destination.u8[8] ^= 0x02;
          uip_ipaddr_copy(&UIP_IP_BUF->srcipaddr, &UIP_ND6_NS_BUF->tgtipaddr);
          uip_ipaddr_copy(&UIP_IP_BUF->destipaddr, &aro_return_destination);
          flags = UIP_ND6_NA_FLAG_SOLICITED | UIP_ND6_NA_FLAG_OVERRIDE;
          aro_return_status = ARO_ROUTER_CACHE_FULL;
          include_aro =1;
          goto create_na;
        }
#endif
      } else {
        //printf("nbr!=null\n");
        uip_lladdr_t *lladdr = (uip_lladdr_t *)uip_ds6_nbr_get_ll(nbr);
        if(memcmp(&nd6_opt_llao[UIP_ND6_OPT_DATA_OFFSET],
            lladdr, UIP_LLADDR_LEN) != 0) {
          //printf("memcmp differ\n");
          memcpy(lladdr, &nd6_opt_llao[UIP_ND6_OPT_DATA_OFFSET], UIP_LLADDR_LEN);
          if(UIP_RECEIVED_INTERFACE_TYPE == UIP_INTERFACE_TYPE_DECT){
            nbr->state = NBR_REGISTERED;
          }else{
            nbr->state = NBR_STALE;
          }

          UIP_READ_RECEIVED_IPEI(&nbr->ipei);
          //memcpy(,UIP_IPEI,sizeof(ule6lo_IPEI_t));
        } else {
          if(nbr->state == NBR_INCOMPLETE) {
            nbr->state = NBR_STALE;
            UIP_READ_RECEIVED_IPEI(&nbr->ipei);
          }
          else if(nbr->state == NBR_TENTATIVE) {
            nbr->state = NBR_REGISTERED;
            UIP_READ_RECEIVED_IPEI(&nbr->ipei);
          }
        }
      }
#if LBR_ROLE
      if(nd6_opt_aro!=NULL) {
        PRINTF("NS receievd from 6LN node, setting reachable timer\n");
        stimer_set(&nbr->reachable,nd6_opt_aro->lifetime*60);
      }
      //TODO Possibly handle reachable value of 0. Don't see it happening but RFC 6775 describes it
#endif

#if UIP_CONF_IPV6_CHECKS
    }
#endif /*UIP_CONF_IPV6_CHECKS */
  }


  addr = uip_ds6_addr_lookup(&UIP_ND6_NS_BUF->tgtipaddr);
#if FILTERED_BORDER_ROUTER
  if(addr == NULL) {
	  // Target not myself, check neighbor list for the target
	  neighbor = uip_ds6_nbr_lookup(&UIP_ND6_NS_BUF->tgtipaddr);
	  if(neighbor!=NULL && neighbor->state != NBR_INCOMPLETE && neighbor->interface_type == UIP_LN_INTERFACE_MASK) {
		  // Reply on the neighbor's behalf
		  PRINTF("NS received for a known node on solicited address\n");
		  uip_ipaddr_copy(&UIP_IP_BUF->destipaddr, &UIP_IP_BUF->srcipaddr);
		  uip_ipaddr_copy(&UIP_IP_BUF->srcipaddr, &UIP_ND6_NS_BUF->tgtipaddr);
		  flags = UIP_ND6_NA_FLAG_SOLICITED | UIP_ND6_NA_FLAG_OVERRIDE;
		  goto create_na;
	  }
  }
#endif
  if(addr != NULL) {
#if UIP_ND6_DEF_MAXDADNS > 0
    if(uip_is_addr_unspecified(&UIP_IP_BUF->srcipaddr)) {
      /* DAD CASE */
#if UIP_CONF_IPV6_CHECKS
      if(!uip_is_addr_solicited_node(&UIP_IP_BUF->destipaddr)) {
        PRINTF("NS received is bad\n");
        goto discard;
      }
#endif /* UIP_CONF_IPV6_CHECKS */
      if(addr->state != ADDR_TENTATIVE) {
        uip_create_linklocal_allnodes_mcast(&UIP_IP_BUF->destipaddr);
        uip_ds6_select_src(&UIP_IP_BUF->srcipaddr, &UIP_IP_BUF->destipaddr);
        flags = UIP_ND6_NA_FLAG_OVERRIDE;
        goto create_na;
      } else {
          /** \todo if I sent a NS before him, I win */
        uip_ds6_dad_failed(addr);
        goto discard;
      }
#else /* UIP_ND6_DEF_MAXDADNS > 0 */
    if(uip_is_addr_unspecified(&UIP_IP_BUF->srcipaddr)) {
      /* DAD CASE */
      goto discard;
#endif /* UIP_ND6_DEF_MAXDADNS > 0 */
    }
#if UIP_CONF_IPV6_CHECKS
    if(uip_ds6_is_my_addr(&UIP_IP_BUF->srcipaddr)) {
        /**
         * \NOTE do we do something here? we both are using the same address.
         * If we are doing dad, we could cancel it, though we should receive a
         * NA in response of DAD NS we sent, hence DAD will fail anyway. If we
         * were not doing DAD, it means there is a duplicate in the network!
         */
      PRINTF("NS received is bad\n");
      goto discard;
    }
#endif /*UIP_CONF_IPV6_CHECKS */

    /* Address resolution case */
    if(uip_is_addr_solicited_node(&UIP_IP_BUF->destipaddr)) {
      uip_ipaddr_copy(&UIP_IP_BUF->destipaddr, &UIP_IP_BUF->srcipaddr);
      uip_ipaddr_copy(&UIP_IP_BUF->srcipaddr, &UIP_ND6_NS_BUF->tgtipaddr);
      flags = UIP_ND6_NA_FLAG_SOLICITED | UIP_ND6_NA_FLAG_OVERRIDE;
      goto create_na;
//#endif
    }

    /* NUD CASE */
    if(uip_ds6_addr_lookup(&UIP_IP_BUF->destipaddr) == addr) {
      uip_ipaddr_copy(&UIP_IP_BUF->destipaddr, &UIP_IP_BUF->srcipaddr);
      uip_ipaddr_copy(&UIP_IP_BUF->srcipaddr, &UIP_ND6_NS_BUF->tgtipaddr);
      flags = UIP_ND6_NA_FLAG_SOLICITED | UIP_ND6_NA_FLAG_OVERRIDE;
      goto create_na;
    } else {
#if UIP_CONF_IPV6_CHECKS
      PRINTF("NS received is bad\n");
      goto discard;
#endif /* UIP_CONF_IPV6_CHECKS */
#if FILTERED_BORDER_ROUTER
      // Target not myself, check neighbor list for the target
      neighbor = uip_ds6_nbr_lookup(&UIP_ND6_NS_BUF->tgtipaddr);
      if(neighbor!=NULL && neighbor->state != NBR_INCOMPLETE && neighbor->interface_type == UIP_LN_INTERFACE_MASK) {
    	  // Reply on the neighbor's behalf
    	  PRINTF("NS received for a known node on unicast address\n");
    	  uip_ipaddr_copy(&UIP_IP_BUF->destipaddr, &UIP_IP_BUF->srcipaddr);
    	  uip_ipaddr_copy(&UIP_IP_BUF->srcipaddr, &UIP_ND6_NS_BUF->tgtipaddr);
    	  flags = UIP_ND6_NA_FLAG_SOLICITED | UIP_ND6_NA_FLAG_OVERRIDE;
    	  goto create_na;
      }
      else {
    	  PRINTF("NS received for a unknown destination on unicast address\n");
    	  goto discard;
      }
#endif
    }
  } else {
    goto discard;
  }


create_na:
    /* If the node is a router it should set R flag in NAs */

#if UIP_CONF_ROUTER
#if FILTERED_BORDER_ROUTER
// Router flag depends on interface.
// For ethernet, it is not a router, for wireless it is.
  if(UIP_RECEIVED_INTERFACE_TYPE == UIP_INTERFACE_TYPE_DECT) {
	  flags = flags | UIP_ND6_NA_FLAG_ROUTER;
  }
#else
    flags = flags | UIP_ND6_NA_FLAG_ROUTER;
#endif
#endif /* UIP_CONF_ROUTER */
  uip_ext_len = 0;
  UIP_IP_BUF->vtc = 0x60;
  UIP_IP_BUF->tcflow = 0;
  UIP_IP_BUF->flow = 0;
  UIP_IP_BUF->len[0] = 0;       /* length will not be more than 255 */
  UIP_IP_BUF->len[1] = UIP_ICMPH_LEN + UIP_ND6_NA_LEN + UIP_ND6_OPT_LLAO_LEN;
  UIP_IP_BUF->proto = UIP_PROTO_ICMP6;
  UIP_IP_BUF->ttl = UIP_ND6_HOP_LIMIT;

  UIP_ICMP_BUF->type = ICMP6_NA;
  UIP_ICMP_BUF->icode = 0;

  UIP_ND6_NA_BUF->flagsreserved = flags;
  memcpy(&UIP_ND6_NA_BUF->tgtipaddr, &UIP_ND6_NS_BUF->tgtipaddr, sizeof(uip_ipaddr_t));

  uip_len =
      UIP_IPH_LEN + UIP_ICMPH_LEN + UIP_ND6_NA_LEN + UIP_ND6_OPT_LLAO_LEN;

#if FILTERED_BORDER_ROUTER
  if(neighbor==NULL) {
	  // Sending from ourselves, use our own mac address
	  PRINTF("Sending on behalf of our own mac address\n");
	  create_llao(&uip_buf[uip_l2_l3_icmp_hdr_len + UIP_ND6_NA_LEN],
	                UIP_ND6_OPT_TLLAO);

#if LBR_ROLE
	  if(include_aro) {
	    create_address_reg(&uip_buf[uip_l2_l3_icmp_hdr_len + UIP_ND6_NA_LEN+UIP_ND6_OPT_LLAO_LEN],
	      &aro_eui64[0],
	      aro_return_status,
	      nd6_opt_aro->lifetime);

	    uip_len+= UIP_ND6_OPT_ADDRESS_REG_LEN;
	    UIP_IP_BUF->len[1] +=UIP_ND6_OPT_ADDRESS_REG_LEN;
	  }
#endif
  }
  else {
	  // Sending on behalf of a neighbor, use that neighbor's mac address.
	  PRINTF("Sending on behalf of neighbor mac address\n");
	  create_llao_with_mac(&uip_buf[uip_l2_l3_icmp_hdr_len + UIP_ND6_NA_LEN],
	  	                UIP_ND6_OPT_TLLAO, uip_ds6_nbr_get_ll(neighbor));
  }

#else
  create_llao(&uip_buf[uip_l2_l3_icmp_hdr_len + UIP_ND6_NA_LEN],
              UIP_ND6_OPT_TLLAO);
#endif

  UIP_ICMP_BUF->icmpchksum = 0;
  UIP_ICMP_BUF->icmpchksum = ~uip_icmp6chksum();


  UIP_STAT(++uip_stat.nd6.sent);
  PRINTF("Sending NA to ");
  PRINT6ADDR(&UIP_IP_BUF->destipaddr);
  PRINTF(" from ");
  PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
  PRINTF(" with target address ");
  PRINT6ADDR(&UIP_ND6_NA_BUF->tgtipaddr);
  PRINTF("\n");

#if FILTERED_BORDER_ROUTER
  // Limiting output to the same interface the request came in on.
  *UIP_TRANSMITTED_FLAGS = UIP_RECEIVED_INTERFACE_TYPE;
#endif
  return;


discard:
  uip_clear_buf();
  return;
}
#endif /* UIP_ND6_SEND_NA */


/*------------------------------------------------------------------*/
void
uip_nd6_ns_output(uip_ipaddr_t * src, uip_ipaddr_t * dest, uip_ipaddr_t * tgt)
{
  uip_ds6_nbr_t* neighbor;
  uip_ext_len = 0;
  UIP_IP_BUF->vtc = 0x60;
  UIP_IP_BUF->tcflow = 0;
  UIP_IP_BUF->flow = 0;
  UIP_IP_BUF->proto = UIP_PROTO_ICMP6;
  UIP_IP_BUF->ttl = UIP_ND6_HOP_LIMIT;

  if(dest == NULL) {
    uip_create_solicited_node(tgt, &UIP_IP_BUF->destipaddr);
  } else {
    uip_ipaddr_copy(&UIP_IP_BUF->destipaddr, dest);
  }
  UIP_ICMP_BUF->type = ICMP6_NS;
  UIP_ICMP_BUF->icode = 0;
  UIP_ND6_NS_BUF->reserved = 0;
  uip_ipaddr_copy((uip_ipaddr_t *) &UIP_ND6_NS_BUF->tgtipaddr, tgt);
  UIP_IP_BUF->len[0] = 0;       /* length will not be more than 255 */
  /*
   * check if we add a SLLAO option: for DAD, MUST NOT, for NUD, MAY
   * (here yes), for Address resolution , MUST 
   */
  if(!(uip_ds6_is_my_addr(tgt))) {
    if(src != NULL) {
      uip_ipaddr_copy(&UIP_IP_BUF->srcipaddr, src);
    } else {
      uip_ds6_select_src(&UIP_IP_BUF->srcipaddr, &UIP_IP_BUF->destipaddr);
    }
    if (uip_is_addr_unspecified(&UIP_IP_BUF->srcipaddr)) {
      PRINTF("Dropping NS due to no suitable source address\n");
      uip_clear_buf();
      return;
    }
    UIP_IP_BUF->len[1] =
      UIP_ICMPH_LEN + UIP_ND6_NS_LEN + UIP_ND6_OPT_LLAO_LEN;

    create_llao(&uip_buf[uip_l2_l3_icmp_hdr_len + UIP_ND6_NS_LEN],
		UIP_ND6_OPT_SLLAO);

    uip_len =
          UIP_IPH_LEN + UIP_ICMPH_LEN + UIP_ND6_NS_LEN + UIP_ND6_OPT_LLAO_LEN;

#if !UIP_CONF_ROUTER
    if(!uip_is_addr_mcast(dest)) {
      // Append ARO option
      //TODO Should it always be added here?
      uip_ipaddr_t ll_addr;
      uip_ds6_set_addr_iid(&ll_addr, &uip_lladdr);
      {
        create_address_reg(&uip_buf[uip_l2_l3_icmp_hdr_len + UIP_ND6_NS_LEN+UIP_ND6_OPT_LLAO_LEN],&ll_addr.u8[8],0,UIP_ND6_ARO_REACHABLE_TIME / 1000);
        uip_len+= UIP_ND6_OPT_ADDRESS_REG_LEN;
        UIP_IP_BUF->len[1] +=UIP_ND6_OPT_ADDRESS_REG_LEN;
  
        // Enable/restart timer waiting for NA reply.
        neighbor =  uip_ds6_nbr_lookup(dest);
        if(neighbor!=NULL) {
          if(neighbor->nscount==0) {
            // Just started sending them, expect result quickly
            stimer_set(&neighbor->sendns,uip_ds6_if.retrans_timer);
          }
          else {
            stimer_restart(&neighbor->sendns);
          }
          neighbor->nscount++;
        }
      }
    }
#endif
  } else {
    uip_create_unspecified(&UIP_IP_BUF->srcipaddr);
    UIP_IP_BUF->len[1] = UIP_ICMPH_LEN + UIP_ND6_NS_LEN;
    uip_len = UIP_IPH_LEN + UIP_ICMPH_LEN + UIP_ND6_NS_LEN;
  }

  UIP_ICMP_BUF->icmpchksum = 0;
  UIP_ICMP_BUF->icmpchksum = ~uip_icmp6chksum();

  UIP_STAT(++uip_stat.nd6.sent);
  PRINTF("Sending NS to");
  PRINT6ADDR(&UIP_IP_BUF->destipaddr);
  PRINTF("from");
  PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
  PRINTF("with target address");
  PRINT6ADDR(tgt);
  PRINTF("\n");
  return;
}
//#if UIP_ND6_SEND_NA
/*------------------------------------------------------------------*/
/**
 * Neighbor Advertisement Processing
 *
 * we might have to send a pkt that had been buffered while address
 * resolution was performed (if we support buffering, see UIP_CONF_QUEUE_PKT)
 *
 * As per RFC 4861, on link layer that have addresses, TLLAO options MUST be
 * included when responding to multicast solicitations, SHOULD be included in
 * response to unicast (here we assume it is for now)
 *
 * NA can be received after sending NS for DAD, Address resolution or NUD. Can
 * be unsolicited as well.
 * It can trigger update of the state of the neighbor in the neighbor cache,
 * router in the router list.
 * If the NS was for DAD, it means DAD failed
 *
 */
static void
na_input(void)
{
  uint8_t is_llchange;
  uint8_t is_router;
  uint8_t is_solicited;
  uint8_t is_override;
  uint8_t temp_eui[8];

  PRINTF("Received NA from");
  PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
  PRINTF("to");
  PRINT6ADDR(&UIP_IP_BUF->destipaddr);
  PRINTF("with target address");
  PRINT6ADDR((uip_ipaddr_t *) (&UIP_ND6_NA_BUF->tgtipaddr));
  PRINTF("\n");
  UIP_STAT(++uip_stat.nd6.recv);

  /* 
   * booleans. the three last one are not 0 or 1 but 0 or 0x80, 0x40, 0x20
   * but it works. Be careful though, do not use tests such as is_router == 1 
   */
  is_llchange = 0;
  is_router = ((UIP_ND6_NA_BUF->flagsreserved & UIP_ND6_NA_FLAG_ROUTER));
  is_solicited =
    ((UIP_ND6_NA_BUF->flagsreserved & UIP_ND6_NA_FLAG_SOLICITED));
  is_override =
    ((UIP_ND6_NA_BUF->flagsreserved & UIP_ND6_NA_FLAG_OVERRIDE));

  PRINTF("Router flag is %i\n",is_router);

#if UIP_CONF_IPV6_CHECKS
  if((UIP_IP_BUF->ttl != UIP_ND6_HOP_LIMIT) ||
     (UIP_ICMP_BUF->icode != 0) ||
     (uip_is_addr_mcast(&UIP_ND6_NA_BUF->tgtipaddr)) ||
     (is_solicited && uip_is_addr_mcast(&UIP_IP_BUF->destipaddr))) {
    PRINTF("NA received is bad\n");
    goto discard;
  }
#endif /*UIP_CONF_IPV6_CHECKS */

  /* Options processing: we handle TLLAO, and must ignore others */
  nd6_opt_offset = UIP_ND6_NA_LEN;
  nd6_opt_llao = NULL;
  while(uip_l3_icmp_hdr_len + nd6_opt_offset < uip_len) {
#if UIP_CONF_IPV6_CHECKS
    if(UIP_ND6_OPT_HDR_BUF->len == 0) {
      PRINTF("NA received is bad\n");
      goto discard;
    }
#endif /*UIP_CONF_IPV6_CHECKS */
    switch (UIP_ND6_OPT_HDR_BUF->type) {
    case UIP_ND6_OPT_TLLAO:
      nd6_opt_llao = (uint8_t *)UIP_ND6_OPT_HDR_BUF;
      break;
#if LN_ROLE
    case UIP_ND6_OPT_ADDRESS_REG:
     {
        uip_ipaddr_t ll_addr;
    	PRINTF("ARO option received\n");
    	nd6_opt_aro = (uip_nd6_opt_address_reg*)UIP_ND6_OPT_HDR_BUF;
    	if(nd6_opt_aro->len !=2) {
    		PRINTF("ARO option length is bad\n");
    		goto discard;
    	}
    	memcpy(temp_eui,nd6_opt_aro->eui64,8);
    	temp_eui[0] ^= 0x02;
    	uip_ds6_set_addr_iid(&ll_addr, &uip_lladdr);
    	if(memcmp(temp_eui,&ll_addr.u8[8],8)!=0) {
    		PRINTF("ARO option EUI64 not matching ours\n");
    		goto discard;
    	}
    	switch(nd6_opt_aro->status) {
    	case ARO_SUCCESS:
    	break;
    	case ARO_DUPLICATE:
    		PRINTF("ARO option status addr duplicate detected\n");
    		addr = uip_ds6_addr_lookup(&UIP_IP_BUF->destipaddr);
    		if(addr!=NULL) {
    		  if(uip_is_addr_linklocal(&addr->ipaddr)) {
    		    PRINTF("Contiki shutdown, ARO for link local address failed\n");
    		    exit(-1);
    		  }
    		  uip_ds6_addr_rm(addr);
    		}
    		goto discard;
    		break;
    	case ARO_ROUTER_CACHE_FULL:
    		PRINTF("ARO option status router cache full\n");
    		defrt = uip_ds6_defrt_lookup(&UIP_IP_BUF->srcipaddr);
    		if(defrt != NULL) {
    			uip_ds6_defrt_rm(defrt);
    		}
    		nbr = uip_ds6_nbr_lookup(&UIP_ND6_NA_BUF->tgtipaddr);
        if(nbr != NULL) {
          uip_ds6_nbr_rm(nbr);
        }
        //start at slow interval. If resend immediately we would spam, since we will get a RA every time..
        uip_ds6_start_rs_sending(MAX_RTR_SOLICITATION_INTERVAL);
    		goto discard;
    		break;
    	default:
    		PRINTF("ARO option status unknown error\n");
    		goto discard;
    		break;
    	}
     }
     break;
#endif /* LN_ROLE */
    default:
      PRINTF("ND option not supported in NA\n");
      break;
    }
    nd6_opt_offset += (UIP_ND6_OPT_HDR_BUF->len << 3);
  }
  addr = uip_ds6_addr_lookup(&UIP_ND6_NA_BUF->tgtipaddr);
  /* Message processing, including TLLAO if any */
  if(addr != NULL) {
#if UIP_ND6_DEF_MAXDADNS > 0
    if(addr->state == ADDR_TENTATIVE) {
      uip_ds6_dad_failed(addr);
    }
#endif /*UIP_ND6_DEF_MAXDADNS > 0 */
    PRINTF("NA received is bad\n");
    goto discard;
  } else {
    uip_lladdr_t *lladdr;
    nbr = uip_ds6_nbr_lookup(&UIP_ND6_NA_BUF->tgtipaddr);
    lladdr = (uip_lladdr_t *)uip_ds6_nbr_get_ll(nbr);
    if(nbr == NULL) {
      goto discard;
    }
    if(nd6_opt_llao != 0) {
      is_llchange =
        memcmp(&nd6_opt_llao[UIP_ND6_OPT_DATA_OFFSET], (void *)lladdr,
               UIP_LLADDR_LEN);
    }
    if(nbr->state == NBR_INCOMPLETE) {
      if(nd6_opt_llao == NULL) {
        goto discard;
      }
      memcpy(lladdr, &nd6_opt_llao[UIP_ND6_OPT_DATA_OFFSET],
	     UIP_LLADDR_LEN);
      if(is_solicited) {
        nbr->state = NBR_REACHABLE;
        nbr->nscount = 0;

        /* reachable time is stored in ms */
        stimer_set(&(nbr->reachable), uip_ds6_if.reachable_time / 1000);

      } else {
        nbr->state = NBR_STALE;
      }
      nbr->isrouter = is_router;
    } else {
      if(!is_override && is_llchange) {
        if(nbr->state == NBR_REACHABLE) {
          nbr->state = NBR_STALE;
        }
        goto discard;
      } else {
        if(is_override || (!is_override && nd6_opt_llao != 0 && !is_llchange)
           || nd6_opt_llao == 0) {
          if(nd6_opt_llao != 0) {
            memcpy(lladdr, &nd6_opt_llao[UIP_ND6_OPT_DATA_OFFSET],
		   UIP_LLADDR_LEN);
          }
          if(is_solicited) {
            nbr->state = NBR_REACHABLE;
            /* reachable time is stored in ms */
            stimer_set(&(nbr->reachable), uip_ds6_if.reachable_time / 1000);
          } else {
            if(nd6_opt_llao != 0 && is_llchange) {
              nbr->state = NBR_STALE;
            }
          }
        }
      }
      if(nbr->isrouter && !is_router) {
        defrt = uip_ds6_defrt_lookup(&UIP_IP_BUF->srcipaddr);
        if(defrt != NULL) {
          uip_ds6_defrt_rm(defrt);
        }
      }
#if LN_ROLE
      uint64_t remaingTime = remaingTime = nd6_opt_aro->lifetime*60;
      stimer_set(&nbr->sendns, remaingTime - remaingTime/NS_EARLY_WARNING_DIVIDER);
      nbr->nscount = 0;
      PRINTF("Set sendNs timer to : %ld seconds\n", remaingTime - remaingTime/NS_EARLY_WARNING_DIVIDER );
#if UIP_CONF_MLD
      uip_mldv1_schedule_report_all_addr();
#endif
#endif
      nbr->isrouter = is_router;
    }

    UIP_READ_RECEIVED_IPEI(&nbr->ipei);
    //memcpy(,UIP_IPEI,sizeof(ule6lo_IPEI_t));
#if FILTERED_BORDER_ROUTER
    nbr->interface_type = UIP_RECEIVED_INTERFACE_TYPE;
#endif
  }
#if UIP_CONF_IPV6_QUEUE_PKT
  /* The nbr is now reachable, check if we had buffered a pkt for it */
  /*if(nbr->queue_buf_len != 0) {
    uip_len = nbr->queue_buf_len;
    memcpy(UIP_IP_BUF, nbr->queue_buf, uip_len);
    nbr->queue_buf_len = 0;
    return;
    }*/
  if(uip_packetqueue_buflen(&nbr->packethandle) != 0) {
    uip_len = uip_packetqueue_buflen(&nbr->packethandle);
    memcpy(UIP_IP_BUF, uip_packetqueue_buf(&nbr->packethandle), uip_len);
    uip_packetqueue_free(&nbr->packethandle);
    return;
  }
  
#endif /*UIP_CONF_IPV6_QUEUE_PKT */
  
discard:
  uip_clear_buf();
  return;
}
//#endif /* UIP_ND6_SEND_NA */


#if UIP_CONF_ROUTER
#if UIP_ND6_SEND_RA
/*---------------------------------------------------------------------------*/
static void
rs_input(void)
{

  PRINTF("Received RS from");
  PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
  PRINTF("to");
  PRINT6ADDR(&UIP_IP_BUF->destipaddr);
  PRINTF("\n");
  UIP_STAT(++uip_stat.nd6.recv);

#if FILTERED_BORDER_ROUTER
  if (UIP_RECEIVED_INTERFACE_TYPE == UIP_INTERFACE_TYPE_ETH) {
	  PRINTF("RS arrived from ethernet, discarding it\n");
	  goto discard;
  }
#endif

#if UIP_CONF_IPV6_CHECKS
  /*
   * Check hop limit / icmp code 
   * target address must not be multicast
   * if the NA is solicited, dest must not be multicast
   */
  if((UIP_IP_BUF->ttl != UIP_ND6_HOP_LIMIT) || (UIP_ICMP_BUF->icode != 0)) {
    PRINTF("RS received is bad\n");
    goto discard;
  }
#endif /*UIP_CONF_IPV6_CHECKS */

  /* Only valid option is Source Link-Layer Address option any thing
     else is discarded */
  nd6_opt_offset = UIP_ND6_RS_LEN;
  nd6_opt_llao = NULL;

  while(uip_l3_icmp_hdr_len + nd6_opt_offset < uip_len) {
#if UIP_CONF_IPV6_CHECKS
    if(UIP_ND6_OPT_HDR_BUF->len == 0) {
      PRINTF("RS received is bad\n");
      goto discard;
    }
#endif /*UIP_CONF_IPV6_CHECKS */
    switch (UIP_ND6_OPT_HDR_BUF->type) {
    case UIP_ND6_OPT_SLLAO:
      nd6_opt_llao = (uint8_t *)UIP_ND6_OPT_HDR_BUF;
      break;
    default:
      PRINTF("ND option not supported in RS\n");
      break;
    }
    nd6_opt_offset += (UIP_ND6_OPT_HDR_BUF->len << 3);
  }
  /* Options processing: only SLLAO */
  if(nd6_opt_llao != NULL) {
#if UIP_CONF_IPV6_CHECKS
    if(uip_is_addr_unspecified(&UIP_IP_BUF->srcipaddr)) {
      PRINTF("RS received is bad\n");
      goto discard;
    } else {
#endif /*UIP_CONF_IPV6_CHECKS */
      uip_lladdr_t lladdr_aligned;
      extract_lladdr_aligned(&lladdr_aligned);
      if( UIP_RECEIVED_INTERFACE_TYPE == UIP_INTERFACE_TYPE_DECT && uip_ds6_nbr_ll_lookup(&lladdr_aligned)){
          //we are not allowed to update nbr list at rs when we already have the mac address in the nbr list..
        PRINTF("We already have this address in our NBR list..\n");
      }
      else if((nbr = uip_ds6_nbr_lookup(&UIP_IP_BUF->srcipaddr)) == NULL) {
        /* we need to add the neighbor */
        if( UIP_RECEIVED_INTERFACE_TYPE == UIP_INTERFACE_TYPE_DECT){
            nbr = uip_ds6_nbr_add(&UIP_IP_BUF->srcipaddr, &lladdr_aligned, 0, NBR_TENTATIVE,UIP_IPEI_RECEIVED, UIP_RECEIVED_INTERFACE_TYPE);
        }
        else{
          nbr = uip_ds6_nbr_add(&UIP_IP_BUF->srcipaddr, &lladdr_aligned, 0, NBR_STALE,UIP_IPEI_RECEIVED, UIP_RECEIVED_INTERFACE_TYPE);
        }
        if( nbr == NULL){
          //No more space in nbr list..
          //discard, since we have no one to answer to in nbr list...
          PRINTF("RS: No more space in nbr list.\n");
          goto discard;
        }
      } else {
        /* If LL address changed, set neighbor state to stale */
        if(memcmp(&nd6_opt_llao[UIP_ND6_OPT_DATA_OFFSET],
            uip_ds6_nbr_get_ll(nbr), UIP_LLADDR_LEN) != 0) {
          uip_ds6_nbr_t nbr_data = *nbr;
          uip_ds6_nbr_rm(nbr);
          if( UIP_RECEIVED_INTERFACE_TYPE == UIP_INTERFACE_TYPE_DECT){
            nbr = uip_ds6_nbr_add(&UIP_IP_BUF->srcipaddr, &lladdr_aligned, 0, NBR_TENTATIVE,UIP_IPEI_RECEIVED, UIP_RECEIVED_INTERFACE_TYPE);
          }else{
            nbr = uip_ds6_nbr_add(&UIP_IP_BUF->srcipaddr, &lladdr_aligned, 0, NBR_STALE,UIP_IPEI_RECEIVED,UIP_RECEIVED_INTERFACE_TYPE);
          }
          nbr->reachable = nbr_data.reachable;
          nbr->sendns = nbr_data.sendns;
          nbr->nscount = nbr_data.nscount;
        }
        nbr->isrouter = 0;
      }
#if UIP_CONF_IPV6_CHECKS
    }
#endif /*UIP_CONF_IPV6_CHECKS */
  }

  /* Send a sollicited RA */
  uip_nd6_ra_output(&UIP_IP_BUF->srcipaddr);
  return;
discard:
  uip_clear_buf();
  return;
}

/*---------------------------------------------------------------------------*/
void
uip_nd6_ra_output(uip_ipaddr_t * dest)
{
  uip_ds6_addr_t *matchaddr = NULL;
  UIP_IP_BUF->vtc = 0x60;
  UIP_IP_BUF->tcflow = 0;
  UIP_IP_BUF->flow = 0;
  UIP_IP_BUF->proto = UIP_PROTO_ICMP6;
  UIP_IP_BUF->ttl = UIP_ND6_HOP_LIMIT;

  if(dest == NULL) {
    uip_create_linklocal_allnodes_mcast(&UIP_IP_BUF->destipaddr);
  } else {
    /* For sollicited RA */
    uip_ipaddr_copy(&UIP_IP_BUF->destipaddr, dest);
  }
  //RA should always be sent with link local, since the node does not keep the global address for the router in the neighbour list..
  matchaddr = uip_ds6_get_link_local(ADDR_PREFERRED);
  if( matchaddr != NULL ){
      uip_ipaddr_copy(&UIP_IP_BUF->srcipaddr, &matchaddr->ipaddr);
  }else{
      uip_ds6_select_src(&UIP_IP_BUF->srcipaddr, &UIP_IP_BUF->destipaddr);
  }

  UIP_ICMP_BUF->type = ICMP6_RA;
  UIP_ICMP_BUF->icode = 0;

  UIP_ND6_RA_BUF->cur_ttl = uip_ds6_if.cur_hop_limit;

  UIP_ND6_RA_BUF->flags_reserved =
    (UIP_ND6_M_FLAG << 7) | (UIP_ND6_O_FLAG << 6);

  UIP_ND6_RA_BUF->router_lifetime = uip_htons(UIP_ND6_ROUTER_LIFETIME);
  //UIP_ND6_RA_BUF->reachable_time = uip_htonl(uip_ds6_if.reachable_time);
  //UIP_ND6_RA_BUF->retrans_timer = uip_htonl(uip_ds6_if.retrans_timer);
  UIP_ND6_RA_BUF->reachable_time = 0;
  UIP_ND6_RA_BUF->retrans_timer = 0;

  uip_len = UIP_IPH_LEN + UIP_ICMPH_LEN + UIP_ND6_RA_LEN;
  nd6_opt_offset = UIP_ND6_RA_LEN;


  /* Prefix list */
  for(prefix = uip_ds6_prefix_list;
      prefix < uip_ds6_prefix_list + UIP_DS6_PREFIX_NB; prefix++) {
    if((prefix->isused) && (prefix->advertise)) {
      UIP_ND6_OPT_PREFIX_BUF->type = UIP_ND6_OPT_PREFIX_INFO;
      UIP_ND6_OPT_PREFIX_BUF->len = UIP_ND6_OPT_PREFIX_INFO_LEN / 8;
      UIP_ND6_OPT_PREFIX_BUF->preflen = prefix->length;
      UIP_ND6_OPT_PREFIX_BUF->flagsreserved1 = prefix->l_a_reserved;
      UIP_ND6_OPT_PREFIX_BUF->validlt = uip_htonl(prefix->vlifetime);
      UIP_ND6_OPT_PREFIX_BUF->preferredlt = uip_htonl(prefix->plifetime);
      UIP_ND6_OPT_PREFIX_BUF->reserved2 = 0;
      uip_ipaddr_copy(&(UIP_ND6_OPT_PREFIX_BUF->prefix), &(prefix->ipaddr));
      nd6_opt_offset += UIP_ND6_OPT_PREFIX_INFO_LEN;
      uip_len += UIP_ND6_OPT_PREFIX_INFO_LEN;
    }
  }

  /* Source link-layer option */
  create_llao((uint8_t *)UIP_ND6_OPT_HDR_BUF, UIP_ND6_OPT_SLLAO);

  uip_len += UIP_ND6_OPT_LLAO_LEN;
  nd6_opt_offset += UIP_ND6_OPT_LLAO_LEN;

  /* MTU */
  UIP_ND6_OPT_MTU_BUF->type = UIP_ND6_OPT_MTU;
  UIP_ND6_OPT_MTU_BUF->len = UIP_ND6_OPT_MTU_LEN >> 3;
  UIP_ND6_OPT_MTU_BUF->reserved = 0;
  //UIP_ND6_OPT_MTU_BUF->mtu = uip_htonl(uip_ds6_if.link_mtu);
  UIP_ND6_OPT_MTU_BUF->mtu = uip_htonl(1500);

  uip_len += UIP_ND6_OPT_MTU_LEN;
  nd6_opt_offset += UIP_ND6_OPT_MTU_LEN;

#if UIP_ND6_RA_RDNSS
  if(uip_nameserver_count() > 0) {
    uint8_t i = 0;
    uip_ipaddr_t *ip = &UIP_ND6_OPT_RDNSS_BUF->ip;
    uip_ipaddr_t *dns = NULL;
    UIP_ND6_OPT_RDNSS_BUF->type = UIP_ND6_OPT_RDNSS;
    UIP_ND6_OPT_RDNSS_BUF->reserved = 0;
    UIP_ND6_OPT_RDNSS_BUF->lifetime = uip_nameserver_next_expiration();
    if(UIP_ND6_OPT_RDNSS_BUF->lifetime != UIP_NAMESERVER_INFINITE_LIFETIME) {
      UIP_ND6_OPT_RDNSS_BUF->lifetime -= clock_seconds();
    }
    UIP_ND6_OPT_RDNSS_BUF->lifetime= uip_htonl(UIP_ND6_OPT_RDNSS_BUF->lifetime);
    while((dns = uip_nameserver_get(i)) != NULL) {
      uip_ipaddr_copy(ip++, dns);
      i++;
    }
    UIP_ND6_OPT_RDNSS_BUF->len = UIP_ND6_OPT_RDNSS_LEN + (i << 1);
    PRINTF("%d nameservers reported\n", i);
    uip_len += UIP_ND6_OPT_RDNSS_BUF->len << 3;
    nd6_opt_offset += UIP_ND6_OPT_RDNSS_BUF->len << 3;
  }
#endif /* UIP_ND6_RA_RDNSS */
#if SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS > 0
  sicslowpan_addr_context* addr_contexts = getAddrContexts();
  for(uint8_t i = 0; i < SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS; i++) {
     sicslowpan_addr_context addr = addr_contexts[i];
     if((addr.length > 0)) {
        UIP_ND6_OPT_6CO_BUF->type = UIP_ND6_OPT_6CO;
        UIP_ND6_OPT_6CO_BUF->len = addr.length;
        UIP_ND6_OPT_6CO_BUF->contextlen = addr.contextlength;
        UIP_ND6_OPT_6CO_BUF->flags_cid = addr.flags_cid;
        UIP_ND6_OPT_6CO_BUF->reserved = 0x0000;
        UIP_ND6_OPT_6CO_BUF->v_lifetime = uip_htons(addr.v_lifetime);
        memset(UIP_ND6_OPT_6CO_BUF->prefix, 0x00, addr.length * 0x08);
        memcpy(UIP_ND6_OPT_6CO_BUF->prefix, addr.prefix, addr.contextlength/0x08);
        nd6_opt_offset += addr.length<<3;
        uip_len += (addr.length)<<3;
     }
  }
#endif//#if SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS > 0
#if UIP_CONF_DS6_ROUTE_INFORMATION
  for(rtinfo = uip_ds6_route_info_list;
		  rtinfo < uip_ds6_route_info_list + UIP_DS6_ROUTE_INFO_NB; rtinfo++) {
	  if((rtinfo->isused)) {
		  UIP_ND6_OPT_ROUTE_BUF->type = UIP_ND6_OPT_ROUTE_INFO;
		  UIP_ND6_OPT_ROUTE_BUF->len =(rtinfo->length >> 6) + 1 ;
		  UIP_ND6_OPT_ROUTE_BUF->preflen = rtinfo->length;
		  UIP_ND6_OPT_ROUTE_BUF->flagsreserved = rtinfo->flags;
		  UIP_ND6_OPT_ROUTE_BUF->rlifetime = uip_htonl(rtinfo->lifetime);
		  uip_ipaddr_copy(&(UIP_ND6_OPT_ROUTE_BUF->prefix), &(rtinfo->ipaddr));
		  nd6_opt_offset += ((rtinfo->length >> 6) + 1)<<3;
		  uip_len += ((rtinfo->length >> 6) + 1)<<3;
	  }
  }
#endif /* UIP_CONF_DS6_ROUTE_INFORMATION */

  UIP_IP_BUF->len[0] = ((uip_len - UIP_IPH_LEN) >> 8);
  UIP_IP_BUF->len[1] = ((uip_len - UIP_IPH_LEN) & 0xff);

  /*ICMP checksum */
  UIP_ICMP_BUF->icmpchksum = 0;
  UIP_ICMP_BUF->icmpchksum = ~uip_icmp6chksum();

  UIP_STAT(++uip_stat.nd6.sent);
  PRINTF("Sending RA to");
  PRINT6ADDR(&UIP_IP_BUF->destipaddr);
  PRINTF("from");
  PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
  PRINTF("\n");

#if FILTERED_BORDER_ROUTER
  // Limiting output to wireless interface. Not router on ETH side.
  *UIP_TRANSMITTED_FLAGS = UIP_INTERFACE_TYPE_DECT;
#endif
  return;
}
#endif /* UIP_ND6_SEND_RA */
#endif /* UIP_CONF_ROUTER */

#if !UIP_CONF_ROUTER || FILTERED_BORDER_ROUTER
/*---------------------------------------------------------------------------*/
void
uip_nd6_rs_output(void)
{
  UIP_IP_BUF->vtc = 0x60;
  UIP_IP_BUF->tcflow = 0;
  UIP_IP_BUF->flow = 0;
  UIP_IP_BUF->proto = UIP_PROTO_ICMP6;
  UIP_IP_BUF->ttl = UIP_ND6_HOP_LIMIT;
  uip_create_linklocal_allrouters_mcast(&UIP_IP_BUF->destipaddr);
  //find src for rs. This depends on which addr that has a timeout.. If none global use link local.
  uip_ds6_select_src(&UIP_IP_BUF->srcipaddr, &UIP_IP_BUF->destipaddr);
#if LN_ROLE
  static uip_ds6_addr_t *locaddr;
  for(locaddr = uip_ds6_if.addr_list;
        locaddr < uip_ds6_if.addr_list + UIP_DS6_ADDR_NB; locaddr++) {
      if(locaddr->isused) {
        if(!locaddr->isinfinite &&
            stimer_remaining(&locaddr->vlifetime) < locaddr->vlifetime.interval / RS_EARLY_WARNING_DIVIDER ) {
          uip_ipaddr_copy(&UIP_IP_BUF->srcipaddr, &locaddr->ipaddr);
          break;//select first with an timeout. next will be send later.
        }
      }
    }
#endif

  UIP_ICMP_BUF->type = ICMP6_RS;
  UIP_ICMP_BUF->icode = 0;
  UIP_ICMP_BUF->reserved = 0;
  UIP_IP_BUF->len[0] = 0;       /* length will not be more than 255 */

  if(uip_is_addr_unspecified(&UIP_IP_BUF->srcipaddr)) {
    UIP_IP_BUF->len[1] = UIP_ICMPH_LEN + UIP_ND6_RS_LEN;
    uip_len = uip_l3_icmp_hdr_len + UIP_ND6_RS_LEN;
  } else {
    uip_len = uip_l3_icmp_hdr_len + UIP_ND6_RS_LEN + UIP_ND6_OPT_LLAO_LEN;
    UIP_IP_BUF->len[1] =
      UIP_ICMPH_LEN + UIP_ND6_RS_LEN + UIP_ND6_OPT_LLAO_LEN;

    create_llao(&uip_buf[uip_l2_l3_icmp_hdr_len + UIP_ND6_RS_LEN],
		UIP_ND6_OPT_SLLAO);
  }

  UIP_ICMP_BUF->icmpchksum = 0;
  UIP_ICMP_BUF->icmpchksum = ~uip_icmp6chksum();

  UIP_STAT(++uip_stat.nd6.sent);
  PRINTF("Sendin RS to");
  PRINT6ADDR(&UIP_IP_BUF->destipaddr);
  PRINTF("from");
  PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
  PRINTF("\n");

#if FILTERED_BORDER_ROUTER
  // Limiting output to ethernet interface. We are router on wireless side.
  *UIP_TRANSMITTED_FLAGS = UIP_INTERFACE_TYPE_ETH;
#endif
  return;
}
/*---------------------------------------------------------------------------*/
/*
 * Process a Router Advertisement
 *
 * - Possible actions when receiving a RA: add router to router list,
 *   recalculate reachable time, update link hop limit, update retrans timer.
 * - If MTU option: update MTU.
 * - If SLLAO option: update entry in neighbor cache
 * - If prefix option: start autoconf, add prefix to prefix list
 */
void
ra_input(void)
{
  uip_ipaddr_t ns_source;
  uip_ipaddr_t ns_dest;
  uip_ipaddr_t ns_target;
  uip_ipaddr_t ns_random;

  PRINTF("Received RA from");
  PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
  PRINTF("to");
  PRINT6ADDR(&UIP_IP_BUF->destipaddr);
  PRINTF("\n");
  UIP_STAT(++uip_stat.nd6.recv);

  //prepare random iid..
  for( uint8_t i = 0; i < 8; i+=2) {
    uint16_t rand = random_rand();
   //PRINTF("Random %d\n", rand);
    memcpy( &ns_random.u8[i], &rand, sizeof(rand));
  }

#if UIP_CONF_IPV6_CHECKS
  if((UIP_IP_BUF->ttl != UIP_ND6_HOP_LIMIT) ||
     (!uip_is_addr_linklocal(&UIP_IP_BUF->srcipaddr)) ||
     (UIP_ICMP_BUF->icode != 0)) {
    PRINTF("RA received is bad");
    goto discard;
  }
#endif /*UIP_CONF_IPV6_CHECKS */

  if(UIP_ND6_RA_BUF->cur_ttl != 0) {
    uip_ds6_if.cur_hop_limit = UIP_ND6_RA_BUF->cur_ttl;
    PRINTF("uip_ds6_if.cur_hop_limit %u\n", uip_ds6_if.cur_hop_limit);
  }

  //always 0 if LN..
  if(UIP_ND6_RA_BUF->reachable_time != 0) {
    if(uip_ds6_if.base_reachable_time !=
       uip_ntohl(UIP_ND6_RA_BUF->reachable_time)) {
      uip_ds6_if.base_reachable_time = uip_ntohl(UIP_ND6_RA_BUF->reachable_time);
      uip_ds6_if.reachable_time = uip_ds6_compute_reachable_time();
    }
  }
#ifdef LN_ROLE
  else{
    uip_ds6_if.base_reachable_time = UIP_ND6_REACHABLE_TIME;
    uip_ds6_if.reachable_time = uip_ds6_compute_reachable_time();
    PRINTF("RA reachable time %d\n", uip_ds6_if.reachable_time);
  }
#endif
  if(UIP_ND6_RA_BUF->retrans_timer != 0) {
    uip_ds6_if.retrans_timer = uip_ntohl(UIP_ND6_RA_BUF->retrans_timer);
  }

  /* Options processing */
  nd6_opt_offset = UIP_ND6_RA_LEN;
  while(uip_l3_icmp_hdr_len + nd6_opt_offset < uip_len) {
    if(UIP_ND6_OPT_HDR_BUF->len == 0) {
      PRINTF("RA received is bad");
      goto discard;
    }
    switch (UIP_ND6_OPT_HDR_BUF->type) {
    case UIP_ND6_OPT_SLLAO:
      PRINTF("Processing SLLAO option in RA\n");
      nd6_opt_llao = (uint8_t *) UIP_ND6_OPT_HDR_BUF;
      nbr = uip_ds6_nbr_lookup(&UIP_IP_BUF->srcipaddr);
      if(nbr == NULL) {
        uip_lladdr_t lladdr_aligned;
        extract_lladdr_aligned(&lladdr_aligned);
#if FILTERED_BORDER_ROUTER
        nbr = uip_ds6_nbr_add(&UIP_IP_BUF->srcipaddr, &lladdr_aligned, 1, NBR_STALE, UIP_IPEI_RECEIVED, UIP_RECEIVED_INTERFACE_TYPE);
#else
        nbr = uip_ds6_nbr_add(&UIP_IP_BUF->srcipaddr, &lladdr_aligned, 1, NBR_STALE, UIP_IPEI_RECEIVED);
#endif
      } else {
        uip_lladdr_t *lladdr = (uip_lladdr_t *)uip_ds6_nbr_get_ll(nbr);
        if(nbr->state == NBR_INCOMPLETE) {
          nbr->state = NBR_STALE;
        }
        if(memcmp(&nd6_opt_llao[UIP_ND6_OPT_DATA_OFFSET],
		  lladdr, UIP_LLADDR_LEN) != 0) {
          memcpy(lladdr, &nd6_opt_llao[UIP_ND6_OPT_DATA_OFFSET],
		 UIP_LLADDR_LEN);
          nbr->state = NBR_STALE;
        }
        nbr->isrouter = 1;
#if FILTERED_BORDER_ROUTER
        nbr->interface_type = UIP_RECEIVED_INTERFACE_TYPE;
#endif
      }
      break;
    case UIP_ND6_OPT_MTU:
      PRINTF("Processing MTU option in RA\n");
      uip_ds6_if.link_mtu =
        uip_ntohl(((uip_nd6_opt_mtu *) UIP_ND6_OPT_HDR_BUF)->mtu);
      break;
    case UIP_ND6_OPT_PREFIX_INFO:
      PRINTF("Processing PREFIX option in RA\n");
      nd6_opt_prefix_info = (uip_nd6_opt_prefix_info *) UIP_ND6_OPT_HDR_BUF;
      if((uip_ntohl(nd6_opt_prefix_info->validlt) >=
          uip_ntohl(nd6_opt_prefix_info->preferredlt))
         && (!uip_is_addr_linklocal(&nd6_opt_prefix_info->prefix))) {
        /* on-link flag related processing */
        if(nd6_opt_prefix_info->flagsreserved1 & UIP_ND6_RA_FLAG_ONLINK) {
        	PRINTF("ON LINK\n");
#if !FILTERED_BORDER_ROUTER
        	prefix =
            uip_ds6_prefix_lookup(&nd6_opt_prefix_info->prefix,
                                  nd6_opt_prefix_info->preflen);
          if(prefix == NULL) {
#else
        	  prefix_non_router =
        	              uip_ds6_prefix_lookup_non_router(&nd6_opt_prefix_info->prefix,
        	                                    nd6_opt_prefix_info->preflen);
        	            if(prefix_non_router == NULL) {
#endif
        	  if(nd6_opt_prefix_info->validlt != 0) {
        		  if(nd6_opt_prefix_info->validlt != UIP_ND6_INFINITE_LIFETIME) {
#if !FILTERED_BORDER_ROUTER
        			  prefix = uip_ds6_prefix_add(&nd6_opt_prefix_info->prefix,
                                            nd6_opt_prefix_info->preflen,
                                            uip_ntohl(nd6_opt_prefix_info->
                                                  validlt));
#else
        			  prefix_non_router = uip_ds6_prefix_add_non_router(&nd6_opt_prefix_info->prefix,
        			                                              nd6_opt_prefix_info->preflen,
        			                                              uip_ntohl(nd6_opt_prefix_info->
        			                                                    validlt));
        			  prefix = uip_ds6_prefix_add(&nd6_opt_prefix_info->prefix,
        			                                              nd6_opt_prefix_info->preflen,
        			                                              1, //Advertise on DECT side
        			                                              UIP_ND6_RA_FLAG_AUTONOMOUS,
        			                                              DECT_VLIFE_TIME,
        			                                              DECT_PLIFE_TIME);
#endif
              } else {
#if !FILTERED_BORDER_ROUTER
            	  prefix = uip_ds6_prefix_add(&nd6_opt_prefix_info->prefix,
                                            nd6_opt_prefix_info->preflen, 0);
#else
  	  	  	  	  prefix_non_router = uip_ds6_prefix_add_non_router(&nd6_opt_prefix_info->prefix,
                                            nd6_opt_prefix_info->preflen, 0);
  	  	  	  	  prefix = uip_ds6_prefix_add(&nd6_opt_prefix_info->prefix,
  	  	  	  			  nd6_opt_prefix_info->preflen,
  	  	  	  			  1, //Advertise on DECT side
  	  	  	  			  UIP_ND6_RA_FLAG_AUTONOMOUS,
  	  	  	  			  DECT_VLIFE_TIME,
  	  	  	  			  DECT_PLIFE_TIME);
#endif
              }
#if FILTERED_BORDER_ROUTER
              //JJO: Trigger sending of RA to Wireless interface.
              // Note that this is not really a proper solicited message, but just a trigger to broadcast it
              // Works as we want in this case, but if it becomes properly solicited in the future it could be problematic.
              if(prefix_non_router!=NULL) {
            	  uip_ds6_send_ra_sollicited();
              }
#endif
            }
          } else {
        	  switch (nd6_opt_prefix_info->validlt) {
            case 0:
#if !FILTERED_BORDER_ROUTER
              uip_ds6_prefix_rm(prefix);
#else
              //JJO: Trigger sending of RA to Wireless interface.
              // Note that this is not really a proper solicited message, but just a trigger to broadcast it
              // Works as we want in this case, but if it becomes properly solicited in the future it could be problematic.
              uip_ds6_prefix_rm_non_router(prefix_non_router);
              prefix = uip_ds6_prefix_lookup(&prefix_non_router->ipaddr,prefix_non_router->length);
              uip_ds6_prefix_rm(prefix);
              uip_ds6_send_ra_sollicited();
#endif
              break;
            case UIP_ND6_INFINITE_LIFETIME:
#if !FILTERED_BORDER_ROUTER
              prefix->isinfinite = 1;
#else
              prefix_non_router->isinfinite = 1;
#endif
              break;
            default:
              PRINTF("Updating timer of prefix");
#if !FILTERED_BORDER_ROUTER
              PRINT6ADDR(&prefix->ipaddr);
              PRINTF("new value %lu\n", uip_ntohl(nd6_opt_prefix_info->validlt));
              stimer_set(&prefix->vlifetime,
                         uip_ntohl(nd6_opt_prefix_info->validlt));
              prefix->isinfinite = 0;
#else
              PRINT6ADDR(&prefix_non_router->ipaddr);
              PRINTF("new value %lu\n", uip_ntohl(nd6_opt_prefix_info->validlt));
              stimer_set(&prefix_non_router->vlifetime,
            		  uip_ntohl(nd6_opt_prefix_info->validlt));
              prefix_non_router->isinfinite = 0;
#endif
              break;
            }
          }
        }
          else {
            PRINTF("OFF LINK\n");
          }
        /* End of on-link flag related processing */
        /* autonomous flag related processing */
        if((nd6_opt_prefix_info->flagsreserved1 & UIP_ND6_RA_FLAG_AUTONOMOUS)
           && (nd6_opt_prefix_info->validlt != 0)
           && (nd6_opt_prefix_info->preflen == UIP_DEFAULT_PREFIX_LEN)) {
          prefix = uip_ds6_prefix_lookup(&nd6_opt_prefix_info->prefix,
              nd6_opt_prefix_info->preflen);
          if(uip_ds6_list_loop
              ((uip_ds6_element_t *)uip_ds6_if.addr_list, UIP_DS6_ADDR_NB,
               sizeof(uip_ds6_addr_t), &nd6_opt_prefix_info->prefix, 64,
               (uip_ds6_element_t **)&ipaddr) == FOUND) {
              PRINTF("PREFIX already added\n");
              //Use correct prefix. Else it will alwys use the same ipaddr.
              memcpy(&ipaddr, &nd6_opt_prefix_info->prefix, 8);
              
            }
            else {
              uip_ipaddr_copy(&ipaddr, &nd6_opt_prefix_info->prefix);
              memcpy( &ipaddr.u8[8], &ns_random, 8);
          }
          addr = uip_ds6_addr_lookup(&ipaddr);
          if((addr != NULL) && (addr->type == ADDR_AUTOCONF)) {
            if(nd6_opt_prefix_info->validlt != UIP_ND6_INFINITE_LIFETIME) {
              /* The processing below is defined in RFC4862 section 5.5.3 e */
              if((uip_ntohl(nd6_opt_prefix_info->validlt) > 2 * 60 * 60) ||
                 (uip_ntohl(nd6_opt_prefix_info->validlt) >
                  stimer_remaining(&addr->vlifetime))) {
                PRINTF("Updating timer of address");
                PRINT6ADDR(&addr->ipaddr);
                PRINTF("new value %lu\n",
                       uip_ntohl(nd6_opt_prefix_info->validlt));
                stimer_set(&addr->vlifetime,
                           uip_ntohl(nd6_opt_prefix_info->validlt));
              } else {
                stimer_set(&addr->vlifetime, 2 * 60 * 60);
                PRINTF("Updating timer of address ");
                PRINT6ADDR(&addr->ipaddr);
                PRINTF("new value %lu\n", (unsigned long)(2 * 60 * 60));
              }
              addr->isinfinite = 0;
            } else {
              addr->isinfinite = 1;
            }
          } else {
#if FILTERED_BORDER_ROUTER
        	printf("Tentative global IPv6 address ");
			uip_debug_ipaddr_print(&ipaddr);
			printf("\n");
#endif
            if(uip_ntohl(nd6_opt_prefix_info->validlt) ==
               UIP_ND6_INFINITE_LIFETIME) {
              uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);
            } else {
              uip_ds6_addr_add(&ipaddr, uip_ntohl(nd6_opt_prefix_info->validlt),
                               ADDR_AUTOCONF);
            }
          }
        }
        /* End of autonomous flag related processing */
      }
      break;
#if UIP_ND6_RA_RDNSS
    case UIP_ND6_OPT_RDNSS:
      if(UIP_ND6_RA_BUF->flags_reserved & (UIP_ND6_O_FLAG << 6)) {
        PRINTF("Processing RDNSS option\n");
        uint8_t naddr = (UIP_ND6_OPT_RDNSS_BUF->len - 1) / 2;
        uip_ipaddr_t *ip = (uip_ipaddr_t *)(&UIP_ND6_OPT_RDNSS_BUF->ip);
        PRINTF("got %d nameservers\n", naddr);
        while(naddr-- > 0) {
          PRINTF(" nameserver: ");
          PRINT6ADDR(ip);
          PRINTF(" lifetime: %lx\n", uip_ntohl(UIP_ND6_OPT_RDNSS_BUF->lifetime));
          uip_nameserver_update(ip, uip_ntohl(UIP_ND6_OPT_RDNSS_BUF->lifetime));
          ip++;
        }
      }
      break;
#endif /* UIP_ND6_RA_RDNSS */
#if SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS > 0
    case UIP_ND6_OPT_6CO:
        {
            nd6_opt_6CO = (uip_nd6_opt_6CO *) UIP_ND6_OPT_HDR_BUF;

            uint8_t cid = (UIP_ND6_OPT_6CO_BUF->flags_cid & UIP_ND6_6CO_CID);
            PRINTF(" 6CO context received %d\n", cid);
            if( uip_ntohs(UIP_ND6_OPT_6CO_BUF->v_lifetime) != 0){
                sicslowpan_addr_context addr_context;
                addr_context.flags_cid = UIP_ND6_OPT_6CO_BUF->flags_cid;
                addr_context.length = UIP_ND6_OPT_6CO_BUF->len;
                addr_context.contextlength = UIP_ND6_OPT_6CO_BUF->contextlen;
                addr_context.v_lifetime = uip_ntohs(UIP_ND6_OPT_6CO_BUF->v_lifetime);
                memset(addr_context.prefix, 0x00, sizeof(addr_context.prefix));
                memcpy(addr_context.prefix, UIP_ND6_OPT_6CO_BUF->prefix, UIP_ND6_OPT_6CO_BUF->contextlen/0x08);
                setAddrContextWithCid(&addr_context, cid);
                setTimerAtCid(cid);
            }
            else {
                //remove from list immediately when lifetime == 0
                removeAddrContextWithCid(cid);
            }
        }
        break;
#endif//#if SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS > 0
    default:
      PRINTF("ND option not supported in RA\n");
      break;
    }
    nd6_opt_offset += (UIP_ND6_OPT_HDR_BUF->len << 3);
  }

  defrt = uip_ds6_defrt_lookup(&UIP_IP_BUF->srcipaddr);
  if(UIP_ND6_RA_BUF->router_lifetime != 0) {
    if(nbr != NULL) {
      nbr->isrouter = 1;
    }
    if(defrt == NULL) {
      uip_ds6_defrt_add(&UIP_IP_BUF->srcipaddr,
                        (unsigned
                         long)(uip_ntohs(UIP_ND6_RA_BUF->router_lifetime)));
    } else {
      stimer_set(&(defrt->lifetime),
                 (unsigned long)(uip_ntohs(UIP_ND6_RA_BUF->router_lifetime)));
    }
  } else {
    if(defrt != NULL) {
      uip_ds6_defrt_rm(defrt);
    }
  }

#if LN_ROLE
  if(nbr != NULL) {
    /* update reachable time. This is okay since we have received a ra and is therefore reachable..*/
    stimer_set(&(nbr->reachable), uip_ds6_if.reachable_time / 1000);
  }
  uip_ds6_stop_rs_sending();
#endif

#if UIP_CONF_IPV6_QUEUE_PKT
  /* If the nbr just became reachable (e.g. it was in NBR_INCOMPLETE state
   * and we got a SLLAO), check if we had buffered a pkt for it */
  /*  if((nbr != NULL) && (nbr->queue_buf_len != 0)) {
    uip_len = nbr->queue_buf_len;
    memcpy(UIP_IP_BUF, nbr->queue_buf, uip_len);
    nbr->queue_buf_len = 0;
    return;
    }*/
  if(nbr != NULL && uip_packetqueue_buflen(&nbr->packethandle) != 0) {
    uip_len = uip_packetqueue_buflen(&nbr->packethandle);
    memcpy(UIP_IP_BUF, uip_packetqueue_buf(&nbr->packethandle), uip_len);
    uip_packetqueue_free(&nbr->packethandle);
    return;
  }

#endif /*UIP_CONF_IPV6_QUEUE_PKT */

discard:
  uip_clear_buf();
  return;
}
#endif /* !UIP_CONF_ROUTER */
/*------------------------------------------------------------------*/
/* ICMPv6 input handlers */
#if UIP_ND6_SEND_NA
UIP_ICMP6_HANDLER(ns_input_handler, ICMP6_NS, UIP_ICMP6_HANDLER_CODE_ANY,
                  ns_input);
#endif
UIP_ICMP6_HANDLER(na_input_handler, ICMP6_NA, UIP_ICMP6_HANDLER_CODE_ANY,
                  na_input);

#if UIP_CONF_ROUTER && UIP_ND6_SEND_RA
UIP_ICMP6_HANDLER(rs_input_handler, ICMP6_RS, UIP_ICMP6_HANDLER_CODE_ANY,
                  rs_input);
#endif

#if !UIP_CONF_ROUTER || FILTERED_BORDER_ROUTER
UIP_ICMP6_HANDLER(ra_input_handler, ICMP6_RA, UIP_ICMP6_HANDLER_CODE_ANY,
                  ra_input);
#endif
#if UIP_CONF_MLD
UIP_ICMP6_HANDLER(ml_query_input_handler, ICMP6_ML_QUERY, UIP_ICMP6_HANDLER_CODE_ANY,
                  uip_icmp6_ml_query_input);
UIP_ICMP6_HANDLER(ml_report_input_handler, ICMP6_ML_REPORT, UIP_ICMP6_HANDLER_CODE_ANY,
                  uip_icmp6_ml_report_input);
UIP_ICMP6_HANDLER(ml_done_input_handler, ICMP6_ML_DONE, UIP_ICMP6_HANDLER_CODE_ANY,
                  uip_icmp6_ml_done_input);
#endif
/*---------------------------------------------------------------------------*/
void
uip_nd6_init()
{

#if UIP_ND6_SEND_NA
  /* Only handle NSs if we are prepared to send out NAs */
  uip_icmp6_register_input_handler(&ns_input_handler);

  /*
   * Only handle NAs if we are prepared to send out NAs.
   * This is perhaps logically incorrect, but this condition was present in
   * uip_process and we keep it until proven wrong
   */
#endif
  uip_icmp6_register_input_handler(&na_input_handler);


#if UIP_CONF_ROUTER && UIP_ND6_SEND_RA
  /* Only accept RS if we are a router and happy to send out RAs */
  uip_icmp6_register_input_handler(&rs_input_handler);
#endif

#if !UIP_CONF_ROUTER || FILTERED_BORDER_ROUTER
  /* Only process RAs if we are not a router */
  // Our we are transparent bridge using interface seperation filter.
  uip_icmp6_register_input_handler(&ra_input_handler);
#endif
#if UIP_CONF_MLD
  uip_icmp6_register_input_handler(&ml_query_input_handler);
  uip_icmp6_register_input_handler(&ml_report_input_handler);
  uip_icmp6_register_input_handler(&ml_done_input_handler);
#endif
}
/*---------------------------------------------------------------------------*/
 /** @} */

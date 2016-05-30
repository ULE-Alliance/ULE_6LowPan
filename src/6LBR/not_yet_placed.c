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

#include <ipv6/uip-ds6.h>
#include <ip/uiplib.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/** Return string containing formatted IP6 address.
 *  NOTE: This function is NOT re-entrant!
 * @param addr
 * @return
 */
char *ip6AddrToStr(const uip_ipaddr_t *addr)
{
    static char str[40];
    snprintf(str, sizeof(str), "%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x",
        ((u8_t *)addr)[0], ((u8_t *)addr)[1], ((u8_t *)addr)[2], ((u8_t *)addr)[3],
        ((u8_t *)addr)[4], ((u8_t *)addr)[5], ((u8_t *)addr)[6], ((u8_t *)addr)[7],
        ((u8_t *)addr)[8], ((u8_t *)addr)[9], ((u8_t *)addr)[10], ((u8_t *)addr)[11],
        ((u8_t *)addr)[12], ((u8_t *)addr)[13], ((u8_t *)addr)[14], ((u8_t *)addr)[15]);
    return str;
}

void sixLowpanBorderRouterInit(const char *addrPrefix)
{
    printf("sixLowpanBorderRouterInit(%s)\n", addrPrefix);

    uip_ipaddr_t ipaddr;
    int enable_send_ra; // Billy claims to be Declared in uip-ds6.h, defined in uip-ds6.c

    // Set the IPv6 prefix supplied by caller
    enable_send_ra = uiplib_ipaddrconv(addrPrefix, &ipaddr);

    if (enable_send_ra)
    {
        // If IPv6 prefix is non-zero then we assume we are a router
        if (ipaddr.u16[0] != 0 || ipaddr.u16[1] != 0 || ipaddr.u16[2] != 0 || ipaddr.u16[3] != 0)
        {
            if (ipaddr.u16[7])
                enable_send_ra = ipaddr.u16[7] = 0;

#if UIP_CONF_ROUTER
            // Set the router prefix to use in RA:
            uip_ds6_prefix_add(&ipaddr, UIP_DEFAULT_PREFIX_LEN,
                    1, // advertise
                    //UIP_ND6_RA_FLAG_ONLINK,
                    UIP_ND6_RA_FLAG_AUTONOMOUS ,
                    900, // vlifetime
                    900 // plifetime
                    );
#else
            uip_ds6_prefix_add(&ipaddr, UIP_DEFAULT_PREFIX_LEN, 0);
#endif
/*#if !UIP_CONF_IPV6_RPL
            // Set the link-layer address (IID) obtained earlier
            uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
            uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);
#endif*/
        }
        // Set the link-layer address (IID) obtained earlier
        uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
        uip_ds6_addr_add(&ipaddr, 0, ADDR_MANUAL);
        printf("Our IPv6 address = %s\n", ip6AddrToStr(&ipaddr));
        // Set address context to be used for 6LoWPAN header compression
        //sicslowpan_set_context((u8_t *) &ipaddr); // Skipped for now, Was included in billy

        // Make a copy of our IPv6 address which we use for neighbor registration
        // (and to compare against in sixLowpanAssertIpAddr())
        //memcpy(&ourIpAddr, &ipaddr, sizeof(ipaddr));	//Memcpy skipped for now. Was included in billy.

    }
}

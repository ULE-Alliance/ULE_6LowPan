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


#ifndef ULE6LOGI_6LBR_H_
#define ULE6LOGI_6LBR_H_
/****************************************************************************
*                               Include Files                               *
****************************************************************************/
#include "ule6lo_types_6LBR.h"
#include <ipv6/sicslowpan.h>

/****************************************************************************
*                             Macro definitions                             *
****************************************************************************/

/****************************************************************************
*                   Enumerations/Type definitions/Structs                   *
****************************************************************************/



/****************************************************************************
*                          Global variables/const                           *
****************************************************************************/


/****************************************************************************
*                            Function prototypes                            *
****************************************************************************/

/**
 * This function is called by the application/OS to initialize the 6LoWPAN library
 */
ule6lo_status_t ule6loGI_init(const ule6lo_IPEI_t *IPEIAddr);

/**
 * Returns status of 6LoWPAN library, STATUS_SUCCESS for working, otherwise not working
 */
ule6lo_status_t	ule6loGI_getStatus(void);

/**
 * Function for getting IP address
 * @param ipType Type of IP address
 * @param ipAddr Pointer to IP address;
 * @param mode The mode of IP addresses valid. Use IP_ADDRESS_ANY if mode is not relevant
 * @return STATUS_NO_DATA if the requested IP address was not located, STATUS_SUCCESSS if it was found.
 */
ule6lo_status_t	ule6loGI_getIp6addr(ule6lo_ipType_t ipType,
                                    ule6lo_ip6addr_t*	ipAddr,
                                    ule6lo_ipMode_t mode);

/**
 * Add a context address used for compression of the prefix in ip address 
 * @param prefix[] Array of bytes for the prefix
 * @param prefixlength Length of prefix in bytes
 */
ule6lo_status_t ule6loGI_addContext(uint8_t prefix[] , uint8_t prefixlength);
/**
 * Add a multicast address to the list.
 */
ule6lo_status_t ule6loGI_addMulticastAddr(ule6lo_ip6addr_t* ipaddress);
/**
 * Remove a multicast address from the list.
 */
ule6lo_status_t ule6loGI_removeMulticastAddr(ule6lo_ip6addr_t* ipaddress);
/**
 * set the mac address for the LBR
 */
void ule6loGI_setMacAddress(ule6lo_macAddr_t* addr);
#endif /* ULE6LOGI_6LBR_H_ */

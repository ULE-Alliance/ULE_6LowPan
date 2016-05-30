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


#ifndef ULE6LO_IPEI_DEFINES_H_
#define ULE6LO_IPEI_DEFINES_H_

#define UIP_IPEI  		 		((ule6lo_IPEI_t*)&uip_buf[0])
#define UIP_IPEI_RECEIVED		UIP_IPEI
#define UIP_IPEI_TRANSMITTED	UIP_IPEI
#define UIP_WRITE_IPEI(ipei) 	memcpy(uip_buf,ipei,sizeof(ule6lo_IPEI_t))
#define UIP_READ_IPEI(ipei)		memcpy(ipei,UIP_IPEI,sizeof(ule6lo_IPEI_t))
#define UIP_WRITE_RECEIVED_IPEI_UNKNOWN() memset(UIP_IPEI_RECEIVED,0x00,sizeof(ule6lo_IPEI_t))

#define UIP_TRANSMITTED_FLAGS				((uint8_t*)&uip_buf[11])

//#define UIP_WRITE_IPEI(ipei) 				memcpy(UIP_IPEI,ipei,sizeof(ule6lo_IPEI_t))
#define UIP_WRITE_RECEIVED_IPEI(ipei) 		memcpy(UIP_IPEI_RECEIVED,ipei,sizeof(ule6lo_IPEI_t))
#define UIP_WRITE_TRANSMITTED_IPEI(ipei)	memcpy(UIP_IPEI_TRANSMITTED,ipei,sizeof(ule6lo_IPEI_t))
#define UIP_WRITE_TRANSMITTED_IPEI_MCAST()	memset(UIP_IPEI_TRANSMITTED,0xFF,sizeof(ule6lo_IPEI_t))
#define UIP_WRITE_TRANSMITTED_IPEI_UNKNOWN() memset(UIP_IPEI_TRANSMITTED,0x00,sizeof(ule6lo_IPEI_t))
//#define UIP_READ_IPEI(ipei)					memcpy(ipei,UIP_IPEI,sizeof(ule6lo_IPEI_t))
#define UIP_READ_RECEIVED_IPEI(ipei)		memcpy(ipei,UIP_IPEI_RECEIVED,sizeof(ule6lo_IPEI_t))
#define UIP_READ_TRANSMITTED_IPEI(ipei)		memcpy(ipei,UIP_IPEI_TRANSMITTED,sizeof(ule6lo_IPEI_t))
/*
#define UIP_LN_INTERFACE_MASK				0x01
#define UIP_ETH_INTERFACE_MASK				0x02
#define UIP_INTERFACE_MULTICAST_MASK		0x04

#define UIP_INTERFACE_TYPE_UNKNOWN			0
#define UIP_INTERFACE_TYPE_DECT				UIP_LN_INTERFACE_MASK
#define UIP_INTERFACE_TYPE_ETH				UIP_ETH_INTERFACE_MASK

#define UIP_RECEIVED_INTERFACE_TYPE			((*UIP_RECEIVED_FLAGS) & (UIP_LN_INTERFACE_MASK | UIP_ETH_INTERFACE_MASK))
#define UIP_TRANSMITTED_INTERFACE_TYPE		((*UIP_TRANSMITTED_FLAGS) & (UIP_LN_INTERFACE_MASK | UIP_ETH_INTERFACE_MASK))

#define UIP_TRANSMITTED_FLAGS_RESET()		(*UIP_TRANSMITTED_FLAGS = UIP_INTERFACE_TYPE_DECT | UIP_INTERFACE_TYPE_ETH)

#define DECT_VLIFE_TIME	900
#define DECT_PLIFE_TIME 900
*/
#endif /* ULE6LO_IPEI_DEFINES_H_ */

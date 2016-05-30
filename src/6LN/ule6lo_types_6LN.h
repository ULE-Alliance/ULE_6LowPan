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

#ifndef ULE6LO_TYPES_6LN_H_
#define ULE6LO_TYPES_6LN_H_

/****************************************************************************
*                               Include Files                               *
****************************************************************************/
#include <stdint.h>


/****************************************************************************
*                             Macro definitions                             *
****************************************************************************/

/****************************************************************************
*                   Enumerations/Type definitions/Structs                   *
****************************************************************************/
/**
 * General status type
 */
typedef enum ule6lo_status
{
	  STATUS_SUCCESS		= 0x00,		//The request completed successfully.
	  STATUS_NOT_CONNECTED	= 0x01,		//Connected
	  STATUS_ERROR			= 0x02,		//Error
	  STATUS_NO_DEVICE    	= 0x03,		//No such device
	  STATUS_NO_DATA      	= 0x04,		//No data is available.
	  STATUS_NOT_READY    	= 0x05,		//The device is not ready. 
	  STATUS_MAX			= 0xFF
} ule6lo_status_t;

/**
 * IP address type. More might be added
 */
typedef enum ule6lo_ipType
{
	  IP_ADDRESS_LINK_LOCAL	= 0x00,		//Link local IP address
	  IP_ADDRESS_GLOBAL	= 0x01,			//Global IP address
	  IP_ADDRESS_MAX	= 0xFF
} ule6lo_ipType_t;

/**
 * IP address mode. Used when obtaining IP addresses.
 */
typedef enum ule6lo_ipMode
{
    IP_ADDRESS_ANY = -1,       // Address of any type is accepted.
    IP_ADDRESS_TENTATIVE = 0, // Not yet active, pending duplicate address detection
    IP_ADDRESS_PREFERRED = 1, // IP address ready and in use.
    IP_ADDRESS_DEPRECATED =2  // IP address no longer used.
} ule6lo_ipMode_t;

/**
 * MAC address type
 */
typedef union
{
	uint8_t	u8[6];
} ule6lo_macAddr_t;

/**
 * IPv6 address type
 */
typedef union ule6lo_ip6addr
{
	uint8_t	 u8[16];
	uint16_t u16[8];
} ule6lo_ip6addr_t;

/**
 * IPEI address type
 */
typedef union ule6lo_IPEI
{
	uint8_t	id[5];
} ule6lo_IPEI_t;

/**
 * An entry in the neighbor cache
 */
typedef struct ule6lo_ds6_nbr
{
	ule6lo_ip6addr_t paddr;
	ule6lo_IPEI_t	lladdr;
} ule6lo_nbr_t;

/**
 * Hook function pointer type
 */
typedef void (*ule6lo_hook_t)(const uint8_t *data, uint16_t dataLength);


/****************************************************************************
*                          Global variables/const                           *
****************************************************************************/

/****************************************************************************
*                            Function prototypes                            *
****************************************************************************/


#endif /* ULE6LO_TYPES_6LN_H_ */

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



#ifndef ULE6LONI_6LBR_H_
#define ULE6LONI_6LBR_H_
/****************************************************************************
*                               Include Files                               *
****************************************************************************/
#include "ule6lo_types_6LBR.h"

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
 * This function is called by the network interface to deliver an incoming IPv6 packet to the 6LoWPAN library.
 * @param data Pointer to IPv6 packet
 * @param dataLength Length of IPv6 packet
 * @return status
 */
ule6lo_status_t ule6loNI_receive(const uint8_t*	data, uint16_t dataLength);


/*3.1.2.2	ule6loNI_send
Description:	This function is called by 6LoWPAN library to deliver an outgoing IPv6 packet to the network.
Returns:	ule6lo_status_t
Parameters:

Type 	Name	Description
const uint8_t *	data	Pointer to IPv6 packet
uint16_t	dataLength	Length of IPv6 packet
*/

/**
 * This function is called by the network interface to send a ping to the network..
  */
void ule6loNI_echoRequest(void);


#endif /* ULE6LONI_6LBR_H_ */

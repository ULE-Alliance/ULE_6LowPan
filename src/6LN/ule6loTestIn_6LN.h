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

#ifndef ULE6LOTESTIN_6LN_H_
#define ULE6LOTESTIN_6LN_H_

/****************************************************************************
*                               Include Files                               *
****************************************************************************/
#include "ule6lo_types_6LN.h"


/****************************************************************************
*                             Macro definitions                             *
****************************************************************************/

/****************************************************************************
*                   Enumerations/Type definitions/Structs                   *
****************************************************************************/



/****************************************************************************
*                          Global variables/const                           *
****************************************************************************/
extern ule6lo_hook_t ule6lo_txHook;
extern ule6lo_hook_t ule6lo_rxHook;

/****************************************************************************
*                            Function prototypes                            *
****************************************************************************/

/**
 * Initialize test interface (allocates buffers, resets packet counts and status ) and returns status
 */
ule6lo_status_t ule6loTestIn_init(void);

/**
 * Terminates test interface, including cleanup of buffer handling and deregister call backs, returns status
 */
ule6lo_status_t ule6loTestIn_deinit(void);

/**
 * Performs soft reset which emulates a hardware reset, everything is cleared including IPs and NB lists. Returns status
 */
ule6lo_status_t ule6loTestIn_reset(void);

/**
 * Returns the length of Neighbor list.
 */
uint16_t ule6loTestIn_getNbListSize(void);

/**
 * Copies specified index of the Neighbor list to specified destination
 * @param index Index in the neighbor list to be copied
 * @param nbListItem Item in neighbor list
 */
ule6lo_status_t ule6loTestIn_getNbList(uint16_t	index, ule6lo_nbr_t* nBListItem);

/**
 * Returns amount of sent packets
 */
uint32_t ule6loTestIn_getnofSentPacket(void);

/**
 * Returns amount of received packets
 */
uint32_t ule6loTestIn_getnofReceivedPacket(void);

/**
 * Register a hook function to be called every time an packet is received, with the raw packet as argument
 * @param rxHook Pointer callback function. This function is called every time a packet is received, with a pointer to the packet and length of the packet
 */
void ule6loTestIn_regRxHook(ule6lo_hook_t rxHook);

/**
 * Register a hook function to be called every time an packet is transmitted, with the raw packet as argument
 * @param txHook Pointer callback function. This function is called every time a packet is received, with a pointer to the packet and length of the packet
 */
void ule6loTestIn_regTxHook(ule6lo_hook_t txHook);


#endif /* ULE6LOTESTIN_6LN_H_ */

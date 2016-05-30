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


#ifndef ULE6LOLLI_6LN_H_
#define ULE6LOLLI_6LN_H_
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
extern uint32_t ule6lo_rx_package_counter;
extern uint32_t ule6lo_tx_package_counter;


/****************************************************************************
*                            Function prototypes                            *
****************************************************************************/

/**
 * Called after location registration to inform the 6LoWPAN library that the 6LN is now connected to 6LBR
 */
ule6lo_status_t ule6loLLI_init(const ule6lo_IPEI_t *IPEIAddr);

/**
 * Called from LL when LL receives a data indication and forwards data to the 6LoWPAN library.
 * @param data Pointer to ULE packet
 * @param Length of ULE packet
 */
ule6lo_status_t ule6loLLI_receive(const uint8_t* data, uint16_t dataLength, const ule6lo_IPEI_t *ULEAddr);

/**
 * Called from LL to indicate status of connection, if ready to receive/transmit data, or disconnected/offline
 */
void ule6loLLI_connected(ule6lo_status_t status);

/**
 * Called from LL when LL receives a page indication, then the 6LN must initiate a connection using send()
 */
void ule6loLLI_check(void);

/**
 * Called from LL after transmission to indicate status.
 * @param status Status of previous transmission
 */
void ule6loLLI_delivered(ule6lo_status_t status);


#endif /* ULE6LOLLI_6LN_H_ */

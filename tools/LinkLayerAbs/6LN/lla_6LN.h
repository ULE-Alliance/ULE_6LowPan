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

#ifndef LLA_6LN_H
#define LLA_6LN_H

/****************************************************************************
*                               Include Files                               *
****************************************************************************/
//#include <sys/socket.h>
//#include <netinet/in.h>
#include <stdint.h>
#include "ule6lo_types_6LN.h"
//#include <arpa/inet.h>

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
void lla_send(const void *data, uint16_t length);
void lla_receive(void *data, uint16_t* length);
void lla_init(const char* destinationIp,int port, ule6lo_IPEI_t* ipei_node);
void ule6loLLI_send(const uint8_t *	data, uint16_t	dataLength);
void llaProcess(void);

#endif

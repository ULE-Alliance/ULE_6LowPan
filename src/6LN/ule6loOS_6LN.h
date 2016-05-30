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


#ifndef ULE6LOOS_6LN_H_
#define ULE6LOOS_6LN_H_
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

/****************************************************************************
*                            Function prototypes                            *
****************************************************************************/


/**
 * This function should be called repeatedly from the main() program / OS to actually run the 6LoWPAN library.
 */
void ule6loOS_processRun(void);

/**
 * Returns the 6LNs MAC address. This function is hardware/ OS depended and needs to be implemented correspondingly
 */
ule6lo_macAddr_t* ule6loOS_getMACAddr(void);

/**
 * Function called by the 6LoWPAN library to get the system timer tick. The function expects a tick increment corresponding to10 ms. The function is hardware/ OS depended and needs to be implemented correspondingly.
 */
uint32_t ule6loOS_getTimerTick(void);


#endif /* ULE6LOOS_6LN_H_ */

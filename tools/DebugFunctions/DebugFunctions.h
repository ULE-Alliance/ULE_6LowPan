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


#ifndef DEBUGFUNCTIONS_H_
#define DEBUGFUNCTIONS_H_
#ifdef LBR_ROLE
#include "ule6lo_types_6LBR.h"
#else
#include "ule6lo_types_6LN.h"
#endif

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

#define PRINT_YELLOW(format, ...) printf(ANSI_COLOR_YELLOW format ANSI_COLOR_RESET ,__VA_ARGS__)
#define PRINT_RED(format, ...) printf(ANSI_COLOR_RED format ANSI_COLOR_RESET ,__VA_ARGS__)
#define PRINT_CYAN(format, ...) printf(ANSI_COLOR_CYAN format ANSI_COLOR_RESET ,__VA_ARGS__)

#if FILTERED_BORDER_ROUTER
void printPrefixListRouter();
void printPrefixListNonRouter();
#endif
void printIPAddresses(void);
void printLinkLocalAddress(void);
void printGloballAddress(void);
void printContikiConfigurationInfo(void);
void printIPV6Neighbourlist(void);
int readPCMacAddress(void);
void setFixedIP(const char* prefix);
void test_rx(const uint8_t *data, uint16_t dataLength);
void test_tx(const uint8_t *data, uint16_t dataLength);
void test_br_rx(const uint8_t *data, uint16_t dataLength);
void test_br_tx(const uint8_t *data, uint16_t dataLength);

#endif /* DEBUGFUNCTIONS_H_ */

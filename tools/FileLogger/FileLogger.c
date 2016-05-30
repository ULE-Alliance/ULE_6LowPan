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


/****************************************************************************
 *                               Include files
 ****************************************************************************/
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <fcntl.h>
#include <stdarg.h>
#include <time.h>


/****************************************************************************
 *                              Macro definitions
 ****************************************************************************/

/****************************************************************************
 *                     Enumerations/Type definitions/Structs
 ****************************************************************************/


/****************************************************************************
 *                            Global variables/const
 ****************************************************************************/


/****************************************************************************
 *                            Local variables/const
 ****************************************************************************/
static FILE* file;
static unsigned char buffer[1024];
static struct tm theTime;
/****************************************************************************
 *                          Local Function prototypes
 ****************************************************************************/
static void update_time();
static void append_time();
/****************************************************************************
 *                                Implementation
 ****************************************************************************/
void file_logger_init(const char* fileName) {
	file = fopen (fileName,"w");
	fflush(stdout);
}

void file_logger_append_raw(const void* data, uint16_t dataLength) {
	append_time();
	fwrite(data,1,dataLength,file);
	fwrite("\n",1,1,file);
	fflush(file);
}

void file_logger_append_format(const void* data, uint16_t dataLength, const char* format, ...) {
	va_list argptr;
	va_start(argptr,format);
	append_time();
	vfprintf(file,format,argptr);
	fwrite(data,1,dataLength,file);
	fwrite("\n",1,1,file);
	fflush(file);
	va_end(argptr);
}

static void update_time() {
	time_t curTime;
	time(&curTime);
	gmtime_r(&curTime,&theTime);
}

static void append_time() {
	update_time();
	fprintf(file,"%02d:%02d:%02d ", theTime.tm_hour, theTime.tm_min, theTime.tm_sec);
	fflush(file);
}

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
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include "ule6loLLI_6LBR.h"
#include "ule6lo_types_6LBR.h"
#include "ule6loTestIn_6LBR.h"

#include "lla_6LBR.h"
#include "../../FileLogger/FileLogger.h"
#include "../../DebugFunctions/DebugFunctions.h"
#include "ule6lo_ipei_defines.h"
#include <ipv6/uip-ds6-nbr.h>
/****************************************************************************
 *                              Macro definitions
 ****************************************************************************/

/****************************************************************************
 *                     Enumerations/Type definitions/Structs
 ****************************************************************************/
#define BUFLEN UIP_CONF_BUFFER_SIZE
#define LBR_PORT 12345
#define MAX_NEIGHBOURS 8//if more than 19, extra handling is needed createNeighbourlist
#define DECT_REG	"DECT_REG"

typedef struct lla_6LBR_nbr
{
  uint16_t port;
  in_addr_t ip;
  ule6lo_IPEI_t	lladdr;
} lla_6LBR_nbr_t;

#define NODES_IPEI_LIST_SIZE 32
static ule6lo_IPEI_t nodes_ipei_list[NODES_IPEI_LIST_SIZE];
static int nodes_ipei_list_count=0;
/****************************************************************************
 *                            Global variables/const
 ****************************************************************************/


/****************************************************************************
 *                            Local variables/const
 ****************************************************************************/
static int udpSocket;
static lla_6LBR_nbr_t neighbourlist[MAX_NEIGHBOURS];

/****************************************************************************
 *                          Local Function prototypes
 ****************************************************************************/
static void die(char *s);
static void createNeighbourlist();
static void lla_send(const void *data, uint16_t length, in_addr_t ip, int port);
/****************************************************************************
 *                                Implementation
 ****************************************************************************/
//TO DO: Implement logging
static void die(char *s)
{
	perror(s);
	exit(1);
}

static void createNeighbourlist() {
	//Create fixed neighbour list
	memset((char*)neighbourlist,0,sizeof(neighbourlist));
	for ( int i = 0; i < MAX_NEIGHBOURS; i++){
	neighbourlist[i].port=11111+(i+1);
	neighbourlist[i].ip = inet_addr("127.0.0.1");
	int x = i;
	if( x > 8 ){//hack for hex handling
	  x += 6;
	}
	neighbourlist[i].lladdr.id[4]=x+1;
	}
}

void lla_init()
{
	int nBytes;
	socklen_t addr_size;
	struct sockaddr_in serverAddr;
	int i;

	createNeighbourlist();

	//Create UDP socket
	udpSocket = socket(PF_INET, SOCK_DGRAM, 0);

	// Put the socket in non-blocking mode:
	if(fcntl(udpSocket, F_SETFL, fcntl(udpSocket, F_GETFL) | O_NONBLOCK) < 0) {
		// handle error
		die("Unable to activate non blocking socket");
	}

	//Configure settings in address struct
	memset((char *) &serverAddr, 0, sizeof(serverAddr));
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(LBR_PORT);
	serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);

	/*Bind socket with address struct*/
	if(bind(udpSocket, (struct sockaddr *) &serverAddr, sizeof(serverAddr))<0)
	{
		printf("Error is %i", errno);
	}

	printf("Starting UDP server\n");
}

static void lla_send(const void *data, uint16_t length, in_addr_t ip, int port)
{
	/*Send message to server*/
	struct sockaddr_in serverAddr;
	socklen_t addr_size;
	addr_size = sizeof serverAddr;

	/*Configure settings in address struct*/
	memset((char *) &serverAddr, 0, sizeof(serverAddr));
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(port);
	serverAddr.sin_addr.s_addr = ip;//inet_addr(/*"127.0.0.1"*/ip);
	//  serverAddr.sin_addr.s_addr = inet_addr("10.10.100.146");

	uint8_t* tmp = (uint8_t*)data;
	//Send message to server
	/*PRINT_RED("lla_send %i bytes: ",length);
	for(int i=0;i<length;i++) {
		PRINT_RED("%02X",tmp[i]);
	}
	PRINT_RED("%s","\n");*/

	sendto(udpSocket,data,length,0,(struct sockaddr *)&serverAddr,addr_size);
}

void llaProcess(void) {
	uint8_t buffer[BUFLEN];
	ule6lo_IPEI_t ipei;
	uint16_t nBytes; // Holds the number of bytes received by link layer
	nBytes = 0;
	lla_receive(buffer, &nBytes);
	if (nBytes > 0) {
		// Message received, handle the message
		/*PRINT_YELLOW("Received %i bytes: ",nBytes);
		for(int i=0;i<nBytes;i++) {
			PRINT_YELLOW("%02X",buffer[i]);
		}
		PRINT_YELLOW("%s","\n");*/
		//handleMessage(buffer, nBytes);

		//Extract the IPEI number from the data
		memcpy(&ipei,buffer,sizeof(ule6lo_IPEI_t));


		// INSTEAD OF DECT REGISTRATION.
		// In a real world application, the method ule6loLLI_init() would be called when a node connects
		if(memcmp(buffer+sizeof(ule6lo_IPEI_t),DECT_REG,strlen(DECT_REG)) ==0 ) {
			//DECT registration message, process here and dont pass on
			uint8_t found=0;
			int i=0;
			for(;i<nodes_ipei_list_count;i++) {
				if(memcmp(&nodes_ipei_list[i],&ipei,sizeof(ule6lo_IPEI_t) )==0) {
					found =1;
				}
			}
			if(!found && i<NODES_IPEI_LIST_SIZE) {
				memcpy(&nodes_ipei_list[i],&ipei,sizeof(ule6lo_IPEI_t));
				PRINT_CYAN("%s,","Registering IPEI\n");
				nodes_ipei_list_count++;
				ule6loLLI_init(&ipei);
			}
		}
		else {
			//Data received, pass to stack, but strip the IPEI number before passing the data forward
			ule6loLLI_receive(buffer+sizeof(ule6lo_IPEI_t),nBytes-sizeof(ule6lo_IPEI_t),&ipei);
		}
	}
}

void lla_receive(void *data, uint16_t* length)
{
	struct sockaddr_storage serverStorage;
	socklen_t addr_size;

	addr_size = sizeof serverStorage;

	*length=0;

	ssize_t res = recvfrom(udpSocket,data,BUFLEN,0,(struct sockaddr *)&serverStorage, &addr_size);
	if(res == -1)
	{
		if(errno == EAGAIN || errno == EWOULDBLOCK)
		{
			// No data received yet, returning to avoid blocking
		}
		else
		{
			die("recvfrom()");
			printf("Error did not receive anything\n");
		}
	}
	else
	{
		*length = res;
		file_logger_append_format(data,res,"lla_receive:");
	}
}

static void lla_send_ipei(const uint8_t* data,uint16_t dataLength, const ule6lo_IPEI_t* ULEAddr) {
	char found;
	int i;
	//Locate the neighbour
	found =0;
	i=0;
	//printIPEI(ULEAddr);
	while(i<MAX_NEIGHBOURS) {
		if(memcmp(ULEAddr,&neighbourlist[i].lladdr,sizeof(ule6lo_IPEI_t)) == 0) {
			found =1;
			//printf("Found target index %i\n",i);
			break;
		}
		i++;
	}
	//HACK to first entry in list
	//found = 1;
	//i=0;

	if(found)
	{
		//printf(" was located at index %i on port %i\n",i, neighbourlist[i].port);
		file_logger_append_format(data,dataLength,"ule6loLLI_send:to port %i:",neighbourlist[i].port);
		lla_send(data,dataLength,neighbourlist[i].ip,neighbourlist[i].port);
	}
	else {
		//printf(" was not a known destination, unable to transmit\n");
	}
}

void ule6loLLI_send(const uint8_t* data,uint16_t dataLength, const ule6lo_IPEI_t* ULEAddr) 
{

	//printf("About to send message to ");
	//printIPEI(ULEAddr);
	//printf("\n");


	/*if(ULEAddr->id[0]==0xFF && ULEAddr->id[1]==0xFF && ULEAddr->id[2]==0xFF && ULEAddr->id[3]==0xFF && ULEAddr->id[4]==0xFF) {
		// Multicast address, send to all neighbours
		printf("Sending to all known neighbours.\n");
		//TODO build proper list based in link layer init calls. Do not use neighbour list for this
		// Do this by creating fake DECTREG message, and send that as first message from LN.
		// Use that to add to list of IPEI's
		
		for(int i=0;i<nodes_ipei_list_count;i++) {
			lla_send_ipei(data,dataLength,&nodes_ipei_list[i]);
		}
	}
	else */{
		printf("Sending to dedicated neoighbour.\n");
		// Normal address, send to neighbour if known
		lla_send_ipei(data,dataLength,ULEAddr);
	}

}


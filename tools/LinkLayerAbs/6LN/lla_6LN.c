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
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <netinet/in.h>
#include <string.h>
#include <syslog.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/if.h>
#include <arpa/inet.h>

#include "lla_6LN.h"
#include "ule6loLLI_6LN.h"
#include "ule6lo_types_6LN.h"
#include "ule6loTestIn_6LN.h"

#include "FileLogger.h"
#include "contiki.h"
#include "contiki-version.h"


#include "contiki.h"
#include "net/netstack.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/rime/rime.h"

#include "../6LBR/not_yet_placed.h"
#include "ule6loLLI_6LN.h"
#include "../../DebugFunctions/DebugFunctions.h"



/****************************************************************************
*                              Macro definitions
****************************************************************************/

/****************************************************************************
*                     Enumerations/Type definitions/Structs
****************************************************************************/
#define BUFLEN UIP_CONF_BUFFER_SIZE
#define LBR_PORT 12345
#define LBR_IP "127.0.0.1"
#define DECT_REG	"DECT_REG"

typedef struct lla_socket_data_t
{
  int clientSocket;
  int portNum;
  struct sockaddr_in serverAddr;
  socklen_t addr_size;
}lla_socket_data_t;

/****************************************************************************
*                            Global variables/const
****************************************************************************/


/****************************************************************************
*                            Local variables/const
****************************************************************************/
static lla_socket_data_t socket_data;
static ule6lo_IPEI_t ipei_lbr;
static ule6lo_IPEI_t ipei_ln;
static uint8_t temp_buffer[2048];	// Used to prepend the IPEI to data being send.

/****************************************************************************
*                          Local Function prototypes
****************************************************************************/
void die(char *s);
/****************************************************************************
*                                Implementation
****************************************************************************/
//TO DO: Implement logging

void die(char *s)
{
  perror(s);
  exit(1);
}

void init_log()
{
    setlogmask(LOG_UPTO(LOG_NOTICE));
    openlog("lla_6LN",LOG_CONS | LOG_PID | LOG_NDELAY, LOG_LOCAL1);
}


void lla_init(const char* destinationIp,int port, ule6lo_IPEI_t* ipei_node)
{    
  socklen_t addr_size;
  struct sockaddr_in serverAddr;
  memcpy(&ipei_ln,ipei_node,sizeof(ule6lo_IPEI_t));


  init_log();
  syslog(LOG_NOTICE, "Session started!!");
  
  closelog();
  /*Create UDP socket*/
  socket_data.clientSocket= socket(PF_INET, SOCK_DGRAM, 0);

  // Put the socket in non-blocking mode: 
  if(fcntl(socket_data.clientSocket, F_SETFL, fcntl(socket_data.clientSocket, F_GETFL) | O_NONBLOCK) < 0) {
    // handle error
    die("Unable to activate non blocking socket");
  }

  //Configure settings in address struct
  socket_data.serverAddr.sin_family = AF_INET;
  //TO DO: READ PORT AND IP FROM CONFIGURATION FILE!!!!!!!!!
  socket_data.serverAddr.sin_port = htons(LBR_PORT);
//  socket_data.serverAddr.sin_addr.s_addr = inet_addr("10.10.102.46");  
  socket_data.serverAddr.sin_addr.s_addr = inet_addr(destinationIp);  

  memset(socket_data.serverAddr.sin_zero, '\0', sizeof socket_data.serverAddr.sin_zero);  

  //Initialize size variable to be used later on
  socket_data.addr_size = sizeof (socket_data.serverAddr);

	
  /*Configure settings in address struct*/
  memset((char *) &serverAddr, 0, sizeof(serverAddr));
  serverAddr.sin_family = AF_INET;
  serverAddr.sin_port = htons(port);
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);

  //Bind socket with address struct
  if(bind(socket_data.clientSocket, (struct sockaddr *) &serverAddr, sizeof(serverAddr))<0) 
  {
    printf("Error is %i", errno);
  }

  printf("Starting UDP server\n");
  
  //Register this node with the LBR. Simulates DECT registration
  lla_send(DECT_REG,strlen(DECT_REG));
	
  //Inform library of connection success.
  ule6loLLI_init(ipei_node);
}



void lla_send(const void *data, uint16_t length)
{
	memcpy(temp_buffer,&ipei_ln,sizeof(ule6lo_IPEI_t));
	memcpy(temp_buffer+sizeof(ule6lo_IPEI_t),data,length);
	uint8_t* tmp = (uint8_t*)temp_buffer;

  //Send message to server
  /*PRINT_RED("lla_send %i bytes: ",length+sizeof(ule6lo_IPEI_t));
  for(int i=0;i<length+sizeof(ule6lo_IPEI_t);i++) {
	  PRINT_RED("%02X",tmp[i]);
  }
  PRINT_RED("%s","\n");*/
  sendto(socket_data.clientSocket,temp_buffer,length+sizeof(ule6lo_IPEI_t),0,(struct sockaddr *)&socket_data.serverAddr,socket_data.addr_size);
}

void ule6loLLI_send(const uint8_t *	data, uint16_t	dataLength) {
  if(dataLength>0) {
    lla_send(data,dataLength);
  }
}

void lla_receive(void *data, uint16_t* length)
{
	*length = 0;
  ssize_t res = recvfrom(socket_data.clientSocket,data,BUFLEN,0,NULL, NULL); 	
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
    /*PRINT_YELLOW("lla receive %i bytes: ",*length);
    uint8_t* tmp = (uint8_t*)data;
    for(res =0;res<*length;res++) {
    	PRINT_YELLOW("%02X",tmp[res]);
     }
    PRINT_YELLOW("%s","\n");*/
  }
}
void llaProcess(void) {
	char lla_buffer[BUFLEN];	// Used as receive buffer from data comming from link layer
	uint16_t nBytes;			// Holds the number of bytes received by link layer
	ule6lo_IPEI_t ipei;
	lla_receive(lla_buffer, &nBytes);
	if(nBytes>0) {
		// Data received in link abstraction layer, pass to library
	    //memcpy(&ipei,buffer,sizeof(ule6lo_IPEI_t), );
	    ipei.id[0] = 0x01;
	    ipei.id[1] = 0x00;
	    ipei.id[2] = 0x00;
	    ipei.id[3] = 0x00;
	    ipei.id[4] = 0x09;

	  ule6loLLI_check();//Sends a empty string at the dect layer. It must send data before it can receive any data.
		ule6loLLI_receive(lla_buffer,nBytes,&ipei);
	}
	
}



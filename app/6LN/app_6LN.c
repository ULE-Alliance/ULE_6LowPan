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
#include <fcntl.h>

#include "lla_6LN.h"
#include "app_6LN.h"
#include "contiki.h"
#include "contiki-version.h"
#include "net/netstack.h"
#include "net/ipv6/uip-ds6.h"
#include "net/queuebuf.h"
#include "ule6lo_radio_6LN.h"
#include "lla_6LN.h"
#include "ule6loLLI_6LN.h"
#include "ule6loTestIn_6LN.h"
#include "ule6loGI_6LN.h"
#include "ule6loOS_6LN.h"
#include "../../tools/DebugFunctions/DebugFunctions.h"
#include <ip/udp-socket.h>
#include <ip/uiplib.h>
#include <net/ip/uip-debug.h>
#include "net/ip/resolv.h"
#include "net/ipv6/uip-mld.h"

/****************************************************************************
*                              Macro definitions
****************************************************************************/

/****************************************************************************
*                     Enumerations/Type definitions/Structs
****************************************************************************/
#define BUFLEN UIP_CONF_BUFFER_SIZE

/****************************************************************************
*                            Global variables/const
****************************************************************************/


/****************************************************************************
*                            Local variables/const
****************************************************************************/

/****************************************************************************
*                          Local Function prototypes
****************************************************************************/

/****************************************************************************
*                                Implementation
****************************************************************************/
//TO DO: Implement logging

//UDP comm test
static struct udp_socket client_sock;


void send_udp(char* data) {
	uip_ipaddr_t ipaddr;

	char* ipEnd;
	char* portEnd;
	uint16_t port;
	int i;
	char found;

	// Extract IP address
	ipEnd = strchr((const char*) data, ' ');
	ipEnd[0]=0;
	uiplib_ipaddrconv(data, &ipaddr);
	portEnd = strchr((const char*) ipEnd + 1, ' ');
	portEnd[0]=0;
	port = atoi(ipEnd + 1);
	//printf("Port target : %i\n",port);



	//uip_ip6addr(&ipaddr,0xFE80,0000,0000,0000,0x3E97,0x0EFF,0xFE00,0x0001 );
	int res = udp_socket_sendto(&client_sock,
			portEnd+1, strlen(portEnd+1),
			&ipaddr, port);
}

static void udp_data_received(struct udp_socket *c,
                                             void *ptr,
                                             const uip_ipaddr_t *source_addr,
                                             uint16_t source_port,
                                             const uip_ipaddr_t *dest_addr,
                                             uint16_t dest_port,
                                             const uint8_t *data,
                                             uint16_t datalen) {
	((uint8_t*)(data))[datalen]=0;
	printf(ANSI_COLOR_CYAN "Received UDP data, %i bytes: %s\n"ANSI_COLOR_RESET,datalen,(char*)data);
}


void init_udp() {
	udp_socket_register(&client_sock,0,&udp_data_received);
	udp_socket_bind(&client_sock, 3001);
}









static struct stimer ipPrintTimer;
void debugPrint() {
	//Print all our IP addresses when timer expires.
	if (stimer_expired(&ipPrintTimer)) {
		stimer_reset(&ipPrintTimer);
		printIPAddresses();
		printLinkLocalAddress();
		printGloballAddress();
		printIPV6Neighbourlist();

		//PRINT_CYAN("Rx counter = %i, TX counter = %i\n",ule6loTestIn_getnofReceivedPacket(),ule6loTestIn_getnofSentPacket());
	}
}

static char app_buffer[BUFLEN];	// Used as receive buffer from data comming from stdin
static ssize_t app_res;		// Holds return value when reading from stdin
static int app_bytes_recv;		// Holds the current number of bytes received from stdin

void readCommandLine() {
	// Check the stdin for data
	app_res = read(0, app_buffer + app_bytes_recv, BUFLEN);
	if (app_res > 0) {
		// Data received from stdin, add to buffer
		app_bytes_recv += app_res;
		if (app_buffer[app_bytes_recv - 1] == '\n') {
			// Command ended with linefeed, terminate string and inform user
		  if(memcmp(app_buffer,"reset",5)==0) {
		    ule6loTestIn_reset();
		  }
		  if(memcmp(app_buffer,"rm",2)==0) {
		      ule6lo_ip6addr_t loc_fipaddr2;
		      memset(&loc_fipaddr2, 0x00, sizeof(loc_fipaddr2));
		      loc_fipaddr2.u8[0] = 0xff;
		      loc_fipaddr2.u8[1] = 0x02;
		      loc_fipaddr2.u8[10] = 0x00;
		      loc_fipaddr2.u8[11] = 0x00;
		      loc_fipaddr2.u8[12] = 0x00;
		      loc_fipaddr2.u8[13] = 0x01;
		      loc_fipaddr2.u8[15] = 0x05;
		      
		    ule6loGI_removeMulticastAddr(&loc_fipaddr2);
		  }
		  else if(memcmp(app_buffer,"query",5)==0)
		  {
		    uip_ip6addr_t loc_fipaddr2;
        memset(&loc_fipaddr2, 0x00, sizeof(loc_fipaddr2));
       /* loc_fipaddr2.u8[0] = 0xff;
        loc_fipaddr2.u8[1] = 0x02;
        loc_fipaddr2.u8[10] = 0x00;
        loc_fipaddr2.u8[11] = 0x01;
        loc_fipaddr2.u8[12] = 0xFF;
        loc_fipaddr2.u8[13] = 0x00;
        loc_fipaddr2.u8[15] = 0x02;*/
        //unspecified..
#ifdef UIP_CONF_MLD
		    uip_icmp6_mldv1_query(&loc_fipaddr2);
#endif
		  }
		  else if(memcmp(app_buffer,"resolv",6)==0) {
		    uip_ipaddr_t* resolvedto;
		    resolv_status_t res;

		    res = resolv_lookup("test.jjo",&resolvedto);
		    if(res == RESOLV_STATUS_CACHED) {
		      printf("Resolved to ");
		      uip_debug_ipaddr_print(resolvedto);
		      printf("\n");
		    }
		    else if ( res != RESOLV_STATUS_RESOLVING ){
		      printf("Unable to resolve, result = %i. Executing query for it\n",res);
		      resolv_query("test.jjo");
		    }
		    else{
		      printf("Unable to resolve, query in progress\n");
		    }
		  }
		  else {
		    app_buffer[app_bytes_recv] = 0;
		    printf("You typed: %s\n", app_buffer);
		    //Send message to server and reset message length
		    send_udp(app_buffer);
		    //ule6loLLI_send(app_buffer, app_bytes_recv);
		  }
		  app_bytes_recv = 0;
		}
	}
}

//this will be LinkLayerAbsInit when linked toghter with 6LowPan
int main(int argc, char **argv)
{
  int flags;			// Used to set non-blocking for stdin	

  int retval;
  ule6lo_status_t status;	// Used as result in many ule6lo lib calls
  ule6lo_IPEI_t ipei;
  ule6lo_macAddr_t mac;


  //Initialize test hooks
  ule6loTestIn_regRxHook(&test_rx);
  ule6loTestIn_regTxHook(&test_tx);

  // Setup stdin to operate non blocking
  flags = fcntl(0, F_GETFL, 0); 
  flags |= O_NONBLOCK; 
  fcntl(0, F_SETFL, flags);

  //Reset number of received bytes
  app_bytes_recv = 0;

  //Print instruction message
  printf("To send a message use this format and acknowledge with enter:\n<Destination ip> <Destination port> <message>\n");
  printf("Use port 3001 for now\n");

  // Debug print of contiki setup (defines etc)
  printContikiConfigurationInfo();



  printf("Args: <LBR IP> <LN port> <LN IPEI(mac addr style,example:00:00:00:00:01)> <Mac addr>\n");
  if(argc>=5)
  {
    status = ule6loGI_init();
    printf("ule6loGI_init() returned %i\n", status);

    // Initialize link layer
    sscanf(argv[3],"%2hhX:%2hhX:%2hhX:%2hhX:%2hhX",&(ipei.id[0]),&(ipei.id[1]),&(ipei.id[2]),&(ipei.id[3]),&(ipei.id[4]));
    sscanf(argv[4],"%2hhX:%2hhX:%2hhX:%2hhX:%2hhX:%2hhX",&(mac.u8[0]),&(mac.u8[1]),&(mac.u8[2]),&(mac.u8[3]),&(mac.u8[4]),&(mac.u8[5]));
    ule6loGI_setMacAddress(&mac);
    lla_init(argv[1],atoi(argv[2]),&ipei);

    printf("Using IP %s and port %i and IPEI %s and MAC addr %s\n",argv[1],atoi(argv[2]), argv[3], argv[4]);
  }
  else {
    printf("Incorrect number of arguments");
    return 1;
  }

  //Wait for connection
  while(ule6loGI_getStatus() != STATUS_SUCCESS) {
    sleep(1);
  }

  // Print our Link local address and our global address
  printLinkLocalAddress();
  printGloballAddress();

  //Create timer to print our IP addresses periodically
  stimer_set(&ipPrintTimer,10 );

  // Print all our IP addresses
  printIPAddresses();
  printIPV6Neighbourlist();


  init_udp();
  ule6lo_ip6addr_t loc_fipaddr2;
  memset(&loc_fipaddr2, 0x00, sizeof(loc_fipaddr2));
  loc_fipaddr2.u8[0] = 0xff;
  loc_fipaddr2.u8[1] = 0x02;
  loc_fipaddr2.u8[10] = 0x00;
  loc_fipaddr2.u8[11] = 0x00;
  loc_fipaddr2.u8[12] = 0x00;
  loc_fipaddr2.u8[13] = 0x01;
  loc_fipaddr2.u8[15] = 0x05;
  ule6loGI_addMulticastAddr(&loc_fipaddr2);
  // Run forever	
  while(1){
    // Check the stdin for data
	readCommandLine(app_res, app_buffer, app_bytes_recv);

    // Let link layer receive data is any is present
    llaProcess();

	debugPrint();

    // Allow the stack to run
    ule6loOS_processRun();
  }

  return 0;
}





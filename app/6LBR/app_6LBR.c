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
#include <arpa/inet.h>
#include <string.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/if.h>
#include <errno.h>

#include "lla_6LBR.h"
#include "app_6LBR.h"
#include "FileLogger.h"
#include "contiki.h"
#include "contiki-version.h"


#include "contiki.h"
#include "net/netstack.h"
#include "net/ip/uip.h"
#include "net/ip/resolv.h"
#include "net/ipv6/uip-ds6-nbr.h"
#include "net/ipv6/uip-ds6.h"
#include "net/rime/rime.h"

#include "not_yet_placed.h"
#include "ule6lo_radio_6LBR.h"
#include "lla_6LBR.h"
#include "../../tools/DebugFunctions/DebugFunctions.h"
#include "ule6loGI_6LBR.h"
#include "ule6loOS_6LBR.h"
#include "ule6loLLI_6LBR.h"
#include "ule6loTestIn_6LBR.h"
#include "ule6loNI_6LBR.h"
#include <ip/udp-socket.h>
#include <ip/uiplib.h>
#include <net/ip/uip-debug.h>

/****************************************************************************
 *                              Macro definitions
 ****************************************************************************/

/****************************************************************************
 *                     Enumerations/Type definitions/Structs
 ****************************************************************************/
#define BUFLEN UIP_CONF_BUFFER_SIZE
#define MAX_NEIGHBOURS 3
#define LBR_PORT 12345
#define IPV6_FIXED_PREFIX "fc00:5254:5800:0000::1"

#ifdef SELECT_CONF_MAX
#define SELECT_MAX SELECT_CONF_MAX
#else
#define SELECT_MAX 8
#endif

static const struct select_callback *select_callback[SELECT_MAX];
static int select_max = 0;

typedef struct lla_6LBR_nbr
{
	uint16_t port;
	in_addr_t ip;
	ule6lo_IPEI_t	lladdr;
} lla_6LBR_nbr_t;


/****************************************************************************
 *                            Global variables/const
 ****************************************************************************/

/****************************************************************************
 *                            Local variables/const
 ****************************************************************************/
static lla_6LBR_nbr_t neighbourlist[MAX_NEIGHBOURS];
/****************************************************************************
 *                          Local Function prototypes
 ****************************************************************************/

/****************************************************************************
 *                                Implementation
 ****************************************************************************/
void handle_callbacks() {
	fd_set fdr;
	fd_set fdw;
	int maxfd;
	int i;
	int retval;
	struct timeval tv;

	tv.tv_sec = 0;
	tv.tv_usec = 1;

	FD_ZERO(&fdr);
	FD_ZERO(&fdw);
	maxfd = 0;
	for(i = 0; i <= select_max; i++) {
		if(select_callback[i] != NULL && select_callback[i]->set_fd(&fdr, &fdw)) {
			maxfd = i;
		}
	}

	retval = select(maxfd + 1, &fdr, &fdw, NULL, &tv);
	if(retval < 0) {
		if(errno != EINTR) {
			perror("select");
		}
	} else if(retval > 0) {
		/* timeout => retval == 0 */
		for(i = 0; i <= maxfd; i++) {
			if(select_callback[i] != NULL) {
				select_callback[i]->handle_fd(&fdr, &fdw);
			}
		}
	}
}

int
select_set_callback(int fd, const struct select_callback *callback)
{
  int i;
  if(fd >= 0 && fd < SELECT_MAX) {
    /* Check that the callback functions are set */
    if(callback != NULL &&
       (callback->set_fd == NULL || callback->handle_fd == NULL)) {
      callback = NULL;
    }

    select_callback[fd] = callback;

    /* Update fd max */
    if(callback != NULL) {
      if(fd > select_max) {
        select_max = fd;
      }
    } else {
      select_max = 0;
      for(i = SELECT_MAX - 1; i > 0; i--) {
        if(select_callback[i] != NULL) {
          select_max = i;
          break;
        }
      }
    }
    return 1;
  }
  return 0;
}


//UDP comm test
static struct udp_socket server_sock;
void reply_udp(const uip_ipaddr_t *addr) {
	int res = udp_socket_sendto(&server_sock,
	                      "Hello world\n\0", 13,
	                      addr, 3001);
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
	reply_udp(source_addr);
}


void init_udp() {
	udp_socket_register(&server_sock,0,&udp_data_received);
	udp_socket_bind(&server_sock, 3001);
}




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
	int res = udp_socket_sendto(&server_sock,
			portEnd+1, strlen(portEnd+1),
			&ipaddr, port);
}


//TO DO: Implement logging
void handleMessage(const uint8_t* data, uint16_t dataLength) {
	char* ipEnd;
	char* portEnd;
	in_addr_t ip;
	uint16_t port;
	int i;
	char found;

	// Extract IP address
	ipEnd = strchr((const char*) data, ' ');
	//ipEnd[0]=NULL;
	ip = inet_addr(data);
	//printf("IP target : %s translates to %i\n",(char*)data,ip);
	portEnd = strchr((const char*) ipEnd + 1, ' ');
	//portEnd[0]=NULL;
	port = atoi(ipEnd + 1);
	//printf("Port target : %i\n",port);

	//Locate the neighbour
	found = 0;
	i = 0;
	while (i < MAX_NEIGHBOURS) {
		//printf("Neighboour IP : %i, port : %i\n",neighbourlist[i].ip,neighbourlist[i].port);
		if (neighbourlist[i].ip == ip && neighbourlist[i].port == port) {
			found = 1;
			break;
		}
		i++;
	}

	if (found) {
		// For a neighbour, forward the information
		//printf("Using Neighboour IP : %i, port : %i\n",neighbourlist[i].ip,neighbourlist[i].port);
		ule6loLLI_send(data, dataLength, &neighbourlist[i].lladdr);
	} else {
		// For me, print the message
		if (port == LBR_PORT) {
			printf("Message Received: ");
			for (i = 0; i < dataLength; i++) {
				printf("%c", ((char*) data)[i]);
			}
			printf("\n");
		}
	}
}

PROCESS(example_pollhandler, "Pollhandler example");
static void
exithandler(void)
{
	printf("Process exited\n");
}
static void
pollhandler(void)
{
	printf("Process polled\n");
}
PROCESS_THREAD(example_pollhandler, ev, data)
{

	PROCESS_POLLHANDLER(pollhandler());
	PROCESS_EXITHANDLER(exithandler());
	PROCESS_BEGIN();
	while(1) {
		printf("Hello from protothread\n");
		PROCESS_WAIT_EVENT();
		//PROCESS_YIELD();
	}

	PROCESS_END();
	return 0;
}


static ssize_t app_res;
static char app_buffer[BUFLEN];
static int app_bytes_recv;

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
		  else if(memcmp(app_buffer,"ping",4)==0){
		    ule6loNI_echoRequest();
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
		    fflush (stdout);
		    // Handle the message
		    //handleMessage(app_buffer, app_bytes_recv);
		    send_udp(app_buffer);
		  }
			app_bytes_recv = 0;
		}
	}

}

static struct stimer ipPrintTimer;
void debugPrint() {
	// Print all our Ip addresses when the timer trigs.
	if (stimer_expired(&ipPrintTimer)) {
		stimer_reset(&ipPrintTimer);
		printIPAddresses();
		printIPV6Neighbourlist();
#if (FILTERED_BORDER_ROUTER)
		printPrefixListRouter();
		printPrefixListNonRouter();
#endif
		ule6lo_nbr_t neigh;
		/*PRINT_CYAN("Neighbour list size = %i\n",ule6loTestIn_getNbListSize());
		for(int i=0;i<ule6loTestIn_getNbListSize();i++) {
			ule6loTestIn_getNbList(i,&neigh);
			printIPEI(&neigh.lladdr);
			printf(" %02X\n",neigh.paddr.u8[15]);
		}*/
		//PRINT_CYAN("Rx counter = %i, TX counter = %i\n",ule6loTestIn_getnofReceivedPacket(),ule6loTestIn_getnofSentPacket());
		PRINT_CYAN("DECT RX =%i, TX = %i. BR RX = %i, TX = %i\n",ule6loTestIn_getnofReceivedPacket(),ule6loTestIn_getnofSentPacket(), ule6loTestIn_getnofBorderRouterReceivedPacket(), ule6loTestIn_getnofBorderRouterSentPacket());
	}
}

int main(int argc, char **argv) {
	int flags; // Used to set non-blocking for stdin
	int retval;
	ule6lo_IPEI_t ipei;
	/* Make standard output unbuffered. */
	setvbuf(stdout, (char *)NULL, _IONBF, 0);

	// Initialize file logger
	file_logger_init("app_6LBR.log");
	file_logger_append_raw("Started\n", 8);
	file_logger_append_format("Test", 4, "%i", 18);

	//Initialize test hooks
	ule6loTestIn_regRxHook(&test_rx);
	ule6loTestIn_regTxHook(&test_tx);
	ule6loTestIn_regBorderRouterRxHook(&test_br_rx);
	ule6loTestIn_regBorderRouterTxHook(&test_br_tx);

	//Create fixed neighbour list
	memset((char*) neighbourlist, 0, sizeof(neighbourlist));
	neighbourlist[0].port = 11111;
	neighbourlist[0].ip = inet_addr("127.0.0.1");
	neighbourlist[0].lladdr.id[0] = 1;
	neighbourlist[1].port = 22222;
	neighbourlist[1].lladdr.id[0] = 2;
	neighbourlist[1].ip = inet_addr("127.0.0.1");
	neighbourlist[2].port = 33333;
	neighbourlist[2].lladdr.id[0] = 3;
	neighbourlist[2].ip = inet_addr("127.0.0.1");

	//Read PC address
	readPCMacAddress();


	// Print contiki setup information (#defines etc. )
	printContikiConfigurationInfo();

  ipei.id[0] = 0x01;
  ipei.id[1] = 0x00;
  ipei.id[2] = 0x00;
  ipei.id[3] = 0x00;
  ipei.id[4] = 0x09;
	// Initialize the stack
	ule6loGI_init(&ipei);
	
	// Force fixed IP address
	sixLowpanBorderRouterInit(IPV6_FIXED_PREFIX);

	//Print our link local address
	printLinkLocalAddress();

	// Initialize link layer
	lla_init();

	// Setup stdin to operate non blocking
	flags = fcntl(0, F_GETFL, 0);
	flags |= O_NONBLOCK;
	fcntl(0, F_SETFL, flags);

	//Reset number of received bytes
	app_bytes_recv = 0;

	//Print instruction message
	printf(
			"To send a message use this format and acknowledge with enter:\n<Destination ip> <Destination port> <message>\n");
	printf("Use port 3001 for now\n");

	process_start(&example_pollhandler, NULL);
	process_post(&example_pollhandler,0,NULL);
	stimer_set(&ipPrintTimer,10 );

	// Print all our Ip addresses
	printIPAddresses();
	printIPV6Neighbourlist();

	debugPrint();

	init_udp();
	uint8_t prefix[16];
	uint8_t prefix2[2];
    memset(prefix, 0, sizeof(prefix));
    memset(prefix2, 0, sizeof(prefix2));
    prefix2[0] = 0xfe;
    prefix2[1] = 0x80;
    prefix[0] = 0x20;
    prefix[1] = 0x01;
    prefix[2] = 0x20;
    prefix[3] = 0x01;
    prefix[4] = 0x20;
    prefix[5] = 0x01;
    prefix[6] = 0x20;
    prefix[7] = 0x02;
    prefix[8] = 0x3e;
    prefix[9] = 0x97;
    prefix[10] = 0x0e;
    prefix[11] = 0xff;
    prefix[12] = 0xfe;
    prefix[13] = 0x00;
    prefix[14] = 0x00;
    prefix[15] = 0x01;
    
    prefix2[0] = 0xfe;
    prefix2[1] = 0x80;
   // ule6loGI_addContext(prefix2, 2);
    ule6loGI_addContext(prefix, 8);
	while (1) {
		// Check the stdin for data
		readCommandLine();

		//Let the Link layer abstraction run
		llaProcess();

		//Let the stack process
		ule6loOS_processRun();

		// Print all our Ip addresses when the timer trigs.
		debugPrint(ipPrintTimer);

		// Handle registered file descriptors
		handle_callbacks();
	}
	return 0;
}


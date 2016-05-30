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


#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <stdio.h>
#include <string.h>

int main( void )
{
    int       max_servers = 2;
    int       server_handle[max_servers];
    int       max_server_handle = 0;
    struct    sockaddr_in6 server_address[max_servers];
    struct    sockaddr_in6 client_address[max_servers];
    char      buffer[1000];
    socklen_t client_length;
    fd_set    read_handles;
    struct    timeval timeout_interval;
    int       bytes_received;
    int       port_number = 3001;
    int       retval;
    int       i;

    printf("Hello, human.\n");

    for (i = 0; i < max_servers; i++)
    {
        printf("Creating socket %d on port: %d\n", i, port_number + i);

        server_handle[i] = socket(PF_INET6, SOCK_DGRAM, 0);
        if (server_handle[i] < 0)
        {
            perror("Unable to create socket.");
            return -1;
        }

        if (server_handle[i] > max_server_handle)
            max_server_handle = server_handle[i];

        memset( &server_address[i], 0, sizeof( server_address[i] ) );
        server_address[i].sin6_family = AF_INET6;
        server_address[i].sin6_addr = in6addr_any;
        server_address[i].sin6_port = htons( port_number + i );

        if (bind( server_handle[i], (struct sockaddr *)&server_address[i], sizeof( server_address[i] )) < 0)
        {
            perror("Unable to bind.");
            return -1;
        }

        printf("Bind %d successful.\n", i);
    }

    while (1)
    {
        FD_ZERO(&read_handles);
        for (i = 0; i < max_servers; i++)
            FD_SET(server_handle[i], &read_handles);

        timeout_interval.tv_sec = 2;
        timeout_interval.tv_usec = 500000;

        retval = select(max_server_handle + 1, &read_handles, NULL, NULL, &timeout_interval);
        if (retval == -1)
        {
            printf("Select error\n");
            //error
        }
        else if (retval == 0)
        {
            printf("timeout\n");
        }
        else
        {
            //good
            for (i = 0; i < max_servers; i++)
            {
                if (FD_ISSET(server_handle[i], &read_handles))
                {
                    client_length = sizeof(client_address[i]);

                    if ((bytes_received = recvfrom(server_handle[i], buffer, sizeof(buffer), 0, (struct sockaddr *)&client_address[i], &client_length)) < 0)
                    {
                        perror("Error in recvfrom.");
                        break;
                    }

                    printf("\nData received on socket %d:", i);
                    printf("\n--------------\n");
                    printf("%.*s", bytes_received, buffer);
                }
            }
        }
    }
}

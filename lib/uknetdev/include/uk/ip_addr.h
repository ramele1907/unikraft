/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

#ifndef __UK_IP_ADDR__
#define __UK_IP_ADDR__

#include <stdint.h>
#include <stddef.h>
#include <uk/assert.h>

typedef union {
	uint32_t ipv4_addr;     /**< IPv4 address in big endian. */
	uint32_t ipv6_addr[4];  /**< IPv6 address in big endian. */
} uk_ip_addr_t;

typedef struct {
	uk_ip_addr_t ip;
	uk_ip_addr_t gateway;
	uk_ip_addr_t netmask;
} uk_ip_info_t;


int uk_ip4addr_aton(const char *cp, uk_ip_addr_t *addr);
char* uk_ip4addr_ntoa(const uk_ip_addr_t *addr, char *buf, int buflen);

int uk_ip6addr_aton(const char *cp, uk_ip_addr_t *addr);
char *uk_ip6addr_ntoa_r(const uk_ip_addr_t *addr, char *buf, int buflen);

#endif //__UK_IP_ADDR__

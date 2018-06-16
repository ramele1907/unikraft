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

#include <uk/ip_addr.h>

#ifndef isprint
#define in_range(c, lo, up)  ((uint8_t)c >= lo && (uint8_t)c <= up)
#define isprint(c)           in_range(c, 0x20, 0x7f)
#define isdigit(c)           in_range(c, '0', '9')
#define isxdigit(c)          (isdigit(c) || in_range(c, 'a', 'f') || in_range(c, 'A', 'F'))
#define islower(c)           in_range(c, 'a', 'z')
#define isspace(c)           (c == ' ' || c == '\f' || c == '\n' || c == '\r' || c == '\t' || c == '\v')
#define xchar(i)             ((i) < 10 ? '0' + (i) : 'A' + (i) - 10)
#endif

#define UK_HTONL(x) ((((x) & 0x000000ffUL) << 24) | \
                     (((x) & 0x0000ff00UL) <<  8) | \
                     (((x) & 0x00ff0000UL) >>  8) | \
                     (((x) & 0xff000000UL) >> 24))

/**
 * Check whether "cp" is a valid ascii representation
 * of an Internet address and convert to a binary address.
 * Returns 1 if the address is valid, 0 if not.
 * This replaces inet_addr, the return value from which
 * cannot distinguish between failure and a local broadcast address.
 *
 * @param cp IP address in ascii representation (e.g. "127.0.0.1")
 * @param addr pointer to which to save the ip address in network order
 * @return 1 if cp could be converted to addr, 0 on failure
 */
int uk_ip4addr_aton(const char *cp, uk_ip_addr_t *addr)
{
	uint32_t val;
	uint8_t base;
	char c;
	uint32_t parts[4];
	uint32_t *pp = parts;

	c = *cp;
	for (;;) {
		/*
		 * Collect number up to ``.''.
		 * Values are specified as for C:
		 * 0x=hex, 0=octal, 1-9=decimal.
		 */
		if (!isdigit(c)) {
			return 0;
		}
		val = 0;
		base = 10;
		if (c == '0') {
			c = *++cp;
			if (c == 'x' || c == 'X') {
				base = 16;
				c = *++cp;
			} else {
				base = 8;
			}
		}
		for (;;) {
			if (isdigit(c)) {
				val = (val * base) + (uint32_t)(c - '0');
				c = *++cp;
			} else if (base == 16 && isxdigit(c)) {
				val = (val << 4) | (uint32_t)(c + 10 - (islower(c) ? 'a' : 'A'));
				c = *++cp;
			} else {
				break;
			}
		}
		if (c == '.') {
			/*
			 * Internet format:
			 *  a.b.c.d
			 *  a.b.c   (with c treated as 16 bits)
			 *  a.b (with b treated as 24 bits)
			 */
			if (pp >= parts + 3) {
				return 0;
			}
			*pp++ = val;
			c = *++cp;
		} else {
			break;
		}
	}
	/*
	 * Check for trailing characters.
	 */
	if (c != '\0' && !isspace(c)) {
		return 0;
	}
	/*
	 * Concoct the address according to
	 * the number of parts specified.
	 */
	switch (pp - parts + 1) {

		case 0:
			return 0;       /* initial nondigit */

		case 1:             /* a -- 32 bits */
			break;

		case 2:             /* a.b -- 8.24 bits */
			if (val > 0xffffffUL) {
				return 0;
			}
			if (parts[0] > 0xff) {
				return 0;
			}
			val |= parts[0] << 24;
			break;

		case 3:             /* a.b.c -- 8.8.16 bits */
			if (val > 0xffff) {
				return 0;
			}
			if ((parts[0] > 0xff) || (parts[1] > 0xff)) {
				return 0;
			}
			val |= (parts[0] << 24) | (parts[1] << 16);
			break;

		case 4:             /* a.b.c.d -- 8.8.8.8 bits */
			if (val > 0xff) {
				return 0;
			}
			if ((parts[0] > 0xff) || (parts[1] > 0xff) || (parts[2] > 0xff)) {
				return 0;
			}
			val |= (parts[0] << 24) | (parts[1] << 16) | (parts[2] << 8);
			break;
		default:
			uk_printd(DLVL_ERR, "ipv4 addr parse error\n");
			break;
	}
	if (addr) {
		addr->ipv4_addr = (uint32_t)UK_HTONL(val);
	}
	return 1;
}

/**
 * Convert numeric IP address into decimal dotted ASCII representation.
 *
 * @param addr ip address in network order to convert
 * @param buf target buffer where the string is stored
 * @param buflen length of buf
 * @return either pointer to buf which now holds the ASCII
 *         representation of addr or NULL if buf was too small
 */
char* uk_ip4addr_ntoa(const uk_ip_addr_t *addr, char *buf, int buflen)
{
	uint32_t s_addr;
	char inv[3];
	char *rp;
	uint8_t *ap;
	uint8_t rem;
	uint8_t n;
	uint8_t i;
	int len = 0;

	s_addr = addr->ipv4_addr;

	rp = buf;
	ap = (uint8_t *)&s_addr;
	for (n = 0; n < 4; n++) {
		i = 0;
		do {
			rem = *ap % (uint8_t)10;
			*ap /= (uint8_t)10;
			inv[i++] = (char)('0' + rem);
		} while (*ap);
		while (i--) {
			if (len++ >= buflen) {
				return NULL;
			}
			*rp++ = inv[i];
		}
		if (len++ >= buflen) {
			return NULL;
		}
		*rp++ = '.';
		ap++;
	}
	*--rp = 0;
	return buf;
}

/**
 * Check whether "cp" is a valid ascii representation
 * of an IPv6 address and convert to a binary address.
 * Returns 1 if the address is valid, 0 if not.
 *
 * @param cp IPv6 address in ascii representation (e.g. "FF01::1")
 * @param addr pointer to which to save the ip address in network order
 * @return 1 if cp could be converted to addr, 0 on failure
 */
int uk_ip6addr_aton(const char *cp, uk_ip_addr_t *addr)
{
	uint32_t addr_index, zero_blocks, current_block_index, current_block_value;
	const char *s;

	/* Count the number of colons, to count the number of blocks in a "::" sequence
	   zero_blocks may be 1 even if there are no :: sequences */
	zero_blocks = 8;
	for (s = cp; *s != 0; s++) {
		if (*s == ':') {
			zero_blocks--;
		} else if (!isxdigit(*s)) {
			break;
		}
	}

	/* parse each block */
	addr_index = 0;
	current_block_index = 0;
	current_block_value = 0;
	for (s = cp; *s != 0; s++) {
		if (*s == ':') {
			if (addr) {
				if (current_block_index & 0x1) {
					addr->ipv6_addr[addr_index++] |= current_block_value;
				}
				else {
					addr->ipv6_addr[addr_index] = current_block_value << 16;
				}
			}
			current_block_index++;
			current_block_value = 0;
			if (current_block_index > 7) {
				/* address too long! */
				return 0;
			}
			if (s[1] == ':') {
				if (s[2] == ':') {
					/* invalid format: three successive colons */
					return 0;
				}
				s++;
				/* "::" found, set zeros */
				while (zero_blocks > 0) {
					zero_blocks--;
					if (current_block_index & 0x1) {
						addr_index++;
					} else {
						if (addr) {
							addr->ipv6_addr[addr_index] = 0;
						}
					}
					current_block_index++;
					if (current_block_index > 7) {
						/* address too long! */
						return 0;
					}
				}
			}
		} else if (isxdigit(*s)) {
			/* add current digit */
			current_block_value = (current_block_value << 4) +
								  (isdigit(*s) ? (uint32_t)(*s - '0') :
								   (uint32_t)(10 + (islower(*s) ? *s - 'a' : *s - 'A')));
		} else {
			/* unexpected digit, space? CRLF? */
			break;
		}
	}

	if (addr) {
		if (current_block_index & 0x1) {
			addr->ipv6_addr[addr_index++] |= current_block_value;
		}
		else {
			addr->ipv6_addr[addr_index] = current_block_value << 16;
		}
	}

	/* convert to network byte order. */
	if (addr) {
		for (addr_index = 0; addr_index < 4; addr_index++) {
			addr->ipv6_addr[addr_index] =
					(uint32_t)UK_HTONL(addr->ipv6_addr[addr_index]);
		}
	}

	if (current_block_index != 7) {
		return 0;
	}

	return 1;
}

/**
 * Convert numeric IPv6 address into ASCII representation.
 *
 * @param addr ip6 address in network order to convert
 * @param buf target buffer where the string is stored
 * @param buflen length of buf
 * @return either pointer to buf which now holds the ASCII
 *         representation of addr or NULL if buf was too small
 */
char *uk_ip6addr_ntoa_r(const uk_ip_addr_t *addr, char *buf, int buflen)
{
	uint32_t current_block_index, current_block_value, next_block_value;
	int32_t i;
	uint8_t zero_flag, empty_block_flag;

	i = 0;
	empty_block_flag = 0; /* used to indicate a zero chain for "::' */

	for (current_block_index = 0; current_block_index < 8; current_block_index++) {
		/* get the current 16-bit block */
		current_block_value =
				(uint32_t)UK_HTONL(addr->ipv6_addr[current_block_index >> 1]);
		if ((current_block_index & 0x1) == 0) {
			current_block_value = current_block_value >> 16;
		}
		current_block_value &= 0xffff;

		/* Check for empty block. */
		if (current_block_value == 0) {
			if (current_block_index == 7 && empty_block_flag == 1) {
				/* special case, we must render a ':' for the last block. */
				buf[i++] = ':';
				if (i >= buflen) {
					return NULL;
				}
				break;
			}
			if (empty_block_flag == 0) {
				/* generate empty block "::", but only if more than one
				 * contiguous zero block, according to current formatting
				 * suggestions RFC 5952. */
				next_block_value = (uint32_t)UK_HTONL(
						addr->ipv6_addr[(current_block_index + 1) >> 1]);
				if ((current_block_index & 0x1) == 0x01) {
					next_block_value = next_block_value >> 16;
				}
				next_block_value &= 0xffff;
				if (next_block_value == 0) {
					empty_block_flag = 1;
					buf[i++] = ':';
					if (i >= buflen) {
						return NULL;
					}
					continue; /* move on to next block. */
				}
			} else if (empty_block_flag == 1) {
				/* move on to next block. */
				continue;
			}
		} else if (empty_block_flag == 1) {
			/* Set this flag value so we don't produce multiple empty blocks. */
			empty_block_flag = 2;
		}

		if (current_block_index > 0) {
			buf[i++] = ':';
			if (i >= buflen) {
				return NULL;
			}
		}

		if ((current_block_value & 0xf000) == 0) {
			zero_flag = 1;
		} else {
			buf[i++] = xchar(((current_block_value & 0xf000) >> 12));
			zero_flag = 0;
			if (i >= buflen) {
				return NULL;
			}
		}

		if (((current_block_value & 0xf00) == 0) && (zero_flag)) {
			/* do nothing */
		} else {
			buf[i++] = xchar(((current_block_value & 0xf00) >> 8));
			zero_flag = 0;
			if (i >= buflen) {
				return NULL;
			}
		}

		if (((current_block_value & 0xf0) == 0) && (zero_flag)) {
			/* do nothing */
		}
		else {
			buf[i++] = xchar(((current_block_value & 0xf0) >> 4));
			zero_flag = 0;
			if (i >= buflen) {
				return NULL;
			}
		}

		buf[i++] = xchar((current_block_value & 0xf));
		if (i >= buflen) {
			return NULL;
		}
	}

	buf[i] = 0;

	return buf;
}
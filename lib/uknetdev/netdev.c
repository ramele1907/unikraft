/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Simon Kuenzer <simon.kuenzer@neclab.eu>
 *
 * Copyright (c) 2017, NEC Europe Ltd., NEC Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * THIS HEADER MAY NOT BE EXTRACTED OR MODIFIED IN ANY WAY.
 */

#include <uk/netdev.h>
#include <string.h>
#include <uk/assert.h>
#include <uk/print.h>
#include <uk/plat/ctors.h>

struct uk_netdev_list uk_netdev_list;
static unsigned int netdev_count;


void uk_netdev_register(struct uk_netdev *dev)
{
	UK_ASSERT(dev != NULL);

	uk_printd(DLVL_INFO, "Register netdev%u: %p\n",
		  netdev_count, dev);
	dev->data->id = netdev_count;
	UK_TAILQ_INSERT_TAIL(&uk_netdev_list, dev, next);

	++netdev_count;
}

unsigned int uk_netdev_count(void)
{
	return netdev_count;
}

struct uk_netdev *uk_netdev_get(unsigned int id)
{
	struct uk_netdev *n;

	UK_NETDEV_LIST_FOREACH(n) {
		if (n->data->id == id)
			return n;
	}
	return NULL;
}

int uk_netdev_configure(struct uk_netdev *dev, const struct uk_netdev_conf *eth_conf)
{
	UK_ASSERT(dev);
	uk_printd(DLVL_INFO, "Configure device 0x%p\n", dev);

	if (eth_conf == NULL) {
		dev->data->driver_mode = UK_NETDEV_MODE_POLLING;
	} else {
		if ((eth_conf->requested_mode & dev->data->supported_modes) != 0) {
			dev->data->driver_mode = eth_conf->requested_mode;
		} else {
			uk_printd(DLVL_ERR, "Invalid driver mode requested\n");
			return -EINVAL;
		}
	}

	uk_netdev_ip_set(dev, NULL);

	return dev->dev_ops->dev_configure(dev);
}

int uk_netdev_rx_queue_setup(struct uk_netdev *dev, uint16_t rx_queue_id,
							 const struct uk_netdev_rxconf *rx_conf)
{
	UK_ASSERT(dev);

	if (dev->data->driver_mode != UK_NETDEV_MODE_POLLING) {
		if (rx_conf == NULL) {
			uk_printd(DLVL_ERR,
					  "Interrupt-based mode requested with no callback\n");
			return -EINVAL;
		}
		dev->rx_cb = rx_conf->rx_cb;
	}

	return dev->dev_ops->rx_queue_setup(dev, rx_queue_id, rx_conf);
}

int uk_netdev_tx_queue_setup(struct uk_netdev *dev, uint16_t tx_queue_id,
							 const struct uk_netdev_txconf *tx_conf)
{
	UK_ASSERT(dev);
	return dev->dev_ops->tx_queue_setup(dev, tx_queue_id, tx_conf);
}

int uk_netdev_start(struct uk_netdev *dev)
{
	UK_ASSERT(dev);
	return dev->dev_ops->dev_start(dev);
}

int uk_netdev_mac_addr_set(struct uk_netdev *dev, struct ether_addr *mac_addr)
{
	UK_ASSERT(dev);
	memcpy(&dev->data->mac_addr, mac_addr, sizeof(struct ether_addr));
	return 0;
}

int uk_netdev_ip_set(struct uk_netdev *dev, uk_ip_info_t *ip)
{
	UK_ASSERT(dev);
	dev->data->ip_info = ip;
	return 0;
}

uk_ip_info_t *uk_netdev_ip_get(struct uk_netdev *dev)
{
	UK_ASSERT(dev);
	return dev->data->ip_info;
}

int uk_netdev_mtu_set(struct uk_netdev *dev, uint16_t mtu)
{
	UK_ASSERT(dev);
	dev->data->mtu = mtu;
	return 0;
}

uint16_t uk_netdev_rx(struct uk_netdev *dev, void *data, uint16_t max_len)
{
	UK_ASSERT(dev);
	return dev->rx_pkt(dev, data, max_len);
}

uint16_t uk_netdev_tx(struct uk_netdev *dev, void *data, uint16_t len)
{
	UK_ASSERT(dev);
	return dev->tx_pkt(dev, data, len);
}

static void _uk_netdev_ctor(void)
{
	UK_TAILQ_INIT(&uk_netdev_list);
	netdev_count = 0;
}

/*
 * Declare the constructor function to initialize the netdev
 */
static void _uk_netdev_ctor(void) __constructor_prio(200U);

/* This library does not have any dependency to another library for
 * initialization, except a libc -> We use priority 1 */
//UKPLAT_CTOR_FUNC(1, _uk_netdev_ctor);

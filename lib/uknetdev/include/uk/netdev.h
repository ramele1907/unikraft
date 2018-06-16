/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Simon Kuenzer <simon.kuenzer@neclab.eu>
 *
 * Copyright (c) 2018, NEC Europe Ltd., NEC Corporation. All rights reserved.
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

#ifndef __UK_NETDEV__
#define __UK_NETDEV__

/**
 * TODO detail doc
 * The functions exported by the Unikraft NET API to setup a device
 * designated by its ID must be invoked in the following order:
 *     - uk_netdev_configure()
 *     - uk_netdev_tx_queue_setup()
 *     - uk_netdev_rx_queue_setup()
 *     - uk_netdev_start()
 */


#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#include <uk/list.h>
#include <uk/alloc.h>
#include <uk/print.h>
#include <uk/assert.h>
#include "netdev_core.h"

#ifdef __cplusplus
extern "C" {
#endif

unsigned int uk_netdev_count();

/**
 * Get a reference to a Unikraft Network Device, based on its ID.
 * This reference should be saved by the application and used for subsequent
 * API calls.
 *
 * @param id
 *   The identifier of the Ethernet device to configure.
 * @return
 *   - NULL: device not found in list
 *   - (struct uk_netdev *): reference to be passed to API calls
 * */
struct uk_netdev *uk_netdev_get(unsigned int id);

/**
 * Configure an Ethernet device.
 * This function must be invoked first before any other function in the
 * Ethernet API. This function can also be re-invoked when a device is in the
 * stopped state.
 *
 * @param dev
 *   The Unikraft Network Device.
 * @param eth_conf
 *   The pointer to the configuration data to be used for the Ethernet device.
 *
 *   Embedding all configuration information in a single data structure
 *   is the more flexible method that allows the addition of new features
 *   without changing the syntax of the API.
 * @return
 *   - 0: Success, device configured.
 *   - <0: Error code returned by the driver configuration function.
 */
int uk_netdev_configure(struct uk_netdev *dev, const struct uk_netdev_conf *conf);

/**
 * Start an Ethernet device.
 *
 * The device start step is the last one and consists of setting the configured
 * offload features and in starting the transmit and the receive units of the
 * device.
 * On success, all basic functions exported by the Ethernet API (link status,
 * receive/transmit, and so on) can be invoked.
 *
 * @param dev
 *   The Unikraft Network Device.
 * @return
 *   - 0: Success, Ethernet device started.
 *   - <0: Error code of the driver device start function.
 */
int uk_netdev_start(struct uk_netdev *dev);

/**
 * Set the default MAC address.
 *
 * @param dev
 *   The Unikraft Network Device.
 * @param mac_addr
 *   New default MAC address.
 * @return
 *   - (0) if successful, or *mac_addr* didn't exist.
 *   - (-ENOTSUP) if hardware doesn't support.
 *   - (-ENODEV) if *id* invalid.
 *   - (-EINVAL) if MAC address is invalid.
 */
int uk_netdev_mac_addr_set(struct uk_netdev *dev, struct ether_addr *mac_addr);

/**
 * TODO doc
 * */
int uk_netdev_ip_set(struct uk_netdev *dev, uk_ip_info_t *ip_info);

/**
 * TODO doc
 * */
uk_ip_info_t *uk_netdev_ip_get(struct uk_netdev *dev);

/**
 * Change the MTU of an Ethernet device.
 *
 * @param dev
 *   The Unikraft Network Device.
 * @param mtu
 *   A uint16_t for the MTU to be applied.
 * @return
 *   - (0) if successful.
 *   - (-ENOTSUP) if operation is not supported.
 *   - (-ENODEV) if *id* invalid.
 *   - (-EIO) if device is removed.
 *   - (-EINVAL) if *mtu* invalid.
 *   - (-EBUSY) if operation is not allowed when the device is running
 */
int uk_netdev_mtu_set(struct uk_netdev *dev, uint16_t mtu);

/**
 * Allocate and set up a receive queue for an Ethernet device.
 *
 * The function handles setup of receive callback for interrupt-based modes.
 *
 * @param dev
 *   The Unikraft Network Device.
 * @param rx_queue_id
 *   The index of the receive queue to set up.
 *   The value must be in the range [0, nb_rx_queue - 1] previously supplied
 *   to rte_eth_dev_configure().
 * @param rx_conf
 *   The pointer to the configuration data to be used for the receive queue.
 *   NULL value is allowed, in which case default RX configuration
 *   will be used.
 *   The *rx_conf* structure contains an *rx_thresh* structure with the values
 *   of the Prefetch, Host, and Write-Back threshold registers of the receive
 *   ring.
 *   In addition it contains the hardware offloads features to activate using
 *   the DEV_RX_OFFLOAD_* flags.
 * @return
 *   - 0: Success, receive queue correctly set up.
 *   - -EIO: if device is removed.
 */
int uk_netdev_rx_queue_setup(struct uk_netdev *dev, uint16_t rx_queue_id,
						   const struct uk_netdev_rxconf *rx_conf);

/**
 * Allocate and set up a transmit queue for an Ethernet device.
 *
 * @param dev
 *   The Unikraft Network Device.
 * @param tx_queue_id
 *   The index of the transmit queue to set up.
 *   The value must be in the range [0, nb_tx_queue - 1] previously supplied
 *   to rte_eth_dev_configure().
 * @param tx_conf
 *   The pointer to the configuration data to be used for the transmit queue.
 *   NULL value is allowed, in which case default TX configuration
 *   will be used.
 * @return
 *   - 0: Success, the transmit queue is correctly set up.
 *   - -ENOMEM: Unable to allocate the transmit ring descriptors.
 */
int uk_netdev_tx_queue_setup(struct uk_netdev *dev, uint16_t tx_queue_id,
						   const struct uk_netdev_txconf *tx_conf);

/**
 * Basic RX function.
 * @param dev
 *   The Unikraft Network Device.
 * @param data
 *   The data pointer where the packet will be placed by the driver.
 * @param max_len
 *   Maximum length of the packet.
 * @return
 *   - 0: No new packets
 *   - >0: Length of the received packet
 */
uint16_t uk_netdev_rx(struct uk_netdev *dev, void *data, uint16_t max_len);

/**
 * Basic TX function.
 * @param dev
 *   The Unikraft Network Device.
 * @param data
 *   Raw packet data (including Ethernet headers) to be sent by the driver.
 * @param len
 *   The length of the packet.
 * @return
 */
uint16_t uk_netdev_tx(struct uk_netdev *dev, void *data, uint16_t len);

void uk_netdev_register(struct uk_netdev *dev);

#ifdef __cplusplus
}
#endif

#endif //__UK_NETDEV__

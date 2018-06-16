
#ifndef __UK_NETDEV_CORE__
#define __UK_NETDEV_CORE__

/**
 * TODO doc
 * */

#include "ip_addr.h"


#define NETDEV_MAX_QUEUES 10
#define NETDEV_NAME_MAX_LEN 64

#define ETHER_ADDR_LEN 6 /**< Length of Ethernet address. */


struct ether_addr {
	uint8_t addr_bytes[ETHER_ADDR_LEN]; /**< Addr bytes in tx order */
} __attribute__((__packed__));

/**
 * A set of values to describe the possible states of an eth device.
 */
enum uk_netdev_state {
	UK_NETDEV_UNCONFIGURED = 0,
	UK_NETDEV_CONFIGURED,
	UK_NETDEV_RUNNING,
};


/**
 * @defgroup driver_mode Driver Receive Mode
 * @{
 */

/** Basic polling mode driver. Provides RX/TX functions and it's the user's
 * responsibility when/how to call them. */
#define UK_NETDEV_MODE_POLLING   0x01U

/** Interrupt/Event mode driver. The user defines a callback which is called
 * by the driver when a new packet is received. */
#define UK_NETDEV_MODE_INTERRUPT 0x02U

/** Mixed Polling/Interrupt mode. Hybrid mode that continuously polls the
 * driver, and after a number of unsuccessful polls enters in sleep mode
 * and enables interrupts. From a user perspective, similar to Interrupt
 * mode. */
#define UK_NETDEV_MODE_HYBRID    0x04U

/**
 * @}
 */

/**
 * A structure used to configure an Ethernet device.
 */
struct uk_netdev_conf {
	uint8_t requested_mode;
};

UK_TAILQ_HEAD(uk_netdev_list, struct uk_netdev);

#define UK_NETDEV_LIST_FOREACH(b)			\
	UK_TAILQ_FOREACH(b, &uk_netdev_list, next)

/**
 * Function type used for RX packet processing packet callbacks.
 *
 * The callback function is called on RX with a packet that has been received
 * on the given device and queue.
 *
 * @param id
 *   The identifier of the device on which RX is being performed.
 * @param queue
 *   The queue on the Ethernet device which is being used to receive the packets.
 * @param data
 *   Content of the received packet.
 * @param len
 *   Length of the packet data.
 * @return
 *   The number of packets returned to the user.
 */
typedef void (*rx_callback_fn)(uint16_t id, uint16_t queue, void *data,
							   uint16_t len);

/**
 * A structure used to configure an Ethernet device.
 */
struct uk_netdev_rxconf {
	rx_callback_fn rx_cb;
};

/**
 * A structure used to configure an Ethernet device.
 */
struct uk_netdev_txconf {
};


typedef int  (*uk_netdev_configure_t)(struct uk_netdev *dev);
/**< @internal Ethernet device configuration. */

typedef int  (*uk_netdev_start_t)(struct uk_netdev *dev);
/**< @internal Function used to start a configured Ethernet device. */

typedef void (*uk_netdev_stop_t)(struct uk_netdev *dev);
/**< @internal Function used to stop a configured Ethernet device. */

typedef void (*uk_netdev_close_t)(struct uk_netdev *dev);
/**< @internal Function used to close a configured Ethernet device. */


typedef void (*uk_netdev_mac_addr_remove_t)(struct uk_netdev *dev);
/**< @internal Remove MAC address*/

typedef void (*uk_netdev_mac_addr_set_t)(struct uk_netdev *dev,
								   struct ether_addr *mac_addr);
/**< @internal Set the MAC address */

typedef int (*uk_netdev_mtu_set_t)(struct uk_netdev *dev, uint16_t mtu);
/**< @internal Set MTU. */

//typedef int (*eth_stats_get_t)(struct uk_netdev *dev,
//							   struct rte_eth_stats *igb_stats);
/**< @internal Get global I/O statistics of an Ethernet device. */

//typedef void (*eth_stats_reset_t)(struct uk_netdev *dev);
/**< @internal Reset global I/O statistics of an Ethernet device to 0. */

//typedef void (*uk_netdev_infos_get_t)(struct uk_netdev *dev,
//									struct uk_netdev_info *dev_info);
/**< @internal Get specific informations of an Ethernet device. */


typedef int (*uk_netdev_rx_queue_setup_t)(struct uk_netdev *dev,
									uint16_t rx_queue_id,
									const struct uk_netdev_rxconf *rx_conf);
/**< @internal Set up a receive queue of an Ethernet device. */

typedef int (*uk_netdev_tx_queue_setup_t)(struct uk_netdev *dev,
									uint16_t tx_queue_id,
									const struct uk_netdev_txconf *tx_conf);
/**< @internal Setup a transmit queue of an Ethernet device. */

typedef void (*uk_netdev_queue_release_t)(void *queue);
/**< @internal Release memory resources allocated by given RX/TX queue. */

typedef int (*uk_netdev_rx_enable_intr_t)(struct uk_netdev *dev,
									uint16_t rx_queue_id);
/**< @internal Enable interrupt of a receive queue of an Ethernet device. */

typedef int (*uk_netdev_rx_disable_intr_t)(struct uk_netdev *dev,
									 uint16_t rx_queue_id);
/**< @internal Disable interrupt of a receive queue of an Ethernet device. */


typedef uint16_t (*uk_netdev_rx_t)(struct uk_netdev *dev,
								   void *data, uint16_t len);
/**< @internal Retrieve one input packet from an Ethernet device. */

typedef uint16_t (*uk_netdev_tx_t)(struct uk_netdev *dev,
								   void *data, uint16_t len);
/**< @internal Send one output packet to an Ethernet device. */

typedef uint16_t (*uk_netdev_rx_burst_t)(void *rxq,
								   void **rx_pkts,
								   uint16_t nb_pkts);
/**< @internal Retrieve input packets from a receive queue of an Ethernet device. */

typedef uint16_t (*uk_netdev_tx_burst_t)(void *txq,
								   void **tx_pkts,
								   uint16_t nb_pkts);
/**< @internal Send output packets on a transmit queue of an Ethernet device. */


/**
 * @internal A structure containing the functions exported by an Ethernet driver.
 */
struct uk_netdev_ops {
	uk_netdev_configure_t      dev_configure; /**< Configure device. */
	uk_netdev_start_t          dev_start;     /**< Start device. */
//	uk_netdev_stop_t           dev_stop;      /**< Stop device. */
//	uk_netdev_close_t          dev_close;     /**< Close device. */

//	uk_netdev_mac_addr_remove_t      mac_addr_remove; /**< Remove MAC address. */
	uk_netdev_mac_addr_set_t         mac_addr_set;  /**< Set a MAC address. */
	uk_netdev_mtu_set_t              mtu_set;       /**< Set MTU. */
//	uk_netdev_stats_get_t            stats_get;     /**< Get generic device statistics. */
//	uk_netdev_stats_reset_t          stats_reset;   /**< Reset generic device statistics. */
//	uk_netdev_infos_get_t            dev_infos_get; /**< Get device info. */

	uk_netdev_rx_queue_setup_t       rx_queue_setup;/**< Set up device RX queue. */
	uk_netdev_queue_release_t        rx_queue_release; /**< Release RX queue. */
	uk_netdev_rx_enable_intr_t       rx_queue_intr_enable;  /**< Enable Rx queue interrupt. */
	uk_netdev_rx_disable_intr_t      rx_queue_intr_disable; /**< Disable Rx queue interrupt. */

	uk_netdev_tx_queue_setup_t       tx_queue_setup;/**< Set up device TX queue. */
	uk_netdev_queue_release_t        tx_queue_release; /**< Release TX queue. */
};

/**
 * @internal
 * The data part, with no function pointers, associated with each ethernet device.
 *
 * This structure is safe to place in shared memory to be common among different
 * processes in a multi-process configuration.
 */
struct uk_netdev_data {
	char name[NETDEV_NAME_MAX_LEN]; /**< Unique identifier name */
	uint16_t id;           /**< Device [external] port identifier. */

	void *rx_queue; /**< Pointer to RX queue. */
	void *tx_queue; /**< Pointer to TX queue. */

	/** Driver mode (@see @ref driver_mode). Requested by user. */
	uint8_t driver_mode;
//	uint16_t nb_rx_queues; /**< Number of RX queues. */
//	uint16_t nb_tx_queues; /**< Number of TX queues. */

	uk_ip_info_t *ip_info;

	struct ether_addr mac_addr;     /**< Device Ethernet Link address. */
//	struct rte_eth_link dev_link;   /**< Link-level information & status */
//	struct rte_eth_conf dev_conf;   /**< Configuration applied to device. */
	uint16_t mtu;                   /**< Maximum Transmission Unit. */

	enum uk_netdev_state state; /**< Flag indicating the device state */
	uint8_t rx_queue_state;
	/** Queues state: STARTED(1) / STOPPED(0) */
	uint8_t tx_queue_state;
	/** Queues state: STARTED(1) / STOPPED(0) */

	/** Supported modes (@see @ref driver_mode). Filled in by the driver. */
	uint8_t supported_modes;
};

/**
 * @internal
 * The generic data structure associated with each ethernet device.
 *
 * Pointers to burst-oriented packet receive and transmit functions are
 * located at the beginning of the structure, along with the pointer to
 * where all the data elements for the particular device are stored in shared
 * memory. This split allows the function pointer and driver data to be per-
 * process, while the actual configuration data for the device is shared.
 */
struct uk_netdev {
	UK_TAILQ_ENTRY(struct uk_netdev) next;

	uk_netdev_rx_burst_t rx_pkt_burst; /**< Pointer to burst receive function. */
	uk_netdev_tx_burst_t tx_pkt_burst; /**< Pointer to burst transmit function. */

	uk_netdev_rx_t rx_pkt; /**< Pointer to receive function. */
	uk_netdev_tx_t tx_pkt; /**< Pointer to transmit function. */

	struct uk_netdev_data *data;  /**< Pointer to device data */
	const struct uk_netdev_ops *dev_ops; /**< Functions exported by driver */

	/**
	 * User-supplied function called from on new packet RX
	 */
	rx_callback_fn rx_cb;

//	struct uk_netdev_intr_handle *intr_handle; /**< Device interrupt handle */
//	/** User application callbacks for NIC interrupts */
//	struct uk_netdev_cb_list link_intr_cbs;
//	/**
//	 * User-supplied function called from rx_burst to post-process
//	 * received packets before passing them to the user
//	 */
//	struct rte_eth_rxtx_callback *post_rx_burst_cb;
//	/**
//	 * User-supplied function called from tx_burst to pre-process
//	 * received packets before passing them to the driver for transmission.
//	 */
//	struct rte_eth_rxtx_callback *pre_tx_burst_cb;
};

#endif //__UK_NETDEV_CORE__

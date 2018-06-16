#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <uk/plat/lcpu.h>
#include <pci_bus.h>
#include <kvm/irq.h>
#include <virtio/virtio_ring.h>
#include <virtio/virtio_pci.h>
#include <uk/wait.h>
#include <uk/netdev.h>
#include <uk/print.h>
#include <uk/assert.h>
#include <uk/essentials.h>
#if CONFIG_HAVE_SCHED
#include <uk/thread.h>
#include <uk/wait.h>
#endif

#define VENDOR_QUMRANET_VIRTIO 0x1af4
#define PCI_CONF_SUBSYS_NET 1

/* The feature bitmap for virtio net */
#define VIRTIO_NET_F_CSUM 0       /* Host handles pkts w/ partial csum */
#define VIRTIO_NET_F_GUEST_CSUM 1 /* Guest handles pkts w/ partial csum */
#define VIRTIO_NET_F_MAC (1 << 5) /* Host has given MAC address. */

#define PKT_BUFFER_LEN 1526

static struct uk_alloc *a;

#define VIRTQ_RECV 0
#define VIRTQ_XMIT 1

/* This header comes first in the scatter-gather list.
 * If VIRTIO_F_ANY_LAYOUT is not negotiated, it must
 * be the first element of the scatter-gather list.  If you don't
 * specify GSO or CSUM features, you can simply ignore the header.
 */
struct __attribute__((__packed__)) virtio_net_hdr {
#define VIRTIO_NET_HDR_F_NEEDS_CSUM 1 /* Use csum_start, csum_offset */
#define VIRTIO_NET_HDR_F_DATA_VALID 2 /* Csum is valid */
	uint8_t flags;
#define VIRTIO_NET_HDR_GSO_NONE 0   /* Not a GSO frame */
#define VIRTIO_NET_HDR_GSO_TCPV4 1  /* GSO frame, IPv4 TCP (TSO) */
#define VIRTIO_NET_HDR_GSO_UDP 3    /* GSO frame, IPv4 UDP (UFO) */
#define VIRTIO_NET_HDR_GSO_TCPV6 4  /* GSO frame, IPv6 TCP */
#define VIRTIO_NET_HDR_GSO_ECN 0x80 /* TCP has ECN set */
	uint8_t gso_type;
	uint16_t hdr_len;     /* Ethernet + IP + tcp/udp hdrs */
	uint16_t gso_size;    /* Bytes to append to hdr_len per frame */
	uint16_t csum_start;  /* Position to start checksumming from */
	uint16_t csum_offset; /* Offset after that to place checksum */
};

struct virtio_net_device {
	struct pci_device *dev;
	struct uk_netdev netdev;
	uint16_t pci_base; /* base in PCI config space */
	struct virtq recvq;
	struct virtq xmitq;
	struct uk_thread *thread;
	struct uk_waitq wq;
};

static int virtio_net_irq_handle(void *arg)
{
	struct virtio_net_device *d = (struct virtio_net_device *) arg;
	uint8_t isr_status;

	if (unlikely(d->netdev.data->state != UK_NETDEV_RUNNING))
		return 0;

	isr_status = inb(d->pci_base + VIRTIO_PCI_ISR);
	if (isr_status & VIRTIO_PCI_ISR_HAS_INTR) {
		/* This interrupt is just to kick the application out of any
		 * poll() that may be running. */
		uk_waitq_wake_up(&d->wq);
		return 1;
	}
	return 0;
}

static void recv_setup(struct virtio_net_device *d)
{
	uint16_t mask = d->recvq.num - 1;
	do {
		struct io_buffer
		    *buf; /* header and data in a single descriptor */
		buf = &d->recvq.bufs[d->recvq.next_avail & mask];
		memset(buf->data, 0, PKT_BUFFER_LEN);
		buf->len = PKT_BUFFER_LEN;
		buf->extra_flags = VIRTQ_DESC_F_WRITE;
		UK_ASSERT(virtq_add_descriptor_chain(
			      &d->recvq, d->recvq.next_avail & mask, 1)
			  == 0);
	} while ((d->recvq.next_avail & mask) != 0);

	outw(d->pci_base + VIRTIO_PCI_QUEUE_NOTIFY, VIRTQ_RECV);
}

static uint16_t virtio_netdev_xmit(struct uk_netdev *n,
		void *data, uint16_t datalen)
{

	struct virtio_net_device *d;
	struct io_buffer *head_buf, *data_buf;
	uint16_t mask;
	uint16_t head;
	int r;

	UK_ASSERT(n != NULL);
	d = __containerof(n, struct virtio_net_device, netdev);
	mask = d->xmitq.num - 1;

	if (unlikely(datalen > PKT_BUFFER_LEN))
		return -EINVAL;

	/* Consume used descriptors from all the previous tx'es. */
	for (; d->xmitq.last_used != d->xmitq.used->idx; d->xmitq.last_used++)
		d->xmitq.num_avail += 2; /* 2 descriptors per chain */

	/* next_avail is incremented by virtq_add_descriptor_chain below. */
	head = d->xmitq.next_avail & mask;
	head_buf = &d->xmitq.bufs[head];
	data_buf = &d->xmitq.bufs[(head + 1) & mask];

	/* The header buf */
	memset(head_buf->data, 0, sizeof(struct virtio_net_hdr));
	head_buf->len = sizeof(struct virtio_net_hdr);
	head_buf->extra_flags = 0;

	/* The data buf */
	memcpy(data_buf->data, data, datalen);
	data_buf->len = datalen;
	data_buf->extra_flags = 0;

	r = virtq_add_descriptor_chain(&d->xmitq, head, 2);

	outw(d->pci_base + VIRTIO_PCI_QUEUE_NOTIFY, VIRTQ_XMIT);

	return (uint16_t) r; /* TODO this is ugly */
}

#if 0 /* TODO */
static int virtio_netdev_poll(struct uk_netdev *n)
{
	struct virtio_net_device *d;

	UK_ASSERT(n != NULL);
	d = __containerof(n, struct virtio_net_device, netdev);

	if (unlikely(d->netdev.data->state != UK_NETDEV_RUNNING))
		return 0;

	/* The device increments used->idx whenever it uses a packet (i.e. it
	 * put a packet on our receive queue) and if it's ahead of last_used it
	 * means that we have a pending packet. */
	if (d->recvq.last_used == d->recvq.used->idx)
		return 0;
	else
		return 1;
}
#endif

/* Get the data from the next_avail (top-most) receive buffer/descriptpr in
 * the available ring. */
static uint8_t *virtio_net_recv_pkt_get(struct virtio_net_device *d, int *size)
{
	uint16_t mask = d->recvq.num - 1;
	struct virtq_used_elem *e;
	struct io_buffer *buf;
	uint16_t desc_idx;

	/* The device increments used->idx whenever it uses a packet (i.e. it
	 * put a packet on our receive queue) and if it's ahead of last_used it
	 * means that we have a pending packet. */
	if (d->recvq.last_used == d->recvq.used->idx)
		return NULL;

	e = &(d->recvq.used->ring[d->recvq.last_used & mask]);
	desc_idx = e->id;

	buf = (struct io_buffer *)d->recvq.desc[desc_idx].addr;
	buf->len = e->len;

	/* Remove the virtio_net_hdr */
	*size = buf->len - sizeof(struct virtio_net_hdr);
	return buf->data + sizeof(struct virtio_net_hdr);
}

/* Return the next_avail (top-most) receive buffer/descriptor to the available
 * ring. */
static void virtio_net_recv_pkt_put(struct virtio_net_device *d)
{
	uint16_t mask = d->recvq.num - 1;
	d->recvq.bufs[d->recvq.next_avail & mask].len = PKT_BUFFER_LEN;
	d->recvq.bufs[d->recvq.next_avail & mask].extra_flags =
	    VIRTQ_DESC_F_WRITE;

	/* This sets the returned descriptor to be ready for incoming packets,
	 * and advances the next_avail index. */
	UK_ASSERT(
	    virtq_add_descriptor_chain(&d->recvq, d->recvq.next_avail & mask, 1)
	    == 0);
	outw(d->pci_base + VIRTIO_PCI_QUEUE_NOTIFY, VIRTQ_RECV);
}

#if 0
/* TODO zero copy */

static int virtio_netdev_recvzc(struct uk_netdev *n, const void **data, size_t *dlen)
{
	struct virtio_net_device *d;
	uint8_t *pkt;
	int len;

	UK_ASSERT(n != NULL);
	d = __containerof(n, struct virtio_net_device, netdev);

	/* We only need interrupts to wake up the application when it's sleeping
	 * and waiting for incoming packets. The app is definitely not doing
	 * that now (as we are here), so disable them. */
	d->recvq.avail->flags |= VIRTQ_AVAIL_F_NO_INTERRUPT;

	pkt = virtio_net_recv_pkt_get(d, &len);
	if (!pkt) {
		d->recvq.avail->flags &= ~VIRTQ_AVAIL_F_NO_INTERRUPT;
#if HAVE_SCHED
		uk_sched_yield();
#endif
		return -1;
	}

	*dlen = (size_t) len;

	*data = pkt;

	return 0;
}

static int virtio_netdev_recvzc_release(struct uk_netdev *n)
{
	struct virtio_net_device *d;

	UK_ASSERT(n != NULL);
	d = __containerof(n, struct virtio_net_device, netdev);

	d->recvq.last_used++;
	d->recvq.num_avail++;

	virtio_net_recv_pkt_put(d);

	d->recvq.avail->flags &= ~VIRTQ_AVAIL_F_NO_INTERRUPT;
	return 0;
}
#endif

static uint16_t virtio_netdev_recv(struct uk_netdev *n,
		void *data, uint16_t maxlen)
{
	struct virtio_net_device *d;
	uint8_t *pkt;
	int pktlen;
	size_t len;

	UK_ASSERT(n != NULL);
	d = __containerof(n, struct virtio_net_device, netdev);

	/* We only need interrupts to wake up the application when it's sleeping
	 * and waiting for incoming packets. The app is definitely not doing
	 * that now (as we are here), so disable them. */
	d->recvq.avail->flags |= VIRTQ_AVAIL_F_NO_INTERRUPT;

	pkt = virtio_net_recv_pkt_get(d, &pktlen);
	if (!pkt) {
		d->recvq.avail->flags &= ~VIRTQ_AVAIL_F_NO_INTERRUPT;
		return -1;
	}
	UK_ASSERT(pktlen <= PKT_BUFFER_LEN);

	/* also, it's clearly not zero copy */
	len = MIN(maxlen, (size_t) pktlen);
	memcpy(data, pkt, len);

	/* Consume the recently used descriptor. */
	d->recvq.last_used++;
	d->recvq.num_avail++;

	virtio_net_recv_pkt_put(d);

	d->recvq.avail->flags &= ~VIRTQ_AVAIL_F_NO_INTERRUPT;

	return (uint16_t) pktlen; /* TODO this is ugly */
}

static int virtio_netdev_rx_queue_setup(struct uk_netdev *n,
		uint16_t queue_id __unused,
		const struct uk_netdev_rxconf *conf __unused)
{
	struct virtio_net_device *d;
	int err;

	UK_ASSERT(n != NULL);
	d = __containerof(n, struct virtio_net_device, netdev);

	/*
	 * 7. Perform device-specific setup, including discovery of virtqueues
	 * for the device, optional per-bus setup, reading and possibly writing
	 * the device's virtio configuration space, and population of
	 * virtqueues.
	 */
	err = virtq_init_rings(a, d->pci_base, &d->recvq, VIRTQ_RECV);
	if (err)
		goto err_out;

	d->recvq.bufs = uk_calloc(a, d->recvq.num, sizeof(struct io_buffer));
	if (!d->recvq.bufs) {
		err = -ENOMEM;
		goto err_freeq;
	}

	recv_setup(d);

	return 0;

err_freeq:
	virtq_fini_rings(a, &d->recvq);
err_out:
	return err;
}

static int virtio_netdev_tx_queue_setup(struct uk_netdev *n,
		uint16_t queue_id __unused,
		const struct uk_netdev_txconf *conf __unused)
{
	struct virtio_net_device *d;
	int err;

	UK_ASSERT(n != NULL);
	d = __containerof(n, struct virtio_net_device, netdev);

	err = virtq_init_rings(a, d->pci_base, &d->xmitq, VIRTQ_XMIT);
	if (err)
		goto err_out;

	d->xmitq.bufs = uk_calloc(a, d->xmitq.num, sizeof(struct io_buffer));
	UK_ASSERT(d->recvq.bufs != NULL);
	if (!d->xmitq.bufs) {
		err = -ENOMEM;
		goto err_freeq;
	}

	return 0;

err_freeq:
	virtq_fini_rings(a, &d->xmitq);
err_out:
	return err;
}

static int virtio_netdev_configure(struct uk_netdev *n/*, struct uk_alloc *a TODO*/)
{
	struct virtio_net_device *d;
	uint32_t host_features, guest_features;
	struct ether_addr mac;

	UK_ASSERT(n != NULL);
	d = __containerof(n, struct virtio_net_device, netdev);

	d->pci_base = d->dev->base;

	/*
	 * 2. Set the ACKNOWLEDGE status bit: the guest OS has notice the
	 *    device.
	 * 3. Set the DRIVER status bit: the guest OS knows how to drive the
	 *    device.
	 */
	outb(d->pci_base + VIRTIO_PCI_STATUS, VIRTIO_PCI_STATUS_ACK);
	outb(d->pci_base + VIRTIO_PCI_STATUS, VIRTIO_PCI_STATUS_DRIVER);

	/*
	 * 4. Read device feature bits, and write the subset of feature bits
	 * understood by the OS and driver to the device. During this step the
	 * driver MAY read (but MUST NOT write) the device-specific
	 * configuration fields to check that it can support the device before
	 * accepting it.
	 */
	host_features = inl(d->pci_base + VIRTIO_PCI_HOST_FEATURES);
	UK_ASSERT(host_features & VIRTIO_NET_F_MAC);

	/* only negotiate that the mac was set for now */
	guest_features = VIRTIO_NET_F_MAC;
	outl(d->pci_base + VIRTIO_PCI_GUEST_FEATURES, guest_features);

	for (int i = 0; i < ETHER_ADDR_LEN; i++)
		mac.addr_bytes[i] = inb(d->pci_base + VIRTIO_PCI_CONFIG_OFF + i);
	uk_netdev_mac_addr_set(&d->netdev, &mac);

	ukplat_irq_register(d->dev->irq, virtio_net_irq_handle, d);

	/*
	 * 8. Set the DRIVER_OK status bit. At this point the device is "live".
	 */
	outb(d->pci_base + VIRTIO_PCI_STATUS, VIRTIO_PCI_STATUS_DRIVER_OK);

	d->netdev.data->state = UK_NETDEV_CONFIGURED;

	uk_printd(DLVL_INFO,
	    "PCI:%02x:%02x: Configured (features=0x%x, irq=%lu)\n",
	    d->dev->addr.bus, d->dev->addr.devid, host_features, d->dev->irq);

	return 0;
}

static void virtio_net_thread(void *arg)
{
	struct virtio_net_device *d = arg;
	struct uk_netdev *n;
	void *pkt;
	ssize_t pkt_size;

	UK_ASSERT(d != NULL);

	n = &d->netdev;

	pkt = uk_malloc(a, PKT_BUFFER_LEN);
	UK_ASSERT(pkt != NULL);

	while (1) {
		uk_waitq_wait_event(&d->wq, d->recvq.last_used != d->recvq.used->idx);

		pkt_size = virtio_netdev_recv(n, pkt, PKT_BUFFER_LEN);

		n->rx_cb(n->data->id, 0, pkt, pkt_size);
	}
}

int virtio_net_start(struct uk_netdev *n)
{
	struct virtio_net_device *d;

	UK_ASSERT(n != NULL);
	d = __containerof(n, struct virtio_net_device, netdev);

	uk_waitq_init(&d->wq);

	sprintf(n->data->name, "virtio-net%d", n->data->id);
	d->thread = uk_thread_create(n->data->name, virtio_net_thread, d);
	if (d->thread == NULL) {
		uk_printd(DLVL_ERR, "Error creating %s thread.", n->data->name);
		return -ENOMEM;
	}

	d->netdev.data->state = UK_NETDEV_RUNNING;

	/*
	 * We don't need to get interrupts every time the device uses our
	 * descriptors. Instead, we check for used packets in the transmit path
	 * of  following packets (as suggested in "5.1.6.2.1 Packet Transmission
	 * Interrupt").
	 */
	d->xmitq.avail->flags |= VIRTQ_AVAIL_F_NO_INTERRUPT;

	return 0;
}

static const struct uk_netdev_ops virtio_netdev_ops = {
	.dev_configure = virtio_netdev_configure,
	.rx_queue_setup = virtio_netdev_rx_queue_setup,
	.tx_queue_setup = virtio_netdev_tx_queue_setup,
	.dev_start = virtio_net_start,
};

static int virtio_net_add_dev(struct pci_device *dev)
{
	struct virtio_net_device *d;
	int err = -EINVAL;

	UK_ASSERT(dev != NULL);

	d = uk_malloc(a, sizeof(*d));
	if (!d) {
		err = -ENOMEM;
		goto err_out;
	}

	d->dev = dev;

	/* register netdev */
	d->netdev.rx_pkt = virtio_netdev_recv;
	d->netdev.tx_pkt = virtio_netdev_xmit;
	d->netdev.dev_ops = &virtio_netdev_ops;
#if 0
	/* TODO zero copy & polling */
	d->netdev.recvzc = virtio_netdev_recvzc;
	d->netdev.recvzc_release = virtio_netdev_recvzc_release;
	d->netdev.poll = virtio_netdev_poll;
#endif

	d->netdev.data = uk_malloc(a, sizeof(struct uk_netdev_data));
	d->netdev.data->state = UK_NETDEV_UNCONFIGURED;
	/* Netfront driver implements basic polling and interrupt */
	d->netdev.data->supported_modes = UK_NETDEV_MODE_POLLING |
									  UK_NETDEV_MODE_INTERRUPT;
	d->netdev.data->mtu = PKT_BUFFER_LEN;

	/* TODO set d->netdev.data->{rx,tx}_queue */

	uk_netdev_register(&d->netdev);

	return 0;

err_out:
	return err;
}

static int virtio_net_drv_init(struct uk_alloc *drv_allocator)
{
	/* driver initialization */
	if (!drv_allocator)
		return -EINVAL;

	a = drv_allocator;
	return 0;
}

static const struct pci_device_id pci_id_map[] = {
	{PCI_CLASS_ANY_ID, VENDOR_QUMRANET_VIRTIO, PCI_ANY_ID, PCI_ANY_ID, PCI_CONF_SUBSYS_NET},
	{PCI_ANY_DEVICE_ID},
};

static struct pci_driver virtio_net_drv = {
	.device_ids = pci_id_map,
	.init = virtio_net_drv_init,
	.add_dev = virtio_net_add_dev
};

PCI_REGISTER_DRIVER(&virtio_net_drv);

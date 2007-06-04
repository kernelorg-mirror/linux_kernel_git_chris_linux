/*
 * Intel IXP4xx Ethernet driver for Linux
 *
 * Copyright (C) 2007 Krzysztof Halasa <khc@pm.waw.pl>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License
 * as published by the Free Software Foundation.
 *
 * Ethernet port config (0x00 is not present on IXP42X):
 *
 * logical port		0x00		0x10		0x20
 * NPE			0 (NPE-A)	1 (NPE-B)	2 (NPE-C)
 * physical PortId	2		0		1
 * TX queue		23		24		25
 * RX-free queue	26		27		28
 * TX-done queue is always 31, RX queue is configurable
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/kernel.h>
#include <linux/mii.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <asm/arch/npe.h>
#include <asm/arch/qmgr.h>

#define DEBUG_QUEUES	0
#define DEBUG_RX	0
#define DEBUG_TX	0
#define DEBUG_PKT_BYTES	0
#define DEBUG_MDIO	0
#define DEBUG_CLOSE	0

#define DRV_NAME	"ixp4xx_eth"

#define TX_QUEUE_LEN	16 /* dwords */
#define PKT_DESCS	64 /* also length of queues: TX-done, RX-ready, RX */

#define POOL_ALLOC_SIZE		(sizeof(struct desc) * (PKT_DESCS))
#define REGS_SIZE		0x1000
#define MAX_MRU			1536 /* 0x600 */

#define MDIO_INTERVAL		(3 * HZ)
#define MAX_MDIO_RETRIES	100 /* microseconds, typically 30 cycles */
#define MAX_CLOSE_WAIT		1000 /* microseconds, typically 2-3 cycles */

#define NPE_ID(port)		((port)->id >> 4)
#define PHYSICAL_ID(port)	((NPE_ID(port) + 2) % 3)
#define TX_QUEUE(plat)		(NPE_ID(port) + 23)
#define RXFREE_QUEUE(plat)	(NPE_ID(port) + 26)
#define TXDONE_QUEUE		31

/* TX Control Registers */
#define TX_CNTRL0_TX_EN		0x01
#define TX_CNTRL0_HALFDUPLEX	0x02
#define TX_CNTRL0_RETRY		0x04
#define TX_CNTRL0_PAD_EN	0x08
#define TX_CNTRL0_APPEND_FCS	0x10
#define TX_CNTRL0_2DEFER	0x20
#define TX_CNTRL0_RMII		0x40 /* reduced MII */
#define TX_CNTRL1_RETRIES	0x0F /* 4 bits */

/* RX Control Registers */
#define RX_CNTRL0_RX_EN		0x01
#define RX_CNTRL0_PADSTRIP_EN	0x02
#define RX_CNTRL0_SEND_FCS	0x04
#define RX_CNTRL0_PAUSE_EN	0x08
#define RX_CNTRL0_LOOP_EN	0x10
#define RX_CNTRL0_ADDR_FLTR_EN	0x20
#define RX_CNTRL0_RX_RUNT_EN	0x40
#define RX_CNTRL0_BCAST_DIS	0x80
#define RX_CNTRL1_DEFER_EN	0x01

/* Core Control Register */
#define CORE_RESET		0x01
#define CORE_RX_FIFO_FLUSH	0x02
#define CORE_TX_FIFO_FLUSH	0x04
#define CORE_SEND_JAM		0x08
#define CORE_MDC_EN		0x10 /* MDIO using NPE-B ETH-0 only */

#define DEFAULT_TX_CNTRL0	(TX_CNTRL0_TX_EN | TX_CNTRL0_RETRY |	\
				 TX_CNTRL0_PAD_EN | TX_CNTRL0_APPEND_FCS | \
				 TX_CNTRL0_2DEFER)
#define DEFAULT_RX_CNTRL0	RX_CNTRL0_RX_EN
#define DEFAULT_CORE_CNTRL	CORE_MDC_EN


/* NPE message codes */
#define NPE_GETSTATUS			0x00
#define NPE_EDB_SETPORTADDRESS		0x01
#define NPE_EDB_GETMACADDRESSDATABASE	0x02
#define NPE_EDB_SETMACADDRESSSDATABASE	0x03
#define NPE_GETSTATS			0x04
#define NPE_RESETSTATS			0x05
#define NPE_SETMAXFRAMELENGTHS		0x06
#define NPE_VLAN_SETRXTAGMODE		0x07
#define NPE_VLAN_SETDEFAULTRXVID	0x08
#define NPE_VLAN_SETPORTVLANTABLEENTRY	0x09
#define NPE_VLAN_SETPORTVLANTABLERANGE	0x0A
#define NPE_VLAN_SETRXQOSENTRY		0x0B
#define NPE_VLAN_SETPORTIDEXTRACTIONMODE 0x0C
#define NPE_STP_SETBLOCKINGSTATE	0x0D
#define NPE_FW_SETFIREWALLMODE		0x0E
#define NPE_PC_SETFRAMECONTROLDURATIONID 0x0F
#define NPE_PC_SETAPMACTABLE		0x11
#define NPE_SETLOOPBACK_MODE		0x12
#define NPE_PC_SETBSSIDTABLE		0x13
#define NPE_ADDRESS_FILTER_CONFIG	0x14
#define NPE_APPENDFCSCONFIG		0x15
#define NPE_NOTIFY_MAC_RECOVERY_DONE	0x16
#define NPE_MAC_RECOVERY_START		0x17


struct eth_regs {
	u32 tx_control[2], __res1[2];		/* 000 */
	u32 rx_control[2], __res2[2];		/* 010 */
	u32 random_seed, __res3[3];		/* 020 */
	u32 partial_empty_threshold, __res4;	/* 030 */
	u32 partial_full_threshold, __res5;	/* 038 */
	u32 tx_start_bytes, __res6[3];		/* 040 */
	u32 tx_deferral, rx_deferral,__res7[2];	/* 050 */
	u32 tx_2part_deferral[2], __res8[2];	/* 060 */
	u32 slot_time, __res9[3];		/* 070 */
	u32 mdio_command[4];			/* 080 */
	u32 mdio_status[4];			/* 090 */
	u32 mcast_mask[6], __res10[2];		/* 0A0 */
	u32 mcast_addr[6], __res11[2];		/* 0C0 */
	u32 int_clock_threshold, __res12[3];	/* 0E0 */
	u32 hw_addr[6], __res13[61];		/* 0F0 */
	u32 core_control;			/* 1FC */
};

struct port {
	struct resource *mem_res;
	struct eth_regs __iomem *regs;
	struct npe *npe;
	struct net_device *netdev;
	struct net_device_stats stat;
	struct mii_if_info mii;
	struct delayed_work mdio_thread;
	struct mac_plat_info *plat;
#ifdef __ARMEB__
	struct sk_buff *rx_buff_tab[PKT_DESCS];
#else
	void *rx_buff_tab[PKT_DESCS];
#endif
	struct desc *rx_desc_tab; /* coherent */
	int id;			/* logical port ID */
	u32 rx_desc_tab_phys;
};

/* NPE message structure */
struct msg {
#ifdef __ARMEB__
	u8 cmd, eth_id, byte2, byte3;
	u8 byte4, byte5, byte6, byte7;
#else
	u8 byte3, byte2, eth_id, cmd;
	u8 byte7, byte6, byte5, byte4;
#endif
};

/* Ethernet packet descriptor */
struct desc {
	u32 next;		/* pointer to next buffer, unused */

#ifdef __ARMEB__
	u16 buf_len;		/* buffer length */
	u16 pkt_len;		/* packet length */
	u32 data;		/* pointer to data buffer in RAM */
	u8 dest_id;
	u8 src_id;
	u16 flags;
	u8 qos;
	u8 padlen;
	u16 vlan_tci;
#else
	u16 pkt_len;		/* packet length */
	u16 buf_len;		/* buffer length */
	u32 data;		/* pointer to data buffer in RAM */
	u16 flags;
	u8 src_id;
	u8 dest_id;
	u16 vlan_tci;
	u8 padlen;
	u8 qos;
#endif

#ifdef __ARMEB__
	u8 dst_mac_0, dst_mac_1, dst_mac_2, dst_mac_3;
	u8 dst_mac_4, dst_mac_5, src_mac_0, src_mac_1;
	u8 src_mac_2, src_mac_3, src_mac_4, src_mac_5;
#else
	u8 dst_mac_3, dst_mac_2, dst_mac_1, dst_mac_0;
	u8 src_mac_1, src_mac_0, dst_mac_5, dst_mac_4;
	u8 src_mac_5, src_mac_4, src_mac_3, src_mac_2;
#endif
};


#define rx_desc_phys(port, n)	((port)->rx_desc_tab_phys +		\
				 (n) * sizeof(struct desc))
#define tx_desc_phys(n)		(tx_desc_tab_phys + (n) * sizeof(struct desc))

#ifndef __ARMEB__
static inline void memcpy_swab32(u32 *dest, u32 *src, int cnt)
{
	int i;
	for (i = 0; i < cnt; i++)
		dest[i] = swab32(src[i]);
}
#endif

static spinlock_t mdio_lock;
static struct eth_regs __iomem *mdio_regs; /* mdio command and status only */
static int ports_open;
static struct dma_pool *dma_pool;
#ifdef __ARMEB__
static struct sk_buff *tx_buff_tab[PKT_DESCS];
#else
static void *tx_buff_tab[PKT_DESCS];
#endif
static struct desc *tx_desc_tab; /* coherent */
static struct device *tx_owner_tab[PKT_DESCS]; /* for dma_unmap_single() */
static u32 tx_desc_tab_phys;


static u16 mdio_cmd(struct net_device *dev, int phy_id, int location,
		    int write, u16 cmd)
{
	int cycles = 0;

	if (__raw_readl(&mdio_regs->mdio_command[3]) & 0x80) {
		printk(KERN_ERR "%s: MII not ready to transmit\n", dev->name);
		return 0;
	}

	if (write) {
		__raw_writel(cmd & 0xFF, &mdio_regs->mdio_command[0]);
		__raw_writel(cmd >> 8, &mdio_regs->mdio_command[1]);
	}
	__raw_writel(((phy_id << 5) | location) & 0xFF,
		     &mdio_regs->mdio_command[2]);
	__raw_writel((phy_id >> 3) | (write << 2) | 0x80 /* GO */,
		     &mdio_regs->mdio_command[3]);

	while ((cycles < MAX_MDIO_RETRIES) &&
	       (__raw_readl(&mdio_regs->mdio_command[3]) & 0x80)) {
		udelay(1);
		cycles++;
	}

	if (cycles == MAX_MDIO_RETRIES) {
		printk(KERN_ERR "%s: MII write failed\n", dev->name);
		return 0;
	}

#if DEBUG_MDIO
	printk(KERN_DEBUG "mdio_cmd() took %i cycles\n", cycles);
#endif

	if (write)
		return 0;

	if (__raw_readl(&mdio_regs->mdio_status[3]) & 0x80) {
		printk(KERN_ERR "%s: MII read failed\n", dev->name);
		return 0;
	}

	return (__raw_readl(&mdio_regs->mdio_status[0]) & 0xFF) |
		(__raw_readl(&mdio_regs->mdio_status[1]) << 8);
}

static int mdio_read(struct net_device *dev, int phy_id, int location)
{
	unsigned long flags;
	u16 val;

	spin_lock_irqsave(&mdio_lock, flags);
	val = mdio_cmd(dev, phy_id, location, 0, 0);
	spin_unlock_irqrestore(&mdio_lock, flags);
	return val;
}

static void mdio_write(struct net_device *dev, int phy_id, int location,
		       int val)
{
	unsigned long flags;

	spin_lock_irqsave(&mdio_lock, flags);
	mdio_cmd(dev, phy_id, location, 1, val);
	spin_unlock_irqrestore(&mdio_lock, flags);
}

static void eth_set_duplex(struct port *port)
{
	if (port->mii.full_duplex)
		__raw_writel(DEFAULT_TX_CNTRL0 & ~TX_CNTRL0_HALFDUPLEX,
			     &port->regs->tx_control[0]);
	else
		__raw_writel(DEFAULT_TX_CNTRL0 | TX_CNTRL0_HALFDUPLEX,
			     &port->regs->tx_control[0]);
}


static void mdio_thread(struct work_struct *work)
{
	struct port *port = container_of(work, struct port, mdio_thread.work);

	if (mii_check_media(&port->mii, 1, 0))
		eth_set_duplex(port);
	schedule_delayed_work(&port->mdio_thread, MDIO_INTERVAL);
}


static inline void debug_pkt(const char *func, u8 *data, int len)
{
#if DEBUG_PKT_BYTES
	int i;

	printk(KERN_DEBUG "%s(%i): ", func, len);
	for (i = 0; i < len; i++) {
		if (i >= DEBUG_PKT_BYTES)
			break;
		printk("%s%02X",
		       ((i == 6) || (i == 12) || (i >= 14)) ? " " : "",
		       data[i]);
	}
	printk("\n");
#endif
}


static inline void debug_desc(unsigned int queue, u32 desc_phys,
			      struct desc *desc, int is_get)
{
#if DEBUG_QUEUES
	const char *op = is_get ? "->" : "<-";

	if (!desc_phys) {
		printk(KERN_DEBUG "queue %2i %s NULL\n", queue, op);
		return;
	}
	printk(KERN_DEBUG "queue %2i %s %X: %X %3X %3X %08X %2X < %2X %4X %X"
	       " %X %X %02X%02X%02X%02X%02X%02X < %02X%02X%02X%02X%02X%02X\n",
	       queue, op, desc_phys, desc->next, desc->buf_len, desc->pkt_len,
	       desc->data, desc->dest_id, desc->src_id, desc->flags,
	       desc->qos, desc->padlen, desc->vlan_tci,
	       desc->dst_mac_0, desc->dst_mac_1, desc->dst_mac_2,
	       desc->dst_mac_3, desc->dst_mac_4, desc->dst_mac_5,
	       desc->src_mac_0, desc->src_mac_1, desc->src_mac_2,
	       desc->src_mac_3, desc->src_mac_4, desc->src_mac_5);
#endif
}

static inline int queue_get_desc(unsigned int queue, struct port *port,
				 int is_tx)
{
	u32 phys, tab_phys, n_desc;
	struct desc *tab;

	if (!(phys = qmgr_get_entry(queue))) {
		debug_desc(queue, phys, NULL, 1);
		return -1;
	}

	phys &= ~0x1F; /* mask out non-address bits */
	tab_phys = is_tx ? tx_desc_phys(0) : rx_desc_phys(port, 0);
	tab = is_tx ? tx_desc_tab : port->rx_desc_tab;
	n_desc = (phys - tab_phys) / sizeof(struct desc);
	BUG_ON(n_desc >= PKT_DESCS);

	debug_desc(queue, phys, &tab[n_desc], 1);
	BUG_ON(tab[n_desc].next);
	return n_desc;
}

static inline void queue_put_desc(unsigned int queue, u32 desc_phys,
				  struct desc *desc)
{
	debug_desc(queue, desc_phys, desc, 0);
	BUG_ON(desc_phys & 0x1F);
	qmgr_put_entry(queue, desc_phys);
}


static void eth_rx_irq(void *pdev)
{
	struct net_device *dev = pdev;
	struct port *port = netdev_priv(dev);

#if DEBUG_RX
	printk(KERN_DEBUG "eth_rx_irq() start\n");
#endif
	qmgr_disable_irq(port->plat->rxq);
	netif_rx_schedule(dev);
}

static int eth_poll(struct net_device *dev, int *budget)
{
	struct port *port = netdev_priv(dev);
	unsigned int rxq = port->plat->rxq, rxfreeq = RXFREE_QUEUE(port->plat);
	int quota = dev->quota, received = 0;

#if DEBUG_RX
	printk(KERN_DEBUG "eth_poll() start\n");
#endif

	while (quota) {
		struct sk_buff *skb;
		struct desc *desc;
		int n;
#ifdef __ARMEB__
		struct sk_buff *temp;
		u32 phys;
#endif

		if ((n = queue_get_desc(rxq, port, 0)) < 0) {
			dev->quota -= received;	/* No packet received */
			*budget -= received;
			received = 0;
			netif_rx_complete(dev);
			qmgr_enable_irq(rxq);
			if (!qmgr_stat_empty(rxq) &&
			    netif_rx_reschedule(dev, 0)) {
				qmgr_disable_irq(rxq);
				continue;
			}
			return 0; /* all work done */
		}

		desc = &port->rx_desc_tab[n];

#ifdef __ARMEB__
		if ((skb = netdev_alloc_skb(dev, MAX_MRU)) != NULL) {
			phys = dma_map_single(&dev->dev, skb->data,
					      MAX_MRU, DMA_FROM_DEVICE);
			if (dma_mapping_error(phys)) {
				dev_kfree_skb(skb);
				skb = NULL;
			}
		}
#else
		skb = netdev_alloc_skb(dev, desc->pkt_len);
#endif

		if (!skb) {
			port->stat.rx_dropped++;
			/* put the desc back on RX-ready queue */
			desc->buf_len = MAX_MRU;
			desc->pkt_len = 0;
			queue_put_desc(rxfreeq, rx_desc_phys(port, n), desc);
			BUG_ON(qmgr_stat_overflow(rxfreeq));
			continue;
		}

		/* process received frame */
#ifdef __ARMEB__
		temp = skb;
		skb = port->rx_buff_tab[n];
		dma_unmap_single(&dev->dev, desc->data,
				 MAX_MRU, DMA_FROM_DEVICE);
#else
		dma_sync_single(&dev->dev, desc->data,
				MAX_MRU, DMA_FROM_DEVICE);
		memcpy_swab32((u32 *)skb->data, (u32 *)port->rx_buff_tab[n],
			      ALIGN(desc->pkt_len, 4) / 4);
#endif
		skb_put(skb, desc->pkt_len);

		debug_pkt("eth_poll", skb->data, skb->len);

		skb->protocol = eth_type_trans(skb, dev);
		dev->last_rx = jiffies;
		port->stat.rx_packets++;
		port->stat.rx_bytes += skb->len;
		netif_receive_skb(skb);

		/* put the new buffer on RX-free queue */
#ifdef __ARMEB__
		port->rx_buff_tab[n] = temp;
		desc->data = phys;
#endif
		desc->buf_len = MAX_MRU;
		desc->pkt_len = 0;
		queue_put_desc(rxfreeq, rx_desc_phys(port, n), desc);
		BUG_ON(qmgr_stat_overflow(rxfreeq));
		quota--;
		received++;
	}
	dev->quota -= received;
	*budget -= received;
	return 1;		/* not all work done */
}

static void eth_xmit_ready_irq(void *pdev)
{
#if DEBUG_TX
	printk(KERN_DEBUG "eth_xmit_ready_irq()\n");
#endif
	netif_start_queue((struct net_device *)pdev);
}

static int eth_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct port *port = netdev_priv(dev);
	struct desc *desc;
	void *buff;
	int n;

#if DEBUG_TX
	printk(KERN_DEBUG "eth_xmit() start\n");
#endif
	if (unlikely(skb->len > MAX_MRU)) {
		dev_kfree_skb(skb);
		port->stat.tx_errors++;
		return NETDEV_TX_OK;
	}

	n = queue_get_desc(TXDONE_QUEUE, port, 1);
	BUG_ON(n < 0);
	desc = &tx_desc_tab[n];

	if ((buff = tx_buff_tab[n]) != NULL) {
		dma_unmap_single(tx_owner_tab[n], desc->data,
				 desc->buf_len, DMA_TO_DEVICE);
#ifdef __ARMEB__
		dev_kfree_skb(buff);
#else
		kfree(buff);
#endif
	}

	/* disable VLAN functions in NPE image for now */
	memset(desc, 0, sizeof(*desc));
	desc->buf_len = desc->pkt_len = skb->len;
#ifdef __ARMEB__
	buff = skb;
	desc->data = dma_map_single(&dev->dev, skb->data,
				    skb->len, DMA_TO_DEVICE);
#else
	if ((buff = kmalloc(ALIGN(skb->len, 4), GFP_ATOMIC)) != NULL) {
		/* buff must be dword - aligned */
		memcpy_swab32(buff, (u32 *)skb->data, ALIGN(skb->len, 4) / 4);
		desc->data = dma_map_single(&dev->dev, buff,
					    ALIGN(skb->len, 4), DMA_TO_DEVICE);
	}
	dev_kfree_skb(skb);
#endif

	if (!buff || dma_mapping_error(desc->data)) {
#ifdef __ARMEB__
		dev_kfree_skb(buff);
#else
		kfree(buff);
#endif
		desc->data = 0;
		tx_buff_tab[n] = NULL;
		port->stat.tx_dropped++;
		/* put the desc back on TX-done queue */
		queue_put_desc(TXDONE_QUEUE, tx_desc_phys(n), desc);
		return NETDEV_TX_OK;
	}

	tx_buff_tab[n] = buff;
	tx_owner_tab[n] = &dev->dev;

#ifdef __ARMEB__
	debug_pkt("eth_xmit", skb->data, desc->pkt_len);
#else
	debug_pkt("eth_xmit", buff, desc->pkt_len);
#endif
	/* NPE firmware pads short frames with zeros internally */
	wmb();
	queue_put_desc(TX_QUEUE(port->plat), tx_desc_phys(n), desc);
	BUG_ON(qmgr_stat_overflow(TX_QUEUE(port->plat)));
	dev->trans_start = jiffies;
	port->stat.tx_packets++;
	port->stat.tx_bytes += desc->pkt_len;

	if (qmgr_stat_full(TX_QUEUE(port->plat))) {
		netif_stop_queue(dev);
		/* we could miss TX ready interrupt */
		if (!qmgr_stat_full(TX_QUEUE(port->plat)))
			netif_start_queue(dev);
	}

#if DEBUG_TX
	printk(KERN_DEBUG "eth_xmit() end\n");
#endif
	return NETDEV_TX_OK;
}


static struct net_device_stats *eth_stats(struct net_device *dev)
{
	struct port *port = netdev_priv(dev);
	return &port->stat;
}

static void eth_set_mcast_list(struct net_device *dev)
{
	struct port *port = netdev_priv(dev);
	struct dev_mc_list *mclist = dev->mc_list;
	u8 diffs[ETH_ALEN], *addr;
	int cnt = dev->mc_count, i;

	if ((dev->flags & IFF_PROMISC) || !mclist || !cnt) {
		__raw_writel(DEFAULT_RX_CNTRL0 & ~RX_CNTRL0_ADDR_FLTR_EN,
			     &port->regs->rx_control[0]);
		return;
	}

	memset(diffs, 0, ETH_ALEN);
	addr = mclist->dmi_addr; /* first MAC address */

	while (--cnt && (mclist = mclist->next))
		for (i = 0; i < ETH_ALEN; i++)
			diffs[i] |= addr[i] ^ mclist->dmi_addr[i];

	for (i = 0; i < ETH_ALEN; i++) {
		__raw_writel(addr[i], &port->regs->mcast_addr[i]);
		__raw_writel(~diffs[i], &port->regs->mcast_mask[i]);
	}

	__raw_writel(DEFAULT_RX_CNTRL0 | RX_CNTRL0_ADDR_FLTR_EN,
		     &port->regs->rx_control[0]);
}


static int eth_ioctl(struct net_device *dev, struct ifreq *req, int cmd)
{
	struct port *port = netdev_priv(dev);
	unsigned int duplex_chg;
	int err;

	if (!netif_running(dev))
		return -EINVAL;
	err = generic_mii_ioctl(&port->mii, if_mii(req), cmd, &duplex_chg);
	if (duplex_chg)
		eth_set_duplex(port);
	return err;
}


static int request_queues(struct port *port)
{
	int err;

	err = qmgr_request_queue(RXFREE_QUEUE(port->plat), PKT_DESCS, 0, 0);
	if (err)
		return err;

	err = qmgr_request_queue(port->plat->rxq, PKT_DESCS, 0, 0);
	if (err)
		goto rel_rxfree;

	err = qmgr_request_queue(TX_QUEUE(port->plat), TX_QUEUE_LEN, 0, 0);
	if (err)
		goto rel_rx;

	/* TX-done queue handles skbs sent out by the NPEs */
	if (!ports_open) {
		err = qmgr_request_queue(TXDONE_QUEUE, PKT_DESCS, 0, 0);
		if (err)
			goto rel_tx;
	}
	return 0;

rel_tx:
	qmgr_release_queue(TX_QUEUE(port->plat));
rel_rx:
	qmgr_release_queue(port->plat->rxq);
rel_rxfree:
	qmgr_release_queue(RXFREE_QUEUE(port->plat));
	return err;
}

static void release_queues(struct port *port)
{
	qmgr_release_queue(RXFREE_QUEUE(port->plat));
	qmgr_release_queue(port->plat->rxq);
	qmgr_release_queue(TX_QUEUE(port->plat));

	if (!ports_open)
		qmgr_release_queue(TXDONE_QUEUE);
}

static int init_queues(struct port *port)
{
	int i;

	if (!ports_open) {
		/* Setup TX descriptors - common to all ports */
		if (!(dma_pool = dma_pool_create(DRV_NAME, NULL,
						 POOL_ALLOC_SIZE, 32, 0)))
			return -ENOMEM;

		if (!(tx_desc_tab = dma_pool_alloc(dma_pool, GFP_KERNEL,
						   &tx_desc_tab_phys)))
			return -ENOMEM;
		memset(tx_desc_tab, 0, POOL_ALLOC_SIZE);
		memset(tx_buff_tab, 0, sizeof(tx_buff_tab)); /* static table */
	}

	/* Setup RX buffers */
	if (!(port->rx_desc_tab = dma_pool_alloc(dma_pool, GFP_KERNEL,
						 &port->rx_desc_tab_phys)))
		return -ENOMEM;
	memset(port->rx_desc_tab, 0, POOL_ALLOC_SIZE);
	memset(port->rx_buff_tab, 0, sizeof(port->rx_buff_tab)); /* table */

	for (i = 0; i < PKT_DESCS; i++) {
		struct desc *desc = &port->rx_desc_tab[i];
		void *data;
#ifdef __ARMEB__
		struct sk_buff *skb;

		if (!(skb = netdev_alloc_skb(port->netdev, MAX_MRU)))
			return -ENOMEM;
		port->rx_buff_tab[i] = skb;
		data = skb->data;
#else
		if (!(data = kmalloc(MAX_MRU, GFP_KERNEL)))
			return -ENOMEM;
		port->rx_buff_tab[i] = data;
#endif
		desc->buf_len = MAX_MRU;
		desc->data = dma_map_single(&port->netdev->dev, data,
					    MAX_MRU, DMA_FROM_DEVICE);
		if (dma_mapping_error(desc->data)) {
			desc->data = 0;
			return -EIO;
		}
	}
	return 0;
}

static void destroy_queues(struct port *port)
{
	int i;

	if (port->rx_desc_tab) {
		for (i = 0; i < PKT_DESCS; i++) {
			struct desc *desc = &port->rx_desc_tab[i];
			void *buff = port->rx_buff_tab[i]; /* may be skb */
			if (buff) {
				if (desc->data)
					dma_unmap_single(&port->netdev->dev,
							 desc->data, MAX_MRU,
							 DMA_FROM_DEVICE);
#ifdef __ARMEB__
				dev_kfree_skb(buff);
#else
				kfree(buff);
#endif
			}
		}
		dma_pool_free(dma_pool, port->rx_desc_tab,
			      port->rx_desc_tab_phys);
		port->rx_desc_tab = NULL;
	}

	if (!ports_open && tx_desc_tab) {
		for (i = 0; i < PKT_DESCS; i++) {
			struct desc *desc = &tx_desc_tab[i];
			void *buff = tx_buff_tab[i]; /* may be skb */
			if (buff) {
				if (desc->data)
					dma_unmap_single(&port->netdev->dev,
							 desc->data,
							 desc->buf_len,
							 DMA_TO_DEVICE);
#ifdef __ARMEB__
				dev_kfree_skb(buff);
#else
				kfree(buff);
#endif
			}
		}
		dma_pool_free(dma_pool, tx_desc_tab, tx_desc_tab_phys);
		tx_desc_tab = NULL;
	}
	if (!ports_open && dma_pool) {
		dma_pool_destroy(dma_pool);
		dma_pool = NULL;
	}
}

static int eth_open(struct net_device *dev)
{
	struct port *port = netdev_priv(dev);
	struct npe *npe = port->npe;
	struct msg msg;
	int i, err;

	if (!npe_running(npe)) {
		err = npe_load_firmware(npe, npe_name(npe), &dev->dev);
		if (err)
			return err;

		if (npe_recv_message(npe, &msg, "ETH_GET_STATUS")) {
			printk(KERN_ERR "%s: %s not responding\n", dev->name,
			       npe_name(npe));
			return -EIO;
		}
	}

	memset(&msg, 0, sizeof(msg));
	msg.cmd = NPE_VLAN_SETRXQOSENTRY;
	msg.eth_id = port->id;
	msg.byte5 = port->plat->rxq | 0x80;
	msg.byte7 = port->plat->rxq << 4;
	for (i = 0; i < 8; i++) {
		msg.byte3 = i;
		if (npe_send_recv_message(port->npe, &msg, "ETH_SET_RXQ"))
			return -EIO;
	}

	msg.cmd = NPE_EDB_SETPORTADDRESS;
	msg.eth_id = PHYSICAL_ID(port);
	msg.byte2 = dev->dev_addr[0];
	msg.byte3 = dev->dev_addr[1];
	msg.byte4 = dev->dev_addr[2];
	msg.byte5 = dev->dev_addr[3];
	msg.byte6 = dev->dev_addr[4];
	msg.byte7 = dev->dev_addr[5];
	if (npe_send_recv_message(port->npe, &msg, "ETH_SET_MAC"))
		return -EIO;

	memset(&msg, 0, sizeof(msg));
	msg.cmd = NPE_FW_SETFIREWALLMODE;
	msg.eth_id = port->id;
	if (npe_send_recv_message(port->npe, &msg, "ETH_SET_FIREWALL_MODE"))
		return -EIO;

	if ((err = request_queues(port)) != 0)
		return err;

	if ((err = init_queues(port)) != 0) {
		destroy_queues(port);
		release_queues(port);
		return err;
	}

	for (i = 0; i < ETH_ALEN; i++)
		__raw_writel(dev->dev_addr[i], &port->regs->hw_addr[i]);
	__raw_writel(0x08, &port->regs->random_seed);
	__raw_writel(0x12, &port->regs->partial_empty_threshold);
	__raw_writel(0x30, &port->regs->partial_full_threshold);
	__raw_writel(0x08, &port->regs->tx_start_bytes);
	__raw_writel(0x15, &port->regs->tx_deferral);
	__raw_writel(0x08, &port->regs->tx_2part_deferral[0]);
	__raw_writel(0x07, &port->regs->tx_2part_deferral[1]);
	__raw_writel(0x80, &port->regs->slot_time);
	__raw_writel(0x01, &port->regs->int_clock_threshold);

	/* Populate queues with buffers, no failure after this point */
	if (!ports_open)
		for (i = 0; i < PKT_DESCS; i++) {
			queue_put_desc(TXDONE_QUEUE, tx_desc_phys(i),
				       &tx_desc_tab[i]);
			BUG_ON(qmgr_stat_overflow(TXDONE_QUEUE));
		}

	for (i = 0; i < PKT_DESCS; i++) {
		queue_put_desc(RXFREE_QUEUE(port->plat),
			       rx_desc_phys(port, i), &port->rx_desc_tab[i]);
		BUG_ON(qmgr_stat_overflow(RXFREE_QUEUE(port->plat)));
	}

	__raw_writel(TX_CNTRL1_RETRIES, &port->regs->tx_control[1]);
	__raw_writel(DEFAULT_TX_CNTRL0, &port->regs->tx_control[0]);
	__raw_writel(0, &port->regs->rx_control[1]);
	__raw_writel(DEFAULT_RX_CNTRL0, &port->regs->rx_control[0]);

	if (mii_check_media(&port->mii, 1, 1))
		eth_set_duplex(port);
	eth_set_mcast_list(dev);
	netif_start_queue(dev);
	schedule_delayed_work(&port->mdio_thread, MDIO_INTERVAL);

	qmgr_set_irq(port->plat->rxq, QUEUE_IRQ_SRC_NOT_EMPTY,
		     eth_rx_irq, dev);
	qmgr_set_irq(TX_QUEUE(port->plat), QUEUE_IRQ_SRC_NOT_FULL,
		     eth_xmit_ready_irq, dev);
	qmgr_enable_irq(TX_QUEUE(port->plat));
	ports_open++;
	netif_rx_schedule(dev);
	return 0;
}

static int eth_close(struct net_device *dev)
{
	struct port *port = netdev_priv(dev);
	struct msg msg;
	int buffs = PKT_DESCS; /* allocated RX buffers */
	int i;

	ports_open--;
	qmgr_disable_irq(port->plat->rxq);
	qmgr_disable_irq(TX_QUEUE(port->plat));
	netif_stop_queue(dev);

	while (queue_get_desc(RXFREE_QUEUE(port->plat), port, 0) >= 0)
		buffs--;

	memset(&msg, 0, sizeof(msg));
	msg.cmd = NPE_SETLOOPBACK_MODE;
	msg.eth_id = port->id;
	msg.byte3 = 1;
	if (npe_send_recv_message(port->npe, &msg, "ETH_ENABLE_LOOPBACK"))
		printk(KERN_CRIT "%s: unable to enable loopback\n", dev->name);

	i = 0;
	do {			/* drain RX buffers */
		while (queue_get_desc(port->plat->rxq, port, 0) >= 0)
			buffs--;
		if (!buffs)
			break;
		if (qmgr_stat_empty(TX_QUEUE(port->plat))) {
			/* we have to inject some packet */
			int n = queue_get_desc(TXDONE_QUEUE, port, 1);
			struct desc *desc;
			u32 phys;

			BUG_ON(n < 0);
			desc = &tx_desc_tab[n];
			phys = tx_desc_phys(n);
			desc->buf_len = desc->pkt_len = 1;
			wmb();
			queue_put_desc(TX_QUEUE(port->plat), phys, desc);
			BUG_ON(qmgr_stat_overflow(TX_QUEUE(port->plat)));
		}
		udelay(1);
	} while (++i < MAX_CLOSE_WAIT);

	if (buffs)
		printk(KERN_CRIT "%s: unable to drain RX queue, %i buffer(s)"
		       " left in NPE\n", dev->name, buffs);
#if DEBUG_CLOSE
	if (!buffs)
		printk(KERN_DEBUG "Draining RX queue took %i cycles\n", i);
#endif

	msg.byte3 = 0;
	if (npe_send_recv_message(port->npe, &msg, "ETH_DISABLE_LOOPBACK"))
		printk(KERN_CRIT "%s: unable to disable loopback\n",
		       dev->name);

	if (ports_open) {
		while ((i = queue_get_desc(TX_QUEUE(port->plat),
					   port, 1)) >= 0) {
			queue_put_desc(TXDONE_QUEUE, tx_desc_phys(i),
				       &tx_desc_tab[i]);
			BUG_ON(qmgr_stat_overflow(TXDONE_QUEUE));
		}
	} else {
		buffs = PKT_DESCS;
		i = 0;
		while (queue_get_desc(TX_QUEUE(port->plat), port, 1) >= 0)
			buffs--; /* cancel TX */
		do {
			while (queue_get_desc(TXDONE_QUEUE, port, 1) >= 0)
				buffs--;
			if (!buffs)
				break;
		} while (++i < MAX_CLOSE_WAIT);
		
		if (buffs)
			printk(KERN_CRIT "%s: unable to drain TX queue, %i"
			       " buffer(s) left in NPE\n", dev->name, buffs);
#if DEBUG_CLOSE
		if (!buffs)
			printk(KERN_DEBUG "Draining TX queues took %i "
			       "cycles\n", i);
#endif
	}

	cancel_rearming_delayed_work(&port->mdio_thread);
	destroy_queues(port);
	release_queues(port);
	return 0;
}

static int __devinit eth_init_one(struct platform_device *pdev)
{
	struct port *port;
	struct net_device *dev;
	struct mac_plat_info *plat = pdev->dev.platform_data;
	u32 regs_phys;
	int err;

	if (!(dev = alloc_etherdev(sizeof(struct port))))
		return -ENOMEM;

	SET_MODULE_OWNER(dev);
	SET_NETDEV_DEV(dev, &pdev->dev);
	port = netdev_priv(dev);
	port->netdev = dev;
	port->id = pdev->id;

	switch (port->id) {
	case IXP4XX_ETH_NPEA:
		port->regs = (struct eth_regs __iomem *)IXP4XX_EthA_BASE_VIRT;
		regs_phys  = IXP4XX_EthA_BASE_PHYS;
		break;
	case IXP4XX_ETH_NPEB:
		port->regs = (struct eth_regs __iomem *)IXP4XX_EthB_BASE_VIRT;
		regs_phys  = IXP4XX_EthB_BASE_PHYS;
		break;
	case IXP4XX_ETH_NPEC:
		port->regs = (struct eth_regs __iomem *)IXP4XX_EthC_BASE_VIRT;
		regs_phys  = IXP4XX_EthC_BASE_PHYS;
		break;
	default:
		err = -ENOSYS;
		goto err_free;
	}

	dev->open = eth_open;
	dev->hard_start_xmit = eth_xmit;
	dev->poll = eth_poll;
	dev->stop = eth_close;
	dev->get_stats = eth_stats;
	dev->do_ioctl = eth_ioctl;
	dev->set_multicast_list = eth_set_mcast_list;
	dev->weight = 16;
	dev->tx_queue_len = 100;

	if (!(port->npe = npe_request(NPE_ID(port)))) {
		err = -EIO;
		goto err_free;
	}

	if (register_netdev(dev)) {
		err = -EIO;
		goto err_npe_rel;
	}

	port->mem_res = request_mem_region(regs_phys, REGS_SIZE, dev->name);
	if (!port->mem_res) {
		err = -EBUSY;
		goto err_unreg;
	}

	port->plat = plat;
	memcpy(dev->dev_addr, plat->hwaddr, ETH_ALEN);

	platform_set_drvdata(pdev, dev);

	__raw_writel(DEFAULT_CORE_CNTRL | CORE_RESET,
		     &port->regs->core_control);
	udelay(50);
	__raw_writel(DEFAULT_CORE_CNTRL, &port->regs->core_control);
	udelay(50);

	port->mii.dev = dev;
	port->mii.mdio_read = mdio_read;
	port->mii.mdio_write = mdio_write;
	port->mii.phy_id = plat->phy;
	port->mii.phy_id_mask = 0x1F;
	port->mii.reg_num_mask = 0x1F;

	INIT_DELAYED_WORK(&port->mdio_thread, mdio_thread);

	printk(KERN_INFO "%s: MII PHY %i on %s\n", dev->name, plat->phy,
	       npe_name(port->npe));
	return 0;

err_unreg:
	unregister_netdev(dev);
err_npe_rel:
	npe_release(port->npe);
err_free:
	free_netdev(dev);
	return err;
}

static int __devexit eth_remove_one(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct port *port = netdev_priv(dev);

	unregister_netdev(dev);
	platform_set_drvdata(pdev, NULL);
	npe_release(port->npe);
	release_resource(port->mem_res);
	free_netdev(dev);
	return 0;
}

static struct platform_driver drv = {
	.driver.name	= DRV_NAME,
	.probe		= eth_init_one,
	.remove		= eth_remove_one,
};

static int __init eth_init_module(void)
{
	if (!(ixp4xx_read_fuses() & IXP4XX_FUSE_NPEB_ETH0))
		return -ENOSYS;

	/* All MII PHY accesses use NPE-B Ethernet registers */
	spin_lock_init(&mdio_lock);
	mdio_regs = (struct eth_regs __iomem *)IXP4XX_EthB_BASE_VIRT;
	__raw_writel(DEFAULT_CORE_CNTRL, &mdio_regs->core_control);

	return platform_driver_register(&drv);
}

static void __exit eth_cleanup_module(void)
{
	platform_driver_unregister(&drv);
}

MODULE_AUTHOR("Krzysztof Halasa");
MODULE_DESCRIPTION("Intel IXP4xx Ethernet driver");
MODULE_LICENSE("GPL v2");
module_init(eth_init_module);
module_exit(eth_cleanup_module);

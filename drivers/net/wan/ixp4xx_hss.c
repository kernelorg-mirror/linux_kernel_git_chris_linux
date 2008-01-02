/*
 * Intel IXP4xx HSS (synchronous serial port) driver for Linux
 *
 * Copyright (C) 2007 Krzysztof Halasa <khc@pm.waw.pl>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License
 * as published by the Free Software Foundation.
 */

#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/hdlc.h>
#include <linux/platform_device.h>
#include <asm/arch/npe.h>
#include <asm/arch/qmgr.h>

#define DEBUG_QUEUES		0
#define DEBUG_DESC		0
#define DEBUG_RX		0
#define DEBUG_TX		0
#define DEBUG_PKT_BYTES		0
#define DEBUG_CLOSE		0

#define DRV_NAME		"ixp4xx_hss"

#define PKT_EXTRA_FLAGS		0 /* orig 1 */
#define FRAME_SYNC_OFFSET	0 /* unused, channelized only */
#define FRAME_SYNC_SIZE		1024
#define PKT_NUM_PIPES		1 /* 1, 2 or 4 */
#define PKT_PIPE_FIFO_SIZEW	4 /* total 4 dwords per HSS */

#define RX_DESCS		16 /* also length of all RX queues */
#define TX_DESCS		16 /* also length of all TX queues */

#define POOL_ALLOC_SIZE		(sizeof(struct desc) * (RX_DESCS + TX_DESCS))
#define RX_SIZE			(HDLC_MAX_MRU + 4) /* NPE needs more space */
#define MAX_CLOSE_WAIT		1000 /* microseconds */

/* Queue IDs */
#define HSS0_CHL_RXTRIG_QUEUE	12	/* orig size = 32 dwords */
#define HSS0_PKT_RX_QUEUE	13	/* orig size = 32 dwords */
#define HSS0_PKT_TX0_QUEUE	14	/* orig size = 16 dwords */
#define HSS0_PKT_TX1_QUEUE	15
#define HSS0_PKT_TX2_QUEUE	16
#define HSS0_PKT_TX3_QUEUE	17
#define HSS0_PKT_RXFREE0_QUEUE	18	/* orig size = 16 dwords */
#define HSS0_PKT_RXFREE1_QUEUE	19
#define HSS0_PKT_RXFREE2_QUEUE	20
#define HSS0_PKT_RXFREE3_QUEUE	21
#define HSS0_PKT_TXDONE_QUEUE	22	/* orig size = 64 dwords */

#define HSS1_CHL_RXTRIG_QUEUE	10
#define HSS1_PKT_RX_QUEUE	0
#define HSS1_PKT_TX0_QUEUE	5
#define HSS1_PKT_TX1_QUEUE	6
#define HSS1_PKT_TX2_QUEUE	7
#define HSS1_PKT_TX3_QUEUE	8
#define HSS1_PKT_RXFREE0_QUEUE	1
#define HSS1_PKT_RXFREE1_QUEUE	2
#define HSS1_PKT_RXFREE2_QUEUE	3
#define HSS1_PKT_RXFREE3_QUEUE	4
#define HSS1_PKT_TXDONE_QUEUE	9

#define NPE_PKT_MODE_HDLC		0
#define NPE_PKT_MODE_RAW		1
#define NPE_PKT_MODE_56KMODE		2
#define NPE_PKT_MODE_56KENDIAN_MSB	4

/* PKT_PIPE_HDLC_CFG_WRITE flags */
#define PKT_HDLC_IDLE_ONES		0x1 /* default = flags */
#define PKT_HDLC_CRC_32			0x2 /* default = CRC-16 */
#define PKT_HDLC_MSB_ENDIAN		0x4 /* default = LE */


/* hss_config, PCRs */
/* Frame sync sampling, default = active low */
#define PCR_FRM_SYNC_ACTIVE_HIGH	0x40000000
#define PCR_FRM_SYNC_FALLINGEDGE	0x80000000
#define PCR_FRM_SYNC_RISINGEDGE		0xC0000000

/* Frame sync pin: input (default) or output generated off a given clk edge */
#define PCR_FRM_SYNC_OUTPUT_FALLING	0x20000000
#define PCR_FRM_SYNC_OUTPUT_RISING	0x30000000

/* Frame and data clock sampling on edge, default = falling */
#define PCR_FCLK_EDGE_RISING		0x08000000
#define PCR_DCLK_EDGE_RISING		0x04000000

/* Clock direction, default = input */
#define PCR_SYNC_CLK_DIR_OUTPUT		0x02000000

/* Generate/Receive frame pulses, default = enabled */
#define PCR_FRM_PULSE_DISABLED		0x01000000

 /* Data rate is full (default) or half the configured clk speed */
#define PCR_HALF_CLK_RATE		0x00200000

/* Invert data between NPE and HSS FIFOs? (default = no) */
#define PCR_DATA_POLARITY_INVERT	0x00100000

/* TX/RX endianness, default = LSB */
#define PCR_MSB_ENDIAN			0x00080000

/* Normal (default) / open drain mode (TX only) */
#define PCR_TX_PINS_OPEN_DRAIN		0x00040000

/* No framing bit transmitted and expected on RX? (default = framing bit) */
#define PCR_SOF_NO_FBIT			0x00020000

/* Drive data pins? */
#define PCR_TX_DATA_ENABLE		0x00010000

/* Voice 56k type: drive the data pins low (default), high, high Z */
#define PCR_TX_V56K_HIGH		0x00002000
#define PCR_TX_V56K_HIGH_IMP		0x00004000

/* Unassigned type: drive the data pins low (default), high, high Z */
#define PCR_TX_UNASS_HIGH		0x00000800
#define PCR_TX_UNASS_HIGH_IMP		0x00001000

/* T1 @ 1.544MHz only: Fbit dictated in FIFO (default) or high Z */
#define PCR_TX_FB_HIGH_IMP		0x00000400

/* 56k data endiannes - which bit unused: high (default) or low */
#define PCR_TX_56KE_BIT_0_UNUSED	0x00000200

/* 56k data transmission type: 32/8 bit data (default) or 56K data */
#define PCR_TX_56KS_56K_DATA		0x00000100

/* hss_config, cCR */
/* Number of packetized clients, default = 1 */
#define CCR_NPE_HFIFO_2_HDLC		0x04000000
#define CCR_NPE_HFIFO_3_OR_4HDLC	0x08000000

/* default = no loopback */
#define CCR_LOOPBACK			0x02000000

/* HSS number, default = 0 (first) */
#define CCR_SECOND_HSS			0x01000000


/* hss_config, clkCR: main:10, num:10, denom:12 */
#define CLK42X_SPEED_EXP	((0x3FF << 22) | (  2 << 12) |   15) /*65 KHz*/

#define CLK42X_SPEED_512KHZ	((  130 << 22) | (  2 << 12) |   15)
#define CLK42X_SPEED_1536KHZ	((   43 << 22) | ( 18 << 12) |   47)
#define CLK42X_SPEED_1544KHZ	((   43 << 22) | ( 33 << 12) |  192)
#define CLK42X_SPEED_2048KHZ	((   32 << 22) | ( 34 << 12) |   63)
#define CLK42X_SPEED_4096KHZ	((   16 << 22) | ( 34 << 12) |  127)
#define CLK42X_SPEED_8192KHZ	((    8 << 22) | ( 34 << 12) |  255)

#define CLK46X_SPEED_512KHZ	((  130 << 22) | ( 24 << 12) |  127)
#define CLK46X_SPEED_1536KHZ	((   43 << 22) | (152 << 12) |  383)
#define CLK46X_SPEED_1544KHZ	((   43 << 22) | ( 66 << 12) |  385)
#define CLK46X_SPEED_2048KHZ	((   32 << 22) | (280 << 12) |  511)
#define CLK46X_SPEED_4096KHZ	((   16 << 22) | (280 << 12) | 1023)
#define CLK46X_SPEED_8192KHZ	((    8 << 22) | (280 << 12) | 2047)


/* hss_config, LUT entries */
#define TDMMAP_UNASSIGNED	0
#define TDMMAP_HDLC		1	/* HDLC - packetized */
#define TDMMAP_VOICE56K		2	/* Voice56K - 7-bit channelized */
#define TDMMAP_VOICE64K		3	/* Voice64K - 8-bit channelized */

#define TIMESLOTS		128
#define LUT_BITS		2

/* offsets into HSS config */
#define HSS_CONFIG_TX_PCR	0x00
#define HSS_CONFIG_RX_PCR	0x04
#define HSS_CONFIG_CORE_CR	0x08
#define HSS_CONFIG_CLOCK_CR	0x0C
#define HSS_CONFIG_TX_FCR	0x10
#define HSS_CONFIG_RX_FCR	0x14
#define HSS_CONFIG_TX_LUT	0x18
#define HSS_CONFIG_RX_LUT	0x38


/* NPE command codes */
/* writes the ConfigWord value to the location specified by offset */
#define PORT_CONFIG_WRITE			0x40

/* triggers the NPE to load the contents of the configuration table */
#define PORT_CONFIG_LOAD			0x41

/* triggers the NPE to return an HssErrorReadResponse message */
#define PORT_ERROR_READ				0x42

/* reset NPE internal status and enable the HssChannelized operation */
#define CHAN_FLOW_ENABLE			0x43
#define CHAN_FLOW_DISABLE			0x44
#define CHAN_IDLE_PATTERN_WRITE			0x45
#define CHAN_NUM_CHANS_WRITE			0x46
#define CHAN_RX_BUF_ADDR_WRITE			0x47
#define CHAN_RX_BUF_CFG_WRITE			0x48
#define CHAN_TX_BLK_CFG_WRITE			0x49
#define CHAN_TX_BUF_ADDR_WRITE			0x4A
#define CHAN_TX_BUF_SIZE_WRITE			0x4B
#define CHAN_TSLOTSWITCH_ENABLE			0x4C
#define CHAN_TSLOTSWITCH_DISABLE		0x4D

/* downloads the gainWord value for a timeslot switching channel associated
   with bypassNum */
#define CHAN_TSLOTSWITCH_GCT_DOWNLOAD		0x4E

/* triggers the NPE to reset internal status and enable the HssPacketized
   operation for the flow specified by pPipe */
#define PKT_PIPE_FLOW_ENABLE			0x50
#define PKT_PIPE_FLOW_DISABLE			0x51
#define PKT_NUM_PIPES_WRITE			0x52
#define PKT_PIPE_FIFO_SIZEW_WRITE		0x53
#define PKT_PIPE_HDLC_CFG_WRITE			0x54
#define PKT_PIPE_IDLE_PATTERN_WRITE		0x55
#define PKT_PIPE_RX_SIZE_WRITE			0x56
#define PKT_PIPE_MODE_WRITE			0x57

/* HDLC packet status values - desc->status */
#define ERR_SHUTDOWN		1 /* stop or shutdown occurrance */
#define ERR_HDLC_ALIGN		2 /* HDLC alignment error */
#define ERR_HDLC_FCS		3 /* HDLC Frame Check Sum error */
#define ERR_RXFREE_Q_EMPTY	4 /* RX-free queue became empty while receiving
				     this packet (if buf_len < pkt_len) */
#define ERR_HDLC_TOO_LONG	5 /* HDLC frame size too long */
#define ERR_HDLC_ABORT		6 /* abort sequence received */
#define ERR_DISCONNECTING	7 /* disconnect is in progress */


#ifdef __ARMEB__
typedef struct sk_buff buffer_t;
#define free_buffer dev_kfree_skb
#define free_buffer_irq dev_kfree_skb_irq
#else
typedef void buffer_t;
#define free_buffer kfree
#define free_buffer_irq kfree
#endif

struct port {
	struct npe *npe;
	struct net_device *netdev;
	struct hss_plat_info *plat;
	buffer_t *rx_buff_tab[RX_DESCS], *tx_buff_tab[TX_DESCS];
	struct desc *desc_tab;	/* coherent */
	u32 desc_tab_phys;
	int id;
	unsigned int clock_type, clock_rate, loopback;
	u8 hdlc_cfg;
};

/* NPE message structure */
struct msg {
#ifdef __ARMEB__
	u8 cmd, unused, hss_port, index;
	union {
		struct { u8 data8a, data8b, data8c, data8d; };
		struct { u16 data16a, data16b; };
		struct { u32 data32; };
	};
#else
	u8 index, hss_port, unused, cmd;
	union {
		struct { u8 data8d, data8c, data8b, data8a; };
		struct { u16 data16b, data16a; };
		struct { u32 data32; };
	};
#endif
};

/* HDLC packet descriptor */
struct desc {
	u32 next;		/* pointer to next buffer, unused */

#ifdef __ARMEB__
	u16 buf_len;		/* buffer length */
	u16 pkt_len;		/* packet length */
	u32 data;		/* pointer to data buffer in RAM */
	u8 status;
	u8 error_count;
	u16 __reserved;
#else
	u16 pkt_len;		/* packet length */
	u16 buf_len;		/* buffer length */
	u32 data;		/* pointer to data buffer in RAM */
	u16 __reserved;
	u8 error_count;
	u8 status;
#endif
	u32 __reserved1[4];
};


#define rx_desc_phys(port, n)	((port)->desc_tab_phys +		\
				 (n) * sizeof(struct desc))
#define rx_desc_ptr(port, n)	(&(port)->desc_tab[n])

#define tx_desc_phys(port, n)	((port)->desc_tab_phys +		\
				 ((n) + RX_DESCS) * sizeof(struct desc))
#define tx_desc_ptr(port, n)	(&(port)->desc_tab[(n) + RX_DESCS])

/*****************************************************************************
 * global variables
 ****************************************************************************/

static int ports_open;
static struct dma_pool *dma_pool;

static const struct {
	int tx, txdone, rx, rxfree;
}queue_ids[2] = {{ HSS0_PKT_TX0_QUEUE, HSS0_PKT_TXDONE_QUEUE,
		   HSS0_PKT_RX_QUEUE, HSS0_PKT_RXFREE0_QUEUE },
		 { HSS1_PKT_TX0_QUEUE, HSS1_PKT_TXDONE_QUEUE,
		   HSS1_PKT_RX_QUEUE, HSS1_PKT_RXFREE0_QUEUE },
};

/*****************************************************************************
 * utility functions
 ****************************************************************************/

static inline struct port* dev_to_port(struct net_device *dev)
{
	return dev_to_hdlc(dev)->priv;
}

#ifndef __ARMEB__
static inline void memcpy_swab32(u32 *dest, u32 *src, int cnt)
{
	int i;
	for (i = 0; i < cnt; i++)
		dest[i] = swab32(src[i]);
}
#endif

static inline void debug_pkt(struct net_device *dev, const char *func,
			     u8 *data, int len)
{
#if DEBUG_PKT_BYTES
	int i;

	printk(KERN_DEBUG "%s: %s(%i) ", dev->name, func, len);
	for (i = 0; i < len; i++) {
		if (i >= DEBUG_PKT_BYTES)
			break;
		printk("%s%02X", !(i % 4) ? " " : "", data[i]);
	}
	printk("\n");
#endif
}


static inline void debug_desc(u32 phys, struct desc *desc)
{
#if DEBUG_DESC
	printk(KERN_DEBUG "%X: %X %3X %3X %08X %X %X\n",
	       phys, desc->next, desc->buf_len, desc->pkt_len,
	       desc->data, desc->status, desc->error_count);
#endif
}

static inline void debug_queue(unsigned int queue, int is_get, u32 phys)
{
#if DEBUG_QUEUES
	static struct {
		int queue;
		char *name;
	} names[] = {
		{ HSS0_PKT_TX0_QUEUE, "TX#0 " },
		{ HSS0_PKT_TXDONE_QUEUE, "TX-done#0 " },
		{ HSS0_PKT_RX_QUEUE, "RX#0 " },
		{ HSS0_PKT_RXFREE0_QUEUE, "RX-free#0 " },
		{ HSS1_PKT_TX0_QUEUE, "TX#1 " },
		{ HSS1_PKT_TXDONE_QUEUE, "TX-done#1 " },
		{ HSS1_PKT_RX_QUEUE, "RX#1 " },
		{ HSS1_PKT_RXFREE0_QUEUE, "RX-free#1 " },
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(names); i++)
		if (names[i].queue == queue)
			break;

	printk(KERN_DEBUG "Queue %i %s%s %X\n", queue,
	       i < ARRAY_SIZE(names) ? names[i].name : "",
	       is_get ? "->" : "<-", phys);
#endif
}

static inline u32 queue_get_entry(unsigned int queue)
{
	u32 phys = qmgr_get_entry(queue);
	debug_queue(queue, 1, phys);
	return phys;
}

static inline int queue_get_desc(unsigned int queue, struct port *port,
				 int is_tx)
{
	u32 phys, tab_phys, n_desc;
	struct desc *tab;

	if (!(phys = queue_get_entry(queue)))
		return -1;

	BUG_ON(phys & 0x1F);
	tab_phys = is_tx ? tx_desc_phys(port, 0) : rx_desc_phys(port, 0);
	tab = is_tx ? tx_desc_ptr(port, 0) : rx_desc_ptr(port, 0);
	n_desc = (phys - tab_phys) / sizeof(struct desc);
	BUG_ON(n_desc >= (is_tx ? TX_DESCS : RX_DESCS));
	debug_desc(phys, &tab[n_desc]);
	BUG_ON(tab[n_desc].next);
	return n_desc;
}

static inline void queue_put_desc(unsigned int queue, u32 phys,
				  struct desc *desc)
{
	debug_queue(queue, 0, phys);
	debug_desc(phys, desc);
	BUG_ON(phys & 0x1F);
	qmgr_put_entry(queue, phys);
	BUG_ON(qmgr_stat_overflow(queue));
}


static inline void dma_unmap_tx(struct port *port, struct desc *desc)
{
#ifdef __ARMEB__
	dma_unmap_single(&port->netdev->dev, desc->data,
			 desc->buf_len, DMA_TO_DEVICE);
#else
	dma_unmap_single(&port->netdev->dev, desc->data & ~3,
			 ALIGN((desc->data & 3) + desc->buf_len, 4),
			 DMA_TO_DEVICE);
#endif
}


static void hss_hdlc_set_carrier(void *pdev, int carrier)
{
	struct net_device *dev = pdev;
	if (carrier)
		netif_carrier_on(dev);
	else
		netif_carrier_off(dev);
}

static void hss_hdlc_rx_irq(void *pdev)
{
	struct net_device *dev = pdev;
	struct port *port = dev_to_port(dev);

#if DEBUG_RX
	printk(KERN_DEBUG "%s: hss_hdlc_rx_irq\n", dev->name);
#endif
	qmgr_disable_irq(queue_ids[port->id].rx);
	netif_rx_schedule(dev);
}

static int hss_hdlc_poll(struct net_device *dev, int *budget)
{
	struct port *port = dev_to_port(dev);
	unsigned int rxq = queue_ids[port->id].rx;
	unsigned int rxfreeq = queue_ids[port->id].rxfree;
	struct net_device_stats *stats = hdlc_stats(dev);
	int quota = dev->quota, received = 0;

#if DEBUG_RX
	printk(KERN_DEBUG "%s: hss_hdlc_poll\n", dev->name);
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
#if DEBUG_RX
			printk(KERN_DEBUG "%s: hss_hdlc_poll"
			       " netif_rx_complete\n", dev->name);
#endif
			netif_rx_complete(dev);
			qmgr_enable_irq(rxq);
			if (!qmgr_stat_empty(rxq) &&
			    netif_rx_reschedule(dev, 0)) {
#if DEBUG_RX
				printk(KERN_DEBUG "%s: hss_hdlc_poll"
				       " netif_rx_reschedule successed\n",
				       dev->name);
#endif
				qmgr_disable_irq(rxq);
				continue;
			}
#if DEBUG_RX
			printk(KERN_DEBUG "%s: hss_hdlc_poll all done\n",
			       dev->name);
#endif
			return 0; /* all work done */
		}

		desc = rx_desc_ptr(port, n);

		if (desc->error_count) /* FIXME - remove printk */
			printk(KERN_DEBUG "%s: hss_hdlc_poll status 0x%02X"
			       " errors %u\n", dev->name, desc->status,
			       desc->error_count);

		skb = NULL;
		switch (desc->status) {
		case 0:
#ifdef __ARMEB__
			if ((skb = netdev_alloc_skb(dev, RX_SIZE)) != NULL) {
				phys = dma_map_single(&dev->dev, skb->data,
						      RX_SIZE,
						      DMA_FROM_DEVICE);
				if (dma_mapping_error(phys)) {
					dev_kfree_skb(skb);
					skb = NULL;
				}
			}
#else
			skb = netdev_alloc_skb(dev, desc->pkt_len);
#endif
			if (!skb)
				stats->rx_dropped++;
			break;
		case ERR_HDLC_ALIGN:
		case ERR_HDLC_ABORT:
			stats->rx_frame_errors++;
			stats->rx_errors++;
			break;
		case ERR_HDLC_FCS:
			stats->rx_crc_errors++;
			stats->rx_errors++;
			break;
		case ERR_HDLC_TOO_LONG:
			stats->rx_length_errors++;
			stats->rx_errors++;
			break;
		default:	/* FIXME - remove printk */
			printk(KERN_ERR "%s: hss_hdlc_poll: status 0x%02X"
			       " errors %u\n", dev->name, desc->status,
			       desc->error_count);
			stats->rx_errors++;
		}

		if (!skb) {
			/* put the desc back on RX-ready queue */
			desc->buf_len = RX_SIZE;
			desc->pkt_len = desc->status = 0;
			queue_put_desc(rxfreeq, rx_desc_phys(port, n), desc);
			continue;
		}

		/* process received frame */
#ifdef __ARMEB__
		temp = skb;
		skb = port->rx_buff_tab[n];
		dma_unmap_single(&dev->dev, desc->data,
				 RX_SIZE, DMA_FROM_DEVICE);
#else
		dma_sync_single(&dev->dev, desc->data,
				RX_SIZE, DMA_FROM_DEVICE);
		memcpy_swab32((u32 *)skb->data, (u32 *)port->rx_buff_tab[n],
			      ALIGN(desc->pkt_len, 4) / 4);
#endif
		skb_put(skb, desc->pkt_len);

		debug_pkt(dev, "hss_hdlc_poll", skb->data, skb->len);

		skb->protocol = hdlc_type_trans(skb, dev);
		dev->last_rx = jiffies;
		stats->rx_packets++;
		stats->rx_bytes += skb->len;
		netif_receive_skb(skb);

		/* put the new buffer on RX-free queue */
#ifdef __ARMEB__
		port->rx_buff_tab[n] = temp;
		desc->data = phys;
#endif
		desc->buf_len = RX_SIZE;
		desc->pkt_len = 0;
		queue_put_desc(rxfreeq, rx_desc_phys(port, n), desc);
		quota--;
		received++;
	}
	dev->quota -= received;
	*budget -= received;
#if DEBUG_RX
	printk(KERN_DEBUG "hss_hdlc_poll: end, not all work done\n");
#endif
	return 1;		/* not all work done */
}


static void hss_hdlc_txdone_irq(void *pdev)
{
	struct net_device *dev = pdev;
	struct port *port = dev_to_port(dev);
	struct net_device_stats *stats = hdlc_stats(dev);
	int n_desc;

#if DEBUG_TX
	printk(KERN_DEBUG DRV_NAME ": hss_hdlc_txdone_irq\n");
#endif
	while ((n_desc = queue_get_desc(queue_ids[port->id].txdone,
					port, 1)) >= 0) {
		struct desc *desc;
		int start;

		desc = tx_desc_ptr(port, n_desc);

		stats->tx_packets++;
		stats->tx_bytes += desc->pkt_len;

		dma_unmap_tx(port, desc);
#if DEBUG_TX
		printk(KERN_DEBUG "%s: hss_hdlc_txdone_irq free %p\n",
		       port->netdev->name, port->tx_buff_tab[n_desc]);
#endif
		free_buffer_irq(port->tx_buff_tab[n_desc]);
		port->tx_buff_tab[n_desc] = NULL;

		start = qmgr_stat_empty(port->plat->txreadyq);
		queue_put_desc(port->plat->txreadyq,
			       tx_desc_phys(port, n_desc), desc);
		if (start) {
#if DEBUG_TX
			printk(KERN_DEBUG "%s: hss_hdlc_txdone_irq xmit"
			       " ready\n", port->netdev->name);
#endif
			netif_wake_queue(port->netdev);
		}
	}
}

static int hss_hdlc_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct port *port = dev_to_port(dev);
	struct net_device_stats *stats = hdlc_stats(dev);
	unsigned int txreadyq = port->plat->txreadyq;
	int len, offset, bytes, n;
	void *mem;
	u32 phys;
	struct desc *desc;

#if DEBUG_TX
	printk(KERN_DEBUG "%s: hss_hdlc_xmit\n", dev->name);
#endif

	if (unlikely(skb->len > HDLC_MAX_MRU)) {
		dev_kfree_skb(skb);
		stats->tx_errors++;
		return NETDEV_TX_OK;
	}

	debug_pkt(dev, "hss_hdlc_xmit", skb->data, skb->len);

	len = skb->len;
#ifdef __ARMEB__
	offset = 0; /* no need to keep alignment */
	bytes = len;
	mem = skb->data;
#else
	offset = (int)skb->data & 3; /* keep 32-bit alignment */
	bytes = ALIGN(offset + len, 4);
	if (!(mem = kmalloc(bytes, GFP_ATOMIC))) {
		dev_kfree_skb(skb);
		stats->tx_dropped++;
		return NETDEV_TX_OK;
	}
	memcpy_swab32(mem, (u32 *)((int)skb->data & ~3), bytes / 4);
	dev_kfree_skb(skb);
#endif

	phys = dma_map_single(&dev->dev, mem, bytes, DMA_TO_DEVICE);
	if (dma_mapping_error(phys)) {
#ifdef __ARMEB__
		dev_kfree_skb(skb);
#else
		kfree(mem);
#endif
		stats->tx_dropped++;
		return NETDEV_TX_OK;
	}

	n = queue_get_desc(txreadyq, port, 1);
	BUG_ON(n < 0);
	desc = tx_desc_ptr(port, n);

#ifdef __ARMEB__
	port->tx_buff_tab[n] = skb;
#else
	port->tx_buff_tab[n] = mem;
#endif
	desc->data = phys + offset;
	desc->buf_len = desc->pkt_len = len;

	wmb();
	queue_put_desc(queue_ids[port->id].tx, tx_desc_phys(port, n), desc);
	dev->trans_start = jiffies;

	if (qmgr_stat_empty(txreadyq)) {
#if DEBUG_TX
		printk(KERN_DEBUG "%s: hss_hdlc_xmit queue full\n", dev->name);
#endif
		netif_stop_queue(dev);
		/* we could miss TX ready interrupt */
		if (!qmgr_stat_empty(txreadyq)) {
#if DEBUG_TX
			printk(KERN_DEBUG "%s: hss_hdlc_xmit ready again\n",
			       dev->name);
#endif
			netif_wake_queue(dev);
		}
	}

#if DEBUG_TX
	printk(KERN_DEBUG "%s: hss_hdlc_xmit end\n", dev->name);
#endif
	return NETDEV_TX_OK;
}


static int request_hdlc_queues(struct port *port)
{
	int err;

	err = qmgr_request_queue(queue_ids[port->id].rxfree, RX_DESCS, 0, 0);
	if (err)
		return err;

	err = qmgr_request_queue(queue_ids[port->id].rx, RX_DESCS, 0, 0);
	if (err)
		goto rel_rxfree;

	err = qmgr_request_queue(queue_ids[port->id].tx, TX_DESCS, 0, 0);
	if (err)
		goto rel_rx;

	err = qmgr_request_queue(port->plat->txreadyq, TX_DESCS, 0, 0);
	if (err)
		goto rel_tx;

	err = qmgr_request_queue(queue_ids[port->id].txdone, TX_DESCS, 0, 0);
	if (err)
		goto rel_txready;
	return 0;

rel_txready:
	qmgr_release_queue(port->plat->txreadyq);
rel_tx:
	qmgr_release_queue(queue_ids[port->id].tx);
rel_rx:
	qmgr_release_queue(queue_ids[port->id].rx);
rel_rxfree:
	qmgr_release_queue(queue_ids[port->id].rxfree);
	printk(KERN_DEBUG "%s: unable to request hardware queues\n",
	       port->netdev->name);
	return err;
}

static void release_hdlc_queues(struct port *port)
{
	qmgr_release_queue(queue_ids[port->id].rxfree);
	qmgr_release_queue(queue_ids[port->id].rx);
	qmgr_release_queue(queue_ids[port->id].txdone);
	qmgr_release_queue(queue_ids[port->id].tx);
	qmgr_release_queue(port->plat->txreadyq);
}

static int init_hdlc_queues(struct port *port)
{
	int i;

	if (!ports_open)
		if (!(dma_pool = dma_pool_create(DRV_NAME, NULL,
						 POOL_ALLOC_SIZE, 32, 0)))
			return -ENOMEM;

	if (!(port->desc_tab = dma_pool_alloc(dma_pool, GFP_KERNEL,
					      &port->desc_tab_phys)))
		return -ENOMEM;
	memset(port->desc_tab, 0, POOL_ALLOC_SIZE);
	memset(port->rx_buff_tab, 0, sizeof(port->rx_buff_tab)); /* tables */
	memset(port->tx_buff_tab, 0, sizeof(port->tx_buff_tab));

	/* Setup RX buffers */
	for (i = 0; i < RX_DESCS; i++) {
		struct desc *desc = rx_desc_ptr(port, i);
		buffer_t *buff;
		void *data;
#ifdef __ARMEB__
		if (!(buff = netdev_alloc_skb(port->netdev, RX_SIZE)))
			return -ENOMEM;
		data = buff->data;
#else
		if (!(buff = kmalloc(RX_SIZE, GFP_KERNEL)))
			return -ENOMEM;
		data = buff;
#endif
		desc->buf_len = RX_SIZE;
		desc->data = dma_map_single(&port->netdev->dev, data,
					    RX_SIZE, DMA_FROM_DEVICE);
		if (dma_mapping_error(desc->data)) {
			free_buffer(buff);
			return -EIO;
		}
		port->rx_buff_tab[i] = buff;
	}

	return 0;
}

static void destroy_hdlc_queues(struct port *port)
{
	int i;

	if (port->desc_tab) {
		for (i = 0; i < RX_DESCS; i++) {
			struct desc *desc = rx_desc_ptr(port, i);
			buffer_t *buff = port->rx_buff_tab[i];
			if (buff) {
				dma_unmap_single(&port->netdev->dev,
						 desc->data, RX_SIZE,
						 DMA_FROM_DEVICE);
				free_buffer(buff);
			}
		}
		for (i = 0; i < TX_DESCS; i++) {
			struct desc *desc = tx_desc_ptr(port, i);
			buffer_t *buff = port->tx_buff_tab[i];
			if (buff) {
				dma_unmap_tx(port, desc);
				free_buffer(buff);
			}
		}
		dma_pool_free(dma_pool, port->desc_tab, port->desc_tab_phys);
		port->desc_tab = NULL;
	}

	if (!ports_open && dma_pool) {
		dma_pool_destroy(dma_pool);
		dma_pool = NULL;
	}
}

static int hss_hdlc_open(struct net_device *dev)
{
	struct port *port = dev_to_port(dev);
	struct npe *npe = port->npe;
	struct msg msg;
	int i, err;

	if (!npe_running(npe)) {
		err = npe_load_firmware(npe, npe_name(npe), &dev->dev);
		if (err)
			return err;
	}

	if ((err = hdlc_open(dev)) != 0)
		return err;

	if (port->plat->open)
		if ((err = port->plat->open(port->id, port->netdev,
					    hss_hdlc_set_carrier)) != 0)
			goto err_hdlc_close;

	/* HSS main configuration */
	memset(&msg, 0, sizeof(msg));
	msg.cmd = PORT_CONFIG_WRITE;
	msg.hss_port = port->id;
	msg.index = 0;		/* offset in HSS config */

	msg.data32 = PCR_FRM_PULSE_DISABLED |
		PCR_SOF_NO_FBIT |
		PCR_MSB_ENDIAN |
		PCR_TX_DATA_ENABLE;

	if (port->clock_type == CLOCK_INT)
		msg.data32 |= PCR_SYNC_CLK_DIR_OUTPUT;

	if ((err = npe_send_message(npe, &msg, "HSS_SET_TX_PCR") != 0))
		goto err_plat_close; /* 0: TX PCR */

	msg.index = 4;
	msg.data32 ^= PCR_TX_DATA_ENABLE | PCR_DCLK_EDGE_RISING;
	if ((err = npe_send_message(npe, &msg, "HSS_SET_RX_PCR") != 0))
		goto err_plat_close; /* 4: RX PCR */

	msg.index = 8;
	msg.data32 = (port->loopback ? CCR_LOOPBACK : 0) |
		(port->id ? CCR_SECOND_HSS : 0);
	if ((err = npe_send_message(npe, &msg, "HSS_SET_CORE_CR") != 0))
		goto err_plat_close; /* 8: Core CR */

	msg.index = 12;
	msg.data32 = CLK42X_SPEED_2048KHZ /* FIXME */;
	if ((err = npe_send_message(npe, &msg, "HSS_SET_CLK_CR") != 0))
		goto err_plat_close; /* 12: CLK CR */

	msg.data32 = (FRAME_SYNC_OFFSET << 16) | (FRAME_SYNC_SIZE - 1);
	msg.index = 16;
	if ((err = npe_send_message(npe, &msg, "HSS_SET_TX_FCR") != 0))
		goto err_plat_close; /* 16: TX FCR */

	msg.index = 20;
	if ((err = npe_send_message(npe, &msg, "HSS_SET_RX_FCR") != 0))
		goto err_plat_close; /* 20: RX FCR */

	msg.data32 = 0;		/* Fill LUT with HDLC timeslots */
	for (i = 0; i < 32 / LUT_BITS; i++)
		msg.data32 |= TDMMAP_HDLC << (LUT_BITS * i);

	for (i = 0; i < 2 /* TX and RX */ * TIMESLOTS * LUT_BITS / 8; i += 4) {
		msg.index = 24 + i; /* 24 - 55: TX LUT, 56 - 87: RX LUT */
		if ((err = npe_send_message(npe, &msg, "HSS_SET_LUT") != 0))
			goto err_plat_close;
	}

	/* HDLC mode configuration */
	memset(&msg, 0, sizeof(msg));
	msg.cmd = PKT_NUM_PIPES_WRITE;
	msg.hss_port = port->id;
	msg.data8a = PKT_NUM_PIPES;
	if ((err = npe_send_message(npe, &msg, "HSS_SET_PKT_PIPES") != 0))
		goto err_plat_close;

	memset(&msg, 0, sizeof(msg));
	msg.cmd = PKT_PIPE_FIFO_SIZEW_WRITE;
	msg.hss_port = port->id;
	msg.data8a = PKT_PIPE_FIFO_SIZEW;
	if ((err = npe_send_message(npe, &msg, "HSS_SET_PKT_FIFO") != 0))
		goto err_plat_close;

	memset(&msg, 0, sizeof(msg));
	msg.cmd = PKT_PIPE_IDLE_PATTERN_WRITE;
	msg.hss_port = port->id;
	msg.data32 = 0x7F7F7F7F;
	if ((err = npe_send_message(npe, &msg, "HSS_SET_PKT_IDLE") != 0))
		goto err_plat_close;

	memset(&msg, 0, sizeof(msg));
	msg.cmd = PORT_CONFIG_LOAD;
	msg.hss_port = port->id;
	if ((err = npe_send_message(npe, &msg, "HSS_LOAD_CONFIG") != 0))
		goto err_plat_close;
	if ((err = npe_recv_message(npe, &msg, "HSS_LOAD_CONFIG") != 0))
		goto err_plat_close;

	/* HSS_LOAD_CONFIG for port #1 returns port_id = #4 */
	if (msg.cmd != PORT_CONFIG_LOAD || msg.data32) {
		printk(KERN_DEBUG "%s: unexpected message received in"
		       " response to HSS_LOAD_CONFIG\n", npe_name(npe));
		err = EIO;
		goto err_plat_close;
	}

	memset(&msg, 0, sizeof(msg));
	msg.cmd = PKT_PIPE_HDLC_CFG_WRITE;
	msg.hss_port = port->id;
	msg.data8a = port->hdlc_cfg; /* rx_cfg */
	msg.data8b = port->hdlc_cfg | (PKT_EXTRA_FLAGS << 3); /* tx_cfg */
	if ((err = npe_send_message(npe, &msg, "HSS_SET_HDLC_CFG") != 0))
		goto err_plat_close;

	memset(&msg, 0, sizeof(msg));
	msg.cmd = PKT_PIPE_MODE_WRITE;
	msg.hss_port = port->id;
	msg.data8a = NPE_PKT_MODE_HDLC;
	/* msg.data8b = inv_mask */
	/* msg.data8c = or_mask */
	if ((err = npe_send_message(npe, &msg, "HSS_SET_PKT_MODE") != 0))
		goto err_plat_close;

	memset(&msg, 0, sizeof(msg));
	msg.cmd = PKT_PIPE_RX_SIZE_WRITE;
	msg.hss_port = port->id;
	msg.data16a = HDLC_MAX_MRU;
	if ((err = npe_send_message(npe, &msg, "HSS_SET_PKT_RX_SIZE") != 0))
		goto err_plat_close;

	if ((err = request_hdlc_queues(port)) != 0)
		goto err_plat_close;

	if ((err = init_hdlc_queues(port)) != 0)
		goto err_destroy_queues;

	memset(&msg, 0, sizeof(msg));
	msg.cmd = PKT_PIPE_FLOW_ENABLE;
	msg.hss_port = port->id;
	if ((err = npe_send_message(npe, &msg, "HSS_ENABLE_PKT_PIPE") != 0))
		goto err_destroy_queues;

	/* Populate queues with buffers, no failure after this point */
	for (i = 0; i < TX_DESCS; i++)
		queue_put_desc(port->plat->txreadyq,
			       tx_desc_phys(port, i), tx_desc_ptr(port, i));

	for (i = 0; i < RX_DESCS; i++)
		queue_put_desc(queue_ids[port->id].rxfree,
			       rx_desc_phys(port, i), rx_desc_ptr(port, i));

	netif_start_queue(dev);

	qmgr_set_irq(queue_ids[port->id].rx, QUEUE_IRQ_SRC_NOT_EMPTY,
		     hss_hdlc_rx_irq, dev);

	qmgr_set_irq(queue_ids[port->id].txdone, QUEUE_IRQ_SRC_NOT_EMPTY,
		     hss_hdlc_txdone_irq, dev);
	qmgr_enable_irq(queue_ids[port->id].txdone);

	ports_open++;
	netif_rx_schedule(dev); /* we may already have RX data, enables IRQ */
	return 0;

err_destroy_queues:
	destroy_hdlc_queues(port);
	release_hdlc_queues(port);
err_plat_close:
	if (port->plat->close)
		port->plat->close(port->id, port->netdev);
err_hdlc_close:
	hdlc_close(dev);
	return err;
}

static int hss_hdlc_close(struct net_device *dev)
{
	struct port *port = dev_to_port(dev);
	struct npe *npe = port->npe;
	struct msg msg;
	int buffs = RX_DESCS; /* allocated RX buffers */
	int i;

	ports_open--;
	qmgr_disable_irq(queue_ids[port->id].rx);
	netif_stop_queue(dev);

	memset(&msg, 0, sizeof(msg));
	msg.cmd = PKT_PIPE_FLOW_DISABLE;
	msg.hss_port = port->id;
	if (npe_send_message(npe, &msg, "HSS_DISABLE_PKT_PIPE")) {
		printk(KERN_CRIT "HSS-%i: unable to stop HDLC flow\n",
		       port->id);
		/* The upper level would ignore the error anyway */
	}

	while (queue_get_desc(queue_ids[port->id].rxfree, port, 0) >= 0)
		buffs--;
	while (queue_get_desc(queue_ids[port->id].rx, port, 0) >= 0)
		buffs--;

	if (buffs)
		printk(KERN_CRIT "%s: unable to drain RX queue, %i buffer(s)"
		       " left in NPE\n", dev->name, buffs);

	buffs = TX_DESCS;
	while (queue_get_desc(queue_ids[port->id].tx, port, 1) >= 0)
		buffs--; /* cancel TX */

	i = 0;
	do {
		while (queue_get_desc(port->plat->txreadyq, port, 1) >= 0)
			buffs--;
		if (!buffs)
			break;
	} while (++i < MAX_CLOSE_WAIT);

	if (buffs)
		printk(KERN_CRIT "%s: unable to drain TX queue, %i buffer(s) "
		       "left in NPE\n", dev->name, buffs);
#if DEBUG_CLOSE
	if (!buffs)
		printk(KERN_DEBUG "Draining TX queues took %i cycles\n", i);
#endif
	qmgr_disable_irq(queue_ids[port->id].txdone);
	destroy_hdlc_queues(port);
	release_hdlc_queues(port);

	if (port->plat->close)
		port->plat->close(port->id, port->netdev);
	hdlc_close(dev);
	return 0;
}


static int hss_hdlc_attach(struct net_device *dev, unsigned short encoding,
			   unsigned short parity)
{
	struct port *port = dev_to_port(dev);

	if (encoding != ENCODING_NRZ)
		return -EINVAL;

	switch(parity) {
	case PARITY_CRC16_PR1_CCITT:
		port->hdlc_cfg = 0;
		return 0;

	case PARITY_CRC32_PR1_CCITT:
		port->hdlc_cfg = PKT_HDLC_CRC_32;
		return 0;

	default:
		return -EINVAL;
	}
}


static int hss_hdlc_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	const size_t size = sizeof(sync_serial_settings);
	sync_serial_settings new_line;
	int clk;
	sync_serial_settings __user *line = ifr->ifr_settings.ifs_ifsu.sync;
	struct port *port = dev_to_port(dev);

	if (cmd != SIOCWANDEV)
		return hdlc_ioctl(dev, ifr, cmd);

	switch(ifr->ifr_settings.type) {
	case IF_GET_IFACE:
		ifr->ifr_settings.type = IF_IFACE_V35;
		if (ifr->ifr_settings.size < size) {
			ifr->ifr_settings.size = size; /* data size wanted */
			return -ENOBUFS;
		}
		memset(&new_line, 0, sizeof(new_line));
		new_line.clock_type = port->clock_type;
		new_line.clock_rate = port->clock_rate;
		new_line.loopback = port->loopback;
		if (copy_to_user(line, &new_line, size))
			return -EFAULT;
		return 0;

	case IF_IFACE_SYNC_SERIAL:
	case IF_IFACE_V35:
		if(!capable(CAP_NET_ADMIN))
			return -EPERM;
		if (dev->flags & IFF_UP)
			return -EBUSY; /* Cannot change parameters when open */

		if (copy_from_user(&new_line, line, size))
			return -EFAULT;

		clk = new_line.clock_type;
		if (port->plat->set_clock)
			clk = port->plat->set_clock(port->id, clk);

		if (clk != CLOCK_EXT && clk != CLOCK_INT)
			return -EINVAL;	/* No such clock setting */

		if (new_line.loopback != 0 && new_line.loopback != 1)
			return -EINVAL;

		port->clock_type = clk; /* Update settings */
		port->clock_rate = new_line.clock_rate;
		port->loopback = new_line.loopback;
		return 0;

	default:
		return hdlc_ioctl(dev, ifr, cmd);
	}
}


static int __devinit hss_init_one(struct platform_device *pdev)
{
	struct port *port;
	struct net_device *dev;
	hdlc_device *hdlc;
	int err;

	if ((port = kzalloc(sizeof(*port), GFP_KERNEL)) == NULL)
		return -ENOMEM;
	platform_set_drvdata(pdev, port);
	port->id = pdev->id;

	if ((port->npe = npe_request(0)) == NULL) {
		err = -ENOSYS;
		goto err_free;
	}

	port->plat = pdev->dev.platform_data;
	if ((port->netdev = dev = alloc_hdlcdev(port)) == NULL) {
		err = -ENOMEM;
		goto err_plat;
	}

	SET_MODULE_OWNER(net);
	SET_NETDEV_DEV(dev, &pdev->dev);
	hdlc = dev_to_hdlc(dev);
	hdlc->attach = hss_hdlc_attach;
	hdlc->xmit = hss_hdlc_xmit;
	dev->open = hss_hdlc_open;
	dev->poll = hss_hdlc_poll;
	dev->stop = hss_hdlc_close;
	dev->do_ioctl = hss_hdlc_ioctl;
	dev->weight = 16;
	dev->tx_queue_len = 100;
	port->clock_type = CLOCK_EXT;
	port->clock_rate = 2048000;

	if (register_hdlc_device(dev)) {
		printk(KERN_ERR "HSS-%i: unable to register HDLC device\n",
		       port->id);
		err = -ENOBUFS;
		goto err_free_netdev;
	}
	printk(KERN_INFO "%s: HSS-%i\n", dev->name, port->id);
	return 0;

err_free_netdev:
	free_netdev(dev);
err_plat:
	npe_release(port->npe);
	platform_set_drvdata(pdev, NULL);
err_free:
	kfree(port);
	return err;
}

static int __devexit hss_remove_one(struct platform_device *pdev)
{
	struct port *port = platform_get_drvdata(pdev);

	unregister_hdlc_device(port->netdev);
	free_netdev(port->netdev);
	npe_release(port->npe);
	platform_set_drvdata(pdev, NULL);
	kfree(port);
	return 0;
}

static struct platform_driver drv = {
	.driver.name	= DRV_NAME,
	.probe		= hss_init_one,
	.remove		= hss_remove_one,
};

static int __init hss_init_module(void)
{
	if ((ixp4xx_read_feature_bits() &
	     (IXP4XX_FEATURE_HDLC | IXP4XX_FEATURE_HSS)) !=
	    (IXP4XX_FEATURE_HDLC | IXP4XX_FEATURE_HSS))
		return -ENOSYS;
	return platform_driver_register(&drv);
}

static void __exit hss_cleanup_module(void)
{
	platform_driver_unregister(&drv);
}

MODULE_AUTHOR("Krzysztof Halasa");
MODULE_DESCRIPTION("Intel IXP4xx HSS driver");
MODULE_LICENSE("GPL v2");
module_init(hss_init_module);
module_exit(hss_cleanup_module);

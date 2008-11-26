/*
 *             Pseudo-driver for the intermediate queue device.
 *
 *             This program is free software; you can redistribute it and/or
 *             modify it under the terms of the GNU General Public License
 *             as published by the Free Software Foundation; either version
 *             2 of the License, or (at your option) any later version.
 *
 * Authors:    Patrick McHardy, <kaber@trash.net>
 *
 *            The first version was written by Martin Devera, <devik@cdi.cz>
 *
 * Credits:    Jan Rafaj <imq2t@cedric.vabo.cz>
 *              - Update patch to 2.4.21
 *             Sebastian Strollo <sstrollo@nortelnetworks.com>
 *              - Fix "Dead-loop on netdevice imq"-issue
 *             Marcel Sebek <sebek64@post.cz>
 *              - Update to 2.6.2-rc1
 *
 *	       After some time of inactivity there is a group taking care
 *	       of IMQ again: http://www.linuximq.net
 *
 *
 *	       2004/06/30 - New version of IMQ patch to kernels <=2.6.7
 *             including the following changes:
 *
 *	       - Correction of ipv6 support "+"s issue (Hasso Tepper)
 *	       - Correction of imq_init_devs() issue that resulted in
 *	       kernel OOPS unloading IMQ as module (Norbert Buchmuller)
 *	       - Addition of functionality to choose number of IMQ devices
 *	       during kernel config (Andre Correa)
 *	       - Addition of functionality to choose how IMQ hooks on
 *	       PRE and POSTROUTING (after or before NAT) (Andre Correa)
 *	       - Cosmetic corrections (Norbert Buchmuller) (Andre Correa)
 *
 *
 *             2005/12/16 - IMQ versions between 2.6.7 and 2.6.13 were
 *             released with almost no problems. 2.6.14-x was released
 *             with some important changes: nfcache was removed; After
 *             some weeks of trouble we figured out that some IMQ fields
 *             in skb were missing in skbuff.c - skb_clone and copy_skb_header.
 *             These functions are correctly patched by this new patch version.
 *
 *             Thanks for all who helped to figure out all the problems with
 *             2.6.14.x: Patrick McHardy, Rune Kock, VeNoMouS, Max CtRiX,
 *             Kevin Shanahan, Richard Lucassen, Valery Dachev (hopefully
 *             I didn't forget anybody). I apologize again for my lack of time.
 *
 *
 *             2008/06/17 - 2.6.25 - Changed imq.c to use qdisc_run() instead 
 *	       of qdisc_restart() and moved qdisc_run() to tasklet to avoid
 *             recursive locking. New initialization routines to fix 'rmmod' not
 *             working anymore. Used code from ifb.c. (Jussi Kivilinna)
 *	       
 *	       Also, many thanks to pablo Sebastian Greco for making the initial
 *	       patch and to those who helped the testing.
 *
 *             More info at: http://www.linuximq.net/ (Andre Correa)
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/rtnetlink.h>
#include <linux/if_arp.h>
#include <linux/netfilter.h>
#include <linux/netfilter_ipv4.h>
#if defined(CONFIG_IPV6) || defined(CONFIG_IPV6_MODULE)
	#include <linux/netfilter_ipv6.h>
#endif
#include <linux/imq.h>
#include <net/pkt_sched.h>
#include <net/netfilter/nf_queue.h>

struct imq_private {
	struct tasklet_struct tasklet;
	int tasklet_pending;
};

static nf_hookfn imq_nf_hook;

static struct nf_hook_ops imq_ingress_ipv4 = {
	.hook		= imq_nf_hook,
	.owner		= THIS_MODULE,
	.pf		= PF_INET,
	.hooknum	= NF_INET_PRE_ROUTING,
#if defined(CONFIG_IMQ_BEHAVIOR_BA) || defined(CONFIG_IMQ_BEHAVIOR_BB)
	.priority	= NF_IP_PRI_MANGLE + 1
#else
	.priority	= NF_IP_PRI_NAT_DST + 1
#endif
};

static struct nf_hook_ops imq_egress_ipv4 = {
	.hook		= imq_nf_hook,
	.owner		= THIS_MODULE,
	.pf		= PF_INET,
	.hooknum	= NF_INET_POST_ROUTING,
#if defined(CONFIG_IMQ_BEHAVIOR_AA) || defined(CONFIG_IMQ_BEHAVIOR_BA)
	.priority	= NF_IP_PRI_LAST
#else
	.priority	= NF_IP_PRI_NAT_SRC - 1
#endif
};

#if defined(CONFIG_IPV6) || defined(CONFIG_IPV6_MODULE)
static struct nf_hook_ops imq_ingress_ipv6 = {
	.hook		= imq_nf_hook,
	.owner		= THIS_MODULE,
	.pf		= PF_INET6,
	.hooknum	= NF_INET_PRE_ROUTING,
#if defined(CONFIG_IMQ_BEHAVIOR_BA) || defined(CONFIG_IMQ_BEHAVIOR_BB)
	.priority	= NF_IP6_PRI_MANGLE + 1
#else
	.priority	= NF_IP6_PRI_NAT_DST + 1
#endif
};

static struct nf_hook_ops imq_egress_ipv6 = {
	.hook		= imq_nf_hook,
	.owner		= THIS_MODULE,
	.pf		= PF_INET6,
	.hooknum	= NF_INET_POST_ROUTING,
#if defined(CONFIG_IMQ_BEHAVIOR_AA) || defined(CONFIG_IMQ_BEHAVIOR_BA)
	.priority	= NF_IP6_PRI_LAST
#else
	.priority	= NF_IP6_PRI_NAT_SRC - 1
#endif
};
#endif

#if defined(CONFIG_IMQ_NUM_DEVS)
static unsigned int numdevs = CONFIG_IMQ_NUM_DEVS;
#else
static unsigned int numdevs = IMQ_MAX_DEVS;
#endif

static struct net_device *imq_devs_cache[IMQ_MAX_DEVS];

static struct net_device_stats *imq_get_stats(struct net_device *dev)
{
	return &dev->stats;
}

/* called for packets kfree'd in qdiscs at places other than enqueue */
static void imq_skb_destructor(struct sk_buff *skb)
{
	struct nf_queue_entry *entry = skb->nf_queue_entry;

	if (entry) {
		if (entry->indev)
			dev_put(entry->indev);
		if (entry->outdev)
			dev_put(entry->outdev);
		kfree(entry);
	}
}

static int imq_dev_xmit(struct sk_buff *skb, struct net_device *dev)
{
	dev->stats.tx_bytes += skb->len;
	dev->stats.tx_packets++;

	skb->imq_flags = 0;
	skb->destructor = NULL;

	dev->trans_start = jiffies;
	nf_reinject(skb->nf_queue_entry, NF_ACCEPT);
	return 0;
}

static int imq_nf_queue(struct nf_queue_entry *entry, unsigned queue_num)
{
	struct net_device *dev;
	struct imq_private *priv;
	struct sk_buff *skb2 = NULL;
	struct Qdisc *q;
	unsigned int index = entry->skb->imq_flags & IMQ_F_IFMASK;
	int ret = -1;

	if (index > numdevs)
		return -1;

	/* check for imq device by index from cache */
	dev = imq_devs_cache[index];
	if (!dev) {
		char buf[8];

		/* get device by name and cache result */
		snprintf(buf, sizeof(buf), "imq%d", index);
		dev = dev_get_by_name(&init_net, buf);
		if (!dev) {
			/* not found ?!*/
			BUG();
			return -1;
		}

		imq_devs_cache[index] = dev;
	}

	priv = netdev_priv(dev);
	if (!(dev->flags & IFF_UP)) {
		entry->skb->imq_flags = 0;
		nf_reinject(entry, NF_ACCEPT);
		return 0;
	}
	dev->last_rx = jiffies;

	if (entry->skb->destructor) {
		skb2 = entry->skb;
		entry->skb = skb_clone(entry->skb, GFP_ATOMIC);
		if (!entry->skb)
			return -1;
	}
	entry->skb->nf_queue_entry = entry;

	dev->stats.rx_bytes += entry->skb->len;
	dev->stats.rx_packets++;

	spin_lock_bh(&dev->queue_lock);
	q = dev->qdisc;
	if (q->enqueue) {
		q->enqueue(skb_get(entry->skb), q);
		if (skb_shared(entry->skb)) {
			entry->skb->destructor = imq_skb_destructor;
			kfree_skb(entry->skb);
			ret = 0;
		}
	}
	if (!test_and_set_bit(1, &priv->tasklet_pending))
		tasklet_schedule(&priv->tasklet);
	spin_unlock_bh(&dev->queue_lock);

	if (skb2)
		kfree_skb(ret ? entry->skb : skb2);

	return ret;
}

static struct nf_queue_handler nfqh = {
	.name  = "imq",
	.outfn = imq_nf_queue,
};

static void qdisc_run_tasklet(unsigned long arg)
{
	struct net_device *dev = (struct net_device *)arg;
	struct imq_private *priv = netdev_priv(dev);

	spin_lock(&dev->queue_lock);
	qdisc_run(dev);
	clear_bit(1, &priv->tasklet_pending);
	spin_unlock(&dev->queue_lock);
}

static unsigned int imq_nf_hook(unsigned int hook, struct sk_buff *pskb,
				const struct net_device *indev,
				const struct net_device *outdev,
				int (*okfn)(struct sk_buff *))
{
	if (pskb->imq_flags & IMQ_F_ENQUEUE)
		return NF_QUEUE;

	return NF_ACCEPT;
}

static int imq_close(struct net_device *dev)
{
	struct imq_private *priv = netdev_priv(dev);

	tasklet_kill(&priv->tasklet);
	netif_stop_queue(dev);

	return 0;
}

static int imq_open(struct net_device *dev)
{
	struct imq_private *priv = netdev_priv(dev);

	tasklet_init(&priv->tasklet, qdisc_run_tasklet, (unsigned long)dev);
	netif_start_queue(dev);

	return 0;
}

static void imq_setup(struct net_device *dev)
{
	dev->hard_start_xmit    = imq_dev_xmit;
	dev->open		= imq_open;
	dev->get_stats		= imq_get_stats;
	dev->stop		= imq_close;
	dev->type               = ARPHRD_VOID;
	dev->mtu                = 16000;
	dev->tx_queue_len       = 11000;
	dev->flags              = IFF_NOARP;
}

static struct rtnl_link_ops imq_link_ops __read_mostly = {
	.kind		= "imq",
	.priv_size	= sizeof(struct imq_private),
	.setup		= imq_setup,
};

static int __init imq_init_hooks(void)
{
	int err;

	err = nf_register_queue_handler(PF_INET, &nfqh);
	if (err)
		goto err1;

	err = nf_register_hook(&imq_ingress_ipv4);
	if (err)
		goto err2;

	err = nf_register_hook(&imq_egress_ipv4);
	if (err)
		goto err3;

#if defined(CONFIG_IPV6) || defined(CONFIG_IPV6_MODULE)
	err = nf_register_queue_handler(PF_INET6, &nfqh);
	if (err)
		goto err4;

	err = nf_register_hook(&imq_ingress_ipv6);
	if (err)
		goto err5;

	err = nf_register_hook(&imq_egress_ipv6);
	if (err)
		goto err6;
#endif

	return 0;

#if defined(CONFIG_IPV6) || defined(CONFIG_IPV6_MODULE)
err6:
	nf_unregister_hook(&imq_ingress_ipv6);
err5:
	nf_unregister_queue_handler(PF_INET6, &nfqh);
err4:
	nf_unregister_hook(&imq_egress_ipv4);
#endif
err3:
	nf_unregister_hook(&imq_ingress_ipv4);
err2:
	nf_unregister_queue_handler(PF_INET, &nfqh);
err1:
	return err;
}

static int __init imq_init_one(int index)
{
	struct net_device *dev;
	int ret;

	dev = alloc_netdev(sizeof(struct imq_private), "imq%d", imq_setup);
	if (!dev)
		return -ENOMEM;

	ret = dev_alloc_name(dev, dev->name);
	if (ret < 0)
		goto fail;

	dev->rtnl_link_ops = &imq_link_ops;
	ret = register_netdevice(dev);
	if (ret < 0)
		goto fail;

	return 0;
fail:
	free_netdev(dev);
	return ret;
}

static int __init imq_init_devs(void)
{
	int err, i;

	if (!numdevs || numdevs > IMQ_MAX_DEVS) {
		printk(KERN_ERR "IMQ: numdevs has to be betweed 1 and %u\n",
		       IMQ_MAX_DEVS);
		return -EINVAL;
	}

	rtnl_lock();
	err = __rtnl_link_register(&imq_link_ops);

	for (i = 0; i < numdevs && !err; i++)
		err = imq_init_one(i);

	if (err) {
		__rtnl_link_unregister(&imq_link_ops);
		memset(imq_devs_cache, 0, sizeof(imq_devs_cache));
	}
	rtnl_unlock();

	return err;
}

static int __init imq_init_module(void)
{
	int err;

	err = imq_init_devs();
	if (err) {
		printk(KERN_ERR "IMQ: Error trying imq_init_devs(net)\n");
		return err;
	}

	err = imq_init_hooks();
	if (err) {
		printk(KERN_ERR "IMQ: Error trying imq_init_hooks()\n");
		rtnl_link_unregister(&imq_link_ops);
		memset(imq_devs_cache, 0, sizeof(imq_devs_cache));
		return err;
	}

	printk(KERN_INFO "IMQ driver loaded successfully.\n");

#if defined(CONFIG_IMQ_BEHAVIOR_BA) || defined(CONFIG_IMQ_BEHAVIOR_BB)
	printk(KERN_INFO "\tHooking IMQ before NAT on PREROUTING.\n");
#else
	printk(KERN_INFO "\tHooking IMQ after NAT on PREROUTING.\n");
#endif
#if defined(CONFIG_IMQ_BEHAVIOR_AB) || defined(CONFIG_IMQ_BEHAVIOR_BB)
	printk(KERN_INFO "\tHooking IMQ before NAT on POSTROUTING.\n");
#else
	printk(KERN_INFO "\tHooking IMQ after NAT on POSTROUTING.\n");
#endif

	return 0;
}

static void __exit imq_unhook(void)
{
#if defined(CONFIG_IPV6) || defined(CONFIG_IPV6_MODULE)
	nf_unregister_hook(&imq_ingress_ipv6);
	nf_unregister_hook(&imq_egress_ipv6);
	nf_unregister_queue_handler(PF_INET6, &nfqh);
#endif
	nf_unregister_hook(&imq_ingress_ipv4);
	nf_unregister_hook(&imq_egress_ipv4);
	nf_unregister_queue_handler(PF_INET, &nfqh);
}

static void __exit imq_cleanup_devs(void)
{
	rtnl_link_unregister(&imq_link_ops);
	memset(imq_devs_cache, 0, sizeof(imq_devs_cache));
}

static void __exit imq_exit_module(void)
{
	imq_unhook();
	imq_cleanup_devs();
	printk(KERN_INFO "IMQ driver unloaded successfully.\n");
}

module_init(imq_init_module);
module_exit(imq_exit_module);

module_param(numdevs, int, 0);
MODULE_PARM_DESC(numdevs, "number of IMQ devices (how many imq* devices will "
			"be created)");
MODULE_AUTHOR("http://www.linuximq.net");
MODULE_DESCRIPTION("Pseudo-driver for the intermediate queue device. See "
			"http://www.linuximq.net/ for more information.");
MODULE_LICENSE("GPL");
MODULE_ALIAS_RTNL_LINK("imq");


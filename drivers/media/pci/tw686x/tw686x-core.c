/*
 * Copyright (C) 2015 Industrial Research Institute for Automation
 * and Measurements PIAP
 *
 * Written by Krzysztof Hałasa.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License
 * as published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include "tw686x.h"
#include "tw686x-regs.h"

static irqreturn_t tw686x_irq(int irq, void *dev_id)
{
	struct tw686x_dev *dev = (struct tw686x_dev *)dev_id;
	u32 int_status = reg_read(dev, INT_STATUS); /* cleared on read */
	unsigned long flags;
	unsigned handled = 0;

	if (int_status) {
		spin_lock_irqsave(&dev->irq_lock, flags);
		dev->dma_requests |= int_status;
		spin_unlock_irqrestore(&dev->irq_lock, flags);

		if (int_status & 0xFF0000FF)
			handled = tw686x_video_irq(dev);
	}

	return IRQ_RETVAL(handled);
}

static int tw686x_probe(struct pci_dev *pci_dev,
			const struct pci_device_id *pci_id)
{
	struct tw686x_dev *dev;
	int err;

	dev = kzalloc(sizeof(*dev) + (pci_id->driver_data & TYPE_MAX_CHANNELS) *
		      sizeof(dev->video_channels[0]), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	sprintf(dev->name, "TW%04X", pci_dev->device);
	dev->type = pci_id->driver_data;

	pr_info("%s: PCI %s, IRQ %d, MMIO 0x%lx\n", dev->name,
		pci_name(pci_dev), pci_dev->irq,
		(unsigned long)pci_resource_start(pci_dev, 0));

	dev->pci_dev = pci_dev;
	if (pci_enable_device(pci_dev)) {
		err = -EIO;
		goto free_dev;
	}

	pci_set_master(pci_dev);

	if (!pci_dma_supported(pci_dev, DMA_BIT_MASK(32))) {
		pr_err("%s: 32-bit PCI DMA not supported\n", dev->name);
		err = -EIO;
		goto disable;
	}

	err = pci_request_regions(pci_dev, dev->name);
	if (err < 0) {
		pr_err("%s: Unable to get MMIO region\n", dev->name);
		goto disable;
	}

	dev->mmio = pci_ioremap_bar(pci_dev, 0);
	if (!dev->mmio) {
		pr_err("%s: Unable to remap MMIO region\n", dev->name);
		err = -EIO;
		goto free_region;
	}

	reg_write(dev, SYS_SOFT_RST, 0x0F); /* Reset all subsystems */
	mdelay(1);

	reg_write(dev, SRST[0], 0x3F);
	if (max_channels(dev) > 4)
		reg_write(dev, SRST[1], 0x3F);
	reg_write(dev, DMA_CMD, 0);
	reg_write(dev, DMA_CHANNEL_ENABLE, 0);
	reg_write(dev, DMA_CHANNEL_TIMEOUT, 0x3EFF0FF0);
	reg_write(dev, DMA_TIMER_INTERVAL, 0x38000);
	reg_write(dev, DMA_CONFIG, 0xFFFFFF04);

	spin_lock_init(&dev->irq_lock);

	err = request_irq(pci_dev->irq, tw686x_irq, IRQF_SHARED, dev->name,
			  dev);
	if (err < 0) {
		pr_err("%s: Unable to get IRQ\n", dev->name);
		goto iounmap;
	}

	err = tw686x_video_init(dev);
	if (err)
		goto free_irq;

	pci_set_drvdata(pci_dev, dev);
	return 0;

free_irq:
	free_irq(pci_dev->irq, dev);
iounmap:
	iounmap(dev->mmio);
free_region:
	release_mem_region(pci_resource_start(pci_dev, 0),
			   pci_resource_len(pci_dev, 0));
disable:
	pci_disable_device(pci_dev);
free_dev:
	kfree(dev);
	return err;
}

static void tw686x_remove(struct pci_dev *pci_dev)
{
	struct tw686x_dev *dev = pci_get_drvdata(pci_dev);

	tw686x_video_free(dev);

	free_irq(pci_dev->irq, dev);
	iounmap(dev->mmio);
	release_mem_region(pci_resource_start(pci_dev, 0),
			   pci_resource_len(pci_dev, 0));
	pci_disable_device(pci_dev);
	kfree(dev);
}

/* driver_data is number of A/V channels */
static const struct pci_device_id tw686x_pci_tbl[] = {
	{PCI_DEVICE(0x1797, 0x6864), .driver_data = 4},
	/* not tested */
	{PCI_DEVICE(0x1797, 0x6865), .driver_data = 4 | TYPE_SECOND_GEN},
	/* TW6868 supports 8 A/V channels with an external TW2865 chip -
	   not supported by the driver */
	{PCI_DEVICE(0x1797, 0x6868), .driver_data = 4}, /* not tested */
	{PCI_DEVICE(0x1797, 0x6869), .driver_data = 8 | TYPE_SECOND_GEN},
	{}
};

static struct pci_driver tw686x_pci_driver = {
	.name = "TW686x",
	.id_table = tw686x_pci_tbl,
	.probe = tw686x_probe,
	.remove = tw686x_remove,
};

MODULE_DESCRIPTION("Driver for video frame grabber cards based on Intersil/Techwell TW686[4589]");
MODULE_AUTHOR("Krzysztof Halasa");
MODULE_LICENSE("GPL v2");
MODULE_DEVICE_TABLE(pci, tw686x_pci_tbl);
module_pci_driver(tw686x_pci_driver);

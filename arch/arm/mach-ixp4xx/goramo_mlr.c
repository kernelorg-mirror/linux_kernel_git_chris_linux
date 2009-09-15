/*
 * Goramo MultiLink router platform code
 * Copyright (C) 2006-2009 Krzysztof Halasa <khc@pm.waw.pl>
 */

#include <linux/delay.h>
#include <linux/hdlc.h>
#include <linux/i2c-gpio.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/serial_8250.h>
#include <asm-generic/rtc.h>
#include <asm/mach-types.h>
#include <asm/system.h>
#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <asm/mach/pci.h>

#define DEBUG_PCI 0
#define DEBUG_MSR 0
#define DEBUG_IRQ 0

#define SLOT_CS5536		0x01	/* IDSEL = AD31 */
#define  DEV_CS5536_SB		0
#define  DEV_CS5536_OHCI	1
#define  DEV_CS5536_EHCI	2
#define  DEV_CS5536_IDE		3
#define SLOT_ETHA		0x0B	/* IDSEL = AD21 */
#define SLOT_ETHB		0x0C	/* IDSEL = AD20 */
#define SLOT_MPCI		0x0D	/* IDSEL = AD19 */
#define SLOT_NEC		0x0E	/* IDSEL = AD18 */

/* GPIO lines */
#define GPIO_SCL		0
#define GPIO_SDA		1
#define GPIO_STR		2
#define GPIO_IRQ_NEC_CS5536	3
#define GPIO_IRQ_ETHA		4
#define GPIO_IRQ_ETHB		5
#define GPIO_HSS0_DCD_N		6
#define GPIO_HSS1_DCD_N		7
#define GPIO_UART0_DCD		8
#define GPIO_UART1_DCD		9
#define GPIO_HSS0_CTS_N		10
#define GPIO_HSS1_CTS_N		11
#define GPIO_IRQ_MPCI		12
#define GPIO_HSS1_RTS_N		13
#define GPIO_HSS0_RTS_N		14
/* GPIO15 is not connected */

/* Control outputs from 74HC4094 */
#define CONTROL_HSS0_CLK_INT	0
#define CONTROL_HSS1_CLK_INT	1
#define CONTROL_HSS0_DTR_N	2
#define CONTROL_HSS1_DTR_N	3
#define CONTROL_EXT		4
#define CONTROL_AUTO_RESET	5
#define CONTROL_PCI_RESET_N	6
#define CONTROL_EEPROM_WC_N	7

/* offsets from start of flash ROM = 0x50000000 */
#define CFG_ETH0_ADDRESS	0x40 /* 6 bytes */
#define CFG_ETH1_ADDRESS	0x46 /* 6 bytes */
#define CFG_REV			0x4C /* u32 */
#define  CFG_REV_MULTILINK	1
#define  CFG_REV_MICRO		2
#define  CFG_REV_MULTILINK2	3
#define CFG_SDRAM_SIZE		0x50 /* u32 */
#define CFG_SDRAM_CONF		0x54 /* u32 */
#define CFG_SDRAM_MODE		0x58 /* u32 */
#define CFG_SDRAM_REFRESH	0x5C /* u32 */

#define CFG_HW_BITS		0x60 /* u32 */
#define  CFG_HW_USB_PORTS	0x00000007 /* 0 = no chip, 1-5 = ports # */
#define  CFG_HW_HAS_PCI_SLOT	0x00000008
#define  CFG_HW_HAS_ETH0	0x00000010
#define  CFG_HW_HAS_ETH1	0x00000020
#define  CFG_HW_HAS_HSS0	0x00000040
#define  CFG_HW_HAS_HSS1	0x00000080
#define  CFG_HW_HAS_UART0	0x00000100
#define  CFG_HW_HAS_UART1	0x00000200
#define  CFG_HW_HAS_EEPROM	0x00000400
#define  CFG_HW_HAS_IDE		0x00000800
#define  CFG_HW_HAS_RTC		0x00001000

#define FLASH_CMD_READ_ARRAY	0xFF
#define FLASH_CMD_READ_ID	0x90
#define FLASH_SER_OFF		0x102 /* 0x81 in 16-bit mode */

#define CS5536_ADDRESS		(1 << (32 - SLOT_CS5536))

/* Use IRQ numbers normally used by GPIO lines which are used as outputs */
#define IRQ_CS5536_IDE		IRQ_IXP4XX_GPIO0 /* CS5536 IRQ 14 - hardwired */
#define IRQ_CS5536_USB		IRQ_IXP4XX_GPIO1 /* CS5536 IRQ 15 - IRQ mapper*/
#define IRQ_CS5536_CPU_BASE	IRQ_CS5536_IDE
#define IRQ_CS5536_SLAVE_PIC_BASE (14 - 8) /* CS5536 IRQs used: 14 and 15 */

#define RTC_CENTURY		14 /* offset in CMOS RAM space */

static u32 hw_bits = 0xFFFFFFFD;    /* assume all hardware present */;
static u8 control_value;

static inline int has_nec(void)
{
	return (system_rev == CFG_REV_MULTILINK) &&
		(hw_bits & CFG_HW_USB_PORTS);
}

static inline int has_cs5536(void)
{
	return (system_rev == CFG_REV_MULTILINK2) &&
		(hw_bits & (CFG_HW_USB_PORTS | CFG_HW_HAS_IDE |
			    CFG_HW_HAS_RTC));
}

static inline int has_pci(void)
{
	return has_nec() || has_cs5536() || (hw_bits & CFG_HW_HAS_PCI_SLOT);
}

static void set_scl(u8 value)
{
	gpio_line_set(GPIO_SCL, !!value);
	udelay(3);
}

static void set_sda(u8 value)
{
	gpio_line_set(GPIO_SDA, !!value);
	udelay(3);
}

static void set_str(u8 value)
{
	gpio_line_set(GPIO_STR, !!value);
	udelay(3);
}

static inline void set_control(int line, int value)
{
	if (value)
		control_value |=  (1 << line);
	else
		control_value &= ~(1 << line);
}


static void output_control(void)
{
	int i;

	gpio_line_config(GPIO_SCL, IXP4XX_GPIO_OUT);
	gpio_line_config(GPIO_SDA, IXP4XX_GPIO_OUT);

	for (i = 0; i < 8; i++) {
		set_scl(0);
		set_sda(control_value & (0x80 >> i)); /* MSB first */
		set_scl(1);	/* active edge */
	}

	set_str(1);
	set_str(0);

	set_scl(0);
	set_sda(1);		/* Be ready for START */
	set_scl(1);
}


static void (*set_carrier_cb_tab[2])(void *pdev, int carrier);

static int hss_set_clock(int port, unsigned int clock_type)
{
	int ctrl_int = port ? CONTROL_HSS1_CLK_INT : CONTROL_HSS0_CLK_INT;

	if (clock_type == CLOCK_DEFAULT)
		clock_type = CLOCK_EXT;

	switch (clock_type & CLOCK_TYPE_MASK) {
	case CLOCK_EXT:
		set_control(ctrl_int, 0);
		output_control();
		return clock_type;

	case CLOCK_INT:
		set_control(ctrl_int, 1);
		output_control();
		return clock_type;

	default:
		return -EINVAL;
	}
}

static irqreturn_t hss_dcd_irq(int irq, void *pdev)
{
	int i, port = (irq == IXP4XX_GPIO_IRQ(GPIO_HSS1_DCD_N));
	gpio_line_get(port ? GPIO_HSS1_DCD_N : GPIO_HSS0_DCD_N, &i);
	set_carrier_cb_tab[port](pdev, !i);
	return IRQ_HANDLED;
}


static int hss_open(int port, void *pdev,
		    void (*set_carrier_cb)(void *pdev, int carrier))
{
	int i, irq;

	if (!port)
		irq = IXP4XX_GPIO_IRQ(GPIO_HSS0_DCD_N);
	else
		irq = IXP4XX_GPIO_IRQ(GPIO_HSS1_DCD_N);

	gpio_line_get(port ? GPIO_HSS1_DCD_N : GPIO_HSS0_DCD_N, &i);
	set_carrier_cb(pdev, !i);

	set_carrier_cb_tab[!!port] = set_carrier_cb;

	if ((i = request_irq(irq, hss_dcd_irq, 0, "IXP4xx HSS", pdev)) != 0) {
		printk(KERN_ERR "ixp4xx_hss: failed to request IRQ%i (%i)\n",
		       irq, i);
		return i;
	}

	set_control(port ? CONTROL_HSS1_DTR_N : CONTROL_HSS0_DTR_N, 0);
	output_control();
	gpio_line_set(port ? GPIO_HSS1_RTS_N : GPIO_HSS0_RTS_N, 0);
	return 0;
}

static void hss_close(int port, void *pdev)
{
	free_irq(port ? IXP4XX_GPIO_IRQ(GPIO_HSS1_DCD_N) :
		 IXP4XX_GPIO_IRQ(GPIO_HSS0_DCD_N), pdev);
	set_carrier_cb_tab[!!port] = NULL; /* catch bugs */

	set_control(port ? CONTROL_HSS1_DTR_N : CONTROL_HSS0_DTR_N, 1);
	output_control();
	gpio_line_set(port ? GPIO_HSS1_RTS_N : GPIO_HSS0_RTS_N, 1);
}


/* Flash memory */
static struct flash_platform_data flash_data = {
	.map_name	= "cfi_probe",
	.width		= 2,
};

static struct resource flash_resource = {
	.flags		= IORESOURCE_MEM,
};

static struct platform_device device_flash = {
	.name		= "IXP4XX-Flash",
	.id		= 0,
	.dev		= { .platform_data = &flash_data },
	.num_resources	= 1,
	.resource	= &flash_resource,
};


/* I^2C interface */
static struct i2c_gpio_platform_data i2c_data = {
	.sda_pin	= GPIO_SDA,
	.scl_pin	= GPIO_SCL,
};

static struct platform_device device_i2c = {
	.name		= "i2c-gpio",
	.id		= 0,
	.dev		= { .platform_data = &i2c_data },
};


/* IXP425 2 UART ports */
static struct resource uart_resources[] = {
	{
		.start		= IXP4XX_UART1_BASE_PHYS,
		.end		= IXP4XX_UART1_BASE_PHYS + 0x0fff,
		.flags		= IORESOURCE_MEM,
	},
	{
		.start		= IXP4XX_UART2_BASE_PHYS,
		.end		= IXP4XX_UART2_BASE_PHYS + 0x0fff,
		.flags		= IORESOURCE_MEM,
	}
};

static struct plat_serial8250_port uart_data[] = {
	{
		.mapbase	= IXP4XX_UART1_BASE_PHYS,
		.membase	= (char __iomem *)IXP4XX_UART1_BASE_VIRT +
			REG_OFFSET,
		.irq		= IRQ_IXP4XX_UART1,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= IXP4XX_UART_XTAL,
	},
	{
		.mapbase	= IXP4XX_UART2_BASE_PHYS,
		.membase	= (char __iomem *)IXP4XX_UART2_BASE_VIRT +
			REG_OFFSET,
		.irq		= IRQ_IXP4XX_UART2,
		.flags		= UPF_BOOT_AUTOCONF | UPF_SKIP_TEST,
		.iotype		= UPIO_MEM,
		.regshift	= 2,
		.uartclk	= IXP4XX_UART_XTAL,
	},
	{ },
};

static struct platform_device device_uarts = {
	.name			= "serial8250",
	.id			= PLAT8250_DEV_PLATFORM,
	.dev.platform_data	= uart_data,
	.num_resources		= 2,
	.resource		= uart_resources,
};


/* Built-in 10/100 Ethernet MAC interfaces */
static struct eth_plat_info eth_plat[] = {
	{
		.phy		= 0,
		.rxq		= 3,
		.txreadyq	= 32,
	}, {
		.phy		= 1,
		.rxq		= 4,
		.txreadyq	= 33,
	}
};

static struct platform_device device_eth_tab[] = {
	{
		.name			= "ixp4xx_eth",
		.id			= IXP4XX_ETH_NPEB,
		.dev.platform_data	= eth_plat,
	}, {
		.name			= "ixp4xx_eth",
		.id			= IXP4XX_ETH_NPEC,
		.dev.platform_data	= eth_plat + 1,
	}
};


/* IXP425 2 synchronous serial ports */
static struct hss_plat_info hss_plat[] = {
	{
		.set_clock	= hss_set_clock,
		.open		= hss_open,
		.close		= hss_close,
		.txreadyq	= 34,
	}, {
		.set_clock	= hss_set_clock,
		.open		= hss_open,
		.close		= hss_close,
		.txreadyq	= 35,
	}
};

static struct platform_device device_hss_tab[] = {
	{
		.name			= "ixp4xx_hss",
		.id			= 0,
		.dev.platform_data	= hss_plat,
	}, {
		.name			= "ixp4xx_hss",
		.id			= 1,
		.dev.platform_data	= hss_plat + 1,
	}
};

/* CS5536 battery-backed RTC */
static struct cmos_rtc_board_info rtc_plat = {
	.rtc_century   = RTC_CENTURY,
};

static struct resource rtc_resource = {
	.start = 0x70,
	.end   = 0x71,
	.flags = IORESOURCE_IO
};

static struct platform_device device_rtc = {
	.name              = "rtc_cmos",
	.num_resources     = 1,
	.resource          = &rtc_resource,
	.dev.platform_data = &rtc_plat,
};


static struct platform_device *device_tab[7] __initdata = {
	&device_flash,		/* index 0 */
};

static inline u8 __init flash_readb(u8 __iomem *flash, u32 addr)
{
#ifdef __ARMEB__
	return __raw_readb(flash + addr);
#else
	return __raw_readb(flash + (addr ^ 3));
#endif
}

static inline u16 __init flash_readw(u8 __iomem *flash, u32 addr)
{
#ifdef __ARMEB__
	return __raw_readw(flash + addr);
#else
	return __raw_readw(flash + (addr ^ 2));
#endif
}

static void __init gmlr_init(void)
{
	u8 __iomem *flash;
	int i, devices = 1; /* flash */

	ixp4xx_sys_init();

	if ((flash = ioremap(IXP4XX_EXP_BUS_BASE_PHYS, 0x80)) == NULL)
		printk(KERN_ERR "goramo-mlr: unable to access system"
		       " configuration data\n");
	else {
		system_rev = __raw_readl(flash + CFG_REV);
		hw_bits = __raw_readl(flash + CFG_HW_BITS);

		for (i = 0; i < ETH_ALEN; i++) {
			eth_plat[0].hwaddr[i] =
				flash_readb(flash, CFG_ETH0_ADDRESS + i);
			eth_plat[1].hwaddr[i] =
				flash_readb(flash, CFG_ETH1_ADDRESS + i);
		}

		__raw_writew(FLASH_CMD_READ_ID, flash);
		system_serial_high = flash_readw(flash, FLASH_SER_OFF);
		system_serial_high <<= 16;
		system_serial_high |= flash_readw(flash, FLASH_SER_OFF + 2);
		system_serial_low = flash_readw(flash, FLASH_SER_OFF + 4);
		system_serial_low <<= 16;
		system_serial_low |= flash_readw(flash, FLASH_SER_OFF + 6);
		__raw_writew(FLASH_CMD_READ_ARRAY, flash);

		iounmap(flash);
	}

	switch (hw_bits & (CFG_HW_HAS_UART0 | CFG_HW_HAS_UART1)) {
	case CFG_HW_HAS_UART0:
		memset(&uart_data[1], 0, sizeof(uart_data[1]));
		device_uarts.num_resources = 1;
		break;

	case CFG_HW_HAS_UART1:
		device_uarts.dev.platform_data = &uart_data[1];
		device_uarts.resource = &uart_resources[1];
		device_uarts.num_resources = 1;
		break;
	}
	if (hw_bits & (CFG_HW_HAS_UART0 | CFG_HW_HAS_UART1))
		device_tab[devices++] = &device_uarts; /* max index 1 */

	if (hw_bits & CFG_HW_HAS_ETH0)
		device_tab[devices++] = &device_eth_tab[0]; /* max index 2 */
	if (hw_bits & CFG_HW_HAS_ETH1)
		device_tab[devices++] = &device_eth_tab[1]; /* max index 3 */

	if (hw_bits & CFG_HW_HAS_HSS0)
		device_tab[devices++] = &device_hss_tab[0]; /* max index 4 */
	if (hw_bits & CFG_HW_HAS_HSS1)
		device_tab[devices++] = &device_hss_tab[1]; /* max index 5 */

	if (hw_bits & CFG_HW_HAS_EEPROM)
		device_tab[devices++] = &device_i2c; /* max index 6 */

	if (hw_bits & CFG_HW_HAS_RTC)
		device_tab[devices++] = &device_rtc; /* max index 7 */

	gpio_line_config(GPIO_SCL, IXP4XX_GPIO_OUT);
	gpio_line_config(GPIO_SDA, IXP4XX_GPIO_OUT);
	gpio_line_config(GPIO_STR, IXP4XX_GPIO_OUT);
	gpio_line_config(GPIO_HSS0_RTS_N, IXP4XX_GPIO_OUT);
	gpio_line_config(GPIO_HSS1_RTS_N, IXP4XX_GPIO_OUT);
	gpio_line_config(GPIO_HSS0_DCD_N, IXP4XX_GPIO_IN);
	gpio_line_config(GPIO_HSS1_DCD_N, IXP4XX_GPIO_IN);
	set_irq_type(IXP4XX_GPIO_IRQ(GPIO_HSS0_DCD_N), IRQ_TYPE_EDGE_BOTH);
	set_irq_type(IXP4XX_GPIO_IRQ(GPIO_HSS1_DCD_N), IRQ_TYPE_EDGE_BOTH);

	set_control(CONTROL_HSS0_DTR_N, 1);
	set_control(CONTROL_HSS1_DTR_N, 1);
	set_control(CONTROL_EEPROM_WC_N, 1);
	set_control(CONTROL_PCI_RESET_N, 0);
	output_control();

	msleep(1);

	set_control(CONTROL_PCI_RESET_N, 1);
	output_control();

	msleep(100);	      /* Wait for PCI devices to initialize */

	flash_resource.start = IXP4XX_EXP_BUS_BASE(0);
	flash_resource.end = IXP4XX_EXP_BUS_BASE(0) + ixp4xx_exp_bus_size - 1;

	platform_add_devices(device_tab, devices);
}


#ifdef CONFIG_PCI
union pci_config_space { /* little-endian */
	struct {
		__le16 vendor_id, device_id;
		__le16 command, status;
		u8 revision, class[3];
		u8 cacheline, latency, header, bist;
		__le32 bars[6];
		__le32 cis_pointer;
		__le16 sub_vendor_id, sub_device_id;
		__le32 rom_bar;
		u8 caps_pointer, res[7];
		u8 irq, irq_pin, min_gnt, max_lat;
		__le32 r40, r44, r48, r4c, r50;
	} regs;
	u8 regs8[0];
	__le16 regs16[0];
	__le32 regs32[0];
};

struct cs5536_pci_device {
	union pci_config_space data;
	union pci_config_space mask; /* 0 = read-only, 1 = read/write */
};

static struct cs5536_pci_device sb = {
	.data.regs.vendor_id     = ~0,
	.data.regs.device_id     = ~0,
	.data.regs.class         = {0, 0x80, 6},
	.data.regs.header        = 0x80,
	/* mask - R/O */
};

static struct cs5536_pci_device ohci = {
	.data.regs.vendor_id     = cpu_to_le16(PCI_VENDOR_ID_AMD),
	.data.regs.device_id     = cpu_to_le16(PCI_DEVICE_ID_AMD_CS5536_OHC),
	.data.regs.command       = cpu_to_le16(0x0006),
	.data.regs.status        = cpu_to_le16(0x0230),
	.data.regs.class         = {0x10, 0x03, 0x0C},
	.data.regs.sub_vendor_id = cpu_to_le16(PCI_VENDOR_ID_AMD),
	.data.regs.sub_device_id = cpu_to_le16(PCI_DEVICE_ID_AMD_CS5536_OHC),
	.data.regs.caps_pointer  = 0x40,
	.data.regs.irq_pin       = 1,
	.data.regs.r40           = cpu_to_le32(0xC8020001), /* Capabilities */

	.mask.regs.command       = ~0,
	.mask.regs.cacheline     = ~0,
	.mask.regs.latency       = ~0,
	.mask.regs.bars[0]       = cpu_to_le32(0xFFFFF000), /* 4 KB (P2D desc)*/
	.mask.regs.irq           = ~0,
};

static struct cs5536_pci_device ehci = {
	.data.regs.vendor_id     = cpu_to_le16(PCI_VENDOR_ID_AMD),
	.data.regs.device_id     = cpu_to_le16(PCI_DEVICE_ID_AMD_CS5536_EHC),
	.data.regs.command       = cpu_to_le16(0x0006),
	.data.regs.status        = cpu_to_le16(0x0230),
	.data.regs.class         = {0x20, 0x03, 0x0C},
	.data.regs.sub_vendor_id = cpu_to_le16(PCI_VENDOR_ID_AMD),
	.data.regs.sub_device_id = cpu_to_le16(PCI_DEVICE_ID_AMD_CS5536_EHC),
	.data.regs.caps_pointer  = 0x40,
	.data.regs.irq_pin       = 1,
	.data.regs.r40           = cpu_to_le32(0xC8020001), /* Capabilities */
	/* mask */
	.mask.regs.command       = ~0,
	.mask.regs.cacheline     = ~0,
	.mask.regs.latency       = ~0,
	.mask.regs.bars[0]       = cpu_to_le32(0xFFFFF000), /* 4 KB (P2D desc)*/
	.mask.regs.irq           = ~0,
};

static struct cs5536_pci_device ide = {
	.data.regs.vendor_id     = cpu_to_le16(PCI_VENDOR_ID_AMD),
	.data.regs.device_id     = cpu_to_le16(PCI_DEVICE_ID_AMD_CS5536_IDE),
	.data.regs.command       = cpu_to_le16(0x0006),
	.data.regs.status        = cpu_to_le16(0x0230),
	.data.regs.class         = {0x85, 0x01, 0x01},
	.data.regs.bars[4]       = cpu_to_le32(1), /* BM DMA registers */
	.data.regs.sub_vendor_id = cpu_to_le16(PCI_VENDOR_ID_AMD),
	.data.regs.sub_device_id = cpu_to_le16(PCI_DEVICE_ID_AMD_CS5536_IDE),
	.data.regs.irq_pin       = 2,
	/* mask */
	.mask.regs.command       = ~0,
	.mask.regs.cacheline     = ~0,
	.mask.regs.latency       = ~0,
	.mask.regs.bars[4]       = cpu_to_le32(0xFFFFFFF8), /* 8 bytes */
	.mask.regs.irq           = ~0,
	.mask.regs.r40           = ~0, /* IDE_CFG */
	.mask.regs.r48           = ~0, /* IDE_DTC */
	.mask.regs.r4c           = ~0, /* IDE_CAST */
	.mask.regs.r50           = ~0, /* IDE_ETC */
};

/*
 * Mask table, bits to mask for quantity of size 1, 2 or 4 bytes.
 * 0 and 3 are not valid indexes...
 */
static const u32 bytemask[] = {
	/*0*/	0,
	/*1*/	0xff,
	/*2*/	0xffff,
	/*3*/	0,
	/*4*/	0xffffffff,
};

static u8 cs5536_slave_irq_mask = 0xFF; /* IDE and USB only */

static u32 byte_lane_enable_bits(u32 n, int size)
{
	if (size == 1)
		return (0xF & ~BIT(n)) << 4;
	if (size == 2)
		return (0xF & ~(BIT(n) | BIT(n + 1))) << 4;
	if (size == 4)
		return 0;
	return 0xFFFFFFFF;
}

static u32 ixp4xx_config_addr(u8 bus_num, u16 devfn, int where)
{
	if (!bus_num)		/* type 0 */
		return BIT(32 - PCI_SLOT(devfn)) | ((PCI_FUNC(devfn)) << 8) |
			(where & ~3);
	else			/* type 1 */
		return (bus_num << 16) | ((PCI_SLOT(devfn)) << 11) |
			((PCI_FUNC(devfn)) << 8) | (where & ~3) | 1;
}

static u32 msr_id(u32 msr)
{
	return ((msr << 9) & 0xFF800000) | (msr & 0x3FFF);
}

static void read_msr(u32 msr, u32 *h, u32 *l)
{
	if (ixp4xx_pci_write(CS5536_ADDRESS + 0xF4, NP_CMD_CONFIGWRITE,
			     msr_id(msr)))
		goto error;
	if (ixp4xx_pci_read(CS5536_ADDRESS + 0xF8, NP_CMD_CONFIGREAD, l))
		goto error;
	if (ixp4xx_pci_read(CS5536_ADDRESS + 0xFC, NP_CMD_CONFIGREAD, h))
		goto error;
#if DEBUG_MSR
	printk(KERN_DEBUG "read_msr %08X: %08X %08X\n", msr, *h, *l);
#endif
	return;
error:
	printk(KERN_CRIT "read_msr(0x%08X) failed\n", msr);
}


static void write_msr(u32 msr, u32 h, u32 l)
{
	if (ixp4xx_pci_write(CS5536_ADDRESS + 0xF4, NP_CMD_CONFIGWRITE,
			     msr_id(msr)))
		goto error;
	if (ixp4xx_pci_write(CS5536_ADDRESS + 0xF8, NP_CMD_CONFIGWRITE, l))
		goto error;
	if (ixp4xx_pci_write(CS5536_ADDRESS + 0xFC, NP_CMD_CONFIGWRITE, h))
		goto error;
#if DEBUG_MSR
	printk(KERN_DEBUG "write_msr %08X: %08X %08X\n", msr, h, l);
#endif
	return;
error:
	printk(KERN_CRIT "write_msr(0x%08X, 0x%08X, 0x%08X) failed\n",
	       msr, h, l);
}


static inline void setup_cs5536_gpio(u16 value, u16 address)
{
	outl(((~(u32)value) << 16) | value, address);
}

static void cs5536_irq_ack(unsigned int irq)
{
#if DEBUG_IRQ
	printk(KERN_INFO "ACK %u GPIO %X\n",
	       irq, !!(readb(IXP4XX_GPIO_BASE_VIRT + 0x0B) & 8));
#endif
}

static void cs5536_irq_mask(unsigned int irq)
{
#if DEBUG_IRQ
	printk(KERN_INFO "MASK %u GPIO %X\n",
	       irq, !!(readb(IXP4XX_GPIO_BASE_VIRT + 0x0B) & 8));
#endif
	cs5536_slave_irq_mask |= 1 << (irq - IRQ_CS5536_CPU_BASE +
				       IRQ_CS5536_SLAVE_PIC_BASE);
	outb(cs5536_slave_irq_mask, 0xA1);
#if DEBUG_IRQ
	printk(KERN_INFO "        GPIO %X\n",
	       !!(readb(IXP4XX_GPIO_BASE_VIRT + 0x0B) & 8));
#endif
}

static void cs5536_irq_unmask(unsigned int irq)
{
#if DEBUG_IRQ
	printk(KERN_INFO "UNMASK %u GPIO %X\n",
	       irq, !!(readb(IXP4XX_GPIO_BASE_VIRT + 0x0B) & 8));
#endif
	cs5536_slave_irq_mask &= ~(1 << (irq - IRQ_CS5536_CPU_BASE +
					 IRQ_CS5536_SLAVE_PIC_BASE));
	outb(cs5536_slave_irq_mask, 0xA1);
#if DEBUG_IRQ
	printk(KERN_INFO "        GPIO %X\n",
	       !!(readb(IXP4XX_GPIO_BASE_VIRT + 0x0B) & 8));
#endif
}

static void cs5536_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	u32 h, l;

#if DEBUG_IRQ
	printk(KERN_INFO "HANDLER %u GPIO %X status %x\n",
	       irq, !!(readb(IXP4XX_GPIO_BASE_VIRT + 0x0B) & 8), desc->status);
#endif
	desc->chip->ack(irq);
	read_msr(0x51400027, &h, &l);
	if (l & 0x40000000) {
		struct irq_desc *d = irq_to_desc(IRQ_CS5536_USB);
#if DEBUG_IRQ
		printk(KERN_INFO "  status %X\n", d->status);
		//BUG_ON (d->status & IRQ_INPROGRESS);
#endif
		d->handle_irq(IRQ_CS5536_USB, d);
	}
	if (l & 0x01000000) {
		struct irq_desc *d = irq_to_desc(IRQ_CS5536_IDE);
#if DEBUG_IRQ
		printk(KERN_INFO "  status %X\n", d->status);
#endif
		d->handle_irq(IRQ_CS5536_IDE, d);
	}
}

static void __init gmlr_pci_preinit(void)
{
	gpio_line_config(GPIO_IRQ_ETHA, IXP4XX_GPIO_IN);
	gpio_line_config(GPIO_IRQ_ETHB, IXP4XX_GPIO_IN);
	gpio_line_config(GPIO_IRQ_NEC_CS5536, IXP4XX_GPIO_IN);
	gpio_line_config(GPIO_IRQ_MPCI, IXP4XX_GPIO_IN);
	set_irq_type(IXP4XX_GPIO_IRQ(GPIO_IRQ_ETHA), IRQ_TYPE_LEVEL_LOW);
	set_irq_type(IXP4XX_GPIO_IRQ(GPIO_IRQ_ETHB), IRQ_TYPE_LEVEL_LOW);
	set_irq_type(IXP4XX_GPIO_IRQ(GPIO_IRQ_NEC_CS5536), IRQ_TYPE_LEVEL_LOW);
	set_irq_type(IXP4XX_GPIO_IRQ(GPIO_IRQ_MPCI), IRQ_TYPE_LEVEL_LOW);
	ixp4xx_pci_preinit();
}

static struct irq_chip cs5536_irqchip = {
	.name = "CS5536",
	.ack = cs5536_irq_ack,
	.mask = cs5536_irq_mask,
	.unmask = cs5536_irq_unmask,
};

static void __init gmlr_pci_postinit(void)
{
	if (has_nec() && (hw_bits & CFG_HW_USB_PORTS) < 5) {
		/* need to adjust number of USB ports on NEC chip */
		u32 value, addr = BIT(32 - SLOT_NEC) | 0xE0;
		if (!ixp4xx_pci_read(addr, NP_CMD_CONFIGREAD, &value)) {
			value &= ~7;
			value |= hw_bits & CFG_HW_USB_PORTS;
			ixp4xx_pci_write(addr, NP_CMD_CONFIGWRITE, value);
		}
	}

	if (has_cs5536()) {
		struct pci_dev *pci_dev;
		u8 __iomem *ptr;
		u32 h, l;

/* GPIO */
		/* FIXME 0x2000 */
		write_msr(0x5140000C, 0xF001, 0x2000); /* GPIO at 0x2000 */
		write_msr(0x510100E2, 0x80000002, 0x000fff00); /* 256 bytes */
		/* GPIO1 = beeper (OUT AUX1)
		   GPIO2 = IDE IRQ (IN AUX1)
		   GPIO5 = IDE cable ID (IN) */
		setup_cs5536_gpio(0x0002, 0x2004); /* OUT enable */
		setup_cs5536_gpio(0x0002, 0x2010); /* OUT AUX1 */
		setup_cs5536_gpio(0x0000, 0x2014); /* OUT AUX2 */
		setup_cs5536_gpio(0xFFFB, 0x2018); /* pull-up enable */
		setup_cs5536_gpio(0x0004, 0x201C); /* pull-down enable */
		setup_cs5536_gpio(0x0024, 0x2020); /* IN enable */
		setup_cs5536_gpio(0x0004, 0x2034); /* IN AUX1 */
		setup_cs5536_gpio(0x0000, 0x20A0); /* IN enable */
		setup_cs5536_gpio(0x0000, 0x20B4); /* IN AUX1 */

/* USB */
		read_msr(0x51200000, &h, &l);
		ohci.data.regs.revision = ehci.data.regs.revision = l;

		/* map and set USB option registers */
		write_msr(0x5120000B, 2, PCIBIOS_MIN_MEM);
		write_msr(0x51010020, 0x40000000 | PCIBIOS_MIN_MEM >> 24,
			  (PCIBIOS_MIN_MEM << 8) | 0xFFFFF);
		if (!(ptr = ioremap(PCIBIOS_MIN_MEM, 0x80)))
			printk(KERN_CRIT "goramo-mlr: unable to access CS5536 "
			       "PCI address space\n");
		else {
			/* assign USB port #4 to USB host controller */
			writel((readl(ptr + 4) & ~3) | 2, ptr + 4);
			iounmap(ptr);
		}
		/* reset maps */
		write_msr(0x5120000B, 0, 0);
		write_msr(0x51010020, 0x000000FF, 0xFFF00000);

/* IDE */
		read_msr(0x51300000, &h, &l);
		ide.data.regs.revision = l;

		read_msr(0x51400015, &h, &l);
		write_msr(0x51400015, h, l | 1); /* IDE, not flash */

		ide.data.regs.bars[0] = cpu_to_le32(0x1F1);
		ide.data.regs.bars[1] = cpu_to_le32(0x3F7);
		pci_dev = pci_get_bus_and_slot(0, PCI_DEVFN(SLOT_CS5536,
							    DEV_CS5536_IDE));
		if (pci_dev) {
			pci_dev->resource[0].start = 0x1F0;
			pci_dev->resource[0].end = 0x1F7;
			pci_dev->resource[0].flags = IORESOURCE_IO |
				IORESOURCE_PCI_FIXED;
			pci_dev->resource[1].start = 0x3F6;
			pci_dev->resource[1].end = 0x3F6;
			pci_dev->resource[1].flags = IORESOURCE_IO |
				IORESOURCE_PCI_FIXED;
		}

		l = 2;		/* channel enabled */
		if (!(inl(0x2030) & 0x20))
			l |= 0x30000; /* assume 80-wire cable */
		write_msr(0x51300010, 0, l);
		ide.data.regs32[0x10] = cpu_to_le32(l);
		read_msr(0x51300012, &h, &l);
		ide.data.regs32[0x12] = cpu_to_le32(l);
		read_msr(0x51300013, &h, &l);
		ide.data.regs32[0x13] = cpu_to_le32(l);
		read_msr(0x51300014, &h, &l);
		ide.data.regs32[0x14] = cpu_to_le32(l);

/* RTC */
		write_msr(0x51400057, 0, RTC_CENTURY);
		outb(RTC_REG_D, 0x70);
		if (!(inb(0x71) & 0x80))
			printk(KERN_ERR "RTC: battery fault recorded\n");
		/* Make sure we do 24 hrs BCD */
		outb(RTC_REG_B, 0x70);
		l = inb(0x71);
		if ((l & (RTC_DM_BINARY | RTC_24H)) != RTC_24H)
			outb((l & ~RTC_DM_BINARY) | RTC_24H, 0x71);

/* Interrupts */
		/* initialize CS5536 dual 8259A IRQ controller */
		outb(0xFF, 0x21); /* mask all IRQs */
		outb(0xFF, 0xA1);
		outb(0x19, 0x20); /* ICW1 (master) level-triggered */
		outb(0x00, 0x21); /* ICW2 */
		outb(0x04, 0x21); /* ICW3 */
		outb(0x01, 0x21); /* ICW4 */
		outb(0x19, 0xA0); /* ICW1 (slave) level-triggered */
		outb(0x00, 0xA1); /* ICW2 */
		outb(0x02, 0xA1); /* ICW3 */
		outb(0x01, 0xA1); /* ICW4 */

		outb(0xFB, 0x21); /* mask all but cascade IRQ2 */
		outb(0xFF, 0xA1);

		write_msr(0x51400020, 0, 0xF00); /* USB uses IRQ15 (Y15) */
		write_msr(0x51000010, 0x44000030, 0x00000013); /* CIS mode C */

		set_irq_chip(IRQ_CS5536_IDE, &cs5536_irqchip);
		set_irq_handler(IRQ_CS5536_IDE, handle_level_irq);
		set_irq_flags(IRQ_CS5536_IDE, IRQF_VALID);
		irq_to_desc(IRQ_CS5536_IDE)->status |= IRQ_LEVEL;

		set_irq_chip(IRQ_CS5536_USB, &cs5536_irqchip);
		set_irq_handler(IRQ_CS5536_USB, handle_level_irq);
		set_irq_flags(IRQ_CS5536_USB, IRQF_VALID);
		irq_to_desc(IRQ_CS5536_USB)->status |= IRQ_LEVEL;

		irq_to_desc(IXP4XX_GPIO_IRQ(GPIO_IRQ_NEC_CS5536))->status |=
			IRQ_LEVEL;
		set_irq_chained_handler(IXP4XX_GPIO_IRQ(GPIO_IRQ_NEC_CS5536),
					cs5536_irq_handler);
	}
}

static int __init gmlr_pci_setup(int nr, struct pci_sys_data *sys)
{
	int res = ixp4xx_setup(nr, sys);
	if (res) {
		u32 v;
		ixp4xx_pci_write(0, NP_CMD_IOWRITE, CS5536_ADDRESS); /* IDSEL */
		ixp4xx_pci_read(CS5536_ADDRESS, NP_CMD_CONFIGREAD, &v);
		sb.data.regs32[0] = cpu_to_le32(v); /* vendor and device ID */
		ixp4xx_pci_read(CS5536_ADDRESS + 8, NP_CMD_CONFIGREAD, &v);
		sb.data.regs.revision = v;
	}
	return res;
}

static int __init gmlr_map_irq(struct pci_dev *dev, u8 slot, u8 pin)
{
	switch (slot) {
	case SLOT_CS5536:
		break;
	case SLOT_ETHA:
		return IXP4XX_GPIO_IRQ(GPIO_IRQ_ETHA);
	case SLOT_ETHB:
		return IXP4XX_GPIO_IRQ(GPIO_IRQ_ETHB);
	case SLOT_NEC:
		return IXP4XX_GPIO_IRQ(GPIO_IRQ_NEC_CS5536);
	default:
		return IXP4XX_GPIO_IRQ(GPIO_IRQ_MPCI);
	}

	switch (pin) {
	case 1:
		return IRQ_CS5536_USB;
	case 2:
		return IRQ_CS5536_IDE;
	default:
		return -1;
	}
}

static struct cs5536_pci_device* cs5536_get_dev(unsigned int devfn)
{
	if (PCI_FUNC(devfn) == DEV_CS5536_SB)
		return &sb;
	else if (PCI_FUNC(devfn) == DEV_CS5536_OHCI &&
		 (hw_bits & CFG_HW_USB_PORTS))
		return &ohci;
	else if (PCI_FUNC(devfn) == DEV_CS5536_EHCI &&
		 (hw_bits & CFG_HW_USB_PORTS))
		return &ehci;
	else if (PCI_FUNC(devfn) == DEV_CS5536_IDE &&
		 (hw_bits & CFG_HW_HAS_IDE))
		return &ide;
	else
		return NULL;
}

static int cs5536_pci_read(unsigned int devfn, int where, int len,
			   uint32_t *value)
{
	struct cs5536_pci_device *device;

	if (!(device = cs5536_get_dev(devfn)))
		return PCIBIOS_DEVICE_NOT_FOUND;

	if (where >= sizeof(device->data)) {
		*value = 0;
		return 0;	/* nothing there */
	}

	switch (len) {
	case 1:
		*value = device->data.regs8[where];
		break;
	case 2:
		*value = le16_to_cpu(device->data.regs16[where >> 1]);
		break;
	case 4:
		*value = le32_to_cpu(device->data.regs32[where >> 2]);
		break;
	default:
		BUG();
	}

#if DEBUG_PCI
	printk(KERN_INFO "cs5536_pci_read from %X size %X dev 0:%X:%X -> %X\n",
	       where, len, PCI_SLOT(devfn), PCI_FUNC(devfn), *value);
#endif
	return 0;
}

static int cs5536_pci_write(unsigned int devfn, int where, int len,
			    uint32_t value)
{
	struct cs5536_pci_device *device;
	__le32 mask;

	if (!(device = cs5536_get_dev(devfn)))
		return PCIBIOS_DEVICE_NOT_FOUND;
#if DEBUG_PCI
	printk(KERN_INFO "cs5536_pci_write to %X size %X value %X dev 0:"
	       "%X:%X\n", where, len, value, PCI_SLOT(devfn), PCI_FUNC(devfn));
#endif

	if (where >= sizeof(device->data))
		return 0;	/* nothing there */

	switch (len) {
	case 1:
		mask = device->mask.regs8[where];
		value &= mask;

		device->data.regs8[where] &= ~mask;
		device->data.regs8[where] |= value;
		break;
	case 2:
		where &= ~1;
		mask = device->mask.regs16[where >> 1]; /* little-endian */
		value &= le16_to_cpu(mask);

		device->data.regs16[where >> 1] &= ~mask;
		device->data.regs16[where >> 1] |= cpu_to_le16(value);
		break;
	case 4:
		where &= ~3;
		mask = device->mask.regs32[where >> 2]; /* little-endian */
		value &= le32_to_cpu(mask);

		device->data.regs32[where >> 2] &= ~mask;
		device->data.regs32[where >> 2] |= cpu_to_le32(value);
		break;
	default:
		BUG();
	}

	if (len == 4 && where == 0x10) { /* write to BAR0 */
		switch (PCI_FUNC(devfn)) {
		case DEV_CS5536_OHCI:
			/* USB OHCI base address MSR */
			write_msr(0x51200008, 6, value);
			/* P2D descriptor for USB OHCI */
			write_msr(0x51010020, 0x40000000 | value >> 24,
				  (value << 8) | 0xFFFFF);
			break;

		case DEV_CS5536_EHCI:
			/* USB EHCI base address MSR */
			write_msr(0x51200009, 0x2006, value);
			/* P2D descriptor for USB EHCI */
			write_msr(0x51010021, 0x40000000 | value >> 24,
				  (value << 8) | 0xFFFFF);
			break;
		}
		return 0;
	}

	if (PCI_FUNC(devfn) == DEV_CS5536_IDE && len == 4)
		switch (where) {
		case 0x20: /* BAR4 */
			/* Bus mastering IDE base address MSR - 20-bit */
			write_msr(0x51300008, 0, value);
			/* IOD descriptor for IDE */
			write_msr(0x510100E1, 0x60000000 | value >> 12,
				  (value << 20) | 0xFFFF8);
			break;
		case 0x40:
			write_msr(0x51300010, 0, value);
			break;
		case 0x48:
			write_msr(0x51300012, 0, value);
			break;
		case 0x4C:
			write_msr(0x51300013, 0, value);
			break;
		case 0x50:
			write_msr(0x51300014, 0, value);
			break;
		}

	return 0;
}

static int gmlr_pci_read_config(struct pci_bus *bus, unsigned int devfn,
				int where, int size, u32 *value)
{
	u32 n, byte_enables, addr, data;
	u8 bus_num = bus->number;

	if (bus_num == 0 && PCI_SLOT(devfn) == SLOT_CS5536 &&
	    (PCI_FUNC(devfn) != DEV_CS5536_SB || where < 0x10))
		return cs5536_pci_read(devfn, where, size, value);

	*value = 0xFFFFFFFF;
	n = where % 4;
	byte_enables = byte_lane_enable_bits(n, size);
	if (byte_enables == 0xffffffff)
		return PCIBIOS_BAD_REGISTER_NUMBER;

	addr = ixp4xx_config_addr(bus_num, devfn, where);
	if (ixp4xx_pci_read(addr, byte_enables | NP_CMD_CONFIGREAD, &data))
		return PCIBIOS_DEVICE_NOT_FOUND;

	*value = (data >> (8 * n)) & bytemask[size];
	return PCIBIOS_SUCCESSFUL;
}

static int gmlr_pci_write_config(struct pci_bus *bus, unsigned int devfn,
				 int where, int size, u32 value)
{
	u32 n, byte_enables, addr, data;
	u8 bus_num = bus->number;

	if (bus_num == 0 && PCI_SLOT(devfn) == SLOT_CS5536 &&
	    (PCI_FUNC(devfn) != DEV_CS5536_SB || where < 0x10))
		return cs5536_pci_write(devfn, where, size, value);

	n = where % 4;
	byte_enables = byte_lane_enable_bits(n, size);
	if (byte_enables == 0xFFFFFFFF)
		return PCIBIOS_BAD_REGISTER_NUMBER;

	addr = ixp4xx_config_addr(bus_num, devfn, where);
	data = value << (8 * n);
	if (ixp4xx_pci_write(addr, byte_enables | NP_CMD_CONFIGWRITE, data))
		return PCIBIOS_DEVICE_NOT_FOUND;

	return PCIBIOS_SUCCESSFUL;
}

struct pci_ops gmlr_ops = {
	.read =  gmlr_pci_read_config,
	.write = gmlr_pci_write_config,
};

struct pci_bus *gmlr_scan_bus(int nr, struct pci_sys_data *sys)
{
	return pci_scan_bus(sys->busnr, &gmlr_ops, sys);
}

static struct hw_pci gmlr_hw_pci __initdata = {
	.nr_controllers = 1,
	.preinit	= gmlr_pci_preinit,
	.postinit	= gmlr_pci_postinit,
	.swizzle	= pci_std_swizzle,
	.setup		= gmlr_pci_setup,
	.scan		= gmlr_scan_bus,
	.map_irq	= gmlr_map_irq,
};

static int __init gmlr_pci_init(void)
{
	if (machine_is_goramo_mlr() && has_pci())
		pci_common_init(&gmlr_hw_pci);
	return 0;
}

subsys_initcall(gmlr_pci_init);
#endif /* CONFIG_PCI */


MACHINE_START(GORAMO_MLR, "MultiLink")
	/* Maintainer: Krzysztof Halasa */
	.phys_io	= IXP4XX_PERIPHERAL_BASE_PHYS,
	.io_pg_offst	= ((IXP4XX_PERIPHERAL_BASE_VIRT) >> 18) & 0xFFFC,
	.map_io		= ixp4xx_map_io,
	.init_irq	= ixp4xx_init_irq,
	.timer		= &ixp4xx_timer,
	.boot_params	= 0x0100,
	.init_machine	= gmlr_init,
MACHINE_END

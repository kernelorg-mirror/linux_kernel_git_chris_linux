/*
 * arch/arm/mach-ixp4xx/include/mach/hardware.h 
 *
 * Copyright (C) 2002 Intel Corporation.
 * Copyright (C) 2003-2004 MontaVista Software, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

/*
 * Hardware definitions for IXP4xx based systems
 */

#ifndef __ASM_ARCH_HARDWARE_H__
#define __ASM_ARCH_HARDWARE_H__

#ifdef CONFIG_IXP4XX_INDIRECT_PCI
#define PCIBIOS_MAX_MEM		0x4FFFFFFF
#else
#define PCIBIOS_MAX_MEM		0x4BFFFFFF
#endif

/* Register locations and bits */
#include "ixp4xx-regs.h"

#ifdef CONFIG_CPU_LITTLE_ENDIAN_DATA_COHERENT
#define MEM_VALUE_COHERENT_BASE_VIRT 0xFFA00000
#define MEM_VALUE_COHERENT_ADDR_MASK 0xFFE00000 /* 2 MB */
#endif

#ifndef __ASSEMBLER__
#include <mach/cpu.h>
#endif

/* Platform helper functions and definitions */
#include "platform.h"

#endif  /* _ASM_ARCH_HARDWARE_H */

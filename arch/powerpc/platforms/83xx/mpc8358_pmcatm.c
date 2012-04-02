/*
 * Copyright (C) 2012 Beyond Electronics Corp. All rights reserved.
 *
 * Author: Bill Farrow <bfarrow@beyondelectronics.us>
 *
 * Based on the file mpc836x_mds.c
 *   Copyright (C) Freescale Semicondutor, Inc. 2006. All rights reserved.
 *
 * Description:
 * BEC PMC ATM board (MPC8358) specific routines.
 *
 * Changelog:
 * Feb 20, 2012	Initial version
 *
 * This program is free software; you can redistribute it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/stddef.h>
#include <linux/kernel.h>
#include <linux/compiler.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/reboot.h>
#include <linux/pci.h>
#include <linux/kdev_t.h>
#include <linux/major.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/seq_file.h>
#include <linux/root_dev.h>
#include <linux/initrd.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>

#include <asm/system.h>
#include <linux/atomic.h>
#include <asm/time.h>
#include <asm/io.h>
#include <asm/machdep.h>
#include <asm/ipic.h>
#include <asm/irq.h>
#include <asm/prom.h>
#include <asm/udbg.h>
#include <sysdev/fsl_soc.h>
#include <sysdev/fsl_pci.h>
#include <sysdev/simple_gpio.h>
#include <asm/qe.h>
#include <asm/qe_ic.h>

#include "mpc83xx.h"

#undef DEBUG
#ifdef DEBUG
#define DBG(fmt...) udbg_printf(fmt)
#else
#define DBG(fmt...)
#endif

/* ************************************************************************
 *
 * Setup the architecture
 *
 */
static void __init mpc8358_pmcatm_setup_arch(void)
{
	struct device_node *np;

	if (ppc_md.progress)
		ppc_md.progress("mpc8358_pmcatm_setup_arch()", 0);

	mpc83xx_setup_pci();

#ifdef CONFIG_QUICC_ENGINE
	qe_reset();

	if ((np = of_find_node_by_name(NULL, "par_io")) != NULL) {
		par_io_init(np);
		of_node_put(np);

		for (np = NULL; (np = of_find_node_by_name(np, "ucc")) != NULL;)
			par_io_of_config(np);
	}

	if ((np = of_find_compatible_node(NULL, "network", "ucc_geth"))
			!= NULL){
		uint svid;

		/* Reset the Ethernet PHY */
//		par_io_data_set(1, 12, 1);	/* Reset Ethernet PHY */
//		par_io_data_set(1, 10, 0);	/* Take the PHY out of coma state COMA_IN=0  */
//		par_io_data_set(1, 11, 0);	/* Enable the 125MHz PHY clock KILL_125_IN=0 */
//		udelay(1000);
//		par_io_data_set(1, 12, 0);	/* Enable Ethernet PHY */

		/* handle mpc8360ea rev.2.1 erratum 2: RGMII Timing */
		svid = mfspr(SPRN_SVR);
		if (svid == 0x80480021) {
			void __iomem *immap;

			immap = ioremap(get_immrbase() + 0x14a8, 8);

			/*
			 * IMMR + 0x14A8[4:5] = 11 (clk delay for UCC 2)
			 * IMMR + 0x14A8[18:19] = 11 (clk delay for UCC 1)
			 */
			setbits32(immap, 0x0c003000);

			/*
			 * IMMR + 0x14AC[20:27] = 10101010
			 * (data delay for both UCC's)
			 */
			clrsetbits_be32(immap + 4, 0xff0, 0xaa0);

			iounmap(immap);
		}

		of_node_put(np);
	}

	if ((np = of_find_compatible_node(NULL, "upc", "fsl,qe_upc")) != NULL)
	{
		//if (qe_clock_is_brg(clk))
		//	qe_setbrg(clk, rate, 1);
		qe_setbrg(QE_BRG5, 48000000, 1);
		qe_setbrg(QE_BRG6, 48000000, 1);

		of_node_put(np);
	}

#endif				/* CONFIG_QUICC_ENGINE */
}

machine_device_initcall(mpc8358_pmcatm, mpc83xx_declare_of_platform_devices);

/*
 * Called very early, MMU is off, device-tree isn't unflattened
 */
static int __init mpc8358_pmcatm_probe(void)
{
        unsigned long root = of_get_flat_dt_root();

        return of_flat_dt_is_compatible(root, "MPC8358PMCATM");
}

define_machine(mpc8358_pmcatm) {
	.name		= "MPC8358 PMC-ATM",
	.probe		= mpc8358_pmcatm_probe,
	.setup_arch	= mpc8358_pmcatm_setup_arch,
	.init_IRQ	= mpc83xx_ipic_and_qe_init_IRQ,
	.get_irq	= ipic_get_irq,
	.restart	= mpc83xx_restart,
	.time_init	= mpc83xx_time_init,
	.calibrate_decr	= generic_calibrate_decr,
	.progress	= udbg_progress,
};

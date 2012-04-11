/*
 * Copyright (C) 2006-2007 Freescale Semiconductor, Inc. All rights reserved.
 *
 * Author: Dave Liu (daveliu@freescale.com)
 *         Tony Li (tony.li@freescale.com)
 *
 * Descrption:
 * suni5384 phy driver specific for fua atm driver
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/atmdev.h>
#include <linux/sonet.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/capability.h>
#include <linux/atm_suni.h>

#include <asm/system.h>
#include <asm/prom.h>
#include <asm/param.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>

#include "pm5384.h"

#if 1
#define DPRINTK(format,args...) printk(KERN_DEBUG format,##args)
#else
#define DPRINTK(format,args...)
#endif


struct suni_priv {
	int phy_id;
	int irq;
	void __iomem *phy_base;	/* register address of pm5384 */
	int loop_mode;			/* loopback mode */
	int port_width;
	struct atm_dev *dev;		/* device back-pointer */
	struct suni_priv *next;		/* next SUNI */
	struct k_sonet_stats sonet_stats; /* link diagnostics */
};

static unsigned char phy_get(void __iomem *reg)
{
	return in_8(reg);
}

static void phy_put(void __iomem *reg, unsigned char val)
{
	out_8(reg, val);
}

#define PRIV(dev) ((struct suni_priv *) dev->phy_data)

#define PUT(val,reg) phy_put(PRIV(dev)->phy_base + SUNI_##reg, val)
#define GET(reg) phy_get(PRIV(dev)->phy_base + SUNI_##reg)
#define REG_CHANGE(mask,shift,value,reg) \
	PUT((GET(reg) & ~(mask)) | ((value) << (shift)),reg)


static struct timer_list poll_timer;
static struct suni_priv *sunis = NULL;
DEFINE_SPINLOCK(sunis_lock);


#define ADD_LIMITED(s,v) \
	atomic_add((v),&stats->s); \
	if (atomic_read(&stats->s) < 0) atomic_set(&stats->s,INT_MAX);


static void suni_hz(unsigned long from_timer)
{
	struct suni_priv *walk;
	struct atm_dev *dev;
	struct k_sonet_stats *stats;

	for (walk = sunis; walk; walk = walk->next) {
		dev = walk->dev;
		stats = &walk->sonet_stats;

		PUT(0, MRI); /* Latch counters */
		udelay(1);
		ADD_LIMITED(section_bip,(GET(RSOP_SBL) & 0xff) |
			((GET(RSOP_SBM) & 0xff) << 8));
		ADD_LIMITED(line_bip,(GET(RLOP_LBL) & 0xff) |
			((GET(RLOP_LB) & 0xff) << 8) |
			((GET(RLOP_LBM) & 0xf) << 16));
		ADD_LIMITED(path_bip,(GET(RPOP_PBL) & 0xff) |
			((GET(RPOP_PBM) & 0xff) << 8));
		ADD_LIMITED(line_febe,(GET(RLOP_LFL) & 0xff) |
			((GET(RLOP_LF) & 0xff) << 8) |
			((GET(RLOP_LFM) & 0xf) << 16));
		ADD_LIMITED(path_febe,(GET(RPOP_PFL) & 0xff) |
			((GET(RPOP_PFM) & 0xff) << 8));
		ADD_LIMITED(uncorr_hcs,GET(RXCP_HCS) & 0xff);
		ADD_LIMITED(rx_cells,(GET(RXCP_RCCL) & 0xff) |
			((GET(RXCP_RCC) & 0xff) << 8) |
			((GET(RXCP_RCCM) & 7) << 16));
		ADD_LIMITED(tx_cells,(GET(TXCP_TCCL) & 0xff) |
			((GET(TXCP_TCC) & 0xff) << 8) |
			((GET(TXCP_TCCM) & 7) << 16));
	}
	if (from_timer) mod_timer(&poll_timer,jiffies+HZ);
}

#undef ADD_LIMITED

static int fetch_stats(struct atm_dev *dev, struct sonet_stats __user *arg, int zero)
{
	struct sonet_stats tmp;
	int error = 0;

	sonet_copy_stats(&PRIV(dev)->sonet_stats, &tmp);
	if (arg) error = copy_to_user(arg, &tmp, sizeof(tmp));
	if (zero && !error) sonet_subtract_stats(&PRIV(dev)->sonet_stats, &tmp);
	return error ? -EFAULT : 0;
}


#define HANDLE_FLAG(flag,reg,bit) \
  if (todo & flag) { \
    if (set) PUT(GET(reg) | bit,reg); \
    else PUT(GET(reg) & ~bit,reg); \
    todo &= ~flag; \
  }


static int change_diag(struct atm_dev *dev,void __user *arg,int set)
{
	int todo;

	if (get_user(todo,(int __user *)arg)) return -EFAULT;
	HANDLE_FLAG(SONET_INS_SBIP,TSOP_DIAG,SUNI_TSOP_DIAG_DBIP8);
	HANDLE_FLAG(SONET_INS_LBIP,TLOP_DIAG,SUNI_TLOP_DIAG_DBIP);
	HANDLE_FLAG(SONET_INS_PBIP,TPOP_CD,SUNI_TPOP_DIAG_DB3);
	HANDLE_FLAG(SONET_INS_FRAME,RSOP_CIE,SUNI_RSOP_CIE_FOOF);
	HANDLE_FLAG(SONET_INS_LAIS,TSOP_CTRL,SUNI_TSOP_CTRL_LAIS);
	HANDLE_FLAG(SONET_INS_PAIS,TPOP_CD,SUNI_TPOP_DIAG_PAIS);
	HANDLE_FLAG(SONET_INS_LOS,TSOP_DIAG,SUNI_TSOP_DIAG_DLOS);
//	HANDLE_FLAG(SONET_INS_HCS,TACP_CS,SUNI_TACP_CS_DHCS);
	return put_user(todo,(int __user *)arg) ? -EFAULT : 0;
}


#undef HANDLE_FLAG


static int get_diag(struct atm_dev *dev,void __user *arg)
{
	int set;

	set = 0;
	if (GET(TSOP_DIAG) & SUNI_TSOP_DIAG_DBIP8) set |= SONET_INS_SBIP;
	if (GET(TLOP_DIAG) & SUNI_TLOP_DIAG_DBIP) set |= SONET_INS_LBIP;
	if (GET(TPOP_CD) & SUNI_TPOP_DIAG_DB3) set |= SONET_INS_PBIP;
	/* SONET_INS_FRAME is one-shot only */
	if (GET(TSOP_CTRL) & SUNI_TSOP_CTRL_LAIS) set |= SONET_INS_LAIS;
	if (GET(TPOP_CD) & SUNI_TPOP_DIAG_PAIS) set |= SONET_INS_PAIS;
	if (GET(TSOP_DIAG) & SUNI_TSOP_DIAG_DLOS) set |= SONET_INS_LOS;
//	if (GET(TACP_CS) & SUNI_TACP_CS_DHCS) set |= SONET_INS_HCS;
	return put_user(set,(int __user *)arg) ? -EFAULT : 0;
}


static int set_loopback(struct atm_dev *dev, int mode)
{
	unsigned char control;

	control = GET(CMC2) & ~(SUNI_CMC2_DLE | SUNI_CMC2_LLE);
	switch (mode) {
		case ATM_LM_NONE:
			break;
		case ATM_LM_LOC_PHY:
			control |= SUNI_CMC2_DLE;
			break;
		case ATM_LM_RMT_PHY:
			control |= SUNI_CMC2_LLE;
			break;
		default:
			return -EINVAL;
	}
	PUT(control,CMC2);
	PRIV(dev)->loop_mode = mode;
	return 0;
}

/*
 * SONET vs. SDH Configuration
 *
 * Z0INS (register 0x06): 0 for SONET, 1 for SDH
 * ENSS (register 0x3D): 0 for SONET, 1 for SDH
 * LEN16 (register 0x28): 0 for SONET, 1 for SDH (n/a for S/UNI 155 QUAD)
 * LEN16 (register 0x50): 0 for SONET, 1 for SDH (n/a for S/UNI 155 QUAD)
 * S[1:0] (register 0x46): 00 for SONET, 10 for SDH
 */

static int set_sonet(struct atm_dev *dev)
{
	REG_CHANGE(SUNI_TPOP_APM_S, SUNI_TPOP_APM_S_SHIFT,
		   SUNI_TPOP_S_SONET, TPOP_APM);

	return 0;
}

static int set_sdh(struct atm_dev *dev)
{
	REG_CHANGE(SUNI_TPOP_APM_S, SUNI_TPOP_APM_S_SHIFT,
		   SUNI_TPOP_S_SDH, TPOP_APM);

	return 0;
}


static int get_framing(struct atm_dev *dev, void __user *arg)
{
	int framing;
	unsigned char s;


	s = (GET(TPOP_APM) & SUNI_TPOP_APM_S) >> SUNI_TPOP_APM_S_SHIFT;
	if (s == SUNI_TPOP_S_SONET)
		framing = SONET_FRAME_SONET;
	else
		framing = SONET_FRAME_SDH;

	return put_user(framing, (int __user *) arg) ? -EFAULT : 0;
}

static int set_framing(struct atm_dev *dev, void __user *arg)
{
	int mode;

	if (get_user(mode, (int __user *) arg))
		return -EFAULT;

	if (mode == SONET_FRAME_SONET)
		return set_sonet(dev);
	else if (mode == SONET_FRAME_SDH)
		return set_sdh(dev);

	return -EINVAL;
}


static int suni_ioctl(struct atm_dev *dev,unsigned int cmd,void __user *arg)
{
	printk(KERN_WARNING "%s(): cmd %d\n", __func__, cmd);
	switch (cmd) {
		case SONET_GETSTATZ:
		case SONET_GETSTAT:
			return fetch_stats(dev, arg, cmd == SONET_GETSTATZ);
		case SONET_SETDIAG:
			return change_diag(dev,arg,1);
		case SONET_CLRDIAG:
			return change_diag(dev,arg,0);
		case SONET_GETDIAG:
			return get_diag(dev,arg);
		case SONET_SETFRAMING:
			if (!capable(CAP_NET_ADMIN))
				return -EPERM;
			return set_framing(dev, arg);
		case SONET_GETFRAMING:
			return get_framing(dev, arg);
		case SONET_GETFRSENSE:
			return -EINVAL;
		case ATM_SETLOOP:
			if (!capable(CAP_NET_ADMIN))
				return -EPERM;
			return set_loopback(dev,(int)(unsigned long)arg);
		case ATM_GETLOOP:
			return put_user(PRIV(dev)->loop_mode,(int __user *)arg) ?
				-EFAULT : 0;
		case ATM_QUERYLOOP:
			return put_user(ATM_LM_LOC_PHY | ATM_LM_RMT_PHY,
			    (int __user *) arg) ? -EFAULT : 0;
		default:
			return -ENOIOCTLCMD;
	}
}

static void poll_los(struct atm_dev *dev)
{
	dev->signal = GET(RSOP_SIS) & SUNI_RSOP_SIS_LOSV ? ATM_PHY_SIG_LOST :
	ATM_PHY_SIG_FOUND;
}

static irqreturn_t suni_int(int irq, void *dev_id)
{
	struct atm_dev *dev = (struct atm_dev *)dev_id;

	poll_los(dev);
	printk(KERN_EMERG "%s(itf %d): signal %s \n",dev->type,dev->number,
		dev->signal == ATM_PHY_SIG_LOST ? "lost" : "detected again");

	return IRQ_HANDLED;
}

static int suni_start(struct atm_dev *dev)
{
	int first;
	unsigned long flags;

	spin_lock_irqsave(&sunis_lock,flags);
	first = !sunis;
	PRIV(dev)->next = sunis;
	sunis = PRIV(dev);
	spin_unlock_irqrestore(&sunis_lock,flags);
	memset(&PRIV(dev)->sonet_stats, 0, sizeof(struct k_sonet_stats));
	PUT(GET(RSOP_CIE) | SUNI_RSOP_CIE_LOSE,RSOP_CIE);
		/* interrupt on loss of signal */
	poll_los(dev); /* ... and clear SUNI interrupts */
	if (dev->signal == ATM_PHY_SIG_LOST)
		DPRINTK(KERN_WARNING "%s(itf %d): no signal\n",dev->type,
			dev->number);
	PRIV(dev)->loop_mode = ATM_LM_NONE;
	suni_hz(0); /* clear SUNI counters */
	(void) fetch_stats(dev, NULL, 1); /* clear kernel counters */

	if (first) {
		init_timer(&poll_timer);
		poll_timer.expires = jiffies+HZ;
		poll_timer.function = suni_hz;
		poll_timer.data = 1;
		add_timer(&poll_timer);
	}
	return 0;
}

static int suni_stop(struct atm_dev *dev)
{
	struct suni_priv **walk;
	unsigned long flags;

	/* let SAR driver worry about stopping interrupts */
	spin_lock_irqsave(&sunis_lock,flags);
	for (walk = &sunis; *walk != PRIV(dev);
		walk = &(*walk)->next);
	*walk = (*walk)->next;
	if (!sunis) del_timer_sync(&poll_timer);
	spin_unlock_irqrestore(&sunis_lock,flags);

	return 0;
}

static const struct atmphy_ops suni_ops = {
	.start		= suni_start,
	.ioctl		= suni_ioctl,
	.stop		= suni_stop,
};

static int pm5384_init(struct atm_dev *dev, int phy_id, int port_width)
{
	unsigned char mri;
	int i;

	/* Make sure id correct */
	mri = GET(MRI);

	DPRINTK("%s: id=%d \n", __func__, mri);
	if ((mri & SUNI_MRI_TYPE_MASK) != SUNI_MRI_TYPE_VAL)
	{
		DPRINTK("id is not correct!\n");
		return -1;
	}

	/* Reset the device */
	PUT(SUNI_MRI_RESET, MRI);

	/* Reset pulse must be at least 100ns */
	udelay(1);

	/* Clear reset */
	PUT(0, MRI);

	if (port_width)
		PUT(SUNI_MC_ATM16, MC); /* Setup UTOPIA bus to 16 bit, default */
	else
		PUT(SUNI_MC_ATM8, MC); /* Setup UTOPIA bus to 8 bit */

	/* Set CSU to correct value */
	PUT(SUNI_CCSC_MODE_VAL, CCSC);

	/* Reset the RFCLK DLL */
	PUT(0, RFCLK_RST);

	/* Reset the TFCLK DLL */
	PUT(0, TFCLK_RST);

	/* Wait */
	udelay(1);

	/* Set RXCP FIFO reset */
	PUT(SUNI_RXCP_FUCC_FIFORST | SUNI_RXCP_FUCC_RCALEVEL0, RXCP_FUCC);

	/* Set TXCP FIFO reset */
	PUT(SUNI_TXCP_CFG1_FIFORST | SUNI_TXCP_CFG1_HCSADD, TXCP_CFG1);

	/* Wait */
	udelay(1);

	/* Clear RXCP FIFO reset */
	PUT(SUNI_RXCP_FUCC_RCALEVEL0, RXCP_FUCC);

	/* Clear TXCP FIFO reset */
	PUT(SUNI_TXCP_CFG1_HCSADD, TXCP_CFG1);

	/* Set the default */
	PUT(SUNI_RXCP_CFG1_HCSADD, RXCP_CFG1);

	/* Set Tx C2 byte for ATM traffic */
	PUT(SUNI_PSL_ATM_CELLS, TPOP_PSL);

	/* Set Expected Rx C2 byte for ATM traffic */
	PUT(SUNI_PSL_ATM_CELLS, SPTB_EPSL);

	/* Set HCSCTLEB to prevent HCS corruption, FIFO deep is 4 cell */
	PUT(SUNI_TXCP_CFG2_HCSCTLEB, TXCP_CFG2);

	/* Reset performance monitoring counters */
	PUT(0, MRI);

	/* Setup Tx idle cell pattern */
	PUT(SUNI_IDLE_PATTERN, TXCP_IPLDC);

	/* Set line alarm to output RALRM signal */
	PUT(SUNI_CRAC2_LOSEN |
		SUNI_CRAC2_LOFEN |
		SUNI_CRAC2_OOFEN |
		SUNI_CRAC2_LAISEN |
		SUNI_CRAC2_LRDIEN |
		SUNI_CRAC2_SDBEREN |
		SUNI_CRAC2_SFBEREN |
		SUNI_CRAC2_STIMEN, CRAC2);

	/* Setup UTOPIA L2 both Rx and Tx address */
	PUT(phy_id, L2RA);
	PUT(phy_id, L2TA);

	/* RSOP interrupt enable */
	PUT(SUNI_RSOP_CIE_LOS, RSOP_CIE);

	return 0;
}

int suni5384_init(struct atm_dev *dev, struct device_node *np,
			int *upc_slot, int *port_width, int *phy_id,
			u32 *line_bitr,	u32 *max_bitr, u32 *min_bitr)
{
	int err = 0;
	const unsigned int *prop;
	const phandle *ph;
	struct device_node *node;
	struct resource res;
	struct suni_priv *suni;
	printk(KERN_WARNING "%s: entering\n", __func__);

	if (!(suni = kzalloc(sizeof(struct suni_priv), GFP_KERNEL)))
		return -ENOMEM;
	suni->dev = dev;

	prop = of_get_property(np, "device-id", NULL);
	suni->phy_id = *prop - 1;
	*phy_id = suni->phy_id;

	err = of_address_to_resource(np, 0, &res);
	if (err) {
		err =  -EINVAL;
		goto out;
	}
	suni->phy_base = ioremap(res.start, res.end - res.start + 1);
	suni->irq = irq_of_parse_and_map(np, 0);

	prop = of_get_property(np, "line_bitr", NULL);
	*line_bitr = *prop;
	prop = of_get_property(np, "max_bitr", NULL);
	*max_bitr = *prop;
	prop = of_get_property(np, "min_bitr", NULL);
	*min_bitr = *prop;

	prop = of_get_property(np, "port-width", NULL);
	suni->port_width = *port_width = *prop;

	ph = of_get_property(np, "upc-slot", NULL);
	if (ph == NULL) {
		err = -ENODEV;
		goto out;
	}
	node = of_find_node_by_phandle(*ph);
	if (node == NULL) {
		err = -ENODEV;
		goto out;
	}
	prop = of_get_property(node, "device-id", NULL);
	*upc_slot = *prop - 1;
	of_node_put(node);

	dev->phy_data = suni;
	dev->phy = &suni_ops;

	if (request_irq(suni->irq, suni_int, IRQF_SHARED, "fua_phy", dev))
	{
		DPRINTK(KERN_ERR"install atm PHY %d irq handler failed\n", PRIV(dev)->irq);
		err =  -EFAULT;
		goto out;
	}
	printk(KERN_DEBUG "%s: install atm PHY %d irq\n", __func__, suni->irq);

	if (pm5384_init(dev, suni->phy_id, suni->port_width)) {
		DPRINTK(KERN_ERR"pm5384 chipset initialization failed\n");
		err = -ENODEV;
		goto out1;
	}

	return 0;
out1:
	free_irq(suni->irq, dev);
out:
	dev->phy_data = NULL;
	kfree(suni);
	return err;
}

void suni5384_exit(struct atm_dev *dev)
{
	free_irq(PRIV(dev)->irq, dev);
	dev->phy_data = NULL;
	dev->phy = NULL;
	iounmap(PRIV(dev)->phy_base);
	kfree(PRIV(dev));
}

EXPORT_SYMBOL_GPL(suni5384_init);
EXPORT_SYMBOL_GPL(suni5384_exit);

MODULE_LICENSE("GPL");

// SPDX-License-Identifier: GPL-2.0
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mailbox_client.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>
#include <linux/reset.h>

#include "remoteproc_internal.h"

#define MBOX_NAME			"mbox-chan"

#define ST_RPROC_VQ0		0
#define ST_RPROC_VQ1		1
#define ST_RPROC_MAX_VRING	2

#define MBOX_RX			0
#define MBOX_TX			1
#define MBOX_MAX		2

// #define SEC_SYS_BASE    ((__u32)0x20B0000)
#define SEC_SUBSYS_BASE 0x02000000
#define SEC_SYS_BASE (SEC_SUBSYS_BASE + 0x000B0000)

#define SEC_CLK_ADDR    ((__u32)0x3003024)


enum cvitek_rp_mbox_messages {
	RP_MBOX_READY		= 0xFFFFFF00,
	RP_MBOX_SHUTDOWN	= 0xFFFFFF01,
	RP_MBOX_END_MSG		= 0xFFFFFF02,
};

struct cvitek_rproc_config {
	bool			sw_reset;	 //soft-reset
	bool			hw_reset;		//hw-reset
	unsigned long		bootaddr_mask;
};

struct cvitek_rproc_mem {
	void __iomem *cpu_addr;
	phys_addr_t bus_addr;
	u32 dev_addr;
	size_t size;
};

struct cvitek_rproc {
	struct rproc			*rproc;
	struct cvitek_rproc_mem mem[4];
	struct cvitek_rproc_config	*config;
	struct reset_control	*reset;
	struct clk		*clk;
	u32			clk_rate;
	struct regmap		*boot_base;
	u32			boot_offset;
	// struct mbox_chan	*mbox_chan;
	// struct mbox_client mbox_client;
	int num_mems;
	struct platform_device *pdev;
};

static inline void clrbits_32(void __iomem *reg, u32 clear)
{
	iowrite32((ioread32(reg) & ~clear), reg);
}

static inline void setbits_32(void __iomem *reg, u32 set)
{
	iowrite32(ioread32(reg) | set, reg);
}

// static void cvitek_rproc_mbox_callback(struct mbox_client *mc, void *data)
// {
// 	struct rproc *rproc = dev_get_drvdata(mc->dev);
// 	//struct sunxi_mbox *mb = container_of(mbox_client, struct sunxi_mbox, client);
// 	//struct sunxi_rproc *cproc = rproc->priv;

// 	if (rproc_vq_interrupt(rproc, data) == IRQ_NONE)
// 		dev_dbg(&rproc->dev, "no message was found in vqid %d\n", data);

// 	dev_info(&rproc->dev, "mbox recv data:0x%x\n", *(uint32_t *)data);
// 	//mb->notifyid = *(uint32_t *)data;

// 	//queue_work(cproc->workqueue, &mb->vq_work);
// }

// static void cvitek_rproc_mb_tx_done(struct mbox_client *mc, void *msg, int r)
// {
// 	struct rproc *rproc = dev_get_drvdata(mc->dev);
// 	devm_kfree(&rproc->dev, msg);
// }


static void cvitek_rproc_kick(struct rproc *rproc, int vqid)
{
	struct cvitek_rproc *cproc = rproc->priv;
	struct device *dev = rproc->dev.parent;


	pr_info("remoteproc >> cvitek_rproc_kick\n");
	/*int ret;

	if (WARN_ON(vqid >= ST_RPROC_MAX_VRING))
		return;

	ret = mbox_send_message(cproc->mbox_chan[vqid * MBOX_MAX + MBOX_TX],
				(void *)&vqid);
	if (ret < 0)
		dev_err(dev, "failed to send message via mbox: %d\n", ret);*/
}

static int cvitek_rproc_start(struct rproc *rproc)
{
	struct device *dev = rproc->dev.parent;

	pr_info(">>cvitek_rproc_start\n");

	void __iomem* clk_reset  = ioremap(SEC_CLK_ADDR, 4);
	
    clrbits_32(clk_reset, 1 << 6);

    setbits_32(clk_reset, 1 << 6);

	iounmap(clk_reset);

	dev_info(dev, "Started from 0x%llx\n", rproc->bootaddr);

	return 0;
}

static int cvitek_rproc_stop(struct rproc *rproc)
{
	void __iomem* clk_reset  = ioremap(SEC_CLK_ADDR, 4);

    clrbits_32(clk_reset, 1 << 6); 

	iounmap(clk_reset);

	pr_info(">>cvitek_rproc_stop\n");

	return 0;
}

static int cvitek_rproc_mem_alloc(struct rproc *rproc,
			      struct rproc_mem_entry *mem)
{
	struct device *dev = rproc->dev.parent;
	void *va;

	va = ioremap_wc(mem->dma, mem->len);
	if (!va) {
		dev_err(dev, "Unable to map memory region: %pa+%zx\n",
			&mem->dma, mem->len);
		return -ENOMEM;
	}

	/* Update memory entry va */
	mem->va = va;

	return 0;
}

static int cvitek_rproc_mem_release(struct rproc *rproc,
				struct rproc_mem_entry *mem)
{
	iounmap(mem->va);

	return 0;
}

static int cvitek_rproc_parse_fw(struct rproc *rproc, const struct firmware *fw)
{
	struct device *dev = rproc->dev.parent;
	struct device_node *np = dev->of_node;
	struct rproc_mem_entry *mem;
	struct reserved_mem *rmem;
	struct of_phandle_iterator it;
	int index = 0;

	of_phandle_iterator_init(&it, np, "memory-region", NULL, 0);
	while (of_phandle_iterator_next(&it) == 0) {
		rmem = of_reserved_mem_lookup(it.node);
		if (!rmem) {
			dev_err(dev, "unable to acquire memory-region\n");
			return -EINVAL;
		}

		/*  No need to map vdev buffer */
		if (strcmp(it.node->name, "vdev0buffer")) {
			/* Register memory region */
			mem = rproc_mem_entry_init(dev, NULL,
						   (dma_addr_t)rmem->base,
						   rmem->size, rmem->base,
						   cvitek_rproc_mem_alloc,
						   cvitek_rproc_mem_release,
						   it.node->name);
		} else {
			/* Register reserved memory for vdev buffer allocation */
			mem = rproc_of_resm_mem_entry_init(dev, index,
							   rmem->size,
							   rmem->base,
							   it.node->name);
		}

		if (!mem)
			return -ENOMEM;

		rproc_add_carveout(rproc, mem);
		index++;
	}

	return rproc_elf_load_rsc_table(rproc, fw);
}


static const struct rproc_ops cvitek_rproc_ops = {
	.kick			= cvitek_rproc_kick,
	.start			= cvitek_rproc_start,
	.stop			= cvitek_rproc_stop,
	.parse_fw		= cvitek_rproc_parse_fw,
	.load			= rproc_elf_load_segments,
	.find_loaded_rsc_table	= rproc_elf_find_loaded_rsc_table,
	.sanity_check		= rproc_elf_sanity_check,
	.get_boot_addr		= rproc_elf_get_boot_addr,
};

static int cvitek_rproc_state(struct platform_device *pdev)
{
	//struct rproc *rproc = platform_get_drvdata(pdev);

	return 1;
}

static const struct cvitek_rproc_config cv1800b_rproc_cfg = {
	.sw_reset = true,
	.hw_reset = false,
	.bootaddr_mask = GENMASK(31, 6),
};

static const struct of_device_id cvitek_rproc_match[] = {
	{ .compatible = "cvitek,cv18xx-c906l-rproc", .data = &cv1800b_rproc_cfg},
	{},
};
MODULE_DEVICE_TABLE(of, cvitek_rproc_match);


static const char *cvitek_rproc_get_firmware(struct platform_device *pdev)
{
	const char *fw_name;
	int ret;

	ret = of_property_read_string(pdev->dev.of_node, "firmware-name",
				      &fw_name);
	if (ret)
		return ERR_PTR(ret);

	return fw_name;
}

static int cvitek_rproc_parse_dt(struct platform_device *pdev)
{
	return 0;
}

static int cvitek_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	struct cvitek_rproc *cproc;
	struct device_node *np = dev->of_node;
	struct rproc *rproc;
	// struct mbox_chan *chan;
	const char *firmware;
	int enabled;
	int ret;
	int i;

	match = of_match_device(cvitek_rproc_match, dev);
	if (!match || !match->data) {
		dev_err(dev, "No device match found\n");
		return -ENODEV;
	}

	firmware = cvitek_rproc_get_firmware(pdev);
	if (IS_ERR(firmware))
		return PTR_ERR(firmware);

	rproc = rproc_alloc(dev, dev_name(dev), &cvitek_rproc_ops, firmware, sizeof(*cproc));
	if (!rproc)
		return -ENOMEM;

	cproc = rproc->priv;

	rproc->has_iommu = false;
	cproc->pdev = pdev;
	cproc->config = (struct cvitek_rproc_config *)match->data;

	platform_set_drvdata(pdev, rproc);

	ret = cvitek_rproc_parse_dt(pdev);
	if (ret)
		goto free_rproc;

	enabled = cvitek_rproc_state(pdev);
	if (enabled < 0) {
		ret = enabled;
		goto free_clk;
	}

	if (enabled) {
		//atomic_inc(&rproc->power);
		//rproc->state = RPROC_RUNNING;
	} else {
		clk_set_rate(cproc->clk, cproc->clk_rate);
	}

	/*if (of_get_property(np, "mbox-names", NULL)) {
		cproc->mbox_client.dev		= dev;
		cproc->mbox_client.tx_done		= cvitek_rproc_mb_tx_done;
		cproc->mbox_client.tx_block	= false;
		cproc->mbox_client.rx_callback	= cvitek_rproc_mbox_callback;

		chan = mbox_request_channel_byname(&cproc->mbox_client, MBOX_NAME);
		if (IS_ERR(chan)) {
			dev_err(dev, "failed to request mbox chan 0\n");
			ret = PTR_ERR(chan);
			chan = NULL;
			//goto free_clk;
		}
		cproc->mbox_chan = chan;
	}*/

	ret = rproc_add(rproc);
	if (ret)
		goto free_clk;

	return 0;

// free_mbox:
// 	mbox_free_channel(cproc->mbox_chan);

free_clk:
	clk_unprepare(cproc->clk);

free_rproc:
	rproc_free(rproc);
	return ret;
}

static int cvitek_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct cvitek_rproc *cproc = rproc->priv;
	int i;

	rproc_del(rproc);

	clk_disable_unprepare(cproc->clk);

	// mbox_free_channel(cproc->mbox_chan);

	rproc_free(rproc);

	return 0;
}

static struct platform_driver cvitek_rproc_driver = {
	.probe = cvitek_rproc_probe,
	.remove = cvitek_rproc_remove,
	.driver = {
		.name = "cvitek-rproc",
		.of_match_table = of_match_ptr(cvitek_rproc_match),
	},
};
module_platform_driver(cvitek_rproc_driver);

MODULE_DESCRIPTION("Cvitek Remote Processor Control Driver");
MODULE_LICENSE("GPL v2");

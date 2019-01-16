/*
 * Crystal Cove  --  Device access for Intel PMIC for VLV2
 *
 * Copyright (c) 2013, Intel Corporation.
 *
 * Author: Yang Bin <bin.yang@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/mfd/intel_mid_pmic.h>
#include <linux/acpi.h>
#include <asm/intel_vlv2.h>
#include <linux/version.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include "./pmic.h"
#include <linux/gpio.h>
#include <linux/slab.h>

#define PMIC_NUM_REG       0xEF
#define PMIC_READ_STRLEN   4
#define PMIC_WRITE_STRLEN  9
#define NR_RETRY_CNT       3

enum pmic_chip_type {CCOVE, DCOVE, WCOVE};

static LIST_HEAD(pdata_list);
struct cell_dev_pdata {
        struct list_head	list;
	const char		*name;
	void			*data;
	int			len;
	int			id;
};

static struct intel_mid_pmic *pmic;
static struct dentry *pmic_dbgfs_root;
int pmic_reg = 0;
static int cache_offset = -1, cache_read_val, cache_write_val, \
	cache_write_pending, cache_flags;

struct device *intel_mid_pmic_dev(void)
{
	return pmic->dev;
}

int intel_mid_pmic_read_multi(int reg, u8 len, u8 *buf)
{
	int ret;

	if (!pmic)
		return -EIO;
	mutex_lock(&pmic->io_lock);
	ret = pmic->readmul(reg, len, buf);
	mutex_unlock(&pmic->io_lock);
	return ret;
}
EXPORT_SYMBOL(intel_mid_pmic_read_multi);

int intel_mid_pmic_write_multi(int reg, u8 len, u8 *buf)
{
	int ret;

	if (!pmic)
		return -EIO;
	mutex_lock(&pmic->io_lock);
	ret = pmic->writemul(reg, len, buf);
	mutex_unlock(&pmic->io_lock);
	return ret;
}

int intel_mid_pmic_readb(int reg)
{
	int ret;

	if (!pmic)
		return -EIO;
	mutex_lock(&pmic->io_lock);
	ret = pmic->readb(reg);
	mutex_unlock(&pmic->io_lock);
	return ret;
}
EXPORT_SYMBOL(intel_mid_pmic_readb);

int intel_mid_pmic_writeb(int reg, u8 val)
{
	int ret;

	if (!pmic)
		return -EIO;
	mutex_lock(&pmic->io_lock);
	ret = pmic->writeb(reg, val);
	mutex_unlock(&pmic->io_lock);
	return ret;
}
EXPORT_SYMBOL(intel_mid_pmic_writeb);

int intel_mid_pmic_setb(int reg, u8 mask)
{
	int ret;
	int val;

	if (!pmic)
		return -EIO;
	mutex_lock(&pmic->io_lock);
	val = pmic->readb(reg);
	val |= mask;
	ret = pmic->writeb(reg, val);
	mutex_unlock(&pmic->io_lock);
	return ret;
}
EXPORT_SYMBOL(intel_mid_pmic_setb);

int intel_mid_pmic_clearb(int reg, u8 mask)
{
	int ret;
	int val;

	if (!pmic)
		return -EIO;
	mutex_lock(&pmic->io_lock);
	val = pmic->readb(reg);
	val &= ~mask;
	ret = pmic->writeb(reg, val);
	mutex_unlock(&pmic->io_lock);
	return ret;
}
EXPORT_SYMBOL(intel_mid_pmic_clearb);

int intel_mid_pmic_update(int reg, u8 val, u8 mask)
{
	int ret;

	if (!pmic)
		return -EIO;
	mutex_lock(&pmic->io_lock);
	ret = pmic->readb(reg);
	if (ret < 0)
		goto err;
	val &= mask;
	ret &= ~mask;
	ret |= val;
	ret = pmic->writeb(reg, ret);
err:
	mutex_unlock(&pmic->io_lock);
	return ret;
}
EXPORT_SYMBOL(intel_mid_pmic_update);

#ifndef CONFIG_BTNS_PMIC
int intel_scu_ipc_read_mip(u8 *data, int len, int offset, int issigned)
{
	return 0;
}
EXPORT_SYMBOL(intel_scu_ipc_read_mip);

int intel_scu_ipc_ioread8(u16 addr, u8 *data)
{
	int ret;

	ret = intel_mid_pmic_readb(addr);
	if (ret < 0)
		return ret;

	*data = ret;
	return 0;
}
EXPORT_SYMBOL(intel_scu_ipc_ioread8);

int intel_scu_ipc_iowrite8(u16 addr, u8 data)
{
	return intel_mid_pmic_writeb(addr, data);
}
EXPORT_SYMBOL(intel_scu_ipc_iowrite8);

int intel_scu_ipc_update_register(u16 addr, u8 data, u8 mask)
{
	return intel_mid_pmic_update(addr, data, mask);
}
EXPORT_SYMBOL(intel_scu_ipc_update_register);

int intel_scu_ipc_readv(u16 *addr, u8 *data, int len)
{
	int i;
	int ret;

	if (len < 1 || len > 4)
		return -EINVAL;

	for (i = 0; i < len; i++) {
		ret = intel_scu_ipc_ioread8(addr[i], &data[i]);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL(intel_scu_ipc_readv);

int intel_scu_ipc_writev(u16 *addr, u8 *data, int len)
{
	int i;
	int ret;

	if (len < 1 || len > 4)
		return -EINVAL;

	for (i = 0; i < len; i++) {
		ret = intel_scu_ipc_iowrite8(addr[i], data[i]);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL(intel_scu_ipc_writev);
#endif

int intel_mid_pmic_set_pdata(const char *name, void *data, int len, int id)
{
	struct cell_dev_pdata *pdata;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		pr_err("intel_pmic: can't set pdata!\n");
		return -ENOMEM;
	}
	pdata->name = name;
	pdata->data = data;
	pdata->len = len;
	pdata->id = id;
	list_add_tail(&pdata->list, &pdata_list);
	return 0;
}
EXPORT_SYMBOL(intel_mid_pmic_set_pdata);

static void __pmic_regmap_flush(void)
{
	if (cache_write_pending)
		pmic->writeb(cache_offset, cache_write_val);
	cache_offset = -1;
	cache_write_pending = 0;
}

static void pmic_regmap_flush(void)
{
	mutex_lock(&pmic->io_lock);
	__pmic_regmap_flush();
	mutex_unlock(&pmic->io_lock);
}

static int pmic_regmap_write(struct intel_pmic_regmap *map, int val)
{
	int ret = 0;

	if (!IS_PMIC_REG_VALID(map))
		return -ENXIO;
	if (IS_PMIC_REG_INV(map))
		val = ~val;

	mutex_lock(&pmic->io_lock);
	if (cache_offset == map->offset) {
		if (cache_flags != map->flags) {
			dev_dbg(pmic->dev, "Same reg with diff flags\n");
			__pmic_regmap_flush();
		}
	}
	if (cache_offset != map->offset) {
		__pmic_regmap_flush();
		if (IS_PMIC_REG_WO(map) || IS_PMIC_REG_W1C(map)) {
			cache_write_val = 0;
			cache_read_val = pmic->readb(map->offset);
		} else {
			cache_read_val = pmic->readb(map->offset);
			cache_write_val = cache_read_val;
		}
		if (cache_read_val < 0) {
			dev_err(pmic->dev, "Register access error\n");
			ret = -EIO;
			goto err;
		}
		cache_offset = map->offset;
		cache_flags = map->flags;
	}
	val = ((val & map->mask) << map->shift);
	cache_write_val &= ~(map->mask << map->shift);
	cache_write_val |= val;
	cache_write_pending = 1;
	if (!IS_PMIC_REG_WO(map) && !IS_PMIC_REG_W1C(map))
		cache_read_val = cache_write_val;
err:
	dev_dbg(pmic->dev, "[%s]: offset=%x, shift=%x, mask=%x, flags=%x\n",
		__func__, map->offset, map->shift, map->mask, map->flags);
	dev_dbg(pmic->dev, "[%s]: cache_read=%x, cache_write=%x, ret=%x\n",
		__func__, cache_read_val, cache_write_val, ret);
	mutex_unlock(&pmic->io_lock);
	return ret;
}

static int pmic_regmap_read(struct intel_pmic_regmap *map)
{
	int ret = 0;

	if (!IS_PMIC_REG_VALID(map))
		return -ENXIO;

	mutex_lock(&pmic->io_lock);
	if (cache_offset == map->offset) {
		if (cache_flags != map->flags) {
			dev_dbg(pmic->dev, "Same reg with diff flags\n");
			__pmic_regmap_flush();
		}
	}
	if (cache_offset != map->offset) {
		__pmic_regmap_flush();
		if (IS_PMIC_REG_WO(map) || IS_PMIC_REG_W1C(map)) {
			cache_write_val = 0;
			cache_read_val = pmic->readb(map->offset);
		} else {
			cache_read_val = pmic->readb(map->offset);
			cache_write_val = cache_read_val;
		}
		if (cache_read_val < 0) {
			dev_err(pmic->dev, "Register access error\n");
			ret = -EIO;
			goto err;
		}
		cache_offset = map->offset;
		cache_flags = map->flags;
	}
	if (IS_PMIC_REG_INV(map))
		ret = ~cache_read_val;
	else
		ret = cache_read_val;
	ret = (ret >> map->shift) & map->mask;
	if (!IS_PMIC_REG_WO(map) && !IS_PMIC_REG_W1C(map))
		cache_write_val = cache_read_val;
err:
	dev_dbg(pmic->dev, "[%s]: offset=%x, shift=%x, mask=%x, flags=%x\n",
		__func__, map->offset, map->shift, map->mask, map->flags);
	dev_dbg(pmic->dev, "[%s]: cache_read=%x, cache_write=%x, ret=%x\n",
		__func__, cache_read_val, cache_write_val, ret);
	mutex_unlock(&pmic->io_lock);
	return ret;
}

static void pmic_irq_enable(struct irq_data *data)
{
	clear_bit((data->irq - pmic->irq_base) % 32,
		&(pmic->irq_mask[(data->irq - pmic->irq_base) / 32]));
	pmic->irq_need_update = 1;

	dev_dbg(pmic->dev, "[%s]: irq_mask = %x", __func__,
				pmic->irq_mask[(data->irq - pmic->irq_base)/32]);
}

static void pmic_irq_disable(struct irq_data *data)
{
	set_bit((data->irq - pmic->irq_base) % 32,
		&(pmic->irq_mask[(data->irq - pmic->irq_base) / 32]));
	pmic->irq_need_update = 1;
	dev_dbg(pmic->dev, "[%s]: irq_mask = %x", __func__,
				pmic->irq_mask[(data->irq - pmic->irq_base)/32]);
}

static void pmic_irq_sync_unlock(struct irq_data *data)
{
	struct intel_pmic_regmap *map;

	dev_dbg(pmic->dev, "[%s]: irq_mask = %x", __func__,
				pmic->irq_mask[(data->irq - pmic->irq_base)/32]);
	if (pmic->irq_need_update) {
		map = &pmic->irq_regmap[(data->irq - pmic->irq_base)].mask;

		if (test_bit((data->irq - pmic->irq_base) % 32,
			&(pmic->irq_mask[(data->irq - pmic->irq_base) / 32])))
			pmic_regmap_write(map, map->mask);
		else
			pmic_regmap_write(map, 0);

		pmic->irq_need_update = 0;
		pmic_regmap_flush();
	}
	mutex_unlock(&pmic->irq_lock);
}

static void pmic_irq_lock(struct irq_data *data)
{
	mutex_lock(&pmic->irq_lock);
}

static irqreturn_t pmic_irq_isr(int irq, void *data)
{
	return IRQ_WAKE_THREAD;
}

static irqreturn_t pmic_irq_thread(int irq, void *data)
{
	int i;

	mutex_lock(&pmic->irq_lock);
	for (i = 0; i < pmic->irq_num; i++) {
		if (test_bit(i % 32, &(pmic->irq_mask[i / 32])))
			continue;
		if (pmic_regmap_read(&pmic->irq_regmap[i].status)) {
			pmic_regmap_write(&pmic->irq_regmap[i].ack,
				pmic->irq_regmap[i].ack.mask);
			handle_nested_irq(pmic->irq_base + i);
		}
	}
	pmic_regmap_flush();
	mutex_unlock(&pmic->irq_lock);
	return IRQ_HANDLED;
}

static struct irq_chip pmic_irq_chip = {
	.name			= "intel_mid_pmic",
	.irq_bus_lock		= pmic_irq_lock,
	.irq_bus_sync_unlock	= pmic_irq_sync_unlock,
	.irq_disable		= pmic_irq_disable,
	.irq_enable		= pmic_irq_enable,
};

static int pmic_irq_init(void)
{
	int cur_irq;
	int ret;
	int i;
	struct intel_pmic_regmap *map;

	/* Mostly, it can help to increase cache hit if merge same register
	   access in one loop */
	for (i = 0; i < pmic->irq_num; i++) {
		map = &pmic->irq_regmap[i].mask;
		if (IS_PMIC_REG_VALID(map)) {
			pmic_regmap_write(map, map->mask);
			set_bit(i % 32, &(pmic->irq_mask[i / 32]));
		}
	}
	for (i = 0; i < pmic->irq_num; i++) {
		map = &pmic->irq_regmap[i].ack;
		if (IS_PMIC_REG_VALID(map))
			pmic_regmap_write(map, map->mask);
	}
	pmic_regmap_flush();

	pmic->irq_base = irq_alloc_descs(-1, VV_PMIC_IRQBASE, pmic->irq_num, 0);
	if (pmic->irq_base < 0) {
		dev_warn(pmic->dev, "Failed to allocate IRQs: %d\n",
			 pmic->irq_base);
		pmic->irq_base = 0;
		return -EINVAL;
	} else {
		dev_info(pmic->dev, "PMIC IRQ Base:%d\n", pmic->irq_base);
	}

	/* Register them with genirq */
	for (cur_irq = pmic->irq_base;
	     cur_irq < pmic->irq_num + pmic->irq_base;
	     cur_irq++) {
		irq_set_chip_data(cur_irq, pmic);
		irq_set_chip_and_handler(cur_irq, &pmic_irq_chip,
					 handle_edge_irq);
		irq_set_nested_thread(cur_irq, 1);
		irq_set_noprobe(cur_irq);
	}

	if (gpio_is_valid(pmic->pmic_int_gpio)) {
		ret = gpio_request_one(pmic->pmic_int_gpio,
					GPIOF_DIR_IN, "PMIC Interupt");
		if (ret) {
			dev_err(pmic->dev, "Request PMIC_INT gpio error\n");
			return ret;
		}

		pmic->irq = gpio_to_irq(pmic->pmic_int_gpio);
	}

	ret = request_threaded_irq(pmic->irq, pmic_irq_isr, pmic_irq_thread,
			pmic->irq_flags, "intel_mid_pmic", pmic);
	if (ret != 0) {
		dev_err(pmic->dev, "Failed to request IRQ %d: %d\n",
				pmic->irq, ret);
		if (gpio_is_valid(pmic->pmic_int_gpio))
			gpio_free(pmic->pmic_int_gpio);
		return ret;
	}
	ret = enable_irq_wake(pmic->irq);
	if (ret != 0) {
		dev_warn(pmic->dev, "Can't enable PMIC IRQ as wake source: %d\n",
			 ret);
	}

	return 0;
}

static int pmic_addr_show(struct seq_file *seq, void *v)
{
	int i = 0;
	int ret = 0;

	for (i = 0; i < NR_RETRY_CNT; i++) {
		ret = intel_mid_pmic_readb(pmic_reg);
		if (ret == -EAGAIN || ret == -ETIMEDOUT)
			continue;
		else
			break;
	}
	if (ret < 0)
		dev_dbg(pmic->dev, "pmic_reg read err:%d\n", ret);

	seq_printf(seq, "0x%x=0x%x\n", pmic_reg, ret);

    return 0;
}

static int pmic_addr_open(struct inode *inode, struct file *file)
{
	return single_open(file, pmic_addr_show, NULL);
}

static ssize_t pmic_addr_write(struct file *filp, const char __user *buffer,
							   size_t count, loff_t *ppos)
{
	char buf[32] = {0};
	int i = 0, ret = 0;
	int val = 0, len = 0;

    if (copy_from_user(buf, buffer, count))
        return -EFAULT;

	len = count - 1;

	if (len == PMIC_READ_STRLEN) {
		sscanf(buf, "0x%x", &pmic_reg);
	} else if (len == PMIC_WRITE_STRLEN) {
		sscanf(buf, "%x=%x", &pmic_reg, &val);
		for (i = 0; i < NR_RETRY_CNT; i++) {
			ret = intel_mid_pmic_writeb(pmic_reg, val);
			if (ret == -EAGAIN || ret == -ETIMEDOUT)
				continue;
			else
				break;
		}
		if (ret < 0)
			dev_dbg(pmic->dev, "pmic_reg write err:%d\n", ret);
	} else {
		dev_dbg(pmic->dev, "Wrong set.\n");
		return -EPERM;
	}

    *ppos += count;

    return count;
}

struct file_operations addr_fops = {
    .owner = THIS_MODULE,
    .open = pmic_addr_open,
    .read = seq_read,
    .write = pmic_addr_write,
};

static int pmic_all_show(struct seq_file *seq, void *v)
{
	int i = 0, j = 0;
	int ret = 0;
	for (i = 0; i < PMIC_NUM_REG; ++i) {
		pmic_reg = i;
		for (j = 0; j < NR_RETRY_CNT; j++) {
			ret = intel_mid_pmic_readb(pmic_reg);
			if (ret == -EAGAIN || ret == -ETIMEDOUT)
				continue;
			else
				break;
		}
		if (ret < 0)
			dev_dbg(pmic->dev, "pmic_reg read err:%d\n", ret);

		seq_printf(seq, "0x%x=0x%x\n", pmic_reg, ret);
	}

    return 0;
}

static int pmic_all_open(struct inode *inode, struct file *file)
{
	return single_open(file, pmic_all_show, NULL);
}

struct file_operations all_fops = {
    .owner = THIS_MODULE,
    .open = pmic_all_open,
    .read = seq_read,
};

static void pmic_create_debugfs()
{
	struct dentry *entry;

	pmic_dbgfs_root = debugfs_create_dir("pmic_debug", NULL);
	if (IS_ERR(pmic_dbgfs_root)) {
		dev_dbg(pmic->dev, "DEBUGFS DIR create failed\n");
		return ;
	}

	entry = debugfs_create_file("all", 0644, pmic_dbgfs_root, NULL, &all_fops);
	if (IS_ERR(entry)) {
		debugfs_remove_recursive(pmic_dbgfs_root);
		pmic_dbgfs_root = NULL;
		dev_dbg(pmic->dev, "DEBUGFS entry Create failed\n");
		return ;
	}

	entry = debugfs_create_file("addr", 0644, pmic_dbgfs_root, NULL, &addr_fops);
	if (IS_ERR(entry)) {
		debugfs_remove_recursive(pmic_dbgfs_root);
		pmic_dbgfs_root = NULL;
		dev_dbg(pmic->dev, "DEBUGFS entry Create failed\n");
		return ;
	}
}

int intel_pmic_add(struct intel_mid_pmic *chip)
{
	int i, ret;
	struct cell_dev_pdata *pdata;

	if (pmic != NULL)
		return -EBUSY;
	mutex_init(&chip->io_lock);
	mutex_init(&chip->irq_lock);
	pmic = chip;
	if (pmic->init) {
		ret = pmic->init();
		if (ret != 0) {
			pmic = NULL;
			return ret;
		}
	}
	pmic_irq_init();
	for (i = 0; pmic->cell_dev[i].name != NULL; i++) {
		list_for_each_entry(pdata, &pdata_list, list) {
			if (!strcmp(pdata->name, pmic->cell_dev[i].name) &&
					(pdata->id == pmic->cell_dev[i].id)) {
				pmic->cell_dev[i].platform_data = pdata->data;
				pmic->cell_dev[i].pdata_size = pdata->len;
			}
		}
	}

	pmic_create_debugfs();

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 1))
	return mfd_add_devices(pmic->dev, -1, pmic->cell_dev, i,
			NULL, pmic->irq_base, NULL);
#else
	return mfd_add_devices(pmic->dev, -1, pmic->cell_dev, i,
			NULL, pmic->irq_base);
#endif
}

int intel_pmic_remove(struct intel_mid_pmic *chip)
{
	if (pmic != chip)
		return -ENODEV;
	mfd_remove_devices(pmic->dev);
	pmic = NULL;

	if (pmic_dbgfs_root)
		debugfs_remove_recursive(pmic_dbgfs_root);

	return 0;
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yang Bin <bin.yang@intel.com");


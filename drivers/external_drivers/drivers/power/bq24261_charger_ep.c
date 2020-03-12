/*
 * bq24261_charger.c - BQ24261 Charger I2C client driver
 *
 * Copyright (C) 2011 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Jenny TC <jenny.tc@intel.com>
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/power_supply.h>
#include <linux/pm_runtime.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/usb/otg.h>
#include <linux/power/bq24261_charger.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/wakelock.h>
#include <linux/switch.h>
#include <linux/sysfs.h>

#include <asm/intel_scu_ipc.h>
#include <linux/gpio.h>

#define DEV_NAME "bq24261_charger"
#define DEV_MANUFACTURER "TI"
#define MODEL_NAME_SIZE 8
#define DEV_MANUFACTURER_NAME_SIZE 4

#define CHRG_TERM_WORKER_DELAY (30 * HZ)
#define EXCEPTION_MONITOR_DELAY (60 * HZ)
#define WDT_RESET_DELAY (5 * HZ)

/* BQ24261 registers */
#define BQ24261_STAT_CTRL0_ADDR		0x00
#define BQ24261_CTRL_ADDR		0x01
#define BQ24261_BATT_VOL_CTRL_ADDR	0x02
#define BQ24261_VENDOR_REV_ADDR		0x03
#define BQ24261_TERM_FCC_ADDR		0x04
#define BQ24261_VINDPM_STAT_ADDR	0x05
#define BQ24261_ST_NTC_MON_ADDR		0x06

#define BQ24261_RESET_MASK		(0x01 << 7)
#define BQ24261_RESET_ENABLE		(0x01 << 7)

#define BQ24261_FAULT_MASK		0x07
#define BQ24261_STAT_MASK		(0x03 << 4)
#define BQ24261_BOOST_MASK		(0x01 << 6)
#define BQ24261_TMR_RST_MASK		(0x01 << 7)
#define BQ24261_TMR_RST			(0x01 << 7)

#define BQ24261_ENABLE_BOOST		(0x01 << 6)

#define BQ24261_VOVP			0x01
#define BQ24261_LOW_SUPPLY		0x02
#define BQ24261_THERMAL_SHUTDOWN	0x03
#define BQ24261_BATT_TEMP_FAULT		0x04
#define BQ24261_TIMER_FAULT		0x05
#define BQ24261_BATT_OVP		0x06
#define BQ24261_NO_BATTERY		0x07
#define BQ24261_STAT_READY		0x00

#define BQ24261_STAT_CHRG_PRGRSS	(0x01 << 4)
#define BQ24261_STAT_CHRG_DONE		(0x02 << 4)
#define BQ24261_STAT_FAULT		(0x03 << 4)

#define BQ24261_CE_MASK			(0x01 << 1)
#define BQ24261_CE_DISABLE		(0x01 << 1)

#define BQ24261_HZ_MASK			(0x01)
#define BQ24261_HZ_ENABLE		(0x01)

#define BQ24261_ICHRG_MASK		(0x1F << 3)

#define BQ24261_ITERM_MASK		(0x07)
#define BQ24261_MIN_ITERM 50 /* 50 mA */
#define BQ24261_MAX_ITERM 300 /* 300 mA */

#define BQ24261_VBREG_MASK		(0x3F << 2)

#define BQ24261_INLMT_MASK		(0x03 << 4)
#define BQ24261_INLMT_100		0x00
#define BQ24261_INLMT_150		(0x01 << 4)
#define BQ24261_INLMT_500		(0x02 << 4)
#define BQ24261_INLMT_900		(0x03 << 4)
#define BQ24261_INLMT_1500		(0x04 << 4)
#define BQ24261_INLMT_2000		(0x05 << 4)
#define BQ24261_INLMT_2500		(0x06 << 4)

#define BQ24261_TE_MASK			(0x01 << 2)
#define BQ24261_TE_ENABLE		(0x01 << 2)
#define BQ24261_STAT_ENABLE_MASK	(0x01 << 3)
#define BQ24261_STAT_ENABLE		(0x01 << 3)

#define BQ24261_VENDOR_MASK		(0x07 << 5)
#define BQ24261_VENDOR			(0x02 << 5)
#define BQ24261_REV_MASK		(0x07)
#define BQ24261_2_3_REV			(0x06)
#define BQ24261_REV			(0x02)
#define BQ24260_REV			(0x01)

#define BQ24261_TS_MASK			(0x01 << 3)
#define BQ24261_TS_ENABLED		(0x01 << 3)
#define BQ24261_BOOST_ILIM_MASK		(0x01 << 4)
#define BQ24261_BOOST_ILIM_500ma	(0x0)
#define BQ24261_BOOST_ILIM_1A		(0x01 << 4)
#define BQ24261_VINDPM_OFF_MASK		(0x01 << 0)
#define BQ24261_VINDPM_OFF_5V		(0x0)
#define BQ24261_VINDPM_OFF_12V		(0x01 << 0)

#define BQ24261_SAFETY_TIMER_MASK	(0x03 << 5)
#define BQ24261_SAFETY_TIMER_40MIN	0x00
#define BQ24261_SAFETY_TIMER_6HR	(0x01 << 5)
#define BQ24261_SAFETY_TIMER_9HR	(0x02 << 5)
#define BQ24261_SAFETY_TIMER_DISABLED	(0x03 << 5)

/* 1% above voltage max design to report over voltage */
#define BQ24261_OVP_MULTIPLIER			1010
#define BQ24261_OVP_RECOVER_MULTIPLIER		990
#define BQ24261_DEF_BAT_VOLT_MAX_DESIGN		4200000

/* Settings for Voltage / DPPM Register (05) */
#define BQ24261_VBATT_LEVEL1		4000000
#define BQ24261_VBATT_LEVEL2		4100000
#define BQ24261_VINDPM_MASK		(0x07)
#define BQ24261_VINDPM_320MV		(0x01 << 2)
#define BQ24261_VINDPM_160MV		(0x01 << 1)
#define BQ24261_VINDPM_80MV		(0x01 << 0)
#define BQ24261_CD_STATUS_MASK		(0x01 << 3)
#define BQ24261_DPM_EN_MASK		(0x01 << 4)
#define BQ24261_DPM_EN_FORCE		(0x01 << 4)
#define BQ24261_LOW_CHG_MASK		(0x01 << 5)
#define BQ24261_LOW_CHG_EN		(0x01 << 5)
#define BQ24261_LOW_CHG_DIS		(~BQ24261_LOW_CHG_EN)
#define BQ24261_DPM_STAT_MASK		(0x01 << 6)
#define BQ24261_MINSYS_STAT_MASK	(0x01 << 7)

#define BQ24261_MIN_CC			500 /* 500mA */
#define BQ24261_MAX_CC			3000 /* 3A */

#define OTG_DCDC_EN_GPIO "otg_dcdc_en"

#define MAX_STR_COPY	25
#define PRECHRG_VBAT_THOLD	3000000
#define PRECHRG_CC		700
#define BAT_MONITOR_DELAY (60 * HZ)

/* No of times retry on -EAGAIN or -ETIMEDOUT error */
#define NR_RETRY_CNT    3

/* 2nd charger IC (keyboard power supply, EP only) */
int is_second_ic(struct i2c_client *client);
int ic2_init_complete = 0;
struct bq24261_charger *chip_ic2;
int ep_kbd_pwr_state = 0;
static void bq24261_wdt_boost_ic2(struct bq24261_charger *chip, int val);

u16 bq24261_sfty_tmr[][2] = {
	{0, BQ24261_SAFETY_TIMER_DISABLED}
	,
	{40, BQ24261_SAFETY_TIMER_40MIN}
	,
	{360, BQ24261_SAFETY_TIMER_6HR}
	,
	{540, BQ24261_SAFETY_TIMER_9HR}
	,
};


u16 bq24261_inlmt[][2] = {
	{100, BQ24261_INLMT_100}
	,
	{150, BQ24261_INLMT_150}
	,
	{500, BQ24261_INLMT_500}
	,
	{900, BQ24261_INLMT_900}
	,
	{1500, BQ24261_INLMT_1500}
	,
	{2000, BQ24261_INLMT_2000}
	,
	{2500, BQ24261_INLMT_2500}
	,
};

#define BQ24261_MIN_CV 3500
#define BQ24261_MAX_CV 4440
#define BQ24261_CV_DIV 20
#define BQ24261_CV_BIT_POS 2

static enum power_supply_property bq24261_usb_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_MAX_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_MAX_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_INLMT,
	POWER_SUPPLY_PROP_ENABLE_CHARGING,
	POWER_SUPPLY_PROP_ENABLE_CHARGER,
	POWER_SUPPLY_PROP_CHARGE_TERM_CUR,
	POWER_SUPPLY_PROP_CABLE_TYPE,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MAX_TEMP,
	POWER_SUPPLY_PROP_MIN_TEMP,
};

enum bq24261_chrgr_stat {
	BQ24261_CHRGR_STAT_UNKNOWN,
	BQ24261_CHRGR_STAT_READY,
	BQ24261_CHRGR_STAT_CHARGING,
	BQ24261_CHRGR_STAT_BAT_FULL,
	BQ24261_CHRGR_STAT_FAULT,
};

struct bq24261_otg_event {
	struct list_head node;
	bool is_enable;
};

struct bq24261_charger {

	struct mutex stat_lock;
	struct i2c_client *client;
	struct bq24261_plat_data *pdata;
	struct power_supply psy_usb;
	struct delayed_work sw_term_work;
	struct delayed_work wdt_work;
	struct delayed_work low_supply_fault_work;
	struct delayed_work exception_mon_work;
	struct delayed_work bat_mon_work;
	struct notifier_block otg_nb;
	struct usb_phy *transceiver;
	struct work_struct otg_work;
	struct work_struct irq_work;
	struct list_head otg_queue;
	struct list_head irq_queue;
	wait_queue_head_t wait_ready;
	spinlock_t otg_queue_lock;
	void __iomem *irq_iomap;
	struct dentry *bq24261_dbgfs_dir;
	struct device *dev;

	int chrgr_health;
	int bat_health;
	int cc;
	int cv;
	int inlmt;
	int max_cc;
	int max_cv;
	int iterm;
	int cable_type;
	int cntl_state;
	int max_temp;
	int min_temp;
	int revision;
	enum bq24261_chrgr_stat chrgr_stat;
	bool online;
	bool present;
	bool is_charging_enabled;
	bool is_charger_enabled;
	bool is_vsys_on;
	bool boost_mode;
	bool is_hw_chrg_term;
	bool kbd_key_pressed;
	int kbd_dock_state;
	int kbd_key;

	char model_name[MODEL_NAME_SIZE];
	char manufacturer[DEV_MANUFACTURER_NAME_SIZE];
	struct wake_lock chrgr_en_wakelock;
	int ext_booster_gpio;
};

enum bq2426x_model_num {
	BQ2426X = 0,
	BQ24260,
	BQ24261,
};

struct bq2426x_model {
	char model_name[MODEL_NAME_SIZE];
	enum bq2426x_model_num model;
};

static struct bq2426x_model bq24261_model_name[] = {
	{ "bq2426x", BQ2426X },
	{ "bq24260", BQ24260 },
	{ "bq24261", BQ24261 },
};

struct dock_switch_data {
	struct switch_dev sdev;
};
static struct dock_switch_data *sdock;

enum kbd_dock_state {
	EXTRA_DOCK_STATE_KEYBOARD_KEYPRESS = 1004,
	EXTRA_DOCK_STATE_KEYBOARD_KEYRELEASE = 1005,
};

#define  WATCH_DOG_TIMEOUT  6
#define SPECIAL_KEY  222

struct i2c_client *bq24261_client;
static inline int get_battery_voltage(int *volt);
static inline int get_battery_current(int *cur);
static int bq24261_handle_irq(struct bq24261_charger *chip, u8 stat_reg);
static inline int bq24261_set_iterm(struct bq24261_charger *chip, int iterm);

enum power_supply_type get_power_supply_type(
		enum power_supply_charger_cable_type cable)
{

	switch (cable) {

	case POWER_SUPPLY_CHARGER_TYPE_USB_DCP:
		return POWER_SUPPLY_TYPE_USB_DCP;
	case POWER_SUPPLY_CHARGER_TYPE_USB_CDP:
		return POWER_SUPPLY_TYPE_USB_CDP;
	case POWER_SUPPLY_CHARGER_TYPE_USB_ACA:
	case POWER_SUPPLY_CHARGER_TYPE_ACA_DOCK:
		return POWER_SUPPLY_TYPE_USB_ACA;
	case POWER_SUPPLY_CHARGER_TYPE_AC:
		return POWER_SUPPLY_TYPE_MAINS;
	case POWER_SUPPLY_CHARGER_TYPE_SE1:
		return POWER_SUPPLY_TYPE_USB_DCP;
	case POWER_SUPPLY_CHARGER_TYPE_DOCK:
		return POWER_SUPPLY_TYPE_DOCK;
	case POWER_SUPPLY_CHARGER_TYPE_NONE:
	case POWER_SUPPLY_CHARGER_TYPE_USB_SDP:
	default:
		return POWER_SUPPLY_TYPE_USB;
	}

	return POWER_SUPPLY_TYPE_USB;
}

static void lookup_regval(u16 tbl[][2], size_t size, u16 in_val, u8 *out_val)
{
	int i;
	for (i = 1; i < size; ++i)
		if (in_val < tbl[i][0])
			break;

	*out_val = (u8) tbl[i - 1][1];
}

int bq24261_set_kbd_key(__u16 key)
{
        int ret = -EINVAL;
        struct bq24261_charger *chip = i2c_get_clientdata(bq24261_client);

        if (chip != 0) {
            chip->kbd_key = ret = (int)key;
            if (chip->kbd_key_pressed) {
				chip->kbd_key_pressed = false;
                dev_info(&bq24261_client->dev, "SPECIAL_KEY_RELEASED\n");
                chip->kbd_dock_state = EXTRA_DOCK_STATE_KEYBOARD_KEYRELEASE;
                switch_set_state(&sdock->sdev, chip->kbd_dock_state);
            } else {
				chip->kbd_key_pressed = true;
                dev_info(&bq24261_client->dev, "SPECIAL_KEY_PRESSED\n");
                chip->kbd_dock_state = EXTRA_DOCK_STATE_KEYBOARD_KEYPRESS;
                switch_set_state(&sdock->sdev, chip->kbd_dock_state);
            }
        }
        return ret;
}

void bq24261_cc_to_reg(int cc, u8 *reg_val)
{
	/* Ichrg bits are B3-B7
	 * Icharge = 500mA + IchrgCode * 100mA
	 */
	cc = clamp_t(int, cc, BQ24261_MIN_CC, BQ24261_MAX_CC);
	cc = cc - BQ24261_MIN_CC;
	*reg_val = (cc / 100) << 3;
}

void bq24261_cv_to_reg(int cv, u8 *reg_val)
{
	int val;

	val = clamp_t(int, cv, BQ24261_MIN_CV, BQ24261_MAX_CV);
	*reg_val =
		(((val - BQ24261_MIN_CV) / BQ24261_CV_DIV)
			<< BQ24261_CV_BIT_POS);
}

void bq24261_inlmt_to_reg(int inlmt, u8 *regval)
{
	return lookup_regval(bq24261_inlmt, ARRAY_SIZE(bq24261_inlmt),
			     inlmt, regval);
}

static inline void bq24261_iterm_to_reg(int iterm, u8 *regval)
{
	/* Iterm bits are B0-B2
	 * Icharge = 50mA + ItermCode * 50mA
	 */
	iterm = clamp_t(int, iterm, BQ24261_MIN_ITERM,  BQ24261_MAX_ITERM);
	iterm = iterm - BQ24261_MIN_ITERM;
	*regval =  iterm / 50;
}

static inline void bq24261_sfty_tmr_to_reg(int tmr, u8 *regval)
{
	return lookup_regval(bq24261_sfty_tmr, ARRAY_SIZE(bq24261_sfty_tmr),
			     tmr, regval);
}

static inline int bq24261_read_reg(struct i2c_client *client, u8 reg)
{
	int ret, i;

	for (i = 0; i < NR_RETRY_CNT; i++) {
		ret = i2c_smbus_read_byte_data(client, reg);
		if (ret == -EAGAIN || ret == -ETIMEDOUT)
			continue;
		else
			break;
	}

	if (ret < 0)
		dev_err(&client->dev, "Error(%d) in reading reg %d\n", ret,
			reg);

	return ret;
}


static inline void bq24261_dump_regs(bool dump_master)
{
	int i;
	int ret;
	int bat_cur, bat_volt;
	struct bq24261_charger *chip;
	char buf[1024] = {0};
	int used = 0;

	if (!bq24261_client)
		return;

	chip = i2c_get_clientdata(bq24261_client);

	dev_info(&bq24261_client->dev, "*======================*\n");
	ret = get_battery_current(&bat_cur);
	if (ret)
		dev_err(&bq24261_client->dev,
			"%s: Error in getting battery current", __func__);
	else
		dev_info(&bq24261_client->dev, "Battery Current=%dma\n",
				(bat_cur/1000));

	ret = get_battery_voltage(&bat_volt);
	if (ret)
		dev_err(&bq24261_client->dev,
			"%s: Error in getting battery voltage", __func__);
	else
		dev_info(&bq24261_client->dev, "Battery VOlatge=%dmV\n",
			(bat_volt/1000));


	dev_info(&bq24261_client->dev, "BQ24261 Register dump:\n");

	for (i = 0; i < 7; ++i) {
		ret = bq24261_read_reg(bq24261_client, i);
		if (ret < 0)
			dev_err(&bq24261_client->dev,
				"Error in reading REG 0x%X\n", i);
		else
			used += snprintf(buf + used, sizeof(buf) - used,
					" 0x%X=0x%X,", i, ret);
	}
	dev_info(&bq24261_client->dev, "%s\n", buf);
	dev_info(&bq24261_client->dev, "*======================*\n");

	if (chip->pdata->dump_master_regs && dump_master)
			chip->pdata->dump_master_regs();

}


#ifdef CONFIG_DEBUG_FS
struct debugfs_data {
	u8 debugfs_reg_addr;
	struct i2c_client *debugfs_bq24261_client;
};

static int bq24261_reg_show(struct seq_file *seq, void *unused)
{
	int val;
	struct debugfs_data *reg_data;

	reg_data = (struct debugfs_data *)seq->private;

	dev_dbg(&reg_data->debugfs_bq24261_client->dev, "%s-reg_addr:%d\n",
			__func__, reg_data->debugfs_reg_addr);
	val = bq24261_read_reg(reg_data->debugfs_bq24261_client,
				reg_data->debugfs_reg_addr);

	seq_printf(seq, "0x%02x\n", val);
	return 0;
}

static int bq24261_dbgfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, bq24261_reg_show, inode->i_private);
}

static u32 bq24261_register_set[] = {
	BQ24261_STAT_CTRL0_ADDR,
	BQ24261_CTRL_ADDR,
	BQ24261_BATT_VOL_CTRL_ADDR,
	BQ24261_VENDOR_REV_ADDR,
	BQ24261_TERM_FCC_ADDR,
	BQ24261_VINDPM_STAT_ADDR,
	BQ24261_ST_NTC_MON_ADDR,
};
static struct dentry *bq24261_dbgfs_dir;
static const struct file_operations bq24261_dbg_fops = {
	.open = bq24261_dbgfs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release
};

/* 2nd charger IC (keyboard power supply, EP only) */
static int ep_kbd_pwr_show(struct seq_file *s, void *unused)
{

	switch (ep_kbd_pwr_state) {
	    case 1:
		    seq_printf(s, "1\n");
		    break;
	    case 0:
		    seq_printf(s, "0\n");
		    break;
	    default:
		    seq_printf(s, "UNKNOWN, this is a BUG %08x\n", ep_kbd_pwr_state);
	}

	return 0;
}

static int ep_kbd_pwr_open(struct inode *inode, struct file *file)
{
	return single_open(file, ep_kbd_pwr_show, inode->i_private);
}

static ssize_t ep_kbd_pwr_write(struct file *file,
		const char __user *ubuf, size_t count, loff_t *ppos)
{
	u32			mode = 2;
	char			buf[32];

	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
		return -EFAULT;

	if (!strncmp(buf, "1", 1))
		mode = 1;

	if (!strncmp(buf, "0", 1))
		mode = 0;

	if (!((mode == 0) || (mode == 1)))
	return -EFAULT;

	if (ic2_init_complete) {
		ep_kbd_pwr_state = mode;     
		bq24261_wdt_boost_ic2(chip_ic2, ep_kbd_pwr_state);
	}

	return count;
}

static const struct file_operations ep_kbd_pwr_fops = {
	.open			= ep_kbd_pwr_open,
	.write			= ep_kbd_pwr_write,
	.read			= seq_read,
	.llseek			= seq_lseek,
	.release		= single_release,
};

static void bq24261_debugfs_init(void)
{
	struct dentry *fentry;
	u32 count = ARRAY_SIZE(bq24261_register_set);
	u32 i;
	char name[6] = {0};
	struct debugfs_data *reg_data;

	bq24261_dbgfs_dir = debugfs_create_dir(DEV_NAME, NULL);
	if (bq24261_dbgfs_dir == NULL)
		goto debugfs_root_exit;

	for (i = 0; i < count; i++) {
		reg_data = devm_kzalloc(&bq24261_client->dev,
					sizeof(*reg_data), GFP_KERNEL);
		if (!reg_data) {
			dev_err(&bq24261_client->dev,
				"mem alloc failed for reg_data %d\n", i);
			goto debugfs_root_exit;
		}

		reg_data->debugfs_bq24261_client = bq24261_client;

		snprintf(name, 6, "%02x", bq24261_register_set[i]);
		reg_data->debugfs_reg_addr = bq24261_register_set[i];
		fentry = debugfs_create_file(name, S_IRUGO,
						bq24261_dbgfs_dir,
						reg_data,
						&bq24261_dbg_fops);
		if (fentry == NULL)
			goto debugfs_err_exit;
	}

	/* 2nd charger IC (keyboard power supply, EP only) */
	fentry = debugfs_create_file("ep_kbd_pwr", S_IRUGO | S_IWUSR,
					bq24261_dbgfs_dir,
					&ep_kbd_pwr_state,
					&ep_kbd_pwr_fops);

	if (fentry == NULL)
		goto debugfs_err_exit;

	dev_err(&bq24261_client->dev, "Debugfs created successfully!!\n");
	return;

debugfs_err_exit:
	debugfs_remove_recursive(bq24261_dbgfs_dir);
debugfs_root_exit:
	dev_err(&bq24261_client->dev, "Error Creating debugfs!!\n");
	return;
}

static void bq24261_debugfs_exit(void)
{
	if (bq24261_dbgfs_dir)
		debugfs_remove_recursive(bq24261_dbgfs_dir);

	return;
}

#else
static void bq24261_debugfs_init(void)
{
	return;
}

static void bq24261_debugfs_exit(void)
{
	return;
}
#endif

/*
 * Sysfs entries to drive gpio and vbus for the keyboard
 */

static ssize_t drive_kbd_key(struct device *device,
			struct device_attribute *attr,
			const char *buf,
			ssize_t count);
static ssize_t show_drive_kbd_key(struct device *device,
                        struct device_attribute *attr,
                        char *buf);

static DEVICE_ATTR(kbd_key, S_IRUGO | S_IWUSR | S_IWGRP,
			show_drive_kbd_key, drive_kbd_key);


static inline int bq24261_write_reg(struct i2c_client *client, u8 reg, u8 data)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, data);
	if (ret < 0)
		dev_err(&client->dev, "Error(%d) in writing %d to reg %d\n",
			ret, data, reg);

	return ret;
}

static inline int bq24261_read_modify_reg(struct i2c_client *client, u8 reg,
					  u8 mask, u8 val)
{
	int ret;

	ret = bq24261_read_reg(client, reg);
	if (ret < 0)
		return ret;
	ret = (ret & ~mask) | (mask & val);
	return bq24261_write_reg(client, reg, ret);
}

static inline int bq24261_tmr_ntc_init(struct bq24261_charger *chip)
{
	u8 reg_val;
	int ret;

	bq24261_sfty_tmr_to_reg(chip->pdata->safety_timer, &reg_val);

	if (chip->pdata->is_ts_enabled)
		reg_val |= BQ24261_TS_ENABLED;

	/* Check if boost mode current configuration is above 1A*/
	if (chip->pdata->boost_mode_ma >= 1000)
		reg_val |= BQ24261_BOOST_ILIM_1A;

	ret = bq24261_read_modify_reg(chip->client, BQ24261_ST_NTC_MON_ADDR,
			BQ24261_TS_MASK|BQ24261_SAFETY_TIMER_MASK|
			BQ24261_BOOST_ILIM_MASK, reg_val);

	return ret;
}

static inline int bq24261_enable_charging(
	struct bq24261_charger *chip, bool val)
{
	int ret, vbat = 0;
	u8 reg_val;
	bool is_ready;

	dev_dbg(&chip->client->dev, "%s=%d\n", __func__, val);
	ret = bq24261_read_reg(chip->client,
					BQ24261_STAT_CTRL0_ADDR);
	if (ret < 0) {
		dev_err(&chip->client->dev,
			"Error(%d) in reading BQ24261_STAT_CTRL0_ADDR\n", ret);
	}

	is_ready =  (ret & BQ24261_STAT_MASK) != BQ24261_STAT_FAULT;

	/* If status is fault, wait for READY before enabling the charging */
	if (!is_ready && val) {
		ret = wait_event_timeout(chip->wait_ready,
			(chip->chrgr_stat == BQ24261_CHRGR_STAT_READY),
				HZ);
		dev_info(&chip->client->dev,
			"chrgr_stat=%x\n", chip->chrgr_stat);
		if (ret == 0) {
			dev_err(&chip->client->dev,
				"ChgrReady timeout, enable charging anyway\n");
		}
	}

	if (chip->pdata->enable_charging) {
		ret = chip->pdata->enable_charging(val);
		if (ret) {
			dev_err(&chip->client->dev,
				"Error(%d) in master enable-charging\n", ret);
		}
	}

	if (val) {
		reg_val = (~BQ24261_CE_DISABLE & BQ24261_CE_MASK);
		if (chip->is_hw_chrg_term)
			reg_val |= BQ24261_TE_ENABLE;
	} else {
		reg_val = BQ24261_CE_DISABLE;
	}

	reg_val |=  BQ24261_STAT_ENABLE;

	ret = bq24261_read_modify_reg(chip->client, BQ24261_CTRL_ADDR,
		       BQ24261_STAT_ENABLE_MASK|BQ24261_RESET_MASK|
				BQ24261_CE_MASK|BQ24261_TE_MASK,
					reg_val);
	if (ret || !val)
		return ret;

	bq24261_set_iterm(chip, chip->iterm);

	/* if the vbat < 3V, do not touch timer register */
	ret = get_battery_voltage(&vbat);
	if (ret)
		dev_err(&chip->client->dev, "Failed to read vbat ");

	if (vbat > PRECHRG_VBAT_THOLD) {
		ret = bq24261_tmr_ntc_init(chip);
		if (ret) {
			dev_err(&chip->client->dev,
				"Error(%d) in tmr_ntc_init\n", ret);
		}
	} else {
		reg_val=0;
		bq24261_sfty_tmr_to_reg(360, &reg_val);
		ret = bq24261_read_modify_reg(chip->client, BQ24261_ST_NTC_MON_ADDR,
								BQ24261_SAFETY_TIMER_MASK, reg_val);
	}

	dev_info(&chip->client->dev, "Completed %s=%d\n", __func__, val);
	bq24261_dump_regs(false);

	return ret;
}

/*
 * drive_kbd_key - sysfs set api for kbd_key attribute
 * Parameter as defined by sysfs
 * Context: can sleep
 */
static ssize_t drive_kbd_key(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				ssize_t count)
{
	struct bq24261_charger *chip = i2c_get_clientdata(bq24261_client);
	dev_dbg(&chip->client->dev, "%s: \n", __func__);
        /* Placeholder funciton if we need to set the key codes in tthe future */
	return count;
}

/*
 * show_drive_kbd_key - sysfs set api for kbd_key attribute
 * Parameter as defined by sysfs
 * Context: can sleep
 */
static ssize_t show_drive_kbd_key(struct device *dev,
                                struct device_attribute *attr,
                                char *buf)
{
	struct bq24261_charger *chip = i2c_get_clientdata(bq24261_client);
	unsigned long value;

        if (chip == 0)
            return -EINVAL;
	value = chip->kbd_key;

	return snprintf(buf, MAX_STR_COPY, "%lu\n", value);
}
static inline int bq24261_reset_timer(struct bq24261_charger *chip)
{
	return bq24261_read_modify_reg(chip->client, BQ24261_STAT_CTRL0_ADDR,
			BQ24261_TMR_RST_MASK, BQ24261_TMR_RST);
}

static inline int bq24261_enable_charger(
	struct bq24261_charger *chip, int val)
{

	/* TODO: Implement enable/disable HiZ mode to enable/
	*  disable charger
	*/
	u8 reg_val;
	int ret;

	dev_dbg(&chip->client->dev, "%s=%d\n", __func__, val);
	reg_val = val ? (~BQ24261_HZ_ENABLE & BQ24261_HZ_MASK)  :
			BQ24261_HZ_ENABLE;

	ret = bq24261_read_modify_reg(chip->client, BQ24261_CTRL_ADDR,
		       BQ24261_HZ_MASK|BQ24261_RESET_MASK, reg_val);
	if (ret)
		return ret;

	return bq24261_reset_timer(chip);
}

static inline int bq24261_set_cc(struct bq24261_charger *chip, int cc)
{
	u8 reg_val;
	int ret, vbat = 0;

	dev_dbg(&chip->client->dev, "%s=%d\n", __func__, cc);

	/* Charging current should be 0.1C if vbat < 3V */
	ret = get_battery_voltage(&vbat);

	if (ret)
		dev_err(&chip->client->dev, "Failed to read vbat ");

	if (vbat < PRECHRG_VBAT_THOLD) {
		cc = PRECHRG_CC;
		/*
		 * schedule the worker to bump up the charging current
		 * expect the battery to cross the pre charge threshold
		 */
		schedule_delayed_work(&chip->bat_mon_work, BAT_MONITOR_DELAY);
	} else {
		/*
		 * Send a notification to framework to resume charging with
		 * higher charging current
		 */
		power_supply_changed(&chip->psy_usb);
	}

	if (chip->pdata->set_cc) {
		ret = chip->pdata->set_cc(cc);
		if (unlikely(ret))
			return ret;
	}

	if (cc && (cc < BQ24261_MIN_CC)) {
		dev_dbg(&chip->client->dev, "Set LOW_CHG bit\n");
		reg_val = BQ24261_LOW_CHG_EN;
		ret = bq24261_read_modify_reg(chip->client,
				BQ24261_VINDPM_STAT_ADDR,
				BQ24261_LOW_CHG_MASK, reg_val);
	} else {
		dev_dbg(&chip->client->dev, "Clear LOW_CHG bit\n");
		reg_val = BQ24261_LOW_CHG_DIS;
		ret = bq24261_read_modify_reg(chip->client,
				BQ24261_VINDPM_STAT_ADDR,
				BQ24261_LOW_CHG_MASK, reg_val);
	}

	/* cc setting will be done by platform specific hardware
	 * but, in case of error-conditions or if the setting fails,
	 * the following will be a fail-safe mechanism.
	 */

	bq24261_cc_to_reg(cc, &reg_val);

	return bq24261_read_modify_reg(chip->client, BQ24261_TERM_FCC_ADDR,
			BQ24261_ICHRG_MASK, reg_val);
}

static inline int bq24261_set_cv(struct bq24261_charger *chip, int cv)
{
	int bat_volt;
	int ret;
	u8 reg_val;
	u8 vindpm_val = 0x0;

	dev_dbg(&chip->client->dev, "%s=%d\n", __func__, cv);
	/*
	* Setting VINDPM value as per the battery voltage
	*  VBatt           Vindpm     Register Setting
	*  < 4.0v           4.2v       0x0 (default)
	*  4.0v - 4.1v      4.36v      0x2
	*  > 4.1v           4.62v      0x5
	*/
	ret = get_battery_voltage(&bat_volt);
	if (ret) {
		dev_err(&chip->client->dev,
			"Error getting battery voltage!!\n");
	} else {
		if (bat_volt > BQ24261_VBATT_LEVEL2)
			vindpm_val =
				(BQ24261_VINDPM_320MV | BQ24261_VINDPM_80MV);
		else if (bat_volt > BQ24261_VBATT_LEVEL1)
			vindpm_val = BQ24261_VINDPM_160MV;
	}

	ret = bq24261_read_modify_reg(chip->client,
			BQ24261_VINDPM_STAT_ADDR,
			BQ24261_VINDPM_MASK,
			vindpm_val);
	if (ret) {
		dev_err(&chip->client->dev,
			"Error setting VINDPM setting!!\n");
		return ret;
	}

	if (chip->pdata->set_cv)
		chip->pdata->set_cv(cv);

	/* cv setting will be done by platform specific hardware
	 * but, in case of error-conditions or if the setting fails,
	 * the following will be a fail-safe mechanism.
	 */
	bq24261_cv_to_reg(cv, &reg_val);

	return bq24261_read_modify_reg(chip->client, BQ24261_BATT_VOL_CTRL_ADDR,
				       BQ24261_VBREG_MASK, reg_val);
}

static inline int bq24261_set_inlmt(struct bq24261_charger *chip, int inlmt)
{
	u8 reg_val;

	dev_dbg(&chip->client->dev, "%s=%d\n", __func__, inlmt);
	if (chip->pdata->set_inlmt)
		return chip->pdata->set_inlmt(inlmt);

	bq24261_inlmt_to_reg(inlmt, &reg_val);

	return bq24261_read_modify_reg(chip->client, BQ24261_CTRL_ADDR,
		       BQ24261_RESET_MASK|BQ24261_INLMT_MASK, reg_val);

}

static inline void resume_charging(struct bq24261_charger *chip)
{

	if (chip->is_charger_enabled)
		bq24261_enable_charger(chip, true);
	if (chip->inlmt)
		bq24261_set_inlmt(chip, chip->inlmt);
	if (chip->cc)
		bq24261_set_cc(chip, chip->cc);
	if (chip->cv)
		bq24261_set_cv(chip, chip->cv);
	if (chip->is_charging_enabled)
		bq24261_enable_charging(chip, true);
}

static inline int bq24261_set_iterm(struct bq24261_charger *chip, int iterm)
{
	u8 reg_val;

	if (chip->pdata->set_iterm)
		return chip->pdata->set_iterm(iterm);

	bq24261_iterm_to_reg(iterm, &reg_val);

	return bq24261_read_modify_reg(chip->client, BQ24261_TERM_FCC_ADDR,
				       BQ24261_ITERM_MASK, reg_val);
}

static inline int bq24261_enable_hw_charge_term(
	struct bq24261_charger *chip, bool val)
{
	u8 data;
	int ret;

	data = val ? BQ24261_TE_ENABLE : (~BQ24261_TE_ENABLE & BQ24261_TE_MASK);


	ret = bq24261_read_modify_reg(chip->client, BQ24261_CTRL_ADDR,
			       BQ24261_RESET_MASK|BQ24261_TE_MASK, data);

	if (ret)
		return ret;

	chip->is_hw_chrg_term = val ? true : false;

	return ret;
}

static inline int bq24261_enable_boost_mode(
	struct bq24261_charger *chip, int val)
{
	int ret = 0;


	if (val) {

		if (((chip->revision & BQ24261_REV_MASK) == BQ24261_REV) ||
				chip->pdata->is_wdt_kick_needed) {
			if (chip->pdata->enable_vbus)
				chip->pdata->enable_vbus(true);
		}

		if (chip->pdata->handle_otgmode)
			chip->pdata->handle_otgmode(true);

		/* TODO: Support different Host Mode Current limits */

		bq24261_enable_charger(chip, true);

		if (unlikely(chip->ext_booster_gpio > 0)) {
			ret = gpio_direction_output(chip->ext_booster_gpio, 1);
			if (ret) {
				dev_err(&chip->client->dev,
					"%s:OTG GPIO_DCDC_EN write is failed.\n",
					chip->ext_booster_gpio);
					return ret;
			}
		} else {
			ret = bq24261_read_modify_reg(chip->client,
						BQ24261_STAT_CTRL0_ADDR,
						BQ24261_BOOST_MASK,
						BQ24261_ENABLE_BOOST);
			if (unlikely(ret))
				return ret;
		}

		ret = bq24261_tmr_ntc_init(chip);
		if (unlikely(ret))
			return ret;
		chip->boost_mode = true;

		if (((chip->revision & BQ24261_REV_MASK) == BQ24261_REV) ||
				chip->pdata->is_wdt_kick_needed)
			schedule_delayed_work(&chip->wdt_work, 0);

		dev_info(&chip->client->dev, "Boost Mode enabled\n");
	} else {

		if (unlikely(chip->ext_booster_gpio > 0)) {
			ret = gpio_direction_output(chip->ext_booster_gpio, 0);
			if (ret) {
				dev_err(&chip->client->dev,
					"%s:OTG GPIO_DCDC_EN write is failed.\n",
					chip->ext_booster_gpio);
					return ret;
			}
		} else {
			ret = bq24261_read_modify_reg(chip->client,
						BQ24261_STAT_CTRL0_ADDR,
						BQ24261_BOOST_MASK,
						~BQ24261_ENABLE_BOOST);

			if (unlikely(ret))
				return ret;
		}
		/* if charging need not to be enabled, disable
		* the charger else keep the charger on
		*/
		if (!chip->is_charging_enabled)
			bq24261_enable_charger(chip, false);
		chip->boost_mode = false;
		dev_info(&chip->client->dev, "Boost Mode disabled\n");

		if (((chip->revision & BQ24261_REV_MASK) == BQ24261_REV) ||
				chip->pdata->is_wdt_kick_needed) {
			cancel_delayed_work_sync(&chip->wdt_work);

			if (chip->pdata->enable_vbus)
				chip->pdata->enable_vbus(false);
		}

		if (chip->pdata->handle_otgmode)
			chip->pdata->handle_otgmode(false);

		/* Notify power supply subsystem to enable charging
		 * if needed. Eg. if DC adapter is connected
		 */
		power_supply_changed(&chip->psy_usb);
	}

	return ret;
}

static inline bool bq24261_is_vsys_on(struct bq24261_charger *chip)
{
	int ret;
	struct i2c_client *client = chip->client;

	ret = bq24261_read_reg(client, BQ24261_CTRL_ADDR);
	if (ret < 0) {
		dev_err(&client->dev,
			"Error(%d) in reading BQ24261_CTRL_ADDR\n", ret);
		return false;
	}

	if (((ret & BQ24261_HZ_MASK) == BQ24261_HZ_ENABLE) &&
			chip->is_charger_enabled) {
		dev_err(&client->dev, "Charger in Hi Z Mode\n");
		bq24261_dump_regs(true);
		return false;
	}

	ret = bq24261_read_reg(client, BQ24261_VINDPM_STAT_ADDR);
	if (ret < 0) {
		dev_err(&client->dev,
			"Error(%d) in reading BQ24261_VINDPM_STAT_ADDR\n", ret);
		return false;
	}

	if (ret & BQ24261_CD_STATUS_MASK) {
		dev_err(&client->dev, "CD line asserted\n");
		bq24261_dump_regs(true);
		return false;
	}

	return true;
}


static inline bool bq24261_is_online(struct bq24261_charger *chip)
{
	if (chip->cable_type == POWER_SUPPLY_CHARGER_TYPE_NONE)
		return false;
	else if (!chip->is_charger_enabled)
		return false;
	/* BQ24261 gives interrupt only on stop/resume charging.
	 * If charging is already stopped, we need to query the hardware
	 * to see charger is still active and can supply vsys or not.
	 */
	else if ((chip->chrgr_stat == BQ24261_CHRGR_STAT_FAULT) ||
		 (!chip->is_charging_enabled))
	{
		return bq24261_is_vsys_on(chip);
	} else
		return chip->is_vsys_on;
}

static int bq24261_usb_set_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    const union power_supply_propval *val)
{
	struct bq24261_charger *chip = container_of(psy,
						    struct bq24261_charger,
						    psy_usb);
	int ret = 0;


	mutex_lock(&chip->stat_lock);


	switch (psp) {

	case POWER_SUPPLY_PROP_PRESENT:
		chip->present = val->intval;
		/*If charging capable cable is present, then
		hold the charger wakelock so that the target
		does not enter suspend mode when charging is
		in progress.
		If charging cable has been removed, then
		unlock the wakelock to allow the target to
		enter the sleep mode*/
		if (!wake_lock_active(&chip->chrgr_en_wakelock) &&
					val->intval)
			wake_lock(&chip->chrgr_en_wakelock);
		else if (wake_lock_active(&chip->chrgr_en_wakelock) &&
					!val->intval)
			wake_unlock(&chip->chrgr_en_wakelock);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		chip->online = val->intval;
		break;
	case POWER_SUPPLY_PROP_ENABLE_CHARGING:

		/* Reset charging to avoid issues of not starting
		 * charging when we're recovering from fault-cases.
		 */
		if (val->intval) {
			dev_info(&chip->client->dev, "Charging reset");
			ret = bq24261_enable_charging(chip, false);
			if (ret)
				dev_err(&chip->client->dev,
					"Error(%d) in charging reset", ret);
		}

		ret = bq24261_enable_charging(chip, val->intval);

		if (ret)
			dev_err(&chip->client->dev,
				"Error(%d) in %s charging", ret,
				(val->intval ? "enable" : "disable"));
		else
			chip->is_charging_enabled = val->intval;

		if (val->intval)
			bq24261_enable_hw_charge_term(chip, true);
		else
			cancel_delayed_work_sync(&chip->sw_term_work);

		break;
	case POWER_SUPPLY_PROP_ENABLE_CHARGER:

		/* Don't enable the charger unless overvoltage is recovered */

		if (chip->bat_health != POWER_SUPPLY_HEALTH_OVERVOLTAGE) {
			ret = bq24261_enable_charger(chip, val->intval);

			if (ret)
				dev_err(&chip->client->dev,
					"Error(%d) in %s charger", ret,
					(val->intval ? "enable" : "disable"));
			else
				chip->is_charger_enabled = val->intval;
		} else {
			dev_info(&chip->client->dev, "Battery Over Voltage. Charger will be disabled\n");
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_CURRENT:
		ret = bq24261_set_cc(chip, val->intval);
		if (!ret)
			chip->cc = val->intval;
		break;
	case POWER_SUPPLY_PROP_CHARGE_VOLTAGE:
		ret = bq24261_set_cv(chip, val->intval);
		if (!ret)
			chip->cv = val->intval;
		break;
	case POWER_SUPPLY_PROP_MAX_CHARGE_CURRENT:
		chip->max_cc = val->intval;
		break;
	case POWER_SUPPLY_PROP_MAX_CHARGE_VOLTAGE:
		chip->max_cv = val->intval;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CUR:
		ret = bq24261_set_iterm(chip, val->intval);
		if (!ret)
			chip->iterm = val->intval;
		break;
	case POWER_SUPPLY_PROP_CABLE_TYPE:

		chip->cable_type = val->intval;
		chip->psy_usb.type = get_power_supply_type(chip->cable_type);
		if (chip->cable_type != POWER_SUPPLY_CHARGER_TYPE_NONE) {
			chip->chrgr_health = POWER_SUPPLY_HEALTH_GOOD;
			chip->chrgr_stat = BQ24261_CHRGR_STAT_UNKNOWN;

			/* Adding this processing in order to check
			for any faults during connect */

			ret = bq24261_read_reg(chip->client,
						BQ24261_STAT_CTRL0_ADDR);
			if (ret < 0)
				dev_err(&chip->client->dev,
				"Error (%d) in reading status register(0x00)\n",
				ret);
			else
				bq24261_handle_irq(chip, ret);
		} else {
			chip->chrgr_stat = BQ24261_CHRGR_STAT_UNKNOWN;
			chip->chrgr_health = POWER_SUPPLY_HEALTH_UNKNOWN;
			cancel_delayed_work_sync(&chip->low_supply_fault_work);
		}


		break;
	case POWER_SUPPLY_PROP_INLMT:
		ret = bq24261_set_inlmt(chip, val->intval);
		if (!ret)
			chip->inlmt = val->intval;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		chip->cntl_state = val->intval;
		break;
	case POWER_SUPPLY_PROP_MAX_TEMP:
		chip->max_temp = val->intval;
		break;
	case POWER_SUPPLY_PROP_MIN_TEMP:
		chip->min_temp = val->intval;
		break;
	default:
		ret = -ENODATA;
	}

	mutex_unlock(&chip->stat_lock);
	return ret;
}

static int bq24261_usb_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct bq24261_charger *chip = container_of(psy,
						    struct bq24261_charger,
						    psy_usb);

	mutex_lock(&chip->stat_lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = chip->present;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = chip->online;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = chip->chrgr_health;
		break;
	case POWER_SUPPLY_PROP_MAX_CHARGE_CURRENT:
		val->intval = chip->max_cc;
		break;
	case POWER_SUPPLY_PROP_MAX_CHARGE_VOLTAGE:
		val->intval = chip->max_cv;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CURRENT:
		val->intval = chip->cc;
		break;
	case POWER_SUPPLY_PROP_CHARGE_VOLTAGE:
		val->intval = chip->cv;
		break;
	case POWER_SUPPLY_PROP_INLMT:
		val->intval = chip->inlmt;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CUR:
		val->intval = chip->iterm;
		break;
	case POWER_SUPPLY_PROP_CABLE_TYPE:
		val->intval = chip->cable_type;
		break;
	case POWER_SUPPLY_PROP_ENABLE_CHARGING:
		if (chip->boost_mode)
			val->intval = false;
		else
			val->intval = (chip->is_charging_enabled &&
			(chip->chrgr_stat == BQ24261_CHRGR_STAT_CHARGING));
		break;
	case POWER_SUPPLY_PROP_ENABLE_CHARGER:
		val->intval = bq24261_is_online(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		val->intval = chip->cntl_state;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		val->intval = chip->pdata->num_throttle_states;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = chip->model_name;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = chip->manufacturer;
		break;
	case POWER_SUPPLY_PROP_MAX_TEMP:
		val->intval = chip->max_temp;
		break;
	case POWER_SUPPLY_PROP_MIN_TEMP:
		val->intval = chip->min_temp;
		break;
	default:
		mutex_unlock(&chip->stat_lock);
		return -EINVAL;
	}

	mutex_unlock(&chip->stat_lock);
	return 0;
}

static inline struct power_supply *get_psy_battery(void)
{
	struct class_dev_iter iter;
	struct device *dev;
	static struct power_supply *pst;

	class_dev_iter_init(&iter, power_supply_class, NULL, NULL);
	while ((dev = class_dev_iter_next(&iter))) {
		pst = (struct power_supply *)dev_get_drvdata(dev);
		if (pst->type == POWER_SUPPLY_TYPE_BATTERY) {
			class_dev_iter_exit(&iter);
			return pst;
		}
	}
	class_dev_iter_exit(&iter);

	return NULL;
}

static inline int get_battery_voltage(int *volt)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;

	psy = get_psy_battery();
	if (!psy)
		return -EINVAL;

	ret = psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (!ret)
		*volt = (val.intval);

	return ret;
}

static inline int get_battery_volt_max_design(int *volt)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;

	psy = get_psy_battery();
	if (!psy)
		return -EINVAL;

	ret = psy->get_property(psy,
		POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN, &val);
	if (!ret)
		(*volt = val.intval);
	return ret;
}

static inline int get_battery_current(int *cur)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;

	psy = get_psy_battery();
	if (!psy)
		return -EINVAL;

	ret = psy->get_property(psy, POWER_SUPPLY_PROP_CURRENT_NOW, &val);
	if (!ret)
		*cur = val.intval;

	return ret;
}

static void bq24261_wdt_reset_worker(struct work_struct *work)
{

	struct bq24261_charger *chip = container_of(work,
			    struct bq24261_charger, wdt_work.work);
	int ret;
	ret = bq24261_reset_timer(chip);

	if (ret)
		dev_err(&chip->client->dev, "Error (%d) in WDT reset\n", ret);

	schedule_delayed_work(&chip->wdt_work, WDT_RESET_DELAY);
}

static void bq24261_wdt_boost_ic2(struct bq24261_charger *chip, int val)
{
	int ret;
	
	if (!ic2_init_complete) {
		dev_err(&chip->client->dev, "Error enabling booster: IC #2 is uninitialized.\n");
		return;
	}

	if (val == 1)
		ret = bq24261_read_modify_reg(chip->client,
						BQ24261_STAT_CTRL0_ADDR,
						BQ24261_BOOST_MASK,
						BQ24261_ENABLE_BOOST);
	else
		ret = bq24261_read_modify_reg(chip->client,
						BQ24261_STAT_CTRL0_ADDR,
						BQ24261_BOOST_MASK,
						~BQ24261_ENABLE_BOOST);
	if (ret)
		dev_err(&chip->client->dev, "Error (%d) in setting boostmode to %d\n", ret, val);

}

static void bq24261_wdt_reset_worker_ic2(struct work_struct *work)
{
	int ret;

	struct bq24261_charger *chip = container_of(work,
			    struct bq24261_charger, wdt_work.work);

	ret = bq24261_reset_timer(chip);

	if (ret)
		dev_err(&chip->client->dev, "Error (%d) in WDT reset of IC #2\n", ret);

	schedule_delayed_work(&chip->wdt_work, WDT_RESET_DELAY);
}

static void bq24261_sw_charge_term_worker(struct work_struct *work)
{

	struct bq24261_charger *chip = container_of(work,
						    struct bq24261_charger,
						    sw_term_work.work);

	power_supply_changed(NULL);

	schedule_delayed_work(&chip->sw_term_work,
			      CHRG_TERM_WORKER_DELAY);

}

int bq24261_get_bat_health(void)
{

	struct bq24261_charger *chip;

	if (!bq24261_client)
		return -ENODEV;

	chip = i2c_get_clientdata(bq24261_client);

	return chip->bat_health;
}


static void bq24261_low_supply_fault_work(struct work_struct *work)
{
	struct bq24261_charger *chip = container_of(work,
						    struct bq24261_charger,
						    low_supply_fault_work.work);

	if (chip->chrgr_stat == BQ24261_CHRGR_STAT_FAULT) {
		dev_err(&chip->client->dev, "Low Supply Fault detected!!\n");
		chip->chrgr_health = POWER_SUPPLY_HEALTH_DEAD;
		power_supply_changed(&chip->psy_usb);
		schedule_delayed_work(&chip->exception_mon_work,
					EXCEPTION_MONITOR_DELAY);
		bq24261_dump_regs(true);
	}
	return;
}


/* is_bat_over_voltage: check battery is over voltage or not
*  @chip: bq24261_charger context
*
*  This function is used to verify the over voltage condition.
*  In some scenarios, HW generates Over Voltage exceptions when
*  battery voltage is normal. This function uses the over voltage
*  condition (voltage_max_design * 1.01) to verify battery is really
*  over charged or not.
*/

static bool is_bat_over_voltage(struct bq24261_charger *chip,
		bool verify_recovery)
{

	int bat_volt, bat_volt_max_des, ret;

	ret = get_battery_voltage(&bat_volt);
	if (ret)
		return verify_recovery ? false : true;

	ret = get_battery_volt_max_design(&bat_volt_max_des);

	if (ret)
		bat_volt_max_des = BQ24261_DEF_BAT_VOLT_MAX_DESIGN;

	dev_info(&chip->client->dev, "bat_volt=%d Voltage Max Design=%d OVP_VOLT=%d OVP recover volt=%d\n",
			bat_volt, bat_volt_max_des,
			(bat_volt_max_des/1000 * BQ24261_OVP_MULTIPLIER),
			(bat_volt_max_des/1000 *
				BQ24261_OVP_RECOVER_MULTIPLIER));
	if (verify_recovery) {
		if ((bat_volt) <= (bat_volt_max_des / 1000 *
				BQ24261_OVP_RECOVER_MULTIPLIER))
			return true;
		else
			return false;
	} else {
		if ((bat_volt) >= (bat_volt_max_des / 1000 *
					BQ24261_OVP_MULTIPLIER))
			return true;
		else
			return false;
	}

	return false;
}

#define IS_BATTERY_OVER_VOLTAGE(chip) \
	is_bat_over_voltage(chip , false)

#define IS_BATTERY_OVER_VOLTAGE_RECOVERED(chip) \
	is_bat_over_voltage(chip , true)

static void handle_battery_over_voltage(struct bq24261_charger *chip)
{
	/* Set Health to Over Voltage. Disable charger to discharge
	*  battery to reduce the battery voltage.
	*/
	chip->bat_health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	bq24261_enable_charger(chip, false);
	chip->is_charger_enabled = false;
	cancel_delayed_work_sync(&chip->exception_mon_work);
	schedule_delayed_work(&chip->exception_mon_work,
			EXCEPTION_MONITOR_DELAY);
}

static void bq24261_exception_mon_work(struct work_struct *work)
{
	struct bq24261_charger *chip = container_of(work,
			struct bq24261_charger,
			exception_mon_work.work);
	int ret;

	if (chip->bat_health == POWER_SUPPLY_HEALTH_OVERVOLTAGE) {
		if (IS_BATTERY_OVER_VOLTAGE_RECOVERED(chip)) {
			dev_info(&chip->client->dev,
					"Battery OVP Exception Recovered\n");
			chip->bat_health = POWER_SUPPLY_HEALTH_GOOD;
			bq24261_enable_charger(chip, true);
			chip->is_charger_enabled = true;
			power_supply_changed(&chip->psy_usb);
		} else {
			schedule_delayed_work(&chip->exception_mon_work,
					EXCEPTION_MONITOR_DELAY);
		}
	}

	if ((chip->chrgr_health == POWER_SUPPLY_HEALTH_OVERVOLTAGE) ||
		(chip->chrgr_health == POWER_SUPPLY_HEALTH_DEAD)) {
		ret = bq24261_read_reg(chip->client, BQ24261_STAT_CTRL0_ADDR);
		if (ret < 0) {
			dev_err(&chip->client->dev, "Error reading reg %x\n",
					BQ24261_STAT_CTRL0_ADDR);
		} else {
			mutex_lock(&chip->stat_lock);
			bq24261_handle_irq(chip, ret);
			mutex_unlock(&chip->stat_lock);
			if ((ret & BQ24261_STAT_MASK) == BQ24261_STAT_READY) {
				dev_info(&chip->client->dev,
				"Charger OVP/Low Supply Exception recovered\n");
				power_supply_changed(&chip->psy_usb);
			}
		}
	}
}

static void bq24261_bat_mon_work(struct work_struct *work)
{
	struct bq24261_charger *chip = container_of(work,
			struct bq24261_charger,
			bat_mon_work.work);

	resume_charging(chip);
}

static int bq24261_handle_irq(struct bq24261_charger *chip, u8 stat_reg)
{
	struct i2c_client *client = chip->client;
	bool notify = true;

	dev_info(&client->dev, "%s:%d stat=0x%x\n",
			__func__, __LINE__, stat_reg);

	switch (stat_reg & BQ24261_STAT_MASK) {
	case BQ24261_STAT_READY:
		chip->chrgr_stat = BQ24261_CHRGR_STAT_READY;
		chip->chrgr_health = POWER_SUPPLY_HEALTH_GOOD;
		chip->bat_health = POWER_SUPPLY_HEALTH_GOOD;
		dev_info(&client->dev, "Charger Status: Ready\n");
		notify = false;
		break;
	case BQ24261_STAT_CHRG_PRGRSS:
		chip->chrgr_stat = BQ24261_CHRGR_STAT_CHARGING;
		chip->chrgr_health = POWER_SUPPLY_HEALTH_GOOD;
		chip->bat_health = POWER_SUPPLY_HEALTH_GOOD;
		dev_info(&client->dev, "Charger Status: Charge Progress\n");
		bq24261_dump_regs(false);
		break;
	case BQ24261_STAT_CHRG_DONE:
		chip->chrgr_health = POWER_SUPPLY_HEALTH_GOOD;
		chip->bat_health = POWER_SUPPLY_HEALTH_GOOD;
		dev_info(&client->dev, "Charger Status: Charge Done\n");

		bq24261_enable_hw_charge_term(chip, false);
		resume_charging(chip);
		schedule_delayed_work(&chip->sw_term_work, 0);
		break;

	case BQ24261_STAT_FAULT:
		break;
	}

	if (stat_reg & BQ24261_BOOST_MASK)
		dev_info(&client->dev, "Boost Mode\n");

	if ((stat_reg & BQ24261_STAT_MASK) == BQ24261_STAT_FAULT) {
		bool dump_master = true;
		chip->chrgr_stat = BQ24261_CHRGR_STAT_FAULT;

		switch (stat_reg & BQ24261_FAULT_MASK) {
		case BQ24261_VOVP:
			chip->chrgr_health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
			schedule_delayed_work(&chip->exception_mon_work,
					EXCEPTION_MONITOR_DELAY);
			dev_err(&client->dev, "Charger OVP Fault\n");
			break;

		case BQ24261_LOW_SUPPLY:
			notify = false;
			if (chip->pdata->handle_low_supply)
				chip->pdata->handle_low_supply();

			if (chip->cable_type !=
					POWER_SUPPLY_CHARGER_TYPE_NONE) {
				schedule_delayed_work
					(&chip->low_supply_fault_work,
					5*HZ);
				dev_dbg(&client->dev,
					"Schedule Low Supply Fault work!!\n");
			}
			return 0;
			break;

		case BQ24261_THERMAL_SHUTDOWN:
			chip->chrgr_health = POWER_SUPPLY_HEALTH_OVERHEAT;
			dev_err(&client->dev, "Charger Thermal Fault\n");
			break;

		case BQ24261_BATT_TEMP_FAULT:
			chip->bat_health = POWER_SUPPLY_HEALTH_OVERHEAT;
			dev_err(&client->dev, "Battery Temperature Fault\n");
			break;

		case BQ24261_TIMER_FAULT:
			chip->bat_health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
			chip->chrgr_health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
			dev_err(&client->dev, "Charger Timer Fault\n");
			break;

		case BQ24261_BATT_OVP:
			notify = false;
			if (chip->bat_health !=
					POWER_SUPPLY_HEALTH_OVERVOLTAGE) {
				if (!IS_BATTERY_OVER_VOLTAGE(chip)) {
					chip->chrgr_stat =
						BQ24261_CHRGR_STAT_UNKNOWN;
					resume_charging(chip);
				} else {
					dev_err(&client->dev, "Battery Over Voltage Fault\n");
					handle_battery_over_voltage(chip);
					notify = true;
				}
			}
			break;
		case BQ24261_NO_BATTERY:
			dev_err(&client->dev, "No Battery Connected\n");
			break;

		}

		if (chip->chrgr_stat == BQ24261_CHRGR_STAT_FAULT && notify)
			bq24261_dump_regs(dump_master);
	}

	wake_up(&chip->wait_ready);

	chip->is_vsys_on = bq24261_is_vsys_on(chip);
	if (notify)
		power_supply_changed(&chip->psy_usb);

	return 0;
}

static void bq24261_irq_worker(struct work_struct *work)
{
	struct bq24261_charger *chip =
	    container_of(work, struct bq24261_charger, irq_work);
	int ret;

	/*Lock to ensure that interrupt register readings are done
	* and processed sequentially. The interrupt Fault registers
	* are read on clear and without sequential processing double
	* fault interrupts or fault recovery cannot be handlled propely
	*/

	mutex_lock(&chip->stat_lock);

	dev_dbg(&chip->client->dev, "%s\n", __func__);

	ret = bq24261_read_reg(chip->client, BQ24261_STAT_CTRL0_ADDR);
	if (ret < 0)
		dev_err(&chip->client->dev,
			"Error (%d) in reading BQ24261_STAT_CTRL0_ADDR\n", ret);
	else
		bq24261_handle_irq(chip, ret);

	mutex_unlock(&chip->stat_lock);
}

static irqreturn_t bq24261_thread_handler(int id, void *data)
{
	struct bq24261_charger *chip = (struct bq24261_charger *)data;

	queue_work(system_nrt_wq, &chip->irq_work);
	return IRQ_HANDLED;
}

static irqreturn_t bq24261_irq_handler(int irq, void *data)
{
	struct bq24261_charger *chip = (struct bq24261_charger *)data;
	u8 intr_stat;

	if (chip->irq_iomap) {
		intr_stat = ioread8(chip->irq_iomap);
		if ((intr_stat & chip->pdata->irq_mask)) {
			dev_dbg(&chip->client->dev, "%s\n", __func__);
			return IRQ_WAKE_THREAD;
		}
	}

	return IRQ_NONE;
}

static void bq24261_boostmode_worker(struct work_struct *work)
{
	struct bq24261_charger *chip =
	    container_of(work, struct bq24261_charger, otg_work);
	struct bq24261_otg_event *evt, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&chip->otg_queue_lock, flags);
	list_for_each_entry_safe(evt, tmp, &chip->otg_queue, node) {
		list_del(&evt->node);
		spin_unlock_irqrestore(&chip->otg_queue_lock, flags);

		dev_info(&chip->client->dev,
			"%s:%d state=%d\n", __FILE__, __LINE__,
				evt->is_enable);
		mutex_lock(&chip->stat_lock);
		if (evt->is_enable)
			bq24261_enable_boost_mode(chip, 1);
		else
			bq24261_enable_boost_mode(chip, 0);

		mutex_unlock(&chip->stat_lock);
		spin_lock_irqsave(&chip->otg_queue_lock, flags);
		kfree(evt);

	}
	spin_unlock_irqrestore(&chip->otg_queue_lock, flags);
}

static int otg_handle_notification(struct notifier_block *nb,
				   unsigned long event, void *param)
{

	struct bq24261_charger *chip =
	    container_of(nb, struct bq24261_charger, otg_nb);
	struct bq24261_otg_event *evt;

	dev_dbg(&chip->client->dev, "OTG notification: %lu\n", event);
	if (!param || event != USB_EVENT_DRIVE_VBUS)
		return NOTIFY_DONE;

	evt = kzalloc(sizeof(*evt), GFP_ATOMIC);
	if (!evt) {
		dev_err(&chip->client->dev,
			"failed to allocate memory for OTG event\n");
		return NOTIFY_DONE;
	}

	evt->is_enable = *(int *)param;
	INIT_LIST_HEAD(&evt->node);

	spin_lock(&chip->otg_queue_lock);
	list_add_tail(&evt->node, &chip->otg_queue);
	spin_unlock(&chip->otg_queue_lock);

	queue_work(system_nrt_wq, &chip->otg_work);
	return NOTIFY_OK;
}

static inline int register_otg_notifications(struct bq24261_charger *chip)
{

	int retval;

	INIT_LIST_HEAD(&chip->otg_queue);
	INIT_WORK(&chip->otg_work, bq24261_boostmode_worker);
	spin_lock_init(&chip->otg_queue_lock);

	chip->otg_nb.notifier_call = otg_handle_notification;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0))
	chip->transceiver = usb_get_transceiver();
#else
	chip->transceiver = usb_get_phy(USB_PHY_TYPE_USB2);
#endif
	if (!chip->transceiver || IS_ERR(chip->transceiver)) {
		dev_err(&chip->client->dev, "failed to get otg transceiver\n");
		return -EINVAL;
	}
	retval = usb_register_notifier(chip->transceiver, &chip->otg_nb);
	if (retval) {
		dev_err(&chip->client->dev,
			"failed to register otg notifier\n");
		return -EINVAL;
	}

	return 0;
}

static enum bq2426x_model_num bq24261_get_model(int bq24261_rev_reg)
{
	switch (bq24261_rev_reg & BQ24261_REV_MASK) {
	case BQ24260_REV:
		return BQ24260;
	case BQ24261_REV:
	case BQ24261_2_3_REV:
		return BQ24261;
	default:
		return BQ2426X;
	}
}

static ssize_t dock_print_name(struct switch_dev *sdev, char *buf)
{
	const char *name;
	const char *name_dock = "dock";
	const char *name_undock = "undock";

	if (!buf)
		return -EINVAL;

	switch (switch_get_state(sdev)) {
	case 0:
		name = name_undock;
		break;
	case 1001:
		name = name_dock;
		break;
	default:
		name = NULL;
		break;
	}

	if (name)
		return snprintf(buf, MAX_STR_COPY, "%s\n", name);
	else
		return -EINVAL;
}

static ssize_t dock_print_state(struct switch_dev *sdev, char *buf)
{
	const char *state;
	const char *state_dock = "1001";
	const char *state_undock = "0";

	if (!buf)
		return -EINVAL;

	switch (switch_get_state(sdev)) {
	case 0:
		state = state_undock;
		break;
	case 1001:
		state = state_dock;
		break;
	default:
		state = NULL;
		break;
	}

	if (state)
		return snprintf(buf, MAX_STR_COPY, "%s\n", state);
	else
		return -EINVAL;
}

/* Dell Venue 7040 (aka Eaglespeak) has two BQ24261 ICs, as it features two batteries
*  The attachable keyboard's power supply is connected to the second IC's VBUS booster
*  This function checks, if the current IC is the second one.
*/
int is_second_ic(struct i2c_client *client)
{

//	return (strcmp("2-006b", dev_name(&client->dev)) == 0) ? 1 : 0;
	if (strcmp("2-006b", dev_name(&client->dev)) == 0)
		return 1;
	else
		return 0;
}

static int bq24261_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter;
	struct bq24261_charger *chip;
	int ret;
	int bq2426x_rev;
	enum bq2426x_model_num bq24261_rev_index;
	
	adapter = to_i2c_adapter(client->dev.parent);

	/* EP second BQ24261 IC */
	if (is_second_ic(client))
	{
		chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
		if (!chip) {
			dev_err(&client->dev, "mem alloc failed\n");
			return -ENOMEM;
		}

		i2c_set_clientdata(client, chip);
		chip->client = client;
		chip->dev = &client->dev;
		chip->pdata = client->dev.platform_data;

		chip_ic2 = chip;

		ic2_init_complete = 1;

		INIT_DELAYED_WORK(&chip->wdt_work,
					bq24261_wdt_reset_worker_ic2);

		schedule_delayed_work(&chip->wdt_work, WDT_RESET_DELAY);
	} else
	/* EP second BQ24261 IC end */
	{
		if (!client->dev.platform_data) {
			dev_err(&client->dev, "platform data is null");
			return -EFAULT;
		}

		if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
			dev_err(&client->dev,
				"I2C adapter %s doesn'tsupport BYTE DATA transfer\n",
				adapter->name);
			return -EIO;
		}

		bq2426x_rev = bq24261_read_reg(client, BQ24261_VENDOR_REV_ADDR);
		if (bq2426x_rev < 0) {
			dev_err(&client->dev,
				"Error (%d) in reading BQ24261_VENDOR_REV_ADDR\n", bq2426x_rev);
			return bq2426x_rev;
		}
		dev_info(&client->dev, "bq2426x revision: 0x%x found!!\n", bq2426x_rev);

		bq24261_rev_index = bq24261_get_model(bq2426x_rev);
		if ((bq2426x_rev & BQ24261_VENDOR_MASK) != BQ24261_VENDOR) {
			dev_err(&client->dev,
				"Invalid Vendor/Revision number in BQ24261_VENDOR_REV_ADDR: %d",
				bq2426x_rev);
			return -ENODEV;
		}

		chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
		if (!chip) {
			dev_err(&client->dev, "mem alloc failed\n");
			return -ENOMEM;
		}

		init_waitqueue_head(&chip->wait_ready);
		i2c_set_clientdata(client, chip);
		chip->client = client;
		chip->dev = &client->dev;
		chip->pdata = client->dev.platform_data;

		dev_err(&client->dev, "%s: num_supplicants:%d, name:%s\n", __func__,
				chip->pdata->num_supplicants,
				chip->pdata->supplied_to[0]);

		/* Remap IRQ map address to read the IRQ status */
		if ((chip->pdata->irq_map) && (chip->pdata->irq_mask)) {
			chip->irq_iomap = ioremap_nocache(chip->pdata->irq_map, 8);
			if (!chip->irq_iomap) {
				dev_err(&client->dev, "Failed: ioremap_nocache\n");
				return -EFAULT;
			}
		}

		/* sysfs for driving key */
		ret = device_create_file(&chip->client->dev,
				&dev_attr_kbd_key);
		if (ret) {
			dev_err(&chip->client->dev,
				"Failed to create sysfs: kbd_key\n");
		}
		/* Create the dock state for supporting keyboard on EP */
		sdock = kzalloc(sizeof(struct dock_switch_data),
							GFP_KERNEL);
		if (!sdock) {
			pr_info("%s mem alloc failed\n", __func__);
			return -ENOMEM;
		}


	    sdock->sdev.name = "dock";
		sdock->sdev.print_name = dock_print_name;
		sdock->sdev.print_state = dock_print_state;
		ret = switch_dev_register(&sdock->sdev);
		if (ret) {
			pr_info("%s switch registration failed!!\n",
			__func__);
		}
		chip->kbd_dock_state =
			EXTRA_DOCK_STATE_KEYBOARD_KEYRELEASE;

		/* Send the initial notification about the keyboard state */
		switch_set_state(&sdock->sdev, chip->kbd_dock_state);
		chip->psy_usb.name = DEV_NAME;
		chip->psy_usb.type = POWER_SUPPLY_TYPE_USB;
		chip->psy_usb.properties = bq24261_usb_props;
		chip->psy_usb.num_properties = ARRAY_SIZE(bq24261_usb_props);
		chip->psy_usb.get_property = bq24261_usb_get_property;
		chip->psy_usb.set_property = bq24261_usb_set_property;
		chip->psy_usb.supplied_to = chip->pdata->supplied_to;
		chip->psy_usb.num_supplicants = chip->pdata->num_supplicants;
		chip->psy_usb.throttle_states = chip->pdata->throttle_states;
		chip->psy_usb.num_throttle_states = chip->pdata->num_throttle_states;
		chip->psy_usb.supported_cables = POWER_SUPPLY_CHARGER_TYPE_USB;
		chip->max_cc = chip->pdata->max_cc;
		chip->max_cv = 4350;
		chip->chrgr_stat = BQ24261_CHRGR_STAT_UNKNOWN;
		chip->chrgr_health = POWER_SUPPLY_HEALTH_UNKNOWN;
		chip->revision = bq2426x_rev;

		strncpy(chip->model_name,
			bq24261_model_name[bq24261_rev_index].model_name,
			MODEL_NAME_SIZE);
		strncpy(chip->manufacturer, DEV_MANUFACTURER,
			DEV_MANUFACTURER_NAME_SIZE);

		mutex_init(&chip->stat_lock);
		wake_lock_init(&chip->chrgr_en_wakelock,
				WAKE_LOCK_SUSPEND, "chrgr_en_wakelock");
		ret = power_supply_register(&client->dev, &chip->psy_usb);
		if (ret) {
			dev_err(&client->dev, "Failed: power supply register (%d)\n",
				ret);
			iounmap(chip->irq_iomap);
			return ret;
		}

		INIT_DELAYED_WORK(&chip->sw_term_work, bq24261_sw_charge_term_worker);
		INIT_DELAYED_WORK(&chip->low_supply_fault_work,
					bq24261_low_supply_fault_work);
		INIT_DELAYED_WORK(&chip->exception_mon_work,
					bq24261_exception_mon_work);
		if (((chip->revision & BQ24261_REV_MASK) == BQ24261_REV) ||
				chip->pdata->is_wdt_kick_needed) {
			INIT_DELAYED_WORK(&chip->wdt_work,
						bq24261_wdt_reset_worker);
		}

		INIT_WORK(&chip->irq_work, bq24261_irq_worker);
		if (chip->client->irq) {
			ret = request_threaded_irq(chip->client->irq,
						   bq24261_irq_handler,
						   bq24261_thread_handler,
						   IRQF_SHARED|IRQF_NO_SUSPEND,
						   DEV_NAME, chip);
			if (ret) {
				dev_err(&client->dev, "Failed: request_irq (%d)\n",
					ret);
				iounmap(chip->irq_iomap);
				power_supply_unregister(&chip->psy_usb);
				return ret;
			}
		}

		if (IS_BATTERY_OVER_VOLTAGE(chip))
			handle_battery_over_voltage(chip);
		else
			chip->bat_health = POWER_SUPPLY_HEALTH_GOOD;

		if (register_otg_notifications(chip))
			dev_err(&client->dev, "Error in registering OTG notifications. Unable to supply power to Host\n");

		bq24261_client = client;
		power_supply_changed(&chip->psy_usb);
		bq24261_debugfs_init();

		chip->ext_booster_gpio = get_gpio_by_name(OTG_DCDC_EN_GPIO);
		if (chip->ext_booster_gpio < 0) {
			dev_info(&client->dev, "No external OTG voltage booster found,"
					"gpio(name: %s)\n", OTG_DCDC_EN_GPIO);
		} else {
			ret = gpio_request(chip->ext_booster_gpio, "OTG_DCDC_EN_GPIO");
			if (ret) {
				dev_err(&client->dev,
					"%s: Failed to request otg_gpio: error %d\n",
						__func__, ret);
				return ret;
			}
			ret = gpio_direction_output(chip->ext_booster_gpio, 0);
			if (ret) {
				dev_err(&client->dev,
					"%s: OTG GPIO_DCDC_EN write is failed\n",
					__func__, chip->ext_booster_gpio);
				return ret;
			}
		}

		/* This worker would monitor the battery voltage*/
		INIT_DELAYED_WORK(&chip->bat_mon_work,
						bq24261_bat_mon_work);
		bq24261_debugfs_init( );
	}

	return 0;
}

static int bq24261_remove(struct i2c_client *client)
{
	/* fcipaq: TODO This routine should probably also be run for the
	 * 2nd IC, but as this only occurs on shutdown, it's not urgent...
	 */
	struct bq24261_charger *chip = i2c_get_clientdata(client);

	/* Power down EP keyboard supply */
	if (ic2_init_complete) {
		cancel_delayed_work_sync(&chip_ic2->wdt_work);
		bq24261_wdt_boost_ic2(chip_ic2, 0);
	}

	if (client->irq)
		free_irq(client->irq, chip);

	flush_scheduled_work();
	wake_lock_destroy(&chip->chrgr_en_wakelock);
	if (chip->irq_iomap)
		iounmap(chip->irq_iomap);
	if (chip->transceiver)
		usb_unregister_notifier(chip->transceiver, &chip->otg_nb);

	power_supply_unregister(&chip->psy_usb);
	bq24261_debugfs_exit();

	if (chip->ext_booster_gpio > 0) {
		gpio_free(chip->ext_booster_gpio);
	}

	switch_dev_unregister(&sdock->sdev);

	return 0;
}

static int bq24261_suspend(struct device *dev)
{
	struct bq24261_charger *chip = dev_get_drvdata(dev);

	if (((chip->revision & BQ24261_REV_MASK) == BQ24261_REV) ||
			chip->pdata->is_wdt_kick_needed) {

		if (chip->boost_mode)
			cancel_delayed_work_sync(&chip->wdt_work);

		/* Power down EP keyboard supply */
		if (ic2_init_complete) {
			cancel_delayed_work_sync(&chip_ic2->wdt_work);
			bq24261_wdt_boost_ic2(chip_ic2, 0);
		}

	}
	dev_dbg(&chip->client->dev, "bq24261 suspend\n");
	return 0;
}

static int bq24261_resume(struct device *dev)
{
	struct bq24261_charger *chip = dev_get_drvdata(dev);

	if (((chip->revision & BQ24261_REV_MASK) == BQ24261_REV) ||
			chip->pdata->is_wdt_kick_needed) {
		if (chip->boost_mode)
			bq24261_enable_boost_mode(chip, 1);
		/* Power up EP keyboard supply */
		if (ic2_init_complete) {
			if (ep_kbd_pwr_state)
				bq24261_wdt_boost_ic2(chip_ic2, 1);
			schedule_delayed_work(&chip_ic2->wdt_work, WDT_RESET_DELAY);
		}
	}

	dev_dbg(&chip->client->dev, "bq24261 resume\n");
	return 0;
}

static int bq24261_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int bq24261_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int bq24261_runtime_idle(struct device *dev)
{

	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static const struct dev_pm_ops bq24261_pm_ops = {
	.suspend = bq24261_suspend,
	.resume = bq24261_resume,
	.runtime_suspend = bq24261_runtime_suspend,
	.runtime_resume = bq24261_runtime_resume,
	.runtime_idle = bq24261_runtime_idle,
};

static const struct i2c_device_id bq24261_id[] = {
	{DEV_NAME, 0},
	{"bq24261_ep", 0},
	{},
};

static void bq24261_shutdown(struct i2c_client *client)
{
	int ret;

	dev_info(&client->dev,"%s called\n", __func__);

	/* Power down EP keyboard supply */
	if (ic2_init_complete) {
		cancel_delayed_work_sync(&chip_ic2->wdt_work);
		bq24261_wdt_boost_ic2(chip_ic2, 0);
	}

	ret =  bq24261_read_modify_reg(client, BQ24261_STAT_CTRL0_ADDR,
                                   BQ24261_TMR_RST_MASK, BQ24261_TMR_RST);
	if (ret)
		dev_err(&client->dev, "Error (%d) in WDT reset\n", ret);
	else
		dev_info(&client->dev, "WDT reset\n");

}

MODULE_DEVICE_TABLE(i2c, bq24261_id);

static struct i2c_driver bq24261_driver = {
	.driver = {
		   .name = DEV_NAME,
		   .pm = &bq24261_pm_ops,
		   },
	.probe = bq24261_probe,
	.remove = bq24261_remove,
	.id_table = bq24261_id,
	.shutdown = bq24261_shutdown,
};

static int __init bq24261_init(void)
{
	return i2c_add_driver(&bq24261_driver);
}

module_init(bq24261_init);

static void __exit bq24261_exit(void)
{
	i2c_del_driver(&bq24261_driver);
}

module_exit(bq24261_exit);

MODULE_AUTHOR("Jenny TC <jenny.tc@intel.com>");
MODULE_DESCRIPTION("BQ24261 Charger Driver");
MODULE_LICENSE("GPL");

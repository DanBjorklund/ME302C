/*
 * Copyright (c) 2012, ASUSTek, Inc. All Rights Reserved.
 * Written by chris chang chris1_chang@asus.com
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>

#include "asus_battery.h"
#include "asus_battery_proc_fs.h"
#include "bq27520_battery_core.h"
#include "smb347_external_include.h"
#include <linux/HWVersion.h>
#include <linux/wakelock.h>

extern int Read_HW_ID(void);
extern int entry_mode;

//struct delayed_work battery_low_init_work;		//battery low init
struct delayed_work battery_poll_data_work;		//polling data
struct delayed_work detect_cable_work;			//check cable status
struct workqueue_struct *battery_work_queue=NULL;

/* wake lock to prevent S3 during charging */
struct wake_lock wakelock;
struct wake_lock wakelock_t;    // for wake_lokc_timout() useage

DEFINE_MUTEX(batt_info_mutex);

static char *supply_list[] = {
        "battery",
};

struct battery_info_reply batt_info = {
        .drv_status = DRV_NOT_READY,
        .cable_status = NO_CABLE,
#ifdef ME302C_ENG_BUILD
        .eng_charging_limit = true,
#endif
#ifndef ME302C_USER_BUILD
        .emerg_poll = false,
#endif
};

void asus_cancel_work()
{
    if (battery_work_queue) {
        BAT_DBG_E(" %s:\n", __func__);
        cancel_delayed_work_sync(&battery_poll_data_work);
        cancel_delayed_work_sync(&detect_cable_work);
    }
}

#ifndef ME302C_USER_BUILD
int asus_emerg_poll_read(char *page, char **start, off_t off, int count, int *eof, void *date)
{
        int len = 0;
        BAT_DBG_E(" %s:\n", __func__);

        mutex_lock(&batt_info_mutex);
        batt_info.emerg_poll = true;
        mutex_unlock(&batt_info_mutex);

        asus_queue_update_all();

        return len;
}

int asus_emerg_poll_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
        BAT_DBG_E(" %s:\n", __func__);
        return count;
}

int init_emerg_poll_toggle(void)
{
        struct proc_dir_entry *entry=NULL;

        entry = create_proc_entry("poll", 0666, NULL);
        if (!entry) {
                BAT_DBG_E("Unable to create /proc/poll\n");
                return -EINVAL;
        }
        entry->read_proc = asus_emerg_poll_read;
        entry->write_proc = asus_emerg_poll_write;

        return 0;
}
#endif

#ifdef ME302C_ENG_BUILD
int asus_charging_toggle_read(char *page, char **start, off_t off, int count, int *eof, void *date)
{
        int len = 0;

        BAT_DBG_E(" %s:\n", __func__);
        return len;
}

int asus_charging_toggle_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
        BAT_DBG_E(" %s:\n", __func__);

        struct battery_info_reply tmp_batt_info;
        bool eng_charging_limit = true;

        mutex_lock(&batt_info_mutex);
        tmp_batt_info = batt_info;
        mutex_unlock(&batt_info_mutex);

        eng_charging_limit = tmp_batt_info.eng_charging_limit;

        if (buffer[0] == '0') {
                /* turn on charging limit in eng mode */
                eng_charging_limit = true;
        }
        else if (buffer[0] == '1') {
                /* turn off charging limit in eng mode */
                eng_charging_limit = false;
        }

        tmp_batt_info.eng_charging_limit = eng_charging_limit;

        mutex_lock(&batt_info_mutex);
        batt_info = tmp_batt_info;
        mutex_unlock(&batt_info_mutex);

        asus_queue_update_all();

        return count;
}

int init_asus_charging_toggle(void)
{
        struct proc_dir_entry *entry=NULL;

        entry = create_proc_entry("asus_eng_charging_limit", 0666, NULL);
        if (!entry) {
                BAT_DBG_E("Unable to create asus_charging_toggle\n");
                return -EINVAL;
        }
        entry->read_proc = asus_charging_toggle_read;
        entry->write_proc = asus_charging_toggle_write;

        return 0;
}
#endif

static enum power_supply_property asus_battery_props[] = {
        POWER_SUPPLY_PROP_STATUS,				//0x00
        POWER_SUPPLY_PROP_HEALTH,				//0x02
        POWER_SUPPLY_PROP_PRESENT,				//0x03
        POWER_SUPPLY_PROP_TECHNOLOGY,			//0x05
        POWER_SUPPLY_PROP_VOLTAGE_NOW,			//0x0A
        POWER_SUPPLY_PROP_CURRENT_NOW,			//0x0C
        POWER_SUPPLY_PROP_ENERGY_NOW,			//0x1B
        POWER_SUPPLY_PROP_CAPACITY,				//0x1D
        POWER_SUPPLY_PROP_CAPACITY_LEVEL,		//0x1E
        POWER_SUPPLY_PROP_TEMP,					//0x1F
        POWER_SUPPLY_PROP_MANUFACTURER,
        POWER_SUPPLY_PROP_MODEL_NAME,
        POWER_SUPPLY_PROP_SERIAL_NUMBER,
        POWER_SUPPLY_PROP_CURRENT_AVG,
#ifdef CONFIG_ME302C_BATTERY
        POWER_SUPPLY_PROP_BATTERY_ID,
        POWER_SUPPLY_PROP_FIRMWARE_VERSION,
        #ifdef CONFIG_ME302C_BATTERY_BQ27520
        POWER_SUPPLY_PROP_CHEMICAL_ID,
        POWER_SUPPLY_PROP_FW_CFG_VER,
        #endif
#endif
};

static enum power_supply_property asus_power_properties[] = {
        POWER_SUPPLY_PROP_PRESENT,	//0x03
        POWER_SUPPLY_PROP_ONLINE,	//0x04
};

static int asus_battery_get_property(struct power_supply *psy, 
                enum power_supply_property psp,
                union power_supply_propval *val);

static int asus_power_get_property(struct power_supply *psy, 
                enum power_supply_property psp,
                union power_supply_propval *val);

static struct power_supply asus_power_supplies[] = {
        {
                .name = "battery",
                .type = POWER_SUPPLY_TYPE_BATTERY,
                .properties = asus_battery_props,
                .num_properties = ARRAY_SIZE(asus_battery_props),
                .get_property = asus_battery_get_property,
        },
        {
                .name = "ac",
                .type = POWER_SUPPLY_TYPE_MAINS,
                .supplied_to = supply_list,
                .num_supplicants = ARRAY_SIZE(supply_list),
                .properties = asus_power_properties,
                .num_properties = ARRAY_SIZE(asus_power_properties),
                .get_property = asus_power_get_property,
        },
        {
                .name = "usb",
                .type = POWER_SUPPLY_TYPE_USB,
                .supplied_to = supply_list,
                .num_supplicants = ARRAY_SIZE(supply_list),
                .properties = asus_power_properties,
                .num_properties = ARRAY_SIZE(asus_power_properties),
                .get_property = asus_power_get_property,
        },
};

int asus_battery_low_event()
{
        drv_status_t drv_status;

        mutex_lock(&batt_info_mutex);    
        drv_status = batt_info.drv_status;
        mutex_unlock(&batt_info_mutex);

        drv_status = DRV_BATTERY_LOW;

        mutex_lock(&batt_info_mutex);    
        batt_info.drv_status = drv_status;
        mutex_unlock(&batt_info_mutex);

        power_supply_changed(&asus_power_supplies[CHARGER_BATTERY]);
        power_supply_changed(&asus_power_supplies[CHARGER_USB]);
        power_supply_changed(&asus_power_supplies[CHARGER_AC]);

        return 0;
}

/*
 * Return the battery full charge capacity in mAh
 * Or < 0 if something fails.
 */
static int asus_battery_update_fcc(void)
{
        int fcc = -EINVAL;
        struct battery_info_reply tmp_batt_info;

        mutex_lock(&batt_info_mutex);
        tmp_batt_info = batt_info;
        mutex_unlock(&batt_info_mutex);

        if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_fcc) {
                fcc = tmp_batt_info.tbl->read_fcc();
        }

        return fcc;
}
static int asus_battery_update_fcc_no_mutex(void)
{
        int fcc = -EINVAL;
        struct battery_info_reply tmp_batt_info;

        tmp_batt_info = batt_info;
        if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_fcc)
                fcc = tmp_batt_info.tbl->read_fcc();

        return fcc;
}

/*
 * Return the battery nominal available capacity
 * Or < 0 if something fails.
 */
static int asus_battery_update_nac(void)
{
        int nac = -EINVAL;
        struct battery_info_reply tmp_batt_info;

        mutex_lock(&batt_info_mutex);
        tmp_batt_info = batt_info;
        mutex_unlock(&batt_info_mutex);

        if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_nac) {
                nac = tmp_batt_info.tbl->read_nac();
        }

        return nac;
}
static int asus_battery_update_nac_no_mutex(void)
{
        int nac = -EINVAL;
        struct battery_info_reply tmp_batt_info;

        tmp_batt_info = batt_info;
        if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_nac)
                nac = tmp_batt_info.tbl->read_nac();

        return nac;
}

/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int asus_battery_update_temp(void)
{
        int temperature = -EINVAL;
        struct battery_info_reply tmp_batt_info;

        mutex_lock(&batt_info_mutex);
        tmp_batt_info = batt_info;
        mutex_unlock(&batt_info_mutex);

        if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_temp) {
                temperature = tmp_batt_info.tbl->read_temp();
        }

        return temperature;
}
static int asus_battery_update_temp_no_mutex(void)
{
        int temperature = -EINVAL;
        struct battery_info_reply tmp_batt_info;

        tmp_batt_info = batt_info;
        if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_temp)
                temperature = tmp_batt_info.tbl->read_temp();

        return temperature;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */

static int asus_battery_update_voltage(void)
{
        int volt = -EINVAL;
        struct battery_info_reply tmp_batt_info;

        mutex_lock(&batt_info_mutex);
        tmp_batt_info = batt_info;
        mutex_unlock(&batt_info_mutex);

        if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_volt) {
                volt = tmp_batt_info.tbl->read_volt();
        }

        return volt;
}
static int asus_battery_update_voltage_no_mutex(void)
{
        int volt = -EINVAL;
        struct battery_info_reply tmp_batt_info;

        tmp_batt_info = batt_info;
        if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_volt)
                volt = tmp_batt_info.tbl->read_volt();

        return volt;
}

/*
 * Return the battery average current
 * Note that current can be negative signed as well(-65536 ~ 65535).
 * So, the value get from device need add a base(0x10000) to be a positive number if no error. 
 * Or < 0 if something fails.
 */
static int asus_battery_update_current(void)
{
        int curr = -EINVAL;
        struct battery_info_reply tmp_batt_info;

        mutex_lock(&batt_info_mutex);
        tmp_batt_info = batt_info;
        mutex_unlock(&batt_info_mutex);

        if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_current) {
                curr = tmp_batt_info.tbl->read_current();
        }

        return curr;
}
static int asus_battery_update_current_no_mutex(void)
{
        int curr = -EINVAL;
        struct battery_info_reply tmp_batt_info;

        tmp_batt_info = batt_info;
        if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_current)
                curr = tmp_batt_info.tbl->read_current();

        return curr;
}

#if CURRENT_IC_VERSION == IC_VERSION_G3
static int asus_battery_update_available_energy(void)
{
        int mWhr = -EINVAL;
        struct battery_info_reply tmp_batt_info;

        mutex_lock(&batt_info_mutex);
        tmp_batt_info = batt_info;
        mutex_unlock(&batt_info_mutex);

        if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_av_energy) {
                mWhr = tmp_batt_info.tbl->read_av_energy();
        }

        return mWhr;
}
static int asus_battery_update_available_energy_no_mutex(void)
{
        int mWhr = -EINVAL;
        struct battery_info_reply tmp_batt_info;

        tmp_batt_info = batt_info;
        if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_av_energy)
                mWhr = tmp_batt_info.tbl->read_av_energy();

        return mWhr;
}
#endif

#ifdef BATTERY_PERCENTAGE_SHIFT
static int percentage_shift(int percentage)
{
        int ret;

        ret = ((ret >= 100) ? 100 : percentage);

        /* start percentage shift:
           battery percentage remapping according to battery discharging curve
        */

        if (ret == 99) {
                ret = 100;
                return ret;
        }
        else if (ret >= 84 && ret <= 98) {
                ret++;
                return ret;
        }

        /* missing 26%, 47%, 58%, 69%, 79% */
        if (ret > 70 && ret < 80)
                ret -= 1;
        else if (ret > 60 && ret <= 70)
                ret -= 2;
        else if (ret > 50 && ret <= 60)
                ret -= 3;
        else if (ret > 30 && ret <= 50)
                ret -= 4;
        else if (ret >= 0 && ret <= 30)
                ret -= 5;

        /* final check to avoid negative value */
        if (ret < 0)
                ret = 0;

        return ret;
}
#endif

static int asus_battery_update_percentage(void)
{
        int percentage = -EINVAL;
        drv_status_t drv_status;
        struct battery_info_reply tmp_batt_info;

        mutex_lock(&batt_info_mutex);
        tmp_batt_info = batt_info;
        drv_status = batt_info.drv_status;
        mutex_unlock(&batt_info_mutex);

        if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_percentage) {
                percentage = tmp_batt_info.tbl->read_percentage();

                if (percentage == ERROR_CODE_I2C_FAILURE)
                    return -EINVAL;

                if (percentage >= 0 && percentage <= 3) {
                        percentage = 0;
                        drv_status = DRV_SHUTDOWN;
                }

                /* chris: illegal RSOC check for UPI gauge IC */
                if (percentage < 0) {
                    printk(KERN_WARNING "*** Wrong battery percentage = %d ***\n", percentage);
                    percentage = 0;
                }
                if (percentage > 100) {
                    printk(KERN_WARNING "*** Wrong battery percentage = %d ***\n", percentage);
                    percentage = 100;
                }

                mutex_lock(&batt_info_mutex);
                batt_info.drv_status = drv_status;
                mutex_unlock(&batt_info_mutex);
        }

        return percentage; //% return adjust percentage, and error code ( < 0)
}
static int asus_battery_update_percentage_no_mutex(u32* rawpercentage)
{
        int percentage = -EINVAL;
        drv_status_t drv_status;
        struct battery_info_reply tmp_batt_info;

        tmp_batt_info = batt_info;
        drv_status = batt_info.drv_status;

        if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_percentage) {
                percentage = tmp_batt_info.tbl->read_percentage();

                if (percentage == ERROR_CODE_I2C_FAILURE)
                    return -EINVAL;

                *rawpercentage = percentage;
#ifdef BATTERY_PERCENTAGE_SHIFT
                percentage = percentage_shift(percentage);
#else
                if (percentage >= 0 && percentage <= 3) {
                        percentage = 0;
                        drv_status = DRV_SHUTDOWN;
                }
#endif

                batt_info.drv_status = drv_status;
        }

        return percentage; //% return adjust percentage, and error code ( < 0)
}

//range:   range_min <= X < range_max
struct lvl_entry {
        char name[30];
        int range_min;
        int range_max;
        int ret_val;
};

#define MAX_DONT_CARE    1000000
#define MIN_DONT_CARE    -999999
struct lvl_entry lvl_tbl[] = {
        { "FULL",      100,    MAX_DONT_CARE,   POWER_SUPPLY_CAPACITY_LEVEL_FULL},  
        { "HIGH",       80,              100,   POWER_SUPPLY_CAPACITY_LEVEL_HIGH},  
        { "NORMAL",     20,               80,   POWER_SUPPLY_CAPACITY_LEVEL_NORMAL},  
        { "LOW",         5,               20,   POWER_SUPPLY_CAPACITY_LEVEL_LOW},  
        { "CRITICAL",    0,                5,   POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL},  
        { "UNKNOWN", MIN_DONT_CARE,  MAX_DONT_CARE,  POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN},  
};

static int asus_battery_update_capacity_level(int percentage)
{	
        int i=0;

        for (i=0; i<sizeof(lvl_tbl)/sizeof(struct lvl_entry); i++) {
                if (lvl_tbl[i].range_max != MAX_DONT_CARE && 
                        percentage >= lvl_tbl[i].range_max)
                        continue;
                if (lvl_tbl[i].range_min != MIN_DONT_CARE && 
                        lvl_tbl[i].range_min > percentage)
                        continue;
                //printk(KERN_WARNING "<BATT> battery level = %s\n", lvl_tbl[i].name);
                return lvl_tbl[i].ret_val;
        }

        return -EINVAL;
}

static int asus_battery_update_status_no_mutex(int percentage)
{
        int status;
        int temperature;
        struct battery_info_reply tmp_batt_info;
        u32 cable_status;

        tmp_batt_info = batt_info;
        cable_status = tmp_batt_info.cable_status;

        if (cable_status == USB_ADAPTER || cable_status == USB_PC) {
                status = POWER_SUPPLY_STATUS_CHARGING;

#ifdef ME302C_ENG_BUILD
                /* ME371MG, ME302C eng mode : stop charging when battery percentage is over 60% */
                if (percentage >= 60 && tmp_batt_info.eng_charging_limit) {
                        smb347_charging_toggle(false);
                        status = POWER_SUPPLY_STATUS_DISCHARGING;
                        goto final;
                }
#endif

                /* ME371MG, ME302C
                stop charging to protect battery from damaged when
                battery temperature is too low or too high */
                temperature = asus_battery_update_temp_no_mutex();
                if (temperature != ERROR_CODE_I2C_FAILURE) {
                        if (temperature < 0) {
                                smb347_charging_toggle(false);
                                status = POWER_SUPPLY_STATUS_DISCHARGING;
                                BAT_DBG_E( "*Error*: Force stop charging due to that battery temperature(%d) is lower than 0 (0.1C)", temperature);
                                goto final;
                        }
                        else if (temperature > 500) {
                                smb347_charging_toggle(false);
                                status = POWER_SUPPLY_STATUS_DISCHARGING;
                                BAT_DBG_E( "*Error*: Force stop charging due to that battery temperature(%d) is higher than 500 (0.1C)", temperature);
                                goto final;
                        }
                }

                if (status == POWER_SUPPLY_STATUS_CHARGING)
                        smb347_charging_toggle(true);
                if (percentage == 100)
                        status = POWER_SUPPLY_STATUS_FULL;
        }
        else {
                if (percentage == 100)
                        status = POWER_SUPPLY_STATUS_FULL;
                else
                        status = POWER_SUPPLY_STATUS_DISCHARGING;
        }

        if (percentage < 0)
                status = POWER_SUPPLY_STATUS_UNKNOWN;
final:
        return status;
}

static void print_battery_info_no_mutex()
{
        int temperature;
        int temperature10;

        /* printk(...) does not support %f and %e (float point and double) */
        if (batt_info.batt_temp >= 10 || batt_info.batt_temp <= -10) {
                temperature10 = batt_info.batt_temp/10;
                temperature = batt_info.batt_temp - (temperature10 * 10);
        }
        else {
                temperature10 = 0;
                temperature = batt_info.batt_temp;
        }

#if CURRENT_IC_VERSION == IC_VERSION_G3
        if (batt_info.status == POWER_SUPPLY_STATUS_FULL)
                printk(KERN_WARNING
                "<BATT> P:%d(%d)%%, V:%dmV, C:%dmA, T:%d.%dC, E:%dmWh, S:FULL, N:%dmAh, L:%dmAh, POLL:%ds\n",
                    batt_info.percentage,
                    batt_info.raw_percentage,
                    batt_info.batt_volt,
                    batt_info.batt_current,
                    temperature10,
                    temperature,
                    batt_info.batt_energy,
                    batt_info.nac,
                    batt_info.fcc,
                    batt_info.polling_time/HZ);
        else if (batt_info.status == POWER_SUPPLY_STATUS_CHARGING)
                printk(KERN_WARNING
                "<BATT> P:%d(%d)%%, V:%dmV, C:%dmA, T:%d.%dC, E:%dmWh, S:CHARGING, N:%dmAh, L:%dmAh, POLL:%ds\n",
                    batt_info.percentage,
                    batt_info.raw_percentage,
                    batt_info.batt_volt,
                    batt_info.batt_current,
                    temperature10,
                    temperature,
                    batt_info.batt_energy,
                    batt_info.nac,
                    batt_info.fcc,
                    batt_info.polling_time/HZ);
        else if (batt_info.status == POWER_SUPPLY_STATUS_DISCHARGING)
                printk(KERN_WARNING
                "<BATT> P:%d(%d)%%, V:%dmV, C:%dmA, T:%d.%dC, E:%dmWh, S:DISCHARGING, N:%dmAh, L:%dmAh, POLL:%ds\n",
                    batt_info.percentage,
                    batt_info.raw_percentage,
                    batt_info.batt_volt,
                    batt_info.batt_current,
                    temperature10,
                    temperature,
                    batt_info.batt_energy,
                    batt_info.nac,
                    batt_info.fcc,
                    batt_info.polling_time/HZ);
        else if (batt_info.status == POWER_SUPPLY_STATUS_NOT_CHARGING)
                printk(KERN_WARNING
                "<BATT> P:%d(%d)%%, V:%dmV, C:%dmA, T:%d.%dC, E:%dmWh, S:NOT_CHARGING, N:%dmAh, L:%dmAh, POLL:%ds\n",
                    batt_info.percentage,
                    batt_info.raw_percentage,
                    batt_info.batt_volt,
                    batt_info.batt_current,
                    temperature10,
                    temperature,
                    batt_info.batt_energy,
                    batt_info.nac,
                    batt_info.fcc,
                    batt_info.polling_time/HZ);
        else if (batt_info.status == POWER_SUPPLY_STATUS_UNKNOWN)
                printk(KERN_WARNING
                "<BATT> P:%d(%d)%%, V:%dmV, C:%dmA, T:%d.%dC, E:%dmWh, S:UNKNOWN, N:%dmAh, L:%dmAh, POLL:%ds\n",
                    batt_info.percentage,
                    batt_info.raw_percentage,
                    batt_info.batt_volt,
                    batt_info.batt_current,
                    temperature10,
                    temperature,
                    batt_info.batt_energy,
                    batt_info.nac,
                    batt_info.fcc,
                    batt_info.polling_time/HZ);
#else
        if (batt_info.status == POWER_SUPPLY_STATUS_FULL)
                printk(KERN_WARNING
                "<BATT> P:%d(%d)%%, V:%dmV, C:%dmA, T:%d.%dC, S:FULL, N:%dmAh, L:%dmAh, POLL:%ds\n",
                    batt_info.percentage,
                    batt_info.raw_percentage,
                    batt_info.batt_volt,
                    batt_info.batt_current,
                    temperature10,
                    temperature,
                    batt_info.nac,
                    batt_info.fcc,
                    batt_info.polling_time/HZ);
        else if (batt_info.status == POWER_SUPPLY_STATUS_CHARGING)
                printk(KERN_WARNING
                "<BATT> P:%d(%d)%%, V:%dmV, C:%dmA, T:%d.%dC, S:CHARGING, N:%dmAh, L:%dmAh, POLL:%ds\n",
                    batt_info.percentage,
                    batt_info.raw_percentage,
                    batt_info.batt_volt,
                    batt_info.batt_current,
                    temperature10,
                    temperature,
                    batt_info.nac,
                    batt_info.fcc,
                    batt_info.polling_time/HZ);
        else if (batt_info.status == POWER_SUPPLY_STATUS_DISCHARGING)
                printk(KERN_WARNING
                "<BATT> P:%d(%d)%%, V:%dmV, C:%dmA, T:%d.%dC, S:DISCHARGING, N:%dmAh, L:%dmAh, POLL:%ds\n",
                    batt_info.percentage,
                    batt_info.raw_percentage,
                    batt_info.batt_volt,
                    batt_info.batt_current,
                    temperature10,
                    temperature,
                    batt_info.nac,
                    batt_info.fcc,
                    batt_info.polling_time/HZ);
        else if (batt_info.status == POWER_SUPPLY_STATUS_NOT_CHARGING)
                printk(KERN_WARNING
                "<BATT> P:%d(%d)%%, V:%dmV, C:%dmA, T:%d.%dC, S:NOT_CHARGING, N:%dmAh, L:%dmAh, POLL:%ds\n",
                    batt_info.percentage,
                    batt_info.raw_percentage,
                    batt_info.batt_volt,
                    batt_info.batt_current,
                    temperature10,
                    temperature,
                    batt_info.nac,
                    batt_info.fcc,
                    batt_info.polling_time/HZ);
        else if (batt_info.status == POWER_SUPPLY_STATUS_UNKNOWN)
                printk(KERN_WARNING
                "<BATT> P:%d(%d)%%, V:%dmV, C:%dmA, T:%d.%dC, S:UNKNOWN, N:%dmAh, L:%dmAh, POLL:%ds\n",
                    batt_info.percentage,
                    batt_info.raw_percentage,
                    batt_info.batt_volt,
                    batt_info.batt_current,
                    temperature10,
                    temperature,
                    batt_info.nac,
                    batt_info.fcc,
                    batt_info.polling_time/HZ);
#endif
}

static void asus_battery_get_info_no_mutex(void)
{
        struct battery_info_reply tmp_batt_info;
        static int pre_batt_percentage = -1;
        static int pre_cable_status = -1;
        int tmp=0;

        tmp_batt_info = batt_info;

        tmp = asus_battery_update_voltage_no_mutex();
        if (tmp >= 0) tmp_batt_info.batt_volt = tmp;

        tmp = asus_battery_update_current_no_mutex();
        if (tmp >= 0) tmp_batt_info.batt_current = tmp - 0x10000;

        tmp = asus_battery_update_percentage_no_mutex(&tmp_batt_info.raw_percentage);
        if (tmp >= 0) {
                if (pre_cable_status < 0)
                        pre_cable_status = tmp_batt_info.cable_status;
                if (pre_batt_percentage<0) {
                        tmp_batt_info.percentage = tmp;
                        pre_batt_percentage = tmp;
                        BAT_DBG(" Init the battery percentage values = %d(%d)\n", pre_batt_percentage, tmp_batt_info.raw_percentage);
                }
                else if ((tmp_batt_info.cable_status == NO_CABLE) && (pre_cable_status == NO_CABLE) && ( pre_batt_percentage < tmp)) {
                        tmp_batt_info.percentage = pre_batt_percentage;
                        printk(KERN_WARNING "<BATT> keep pre battery percentage = (pre:%d, now:%d)\n", pre_batt_percentage, tmp);
                }
                else {
                        /* FIX: suddently battery percentage drop while
                           it is nearly battery low (TT259005).
                           We adopt the Dichotomy method to report the percentage smoothly
                         */
                        if (tmp < 4 && pre_batt_percentage > 5) {
                            printk(KERN_WARNING "<BATT> modify dropping percentage = (now:%d, mod:%d)\n", tmp, (tmp+pre_batt_percentage)/2);
                            tmp = (tmp + pre_batt_percentage) / 2;
                        }
                        tmp_batt_info.percentage = tmp;
                        pre_batt_percentage = tmp;
                }
                pre_cable_status = tmp_batt_info.cable_status;
        }
        else {
                /* DEF: define the returned value of battery capacity
                        with -99 when i2c communication failure.
                */
                tmp_batt_info.percentage = -99;
        }

        tmp = asus_battery_update_capacity_level(tmp_batt_info.percentage);
        if (tmp >= 0) tmp_batt_info.level = tmp;

        tmp = asus_battery_update_status_no_mutex(tmp_batt_info.percentage);
        if (tmp >= 0) tmp_batt_info.status = tmp;

#if CURRENT_IC_VERSION == IC_VERSION_G3
        tmp = asus_battery_update_available_energy_no_mutex();
        if (tmp >= 0) tmp_batt_info.batt_energy = tmp;
#endif
        tmp = asus_battery_update_fcc_no_mutex();
        if (tmp != ERROR_CODE_I2C_FAILURE && tmp != -EINVAL) tmp_batt_info.fcc = tmp;

        tmp = asus_battery_update_nac_no_mutex();
        if (tmp != ERROR_CODE_I2C_FAILURE && tmp != -EINVAL) tmp_batt_info.nac = tmp;

        tmp = asus_battery_update_temp_no_mutex();
        if (tmp != ERROR_CODE_I2C_FAILURE && tmp != -EINVAL) tmp_batt_info.batt_temp = tmp;

        if (tmp_batt_info.percentage >= 50 && tmp_batt_info.percentage <= 100)
                tmp_batt_info.polling_time = 60*HZ;
        else if (tmp_batt_info.percentage >= 20 && tmp_batt_info.percentage <= 49)
                tmp_batt_info.polling_time = 30*HZ;
        else if (tmp_batt_info.percentage >= 5 && tmp_batt_info.percentage <= 19)
                tmp_batt_info.polling_time = 10*HZ;
        else if (tmp_batt_info.percentage >= 0 && tmp_batt_info.percentage <= 4)
                tmp_batt_info.polling_time = 5*HZ;
        else {
                BAT_DBG_E("*** Battery percentage is out of the legal range (0% ~ 100%) ***\n");
                tmp_batt_info.polling_time = 5*HZ;
        }

        /* ME371MG, ME302C battery polling algorithm */
        /* ME371MG, ME302C
        According to charger IC and gauge IC spec. */
        if (tmp_batt_info.batt_temp >= 630) {
                BAT_DBG("Critical condition!! -> temperature \n");
                tmp_batt_info.polling_time = BATTERY_CRITICAL_POLL_TIME;
        }
        else if (tmp_batt_info.batt_temp >= 500) {
                BAT_DBG("Nearly critical condition!! -> high battery temperature > 50.0C\n");
                if (tmp_batt_info.polling_time > (10*HZ)) tmp_batt_info.polling_time = 10*HZ;
        }
        else if (tmp_batt_info.batt_temp < -100) {
                BAT_DBG("Critical condition!! -> low battery temperature < -10.0C\n");
                tmp_batt_info.polling_time = BATTERY_CRITICAL_POLL_TIME;
        }
        else if (tmp_batt_info.batt_temp < 30) {
                BAT_DBG("Nearly critical condition!! -> low battery temperature < 3.0C\n");
                if (tmp_batt_info.polling_time > (10*HZ)) tmp_batt_info.polling_time = 10*HZ;
        }

#ifndef ME302C_USER_BUILD
        /* battery emergency polling time interval */
        if (batt_info.emerg_poll)
                tmp_batt_info.polling_time = BATTERY_EMERGENCY_POLL_TIME;
#endif

        batt_info = tmp_batt_info;
        print_battery_info_no_mutex();
}

static void asus_polling_data(struct work_struct *dat)
{
        u32 polling_time = 60*HZ;

        polling_time = asus_update_all();
        queue_delayed_work(battery_work_queue, &battery_poll_data_work, polling_time);
}

u32 asus_update_all(void)
{
        u32 time;

        mutex_lock(&batt_info_mutex);

        asus_battery_get_info_no_mutex();
        time = batt_info.polling_time;
        // if the power source changes, all power supplies may change state 
        power_supply_changed(&asus_power_supplies[CHARGER_BATTERY]);
        power_supply_changed(&asus_power_supplies[CHARGER_AC]);
        power_supply_changed(&asus_power_supplies[CHARGER_USB]);

        mutex_unlock(&batt_info_mutex);

        return time;
}

static void USB_cable_status_worker(struct work_struct *dat)
{
        asus_update_all();
}

void asus_queue_update_all(void)
{
        queue_delayed_work(battery_work_queue, &detect_cable_work, 0.1*HZ);
}

void usb_to_battery_callback(u32 usb_cable_state)
{
        drv_status_t drv_sts;    
#if 0
        BAT_DBG_E("%s\n", __func__) ;
#endif
        mutex_lock(&batt_info_mutex);
        drv_sts = batt_info.drv_status;
        mutex_unlock(&batt_info_mutex);

        mutex_lock(&batt_info_mutex);
        batt_info.cable_status = usb_cable_state;
        mutex_unlock(&batt_info_mutex);

        if (drv_sts != DRV_REGISTER_OK) {
                BAT_DBG_E("Battery module not ready\n");
                return;
        }

        mutex_lock(&batt_info_mutex);

        /* prevent system from entering s3 in COS while AC charger is connected */
        if (entry_mode == 4) {
            if (batt_info.cable_status == USB_ADAPTER) {
                if (!wake_lock_active(&wakelock)) {
                    BAT_DBG(" %s: asus_battery_power_wakelock -> wake lock\n", __func__);
                    wake_lock(&wakelock);
                }
            }
            else if (batt_info.cable_status == NO_CABLE) {
                if (wake_lock_active(&wakelock)) {
                    BAT_DBG(" %s: asus_battery_power_wakelock -> wake unlock\n", __func__);
                    wake_lock_timeout(&wakelock_t, 3*HZ);  // timeout value as same as the <charger.exe>\asus_global.h #define ASUS_UNPLUGGED_SHUTDOWN_TIME(3 sec)
                    wake_unlock(&wakelock);
                } else { // for PC case
                    wake_lock_timeout(&wakelock_t, 3*HZ);
                }
            }
        }

        mutex_unlock(&batt_info_mutex);

        queue_delayed_work(battery_work_queue, &detect_cable_work, 0.1*HZ);
}
EXPORT_SYMBOL(usb_to_battery_callback);

int receive_USBcable_type(void)
{
        u32 cable_status;
        struct battery_info_reply tmp_batt_info;

        BAT_DBG("%s\n", __func__);

        mutex_lock(&batt_info_mutex);
        tmp_batt_info = batt_info;
        mutex_unlock(&batt_info_mutex);

        cable_status = tmp_batt_info.cable_status ;

        return cable_status; 
}
EXPORT_SYMBOL(receive_USBcable_type);

static int asus_battery_get_property(struct power_supply *psy,
                enum power_supply_property psp,
                union power_supply_propval *val)
{
        int ret = 0;
        struct battery_info_reply tmp_batt_info;

        //BAT_DBG("%s 0x%04X\n", __func__, psp);

        mutex_lock(&batt_info_mutex);
        tmp_batt_info = batt_info;
        mutex_unlock(&batt_info_mutex);

        switch (psp) {
        case POWER_SUPPLY_PROP_STATUS:
                val->intval = tmp_batt_info.status;
                break;
        case POWER_SUPPLY_PROP_HEALTH:
                val->intval = POWER_SUPPLY_HEALTH_GOOD;
                break;
        case POWER_SUPPLY_PROP_PRESENT:
                val->intval = tmp_batt_info.present;
                break;
        case POWER_SUPPLY_PROP_TECHNOLOGY:
                val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
                break;
        case POWER_SUPPLY_PROP_VOLTAGE_NOW:
                val->intval = tmp_batt_info.batt_volt;
                /* ME371MG, ME302C: change the voltage unit from Milli Voltage (mV) to Micro Voltage (uV) */
                val->intval *= 1000;
                break;
        case POWER_SUPPLY_PROP_CURRENT_NOW:
                /* ME371MG, ME302C: change the current unit from Milli Ampere (mA) to Micro Ampere (uA) */
                val->intval = tmp_batt_info.batt_current;
                val->intval *= 1000;
                break;
        case POWER_SUPPLY_PROP_CURRENT_AVG:
                val->intval = tmp_batt_info.batt_current;
                break;
        case POWER_SUPPLY_PROP_ENERGY_NOW:
                val->intval = tmp_batt_info.batt_energy;
                break;
        case POWER_SUPPLY_PROP_CAPACITY:
                val->intval = tmp_batt_info.percentage;
                break;
        case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
                val->intval = tmp_batt_info.level;
                break;
        case POWER_SUPPLY_PROP_MANUFACTURER:
        case POWER_SUPPLY_PROP_BATTERY_ID:
                val->strval = tmp_batt_info.manufacturer;
                break;
        case POWER_SUPPLY_PROP_MODEL_NAME:
                val->strval = tmp_batt_info.model;
                break;
        case POWER_SUPPLY_PROP_SERIAL_NUMBER:
        case POWER_SUPPLY_PROP_FIRMWARE_VERSION:
                val->strval = tmp_batt_info.serial_number;
                break;
#ifdef CONFIG_ME302C_BATTERY_BQ27520
        case POWER_SUPPLY_PROP_CHEMICAL_ID:
                val->strval = tmp_batt_info.chemical_id;
                break;
        case POWER_SUPPLY_PROP_FW_CFG_VER:
                val->strval = tmp_batt_info.fw_cfg_version;
                break;
#endif
        case POWER_SUPPLY_PROP_TEMP:
                val->intval = tmp_batt_info.batt_temp;
                break;
                        //	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
                        //		ret = bq27541_battery_time(BQ27541_REG_TTE, val);
                        //		break;
                        //	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
                        //		ret = bq27541_battery_time(BQ27541_REG_TTECP, val);
                        //		break;
                        //	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
                        //		ret = bq27541_battery_time(BQ27541_REG_TTF, val);
                        //		break;
        default:
                return -EINVAL;
        }

        return ret;
}

static int asus_power_get_property(struct power_supply *psy, 
                enum power_supply_property psp,
                union power_supply_propval *val)
{
        int ret;
        struct battery_info_reply tmp_batt_info;
        u32 cable_status;
        u32 drv_status;

        mutex_lock(&batt_info_mutex);
        tmp_batt_info = batt_info;
        mutex_unlock(&batt_info_mutex);

        cable_status = tmp_batt_info.cable_status;
        drv_status = tmp_batt_info.drv_status;

        ret = 0;
        switch (psp) {
        case POWER_SUPPLY_PROP_ONLINE:
                if (drv_status == DRV_SHUTDOWN) {
                        val->intval = 0;

                }else if (drv_status == DRV_BATTERY_LOW) {
                        val->intval = 0;

#if 0
                } else if (psy->type == POWER_SUPPLY_TYPE_USB || psy->type == POWER_SUPPLY_TYPE_MAINS) {
                        val->intval = ((cable_status == USB_ADAPTER || 
                                        cable_status == USB_PC) ? 1 : 0);
#endif
                } else if (psy->type == POWER_SUPPLY_TYPE_USB) {
                        val->intval = (cable_status == USB_PC) ? 1 : 0;

                } else if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
                        val->intval = (cable_status == USB_ADAPTER) ? 1 : 0;

                } else {
                        ret = -EINVAL;
                }
                break;
        case POWER_SUPPLY_PROP_PRESENT:
                if (psy->type == POWER_SUPPLY_TYPE_USB || psy->type == POWER_SUPPLY_TYPE_MAINS) {
                    /* for ATD test to acquire the status about charger ic */
                    if (!smb347_has_charger_error() || smb347_get_charging_status() == POWER_SUPPLY_STATUS_CHARGING)
                        val->intval = 1;
                    else
                        val->intval = 0;
                } else {
                    ret = -EINVAL;
                }
                break;
        default:
                ret = -EINVAL;
        }

        return ret;
}


int asus_register_power_supply(struct device *dev, struct dev_func *tbl)
{
        int ret;
        int test_flag=0;
        drv_status_t drv_status;
        int voltage=0, percentage=0;
        u32 cable_status;

        BAT_DBG_E("%s\n", __func__);

        mutex_lock(&batt_info_mutex);
        drv_status = batt_info.drv_status;
        test_flag = batt_info.test_flag;
        mutex_unlock(&batt_info_mutex);

        if (!dev) {
                BAT_DBG_E("%s, device pointer is NULL.\n", __func__);
                return -EINVAL;
        } else if (!tbl) {
                BAT_DBG_E("%s, dev_func pointer is NULL.\n", __func__);
                return -EINVAL;
        }else if (drv_status != DRV_INIT_OK) {
                BAT_DBG_E("%s, asus_battery not init ok.\n", __func__);
                return -EINVAL;
        } else if (test_flag & TEST_INFO_NO_REG_POWER) {
                BAT_DBG_E("%s, Not register power class.\n", __func__);
                return 0;
        }

        mutex_lock(&batt_info_mutex);
        batt_info.drv_status = DRV_REGISTER;
        batt_info.tbl = tbl;
        mutex_unlock(&batt_info_mutex);

#if 0
        /* ME371MG: check if device connect to power supply without
         *          battery insertions in userdebug or eng build.
         */
        if (batt_info.tbl && (build_version == 1 || build_version == 2)) {
            percentage = asus_battery_update_percentage();
            voltage = asus_battery_update_voltage();
            cable_status = batt_info.cable_status;
            if (voltage > 3800 && percentage == 0 && cable_status == NO_CABLE) {
                BAT_DBG_E("%s:Not register power class. Reason: batt votage>3.8V, batt perc=0% without USB\n", __func__);
                return -EINVAL;
            }
        }
#endif

        //register to power supply driver
        ret = power_supply_register(dev, &asus_power_supplies[CHARGER_BATTERY]);
        if (ret) { BAT_DBG_E("Fail to register battery\n"); goto batt_err_reg_fail_battery; }

        ret = power_supply_register(dev, &asus_power_supplies[CHARGER_USB]);
        if (ret) { BAT_DBG_E("Fail to register USB\n"); goto batt_err_reg_fail_usb; }

        ret = power_supply_register(dev, &asus_power_supplies[CHARGER_AC]);
        if (ret) { BAT_DBG_E("Fail to register AC\n"); goto batt_err_reg_fail_ac; }

        //first update current information
        mutex_lock(&batt_info_mutex);
        asus_battery_get_info_no_mutex();
        batt_info.drv_status = DRV_REGISTER_OK;
        batt_info.present = 1;
        mutex_unlock(&batt_info_mutex);

        /* init wake lock in COS */
        if (entry_mode == 4) {
            BAT_DBG(" %s: wake lock init: asus_battery_power_wakelock\n", __func__);
            wake_lock_init(&wakelock, WAKE_LOCK_SUSPEND, "asus_battery_power_wakelock");
            wake_lock_init(&wakelock_t, WAKE_LOCK_SUSPEND, "asus_battery_power_wakelock_timeout");
        }

        /* prevent system from entering s3 in COS while AC charger is connected */
        if (entry_mode == 4) {
            if (batt_info.cable_status == USB_ADAPTER) {
                if (!wake_lock_active(&wakelock)) {
                    BAT_DBG(" %s: asus_battery_power_wakelock -> wake lock\n", __func__);
                    wake_lock(&wakelock);
                }
            }
        }

        //start working 
        queue_delayed_work(battery_work_queue, &battery_poll_data_work, 
                        30*HZ);

        BAT_DBG("%s register OK.\n", __func__);
        return 0;

batt_err_reg_fail_ac:
        power_supply_unregister(&asus_power_supplies[CHARGER_USB]);
batt_err_reg_fail_usb:
        power_supply_unregister(&asus_power_supplies[CHARGER_BATTERY]);
batt_err_reg_fail_battery:

        return ret;
}
EXPORT_SYMBOL(asus_register_power_supply);

int asus_battery_init(
        u32 polling_time, 
        u32 critical_polling_time, 
        u32 test_flag
)
{
        int ret=0;
        drv_status_t drv_sts;    

        BAT_DBG("%s, %d, %d, 0x%08X\n", __func__, polling_time, critical_polling_time, test_flag);

        mutex_lock(&batt_info_mutex);
        drv_sts = batt_info.drv_status;
        mutex_unlock(&batt_info_mutex);

        if (drv_sts != DRV_NOT_READY) {
                //other battery device registered.
                BAT_DBG_E("Error!! Already registered by other driver\n");
                ret = -EINVAL;
                if (test_flag & TEST_INFO_NO_REG_POWER) {
                        ret = 0;
                }
                                
                goto already_init;

        }

        mutex_lock(&batt_info_mutex);
        batt_info.drv_status = DRV_INIT;
        batt_info.polling_time = polling_time > (5*HZ) ? polling_time : BATTERY_DEFAULT_POLL_TIME;
        batt_info.critical_polling_time = critical_polling_time > (3*HZ) ? critical_polling_time : BATTERY_CRITICAL_POLL_TIME;
        batt_info.critical_polling_time = BATTERY_CRITICAL_POLL_TIME;
        batt_info.percentage = 50;
/* chris20121116: do not initialize it here */
#if 0
        batt_info.cable_status = NO_CABLE;
#endif
        batt_info.batt_temp = 250;
        batt_info.test_flag = test_flag;
        if (test_flag)
                BAT_DBG("test_flag: 0x%08X\n", test_flag);
        mutex_unlock(&batt_info_mutex);

        if (test_flag & TEST_INFO_NO_REG_POWER) {
                BAT_DBG_E("Not allow initiallize  power class. Skip ...\n");
                ret = 0;

                mutex_lock(&batt_info_mutex);
                batt_info.drv_status = DRV_INIT_OK;
                mutex_unlock(&batt_info_mutex);

                goto already_init;
        }

#if CONFIG_PROC_FS
        ret = asus_battery_register_proc_fs();
        if (ret) {
                BAT_DBG_E("Unable to create proc file\n");
                goto proc_fail;
        }
#endif

        battery_work_queue = create_singlethread_workqueue("asus_battery");
        if(battery_work_queue == NULL)
        {
                BAT_DBG_E("Create battery thread fail");
                ret = -ENOMEM;
                goto error_workq;
        }
        INIT_DELAYED_WORK(&battery_poll_data_work, asus_polling_data);
        INIT_DELAYED_WORK(&detect_cable_work, USB_cable_status_worker);

#ifdef ME302C_ENG_BUILD
        ret = init_asus_charging_toggle();
        if (ret) {
                BAT_DBG_E("Unable to create proc file\n");
                goto proc_fail;
        }
#endif

#ifndef ME302C_USER_BUILD
        ret = init_emerg_poll_toggle();
        if (ret) {
                BAT_DBG_E("Unable to create proc file\n");
                goto proc_fail;
        }
#endif
        mutex_lock(&batt_info_mutex);
        batt_info.drv_status = DRV_INIT_OK;
        mutex_unlock(&batt_info_mutex);

        BAT_DBG("%s: success\n", __func__);

        return 0;

error_workq:    
#if CONFIG_PROC_FS
proc_fail:
#endif
already_init:
        return ret;
}
EXPORT_SYMBOL(asus_battery_init);

void asus_battery_exit(void)
{
        drv_status_t drv_sts;    

        BAT_DBG("Driver unload\n");

        mutex_lock(&batt_info_mutex);
        drv_sts = batt_info.drv_status;
        mutex_unlock(&batt_info_mutex);

        if (drv_sts == DRV_REGISTER_OK) {
                power_supply_unregister(&asus_power_supplies[CHARGER_BATTERY]);
                power_supply_unregister(&asus_power_supplies[CHARGER_USB]);
                power_supply_unregister(&asus_power_supplies[CHARGER_AC]);
                if (entry_mode == 4)
                    wake_lock_destroy(&wakelock);
        }
        BAT_DBG("Driver unload OK\n");
}

static int __init asus_battery_fake_init(void)
{
        return 0;
}
late_initcall(asus_battery_fake_init);

static void __exit asus_battery_fake_exit(void)
{
        /* SHOULD NOT REACHE HERE */
        BAT_DBG("%s exit\n", __func__);
}
module_exit(asus_battery_fake_exit);

MODULE_AUTHOR("chris1_chang@asus.com");
MODULE_DESCRIPTION("battery driver");
MODULE_LICENSE("GPL");


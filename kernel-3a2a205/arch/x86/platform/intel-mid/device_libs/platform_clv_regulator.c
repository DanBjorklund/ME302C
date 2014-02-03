/*
 * CLV regulator platform data
 * Copyright (c) 2012, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/lcd.h>
#include <linux/regulator/intel_pmic.h>
#include <linux/regulator/machine.h>
#include <asm/intel_scu_flis.h>
/***********VPROG1 REGUATOR platform data*************/
static struct regulator_consumer_supply vprog1_consumer[] = {
	REGULATOR_SUPPLY("vprog1", "4-0048"),
	//REGULATOR_SUPPLY("vprog1", "4-0036"),
	REGULATOR_SUPPLY("vprog1", "4-003C"), //ASUS_BSP+++
};

static struct regulator_init_data vprog1_data = {
	   .constraints = {
			.min_uV			= 1200000,
			.max_uV			= 2800000,
			.valid_ops_mask	= REGULATOR_CHANGE_STATUS |
							REGULATOR_CHANGE_MODE |
						REGULATOR_CHANGE_VOLTAGE,
			.valid_modes_mask	= REGULATOR_MODE_NORMAL |
							REGULATOR_MODE_STANDBY |
						REGULATOR_MODE_FAST,
            		.force_boot_off = 1, //ASUS_BSP++, add for turn off power at booting
		},
		.num_consumer_supplies	=	ARRAY_SIZE(vprog1_consumer),
		.consumer_supplies	=	vprog1_consumer,
};

static struct intel_pmic_info vprog1_info = {
		.pmic_reg   = VPROG1CNT_ADDR,
		.init_data  = &vprog1_data,
		.table_len  = ARRAY_SIZE(VPROG1_VSEL_table),
		.table      = VPROG1_VSEL_table,
};
static struct platform_device vprog1_device = {
	.name = "intel_regulator",
	.id = VPROG1,
	.dev = {
		.platform_data = &vprog1_info,
	},
};
/***********VPROG2 REGUATOR platform data*************/
static struct regulator_consumer_supply vprog2_consumer[] = {
	REGULATOR_SUPPLY("vprog2", "4-0048"), //ASUS_BSP+++
	REGULATOR_SUPPLY("vprog2", "4-003C"), //ASUS_BSP+++
};
static struct regulator_init_data vprog2_data = {
		.constraints = {
			.min_uV			= 1200000,
			.max_uV			= 2800000,
			.valid_ops_mask	= REGULATOR_CHANGE_STATUS |
							REGULATOR_CHANGE_MODE |
						REGULATOR_CHANGE_VOLTAGE,
			.valid_modes_mask	= REGULATOR_MODE_NORMAL |
							REGULATOR_MODE_STANDBY |
						REGULATOR_MODE_FAST,
            		.force_boot_off = 1, //ASUS_BSP++, add for turn off power at booting
						},
		.num_consumer_supplies	=	ARRAY_SIZE(vprog2_consumer),
		.consumer_supplies	=	vprog2_consumer,
};
static struct intel_pmic_info vprog2_info = {
		.pmic_reg   = VPROG2CNT_ADDR,
		.init_data  = &vprog2_data,
		.table_len  = ARRAY_SIZE(VPROG2_VSEL_table),
		.table      = VPROG2_VSEL_table,
};
static struct platform_device vprog2_device = {
	.name = "intel_regulator",
	.id = VPROG2,
	.dev = {
		.platform_data = &vprog2_info,
	},
};
/***********VEMMC1 REGUATOR platform data*************/
static struct regulator_consumer_supply vemmc1_consumer[] = {
	REGULATOR_SUPPLY("vemmc1", "4-003C"), //ASUS_BSP+++
};
static struct regulator_init_data vemmc1_data = {
		.constraints = {
			.min_uV			= 2850000,
			.max_uV			= 2850000,
			.valid_ops_mask	=	REGULATOR_CHANGE_STATUS |
							REGULATOR_CHANGE_MODE,
			.valid_modes_mask	= REGULATOR_MODE_NORMAL |
							REGULATOR_MODE_STANDBY |
						REGULATOR_MODE_FAST,

            		.force_boot_off = 1, //ASUS_BSP++, add for turn off power at booting
		 },
		.num_consumer_supplies	=	ARRAY_SIZE(vemmc1_consumer),
		.consumer_supplies	=	vemmc1_consumer,
};

static struct intel_pmic_info vemmc1_info = {
		.pmic_reg   = VEMMC1CNT_ADDR,
		.init_data  = &vemmc1_data,
		.table_len  = ARRAY_SIZE(VEMMC1_VSEL_table),
		.table      = VEMMC1_VSEL_table,
};

static struct platform_device vemmc1_device = {
	.name = "intel_regulator",
	.id = VEMMC1,
	.dev = {
		.platform_data = &vemmc1_info,
	},
};

/***********VEMMC2 REGUATOR platform data*************/
static struct regulator_consumer_supply vemmc2_consumer[] = {
};

static struct regulator_init_data vemmc2_data = {
		.constraints = {
			.min_uV			= 2850000,
			.max_uV			= 2850000,
			.valid_ops_mask	=	REGULATOR_CHANGE_STATUS |
							REGULATOR_CHANGE_MODE,
			.valid_modes_mask	=	REGULATOR_MODE_NORMAL |
							REGULATOR_MODE_STANDBY |
						REGULATOR_MODE_FAST,
		      },
		.num_consumer_supplies		= ARRAY_SIZE(vemmc2_consumer),
		.consumer_supplies		 = vemmc2_consumer,
};

static struct regulator_consumer_supply vccsdio_consumer[] = {
    REGULATOR_SUPPLY("vmmc", "0000:00:04.0"),
};
static struct regulator_init_data vccsdio_data = {
		.constraints = {
			.min_uV			= 2850000,
			.max_uV			= 2850000,
			.valid_ops_mask	=	REGULATOR_CHANGE_STATUS |
							REGULATOR_CHANGE_MODE,
			.valid_modes_mask	=	REGULATOR_MODE_NORMAL |
							REGULATOR_MODE_STANDBY |
						REGULATOR_MODE_FAST,
            .force_boot_off = 1,
		      },
		.num_consumer_supplies		= ARRAY_SIZE(vccsdio_consumer),
		.consumer_supplies		 = vccsdio_consumer,
};

static struct intel_pmic_info vemmc2_info = {
		.pmic_reg   = VEMMC2CNT_ADDR,
		.init_data  = &vemmc2_data,
		.table_len  = ARRAY_SIZE(VEMMC2_VSEL_table),
		.table      = VEMMC2_VSEL_table,
};

#define ENCTRL0_ISOLATE		0x55555557
#define ENCTRL1_ISOLATE		0x5555
#define STORAGESTIO_FLISNUM	0x8
#define ENCTRL0_OFF		0x10
#define ENCTRL1_OFF		0x11

static unsigned int enctrl0_orig;
static unsigned int enctrl1_orig;

static int _mmc1_disconnect_shim(void)
{
	int err = 0;

	err = intel_scu_ipc_read_shim(&enctrl0_orig,
			STORAGESTIO_FLISNUM, ENCTRL0_OFF);
	if (err) {
		pr_err("%s: ENCTRL0 read failed\n", __func__);
		goto out;
	}
	err = intel_scu_ipc_read_shim(&enctrl1_orig,
			STORAGESTIO_FLISNUM, ENCTRL1_OFF);
	if (err) {
		pr_err("%s: ENCTRL1 read failed\n", __func__);
		goto out;
	}

	pr_info("%s: mmc1.vmmc save original enctrl 0x%x, 0x%x\n",
			__func__, enctrl0_orig, enctrl1_orig);
	/* isolate shim */
	err = intel_scu_ipc_write_shim(ENCTRL0_ISOLATE,
			STORAGESTIO_FLISNUM, ENCTRL0_OFF);
	if (err) {
		pr_err("%s: ENCTRL0 ISOLATE failed\n",
				__func__);
		goto out;
	}

	err = intel_scu_ipc_write_shim(ENCTRL1_ISOLATE,
			STORAGESTIO_FLISNUM, ENCTRL1_OFF);
	if (err) {
		pr_err("%s: ENCTRL1 ISOLATE failed\n", __func__);
		goto out;
	}
  out:
    return err;
}
/* Wrapper function, always return 0 (pass) */
static int mmc1_disconnect_shim(void)
{
    _mmc1_disconnect_shim();
    return 0;
}
static int _mmc1_reconnect_shim(void)
{
	int err = 0;

	/* reconnect shim */
	err = intel_scu_ipc_write_shim(enctrl0_orig,
			STORAGESTIO_FLISNUM, ENCTRL0_OFF);
	if (err) {
		pr_err("%s: ENCTRL0 CONNECT shim failed\n", __func__);
		goto out;
	}

	err = intel_scu_ipc_write_shim(enctrl1_orig,
			STORAGESTIO_FLISNUM, ENCTRL1_OFF);
	if (err) {
		pr_err("%s: ENCTRL1 CONNECT shim failed\n", __func__);
		goto out;
	}

	pr_info("%s: mmc1.vmmc recover original enctrl 0x%x, 0x%x\n",
			__func__, enctrl0_orig, enctrl1_orig);
out:
	return err;
}
/* Wrapper function, always return 0 (pass) */
static int mmc1_reconnect_shim(void)
{
    _mmc1_reconnect_shim();
    return 0;
}

static struct intel_pmic_info vccsdio_info = {
		.pmic_reg   = VCCSDIOCNT_ADDR,
		.init_data  = &vccsdio_data,
		.table_len  = ARRAY_SIZE(VCCSDIO_VSEL_table),
		.table      = VCCSDIO_VSEL_table,
        .before_vreg_off = mmc1_disconnect_shim,
        .after_vreg_on = mmc1_reconnect_shim,
};

static struct platform_device vemmc2_device = {
	.name = "intel_regulator",
	.id = VEMMC2,
	.dev = {
		.platform_data = &vemmc2_info,
	},
};

static struct platform_device vccsdio_device = {
	.name = "intel_regulator",
	.id = VCCSDIO,
	.dev = {
		.platform_data = &vccsdio_info,
	},
};

static struct platform_device *regulator_devices[] __initdata = {
	&vprog1_device,
	&vprog2_device,
	&vemmc1_device,
	&vemmc2_device,
    &vccsdio_device,
};

static int __init regulator_init(void)
{
	platform_add_devices(regulator_devices,
		ARRAY_SIZE(regulator_devices));
	return 0;
}
device_initcall(regulator_init);

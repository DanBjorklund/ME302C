/*
 * intel_soc_clv.c - This driver provides utility api's for
 * Cloverview platform
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

#include "intel_soc_pmu.h"
#include <linux/HWVersion.h>
#include <linux/gpio.h>
#if defined(CONFIG_ME302C_SPC_PWR_SET)
#define PIN_5VSUS_EN 160
#endif

extern int Read_HW_ID(void);
extern int Read_PROJ_ID(void);
static int HW_ID;
static int PROJ_ID;

static int clv_pmu_init(void)
{
	mid_pmu_cxt->s3_hint = C6_HINT;
	return 0;
}

static bool clv_pmu_enter(int s0ix_state)
{
	u32 s0ix_value;
	int num_retry = PMU_MISC_SET_TIMEOUT;
        
#if defined(CONFIG_ME302C_SPC_PWR_SET)
	if((PROJ_ID == PROJ_ID_ME302C && !(HW_ID == HW_ID_SR1 || HW_ID == HW_ID_SR2)) || PROJ_ID == PROJ_ID_GEMINI){
	      if(MID_S3_STATE==s0ix_state){
	           gpio_direction_output(PIN_5VSUS_EN, 0);
	       }
	}
#endif
	s0ix_value = get_s0ix_val_set_pm_ssc(s0ix_state);
	/* issue a command to SCU */
	writel(s0ix_value, &mid_pmu_cxt->pmu_reg->pm_cmd);

	do {
		if (readl(&mid_pmu_cxt->pmu_reg->pm_msic))
			break;
		udelay(1);
	} while (--num_retry);

	if (!num_retry && !readl(&mid_pmu_cxt->pmu_reg->pm_msic))
		WARN(1, "%s: pm_msic not set.\n", __func__);

	mid_pmu_cxt->s0ix_entered = s0ix_state;

	return true;
}

static void clv_pmu_remove(void)
{
	/* Place holder */
}

static void clv_pmu_wakeup(void)
{

	/* Wakeup allother CPU's */
	if (mid_pmu_cxt->s0ix_entered)
		apic->send_IPI_allbutself(RESCHEDULE_VECTOR);
#if defined(CONFIG_ME302C_SPC_PWR_SET)
	if((PROJ_ID == PROJ_ID_ME302C && !(HW_ID == HW_ID_SR1 || HW_ID == HW_ID_SR2)) || PROJ_ID == PROJ_ID_GEMINI){
	      if(MID_S3_STATE==mid_pmu_cxt->s0ix_entered){
	           gpio_direction_output(PIN_5VSUS_EN, 1);
	       }
	}
#endif

}

static pci_power_t clv_pmu_choose_state(int device_lss)
{
	pci_power_t state;

	switch (device_lss) {
	case PMU_SECURITY_LSS_04:
		state = PCI_D2;
		break;

	case PMU_USB_OTG_LSS_06:
	case PMU_USB_HSIC_LSS_07:
	case PMU_UART2_LSS_41:
		state = PCI_D1;
		break;

	default:
		state = PCI_D3hot;
		break;
	}

	return state;
}

/**
 *      platform_set_pmu_ops - Set the global pmu method table.
 *      @ops:   Pointer to ops structure.
 */
void platform_set_pmu_ops(void)
{
	HW_ID = Read_HW_ID();
	PROJ_ID = Read_PROJ_ID();
	pmu_ops = &clv_pmu_ops;
}

struct platform_pmu_ops clv_pmu_ops = {
	.init		= clv_pmu_init,
	.enter		 = clv_pmu_enter,
	.wakeup		 = clv_pmu_wakeup,
	.remove		= clv_pmu_remove,
	.pci_choose_state = clv_pmu_choose_state,
	.set_power_state_ops = pmu_set_s0ix_possible,
	.set_s0ix_complete = s0ix_complete,
	.nc_set_power_state = mdfld_clv_nc_set_power_state,
};

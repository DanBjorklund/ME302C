#ifndef _ASM_X86_INTEL_SCU_IPC_H_
#define  _ASM_X86_INTEL_SCU_IPC_H_

#include <linux/notifier.h>
#include <asm/intel-mid.h>

/* IPC defines the following message types */
#define IPCMSG_GET_HOBADDR	0xE5 /* OSHOB access. */
#define IPCMSG_BATTERY          0xEF /* Coulomb Counter Accumulator */
#define IPCMSG_MIP_ACCESS       0xEC /* IA MIP access */
#define IPCMSG_PMDB_CMD		0xE0
#define IPCMSG_WARM_RESET	0xF0
#define IPCMSG_COLD_RESET	0xF1
#define IPCMSG_SOFT_RESET	0xF2
#define IPCMSG_COLD_BOOT	0xF3
#define IPCMSG_COLD_OFF		0x80 /* for TNG only */
#define IPCMSG_FW_REVISION      0xF4 /* Get firmware revision */
#define IPCMSG_SHIM_CONFIG	0xF5 /* Configure SHIM */
#define IPCMSG_WATCHDOG_TIMER   0xF8 /* Set Kernel Watchdog Threshold */
#define IPCMSG_VRTC		0xFA	 /* Set vRTC device */
#define IPCMSG_FW_UPDATE        0xFE /* Firmware update */
#define IPCMSG_PCNTRL           0xFF /* Power controller unit read/write */
#define IPCMSG_OSC_CLK		0xE6 /* Turn on/off osc clock */
#define IPCMSG_S0IX_COUNTER	0xEB /* Get S0ix residency */

#define IPC_CMD_UMIP_RD     0
#define IPC_CMD_UMIP_WR     1
#define IPC_CMD_SMIP_RD     2

/* Command id associated with message IPCMSG_PCNTRL */
#define IPC_CMD_PCNTRL_W      0 /* Register write */
#define IPC_CMD_PCNTRL_R      1 /* Register read */
#define IPC_CMD_PCNTRL_M      2 /* Register read-modify-write */

#define IPC_ERR_NONE			0
#define IPC_ERR_CMD_NOT_SUPPORTED	1
#define IPC_ERR_CMD_NOT_SERVICED	2
#define IPC_ERR_UNABLE_TO_SERVICE	3
#define IPC_ERR_CMD_INVALID		4
#define IPC_ERR_CMD_FAILED		5
#define IPC_ERR_EMSECURITY		6

#define MSIC_DEBUG_FILE "msic"
#define MSIC_ALL_DEBUG_FILE "msic_all"
#define MAX_MSIC_REG   0x3FF
#define MIN_MSIC_REG   0x0



/* Command id associated with message IPCMSG_VRTC */
#define IPC_CMD_VRTC_SETTIME      1 /* Set time */
#define IPC_CMD_VRTC_SETALARM     2 /* Set alarm */
#define IPC_CMD_VRTC_SYNC_RTC     3 /* Sync MSIC/PMIC RTC to VRTC */

/* Command id associated with message IPCMSG_SHIM_CONFIG */
#define IPC_CMD_SHIM_RD		0 /* SHIM read */
#define IPC_CMD_SHIM_WR		1 /* SHIM write */

/* Send ipc command and check ipc status */
void intel_scu_ipc_send_command(u32 cmd);
int intel_scu_ipc_check_status(void);

/* Issue commands to the SCU with or without data */
int intel_scu_ipc_simple_command(int cmd, int sub);
int intel_scu_ipc_command(u32 cmd, u32 sub, u8 *in, u32 inlen,
		u32 *out, u32 outlen);
int intel_scu_ipc_raw_cmd(u32 cmd, u32 sub, u8 *in, u32 inlen,
		u32 *out, u32 outlen, u32 dptr, u32 sptr);

/* IPC locking */
void intel_scu_ipc_lock(void);
void intel_scu_ipc_unlock(void);

/* I2C control api */
int intel_scu_ipc_i2c_cntrl(u32 addr, u32 *data);

/* Update FW version */
int intel_scu_ipc_fw_update(void);
int intel_scu_ipc_mrstfw_update(u8 *buffer, u32 length);
int intel_scu_ipc_medfw_prepare(void __user *arg);

int intel_scu_ipc_read_mip(u8 *data, int len, int offset, int issigned);
int intel_scu_ipc_write_umip(u8 *data, int len, int offset);

/* Penwell has 4 osc clocks */
#define OSC_CLK_AUDIO	0	/* Audio */
#define OSC_CLK_CAM0	1	/* Primary camera */
#define OSC_CLK_CAM1	2	/* Secondary camera */
#define OSC_CLK_DISP	3	/* Display buffer */

int intel_scu_ipc_osc_clk(u8 clk, unsigned int khz);

extern struct blocking_notifier_head intel_scu_notifier;

static inline void intel_scu_notifier_add(struct notifier_block *nb)
{
	blocking_notifier_chain_register(&intel_scu_notifier, nb);
}

static inline void intel_scu_notifier_remove(struct notifier_block *nb)
{
	blocking_notifier_chain_unregister(&intel_scu_notifier, nb);
}

static inline int intel_scu_notifier_post(unsigned long v, void *p)
{
	return blocking_notifier_call_chain(&intel_scu_notifier, v, p);
}

#define		SCU_AVAILABLE		1
#define		SCU_DOWN		2

#endif

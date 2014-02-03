#
#  Copyright (c) 2012, ASUSTek, Inc. All Rights Reserved.
#  Written by chris chang chris1_chang@asus.com
#

ifeq ($(TARGET_BUILD_VARIANT),eng)
        KBUILD_CFLAGS += -DME302C_ENG_BUILD
else ifeq ($(TARGET_BUILD_VARIANT),user)
        KBUILD_CFLAGS += -DME302C_USER_BUILD
else ifeq ($(TARGET_BUILD_VARIANT),userdebug)
        KBUILD_CFLAGS += -DME302C_USERDEBUG_BUILD
else
        KBUILD_CFLAGS += -DME302C_OTHER_BUILD
endif

obj-$(CONFIG_ME302C_BATTERY_SMB347)	+= battery_smb347.o
obj-$(CONFIG_ME302C_BATTERY_BQ27541)	+= battery_bq27541.o
obj-$(CONFIG_ASUS_ENGINEER_MODE) += asus_battery_proc_fs_asustag.o

obj-$(CONFIG_ME302C_BATTERY) += asus_battery_power.o
obj-$(CONFIG_PROC_FS) += asus_battery_proc_fs.o

obj-$(CONFIG_ME302C_BATTERY_BQ27520) += bq27520_battery_core.o
obj-$(CONFIG_ME302C_BATTERY_BQ27520) += bq27520_factory_data.o
obj-$(CONFIG_PROC_FS) += bq27520_proc_fs.o
obj-$(CONFIG_ASUS_ENGINEER_MODE) += bq27520_proc_fs_asustag.o
obj-$(CONFIG_ASUS_ENGINEER_MODE) += bq27520_battery_upt_i2c.o


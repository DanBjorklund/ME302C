/*
 * Copyright (c) 2012, ASUSTek, Inc. All Rights Reserved.
 * Written by chris chang chris1_chang@asus.com
 */
#include <linux/proc_fs.h>
#include <linux/uaccess.h>

#include "bq27520_proc_fs.h" 

int bq27520_register_proc_fs(void)
{
    int ret=0;

#if defined(CONFIG_ASUS_ENGINEER_MODE) && CONFIG_ASUS_ENGINEER_MODE
    ret = bq27520_register_proc_fs_test();
#endif
    return ret;
}

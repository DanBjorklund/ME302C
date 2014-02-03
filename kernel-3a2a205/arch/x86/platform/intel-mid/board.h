/*
 * board.h: intel mid board header file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _BOARD_H_
#define _BOARD_H_
#ifdef CONFIG_ME302C
extern struct devs_id __initconst device_ids_me302c[] __attribute__((weak));
#endif

#ifdef CONFIG_TX201LA
extern struct devs_id __initconst device_ids_tx201la[] __attribute__((weak));
#endif

#ifdef CONFIG_ME372CG
extern struct devs_id __initconst device_ids_me372cg[] __attribute__((weak));
#endif
#endif

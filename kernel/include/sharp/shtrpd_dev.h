/* kernel/include/sharp/shtrpd_dev.h  (TrackPad Driver)
 *
 * Copyright (C) 2014 SHARP CORPORATION
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

 
#ifndef _SHTPRD_DEV_H_
#define _SHTPRD_DEV_H_

extern void msm_trpd_setsleep(int on);
extern void msm_trpd_set_flip_state(int state);
extern void msm_trpd_set_poweron(void);

#endif /* _SHTPRD_DEV_H_ */

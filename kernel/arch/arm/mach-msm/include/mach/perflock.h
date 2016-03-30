/* arch/arm/mach-msm/perflock.h
 *
 * MSM performance lock driver header
 *
 * Copyright (C) 2008 HTC Corporation
 * Author: Eiven Peng <eiven_peng@htc.com>
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

#ifndef __ARCH_ARM_MACH_PERF_LOCK_H
#define __ARCH_ARM_MACH_PERF_LOCK_H

#include <linux/list.h>
#include <linux/cpufreq.h>


enum {
	TYPE_PERF_LOCK = 0,
	TYPE_CPUFREQ_CEILING,
};

#ifdef CONFIG_SHSYS_CUST_PERFLOCK
enum {
	PERF_LOCK_300000KHz,	/* PERF_LEVEL0 */
	PERF_LOCK_384000KHz,	/* PERF_LEVEL1 */
	PERF_LOCK_600000KHz,	/* PERF_LEVEL2 */
	PERF_LOCK_787200KHz,	/* PERF_LEVEL3 */
	PERF_LOCK_998400KHz,	/* PERF_LEVEL4 */
	PERF_LOCK_1094400KHz,	/* PERF_LEVEL5 */
	PERF_LOCK_1190400KHz,	/* PERF_LEVEL6 */
	PERF_LOCK_HIGHEST = PERF_LOCK_1190400KHz,
	PERF_LOCK_INVALID,	/* INVALID */
};
#else
enum {
	PERF_LOCK_LOWEST,
	PERF_LOCK_LOW,
	PERF_LOCK_MEDIUM,
	PERF_LOCK_HIGH,
	PERF_LOCK_HIGHEST,
	PERF_LOCK_INVALID,
};
#endif

struct perf_lock {
	struct list_head link;
	unsigned int flags;
	unsigned int level;
	const char *name;
	unsigned int type;
};

struct perflock_data {
	unsigned int *perf_acpu_table;
	unsigned int table_size;
};

struct perflock_pdata {
	struct perflock_data *perf_floor;
	struct perflock_data *perf_ceiling;
};


#ifndef CONFIG_PERFLOCK
#ifndef CONFIG_SHSYS_CUST_PERFLOCK
static inline void __init perflock_init(
	struct perflock_data *pdata) { return; }
static inline void __init cpufreq_ceiling_init(
	struct perflock_data *pdata) { return; }
#endif
static inline void perf_lock_init(struct perf_lock *lock,
	unsigned int level, const char *name) { return; }
#ifdef CONFIG_SHSYS_CUST_PERFLOCK
static inline void limit_lock_init(struct perf_lock *lock,
	unsigned int level, const char *name) { return; }
static inline void limit_lock(struct perf_lock *lock) { return; }
static inline void limit_unlock(struct perf_lock *lock) { return; }
#else
static inline void perf_lock_init_v2(struct perf_lock *lock,
	unsigned int level, const char *name) { return; }
#endif
static inline void perf_lock(struct perf_lock *lock) { return; }
static inline void perf_unlock(struct perf_lock *lock) { return; }
static inline int is_perf_lock_active(struct perf_lock *lock) { return 0; }
static inline int is_perf_locked(void) { return 0; }
static inline void perflock_scaling_max_freq(unsigned int freq, unsigned int cpu) { return; }
static inline void perflock_scaling_min_freq(unsigned int freq, unsigned int cpu) { return; }
#ifdef CONFIG_SHSYS_CUST_PERFLOCK
static inline unsigned int get_perflock_acpu_table(unsigned int level) { return 0; }
static inline unsigned int get_limitlock_acpu_table(unsigned int level) { return 0; }
#else
static inline void htc_print_active_perf_locks(void) { return; }
#endif
static inline int perflock_override(const struct cpufreq_policy *policy) { return 0; }
static inline struct perf_lock *perflock_acquire(const char *name) { return NULL; }
static inline int perflock_release(const char *name) { return 0; }
#else
#ifndef CONFIG_SHSYS_CUST_PERFLOCK
extern void __init perflock_init(struct perflock_data *pdata);
extern void __init cpufreq_ceiling_init(struct perflock_data *pdata);
#endif
extern void perf_lock_init(struct perf_lock *lock,
	unsigned int level, const char *name);
#ifdef CONFIG_SHSYS_CUST_PERFLOCK
extern void limit_lock_init(struct perf_lock *lock,
	unsigned int level, const char *name);
extern void limit_lock(struct perf_lock *lock);
extern void limit_unlock(struct perf_lock *lock);
#else
extern void perf_lock_init_v2(struct perf_lock *lock,
	unsigned int level, const char *name);
#endif
extern void perf_lock(struct perf_lock *lock);
extern void perf_unlock(struct perf_lock *lock);
extern int is_perf_lock_active(struct perf_lock *lock);
extern int is_perf_locked(void);
extern void perflock_scaling_max_freq(unsigned int freq, unsigned int cpu);
extern void perflock_scaling_min_freq(unsigned int freq, unsigned int cpu);
extern int perflock_override(const struct cpufreq_policy *policy, const unsigned int new_freq);
#ifdef CONFIG_SHSYS_CUST_PERFLOCK
extern unsigned int get_perflock_acpu_table(unsigned int level);
extern unsigned int get_limitlock_acpu_table(unsigned int level);
#else
extern void htc_print_active_perf_locks(void);
extern struct perf_lock *perflock_acquire(const char *name);
extern int perflock_release(const char *name);
#ifdef CONFIG_PERFLOCK_BOOT_LOCK
extern void release_boot_lock(void);
#endif
#endif
#endif

#ifdef CONFIG_PERFLOCK_SUSPEND_LOCK
extern void shsys_enter_state_perf_lock(void);
extern void shsys_enter_state_perf_unlock(void);
#endif

#endif


/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * 2012 Enhanced by motley <motley.slate@gmail.com>
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/msm_tsens.h>
#include <linux/workqueue.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/msm_tsens.h>
#include <linux/msm_thermal.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <mach/cpufreq.h>
#include <linux/reboot.h>

static int enabled;
static struct msm_thermal_data msm_thermal_info;
static uint32_t limited_max_freq = MSM_CPUFREQ_NO_LIMIT;
static struct delayed_work check_temp_work;

static int limit_idx;
static int limit_idx_low;
static int limit_idx_high;
static struct cpufreq_frequency_table *table;

static DEFINE_MUTEX(emergency_shutdown_mutex);

/*
 * Controls
 * DEFAULT_THROTTLE_TEMP - default throttle temp at boot time
 * MAX_THROTTLE_TEMP - max able to be set by user
 * SHUTDOWN_TEMP - max temp in C where CPU will shut down
 * COOL_TEMP - temp in C where where we can slow down polling
 * COOL_TEMP_OFFSET_MS - number of ms to add to polling time when temps are cool
 * HOT_TEMP_OFFSET_MS - number of ms to subtract from polling time when temps are cool
 * */

#define DEFAULT_THROTTLE_TEMP	64
#define MAX_THROTTLE_TEMP		74
#define SHUTDOWN_TEMP			80
#define COOL_TEMP				40
#define COOL_TEMP_OFFSET_MS		250
#define HOT_TEMP_OFFSET_MS		250

static bool thermal_debug = false;
static int throttle_temp = DEFAULT_THROTTLE_TEMP;

static int msm_thermal_get_freq_table(void)
{
	int ret = 0;
	int i = 0;

	table = cpufreq_frequency_get_table(0);
	if (table == NULL) {
		pr_debug("%s: error reading cpufreq table\n", __func__);
		ret = -EINVAL;
		goto fail;
	}

	while (table[i].frequency != CPUFREQ_TABLE_END)
		i++;

	limit_idx_low = 0;
	limit_idx_high = limit_idx = i - 1;
	BUG_ON(limit_idx_high <= 0 || limit_idx_high <= limit_idx_low);
fail:
	return ret;
}

static int update_cpu_max_freq(int cpu, uint32_t max_freq)
{
	int ret = 0;

	ret = msm_cpufreq_set_freq_limits(cpu, MSM_CPUFREQ_NO_LIMIT, max_freq);
	if (ret)
		return ret;

	limited_max_freq = max_freq;
	if (max_freq != MSM_CPUFREQ_NO_LIMIT)
		pr_info("msm_thermal: Limiting cpu%d max frequency to %d\n",
				cpu, max_freq);
	else
		pr_info("msm_thermal: Max frequency reset for cpu%d\n", cpu);

	ret = cpufreq_update_policy(cpu);

	return ret;
}

static void check_temp(struct work_struct *work)
{
	static int limit_init;
	struct tsens_device tsens_dev;
	unsigned long temp = 0;
	uint32_t max_freq = limited_max_freq;
	int cpu = 0;
	int ret = 0;
	int near_limit = 0;

	tsens_dev.sensor_num = msm_thermal_info.sensor_id;
	ret = tsens_get_temp(&tsens_dev, &temp);
	if (ret) {
		if (thermal_debug)
			pr_info("msm_thermal: Unable to read TSENS sensor %d\n",
				tsens_dev.sensor_num);
		goto reschedule;
	}

	if (thermal_debug)
		pr_info("msm_thermal: current CPU temperature %lu\n",temp);

	if (!limit_init) {
		ret = msm_thermal_get_freq_table();
		if (ret)
			goto reschedule;
		else
			limit_init = 1;
	}

	if (temp >= SHUTDOWN_TEMP) {
		mutex_lock(&emergency_shutdown_mutex);
		pr_warn("msm_thermal: ***SHUTDOWN TEMP REACHED: %dC ORDERLY POWEROFF STARTED***\n", SHUTDOWN_TEMP);
		orderly_poweroff(true);
		max_freq = table[limit_idx].frequency;
		mutex_unlock(&emergency_shutdown_mutex);
		enabled = 0;
		goto setmaxfreq;
	}

	if (temp > throttle_temp - msm_thermal_info.temp_hysteresis_degC) {
		near_limit = 1;
		if (thermal_debug)
				pr_info("msm_thermal: CPU temp is nearing the throttle threshold %d\n",throttle_temp - msm_thermal_info.temp_hysteresis_degC);
	} else {
		near_limit = 0;		
	}

	if (temp >= throttle_temp) {
		if (limit_idx == limit_idx_low)
			goto reschedule;

		limit_idx -= msm_thermal_info.freq_step;
		if (limit_idx < limit_idx_low)
			limit_idx = limit_idx_low;
		max_freq = table[limit_idx].frequency;
		pr_info("msm_thermal: throttle temp exceeded:%d, throttle freq: %dMHz\n", throttle_temp, max_freq);

	} else if (temp < throttle_temp - msm_thermal_info.temp_hysteresis_degC) {
		if (limit_idx == limit_idx_high)
			goto reschedule;

		limit_idx += msm_thermal_info.freq_step;
		if (limit_idx >= limit_idx_high) {
			limit_idx = limit_idx_high;
			max_freq = MSM_CPUFREQ_NO_LIMIT;
		} else
			max_freq = table[limit_idx].frequency;
	}
	if (max_freq == limited_max_freq)
		goto reschedule;

setmaxfreq:

	/* Update new limits */
	for_each_possible_cpu(cpu) {
		ret = update_cpu_max_freq(cpu, max_freq);
		if (ret) {
			if (thermal_debug)
				pr_info("msm_thermal: unable to limit cpu%d max freq to %d\n", cpu, max_freq);
		}
	}

reschedule:

	if (enabled) {
		if (temp > COOL_TEMP) {
			if (near_limit) {
				if (thermal_debug)
					pr_info("msm_thermal: throttle temp is near, polling at %dms\n",msm_thermal_info.poll_ms + HOT_TEMP_OFFSET_MS);
				schedule_delayed_work(&check_temp_work,
						msecs_to_jiffies(msm_thermal_info.poll_ms - HOT_TEMP_OFFSET_MS));
			} else {
				if (thermal_debug)
					pr_info("msm_thermal: CPU temp is fine, polling at %dms\n",msm_thermal_info.poll_ms + HOT_TEMP_OFFSET_MS);
				schedule_delayed_work(&check_temp_work,
						msecs_to_jiffies(msm_thermal_info.poll_ms));
			}
		} else {
			if (thermal_debug)
				pr_info("msm_thermal: CPU temp cool, polling at %dms\n",msm_thermal_info.poll_ms + COOL_TEMP_OFFSET_MS);
			schedule_delayed_work(&check_temp_work,
					msecs_to_jiffies(msm_thermal_info.poll_ms + COOL_TEMP_OFFSET_MS));
		}
	}
}

static void disable_msm_thermal(void)
{
	int cpu = 0;

	/* make sure check_temp is no longer running */
	cancel_delayed_work(&check_temp_work);
	flush_scheduled_work();

	if (limited_max_freq == MSM_CPUFREQ_NO_LIMIT)
		return;

	for_each_possible_cpu(cpu) {
		update_cpu_max_freq(cpu, MSM_CPUFREQ_NO_LIMIT);
	}
}

static int set_enabled(const char *val, const struct kernel_param *kp)
{
	int ret = 0;

	ret = param_set_bool(val, kp);
	if (!enabled)
		disable_msm_thermal();
	else
		pr_info("msm_thermal: no action for enabled = %d\n", enabled);

	pr_info("msm_thermal: enabled = %d\n", enabled);

	return ret;
}

static int set_debug(const char *val, const struct kernel_param *kp)
{
	int ret = 0;

	ret = param_set_bool(val, kp);
	pr_info("msm_thermal: debug = %d\n", thermal_debug);

	return ret;
}

static int set_throttle_temp(const char *val, const struct kernel_param *kp)
{
	int ret = 0;
	long num;

	if (!val)
		return -EINVAL;

	ret = strict_strtol(val, 0, &num);
	if (ret == -EINVAL || num > MAX_THROTTLE_TEMP || num < COOL_TEMP)
		return -EINVAL;

	ret = param_set_int(val, kp);

	pr_info("msm_thermal: throttle_temp = %d\n", throttle_temp);

	return ret;
}

static struct kernel_param_ops module_ops_enabled = {
	.set = set_enabled,
	.get = param_get_bool,
};

static struct kernel_param_ops module_ops_debug = {
	.set = set_debug,
	.get = param_get_bool,
};

static struct kernel_param_ops module_ops_thermal_temp = {
	.set = set_throttle_temp,
	.get = param_get_uint,
};

module_param_cb(enabled, &module_ops_enabled, &enabled, 0775);
MODULE_PARM_DESC(enabled, "msm_thermal enforce limit on cpu (Y/N)");

module_param_cb(thermal_debug, &module_ops_debug, &thermal_debug, 0775);
MODULE_PARM_DESC(enabled, "msm_thermal debug to kernel log (Y/N)");

module_param_cb(throttle_temp, &module_ops_thermal_temp, &throttle_temp, 0775);
MODULE_PARM_DESC(throttle_temp, "msm_thermal throttle temperature (C)");

int __devinit msm_thermal_init(struct msm_thermal_data *pdata)
{
	int ret = 0;

	BUG_ON(!pdata);
	BUG_ON(pdata->sensor_id >= TSENS_MAX_SENSORS);
	memcpy(&msm_thermal_info, pdata, sizeof(struct msm_thermal_data));

	enabled = 1;
	INIT_DELAYED_WORK(&check_temp_work, check_temp);
	schedule_delayed_work(&check_temp_work, 0);

	return ret;
}

static int __devinit msm_thermal_dev_probe(struct platform_device *pdev)
{
	int ret = 0;
	char *key = NULL;
	struct device_node *node = pdev->dev.of_node;
	struct msm_thermal_data data;

	memset(&data, 0, sizeof(struct msm_thermal_data));
	key = "qcom,sensor-id";
	ret = of_property_read_u32(node, key, &data.sensor_id);
	if (ret)
		goto fail;
	WARN_ON(data.sensor_id >= TSENS_MAX_SENSORS);

	key = "qcom,poll-ms";
	ret = of_property_read_u32(node, key, &data.poll_ms);
	if (ret)
		goto fail;

	key = "qcom,limit-temp";
	ret = of_property_read_u32(node, key, &data.limit_temp_degC);
	if (ret)
		goto fail;

	key = "qcom,temp-hysteresis";
	ret = of_property_read_u32(node, key, &data.temp_hysteresis_degC);
	if (ret)
		goto fail;

	key = "qcom,freq-step";
	ret = of_property_read_u32(node, key, &data.freq_step);

fail:
	if (ret)
		pr_err("%s: Failed reading node=%s, key=%s\n",
		       __func__, node->full_name, key);
	else
		ret = msm_thermal_init(&data);

	return ret;
}

static struct of_device_id msm_thermal_match_table[] = {
	{.compatible = "qcom,msm-thermal"},
	{},
};

static struct platform_driver msm_thermal_device_driver = {
	.probe = msm_thermal_dev_probe,
	.driver = {
		.name = "msm-thermal",
		.owner = THIS_MODULE,
		.of_match_table = msm_thermal_match_table,
	},
};

int __init msm_thermal_device_init(void)
{
	return platform_driver_register(&msm_thermal_device_driver);
}

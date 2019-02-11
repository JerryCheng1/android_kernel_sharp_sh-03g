/* drivers/sharp/shtps/sy3000/shtps_rmi_sub.c
 *
 * Copyright (c) 2014, Sharp. All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/input/mt.h>

#include <linux/pm_qos.h>

#include <linux/init.h>
#include <asm/processor.h>
#include <asm/uaccess.h>
#include <linux/pinctrl/pinconf-generic.h>

#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
#include <linux/cpufreq.h>
#include <linux/notifier.h>
#endif /* defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE) */

#include <sharp/shtps_dev.h>
//#include <sharp/sh_smem.h>
#include <sharp/sh_boot_manager.h>

#include "shtps_rmi.h"
#include "shtps_param_extern.h"
#include "shtps_rmi_sub.h"
#include "shtps_fwctl.h"
#include "shtps_log.h"

/* -----------------------------------------------------------------------------------
 */
#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
	static DEFINE_MUTEX(shtps_cpu_clock_ctrl_lock);
#endif /* SHTPS_CPU_CLOCK_CONTROL_ENABLE */

#if defined(SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE)
	static DEFINE_MUTEX(shtps_cpu_idle_sleep_ctrl_lock);
#endif /* SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE */

#if defined(SHTPS_CPU_SLEEP_CONTROL_FOR_FWUPDATE_ENABLE)
	static DEFINE_MUTEX(shtps_cpu_sleep_ctrl_for_fwupdate_lock);
#endif /* SHTPS_CPU_SLEEP_CONTROL_FOR_FWUPDATE_ENABLE */


#define	TPS_MUTEX_LOG_LOCK(VALUE)		SHTPS_LOG_DBG_PRINT("mutex_lock:   -> (%s)\n",VALUE)
#define	TPS_MUTEX_LOG_UNLOCK(VALUE)		SHTPS_LOG_DBG_PRINT("mutex_unlock: <- (%s)\n",VALUE)

/* -----------------------------------------------------------------------------------
 */
#if defined(SHTPS_PERFORMANCE_CHECK_PIN_ENABLE)
	#define SHTPS_PERFORMANCE_CHECK_PIN		(32)
	static int shtps_performance_check_pin_state = 0;
#endif /* SHTPS_PERFORMANCE_CHECK_PIN_ENABLE */

#if defined( SHTPS_PERFORMANCE_TIME_LOG_ENABLE )
	static int shtps_performace_log_point = 0;
	static struct timeval shtps_performance_log_tv;
#endif /* SHTPS_PERFORMANCE_TIME_LOG_ENABLE */

#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
	static void shtps_mutex_lock_cpu_clock_ctrl(void);
	static void shtps_mutex_unlock_cpu_clock_ctrl(void);
	static void shtps_perflock_enable(struct shtps_rmi_spi *ts);
	static void shtps_perflock_disable(struct shtps_rmi_spi *ts);
	static int shtps_perflock_disable_timer_start(struct shtps_rmi_spi *ts, unsigned long delay_ms);
	static void shtps_perflock_disable_delayed_work_function(struct work_struct *work);
#endif /* SHTPS_CPU_CLOCK_CONTROL_ENABLE */

#if defined(SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE)
	static void shtps_mutex_lock_cpu_idle_sleep_ctrl(void);
	static void shtps_mutex_unlock_cpu_idle_sleep_ctrl(void);
	static void shtps_wake_lock_idle_l(struct shtps_rmi_spi *ts);
	static void shtps_wake_unlock_idle_l(struct shtps_rmi_spi *ts);
#endif /* SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE */

#if defined(SHTPS_CPU_SLEEP_CONTROL_FOR_FWUPDATE_ENABLE)
	static void shtps_mutex_lock_cpu_sleep_ctrl(void);
	static void shtps_mutex_unlock_cpu_sleep_ctrl(void);
#endif /* SHTPS_CPU_SLEEP_CONTROL_FOR_FWUPDATE_ENABLE */

	static void shtps_func_open(struct shtps_rmi_spi *ts);
	static void shtps_func_close(struct shtps_rmi_spi *ts);
	static int shtps_func_enable(struct shtps_rmi_spi *ts);
	static void shtps_func_disable(struct shtps_rmi_spi *ts);

	static void shtps_func_request_async_complete(void *arg_p);
	static void shtps_func_request_sync_complete(void *arg_p);
	static void shtps_func_workq( struct work_struct *work_p );

/* -------------------------------------------------------------------------- */
struct shtps_req_msg {	//**********	SHTPS_ASYNC_OPEN_ENABLE
	struct list_head queue;
	void	(*complete)(void *context);
	void	*context;
	int		status;
	int		event;
	void	*param_p;
};

/* -------------------------------------------------------------------------- */
#if defined(SHTPS_PERFORMANCE_CHECK_ENABLE)
void shtps_performance_check_init(void)
{
	#if defined( SHTPS_PERFORMANCE_CHECK_PIN_ENABLE )
		int result;
		SHTPS_LOG_FUNC_CALL();
		result = gpio_request(SHTPS_PERFORMANCE_CHECK_PIN, "tps_test");
		if(result < 0){
			DBG_PRINTK("test pin gpio_request() error : %d\n", result);
		}

		result = gpio_tlmm_config(GPIO_CFG(SHTPS_PERFORMANCE_CHECK_PIN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if(result < 0){
			DBG_PRINTK("test pin gpio_tlmm_config() error : %d\n", result);
		}

		shtps_performance_check_pin_state = 0;
		gpio_set_value(SHTPS_PERFORMANCE_CHECK_PIN, shtps_performance_check_pin_state);
	#endif /* SHTPS_PERFORMANCE_CHECK_PIN_ENABLE */

	#if defined( SHTPS_PERFORMANCE_TIME_LOG_ENABLE )
		shtps_performace_log_point = 0;
	#endif /* SHTPS_PERFORMANCE_TIME_LOG_ENABLE */
}

void shtps_performance_check(int state)
{
	#if defined( SHTPS_PERFORMANCE_CHECK_PIN_ENABLE ) || defined( SHTPS_PERFORMANCE_TIME_LOG_ENABLE )
	//	SHTPS_LOG_FUNC_CALL();
	#endif

	#if defined( SHTPS_PERFORMANCE_CHECK_PIN_ENABLE )
		if(state == SHTPS_PERFORMANCE_CHECK_STATE_START){
			shtps_performance_check_pin_state = 1;
		}else{
			shtps_performance_check_pin_state = 
				(shtps_performance_check_pin_state == 0)? 1 : 0;
		}

		gpio_set_value(SHTPS_PERFORMANCE_CHECK_PIN, shtps_performance_check_pin_state);

		if(state == SHTPS_PERFORMANCE_CHECK_STATE_END){
			shtps_performance_check_pin_state = 0;
			gpio_set_value(SHTPS_PERFORMANCE_CHECK_PIN, shtps_performance_check_pin_state);
		}
	#endif /* SHTPS_PERFORMANCE_CHECK_PIN_ENABLE */

	#if defined( SHTPS_PERFORMANCE_TIME_LOG_ENABLE )
		if(state == SHTPS_PERFORMANCE_CHECK_STATE_START){
			shtps_performace_log_point = 1;
		}else{
			static struct timeval tv;
			do_gettimeofday(&tv);

			DBG_PRINTK("[performace] pt:%02d time:%ldus\n",
				shtps_performace_log_point++,
				(tv.tv_sec * 1000000 + tv.tv_usec) - 
					(shtps_performance_log_tv.tv_sec * 1000000 + shtps_performance_log_tv.tv_usec));
		}
		do_gettimeofday(&shtps_performance_log_tv);
	#endif /* SHTPS_PERFORMANCE_TIME_LOG_ENABLE */
}
#endif	/* SHTPS_PERFORMANCE_CHECK_ENABLE */

/* -----------------------------------------------------------------------------------
 */
#ifdef SHTPS_SEND_SHTERM_EVENT_ENABLE
void shtps_send_shterm_event(int event_num)
{
	shbattlog_info_t info;
	SHTPS_LOG_FUNC_CALL();
	memset(&info, 0x00, sizeof(info));
	info.event_num = event_num;
	shterm_k_set_event(&info);
}
#endif  /* SHTPS_SEND_SHTERM_EVENT_ENABLE */

/* -----------------------------------------------------------------------------------
 */
#if defined(SHTPS_PROXIMITY_SUPPORT_ENABLE)
int shtps_proximity_state_check(struct shtps_rmi_spi *ts)
{
	int state = -1;
	int data = -1;

	SHTPS_LOG_FUNC_CALL();

	SHTPS_LOG_DBG_PRINT("[proximity] check state start\n");
	PROX_stateread_func(&state, &data);
	SHTPS_LOG_DBG_PRINT("[proximity] check state end(state:%d, data:%d)\n",state, data);

	if (state == SHTPS_PROXIMITY_ENABLE &&
		data == SHTPS_PROXIMITY_NEAR) {
		return 1;
	} else {
		return 0;
	}
}

int shtps_proximity_check(struct shtps_rmi_spi *ts)
{
	int data = -1;

	SHTPS_LOG_FUNC_CALL();

	SHTPS_LOG_DBG_PRINT("[proximity] check start\n");
	PROX_dataread_func(&data);
	SHTPS_LOG_DBG_PRINT("[proximity] check end(data:%d)\n",data);

	return data;
}

void shtps_lpwg_proximity_check_delayed_work_function(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct shtps_rmi_spi *ts = container_of(dw, struct shtps_rmi_spi, proximity_check_delayed_work);

	SHTPS_LOG_FUNC_CALL();
	shtps_func_request_async(ts, SHTPS_FUNC_REQ_EVEMT_PROXIMITY_CHECK);
}

void shtps_lpwg_proximity_check_cancel(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	cancel_delayed_work(&ts->proximity_check_delayed_work);
}

void shtps_lpwg_proximity_check_start(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	shtps_lpwg_proximity_check_cancel(ts);
	schedule_delayed_work(&ts->proximity_check_delayed_work, msecs_to_jiffies(SHTPS_LPWG_PROXIMITY_CHECK_PREWAIT));

	SHTPS_LOG_DBG_PRINT("[LPWG] notify interval start\n");
}

static void shtps_func_proximity_check(struct shtps_rmi_spi *ts)
{
	int proximity_data = -1;

	wake_lock(&ts->wake_lock_proximity);
    pm_qos_update_request(&ts->pm_qos_lock_idle_proximity, SHTPS_QOS_LATENCY_DEF_VALUE);

	proximity_data = shtps_proximity_check(ts);

	#if defined(SHTPS_LPWG_MODE_ENABLE)
	{
		unsigned long time = 0;

		ts->lpwg_proximity_get_data = proximity_data;
		if(ts->lpwg_proximity_get_data == SHTPS_PROXIMITY_NEAR){
			SHTPS_LOG_DBG_PRINT("[LPWG][Func] proximity near");

			shtps_mutex_lock_ctrl();
			if(ts->lpwg.is_notified != 0){
				if(time_after(jiffies, ts->lpwg.notify_time + msecs_to_jiffies(SHTPS_LPWG_MIN_NOTIFY_CANCEL_INTERVAL))){
					time = 0;
				}else{
					time = jiffies_to_msecs(((ts->lpwg.notify_time + msecs_to_jiffies(SHTPS_LPWG_MIN_NOTIFY_CANCEL_INTERVAL)) - jiffies)
												% SHTPS_LPWG_MIN_NOTIFY_CANCEL_INTERVAL);
				}
			}
			shtps_mutex_unlock_ctrl();

			if(time > 0){
				SHTPS_LOG_DBG_PRINT("[LPWG] cancel notify wait <%lums>\n", time);
				msleep(time);
			}

			shtps_mutex_lock_ctrl();
			if(ts->lpwg.is_notified != 0){
				shtps_notify_cancel_wakeup_event(ts);
				ts->lpwg.is_notified = 0;
			}
			shtps_mutex_unlock_ctrl();
		}
	}
	#endif /* SHTPS_LPWG_MODE_ENABLE */

    pm_qos_update_request(&ts->pm_qos_lock_idle_proximity, PM_QOS_DEFAULT_VALUE);
	wake_unlock(&ts->wake_lock_proximity);
}
#endif /* SHTPS_PROXIMITY_SUPPORT_ENABLE */

/* -----------------------------------------------------------------------------------
 */
#if defined(SHTPS_CPU_CLOCK_CONTROL_ENABLE)
static DEFINE_PER_CPU(unsigned int, perflock_freq) = 0;


static void shtps_mutex_lock_cpu_clock_ctrl(void)
{
	TPS_MUTEX_LOG_LOCK("shtps_cpu_clock_ctrl_lock");
	mutex_lock(&shtps_cpu_clock_ctrl_lock);
}

static void shtps_mutex_unlock_cpu_clock_ctrl(void)
{
	TPS_MUTEX_LOG_UNLOCK("shtps_cpu_clock_ctrl_lock");
	mutex_unlock(&shtps_cpu_clock_ctrl_lock);
}

static void shtps_perflock_enable(struct shtps_rmi_spi *ts)
{
	int cpu;
	shtps_mutex_lock_cpu_clock_ctrl();

	for_each_possible_cpu(cpu) {
		if (cpu <= 3) {
			#if defined(SHTPS_DEF_CTRL_CPU_CLOCK_LINKED_LCD_BRIGHT_ENABLE)
				if( ((ts->lpmode_req_state | ts->lpmode_continuous_req_state) & SHTPS_LPMODE_REQ_LCD_BRIGHT) == 0 ){
					per_cpu(perflock_freq, cpu) = SHTPS_PERF_LOCK_CLOCK_FREQUENCY;
				}else{
					per_cpu(perflock_freq, cpu) = SHTPS_PERF_LOCK_CLOCK_FREQUENCY_LCD_LPM;
				}
			#else
				per_cpu(perflock_freq, cpu) = SHTPS_PERF_LOCK_CLOCK_FREQUENCY;
			#endif /* SHTPS_DEF_CTRL_CPU_CLOCK_LINKED_LCD_BRIGHT_ENABLE */
		}
	}
	for_each_online_cpu(cpu) {
		cpufreq_update_policy(cpu);
	}
	SHTPS_LOG_DBG_PRINT("perf_lock start\n");

	shtps_mutex_unlock_cpu_clock_ctrl();
}

static void shtps_perflock_disable(struct shtps_rmi_spi *ts)
{
	int cpu;
	shtps_mutex_lock_cpu_clock_ctrl();

	for_each_possible_cpu(cpu) {
		if (cpu <= 3) {
			per_cpu(perflock_freq, cpu) = 0;
		}
	}
	for_each_online_cpu(cpu) { 
		cpufreq_update_policy(cpu);
	}
	SHTPS_LOG_DBG_PRINT("perf_lock end\n");

	shtps_mutex_unlock_cpu_clock_ctrl();
}

static int shtps_perflock_ctrl_callback(struct notifier_block *nb, unsigned long event, void *data)
{
	struct cpufreq_policy *policy = data;
	switch (event) {
	case CPUFREQ_ADJUST:
//		SHTPS_LOG_DBG_PRINT("set perflock [%d]\n", perflock_freq);
		cpufreq_verify_within_limits(policy, per_cpu(perflock_freq, policy->cpu), UINT_MAX); 
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}

static int shtps_perflock_disable_timer_start(struct shtps_rmi_spi *ts, unsigned long delay_ms)
{
	cancel_delayed_work(&ts->cpu_clock_ctrl_p->perf_lock_disable_delayed_work);
	schedule_delayed_work(&ts->cpu_clock_ctrl_p->perf_lock_disable_delayed_work, msecs_to_jiffies(delay_ms));

	return 0;
}

static void shtps_perflock_disable_delayed_work_function(struct work_struct *work)
{
	struct delayed_work *dw = container_of(work, struct delayed_work, work);
	struct shtps_cpu_clock_ctrl_info *cpuctrl = container_of(dw, struct shtps_cpu_clock_ctrl_info, perf_lock_disable_delayed_work);
	//struct shtps_rmi_spi *ts = container_of(cpuctrl, struct shtps_rmi_spi, cpu_clock_ctrl_p);
	struct shtps_rmi_spi *ts = cpuctrl->ts_p;

	SHTPS_LOG_FUNC_CALL();

	shtps_perflock_disable(ts);
	SHTPS_LOG_DBG_PRINT("perf_lock end by Timer\n");

	return;
}

static struct notifier_block shtps_perflock_ctrl_notifier = { 
	.notifier_call = shtps_perflock_ctrl_callback,
};

void shtps_perflock_register_notifier(void)
{
	if( cpufreq_register_notifier(&shtps_perflock_ctrl_notifier, CPUFREQ_POLICY_NOTIFIER) ){
		pr_err("%s: cannot register cpufreq notifier\n", __func__);
	}
}

void shtps_perflock_unregister_notifier(void)
{
	if( cpufreq_unregister_notifier(&shtps_perflock_ctrl_notifier, CPUFREQ_POLICY_NOTIFIER) ){
		pr_err("%s: cannot unregister cpufreq notifier\n", __func__);
	}
}

#if defined(SHTPS_CPU_CLOCK_CONTROL_CHECK_CURFREQ_ENABLE)
static int shtps_read_curfreq_file(char *filename, long *freq_p)
{
	int ret = 0;
	struct file *file;
	mm_segment_t fs;
	char buf[16];
	int nr_read;

	file = filp_open(filename, O_RDONLY, 0); 

	if(IS_ERR(file)){
		return -1;
	}

	fs = get_fs();
	set_fs(get_ds());

	nr_read = file->f_op->read(file, buf, sizeof(buf)-1, &file->f_pos);
	buf[nr_read] = '\0';

	if(nr_read == 0){
		ret = -1;
	}else{
		ret = kstrtol(buf, 10, freq_p);
	}

	set_fs(fs);
	filp_close(file, NULL);

	return ret;
}

static int shtps_check_curfreq_condition(void)
{
	long cpu0_freq;
	long cpu3_freq;
	if(shtps_read_curfreq_file("sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq", &cpu0_freq) != 0){
		SHTPS_LOG_DBG_PRINT("File not found : sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq \n");
		return 0;
	}

	if(shtps_read_curfreq_file("sys/devices/system/cpu/cpu3/cpufreq/scaling_cur_freq", &cpu3_freq) != 0){
		SHTPS_LOG_DBG_PRINT("File not found : sys/devices/system/cpu/cpu3/cpufreq/scaling_cur_freq \n");
		return 0;
	}

	if(cpu0_freq > 960000 || cpu3_freq > 960000){
		SHTPS_LOG_DBG_PRINT("cpu_freq > 960000\n");
		return 0;
	}

	return 1;
}
#endif /* SHTPS_CPU_CLOCK_CONTROL_CHECK_CURFREQ_ENABLE */

void shtps_perflock_enable_start(struct shtps_rmi_spi *ts, u8 event, struct shtps_touch_info *info)
{
	int fingerMax = shtps_get_fingermax(ts);
	int i;
	int perf_lock_enable_time;
	int x_diff;
	int y_diff;
	int xy_diff;
	int max_diff;


	if(SHTPS_PERF_LOCK_ENABLE_ALLEVENTS == 1
		#if defined(SHTPS_DEF_CTRL_CPU_CLOCK_LINKED_LCD_BRIGHT_ENABLE)
			&& (((ts->lpmode_req_state | ts->lpmode_continuous_req_state) & SHTPS_LPMODE_REQ_LCD_BRIGHT) == 0)
		#endif /* SHTPS_DEF_CTRL_CPU_CLOCK_LINKED_LCD_BRIGHT_ENABLE */
		){

		#if defined(SHTPS_CPU_CLOCK_CONTROL_CHECK_CURFREQ_ENABLE)
			if(shtps_check_curfreq_condition() != 1)	return;
		#endif /*SHTPS_CPU_CLOCK_CONTROL_CHECK_CURFREQ_ENABLE*/

		perf_lock_enable_time = SHTPS_PERF_LOCK_ENABLE_TIME_MS;

		if(event == SHTPS_EVENT_TU){
			max_diff = 0;

			for(i = 0; i < fingerMax; i++){
				if( (info->fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH) &&
					(ts->cpu_clock_ctrl_p->fingers_hist[0][i].state != SHTPS_TOUCH_STATE_NO_TOUCH) ){
					if(ts->cpu_clock_ctrl_p->fingers_hist[1][i].state == SHTPS_TOUCH_STATE_NO_TOUCH){
						x_diff = 0;
						y_diff = 0;
					}
					else{
						x_diff = SHTPS_ABS_CALC(ts->cpu_clock_ctrl_p->fingers_hist[1][i].x, ts->cpu_clock_ctrl_p->fingers_hist[0][i].x);
						y_diff = SHTPS_ABS_CALC(ts->cpu_clock_ctrl_p->fingers_hist[1][i].y, ts->cpu_clock_ctrl_p->fingers_hist[0][i].y);
					}
					xy_diff = (x_diff > y_diff ? x_diff : y_diff);
					if(xy_diff > max_diff){
						max_diff = xy_diff;
					}
				}
			}

			if(max_diff >= SHTPS_PERF_LOCK_ENABLE_FLICK_DIST){
				perf_lock_enable_time = SHTPS_PERF_LOCK_ENABLE_TIME_MS_FOR_FLICK;
			}

			memset(ts->cpu_clock_ctrl_p->fingers_hist, 0, sizeof(ts->cpu_clock_ctrl_p->fingers_hist));
		}
		else{
			memcpy(ts->cpu_clock_ctrl_p->fingers_hist[1], ts->cpu_clock_ctrl_p->fingers_hist[0], sizeof(ts->cpu_clock_ctrl_p->fingers_hist[0]));
			memcpy(ts->cpu_clock_ctrl_p->fingers_hist[0], info->fingers, sizeof(info->fingers));
		}

		if( shtps_fwctl_get_dev_state(ts) == SHTPS_DEV_STATE_ACTIVE
		#if !defined(SHTPS_PERFLOCK_DOZE_DISABLE)
			|| shtps_fwctl_get_dev_state(ts) == SHTPS_DEV_STATE_DOZE
		#endif /*!defined(SHTPS_PERFLOCK_DOZE_DISABLE)*/
		 ){
			shtps_perflock_enable(ts);
			shtps_perflock_disable_timer_start(ts, perf_lock_enable_time /* ts->perf_lock_enable_time_ms */);
			SHTPS_LOG_DBG_PRINT("perf_lock start (%dms) by %s\n",
								perf_lock_enable_time,
								( (event == SHTPS_EVENT_TD) ? "TouchDown" :
								  (event == SHTPS_EVENT_DRAG) ? "Drag" :
								  (event == SHTPS_EVENT_TU) ? "TouchUp" :
								  (event == SHTPS_EVENT_MTDU) ? "MultiTouchUpDown" : "unknown" )
								);
		}
	}
	else{
		if(event == SHTPS_EVENT_TD){
			
			#if defined(SHTPS_CPU_CLOCK_CONTROL_CHECK_CURFREQ_ENABLE)
				if(shtps_check_curfreq_condition() != 1)	return;
			#endif /*SHTPS_CPU_CLOCK_CONTROL_CHECK_CURFREQ_ENABLE*/
			
			if( shtps_fwctl_get_dev_state(ts) == SHTPS_DEV_STATE_ACTIVE
			#if !defined(SHTPS_PERFLOCK_DOZE_DISABLE)
				|| shtps_fwctl_get_dev_state(ts) == SHTPS_DEV_STATE_DOZE
			#endif /*!defined(SHTPS_PERFLOCK_DOZE_DISABLE)*/
			 ){
				shtps_perflock_enable(ts);
				shtps_perflock_disable_timer_start(ts, SHTPS_PERF_LOCK_ENABLE_TIME_MS /* ts->perf_lock_enable_time_ms */);
				SHTPS_LOG_DBG_PRINT("perf_lock start by TouchDown\n");
			}

		}else if(event == SHTPS_EVENT_DRAG){
			if(ts->cpu_clock_ctrl_p->report_event == SHTPS_EVENT_TD){
				
				#if defined(SHTPS_CPU_CLOCK_CONTROL_CHECK_CURFREQ_ENABLE)
					if(shtps_check_curfreq_condition() != 1)	return;
				#endif /*SHTPS_CPU_CLOCK_CONTROL_CHECK_CURFREQ_ENABLE*/
				
				if( shtps_fwctl_get_dev_state(ts) == SHTPS_DEV_STATE_ACTIVE
				#if !defined(SHTPS_PERFLOCK_DOZE_DISABLE)
					|| shtps_fwctl_get_dev_state(ts) == SHTPS_DEV_STATE_DOZE
				#endif /*!defined(SHTPS_PERFLOCK_DOZE_DISABLE)*/
				 ){
					shtps_perflock_enable(ts);
					shtps_perflock_disable_timer_start(ts, SHTPS_PERF_LOCK_ENABLE_TIME_MS /* ts->perf_lock_enable_time_ms */);
					SHTPS_LOG_DBG_PRINT("perf_lock start by Drag\n");
				}
			}
		}
		#if !defined( SHTPS_PERFLOCK_TU_RELEASE_DISABLE )
		else if(event == SHTPS_EVENT_TU){
			shtps_perflock_disable(ts);
			SHTPS_LOG_DBG_PRINT("perf_lock end by TouchUp\n");
		}
		#endif /* SHTPS_PERFLOCK_TU_RELEASE_DISABLE */
	}
}

void shtps_perflock_sleep(struct shtps_rmi_spi *ts)
{
	shtps_perflock_set_event(ts, SHTPS_EVENT_TU);	/* ts->cpu_clock_ctrl_p->report_event = SHTPS_EVENT_TU; */
	shtps_perflock_disable(ts);
	SHTPS_LOG_DBG_PRINT("perf_lock end by ForceTouchUp\n");

	memset(ts->cpu_clock_ctrl_p->fingers_hist, 0, sizeof(ts->cpu_clock_ctrl_p->fingers_hist));
}

void shtps_perflock_init(struct shtps_rmi_spi *ts)
{
	ts->cpu_clock_ctrl_p = kzalloc(sizeof(struct shtps_cpu_clock_ctrl_info), GFP_KERNEL);
	if(ts->cpu_clock_ctrl_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}

	shtps_perflock_set_event(ts, SHTPS_EVENT_TU);	/* ts->cpu_clock_ctrl_p->report_event = SHTPS_EVENT_TU; */
	//ts->cpu_clock_ctrl_p->perf_lock_enable_time_ms = SHTPS_perflock_ENABLE_TIME_MS;
//	perf_lock_init(&ts->cpu_clock_ctrl_p->perf_lock, SHTPS_perflock_CLOCK_FREQUENCY, "shtps_perf_lock");
	INIT_DELAYED_WORK(&ts->cpu_clock_ctrl_p->perf_lock_disable_delayed_work, shtps_perflock_disable_delayed_work_function);
	ts->cpu_clock_ctrl_p->ts_p = ts;
}

void shtps_perflock_deinit(struct shtps_rmi_spi *ts)
{
	if(ts->cpu_clock_ctrl_p)	kfree(ts->cpu_clock_ctrl_p);
	ts->cpu_clock_ctrl_p = NULL;

}

void shtps_perflock_set_event(struct shtps_rmi_spi *ts, u8 event)
{
	ts->cpu_clock_ctrl_p->report_event = event;
}
#endif /* SHTPS_CPU_CLOCK_CONTROL_ENABLE */

/* -----------------------------------------------------------------------------------
 */
#if defined(SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE)
static void shtps_mutex_lock_cpu_idle_sleep_ctrl(void)
{
	TPS_MUTEX_LOG_LOCK("shtps_cpu_idle_sleep_ctrl_lock");
	mutex_lock(&shtps_cpu_idle_sleep_ctrl_lock);
}

static void shtps_mutex_unlock_cpu_idle_sleep_ctrl(void)
{
	TPS_MUTEX_LOG_UNLOCK("shtps_cpu_idle_sleep_ctrl_lock");
	mutex_unlock(&shtps_cpu_idle_sleep_ctrl_lock);
}

static void shtps_wake_lock_idle_l(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	shtps_mutex_lock_cpu_idle_sleep_ctrl();
	if(ts->cpu_idle_sleep_ctrl_p->wake_lock_idle_state == 0){
		SHTPS_LOG_DBG_PRINT("wake_lock_idle on\n");
		pm_qos_update_request(&ts->cpu_idle_sleep_ctrl_p->qos_cpu_latency, SHTPS_QOS_LATENCY_DEF_VALUE);
		ts->cpu_idle_sleep_ctrl_p->wake_lock_idle_state = 1;
	}
	shtps_mutex_unlock_cpu_idle_sleep_ctrl();
}

static void shtps_wake_unlock_idle_l(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	shtps_mutex_lock_cpu_idle_sleep_ctrl();
	if(ts->cpu_idle_sleep_ctrl_p->wake_lock_idle_state != 0){
		SHTPS_LOG_DBG_PRINT("wake_lock_idle off\n");
		pm_qos_update_request(&ts->cpu_idle_sleep_ctrl_p->qos_cpu_latency, PM_QOS_DEFAULT_VALUE);
		ts->cpu_idle_sleep_ctrl_p->wake_lock_idle_state = 0;
	}
	shtps_mutex_unlock_cpu_idle_sleep_ctrl();
}

void shtps_wake_lock_idle(struct shtps_rmi_spi *ts)
{
	if(SHTPS_STATE_ACTIVE == ts->state_mgr.state){
		shtps_wake_lock_idle_l(ts);
	}
}

void shtps_wake_unlock_idle(struct shtps_rmi_spi *ts)
{
	shtps_wake_unlock_idle_l(ts);
}

void shtps_cpu_idle_sleep_wake_lock_init( struct shtps_rmi_spi *ts )
{
	ts->cpu_idle_sleep_ctrl_p = kzalloc(sizeof(struct shtps_cpu_idle_sleep_ctrl_info), GFP_KERNEL);
	if(ts->cpu_idle_sleep_ctrl_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}
	// ts->cpu_idle_sleep_ctrl_p->wake_lock_idle_state = 0;	// no need
	ts->cpu_idle_sleep_ctrl_p->qos_cpu_latency.type = PM_QOS_REQ_AFFINE_CORES;
	ts->cpu_idle_sleep_ctrl_p->qos_cpu_latency.cpus_affine.bits[0] = 0x0f;
	pm_qos_add_request(&ts->cpu_idle_sleep_ctrl_p->qos_cpu_latency, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
}

void shtps_cpu_idle_sleep_wake_lock_deinit( struct shtps_rmi_spi *ts )
{
	pm_qos_remove_request(&ts->cpu_idle_sleep_ctrl_p->qos_cpu_latency);

	if(ts->cpu_idle_sleep_ctrl_p)	kfree(ts->cpu_idle_sleep_ctrl_p);
	ts->cpu_idle_sleep_ctrl_p = NULL;
}

#endif /* SHTPS_CPU_IDLE_SLEEP_CONTROL_ENABLE */

/* -----------------------------------------------------------------------------------
 */
#if defined(SHTPS_CPU_SLEEP_CONTROL_FOR_FWUPDATE_ENABLE)
static void shtps_mutex_lock_cpu_sleep_ctrl(void)
{
	TPS_MUTEX_LOG_LOCK("shtps_cpu_sleep_ctrl_for_fwupdate_lock");
	mutex_lock(&shtps_cpu_sleep_ctrl_for_fwupdate_lock);
}

static void shtps_mutex_unlock_cpu_sleep_ctrl(void)
{
	TPS_MUTEX_LOG_UNLOCK("shtps_cpu_sleep_ctrl_for_fwupdate_lock");
	mutex_unlock(&shtps_cpu_sleep_ctrl_for_fwupdate_lock);
}

void shtps_wake_lock_for_fwupdate(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	shtps_mutex_lock_cpu_sleep_ctrl();
	if(ts->cpu_sleep_ctrl_fwupdate_p->wake_lock_for_fwupdate_state == 0){
		SHTPS_LOG_DBG_PRINT("wake_lock_for_fwupdate on\n");
		wake_lock(&ts->cpu_sleep_ctrl_fwupdate_p->wake_lock_for_fwupdate);
		pm_qos_update_request(&ts->cpu_sleep_ctrl_fwupdate_p->qos_cpu_latency_for_fwupdate, SHTPS_QOS_LATENCY_DEF_VALUE);
		ts->cpu_sleep_ctrl_fwupdate_p->wake_lock_for_fwupdate_state = 1;
	}
	shtps_mutex_unlock_cpu_sleep_ctrl();
}

void shtps_wake_unlock_for_fwupdate(struct shtps_rmi_spi *ts)
{
	SHTPS_LOG_FUNC_CALL();

	shtps_mutex_lock_cpu_sleep_ctrl();
	if(ts->cpu_sleep_ctrl_fwupdate_p->wake_lock_for_fwupdate_state != 0){
		SHTPS_LOG_DBG_PRINT("wake_lock_for_fwupdate off\n");
		wake_unlock(&ts->cpu_sleep_ctrl_fwupdate_p->wake_lock_for_fwupdate);
		pm_qos_update_request(&ts->cpu_sleep_ctrl_fwupdate_p->qos_cpu_latency_for_fwupdate, PM_QOS_DEFAULT_VALUE);
		ts->cpu_sleep_ctrl_fwupdate_p->wake_lock_for_fwupdate_state = 0;
	}
	shtps_mutex_unlock_cpu_sleep_ctrl();
}

void shtps_fwupdate_wake_lock_init( struct shtps_rmi_spi *ts )
{
	ts->cpu_sleep_ctrl_fwupdate_p = kzalloc(sizeof(struct shtps_cpu_sleep_ctrl_fwupdate_info), GFP_KERNEL);
	if(ts->cpu_sleep_ctrl_fwupdate_p == NULL){
		PR_ERROR("memory allocation error:%s()\n", __func__);
		return;
	}

	//ts->cpu_sleep_ctrl_fwupdate_p->wake_lock_for_fwupdate_state = 0;	// no need
	wake_lock_init(&ts->cpu_sleep_ctrl_fwupdate_p->wake_lock_for_fwupdate, WAKE_LOCK_SUSPEND, "shtps_wake_lock_for_fwupdate");
	ts->cpu_sleep_ctrl_fwupdate_p->qos_cpu_latency_for_fwupdate.type = PM_QOS_REQ_AFFINE_CORES;
	ts->cpu_sleep_ctrl_fwupdate_p->qos_cpu_latency_for_fwupdate.cpus_affine.bits[0] = 0x0f;
	pm_qos_add_request(&ts->cpu_sleep_ctrl_fwupdate_p->qos_cpu_latency_for_fwupdate, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
}

void shtps_fwupdate_wake_lock_deinit( struct shtps_rmi_spi *ts )
{
	pm_qos_remove_request(&ts->cpu_sleep_ctrl_fwupdate_p->qos_cpu_latency_for_fwupdate);
	wake_lock_destroy(&ts->cpu_sleep_ctrl_fwupdate_p->wake_lock_for_fwupdate);

	if(ts->cpu_sleep_ctrl_fwupdate_p)	kfree(ts->cpu_sleep_ctrl_fwupdate_p);
	ts->cpu_sleep_ctrl_fwupdate_p = NULL;
}

#endif /* SHTPS_CPU_SLEEP_CONTROL_FOR_FWUPDATE_ENABLE */

/* -----------------------------------------------------------------------------------
 */
#if defined( SHTPS_CHECK_CRC_ERROR_ENABLE )
void shtps_func_check_crc_error(struct shtps_rmi_spi *ts)
{
	u8 buf;
	const unsigned char* fw_data = NULL;
	u8 update = 0;

	SHTPS_LOG_FUNC_CALL();
	if(shtps_tpin_enable_check(ts) != 0){
		return;
	}
	
	if(shtps_start(ts) == 0){
		shtps_wait_startup(ts);
	}

	shtps_fwctl_get_device_status(ts, &buf);

	if((buf & 0x0F) == 4 || (buf & 0x0F) == 5 || (buf & 0x0F) == 6){
		SHTPS_LOG_ERR_PRINT("Touch panel CRC error detect\n");
		update = 1;
	}

	if(update != 0){
		fw_data = shtps_fwdata_builtin(ts);
		if(fw_data){
			int ret;
			int retry = 5;
			do{
				ret = shtps_fw_update(ts, fw_data);
				request_event(ts, SHTPS_EVENT_STOP, 0);
			}while(ret != 0 && (retry-- > 0));
		}
	}

	if(shtps_start(ts) == 0){
		shtps_wait_startup(ts);
	}
}
#endif /* if defined( SHTPS_CHECK_CRC_ERROR_ENABLE ) */

/* -----------------------------------------------------------------------------------
 */
static void shtps_func_lcd_on(struct shtps_rmi_spi *ts)
{
	#if defined(SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE)
		ts->system_boot_mode = SH_BOOT_NORMAL;
	#endif /* SHTPS_SYSTEM_BOOT_MODE_CHECK_ENABLE */

	request_event(ts, SHTPS_EVENT_WAKEUP, 0);
}

static void shtps_func_lcd_off(struct shtps_rmi_spi *ts)
{
	#if defined(SHTPS_DYNAMIC_RESET_CONTROL_ENABLE)
		shtps_filter_dynamic_reset_sleep_process(ts);
	#endif /* SHTPS_DYNAMIC_RESET_CONTROL_ENABLE */

	request_event(ts, SHTPS_EVENT_SLEEP, 0);
}

/* -----------------------------------------------------------------------------------
 */
#if defined(SHTPS_DEF_LPWG_GRIP_PROC_ASYNC_ENABLE)
static void shtps_func_grip_on(struct shtps_rmi_spi *ts)
{
	shtps_lpwg_grip_set_state(ts, 1);
}

static void shtps_func_grip_off(struct shtps_rmi_spi *ts)
{
	if(SHTPS_PRM_LPWG_GRIP_OFF_TPS_RESET_ENABLE != 0)
	{
		int driver_state;
		u8 lpwg_state;

		shtps_mutex_lock_ctrl();
		driver_state = ts->state_mgr.state;
		lpwg_state = ts->lpwg.lpwg_state;
		shtps_mutex_unlock_ctrl();

		if( (driver_state == SHTPS_STATE_SLEEP) &&
			(lpwg_state == SHTPS_LPWG_STATE_GRIP_ONLY) )
		{
			shtps_mutex_lock_ctrl();
			ts->lpwg.lpwg_switch = 0;
			ts->lpwg.tu_rezero_req = 0;
			ts->lpwg.block_touchevent = 0;
			ts->lpwg.grip_state = 0;
			SHTPS_LOG_DBG_PRINT("[LPWG] grip_state = %d\n", ts->lpwg.grip_state);
			shtps_mutex_unlock_ctrl();

			SHTPS_LOG_DBG_PRINT("[LPWG] %s() grip off tps reset start\n", __func__);
			shtps_shutdown(ts);

			shtps_set_startup_min_time(ts, SHTPS_PRM_STARTUP_MIN_TIME);
			if(shtps_start(ts) == 0){
				shtps_wait_startup(ts);
			}
			SHTPS_LOG_DBG_PRINT("[LPWG] %s() grip off tps reset end\n", __func__);
			shtps_set_startup_min_time(ts, SHTPS_STARTUP_MIN_TIME);

			request_event(ts, SHTPS_EVENT_SLEEP, 0);
		}
		else{
			shtps_lpwg_grip_set_state(ts, 0);
		}
	}
	else{
		shtps_lpwg_grip_set_state(ts, 0);
	}
}
#endif /* SHTPS_DEF_LPWG_GRIP_PROC_ASYNC_ENABLE */

/* -----------------------------------------------------------------------------------
 */
#if defined( SHTPS_TPIN_CHECK_ENABLE ) || defined( SHTPS_CHECK_CRC_ERROR_ENABLE )
int shtps_tpin_enable_check(struct shtps_rmi_spi *ts)
{
	int ret;
    int val;

	SHTPS_LOG_FUNC_CALL();
    gpio_request(SHTPS_GPIO_TPIN_NO, "tpin");

    ret = pin_config_set(SHTPS_GPIO_TPIN_DEVNAME, SHTPS_GPIO_TPIN_NAME, PIN_CONFIG_BIAS_PULL_UP);
	if(ret){
		SHTPS_LOG_DBG_PRINT("%s() pin_config_set(set) error[%d]\n", __func__, ret);
	}

	mb();
	udelay(5);
    val = gpio_get_value(SHTPS_GPIO_TPIN_NO);
	SHTPS_LOG_DBG_PRINT("%s() gpio_get_value() val = %d\n", __func__, val);

    ret = pin_config_set(SHTPS_GPIO_TPIN_DEVNAME, SHTPS_GPIO_TPIN_NAME, PIN_CONFIG_BIAS_PULL_DOWN);
	if(ret){
		SHTPS_LOG_DBG_PRINT("%s() pin_config_set(reset) error[%d]\n", __func__, ret);
	}

    gpio_free(SHTPS_GPIO_TPIN_NO);

	if(!val) {
		SHTPS_LOG_ERR_PRINT("Upper unit does not exist.\n");
		return -1;
	}
	return 0;
}
#endif /* SHTPS_TPIN_CHECK_ENABLE || SHTPS_CHECK_CRC_ERROR_ENABLE*/

/* -----------------------------------------------------------------------------------
 */
static void shtps_func_open(struct shtps_rmi_spi *ts)
{
	#if defined( SHTPS_BOOT_FWUPDATE_ENABLE )
		if( shtps_boot_fwupdate_enable_check(ts) != 0 ){
			u8 buf;
			const unsigned char* fw_data = NULL;
			int ver;
			u8 update = 0;

			if(shtps_start(ts) == 0){
				shtps_wait_startup(ts);
			}

			shtps_fwctl_get_device_status(ts, &buf);

			#if defined( SHTPS_BOOT_FWUPDATE_FORCE_UPDATE )
				ver = shtps_fwver(ts);
				SHTPS_LOG_ERR_PRINT("fw version = 0x%04x\n", ver);
				if(ver != shtps_fwver_builtin(ts)){
					update = 1;
				}
			#else
				if(shtps_fwup_flag_check() > 0){
					ver = shtps_fwver(ts);
					if(ver != shtps_fwver_builtin(ts)){
						update = 1;
					}
				}
			#endif /* if defined( SHTPS_BOOT_FWUPDATE_FORCE_UPDATE ) */

			if((buf & 0x0F) == 4 || (buf & 0x0F) == 5 || (buf & 0x0F) == 6){
				#ifdef SHTPS_SEND_SHTERM_EVENT_ENABLE
					shtps_send_shterm_event(SHBATTLOG_EVENT_TPS_CRC_ERROR);
					SHTPS_LOG_ERR_PRINT("SHBATTLOG_EVENT_TPS_CRC_ERROR (first check)\n");
				#endif /* SHTPS_SEND_SHTERM_EVENT_ENABLE */
				SHTPS_LOG_ERR_PRINT("Touch panel CRC error detect\n");
				update = 1;
			}

			if(update != 0){
				fw_data = shtps_fwdata_builtin(ts);
				if(fw_data){
					int ret;
					int retry = 5;
					do{
						ret = shtps_fw_update(ts, fw_data);
						request_event(ts, SHTPS_EVENT_STOP, 0);
						#ifdef SHTPS_SEND_SHTERM_EVENT_ENABLE
							if (ret != 0) {
								shtps_send_shterm_event(SHBATTLOG_EVENT_TPS_CRC_ERROR);
								SHTPS_LOG_ERR_PRINT("SHBATTLOG_EVENT_TPS_CRC_ERROR (retry=%d)\n", retry);
							}
						#endif /* SHTPS_SEND_SHTERM_EVENT_ENABLE */
					}while(ret != 0 && (retry-- > 0));
					
					#ifdef SHTPS_SEND_SHTERM_EVENT_ENABLE
						if (ret != 0 && retry < 0) {
							shtps_send_shterm_event(SHBATTLOG_EVENT_TPS_CRC_ERROR_MAX);
							SHTPS_LOG_ERR_PRINT("SHBATTLOG_EVENT_TPS_CRC_ERROR_MAX\n");
						} else {
							shtps_send_shterm_event(SHBATTLOG_EVENT_TPS_CRC_ERROR_FIX);
							SHTPS_LOG_ERR_PRINT("SHBATTLOG_EVENT_TPS_CRC_ERROR_FIX\n");
						}
					#endif /* SHTPS_SEND_SHTERM_EVENT_ENABLE */
				}
			}
		}
		shtps_fwup_flag_clear();
	#endif /* #if defined( SHTPS_BOOT_FWUPDATE_ENABLE ) */

	if(shtps_start(ts) == 0){
		shtps_wait_startup(ts);
	}
}

static void shtps_func_close(struct shtps_rmi_spi *ts)
{
	shtps_shutdown(ts);
}

static int shtps_func_enable(struct shtps_rmi_spi *ts)
{
	if(shtps_start(ts) != 0){
		return -EFAULT;
	}
	if(shtps_wait_startup(ts) != 0){
		return -EFAULT;
	}

	return 0;
}

static void shtps_func_disable(struct shtps_rmi_spi *ts)
{
	shtps_shutdown(ts);
}


static void shtps_func_request_async_complete(void *arg_p)
{
	kfree( arg_p );
}

void shtps_func_request_async( struct shtps_rmi_spi *ts, int event)
{
	struct shtps_req_msg		*msg_p;
	unsigned long	flags;

	msg_p = (struct shtps_req_msg *)kzalloc( sizeof( struct shtps_req_msg ), GFP_KERNEL );
	if ( msg_p == NULL ){
		SHTPS_LOG_ERR_PRINT("Out of memory [event:%d]\n", event);
		return;
	}

	msg_p->complete = shtps_func_request_async_complete;
	msg_p->event = event;
	msg_p->context = msg_p;
	msg_p->status = -1;

	spin_lock_irqsave( &(ts->queue_lock), flags);
	list_add_tail( &(msg_p->queue), &(ts->queue) );
	spin_unlock_irqrestore( &(ts->queue_lock), flags);
	queue_work(ts->workqueue_p, &(ts->work_data) );
}

static void shtps_func_request_sync_complete(void *arg_p)
{
	complete( arg_p );
}

int shtps_func_request_sync( struct shtps_rmi_spi *ts, int event)
{
	DECLARE_COMPLETION_ONSTACK(done);
	struct shtps_req_msg msg;
	unsigned long	flags;

	msg.complete = shtps_func_request_sync_complete;
	msg.event = event;
	msg.context = &done;
	msg.status = -1;

	spin_lock_irqsave( &(ts->queue_lock), flags);
	list_add_tail( &(msg.queue), &(ts->queue) );
	spin_unlock_irqrestore( &(ts->queue_lock), flags);
	queue_work(ts->workqueue_p, &(ts->work_data) );

	wait_for_completion(&done);

	return msg.status;
}

static void shtps_overlap_event_check(struct shtps_rmi_spi *ts)
{
	unsigned long flags;
	struct list_head *list_p;
	int overlap_req_cnt_lcd = -1;
	int overlap_req_cnt_grip = -1;

	spin_lock_irqsave( &(ts->queue_lock), flags );
	list_for_each(list_p, &(ts->queue)){
		ts->cur_msg_p = list_entry(list_p, struct shtps_req_msg, queue);
		spin_unlock_irqrestore( &(ts->queue_lock), flags );

		if( (ts->cur_msg_p->event == SHTPS_FUNC_REQ_EVEMT_LCD_ON) ||
			(ts->cur_msg_p->event == SHTPS_FUNC_REQ_EVEMT_LCD_OFF) )
		{
			overlap_req_cnt_lcd++;
		}

		if( (ts->cur_msg_p->event == SHTPS_FUNC_REQ_EVEMT_GRIP_ON) ||
			(ts->cur_msg_p->event == SHTPS_FUNC_REQ_EVEMT_GRIP_OFF) )
		{
			overlap_req_cnt_grip++;
		}

		spin_lock_irqsave( &(ts->queue_lock), flags );
	}
	spin_unlock_irqrestore( &(ts->queue_lock), flags );

	if( (overlap_req_cnt_lcd > 0) || (overlap_req_cnt_grip > 0) ){
		spin_lock_irqsave( &(ts->queue_lock), flags );
		list_for_each(list_p, &(ts->queue)){
			ts->cur_msg_p = list_entry(list_p, struct shtps_req_msg, queue);
			spin_unlock_irqrestore( &(ts->queue_lock), flags );

			if(ts->cur_msg_p != NULL){
				if( (ts->cur_msg_p->event == SHTPS_FUNC_REQ_EVEMT_LCD_ON) ||
					(ts->cur_msg_p->event == SHTPS_FUNC_REQ_EVEMT_LCD_OFF) )
				{
					if(overlap_req_cnt_lcd > 0){
						spin_lock_irqsave( &(ts->queue_lock), flags );
						list_p = list_p->prev;
						list_del_init( &(ts->cur_msg_p->queue) );
						spin_unlock_irqrestore( &(ts->queue_lock), flags );
						ts->cur_msg_p->status = 0;
						if( ts->cur_msg_p->complete ){
							ts->cur_msg_p->complete( ts->cur_msg_p->context );
						}
						overlap_req_cnt_lcd--;
						SHTPS_LOG_DBG_PRINT("FuncReq[%d] overlap canceled\n", ts->cur_msg_p->event);
					}
				}

				if( (ts->cur_msg_p->event == SHTPS_FUNC_REQ_EVEMT_GRIP_ON) ||
					(ts->cur_msg_p->event == SHTPS_FUNC_REQ_EVEMT_GRIP_OFF) )
				{
					if(overlap_req_cnt_grip > 0){
						spin_lock_irqsave( &(ts->queue_lock), flags );
						list_p = list_p->prev;
						list_del_init( &(ts->cur_msg_p->queue) );
						spin_unlock_irqrestore( &(ts->queue_lock), flags );
						ts->cur_msg_p->status = 0;
						if( ts->cur_msg_p->complete ){
							ts->cur_msg_p->complete( ts->cur_msg_p->context );
						}
						overlap_req_cnt_grip--;
						SHTPS_LOG_DBG_PRINT("FuncReq[%d] overlap canceled\n", ts->cur_msg_p->event);
					}
				}
			}else{
				SHTPS_LOG_DBG_PRINT("%s() overlap event delete check msg NULL\n", __func__);
			}

			spin_lock_irqsave( &(ts->queue_lock), flags );
		}

		spin_unlock_irqrestore( &(ts->queue_lock), flags );
	}
}

static void shtps_func_workq( struct work_struct *work_p )
{
	struct shtps_rmi_spi	*ts;
	unsigned long			flags;

	ts = container_of(work_p, struct shtps_rmi_spi, work_data);

	spin_lock_irqsave( &(ts->queue_lock), flags );
	while( list_empty( &(ts->queue) ) == 0 ){
		ts->cur_msg_p = list_entry( ts->queue.next, struct shtps_req_msg, queue);
		list_del_init( &(ts->cur_msg_p->queue) );
		spin_unlock_irqrestore( &(ts->queue_lock), flags );

		SHTPS_LOG_DBG_PRINT("FuncReq[%d] start\n", ts->cur_msg_p->event);

		switch(ts->cur_msg_p->event){
			case SHTPS_FUNC_REQ_EVEMT_OPEN:
				#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
					if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_OPEN, 0) == 0){
						shtps_func_open(ts);
					}
				#else
					shtps_func_open(ts);
				#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */
				ts->cur_msg_p->status = 0;
				break;

			case SHTPS_FUNC_REQ_EVEMT_CLOSE:
				#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
					if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_CLOSE, 0) == 0){
						shtps_func_close(ts);
					}
				#else
					shtps_func_close(ts);
				#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */
				ts->cur_msg_p->status = 0;
				break;

			case SHTPS_FUNC_REQ_EVEMT_ENABLE:
				#if defined(SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE)
					if(shtps_check_suspend_state(ts, SHTPS_DETER_SUSPEND_SPI_PROC_ENABLE, 0) == 0){
						ts->cur_msg_p->status = shtps_func_enable(ts);
					}else{
						ts->cur_msg_p->status = 0;
					}
				#else
					ts->cur_msg_p->status = shtps_func_enable(ts);
				#endif /* SHTPS_GUARANTEE_SPI_ACCESS_IN_WAKE_ENABLE */
				break;

			case SHTPS_FUNC_REQ_EVEMT_DISABLE:
				shtps_func_disable(ts);
				ts->cur_msg_p->status = 0;
				break;

			#if defined( SHTPS_CHECK_CRC_ERROR_ENABLE )
				case SHTPS_FUNC_REQ_EVEMT_CHECK_CRC_ERROR:
					shtps_func_check_crc_error(ts);
					ts->cur_msg_p->status = 0;
					break;
			#endif /* #if defined( SHTPS_CHECK_CRC_ERROR_ENABLE ) */

			case SHTPS_FUNC_REQ_EVEMT_LCD_ON:
				shtps_func_lcd_on(ts);
				ts->cur_msg_p->status = 0;
				break;

			case SHTPS_FUNC_REQ_EVEMT_LCD_OFF:
				shtps_func_lcd_off(ts);
				ts->cur_msg_p->status = 0;
				break;

			#if defined(SHTPS_DEF_LPWG_GRIP_PROC_ASYNC_ENABLE)
				case SHTPS_FUNC_REQ_EVEMT_GRIP_ON:
					shtps_lpwg_grip_wakelock(ts, 1);
					shtps_func_grip_on(ts);
					shtps_lpwg_grip_wakelock(ts, 0);
					ts->cur_msg_p->status = 0;
					break;

				case SHTPS_FUNC_REQ_EVEMT_GRIP_OFF:
					shtps_lpwg_grip_wakelock(ts, 1);
					shtps_func_grip_off(ts);
					shtps_lpwg_grip_wakelock(ts, 0);
					ts->cur_msg_p->status = 0;
					break;
			#endif /* SHTPS_DEF_LPWG_GRIP_PROC_ASYNC_ENABLE */

			#if defined(SHTPS_PROXIMITY_SUPPORT_ENABLE)
				case SHTPS_FUNC_REQ_EVEMT_PROXIMITY_CHECK:
					shtps_func_proximity_check(ts);
					ts->cur_msg_p->status = 0;
					break;
			#endif /* SHTPS_PROXIMITY_SUPPORT_ENABLE */

			default:
				ts->cur_msg_p->status = -1;
				break;
		}

		SHTPS_LOG_DBG_PRINT("FuncReq[%d] end\n", ts->cur_msg_p->event);

		if( ts->cur_msg_p->complete ){
			ts->cur_msg_p->complete( ts->cur_msg_p->context );
		}

		shtps_overlap_event_check(ts);

		spin_lock_irqsave( &(ts->queue_lock), flags );
	}
	spin_unlock_irqrestore( &(ts->queue_lock), flags );
}

int shtps_func_async_init( struct shtps_rmi_spi *ts)
{
	ts->workqueue_p = alloc_workqueue("TPS_WORK", WQ_UNBOUND, 1);
	if(ts->workqueue_p == NULL){
		return -ENOMEM;
	}
	INIT_WORK( &(ts->work_data), shtps_func_workq );
	INIT_LIST_HEAD( &(ts->queue) );
	spin_lock_init( &(ts->queue_lock) );

	return 0;
}

void shtps_func_async_deinit( struct shtps_rmi_spi *ts)
{
	if(ts->workqueue_p){
		destroy_workqueue(ts->workqueue_p);
	}
}
 
/* -----------------------------------------------------------------------------------
 */

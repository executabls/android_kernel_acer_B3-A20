#ifndef _MT_SPM_IDLE_
#define _MT_SPM_IDLE_

#include <linux/kernel.h>
#include <linux/xlog.h>
#include <mach/mt_spm.h>
#include <mach/mt_spm_sleep.h>

#define spm_idle_ver(fmt, args...)		pr_info("[SPM-Idle] " fmt, ##args)

/*
 * for Deep Idle
 */
void spm_dpidle_before_wfi(void);		 /* can be redefined */
void spm_dpidle_after_wfi(void);		 /* can be redefined */
wake_reason_t spm_go_to_dpidle(u32 spm_flags, u32 spm_data);
wake_reason_t spm_go_to_sleep_dpidle(u32 spm_flags, u32 spm_data);
int spm_set_dpidle_wakesrc(u32 wakesrc, bool enable, bool replace);
bool spm_set_dpidle_pcm_init_flag(void);


/*
 * for Screen On Deep Idle
 */
void spm_go_to_sodi(u32 spm_flags, u32 spm_data);
void spm_sodi_lcm_video_mode(bool IsLcmVideoMode);
void spm_sodi_mempll_pwr_mode(bool pwr_mode);
void spm_enable_sodi(bool);
bool spm_get_sodi_en(void);
void spm_sodi_cpu_dvs_en(bool en);

/*
 * for Multi Core Deep Idle
 */
 enum spm_mcdi_lock_id{
    SPM_MCDI_IDLE= 0,
    SPM_MCDI_VCORE_DVFS= 1,
    SPM_MCDI_EARLY_SUSPEND= 2,
};
int spm_mcdi_init(void);
void spm_mcdi_wakeup_all_cores(void);
void spm_mcdi_switch_on_off(enum spm_mcdi_lock_id id, int mcdi_en);
bool spm_mcdi_wfi(int core_id);
bool spm_mcdi_can_enter(void);
bool spm_is_cpu_irq_occur(int core_id);

#endif

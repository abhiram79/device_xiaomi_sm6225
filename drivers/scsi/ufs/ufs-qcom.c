// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2013-2021, Linux Foundation. All rights reserved.
 * Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/acpi.h>
#include <linux/time.h>
#include <linux/of.h>
#include <linux/bitfield.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/gpio/consumer.h>
#include <linux/reset-controller.h>
#include <linux/interconnect.h>
#include <linux/phy/phy-qcom-ufs.h>
#include <linux/clk/qcom.h>
#include <linux/devfreq.h>
#include <linux/cpu.h>
#include <linux/blk-mq.h>
#include <linux/blk_types.h>
#include <linux/thermal.h>
#include <linux/cpufreq.h>
#include <linux/debugfs.h>
#include <trace/hooks/ufshcd.h>
#include <linux/ipc_logging.h>
#include <soc/qcom/minidump.h>
#include <linux/nvmem-consumer.h>


#define CREATE_TRACE_POINTS
#include "ufs-qcom-trace.h"
#include "ufshcd.h"
#include "ufshcd-pltfrm.h"
#include "unipro.h"
#include "ufs-qcom.h"
#include "ufshci.h"
#include "ufs_quirks.h"
#include "ufshcd-crypto-qti.h"
#if IS_ENABLED(CONFIG_QTI_CRYPTO_FDE)
#include <linux/crypto-qti-common.h>
#endif

#define UFS_QCOM_DEFAULT_DBG_PRINT_EN	\
	(UFS_QCOM_DBG_PRINT_REGS_EN | UFS_QCOM_DBG_PRINT_TEST_BUS_EN)

#define UFS_DDR "ufs-ddr"
#define CPU_UFS "cpu-ufs"
#define MAX_PROP_SIZE		   32
#define VDDP_REF_CLK_MIN_UV        1200000
#define VDDP_REF_CLK_MAX_UV        1200000
#define VCCQ_DEFAULT_1_2V	1200000

#define UFS_QCOM_LOAD_MON_DLY_MS 30

#define	ANDROID_BOOT_DEV_MAX	30
#define	UFS_BOOT_DEVICE		0x4

#define	UFS_QCOM_IRQ_PRIME_MASK	0x80
#define	UFS_QCOM_IRQ_SLVR_MASK	0x0f

#define UFS_QCOM_BER_TH_DEF_G1_G4	0
#define UFS_QCOM_BER_TH_DEF_G5	3
/*
 * Default time window of PHY BER monitor in millisecond.
 * Can be overridden by MODULE CmdLine and MODULE sysfs node.
 */
#define UFS_QCOM_BER_DUR_DEF_MS	(60*60*1000)

/* Max number of log pages */
#define UFS_QCOM_MAX_LOG_SZ	10
#define ufs_qcom_log_str(host, fmt, ...)	\
	do {	\
		if (host->ufs_ipc_log_ctx && host->dbg_en)	\
			ipc_log_string(host->ufs_ipc_log_ctx,	\
					",%d,"fmt, current->cpu, ##__VA_ARGS__);\
	} while (0)

#define ufs_qcom_msg(level, dev, fmt, ...) \
	do { \
		if (!IS_ERR_OR_NULL(dev)) { \
			switch (level) { \
			case ERR: \
				dev_err(dev, "ERROR: "fmt, ##__VA_ARGS__); break; \
			case WARN: \
				dev_warn(dev, "WARN: "fmt, ##__VA_ARGS__); break; \
			case DBG: \
				dev_dbg(dev, "DEBUG: "fmt, ##__VA_ARGS__); break; \
			default: \
				dev_info(dev, "INFO: "fmt, ##__VA_ARGS__); \
			} \
		} \
		else { \
			switch (level) { \
			case ERR: \
				pr_err("ERROR: "fmt, ##__VA_ARGS__); break; \
			case WARN: \
				pr_warn("WARN: "fmt, ##__VA_ARGS__); break; \
			case DBG: \
				pr_debug("DEBUG: "fmt, ##__VA_ARGS__); break; \
			default: \
				pr_info("INFO: "fmt, ##__VA_ARGS__); \
			} \
		} \
	} while (0) \

enum {
	UFS_QCOM_SYSFS_NONE,
	UFS_QCOM_SYSFS_S2R,
	UFS_QCOM_SYSFS_DEEPSLEEP,
};

static char android_boot_dev[ANDROID_BOOT_DEV_MAX];

static DEFINE_PER_CPU(struct freq_qos_request, qos_min_req);

enum {
	TSTBUS_UAWM,
	TSTBUS_UARM,
	TSTBUS_TXUC,
	TSTBUS_RXUC,
	TSTBUS_DFC,
	TSTBUS_TRLUT,
	TSTBUS_TMRLUT,
	TSTBUS_OCSC,
	TSTBUS_UTP_HCI,
	TSTBUS_COMBINED,
	TSTBUS_WRAPPER,
	TSTBUS_UNIPRO,
	TSTBUS_MAX,
};

enum ufs_msg_level {
	ERR = 3,
	WARN = 4,
	INFO = 6,
	DBG = 7,
};

enum {
	UFS_QCOM_CMD_SEND,
	UFS_QCOM_CMD_COMPL,
};

struct ufs_qcom_dev_params {
	u32 pwm_rx_gear;	/* pwm rx gear to work in */
	u32 pwm_tx_gear;	/* pwm tx gear to work in */
	u32 hs_rx_gear;		/* hs rx gear to work in */
	u32 hs_tx_gear;		/* hs tx gear to work in */
	u32 rx_lanes;		/* number of rx lanes */
	u32 tx_lanes;		/* number of tx lanes */
	u32 rx_pwr_pwm;		/* rx pwm working pwr */
	u32 tx_pwr_pwm;		/* tx pwm working pwr */
	u32 rx_pwr_hs;		/* rx hs working pwr */
	u32 tx_pwr_hs;		/* tx hs working pwr */
	u32 hs_rate;		/* rate A/B to work in HS */
	u32 desired_working_mode;
};

static struct ufs_qcom_host *ufs_qcom_hosts[MAX_UFS_QCOM_HOSTS];

static void ufs_qcom_get_default_testbus_cfg(struct ufs_qcom_host *host);
static int ufs_qcom_set_dme_vs_core_clk_ctrl_clear_div(struct ufs_hba *hba,
						       u32 clk_1us_cycles,
						       u32 clk_40ns_cycles);
static void ufs_qcom_parse_limits(struct ufs_qcom_host *host);
static void ufs_qcom_parse_lpm(struct ufs_qcom_host *host);
static void ufs_qcom_parse_wb(struct ufs_qcom_host *host);
static int ufs_qcom_set_dme_vs_core_clk_ctrl_max_freq_mode(struct ufs_hba *hba);
static int ufs_qcom_init_sysfs(struct ufs_hba *hba);
static int ufs_qcom_update_qos_constraints(struct qos_cpu_group *qcg,
					   enum constraint type);
static int ufs_qcom_unvote_qos_all(struct ufs_hba *hba);
static void ufs_qcom_parse_g4_workaround_flag(struct ufs_qcom_host *host);
static int ufs_qcom_mod_min_cpufreq(unsigned int cpu, s32 new_val);
static int ufs_qcom_config_shared_ice(struct ufs_qcom_host *host);
static int ufs_qcom_ber_threshold_set(const char *val, const struct kernel_param *kp);
static int ufs_qcom_ber_duration_set(const char *val, const struct kernel_param *kp);
static void ufs_qcom_ber_mon_init(struct ufs_hba *hba);
static void ufs_qcom_populate_available_cpus(struct ufs_hba *hba);

static s64 idle_time[UFS_QCOM_BER_MODE_MAX];
static ktime_t idle_start;
static bool crash_on_ber;
static bool override_ber_threshold;
static bool override_ber_duration;
static unsigned int ber_threshold = UFS_QCOM_BER_TH_DEF_G1_G4;
static unsigned int ber_mon_dur_ms = UFS_QCOM_BER_DUR_DEF_MS;

static struct ufs_qcom_ber_table ber_table[] = {
	[UFS_HS_DONT_CHANGE] = {UFS_QCOM_BER_MODE_G1_G4, UFS_QCOM_BER_TH_DEF_G1_G4},
	[UFS_HS_G1] = {UFS_QCOM_BER_MODE_G1_G4, UFS_QCOM_BER_TH_DEF_G1_G4},
	[UFS_HS_G2] = {UFS_QCOM_BER_MODE_G1_G4, UFS_QCOM_BER_TH_DEF_G1_G4},
	[UFS_HS_G3] = {UFS_QCOM_BER_MODE_G1_G4, UFS_QCOM_BER_TH_DEF_G1_G4},
	[UFS_HS_G4] = {UFS_QCOM_BER_MODE_G1_G4, UFS_QCOM_BER_TH_DEF_G1_G4},
	[UFS_HS_G5] = {UFS_QCOM_BER_MODE_G5, UFS_QCOM_BER_TH_DEF_G5},
};

module_param(crash_on_ber, bool, 0644);
MODULE_PARM_DESC(crash_on_ber, "Crash if PHY BER exceeds threshold. the default value is false");

static const struct kernel_param_ops ber_threshold_ops = {
	.set = ufs_qcom_ber_threshold_set,
	.get = param_get_uint,
};

module_param_cb(ber_threshold, &ber_threshold_ops, &ber_threshold, 0644);
MODULE_PARM_DESC(ber_threshold, "Set UFS BER threshold, should be less than 16.\n"
								"The default value is 3 for G5 and 0 for Non G5.");

static int ufs_qcom_ber_threshold_set(const char *val, const struct kernel_param *kp)
{
	unsigned int n;
	int ret;

	ret = kstrtou32(val, 0, &n);
	if (ret != 0 || n > (UFS_QCOM_EVT_LEN - 1))
		return -EINVAL;
	if (n)
		override_ber_threshold = true;
	else
		override_ber_threshold = false;

	return param_set_uint(val, kp);
}

static const struct kernel_param_ops ber_duration_ops = {
	.set = ufs_qcom_ber_duration_set,
	.get = param_get_uint,
};

module_param_cb(ber_duration_ms, &ber_duration_ops, &ber_mon_dur_ms, 0644);
MODULE_PARM_DESC(ber_duration_ms, "Set the duration of BER monitor window.\n"
								"The default value is 3600000(1 hour)");

static int ufs_qcom_ber_duration_set(const char *val, const struct kernel_param *kp)
{
	s64 n;

	if (kstrtos64(val, 0, &n))
		return -EINVAL;

	if (n)
		override_ber_duration = true;
	else
		override_ber_duration = false;

	return param_set_uint(val, kp);
}

/**
 * ufs_qcom_save_regs - read register value and save to memory for specified domain.
 * If it is a new domain which never been saved, allocate memory for it and add to list.
 * @host - ufs_qcom_host
 * @offset - register address offest
 * @len - length or size
 * @prefix - domain name
 */

static int ufs_qcom_save_regs(struct ufs_qcom_host *host, size_t offset, size_t len,
		     const char *prefix)
{
	struct ufs_qcom_regs *regs = NULL;
	struct list_head *head = &host->regs_list_head;
	unsigned int noio_flag;
	size_t pos;

	if (offset % 4 != 0 || len % 4 != 0)
		return -EINVAL;

	/* find the node if this register domain has been saved before */
	list_for_each_entry(regs, head, list)
		if (regs->prefix && !strcmp(regs->prefix, prefix))
			break;

	/* create a new node and add it to list if this domain never been written */
	if (&regs->list == head) {
		/* use memalloc_noio_save() here as GFP_ATOMIC should not be invoked
		 * in an IO error context
		 */
		noio_flag = memalloc_noio_save();
		regs = devm_kzalloc(host->hba->dev, sizeof(*regs), GFP_ATOMIC);
		if (!regs)
			goto out;
		regs->ptr = devm_kzalloc(host->hba->dev, len, GFP_ATOMIC);
		if (!regs->ptr)
			goto out;
		memalloc_noio_restore(noio_flag);
		regs->prefix = prefix;
		regs->len = len;
		list_add_tail(&regs->list, &host->regs_list_head);
	}

	for (pos = 0; pos < len; pos += 4) {
		if (offset == 0 &&
		    pos >= REG_UIC_ERROR_CODE_PHY_ADAPTER_LAYER &&
		    pos <= REG_UIC_ERROR_CODE_DME)
			continue;
		regs->ptr[pos / 4] = ufshcd_readl(host->hba, offset + pos);
	}
	return 0;

out:
	memalloc_noio_restore(noio_flag);
	return -ENOMEM;
}

static int ufs_qcom_save_testbus(struct ufs_qcom_host *host)
{
	struct ufs_qcom_regs *regs = NULL;
	struct list_head *head = &host->regs_list_head;
	int i, j;
	int nminor = 32, testbus_len = nminor * sizeof(u32);
	unsigned int noio_flag;
	char *prefix;

	for (j = 0; j < TSTBUS_MAX; j++) {
		switch (j) {
		case TSTBUS_UAWM:
			prefix = "TSTBUS_UAWM ";
			break;
		case TSTBUS_UARM:
			prefix = "TSTBUS_UARM ";
			break;
		case TSTBUS_TXUC:
			prefix = "TSTBUS_TXUC ";
			break;
		case TSTBUS_RXUC:
			prefix = "TSTBUS_RXUC ";
			break;
		case TSTBUS_DFC:
			prefix = "TSTBUS_DFC ";
			break;
		case TSTBUS_TRLUT:
			prefix = "TSTBUS_TRLUT ";
			break;
		case TSTBUS_TMRLUT:
			prefix = "TSTBUS_TMRLUT ";
			break;
		case TSTBUS_OCSC:
			prefix = "TSTBUS_OCSC ";
			break;
		case TSTBUS_UTP_HCI:
			prefix = "TSTBUS_UTP_HCI ";
			break;
		case TSTBUS_COMBINED:
			prefix = "TSTBUS_COMBINED ";
			break;
		case TSTBUS_WRAPPER:
			prefix = "TSTBUS_WRAPPER ";
			break;
		case TSTBUS_UNIPRO:
			prefix = "TSTBUS_UNIPRO ";
			break;
		default:
			prefix = "UNKNOWN ";
			break;
		}

		/* find the node if this register domain has been saved before */
		list_for_each_entry(regs, head, list)
			if (regs->prefix && !strcmp(regs->prefix, prefix))
				break;

		/* create a new node and add it to list if this domain never been written */
		if (&regs->list == head) {
			noio_flag = memalloc_noio_save();
			regs = devm_kzalloc(host->hba->dev, sizeof(*regs), GFP_ATOMIC);
			if (!regs)
				goto out;
			regs->ptr = devm_kzalloc(host->hba->dev, testbus_len, GFP_ATOMIC);
			if (!regs->ptr)
				goto out;
			memalloc_noio_restore(noio_flag);
			regs->prefix = prefix;
			regs->len = testbus_len;
			list_add_tail(&regs->list, &host->regs_list_head);
		}

		host->testbus.select_major = j;
		for (i = 0; i < nminor; i++) {
			host->testbus.select_minor = i;
			ufs_qcom_testbus_config(host);
			regs->ptr[i] = ufshcd_readl(host->hba, UFS_TEST_BUS);
		}
	}
	return 0;

out:
	memalloc_noio_restore(noio_flag);
	return -ENOMEM;
}

static inline void cancel_dwork_unvote_cpufreq(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	int err;

	if (host->cpufreq_dis)
		return;

	cancel_delayed_work_sync(&host->fwork);
	if (!host->cur_freq_vote)
		return;
	atomic_set(&host->num_reqs_threshold, 0);

	err = ufs_qcom_mod_min_cpufreq(host->config_cpu,
				       host->min_cpu_scale_freq);
	if (err < 0)
		ufs_qcom_msg(ERR, hba->dev, "fail set cpufreq-fmin_def %d:\n",
				err);
	else
		host->cur_freq_vote = false;
	ufs_qcom_msg(DBG, hba->dev, "%s,err=%d\n", __func__, err);
}

static int ufs_qcom_get_pwr_dev_param(struct ufs_qcom_dev_params *qcom_param,
				      struct ufs_pa_layer_attr *dev_max,
				      struct ufs_pa_layer_attr *agreed_pwr)
{
	int min_qcom_gear;
	int min_dev_gear;
	bool is_dev_sup_hs = false;
	bool is_qcom_max_hs = false;

	if (dev_max->pwr_rx == FAST_MODE)
		is_dev_sup_hs = true;

	if (qcom_param->desired_working_mode == FAST) {
		is_qcom_max_hs = true;
		min_qcom_gear = min_t(u32, qcom_param->hs_rx_gear,
				      qcom_param->hs_tx_gear);
	} else {
		min_qcom_gear = min_t(u32, qcom_param->pwm_rx_gear,
				      qcom_param->pwm_tx_gear);
	}

	/*
	 * device doesn't support HS but qcom_param->desired_working_mode is
	 * HS, thus device and qcom_param don't agree
	 */
	if (!is_dev_sup_hs && is_qcom_max_hs) {
		ufs_qcom_msg(ERR, NULL,
			"%s: failed to agree on power mode (device doesn't support HS but requested power is HS)\n",
			__func__);
		return -EOPNOTSUPP;
	} else if (is_dev_sup_hs && is_qcom_max_hs) {
		/*
		 * since device supports HS, it supports FAST_MODE.
		 * since qcom_param->desired_working_mode is also HS
		 * then final decision (FAST/FASTAUTO) is done according
		 * to qcom_params as it is the restricting factor
		 */
		agreed_pwr->pwr_rx = agreed_pwr->pwr_tx =
						qcom_param->rx_pwr_hs;
	} else {
		/*
		 * here qcom_param->desired_working_mode is PWM.
		 * it doesn't matter whether device supports HS or PWM,
		 * in both cases qcom_param->desired_working_mode will
		 * determine the mode
		 */
		agreed_pwr->pwr_rx = agreed_pwr->pwr_tx =
			qcom_param->rx_pwr_pwm;
	}

	/*
	 * we would like tx to work in the minimum number of lanes
	 * between device capability and vendor preferences.
	 * the same decision will be made for rx
	 */
	agreed_pwr->lane_tx = min_t(u32, dev_max->lane_tx,
						qcom_param->tx_lanes);
	agreed_pwr->lane_rx = min_t(u32, dev_max->lane_rx,
						qcom_param->rx_lanes);

	/* device maximum gear is the minimum between device rx and tx gears */
	min_dev_gear = min_t(u32, dev_max->gear_rx, dev_max->gear_tx);

	/*
	 * if both device capabilities and vendor pre-defined preferences are
	 * both HS or both PWM then set the minimum gear to be the chosen
	 * working gear.
	 * if one is PWM and one is HS then the one that is PWM get to decide
	 * what is the gear, as it is the one that also decided previously what
	 * pwr the device will be configured to.
	 */
	if ((is_dev_sup_hs && is_qcom_max_hs) ||
	    (!is_dev_sup_hs && !is_qcom_max_hs))
		agreed_pwr->gear_rx = agreed_pwr->gear_tx =
			min_t(u32, min_dev_gear, min_qcom_gear);
	else if (!is_dev_sup_hs)
		agreed_pwr->gear_rx = agreed_pwr->gear_tx = min_dev_gear;
	else
		agreed_pwr->gear_rx = agreed_pwr->gear_tx = min_qcom_gear;

	agreed_pwr->hs_rate = qcom_param->hs_rate;
	return 0;
}

static struct ufs_qcom_host *rcdev_to_ufs_host(struct reset_controller_dev *rcd)
{
	return container_of(rcd, struct ufs_qcom_host, rcdev);
}

static void ufs_qcom_dump_regs_wrapper(struct ufs_hba *hba, int offset, int len,
				       const char *prefix, void *priv)
{
	ufshcd_dump_regs(hba, offset, len * 4, prefix);
}

static int ufs_qcom_get_connected_tx_lanes(struct ufs_hba *hba, u32 *tx_lanes)
{
	int err = 0;

	err = ufshcd_dme_get(hba,
			UIC_ARG_MIB(PA_CONNECTEDTXDATALANES), tx_lanes);
	if (err)
		ufs_qcom_msg(ERR, hba->dev, "%s: couldn't read PA_CONNECTEDTXDATALANES %d\n",
				__func__, err);

	return err;
}

static int ufs_qcom_get_connected_rx_lanes(struct ufs_hba *hba, u32 *rx_lanes)
{
	int err = 0;

	err = ufshcd_dme_get(hba,
			UIC_ARG_MIB(PA_CONNECTEDRXDATALANES), rx_lanes);
	if (err)
		ufs_qcom_msg(ERR, hba->dev, "%s: couldn't read PA_CONNECTEDRXDATALANES %d\n",
				__func__, err);

	return err;
}

static int ufs_qcom_host_clk_get(struct device *dev,
		const char *name, struct clk **clk_out, bool optional)
{
	struct clk *clk;
	int err = 0;

	clk = devm_clk_get(dev, name);
	if (!IS_ERR(clk)) {
		*clk_out = clk;
		return 0;
	}

	err = PTR_ERR(clk);

	if (optional && err == -ENOENT) {
		*clk_out = NULL;
		return 0;
	}

	if (err != -EPROBE_DEFER)
		ufs_qcom_msg(ERR, dev, "failed to get %s err %d\n", name, err);

	return err;
}

static int ufs_qcom_host_clk_enable(struct device *dev,
		const char *name, struct clk *clk)
{
	int err = 0;

	err = clk_prepare_enable(clk);
	if (err)
		ufs_qcom_msg(ERR, dev, "%s: %s enable failed %d\n", __func__, name, err);

	return err;
}

static void ufs_qcom_disable_lane_clks(struct ufs_qcom_host *host)
{
	if (!host->is_lane_clks_enabled)
		return;

	if (host->tx_l1_sync_clk)
		clk_disable_unprepare(host->tx_l1_sync_clk);
	clk_disable_unprepare(host->tx_l0_sync_clk);
	if (host->rx_l1_sync_clk)
		clk_disable_unprepare(host->rx_l1_sync_clk);
	clk_disable_unprepare(host->rx_l0_sync_clk);

	host->is_lane_clks_enabled = false;
}

static int ufs_qcom_enable_lane_clks(struct ufs_qcom_host *host)
{
	int err = 0;
	struct device *dev = host->hba->dev;

	if (host->is_lane_clks_enabled)
		return 0;

	err = ufs_qcom_host_clk_enable(dev, "rx_lane0_sync_clk",
		host->rx_l0_sync_clk);
	if (err)
		goto out;

	err = ufs_qcom_host_clk_enable(dev, "tx_lane0_sync_clk",
		host->tx_l0_sync_clk);
	if (err)
		goto disable_rx_l0;

	if (host->hba->lanes_per_direction > 1) {
		err = ufs_qcom_host_clk_enable(dev, "rx_lane1_sync_clk",
			host->rx_l1_sync_clk);
		if (err)
			goto disable_tx_l0;

		/* The tx lane1 clk could be muxed, hence keep this optional */
		if (host->tx_l1_sync_clk) {
			err = ufs_qcom_host_clk_enable(dev, "tx_lane1_sync_clk",
					host->tx_l1_sync_clk);
			if (err)
				goto disable_rx_l1;
		}
	}

	host->is_lane_clks_enabled = true;
	goto out;

disable_rx_l1:
	clk_disable_unprepare(host->rx_l1_sync_clk);
disable_tx_l0:
	clk_disable_unprepare(host->tx_l0_sync_clk);
disable_rx_l0:
	clk_disable_unprepare(host->rx_l0_sync_clk);
out:
	return err;
}

static int ufs_qcom_init_lane_clks(struct ufs_qcom_host *host)
{
	int err = 0;
	struct device *dev = host->hba->dev;

	if (has_acpi_companion(dev))
		return 0;

	err = ufs_qcom_host_clk_get(dev, "rx_lane0_sync_clk",
					&host->rx_l0_sync_clk, false);
	if (err) {
		ufs_qcom_msg(ERR, dev, "%s: failed to get rx_lane0_sync_clk, err %d\n",
				__func__, err);
		goto out;
	}

	err = ufs_qcom_host_clk_get(dev, "tx_lane0_sync_clk",
					&host->tx_l0_sync_clk, false);
	if (err) {
		ufs_qcom_msg(ERR, dev, "%s: failed to get tx_lane0_sync_clk, err %d\n",
				__func__, err);
		goto out;
	}

	/* In case of single lane per direction, don't read lane1 clocks */
	if (host->hba->lanes_per_direction > 1) {
		err = ufs_qcom_host_clk_get(dev, "rx_lane1_sync_clk",
			&host->rx_l1_sync_clk, false);
		if (err) {
			ufs_qcom_msg(ERR, dev, "%s: failed to get rx_lane1_sync_clk, err %d\n",
					__func__, err);
			goto out;
		}

		err = ufs_qcom_host_clk_get(dev, "tx_lane1_sync_clk",
			&host->tx_l1_sync_clk, true);
	}
out:
	return err;
}

static int ufs_qcom_link_startup_post_change(struct ufs_hba *hba)
{
	u32 tx_lanes;
	int err = 0;
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	struct phy *phy = host->generic_phy;

	err = ufs_qcom_get_connected_tx_lanes(hba, &tx_lanes);
	if (err)
		goto out;

	ufs_qcom_phy_set_tx_lane_enable(phy, tx_lanes);
	/*
	 * Some UFS devices send incorrect LineCfg data as part of power mode
	 * change sequence which may cause host PHY to go into bad state.
	 * Disabling Rx LineCfg of host PHY should help avoid this.
	 */
	if (ufshcd_get_local_unipro_ver(hba) == UFS_UNIPRO_VER_1_41)
		ufs_qcom_phy_ctrl_rx_linecfg(phy, false);

	/*
	 * UFS controller has *clk_req output to GCC, for each of the clocks
	 * entering it. When *clk_req for a specific clock is de-asserted,
	 * a corresponding clock from GCC is stopped. UFS controller de-asserts
	 * *clk_req outputs when it is in Auto Hibernate state only if the
	 * Clock request feature is enabled.
	 * Enable the Clock request feature:
	 * - Enable HW clock control for UFS clocks in GCC (handled by the
	 *   clock driver as part of clk_prepare_enable).
	 * - Set the AH8_CFG.*CLK_REQ register bits to 1.
	 */
	if (ufshcd_is_auto_hibern8_supported(hba))
		ufshcd_writel(hba, ufshcd_readl(hba, UFS_AH8_CFG) |
				   UFS_HW_CLK_CTRL_EN,
				   UFS_AH8_CFG);
	/*
	 * Make sure clock request feature gets enabled for HW clk gating
	 * before further operations.
	 */
	mb();

out:
	return err;
}

static int ufs_qcom_check_hibern8(struct ufs_hba *hba)
{
	int err;
	u32 tx_fsm_val = 0;
	unsigned long timeout = jiffies + msecs_to_jiffies(HBRN8_POLL_TOUT_MS);

	do {
		err = ufshcd_dme_get(hba,
				UIC_ARG_MIB_SEL(MPHY_TX_FSM_STATE,
					UIC_ARG_MPHY_TX_GEN_SEL_INDEX(0)),
				&tx_fsm_val);
		if (err || tx_fsm_val == TX_FSM_HIBERN8)
			break;

		/* sleep for max. 200us */
		usleep_range(100, 200);
	} while (time_before(jiffies, timeout));

	/*
	 * we might have scheduled out for long during polling so
	 * check the state again.
	 */
	if (time_after(jiffies, timeout))
		err = ufshcd_dme_get(hba,
				UIC_ARG_MIB_SEL(MPHY_TX_FSM_STATE,
					UIC_ARG_MPHY_TX_GEN_SEL_INDEX(0)),
				&tx_fsm_val);

	if (err) {
		ufs_qcom_msg(ERR, hba->dev, "%s: unable to get TX_FSM_STATE, err %d\n",
				__func__, err);
	} else if (tx_fsm_val != TX_FSM_HIBERN8) {
		err = tx_fsm_val;
		ufs_qcom_msg(ERR, hba->dev, "%s: invalid TX_FSM_STATE = %d\n",
				__func__, err);
	}

	return err;
}

static void ufs_qcom_select_unipro_mode(struct ufs_qcom_host *host)
{
	ufshcd_rmwl(host->hba, QUNIPRO_SEL,
		   ufs_qcom_cap_qunipro(host) ? QUNIPRO_SEL : 0,
		   REG_UFS_CFG1);

	if (host->hw_ver.major >= 0x05)
		ufshcd_rmwl(host->hba, QUNIPRO_G4_SEL, 0, REG_UFS_CFG0);
}

/*
 * ufs_qcom_host_reset - reset host controller and PHY
 */
static int ufs_qcom_host_reset(struct ufs_hba *hba)
{
	int ret = 0;
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	bool reenable_intr = false;

	host->reset_in_progress = true;

	if (!host->core_reset) {
		ufs_qcom_msg(WARN, hba->dev, "%s: reset control not set\n", __func__);
		goto out;
	}

	reenable_intr = hba->is_irq_enabled;
	disable_irq(hba->irq);
	hba->is_irq_enabled = false;

	ret = reset_control_assert(host->core_reset);
	if (ret) {
		ufs_qcom_msg(ERR, hba->dev, "%s: core_reset assert failed, err = %d\n",
				 __func__, ret);
		goto out;
	}

	/*
	 * The hardware requirement for delay between assert/deassert
	 * is at least 3-4 sleep clock (32.7KHz) cycles, which comes to
	 * ~125us (4/32768). To be on the safe side add 200us delay.
	 */
	usleep_range(200, 210);

	ret = reset_control_deassert(host->core_reset);
	if (ret)
		ufs_qcom_msg(ERR, hba->dev, "%s: core_reset deassert failed, err = %d\n",
				 __func__, ret);

	usleep_range(1000, 1100);
	/*
	 * The ice registers are also reset to default values after a ufs
	 * host controller reset. Reset the ice internal software flags here
	 * so that the ice hardware will be re-initialized properly in the
	 * later part of the UFS host controller reset.
	 */
	ufs_qcom_ice_disable(host);

	if (reenable_intr) {
		enable_irq(hba->irq);
		hba->is_irq_enabled = true;
	}

out:
	host->vdd_hba_pc = false;
	host->reset_in_progress = false;
	return ret;
}

static int ufs_qcom_phy_power_on(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	struct phy *phy = host->generic_phy;
	int ret = 0;

	mutex_lock(&host->phy_mutex);
	if (!host->is_phy_pwr_on) {
		ret = phy_power_on(phy);
		if (ret) {
			mutex_unlock(&host->phy_mutex);
			return ret;
		}
		host->is_phy_pwr_on = true;
	}
	mutex_unlock(&host->phy_mutex);

	return ret;
}

static int ufs_qcom_phy_power_off(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	struct phy *phy = host->generic_phy;
	int ret = 0;

	mutex_lock(&host->phy_mutex);
	if (host->is_phy_pwr_on) {
		ret = phy_power_off(phy);
		if (ret) {
			mutex_unlock(&host->phy_mutex);
			return ret;
		}
		host->is_phy_pwr_on = false;
	}
	mutex_unlock(&host->phy_mutex);

	return ret;
}

static int ufs_qcom_power_up_sequence(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	struct phy *phy = host->generic_phy;
	int ret = 0;
	enum phy_mode mode = (host->limit_rate == PA_HS_MODE_B) ?
					PHY_MODE_UFS_HS_B : PHY_MODE_UFS_HS_A;
	int submode = host->limit_phy_submode;

	if (host->hw_ver.major < 0x4)
		submode = UFS_QCOM_PHY_SUBMODE_NON_G4;
	phy_set_mode_ext(phy, mode, submode);

	ret = ufs_qcom_phy_power_on(hba);
	if (ret) {
		ufs_qcom_msg(ERR, hba->dev, "%s: phy power on failed, ret = %d\n",
				 __func__, ret);
		goto out;
	}

	ret = phy_calibrate(phy);
	if (ret) {
		ufs_qcom_msg(ERR, hba->dev, "%s: Failed to calibrate PHY %d\n",
				  __func__, ret);
		goto out;
	}

	ufs_qcom_select_unipro_mode(host);

out:
	return ret;
}

/*
 * The UTP controller has a number of internal clock gating cells (CGCs).
 * Internal hardware sub-modules within the UTP controller control the CGCs.
 * Hardware CGCs disable the clock to inactivate UTP sub-modules not involved
 * in a specific operation, UTP controller CGCs are by default disabled and
 * this function enables them (after every UFS link startup) to save some power
 * leakage.
 *
 * UFS host controller v3.0.0 onwards has internal clock gating mechanism
 * in Qunipro, enable them to save additional power.
 */
static int ufs_qcom_enable_hw_clk_gating(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	int err = 0;

	/* Enable UTP internal clock gating */
	ufshcd_writel(hba,
		ufshcd_readl(hba, REG_UFS_CFG2) | REG_UFS_CFG2_CGC_EN_ALL,
		REG_UFS_CFG2);

	if (host->hw_ver.major == 0x05)
		/* Ensure unused Unipro block's clock is gated */
		ufshcd_rmwl(host->hba, UNUSED_UNIPRO_CLK_GATED,
			UNUSED_UNIPRO_CLK_GATED, UFS_AH8_CFG);

	/* Ensure that HW clock gating is enabled before next operations */
	mb();

	/* Enable Qunipro internal clock gating if supported */
	if (!ufs_qcom_cap_qunipro_clk_gating(host))
		goto out;

	/* Enable all the mask bits */
	err = ufshcd_dme_rmw(hba, DL_VS_CLK_CFG_MASK,
				DL_VS_CLK_CFG_MASK, DL_VS_CLK_CFG);
	if (err)
		goto out;

	err = ufshcd_dme_rmw(hba, PA_VS_CLK_CFG_REG_MASK,
				PA_VS_CLK_CFG_REG_MASK, PA_VS_CLK_CFG_REG);
	if (err)
		goto out;

	if (!((host->hw_ver.major == 4) && (host->hw_ver.minor == 0) &&
	     (host->hw_ver.step == 0))) {
		err = ufshcd_dme_rmw(hba, DME_VS_CORE_CLK_CTRL_DME_HW_CGC_EN,
					DME_VS_CORE_CLK_CTRL_DME_HW_CGC_EN,
					DME_VS_CORE_CLK_CTRL);
	} else {
		ufs_qcom_msg(ERR, hba->dev, "%s: skipping DME_HW_CGC_EN set\n",
			__func__);
	}
out:
	return err;
}

static void ufs_qcom_force_mem_config(struct ufs_hba *hba)
{
	struct ufs_clk_info *clki;

	/*
	 * Configure the behavior of ufs clocks core and peripheral
	 * memory state when they are turned off.
	 * This configuration is required to allow retaining
	 * ICE crypto configuration (including keys) when
	 * core_clk_ice is turned off, and powering down
	 * non-ICE RAMs of host controller.
	 *
	 * This is applicable only to gcc clocks.
	 */
	list_for_each_entry(clki, &hba->clk_list_head, list) {

		/* skip it for non-gcc (rpmh) clocks */
		if (!strcmp(clki->name, "ref_clk"))
			continue;

		if (!strcmp(clki->name, "core_clk_ice") ||
			!strcmp(clki->name, "core_clk_ice_hw_ctl"))
			qcom_clk_set_flags(clki->clk, CLKFLAG_RETAIN_MEM);
		else
			qcom_clk_set_flags(clki->clk, CLKFLAG_NORETAIN_MEM);
		qcom_clk_set_flags(clki->clk, CLKFLAG_NORETAIN_PERIPH);
		qcom_clk_set_flags(clki->clk, CLKFLAG_PERIPH_OFF_CLEAR);
	}
	ufshcd_readl(hba, REG_UFS_CFG2);
}

static int ufs_qcom_hce_enable_notify(struct ufs_hba *hba,
				      enum ufs_notify_change_status status)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	int err = 0;

	switch (status) {
	case PRE_CHANGE:
		ufs_qcom_force_mem_config(hba);
		ufs_qcom_power_up_sequence(hba);
		/*
		 * The PHY PLL output is the source of tx/rx lane symbol
		 * clocks, hence, enable the lane clocks only after PHY
		 * is initialized.
		 */
		err = ufs_qcom_enable_lane_clks(host);
		if (err)
			ufs_qcom_msg(ERR, hba->dev, "%s: enable lane clks failed, ret=%d\n",
				__func__, err);
		/*
		 * ICE enable needs to be called before ufshcd_crypto_enable
		 * during resume as it is needed before reprogramming all
		 * keys. So moving it to PRE_CHANGE.
		 */
		ufs_qcom_ice_enable(host);
		break;
	case POST_CHANGE:
		err = ufs_qcom_config_shared_ice(host);
		if (err) {
			ufs_qcom_msg(ERR, hba->dev, "%s: config shared ice failed, ret=%d\n",
				__func__, err);
			break;
		}

		/* check if UFS PHY moved from DISABLED to HIBERN8 */
		err = ufs_qcom_check_hibern8(hba);
		ufs_qcom_enable_hw_clk_gating(hba);
		break;
	default:
		ufs_qcom_msg(ERR, hba->dev, "%s: invalid status %d\n", __func__, status);
		err = -EINVAL;
		break;
	}

	ufs_qcom_log_str(host, "-,%d,%d\n", status, err);

	if (host->dbg_en)
		trace_ufs_qcom_hce_enable_notify(dev_name(hba->dev), status, err);

	return err;
}

/*
 * Returns zero for success and non-zero in case of a failure
 */
static int __ufs_qcom_cfg_timers(struct ufs_hba *hba, u32 gear,
			       u32 hs, u32 rate, bool update_link_startup_timer,
			       bool is_pre_scale_up)
{
	int ret = 0;
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	struct ufs_clk_info *clki;
	u32 core_clk_period_in_ns;
	u32 tx_clk_cycles_per_us = 0;
	unsigned long core_clk_rate = 0;
	u32 core_clk_cycles_per_us = 0;

	static u32 pwm_fr_table[][2] = {
		{UFS_PWM_G1, 0x1},
		{UFS_PWM_G2, 0x1},
		{UFS_PWM_G3, 0x1},
		{UFS_PWM_G4, 0x1},
	};

	static u32 hs_fr_table_rA[][2] = {
		{UFS_HS_G1, 0x1F},
		{UFS_HS_G2, 0x3e},
		{UFS_HS_G3, 0x7D},
	};

	static u32 hs_fr_table_rB[][2] = {
		{UFS_HS_G1, 0x24},
		{UFS_HS_G2, 0x49},
		{UFS_HS_G3, 0x92},
	};

	/*
	 * The Qunipro controller does not use following registers:
	 * SYS1CLK_1US_REG, TX_SYMBOL_CLK_1US_REG, CLK_NS_REG &
	 * UFS_REG_PA_LINK_STARTUP_TIMER
	 * But UTP controller uses SYS1CLK_1US_REG register for Interrupt
	 * Aggregation logic / Auto hibern8 logic.
	 * It is mandatory to write SYS1CLK_1US_REG register on UFS host
	 * controller V4.0.0 onwards.
	*/
	if (ufs_qcom_cap_qunipro(host) &&
	    (!(ufshcd_is_intr_aggr_allowed(hba) ||
	       ufshcd_is_auto_hibern8_supported(hba) ||
	       host->hw_ver.major >= 4)))
		goto out;

	if (gear == 0) {
		ufs_qcom_msg(ERR, hba->dev, "%s: invalid gear = %d\n", __func__, gear);
		goto out_error;
	}

	list_for_each_entry(clki, &hba->clk_list_head, list) {
		if (!strcmp(clki->name, "core_clk")) {
			if (is_pre_scale_up)
				core_clk_rate = clki->max_freq;
			else
				core_clk_rate = clk_get_rate(clki->clk);
		}
	}

	/* If frequency is smaller than 1MHz, set to 1MHz */
	if (core_clk_rate < DEFAULT_CLK_RATE_HZ)
		core_clk_rate = DEFAULT_CLK_RATE_HZ;

	core_clk_cycles_per_us = core_clk_rate / USEC_PER_SEC;
	if (ufshcd_readl(hba, REG_UFS_SYS1CLK_1US) != core_clk_cycles_per_us) {
		ufshcd_writel(hba, core_clk_cycles_per_us, REG_UFS_SYS1CLK_1US);
		/*
		 * make sure above write gets applied before we return from
		 * this function.
		 */
		ufshcd_readl(hba, REG_UFS_SYS1CLK_1US);
	}

	if (ufs_qcom_cap_qunipro(host))
		goto out;

	core_clk_period_in_ns = NSEC_PER_SEC / core_clk_rate;
	core_clk_period_in_ns <<= OFFSET_CLK_NS_REG;
	core_clk_period_in_ns &= MASK_CLK_NS_REG;

	switch (hs) {
	case FASTAUTO_MODE:
	case FAST_MODE:
		if (rate == PA_HS_MODE_A) {
			if (gear > ARRAY_SIZE(hs_fr_table_rA)) {
				ufs_qcom_msg(ERR, hba->dev,
					"%s: index %d exceeds table size %zu\n",
					__func__, gear,
					ARRAY_SIZE(hs_fr_table_rA));
				goto out_error;
			}
			tx_clk_cycles_per_us = hs_fr_table_rA[gear-1][1];
		} else if (rate == PA_HS_MODE_B) {
			if (gear > ARRAY_SIZE(hs_fr_table_rB)) {
				ufs_qcom_msg(ERR, hba->dev,
					"%s: index %d exceeds table size %zu\n",
					__func__, gear,
					ARRAY_SIZE(hs_fr_table_rB));
				goto out_error;
			}
			tx_clk_cycles_per_us = hs_fr_table_rB[gear-1][1];
		} else {
			ufs_qcom_msg(ERR, hba->dev, "%s: invalid rate = %d\n",
				__func__, rate);
			goto out_error;
		}
		break;
	case SLOWAUTO_MODE:
	case SLOW_MODE:
		if (gear > ARRAY_SIZE(pwm_fr_table)) {
			ufs_qcom_msg(ERR, hba->dev,
					"%s: index %d exceeds table size %zu\n",
					__func__, gear,
					ARRAY_SIZE(pwm_fr_table));
			goto out_error;
		}
		tx_clk_cycles_per_us = pwm_fr_table[gear-1][1];
		break;
	case UNCHANGED:
	default:
		ufs_qcom_msg(ERR, hba->dev, "%s: invalid mode = %d\n", __func__, hs);
		goto out_error;
	}

	if (ufshcd_readl(hba, REG_UFS_TX_SYMBOL_CLK_NS_US) !=
	    (core_clk_period_in_ns | tx_clk_cycles_per_us)) {
		/* this register 2 fields shall be written at once */
		ufshcd_writel(hba, core_clk_period_in_ns | tx_clk_cycles_per_us,
			      REG_UFS_TX_SYMBOL_CLK_NS_US);
		/*
		 * make sure above write gets applied before we return from
		 * this function.
		 */
		mb();
	}

	if (update_link_startup_timer && host->hw_ver.major != 0x5) {
		ufshcd_writel(hba, ((core_clk_rate / MSEC_PER_SEC) * 100),
			      REG_UFS_CFG0);
		/*
		 * make sure that this configuration is applied before
		 * we return
		 */
		mb();
	}
	goto out;

out_error:
	ret = -EINVAL;
out:
	return ret;
}

static int ufs_qcom_cfg_timers(struct ufs_hba *hba, u32 gear,
			       u32 hs, u32 rate, bool update_link_startup_timer)
{
	return  __ufs_qcom_cfg_timers(hba, gear, hs, rate,
				      update_link_startup_timer, false);
}

static int ufs_qcom_set_dme_vs_core_clk_ctrl_max_freq_mode(struct ufs_hba *hba)
{
	struct ufs_clk_info *clki;
	struct list_head *head = &hba->clk_list_head;
	u32 max_freq = 0;
	int err = 0;

	list_for_each_entry(clki, head, list) {
		if (!IS_ERR_OR_NULL(clki->clk) &&
		    (!strcmp(clki->name, "core_clk_unipro"))) {
			max_freq = clki->max_freq;
			break;
		}
	}

	switch (max_freq) {
	case 300000000:
		err = ufs_qcom_set_dme_vs_core_clk_ctrl_clear_div(hba, 300, 12);
		break;
	case 150000000:
		err = ufs_qcom_set_dme_vs_core_clk_ctrl_clear_div(hba, 150, 6);
		break;
	default:
		err = -EINVAL;
		break;
	}

	return err;
}

/**
 * ufs_qcom_bypass_cfgready_signal - Tunes PA_VS_CONFIG_REG1 and
 * PA_VS_CONFIG_REG2 vendor specific attributes of local unipro
 * to bypass CFGREADY signal on Config interface between UFS
 * controller and PHY.
 *
 * The issue is related to config signals sampling from PHY
 * to controller. The PHY signals which are driven by 150MHz
 * clock and sampled by 300MHz instead of 150MHZ.
 *
 * The issue will be seen when only one of tx_cfg_rdyn_0
 * and tx_cfg_rdyn_1 is 0 around sampling clock edge and
 * if timing is not met as timing margin for some devices is
 * very less in one of the corner.
 *
 * To workaround this issue, controller should bypass the Cfgready
 * signal(TX_CFGREADY and RX_CFGREDY) because controller still wait
 * for another signal tx_savestatusn which will serve same purpose.
 *
 * The corresponding HW CR: 'QCTDD06985523' UFS HSG4 test fails
 * in SDF MAX GLS is linked to this issue.
 */
static int ufs_qcom_bypass_cfgready_signal(struct ufs_hba *hba)
{
	int err = 0;
	u32 pa_vs_config_reg1;
	u32 pa_vs_config_reg2;
	u32 mask;

	err = ufshcd_dme_get(hba, UIC_ARG_MIB(PA_VS_CONFIG_REG1),
			&pa_vs_config_reg1);
	if (err)
		goto out;

	err = ufshcd_dme_set(hba, UIC_ARG_MIB(PA_VS_CONFIG_REG1),
			(pa_vs_config_reg1 | BIT_TX_EOB_COND));
	if (err)
		goto out;

	err = ufshcd_dme_get(hba, UIC_ARG_MIB(PA_VS_CONFIG_REG2),
			&pa_vs_config_reg2);
	if (err)
		goto out;

	mask = (BIT_RX_EOB_COND | BIT_LINKCFG_WAIT_LL1_RX_CFG_RDY |
					H8_ENTER_COND_MASK);
	pa_vs_config_reg2 = (pa_vs_config_reg2 & ~mask) |
				(0x2 << H8_ENTER_COND_OFFSET);

	err = ufshcd_dme_set(hba, UIC_ARG_MIB(PA_VS_CONFIG_REG2),
			(pa_vs_config_reg2));
out:
	return err;
}

static void ufs_qcom_dump_attribs(struct ufs_hba *hba)
{
	int ret;
	int attrs[] = {0x15a0, 0x1552, 0x1553, 0x1554,
		       0x1555, 0x1556, 0x1557, 0x155a,
		       0x155b, 0x155c, 0x155d, 0x155e,
		       0x155f, 0x1560, 0x1561, 0x1568,
		       0x1569, 0x156a, 0x1571, 0x1580,
		       0x1581, 0x1583, 0x1584, 0x1585,
		       0x1586, 0x1587, 0x1590, 0x1591,
		       0x15a1, 0x15a2, 0x15a3, 0x15a4,
		       0x15a5, 0x15a6, 0x15a7, 0x15a8,
		       0x15a9, 0x15aa, 0x15ab, 0x15c0,
		       0x15c1, 0x15c2, 0x15d0, 0x15d1,
		       0x15d2, 0x15d3, 0x15d4, 0x15d5,
	};
	int cnt = ARRAY_SIZE(attrs);
	int i = 0, val;

	for (; i < cnt; i++) {
		ret = ufshcd_dme_get(hba, UIC_ARG_MIB(attrs[i]), &val);
		if (ret) {
			ufs_qcom_msg(ERR, hba->dev, "Failed reading: 0x%04x, ret:%d\n",
				attrs[i], ret);
			continue;
		}
		ufs_qcom_msg(ERR, hba->dev, "0x%04x: %d\n", attrs[i], val);
	}
}

static void ufs_qcom_validate_link_params(struct ufs_hba *hba)
{
	int val = 0;
	bool err = false;

	WARN_ON(ufs_qcom_get_connected_tx_lanes(hba, &val));
	if (val != hba->lanes_per_direction) {
		ufs_qcom_msg(ERR, hba->dev, "%s: Tx lane mismatch [config,reported] [%d,%d]\n",
			__func__, hba->lanes_per_direction, val);
		WARN_ON(1);
		err = true;
	}

	val = 0;
	WARN_ON(ufs_qcom_get_connected_rx_lanes(hba, &val));
	if (val != hba->lanes_per_direction) {
		ufs_qcom_msg(ERR, hba->dev, "%s: Rx lane mismatch [config,reported] [%d,%d]\n",
			__func__, hba->lanes_per_direction, val);
		WARN_ON(1);
		err = true;
	}

	if (err)
		ufs_qcom_dump_attribs(hba);
}

static int ufs_qcom_link_startup_notify(struct ufs_hba *hba,
					enum ufs_notify_change_status status)
{
	int err = 0;
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	struct phy *phy = host->generic_phy;
	struct device *dev = hba->dev;
	struct device_node *np = dev->of_node;
	u32 temp;

	switch (status) {
	case PRE_CHANGE:
		if (!of_property_read_bool(np, "secondary-storage") &&
				strlen(android_boot_dev) &&
				strcmp(android_boot_dev, dev_name(dev)))
			return -ENODEV;


		if (ufs_qcom_cfg_timers(hba, UFS_PWM_G1, SLOWAUTO_MODE,
					0, true)) {
			ufs_qcom_msg(ERR, hba->dev, "%s: ufs_qcom_cfg_timers() failed\n",
				__func__);
			err = -EINVAL;
			goto out;
		}

		ufs_qcom_phy_ctrl_rx_linecfg(phy, true);

		if (ufs_qcom_cap_qunipro(host)) {
			err = ufs_qcom_set_dme_vs_core_clk_ctrl_max_freq_mode(
				hba);
			if (err)
				goto out;
		}

		err = ufs_qcom_enable_hw_clk_gating(hba);
		if (err)
			goto out;

		/*
		 * Controller checks ICE configuration error without
		 * checking if the command is SCSI command
		 */
		temp = readl_relaxed(host->dev_ref_clk_ctrl_mmio);
		temp |= BIT(31);
		writel_relaxed(temp, host->dev_ref_clk_ctrl_mmio);
		/* ensure that UTP_SCASI_CHECK_DIS is enabled before link startup */
		wmb();

		/*
		 * Some UFS devices (and may be host) have issues if LCC is
		 * enabled. So we are setting PA_Local_TX_LCC_Enable to 0
		 * before link startup which will make sure that both host
		 * and device TX LCC are disabled once link startup is
		 * completed.
		 */
		if (ufshcd_get_local_unipro_ver(hba) != UFS_UNIPRO_VER_1_41)
			err = ufshcd_disable_host_tx_lcc(hba);
		if (err)
			goto out;

		if (host->bypass_g4_cfgready)
			err = ufs_qcom_bypass_cfgready_signal(hba);
		break;
	case POST_CHANGE:
		ufs_qcom_link_startup_post_change(hba);
		ufs_qcom_validate_link_params(hba);
		break;
	default:
		break;
	}

out:
	ufs_qcom_log_str(host, "*,%d,%d\n", status, err);
	if (host->dbg_en)
		trace_ufs_qcom_link_startup_notify(dev_name(hba->dev), status, err);

	return err;
}

static int ufs_qcom_config_vreg(struct device *dev,
		struct ufs_vreg *vreg, bool on)
{
	int ret = 0;
	struct regulator *reg;
	int min_uV, uA_load;

	if (!vreg) {
		WARN_ON(1);
		ret = -EINVAL;
		goto out;
	}

	reg = vreg->reg;
	if (regulator_count_voltages(reg) > 0) {
		uA_load = on ? vreg->max_uA : 0;
		ret = regulator_set_load(vreg->reg, uA_load);
		if (ret)
			goto out;
		if (vreg->min_uV && vreg->max_uV) {
			min_uV = on ? vreg->min_uV : 0;
			ret = regulator_set_voltage(reg, min_uV, vreg->max_uV);
			if (ret) {
				ufs_qcom_msg(ERR, dev, "%s: %s failed, err=%d\n",
					__func__, vreg->name, ret);
				goto out;
			}
		}
	}
out:
	return ret;
}

static int ufs_qcom_enable_vreg(struct device *dev, struct ufs_vreg *vreg)
{
	int ret = 0;

	if (vreg->enabled)
		return ret;

	ret = ufs_qcom_config_vreg(dev, vreg, true);
	if (ret)
		goto out;

	ret = regulator_enable(vreg->reg);
	if (ret)
		goto out;

	vreg->enabled = true;
out:
	return ret;
}

static int ufs_qcom_disable_vreg(struct device *dev, struct ufs_vreg *vreg)
{
	int ret = 0;

	if (!vreg->enabled)
		return ret;

	ret = regulator_disable(vreg->reg);
	if (ret)
		goto out;

	ret = ufs_qcom_config_vreg(dev, vreg, false);
	if (ret)
		goto out;

	vreg->enabled = false;
out:
	return ret;
}


static int ufs_qcom_mod_min_cpufreq(unsigned int cpu, s32 new_val)
{
	int ret = 0;
	struct freq_qos_request *qos_req;

	cpus_read_lock();
	if (cpu_online(cpu)) {
		qos_req = &per_cpu(qos_min_req, cpu);
		ret = freq_qos_update_request(qos_req, new_val);
	}
	cpus_read_unlock();
	return ret;
}

static int ufs_qcom_init_cpu_minfreq_req(struct ufs_qcom_host *host,
					unsigned int cpu)
{
	int ret;
	struct cpufreq_policy *policy;
	struct freq_qos_request *req;

	policy = cpufreq_cpu_get(cpu);
	if (!policy) {
		ufs_qcom_msg(ERR, host->hba->dev, "Failed to get cpu(%u)freq policy\n",
			cpu);
		return -EINVAL;
	}

	req = &per_cpu(qos_min_req, cpu);
	ret = freq_qos_add_request(&policy->constraints, req, FREQ_QOS_MIN,
				FREQ_QOS_MIN_DEFAULT_VALUE);
	if (ret < 0)
		ufs_qcom_msg(ERR, host->hba->dev, "Failed to add freq qos req\n");
	cpufreq_cpu_put(policy);

	return ret;
}

static void ufs_qcom_set_affinity_hint(struct ufs_hba *hba, bool prime)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	unsigned int set = 0;
	unsigned int clear = 0;
	int ret;
	cpumask_t *affinity_mask;

	if (prime) {
		set = IRQ_NO_BALANCING;
		affinity_mask = &host->perf_mask;
	} else {
		clear = IRQ_NO_BALANCING;
		affinity_mask = &host->silver_mask;
	}

	irq_modify_status(hba->irq, clear, set);
	ret = irq_set_affinity_hint(hba->irq, affinity_mask);
	if (ret < 0)
		ufs_qcom_msg(ERR, host->hba->dev, "prime=%d, err=%d\n", prime, ret);
}

static void ufs_qcom_toggle_pri_affinity(struct ufs_hba *hba, bool on)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	if (on && atomic_read(&host->therm_mitigation))
		return;

	atomic_set(&host->hi_pri_en, on);
	ufs_qcom_set_affinity_hint(hba, on);
}

static void ufs_qcom_cpufreq_dwork(struct work_struct *work)
{
	struct ufs_qcom_host *host = container_of(to_delayed_work(work),
							struct ufs_qcom_host,
							fwork);
	unsigned long cur_thres = atomic_read(&host->num_reqs_threshold);
	unsigned int freq_val = -1;
	int err = -1;

	atomic_set(&host->num_reqs_threshold, 0);

	if (cur_thres > NUM_REQS_HIGH_THRESH && !host->cur_freq_vote) {
		freq_val = host->max_cpu_scale_freq;
		if (host->irq_affinity_support)
			ufs_qcom_toggle_pri_affinity(host->hba, true);
	} else if (cur_thres < NUM_REQS_LOW_THRESH && host->cur_freq_vote) {
		freq_val = host->min_cpu_scale_freq;
		if (host->irq_affinity_support)
			ufs_qcom_toggle_pri_affinity(host->hba, false);
	}

	if (freq_val == -1)
		goto out;

	err = ufs_qcom_mod_min_cpufreq(host->config_cpu, freq_val);
	if (err < 0)
		ufs_qcom_msg(ERR, host->hba->dev, "fail set cpufreq-fmin to %d: %u\n",
				err, freq_val);
	else if (freq_val == host->max_cpu_scale_freq)
		host->cur_freq_vote = true;
	else if (freq_val == host->min_cpu_scale_freq)
		host->cur_freq_vote = false;
	ufs_qcom_msg(DBG, host->hba->dev, "cur_freq_vote=%d,freq_val=%u,cth=%u\n",
		host->cur_freq_vote, freq_val, cur_thres);
out:
	queue_delayed_work(host->ufs_qos->workq, &host->fwork,
			   msecs_to_jiffies(UFS_QCOM_LOAD_MON_DLY_MS));
}

static int add_group_qos(struct qos_cpu_group *qcg, enum constraint type)
{
	int cpu, err;
	struct dev_pm_qos_request *qos_req = qcg->qos_req;

	for_each_cpu(cpu, &qcg->mask) {
		ufs_qcom_msg(DBG, qcg->host->hba->dev,
			"%s: cpu: %d | mask: 0x%08x | assoc-qos-req: 0x%08x\n",
			__func__, cpu, qcg->mask, qos_req);
		memset(qos_req, 0,
		       sizeof(struct dev_pm_qos_request));
		err = dev_pm_qos_add_request(get_cpu_device(cpu),
					     qos_req,
					     DEV_PM_QOS_RESUME_LATENCY,
					     type);
		if (err < 0)
			return err;
		qos_req++;
	}
	return 0;
}

static int remove_group_qos(struct qos_cpu_group *qcg)
{
	int err, cpu;
	struct dev_pm_qos_request *qos_req = qcg->qos_req;

	for_each_cpu(cpu, &qcg->mask) {
		if (!dev_pm_qos_request_active(qos_req)) {
			qos_req++;
			continue;
		}
		err = dev_pm_qos_remove_request(qos_req);
		if (err < 0)
			return err;
		qos_req++;
	}
	return 0;
}

static void ufs_qcom_device_reset_ctrl(struct ufs_hba *hba, bool asserted)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	/* reset gpio is optional */
	if (!host->device_reset)
		return;

	gpiod_set_value_cansleep(host->device_reset, asserted);
}

static int ufs_qcom_suspend(struct ufs_hba *hba, enum ufs_pm_op pm_op,
	enum ufs_notify_change_status status)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	int err = 0;

	if (status == PRE_CHANGE)
		return 0;

	/*
	 * If UniPro link is not active or OFF, PHY ref_clk, main PHY analog
	 * power rail and low noise analog power rail for PLL can be
	 * switched off.
	 */
	if (!ufs_qcom_is_link_active(hba)) {
		ufs_qcom_disable_lane_clks(host);
		if (host->vddp_ref_clk && ufs_qcom_is_link_off(hba))
			err = ufs_qcom_disable_vreg(hba->dev,
					host->vddp_ref_clk);
		if (host->vccq_parent && !hba->auto_bkops_enabled)
			ufs_qcom_disable_vreg(hba->dev, host->vccq_parent);
		if (!err)
			err = ufs_qcom_unvote_qos_all(hba);
	}

	if (!err && ufs_qcom_is_link_off(hba) && host->device_reset)
		ufs_qcom_device_reset_ctrl(hba, true);
	else if (err)
		goto out;

	if (host->vccq_lpm_uV && !ufs_qcom_is_ufs_dev_active(hba) &&
		ufs_qcom_is_link_hibern8(hba)) {
		err = regulator_set_voltage(hba->vreg_info.vccq->reg,
			host->vccq_lpm_uV, host->vccq_lpm_uV);
		if (err)
			ufs_qcom_msg(ERR, hba->dev, "%s:vccq set %duV failed\n",
				__func__, host->vccq_lpm_uV);
	}
out:
	ufs_qcom_log_str(host, "&,%d,%d,%d,%d,%d,%d\n",
			pm_op, hba->rpm_lvl, hba->spm_lvl, hba->uic_link_state,
			hba->curr_dev_pwr_mode, err);
	if (host->dbg_en)
		trace_ufs_qcom_suspend(dev_name(hba->dev), pm_op, hba->rpm_lvl, hba->spm_lvl,
				hba->uic_link_state, hba->curr_dev_pwr_mode, err);
	ufs_qcom_ice_disable(host);

	cancel_dwork_unvote_cpufreq(hba);
	return err;
}

static int ufs_qcom_resume(struct ufs_hba *hba, enum ufs_pm_op pm_op)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	int err;

	if (host->vddp_ref_clk && (hba->rpm_lvl > UFS_PM_LVL_3 ||
				   hba->spm_lvl > UFS_PM_LVL_3))
		ufs_qcom_enable_vreg(hba->dev,
				      host->vddp_ref_clk);

	if (host->vccq_parent)
		ufs_qcom_enable_vreg(hba->dev, host->vccq_parent);

	err = ufs_qcom_enable_lane_clks(host);
	if (err)
		goto out;

	if (host->vccq_lpm_uV) {
		err = regulator_set_voltage(hba->vreg_info.vccq->reg,
			VCCQ_DEFAULT_1_2V, VCCQ_DEFAULT_1_2V);
		if (err)
			ufs_qcom_msg(ERR, hba->dev, "%s:vccq set 1.2V failed\n",
				__func__);
	}
out:
	ufs_qcom_log_str(host, "$,%d,%d,%d,%d,%d,%d\n",
			pm_op, hba->rpm_lvl, hba->spm_lvl, hba->uic_link_state,
			hba->curr_dev_pwr_mode, err);
	if (host->dbg_en)
		trace_ufs_qcom_resume(dev_name(hba->dev), pm_op, hba->rpm_lvl, hba->spm_lvl,
				hba->uic_link_state, hba->curr_dev_pwr_mode, err);
	return err;
}

static int ufs_qcom_get_bus_vote(struct ufs_qcom_host *host,
		const char *speed_mode)
{
	struct device *dev = host->hba->dev;
	struct device_node *np = dev->of_node;
	int err;
	const char *key = "qcom,bus-vector-names";

	if (!speed_mode) {
		err = -EINVAL;
		goto out;
	}

	if (host->bus_vote.is_max_bw_needed && !!strcmp(speed_mode, "MIN"))
		err = of_property_match_string(np, key, "MAX");
	else
		err = of_property_match_string(np, key, speed_mode);

out:
	if (err < 0)
		ufs_qcom_msg(ERR, dev, "%s: Invalid %s mode %d\n",
				__func__, speed_mode, err);
	return err;
}

static void ufs_qcom_get_speed_mode(struct ufs_pa_layer_attr *p, char *result)
{
	int gear = max_t(u32, p->gear_rx, p->gear_tx);
	int lanes = max_t(u32, p->lane_rx, p->lane_tx);
	int pwr;

	/* default to PWM Gear 1, Lane 1 if power mode is not initialized */
	if (!gear)
		gear = 1;

	if (!lanes)
		lanes = 1;

	if (!p->pwr_rx && !p->pwr_tx) {
		pwr = SLOWAUTO_MODE;
		snprintf(result, BUS_VECTOR_NAME_LEN, "MIN");
	} else if (p->pwr_rx == FAST_MODE || p->pwr_rx == FASTAUTO_MODE ||
		 p->pwr_tx == FAST_MODE || p->pwr_tx == FASTAUTO_MODE) {
		pwr = FAST_MODE;
		snprintf(result, BUS_VECTOR_NAME_LEN, "%s_R%s_G%d_L%d", "HS",
			 p->hs_rate == PA_HS_MODE_B ? "B" : "A", gear, lanes);
	} else {
		pwr = SLOW_MODE;
		snprintf(result, BUS_VECTOR_NAME_LEN, "%s_G%d_L%d",
			 "PWM", gear, lanes);
	}
}

static int ufs_qcom_get_ib_ab(struct ufs_qcom_host *host, int index,
			      struct qcom_bus_vectors *ufs_ddr_vec,
			      struct qcom_bus_vectors *cpu_ufs_vec)
{
	struct qcom_bus_path *usecase;

	if (!host->qbsd)
		return -EINVAL;

	if (index > host->qbsd->num_usecase)
		return -EINVAL;

	usecase = host->qbsd->usecase;

	/*
	 *
	 * usecase:0  usecase:0
	 * ufs->ddr   cpu->ufs
	 * |vec[0&1] | vec[2&3]|
	 * +----+----+----+----+
	 * | ab | ib | ab | ib |
	 * |----+----+----+----+
	 * .
	 * .
	 * .
	 * usecase:n  usecase:n
	 * ufs->ddr   cpu->ufs
	 * |vec[0&1] | vec[2&3]|
	 * +----+----+----+----+
	 * | ab | ib | ab | ib |
	 * |----+----+----+----+
	 */

	/* index refers to offset in usecase */
	ufs_ddr_vec->ab = usecase[index].vec[0].ab;
	ufs_ddr_vec->ib = usecase[index].vec[0].ib;

	cpu_ufs_vec->ab = usecase[index].vec[1].ab;
	cpu_ufs_vec->ib = usecase[index].vec[1].ib;

	return 0;
}

static int __ufs_qcom_set_bus_vote(struct ufs_qcom_host *host, int vote)
{
	int err = 0;
	struct qcom_bus_scale_data *d = host->qbsd;
	struct qcom_bus_vectors path0, path1;
	struct device *dev = host->hba->dev;

	err = ufs_qcom_get_ib_ab(host, vote, &path0, &path1);
	if (err) {
		ufs_qcom_msg(ERR, dev, "Error: failed (%d) to get ib/ab\n",
			err);
		return err;
	}

	ufs_qcom_msg(DBG, dev, "Setting vote: %d: ufs-ddr: ab: %llu ib: %llu\n", vote,
		path0.ab, path0.ib);
	err = icc_set_bw(d->ufs_ddr, path0.ab, path0.ib);
	if (err) {
		ufs_qcom_msg(ERR, dev, "Error: failed setting (%s) bus vote\n", err,
			UFS_DDR);
		return err;
	}

	ufs_qcom_msg(DBG, dev, "Setting: cpu-ufs: ab: %llu ib: %llu\n", path1.ab,
		path1.ib);
	err = icc_set_bw(d->cpu_ufs, path1.ab, path1.ib);
	if (err) {
		ufs_qcom_msg(ERR, dev, "Error: failed setting (%s) bus vote\n", err,
			CPU_UFS);
		return err;
	}

	host->bus_vote.curr_vote = vote;

	return err;
}

static int ufs_qcom_update_bus_bw_vote(struct ufs_qcom_host *host)
{
	int vote;
	int err = 0;
	char mode[BUS_VECTOR_NAME_LEN];

	ufs_qcom_get_speed_mode(&host->dev_req_params, mode);

	vote = ufs_qcom_get_bus_vote(host, mode);
	if (vote >= 0)
		err = __ufs_qcom_set_bus_vote(host, vote);
	else
		err = vote;

	if (err)
		ufs_qcom_msg(ERR, host->hba->dev, "%s: failed %d\n", __func__, err);
	else
		host->bus_vote.saved_vote = vote;
	return err;
}

static int ufs_qcom_set_bus_vote(struct ufs_hba *hba, bool on)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	int vote, err;

	/*
	 * In case ufs_qcom_init() is not yet done, simply ignore.
	 * This ufs_qcom_set_bus_vote() shall be called from
	 * ufs_qcom_init() after init is done.
	 */
	if (!host)
		return 0;

	if (on) {
		vote = host->bus_vote.saved_vote;
		if (vote == host->bus_vote.min_bw_vote)
			ufs_qcom_update_bus_bw_vote(host);
	} else {
		vote = host->bus_vote.min_bw_vote;
	}

	err = __ufs_qcom_set_bus_vote(host, vote);
	if (err)
		ufs_qcom_msg(ERR, hba->dev, "%s: set bus vote failed %d\n",
				 __func__, err);

	return err;
}

static ssize_t
show_ufs_to_mem_max_bus_bw(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	return scnprintf(buf, PAGE_SIZE, "%u\n",
			host->bus_vote.is_max_bw_needed);
}

static ssize_t
store_ufs_to_mem_max_bus_bw(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	uint32_t value;

	if (!kstrtou32(buf, 0, &value)) {
		host->bus_vote.is_max_bw_needed = !!value;
		ufs_qcom_update_bus_bw_vote(host);
	}

	return count;
}

static struct qcom_bus_scale_data *ufs_qcom_get_bus_scale_data(struct device
							       *dev)

{
	struct platform_device *pdev = to_platform_device(dev);
	struct device_node *of_node = dev->of_node;
	struct qcom_bus_scale_data *qsd;
	struct qcom_bus_path *usecase = NULL;
	int ret = 0, i = 0, j, num_paths, len;
	const uint32_t *vec_arr = NULL;
	bool mem_err = false;

	if (!pdev) {
		ufs_qcom_msg(ERR, dev, "Null platform device!\n");
		return NULL;
	}

	qsd = devm_kzalloc(dev, sizeof(struct qcom_bus_scale_data), GFP_KERNEL);
	if (!qsd)
		return NULL;

	ret = of_property_read_string(of_node, "qcom,ufs-bus-bw,name",
				      &qsd->name);
	if (ret) {
		ufs_qcom_msg(ERR, dev, "Error: (%d) Bus name missing!\n", ret);
		return NULL;
	}

	ret = of_property_read_u32(of_node, "qcom,ufs-bus-bw,num-cases",
		&qsd->num_usecase);
	if (ret) {
		ufs_qcom_msg(ERR, NULL, "Error: num-usecases not found\n");
		goto err;
	}

	usecase = devm_kzalloc(dev, (sizeof(struct qcom_bus_path) *
				   qsd->num_usecase), GFP_KERNEL);
	if (!usecase)
		return NULL;

	ret = of_property_read_u32(of_node, "qcom,ufs-bus-bw,num-paths",
				   &num_paths);
	if (ret) {
		ufs_qcom_msg(ERR, NULL, "Error: num_paths not found\n");
		return NULL;
	}

	vec_arr = of_get_property(of_node, "qcom,ufs-bus-bw,vectors-KBps",
				  &len);
	if (vec_arr == NULL) {
		ufs_qcom_msg(ERR, NULL, "Error: Vector array not found\n");
		return NULL;
	}

	for (i = 0; i < qsd->num_usecase; i++) {
		usecase[i].num_paths = num_paths;
		usecase[i].vec = devm_kzalloc(dev, num_paths *
					      sizeof(struct qcom_bus_vectors),
					      GFP_KERNEL);
		if (!usecase[i].vec) {
			mem_err = true;
			ufs_qcom_msg(ERR, dev, "Error: Failed to alloc mem for vectors\n");
			goto err;
		}

		for (j = 0; j < num_paths; j++) {
			uint32_t tab;
			int idx = ((i * num_paths) + j) * 2;

			tab = vec_arr[idx];
			usecase[i].vec[j].ab = ((tab & 0xff000000) >> 24) |
				((tab & 0x00ff0000) >> 8) |
				((tab & 0x0000ff00) << 8) | (tab << 24);

			tab = vec_arr[idx + 1];
			usecase[i].vec[j].ib = ((tab & 0xff000000) >> 24) |
				((tab & 0x00ff0000) >> 8) |
				((tab & 0x0000ff00) << 8) | (tab << 24);

			ufs_qcom_msg(DBG, dev, "ab: %llu ib:%llu [i]: %d [j]: %d\n",
				usecase[i].vec[j].ab, usecase[i].vec[j].ib, i,
				j);
		}
	}

	qsd->usecase = usecase;
	return qsd;
err:
	return NULL;
}


/**
 * ufs_qcom_enable_crash_on_err - read from DTS whether crash_on_err
 * should be enabled during boot.
 * @hba: per adapter instance
 */
static void ufs_qcom_enable_crash_on_err(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	struct device *dev = hba->dev;
	struct device_node *np = dev->of_node;

	if (!np)
		return;
	host->crash_on_err =
		of_property_read_bool(np, "qcom,enable_crash_on_err");
}

static int ufs_qcom_bus_register(struct ufs_qcom_host *host)
{
	int err = 0;
	struct device *dev = host->hba->dev;
	struct qcom_bus_scale_data *qsd;

	qsd = ufs_qcom_get_bus_scale_data(dev);
	if (!qsd) {
		ufs_qcom_msg(ERR, dev, "Failed: getting bus_scale data\n");
		return 0;
	}
	host->qbsd = qsd;

	qsd->ufs_ddr = of_icc_get(dev, UFS_DDR);
	if (IS_ERR(qsd->ufs_ddr)) {
		ufs_qcom_msg(ERR, dev, "Error: (%d) failed getting %s path\n",
			PTR_ERR(qsd->ufs_ddr), UFS_DDR);
		return PTR_ERR(qsd->ufs_ddr);
	}

	qsd->cpu_ufs = of_icc_get(dev, CPU_UFS);
	if (IS_ERR(qsd->cpu_ufs)) {
		ufs_qcom_msg(ERR, dev, "Error: (%d) failed getting %s path\n",
			PTR_ERR(qsd->cpu_ufs), CPU_UFS);
		return PTR_ERR(qsd->cpu_ufs);
	}

	/* cache the vote index for minimum and maximum bandwidth */
	host->bus_vote.min_bw_vote = ufs_qcom_get_bus_vote(host, "MIN");
	host->bus_vote.max_bw_vote = ufs_qcom_get_bus_vote(host, "MAX");

	host->bus_vote.max_bus_bw.show = show_ufs_to_mem_max_bus_bw;
	host->bus_vote.max_bus_bw.store = store_ufs_to_mem_max_bus_bw;
	sysfs_attr_init(&host->bus_vote.max_bus_bw.attr);
	host->bus_vote.max_bus_bw.attr.name = "max_bus_bw";
	host->bus_vote.max_bus_bw.attr.mode = 0644;
	err = device_create_file(dev, &host->bus_vote.max_bus_bw);
	if (err)
		ufs_qcom_msg(ERR, dev, "Error: (%d) Failed to create sysfs entries\n",
			err);
	return 0;
}

static void ufs_qcom_dev_ref_clk_ctrl(struct ufs_qcom_host *host, bool enable)
{
	if (host->dev_ref_clk_ctrl_mmio &&
	    (enable ^ host->is_dev_ref_clk_enabled)) {
		u32 temp = readl_relaxed(host->dev_ref_clk_ctrl_mmio);

		if (enable)
			temp |= host->dev_ref_clk_en_mask;
		else
			temp &= ~host->dev_ref_clk_en_mask;

		/*
		 * If we are here to disable this clock it might be immediately
		 * after entering into hibern8 in which case we need to make
		 * sure that device ref_clk is active for specific time after
		 * hibern8 enter.
		 */
		if (!enable) {
			unsigned long gating_wait;

			gating_wait = host->hba->dev_info.clk_gating_wait_us;
			if (!gating_wait) {
				udelay(1);
			} else {
				/*
				 * bRefClkGatingWaitTime defines the minimum
				 * time for which the reference clock is
				 * required by device during transition from
				 * HS-MODE to LS-MODE or HIBERN8 state. Give it
				 * more delay to be on the safe side.
				 */
				gating_wait += 10;
				usleep_range(gating_wait, gating_wait + 10);
			}
		}

		writel_relaxed(temp, host->dev_ref_clk_ctrl_mmio);

		/*
		 * Make sure the write to ref_clk reaches the destination and
		 * not stored in a Write Buffer (WB).
		 */
		readl(host->dev_ref_clk_ctrl_mmio);

		/*
		 * If we call hibern8 exit after this, we need to make sure that
		 * device ref_clk is stable for a given time before the hibern8
		 * exit command.
		 */
		if (enable)
			usleep_range(50, 60);

		host->is_dev_ref_clk_enabled = enable;
	}
}

static int ufs_qcom_pwr_change_notify(struct ufs_hba *hba,
				enum ufs_notify_change_status status,
				struct ufs_pa_layer_attr *dev_max_params,
				struct ufs_pa_layer_attr *dev_req_params)
{
	u32 val;
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	struct phy *phy = host->generic_phy;
struct ufs_qcom_dev_params ufs_qcom_cap;
	int ret = 0;

	if (!dev_req_params) {
		ufs_qcom_msg(ERR, NULL, "%s: incoming dev_req_params is NULL\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	switch (status) {
	case PRE_CHANGE:
		ufs_qcom_cap.hs_rx_gear = host->limit_rx_hs_gear;
		ufs_qcom_cap.hs_tx_gear = host->limit_tx_hs_gear;
		ufs_qcom_cap.pwm_tx_gear = host->limit_tx_pwm_gear;
		ufs_qcom_cap.pwm_rx_gear = host->limit_rx_pwm_gear;

		ufs_qcom_cap.tx_lanes = UFS_QCOM_LIMIT_NUM_LANES_TX;
		ufs_qcom_cap.rx_lanes = UFS_QCOM_LIMIT_NUM_LANES_RX;

		ufs_qcom_cap.rx_pwr_pwm = UFS_QCOM_LIMIT_RX_PWR_PWM;
		ufs_qcom_cap.tx_pwr_pwm = UFS_QCOM_LIMIT_TX_PWR_PWM;
		ufs_qcom_cap.rx_pwr_hs = UFS_QCOM_LIMIT_RX_PWR_HS;
		ufs_qcom_cap.tx_pwr_hs = UFS_QCOM_LIMIT_TX_PWR_HS;

		ufs_qcom_cap.hs_rate = host->limit_rate;

		ufs_qcom_cap.desired_working_mode =
					UFS_QCOM_LIMIT_DESIRED_MODE;

		ret = ufs_qcom_get_pwr_dev_param(&ufs_qcom_cap,
						 dev_max_params,
						 dev_req_params);
		if (ret) {
			ufs_qcom_msg(ERR, NULL, "%s: failed to determine capabilities\n",
					__func__);
			goto out;
		}

		/* enable the device ref clock before changing to HS mode */
		if (!ufshcd_is_hs_mode(&hba->pwr_info) &&
			ufshcd_is_hs_mode(dev_req_params))
			ufs_qcom_dev_ref_clk_ctrl(host, true);

		if (host->hw_ver.major >= 0x4) {
			if (dev_req_params->gear_tx >= UFS_HS_G4) {
				/* INITIAL ADAPT */
				ufshcd_dme_set(hba,
					       UIC_ARG_MIB(PA_TXHSADAPTTYPE),
					       PA_INITIAL_ADAPT);
			} else {
				/* NO ADAPT */
				ufshcd_dme_set(hba,
					       UIC_ARG_MIB(PA_TXHSADAPTTYPE),
					       PA_NO_ADAPT);
			}
		}
		break;
	case POST_CHANGE:
		if (ufs_qcom_cfg_timers(hba, dev_req_params->gear_rx,
					dev_req_params->pwr_rx,
					dev_req_params->hs_rate, false)) {
			ufs_qcom_msg(ERR, hba->dev, "%s: ufs_qcom_cfg_timers() failed\n",
				__func__);
			/*
			 * we return error code at the end of the routine,
			 * but continue to configure UFS_PHY_TX_LANE_ENABLE
			 * and bus voting as usual
			 */
			ret = -EINVAL;
		}

		val = ~(MAX_U32 << dev_req_params->lane_tx);
		ufs_qcom_phy_set_tx_lane_enable(phy, val);

		/* cache the power mode parameters to use internally */
		memcpy(&host->dev_req_params,
				dev_req_params, sizeof(*dev_req_params));
		ufs_qcom_update_bus_bw_vote(host);

		/* disable the device ref clock if entered PWM mode */
		if (ufshcd_is_hs_mode(&hba->pwr_info) &&
			!ufshcd_is_hs_mode(dev_req_params))
			ufs_qcom_dev_ref_clk_ctrl(host, false);
		/* update BER threshold depends on gear mode */
		if (!override_ber_threshold && !ret)
			ber_threshold = ber_table[dev_req_params->gear_rx].ber_threshold;
		break;
	default:
		ret = -EINVAL;
		break;
	}
out:
	if (dev_req_params) {
		ufs_qcom_log_str(host, "@,%d,%d,%d,%d,%d\n", status,
			dev_req_params->gear_rx, dev_req_params->pwr_rx,
			dev_req_params->hs_rate, ret);
		if (host->dbg_en)
			trace_ufs_qcom_pwr_change_notify(dev_name(hba->dev),
				status, dev_req_params->gear_rx,
				dev_req_params->pwr_rx,
				dev_req_params->hs_rate, ret);
	} else {
		ufs_qcom_log_str(host, "@,%d,%d,%d,%d,%d\n", status,
			0, 0, 0, ret);
		if (host->dbg_en)
			trace_ufs_qcom_pwr_change_notify(dev_name(hba->dev),
				status, 0, 0, 0, ret);
	}
	return ret;
}

static int ufs_qcom_vdd_hba_reg_notifier(struct notifier_block *nb,
					 unsigned long event, void *data)
{
	struct ufs_qcom_host *host = container_of(nb, struct ufs_qcom_host,
						  vdd_hba_reg_nb);

	switch (event) {
	case REGULATOR_EVENT_DISABLE:
		/* The flag will be cleared during h8 exit post change */
		if (ufs_qcom_is_link_hibern8(host->hba) &&
		    (host->chosen_algo != STATIC_ALLOC_ALG1))
			host->vdd_hba_pc = true;
		break;
	default:
		break;
	}

	return NOTIFY_OK;
}

static void ufs_qcom_hibern8_notify(struct ufs_hba *hba,
				    enum uic_cmd_dme uic_cmd,
				    enum ufs_notify_change_status status)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	/* Apply shared ICE WA */
	if (uic_cmd == UIC_CMD_DME_HIBER_EXIT && status == POST_CHANGE &&
	    host->vdd_hba_pc) {
		WARN_ON(host->chosen_algo == STATIC_ALLOC_ALG1);
		host->vdd_hba_pc = false;
		ufshcd_writel(hba, 0x18, UFS_MEM_ICE);
		ufshcd_writel(hba, 0x0, UFS_MEM_ICE);
	}
}

static int ufs_qcom_quirk_host_pa_saveconfigtime(struct ufs_hba *hba)
{
	int err;
	u32 pa_vs_config_reg1;

	err = ufshcd_dme_get(hba, UIC_ARG_MIB(PA_VS_CONFIG_REG1),
			     &pa_vs_config_reg1);
	if (err)
		goto out;

	/* Allow extension of MSB bits of PA_SaveConfigTime attribute */
	err = ufshcd_dme_set(hba, UIC_ARG_MIB(PA_VS_CONFIG_REG1),
			    (pa_vs_config_reg1 | (1 << 12)));

out:
	return err;
}

static void ufs_qcom_override_pa_h8time(struct ufs_hba *hba)
{
	int ret;
	u32 pa_h8time = 0;

	ret = ufshcd_dme_get(hba, UIC_ARG_MIB(PA_HIBERN8TIME),
				&pa_h8time);
	if (ret) {
		ufs_qcom_msg(ERR, hba->dev, "Failed getting PA_HIBERN8TIME: %d\n", ret);
		return;
	}


	/* 1 implies 100 us */
	ret = ufshcd_dme_set(hba, UIC_ARG_MIB(PA_HIBERN8TIME),
				pa_h8time + 1);
	if (ret)
		ufs_qcom_msg(ERR, hba->dev, "Failed updating PA_HIBERN8TIME: %d\n", ret);

}

static inline bool
ufshcd_is_valid_pm_lvl(enum ufs_pm_level lvl)
{
	return lvl >= 0 && lvl < UFS_PM_LVL_MAX;
}

static void ufshcd_parse_pm_levels(struct ufs_hba *hba)
{
	struct device *dev = hba->dev;
	struct device_node *np = dev->of_node;
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	enum ufs_pm_level rpm_lvl = UFS_PM_LVL_MAX, spm_lvl = UFS_PM_LVL_MAX;

	if (!np)
		return;

	if (host->is_dt_pm_level_read)
		return;

	if (!of_property_read_u32(np, "rpm-level", &rpm_lvl) &&
		ufshcd_is_valid_pm_lvl(rpm_lvl))
		hba->rpm_lvl = rpm_lvl;
	if (!of_property_read_u32(np, "spm-level", &spm_lvl) &&
		ufshcd_is_valid_pm_lvl(spm_lvl))
		hba->spm_lvl = spm_lvl;
	host->is_dt_pm_level_read = true;

	host->spm_lvl_default = hba->spm_lvl;
}

/*
 * ufs_qcom_parse_pbl_rst_workaround_flag - read bypass-pbl-rst-wa entry from DT
 */
static void ufs_qcom_parse_pbl_rst_workaround_flag(struct ufs_qcom_host *host)
{
	struct device_node *np = host->hba->dev->of_node;
	const char *str  = "qcom,bypass-pbl-rst-wa";

	if (!np)
		return;

	host->bypass_pbl_rst_wa = of_property_read_bool(np, str);
}


static void ufs_qcom_override_pa_tx_hsg1_sync_len(struct ufs_hba *hba)
{
#define PA_TX_HSG1_SYNC_LENGTH 0x1552
	int err;
	int sync_len_val = 0x4A;

	err = ufshcd_dme_peer_set(hba, UIC_ARG_MIB(PA_TX_HSG1_SYNC_LENGTH),
				  sync_len_val);
	if (err)
		ufs_qcom_msg(ERR, hba->dev, "Failed (%d) set PA_TX_HSG1_SYNC_LENGTH(%d)\n",
			     err, sync_len_val);
}

static int ufs_qcom_apply_dev_quirks(struct ufs_hba *hba)
{
	unsigned long flags;
	int err = 0;

	spin_lock_irqsave(hba->host->host_lock, flags);
	/* Set the rpm auto suspend delay to 3s */
	hba->host->hostt->rpm_autosuspend_delay = UFS_QCOM_AUTO_SUSPEND_DELAY;
	/* Set the default auto-hiberate idle timer value to 5ms */
	hba->ahit = FIELD_PREP(UFSHCI_AHIBERN8_TIMER_MASK, 5) |
		    FIELD_PREP(UFSHCI_AHIBERN8_SCALE_MASK, 3);
	/* Set the clock gating delay to performance mode */
	hba->clk_gating.delay_ms = UFS_QCOM_CLK_GATING_DELAY_MS_PERF;
	spin_unlock_irqrestore(hba->host->host_lock, flags);

	if (hba->dev_quirks & UFS_DEVICE_QUIRK_HOST_PA_SAVECONFIGTIME)
		err = ufs_qcom_quirk_host_pa_saveconfigtime(hba);

	if (hba->dev_info.wmanufacturerid == UFS_VENDOR_WDC)
		hba->dev_quirks |= UFS_DEVICE_QUIRK_HOST_PA_TACTIVATE;

	if (hba->dev_quirks & UFS_DEVICE_QUIRK_PA_HIBER8TIME)
		ufs_qcom_override_pa_h8time(hba);

	if (hba->dev_quirks & UFS_DEVICE_QUIRK_PA_TX_HSG1_SYNC_LENGTH)
		ufs_qcom_override_pa_tx_hsg1_sync_len(hba);

	ufshcd_parse_pm_levels(hba);

	if (hba->dev_info.wmanufacturerid == UFS_VENDOR_MICRON)
		hba->dev_quirks |= UFS_DEVICE_QUIRK_DELAY_BEFORE_LPM;

	return err;
}

static u32 ufs_qcom_get_ufs_hci_version(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	if (host->hw_ver.major == 0x1)
		return ufshci_version(1, 1);
	else
		return ufshci_version(2, 0);
}

/**
 * ufs_qcom_advertise_quirks - advertise the known QCOM UFS controller quirks
 * @hba: host controller instance
 *
 * QCOM UFS host controller might have some non standard behaviours (quirks)
 * than what is specified by UFSHCI specification. Advertise all such
 * quirks to standard UFS host controller driver so standard takes them into
 * account.
 */
static void ufs_qcom_advertise_quirks(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	if (host->hw_ver.major == 0x01) {
		hba->quirks |= UFSHCD_QUIRK_DELAY_BEFORE_DME_CMDS
			    | UFSHCD_QUIRK_BROKEN_PA_RXHSUNTERMCAP
			    | UFSHCD_QUIRK_DME_PEER_ACCESS_AUTO_MODE;

		if (host->hw_ver.minor == 0x0001 && host->hw_ver.step == 0x0001)
			hba->quirks |= UFSHCD_QUIRK_BROKEN_INTR_AGGR;

		hba->quirks |= UFSHCD_QUIRK_BROKEN_LCC;
	}

	if (host->hw_ver.major == 0x2) {
		hba->quirks |= UFSHCD_QUIRK_BROKEN_UFS_HCI_VERSION;

		if (!ufs_qcom_cap_qunipro(host))
			/* Legacy UniPro mode still need following quirks */
			hba->quirks |= (UFSHCD_QUIRK_DELAY_BEFORE_DME_CMDS
				| UFSHCD_QUIRK_DME_PEER_ACCESS_AUTO_MODE
				| UFSHCD_QUIRK_BROKEN_PA_RXHSUNTERMCAP);
	}

	if (host->disable_lpm)
		hba->quirks |= UFSHCD_QUIRK_BROKEN_AUTO_HIBERN8;

#if IS_ENABLED(CONFIG_SCSI_UFS_CRYPTO_QTI)
	hba->quirks |= UFSHCD_QUIRK_CUSTOM_KEYSLOT_MANAGER;
#endif
}

static void ufs_qcom_set_caps(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	if (!host->disable_lpm) {
		hba->caps |= UFSHCD_CAP_CLK_GATING |
		UFSHCD_CAP_HIBERN8_WITH_CLK_GATING |
		UFSHCD_CAP_CLK_SCALING |
		UFSHCD_CAP_WB_WITH_CLK_SCALING |
		UFSHCD_CAP_AUTO_BKOPS_SUSPEND;
		if (!host->disable_wb_support)
			hba->caps |= UFSHCD_CAP_WB_EN;
		hba->caps |= UFSHCD_CAP_AGGR_POWER_COLLAPSE;
	}

	hba->caps |= UFSHCD_CAP_CRYPTO;

	if (host->hw_ver.major >= 0x2)
		host->caps = UFS_QCOM_CAP_QUNIPRO |
			     UFS_QCOM_CAP_RETAIN_SEC_CFG_AFTER_PWR_COLLAPSE;

	if (host->hw_ver.major >= 0x3) {
		host->caps |= UFS_QCOM_CAP_QUNIPRO_CLK_GATING;
		/*
		 * The UFS PHY attached to v3.0.0 controller supports entering
		 * deeper low power state of SVS2. This lets the controller
		 * run at much lower clock frequencies for saving power.
		 * Assuming this and any future revisions of the controller
		 * support this capability. Need to revist this assumption if
		 * any future platform with this core doesn't support the
		 * capability, as there will be no benefit running at lower
		 * frequencies then.
		 */
		host->caps |= UFS_QCOM_CAP_SVS2;
	}

	if (host->hw_ver.major >= 0x5)
		host->caps |= UFS_QCOM_CAP_SHARED_ICE;
}

static int ufs_qcom_unvote_qos_all(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	struct ufs_qcom_qos_req *ufs_qos_req = host->ufs_qos;
	struct qos_cpu_group *qcg;
	int err = 0, i;

	if (!host->ufs_qos)
		return 0;

	qcg = ufs_qos_req->qcg;
	for (i = 0; i < ufs_qos_req->num_groups; i++, qcg++) {
		flush_work(&qcg->vwork);
		if (!qcg->voted)
			continue;
		err = ufs_qcom_update_qos_constraints(qcg, QOS_MAX);
		if (err)
			ufs_qcom_msg(ERR, hba->dev, "Failed (%d) removing qos grp(%d)\n",
				err, i);
	}
	return err;
}

/**
 * ufs_qcom_setup_clocks - enables/disable clocks
 * @hba: host controller instance
 * @on: If true, enable clocks else disable them.
 * @status: PRE_CHANGE or POST_CHANGE notify
 *
 * Returns 0 on success, non-zero on failure.
 */
static int ufs_qcom_setup_clocks(struct ufs_hba *hba, bool on,
				 enum ufs_notify_change_status status)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	int err = 0;
	struct phy *phy;
	u32 mode;

	/*
	 * In case ufs_qcom_init() is not yet done, simply ignore.
	 * This ufs_qcom_setup_clocks() shall be called from
	 * ufs_qcom_init() after init is done.
	 */
	if (!host)
		return 0;

	phy =  host->generic_phy;
	switch (status) {
	case PRE_CHANGE:
		if (on) {
			err = ufs_qcom_set_bus_vote(hba, true);
			if (ufs_qcom_is_link_hibern8(hba)) {
				err = ufs_qcom_enable_lane_clks(host);
				if (err) {
					ufs_qcom_msg(ERR, hba->dev,
							"%s: enable lane clks failed, ret=%d\n",
							__func__, err);
					return err;
				}
				ufs_qcom_phy_set_src_clk_h8_exit(phy);
			}

			if (!host->ref_clki->enabled) {
				err = clk_prepare_enable(host->ref_clki->clk);
				if (!err)
					host->ref_clki->enabled = on;
				else
					ufs_qcom_msg(ERR, hba->dev,
						"%s: Fail dev-ref-clk enabled, ret=%d\n",
						__func__, err);
				}

			if (!host->core_unipro_clki->enabled) {
				err = clk_prepare_enable(host->core_unipro_clki->clk);
				if (!err)
					host->core_unipro_clki->enabled = on;
				else
					ufs_qcom_msg(ERR, hba->dev,
						"%s: Fail core-unipro-clk enabled, ret=%d\n",
						__func__, err);
			}

		} else {
			if (!ufs_qcom_is_link_active(hba)) {
				clk_disable_unprepare(host->core_unipro_clki->clk);
				host->core_unipro_clki->enabled = on;

				err = ufs_qcom_phy_power_off(hba);
				if (err) {
					ufs_qcom_msg(ERR, hba->dev,
						"%s: phy power off failed, ret=%d\n",
						__func__, err);
					return err;
				}
				ufs_qcom_dev_ref_clk_ctrl(host, false);

				clk_disable_unprepare(host->ref_clki->clk);
				host->ref_clki->enabled = on;
			}
		}
		break;
	case POST_CHANGE:
		if (!on) {
			if (ufs_qcom_is_link_hibern8(hba)) {
				ufs_qcom_phy_set_src_clk_h8_enter(phy);
				/*
				 * As XO is set to the source of lane clocks, hence
				 * disable lane clocks to unvote on XO and allow XO shutdown
				 */
				ufs_qcom_disable_lane_clks(host);
			}

			err = ufs_qcom_set_bus_vote(hba, false);
			if (err)
				return err;
			err = ufs_qcom_unvote_qos_all(hba);
			idle_start = ktime_get();
		} else {
			err = ufs_qcom_phy_power_on(hba);
			if (err) {
				ufs_qcom_msg(ERR, hba->dev, "%s: phy power on failed, ret = %d\n",
						 __func__, err);
				return err;
			}

			if (ufshcd_is_hs_mode(&hba->pwr_info))
				ufs_qcom_dev_ref_clk_ctrl(host, true);

			for (mode = 0; mode < UFS_QCOM_BER_MODE_MAX; mode++)
				idle_time[mode] += ktime_to_ms(ktime_sub(ktime_get(),
									idle_start));
		}
		if (!err)
			atomic_set(&host->clks_on, on);
		break;
	}
	ufs_qcom_log_str(host, "#,%d,%d,%d\n", status, on, err);
	if (host->dbg_en)
		trace_ufs_qcom_setup_clocks(dev_name(hba->dev), status, on, err);

	return err;
}

static int
ufs_qcom_reset_assert(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct ufs_qcom_host *host = rcdev_to_ufs_host(rcdev);

	/* Currently this code only knows about a single reset. */
	WARN_ON(id);
	ufs_qcom_assert_reset(host->hba);
	/* provide 1ms delay to let the reset pulse propagate. */
	usleep_range(1000, 1100);
	return 0;
}

static int
ufs_qcom_reset_deassert(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct ufs_qcom_host *host = rcdev_to_ufs_host(rcdev);

	/* Currently this code only knows about a single reset. */
	WARN_ON(id);
	ufs_qcom_deassert_reset(host->hba);

	/*
	 * after reset deassertion, phy will need all ref clocks,
	 * voltage, current to settle down before starting serdes.
	 */
	usleep_range(1000, 1100);
	return 0;
}

static const struct reset_control_ops ufs_qcom_reset_ops = {
	.assert = ufs_qcom_reset_assert,
	.deassert = ufs_qcom_reset_deassert,
};

#ifndef MODULE
static int __init get_android_boot_dev(char *str)
{
	strscpy(android_boot_dev, str, ANDROID_BOOT_DEV_MAX);
	return 1;
}
__setup("androidboot.bootdevice=", get_android_boot_dev);
#endif

static int ufs_qcom_parse_reg_info(struct ufs_qcom_host *host, char *name,
				   struct ufs_vreg **out_vreg)
{
	int ret = 0;
	char prop_name[MAX_PROP_SIZE];
	struct ufs_vreg *vreg = NULL;
	struct device *dev = host->hba->dev;
	struct device_node *np = dev->of_node;

	if (!np) {
		ufs_qcom_msg(ERR, dev, "%s: non DT initialization\n", __func__);
		goto out;
	}

	snprintf(prop_name, MAX_PROP_SIZE, "%s-supply", name);
	if (!of_parse_phandle(np, prop_name, 0)) {
		ufs_qcom_msg(INFO, dev, "%s: Unable to find %s regulator, assuming enabled\n",
			 __func__, prop_name);
		ret = -ENODEV;
		goto out;
	}

	vreg = devm_kzalloc(dev, sizeof(*vreg), GFP_KERNEL);
	if (!vreg)
		return -ENOMEM;

	vreg->name = name;

	snprintf(prop_name, MAX_PROP_SIZE, "%s-max-microamp", name);
	ret = of_property_read_u32(np, prop_name, &vreg->max_uA);
	if (ret) {
		ufs_qcom_msg(ERR, dev, "%s: unable to find %s err %d\n",
			__func__, prop_name, ret);
		goto out;
	}

	vreg->reg = devm_regulator_get(dev, vreg->name);
	if (IS_ERR(vreg->reg)) {
		ret = PTR_ERR(vreg->reg);
		ufs_qcom_msg(ERR, dev, "%s: %s get failed, err=%d\n",
			__func__, vreg->name, ret);
	}

	snprintf(prop_name, MAX_PROP_SIZE, "%s-min-uV", name);
	ret = of_property_read_u32(np, prop_name, &vreg->min_uV);
	if (ret) {
		ufs_qcom_msg(DBG, dev, "%s: unable to find %s err %d, using default\n",
			__func__, prop_name, ret);
		if (!strcmp(name, "qcom,vddp-ref-clk"))
			vreg->min_uV = VDDP_REF_CLK_MIN_UV;
		else if (!strcmp(name, "qcom,vccq-parent"))
			vreg->min_uV = 0;
		ret = 0;
	}

	snprintf(prop_name, MAX_PROP_SIZE, "%s-max-uV", name);
	ret = of_property_read_u32(np, prop_name, &vreg->max_uV);
	if (ret) {
		ufs_qcom_msg(DBG, dev, "%s: unable to find %s err %d, using default\n",
			__func__, prop_name, ret);
		if (!strcmp(name, "qcom,vddp-ref-clk"))
			vreg->max_uV = VDDP_REF_CLK_MAX_UV;
		else if (!strcmp(name, "qcom,vccq-parent"))
			vreg->max_uV = 0;
		ret = 0;
	}

out:
	if (!ret)
		*out_vreg = vreg;
	return ret;
}

static void ufs_qcom_save_host_ptr(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	int id;

	if (!hba->dev->of_node)
		return;

	/* Extract platform data */
	id = of_alias_get_id(hba->dev->of_node, "ufshc");
	if (id <= 0)
		ufs_qcom_msg(ERR, hba->dev, "Failed to get host index %d\n", id);
	else if (id <= MAX_UFS_QCOM_HOSTS)
		ufs_qcom_hosts[id - 1] = host;
	else
		ufs_qcom_msg(ERR, hba->dev, "invalid host index %d\n", id);
}

/**
 * ufs_qcom_query_ioctl - perform user read queries
 * @hba: per-adapter instance
 * @lun: used for lun specific queries
 * @buffer: user space buffer for reading and submitting query data and params
 * @return: 0 for success negative error code otherwise
 *
 * Expected/Submitted buffer structure is struct ufs_ioctl_query_data.
 * It will read the opcode, idn and buf_length parameters, and, put the
 * response in the buffer field while updating the used size in buf_length.
 */
static int
ufs_qcom_query_ioctl(struct ufs_hba *hba, u8 lun, void __user *buffer)
{
	struct ufs_ioctl_query_data *ioctl_data;
	int err = 0;
	int length = 0;
	void *data_ptr;
	bool flag;
	u32 att;
	u8 index;
	u8 *desc = NULL;

	ioctl_data = kzalloc(sizeof(*ioctl_data), GFP_KERNEL);
	if (!ioctl_data) {
		err = -ENOMEM;
		goto out;
	}

	/* extract params from user buffer */
	err = copy_from_user(ioctl_data, buffer,
			     sizeof(struct ufs_ioctl_query_data));
	if (err) {
		ufs_qcom_msg(ERR, hba->dev,
			"%s: Failed copying buffer from user, err %d\n",
			__func__, err);
		goto out_release_mem;
	}

	/* verify legal parameters & send query */
	switch (ioctl_data->opcode) {
	case UPIU_QUERY_OPCODE_READ_DESC:
		switch (ioctl_data->idn) {
		case QUERY_DESC_IDN_DEVICE:
		case QUERY_DESC_IDN_CONFIGURATION:
		case QUERY_DESC_IDN_INTERCONNECT:
		case QUERY_DESC_IDN_GEOMETRY:
		case QUERY_DESC_IDN_POWER:
			index = 0;
			break;
		case QUERY_DESC_IDN_UNIT:
			if (!ufs_is_valid_unit_desc_lun(&hba->dev_info, lun, 0)) {
				ufs_qcom_msg(ERR, hba->dev,
					"%s: No unit descriptor for lun 0x%x\n",
					__func__, lun);
				err = -EINVAL;
				goto out_release_mem;
			}
			index = lun;
			break;
		default:
			goto out_einval;
		}
		length = min_t(int, QUERY_DESC_MAX_SIZE,
			       ioctl_data->buf_size);
		desc = kzalloc(length, GFP_KERNEL);
		if (!desc) {
			ufs_qcom_msg(ERR, hba->dev, "%s: Failed allocating %d bytes\n",
				__func__, length);
			err = -ENOMEM;
			goto out_release_mem;
		}
		err = ufshcd_query_descriptor_retry(hba, ioctl_data->opcode,
						    ioctl_data->idn, index, 0,
						    desc, &length);
		break;
	case UPIU_QUERY_OPCODE_READ_ATTR:
		switch (ioctl_data->idn) {
		case QUERY_ATTR_IDN_BOOT_LU_EN:
		case QUERY_ATTR_IDN_POWER_MODE:
		case QUERY_ATTR_IDN_ACTIVE_ICC_LVL:
		case QUERY_ATTR_IDN_OOO_DATA_EN:
		case QUERY_ATTR_IDN_BKOPS_STATUS:
		case QUERY_ATTR_IDN_PURGE_STATUS:
		case QUERY_ATTR_IDN_MAX_DATA_IN:
		case QUERY_ATTR_IDN_MAX_DATA_OUT:
		case QUERY_ATTR_IDN_REF_CLK_FREQ:
		case QUERY_ATTR_IDN_CONF_DESC_LOCK:
		case QUERY_ATTR_IDN_MAX_NUM_OF_RTT:
		case QUERY_ATTR_IDN_EE_CONTROL:
		case QUERY_ATTR_IDN_EE_STATUS:
		case QUERY_ATTR_IDN_SECONDS_PASSED:
			index = 0;
			break;
		case QUERY_ATTR_IDN_DYN_CAP_NEEDED:
		case QUERY_ATTR_IDN_CORR_PRG_BLK_NUM:
			index = lun;
			break;
		default:
			goto out_einval;
		}
		err = ufshcd_query_attr(hba, ioctl_data->opcode,
					ioctl_data->idn, index, 0, &att);
		break;

	case UPIU_QUERY_OPCODE_WRITE_ATTR:
		err = copy_from_user(&att,
				     buffer +
				     sizeof(struct ufs_ioctl_query_data),
				     sizeof(u32));
		if (err) {
			ufs_qcom_msg(ERR, hba->dev,
				"%s: Failed copying buffer from user, err %d\n",
				__func__, err);
			goto out_release_mem;
		}

		switch (ioctl_data->idn) {
		case QUERY_ATTR_IDN_BOOT_LU_EN:
			index = 0;
			if (!att) {
				ufs_qcom_msg(ERR, hba->dev,
					"%s: Illegal ufs query ioctl data, opcode 0x%x, idn 0x%x, att 0x%x\n",
					__func__, ioctl_data->opcode,
					(unsigned int)ioctl_data->idn, att);
				err = -EINVAL;
				goto out_release_mem;
			}
			break;
		default:
			goto out_einval;
		}
		err = ufshcd_query_attr(hba, ioctl_data->opcode,
					ioctl_data->idn, index, 0, &att);
		break;

	case UPIU_QUERY_OPCODE_READ_FLAG:
		switch (ioctl_data->idn) {
		case QUERY_FLAG_IDN_FDEVICEINIT:
		case QUERY_FLAG_IDN_PERMANENT_WPE:
		case QUERY_FLAG_IDN_PWR_ON_WPE:
		case QUERY_FLAG_IDN_BKOPS_EN:
		case QUERY_FLAG_IDN_PURGE_ENABLE:
		case QUERY_FLAG_IDN_FPHYRESOURCEREMOVAL:
		case QUERY_FLAG_IDN_BUSY_RTC:
			break;
		default:
			goto out_einval;
		}
		err = ufshcd_query_flag(hba, ioctl_data->opcode,
					ioctl_data->idn, 0, &flag);
		break;
	default:
		goto out_einval;
	}

	if (err) {
		ufs_qcom_msg(ERR, hba->dev, "%s: Query for idn %d failed\n", __func__,
			ioctl_data->idn);
		goto out_release_mem;
	}

	/*
	 * copy response data
	 * As we might end up reading less data than what is specified in
	 * "ioctl_data->buf_size". So we are updating "ioctl_data->
	 * buf_size" to what exactly we have read.
	 */
	switch (ioctl_data->opcode) {
	case UPIU_QUERY_OPCODE_READ_DESC:
		ioctl_data->buf_size = min_t(int, ioctl_data->buf_size, length);
		data_ptr = desc;
		break;
	case UPIU_QUERY_OPCODE_READ_ATTR:
		ioctl_data->buf_size = sizeof(u32);
		data_ptr = &att;
		break;
	case UPIU_QUERY_OPCODE_READ_FLAG:
		ioctl_data->buf_size = 1;
		data_ptr = &flag;
		break;
	case UPIU_QUERY_OPCODE_WRITE_ATTR:
		goto out_release_mem;
	default:
		goto out_einval;
	}

	/* copy to user */
	err = copy_to_user(buffer, ioctl_data,
			   sizeof(struct ufs_ioctl_query_data));
	if (err)
		ufs_qcom_msg(ERR, hba->dev, "%s: Failed copying back to user.\n",
			__func__);
	err = copy_to_user(buffer + sizeof(struct ufs_ioctl_query_data),
			   data_ptr, ioctl_data->buf_size);
	if (err)
		ufs_qcom_msg(ERR, hba->dev, "%s: err %d copying back to user.\n",
			__func__, err);
	goto out_release_mem;

out_einval:
	ufs_qcom_msg(ERR, hba->dev,
		"%s: illegal ufs query ioctl data, opcode 0x%x, idn 0x%x\n",
		__func__, ioctl_data->opcode, (unsigned int)ioctl_data->idn);
	err = -EINVAL;
out_release_mem:
	kfree(ioctl_data);
	kfree(desc);
out:
	return err;
}

/**
 * ufs_qcom_ioctl - ufs ioctl callback registered in scsi_host
 * @dev: scsi device required for per LUN queries
 * @cmd: command opcode
 * @buffer: user space buffer for transferring data
 *
 * Supported commands:
 * UFS_IOCTL_QUERY
 */
static int
ufs_qcom_ioctl(struct scsi_device *dev, unsigned int cmd, void __user *buffer)
{
	struct ufs_hba *hba = shost_priv(dev->host);
	int err = 0;

	BUG_ON(!hba);

	switch (cmd) {
	case UFS_IOCTL_QUERY:
		if (!buffer) {
			ufs_qcom_msg(ERR, hba->dev, "%s: User buffer is NULL!\n", __func__);
			return -EINVAL;
		}
		ufshcd_rpm_get_sync(hba);
		err = ufs_qcom_query_ioctl(hba,
					   ufshcd_scsi_to_upiu_lun(dev->lun),
					   buffer);
		ufshcd_rpm_put_sync(hba);
		break;
	default:
		err = -ENOIOCTLCMD;
		ufs_qcom_msg(DBG, hba->dev, "%s: Unsupported ioctl cmd %d\n", __func__,
			cmd);
		break;
	}

	return err;
}

static int tag_to_cpu(struct ufs_hba *hba, unsigned int tag)
{
	struct ufshcd_lrb *lrbp = &hba->lrb[tag];

	if (lrbp && lrbp->cmd && scsi_cmd_to_rq(lrbp->cmd))
		return blk_mq_rq_cpu(scsi_cmd_to_rq(lrbp->cmd));
	return -EINVAL;
}

static struct qos_cpu_group *cpu_to_group(struct ufs_qcom_qos_req *r,
					  unsigned int cpu)
{
	int i;
	struct qos_cpu_group *g = r->qcg;

	if (cpu > num_possible_cpus())
		return NULL;
	for (i = 0; i < r->num_groups; i++, g++) {
		if (cpumask_test_cpu(cpu, &g->mask))
			return &r->qcg[i];
	}
	return NULL;
}

static int ufs_qcom_update_qos_constraints(struct qos_cpu_group *qcg,
					   enum constraint type)
{
	unsigned int vote;
	int cpu, err;
	struct dev_pm_qos_request *qos_req = qcg->qos_req;

	if (type == QOS_MAX)
		vote = S32_MAX;
	else
		vote = qcg->votes[type];

	ufs_qcom_msg(DBG, qcg->host->hba->dev, "%s: qcg: 0x%08x | const: %d\n",
		__func__, qcg, type);
	if (qcg->curr_vote == vote)
		return 0;
	for_each_cpu(cpu, &qcg->mask) {
		err = dev_pm_qos_update_request(qos_req, vote);
		ufs_qcom_msg(DBG, qcg->host->hba->dev, "%s: vote: %d | cpu: %d | qos_req: 0x%08x\n",
			__func__, vote, cpu, qos_req);
		if (err < 0)
			return err;
		++qos_req;
	}
	if (type == QOS_MAX)
		qcg->voted = false;
	else
		qcg->voted = true;
	qcg->curr_vote = vote;
	return 0;
}

static void ufs_qcom_qos(struct ufs_hba *hba, int tag, bool is_scsi_cmd)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	struct qos_cpu_group *qcg;
	int cpu;

	if (!host->ufs_qos)
		return;
	cpu = tag_to_cpu(hba, tag);
	if (cpu < 0)
		return;
	qcg = cpu_to_group(host->ufs_qos, cpu);
	if (!qcg)
		return;

	if (qcg->perf_core && !host->cpufreq_dis &&
					!!atomic_read(&host->scale_up))
		atomic_inc(&host->num_reqs_threshold);

	if (qcg->voted) {
		ufs_qcom_msg(DBG, qcg->host->hba->dev,
			"%s: qcg: 0x%08x | Mask: 0x%08x - Already voted - return\n",
			__func__, qcg, qcg->mask);
		return;
	}
	queue_work(host->ufs_qos->workq, &qcg->vwork);
	ufs_qcom_msg(DBG, hba->dev, "Queued QoS work- cpu: %d\n", cpu);
}

static void ufs_qcom_vote_work(struct work_struct *work)
{
	int err;
	struct qos_cpu_group *qcg = container_of(work, struct qos_cpu_group,
						 vwork);

	err = ufs_qcom_update_qos_constraints(qcg, QOS_PERF);
	if (err)
		ufs_qcom_msg(ERR, qcg->host->hba->dev, "%s: update qos - failed: %d\n",
			__func__, err);
}

static int ufs_qcom_setup_qos(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	struct device *dev = hba->dev;
	struct ufs_qcom_qos_req *qr = host->ufs_qos;
	struct qos_cpu_group *qcg = qr->qcg;
	struct cpufreq_policy *policy;
	unsigned int cpu;
	int i, err;

	for (i = 0; i < qr->num_groups; i++, qcg++) {
		qcg->qos_req = kcalloc(cpumask_weight(&qcg->mask),
					sizeof(struct dev_pm_qos_request),
					GFP_KERNEL);
		if (!qcg->qos_req) {
			err = -ENOMEM;
			if (!i)
				return err;
			goto free_mem;
		}
		ufs_qcom_msg(DBG, dev,
			"%s: qcg: 0x%08x | mask: 0x%08x | mask-wt: %d | qos_req: 0x%08x\n",
			__func__, qcg, qcg->mask, cpumask_weight(&qcg->mask),
			qcg->qos_req);
		err = add_group_qos(qcg, S32_MAX);
		if (err < 0) {
			ufs_qcom_msg(ERR, dev, "Fail (%d) add qos-req: grp-%d\n",
				err, i);
			if (!i) {
				kfree(qcg->qos_req);
				return err;
			}
			goto free_mem;
		}
		INIT_WORK(&qcg->vwork, ufs_qcom_vote_work);

		if (host->cpufreq_dis || qcg->perf_core)
			continue;
		/* Bump up the cpufreq of the non-perf group */
		cpu = cpumask_first((const struct cpumask *)&qcg->mask);
		policy = cpufreq_cpu_get(cpu);
		if (!policy) {
			ufs_qcom_msg(ERR, dev, "Failed cpufreq policy,cpu=%d,mask=0x%08x\n",
				__func__, cpu, qcg->mask);
			host->cpufreq_dis = true;
			host->config_cpu = -1;
			continue;
		}
		host->config_cpu = cpu;
		host->min_cpu_scale_freq = policy->cpuinfo.min_freq;
		host->max_cpu_scale_freq = policy->cpuinfo.max_freq;
		cpufreq_cpu_put(policy);
	}

	if (!host->cpufreq_dis) {
		err = ufs_qcom_init_cpu_minfreq_req(host, host->config_cpu);
		if (err) {
			ufs_qcom_msg(ERR, dev, "Failed to register for freq_qos: %d\n",
				err);
			host->cpufreq_dis = true;
		} else {
			INIT_DELAYED_WORK(&host->fwork, ufs_qcom_cpufreq_dwork);
		}
	}
	qr->workq = create_singlethread_workqueue("qc_ufs_qos_swq");
	if (qr->workq)
		return 0;
	err = -1;
free_mem:
	while (i--) {
		kfree(qcg->qos_req);
		qcg--;
	}
	return err;
}

static void ufs_qcom_qos_init(struct ufs_hba *hba)
{
	struct device *dev = hba->dev;
	struct device_node *np = dev->of_node;
	struct device_node *group_node;
	struct ufs_qcom_qos_req *qr;
	struct qos_cpu_group *qcg;
	int i, err;
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	host->cpufreq_dis = true;
	qr = kzalloc(sizeof(*qr), GFP_KERNEL);
	if (!qr)
		return;

	host->ufs_qos = qr;
	qr->num_groups = of_get_available_child_count(np);
	ufs_qcom_msg(DBG, hba->dev, "num-groups: %d\n", qr->num_groups);
	if (!qr->num_groups) {
		ufs_qcom_msg(ERR, dev, "QoS groups undefined\n");
		kfree(qr);
		host->ufs_qos = NULL;
		return;
	}
	qcg = kzalloc(sizeof(*qcg) * qr->num_groups, GFP_KERNEL);
	if (!qcg) {
		kfree(qr);
		host->ufs_qos = NULL;
		return;
	}
	qr->qcg = qcg;
	for_each_available_child_of_node(np, group_node) {
		/*
		 * Due to Logical contiguous CPU numbering, one to one mapping
		 * between physical and logical cpu is no more applicable.
		 * Hence we don't need to pass the qos mask from the device tree
		 * and instead need to populate the mask dynamically
		 * using available kernel API.
		 */
		if (of_property_read_bool(group_node, "perf")) {
			qcg->perf_core = true;
			qcg->mask.bits[0] = host->gold_mask.bits[0] |
					host->gold_prime_mask.bits[0];
			host->cpufreq_dis = false;
		} else {
			qcg->perf_core = false;
			qcg->mask.bits[0] = host->silver_mask.bits[0];
		}

		err = of_property_count_u32_elems(group_node, "vote");
		if (err <= 0) {
			ufs_qcom_msg(ERR, dev, "1 vote is needed, bailing out: %d\n",
				err);
			goto out_err;
		}
		qcg->votes = kmalloc(sizeof(*qcg->votes) * err, GFP_KERNEL);
		if (!qcg->votes)
			goto out_err;
		for (i = 0; i < err; i++) {
			if (of_property_read_u32_index(group_node, "vote", i,
						       &qcg->votes[i]))
				goto out_vote_err;
		}
		ufs_qcom_msg(DBG, dev, "%s: qcg: 0x%08x\n", __func__, qcg);
		qcg->host = host;
		++qcg;
	}
	if (ufs_qcom_setup_qos(hba))
		goto out_vote_err;
	return;
out_vote_err:
	for (i = 0, qcg = qr->qcg; i < qr->num_groups; i++, qcg++)
		kfree(qcg->votes);
out_err:
	kfree(qr->qcg);
	kfree(qr);
	host->ufs_qos = NULL;
}

/**
 * ufs_qcom_populate_available_cpus - Populate all the available cpu masks -
 * Silver, gold and gold prime.
 * @hba: per adapter instance
 */
static void ufs_qcom_populate_available_cpus(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	int cid_cpu[MAX_NUM_CLUSTERS] = {-1, -1, -1};
	int cid = -1;
	int prev_cid = -1;
	int cpu;

	/*
	 * Due to Logical contiguous CPU numbering, one to one mapping
	 * between physical and logical cpu is no more applicable.
	 * Hence we are not passing cpu mask from the device tree.
	 * Hence populate the cpu mask dynamically as below.
	 */
	for_each_cpu(cpu, cpu_possible_mask) {
		cid = topology_physical_package_id(cpu);
		if (cid != prev_cid) {
			cid_cpu[cid] = cpu;
			prev_cid = cid;
		}
	}

	/*
	 * perf_mask which is needed for dynamic irq_affinity feature is updated
	 * with either gold or gold_prime depending on the availability of the core.
	 */
	if (cid_cpu[SILVER_CORE] != -1)
		host->silver_mask.bits[0] =
			topology_core_cpumask(cid_cpu[SILVER_CORE])->bits[0];

	if (cid_cpu[GOLD_CORE] != -1) {
		host->gold_mask.bits[0] =
			topology_core_cpumask(cid_cpu[GOLD_CORE])->bits[0];
		host->perf_mask.bits[0] = host->gold_mask.bits[0];
	}

	if (cid_cpu[GOLD_PRIME_CORE] != -1) {
		host->gold_prime_mask.bits[0] =
			topology_core_cpumask(cid_cpu[GOLD_PRIME_CORE])->bits[0];
		host->perf_mask.bits[0] = host->gold_prime_mask.bits[0];
	}
}

static void ufs_qcom_parse_irq_affinity(struct ufs_hba *hba)
{
	struct device *dev = hba->dev;
	struct device_node *np = dev->of_node;
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	int err;

	if (np) {
		/* check for dynamic irq affinity feature support */
		err = of_property_read_bool(np, "qcom,dynamic-irq-affinity");
		if (!err) {
			ufs_qcom_msg(DBG, host->hba->dev, "dynamic irq affinity not supported\n");
			return;
		}
	}

	/* If perf mask is available then only enable dynamic irq affinity feature */
	if (host->perf_mask.bits[0])
		host->irq_affinity_support = true;
}

static void ufs_qcom_parse_pm_level(struct ufs_hba *hba)
{
	struct device *dev = hba->dev;
	struct device_node *np = dev->of_node;
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	if (np) {
		if (of_property_read_u32(np, "rpm-level",
					 &hba->rpm_lvl))
			hba->rpm_lvl = -1;
		if (of_property_read_u32(np, "spm-level",
					 &hba->spm_lvl))
			hba->spm_lvl = -1;

		if (of_property_read_bool(np, "set-ds-spm-level"))
			host->set_ds_spm_level = true;
	}
}

/* Returns the max mitigation level supported */
static int ufs_qcom_get_max_therm_state(struct thermal_cooling_device *tcd,
				  unsigned long *data)
{
	*data = UFS_QCOM_LVL_MAX_THERM;

	return 0;
}

/* Returns the current mitigation level */
static int ufs_qcom_get_cur_therm_state(struct thermal_cooling_device *tcd,
				  unsigned long *data)
{
	struct ufs_hba *hba = dev_get_drvdata(tcd->devdata);
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	*data = host->uqt.curr_state;

	return 0;
}

/* Convert microseconds to Auto-Hibernate Idle Timer register value */
static u32 ufs_qcom_us_to_ahit(unsigned int timer)
{
	unsigned int scale;

	for (scale = 0; timer > UFSHCI_AHIBERN8_TIMER_MASK; ++scale)
		timer /= UFSHCI_AHIBERN8_SCALE_FACTOR;

	return FIELD_PREP(UFSHCI_AHIBERN8_TIMER_MASK, timer) |
	       FIELD_PREP(UFSHCI_AHIBERN8_SCALE_MASK, scale);
}

/* Sets the mitigation level to requested level */
static int ufs_qcom_set_cur_therm_state(struct thermal_cooling_device *tcd,
				  unsigned long data)
{
	struct ufs_hba *hba = dev_get_drvdata(tcd->devdata);
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	struct scsi_device *sdev;

	if (data == host->uqt.curr_state)
		return 0;

	switch (data) {
	case UFS_QCOM_LVL_NO_THERM:
		ufs_qcom_msg(WARN, tcd->devdata, "UFS host thermal mitigation stops\n");
		if (host->irq_affinity_support)
			atomic_set(&host->therm_mitigation, 0);

		/* Set the default auto-hiberate idle timer to 5 ms */
		ufshcd_auto_hibern8_update(hba, ufs_qcom_us_to_ahit(5000));

		/* Set the default auto suspend delay to 3000 ms */
		shost_for_each_device(sdev, hba->host)
			pm_runtime_set_autosuspend_delay(&sdev->sdev_gendev,
						UFS_QCOM_AUTO_SUSPEND_DELAY);
		break;
	case UFS_QCOM_LVL_AGGR_THERM:
	case UFS_QCOM_LVL_MAX_THERM:
		ufs_qcom_msg(WARN, tcd->devdata,
			"Going into UFS host thermal mitigation state, performance may be impacted before UFS host thermal mitigation stops\n");

		if (host->irq_affinity_support) {
			/* Stop setting hi-pri to requests and set irq affinity to default value */
			atomic_set(&host->therm_mitigation, 1);
			cancel_dwork_unvote_cpufreq(hba);
			ufs_qcom_toggle_pri_affinity(hba, false);
		}
		/* Set the default auto-hiberate idle timer to 1 ms */
		ufshcd_auto_hibern8_update(hba, ufs_qcom_us_to_ahit(1000));

		/* Set the default auto suspend delay to 100 ms */
		shost_for_each_device(sdev, hba->host)
			pm_runtime_set_autosuspend_delay(&sdev->sdev_gendev,
							 100);
		break;
	default:
		ufs_qcom_msg(ERR, tcd->devdata, "Invalid UFS thermal state (%d)\n", data);
		return -EINVAL;
	}

	host->uqt.curr_state = data;

	return 0;
}

struct thermal_cooling_device_ops ufs_thermal_ops = {
	.get_max_state = ufs_qcom_get_max_therm_state,
	.get_cur_state = ufs_qcom_get_cur_therm_state,
	.set_cur_state = ufs_qcom_set_cur_therm_state,
};

/* Static Algorithm */
static int ufs_qcom_config_alg1(struct ufs_hba *hba)
{
	int ret;
	unsigned int val, rx_aes;
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	unsigned int num_aes_cores;

	ret = of_property_read_u32(host->np, "rx-alloc-percent", &val);
	if (ret < 0)
		return ret;

	num_aes_cores = ufshcd_readl(hba, REG_UFS_MEM_ICE_NUM_AES_CORES);
	ufshcd_writel(hba, STATIC_ALLOC_ALG1, REG_UFS_MEM_SHARED_ICE_CONFIG);
	/*
	 * DTS specifies the percent allocation to rx stream
	 * Calculation -
	 *  Num Tx stream = N_TOT - (N_TOT * percent of rx stream allocation)
	 */
	rx_aes = DIV_ROUND_CLOSEST(num_aes_cores * val, 100);
	val = rx_aes | ((num_aes_cores - rx_aes) << 8);
	ufshcd_writel(hba, val, REG_UFS_MEM_SHARED_ICE_ALG1_NUM_CORE);

	return 0;
}

/* Floor based algorithm */
static int ufs_qcom_config_alg2(struct ufs_hba *hba)
{
	int i, ret;
	unsigned int reg = REG_UFS_MEM_SHARED_ICE_ALG2_NUM_CORE_0;
	/* 6 values for each group, refer struct shared_ice_alg2_config */
	unsigned int override_val[6];
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	char name[8];

	memset(name, 0, sizeof(name));
	ufshcd_writel(hba, FLOOR_BASED_ALG2, REG_UFS_MEM_SHARED_ICE_CONFIG);
	for (i = 0; i < ARRAY_SIZE(alg2_config); i++) {
		int core = 0, task = 0;

		if (host->np) {
			snprintf(name, sizeof(name), "%s%d", "g", i);
			ret = of_property_read_variable_u32_array(host->np,
								  name,
								  override_val,
								  6, 6);
			/* Some/All parameters may be overwritten */
			if (ret > 0)
				__get_alg2_grp_params(override_val, &core,
						      &task);
			else
				get_alg2_grp_params(i, &core, &task);
		} else {
			get_alg2_grp_params(i, &core, &task);
		}
		/* Num Core and Num task are contiguous & configured for a group together */
		ufshcd_writel(hba, core, reg);
		reg += 4;
		ufshcd_writel(hba, task, reg);
		reg += 4;
	}

	return 0;
}

/* Instantaneous algorithm */
static int ufs_qcom_config_alg3(struct ufs_hba *hba)
{
	unsigned int val[4];
	int ret;
	unsigned int config;
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	ret = of_property_read_variable_u32_array(host->np, "num-core", val,
						  4, 4);
	if (ret < 0)
		return ret;

	ufshcd_writel(hba, INSTANTANEOUS_ALG3, REG_UFS_MEM_SHARED_ICE_CONFIG);
	config = val[0] | (val[1] << 8) | (val[2] << 16) | (val[3] << 24);
	ufshcd_writel(hba, config, REG_UFS_MEM_SHARED_ICE_ALG3_NUM_CORE);

	return 0;
}

static int ufs_qcom_parse_shared_ice_config(struct ufs_hba *hba)
{
	struct device_node *np;
	int ret;
	const char *alg_name;
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	np = of_parse_phandle(hba->dev->of_node, "shared-ice-cfg", 0);
	if (!np)
		return -ENOENT;

	/* Only 1 algo can be enabled, pick the first */
	host->np = of_get_next_available_child(np, NULL);
	if (!host->np) {
		ufs_qcom_msg(ERR, hba->dev, "Resort to default alg2\n");
		/* No overrides, use floor based as default */
		host->chosen_algo = FLOOR_BASED_ALG2;
		return 0;
	}

	ret = of_property_read_string(host->np, "alg-name", &alg_name);
	if (ret < 0)
		return ret;

	if (!strcmp(alg_name, "alg1"))
		host->chosen_algo = STATIC_ALLOC_ALG1;
	else if (!strcmp(alg_name, "alg2"))
		host->chosen_algo = FLOOR_BASED_ALG2;
	else if (!strcmp(alg_name, "alg3"))
		host->chosen_algo = INSTANTANEOUS_ALG3;
	else
		/* Absurd condition */
		return -ENODATA;
	return ret;
}

static int ufs_qcom_config_shared_ice(struct ufs_qcom_host *host)
{
	if (!is_shared_ice_supported(host))
		return 0;

	switch (host->chosen_algo) {
	case STATIC_ALLOC_ALG1:
		return ufs_qcom_config_alg1(host->hba);
	case FLOOR_BASED_ALG2:
		return ufs_qcom_config_alg2(host->hba);
	case INSTANTANEOUS_ALG3:
		return ufs_qcom_config_alg3(host->hba);
	default:
		ufs_qcom_msg(ERR, host->hba->dev, "Unknown shared ICE algo (%d)\n",
			host->chosen_algo);
	}
	return -EINVAL;
}

static int ufs_qcom_shared_ice_init(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	if (!is_shared_ice_supported(host))
		return 0;

	/* Shared ICE is enabled by default */
	return ufs_qcom_parse_shared_ice_config(hba);
}

static int ufs_qcom_populate_ref_clk_ctrl(struct ufs_hba *hba)
{
	struct device *dev = hba->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	struct resource *res;

	/*
	 * for newer controllers, device reference clock control bit has
	 * moved inside UFS controller register address space itself.
	 */
	if (host->hw_ver.major >= 0x02) {
		host->dev_ref_clk_ctrl_mmio = hba->mmio_base + REG_UFS_CFG1;
		host->dev_ref_clk_en_mask = BIT(26);
		return 0;
	}

	/* "dev_ref_clk_ctrl_mem" is optional resource */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						"dev_ref_clk_ctrl_mem");
	if (res) {
		host->dev_ref_clk_ctrl_mmio =
			devm_ioremap_resource(dev, res);
		if (IS_ERR(host->dev_ref_clk_ctrl_mmio)) {
			dev_warn(dev,
				"%s: could not map dev_ref_clk_ctrl_mmio, err %ld\n",
				__func__, PTR_ERR(host->dev_ref_clk_ctrl_mmio));
				host->dev_ref_clk_ctrl_mmio = NULL;
		}
		host->dev_ref_clk_en_mask = BIT(5);
	}
	return 0;
}

static void ufs_qcom_setup_max_hs_gear(struct ufs_qcom_host *host)
{
	u32 param0;

	if (host->hw_ver.major == 0x1) {
		/*
		 * HS-G3 operations may not reliably work on legacy QCOM UFS
		 * host controller hardware even though capability exchange
		 * during link startup phase may end up negotiating maximum
		 * supported gear as G3. Hence, downgrade the maximum supported
		 * gear to HS-G2.
		 */
		host->max_hs_gear = UFS_HS_G2;
	} else if (host->hw_ver.major < 0x4) {
		host->max_hs_gear = UFS_HS_G3;
	} else {
		param0 = ufshcd_readl(host->hba, REG_UFS_PARAM0);
		host->max_hs_gear = UFS_QCOM_MAX_HS_GEAR(param0);
	}
}

static void ufs_qcom_register_minidump(uintptr_t vaddr, u64 size,
					const char *buf_name, u64 id)
{
	struct md_region md_entry;
	int ret;

	if (!msm_minidump_enabled())
		return;

	scnprintf(md_entry.name, sizeof(md_entry.name), "%s_%d",
			buf_name, id);
	md_entry.virt_addr = vaddr;
	md_entry.phys_addr = virt_to_phys((void *)vaddr);
	md_entry.size = size;

	ret = msm_minidump_add_region(&md_entry);
	if (ret < 0) {
		pr_err("Failed to register UFS buffer %s in Minidump ret %d\n",
				buf_name, ret);
		return;
	}
}

static int ufs_qcom_panic_handler(struct notifier_block *nb,
					unsigned long e, void *p)
{
	struct ufs_qcom_host *host = container_of(nb, struct ufs_qcom_host, ufs_qcom_panic_nb);
	struct ufs_hba *hba = host->hba;

	dev_err(hba->dev, "......dumping ufs info .......\n");

	dev_err(hba->dev, "UFS Host state=%d\n", hba->ufshcd_state);
	dev_err(hba->dev, "outstanding reqs=0x%lx tasks=0x%lx\n",
		hba->outstanding_reqs, hba->outstanding_tasks);
	dev_err(hba->dev, "saved_err=0x%x, saved_uic_err=0x%x\n",
		hba->saved_err, hba->saved_uic_err);
	dev_err(hba->dev, "Device power mode=%d, UIC link state=%d\n",
		hba->curr_dev_pwr_mode, hba->uic_link_state);
	dev_err(hba->dev, "PM in progress=%d, sys. suspended=%d\n",
		hba->pm_op_in_progress, hba->is_sys_suspended);

	dev_err(hba->dev, "Clk gate=%d\n", hba->clk_gating.state);
	dev_err(hba->dev, "last_hibern8_exit_tstamp at %lld us, hibern8_exit_cnt=%d\n",
		ktime_to_us(hba->ufs_stats.last_hibern8_exit_tstamp),
		hba->ufs_stats.hibern8_exit_cnt);
	dev_err(hba->dev, "last intr at %lld us, last intr status=0x%x\n",
		ktime_to_us(hba->ufs_stats.last_intr_ts),
		hba->ufs_stats.last_intr_status);
	dev_err(hba->dev, "error handling flags=0x%x, req. abort count=%d\n",
		hba->eh_flags, hba->req_abort_count);
	dev_err(hba->dev, "hba->ufs_version=0x%x, Host capabilities=0x%x, caps=0x%x\n",
		hba->ufs_version, hba->capabilities, hba->caps);
	dev_err(hba->dev, "quirks=0x%x, dev. quirks=0x%x\n", hba->quirks,
		hba->dev_quirks);

	dev_err(hba->dev, "[RX, TX]: gear=[%d, %d], lane[%d, %d], rate = %d\n",
		hba->pwr_info.gear_rx, hba->pwr_info.gear_tx,
		hba->pwr_info.lane_rx, hba->pwr_info.lane_tx,
		hba->pwr_info.hs_rate);

	dev_err(hba->dev, "UFS RPM level = %d\n", hba->rpm_lvl);
	dev_err(hba->dev, "UFS SPM level = %d\n", hba->spm_lvl);

	dev_err(hba->dev, "host_blocked=%d\n host_failed =%d\n Host self-block=%d\n",
		hba->host->host_blocked, hba->host->host_failed, hba->host->host_self_blocked);
	dev_err(hba->dev, "............. ufs dump complete ..........\n");

	return NOTIFY_OK;
}

static void ufs_qcom_set_rate_a(struct ufs_qcom_host *host)
{
	size_t len;
	u8 *data;
	struct nvmem_cell *nvmem_cell;

	nvmem_cell = nvmem_cell_get(host->hba->dev, "ufs_dev");
	if (IS_ERR(nvmem_cell)) {
		dev_err(host->hba->dev, "(%s) Failed to get nvmem cell\n", __func__);
		return;
	}

	data = (u8 *)nvmem_cell_read(nvmem_cell, &len);
	if (IS_ERR(data)) {
		dev_err(host->hba->dev, "(%s) Failed to read from nvmem\n", __func__);
		goto cell_put;
	}

	/*
	 * data equal to zero shows that ufs 2.x card is connected while
	 * non-zero value shows that ufs 3.x card is connected
	 */
	if (*data) {
		host->limit_rate = PA_HS_MODE_A;
		dev_dbg(host->hba->dev, "UFS 3.x device is detected, Mode is set to RATE A\n");
	}

	kfree(data);

cell_put:
	nvmem_cell_put(nvmem_cell);
}

/**
 * ufs_qcom_init - bind phy with controller
 * @hba: host controller instance
 *
 * Binds PHY with controller and powers up PHY enabling clocks
 * and regulators.
 *
 * Returns -EPROBE_DEFER if binding fails, returns negative error
 * on phy power up failure and returns zero on success.
 */
static int ufs_qcom_init(struct ufs_hba *hba)
{
	char type[5];
	int err, host_id = 0;
	struct device *dev = hba->dev;
	struct ufs_qcom_host *host;
	struct ufs_qcom_thermal *ut;
	struct ufs_clk_info *clki;

	host = devm_kzalloc(dev, sizeof(*host), GFP_KERNEL);
	if (!host) {
		err = -ENOMEM;
		ufs_qcom_msg(ERR, dev, "%s: no memory for qcom ufs host\n", __func__);
		goto out;
	}

	/* Make a two way bind between the qcom host and the hba */
	host->hba = hba;
	ufshcd_set_variant(hba, host);
	ut = &host->uqt;
#if defined(CONFIG_UFS_DBG)
	host->dbg_en = true;
#endif

	ufs_qcom_enable_crash_on_err(hba);
	/* Setup the reset control of HCI */
	host->core_reset = devm_reset_control_get(hba->dev, "rst");
	if (IS_ERR(host->core_reset)) {
		err = PTR_ERR(host->core_reset);
		ufs_qcom_msg(WARN, dev, "Failed to get reset control %d\n", err);
		host->core_reset = NULL;
		err = 0;
	}

	/* Fire up the reset controller. Failure here is non-fatal. */
	host->rcdev.of_node = dev->of_node;
	host->rcdev.ops = &ufs_qcom_reset_ops;
	host->rcdev.owner = dev->driver->owner;
	host->rcdev.nr_resets = 1;
	err = devm_reset_controller_register(dev, &host->rcdev);
	if (err) {
		ufs_qcom_msg(WARN, dev, "Failed to register reset controller\n");
		err = 0;
	}

	/*
	 * voting/devoting device ref_clk source is time consuming hence
	 * skip devoting it during aggressive clock gating. This clock
	 * will still be gated off during runtime suspend.
	 */
	host->generic_phy = devm_phy_get(dev, "ufsphy");

	if (host->generic_phy == ERR_PTR(-EPROBE_DEFER)) {
		/*
		 * UFS driver might be probed before the phy driver does.
		 * In that case we would like to return EPROBE_DEFER code.
		 */
		err = -EPROBE_DEFER;
		ufs_qcom_msg(WARN, dev, "%s: required phy device. hasn't probed yet. err = %d\n",
			__func__, err);
		goto out_variant_clear;
	} else if (IS_ERR(host->generic_phy)) {
		if (has_acpi_companion(dev)) {
			host->generic_phy = NULL;
		} else {
			err = PTR_ERR(host->generic_phy);
			ufs_qcom_msg(ERR, dev, "%s: PHY get failed %d\n", __func__, err);
			goto out_variant_clear;
		}
	}

	host->device_reset = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(host->device_reset)) {
		err = PTR_ERR(host->device_reset);
		if (err != -EPROBE_DEFER)
			ufs_qcom_msg(ERR, dev, "failed to acquire reset gpio: %d\n", err);
		goto out_variant_clear;
	}

	err = ufs_qcom_bus_register(host);
	if (err)
		goto out_variant_clear;

	ufs_qcom_get_controller_revision(hba, &host->hw_ver.major,
		&host->hw_ver.minor, &host->hw_ver.step);

	ufs_qcom_populate_ref_clk_ctrl(hba);

	ufs_qcom_setup_max_hs_gear(host);

	/* Register vdd_hba vreg callback */
	host->vdd_hba_reg_nb.notifier_call = ufs_qcom_vdd_hba_reg_notifier;
	devm_regulator_register_notifier(hba->vreg_info.vdd_hba->reg,
					 &host->vdd_hba_reg_nb);

	/* update phy revision information before calling phy_init() */
	err = ufs_qcom_phy_save_controller_version(host->generic_phy,
			host->hw_ver.major, host->hw_ver.minor, host->hw_ver.step);

	if (err == -EPROBE_DEFER) {
		pr_err("%s: phy device probe is not completed yet\n", __func__);
		goto out_variant_clear;
	}

	err = ufs_qcom_parse_reg_info(host, "qcom,vddp-ref-clk",
				      &host->vddp_ref_clk);

	err = phy_init(host->generic_phy);
	if (err) {
		ufs_qcom_msg(ERR, hba->dev, "%s: phy init failed, err %d\n",
				__func__, err);
		goto out_variant_clear;
	}
	mutex_init(&host->phy_mutex);

	if (host->vddp_ref_clk) {
		err = ufs_qcom_enable_vreg(dev, host->vddp_ref_clk);
		if (err) {
			ufs_qcom_msg(ERR, dev, "%s: failed enabling ref clk supply: %d\n",
				__func__, err);
			goto out_phy_exit;
		}
	}

	err = ufs_qcom_parse_reg_info(host, "qcom,vccq-parent",
				      &host->vccq_parent);
	if (host->vccq_parent) {
		err = ufs_qcom_enable_vreg(dev, host->vccq_parent);
		if (err) {
			ufs_qcom_msg(ERR, dev, "%s: failed enable vccq-parent err=%d\n",
				__func__, err);
			goto out_disable_vddp;
		}
	}

	list_for_each_entry(clki, &hba->clk_list_head, list) {
		if (!strcmp(clki->name, "core_clk_unipro")) {
			clki->keep_link_active = true;
			host->core_unipro_clki = clki;
		} else if (!strcmp(clki->name, "ref_clk"))
			host->ref_clki = clki;
	}

	err = ufs_qcom_init_lane_clks(host);
	if (err)
		goto out_disable_vccq_parent;

	ufs_qcom_parse_pbl_rst_workaround_flag(host);
	ufs_qcom_parse_pm_level(hba);
	ufs_qcom_parse_limits(host);
	ufs_qcom_parse_g4_workaround_flag(host);
	ufs_qcom_parse_lpm(host);
	if (host->disable_lpm)
		pm_runtime_forbid(host->hba->dev);

	ufs_qcom_parse_wb(host);
	ufs_qcom_set_caps(hba);
	ufs_qcom_advertise_quirks(hba);

	err = ufs_qcom_shared_ice_init(hba);
	if (err)
		ufs_qcom_msg(ERR, hba->dev, "Shared ICE Init failed, ret=%d\n", err);

	err = ufs_qcom_ice_init(host);
	if (err)
		goto out_variant_clear;

	/*
	 * Instantiate crypto capabilities for wrapped keys.
	 * It is controlled by CONFIG_SCSI_UFS_CRYPTO_QTI.
	 * If this is not defined, this API would return zero and
	 * non-wrapped crypto capabilities will be instantiated.
	 */
	ufshcd_qti_hba_init_crypto_capabilities(hba);

	ufs_qcom_set_bus_vote(hba, true);
	/* enable the device ref clock for HS mode*/
	if (ufshcd_is_hs_mode(&hba->pwr_info))
		ufs_qcom_dev_ref_clk_ctrl(host, true);

	if (hba->dev->id < MAX_UFS_QCOM_HOSTS)
		ufs_qcom_hosts[hba->dev->id] = host;

	host->dbg_print_en |= UFS_QCOM_DEFAULT_DBG_PRINT_EN;
	ufs_qcom_get_default_testbus_cfg(host);
	err = ufs_qcom_testbus_config(host);
	if (err) {
		ufs_qcom_msg(WARN, dev, "%s: failed to configure the testbus %d\n",
				__func__, err);
		err = 0;
	}

	ufs_qcom_init_sysfs(hba);

	/* Provide SCSI host ioctl API */
	hba->host->hostt->ioctl = (int (*)(struct scsi_device *, unsigned int,
				   void __user *))ufs_qcom_ioctl;
#ifdef CONFIG_COMPAT
	hba->host->hostt->compat_ioctl = (int (*)(struct scsi_device *,
					  unsigned int,
					  void __user *))ufs_qcom_ioctl;
#endif

	/*
	 * Based on host_id, pass the appropriate device type
	 * to register thermal cooling device.
	 */

	host_id = of_alias_get_id(hba->dev->of_node, "ufshc");
	if ((host_id < 0) || (host_id > MAX_UFS_QCOM_HOSTS)) {
		ufs_qcom_msg(ERR, hba->dev, "Failed to get host index %d\n", host_id);
		host_id = 1;
	}

	snprintf(type, sizeof(type), "ufs%d", host_id);
	ut->tcd = devm_thermal_of_cooling_device_register(dev,
							  dev->of_node,
							  type,
							  dev,
							  &ufs_thermal_ops);
	if (IS_ERR(ut->tcd))
		ufs_qcom_msg(WARN, dev, "Thermal mitigation registration failed: %d\n",
			 PTR_ERR(ut->tcd));
	else
		host->uqt.curr_state = UFS_QCOM_LVL_NO_THERM;

	ufs_qcom_save_host_ptr(hba);

	ufs_qcom_populate_available_cpus(hba);
	ufs_qcom_qos_init(hba);
	ufs_qcom_parse_irq_affinity(hba);
	ufs_qcom_ber_mon_init(hba);
	host->ufs_ipc_log_ctx = ipc_log_context_create(UFS_QCOM_MAX_LOG_SZ,
							"ufs-qcom", 0);
	if (!host->ufs_ipc_log_ctx)
		ufs_qcom_msg(WARN, dev, "IPC Log init - failed\n");

	/* register minidump */
	if (msm_minidump_enabled()) {
		ufs_qcom_register_minidump((uintptr_t)host,
					sizeof(struct ufs_qcom_host), "UFS_QHOST", host_id);
		ufs_qcom_register_minidump((uintptr_t)hba,
					sizeof(struct ufs_hba), "UFS_HBA", host_id);
		ufs_qcom_register_minidump((uintptr_t)hba->host,
					sizeof(struct Scsi_Host), "UFS_SHOST", host_id);

		/* Register Panic handler to dump more information in case of kernel panic */
		host->ufs_qcom_panic_nb.notifier_call = ufs_qcom_panic_handler;
		host->ufs_qcom_panic_nb.priority = INT_MAX - 1;
		err = atomic_notifier_chain_register(&panic_notifier_list,
				&host->ufs_qcom_panic_nb);
		if (err)
			ufs_qcom_msg(WARN, dev, "Fail to register UFS panic notifier\n");
	}

	goto out;

out_disable_vccq_parent:
	if (host->vccq_parent)
		ufs_qcom_disable_vreg(dev, host->vccq_parent);
out_disable_vddp:
	if (host->vddp_ref_clk)
		ufs_qcom_disable_vreg(dev, host->vddp_ref_clk);
out_phy_exit:
	phy_exit(host->generic_phy);
out_variant_clear:
	ufshcd_set_variant(hba, NULL);
out:
	return err;
}

static void ufs_qcom_exit(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	ufs_qcom_disable_lane_clks(host);
	ufs_qcom_phy_power_off(hba);
	phy_exit(host->generic_phy);
}

static int ufs_qcom_set_dme_vs_core_clk_ctrl_clear_div(struct ufs_hba *hba,
						       u32 clk_1us_cycles,
						       u32 clk_40ns_cycles)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	int err;
	u32 core_clk_ctrl_reg, clk_cycles;
	u32 mask = DME_VS_CORE_CLK_CTRL_MAX_CORE_CLK_1US_CYCLES_MASK;
	u32 offset = 0;

	/* Bits mask and offset changed on UFS host controller V4.0.0 onwards */
	if (host->hw_ver.major >= 4) {
		mask = DME_VS_CORE_CLK_CTRL_MAX_CORE_CLK_1US_CYCLES_MASK_V4;
		offset = DME_VS_CORE_CLK_CTRL_MAX_CORE_CLK_1US_CYCLES_OFFSET_V4;
	}

	if (clk_1us_cycles > mask)
		return -EINVAL;

	err = ufshcd_dme_get(hba,
			     UIC_ARG_MIB(DME_VS_CORE_CLK_CTRL),
			     &core_clk_ctrl_reg);
	if (err)
		goto out;

	core_clk_ctrl_reg &= ~mask;
	core_clk_ctrl_reg |= clk_1us_cycles;
	core_clk_ctrl_reg <<= offset;

	/* Clear CORE_CLK_DIV_EN */
	core_clk_ctrl_reg &= ~DME_VS_CORE_CLK_CTRL_CORE_CLK_DIV_EN_BIT;

	err = ufshcd_dme_set(hba,
			     UIC_ARG_MIB(DME_VS_CORE_CLK_CTRL),
			     core_clk_ctrl_reg);

	/* UFS host controller V4.0.0 onwards needs to program
	 * PA_VS_CORE_CLK_40NS_CYCLES attribute per programmed frequency of
	 * unipro core clk of UFS host controller.
	 */
	if (!err && (host->hw_ver.major >= 4)) {
		if (clk_40ns_cycles > PA_VS_CORE_CLK_40NS_CYCLES_MASK)
			return -EINVAL;

		err = ufshcd_dme_get(hba,
				     UIC_ARG_MIB(PA_VS_CORE_CLK_40NS_CYCLES),
				     &clk_cycles);
		if (err)
			goto out;

		clk_cycles &= ~PA_VS_CORE_CLK_40NS_CYCLES_MASK;
		clk_cycles |= clk_40ns_cycles;

		err = ufshcd_dme_set(hba,
				     UIC_ARG_MIB(PA_VS_CORE_CLK_40NS_CYCLES),
				     clk_cycles);
	}
out:
	return err;
}

static int ufs_qcom_clk_scale_up_pre_change(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	struct ufs_pa_layer_attr *attr = &host->dev_req_params;
	int err = 0;

	if (!ufs_qcom_cap_qunipro(host))
		goto out;

	if (attr)
		__ufs_qcom_cfg_timers(hba, attr->gear_rx, attr->pwr_rx,
				      attr->hs_rate, false, true);

	err = ufs_qcom_set_dme_vs_core_clk_ctrl_max_freq_mode(hba);
out:
	return err;
}

static int ufs_qcom_clk_scale_up_post_change(struct ufs_hba *hba)
{
	unsigned long flags;

	spin_lock_irqsave(hba->host->host_lock, flags);
	hba->clk_gating.delay_ms = UFS_QCOM_CLK_GATING_DELAY_MS_PERF;
	spin_unlock_irqrestore(hba->host->host_lock, flags);

	return 0;
}

static int ufs_qcom_clk_scale_down_pre_change(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	int err;
	u32 core_clk_ctrl_reg;

	if (!ufs_qcom_cap_qunipro(host))
		return 0;

	err = ufshcd_dme_get(hba,
			    UIC_ARG_MIB(DME_VS_CORE_CLK_CTRL),
			    &core_clk_ctrl_reg);

	/* make sure CORE_CLK_DIV_EN is cleared */
	if (!err &&
	    (core_clk_ctrl_reg & DME_VS_CORE_CLK_CTRL_CORE_CLK_DIV_EN_BIT)) {
		core_clk_ctrl_reg &= ~DME_VS_CORE_CLK_CTRL_CORE_CLK_DIV_EN_BIT;
		err = ufshcd_dme_set(hba,
				    UIC_ARG_MIB(DME_VS_CORE_CLK_CTRL),
				    core_clk_ctrl_reg);
	}

	return err;
}

static int ufs_qcom_clk_scale_down_post_change(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	struct ufs_pa_layer_attr *attr = &host->dev_req_params;
	int err = 0;
	struct ufs_clk_info *clki;
	struct list_head *head = &hba->clk_list_head;
	u32 curr_freq = 0;
	unsigned long flags;

	spin_lock_irqsave(hba->host->host_lock, flags);
	hba->clk_gating.delay_ms = UFS_QCOM_CLK_GATING_DELAY_MS_PWR_SAVE;
	spin_unlock_irqrestore(hba->host->host_lock, flags);

	if (!ufs_qcom_cap_qunipro(host))
		return 0;

	if (attr)
		ufs_qcom_cfg_timers(hba, attr->gear_rx, attr->pwr_rx,
				    attr->hs_rate, false);

	list_for_each_entry(clki, head, list) {
		if (!IS_ERR_OR_NULL(clki->clk) &&
		    (!strcmp(clki->name, "core_clk_unipro"))) {
			curr_freq = clk_get_rate(clki->clk);
			break;
		}
	}

	switch (curr_freq) {
	case 37500000:
		err = ufs_qcom_set_dme_vs_core_clk_ctrl_clear_div(hba, 38, 2);
		break;
	case 75000000:
		err = ufs_qcom_set_dme_vs_core_clk_ctrl_clear_div(hba, 75, 3);
		break;
	default:
		err = -EINVAL;
		break;
	}

	return err;
}

/**
 * Check controller status
 * Returns true if controller is active, false otherwise
 */
static bool is_hba_active(struct ufs_hba *hba)
{
	return !!(ufshcd_readl(hba, REG_CONTROLLER_ENABLE) & CONTROLLER_ENABLE);
}

static int ufs_qcom_clk_scale_notify(struct ufs_hba *hba,
		bool scale_up, enum ufs_notify_change_status status)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	struct ufs_pa_layer_attr *dev_req_params = &host->dev_req_params;
	int err = 0;

	/* Check if controller is active to send commands, else commands will timeout */
	if (!is_hba_active(hba))
		goto out;

	if (status == PRE_CHANGE) {
		err = ufshcd_uic_hibern8_enter(hba);
		if (err)
			return err;
		if (scale_up) {
			err = ufs_qcom_clk_scale_up_pre_change(hba);
			if (!host->cpufreq_dis &&
			    !(atomic_read(&host->therm_mitigation))) {
				atomic_set(&host->num_reqs_threshold, 0);
				queue_delayed_work(host->ufs_qos->workq,
						  &host->fwork,
					msecs_to_jiffies(
						UFS_QCOM_LOAD_MON_DLY_MS));
			}
		} else {
			err = ufs_qcom_clk_scale_down_pre_change(hba);
			cancel_dwork_unvote_cpufreq(hba);
		}
		if (err)
			ufshcd_uic_hibern8_exit(hba);
	} else {
		if (scale_up)
			err = ufs_qcom_clk_scale_up_post_change(hba);
		else
			err = ufs_qcom_clk_scale_down_post_change(hba);


		if (err || !dev_req_params) {
			ufshcd_uic_hibern8_exit(hba);
			goto out;
		}
		ufs_qcom_cfg_timers(hba,
				    dev_req_params->gear_rx,
				    dev_req_params->pwr_rx,
				    dev_req_params->hs_rate,
				    false);
		ufs_qcom_update_bus_bw_vote(host);
		ufshcd_uic_hibern8_exit(hba);
	}

	if (!err)
		atomic_set(&host->scale_up, scale_up);
out:
	if (scale_up)
		ufs_qcom_log_str(host, "^,%d,%d\n", status, err);
	else
		ufs_qcom_log_str(host, "v,%d,%d\n", status, err);

	if (host->dbg_en)
		trace_ufs_qcom_clk_scale_notify(dev_name(hba->dev), status, scale_up, err);

	return err;
}

/*
 * ufs_qcom_ber_mon_init - init BER monition history mode name and register domain list.
 */
static void ufs_qcom_ber_mon_init(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	int m = UFS_QCOM_BER_MODE_G1_G4;

	INIT_LIST_HEAD(&host->regs_list_head);

	for (m = UFS_QCOM_BER_MODE_G1_G4; m < UFS_QCOM_BER_MODE_MAX; m++) {
		switch (m) {
		case UFS_QCOM_BER_MODE_G1_G4:
			host->ber_hist[m].name = "gear1-4";
			break;
		case UFS_QCOM_BER_MODE_G5:
			host->ber_hist[m].name = "gear5";
			break;
		default:
			break;
		}
	}
}

static void ufs_qcom_print_ber_hist(struct ufs_qcom_host *host)
{
	struct ufs_qcom_ber_hist *h;
	int m = UFS_QCOM_BER_MODE_G1_G4;
	int i;

	for (m = UFS_QCOM_BER_MODE_G1_G4; m < UFS_QCOM_BER_MODE_MAX; m++) {
		h = &host->ber_hist[m];
		for (i = 0; i < UFS_QCOM_EVT_LEN; i++) {
			int p = (i + h->pos) % UFS_QCOM_EVT_LEN;

			if (h->tstamp[p] == 0)
				continue;
			dev_err(host->hba->dev, "pa_err_%d=0x%x, err_code=0x%x, gear=%d, time=%lld us\n",
						p, h->uec_pa[p], h->err_code[p], h->gear[p],
						ktime_to_us(h->tstamp[p]));
		}
		dev_err(host->hba->dev, "total %llu PA error on %s mode\n", h->cnt, h->name);
	}
}

static bool ufs_qcom_cal_ber(struct ufs_qcom_host *host, struct ufs_qcom_ber_hist *h)
{
	int idx_start, idx_end, i;
	s64 total_run_time = 0;
	s64 total_full_time = 0;
	u32 gear = h->gear[(h->pos + UFS_QCOM_EVT_LEN - 1) % UFS_QCOM_EVT_LEN];

	if (!override_ber_threshold)
		ber_threshold = ber_table[gear].ber_threshold;

	if (!override_ber_duration)
		ber_mon_dur_ms = UFS_QCOM_BER_DUR_DEF_MS;

	dev_dbg(host->hba->dev, "ber=%d, dur_ms=%d, h->cnt=%d, pos=%d\n",
					ber_threshold, ber_mon_dur_ms, h->cnt, h->pos);

	/* return from here if total error count is not more than BER threshold */
	if (h->cnt <= ber_threshold)
		return false;

	/*
	 * BER threshold allows num (ber_threshold) of errors, and we have h->cnt errors,
	 * go find the error event slot which records the #(h->cnt - ber_threshold) error.
	 */
	idx_end = (h->cnt - 1) % UFS_QCOM_EVT_LEN;
	idx_start = ((h->cnt - 1) - ber_threshold) % UFS_QCOM_EVT_LEN;


	/*
	 * Calcuate the total valid running time
	 * plus one here due to the first step in
	 * following while loop is i--.
	 */
	i = idx_end + 1;
	do {
		i--;
		if (i < 0)
			i += UFS_QCOM_EVT_LEN;
		total_run_time += h->run_time[i];
		total_full_time += h->full_time[i];
	} while (i != idx_start);

	dev_dbg(host->hba->dev, "idx_start=%d, idx_end=%d, total_time=%lld ms, total_run_time=%lld ms\n",
					idx_start, idx_end, total_full_time, total_run_time);

	if (total_run_time < ber_mon_dur_ms)
		return true;

	return false;
}

static bool ufs_qcom_update_ber_event(struct ufs_qcom_host *host,
									  u32 data,
									  enum ufs_hs_gear_tag gear)
{
	struct ufs_qcom_ber_hist *h;
	ktime_t t_prv;
	u32 mode = ber_table[gear].mode;
	bool rc = false;

	h = &host->ber_hist[mode];
	h->uec_pa[h->pos] = data;

	/* For PHY errors, record PA error code */
	if ((data & 0xFF) & ~UIC_PHY_ADAPTER_LAYER_GENERIC_ERROR)
		h->err_code[h->pos] = ufshcd_readl(host->hba, UFS_MEM_REG_PA_ERR_CODE);

	h->gear[h->pos] = gear;
	h->tstamp[h->pos] = ktime_get();

	if (h->pos != 0)
		t_prv = h->tstamp[h->pos - 1];
	else if (h->cnt == 0)
		t_prv = ms_to_ktime(0);
	else
		t_prv = h->tstamp[UFS_QCOM_EVT_LEN - 1];

	/* Calcuate the valid running time since last PA error event */
	h->full_time[h->pos] = ktime_to_ms(ktime_sub(h->tstamp[h->pos], t_prv));
	h->run_time[h->pos] = h->full_time[h->pos] - idle_time[mode];

	/* reset idle time for next error event. */
	idle_time[mode] = 0;

	h->cnt++;
	h->pos = (h->pos + 1) % UFS_QCOM_EVT_LEN;

	/* don't need to calculate again if BER has already exceeded threshold */
	if (!host->ber_th_exceeded)
		rc = ufs_qcom_cal_ber(host, h);

	return rc;
}


static void ufs_qcom_save_all_regs(struct ufs_qcom_host *host)
{
	struct phy *phy = host->generic_phy;

	ufs_qcom_save_regs(host, 0, UFSHCI_REG_SPACE_SIZE,
							"host_regs ");
	ufs_qcom_save_regs(host, REG_UFS_SYS1CLK_1US, 16 * 4,
							"HCI Vendor Specific Registers ");
	ufs_qcom_save_regs(host, UFS_MEM_ICE, 29 * 4,
							"HCI Shared ICE Registers ");
	ufs_qcom_save_regs(host, UFS_TEST_BUS, 4,
							"UFS_TEST_BUS ");
	ufs_qcom_save_testbus(host);
	ufs_qcom_phy_dbg_register_save(phy);

}

static void ufs_qcom_event_notify(struct ufs_hba *hba,
								  enum ufs_event_type evt,
								  void *data)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	struct phy *phy = host->generic_phy;
	u32 reg = *(u32 *)data;
	bool ber_th_exceeded = false;
	bool evt_valid = true;

	switch (evt) {
	case UFS_EVT_PA_ERR:
		ber_th_exceeded = ufs_qcom_update_ber_event(host, reg,
								hba->pwr_info.gear_rx);

		if (ber_th_exceeded) {
			host->ber_th_exceeded = true;
			ufs_qcom_save_all_regs(host);
			ufs_qcom_print_ber_hist(host);

			if (crash_on_ber) {
				ufs_qcom_phy_dbg_register_dump(phy);
				BUG_ON(1);
			}
		}

		if (host->ber_th_exceeded)
			dev_warn_ratelimited(hba->dev, "Warning: UFS BER exceeds threshold !!!\n");

		break;
	case UFS_EVT_DL_ERR:
		if (reg == UIC_DATA_LINK_LAYER_EC_PA_ERROR_IND_RECEIVED)
			evt_valid = false;
		break;
	default:
		break;
	}

	if (evt_valid)
		host->valid_evt_cnt[evt]++;
}

void ufs_qcom_print_hw_debug_reg_all(struct ufs_hba *hba,
		void *priv, void (*print_fn)(struct ufs_hba *hba,
		int offset, int num_regs, const char *str, void *priv))
{
	u32 reg;
	struct ufs_qcom_host *host;

	if (unlikely(!hba)) {
		ufs_qcom_msg(ERR, NULL, "%s: hba is NULL\n", __func__);
		return;
	}
	if (unlikely(!print_fn)) {
		ufs_qcom_msg(ERR, hba->dev, "%s: print_fn is NULL\n", __func__);
		return;
	}

	host = ufshcd_get_variant(hba);
	if (!(host->dbg_print_en & UFS_QCOM_DBG_PRINT_REGS_EN))
		return;

	reg = ufs_qcom_get_debug_reg_offset(host, UFS_UFS_DBG_RD_REG_OCSC);
	print_fn(hba, reg, 44, "UFS_UFS_DBG_RD_REG_OCSC ", priv);

	reg = ufshcd_readl(hba, REG_UFS_CFG1);
	reg |= UTP_DBG_RAMS_EN;
	ufshcd_writel(hba, reg, REG_UFS_CFG1);

	reg = ufs_qcom_get_debug_reg_offset(host, UFS_UFS_DBG_RD_EDTL_RAM);
	print_fn(hba, reg, 32, "UFS_UFS_DBG_RD_EDTL_RAM ", priv);

	reg = ufs_qcom_get_debug_reg_offset(host, UFS_UFS_DBG_RD_DESC_RAM);
	print_fn(hba, reg, 128, "UFS_UFS_DBG_RD_DESC_RAM ", priv);

	reg = ufs_qcom_get_debug_reg_offset(host, UFS_UFS_DBG_RD_PRDT_RAM);
	print_fn(hba, reg, 64, "UFS_UFS_DBG_RD_PRDT_RAM ", priv);

	/* clear bit 17 - UTP_DBG_RAMS_EN */
	ufshcd_rmwl(hba, UTP_DBG_RAMS_EN, 0, REG_UFS_CFG1);

	reg = ufs_qcom_get_debug_reg_offset(host, UFS_DBG_RD_REG_UAWM);
	print_fn(hba, reg, 4, "UFS_DBG_RD_REG_UAWM ", priv);

	reg = ufs_qcom_get_debug_reg_offset(host, UFS_DBG_RD_REG_UARM);
	print_fn(hba, reg, 4, "UFS_DBG_RD_REG_UARM ", priv);

	reg = ufs_qcom_get_debug_reg_offset(host, UFS_DBG_RD_REG_TXUC);
	print_fn(hba, reg, 48, "UFS_DBG_RD_REG_TXUC ", priv);

	reg = ufs_qcom_get_debug_reg_offset(host, UFS_DBG_RD_REG_RXUC);
	print_fn(hba, reg, 27, "UFS_DBG_RD_REG_RXUC ", priv);

	reg = ufs_qcom_get_debug_reg_offset(host, UFS_DBG_RD_REG_DFC);
	print_fn(hba, reg, 19, "UFS_DBG_RD_REG_DFC ", priv);

	reg = ufs_qcom_get_debug_reg_offset(host, UFS_DBG_RD_REG_TRLUT);
	print_fn(hba, reg, 34, "UFS_DBG_RD_REG_TRLUT ", priv);

	reg = ufs_qcom_get_debug_reg_offset(host, UFS_DBG_RD_REG_TMRLUT);
	print_fn(hba, reg, 9, "UFS_DBG_RD_REG_TMRLUT ", priv);
}

static void ufs_qcom_enable_test_bus(struct ufs_qcom_host *host)
{
	if (host->dbg_print_en & UFS_QCOM_DBG_PRINT_TEST_BUS_EN) {
		ufshcd_rmwl(host->hba, UFS_REG_TEST_BUS_EN,
				UFS_REG_TEST_BUS_EN, REG_UFS_CFG1);
		ufshcd_rmwl(host->hba, TEST_BUS_EN, TEST_BUS_EN, REG_UFS_CFG1);
	} else {
		ufshcd_rmwl(host->hba, UFS_REG_TEST_BUS_EN, 0, REG_UFS_CFG1);
		ufshcd_rmwl(host->hba, TEST_BUS_EN, 0, REG_UFS_CFG1);
	}
}

static void ufs_qcom_get_default_testbus_cfg(struct ufs_qcom_host *host)
{
	/* provide a legal default configuration */
	host->testbus.select_major = TSTBUS_UNIPRO;
	host->testbus.select_minor = 37;
}

static bool ufs_qcom_testbus_cfg_is_ok(struct ufs_qcom_host *host)
{
	if (host->testbus.select_major >= TSTBUS_MAX) {
		ufs_qcom_msg(ERR, host->hba->dev,
			"%s: UFS_CFG1[TEST_BUS_SEL} may not equal 0x%05X\n",
			__func__, host->testbus.select_major);
		return false;
	}

	return true;
}

/*
 * The caller of this function must make sure that the controller
 * is out of runtime suspend and appropriate clocks are enabled
 * before accessing.
 */
int ufs_qcom_testbus_config(struct ufs_qcom_host *host)
{
	int reg;
	int offset;
	u32 mask = TEST_BUS_SUB_SEL_MASK;
	unsigned long flags;
	struct ufs_hba *hba;

	if (!host)
		return -EINVAL;
	hba = host->hba;
	if (in_task())
		spin_lock_irqsave(hba->host->host_lock, flags);

	if (!ufs_qcom_testbus_cfg_is_ok(host))
		return -EPERM;

	switch (host->testbus.select_major) {
	case TSTBUS_UAWM:
		reg = UFS_TEST_BUS_CTRL_0;
		offset = 24;
		break;
	case TSTBUS_UARM:
		reg = UFS_TEST_BUS_CTRL_0;
		offset = 16;
		break;
	case TSTBUS_TXUC:
		reg = UFS_TEST_BUS_CTRL_0;
		offset = 8;
		break;
	case TSTBUS_RXUC:
		reg = UFS_TEST_BUS_CTRL_0;
		offset = 0;
		break;
	case TSTBUS_DFC:
		reg = UFS_TEST_BUS_CTRL_1;
		offset = 24;
		break;
	case TSTBUS_TRLUT:
		reg = UFS_TEST_BUS_CTRL_1;
		offset = 16;
		break;
	case TSTBUS_TMRLUT:
		reg = UFS_TEST_BUS_CTRL_1;
		offset = 8;
		break;
	case TSTBUS_OCSC:
		reg = UFS_TEST_BUS_CTRL_1;
		offset = 0;
		break;
	case TSTBUS_WRAPPER:
		reg = UFS_TEST_BUS_CTRL_2;
		offset = 16;
		break;
	case TSTBUS_COMBINED:
		reg = UFS_TEST_BUS_CTRL_2;
		offset = 8;
		break;
	case TSTBUS_UTP_HCI:
		reg = UFS_TEST_BUS_CTRL_2;
		offset = 0;
		break;
	case TSTBUS_UNIPRO:
		reg = UFS_UNIPRO_CFG;
		offset = 20;
		mask = 0xFFF;
		break;
	/*
	 * No need for a default case, since
	 * ufs_qcom_testbus_cfg_is_ok() checks that the configuration
	 * is legal
	 */
	}
	mask <<= offset;

	if (in_task())
		spin_unlock_irqrestore(hba->host->host_lock, flags);

	ufshcd_rmwl(host->hba, TEST_BUS_SEL,
		    (u32)host->testbus.select_major << 19,
		    REG_UFS_CFG1);
	ufshcd_rmwl(host->hba, mask,
		    (u32)host->testbus.select_minor << offset,
		    reg);
	ufs_qcom_enable_test_bus(host);
	/*
	 * Make sure the test bus configuration is
	 * committed before returning.
	 */
	mb();

	return 0;
}

static void ufs_qcom_testbus_read(struct ufs_hba *hba)
{
	ufshcd_dump_regs(hba, UFS_TEST_BUS, 4, "UFS_TEST_BUS ");
}

static void ufs_qcom_print_unipro_testbus(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	u32 *testbus = NULL;
	int i, nminor = 256, testbus_len = nminor * sizeof(u32);

	testbus = kmalloc(testbus_len, GFP_KERNEL);
	if (!testbus)
		return;

	host->testbus.select_major = TSTBUS_UNIPRO;
	for (i = 0; i < nminor; i++) {
		host->testbus.select_minor = i;
		ufs_qcom_testbus_config(host);
		testbus[i] = ufshcd_readl(hba, UFS_TEST_BUS);
	}
	print_hex_dump(KERN_ERR, "UNIPRO_TEST_BUS ", DUMP_PREFIX_OFFSET,
			16, 4, testbus, testbus_len, false);
	kfree(testbus);
}

static void ufs_qcom_print_utp_hci_testbus(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	u32 *testbus = NULL;
	int i, nminor = 32, testbus_len = nminor * sizeof(u32);

	testbus = kmalloc(testbus_len, GFP_KERNEL);
	if (!testbus)
		return;

	host->testbus.select_major = TSTBUS_UTP_HCI;
	for (i = 0; i < nminor; i++) {
		host->testbus.select_minor = i;
		ufs_qcom_testbus_config(host);
		testbus[i] = ufshcd_readl(hba, UFS_TEST_BUS);
	}
	print_hex_dump(KERN_ERR, "UTP_HCI_TEST_BUS ", DUMP_PREFIX_OFFSET,
			16, 4, testbus, testbus_len, false);
	kfree(testbus);
}

static void ufs_qcom_dump_dbg_regs(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	struct phy *phy = host->generic_phy;

	host->err_occurred = true;

	ufshcd_dump_regs(hba, REG_UFS_SYS1CLK_1US, 16 * 4,
			 "HCI Vendor Specific Registers ");

	ufshcd_dump_regs(hba, UFS_MEM_ICE, 29 * 4,
			 "HCI Shared ICE Registers ");

	/* sleep a bit intermittently as we are dumping too much data */
	ufs_qcom_print_hw_debug_reg_all(hba, NULL, ufs_qcom_dump_regs_wrapper);

	if (in_task()) {
		usleep_range(1000, 1100);
		ufs_qcom_testbus_read(hba);
		usleep_range(1000, 1100);
		ufs_qcom_print_unipro_testbus(hba);
		usleep_range(1000, 1100);
		ufs_qcom_print_utp_hci_testbus(hba);
		usleep_range(1000, 1100);
		ufs_qcom_phy_dbg_register_dump(phy);
	}

	BUG_ON(host->crash_on_err);
}

/*
 * ufs_qcom_parse_limits - read limits from DTS
 */
static void ufs_qcom_parse_limits(struct ufs_qcom_host *host)
{
	struct device_node *np = host->hba->dev->of_node;
	u32 val;
	u32 dev_major = 0, dev_minor = 0;

	if (!np)
		return;

	host->limit_tx_hs_gear = host->max_hs_gear;
	host->limit_rx_hs_gear = host->max_hs_gear;
	host->limit_tx_pwm_gear = UFS_QCOM_LIMIT_PWMGEAR_TX;
	host->limit_rx_pwm_gear = UFS_QCOM_LIMIT_PWMGEAR_RX;
	host->limit_rate = UFS_QCOM_LIMIT_HS_RATE;
	host->limit_phy_submode = UFS_QCOM_LIMIT_PHY_SUBMODE;

	/*
	 * The bootloader passes the on board device
	 * information to the HLOS using the UFS host controller register's
	 * UFS_MEM_DEBUG_SPARE_CFG Bit[0:3] = device's minor revision
	 * UFS_MEM_DEBUG_SPARE_CFG Bit[4:7] = device's major revision
	 * For example, UFS 3.1 devices would have a 0x31, and UFS 4.0 devices
	 * would have a 0x40 as the content of the mentioned register.
	 * If the bootloader does not support this feature, the default
	 * hardcoded setting would be used. The DT settings can be used to
	 * override any other gear's and Rate's settings.
	 */
	if (host->hw_ver.major >= 0x5) {
		val = ufshcd_readl(host->hba, REG_UFS_DEBUG_SPARE_CFG);
		dev_major = (val & UFS_DEVICE_VER_MAJOR_MASK) >>
				UFS_DEVICE_VER_MAJOR_SHFT;
		dev_minor = val & UFS_DEVICE_VER_MINOR_MASK;
	}

	if (host->hw_ver.major == 0x5 && dev_major == 0x4 && dev_minor == 0) {
		host->limit_rate = PA_HS_MODE_A;
		host->limit_phy_submode = UFS_QCOM_PHY_SUBMODE_G5;
	}

	of_property_read_u32(np, "limit-tx-hs-gear", &host->limit_tx_hs_gear);
	of_property_read_u32(np, "limit-rx-hs-gear", &host->limit_rx_hs_gear);
	of_property_read_u32(np, "limit-tx-pwm-gear", &host->limit_tx_pwm_gear);
	of_property_read_u32(np, "limit-rx-pwm-gear", &host->limit_rx_pwm_gear);
	of_property_read_u32(np, "limit-rate", &host->limit_rate);
	of_property_read_u32(np, "limit-phy-submode", &host->limit_phy_submode);

	if (of_property_read_bool(np, "limit-rate-ufs3"))
		ufs_qcom_set_rate_a(host);
}

/*
 * ufs_qcom_parse_g4_workaround_flag - read bypass-g4-cfgready entry from DT
 */
static void ufs_qcom_parse_g4_workaround_flag(struct ufs_qcom_host *host)
{
	struct device_node *np = host->hba->dev->of_node;
	const char *str  = "bypass-g4-cfgready";

	if (!np)
		return;

	host->bypass_g4_cfgready = of_property_read_bool(np, str);
}

/*
 * ufs_qcom_parse_lpm - read from DTS whether LPM modes should be disabled.
 */
static void ufs_qcom_parse_lpm(struct ufs_qcom_host *host)
{
	struct device_node *node = host->hba->dev->of_node;

	host->disable_lpm = of_property_read_bool(node, "qcom,disable-lpm");
	if (host->disable_lpm)
		ufs_qcom_msg(INFO, host->hba->dev, "(%s) All LPM is disabled\n",
			 __func__);
	of_property_read_u32(node, "qcom,vccq-lpm-uV", &host->vccq_lpm_uV);
}

/*
 * ufs_qcom_parse_wb - read from DTS whether WB support should be enabled.
 */
static void ufs_qcom_parse_wb(struct ufs_qcom_host *host)
{
	struct device_node *node = host->hba->dev->of_node;

	host->disable_wb_support = of_property_read_bool(node,
			"qcom,disable-wb-support");
	if (host->disable_wb_support)
		pr_info("%s: WB support disabled\n", __func__);
}

/**
 * ufs_qcom_device_reset() - toggle the (optional) device reset line
 * @hba: per-adapter instance
 *
 * Toggles the (optional) reset line to reset the attached device.
 */
static int ufs_qcom_device_reset(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	int ret = 0;

	/* Reset UFS Host Controller and PHY */
	ret = ufs_qcom_host_reset(hba);
	if (ret)
		ufs_qcom_msg(WARN, hba->dev, "%s: host reset returned %d\n",
				 __func__, ret);

	/* reset gpio is optional */
	if (!host->device_reset)
		return -EOPNOTSUPP;

	/*
	 * The UFS device shall detect reset pulses of 1us, sleep for 10us to
	 * be on the safe side.
	 */
	ufs_qcom_device_reset_ctrl(hba, true);
	usleep_range(10, 15);

	ufs_qcom_device_reset_ctrl(hba, false);
	usleep_range(10, 15);

	return 0;
}

#if IS_ENABLED(CONFIG_DEVFREQ_GOV_SIMPLE_ONDEMAND)
static void ufs_qcom_config_scaling_param(struct ufs_hba *hba,
					  struct devfreq_dev_profile *p,
					  void *data)
{
	static struct devfreq_simple_ondemand_data *d;

	if (!data)
		return;

	d = (struct devfreq_simple_ondemand_data *)data;
	p->polling_ms = 60;
	p->timer = DEVFREQ_TIMER_DELAYED;
	d->upthreshold = 70;
	d->downdifferential = 65;
}
#else
static void ufs_qcom_config_scaling_param(struct ufs_hba *hba,
					  struct devfreq_dev_profile *p,
					  void *data)
{
}
#endif

static struct ufs_dev_fix ufs_qcom_dev_fixups[] = {
	UFS_FIX(UFS_VENDOR_SAMSUNG, UFS_ANY_MODEL,
		UFS_DEVICE_QUIRK_PA_HIBER8TIME |
		UFS_DEVICE_QUIRK_PA_TX_HSG1_SYNC_LENGTH),
	UFS_FIX(UFS_VENDOR_MICRON, UFS_ANY_MODEL,
		UFS_DEVICE_QUIRK_DELAY_BEFORE_LPM),
	UFS_FIX(UFS_VENDOR_SKHYNIX, UFS_ANY_MODEL,
		UFS_DEVICE_QUIRK_DELAY_BEFORE_LPM),
	UFS_FIX(UFS_VENDOR_WDC, UFS_ANY_MODEL,
		UFS_DEVICE_QUIRK_HOST_PA_TACTIVATE),
	UFS_FIX(UFS_VENDOR_TOSHIBA, UFS_ANY_MODEL,
		UFS_DEVICE_QUIRK_DELAY_AFTER_LPM),
	END_FIX
};

static void ufs_qcom_fixup_dev_quirks(struct ufs_hba *hba)
{
	ufshcd_fixup_dev_quirks(hba, ufs_qcom_dev_fixups);
}

/*
 * struct ufs_hba_qcom_vops - UFS QCOM specific variant operations
 *
 * The variant operations configure the necessary controller and PHY
 * handshake during initialization.
 */
static const struct ufs_hba_variant_ops ufs_hba_qcom_vops = {
	.name                   = "qcom",
	.init                   = ufs_qcom_init,
	.exit                   = ufs_qcom_exit,
	.get_ufs_hci_version	= ufs_qcom_get_ufs_hci_version,
	.clk_scale_notify	= ufs_qcom_clk_scale_notify,
	.event_notify       = ufs_qcom_event_notify,
	.setup_clocks           = ufs_qcom_setup_clocks,
	.hce_enable_notify      = ufs_qcom_hce_enable_notify,
	.link_startup_notify    = ufs_qcom_link_startup_notify,
	.pwr_change_notify	= ufs_qcom_pwr_change_notify,
	.hibern8_notify		= ufs_qcom_hibern8_notify,
	.apply_dev_quirks	= ufs_qcom_apply_dev_quirks,
	.suspend		= ufs_qcom_suspend,
	.resume			= ufs_qcom_resume,
	.dbg_register_dump	= ufs_qcom_dump_dbg_regs,
	.device_reset		= ufs_qcom_device_reset,
	.config_scaling_param = ufs_qcom_config_scaling_param,
	.setup_xfer_req         = ufs_qcom_qos,
	.program_key		= ufs_qcom_ice_program_key,
	.fixup_dev_quirks       = ufs_qcom_fixup_dev_quirks,
};

/**
 * QCOM specific sysfs group and nodes
 */
static ssize_t err_state_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	return scnprintf(buf, PAGE_SIZE, "%d\n", !!host->err_occurred);
}

static DEVICE_ATTR_RO(err_state);

static ssize_t power_mode_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	static const char * const names[] = {
		"INVALID MODE",
		"FAST MODE",
		"SLOW MODE",
		"INVALID MODE",
		"FASTAUTO MODE",
		"SLOWAUTO MODE",
		"INVALID MODE",
	};

	/* Print current power info */
	return scnprintf(buf, PAGE_SIZE,
		"[Rx,Tx]: Gear[%d,%d], Lane[%d,%d], PWR[%s,%s], Rate-%c\n",
		hba->pwr_info.gear_rx, hba->pwr_info.gear_tx,
		hba->pwr_info.lane_rx, hba->pwr_info.lane_tx,
		names[hba->pwr_info.pwr_rx],
		names[hba->pwr_info.pwr_tx],
		hba->pwr_info.hs_rate == PA_HS_MODE_B ? 'B' : 'A');
}

static DEVICE_ATTR_RO(power_mode);

static ssize_t bus_speed_mode_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 !!atomic_read(&host->scale_up));
}

static DEVICE_ATTR_RO(bus_speed_mode);

static ssize_t clk_status_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 !!atomic_read(&host->clks_on));
}

static DEVICE_ATTR_RO(clk_status);

static unsigned int ufs_qcom_gec(struct ufs_hba *hba, u32 id,
				 char *err_name)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	unsigned long flags;
	int i;
	struct ufs_event_hist *e;

	if (id >= UFS_EVT_CNT)
		return -EINVAL;

	e = &hba->ufs_stats.event[id];

	spin_lock_irqsave(hba->host->host_lock, flags);
	for (i = 0; i < UFS_EVENT_HIST_LENGTH; i++) {
		int p = (i + e->pos) % UFS_EVENT_HIST_LENGTH;

		if (e->tstamp[p] == 0)
			continue;
		ufs_qcom_msg(ERR, hba->dev, "%s[%d] = 0x%x at %lld us\n", err_name, p,
			e->val[p], ktime_to_us(e->tstamp[p]));

	}

	spin_unlock_irqrestore(hba->host->host_lock, flags);
	return host->valid_evt_cnt[id];
}

static ssize_t err_count_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE,
			 "%s: %d\n%s: %d\n%s: %d\n%s: %d\n",
			 "pa_err_cnt_total",
			 ufs_qcom_gec(hba, UFS_EVT_PA_ERR,
				      "pa_err_cnt_total"),
			 "dl_err_cnt_total",
			 ufs_qcom_gec(hba, UFS_EVT_DL_ERR,
				      "dl_err_cnt_total"),
			 "dme_err_cnt",
			 ufs_qcom_gec(hba, UFS_EVT_DME_ERR,
				      "dme_err_cnt"),
			 "req_abort_cnt",
			  hba->req_abort_count);

}

static DEVICE_ATTR_RO(err_count);

static ssize_t dbg_state_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	bool v;

	if (!capable(CAP_SYS_ADMIN))
		return -EACCES;

	if (kstrtobool(buf, &v))
		return -EINVAL;

	host->dbg_en = v;

	return count;
}

static ssize_t dbg_state_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	return scnprintf(buf, PAGE_SIZE, "%d\n", host->dbg_en);
}


static DEVICE_ATTR_RW(dbg_state);

static ssize_t crash_on_err_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	return scnprintf(buf, PAGE_SIZE, "%d\n", !!host->crash_on_err);
}

static ssize_t crash_on_err_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	bool v;

	if (!capable(CAP_SYS_ADMIN))
		return -EACCES;

	if (kstrtobool(buf, &v))
		return -EINVAL;

	host->crash_on_err = v;

	return count;
}


static DEVICE_ATTR_RW(crash_on_err);

static ssize_t hibern8_count_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	u32 hw_h8_enter;
	u32 sw_h8_enter;
	u32 sw_hw_h8_enter;
	u32 hw_h8_exit;
	u32	sw_h8_exit;
	struct ufs_hba *hba = dev_get_drvdata(dev);

	pm_runtime_get_sync(hba->dev);
	ufshcd_hold(hba, false);
	hw_h8_enter = ufshcd_readl(hba, REG_UFS_HW_H8_ENTER_CNT);
	sw_h8_enter = ufshcd_readl(hba, REG_UFS_SW_H8_ENTER_CNT);
	sw_hw_h8_enter = ufshcd_readl(hba, REG_UFS_SW_AFTER_HW_H8_ENTER_CNT);
	hw_h8_exit = ufshcd_readl(hba, REG_UFS_HW_H8_EXIT_CNT);
	sw_h8_exit = ufshcd_readl(hba, REG_UFS_SW_H8_EXIT_CNT);
	ufshcd_release(hba);
	pm_runtime_put_sync(hba->dev);

	return scnprintf(buf, PAGE_SIZE,
			 "%s: %d\n%s: %d\n%s: %d\n%s: %d\n%s: %d\n",
			 "hw_h8_enter", hw_h8_enter,
			 "sw_h8_enter", sw_h8_enter,
			 "sw_after_hw_h8_enter", sw_hw_h8_enter,
			 "hw_h8_exit", hw_h8_exit,
			 "sw_h8_exit", sw_h8_exit);
}

static DEVICE_ATTR_RO(hibern8_count);

static ssize_t ber_th_exceeded_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	return scnprintf(buf, PAGE_SIZE, "%d\n", host->ber_th_exceeded);
}

static DEVICE_ATTR_RO(ber_th_exceeded);

static ssize_t irq_affinity_support_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	bool value;

	if (!capable(CAP_SYS_ADMIN))
		return -EACCES;

	if (kstrtobool(buf, &value))
		return -EINVAL;

	/* if current irq_affinity_support is same as requested one, don't proceed */
	if (value == host->irq_affinity_support)
		goto out;

	/* if perf-mask is not set, don't proceed */
	if (!host->perf_mask.bits[0])
		goto out;

	host->irq_affinity_support = !!value;
	/* Reset cpu affinity accordingly */
	if (host->irq_affinity_support)
		ufs_qcom_set_affinity_hint(hba, true);
	else
		ufs_qcom_set_affinity_hint(hba, false);

	return count;

out:
	return -EINVAL;
}

static ssize_t irq_affinity_support_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	return scnprintf(buf, PAGE_SIZE, "%d\n", !!host->irq_affinity_support);
}

static DEVICE_ATTR_RW(irq_affinity_support);

static ssize_t ufs_pm_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = -1;
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	switch (host->ufs_pm_mode) {
	case 0:
		ret = scnprintf(buf, 6, "NONE\n");
		break;
	case 1:
		ret = scnprintf(buf, 5, "S2R\n");
		break;
	case 2:
		ret = scnprintf(buf, 12, "DEEPSLEEP\n");
		break;
	default:
		break;
	}

	return ret;
}

static ssize_t ufs_pm_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	char kbuff[12] = {0};

	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	if (!buf)
		return -EINVAL;

	strscpy(kbuff, buf, 11);

	if (!strncasecmp(kbuff, "NONE", 4))
		host->ufs_pm_mode = 0;
	else if (!strncasecmp(kbuff, "S2R", 3))
		host->ufs_pm_mode = 1;
	else if (!strncasecmp(kbuff, "DEEPSLEEP", 9))
		host->ufs_pm_mode = 2;
	else
		dev_err(hba->dev, "Invalid entry for ufs_pm_mode\n");

	return count;
}


static DEVICE_ATTR_RW(ufs_pm_mode);

static struct attribute *ufs_qcom_sysfs_attrs[] = {
	&dev_attr_err_state.attr,
	&dev_attr_power_mode.attr,
	&dev_attr_bus_speed_mode.attr,
	&dev_attr_clk_status.attr,
	&dev_attr_err_count.attr,
	&dev_attr_dbg_state.attr,
	&dev_attr_crash_on_err.attr,
	&dev_attr_hibern8_count.attr,
	&dev_attr_ber_th_exceeded.attr,
	&dev_attr_irq_affinity_support.attr,
	&dev_attr_ufs_pm_mode.attr,
	NULL
};

static const struct attribute_group ufs_qcom_sysfs_group = {
	.name = "qcom",
	.attrs = ufs_qcom_sysfs_attrs,
};

static int ufs_qcom_init_sysfs(struct ufs_hba *hba)
{
	int ret;

	ret = sysfs_create_group(&hba->dev->kobj, &ufs_qcom_sysfs_group);
	if (ret)
		ufs_qcom_msg(ERR, hba->dev, "%s: Failed to create qcom sysfs group (err = %d)\n",
				 __func__, ret);

	return ret;
}

static void ufs_qcom_hook_send_command(void *param, struct ufs_hba *hba,
				       struct ufshcd_lrb *lrbp)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	if (lrbp && lrbp->cmd && lrbp->cmd->cmnd[0]) {
		struct request *rq = scsi_cmd_to_rq(lrbp->cmd);
		int sz = rq ? blk_rq_sectors(rq) : 0;
		ufs_qcom_log_str(host, "<,%x,%d,%x,%d\n",
				lrbp->cmd->cmnd[0],
				lrbp->task_tag,
				ufshcd_readl(hba,
					REG_UTP_TRANSFER_REQ_DOOR_BELL),
				sz);
		if (host->dbg_en)
			trace_ufs_qcom_command(dev_name(hba->dev), UFS_QCOM_CMD_SEND,
					lrbp->cmd->cmnd[0],
					lrbp->task_tag,
					ufshcd_readl(hba,
						REG_UTP_TRANSFER_REQ_DOOR_BELL),
					sz);
		if ((host->irq_affinity_support) && atomic_read(&host->hi_pri_en) && rq)
			rq->cmd_flags |= REQ_HIPRI;
	}
}

static void ufs_qcom_hook_compl_command(void *param, struct ufs_hba *hba,
				       struct ufshcd_lrb *lrbp)
{

	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	if (lrbp && lrbp->cmd) {
		int sz = scsi_cmd_to_rq(lrbp->cmd) ?
				blk_rq_sectors(scsi_cmd_to_rq(lrbp->cmd)) : 0;
		ufs_qcom_log_str(host, ">,%x,%d,%x,%d\n",
				lrbp->cmd->cmnd[0],
				lrbp->task_tag,
				ufshcd_readl(hba,
					REG_UTP_TRANSFER_REQ_DOOR_BELL),
				sz);
		if (host->dbg_en)
			trace_ufs_qcom_command(dev_name(hba->dev), UFS_QCOM_CMD_COMPL,
					lrbp->cmd->cmnd[0],
					lrbp->task_tag,
					ufshcd_readl(hba,
						REG_UTP_TRANSFER_REQ_DOOR_BELL),
					sz);
	}
}

static void ufs_qcom_hook_send_uic_command(void *param, struct ufs_hba *hba,
					struct uic_command *ucmd,
					int str_t)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	unsigned int a, b, c, cmd;
	char ch;

	if (str_t == UFS_CMD_SEND) {
		a = ucmd->argument1;
		b = ucmd->argument2;
		c = ucmd->argument3;
		cmd = ucmd->command;
		ch = '(';
		if (host->dbg_en)
			trace_ufs_qcom_uic(dev_name(hba->dev), UFS_QCOM_CMD_SEND,
					cmd, a, b, c);
	} else {
		a = ufshcd_readl(hba, REG_UIC_COMMAND_ARG_1),
		b = ufshcd_readl(hba, REG_UIC_COMMAND_ARG_2),
		c = ufshcd_readl(hba, REG_UIC_COMMAND_ARG_3);
		cmd = ufshcd_readl(hba, REG_UIC_COMMAND);
		ch = ')';
		if (host->dbg_en)
			trace_ufs_qcom_uic(dev_name(hba->dev), UFS_QCOM_CMD_COMPL,
					cmd, a, b, c);
	}
	ufs_qcom_log_str(host, "%c,%x,%x,%x,%x\n",
				ch, cmd, a, b, c);
}

static void ufs_qcom_hook_send_tm_command(void *param, struct ufs_hba *hba,
					int tag, int str)
{

}

static void ufs_qcom_hook_check_int_errors(void *param, struct ufs_hba *hba,
					bool queue_eh_work)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);

	if (queue_eh_work) {
		ufs_qcom_log_str(host, "_,%x,%x\n",
					hba->errors, hba->uic_error);
		if (host->dbg_en)
			trace_ufs_qcom_hook_check_int_errors(dev_name(hba->dev), hba->errors,
						hba->uic_error);
	}
}

static void ufs_qcom_update_sdev(void *param, struct scsi_device *sdev)
{
	sdev->broken_fua = 1;
}

static void ufs_qcom_hook_prepare_command(void *param, struct ufs_hba *hba,
					   struct request *rq, struct ufshcd_lrb *lrbp, int *err)
{
#if IS_ENABLED(CONFIG_QTI_CRYPTO_FDE)
	struct ice_data_setting setting;

	if (!rq->crypt_keyslot) {
		if (!crypto_qti_ice_config_start(rq, &setting)) {
			if ((rq_data_dir(rq) == WRITE) ? setting.encr_bypass :
					setting.decr_bypass) {
				lrbp->crypto_key_slot = -1;
			} else {
				lrbp->crypto_key_slot = setting.crypto_data.key_index;
				lrbp->data_unit_num = rq->bio->bi_iter.bi_sector >>
						      ICE_CRYPTO_DATA_UNIT_4_KB;
			}
		}
	} else {
		lrbp->crypto_key_slot = blk_ksm_get_slot_idx(rq->crypt_keyslot);
		lrbp->data_unit_num = rq->crypt_ctx->bc_dun[0];
	}
#endif
}

/*
 * Refer: common/include/trace/hooks/ufshcd.h for available hooks
 */
static void ufs_qcom_register_hooks(void)
{
	register_trace_android_vh_ufs_send_command(ufs_qcom_hook_send_command,
						NULL);
	register_trace_android_vh_ufs_compl_command(
				ufs_qcom_hook_compl_command, NULL);
	register_trace_android_vh_ufs_send_uic_command(
				ufs_qcom_hook_send_uic_command, NULL);
	register_trace_android_vh_ufs_send_tm_command(
				ufs_qcom_hook_send_tm_command, NULL);
	register_trace_android_vh_ufs_check_int_errors(
				ufs_qcom_hook_check_int_errors, NULL);
	register_trace_android_vh_ufs_update_sdev(ufs_qcom_update_sdev, NULL);
	register_trace_android_vh_ufs_prepare_command(
				ufs_qcom_hook_prepare_command, NULL);
}

#ifdef CONFIG_ARM_QCOM_CPUFREQ_HW
static int ufs_cpufreq_status(void)
{
	struct cpufreq_policy *policy;

	policy = cpufreq_cpu_get(0);
	if (!policy) {
		ufs_qcom_msg(WARN, NULL, "cpufreq not probed yet, defer once\n");
		return -EPROBE_DEFER;
	}

	cpufreq_cpu_put(policy);

	return 0;
}
#else
static int ufs_cpufreq_status(void)
{
	return 0;
}
#endif

static u32 is_bootdevice_ufs = UFS_BOOT_DEVICE;

static int ufs_qcom_read_boot_config(struct platform_device *pdev)
{
	u8 *buf;
	size_t len;
	struct nvmem_cell *cell;

	cell = nvmem_cell_get(&pdev->dev, "boot_conf");
	if (IS_ERR(cell))
		return -EINVAL;

	buf = (u8 *)nvmem_cell_read(cell, &len);
	if (IS_ERR(buf))
		return -EINVAL;

	is_bootdevice_ufs = (*buf) >> 1 & 0x1f;
	dev_dbg(&pdev->dev, "boot_config val = %x is_bootdevice_ufs = %x\n",
			*buf, is_bootdevice_ufs);
	kfree(buf);
	nvmem_cell_put(cell);

	return is_bootdevice_ufs;
}

/**
 * ufs_qcom_probe - probe routine of the driver
 * @pdev: pointer to Platform device handle
 *
 * Return zero for success and non-zero for failure
 */
static int ufs_qcom_probe(struct platform_device *pdev)
{
	int err = 0;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;

	if (!ufs_qcom_read_boot_config(pdev)) {
		dev_err(dev, "UFS is not boot dev.\n");
		return err;
	}

	/**
	 * CPUFreq driver is needed for performance reasons.
	 * Assumption - cpufreq gets probed the second time.
	 * If cpufreq is not probed by the time the driver requests for cpu
	 * policy later on, cpufreq would be disabled and performance would be
	 * impacted adversly.
	 */
	err = ufs_cpufreq_status();
	if (err)
		return err;

	/*
	 * On qcom platforms, bootdevice is the primary storage
	 * device. This device can either be eMMC or UFS.
	 * The type of device connected is detected at runtime.
	 * So, if an eMMC device is connected, and this function
	 * is invoked, it would turn-off the regulator if it detects
	 * that the storage device is not ufs.
	 * These regulators are turned ON by the bootloaders & turning
	 * them off without sending PON may damage the connected device.
	 * Hence, check for the connected device early-on & don't turn-off
	 * the regulators.
	 */

	/*
	 *Defer secondary UFS device probe if all the LUNS of
	 *primary UFS boot device are not enumerated.
	 */
	if ((of_property_read_bool(np, "secondary-storage")) && (!ufs_qcom_hosts[0]
		|| !(ufs_qcom_hosts[0]->hba->luns_avail == 1))) {
		err = -EPROBE_DEFER;
		return err;
	}

	/* Perform generic probe */
	err = ufshcd_pltfrm_init(pdev, &ufs_hba_qcom_vops);
	if (err)
		ufs_qcom_msg(ERR, dev, "ufshcd_pltfrm_init() failed %d\n", err);

	if (!(of_property_read_bool(np, "secondary-storage")))
		ufs_qcom_register_hooks();

	return err;
}

/**
 * ufs_qcom_remove - set driver_data of the device to NULL
 * @pdev: pointer to platform device handle
 *
 * Always returns 0
 */
static int ufs_qcom_remove(struct platform_device *pdev)
{
	struct ufs_hba *hba;
	struct ufs_qcom_host *host;
	struct ufs_qcom_qos_req *r;
	struct qos_cpu_group *qcg;
	int i;

	if (!is_bootdevice_ufs) {
		dev_info(&pdev->dev, "UFS is not boot dev.\n");
		return 0;
	}

	hba =  platform_get_drvdata(pdev);
	host = ufshcd_get_variant(hba);
	r = host->ufs_qos;
	qcg = r->qcg;

	if (msm_minidump_enabled())
		atomic_notifier_chain_unregister(&panic_notifier_list,
				&host->ufs_qcom_panic_nb);

	pm_runtime_get_sync(&(pdev)->dev);
	for (i = 0; i < r->num_groups; i++, qcg++)
		remove_group_qos(qcg);

	ufshcd_remove(hba);
	return 0;
}

/**
 * ufs_qcom_enable_vccq_shutdown - read from DTS whether vccq_shutdown
 * should be enabled for puting additional vote on VCCQ LDO during shutdown.
 * @hba: per adapter instance
 */

static void ufs_qcom_enable_vccq_shutdown(struct ufs_hba *hba)
{
	struct ufs_qcom_host *host = ufshcd_get_variant(hba);
	int err;

	err = ufs_qcom_parse_reg_info(host, "qcom,vccq-shutdown",
			&host->vccq_shutdown);

	if (host->vccq_shutdown) {
		err = ufs_qcom_enable_vreg(hba->dev, host->vccq_shutdown);
		if (err)
			ufs_qcom_msg(ERR, hba->dev, "%s: failed enable vccq_shutdown err=%d\n",
						__func__, err);
	}
}

static void ufs_qcom_shutdown(struct platform_device *pdev)
{
	struct ufs_hba *hba;
	struct ufs_qcom_host *host;

	if (!is_bootdevice_ufs) {
		dev_info(&pdev->dev, "UFS is not boot dev.\n");
		return;
	}

	hba =  platform_get_drvdata(pdev);
	host = ufshcd_get_variant(hba);

	ufs_qcom_log_str(host, "0xdead\n");
	if (host->dbg_en)
		trace_ufs_qcom_shutdown(dev_name(hba->dev));

	/* put an additional vote on UFS VCCQ LDO if required */
	ufs_qcom_enable_vccq_shutdown(hba);

	ufshcd_pltfrm_shutdown(pdev);

	/* UFS_RESET TLMM register cannot reset to POR value '1' after warm
	 * reset, so deassert ufs device reset line after UFS device shutdown
	 * to ensure the UFS_RESET TLMM register value is POR value
	 */
	if (!host->bypass_pbl_rst_wa)
		ufs_qcom_device_reset_ctrl(hba, false);
}

static int ufs_qcom_system_suspend(struct device *dev)
{
	struct device_node *np = dev->of_node;

	if (of_property_read_bool(np, "non-removable") && !is_bootdevice_ufs) {
		dev_info(dev, "UFS is not boot dev.\n");
		return 0;
	}

	return ufshcd_system_suspend(dev);
}

static int ufs_qcom_system_resume(struct device *dev)
{
	if (!is_bootdevice_ufs) {
		dev_info(dev, "UFS is not boot dev.\n");
		return 0;
	}

	return ufshcd_system_resume(dev);
}

static int ufs_qcom_suspend_prepare(struct device *dev)
{
	struct ufs_hba *hba;
	struct ufs_qcom_host *host;

	if (!is_bootdevice_ufs) {
		dev_info(dev, "UFS is not boot dev.\n");
		return 0;
	}

	hba = dev_get_drvdata(dev);
	host = ufshcd_get_variant(hba);

	host->spm_lvl_prev = hba->spm_lvl;

	/*
	 * For deep sleep, if "set_ds_spm_level" flag is true, set the
	 * spm level to lvl 5 because all regulators is turned off in DS.
	 * For other scenarios like s2idle, retain the default spm level.
	 */
	switch (host->ufs_pm_mode) {
	case UFS_QCOM_SYSFS_NONE:
		if (host->set_ds_spm_level && (pm_suspend_target_state == PM_SUSPEND_MEM))
			hba->spm_lvl = UFS_PM_LVL_5;
		break;
	case UFS_QCOM_SYSFS_DEEPSLEEP:
		if (host->set_ds_spm_level)
			hba->spm_lvl = UFS_PM_LVL_5;
		break;
	case UFS_QCOM_SYSFS_S2R:
	default:
		break;
	}

	if (hba->spm_lvl != host->spm_lvl_prev)
		dev_info(dev, "spm level is changed from %d to %d\n",
			host->spm_lvl_prev, hba->spm_lvl);

	return ufshcd_suspend_prepare(dev);
}

static void ufs_qcom_resume_complete(struct device *dev)
{
	struct ufs_hba *hba;
	struct ufs_qcom_host *host;

	if (!is_bootdevice_ufs) {
		dev_info(dev, "UFS is not boot dev.\n");
		return;
	}

	hba = dev_get_drvdata(dev);
	host = ufshcd_get_variant(hba);

	if (host->set_ds_spm_level)
		hba->spm_lvl = host->spm_lvl_prev;

	host->ufs_pm_mode = UFS_QCOM_SYSFS_NONE;

	return ufshcd_resume_complete(dev);
}

static const struct of_device_id ufs_qcom_of_match[] = {
	{ .compatible = "qcom,ufshc"},
	{},
};
MODULE_DEVICE_TABLE(of, ufs_qcom_of_match);

#ifdef CONFIG_ACPI
static const struct acpi_device_id ufs_qcom_acpi_match[] = {
	{ "QCOM24A5" },
	{ },
};
MODULE_DEVICE_TABLE(acpi, ufs_qcom_acpi_match);
#endif

static const struct dev_pm_ops ufs_qcom_pm_ops = {
	SET_RUNTIME_PM_OPS(ufshcd_runtime_suspend, ufshcd_runtime_resume, NULL)
	.prepare	= ufs_qcom_suspend_prepare,
	.complete	= ufs_qcom_resume_complete,
#ifdef CONFIG_PM_SLEEP
	.suspend         = ufs_qcom_system_suspend,
	.resume          = ufs_qcom_system_resume,
	.freeze          = ufshcd_system_freeze,
	.restore         = ufshcd_system_restore,
	.thaw            = ufshcd_system_thaw,
#endif
};

static struct platform_driver ufs_qcom_pltform = {
	.probe	= ufs_qcom_probe,
	.remove	= ufs_qcom_remove,
	.shutdown = ufs_qcom_shutdown,
	.driver	= {
		.name	= "ufshcd-qcom",
		.pm	= &ufs_qcom_pm_ops,
		.of_match_table = of_match_ptr(ufs_qcom_of_match),
		.acpi_match_table = ACPI_PTR(ufs_qcom_acpi_match),
	},
};
module_platform_driver(ufs_qcom_pltform);

MODULE_LICENSE("GPL v2");

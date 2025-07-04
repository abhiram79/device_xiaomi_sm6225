// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2023, 2025, Qualcomm Innovation Center, Inc. All rights reserved.
 */

#define pr_fmt(fmt) "clk: %s: " fmt, __func__

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "clk-debug.h"
#include "common.h"

static struct measure_clk_data debug_mux_priv = {
	.ctl_reg = 0x62038,
	.status_reg = 0x6203C,
	.xo_div4_cbcr = 0x28008,
};

static const char *const apss_cc_debug_mux_parent_names[] = {
	"measure_only_perf_clk",
	"measure_only_pwclk_clk",
};

static int apss_cc_debug_mux_sels[] = {
	0x1,		/* measure_only_perf_clk */
	0x0,		/* measure_only_pwclk_clk */
};

static int apss_cc_debug_mux_pre_divs[] = {
	0x8,		/* measure_only_perf_clk */
	0x8,		/* measure_only_pwclk_clk */
};

static struct clk_debug_mux apss_cc_debug_mux = {
	.priv = &debug_mux_priv,
	.debug_offset = 0x0,
	.post_div_offset = 0x0,
	.cbcr_offset = U32_MAX,
	.src_sel_mask = 0x3FF00,
	.src_sel_shift = 8,
	.post_div_mask = 0xF0000000,
	.post_div_shift = 28,
	.post_div_val = 1,
	.mux_sels = apss_cc_debug_mux_sels,
	.num_mux_sels = ARRAY_SIZE(apss_cc_debug_mux_sels),
	.pre_div_vals = apss_cc_debug_mux_pre_divs,
	.hw.init = &(const struct clk_init_data){
		.name = "apss_cc_debug_mux",
		.ops = &clk_debug_mux_ops,
		.parent_names = apss_cc_debug_mux_parent_names,
		.num_parents = ARRAY_SIZE(apss_cc_debug_mux_parent_names),
	},
};

static const char *const disp_cc_debug_mux_parent_names[] = {
	"disp_cc_mdss_ahb_clk",
	"disp_cc_mdss_byte0_clk",
	"disp_cc_mdss_byte0_intf_clk",
	"disp_cc_mdss_esc0_clk",
	"disp_cc_mdss_mdp_clk",
	"disp_cc_mdss_mdp_lut_clk",
	"disp_cc_mdss_non_gdsc_ahb_clk",
	"disp_cc_mdss_pclk0_clk",
	"disp_cc_mdss_rot_clk",
	"disp_cc_mdss_rscc_ahb_clk",
	"disp_cc_mdss_rscc_vsync_clk",
	"disp_cc_mdss_vsync_clk",
	"measure_only_disp_cc_sleep_clk",
	"measure_only_disp_cc_xo_clk",
};

static int disp_cc_debug_mux_sels[] = {
	0x14,		/* disp_cc_mdss_ahb_clk */
	0xC,		/* disp_cc_mdss_byte0_clk */
	0xD,		/* disp_cc_mdss_byte0_intf_clk */
	0xE,		/* disp_cc_mdss_esc0_clk */
	0x8,		/* disp_cc_mdss_mdp_clk */
	0xA,		/* disp_cc_mdss_mdp_lut_clk */
	0x15,		/* disp_cc_mdss_non_gdsc_ahb_clk */
	0x7,		/* disp_cc_mdss_pclk0_clk */
	0x9,		/* disp_cc_mdss_rot_clk */
	0x17,		/* disp_cc_mdss_rscc_ahb_clk */
	0x16,		/* disp_cc_mdss_rscc_vsync_clk */
	0xB,		/* disp_cc_mdss_vsync_clk */
	0x1D,		/* measure_only_disp_cc_sleep_clk */
	0x1E,		/* measure_only_disp_cc_xo_clk */
};

static struct clk_debug_mux disp_cc_debug_mux = {
	.priv = &debug_mux_priv,
	.debug_offset = 0x7000,
	.post_div_offset = 0x3000,
	.cbcr_offset = 0x3004,
	.src_sel_mask = 0xFF,
	.src_sel_shift = 0,
	.post_div_mask = 0xF,
	.post_div_shift = 0,
	.post_div_val = 4,
	.mux_sels = disp_cc_debug_mux_sels,
	.num_mux_sels = ARRAY_SIZE(disp_cc_debug_mux_sels),
	.hw.init = &(const struct clk_init_data){
		.name = "disp_cc_debug_mux",
		.ops = &clk_debug_mux_ops,
		.parent_names = disp_cc_debug_mux_parent_names,
		.num_parents = ARRAY_SIZE(disp_cc_debug_mux_parent_names),
	},
};

static const char *const gcc_debug_mux_parent_names[] = {
	"apss_cc_debug_mux",
	"disp_cc_debug_mux",
	"gcc_ahb2phy_csi_clk",
	"gcc_ahb2phy_usb_clk",
	"gcc_apc_vs_clk",
	"gcc_bimc_gpu_axi_clk",
	"gcc_boot_rom_ahb_clk",
	"gcc_cam_throttle_nrt_clk",
	"gcc_cam_throttle_rt_clk",
	"gcc_camera_ahb_clk",
	"gcc_camss_axi_clk",
	"gcc_camss_camnoc_atb_clk",
	"gcc_camss_camnoc_nts_xo_clk",
	"gcc_camss_cci_0_clk",
	"gcc_camss_cphy_0_clk",
	"gcc_camss_cphy_1_clk",
	"gcc_camss_cphy_2_clk",
	"gcc_camss_csi0phytimer_clk",
	"gcc_camss_csi1phytimer_clk",
	"gcc_camss_csi2phytimer_clk",
	"gcc_camss_mclk0_clk",
	"gcc_camss_mclk1_clk",
	"gcc_camss_mclk2_clk",
	"gcc_camss_mclk3_clk",
	"gcc_camss_nrt_axi_clk",
	"gcc_camss_ope_ahb_clk",
	"gcc_camss_ope_clk",
	"gcc_camss_rt_axi_clk",
	"gcc_camss_tfe_0_clk",
	"gcc_camss_tfe_0_cphy_rx_clk",
	"gcc_camss_tfe_0_csid_clk",
	"gcc_camss_tfe_1_clk",
	"gcc_camss_tfe_1_cphy_rx_clk",
	"gcc_camss_tfe_1_csid_clk",
	"gcc_camss_tfe_2_clk",
	"gcc_camss_tfe_2_cphy_rx_clk",
	"gcc_camss_tfe_2_csid_clk",
	"gcc_camss_top_ahb_clk",
	"gcc_cfg_noc_usb3_prim_axi_clk",
	"gcc_disp_ahb_clk",
	"gcc_disp_gpll0_div_clk_src",
	"gcc_disp_hf_axi_clk",
	"gcc_disp_sleep_clk",
	"gcc_disp_throttle_core_clk",
	"gcc_gp1_clk",
	"gcc_gp2_clk",
	"gcc_gp3_clk",
	"gcc_gpu_gpll0_clk_src",
	"gcc_gpu_gpll0_div_clk_src",
	"gcc_gpu_memnoc_gfx_clk",
	"gcc_gpu_snoc_dvm_gfx_clk",
	"gcc_gpu_throttle_core_clk",
	"gcc_mss_vs_clk",
	"gcc_pdm2_clk",
	"gcc_pdm_ahb_clk",
	"gcc_pdm_xo4_clk",
	"gcc_prng_ahb_clk",
	"gcc_qmip_camera_nrt_ahb_clk",
	"gcc_qmip_camera_rt_ahb_clk",
	"gcc_qmip_disp_ahb_clk",
	"gcc_qmip_gpu_cfg_ahb_clk",
	"gcc_qmip_video_vcodec_ahb_clk",
	"gcc_qupv3_wrap0_core_2x_clk",
	"gcc_qupv3_wrap0_core_clk",
	"gcc_qupv3_wrap0_s0_clk",
	"gcc_qupv3_wrap0_s1_clk",
	"gcc_qupv3_wrap0_s2_clk",
	"gcc_qupv3_wrap0_s3_clk",
	"gcc_qupv3_wrap0_s4_clk",
	"gcc_qupv3_wrap0_s5_clk",
	"gcc_qupv3_wrap_0_m_ahb_clk",
	"gcc_qupv3_wrap_0_s_ahb_clk",
	"gcc_sdcc1_ahb_clk",
	"gcc_sdcc1_apps_clk",
	"gcc_sdcc1_ice_core_clk",
	"gcc_sdcc2_ahb_clk",
	"gcc_sdcc2_apps_clk",
	"gcc_sys_noc_cpuss_ahb_clk",
	"gcc_sys_noc_ufs_phy_axi_clk",
	"gcc_sys_noc_usb3_prim_axi_clk",
	"gcc_ufs_phy_ahb_clk",
	"gcc_ufs_phy_axi_clk",
	"gcc_ufs_phy_ice_core_clk",
	"gcc_ufs_phy_phy_aux_clk",
	"gcc_ufs_phy_rx_symbol_0_clk",
	"gcc_ufs_phy_rx_symbol_1_clk",
	"gcc_ufs_phy_tx_symbol_0_clk",
	"gcc_ufs_phy_unipro_core_clk",
	"gcc_usb30_prim_master_clk",
	"gcc_usb30_prim_mock_utmi_clk",
	"gcc_usb30_prim_sleep_clk",
	"gcc_usb3_prim_phy_com_aux_clk",
	"gcc_usb3_prim_phy_pipe_clk",
	"gcc_vcodec0_axi_clk",
	"gcc_vdda_vs_clk",
	"gcc_vddcx_vs_clk",
	"gcc_vddmx_vs_clk",
	"gcc_venus_ahb_clk",
	"gcc_venus_ctl_axi_clk",
	"gcc_video_ahb_clk",
	"gcc_video_axi0_clk",
	"gcc_video_throttle_core_clk",
	"gcc_video_vcodec0_sys_clk",
	"gcc_video_venus_ctl_clk",
	"gcc_video_xo_clk",
	"gcc_vs_ctrl_ahb_clk",
	"gcc_vs_ctrl_clk",
	"gcc_wcss_vs_clk",
	"gpu_cc_debug_mux",
	"lpass_aon_cc_debug_mux",
	"measure_only_cnoc_clk",
	"measure_only_gcc_camera_xo_clk",
	"measure_only_gcc_cpuss_gnoc_clk",
	"measure_only_gcc_disp_xo_clk",
	"measure_only_gcc_gpu_cfg_ahb_clk",
	"measure_only_gcc_lpass_sway_clk",
	"measure_only_ipa_2x_clk",
	"mc_cc_debug_mux",
	"measure_only_snoc_clk",
	"measure_only_usb3_phy_wrapper_gcc_usb30_pipe_clk",
	"ufs_phy_rx_symbol_0_clk",
	"ufs_phy_rx_symbol_1_clk",
	"ufs_phy_tx_symbol_0_clk",
};

static int gcc_debug_mux_sels[] = {
	0xB6,		/* apss_cc_debug_mux */
	0x45,		/* disp_cc_debug_mux */
	0x6A,		/* gcc_ahb2phy_csi_clk */
	0x6B,		/* gcc_ahb2phy_usb_clk */
	0xCA,		/* gcc_apc_vs_clk */
	0x97,		/* gcc_bimc_gpu_axi_clk */
	0x7D,		/* gcc_boot_rom_ahb_clk */
	0x4F,		/* gcc_cam_throttle_nrt_clk */
	0x4E,		/* gcc_cam_throttle_rt_clk */
	0x3A,		/* gcc_camera_ahb_clk */
	0x141,		/* gcc_camss_axi_clk */
	0x143,		/* gcc_camss_camnoc_atb_clk */
	0x144,		/* gcc_camss_camnoc_nts_xo_clk */
	0x13F,		/* gcc_camss_cci_0_clk */
	0x130,		/* gcc_camss_cphy_0_clk */
	0x131,		/* gcc_camss_cphy_1_clk */
	0x132,		/* gcc_camss_cphy_2_clk */
	0x122,		/* gcc_camss_csi0phytimer_clk */
	0x123,		/* gcc_camss_csi1phytimer_clk */
	0x124,		/* gcc_camss_csi2phytimer_clk */
	0x125,		/* gcc_camss_mclk0_clk */
	0x126,		/* gcc_camss_mclk1_clk */
	0x127,		/* gcc_camss_mclk2_clk */
	0x128,		/* gcc_camss_mclk3_clk */
	0x145,		/* gcc_camss_nrt_axi_clk */
	0x13E,		/* gcc_camss_ope_ahb_clk */
	0x13C,		/* gcc_camss_ope_clk */
	0x147,		/* gcc_camss_rt_axi_clk */
	0x129,		/* gcc_camss_tfe_0_clk */
	0x12D,		/* gcc_camss_tfe_0_cphy_rx_clk */
	0x133,		/* gcc_camss_tfe_0_csid_clk */
	0x12A,		/* gcc_camss_tfe_1_clk */
	0x12E,		/* gcc_camss_tfe_1_cphy_rx_clk */
	0x135,		/* gcc_camss_tfe_1_csid_clk */
	0x12B,		/* gcc_camss_tfe_2_clk */
	0x12F,		/* gcc_camss_tfe_2_cphy_rx_clk */
	0x137,		/* gcc_camss_tfe_2_csid_clk */
	0x140,		/* gcc_camss_top_ahb_clk */
	0x1E,		/* gcc_cfg_noc_usb3_prim_axi_clk */
	0x3B,		/* gcc_disp_ahb_clk */
	0x4A,		/* gcc_disp_gpll0_div_clk_src */
	0x40,		/* gcc_disp_hf_axi_clk */
	0x50,		/* gcc_disp_sleep_clk */
	0x4C,		/* gcc_disp_throttle_core_clk */
	0xC1,		/* gcc_gp1_clk */
	0xC2,		/* gcc_gp2_clk */
	0xC3,		/* gcc_gp3_clk */
	0xF1,		/* gcc_gpu_gpll0_clk_src */
	0xF2,		/* gcc_gpu_gpll0_div_clk_src */
	0xEE,		/* gcc_gpu_memnoc_gfx_clk */
	0xF0,		/* gcc_gpu_snoc_dvm_gfx_clk */
	0xF5,		/* gcc_gpu_throttle_core_clk */
	0xC9,		/* gcc_mss_vs_clk */
	0x7A,		/* gcc_pdm2_clk */
	0x78,		/* gcc_pdm_ahb_clk */
	0x79,		/* gcc_pdm_xo4_clk */
	0x7B,		/* gcc_prng_ahb_clk */
	0x3D,		/* gcc_qmip_camera_nrt_ahb_clk */
	0x4B,		/* gcc_qmip_camera_rt_ahb_clk */
	0x3E,		/* gcc_qmip_disp_ahb_clk */
	0xF3,		/* gcc_qmip_gpu_cfg_ahb_clk */
	0x3C,		/* gcc_qmip_video_vcodec_ahb_clk */
	0x71,		/* gcc_qupv3_wrap0_core_2x_clk */
	0x70,		/* gcc_qupv3_wrap0_core_clk */
	0x72,		/* gcc_qupv3_wrap0_s0_clk */
	0x73,		/* gcc_qupv3_wrap0_s1_clk */
	0x74,		/* gcc_qupv3_wrap0_s2_clk */
	0x75,		/* gcc_qupv3_wrap0_s3_clk */
	0x76,		/* gcc_qupv3_wrap0_s4_clk */
	0x77,		/* gcc_qupv3_wrap0_s5_clk */
	0x6E,		/* gcc_qupv3_wrap_0_m_ahb_clk */
	0x6F,		/* gcc_qupv3_wrap_0_s_ahb_clk */
	0xF9,		/* gcc_sdcc1_ahb_clk */
	0xF8,		/* gcc_sdcc1_apps_clk */
	0xFA,		/* gcc_sdcc1_ice_core_clk */
	0x6D,		/* gcc_sdcc2_ahb_clk */
	0x6C,		/* gcc_sdcc2_apps_clk */
	0x9,		/* gcc_sys_noc_cpuss_ahb_clk */
	0x19,		/* gcc_sys_noc_ufs_phy_axi_clk */
	0x18,		/* gcc_sys_noc_usb3_prim_axi_clk */
	0x117,		/* gcc_ufs_phy_ahb_clk */
	0x116,		/* gcc_ufs_phy_axi_clk */
	0x11D,		/* gcc_ufs_phy_ice_core_clk */
	0x11E,		/* gcc_ufs_phy_phy_aux_clk */
	0x119,		/* gcc_ufs_phy_rx_symbol_0_clk */
	0x121,		/* gcc_ufs_phy_rx_symbol_1_clk */
	0x118,		/* gcc_ufs_phy_tx_symbol_0_clk */
	0x11C,		/* gcc_ufs_phy_unipro_core_clk */
	0x60,		/* gcc_usb30_prim_master_clk */
	0x62,		/* gcc_usb30_prim_mock_utmi_clk */
	0x61,		/* gcc_usb30_prim_sleep_clk */
	0x63,		/* gcc_usb3_prim_phy_com_aux_clk */
	0x64,		/* gcc_usb3_prim_phy_pipe_clk */
	0x14D,		/* gcc_vcodec0_axi_clk */
	0xC6,		/* gcc_vdda_vs_clk */
	0xC4,		/* gcc_vddcx_vs_clk */
	0xC5,		/* gcc_vddmx_vs_clk */
	0x14E,		/* gcc_venus_ahb_clk */
	0x14C,		/* gcc_venus_ctl_axi_clk */
	0x39,		/* gcc_video_ahb_clk */
	0x3F,		/* gcc_video_axi0_clk */
	0x4D,		/* gcc_video_throttle_core_clk */
	0x14A,		/* gcc_video_vcodec0_sys_clk */
	0x148,		/* gcc_video_venus_ctl_clk */
	0x41,		/* gcc_video_xo_clk */
	0xC8,		/* gcc_vs_ctrl_ahb_clk */
	0xC7,		/* gcc_vs_ctrl_clk */
	0xCB,		/* gcc_wcss_vs_clk */
	0xED,		/* gpu_cc_debug_mux */
	0xFF,		/* lpass_aon_cc_debug_mux */
	0x1C,		/* measure_only_cnoc_clk */
	0x42,		/* measure_only_gcc_camera_xo_clk */
	0xB1,		/* measure_only_gcc_cpuss_gnoc_clk */
	0x43,		/* measure_only_gcc_disp_xo_clk */
	0xEB,		/* measure_only_gcc_gpu_cfg_ahb_clk */
	0xFD,		/* measure_only_gcc_lpass_sway_clk */
	0xCD,		/* measure_only_ipa_2x_clk */
	0xA5,		/* mc_cc_debug_mux or ddrss_gcc_debug_clk */
	0x7,		/* measure_only_snoc_clk */
	0x68,		/* measure_only_usb3_phy_wrapper_gcc_usb30_pipe_clk */
	0x11B,		/* ufs_phy_rx_symbol_0_clk */
	0x120,		/* ufs_phy_rx_symbol_1_clk */
	0x11A,		/* ufs_phy_tx_symbol_0_clk */
};

static struct clk_debug_mux gcc_debug_mux = {
	.priv = &debug_mux_priv,
	.debug_offset = 0x62000,
	.post_div_offset = 0x30000,
	.cbcr_offset = 0x30004,
	.src_sel_mask = 0x3FF,
	.src_sel_shift = 0,
	.post_div_mask = 0xF,
	.post_div_shift = 0,
	.post_div_val = 1,
	.mux_sels = gcc_debug_mux_sels,
	.num_mux_sels = ARRAY_SIZE(gcc_debug_mux_sels),
	.hw.init = &(const struct clk_init_data){
		.name = "gcc_debug_mux",
		.ops = &clk_debug_mux_ops,
		.parent_names = gcc_debug_mux_parent_names,
		.num_parents = ARRAY_SIZE(gcc_debug_mux_parent_names),
	},
};

static const char *const gpu_cc_debug_mux_parent_names[] = {
	"gpu_cc_ahb_clk",
	"gpu_cc_crc_ahb_clk",
	"gpu_cc_cx_gfx3d_clk",
	"gpu_cc_cx_gfx3d_slv_clk",
	"gpu_cc_cx_gmu_clk",
	"gpu_cc_cx_snoc_dvm_clk",
	"gpu_cc_cxo_aon_clk",
	"gpu_cc_cxo_clk",
	"gpu_cc_gx_gfx3d_clk",
	"gpu_cc_sleep_clk",
	"measure_only_gcc_gpu_cfg_ahb_clk",
	"measure_only_gpu_cc_gx_cxo_clk",
};

static int gpu_cc_debug_mux_sels[] = {
	0x10,		/* gpu_cc_ahb_clk */
	0x11,		/* gpu_cc_crc_ahb_clk */
	0x1A,		/* gpu_cc_cx_gfx3d_clk */
	0x1B,		/* gpu_cc_cx_gfx3d_slv_clk */
	0x18,		/* gpu_cc_cx_gmu_clk */
	0x15,		/* gpu_cc_cx_snoc_dvm_clk */
	0xA,		/* gpu_cc_cxo_aon_clk */
	0x19,		/* gpu_cc_cxo_clk */
	0xB,		/* gpu_cc_gx_gfx3d_clk */
	0x16,		/* gpu_cc_sleep_clk */
	0x1,		/* measure_only_gcc_gpu_cfg_ahb_clk */
	0xE,		/* measure_only_gpu_cc_gx_cxo_clk */
};

static struct clk_debug_mux gpu_cc_debug_mux = {
	.priv = &debug_mux_priv,
	.debug_offset = 0x1568,
	.post_div_offset = 0x10FC,
	.cbcr_offset = 0x1100,
	.src_sel_mask = 0xFF,
	.src_sel_shift = 0,
	.post_div_mask = 0xF,
	.post_div_shift = 0,
	.post_div_val = 2,
	.mux_sels = gpu_cc_debug_mux_sels,
	.num_mux_sels = ARRAY_SIZE(gpu_cc_debug_mux_sels),
	.hw.init = &(const struct clk_init_data){
		.name = "gpu_cc_debug_mux",
		.ops = &clk_debug_mux_ops,
		.parent_names = gpu_cc_debug_mux_parent_names,
		.num_parents = ARRAY_SIZE(gpu_cc_debug_mux_parent_names),
	},
};

static const char *const lpass_aon_cc_debug_mux_parent_names[] = {
	"lpass_aon_cc_ahb_timeout_clk",
	"lpass_aon_cc_aon_h_clk",
	"lpass_aon_cc_bus_alt_clk",
	"lpass_aon_cc_csr_h_clk",
	"lpass_aon_cc_mcc_access_clk",
	"lpass_aon_cc_pdc_h_clk",
	"lpass_aon_cc_q6_atbm_clk",
	"lpass_aon_cc_q6_debug_mux",
	"lpass_aon_cc_qsm_xo_clk",
	"lpass_aon_cc_rsc_hclk_clk",
	"lpass_aon_cc_sleep_clk",
	"lpass_aon_cc_ssc_h_clk",
	"lpass_aon_cc_tx_mclk_2x_clk",
	"lpass_aon_cc_tx_mclk_clk",
	"lpass_aon_cc_va_2x_clk",
	"lpass_aon_cc_va_clk",
	"lpass_aon_cc_va_mem0_clk",
	"lpass_aon_cc_va_xpu2_client_clk",
	"lpass_aon_cc_vs_vddcx_clk",
	"lpass_aon_cc_vs_vddmx_clk",
	"lpass_audio_cc_debug_mux",
	"scc_debug_mux",
};

static int lpass_aon_cc_debug_mux_sels[] = {
	0x14,		/* lpass_aon_cc_ahb_timeout_clk */
	0x2,		/* lpass_aon_cc_aon_h_clk */
	0xD,		/* lpass_aon_cc_bus_alt_clk */
	0x5,		/* lpass_aon_cc_csr_h_clk */
	0x24,		/* lpass_aon_cc_mcc_access_clk */
	0x2F,		/* lpass_aon_cc_pdc_h_clk */
	0x18,		/* lpass_aon_cc_q6_atbm_clk */
	0x1E,		/* lpass_aon_cc_q6_debug_mux */
	0xF,		/* lpass_aon_cc_qsm_xo_clk */
	0x21,		/* lpass_aon_cc_rsc_hclk_clk */
	0x2E,		/* lpass_aon_cc_sleep_clk */
	0x13,		/* lpass_aon_cc_ssc_h_clk */
	0x4,		/* lpass_aon_cc_tx_mclk_2x_clk */
	0x3,		/* lpass_aon_cc_tx_mclk_clk */
	0xE,		/* lpass_aon_cc_va_2x_clk */
	0x9,		/* lpass_aon_cc_va_clk */
	0x12,		/* lpass_aon_cc_va_mem0_clk */
	0x6,		/* lpass_aon_cc_va_xpu2_client_clk */
	0x23,		/* lpass_aon_cc_vs_vddcx_clk */
	0x22,		/* lpass_aon_cc_vs_vddmx_clk */
	0x3F,		/* lpass_audio_cc_debug_mux */
	0x3E,		/* scc_debug_mux */
};

static struct clk_debug_mux lpass_aon_cc_debug_mux = {
	.priv = &debug_mux_priv,
	.debug_offset = 0x5C,
	.post_div_offset = U32_MAX,
	.cbcr_offset = 0xA084,
	.src_sel_mask = 0x7E,
	.src_sel_shift = 1,
	.post_div_mask = 0x1,
	.post_div_shift = 0,
	.post_div_val = 1,
	.mux_sels = lpass_aon_cc_debug_mux_sels,
	.num_mux_sels = ARRAY_SIZE(lpass_aon_cc_debug_mux_sels),
	.hw.init = &(const struct clk_init_data){
		.name = "lpass_aon_cc_debug_mux",
		.ops = &clk_debug_mux_ops,
		.parent_names = lpass_aon_cc_debug_mux_parent_names,
		.num_parents = ARRAY_SIZE(lpass_aon_cc_debug_mux_parent_names),
	},
};

static const char *const lpass_audio_cc_debug_mux_parent_names[] = {
	"lpass_audio_cc_aud_slimbus_clk",
	"lpass_audio_cc_aud_slimbus_core_clk",
	"lpass_audio_cc_aud_slimbus_npl_clk",
	"lpass_audio_cc_bus_clk",
	"lpass_audio_cc_bus_timeout_clk",
	"lpass_audio_cc_codec_mem0_clk",
	"lpass_audio_cc_codec_mem1_clk",
	"lpass_audio_cc_codec_mem2_clk",
	"lpass_audio_cc_codec_mem3_clk",
	"lpass_audio_cc_codec_mem_clk",
	"lpass_audio_cc_ext_if1_ebit_clk",
	"lpass_audio_cc_ext_if1_ibit_clk",
	"lpass_audio_cc_ext_if2_ebit_clk",
	"lpass_audio_cc_ext_if2_ibit_clk",
	"lpass_audio_cc_ext_if3_ebit_clk",
	"lpass_audio_cc_ext_if3_ibit_clk",
	"lpass_audio_cc_ext_if4_ebit_clk",
	"lpass_audio_cc_ext_if4_ibit_clk",
	"lpass_audio_cc_ext_mclk0_clk",
	"lpass_audio_cc_ext_mclk1_clk",
	"lpass_audio_cc_lpaif_pcmoe_clk",
	"lpass_audio_cc_rx_mclk_2x_clk",
	"lpass_audio_cc_rx_mclk_clk",
	"lpass_audio_cc_sampling_clk",
	"lpass_audio_cc_wsa_mclk_2x_clk",
	"lpass_audio_cc_wsa_mclk_clk",
};

static int lpass_audio_cc_debug_mux_sels[] = {
	0x32,		/* lpass_audio_cc_aud_slimbus_clk */
	0x2,		/* lpass_audio_cc_aud_slimbus_core_clk */
	0x3F,		/* lpass_audio_cc_aud_slimbus_npl_clk */
	0x1,		/* lpass_audio_cc_bus_clk */
	0x7,		/* lpass_audio_cc_bus_timeout_clk */
	0x3,		/* lpass_audio_cc_codec_mem0_clk */
	0x4,		/* lpass_audio_cc_codec_mem1_clk */
	0x5,		/* lpass_audio_cc_codec_mem2_clk */
	0x6,		/* lpass_audio_cc_codec_mem3_clk */
	0x0,		/* lpass_audio_cc_codec_mem_clk */
	0x9,		/* lpass_audio_cc_ext_if1_ebit_clk */
	0x8,		/* lpass_audio_cc_ext_if1_ibit_clk */
	0xB,		/* lpass_audio_cc_ext_if2_ebit_clk */
	0xA,		/* lpass_audio_cc_ext_if2_ibit_clk */
	0xD,		/* lpass_audio_cc_ext_if3_ebit_clk */
	0xC,		/* lpass_audio_cc_ext_if3_ibit_clk */
	0x19,		/* lpass_audio_cc_ext_if4_ebit_clk */
	0x18,		/* lpass_audio_cc_ext_if4_ibit_clk */
	0xE,		/* lpass_audio_cc_ext_mclk0_clk */
	0xF,		/* lpass_audio_cc_ext_mclk1_clk */
	0x16,		/* lpass_audio_cc_lpaif_pcmoe_clk */
	0x14,		/* lpass_audio_cc_rx_mclk_2x_clk */
	0x15,		/* lpass_audio_cc_rx_mclk_clk */
	0x17,		/* lpass_audio_cc_sampling_clk */
	0x10,		/* lpass_audio_cc_wsa_mclk_2x_clk */
	0x11,		/* lpass_audio_cc_wsa_mclk_clk */
};

static struct clk_debug_mux lpass_audio_cc_debug_mux = {
	.priv = &debug_mux_priv,
	.debug_offset = 0x2F000,
	.post_div_offset = U32_MAX,
	.cbcr_offset = 0x2D004,
	.src_sel_mask = 0x3F,
	.src_sel_shift = 0,
	.post_div_mask = 0x1,
	.post_div_shift = 0,
	.post_div_val = 1,
	.mux_sels = lpass_audio_cc_debug_mux_sels,
	.num_mux_sels = ARRAY_SIZE(lpass_audio_cc_debug_mux_sels),
	.hw.init = &(const struct clk_init_data){
		.name = "lpass_audio_cc_debug_mux",
		.ops = &clk_debug_mux_ops,
		.parent_names = lpass_audio_cc_debug_mux_parent_names,
		.num_parents = ARRAY_SIZE(lpass_audio_cc_debug_mux_parent_names),
	},
};

static const char *const mc_cc_debug_mux_parent_names[] = {
	"measure_only_mccc_clk",
};

static struct clk_debug_mux mc_cc_debug_mux = {
	.period_offset = 0x20,
	.hw.init = &(struct clk_init_data){
		.name = "mc_cc_debug_mux",
		.ops = &clk_debug_mux_ops,
		.parent_names = mc_cc_debug_mux_parent_names,
		.num_parents = ARRAY_SIZE(mc_cc_debug_mux_parent_names),
	},
};

static struct mux_regmap_names mux_list[] = {
	{ .mux = &lpass_audio_cc_debug_mux, .regmap_name = "qcom,lpassaudiocc" },
	{ .mux = &lpass_aon_cc_debug_mux, .regmap_name = "qcom,lpassaoncc" },
	{ .mux = &apss_cc_debug_mux, .regmap_name = "qcom,apsscc" },
	{ .mux = &disp_cc_debug_mux, .regmap_name = "qcom,dispcc" },
	{ .mux = &gpu_cc_debug_mux, .regmap_name = "qcom,gpucc" },
	{ .mux = &mc_cc_debug_mux, .regmap_name = "qcom,mccc" },
	{ .mux = &gcc_debug_mux, .regmap_name = "qcom,gcc" },
};

static struct clk_dummy measure_only_cnoc_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_cnoc_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_disp_cc_sleep_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_disp_cc_sleep_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_disp_cc_xo_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_disp_cc_xo_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gcc_camera_xo_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_gcc_camera_xo_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gcc_cpuss_gnoc_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_gcc_cpuss_gnoc_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gcc_disp_xo_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_gcc_disp_xo_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gcc_gpu_cfg_ahb_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_gcc_gpu_cfg_ahb_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gcc_lpass_sway_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_gcc_lpass_sway_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_gpu_cc_gx_cxo_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_gpu_cc_gx_cxo_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_ipa_2x_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_ipa_2x_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_mccc_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_mccc_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_perf_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_perf_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_pwclk_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_pwclk_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_snoc_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_snoc_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_usb3_phy_wrapper_gcc_usb30_pipe_clk = {
	.rrate = 1000,
	.hw.init = &(const struct clk_init_data){
		.name = "measure_only_usb3_phy_wrapper_gcc_usb30_pipe_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_hw *debugcc_khaje_hws[] = {
	&measure_only_cnoc_clk.hw,
	&measure_only_disp_cc_sleep_clk.hw,
	&measure_only_disp_cc_xo_clk.hw,
	&measure_only_gcc_camera_xo_clk.hw,
	&measure_only_gcc_cpuss_gnoc_clk.hw,
	&measure_only_gcc_disp_xo_clk.hw,
	&measure_only_gcc_gpu_cfg_ahb_clk.hw,
	&measure_only_gcc_lpass_sway_clk.hw,
	&measure_only_gpu_cc_gx_cxo_clk.hw,
	&measure_only_ipa_2x_clk.hw,
	&measure_only_mccc_clk.hw,
	&measure_only_perf_clk.hw,
	&measure_only_pwclk_clk.hw,
	&measure_only_snoc_clk.hw,
	&measure_only_usb3_phy_wrapper_gcc_usb30_pipe_clk.hw,
};

static const struct of_device_id clk_debug_match_table[] = {
	{ .compatible = "qcom,khaje-debugcc" },
	{ }
};

static int clk_debug_khaje_probe(struct platform_device *pdev)
{
	struct clk *clk;
	int ret = 0, i;

	BUILD_BUG_ON(ARRAY_SIZE(apss_cc_debug_mux_parent_names) !=
		ARRAY_SIZE(apss_cc_debug_mux_sels));
	BUILD_BUG_ON(ARRAY_SIZE(disp_cc_debug_mux_parent_names) !=
		ARRAY_SIZE(disp_cc_debug_mux_sels));
	BUILD_BUG_ON(ARRAY_SIZE(gcc_debug_mux_parent_names) != ARRAY_SIZE(gcc_debug_mux_sels));
	BUILD_BUG_ON(ARRAY_SIZE(gpu_cc_debug_mux_parent_names) !=
		ARRAY_SIZE(gpu_cc_debug_mux_sels));
	BUILD_BUG_ON(ARRAY_SIZE(lpass_aon_cc_debug_mux_parent_names) !=
		ARRAY_SIZE(lpass_aon_cc_debug_mux_sels));
	BUILD_BUG_ON(ARRAY_SIZE(lpass_audio_cc_debug_mux_parent_names) !=
		ARRAY_SIZE(lpass_audio_cc_debug_mux_sels));

	clk = devm_clk_get(&pdev->dev, "xo_clk_src");
	if (IS_ERR(clk)) {
		if (PTR_ERR(clk) != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Unable to get xo clock\n");
		return PTR_ERR(clk);
	}

	debug_mux_priv.cxo = clk;

	for (i = 0; i < ARRAY_SIZE(mux_list); i++) {
		if (IS_ERR_OR_NULL(mux_list[i].mux->regmap)) {
			ret = map_debug_bases(pdev,
				mux_list[i].regmap_name, mux_list[i].mux);
			if (ret == -EBADR)
				continue;
			else if (ret)
				return ret;
		}
	}

	for (i = 0; i < ARRAY_SIZE(debugcc_khaje_hws); i++) {
		clk = devm_clk_register(&pdev->dev, debugcc_khaje_hws[i]);
		if (IS_ERR(clk)) {
			dev_err(&pdev->dev, "Unable to register %s, err:(%d)\n",
				clk_hw_get_name(debugcc_khaje_hws[i]),
				PTR_ERR(clk));
			return PTR_ERR(clk);
		}
	}

	for (i = 0; i < ARRAY_SIZE(mux_list); i++) {
		ret = devm_clk_register_debug_mux(&pdev->dev, mux_list[i].mux);
		if (ret) {
			dev_err(&pdev->dev, "Unable to register mux clk %s, err:(%d)\n",
				clk_hw_get_name(&mux_list[i].mux->hw),
				ret);
			return ret;
		}
	}

	ret = clk_debug_measure_register(&gcc_debug_mux.hw);
	if (ret) {
		dev_err(&pdev->dev, "Could not register Measure clocks\n");
		return ret;
	}

	dev_info(&pdev->dev, "Registered debug measure clocks\n");

	return ret;
}

static struct platform_driver clk_debug_driver = {
	.probe = clk_debug_khaje_probe,
	.driver = {
		.name = "khaje-debugcc",
		.of_match_table = clk_debug_match_table,
	},
};

static int __init clk_debug_khaje_init(void)
{
	return platform_driver_register(&clk_debug_driver);
}
fs_initcall(clk_debug_khaje_init);

MODULE_DESCRIPTION("QTI DEBUG CC KHAJE Driver");
MODULE_LICENSE("GPL v2");

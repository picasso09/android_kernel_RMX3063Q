/*
 * Copyright (C) 2016 MediaTek Inc.

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>

#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>
#include <mt-plat/upmu_common.h>

unsigned int mt6355_upmu_get_hwcid(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_HWCID_ADDR,
		&val,
		PMIC_HWCID_MASK,
		PMIC_HWCID_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_swcid(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_SWCID_ADDR,
		&val,
		PMIC_SWCID_MASK,
		PMIC_SWCID_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_srclken_in0_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SRCLKEN_IN0_EN_ADDR,
		val,
		PMIC_RG_SRCLKEN_IN0_EN_MASK,
		PMIC_RG_SRCLKEN_IN0_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_srclken_in0_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_SRCLKEN_IN0_EN_ADDR,
		&val,
		PMIC_RG_SRCLKEN_IN0_EN_MASK,
		PMIC_RG_SRCLKEN_IN0_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_srclken_in1_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SRCLKEN_IN1_EN_ADDR,
		val,
		PMIC_RG_SRCLKEN_IN1_EN_MASK,
		PMIC_RG_SRCLKEN_IN1_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_srclken_in1_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_SRCLKEN_IN1_EN_ADDR,
		&val,
		PMIC_RG_SRCLKEN_IN1_EN_MASK,
		PMIC_RG_SRCLKEN_IN1_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_osc_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_OSC_SEL_ADDR,
		val,
		PMIC_RG_OSC_SEL_MASK,
		PMIC_RG_OSC_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_srclken_in0_hw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SRCLKEN_IN0_HW_MODE_ADDR,
		val,
		PMIC_RG_SRCLKEN_IN0_HW_MODE_MASK,
		PMIC_RG_SRCLKEN_IN0_HW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_srclken_in1_hw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SRCLKEN_IN1_HW_MODE_ADDR,
		val,
		PMIC_RG_SRCLKEN_IN1_HW_MODE_MASK,
		PMIC_RG_SRCLKEN_IN1_HW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_osc_sel_hw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_OSC_SEL_HW_MODE_ADDR,
		val,
		PMIC_RG_OSC_SEL_HW_MODE_MASK,
		PMIC_RG_OSC_SEL_HW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_srclken_in_sync_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SRCLKEN_IN_SYNC_EN_ADDR,
		val,
		PMIC_RG_SRCLKEN_IN_SYNC_EN_MASK,
		PMIC_RG_SRCLKEN_IN_SYNC_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_srclken_in_sync_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_SRCLKEN_IN_SYNC_EN_ADDR,
		&val,
		PMIC_RG_SRCLKEN_IN_SYNC_EN_MASK,
		PMIC_RG_SRCLKEN_IN_SYNC_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_osc_en_auto_off(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_OSC_EN_AUTO_OFF_ADDR,
		val,
		PMIC_RG_OSC_EN_AUTO_OFF_MASK,
		PMIC_RG_OSC_EN_AUTO_OFF_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_osc_en_auto_off(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_OSC_EN_AUTO_OFF_ADDR,
		&val,
		PMIC_RG_OSC_EN_AUTO_OFF_MASK,
		PMIC_RG_OSC_EN_AUTO_OFF_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_test_out(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_TEST_OUT_ADDR,
		&val,
		PMIC_TEST_OUT_MASK,
		PMIC_TEST_OUT_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_mon_flag_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_MON_FLAG_SEL_ADDR,
		val,
		PMIC_RG_MON_FLAG_SEL_MASK,
		PMIC_RG_MON_FLAG_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_mon_grp_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_MON_GRP_SEL_ADDR,
		val,
		PMIC_RG_MON_GRP_SEL_MASK,
		PMIC_RG_MON_GRP_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_nandtree_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_NANDTREE_MODE_ADDR,
		val,
		PMIC_RG_NANDTREE_MODE_MASK,
		PMIC_RG_NANDTREE_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_test_auxadc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_TEST_AUXADC_ADDR,
		val,
		PMIC_RG_TEST_AUXADC_MASK,
		PMIC_RG_TEST_AUXADC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_efuse_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_EFUSE_MODE_ADDR,
		val,
		PMIC_RG_EFUSE_MODE_MASK,
		PMIC_RG_EFUSE_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_test_strup(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_TEST_STRUP_ADDR,
		val,
		PMIC_RG_TEST_STRUP_MASK,
		PMIC_RG_TEST_STRUP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_testmode_sw(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_TESTMODE_SW_ADDR,
		val,
		PMIC_TESTMODE_SW_MASK,
		PMIC_TESTMODE_SW_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_va12_pg_deb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VA12_PG_DEB_ADDR,
		&val,
		PMIC_VA12_PG_DEB_MASK,
		PMIC_VA12_PG_DEB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_va10_pg_deb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VA10_PG_DEB_ADDR,
		&val,
		PMIC_VA10_PG_DEB_MASK,
		PMIC_VA10_PG_DEB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vsram_gpu_pg_deb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VSRAM_GPU_PG_DEB_ADDR,
		&val,
		PMIC_VSRAM_GPU_PG_DEB_MASK,
		PMIC_VSRAM_GPU_PG_DEB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vsram_md_pg_deb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VSRAM_MD_PG_DEB_ADDR,
		&val,
		PMIC_VSRAM_MD_PG_DEB_MASK,
		PMIC_VSRAM_MD_PG_DEB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vsram_core_pg_deb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VSRAM_CORE_PG_DEB_ADDR,
		&val,
		PMIC_VSRAM_CORE_PG_DEB_MASK,
		PMIC_VSRAM_CORE_PG_DEB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_va18_pg_deb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VA18_PG_DEB_ADDR,
		&val,
		PMIC_VA18_PG_DEB_MASK,
		PMIC_VA18_PG_DEB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_buck_rsv_pg_deb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_BUCK_RSV_PG_DEB_ADDR,
		&val,
		PMIC_BUCK_RSV_PG_DEB_MASK,
		PMIC_BUCK_RSV_PG_DEB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vdram2_pg_deb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VDRAM2_PG_DEB_ADDR,
		&val,
		PMIC_VDRAM2_PG_DEB_MASK,
		PMIC_VDRAM2_PG_DEB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vdram1_pg_deb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VDRAM1_PG_DEB_ADDR,
		&val,
		PMIC_VDRAM1_PG_DEB_MASK,
		PMIC_VDRAM1_PG_DEB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vproc12_pg_deb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VPROC12_PG_DEB_ADDR,
		&val,
		PMIC_VPROC12_PG_DEB_MASK,
		PMIC_VPROC12_PG_DEB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vproc11_pg_deb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VPROC11_PG_DEB_ADDR,
		&val,
		PMIC_VPROC11_PG_DEB_MASK,
		PMIC_VPROC11_PG_DEB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vs1_pg_deb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VS1_PG_DEB_ADDR,
		&val,
		PMIC_VS1_PG_DEB_MASK,
		PMIC_VS1_PG_DEB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vmodem_pg_deb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VMODEM_PG_DEB_ADDR,
		&val,
		PMIC_VMODEM_PG_DEB_MASK,
		PMIC_VMODEM_PG_DEB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vgpu_pg_deb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VGPU_PG_DEB_ADDR,
		&val,
		PMIC_VGPU_PG_DEB_MASK,
		PMIC_VGPU_PG_DEB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vcore_pg_deb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VCORE_PG_DEB_ADDR,
		&val,
		PMIC_VCORE_PG_DEB_MASK,
		PMIC_VCORE_PG_DEB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vs2_pg_deb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VS2_PG_DEB_ADDR,
		&val,
		PMIC_VS2_PG_DEB_MASK,
		PMIC_VS2_PG_DEB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_ext_pmic_pg_deb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_EXT_PMIC_PG_DEB_ADDR,
		&val,
		PMIC_EXT_PMIC_PG_DEB_MASK,
		PMIC_EXT_PMIC_PG_DEB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vxo18_pg_deb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VXO18_PG_DEB_ADDR,
		&val,
		PMIC_VXO18_PG_DEB_MASK,
		PMIC_VXO18_PG_DEB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vxo22_pg_deb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VXO22_PG_DEB_ADDR,
		&val,
		PMIC_VXO22_PG_DEB_MASK,
		PMIC_VXO22_PG_DEB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vusb33_pg_deb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VUSB33_PG_DEB_ADDR,
		&val,
		PMIC_VUSB33_PG_DEB_MASK,
		PMIC_VUSB33_PG_DEB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vsram_proc_pg_deb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VSRAM_PROC_PG_DEB_ADDR,
		&val,
		PMIC_VSRAM_PROC_PG_DEB_MASK,
		PMIC_VSRAM_PROC_PG_DEB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vio28_pg_deb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VIO28_PG_DEB_ADDR,
		&val,
		PMIC_VIO28_PG_DEB_MASK,
		PMIC_VIO28_PG_DEB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vufs18_pg_deb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VUFS18_PG_DEB_ADDR,
		&val,
		PMIC_VUFS18_PG_DEB_MASK,
		PMIC_VUFS18_PG_DEB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vemc_pg_deb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VEMC_PG_DEB_ADDR,
		&val,
		PMIC_VEMC_PG_DEB_MASK,
		PMIC_VEMC_PG_DEB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vio18_pg_deb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VIO18_PG_DEB_ADDR,
		&val,
		PMIC_VIO18_PG_DEB_MASK,
		PMIC_VIO18_PG_DEB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_va12_pg_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_VA12_PG_STATUS_ADDR,
		&val,
		PMIC_STRUP_VA12_PG_STATUS_MASK,
		PMIC_STRUP_VA12_PG_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_va10_pg_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_VA10_PG_STATUS_ADDR,
		&val,
		PMIC_STRUP_VA10_PG_STATUS_MASK,
		PMIC_STRUP_VA10_PG_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_vsram_gpu_pg_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_VSRAM_GPU_PG_STATUS_ADDR,
		&val,
		PMIC_STRUP_VSRAM_GPU_PG_STATUS_MASK,
		PMIC_STRUP_VSRAM_GPU_PG_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_vsram_md_pg_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_VSRAM_MD_PG_STATUS_ADDR,
		&val,
		PMIC_STRUP_VSRAM_MD_PG_STATUS_MASK,
		PMIC_STRUP_VSRAM_MD_PG_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_vsram_core_pg_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_VSRAM_CORE_PG_STATUS_ADDR,
		&val,
		PMIC_STRUP_VSRAM_CORE_PG_STATUS_MASK,
		PMIC_STRUP_VSRAM_CORE_PG_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_va18_pg_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_VA18_PG_STATUS_ADDR,
		&val,
		PMIC_STRUP_VA18_PG_STATUS_MASK,
		PMIC_STRUP_VA18_PG_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_buck_rsv_pg_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_BUCK_RSV_PG_STATUS_ADDR,
		&val,
		PMIC_STRUP_BUCK_RSV_PG_STATUS_MASK,
		PMIC_STRUP_BUCK_RSV_PG_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_vdram2_pg_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_VDRAM2_PG_STATUS_ADDR,
		&val,
		PMIC_STRUP_VDRAM2_PG_STATUS_MASK,
		PMIC_STRUP_VDRAM2_PG_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_vdram1_pg_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_VDRAM1_PG_STATUS_ADDR,
		&val,
		PMIC_STRUP_VDRAM1_PG_STATUS_MASK,
		PMIC_STRUP_VDRAM1_PG_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_vproc12_pg_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_VPROC12_PG_STATUS_ADDR,
		&val,
		PMIC_STRUP_VPROC12_PG_STATUS_MASK,
		PMIC_STRUP_VPROC12_PG_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_vproc11_pg_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_VPROC11_PG_STATUS_ADDR,
		&val,
		PMIC_STRUP_VPROC11_PG_STATUS_MASK,
		PMIC_STRUP_VPROC11_PG_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_vs1_pg_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_VS1_PG_STATUS_ADDR,
		&val,
		PMIC_STRUP_VS1_PG_STATUS_MASK,
		PMIC_STRUP_VS1_PG_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_vmodem_pg_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_VMODEM_PG_STATUS_ADDR,
		&val,
		PMIC_STRUP_VMODEM_PG_STATUS_MASK,
		PMIC_STRUP_VMODEM_PG_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_vgpu_pg_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_VGPU_PG_STATUS_ADDR,
		&val,
		PMIC_STRUP_VGPU_PG_STATUS_MASK,
		PMIC_STRUP_VGPU_PG_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_vcore_pg_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_VCORE_PG_STATUS_ADDR,
		&val,
		PMIC_STRUP_VCORE_PG_STATUS_MASK,
		PMIC_STRUP_VCORE_PG_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_vs2_pg_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_VS2_PG_STATUS_ADDR,
		&val,
		PMIC_STRUP_VS2_PG_STATUS_MASK,
		PMIC_STRUP_VS2_PG_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_ext_pmic_pg_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_EXT_PMIC_PG_STATUS_ADDR,
		&val,
		PMIC_STRUP_EXT_PMIC_PG_STATUS_MASK,
		PMIC_STRUP_EXT_PMIC_PG_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_vxo18_pg_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_VXO18_PG_STATUS_ADDR,
		&val,
		PMIC_STRUP_VXO18_PG_STATUS_MASK,
		PMIC_STRUP_VXO18_PG_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_vxo22_pg_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_VXO22_PG_STATUS_ADDR,
		&val,
		PMIC_STRUP_VXO22_PG_STATUS_MASK,
		PMIC_STRUP_VXO22_PG_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_vusb33_pg_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_VUSB33_PG_STATUS_ADDR,
		&val,
		PMIC_STRUP_VUSB33_PG_STATUS_MASK,
		PMIC_STRUP_VUSB33_PG_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_vsram_proc_pg_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_VSRAM_PROC_PG_STATUS_ADDR,
		&val,
		PMIC_STRUP_VSRAM_PROC_PG_STATUS_MASK,
		PMIC_STRUP_VSRAM_PROC_PG_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_vio28_pg_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_VIO28_PG_STATUS_ADDR,
		&val,
		PMIC_STRUP_VIO28_PG_STATUS_MASK,
		PMIC_STRUP_VIO28_PG_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_vufs18_pg_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_VUFS18_PG_STATUS_ADDR,
		&val,
		PMIC_STRUP_VUFS18_PG_STATUS_MASK,
		PMIC_STRUP_VUFS18_PG_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_vemc_pg_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_VEMC_PG_STATUS_ADDR,
		&val,
		PMIC_STRUP_VEMC_PG_STATUS_MASK,
		PMIC_STRUP_VEMC_PG_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_vio18_pg_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_VIO18_PG_STATUS_ADDR,
		&val,
		PMIC_STRUP_VIO18_PG_STATUS_MASK,
		PMIC_STRUP_VIO18_PG_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_buck_rsv_oc_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_BUCK_RSV_OC_STATUS_ADDR,
		&val,
		PMIC_STRUP_BUCK_RSV_OC_STATUS_MASK,
		PMIC_STRUP_BUCK_RSV_OC_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_vdram2_oc_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_VDRAM2_OC_STATUS_ADDR,
		&val,
		PMIC_STRUP_VDRAM2_OC_STATUS_MASK,
		PMIC_STRUP_VDRAM2_OC_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_vdram1_oc_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_VDRAM1_OC_STATUS_ADDR,
		&val,
		PMIC_STRUP_VDRAM1_OC_STATUS_MASK,
		PMIC_STRUP_VDRAM1_OC_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_vproc12_oc_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_VPROC12_OC_STATUS_ADDR,
		&val,
		PMIC_STRUP_VPROC12_OC_STATUS_MASK,
		PMIC_STRUP_VPROC12_OC_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_vproc11_oc_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_VPROC11_OC_STATUS_ADDR,
		&val,
		PMIC_STRUP_VPROC11_OC_STATUS_MASK,
		PMIC_STRUP_VPROC11_OC_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_vs1_oc_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_VS1_OC_STATUS_ADDR,
		&val,
		PMIC_STRUP_VS1_OC_STATUS_MASK,
		PMIC_STRUP_VS1_OC_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_vmodem_oc_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_VMODEM_OC_STATUS_ADDR,
		&val,
		PMIC_STRUP_VMODEM_OC_STATUS_MASK,
		PMIC_STRUP_VMODEM_OC_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_vgpu_oc_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_VGPU_OC_STATUS_ADDR,
		&val,
		PMIC_STRUP_VGPU_OC_STATUS_MASK,
		PMIC_STRUP_VGPU_OC_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_vcore_oc_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_VCORE_OC_STATUS_ADDR,
		&val,
		PMIC_STRUP_VCORE_OC_STATUS_MASK,
		PMIC_STRUP_VCORE_OC_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_vs2_oc_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_VS2_OC_STATUS_ADDR,
		&val,
		PMIC_STRUP_VS2_OC_STATUS_MASK,
		PMIC_STRUP_VS2_OC_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_pmu_thermal_deb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_PMU_THERMAL_DEB_ADDR,
		&val,
		PMIC_PMU_THERMAL_DEB_MASK,
		PMIC_PMU_THERMAL_DEB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_strup_thermal_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STRUP_THERMAL_STATUS_ADDR,
		&val,
		PMIC_STRUP_THERMAL_STATUS_MASK,
		PMIC_STRUP_THERMAL_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_pmu_test_mode_scan(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_PMU_TEST_MODE_SCAN_ADDR,
		&val,
		PMIC_PMU_TEST_MODE_SCAN_MASK,
		PMIC_PMU_TEST_MODE_SCAN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_pwrkey_deb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_PWRKEY_DEB_ADDR,
		&val,
		PMIC_PWRKEY_DEB_MASK,
		PMIC_PWRKEY_DEB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_homekey_deb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_HOMEKEY_DEB_ADDR,
		&val,
		PMIC_HOMEKEY_DEB_MASK,
		PMIC_HOMEKEY_DEB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rtc_xtal_det_done(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RTC_XTAL_DET_DONE_ADDR,
		&val,
		PMIC_RTC_XTAL_DET_DONE_MASK,
		PMIC_RTC_XTAL_DET_DONE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_xosc32_enb_det(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_XOSC32_ENB_DET_ADDR,
		&val,
		PMIC_XOSC32_ENB_DET_MASK,
		PMIC_XOSC32_ENB_DET_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rtc_xtal_det_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RTC_XTAL_DET_RSV_ADDR,
		val,
		PMIC_RTC_XTAL_DET_RSV_MASK,
		PMIC_RTC_XTAL_DET_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_pmu_tdsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_PMU_TDSEL_ADDR,
		val,
		PMIC_RG_PMU_TDSEL_MASK,
		PMIC_RG_PMU_TDSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_spi_tdsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SPI_TDSEL_ADDR,
		val,
		PMIC_RG_SPI_TDSEL_MASK,
		PMIC_RG_SPI_TDSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_aud_tdsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_AUD_TDSEL_ADDR,
		val,
		PMIC_RG_AUD_TDSEL_MASK,
		PMIC_RG_AUD_TDSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_e32cal_tdsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_E32CAL_TDSEL_ADDR,
		val,
		PMIC_RG_E32CAL_TDSEL_MASK,
		PMIC_RG_E32CAL_TDSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_pmu_rdsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_PMU_RDSEL_ADDR,
		val,
		PMIC_RG_PMU_RDSEL_MASK,
		PMIC_RG_PMU_RDSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_spi_rdsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SPI_RDSEL_ADDR,
		val,
		PMIC_RG_SPI_RDSEL_MASK,
		PMIC_RG_SPI_RDSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_aud_rdsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_AUD_RDSEL_ADDR,
		val,
		PMIC_RG_AUD_RDSEL_MASK,
		PMIC_RG_AUD_RDSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_e32cal_rdsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_E32CAL_RDSEL_ADDR,
		val,
		PMIC_RG_E32CAL_RDSEL_MASK,
		PMIC_RG_E32CAL_RDSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_smt_wdtrstb_in(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SMT_WDTRSTB_IN_ADDR,
		val,
		PMIC_RG_SMT_WDTRSTB_IN_MASK,
		PMIC_RG_SMT_WDTRSTB_IN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_smt_homekey(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SMT_HOMEKEY_ADDR,
		val,
		PMIC_RG_SMT_HOMEKEY_MASK,
		PMIC_RG_SMT_HOMEKEY_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_smt_srclken_in0(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SMT_SRCLKEN_IN0_ADDR,
		val,
		PMIC_RG_SMT_SRCLKEN_IN0_MASK,
		PMIC_RG_SMT_SRCLKEN_IN0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_smt_srclken_in1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SMT_SRCLKEN_IN1_ADDR,
		val,
		PMIC_RG_SMT_SRCLKEN_IN1_MASK,
		PMIC_RG_SMT_SRCLKEN_IN1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_smt_rtc_32k1v8_0(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SMT_RTC_32K1V8_0_ADDR,
		val,
		PMIC_RG_SMT_RTC_32K1V8_0_MASK,
		PMIC_RG_SMT_RTC_32K1V8_0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_smt_rtc_32k1v8_1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SMT_RTC_32K1V8_1_ADDR,
		val,
		PMIC_RG_SMT_RTC_32K1V8_1_MASK,
		PMIC_RG_SMT_RTC_32K1V8_1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_smt_scp_vreq_vao(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SMT_SCP_VREQ_VAO_ADDR,
		val,
		PMIC_RG_SMT_SCP_VREQ_VAO_MASK,
		PMIC_RG_SMT_SCP_VREQ_VAO_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_smt_spi_clk(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SMT_SPI_CLK_ADDR,
		val,
		PMIC_RG_SMT_SPI_CLK_MASK,
		PMIC_RG_SMT_SPI_CLK_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_smt_spi_csn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SMT_SPI_CSN_ADDR,
		val,
		PMIC_RG_SMT_SPI_CSN_MASK,
		PMIC_RG_SMT_SPI_CSN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_smt_spi_mosi(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SMT_SPI_MOSI_ADDR,
		val,
		PMIC_RG_SMT_SPI_MOSI_MASK,
		PMIC_RG_SMT_SPI_MOSI_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_smt_spi_miso(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SMT_SPI_MISO_ADDR,
		val,
		PMIC_RG_SMT_SPI_MISO_MASK,
		PMIC_RG_SMT_SPI_MISO_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_smt_aud_clk(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SMT_AUD_CLK_ADDR,
		val,
		PMIC_RG_SMT_AUD_CLK_MASK,
		PMIC_RG_SMT_AUD_CLK_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_smt_aud_dat_mosi(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SMT_AUD_DAT_MOSI_ADDR,
		val,
		PMIC_RG_SMT_AUD_DAT_MOSI_MASK,
		PMIC_RG_SMT_AUD_DAT_MOSI_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_smt_aud_dat_miso(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SMT_AUD_DAT_MISO_ADDR,
		val,
		PMIC_RG_SMT_AUD_DAT_MISO_MASK,
		PMIC_RG_SMT_AUD_DAT_MISO_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_smt_vow_clk_miso(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SMT_VOW_CLK_MISO_ADDR,
		val,
		PMIC_RG_SMT_VOW_CLK_MISO_MASK,
		PMIC_RG_SMT_VOW_CLK_MISO_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_octl_srclken_in0(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_OCTL_SRCLKEN_IN0_ADDR,
		val,
		PMIC_RG_OCTL_SRCLKEN_IN0_MASK,
		PMIC_RG_OCTL_SRCLKEN_IN0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_octl_srclken_in1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_OCTL_SRCLKEN_IN1_ADDR,
		val,
		PMIC_RG_OCTL_SRCLKEN_IN1_MASK,
		PMIC_RG_OCTL_SRCLKEN_IN1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_octl_rtc_32k1v8_0(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_OCTL_RTC_32K1V8_0_ADDR,
		val,
		PMIC_RG_OCTL_RTC_32K1V8_0_MASK,
		PMIC_RG_OCTL_RTC_32K1V8_0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_octl_rtc_32k1v8_1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_OCTL_RTC_32K1V8_1_ADDR,
		val,
		PMIC_RG_OCTL_RTC_32K1V8_1_MASK,
		PMIC_RG_OCTL_RTC_32K1V8_1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_octl_spi_clk(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_OCTL_SPI_CLK_ADDR,
		val,
		PMIC_RG_OCTL_SPI_CLK_MASK,
		PMIC_RG_OCTL_SPI_CLK_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_octl_spi_csn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_OCTL_SPI_CSN_ADDR,
		val,
		PMIC_RG_OCTL_SPI_CSN_MASK,
		PMIC_RG_OCTL_SPI_CSN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_octl_spi_mosi(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_OCTL_SPI_MOSI_ADDR,
		val,
		PMIC_RG_OCTL_SPI_MOSI_MASK,
		PMIC_RG_OCTL_SPI_MOSI_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_octl_spi_miso(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_OCTL_SPI_MISO_ADDR,
		val,
		PMIC_RG_OCTL_SPI_MISO_MASK,
		PMIC_RG_OCTL_SPI_MISO_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_octl_aud_clk(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_OCTL_AUD_CLK_ADDR,
		val,
		PMIC_RG_OCTL_AUD_CLK_MASK,
		PMIC_RG_OCTL_AUD_CLK_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_octl_aud_dat_mosi(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_OCTL_AUD_DAT_MOSI_ADDR,
		val,
		PMIC_RG_OCTL_AUD_DAT_MOSI_MASK,
		PMIC_RG_OCTL_AUD_DAT_MOSI_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_octl_aud_dat_miso(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_OCTL_AUD_DAT_MISO_ADDR,
		val,
		PMIC_RG_OCTL_AUD_DAT_MISO_MASK,
		PMIC_RG_OCTL_AUD_DAT_MISO_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_octl_vow_clk_miso(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_OCTL_VOW_CLK_MISO_ADDR,
		val,
		PMIC_RG_OCTL_VOW_CLK_MISO_MASK,
		PMIC_RG_OCTL_VOW_CLK_MISO_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_octl_homekey(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_OCTL_HOMEKEY_ADDR,
		val,
		PMIC_RG_OCTL_HOMEKEY_MASK,
		PMIC_RG_OCTL_HOMEKEY_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_octl_scp_vreq_vao(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_OCTL_SCP_VREQ_VAO_ADDR,
		val,
		PMIC_RG_OCTL_SCP_VREQ_VAO_MASK,
		PMIC_RG_OCTL_SCP_VREQ_VAO_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_spi_clk_filter_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SPI_CLK_FILTER_EN_ADDR,
		val,
		PMIC_RG_SPI_CLK_FILTER_EN_MASK,
		PMIC_RG_SPI_CLK_FILTER_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_spi_clk_filter_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_SPI_CLK_FILTER_EN_ADDR,
		&val,
		PMIC_RG_SPI_CLK_FILTER_EN_MASK,
		PMIC_RG_SPI_CLK_FILTER_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_spi_csn_filter_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SPI_CSN_FILTER_EN_ADDR,
		val,
		PMIC_RG_SPI_CSN_FILTER_EN_MASK,
		PMIC_RG_SPI_CSN_FILTER_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_spi_csn_filter_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_SPI_CSN_FILTER_EN_ADDR,
		&val,
		PMIC_RG_SPI_CSN_FILTER_EN_MASK,
		PMIC_RG_SPI_CSN_FILTER_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_spi_mosi_filter_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SPI_MOSI_FILTER_EN_ADDR,
		val,
		PMIC_RG_SPI_MOSI_FILTER_EN_MASK,
		PMIC_RG_SPI_MOSI_FILTER_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_spi_mosi_filter_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_SPI_MOSI_FILTER_EN_ADDR,
		&val,
		PMIC_RG_SPI_MOSI_FILTER_EN_MASK,
		PMIC_RG_SPI_MOSI_FILTER_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_spi_miso_filter_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SPI_MISO_FILTER_EN_ADDR,
		val,
		PMIC_RG_SPI_MISO_FILTER_EN_MASK,
		PMIC_RG_SPI_MISO_FILTER_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_spi_miso_filter_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_SPI_MISO_FILTER_EN_ADDR,
		&val,
		PMIC_RG_SPI_MISO_FILTER_EN_MASK,
		PMIC_RG_SPI_MISO_FILTER_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_aud_clk_filter_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_AUD_CLK_FILTER_EN_ADDR,
		val,
		PMIC_RG_AUD_CLK_FILTER_EN_MASK,
		PMIC_RG_AUD_CLK_FILTER_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_aud_clk_filter_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_AUD_CLK_FILTER_EN_ADDR,
		&val,
		PMIC_RG_AUD_CLK_FILTER_EN_MASK,
		PMIC_RG_AUD_CLK_FILTER_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_aud_dat_mosi_filter_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_AUD_DAT_MOSI_FILTER_EN_ADDR,
		val,
		PMIC_RG_AUD_DAT_MOSI_FILTER_EN_MASK,
		PMIC_RG_AUD_DAT_MOSI_FILTER_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_aud_dat_mosi_filter_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_AUD_DAT_MOSI_FILTER_EN_ADDR,
		&val,
		PMIC_RG_AUD_DAT_MOSI_FILTER_EN_MASK,
		PMIC_RG_AUD_DAT_MOSI_FILTER_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_aud_dat_miso_filter_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_AUD_DAT_MISO_FILTER_EN_ADDR,
		val,
		PMIC_RG_AUD_DAT_MISO_FILTER_EN_MASK,
		PMIC_RG_AUD_DAT_MISO_FILTER_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_aud_dat_miso_filter_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_AUD_DAT_MISO_FILTER_EN_ADDR,
		&val,
		PMIC_RG_AUD_DAT_MISO_FILTER_EN_MASK,
		PMIC_RG_AUD_DAT_MISO_FILTER_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vow_clk_miso_filter_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VOW_CLK_MISO_FILTER_EN_ADDR,
		val,
		PMIC_RG_VOW_CLK_MISO_FILTER_EN_MASK,
		PMIC_RG_VOW_CLK_MISO_FILTER_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vow_clk_miso_filter_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VOW_CLK_MISO_FILTER_EN_ADDR,
		&val,
		PMIC_RG_VOW_CLK_MISO_FILTER_EN_MASK,
		PMIC_RG_VOW_CLK_MISO_FILTER_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_wdtrstb_in_filter_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_WDTRSTB_IN_FILTER_EN_ADDR,
		val,
		PMIC_RG_WDTRSTB_IN_FILTER_EN_MASK,
		PMIC_RG_WDTRSTB_IN_FILTER_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_wdtrstb_in_filter_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_WDTRSTB_IN_FILTER_EN_ADDR,
		&val,
		PMIC_RG_WDTRSTB_IN_FILTER_EN_MASK,
		PMIC_RG_WDTRSTB_IN_FILTER_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_homekey_filter_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_HOMEKEY_FILTER_EN_ADDR,
		val,
		PMIC_RG_HOMEKEY_FILTER_EN_MASK,
		PMIC_RG_HOMEKEY_FILTER_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_homekey_filter_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_HOMEKEY_FILTER_EN_ADDR,
		&val,
		PMIC_RG_HOMEKEY_FILTER_EN_MASK,
		PMIC_RG_HOMEKEY_FILTER_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_srclken_in0_filter_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SRCLKEN_IN0_FILTER_EN_ADDR,
		val,
		PMIC_RG_SRCLKEN_IN0_FILTER_EN_MASK,
		PMIC_RG_SRCLKEN_IN0_FILTER_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_srclken_in0_filter_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_SRCLKEN_IN0_FILTER_EN_ADDR,
		&val,
		PMIC_RG_SRCLKEN_IN0_FILTER_EN_MASK,
		PMIC_RG_SRCLKEN_IN0_FILTER_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_srclken_in1_filter_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SRCLKEN_IN1_FILTER_EN_ADDR,
		val,
		PMIC_RG_SRCLKEN_IN1_FILTER_EN_MASK,
		PMIC_RG_SRCLKEN_IN1_FILTER_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_srclken_in1_filter_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_SRCLKEN_IN1_FILTER_EN_ADDR,
		&val,
		PMIC_RG_SRCLKEN_IN1_FILTER_EN_MASK,
		PMIC_RG_SRCLKEN_IN1_FILTER_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_rtc32k_1v8_0_filter_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RTC32K_1V8_0_FILTER_EN_ADDR,
		val,
		PMIC_RG_RTC32K_1V8_0_FILTER_EN_MASK,
		PMIC_RG_RTC32K_1V8_0_FILTER_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_rtc32k_1v8_0_filter_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_RTC32K_1V8_0_FILTER_EN_ADDR,
		&val,
		PMIC_RG_RTC32K_1V8_0_FILTER_EN_MASK,
		PMIC_RG_RTC32K_1V8_0_FILTER_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_rtc32k_1v8_1_filter_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RTC32K_1V8_1_FILTER_EN_ADDR,
		val,
		PMIC_RG_RTC32K_1V8_1_FILTER_EN_MASK,
		PMIC_RG_RTC32K_1V8_1_FILTER_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_rtc32k_1v8_1_filter_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_RTC32K_1V8_1_FILTER_EN_ADDR,
		&val,
		PMIC_RG_RTC32K_1V8_1_FILTER_EN_MASK,
		PMIC_RG_RTC32K_1V8_1_FILTER_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_scp_vreq_vao_filter_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SCP_VREQ_VAO_FILTER_EN_ADDR,
		val,
		PMIC_RG_SCP_VREQ_VAO_FILTER_EN_MASK,
		PMIC_RG_SCP_VREQ_VAO_FILTER_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_scp_vreq_vao_filter_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_SCP_VREQ_VAO_FILTER_EN_ADDR,
		&val,
		PMIC_RG_SCP_VREQ_VAO_FILTER_EN_MASK,
		PMIC_RG_SCP_VREQ_VAO_FILTER_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_spi_clk_rcsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SPI_CLK_RCSEL_ADDR,
		val,
		PMIC_RG_SPI_CLK_RCSEL_MASK,
		PMIC_RG_SPI_CLK_RCSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_spi_csn_rcsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SPI_CSN_RCSEL_ADDR,
		val,
		PMIC_RG_SPI_CSN_RCSEL_MASK,
		PMIC_RG_SPI_CSN_RCSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_spi_mosi_rcsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SPI_MOSI_RCSEL_ADDR,
		val,
		PMIC_RG_SPI_MOSI_RCSEL_MASK,
		PMIC_RG_SPI_MOSI_RCSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_spi_miso_rcsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SPI_MISO_RCSEL_ADDR,
		val,
		PMIC_RG_SPI_MISO_RCSEL_MASK,
		PMIC_RG_SPI_MISO_RCSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_aud_clk_rcsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_AUD_CLK_RCSEL_ADDR,
		val,
		PMIC_RG_AUD_CLK_RCSEL_MASK,
		PMIC_RG_AUD_CLK_RCSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_aud_dat_mosi_rcsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_AUD_DAT_MOSI_RCSEL_ADDR,
		val,
		PMIC_RG_AUD_DAT_MOSI_RCSEL_MASK,
		PMIC_RG_AUD_DAT_MOSI_RCSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_aud_dat_miso_rcsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_AUD_DAT_MISO_RCSEL_ADDR,
		val,
		PMIC_RG_AUD_DAT_MISO_RCSEL_MASK,
		PMIC_RG_AUD_DAT_MISO_RCSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vow_clk_miso_rcsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VOW_CLK_MISO_RCSEL_ADDR,
		val,
		PMIC_RG_VOW_CLK_MISO_RCSEL_MASK,
		PMIC_RG_VOW_CLK_MISO_RCSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_wdtrstb_in_rcsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_WDTRSTB_IN_RCSEL_ADDR,
		val,
		PMIC_RG_WDTRSTB_IN_RCSEL_MASK,
		PMIC_RG_WDTRSTB_IN_RCSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_homekey_rcsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_HOMEKEY_RCSEL_ADDR,
		val,
		PMIC_RG_HOMEKEY_RCSEL_MASK,
		PMIC_RG_HOMEKEY_RCSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_srclken_in0_rcsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SRCLKEN_IN0_RCSEL_ADDR,
		val,
		PMIC_RG_SRCLKEN_IN0_RCSEL_MASK,
		PMIC_RG_SRCLKEN_IN0_RCSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_srclken_in1_rcsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SRCLKEN_IN1_RCSEL_ADDR,
		val,
		PMIC_RG_SRCLKEN_IN1_RCSEL_MASK,
		PMIC_RG_SRCLKEN_IN1_RCSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_rtc32k_1v8_0_rcsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RTC32K_1V8_0_RCSEL_ADDR,
		val,
		PMIC_RG_RTC32K_1V8_0_RCSEL_MASK,
		PMIC_RG_RTC32K_1V8_0_RCSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_rtc32k_1v8_1_rcsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RTC32K_1V8_1_RCSEL_ADDR,
		val,
		PMIC_RG_RTC32K_1V8_1_RCSEL_MASK,
		PMIC_RG_RTC32K_1V8_1_RCSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_scp_vreq_vao_rcsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SCP_VREQ_VAO_RCSEL_ADDR,
		val,
		PMIC_RG_SCP_VREQ_VAO_RCSEL_MASK,
		PMIC_RG_SCP_VREQ_VAO_RCSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_top_status(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_TOP_STATUS_ADDR,
		val,
		PMIC_TOP_STATUS_MASK,
		PMIC_TOP_STATUS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_srclken_in2_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SRCLKEN_IN2_EN_ADDR,
		val,
		PMIC_RG_SRCLKEN_IN2_EN_MASK,
		PMIC_RG_SRCLKEN_IN2_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_srclken_in2_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_SRCLKEN_IN2_EN_ADDR,
		&val,
		PMIC_RG_SRCLKEN_IN2_EN_MASK,
		PMIC_RG_SRCLKEN_IN2_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_srclken_in3_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SRCLKEN_IN3_EN_ADDR,
		val,
		PMIC_RG_SRCLKEN_IN3_EN_MASK,
		PMIC_RG_SRCLKEN_IN3_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_srclken_in3_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_SRCLKEN_IN3_EN_ADDR,
		&val,
		PMIC_RG_SRCLKEN_IN3_EN_MASK,
		PMIC_RG_SRCLKEN_IN3_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vm_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VM_MODE_ADDR,
		&val,
		PMIC_VM_MODE_MASK,
		PMIC_VM_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_g_smps_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_G_SMPS_CK_PDN_ADDR,
		val,
		PMIC_RG_G_SMPS_CK_PDN_MASK,
		PMIC_RG_G_SMPS_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_g_smps_test_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_G_SMPS_TEST_CK_PDN_ADDR,
		val,
		PMIC_RG_G_SMPS_TEST_CK_PDN_MASK,
		PMIC_RG_G_SMPS_TEST_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_intrp_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INTRP_CK_PDN_ADDR,
		val,
		PMIC_RG_INTRP_CK_PDN_MASK,
		PMIC_RG_INTRP_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_intrp_pre_oc_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INTRP_PRE_OC_CK_PDN_ADDR,
		val,
		PMIC_RG_INTRP_PRE_OC_CK_PDN_MASK,
		PMIC_RG_INTRP_PRE_OC_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_g_bif_1m_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_G_BIF_1M_CK_PDN_ADDR,
		val,
		PMIC_RG_G_BIF_1M_CK_PDN_MASK,
		PMIC_RG_G_BIF_1M_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_bif_x1_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BIF_X1_CK_PDN_ADDR,
		val,
		PMIC_RG_BIF_X1_CK_PDN_MASK,
		PMIC_RG_BIF_X1_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_bif_x4_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BIF_X4_CK_PDN_ADDR,
		val,
		PMIC_RG_BIF_X4_CK_PDN_MASK,
		PMIC_RG_BIF_X4_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_bif_x72_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BIF_X72_CK_PDN_ADDR,
		val,
		PMIC_RG_BIF_X72_CK_PDN_MASK,
		PMIC_RG_BIF_X72_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_auxadc_ao_1m_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_AUXADC_AO_1M_CK_PDN_ADDR,
		val,
		PMIC_RG_AUXADC_AO_1M_CK_PDN_MASK,
		PMIC_RG_AUXADC_AO_1M_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_auxadc_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_AUXADC_CK_PDN_ADDR,
		val,
		PMIC_RG_AUXADC_CK_PDN_MASK,
		PMIC_RG_AUXADC_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_auxadc_rng_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_AUXADC_RNG_CK_PDN_ADDR,
		val,
		PMIC_RG_AUXADC_RNG_CK_PDN_MASK,
		PMIC_RG_AUXADC_RNG_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_auxadc_1m_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_AUXADC_1M_CK_PDN_ADDR,
		val,
		PMIC_RG_AUXADC_1M_CK_PDN_MASK,
		PMIC_RG_AUXADC_1M_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_drv_32k_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_DRV_32K_CK_PDN_ADDR,
		val,
		PMIC_RG_DRV_32K_CK_PDN_MASK,
		PMIC_RG_DRV_32K_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_top_ckpdn_con0_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_TOP_CKPDN_CON0_RSV_ADDR,
		val,
		PMIC_RG_TOP_CKPDN_CON0_RSV_MASK,
		PMIC_RG_TOP_CKPDN_CON0_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_9m_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_9M_CK_PDN_ADDR,
		val,
		PMIC_RG_BUCK_9M_CK_PDN_MASK,
		PMIC_RG_BUCK_9M_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_1m_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_1M_CK_PDN_ADDR,
		val,
		PMIC_RG_BUCK_1M_CK_PDN_MASK,
		PMIC_RG_BUCK_1M_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_18m_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_18M_CK_PDN_ADDR,
		val,
		PMIC_RG_BUCK_18M_CK_PDN_MASK,
		PMIC_RG_BUCK_18M_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_32k_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_32K_CK_PDN_ADDR,
		val,
		PMIC_RG_BUCK_32K_CK_PDN_MASK,
		PMIC_RG_BUCK_32K_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_9m_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_9M_CK_PDN_ADDR,
		val,
		PMIC_RG_LDO_9M_CK_PDN_MASK,
		PMIC_RG_LDO_9M_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_1m_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_1M_CK_PDN_ADDR,
		val,
		PMIC_RG_LDO_1M_CK_PDN_MASK,
		PMIC_RG_LDO_1M_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_rsv0_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RSV0_CK_PDN_ADDR,
		val,
		PMIC_RG_RSV0_CK_PDN_MASK,
		PMIC_RG_RSV0_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_ana_clk_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_ANA_CLK_PDN_ADDR,
		val,
		PMIC_RG_BUCK_ANA_CLK_PDN_MASK,
		PMIC_RG_BUCK_ANA_CLK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_trim_75k_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_TRIM_75K_CK_PDN_ADDR,
		val,
		PMIC_RG_TRIM_75K_CK_PDN_MASK,
		PMIC_RG_TRIM_75K_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_chdet_75k_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_CHDET_75K_CK_PDN_ADDR,
		val,
		PMIC_RG_CHDET_75K_CK_PDN_MASK,
		PMIC_RG_CHDET_75K_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_spi_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SPI_CK_PDN_ADDR,
		val,
		PMIC_RG_SPI_CK_PDN_MASK,
		PMIC_RG_SPI_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_reg_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_REG_CK_PDN_ADDR,
		val,
		PMIC_RG_REG_CK_PDN_MASK,
		PMIC_RG_REG_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_bgr_test_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BGR_TEST_CK_PDN_ADDR,
		val,
		PMIC_RG_BGR_TEST_CK_PDN_MASK,
		PMIC_RG_BGR_TEST_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_fqmtr_32k_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_FQMTR_32K_CK_PDN_ADDR,
		val,
		PMIC_RG_FQMTR_32K_CK_PDN_MASK,
		PMIC_RG_FQMTR_32K_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_fqmtr_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_FQMTR_CK_PDN_ADDR,
		val,
		PMIC_RG_FQMTR_CK_PDN_MASK,
		PMIC_RG_FQMTR_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_ana_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_ANA_CK_PDN_ADDR,
		val,
		PMIC_RG_BUCK_ANA_CK_PDN_MASK,
		PMIC_RG_BUCK_ANA_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_eosc_cali_test_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_EOSC_CALI_TEST_CK_PDN_ADDR,
		val,
		PMIC_RG_EOSC_CALI_TEST_CK_PDN_MASK,
		PMIC_RG_EOSC_CALI_TEST_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_rtc_eosc32_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RTC_EOSC32_CK_PDN_ADDR,
		val,
		PMIC_RG_RTC_EOSC32_CK_PDN_MASK,
		PMIC_RG_RTC_EOSC32_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_rtc_sec_32k_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RTC_SEC_32K_CK_PDN_ADDR,
		val,
		PMIC_RG_RTC_SEC_32K_CK_PDN_MASK,
		PMIC_RG_RTC_SEC_32K_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_rtc_mclk_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RTC_MCLK_PDN_ADDR,
		val,
		PMIC_RG_RTC_MCLK_PDN_MASK,
		PMIC_RG_RTC_MCLK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_rtc_32k_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RTC_32K_CK_PDN_ADDR,
		val,
		PMIC_RG_RTC_32K_CK_PDN_MASK,
		PMIC_RG_RTC_32K_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_rtc_26m_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RTC_26M_CK_PDN_ADDR,
		val,
		PMIC_RG_RTC_26M_CK_PDN_MASK,
		PMIC_RG_RTC_26M_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_fgadc_ft_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_FGADC_FT_CK_PDN_ADDR,
		val,
		PMIC_RG_FGADC_FT_CK_PDN_MASK,
		PMIC_RG_FGADC_FT_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_fgadc_dig_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_FGADC_DIG_CK_PDN_ADDR,
		val,
		PMIC_RG_FGADC_DIG_CK_PDN_MASK,
		PMIC_RG_FGADC_DIG_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_fgadc_ana_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_FGADC_ANA_CK_PDN_ADDR,
		val,
		PMIC_RG_FGADC_ANA_CK_PDN_MASK,
		PMIC_RG_FGADC_ANA_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_strup_32k_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_32K_CK_PDN_ADDR,
		val,
		PMIC_RG_STRUP_32K_CK_PDN_MASK,
		PMIC_RG_STRUP_32K_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_strup_75k_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_75K_CK_PDN_ADDR,
		val,
		PMIC_RG_STRUP_75K_CK_PDN_MASK,
		PMIC_RG_STRUP_75K_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_rtc32k_1v8_0_o_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RTC32K_1V8_0_O_PDN_ADDR,
		val,
		PMIC_RG_RTC32K_1V8_0_O_PDN_MASK,
		PMIC_RG_RTC32K_1V8_0_O_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_rtc32k_1v8_1_o_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RTC32K_1V8_1_O_PDN_ADDR,
		val,
		PMIC_RG_RTC32K_1V8_1_O_PDN_MASK,
		PMIC_RG_RTC32K_1V8_1_O_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_rtc_2sec_off_det_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RTC_2SEC_OFF_DET_PDN_ADDR,
		val,
		PMIC_RG_RTC_2SEC_OFF_DET_PDN_MASK,
		PMIC_RG_RTC_2SEC_OFF_DET_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_smps_ck_div_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SMPS_CK_DIV_PDN_ADDR,
		val,
		PMIC_RG_SMPS_CK_DIV_PDN_MASK,
		PMIC_RG_SMPS_CK_DIV_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_baton_75k_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BATON_75K_CK_PDN_ADDR,
		val,
		PMIC_RG_BATON_75K_CK_PDN_MASK,
		PMIC_RG_BATON_75K_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_efuse_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_EFUSE_CK_PDN_ADDR,
		val,
		PMIC_RG_EFUSE_CK_PDN_MASK,
		PMIC_RG_EFUSE_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_accdet_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_ACCDET_CK_PDN_ADDR,
		val,
		PMIC_RG_ACCDET_CK_PDN_MASK,
		PMIC_RG_ACCDET_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_aud_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_AUD_CK_PDN_ADDR,
		val,
		PMIC_RG_AUD_CK_PDN_MASK,
		PMIC_RG_AUD_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_audif_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_AUDIF_CK_PDN_ADDR,
		val,
		PMIC_RG_AUDIF_CK_PDN_MASK,
		PMIC_RG_AUDIF_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vow32k_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VOW32K_CK_PDN_ADDR,
		val,
		PMIC_RG_VOW32K_CK_PDN_MASK,
		PMIC_RG_VOW32K_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vow12m_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VOW12M_CK_PDN_ADDR,
		val,
		PMIC_RG_VOW12M_CK_PDN_MASK,
		PMIC_RG_VOW12M_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_zcd13m_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_ZCD13M_CK_PDN_ADDR,
		val,
		PMIC_RG_ZCD13M_CK_PDN_MASK,
		PMIC_RG_ZCD13M_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_rtc_sec_mclk_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RTC_SEC_MCLK_PDN_ADDR,
		val,
		PMIC_RG_RTC_SEC_MCLK_PDN_MASK,
		PMIC_RG_RTC_SEC_MCLK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_auxadc_32k_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_AUXADC_32K_CK_PDN_ADDR,
		val,
		PMIC_RG_AUXADC_32K_CK_PDN_MASK,
		PMIC_RG_AUXADC_32K_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_eint_32k_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_EINT_32K_CK_PDN_ADDR,
		val,
		PMIC_RG_EINT_32K_CK_PDN_MASK,
		PMIC_RG_EINT_32K_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_top_ckpdn_con3_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_TOP_CKPDN_CON3_RSV_ADDR,
		val,
		PMIC_RG_TOP_CKPDN_CON3_RSV_MASK,
		PMIC_RG_TOP_CKPDN_CON3_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_rtcdet_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RTCDET_CK_PDN_ADDR,
		val,
		PMIC_RG_RTCDET_CK_PDN_MASK,
		PMIC_RG_RTCDET_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_rtc_75k_ck_pdn(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RTC_75K_CK_PDN_ADDR,
		val,
		PMIC_RG_RTC_75K_CK_PDN_MASK,
		PMIC_RG_RTC_75K_CK_PDN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_fqmtr_ck_cksel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_FQMTR_CK_CKSEL_ADDR,
		val,
		PMIC_RG_FQMTR_CK_CKSEL_MASK,
		PMIC_RG_FQMTR_CK_CKSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_75k_32k_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_75K_32K_SEL_ADDR,
		val,
		PMIC_RG_75K_32K_SEL_MASK,
		PMIC_RG_75K_32K_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_fgadc_ana_ck_cksel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_FGADC_ANA_CK_CKSEL_ADDR,
		val,
		PMIC_RG_FGADC_ANA_CK_CKSEL_MASK,
		PMIC_RG_FGADC_ANA_CK_CKSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_bgr_test_ck_cksel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BGR_TEST_CK_CKSEL_ADDR,
		val,
		PMIC_RG_BGR_TEST_CK_CKSEL_MASK,
		PMIC_RG_BGR_TEST_CK_CKSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_aud_ck_cksel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_AUD_CK_CKSEL_ADDR,
		val,
		PMIC_RG_AUD_CK_CKSEL_MASK,
		PMIC_RG_AUD_CK_CKSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_audif_ck_cksel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_AUDIF_CK_CKSEL_ADDR,
		val,
		PMIC_RG_AUDIF_CK_CKSEL_MASK,
		PMIC_RG_AUDIF_CK_CKSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_top_cksel_con0_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_TOP_CKSEL_CON0_RSV_ADDR,
		val,
		PMIC_RG_TOP_CKSEL_CON0_RSV_MASK,
		PMIC_RG_TOP_CKSEL_CON0_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_srcvolten_sw(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SRCVOLTEN_SW_ADDR,
		val,
		PMIC_RG_SRCVOLTEN_SW_MASK,
		PMIC_RG_SRCVOLTEN_SW_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_osc_sel_sw(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_OSC_SEL_SW_ADDR,
		val,
		PMIC_RG_BUCK_OSC_SEL_SW_MASK,
		PMIC_RG_BUCK_OSC_SEL_SW_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vowen_sw(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VOWEN_SW_ADDR,
		val,
		PMIC_RG_VOWEN_SW_MASK,
		PMIC_RG_VOWEN_SW_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_srcvolten_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SRCVOLTEN_MODE_ADDR,
		val,
		PMIC_RG_SRCVOLTEN_MODE_MASK,
		PMIC_RG_SRCVOLTEN_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_osc_sel_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_OSC_SEL_MODE_ADDR,
		val,
		PMIC_RG_BUCK_OSC_SEL_MODE_MASK,
		PMIC_RG_BUCK_OSC_SEL_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vowen_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VOWEN_MODE_ADDR,
		val,
		PMIC_RG_VOWEN_MODE_MASK,
		PMIC_RG_VOWEN_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_top_cksel_con2_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_TOP_CKSEL_CON2_RSV_ADDR,
		val,
		PMIC_RG_TOP_CKSEL_CON2_RSV_MASK,
		PMIC_RG_TOP_CKSEL_CON2_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_auxadc_ck_divsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_AUXADC_CK_DIVSEL_ADDR,
		val,
		PMIC_RG_AUXADC_CK_DIVSEL_MASK,
		PMIC_RG_AUXADC_CK_DIVSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_9m_ck_divsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_9M_CK_DIVSEL_ADDR,
		val,
		PMIC_RG_LDO_9M_CK_DIVSEL_MASK,
		PMIC_RG_LDO_9M_CK_DIVSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_9m_ck_divsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_9M_CK_DIVSEL_ADDR,
		val,
		PMIC_RG_BUCK_9M_CK_DIVSEL_MASK,
		PMIC_RG_BUCK_9M_CK_DIVSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_bif_x4_ck_divsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BIF_X4_CK_DIVSEL_ADDR,
		val,
		PMIC_RG_BIF_X4_CK_DIVSEL_MASK,
		PMIC_RG_BIF_X4_CK_DIVSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_reg_ck_divsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_REG_CK_DIVSEL_ADDR,
		val,
		PMIC_RG_REG_CK_DIVSEL_MASK,
		PMIC_RG_REG_CK_DIVSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_top_ckdivsel_con0_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_TOP_CKDIVSEL_CON0_RSV_ADDR,
		val,
		PMIC_TOP_CKDIVSEL_CON0_RSV_MASK,
		PMIC_TOP_CKDIVSEL_CON0_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_g_smps_ck_pdn_hwen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_G_SMPS_CK_PDN_HWEN_ADDR,
		val,
		PMIC_RG_G_SMPS_CK_PDN_HWEN_MASK,
		PMIC_RG_G_SMPS_CK_PDN_HWEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_smps_pd_1m_ck_pdn_hwen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SMPS_PD_1M_CK_PDN_HWEN_ADDR,
		val,
		PMIC_RG_SMPS_PD_1M_CK_PDN_HWEN_MASK,
		PMIC_RG_SMPS_PD_1M_CK_PDN_HWEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_auxadc_ck_pdn_hwen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_AUXADC_CK_PDN_HWEN_ADDR,
		val,
		PMIC_RG_AUXADC_CK_PDN_HWEN_MASK,
		PMIC_RG_AUXADC_CK_PDN_HWEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_auxadc_rng_ck_pdn_hwen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_AUXADC_RNG_CK_PDN_HWEN_ADDR,
		val,
		PMIC_RG_AUXADC_RNG_CK_PDN_HWEN_MASK,
		PMIC_RG_AUXADC_RNG_CK_PDN_HWEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_bif_x4_ck_pdn_hwen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BIF_X4_CK_PDN_HWEN_ADDR,
		val,
		PMIC_RG_BIF_X4_CK_PDN_HWEN_MASK,
		PMIC_RG_BIF_X4_CK_PDN_HWEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_bif_x72_ck_pdn_hwen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BIF_X72_CK_PDN_HWEN_ADDR,
		val,
		PMIC_RG_BIF_X72_CK_PDN_HWEN_MASK,
		PMIC_RG_BIF_X72_CK_PDN_HWEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_rtc_26m_ck_pdn_hwen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RTC_26M_CK_PDN_HWEN_ADDR,
		val,
		PMIC_RG_RTC_26M_CK_PDN_HWEN_MASK,
		PMIC_RG_RTC_26M_CK_PDN_HWEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_reg_ck_pdn_hwen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_REG_CK_PDN_HWEN_ADDR,
		val,
		PMIC_RG_REG_CK_PDN_HWEN_MASK,
		PMIC_RG_REG_CK_PDN_HWEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_rtc_mclk_pdn_hwen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RTC_MCLK_PDN_HWEN_ADDR,
		val,
		PMIC_RG_RTC_MCLK_PDN_HWEN_MASK,
		PMIC_RG_RTC_MCLK_PDN_HWEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_rtc_sec_32k_ck_pdn_hwen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RTC_SEC_32K_CK_PDN_HWEN_ADDR,
		val,
		PMIC_RG_RTC_SEC_32K_CK_PDN_HWEN_MASK,
		PMIC_RG_RTC_SEC_32K_CK_PDN_HWEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_efuse_ck_pdn_hwen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_EFUSE_CK_PDN_HWEN_ADDR,
		val,
		PMIC_RG_EFUSE_CK_PDN_HWEN_MASK,
		PMIC_RG_EFUSE_CK_PDN_HWEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_rtc_sec_mclk_pdn_hwen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RTC_SEC_MCLK_PDN_HWEN_ADDR,
		val,
		PMIC_RG_RTC_SEC_MCLK_PDN_HWEN_MASK,
		PMIC_RG_RTC_SEC_MCLK_PDN_HWEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_auxadc_32k_ck_pdn_hwen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_AUXADC_32K_CK_PDN_HWEN_ADDR,
		val,
		PMIC_RG_AUXADC_32K_CK_PDN_HWEN_MASK,
		PMIC_RG_AUXADC_32K_CK_PDN_HWEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_eint_32k_ck_pdn_hwen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_EINT_32K_CK_PDN_HWEN_ADDR,
		val,
		PMIC_RG_EINT_32K_CK_PDN_HWEN_MASK,
		PMIC_RG_EINT_32K_CK_PDN_HWEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_top_ckhwen_con0_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_TOP_CKHWEN_CON0_RSV_ADDR,
		val,
		PMIC_TOP_CKHWEN_CON0_RSV_MASK,
		PMIC_TOP_CKHWEN_CON0_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_9m_ck_pdn_hwen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_9M_CK_PDN_HWEN_ADDR,
		val,
		PMIC_RG_BUCK_9M_CK_PDN_HWEN_MASK,
		PMIC_RG_BUCK_9M_CK_PDN_HWEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_1m_ck_pdn_hwen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_1M_CK_PDN_HWEN_ADDR,
		val,
		PMIC_RG_BUCK_1M_CK_PDN_HWEN_MASK,
		PMIC_RG_BUCK_1M_CK_PDN_HWEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_18m_ck_pdn_hwen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_18M_CK_PDN_HWEN_ADDR,
		val,
		PMIC_RG_BUCK_18M_CK_PDN_HWEN_MASK,
		PMIC_RG_BUCK_18M_CK_PDN_HWEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_1m_ck_pdn_hwen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_1M_CK_PDN_HWEN_ADDR,
		val,
		PMIC_RG_LDO_1M_CK_PDN_HWEN_MASK,
		PMIC_RG_LDO_1M_CK_PDN_HWEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_9m_ck_pdn_hwen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_9M_CK_PDN_HWEN_ADDR,
		val,
		PMIC_RG_LDO_9M_CK_PDN_HWEN_MASK,
		PMIC_RG_LDO_9M_CK_PDN_HWEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_1m_ck_pdn_hwen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_1M_CK_PDN_HWEN_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_1M_CK_PDN_HWEN_MASK,
		PMIC_RG_BUCK_VMODEM_1M_CK_PDN_HWEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_auxadc_1m_ck_pdn_hwen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_AUXADC_1M_CK_PDN_HWEN_ADDR,
		val,
		PMIC_RG_AUXADC_1M_CK_PDN_HWEN_MASK,
		PMIC_RG_AUXADC_1M_CK_PDN_HWEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_top_ckhwen_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_TOP_CKHWEN_RSV_ADDR,
		val,
		PMIC_TOP_CKHWEN_RSV_MASK,
		PMIC_TOP_CKHWEN_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_freq_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_FREQ_SEL_ADDR,
		val,
		PMIC_RG_VPROC11_FREQ_SEL_MASK,
		PMIC_RG_VPROC11_FREQ_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_freq_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_FREQ_SEL_ADDR,
		val,
		PMIC_RG_VPROC12_FREQ_SEL_MASK,
		PMIC_RG_VPROC12_FREQ_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_freq_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_FREQ_SEL_ADDR,
		val,
		PMIC_RG_VCORE_FREQ_SEL_MASK,
		PMIC_RG_VCORE_FREQ_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_freq_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_FREQ_SEL_ADDR,
		val,
		PMIC_RG_VGPU_FREQ_SEL_MASK,
		PMIC_RG_VGPU_FREQ_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_freq_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_FREQ_SEL_ADDR,
		val,
		PMIC_RG_VDRAM1_FREQ_SEL_MASK,
		PMIC_RG_VDRAM1_FREQ_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_freq_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_FREQ_SEL_ADDR,
		val,
		PMIC_RG_VDRAM2_FREQ_SEL_MASK,
		PMIC_RG_VDRAM2_FREQ_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_freq_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_FREQ_SEL_ADDR,
		val,
		PMIC_RG_VMODEM_FREQ_SEL_MASK,
		PMIC_RG_VMODEM_FREQ_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_top_buck_anack_freq_sel_con0_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_TOP_BUCK_ANACK_FREQ_SEL_CON0_RSV_ADDR,
		val,
		PMIC_TOP_BUCK_ANACK_FREQ_SEL_CON0_RSV_MASK,
		PMIC_TOP_BUCK_ANACK_FREQ_SEL_CON0_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs1_freq_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS1_FREQ_SEL_ADDR,
		val,
		PMIC_RG_VS1_FREQ_SEL_MASK,
		PMIC_RG_VS1_FREQ_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs2_freq_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS2_FREQ_SEL_ADDR,
		val,
		PMIC_RG_VS2_FREQ_SEL_MASK,
		PMIC_RG_VS2_FREQ_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vpa_phs_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPA_PHS_SEL_ADDR,
		val,
		PMIC_RG_VPA_PHS_SEL_MASK,
		PMIC_RG_VPA_PHS_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_top_buck_anack_freq_sel_con1_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_TOP_BUCK_ANACK_FREQ_SEL_CON1_RSV_ADDR,
		val,
		PMIC_TOP_BUCK_ANACK_FREQ_SEL_CON1_RSV_MASK,
		PMIC_TOP_BUCK_ANACK_FREQ_SEL_CON1_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_pmu75k_ck_tst_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_PMU75K_CK_TST_DIS_ADDR,
		val,
		PMIC_RG_PMU75K_CK_TST_DIS_MASK,
		PMIC_RG_PMU75K_CK_TST_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_smps_ck_tst_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SMPS_CK_TST_DIS_ADDR,
		val,
		PMIC_RG_SMPS_CK_TST_DIS_MASK,
		PMIC_RG_SMPS_CK_TST_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_rtc32k_ck_tst_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RTC32K_CK_TST_DIS_ADDR,
		val,
		PMIC_RG_RTC32K_CK_TST_DIS_MASK,
		PMIC_RG_RTC32K_CK_TST_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_fg_ck_tst_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_FG_CK_TST_DIS_ADDR,
		val,
		PMIC_RG_FG_CK_TST_DIS_MASK,
		PMIC_RG_FG_CK_TST_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_rtc26m_ck_tst_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RTC26M_CK_TST_DIS_ADDR,
		val,
		PMIC_RG_RTC26M_CK_TST_DIS_MASK,
		PMIC_RG_RTC26M_CK_TST_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_aud26m_ck_tst_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_AUD26M_CK_TST_DIS_ADDR,
		val,
		PMIC_RG_AUD26M_CK_TST_DIS_MASK,
		PMIC_RG_AUD26M_CK_TST_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vow12m_ck_tst_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VOW12M_CK_TST_DIS_ADDR,
		val,
		PMIC_RG_VOW12M_CK_TST_DIS_MASK,
		PMIC_RG_VOW12M_CK_TST_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_top_cktst_con0_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_TOP_CKTST_CON0_RSV_ADDR,
		val,
		PMIC_TOP_CKTST_CON0_RSV_MASK,
		PMIC_TOP_CKTST_CON0_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_ana_auto_off_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_ANA_AUTO_OFF_DIS_ADDR,
		val,
		PMIC_RG_BUCK_ANA_AUTO_OFF_DIS_MASK,
		PMIC_RG_BUCK_ANA_AUTO_OFF_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_auxadc_ck_tstsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_AUXADC_CK_TSTSEL_ADDR,
		val,
		PMIC_RG_AUXADC_CK_TSTSEL_MASK,
		PMIC_RG_AUXADC_CK_TSTSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_fqmtr_ck_tstsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_FQMTR_CK_TSTSEL_ADDR,
		val,
		PMIC_RG_FQMTR_CK_TSTSEL_MASK,
		PMIC_RG_FQMTR_CK_TSTSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_rtcdet_ck_tstsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RTCDET_CK_TSTSEL_ADDR,
		val,
		PMIC_RG_RTCDET_CK_TSTSEL_MASK,
		PMIC_RG_RTCDET_CK_TSTSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_rtc_eosc32_ck_tstsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RTC_EOSC32_CK_TSTSEL_ADDR,
		val,
		PMIC_RG_RTC_EOSC32_CK_TSTSEL_MASK,
		PMIC_RG_RTC_EOSC32_CK_TSTSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_eosc_cali_test_ck_tstsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_EOSC_CALI_TEST_CK_TSTSEL_ADDR,
		val,
		PMIC_RG_EOSC_CALI_TEST_CK_TSTSEL_MASK,
		PMIC_RG_EOSC_CALI_TEST_CK_TSTSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_rtc26m_ck_tstsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RTC26M_CK_TSTSEL_ADDR,
		val,
		PMIC_RG_RTC26M_CK_TSTSEL_MASK,
		PMIC_RG_RTC26M_CK_TSTSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_rtc32k_ck_tstsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RTC32K_CK_TSTSEL_ADDR,
		val,
		PMIC_RG_RTC32K_CK_TSTSEL_MASK,
		PMIC_RG_RTC32K_CK_TSTSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_fg_ck_tstsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_FG_CK_TSTSEL_ADDR,
		val,
		PMIC_RG_FG_CK_TSTSEL_MASK,
		PMIC_RG_FG_CK_TSTSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_fgadc_ana_ck_tstsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_FGADC_ANA_CK_TSTSEL_ADDR,
		val,
		PMIC_RG_FGADC_ANA_CK_TSTSEL_MASK,
		PMIC_RG_FGADC_ANA_CK_TSTSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_bgr_test_ck_tstsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BGR_TEST_CK_TSTSEL_ADDR,
		val,
		PMIC_RG_BGR_TEST_CK_TSTSEL_MASK,
		PMIC_RG_BGR_TEST_CK_TSTSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_pmu75k_ck_tstsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_PMU75K_CK_TSTSEL_ADDR,
		val,
		PMIC_RG_PMU75K_CK_TSTSEL_MASK,
		PMIC_RG_PMU75K_CK_TSTSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_smps_ck_tstsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SMPS_CK_TSTSEL_ADDR,
		val,
		PMIC_RG_SMPS_CK_TSTSEL_MASK,
		PMIC_RG_SMPS_CK_TSTSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_aud_ck_tstsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_AUD_CK_TSTSEL_ADDR,
		val,
		PMIC_RG_AUD_CK_TSTSEL_MASK,
		PMIC_RG_AUD_CK_TSTSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_audif_ck_tstsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_AUDIF_CK_TSTSEL_ADDR,
		val,
		PMIC_RG_AUDIF_CK_TSTSEL_MASK,
		PMIC_RG_AUDIF_CK_TSTSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_aud26m_ck_tstsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_AUD26M_CK_TSTSEL_ADDR,
		val,
		PMIC_RG_AUD26M_CK_TSTSEL_MASK,
		PMIC_RG_AUD26M_CK_TSTSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vow12m_ck_tstsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VOW12M_CK_TSTSEL_ADDR,
		val,
		PMIC_RG_VOW12M_CK_TSTSEL_MASK,
		PMIC_RG_VOW12M_CK_TSTSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_clksq_en_aud(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_CLKSQ_EN_AUD_ADDR,
		val,
		PMIC_RG_CLKSQ_EN_AUD_MASK,
		PMIC_RG_CLKSQ_EN_AUD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_clksq_en_aud(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_CLKSQ_EN_AUD_ADDR,
		&val,
		PMIC_RG_CLKSQ_EN_AUD_MASK,
		PMIC_RG_CLKSQ_EN_AUD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_clksq_en_fqr(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_CLKSQ_EN_FQR_ADDR,
		val,
		PMIC_RG_CLKSQ_EN_FQR_MASK,
		PMIC_RG_CLKSQ_EN_FQR_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_clksq_en_fqr(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_CLKSQ_EN_FQR_ADDR,
		&val,
		PMIC_RG_CLKSQ_EN_FQR_MASK,
		PMIC_RG_CLKSQ_EN_FQR_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_clksq_en_aux_ap(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_CLKSQ_EN_AUX_AP_ADDR,
		val,
		PMIC_RG_CLKSQ_EN_AUX_AP_MASK,
		PMIC_RG_CLKSQ_EN_AUX_AP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_clksq_en_aux_ap(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_CLKSQ_EN_AUX_AP_ADDR,
		&val,
		PMIC_RG_CLKSQ_EN_AUX_AP_MASK,
		PMIC_RG_CLKSQ_EN_AUX_AP_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_clksq_en_aux_md(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_CLKSQ_EN_AUX_MD_ADDR,
		val,
		PMIC_RG_CLKSQ_EN_AUX_MD_MASK,
		PMIC_RG_CLKSQ_EN_AUX_MD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_clksq_en_aux_md(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_CLKSQ_EN_AUX_MD_ADDR,
		&val,
		PMIC_RG_CLKSQ_EN_AUX_MD_MASK,
		PMIC_RG_CLKSQ_EN_AUX_MD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_clksq_en_aux_gps(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_CLKSQ_EN_AUX_GPS_ADDR,
		val,
		PMIC_RG_CLKSQ_EN_AUX_GPS_MASK,
		PMIC_RG_CLKSQ_EN_AUX_GPS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_clksq_en_aux_gps(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_CLKSQ_EN_AUX_GPS_ADDR,
		&val,
		PMIC_RG_CLKSQ_EN_AUX_GPS_MASK,
		PMIC_RG_CLKSQ_EN_AUX_GPS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_clksq_en_aux_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_CLKSQ_EN_AUX_RSV_ADDR,
		val,
		PMIC_RG_CLKSQ_EN_AUX_RSV_MASK,
		PMIC_RG_CLKSQ_EN_AUX_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_clksq_en_aux_rsv(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_CLKSQ_EN_AUX_RSV_ADDR,
		&val,
		PMIC_RG_CLKSQ_EN_AUX_RSV_MASK,
		PMIC_RG_CLKSQ_EN_AUX_RSV_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_clksq_en_aux_ap_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_CLKSQ_EN_AUX_AP_MODE_ADDR,
		val,
		PMIC_RG_CLKSQ_EN_AUX_AP_MODE_MASK,
		PMIC_RG_CLKSQ_EN_AUX_AP_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_clksq_en_aux_ap_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_CLKSQ_EN_AUX_AP_MODE_ADDR,
		&val,
		PMIC_RG_CLKSQ_EN_AUX_AP_MODE_MASK,
		PMIC_RG_CLKSQ_EN_AUX_AP_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_clksq_en_aux_md_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_CLKSQ_EN_AUX_MD_MODE_ADDR,
		val,
		PMIC_RG_CLKSQ_EN_AUX_MD_MODE_MASK,
		PMIC_RG_CLKSQ_EN_AUX_MD_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_clksq_en_aux_md_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_CLKSQ_EN_AUX_MD_MODE_ADDR,
		&val,
		PMIC_RG_CLKSQ_EN_AUX_MD_MODE_MASK,
		PMIC_RG_CLKSQ_EN_AUX_MD_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_clksq_in_sel_va18(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_CLKSQ_IN_SEL_VA18_ADDR,
		val,
		PMIC_RG_CLKSQ_IN_SEL_VA18_MASK,
		PMIC_RG_CLKSQ_IN_SEL_VA18_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_clksq_in_sel_va18_swctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_CLKSQ_IN_SEL_VA18_SWCTRL_ADDR,
		val,
		PMIC_RG_CLKSQ_IN_SEL_VA18_SWCTRL_MASK,
		PMIC_RG_CLKSQ_IN_SEL_VA18_SWCTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_top_clksq_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_TOP_CLKSQ_RSV_ADDR,
		val,
		PMIC_TOP_CLKSQ_RSV_MASK,
		PMIC_TOP_CLKSQ_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_clksq_en_va18(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_CLKSQ_EN_VA18_ADDR,
		&val,
		PMIC_DA_CLKSQ_EN_VA18_MASK,
		PMIC_DA_CLKSQ_EN_VA18_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_clksq_rtc_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_CLKSQ_RTC_EN_ADDR,
		val,
		PMIC_RG_CLKSQ_RTC_EN_MASK,
		PMIC_RG_CLKSQ_RTC_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_clksq_rtc_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_CLKSQ_RTC_EN_ADDR,
		&val,
		PMIC_RG_CLKSQ_RTC_EN_MASK,
		PMIC_RG_CLKSQ_RTC_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_clksq_rtc_en_hw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_CLKSQ_RTC_EN_HW_MODE_ADDR,
		val,
		PMIC_RG_CLKSQ_RTC_EN_HW_MODE_MASK,
		PMIC_RG_CLKSQ_RTC_EN_HW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_clksq_rtc_en_hw_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_CLKSQ_RTC_EN_HW_MODE_ADDR,
		&val,
		PMIC_RG_CLKSQ_RTC_EN_HW_MODE_MASK,
		PMIC_RG_CLKSQ_RTC_EN_HW_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_top_clksq_rtc_rsv0(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_TOP_CLKSQ_RTC_RSV0_ADDR,
		val,
		PMIC_TOP_CLKSQ_RTC_RSV0_MASK,
		PMIC_TOP_CLKSQ_RTC_RSV0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_enbb_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_ENBB_SEL_ADDR,
		val,
		PMIC_RG_ENBB_SEL_MASK,
		PMIC_RG_ENBB_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_xosc_en_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_XOSC_EN_SEL_ADDR,
		val,
		PMIC_RG_XOSC_EN_SEL_MASK,
		PMIC_RG_XOSC_EN_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_xosc_en_sel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_XOSC_EN_SEL_ADDR,
		&val,
		PMIC_RG_XOSC_EN_SEL_MASK,
		PMIC_RG_XOSC_EN_SEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_top_clksq_rtc_rsv1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_TOP_CLKSQ_RTC_RSV1_ADDR,
		val,
		PMIC_TOP_CLKSQ_RTC_RSV1_MASK,
		PMIC_TOP_CLKSQ_RTC_RSV1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_clksq_en_vdig18(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_CLKSQ_EN_VDIG18_ADDR,
		&val,
		PMIC_DA_CLKSQ_EN_VDIG18_MASK,
		PMIC_DA_CLKSQ_EN_VDIG18_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_osc_75k_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_OSC_75K_TRIM_ADDR,
		val,
		PMIC_RG_OSC_75K_TRIM_MASK,
		PMIC_RG_OSC_75K_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_osc_75k_trim_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_OSC_75K_TRIM_EN_ADDR,
		val,
		PMIC_RG_OSC_75K_TRIM_EN_MASK,
		PMIC_RG_OSC_75K_TRIM_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_osc_75k_trim_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_OSC_75K_TRIM_EN_ADDR,
		&val,
		PMIC_RG_OSC_75K_TRIM_EN_MASK,
		PMIC_RG_OSC_75K_TRIM_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_osc_75k_trim_rate(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_OSC_75K_TRIM_RATE_ADDR,
		val,
		PMIC_RG_OSC_75K_TRIM_RATE_MASK,
		PMIC_RG_OSC_75K_TRIM_RATE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_osc_75k_trim(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_OSC_75K_TRIM_ADDR,
		&val,
		PMIC_DA_OSC_75K_TRIM_MASK,
		PMIC_DA_OSC_75K_TRIM_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_g_smps_ck_pdn_srclken0_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_G_SMPS_CK_PDN_SRCLKEN0_EN_ADDR,
		val,
		PMIC_RG_G_SMPS_CK_PDN_SRCLKEN0_EN_MASK,
		PMIC_RG_G_SMPS_CK_PDN_SRCLKEN0_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_g_smps_ck_pdn_srclken0_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_G_SMPS_CK_PDN_SRCLKEN0_EN_ADDR,
		&val,
		PMIC_RG_G_SMPS_CK_PDN_SRCLKEN0_EN_MASK,
		PMIC_RG_G_SMPS_CK_PDN_SRCLKEN0_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_g_smps_ck_pdn_srclken1_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_G_SMPS_CK_PDN_SRCLKEN1_EN_ADDR,
		val,
		PMIC_RG_G_SMPS_CK_PDN_SRCLKEN1_EN_MASK,
		PMIC_RG_G_SMPS_CK_PDN_SRCLKEN1_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_g_smps_ck_pdn_srclken1_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_G_SMPS_CK_PDN_SRCLKEN1_EN_ADDR,
		&val,
		PMIC_RG_G_SMPS_CK_PDN_SRCLKEN1_EN_MASK,
		PMIC_RG_G_SMPS_CK_PDN_SRCLKEN1_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_g_smps_ck_pdn_srclken2_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_G_SMPS_CK_PDN_SRCLKEN2_EN_ADDR,
		val,
		PMIC_RG_G_SMPS_CK_PDN_SRCLKEN2_EN_MASK,
		PMIC_RG_G_SMPS_CK_PDN_SRCLKEN2_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_g_smps_ck_pdn_srclken2_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_G_SMPS_CK_PDN_SRCLKEN2_EN_ADDR,
		&val,
		PMIC_RG_G_SMPS_CK_PDN_SRCLKEN2_EN_MASK,
		PMIC_RG_G_SMPS_CK_PDN_SRCLKEN2_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_g_smps_ck_pdn_buck_osc_sel_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_G_SMPS_CK_PDN_BUCK_OSC_SEL_EN_ADDR,
		val,
		PMIC_RG_G_SMPS_CK_PDN_BUCK_OSC_SEL_EN_MASK,
		PMIC_RG_G_SMPS_CK_PDN_BUCK_OSC_SEL_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_g_smps_ck_pdn_buck_osc_sel_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_G_SMPS_CK_PDN_BUCK_OSC_SEL_EN_ADDR,
		&val,
		PMIC_RG_G_SMPS_CK_PDN_BUCK_OSC_SEL_EN_MASK,
		PMIC_RG_G_SMPS_CK_PDN_BUCK_OSC_SEL_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_g_smps_ck_pdn_vowen_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_G_SMPS_CK_PDN_VOWEN_EN_ADDR,
		val,
		PMIC_RG_G_SMPS_CK_PDN_VOWEN_EN_MASK,
		PMIC_RG_G_SMPS_CK_PDN_VOWEN_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_g_smps_ck_pdn_vowen_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_G_SMPS_CK_PDN_VOWEN_EN_ADDR,
		&val,
		PMIC_RG_G_SMPS_CK_PDN_VOWEN_EN_MASK,
		PMIC_RG_G_SMPS_CK_PDN_VOWEN_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_osc_sel_srclken0_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_OSC_SEL_SRCLKEN0_EN_ADDR,
		val,
		PMIC_RG_OSC_SEL_SRCLKEN0_EN_MASK,
		PMIC_RG_OSC_SEL_SRCLKEN0_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_osc_sel_srclken0_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_OSC_SEL_SRCLKEN0_EN_ADDR,
		&val,
		PMIC_RG_OSC_SEL_SRCLKEN0_EN_MASK,
		PMIC_RG_OSC_SEL_SRCLKEN0_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_osc_sel_srclken1_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_OSC_SEL_SRCLKEN1_EN_ADDR,
		val,
		PMIC_RG_OSC_SEL_SRCLKEN1_EN_MASK,
		PMIC_RG_OSC_SEL_SRCLKEN1_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_osc_sel_srclken1_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_OSC_SEL_SRCLKEN1_EN_ADDR,
		&val,
		PMIC_RG_OSC_SEL_SRCLKEN1_EN_MASK,
		PMIC_RG_OSC_SEL_SRCLKEN1_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_osc_sel_srclken2_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_OSC_SEL_SRCLKEN2_EN_ADDR,
		val,
		PMIC_RG_OSC_SEL_SRCLKEN2_EN_MASK,
		PMIC_RG_OSC_SEL_SRCLKEN2_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_osc_sel_srclken2_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_OSC_SEL_SRCLKEN2_EN_ADDR,
		&val,
		PMIC_RG_OSC_SEL_SRCLKEN2_EN_MASK,
		PMIC_RG_OSC_SEL_SRCLKEN2_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_osc_sel_buck_ldo_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_OSC_SEL_BUCK_LDO_EN_ADDR,
		val,
		PMIC_RG_OSC_SEL_BUCK_LDO_EN_MASK,
		PMIC_RG_OSC_SEL_BUCK_LDO_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_osc_sel_buck_ldo_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_OSC_SEL_BUCK_LDO_EN_ADDR,
		&val,
		PMIC_RG_OSC_SEL_BUCK_LDO_EN_MASK,
		PMIC_RG_OSC_SEL_BUCK_LDO_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_osc_sel_vowen_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_OSC_SEL_VOWEN_EN_ADDR,
		val,
		PMIC_RG_OSC_SEL_VOWEN_EN_MASK,
		PMIC_RG_OSC_SEL_VOWEN_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_osc_sel_vowen_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_OSC_SEL_VOWEN_EN_ADDR,
		&val,
		PMIC_RG_OSC_SEL_VOWEN_EN_MASK,
		PMIC_RG_OSC_SEL_VOWEN_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_clk_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_CLK_RSV_ADDR,
		val,
		PMIC_RG_CLK_RSV_MASK,
		PMIC_RG_CLK_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc2_ckmux_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC2_CKMUX_EN_ADDR,
		val,
		PMIC_RG_VPROC2_CKMUX_EN_MASK,
		PMIC_RG_VPROC2_CKMUX_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vproc2_ckmux_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VPROC2_CKMUX_EN_ADDR,
		&val,
		PMIC_RG_VPROC2_CKMUX_EN_MASK,
		PMIC_RG_VPROC2_CKMUX_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vpa_sw_pdn_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPA_SW_PDN_EN_ADDR,
		val,
		PMIC_RG_VPA_SW_PDN_EN_MASK,
		PMIC_RG_VPA_SW_PDN_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vpa_sw_pdn_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VPA_SW_PDN_EN_ADDR,
		&val,
		PMIC_RG_VPA_SW_PDN_EN_MASK,
		PMIC_RG_VPA_SW_PDN_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_1m_pdn_w_osc_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_1M_PDN_W_OSC_SEL_ADDR,
		val,
		PMIC_RG_LDO_1M_PDN_W_OSC_SEL_MASK,
		PMIC_RG_LDO_1M_PDN_W_OSC_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_1m_pdn_w_osc_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_1M_PDN_W_OSC_SEL_ADDR,
		val,
		PMIC_RG_BUCK_1M_PDN_W_OSC_SEL_MASK,
		PMIC_RG_BUCK_1M_PDN_W_OSC_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_clkctl_rsv0(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_CLKCTL_RSV0_ADDR,
		val,
		PMIC_RG_CLKCTL_RSV0_MASK,
		PMIC_RG_CLKCTL_RSV0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_dcxo_pwrkey_rstb_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_DCXO_PWRKEY_RSTB_SEL_ADDR,
		val,
		PMIC_RG_DCXO_PWRKEY_RSTB_SEL_MASK,
		PMIC_RG_DCXO_PWRKEY_RSTB_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_efuse_man_rst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_EFUSE_MAN_RST_ADDR,
		val,
		PMIC_RG_EFUSE_MAN_RST_MASK,
		PMIC_RG_EFUSE_MAN_RST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_auxadc_rst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_AUXADC_RST_ADDR,
		val,
		PMIC_RG_AUXADC_RST_MASK,
		PMIC_RG_AUXADC_RST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_auxadc_reg_rst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_AUXADC_REG_RST_ADDR,
		val,
		PMIC_RG_AUXADC_REG_RST_MASK,
		PMIC_RG_AUXADC_REG_RST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_audio_rst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_AUDIO_RST_ADDR,
		val,
		PMIC_RG_AUDIO_RST_MASK,
		PMIC_RG_AUDIO_RST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_accdet_rst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_ACCDET_RST_ADDR,
		val,
		PMIC_RG_ACCDET_RST_MASK,
		PMIC_RG_ACCDET_RST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_bif_rst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BIF_RST_ADDR,
		val,
		PMIC_RG_BIF_RST_MASK,
		PMIC_RG_BIF_RST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_driver_rst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_DRIVER_RST_ADDR,
		val,
		PMIC_RG_DRIVER_RST_MASK,
		PMIC_RG_DRIVER_RST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_fgadc_rst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_FGADC_RST_ADDR,
		val,
		PMIC_RG_FGADC_RST_MASK,
		PMIC_RG_FGADC_RST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_fqmtr_rst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_FQMTR_RST_ADDR,
		val,
		PMIC_RG_FQMTR_RST_MASK,
		PMIC_RG_FQMTR_RST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_rtc_rst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RTC_RST_ADDR,
		val,
		PMIC_RG_RTC_RST_MASK,
		PMIC_RG_RTC_RST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_type_c_cc_rst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_TYPE_C_CC_RST_ADDR,
		val,
		PMIC_RG_TYPE_C_CC_RST_MASK,
		PMIC_RG_TYPE_C_CC_RST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_chrwdt_rst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_CHRWDT_RST_ADDR,
		val,
		PMIC_RG_CHRWDT_RST_MASK,
		PMIC_RG_CHRWDT_RST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_zcd_rst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_ZCD_RST_ADDR,
		val,
		PMIC_RG_ZCD_RST_MASK,
		PMIC_RG_ZCD_RST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_audncp_rst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_AUDNCP_RST_ADDR,
		val,
		PMIC_RG_AUDNCP_RST_MASK,
		PMIC_RG_AUDNCP_RST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_clk_trim_rst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_CLK_TRIM_RST_ADDR,
		val,
		PMIC_RG_CLK_TRIM_RST_MASK,
		PMIC_RG_CLK_TRIM_RST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_srclken_rst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_SRCLKEN_RST_ADDR,
		val,
		PMIC_RG_BUCK_SRCLKEN_RST_MASK,
		PMIC_RG_BUCK_SRCLKEN_RST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_strup_long_press_rst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_LONG_PRESS_RST_ADDR,
		val,
		PMIC_RG_STRUP_LONG_PRESS_RST_MASK,
		PMIC_RG_STRUP_LONG_PRESS_RST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_prot_pmpp_rst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_PROT_PMPP_RST_ADDR,
		val,
		PMIC_RG_BUCK_PROT_PMPP_RST_MASK,
		PMIC_RG_BUCK_PROT_PMPP_RST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_spk_rst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SPK_RST_ADDR,
		val,
		PMIC_RG_SPK_RST_MASK,
		PMIC_RG_SPK_RST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_chrdet_rst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_CHRDET_RST_ADDR,
		val,
		PMIC_RG_CHRDET_RST_MASK,
		PMIC_RG_CHRDET_RST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_ldo_ft_testmode_rst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_LDO_FT_TESTMODE_RST_ADDR,
		val,
		PMIC_RG_BUCK_LDO_FT_TESTMODE_RST_MASK,
		PMIC_RG_BUCK_LDO_FT_TESTMODE_RST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_baton_rst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BATON_RST_ADDR,
		val,
		PMIC_RG_BATON_RST_MASK,
		PMIC_RG_BATON_RST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_fgadc_rst_src_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_FGADC_RST_SRC_SEL_ADDR,
		val,
		PMIC_RG_FGADC_RST_SRC_SEL_MASK,
		PMIC_RG_FGADC_RST_SRC_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_cali_rst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_CALI_RST_ADDR,
		val,
		PMIC_RG_LDO_CALI_RST_MASK,
		PMIC_RG_LDO_CALI_RST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_pseq_pwrmsk_rst_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_PSEQ_PWRMSK_RST_SEL_ADDR,
		val,
		PMIC_RG_PSEQ_PWRMSK_RST_SEL_MASK,
		PMIC_RG_PSEQ_PWRMSK_RST_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_top_rst_con1_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_TOP_RST_CON1_RSV_ADDR,
		val,
		PMIC_TOP_RST_CON1_RSV_MASK,
		PMIC_TOP_RST_CON1_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_chr_ldo_det_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_CHR_LDO_DET_MODE_ADDR,
		val,
		PMIC_RG_CHR_LDO_DET_MODE_MASK,
		PMIC_RG_CHR_LDO_DET_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_chr_ldo_det_sw(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_CHR_LDO_DET_SW_ADDR,
		val,
		PMIC_RG_CHR_LDO_DET_SW_MASK,
		PMIC_RG_CHR_LDO_DET_SW_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_chrwdt_flag_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_CHRWDT_FLAG_MODE_ADDR,
		val,
		PMIC_RG_CHRWDT_FLAG_MODE_MASK,
		PMIC_RG_CHRWDT_FLAG_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_chrwdt_flag_sw(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_CHRWDT_FLAG_SW_ADDR,
		val,
		PMIC_RG_CHRWDT_FLAG_SW_MASK,
		PMIC_RG_CHRWDT_FLAG_SW_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_top_rst_con2_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_TOP_RST_CON2_RSV_ADDR,
		val,
		PMIC_TOP_RST_CON2_RSV_MASK,
		PMIC_TOP_RST_CON2_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_wdtrstb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_WDTRSTB_EN_ADDR,
		val,
		PMIC_RG_WDTRSTB_EN_MASK,
		PMIC_RG_WDTRSTB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_wdtrstb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_WDTRSTB_EN_ADDR,
		&val,
		PMIC_RG_WDTRSTB_EN_MASK,
		PMIC_RG_WDTRSTB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_wdtrstb_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_WDTRSTB_MODE_ADDR,
		val,
		PMIC_RG_WDTRSTB_MODE_MASK,
		PMIC_RG_WDTRSTB_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_wdtrstb_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_WDTRSTB_STATUS_ADDR,
		&val,
		PMIC_WDTRSTB_STATUS_MASK,
		PMIC_WDTRSTB_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_wdtrstb_status_clr(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_WDTRSTB_STATUS_CLR_ADDR,
		val,
		PMIC_WDTRSTB_STATUS_CLR_MASK,
		PMIC_WDTRSTB_STATUS_CLR_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_wdtrstb_fb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_WDTRSTB_FB_EN_ADDR,
		val,
		PMIC_RG_WDTRSTB_FB_EN_MASK,
		PMIC_RG_WDTRSTB_FB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_wdtrstb_fb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_WDTRSTB_FB_EN_ADDR,
		&val,
		PMIC_RG_WDTRSTB_FB_EN_MASK,
		PMIC_RG_WDTRSTB_FB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_wdtrstb_deb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_WDTRSTB_DEB_ADDR,
		val,
		PMIC_RG_WDTRSTB_DEB_MASK,
		PMIC_RG_WDTRSTB_DEB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_homekey_rst_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_HOMEKEY_RST_EN_ADDR,
		val,
		PMIC_RG_HOMEKEY_RST_EN_MASK,
		PMIC_RG_HOMEKEY_RST_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_homekey_rst_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_HOMEKEY_RST_EN_ADDR,
		&val,
		PMIC_RG_HOMEKEY_RST_EN_MASK,
		PMIC_RG_HOMEKEY_RST_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_pwrkey_rst_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_PWRKEY_RST_EN_ADDR,
		val,
		PMIC_RG_PWRKEY_RST_EN_MASK,
		PMIC_RG_PWRKEY_RST_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_pwrkey_rst_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_PWRKEY_RST_EN_ADDR,
		&val,
		PMIC_RG_PWRKEY_RST_EN_MASK,
		PMIC_RG_PWRKEY_RST_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_pwrrst_tmr_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_PWRRST_TMR_DIS_ADDR,
		val,
		PMIC_RG_PWRRST_TMR_DIS_MASK,
		PMIC_RG_PWRRST_TMR_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_pwrkey_rst_td(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_PWRKEY_RST_TD_ADDR,
		val,
		PMIC_RG_PWRKEY_RST_TD_MASK,
		PMIC_RG_PWRKEY_RST_TD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_top_rst_misc_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_TOP_RST_MISC_RSV_ADDR,
		val,
		PMIC_TOP_RST_MISC_RSV_MASK,
		PMIC_TOP_RST_MISC_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_vpwrin_rstb_status(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_VPWRIN_RSTB_STATUS_ADDR,
		val,
		PMIC_VPWRIN_RSTB_STATUS_MASK,
		PMIC_VPWRIN_RSTB_STATUS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_ddlo_rstb_status(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_DDLO_RSTB_STATUS_ADDR,
		val,
		PMIC_DDLO_RSTB_STATUS_MASK,
		PMIC_DDLO_RSTB_STATUS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_uvlo_rstb_status(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_UVLO_RSTB_STATUS_ADDR,
		val,
		PMIC_UVLO_RSTB_STATUS_MASK,
		PMIC_UVLO_RSTB_STATUS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rtc_ddlo_rstb_status(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RTC_DDLO_RSTB_STATUS_ADDR,
		val,
		PMIC_RTC_DDLO_RSTB_STATUS_MASK,
		PMIC_RTC_DDLO_RSTB_STATUS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_chrwdt_reg_rstb_status(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_CHRWDT_REG_RSTB_STATUS_ADDR,
		val,
		PMIC_CHRWDT_REG_RSTB_STATUS_MASK,
		PMIC_CHRWDT_REG_RSTB_STATUS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_chrdet_reg_rstb_status(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_CHRDET_REG_RSTB_STATUS_ADDR,
		val,
		PMIC_CHRDET_REG_RSTB_STATUS_MASK,
		PMIC_CHRDET_REG_RSTB_STATUS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_bwdt_ddlo_rstb_status(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_BWDT_DDLO_RSTB_STATUS_ADDR,
		val,
		PMIC_BWDT_DDLO_RSTB_STATUS_MASK,
		PMIC_BWDT_DDLO_RSTB_STATUS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_top_rst_status_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_TOP_RST_STATUS_RSV_ADDR,
		val,
		PMIC_TOP_RST_STATUS_RSV_MASK,
		PMIC_TOP_RST_STATUS_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_top_rst_rsv_con0(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_TOP_RST_RSV_CON0_ADDR,
		val,
		PMIC_TOP_RST_RSV_CON0_MASK,
		PMIC_TOP_RST_RSV_CON0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_top_rst_rsv_con1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_TOP_RST_RSV_CON1_ADDR,
		val,
		PMIC_TOP_RST_RSV_CON1_MASK,
		PMIC_TOP_RST_RSV_CON1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_bank_fqmtr_swrst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_BANK_FQMTR_SWRST_ADDR,
		val,
		PMIC_BANK_FQMTR_SWRST_MASK,
		PMIC_BANK_FQMTR_SWRST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_bank_spi_swrst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_BANK_SPI_SWRST_ADDR,
		val,
		PMIC_BANK_SPI_SWRST_MASK,
		PMIC_BANK_SPI_SWRST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_bank_strup_swrst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_BANK_STRUP_SWRST_ADDR,
		val,
		PMIC_BANK_STRUP_SWRST_MASK,
		PMIC_BANK_STRUP_SWRST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_bank_buck_swrst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_BANK_BUCK_SWRST_ADDR,
		val,
		PMIC_BANK_BUCK_SWRST_MASK,
		PMIC_BANK_BUCK_SWRST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_bank_buck_ana_swrst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_BANK_BUCK_ANA_SWRST_ADDR,
		val,
		PMIC_BANK_BUCK_ANA_SWRST_MASK,
		PMIC_BANK_BUCK_ANA_SWRST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_bank_wdtdbg_swrst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_BANK_WDTDBG_SWRST_ADDR,
		val,
		PMIC_BANK_WDTDBG_SWRST_MASK,
		PMIC_BANK_WDTDBG_SWRST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_bank_ldo_0_swrst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_BANK_LDO_0_SWRST_ADDR,
		val,
		PMIC_BANK_LDO_0_SWRST_MASK,
		PMIC_BANK_LDO_0_SWRST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_bank_ldo_1_swrst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_BANK_LDO_1_SWRST_ADDR,
		val,
		PMIC_BANK_LDO_1_SWRST_MASK,
		PMIC_BANK_LDO_1_SWRST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_bank_ldo_ana_swrst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_BANK_LDO_ANA_SWRST_ADDR,
		val,
		PMIC_BANK_LDO_ANA_SWRST_MASK,
		PMIC_BANK_LDO_ANA_SWRST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_bank_accdet_swrst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_BANK_ACCDET_SWRST_ADDR,
		val,
		PMIC_BANK_ACCDET_SWRST_MASK,
		PMIC_BANK_ACCDET_SWRST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_bank_efuse_swrst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_BANK_EFUSE_SWRST_ADDR,
		val,
		PMIC_BANK_EFUSE_SWRST_MASK,
		PMIC_BANK_EFUSE_SWRST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_bank_dcxo_swrst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_BANK_DCXO_SWRST_ADDR,
		val,
		PMIC_BANK_DCXO_SWRST_MASK,
		PMIC_BANK_DCXO_SWRST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_bank_pchr_swrst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_BANK_PCHR_SWRST_ADDR,
		val,
		PMIC_BANK_PCHR_SWRST_MASK,
		PMIC_BANK_PCHR_SWRST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_bank_gpio_swrst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_BANK_GPIO_SWRST_ADDR,
		val,
		PMIC_BANK_GPIO_SWRST_MASK,
		PMIC_BANK_GPIO_SWRST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_bank_eosc_cali_swrst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_BANK_EOSC_CALI_SWRST_ADDR,
		val,
		PMIC_BANK_EOSC_CALI_SWRST_MASK,
		PMIC_BANK_EOSC_CALI_SWRST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_bank_vrtc_pwm_swrst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_BANK_VRTC_PWM_SWRST_ADDR,
		val,
		PMIC_BANK_VRTC_PWM_SWRST_MASK,
		PMIC_BANK_VRTC_PWM_SWRST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_bank_rtc_swrst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_BANK_RTC_SWRST_ADDR,
		val,
		PMIC_BANK_RTC_SWRST_MASK,
		PMIC_BANK_RTC_SWRST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_bank_rtc_sec_swrst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_BANK_RTC_SEC_SWRST_ADDR,
		val,
		PMIC_BANK_RTC_SEC_SWRST_MASK,
		PMIC_BANK_RTC_SEC_SWRST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_bank_bif_swrst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_BANK_BIF_SWRST_ADDR,
		val,
		PMIC_BANK_BIF_SWRST_MASK,
		PMIC_BANK_BIF_SWRST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_bank_fgadc_swrst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_BANK_FGADC_SWRST_ADDR,
		val,
		PMIC_BANK_FGADC_SWRST_MASK,
		PMIC_BANK_FGADC_SWRST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_bank_auxadc_swrst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_BANK_AUXADC_SWRST_ADDR,
		val,
		PMIC_BANK_AUXADC_SWRST_MASK,
		PMIC_BANK_AUXADC_SWRST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_bank_driver_swrst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_BANK_DRIVER_SWRST_ADDR,
		val,
		PMIC_BANK_DRIVER_SWRST_MASK,
		PMIC_BANK_DRIVER_SWRST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_bank_audio_swrst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_BANK_AUDIO_SWRST_ADDR,
		val,
		PMIC_BANK_AUDIO_SWRST_MASK,
		PMIC_BANK_AUDIO_SWRST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_bank_audzcd_swrst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_BANK_AUDZCD_SWRST_ADDR,
		val,
		PMIC_BANK_AUDZCD_SWRST_MASK,
		PMIC_BANK_AUDZCD_SWRST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_top_rst_bank_con1_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_TOP_RST_BANK_CON1_RSV_ADDR,
		val,
		PMIC_TOP_RST_BANK_CON1_RSV_MASK,
		PMIC_TOP_RST_BANK_CON1_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_en_pwrkey(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_PWRKEY_ADDR,
		val,
		PMIC_RG_INT_EN_PWRKEY_MASK,
		PMIC_RG_INT_EN_PWRKEY_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_pwrkey(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_PWRKEY_ADDR,
		&val,
		PMIC_RG_INT_EN_PWRKEY_MASK,
		PMIC_RG_INT_EN_PWRKEY_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_homekey(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_HOMEKEY_ADDR,
		val,
		PMIC_RG_INT_EN_HOMEKEY_MASK,
		PMIC_RG_INT_EN_HOMEKEY_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_homekey(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_HOMEKEY_ADDR,
		&val,
		PMIC_RG_INT_EN_HOMEKEY_MASK,
		PMIC_RG_INT_EN_HOMEKEY_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_pwrkey_r(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_PWRKEY_R_ADDR,
		val,
		PMIC_RG_INT_EN_PWRKEY_R_MASK,
		PMIC_RG_INT_EN_PWRKEY_R_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_pwrkey_r(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_PWRKEY_R_ADDR,
		&val,
		PMIC_RG_INT_EN_PWRKEY_R_MASK,
		PMIC_RG_INT_EN_PWRKEY_R_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_homekey_r(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_HOMEKEY_R_ADDR,
		val,
		PMIC_RG_INT_EN_HOMEKEY_R_MASK,
		PMIC_RG_INT_EN_HOMEKEY_R_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_homekey_r(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_HOMEKEY_R_ADDR,
		&val,
		PMIC_RG_INT_EN_HOMEKEY_R_MASK,
		PMIC_RG_INT_EN_HOMEKEY_R_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_ni_lbat_int(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_NI_LBAT_INT_ADDR,
		val,
		PMIC_RG_INT_EN_NI_LBAT_INT_MASK,
		PMIC_RG_INT_EN_NI_LBAT_INT_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_ni_lbat_int(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_NI_LBAT_INT_ADDR,
		&val,
		PMIC_RG_INT_EN_NI_LBAT_INT_MASK,
		PMIC_RG_INT_EN_NI_LBAT_INT_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_chrdet(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_CHRDET_ADDR,
		val,
		PMIC_RG_INT_EN_CHRDET_MASK,
		PMIC_RG_INT_EN_CHRDET_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_chrdet(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_CHRDET_ADDR,
		&val,
		PMIC_RG_INT_EN_CHRDET_MASK,
		PMIC_RG_INT_EN_CHRDET_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_chrdet_edge(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_CHRDET_EDGE_ADDR,
		val,
		PMIC_RG_INT_EN_CHRDET_EDGE_MASK,
		PMIC_RG_INT_EN_CHRDET_EDGE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_chrdet_edge(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_CHRDET_EDGE_ADDR,
		&val,
		PMIC_RG_INT_EN_CHRDET_EDGE_MASK,
		PMIC_RG_INT_EN_CHRDET_EDGE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_baton_lv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_BATON_LV_ADDR,
		val,
		PMIC_RG_INT_EN_BATON_LV_MASK,
		PMIC_RG_INT_EN_BATON_LV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_baton_lv(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_BATON_LV_ADDR,
		&val,
		PMIC_RG_INT_EN_BATON_LV_MASK,
		PMIC_RG_INT_EN_BATON_LV_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_baton_hv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_BATON_HV_ADDR,
		val,
		PMIC_RG_INT_EN_BATON_HV_MASK,
		PMIC_RG_INT_EN_BATON_HV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_baton_hv(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_BATON_HV_ADDR,
		&val,
		PMIC_RG_INT_EN_BATON_HV_MASK,
		PMIC_RG_INT_EN_BATON_HV_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_baton_bat_in(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_BATON_BAT_IN_ADDR,
		val,
		PMIC_RG_INT_EN_BATON_BAT_IN_MASK,
		PMIC_RG_INT_EN_BATON_BAT_IN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_baton_bat_in(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_BATON_BAT_IN_ADDR,
		&val,
		PMIC_RG_INT_EN_BATON_BAT_IN_MASK,
		PMIC_RG_INT_EN_BATON_BAT_IN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_baton_bat_out(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_BATON_BAT_OUT_ADDR,
		val,
		PMIC_RG_INT_EN_BATON_BAT_OUT_MASK,
		PMIC_RG_INT_EN_BATON_BAT_OUT_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_baton_bat_out(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_BATON_BAT_OUT_ADDR,
		&val,
		PMIC_RG_INT_EN_BATON_BAT_OUT_MASK,
		PMIC_RG_INT_EN_BATON_BAT_OUT_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_rtc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_RTC_ADDR,
		val,
		PMIC_RG_INT_EN_RTC_MASK,
		PMIC_RG_INT_EN_RTC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_rtc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_RTC_ADDR,
		&val,
		PMIC_RG_INT_EN_RTC_MASK,
		PMIC_RG_INT_EN_RTC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_rtc_nsec(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_RTC_NSEC_ADDR,
		val,
		PMIC_RG_INT_EN_RTC_NSEC_MASK,
		PMIC_RG_INT_EN_RTC_NSEC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_rtc_nsec(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_RTC_NSEC_ADDR,
		&val,
		PMIC_RG_INT_EN_RTC_NSEC_MASK,
		PMIC_RG_INT_EN_RTC_NSEC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_bif(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_BIF_ADDR,
		val,
		PMIC_RG_INT_EN_BIF_MASK,
		PMIC_RG_INT_EN_BIF_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_bif(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_BIF_ADDR,
		&val,
		PMIC_RG_INT_EN_BIF_MASK,
		PMIC_RG_INT_EN_BIF_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vcdt_hv_det(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VCDT_HV_DET_ADDR,
		val,
		PMIC_RG_INT_EN_VCDT_HV_DET_MASK,
		PMIC_RG_INT_EN_VCDT_HV_DET_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vcdt_hv_det(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VCDT_HV_DET_ADDR,
		&val,
		PMIC_RG_INT_EN_VCDT_HV_DET_MASK,
		PMIC_RG_INT_EN_VCDT_HV_DET_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_thr_h(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_THR_H_ADDR,
		val,
		PMIC_RG_INT_EN_THR_H_MASK,
		PMIC_RG_INT_EN_THR_H_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_thr_h(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_THR_H_ADDR,
		&val,
		PMIC_RG_INT_EN_THR_H_MASK,
		PMIC_RG_INT_EN_THR_H_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_thr_l(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_THR_L_ADDR,
		val,
		PMIC_RG_INT_EN_THR_L_MASK,
		PMIC_RG_INT_EN_THR_L_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_thr_l(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_THR_L_ADDR,
		&val,
		PMIC_RG_INT_EN_THR_L_MASK,
		PMIC_RG_INT_EN_THR_L_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_bat_h(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_BAT_H_ADDR,
		val,
		PMIC_RG_INT_EN_BAT_H_MASK,
		PMIC_RG_INT_EN_BAT_H_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_bat_h(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_BAT_H_ADDR,
		&val,
		PMIC_RG_INT_EN_BAT_H_MASK,
		PMIC_RG_INT_EN_BAT_H_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_bat_l(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_BAT_L_ADDR,
		val,
		PMIC_RG_INT_EN_BAT_L_MASK,
		PMIC_RG_INT_EN_BAT_L_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_bat_l(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_BAT_L_ADDR,
		&val,
		PMIC_RG_INT_EN_BAT_L_MASK,
		PMIC_RG_INT_EN_BAT_L_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_bat2_h(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_BAT2_H_ADDR,
		val,
		PMIC_RG_INT_EN_BAT2_H_MASK,
		PMIC_RG_INT_EN_BAT2_H_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_bat2_h(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_BAT2_H_ADDR,
		&val,
		PMIC_RG_INT_EN_BAT2_H_MASK,
		PMIC_RG_INT_EN_BAT2_H_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_bat2_l(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_BAT2_L_ADDR,
		val,
		PMIC_RG_INT_EN_BAT2_L_MASK,
		PMIC_RG_INT_EN_BAT2_L_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_bat2_l(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_BAT2_L_ADDR,
		&val,
		PMIC_RG_INT_EN_BAT2_L_MASK,
		PMIC_RG_INT_EN_BAT2_L_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_bat_temp_h(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_BAT_TEMP_H_ADDR,
		val,
		PMIC_RG_INT_EN_BAT_TEMP_H_MASK,
		PMIC_RG_INT_EN_BAT_TEMP_H_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_bat_temp_h(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_BAT_TEMP_H_ADDR,
		&val,
		PMIC_RG_INT_EN_BAT_TEMP_H_MASK,
		PMIC_RG_INT_EN_BAT_TEMP_H_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_bat_temp_l(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_BAT_TEMP_L_ADDR,
		val,
		PMIC_RG_INT_EN_BAT_TEMP_L_MASK,
		PMIC_RG_INT_EN_BAT_TEMP_L_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_bat_temp_l(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_BAT_TEMP_L_ADDR,
		&val,
		PMIC_RG_INT_EN_BAT_TEMP_L_MASK,
		PMIC_RG_INT_EN_BAT_TEMP_L_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_auxadc_imp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_AUXADC_IMP_ADDR,
		val,
		PMIC_RG_INT_EN_AUXADC_IMP_MASK,
		PMIC_RG_INT_EN_AUXADC_IMP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_auxadc_imp(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_AUXADC_IMP_ADDR,
		&val,
		PMIC_RG_INT_EN_AUXADC_IMP_MASK,
		PMIC_RG_INT_EN_AUXADC_IMP_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_nag_c_dltv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_NAG_C_DLTV_ADDR,
		val,
		PMIC_RG_INT_EN_NAG_C_DLTV_MASK,
		PMIC_RG_INT_EN_NAG_C_DLTV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_nag_c_dltv(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_NAG_C_DLTV_ADDR,
		&val,
		PMIC_RG_INT_EN_NAG_C_DLTV_MASK,
		PMIC_RG_INT_EN_NAG_C_DLTV_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_jeita_hot(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_JEITA_HOT_ADDR,
		val,
		PMIC_RG_INT_EN_JEITA_HOT_MASK,
		PMIC_RG_INT_EN_JEITA_HOT_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_jeita_hot(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_JEITA_HOT_ADDR,
		&val,
		PMIC_RG_INT_EN_JEITA_HOT_MASK,
		PMIC_RG_INT_EN_JEITA_HOT_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_jeita_warm(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_JEITA_WARM_ADDR,
		val,
		PMIC_RG_INT_EN_JEITA_WARM_MASK,
		PMIC_RG_INT_EN_JEITA_WARM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_jeita_warm(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_JEITA_WARM_ADDR,
		&val,
		PMIC_RG_INT_EN_JEITA_WARM_MASK,
		PMIC_RG_INT_EN_JEITA_WARM_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_jeita_cool(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_JEITA_COOL_ADDR,
		val,
		PMIC_RG_INT_EN_JEITA_COOL_MASK,
		PMIC_RG_INT_EN_JEITA_COOL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_jeita_cool(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_JEITA_COOL_ADDR,
		&val,
		PMIC_RG_INT_EN_JEITA_COOL_MASK,
		PMIC_RG_INT_EN_JEITA_COOL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_jeita_cold(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_JEITA_COLD_ADDR,
		val,
		PMIC_RG_INT_EN_JEITA_COLD_MASK,
		PMIC_RG_INT_EN_JEITA_COLD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_jeita_cold(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_JEITA_COLD_ADDR,
		&val,
		PMIC_RG_INT_EN_JEITA_COLD_MASK,
		PMIC_RG_INT_EN_JEITA_COLD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vproc11_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VPROC11_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VPROC11_OC_MASK,
		PMIC_RG_INT_EN_VPROC11_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vproc11_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VPROC11_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VPROC11_OC_MASK,
		PMIC_RG_INT_EN_VPROC11_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vproc12_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VPROC12_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VPROC12_OC_MASK,
		PMIC_RG_INT_EN_VPROC12_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vproc12_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VPROC12_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VPROC12_OC_MASK,
		PMIC_RG_INT_EN_VPROC12_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vcore_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VCORE_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VCORE_OC_MASK,
		PMIC_RG_INT_EN_VCORE_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vcore_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VCORE_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VCORE_OC_MASK,
		PMIC_RG_INT_EN_VCORE_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vgpu_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VGPU_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VGPU_OC_MASK,
		PMIC_RG_INT_EN_VGPU_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vgpu_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VGPU_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VGPU_OC_MASK,
		PMIC_RG_INT_EN_VGPU_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vdram1_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VDRAM1_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VDRAM1_OC_MASK,
		PMIC_RG_INT_EN_VDRAM1_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vdram1_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VDRAM1_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VDRAM1_OC_MASK,
		PMIC_RG_INT_EN_VDRAM1_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vdram2_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VDRAM2_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VDRAM2_OC_MASK,
		PMIC_RG_INT_EN_VDRAM2_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vdram2_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VDRAM2_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VDRAM2_OC_MASK,
		PMIC_RG_INT_EN_VDRAM2_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vmodem_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VMODEM_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VMODEM_OC_MASK,
		PMIC_RG_INT_EN_VMODEM_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vmodem_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VMODEM_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VMODEM_OC_MASK,
		PMIC_RG_INT_EN_VMODEM_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vs1_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VS1_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VS1_OC_MASK,
		PMIC_RG_INT_EN_VS1_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vs1_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VS1_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VS1_OC_MASK,
		PMIC_RG_INT_EN_VS1_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vs2_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VS2_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VS2_OC_MASK,
		PMIC_RG_INT_EN_VS2_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vs2_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VS2_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VS2_OC_MASK,
		PMIC_RG_INT_EN_VS2_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vpa_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VPA_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VPA_OC_MASK,
		PMIC_RG_INT_EN_VPA_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vpa_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VPA_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VPA_OC_MASK,
		PMIC_RG_INT_EN_VPA_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vcore_preoc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VCORE_PREOC_ADDR,
		val,
		PMIC_RG_INT_EN_VCORE_PREOC_MASK,
		PMIC_RG_INT_EN_VCORE_PREOC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vcore_preoc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VCORE_PREOC_ADDR,
		&val,
		PMIC_RG_INT_EN_VCORE_PREOC_MASK,
		PMIC_RG_INT_EN_VCORE_PREOC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_va10_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VA10_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VA10_OC_MASK,
		PMIC_RG_INT_EN_VA10_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_va10_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VA10_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VA10_OC_MASK,
		PMIC_RG_INT_EN_VA10_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_va12_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VA12_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VA12_OC_MASK,
		PMIC_RG_INT_EN_VA12_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_va12_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VA12_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VA12_OC_MASK,
		PMIC_RG_INT_EN_VA12_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_va18_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VA18_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VA18_OC_MASK,
		PMIC_RG_INT_EN_VA18_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_va18_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VA18_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VA18_OC_MASK,
		PMIC_RG_INT_EN_VA18_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vbif28_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VBIF28_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VBIF28_OC_MASK,
		PMIC_RG_INT_EN_VBIF28_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vbif28_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VBIF28_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VBIF28_OC_MASK,
		PMIC_RG_INT_EN_VBIF28_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vcama1_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VCAMA1_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VCAMA1_OC_MASK,
		PMIC_RG_INT_EN_VCAMA1_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vcama1_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VCAMA1_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VCAMA1_OC_MASK,
		PMIC_RG_INT_EN_VCAMA1_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vcama2_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VCAMA2_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VCAMA2_OC_MASK,
		PMIC_RG_INT_EN_VCAMA2_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vcama2_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VCAMA2_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VCAMA2_OC_MASK,
		PMIC_RG_INT_EN_VCAMA2_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vxo18_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VXO18_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VXO18_OC_MASK,
		PMIC_RG_INT_EN_VXO18_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vxo18_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VXO18_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VXO18_OC_MASK,
		PMIC_RG_INT_EN_VXO18_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vcamd1_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VCAMD1_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VCAMD1_OC_MASK,
		PMIC_RG_INT_EN_VCAMD1_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vcamd1_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VCAMD1_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VCAMD1_OC_MASK,
		PMIC_RG_INT_EN_VCAMD1_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vcamd2_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VCAMD2_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VCAMD2_OC_MASK,
		PMIC_RG_INT_EN_VCAMD2_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vcamd2_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VCAMD2_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VCAMD2_OC_MASK,
		PMIC_RG_INT_EN_VCAMD2_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vcamio_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VCAMIO_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VCAMIO_OC_MASK,
		PMIC_RG_INT_EN_VCAMIO_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vcamio_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VCAMIO_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VCAMIO_OC_MASK,
		PMIC_RG_INT_EN_VCAMIO_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vcn18_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VCN18_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VCN18_OC_MASK,
		PMIC_RG_INT_EN_VCN18_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vcn18_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VCN18_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VCN18_OC_MASK,
		PMIC_RG_INT_EN_VCN18_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vcn28_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VCN28_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VCN28_OC_MASK,
		PMIC_RG_INT_EN_VCN28_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vcn28_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VCN28_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VCN28_OC_MASK,
		PMIC_RG_INT_EN_VCN28_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vcn33_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VCN33_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VCN33_OC_MASK,
		PMIC_RG_INT_EN_VCN33_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vcn33_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VCN33_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VCN33_OC_MASK,
		PMIC_RG_INT_EN_VCN33_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vtcxo24_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VTCXO24_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VTCXO24_OC_MASK,
		PMIC_RG_INT_EN_VTCXO24_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vtcxo24_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VTCXO24_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VTCXO24_OC_MASK,
		PMIC_RG_INT_EN_VTCXO24_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vemc_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VEMC_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VEMC_OC_MASK,
		PMIC_RG_INT_EN_VEMC_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vemc_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VEMC_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VEMC_OC_MASK,
		PMIC_RG_INT_EN_VEMC_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vfe28_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VFE28_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VFE28_OC_MASK,
		PMIC_RG_INT_EN_VFE28_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vfe28_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VFE28_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VFE28_OC_MASK,
		PMIC_RG_INT_EN_VFE28_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vgp_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VGP_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VGP_OC_MASK,
		PMIC_RG_INT_EN_VGP_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vgp_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VGP_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VGP_OC_MASK,
		PMIC_RG_INT_EN_VGP_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vldo28_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VLDO28_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VLDO28_OC_MASK,
		PMIC_RG_INT_EN_VLDO28_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vldo28_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VLDO28_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VLDO28_OC_MASK,
		PMIC_RG_INT_EN_VLDO28_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vio18_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VIO18_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VIO18_OC_MASK,
		PMIC_RG_INT_EN_VIO18_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vio18_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VIO18_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VIO18_OC_MASK,
		PMIC_RG_INT_EN_VIO18_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vio28_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VIO28_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VIO28_OC_MASK,
		PMIC_RG_INT_EN_VIO28_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vio28_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VIO28_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VIO28_OC_MASK,
		PMIC_RG_INT_EN_VIO28_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vmc_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VMC_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VMC_OC_MASK,
		PMIC_RG_INT_EN_VMC_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vmc_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VMC_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VMC_OC_MASK,
		PMIC_RG_INT_EN_VMC_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vmch_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VMCH_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VMCH_OC_MASK,
		PMIC_RG_INT_EN_VMCH_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vmch_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VMCH_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VMCH_OC_MASK,
		PMIC_RG_INT_EN_VMCH_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vmipi_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VMIPI_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VMIPI_OC_MASK,
		PMIC_RG_INT_EN_VMIPI_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vmipi_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VMIPI_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VMIPI_OC_MASK,
		PMIC_RG_INT_EN_VMIPI_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vrf12_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VRF12_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VRF12_OC_MASK,
		PMIC_RG_INT_EN_VRF12_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vrf12_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VRF12_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VRF12_OC_MASK,
		PMIC_RG_INT_EN_VRF12_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vrf18_1_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VRF18_1_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VRF18_1_OC_MASK,
		PMIC_RG_INT_EN_VRF18_1_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vrf18_1_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VRF18_1_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VRF18_1_OC_MASK,
		PMIC_RG_INT_EN_VRF18_1_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vrf18_2_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VRF18_2_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VRF18_2_OC_MASK,
		PMIC_RG_INT_EN_VRF18_2_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vrf18_2_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VRF18_2_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VRF18_2_OC_MASK,
		PMIC_RG_INT_EN_VRF18_2_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vsim1_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VSIM1_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VSIM1_OC_MASK,
		PMIC_RG_INT_EN_VSIM1_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vsim1_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VSIM1_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VSIM1_OC_MASK,
		PMIC_RG_INT_EN_VSIM1_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vsim2_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VSIM2_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VSIM2_OC_MASK,
		PMIC_RG_INT_EN_VSIM2_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vsim2_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VSIM2_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VSIM2_OC_MASK,
		PMIC_RG_INT_EN_VSIM2_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vgp2_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VGP2_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VGP2_OC_MASK,
		PMIC_RG_INT_EN_VGP2_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vgp2_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VGP2_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VGP2_OC_MASK,
		PMIC_RG_INT_EN_VGP2_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vsram_core_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VSRAM_CORE_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VSRAM_CORE_OC_MASK,
		PMIC_RG_INT_EN_VSRAM_CORE_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vsram_core_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VSRAM_CORE_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VSRAM_CORE_OC_MASK,
		PMIC_RG_INT_EN_VSRAM_CORE_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vsram_proc_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VSRAM_PROC_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VSRAM_PROC_OC_MASK,
		PMIC_RG_INT_EN_VSRAM_PROC_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vsram_proc_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VSRAM_PROC_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VSRAM_PROC_OC_MASK,
		PMIC_RG_INT_EN_VSRAM_PROC_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vsram_gpu_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VSRAM_GPU_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VSRAM_GPU_OC_MASK,
		PMIC_RG_INT_EN_VSRAM_GPU_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vsram_gpu_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VSRAM_GPU_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VSRAM_GPU_OC_MASK,
		PMIC_RG_INT_EN_VSRAM_GPU_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vsram_md_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VSRAM_MD_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VSRAM_MD_OC_MASK,
		PMIC_RG_INT_EN_VSRAM_MD_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vsram_md_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VSRAM_MD_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VSRAM_MD_OC_MASK,
		PMIC_RG_INT_EN_VSRAM_MD_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vufs18_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VUFS18_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VUFS18_OC_MASK,
		PMIC_RG_INT_EN_VUFS18_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vufs18_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VUFS18_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VUFS18_OC_MASK,
		PMIC_RG_INT_EN_VUFS18_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vusb33_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VUSB33_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VUSB33_OC_MASK,
		PMIC_RG_INT_EN_VUSB33_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vusb33_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VUSB33_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VUSB33_OC_MASK,
		PMIC_RG_INT_EN_VUSB33_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_vxo22_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_VXO22_OC_ADDR,
		val,
		PMIC_RG_INT_EN_VXO22_OC_MASK,
		PMIC_RG_INT_EN_VXO22_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_vxo22_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_VXO22_OC_ADDR,
		&val,
		PMIC_RG_INT_EN_VXO22_OC_MASK,
		PMIC_RG_INT_EN_VXO22_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_fg_bat0_h(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_FG_BAT0_H_ADDR,
		val,
		PMIC_RG_INT_EN_FG_BAT0_H_MASK,
		PMIC_RG_INT_EN_FG_BAT0_H_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_fg_bat0_h(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_FG_BAT0_H_ADDR,
		&val,
		PMIC_RG_INT_EN_FG_BAT0_H_MASK,
		PMIC_RG_INT_EN_FG_BAT0_H_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_fg_bat0_l(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_FG_BAT0_L_ADDR,
		val,
		PMIC_RG_INT_EN_FG_BAT0_L_MASK,
		PMIC_RG_INT_EN_FG_BAT0_L_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_fg_bat0_l(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_FG_BAT0_L_ADDR,
		&val,
		PMIC_RG_INT_EN_FG_BAT0_L_MASK,
		PMIC_RG_INT_EN_FG_BAT0_L_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_fg_cur_h(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_FG_CUR_H_ADDR,
		val,
		PMIC_RG_INT_EN_FG_CUR_H_MASK,
		PMIC_RG_INT_EN_FG_CUR_H_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_fg_cur_h(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_FG_CUR_H_ADDR,
		&val,
		PMIC_RG_INT_EN_FG_CUR_H_MASK,
		PMIC_RG_INT_EN_FG_CUR_H_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_fg_cur_l(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_FG_CUR_L_ADDR,
		val,
		PMIC_RG_INT_EN_FG_CUR_L_MASK,
		PMIC_RG_INT_EN_FG_CUR_L_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_fg_cur_l(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_FG_CUR_L_ADDR,
		&val,
		PMIC_RG_INT_EN_FG_CUR_L_MASK,
		PMIC_RG_INT_EN_FG_CUR_L_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_fg_zcv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_FG_ZCV_ADDR,
		val,
		PMIC_RG_INT_EN_FG_ZCV_MASK,
		PMIC_RG_INT_EN_FG_ZCV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_fg_zcv(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_FG_ZCV_ADDR,
		&val,
		PMIC_RG_INT_EN_FG_ZCV_MASK,
		PMIC_RG_INT_EN_FG_ZCV_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_fg_bat1_h(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_FG_BAT1_H_ADDR,
		val,
		PMIC_RG_INT_EN_FG_BAT1_H_MASK,
		PMIC_RG_INT_EN_FG_BAT1_H_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_fg_bat1_h(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_FG_BAT1_H_ADDR,
		&val,
		PMIC_RG_INT_EN_FG_BAT1_H_MASK,
		PMIC_RG_INT_EN_FG_BAT1_H_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_fg_bat1_l(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_FG_BAT1_L_ADDR,
		val,
		PMIC_RG_INT_EN_FG_BAT1_L_MASK,
		PMIC_RG_INT_EN_FG_BAT1_L_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_fg_bat1_l(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_FG_BAT1_L_ADDR,
		&val,
		PMIC_RG_INT_EN_FG_BAT1_L_MASK,
		PMIC_RG_INT_EN_FG_BAT1_L_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_fg_n_charge_l(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_FG_N_CHARGE_L_ADDR,
		val,
		PMIC_RG_INT_EN_FG_N_CHARGE_L_MASK,
		PMIC_RG_INT_EN_FG_N_CHARGE_L_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_fg_n_charge_l(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_FG_N_CHARGE_L_ADDR,
		&val,
		PMIC_RG_INT_EN_FG_N_CHARGE_L_MASK,
		PMIC_RG_INT_EN_FG_N_CHARGE_L_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_fg_iavg_h(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_FG_IAVG_H_ADDR,
		val,
		PMIC_RG_INT_EN_FG_IAVG_H_MASK,
		PMIC_RG_INT_EN_FG_IAVG_H_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_fg_iavg_h(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_FG_IAVG_H_ADDR,
		&val,
		PMIC_RG_INT_EN_FG_IAVG_H_MASK,
		PMIC_RG_INT_EN_FG_IAVG_H_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_fg_iavg_l(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_FG_IAVG_L_ADDR,
		val,
		PMIC_RG_INT_EN_FG_IAVG_L_MASK,
		PMIC_RG_INT_EN_FG_IAVG_L_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_fg_iavg_l(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_FG_IAVG_L_ADDR,
		&val,
		PMIC_RG_INT_EN_FG_IAVG_L_MASK,
		PMIC_RG_INT_EN_FG_IAVG_L_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_fg_time_h(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_FG_TIME_H_ADDR,
		val,
		PMIC_RG_INT_EN_FG_TIME_H_MASK,
		PMIC_RG_INT_EN_FG_TIME_H_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_fg_time_h(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_FG_TIME_H_ADDR,
		&val,
		PMIC_RG_INT_EN_FG_TIME_H_MASK,
		PMIC_RG_INT_EN_FG_TIME_H_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_fg_discharge(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_FG_DISCHARGE_ADDR,
		val,
		PMIC_RG_INT_EN_FG_DISCHARGE_MASK,
		PMIC_RG_INT_EN_FG_DISCHARGE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_fg_discharge(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_FG_DISCHARGE_ADDR,
		&val,
		PMIC_RG_INT_EN_FG_DISCHARGE_MASK,
		PMIC_RG_INT_EN_FG_DISCHARGE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_fg_charge(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_FG_CHARGE_ADDR,
		val,
		PMIC_RG_INT_EN_FG_CHARGE_MASK,
		PMIC_RG_INT_EN_FG_CHARGE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_fg_charge(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_FG_CHARGE_ADDR,
		&val,
		PMIC_RG_INT_EN_FG_CHARGE_MASK,
		PMIC_RG_INT_EN_FG_CHARGE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_con5(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_CON5_ADDR,
		val,
		PMIC_RG_INT_EN_CON5_MASK,
		PMIC_RG_INT_EN_CON5_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_con5(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_CON5_ADDR,
		&val,
		PMIC_RG_INT_EN_CON5_MASK,
		PMIC_RG_INT_EN_CON5_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_audio(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_AUDIO_ADDR,
		val,
		PMIC_RG_INT_EN_AUDIO_MASK,
		PMIC_RG_INT_EN_AUDIO_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_audio(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_AUDIO_ADDR,
		&val,
		PMIC_RG_INT_EN_AUDIO_MASK,
		PMIC_RG_INT_EN_AUDIO_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_mad(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_MAD_ADDR,
		val,
		PMIC_RG_INT_EN_MAD_MASK,
		PMIC_RG_INT_EN_MAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_mad(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_MAD_ADDR,
		&val,
		PMIC_RG_INT_EN_MAD_MASK,
		PMIC_RG_INT_EN_MAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_eint_rtc32k_1v8_1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_EINT_RTC32K_1V8_1_ADDR,
		val,
		PMIC_RG_INT_EN_EINT_RTC32K_1V8_1_MASK,
		PMIC_RG_INT_EN_EINT_RTC32K_1V8_1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_eint_rtc32k_1v8_1(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_EINT_RTC32K_1V8_1_ADDR,
		&val,
		PMIC_RG_INT_EN_EINT_RTC32K_1V8_1_MASK,
		PMIC_RG_INT_EN_EINT_RTC32K_1V8_1_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_eint_aud_clk(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_EINT_AUD_CLK_ADDR,
		val,
		PMIC_RG_INT_EN_EINT_AUD_CLK_MASK,
		PMIC_RG_INT_EN_EINT_AUD_CLK_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_eint_aud_clk(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_EINT_AUD_CLK_ADDR,
		&val,
		PMIC_RG_INT_EN_EINT_AUD_CLK_MASK,
		PMIC_RG_INT_EN_EINT_AUD_CLK_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_eint_aud_dat_mosi(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_EINT_AUD_DAT_MOSI_ADDR,
		val,
		PMIC_RG_INT_EN_EINT_AUD_DAT_MOSI_MASK,
		PMIC_RG_INT_EN_EINT_AUD_DAT_MOSI_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_eint_aud_dat_mosi(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_EINT_AUD_DAT_MOSI_ADDR,
		&val,
		PMIC_RG_INT_EN_EINT_AUD_DAT_MOSI_MASK,
		PMIC_RG_INT_EN_EINT_AUD_DAT_MOSI_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_eint_aud_dat_miso(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_EINT_AUD_DAT_MISO_ADDR,
		val,
		PMIC_RG_INT_EN_EINT_AUD_DAT_MISO_MASK,
		PMIC_RG_INT_EN_EINT_AUD_DAT_MISO_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_eint_aud_dat_miso(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_EINT_AUD_DAT_MISO_ADDR,
		&val,
		PMIC_RG_INT_EN_EINT_AUD_DAT_MISO_MASK,
		PMIC_RG_INT_EN_EINT_AUD_DAT_MISO_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_eint_vow_clk_miso(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_EINT_VOW_CLK_MISO_ADDR,
		val,
		PMIC_RG_INT_EN_EINT_VOW_CLK_MISO_MASK,
		PMIC_RG_INT_EN_EINT_VOW_CLK_MISO_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_eint_vow_clk_miso(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_EINT_VOW_CLK_MISO_ADDR,
		&val,
		PMIC_RG_INT_EN_EINT_VOW_CLK_MISO_MASK,
		PMIC_RG_INT_EN_EINT_VOW_CLK_MISO_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_accdet(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_ACCDET_ADDR,
		val,
		PMIC_RG_INT_EN_ACCDET_MASK,
		PMIC_RG_INT_EN_ACCDET_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_accdet(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_ACCDET_ADDR,
		&val,
		PMIC_RG_INT_EN_ACCDET_MASK,
		PMIC_RG_INT_EN_ACCDET_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_accdet_eint(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_ACCDET_EINT_ADDR,
		val,
		PMIC_RG_INT_EN_ACCDET_EINT_MASK,
		PMIC_RG_INT_EN_ACCDET_EINT_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_accdet_eint(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_ACCDET_EINT_ADDR,
		&val,
		PMIC_RG_INT_EN_ACCDET_EINT_MASK,
		PMIC_RG_INT_EN_ACCDET_EINT_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_en_spi_cmd_alert(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_EN_SPI_CMD_ALERT_ADDR,
		val,
		PMIC_RG_INT_EN_SPI_CMD_ALERT_MASK,
		PMIC_RG_INT_EN_SPI_CMD_ALERT_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_en_spi_cmd_alert(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_EN_SPI_CMD_ALERT_ADDR,
		&val,
		PMIC_RG_INT_EN_SPI_CMD_ALERT_MASK,
		PMIC_RG_INT_EN_SPI_CMD_ALERT_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_int_mask_pwrkey(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_PWRKEY_ADDR,
		val,
		PMIC_RG_INT_MASK_PWRKEY_MASK,
		PMIC_RG_INT_MASK_PWRKEY_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_homekey(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_HOMEKEY_ADDR,
		val,
		PMIC_RG_INT_MASK_HOMEKEY_MASK,
		PMIC_RG_INT_MASK_HOMEKEY_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_pwrkey_r(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_PWRKEY_R_ADDR,
		val,
		PMIC_RG_INT_MASK_PWRKEY_R_MASK,
		PMIC_RG_INT_MASK_PWRKEY_R_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_homekey_r(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_HOMEKEY_R_ADDR,
		val,
		PMIC_RG_INT_MASK_HOMEKEY_R_MASK,
		PMIC_RG_INT_MASK_HOMEKEY_R_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_ni_lbat_int(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_NI_LBAT_INT_ADDR,
		val,
		PMIC_RG_INT_MASK_NI_LBAT_INT_MASK,
		PMIC_RG_INT_MASK_NI_LBAT_INT_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_chrdet(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_CHRDET_ADDR,
		val,
		PMIC_RG_INT_MASK_CHRDET_MASK,
		PMIC_RG_INT_MASK_CHRDET_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_chrdet_edge(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_CHRDET_EDGE_ADDR,
		val,
		PMIC_RG_INT_MASK_CHRDET_EDGE_MASK,
		PMIC_RG_INT_MASK_CHRDET_EDGE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_baton_lv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_BATON_LV_ADDR,
		val,
		PMIC_RG_INT_MASK_BATON_LV_MASK,
		PMIC_RG_INT_MASK_BATON_LV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_baton_hv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_BATON_HV_ADDR,
		val,
		PMIC_RG_INT_MASK_BATON_HV_MASK,
		PMIC_RG_INT_MASK_BATON_HV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_baton_bat_in(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_BATON_BAT_IN_ADDR,
		val,
		PMIC_RG_INT_MASK_BATON_BAT_IN_MASK,
		PMIC_RG_INT_MASK_BATON_BAT_IN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_baton_bat_out(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_BATON_BAT_OUT_ADDR,
		val,
		PMIC_RG_INT_MASK_BATON_BAT_OUT_MASK,
		PMIC_RG_INT_MASK_BATON_BAT_OUT_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_rtc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_RTC_ADDR,
		val,
		PMIC_RG_INT_MASK_RTC_MASK,
		PMIC_RG_INT_MASK_RTC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_rtc_nsec(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_RTC_NSEC_ADDR,
		val,
		PMIC_RG_INT_MASK_RTC_NSEC_MASK,
		PMIC_RG_INT_MASK_RTC_NSEC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_bif(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_BIF_ADDR,
		val,
		PMIC_RG_INT_MASK_BIF_MASK,
		PMIC_RG_INT_MASK_BIF_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vcdt_hv_det(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VCDT_HV_DET_ADDR,
		val,
		PMIC_RG_INT_MASK_VCDT_HV_DET_MASK,
		PMIC_RG_INT_MASK_VCDT_HV_DET_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_thr_h(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_THR_H_ADDR,
		val,
		PMIC_RG_INT_MASK_THR_H_MASK,
		PMIC_RG_INT_MASK_THR_H_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_thr_l(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_THR_L_ADDR,
		val,
		PMIC_RG_INT_MASK_THR_L_MASK,
		PMIC_RG_INT_MASK_THR_L_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_bat_h(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_BAT_H_ADDR,
		val,
		PMIC_RG_INT_MASK_BAT_H_MASK,
		PMIC_RG_INT_MASK_BAT_H_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_bat_l(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_BAT_L_ADDR,
		val,
		PMIC_RG_INT_MASK_BAT_L_MASK,
		PMIC_RG_INT_MASK_BAT_L_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_bat2_h(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_BAT2_H_ADDR,
		val,
		PMIC_RG_INT_MASK_BAT2_H_MASK,
		PMIC_RG_INT_MASK_BAT2_H_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_bat2_l(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_BAT2_L_ADDR,
		val,
		PMIC_RG_INT_MASK_BAT2_L_MASK,
		PMIC_RG_INT_MASK_BAT2_L_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_bat_temp_h(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_BAT_TEMP_H_ADDR,
		val,
		PMIC_RG_INT_MASK_BAT_TEMP_H_MASK,
		PMIC_RG_INT_MASK_BAT_TEMP_H_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_bat_temp_l(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_BAT_TEMP_L_ADDR,
		val,
		PMIC_RG_INT_MASK_BAT_TEMP_L_MASK,
		PMIC_RG_INT_MASK_BAT_TEMP_L_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_auxadc_imp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_AUXADC_IMP_ADDR,
		val,
		PMIC_RG_INT_MASK_AUXADC_IMP_MASK,
		PMIC_RG_INT_MASK_AUXADC_IMP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_nag_c_dltv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_NAG_C_DLTV_ADDR,
		val,
		PMIC_RG_INT_MASK_NAG_C_DLTV_MASK,
		PMIC_RG_INT_MASK_NAG_C_DLTV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_jeita_hot(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_JEITA_HOT_ADDR,
		val,
		PMIC_RG_INT_MASK_JEITA_HOT_MASK,
		PMIC_RG_INT_MASK_JEITA_HOT_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_jeita_warm(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_JEITA_WARM_ADDR,
		val,
		PMIC_RG_INT_MASK_JEITA_WARM_MASK,
		PMIC_RG_INT_MASK_JEITA_WARM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_jeita_cool(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_JEITA_COOL_ADDR,
		val,
		PMIC_RG_INT_MASK_JEITA_COOL_MASK,
		PMIC_RG_INT_MASK_JEITA_COOL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_jeita_cold(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_JEITA_COLD_ADDR,
		val,
		PMIC_RG_INT_MASK_JEITA_COLD_MASK,
		PMIC_RG_INT_MASK_JEITA_COLD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vproc11_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VPROC11_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VPROC11_OC_MASK,
		PMIC_RG_INT_MASK_VPROC11_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vproc12_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VPROC12_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VPROC12_OC_MASK,
		PMIC_RG_INT_MASK_VPROC12_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vcore_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VCORE_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VCORE_OC_MASK,
		PMIC_RG_INT_MASK_VCORE_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vgpu_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VGPU_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VGPU_OC_MASK,
		PMIC_RG_INT_MASK_VGPU_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vdram1_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VDRAM1_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VDRAM1_OC_MASK,
		PMIC_RG_INT_MASK_VDRAM1_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vdram2_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VDRAM2_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VDRAM2_OC_MASK,
		PMIC_RG_INT_MASK_VDRAM2_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vmodem_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VMODEM_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VMODEM_OC_MASK,
		PMIC_RG_INT_MASK_VMODEM_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vs1_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VS1_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VS1_OC_MASK,
		PMIC_RG_INT_MASK_VS1_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vs2_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VS2_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VS2_OC_MASK,
		PMIC_RG_INT_MASK_VS2_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vpa_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VPA_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VPA_OC_MASK,
		PMIC_RG_INT_MASK_VPA_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vcore_preoc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VCORE_PREOC_ADDR,
		val,
		PMIC_RG_INT_MASK_VCORE_PREOC_MASK,
		PMIC_RG_INT_MASK_VCORE_PREOC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_va10_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VA10_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VA10_OC_MASK,
		PMIC_RG_INT_MASK_VA10_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_va12_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VA12_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VA12_OC_MASK,
		PMIC_RG_INT_MASK_VA12_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_va18_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VA18_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VA18_OC_MASK,
		PMIC_RG_INT_MASK_VA18_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vbif28_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VBIF28_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VBIF28_OC_MASK,
		PMIC_RG_INT_MASK_VBIF28_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vcama1_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VCAMA1_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VCAMA1_OC_MASK,
		PMIC_RG_INT_MASK_VCAMA1_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vcama2_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VCAMA2_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VCAMA2_OC_MASK,
		PMIC_RG_INT_MASK_VCAMA2_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vxo18_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VXO18_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VXO18_OC_MASK,
		PMIC_RG_INT_MASK_VXO18_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vcamd1_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VCAMD1_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VCAMD1_OC_MASK,
		PMIC_RG_INT_MASK_VCAMD1_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vcamd2_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VCAMD2_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VCAMD2_OC_MASK,
		PMIC_RG_INT_MASK_VCAMD2_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vcamio_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VCAMIO_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VCAMIO_OC_MASK,
		PMIC_RG_INT_MASK_VCAMIO_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vcn18_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VCN18_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VCN18_OC_MASK,
		PMIC_RG_INT_MASK_VCN18_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vcn28_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VCN28_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VCN28_OC_MASK,
		PMIC_RG_INT_MASK_VCN28_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vcn33_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VCN33_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VCN33_OC_MASK,
		PMIC_RG_INT_MASK_VCN33_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vtcxo24_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VTCXO24_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VTCXO24_OC_MASK,
		PMIC_RG_INT_MASK_VTCXO24_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vemc_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VEMC_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VEMC_OC_MASK,
		PMIC_RG_INT_MASK_VEMC_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vfe28_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VFE28_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VFE28_OC_MASK,
		PMIC_RG_INT_MASK_VFE28_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vgp_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VGP_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VGP_OC_MASK,
		PMIC_RG_INT_MASK_VGP_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vldo28_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VLDO28_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VLDO28_OC_MASK,
		PMIC_RG_INT_MASK_VLDO28_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vio18_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VIO18_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VIO18_OC_MASK,
		PMIC_RG_INT_MASK_VIO18_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vio28_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VIO28_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VIO28_OC_MASK,
		PMIC_RG_INT_MASK_VIO28_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vmc_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VMC_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VMC_OC_MASK,
		PMIC_RG_INT_MASK_VMC_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vmch_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VMCH_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VMCH_OC_MASK,
		PMIC_RG_INT_MASK_VMCH_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vmipi_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VMIPI_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VMIPI_OC_MASK,
		PMIC_RG_INT_MASK_VMIPI_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vrf12_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VRF12_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VRF12_OC_MASK,
		PMIC_RG_INT_MASK_VRF12_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vrf18_1_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VRF18_1_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VRF18_1_OC_MASK,
		PMIC_RG_INT_MASK_VRF18_1_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vrf18_2_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VRF18_2_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VRF18_2_OC_MASK,
		PMIC_RG_INT_MASK_VRF18_2_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vsim1_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VSIM1_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VSIM1_OC_MASK,
		PMIC_RG_INT_MASK_VSIM1_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vsim2_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VSIM2_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VSIM2_OC_MASK,
		PMIC_RG_INT_MASK_VSIM2_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vgp2_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VGP2_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VGP2_OC_MASK,
		PMIC_RG_INT_MASK_VGP2_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vsram_core_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VSRAM_CORE_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VSRAM_CORE_OC_MASK,
		PMIC_RG_INT_MASK_VSRAM_CORE_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vsram_proc_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VSRAM_PROC_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VSRAM_PROC_OC_MASK,
		PMIC_RG_INT_MASK_VSRAM_PROC_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vsram_gpu_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VSRAM_GPU_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VSRAM_GPU_OC_MASK,
		PMIC_RG_INT_MASK_VSRAM_GPU_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vsram_md_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VSRAM_MD_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VSRAM_MD_OC_MASK,
		PMIC_RG_INT_MASK_VSRAM_MD_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vufs18_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VUFS18_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VUFS18_OC_MASK,
		PMIC_RG_INT_MASK_VUFS18_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vusb33_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VUSB33_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VUSB33_OC_MASK,
		PMIC_RG_INT_MASK_VUSB33_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_vxo22_oc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_VXO22_OC_ADDR,
		val,
		PMIC_RG_INT_MASK_VXO22_OC_MASK,
		PMIC_RG_INT_MASK_VXO22_OC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_fg_bat0_h(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_FG_BAT0_H_ADDR,
		val,
		PMIC_RG_INT_MASK_FG_BAT0_H_MASK,
		PMIC_RG_INT_MASK_FG_BAT0_H_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_fg_bat0_l(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_FG_BAT0_L_ADDR,
		val,
		PMIC_RG_INT_MASK_FG_BAT0_L_MASK,
		PMIC_RG_INT_MASK_FG_BAT0_L_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_fg_cur_h(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_FG_CUR_H_ADDR,
		val,
		PMIC_RG_INT_MASK_FG_CUR_H_MASK,
		PMIC_RG_INT_MASK_FG_CUR_H_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_fg_cur_l(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_FG_CUR_L_ADDR,
		val,
		PMIC_RG_INT_MASK_FG_CUR_L_MASK,
		PMIC_RG_INT_MASK_FG_CUR_L_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_fg_zcv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_FG_ZCV_ADDR,
		val,
		PMIC_RG_INT_MASK_FG_ZCV_MASK,
		PMIC_RG_INT_MASK_FG_ZCV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_fg_bat1_h(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_FG_BAT1_H_ADDR,
		val,
		PMIC_RG_INT_MASK_FG_BAT1_H_MASK,
		PMIC_RG_INT_MASK_FG_BAT1_H_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_fg_bat1_l(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_FG_BAT1_L_ADDR,
		val,
		PMIC_RG_INT_MASK_FG_BAT1_L_MASK,
		PMIC_RG_INT_MASK_FG_BAT1_L_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_fg_n_charge_l(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_FG_N_CHARGE_L_ADDR,
		val,
		PMIC_RG_INT_MASK_FG_N_CHARGE_L_MASK,
		PMIC_RG_INT_MASK_FG_N_CHARGE_L_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_fg_iavg_h(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_FG_IAVG_H_ADDR,
		val,
		PMIC_RG_INT_MASK_FG_IAVG_H_MASK,
		PMIC_RG_INT_MASK_FG_IAVG_H_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_fg_iavg_l(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_FG_IAVG_L_ADDR,
		val,
		PMIC_RG_INT_MASK_FG_IAVG_L_MASK,
		PMIC_RG_INT_MASK_FG_IAVG_L_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_fg_time_h(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_FG_TIME_H_ADDR,
		val,
		PMIC_RG_INT_MASK_FG_TIME_H_MASK,
		PMIC_RG_INT_MASK_FG_TIME_H_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_fg_discharge(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_FG_DISCHARGE_ADDR,
		val,
		PMIC_RG_INT_MASK_FG_DISCHARGE_MASK,
		PMIC_RG_INT_MASK_FG_DISCHARGE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_fg_charge(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_FG_CHARGE_ADDR,
		val,
		PMIC_RG_INT_MASK_FG_CHARGE_MASK,
		PMIC_RG_INT_MASK_FG_CHARGE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_con5(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_CON5_ADDR,
		val,
		PMIC_RG_INT_MASK_CON5_MASK,
		PMIC_RG_INT_MASK_CON5_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_audio(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_AUDIO_ADDR,
		val,
		PMIC_RG_INT_MASK_AUDIO_MASK,
		PMIC_RG_INT_MASK_AUDIO_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_mad(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_MAD_ADDR,
		val,
		PMIC_RG_INT_MASK_MAD_MASK,
		PMIC_RG_INT_MASK_MAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_eint_rtc32k_1v8_1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_EINT_RTC32K_1V8_1_ADDR,
		val,
		PMIC_RG_INT_MASK_EINT_RTC32K_1V8_1_MASK,
		PMIC_RG_INT_MASK_EINT_RTC32K_1V8_1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_eint_aud_clk(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_EINT_AUD_CLK_ADDR,
		val,
		PMIC_RG_INT_MASK_EINT_AUD_CLK_MASK,
		PMIC_RG_INT_MASK_EINT_AUD_CLK_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_eint_aud_dat_mosi(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_EINT_AUD_DAT_MOSI_ADDR,
		val,
		PMIC_RG_INT_MASK_EINT_AUD_DAT_MOSI_MASK,
		PMIC_RG_INT_MASK_EINT_AUD_DAT_MOSI_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_eint_aud_dat_miso(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_EINT_AUD_DAT_MISO_ADDR,
		val,
		PMIC_RG_INT_MASK_EINT_AUD_DAT_MISO_MASK,
		PMIC_RG_INT_MASK_EINT_AUD_DAT_MISO_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_eint_vow_clk_miso(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_EINT_VOW_CLK_MISO_ADDR,
		val,
		PMIC_RG_INT_MASK_EINT_VOW_CLK_MISO_MASK,
		PMIC_RG_INT_MASK_EINT_VOW_CLK_MISO_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_accdet(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_ACCDET_ADDR,
		val,
		PMIC_RG_INT_MASK_ACCDET_MASK,
		PMIC_RG_INT_MASK_ACCDET_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_accdet_eint(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_ACCDET_EINT_ADDR,
		val,
		PMIC_RG_INT_MASK_ACCDET_EINT_MASK,
		PMIC_RG_INT_MASK_ACCDET_EINT_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_mask_spi_cmd_alert(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_MASK_SPI_CMD_ALERT_ADDR,
		val,
		PMIC_RG_INT_MASK_SPI_CMD_ALERT_MASK,
		PMIC_RG_INT_MASK_SPI_CMD_ALERT_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_int_status_pwrkey(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_PWRKEY_ADDR,
		&val,
		PMIC_RG_INT_STATUS_PWRKEY_MASK,
		PMIC_RG_INT_STATUS_PWRKEY_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_homekey(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_HOMEKEY_ADDR,
		&val,
		PMIC_RG_INT_STATUS_HOMEKEY_MASK,
		PMIC_RG_INT_STATUS_HOMEKEY_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_pwrkey_r(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_PWRKEY_R_ADDR,
		&val,
		PMIC_RG_INT_STATUS_PWRKEY_R_MASK,
		PMIC_RG_INT_STATUS_PWRKEY_R_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_homekey_r(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_HOMEKEY_R_ADDR,
		&val,
		PMIC_RG_INT_STATUS_HOMEKEY_R_MASK,
		PMIC_RG_INT_STATUS_HOMEKEY_R_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_ni_lbat_int(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_NI_LBAT_INT_ADDR,
		&val,
		PMIC_RG_INT_STATUS_NI_LBAT_INT_MASK,
		PMIC_RG_INT_STATUS_NI_LBAT_INT_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_chrdet(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_CHRDET_ADDR,
		&val,
		PMIC_RG_INT_STATUS_CHRDET_MASK,
		PMIC_RG_INT_STATUS_CHRDET_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_chrdet_edge(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_CHRDET_EDGE_ADDR,
		&val,
		PMIC_RG_INT_STATUS_CHRDET_EDGE_MASK,
		PMIC_RG_INT_STATUS_CHRDET_EDGE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_baton_lv(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_BATON_LV_ADDR,
		&val,
		PMIC_RG_INT_STATUS_BATON_LV_MASK,
		PMIC_RG_INT_STATUS_BATON_LV_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_baton_hv(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_BATON_HV_ADDR,
		&val,
		PMIC_RG_INT_STATUS_BATON_HV_MASK,
		PMIC_RG_INT_STATUS_BATON_HV_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_baton_bat_in(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_BATON_BAT_IN_ADDR,
		&val,
		PMIC_RG_INT_STATUS_BATON_BAT_IN_MASK,
		PMIC_RG_INT_STATUS_BATON_BAT_IN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_baton_bat_out(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_BATON_BAT_OUT_ADDR,
		&val,
		PMIC_RG_INT_STATUS_BATON_BAT_OUT_MASK,
		PMIC_RG_INT_STATUS_BATON_BAT_OUT_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_rtc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_RTC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_RTC_MASK,
		PMIC_RG_INT_STATUS_RTC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_rtc_nsec(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_RTC_NSEC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_RTC_NSEC_MASK,
		PMIC_RG_INT_STATUS_RTC_NSEC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_bif(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_BIF_ADDR,
		&val,
		PMIC_RG_INT_STATUS_BIF_MASK,
		PMIC_RG_INT_STATUS_BIF_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vcdt_hv_det(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VCDT_HV_DET_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VCDT_HV_DET_MASK,
		PMIC_RG_INT_STATUS_VCDT_HV_DET_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_thr_h(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_THR_H_ADDR,
		&val,
		PMIC_RG_INT_STATUS_THR_H_MASK,
		PMIC_RG_INT_STATUS_THR_H_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_thr_l(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_THR_L_ADDR,
		&val,
		PMIC_RG_INT_STATUS_THR_L_MASK,
		PMIC_RG_INT_STATUS_THR_L_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_bat_h(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_BAT_H_ADDR,
		&val,
		PMIC_RG_INT_STATUS_BAT_H_MASK,
		PMIC_RG_INT_STATUS_BAT_H_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_bat_l(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_BAT_L_ADDR,
		&val,
		PMIC_RG_INT_STATUS_BAT_L_MASK,
		PMIC_RG_INT_STATUS_BAT_L_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_bat2_h(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_BAT2_H_ADDR,
		&val,
		PMIC_RG_INT_STATUS_BAT2_H_MASK,
		PMIC_RG_INT_STATUS_BAT2_H_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_bat2_l(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_BAT2_L_ADDR,
		&val,
		PMIC_RG_INT_STATUS_BAT2_L_MASK,
		PMIC_RG_INT_STATUS_BAT2_L_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_bat_temp_h(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_BAT_TEMP_H_ADDR,
		&val,
		PMIC_RG_INT_STATUS_BAT_TEMP_H_MASK,
		PMIC_RG_INT_STATUS_BAT_TEMP_H_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_bat_temp_l(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_BAT_TEMP_L_ADDR,
		&val,
		PMIC_RG_INT_STATUS_BAT_TEMP_L_MASK,
		PMIC_RG_INT_STATUS_BAT_TEMP_L_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_auxadc_imp(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_AUXADC_IMP_ADDR,
		&val,
		PMIC_RG_INT_STATUS_AUXADC_IMP_MASK,
		PMIC_RG_INT_STATUS_AUXADC_IMP_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_nag_c_dltv(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_NAG_C_DLTV_ADDR,
		&val,
		PMIC_RG_INT_STATUS_NAG_C_DLTV_MASK,
		PMIC_RG_INT_STATUS_NAG_C_DLTV_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_jeita_hot(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_JEITA_HOT_ADDR,
		&val,
		PMIC_RG_INT_STATUS_JEITA_HOT_MASK,
		PMIC_RG_INT_STATUS_JEITA_HOT_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_jeita_warm(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_JEITA_WARM_ADDR,
		&val,
		PMIC_RG_INT_STATUS_JEITA_WARM_MASK,
		PMIC_RG_INT_STATUS_JEITA_WARM_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_jeita_cool(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_JEITA_COOL_ADDR,
		&val,
		PMIC_RG_INT_STATUS_JEITA_COOL_MASK,
		PMIC_RG_INT_STATUS_JEITA_COOL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_jeita_cold(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_JEITA_COLD_ADDR,
		&val,
		PMIC_RG_INT_STATUS_JEITA_COLD_MASK,
		PMIC_RG_INT_STATUS_JEITA_COLD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vproc11_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VPROC11_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VPROC11_OC_MASK,
		PMIC_RG_INT_STATUS_VPROC11_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vproc12_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VPROC12_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VPROC12_OC_MASK,
		PMIC_RG_INT_STATUS_VPROC12_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vcore_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VCORE_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VCORE_OC_MASK,
		PMIC_RG_INT_STATUS_VCORE_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vgpu_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VGPU_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VGPU_OC_MASK,
		PMIC_RG_INT_STATUS_VGPU_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vdram1_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VDRAM1_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VDRAM1_OC_MASK,
		PMIC_RG_INT_STATUS_VDRAM1_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vdram2_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VDRAM2_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VDRAM2_OC_MASK,
		PMIC_RG_INT_STATUS_VDRAM2_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vmodem_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VMODEM_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VMODEM_OC_MASK,
		PMIC_RG_INT_STATUS_VMODEM_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vs1_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VS1_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VS1_OC_MASK,
		PMIC_RG_INT_STATUS_VS1_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vs2_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VS2_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VS2_OC_MASK,
		PMIC_RG_INT_STATUS_VS2_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vpa_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VPA_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VPA_OC_MASK,
		PMIC_RG_INT_STATUS_VPA_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vcore_preoc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VCORE_PREOC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VCORE_PREOC_MASK,
		PMIC_RG_INT_STATUS_VCORE_PREOC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_va10_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VA10_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VA10_OC_MASK,
		PMIC_RG_INT_STATUS_VA10_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_va12_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VA12_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VA12_OC_MASK,
		PMIC_RG_INT_STATUS_VA12_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_va18_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VA18_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VA18_OC_MASK,
		PMIC_RG_INT_STATUS_VA18_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vbif28_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VBIF28_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VBIF28_OC_MASK,
		PMIC_RG_INT_STATUS_VBIF28_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vcama1_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VCAMA1_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VCAMA1_OC_MASK,
		PMIC_RG_INT_STATUS_VCAMA1_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vcama2_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VCAMA2_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VCAMA2_OC_MASK,
		PMIC_RG_INT_STATUS_VCAMA2_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vxo18_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VXO18_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VXO18_OC_MASK,
		PMIC_RG_INT_STATUS_VXO18_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vcamd1_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VCAMD1_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VCAMD1_OC_MASK,
		PMIC_RG_INT_STATUS_VCAMD1_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vcamd2_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VCAMD2_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VCAMD2_OC_MASK,
		PMIC_RG_INT_STATUS_VCAMD2_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vcamio_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VCAMIO_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VCAMIO_OC_MASK,
		PMIC_RG_INT_STATUS_VCAMIO_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vcn18_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VCN18_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VCN18_OC_MASK,
		PMIC_RG_INT_STATUS_VCN18_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vcn28_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VCN28_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VCN28_OC_MASK,
		PMIC_RG_INT_STATUS_VCN28_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vcn33_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VCN33_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VCN33_OC_MASK,
		PMIC_RG_INT_STATUS_VCN33_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vtcxo24_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VTCXO24_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VTCXO24_OC_MASK,
		PMIC_RG_INT_STATUS_VTCXO24_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vemc_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VEMC_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VEMC_OC_MASK,
		PMIC_RG_INT_STATUS_VEMC_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vfe28_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VFE28_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VFE28_OC_MASK,
		PMIC_RG_INT_STATUS_VFE28_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vgp_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VGP_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VGP_OC_MASK,
		PMIC_RG_INT_STATUS_VGP_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vldo28_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VLDO28_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VLDO28_OC_MASK,
		PMIC_RG_INT_STATUS_VLDO28_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vio18_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VIO18_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VIO18_OC_MASK,
		PMIC_RG_INT_STATUS_VIO18_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vio28_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VIO28_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VIO28_OC_MASK,
		PMIC_RG_INT_STATUS_VIO28_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vmc_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VMC_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VMC_OC_MASK,
		PMIC_RG_INT_STATUS_VMC_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vmch_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VMCH_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VMCH_OC_MASK,
		PMIC_RG_INT_STATUS_VMCH_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vmipi_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VMIPI_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VMIPI_OC_MASK,
		PMIC_RG_INT_STATUS_VMIPI_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vrf12_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VRF12_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VRF12_OC_MASK,
		PMIC_RG_INT_STATUS_VRF12_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vrf18_1_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VRF18_1_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VRF18_1_OC_MASK,
		PMIC_RG_INT_STATUS_VRF18_1_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vrf18_2_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VRF18_2_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VRF18_2_OC_MASK,
		PMIC_RG_INT_STATUS_VRF18_2_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vsim1_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VSIM1_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VSIM1_OC_MASK,
		PMIC_RG_INT_STATUS_VSIM1_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vsim2_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VSIM2_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VSIM2_OC_MASK,
		PMIC_RG_INT_STATUS_VSIM2_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vgp2_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VGP2_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VGP2_OC_MASK,
		PMIC_RG_INT_STATUS_VGP2_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vsram_core_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VSRAM_CORE_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VSRAM_CORE_OC_MASK,
		PMIC_RG_INT_STATUS_VSRAM_CORE_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vsram_proc_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VSRAM_PROC_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VSRAM_PROC_OC_MASK,
		PMIC_RG_INT_STATUS_VSRAM_PROC_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vsram_gpu_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VSRAM_GPU_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VSRAM_GPU_OC_MASK,
		PMIC_RG_INT_STATUS_VSRAM_GPU_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vsram_md_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VSRAM_MD_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VSRAM_MD_OC_MASK,
		PMIC_RG_INT_STATUS_VSRAM_MD_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vufs18_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VUFS18_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VUFS18_OC_MASK,
		PMIC_RG_INT_STATUS_VUFS18_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vusb33_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VUSB33_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VUSB33_OC_MASK,
		PMIC_RG_INT_STATUS_VUSB33_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_vxo22_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_VXO22_OC_ADDR,
		&val,
		PMIC_RG_INT_STATUS_VXO22_OC_MASK,
		PMIC_RG_INT_STATUS_VXO22_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_fg_bat0_h(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_FG_BAT0_H_ADDR,
		&val,
		PMIC_RG_INT_STATUS_FG_BAT0_H_MASK,
		PMIC_RG_INT_STATUS_FG_BAT0_H_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_fg_bat0_l(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_FG_BAT0_L_ADDR,
		&val,
		PMIC_RG_INT_STATUS_FG_BAT0_L_MASK,
		PMIC_RG_INT_STATUS_FG_BAT0_L_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_fg_cur_h(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_FG_CUR_H_ADDR,
		&val,
		PMIC_RG_INT_STATUS_FG_CUR_H_MASK,
		PMIC_RG_INT_STATUS_FG_CUR_H_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_fg_cur_l(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_FG_CUR_L_ADDR,
		&val,
		PMIC_RG_INT_STATUS_FG_CUR_L_MASK,
		PMIC_RG_INT_STATUS_FG_CUR_L_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_fg_zcv(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_FG_ZCV_ADDR,
		&val,
		PMIC_RG_INT_STATUS_FG_ZCV_MASK,
		PMIC_RG_INT_STATUS_FG_ZCV_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_fg_bat1_h(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_FG_BAT1_H_ADDR,
		&val,
		PMIC_RG_INT_STATUS_FG_BAT1_H_MASK,
		PMIC_RG_INT_STATUS_FG_BAT1_H_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_fg_bat1_l(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_FG_BAT1_L_ADDR,
		&val,
		PMIC_RG_INT_STATUS_FG_BAT1_L_MASK,
		PMIC_RG_INT_STATUS_FG_BAT1_L_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_fg_n_charge_l(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_FG_N_CHARGE_L_ADDR,
		&val,
		PMIC_RG_INT_STATUS_FG_N_CHARGE_L_MASK,
		PMIC_RG_INT_STATUS_FG_N_CHARGE_L_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_fg_iavg_h(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_FG_IAVG_H_ADDR,
		&val,
		PMIC_RG_INT_STATUS_FG_IAVG_H_MASK,
		PMIC_RG_INT_STATUS_FG_IAVG_H_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_fg_iavg_l(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_FG_IAVG_L_ADDR,
		&val,
		PMIC_RG_INT_STATUS_FG_IAVG_L_MASK,
		PMIC_RG_INT_STATUS_FG_IAVG_L_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_fg_time_h(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_FG_TIME_H_ADDR,
		&val,
		PMIC_RG_INT_STATUS_FG_TIME_H_MASK,
		PMIC_RG_INT_STATUS_FG_TIME_H_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_fg_discharge(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_FG_DISCHARGE_ADDR,
		&val,
		PMIC_RG_INT_STATUS_FG_DISCHARGE_MASK,
		PMIC_RG_INT_STATUS_FG_DISCHARGE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_fg_charge(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_FG_CHARGE_ADDR,
		&val,
		PMIC_RG_INT_STATUS_FG_CHARGE_MASK,
		PMIC_RG_INT_STATUS_FG_CHARGE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_con5(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_CON5_ADDR,
		&val,
		PMIC_RG_INT_STATUS_CON5_MASK,
		PMIC_RG_INT_STATUS_CON5_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_audio(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_AUDIO_ADDR,
		&val,
		PMIC_RG_INT_STATUS_AUDIO_MASK,
		PMIC_RG_INT_STATUS_AUDIO_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_mad(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_MAD_ADDR,
		&val,
		PMIC_RG_INT_STATUS_MAD_MASK,
		PMIC_RG_INT_STATUS_MAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_eint_rtc32k_1v8_1(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_EINT_RTC32K_1V8_1_ADDR,
		&val,
		PMIC_RG_INT_STATUS_EINT_RTC32K_1V8_1_MASK,
		PMIC_RG_INT_STATUS_EINT_RTC32K_1V8_1_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_eint_aud_clk(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_EINT_AUD_CLK_ADDR,
		&val,
		PMIC_RG_INT_STATUS_EINT_AUD_CLK_MASK,
		PMIC_RG_INT_STATUS_EINT_AUD_CLK_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_eint_aud_dat_mosi(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_EINT_AUD_DAT_MOSI_ADDR,
		&val,
		PMIC_RG_INT_STATUS_EINT_AUD_DAT_MOSI_MASK,
		PMIC_RG_INT_STATUS_EINT_AUD_DAT_MOSI_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_eint_aud_dat_miso(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_EINT_AUD_DAT_MISO_ADDR,
		&val,
		PMIC_RG_INT_STATUS_EINT_AUD_DAT_MISO_MASK,
		PMIC_RG_INT_STATUS_EINT_AUD_DAT_MISO_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_eint_vow_clk_miso(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_EINT_VOW_CLK_MISO_ADDR,
		&val,
		PMIC_RG_INT_STATUS_EINT_VOW_CLK_MISO_MASK,
		PMIC_RG_INT_STATUS_EINT_VOW_CLK_MISO_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_accdet(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_ACCDET_ADDR,
		&val,
		PMIC_RG_INT_STATUS_ACCDET_MASK,
		PMIC_RG_INT_STATUS_ACCDET_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_accdet_eint(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_ACCDET_EINT_ADDR,
		&val,
		PMIC_RG_INT_STATUS_ACCDET_EINT_MASK,
		PMIC_RG_INT_STATUS_ACCDET_EINT_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_status_spi_cmd_alert(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_STATUS_SPI_CMD_ALERT_ADDR,
		&val,
		PMIC_RG_INT_STATUS_SPI_CMD_ALERT_MASK,
		PMIC_RG_INT_STATUS_SPI_CMD_ALERT_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_pwrkey(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_PWRKEY_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_PWRKEY_MASK,
		PMIC_RG_INT_RAW_STATUS_PWRKEY_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_homekey(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_HOMEKEY_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_HOMEKEY_MASK,
		PMIC_RG_INT_RAW_STATUS_HOMEKEY_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_pwrkey_r(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_PWRKEY_R_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_PWRKEY_R_MASK,
		PMIC_RG_INT_RAW_STATUS_PWRKEY_R_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_homekey_r(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_HOMEKEY_R_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_HOMEKEY_R_MASK,
		PMIC_RG_INT_RAW_STATUS_HOMEKEY_R_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_ni_lbat_int(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_NI_LBAT_INT_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_NI_LBAT_INT_MASK,
		PMIC_RG_INT_RAW_STATUS_NI_LBAT_INT_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_chrdet(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_CHRDET_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_CHRDET_MASK,
		PMIC_RG_INT_RAW_STATUS_CHRDET_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_chrdet_edge(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_CHRDET_EDGE_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_CHRDET_EDGE_MASK,
		PMIC_RG_INT_RAW_STATUS_CHRDET_EDGE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_baton_lv(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_BATON_LV_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_BATON_LV_MASK,
		PMIC_RG_INT_RAW_STATUS_BATON_LV_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_baton_hv(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_BATON_HV_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_BATON_HV_MASK,
		PMIC_RG_INT_RAW_STATUS_BATON_HV_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_baton_bat_in(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_BATON_BAT_IN_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_BATON_BAT_IN_MASK,
		PMIC_RG_INT_RAW_STATUS_BATON_BAT_IN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_baton_bat_out(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_BATON_BAT_OUT_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_BATON_BAT_OUT_MASK,
		PMIC_RG_INT_RAW_STATUS_BATON_BAT_OUT_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_rtc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_RTC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_RTC_MASK,
		PMIC_RG_INT_RAW_STATUS_RTC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_rtc_nsec(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_RTC_NSEC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_RTC_NSEC_MASK,
		PMIC_RG_INT_RAW_STATUS_RTC_NSEC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_bif(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_BIF_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_BIF_MASK,
		PMIC_RG_INT_RAW_STATUS_BIF_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vcdt_hv_det(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VCDT_HV_DET_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VCDT_HV_DET_MASK,
		PMIC_RG_INT_RAW_STATUS_VCDT_HV_DET_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_thr_h(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_THR_H_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_THR_H_MASK,
		PMIC_RG_INT_RAW_STATUS_THR_H_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_thr_l(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_THR_L_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_THR_L_MASK,
		PMIC_RG_INT_RAW_STATUS_THR_L_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_bat_h(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_BAT_H_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_BAT_H_MASK,
		PMIC_RG_INT_RAW_STATUS_BAT_H_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_bat_l(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_BAT_L_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_BAT_L_MASK,
		PMIC_RG_INT_RAW_STATUS_BAT_L_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_bat2_h(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_BAT2_H_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_BAT2_H_MASK,
		PMIC_RG_INT_RAW_STATUS_BAT2_H_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_bat2_l(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_BAT2_L_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_BAT2_L_MASK,
		PMIC_RG_INT_RAW_STATUS_BAT2_L_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_bat_temp_h(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_BAT_TEMP_H_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_BAT_TEMP_H_MASK,
		PMIC_RG_INT_RAW_STATUS_BAT_TEMP_H_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_bat_temp_l(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_BAT_TEMP_L_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_BAT_TEMP_L_MASK,
		PMIC_RG_INT_RAW_STATUS_BAT_TEMP_L_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_auxadc_imp(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_AUXADC_IMP_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_AUXADC_IMP_MASK,
		PMIC_RG_INT_RAW_STATUS_AUXADC_IMP_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_nag_c_dltv(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_NAG_C_DLTV_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_NAG_C_DLTV_MASK,
		PMIC_RG_INT_RAW_STATUS_NAG_C_DLTV_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_jeita_hot(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_JEITA_HOT_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_JEITA_HOT_MASK,
		PMIC_RG_INT_RAW_STATUS_JEITA_HOT_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_jeita_warm(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_JEITA_WARM_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_JEITA_WARM_MASK,
		PMIC_RG_INT_RAW_STATUS_JEITA_WARM_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_jeita_cool(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_JEITA_COOL_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_JEITA_COOL_MASK,
		PMIC_RG_INT_RAW_STATUS_JEITA_COOL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_jeita_cold(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_JEITA_COLD_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_JEITA_COLD_MASK,
		PMIC_RG_INT_RAW_STATUS_JEITA_COLD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vproc11_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VPROC11_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VPROC11_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VPROC11_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vproc12_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VPROC12_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VPROC12_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VPROC12_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vcore_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VCORE_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VCORE_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VCORE_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vgpu_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VGPU_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VGPU_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VGPU_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vdram1_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VDRAM1_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VDRAM1_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VDRAM1_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vdram2_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VDRAM2_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VDRAM2_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VDRAM2_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vmodem_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VMODEM_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VMODEM_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VMODEM_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vs1_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VS1_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VS1_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VS1_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vs2_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VS2_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VS2_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VS2_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vpa_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VPA_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VPA_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VPA_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vcore_preoc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VCORE_PREOC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VCORE_PREOC_MASK,
		PMIC_RG_INT_RAW_STATUS_VCORE_PREOC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_va10_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VA10_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VA10_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VA10_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_va12_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VA12_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VA12_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VA12_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_va18_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VA18_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VA18_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VA18_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vbif28_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VBIF28_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VBIF28_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VBIF28_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vcama1_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VCAMA1_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VCAMA1_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VCAMA1_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vcama2_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VCAMA2_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VCAMA2_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VCAMA2_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vxo18_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VXO18_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VXO18_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VXO18_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vcamd1_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VCAMD1_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VCAMD1_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VCAMD1_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vcamd2_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VCAMD2_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VCAMD2_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VCAMD2_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vcamio_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VCAMIO_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VCAMIO_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VCAMIO_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vcn18_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VCN18_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VCN18_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VCN18_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vcn28_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VCN28_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VCN28_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VCN28_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vcn33_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VCN33_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VCN33_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VCN33_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vtcxo24_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VTCXO24_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VTCXO24_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VTCXO24_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vemc_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VEMC_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VEMC_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VEMC_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vfe28_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VFE28_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VFE28_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VFE28_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vgp_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VGP_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VGP_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VGP_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vldo28_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VLDO28_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VLDO28_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VLDO28_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vio18_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VIO18_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VIO18_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VIO18_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vio28_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VIO28_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VIO28_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VIO28_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vmc_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VMC_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VMC_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VMC_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vmch_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VMCH_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VMCH_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VMCH_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vmipi_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VMIPI_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VMIPI_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VMIPI_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vrf12_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VRF12_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VRF12_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VRF12_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vrf18_1_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VRF18_1_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VRF18_1_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VRF18_1_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vrf18_2_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VRF18_2_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VRF18_2_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VRF18_2_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vsim1_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VSIM1_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VSIM1_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VSIM1_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vsim2_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VSIM2_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VSIM2_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VSIM2_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vgp2_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VGP2_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VGP2_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VGP2_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vsram_core_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VSRAM_CORE_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VSRAM_CORE_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VSRAM_CORE_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vsram_proc_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VSRAM_PROC_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VSRAM_PROC_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VSRAM_PROC_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vsram_gpu_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VSRAM_GPU_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VSRAM_GPU_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VSRAM_GPU_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vsram_md_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VSRAM_MD_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VSRAM_MD_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VSRAM_MD_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vufs18_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VUFS18_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VUFS18_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VUFS18_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vusb33_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VUSB33_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VUSB33_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VUSB33_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_vxo22_oc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_VXO22_OC_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_VXO22_OC_MASK,
		PMIC_RG_INT_RAW_STATUS_VXO22_OC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_fg_bat0_h(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_FG_BAT0_H_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_FG_BAT0_H_MASK,
		PMIC_RG_INT_RAW_STATUS_FG_BAT0_H_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_fg_bat0_l(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_FG_BAT0_L_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_FG_BAT0_L_MASK,
		PMIC_RG_INT_RAW_STATUS_FG_BAT0_L_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_fg_cur_h(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_FG_CUR_H_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_FG_CUR_H_MASK,
		PMIC_RG_INT_RAW_STATUS_FG_CUR_H_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_fg_cur_l(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_FG_CUR_L_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_FG_CUR_L_MASK,
		PMIC_RG_INT_RAW_STATUS_FG_CUR_L_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_fg_zcv(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_FG_ZCV_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_FG_ZCV_MASK,
		PMIC_RG_INT_RAW_STATUS_FG_ZCV_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_fg_bat1_h(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_FG_BAT1_H_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_FG_BAT1_H_MASK,
		PMIC_RG_INT_RAW_STATUS_FG_BAT1_H_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_fg_bat1_l(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_FG_BAT1_L_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_FG_BAT1_L_MASK,
		PMIC_RG_INT_RAW_STATUS_FG_BAT1_L_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_fg_n_charge_l(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_FG_N_CHARGE_L_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_FG_N_CHARGE_L_MASK,
		PMIC_RG_INT_RAW_STATUS_FG_N_CHARGE_L_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_fg_iavg_h(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_FG_IAVG_H_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_FG_IAVG_H_MASK,
		PMIC_RG_INT_RAW_STATUS_FG_IAVG_H_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_fg_iavg_l(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_FG_IAVG_L_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_FG_IAVG_L_MASK,
		PMIC_RG_INT_RAW_STATUS_FG_IAVG_L_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_fg_time_h(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_FG_TIME_H_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_FG_TIME_H_MASK,
		PMIC_RG_INT_RAW_STATUS_FG_TIME_H_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_fg_discharge(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_FG_DISCHARGE_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_FG_DISCHARGE_MASK,
		PMIC_RG_INT_RAW_STATUS_FG_DISCHARGE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_fg_charge(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_FG_CHARGE_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_FG_CHARGE_MASK,
		PMIC_RG_INT_RAW_STATUS_FG_CHARGE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_con5(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_CON5_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_CON5_MASK,
		PMIC_RG_INT_RAW_STATUS_CON5_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_audio(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_AUDIO_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_AUDIO_MASK,
		PMIC_RG_INT_RAW_STATUS_AUDIO_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_mad(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_MAD_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_MAD_MASK,
		PMIC_RG_INT_RAW_STATUS_MAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_eint_rtc32k_1v8_1(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_EINT_RTC32K_1V8_1_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_EINT_RTC32K_1V8_1_MASK,
		PMIC_RG_INT_RAW_STATUS_EINT_RTC32K_1V8_1_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_eint_aud_clk(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_EINT_AUD_CLK_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_EINT_AUD_CLK_MASK,
		PMIC_RG_INT_RAW_STATUS_EINT_AUD_CLK_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_eint_aud_dat_mosi(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_EINT_AUD_DAT_MOSI_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_EINT_AUD_DAT_MOSI_MASK,
		PMIC_RG_INT_RAW_STATUS_EINT_AUD_DAT_MOSI_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_eint_aud_dat_miso(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_EINT_AUD_DAT_MISO_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_EINT_AUD_DAT_MISO_MASK,
		PMIC_RG_INT_RAW_STATUS_EINT_AUD_DAT_MISO_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_eint_vow_clk_miso(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_EINT_VOW_CLK_MISO_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_EINT_VOW_CLK_MISO_MASK,
		PMIC_RG_INT_RAW_STATUS_EINT_VOW_CLK_MISO_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_accdet(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_ACCDET_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_ACCDET_MASK,
		PMIC_RG_INT_RAW_STATUS_ACCDET_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_accdet_eint(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_ACCDET_EINT_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_ACCDET_EINT_MASK,
		PMIC_RG_INT_RAW_STATUS_ACCDET_EINT_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_int_raw_status_spi_cmd_alert(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_INT_RAW_STATUS_SPI_CMD_ALERT_ADDR,
		&val,
		PMIC_RG_INT_RAW_STATUS_SPI_CMD_ALERT_MASK,
		PMIC_RG_INT_RAW_STATUS_SPI_CMD_ALERT_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_polarity(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_POLARITY_ADDR,
		val,
		PMIC_POLARITY_MASK,
		PMIC_POLARITY_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_homekey_int_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_HOMEKEY_INT_SEL_ADDR,
		val,
		PMIC_RG_HOMEKEY_INT_SEL_MASK,
		PMIC_RG_HOMEKEY_INT_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_pwrkey_int_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_PWRKEY_INT_SEL_ADDR,
		val,
		PMIC_RG_PWRKEY_INT_SEL_MASK,
		PMIC_RG_PWRKEY_INT_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_chrdet_int_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_CHRDET_INT_SEL_ADDR,
		val,
		PMIC_RG_CHRDET_INT_SEL_MASK,
		PMIC_RG_CHRDET_INT_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_pchr_cm_vinc_polarity_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_PCHR_CM_VINC_POLARITY_RSV_ADDR,
		val,
		PMIC_RG_PCHR_CM_VINC_POLARITY_RSV_MASK,
		PMIC_RG_PCHR_CM_VINC_POLARITY_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_pchr_cm_vdec_polarity_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_PCHR_CM_VDEC_POLARITY_RSV_ADDR,
		val,
		PMIC_RG_PCHR_CM_VDEC_POLARITY_RSV_MASK,
		PMIC_RG_PCHR_CM_VDEC_POLARITY_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_sen_eint_rtc32k_1v8_1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_SEN_EINT_RTC32K_1V8_1_ADDR,
		val,
		PMIC_RG_INT_SEN_EINT_RTC32K_1V8_1_MASK,
		PMIC_RG_INT_SEN_EINT_RTC32K_1V8_1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_sen_eint_aud_clk(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_SEN_EINT_AUD_CLK_ADDR,
		val,
		PMIC_RG_INT_SEN_EINT_AUD_CLK_MASK,
		PMIC_RG_INT_SEN_EINT_AUD_CLK_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_sen_eint_aud_dat_mosi(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_SEN_EINT_AUD_DAT_MOSI_ADDR,
		val,
		PMIC_RG_INT_SEN_EINT_AUD_DAT_MOSI_MASK,
		PMIC_RG_INT_SEN_EINT_AUD_DAT_MOSI_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_sen_eint_aud_dat_miso(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_SEN_EINT_AUD_DAT_MISO_ADDR,
		val,
		PMIC_RG_INT_SEN_EINT_AUD_DAT_MISO_MASK,
		PMIC_RG_INT_SEN_EINT_AUD_DAT_MISO_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_sen_eint_vow_clk_miso(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_SEN_EINT_VOW_CLK_MISO_ADDR,
		val,
		PMIC_RG_INT_SEN_EINT_VOW_CLK_MISO_MASK,
		PMIC_RG_INT_SEN_EINT_VOW_CLK_MISO_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_pol_eint_rtc32k_1v8_1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_POL_EINT_RTC32K_1V8_1_ADDR,
		val,
		PMIC_RG_INT_POL_EINT_RTC32K_1V8_1_MASK,
		PMIC_RG_INT_POL_EINT_RTC32K_1V8_1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_pol_eint_aud_clk(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_POL_EINT_AUD_CLK_ADDR,
		val,
		PMIC_RG_INT_POL_EINT_AUD_CLK_MASK,
		PMIC_RG_INT_POL_EINT_AUD_CLK_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_pol_eint_aud_dat_mosi(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_POL_EINT_AUD_DAT_MOSI_ADDR,
		val,
		PMIC_RG_INT_POL_EINT_AUD_DAT_MOSI_MASK,
		PMIC_RG_INT_POL_EINT_AUD_DAT_MOSI_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_pol_eint_aud_dat_miso(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_POL_EINT_AUD_DAT_MISO_ADDR,
		val,
		PMIC_RG_INT_POL_EINT_AUD_DAT_MISO_MASK,
		PMIC_RG_INT_POL_EINT_AUD_DAT_MISO_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_int_pol_eint_vow_clk_miso(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_INT_POL_EINT_VOW_CLK_MISO_ADDR,
		val,
		PMIC_RG_INT_POL_EINT_VOW_CLK_MISO_MASK,
		PMIC_RG_INT_POL_EINT_VOW_CLK_MISO_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_sw_sel_eint_rtc32k_1v8_1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SW_SEL_EINT_RTC32K_1V8_1_ADDR,
		val,
		PMIC_RG_SW_SEL_EINT_RTC32K_1V8_1_MASK,
		PMIC_RG_SW_SEL_EINT_RTC32K_1V8_1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_sw_sel_eint_aud_clk(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SW_SEL_EINT_AUD_CLK_ADDR,
		val,
		PMIC_RG_SW_SEL_EINT_AUD_CLK_MASK,
		PMIC_RG_SW_SEL_EINT_AUD_CLK_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_sw_sel_eint_aud_dat_mosi(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SW_SEL_EINT_AUD_DAT_MOSI_ADDR,
		val,
		PMIC_RG_SW_SEL_EINT_AUD_DAT_MOSI_MASK,
		PMIC_RG_SW_SEL_EINT_AUD_DAT_MOSI_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_sw_sel_eint_aud_dat_miso(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SW_SEL_EINT_AUD_DAT_MISO_ADDR,
		val,
		PMIC_RG_SW_SEL_EINT_AUD_DAT_MISO_MASK,
		PMIC_RG_SW_SEL_EINT_AUD_DAT_MISO_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_sw_sel_eint_vow_clk_miso(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SW_SEL_EINT_VOW_CLK_MISO_ADDR,
		val,
		PMIC_RG_SW_SEL_EINT_VOW_CLK_MISO_MASK,
		PMIC_RG_SW_SEL_EINT_VOW_CLK_MISO_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_sw_eint_rtc32k_1v8_1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SW_EINT_RTC32K_1V8_1_ADDR,
		val,
		PMIC_RG_SW_EINT_RTC32K_1V8_1_MASK,
		PMIC_RG_SW_EINT_RTC32K_1V8_1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_sw_eint_aud_clk(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SW_EINT_AUD_CLK_ADDR,
		val,
		PMIC_RG_SW_EINT_AUD_CLK_MASK,
		PMIC_RG_SW_EINT_AUD_CLK_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_sw_eint_aud_dat_mosi(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SW_EINT_AUD_DAT_MOSI_ADDR,
		val,
		PMIC_RG_SW_EINT_AUD_DAT_MOSI_MASK,
		PMIC_RG_SW_EINT_AUD_DAT_MOSI_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_sw_eint_aud_dat_miso(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SW_EINT_AUD_DAT_MISO_ADDR,
		val,
		PMIC_RG_SW_EINT_AUD_DAT_MISO_MASK,
		PMIC_RG_SW_EINT_AUD_DAT_MISO_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_sw_eint_vow_clk_miso(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SW_EINT_VOW_CLK_MISO_ADDR,
		val,
		PMIC_RG_SW_EINT_VOW_CLK_MISO_MASK,
		PMIC_RG_SW_EINT_VOW_CLK_MISO_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_deb_eint_rtc32k_1v8_1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_DEB_EINT_RTC32K_1V8_1_ADDR,
		val,
		PMIC_RG_DEB_EINT_RTC32K_1V8_1_MASK,
		PMIC_RG_DEB_EINT_RTC32K_1V8_1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_deb_eint_aud_clk(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_DEB_EINT_AUD_CLK_ADDR,
		val,
		PMIC_RG_DEB_EINT_AUD_CLK_MASK,
		PMIC_RG_DEB_EINT_AUD_CLK_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_deb_eint_aud_dat_mosi(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_DEB_EINT_AUD_DAT_MOSI_ADDR,
		val,
		PMIC_RG_DEB_EINT_AUD_DAT_MOSI_MASK,
		PMIC_RG_DEB_EINT_AUD_DAT_MOSI_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_deb_eint_aud_dat_miso(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_DEB_EINT_AUD_DAT_MISO_ADDR,
		val,
		PMIC_RG_DEB_EINT_AUD_DAT_MISO_MASK,
		PMIC_RG_DEB_EINT_AUD_DAT_MISO_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_deb_eint_vow_clk_miso(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_DEB_EINT_VOW_CLK_MISO_ADDR,
		val,
		PMIC_RG_DEB_EINT_VOW_CLK_MISO_MASK,
		PMIC_RG_DEB_EINT_VOW_CLK_MISO_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_fqmtr_tcksel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_FQMTR_TCKSEL_ADDR,
		val,
		PMIC_FQMTR_TCKSEL_MASK,
		PMIC_FQMTR_TCKSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_fqmtr_busy(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_FQMTR_BUSY_ADDR,
		&val,
		PMIC_FQMTR_BUSY_MASK,
		PMIC_FQMTR_BUSY_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_fqmtr_dcxo26m_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_FQMTR_DCXO26M_EN_ADDR,
		val,
		PMIC_FQMTR_DCXO26M_EN_MASK,
		PMIC_FQMTR_DCXO26M_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_fqmtr_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_FQMTR_EN_ADDR,
		val,
		PMIC_FQMTR_EN_MASK,
		PMIC_FQMTR_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_fqmtr_winset(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_FQMTR_WINSET_ADDR,
		val,
		PMIC_FQMTR_WINSET_MASK,
		PMIC_FQMTR_WINSET_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_fqmtr_data(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_FQMTR_DATA_ADDR,
		&val,
		PMIC_FQMTR_DATA_MASK,
		PMIC_FQMTR_DATA_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_slp_rw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SLP_RW_EN_ADDR,
		val,
		PMIC_RG_SLP_RW_EN_MASK,
		PMIC_RG_SLP_RW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_slp_rw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_SLP_RW_EN_ADDR,
		&val,
		PMIC_RG_SLP_RW_EN_MASK,
		PMIC_RG_SLP_RW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_spi_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SPI_RSV_ADDR,
		val,
		PMIC_RG_SPI_RSV_MASK,
		PMIC_RG_SPI_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_dew_dio_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_DEW_DIO_EN_ADDR,
		val,
		PMIC_DEW_DIO_EN_MASK,
		PMIC_DEW_DIO_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_dew_read_test(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DEW_READ_TEST_ADDR,
		&val,
		PMIC_DEW_READ_TEST_MASK,
		PMIC_DEW_READ_TEST_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_dew_write_test(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_DEW_WRITE_TEST_ADDR,
		val,
		PMIC_DEW_WRITE_TEST_MASK,
		PMIC_DEW_WRITE_TEST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_dew_crc_swrst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_DEW_CRC_SWRST_ADDR,
		val,
		PMIC_DEW_CRC_SWRST_MASK,
		PMIC_DEW_CRC_SWRST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_dew_crc_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_DEW_CRC_EN_ADDR,
		val,
		PMIC_DEW_CRC_EN_MASK,
		PMIC_DEW_CRC_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_dew_crc_val(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DEW_CRC_VAL_ADDR,
		&val,
		PMIC_DEW_CRC_VAL_MASK,
		PMIC_DEW_CRC_VAL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_dew_dbg_mon_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_DEW_DBG_MON_SEL_ADDR,
		val,
		PMIC_DEW_DBG_MON_SEL_MASK,
		PMIC_DEW_DBG_MON_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_dew_cipher_key_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_DEW_CIPHER_KEY_SEL_ADDR,
		val,
		PMIC_DEW_CIPHER_KEY_SEL_MASK,
		PMIC_DEW_CIPHER_KEY_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_dew_cipher_iv_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_DEW_CIPHER_IV_SEL_ADDR,
		val,
		PMIC_DEW_CIPHER_IV_SEL_MASK,
		PMIC_DEW_CIPHER_IV_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_dew_cipher_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_DEW_CIPHER_EN_ADDR,
		val,
		PMIC_DEW_CIPHER_EN_MASK,
		PMIC_DEW_CIPHER_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_dew_cipher_rdy(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DEW_CIPHER_RDY_ADDR,
		&val,
		PMIC_DEW_CIPHER_RDY_MASK,
		PMIC_DEW_CIPHER_RDY_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_dew_cipher_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_DEW_CIPHER_MODE_ADDR,
		val,
		PMIC_DEW_CIPHER_MODE_MASK,
		PMIC_DEW_CIPHER_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_dew_cipher_swrst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_DEW_CIPHER_SWRST_ADDR,
		val,
		PMIC_DEW_CIPHER_SWRST_MASK,
		PMIC_DEW_CIPHER_SWRST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_dew_rddmy_no(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_DEW_RDDMY_NO_ADDR,
		val,
		PMIC_DEW_RDDMY_NO_MASK,
		PMIC_DEW_RDDMY_NO_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_int_type_con0(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_INT_TYPE_CON0_ADDR,
		val,
		PMIC_INT_TYPE_CON0_MASK,
		PMIC_INT_TYPE_CON0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_int_type_con1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_INT_TYPE_CON1_ADDR,
		val,
		PMIC_INT_TYPE_CON1_MASK,
		PMIC_INT_TYPE_CON1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_int_type_con2(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_INT_TYPE_CON2_ADDR,
		val,
		PMIC_INT_TYPE_CON2_MASK,
		PMIC_INT_TYPE_CON2_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_int_type_con3(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_INT_TYPE_CON3_ADDR,
		val,
		PMIC_INT_TYPE_CON3_MASK,
		PMIC_INT_TYPE_CON3_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_int_type_con4(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_INT_TYPE_CON4_ADDR,
		val,
		PMIC_INT_TYPE_CON4_MASK,
		PMIC_INT_TYPE_CON4_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_int_type_con5(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_INT_TYPE_CON5_ADDR,
		val,
		PMIC_INT_TYPE_CON5_MASK,
		PMIC_INT_TYPE_CON5_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_int_type_con6(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_INT_TYPE_CON6_ADDR,
		val,
		PMIC_INT_TYPE_CON6_MASK,
		PMIC_INT_TYPE_CON6_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_cpu_int_sta(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_CPU_INT_STA_ADDR,
		&val,
		PMIC_CPU_INT_STA_MASK,
		PMIC_CPU_INT_STA_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_md32_int_sta(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_MD32_INT_STA_ADDR,
		&val,
		PMIC_MD32_INT_STA_MASK,
		PMIC_MD32_INT_STA_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_srclken_in3_smps_clk_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SRCLKEN_IN3_SMPS_CLK_MODE_ADDR,
		val,
		PMIC_RG_SRCLKEN_IN3_SMPS_CLK_MODE_MASK,
		PMIC_RG_SRCLKEN_IN3_SMPS_CLK_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_srclken_in3_en_smps_test(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SRCLKEN_IN3_EN_SMPS_TEST_ADDR,
		val,
		PMIC_RG_SRCLKEN_IN3_EN_SMPS_TEST_MASK,
		PMIC_RG_SRCLKEN_IN3_EN_SMPS_TEST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_srclken_in3_en_smps_test(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_SRCLKEN_IN3_EN_SMPS_TEST_ADDR,
		&val,
		PMIC_RG_SRCLKEN_IN3_EN_SMPS_TEST_MASK,
		PMIC_RG_SRCLKEN_IN3_EN_SMPS_TEST_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_srclken_in2_smps_clk_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SRCLKEN_IN2_SMPS_CLK_MODE_ADDR,
		val,
		PMIC_RG_SRCLKEN_IN2_SMPS_CLK_MODE_MASK,
		PMIC_RG_SRCLKEN_IN2_SMPS_CLK_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_srclken_in2_en_smps_test(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SRCLKEN_IN2_EN_SMPS_TEST_ADDR,
		val,
		PMIC_RG_SRCLKEN_IN2_EN_SMPS_TEST_MASK,
		PMIC_RG_SRCLKEN_IN2_EN_SMPS_TEST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_srclken_in2_en_smps_test(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_SRCLKEN_IN2_EN_SMPS_TEST_ADDR,
		&val,
		PMIC_RG_SRCLKEN_IN2_EN_SMPS_TEST_MASK,
		PMIC_RG_SRCLKEN_IN2_EN_SMPS_TEST_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_spi_dly_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SPI_DLY_SEL_ADDR,
		val,
		PMIC_RG_SPI_DLY_SEL_MASK,
		PMIC_RG_SPI_DLY_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_record_cmd0(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RECORD_CMD0_ADDR,
		&val,
		PMIC_RECORD_CMD0_MASK,
		PMIC_RECORD_CMD0_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_record_cmd1(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RECORD_CMD1_ADDR,
		&val,
		PMIC_RECORD_CMD1_MASK,
		PMIC_RECORD_CMD1_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_record_cmd2(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RECORD_CMD2_ADDR,
		&val,
		PMIC_RECORD_CMD2_MASK,
		PMIC_RECORD_CMD2_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_record_wdata0(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RECORD_WDATA0_ADDR,
		&val,
		PMIC_RECORD_WDATA0_MASK,
		PMIC_RECORD_WDATA0_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_record_wdata1(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RECORD_WDATA1_ADDR,
		&val,
		PMIC_RECORD_WDATA1_MASK,
		PMIC_RECORD_WDATA1_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_record_wdata2(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RECORD_WDATA2_ADDR,
		&val,
		PMIC_RECORD_WDATA2_MASK,
		PMIC_RECORD_WDATA2_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_addr_target(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_ADDR_TARGET_ADDR,
		val,
		PMIC_RG_ADDR_TARGET_MASK,
		PMIC_RG_ADDR_TARGET_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_addr_mask(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_ADDR_MASK_ADDR,
		val,
		PMIC_RG_ADDR_MASK_MASK,
		PMIC_RG_ADDR_MASK_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_wdata_target(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_WDATA_TARGET_ADDR,
		val,
		PMIC_RG_WDATA_TARGET_MASK,
		PMIC_RG_WDATA_TARGET_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_wdata_mask(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_WDATA_MASK_ADDR,
		val,
		PMIC_RG_WDATA_MASK_MASK,
		PMIC_RG_WDATA_MASK_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_spi_record_clr(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SPI_RECORD_CLR_ADDR,
		val,
		PMIC_RG_SPI_RECORD_CLR_MASK,
		PMIC_RG_SPI_RECORD_CLR_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_cmd_alert_clr(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_CMD_ALERT_CLR_ADDR,
		val,
		PMIC_RG_CMD_ALERT_CLR_MASK,
		PMIC_RG_CMD_ALERT_CLR_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_thr_det_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_THR_DET_DIS_ADDR,
		val,
		PMIC_RG_THR_DET_DIS_MASK,
		PMIC_RG_THR_DET_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_thr_test(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_THR_TEST_ADDR,
		val,
		PMIC_RG_THR_TEST_MASK,
		PMIC_RG_THR_TEST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_strup_ther_deb_rmax(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_THER_DEB_RMAX_ADDR,
		val,
		PMIC_RG_STRUP_THER_DEB_RMAX_MASK,
		PMIC_RG_STRUP_THER_DEB_RMAX_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_strup_ther_deb_fmax(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_THER_DEB_FMAX_ADDR,
		val,
		PMIC_RG_STRUP_THER_DEB_FMAX_MASK,
		PMIC_RG_STRUP_THER_DEB_FMAX_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_dduvlo_deb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_DDUVLO_DEB_EN_ADDR,
		val,
		PMIC_DDUVLO_DEB_EN_MASK,
		PMIC_DDUVLO_DEB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_strup_osc_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_OSC_EN_ADDR,
		val,
		PMIC_RG_STRUP_OSC_EN_MASK,
		PMIC_RG_STRUP_OSC_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_osc_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_OSC_EN_ADDR,
		&val,
		PMIC_RG_STRUP_OSC_EN_MASK,
		PMIC_RG_STRUP_OSC_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_osc_en_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_OSC_EN_SEL_ADDR,
		val,
		PMIC_RG_STRUP_OSC_EN_SEL_MASK,
		PMIC_RG_STRUP_OSC_EN_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_osc_en_sel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_OSC_EN_SEL_ADDR,
		&val,
		PMIC_RG_STRUP_OSC_EN_SEL_MASK,
		PMIC_RG_STRUP_OSC_EN_SEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_ft_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_FT_CTRL_ADDR,
		val,
		PMIC_RG_STRUP_FT_CTRL_MASK,
		PMIC_RG_STRUP_FT_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_strup_pwron_force(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_PWRON_FORCE_ADDR,
		val,
		PMIC_RG_STRUP_PWRON_FORCE_MASK,
		PMIC_RG_STRUP_PWRON_FORCE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_biasgen_force(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BIASGEN_FORCE_ADDR,
		val,
		PMIC_RG_BIASGEN_FORCE_MASK,
		PMIC_RG_BIASGEN_FORCE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_strup_pwron(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_PWRON_ADDR,
		val,
		PMIC_RG_STRUP_PWRON_MASK,
		PMIC_RG_STRUP_PWRON_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_strup_pwron_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_PWRON_SEL_ADDR,
		val,
		PMIC_RG_STRUP_PWRON_SEL_MASK,
		PMIC_RG_STRUP_PWRON_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_biasgen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BIASGEN_ADDR,
		val,
		PMIC_RG_BIASGEN_MASK,
		PMIC_RG_BIASGEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_biasgen_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BIASGEN_SEL_ADDR,
		val,
		PMIC_RG_BIASGEN_SEL_MASK,
		PMIC_RG_BIASGEN_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_rtc_xosc32_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RTC_XOSC32_ENB_ADDR,
		val,
		PMIC_RG_RTC_XOSC32_ENB_MASK,
		PMIC_RG_RTC_XOSC32_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_rtc_xosc32_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_RTC_XOSC32_ENB_ADDR,
		&val,
		PMIC_RG_RTC_XOSC32_ENB_MASK,
		PMIC_RG_RTC_XOSC32_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_rtc_xosc32_enb_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RTC_XOSC32_ENB_SEL_ADDR,
		val,
		PMIC_RG_RTC_XOSC32_ENB_SEL_MASK,
		PMIC_RG_RTC_XOSC32_ENB_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_rtc_xosc32_enb_sel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_RTC_XOSC32_ENB_SEL_ADDR,
		&val,
		PMIC_RG_RTC_XOSC32_ENB_SEL_MASK,
		PMIC_RG_RTC_XOSC32_ENB_SEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_strup_dig_io_pg_force(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_STRUP_DIG_IO_PG_FORCE_ADDR,
		val,
		PMIC_STRUP_DIG_IO_PG_FORCE_MASK,
		PMIC_STRUP_DIG_IO_PG_FORCE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_clr_just_smart_rst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_CLR_JUST_SMART_RST_ADDR,
		val,
		PMIC_RG_CLR_JUST_SMART_RST_MASK,
		PMIC_RG_CLR_JUST_SMART_RST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_clr_just_rst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_CLR_JUST_RST_ADDR,
		val,
		PMIC_CLR_JUST_RST_MASK,
		PMIC_CLR_JUST_RST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_uvlo_l2h_deb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_UVLO_L2H_DEB_EN_ADDR,
		val,
		PMIC_UVLO_L2H_DEB_EN_MASK,
		PMIC_UVLO_L2H_DEB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_just_smart_rst(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_JUST_SMART_RST_ADDR,
		&val,
		PMIC_JUST_SMART_RST_MASK,
		PMIC_JUST_SMART_RST_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_just_pwrkey_rst(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_JUST_PWRKEY_RST_ADDR,
		&val,
		PMIC_JUST_PWRKEY_RST_MASK,
		PMIC_JUST_PWRKEY_RST_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_osc_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_OSC_EN_ADDR,
		&val,
		PMIC_DA_QI_OSC_EN_MASK,
		PMIC_DA_QI_OSC_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_ext_pmic_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_EXT_PMIC_EN_ADDR,
		val,
		PMIC_RG_STRUP_EXT_PMIC_EN_MASK,
		PMIC_RG_STRUP_EXT_PMIC_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_ext_pmic_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_EXT_PMIC_EN_ADDR,
		&val,
		PMIC_RG_STRUP_EXT_PMIC_EN_MASK,
		PMIC_RG_STRUP_EXT_PMIC_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_ext_pmic_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_EXT_PMIC_SEL_ADDR,
		val,
		PMIC_RG_STRUP_EXT_PMIC_SEL_MASK,
		PMIC_RG_STRUP_EXT_PMIC_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_strup_con8_rsv0(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_STRUP_CON8_RSV0_ADDR,
		val,
		PMIC_STRUP_CON8_RSV0_MASK,
		PMIC_STRUP_CON8_RSV0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_ext_pmic_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_EXT_PMIC_EN_ADDR,
		&val,
		PMIC_DA_QI_EXT_PMIC_EN_MASK,
		PMIC_DA_QI_EXT_PMIC_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_auxadc_start_sw(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_AUXADC_START_SW_ADDR,
		val,
		PMIC_RG_STRUP_AUXADC_START_SW_MASK,
		PMIC_RG_STRUP_AUXADC_START_SW_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_strup_auxadc_rstb_sw(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_AUXADC_RSTB_SW_ADDR,
		val,
		PMIC_RG_STRUP_AUXADC_RSTB_SW_MASK,
		PMIC_RG_STRUP_AUXADC_RSTB_SW_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_strup_auxadc_start_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_AUXADC_START_SEL_ADDR,
		val,
		PMIC_RG_STRUP_AUXADC_START_SEL_MASK,
		PMIC_RG_STRUP_AUXADC_START_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_strup_auxadc_rstb_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_AUXADC_RSTB_SEL_ADDR,
		val,
		PMIC_RG_STRUP_AUXADC_RSTB_SEL_MASK,
		PMIC_RG_STRUP_AUXADC_RSTB_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_strup_auxadc_rpcnt_max(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_AUXADC_RPCNT_MAX_ADDR,
		val,
		PMIC_RG_STRUP_AUXADC_RPCNT_MAX_MASK,
		PMIC_RG_STRUP_AUXADC_RPCNT_MAX_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_strup_pwroff_seq_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_STRUP_PWROFF_SEQ_EN_ADDR,
		val,
		PMIC_STRUP_PWROFF_SEQ_EN_MASK,
		PMIC_STRUP_PWROFF_SEQ_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_strup_pwroff_preoff_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_STRUP_PWROFF_PREOFF_EN_ADDR,
		val,
		PMIC_STRUP_PWROFF_PREOFF_EN_MASK,
		PMIC_STRUP_PWROFF_PREOFF_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_strup_dig0_rsv0(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_STRUP_DIG0_RSV0_ADDR,
		val,
		PMIC_STRUP_DIG0_RSV0_MASK,
		PMIC_STRUP_DIG0_RSV0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_strup_dig1_rsv0(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_STRUP_DIG1_RSV0_ADDR,
		val,
		PMIC_STRUP_DIG1_RSV0_MASK,
		PMIC_STRUP_DIG1_RSV0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_rsv_swreg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RSV_SWREG_ADDR,
		val,
		PMIC_RG_RSV_SWREG_MASK,
		PMIC_RG_RSV_SWREG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_strup_uvlo_u1u2_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_UVLO_U1U2_SEL_ADDR,
		val,
		PMIC_RG_STRUP_UVLO_U1U2_SEL_MASK,
		PMIC_RG_STRUP_UVLO_U1U2_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_strup_uvlo_u1u2_sel_swctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_UVLO_U1U2_SEL_SWCTRL_ADDR,
		val,
		PMIC_RG_STRUP_UVLO_U1U2_SEL_SWCTRL_MASK,
		PMIC_RG_STRUP_UVLO_U1U2_SEL_SWCTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_strup_thr_clr(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_THR_CLR_ADDR,
		val,
		PMIC_RG_STRUP_THR_CLR_MASK,
		PMIC_RG_STRUP_THR_CLR_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_strup_long_press_ext_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_LONG_PRESS_EXT_SEL_ADDR,
		val,
		PMIC_RG_STRUP_LONG_PRESS_EXT_SEL_MASK,
		PMIC_RG_STRUP_LONG_PRESS_EXT_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_strup_long_press_ext_td(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_LONG_PRESS_EXT_TD_ADDR,
		val,
		PMIC_RG_STRUP_LONG_PRESS_EXT_TD_MASK,
		PMIC_RG_STRUP_LONG_PRESS_EXT_TD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_strup_long_press_ext_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_LONG_PRESS_EXT_EN_ADDR,
		val,
		PMIC_RG_STRUP_LONG_PRESS_EXT_EN_MASK,
		PMIC_RG_STRUP_LONG_PRESS_EXT_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_long_press_ext_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_LONG_PRESS_EXT_EN_ADDR,
		&val,
		PMIC_RG_STRUP_LONG_PRESS_EXT_EN_MASK,
		PMIC_RG_STRUP_LONG_PRESS_EXT_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_long_press_ext_chr_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_LONG_PRESS_EXT_CHR_CTRL_ADDR,
		val,
		PMIC_RG_STRUP_LONG_PRESS_EXT_CHR_CTRL_MASK,
		PMIC_RG_STRUP_LONG_PRESS_EXT_CHR_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_strup_long_press_ext_pwrkey_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_LONG_PRESS_EXT_PWRKEY_CTRL_ADDR,
		val,
		PMIC_RG_STRUP_LONG_PRESS_EXT_PWRKEY_CTRL_MASK,
		PMIC_RG_STRUP_LONG_PRESS_EXT_PWRKEY_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_strup_long_press_ext_spar_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_LONG_PRESS_EXT_SPAR_CTRL_ADDR,
		val,
		PMIC_RG_STRUP_LONG_PRESS_EXT_SPAR_CTRL_MASK,
		PMIC_RG_STRUP_LONG_PRESS_EXT_SPAR_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_strup_long_press_ext_rtca_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_LONG_PRESS_EXT_RTCA_CTRL_ADDR,
		val,
		PMIC_RG_STRUP_LONG_PRESS_EXT_RTCA_CTRL_MASK,
		PMIC_RG_STRUP_LONG_PRESS_EXT_RTCA_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_smart_rst_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SMART_RST_MODE_ADDR,
		val,
		PMIC_RG_SMART_RST_MODE_MASK,
		PMIC_RG_SMART_RST_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_strup_envtem(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_ENVTEM_ADDR,
		val,
		PMIC_RG_STRUP_ENVTEM_MASK,
		PMIC_RG_STRUP_ENVTEM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_envtem(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_ENVTEM_ADDR,
		&val,
		PMIC_RG_STRUP_ENVTEM_MASK,
		PMIC_RG_STRUP_ENVTEM_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_envtem_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_ENVTEM_CTRL_ADDR,
		val,
		PMIC_RG_STRUP_ENVTEM_CTRL_MASK,
		PMIC_RG_STRUP_ENVTEM_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_envtem_ctrl(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_ENVTEM_CTRL_ADDR,
		&val,
		PMIC_RG_STRUP_ENVTEM_CTRL_MASK,
		PMIC_RG_STRUP_ENVTEM_CTRL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_pwrkey_count_reset(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_PWRKEY_COUNT_RESET_ADDR,
		val,
		PMIC_RG_STRUP_PWRKEY_COUNT_RESET_MASK,
		PMIC_RG_STRUP_PWRKEY_COUNT_RESET_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_strup_va12_pg_h2l_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VA12_PG_H2L_EN_ADDR,
		val,
		PMIC_RG_STRUP_VA12_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VA12_PG_H2L_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_va12_pg_h2l_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VA12_PG_H2L_EN_ADDR,
		&val,
		PMIC_RG_STRUP_VA12_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VA12_PG_H2L_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_va10_pg_h2l_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VA10_PG_H2L_EN_ADDR,
		val,
		PMIC_RG_STRUP_VA10_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VA10_PG_H2L_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_va10_pg_h2l_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VA10_PG_H2L_EN_ADDR,
		&val,
		PMIC_RG_STRUP_VA10_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VA10_PG_H2L_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vsram_gpu_pg_h2l_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VSRAM_GPU_PG_H2L_EN_ADDR,
		val,
		PMIC_RG_STRUP_VSRAM_GPU_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VSRAM_GPU_PG_H2L_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vsram_gpu_pg_h2l_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VSRAM_GPU_PG_H2L_EN_ADDR,
		&val,
		PMIC_RG_STRUP_VSRAM_GPU_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VSRAM_GPU_PG_H2L_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vsram_md_pg_h2l_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VSRAM_MD_PG_H2L_EN_ADDR,
		val,
		PMIC_RG_STRUP_VSRAM_MD_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VSRAM_MD_PG_H2L_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vsram_md_pg_h2l_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VSRAM_MD_PG_H2L_EN_ADDR,
		&val,
		PMIC_RG_STRUP_VSRAM_MD_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VSRAM_MD_PG_H2L_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vsram_core_pg_h2l_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VSRAM_CORE_PG_H2L_EN_ADDR,
		val,
		PMIC_RG_STRUP_VSRAM_CORE_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VSRAM_CORE_PG_H2L_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vsram_core_pg_h2l_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VSRAM_CORE_PG_H2L_EN_ADDR,
		&val,
		PMIC_RG_STRUP_VSRAM_CORE_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VSRAM_CORE_PG_H2L_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_va18_pg_h2l_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VA18_PG_H2L_EN_ADDR,
		val,
		PMIC_RG_STRUP_VA18_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VA18_PG_H2L_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_va18_pg_h2l_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VA18_PG_H2L_EN_ADDR,
		&val,
		PMIC_RG_STRUP_VA18_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VA18_PG_H2L_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_buck_rsv_pg_h2l_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_BUCK_RSV_PG_H2L_EN_ADDR,
		val,
		PMIC_RG_STRUP_BUCK_RSV_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_BUCK_RSV_PG_H2L_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_buck_rsv_pg_h2l_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_BUCK_RSV_PG_H2L_EN_ADDR,
		&val,
		PMIC_RG_STRUP_BUCK_RSV_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_BUCK_RSV_PG_H2L_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vdram2_pg_h2l_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VDRAM2_PG_H2L_EN_ADDR,
		val,
		PMIC_RG_STRUP_VDRAM2_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VDRAM2_PG_H2L_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vdram2_pg_h2l_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VDRAM2_PG_H2L_EN_ADDR,
		&val,
		PMIC_RG_STRUP_VDRAM2_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VDRAM2_PG_H2L_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vdram1_pg_h2l_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VDRAM1_PG_H2L_EN_ADDR,
		val,
		PMIC_RG_STRUP_VDRAM1_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VDRAM1_PG_H2L_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vdram1_pg_h2l_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VDRAM1_PG_H2L_EN_ADDR,
		&val,
		PMIC_RG_STRUP_VDRAM1_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VDRAM1_PG_H2L_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vproc12_pg_h2l_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VPROC12_PG_H2L_EN_ADDR,
		val,
		PMIC_RG_STRUP_VPROC12_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VPROC12_PG_H2L_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vproc12_pg_h2l_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VPROC12_PG_H2L_EN_ADDR,
		&val,
		PMIC_RG_STRUP_VPROC12_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VPROC12_PG_H2L_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vproc11_pg_h2l_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VPROC11_PG_H2L_EN_ADDR,
		val,
		PMIC_RG_STRUP_VPROC11_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VPROC11_PG_H2L_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vproc11_pg_h2l_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VPROC11_PG_H2L_EN_ADDR,
		&val,
		PMIC_RG_STRUP_VPROC11_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VPROC11_PG_H2L_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vs1_pg_h2l_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VS1_PG_H2L_EN_ADDR,
		val,
		PMIC_RG_STRUP_VS1_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VS1_PG_H2L_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vs1_pg_h2l_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VS1_PG_H2L_EN_ADDR,
		&val,
		PMIC_RG_STRUP_VS1_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VS1_PG_H2L_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vmodem_pg_h2l_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VMODEM_PG_H2L_EN_ADDR,
		val,
		PMIC_RG_STRUP_VMODEM_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VMODEM_PG_H2L_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vmodem_pg_h2l_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VMODEM_PG_H2L_EN_ADDR,
		&val,
		PMIC_RG_STRUP_VMODEM_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VMODEM_PG_H2L_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vgpu_pg_h2l_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VGPU_PG_H2L_EN_ADDR,
		val,
		PMIC_RG_STRUP_VGPU_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VGPU_PG_H2L_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vgpu_pg_h2l_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VGPU_PG_H2L_EN_ADDR,
		&val,
		PMIC_RG_STRUP_VGPU_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VGPU_PG_H2L_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vcore_pg_h2l_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VCORE_PG_H2L_EN_ADDR,
		val,
		PMIC_RG_STRUP_VCORE_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VCORE_PG_H2L_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vcore_pg_h2l_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VCORE_PG_H2L_EN_ADDR,
		&val,
		PMIC_RG_STRUP_VCORE_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VCORE_PG_H2L_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vs2_pg_h2l_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VS2_PG_H2L_EN_ADDR,
		val,
		PMIC_RG_STRUP_VS2_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VS2_PG_H2L_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vs2_pg_h2l_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VS2_PG_H2L_EN_ADDR,
		&val,
		PMIC_RG_STRUP_VS2_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VS2_PG_H2L_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_ext_pmic_pg_h2l_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_EXT_PMIC_PG_H2L_EN_ADDR,
		val,
		PMIC_RG_STRUP_EXT_PMIC_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_EXT_PMIC_PG_H2L_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_ext_pmic_pg_h2l_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_EXT_PMIC_PG_H2L_EN_ADDR,
		&val,
		PMIC_RG_STRUP_EXT_PMIC_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_EXT_PMIC_PG_H2L_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vusb33_pg_h2l_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VUSB33_PG_H2L_EN_ADDR,
		val,
		PMIC_RG_STRUP_VUSB33_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VUSB33_PG_H2L_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vusb33_pg_h2l_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VUSB33_PG_H2L_EN_ADDR,
		&val,
		PMIC_RG_STRUP_VUSB33_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VUSB33_PG_H2L_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vsram_proc_pg_h2l_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VSRAM_PROC_PG_H2L_EN_ADDR,
		val,
		PMIC_RG_STRUP_VSRAM_PROC_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VSRAM_PROC_PG_H2L_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vsram_proc_pg_h2l_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VSRAM_PROC_PG_H2L_EN_ADDR,
		&val,
		PMIC_RG_STRUP_VSRAM_PROC_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VSRAM_PROC_PG_H2L_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vufs18_pg_h2l_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VUFS18_PG_H2L_EN_ADDR,
		val,
		PMIC_RG_STRUP_VUFS18_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VUFS18_PG_H2L_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vufs18_pg_h2l_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VUFS18_PG_H2L_EN_ADDR,
		&val,
		PMIC_RG_STRUP_VUFS18_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VUFS18_PG_H2L_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vemc_pg_h2l_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VEMC_PG_H2L_EN_ADDR,
		val,
		PMIC_RG_STRUP_VEMC_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VEMC_PG_H2L_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vemc_pg_h2l_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VEMC_PG_H2L_EN_ADDR,
		&val,
		PMIC_RG_STRUP_VEMC_PG_H2L_EN_MASK,
		PMIC_RG_STRUP_VEMC_PG_H2L_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_va12_pg_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VA12_PG_ENB_ADDR,
		val,
		PMIC_RG_STRUP_VA12_PG_ENB_MASK,
		PMIC_RG_STRUP_VA12_PG_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_va12_pg_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VA12_PG_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_VA12_PG_ENB_MASK,
		PMIC_RG_STRUP_VA12_PG_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_va10_pg_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VA10_PG_ENB_ADDR,
		val,
		PMIC_RG_STRUP_VA10_PG_ENB_MASK,
		PMIC_RG_STRUP_VA10_PG_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_va10_pg_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VA10_PG_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_VA10_PG_ENB_MASK,
		PMIC_RG_STRUP_VA10_PG_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vsram_gpu_pg_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VSRAM_GPU_PG_ENB_ADDR,
		val,
		PMIC_RG_STRUP_VSRAM_GPU_PG_ENB_MASK,
		PMIC_RG_STRUP_VSRAM_GPU_PG_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vsram_gpu_pg_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VSRAM_GPU_PG_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_VSRAM_GPU_PG_ENB_MASK,
		PMIC_RG_STRUP_VSRAM_GPU_PG_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vsram_md_pg_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VSRAM_MD_PG_ENB_ADDR,
		val,
		PMIC_RG_STRUP_VSRAM_MD_PG_ENB_MASK,
		PMIC_RG_STRUP_VSRAM_MD_PG_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vsram_md_pg_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VSRAM_MD_PG_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_VSRAM_MD_PG_ENB_MASK,
		PMIC_RG_STRUP_VSRAM_MD_PG_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vsram_core_pg_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VSRAM_CORE_PG_ENB_ADDR,
		val,
		PMIC_RG_STRUP_VSRAM_CORE_PG_ENB_MASK,
		PMIC_RG_STRUP_VSRAM_CORE_PG_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vsram_core_pg_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VSRAM_CORE_PG_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_VSRAM_CORE_PG_ENB_MASK,
		PMIC_RG_STRUP_VSRAM_CORE_PG_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_va18_pg_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VA18_PG_ENB_ADDR,
		val,
		PMIC_RG_STRUP_VA18_PG_ENB_MASK,
		PMIC_RG_STRUP_VA18_PG_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_va18_pg_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VA18_PG_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_VA18_PG_ENB_MASK,
		PMIC_RG_STRUP_VA18_PG_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_buck_rsv_pg_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_BUCK_RSV_PG_ENB_ADDR,
		val,
		PMIC_RG_STRUP_BUCK_RSV_PG_ENB_MASK,
		PMIC_RG_STRUP_BUCK_RSV_PG_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_buck_rsv_pg_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_BUCK_RSV_PG_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_BUCK_RSV_PG_ENB_MASK,
		PMIC_RG_STRUP_BUCK_RSV_PG_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vdram2_pg_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VDRAM2_PG_ENB_ADDR,
		val,
		PMIC_RG_STRUP_VDRAM2_PG_ENB_MASK,
		PMIC_RG_STRUP_VDRAM2_PG_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vdram2_pg_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VDRAM2_PG_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_VDRAM2_PG_ENB_MASK,
		PMIC_RG_STRUP_VDRAM2_PG_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vdram1_pg_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VDRAM1_PG_ENB_ADDR,
		val,
		PMIC_RG_STRUP_VDRAM1_PG_ENB_MASK,
		PMIC_RG_STRUP_VDRAM1_PG_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vdram1_pg_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VDRAM1_PG_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_VDRAM1_PG_ENB_MASK,
		PMIC_RG_STRUP_VDRAM1_PG_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vproc12_pg_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VPROC12_PG_ENB_ADDR,
		val,
		PMIC_RG_STRUP_VPROC12_PG_ENB_MASK,
		PMIC_RG_STRUP_VPROC12_PG_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vproc12_pg_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VPROC12_PG_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_VPROC12_PG_ENB_MASK,
		PMIC_RG_STRUP_VPROC12_PG_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vproc11_pg_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VPROC11_PG_ENB_ADDR,
		val,
		PMIC_RG_STRUP_VPROC11_PG_ENB_MASK,
		PMIC_RG_STRUP_VPROC11_PG_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vproc11_pg_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VPROC11_PG_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_VPROC11_PG_ENB_MASK,
		PMIC_RG_STRUP_VPROC11_PG_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vs1_pg_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VS1_PG_ENB_ADDR,
		val,
		PMIC_RG_STRUP_VS1_PG_ENB_MASK,
		PMIC_RG_STRUP_VS1_PG_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vs1_pg_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VS1_PG_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_VS1_PG_ENB_MASK,
		PMIC_RG_STRUP_VS1_PG_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vmodem_pg_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VMODEM_PG_ENB_ADDR,
		val,
		PMIC_RG_STRUP_VMODEM_PG_ENB_MASK,
		PMIC_RG_STRUP_VMODEM_PG_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vmodem_pg_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VMODEM_PG_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_VMODEM_PG_ENB_MASK,
		PMIC_RG_STRUP_VMODEM_PG_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vgpu_pg_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VGPU_PG_ENB_ADDR,
		val,
		PMIC_RG_STRUP_VGPU_PG_ENB_MASK,
		PMIC_RG_STRUP_VGPU_PG_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vgpu_pg_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VGPU_PG_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_VGPU_PG_ENB_MASK,
		PMIC_RG_STRUP_VGPU_PG_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vcore_pg_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VCORE_PG_ENB_ADDR,
		val,
		PMIC_RG_STRUP_VCORE_PG_ENB_MASK,
		PMIC_RG_STRUP_VCORE_PG_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vcore_pg_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VCORE_PG_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_VCORE_PG_ENB_MASK,
		PMIC_RG_STRUP_VCORE_PG_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vs2_pg_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VS2_PG_ENB_ADDR,
		val,
		PMIC_RG_STRUP_VS2_PG_ENB_MASK,
		PMIC_RG_STRUP_VS2_PG_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vs2_pg_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VS2_PG_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_VS2_PG_ENB_MASK,
		PMIC_RG_STRUP_VS2_PG_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_ext_pmic_pg_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_EXT_PMIC_PG_ENB_ADDR,
		val,
		PMIC_RG_STRUP_EXT_PMIC_PG_ENB_MASK,
		PMIC_RG_STRUP_EXT_PMIC_PG_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_ext_pmic_pg_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_EXT_PMIC_PG_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_EXT_PMIC_PG_ENB_MASK,
		PMIC_RG_STRUP_EXT_PMIC_PG_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vxo18_pg_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VXO18_PG_ENB_ADDR,
		val,
		PMIC_RG_STRUP_VXO18_PG_ENB_MASK,
		PMIC_RG_STRUP_VXO18_PG_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vxo18_pg_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VXO18_PG_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_VXO18_PG_ENB_MASK,
		PMIC_RG_STRUP_VXO18_PG_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vxo22_pg_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VXO22_PG_ENB_ADDR,
		val,
		PMIC_RG_STRUP_VXO22_PG_ENB_MASK,
		PMIC_RG_STRUP_VXO22_PG_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vxo22_pg_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VXO22_PG_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_VXO22_PG_ENB_MASK,
		PMIC_RG_STRUP_VXO22_PG_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vusb33_pg_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VUSB33_PG_ENB_ADDR,
		val,
		PMIC_RG_STRUP_VUSB33_PG_ENB_MASK,
		PMIC_RG_STRUP_VUSB33_PG_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vusb33_pg_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VUSB33_PG_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_VUSB33_PG_ENB_MASK,
		PMIC_RG_STRUP_VUSB33_PG_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vsram_proc_pg_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VSRAM_PROC_PG_ENB_ADDR,
		val,
		PMIC_RG_STRUP_VSRAM_PROC_PG_ENB_MASK,
		PMIC_RG_STRUP_VSRAM_PROC_PG_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vsram_proc_pg_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VSRAM_PROC_PG_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_VSRAM_PROC_PG_ENB_MASK,
		PMIC_RG_STRUP_VSRAM_PROC_PG_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vio28_pg_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VIO28_PG_ENB_ADDR,
		val,
		PMIC_RG_STRUP_VIO28_PG_ENB_MASK,
		PMIC_RG_STRUP_VIO28_PG_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vio28_pg_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VIO28_PG_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_VIO28_PG_ENB_MASK,
		PMIC_RG_STRUP_VIO28_PG_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vufs18_pg_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VUFS18_PG_ENB_ADDR,
		val,
		PMIC_RG_STRUP_VUFS18_PG_ENB_MASK,
		PMIC_RG_STRUP_VUFS18_PG_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vufs18_pg_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VUFS18_PG_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_VUFS18_PG_ENB_MASK,
		PMIC_RG_STRUP_VUFS18_PG_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vemc_pg_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VEMC_PG_ENB_ADDR,
		val,
		PMIC_RG_STRUP_VEMC_PG_ENB_MASK,
		PMIC_RG_STRUP_VEMC_PG_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vemc_pg_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VEMC_PG_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_VEMC_PG_ENB_MASK,
		PMIC_RG_STRUP_VEMC_PG_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vio18_pg_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VIO18_PG_ENB_ADDR,
		val,
		PMIC_RG_STRUP_VIO18_PG_ENB_MASK,
		PMIC_RG_STRUP_VIO18_PG_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vio18_pg_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VIO18_PG_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_VIO18_PG_ENB_MASK,
		PMIC_RG_STRUP_VIO18_PG_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_buck_rsv_oc_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_BUCK_RSV_OC_ENB_ADDR,
		val,
		PMIC_RG_STRUP_BUCK_RSV_OC_ENB_MASK,
		PMIC_RG_STRUP_BUCK_RSV_OC_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_buck_rsv_oc_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_BUCK_RSV_OC_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_BUCK_RSV_OC_ENB_MASK,
		PMIC_RG_STRUP_BUCK_RSV_OC_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vdram2_oc_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VDRAM2_OC_ENB_ADDR,
		val,
		PMIC_RG_STRUP_VDRAM2_OC_ENB_MASK,
		PMIC_RG_STRUP_VDRAM2_OC_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vdram2_oc_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VDRAM2_OC_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_VDRAM2_OC_ENB_MASK,
		PMIC_RG_STRUP_VDRAM2_OC_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vdram1_oc_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VDRAM1_OC_ENB_ADDR,
		val,
		PMIC_RG_STRUP_VDRAM1_OC_ENB_MASK,
		PMIC_RG_STRUP_VDRAM1_OC_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vdram1_oc_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VDRAM1_OC_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_VDRAM1_OC_ENB_MASK,
		PMIC_RG_STRUP_VDRAM1_OC_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vproc12_oc_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VPROC12_OC_ENB_ADDR,
		val,
		PMIC_RG_STRUP_VPROC12_OC_ENB_MASK,
		PMIC_RG_STRUP_VPROC12_OC_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vproc12_oc_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VPROC12_OC_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_VPROC12_OC_ENB_MASK,
		PMIC_RG_STRUP_VPROC12_OC_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vproc11_oc_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VPROC11_OC_ENB_ADDR,
		val,
		PMIC_RG_STRUP_VPROC11_OC_ENB_MASK,
		PMIC_RG_STRUP_VPROC11_OC_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vproc11_oc_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VPROC11_OC_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_VPROC11_OC_ENB_MASK,
		PMIC_RG_STRUP_VPROC11_OC_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vs1_oc_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VS1_OC_ENB_ADDR,
		val,
		PMIC_RG_STRUP_VS1_OC_ENB_MASK,
		PMIC_RG_STRUP_VS1_OC_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vs1_oc_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VS1_OC_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_VS1_OC_ENB_MASK,
		PMIC_RG_STRUP_VS1_OC_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vmodem_oc_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VMODEM_OC_ENB_ADDR,
		val,
		PMIC_RG_STRUP_VMODEM_OC_ENB_MASK,
		PMIC_RG_STRUP_VMODEM_OC_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vmodem_oc_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VMODEM_OC_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_VMODEM_OC_ENB_MASK,
		PMIC_RG_STRUP_VMODEM_OC_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vgpu_oc_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VGPU_OC_ENB_ADDR,
		val,
		PMIC_RG_STRUP_VGPU_OC_ENB_MASK,
		PMIC_RG_STRUP_VGPU_OC_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vgpu_oc_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VGPU_OC_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_VGPU_OC_ENB_MASK,
		PMIC_RG_STRUP_VGPU_OC_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vcore_oc_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VCORE_OC_ENB_ADDR,
		val,
		PMIC_RG_STRUP_VCORE_OC_ENB_MASK,
		PMIC_RG_STRUP_VCORE_OC_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vcore_oc_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VCORE_OC_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_VCORE_OC_ENB_MASK,
		PMIC_RG_STRUP_VCORE_OC_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_vs2_oc_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_VS2_OC_ENB_ADDR,
		val,
		PMIC_RG_STRUP_VS2_OC_ENB_MASK,
		PMIC_RG_STRUP_VS2_OC_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_strup_vs2_oc_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_STRUP_VS2_OC_ENB_ADDR,
		&val,
		PMIC_RG_STRUP_VS2_OC_ENB_MASK,
		PMIC_RG_STRUP_VS2_OC_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_strup_long_press_reset_extend(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_LONG_PRESS_RESET_EXTEND_ADDR,
		val,
		PMIC_RG_STRUP_LONG_PRESS_RESET_EXTEND_MASK,
		PMIC_RG_STRUP_LONG_PRESS_RESET_EXTEND_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ext_pmic_pg_debtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_EXT_PMIC_PG_DEBTD_ADDR,
		val,
		PMIC_RG_EXT_PMIC_PG_DEBTD_MASK,
		PMIC_RG_EXT_PMIC_PG_DEBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_rtc_spar_deb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RTC_SPAR_DEB_EN_ADDR,
		val,
		PMIC_RG_RTC_SPAR_DEB_EN_MASK,
		PMIC_RG_RTC_SPAR_DEB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_rtc_spar_deb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_RTC_SPAR_DEB_EN_ADDR,
		&val,
		PMIC_RG_RTC_SPAR_DEB_EN_MASK,
		PMIC_RG_RTC_SPAR_DEB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_rtc_alarm_deb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RTC_ALARM_DEB_EN_ADDR,
		val,
		PMIC_RG_RTC_ALARM_DEB_EN_MASK,
		PMIC_RG_RTC_ALARM_DEB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_rtc_alarm_deb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_RTC_ALARM_DEB_EN_ADDR,
		&val,
		PMIC_RG_RTC_ALARM_DEB_EN_MASK,
		PMIC_RG_RTC_ALARM_DEB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_tm_out(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_TM_OUT_ADDR,
		val,
		PMIC_RG_TM_OUT_MASK,
		PMIC_RG_TM_OUT_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_thr_loc_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_THR_LOC_SEL_ADDR,
		val,
		PMIC_RG_THR_LOC_SEL_MASK,
		PMIC_RG_THR_LOC_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_thrdet_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_THRDET_SEL_ADDR,
		val,
		PMIC_RG_THRDET_SEL_MASK,
		PMIC_RG_THRDET_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_strup_thr_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_THR_SEL_ADDR,
		val,
		PMIC_RG_STRUP_THR_SEL_MASK,
		PMIC_RG_STRUP_THR_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_thr_tmode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_THR_TMODE_ADDR,
		val,
		PMIC_RG_THR_TMODE_MASK,
		PMIC_RG_THR_TMODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vref_bg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VREF_BG_ADDR,
		val,
		PMIC_RG_VREF_BG_MASK,
		PMIC_RG_VREF_BG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_strup_iref_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_STRUP_IREF_TRIM_ADDR,
		val,
		PMIC_RG_STRUP_IREF_TRIM_MASK,
		PMIC_RG_STRUP_IREF_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_rst_drvsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RST_DRVSEL_ADDR,
		val,
		PMIC_RG_RST_DRVSEL_MASK,
		PMIC_RG_RST_DRVSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_en_drvsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_EN_DRVSEL_ADDR,
		val,
		PMIC_RG_EN_DRVSEL_MASK,
		PMIC_RG_EN_DRVSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_pmu_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_PMU_RSV_ADDR,
		val,
		PMIC_RG_PMU_RSV_MASK,
		PMIC_RG_PMU_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rgs_ana_chip_id(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_ANA_CHIP_ID_ADDR,
		&val,
		PMIC_RGS_ANA_CHIP_ID_MASK,
		PMIC_RGS_ANA_CHIP_ID_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_pwrhold(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_PWRHOLD_ADDR,
		val,
		PMIC_RG_PWRHOLD_MASK,
		PMIC_RG_PWRHOLD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_usbdl_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_USBDL_MODE_ADDR,
		val,
		PMIC_RG_USBDL_MODE_MASK,
		PMIC_RG_USBDL_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_crst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_CRST_ADDR,
		val,
		PMIC_RG_CRST_MASK,
		PMIC_RG_CRST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_wrst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_WRST_ADDR,
		val,
		PMIC_RG_WRST_MASK,
		PMIC_RG_WRST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_rstb_onintv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_RSTB_ONINTV_ADDR,
		val,
		PMIC_RG_RSTB_ONINTV_MASK,
		PMIC_RG_RSTB_ONINTV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_crst_intv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_CRST_INTV_ADDR,
		val,
		PMIC_RG_CRST_INTV_MASK,
		PMIC_RG_CRST_INTV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_wrst_intv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_WRST_INTV_ADDR,
		val,
		PMIC_RG_WRST_INTV_MASK,
		PMIC_RG_WRST_INTV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_pseq_ivgen_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_PSEQ_IVGEN_SEL_ADDR,
		val,
		PMIC_RG_PSEQ_IVGEN_SEL_MASK,
		PMIC_RG_PSEQ_IVGEN_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_pseq_fsm_rst_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_PSEQ_FSM_RST_SEL_ADDR,
		val,
		PMIC_RG_PSEQ_FSM_RST_SEL_MASK,
		PMIC_RG_PSEQ_FSM_RST_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_pseq_pg_ck_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_PSEQ_PG_CK_SEL_ADDR,
		val,
		PMIC_RG_PSEQ_PG_CK_SEL_MASK,
		PMIC_RG_PSEQ_PG_CK_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_pseq_1ms_tk_ext(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_PSEQ_1MS_TK_EXT_ADDR,
		val,
		PMIC_RG_PSEQ_1MS_TK_EXT_MASK,
		PMIC_RG_PSEQ_1MS_TK_EXT_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_pseq_spar_xcpt_mask(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_PSEQ_SPAR_XCPT_MASK_ADDR,
		val,
		PMIC_RG_PSEQ_SPAR_XCPT_MASK_MASK,
		PMIC_RG_PSEQ_SPAR_XCPT_MASK_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_pseq_rtca_xcpt_mask(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_PSEQ_RTCA_XCPT_MASK_ADDR,
		val,
		PMIC_RG_PSEQ_RTCA_XCPT_MASK_MASK,
		PMIC_RG_PSEQ_RTCA_XCPT_MASK_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_wdtrst_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_WDTRST_EN_ADDR,
		val,
		PMIC_RG_WDTRST_EN_MASK,
		PMIC_RG_WDTRST_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_wdtrst_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_WDTRST_EN_ADDR,
		&val,
		PMIC_RG_WDTRST_EN_MASK,
		PMIC_RG_WDTRST_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_wdtrst_act(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_WDTRST_ACT_ADDR,
		val,
		PMIC_RG_WDTRST_ACT_MASK,
		PMIC_RG_WDTRST_ACT_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_pspg_shdn_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_PSPG_SHDN_EN_ADDR,
		val,
		PMIC_RG_PSPG_SHDN_EN_MASK,
		PMIC_RG_PSPG_SHDN_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_pspg_shdn_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_PSPG_SHDN_EN_ADDR,
		&val,
		PMIC_RG_PSPG_SHDN_EN_MASK,
		PMIC_RG_PSPG_SHDN_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_thm_shdn_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_THM_SHDN_EN_ADDR,
		val,
		PMIC_RG_THM_SHDN_EN_MASK,
		PMIC_RG_THM_SHDN_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_thm_shdn_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_THM_SHDN_EN_ADDR,
		&val,
		PMIC_RG_THM_SHDN_EN_MASK,
		PMIC_RG_THM_SHDN_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_keypwr_vcore_opt(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_KEYPWR_VCORE_OPT_ADDR,
		val,
		PMIC_RG_KEYPWR_VCORE_OPT_MASK,
		PMIC_RG_KEYPWR_VCORE_OPT_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_pseq_force_on(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_PSEQ_FORCE_ON_ADDR,
		val,
		PMIC_RG_PSEQ_FORCE_ON_MASK,
		PMIC_RG_PSEQ_FORCE_ON_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_pseq_force_all_doff(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_PSEQ_FORCE_ALL_DOFF_ADDR,
		val,
		PMIC_RG_PSEQ_FORCE_ALL_DOFF_MASK,
		PMIC_RG_PSEQ_FORCE_ALL_DOFF_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_pseq_f75k_force(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_PSEQ_F75K_FORCE_ADDR,
		val,
		PMIC_RG_PSEQ_F75K_FORCE_MASK,
		PMIC_RG_PSEQ_F75K_FORCE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_pseq_rsv0(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_PSEQ_RSV0_ADDR,
		val,
		PMIC_RG_PSEQ_RSV0_MASK,
		PMIC_RG_PSEQ_RSV0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_pseq_rsv1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_PSEQ_RSV1_ADDR,
		val,
		PMIC_RG_PSEQ_RSV1_MASK,
		PMIC_RG_PSEQ_RSV1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_pseq_rsv2(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_PSEQ_RSV2_ADDR,
		val,
		PMIC_RG_PSEQ_RSV2_MASK,
		PMIC_RG_PSEQ_RSV2_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_bwdt_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BWDT_EN_ADDR,
		val,
		PMIC_RG_BWDT_EN_MASK,
		PMIC_RG_BWDT_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_bwdt_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BWDT_EN_ADDR,
		&val,
		PMIC_RG_BWDT_EN_MASK,
		PMIC_RG_BWDT_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_bwdt_tsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BWDT_TSEL_ADDR,
		val,
		PMIC_RG_BWDT_TSEL_MASK,
		PMIC_RG_BWDT_TSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_bwdt_csel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BWDT_CSEL_ADDR,
		val,
		PMIC_RG_BWDT_CSEL_MASK,
		PMIC_RG_BWDT_CSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_bwdt_td(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BWDT_TD_ADDR,
		val,
		PMIC_RG_BWDT_TD_MASK,
		PMIC_RG_BWDT_TD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_bwdt_chrtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BWDT_CHRTD_ADDR,
		val,
		PMIC_RG_BWDT_CHRTD_MASK,
		PMIC_RG_BWDT_CHRTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_bwdt_ddlo_td(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BWDT_DDLO_TD_ADDR,
		val,
		PMIC_RG_BWDT_DDLO_TD_MASK,
		PMIC_RG_BWDT_DDLO_TD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_bwdt_srcsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BWDT_SRCSEL_ADDR,
		val,
		PMIC_RG_BWDT_SRCSEL_MASK,
		PMIC_RG_BWDT_SRCSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_cps_w_key(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_CPS_W_KEY_ADDR,
		val,
		PMIC_RG_CPS_W_KEY_MASK,
		PMIC_RG_CPS_W_KEY_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_slot_intv_up(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SLOT_INTV_UP_ADDR,
		val,
		PMIC_RG_SLOT_INTV_UP_MASK,
		PMIC_RG_SLOT_INTV_UP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_seq_len(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SEQ_LEN_ADDR,
		val,
		PMIC_RG_SEQ_LEN_MASK,
		PMIC_RG_SEQ_LEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_slot_intv_down(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SLOT_INTV_DOWN_ADDR,
		val,
		PMIC_RG_SLOT_INTV_DOWN_MASK,
		PMIC_RG_SLOT_INTV_DOWN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_dseq_len(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_DSEQ_LEN_ADDR,
		val,
		PMIC_RG_DSEQ_LEN_MASK,
		PMIC_RG_DSEQ_LEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs2_usa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS2_USA_ADDR,
		val,
		PMIC_RG_VS2_USA_MASK,
		PMIC_RG_VS2_USA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vsram_core_usa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VSRAM_CORE_USA_ADDR,
		val,
		PMIC_RG_VSRAM_CORE_USA_MASK,
		PMIC_RG_VSRAM_CORE_USA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vsram_md_usa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VSRAM_MD_USA_ADDR,
		val,
		PMIC_RG_VSRAM_MD_USA_MASK,
		PMIC_RG_VSRAM_MD_USA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vsram_gpu_usa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VSRAM_GPU_USA_ADDR,
		val,
		PMIC_RG_VSRAM_GPU_USA_MASK,
		PMIC_RG_VSRAM_GPU_USA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_usa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_USA_ADDR,
		val,
		PMIC_RG_VCORE_USA_MASK,
		PMIC_RG_VCORE_USA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_usa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_USA_ADDR,
		val,
		PMIC_RG_VGPU_USA_MASK,
		PMIC_RG_VGPU_USA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_usa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_USA_ADDR,
		val,
		PMIC_RG_VMODEM_USA_MASK,
		PMIC_RG_VMODEM_USA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs1_usa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS1_USA_ADDR,
		val,
		PMIC_RG_VS1_USA_MASK,
		PMIC_RG_VS1_USA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_va10_usa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VA10_USA_ADDR,
		val,
		PMIC_RG_VA10_USA_MASK,
		PMIC_RG_VA10_USA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_va12_usa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VA12_USA_ADDR,
		val,
		PMIC_RG_VA12_USA_MASK,
		PMIC_RG_VA12_USA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vio18_usa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VIO18_USA_ADDR,
		val,
		PMIC_RG_VIO18_USA_MASK,
		PMIC_RG_VIO18_USA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vemc_usa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VEMC_USA_ADDR,
		val,
		PMIC_RG_VEMC_USA_MASK,
		PMIC_RG_VEMC_USA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vufs18_usa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VUFS18_USA_ADDR,
		val,
		PMIC_RG_VUFS18_USA_MASK,
		PMIC_RG_VUFS18_USA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vio28_usa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VIO28_USA_ADDR,
		val,
		PMIC_RG_VIO28_USA_MASK,
		PMIC_RG_VIO28_USA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vsram_proc_usa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VSRAM_PROC_USA_ADDR,
		val,
		PMIC_RG_VSRAM_PROC_USA_MASK,
		PMIC_RG_VSRAM_PROC_USA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_usa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_USA_ADDR,
		val,
		PMIC_RG_VPROC11_USA_MASK,
		PMIC_RG_VPROC11_USA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_usa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_USA_ADDR,
		val,
		PMIC_RG_VPROC12_USA_MASK,
		PMIC_RG_VPROC12_USA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ext_pmic_usa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_EXT_PMIC_USA_ADDR,
		val,
		PMIC_RG_EXT_PMIC_USA_MASK,
		PMIC_RG_EXT_PMIC_USA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_usa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_USA_ADDR,
		val,
		PMIC_RG_VDRAM1_USA_MASK,
		PMIC_RG_VDRAM1_USA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_usa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_USA_ADDR,
		val,
		PMIC_RG_VDRAM2_USA_MASK,
		PMIC_RG_VDRAM2_USA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vusb33_usa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VUSB33_USA_ADDR,
		val,
		PMIC_RG_VUSB33_USA_MASK,
		PMIC_RG_VUSB33_USA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vxo22_usa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VXO22_USA_ADDR,
		val,
		PMIC_RG_VXO22_USA_MASK,
		PMIC_RG_VXO22_USA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vxo18_usa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VXO18_USA_ADDR,
		val,
		PMIC_RG_VXO18_USA_MASK,
		PMIC_RG_VXO18_USA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_rsv_usa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_RSV_USA_ADDR,
		val,
		PMIC_RG_BUCK_RSV_USA_MASK,
		PMIC_RG_BUCK_RSV_USA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs2_dsa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS2_DSA_ADDR,
		val,
		PMIC_RG_VS2_DSA_MASK,
		PMIC_RG_VS2_DSA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vsram_core_dsa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VSRAM_CORE_DSA_ADDR,
		val,
		PMIC_RG_VSRAM_CORE_DSA_MASK,
		PMIC_RG_VSRAM_CORE_DSA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vsram_md_dsa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VSRAM_MD_DSA_ADDR,
		val,
		PMIC_RG_VSRAM_MD_DSA_MASK,
		PMIC_RG_VSRAM_MD_DSA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vsram_gpu_dsa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VSRAM_GPU_DSA_ADDR,
		val,
		PMIC_RG_VSRAM_GPU_DSA_MASK,
		PMIC_RG_VSRAM_GPU_DSA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_dsa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_DSA_ADDR,
		val,
		PMIC_RG_VCORE_DSA_MASK,
		PMIC_RG_VCORE_DSA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_dsa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_DSA_ADDR,
		val,
		PMIC_RG_VGPU_DSA_MASK,
		PMIC_RG_VGPU_DSA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_dsa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_DSA_ADDR,
		val,
		PMIC_RG_VMODEM_DSA_MASK,
		PMIC_RG_VMODEM_DSA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs1_dsa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS1_DSA_ADDR,
		val,
		PMIC_RG_VS1_DSA_MASK,
		PMIC_RG_VS1_DSA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_va10_dsa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VA10_DSA_ADDR,
		val,
		PMIC_RG_VA10_DSA_MASK,
		PMIC_RG_VA10_DSA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_va12_dsa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VA12_DSA_ADDR,
		val,
		PMIC_RG_VA12_DSA_MASK,
		PMIC_RG_VA12_DSA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vio18_dsa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VIO18_DSA_ADDR,
		val,
		PMIC_RG_VIO18_DSA_MASK,
		PMIC_RG_VIO18_DSA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vemc_dsa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VEMC_DSA_ADDR,
		val,
		PMIC_RG_VEMC_DSA_MASK,
		PMIC_RG_VEMC_DSA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vufs18_dsa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VUFS18_DSA_ADDR,
		val,
		PMIC_RG_VUFS18_DSA_MASK,
		PMIC_RG_VUFS18_DSA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vio28_dsa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VIO28_DSA_ADDR,
		val,
		PMIC_RG_VIO28_DSA_MASK,
		PMIC_RG_VIO28_DSA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vsram_proc_dsa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VSRAM_PROC_DSA_ADDR,
		val,
		PMIC_RG_VSRAM_PROC_DSA_MASK,
		PMIC_RG_VSRAM_PROC_DSA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_dsa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_DSA_ADDR,
		val,
		PMIC_RG_VPROC11_DSA_MASK,
		PMIC_RG_VPROC11_DSA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_dsa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_DSA_ADDR,
		val,
		PMIC_RG_VPROC12_DSA_MASK,
		PMIC_RG_VPROC12_DSA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ext_pmic_dsa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_EXT_PMIC_DSA_ADDR,
		val,
		PMIC_RG_EXT_PMIC_DSA_MASK,
		PMIC_RG_EXT_PMIC_DSA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_dsa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_DSA_ADDR,
		val,
		PMIC_RG_VDRAM1_DSA_MASK,
		PMIC_RG_VDRAM1_DSA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_dsa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_DSA_ADDR,
		val,
		PMIC_RG_VDRAM2_DSA_MASK,
		PMIC_RG_VDRAM2_DSA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vusb33_dsa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VUSB33_DSA_ADDR,
		val,
		PMIC_RG_VUSB33_DSA_MASK,
		PMIC_RG_VUSB33_DSA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vxo22_dsa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VXO22_DSA_ADDR,
		val,
		PMIC_RG_VXO22_DSA_MASK,
		PMIC_RG_VXO22_DSA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vxo18_dsa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VXO18_DSA_ADDR,
		val,
		PMIC_RG_VXO18_DSA_MASK,
		PMIC_RG_VXO18_DSA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_rsv_dsa(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_RSV_DSA_ADDR,
		val,
		PMIC_RG_BUCK_RSV_DSA_MASK,
		PMIC_RG_BUCK_RSV_DSA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_por_flag(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_POR_FLAG_ADDR,
		val,
		PMIC_RG_POR_FLAG_MASK,
		PMIC_RG_POR_FLAG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_sts_pwrkey(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STS_PWRKEY_ADDR,
		&val,
		PMIC_STS_PWRKEY_MASK,
		PMIC_STS_PWRKEY_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_sts_rtca(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STS_RTCA_ADDR,
		&val,
		PMIC_STS_RTCA_MASK,
		PMIC_STS_RTCA_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_sts_chrin(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STS_CHRIN_ADDR,
		&val,
		PMIC_STS_CHRIN_MASK,
		PMIC_STS_CHRIN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_sts_spar(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STS_SPAR_ADDR,
		&val,
		PMIC_STS_SPAR_MASK,
		PMIC_STS_SPAR_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_sts_rboot(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STS_RBOOT_ADDR,
		&val,
		PMIC_STS_RBOOT_MASK,
		PMIC_STS_RBOOT_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_sts_uvlo(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STS_UVLO_ADDR,
		&val,
		PMIC_STS_UVLO_MASK,
		PMIC_STS_UVLO_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_sts_pgfail(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STS_PGFAIL_ADDR,
		&val,
		PMIC_STS_PGFAIL_MASK,
		PMIC_STS_PGFAIL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_sts_psoc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STS_PSOC_ADDR,
		&val,
		PMIC_STS_PSOC_MASK,
		PMIC_STS_PSOC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_sts_thrdn(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STS_THRDN_ADDR,
		&val,
		PMIC_STS_THRDN_MASK,
		PMIC_STS_THRDN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_sts_wrst(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STS_WRST_ADDR,
		&val,
		PMIC_STS_WRST_MASK,
		PMIC_STS_WRST_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_sts_crst(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STS_CRST_ADDR,
		&val,
		PMIC_STS_CRST_MASK,
		PMIC_STS_CRST_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_sts_pkeylp(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STS_PKEYLP_ADDR,
		&val,
		PMIC_STS_PKEYLP_MASK,
		PMIC_STS_PKEYLP_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_sts_normoff(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STS_NORMOFF_ADDR,
		&val,
		PMIC_STS_NORMOFF_MASK,
		PMIC_STS_NORMOFF_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_sts_bwdt(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STS_BWDT_ADDR,
		&val,
		PMIC_STS_BWDT_MASK,
		PMIC_STS_BWDT_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_sts_ddlo(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STS_DDLO_ADDR,
		&val,
		PMIC_STS_DDLO_MASK,
		PMIC_STS_DDLO_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_sts_wdt(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STS_WDT_ADDR,
		&val,
		PMIC_STS_WDT_MASK,
		PMIC_STS_WDT_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_sts_pupsrc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STS_PUPSRC_ADDR,
		&val,
		PMIC_STS_PUPSRC_MASK,
		PMIC_STS_PUPSRC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_sts_keypwr(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_STS_KEYPWR_ADDR,
		&val,
		PMIC_STS_KEYPWR_MASK,
		PMIC_STS_KEYPWR_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_poffsts_clr(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_POFFSTS_CLR_ADDR,
		val,
		PMIC_RG_POFFSTS_CLR_MASK,
		PMIC_RG_POFFSTS_CLR_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ponsts_clr(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_PONSTS_CLR_ADDR,
		val,
		PMIC_RG_PONSTS_CLR_MASK,
		PMIC_RG_PONSTS_CLR_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_ldo_ft_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_LDO_FT_EN_ADDR,
		val,
		PMIC_RG_BUCK_LDO_FT_EN_MASK,
		PMIC_RG_BUCK_LDO_FT_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_ldo_ft_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_LDO_FT_EN_ADDR,
		&val,
		PMIC_RG_BUCK_LDO_FT_EN_MASK,
		PMIC_RG_BUCK_LDO_FT_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_dcm_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_DCM_MODE_ADDR,
		val,
		PMIC_RG_BUCK_DCM_MODE_MASK,
		PMIC_RG_BUCK_DCM_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_all_con0_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_ALL_CON0_RSV_ADDR,
		val,
		PMIC_RG_BUCK_ALL_CON0_RSV_MASK,
		PMIC_RG_BUCK_ALL_CON0_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_stb_max(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_STB_MAX_ADDR,
		val,
		PMIC_RG_BUCK_STB_MAX_MASK,
		PMIC_RG_BUCK_STB_MAX_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_lp_prot_disable(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_LP_PROT_DISABLE_ADDR,
		val,
		PMIC_RG_BUCK_LP_PROT_DISABLE_MASK,
		PMIC_RG_BUCK_LP_PROT_DISABLE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vsleep_src0(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VSLEEP_SRC0_ADDR,
		val,
		PMIC_RG_BUCK_VSLEEP_SRC0_MASK,
		PMIC_RG_BUCK_VSLEEP_SRC0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vsleep_src1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VSLEEP_SRC1_ADDR,
		val,
		PMIC_RG_BUCK_VSLEEP_SRC1_MASK,
		PMIC_RG_BUCK_VSLEEP_SRC1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_r2r_src0(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_R2R_SRC0_ADDR,
		val,
		PMIC_RG_BUCK_R2R_SRC0_MASK,
		PMIC_RG_BUCK_R2R_SRC0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_r2r_src1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_R2R_SRC1_ADDR,
		val,
		PMIC_RG_BUCK_R2R_SRC1_MASK,
		PMIC_RG_BUCK_R2R_SRC1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_lp_seq_count(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_LP_SEQ_COUNT_ADDR,
		val,
		PMIC_RG_BUCK_LP_SEQ_COUNT_MASK,
		PMIC_RG_BUCK_LP_SEQ_COUNT_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_on_seq_count(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_ON_SEQ_COUNT_ADDR,
		val,
		PMIC_RG_BUCK_ON_SEQ_COUNT_MASK,
		PMIC_RG_BUCK_ON_SEQ_COUNT_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_minfreq_latency_max(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_MINFREQ_LATENCY_MAX_ADDR,
		val,
		PMIC_RG_BUCK_MINFREQ_LATENCY_MAX_MASK,
		PMIC_RG_BUCK_MINFREQ_LATENCY_MAX_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_minfreq_duration_max(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_MINFREQ_DURATION_MAX_ADDR,
		val,
		PMIC_RG_BUCK_MINFREQ_DURATION_MAX_MASK,
		PMIC_RG_BUCK_MINFREQ_DURATION_MAX_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc11_oc_sdn_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC11_OC_SDN_STATUS_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC11_OC_SDN_STATUS_MASK,
		PMIC_RG_BUCK_VPROC11_OC_SDN_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_buck_vproc12_oc_sdn_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC12_OC_SDN_STATUS_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC12_OC_SDN_STATUS_MASK,
		PMIC_RG_BUCK_VPROC12_OC_SDN_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_buck_vcore_oc_sdn_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VCORE_OC_SDN_STATUS_ADDR,
		&val,
		PMIC_RG_BUCK_VCORE_OC_SDN_STATUS_MASK,
		PMIC_RG_BUCK_VCORE_OC_SDN_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_buck_vgpu_oc_sdn_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VGPU_OC_SDN_STATUS_ADDR,
		&val,
		PMIC_RG_BUCK_VGPU_OC_SDN_STATUS_MASK,
		PMIC_RG_BUCK_VGPU_OC_SDN_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_buck_vdram1_oc_sdn_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM1_OC_SDN_STATUS_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM1_OC_SDN_STATUS_MASK,
		PMIC_RG_BUCK_VDRAM1_OC_SDN_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_buck_vdram2_oc_sdn_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM2_OC_SDN_STATUS_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM2_OC_SDN_STATUS_MASK,
		PMIC_RG_BUCK_VDRAM2_OC_SDN_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_buck_vmodem_oc_sdn_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VMODEM_OC_SDN_STATUS_ADDR,
		&val,
		PMIC_RG_BUCK_VMODEM_OC_SDN_STATUS_MASK,
		PMIC_RG_BUCK_VMODEM_OC_SDN_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_buck_vs1_oc_sdn_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS1_OC_SDN_STATUS_ADDR,
		&val,
		PMIC_RG_BUCK_VS1_OC_SDN_STATUS_MASK,
		PMIC_RG_BUCK_VS1_OC_SDN_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_buck_vs2_oc_sdn_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS2_OC_SDN_STATUS_ADDR,
		&val,
		PMIC_RG_BUCK_VS2_OC_SDN_STATUS_MASK,
		PMIC_RG_BUCK_VS2_OC_SDN_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rg_buck_vpa_oc_sdn_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPA_OC_SDN_STATUS_ADDR,
		&val,
		PMIC_RG_BUCK_VPA_OC_SDN_STATUS_MASK,
		PMIC_RG_BUCK_VPA_OC_SDN_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_oc_sdn_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_OC_SDN_EN_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_OC_SDN_EN_MASK,
		PMIC_RG_BUCK_VPROC11_OC_SDN_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc11_oc_sdn_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC11_OC_SDN_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC11_OC_SDN_EN_MASK,
		PMIC_RG_BUCK_VPROC11_OC_SDN_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_oc_sdn_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_OC_SDN_EN_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_OC_SDN_EN_MASK,
		PMIC_RG_BUCK_VPROC12_OC_SDN_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc12_oc_sdn_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC12_OC_SDN_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC12_OC_SDN_EN_MASK,
		PMIC_RG_BUCK_VPROC12_OC_SDN_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_oc_sdn_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_OC_SDN_EN_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_OC_SDN_EN_MASK,
		PMIC_RG_BUCK_VCORE_OC_SDN_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vcore_oc_sdn_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VCORE_OC_SDN_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VCORE_OC_SDN_EN_MASK,
		PMIC_RG_BUCK_VCORE_OC_SDN_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_oc_sdn_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_OC_SDN_EN_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_OC_SDN_EN_MASK,
		PMIC_RG_BUCK_VGPU_OC_SDN_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vgpu_oc_sdn_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VGPU_OC_SDN_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VGPU_OC_SDN_EN_MASK,
		PMIC_RG_BUCK_VGPU_OC_SDN_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_oc_sdn_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_OC_SDN_EN_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_OC_SDN_EN_MASK,
		PMIC_RG_BUCK_VDRAM1_OC_SDN_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram1_oc_sdn_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM1_OC_SDN_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM1_OC_SDN_EN_MASK,
		PMIC_RG_BUCK_VDRAM1_OC_SDN_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_oc_sdn_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_OC_SDN_EN_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_OC_SDN_EN_MASK,
		PMIC_RG_BUCK_VDRAM2_OC_SDN_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram2_oc_sdn_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM2_OC_SDN_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM2_OC_SDN_EN_MASK,
		PMIC_RG_BUCK_VDRAM2_OC_SDN_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_oc_sdn_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_OC_SDN_EN_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_OC_SDN_EN_MASK,
		PMIC_RG_BUCK_VMODEM_OC_SDN_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vmodem_oc_sdn_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VMODEM_OC_SDN_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VMODEM_OC_SDN_EN_MASK,
		PMIC_RG_BUCK_VMODEM_OC_SDN_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_oc_sdn_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_OC_SDN_EN_ADDR,
		val,
		PMIC_RG_BUCK_VS1_OC_SDN_EN_MASK,
		PMIC_RG_BUCK_VS1_OC_SDN_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs1_oc_sdn_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS1_OC_SDN_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VS1_OC_SDN_EN_MASK,
		PMIC_RG_BUCK_VS1_OC_SDN_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_oc_sdn_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_OC_SDN_EN_ADDR,
		val,
		PMIC_RG_BUCK_VS2_OC_SDN_EN_MASK,
		PMIC_RG_BUCK_VS2_OC_SDN_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs2_oc_sdn_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS2_OC_SDN_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VS2_OC_SDN_EN_MASK,
		PMIC_RG_BUCK_VS2_OC_SDN_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_oc_sdn_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_OC_SDN_EN_ADDR,
		val,
		PMIC_RG_BUCK_VPA_OC_SDN_EN_MASK,
		PMIC_RG_BUCK_VPA_OC_SDN_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vpa_oc_sdn_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPA_OC_SDN_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VPA_OC_SDN_EN_MASK,
		PMIC_RG_BUCK_VPA_OC_SDN_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_k_rst_done(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_K_RST_DONE_ADDR,
		val,
		PMIC_RG_BUCK_K_RST_DONE_MASK,
		PMIC_RG_BUCK_K_RST_DONE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_k_map_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_K_MAP_SEL_ADDR,
		val,
		PMIC_RG_BUCK_K_MAP_SEL_MASK,
		PMIC_RG_BUCK_K_MAP_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_k_once_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_K_ONCE_EN_ADDR,
		val,
		PMIC_RG_BUCK_K_ONCE_EN_MASK,
		PMIC_RG_BUCK_K_ONCE_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_k_once_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_K_ONCE_EN_ADDR,
		&val,
		PMIC_RG_BUCK_K_ONCE_EN_MASK,
		PMIC_RG_BUCK_K_ONCE_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_k_once(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_K_ONCE_ADDR,
		val,
		PMIC_RG_BUCK_K_ONCE_MASK,
		PMIC_RG_BUCK_K_ONCE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_k_start_manual(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_K_START_MANUAL_ADDR,
		val,
		PMIC_RG_BUCK_K_START_MANUAL_MASK,
		PMIC_RG_BUCK_K_START_MANUAL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_k_src_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_K_SRC_SEL_ADDR,
		val,
		PMIC_RG_BUCK_K_SRC_SEL_MASK,
		PMIC_RG_BUCK_K_SRC_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_k_auto_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_K_AUTO_EN_ADDR,
		val,
		PMIC_RG_BUCK_K_AUTO_EN_MASK,
		PMIC_RG_BUCK_K_AUTO_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_k_auto_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_K_AUTO_EN_ADDR,
		&val,
		PMIC_RG_BUCK_K_AUTO_EN_MASK,
		PMIC_RG_BUCK_K_AUTO_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_k_inv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_K_INV_ADDR,
		val,
		PMIC_RG_BUCK_K_INV_MASK,
		PMIC_RG_BUCK_K_INV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_k_ck_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_K_CK_EN_ADDR,
		val,
		PMIC_RG_BUCK_K_CK_EN_MASK,
		PMIC_RG_BUCK_K_CK_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_k_ck_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_K_CK_EN_ADDR,
		&val,
		PMIC_RG_BUCK_K_CK_EN_MASK,
		PMIC_RG_BUCK_K_CK_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_k_control_smps(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_K_CONTROL_SMPS_ADDR,
		val,
		PMIC_RG_BUCK_K_CONTROL_SMPS_MASK,
		PMIC_RG_BUCK_K_CONTROL_SMPS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_buck_k_result(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_BUCK_K_RESULT_ADDR,
		&val,
		PMIC_BUCK_K_RESULT_MASK,
		PMIC_BUCK_K_RESULT_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_buck_k_done(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_BUCK_K_DONE_ADDR,
		&val,
		PMIC_BUCK_K_DONE_MASK,
		PMIC_BUCK_K_DONE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_buck_k_control(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_BUCK_K_CONTROL_ADDR,
		&val,
		PMIC_BUCK_K_CONTROL_MASK,
		PMIC_BUCK_K_CONTROL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_smps_osc_cal(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_SMPS_OSC_CAL_ADDR,
		&val,
		PMIC_DA_QI_SMPS_OSC_CAL_MASK,
		PMIC_DA_QI_SMPS_OSC_CAL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_k_buck_ck_cnt(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_K_BUCK_CK_CNT_ADDR,
		val,
		PMIC_RG_BUCK_K_BUCK_CK_CNT_MASK,
		PMIC_RG_BUCK_K_BUCK_CK_CNT_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vow_buck_vcore_dvs_done(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VOW_BUCK_VCORE_DVS_DONE_ADDR,
		val,
		PMIC_RG_VOW_BUCK_VCORE_DVS_DONE_MASK,
		PMIC_RG_VOW_BUCK_VCORE_DVS_DONE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vow_ldo_vsram_core_dvs_done(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VOW_LDO_VSRAM_CORE_DVS_DONE_ADDR,
		val,
		PMIC_RG_VOW_LDO_VSRAM_CORE_DVS_DONE_MASK,
		PMIC_RG_VOW_LDO_VSRAM_CORE_DVS_DONE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vow_buck_vcore_dvs_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VOW_BUCK_VCORE_DVS_SW_MODE_ADDR,
		val,
		PMIC_RG_VOW_BUCK_VCORE_DVS_SW_MODE_MASK,
		PMIC_RG_VOW_BUCK_VCORE_DVS_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vow_ldo_vsram_core_dvs_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VOW_LDO_VSRAM_CORE_DVS_SW_MODE_ADDR,
		val,
		PMIC_RG_VOW_LDO_VSRAM_CORE_DVS_SW_MODE_MASK,
		PMIC_RG_VOW_LDO_VSRAM_CORE_DVS_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_vow_buck_vcore_dvs_done(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VOW_BUCK_VCORE_DVS_DONE_ADDR,
		&val,
		PMIC_VOW_BUCK_VCORE_DVS_DONE_MASK,
		PMIC_VOW_BUCK_VCORE_DVS_DONE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vow_ldo_vsram_core_dvs_done(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VOW_LDO_VSRAM_CORE_DVS_DONE_ADDR,
		&val,
		PMIC_VOW_LDO_VSRAM_CORE_DVS_DONE_MASK,
		PMIC_VOW_LDO_VSRAM_CORE_DVS_DONE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vow_dvs_done(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VOW_DVS_DONE_ADDR,
		&val,
		PMIC_VOW_DVS_DONE_MASK,
		PMIC_VOW_DVS_DONE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_EN_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_EN_MASK,
		PMIC_RG_BUCK_VPROC11_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc11_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC11_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC11_EN_MASK,
		PMIC_RG_BUCK_VPROC11_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_LP_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_LP_MASK,
		PMIC_RG_BUCK_VPROC11_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_VOSEL_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_VOSEL_MASK,
		PMIC_RG_BUCK_VPROC11_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc11_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC11_VOSEL_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC11_VOSEL_MASK,
		PMIC_RG_BUCK_VPROC11_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_vosel_sleep(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_VOSEL_SLEEP_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_VOSEL_SLEEP_MASK,
		PMIC_RG_BUCK_VPROC11_VOSEL_SLEEP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc11_vosel_sleep(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC11_VOSEL_SLEEP_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC11_VOSEL_SLEEP_MASK,
		PMIC_RG_BUCK_VPROC11_VOSEL_SLEEP_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_sfchg_frate(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_SFCHG_FRATE_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_SFCHG_FRATE_MASK,
		PMIC_RG_BUCK_VPROC11_SFCHG_FRATE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_sfchg_fen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_SFCHG_FEN_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_SFCHG_FEN_MASK,
		PMIC_RG_BUCK_VPROC11_SFCHG_FEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_sfchg_rrate(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_SFCHG_RRATE_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_SFCHG_RRATE_MASK,
		PMIC_RG_BUCK_VPROC11_SFCHG_RRATE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_sfchg_ren(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_SFCHG_REN_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_SFCHG_REN_MASK,
		PMIC_RG_BUCK_VPROC11_SFCHG_REN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_dvs_en_td(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_DVS_EN_TD_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_DVS_EN_TD_MASK,
		PMIC_RG_BUCK_VPROC11_DVS_EN_TD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc11_dvs_en_td(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC11_DVS_EN_TD_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC11_DVS_EN_TD_MASK,
		PMIC_RG_BUCK_VPROC11_DVS_EN_TD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_dvs_en_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_DVS_EN_CTRL_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_DVS_EN_CTRL_MASK,
		PMIC_RG_BUCK_VPROC11_DVS_EN_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc11_dvs_en_ctrl(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC11_DVS_EN_CTRL_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC11_DVS_EN_CTRL_MASK,
		PMIC_RG_BUCK_VPROC11_DVS_EN_CTRL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_dvs_en_once(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_DVS_EN_ONCE_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_DVS_EN_ONCE_MASK,
		PMIC_RG_BUCK_VPROC11_DVS_EN_ONCE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc11_dvs_en_once(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC11_DVS_EN_ONCE_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC11_DVS_EN_ONCE_MASK,
		PMIC_RG_BUCK_VPROC11_DVS_EN_ONCE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_dvs_down_td(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_DVS_DOWN_TD_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_DVS_DOWN_TD_MASK,
		PMIC_RG_BUCK_VPROC11_DVS_DOWN_TD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_dvs_down_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_DVS_DOWN_CTRL_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_DVS_DOWN_CTRL_MASK,
		PMIC_RG_BUCK_VPROC11_DVS_DOWN_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_dvs_down_once(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_DVS_DOWN_ONCE_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_DVS_DOWN_ONCE_MASK,
		PMIC_RG_BUCK_VPROC11_DVS_DOWN_ONCE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_SW_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_SW_OP_EN_MASK,
		PMIC_RG_BUCK_VPROC11_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc11_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC11_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC11_SW_OP_EN_MASK,
		PMIC_RG_BUCK_VPROC11_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_HW0_OP_EN_MASK,
		PMIC_RG_BUCK_VPROC11_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc11_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC11_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC11_HW0_OP_EN_MASK,
		PMIC_RG_BUCK_VPROC11_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_HW1_OP_EN_MASK,
		PMIC_RG_BUCK_VPROC11_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc11_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC11_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC11_HW1_OP_EN_MASK,
		PMIC_RG_BUCK_VPROC11_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_HW2_OP_EN_MASK,
		PMIC_RG_BUCK_VPROC11_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc11_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC11_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC11_HW2_OP_EN_MASK,
		PMIC_RG_BUCK_VPROC11_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_HW0_OP_CFG_MASK,
		PMIC_RG_BUCK_VPROC11_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc11_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC11_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC11_HW0_OP_CFG_MASK,
		PMIC_RG_BUCK_VPROC11_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_HW1_OP_CFG_MASK,
		PMIC_RG_BUCK_VPROC11_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc11_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC11_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC11_HW1_OP_CFG_MASK,
		PMIC_RG_BUCK_VPROC11_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_HW2_OP_CFG_MASK,
		PMIC_RG_BUCK_VPROC11_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc11_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC11_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC11_HW2_OP_CFG_MASK,
		PMIC_RG_BUCK_VPROC11_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_ON_OP_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_ON_OP_MASK,
		PMIC_RG_BUCK_VPROC11_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_LP_OP_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_LP_OP_MASK,
		PMIC_RG_BUCK_VPROC11_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_sp_sw_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_SP_SW_VOSEL_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_SP_SW_VOSEL_MASK,
		PMIC_RG_BUCK_VPROC11_SP_SW_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc11_sp_sw_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC11_SP_SW_VOSEL_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC11_SP_SW_VOSEL_MASK,
		PMIC_RG_BUCK_VPROC11_SP_SW_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_sp_sw_vosel_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_SP_SW_VOSEL_EN_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_SP_SW_VOSEL_EN_MASK,
		PMIC_RG_BUCK_VPROC11_SP_SW_VOSEL_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc11_sp_sw_vosel_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC11_SP_SW_VOSEL_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC11_SP_SW_VOSEL_EN_MASK,
		PMIC_RG_BUCK_VPROC11_SP_SW_VOSEL_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_sp_on_vosel_mux_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_SP_ON_VOSEL_MUX_SEL_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_SP_ON_VOSEL_MUX_SEL_MASK,
		PMIC_RG_BUCK_VPROC11_SP_ON_VOSEL_MUX_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc11_sp_on_vosel_mux_sel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC11_SP_ON_VOSEL_MUX_SEL_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC11_SP_ON_VOSEL_MUX_SEL_MASK,
		PMIC_RG_BUCK_VPROC11_SP_ON_VOSEL_MUX_SEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_oc_deg_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_OC_DEG_EN_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_OC_DEG_EN_MASK,
		PMIC_RG_BUCK_VPROC11_OC_DEG_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc11_oc_deg_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC11_OC_DEG_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC11_OC_DEG_EN_MASK,
		PMIC_RG_BUCK_VPROC11_OC_DEG_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_oc_wnd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_OC_WND_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_OC_WND_MASK,
		PMIC_RG_BUCK_VPROC11_OC_WND_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_oc_thd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_OC_THD_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_OC_THD_MASK,
		PMIC_RG_BUCK_VPROC11_OC_THD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_vproc11_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VPROC11_VOSEL_ADDR,
		&val,
		PMIC_DA_VPROC11_VOSEL_MASK,
		PMIC_DA_VPROC11_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vproc11_vosel_gray(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VPROC11_VOSEL_GRAY_ADDR,
		&val,
		PMIC_DA_VPROC11_VOSEL_GRAY_MASK,
		PMIC_DA_VPROC11_VOSEL_GRAY_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vproc11_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VPROC11_EN_ADDR,
		&val,
		PMIC_DA_VPROC11_EN_MASK,
		PMIC_DA_VPROC11_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vproc11_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VPROC11_STB_ADDR,
		&val,
		PMIC_DA_VPROC11_STB_MASK,
		PMIC_DA_VPROC11_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vproc11_vsleep_sel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VPROC11_VSLEEP_SEL_ADDR,
		&val,
		PMIC_DA_VPROC11_VSLEEP_SEL_MASK,
		PMIC_DA_VPROC11_VSLEEP_SEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vproc11_r2r_pdn(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VPROC11_R2R_PDN_ADDR,
		&val,
		PMIC_DA_VPROC11_R2R_PDN_MASK,
		PMIC_DA_VPROC11_R2R_PDN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vproc11_dvs_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VPROC11_DVS_EN_ADDR,
		&val,
		PMIC_DA_VPROC11_DVS_EN_MASK,
		PMIC_DA_VPROC11_DVS_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vproc11_dvs_down(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VPROC11_DVS_DOWN_ADDR,
		&val,
		PMIC_DA_VPROC11_DVS_DOWN_MASK,
		PMIC_DA_VPROC11_DVS_DOWN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vproc11_ssh(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VPROC11_SSH_ADDR,
		&val,
		PMIC_DA_VPROC11_SSH_MASK,
		PMIC_DA_VPROC11_SSH_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vproc11_minfreq_discharge(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VPROC11_MINFREQ_DISCHARGE_ADDR,
		&val,
		PMIC_DA_VPROC11_MINFREQ_DISCHARGE_MASK,
		PMIC_DA_VPROC11_MINFREQ_DISCHARGE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_oc_flag_clr_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_OC_FLAG_CLR_SEL_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_OC_FLAG_CLR_SEL_MASK,
		PMIC_RG_BUCK_VPROC11_OC_FLAG_CLR_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_OSC_SEL_DIS_MASK,
		PMIC_RG_BUCK_VPROC11_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_CK_SW_MODE_MASK,
		PMIC_RG_BUCK_VPROC11_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vproc11_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC11_CK_SW_EN_ADDR,
		val,
		PMIC_RG_BUCK_VPROC11_CK_SW_EN_MASK,
		PMIC_RG_BUCK_VPROC11_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc11_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC11_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC11_CK_SW_EN_MASK,
		PMIC_RG_BUCK_VPROC11_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_EN_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_EN_MASK,
		PMIC_RG_BUCK_VPROC12_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc12_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC12_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC12_EN_MASK,
		PMIC_RG_BUCK_VPROC12_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_LP_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_LP_MASK,
		PMIC_RG_BUCK_VPROC12_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_VOSEL_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_VOSEL_MASK,
		PMIC_RG_BUCK_VPROC12_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc12_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC12_VOSEL_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC12_VOSEL_MASK,
		PMIC_RG_BUCK_VPROC12_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_vosel_sleep(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_VOSEL_SLEEP_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_VOSEL_SLEEP_MASK,
		PMIC_RG_BUCK_VPROC12_VOSEL_SLEEP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc12_vosel_sleep(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC12_VOSEL_SLEEP_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC12_VOSEL_SLEEP_MASK,
		PMIC_RG_BUCK_VPROC12_VOSEL_SLEEP_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_sfchg_frate(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_SFCHG_FRATE_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_SFCHG_FRATE_MASK,
		PMIC_RG_BUCK_VPROC12_SFCHG_FRATE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_sfchg_fen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_SFCHG_FEN_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_SFCHG_FEN_MASK,
		PMIC_RG_BUCK_VPROC12_SFCHG_FEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_sfchg_rrate(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_SFCHG_RRATE_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_SFCHG_RRATE_MASK,
		PMIC_RG_BUCK_VPROC12_SFCHG_RRATE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_sfchg_ren(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_SFCHG_REN_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_SFCHG_REN_MASK,
		PMIC_RG_BUCK_VPROC12_SFCHG_REN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_dvs_en_td(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_DVS_EN_TD_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_DVS_EN_TD_MASK,
		PMIC_RG_BUCK_VPROC12_DVS_EN_TD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc12_dvs_en_td(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC12_DVS_EN_TD_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC12_DVS_EN_TD_MASK,
		PMIC_RG_BUCK_VPROC12_DVS_EN_TD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_dvs_en_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_DVS_EN_CTRL_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_DVS_EN_CTRL_MASK,
		PMIC_RG_BUCK_VPROC12_DVS_EN_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc12_dvs_en_ctrl(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC12_DVS_EN_CTRL_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC12_DVS_EN_CTRL_MASK,
		PMIC_RG_BUCK_VPROC12_DVS_EN_CTRL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_dvs_en_once(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_DVS_EN_ONCE_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_DVS_EN_ONCE_MASK,
		PMIC_RG_BUCK_VPROC12_DVS_EN_ONCE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc12_dvs_en_once(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC12_DVS_EN_ONCE_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC12_DVS_EN_ONCE_MASK,
		PMIC_RG_BUCK_VPROC12_DVS_EN_ONCE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_dvs_down_td(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_DVS_DOWN_TD_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_DVS_DOWN_TD_MASK,
		PMIC_RG_BUCK_VPROC12_DVS_DOWN_TD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_dvs_down_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_DVS_DOWN_CTRL_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_DVS_DOWN_CTRL_MASK,
		PMIC_RG_BUCK_VPROC12_DVS_DOWN_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_dvs_down_once(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_DVS_DOWN_ONCE_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_DVS_DOWN_ONCE_MASK,
		PMIC_RG_BUCK_VPROC12_DVS_DOWN_ONCE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_SW_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_SW_OP_EN_MASK,
		PMIC_RG_BUCK_VPROC12_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc12_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC12_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC12_SW_OP_EN_MASK,
		PMIC_RG_BUCK_VPROC12_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_HW0_OP_EN_MASK,
		PMIC_RG_BUCK_VPROC12_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc12_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC12_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC12_HW0_OP_EN_MASK,
		PMIC_RG_BUCK_VPROC12_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_HW1_OP_EN_MASK,
		PMIC_RG_BUCK_VPROC12_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc12_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC12_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC12_HW1_OP_EN_MASK,
		PMIC_RG_BUCK_VPROC12_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_HW2_OP_EN_MASK,
		PMIC_RG_BUCK_VPROC12_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc12_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC12_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC12_HW2_OP_EN_MASK,
		PMIC_RG_BUCK_VPROC12_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_HW0_OP_CFG_MASK,
		PMIC_RG_BUCK_VPROC12_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc12_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC12_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC12_HW0_OP_CFG_MASK,
		PMIC_RG_BUCK_VPROC12_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_HW1_OP_CFG_MASK,
		PMIC_RG_BUCK_VPROC12_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc12_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC12_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC12_HW1_OP_CFG_MASK,
		PMIC_RG_BUCK_VPROC12_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_HW2_OP_CFG_MASK,
		PMIC_RG_BUCK_VPROC12_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc12_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC12_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC12_HW2_OP_CFG_MASK,
		PMIC_RG_BUCK_VPROC12_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_ON_OP_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_ON_OP_MASK,
		PMIC_RG_BUCK_VPROC12_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_LP_OP_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_LP_OP_MASK,
		PMIC_RG_BUCK_VPROC12_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_sp_sw_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_SP_SW_VOSEL_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_SP_SW_VOSEL_MASK,
		PMIC_RG_BUCK_VPROC12_SP_SW_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc12_sp_sw_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC12_SP_SW_VOSEL_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC12_SP_SW_VOSEL_MASK,
		PMIC_RG_BUCK_VPROC12_SP_SW_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_sp_sw_vosel_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_SP_SW_VOSEL_EN_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_SP_SW_VOSEL_EN_MASK,
		PMIC_RG_BUCK_VPROC12_SP_SW_VOSEL_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc12_sp_sw_vosel_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC12_SP_SW_VOSEL_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC12_SP_SW_VOSEL_EN_MASK,
		PMIC_RG_BUCK_VPROC12_SP_SW_VOSEL_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_sp_on_vosel_mux_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_SP_ON_VOSEL_MUX_SEL_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_SP_ON_VOSEL_MUX_SEL_MASK,
		PMIC_RG_BUCK_VPROC12_SP_ON_VOSEL_MUX_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc12_sp_on_vosel_mux_sel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC12_SP_ON_VOSEL_MUX_SEL_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC12_SP_ON_VOSEL_MUX_SEL_MASK,
		PMIC_RG_BUCK_VPROC12_SP_ON_VOSEL_MUX_SEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_oc_deg_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_OC_DEG_EN_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_OC_DEG_EN_MASK,
		PMIC_RG_BUCK_VPROC12_OC_DEG_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc12_oc_deg_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC12_OC_DEG_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC12_OC_DEG_EN_MASK,
		PMIC_RG_BUCK_VPROC12_OC_DEG_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_oc_wnd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_OC_WND_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_OC_WND_MASK,
		PMIC_RG_BUCK_VPROC12_OC_WND_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_oc_thd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_OC_THD_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_OC_THD_MASK,
		PMIC_RG_BUCK_VPROC12_OC_THD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_vproc12_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VPROC12_VOSEL_ADDR,
		&val,
		PMIC_DA_VPROC12_VOSEL_MASK,
		PMIC_DA_VPROC12_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vproc12_vosel_gray(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VPROC12_VOSEL_GRAY_ADDR,
		&val,
		PMIC_DA_VPROC12_VOSEL_GRAY_MASK,
		PMIC_DA_VPROC12_VOSEL_GRAY_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vproc12_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VPROC12_EN_ADDR,
		&val,
		PMIC_DA_VPROC12_EN_MASK,
		PMIC_DA_VPROC12_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vproc12_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VPROC12_STB_ADDR,
		&val,
		PMIC_DA_VPROC12_STB_MASK,
		PMIC_DA_VPROC12_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vproc12_vsleep_sel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VPROC12_VSLEEP_SEL_ADDR,
		&val,
		PMIC_DA_VPROC12_VSLEEP_SEL_MASK,
		PMIC_DA_VPROC12_VSLEEP_SEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vproc12_r2r_pdn(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VPROC12_R2R_PDN_ADDR,
		&val,
		PMIC_DA_VPROC12_R2R_PDN_MASK,
		PMIC_DA_VPROC12_R2R_PDN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vproc12_dvs_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VPROC12_DVS_EN_ADDR,
		&val,
		PMIC_DA_VPROC12_DVS_EN_MASK,
		PMIC_DA_VPROC12_DVS_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vproc12_dvs_down(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VPROC12_DVS_DOWN_ADDR,
		&val,
		PMIC_DA_VPROC12_DVS_DOWN_MASK,
		PMIC_DA_VPROC12_DVS_DOWN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vproc12_ssh(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VPROC12_SSH_ADDR,
		&val,
		PMIC_DA_VPROC12_SSH_MASK,
		PMIC_DA_VPROC12_SSH_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vproc12_minfreq_discharge(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VPROC12_MINFREQ_DISCHARGE_ADDR,
		&val,
		PMIC_DA_VPROC12_MINFREQ_DISCHARGE_MASK,
		PMIC_DA_VPROC12_MINFREQ_DISCHARGE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_oc_flag_clr_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_OC_FLAG_CLR_SEL_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_OC_FLAG_CLR_SEL_MASK,
		PMIC_RG_BUCK_VPROC12_OC_FLAG_CLR_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_OSC_SEL_DIS_MASK,
		PMIC_RG_BUCK_VPROC12_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_CK_SW_MODE_MASK,
		PMIC_RG_BUCK_VPROC12_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vproc12_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPROC12_CK_SW_EN_ADDR,
		val,
		PMIC_RG_BUCK_VPROC12_CK_SW_EN_MASK,
		PMIC_RG_BUCK_VPROC12_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vproc12_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPROC12_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VPROC12_CK_SW_EN_MASK,
		PMIC_RG_BUCK_VPROC12_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_EN_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_EN_MASK,
		PMIC_RG_BUCK_VCORE_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vcore_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VCORE_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VCORE_EN_MASK,
		PMIC_RG_BUCK_VCORE_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_LP_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_LP_MASK,
		PMIC_RG_BUCK_VCORE_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_VOSEL_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_VOSEL_MASK,
		PMIC_RG_BUCK_VCORE_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vcore_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VCORE_VOSEL_ADDR,
		&val,
		PMIC_RG_BUCK_VCORE_VOSEL_MASK,
		PMIC_RG_BUCK_VCORE_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_vosel_sleep(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_VOSEL_SLEEP_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_VOSEL_SLEEP_MASK,
		PMIC_RG_BUCK_VCORE_VOSEL_SLEEP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vcore_vosel_sleep(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VCORE_VOSEL_SLEEP_ADDR,
		&val,
		PMIC_RG_BUCK_VCORE_VOSEL_SLEEP_MASK,
		PMIC_RG_BUCK_VCORE_VOSEL_SLEEP_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_sfchg_frate(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_SFCHG_FRATE_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_SFCHG_FRATE_MASK,
		PMIC_RG_BUCK_VCORE_SFCHG_FRATE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_sfchg_fen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_SFCHG_FEN_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_SFCHG_FEN_MASK,
		PMIC_RG_BUCK_VCORE_SFCHG_FEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_sfchg_rrate(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_SFCHG_RRATE_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_SFCHG_RRATE_MASK,
		PMIC_RG_BUCK_VCORE_SFCHG_RRATE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_sfchg_ren(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_SFCHG_REN_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_SFCHG_REN_MASK,
		PMIC_RG_BUCK_VCORE_SFCHG_REN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_dvs_en_td(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_DVS_EN_TD_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_DVS_EN_TD_MASK,
		PMIC_RG_BUCK_VCORE_DVS_EN_TD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vcore_dvs_en_td(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VCORE_DVS_EN_TD_ADDR,
		&val,
		PMIC_RG_BUCK_VCORE_DVS_EN_TD_MASK,
		PMIC_RG_BUCK_VCORE_DVS_EN_TD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_dvs_en_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_DVS_EN_CTRL_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_DVS_EN_CTRL_MASK,
		PMIC_RG_BUCK_VCORE_DVS_EN_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vcore_dvs_en_ctrl(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VCORE_DVS_EN_CTRL_ADDR,
		&val,
		PMIC_RG_BUCK_VCORE_DVS_EN_CTRL_MASK,
		PMIC_RG_BUCK_VCORE_DVS_EN_CTRL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_dvs_en_once(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_DVS_EN_ONCE_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_DVS_EN_ONCE_MASK,
		PMIC_RG_BUCK_VCORE_DVS_EN_ONCE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vcore_dvs_en_once(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VCORE_DVS_EN_ONCE_ADDR,
		&val,
		PMIC_RG_BUCK_VCORE_DVS_EN_ONCE_MASK,
		PMIC_RG_BUCK_VCORE_DVS_EN_ONCE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_dvs_down_td(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_DVS_DOWN_TD_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_DVS_DOWN_TD_MASK,
		PMIC_RG_BUCK_VCORE_DVS_DOWN_TD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_dvs_down_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_DVS_DOWN_CTRL_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_DVS_DOWN_CTRL_MASK,
		PMIC_RG_BUCK_VCORE_DVS_DOWN_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_dvs_down_once(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_DVS_DOWN_ONCE_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_DVS_DOWN_ONCE_MASK,
		PMIC_RG_BUCK_VCORE_DVS_DOWN_ONCE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_SW_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_SW_OP_EN_MASK,
		PMIC_RG_BUCK_VCORE_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vcore_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VCORE_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VCORE_SW_OP_EN_MASK,
		PMIC_RG_BUCK_VCORE_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_HW0_OP_EN_MASK,
		PMIC_RG_BUCK_VCORE_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vcore_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VCORE_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VCORE_HW0_OP_EN_MASK,
		PMIC_RG_BUCK_VCORE_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_HW1_OP_EN_MASK,
		PMIC_RG_BUCK_VCORE_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vcore_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VCORE_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VCORE_HW1_OP_EN_MASK,
		PMIC_RG_BUCK_VCORE_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_HW2_OP_EN_MASK,
		PMIC_RG_BUCK_VCORE_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vcore_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VCORE_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VCORE_HW2_OP_EN_MASK,
		PMIC_RG_BUCK_VCORE_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_HW0_OP_CFG_MASK,
		PMIC_RG_BUCK_VCORE_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vcore_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VCORE_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_BUCK_VCORE_HW0_OP_CFG_MASK,
		PMIC_RG_BUCK_VCORE_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_HW1_OP_CFG_MASK,
		PMIC_RG_BUCK_VCORE_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vcore_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VCORE_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_BUCK_VCORE_HW1_OP_CFG_MASK,
		PMIC_RG_BUCK_VCORE_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_HW2_OP_CFG_MASK,
		PMIC_RG_BUCK_VCORE_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vcore_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VCORE_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_BUCK_VCORE_HW2_OP_CFG_MASK,
		PMIC_RG_BUCK_VCORE_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_ON_OP_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_ON_OP_MASK,
		PMIC_RG_BUCK_VCORE_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_LP_OP_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_LP_OP_MASK,
		PMIC_RG_BUCK_VCORE_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_sp_sw_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_SP_SW_VOSEL_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_SP_SW_VOSEL_MASK,
		PMIC_RG_BUCK_VCORE_SP_SW_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vcore_sp_sw_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VCORE_SP_SW_VOSEL_ADDR,
		&val,
		PMIC_RG_BUCK_VCORE_SP_SW_VOSEL_MASK,
		PMIC_RG_BUCK_VCORE_SP_SW_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_sp_sw_vosel_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_SP_SW_VOSEL_EN_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_SP_SW_VOSEL_EN_MASK,
		PMIC_RG_BUCK_VCORE_SP_SW_VOSEL_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vcore_sp_sw_vosel_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VCORE_SP_SW_VOSEL_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VCORE_SP_SW_VOSEL_EN_MASK,
		PMIC_RG_BUCK_VCORE_SP_SW_VOSEL_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_sp_on_vosel_mux_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_SP_ON_VOSEL_MUX_SEL_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_SP_ON_VOSEL_MUX_SEL_MASK,
		PMIC_RG_BUCK_VCORE_SP_ON_VOSEL_MUX_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vcore_sp_on_vosel_mux_sel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VCORE_SP_ON_VOSEL_MUX_SEL_ADDR,
		&val,
		PMIC_RG_BUCK_VCORE_SP_ON_VOSEL_MUX_SEL_MASK,
		PMIC_RG_BUCK_VCORE_SP_ON_VOSEL_MUX_SEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_oc_deg_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_OC_DEG_EN_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_OC_DEG_EN_MASK,
		PMIC_RG_BUCK_VCORE_OC_DEG_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vcore_oc_deg_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VCORE_OC_DEG_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VCORE_OC_DEG_EN_MASK,
		PMIC_RG_BUCK_VCORE_OC_DEG_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_oc_wnd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_OC_WND_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_OC_WND_MASK,
		PMIC_RG_BUCK_VCORE_OC_WND_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_oc_thd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_OC_THD_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_OC_THD_MASK,
		PMIC_RG_BUCK_VCORE_OC_THD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_vcore_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VCORE_VOSEL_ADDR,
		&val,
		PMIC_DA_VCORE_VOSEL_MASK,
		PMIC_DA_VCORE_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vcore_vosel_gray(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VCORE_VOSEL_GRAY_ADDR,
		&val,
		PMIC_DA_VCORE_VOSEL_GRAY_MASK,
		PMIC_DA_VCORE_VOSEL_GRAY_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vcore_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VCORE_EN_ADDR,
		&val,
		PMIC_DA_VCORE_EN_MASK,
		PMIC_DA_VCORE_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vcore_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VCORE_STB_ADDR,
		&val,
		PMIC_DA_VCORE_STB_MASK,
		PMIC_DA_VCORE_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vcore_vsleep_sel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VCORE_VSLEEP_SEL_ADDR,
		&val,
		PMIC_DA_VCORE_VSLEEP_SEL_MASK,
		PMIC_DA_VCORE_VSLEEP_SEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vcore_r2r_pdn(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VCORE_R2R_PDN_ADDR,
		&val,
		PMIC_DA_VCORE_R2R_PDN_MASK,
		PMIC_DA_VCORE_R2R_PDN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vcore_dvs_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VCORE_DVS_EN_ADDR,
		&val,
		PMIC_DA_VCORE_DVS_EN_MASK,
		PMIC_DA_VCORE_DVS_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vcore_dvs_down(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VCORE_DVS_DOWN_ADDR,
		&val,
		PMIC_DA_VCORE_DVS_DOWN_MASK,
		PMIC_DA_VCORE_DVS_DOWN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vcore_ssh(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VCORE_SSH_ADDR,
		&val,
		PMIC_DA_VCORE_SSH_MASK,
		PMIC_DA_VCORE_SSH_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vcore_minfreq_discharge(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VCORE_MINFREQ_DISCHARGE_ADDR,
		&val,
		PMIC_DA_VCORE_MINFREQ_DISCHARGE_MASK,
		PMIC_DA_VCORE_MINFREQ_DISCHARGE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_oc_flag_clr_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_OC_FLAG_CLR_SEL_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_OC_FLAG_CLR_SEL_MASK,
		PMIC_RG_BUCK_VCORE_OC_FLAG_CLR_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_OSC_SEL_DIS_MASK,
		PMIC_RG_BUCK_VCORE_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_CK_SW_MODE_MASK,
		PMIC_RG_BUCK_VCORE_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_CK_SW_EN_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_CK_SW_EN_MASK,
		PMIC_RG_BUCK_VCORE_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vcore_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VCORE_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VCORE_CK_SW_EN_MASK,
		PMIC_RG_BUCK_VCORE_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_sshub_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_SSHUB_MODE_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_SSHUB_MODE_MASK,
		PMIC_RG_BUCK_VCORE_SSHUB_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_sshub_on(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_SSHUB_ON_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_SSHUB_ON_MASK,
		PMIC_RG_BUCK_VCORE_SSHUB_ON_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vcore_sshub_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VCORE_SSHUB_VOSEL_ADDR,
		val,
		PMIC_RG_BUCK_VCORE_SSHUB_VOSEL_MASK,
		PMIC_RG_BUCK_VCORE_SSHUB_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vcore_sshub_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VCORE_SSHUB_VOSEL_ADDR,
		&val,
		PMIC_RG_BUCK_VCORE_SSHUB_VOSEL_MASK,
		PMIC_RG_BUCK_VCORE_SSHUB_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_EN_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_EN_MASK,
		PMIC_RG_BUCK_VGPU_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vgpu_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VGPU_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VGPU_EN_MASK,
		PMIC_RG_BUCK_VGPU_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_LP_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_LP_MASK,
		PMIC_RG_BUCK_VGPU_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_VOSEL_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_VOSEL_MASK,
		PMIC_RG_BUCK_VGPU_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vgpu_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VGPU_VOSEL_ADDR,
		&val,
		PMIC_RG_BUCK_VGPU_VOSEL_MASK,
		PMIC_RG_BUCK_VGPU_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_vosel_sleep(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_VOSEL_SLEEP_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_VOSEL_SLEEP_MASK,
		PMIC_RG_BUCK_VGPU_VOSEL_SLEEP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vgpu_vosel_sleep(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VGPU_VOSEL_SLEEP_ADDR,
		&val,
		PMIC_RG_BUCK_VGPU_VOSEL_SLEEP_MASK,
		PMIC_RG_BUCK_VGPU_VOSEL_SLEEP_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_sfchg_frate(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_SFCHG_FRATE_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_SFCHG_FRATE_MASK,
		PMIC_RG_BUCK_VGPU_SFCHG_FRATE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_sfchg_fen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_SFCHG_FEN_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_SFCHG_FEN_MASK,
		PMIC_RG_BUCK_VGPU_SFCHG_FEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_sfchg_rrate(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_SFCHG_RRATE_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_SFCHG_RRATE_MASK,
		PMIC_RG_BUCK_VGPU_SFCHG_RRATE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_sfchg_ren(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_SFCHG_REN_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_SFCHG_REN_MASK,
		PMIC_RG_BUCK_VGPU_SFCHG_REN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_dvs_en_td(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_DVS_EN_TD_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_DVS_EN_TD_MASK,
		PMIC_RG_BUCK_VGPU_DVS_EN_TD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vgpu_dvs_en_td(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VGPU_DVS_EN_TD_ADDR,
		&val,
		PMIC_RG_BUCK_VGPU_DVS_EN_TD_MASK,
		PMIC_RG_BUCK_VGPU_DVS_EN_TD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_dvs_en_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_DVS_EN_CTRL_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_DVS_EN_CTRL_MASK,
		PMIC_RG_BUCK_VGPU_DVS_EN_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vgpu_dvs_en_ctrl(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VGPU_DVS_EN_CTRL_ADDR,
		&val,
		PMIC_RG_BUCK_VGPU_DVS_EN_CTRL_MASK,
		PMIC_RG_BUCK_VGPU_DVS_EN_CTRL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_dvs_en_once(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_DVS_EN_ONCE_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_DVS_EN_ONCE_MASK,
		PMIC_RG_BUCK_VGPU_DVS_EN_ONCE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vgpu_dvs_en_once(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VGPU_DVS_EN_ONCE_ADDR,
		&val,
		PMIC_RG_BUCK_VGPU_DVS_EN_ONCE_MASK,
		PMIC_RG_BUCK_VGPU_DVS_EN_ONCE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_dvs_down_td(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_DVS_DOWN_TD_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_DVS_DOWN_TD_MASK,
		PMIC_RG_BUCK_VGPU_DVS_DOWN_TD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_dvs_down_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_DVS_DOWN_CTRL_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_DVS_DOWN_CTRL_MASK,
		PMIC_RG_BUCK_VGPU_DVS_DOWN_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_dvs_down_once(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_DVS_DOWN_ONCE_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_DVS_DOWN_ONCE_MASK,
		PMIC_RG_BUCK_VGPU_DVS_DOWN_ONCE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_SW_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_SW_OP_EN_MASK,
		PMIC_RG_BUCK_VGPU_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vgpu_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VGPU_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VGPU_SW_OP_EN_MASK,
		PMIC_RG_BUCK_VGPU_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_HW0_OP_EN_MASK,
		PMIC_RG_BUCK_VGPU_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vgpu_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VGPU_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VGPU_HW0_OP_EN_MASK,
		PMIC_RG_BUCK_VGPU_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_HW1_OP_EN_MASK,
		PMIC_RG_BUCK_VGPU_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vgpu_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VGPU_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VGPU_HW1_OP_EN_MASK,
		PMIC_RG_BUCK_VGPU_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_HW2_OP_EN_MASK,
		PMIC_RG_BUCK_VGPU_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vgpu_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VGPU_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VGPU_HW2_OP_EN_MASK,
		PMIC_RG_BUCK_VGPU_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_HW0_OP_CFG_MASK,
		PMIC_RG_BUCK_VGPU_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vgpu_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VGPU_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_BUCK_VGPU_HW0_OP_CFG_MASK,
		PMIC_RG_BUCK_VGPU_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_HW1_OP_CFG_MASK,
		PMIC_RG_BUCK_VGPU_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vgpu_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VGPU_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_BUCK_VGPU_HW1_OP_CFG_MASK,
		PMIC_RG_BUCK_VGPU_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_HW2_OP_CFG_MASK,
		PMIC_RG_BUCK_VGPU_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vgpu_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VGPU_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_BUCK_VGPU_HW2_OP_CFG_MASK,
		PMIC_RG_BUCK_VGPU_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_ON_OP_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_ON_OP_MASK,
		PMIC_RG_BUCK_VGPU_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_LP_OP_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_LP_OP_MASK,
		PMIC_RG_BUCK_VGPU_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_sp_sw_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_SP_SW_VOSEL_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_SP_SW_VOSEL_MASK,
		PMIC_RG_BUCK_VGPU_SP_SW_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vgpu_sp_sw_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VGPU_SP_SW_VOSEL_ADDR,
		&val,
		PMIC_RG_BUCK_VGPU_SP_SW_VOSEL_MASK,
		PMIC_RG_BUCK_VGPU_SP_SW_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_sp_sw_vosel_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_SP_SW_VOSEL_EN_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_SP_SW_VOSEL_EN_MASK,
		PMIC_RG_BUCK_VGPU_SP_SW_VOSEL_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vgpu_sp_sw_vosel_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VGPU_SP_SW_VOSEL_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VGPU_SP_SW_VOSEL_EN_MASK,
		PMIC_RG_BUCK_VGPU_SP_SW_VOSEL_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_sp_on_vosel_mux_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_SP_ON_VOSEL_MUX_SEL_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_SP_ON_VOSEL_MUX_SEL_MASK,
		PMIC_RG_BUCK_VGPU_SP_ON_VOSEL_MUX_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vgpu_sp_on_vosel_mux_sel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VGPU_SP_ON_VOSEL_MUX_SEL_ADDR,
		&val,
		PMIC_RG_BUCK_VGPU_SP_ON_VOSEL_MUX_SEL_MASK,
		PMIC_RG_BUCK_VGPU_SP_ON_VOSEL_MUX_SEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_oc_deg_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_OC_DEG_EN_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_OC_DEG_EN_MASK,
		PMIC_RG_BUCK_VGPU_OC_DEG_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vgpu_oc_deg_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VGPU_OC_DEG_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VGPU_OC_DEG_EN_MASK,
		PMIC_RG_BUCK_VGPU_OC_DEG_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_oc_wnd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_OC_WND_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_OC_WND_MASK,
		PMIC_RG_BUCK_VGPU_OC_WND_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_oc_thd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_OC_THD_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_OC_THD_MASK,
		PMIC_RG_BUCK_VGPU_OC_THD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_vgpu_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VGPU_VOSEL_ADDR,
		&val,
		PMIC_DA_VGPU_VOSEL_MASK,
		PMIC_DA_VGPU_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vgpu_vosel_gray(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VGPU_VOSEL_GRAY_ADDR,
		&val,
		PMIC_DA_VGPU_VOSEL_GRAY_MASK,
		PMIC_DA_VGPU_VOSEL_GRAY_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vgpu_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VGPU_EN_ADDR,
		&val,
		PMIC_DA_VGPU_EN_MASK,
		PMIC_DA_VGPU_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vgpu_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VGPU_STB_ADDR,
		&val,
		PMIC_DA_VGPU_STB_MASK,
		PMIC_DA_VGPU_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vgpu_vsleep_sel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VGPU_VSLEEP_SEL_ADDR,
		&val,
		PMIC_DA_VGPU_VSLEEP_SEL_MASK,
		PMIC_DA_VGPU_VSLEEP_SEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vgpu_r2r_pdn(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VGPU_R2R_PDN_ADDR,
		&val,
		PMIC_DA_VGPU_R2R_PDN_MASK,
		PMIC_DA_VGPU_R2R_PDN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vgpu_dvs_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VGPU_DVS_EN_ADDR,
		&val,
		PMIC_DA_VGPU_DVS_EN_MASK,
		PMIC_DA_VGPU_DVS_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vgpu_dvs_down(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VGPU_DVS_DOWN_ADDR,
		&val,
		PMIC_DA_VGPU_DVS_DOWN_MASK,
		PMIC_DA_VGPU_DVS_DOWN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vgpu_ssh(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VGPU_SSH_ADDR,
		&val,
		PMIC_DA_VGPU_SSH_MASK,
		PMIC_DA_VGPU_SSH_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vgpu_minfreq_discharge(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VGPU_MINFREQ_DISCHARGE_ADDR,
		&val,
		PMIC_DA_VGPU_MINFREQ_DISCHARGE_MASK,
		PMIC_DA_VGPU_MINFREQ_DISCHARGE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_oc_flag_clr_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_OC_FLAG_CLR_SEL_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_OC_FLAG_CLR_SEL_MASK,
		PMIC_RG_BUCK_VGPU_OC_FLAG_CLR_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_OSC_SEL_DIS_MASK,
		PMIC_RG_BUCK_VGPU_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_CK_SW_MODE_MASK,
		PMIC_RG_BUCK_VGPU_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vgpu_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VGPU_CK_SW_EN_ADDR,
		val,
		PMIC_RG_BUCK_VGPU_CK_SW_EN_MASK,
		PMIC_RG_BUCK_VGPU_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vgpu_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VGPU_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VGPU_CK_SW_EN_MASK,
		PMIC_RG_BUCK_VGPU_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_EN_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_EN_MASK,
		PMIC_RG_BUCK_VDRAM1_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram1_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM1_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM1_EN_MASK,
		PMIC_RG_BUCK_VDRAM1_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_LP_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_LP_MASK,
		PMIC_RG_BUCK_VDRAM1_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_VOSEL_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_VOSEL_MASK,
		PMIC_RG_BUCK_VDRAM1_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram1_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM1_VOSEL_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM1_VOSEL_MASK,
		PMIC_RG_BUCK_VDRAM1_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_vosel_sleep(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_VOSEL_SLEEP_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_VOSEL_SLEEP_MASK,
		PMIC_RG_BUCK_VDRAM1_VOSEL_SLEEP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram1_vosel_sleep(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM1_VOSEL_SLEEP_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM1_VOSEL_SLEEP_MASK,
		PMIC_RG_BUCK_VDRAM1_VOSEL_SLEEP_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_sfchg_frate(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_SFCHG_FRATE_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_SFCHG_FRATE_MASK,
		PMIC_RG_BUCK_VDRAM1_SFCHG_FRATE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_sfchg_fen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_SFCHG_FEN_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_SFCHG_FEN_MASK,
		PMIC_RG_BUCK_VDRAM1_SFCHG_FEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_sfchg_rrate(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_SFCHG_RRATE_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_SFCHG_RRATE_MASK,
		PMIC_RG_BUCK_VDRAM1_SFCHG_RRATE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_sfchg_ren(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_SFCHG_REN_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_SFCHG_REN_MASK,
		PMIC_RG_BUCK_VDRAM1_SFCHG_REN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_dvs_en_td(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_DVS_EN_TD_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_DVS_EN_TD_MASK,
		PMIC_RG_BUCK_VDRAM1_DVS_EN_TD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram1_dvs_en_td(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM1_DVS_EN_TD_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM1_DVS_EN_TD_MASK,
		PMIC_RG_BUCK_VDRAM1_DVS_EN_TD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_dvs_en_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_DVS_EN_CTRL_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_DVS_EN_CTRL_MASK,
		PMIC_RG_BUCK_VDRAM1_DVS_EN_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram1_dvs_en_ctrl(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM1_DVS_EN_CTRL_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM1_DVS_EN_CTRL_MASK,
		PMIC_RG_BUCK_VDRAM1_DVS_EN_CTRL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_dvs_en_once(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_DVS_EN_ONCE_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_DVS_EN_ONCE_MASK,
		PMIC_RG_BUCK_VDRAM1_DVS_EN_ONCE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram1_dvs_en_once(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM1_DVS_EN_ONCE_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM1_DVS_EN_ONCE_MASK,
		PMIC_RG_BUCK_VDRAM1_DVS_EN_ONCE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_dvs_down_td(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_DVS_DOWN_TD_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_DVS_DOWN_TD_MASK,
		PMIC_RG_BUCK_VDRAM1_DVS_DOWN_TD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_dvs_down_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_DVS_DOWN_CTRL_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_DVS_DOWN_CTRL_MASK,
		PMIC_RG_BUCK_VDRAM1_DVS_DOWN_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_dvs_down_once(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_DVS_DOWN_ONCE_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_DVS_DOWN_ONCE_MASK,
		PMIC_RG_BUCK_VDRAM1_DVS_DOWN_ONCE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_SW_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_SW_OP_EN_MASK,
		PMIC_RG_BUCK_VDRAM1_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram1_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM1_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM1_SW_OP_EN_MASK,
		PMIC_RG_BUCK_VDRAM1_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_HW0_OP_EN_MASK,
		PMIC_RG_BUCK_VDRAM1_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram1_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM1_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM1_HW0_OP_EN_MASK,
		PMIC_RG_BUCK_VDRAM1_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_HW1_OP_EN_MASK,
		PMIC_RG_BUCK_VDRAM1_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram1_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM1_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM1_HW1_OP_EN_MASK,
		PMIC_RG_BUCK_VDRAM1_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_HW2_OP_EN_MASK,
		PMIC_RG_BUCK_VDRAM1_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram1_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM1_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM1_HW2_OP_EN_MASK,
		PMIC_RG_BUCK_VDRAM1_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_HW0_OP_CFG_MASK,
		PMIC_RG_BUCK_VDRAM1_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram1_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM1_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM1_HW0_OP_CFG_MASK,
		PMIC_RG_BUCK_VDRAM1_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_HW1_OP_CFG_MASK,
		PMIC_RG_BUCK_VDRAM1_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram1_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM1_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM1_HW1_OP_CFG_MASK,
		PMIC_RG_BUCK_VDRAM1_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_HW2_OP_CFG_MASK,
		PMIC_RG_BUCK_VDRAM1_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram1_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM1_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM1_HW2_OP_CFG_MASK,
		PMIC_RG_BUCK_VDRAM1_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_ON_OP_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_ON_OP_MASK,
		PMIC_RG_BUCK_VDRAM1_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_LP_OP_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_LP_OP_MASK,
		PMIC_RG_BUCK_VDRAM1_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_sp_sw_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_SP_SW_VOSEL_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_SP_SW_VOSEL_MASK,
		PMIC_RG_BUCK_VDRAM1_SP_SW_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram1_sp_sw_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM1_SP_SW_VOSEL_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM1_SP_SW_VOSEL_MASK,
		PMIC_RG_BUCK_VDRAM1_SP_SW_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_sp_sw_vosel_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_SP_SW_VOSEL_EN_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_SP_SW_VOSEL_EN_MASK,
		PMIC_RG_BUCK_VDRAM1_SP_SW_VOSEL_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram1_sp_sw_vosel_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM1_SP_SW_VOSEL_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM1_SP_SW_VOSEL_EN_MASK,
		PMIC_RG_BUCK_VDRAM1_SP_SW_VOSEL_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_sp_on_vosel_mux_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_SP_ON_VOSEL_MUX_SEL_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_SP_ON_VOSEL_MUX_SEL_MASK,
		PMIC_RG_BUCK_VDRAM1_SP_ON_VOSEL_MUX_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram1_sp_on_vosel_mux_sel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM1_SP_ON_VOSEL_MUX_SEL_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM1_SP_ON_VOSEL_MUX_SEL_MASK,
		PMIC_RG_BUCK_VDRAM1_SP_ON_VOSEL_MUX_SEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_oc_deg_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_OC_DEG_EN_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_OC_DEG_EN_MASK,
		PMIC_RG_BUCK_VDRAM1_OC_DEG_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram1_oc_deg_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM1_OC_DEG_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM1_OC_DEG_EN_MASK,
		PMIC_RG_BUCK_VDRAM1_OC_DEG_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_oc_wnd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_OC_WND_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_OC_WND_MASK,
		PMIC_RG_BUCK_VDRAM1_OC_WND_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_oc_thd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_OC_THD_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_OC_THD_MASK,
		PMIC_RG_BUCK_VDRAM1_OC_THD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_vdram1_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VDRAM1_VOSEL_ADDR,
		&val,
		PMIC_DA_VDRAM1_VOSEL_MASK,
		PMIC_DA_VDRAM1_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vdram1_vosel_gray(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VDRAM1_VOSEL_GRAY_ADDR,
		&val,
		PMIC_DA_VDRAM1_VOSEL_GRAY_MASK,
		PMIC_DA_VDRAM1_VOSEL_GRAY_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vdram1_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VDRAM1_EN_ADDR,
		&val,
		PMIC_DA_VDRAM1_EN_MASK,
		PMIC_DA_VDRAM1_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vdram1_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VDRAM1_STB_ADDR,
		&val,
		PMIC_DA_VDRAM1_STB_MASK,
		PMIC_DA_VDRAM1_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vdram1_vsleep_sel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VDRAM1_VSLEEP_SEL_ADDR,
		&val,
		PMIC_DA_VDRAM1_VSLEEP_SEL_MASK,
		PMIC_DA_VDRAM1_VSLEEP_SEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vdram1_r2r_pdn(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VDRAM1_R2R_PDN_ADDR,
		&val,
		PMIC_DA_VDRAM1_R2R_PDN_MASK,
		PMIC_DA_VDRAM1_R2R_PDN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vdram1_dvs_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VDRAM1_DVS_EN_ADDR,
		&val,
		PMIC_DA_VDRAM1_DVS_EN_MASK,
		PMIC_DA_VDRAM1_DVS_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vdram1_dvs_down(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VDRAM1_DVS_DOWN_ADDR,
		&val,
		PMIC_DA_VDRAM1_DVS_DOWN_MASK,
		PMIC_DA_VDRAM1_DVS_DOWN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vdram1_ssh(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VDRAM1_SSH_ADDR,
		&val,
		PMIC_DA_VDRAM1_SSH_MASK,
		PMIC_DA_VDRAM1_SSH_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vdram1_minfreq_discharge(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VDRAM1_MINFREQ_DISCHARGE_ADDR,
		&val,
		PMIC_DA_VDRAM1_MINFREQ_DISCHARGE_MASK,
		PMIC_DA_VDRAM1_MINFREQ_DISCHARGE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_oc_flag_clr_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_OC_FLAG_CLR_SEL_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_OC_FLAG_CLR_SEL_MASK,
		PMIC_RG_BUCK_VDRAM1_OC_FLAG_CLR_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_OSC_SEL_DIS_MASK,
		PMIC_RG_BUCK_VDRAM1_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_CK_SW_MODE_MASK,
		PMIC_RG_BUCK_VDRAM1_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vdram1_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM1_CK_SW_EN_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM1_CK_SW_EN_MASK,
		PMIC_RG_BUCK_VDRAM1_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram1_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM1_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM1_CK_SW_EN_MASK,
		PMIC_RG_BUCK_VDRAM1_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_EN_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_EN_MASK,
		PMIC_RG_BUCK_VDRAM2_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram2_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM2_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM2_EN_MASK,
		PMIC_RG_BUCK_VDRAM2_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_LP_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_LP_MASK,
		PMIC_RG_BUCK_VDRAM2_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_VOSEL_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_VOSEL_MASK,
		PMIC_RG_BUCK_VDRAM2_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram2_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM2_VOSEL_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM2_VOSEL_MASK,
		PMIC_RG_BUCK_VDRAM2_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_vosel_sleep(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_VOSEL_SLEEP_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_VOSEL_SLEEP_MASK,
		PMIC_RG_BUCK_VDRAM2_VOSEL_SLEEP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram2_vosel_sleep(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM2_VOSEL_SLEEP_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM2_VOSEL_SLEEP_MASK,
		PMIC_RG_BUCK_VDRAM2_VOSEL_SLEEP_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_sfchg_frate(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_SFCHG_FRATE_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_SFCHG_FRATE_MASK,
		PMIC_RG_BUCK_VDRAM2_SFCHG_FRATE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_sfchg_fen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_SFCHG_FEN_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_SFCHG_FEN_MASK,
		PMIC_RG_BUCK_VDRAM2_SFCHG_FEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_sfchg_rrate(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_SFCHG_RRATE_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_SFCHG_RRATE_MASK,
		PMIC_RG_BUCK_VDRAM2_SFCHG_RRATE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_sfchg_ren(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_SFCHG_REN_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_SFCHG_REN_MASK,
		PMIC_RG_BUCK_VDRAM2_SFCHG_REN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_dvs_en_td(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_DVS_EN_TD_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_DVS_EN_TD_MASK,
		PMIC_RG_BUCK_VDRAM2_DVS_EN_TD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram2_dvs_en_td(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM2_DVS_EN_TD_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM2_DVS_EN_TD_MASK,
		PMIC_RG_BUCK_VDRAM2_DVS_EN_TD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_dvs_en_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_DVS_EN_CTRL_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_DVS_EN_CTRL_MASK,
		PMIC_RG_BUCK_VDRAM2_DVS_EN_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram2_dvs_en_ctrl(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM2_DVS_EN_CTRL_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM2_DVS_EN_CTRL_MASK,
		PMIC_RG_BUCK_VDRAM2_DVS_EN_CTRL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_dvs_en_once(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_DVS_EN_ONCE_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_DVS_EN_ONCE_MASK,
		PMIC_RG_BUCK_VDRAM2_DVS_EN_ONCE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram2_dvs_en_once(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM2_DVS_EN_ONCE_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM2_DVS_EN_ONCE_MASK,
		PMIC_RG_BUCK_VDRAM2_DVS_EN_ONCE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_dvs_down_td(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_DVS_DOWN_TD_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_DVS_DOWN_TD_MASK,
		PMIC_RG_BUCK_VDRAM2_DVS_DOWN_TD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_dvs_down_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_DVS_DOWN_CTRL_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_DVS_DOWN_CTRL_MASK,
		PMIC_RG_BUCK_VDRAM2_DVS_DOWN_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_dvs_down_once(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_DVS_DOWN_ONCE_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_DVS_DOWN_ONCE_MASK,
		PMIC_RG_BUCK_VDRAM2_DVS_DOWN_ONCE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_SW_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_SW_OP_EN_MASK,
		PMIC_RG_BUCK_VDRAM2_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram2_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM2_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM2_SW_OP_EN_MASK,
		PMIC_RG_BUCK_VDRAM2_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_HW0_OP_EN_MASK,
		PMIC_RG_BUCK_VDRAM2_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram2_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM2_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM2_HW0_OP_EN_MASK,
		PMIC_RG_BUCK_VDRAM2_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_HW1_OP_EN_MASK,
		PMIC_RG_BUCK_VDRAM2_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram2_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM2_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM2_HW1_OP_EN_MASK,
		PMIC_RG_BUCK_VDRAM2_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_HW2_OP_EN_MASK,
		PMIC_RG_BUCK_VDRAM2_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram2_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM2_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM2_HW2_OP_EN_MASK,
		PMIC_RG_BUCK_VDRAM2_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_HW0_OP_CFG_MASK,
		PMIC_RG_BUCK_VDRAM2_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram2_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM2_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM2_HW0_OP_CFG_MASK,
		PMIC_RG_BUCK_VDRAM2_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_HW1_OP_CFG_MASK,
		PMIC_RG_BUCK_VDRAM2_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram2_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM2_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM2_HW1_OP_CFG_MASK,
		PMIC_RG_BUCK_VDRAM2_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_HW2_OP_CFG_MASK,
		PMIC_RG_BUCK_VDRAM2_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram2_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM2_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM2_HW2_OP_CFG_MASK,
		PMIC_RG_BUCK_VDRAM2_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_ON_OP_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_ON_OP_MASK,
		PMIC_RG_BUCK_VDRAM2_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_LP_OP_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_LP_OP_MASK,
		PMIC_RG_BUCK_VDRAM2_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_sp_sw_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_SP_SW_VOSEL_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_SP_SW_VOSEL_MASK,
		PMIC_RG_BUCK_VDRAM2_SP_SW_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram2_sp_sw_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM2_SP_SW_VOSEL_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM2_SP_SW_VOSEL_MASK,
		PMIC_RG_BUCK_VDRAM2_SP_SW_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_sp_sw_vosel_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_SP_SW_VOSEL_EN_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_SP_SW_VOSEL_EN_MASK,
		PMIC_RG_BUCK_VDRAM2_SP_SW_VOSEL_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram2_sp_sw_vosel_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM2_SP_SW_VOSEL_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM2_SP_SW_VOSEL_EN_MASK,
		PMIC_RG_BUCK_VDRAM2_SP_SW_VOSEL_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_sp_on_vosel_mux_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_SP_ON_VOSEL_MUX_SEL_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_SP_ON_VOSEL_MUX_SEL_MASK,
		PMIC_RG_BUCK_VDRAM2_SP_ON_VOSEL_MUX_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram2_sp_on_vosel_mux_sel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM2_SP_ON_VOSEL_MUX_SEL_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM2_SP_ON_VOSEL_MUX_SEL_MASK,
		PMIC_RG_BUCK_VDRAM2_SP_ON_VOSEL_MUX_SEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_oc_deg_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_OC_DEG_EN_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_OC_DEG_EN_MASK,
		PMIC_RG_BUCK_VDRAM2_OC_DEG_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram2_oc_deg_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM2_OC_DEG_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM2_OC_DEG_EN_MASK,
		PMIC_RG_BUCK_VDRAM2_OC_DEG_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_oc_wnd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_OC_WND_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_OC_WND_MASK,
		PMIC_RG_BUCK_VDRAM2_OC_WND_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_oc_thd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_OC_THD_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_OC_THD_MASK,
		PMIC_RG_BUCK_VDRAM2_OC_THD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_vdram2_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VDRAM2_VOSEL_ADDR,
		&val,
		PMIC_DA_VDRAM2_VOSEL_MASK,
		PMIC_DA_VDRAM2_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vdram2_vosel_gray(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VDRAM2_VOSEL_GRAY_ADDR,
		&val,
		PMIC_DA_VDRAM2_VOSEL_GRAY_MASK,
		PMIC_DA_VDRAM2_VOSEL_GRAY_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vdram2_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VDRAM2_EN_ADDR,
		&val,
		PMIC_DA_VDRAM2_EN_MASK,
		PMIC_DA_VDRAM2_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vdram2_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VDRAM2_STB_ADDR,
		&val,
		PMIC_DA_VDRAM2_STB_MASK,
		PMIC_DA_VDRAM2_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vdram2_vsleep_sel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VDRAM2_VSLEEP_SEL_ADDR,
		&val,
		PMIC_DA_VDRAM2_VSLEEP_SEL_MASK,
		PMIC_DA_VDRAM2_VSLEEP_SEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vdram2_r2r_pdn(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VDRAM2_R2R_PDN_ADDR,
		&val,
		PMIC_DA_VDRAM2_R2R_PDN_MASK,
		PMIC_DA_VDRAM2_R2R_PDN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vdram2_dvs_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VDRAM2_DVS_EN_ADDR,
		&val,
		PMIC_DA_VDRAM2_DVS_EN_MASK,
		PMIC_DA_VDRAM2_DVS_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vdram2_dvs_down(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VDRAM2_DVS_DOWN_ADDR,
		&val,
		PMIC_DA_VDRAM2_DVS_DOWN_MASK,
		PMIC_DA_VDRAM2_DVS_DOWN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vdram2_ssh(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VDRAM2_SSH_ADDR,
		&val,
		PMIC_DA_VDRAM2_SSH_MASK,
		PMIC_DA_VDRAM2_SSH_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vdram2_minfreq_discharge(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VDRAM2_MINFREQ_DISCHARGE_ADDR,
		&val,
		PMIC_DA_VDRAM2_MINFREQ_DISCHARGE_MASK,
		PMIC_DA_VDRAM2_MINFREQ_DISCHARGE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_oc_flag_clr_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_OC_FLAG_CLR_SEL_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_OC_FLAG_CLR_SEL_MASK,
		PMIC_RG_BUCK_VDRAM2_OC_FLAG_CLR_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_OSC_SEL_DIS_MASK,
		PMIC_RG_BUCK_VDRAM2_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_CK_SW_MODE_MASK,
		PMIC_RG_BUCK_VDRAM2_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vdram2_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VDRAM2_CK_SW_EN_ADDR,
		val,
		PMIC_RG_BUCK_VDRAM2_CK_SW_EN_MASK,
		PMIC_RG_BUCK_VDRAM2_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vdram2_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VDRAM2_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VDRAM2_CK_SW_EN_MASK,
		PMIC_RG_BUCK_VDRAM2_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_EN_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_EN_MASK,
		PMIC_RG_BUCK_VMODEM_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vmodem_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VMODEM_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VMODEM_EN_MASK,
		PMIC_RG_BUCK_VMODEM_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_LP_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_LP_MASK,
		PMIC_RG_BUCK_VMODEM_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_VOSEL_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_VOSEL_MASK,
		PMIC_RG_BUCK_VMODEM_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vmodem_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VMODEM_VOSEL_ADDR,
		&val,
		PMIC_RG_BUCK_VMODEM_VOSEL_MASK,
		PMIC_RG_BUCK_VMODEM_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_vosel_sleep(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_VOSEL_SLEEP_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_VOSEL_SLEEP_MASK,
		PMIC_RG_BUCK_VMODEM_VOSEL_SLEEP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vmodem_vosel_sleep(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VMODEM_VOSEL_SLEEP_ADDR,
		&val,
		PMIC_RG_BUCK_VMODEM_VOSEL_SLEEP_MASK,
		PMIC_RG_BUCK_VMODEM_VOSEL_SLEEP_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_sfchg_frate(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_SFCHG_FRATE_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_SFCHG_FRATE_MASK,
		PMIC_RG_BUCK_VMODEM_SFCHG_FRATE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_sfchg_fen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_SFCHG_FEN_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_SFCHG_FEN_MASK,
		PMIC_RG_BUCK_VMODEM_SFCHG_FEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_sfchg_rrate(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_SFCHG_RRATE_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_SFCHG_RRATE_MASK,
		PMIC_RG_BUCK_VMODEM_SFCHG_RRATE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_sfchg_ren(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_SFCHG_REN_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_SFCHG_REN_MASK,
		PMIC_RG_BUCK_VMODEM_SFCHG_REN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_dvs_en_td(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_DVS_EN_TD_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_DVS_EN_TD_MASK,
		PMIC_RG_BUCK_VMODEM_DVS_EN_TD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vmodem_dvs_en_td(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VMODEM_DVS_EN_TD_ADDR,
		&val,
		PMIC_RG_BUCK_VMODEM_DVS_EN_TD_MASK,
		PMIC_RG_BUCK_VMODEM_DVS_EN_TD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_dvs_en_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_DVS_EN_CTRL_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_DVS_EN_CTRL_MASK,
		PMIC_RG_BUCK_VMODEM_DVS_EN_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vmodem_dvs_en_ctrl(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VMODEM_DVS_EN_CTRL_ADDR,
		&val,
		PMIC_RG_BUCK_VMODEM_DVS_EN_CTRL_MASK,
		PMIC_RG_BUCK_VMODEM_DVS_EN_CTRL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_dvs_en_once(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_DVS_EN_ONCE_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_DVS_EN_ONCE_MASK,
		PMIC_RG_BUCK_VMODEM_DVS_EN_ONCE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vmodem_dvs_en_once(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VMODEM_DVS_EN_ONCE_ADDR,
		&val,
		PMIC_RG_BUCK_VMODEM_DVS_EN_ONCE_MASK,
		PMIC_RG_BUCK_VMODEM_DVS_EN_ONCE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_dvs_down_td(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_DVS_DOWN_TD_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_DVS_DOWN_TD_MASK,
		PMIC_RG_BUCK_VMODEM_DVS_DOWN_TD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_dvs_down_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_DVS_DOWN_CTRL_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_DVS_DOWN_CTRL_MASK,
		PMIC_RG_BUCK_VMODEM_DVS_DOWN_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_dvs_down_once(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_DVS_DOWN_ONCE_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_DVS_DOWN_ONCE_MASK,
		PMIC_RG_BUCK_VMODEM_DVS_DOWN_ONCE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_SW_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_SW_OP_EN_MASK,
		PMIC_RG_BUCK_VMODEM_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vmodem_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VMODEM_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VMODEM_SW_OP_EN_MASK,
		PMIC_RG_BUCK_VMODEM_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_HW0_OP_EN_MASK,
		PMIC_RG_BUCK_VMODEM_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vmodem_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VMODEM_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VMODEM_HW0_OP_EN_MASK,
		PMIC_RG_BUCK_VMODEM_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_HW1_OP_EN_MASK,
		PMIC_RG_BUCK_VMODEM_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vmodem_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VMODEM_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VMODEM_HW1_OP_EN_MASK,
		PMIC_RG_BUCK_VMODEM_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_HW2_OP_EN_MASK,
		PMIC_RG_BUCK_VMODEM_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vmodem_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VMODEM_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VMODEM_HW2_OP_EN_MASK,
		PMIC_RG_BUCK_VMODEM_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_HW0_OP_CFG_MASK,
		PMIC_RG_BUCK_VMODEM_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vmodem_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VMODEM_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_BUCK_VMODEM_HW0_OP_CFG_MASK,
		PMIC_RG_BUCK_VMODEM_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_HW1_OP_CFG_MASK,
		PMIC_RG_BUCK_VMODEM_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vmodem_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VMODEM_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_BUCK_VMODEM_HW1_OP_CFG_MASK,
		PMIC_RG_BUCK_VMODEM_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_HW2_OP_CFG_MASK,
		PMIC_RG_BUCK_VMODEM_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vmodem_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VMODEM_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_BUCK_VMODEM_HW2_OP_CFG_MASK,
		PMIC_RG_BUCK_VMODEM_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_ON_OP_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_ON_OP_MASK,
		PMIC_RG_BUCK_VMODEM_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_LP_OP_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_LP_OP_MASK,
		PMIC_RG_BUCK_VMODEM_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_sp_sw_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_SP_SW_VOSEL_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_SP_SW_VOSEL_MASK,
		PMIC_RG_BUCK_VMODEM_SP_SW_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vmodem_sp_sw_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VMODEM_SP_SW_VOSEL_ADDR,
		&val,
		PMIC_RG_BUCK_VMODEM_SP_SW_VOSEL_MASK,
		PMIC_RG_BUCK_VMODEM_SP_SW_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_sp_sw_vosel_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_SP_SW_VOSEL_EN_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_SP_SW_VOSEL_EN_MASK,
		PMIC_RG_BUCK_VMODEM_SP_SW_VOSEL_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vmodem_sp_sw_vosel_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VMODEM_SP_SW_VOSEL_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VMODEM_SP_SW_VOSEL_EN_MASK,
		PMIC_RG_BUCK_VMODEM_SP_SW_VOSEL_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_sp_on_vosel_mux_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_SP_ON_VOSEL_MUX_SEL_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_SP_ON_VOSEL_MUX_SEL_MASK,
		PMIC_RG_BUCK_VMODEM_SP_ON_VOSEL_MUX_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vmodem_sp_on_vosel_mux_sel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VMODEM_SP_ON_VOSEL_MUX_SEL_ADDR,
		&val,
		PMIC_RG_BUCK_VMODEM_SP_ON_VOSEL_MUX_SEL_MASK,
		PMIC_RG_BUCK_VMODEM_SP_ON_VOSEL_MUX_SEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_oc_deg_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_OC_DEG_EN_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_OC_DEG_EN_MASK,
		PMIC_RG_BUCK_VMODEM_OC_DEG_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vmodem_oc_deg_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VMODEM_OC_DEG_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VMODEM_OC_DEG_EN_MASK,
		PMIC_RG_BUCK_VMODEM_OC_DEG_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_oc_wnd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_OC_WND_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_OC_WND_MASK,
		PMIC_RG_BUCK_VMODEM_OC_WND_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_oc_thd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_OC_THD_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_OC_THD_MASK,
		PMIC_RG_BUCK_VMODEM_OC_THD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_vmodem_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VMODEM_VOSEL_ADDR,
		&val,
		PMIC_DA_VMODEM_VOSEL_MASK,
		PMIC_DA_VMODEM_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vmodem_vosel_gray(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VMODEM_VOSEL_GRAY_ADDR,
		&val,
		PMIC_DA_VMODEM_VOSEL_GRAY_MASK,
		PMIC_DA_VMODEM_VOSEL_GRAY_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vmodem_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VMODEM_EN_ADDR,
		&val,
		PMIC_DA_VMODEM_EN_MASK,
		PMIC_DA_VMODEM_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vmodem_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VMODEM_STB_ADDR,
		&val,
		PMIC_DA_VMODEM_STB_MASK,
		PMIC_DA_VMODEM_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vmodem_vsleep_sel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VMODEM_VSLEEP_SEL_ADDR,
		&val,
		PMIC_DA_VMODEM_VSLEEP_SEL_MASK,
		PMIC_DA_VMODEM_VSLEEP_SEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vmodem_r2r_pdn(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VMODEM_R2R_PDN_ADDR,
		&val,
		PMIC_DA_VMODEM_R2R_PDN_MASK,
		PMIC_DA_VMODEM_R2R_PDN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vmodem_dvs_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VMODEM_DVS_EN_ADDR,
		&val,
		PMIC_DA_VMODEM_DVS_EN_MASK,
		PMIC_DA_VMODEM_DVS_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vmodem_minfreq_discharge(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VMODEM_MINFREQ_DISCHARGE_ADDR,
		&val,
		PMIC_DA_VMODEM_MINFREQ_DISCHARGE_MASK,
		PMIC_DA_VMODEM_MINFREQ_DISCHARGE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_oc_flag_clr_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_OC_FLAG_CLR_SEL_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_OC_FLAG_CLR_SEL_MASK,
		PMIC_RG_BUCK_VMODEM_OC_FLAG_CLR_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_OSC_SEL_DIS_MASK,
		PMIC_RG_BUCK_VMODEM_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_CK_SW_MODE_MASK,
		PMIC_RG_BUCK_VMODEM_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_CK_SW_EN_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_CK_SW_EN_MASK,
		PMIC_RG_BUCK_VMODEM_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vmodem_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VMODEM_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VMODEM_CK_SW_EN_MASK,
		PMIC_RG_BUCK_VMODEM_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_vosel_dlc0(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_VOSEL_DLC0_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_VOSEL_DLC0_MASK,
		PMIC_RG_BUCK_VMODEM_VOSEL_DLC0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vmodem_vosel_dlc0(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VMODEM_VOSEL_DLC0_ADDR,
		&val,
		PMIC_RG_BUCK_VMODEM_VOSEL_DLC0_MASK,
		PMIC_RG_BUCK_VMODEM_VOSEL_DLC0_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_vosel_dlc1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_VOSEL_DLC1_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_VOSEL_DLC1_MASK,
		PMIC_RG_BUCK_VMODEM_VOSEL_DLC1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vmodem_vosel_dlc1(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VMODEM_VOSEL_DLC1_ADDR,
		&val,
		PMIC_RG_BUCK_VMODEM_VOSEL_DLC1_MASK,
		PMIC_RG_BUCK_VMODEM_VOSEL_DLC1_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_vosel_dlc2(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_VOSEL_DLC2_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_VOSEL_DLC2_MASK,
		PMIC_RG_BUCK_VMODEM_VOSEL_DLC2_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vmodem_vosel_dlc2(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VMODEM_VOSEL_DLC2_ADDR,
		&val,
		PMIC_RG_BUCK_VMODEM_VOSEL_DLC2_MASK,
		PMIC_RG_BUCK_VMODEM_VOSEL_DLC2_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_dlc0(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_DLC0_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_DLC0_MASK,
		PMIC_RG_BUCK_VMODEM_DLC0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_dlc1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_DLC1_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_DLC1_MASK,
		PMIC_RG_BUCK_VMODEM_DLC1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_dlc2(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_DLC2_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_DLC2_MASK,
		PMIC_RG_BUCK_VMODEM_DLC2_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_dlc3(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_DLC3_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_DLC3_MASK,
		PMIC_RG_BUCK_VMODEM_DLC3_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_dlc_map_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_DLC_MAP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_DLC_MAP_EN_MASK,
		PMIC_RG_BUCK_VMODEM_DLC_MAP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vmodem_dlc_map_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VMODEM_DLC_MAP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VMODEM_DLC_MAP_EN_MASK,
		PMIC_RG_BUCK_VMODEM_DLC_MAP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vmodem_dlc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VMODEM_DLC_ADDR,
		val,
		PMIC_RG_BUCK_VMODEM_DLC_MASK,
		PMIC_RG_BUCK_VMODEM_DLC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_vmodem_dlc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VMODEM_DLC_ADDR,
		&val,
		PMIC_DA_VMODEM_DLC_MASK,
		PMIC_DA_VMODEM_DLC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_EN_ADDR,
		val,
		PMIC_RG_BUCK_VS1_EN_MASK,
		PMIC_RG_BUCK_VS1_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs1_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS1_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VS1_EN_MASK,
		PMIC_RG_BUCK_VS1_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_LP_ADDR,
		val,
		PMIC_RG_BUCK_VS1_LP_MASK,
		PMIC_RG_BUCK_VS1_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_VOSEL_ADDR,
		val,
		PMIC_RG_BUCK_VS1_VOSEL_MASK,
		PMIC_RG_BUCK_VS1_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs1_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS1_VOSEL_ADDR,
		&val,
		PMIC_RG_BUCK_VS1_VOSEL_MASK,
		PMIC_RG_BUCK_VS1_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_vosel_sleep(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_VOSEL_SLEEP_ADDR,
		val,
		PMIC_RG_BUCK_VS1_VOSEL_SLEEP_MASK,
		PMIC_RG_BUCK_VS1_VOSEL_SLEEP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs1_vosel_sleep(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS1_VOSEL_SLEEP_ADDR,
		&val,
		PMIC_RG_BUCK_VS1_VOSEL_SLEEP_MASK,
		PMIC_RG_BUCK_VS1_VOSEL_SLEEP_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_sfchg_frate(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_SFCHG_FRATE_ADDR,
		val,
		PMIC_RG_BUCK_VS1_SFCHG_FRATE_MASK,
		PMIC_RG_BUCK_VS1_SFCHG_FRATE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_sfchg_fen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_SFCHG_FEN_ADDR,
		val,
		PMIC_RG_BUCK_VS1_SFCHG_FEN_MASK,
		PMIC_RG_BUCK_VS1_SFCHG_FEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_sfchg_rrate(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_SFCHG_RRATE_ADDR,
		val,
		PMIC_RG_BUCK_VS1_SFCHG_RRATE_MASK,
		PMIC_RG_BUCK_VS1_SFCHG_RRATE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_sfchg_ren(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_SFCHG_REN_ADDR,
		val,
		PMIC_RG_BUCK_VS1_SFCHG_REN_MASK,
		PMIC_RG_BUCK_VS1_SFCHG_REN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_dvs_en_td(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_DVS_EN_TD_ADDR,
		val,
		PMIC_RG_BUCK_VS1_DVS_EN_TD_MASK,
		PMIC_RG_BUCK_VS1_DVS_EN_TD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs1_dvs_en_td(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS1_DVS_EN_TD_ADDR,
		&val,
		PMIC_RG_BUCK_VS1_DVS_EN_TD_MASK,
		PMIC_RG_BUCK_VS1_DVS_EN_TD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_dvs_en_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_DVS_EN_CTRL_ADDR,
		val,
		PMIC_RG_BUCK_VS1_DVS_EN_CTRL_MASK,
		PMIC_RG_BUCK_VS1_DVS_EN_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs1_dvs_en_ctrl(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS1_DVS_EN_CTRL_ADDR,
		&val,
		PMIC_RG_BUCK_VS1_DVS_EN_CTRL_MASK,
		PMIC_RG_BUCK_VS1_DVS_EN_CTRL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_dvs_en_once(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_DVS_EN_ONCE_ADDR,
		val,
		PMIC_RG_BUCK_VS1_DVS_EN_ONCE_MASK,
		PMIC_RG_BUCK_VS1_DVS_EN_ONCE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs1_dvs_en_once(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS1_DVS_EN_ONCE_ADDR,
		&val,
		PMIC_RG_BUCK_VS1_DVS_EN_ONCE_MASK,
		PMIC_RG_BUCK_VS1_DVS_EN_ONCE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_dvs_down_td(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_DVS_DOWN_TD_ADDR,
		val,
		PMIC_RG_BUCK_VS1_DVS_DOWN_TD_MASK,
		PMIC_RG_BUCK_VS1_DVS_DOWN_TD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_dvs_down_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_DVS_DOWN_CTRL_ADDR,
		val,
		PMIC_RG_BUCK_VS1_DVS_DOWN_CTRL_MASK,
		PMIC_RG_BUCK_VS1_DVS_DOWN_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_dvs_down_once(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_DVS_DOWN_ONCE_ADDR,
		val,
		PMIC_RG_BUCK_VS1_DVS_DOWN_ONCE_MASK,
		PMIC_RG_BUCK_VS1_DVS_DOWN_ONCE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_SW_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VS1_SW_OP_EN_MASK,
		PMIC_RG_BUCK_VS1_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs1_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS1_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VS1_SW_OP_EN_MASK,
		PMIC_RG_BUCK_VS1_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VS1_HW0_OP_EN_MASK,
		PMIC_RG_BUCK_VS1_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs1_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS1_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VS1_HW0_OP_EN_MASK,
		PMIC_RG_BUCK_VS1_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VS1_HW1_OP_EN_MASK,
		PMIC_RG_BUCK_VS1_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs1_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS1_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VS1_HW1_OP_EN_MASK,
		PMIC_RG_BUCK_VS1_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VS1_HW2_OP_EN_MASK,
		PMIC_RG_BUCK_VS1_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs1_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS1_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VS1_HW2_OP_EN_MASK,
		PMIC_RG_BUCK_VS1_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_BUCK_VS1_HW0_OP_CFG_MASK,
		PMIC_RG_BUCK_VS1_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs1_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS1_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_BUCK_VS1_HW0_OP_CFG_MASK,
		PMIC_RG_BUCK_VS1_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_BUCK_VS1_HW1_OP_CFG_MASK,
		PMIC_RG_BUCK_VS1_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs1_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS1_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_BUCK_VS1_HW1_OP_CFG_MASK,
		PMIC_RG_BUCK_VS1_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_BUCK_VS1_HW2_OP_CFG_MASK,
		PMIC_RG_BUCK_VS1_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs1_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS1_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_BUCK_VS1_HW2_OP_CFG_MASK,
		PMIC_RG_BUCK_VS1_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_ON_OP_ADDR,
		val,
		PMIC_RG_BUCK_VS1_ON_OP_MASK,
		PMIC_RG_BUCK_VS1_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_LP_OP_ADDR,
		val,
		PMIC_RG_BUCK_VS1_LP_OP_MASK,
		PMIC_RG_BUCK_VS1_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_sp_sw_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_SP_SW_VOSEL_ADDR,
		val,
		PMIC_RG_BUCK_VS1_SP_SW_VOSEL_MASK,
		PMIC_RG_BUCK_VS1_SP_SW_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs1_sp_sw_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS1_SP_SW_VOSEL_ADDR,
		&val,
		PMIC_RG_BUCK_VS1_SP_SW_VOSEL_MASK,
		PMIC_RG_BUCK_VS1_SP_SW_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_sp_sw_vosel_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_SP_SW_VOSEL_EN_ADDR,
		val,
		PMIC_RG_BUCK_VS1_SP_SW_VOSEL_EN_MASK,
		PMIC_RG_BUCK_VS1_SP_SW_VOSEL_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs1_sp_sw_vosel_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS1_SP_SW_VOSEL_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VS1_SP_SW_VOSEL_EN_MASK,
		PMIC_RG_BUCK_VS1_SP_SW_VOSEL_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_sp_on_vosel_mux_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_SP_ON_VOSEL_MUX_SEL_ADDR,
		val,
		PMIC_RG_BUCK_VS1_SP_ON_VOSEL_MUX_SEL_MASK,
		PMIC_RG_BUCK_VS1_SP_ON_VOSEL_MUX_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs1_sp_on_vosel_mux_sel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS1_SP_ON_VOSEL_MUX_SEL_ADDR,
		&val,
		PMIC_RG_BUCK_VS1_SP_ON_VOSEL_MUX_SEL_MASK,
		PMIC_RG_BUCK_VS1_SP_ON_VOSEL_MUX_SEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_oc_deg_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_OC_DEG_EN_ADDR,
		val,
		PMIC_RG_BUCK_VS1_OC_DEG_EN_MASK,
		PMIC_RG_BUCK_VS1_OC_DEG_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs1_oc_deg_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS1_OC_DEG_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VS1_OC_DEG_EN_MASK,
		PMIC_RG_BUCK_VS1_OC_DEG_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_oc_wnd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_OC_WND_ADDR,
		val,
		PMIC_RG_BUCK_VS1_OC_WND_MASK,
		PMIC_RG_BUCK_VS1_OC_WND_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_oc_thd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_OC_THD_ADDR,
		val,
		PMIC_RG_BUCK_VS1_OC_THD_MASK,
		PMIC_RG_BUCK_VS1_OC_THD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_vs1_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VS1_VOSEL_ADDR,
		&val,
		PMIC_DA_VS1_VOSEL_MASK,
		PMIC_DA_VS1_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vs1_vosel_gray(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VS1_VOSEL_GRAY_ADDR,
		&val,
		PMIC_DA_VS1_VOSEL_GRAY_MASK,
		PMIC_DA_VS1_VOSEL_GRAY_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vs1_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VS1_EN_ADDR,
		&val,
		PMIC_DA_VS1_EN_MASK,
		PMIC_DA_VS1_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vs1_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VS1_STB_ADDR,
		&val,
		PMIC_DA_VS1_STB_MASK,
		PMIC_DA_VS1_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vs1_vsleep_sel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VS1_VSLEEP_SEL_ADDR,
		&val,
		PMIC_DA_VS1_VSLEEP_SEL_MASK,
		PMIC_DA_VS1_VSLEEP_SEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vs1_r2r_pdn(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VS1_R2R_PDN_ADDR,
		&val,
		PMIC_DA_VS1_R2R_PDN_MASK,
		PMIC_DA_VS1_R2R_PDN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vs1_dvs_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VS1_DVS_EN_ADDR,
		&val,
		PMIC_DA_VS1_DVS_EN_MASK,
		PMIC_DA_VS1_DVS_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vs1_minfreq_discharge(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VS1_MINFREQ_DISCHARGE_ADDR,
		&val,
		PMIC_DA_VS1_MINFREQ_DISCHARGE_MASK,
		PMIC_DA_VS1_MINFREQ_DISCHARGE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_oc_flag_clr_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_OC_FLAG_CLR_SEL_ADDR,
		val,
		PMIC_RG_BUCK_VS1_OC_FLAG_CLR_SEL_MASK,
		PMIC_RG_BUCK_VS1_OC_FLAG_CLR_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_BUCK_VS1_OSC_SEL_DIS_MASK,
		PMIC_RG_BUCK_VS1_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_BUCK_VS1_CK_SW_MODE_MASK,
		PMIC_RG_BUCK_VS1_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_CK_SW_EN_ADDR,
		val,
		PMIC_RG_BUCK_VS1_CK_SW_EN_MASK,
		PMIC_RG_BUCK_VS1_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs1_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS1_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VS1_CK_SW_EN_MASK,
		PMIC_RG_BUCK_VS1_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_voter_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_VOTER_EN_ADDR,
		val,
		PMIC_RG_BUCK_VS1_VOTER_EN_MASK,
		PMIC_RG_BUCK_VS1_VOTER_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs1_voter_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS1_VOTER_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VS1_VOTER_EN_MASK,
		PMIC_RG_BUCK_VS1_VOTER_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs1_voter_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS1_VOTER_VOSEL_ADDR,
		val,
		PMIC_RG_BUCK_VS1_VOTER_VOSEL_MASK,
		PMIC_RG_BUCK_VS1_VOTER_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs1_voter_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS1_VOTER_VOSEL_ADDR,
		&val,
		PMIC_RG_BUCK_VS1_VOTER_VOSEL_MASK,
		PMIC_RG_BUCK_VS1_VOTER_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_EN_ADDR,
		val,
		PMIC_RG_BUCK_VS2_EN_MASK,
		PMIC_RG_BUCK_VS2_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs2_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS2_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VS2_EN_MASK,
		PMIC_RG_BUCK_VS2_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_LP_ADDR,
		val,
		PMIC_RG_BUCK_VS2_LP_MASK,
		PMIC_RG_BUCK_VS2_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_VOSEL_ADDR,
		val,
		PMIC_RG_BUCK_VS2_VOSEL_MASK,
		PMIC_RG_BUCK_VS2_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs2_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS2_VOSEL_ADDR,
		&val,
		PMIC_RG_BUCK_VS2_VOSEL_MASK,
		PMIC_RG_BUCK_VS2_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_vosel_sleep(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_VOSEL_SLEEP_ADDR,
		val,
		PMIC_RG_BUCK_VS2_VOSEL_SLEEP_MASK,
		PMIC_RG_BUCK_VS2_VOSEL_SLEEP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs2_vosel_sleep(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS2_VOSEL_SLEEP_ADDR,
		&val,
		PMIC_RG_BUCK_VS2_VOSEL_SLEEP_MASK,
		PMIC_RG_BUCK_VS2_VOSEL_SLEEP_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_sfchg_frate(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_SFCHG_FRATE_ADDR,
		val,
		PMIC_RG_BUCK_VS2_SFCHG_FRATE_MASK,
		PMIC_RG_BUCK_VS2_SFCHG_FRATE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_sfchg_fen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_SFCHG_FEN_ADDR,
		val,
		PMIC_RG_BUCK_VS2_SFCHG_FEN_MASK,
		PMIC_RG_BUCK_VS2_SFCHG_FEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_sfchg_rrate(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_SFCHG_RRATE_ADDR,
		val,
		PMIC_RG_BUCK_VS2_SFCHG_RRATE_MASK,
		PMIC_RG_BUCK_VS2_SFCHG_RRATE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_sfchg_ren(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_SFCHG_REN_ADDR,
		val,
		PMIC_RG_BUCK_VS2_SFCHG_REN_MASK,
		PMIC_RG_BUCK_VS2_SFCHG_REN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_dvs_en_td(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_DVS_EN_TD_ADDR,
		val,
		PMIC_RG_BUCK_VS2_DVS_EN_TD_MASK,
		PMIC_RG_BUCK_VS2_DVS_EN_TD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs2_dvs_en_td(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS2_DVS_EN_TD_ADDR,
		&val,
		PMIC_RG_BUCK_VS2_DVS_EN_TD_MASK,
		PMIC_RG_BUCK_VS2_DVS_EN_TD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_dvs_en_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_DVS_EN_CTRL_ADDR,
		val,
		PMIC_RG_BUCK_VS2_DVS_EN_CTRL_MASK,
		PMIC_RG_BUCK_VS2_DVS_EN_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs2_dvs_en_ctrl(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS2_DVS_EN_CTRL_ADDR,
		&val,
		PMIC_RG_BUCK_VS2_DVS_EN_CTRL_MASK,
		PMIC_RG_BUCK_VS2_DVS_EN_CTRL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_dvs_en_once(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_DVS_EN_ONCE_ADDR,
		val,
		PMIC_RG_BUCK_VS2_DVS_EN_ONCE_MASK,
		PMIC_RG_BUCK_VS2_DVS_EN_ONCE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs2_dvs_en_once(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS2_DVS_EN_ONCE_ADDR,
		&val,
		PMIC_RG_BUCK_VS2_DVS_EN_ONCE_MASK,
		PMIC_RG_BUCK_VS2_DVS_EN_ONCE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_dvs_down_td(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_DVS_DOWN_TD_ADDR,
		val,
		PMIC_RG_BUCK_VS2_DVS_DOWN_TD_MASK,
		PMIC_RG_BUCK_VS2_DVS_DOWN_TD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_dvs_down_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_DVS_DOWN_CTRL_ADDR,
		val,
		PMIC_RG_BUCK_VS2_DVS_DOWN_CTRL_MASK,
		PMIC_RG_BUCK_VS2_DVS_DOWN_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_dvs_down_once(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_DVS_DOWN_ONCE_ADDR,
		val,
		PMIC_RG_BUCK_VS2_DVS_DOWN_ONCE_MASK,
		PMIC_RG_BUCK_VS2_DVS_DOWN_ONCE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_SW_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VS2_SW_OP_EN_MASK,
		PMIC_RG_BUCK_VS2_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs2_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS2_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VS2_SW_OP_EN_MASK,
		PMIC_RG_BUCK_VS2_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VS2_HW0_OP_EN_MASK,
		PMIC_RG_BUCK_VS2_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs2_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS2_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VS2_HW0_OP_EN_MASK,
		PMIC_RG_BUCK_VS2_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VS2_HW1_OP_EN_MASK,
		PMIC_RG_BUCK_VS2_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs2_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS2_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VS2_HW1_OP_EN_MASK,
		PMIC_RG_BUCK_VS2_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VS2_HW2_OP_EN_MASK,
		PMIC_RG_BUCK_VS2_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs2_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS2_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VS2_HW2_OP_EN_MASK,
		PMIC_RG_BUCK_VS2_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_BUCK_VS2_HW0_OP_CFG_MASK,
		PMIC_RG_BUCK_VS2_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs2_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS2_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_BUCK_VS2_HW0_OP_CFG_MASK,
		PMIC_RG_BUCK_VS2_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_BUCK_VS2_HW1_OP_CFG_MASK,
		PMIC_RG_BUCK_VS2_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs2_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS2_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_BUCK_VS2_HW1_OP_CFG_MASK,
		PMIC_RG_BUCK_VS2_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_BUCK_VS2_HW2_OP_CFG_MASK,
		PMIC_RG_BUCK_VS2_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs2_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS2_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_BUCK_VS2_HW2_OP_CFG_MASK,
		PMIC_RG_BUCK_VS2_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_ON_OP_ADDR,
		val,
		PMIC_RG_BUCK_VS2_ON_OP_MASK,
		PMIC_RG_BUCK_VS2_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_LP_OP_ADDR,
		val,
		PMIC_RG_BUCK_VS2_LP_OP_MASK,
		PMIC_RG_BUCK_VS2_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_sp_sw_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_SP_SW_VOSEL_ADDR,
		val,
		PMIC_RG_BUCK_VS2_SP_SW_VOSEL_MASK,
		PMIC_RG_BUCK_VS2_SP_SW_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs2_sp_sw_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS2_SP_SW_VOSEL_ADDR,
		&val,
		PMIC_RG_BUCK_VS2_SP_SW_VOSEL_MASK,
		PMIC_RG_BUCK_VS2_SP_SW_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_sp_sw_vosel_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_SP_SW_VOSEL_EN_ADDR,
		val,
		PMIC_RG_BUCK_VS2_SP_SW_VOSEL_EN_MASK,
		PMIC_RG_BUCK_VS2_SP_SW_VOSEL_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs2_sp_sw_vosel_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS2_SP_SW_VOSEL_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VS2_SP_SW_VOSEL_EN_MASK,
		PMIC_RG_BUCK_VS2_SP_SW_VOSEL_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_sp_on_vosel_mux_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_SP_ON_VOSEL_MUX_SEL_ADDR,
		val,
		PMIC_RG_BUCK_VS2_SP_ON_VOSEL_MUX_SEL_MASK,
		PMIC_RG_BUCK_VS2_SP_ON_VOSEL_MUX_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs2_sp_on_vosel_mux_sel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS2_SP_ON_VOSEL_MUX_SEL_ADDR,
		&val,
		PMIC_RG_BUCK_VS2_SP_ON_VOSEL_MUX_SEL_MASK,
		PMIC_RG_BUCK_VS2_SP_ON_VOSEL_MUX_SEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_oc_deg_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_OC_DEG_EN_ADDR,
		val,
		PMIC_RG_BUCK_VS2_OC_DEG_EN_MASK,
		PMIC_RG_BUCK_VS2_OC_DEG_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs2_oc_deg_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS2_OC_DEG_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VS2_OC_DEG_EN_MASK,
		PMIC_RG_BUCK_VS2_OC_DEG_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_oc_wnd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_OC_WND_ADDR,
		val,
		PMIC_RG_BUCK_VS2_OC_WND_MASK,
		PMIC_RG_BUCK_VS2_OC_WND_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_oc_thd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_OC_THD_ADDR,
		val,
		PMIC_RG_BUCK_VS2_OC_THD_MASK,
		PMIC_RG_BUCK_VS2_OC_THD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_vs2_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VS2_VOSEL_ADDR,
		&val,
		PMIC_DA_VS2_VOSEL_MASK,
		PMIC_DA_VS2_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vs2_vosel_gray(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VS2_VOSEL_GRAY_ADDR,
		&val,
		PMIC_DA_VS2_VOSEL_GRAY_MASK,
		PMIC_DA_VS2_VOSEL_GRAY_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vs2_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VS2_EN_ADDR,
		&val,
		PMIC_DA_VS2_EN_MASK,
		PMIC_DA_VS2_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vs2_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VS2_STB_ADDR,
		&val,
		PMIC_DA_VS2_STB_MASK,
		PMIC_DA_VS2_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vs2_vsleep_sel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VS2_VSLEEP_SEL_ADDR,
		&val,
		PMIC_DA_VS2_VSLEEP_SEL_MASK,
		PMIC_DA_VS2_VSLEEP_SEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vs2_r2r_pdn(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VS2_R2R_PDN_ADDR,
		&val,
		PMIC_DA_VS2_R2R_PDN_MASK,
		PMIC_DA_VS2_R2R_PDN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vs2_dvs_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VS2_DVS_EN_ADDR,
		&val,
		PMIC_DA_VS2_DVS_EN_MASK,
		PMIC_DA_VS2_DVS_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vs2_minfreq_discharge(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VS2_MINFREQ_DISCHARGE_ADDR,
		&val,
		PMIC_DA_VS2_MINFREQ_DISCHARGE_MASK,
		PMIC_DA_VS2_MINFREQ_DISCHARGE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_oc_flag_clr_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_OC_FLAG_CLR_SEL_ADDR,
		val,
		PMIC_RG_BUCK_VS2_OC_FLAG_CLR_SEL_MASK,
		PMIC_RG_BUCK_VS2_OC_FLAG_CLR_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_BUCK_VS2_OSC_SEL_DIS_MASK,
		PMIC_RG_BUCK_VS2_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_BUCK_VS2_CK_SW_MODE_MASK,
		PMIC_RG_BUCK_VS2_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_CK_SW_EN_ADDR,
		val,
		PMIC_RG_BUCK_VS2_CK_SW_EN_MASK,
		PMIC_RG_BUCK_VS2_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs2_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS2_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VS2_CK_SW_EN_MASK,
		PMIC_RG_BUCK_VS2_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_voter_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_VOTER_EN_ADDR,
		val,
		PMIC_RG_BUCK_VS2_VOTER_EN_MASK,
		PMIC_RG_BUCK_VS2_VOTER_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs2_voter_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS2_VOTER_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VS2_VOTER_EN_MASK,
		PMIC_RG_BUCK_VS2_VOTER_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vs2_voter_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VS2_VOTER_VOSEL_ADDR,
		val,
		PMIC_RG_BUCK_VS2_VOTER_VOSEL_MASK,
		PMIC_RG_BUCK_VS2_VOTER_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vs2_voter_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VS2_VOTER_VOSEL_ADDR,
		&val,
		PMIC_RG_BUCK_VS2_VOTER_VOSEL_MASK,
		PMIC_RG_BUCK_VS2_VOTER_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_EN_ADDR,
		val,
		PMIC_RG_BUCK_VPA_EN_MASK,
		PMIC_RG_BUCK_VPA_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vpa_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPA_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VPA_EN_MASK,
		PMIC_RG_BUCK_VPA_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_VOSEL_ADDR,
		val,
		PMIC_RG_BUCK_VPA_VOSEL_MASK,
		PMIC_RG_BUCK_VPA_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vpa_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPA_VOSEL_ADDR,
		&val,
		PMIC_RG_BUCK_VPA_VOSEL_MASK,
		PMIC_RG_BUCK_VPA_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_sfchg_frate(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_SFCHG_FRATE_ADDR,
		val,
		PMIC_RG_BUCK_VPA_SFCHG_FRATE_MASK,
		PMIC_RG_BUCK_VPA_SFCHG_FRATE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_sfchg_fen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_SFCHG_FEN_ADDR,
		val,
		PMIC_RG_BUCK_VPA_SFCHG_FEN_MASK,
		PMIC_RG_BUCK_VPA_SFCHG_FEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_sfchg_rrate(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_SFCHG_RRATE_ADDR,
		val,
		PMIC_RG_BUCK_VPA_SFCHG_RRATE_MASK,
		PMIC_RG_BUCK_VPA_SFCHG_RRATE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_sfchg_ren(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_SFCHG_REN_ADDR,
		val,
		PMIC_RG_BUCK_VPA_SFCHG_REN_MASK,
		PMIC_RG_BUCK_VPA_SFCHG_REN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_dvs_transt_td(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_DVS_TRANST_TD_ADDR,
		val,
		PMIC_RG_BUCK_VPA_DVS_TRANST_TD_MASK,
		PMIC_RG_BUCK_VPA_DVS_TRANST_TD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_dvs_transt_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_DVS_TRANST_CTRL_ADDR,
		val,
		PMIC_RG_BUCK_VPA_DVS_TRANST_CTRL_MASK,
		PMIC_RG_BUCK_VPA_DVS_TRANST_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_dvs_transt_once(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_DVS_TRANST_ONCE_ADDR,
		val,
		PMIC_RG_BUCK_VPA_DVS_TRANST_ONCE_MASK,
		PMIC_RG_BUCK_VPA_DVS_TRANST_ONCE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_dvs_bw_td(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_DVS_BW_TD_ADDR,
		val,
		PMIC_RG_BUCK_VPA_DVS_BW_TD_MASK,
		PMIC_RG_BUCK_VPA_DVS_BW_TD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_dvs_bw_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_DVS_BW_CTRL_ADDR,
		val,
		PMIC_RG_BUCK_VPA_DVS_BW_CTRL_MASK,
		PMIC_RG_BUCK_VPA_DVS_BW_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_dvs_bw_once(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_DVS_BW_ONCE_ADDR,
		val,
		PMIC_RG_BUCK_VPA_DVS_BW_ONCE_MASK,
		PMIC_RG_BUCK_VPA_DVS_BW_ONCE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_oc_deg_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_OC_DEG_EN_ADDR,
		val,
		PMIC_RG_BUCK_VPA_OC_DEG_EN_MASK,
		PMIC_RG_BUCK_VPA_OC_DEG_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vpa_oc_deg_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPA_OC_DEG_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VPA_OC_DEG_EN_MASK,
		PMIC_RG_BUCK_VPA_OC_DEG_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_oc_wnd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_OC_WND_ADDR,
		val,
		PMIC_RG_BUCK_VPA_OC_WND_MASK,
		PMIC_RG_BUCK_VPA_OC_WND_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_oc_thd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_OC_THD_ADDR,
		val,
		PMIC_RG_BUCK_VPA_OC_THD_MASK,
		PMIC_RG_BUCK_VPA_OC_THD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_vpa_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VPA_VOSEL_ADDR,
		&val,
		PMIC_DA_VPA_VOSEL_MASK,
		PMIC_DA_VPA_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vpa_vosel_gray(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VPA_VOSEL_GRAY_ADDR,
		&val,
		PMIC_DA_VPA_VOSEL_GRAY_MASK,
		PMIC_DA_VPA_VOSEL_GRAY_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vpa_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VPA_EN_ADDR,
		&val,
		PMIC_DA_VPA_EN_MASK,
		PMIC_DA_VPA_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vpa_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VPA_STB_ADDR,
		&val,
		PMIC_DA_VPA_STB_MASK,
		PMIC_DA_VPA_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vpa_dvs_transt(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VPA_DVS_TRANST_ADDR,
		&val,
		PMIC_DA_VPA_DVS_TRANST_MASK,
		PMIC_DA_VPA_DVS_TRANST_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_vpa_dvs_bw(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VPA_DVS_BW_ADDR,
		&val,
		PMIC_DA_VPA_DVS_BW_MASK,
		PMIC_DA_VPA_DVS_BW_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_oc_flag_clr_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_OC_FLAG_CLR_SEL_ADDR,
		val,
		PMIC_RG_BUCK_VPA_OC_FLAG_CLR_SEL_MASK,
		PMIC_RG_BUCK_VPA_OC_FLAG_CLR_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_BUCK_VPA_OSC_SEL_DIS_MASK,
		PMIC_RG_BUCK_VPA_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_BUCK_VPA_CK_SW_MODE_MASK,
		PMIC_RG_BUCK_VPA_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_CK_SW_EN_ADDR,
		val,
		PMIC_RG_BUCK_VPA_CK_SW_EN_MASK,
		PMIC_RG_BUCK_VPA_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vpa_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPA_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VPA_CK_SW_EN_MASK,
		PMIC_RG_BUCK_VPA_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_vosel_dlc011(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_VOSEL_DLC011_ADDR,
		val,
		PMIC_RG_BUCK_VPA_VOSEL_DLC011_MASK,
		PMIC_RG_BUCK_VPA_VOSEL_DLC011_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vpa_vosel_dlc011(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPA_VOSEL_DLC011_ADDR,
		&val,
		PMIC_RG_BUCK_VPA_VOSEL_DLC011_MASK,
		PMIC_RG_BUCK_VPA_VOSEL_DLC011_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_vosel_dlc111(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_VOSEL_DLC111_ADDR,
		val,
		PMIC_RG_BUCK_VPA_VOSEL_DLC111_MASK,
		PMIC_RG_BUCK_VPA_VOSEL_DLC111_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vpa_vosel_dlc111(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPA_VOSEL_DLC111_ADDR,
		&val,
		PMIC_RG_BUCK_VPA_VOSEL_DLC111_MASK,
		PMIC_RG_BUCK_VPA_VOSEL_DLC111_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_vosel_dlc001(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_VOSEL_DLC001_ADDR,
		val,
		PMIC_RG_BUCK_VPA_VOSEL_DLC001_MASK,
		PMIC_RG_BUCK_VPA_VOSEL_DLC001_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vpa_vosel_dlc001(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPA_VOSEL_DLC001_ADDR,
		&val,
		PMIC_RG_BUCK_VPA_VOSEL_DLC001_MASK,
		PMIC_RG_BUCK_VPA_VOSEL_DLC001_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_dlc_map_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_DLC_MAP_EN_ADDR,
		val,
		PMIC_RG_BUCK_VPA_DLC_MAP_EN_MASK,
		PMIC_RG_BUCK_VPA_DLC_MAP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vpa_dlc_map_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPA_DLC_MAP_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VPA_DLC_MAP_EN_MASK,
		PMIC_RG_BUCK_VPA_DLC_MAP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_dlc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_DLC_ADDR,
		val,
		PMIC_RG_BUCK_VPA_DLC_MASK,
		PMIC_RG_BUCK_VPA_DLC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_vpa_dlc(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_VPA_DLC_ADDR,
		&val,
		PMIC_DA_VPA_DLC_MASK,
		PMIC_DA_VPA_DLC_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_msfg_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_MSFG_EN_ADDR,
		val,
		PMIC_RG_BUCK_VPA_MSFG_EN_MASK,
		PMIC_RG_BUCK_VPA_MSFG_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_buck_vpa_msfg_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_BUCK_VPA_MSFG_EN_ADDR,
		&val,
		PMIC_RG_BUCK_VPA_MSFG_EN_MASK,
		PMIC_RG_BUCK_VPA_MSFG_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_msfg_rdelta2go(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_MSFG_RDELTA2GO_ADDR,
		val,
		PMIC_RG_BUCK_VPA_MSFG_RDELTA2GO_MASK,
		PMIC_RG_BUCK_VPA_MSFG_RDELTA2GO_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_msfg_fdelta2go(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_MSFG_FDELTA2GO_ADDR,
		val,
		PMIC_RG_BUCK_VPA_MSFG_FDELTA2GO_MASK,
		PMIC_RG_BUCK_VPA_MSFG_FDELTA2GO_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_msfg_rrate0(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_MSFG_RRATE0_ADDR,
		val,
		PMIC_RG_BUCK_VPA_MSFG_RRATE0_MASK,
		PMIC_RG_BUCK_VPA_MSFG_RRATE0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_msfg_rrate1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_MSFG_RRATE1_ADDR,
		val,
		PMIC_RG_BUCK_VPA_MSFG_RRATE1_MASK,
		PMIC_RG_BUCK_VPA_MSFG_RRATE1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_msfg_rrate2(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_MSFG_RRATE2_ADDR,
		val,
		PMIC_RG_BUCK_VPA_MSFG_RRATE2_MASK,
		PMIC_RG_BUCK_VPA_MSFG_RRATE2_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_msfg_rrate3(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_MSFG_RRATE3_ADDR,
		val,
		PMIC_RG_BUCK_VPA_MSFG_RRATE3_MASK,
		PMIC_RG_BUCK_VPA_MSFG_RRATE3_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_msfg_rrate4(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_MSFG_RRATE4_ADDR,
		val,
		PMIC_RG_BUCK_VPA_MSFG_RRATE4_MASK,
		PMIC_RG_BUCK_VPA_MSFG_RRATE4_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_msfg_rrate5(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_MSFG_RRATE5_ADDR,
		val,
		PMIC_RG_BUCK_VPA_MSFG_RRATE5_MASK,
		PMIC_RG_BUCK_VPA_MSFG_RRATE5_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_msfg_rthd0(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_MSFG_RTHD0_ADDR,
		val,
		PMIC_RG_BUCK_VPA_MSFG_RTHD0_MASK,
		PMIC_RG_BUCK_VPA_MSFG_RTHD0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_msfg_rthd1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_MSFG_RTHD1_ADDR,
		val,
		PMIC_RG_BUCK_VPA_MSFG_RTHD1_MASK,
		PMIC_RG_BUCK_VPA_MSFG_RTHD1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_msfg_rthd2(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_MSFG_RTHD2_ADDR,
		val,
		PMIC_RG_BUCK_VPA_MSFG_RTHD2_MASK,
		PMIC_RG_BUCK_VPA_MSFG_RTHD2_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_msfg_rthd3(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_MSFG_RTHD3_ADDR,
		val,
		PMIC_RG_BUCK_VPA_MSFG_RTHD3_MASK,
		PMIC_RG_BUCK_VPA_MSFG_RTHD3_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_msfg_rthd4(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_MSFG_RTHD4_ADDR,
		val,
		PMIC_RG_BUCK_VPA_MSFG_RTHD4_MASK,
		PMIC_RG_BUCK_VPA_MSFG_RTHD4_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_msfg_frate0(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_MSFG_FRATE0_ADDR,
		val,
		PMIC_RG_BUCK_VPA_MSFG_FRATE0_MASK,
		PMIC_RG_BUCK_VPA_MSFG_FRATE0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_msfg_frate1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_MSFG_FRATE1_ADDR,
		val,
		PMIC_RG_BUCK_VPA_MSFG_FRATE1_MASK,
		PMIC_RG_BUCK_VPA_MSFG_FRATE1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_msfg_frate2(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_MSFG_FRATE2_ADDR,
		val,
		PMIC_RG_BUCK_VPA_MSFG_FRATE2_MASK,
		PMIC_RG_BUCK_VPA_MSFG_FRATE2_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_msfg_frate3(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_MSFG_FRATE3_ADDR,
		val,
		PMIC_RG_BUCK_VPA_MSFG_FRATE3_MASK,
		PMIC_RG_BUCK_VPA_MSFG_FRATE3_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_msfg_frate4(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_MSFG_FRATE4_ADDR,
		val,
		PMIC_RG_BUCK_VPA_MSFG_FRATE4_MASK,
		PMIC_RG_BUCK_VPA_MSFG_FRATE4_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_msfg_frate5(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_MSFG_FRATE5_ADDR,
		val,
		PMIC_RG_BUCK_VPA_MSFG_FRATE5_MASK,
		PMIC_RG_BUCK_VPA_MSFG_FRATE5_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_msfg_fthd0(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_MSFG_FTHD0_ADDR,
		val,
		PMIC_RG_BUCK_VPA_MSFG_FTHD0_MASK,
		PMIC_RG_BUCK_VPA_MSFG_FTHD0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_msfg_fthd1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_MSFG_FTHD1_ADDR,
		val,
		PMIC_RG_BUCK_VPA_MSFG_FTHD1_MASK,
		PMIC_RG_BUCK_VPA_MSFG_FTHD1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_msfg_fthd2(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_MSFG_FTHD2_ADDR,
		val,
		PMIC_RG_BUCK_VPA_MSFG_FTHD2_MASK,
		PMIC_RG_BUCK_VPA_MSFG_FTHD2_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_msfg_fthd3(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_MSFG_FTHD3_ADDR,
		val,
		PMIC_RG_BUCK_VPA_MSFG_FTHD3_MASK,
		PMIC_RG_BUCK_VPA_MSFG_FTHD3_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_buck_vpa_msfg_fthd4(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_BUCK_VPA_MSFG_FTHD4_ADDR,
		val,
		PMIC_RG_BUCK_VPA_MSFG_FTHD4_MASK,
		PMIC_RG_BUCK_VPA_MSFG_FTHD4_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_smps_testmode_b(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SMPS_TESTMODE_B_ADDR,
		val,
		PMIC_RG_SMPS_TESTMODE_B_MASK,
		PMIC_RG_SMPS_TESTMODE_B_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vpa_bursth(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPA_BURSTH_ADDR,
		val,
		PMIC_RG_VPA_BURSTH_MASK,
		PMIC_RG_VPA_BURSTH_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vpa_burstl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPA_BURSTL_ADDR,
		val,
		PMIC_RG_VPA_BURSTL_MASK,
		PMIC_RG_VPA_BURSTL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vpa_trimh(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPA_TRIMH_ADDR,
		val,
		PMIC_RG_VPA_TRIMH_MASK,
		PMIC_RG_VPA_TRIMH_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vpa_triml(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPA_TRIML_ADDR,
		val,
		PMIC_RG_VPA_TRIML_MASK,
		PMIC_RG_VPA_TRIML_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vpa_trim_ref(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPA_TRIM_REF_ADDR,
		val,
		PMIC_RG_VPA_TRIM_REF_MASK,
		PMIC_RG_VPA_TRIM_REF_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_trimh(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_TRIMH_ADDR,
		val,
		PMIC_RG_VCORE_TRIMH_MASK,
		PMIC_RG_VCORE_TRIMH_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_triml(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_TRIML_ADDR,
		val,
		PMIC_RG_VCORE_TRIML_MASK,
		PMIC_RG_VCORE_TRIML_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_vsleep_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_VSLEEP_TRIM_ADDR,
		val,
		PMIC_RG_VCORE_VSLEEP_TRIM_MASK,
		PMIC_RG_VCORE_VSLEEP_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_sleep_voltage(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_SLEEP_VOLTAGE_ADDR,
		val,
		PMIC_RG_VCORE_SLEEP_VOLTAGE_MASK,
		PMIC_RG_VCORE_SLEEP_VOLTAGE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_trimh(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_TRIMH_ADDR,
		val,
		PMIC_RG_VDRAM1_TRIMH_MASK,
		PMIC_RG_VDRAM1_TRIMH_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_triml(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_TRIML_ADDR,
		val,
		PMIC_RG_VDRAM1_TRIML_MASK,
		PMIC_RG_VDRAM1_TRIML_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_vsleep_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_VSLEEP_TRIM_ADDR,
		val,
		PMIC_RG_VDRAM1_VSLEEP_TRIM_MASK,
		PMIC_RG_VDRAM1_VSLEEP_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_sleep_voltage(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_SLEEP_VOLTAGE_ADDR,
		val,
		PMIC_RG_VDRAM1_SLEEP_VOLTAGE_MASK,
		PMIC_RG_VDRAM1_SLEEP_VOLTAGE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_trimh(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_TRIMH_ADDR,
		val,
		PMIC_RG_VMODEM_TRIMH_MASK,
		PMIC_RG_VMODEM_TRIMH_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_triml(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_TRIML_ADDR,
		val,
		PMIC_RG_VMODEM_TRIML_MASK,
		PMIC_RG_VMODEM_TRIML_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_vsleep_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_VSLEEP_TRIM_ADDR,
		val,
		PMIC_RG_VMODEM_VSLEEP_TRIM_MASK,
		PMIC_RG_VMODEM_VSLEEP_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_sleep_voltage(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_SLEEP_VOLTAGE_ADDR,
		val,
		PMIC_RG_VMODEM_SLEEP_VOLTAGE_MASK,
		PMIC_RG_VMODEM_SLEEP_VOLTAGE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_trimh(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_TRIMH_ADDR,
		val,
		PMIC_RG_VGPU_TRIMH_MASK,
		PMIC_RG_VGPU_TRIMH_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_triml(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_TRIML_ADDR,
		val,
		PMIC_RG_VGPU_TRIML_MASK,
		PMIC_RG_VGPU_TRIML_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_vsleep_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_VSLEEP_TRIM_ADDR,
		val,
		PMIC_RG_VGPU_VSLEEP_TRIM_MASK,
		PMIC_RG_VGPU_VSLEEP_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_sleep_voltage(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_SLEEP_VOLTAGE_ADDR,
		val,
		PMIC_RG_VGPU_SLEEP_VOLTAGE_MASK,
		PMIC_RG_VGPU_SLEEP_VOLTAGE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs1_trimh(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS1_TRIMH_ADDR,
		val,
		PMIC_RG_VS1_TRIMH_MASK,
		PMIC_RG_VS1_TRIMH_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs1_triml(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS1_TRIML_ADDR,
		val,
		PMIC_RG_VS1_TRIML_MASK,
		PMIC_RG_VS1_TRIML_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs1_vsleep_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS1_VSLEEP_TRIM_ADDR,
		val,
		PMIC_RG_VS1_VSLEEP_TRIM_MASK,
		PMIC_RG_VS1_VSLEEP_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs1_sleep_voltage(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS1_SLEEP_VOLTAGE_ADDR,
		val,
		PMIC_RG_VS1_SLEEP_VOLTAGE_MASK,
		PMIC_RG_VS1_SLEEP_VOLTAGE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs2_trimh(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS2_TRIMH_ADDR,
		val,
		PMIC_RG_VS2_TRIMH_MASK,
		PMIC_RG_VS2_TRIMH_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs2_triml(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS2_TRIML_ADDR,
		val,
		PMIC_RG_VS2_TRIML_MASK,
		PMIC_RG_VS2_TRIML_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs2_vsleep_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS2_VSLEEP_TRIM_ADDR,
		val,
		PMIC_RG_VS2_VSLEEP_TRIM_MASK,
		PMIC_RG_VS2_VSLEEP_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs2_sleep_voltage(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS2_SLEEP_VOLTAGE_ADDR,
		val,
		PMIC_RG_VS2_SLEEP_VOLTAGE_MASK,
		PMIC_RG_VS2_SLEEP_VOLTAGE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_trimh(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_TRIMH_ADDR,
		val,
		PMIC_RG_VDRAM2_TRIMH_MASK,
		PMIC_RG_VDRAM2_TRIMH_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_triml(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_TRIML_ADDR,
		val,
		PMIC_RG_VDRAM2_TRIML_MASK,
		PMIC_RG_VDRAM2_TRIML_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_vsleep_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_VSLEEP_TRIM_ADDR,
		val,
		PMIC_RG_VDRAM2_VSLEEP_TRIM_MASK,
		PMIC_RG_VDRAM2_VSLEEP_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_sleep_voltage(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_SLEEP_VOLTAGE_ADDR,
		val,
		PMIC_RG_VDRAM2_SLEEP_VOLTAGE_MASK,
		PMIC_RG_VDRAM2_SLEEP_VOLTAGE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_trimh(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_TRIMH_ADDR,
		val,
		PMIC_RG_VPROC11_TRIMH_MASK,
		PMIC_RG_VPROC11_TRIMH_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_triml(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_TRIML_ADDR,
		val,
		PMIC_RG_VPROC11_TRIML_MASK,
		PMIC_RG_VPROC11_TRIML_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_vsleep_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_VSLEEP_TRIM_ADDR,
		val,
		PMIC_RG_VPROC11_VSLEEP_TRIM_MASK,
		PMIC_RG_VPROC11_VSLEEP_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_sleep_voltage(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_SLEEP_VOLTAGE_ADDR,
		val,
		PMIC_RG_VPROC11_SLEEP_VOLTAGE_MASK,
		PMIC_RG_VPROC11_SLEEP_VOLTAGE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_trimh(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_TRIMH_ADDR,
		val,
		PMIC_RG_VPROC12_TRIMH_MASK,
		PMIC_RG_VPROC12_TRIMH_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_triml(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_TRIML_ADDR,
		val,
		PMIC_RG_VPROC12_TRIML_MASK,
		PMIC_RG_VPROC12_TRIML_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_vsleep_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_VSLEEP_TRIM_ADDR,
		val,
		PMIC_RG_VPROC12_VSLEEP_TRIM_MASK,
		PMIC_RG_VPROC12_VSLEEP_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_sleep_voltage(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_SLEEP_VOLTAGE_ADDR,
		val,
		PMIC_RG_VPROC12_SLEEP_VOLTAGE_MASK,
		PMIC_RG_VPROC12_SLEEP_VOLTAGE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vsram_proc_trimh(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VSRAM_PROC_TRIMH_ADDR,
		val,
		PMIC_RG_VSRAM_PROC_TRIMH_MASK,
		PMIC_RG_VSRAM_PROC_TRIMH_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vsram_proc_triml(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VSRAM_PROC_TRIML_ADDR,
		val,
		PMIC_RG_VSRAM_PROC_TRIML_MASK,
		PMIC_RG_VSRAM_PROC_TRIML_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vsram_proc_vsleep_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VSRAM_PROC_VSLEEP_TRIM_ADDR,
		val,
		PMIC_RG_VSRAM_PROC_VSLEEP_TRIM_MASK,
		PMIC_RG_VSRAM_PROC_VSLEEP_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vsram_proc_sleep_voltage(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VSRAM_PROC_SLEEP_VOLTAGE_ADDR,
		val,
		PMIC_RG_VSRAM_PROC_SLEEP_VOLTAGE_MASK,
		PMIC_RG_VSRAM_PROC_SLEEP_VOLTAGE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vsram_core_trimh(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VSRAM_CORE_TRIMH_ADDR,
		val,
		PMIC_RG_VSRAM_CORE_TRIMH_MASK,
		PMIC_RG_VSRAM_CORE_TRIMH_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vsram_core_triml(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VSRAM_CORE_TRIML_ADDR,
		val,
		PMIC_RG_VSRAM_CORE_TRIML_MASK,
		PMIC_RG_VSRAM_CORE_TRIML_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vsram_core_vsleep_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VSRAM_CORE_VSLEEP_TRIM_ADDR,
		val,
		PMIC_RG_VSRAM_CORE_VSLEEP_TRIM_MASK,
		PMIC_RG_VSRAM_CORE_VSLEEP_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vsram_core_sleep_voltage(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VSRAM_CORE_SLEEP_VOLTAGE_ADDR,
		val,
		PMIC_RG_VSRAM_CORE_SLEEP_VOLTAGE_MASK,
		PMIC_RG_VSRAM_CORE_SLEEP_VOLTAGE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vsram_gpu_trimh(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VSRAM_GPU_TRIMH_ADDR,
		val,
		PMIC_RG_VSRAM_GPU_TRIMH_MASK,
		PMIC_RG_VSRAM_GPU_TRIMH_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vsram_gpu_triml(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VSRAM_GPU_TRIML_ADDR,
		val,
		PMIC_RG_VSRAM_GPU_TRIML_MASK,
		PMIC_RG_VSRAM_GPU_TRIML_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vsram_gpu_vsleep_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VSRAM_GPU_VSLEEP_TRIM_ADDR,
		val,
		PMIC_RG_VSRAM_GPU_VSLEEP_TRIM_MASK,
		PMIC_RG_VSRAM_GPU_VSLEEP_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vsram_gpu_sleep_voltage(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VSRAM_GPU_SLEEP_VOLTAGE_ADDR,
		val,
		PMIC_RG_VSRAM_GPU_SLEEP_VOLTAGE_MASK,
		PMIC_RG_VSRAM_GPU_SLEEP_VOLTAGE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vsram_md_trimh(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VSRAM_MD_TRIMH_ADDR,
		val,
		PMIC_RG_VSRAM_MD_TRIMH_MASK,
		PMIC_RG_VSRAM_MD_TRIMH_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vsram_md_triml(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VSRAM_MD_TRIML_ADDR,
		val,
		PMIC_RG_VSRAM_MD_TRIML_MASK,
		PMIC_RG_VSRAM_MD_TRIML_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vsram_md_vsleep_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VSRAM_MD_VSLEEP_TRIM_ADDR,
		val,
		PMIC_RG_VSRAM_MD_VSLEEP_TRIM_MASK,
		PMIC_RG_VSRAM_MD_VSLEEP_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vsram_md_sleep_voltage(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VSRAM_MD_SLEEP_VOLTAGE_ADDR,
		val,
		PMIC_RG_VSRAM_MD_SLEEP_VOLTAGE_MASK,
		PMIC_RG_VSRAM_MD_SLEEP_VOLTAGE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_smps_ivgd_det(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_SMPS_IVGD_DET_ADDR,
		val,
		PMIC_RG_SMPS_IVGD_DET_MASK,
		PMIC_RG_SMPS_IVGD_DET_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_voutdet_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VOUTDET_EN_ADDR,
		val,
		PMIC_RG_VOUTDET_EN_MASK,
		PMIC_RG_VOUTDET_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_voutdet_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VOUTDET_EN_ADDR,
		&val,
		PMIC_RG_VOUTDET_EN_MASK,
		PMIC_RG_VOUTDET_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_autok_rst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_AUTOK_RST_ADDR,
		val,
		PMIC_RG_AUTOK_RST_MASK,
		PMIC_RG_AUTOK_RST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_fpwm(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_FPWM_ADDR,
		val,
		PMIC_RG_VPROC11_FPWM_MASK,
		PMIC_RG_VPROC11_FPWM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_fpwm(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_FPWM_ADDR,
		val,
		PMIC_RG_VPROC12_FPWM_MASK,
		PMIC_RG_VPROC12_FPWM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_ndis_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_NDIS_EN_ADDR,
		val,
		PMIC_RG_VPROC11_NDIS_EN_MASK,
		PMIC_RG_VPROC11_NDIS_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vproc11_ndis_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VPROC11_NDIS_EN_ADDR,
		&val,
		PMIC_RG_VPROC11_NDIS_EN_MASK,
		PMIC_RG_VPROC11_NDIS_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vproc12_ndis_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_NDIS_EN_ADDR,
		val,
		PMIC_RG_VPROC12_NDIS_EN_MASK,
		PMIC_RG_VPROC12_NDIS_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vproc12_ndis_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VPROC12_NDIS_EN_ADDR,
		&val,
		PMIC_RG_VPROC12_NDIS_EN_MASK,
		PMIC_RG_VPROC12_NDIS_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vproc11_fcot(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_FCOT_ADDR,
		val,
		PMIC_RG_VPROC11_FCOT_MASK,
		PMIC_RG_VPROC11_FCOT_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_fcot(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_FCOT_ADDR,
		val,
		PMIC_RG_VPROC12_FCOT_MASK,
		PMIC_RG_VPROC12_FCOT_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc_tmdl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC_TMDL_ADDR,
		val,
		PMIC_RG_VPROC_TMDL_MASK,
		PMIC_RG_VPROC_TMDL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc_disconfig20(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC_DISCONFIG20_ADDR,
		val,
		PMIC_RG_VPROC_DISCONFIG20_MASK,
		PMIC_RG_VPROC_DISCONFIG20_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_tbdis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_TBDIS_ADDR,
		val,
		PMIC_RG_VPROC11_TBDIS_MASK,
		PMIC_RG_VPROC11_TBDIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_tbdis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_TBDIS_ADDR,
		val,
		PMIC_RG_VPROC12_TBDIS_MASK,
		PMIC_RG_VPROC12_TBDIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_vdiffoff(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_VDIFFOFF_ADDR,
		val,
		PMIC_RG_VPROC11_VDIFFOFF_MASK,
		PMIC_RG_VPROC11_VDIFFOFF_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_vdiffoff(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_VDIFFOFF_ADDR,
		val,
		PMIC_RG_VPROC12_VDIFFOFF_MASK,
		PMIC_RG_VPROC12_VDIFFOFF_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_rcomp0(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_RCOMP0_ADDR,
		val,
		PMIC_RG_VPROC11_RCOMP0_MASK,
		PMIC_RG_VPROC11_RCOMP0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_rcomp1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_RCOMP1_ADDR,
		val,
		PMIC_RG_VPROC11_RCOMP1_MASK,
		PMIC_RG_VPROC11_RCOMP1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_ccomp0(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_CCOMP0_ADDR,
		val,
		PMIC_RG_VPROC11_CCOMP0_MASK,
		PMIC_RG_VPROC11_CCOMP0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_ccomp1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_CCOMP1_ADDR,
		val,
		PMIC_RG_VPROC11_CCOMP1_MASK,
		PMIC_RG_VPROC11_CCOMP1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_ramp_slp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_RAMP_SLP_ADDR,
		val,
		PMIC_RG_VPROC11_RAMP_SLP_MASK,
		PMIC_RG_VPROC11_RAMP_SLP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_rcomp0(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_RCOMP0_ADDR,
		val,
		PMIC_RG_VPROC12_RCOMP0_MASK,
		PMIC_RG_VPROC12_RCOMP0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_rcomp1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_RCOMP1_ADDR,
		val,
		PMIC_RG_VPROC12_RCOMP1_MASK,
		PMIC_RG_VPROC12_RCOMP1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_ccomp0(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_CCOMP0_ADDR,
		val,
		PMIC_RG_VPROC12_CCOMP0_MASK,
		PMIC_RG_VPROC12_CCOMP0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_ccomp1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_CCOMP1_ADDR,
		val,
		PMIC_RG_VPROC12_CCOMP1_MASK,
		PMIC_RG_VPROC12_CCOMP1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_ramp_slp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_RAMP_SLP_ADDR,
		val,
		PMIC_RG_VPROC12_RAMP_SLP_MASK,
		PMIC_RG_VPROC12_RAMP_SLP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_rcs(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_RCS_ADDR,
		val,
		PMIC_RG_VPROC11_RCS_MASK,
		PMIC_RG_VPROC11_RCS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_rcs(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_RCS_ADDR,
		val,
		PMIC_RG_VPROC12_RCS_MASK,
		PMIC_RG_VPROC12_RCS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_rcb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_RCB_ADDR,
		val,
		PMIC_RG_VPROC11_RCB_MASK,
		PMIC_RG_VPROC11_RCB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_rcb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_RCB_ADDR,
		val,
		PMIC_RG_VPROC12_RCB_MASK,
		PMIC_RG_VPROC12_RCB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_csp_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_CSP_TRIM_ADDR,
		val,
		PMIC_RG_VPROC11_CSP_TRIM_MASK,
		PMIC_RG_VPROC11_CSP_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_csp_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_CSP_TRIM_ADDR,
		val,
		PMIC_RG_VPROC12_CSP_TRIM_MASK,
		PMIC_RG_VPROC12_CSP_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_csn_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_CSN_TRIM_ADDR,
		val,
		PMIC_RG_VPROC11_CSN_TRIM_MASK,
		PMIC_RG_VPROC11_CSN_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_csn_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_CSN_TRIM_ADDR,
		val,
		PMIC_RG_VPROC12_CSN_TRIM_MASK,
		PMIC_RG_VPROC12_CSN_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_zc_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_ZC_TRIM_ADDR,
		val,
		PMIC_RG_VPROC11_ZC_TRIM_MASK,
		PMIC_RG_VPROC11_ZC_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_zc_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_ZC_TRIM_ADDR,
		val,
		PMIC_RG_VPROC12_ZC_TRIM_MASK,
		PMIC_RG_VPROC12_ZC_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_nlim_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_NLIM_TRIM_ADDR,
		val,
		PMIC_RG_VPROC11_NLIM_TRIM_MASK,
		PMIC_RG_VPROC11_NLIM_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_nlim_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_NLIM_TRIM_ADDR,
		val,
		PMIC_RG_VPROC12_NLIM_TRIM_MASK,
		PMIC_RG_VPROC12_NLIM_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_rpsi1_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_RPSI1_TRIM_ADDR,
		val,
		PMIC_RG_VPROC11_RPSI1_TRIM_MASK,
		PMIC_RG_VPROC11_RPSI1_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_rpsi1_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_RPSI1_TRIM_ADDR,
		val,
		PMIC_RG_VPROC12_RPSI1_TRIM_MASK,
		PMIC_RG_VPROC12_RPSI1_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_tb_width(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_TB_WIDTH_ADDR,
		val,
		PMIC_RG_VPROC11_TB_WIDTH_MASK,
		PMIC_RG_VPROC11_TB_WIDTH_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_tb_width(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_TB_WIDTH_ADDR,
		val,
		PMIC_RG_VPROC12_TB_WIDTH_MASK,
		PMIC_RG_VPROC12_TB_WIDTH_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_ug_sr(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_UG_SR_ADDR,
		val,
		PMIC_RG_VPROC11_UG_SR_MASK,
		PMIC_RG_VPROC11_UG_SR_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_lg_sr(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_LG_SR_ADDR,
		val,
		PMIC_RG_VPROC11_LG_SR_MASK,
		PMIC_RG_VPROC11_LG_SR_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_ug_sr(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_UG_SR_ADDR,
		val,
		PMIC_RG_VPROC12_UG_SR_MASK,
		PMIC_RG_VPROC12_UG_SR_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_lg_sr(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_LG_SR_ADDR,
		val,
		PMIC_RG_VPROC12_LG_SR_MASK,
		PMIC_RG_VPROC12_LG_SR_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_pfm_ton(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_PFM_TON_ADDR,
		val,
		PMIC_RG_VPROC11_PFM_TON_MASK,
		PMIC_RG_VPROC11_PFM_TON_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_pfm_ton(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_PFM_TON_ADDR,
		val,
		PMIC_RG_VPROC12_PFM_TON_MASK,
		PMIC_RG_VPROC12_PFM_TON_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_ton_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_TON_TRIM_ADDR,
		val,
		PMIC_RG_VPROC11_TON_TRIM_MASK,
		PMIC_RG_VPROC11_TON_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_ton_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_TON_TRIM_ADDR,
		val,
		PMIC_RG_VPROC12_TON_TRIM_MASK,
		PMIC_RG_VPROC12_TON_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rgs_vproc11_oc_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VPROC11_OC_STATUS_ADDR,
		&val,
		PMIC_RGS_VPROC11_OC_STATUS_MASK,
		PMIC_RGS_VPROC11_OC_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rgs_vproc12_oc_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VPROC12_OC_STATUS_ADDR,
		&val,
		PMIC_RGS_VPROC12_OC_STATUS_MASK,
		PMIC_RGS_VPROC12_OC_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rgs_vproc_trimok_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VPROC_TRIMOK_STATUS_ADDR,
		&val,
		PMIC_RGS_VPROC_TRIMOK_STATUS_MASK,
		PMIC_RGS_VPROC_TRIMOK_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rgs_vproc_config20_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VPROC_CONFIG20_STATUS_ADDR,
		&val,
		PMIC_RGS_VPROC_CONFIG20_STATUS_MASK,
		PMIC_RGS_VPROC_CONFIG20_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rgs_vproc11_preoc_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VPROC11_PREOC_STATUS_ADDR,
		&val,
		PMIC_RGS_VPROC11_PREOC_STATUS_MASK,
		PMIC_RGS_VPROC11_PREOC_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rgs_vproc11_dig_mon(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VPROC11_DIG_MON_ADDR,
		&val,
		PMIC_RGS_VPROC11_DIG_MON_MASK,
		PMIC_RGS_VPROC11_DIG_MON_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rgs_vproc12_dig_mon(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VPROC12_DIG_MON_ADDR,
		&val,
		PMIC_RGS_VPROC12_DIG_MON_MASK,
		PMIC_RGS_VPROC12_DIG_MON_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vproc11_tran_bst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_TRAN_BST_ADDR,
		val,
		PMIC_RG_VPROC11_TRAN_BST_MASK,
		PMIC_RG_VPROC11_TRAN_BST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_tran_bst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_TRAN_BST_ADDR,
		val,
		PMIC_RG_VPROC12_TRAN_BST_MASK,
		PMIC_RG_VPROC12_TRAN_BST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_cotramp_slp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_COTRAMP_SLP_ADDR,
		val,
		PMIC_RG_VPROC11_COTRAMP_SLP_MASK,
		PMIC_RG_VPROC11_COTRAMP_SLP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_cotramp_slp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_COTRAMP_SLP_ADDR,
		val,
		PMIC_RG_VPROC12_COTRAMP_SLP_MASK,
		PMIC_RG_VPROC12_COTRAMP_SLP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_sleep_time(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_SLEEP_TIME_ADDR,
		val,
		PMIC_RG_VPROC11_SLEEP_TIME_MASK,
		PMIC_RG_VPROC11_SLEEP_TIME_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_sleep_time(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_SLEEP_TIME_ADDR,
		val,
		PMIC_RG_VPROC12_SLEEP_TIME_MASK,
		PMIC_RG_VPROC12_SLEEP_TIME_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_vreftb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_VREFTB_ADDR,
		val,
		PMIC_RG_VPROC11_VREFTB_MASK,
		PMIC_RG_VPROC11_VREFTB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_vreftb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_VREFTB_ADDR,
		val,
		PMIC_RG_VPROC12_VREFTB_MASK,
		PMIC_RG_VPROC12_VREFTB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_csnslp_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_CSNSLP_TRIM_ADDR,
		val,
		PMIC_RG_VPROC11_CSNSLP_TRIM_MASK,
		PMIC_RG_VPROC11_CSNSLP_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_csnslp_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_CSNSLP_TRIM_ADDR,
		val,
		PMIC_RG_VPROC12_CSNSLP_TRIM_MASK,
		PMIC_RG_VPROC12_CSNSLP_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_cspslp_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_CSPSLP_TRIM_ADDR,
		val,
		PMIC_RG_VPROC11_CSPSLP_TRIM_MASK,
		PMIC_RG_VPROC11_CSPSLP_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_cspslp_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_CSPSLP_TRIM_ADDR,
		val,
		PMIC_RG_VPROC12_CSPSLP_TRIM_MASK,
		PMIC_RG_VPROC12_CSPSLP_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_fugon(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_FUGON_ADDR,
		val,
		PMIC_RG_VPROC11_FUGON_MASK,
		PMIC_RG_VPROC11_FUGON_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_fugon(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_FUGON_ADDR,
		val,
		PMIC_RG_VPROC12_FUGON_MASK,
		PMIC_RG_VPROC12_FUGON_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_flgon(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_FLGON_ADDR,
		val,
		PMIC_RG_VPROC11_FLGON_MASK,
		PMIC_RG_VPROC11_FLGON_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_flgon(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_FLGON_ADDR,
		val,
		PMIC_RG_VPROC12_FLGON_MASK,
		PMIC_RG_VPROC12_FLGON_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_preoc_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_PREOC_TRIM_ADDR,
		val,
		PMIC_RG_VPROC11_PREOC_TRIM_MASK,
		PMIC_RG_VPROC11_PREOC_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_RSV_ADDR,
		val,
		PMIC_RG_VPROC11_RSV_MASK,
		PMIC_RG_VPROC11_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc12_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_RSV_ADDR,
		val,
		PMIC_RG_VPROC12_RSV_MASK,
		PMIC_RG_VPROC12_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vproc11_nonaudible_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC11_NONAUDIBLE_EN_ADDR,
		val,
		PMIC_RG_VPROC11_NONAUDIBLE_EN_MASK,
		PMIC_RG_VPROC11_NONAUDIBLE_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vproc11_nonaudible_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VPROC11_NONAUDIBLE_EN_ADDR,
		&val,
		PMIC_RG_VPROC11_NONAUDIBLE_EN_MASK,
		PMIC_RG_VPROC11_NONAUDIBLE_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vproc12_nonaudible_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC12_NONAUDIBLE_EN_ADDR,
		val,
		PMIC_RG_VPROC12_NONAUDIBLE_EN_MASK,
		PMIC_RG_VPROC12_NONAUDIBLE_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vproc12_nonaudible_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VPROC12_NONAUDIBLE_EN_ADDR,
		&val,
		PMIC_RG_VPROC12_NONAUDIBLE_EN_MASK,
		PMIC_RG_VPROC12_NONAUDIBLE_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vproc_disautok(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPROC_DISAUTOK_ADDR,
		val,
		PMIC_RG_VPROC_DISAUTOK_MASK,
		PMIC_RG_VPROC_DISAUTOK_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_fpwm(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_FPWM_ADDR,
		val,
		PMIC_RG_VCORE_FPWM_MASK,
		PMIC_RG_VCORE_FPWM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_fpwm(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_FPWM_ADDR,
		val,
		PMIC_RG_VGPU_FPWM_MASK,
		PMIC_RG_VGPU_FPWM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_ndis_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_NDIS_EN_ADDR,
		val,
		PMIC_RG_VCORE_NDIS_EN_MASK,
		PMIC_RG_VCORE_NDIS_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vcore_ndis_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VCORE_NDIS_EN_ADDR,
		&val,
		PMIC_RG_VCORE_NDIS_EN_MASK,
		PMIC_RG_VCORE_NDIS_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vgpu_ndis_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_NDIS_EN_ADDR,
		val,
		PMIC_RG_VGPU_NDIS_EN_MASK,
		PMIC_RG_VGPU_NDIS_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vgpu_ndis_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VGPU_NDIS_EN_ADDR,
		&val,
		PMIC_RG_VGPU_NDIS_EN_MASK,
		PMIC_RG_VGPU_NDIS_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vcore_fcot(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_FCOT_ADDR,
		val,
		PMIC_RG_VCORE_FCOT_MASK,
		PMIC_RG_VCORE_FCOT_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_fcot(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_FCOT_ADDR,
		val,
		PMIC_RG_VGPU_FCOT_MASK,
		PMIC_RG_VGPU_FCOT_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcorevgpu_tmdl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCOREVGPU_TMDL_ADDR,
		val,
		PMIC_RG_VCOREVGPU_TMDL_MASK,
		PMIC_RG_VCOREVGPU_TMDL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcorevgpu_disconfig20(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCOREVGPU_DISCONFIG20_ADDR,
		val,
		PMIC_RG_VCOREVGPU_DISCONFIG20_MASK,
		PMIC_RG_VCOREVGPU_DISCONFIG20_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_tbdis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_TBDIS_ADDR,
		val,
		PMIC_RG_VCORE_TBDIS_MASK,
		PMIC_RG_VCORE_TBDIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_tbdis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_TBDIS_ADDR,
		val,
		PMIC_RG_VGPU_TBDIS_MASK,
		PMIC_RG_VGPU_TBDIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_vdiffoff(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_VDIFFOFF_ADDR,
		val,
		PMIC_RG_VCORE_VDIFFOFF_MASK,
		PMIC_RG_VCORE_VDIFFOFF_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_vdiffoff(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_VDIFFOFF_ADDR,
		val,
		PMIC_RG_VGPU_VDIFFOFF_MASK,
		PMIC_RG_VGPU_VDIFFOFF_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_rcomp0(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_RCOMP0_ADDR,
		val,
		PMIC_RG_VCORE_RCOMP0_MASK,
		PMIC_RG_VCORE_RCOMP0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_rcomp1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_RCOMP1_ADDR,
		val,
		PMIC_RG_VCORE_RCOMP1_MASK,
		PMIC_RG_VCORE_RCOMP1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_ccomp0(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_CCOMP0_ADDR,
		val,
		PMIC_RG_VCORE_CCOMP0_MASK,
		PMIC_RG_VCORE_CCOMP0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_ccomp1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_CCOMP1_ADDR,
		val,
		PMIC_RG_VCORE_CCOMP1_MASK,
		PMIC_RG_VCORE_CCOMP1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_ramp_slp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_RAMP_SLP_ADDR,
		val,
		PMIC_RG_VCORE_RAMP_SLP_MASK,
		PMIC_RG_VCORE_RAMP_SLP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_rcomp0(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_RCOMP0_ADDR,
		val,
		PMIC_RG_VGPU_RCOMP0_MASK,
		PMIC_RG_VGPU_RCOMP0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_rcomp1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_RCOMP1_ADDR,
		val,
		PMIC_RG_VGPU_RCOMP1_MASK,
		PMIC_RG_VGPU_RCOMP1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_ccomp0(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_CCOMP0_ADDR,
		val,
		PMIC_RG_VGPU_CCOMP0_MASK,
		PMIC_RG_VGPU_CCOMP0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_ccomp1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_CCOMP1_ADDR,
		val,
		PMIC_RG_VGPU_CCOMP1_MASK,
		PMIC_RG_VGPU_CCOMP1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_ramp_slp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_RAMP_SLP_ADDR,
		val,
		PMIC_RG_VGPU_RAMP_SLP_MASK,
		PMIC_RG_VGPU_RAMP_SLP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_rcs(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_RCS_ADDR,
		val,
		PMIC_RG_VCORE_RCS_MASK,
		PMIC_RG_VCORE_RCS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_rcs(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_RCS_ADDR,
		val,
		PMIC_RG_VGPU_RCS_MASK,
		PMIC_RG_VGPU_RCS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_rcb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_RCB_ADDR,
		val,
		PMIC_RG_VCORE_RCB_MASK,
		PMIC_RG_VCORE_RCB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_rcb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_RCB_ADDR,
		val,
		PMIC_RG_VGPU_RCB_MASK,
		PMIC_RG_VGPU_RCB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_csp_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_CSP_TRIM_ADDR,
		val,
		PMIC_RG_VCORE_CSP_TRIM_MASK,
		PMIC_RG_VCORE_CSP_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_csp_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_CSP_TRIM_ADDR,
		val,
		PMIC_RG_VGPU_CSP_TRIM_MASK,
		PMIC_RG_VGPU_CSP_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_csn_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_CSN_TRIM_ADDR,
		val,
		PMIC_RG_VCORE_CSN_TRIM_MASK,
		PMIC_RG_VCORE_CSN_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_csn_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_CSN_TRIM_ADDR,
		val,
		PMIC_RG_VGPU_CSN_TRIM_MASK,
		PMIC_RG_VGPU_CSN_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_zc_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_ZC_TRIM_ADDR,
		val,
		PMIC_RG_VCORE_ZC_TRIM_MASK,
		PMIC_RG_VCORE_ZC_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_zc_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_ZC_TRIM_ADDR,
		val,
		PMIC_RG_VGPU_ZC_TRIM_MASK,
		PMIC_RG_VGPU_ZC_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_nlim_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_NLIM_TRIM_ADDR,
		val,
		PMIC_RG_VCORE_NLIM_TRIM_MASK,
		PMIC_RG_VCORE_NLIM_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_nlim_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_NLIM_TRIM_ADDR,
		val,
		PMIC_RG_VGPU_NLIM_TRIM_MASK,
		PMIC_RG_VGPU_NLIM_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_rpsi1_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_RPSI1_TRIM_ADDR,
		val,
		PMIC_RG_VCORE_RPSI1_TRIM_MASK,
		PMIC_RG_VCORE_RPSI1_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_rpsi1_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_RPSI1_TRIM_ADDR,
		val,
		PMIC_RG_VGPU_RPSI1_TRIM_MASK,
		PMIC_RG_VGPU_RPSI1_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_tb_width(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_TB_WIDTH_ADDR,
		val,
		PMIC_RG_VCORE_TB_WIDTH_MASK,
		PMIC_RG_VCORE_TB_WIDTH_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_tb_width(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_TB_WIDTH_ADDR,
		val,
		PMIC_RG_VGPU_TB_WIDTH_MASK,
		PMIC_RG_VGPU_TB_WIDTH_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_ug_sr(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_UG_SR_ADDR,
		val,
		PMIC_RG_VCORE_UG_SR_MASK,
		PMIC_RG_VCORE_UG_SR_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_lg_sr(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_LG_SR_ADDR,
		val,
		PMIC_RG_VCORE_LG_SR_MASK,
		PMIC_RG_VCORE_LG_SR_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_ug_sr(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_UG_SR_ADDR,
		val,
		PMIC_RG_VGPU_UG_SR_MASK,
		PMIC_RG_VGPU_UG_SR_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_lg_sr(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_LG_SR_ADDR,
		val,
		PMIC_RG_VGPU_LG_SR_MASK,
		PMIC_RG_VGPU_LG_SR_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_pfm_ton(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_PFM_TON_ADDR,
		val,
		PMIC_RG_VCORE_PFM_TON_MASK,
		PMIC_RG_VCORE_PFM_TON_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_pfm_ton(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_PFM_TON_ADDR,
		val,
		PMIC_RG_VGPU_PFM_TON_MASK,
		PMIC_RG_VGPU_PFM_TON_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_ton_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_TON_TRIM_ADDR,
		val,
		PMIC_RG_VCORE_TON_TRIM_MASK,
		PMIC_RG_VCORE_TON_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_ton_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_TON_TRIM_ADDR,
		val,
		PMIC_RG_VGPU_TON_TRIM_MASK,
		PMIC_RG_VGPU_TON_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rgs_vcore_oc_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VCORE_OC_STATUS_ADDR,
		&val,
		PMIC_RGS_VCORE_OC_STATUS_MASK,
		PMIC_RGS_VCORE_OC_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rgs_vgpu_oc_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VGPU_OC_STATUS_ADDR,
		&val,
		PMIC_RGS_VGPU_OC_STATUS_MASK,
		PMIC_RGS_VGPU_OC_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rgs_vcorevgpu_trimok_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VCOREVGPU_TRIMOK_STATUS_ADDR,
		&val,
		PMIC_RGS_VCOREVGPU_TRIMOK_STATUS_MASK,
		PMIC_RGS_VCOREVGPU_TRIMOK_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rgs_vcorevgpu_config20_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VCOREVGPU_CONFIG20_STATUS_ADDR,
		&val,
		PMIC_RGS_VCOREVGPU_CONFIG20_STATUS_MASK,
		PMIC_RGS_VCOREVGPU_CONFIG20_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rgs_vcore_preoc_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VCORE_PREOC_STATUS_ADDR,
		&val,
		PMIC_RGS_VCORE_PREOC_STATUS_MASK,
		PMIC_RGS_VCORE_PREOC_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rgs_vcore_dig_mon(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VCORE_DIG_MON_ADDR,
		&val,
		PMIC_RGS_VCORE_DIG_MON_MASK,
		PMIC_RGS_VCORE_DIG_MON_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rgs_vgpu_dig_mon(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VGPU_DIG_MON_ADDR,
		&val,
		PMIC_RGS_VGPU_DIG_MON_MASK,
		PMIC_RGS_VGPU_DIG_MON_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vcore_tran_bst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_TRAN_BST_ADDR,
		val,
		PMIC_RG_VCORE_TRAN_BST_MASK,
		PMIC_RG_VCORE_TRAN_BST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_tran_bst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_TRAN_BST_ADDR,
		val,
		PMIC_RG_VGPU_TRAN_BST_MASK,
		PMIC_RG_VGPU_TRAN_BST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_cotramp_slp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_COTRAMP_SLP_ADDR,
		val,
		PMIC_RG_VCORE_COTRAMP_SLP_MASK,
		PMIC_RG_VCORE_COTRAMP_SLP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_cotramp_slp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_COTRAMP_SLP_ADDR,
		val,
		PMIC_RG_VGPU_COTRAMP_SLP_MASK,
		PMIC_RG_VGPU_COTRAMP_SLP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_sleep_time(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_SLEEP_TIME_ADDR,
		val,
		PMIC_RG_VCORE_SLEEP_TIME_MASK,
		PMIC_RG_VCORE_SLEEP_TIME_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_sleep_time(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_SLEEP_TIME_ADDR,
		val,
		PMIC_RG_VGPU_SLEEP_TIME_MASK,
		PMIC_RG_VGPU_SLEEP_TIME_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_vreftb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_VREFTB_ADDR,
		val,
		PMIC_RG_VCORE_VREFTB_MASK,
		PMIC_RG_VCORE_VREFTB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_vreftb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_VREFTB_ADDR,
		val,
		PMIC_RG_VGPU_VREFTB_MASK,
		PMIC_RG_VGPU_VREFTB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_csnslp_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_CSNSLP_TRIM_ADDR,
		val,
		PMIC_RG_VCORE_CSNSLP_TRIM_MASK,
		PMIC_RG_VCORE_CSNSLP_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_csnslp_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_CSNSLP_TRIM_ADDR,
		val,
		PMIC_RG_VGPU_CSNSLP_TRIM_MASK,
		PMIC_RG_VGPU_CSNSLP_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_cspslp_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_CSPSLP_TRIM_ADDR,
		val,
		PMIC_RG_VCORE_CSPSLP_TRIM_MASK,
		PMIC_RG_VCORE_CSPSLP_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_cspslp_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_CSPSLP_TRIM_ADDR,
		val,
		PMIC_RG_VGPU_CSPSLP_TRIM_MASK,
		PMIC_RG_VGPU_CSPSLP_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_fugon(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_FUGON_ADDR,
		val,
		PMIC_RG_VCORE_FUGON_MASK,
		PMIC_RG_VCORE_FUGON_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_fugon(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_FUGON_ADDR,
		val,
		PMIC_RG_VGPU_FUGON_MASK,
		PMIC_RG_VGPU_FUGON_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_flgon(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_FLGON_ADDR,
		val,
		PMIC_RG_VCORE_FLGON_MASK,
		PMIC_RG_VCORE_FLGON_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_flgon(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_FLGON_ADDR,
		val,
		PMIC_RG_VGPU_FLGON_MASK,
		PMIC_RG_VGPU_FLGON_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_preoc_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_PREOC_TRIM_ADDR,
		val,
		PMIC_RG_VCORE_PREOC_TRIM_MASK,
		PMIC_RG_VCORE_PREOC_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_RSV_ADDR,
		val,
		PMIC_RG_VCORE_RSV_MASK,
		PMIC_RG_VCORE_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vgpu_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_RSV_ADDR,
		val,
		PMIC_RG_VGPU_RSV_MASK,
		PMIC_RG_VGPU_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vcore_nonaudible_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCORE_NONAUDIBLE_EN_ADDR,
		val,
		PMIC_RG_VCORE_NONAUDIBLE_EN_MASK,
		PMIC_RG_VCORE_NONAUDIBLE_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vcore_nonaudible_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VCORE_NONAUDIBLE_EN_ADDR,
		&val,
		PMIC_RG_VCORE_NONAUDIBLE_EN_MASK,
		PMIC_RG_VCORE_NONAUDIBLE_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vgpu_nonaudible_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VGPU_NONAUDIBLE_EN_ADDR,
		val,
		PMIC_RG_VGPU_NONAUDIBLE_EN_MASK,
		PMIC_RG_VGPU_NONAUDIBLE_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vgpu_nonaudible_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VGPU_NONAUDIBLE_EN_ADDR,
		&val,
		PMIC_RG_VGPU_NONAUDIBLE_EN_MASK,
		PMIC_RG_VGPU_NONAUDIBLE_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vcorevgpu_disautok(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VCOREVGPU_DISAUTOK_ADDR,
		val,
		PMIC_RG_VCOREVGPU_DISAUTOK_MASK,
		PMIC_RG_VCOREVGPU_DISAUTOK_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_fcot(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_FCOT_ADDR,
		val,
		PMIC_RG_VDRAM1_FCOT_MASK,
		PMIC_RG_VDRAM1_FCOT_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_rcomp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_RCOMP_ADDR,
		val,
		PMIC_RG_VDRAM1_RCOMP_MASK,
		PMIC_RG_VDRAM1_RCOMP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_tb_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_TB_DIS_ADDR,
		val,
		PMIC_RG_VDRAM1_TB_DIS_MASK,
		PMIC_RG_VDRAM1_TB_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_dispg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_DISPG_ADDR,
		val,
		PMIC_RG_VDRAM1_DISPG_MASK,
		PMIC_RG_VDRAM1_DISPG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_fpwm(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_FPWM_ADDR,
		val,
		PMIC_RG_VDRAM1_FPWM_MASK,
		PMIC_RG_VDRAM1_FPWM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_zc_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_ZC_TRIM_ADDR,
		val,
		PMIC_RG_VDRAM1_ZC_TRIM_MASK,
		PMIC_RG_VDRAM1_ZC_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_nlim_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_NLIM_TRIM_ADDR,
		val,
		PMIC_RG_VDRAM1_NLIM_TRIM_MASK,
		PMIC_RG_VDRAM1_NLIM_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_pfm_ton(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_PFM_TON_ADDR,
		val,
		PMIC_RG_VDRAM1_PFM_TON_MASK,
		PMIC_RG_VDRAM1_PFM_TON_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_pwmramp_slp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_PWMRAMP_SLP_ADDR,
		val,
		PMIC_RG_VDRAM1_PWMRAMP_SLP_MASK,
		PMIC_RG_VDRAM1_PWMRAMP_SLP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_cotramp_slp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_COTRAMP_SLP_ADDR,
		val,
		PMIC_RG_VDRAM1_COTRAMP_SLP_MASK,
		PMIC_RG_VDRAM1_COTRAMP_SLP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_rcs(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_RCS_ADDR,
		val,
		PMIC_RG_VDRAM1_RCS_MASK,
		PMIC_RG_VDRAM1_RCS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_csn_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_CSN_TRIM_ADDR,
		val,
		PMIC_RG_VDRAM1_CSN_TRIM_MASK,
		PMIC_RG_VDRAM1_CSN_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_csp_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_CSP_TRIM_ADDR,
		val,
		PMIC_RG_VDRAM1_CSP_TRIM_MASK,
		PMIC_RG_VDRAM1_CSP_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_rpsi_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_RPSI_TRIM_ADDR,
		val,
		PMIC_RG_VDRAM1_RPSI_TRIM_MASK,
		PMIC_RG_VDRAM1_RPSI_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_sleep_time(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_SLEEP_TIME_ADDR,
		val,
		PMIC_RG_VDRAM1_SLEEP_TIME_MASK,
		PMIC_RG_VDRAM1_SLEEP_TIME_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_nlim_gating(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_NLIM_GATING_ADDR,
		val,
		PMIC_RG_VDRAM1_NLIM_GATING_MASK,
		PMIC_RG_VDRAM1_NLIM_GATING_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_ton_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_TON_TRIM_ADDR,
		val,
		PMIC_RG_VDRAM1_TON_TRIM_MASK,
		PMIC_RG_VDRAM1_TON_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_vdiff_off(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_VDIFF_OFF_ADDR,
		val,
		PMIC_RG_VDRAM1_VDIFF_OFF_MASK,
		PMIC_RG_VDRAM1_VDIFF_OFF_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_vrefup(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_VREFUP_ADDR,
		val,
		PMIC_RG_VDRAM1_VREFUP_MASK,
		PMIC_RG_VDRAM1_VREFUP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_tb_width(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_TB_WIDTH_ADDR,
		val,
		PMIC_RG_VDRAM1_TB_WIDTH_MASK,
		PMIC_RG_VDRAM1_TB_WIDTH_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_ug_sr(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_UG_SR_ADDR,
		val,
		PMIC_RG_VDRAM1_UG_SR_MASK,
		PMIC_RG_VDRAM1_UG_SR_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_lg_sr(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_LG_SR_ADDR,
		val,
		PMIC_RG_VDRAM1_LG_SR_MASK,
		PMIC_RG_VDRAM1_LG_SR_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_ccomp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_CCOMP_ADDR,
		val,
		PMIC_RG_VDRAM1_CCOMP_MASK,
		PMIC_RG_VDRAM1_CCOMP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_ndis_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_NDIS_EN_ADDR,
		val,
		PMIC_RG_VDRAM1_NDIS_EN_MASK,
		PMIC_RG_VDRAM1_NDIS_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vdram1_ndis_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VDRAM1_NDIS_EN_ADDR,
		&val,
		PMIC_RG_VDRAM1_NDIS_EN_MASK,
		PMIC_RG_VDRAM1_NDIS_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vdram1_tmdl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_TMDL_ADDR,
		val,
		PMIC_RG_VDRAM1_TMDL_MASK,
		PMIC_RG_VDRAM1_TMDL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_RSV_ADDR,
		val,
		PMIC_RG_VDRAM1_RSV_MASK,
		PMIC_RG_VDRAM1_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_csnslp_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_CSNSLP_TRIM_ADDR,
		val,
		PMIC_RG_VDRAM1_CSNSLP_TRIM_MASK,
		PMIC_RG_VDRAM1_CSNSLP_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_cspslp_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_CSPSLP_TRIM_ADDR,
		val,
		PMIC_RG_VDRAM1_CSPSLP_TRIM_MASK,
		PMIC_RG_VDRAM1_CSPSLP_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_fugon(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_FUGON_ADDR,
		val,
		PMIC_RG_VDRAM1_FUGON_MASK,
		PMIC_RG_VDRAM1_FUGON_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_flgon(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_FLGON_ADDR,
		val,
		PMIC_RG_VDRAM1_FLGON_MASK,
		PMIC_RG_VDRAM1_FLGON_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram1_nonaudible_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_NONAUDIBLE_EN_ADDR,
		val,
		PMIC_RG_VDRAM1_NONAUDIBLE_EN_MASK,
		PMIC_RG_VDRAM1_NONAUDIBLE_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vdram1_nonaudible_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VDRAM1_NONAUDIBLE_EN_ADDR,
		&val,
		PMIC_RG_VDRAM1_NONAUDIBLE_EN_MASK,
		PMIC_RG_VDRAM1_NONAUDIBLE_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vdram1_vdiffpfm_off(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_VDIFFPFM_OFF_ADDR,
		val,
		PMIC_RG_VDRAM1_VDIFFPFM_OFF_MASK,
		PMIC_RG_VDRAM1_VDIFFPFM_OFF_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rgs_vdram1_oc_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VDRAM1_OC_STATUS_ADDR,
		&val,
		PMIC_RGS_VDRAM1_OC_STATUS_MASK,
		PMIC_RGS_VDRAM1_OC_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rgs_vdram1_enpwm_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VDRAM1_ENPWM_STATUS_ADDR,
		&val,
		PMIC_RGS_VDRAM1_ENPWM_STATUS_MASK,
		PMIC_RGS_VDRAM1_ENPWM_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vdram1_disautok(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM1_DISAUTOK_ADDR,
		val,
		PMIC_RG_VDRAM1_DISAUTOK_MASK,
		PMIC_RG_VDRAM1_DISAUTOK_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rgs_vdram1_trimok_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VDRAM1_TRIMOK_STATUS_ADDR,
		&val,
		PMIC_RGS_VDRAM1_TRIMOK_STATUS_MASK,
		PMIC_RGS_VDRAM1_TRIMOK_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rgs_vdram1_dig_mon(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VDRAM1_DIG_MON_ADDR,
		&val,
		PMIC_RGS_VDRAM1_DIG_MON_MASK,
		PMIC_RGS_VDRAM1_DIG_MON_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vdram2_fcot(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_FCOT_ADDR,
		val,
		PMIC_RG_VDRAM2_FCOT_MASK,
		PMIC_RG_VDRAM2_FCOT_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_rcomp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_RCOMP_ADDR,
		val,
		PMIC_RG_VDRAM2_RCOMP_MASK,
		PMIC_RG_VDRAM2_RCOMP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_tb_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_TB_DIS_ADDR,
		val,
		PMIC_RG_VDRAM2_TB_DIS_MASK,
		PMIC_RG_VDRAM2_TB_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_dispg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_DISPG_ADDR,
		val,
		PMIC_RG_VDRAM2_DISPG_MASK,
		PMIC_RG_VDRAM2_DISPG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_fpwm(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_FPWM_ADDR,
		val,
		PMIC_RG_VDRAM2_FPWM_MASK,
		PMIC_RG_VDRAM2_FPWM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_zc_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_ZC_TRIM_ADDR,
		val,
		PMIC_RG_VDRAM2_ZC_TRIM_MASK,
		PMIC_RG_VDRAM2_ZC_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_nlim_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_NLIM_TRIM_ADDR,
		val,
		PMIC_RG_VDRAM2_NLIM_TRIM_MASK,
		PMIC_RG_VDRAM2_NLIM_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_pfm_ton(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_PFM_TON_ADDR,
		val,
		PMIC_RG_VDRAM2_PFM_TON_MASK,
		PMIC_RG_VDRAM2_PFM_TON_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_pwmramp_slp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_PWMRAMP_SLP_ADDR,
		val,
		PMIC_RG_VDRAM2_PWMRAMP_SLP_MASK,
		PMIC_RG_VDRAM2_PWMRAMP_SLP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_cotramp_slp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_COTRAMP_SLP_ADDR,
		val,
		PMIC_RG_VDRAM2_COTRAMP_SLP_MASK,
		PMIC_RG_VDRAM2_COTRAMP_SLP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_rcs(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_RCS_ADDR,
		val,
		PMIC_RG_VDRAM2_RCS_MASK,
		PMIC_RG_VDRAM2_RCS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_csn_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_CSN_TRIM_ADDR,
		val,
		PMIC_RG_VDRAM2_CSN_TRIM_MASK,
		PMIC_RG_VDRAM2_CSN_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_csp_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_CSP_TRIM_ADDR,
		val,
		PMIC_RG_VDRAM2_CSP_TRIM_MASK,
		PMIC_RG_VDRAM2_CSP_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_rpsi_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_RPSI_TRIM_ADDR,
		val,
		PMIC_RG_VDRAM2_RPSI_TRIM_MASK,
		PMIC_RG_VDRAM2_RPSI_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_sleep_time(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_SLEEP_TIME_ADDR,
		val,
		PMIC_RG_VDRAM2_SLEEP_TIME_MASK,
		PMIC_RG_VDRAM2_SLEEP_TIME_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_nlim_gating(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_NLIM_GATING_ADDR,
		val,
		PMIC_RG_VDRAM2_NLIM_GATING_MASK,
		PMIC_RG_VDRAM2_NLIM_GATING_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_ton_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_TON_TRIM_ADDR,
		val,
		PMIC_RG_VDRAM2_TON_TRIM_MASK,
		PMIC_RG_VDRAM2_TON_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_vdiff_off(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_VDIFF_OFF_ADDR,
		val,
		PMIC_RG_VDRAM2_VDIFF_OFF_MASK,
		PMIC_RG_VDRAM2_VDIFF_OFF_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_vrefup(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_VREFUP_ADDR,
		val,
		PMIC_RG_VDRAM2_VREFUP_MASK,
		PMIC_RG_VDRAM2_VREFUP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_tb_width(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_TB_WIDTH_ADDR,
		val,
		PMIC_RG_VDRAM2_TB_WIDTH_MASK,
		PMIC_RG_VDRAM2_TB_WIDTH_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_ug_sr(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_UG_SR_ADDR,
		val,
		PMIC_RG_VDRAM2_UG_SR_MASK,
		PMIC_RG_VDRAM2_UG_SR_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_lg_sr(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_LG_SR_ADDR,
		val,
		PMIC_RG_VDRAM2_LG_SR_MASK,
		PMIC_RG_VDRAM2_LG_SR_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_ccomp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_CCOMP_ADDR,
		val,
		PMIC_RG_VDRAM2_CCOMP_MASK,
		PMIC_RG_VDRAM2_CCOMP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_ndis_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_NDIS_EN_ADDR,
		val,
		PMIC_RG_VDRAM2_NDIS_EN_MASK,
		PMIC_RG_VDRAM2_NDIS_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vdram2_ndis_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VDRAM2_NDIS_EN_ADDR,
		&val,
		PMIC_RG_VDRAM2_NDIS_EN_MASK,
		PMIC_RG_VDRAM2_NDIS_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vdram2_tmdl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_TMDL_ADDR,
		val,
		PMIC_RG_VDRAM2_TMDL_MASK,
		PMIC_RG_VDRAM2_TMDL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_RSV_ADDR,
		val,
		PMIC_RG_VDRAM2_RSV_MASK,
		PMIC_RG_VDRAM2_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_csnslp_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_CSNSLP_TRIM_ADDR,
		val,
		PMIC_RG_VDRAM2_CSNSLP_TRIM_MASK,
		PMIC_RG_VDRAM2_CSNSLP_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_cspslp_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_CSPSLP_TRIM_ADDR,
		val,
		PMIC_RG_VDRAM2_CSPSLP_TRIM_MASK,
		PMIC_RG_VDRAM2_CSPSLP_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_fugon(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_FUGON_ADDR,
		val,
		PMIC_RG_VDRAM2_FUGON_MASK,
		PMIC_RG_VDRAM2_FUGON_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_flgon(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_FLGON_ADDR,
		val,
		PMIC_RG_VDRAM2_FLGON_MASK,
		PMIC_RG_VDRAM2_FLGON_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vdram2_nonaudible_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_NONAUDIBLE_EN_ADDR,
		val,
		PMIC_RG_VDRAM2_NONAUDIBLE_EN_MASK,
		PMIC_RG_VDRAM2_NONAUDIBLE_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vdram2_nonaudible_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VDRAM2_NONAUDIBLE_EN_ADDR,
		&val,
		PMIC_RG_VDRAM2_NONAUDIBLE_EN_MASK,
		PMIC_RG_VDRAM2_NONAUDIBLE_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vdram2_vdiffpfm_off(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_VDIFFPFM_OFF_ADDR,
		val,
		PMIC_RG_VDRAM2_VDIFFPFM_OFF_MASK,
		PMIC_RG_VDRAM2_VDIFFPFM_OFF_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rgs_vdram2_oc_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VDRAM2_OC_STATUS_ADDR,
		&val,
		PMIC_RGS_VDRAM2_OC_STATUS_MASK,
		PMIC_RGS_VDRAM2_OC_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rgs_vdram2_enpwm_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VDRAM2_ENPWM_STATUS_ADDR,
		&val,
		PMIC_RGS_VDRAM2_ENPWM_STATUS_MASK,
		PMIC_RGS_VDRAM2_ENPWM_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vdram2_disautok(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VDRAM2_DISAUTOK_ADDR,
		val,
		PMIC_RG_VDRAM2_DISAUTOK_MASK,
		PMIC_RG_VDRAM2_DISAUTOK_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rgs_vdram2_trimok_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VDRAM2_TRIMOK_STATUS_ADDR,
		&val,
		PMIC_RGS_VDRAM2_TRIMOK_STATUS_MASK,
		PMIC_RGS_VDRAM2_TRIMOK_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rgs_vdram2_dig_mon(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VDRAM2_DIG_MON_ADDR,
		&val,
		PMIC_RGS_VDRAM2_DIG_MON_MASK,
		PMIC_RGS_VDRAM2_DIG_MON_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vmodem_modeset(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_MODESET_ADDR,
		val,
		PMIC_RG_VMODEM_MODESET_MASK,
		PMIC_RG_VMODEM_MODESET_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_ndis_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_NDIS_EN_ADDR,
		val,
		PMIC_RG_VMODEM_NDIS_EN_MASK,
		PMIC_RG_VMODEM_NDIS_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vmodem_ndis_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VMODEM_NDIS_EN_ADDR,
		&val,
		PMIC_RG_VMODEM_NDIS_EN_MASK,
		PMIC_RG_VMODEM_NDIS_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vmodem_vrf18_sstart_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_VRF18_SSTART_EN_ADDR,
		val,
		PMIC_RG_VMODEM_VRF18_SSTART_EN_MASK,
		PMIC_RG_VMODEM_VRF18_SSTART_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vmodem_vrf18_sstart_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VMODEM_VRF18_SSTART_EN_ADDR,
		&val,
		PMIC_RG_VMODEM_VRF18_SSTART_EN_MASK,
		PMIC_RG_VMODEM_VRF18_SSTART_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vmodem_auto_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_AUTO_MODE_ADDR,
		val,
		PMIC_RG_VMODEM_AUTO_MODE_MASK,
		PMIC_RG_VMODEM_AUTO_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_rzsel0(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_RZSEL0_ADDR,
		val,
		PMIC_RG_VMODEM_RZSEL0_MASK,
		PMIC_RG_VMODEM_RZSEL0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_rzsel1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_RZSEL1_ADDR,
		val,
		PMIC_RG_VMODEM_RZSEL1_MASK,
		PMIC_RG_VMODEM_RZSEL1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_ccsel0(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_CCSEL0_ADDR,
		val,
		PMIC_RG_VMODEM_CCSEL0_MASK,
		PMIC_RG_VMODEM_CCSEL0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_ccsel1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_CCSEL1_ADDR,
		val,
		PMIC_RG_VMODEM_CCSEL1_MASK,
		PMIC_RG_VMODEM_CCSEL1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_csl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_CSL_ADDR,
		val,
		PMIC_RG_VMODEM_CSL_MASK,
		PMIC_RG_VMODEM_CSL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_slp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_SLP_ADDR,
		val,
		PMIC_RG_VMODEM_SLP_MASK,
		PMIC_RG_VMODEM_SLP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_adrc_fen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_ADRC_FEN_ADDR,
		val,
		PMIC_RG_VMODEM_ADRC_FEN_MASK,
		PMIC_RG_VMODEM_ADRC_FEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_vc_cap_clamp_fen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_VC_CAP_CLAMP_FEN_ADDR,
		val,
		PMIC_RG_VMODEM_VC_CAP_CLAMP_FEN_MASK,
		PMIC_RG_VMODEM_VC_CAP_CLAMP_FEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_vc_clamp_fen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_VC_CLAMP_FEN_ADDR,
		val,
		PMIC_RG_VMODEM_VC_CLAMP_FEN_MASK,
		PMIC_RG_VMODEM_VC_CLAMP_FEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_burst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_BURST_ADDR,
		val,
		PMIC_RG_VMODEM_BURST_MASK,
		PMIC_RG_VMODEM_BURST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_csr(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_CSR_ADDR,
		val,
		PMIC_RG_VMODEM_CSR_MASK,
		PMIC_RG_VMODEM_CSR_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_zxos_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_ZXOS_TRIM_ADDR,
		val,
		PMIC_RG_VMODEM_ZXOS_TRIM_MASK,
		PMIC_RG_VMODEM_ZXOS_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_pfmsr_eh(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_PFMSR_EH_ADDR,
		val,
		PMIC_RG_VMODEM_PFMSR_EH_MASK,
		PMIC_RG_VMODEM_PFMSR_EH_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_nlim_gating(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_NLIM_GATING_ADDR,
		val,
		PMIC_RG_VMODEM_NLIM_GATING_MASK,
		PMIC_RG_VMODEM_NLIM_GATING_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_pwmsr_eh(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_PWMSR_EH_ADDR,
		val,
		PMIC_RG_VMODEM_PWMSR_EH_MASK,
		PMIC_RG_VMODEM_PWMSR_EH_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_hs_vthdet(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_HS_VTHDET_ADDR,
		val,
		PMIC_RG_VMODEM_HS_VTHDET_MASK,
		PMIC_RG_VMODEM_HS_VTHDET_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_pg_gating(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_PG_GATING_ADDR,
		val,
		PMIC_RG_VMODEM_PG_GATING_MASK,
		PMIC_RG_VMODEM_PG_GATING_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_hs_onspeed_eh(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_HS_ONSPEED_EH_ADDR,
		val,
		PMIC_RG_VMODEM_HS_ONSPEED_EH_MASK,
		PMIC_RG_VMODEM_HS_ONSPEED_EH_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_nlim_trimming(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_NLIM_TRIMMING_ADDR,
		val,
		PMIC_RG_VMODEM_NLIM_TRIMMING_MASK,
		PMIC_RG_VMODEM_NLIM_TRIMMING_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_sr_p(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_SR_P_ADDR,
		val,
		PMIC_RG_VMODEM_SR_P_MASK,
		PMIC_RG_VMODEM_SR_P_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_sr_n(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_SR_N_ADDR,
		val,
		PMIC_RG_VMODEM_SR_N_MASK,
		PMIC_RG_VMODEM_SR_N_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_pfm_rip(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_PFM_RIP_ADDR,
		val,
		PMIC_RG_VMODEM_PFM_RIP_MASK,
		PMIC_RG_VMODEM_PFM_RIP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_tran_bst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_TRAN_BST_ADDR,
		val,
		PMIC_RG_VMODEM_TRAN_BST_MASK,
		PMIC_RG_VMODEM_TRAN_BST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_dts_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_DTS_ENB_ADDR,
		val,
		PMIC_RG_VMODEM_DTS_ENB_MASK,
		PMIC_RG_VMODEM_DTS_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vmodem_dts_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VMODEM_DTS_ENB_ADDR,
		&val,
		PMIC_RG_VMODEM_DTS_ENB_MASK,
		PMIC_RG_VMODEM_DTS_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vmodem_min_off(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_MIN_OFF_ADDR,
		val,
		PMIC_RG_VMODEM_MIN_OFF_MASK,
		PMIC_RG_VMODEM_MIN_OFF_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_1p35up_sel_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_1P35UP_SEL_EN_ADDR,
		val,
		PMIC_RG_VMODEM_1P35UP_SEL_EN_MASK,
		PMIC_RG_VMODEM_1P35UP_SEL_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vmodem_1p35up_sel_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VMODEM_1P35UP_SEL_EN_ADDR,
		&val,
		PMIC_RG_VMODEM_1P35UP_SEL_EN_MASK,
		PMIC_RG_VMODEM_1P35UP_SEL_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vmodem_dlc_auto_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_DLC_AUTO_MODE_ADDR,
		val,
		PMIC_RG_VMODEM_DLC_AUTO_MODE_MASK,
		PMIC_RG_VMODEM_DLC_AUTO_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_src_auto_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_SRC_AUTO_MODE_ADDR,
		val,
		PMIC_RG_VMODEM_SRC_AUTO_MODE_MASK,
		PMIC_RG_VMODEM_SRC_AUTO_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_ugp_sr(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_UGP_SR_ADDR,
		val,
		PMIC_RG_VMODEM_UGP_SR_MASK,
		PMIC_RG_VMODEM_UGP_SR_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_lgp_sr(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_LGP_SR_ADDR,
		val,
		PMIC_RG_VMODEM_LGP_SR_MASK,
		PMIC_RG_VMODEM_LGP_SR_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_ugp_sr_pfm(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_UGP_SR_PFM_ADDR,
		val,
		PMIC_RG_VMODEM_UGP_SR_PFM_MASK,
		PMIC_RG_VMODEM_UGP_SR_PFM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_lgp_sr_pfm(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_LGP_SR_PFM_ADDR,
		val,
		PMIC_RG_VMODEM_LGP_SR_PFM_MASK,
		PMIC_RG_VMODEM_LGP_SR_PFM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_ugd_vthsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_UGD_VTHSEL_ADDR,
		val,
		PMIC_RG_VMODEM_UGD_VTHSEL_MASK,
		PMIC_RG_VMODEM_UGD_VTHSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_fnlx(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_FNLX_ADDR,
		val,
		PMIC_RG_VMODEM_FNLX_MASK,
		PMIC_RG_VMODEM_FNLX_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_vdiff_enlowiq(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_VDIFF_ENLOWIQ_ADDR,
		val,
		PMIC_RG_VMODEM_VDIFF_ENLOWIQ_MASK,
		PMIC_RG_VMODEM_VDIFF_ENLOWIQ_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vmodem_vdiff_enlowiq(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VMODEM_VDIFF_ENLOWIQ_ADDR,
		&val,
		PMIC_RG_VMODEM_VDIFF_ENLOWIQ_MASK,
		PMIC_RG_VMODEM_VDIFF_ENLOWIQ_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vmodem_pfmoc_fwupoff(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_PFMOC_FWUPOFF_ADDR,
		val,
		PMIC_RG_VMODEM_PFMOC_FWUPOFF_MASK,
		PMIC_RG_VMODEM_PFMOC_FWUPOFF_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_pwmoc_fwupoff(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_PWMOC_FWUPOFF_ADDR,
		val,
		PMIC_RG_VMODEM_PWMOC_FWUPOFF_MASK,
		PMIC_RG_VMODEM_PWMOC_FWUPOFF_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_cp_fwupoff(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_CP_FWUPOFF_ADDR,
		val,
		PMIC_RG_VMODEM_CP_FWUPOFF_MASK,
		PMIC_RG_VMODEM_CP_FWUPOFF_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_zx_gating(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_ZX_GATING_ADDR,
		val,
		PMIC_RG_VMODEM_ZX_GATING_MASK,
		PMIC_RG_VMODEM_ZX_GATING_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_RSV_ADDR,
		val,
		PMIC_RG_VMODEM_RSV_MASK,
		PMIC_RG_VMODEM_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_azc_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_AZC_EN_ADDR,
		val,
		PMIC_RG_VMODEM_AZC_EN_MASK,
		PMIC_RG_VMODEM_AZC_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vmodem_azc_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VMODEM_AZC_EN_ADDR,
		&val,
		PMIC_RG_VMODEM_AZC_EN_MASK,
		PMIC_RG_VMODEM_AZC_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vmodem_azc_delay(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_AZC_DELAY_ADDR,
		val,
		PMIC_RG_VMODEM_AZC_DELAY_MASK,
		PMIC_RG_VMODEM_AZC_DELAY_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_azc_hold_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_AZC_HOLD_ENB_ADDR,
		val,
		PMIC_RG_VMODEM_AZC_HOLD_ENB_MASK,
		PMIC_RG_VMODEM_AZC_HOLD_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vmodem_azc_hold_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VMODEM_AZC_HOLD_ENB_ADDR,
		&val,
		PMIC_RG_VMODEM_AZC_HOLD_ENB_MASK,
		PMIC_RG_VMODEM_AZC_HOLD_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rgs_vmodem_oc_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VMODEM_OC_STATUS_ADDR,
		&val,
		PMIC_RGS_VMODEM_OC_STATUS_MASK,
		PMIC_RGS_VMODEM_OC_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rgs_vmodem_dig_mon(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VMODEM_DIG_MON_ADDR,
		&val,
		PMIC_RGS_VMODEM_DIG_MON_MASK,
		PMIC_RGS_VMODEM_DIG_MON_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rgs_vmodem_enpwm_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VMODEM_ENPWM_STATUS_ADDR,
		&val,
		PMIC_RGS_VMODEM_ENPWM_STATUS_MASK,
		PMIC_RGS_VMODEM_ENPWM_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vmodem_iodetect_en18(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_IODETECT_EN18_ADDR,
		val,
		PMIC_RG_VMODEM_IODETECT_EN18_MASK,
		PMIC_RG_VMODEM_IODETECT_EN18_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vmodem_iodetect_en18(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VMODEM_IODETECT_EN18_ADDR,
		&val,
		PMIC_RG_VMODEM_IODETECT_EN18_MASK,
		PMIC_RG_VMODEM_IODETECT_EN18_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vmodem_preoc_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_PREOC_SEL_ADDR,
		val,
		PMIC_RG_VMODEM_PREOC_SEL_MASK,
		PMIC_RG_VMODEM_PREOC_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vmodem_nonaudible_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VMODEM_NONAUDIBLE_EN_ADDR,
		val,
		PMIC_RG_VMODEM_NONAUDIBLE_EN_MASK,
		PMIC_RG_VMODEM_NONAUDIBLE_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vmodem_nonaudible_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VMODEM_NONAUDIBLE_EN_ADDR,
		&val,
		PMIC_RG_VMODEM_NONAUDIBLE_EN_MASK,
		PMIC_RG_VMODEM_NONAUDIBLE_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vs1_min_off(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS1_MIN_OFF_ADDR,
		val,
		PMIC_RG_VS1_MIN_OFF_MASK,
		PMIC_RG_VS1_MIN_OFF_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs1_vrf18_sstart_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS1_VRF18_SSTART_EN_ADDR,
		val,
		PMIC_RG_VS1_VRF18_SSTART_EN_MASK,
		PMIC_RG_VS1_VRF18_SSTART_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vs1_vrf18_sstart_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VS1_VRF18_SSTART_EN_ADDR,
		&val,
		PMIC_RG_VS1_VRF18_SSTART_EN_MASK,
		PMIC_RG_VS1_VRF18_SSTART_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vs1_1p35up_sel_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS1_1P35UP_SEL_EN_ADDR,
		val,
		PMIC_RG_VS1_1P35UP_SEL_EN_MASK,
		PMIC_RG_VS1_1P35UP_SEL_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vs1_1p35up_sel_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VS1_1P35UP_SEL_EN_ADDR,
		&val,
		PMIC_RG_VS1_1P35UP_SEL_EN_MASK,
		PMIC_RG_VS1_1P35UP_SEL_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vs1_rzsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS1_RZSEL_ADDR,
		val,
		PMIC_RG_VS1_RZSEL_MASK,
		PMIC_RG_VS1_RZSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs1_csr(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS1_CSR_ADDR,
		val,
		PMIC_RG_VS1_CSR_MASK,
		PMIC_RG_VS1_CSR_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs1_csl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS1_CSL_ADDR,
		val,
		PMIC_RG_VS1_CSL_MASK,
		PMIC_RG_VS1_CSL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs1_slp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS1_SLP_ADDR,
		val,
		PMIC_RG_VS1_SLP_MASK,
		PMIC_RG_VS1_SLP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs1_zx_os(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS1_ZX_OS_ADDR,
		val,
		PMIC_RG_VS1_ZX_OS_MASK,
		PMIC_RG_VS1_ZX_OS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs1_ndis_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS1_NDIS_EN_ADDR,
		val,
		PMIC_RG_VS1_NDIS_EN_MASK,
		PMIC_RG_VS1_NDIS_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vs1_ndis_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VS1_NDIS_EN_ADDR,
		&val,
		PMIC_RG_VS1_NDIS_EN_MASK,
		PMIC_RG_VS1_NDIS_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vs1_csm_n(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS1_CSM_N_ADDR,
		val,
		PMIC_RG_VS1_CSM_N_MASK,
		PMIC_RG_VS1_CSM_N_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs1_csm_p(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS1_CSM_P_ADDR,
		val,
		PMIC_RG_VS1_CSM_P_MASK,
		PMIC_RG_VS1_CSM_P_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs1_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS1_RSV_ADDR,
		val,
		PMIC_RG_VS1_RSV_MASK,
		PMIC_RG_VS1_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs1_zxos_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS1_ZXOS_TRIM_ADDR,
		val,
		PMIC_RG_VS1_ZXOS_TRIM_MASK,
		PMIC_RG_VS1_ZXOS_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs1_modeset(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS1_MODESET_ADDR,
		val,
		PMIC_RG_VS1_MODESET_MASK,
		PMIC_RG_VS1_MODESET_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs1_pfm_rip(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS1_PFM_RIP_ADDR,
		val,
		PMIC_RG_VS1_PFM_RIP_MASK,
		PMIC_RG_VS1_PFM_RIP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs1_tran_bst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS1_TRAN_BST_ADDR,
		val,
		PMIC_RG_VS1_TRAN_BST_MASK,
		PMIC_RG_VS1_TRAN_BST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs1_dts_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS1_DTS_ENB_ADDR,
		val,
		PMIC_RG_VS1_DTS_ENB_MASK,
		PMIC_RG_VS1_DTS_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vs1_dts_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VS1_DTS_ENB_ADDR,
		&val,
		PMIC_RG_VS1_DTS_ENB_MASK,
		PMIC_RG_VS1_DTS_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vs1_auto_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS1_AUTO_MODE_ADDR,
		val,
		PMIC_RG_VS1_AUTO_MODE_MASK,
		PMIC_RG_VS1_AUTO_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs1_pwm_trig(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS1_PWM_TRIG_ADDR,
		val,
		PMIC_RG_VS1_PWM_TRIG_MASK,
		PMIC_RG_VS1_PWM_TRIG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs1_rsv_h(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS1_RSV_H_ADDR,
		val,
		PMIC_RG_VS1_RSV_H_MASK,
		PMIC_RG_VS1_RSV_H_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs1_rsv_l(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS1_RSV_L_ADDR,
		val,
		PMIC_RG_VS1_RSV_L_MASK,
		PMIC_RG_VS1_RSV_L_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs1_nonaudible_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS1_NONAUDIBLE_EN_ADDR,
		val,
		PMIC_RG_VS1_NONAUDIBLE_EN_MASK,
		PMIC_RG_VS1_NONAUDIBLE_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vs1_nonaudible_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VS1_NONAUDIBLE_EN_ADDR,
		&val,
		PMIC_RG_VS1_NONAUDIBLE_EN_MASK,
		PMIC_RG_VS1_NONAUDIBLE_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vs1_sr_p(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS1_SR_P_ADDR,
		val,
		PMIC_RG_VS1_SR_P_MASK,
		PMIC_RG_VS1_SR_P_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs1_sr_n(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS1_SR_N_ADDR,
		val,
		PMIC_RG_VS1_SR_N_MASK,
		PMIC_RG_VS1_SR_N_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs1_burst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS1_BURST_ADDR,
		val,
		PMIC_RG_VS1_BURST_MASK,
		PMIC_RG_VS1_BURST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rgs_vs1_enpwm_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VS1_ENPWM_STATUS_ADDR,
		&val,
		PMIC_RGS_VS1_ENPWM_STATUS_MASK,
		PMIC_RGS_VS1_ENPWM_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rgs_vs1_oc_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VS1_OC_STATUS_ADDR,
		&val,
		PMIC_RGS_VS1_OC_STATUS_MASK,
		PMIC_RGS_VS1_OC_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rgs_vs1_dig_mon(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VS1_DIG_MON_ADDR,
		&val,
		PMIC_RGS_VS1_DIG_MON_MASK,
		PMIC_RGS_VS1_DIG_MON_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vs2_min_off(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS2_MIN_OFF_ADDR,
		val,
		PMIC_RG_VS2_MIN_OFF_MASK,
		PMIC_RG_VS2_MIN_OFF_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs2_vrf18_sstart_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS2_VRF18_SSTART_EN_ADDR,
		val,
		PMIC_RG_VS2_VRF18_SSTART_EN_MASK,
		PMIC_RG_VS2_VRF18_SSTART_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vs2_vrf18_sstart_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VS2_VRF18_SSTART_EN_ADDR,
		&val,
		PMIC_RG_VS2_VRF18_SSTART_EN_MASK,
		PMIC_RG_VS2_VRF18_SSTART_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vs2_1p35up_sel_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS2_1P35UP_SEL_EN_ADDR,
		val,
		PMIC_RG_VS2_1P35UP_SEL_EN_MASK,
		PMIC_RG_VS2_1P35UP_SEL_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vs2_1p35up_sel_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VS2_1P35UP_SEL_EN_ADDR,
		&val,
		PMIC_RG_VS2_1P35UP_SEL_EN_MASK,
		PMIC_RG_VS2_1P35UP_SEL_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vs2_rzsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS2_RZSEL_ADDR,
		val,
		PMIC_RG_VS2_RZSEL_MASK,
		PMIC_RG_VS2_RZSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs2_csr(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS2_CSR_ADDR,
		val,
		PMIC_RG_VS2_CSR_MASK,
		PMIC_RG_VS2_CSR_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs2_csl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS2_CSL_ADDR,
		val,
		PMIC_RG_VS2_CSL_MASK,
		PMIC_RG_VS2_CSL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs2_slp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS2_SLP_ADDR,
		val,
		PMIC_RG_VS2_SLP_MASK,
		PMIC_RG_VS2_SLP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs2_zx_os(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS2_ZX_OS_ADDR,
		val,
		PMIC_RG_VS2_ZX_OS_MASK,
		PMIC_RG_VS2_ZX_OS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs2_ndis_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS2_NDIS_EN_ADDR,
		val,
		PMIC_RG_VS2_NDIS_EN_MASK,
		PMIC_RG_VS2_NDIS_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vs2_ndis_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VS2_NDIS_EN_ADDR,
		&val,
		PMIC_RG_VS2_NDIS_EN_MASK,
		PMIC_RG_VS2_NDIS_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vs2_csm_n(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS2_CSM_N_ADDR,
		val,
		PMIC_RG_VS2_CSM_N_MASK,
		PMIC_RG_VS2_CSM_N_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs2_csm_p(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS2_CSM_P_ADDR,
		val,
		PMIC_RG_VS2_CSM_P_MASK,
		PMIC_RG_VS2_CSM_P_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs2_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS2_RSV_ADDR,
		val,
		PMIC_RG_VS2_RSV_MASK,
		PMIC_RG_VS2_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs2_zxos_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS2_ZXOS_TRIM_ADDR,
		val,
		PMIC_RG_VS2_ZXOS_TRIM_MASK,
		PMIC_RG_VS2_ZXOS_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs2_modeset(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS2_MODESET_ADDR,
		val,
		PMIC_RG_VS2_MODESET_MASK,
		PMIC_RG_VS2_MODESET_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs2_pfm_rip(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS2_PFM_RIP_ADDR,
		val,
		PMIC_RG_VS2_PFM_RIP_MASK,
		PMIC_RG_VS2_PFM_RIP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs2_tran_bst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS2_TRAN_BST_ADDR,
		val,
		PMIC_RG_VS2_TRAN_BST_MASK,
		PMIC_RG_VS2_TRAN_BST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs2_dts_enb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS2_DTS_ENB_ADDR,
		val,
		PMIC_RG_VS2_DTS_ENB_MASK,
		PMIC_RG_VS2_DTS_ENB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vs2_dts_enb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VS2_DTS_ENB_ADDR,
		&val,
		PMIC_RG_VS2_DTS_ENB_MASK,
		PMIC_RG_VS2_DTS_ENB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vs2_auto_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS2_AUTO_MODE_ADDR,
		val,
		PMIC_RG_VS2_AUTO_MODE_MASK,
		PMIC_RG_VS2_AUTO_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs2_pwm_trig(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS2_PWM_TRIG_ADDR,
		val,
		PMIC_RG_VS2_PWM_TRIG_MASK,
		PMIC_RG_VS2_PWM_TRIG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs2_rsv_h(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS2_RSV_H_ADDR,
		val,
		PMIC_RG_VS2_RSV_H_MASK,
		PMIC_RG_VS2_RSV_H_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs2_rsv_l(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS2_RSV_L_ADDR,
		val,
		PMIC_RG_VS2_RSV_L_MASK,
		PMIC_RG_VS2_RSV_L_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs2_nonaudible_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS2_NONAUDIBLE_EN_ADDR,
		val,
		PMIC_RG_VS2_NONAUDIBLE_EN_MASK,
		PMIC_RG_VS2_NONAUDIBLE_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vs2_nonaudible_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VS2_NONAUDIBLE_EN_ADDR,
		&val,
		PMIC_RG_VS2_NONAUDIBLE_EN_MASK,
		PMIC_RG_VS2_NONAUDIBLE_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vs2_sr_p(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS2_SR_P_ADDR,
		val,
		PMIC_RG_VS2_SR_P_MASK,
		PMIC_RG_VS2_SR_P_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs2_sr_n(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS2_SR_N_ADDR,
		val,
		PMIC_RG_VS2_SR_N_MASK,
		PMIC_RG_VS2_SR_N_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vs2_burst(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VS2_BURST_ADDR,
		val,
		PMIC_RG_VS2_BURST_MASK,
		PMIC_RG_VS2_BURST_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rgs_vs2_enpwm_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VS2_ENPWM_STATUS_ADDR,
		&val,
		PMIC_RGS_VS2_ENPWM_STATUS_MASK,
		PMIC_RGS_VS2_ENPWM_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rgs_vs2_oc_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VS2_OC_STATUS_ADDR,
		&val,
		PMIC_RGS_VS2_OC_STATUS_MASK,
		PMIC_RGS_VS2_OC_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rgs_vs2_dig_mon(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VS2_DIG_MON_ADDR,
		&val,
		PMIC_RGS_VS2_DIG_MON_MASK,
		PMIC_RGS_VS2_DIG_MON_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vpa_ndis_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPA_NDIS_EN_ADDR,
		val,
		PMIC_RG_VPA_NDIS_EN_MASK,
		PMIC_RG_VPA_NDIS_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vpa_ndis_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VPA_NDIS_EN_ADDR,
		&val,
		PMIC_RG_VPA_NDIS_EN_MASK,
		PMIC_RG_VPA_NDIS_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vpa_modeset(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPA_MODESET_ADDR,
		val,
		PMIC_RG_VPA_MODESET_MASK,
		PMIC_RG_VPA_MODESET_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vpa_cc(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPA_CC_ADDR,
		val,
		PMIC_RG_VPA_CC_MASK,
		PMIC_RG_VPA_CC_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vpa_csr(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPA_CSR_ADDR,
		val,
		PMIC_RG_VPA_CSR_MASK,
		PMIC_RG_VPA_CSR_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vpa_csmir(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPA_CSMIR_ADDR,
		val,
		PMIC_RG_VPA_CSMIR_MASK,
		PMIC_RG_VPA_CSMIR_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vpa_csl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPA_CSL_ADDR,
		val,
		PMIC_RG_VPA_CSL_MASK,
		PMIC_RG_VPA_CSL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vpa_slp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPA_SLP_ADDR,
		val,
		PMIC_RG_VPA_SLP_MASK,
		PMIC_RG_VPA_SLP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vpa_azc_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPA_AZC_EN_ADDR,
		val,
		PMIC_RG_VPA_AZC_EN_MASK,
		PMIC_RG_VPA_AZC_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vpa_azc_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VPA_AZC_EN_ADDR,
		&val,
		PMIC_RG_VPA_AZC_EN_MASK,
		PMIC_RG_VPA_AZC_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vpa_cp_fwupoff(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPA_CP_FWUPOFF_ADDR,
		val,
		PMIC_RG_VPA_CP_FWUPOFF_MASK,
		PMIC_RG_VPA_CP_FWUPOFF_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vpa_azc_delay(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPA_AZC_DELAY_ADDR,
		val,
		PMIC_RG_VPA_AZC_DELAY_MASK,
		PMIC_RG_VPA_AZC_DELAY_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vpa_rzsel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPA_RZSEL_ADDR,
		val,
		PMIC_RG_VPA_RZSEL_MASK,
		PMIC_RG_VPA_RZSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vpa_zxref(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPA_ZXREF_ADDR,
		val,
		PMIC_RG_VPA_ZXREF_MASK,
		PMIC_RG_VPA_ZXREF_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vpa_nlim_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPA_NLIM_SEL_ADDR,
		val,
		PMIC_RG_VPA_NLIM_SEL_MASK,
		PMIC_RG_VPA_NLIM_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vpa_hzp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPA_HZP_ADDR,
		val,
		PMIC_RG_VPA_HZP_MASK,
		PMIC_RG_VPA_HZP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vpa_bwex_gat(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPA_BWEX_GAT_ADDR,
		val,
		PMIC_RG_VPA_BWEX_GAT_MASK,
		PMIC_RG_VPA_BWEX_GAT_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vpa_slew(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPA_SLEW_ADDR,
		val,
		PMIC_RG_VPA_SLEW_MASK,
		PMIC_RG_VPA_SLEW_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vpa_slew_nmos(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPA_SLEW_NMOS_ADDR,
		val,
		PMIC_RG_VPA_SLEW_NMOS_MASK,
		PMIC_RG_VPA_SLEW_NMOS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vpa_min_on(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPA_MIN_ON_ADDR,
		val,
		PMIC_RG_VPA_MIN_ON_MASK,
		PMIC_RG_VPA_MIN_ON_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vpa_vbat_del(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPA_VBAT_DEL_ADDR,
		val,
		PMIC_RG_VPA_VBAT_DEL_MASK,
		PMIC_RG_VPA_VBAT_DEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rgs_vpa_azc_vos_sel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VPA_AZC_VOS_SEL_ADDR,
		&val,
		PMIC_RGS_VPA_AZC_VOS_SEL_MASK,
		PMIC_RGS_VPA_AZC_VOS_SEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vpa_min_pk(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPA_MIN_PK_ADDR,
		val,
		PMIC_RG_VPA_MIN_PK_MASK,
		PMIC_RG_VPA_MIN_PK_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vpa_rsv1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPA_RSV1_ADDR,
		val,
		PMIC_RG_VPA_RSV1_MASK,
		PMIC_RG_VPA_RSV1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vpa_rsv2(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VPA_RSV2_ADDR,
		val,
		PMIC_RG_VPA_RSV2_MASK,
		PMIC_RG_VPA_RSV2_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rgs_vpa_oc_status(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VPA_OC_STATUS_ADDR,
		&val,
		PMIC_RGS_VPA_OC_STATUS_MASK,
		PMIC_RGS_VPA_OC_STATUS_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_rgs_vpa_azc_zx(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VPA_AZC_ZX_ADDR,
		&val,
		PMIC_RGS_VPA_AZC_ZX_MASK,
		PMIC_RGS_VPA_AZC_ZX_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_wdtdbg_clr(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_WDTDBG_CLR_ADDR,
		val,
		PMIC_WDTDBG_CLR_MASK,
		PMIC_WDTDBG_CLR_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_wdtdbg_con0_rsv0(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_WDTDBG_CON0_RSV0_ADDR,
		val,
		PMIC_WDTDBG_CON0_RSV0_MASK,
		PMIC_WDTDBG_CON0_RSV0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_vproc11_vosel_wdtdbg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VPROC11_VOSEL_WDTDBG_ADDR,
		&val,
		PMIC_VPROC11_VOSEL_WDTDBG_MASK,
		PMIC_VPROC11_VOSEL_WDTDBG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vproc12_vosel_wdtdbg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VPROC12_VOSEL_WDTDBG_ADDR,
		&val,
		PMIC_VPROC12_VOSEL_WDTDBG_MASK,
		PMIC_VPROC12_VOSEL_WDTDBG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vcore_vosel_wdtdbg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VCORE_VOSEL_WDTDBG_ADDR,
		&val,
		PMIC_VCORE_VOSEL_WDTDBG_MASK,
		PMIC_VCORE_VOSEL_WDTDBG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vgpu_vosel_wdtdbg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VGPU_VOSEL_WDTDBG_ADDR,
		&val,
		PMIC_VGPU_VOSEL_WDTDBG_MASK,
		PMIC_VGPU_VOSEL_WDTDBG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vdram1_vosel_wdtdbg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VDRAM1_VOSEL_WDTDBG_ADDR,
		&val,
		PMIC_VDRAM1_VOSEL_WDTDBG_MASK,
		PMIC_VDRAM1_VOSEL_WDTDBG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vdram2_vosel_wdtdbg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VDRAM2_VOSEL_WDTDBG_ADDR,
		&val,
		PMIC_VDRAM2_VOSEL_WDTDBG_MASK,
		PMIC_VDRAM2_VOSEL_WDTDBG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vmodem_vosel_wdtdbg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VMODEM_VOSEL_WDTDBG_ADDR,
		&val,
		PMIC_VMODEM_VOSEL_WDTDBG_MASK,
		PMIC_VMODEM_VOSEL_WDTDBG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vs1_vosel_wdtdbg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VS1_VOSEL_WDTDBG_ADDR,
		&val,
		PMIC_VS1_VOSEL_WDTDBG_MASK,
		PMIC_VS1_VOSEL_WDTDBG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vs2_vosel_wdtdbg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VS2_VOSEL_WDTDBG_ADDR,
		&val,
		PMIC_VS2_VOSEL_WDTDBG_MASK,
		PMIC_VS2_VOSEL_WDTDBG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vpa_vosel_wdtdbg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VPA_VOSEL_WDTDBG_ADDR,
		&val,
		PMIC_VPA_VOSEL_WDTDBG_MASK,
		PMIC_VPA_VOSEL_WDTDBG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vsram_proc_vosel_wdtdbg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VSRAM_PROC_VOSEL_WDTDBG_ADDR,
		&val,
		PMIC_VSRAM_PROC_VOSEL_WDTDBG_MASK,
		PMIC_VSRAM_PROC_VOSEL_WDTDBG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vsram_core_vosel_wdtdbg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VSRAM_CORE_VOSEL_WDTDBG_ADDR,
		&val,
		PMIC_VSRAM_CORE_VOSEL_WDTDBG_MASK,
		PMIC_VSRAM_CORE_VOSEL_WDTDBG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vsram_gpu_vosel_wdtdbg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VSRAM_GPU_VOSEL_WDTDBG_ADDR,
		&val,
		PMIC_VSRAM_GPU_VOSEL_WDTDBG_MASK,
		PMIC_VSRAM_GPU_VOSEL_WDTDBG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_vsram_md_vosel_wdtdbg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_VSRAM_MD_VOSEL_WDTDBG_ADDR,
		&val,
		PMIC_VSRAM_MD_VOSEL_WDTDBG_MASK,
		PMIC_VSRAM_MD_VOSEL_WDTDBG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vio28_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO28_EN_ADDR,
		val,
		PMIC_RG_LDO_VIO28_EN_MASK,
		PMIC_RG_LDO_VIO28_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vio28_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VIO28_EN_ADDR,
		&val,
		PMIC_RG_LDO_VIO28_EN_MASK,
		PMIC_RG_LDO_VIO28_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vio28_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO28_LP_ADDR,
		val,
		PMIC_RG_LDO_VIO28_LP_MASK,
		PMIC_RG_LDO_VIO28_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vio28_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO28_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VIO28_SW_OP_EN_MASK,
		PMIC_RG_LDO_VIO28_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vio28_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VIO28_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VIO28_SW_OP_EN_MASK,
		PMIC_RG_LDO_VIO28_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vio28_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO28_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VIO28_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VIO28_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vio28_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VIO28_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VIO28_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VIO28_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vio28_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO28_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VIO28_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VIO28_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vio28_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VIO28_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VIO28_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VIO28_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vio28_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO28_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VIO28_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VIO28_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vio28_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VIO28_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VIO28_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VIO28_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vio28_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO28_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VIO28_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VIO28_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vio28_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VIO28_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VIO28_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VIO28_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vio28_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO28_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VIO28_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VIO28_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vio28_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VIO28_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VIO28_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VIO28_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vio28_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO28_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VIO28_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VIO28_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vio28_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VIO28_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VIO28_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VIO28_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vio28_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO28_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VIO28_ON_OP_MASK,
		PMIC_RG_LDO_VIO28_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vio28_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO28_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VIO28_LP_OP_MASK,
		PMIC_RG_LDO_VIO28_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vio28_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VIO28_MODE_ADDR,
		&val,
		PMIC_DA_QI_VIO28_MODE_MASK,
		PMIC_DA_QI_VIO28_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vio28_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO28_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VIO28_STBTD_MASK,
		PMIC_RG_LDO_VIO28_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vio28_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VIO28_STB_ADDR,
		&val,
		PMIC_DA_QI_VIO28_STB_MASK,
		PMIC_DA_QI_VIO28_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vio28_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VIO28_EN_ADDR,
		&val,
		PMIC_DA_QI_VIO28_EN_MASK,
		PMIC_DA_QI_VIO28_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vio28_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO28_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VIO28_OCFB_EN_MASK,
		PMIC_RG_LDO_VIO28_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vio28_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VIO28_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VIO28_OCFB_EN_MASK,
		PMIC_RG_LDO_VIO28_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vio28_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VIO28_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VIO28_OCFB_EN_MASK,
		PMIC_DA_QI_VIO28_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vio28_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO28_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VIO28_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VIO28_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vio28_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VIO28_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VIO28_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VIO28_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vio18_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO18_EN_ADDR,
		val,
		PMIC_RG_LDO_VIO18_EN_MASK,
		PMIC_RG_LDO_VIO18_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vio18_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VIO18_EN_ADDR,
		&val,
		PMIC_RG_LDO_VIO18_EN_MASK,
		PMIC_RG_LDO_VIO18_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vio18_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO18_LP_ADDR,
		val,
		PMIC_RG_LDO_VIO18_LP_MASK,
		PMIC_RG_LDO_VIO18_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vio18_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO18_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VIO18_SW_OP_EN_MASK,
		PMIC_RG_LDO_VIO18_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vio18_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VIO18_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VIO18_SW_OP_EN_MASK,
		PMIC_RG_LDO_VIO18_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vio18_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO18_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VIO18_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VIO18_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vio18_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VIO18_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VIO18_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VIO18_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vio18_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO18_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VIO18_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VIO18_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vio18_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VIO18_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VIO18_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VIO18_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vio18_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO18_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VIO18_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VIO18_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vio18_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VIO18_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VIO18_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VIO18_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vio18_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO18_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VIO18_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VIO18_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vio18_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VIO18_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VIO18_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VIO18_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vio18_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO18_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VIO18_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VIO18_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vio18_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VIO18_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VIO18_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VIO18_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vio18_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO18_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VIO18_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VIO18_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vio18_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VIO18_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VIO18_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VIO18_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vio18_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO18_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VIO18_ON_OP_MASK,
		PMIC_RG_LDO_VIO18_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vio18_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO18_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VIO18_LP_OP_MASK,
		PMIC_RG_LDO_VIO18_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vio18_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VIO18_MODE_ADDR,
		&val,
		PMIC_DA_QI_VIO18_MODE_MASK,
		PMIC_DA_QI_VIO18_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vio18_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO18_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VIO18_STBTD_MASK,
		PMIC_RG_LDO_VIO18_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vio18_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VIO18_STB_ADDR,
		&val,
		PMIC_DA_QI_VIO18_STB_MASK,
		PMIC_DA_QI_VIO18_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vio18_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VIO18_EN_ADDR,
		&val,
		PMIC_DA_QI_VIO18_EN_MASK,
		PMIC_DA_QI_VIO18_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vio18_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO18_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VIO18_OCFB_EN_MASK,
		PMIC_RG_LDO_VIO18_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vio18_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VIO18_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VIO18_OCFB_EN_MASK,
		PMIC_RG_LDO_VIO18_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vio18_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VIO18_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VIO18_OCFB_EN_MASK,
		PMIC_DA_QI_VIO18_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vio18_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO18_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VIO18_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VIO18_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vio18_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VIO18_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VIO18_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VIO18_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vufs18_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUFS18_EN_ADDR,
		val,
		PMIC_RG_LDO_VUFS18_EN_MASK,
		PMIC_RG_LDO_VUFS18_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vufs18_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VUFS18_EN_ADDR,
		&val,
		PMIC_RG_LDO_VUFS18_EN_MASK,
		PMIC_RG_LDO_VUFS18_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vufs18_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUFS18_LP_ADDR,
		val,
		PMIC_RG_LDO_VUFS18_LP_MASK,
		PMIC_RG_LDO_VUFS18_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vufs18_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUFS18_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VUFS18_SW_OP_EN_MASK,
		PMIC_RG_LDO_VUFS18_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vufs18_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VUFS18_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VUFS18_SW_OP_EN_MASK,
		PMIC_RG_LDO_VUFS18_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vufs18_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUFS18_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VUFS18_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VUFS18_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vufs18_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VUFS18_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VUFS18_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VUFS18_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vufs18_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUFS18_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VUFS18_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VUFS18_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vufs18_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VUFS18_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VUFS18_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VUFS18_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vufs18_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUFS18_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VUFS18_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VUFS18_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vufs18_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VUFS18_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VUFS18_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VUFS18_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vufs18_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUFS18_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VUFS18_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VUFS18_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vufs18_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VUFS18_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VUFS18_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VUFS18_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vufs18_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUFS18_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VUFS18_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VUFS18_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vufs18_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VUFS18_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VUFS18_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VUFS18_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vufs18_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUFS18_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VUFS18_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VUFS18_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vufs18_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VUFS18_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VUFS18_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VUFS18_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vufs18_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUFS18_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VUFS18_ON_OP_MASK,
		PMIC_RG_LDO_VUFS18_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vufs18_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUFS18_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VUFS18_LP_OP_MASK,
		PMIC_RG_LDO_VUFS18_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vufs18_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VUFS18_MODE_ADDR,
		&val,
		PMIC_DA_QI_VUFS18_MODE_MASK,
		PMIC_DA_QI_VUFS18_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vufs18_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUFS18_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VUFS18_STBTD_MASK,
		PMIC_RG_LDO_VUFS18_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vufs18_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VUFS18_STB_ADDR,
		&val,
		PMIC_DA_QI_VUFS18_STB_MASK,
		PMIC_DA_QI_VUFS18_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vufs18_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VUFS18_EN_ADDR,
		&val,
		PMIC_DA_QI_VUFS18_EN_MASK,
		PMIC_DA_QI_VUFS18_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vufs18_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUFS18_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VUFS18_OCFB_EN_MASK,
		PMIC_RG_LDO_VUFS18_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vufs18_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VUFS18_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VUFS18_OCFB_EN_MASK,
		PMIC_RG_LDO_VUFS18_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vufs18_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VUFS18_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VUFS18_OCFB_EN_MASK,
		PMIC_DA_QI_VUFS18_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vufs18_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUFS18_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VUFS18_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VUFS18_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vufs18_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VUFS18_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VUFS18_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VUFS18_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va10_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA10_EN_ADDR,
		val,
		PMIC_RG_LDO_VA10_EN_MASK,
		PMIC_RG_LDO_VA10_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_va10_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VA10_EN_ADDR,
		&val,
		PMIC_RG_LDO_VA10_EN_MASK,
		PMIC_RG_LDO_VA10_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va10_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA10_LP_ADDR,
		val,
		PMIC_RG_LDO_VA10_LP_MASK,
		PMIC_RG_LDO_VA10_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_va10_vocal(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VA10_VOCAL_ADDR,
		val,
		PMIC_RG_VA10_VOCAL_MASK,
		PMIC_RG_VA10_VOCAL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_va10_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VA10_VOSEL_ADDR,
		val,
		PMIC_RG_VA10_VOSEL_MASK,
		PMIC_RG_VA10_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_va10_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VA10_VOSEL_ADDR,
		&val,
		PMIC_RG_VA10_VOSEL_MASK,
		PMIC_RG_VA10_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va10_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA10_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VA10_SW_OP_EN_MASK,
		PMIC_RG_LDO_VA10_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_va10_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VA10_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VA10_SW_OP_EN_MASK,
		PMIC_RG_LDO_VA10_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va10_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA10_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VA10_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VA10_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_va10_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VA10_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VA10_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VA10_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va10_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA10_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VA10_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VA10_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_va10_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VA10_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VA10_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VA10_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va10_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA10_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VA10_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VA10_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_va10_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VA10_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VA10_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VA10_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va10_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA10_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VA10_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VA10_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_va10_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VA10_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VA10_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VA10_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va10_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA10_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VA10_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VA10_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_va10_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VA10_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VA10_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VA10_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va10_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA10_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VA10_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VA10_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_va10_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VA10_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VA10_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VA10_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va10_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA10_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VA10_ON_OP_MASK,
		PMIC_RG_LDO_VA10_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_va10_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA10_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VA10_LP_OP_MASK,
		PMIC_RG_LDO_VA10_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_va10_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VA10_MODE_ADDR,
		&val,
		PMIC_DA_QI_VA10_MODE_MASK,
		PMIC_DA_QI_VA10_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va10_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA10_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VA10_STBTD_MASK,
		PMIC_RG_LDO_VA10_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_va10_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VA10_STB_ADDR,
		&val,
		PMIC_DA_QI_VA10_STB_MASK,
		PMIC_DA_QI_VA10_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_va10_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VA10_EN_ADDR,
		&val,
		PMIC_DA_QI_VA10_EN_MASK,
		PMIC_DA_QI_VA10_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va10_sleep_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA10_SLEEP_MODE_ADDR,
		val,
		PMIC_RG_LDO_VA10_SLEEP_MODE_MASK,
		PMIC_RG_LDO_VA10_SLEEP_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_va10_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA10_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VA10_OCFB_EN_MASK,
		PMIC_RG_LDO_VA10_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_va10_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VA10_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VA10_OCFB_EN_MASK,
		PMIC_RG_LDO_VA10_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_va10_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VA10_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VA10_OCFB_EN_MASK,
		PMIC_DA_QI_VA10_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va10_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA10_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VA10_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VA10_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_va10_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VA10_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VA10_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VA10_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va12_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA12_EN_ADDR,
		val,
		PMIC_RG_LDO_VA12_EN_MASK,
		PMIC_RG_LDO_VA12_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_va12_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VA12_EN_ADDR,
		&val,
		PMIC_RG_LDO_VA12_EN_MASK,
		PMIC_RG_LDO_VA12_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va12_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA12_LP_ADDR,
		val,
		PMIC_RG_LDO_VA12_LP_MASK,
		PMIC_RG_LDO_VA12_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_va12_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA12_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VA12_SW_OP_EN_MASK,
		PMIC_RG_LDO_VA12_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_va12_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VA12_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VA12_SW_OP_EN_MASK,
		PMIC_RG_LDO_VA12_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va12_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA12_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VA12_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VA12_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_va12_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VA12_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VA12_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VA12_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va12_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA12_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VA12_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VA12_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_va12_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VA12_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VA12_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VA12_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va12_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA12_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VA12_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VA12_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_va12_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VA12_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VA12_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VA12_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va12_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA12_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VA12_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VA12_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_va12_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VA12_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VA12_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VA12_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va12_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA12_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VA12_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VA12_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_va12_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VA12_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VA12_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VA12_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va12_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA12_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VA12_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VA12_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_va12_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VA12_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VA12_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VA12_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va12_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA12_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VA12_ON_OP_MASK,
		PMIC_RG_LDO_VA12_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_va12_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA12_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VA12_LP_OP_MASK,
		PMIC_RG_LDO_VA12_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_va12_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VA12_MODE_ADDR,
		&val,
		PMIC_DA_QI_VA12_MODE_MASK,
		PMIC_DA_QI_VA12_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va12_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA12_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VA12_STBTD_MASK,
		PMIC_RG_LDO_VA12_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_va12_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VA12_STB_ADDR,
		&val,
		PMIC_DA_QI_VA12_STB_MASK,
		PMIC_DA_QI_VA12_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_va12_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VA12_EN_ADDR,
		&val,
		PMIC_DA_QI_VA12_EN_MASK,
		PMIC_DA_QI_VA12_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va12_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA12_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VA12_OCFB_EN_MASK,
		PMIC_RG_LDO_VA12_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_va12_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VA12_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VA12_OCFB_EN_MASK,
		PMIC_RG_LDO_VA12_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_va12_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VA12_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VA12_OCFB_EN_MASK,
		PMIC_DA_QI_VA12_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va12_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA12_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VA12_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VA12_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_va12_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VA12_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VA12_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VA12_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va18_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA18_EN_ADDR,
		val,
		PMIC_RG_LDO_VA18_EN_MASK,
		PMIC_RG_LDO_VA18_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_va18_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VA18_EN_ADDR,
		&val,
		PMIC_RG_LDO_VA18_EN_MASK,
		PMIC_RG_LDO_VA18_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va18_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA18_LP_ADDR,
		val,
		PMIC_RG_LDO_VA18_LP_MASK,
		PMIC_RG_LDO_VA18_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_va18_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA18_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VA18_SW_OP_EN_MASK,
		PMIC_RG_LDO_VA18_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_va18_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VA18_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VA18_SW_OP_EN_MASK,
		PMIC_RG_LDO_VA18_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va18_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA18_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VA18_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VA18_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_va18_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VA18_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VA18_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VA18_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va18_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA18_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VA18_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VA18_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_va18_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VA18_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VA18_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VA18_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va18_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA18_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VA18_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VA18_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_va18_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VA18_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VA18_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VA18_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va18_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA18_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VA18_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VA18_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_va18_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VA18_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VA18_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VA18_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va18_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA18_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VA18_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VA18_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_va18_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VA18_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VA18_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VA18_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va18_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA18_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VA18_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VA18_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_va18_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VA18_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VA18_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VA18_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va18_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA18_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VA18_ON_OP_MASK,
		PMIC_RG_LDO_VA18_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_va18_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA18_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VA18_LP_OP_MASK,
		PMIC_RG_LDO_VA18_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_va18_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VA18_MODE_ADDR,
		&val,
		PMIC_DA_QI_VA18_MODE_MASK,
		PMIC_DA_QI_VA18_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va18_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA18_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VA18_STBTD_MASK,
		PMIC_RG_LDO_VA18_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_va18_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VA18_STB_ADDR,
		&val,
		PMIC_DA_QI_VA18_STB_MASK,
		PMIC_DA_QI_VA18_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_va18_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VA18_EN_ADDR,
		&val,
		PMIC_DA_QI_VA18_EN_MASK,
		PMIC_DA_QI_VA18_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va18_auxadc_pwdb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA18_AUXADC_PWDB_EN_ADDR,
		val,
		PMIC_RG_LDO_VA18_AUXADC_PWDB_EN_MASK,
		PMIC_RG_LDO_VA18_AUXADC_PWDB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_va18_auxadc_pwdb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VA18_AUXADC_PWDB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VA18_AUXADC_PWDB_EN_MASK,
		PMIC_RG_LDO_VA18_AUXADC_PWDB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va18_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA18_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VA18_OCFB_EN_MASK,
		PMIC_RG_LDO_VA18_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_va18_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VA18_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VA18_OCFB_EN_MASK,
		PMIC_RG_LDO_VA18_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_va18_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VA18_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VA18_OCFB_EN_MASK,
		PMIC_DA_QI_VA18_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va18_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA18_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VA18_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VA18_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_va18_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VA18_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VA18_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VA18_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vusb33_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUSB33_EN_0_ADDR,
		val,
		PMIC_RG_LDO_VUSB33_EN_0_MASK,
		PMIC_RG_LDO_VUSB33_EN_0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vusb33_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VUSB33_EN_0_ADDR,
		&val,
		PMIC_RG_LDO_VUSB33_EN_0_MASK,
		PMIC_RG_LDO_VUSB33_EN_0_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vusb33_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUSB33_LP_ADDR,
		val,
		PMIC_RG_LDO_VUSB33_LP_MASK,
		PMIC_RG_LDO_VUSB33_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vusb33_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUSB33_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VUSB33_SW_OP_EN_MASK,
		PMIC_RG_LDO_VUSB33_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vusb33_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VUSB33_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VUSB33_SW_OP_EN_MASK,
		PMIC_RG_LDO_VUSB33_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vusb33_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUSB33_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VUSB33_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VUSB33_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vusb33_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VUSB33_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VUSB33_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VUSB33_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vusb33_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUSB33_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VUSB33_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VUSB33_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vusb33_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VUSB33_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VUSB33_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VUSB33_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vusb33_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUSB33_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VUSB33_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VUSB33_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vusb33_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VUSB33_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VUSB33_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VUSB33_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vusb33_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUSB33_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VUSB33_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VUSB33_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vusb33_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VUSB33_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VUSB33_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VUSB33_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vusb33_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUSB33_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VUSB33_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VUSB33_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vusb33_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VUSB33_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VUSB33_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VUSB33_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vusb33_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUSB33_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VUSB33_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VUSB33_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vusb33_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VUSB33_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VUSB33_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VUSB33_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vusb33_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUSB33_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VUSB33_ON_OP_MASK,
		PMIC_RG_LDO_VUSB33_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vusb33_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUSB33_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VUSB33_LP_OP_MASK,
		PMIC_RG_LDO_VUSB33_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vusb33_1_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUSB33_EN_1_ADDR,
		val,
		PMIC_RG_LDO_VUSB33_EN_1_MASK,
		PMIC_RG_LDO_VUSB33_EN_1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vusb33_1_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VUSB33_EN_1_ADDR,
		&val,
		PMIC_RG_LDO_VUSB33_EN_1_MASK,
		PMIC_RG_LDO_VUSB33_EN_1_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vusb33_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VUSB33_MODE_ADDR,
		&val,
		PMIC_DA_QI_VUSB33_MODE_MASK,
		PMIC_DA_QI_VUSB33_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vusb33_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUSB33_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VUSB33_STBTD_MASK,
		PMIC_RG_LDO_VUSB33_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vusb33_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VUSB33_STB_ADDR,
		&val,
		PMIC_DA_QI_VUSB33_STB_MASK,
		PMIC_DA_QI_VUSB33_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vusb33_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VUSB33_EN_ADDR,
		&val,
		PMIC_DA_QI_VUSB33_EN_MASK,
		PMIC_DA_QI_VUSB33_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vusb33_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUSB33_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VUSB33_OCFB_EN_MASK,
		PMIC_RG_LDO_VUSB33_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vusb33_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VUSB33_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VUSB33_OCFB_EN_MASK,
		PMIC_RG_LDO_VUSB33_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vusb33_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VUSB33_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VUSB33_OCFB_EN_MASK,
		PMIC_DA_QI_VUSB33_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vusb33_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUSB33_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VUSB33_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VUSB33_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vusb33_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VUSB33_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VUSB33_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VUSB33_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vemc_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VEMC_EN_ADDR,
		val,
		PMIC_RG_LDO_VEMC_EN_MASK,
		PMIC_RG_LDO_VEMC_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vemc_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VEMC_EN_ADDR,
		&val,
		PMIC_RG_LDO_VEMC_EN_MASK,
		PMIC_RG_LDO_VEMC_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vemc_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VEMC_LP_ADDR,
		val,
		PMIC_RG_LDO_VEMC_LP_MASK,
		PMIC_RG_LDO_VEMC_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vemc_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VEMC_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VEMC_SW_OP_EN_MASK,
		PMIC_RG_LDO_VEMC_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vemc_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VEMC_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VEMC_SW_OP_EN_MASK,
		PMIC_RG_LDO_VEMC_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vemc_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VEMC_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VEMC_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VEMC_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vemc_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VEMC_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VEMC_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VEMC_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vemc_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VEMC_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VEMC_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VEMC_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vemc_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VEMC_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VEMC_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VEMC_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vemc_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VEMC_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VEMC_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VEMC_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vemc_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VEMC_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VEMC_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VEMC_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vemc_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VEMC_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VEMC_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VEMC_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vemc_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VEMC_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VEMC_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VEMC_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vemc_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VEMC_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VEMC_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VEMC_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vemc_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VEMC_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VEMC_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VEMC_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vemc_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VEMC_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VEMC_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VEMC_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vemc_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VEMC_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VEMC_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VEMC_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vemc_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VEMC_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VEMC_ON_OP_MASK,
		PMIC_RG_LDO_VEMC_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vemc_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VEMC_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VEMC_LP_OP_MASK,
		PMIC_RG_LDO_VEMC_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vemc_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VEMC_MODE_ADDR,
		&val,
		PMIC_DA_QI_VEMC_MODE_MASK,
		PMIC_DA_QI_VEMC_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vemc_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VEMC_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VEMC_STBTD_MASK,
		PMIC_RG_LDO_VEMC_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vemc_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VEMC_STB_ADDR,
		&val,
		PMIC_DA_QI_VEMC_STB_MASK,
		PMIC_DA_QI_VEMC_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vemc_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VEMC_EN_ADDR,
		&val,
		PMIC_DA_QI_VEMC_EN_MASK,
		PMIC_DA_QI_VEMC_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vemc_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VEMC_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VEMC_OCFB_EN_MASK,
		PMIC_RG_LDO_VEMC_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vemc_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VEMC_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VEMC_OCFB_EN_MASK,
		PMIC_RG_LDO_VEMC_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vemc_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VEMC_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VEMC_OCFB_EN_MASK,
		PMIC_DA_QI_VEMC_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vemc_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VEMC_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VEMC_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VEMC_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vemc_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VEMC_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VEMC_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VEMC_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo22_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO22_EN_ADDR,
		val,
		PMIC_RG_LDO_VXO22_EN_MASK,
		PMIC_RG_LDO_VXO22_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vxo22_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VXO22_EN_ADDR,
		&val,
		PMIC_RG_LDO_VXO22_EN_MASK,
		PMIC_RG_LDO_VXO22_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo22_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO22_LP_ADDR,
		val,
		PMIC_RG_LDO_VXO22_LP_MASK,
		PMIC_RG_LDO_VXO22_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo22_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO22_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VXO22_SW_OP_EN_MASK,
		PMIC_RG_LDO_VXO22_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vxo22_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VXO22_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VXO22_SW_OP_EN_MASK,
		PMIC_RG_LDO_VXO22_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo22_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO22_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VXO22_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VXO22_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vxo22_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VXO22_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VXO22_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VXO22_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo22_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO22_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VXO22_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VXO22_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vxo22_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VXO22_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VXO22_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VXO22_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo22_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO22_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VXO22_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VXO22_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vxo22_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VXO22_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VXO22_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VXO22_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo22_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO22_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VXO22_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VXO22_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vxo22_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VXO22_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VXO22_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VXO22_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo22_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO22_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VXO22_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VXO22_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vxo22_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VXO22_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VXO22_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VXO22_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo22_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO22_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VXO22_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VXO22_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vxo22_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VXO22_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VXO22_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VXO22_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo22_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO22_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VXO22_ON_OP_MASK,
		PMIC_RG_LDO_VXO22_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo22_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO22_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VXO22_LP_OP_MASK,
		PMIC_RG_LDO_VXO22_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vxo22_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VXO22_MODE_ADDR,
		&val,
		PMIC_DA_QI_VXO22_MODE_MASK,
		PMIC_DA_QI_VXO22_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo22_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO22_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VXO22_STBTD_MASK,
		PMIC_RG_LDO_VXO22_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vxo22_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VXO22_STB_ADDR,
		&val,
		PMIC_DA_QI_VXO22_STB_MASK,
		PMIC_DA_QI_VXO22_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vxo22_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VXO22_EN_ADDR,
		&val,
		PMIC_DA_QI_VXO22_EN_MASK,
		PMIC_DA_QI_VXO22_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo22_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO22_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VXO22_OCFB_EN_MASK,
		PMIC_RG_LDO_VXO22_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vxo22_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VXO22_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VXO22_OCFB_EN_MASK,
		PMIC_RG_LDO_VXO22_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vxo22_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VXO22_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VXO22_OCFB_EN_MASK,
		PMIC_DA_QI_VXO22_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo22_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO22_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VXO22_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VXO22_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vxo22_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VXO22_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VXO22_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VXO22_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo18_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO18_EN_ADDR,
		val,
		PMIC_RG_LDO_VXO18_EN_MASK,
		PMIC_RG_LDO_VXO18_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vxo18_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VXO18_EN_ADDR,
		&val,
		PMIC_RG_LDO_VXO18_EN_MASK,
		PMIC_RG_LDO_VXO18_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo18_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO18_LP_ADDR,
		val,
		PMIC_RG_LDO_VXO18_LP_MASK,
		PMIC_RG_LDO_VXO18_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo18_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO18_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VXO18_SW_OP_EN_MASK,
		PMIC_RG_LDO_VXO18_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vxo18_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VXO18_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VXO18_SW_OP_EN_MASK,
		PMIC_RG_LDO_VXO18_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo18_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO18_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VXO18_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VXO18_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vxo18_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VXO18_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VXO18_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VXO18_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo18_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO18_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VXO18_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VXO18_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vxo18_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VXO18_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VXO18_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VXO18_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo18_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO18_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VXO18_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VXO18_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vxo18_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VXO18_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VXO18_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VXO18_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo18_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO18_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VXO18_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VXO18_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vxo18_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VXO18_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VXO18_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VXO18_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo18_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO18_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VXO18_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VXO18_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vxo18_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VXO18_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VXO18_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VXO18_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo18_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO18_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VXO18_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VXO18_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vxo18_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VXO18_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VXO18_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VXO18_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo18_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO18_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VXO18_ON_OP_MASK,
		PMIC_RG_LDO_VXO18_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo18_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO18_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VXO18_LP_OP_MASK,
		PMIC_RG_LDO_VXO18_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vxo18_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VXO18_MODE_ADDR,
		&val,
		PMIC_DA_QI_VXO18_MODE_MASK,
		PMIC_DA_QI_VXO18_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo18_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO18_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VXO18_STBTD_MASK,
		PMIC_RG_LDO_VXO18_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vxo18_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VXO18_STB_ADDR,
		&val,
		PMIC_DA_QI_VXO18_STB_MASK,
		PMIC_DA_QI_VXO18_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vxo18_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VXO18_EN_ADDR,
		&val,
		PMIC_DA_QI_VXO18_EN_MASK,
		PMIC_DA_QI_VXO18_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo18_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO18_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VXO18_OCFB_EN_MASK,
		PMIC_RG_LDO_VXO18_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vxo18_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VXO18_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VXO18_OCFB_EN_MASK,
		PMIC_RG_LDO_VXO18_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vxo18_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VXO18_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VXO18_OCFB_EN_MASK,
		PMIC_DA_QI_VXO18_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo18_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO18_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VXO18_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VXO18_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vxo18_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VXO18_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VXO18_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VXO18_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim1_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM1_EN_ADDR,
		val,
		PMIC_RG_LDO_VSIM1_EN_MASK,
		PMIC_RG_LDO_VSIM1_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsim1_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSIM1_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSIM1_EN_MASK,
		PMIC_RG_LDO_VSIM1_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim1_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM1_LP_ADDR,
		val,
		PMIC_RG_LDO_VSIM1_LP_MASK,
		PMIC_RG_LDO_VSIM1_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim1_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM1_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VSIM1_SW_OP_EN_MASK,
		PMIC_RG_LDO_VSIM1_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsim1_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSIM1_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSIM1_SW_OP_EN_MASK,
		PMIC_RG_LDO_VSIM1_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim1_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM1_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VSIM1_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VSIM1_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsim1_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSIM1_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSIM1_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VSIM1_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim1_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM1_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VSIM1_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VSIM1_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsim1_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSIM1_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSIM1_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VSIM1_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim1_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM1_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VSIM1_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VSIM1_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsim1_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSIM1_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSIM1_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VSIM1_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim1_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM1_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VSIM1_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VSIM1_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsim1_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSIM1_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VSIM1_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VSIM1_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim1_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM1_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VSIM1_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VSIM1_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsim1_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSIM1_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VSIM1_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VSIM1_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim1_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM1_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VSIM1_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VSIM1_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsim1_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSIM1_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VSIM1_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VSIM1_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim1_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM1_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VSIM1_ON_OP_MASK,
		PMIC_RG_LDO_VSIM1_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim1_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM1_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VSIM1_LP_OP_MASK,
		PMIC_RG_LDO_VSIM1_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vsim1_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSIM1_MODE_ADDR,
		&val,
		PMIC_DA_QI_VSIM1_MODE_MASK,
		PMIC_DA_QI_VSIM1_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim1_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM1_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VSIM1_STBTD_MASK,
		PMIC_RG_LDO_VSIM1_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vsim1_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSIM1_STB_ADDR,
		&val,
		PMIC_DA_QI_VSIM1_STB_MASK,
		PMIC_DA_QI_VSIM1_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vsim1_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSIM1_EN_ADDR,
		&val,
		PMIC_DA_QI_VSIM1_EN_MASK,
		PMIC_DA_QI_VSIM1_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim1_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM1_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VSIM1_OCFB_EN_MASK,
		PMIC_RG_LDO_VSIM1_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsim1_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSIM1_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSIM1_OCFB_EN_MASK,
		PMIC_RG_LDO_VSIM1_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vsim1_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSIM1_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VSIM1_OCFB_EN_MASK,
		PMIC_DA_QI_VSIM1_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim1_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM1_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VSIM1_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VSIM1_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vsim1_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSIM1_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VSIM1_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VSIM1_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim2_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM2_EN_ADDR,
		val,
		PMIC_RG_LDO_VSIM2_EN_MASK,
		PMIC_RG_LDO_VSIM2_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsim2_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSIM2_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSIM2_EN_MASK,
		PMIC_RG_LDO_VSIM2_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim2_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM2_LP_ADDR,
		val,
		PMIC_RG_LDO_VSIM2_LP_MASK,
		PMIC_RG_LDO_VSIM2_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim2_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM2_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VSIM2_SW_OP_EN_MASK,
		PMIC_RG_LDO_VSIM2_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsim2_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSIM2_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSIM2_SW_OP_EN_MASK,
		PMIC_RG_LDO_VSIM2_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim2_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM2_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VSIM2_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VSIM2_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsim2_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSIM2_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSIM2_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VSIM2_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim2_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM2_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VSIM2_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VSIM2_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsim2_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSIM2_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSIM2_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VSIM2_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim2_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM2_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VSIM2_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VSIM2_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsim2_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSIM2_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSIM2_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VSIM2_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim2_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM2_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VSIM2_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VSIM2_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsim2_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSIM2_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VSIM2_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VSIM2_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim2_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM2_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VSIM2_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VSIM2_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsim2_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSIM2_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VSIM2_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VSIM2_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim2_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM2_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VSIM2_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VSIM2_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsim2_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSIM2_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VSIM2_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VSIM2_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim2_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM2_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VSIM2_ON_OP_MASK,
		PMIC_RG_LDO_VSIM2_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim2_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM2_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VSIM2_LP_OP_MASK,
		PMIC_RG_LDO_VSIM2_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vsim2_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSIM2_MODE_ADDR,
		&val,
		PMIC_DA_QI_VSIM2_MODE_MASK,
		PMIC_DA_QI_VSIM2_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim2_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM2_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VSIM2_STBTD_MASK,
		PMIC_RG_LDO_VSIM2_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vsim2_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSIM2_STB_ADDR,
		&val,
		PMIC_DA_QI_VSIM2_STB_MASK,
		PMIC_DA_QI_VSIM2_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vsim2_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSIM2_EN_ADDR,
		&val,
		PMIC_DA_QI_VSIM2_EN_MASK,
		PMIC_DA_QI_VSIM2_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim2_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM2_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VSIM2_OCFB_EN_MASK,
		PMIC_RG_LDO_VSIM2_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsim2_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSIM2_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSIM2_OCFB_EN_MASK,
		PMIC_RG_LDO_VSIM2_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vsim2_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSIM2_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VSIM2_OCFB_EN_MASK,
		PMIC_DA_QI_VSIM2_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim2_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM2_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VSIM2_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VSIM2_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vsim2_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSIM2_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VSIM2_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VSIM2_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd1_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD1_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMD1_EN_MASK,
		PMIC_RG_LDO_VCAMD1_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcamd1_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMD1_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMD1_EN_MASK,
		PMIC_RG_LDO_VCAMD1_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd1_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD1_LP_ADDR,
		val,
		PMIC_RG_LDO_VCAMD1_LP_MASK,
		PMIC_RG_LDO_VCAMD1_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd1_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD1_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMD1_SW_OP_EN_MASK,
		PMIC_RG_LDO_VCAMD1_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcamd1_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMD1_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMD1_SW_OP_EN_MASK,
		PMIC_RG_LDO_VCAMD1_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd1_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD1_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMD1_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VCAMD1_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcamd1_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMD1_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMD1_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VCAMD1_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd1_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD1_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMD1_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VCAMD1_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcamd1_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMD1_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMD1_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VCAMD1_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd1_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD1_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMD1_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VCAMD1_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcamd1_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMD1_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMD1_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VCAMD1_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd1_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD1_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VCAMD1_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VCAMD1_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcamd1_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMD1_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VCAMD1_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VCAMD1_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd1_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD1_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VCAMD1_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VCAMD1_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcamd1_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMD1_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VCAMD1_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VCAMD1_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd1_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD1_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VCAMD1_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VCAMD1_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcamd1_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMD1_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VCAMD1_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VCAMD1_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd1_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD1_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VCAMD1_ON_OP_MASK,
		PMIC_RG_LDO_VCAMD1_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd1_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD1_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VCAMD1_LP_OP_MASK,
		PMIC_RG_LDO_VCAMD1_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vcamd1_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCAMD1_MODE_ADDR,
		&val,
		PMIC_DA_QI_VCAMD1_MODE_MASK,
		PMIC_DA_QI_VCAMD1_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd1_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD1_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VCAMD1_STBTD_MASK,
		PMIC_RG_LDO_VCAMD1_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vcamd1_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCAMD1_STB_ADDR,
		&val,
		PMIC_DA_QI_VCAMD1_STB_MASK,
		PMIC_DA_QI_VCAMD1_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vcamd1_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCAMD1_EN_ADDR,
		&val,
		PMIC_DA_QI_VCAMD1_EN_MASK,
		PMIC_DA_QI_VCAMD1_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd1_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD1_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMD1_OCFB_EN_MASK,
		PMIC_RG_LDO_VCAMD1_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcamd1_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMD1_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMD1_OCFB_EN_MASK,
		PMIC_RG_LDO_VCAMD1_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vcamd1_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCAMD1_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VCAMD1_OCFB_EN_MASK,
		PMIC_DA_QI_VCAMD1_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd1_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD1_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VCAMD1_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VCAMD1_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vcamd1_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCAMD1_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VCAMD1_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VCAMD1_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd2_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD2_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMD2_EN_MASK,
		PMIC_RG_LDO_VCAMD2_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcamd2_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMD2_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMD2_EN_MASK,
		PMIC_RG_LDO_VCAMD2_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd2_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD2_LP_ADDR,
		val,
		PMIC_RG_LDO_VCAMD2_LP_MASK,
		PMIC_RG_LDO_VCAMD2_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd2_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD2_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMD2_SW_OP_EN_MASK,
		PMIC_RG_LDO_VCAMD2_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcamd2_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMD2_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMD2_SW_OP_EN_MASK,
		PMIC_RG_LDO_VCAMD2_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd2_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD2_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMD2_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VCAMD2_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcamd2_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMD2_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMD2_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VCAMD2_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd2_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD2_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMD2_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VCAMD2_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcamd2_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMD2_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMD2_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VCAMD2_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd2_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD2_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMD2_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VCAMD2_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcamd2_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMD2_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMD2_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VCAMD2_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd2_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD2_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VCAMD2_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VCAMD2_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcamd2_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMD2_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VCAMD2_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VCAMD2_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd2_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD2_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VCAMD2_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VCAMD2_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcamd2_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMD2_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VCAMD2_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VCAMD2_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd2_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD2_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VCAMD2_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VCAMD2_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcamd2_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMD2_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VCAMD2_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VCAMD2_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd2_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD2_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VCAMD2_ON_OP_MASK,
		PMIC_RG_LDO_VCAMD2_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd2_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD2_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VCAMD2_LP_OP_MASK,
		PMIC_RG_LDO_VCAMD2_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vcamd2_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCAMD2_MODE_ADDR,
		&val,
		PMIC_DA_QI_VCAMD2_MODE_MASK,
		PMIC_DA_QI_VCAMD2_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd2_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD2_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VCAMD2_STBTD_MASK,
		PMIC_RG_LDO_VCAMD2_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vcamd2_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCAMD2_STB_ADDR,
		&val,
		PMIC_DA_QI_VCAMD2_STB_MASK,
		PMIC_DA_QI_VCAMD2_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vcamd2_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCAMD2_EN_ADDR,
		&val,
		PMIC_DA_QI_VCAMD2_EN_MASK,
		PMIC_DA_QI_VCAMD2_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd2_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD2_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMD2_OCFB_EN_MASK,
		PMIC_RG_LDO_VCAMD2_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcamd2_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMD2_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMD2_OCFB_EN_MASK,
		PMIC_RG_LDO_VCAMD2_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vcamd2_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCAMD2_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VCAMD2_OCFB_EN_MASK,
		PMIC_DA_QI_VCAMD2_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd2_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD2_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VCAMD2_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VCAMD2_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vcamd2_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCAMD2_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VCAMD2_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VCAMD2_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamio_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMIO_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMIO_EN_MASK,
		PMIC_RG_LDO_VCAMIO_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcamio_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMIO_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMIO_EN_MASK,
		PMIC_RG_LDO_VCAMIO_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamio_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMIO_LP_ADDR,
		val,
		PMIC_RG_LDO_VCAMIO_LP_MASK,
		PMIC_RG_LDO_VCAMIO_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamio_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMIO_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMIO_SW_OP_EN_MASK,
		PMIC_RG_LDO_VCAMIO_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcamio_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMIO_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMIO_SW_OP_EN_MASK,
		PMIC_RG_LDO_VCAMIO_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamio_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMIO_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMIO_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VCAMIO_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcamio_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMIO_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMIO_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VCAMIO_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamio_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMIO_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMIO_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VCAMIO_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcamio_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMIO_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMIO_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VCAMIO_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamio_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMIO_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMIO_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VCAMIO_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcamio_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMIO_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMIO_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VCAMIO_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamio_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMIO_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VCAMIO_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VCAMIO_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcamio_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMIO_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VCAMIO_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VCAMIO_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamio_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMIO_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VCAMIO_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VCAMIO_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcamio_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMIO_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VCAMIO_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VCAMIO_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamio_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMIO_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VCAMIO_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VCAMIO_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcamio_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMIO_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VCAMIO_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VCAMIO_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamio_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMIO_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VCAMIO_ON_OP_MASK,
		PMIC_RG_LDO_VCAMIO_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamio_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMIO_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VCAMIO_LP_OP_MASK,
		PMIC_RG_LDO_VCAMIO_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vcamio_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCAMIO_MODE_ADDR,
		&val,
		PMIC_DA_QI_VCAMIO_MODE_MASK,
		PMIC_DA_QI_VCAMIO_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamio_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMIO_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VCAMIO_STBTD_MASK,
		PMIC_RG_LDO_VCAMIO_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vcamio_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCAMIO_STB_ADDR,
		&val,
		PMIC_DA_QI_VCAMIO_STB_MASK,
		PMIC_DA_QI_VCAMIO_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vcamio_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCAMIO_EN_ADDR,
		&val,
		PMIC_DA_QI_VCAMIO_EN_MASK,
		PMIC_DA_QI_VCAMIO_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamio_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMIO_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMIO_OCFB_EN_MASK,
		PMIC_RG_LDO_VCAMIO_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcamio_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMIO_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMIO_OCFB_EN_MASK,
		PMIC_RG_LDO_VCAMIO_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vcamio_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCAMIO_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VCAMIO_OCFB_EN_MASK,
		PMIC_DA_QI_VCAMIO_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamio_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMIO_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VCAMIO_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VCAMIO_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vcamio_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCAMIO_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VCAMIO_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VCAMIO_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmipi_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMIPI_EN_ADDR,
		val,
		PMIC_RG_LDO_VMIPI_EN_MASK,
		PMIC_RG_LDO_VMIPI_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vmipi_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VMIPI_EN_ADDR,
		&val,
		PMIC_RG_LDO_VMIPI_EN_MASK,
		PMIC_RG_LDO_VMIPI_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmipi_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMIPI_LP_ADDR,
		val,
		PMIC_RG_LDO_VMIPI_LP_MASK,
		PMIC_RG_LDO_VMIPI_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vmipi_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMIPI_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VMIPI_SW_OP_EN_MASK,
		PMIC_RG_LDO_VMIPI_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vmipi_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VMIPI_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VMIPI_SW_OP_EN_MASK,
		PMIC_RG_LDO_VMIPI_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmipi_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMIPI_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VMIPI_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VMIPI_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vmipi_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VMIPI_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VMIPI_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VMIPI_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmipi_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMIPI_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VMIPI_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VMIPI_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vmipi_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VMIPI_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VMIPI_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VMIPI_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmipi_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMIPI_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VMIPI_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VMIPI_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vmipi_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VMIPI_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VMIPI_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VMIPI_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmipi_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMIPI_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VMIPI_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VMIPI_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vmipi_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VMIPI_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VMIPI_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VMIPI_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmipi_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMIPI_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VMIPI_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VMIPI_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vmipi_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VMIPI_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VMIPI_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VMIPI_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmipi_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMIPI_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VMIPI_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VMIPI_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vmipi_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VMIPI_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VMIPI_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VMIPI_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmipi_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMIPI_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VMIPI_ON_OP_MASK,
		PMIC_RG_LDO_VMIPI_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vmipi_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMIPI_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VMIPI_LP_OP_MASK,
		PMIC_RG_LDO_VMIPI_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vmipi_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VMIPI_MODE_ADDR,
		&val,
		PMIC_DA_QI_VMIPI_MODE_MASK,
		PMIC_DA_QI_VMIPI_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmipi_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMIPI_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VMIPI_STBTD_MASK,
		PMIC_RG_LDO_VMIPI_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vmipi_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VMIPI_STB_ADDR,
		&val,
		PMIC_DA_QI_VMIPI_STB_MASK,
		PMIC_DA_QI_VMIPI_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vmipi_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VMIPI_EN_ADDR,
		&val,
		PMIC_DA_QI_VMIPI_EN_MASK,
		PMIC_DA_QI_VMIPI_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmipi_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMIPI_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VMIPI_OCFB_EN_MASK,
		PMIC_RG_LDO_VMIPI_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vmipi_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VMIPI_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VMIPI_OCFB_EN_MASK,
		PMIC_RG_LDO_VMIPI_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vmipi_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VMIPI_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VMIPI_OCFB_EN_MASK,
		PMIC_DA_QI_VMIPI_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmipi_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMIPI_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VMIPI_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VMIPI_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vmipi_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VMIPI_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VMIPI_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VMIPI_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP_EN_ADDR,
		val,
		PMIC_RG_LDO_VGP_EN_MASK,
		PMIC_RG_LDO_VGP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vgp_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VGP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VGP_EN_MASK,
		PMIC_RG_LDO_VGP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP_LP_ADDR,
		val,
		PMIC_RG_LDO_VGP_LP_MASK,
		PMIC_RG_LDO_VGP_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VGP_SW_OP_EN_MASK,
		PMIC_RG_LDO_VGP_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vgp_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VGP_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VGP_SW_OP_EN_MASK,
		PMIC_RG_LDO_VGP_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VGP_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VGP_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vgp_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VGP_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VGP_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VGP_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VGP_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VGP_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vgp_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VGP_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VGP_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VGP_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VGP_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VGP_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vgp_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VGP_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VGP_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VGP_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VGP_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VGP_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vgp_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VGP_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VGP_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VGP_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VGP_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VGP_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vgp_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VGP_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VGP_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VGP_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VGP_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VGP_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vgp_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VGP_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VGP_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VGP_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VGP_ON_OP_MASK,
		PMIC_RG_LDO_VGP_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VGP_LP_OP_MASK,
		PMIC_RG_LDO_VGP_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vgp_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VGP_MODE_ADDR,
		&val,
		PMIC_DA_QI_VGP_MODE_MASK,
		PMIC_DA_QI_VGP_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VGP_STBTD_MASK,
		PMIC_RG_LDO_VGP_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vgp_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VGP_STB_ADDR,
		&val,
		PMIC_DA_QI_VGP_STB_MASK,
		PMIC_DA_QI_VGP_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vgp_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VGP_EN_ADDR,
		&val,
		PMIC_DA_QI_VGP_EN_MASK,
		PMIC_DA_QI_VGP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VGP_OCFB_EN_MASK,
		PMIC_RG_LDO_VGP_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vgp_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VGP_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VGP_OCFB_EN_MASK,
		PMIC_RG_LDO_VGP_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vgp_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VGP_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VGP_OCFB_EN_MASK,
		PMIC_DA_QI_VGP_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VGP_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VGP_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vgp_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VGP_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VGP_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VGP_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn33_bt_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN33_EN_BT_ADDR,
		val,
		PMIC_RG_LDO_VCN33_EN_BT_MASK,
		PMIC_RG_LDO_VCN33_EN_BT_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn33_bt_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN33_EN_BT_ADDR,
		&val,
		PMIC_RG_LDO_VCN33_EN_BT_MASK,
		PMIC_RG_LDO_VCN33_EN_BT_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn33_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN33_LP_ADDR,
		val,
		PMIC_RG_LDO_VCN33_LP_MASK,
		PMIC_RG_LDO_VCN33_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn33_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN33_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCN33_SW_OP_EN_MASK,
		PMIC_RG_LDO_VCN33_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn33_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN33_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCN33_SW_OP_EN_MASK,
		PMIC_RG_LDO_VCN33_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn33_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN33_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCN33_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VCN33_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn33_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN33_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCN33_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VCN33_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn33_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN33_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCN33_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VCN33_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn33_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN33_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCN33_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VCN33_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn33_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN33_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCN33_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VCN33_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn33_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN33_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCN33_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VCN33_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn33_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN33_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VCN33_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VCN33_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn33_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN33_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VCN33_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VCN33_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn33_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN33_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VCN33_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VCN33_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn33_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN33_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VCN33_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VCN33_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn33_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN33_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VCN33_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VCN33_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn33_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN33_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VCN33_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VCN33_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn33_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN33_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VCN33_ON_OP_MASK,
		PMIC_RG_LDO_VCN33_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn33_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN33_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VCN33_LP_OP_MASK,
		PMIC_RG_LDO_VCN33_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn33_wifi_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN33_EN_WIFI_ADDR,
		val,
		PMIC_RG_LDO_VCN33_EN_WIFI_MASK,
		PMIC_RG_LDO_VCN33_EN_WIFI_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn33_wifi_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN33_EN_WIFI_ADDR,
		&val,
		PMIC_RG_LDO_VCN33_EN_WIFI_MASK,
		PMIC_RG_LDO_VCN33_EN_WIFI_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vcn33_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCN33_MODE_ADDR,
		&val,
		PMIC_DA_QI_VCN33_MODE_MASK,
		PMIC_DA_QI_VCN33_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn33_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN33_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VCN33_STBTD_MASK,
		PMIC_RG_LDO_VCN33_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vcn33_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCN33_STB_ADDR,
		&val,
		PMIC_DA_QI_VCN33_STB_MASK,
		PMIC_DA_QI_VCN33_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vcn33_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCN33_EN_ADDR,
		&val,
		PMIC_DA_QI_VCN33_EN_MASK,
		PMIC_DA_QI_VCN33_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vcn33_bt_en(void)
{
	return mt6355_upmu_get_da_qi_vcn33_en();
}

unsigned int mt6355_upmu_get_da_qi_vcn33_wifi_en(void)
{
	return mt6355_upmu_get_da_qi_vcn33_en();
}

unsigned int mt6355_upmu_set_rg_ldo_vcn33_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN33_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VCN33_OCFB_EN_MASK,
		PMIC_RG_LDO_VCN33_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn33_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN33_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCN33_OCFB_EN_MASK,
		PMIC_RG_LDO_VCN33_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vcn33_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCN33_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VCN33_OCFB_EN_MASK,
		PMIC_DA_QI_VCN33_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn33_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN33_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VCN33_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VCN33_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vcn33_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCN33_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VCN33_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VCN33_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn18_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN18_EN_ADDR,
		val,
		PMIC_RG_LDO_VCN18_EN_MASK,
		PMIC_RG_LDO_VCN18_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn18_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN18_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCN18_EN_MASK,
		PMIC_RG_LDO_VCN18_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn18_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN18_LP_ADDR,
		val,
		PMIC_RG_LDO_VCN18_LP_MASK,
		PMIC_RG_LDO_VCN18_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn18_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN18_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCN18_SW_OP_EN_MASK,
		PMIC_RG_LDO_VCN18_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn18_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN18_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCN18_SW_OP_EN_MASK,
		PMIC_RG_LDO_VCN18_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn18_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN18_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCN18_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VCN18_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn18_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN18_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCN18_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VCN18_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn18_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN18_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCN18_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VCN18_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn18_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN18_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCN18_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VCN18_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn18_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN18_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCN18_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VCN18_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn18_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN18_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCN18_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VCN18_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn18_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN18_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VCN18_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VCN18_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn18_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN18_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VCN18_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VCN18_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn18_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN18_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VCN18_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VCN18_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn18_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN18_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VCN18_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VCN18_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn18_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN18_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VCN18_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VCN18_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn18_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN18_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VCN18_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VCN18_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn18_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN18_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VCN18_ON_OP_MASK,
		PMIC_RG_LDO_VCN18_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn18_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN18_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VCN18_LP_OP_MASK,
		PMIC_RG_LDO_VCN18_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vcn18_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCN18_MODE_ADDR,
		&val,
		PMIC_DA_QI_VCN18_MODE_MASK,
		PMIC_DA_QI_VCN18_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn18_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN18_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VCN18_STBTD_MASK,
		PMIC_RG_LDO_VCN18_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vcn18_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCN18_STB_ADDR,
		&val,
		PMIC_DA_QI_VCN18_STB_MASK,
		PMIC_DA_QI_VCN18_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vcn18_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCN18_EN_ADDR,
		&val,
		PMIC_DA_QI_VCN18_EN_MASK,
		PMIC_DA_QI_VCN18_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn18_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN18_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VCN18_OCFB_EN_MASK,
		PMIC_RG_LDO_VCN18_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn18_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN18_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCN18_OCFB_EN_MASK,
		PMIC_RG_LDO_VCN18_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vcn18_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCN18_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VCN18_OCFB_EN_MASK,
		PMIC_DA_QI_VCN18_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn18_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN18_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VCN18_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VCN18_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vcn18_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCN18_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VCN18_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VCN18_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn28_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN28_EN_ADDR,
		val,
		PMIC_RG_LDO_VCN28_EN_MASK,
		PMIC_RG_LDO_VCN28_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn28_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN28_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCN28_EN_MASK,
		PMIC_RG_LDO_VCN28_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn28_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN28_LP_ADDR,
		val,
		PMIC_RG_LDO_VCN28_LP_MASK,
		PMIC_RG_LDO_VCN28_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn28_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN28_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCN28_SW_OP_EN_MASK,
		PMIC_RG_LDO_VCN28_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn28_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN28_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCN28_SW_OP_EN_MASK,
		PMIC_RG_LDO_VCN28_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn28_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN28_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCN28_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VCN28_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn28_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN28_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCN28_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VCN28_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn28_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN28_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCN28_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VCN28_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn28_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN28_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCN28_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VCN28_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn28_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN28_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCN28_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VCN28_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn28_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN28_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCN28_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VCN28_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn28_hw3_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN28_HW3_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCN28_HW3_OP_EN_MASK,
		PMIC_RG_LDO_VCN28_HW3_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn28_hw3_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN28_HW3_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCN28_HW3_OP_EN_MASK,
		PMIC_RG_LDO_VCN28_HW3_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn28_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN28_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VCN28_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VCN28_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn28_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN28_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VCN28_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VCN28_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn28_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN28_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VCN28_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VCN28_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn28_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN28_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VCN28_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VCN28_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn28_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN28_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VCN28_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VCN28_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn28_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN28_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VCN28_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VCN28_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn28_hw3_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN28_HW3_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VCN28_HW3_OP_CFG_MASK,
		PMIC_RG_LDO_VCN28_HW3_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn28_hw3_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN28_HW3_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VCN28_HW3_OP_CFG_MASK,
		PMIC_RG_LDO_VCN28_HW3_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn28_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN28_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VCN28_ON_OP_MASK,
		PMIC_RG_LDO_VCN28_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn28_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN28_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VCN28_LP_OP_MASK,
		PMIC_RG_LDO_VCN28_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vcn28_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCN28_MODE_ADDR,
		&val,
		PMIC_DA_QI_VCN28_MODE_MASK,
		PMIC_DA_QI_VCN28_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn28_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN28_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VCN28_STBTD_MASK,
		PMIC_RG_LDO_VCN28_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vcn28_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCN28_STB_ADDR,
		&val,
		PMIC_DA_QI_VCN28_STB_MASK,
		PMIC_DA_QI_VCN28_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vcn28_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCN28_EN_ADDR,
		&val,
		PMIC_DA_QI_VCN28_EN_MASK,
		PMIC_DA_QI_VCN28_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn28_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN28_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VCN28_OCFB_EN_MASK,
		PMIC_RG_LDO_VCN28_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn28_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN28_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCN28_OCFB_EN_MASK,
		PMIC_RG_LDO_VCN28_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vcn28_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCN28_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VCN28_OCFB_EN_MASK,
		PMIC_DA_QI_VCN28_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn28_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN28_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VCN28_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VCN28_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vcn28_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCN28_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VCN28_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VCN28_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vbif28_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VBIF28_EN_ADDR,
		val,
		PMIC_RG_LDO_VBIF28_EN_MASK,
		PMIC_RG_LDO_VBIF28_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vbif28_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VBIF28_EN_ADDR,
		&val,
		PMIC_RG_LDO_VBIF28_EN_MASK,
		PMIC_RG_LDO_VBIF28_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vbif28_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VBIF28_LP_ADDR,
		val,
		PMIC_RG_LDO_VBIF28_LP_MASK,
		PMIC_RG_LDO_VBIF28_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vbif28_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VBIF28_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VBIF28_SW_OP_EN_MASK,
		PMIC_RG_LDO_VBIF28_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vbif28_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VBIF28_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VBIF28_SW_OP_EN_MASK,
		PMIC_RG_LDO_VBIF28_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vbif28_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VBIF28_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VBIF28_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VBIF28_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vbif28_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VBIF28_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VBIF28_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VBIF28_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vbif28_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VBIF28_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VBIF28_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VBIF28_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vbif28_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VBIF28_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VBIF28_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VBIF28_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vbif28_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VBIF28_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VBIF28_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VBIF28_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vbif28_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VBIF28_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VBIF28_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VBIF28_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vbif28_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VBIF28_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VBIF28_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VBIF28_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vbif28_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VBIF28_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VBIF28_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VBIF28_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vbif28_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VBIF28_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VBIF28_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VBIF28_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vbif28_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VBIF28_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VBIF28_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VBIF28_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vbif28_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VBIF28_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VBIF28_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VBIF28_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vbif28_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VBIF28_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VBIF28_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VBIF28_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vbif28_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VBIF28_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VBIF28_ON_OP_MASK,
		PMIC_RG_LDO_VBIF28_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vbif28_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VBIF28_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VBIF28_LP_OP_MASK,
		PMIC_RG_LDO_VBIF28_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vbif28_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VBIF28_MODE_ADDR,
		&val,
		PMIC_DA_QI_VBIF28_MODE_MASK,
		PMIC_DA_QI_VBIF28_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vbif28_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VBIF28_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VBIF28_STBTD_MASK,
		PMIC_RG_LDO_VBIF28_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vbif28_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VBIF28_STB_ADDR,
		&val,
		PMIC_DA_QI_VBIF28_STB_MASK,
		PMIC_DA_QI_VBIF28_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vbif28_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VBIF28_EN_ADDR,
		&val,
		PMIC_DA_QI_VBIF28_EN_MASK,
		PMIC_DA_QI_VBIF28_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vbif28_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VBIF28_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VBIF28_OCFB_EN_MASK,
		PMIC_RG_LDO_VBIF28_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vbif28_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VBIF28_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VBIF28_OCFB_EN_MASK,
		PMIC_RG_LDO_VBIF28_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vbif28_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VBIF28_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VBIF28_OCFB_EN_MASK,
		PMIC_DA_QI_VBIF28_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vbif28_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VBIF28_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VBIF28_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VBIF28_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vbif28_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VBIF28_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VBIF28_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VBIF28_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vtcxo24_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VTCXO24_EN_ADDR,
		val,
		PMIC_RG_LDO_VTCXO24_EN_MASK,
		PMIC_RG_LDO_VTCXO24_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vtcxo24_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VTCXO24_EN_ADDR,
		&val,
		PMIC_RG_LDO_VTCXO24_EN_MASK,
		PMIC_RG_LDO_VTCXO24_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vtcxo24_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VTCXO24_LP_ADDR,
		val,
		PMIC_RG_LDO_VTCXO24_LP_MASK,
		PMIC_RG_LDO_VTCXO24_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vtcxo24_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VTCXO24_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VTCXO24_SW_OP_EN_MASK,
		PMIC_RG_LDO_VTCXO24_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vtcxo24_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VTCXO24_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VTCXO24_SW_OP_EN_MASK,
		PMIC_RG_LDO_VTCXO24_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vtcxo24_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VTCXO24_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VTCXO24_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VTCXO24_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vtcxo24_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VTCXO24_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VTCXO24_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VTCXO24_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vtcxo24_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VTCXO24_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VTCXO24_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VTCXO24_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vtcxo24_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VTCXO24_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VTCXO24_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VTCXO24_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vtcxo24_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VTCXO24_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VTCXO24_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VTCXO24_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vtcxo24_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VTCXO24_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VTCXO24_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VTCXO24_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vtcxo24_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VTCXO24_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VTCXO24_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VTCXO24_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vtcxo24_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VTCXO24_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VTCXO24_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VTCXO24_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vtcxo24_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VTCXO24_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VTCXO24_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VTCXO24_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vtcxo24_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VTCXO24_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VTCXO24_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VTCXO24_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vtcxo24_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VTCXO24_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VTCXO24_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VTCXO24_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vtcxo24_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VTCXO24_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VTCXO24_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VTCXO24_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vtcxo24_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VTCXO24_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VTCXO24_ON_OP_MASK,
		PMIC_RG_LDO_VTCXO24_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vtcxo24_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VTCXO24_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VTCXO24_LP_OP_MASK,
		PMIC_RG_LDO_VTCXO24_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vtcxo24_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VTCXO24_MODE_ADDR,
		&val,
		PMIC_DA_QI_VTCXO24_MODE_MASK,
		PMIC_DA_QI_VTCXO24_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vtcxo24_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VTCXO24_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VTCXO24_STBTD_MASK,
		PMIC_RG_LDO_VTCXO24_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vtcxo24_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VTCXO24_STB_ADDR,
		&val,
		PMIC_DA_QI_VTCXO24_STB_MASK,
		PMIC_DA_QI_VTCXO24_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vtcxo24_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VTCXO24_EN_ADDR,
		&val,
		PMIC_DA_QI_VTCXO24_EN_MASK,
		PMIC_DA_QI_VTCXO24_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vtcxo24_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VTCXO24_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VTCXO24_OCFB_EN_MASK,
		PMIC_RG_LDO_VTCXO24_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vtcxo24_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VTCXO24_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VTCXO24_OCFB_EN_MASK,
		PMIC_RG_LDO_VTCXO24_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vtcxo24_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VTCXO24_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VTCXO24_OCFB_EN_MASK,
		PMIC_DA_QI_VTCXO24_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vtcxo24_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VTCXO24_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VTCXO24_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VTCXO24_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vtcxo24_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VTCXO24_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VTCXO24_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VTCXO24_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vldo28_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VLDO28_EN_AF_ADDR,
		val,
		PMIC_RG_LDO_VLDO28_EN_AF_MASK,
		PMIC_RG_LDO_VLDO28_EN_AF_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vldo28_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VLDO28_EN_AF_ADDR,
		&val,
		PMIC_RG_LDO_VLDO28_EN_AF_MASK,
		PMIC_RG_LDO_VLDO28_EN_AF_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vldo28_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VLDO28_LP_ADDR,
		val,
		PMIC_RG_LDO_VLDO28_LP_MASK,
		PMIC_RG_LDO_VLDO28_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vldo28_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VLDO28_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VLDO28_SW_OP_EN_MASK,
		PMIC_RG_LDO_VLDO28_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vldo28_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VLDO28_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VLDO28_SW_OP_EN_MASK,
		PMIC_RG_LDO_VLDO28_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vldo28_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VLDO28_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VLDO28_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VLDO28_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vldo28_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VLDO28_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VLDO28_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VLDO28_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vldo28_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VLDO28_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VLDO28_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VLDO28_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vldo28_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VLDO28_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VLDO28_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VLDO28_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vldo28_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VLDO28_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VLDO28_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VLDO28_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vldo28_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VLDO28_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VLDO28_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VLDO28_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vldo28_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VLDO28_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VLDO28_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VLDO28_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vldo28_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VLDO28_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VLDO28_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VLDO28_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vldo28_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VLDO28_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VLDO28_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VLDO28_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vldo28_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VLDO28_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VLDO28_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VLDO28_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vldo28_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VLDO28_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VLDO28_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VLDO28_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vldo28_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VLDO28_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VLDO28_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VLDO28_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vldo28_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VLDO28_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VLDO28_ON_OP_MASK,
		PMIC_RG_LDO_VLDO28_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vldo28_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VLDO28_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VLDO28_LP_OP_MASK,
		PMIC_RG_LDO_VLDO28_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vldo28_tp_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VLDO28_EN_TP_ADDR,
		val,
		PMIC_RG_LDO_VLDO28_EN_TP_MASK,
		PMIC_RG_LDO_VLDO28_EN_TP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vldo28_tp_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VLDO28_EN_TP_ADDR,
		&val,
		PMIC_RG_LDO_VLDO28_EN_TP_MASK,
		PMIC_RG_LDO_VLDO28_EN_TP_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vldo28_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VLDO28_MODE_ADDR,
		&val,
		PMIC_DA_QI_VLDO28_MODE_MASK,
		PMIC_DA_QI_VLDO28_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vldo28_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VLDO28_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VLDO28_STBTD_MASK,
		PMIC_RG_LDO_VLDO28_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vldo28_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VLDO28_STB_ADDR,
		&val,
		PMIC_DA_QI_VLDO28_STB_MASK,
		PMIC_DA_QI_VLDO28_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vldo28_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VLDO28_EN_ADDR,
		&val,
		PMIC_DA_QI_VLDO28_EN_MASK,
		PMIC_DA_QI_VLDO28_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vldo28_tp_en(void)
{
	return mt6355_upmu_get_da_qi_vldo28_en();
}

unsigned int mt6355_upmu_set_rg_ldo_vldo28_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VLDO28_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VLDO28_OCFB_EN_MASK,
		PMIC_RG_LDO_VLDO28_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vldo28_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VLDO28_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VLDO28_OCFB_EN_MASK,
		PMIC_RG_LDO_VLDO28_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vldo28_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VLDO28_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VLDO28_OCFB_EN_MASK,
		PMIC_DA_QI_VLDO28_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vldo28_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VLDO28_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VLDO28_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VLDO28_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vldo28_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VLDO28_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VLDO28_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VLDO28_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp2_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP2_EN_ADDR,
		val,
		PMIC_RG_LDO_VGP2_EN_MASK,
		PMIC_RG_LDO_VGP2_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vgp2_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VGP2_EN_ADDR,
		&val,
		PMIC_RG_LDO_VGP2_EN_MASK,
		PMIC_RG_LDO_VGP2_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp2_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP2_LP_ADDR,
		val,
		PMIC_RG_LDO_VGP2_LP_MASK,
		PMIC_RG_LDO_VGP2_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp2_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP2_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VGP2_SW_OP_EN_MASK,
		PMIC_RG_LDO_VGP2_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vgp2_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VGP2_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VGP2_SW_OP_EN_MASK,
		PMIC_RG_LDO_VGP2_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp2_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP2_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VGP2_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VGP2_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vgp2_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VGP2_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VGP2_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VGP2_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp2_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP2_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VGP2_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VGP2_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vgp2_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VGP2_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VGP2_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VGP2_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp2_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP2_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VGP2_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VGP2_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vgp2_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VGP2_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VGP2_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VGP2_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp2_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP2_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VGP2_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VGP2_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vgp2_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VGP2_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VGP2_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VGP2_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp2_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP2_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VGP2_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VGP2_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vgp2_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VGP2_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VGP2_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VGP2_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp2_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP2_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VGP2_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VGP2_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vgp2_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VGP2_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VGP2_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VGP2_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp2_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP2_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VGP2_ON_OP_MASK,
		PMIC_RG_LDO_VGP2_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp2_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP2_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VGP2_LP_OP_MASK,
		PMIC_RG_LDO_VGP2_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vgp2_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VGP2_MODE_ADDR,
		&val,
		PMIC_DA_QI_VGP2_MODE_MASK,
		PMIC_DA_QI_VGP2_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp2_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP2_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VGP2_STBTD_MASK,
		PMIC_RG_LDO_VGP2_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vgp2_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VGP2_STB_ADDR,
		&val,
		PMIC_DA_QI_VGP2_STB_MASK,
		PMIC_DA_QI_VGP2_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vgp2_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VGP2_EN_ADDR,
		&val,
		PMIC_DA_QI_VGP2_EN_MASK,
		PMIC_DA_QI_VGP2_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp2_ther_sdn_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP2_THER_SDN_EN_ADDR,
		val,
		PMIC_RG_LDO_VGP2_THER_SDN_EN_MASK,
		PMIC_RG_LDO_VGP2_THER_SDN_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vgp2_ther_sdn_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VGP2_THER_SDN_EN_ADDR,
		&val,
		PMIC_RG_LDO_VGP2_THER_SDN_EN_MASK,
		PMIC_RG_LDO_VGP2_THER_SDN_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp2_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP2_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VGP2_OCFB_EN_MASK,
		PMIC_RG_LDO_VGP2_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vgp2_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VGP2_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VGP2_OCFB_EN_MASK,
		PMIC_RG_LDO_VGP2_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vgp2_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VGP2_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VGP2_OCFB_EN_MASK,
		PMIC_DA_QI_VGP2_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp2_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP2_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VGP2_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VGP2_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vgp2_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VGP2_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VGP2_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VGP2_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vfe28_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VFE28_EN_ADDR,
		val,
		PMIC_RG_LDO_VFE28_EN_MASK,
		PMIC_RG_LDO_VFE28_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vfe28_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VFE28_EN_ADDR,
		&val,
		PMIC_RG_LDO_VFE28_EN_MASK,
		PMIC_RG_LDO_VFE28_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vfe28_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VFE28_LP_ADDR,
		val,
		PMIC_RG_LDO_VFE28_LP_MASK,
		PMIC_RG_LDO_VFE28_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vfe28_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VFE28_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VFE28_SW_OP_EN_MASK,
		PMIC_RG_LDO_VFE28_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vfe28_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VFE28_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VFE28_SW_OP_EN_MASK,
		PMIC_RG_LDO_VFE28_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vfe28_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VFE28_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VFE28_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VFE28_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vfe28_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VFE28_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VFE28_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VFE28_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vfe28_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VFE28_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VFE28_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VFE28_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vfe28_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VFE28_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VFE28_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VFE28_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vfe28_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VFE28_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VFE28_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VFE28_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vfe28_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VFE28_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VFE28_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VFE28_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vfe28_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VFE28_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VFE28_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VFE28_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vfe28_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VFE28_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VFE28_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VFE28_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vfe28_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VFE28_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VFE28_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VFE28_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vfe28_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VFE28_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VFE28_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VFE28_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vfe28_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VFE28_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VFE28_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VFE28_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vfe28_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VFE28_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VFE28_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VFE28_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vfe28_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VFE28_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VFE28_ON_OP_MASK,
		PMIC_RG_LDO_VFE28_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vfe28_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VFE28_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VFE28_LP_OP_MASK,
		PMIC_RG_LDO_VFE28_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vfe28_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VFE28_MODE_ADDR,
		&val,
		PMIC_DA_QI_VFE28_MODE_MASK,
		PMIC_DA_QI_VFE28_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vfe28_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VFE28_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VFE28_STBTD_MASK,
		PMIC_RG_LDO_VFE28_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vfe28_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VFE28_STB_ADDR,
		&val,
		PMIC_DA_QI_VFE28_STB_MASK,
		PMIC_DA_QI_VFE28_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vfe28_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VFE28_EN_ADDR,
		&val,
		PMIC_DA_QI_VFE28_EN_MASK,
		PMIC_DA_QI_VFE28_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vfe28_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VFE28_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VFE28_OCFB_EN_MASK,
		PMIC_RG_LDO_VFE28_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vfe28_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VFE28_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VFE28_OCFB_EN_MASK,
		PMIC_RG_LDO_VFE28_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vfe28_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VFE28_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VFE28_OCFB_EN_MASK,
		PMIC_DA_QI_VFE28_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vfe28_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VFE28_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VFE28_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VFE28_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vfe28_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VFE28_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VFE28_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VFE28_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmch_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMCH_EN_ADDR,
		val,
		PMIC_RG_LDO_VMCH_EN_MASK,
		PMIC_RG_LDO_VMCH_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vmch_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VMCH_EN_ADDR,
		&val,
		PMIC_RG_LDO_VMCH_EN_MASK,
		PMIC_RG_LDO_VMCH_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmch_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMCH_LP_ADDR,
		val,
		PMIC_RG_LDO_VMCH_LP_MASK,
		PMIC_RG_LDO_VMCH_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vmch_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMCH_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VMCH_SW_OP_EN_MASK,
		PMIC_RG_LDO_VMCH_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vmch_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VMCH_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VMCH_SW_OP_EN_MASK,
		PMIC_RG_LDO_VMCH_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmch_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMCH_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VMCH_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VMCH_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vmch_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VMCH_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VMCH_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VMCH_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmch_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMCH_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VMCH_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VMCH_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vmch_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VMCH_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VMCH_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VMCH_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmch_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMCH_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VMCH_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VMCH_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vmch_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VMCH_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VMCH_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VMCH_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmch_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMCH_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VMCH_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VMCH_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vmch_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VMCH_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VMCH_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VMCH_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmch_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMCH_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VMCH_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VMCH_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vmch_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VMCH_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VMCH_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VMCH_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmch_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMCH_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VMCH_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VMCH_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vmch_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VMCH_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VMCH_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VMCH_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmch_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMCH_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VMCH_ON_OP_MASK,
		PMIC_RG_LDO_VMCH_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vmch_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMCH_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VMCH_LP_OP_MASK,
		PMIC_RG_LDO_VMCH_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vmch_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VMCH_MODE_ADDR,
		&val,
		PMIC_DA_QI_VMCH_MODE_MASK,
		PMIC_DA_QI_VMCH_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmch_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMCH_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VMCH_STBTD_MASK,
		PMIC_RG_LDO_VMCH_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vmch_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VMCH_STB_ADDR,
		&val,
		PMIC_DA_QI_VMCH_STB_MASK,
		PMIC_DA_QI_VMCH_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vmch_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VMCH_EN_ADDR,
		&val,
		PMIC_DA_QI_VMCH_EN_MASK,
		PMIC_DA_QI_VMCH_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmch_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMCH_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VMCH_OCFB_EN_MASK,
		PMIC_RG_LDO_VMCH_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vmch_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VMCH_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VMCH_OCFB_EN_MASK,
		PMIC_RG_LDO_VMCH_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vmch_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VMCH_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VMCH_OCFB_EN_MASK,
		PMIC_DA_QI_VMCH_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmch_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMCH_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VMCH_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VMCH_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vmch_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VMCH_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VMCH_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VMCH_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmc_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMC_EN_ADDR,
		val,
		PMIC_RG_LDO_VMC_EN_MASK,
		PMIC_RG_LDO_VMC_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vmc_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VMC_EN_ADDR,
		&val,
		PMIC_RG_LDO_VMC_EN_MASK,
		PMIC_RG_LDO_VMC_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmc_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMC_LP_ADDR,
		val,
		PMIC_RG_LDO_VMC_LP_MASK,
		PMIC_RG_LDO_VMC_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vmc_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMC_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VMC_SW_OP_EN_MASK,
		PMIC_RG_LDO_VMC_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vmc_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VMC_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VMC_SW_OP_EN_MASK,
		PMIC_RG_LDO_VMC_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmc_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMC_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VMC_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VMC_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vmc_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VMC_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VMC_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VMC_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmc_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMC_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VMC_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VMC_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vmc_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VMC_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VMC_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VMC_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmc_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMC_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VMC_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VMC_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vmc_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VMC_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VMC_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VMC_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmc_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMC_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VMC_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VMC_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vmc_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VMC_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VMC_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VMC_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmc_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMC_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VMC_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VMC_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vmc_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VMC_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VMC_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VMC_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmc_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMC_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VMC_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VMC_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vmc_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VMC_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VMC_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VMC_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmc_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMC_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VMC_ON_OP_MASK,
		PMIC_RG_LDO_VMC_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vmc_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMC_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VMC_LP_OP_MASK,
		PMIC_RG_LDO_VMC_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vmc_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VMC_MODE_ADDR,
		&val,
		PMIC_DA_QI_VMC_MODE_MASK,
		PMIC_DA_QI_VMC_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmc_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMC_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VMC_STBTD_MASK,
		PMIC_RG_LDO_VMC_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vmc_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VMC_STB_ADDR,
		&val,
		PMIC_DA_QI_VMC_STB_MASK,
		PMIC_DA_QI_VMC_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vmc_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VMC_EN_ADDR,
		&val,
		PMIC_DA_QI_VMC_EN_MASK,
		PMIC_DA_QI_VMC_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmc_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMC_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VMC_OCFB_EN_MASK,
		PMIC_RG_LDO_VMC_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vmc_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VMC_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VMC_OCFB_EN_MASK,
		PMIC_RG_LDO_VMC_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vmc_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VMC_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VMC_OCFB_EN_MASK,
		PMIC_DA_QI_VMC_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmc_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMC_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VMC_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VMC_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vmc_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VMC_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VMC_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VMC_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_1_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_1_EN_ADDR,
		val,
		PMIC_RG_LDO_VRF18_1_EN_MASK,
		PMIC_RG_LDO_VRF18_1_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vrf18_1_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VRF18_1_EN_ADDR,
		&val,
		PMIC_RG_LDO_VRF18_1_EN_MASK,
		PMIC_RG_LDO_VRF18_1_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_1_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_1_LP_ADDR,
		val,
		PMIC_RG_LDO_VRF18_1_LP_MASK,
		PMIC_RG_LDO_VRF18_1_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_1_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_1_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VRF18_1_SW_OP_EN_MASK,
		PMIC_RG_LDO_VRF18_1_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vrf18_1_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VRF18_1_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VRF18_1_SW_OP_EN_MASK,
		PMIC_RG_LDO_VRF18_1_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_1_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_1_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VRF18_1_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VRF18_1_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vrf18_1_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VRF18_1_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VRF18_1_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VRF18_1_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_1_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_1_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VRF18_1_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VRF18_1_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vrf18_1_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VRF18_1_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VRF18_1_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VRF18_1_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_1_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_1_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VRF18_1_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VRF18_1_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vrf18_1_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VRF18_1_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VRF18_1_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VRF18_1_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_1_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_1_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VRF18_1_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VRF18_1_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vrf18_1_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VRF18_1_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VRF18_1_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VRF18_1_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_1_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_1_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VRF18_1_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VRF18_1_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vrf18_1_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VRF18_1_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VRF18_1_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VRF18_1_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_1_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_1_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VRF18_1_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VRF18_1_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vrf18_1_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VRF18_1_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VRF18_1_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VRF18_1_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_1_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_1_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VRF18_1_ON_OP_MASK,
		PMIC_RG_LDO_VRF18_1_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_1_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_1_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VRF18_1_LP_OP_MASK,
		PMIC_RG_LDO_VRF18_1_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vrf18_1_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VRF18_1_MODE_ADDR,
		&val,
		PMIC_DA_QI_VRF18_1_MODE_MASK,
		PMIC_DA_QI_VRF18_1_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_1_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_1_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VRF18_1_STBTD_MASK,
		PMIC_RG_LDO_VRF18_1_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vrf18_1_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VRF18_1_STB_ADDR,
		&val,
		PMIC_DA_QI_VRF18_1_STB_MASK,
		PMIC_DA_QI_VRF18_1_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vrf18_1_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VRF18_1_EN_ADDR,
		&val,
		PMIC_DA_QI_VRF18_1_EN_MASK,
		PMIC_DA_QI_VRF18_1_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_1_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_1_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VRF18_1_OCFB_EN_MASK,
		PMIC_RG_LDO_VRF18_1_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vrf18_1_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VRF18_1_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VRF18_1_OCFB_EN_MASK,
		PMIC_RG_LDO_VRF18_1_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vrf18_1_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VRF18_1_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VRF18_1_OCFB_EN_MASK,
		PMIC_DA_QI_VRF18_1_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_1_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_1_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VRF18_1_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VRF18_1_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vrf18_1_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VRF18_1_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VRF18_1_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VRF18_1_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_2_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_2_EN_ADDR,
		val,
		PMIC_RG_LDO_VRF18_2_EN_MASK,
		PMIC_RG_LDO_VRF18_2_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vrf18_2_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VRF18_2_EN_ADDR,
		&val,
		PMIC_RG_LDO_VRF18_2_EN_MASK,
		PMIC_RG_LDO_VRF18_2_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_2_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_2_LP_ADDR,
		val,
		PMIC_RG_LDO_VRF18_2_LP_MASK,
		PMIC_RG_LDO_VRF18_2_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_2_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_2_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VRF18_2_SW_OP_EN_MASK,
		PMIC_RG_LDO_VRF18_2_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vrf18_2_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VRF18_2_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VRF18_2_SW_OP_EN_MASK,
		PMIC_RG_LDO_VRF18_2_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_2_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_2_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VRF18_2_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VRF18_2_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vrf18_2_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VRF18_2_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VRF18_2_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VRF18_2_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_2_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_2_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VRF18_2_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VRF18_2_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vrf18_2_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VRF18_2_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VRF18_2_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VRF18_2_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_2_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_2_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VRF18_2_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VRF18_2_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vrf18_2_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VRF18_2_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VRF18_2_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VRF18_2_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_2_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_2_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VRF18_2_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VRF18_2_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vrf18_2_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VRF18_2_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VRF18_2_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VRF18_2_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_2_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_2_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VRF18_2_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VRF18_2_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vrf18_2_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VRF18_2_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VRF18_2_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VRF18_2_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_2_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_2_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VRF18_2_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VRF18_2_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vrf18_2_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VRF18_2_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VRF18_2_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VRF18_2_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_2_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_2_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VRF18_2_ON_OP_MASK,
		PMIC_RG_LDO_VRF18_2_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_2_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_2_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VRF18_2_LP_OP_MASK,
		PMIC_RG_LDO_VRF18_2_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vrf18_2_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VRF18_2_MODE_ADDR,
		&val,
		PMIC_DA_QI_VRF18_2_MODE_MASK,
		PMIC_DA_QI_VRF18_2_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_2_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_2_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VRF18_2_STBTD_MASK,
		PMIC_RG_LDO_VRF18_2_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vrf18_2_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VRF18_2_STB_ADDR,
		&val,
		PMIC_DA_QI_VRF18_2_STB_MASK,
		PMIC_DA_QI_VRF18_2_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vrf18_2_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VRF18_2_EN_ADDR,
		&val,
		PMIC_DA_QI_VRF18_2_EN_MASK,
		PMIC_DA_QI_VRF18_2_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_2_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_2_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VRF18_2_OCFB_EN_MASK,
		PMIC_RG_LDO_VRF18_2_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vrf18_2_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VRF18_2_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VRF18_2_OCFB_EN_MASK,
		PMIC_RG_LDO_VRF18_2_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vrf18_2_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VRF18_2_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VRF18_2_OCFB_EN_MASK,
		PMIC_DA_QI_VRF18_2_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_2_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_2_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VRF18_2_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VRF18_2_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vrf18_2_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VRF18_2_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VRF18_2_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VRF18_2_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf12_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF12_EN_ADDR,
		val,
		PMIC_RG_LDO_VRF12_EN_MASK,
		PMIC_RG_LDO_VRF12_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vrf12_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VRF12_EN_ADDR,
		&val,
		PMIC_RG_LDO_VRF12_EN_MASK,
		PMIC_RG_LDO_VRF12_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf12_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF12_LP_ADDR,
		val,
		PMIC_RG_LDO_VRF12_LP_MASK,
		PMIC_RG_LDO_VRF12_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf12_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF12_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VRF12_SW_OP_EN_MASK,
		PMIC_RG_LDO_VRF12_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vrf12_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VRF12_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VRF12_SW_OP_EN_MASK,
		PMIC_RG_LDO_VRF12_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf12_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF12_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VRF12_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VRF12_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vrf12_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VRF12_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VRF12_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VRF12_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf12_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF12_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VRF12_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VRF12_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vrf12_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VRF12_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VRF12_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VRF12_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf12_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF12_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VRF12_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VRF12_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vrf12_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VRF12_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VRF12_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VRF12_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf12_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF12_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VRF12_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VRF12_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vrf12_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VRF12_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VRF12_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VRF12_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf12_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF12_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VRF12_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VRF12_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vrf12_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VRF12_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VRF12_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VRF12_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf12_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF12_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VRF12_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VRF12_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vrf12_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VRF12_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VRF12_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VRF12_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf12_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF12_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VRF12_ON_OP_MASK,
		PMIC_RG_LDO_VRF12_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf12_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF12_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VRF12_LP_OP_MASK,
		PMIC_RG_LDO_VRF12_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vrf12_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VRF12_MODE_ADDR,
		&val,
		PMIC_DA_QI_VRF12_MODE_MASK,
		PMIC_DA_QI_VRF12_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf12_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF12_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VRF12_STBTD_MASK,
		PMIC_RG_LDO_VRF12_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vrf12_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VRF12_STB_ADDR,
		&val,
		PMIC_DA_QI_VRF12_STB_MASK,
		PMIC_DA_QI_VRF12_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vrf12_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VRF12_EN_ADDR,
		&val,
		PMIC_DA_QI_VRF12_EN_MASK,
		PMIC_DA_QI_VRF12_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf12_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF12_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VRF12_OCFB_EN_MASK,
		PMIC_RG_LDO_VRF12_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vrf12_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VRF12_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VRF12_OCFB_EN_MASK,
		PMIC_RG_LDO_VRF12_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vrf12_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VRF12_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VRF12_OCFB_EN_MASK,
		PMIC_DA_QI_VRF12_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf12_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF12_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VRF12_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VRF12_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vrf12_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VRF12_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VRF12_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VRF12_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama1_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA1_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMA1_EN_MASK,
		PMIC_RG_LDO_VCAMA1_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcama1_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMA1_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMA1_EN_MASK,
		PMIC_RG_LDO_VCAMA1_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama1_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA1_LP_ADDR,
		val,
		PMIC_RG_LDO_VCAMA1_LP_MASK,
		PMIC_RG_LDO_VCAMA1_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama1_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA1_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMA1_SW_OP_EN_MASK,
		PMIC_RG_LDO_VCAMA1_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcama1_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMA1_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMA1_SW_OP_EN_MASK,
		PMIC_RG_LDO_VCAMA1_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama1_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA1_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMA1_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VCAMA1_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcama1_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMA1_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMA1_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VCAMA1_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama1_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA1_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMA1_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VCAMA1_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcama1_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMA1_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMA1_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VCAMA1_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama1_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA1_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMA1_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VCAMA1_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcama1_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMA1_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMA1_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VCAMA1_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama1_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA1_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VCAMA1_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VCAMA1_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcama1_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMA1_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VCAMA1_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VCAMA1_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama1_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA1_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VCAMA1_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VCAMA1_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcama1_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMA1_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VCAMA1_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VCAMA1_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama1_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA1_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VCAMA1_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VCAMA1_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcama1_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMA1_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VCAMA1_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VCAMA1_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama1_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA1_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VCAMA1_ON_OP_MASK,
		PMIC_RG_LDO_VCAMA1_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama1_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA1_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VCAMA1_LP_OP_MASK,
		PMIC_RG_LDO_VCAMA1_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vcama1_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCAMA1_MODE_ADDR,
		&val,
		PMIC_DA_QI_VCAMA1_MODE_MASK,
		PMIC_DA_QI_VCAMA1_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama1_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA1_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VCAMA1_STBTD_MASK,
		PMIC_RG_LDO_VCAMA1_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vcama1_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCAMA1_STB_ADDR,
		&val,
		PMIC_DA_QI_VCAMA1_STB_MASK,
		PMIC_DA_QI_VCAMA1_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vcama1_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCAMA1_EN_ADDR,
		&val,
		PMIC_DA_QI_VCAMA1_EN_MASK,
		PMIC_DA_QI_VCAMA1_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama1_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA1_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMA1_OCFB_EN_MASK,
		PMIC_RG_LDO_VCAMA1_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcama1_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMA1_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMA1_OCFB_EN_MASK,
		PMIC_RG_LDO_VCAMA1_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vcama1_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCAMA1_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VCAMA1_OCFB_EN_MASK,
		PMIC_DA_QI_VCAMA1_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama1_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA1_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VCAMA1_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VCAMA1_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vcama1_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCAMA1_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VCAMA1_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VCAMA1_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama2_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA2_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMA2_EN_MASK,
		PMIC_RG_LDO_VCAMA2_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcama2_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMA2_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMA2_EN_MASK,
		PMIC_RG_LDO_VCAMA2_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama2_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA2_LP_ADDR,
		val,
		PMIC_RG_LDO_VCAMA2_LP_MASK,
		PMIC_RG_LDO_VCAMA2_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama2_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA2_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMA2_SW_OP_EN_MASK,
		PMIC_RG_LDO_VCAMA2_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcama2_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMA2_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMA2_SW_OP_EN_MASK,
		PMIC_RG_LDO_VCAMA2_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama2_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA2_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMA2_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VCAMA2_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcama2_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMA2_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMA2_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VCAMA2_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama2_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA2_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMA2_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VCAMA2_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcama2_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMA2_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMA2_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VCAMA2_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama2_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA2_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMA2_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VCAMA2_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcama2_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMA2_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMA2_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VCAMA2_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama2_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA2_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VCAMA2_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VCAMA2_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcama2_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMA2_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VCAMA2_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VCAMA2_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama2_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA2_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VCAMA2_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VCAMA2_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcama2_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMA2_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VCAMA2_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VCAMA2_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama2_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA2_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VCAMA2_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VCAMA2_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcama2_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMA2_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VCAMA2_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VCAMA2_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama2_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA2_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VCAMA2_ON_OP_MASK,
		PMIC_RG_LDO_VCAMA2_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama2_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA2_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VCAMA2_LP_OP_MASK,
		PMIC_RG_LDO_VCAMA2_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vcama2_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCAMA2_MODE_ADDR,
		&val,
		PMIC_DA_QI_VCAMA2_MODE_MASK,
		PMIC_DA_QI_VCAMA2_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama2_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA2_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VCAMA2_STBTD_MASK,
		PMIC_RG_LDO_VCAMA2_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vcama2_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCAMA2_STB_ADDR,
		&val,
		PMIC_DA_QI_VCAMA2_STB_MASK,
		PMIC_DA_QI_VCAMA2_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vcama2_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCAMA2_EN_ADDR,
		&val,
		PMIC_DA_QI_VCAMA2_EN_MASK,
		PMIC_DA_QI_VCAMA2_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama2_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA2_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMA2_OCFB_EN_MASK,
		PMIC_RG_LDO_VCAMA2_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcama2_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMA2_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMA2_OCFB_EN_MASK,
		PMIC_RG_LDO_VCAMA2_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vcama2_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCAMA2_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VCAMA2_OCFB_EN_MASK,
		PMIC_DA_QI_VCAMA2_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama2_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA2_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VCAMA2_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VCAMA2_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vcama2_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VCAMA2_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VCAMA2_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VCAMA2_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_ldo_degtd_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_LDO_DEGTD_SEL_ADDR,
		val,
		PMIC_LDO_DEGTD_SEL_MASK,
		PMIC_LDO_DEGTD_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vrtc_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VRTC_EN_ADDR,
		val,
		PMIC_RG_VRTC_EN_MASK,
		PMIC_RG_VRTC_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vrtc_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VRTC_EN_ADDR,
		&val,
		PMIC_RG_VRTC_EN_MASK,
		PMIC_RG_VRTC_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vrtc_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VRTC_EN_ADDR,
		&val,
		PMIC_DA_QI_VRTC_EN_MASK,
		PMIC_DA_QI_VRTC_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_rsv1(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_RSV1_ADDR,
		val,
		PMIC_RG_LDO_RSV1_MASK,
		PMIC_RG_LDO_RSV1_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_rsv0(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_RSV0_ADDR,
		val,
		PMIC_RG_LDO_RSV0_MASK,
		PMIC_RG_LDO_RSV0_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_rsv2(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_RSV2_ADDR,
		val,
		PMIC_RG_LDO_RSV2_MASK,
		PMIC_RG_LDO_RSV2_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_proc_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_PROC_EN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_PROC_EN_MASK,
		PMIC_RG_LDO_VSRAM_PROC_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_proc_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_PROC_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_PROC_EN_MASK,
		PMIC_RG_LDO_VSRAM_PROC_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_proc_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_PROC_LP_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_PROC_LP_MASK,
		PMIC_RG_LDO_VSRAM_PROC_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_proc_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_PROC_VOSEL_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_PROC_VOSEL_MASK,
		PMIC_RG_LDO_VSRAM_PROC_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_proc_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_PROC_VOSEL_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_PROC_VOSEL_MASK,
		PMIC_RG_LDO_VSRAM_PROC_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_proc_vosel_sleep(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_PROC_VOSEL_SLEEP_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_PROC_VOSEL_SLEEP_MASK,
		PMIC_RG_LDO_VSRAM_PROC_VOSEL_SLEEP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_proc_vosel_sleep(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_PROC_VOSEL_SLEEP_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_PROC_VOSEL_SLEEP_MASK,
		PMIC_RG_LDO_VSRAM_PROC_VOSEL_SLEEP_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_proc_sfchg_frate(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_PROC_SFCHG_FRATE_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_PROC_SFCHG_FRATE_MASK,
		PMIC_RG_LDO_VSRAM_PROC_SFCHG_FRATE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_proc_sfchg_fen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_PROC_SFCHG_FEN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_PROC_SFCHG_FEN_MASK,
		PMIC_RG_LDO_VSRAM_PROC_SFCHG_FEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_proc_sfchg_rrate(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_PROC_SFCHG_RRATE_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_PROC_SFCHG_RRATE_MASK,
		PMIC_RG_LDO_VSRAM_PROC_SFCHG_RRATE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_proc_sfchg_ren(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_PROC_SFCHG_REN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_PROC_SFCHG_REN_MASK,
		PMIC_RG_LDO_VSRAM_PROC_SFCHG_REN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_proc_dvs_trans_td(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_PROC_DVS_TRANS_TD_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_PROC_DVS_TRANS_TD_MASK,
		PMIC_RG_LDO_VSRAM_PROC_DVS_TRANS_TD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_proc_dvs_trans_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_PROC_DVS_TRANS_CTRL_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_PROC_DVS_TRANS_CTRL_MASK,
		PMIC_RG_LDO_VSRAM_PROC_DVS_TRANS_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_proc_dvs_trans_once(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_PROC_DVS_TRANS_ONCE_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_PROC_DVS_TRANS_ONCE_MASK,
		PMIC_RG_LDO_VSRAM_PROC_DVS_TRANS_ONCE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_proc_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_PROC_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_PROC_SW_OP_EN_MASK,
		PMIC_RG_LDO_VSRAM_PROC_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_proc_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_PROC_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_PROC_SW_OP_EN_MASK,
		PMIC_RG_LDO_VSRAM_PROC_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_proc_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_PROC_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_PROC_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VSRAM_PROC_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_proc_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_PROC_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_PROC_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VSRAM_PROC_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_proc_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_PROC_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_PROC_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VSRAM_PROC_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_proc_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_PROC_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_PROC_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VSRAM_PROC_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_proc_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_PROC_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_PROC_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VSRAM_PROC_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_proc_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_PROC_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_PROC_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VSRAM_PROC_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_proc_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_PROC_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_PROC_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VSRAM_PROC_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_proc_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_PROC_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_PROC_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VSRAM_PROC_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_proc_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_PROC_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_PROC_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VSRAM_PROC_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_proc_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_PROC_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_PROC_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VSRAM_PROC_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_proc_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_PROC_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_PROC_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VSRAM_PROC_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_proc_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_PROC_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_PROC_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VSRAM_PROC_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_proc_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_PROC_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_PROC_ON_OP_MASK,
		PMIC_RG_LDO_VSRAM_PROC_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_proc_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_PROC_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_PROC_LP_OP_MASK,
		PMIC_RG_LDO_VSRAM_PROC_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vsram_proc_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSRAM_PROC_MODE_ADDR,
		&val,
		PMIC_DA_QI_VSRAM_PROC_MODE_MASK,
		PMIC_DA_QI_VSRAM_PROC_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_proc_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_PROC_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_PROC_STBTD_MASK,
		PMIC_RG_LDO_VSRAM_PROC_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_proc_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_PROC_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_PROC_OCFB_EN_MASK,
		PMIC_RG_LDO_VSRAM_PROC_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_proc_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_PROC_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_PROC_OCFB_EN_MASK,
		PMIC_RG_LDO_VSRAM_PROC_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vsram_proc_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSRAM_PROC_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VSRAM_PROC_OCFB_EN_MASK,
		PMIC_DA_QI_VSRAM_PROC_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_proc_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_PROC_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_PROC_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VSRAM_PROC_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vsram_proc_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSRAM_PROC_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VSRAM_PROC_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VSRAM_PROC_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vsram_proc_vosel_gray(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSRAM_PROC_VOSEL_GRAY_ADDR,
		&val,
		PMIC_DA_QI_VSRAM_PROC_VOSEL_GRAY_MASK,
		PMIC_DA_QI_VSRAM_PROC_VOSEL_GRAY_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vsram_proc_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSRAM_PROC_VOSEL_ADDR,
		&val,
		PMIC_DA_QI_VSRAM_PROC_VOSEL_MASK,
		PMIC_DA_QI_VSRAM_PROC_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vsram_proc_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSRAM_PROC_EN_ADDR,
		&val,
		PMIC_DA_QI_VSRAM_PROC_EN_MASK,
		PMIC_DA_QI_VSRAM_PROC_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vsram_proc_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSRAM_PROC_STB_ADDR,
		&val,
		PMIC_DA_QI_VSRAM_PROC_STB_MASK,
		PMIC_DA_QI_VSRAM_PROC_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_ni_vsram_proc_vsleep_sel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_NI_VSRAM_PROC_VSLEEP_SEL_ADDR,
		&val,
		PMIC_DA_NI_VSRAM_PROC_VSLEEP_SEL_MASK,
		PMIC_DA_NI_VSRAM_PROC_VSLEEP_SEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_ni_vsram_proc_r2r_pdn(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_NI_VSRAM_PROC_R2R_PDN_ADDR,
		&val,
		PMIC_DA_NI_VSRAM_PROC_R2R_PDN_MASK,
		PMIC_DA_NI_VSRAM_PROC_R2R_PDN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_ni_vsram_proc_track_ndis_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_NI_VSRAM_PROC_TRACK_NDIS_EN_ADDR,
		&val,
		PMIC_DA_NI_VSRAM_PROC_TRACK_NDIS_EN_MASK,
		PMIC_DA_NI_VSRAM_PROC_TRACK_NDIS_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_EN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_EN_MASK,
		PMIC_RG_LDO_VSRAM_CORE_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_core_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_CORE_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_CORE_EN_MASK,
		PMIC_RG_LDO_VSRAM_CORE_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_LP_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_LP_MASK,
		PMIC_RG_LDO_VSRAM_CORE_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_MASK,
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_core_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_MASK,
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_vosel_sleep(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_SLEEP_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_SLEEP_MASK,
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_SLEEP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_core_vosel_sleep(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_SLEEP_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_SLEEP_MASK,
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_SLEEP_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_sfchg_frate(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_SFCHG_FRATE_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_SFCHG_FRATE_MASK,
		PMIC_RG_LDO_VSRAM_CORE_SFCHG_FRATE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_sfchg_fen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_SFCHG_FEN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_SFCHG_FEN_MASK,
		PMIC_RG_LDO_VSRAM_CORE_SFCHG_FEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_sfchg_rrate(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_SFCHG_RRATE_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_SFCHG_RRATE_MASK,
		PMIC_RG_LDO_VSRAM_CORE_SFCHG_RRATE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_sfchg_ren(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_SFCHG_REN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_SFCHG_REN_MASK,
		PMIC_RG_LDO_VSRAM_CORE_SFCHG_REN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_dvs_trans_td(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_DVS_TRANS_TD_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_DVS_TRANS_TD_MASK,
		PMIC_RG_LDO_VSRAM_CORE_DVS_TRANS_TD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_dvs_trans_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_DVS_TRANS_CTRL_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_DVS_TRANS_CTRL_MASK,
		PMIC_RG_LDO_VSRAM_CORE_DVS_TRANS_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_dvs_trans_once(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_DVS_TRANS_ONCE_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_DVS_TRANS_ONCE_MASK,
		PMIC_RG_LDO_VSRAM_CORE_DVS_TRANS_ONCE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_SW_OP_EN_MASK,
		PMIC_RG_LDO_VSRAM_CORE_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_core_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_CORE_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_CORE_SW_OP_EN_MASK,
		PMIC_RG_LDO_VSRAM_CORE_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VSRAM_CORE_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_core_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_CORE_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_CORE_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VSRAM_CORE_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VSRAM_CORE_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_core_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_CORE_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_CORE_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VSRAM_CORE_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VSRAM_CORE_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_core_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_CORE_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_CORE_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VSRAM_CORE_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VSRAM_CORE_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_core_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_CORE_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_CORE_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VSRAM_CORE_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VSRAM_CORE_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_core_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_CORE_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_CORE_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VSRAM_CORE_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VSRAM_CORE_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_core_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_CORE_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_CORE_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VSRAM_CORE_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_ON_OP_MASK,
		PMIC_RG_LDO_VSRAM_CORE_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_LP_OP_MASK,
		PMIC_RG_LDO_VSRAM_CORE_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vsram_core_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSRAM_CORE_MODE_ADDR,
		&val,
		PMIC_DA_QI_VSRAM_CORE_MODE_MASK,
		PMIC_DA_QI_VSRAM_CORE_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_STBTD_MASK,
		PMIC_RG_LDO_VSRAM_CORE_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_OCFB_EN_MASK,
		PMIC_RG_LDO_VSRAM_CORE_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_core_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_CORE_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_CORE_OCFB_EN_MASK,
		PMIC_RG_LDO_VSRAM_CORE_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vsram_core_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSRAM_CORE_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VSRAM_CORE_OCFB_EN_MASK,
		PMIC_DA_QI_VSRAM_CORE_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VSRAM_CORE_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vsram_core_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSRAM_CORE_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VSRAM_CORE_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VSRAM_CORE_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vsram_core_vosel_gray(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSRAM_CORE_VOSEL_GRAY_ADDR,
		&val,
		PMIC_DA_QI_VSRAM_CORE_VOSEL_GRAY_MASK,
		PMIC_DA_QI_VSRAM_CORE_VOSEL_GRAY_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vsram_core_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSRAM_CORE_VOSEL_ADDR,
		&val,
		PMIC_DA_QI_VSRAM_CORE_VOSEL_MASK,
		PMIC_DA_QI_VSRAM_CORE_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vsram_core_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSRAM_CORE_EN_ADDR,
		&val,
		PMIC_DA_QI_VSRAM_CORE_EN_MASK,
		PMIC_DA_QI_VSRAM_CORE_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vsram_core_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSRAM_CORE_STB_ADDR,
		&val,
		PMIC_DA_QI_VSRAM_CORE_STB_MASK,
		PMIC_DA_QI_VSRAM_CORE_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_ni_vsram_core_vsleep_sel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_NI_VSRAM_CORE_VSLEEP_SEL_ADDR,
		&val,
		PMIC_DA_NI_VSRAM_CORE_VSLEEP_SEL_MASK,
		PMIC_DA_NI_VSRAM_CORE_VSLEEP_SEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_ni_vsram_core_r2r_pdn(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_NI_VSRAM_CORE_R2R_PDN_ADDR,
		&val,
		PMIC_DA_NI_VSRAM_CORE_R2R_PDN_MASK,
		PMIC_DA_NI_VSRAM_CORE_R2R_PDN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_ni_vsram_core_track_ndis_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_NI_VSRAM_CORE_TRACK_NDIS_EN_ADDR,
		&val,
		PMIC_DA_NI_VSRAM_CORE_TRACK_NDIS_EN_MASK,
		PMIC_DA_NI_VSRAM_CORE_TRACK_NDIS_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_gpu_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_GPU_EN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_GPU_EN_MASK,
		PMIC_RG_LDO_VSRAM_GPU_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_gpu_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_GPU_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_GPU_EN_MASK,
		PMIC_RG_LDO_VSRAM_GPU_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_gpu_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_GPU_LP_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_GPU_LP_MASK,
		PMIC_RG_LDO_VSRAM_GPU_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_gpu_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_GPU_VOSEL_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_GPU_VOSEL_MASK,
		PMIC_RG_LDO_VSRAM_GPU_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_gpu_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_GPU_VOSEL_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_GPU_VOSEL_MASK,
		PMIC_RG_LDO_VSRAM_GPU_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_gpu_vosel_sleep(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_GPU_VOSEL_SLEEP_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_GPU_VOSEL_SLEEP_MASK,
		PMIC_RG_LDO_VSRAM_GPU_VOSEL_SLEEP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_gpu_vosel_sleep(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_GPU_VOSEL_SLEEP_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_GPU_VOSEL_SLEEP_MASK,
		PMIC_RG_LDO_VSRAM_GPU_VOSEL_SLEEP_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_gpu_sfchg_frate(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_GPU_SFCHG_FRATE_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_GPU_SFCHG_FRATE_MASK,
		PMIC_RG_LDO_VSRAM_GPU_SFCHG_FRATE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_gpu_sfchg_fen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_GPU_SFCHG_FEN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_GPU_SFCHG_FEN_MASK,
		PMIC_RG_LDO_VSRAM_GPU_SFCHG_FEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_gpu_sfchg_rrate(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_GPU_SFCHG_RRATE_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_GPU_SFCHG_RRATE_MASK,
		PMIC_RG_LDO_VSRAM_GPU_SFCHG_RRATE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_gpu_sfchg_ren(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_GPU_SFCHG_REN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_GPU_SFCHG_REN_MASK,
		PMIC_RG_LDO_VSRAM_GPU_SFCHG_REN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_gpu_dvs_trans_td(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_GPU_DVS_TRANS_TD_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_GPU_DVS_TRANS_TD_MASK,
		PMIC_RG_LDO_VSRAM_GPU_DVS_TRANS_TD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_gpu_dvs_trans_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_GPU_DVS_TRANS_CTRL_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_GPU_DVS_TRANS_CTRL_MASK,
		PMIC_RG_LDO_VSRAM_GPU_DVS_TRANS_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_gpu_dvs_trans_once(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_GPU_DVS_TRANS_ONCE_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_GPU_DVS_TRANS_ONCE_MASK,
		PMIC_RG_LDO_VSRAM_GPU_DVS_TRANS_ONCE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_gpu_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_GPU_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_GPU_SW_OP_EN_MASK,
		PMIC_RG_LDO_VSRAM_GPU_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_gpu_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_GPU_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_GPU_SW_OP_EN_MASK,
		PMIC_RG_LDO_VSRAM_GPU_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_gpu_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_GPU_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_GPU_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VSRAM_GPU_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_gpu_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_GPU_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_GPU_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VSRAM_GPU_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_gpu_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_GPU_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_GPU_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VSRAM_GPU_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_gpu_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_GPU_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_GPU_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VSRAM_GPU_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_gpu_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_GPU_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_GPU_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VSRAM_GPU_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_gpu_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_GPU_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_GPU_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VSRAM_GPU_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_gpu_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_GPU_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_GPU_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VSRAM_GPU_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_gpu_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_GPU_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_GPU_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VSRAM_GPU_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_gpu_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_GPU_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_GPU_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VSRAM_GPU_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_gpu_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_GPU_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_GPU_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VSRAM_GPU_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_gpu_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_GPU_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_GPU_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VSRAM_GPU_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_gpu_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_GPU_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_GPU_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VSRAM_GPU_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_gpu_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_GPU_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_GPU_ON_OP_MASK,
		PMIC_RG_LDO_VSRAM_GPU_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_gpu_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_GPU_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_GPU_LP_OP_MASK,
		PMIC_RG_LDO_VSRAM_GPU_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vsram_gpu_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSRAM_GPU_MODE_ADDR,
		&val,
		PMIC_DA_QI_VSRAM_GPU_MODE_MASK,
		PMIC_DA_QI_VSRAM_GPU_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_gpu_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_GPU_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_GPU_STBTD_MASK,
		PMIC_RG_LDO_VSRAM_GPU_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_gpu_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_GPU_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_GPU_OCFB_EN_MASK,
		PMIC_RG_LDO_VSRAM_GPU_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_gpu_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_GPU_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_GPU_OCFB_EN_MASK,
		PMIC_RG_LDO_VSRAM_GPU_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vsram_gpu_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSRAM_GPU_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VSRAM_GPU_OCFB_EN_MASK,
		PMIC_DA_QI_VSRAM_GPU_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_gpu_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_GPU_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_GPU_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VSRAM_GPU_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vsram_gpu_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSRAM_GPU_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VSRAM_GPU_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VSRAM_GPU_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vsram_gpu_vosel_gray(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSRAM_GPU_VOSEL_GRAY_ADDR,
		&val,
		PMIC_DA_QI_VSRAM_GPU_VOSEL_GRAY_MASK,
		PMIC_DA_QI_VSRAM_GPU_VOSEL_GRAY_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vsram_gpu_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSRAM_GPU_VOSEL_ADDR,
		&val,
		PMIC_DA_QI_VSRAM_GPU_VOSEL_MASK,
		PMIC_DA_QI_VSRAM_GPU_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vsram_gpu_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSRAM_GPU_EN_ADDR,
		&val,
		PMIC_DA_QI_VSRAM_GPU_EN_MASK,
		PMIC_DA_QI_VSRAM_GPU_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vsram_gpu_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSRAM_GPU_STB_ADDR,
		&val,
		PMIC_DA_QI_VSRAM_GPU_STB_MASK,
		PMIC_DA_QI_VSRAM_GPU_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_ni_vsram_gpu_vsleep_sel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_NI_VSRAM_GPU_VSLEEP_SEL_ADDR,
		&val,
		PMIC_DA_NI_VSRAM_GPU_VSLEEP_SEL_MASK,
		PMIC_DA_NI_VSRAM_GPU_VSLEEP_SEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_ni_vsram_gpu_r2r_pdn(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_NI_VSRAM_GPU_R2R_PDN_ADDR,
		&val,
		PMIC_DA_NI_VSRAM_GPU_R2R_PDN_MASK,
		PMIC_DA_NI_VSRAM_GPU_R2R_PDN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_ni_vsram_gpu_track_ndis_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_NI_VSRAM_GPU_TRACK_NDIS_EN_ADDR,
		&val,
		PMIC_DA_NI_VSRAM_GPU_TRACK_NDIS_EN_MASK,
		PMIC_DA_NI_VSRAM_GPU_TRACK_NDIS_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_md_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_MD_EN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_MD_EN_MASK,
		PMIC_RG_LDO_VSRAM_MD_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_md_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_MD_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_MD_EN_MASK,
		PMIC_RG_LDO_VSRAM_MD_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_md_lp(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_MD_LP_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_MD_LP_MASK,
		PMIC_RG_LDO_VSRAM_MD_LP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_md_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_MD_VOSEL_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_MD_VOSEL_MASK,
		PMIC_RG_LDO_VSRAM_MD_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_md_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_MD_VOSEL_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_MD_VOSEL_MASK,
		PMIC_RG_LDO_VSRAM_MD_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_md_vosel_sleep(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_MD_VOSEL_SLEEP_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_MD_VOSEL_SLEEP_MASK,
		PMIC_RG_LDO_VSRAM_MD_VOSEL_SLEEP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_md_vosel_sleep(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_MD_VOSEL_SLEEP_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_MD_VOSEL_SLEEP_MASK,
		PMIC_RG_LDO_VSRAM_MD_VOSEL_SLEEP_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_md_sfchg_frate(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_MD_SFCHG_FRATE_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_MD_SFCHG_FRATE_MASK,
		PMIC_RG_LDO_VSRAM_MD_SFCHG_FRATE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_md_sfchg_fen(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_MD_SFCHG_FEN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_MD_SFCHG_FEN_MASK,
		PMIC_RG_LDO_VSRAM_MD_SFCHG_FEN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_md_sfchg_rrate(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_MD_SFCHG_RRATE_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_MD_SFCHG_RRATE_MASK,
		PMIC_RG_LDO_VSRAM_MD_SFCHG_RRATE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_md_sfchg_ren(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_MD_SFCHG_REN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_MD_SFCHG_REN_MASK,
		PMIC_RG_LDO_VSRAM_MD_SFCHG_REN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_md_dvs_trans_td(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_MD_DVS_TRANS_TD_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_MD_DVS_TRANS_TD_MASK,
		PMIC_RG_LDO_VSRAM_MD_DVS_TRANS_TD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_md_dvs_trans_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_MD_DVS_TRANS_CTRL_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_MD_DVS_TRANS_CTRL_MASK,
		PMIC_RG_LDO_VSRAM_MD_DVS_TRANS_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_md_dvs_trans_once(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_MD_DVS_TRANS_ONCE_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_MD_DVS_TRANS_ONCE_MASK,
		PMIC_RG_LDO_VSRAM_MD_DVS_TRANS_ONCE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_md_sw_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_MD_SW_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_MD_SW_OP_EN_MASK,
		PMIC_RG_LDO_VSRAM_MD_SW_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_md_sw_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_MD_SW_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_MD_SW_OP_EN_MASK,
		PMIC_RG_LDO_VSRAM_MD_SW_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_md_hw0_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_MD_HW0_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_MD_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VSRAM_MD_HW0_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_md_hw0_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_MD_HW0_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_MD_HW0_OP_EN_MASK,
		PMIC_RG_LDO_VSRAM_MD_HW0_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_md_hw1_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_MD_HW1_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_MD_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VSRAM_MD_HW1_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_md_hw1_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_MD_HW1_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_MD_HW1_OP_EN_MASK,
		PMIC_RG_LDO_VSRAM_MD_HW1_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_md_hw2_op_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_MD_HW2_OP_EN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_MD_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VSRAM_MD_HW2_OP_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_md_hw2_op_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_MD_HW2_OP_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_MD_HW2_OP_EN_MASK,
		PMIC_RG_LDO_VSRAM_MD_HW2_OP_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_md_hw0_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_MD_HW0_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_MD_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VSRAM_MD_HW0_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_md_hw0_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_MD_HW0_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_MD_HW0_OP_CFG_MASK,
		PMIC_RG_LDO_VSRAM_MD_HW0_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_md_hw1_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_MD_HW1_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_MD_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VSRAM_MD_HW1_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_md_hw1_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_MD_HW1_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_MD_HW1_OP_CFG_MASK,
		PMIC_RG_LDO_VSRAM_MD_HW1_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_md_hw2_op_cfg(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_MD_HW2_OP_CFG_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_MD_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VSRAM_MD_HW2_OP_CFG_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_md_hw2_op_cfg(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_MD_HW2_OP_CFG_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_MD_HW2_OP_CFG_MASK,
		PMIC_RG_LDO_VSRAM_MD_HW2_OP_CFG_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_md_on_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_MD_ON_OP_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_MD_ON_OP_MASK,
		PMIC_RG_LDO_VSRAM_MD_ON_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_md_lp_op(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_MD_LP_OP_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_MD_LP_OP_MASK,
		PMIC_RG_LDO_VSRAM_MD_LP_OP_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vsram_md_mode(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSRAM_MD_MODE_ADDR,
		&val,
		PMIC_DA_QI_VSRAM_MD_MODE_MASK,
		PMIC_DA_QI_VSRAM_MD_MODE_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_md_stbtd(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_MD_STBTD_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_MD_STBTD_MASK,
		PMIC_RG_LDO_VSRAM_MD_STBTD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_md_ocfb_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_MD_OCFB_EN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_MD_OCFB_EN_MASK,
		PMIC_RG_LDO_VSRAM_MD_OCFB_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_md_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_MD_OCFB_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_MD_OCFB_EN_MASK,
		PMIC_RG_LDO_VSRAM_MD_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vsram_md_ocfb_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSRAM_MD_OCFB_EN_ADDR,
		&val,
		PMIC_DA_QI_VSRAM_MD_OCFB_EN_MASK,
		PMIC_DA_QI_VSRAM_MD_OCFB_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_md_dummy_load(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_MD_DUMMY_LOAD_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_MD_DUMMY_LOAD_MASK,
		PMIC_RG_LDO_VSRAM_MD_DUMMY_LOAD_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_da_qi_vsram_md_dummy_load(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSRAM_MD_DUMMY_LOAD_ADDR,
		&val,
		PMIC_DA_QI_VSRAM_MD_DUMMY_LOAD_MASK,
		PMIC_DA_QI_VSRAM_MD_DUMMY_LOAD_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vsram_md_vosel_gray(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSRAM_MD_VOSEL_GRAY_ADDR,
		&val,
		PMIC_DA_QI_VSRAM_MD_VOSEL_GRAY_MASK,
		PMIC_DA_QI_VSRAM_MD_VOSEL_GRAY_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vsram_md_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSRAM_MD_VOSEL_ADDR,
		&val,
		PMIC_DA_QI_VSRAM_MD_VOSEL_MASK,
		PMIC_DA_QI_VSRAM_MD_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vsram_md_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSRAM_MD_EN_ADDR,
		&val,
		PMIC_DA_QI_VSRAM_MD_EN_MASK,
		PMIC_DA_QI_VSRAM_MD_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_qi_vsram_md_stb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_QI_VSRAM_MD_STB_ADDR,
		&val,
		PMIC_DA_QI_VSRAM_MD_STB_MASK,
		PMIC_DA_QI_VSRAM_MD_STB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_ni_vsram_md_vsleep_sel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_NI_VSRAM_MD_VSLEEP_SEL_ADDR,
		&val,
		PMIC_DA_NI_VSRAM_MD_VSLEEP_SEL_MASK,
		PMIC_DA_NI_VSRAM_MD_VSLEEP_SEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_ni_vsram_md_r2r_pdn(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_NI_VSRAM_MD_R2R_PDN_ADDR,
		&val,
		PMIC_DA_NI_VSRAM_MD_R2R_PDN_MASK,
		PMIC_DA_NI_VSRAM_MD_R2R_PDN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_get_da_ni_vsram_md_track_ndis_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_DA_NI_VSRAM_MD_TRACK_NDIS_EN_ADDR,
		&val,
		PMIC_DA_NI_VSRAM_MD_TRACK_NDIS_EN_MASK,
		PMIC_DA_NI_VSRAM_MD_TRACK_NDIS_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_track_sleep_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_TRACK_SLEEP_CTRL_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_TRACK_SLEEP_CTRL_MASK,
		PMIC_RG_LDO_VSRAM_CORE_TRACK_SLEEP_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_track_on_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_TRACK_ON_CTRL_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_TRACK_ON_CTRL_MASK,
		PMIC_RG_LDO_VSRAM_CORE_TRACK_ON_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_track_vbuck_on_ctrl(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_TRACK_VBUCK_ON_CTRL_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_TRACK_VBUCK_ON_CTRL_MASK,
		PMIC_RG_LDO_VSRAM_CORE_TRACK_VBUCK_ON_CTRL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_vosel_delta(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_DELTA_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_DELTA_MASK,
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_DELTA_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_core_vosel_delta(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_DELTA_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_DELTA_MASK,
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_DELTA_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_vosel_offset(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_OFFSET_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_OFFSET_MASK,
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_OFFSET_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_core_vosel_offset(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_OFFSET_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_OFFSET_MASK,
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_OFFSET_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_vosel_on_lb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_ON_LB_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_ON_LB_MASK,
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_ON_LB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_core_vosel_on_lb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_ON_LB_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_ON_LB_MASK,
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_ON_LB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_vosel_on_hb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_ON_HB_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_ON_HB_MASK,
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_ON_HB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_core_vosel_on_hb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_ON_HB_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_ON_HB_MASK,
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_ON_HB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_vosel_sleep_lb(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_SLEEP_LB_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_SLEEP_LB_MASK,
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_SLEEP_LB_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_core_vosel_sleep_lb(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_SLEEP_LB_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_SLEEP_LB_MASK,
		PMIC_RG_LDO_VSRAM_CORE_VOSEL_SLEEP_LB_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_dcm_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_DCM_MODE_ADDR,
		val,
		PMIC_RG_LDO_DCM_MODE_MASK,
		PMIC_RG_LDO_DCM_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vio28_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO28_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VIO28_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VIO28_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vio28_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO28_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VIO28_CK_SW_EN_MASK,
		PMIC_RG_LDO_VIO28_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vio28_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VIO28_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VIO28_CK_SW_EN_MASK,
		PMIC_RG_LDO_VIO28_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vio28_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO28_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VIO28_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VIO28_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vio18_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO18_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VIO18_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VIO18_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vio18_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO18_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VIO18_CK_SW_EN_MASK,
		PMIC_RG_LDO_VIO18_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vio18_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VIO18_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VIO18_CK_SW_EN_MASK,
		PMIC_RG_LDO_VIO18_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vio18_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VIO18_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VIO18_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VIO18_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vufs18_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUFS18_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VUFS18_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VUFS18_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vufs18_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUFS18_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VUFS18_CK_SW_EN_MASK,
		PMIC_RG_LDO_VUFS18_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vufs18_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VUFS18_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VUFS18_CK_SW_EN_MASK,
		PMIC_RG_LDO_VUFS18_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vufs18_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUFS18_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VUFS18_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VUFS18_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_va10_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA10_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VA10_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VA10_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_va10_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA10_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VA10_CK_SW_EN_MASK,
		PMIC_RG_LDO_VA10_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_va10_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VA10_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VA10_CK_SW_EN_MASK,
		PMIC_RG_LDO_VA10_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va10_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA10_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VA10_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VA10_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_va12_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA12_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VA12_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VA12_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_va12_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA12_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VA12_CK_SW_EN_MASK,
		PMIC_RG_LDO_VA12_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_va12_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VA12_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VA12_CK_SW_EN_MASK,
		PMIC_RG_LDO_VA12_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va12_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA12_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VA12_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VA12_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_proc_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_PROC_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_PROC_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VSRAM_PROC_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_proc_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_PROC_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_PROC_CK_SW_EN_MASK,
		PMIC_RG_LDO_VSRAM_PROC_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_proc_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_PROC_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_PROC_CK_SW_EN_MASK,
		PMIC_RG_LDO_VSRAM_PROC_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_proc_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_PROC_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_PROC_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VSRAM_PROC_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VSRAM_CORE_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_CK_SW_EN_MASK,
		PMIC_RG_LDO_VSRAM_CORE_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_core_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_CORE_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_CORE_CK_SW_EN_MASK,
		PMIC_RG_LDO_VSRAM_CORE_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VSRAM_CORE_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_gpu_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_GPU_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_GPU_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VSRAM_GPU_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_gpu_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_GPU_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_GPU_CK_SW_EN_MASK,
		PMIC_RG_LDO_VSRAM_GPU_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_gpu_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_GPU_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_GPU_CK_SW_EN_MASK,
		PMIC_RG_LDO_VSRAM_GPU_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_gpu_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_GPU_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_GPU_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VSRAM_GPU_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_md_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_MD_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_MD_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VSRAM_MD_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_md_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_MD_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_MD_CK_SW_EN_MASK,
		PMIC_RG_LDO_VSRAM_MD_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_md_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_MD_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_MD_CK_SW_EN_MASK,
		PMIC_RG_LDO_VSRAM_MD_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_md_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_MD_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_MD_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VSRAM_MD_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_va18_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA18_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VA18_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VA18_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_va18_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA18_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VA18_CK_SW_EN_MASK,
		PMIC_RG_LDO_VA18_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_va18_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VA18_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VA18_CK_SW_EN_MASK,
		PMIC_RG_LDO_VA18_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_va18_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VA18_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VA18_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VA18_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vusb33_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUSB33_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VUSB33_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VUSB33_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vusb33_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUSB33_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VUSB33_CK_SW_EN_MASK,
		PMIC_RG_LDO_VUSB33_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vusb33_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VUSB33_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VUSB33_CK_SW_EN_MASK,
		PMIC_RG_LDO_VUSB33_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vusb33_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VUSB33_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VUSB33_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VUSB33_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vemc_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VEMC_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VEMC_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VEMC_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vemc_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VEMC_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VEMC_CK_SW_EN_MASK,
		PMIC_RG_LDO_VEMC_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vemc_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VEMC_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VEMC_CK_SW_EN_MASK,
		PMIC_RG_LDO_VEMC_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vemc_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VEMC_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VEMC_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VEMC_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo22_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO22_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VXO22_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VXO22_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo22_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO22_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VXO22_CK_SW_EN_MASK,
		PMIC_RG_LDO_VXO22_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vxo22_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VXO22_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VXO22_CK_SW_EN_MASK,
		PMIC_RG_LDO_VXO22_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo22_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO22_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VXO22_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VXO22_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo18_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO18_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VXO18_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VXO18_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo18_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO18_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VXO18_CK_SW_EN_MASK,
		PMIC_RG_LDO_VXO18_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vxo18_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VXO18_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VXO18_CK_SW_EN_MASK,
		PMIC_RG_LDO_VXO18_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vxo18_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VXO18_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VXO18_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VXO18_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim1_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM1_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VSIM1_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VSIM1_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim1_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM1_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VSIM1_CK_SW_EN_MASK,
		PMIC_RG_LDO_VSIM1_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsim1_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSIM1_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSIM1_CK_SW_EN_MASK,
		PMIC_RG_LDO_VSIM1_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim1_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM1_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VSIM1_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VSIM1_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim2_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM2_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VSIM2_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VSIM2_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim2_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM2_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VSIM2_CK_SW_EN_MASK,
		PMIC_RG_LDO_VSIM2_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsim2_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSIM2_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSIM2_CK_SW_EN_MASK,
		PMIC_RG_LDO_VSIM2_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsim2_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSIM2_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VSIM2_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VSIM2_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd1_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD1_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VCAMD1_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VCAMD1_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd1_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD1_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMD1_CK_SW_EN_MASK,
		PMIC_RG_LDO_VCAMD1_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcamd1_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMD1_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMD1_CK_SW_EN_MASK,
		PMIC_RG_LDO_VCAMD1_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd1_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD1_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VCAMD1_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VCAMD1_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd2_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD2_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VCAMD2_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VCAMD2_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd2_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD2_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMD2_CK_SW_EN_MASK,
		PMIC_RG_LDO_VCAMD2_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcamd2_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMD2_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMD2_CK_SW_EN_MASK,
		PMIC_RG_LDO_VCAMD2_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamd2_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMD2_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VCAMD2_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VCAMD2_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamio_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMIO_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VCAMIO_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VCAMIO_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamio_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMIO_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMIO_CK_SW_EN_MASK,
		PMIC_RG_LDO_VCAMIO_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcamio_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMIO_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMIO_CK_SW_EN_MASK,
		PMIC_RG_LDO_VCAMIO_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcamio_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMIO_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VCAMIO_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VCAMIO_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vmipi_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMIPI_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VMIPI_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VMIPI_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vmipi_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMIPI_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VMIPI_CK_SW_EN_MASK,
		PMIC_RG_LDO_VMIPI_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vmipi_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VMIPI_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VMIPI_CK_SW_EN_MASK,
		PMIC_RG_LDO_VMIPI_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmipi_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMIPI_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VMIPI_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VMIPI_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VGP_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VGP_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VGP_CK_SW_EN_MASK,
		PMIC_RG_LDO_VGP_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vgp_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VGP_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VGP_CK_SW_EN_MASK,
		PMIC_RG_LDO_VGP_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VGP_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VGP_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn33_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN33_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VCN33_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VCN33_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn33_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN33_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VCN33_CK_SW_EN_MASK,
		PMIC_RG_LDO_VCN33_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn33_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN33_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCN33_CK_SW_EN_MASK,
		PMIC_RG_LDO_VCN33_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn33_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN33_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VCN33_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VCN33_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn18_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN18_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VCN18_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VCN18_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn18_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN18_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VCN18_CK_SW_EN_MASK,
		PMIC_RG_LDO_VCN18_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn18_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN18_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCN18_CK_SW_EN_MASK,
		PMIC_RG_LDO_VCN18_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn18_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN18_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VCN18_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VCN18_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn28_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN28_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VCN28_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VCN28_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn28_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN28_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VCN28_CK_SW_EN_MASK,
		PMIC_RG_LDO_VCN28_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcn28_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCN28_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCN28_CK_SW_EN_MASK,
		PMIC_RG_LDO_VCN28_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcn28_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCN28_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VCN28_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VCN28_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp2_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP2_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VGP2_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VGP2_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp2_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP2_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VGP2_CK_SW_EN_MASK,
		PMIC_RG_LDO_VGP2_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vgp2_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VGP2_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VGP2_CK_SW_EN_MASK,
		PMIC_RG_LDO_VGP2_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vgp2_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VGP2_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VGP2_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VGP2_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vbif28_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VBIF28_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VBIF28_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VBIF28_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vbif28_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VBIF28_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VBIF28_CK_SW_EN_MASK,
		PMIC_RG_LDO_VBIF28_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vbif28_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VBIF28_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VBIF28_CK_SW_EN_MASK,
		PMIC_RG_LDO_VBIF28_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vbif28_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VBIF28_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VBIF28_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VBIF28_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vfe28_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VFE28_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VFE28_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VFE28_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vfe28_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VFE28_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VFE28_CK_SW_EN_MASK,
		PMIC_RG_LDO_VFE28_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vfe28_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VFE28_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VFE28_CK_SW_EN_MASK,
		PMIC_RG_LDO_VFE28_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vfe28_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VFE28_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VFE28_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VFE28_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vmch_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMCH_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VMCH_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VMCH_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vmch_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMCH_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VMCH_CK_SW_EN_MASK,
		PMIC_RG_LDO_VMCH_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vmch_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VMCH_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VMCH_CK_SW_EN_MASK,
		PMIC_RG_LDO_VMCH_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmch_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMCH_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VMCH_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VMCH_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vmc_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMC_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VMC_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VMC_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vmc_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMC_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VMC_CK_SW_EN_MASK,
		PMIC_RG_LDO_VMC_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vmc_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VMC_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VMC_CK_SW_EN_MASK,
		PMIC_RG_LDO_VMC_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vmc_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VMC_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VMC_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VMC_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_1_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_1_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VRF18_1_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VRF18_1_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_1_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_1_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VRF18_1_CK_SW_EN_MASK,
		PMIC_RG_LDO_VRF18_1_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vrf18_1_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VRF18_1_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VRF18_1_CK_SW_EN_MASK,
		PMIC_RG_LDO_VRF18_1_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_1_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_1_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VRF18_1_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VRF18_1_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_2_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_2_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VRF18_2_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VRF18_2_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_2_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_2_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VRF18_2_CK_SW_EN_MASK,
		PMIC_RG_LDO_VRF18_2_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vrf18_2_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VRF18_2_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VRF18_2_CK_SW_EN_MASK,
		PMIC_RG_LDO_VRF18_2_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf18_2_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF18_2_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VRF18_2_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VRF18_2_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vtcxo24_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VTCXO24_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VTCXO24_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VTCXO24_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vtcxo24_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VTCXO24_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VTCXO24_CK_SW_EN_MASK,
		PMIC_RG_LDO_VTCXO24_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vtcxo24_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VTCXO24_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VTCXO24_CK_SW_EN_MASK,
		PMIC_RG_LDO_VTCXO24_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vtcxo24_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VTCXO24_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VTCXO24_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VTCXO24_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vldo28_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VLDO28_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VLDO28_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VLDO28_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vldo28_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VLDO28_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VLDO28_CK_SW_EN_MASK,
		PMIC_RG_LDO_VLDO28_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vldo28_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VLDO28_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VLDO28_CK_SW_EN_MASK,
		PMIC_RG_LDO_VLDO28_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vldo28_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VLDO28_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VLDO28_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VLDO28_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf12_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF12_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VRF12_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VRF12_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf12_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF12_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VRF12_CK_SW_EN_MASK,
		PMIC_RG_LDO_VRF12_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vrf12_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VRF12_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VRF12_CK_SW_EN_MASK,
		PMIC_RG_LDO_VRF12_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vrf12_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VRF12_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VRF12_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VRF12_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama1_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA1_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VCAMA1_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VCAMA1_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama1_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA1_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMA1_CK_SW_EN_MASK,
		PMIC_RG_LDO_VCAMA1_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcama1_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMA1_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMA1_CK_SW_EN_MASK,
		PMIC_RG_LDO_VCAMA1_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama1_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA1_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VCAMA1_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VCAMA1_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama2_ck_sw_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA2_CK_SW_MODE_ADDR,
		val,
		PMIC_RG_LDO_VCAMA2_CK_SW_MODE_MASK,
		PMIC_RG_LDO_VCAMA2_CK_SW_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama2_ck_sw_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA2_CK_SW_EN_ADDR,
		val,
		PMIC_RG_LDO_VCAMA2_CK_SW_EN_MASK,
		PMIC_RG_LDO_VCAMA2_CK_SW_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vcama2_ck_sw_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VCAMA2_CK_SW_EN_ADDR,
		&val,
		PMIC_RG_LDO_VCAMA2_CK_SW_EN_MASK,
		PMIC_RG_LDO_VCAMA2_CK_SW_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vcama2_osc_sel_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VCAMA2_OSC_SEL_DIS_ADDR,
		val,
		PMIC_RG_LDO_VCAMA2_OSC_SEL_DIS_MASK,
		PMIC_RG_LDO_VCAMA2_OSC_SEL_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_proc_sp_sw_vosel_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_PROC_SP_SW_VOSEL_EN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_PROC_SP_SW_VOSEL_EN_MASK,
		PMIC_RG_LDO_VSRAM_PROC_SP_SW_VOSEL_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_proc_sp_sw_vosel_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_PROC_SP_SW_VOSEL_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_PROC_SP_SW_VOSEL_EN_MASK,
		PMIC_RG_LDO_VSRAM_PROC_SP_SW_VOSEL_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_proc_sp_sw_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_PROC_SP_SW_VOSEL_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_PROC_SP_SW_VOSEL_MASK,
		PMIC_RG_LDO_VSRAM_PROC_SP_SW_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_proc_sp_sw_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_PROC_SP_SW_VOSEL_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_PROC_SP_SW_VOSEL_MASK,
		PMIC_RG_LDO_VSRAM_PROC_SP_SW_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_gpu_sp_sw_vosel_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_GPU_SP_SW_VOSEL_EN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_GPU_SP_SW_VOSEL_EN_MASK,
		PMIC_RG_LDO_VSRAM_GPU_SP_SW_VOSEL_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_gpu_sp_sw_vosel_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_GPU_SP_SW_VOSEL_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_GPU_SP_SW_VOSEL_EN_MASK,
		PMIC_RG_LDO_VSRAM_GPU_SP_SW_VOSEL_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_gpu_sp_sw_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_GPU_SP_SW_VOSEL_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_GPU_SP_SW_VOSEL_MASK,
		PMIC_RG_LDO_VSRAM_GPU_SP_SW_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_gpu_sp_sw_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_GPU_SP_SW_VOSEL_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_GPU_SP_SW_VOSEL_MASK,
		PMIC_RG_LDO_VSRAM_GPU_SP_SW_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_md_sp_sw_vosel_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_MD_SP_SW_VOSEL_EN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_MD_SP_SW_VOSEL_EN_MASK,
		PMIC_RG_LDO_VSRAM_MD_SP_SW_VOSEL_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_md_sp_sw_vosel_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_MD_SP_SW_VOSEL_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_MD_SP_SW_VOSEL_EN_MASK,
		PMIC_RG_LDO_VSRAM_MD_SP_SW_VOSEL_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_md_sp_sw_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_MD_SP_SW_VOSEL_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_MD_SP_SW_VOSEL_MASK,
		PMIC_RG_LDO_VSRAM_MD_SP_SW_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_md_sp_sw_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_MD_SP_SW_VOSEL_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_MD_SP_SW_VOSEL_MASK,
		PMIC_RG_LDO_VSRAM_MD_SP_SW_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_sp_sw_vosel_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_SP_SW_VOSEL_EN_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_SP_SW_VOSEL_EN_MASK,
		PMIC_RG_LDO_VSRAM_CORE_SP_SW_VOSEL_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_core_sp_sw_vosel_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_CORE_SP_SW_VOSEL_EN_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_CORE_SP_SW_VOSEL_EN_MASK,
		PMIC_RG_LDO_VSRAM_CORE_SP_SW_VOSEL_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_sp_sw_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_SP_SW_VOSEL_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_SP_SW_VOSEL_MASK,
		PMIC_RG_LDO_VSRAM_CORE_SP_SW_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_core_sp_sw_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_CORE_SP_SW_VOSEL_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_CORE_SP_SW_VOSEL_MASK,
		PMIC_RG_LDO_VSRAM_CORE_SP_SW_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_sshub_on(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_SSHUB_ON_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_SSHUB_ON_MASK,
		PMIC_RG_LDO_VSRAM_CORE_SSHUB_ON_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_sshub_mode(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_SSHUB_MODE_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_SSHUB_MODE_MASK,
		PMIC_RG_LDO_VSRAM_CORE_SSHUB_MODE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_sshub_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_SSHUB_VOSEL_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_SSHUB_VOSEL_MASK,
		PMIC_RG_LDO_VSRAM_CORE_SSHUB_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_ldo_vsram_core_sshub_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_LDO_VSRAM_CORE_SSHUB_VOSEL_ADDR,
		&val,
		PMIC_RG_LDO_VSRAM_CORE_SSHUB_VOSEL_MASK,
		PMIC_RG_LDO_VSRAM_CORE_SSHUB_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_ldo_lp_prot_disable(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_LP_PROT_DISABLE_ADDR,
		val,
		PMIC_RG_LDO_LP_PROT_DISABLE_MASK,
		PMIC_RG_LDO_LP_PROT_DISABLE_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_dummy_load_gated_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_DUMMY_LOAD_GATED_DIS_ADDR,
		val,
		PMIC_RG_LDO_DUMMY_LOAD_GATED_DIS_MASK,
		PMIC_RG_LDO_DUMMY_LOAD_GATED_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_proc_r2r_pdn_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_PROC_R2R_PDN_DIS_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_PROC_R2R_PDN_DIS_MASK,
		PMIC_RG_LDO_VSRAM_PROC_R2R_PDN_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_core_r2r_pdn_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_CORE_R2R_PDN_DIS_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_CORE_R2R_PDN_DIS_MASK,
		PMIC_RG_LDO_VSRAM_CORE_R2R_PDN_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_gpu_r2r_pdn_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_GPU_R2R_PDN_DIS_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_GPU_R2R_PDN_DIS_MASK,
		PMIC_RG_LDO_VSRAM_GPU_R2R_PDN_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_ldo_vsram_md_r2r_pdn_dis(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_LDO_VSRAM_MD_R2R_PDN_DIS_ADDR,
		val,
		PMIC_RG_LDO_VSRAM_MD_R2R_PDN_DIS_MASK,
		PMIC_RG_LDO_VSRAM_MD_R2R_PDN_DIS_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vfe28_vocal(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VFE28_VOCAL_ADDR,
		val,
		PMIC_RG_VFE28_VOCAL_MASK,
		PMIC_RG_VFE28_VOCAL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vfe28_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VFE28_VOSEL_ADDR,
		val,
		PMIC_RG_VFE28_VOSEL_MASK,
		PMIC_RG_VFE28_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vfe28_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VFE28_VOSEL_ADDR,
		&val,
		PMIC_RG_VFE28_VOSEL_MASK,
		PMIC_RG_VFE28_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vfe28_votrim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VFE28_VOTRIM_ADDR,
		val,
		PMIC_RG_VFE28_VOTRIM_MASK,
		PMIC_RG_VFE28_VOTRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rgs_vfe28_cal_indi(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VFE28_CAL_INDI_ADDR,
		&val,
		PMIC_RGS_VFE28_CAL_INDI_MASK,
		PMIC_RGS_VFE28_CAL_INDI_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vfe28_oc_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VFE28_OC_TRIM_ADDR,
		val,
		PMIC_RG_VFE28_OC_TRIM_MASK,
		PMIC_RG_VFE28_OC_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vfe28_ndis_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VFE28_NDIS_EN_ADDR,
		val,
		PMIC_RG_VFE28_NDIS_EN_MASK,
		PMIC_RG_VFE28_NDIS_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vfe28_ndis_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VFE28_NDIS_EN_ADDR,
		&val,
		PMIC_RG_VFE28_NDIS_EN_MASK,
		PMIC_RG_VFE28_NDIS_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vfe28_stb_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VFE28_STB_SEL_ADDR,
		val,
		PMIC_RG_VFE28_STB_SEL_MASK,
		PMIC_RG_VFE28_STB_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vfe28_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VFE28_RSV_ADDR,
		val,
		PMIC_RG_VFE28_RSV_MASK,
		PMIC_RG_VFE28_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vfe28_vos_cal_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VFE28_VOS_CAL_EN_ADDR,
		val,
		PMIC_RG_VFE28_VOS_CAL_EN_MASK,
		PMIC_RG_VFE28_VOS_CAL_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vfe28_vos_cal_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VFE28_VOS_CAL_EN_ADDR,
		&val,
		PMIC_RG_VFE28_VOS_CAL_EN_MASK,
		PMIC_RG_VFE28_VOS_CAL_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vtcxo24_vocal(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VTCXO24_VOCAL_ADDR,
		val,
		PMIC_RG_VTCXO24_VOCAL_MASK,
		PMIC_RG_VTCXO24_VOCAL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vtcxo24_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VTCXO24_VOSEL_ADDR,
		val,
		PMIC_RG_VTCXO24_VOSEL_MASK,
		PMIC_RG_VTCXO24_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vtcxo24_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VTCXO24_VOSEL_ADDR,
		&val,
		PMIC_RG_VTCXO24_VOSEL_MASK,
		PMIC_RG_VTCXO24_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vtcxo24_votrim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VTCXO24_VOTRIM_ADDR,
		val,
		PMIC_RG_VTCXO24_VOTRIM_MASK,
		PMIC_RG_VTCXO24_VOTRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rgs_vtcxo24_cal_indi(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VTCXO24_CAL_INDI_ADDR,
		&val,
		PMIC_RGS_VTCXO24_CAL_INDI_MASK,
		PMIC_RGS_VTCXO24_CAL_INDI_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vtcxo24_oc_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VTCXO24_OC_TRIM_ADDR,
		val,
		PMIC_RG_VTCXO24_OC_TRIM_MASK,
		PMIC_RG_VTCXO24_OC_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vtcxo24_ndis_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VTCXO24_NDIS_EN_ADDR,
		val,
		PMIC_RG_VTCXO24_NDIS_EN_MASK,
		PMIC_RG_VTCXO24_NDIS_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vtcxo24_ndis_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VTCXO24_NDIS_EN_ADDR,
		&val,
		PMIC_RG_VTCXO24_NDIS_EN_MASK,
		PMIC_RG_VTCXO24_NDIS_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vtcxo24_stb_sel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VTCXO24_STB_SEL_ADDR,
		val,
		PMIC_RG_VTCXO24_STB_SEL_MASK,
		PMIC_RG_VTCXO24_STB_SEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vtcxo24_rsv(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VTCXO24_RSV_ADDR,
		val,
		PMIC_RG_VTCXO24_RSV_MASK,
		PMIC_RG_VTCXO24_RSV_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vtcxo24_vos_cal_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VTCXO24_VOS_CAL_EN_ADDR,
		val,
		PMIC_RG_VTCXO24_VOS_CAL_EN_MASK,
		PMIC_RG_VTCXO24_VOS_CAL_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vtcxo24_vos_cal_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VTCXO24_VOS_CAL_EN_ADDR,
		&val,
		PMIC_RG_VTCXO24_VOS_CAL_EN_MASK,
		PMIC_RG_VTCXO24_VOS_CAL_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vxo22_vocal(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VXO22_VOCAL_ADDR,
		val,
		PMIC_RG_VXO22_VOCAL_MASK,
		PMIC_RG_VXO22_VOCAL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vxo22_vosel(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VXO22_VOSEL_ADDR,
		val,
		PMIC_RG_VXO22_VOSEL_MASK,
		PMIC_RG_VXO22_VOSEL_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vxo22_vosel(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VXO22_VOSEL_ADDR,
		&val,
		PMIC_RG_VXO22_VOSEL_MASK,
		PMIC_RG_VXO22_VOSEL_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vxo22_votrim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VXO22_VOTRIM_ADDR,
		val,
		PMIC_RG_VXO22_VOTRIM_MASK,
		PMIC_RG_VXO22_VOTRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rgs_vxo22_cal_indi(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RGS_VXO22_CAL_INDI_ADDR,
		&val,
		PMIC_RGS_VXO22_CAL_INDI_MASK,
		PMIC_RGS_VXO22_CAL_INDI_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vxo22_oc_trim(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VXO22_OC_TRIM_ADDR,
		val,
		PMIC_RG_VXO22_OC_TRIM_MASK,
		PMIC_RG_VXO22_OC_TRIM_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_set_rg_vxo22_ndis_en(
	unsigned int val)
{
	unsigned int ret = 0;

	ret = pmic_config_interface(
		PMIC_RG_VXO22_NDIS_EN_ADDR,
		val,
		PMIC_RG_VXO22_NDIS_EN_MASK,
		PMIC_RG_VXO22_NDIS_EN_SHIFT);

	return ret;
}

unsigned int mt6355_upmu_get_rg_vxo22_ndis_en(
	void)
{
	unsigned int ret = 0;
	unsigned int val = 0;

	ret = pmic_read_interface(
		PMIC_RG_VXO22_NDIS_EN_ADDR,
		&val,
		PMIC_RG_VXO22_NDIS_EN_MASK,
		PMIC_RG_VXO22_NDIS_EN_SHIFT);

	return val;
}

unsigned int mt6355_upmu_set_rg_vxo22_stb_sel(
	unsigned int val)
{
	unsigned int ret = 0;


}

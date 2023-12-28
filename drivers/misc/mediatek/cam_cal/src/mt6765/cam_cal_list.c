/*
 * Copyright (C) 2018 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */
#include <linux/kernel.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "kd_imgsensor.h"

#ifdef ODM_HQ_EDIT
/* Pengfei.Zhao@ODM.Camera.Drv  20200903 RIO OTP Bringup*/
#define hi846_MAX_EEPROM_SIZE_8K 0x2000
#endif

struct stCAM_CAL_LIST_STRUCT g_camCalList[] = {
	/*Below is commom sensor */
	{IMX230_SENSOR_ID, 0xA0, Common_read_region},
	{S5K2T7SP_SENSOR_ID, 0xA4, Common_read_region},
	{IMX338_SENSOR_ID, 0xA0, Common_read_region},
	{S5K4E6_SENSOR_ID, 0xA8, Common_read_region},
	{IMX386_SENSOR_ID, 0xA0, Common_read_region},
	{S5K3M3_SENSOR_ID, 0xA0, Common_read_region},
	{S5K2L7_SENSOR_ID, 0xA0, Common_read_region},
	{IMX398_SENSOR_ID, 0xA0, Common_read_region},
	{IMX318_SENSOR_ID, 0xA0, Common_read_region},
	{OV8858_SENSOR_ID, 0xA8, Common_read_region},
	{IMX386_MONO_SENSOR_ID, 0xA0, Common_read_region},
	/*B+B*/
	{S5K2P7_SENSOR_ID, 0xA0, Common_read_region},
	{OV8856_SENSOR_ID, 0xA0, Common_read_region},
	/*61*/
	{IMX499_SENSOR_ID, 0xA0, Common_read_region},
	{S5K3L8_SENSOR_ID, 0xA0, Common_read_region},
	{S5K5E8YX_SENSOR_ID, 0xA2, Common_read_region},
	/*99*/
	{IMX258_SENSOR_ID, 0xA0, Common_read_region},
	{IMX258_MONO_SENSOR_ID, 0xA0, Common_read_region},
	/*97*/
	{OV23850_SENSOR_ID, 0xA0, Common_read_region},
	{OV23850_SENSOR_ID, 0xA8, Common_read_region},
	{S5K3M2_SENSOR_ID, 0xA0, Common_read_region},
	/*55*/
	{S5K2P8_SENSOR_ID, 0xA2, Common_read_region},
	{S5K2P8_SENSOR_ID, 0xA0, Common_read_region},
	{OV8858_SENSOR_ID, 0xA2, Common_read_region},
	/* Others */
	{S5K2X8_SENSOR_ID, 0xA0, Common_read_region},
	{IMX377_SENSOR_ID, 0xA0, Common_read_region},
	{IMX214_SENSOR_ID, 0xA0, Common_read_region},
	{IMX214_MONO_SENSOR_ID, 0xA0, Common_read_region},
	{IMX486_SENSOR_ID, 0xA8, Common_read_region},
	{OV12A10_SENSOR_ID, 0xA8, Common_read_region},
	{OV13855_SENSOR_ID, 0xA0, Common_read_region},
	{S5K3L8_SENSOR_ID, 0xA0, Common_read_region},
	{S5K5E8YX_SENSOR_ID, 0x5a, Common_read_region},
	{S5K5E8YXREAR2_SENSOR_ID, 0x5a, Common_read_region},
	/* Pengfei.Zhao@ODM.Camera.Drv  20200903 RIO OTP Bringup*/
	{HI846_SENSOR_ID, 0xA0, Common_read_region,hi846_MAX_EEPROM_SIZE_8K},
	{GC8034_SENSOR_ID, 0xA0, Common_read_region},
	{GC5035_SENSOR_ID, 0xA0, Common_read_region},
	{OV13B10_SENSOR_ID, 0xA0, Common_read_region},
	{GC02M1_RIO_SENSOR_ID, 0xA4, Common_read_region},
	{OV02B10_MIPI_SENSOR_ID, 0xA4, Common_read_region},
	{GC02M1_SW_SENSOR_ID, 0xA4, Common_read_region},
    #ifdef ODM_HQ_EDIT
	{HI556_SENSOR_ID, 0x40, Hi556_read_region},
	{S5K3H7YX_SENSOR_ID, 0x20, s5k3h7yx_read_region},
	{S5K3H7YX_TXD_SENSOR_ID, 0x20, s5k3h7yx_txd_read_region},
	{GC5035_OFG_SENSOR_ID, 0xA8, gc5035_ofg_read_region},
	{S5K3H7YX_TS_SENSOR_ID, 0x20, s5k3h7yx_ts_read_region},
    #endif
	//Cong.Zhou@ODM_HQ.Camera.Driver. 20190219 add for shuimitao
	//rear camera hi1336 otp
	{HI1336_SENSOR_ID, 0xA0, Common_read_region},
	//fengbin@ODM_HQ.Camera.Driver. 20190309 add for honeypeatch
	//rear camera s5k3l6 otp
	{S5K3L6_SENSOR_ID, 0xA0, Common_read_region},
    #ifdef ODM_HQ_EDIT
	{S5K4H7_SENSOR_ID, 0xA0, Common_read_region},
    #endif
    #ifdef ODM_HQ_EDIT
	{HI556_TXD_SENSOR_ID, 0x50, Hi556_TXD_read_region},
    #endif
	/*  ADD before this line */
	{0, 0, 0}       /*end of list */
};

unsigned int cam_cal_get_sensor_list(
	struct stCAM_CAL_LIST_STRUCT **ppCamcalList)
{
	if (ppCamcalList == NULL)
		return 1;

	*ppCamcalList = &g_camCalList[0];
	return 0;
}



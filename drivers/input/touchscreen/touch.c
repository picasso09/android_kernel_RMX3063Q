/***************************************************
 * File:touch.c
 * VENDOR_EDIT
 * Copyright (c)  2008- 2030  Oppo Mobile communication Corp.ltd.
 * Description:
 *             tp dev
 * Version:1.0:
 * Date created:2016/09/02
 * Author: hao.wang@Bsp.Driver
 * TAG: BSP.TP.Init
*/

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/serio.h>
#include "oppo_touchscreen/Synaptics/S3508/synaptics_s3508.h"
#include "oppo_touchscreen/tp_devices.h" 
#include "oppo_touchscreen/touchpanel_common.h"
#include <soc/oppo/oppo_project.h>
#include "touch.h"

#define MAX_LIMIT_DATA_LENGTH         100
#define S3706_FW_NAME_18073 "tp/18073/18073_FW_S3706_SYNAPTICS.img"
#define S3706_BASELINE_TEST_LIMIT_NAME_18073 "tp/18073/18073_Limit_data.img"
#define S3706_FW_NAME_19301 "tp/19301/19301_FW_S3706_SYNAPTICS.img"
#define S3706_BASELINE_TEST_LIMIT_NAME_19301 "tp/19301/19301_Limit_data.img"
#define GT9886_FW_NAME_19551 "tp/19551/FW_GT9886_SAMSUNG.img"
#define GT9886_BASELINE_TEST_LIMIT_NAME_19551 "tp/19551/LIMIT_GT9886_SAMSUNG.img"
#define GT9886_FW_NAME_19357 "tp/19357/FW_GT9886_SAMSUNG.img"
#define GT9886_BASELINE_TEST_LIMIT_NAME_19357 "tp/19357/LIMIT_GT9886_SAMSUNG.img"

#define ili9882n_FW_NAME_20701 "tp/20701/FW_NF_ILI9882N_90HZ_BOE.bin"
#define ili9882n_BASELINE_TEST_LIMIT_NAME_20701 "tp/20701/LIMIT_NF_ILI9882N_90HZ_BOE.ini"
#define ili9882n_MN228_FW_NAME_20701 "tp/20701/FW_ILI9882N_NF_MN228_TXD.bin"
#define ili9882n_MN228_BASELINE_TEST_LIMIT_NAME_20701 "tp/20701/LIMIT_ILI9882N_NF_MN228_TXD.ini"
#define ili9882n_TRULY_FW_NAME_20701 "tp/20701/FW_ILI9882N_NF_90HZ_TRULY.bin"
#define ili9882n_TRULY_BASELINE_TEST_LIMIT_NAME_20701 "tp/20701/LIMIT_ILI9882N_NF_90HZ_TRULY.ini"
#define ili9882h_FW_NAME_20701 "tp/20701/FW_NF_ILI9882H_90HZ_BOE.bin"
#define ili9882h_BASELINE_TEST_LIMIT_NAME_20701 "tp/20701/LIMIT_NF_ILI9882H_90HZ_BOE.ini"
#define nt36525b_FW_NAME_20701 "tp/20701/FW_NF_NT36525B_90HZ_BOE.bin"
#define nt36525b_BASELINE_TEST_LIMIT_NAME_20701 "tp/20701/LIMIT_NF_NT36525B_90HZ_BOE.ini"
#define hx83102d_FW_NAME_20701 "tp/20701/FW_NF_HX83102D_90HZ_BOE.bin"
#define hx83102d_BASELINE_TEST_LIMIT_NAME_20701 "tp/20701/LIMIT_NF_HX83102D_90HZ_BOE.ini"
#define hx83102d_TRULY_FW_NAME_20701 "tp/20701/FW_NF_HX83102D_90HZ_TRULY.bin"
#define hx83102d_TRULY_BASELINE_TEST_LIMIT_NAME_20701 "tp/20701/LIMIT_NF_HX83102D_90HZ_TRULY.ini"
#define ft8006s_FW_NAME_20701 "tp/20701/FW_FT8006S_NF_90HZ_BOE.bin"
#define ft8006s_BASELINE_TEST_LIMIT_NAME_20701 "tp/20701/LIMIT_FT8006S_NF_90HZ_BOE.ini"
#define ft8006s_TRULY_FW_NAME_20701 "tp/20701/FW_FT8006S_NF_90HZ_TRULY.bin"
#define ft8006s_TRULY_BASELINE_TEST_LIMIT_NAME_20701 "tp/20701/LIMIT_FT8006S_NF_90HZ_TRULY.ini"
#define ft8722_FW_NAME_20701 "tp/20701/FW_FT8722_NF_90HZ_BOE.bin"
#define ft8722_BASELINE_TEST_LIMIT_NAME_20701 "tp/20701/LIMIT_FT8722_NF_90HZ_BOE.ini"

/*  */
#define hx83112a_FW_NAME_20701 "tp/20701/FW_NF_HX83112A_90HZ_BOE.bin"
#define hx83112a_BASELINE_TEST_LIMIT_NAME_20701 "tp/20701/LIMIT_NF_HX83112A_90HZ_BOE.ini"



#define TD4330_NF_CHIP_NAME "TD4330_NF"
#define ILI9882n_NF_CHIP_NAME "ILI9882N_NF"
#define ILI9882h_NF_CHIP_NAME "ILI9882H_NF"
#define NV36525B_NF_CHIP_NAME "NV36525B_NF"
#define HX83102D_NF_CHIP_NAME "HX83102D_NF"
#define FT8006S_NF_CHIP_NAME "FT8006S_NF"
#define HX83112A_NF_CHIP_NAME "HX83112A_NF"
#define FT8722_NF_CHIP_NAME "FT8722_NF"



/*if can not compile success, please update vendor/oppo_touchsreen*/
struct tp_dev_name tp_dev_names[] = {
     {TP_OFILM, "OFILM"},
     {TP_BIEL, "BIEL"},
     {TP_TRULY, "TRULY"},
     {TP_BOE, "BOE"},
     {TP_G2Y, "G2Y"},
     {TP_TPK, "TPK"},
     {TP_JDI, "JDI"},
     {TP_TIANMA, "TIANMA"},
     {TP_SAMSUNG, "SAMSUNG"},
     {TP_DSJM, "DSJM"},
     {TP_BOE_B8, "BOEB8"},
     {TP_INNOLUX, "INNOLUX"},
     {TP_HIMAX_DPT, "DPT"},
     {TP_AUO, "AUO"},
     {TP_DEPUTE, "DEPUTE"},
     {TP_BOE_90HZ_82n, "90HZ_TXD"},
     {TP_BOE_90HZ_06S, "90HZ_HLT"},
     {TP_BOE_90HZ_722, "90HZ_HLT"},
     {TP_TRULY_06S, "90HZ_TRULY"},
     {TP_PANDA_90HZ_82h, "90HZ_LS"},
     {TP_MN228_82n, "MN228_TXD"},
     {TP_TRULY_82n, "90HZ_TRULY"},
     {TP_PANDA_90HZ_102D, "90HZ_TXD"},
     {TP_TRULY_90HZ_102D, "90HZ_TRULY"},
     {TP_PANDA_90HZ_112A, "112a_90HZ_TXD"},
     {TP_NVT_90HZ_15b, "90HZ_HLT"},
     {TP_UNKNOWN, "UNKNOWN"},
};

#define GET_TP_DEV_NAME(tp_type) ((tp_dev_names[tp_type].type == (tp_type))?tp_dev_names[tp_type].name:"UNMATCH")

int g_tp_dev_vendor = TP_UNKNOWN;
char *g_tp_chip_name;
static bool is_tp_type_got_in_match = false;    /*indicate whether the tp type is got in the process of ic match*/

/*
* this function is used to judge whether the ic driver should be loaded
* For incell module, tp is defined by lcd module, so if we judge the tp ic
* by the boot command line of containing lcd string, we can also get tp type.
*/
bool __init tp_judge_ic_match(char * tp_ic_name)
{
    pr_err("[TP] tp_ic_name = %s \n", tp_ic_name);
    pr_err("[TP] boot_command_line = %s \n", boot_command_line);

    //switch(get_project()) {
    //case 18151:
        is_tp_type_got_in_match = true;
        if (strstr(tp_ic_name, "td4320_nf") && strstr(boot_command_line, "dsjm_jdi_td4330")) {
            g_tp_dev_vendor = TP_DSJM;
            #ifdef CONFIG_TOUCHPANEL_MULTI_NOFLASH
            g_tp_chip_name = kzalloc(sizeof(TD4330_NF_CHIP_NAME), GFP_KERNEL);
            g_tp_chip_name = TD4330_NF_CHIP_NAME;
            #endif
            return true;
        }
        if (strstr(tp_ic_name, "td4320_nf") && strstr(boot_command_line, "dpt_jdi_td4330")) {
            g_tp_dev_vendor = TP_HIMAX_DPT;
            #ifdef CONFIG_TOUCHPANEL_MULTI_NOFLASH
            g_tp_chip_name = kzalloc(sizeof(TD4330_NF_CHIP_NAME), GFP_KERNEL);
            g_tp_chip_name = TD4330_NF_CHIP_NAME;
            #endif
            return true;
        }
		/*
		if (strstr(tp_ic_name, "himax,hx83112a_nf") && strstr(boot_command_line, "hx83112a_txd_boe_fives")) {
				g_tp_dev_vendor = TP_PANDA_90HZ_112A;
				g_tp_chip_name = kzalloc(sizeof(HX83112A_NF_CHIP_NAME), GFP_KERNEL);
				g_tp_chip_name = HX83112A_NF_CHIP_NAME;
				return true;
			}
		*/
		
		
    switch(get_project()) {
        case OPPO_19550:
        case OPPO_19551:
        case OPPO_19553:
        case OPPO_19556:
        case OPPO_19597:
                if (strstr(tp_ic_name, "synaptics-s3706") && strstr(boot_command_line, "oppo18073_samsung_ams641rw01_1080p_dsi_cmd")) {
                    g_tp_dev_vendor = TP_SAMSUNG;
                    return true;
                }
                if (strstr(tp_ic_name, "Goodix-gt9886") && strstr(boot_command_line, "oppo19551_samsung_ams644vk01_1080p_dsi_cmd")) {
                    g_tp_dev_vendor = TP_SAMSUNG;
                    return true;
                }
                pr_err("[TP] Driver does not match the project\n");
                break;

        case OPPO_19357:
        case OPPO_19358:
        case OPPO_19359:
        case OPPO_19354:
            if (strstr(tp_ic_name, "synaptics-s3706") && strstr(boot_command_line, "oppo18073_samsung_ams641rw01_1080p_dsi_cmd")) {
               g_tp_dev_vendor = TP_SAMSUNG;
                return true;
            }
            if (strstr(tp_ic_name, "Goodix-gt9886") && strstr(boot_command_line, "oppo19357_samsung_ams644va04_1080p_dsi_cmd")) {
               g_tp_dev_vendor = TP_SAMSUNG;
                return true;
            }
        pr_err("[TP] Driver does not match the project\n");
        break;
        case OPPO_18073:
        case OPPO_18593:
        case OPPO_19011:
        case OPPO_19301:
            {
               g_tp_dev_vendor = TP_SAMSUNG;
                return true;
            }
        break;
		
		/*jianghao tp buring up  start */
		case OPPO_20701:
		{
			if (strstr(tp_ic_name, "ilitek,ili9882n") && strstr(boot_command_line, "ili9882n_txd_inx")) {
				g_tp_dev_vendor = TP_BOE_90HZ_82n;
				g_tp_chip_name = kzalloc(sizeof(ILI9882n_NF_CHIP_NAME), GFP_KERNEL);
				g_tp_chip_name = ILI9882n_NF_CHIP_NAME;
				return true;
			}

			if (strstr(tp_ic_name, "ilitek,ili9882n") && strstr(boot_command_line, "ili9882n_mn228_txd_inx")) {
				g_tp_dev_vendor = TP_MN228_82n;
				g_tp_chip_name = kzalloc(sizeof(ILI9882n_NF_CHIP_NAME), GFP_KERNEL);
				g_tp_chip_name = ILI9882n_NF_CHIP_NAME;
				return true;
			}
			
			if (strstr(tp_ic_name, "ilitek,ili9882n") && strstr(boot_command_line, "ili9882n_truly_truly")) {
				g_tp_dev_vendor = TP_TRULY_82n;
				g_tp_chip_name = kzalloc(sizeof(ILI9882n_NF_CHIP_NAME), GFP_KERNEL);
				g_tp_chip_name = ILI9882n_NF_CHIP_NAME;
				return true;
			}

			if (strstr(tp_ic_name, "ilitek,ili9882n") && strstr(boot_command_line, "ili9882h_ls_panda")) {
				g_tp_dev_vendor = TP_PANDA_90HZ_82h;
				g_tp_chip_name = kzalloc(sizeof(ILI9882h_NF_CHIP_NAME), GFP_KERNEL);
				g_tp_chip_name = ILI9882h_NF_CHIP_NAME;
				return true;
			}

			if (strstr(tp_ic_name, "novatek,nf_nt36525b") && strstr(boot_command_line, "nt36525b_hlt_boe")) {
				g_tp_dev_vendor = TP_NVT_90HZ_15b;
				g_tp_chip_name = kzalloc(sizeof(NV36525B_NF_CHIP_NAME), GFP_KERNEL);
				g_tp_chip_name = NV36525B_NF_CHIP_NAME;
				return true;
			}

			if (strstr(tp_ic_name, "himax,hx83102d_nf") && strstr(boot_command_line, "hx83102d_txd_inx")) {
				g_tp_dev_vendor = TP_PANDA_90HZ_102D;
				g_tp_chip_name = kzalloc(sizeof(HX83102D_NF_CHIP_NAME), GFP_KERNEL);
				g_tp_chip_name = HX83102D_NF_CHIP_NAME;
				return true;
			}

			if (strstr(tp_ic_name, "himax,hx83102d_nf") && strstr(boot_command_line, "hx83102d_truly_truly")) {
				g_tp_dev_vendor = TP_TRULY_90HZ_102D;
				g_tp_chip_name = kzalloc(sizeof(HX83102D_NF_CHIP_NAME), GFP_KERNEL);
				g_tp_chip_name = HX83102D_NF_CHIP_NAME;
				return true;
			}

			if (strstr(tp_ic_name, "focaltech,fts") && strstr(boot_command_line, "ft8006s_hlt_boe")) {
				g_tp_dev_vendor = TP_BOE_90HZ_06S;
				g_tp_chip_name = kzalloc(sizeof(FT8006S_NF_CHIP_NAME), GFP_KERNEL);
				g_tp_chip_name = FT8006S_NF_CHIP_NAME;
				return true;
			}

			if (strstr(tp_ic_name, "focaltech,fts") && strstr(boot_command_line, "ft8006saa_truly_truly")) {
				g_tp_dev_vendor = TP_TRULY_06S;
				g_tp_chip_name = kzalloc(sizeof(FT8006S_NF_CHIP_NAME), GFP_KERNEL);
				g_tp_chip_name = FT8006S_NF_CHIP_NAME;
				return true;
			}

			if (strstr(tp_ic_name, "focaltech,ft8722") && strstr(boot_command_line, "ft8722_hlt_boe_eighth")) {
				g_tp_dev_vendor = TP_BOE_90HZ_722;
				g_tp_chip_name = kzalloc(sizeof(FT8722_NF_CHIP_NAME), GFP_KERNEL);
				g_tp_chip_name = FT8722_NF_CHIP_NAME;
				return true;
			}

			if (strstr(tp_ic_name, "himax,hx83112a_nf") && strstr(boot_command_line, "hx83112a_txd_boe_fives")) {
				g_tp_dev_vendor = TP_PANDA_90HZ_112A;
				g_tp_chip_name = kzalloc(sizeof(HX83112A_NF_CHIP_NAME), GFP_KERNEL);
				g_tp_chip_name = HX83112A_NF_CHIP_NAME;
				return true;
			}
			if (strstr(tp_ic_name, "himax,hx83112a_nf") && strstr(boot_command_line, "hx83112a_txd_boe_new_fives")) {
				g_tp_dev_vendor = TP_PANDA_90HZ_112A;
				g_tp_chip_name = kzalloc(sizeof(HX83112A_NF_CHIP_NAME), GFP_KERNEL);
				g_tp_chip_name = HX83112A_NF_CHIP_NAME;
				return true;
			}
		}
		/*jianghao tp buring up end*/

        default:
                pr_err("Invalid project\n");
        break;
     }

    pr_err("[TP]Lcd module not found\n");
    return false;
}

/*
* For separate lcd and tp, tp can be distingwished by gpio pins
* different project may have different combination, if needed,
* add your combination with project distingwished by is_project function.
*/
static void tp_get_vendor_via_pin(struct hw_resource *hw_res, struct panel_info *panel_data)
{
    panel_data->tp_type = TP_UNKNOWN;
    return;
}

/*
* If no gpio pins used to distingwish tp module, maybe have other ways(like command line)
* add your way of getting vendor info with project distingwished by is_project function.
*/
static void tp_get_vendor_separate(struct hw_resource *hw_res, struct panel_info *panel_data)
{
    panel_data->tp_type = TP_UNKNOWN;
    return;
}

int tp_util_get_vendor(struct hw_resource *hw_res, struct panel_info *panel_data)
{
    char* vendor;

//    #ifdef CONFIG_TOUCHPANEL_MULTI_NOFLASH
    if (g_tp_chip_name != NULL) {
        panel_data->chip_name = g_tp_chip_name;
    }
//    #endif
    if (is_project(OPPO_20701)||is_project(OPPO_18151)) {
        panel_data->test_limit_name = kzalloc(MAX_LIMIT_DATA_LENGTH, GFP_KERNEL);
        if (panel_data->test_limit_name == NULL) {
            pr_err("[TP]panel_data.test_limit_name kzalloc error\n");
        }

        panel_data->extra= kzalloc(MAX_LIMIT_DATA_LENGTH, GFP_KERNEL);
        if (panel_data->extra == NULL) {
            pr_err("[TP]panel_data.test_limit_name kzalloc error\n");
        }

    /*TP is first distingwished by gpio pins, and then by other ways*/
        if (is_tp_type_got_in_match) {
            panel_data->tp_type = g_tp_dev_vendor;
			
			printk("himax_tp_type: %d\n", panel_data->tp_type);

            if(is_project(OPPO_18151)) {
                if (strstr(boot_command_line, "dsjm_jdi_td4330")) {
                    memcpy(panel_data->manufacture_info.version, "0xDD075E", 8);
                }
                if (strstr(boot_command_line, "dpt_jdi_td4330")) {
                    memcpy(panel_data->manufacture_info.version, "0xDD075D", 8);
                }
            }
        } else if (gpio_is_valid(hw_res->id1_gpio) || gpio_is_valid(hw_res->id2_gpio) || gpio_is_valid(hw_res->id3_gpio)) {
            tp_get_vendor_via_pin(hw_res, panel_data);
        } else {
            tp_get_vendor_separate(hw_res, panel_data);
        }

        if (panel_data->tp_type == TP_UNKNOWN) {
            pr_err("[TP]%s type is unknown\n", __func__);
            return 0;
        }

        vendor = GET_TP_DEV_NAME(panel_data->tp_type);

        strcpy(panel_data->manufacture_info.manufacture, vendor);
        snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
                "tp/%d/FW_%s_%s.img",
                get_project(), panel_data->chip_name, vendor);

        if (panel_data->test_limit_name) {
            snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
                "tp/%d/LIMIT_%s_%s.img",
                get_project(), panel_data->chip_name, vendor);
        }

        if (panel_data->extra) {
            snprintf(panel_data->extra, MAX_LIMIT_DATA_LENGTH,
                "tp/%d/BOOT_FW_%s_%s.ihex",
                get_project(), panel_data->chip_name, vendor);
        }
        panel_data->manufacture_info.fw_path = panel_data->fw_name;

        switch(get_project()) {
        case OPPO_18151:
            if (strstr(boot_command_line, "dsjm_jdi_td4330")) {
                panel_data->firmware_headfile.firmware_data = FW_18151_TD4330_NF_DSJM;
                panel_data->firmware_headfile.firmware_size = sizeof(FW_18151_TD4330_NF_DSJM);
            } else if (strstr(boot_command_line, "dpt_jdi_td4330")) {
                panel_data->firmware_headfile.firmware_data = FW_18151_TD4330_NF_DPT;
                panel_data->firmware_headfile.firmware_size = sizeof(FW_18151_TD4330_NF_DPT);
            } else {
                panel_data->firmware_headfile.firmware_data = NULL;
                panel_data->firmware_headfile.firmware_size = 0;
           }
           break;
		 
		/*jianghao tp buring up  start */
        case OPPO_20701:
            if (strstr(boot_command_line, "ili9882n_txd_inx")) {
				panel_data->vid_len = 9;
			 	memcpy(panel_data->manufacture_info.version, "ILI9882N_", 9);
                panel_data->firmware_headfile.firmware_data = FW_20701_ILI9882N_TXD;
                panel_data->firmware_headfile.firmware_size = sizeof(FW_20701_ILI9882N_TXD);
			}else if (strstr(boot_command_line, "ili9882n_mn228_txd_inx")) {
				panel_data->vid_len = 9;
				memcpy(panel_data->manufacture_info.version, "ILIMN228_", 9);
			    panel_data->firmware_headfile.firmware_data = FW_20701_ILI9882N_MN228;
                panel_data->firmware_headfile.firmware_size = sizeof(FW_20701_ILI9882N_MN228);
			}else if (strstr(boot_command_line, "ili9882n_truly_truly")) {
				panel_data->vid_len = 9;
				memcpy(panel_data->manufacture_info.version, "ILITRULY_", 9);
			    panel_data->firmware_headfile.firmware_data = FW_20701_ILI9882N_TRULY;
                panel_data->firmware_headfile.firmware_size = sizeof(FW_20701_ILI9882N_TRULY);
			}else if (strstr(boot_command_line, "ili9882h_ls_panda")) {
				panel_data->vid_len = 9;
			 	memcpy(panel_data->manufacture_info.version, "ILI9882H_", 9);
			    panel_data->firmware_headfile.firmware_data = FW_20701_ILI9882H_LS;
                panel_data->firmware_headfile.firmware_size = sizeof(FW_20701_ILI9882H_LS);
			}else if (strstr(boot_command_line, "nt36525b_hlt_boe")) {
				panel_data->vid_len = 9;
			 	memcpy(panel_data->manufacture_info.version, "NV36525B_", 9);
			    panel_data->firmware_headfile.firmware_data = FW_20701_NT36525B_HLT;
                panel_data->firmware_headfile.firmware_size = sizeof(FW_20701_NT36525B_HLT);
			}else if (strstr(boot_command_line, "hx83102d_txd_inx")) {
				panel_data->vid_len = 9;
			 	memcpy(panel_data->manufacture_info.version, "HX83102D_", 9);
			    panel_data->firmware_headfile.firmware_data = FW_20701_HX83102D_TXD;
                panel_data->firmware_headfile.firmware_size = sizeof(FW_20701_HX83102D_TXD);
			}else if (strstr(boot_command_line, "hx83102d_truly_truly")) {
				panel_data->vid_len = 12;
			 	memcpy(panel_data->manufacture_info.version, "HX102DTRULY_", 12);
			    panel_data->firmware_headfile.firmware_data = FW_20701_HX83102D_TRULY;
                panel_data->firmware_headfile.firmware_size = sizeof(FW_20701_HX83102D_TRULY);
			}else if (strstr(boot_command_line, "ft8006s_hlt_boe")) {
				panel_data->vid_len = 9;
			 	memcpy(panel_data->manufacture_info.version, "FT8006S_", 9);
			    panel_data->firmware_headfile.firmware_data = FW_20701_FT8006S_HLT;
                panel_data->firmware_headfile.firmware_size = sizeof(FW_20701_FT8006S_HLT);
			}else if (strstr(boot_command_line, "ft8006saa_truly_truly")) {
				panel_data->vid_len = 10;
			 	memcpy(panel_data->manufacture_info.version, "FT06SaaT_", 10);
			    panel_data->firmware_headfile.firmware_data = FW_20701_FT8006S_TRULY;
                panel_data->firmware_headfile.firmware_size = sizeof(FW_20701_FT8006S_TRULY);
			}else if (strstr(boot_command_line, "ft8722_hlt_boe_eighth")) {
				panel_data->vid_len = 9;
			 	memcpy(panel_data->manufacture_info.version, "FT8722_", 9);
			    panel_data->firmware_headfile.firmware_data = FW_20701_FT8722_HLT;
                panel_data->firmware_headfile.firmware_size = sizeof(FW_20701_FT8722_HLT);
			}else if (strstr(boot_command_line, "hx83112a_txd_boe_fives")) {
				panel_data->vid_len = 9;
			 	memcpy(panel_data->manufacture_info.version, "HX83112A_", 9);
			    panel_data->firmware_headfile.firmware_data = FW_20701_HX83112A_TXD;
                panel_data->firmware_headfile.firmware_size = sizeof(FW_20701_HX83112A_TXD);
			}else if (strstr(boot_command_line, "hx83112a_txd_boe_new_fives")) {
				panel_data->vid_len = 10;
			 	memcpy(panel_data->manufacture_info.version, "HX112ANEW_", 10);
			    panel_data->firmware_headfile.firmware_data = FW_20701_HX83112A_TXD;
                panel_data->firmware_headfile.firmware_size = sizeof(FW_20701_HX83112A_TXD);
			}else {
                panel_data->firmware_headfile.firmware_data = NULL;
                panel_data->firmware_headfile.firmware_size = 0;
           }
           break;

           default:
               panel_data->firmware_headfile.firmware_data = NULL;
               panel_data->firmware_headfile.firmware_size = 0;
        }
		/*jianghao tp buring up  end */

        pr_info("Vendor:%s\n", vendor);
        pr_info("Fw:%s\n", panel_data->fw_name);
        pr_info("Limit:%s\n", panel_data->test_limit_name==NULL?"NO Limit":panel_data->test_limit_name);
        pr_info("Extra:%s\n", panel_data->extra==NULL?"NO Extra":panel_data->extra);
        pr_info("is matched %d, type %d\n", is_tp_type_got_in_match, panel_data->tp_type);
        return 0;
    }
    if (is_project(OPPO_18073) || is_project(OPPO_18593)) {
        panel_data->test_limit_name = kzalloc(sizeof(S3706_BASELINE_TEST_LIMIT_NAME_18073), GFP_KERNEL);
        if (panel_data->test_limit_name == NULL) {
            pr_err("[TP]panel_data.test_limit_name kzalloc error\n");
            return -1;
        }
        strcpy(panel_data->test_limit_name, S3706_BASELINE_TEST_LIMIT_NAME_18073);
        strcpy(panel_data->fw_name, S3706_FW_NAME_18073);
        pr_err("[TP]%s: fw_name = %s \n",__func__, panel_data->fw_name);
    }
    if (is_project(OPPO_19301) || is_project(OPPO_19011)) {
        panel_data->test_limit_name = kzalloc(sizeof(S3706_BASELINE_TEST_LIMIT_NAME_19301), GFP_KERNEL);
        if (panel_data->test_limit_name == NULL) {
            pr_err("[TP]panel_data.test_limit_name kzalloc error\n");
            return -1;
        }
        strcpy(panel_data->test_limit_name, S3706_BASELINE_TEST_LIMIT_NAME_19301);
        strcpy(panel_data->fw_name, S3706_FW_NAME_19301);
        pr_err("[TP]%s: fw_name = %s \n",__func__, panel_data->fw_name);
    }
    switch(get_project()) {
    case OPPO_19550:
    case OPPO_19551:
    case OPPO_19553:
    case OPPO_19556:
    case OPPO_19597:
         if (strstr(boot_command_line, "oppo18073_samsung_ams641rw01_1080p_dsi_cmd")) {
             panel_data->test_limit_name = kzalloc(sizeof(S3706_BASELINE_TEST_LIMIT_NAME_18073), GFP_KERNEL);
             if (panel_data->test_limit_name == NULL) {
                 pr_err("[TP]panel_data.test_limit_name kzalloc error\n");
                 return -1;
             }
             strcpy(panel_data->test_limit_name, S3706_BASELINE_TEST_LIMIT_NAME_18073);
             strcpy(panel_data->fw_name, S3706_FW_NAME_18073);
             pr_err("[TP]%s: fw_name = %s \n",__func__, panel_data->fw_name);
         } else if (strstr(boot_command_line, "oppo19551_samsung_ams644vk01_1080p_dsi_cmd")) {
             panel_data->test_limit_name = kzalloc(sizeof(GT9886_BASELINE_TEST_LIMIT_NAME_19551), GFP_KERNEL);
             if (panel_data->test_limit_name == NULL) {
             pr_err("[TP]panel_data.test_limit_name kzalloc error\n");
             return -1;
             }
             strcpy(panel_data->test_limit_name, GT9886_BASELINE_TEST_LIMIT_NAME_19551);
             strcpy(panel_data->fw_name, GT9886_FW_NAME_19551);
             memcpy(panel_data->manufacture_info.version, "0xbd2860000", 11);
             pr_err("[TP]%s: fw_name = %s \n",__func__, panel_data->fw_name);
         }
         break;
     case OPPO_19357:
     case OPPO_19358:
     case OPPO_19359:
     case OPPO_19354:
         if (strstr(boot_command_line, "oppo18073_samsung_ams641rw01_1080p_dsi_cmd")) {
             panel_data->test_limit_name = kzalloc(sizeof(S3706_BASELINE_TEST_LIMIT_NAME_18073), GFP_KERNEL);
             if (panel_data->test_limit_name == NULL) {
                 pr_err("[TP]panel_data.test_limit_name kzalloc error\n");
                 return -1;
             }
             strcpy(panel_data->test_limit_name, S3706_BASELINE_TEST_LIMIT_NAME_18073);
             strcpy(panel_data->fw_name, S3706_FW_NAME_18073);
             pr_err("[TP]%s: fw_name = %s \n",__func__, panel_data->fw_name);
         } else if (strstr(boot_command_line, "oppo19357_samsung_ams644va04_1080p_dsi_cmd")) {
             panel_data->test_limit_name = kzalloc(sizeof(GT9886_BASELINE_TEST_LIMIT_NAME_19357), GFP_KERNEL);
             if (panel_data->test_limit_name == NULL) {
             pr_err("[TP]panel_data.test_limit_name kzalloc error\n");
             return -1;
             }
             strcpy(panel_data->test_limit_name, GT9886_BASELINE_TEST_LIMIT_NAME_19357);
             strcpy(panel_data->fw_name, GT9886_FW_NAME_19357);
             memcpy(panel_data->manufacture_info.version, "0xfa1920000", 11);
             pr_err("[TP]%s: fw_name = %s \n",__func__, panel_data->fw_name);
         }
         break;
	/*jianghao tp buring up  start */	 
	 case OPPO_20701:
		if (strstr(boot_command_line, "ili9882n_txd_inx")) {
             panel_data->test_limit_name = kzalloc(sizeof(ili9882n_BASELINE_TEST_LIMIT_NAME_20701), GFP_KERNEL);
             if (panel_data->test_limit_name == NULL) {
                 pr_err("[TP]panel_data.test_limit_name kzalloc error\n");
                 return -1;
             }
             strcpy(panel_data->test_limit_name, ili9882n_BASELINE_TEST_LIMIT_NAME_20701);
             strcpy(panel_data->fw_name, ili9882n_FW_NAME_20701);
             pr_err("[TP]%s: fw_name = %s \n",__func__, panel_data->fw_name);
		}

		if (strstr(boot_command_line, "ili9882n_mn228_txd_inx")) {
             panel_data->test_limit_name = kzalloc(sizeof(ili9882n_MN228_BASELINE_TEST_LIMIT_NAME_20701), GFP_KERNEL);
             if (panel_data->test_limit_name == NULL) {
                 pr_err("[TP]panel_data.test_limit_name kzalloc error\n");
                 return -1;
             }
             strcpy(panel_data->test_limit_name, ili9882n_MN228_BASELINE_TEST_LIMIT_NAME_20701);
             strcpy(panel_data->fw_name, ili9882n_MN228_FW_NAME_20701);
             pr_err("[TP]%s: fw_name = %s \n",__func__, panel_data->fw_name);
		}
		if (strstr(boot_command_line, "ili9882n_truly_truly")) {
             panel_data->test_limit_name = kzalloc(sizeof(ili9882n_TRULY_BASELINE_TEST_LIMIT_NAME_20701), GFP_KERNEL);
             if (panel_data->test_limit_name == NULL) {
                 pr_err("[TP]panel_data.test_limit_name kzalloc error\n");
                 return -1;
             }
             strcpy(panel_data->test_limit_name, ili9882n_TRULY_BASELINE_TEST_LIMIT_NAME_20701);
             strcpy(panel_data->fw_name, ili9882n_TRULY_FW_NAME_20701);
             pr_err("[TP]%s: fw_name = %s \n",__func__, panel_data->fw_name);
		}
		if (strstr(boot_command_line, "ili9882h_ls_panda")) {
             panel_data->test_limit_name = kzalloc(sizeof(ili9882h_BASELINE_TEST_LIMIT_NAME_20701), GFP_KERNEL);
             if (panel_data->test_limit_name == NULL) {
                 pr_err("[TP]panel_data.test_limit_name kzalloc error\n");
                 return -1;
             }
             strcpy(panel_data->test_limit_name, ili9882h_BASELINE_TEST_LIMIT_NAME_20701);
             strcpy(panel_data->fw_name, ili9882h_FW_NAME_20701);
             pr_err("[TP]%s: fw_name = %s \n",__func__, panel_data->fw_name);
		}

		if (strstr(boot_command_line, "nt36525b_hlt_boe")) {
             panel_data->test_limit_name = kzalloc(sizeof(nt36525b_BASELINE_TEST_LIMIT_NAME_20701), GFP_KERNEL);
             if (panel_data->test_limit_name == NULL) {
                 pr_err("[TP]panel_data.test_limit_name kzalloc error\n");
                 return -1;
             }
             strcpy(panel_data->test_limit_name, nt36525b_BASELINE_TEST_LIMIT_NAME_20701);
             strcpy(panel_data->fw_name, nt36525b_FW_NAME_20701);
             pr_err("[TP]%s: fw_name = %s \n",__func__, panel_data->fw_name);
		}

		if (strstr(boot_command_line, "hx83102d_txd_inx")) {
             panel_data->test_limit_name = kzalloc(sizeof(hx83102d_BASELINE_TEST_LIMIT_NAME_20701), GFP_KERNEL);
             if (panel_data->test_limit_name == NULL) {
                 pr_err("[TP]panel_data.test_limit_name kzalloc error\n");
                 return -1;
             }
             strcpy(panel_data->test_limit_name, hx83102d_BASELINE_TEST_LIMIT_NAME_20701);
             strcpy(panel_data->fw_name, hx83102d_FW_NAME_20701);
             pr_err("[TP]%s: fw_name = %s \n",__func__, panel_data->fw_name);
		}

		if (strstr(boot_command_line, "hx83102d_truly_truly")) {
             panel_data->test_limit_name = kzalloc(sizeof(hx83102d_TRULY_BASELINE_TEST_LIMIT_NAME_20701), GFP_KERNEL);
             if (panel_data->test_limit_name == NULL) {
                 pr_err("[TP]panel_data.test_limit_name kzalloc error\n");
                 return -1;
             }
             strcpy(panel_data->test_limit_name, hx83102d_TRULY_BASELINE_TEST_LIMIT_NAME_20701);
             strcpy(panel_data->fw_name, hx83102d_TRULY_FW_NAME_20701);
             pr_err("[TP]%s: fw_name = %s \n",__func__, panel_data->fw_name);
		}

		if (strstr(boot_command_line, "hx83112a_txd_boe_fives")) {
			 panel_data->test_limit_name = kzalloc(sizeof(hx83112a_BASELINE_TEST_LIMIT_NAME_20701), GFP_KERNEL);
				 if (panel_data->test_limit_name == NULL) {
				     pr_err("[TP]panel_data.test_limit_name kzalloc error\n");
					 return -1;
			 }
			 strcpy(panel_data->test_limit_name, hx83112a_BASELINE_TEST_LIMIT_NAME_20701);
			 strcpy(panel_data->fw_name, hx83112a_FW_NAME_20701);
			 pr_err("[TP]%s: fw_name = %s \n",__func__, panel_data->fw_name);
		}
		if (strstr(boot_command_line, "hx83112a_txd_boe_new_fives")) {
			 panel_data->test_limit_name = kzalloc(sizeof(hx83112a_BASELINE_TEST_LIMIT_NAME_20701), GFP_KERNEL);
			 if (panel_data->test_limit_name == NULL) {
				 pr_err("[TP]panel_data.test_limit_name kzalloc error\n");
				 return -1;
			 }
			 strcpy(panel_data->test_limit_name, hx83112a_BASELINE_TEST_LIMIT_NAME_20701);
			 strcpy(panel_data->fw_name, hx83112a_FW_NAME_20701);
			 pr_err("[TP]%s: fw_name = %s \n",__func__, panel_data->fw_name);
		}

		if (strstr(boot_command_line, "ft8006saa_truly_truly")) {
             panel_data->test_limit_name = kzalloc(sizeof(ft8006s_TRULY_BASELINE_TEST_LIMIT_NAME_20701), GFP_KERNEL);
             if (panel_data->test_limit_name == NULL) {
                 pr_err("[TP]panel_data.test_limit_name kzalloc error\n");
                 return -1;
             }
             strcpy(panel_data->test_limit_name, ft8006s_TRULY_BASELINE_TEST_LIMIT_NAME_20701);
             strcpy(panel_data->fw_name, ft8006s_TRULY_FW_NAME_20701);
             pr_err("[TP]%s: fw_name = %s \n",__func__, panel_data->fw_name);
		}

		if (strstr(boot_command_line, "ft8006s_hlt_boe")) {
             panel_data->test_limit_name = kzalloc(sizeof(ft8006s_BASELINE_TEST_LIMIT_NAME_20701), GFP_KERNEL);
             if (panel_data->test_limit_name == NULL) {
                 pr_err("[TP]panel_data.test_limit_name kzalloc error\n");
                 return -1;
             }
             strcpy(panel_data->test_limit_name, ft8006s_BASELINE_TEST_LIMIT_NAME_20701);
             strcpy(panel_data->fw_name, ft8006s_FW_NAME_20701);
             pr_err("[TP]%s: fw_name = %s \n",__func__, panel_data->fw_name);
		}
		

		if (strstr(boot_command_line, "ft8722_hlt_boe_eighth")) {
             panel_data->test_limit_name = kzalloc(sizeof(ft8722_BASELINE_TEST_LIMIT_NAME_20701), GFP_KERNEL);
             if (panel_data->test_limit_name == NULL) {
                 pr_err("[TP]panel_data.test_limit_name kzalloc error\n");
                 return -1;
             }
             strcpy(panel_data->test_limit_name, ft8722_BASELINE_TEST_LIMIT_NAME_20701);
             strcpy(panel_data->fw_name, ft8722_FW_NAME_20701);
             pr_err("[TP]%s: fw_name = %s \n",__func__, panel_data->fw_name);
		}
		break;
	/*jianghao tp buring up  end */
    }

    strcpy(panel_data->manufacture_info.manufacture, "SAMSUNG");
    panel_data->manufacture_info.fw_path = panel_data->fw_name;
    //pr_info("Vendor:%s\n", vendor);
    //pr_info("Fw:%s\n", panel_data->fw_name);
    //pr_info("Limit:%s\n", panel_data->test_limit_name==NULL?"NO Limit":panel_data->test_limit_name);
    //pr_info("Extra:%s\n", panel_data->extra==NULL?"NO Extra":panel_data->extra);
    //pr_info("is matched %d, type %d\n", is_tp_type_got_in_match, panel_data->tp_type);
    return 0;
}

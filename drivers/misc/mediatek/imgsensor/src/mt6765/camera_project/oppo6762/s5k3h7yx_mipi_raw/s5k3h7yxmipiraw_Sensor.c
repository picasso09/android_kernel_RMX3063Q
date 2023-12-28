/*****************************************************************************
 *
 * Filename:
 * ---------
 *   S5K3H7mipi_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>


#include "s5k3h7yxmipiraw_Sensor.h"

#define PFX "s5k3h7yx_camera_sensor"


#if 0
#define LOG_INF(format, args...)    pr_info(PFX "[%s] " format, __FUNCTION__, ##args)
#else
#define LOG_INF(format, args...)
#endif
#define LOG_ERR(format, args...)    pr_err(PFX "[%s] " format, __FUNCTION__, ##args)


static DEFINE_SPINLOCK(imgsensor_drv_lock);

#define SLOW_MOTION_120FPS

static struct imgsensor_info_struct imgsensor_info = {
    .sensor_id = S5K3H7YX_SENSOR_ID,
    .checksum_value = 0x9c198b8c,
    .pre = {
        .pclk = 280000000,
        .linelength = 3688,
        .framelength = 2530,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1632,
        .grabwindow_height = 1224,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 264000000,
        .max_framerate = 300,
    },
    .cap = {
        .pclk = 280000000,
        .linelength = 3688,
        .framelength = 2485,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 2448,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 264000000,
        .max_framerate = 300,
        },
    .cap1 = {
        .pclk = 280000000,
        .linelength = 3688,
        .framelength = 2485,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 2448,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 264000000,
        .max_framerate = 300,
    },
    .normal_video = {
        .pclk = 280000000,
        .linelength = 3688,
        .framelength = 2528,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 3264,
        .grabwindow_height = 1840,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 264000000,
        .max_framerate = 300,
    },
    .hs_video = {
        .pclk = 280000000,
        .linelength = 3688,
        .framelength = 628,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 640,
        .grabwindow_height = 480,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 264000000,
        .max_framerate = 1200,
    },
    .slim_video = {
        .pclk = 140000000,
        .linelength = 3088,
        .framelength = 1510,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 640,
        .grabwindow_height = 480,
        .mipi_data_lp2hs_settle_dc = 85,
        .mipi_pixel_rate = 208000000,
        .max_framerate = 300,
    },

    .margin = 8,
    .min_shutter = 3,
    .max_frame_length = 0xffff,
    .ae_shut_delay_frame = 0,
    .ae_sensor_gain_delay_frame = 0,
    .ae_ispGain_delay_frame = 2,
    .ihdr_support = 0,      //1, support; 0,not support
    .ihdr_le_firstline = 0,  //1,le first ; 0, se first
    .sensor_mode_num = 5,      //support sensor mode num

    .cap_delay_frame = 3,
    .pre_delay_frame = 3,
    .video_delay_frame = 2,
    .hs_video_delay_frame = 2,
    .slim_video_delay_frame = 2,

    .isp_driving_current = ISP_DRIVING_4MA,
    .sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
    .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
    .mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANUAL
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,
    .mclk = 24,
    .mipi_lane_num = SENSOR_MIPI_4_LANE,
    .i2c_addr_table = {0x20, 0x5a, 0xff},
};


static struct imgsensor_struct imgsensor = {
    .mirror = IMAGE_NORMAL,                //mirrorflip information
    .sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
    .shutter = 0x3D0,                    //current shutter
    .gain = 0x100,                        //current gain
    .dummy_pixel = 0,                    //current dummypixel
    .dummy_line = 0,                    //current dummyline
    .current_fps = 30,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
    .autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
    .test_pattern = KAL_FALSE,        //test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
    .ihdr_en = KAL_FALSE, //sensor need support LE, SE with HDR feature
    .i2c_write_id = 0x20,
    .current_ae_effective_frame = 1,
};


/* Sensor output window information */
static struct SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =
{{ 3280, 2464,     4,   4, 3264, 2448, 1632, 1224, 0, 0, 1632, 1224, 0, 0, 1632, 1224}, // Preview
 { 3280, 2464,     4,   4, 3264, 2448, 3264, 2448, 0, 0, 3264, 2448, 0, 0, 3264, 2448}, // capture
 { 3280, 2464,     4,   308, 3264, 1840, 3264, 1840, 0, 0, 3264, 1840, 0, 0, 3264, 1840}, // video
 { 3280, 2464, 356, 268, 2560, 1920, 640, 480, 0, 0, 640,  480, 0, 0, 640,  480}, // hight speed video
 { 3280, 2464, 356, 268, 2560, 1920, 2560, 1920, 0, 0, 640,  480, 0, 0, 640,  480}};// slim video

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 2, imgsensor.i2c_write_id);
    return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}


static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
    iWriteRegI2C(pusendcmd , 4, imgsensor.i2c_write_id);
}
/*
static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,imgsensor.i2c_write_id);
    return get_byte;
}
*/
static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
    iWriteRegI2C(pusendcmd , 3, imgsensor.i2c_write_id);
}


static void set_dummy(void)
{

    LOG_INF("dummyline = %d, dummypixels = %d ", imgsensor.dummy_line, imgsensor.dummy_pixel);
    write_cmos_sensor_8(0x0104, 0x01);
    write_cmos_sensor(0x0340, imgsensor.frame_length);
    write_cmos_sensor(0x0342, imgsensor.line_length);
    write_cmos_sensor_8(0x0104, 0x00);


}    /*    set_dummy  */

static kal_uint32 return_sensor_id(void)
{
    return (read_cmos_sensor(0x0000));
}
static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
    kal_uint32 frame_length = imgsensor.frame_length;

    LOG_INF("framerate = %d, min framelength should enable?(%d) \n", framerate,min_framelength_en);

    frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
    spin_lock(&imgsensor_drv_lock);
    imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length;
    imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
    //dummy_line = frame_length - imgsensor.min_frame_length;
    //if (dummy_line < 0)
        //imgsensor.dummy_line = 0;
    //else
        //imgsensor.dummy_line = dummy_line;
    //imgsensor.frame_length = frame_length + imgsensor.dummy_line;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
    {
        imgsensor.frame_length = imgsensor_info.max_frame_length;
        imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
    }
    if (min_framelength_en)
        imgsensor.min_frame_length = imgsensor.frame_length;
    spin_unlock(&imgsensor_drv_lock);
    set_dummy();


}    /*    set_max_framerate  */

static void check_streamoff(void)
{
    unsigned int i = 0;
    int timeout = (10000 / imgsensor.current_fps) + 1;

    mdelay(3);
    for (i = 0; i < timeout; i++) {
        if (read_cmos_sensor(0x0005) != 0xFF)
            mdelay(1);
        else
            break;
    }
    LOG_INF("%s exit!\n", __func__);
}
static kal_uint32 streaming_control(kal_bool enable)
{
    LOG_INF("streaming_enable(0=Sw Standby,1=streaming): %d\n", enable);

    if (enable) {
      write_cmos_sensor(0x0100, 0x0103);
    } else {
      write_cmos_sensor(0x0100, 0x0003);
        check_streamoff();
    }
    return ERROR_NONE;
}

static void write_shutter(unsigned long long shutter)
{

    //kal_uint16 realtime_fps = 0;
    static int long_expose_stats =0;
    static unsigned int reg_value = 0;
    spin_lock(&imgsensor_drv_lock);
    if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = shutter + imgsensor_info.margin;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);
    shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
    //shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

    if(shutter > 65535){

        /* Frame exposure mode customization for LE*/
        imgsensor.ae_frm_mode.frame_mode_1 = IMGSENSOR_AE_MODE_SE;
        imgsensor.ae_frm_mode.frame_mode_2 = IMGSENSOR_AE_MODE_SE;
        imgsensor.current_ae_effective_frame = 1;


        long_expose_stats = 1;
        switch(shutter){
        case 75918://1
            reg_value = 0x10B1;
            break;
        case 113877://1.5
            reg_value = 0x190A;
            break;
        case 189796://2.5
            reg_value = 0x29BC;
            break;
        case 227755://3
            reg_value = 0x3214;
            break;
        case 303674://4
        case 303675:
            reg_value = 0x42C6;
            break;
        case 379593://5
            reg_value = 0x5377;
            break;
        case 455511://6
            reg_value = 0x6429;
            break;
        case 531430://7
            reg_value = 0x74DA;
            break;
        case 607348://8
            reg_value = 0x858C;
            break;
        case 759186://10
            reg_value = 0xA6EF;
            break;
        case 986941://13
            reg_value = 0xD904;
            break;
        default:
            LOG_ERR("zyk longexpose unsoported shutter is %ld,",shutter);
            break;

        }
        LOG_ERR("zyk longexpose gogo mtk enable shutter is %ld,",shutter);
        streaming_control(0);

        write_cmos_sensor_8(0x0105,0x03);
        write_cmos_sensor_8(0x3015,0x01);
        write_cmos_sensor_8(0x021C,0x01);

        write_cmos_sensor_8(0x0104,0x01);
        write_cmos_sensor(0x0202,0x0005);

        //write_cmos_sensor_8(0x0104,0x00);
        //write_cmos_sensor_8(0x0104,0x01);

        write_cmos_sensor(0x021E,reg_value + 0x10);// 0x10C1
        write_cmos_sensor(0x0342,0xFFF0);
        write_cmos_sensor(0x0218,reg_value);
        write_cmos_sensor(0x0340,reg_value + 0x10);
        write_cmos_sensor(0x0342,0xFFF0);
        write_cmos_sensor(0x0202,reg_value);
        write_cmos_sensor_8(0x0104,0x00);
        streaming_control(1);

    }else{
        if(long_expose_stats){
            LOG_ERR("zyk longexpose exit shutter is %ld,",shutter);
            streaming_control(0);
            write_cmos_sensor_8(0x0105,0x04);
            write_cmos_sensor_8(0x3015,0x00);
            write_cmos_sensor_8(0x021C,0x00);
            write_cmos_sensor_8(0x0104,0x01);

            write_cmos_sensor(0x021E,0x09B5);
            write_cmos_sensor(0x0202,0x0100);
            write_cmos_sensor(0x0342,0x0E68);
            write_cmos_sensor(0x0340,0x09B5);
            write_cmos_sensor_8(0x0104,0x00);
            streaming_control(1);
            long_expose_stats = 0;
        }else{
            imgsensor.current_ae_effective_frame = 1;
            // Update Shutter
            //write_cmos_sensor_8(0x0104,0x01);
            write_cmos_sensor(0x0340, imgsensor.frame_length);
            write_cmos_sensor(0x0202, shutter);
            //write_cmos_sensor_8(0x0104,0x00);
        }
    }
    LOG_INF("Currently camera mode is %d,shutter is %d, framelength=%d,linelength=%d\n",imgsensor.sensor_mode,shutter,imgsensor.frame_length,imgsensor.line_length);

}    /*    write_shutter  */



/*************************************************************************
* FUNCTION
*    set_shutter
*
* DESCRIPTION
*    This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*    iShutter : exposured lines
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(unsigned long shutter)
{
    unsigned long flags;

    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    write_shutter(shutter);
}    /*    set_shutter */
static kal_uint16 gain2reg(const kal_uint16 gain)
{
    kal_uint16 reg_gain = 0x0;

    reg_gain = gain/2;
    return (kal_uint16)reg_gain;
}

/*************************************************************************
* FUNCTION
*    set_gain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    iGain : sensor global gain(base: 0x40)
*
* RETURNS
*    the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
     kal_uint16 reg_gain;

     /* 0x350A[0:1], 0x350B[0:7] AGC real gain */
     /* [0:3] = N meams N /16 X  */
     /* [4:9] = M meams M X       */
     /* Total gain = M + N /16 X   */

     //
     if (gain < BASEGAIN || gain > 32 * BASEGAIN) {
         LOG_INF("Error gain setting");

         if (gain < BASEGAIN)
             gain = BASEGAIN;
         else if (gain > 32 * BASEGAIN)
             gain = 32 * BASEGAIN;
     }

     reg_gain = gain2reg(gain);
     spin_lock(&imgsensor_drv_lock);
     imgsensor.gain = reg_gain;
     spin_unlock(&imgsensor_drv_lock);
     LOG_INF("gain = %d , reg_gain = 0x%x,shutter=%d,the result of gain*shutter is %d ", gain, reg_gain,imgsensor.shutter,gain*(imgsensor.shutter));

     //write_cmos_sensor_8(0x0104, 0x01);
     write_cmos_sensor_8(0x0204,(reg_gain>>8));
     write_cmos_sensor_8(0x0205,(reg_gain&0xff));
     //write_cmos_sensor_8(0x0104, 0x00);


     return gain;

}

static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
    #if 1
    LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
    if (imgsensor.ihdr_en) {

        spin_lock(&imgsensor_drv_lock);
        if (le > imgsensor.min_frame_length - imgsensor_info.margin)
            imgsensor.frame_length = le + imgsensor_info.margin;
        else
            imgsensor.frame_length = imgsensor.min_frame_length;
        if (imgsensor.frame_length > imgsensor_info.max_frame_length)
            imgsensor.frame_length = imgsensor_info.max_frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (le < imgsensor_info.min_shutter) le = imgsensor_info.min_shutter;
        if (se < imgsensor_info.min_shutter) se = imgsensor_info.min_shutter;


                // Extend frame length first
        write_cmos_sensor_8(0x0104,0x01);
        write_cmos_sensor(0x0340, imgsensor.frame_length);

        //write_cmos_sensor(0x0202, se);
        //write_cmos_sensor(0x021e,le);
        write_cmos_sensor(0x602A,0x021e);
        write_cmos_sensor(0x6f12,le);
        write_cmos_sensor(0x602A,0x0202);
        write_cmos_sensor(0x6f12,se);
         write_cmos_sensor_8(0x0104,0x00);
        LOG_INF("iHDR:imgsensor.frame_length=%d\n",imgsensor.frame_length);
        set_gain(gain);
    }

    #endif

}

#if 0
static void set_mirror_flip(kal_uint8 image_mirror)
{
    LOG_INF("image_mirror = %d", image_mirror);

    /********************************************************
       *
       *   0x3820[2] ISP Vertical flip
       *   0x3820[1] Sensor Vertical flip
       *
       *   0x3821[2] ISP Horizontal mirror
       *   0x3821[1] Sensor Horizontal mirror
       *
       *   ISP and Sensor flip or mirror register bit should be the same!!
       *
       ********************************************************/
    spin_lock(&imgsensor_drv_lock);
    imgsensor.mirror= image_mirror;
    spin_unlock(&imgsensor_drv_lock);
    switch (image_mirror) {

        case IMAGE_NORMAL:
            write_cmos_sensor_8(0x0101,0x00);   // Gr
            break;
        case IMAGE_H_MIRROR:
            write_cmos_sensor_8(0x0101,0x01);
            break;
        case IMAGE_V_MIRROR:
            write_cmos_sensor_8(0x0101,0x02);
            break;
        case IMAGE_HV_MIRROR:
            write_cmos_sensor_8(0x0101,0x03);//Gb
            break;
        default:
            LOG_INF("Error image_mirror setting\n");
        break;
    }

}
#endif
#define MULTI_WRITE 1
#if MULTI_WRITE
#define I2C_BUFFER_LEN 765


static kal_uint16 s5k3h7yx_table_write_cmos_sensor(
                    kal_uint16 *para, kal_uint32 len)
{
    char puSendCmd[I2C_BUFFER_LEN];
    kal_uint32 tosend, IDX;
    kal_uint16 addr = 0, addr_last = 0, data;

    tosend = 0;
    IDX = 0;
    while (len > IDX) {
        addr = para[IDX];

        {
            puSendCmd[tosend++] = (char)(addr >> 8);
            puSendCmd[tosend++] = (char)(addr & 0xFF);
            data = para[IDX + 1];
            puSendCmd[tosend++] = (char)(data >> 8);
            puSendCmd[tosend++] = (char)(data & 0xFF);
            IDX += 2;
            addr_last = addr;
        }

        if ((I2C_BUFFER_LEN - tosend) < 4 ||
            len == IDX ||
            addr != addr_last) {
            iBurstWriteReg_multi(puSendCmd, tosend,
                imgsensor.i2c_write_id,
                4, 400);

            tosend = 0;
        }
    }
    return 0;
}


static kal_uint16 addr_data_pair_init_s5k3h7yx[] = {
    0x6028,0x7000,
    0x602A,0x1750,
    0x6F12,0x10B5,
    0x6F12,0x00F0,
    0x6F12,0xE1FB,
    0x6F12,0x00F0,
    0x6F12,0xE3FB,
    0x6F12,0x10BC,
    0x6F12,0x08BC,
    0x6F12,0x1847,
    0x6F12,0x2DE9,
    0x6F12,0xF041,
    0x6F12,0x9FE5,
    0x6F12,0x2437,
    0x6F12,0x9FE5,
    0x6F12,0x2447,
    0x6F12,0x83E2,
    0x6F12,0x01CC,
    0x6F12,0x84E5,
    0x6F12,0x0030,
    0x6F12,0x84E5,
    0x6F12,0x04C0,
    0x6F12,0x83E2,
    0x6F12,0x02CC,
    0x6F12,0x83E2,
    0x6F12,0x033C,
    0x6F12,0x84E5,
    0x6F12,0x0C30,
    0x6F12,0x44E2,
    0x6F12,0x0C70,
    0x6F12,0x84E5,
    0x6F12,0x08C0,
    0x6F12,0xD7E1,
    0x6F12,0xB230,
    0x6F12,0xA0E3,
    0x6F12,0x0150,
    0x6F12,0x9FE5,
    0x6F12,0xFC66,
    0x6F12,0xA0E1,
    0x6F12,0x15C3,
    0x6F12,0x83E2,
    0x6F12,0x0130,
    0x6F12,0x8CE1,
    0x6F12,0x1533,
    0x6F12,0xD6E1,
    0x6F12,0xB0C0,
    0x6F12,0xCCE1,
    0x6F12,0x0330,
    0x6F12,0xC6E1,
    0x6F12,0xB030,
    0x6F12,0x00EB,
    0x6F12,0xDD01,
    0x6F12,0xD7E1,
    0x6F12,0xB200,
    0x6F12,0xA0E1,
    0x6F12,0x1510,
    0x6F12,0x80E2,
    0x6F12,0x0100,
    0x6F12,0x81E1,
    0x6F12,0x1500,
    0x6F12,0xD6E1,
    0x6F12,0xB010,
    0x6F12,0x80E1,
    0x6F12,0x0100,
    0x6F12,0x9FE5,
    0x6F12,0xC816,
    0x6F12,0xC6E1,
    0x6F12,0xB000,
    0x6F12,0x91E5,
    0x6F12,0x0000,
    0x6F12,0xD0E5,
    0x6F12,0x4801,
    0x6F12,0x50E3,
    0x6F12,0x0000,
    0x6F12,0x000A,
    0x6F12,0x0D00,
    0x6F12,0x94E5,
    0x6F12,0x0020,
    0x6F12,0xC2E5,
    0x6F12,0x0000,
    0x6F12,0x91E5,
    0x6F12,0x0000,
    0x6F12,0x94E5,
    0x6F12,0x0420,
    0x6F12,0xD0E5,
    0x6F12,0x4801,
    0x6F12,0xC2E5,
    0x6F12,0x0000,
    0x6F12,0x91E5,
    0x6F12,0x0000,
    0x6F12,0x94E5,
    0x6F12,0x0820,
    0x6F12,0xD0E5,
    0x6F12,0x4801,
    0x6F12,0xC2E5,
    0x6F12,0x0000,
    0x6F12,0x91E5,
    0x6F12,0x0000,
    0x6F12,0x94E5,
    0x6F12,0x0C10,
    0x6F12,0xD0E5,
    0x6F12,0x4801,
    0x6F12,0xC1E5,
    0x6F12,0x0000,
    0x6F12,0xBDE8,
    0x6F12,0xF041,
    0x6F12,0x2FE1,
    0x6F12,0x1EFF,
    0x6F12,0x2DE9,
    0x6F12,0x7040,
    0x6F12,0x9FE5,
    0x6F12,0x7066,
    0x6F12,0xA0E3,
    0x6F12,0x0140,
    0x6F12,0xD6E1,
    0x6F12,0xB010,
    0x6F12,0x9FE5,
    0x6F12,0x5C56,
    0x6F12,0xA0E1,
    0x6F12,0x1421,
    0x6F12,0x81E2,
    0x6F12,0x0110,
    0x6F12,0x82E1,
    0x6F12,0x1411,
    0x6F12,0xD5E1,
    0x6F12,0xB020,
    0x6F12,0xC2E1,
    0x6F12,0x0110,
    0x6F12,0xC5E1,
    0x6F12,0xB010,
    0x6F12,0x00EB,
    0x6F12,0xB701,
    0x6F12,0xD6E1,
    0x6F12,0xB000,
    0x6F12,0x9FE5,
    0x6F12,0x4C26,
    0x6F12,0xA0E1,
    0x6F12,0x1410,
    0x6F12,0x80E2,
    0x6F12,0x0100,
    0x6F12,0x81E1,
    0x6F12,0x1400,
    0x6F12,0xD5E1,
    0x6F12,0xB010,
    0x6F12,0x80E1,
    0x6F12,0x0100,
    0x6F12,0xC5E1,
    0x6F12,0xB000,
    0x6F12,0x9FE5,
    0x6F12,0x2806,
    0x6F12,0xD0E1,
    0x6F12,0xB00C,
    0x6F12,0xA0E1,
    0x6F12,0xA011,
    0x6F12,0x9FE5,
    0x6F12,0x2006,
    0x6F12,0x90E5,
    0x6F12,0x0000,
    0x6F12,0xD0E1,
    0x6F12,0xBA39,
    0x6F12,0x53E1,
    0x6F12,0x0100,
    0x6F12,0xD091,
    0x6F12,0xBE09,
    0x6F12,0xD081,
    0x6F12,0xBC09,
    0x6F12,0xC2E1,
    0x6F12,0xB003,
    0x6F12,0xBDE8,
    0x6F12,0x7040,
    0x6F12,0x2FE1,
    0x6F12,0x1EFF,
    0x6F12,0x2DE9,
    0x6F12,0xF840,
    0x6F12,0x10E3,
    0x6F12,0x0100,
    0x6F12,0xA0E1,
    0x6F12,0x0040,
    0x6F12,0x9F15,
    0x6F12,0xF805,
    0x6F12,0x9015,
    0x6F12,0x2400,
    0x6F12,0x5013,
    0x6F12,0x0000,
    0x6F12,0x000A,
    0x6F12,0x2000,
    0x6F12,0x9FE5,
    0x6F12,0xEC05,
    0x6F12,0xA0E3,
    0x6F12,0x3510,
    0x6F12,0xD0E5,
    0x6F12,0x5E00,
    0x6F12,0x9FE5,
    0x6F12,0xE455,
    0x6F12,0x50E3,
    0x6F12,0x0100,
    0x6F12,0xA093,
    0x6F12,0x0100,
    0x6F12,0x81E0,
    0x6F12,0x8060,
    0x6F12,0xA0E3,
    0x6F12,0x3610,
    0x6F12,0x81E0,
    0x6F12,0x8000,
    0x6F12,0xD5E5,
    0x6F12,0xD710,
    0x6F12,0x51E3,
    0x6F12,0x0000,
    0x6F12,0x001A,
    0x6F12,0x0400,
    0x6F12,0xA0E3,
    0x6F12,0x0120,
    0x6F12,0x8DE2,
    0x6F12,0x0010,
    0x6F12,0x00EB,
    0x6F12,0x8F01,
    0x6F12,0xDDE5,
    0x6F12,0x0000,
    0x6F12,0xC5E5,
    0x6F12,0xD700,
    0x6F12,0x9FE5,
    0x6F12,0x9C55,
    0x6F12,0x95E5,
    0x6F12,0x0000,
    0x6F12,0xD0E5,
    0x6F12,0x1102,
    0x6F12,0x50E3,
    0x6F12,0x0000,
    0x6F12,0x001A,
    0x6F12,0x0A00,
    0x6F12,0xA0E3,
    0x6F12,0x0120,
    0x6F12,0x8DE2,
    0x6F12,0x0010,
    0x6F12,0xA0E1,
    0x6F12,0x0600,
    0x6F12,0x00EB,
    0x6F12,0x8401,
    0x6F12,0xDDE5,
    0x6F12,0x0010,
    0x6F12,0x95E5,
    0x6F12,0x0000,
    0x6F12,0xC0E5,
    0x6F12,0x1112,
    0x6F12,0xA0E1,
    0x6F12,0x010C,
    0x6F12,0x9FE5,
    0x6F12,0x6415,
    0x6F12,0xA0E1,
    0x6F12,0xA00B,
    0x6F12,0xC1E1,
    0x6F12,0xBE04,
    0x6F12,0xA0E1,
    0x6F12,0x0400,
    0x6F12,0xBDE8,
    0x6F12,0xF840,
    0x6F12,0x00EA,
    0x6F12,0x7C01,
    0x6F12,0x9FE5,
    0x6F12,0x4C15,
    0x6F12,0xA0E3,
    0x6F12,0x0000,
    0x6F12,0xA0E3,
    0x6F12,0x012C,
    0x6F12,0x81E0,
    0x6F12,0x8030,
    0x6F12,0x83E2,
    0x6F12,0x013C,
    0x6F12,0x80E2,
    0x6F12,0x0100,
    0x6F12,0x50E3,
    0x6F12,0x0400,
    0x6F12,0xC3E1,
    0x6F12,0xBE28,
    0x6F12,0xFFBA,
    0x6F12,0xF9FF,
    0x6F12,0x2FE1,
    0x6F12,0x1EFF,
    0x6F12,0x2DE9,
    0x6F12,0x7040,
    0x6F12,0x9FE5,
    0x6F12,0x20C5,
    0x6F12,0xDCE5,
    0x6F12,0x1021,
    0x6F12,0x52E3,
    0x6F12,0x0000,
    0x6F12,0x001A,
    0x6F12,0x0A00,
    0x6F12,0x9FE5,
    0x6F12,0x28E5,
    0x6F12,0x8CE0,
    0x6F12,0x0231,
    0x6F12,0x8EE0,
    0x6F12,0x8250,
    0x6F12,0xD5E1,
    0x6F12,0xB050,
    0x6F12,0x93E5,
    0x6F12,0xD840,
    0x6F12,0x82E2,
    0x6F12,0x0120,
    0x6F12,0x04E0,
    0x6F12,0x9504,
    0x6F12,0xA0E1,
    0x6F12,0x2444,
    0x6F12,0x52E3,
    0x6F12,0x0400,
    0x6F12,0x83E5,
    0x6F12,0xD840,
    0x6F12,0xFFBA,
    0x6F12,0xF5FF,
    0x6F12,0xBDE8,
    0x6F12,0x7040,
    0x6F12,0x00EA,
    0x6F12,0x6201,
    0x6F12,0x2DE9,
    0x6F12,0x1040,
    0x6F12,0x00EB,
    0x6F12,0x6201,
    0x6F12,0x9FE5,
    0x6F12,0xF004,
    0x6F12,0xD0E5,
    0x6F12,0x7310,
    0x6F12,0xBDE8,
    0x6F12,0x1040,
    0x6F12,0x9FE5,
    0x6F12,0xE804,
    0x6F12,0xFFEA,
    0x6F12,0xE6FF,
    0x6F12,0x2DE9,
    0x6F12,0xFF4F,
    0x6F12,0x9FE5,
    0x6F12,0xD094,
    0x6F12,0xA0E3,
    0x6F12,0x0080,
    0x6F12,0xD9E5,
    0x6F12,0xD700,
    0x6F12,0xD9E5,
    0x6F12,0x9C40,
    0x6F12,0x4DE2,
    0x6F12,0x94D0,
    0x6F12,0x80E2,
    0x6F12,0x0300,
    0x6F12,0x00E2,
    0x6F12,0xFF50,
    0x6F12,0x48E2,
    0x6F12,0x0209,
    0x6F12,0xA0E1,
    0x6F12,0x0370,
    0x6F12,0xCDE1,
    0x6F12,0xB007,
    0x6F12,0xCDE1,
    0x6F12,0xB005,
    0x6F12,0xD7E1,
    0x6F12,0xF600,
    0x6F12,0x54E3,
    0x6F12,0x1000,
    0x6F12,0xA0E3,
    0x6F12,0x0110,
    0x6F12,0x8DE5,
    0x6F12,0x4800,
    0x6F12,0xD7E5,
    0x6F12,0x0800,
    0x6F12,0xA023,
    0x6F12,0x1040,
    0x6F12,0xA0E1,
    0x6F12,0x0860,
    0x6F12,0x80E2,
    0x6F12,0x0900,
    0x6F12,0x8DE5,
    0x6F12,0x4400,
    0x6F12,0xA0E1,
    0x6F12,0x1100,
    0x6F12,0x80E0,
    0x6F12,0xA00F,
    0x6F12,0xA0E1,
    0x6F12,0xC000,
    0x6F12,0x8DE5,
    0x6F12,0x4000,
    0x6F12,0x9FE5,
    0x6F12,0x8014,
    0x6F12,0xA0E3,
    0x6F12,0x2020,
    0x6F12,0x8DE2,
    0x6F12,0x2000,
    0x6F12,0x00EB,
    0x6F12,0x4201,
    0x6F12,0x9FE5,
    0x6F12,0x7414,
    0x6F12,0xA0E3,
    0x6F12,0x1820,
    0x6F12,0x8DE2,
    0x6F12,0x0800,
    0x6F12,0x00EB,
    0x6F12,0x3E01,
    0x6F12,0x9FE5,
    0x6F12,0x3424,
    0x6F12,0x9FE5,
    0x6F12,0x3804,
    0x6F12,0x92E5,
    0x6F12,0x0020,
    0x6F12,0x90E5,
    0x6F12,0x4010,
    0x6F12,0xD2E5,
    0x6F12,0x5921,
    0x6F12,0x9DE5,
    0x6F12,0x9C30,
    0x6F12,0xD0E1,
    0x6F12,0xBE04,
    0x6F12,0x82E0,
    0x6F12,0x8221,
    0x6F12,0x81E0,
    0x6F12,0x8210,
    0x6F12,0x81E0,
    0x6F12,0x8310,
    0x6F12,0xD1E1,
    0x6F12,0xFA30,
    0x6F12,0x60E2,
    0x6F12,0x012C,
    0x6F12,0xD1E1,
    0x6F12,0xF210,
    0x6F12,0x02E0,
    0x6F12,0x9302,
    0x6F12,0x20E0,
    0x6F12,0x9120,
    0x6F12,0xA0E1,
    0x6F12,0x0004,
    0x6F12,0xA0E1,
    0x6F12,0x4008,
    0x6F12,0xA0E3,
    0x6F12,0x00E0,
    0x6F12,0x8DE2,
    0x6F12,0x50B0,
    0x6F12,0x8DE5,
    0x6F12,0x0400,
    0x6F12,0x9FE5,
    0x6F12,0x0024,
    0x6F12,0x82E0,
    0x6F12,0x8800,
    0x6F12,0xD0E1,
    0x6F12,0xFC3B,
    0x6F12,0xA0E3,
    0x6F12,0x0100,
    0x6F12,0x50E1,
    0x6F12,0x0400,
    0x6F12,0x8BD0,
    0x6F12,0x8010,
    0x6F12,0x51D1,
    0x6F12,0xF2C0,
    0x6F12,0xA0D3,
    0x6F12,0x0199,
    0x6F12,0x80D2,
    0x6F12,0x0100,
    0x6F12,0x0CD0,
    0x6F12,0x930C,
    0x6F12,0x89D0,
    0x6F12,0x8CC0,
    0x6F12,0xA0D1,
    0x6F12,0xCCC7,
    0x6F12,0xC1D1,
    0x6F12,0xB0C0,
    0x6F12,0xFFDA,
    0x6F12,0xF5FF,
    0x6F12,0xA0E3,
    0x6F12,0x00C0,
    0x6F12,0x9FE5,
    0x6F12,0xC403,
    0x6F12,0xA0E3,
    0x6F12,0x0199,
    0x6F12,0x80E0,
    0x6F12,0x8C00,
    0x6F12,0xD0E1,
    0x6F12,0xFE39,
    0x6F12,0xA0E3,
    0x6F12,0x0100,
    0x6F12,0x50E1,
    0x6F12,0x0400,
    0x6F12,0x8DD2,
    0x6F12,0x7010,
    0x6F12,0x81D0,
    0x6F12,0x8010,
    0x6F12,0x51D1,
    0x6F12,0xF220,
    0x6F12,0x80D2,
    0x6F12,0x0100,
    0x6F12,0x02D0,
    0x6F12,0x9302,
    0x6F12,0x89D0,
    0x6F12,0x8220,
    0x6F12,0xA0D1,
    0x6F12,0xC227,
    0x6F12,0xC1D1,
    0x6F12,0xB020,
    0x6F12,0xFFDA,
    0x6F12,0xF5FF,
    0x6F12,0xA0E3,
    0x6F12,0x0030,
    0x6F12,0xA0E1,
    0x6F12,0x0320,
    0x6F12,0xA0E1,
    0x6F12,0x0310,
    0x6F12,0x00EA,
    0x6F12,0x1700,
    0x6F12,0xA0E3,
    0x6F12,0x0000,
    0x6F12,0x00EA,
    0x6F12,0x1200,
    0x6F12,0x41E0,
    0x6F12,0x0090,
    0x6F12,0x8DE2,
    0x6F12,0x70A0,
    0x6F12,0x8AE0,
    0x6F12,0x80A0,
    0x6F12,0x8BE0,
    0x6F12,0x8990,
    0x6F12,0xDAE1,
    0x6F12,0xF0A0,
    0x6F12,0xD9E1,
    0x6F12,0xF090,
    0x6F12,0x80E2,
    0x6F12,0x0100,
    0x6F12,0x09E0,
    0x6F12,0x9A09,
    0x6F12,0x9DE5,
    0x6F12,0x94A0,
    0x6F12,0x89E2,
    0x6F12,0x0199,
    0x6F12,0x8AE0,
    0x6F12,0x82A0,
    0x6F12,0xDAE1,
    0x6F12,0xF0A0,
    0x6F12,0xA0E1,
    0x6F12,0xC997,
    0x6F12,0x82E2,
    0x6F12,0x0120,
    0x6F12,0x09E0,
    0x6F12,0x9A09,
    0x6F12,0xA0E3,
    0x6F12,0x01A0,
    0x6F12,0x89E0,
    0x6F12,0x1A95,
    0x6F12,0x85E2,
    0x6F12,0x01A0,
    0x6F12,0x83E0,
    0x6F12,0x593A,
    0x6F12,0x50E1,
    0x6F12,0x0100,
    0x6F12,0xFFDA,
    0x6F12,0xEAFF,
    0x6F12,0x81E2,
    0x6F12,0x0110,
    0x6F12,0x51E1,
    0x6F12,0x0400,
    0x6F12,0xFFDA,
    0x6F12,0xE5FF,
    0x6F12,0x9DE5,
    0x6F12,0x0400,
    0x6F12,0xA0E3,
    0x6F12,0x021B,
    0x6F12,0x00E0,
    0x6F12,0x9300,
    0x6F12,0x81E0,
    0x6F12,0x4014,
    0x6F12,0x51E3,
    0x6F12,0x020B,
    0x6F12,0xA0A1,
    0x6F12,0x4004,
    0x6F12,0x80A2,
    0x6F12,0x020B,
    0x6F12,0xA0B3,
    0x6F12,0x020B,
    0x6F12,0x9DE5,
    0x6F12,0x9C10,
    0x6F12,0xA0E1,
    0x6F12,0x0008,
    0x6F12,0xA0E1,
    0x6F12,0x4008,
    0x6F12,0x9DE5,
    0x6F12,0x9820,
    0x6F12,0x81E0,
    0x6F12,0x0611,
    0x6F12,0x50E1,
    0x6F12,0x0E00,
    0x6F12,0x82E0,
    0x6F12,0x8110,
    0x6F12,0xA0C1,
    0x6F12,0x00E0,
    0x6F12,0x8CE2,
    0x6F12,0x01C0,
    0x6F12,0x5CE3,
    0x6F12,0x0F00,
    0x6F12,0x86E2,
    0x6F12,0x0160,
    0x6F12,0xC1E1,
    0x6F12,0xB000,
    0x6F12,0xFFBA,
    0x6F12,0xBDFF,
    0x6F12,0x88E2,
    0x6F12,0x0180,
    0x6F12,0x58E3,
    0x6F12,0x0B00,
    0x6F12,0xFFBA,
    0x6F12,0xABFF,
    0x6F12,0x5EE3,
    0x6F12,0x020A,
    0x6F12,0xA0C1,
    0x6F12,0x0E04,
    0x6F12,0xA0C1,
    0x6F12,0xC01F,
    0x6F12,0x80C0,
    0x6F12,0xA109,
    0x6F12,0x9FE5,
    0x6F12,0xA412,
    0x6F12,0x9DE5,
    0x6F12,0x9C20,
    0x6F12,0xA0D3,
    0x6F12,0x010C,
    0x6F12,0xA0C1,
    0x6F12,0xC006,
    0x6F12,0x81E0,
    0x6F12,0x82A0,
    0x6F12,0xCAE1,
    0x6F12,0xB000,
    0x6F12,0xD7E1,
    0x6F12,0xF400,
    0x6F12,0xDAE1,
    0x6F12,0xB010,
    0x6F12,0xA0E3,
    0x6F12,0x0050,
    0x6F12,0xA0E1,
    0x6F12,0x0004,
    0x6F12,0x00EB,
    0x6F12,0xC900,
    0x6F12,0xA0E1,
    0x6F12,0x0008,
    0x6F12,0xA0E1,
    0x6F12,0x4008,
    0x6F12,0xC7E1,
    0x6F12,0xB400,
    0x6F12,0x8DE5,
    0x6F12,0x4C00,
    0x6F12,0x9DE5,
    0x6F12,0x4800,
    0x6F12,0xA0E3,
    0x6F12,0x0060,
    0x6F12,0x40E2,
    0x6F12,0x020B,
    0x6F12,0x8DE5,
    0x6F12,0x9000,
    0x6F12,0x8DE2,
    0x6F12,0x0800,
    0x6F12,0x80E0,
    0x6F12,0x8600,
    0x6F12,0xD0E1,
    0x6F12,0xF000,
    0x6F12,0xD7E1,
    0x6F12,0xF210,
    0x6F12,0xA0E3,
    0x6F12,0x0040,
    0x6F12,0x40E0,
    0x6F12,0x0100,
    0x6F12,0x09E0,
    0x6F12,0x9000,
    0x6F12,0x8DE2,
    0x6F12,0x2000,
    0x6F12,0x80E0,
    0x6F12,0x8400,
    0x6F12,0xD0E1,
    0x6F12,0xF000,
    0x6F12,0xD7E1,
    0x6F12,0xF010,
    0x6F12,0x9DE5,
    0x6F12,0x4C20,
    0x6F12,0x9DE5,
    0x6F12,0x9030,
    0x6F12,0x40E0,
    0x6F12,0x0100,
    0x6F12,0x01E0,
    0x6F12,0x9000,
    0x6F12,0x81E0,
    0x6F12,0x0900,
    0x6F12,0x9DE5,
    0x6F12,0x4010,
    0x6F12,0x80E2,
    0x6F12,0x010C,
    0x6F12,0xA0E1,
    0x6F12,0xA004,
    0x6F12,0x21E0,
    0x6F12,0x9210,
    0x6F12,0x9DE5,
    0x6F12,0x4420,
    0x6F12,0xA0E1,
    0x6F12,0x5112,
    0x6F12,0x9DE5,
    0x6F12,0x4020,
    0x6F12,0x20E0,
    0x6F12,0x9320,
    0x6F12,0x9DE5,
    0x6F12,0x4420,
    0x6F12,0xA0E1,
    0x6F12,0x5002,
    0x6F12,0x80E2,
    0x6F12,0x020B,
    0x6F12,0x00E0,
    0x6F12,0x9100,
    0x6F12,0x80E2,
    0x6F12,0x010B,
    0x6F12,0xA0E1,
    0x6F12,0xC0B5,
    0x6F12,0x9DE5,
    0x6F12,0x9C00,
    0x6F12,0x9DE5,
    0x6F12,0x9820,
    0x6F12,0x80E0,
    0x6F12,0x0501,
    0x6F12,0x82E0,
    0x6F12,0x8080,
    0x6F12,0xD8E1,
    0x6F12,0xF000,
    0x6F12,0xDAE1,
    0x6F12,0xB010,
    0x6F12,0xA0E1,
    0x6F12,0x0004,
    0x6F12,0x00EB,
    0x6F12,0x9B00,
    0x6F12,0x40E0,
    0x6F12,0x0B00,
    0x6F12,0x84E2,
    0x6F12,0x0140,
    0x6F12,0x54E3,
    0x6F12,0x0F00,
    0x6F12,0x85E2,
    0x6F12,0x0150,
    0x6F12,0xC8E1,
    0x6F12,0xB000,
    0x6F12,0xFFBA,
    0x6F12,0xDAFF,
    0x6F12,0x86E2,
    0x6F12,0x0160,
    0x6F12,0x56E3,
    0x6F12,0x0B00,
    0x6F12,0xFFBA,
    0x6F12,0xD0FF,
    0x6F12,0x8DE2,
    0x6F12,0xA4D0,
    0x6F12,0xBDE8,
    0x6F12,0xF04F,
    0x6F12,0x2FE1,
    0x6F12,0x1EFF,
    0x6F12,0x2DE9,
    0x6F12,0xF041,
    0x6F12,0x00EB,
    0x6F12,0x8F00,
    0x6F12,0x50E3,
    0x6F12,0x0000,
    0x6F12,0xBD08,
    0x6F12,0xF041,
    0x6F12,0xA003,
    0x6F12,0x0010,
    0x6F12,0xA003,
    0x6F12,0x3800,
    0x6F12,0x000A,
    0x6F12,0x8C00,
    0x6F12,0x9FE5,
    0x6F12,0x8811,
    0x6F12,0xD1E1,
    0x6F12,0xBA01,
    0x6F12,0xD1E1,
    0x6F12,0xBC21,
    0x6F12,0xD1E1,
    0x6F12,0xBE11,
    0x6F12,0x80E1,
    0x6F12,0x0208,
    0x6F12,0x00EB,
    0x6F12,0x8800,
    0x6F12,0xA0E1,
    0x6F12,0x0070,
    0x6F12,0x9FE5,
    0x6F12,0x5051,
    0x6F12,0x9FE5,
    0x6F12,0x6C01,
    0x6F12,0xD5E1,
    0x6F12,0xF030,
    0x6F12,0xD0E1,
    0x6F12,0xBAEA,
    0x6F12,0xD0E1,
    0x6F12,0xBCCA,
    0x6F12,0xD5E1,
    0x6F12,0xF220,
    0x6F12,0x00E0,
    0x6F12,0x930C,
    0x6F12,0x42E0,
    0x6F12,0x0360,
    0x6F12,0x02E0,
    0x6F12,0x9E02,
    0x6F12,0x4CE0,
    0x6F12,0x0E40,
    0x6F12,0xA0E1,
    0x6F12,0x0410,
    0x6F12,0x40E0,
    0x6F12,0x0200,
    0x6F12,0x00EB,
    0x6F12,0x7400,
    0x6F12,0xA0E1,
    0x6F12,0x0080,
    0x6F12,0x9FE5,
    0x6F12,0x3C01,
    0x6F12,0xD0E1,
    0x6F12,0xB000,
    0x6F12,0x10E3,
    0x6F12,0x020C,
    0x6F12,0xA011,
    0x6F12,0x0700,
    0x6F12,0x001B,
    0x6F12,0x7600,
    0x6F12,0x56E3,
    0x6F12,0x0000,
    0x6F12,0xE003,
    0x6F12,0x0000,
    0x6F12,0x000A,
    0x6F12,0x0300,
    0x6F12,0x47E0,
    0x6F12,0x0800,
    0x6F12,0x00E0,
    0x6F12,0x9400,
    0x6F12,0xA0E1,
    0x6F12,0x0610,
    0x6F12,0x00EB,
    0x6F12,0x6D00,
    0x6F12,0xC5E1,
    0x6F12,0xB400,
    0x6F12,0xBDE8,
    0x6F12,0xF041,
    0x6F12,0x2FE1,
    0x6F12,0x1EFF,
    0x6F12,0x9FE5,
    0x6F12,0x0411,
    0x6F12,0x9FE5,
    0x6F12,0x0401,
    0x6F12,0x2DE9,
    0x6F12,0x1040,
    0x6F12,0x9FE5,
    0x6F12,0x0021,
    0x6F12,0x80E5,
    0x6F12,0x5010,
    0x6F12,0x42E0,
    0x6F12,0x0110,
    0x6F12,0xC0E1,
    0x6F12,0xB415,
    0x6F12,0x9FE5,
    0x6F12,0xF400,
    0x6F12,0x4FE2,
    0x6F12,0xD410,
    0x6F12,0x00EB,
    0x6F12,0x6400,
    0x6F12,0x9FE5,
    0x6F12,0xEC00,
    0x6F12,0x9FE5,
    0x6F12,0xEC40,
    0x6F12,0x9FE5,
    0x6F12,0xB420,
    0x6F12,0x84E5,
    0x6F12,0x0400,
    0x6F12,0xA0E3,
    0x6F12,0x0000,
    0x6F12,0xA0E3,
    0x6F12,0x011C,
    0x6F12,0x82E0,
    0x6F12,0x8030,
    0x6F12,0x80E2,
    0x6F12,0x0100,
    0x6F12,0x50E3,
    0x6F12,0x0400,
    0x6F12,0xC3E1,
    0x6F12,0xB010,
    0x6F12,0xFF3A,
    0x6F12,0xFAFF,
    0x6F12,0x9FE5,
    0x6F12,0xC800,
    0x6F12,0x9FE5,
    0x6F12,0xCC10,
    0x6F12,0x84E5,
    0x6F12,0x5C00,
    0x6F12,0x9FE5,
    0x6F12,0xC000,
    0x6F12,0x84E5,
    0x6F12,0x2C00,
    0x6F12,0x9FE5,
    0x6F12,0xC000,
    0x6F12,0x00EB,
    0x6F12,0x5200,
    0x6F12,0x9FE5,
    0x6F12,0xBC00,
    0x6F12,0x9FE5,
    0x6F12,0xBC10,
    0x6F12,0x84E5,
    0x6F12,0x0000,
    0x6F12,0x9FE5,
    0x6F12,0xB800,
    0x6F12,0x00EB,
    0x6F12,0x4D00,
    0x6F12,0x9FE5,
    0x6F12,0x4440,
    0x6F12,0x9FE5,
    0x6F12,0xB010,
    0x6F12,0xC4E1,
    0x6F12,0xB000,
    0x6F12,0x9FE5,
    0x6F12,0xAC00,
    0x6F12,0x00EB,
    0x6F12,0x4800,
    0x6F12,0xC4E1,
    0x6F12,0xB200,
    0x6F12,0x9FE5,
    0x6F12,0x6400,
    0x6F12,0xD0E1,
    0x6F12,0xB012,
    0x6F12,0x51E3,
    0x6F12,0x1000,
    0x6F12,0x009A,
    0x6F12,0x0200,
    0x6F12,0xA0E3,
    0x6F12,0x090C,
    0x6F12,0x00EB,
    0x6F12,0x3B00,
    0x6F12,0xFFEA,
    0x6F12,0xFEFF,
    0x6F12,0xBDE8,
    0x6F12,0x1040,
    0x6F12,0x2FE1,
    0x6F12,0x1EFF,
    0x6F12,0x0070,
    0x6F12,0x0040,
    0x6F12,0x0070,
    0x6F12,0xD81F,
    0x6F12,0x00D0,
    0x6F12,0x0061,
    0x6F12,0x0070,
    0x6F12,0x0400,
    0x6F12,0x0070,
    0x6F12,0xCC1F,
    0x6F12,0x0070,
    0x6F12,0x5014,
    0x6F12,0x0070,
    0x6F12,0x0000,
    0x6F12,0x00D0,
    0x6F12,0x00F4,
    0x6F12,0x0070,
    0x6F12,0x7004,
    0x6F12,0x0070,
    0x6F12,0x8012,
    0x6F12,0x0070,
    0x6F12,0xD005,
    0x6F12,0x0070,
    0x6F12,0xD01F,
    0x6F12,0x0070,
    0x6F12,0x1013,
    0x6F12,0x0070,
    0x6F12,0xB412,
    0x6F12,0x0070,
    0x6F12,0x941F,
    0x6F12,0x0070,
    0x6F12,0xB41F,
    0x6F12,0x00D0,
    0x6F12,0x0093,
    0x6F12,0x0070,
    0x6F12,0xC00B,
    0x6F12,0x0070,
    0x6F12,0xE012,
    0x6F12,0x0070,
    0x6F12,0xE81F,
    0x6F12,0x0070,
    0x6F12,0x7005,
    0x6F12,0x0070,
    0x6F12,0x902D,
    0x6F12,0x0000,
    0x6F12,0x90A6,
    0x6F12,0x0070,
    0x6F12,0xDC19,
    0x6F12,0x0070,
    0x6F12,0xF804,
    0x6F12,0x0070,
    0x6F12,0x7819,
    0x6F12,0x0070,
    0x6F12,0xC019,
    0x6F12,0x0070,
    0x6F12,0x5019,
    0x6F12,0x0000,
    0x6F12,0xC06A,
    0x6F12,0x0070,
    0x6F12,0xA418,
    0x6F12,0x0070,
    0x6F12,0x2418,
    0x6F12,0x0000,
    0x6F12,0x781C,
    0x6F12,0x0070,
    0x6F12,0x6017,
    0x6F12,0x0000,
    0x6F12,0xF004,
    0x6F12,0x7847,
    0x6F12,0xC046,
    0x6F12,0xFFEA,
    0x6F12,0xABFF,
    0x6F12,0x7847,
    0x6F12,0xC046,
    0x6F12,0x1FE5,
    0x6F12,0x04F0,
    0x6F12,0x0000,
    0x6F12,0x6CCE,
    0x6F12,0x1FE5,
    0x6F12,0x04F0,
    0x6F12,0x0000,
    0x6F12,0xF004,
    0x6F12,0x1FE5,
    0x6F12,0x04F0,
    0x6F12,0x0000,
    0x6F12,0x781C,
    0x6F12,0x1FE5,
    0x6F12,0x04F0,
    0x6F12,0x0000,
    0x6F12,0x54C0,
    0x6F12,0x1FE5,
    0x6F12,0x04F0,
    0x6F12,0x0000,
    0x6F12,0x8448,
    0x6F12,0x1FE5,
    0x6F12,0x04F0,
    0x6F12,0x0000,
    0x6F12,0x146C,
    0x6F12,0x1FE5,
    0x6F12,0x04F0,
    0x6F12,0x0000,
    0x6F12,0x4C7E,
    0x6F12,0x1FE5,
    0x6F12,0x04F0,
    0x6F12,0x0000,
    0x6F12,0x8CDC,
    0x6F12,0x1FE5,
    0x6F12,0x04F0,
    0x6F12,0x0000,
    0x6F12,0x48DD,
    0x6F12,0x1FE5,
    0x6F12,0x04F0,
    0x6F12,0x0000,
    0x6F12,0x7C55,
    0x6F12,0x1FE5,
    0x6F12,0x04F0,
    0x6F12,0x0000,
    0x6F12,0x744C,
    0x6F12,0x1FE5,
    0x6F12,0x04F0,
    0x6F12,0x0000,
    0x6F12,0xE8DE,
    0x6F12,0x1FE5,
    0x6F12,0x04F0,
    0x6F12,0x0000,
    0x6F12,0x4045,
    0x6F12,0x1FE5,
    0x6F12,0x04F0,
    0x6F12,0x0000,
    0x6F12,0xE8CD,
    0x6F12,0x80F9,
    0x6F12,0x00FA,
    0x6F12,0x00FB,
    0x6F12,0x00FC,
    0x6F12,0x00FD,
    0x6F12,0x00FE,
    0x6F12,0x00FF,
    0x6F12,0x0000,
    0x6F12,0x0001,
    0x6F12,0x0002,
    0x6F12,0x0003,
    0x6F12,0x0004,
    0x6F12,0x0005,
    0x6F12,0x0006,
    0x6F12,0x8006,
    0x6F12,0x0000,
    0x6F12,0x00FB,
    0x6F12,0x00FC,
    0x6F12,0x00FD,
    0x6F12,0x00FE,
    0x6F12,0x00FF,
    0x6F12,0x0000,
    0x6F12,0x0001,
    0x6F12,0x0002,
    0x6F12,0x0003,
    0x6F12,0x0004,
    0x6F12,0x0005,
    0x6F12,0x0000,
    0x6028,0xD000,
    0x38FA,0x0030,
    0x38FC,0x0030,
    0x0086,0x01FF,
    0x012A,0x0060,
    0x012C,0x7077,
    0x012E,0x7777,
    0x32CE,0x0060,
    0x32D0,0x0024,
    0x6218,0xF1D0,
    0x6214,0xF9F0,
    0x6226,0x0001,
    0xB0C0,0x000C,
    0xF400,0x0BBC,
    0xF616,0x0004,
    0x6226,0x0000,
    0x6218,0xF9F0,
    0x3338,0x0264,
    0x0104,0x0004,
    0x0B00,0x011C,
};


#endif


/*************************************************************************
* FUNCTION
*    night_mode
*
* DESCRIPTION
*    This function night mode of sensor.
*
* PARAMETERS
*    bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/


static void sensor_init(void)
{
    LOG_INF("enter\n");
    write_cmos_sensor(0x6010, 0x0001);
    write_cmos_sensor(0x0000, 0x0023);
    write_cmos_sensor(0x0000, 0x3087);
    mdelay(3);
    write_cmos_sensor(0x0A02, 0x0014);
    #if MULTI_WRITE
    s5k3h7yx_table_write_cmos_sensor(
        addr_data_pair_init_s5k3h7yx,
        sizeof(addr_data_pair_init_s5k3h7yx) /
        sizeof(kal_uint16));
    #else
    write_cmos_sensor(0x6028, 0x7000);
    write_cmos_sensor(0x602A, 0x1750);
    write_cmos_sensor(0x6F12, 0x10B5);
    write_cmos_sensor(0x6F12, 0x00F0);
    write_cmos_sensor(0x6F12, 0xE1FB);
    write_cmos_sensor(0x6F12, 0x00F0);
    write_cmos_sensor(0x6F12, 0xE3FB);
    write_cmos_sensor(0x6F12, 0x10BC);
    write_cmos_sensor(0x6F12, 0x08BC);
    write_cmos_sensor(0x6F12, 0x1847);
    write_cmos_sensor(0x6F12, 0x2DE9);
    write_cmos_sensor(0x6F12, 0xF041);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0x2437);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0x2447);
    write_cmos_sensor(0x6F12, 0x83E2);
    write_cmos_sensor(0x6F12, 0x01CC);
    write_cmos_sensor(0x6F12, 0x84E5);
    write_cmos_sensor(0x6F12, 0x0030);
    write_cmos_sensor(0x6F12, 0x84E5);
    write_cmos_sensor(0x6F12, 0x04C0);
    write_cmos_sensor(0x6F12, 0x83E2);
    write_cmos_sensor(0x6F12, 0x02CC);
    write_cmos_sensor(0x6F12, 0x83E2);
    write_cmos_sensor(0x6F12, 0x033C);
    write_cmos_sensor(0x6F12, 0x84E5);
    write_cmos_sensor(0x6F12, 0x0C30);
    write_cmos_sensor(0x6F12, 0x44E2);
    write_cmos_sensor(0x6F12, 0x0C70);
    write_cmos_sensor(0x6F12, 0x84E5);
    write_cmos_sensor(0x6F12, 0x08C0);
    write_cmos_sensor(0x6F12, 0xD7E1);
    write_cmos_sensor(0x6F12, 0xB230);
    write_cmos_sensor(0x6F12, 0xA0E3);
    write_cmos_sensor(0x6F12, 0x0150);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0xFC66);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0x15C3);
    write_cmos_sensor(0x6F12, 0x83E2);
    write_cmos_sensor(0x6F12, 0x0130);
    write_cmos_sensor(0x6F12, 0x8CE1);
    write_cmos_sensor(0x6F12, 0x1533);
    write_cmos_sensor(0x6F12, 0xD6E1);
    write_cmos_sensor(0x6F12, 0xB0C0);
    write_cmos_sensor(0x6F12, 0xCCE1);
    write_cmos_sensor(0x6F12, 0x0330);
    write_cmos_sensor(0x6F12, 0xC6E1);
    write_cmos_sensor(0x6F12, 0xB030);
    write_cmos_sensor(0x6F12, 0x00EB);
    write_cmos_sensor(0x6F12, 0xDD01);
    write_cmos_sensor(0x6F12, 0xD7E1);
    write_cmos_sensor(0x6F12, 0xB200);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0x1510);
    write_cmos_sensor(0x6F12, 0x80E2);
    write_cmos_sensor(0x6F12, 0x0100);
    write_cmos_sensor(0x6F12, 0x81E1);
    write_cmos_sensor(0x6F12, 0x1500);
    write_cmos_sensor(0x6F12, 0xD6E1);
    write_cmos_sensor(0x6F12, 0xB010);
    write_cmos_sensor(0x6F12, 0x80E1);
    write_cmos_sensor(0x6F12, 0x0100);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0xC816);
    write_cmos_sensor(0x6F12, 0xC6E1);
    write_cmos_sensor(0x6F12, 0xB000);
    write_cmos_sensor(0x6F12, 0x91E5);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0xD0E5);
    write_cmos_sensor(0x6F12, 0x4801);
    write_cmos_sensor(0x6F12, 0x50E3);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x000A);
    write_cmos_sensor(0x6F12, 0x0D00);
    write_cmos_sensor(0x6F12, 0x94E5);
    write_cmos_sensor(0x6F12, 0x0020);
    write_cmos_sensor(0x6F12, 0xC2E5);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x91E5);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x94E5);
    write_cmos_sensor(0x6F12, 0x0420);
    write_cmos_sensor(0x6F12, 0xD0E5);
    write_cmos_sensor(0x6F12, 0x4801);
    write_cmos_sensor(0x6F12, 0xC2E5);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x91E5);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x94E5);
    write_cmos_sensor(0x6F12, 0x0820);
    write_cmos_sensor(0x6F12, 0xD0E5);
    write_cmos_sensor(0x6F12, 0x4801);
    write_cmos_sensor(0x6F12, 0xC2E5);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x91E5);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x94E5);
    write_cmos_sensor(0x6F12, 0x0C10);
    write_cmos_sensor(0x6F12, 0xD0E5);
    write_cmos_sensor(0x6F12, 0x4801);
    write_cmos_sensor(0x6F12, 0xC1E5);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0xBDE8);
    write_cmos_sensor(0x6F12, 0xF041);
    write_cmos_sensor(0x6F12, 0x2FE1);
    write_cmos_sensor(0x6F12, 0x1EFF);
    write_cmos_sensor(0x6F12, 0x2DE9);
    write_cmos_sensor(0x6F12, 0x7040);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0x7066);
    write_cmos_sensor(0x6F12, 0xA0E3);
    write_cmos_sensor(0x6F12, 0x0140);
    write_cmos_sensor(0x6F12, 0xD6E1);
    write_cmos_sensor(0x6F12, 0xB010);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0x5C56);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0x1421);
    write_cmos_sensor(0x6F12, 0x81E2);
    write_cmos_sensor(0x6F12, 0x0110);
    write_cmos_sensor(0x6F12, 0x82E1);
    write_cmos_sensor(0x6F12, 0x1411);
    write_cmos_sensor(0x6F12, 0xD5E1);
    write_cmos_sensor(0x6F12, 0xB020);
    write_cmos_sensor(0x6F12, 0xC2E1);
    write_cmos_sensor(0x6F12, 0x0110);
    write_cmos_sensor(0x6F12, 0xC5E1);
    write_cmos_sensor(0x6F12, 0xB010);
    write_cmos_sensor(0x6F12, 0x00EB);
    write_cmos_sensor(0x6F12, 0xB701);
    write_cmos_sensor(0x6F12, 0xD6E1);
    write_cmos_sensor(0x6F12, 0xB000);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0x4C26);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0x1410);
    write_cmos_sensor(0x6F12, 0x80E2);
    write_cmos_sensor(0x6F12, 0x0100);
    write_cmos_sensor(0x6F12, 0x81E1);
    write_cmos_sensor(0x6F12, 0x1400);
    write_cmos_sensor(0x6F12, 0xD5E1);
    write_cmos_sensor(0x6F12, 0xB010);
    write_cmos_sensor(0x6F12, 0x80E1);
    write_cmos_sensor(0x6F12, 0x0100);
    write_cmos_sensor(0x6F12, 0xC5E1);
    write_cmos_sensor(0x6F12, 0xB000);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0x2806);
    write_cmos_sensor(0x6F12, 0xD0E1);
    write_cmos_sensor(0x6F12, 0xB00C);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0xA011);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0x2006);
    write_cmos_sensor(0x6F12, 0x90E5);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0xD0E1);
    write_cmos_sensor(0x6F12, 0xBA39);
    write_cmos_sensor(0x6F12, 0x53E1);
    write_cmos_sensor(0x6F12, 0x0100);
    write_cmos_sensor(0x6F12, 0xD091);
    write_cmos_sensor(0x6F12, 0xBE09);
    write_cmos_sensor(0x6F12, 0xD081);
    write_cmos_sensor(0x6F12, 0xBC09);
    write_cmos_sensor(0x6F12, 0xC2E1);
    write_cmos_sensor(0x6F12, 0xB003);
    write_cmos_sensor(0x6F12, 0xBDE8);
    write_cmos_sensor(0x6F12, 0x7040);
    write_cmos_sensor(0x6F12, 0x2FE1);
    write_cmos_sensor(0x6F12, 0x1EFF);
    write_cmos_sensor(0x6F12, 0x2DE9);
    write_cmos_sensor(0x6F12, 0xF840);
    write_cmos_sensor(0x6F12, 0x10E3);
    write_cmos_sensor(0x6F12, 0x0100);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0x0040);
    write_cmos_sensor(0x6F12, 0x9F15);
    write_cmos_sensor(0x6F12, 0xF805);
    write_cmos_sensor(0x6F12, 0x9015);
    write_cmos_sensor(0x6F12, 0x2400);
    write_cmos_sensor(0x6F12, 0x5013);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x000A);
    write_cmos_sensor(0x6F12, 0x2000);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0xEC05);
    write_cmos_sensor(0x6F12, 0xA0E3);
    write_cmos_sensor(0x6F12, 0x3510);
    write_cmos_sensor(0x6F12, 0xD0E5);
    write_cmos_sensor(0x6F12, 0x5E00);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0xE455);
    write_cmos_sensor(0x6F12, 0x50E3);
    write_cmos_sensor(0x6F12, 0x0100);
    write_cmos_sensor(0x6F12, 0xA093);
    write_cmos_sensor(0x6F12, 0x0100);
    write_cmos_sensor(0x6F12, 0x81E0);
    write_cmos_sensor(0x6F12, 0x8060);
    write_cmos_sensor(0x6F12, 0xA0E3);
    write_cmos_sensor(0x6F12, 0x3610);
    write_cmos_sensor(0x6F12, 0x81E0);
    write_cmos_sensor(0x6F12, 0x8000);
    write_cmos_sensor(0x6F12, 0xD5E5);
    write_cmos_sensor(0x6F12, 0xD710);
    write_cmos_sensor(0x6F12, 0x51E3);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x001A);
    write_cmos_sensor(0x6F12, 0x0400);
    write_cmos_sensor(0x6F12, 0xA0E3);
    write_cmos_sensor(0x6F12, 0x0120);
    write_cmos_sensor(0x6F12, 0x8DE2);
    write_cmos_sensor(0x6F12, 0x0010);
    write_cmos_sensor(0x6F12, 0x00EB);
    write_cmos_sensor(0x6F12, 0x8F01);
    write_cmos_sensor(0x6F12, 0xDDE5);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0xC5E5);
    write_cmos_sensor(0x6F12, 0xD700);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0x9C55);
    write_cmos_sensor(0x6F12, 0x95E5);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0xD0E5);
    write_cmos_sensor(0x6F12, 0x1102);
    write_cmos_sensor(0x6F12, 0x50E3);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x001A);
    write_cmos_sensor(0x6F12, 0x0A00);
    write_cmos_sensor(0x6F12, 0xA0E3);
    write_cmos_sensor(0x6F12, 0x0120);
    write_cmos_sensor(0x6F12, 0x8DE2);
    write_cmos_sensor(0x6F12, 0x0010);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0x0600);
    write_cmos_sensor(0x6F12, 0x00EB);
    write_cmos_sensor(0x6F12, 0x8401);
    write_cmos_sensor(0x6F12, 0xDDE5);
    write_cmos_sensor(0x6F12, 0x0010);
    write_cmos_sensor(0x6F12, 0x95E5);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0xC0E5);
    write_cmos_sensor(0x6F12, 0x1112);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0x010C);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0x6415);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0xA00B);
    write_cmos_sensor(0x6F12, 0xC1E1);
    write_cmos_sensor(0x6F12, 0xBE04);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0x0400);
    write_cmos_sensor(0x6F12, 0xBDE8);
    write_cmos_sensor(0x6F12, 0xF840);
    write_cmos_sensor(0x6F12, 0x00EA);
    write_cmos_sensor(0x6F12, 0x7C01);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0x4C15);
    write_cmos_sensor(0x6F12, 0xA0E3);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0xA0E3);
    write_cmos_sensor(0x6F12, 0x012C);
    write_cmos_sensor(0x6F12, 0x81E0);
    write_cmos_sensor(0x6F12, 0x8030);
    write_cmos_sensor(0x6F12, 0x83E2);
    write_cmos_sensor(0x6F12, 0x013C);
    write_cmos_sensor(0x6F12, 0x80E2);
    write_cmos_sensor(0x6F12, 0x0100);
    write_cmos_sensor(0x6F12, 0x50E3);
    write_cmos_sensor(0x6F12, 0x0400);
    write_cmos_sensor(0x6F12, 0xC3E1);
    write_cmos_sensor(0x6F12, 0xBE28);
    write_cmos_sensor(0x6F12, 0xFFBA);
    write_cmos_sensor(0x6F12, 0xF9FF);
    write_cmos_sensor(0x6F12, 0x2FE1);
    write_cmos_sensor(0x6F12, 0x1EFF);
    write_cmos_sensor(0x6F12, 0x2DE9);
    write_cmos_sensor(0x6F12, 0x7040);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0x20C5);
    write_cmos_sensor(0x6F12, 0xDCE5);
    write_cmos_sensor(0x6F12, 0x1021);
    write_cmos_sensor(0x6F12, 0x52E3);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x001A);
    write_cmos_sensor(0x6F12, 0x0A00);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0x28E5);
    write_cmos_sensor(0x6F12, 0x8CE0);
    write_cmos_sensor(0x6F12, 0x0231);
    write_cmos_sensor(0x6F12, 0x8EE0);
    write_cmos_sensor(0x6F12, 0x8250);
    write_cmos_sensor(0x6F12, 0xD5E1);
    write_cmos_sensor(0x6F12, 0xB050);
    write_cmos_sensor(0x6F12, 0x93E5);
    write_cmos_sensor(0x6F12, 0xD840);
    write_cmos_sensor(0x6F12, 0x82E2);
    write_cmos_sensor(0x6F12, 0x0120);
    write_cmos_sensor(0x6F12, 0x04E0);
    write_cmos_sensor(0x6F12, 0x9504);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0x2444);
    write_cmos_sensor(0x6F12, 0x52E3);
    write_cmos_sensor(0x6F12, 0x0400);
    write_cmos_sensor(0x6F12, 0x83E5);
    write_cmos_sensor(0x6F12, 0xD840);
    write_cmos_sensor(0x6F12, 0xFFBA);
    write_cmos_sensor(0x6F12, 0xF5FF);
    write_cmos_sensor(0x6F12, 0xBDE8);
    write_cmos_sensor(0x6F12, 0x7040);
    write_cmos_sensor(0x6F12, 0x00EA);
    write_cmos_sensor(0x6F12, 0x6201);
    write_cmos_sensor(0x6F12, 0x2DE9);
    write_cmos_sensor(0x6F12, 0x1040);
    write_cmos_sensor(0x6F12, 0x00EB);
    write_cmos_sensor(0x6F12, 0x6201);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0xF004);
    write_cmos_sensor(0x6F12, 0xD0E5);
    write_cmos_sensor(0x6F12, 0x7310);
    write_cmos_sensor(0x6F12, 0xBDE8);
    write_cmos_sensor(0x6F12, 0x1040);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0xE804);
    write_cmos_sensor(0x6F12, 0xFFEA);
    write_cmos_sensor(0x6F12, 0xE6FF);
    write_cmos_sensor(0x6F12, 0x2DE9);
    write_cmos_sensor(0x6F12, 0xFF4F);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0xD094);
    write_cmos_sensor(0x6F12, 0xA0E3);
    write_cmos_sensor(0x6F12, 0x0080);
    write_cmos_sensor(0x6F12, 0xD9E5);
    write_cmos_sensor(0x6F12, 0xD700);
    write_cmos_sensor(0x6F12, 0xD9E5);
    write_cmos_sensor(0x6F12, 0x9C40);
    write_cmos_sensor(0x6F12, 0x4DE2);
    write_cmos_sensor(0x6F12, 0x94D0);
    write_cmos_sensor(0x6F12, 0x80E2);
    write_cmos_sensor(0x6F12, 0x0300);
    write_cmos_sensor(0x6F12, 0x00E2);
    write_cmos_sensor(0x6F12, 0xFF50);
    write_cmos_sensor(0x6F12, 0x48E2);
    write_cmos_sensor(0x6F12, 0x0209);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0x0370);
    write_cmos_sensor(0x6F12, 0xCDE1);
    write_cmos_sensor(0x6F12, 0xB007);
    write_cmos_sensor(0x6F12, 0xCDE1);
    write_cmos_sensor(0x6F12, 0xB005);
    write_cmos_sensor(0x6F12, 0xD7E1);
    write_cmos_sensor(0x6F12, 0xF600);
    write_cmos_sensor(0x6F12, 0x54E3);
    write_cmos_sensor(0x6F12, 0x1000);
    write_cmos_sensor(0x6F12, 0xA0E3);
    write_cmos_sensor(0x6F12, 0x0110);
    write_cmos_sensor(0x6F12, 0x8DE5);
    write_cmos_sensor(0x6F12, 0x4800);
    write_cmos_sensor(0x6F12, 0xD7E5);
    write_cmos_sensor(0x6F12, 0x0800);
    write_cmos_sensor(0x6F12, 0xA023);
    write_cmos_sensor(0x6F12, 0x1040);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0x0860);
    write_cmos_sensor(0x6F12, 0x80E2);
    write_cmos_sensor(0x6F12, 0x0900);
    write_cmos_sensor(0x6F12, 0x8DE5);
    write_cmos_sensor(0x6F12, 0x4400);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0x1100);
    write_cmos_sensor(0x6F12, 0x80E0);
    write_cmos_sensor(0x6F12, 0xA00F);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0xC000);
    write_cmos_sensor(0x6F12, 0x8DE5);
    write_cmos_sensor(0x6F12, 0x4000);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0x8014);
    write_cmos_sensor(0x6F12, 0xA0E3);
    write_cmos_sensor(0x6F12, 0x2020);
    write_cmos_sensor(0x6F12, 0x8DE2);
    write_cmos_sensor(0x6F12, 0x2000);
    write_cmos_sensor(0x6F12, 0x00EB);
    write_cmos_sensor(0x6F12, 0x4201);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0x7414);
    write_cmos_sensor(0x6F12, 0xA0E3);
    write_cmos_sensor(0x6F12, 0x1820);
    write_cmos_sensor(0x6F12, 0x8DE2);
    write_cmos_sensor(0x6F12, 0x0800);
    write_cmos_sensor(0x6F12, 0x00EB);
    write_cmos_sensor(0x6F12, 0x3E01);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0x3424);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0x3804);
    write_cmos_sensor(0x6F12, 0x92E5);
    write_cmos_sensor(0x6F12, 0x0020);
    write_cmos_sensor(0x6F12, 0x90E5);
    write_cmos_sensor(0x6F12, 0x4010);
    write_cmos_sensor(0x6F12, 0xD2E5);
    write_cmos_sensor(0x6F12, 0x5921);
    write_cmos_sensor(0x6F12, 0x9DE5);
    write_cmos_sensor(0x6F12, 0x9C30);
    write_cmos_sensor(0x6F12, 0xD0E1);
    write_cmos_sensor(0x6F12, 0xBE04);
    write_cmos_sensor(0x6F12, 0x82E0);
    write_cmos_sensor(0x6F12, 0x8221);
    write_cmos_sensor(0x6F12, 0x81E0);
    write_cmos_sensor(0x6F12, 0x8210);
    write_cmos_sensor(0x6F12, 0x81E0);
    write_cmos_sensor(0x6F12, 0x8310);
    write_cmos_sensor(0x6F12, 0xD1E1);
    write_cmos_sensor(0x6F12, 0xFA30);
    write_cmos_sensor(0x6F12, 0x60E2);
    write_cmos_sensor(0x6F12, 0x012C);
    write_cmos_sensor(0x6F12, 0xD1E1);
    write_cmos_sensor(0x6F12, 0xF210);
    write_cmos_sensor(0x6F12, 0x02E0);
    write_cmos_sensor(0x6F12, 0x9302);
    write_cmos_sensor(0x6F12, 0x20E0);
    write_cmos_sensor(0x6F12, 0x9120);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0x0004);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0x4008);
    write_cmos_sensor(0x6F12, 0xA0E3);
    write_cmos_sensor(0x6F12, 0x00E0);
    write_cmos_sensor(0x6F12, 0x8DE2);
    write_cmos_sensor(0x6F12, 0x50B0);
    write_cmos_sensor(0x6F12, 0x8DE5);
    write_cmos_sensor(0x6F12, 0x0400);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0x0024);
    write_cmos_sensor(0x6F12, 0x82E0);
    write_cmos_sensor(0x6F12, 0x8800);
    write_cmos_sensor(0x6F12, 0xD0E1);
    write_cmos_sensor(0x6F12, 0xFC3B);
    write_cmos_sensor(0x6F12, 0xA0E3);
    write_cmos_sensor(0x6F12, 0x0100);
    write_cmos_sensor(0x6F12, 0x50E1);
    write_cmos_sensor(0x6F12, 0x0400);
    write_cmos_sensor(0x6F12, 0x8BD0);
    write_cmos_sensor(0x6F12, 0x8010);
    write_cmos_sensor(0x6F12, 0x51D1);
    write_cmos_sensor(0x6F12, 0xF2C0);
    write_cmos_sensor(0x6F12, 0xA0D3);
    write_cmos_sensor(0x6F12, 0x0199);
    write_cmos_sensor(0x6F12, 0x80D2);
    write_cmos_sensor(0x6F12, 0x0100);
    write_cmos_sensor(0x6F12, 0x0CD0);
    write_cmos_sensor(0x6F12, 0x930C);
    write_cmos_sensor(0x6F12, 0x89D0);
    write_cmos_sensor(0x6F12, 0x8CC0);
    write_cmos_sensor(0x6F12, 0xA0D1);
    write_cmos_sensor(0x6F12, 0xCCC7);
    write_cmos_sensor(0x6F12, 0xC1D1);
    write_cmos_sensor(0x6F12, 0xB0C0);
    write_cmos_sensor(0x6F12, 0xFFDA);
    write_cmos_sensor(0x6F12, 0xF5FF);
    write_cmos_sensor(0x6F12, 0xA0E3);
    write_cmos_sensor(0x6F12, 0x00C0);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0xC403);
    write_cmos_sensor(0x6F12, 0xA0E3);
    write_cmos_sensor(0x6F12, 0x0199);
    write_cmos_sensor(0x6F12, 0x80E0);
    write_cmos_sensor(0x6F12, 0x8C00);
    write_cmos_sensor(0x6F12, 0xD0E1);
    write_cmos_sensor(0x6F12, 0xFE39);
    write_cmos_sensor(0x6F12, 0xA0E3);
    write_cmos_sensor(0x6F12, 0x0100);
    write_cmos_sensor(0x6F12, 0x50E1);
    write_cmos_sensor(0x6F12, 0x0400);
    write_cmos_sensor(0x6F12, 0x8DD2);
    write_cmos_sensor(0x6F12, 0x7010);
    write_cmos_sensor(0x6F12, 0x81D0);
    write_cmos_sensor(0x6F12, 0x8010);
    write_cmos_sensor(0x6F12, 0x51D1);
    write_cmos_sensor(0x6F12, 0xF220);
    write_cmos_sensor(0x6F12, 0x80D2);
    write_cmos_sensor(0x6F12, 0x0100);
    write_cmos_sensor(0x6F12, 0x02D0);
    write_cmos_sensor(0x6F12, 0x9302);
    write_cmos_sensor(0x6F12, 0x89D0);
    write_cmos_sensor(0x6F12, 0x8220);
    write_cmos_sensor(0x6F12, 0xA0D1);
    write_cmos_sensor(0x6F12, 0xC227);
    write_cmos_sensor(0x6F12, 0xC1D1);
    write_cmos_sensor(0x6F12, 0xB020);
    write_cmos_sensor(0x6F12, 0xFFDA);
    write_cmos_sensor(0x6F12, 0xF5FF);
    write_cmos_sensor(0x6F12, 0xA0E3);
    write_cmos_sensor(0x6F12, 0x0030);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0x0320);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0x0310);
    write_cmos_sensor(0x6F12, 0x00EA);
    write_cmos_sensor(0x6F12, 0x1700);
    write_cmos_sensor(0x6F12, 0xA0E3);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x00EA);
    write_cmos_sensor(0x6F12, 0x1200);
    write_cmos_sensor(0x6F12, 0x41E0);
    write_cmos_sensor(0x6F12, 0x0090);
    write_cmos_sensor(0x6F12, 0x8DE2);
    write_cmos_sensor(0x6F12, 0x70A0);
    write_cmos_sensor(0x6F12, 0x8AE0);
    write_cmos_sensor(0x6F12, 0x80A0);
    write_cmos_sensor(0x6F12, 0x8BE0);
    write_cmos_sensor(0x6F12, 0x8990);
    write_cmos_sensor(0x6F12, 0xDAE1);
    write_cmos_sensor(0x6F12, 0xF0A0);
    write_cmos_sensor(0x6F12, 0xD9E1);
    write_cmos_sensor(0x6F12, 0xF090);
    write_cmos_sensor(0x6F12, 0x80E2);
    write_cmos_sensor(0x6F12, 0x0100);
    write_cmos_sensor(0x6F12, 0x09E0);
    write_cmos_sensor(0x6F12, 0x9A09);
    write_cmos_sensor(0x6F12, 0x9DE5);
    write_cmos_sensor(0x6F12, 0x94A0);
    write_cmos_sensor(0x6F12, 0x89E2);
    write_cmos_sensor(0x6F12, 0x0199);
    write_cmos_sensor(0x6F12, 0x8AE0);
    write_cmos_sensor(0x6F12, 0x82A0);
    write_cmos_sensor(0x6F12, 0xDAE1);
    write_cmos_sensor(0x6F12, 0xF0A0);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0xC997);
    write_cmos_sensor(0x6F12, 0x82E2);
    write_cmos_sensor(0x6F12, 0x0120);
    write_cmos_sensor(0x6F12, 0x09E0);
    write_cmos_sensor(0x6F12, 0x9A09);
    write_cmos_sensor(0x6F12, 0xA0E3);
    write_cmos_sensor(0x6F12, 0x01A0);
    write_cmos_sensor(0x6F12, 0x89E0);
    write_cmos_sensor(0x6F12, 0x1A95);
    write_cmos_sensor(0x6F12, 0x85E2);
    write_cmos_sensor(0x6F12, 0x01A0);
    write_cmos_sensor(0x6F12, 0x83E0);
    write_cmos_sensor(0x6F12, 0x593A);
    write_cmos_sensor(0x6F12, 0x50E1);
    write_cmos_sensor(0x6F12, 0x0100);
    write_cmos_sensor(0x6F12, 0xFFDA);
    write_cmos_sensor(0x6F12, 0xEAFF);
    write_cmos_sensor(0x6F12, 0x81E2);
    write_cmos_sensor(0x6F12, 0x0110);
    write_cmos_sensor(0x6F12, 0x51E1);
    write_cmos_sensor(0x6F12, 0x0400);
    write_cmos_sensor(0x6F12, 0xFFDA);
    write_cmos_sensor(0x6F12, 0xE5FF);
    write_cmos_sensor(0x6F12, 0x9DE5);
    write_cmos_sensor(0x6F12, 0x0400);
    write_cmos_sensor(0x6F12, 0xA0E3);
    write_cmos_sensor(0x6F12, 0x021B);
    write_cmos_sensor(0x6F12, 0x00E0);
    write_cmos_sensor(0x6F12, 0x9300);
    write_cmos_sensor(0x6F12, 0x81E0);
    write_cmos_sensor(0x6F12, 0x4014);
    write_cmos_sensor(0x6F12, 0x51E3);
    write_cmos_sensor(0x6F12, 0x020B);
    write_cmos_sensor(0x6F12, 0xA0A1);
    write_cmos_sensor(0x6F12, 0x4004);
    write_cmos_sensor(0x6F12, 0x80A2);
    write_cmos_sensor(0x6F12, 0x020B);
    write_cmos_sensor(0x6F12, 0xA0B3);
    write_cmos_sensor(0x6F12, 0x020B);
    write_cmos_sensor(0x6F12, 0x9DE5);
    write_cmos_sensor(0x6F12, 0x9C10);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0x0008);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0x4008);
    write_cmos_sensor(0x6F12, 0x9DE5);
    write_cmos_sensor(0x6F12, 0x9820);
    write_cmos_sensor(0x6F12, 0x81E0);
    write_cmos_sensor(0x6F12, 0x0611);
    write_cmos_sensor(0x6F12, 0x50E1);
    write_cmos_sensor(0x6F12, 0x0E00);
    write_cmos_sensor(0x6F12, 0x82E0);
    write_cmos_sensor(0x6F12, 0x8110);
    write_cmos_sensor(0x6F12, 0xA0C1);
    write_cmos_sensor(0x6F12, 0x00E0);
    write_cmos_sensor(0x6F12, 0x8CE2);
    write_cmos_sensor(0x6F12, 0x01C0);
    write_cmos_sensor(0x6F12, 0x5CE3);
    write_cmos_sensor(0x6F12, 0x0F00);
    write_cmos_sensor(0x6F12, 0x86E2);
    write_cmos_sensor(0x6F12, 0x0160);
    write_cmos_sensor(0x6F12, 0xC1E1);
    write_cmos_sensor(0x6F12, 0xB000);
    write_cmos_sensor(0x6F12, 0xFFBA);
    write_cmos_sensor(0x6F12, 0xBDFF);
    write_cmos_sensor(0x6F12, 0x88E2);
    write_cmos_sensor(0x6F12, 0x0180);
    write_cmos_sensor(0x6F12, 0x58E3);
    write_cmos_sensor(0x6F12, 0x0B00);
    write_cmos_sensor(0x6F12, 0xFFBA);
    write_cmos_sensor(0x6F12, 0xABFF);
    write_cmos_sensor(0x6F12, 0x5EE3);
    write_cmos_sensor(0x6F12, 0x020A);
    write_cmos_sensor(0x6F12, 0xA0C1);
    write_cmos_sensor(0x6F12, 0x0E04);
    write_cmos_sensor(0x6F12, 0xA0C1);
    write_cmos_sensor(0x6F12, 0xC01F);
    write_cmos_sensor(0x6F12, 0x80C0);
    write_cmos_sensor(0x6F12, 0xA109);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0xA412);
    write_cmos_sensor(0x6F12, 0x9DE5);
    write_cmos_sensor(0x6F12, 0x9C20);
    write_cmos_sensor(0x6F12, 0xA0D3);
    write_cmos_sensor(0x6F12, 0x010C);
    write_cmos_sensor(0x6F12, 0xA0C1);
    write_cmos_sensor(0x6F12, 0xC006);
    write_cmos_sensor(0x6F12, 0x81E0);
    write_cmos_sensor(0x6F12, 0x82A0);
    write_cmos_sensor(0x6F12, 0xCAE1);
    write_cmos_sensor(0x6F12, 0xB000);
    write_cmos_sensor(0x6F12, 0xD7E1);
    write_cmos_sensor(0x6F12, 0xF400);
    write_cmos_sensor(0x6F12, 0xDAE1);
    write_cmos_sensor(0x6F12, 0xB010);
    write_cmos_sensor(0x6F12, 0xA0E3);
    write_cmos_sensor(0x6F12, 0x0050);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0x0004);
    write_cmos_sensor(0x6F12, 0x00EB);
    write_cmos_sensor(0x6F12, 0xC900);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0x0008);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0x4008);
    write_cmos_sensor(0x6F12, 0xC7E1);
    write_cmos_sensor(0x6F12, 0xB400);
    write_cmos_sensor(0x6F12, 0x8DE5);
    write_cmos_sensor(0x6F12, 0x4C00);
    write_cmos_sensor(0x6F12, 0x9DE5);
    write_cmos_sensor(0x6F12, 0x4800);
    write_cmos_sensor(0x6F12, 0xA0E3);
    write_cmos_sensor(0x6F12, 0x0060);
    write_cmos_sensor(0x6F12, 0x40E2);
    write_cmos_sensor(0x6F12, 0x020B);
    write_cmos_sensor(0x6F12, 0x8DE5);
    write_cmos_sensor(0x6F12, 0x9000);
    write_cmos_sensor(0x6F12, 0x8DE2);
    write_cmos_sensor(0x6F12, 0x0800);
    write_cmos_sensor(0x6F12, 0x80E0);
    write_cmos_sensor(0x6F12, 0x8600);
    write_cmos_sensor(0x6F12, 0xD0E1);
    write_cmos_sensor(0x6F12, 0xF000);
    write_cmos_sensor(0x6F12, 0xD7E1);
    write_cmos_sensor(0x6F12, 0xF210);
    write_cmos_sensor(0x6F12, 0xA0E3);
    write_cmos_sensor(0x6F12, 0x0040);
    write_cmos_sensor(0x6F12, 0x40E0);
    write_cmos_sensor(0x6F12, 0x0100);
    write_cmos_sensor(0x6F12, 0x09E0);
    write_cmos_sensor(0x6F12, 0x9000);
    write_cmos_sensor(0x6F12, 0x8DE2);
    write_cmos_sensor(0x6F12, 0x2000);
    write_cmos_sensor(0x6F12, 0x80E0);
    write_cmos_sensor(0x6F12, 0x8400);
    write_cmos_sensor(0x6F12, 0xD0E1);
    write_cmos_sensor(0x6F12, 0xF000);
    write_cmos_sensor(0x6F12, 0xD7E1);
    write_cmos_sensor(0x6F12, 0xF010);
    write_cmos_sensor(0x6F12, 0x9DE5);
    write_cmos_sensor(0x6F12, 0x4C20);
    write_cmos_sensor(0x6F12, 0x9DE5);
    write_cmos_sensor(0x6F12, 0x9030);
    write_cmos_sensor(0x6F12, 0x40E0);
    write_cmos_sensor(0x6F12, 0x0100);
    write_cmos_sensor(0x6F12, 0x01E0);
    write_cmos_sensor(0x6F12, 0x9000);
    write_cmos_sensor(0x6F12, 0x81E0);
    write_cmos_sensor(0x6F12, 0x0900);
    write_cmos_sensor(0x6F12, 0x9DE5);
    write_cmos_sensor(0x6F12, 0x4010);
    write_cmos_sensor(0x6F12, 0x80E2);
    write_cmos_sensor(0x6F12, 0x010C);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0xA004);
    write_cmos_sensor(0x6F12, 0x21E0);
    write_cmos_sensor(0x6F12, 0x9210);
    write_cmos_sensor(0x6F12, 0x9DE5);
    write_cmos_sensor(0x6F12, 0x4420);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0x5112);
    write_cmos_sensor(0x6F12, 0x9DE5);
    write_cmos_sensor(0x6F12, 0x4020);
    write_cmos_sensor(0x6F12, 0x20E0);
    write_cmos_sensor(0x6F12, 0x9320);
    write_cmos_sensor(0x6F12, 0x9DE5);
    write_cmos_sensor(0x6F12, 0x4420);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0x5002);
    write_cmos_sensor(0x6F12, 0x80E2);
    write_cmos_sensor(0x6F12, 0x020B);
    write_cmos_sensor(0x6F12, 0x00E0);
    write_cmos_sensor(0x6F12, 0x9100);
    write_cmos_sensor(0x6F12, 0x80E2);
    write_cmos_sensor(0x6F12, 0x010B);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0xC0B5);
    write_cmos_sensor(0x6F12, 0x9DE5);
    write_cmos_sensor(0x6F12, 0x9C00);
    write_cmos_sensor(0x6F12, 0x9DE5);
    write_cmos_sensor(0x6F12, 0x9820);
    write_cmos_sensor(0x6F12, 0x80E0);
    write_cmos_sensor(0x6F12, 0x0501);
    write_cmos_sensor(0x6F12, 0x82E0);
    write_cmos_sensor(0x6F12, 0x8080);
    write_cmos_sensor(0x6F12, 0xD8E1);
    write_cmos_sensor(0x6F12, 0xF000);
    write_cmos_sensor(0x6F12, 0xDAE1);
    write_cmos_sensor(0x6F12, 0xB010);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0x0004);
    write_cmos_sensor(0x6F12, 0x00EB);
    write_cmos_sensor(0x6F12, 0x9B00);
    write_cmos_sensor(0x6F12, 0x40E0);
    write_cmos_sensor(0x6F12, 0x0B00);
    write_cmos_sensor(0x6F12, 0x84E2);
    write_cmos_sensor(0x6F12, 0x0140);
    write_cmos_sensor(0x6F12, 0x54E3);
    write_cmos_sensor(0x6F12, 0x0F00);
    write_cmos_sensor(0x6F12, 0x85E2);
    write_cmos_sensor(0x6F12, 0x0150);
    write_cmos_sensor(0x6F12, 0xC8E1);
    write_cmos_sensor(0x6F12, 0xB000);
    write_cmos_sensor(0x6F12, 0xFFBA);
    write_cmos_sensor(0x6F12, 0xDAFF);
    write_cmos_sensor(0x6F12, 0x86E2);
    write_cmos_sensor(0x6F12, 0x0160);
    write_cmos_sensor(0x6F12, 0x56E3);
    write_cmos_sensor(0x6F12, 0x0B00);
    write_cmos_sensor(0x6F12, 0xFFBA);
    write_cmos_sensor(0x6F12, 0xD0FF);
    write_cmos_sensor(0x6F12, 0x8DE2);
    write_cmos_sensor(0x6F12, 0xA4D0);
    write_cmos_sensor(0x6F12, 0xBDE8);
    write_cmos_sensor(0x6F12, 0xF04F);
    write_cmos_sensor(0x6F12, 0x2FE1);
    write_cmos_sensor(0x6F12, 0x1EFF);
    write_cmos_sensor(0x6F12, 0x2DE9);
    write_cmos_sensor(0x6F12, 0xF041);
    write_cmos_sensor(0x6F12, 0x00EB);
    write_cmos_sensor(0x6F12, 0x8F00);
    write_cmos_sensor(0x6F12, 0x50E3);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0xBD08);
    write_cmos_sensor(0x6F12, 0xF041);
    write_cmos_sensor(0x6F12, 0xA003);
    write_cmos_sensor(0x6F12, 0x0010);
    write_cmos_sensor(0x6F12, 0xA003);
    write_cmos_sensor(0x6F12, 0x3800);
    write_cmos_sensor(0x6F12, 0x000A);
    write_cmos_sensor(0x6F12, 0x8C00);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0x8811);
    write_cmos_sensor(0x6F12, 0xD1E1);
    write_cmos_sensor(0x6F12, 0xBA01);
    write_cmos_sensor(0x6F12, 0xD1E1);
    write_cmos_sensor(0x6F12, 0xBC21);
    write_cmos_sensor(0x6F12, 0xD1E1);
    write_cmos_sensor(0x6F12, 0xBE11);
    write_cmos_sensor(0x6F12, 0x80E1);
    write_cmos_sensor(0x6F12, 0x0208);
    write_cmos_sensor(0x6F12, 0x00EB);
    write_cmos_sensor(0x6F12, 0x8800);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0x0070);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0x5051);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0x6C01);
    write_cmos_sensor(0x6F12, 0xD5E1);
    write_cmos_sensor(0x6F12, 0xF030);
    write_cmos_sensor(0x6F12, 0xD0E1);
    write_cmos_sensor(0x6F12, 0xBAEA);
    write_cmos_sensor(0x6F12, 0xD0E1);
    write_cmos_sensor(0x6F12, 0xBCCA);
    write_cmos_sensor(0x6F12, 0xD5E1);
    write_cmos_sensor(0x6F12, 0xF220);
    write_cmos_sensor(0x6F12, 0x00E0);
    write_cmos_sensor(0x6F12, 0x930C);
    write_cmos_sensor(0x6F12, 0x42E0);
    write_cmos_sensor(0x6F12, 0x0360);
    write_cmos_sensor(0x6F12, 0x02E0);
    write_cmos_sensor(0x6F12, 0x9E02);
    write_cmos_sensor(0x6F12, 0x4CE0);
    write_cmos_sensor(0x6F12, 0x0E40);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0x0410);
    write_cmos_sensor(0x6F12, 0x40E0);
    write_cmos_sensor(0x6F12, 0x0200);
    write_cmos_sensor(0x6F12, 0x00EB);
    write_cmos_sensor(0x6F12, 0x7400);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0x0080);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0x3C01);
    write_cmos_sensor(0x6F12, 0xD0E1);
    write_cmos_sensor(0x6F12, 0xB000);
    write_cmos_sensor(0x6F12, 0x10E3);
    write_cmos_sensor(0x6F12, 0x020C);
    write_cmos_sensor(0x6F12, 0xA011);
    write_cmos_sensor(0x6F12, 0x0700);
    write_cmos_sensor(0x6F12, 0x001B);
    write_cmos_sensor(0x6F12, 0x7600);
    write_cmos_sensor(0x6F12, 0x56E3);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0xE003);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x000A);
    write_cmos_sensor(0x6F12, 0x0300);
    write_cmos_sensor(0x6F12, 0x47E0);
    write_cmos_sensor(0x6F12, 0x0800);
    write_cmos_sensor(0x6F12, 0x00E0);
    write_cmos_sensor(0x6F12, 0x9400);
    write_cmos_sensor(0x6F12, 0xA0E1);
    write_cmos_sensor(0x6F12, 0x0610);
    write_cmos_sensor(0x6F12, 0x00EB);
    write_cmos_sensor(0x6F12, 0x6D00);
    write_cmos_sensor(0x6F12, 0xC5E1);
    write_cmos_sensor(0x6F12, 0xB400);
    write_cmos_sensor(0x6F12, 0xBDE8);
    write_cmos_sensor(0x6F12, 0xF041);
    write_cmos_sensor(0x6F12, 0x2FE1);
    write_cmos_sensor(0x6F12, 0x1EFF);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0x0411);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0x0401);
    write_cmos_sensor(0x6F12, 0x2DE9);
    write_cmos_sensor(0x6F12, 0x1040);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0x0021);
    write_cmos_sensor(0x6F12, 0x80E5);
    write_cmos_sensor(0x6F12, 0x5010);
    write_cmos_sensor(0x6F12, 0x42E0);
    write_cmos_sensor(0x6F12, 0x0110);
    write_cmos_sensor(0x6F12, 0xC0E1);
    write_cmos_sensor(0x6F12, 0xB415);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0xF400);
    write_cmos_sensor(0x6F12, 0x4FE2);
    write_cmos_sensor(0x6F12, 0xD410);
    write_cmos_sensor(0x6F12, 0x00EB);
    write_cmos_sensor(0x6F12, 0x6400);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0xEC00);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0xEC40);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0xB420);
    write_cmos_sensor(0x6F12, 0x84E5);
    write_cmos_sensor(0x6F12, 0x0400);
    write_cmos_sensor(0x6F12, 0xA0E3);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0xA0E3);
    write_cmos_sensor(0x6F12, 0x011C);
    write_cmos_sensor(0x6F12, 0x82E0);
    write_cmos_sensor(0x6F12, 0x8030);
    write_cmos_sensor(0x6F12, 0x80E2);
    write_cmos_sensor(0x6F12, 0x0100);
    write_cmos_sensor(0x6F12, 0x50E3);
    write_cmos_sensor(0x6F12, 0x0400);
    write_cmos_sensor(0x6F12, 0xC3E1);
    write_cmos_sensor(0x6F12, 0xB010);
    write_cmos_sensor(0x6F12, 0xFF3A);
    write_cmos_sensor(0x6F12, 0xFAFF);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0xC800);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0xCC10);
    write_cmos_sensor(0x6F12, 0x84E5);
    write_cmos_sensor(0x6F12, 0x5C00);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0xC000);
    write_cmos_sensor(0x6F12, 0x84E5);
    write_cmos_sensor(0x6F12, 0x2C00);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0xC000);
    write_cmos_sensor(0x6F12, 0x00EB);
    write_cmos_sensor(0x6F12, 0x5200);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0xBC00);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0xBC10);
    write_cmos_sensor(0x6F12, 0x84E5);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0xB800);
    write_cmos_sensor(0x6F12, 0x00EB);
    write_cmos_sensor(0x6F12, 0x4D00);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0x4440);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0xB010);
    write_cmos_sensor(0x6F12, 0xC4E1);
    write_cmos_sensor(0x6F12, 0xB000);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0xAC00);
    write_cmos_sensor(0x6F12, 0x00EB);
    write_cmos_sensor(0x6F12, 0x4800);
    write_cmos_sensor(0x6F12, 0xC4E1);
    write_cmos_sensor(0x6F12, 0xB200);
    write_cmos_sensor(0x6F12, 0x9FE5);
    write_cmos_sensor(0x6F12, 0x6400);
    write_cmos_sensor(0x6F12, 0xD0E1);
    write_cmos_sensor(0x6F12, 0xB012);
    write_cmos_sensor(0x6F12, 0x51E3);
    write_cmos_sensor(0x6F12, 0x1000);
    write_cmos_sensor(0x6F12, 0x009A);
    write_cmos_sensor(0x6F12, 0x0200);
    write_cmos_sensor(0x6F12, 0xA0E3);
    write_cmos_sensor(0x6F12, 0x090C);
    write_cmos_sensor(0x6F12, 0x00EB);
    write_cmos_sensor(0x6F12, 0x3B00);
    write_cmos_sensor(0x6F12, 0xFFEA);
    write_cmos_sensor(0x6F12, 0xFEFF);
    write_cmos_sensor(0x6F12, 0xBDE8);
    write_cmos_sensor(0x6F12, 0x1040);
    write_cmos_sensor(0x6F12, 0x2FE1);
    write_cmos_sensor(0x6F12, 0x1EFF);
    write_cmos_sensor(0x6F12, 0x0070);
    write_cmos_sensor(0x6F12, 0x0040);
    write_cmos_sensor(0x6F12, 0x0070);
    write_cmos_sensor(0x6F12, 0xD81F);
    write_cmos_sensor(0x6F12, 0x00D0);
    write_cmos_sensor(0x6F12, 0x0061);
    write_cmos_sensor(0x6F12, 0x0070);
    write_cmos_sensor(0x6F12, 0x0400);
    write_cmos_sensor(0x6F12, 0x0070);
    write_cmos_sensor(0x6F12, 0xCC1F);
    write_cmos_sensor(0x6F12, 0x0070);
    write_cmos_sensor(0x6F12, 0x5014);
    write_cmos_sensor(0x6F12, 0x0070);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x00D0);
    write_cmos_sensor(0x6F12, 0x00F4);
    write_cmos_sensor(0x6F12, 0x0070);
    write_cmos_sensor(0x6F12, 0x7004);
    write_cmos_sensor(0x6F12, 0x0070);
    write_cmos_sensor(0x6F12, 0x8012);
    write_cmos_sensor(0x6F12, 0x0070);
    write_cmos_sensor(0x6F12, 0xD005);
    write_cmos_sensor(0x6F12, 0x0070);
    write_cmos_sensor(0x6F12, 0xD01F);
    write_cmos_sensor(0x6F12, 0x0070);
    write_cmos_sensor(0x6F12, 0x1013);
    write_cmos_sensor(0x6F12, 0x0070);
    write_cmos_sensor(0x6F12, 0xB412);
    write_cmos_sensor(0x6F12, 0x0070);
    write_cmos_sensor(0x6F12, 0x941F);
    write_cmos_sensor(0x6F12, 0x0070);
    write_cmos_sensor(0x6F12, 0xB41F);
    write_cmos_sensor(0x6F12, 0x00D0);
    write_cmos_sensor(0x6F12, 0x0093);
    write_cmos_sensor(0x6F12, 0x0070);
    write_cmos_sensor(0x6F12, 0xC00B);
    write_cmos_sensor(0x6F12, 0x0070);
    write_cmos_sensor(0x6F12, 0xE012);
    write_cmos_sensor(0x6F12, 0x0070);
    write_cmos_sensor(0x6F12, 0xE81F);
    write_cmos_sensor(0x6F12, 0x0070);
    write_cmos_sensor(0x6F12, 0x7005);
    write_cmos_sensor(0x6F12, 0x0070);
    write_cmos_sensor(0x6F12, 0x902D);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x90A6);
    write_cmos_sensor(0x6F12, 0x0070);
    write_cmos_sensor(0x6F12, 0xDC19);
    write_cmos_sensor(0x6F12, 0x0070);
    write_cmos_sensor(0x6F12, 0xF804);
    write_cmos_sensor(0x6F12, 0x0070);
    write_cmos_sensor(0x6F12, 0x7819);
    write_cmos_sensor(0x6F12, 0x0070);
    write_cmos_sensor(0x6F12, 0xC019);
    write_cmos_sensor(0x6F12, 0x0070);
    write_cmos_sensor(0x6F12, 0x5019);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0xC06A);
    write_cmos_sensor(0x6F12, 0x0070);
    write_cmos_sensor(0x6F12, 0xA418);
    write_cmos_sensor(0x6F12, 0x0070);
    write_cmos_sensor(0x6F12, 0x2418);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x781C);
    write_cmos_sensor(0x6F12, 0x0070);
    write_cmos_sensor(0x6F12, 0x6017);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0xF004);
    write_cmos_sensor(0x6F12, 0x7847);
    write_cmos_sensor(0x6F12, 0xC046);
    write_cmos_sensor(0x6F12, 0xFFEA);
    write_cmos_sensor(0x6F12, 0xABFF);
    write_cmos_sensor(0x6F12, 0x7847);
    write_cmos_sensor(0x6F12, 0xC046);
    write_cmos_sensor(0x6F12, 0x1FE5);
    write_cmos_sensor(0x6F12, 0x04F0);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x6CCE);
    write_cmos_sensor(0x6F12, 0x1FE5);
    write_cmos_sensor(0x6F12, 0x04F0);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0xF004);
    write_cmos_sensor(0x6F12, 0x1FE5);
    write_cmos_sensor(0x6F12, 0x04F0);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x781C);
    write_cmos_sensor(0x6F12, 0x1FE5);
    write_cmos_sensor(0x6F12, 0x04F0);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x54C0);
    write_cmos_sensor(0x6F12, 0x1FE5);
    write_cmos_sensor(0x6F12, 0x04F0);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x8448);
    write_cmos_sensor(0x6F12, 0x1FE5);
    write_cmos_sensor(0x6F12, 0x04F0);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x146C);
    write_cmos_sensor(0x6F12, 0x1FE5);
    write_cmos_sensor(0x6F12, 0x04F0);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x4C7E);
    write_cmos_sensor(0x6F12, 0x1FE5);
    write_cmos_sensor(0x6F12, 0x04F0);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x8CDC);
    write_cmos_sensor(0x6F12, 0x1FE5);
    write_cmos_sensor(0x6F12, 0x04F0);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x48DD);
    write_cmos_sensor(0x6F12, 0x1FE5);
    write_cmos_sensor(0x6F12, 0x04F0);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x7C55);
    write_cmos_sensor(0x6F12, 0x1FE5);
    write_cmos_sensor(0x6F12, 0x04F0);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x744C);
    write_cmos_sensor(0x6F12, 0x1FE5);
    write_cmos_sensor(0x6F12, 0x04F0);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0xE8DE);
    write_cmos_sensor(0x6F12, 0x1FE5);
    write_cmos_sensor(0x6F12, 0x04F0);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x4045);
    write_cmos_sensor(0x6F12, 0x1FE5);
    write_cmos_sensor(0x6F12, 0x04F0);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0xE8CD);
    write_cmos_sensor(0x6F12, 0x80F9);
    write_cmos_sensor(0x6F12, 0x00FA);
    write_cmos_sensor(0x6F12, 0x00FB);
    write_cmos_sensor(0x6F12, 0x00FC);
    write_cmos_sensor(0x6F12, 0x00FD);
    write_cmos_sensor(0x6F12, 0x00FE);
    write_cmos_sensor(0x6F12, 0x00FF);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x0001);
    write_cmos_sensor(0x6F12, 0x0002);
    write_cmos_sensor(0x6F12, 0x0003);
    write_cmos_sensor(0x6F12, 0x0004);
    write_cmos_sensor(0x6F12, 0x0005);
    write_cmos_sensor(0x6F12, 0x0006);
    write_cmos_sensor(0x6F12, 0x8006);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x00FB);
    write_cmos_sensor(0x6F12, 0x00FC);
    write_cmos_sensor(0x6F12, 0x00FD);
    write_cmos_sensor(0x6F12, 0x00FE);
    write_cmos_sensor(0x6F12, 0x00FF);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6F12, 0x0001);
    write_cmos_sensor(0x6F12, 0x0002);
    write_cmos_sensor(0x6F12, 0x0003);
    write_cmos_sensor(0x6F12, 0x0004);
    write_cmos_sensor(0x6F12, 0x0005);
    write_cmos_sensor(0x6F12, 0x0000);
    write_cmos_sensor(0x6028, 0xD000);
    write_cmos_sensor(0x38FA, 0x0030);
    write_cmos_sensor(0x38FC, 0x0030);
    write_cmos_sensor(0x0086, 0x01FF);
    write_cmos_sensor(0x012A, 0x0060);
    write_cmos_sensor(0x012C, 0x7077);
    write_cmos_sensor(0x012E, 0x7777);
    write_cmos_sensor(0x32CE, 0x0060);
    write_cmos_sensor(0x32D0, 0x0024);
    write_cmos_sensor(0x6218, 0xF1D0);
    write_cmos_sensor(0x6214, 0xF9F0);
    write_cmos_sensor(0x6226, 0x0001);
    write_cmos_sensor(0xB0C0, 0x000C);
    write_cmos_sensor(0xF400, 0x0BBC);
    write_cmos_sensor(0xF616, 0x0004);
    write_cmos_sensor(0x6226, 0x0000);
    write_cmos_sensor(0x6218, 0xF9F0);
    write_cmos_sensor(0x3338, 0x0264);
    write_cmos_sensor(0x0104, 0x0004);
    write_cmos_sensor(0x0B00, 0x011C);//lsc enable lux ratio 0x1c
    #endif
}    /*    sensor_init  */


static void preview_setting(void)
{
   LOG_INF("preview seting enter\n");

   write_cmos_sensor(0x0136, 0x1800);
   write_cmos_sensor(0x0300, 0x0002);
   write_cmos_sensor(0x0302, 0x0001);
   write_cmos_sensor(0x0304, 0x0006);
   write_cmos_sensor(0x0306, 0x008C);
   write_cmos_sensor(0x0308, 0x0008);
   write_cmos_sensor(0x030A, 0x0001);
   write_cmos_sensor(0x030C, 0x0006);
   write_cmos_sensor(0x030E, 0x00A5);
   write_cmos_sensor(0x311C, 0x0BB8);
   write_cmos_sensor(0x311E, 0x0BB8);
   write_cmos_sensor(0x0342, 0x0E68);
   write_cmos_sensor(0x0340, 0x09E2);
   write_cmos_sensor(0x0200, 0x0618);
   write_cmos_sensor(0x0202, 0x0002);
   write_cmos_sensor(0x0114, 0x0300);
   write_cmos_sensor(0x0204, 0x0020);
   write_cmos_sensor(0x0344, 0x0004);
   write_cmos_sensor(0x0348, 0x0CC3);
   write_cmos_sensor(0x0346, 0x0004);
   write_cmos_sensor(0x034A, 0x0993);
   write_cmos_sensor(0x034C, 0x0660);
   write_cmos_sensor(0x034E, 0x04C8);
   write_cmos_sensor_8(0x3011, 0x01);
   write_cmos_sensor(0x0382, 0x0003);
   write_cmos_sensor(0x0386, 0x0003);
   write_cmos_sensor_8(0x0900, 0x01);
   write_cmos_sensor_8(0x0901, 0x22);
   write_cmos_sensor(0x31FE, 0xC004);
   write_cmos_sensor(0x3200, 0xC4F0);
   write_cmos_sensor(0x3202, 0xCEC8);
   write_cmos_sensor(0x3204, 0xD8A0);
   write_cmos_sensor(0x3206, 0xE278);
   write_cmos_sensor(0x3208, 0xEC50);
   write_cmos_sensor(0x320A, 0xF628);
   write_cmos_sensor(0x320C, 0x0000);
   write_cmos_sensor(0x320E, 0x09D8);
   write_cmos_sensor(0x3210, 0x13B0);
   write_cmos_sensor(0x3212, 0x1D88);
   write_cmos_sensor(0x3214, 0x2760);
   write_cmos_sensor(0x3216, 0x3138);
   write_cmos_sensor(0x3218, 0x3B10);
   write_cmos_sensor(0x321A, 0x3FFC);
   write_cmos_sensor(0x321C, 0xC004);
   write_cmos_sensor(0x321E, 0xCCD0);
   write_cmos_sensor(0x3220, 0xD99C);
   write_cmos_sensor(0x3222, 0xE668);
   write_cmos_sensor(0x3224, 0xF334);
   write_cmos_sensor(0x3226, 0x0000);
   write_cmos_sensor(0x3228, 0x0CCC);
   write_cmos_sensor(0x322A, 0x1998);
   write_cmos_sensor(0x322C, 0x2664);
   write_cmos_sensor(0x322E, 0x3330);
   write_cmos_sensor(0x3230, 0x3FFC);
   write_cmos_sensor(0x3232, 0x0100);
   write_cmos_sensor(0x3234, 0x0100);
   write_cmos_sensor_8(0x3237, 0x00);
   write_cmos_sensor_8(0x3238, 0x09);
   write_cmos_sensor_8(0x3239, 0x09);
   write_cmos_sensor_8(0x323A, 0x0B);
   write_cmos_sensor(0x3160, 0x0600);
   write_cmos_sensor(0x3164, 0x09C4);
   write_cmos_sensor(0x3166, 0x0100);
   write_cmos_sensor(0x3168, 0x0100);
   write_cmos_sensor(0x316A, 0x0100);
   write_cmos_sensor(0x316C, 0x0100);
   write_cmos_sensor(0x316E, 0x001A);
   write_cmos_sensor(0x3170, 0x002F);
   write_cmos_sensor(0x3172, 0x0000);
   write_cmos_sensor(0x3174, 0x001A);
   write_cmos_sensor(0x3176, 0x0A8C);
   write_cmos_sensor(0x3178, 0x0100);
   write_cmos_sensor(0x317A, 0x09C5);
   write_cmos_sensor(0x317C, 0x0100);
   write_cmos_sensor(0x317E, 0x0100);
   write_cmos_sensor(0x3180, 0x001A);
   write_cmos_sensor(0x3182, 0x002F);
   write_cmos_sensor(0x3184, 0x0000);
   write_cmos_sensor(0x3186, 0x001A);
   write_cmos_sensor(0x3188, 0x0CE4);
   write_cmos_sensor(0x318A, 0x0100);
   write_cmos_sensor(0x318C, 0x0100);
   write_cmos_sensor(0x318E, 0x0100);
   write_cmos_sensor(0x3190, 0x0100);
   write_cmos_sensor(0x3192, 0x001A);
   write_cmos_sensor(0x3194, 0x002F);
   write_cmos_sensor(0x3196, 0x0000);
   write_cmos_sensor(0x3198, 0x001A);
   write_cmos_sensor(0x319A, 0x1004);
   write_cmos_sensor(0x319C, 0x0100);
   write_cmos_sensor(0x319E, 0x0100);
   write_cmos_sensor(0x31A0, 0x0100);
   write_cmos_sensor(0x31A2, 0x0100);
   write_cmos_sensor(0x31A4, 0x001A);
   write_cmos_sensor(0x31A6, 0x002F);
   write_cmos_sensor(0x31A8, 0x0000);
   write_cmos_sensor(0x31AA, 0x001A);
   write_cmos_sensor(0x31AC, 0x1388);
   write_cmos_sensor(0x31AE, 0x0100);
   write_cmos_sensor(0x31B0, 0x0100);
   write_cmos_sensor(0x31B2, 0x0100);
   write_cmos_sensor(0x31B4, 0x0100);
   write_cmos_sensor(0x31B6, 0x001A);
   write_cmos_sensor(0x31B8, 0x002F);
   write_cmos_sensor(0x31BA, 0x0000);
   write_cmos_sensor(0x31BC, 0x001A);
   write_cmos_sensor(0x31BE, 0x1964);
   write_cmos_sensor(0x31C0, 0x0100);
   write_cmos_sensor(0x31C2, 0x0100);
   write_cmos_sensor(0x31C4, 0x0100);
   write_cmos_sensor(0x31C6, 0x0100);
   write_cmos_sensor(0x31C8, 0x001A);
   write_cmos_sensor(0x31CA, 0x002F);
   write_cmos_sensor(0x31CC, 0x0000);
   write_cmos_sensor(0x31CE, 0x001A);
   write_cmos_sensor(0x31D0, 0x1D4C);
   write_cmos_sensor(0x31D2, 0x0100);
   write_cmos_sensor(0x31D4, 0x0100);
   write_cmos_sensor(0x31D6, 0x0100);
   write_cmos_sensor(0x31D8, 0x0100);
   write_cmos_sensor(0x31DA, 0x001A);
   write_cmos_sensor(0x31DC, 0x002F);
   write_cmos_sensor(0x31DE, 0x0000);
   write_cmos_sensor(0x31E0, 0x001A);
   write_cmos_sensor_8(0x3162, 0x01);
   write_cmos_sensor(0x301C, 0x0100);
   write_cmos_sensor_8(0x301E, 0x03);
   write_cmos_sensor_8(0x323C, 0x00);
   write_cmos_sensor_8(0x323D, 0x01);
   write_cmos_sensor_8(0x1989, 0x04);

}    /*    preview_setting  */


static void normal_capture_setting(void)
{
    LOG_INF("capture setting enter\n");
    write_cmos_sensor(0x0136, 0x1800);
    write_cmos_sensor(0x0300, 0x0002);
    write_cmos_sensor(0x0302, 0x0001);
    write_cmos_sensor(0x0304, 0x0006);
    write_cmos_sensor(0x0306, 0x008C);
    write_cmos_sensor(0x0308, 0x0008);
    write_cmos_sensor(0x030A, 0x0001);
    write_cmos_sensor(0x030C, 0x0006);
    write_cmos_sensor(0x030E, 0x00A5);
    write_cmos_sensor(0x311C, 0x0BB8);
    write_cmos_sensor(0x311E, 0x0BB8);
    write_cmos_sensor(0x0342, 0x0E68);
    write_cmos_sensor(0x0340, 0x09B5);
    write_cmos_sensor(0x0200, 0x0618);
    write_cmos_sensor(0x0202, 0x0002);
    write_cmos_sensor(0x0114, 0x0300);
    write_cmos_sensor(0x0204, 0x0020);
    write_cmos_sensor(0x0344, 0x0004);
    write_cmos_sensor(0x0348, 0x0CC3);
    write_cmos_sensor(0x0346, 0x0004);
    write_cmos_sensor(0x034A, 0x0993);
    write_cmos_sensor(0x034C, 0x0CC0);
    write_cmos_sensor(0x034E, 0x0990);
    write_cmos_sensor_8(0x3011, 0x01);
    write_cmos_sensor(0x0382, 0x0001);
    write_cmos_sensor(0x0386, 0x0001);
    write_cmos_sensor_8(0x0900, 0x00);
    write_cmos_sensor_8(0x0901, 0x11);
    write_cmos_sensor(0x31FE, 0xC004);
    write_cmos_sensor(0x3200, 0xC4F0);
    write_cmos_sensor(0x3202, 0xCEC8);
    write_cmos_sensor(0x3204, 0xD8A0);
    write_cmos_sensor(0x3206, 0xE278);
    write_cmos_sensor(0x3208, 0xEC50);
    write_cmos_sensor(0x320A, 0xF628);
    write_cmos_sensor(0x320C, 0x0000);
    write_cmos_sensor(0x320E, 0x09D8);
    write_cmos_sensor(0x3210, 0x13B0);
    write_cmos_sensor(0x3212, 0x1D88);
    write_cmos_sensor(0x3214, 0x2760);
    write_cmos_sensor(0x3216, 0x3138);
    write_cmos_sensor(0x3218, 0x3B10);
    write_cmos_sensor(0x321A, 0x3FFC);
    write_cmos_sensor(0x321C, 0xC004);
    write_cmos_sensor(0x321E, 0xCCD0);
    write_cmos_sensor(0x3220, 0xD99C);
    write_cmos_sensor(0x3222, 0xE668);
    write_cmos_sensor(0x3224, 0xF334);
    write_cmos_sensor(0x3226, 0x0000);
    write_cmos_sensor(0x3228, 0x0CCC);
    write_cmos_sensor(0x322A, 0x1998);
    write_cmos_sensor(0x322C, 0x2664);
    write_cmos_sensor(0x322E, 0x3330);
    write_cmos_sensor(0x3230, 0x3FFC);
    write_cmos_sensor(0x3232, 0x0100);
    write_cmos_sensor(0x3234, 0x0100);
    write_cmos_sensor_8(0x3237, 0x00);
    write_cmos_sensor_8(0x3238, 0x09);
    write_cmos_sensor_8(0x3239, 0x09);
    write_cmos_sensor_8(0x323A, 0x0B);
    write_cmos_sensor(0x3160, 0x0600);
    write_cmos_sensor(0x3164, 0x09C4);
    write_cmos_sensor(0x3166, 0x0100);
    write_cmos_sensor(0x3168, 0x0100);
    write_cmos_sensor(0x316A, 0x0100);
    write_cmos_sensor(0x316C, 0x0100);
    write_cmos_sensor(0x316E, 0x001A);
    write_cmos_sensor(0x3170, 0x002F);
    write_cmos_sensor(0x3172, 0x0000);
    write_cmos_sensor(0x3174, 0x001A);
    write_cmos_sensor(0x3176, 0x0A8C);
    write_cmos_sensor(0x3178, 0x0100);
    write_cmos_sensor(0x317A, 0x0100);
    write_cmos_sensor(0x317C, 0x0100);
    write_cmos_sensor(0x317E, 0x0100);
    write_cmos_sensor(0x3180, 0x001A);
    write_cmos_sensor(0x3182, 0x002F);
    write_cmos_sensor(0x3184, 0x0000);
    write_cmos_sensor(0x3186, 0x001A);
    write_cmos_sensor(0x3188, 0x0CE4);
    write_cmos_sensor(0x318A, 0x0100);
    write_cmos_sensor(0x318C, 0x0100);
    write_cmos_sensor(0x318E, 0x0100);
    write_cmos_sensor(0x3190, 0x0100);
    write_cmos_sensor(0x3192, 0x001A);
    write_cmos_sensor(0x3194, 0x002F);
    write_cmos_sensor(0x3196, 0x0000);
    write_cmos_sensor(0x3198, 0x001A);
    write_cmos_sensor(0x319A, 0x1004);
    write_cmos_sensor(0x319C, 0x0100);
    write_cmos_sensor(0x319E, 0x0100);
    write_cmos_sensor(0x31A0, 0x0100);
    write_cmos_sensor(0x31A2, 0x0100);
    write_cmos_sensor(0x31A4, 0x001A);
    write_cmos_sensor(0x31A6, 0x002F);
    write_cmos_sensor(0x31A8, 0x0000);
    write_cmos_sensor(0x31AA, 0x001A);
    write_cmos_sensor(0x31AC, 0x1388);
    write_cmos_sensor(0x31AE, 0x0100);
    write_cmos_sensor(0x31B0, 0x0100);
    write_cmos_sensor(0x31B2, 0x0100);
    write_cmos_sensor(0x31B4, 0x0100);
    write_cmos_sensor(0x31B6, 0x001A);
    write_cmos_sensor(0x31B8, 0x002F);
    write_cmos_sensor(0x31BA, 0x0000);
    write_cmos_sensor(0x31BC, 0x001A);
    write_cmos_sensor(0x31BE, 0x1964);
    write_cmos_sensor(0x31C0, 0x0100);
    write_cmos_sensor(0x31C2, 0x0100);
    write_cmos_sensor(0x31C4, 0x0100);
    write_cmos_sensor(0x31C6, 0x0100);
    write_cmos_sensor(0x31C8, 0x001A);
    write_cmos_sensor(0x31CA, 0x002F);
    write_cmos_sensor(0x31CC, 0x0000);
    write_cmos_sensor(0x31CE, 0x001A);
    write_cmos_sensor(0x31D0, 0x1D4C);
    write_cmos_sensor(0x31D2, 0x0100);
    write_cmos_sensor(0x31D4, 0x0100);
    write_cmos_sensor(0x31D6, 0x0100);
    write_cmos_sensor(0x31D8, 0x0100);
    write_cmos_sensor(0x31DA, 0x001A);
    write_cmos_sensor(0x31DC, 0x002F);
    write_cmos_sensor(0x31DE, 0x0000);
    write_cmos_sensor(0x31E0, 0x001A);
    write_cmos_sensor_8(0x3162, 0x01);
    write_cmos_sensor(0x301C, 0x0100);
    write_cmos_sensor_8(0x301E, 0x03);
    write_cmos_sensor_8(0x323C, 0x00);
    write_cmos_sensor_8(0x323D, 0x01);
    write_cmos_sensor_8(0x1989, 0x04);
}


static void capture_setting(kal_uint16 currefps)
{
    normal_capture_setting();
}

static void normal_video_setting(kal_uint16 currefps)
{
    write_cmos_sensor(0x0136, 0x1800);
    write_cmos_sensor(0x0300, 0x0002);
    write_cmos_sensor(0x0302, 0x0001);
    write_cmos_sensor(0x0304, 0x0006);
    write_cmos_sensor(0x0306, 0x008C);
    write_cmos_sensor(0x0308, 0x0008);
    write_cmos_sensor(0x030A, 0x0001);
    write_cmos_sensor(0x030C, 0x0006);
    write_cmos_sensor(0x030E, 0x00A5);
    write_cmos_sensor(0x311C, 0x0BB8);
    write_cmos_sensor(0x311E, 0x0BB8);
    write_cmos_sensor(0x0342, 0x0E68);
    write_cmos_sensor(0x0340, 0x09E0);
    write_cmos_sensor(0x0200, 0x0618);
    write_cmos_sensor(0x0202, 0x0002);
    write_cmos_sensor(0x0114, 0x0300);
    write_cmos_sensor(0x0204, 0x0020);
    write_cmos_sensor(0x0344, 0x0004);
    write_cmos_sensor(0x0348, 0x0CC3);
    write_cmos_sensor(0x0346, 0x0134);
    write_cmos_sensor(0x034A, 0x0863);
    write_cmos_sensor(0x034C, 0x0CC0);
    write_cmos_sensor(0x034E, 0x0730);
    write_cmos_sensor(0x3010, 0x0001);
    write_cmos_sensor(0x0382, 0x0001);
    write_cmos_sensor(0x0386, 0x0001);
    write_cmos_sensor(0x0900, 0x0011);
    write_cmos_sensor(0x31FE, 0xC004);
    write_cmos_sensor(0x3200, 0xC4F0);
    write_cmos_sensor(0x3202, 0xCEC8);
    write_cmos_sensor(0x3204, 0xD8A0);
    write_cmos_sensor(0x3206, 0xE278);
    write_cmos_sensor(0x3208, 0xEC50);
    write_cmos_sensor(0x320A, 0xF628);
    write_cmos_sensor(0x320C, 0x0000);
    write_cmos_sensor(0x320E, 0x09D8);
    write_cmos_sensor(0x3210, 0x13B0);
    write_cmos_sensor(0x3212, 0x1D88);
    write_cmos_sensor(0x3214, 0x2760);
    write_cmos_sensor(0x3216, 0x3138);
    write_cmos_sensor(0x3218, 0x3B10);
    write_cmos_sensor(0x321A, 0x3FFC);
    write_cmos_sensor(0x321C, 0xC004);
    write_cmos_sensor(0x321E, 0xCCD0);
    write_cmos_sensor(0x3220, 0xD99C);
    write_cmos_sensor(0x3222, 0xE668);
    write_cmos_sensor(0x3224, 0xF334);
    write_cmos_sensor(0x3226, 0x0000);
    write_cmos_sensor(0x3228, 0x0CCC);
    write_cmos_sensor(0x322A, 0x1998);
    write_cmos_sensor(0x322C, 0x2664);
    write_cmos_sensor(0x322E, 0x3330);
    write_cmos_sensor(0x3230, 0x3FFC);
    write_cmos_sensor(0x3232, 0x0100);
    write_cmos_sensor(0x3234, 0x0100);
    write_cmos_sensor(0x3236, 0x0E10);
    write_cmos_sensor(0x3238, 0x0909);
    write_cmos_sensor(0x323A, 0x0B0F);
    write_cmos_sensor(0x3160, 0x0600);
    write_cmos_sensor(0x3164, 0x09C4);
    write_cmos_sensor(0x3166, 0x0100);
    write_cmos_sensor(0x3168, 0x0100);
    write_cmos_sensor(0x316A, 0x0100);
    write_cmos_sensor(0x316C, 0x0100);
    write_cmos_sensor(0x316E, 0x001A);
    write_cmos_sensor(0x3170, 0x002F);
    write_cmos_sensor(0x3172, 0x0000);
    write_cmos_sensor(0x3174, 0x001A);
    write_cmos_sensor(0x3176, 0x0A8C);
    write_cmos_sensor(0x3178, 0x0100);
    write_cmos_sensor(0x317A, 0x0100);
    write_cmos_sensor(0x317C, 0x0100);
    write_cmos_sensor(0x317E, 0x0100);
    write_cmos_sensor(0x3180, 0x001A);
    write_cmos_sensor(0x3182, 0x002F);
    write_cmos_sensor(0x3184, 0x0000);
    write_cmos_sensor(0x3186, 0x001A);
    write_cmos_sensor(0x3188, 0x0CE4);
    write_cmos_sensor(0x318A, 0x0100);
    write_cmos_sensor(0x318C, 0x0100);
    write_cmos_sensor(0x318E, 0x0100);
    write_cmos_sensor(0x3190, 0x0100);
    write_cmos_sensor(0x3192, 0x001A);
    write_cmos_sensor(0x3194, 0x002F);
    write_cmos_sensor(0x3196, 0x0000);
    write_cmos_sensor(0x3198, 0x001A);
    write_cmos_sensor(0x319A, 0x1004);
    write_cmos_sensor(0x319C, 0x0100);
    write_cmos_sensor(0x319E, 0x0100);
    write_cmos_sensor(0x31A0, 0x0100);
    write_cmos_sensor(0x31A2, 0x0100);
    write_cmos_sensor(0x31A4, 0x001A);
    write_cmos_sensor(0x31A6, 0x002F);
    write_cmos_sensor(0x31A8, 0x0000);
    write_cmos_sensor(0x31AA, 0x001A);
    write_cmos_sensor(0x31AC, 0x1388);
    write_cmos_sensor(0x31AE, 0x0100);
    write_cmos_sensor(0x31B0, 0x0100);
    write_cmos_sensor(0x31B2, 0x0100);
    write_cmos_sensor(0x31B4, 0x0100);
    write_cmos_sensor(0x31B6, 0x001A);
    write_cmos_sensor(0x31B8, 0x002F);
    write_cmos_sensor(0x31BA, 0x0000);
    write_cmos_sensor(0x31BC, 0x001A);
    write_cmos_sensor(0x31BE, 0x1964);
    write_cmos_sensor(0x31C0, 0x0100);
    write_cmos_sensor(0x31C2, 0x0100);
    write_cmos_sensor(0x31C4, 0x0100);
    write_cmos_sensor(0x31C6, 0x0100);
    write_cmos_sensor(0x31C8, 0x001A);
    write_cmos_sensor(0x31CA, 0x002F);
    write_cmos_sensor(0x31CC, 0x0000);
    write_cmos_sensor(0x31CE, 0x001A);
    write_cmos_sensor(0x31D0, 0x1D4C);
    write_cmos_sensor(0x31D2, 0x0100);
    write_cmos_sensor(0x31D4, 0x0100);
    write_cmos_sensor(0x31D6, 0x0100);
    write_cmos_sensor(0x31D8, 0x0100);
    write_cmos_sensor(0x31DA, 0x001A);
    write_cmos_sensor(0x31DC, 0x002F);
    write_cmos_sensor(0x31DE, 0x0000);
    write_cmos_sensor(0x31E0, 0x001A);
    write_cmos_sensor(0x3162, 0x0100);
    write_cmos_sensor(0x301C, 0x0100);
    write_cmos_sensor(0x301E, 0x0300);
    write_cmos_sensor(0x323C, 0x0001);
    write_cmos_sensor(0x1988, 0x0004);
}

static void hs_video_setting(void)
{
    LOG_INF("enter hs_video setting\n");

    write_cmos_sensor(0x0136, 0x1800);
    write_cmos_sensor(0x0300, 0x0002);
    write_cmos_sensor(0x0302, 0x0001);
    write_cmos_sensor(0x0304, 0x0006);
    write_cmos_sensor(0x0306, 0x008C);
    write_cmos_sensor(0x0308, 0x0008);
    write_cmos_sensor(0x030A, 0x0001);
    write_cmos_sensor(0x030C, 0x0006);
    write_cmos_sensor(0x030E, 0x00A5);
    write_cmos_sensor(0x311C, 0x0BB8);
    write_cmos_sensor(0x311E, 0x0BB8);
    write_cmos_sensor(0x0342, 0x0E68);
    write_cmos_sensor(0x0340, 0x0274);
    write_cmos_sensor(0x0200, 0x0618);
    write_cmos_sensor(0x0202, 0x0002);
    write_cmos_sensor(0x0114, 0x0300);
    write_cmos_sensor(0x0204, 0x0020);
    write_cmos_sensor(0x0344, 0x0164);
    write_cmos_sensor(0x0348, 0x0B63);
    write_cmos_sensor(0x0346, 0x010C);
    write_cmos_sensor(0x034A, 0x088B);
    write_cmos_sensor(0x034C, 0x0280);
    write_cmos_sensor(0x034E, 0x01E0);
    write_cmos_sensor_8(0x3011, 0x01);
    write_cmos_sensor(0x0382, 0x0005);
    write_cmos_sensor(0x0386, 0x0005);
    write_cmos_sensor_8(0x0900, 0x01);
    write_cmos_sensor_8(0x0901, 0x33);
    write_cmos_sensor(0x31FE, 0xC004);
    write_cmos_sensor(0x3200, 0xC4F0);
    write_cmos_sensor(0x3202, 0xCEC8);
    write_cmos_sensor(0x3204, 0xD8A0);
    write_cmos_sensor(0x3206, 0xE278);
    write_cmos_sensor(0x3208, 0xEC50);
    write_cmos_sensor(0x320A, 0xF628);
    write_cmos_sensor(0x320C, 0x0000);
    write_cmos_sensor(0x320E, 0x09D8);
    write_cmos_sensor(0x3210, 0x13B0);
    write_cmos_sensor(0x3212, 0x1D88);
    write_cmos_sensor(0x3214, 0x2760);
    write_cmos_sensor(0x3216, 0x3138);
    write_cmos_sensor(0x3218, 0x3B10);
    write_cmos_sensor(0x321A, 0x3FFC);
    write_cmos_sensor(0x321C, 0xC004);
    write_cmos_sensor(0x321E, 0xCCD0);
    write_cmos_sensor(0x3220, 0xD99C);
    write_cmos_sensor(0x3222, 0xE668);
    write_cmos_sensor(0x3224, 0xF334);
    write_cmos_sensor(0x3226, 0x0000);
    write_cmos_sensor(0x3228, 0x0CCC);
    write_cmos_sensor(0x322A, 0x1998);
    write_cmos_sensor(0x322C, 0x2664);
    write_cmos_sensor(0x322E, 0x3330);
    write_cmos_sensor(0x3230, 0x3FFC);
    write_cmos_sensor(0x3232, 0x0100);
    write_cmos_sensor(0x3234, 0x0100);
    write_cmos_sensor_8(0x3237, 0x00);
    write_cmos_sensor_8(0x3238, 0x09);
    write_cmos_sensor_8(0x3239, 0x09);
    write_cmos_sensor_8(0x323A, 0x0B);
    write_cmos_sensor(0x3164, 0x09C4);
    write_cmos_sensor(0x3166, 0x0100);
    write_cmos_sensor(0x3168, 0x0100);
    write_cmos_sensor(0x316A, 0x0100);
    write_cmos_sensor(0x316C, 0x0100);
    write_cmos_sensor(0x316E, 0x001A);
    write_cmos_sensor(0x3170, 0x002F);
    write_cmos_sensor(0x3172, 0x0000);
    write_cmos_sensor(0x3174, 0x001A);
    write_cmos_sensor(0x3176, 0x0A8C);
    write_cmos_sensor(0x3178, 0x0100);
    write_cmos_sensor(0x317A, 0x0100);
    write_cmos_sensor(0x317C, 0x0100);
    write_cmos_sensor(0x317E, 0x0100);
    write_cmos_sensor(0x3180, 0x001A);
    write_cmos_sensor(0x3182, 0x002F);
    write_cmos_sensor(0x3184, 0x0000);
    write_cmos_sensor(0x3186, 0x001A);
    write_cmos_sensor(0x3188, 0x0CE4);
    write_cmos_sensor(0x318A, 0x0100);
    write_cmos_sensor(0x318C, 0x0100);
    write_cmos_sensor(0x318E, 0x0100);
    write_cmos_sensor(0x3190, 0x0100);
    write_cmos_sensor(0x3192, 0x001A);
    write_cmos_sensor(0x3194, 0x002F);
    write_cmos_sensor(0x3196, 0x0000);
    write_cmos_sensor(0x3198, 0x001A);
    write_cmos_sensor(0x319A, 0x1004);
    write_cmos_sensor(0x319C, 0x0100);
    write_cmos_sensor(0x319E, 0x0100);
    write_cmos_sensor(0x31A0, 0x0100);
    write_cmos_sensor(0x31A2, 0x0100);
    write_cmos_sensor(0x31A4, 0x001A);
    write_cmos_sensor(0x31A6, 0x002F);
    write_cmos_sensor(0x31A8, 0x0000);
    write_cmos_sensor(0x31AA, 0x001A);
    write_cmos_sensor(0x31AC, 0x1388);
    write_cmos_sensor(0x31AE, 0x0100);
    write_cmos_sensor(0x31B0, 0x0100);
    write_cmos_sensor(0x31B2, 0x0100);
    write_cmos_sensor(0x31B4, 0x0100);
    write_cmos_sensor(0x31B6, 0x001A);
    write_cmos_sensor(0x31B8, 0x002F);
    write_cmos_sensor(0x31BA, 0x0000);
    write_cmos_sensor(0x31BC, 0x001A);
    write_cmos_sensor(0x31BE, 0x1964);
    write_cmos_sensor(0x31C0, 0x0100);
    write_cmos_sensor(0x31C2, 0x0100);
    write_cmos_sensor(0x31C4, 0x0100);
    write_cmos_sensor(0x31C6, 0x0100);
    write_cmos_sensor(0x31C8, 0x001A);
    write_cmos_sensor(0x31CA, 0x002F);
    write_cmos_sensor(0x31CC, 0x0000);
    write_cmos_sensor(0x31CE, 0x001A);
    write_cmos_sensor(0x31D0, 0x1D4C);
    write_cmos_sensor(0x31D2, 0x0100);
    write_cmos_sensor(0x31D4, 0x0100);
    write_cmos_sensor(0x31D6, 0x0100);
    write_cmos_sensor(0x31D8, 0x0100);
    write_cmos_sensor(0x31DA, 0x001A);
    write_cmos_sensor(0x31DC, 0x002F);
    write_cmos_sensor(0x31DE, 0x0000);
    write_cmos_sensor(0x31E0, 0x001A);
    write_cmos_sensor_8(0x3162, 0x01);
    write_cmos_sensor(0x301C, 0x0100);
    write_cmos_sensor_8(0x301E, 0x03);
    write_cmos_sensor_8(0x323C, 0x00);
    write_cmos_sensor_8(0x323D, 0x01);
    write_cmos_sensor_8(0x1989, 0x04);
}

static void slim_video_setting(void)
{
    write_cmos_sensor(0x0136, 0x1800);
    write_cmos_sensor(0x0300, 0x0004);
    write_cmos_sensor(0x0302, 0x0001);
    write_cmos_sensor(0x0304, 0x0006);
    write_cmos_sensor(0x0306, 0x008C);
    write_cmos_sensor(0x0308, 0x0008);
    write_cmos_sensor(0x030A, 0x0001);
    write_cmos_sensor(0x030C, 0x0006);
    write_cmos_sensor(0x030E, 0x0082);
    write_cmos_sensor(0x311C, 0x0BB8);
    write_cmos_sensor(0x311E, 0x0BB8);
    write_cmos_sensor(0x0342, 0x0C10);
    write_cmos_sensor(0x0340, 0x05E6);
    write_cmos_sensor(0x0200, 0x0618);
    write_cmos_sensor(0x0202, 0x0002);
    write_cmos_sensor(0x0114, 0x0300);
    write_cmos_sensor(0x0204, 0x0020);
    write_cmos_sensor(0x0344, 0x0164);
    write_cmos_sensor(0x0348, 0x0B63);
    write_cmos_sensor(0x0346, 0x010C);
    write_cmos_sensor(0x034A, 0x088B);
    write_cmos_sensor(0x034C, 0x0280);
    write_cmos_sensor(0x034E, 0x01E0);
    write_cmos_sensor_8(0x3011, 0x01);
    write_cmos_sensor(0x0382, 0x0005);
    write_cmos_sensor(0x0386, 0x0005);
    write_cmos_sensor_8(0x0900, 0x01);
    write_cmos_sensor_8(0x0901, 0x33);
    write_cmos_sensor(0x31FE, 0xC004);
    write_cmos_sensor(0x3200, 0xC4F0);
    write_cmos_sensor(0x3202, 0xCEC8);
    write_cmos_sensor(0x3204, 0xD8A0);
    write_cmos_sensor(0x3206, 0xE278);
    write_cmos_sensor(0x3208, 0xEC50);
    write_cmos_sensor(0x320A, 0xF628);
    write_cmos_sensor(0x320C, 0x0000);
    write_cmos_sensor(0x320E, 0x09D8);
    write_cmos_sensor(0x3210, 0x13B0);
    write_cmos_sensor(0x3212, 0x1D88);
    write_cmos_sensor(0x3214, 0x2760);
    write_cmos_sensor(0x3216, 0x3138);
    write_cmos_sensor(0x3218, 0x3B10);
    write_cmos_sensor(0x321A, 0x3FFC);
    write_cmos_sensor(0x321C, 0xC004);
    write_cmos_sensor(0x321E, 0xCCD0);
    write_cmos_sensor(0x3220, 0xD99C);
    write_cmos_sensor(0x3222, 0xE668);
    write_cmos_sensor(0x3224, 0xF334);
    write_cmos_sensor(0x3226, 0x0000);
    write_cmos_sensor(0x3228, 0x0CCC);
    write_cmos_sensor(0x322A, 0x1998);
    write_cmos_sensor(0x322C, 0x2664);
    write_cmos_sensor(0x322E, 0x3330);
    write_cmos_sensor(0x3230, 0x3FFC);
    write_cmos_sensor(0x3232, 0x0100);
    write_cmos_sensor(0x3234, 0x0100);
    write_cmos_sensor_8(0x3237, 0x00);
    write_cmos_sensor_8(0x3238, 0x09);
    write_cmos_sensor_8(0x3239, 0x09);
    write_cmos_sensor_8(0x323A, 0x0B);
    write_cmos_sensor(0x3160, 0x0600);
    write_cmos_sensor(0x3164, 0x09C4);
    write_cmos_sensor(0x3166, 0x0100);
    write_cmos_sensor(0x3168, 0x0100);
    write_cmos_sensor(0x316A, 0x0100);
    write_cmos_sensor(0x316C, 0x0100);
    write_cmos_sensor(0x316E, 0x001A);
    write_cmos_sensor(0x3170, 0x002F);
    write_cmos_sensor(0x3172, 0x0000);
    write_cmos_sensor(0x3174, 0x001A);
    write_cmos_sensor(0x3176, 0x0A8C);
    write_cmos_sensor(0x3178, 0x0100);
    write_cmos_sensor(0x317A, 0x0100);
    write_cmos_sensor(0x317C, 0x0100);
    write_cmos_sensor(0x317E, 0x0100);
    write_cmos_sensor(0x3180, 0x001A);
    write_cmos_sensor(0x3182, 0x002F);
    write_cmos_sensor(0x3184, 0x0000);
    write_cmos_sensor(0x3186, 0x001A);
    write_cmos_sensor(0x3188, 0x0CE4);
    write_cmos_sensor(0x318A, 0x0100);
    write_cmos_sensor(0x318C, 0x0100);
    write_cmos_sensor(0x318E, 0x0100);
    write_cmos_sensor(0x3190, 0x0100);
    write_cmos_sensor(0x3192, 0x001A);
    write_cmos_sensor(0x3194, 0x002F);
    write_cmos_sensor(0x3196, 0x0000);
    write_cmos_sensor(0x3198, 0x001A);
    write_cmos_sensor(0x319A, 0x1004);
    write_cmos_sensor(0x319C, 0x0100);
    write_cmos_sensor(0x319E, 0x0100);
    write_cmos_sensor(0x31A0, 0x0100);
    write_cmos_sensor(0x31A2, 0x0100);
    write_cmos_sensor(0x31A4, 0x001A);
    write_cmos_sensor(0x31A6, 0x002F);
    write_cmos_sensor(0x31A8, 0x0000);
    write_cmos_sensor(0x31AA, 0x001A);
    write_cmos_sensor(0x31AC, 0x1388);
    write_cmos_sensor(0x31AE, 0x0100);
    write_cmos_sensor(0x31B0, 0x0100);
    write_cmos_sensor(0x31B2, 0x0100);
    write_cmos_sensor(0x31B4, 0x0100);
    write_cmos_sensor(0x31B6, 0x001A);
    write_cmos_sensor(0x31B8, 0x002F);
    write_cmos_sensor(0x31BA, 0x0000);
    write_cmos_sensor(0x31BC, 0x001A);
    write_cmos_sensor(0x31BE, 0x1964);
    write_cmos_sensor(0x31C0, 0x0100);
    write_cmos_sensor(0x31C2, 0x0100);
    write_cmos_sensor(0x31C4, 0x0100);
    write_cmos_sensor(0x31C6, 0x0100);
    write_cmos_sensor(0x31C8, 0x001A);
    write_cmos_sensor(0x31CA, 0x002F);
    write_cmos_sensor(0x31CC, 0x0000);
    write_cmos_sensor(0x31CE, 0x001A);
    write_cmos_sensor(0x31D0, 0x1D4C);
    write_cmos_sensor(0x31D2, 0x0100);
    write_cmos_sensor(0x31D4, 0x0100);
    write_cmos_sensor(0x31D6, 0x0100);
    write_cmos_sensor(0x31D8, 0x0100);
    write_cmos_sensor(0x31DA, 0x001A);
    write_cmos_sensor(0x31DC, 0x002F);
    write_cmos_sensor(0x31DE, 0x0000);
    write_cmos_sensor(0x31E0, 0x001A);
    write_cmos_sensor_8(0x3162, 0x01);
    write_cmos_sensor(0x301C, 0x0100);
    write_cmos_sensor_8(0x301E, 0x03);
    write_cmos_sensor_8(0x323C, 0x00);
    write_cmos_sensor_8(0x323D, 0x01);
    write_cmos_sensor_8(0x1989, 0x04);
}


extern unsigned int s5k3h7yx_read_region(struct i2c_client *client, unsigned int addr,unsigned char *data,unsigned int size);


/*************************************************************************
* FUNCTION
*    get_imgsensor_id
*
* DESCRIPTION
*    This function get the sensor ID
*
* PARAMETERS
*    *sensorID : return the sensor ID
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    kal_uint8 vendor_id =0;
    //sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            *sensor_id = return_sensor_id();
            if (*sensor_id == imgsensor_info.sensor_id) {
                write_cmos_sensor(0x0100,0x0103);
                if( s5k3h7yx_read_region(NULL,0,&vendor_id,1) < 0){
                    pr_err("s5k3h7yx read otp failed");
                    *sensor_id = 0xFFFFFFFF;
                    return ERROR_SENSOR_CONNECT_FAIL;
                }else{
                    if(0x57 == vendor_id){
                        LOG_INF("i2c write id: 0x%x, sensor id: 0x%x vendor_id 0x%x\n",imgsensor.i2c_write_id,*sensor_id,vendor_id);
                        return ERROR_NONE;
                    }else{
                        pr_err("Read sensor id fail i2c write id: 0x%x, sensor id: 0x%x vendor_id 0x%x\n",imgsensor.i2c_write_id,*sensor_id,vendor_id);
                        *sensor_id = 0xFFFFFFFF;
                        return ERROR_SENSOR_CONNECT_FAIL;
                    }
                }
            }
            LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        retry = 2;
    }
    if (*sensor_id != imgsensor_info.sensor_id) {
        // if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF
        *sensor_id = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*    open
*
* DESCRIPTION
*    This function initialize the registers of CMOS sensor
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
    //const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2};
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    kal_uint16 sensor_id = 0;

    //sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            sensor_id = return_sensor_id();
            if (sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
                break;
            }
            LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        if (sensor_id == imgsensor_info.sensor_id)
            break;
        retry = 2;
    }
    if (imgsensor_info.sensor_id != sensor_id)
        return ERROR_SENSOR_CONNECT_FAIL;

    /* initail sequence write in  */
    sensor_init();

    spin_lock(&imgsensor_drv_lock);

    imgsensor.autoflicker_en= KAL_FALSE;
    imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.dummy_pixel = 0;
    imgsensor.dummy_line = 0;
    imgsensor.ihdr_en = KAL_FALSE;
    imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    spin_unlock(&imgsensor_drv_lock);

    return ERROR_NONE;
}    /*    open  */



/*************************************************************************
* FUNCTION
*    close
*
* DESCRIPTION
*
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
    LOG_INF("E\n");
    streaming_control(0);
    /*No Need to implement this function*/

    return ERROR_NONE;
}    /*    close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*    This function start the sensor preview.
*
* PARAMETERS
*    *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    preview_setting();
    return ERROR_NONE;
}    /*    preview   */

/*************************************************************************
* FUNCTION
*    capture
*
* DESCRIPTION
*    This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                          MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E");
    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;

    if (imgsensor.current_fps == imgsensor_info.cap.max_framerate) // 30fps
    {
        imgsensor.pclk = imgsensor_info.cap.pclk;
        imgsensor.line_length = imgsensor_info.cap.linelength;
        imgsensor.frame_length = imgsensor_info.cap.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    }
    else //PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
    {
        if (imgsensor.current_fps != imgsensor_info.cap1.max_framerate)
            LOG_INF("Warning: current_fps %d fps is not support, so use cap1's setting: %d fps!\n",imgsensor.current_fps,imgsensor_info.cap1.max_framerate/10);
        imgsensor.pclk = imgsensor_info.cap1.pclk;
        imgsensor.line_length = imgsensor_info.cap1.linelength;
        imgsensor.frame_length = imgsensor_info.cap1.framelength;
        imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
        imgsensor.autoflicker_en = KAL_FALSE;
    }

    spin_unlock(&imgsensor_drv_lock);
    LOG_INF("Caputre fps:%d\n",imgsensor.current_fps);
    capture_setting(imgsensor.current_fps);
    return ERROR_NONE;
}    /* capture() */

static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
    imgsensor.pclk = imgsensor_info.normal_video.pclk;
    imgsensor.line_length = imgsensor_info.normal_video.linelength;
    imgsensor.frame_length = imgsensor_info.normal_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
    //imgsensor.current_fps = 300;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    normal_video_setting(imgsensor.current_fps);
    return ERROR_NONE;
}    /*    normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("enter");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
    imgsensor.pclk = imgsensor_info.hs_video.pclk;
    //imgsensor.video_mode = KAL_TRUE;
    imgsensor.line_length = imgsensor_info.hs_video.linelength;
    imgsensor.frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    hs_video_setting();
    return ERROR_NONE;
}    /*    hs_video   */


static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("slim video enter in");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
    imgsensor.pclk = imgsensor_info.slim_video.pclk;
    imgsensor.line_length = imgsensor_info.slim_video.linelength;
    imgsensor.frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    slim_video_setting();
    return ERROR_NONE;
}    /*    slim_video     */

static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
    LOG_INF(" get resolution, now the mode is%d",imgsensor.sensor_mode);
    sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
    sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

    sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
    sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

    sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
    sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


    sensor_resolution->SensorHighSpeedVideoWidth     = imgsensor_info.hs_video.grabwindow_width;
    sensor_resolution->SensorHighSpeedVideoHeight     = imgsensor_info.hs_video.grabwindow_height;

    sensor_resolution->SensorSlimVideoWidth     = imgsensor_info.slim_video.grabwindow_width;
    sensor_resolution->SensorSlimVideoHeight     = imgsensor_info.slim_video.grabwindow_height;

    return ERROR_NONE;
}    /*    get_resolution    */

static kal_uint32 get_info(enum MSDK_SCENARIO_ID_ENUM scenario_id,
                      MSDK_SENSOR_INFO_STRUCT *sensor_info,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d", scenario_id);


    //sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
    //sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
    //imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

    sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
    sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
    sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorInterruptDelayLines = 4; /* not use */
    sensor_info->SensorResetActiveHigh = FALSE; /* not use */
    sensor_info->SensorResetDelayCount = 5; /* not use */

    sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
    sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
    sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
    sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

    sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
    sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
    sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
    sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
    sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

    sensor_info->SensorMasterClockSwitch = 0; /* not use */
    sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;          /* The frame of setting shutter default 0 for TG int */
    sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;    /* The frame of setting sensor gain */
    sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
    sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
    sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
    sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

    sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
    sensor_info->SensorClockFreq = imgsensor_info.mclk;
    sensor_info->SensorClockDividCount = 3; /* not use */
    sensor_info->SensorClockRisingCount = 0;
    sensor_info->SensorClockFallingCount = 2; /* not use */
    sensor_info->SensorPixelClockCount = 3; /* not use */
    sensor_info->SensorDataLatchCount = 2; /* not use */

    sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
    sensor_info->SensorHightSampling = 0;    // 0 is default 1x
    sensor_info->SensorPacketECCOrder = 1;

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

            sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

            break;
        default:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
            break;
    }

    return ERROR_NONE;
}    /*    get_info  */


static kal_uint32 control(enum MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d", scenario_id);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.current_scenario_id = scenario_id;
    spin_unlock(&imgsensor_drv_lock);
    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            preview(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            capture(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            normal_video(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            hs_video(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            slim_video(image_window, sensor_config_data);
            break;
        default:
            LOG_INF("Error ScenarioId setting");
            preview(image_window, sensor_config_data);
            return ERROR_INVALID_SCENARIO_ID;
    }
    return ERROR_NONE;
}    /* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{

    LOG_INF("framerate = %d\n ", framerate);
    // SetVideoMode Function should fix framerate
    if (framerate == 0)
        // Dynamic frame rate
        return ERROR_NONE;
    spin_lock(&imgsensor_drv_lock);
    if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
        imgsensor.current_fps = 296;
    else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
        imgsensor.current_fps = 146;
    else
        imgsensor.current_fps = framerate;
    spin_unlock(&imgsensor_drv_lock);
    set_max_framerate(imgsensor.current_fps,1);

    return ERROR_NONE;

}


static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
    LOG_INF("enable = %d, framerate = %d ", enable, framerate);
    spin_lock(&imgsensor_drv_lock);
    if (enable)
        imgsensor.autoflicker_en = KAL_TRUE;
    else //Cancel Auto flick
        imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
    kal_uint32 frame_length;

    LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            if(framerate == 0)
                return ERROR_NONE;
            frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            if(framerate==300)
            {
            frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            }
            else
            {
            frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            }
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            LOG_INF("scenario hs_video:Currently camera mode is %d,framerate is %d , framelength=%d,linelength=%d\n",imgsensor.sensor_mode,imgsensor.current_fps,imgsensor.frame_length,imgsensor.line_length);
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
            imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        default:  //coding with  preview scenario by default
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
            break;
    }
    return ERROR_NONE;

}


static kal_uint32 get_default_framerate_by_scenario(enum MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
    LOG_INF("scenario_id = %d\n", scenario_id);

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            *framerate = imgsensor_info.pre.max_framerate;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            *framerate = imgsensor_info.normal_video.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            *framerate = imgsensor_info.cap.max_framerate;
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            *framerate = imgsensor_info.hs_video.max_framerate;
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            *framerate = imgsensor_info.slim_video.max_framerate;
            break;
        default:
            break;
    }

    return ERROR_NONE;

}

static kal_uint32 set_test_pattern_mode(kal_bool enable){
    LOG_INF("enable: %d\n", enable);

    if (enable) {
        // 0x5E00[8]: 1 enable,  0 disable
        // 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
        write_cmos_sensor(0x0600, 0x0002);
    } else {
        // 0x5E00[8]: 1 enable,  0 disable
        // 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
        write_cmos_sensor(0x0600, 0x0000);
    }
    spin_lock(&imgsensor_drv_lock);
    imgsensor.test_pattern = enable;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}


static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                             UINT8 *feature_para,UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16=(UINT16 *) feature_para;
    UINT16 *feature_data_16=(UINT16 *) feature_para;
    UINT32 *feature_return_para_32=(UINT32 *) feature_para;
    UINT32 *feature_data_32=(UINT32 *) feature_para;

     unsigned long long *feature_data=(unsigned long long *) feature_para;
     //unsigned long long *feature_return_para=(unsigned long long *) feature_para;

    struct SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

    LOG_INF("feature_id = %d", feature_id);
    switch (feature_id) {
        case SENSOR_FEATURE_GET_PERIOD:
            *feature_return_para_16++ = imgsensor.line_length;
            *feature_return_para_16 = imgsensor.frame_length;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            *feature_return_para_32 = imgsensor.pclk;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            write_shutter(*feature_data);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
             //night_mode((BOOL) *feature_data);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            set_gain((UINT16) *feature_data);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            if((sensor_reg_data->RegData>>8)>0)
               write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
            else
                write_cmos_sensor_8(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            get_imgsensor_id(feature_return_para_32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
            break;
        case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
            break;
        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((enum MSDK_SCENARIO_ID_ENUM)*feature_data, (MUINT32 *)(uintptr_t)(*(feature_data+1)));
            break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
            break;
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
            *feature_return_para_32 = imgsensor_info.checksum_value;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_FRAMERATE:
            LOG_INF("current fps :%d\n", (UINT32)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_SET_HDR:
            //LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
            LOG_INF("Warning! Not Support IHDR Feature");
            spin_lock(&imgsensor_drv_lock);
            //imgsensor.ihdr_en = (BOOL)*feature_data;
            imgsensor.ihdr_en = KAL_FALSE;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data_32);
        wininfo = (struct SENSOR_WINSIZE_INFO_STRUCT *)
            (uintptr_t)(*(feature_data+1));

        switch (*feature_data_32) {
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            memcpy((void *)wininfo,
                (void *)&imgsensor_winsize_info[1],
                sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
        break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            memcpy((void *)wininfo,
                (void *)&imgsensor_winsize_info[2],
                sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
        break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            memcpy((void *)wininfo,
                (void *)&imgsensor_winsize_info[3],
                sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
        break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            memcpy((void *)wininfo,
                (void *)&imgsensor_winsize_info[4],
                sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
        break;
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        default:
            memcpy((void *)wininfo,
                (void *)&imgsensor_winsize_info[0],
                sizeof(struct SENSOR_WINSIZE_INFO_STRUCT));
        break;
            }
            break;
        case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
                 LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));

                        ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            break;
        case SENSOR_FEATURE_SET_STREAMING_SUSPEND:
            streaming_control(KAL_FALSE);
            break;
        case SENSOR_FEATURE_SET_STREAMING_RESUME:
            if (*feature_data != 0)
                set_shutter(*feature_data);
            streaming_control(KAL_TRUE);
            break;
        case SENSOR_FEATURE_GET_MIPI_PIXEL_RATE:
            {
                kal_uint32 rate;
                switch (*feature_data) {
                    case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                        rate = imgsensor_info.cap.mipi_pixel_rate;
                        break;
                    case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                        rate = imgsensor_info.normal_video.mipi_pixel_rate;
                        break;
                    case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                        rate = imgsensor_info.hs_video.mipi_pixel_rate;
                        break;
                    case MSDK_SCENARIO_ID_SLIM_VIDEO:
                        rate = imgsensor_info.slim_video.mipi_pixel_rate;
                        break;
                    case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                        rate = imgsensor_info.pre.mipi_pixel_rate;
                        break;
                    default:
                        rate = 0;
                        break;
                   }
                  *(MUINT32 *)(uintptr_t)(*(feature_data + 1)) = rate;
            }
            break;
    case SENSOR_FEATURE_GET_AE_EFFECTIVE_FRAME_FOR_LE:
       *feature_return_para_32 = imgsensor.current_ae_effective_frame;
        break;
    case SENSOR_FEATURE_GET_AE_FRAME_MODE_FOR_LE:
         memcpy(feature_return_para_32, &imgsensor.ae_frm_mode,
               sizeof(struct IMGSENSOR_AE_FRM_MODE));
        break;

        default:
            break;
    }

    return ERROR_NONE;
}    /*    feature_control()  */

static struct SENSOR_FUNCTION_STRUCT sensor_func = {
    open,
    get_info,
    get_resolution,
    feature_control,
    control,
    close
};

UINT32 S5K3H7YX_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&sensor_func;
    return ERROR_NONE;
}    /*    s5k2p8_MIPI_RAW_SensorInit    */

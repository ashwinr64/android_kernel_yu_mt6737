#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h> 
#include <asm/atomic.h>
//#include <linux/xlog.h>
//#include <asm/system.h>

//#include "kd_camera_hw.h"
#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "ov5675mipi_Sensor.h"

#define PFX "ov5675_sunwin_otp"
#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)
#define i2c_write_id  0x20
static kal_uint16 OV5675_read_i2c(kal_uint32 addr)
{
	kal_uint16 get_byte=0;

	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, 0x20);

	return get_byte;
}

static void OV5675_write_i2c(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
	iWriteRegI2C(pu_send_cmd, 3, 0x20);
}

struct otp_struct {
    int flag; // bit[7]: info, bit[6]:wb
    int module_integrator_id;
    int lens_id;
    int production_year;
    int production_month;
    int production_day;
    int rg_ratio;
    int bg_ratio;
}otp_ptr;

int RG_Ratio_Typical = 300; //golden module
int BG_Ratio_Typical = 304;
// return value:
// bit[7]: 0 no otp info, 1 valid otp info
// bit[6]: 0 no otp wb, 1 valib otp wb
int ov5675_read_otp(void)
{
    LOG_INF("TINNO_CAMERA_HAL_DRIVER_LOG.goto ov5675 read awb otp\n");
#if 1   
    u16 otp_flag = 0, addr = 0, temp = 0, i = 0;
    //set 0x5002 to ¡°0x02¡±
    u16 temp1 = 0;
    OV5675_write_i2c(0x0100, 0x01);
    temp1 = OV5675_read_i2c(0x5001);
    OV5675_write_i2c(0x5001, 0x02);
    // read OTP into buffer
    OV5675_write_i2c(0x3d84, 0xC0);
    OV5675_write_i2c(0x3d88, 0x70); // OTP start address
    OV5675_write_i2c(0x3d89, 0x10);
    OV5675_write_i2c(0x3d8A, 0x70); // OTP end address
    OV5675_write_i2c(0x3d8B, 0x29);
    OV5675_write_i2c(0x3d81, 0x01); // load otp into buffer
    // OTP into
    mdelay(10);
    otp_flag = OV5675_read_i2c(0x7010);
    addr = 0;
    if((otp_flag & 0xc0) == 0x40) {
        addr = 0x7011; // base address of info group 1
    }
    else if((otp_flag & 0x30) == 0x10) {
        addr = 0x7016; // base address of info group 2
    }
    else if((otp_flag & 0x0c) == 0x04) {
    addr = 0x701b; // base address of info group 3
    }
    if(addr != 0) {
        otp_ptr.flag = 0x80; // valid base info in OTP
        otp_ptr.module_integrator_id = OV5675_read_i2c(addr);
        otp_ptr.lens_id = OV5675_read_i2c(addr + 1);
        otp_ptr.production_year = OV5675_read_i2c(addr + 2);
        otp_ptr.production_month = OV5675_read_i2c(addr + 3);
        otp_ptr.production_day = OV5675_read_i2c(addr + 4);
    }
    else {
        otp_ptr.flag = 0x00; // not info in OTP
       otp_ptr.module_integrator_id = 0;
        otp_ptr.lens_id = 0;
        otp_ptr.production_year = 0;
        otp_ptr.production_month = 0;
        otp_ptr.production_day = 0;
    }
    LOG_INF("TINNO_CAMERA_HAL_DRIVER_LOG.OV5675_module_integrator_id = %d\n",otp_ptr.module_integrator_id);
    // OTP WB Calibration
    otp_flag = OV5675_read_i2c(0x7020);
    addr = 0;
    if((otp_flag & 0xc0) == 0x40) {
         addr = 0x7021; // base address of WB Calibration group 1
    }
    else if((otp_flag & 0x30) == 0x10) {
        addr = 0x7024; // base address of WB Calibration group 2
    }
    else if((otp_flag & 0x0c) == 0x04) {
        addr = 0x7027; // base address of WB Calibration group 3
    }
    if(addr != 0) {
        otp_ptr.flag |= 0x40;
        temp = OV5675_read_i2c(addr + 2);
        otp_ptr.rg_ratio = (OV5675_read_i2c(addr)<<2) + ((temp>>6) & 0x03);
        otp_ptr.bg_ratio = (OV5675_read_i2c(addr + 1)<<2) + ((temp>>4) & 0x03);
        LOG_INF("TINNO_CAMERA_HAL_DRIVER_LOG_2A_OTP_DATA.ov5675_rg_ratio =%d,bg_ratio = %d\n",otp_ptr.rg_ratio,otp_ptr.bg_ratio);
    }
    else {
        otp_ptr.rg_ratio = 0;
        otp_ptr.bg_ratio = 0;
    }
#endif
    for(i=0x7010;i<=0x7029;i++) {
        OV5675_write_i2c(i,0); // clear OTP buffer, recommended use continuous write to accelarate
    }
    //set 0x5002 to ¡°0x0a¡±
   temp1 = OV5675_read_i2c(0x5001);
   OV5675_write_i2c(0x5001, 0x0a);
  // OV5675_write_i2c(0x5001, 0x0a);
   // OV5675_write_i2c(0x5002, 0x03);
    return otp_ptr.flag;

}
// return value:
// bit[7]: 0 no otp info, 1 valid otp info
// bit[6]: 0 no otp wb, 1 valib otp wb
int ov5675_apply_otp(void)
{
    int rg, bg, R_gain, G_gain, B_gain, Base_gain;
    LOG_INF("TINNO_CAMERA_HAL_DRIVER_LOG.ov5675_apply_otp\n");
    LOG_INF("TINNO_CAMERA_HAL_DRIVER_LOG.ov5675.rg_ratio = %d,bg_ratio = %d\n",otp_ptr.rg_ratio,otp_ptr.bg_ratio);
    // apply OTP WB Calibration
    if (otp_ptr.flag & 0x40) 
    {
        rg = otp_ptr.rg_ratio;
        bg = otp_ptr.bg_ratio;
        //calculate G gain
        R_gain = (RG_Ratio_Typical*1000) / rg;
        B_gain = (BG_Ratio_Typical*1000) / bg;
        G_gain = 1000;
        if (R_gain < 1000 || B_gain < 1000)
        {
            if (R_gain < B_gain) Base_gain = R_gain;
        else
        {
             Base_gain = B_gain;
        }
        }
        else
        {
            Base_gain = G_gain;
        }
        R_gain = 0x400 * R_gain / (Base_gain);
        B_gain = 0x400 * B_gain / (Base_gain);
        G_gain = 0x400 * G_gain / (Base_gain);
         if(g_AWB_Value.R_G != 0)
        {
           R_gain = g_AWB_Value.R_G;           
        }
         if(g_AWB_Value.B_G !=0)
            {
            B_gain = g_AWB_Value.B_G;           
         }
        LOG_INF("TINNO_CAMERA_HAL_DRIVER_LOG_GAIN.OV5675 [R_GAIN=%d],[G_GAIN=%d],[B_GAIN=%d] \n",R_gain, G_gain,B_gain);
        // update sensorWB gain
        if (R_gain>0x400) {
            OV5675_write_i2c(0x5019, R_gain>>8);
            OV5675_write_i2c(0x501a, R_gain & 0x00ff);
        }
        if (G_gain>0x400) {
            OV5675_write_i2c(0x501b, G_gain>>8);
            OV5675_write_i2c(0x501c, G_gain & 0x00ff);
        }
        if (B_gain>0x400) {
            OV5675_write_i2c(0x501d, B_gain>>8);
            OV5675_write_i2c(0x501e, B_gain & 0x00ff);
        }
     }
        return otp_ptr.flag;
}

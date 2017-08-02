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
#include "kd_camera_typedef.h"
#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "s5k5e8yxmipiraw_Sensor.h"

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern void kdSetI2CSpeed(u16 i2cSpeed);

#define PFX "s5k5e8_sunwin_otp"
#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)

//#define LOG_INF(format, args...) pr_debug("[%s] " fmt, __FUNCTION__, ##arg)
#define I2c_write_id    0x20

#define gain_default 0x0100
#define R_ave_golden  99//golden module
#define B_ave_golden   92
#define G_ave_golden   146

 static u8 R_ave = 1;//current module
 static u8 B_ave = 1;
 static u8 G_ave = 1;
 
static kal_uint16 read_cmos_sensor_8(kal_uint32 addr)
{
	kal_uint16 get_byte=0;

	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, I2c_write_id);

	return get_byte;
}

static void write_cmos_sensor_8(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
	iWriteRegI2C(pu_send_cmd, 3, I2c_write_id);
}
static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para >> 8), (char)(para & 0xFF)};
	iWriteRegI2C(pu_send_cmd, 4, I2c_write_id);
}

void start_read_otp(void)
{
    write_cmos_sensor_8(0x0A00,0x04);
    write_cmos_sensor_8(0x0A02,0x04);   //PAGE 0X04
    write_cmos_sensor_8(0x0A00,0x01);   //READ ENAB
}
int S5K5E8_OTP_TEST(void)
{
    int info_flag;
    u8 year = 0, month = 0, day = 0; 
    start_read_otp();
    info_flag = read_cmos_sensor_8(0x0A04);
    if(info_flag != 0x40 || info_flag == 0)
    {
        LOG_INF("info flag error!");
        return 0;
    }
    else
     {
           year = read_cmos_sensor_8(0x0A26);
            month = read_cmos_sensor_8(0x0A27);
            day = read_cmos_sensor_8(0x0A28);
           
    }
    LOG_INF("ls.S5K5E8_OTP_TEST.year =20%d,month = %d,day = %d",year,month,day);
    return 1;
    
}
int Read_S5K5E8_OTP_Data(void)
{ 
    LOG_INF("goto_Read_S5K5E8_OTP_Data");
   int info_flag = 0,awb_flag = 0;
   start_read_otp();
   info_flag = read_cmos_sensor_8(0x0A04);
   awb_flag = read_cmos_sensor_8(0x0A0f);
   LOG_INF("S5K5E8_OTP_TEST.info_flag = %d,awb_flag = %d\n",info_flag,awb_flag);
   if((info_flag == 0) || (awb_flag == 0))
   {
        return 0;
   }
   if(awb_flag != 0x40)
    {
        return 0;
   }
   R_ave = read_cmos_sensor_8(0x0A11);
   G_ave = read_cmos_sensor_8(0x0A13);
   B_ave = read_cmos_sensor_8(0x0A15);
   LOG_INF("S5K5E8_OTP_TEST.AWB_R_ave = %d,B_ave = %d,G_ave = %d\n",R_ave,B_ave,G_ave);
   LOG_INF("TINNO_CAMERA_HAL_DRIVER_LOG_2A_OTP_DATA.S5K5E8.R_ave = %d,B_ave = %d,G_ave = %d\n",R_ave,B_ave,G_ave);
   LOG_INF("TINNO_CAMERA_HAL_DRIVER_LOG_2A_OTP_DATA.S5K5E8.R_ave_golden = %d,B_ave_golden = %d,G_ave_golden = %d\n",R_ave_golden,B_ave_golden,G_ave_golden);
   return 1;
   
}

void S5K5E8_OTP_AUTO_LOAD_LSC(void)
{
    LOG_INF("TINNO_CAMERA_HAL_DRIVER_LOG.S5K5E8_OTP_AUTO_LOAD_LSC\n");
    start_read_otp();
    write_cmos_sensor_8(0x0B00,0x01);   //LSC AUTO LOAD 
    write_cmos_sensor_8(0x3400,0x00);
}
#if 1 //0
int S5K5E8_sunwin_update_otp_wb(void)
{
  u16 R_gain = 0,B_gain = 0,G_gain = 0;     
  
 float  R_ratio = (R_ave_golden/G_ave_golden) / (R_ave / G_ave);
 float B_ratio = (B_ave_golden/G_ave_golden) / (B_ave / G_ave);

  if(R_ratio >= 1)
  {
        if(B_ratio >=1)
        {
             G_gain = gain_default;
            R_gain = gain_default * R_ratio;
            B_gain = gain_default * B_ratio;
        }
        else
        {
            B_gain = gain_default;
            G_gain = gain_default / B_ratio;
            R_gain = gain_default * R_ratio / B_ratio;
        }
  }
  else
   {
        if(B_ratio >= 1)
        {
            R_gain = gain_default;
            G_gain = gain_default / R_ratio;
            B_gain = gain_default * B_ratio / R_ratio;
        }
        else
        {
            if(R_ratio>=B_ratio<1)
            {
                B_gain = gain_default;
                G_gain = gain_default / B_ratio;
                R_gain = gain_default * R_ratio / B_ratio;   
            }
            else
             {
                R_gain = gain_default;
                G_gain = gain_default / R_ratio;
                B_gain = gain_default * B_ratio / R_ratio ;           
            }
        }
  }
  //=====================================
  //R_gain  G_gain B_gain Ð´Èë¼Ä´æÆ÷
  //=====================================

}
#endif

int  S5K5E8_sunwin_update_gain(void)
{

	  u32 R_ration,B_ration;
        u32 R_G_golden,B_G_golden;
        u32 R_G,B_G;
       LOG_INF("TINNO_CAMERA_HAL_DRIVER_LOG.S5K5E8_sunwin_update_gain\n");
       LOG_INF("TINNO_CAMERA_HAL_DRIVER_LOG.S5K5E8_READ_WB_DATA.R_ave = %d,B_ave = %d,G_ave = %d\n",R_ave,B_ave,G_ave);
       R_G_golden = 1000 * R_ave_golden  / G_ave_golden;
       B_G_golden = 1000 * B_ave_golden / G_ave_golden; //1200
       R_G = 1000*R_ave / G_ave; 
       B_G = 1000*B_ave / G_ave; 

       R_ration = 1000 * R_G_golden / R_G ;
       B_ration = 1000 * B_G_golden / B_G ;
       LOG_INF("R_ave = %d,B_ave= %d,G_ave = %d\n",R_ave,B_ave,G_ave);
       LOG_INF("R_G = %d,B_G = %d\n",R_G,B_G);
       LOG_INF("R_ration = %d,B_ration = %d\n",R_ration,B_ration);
       
	u16 R_gain,G_gain,B_gain;
    
	if(R_ration>=1000)
	{
		if(B_ration>=1000)
		{
			G_gain = gain_default;
			R_gain = gain_default*R_ration/1000;
			B_gain = gain_default*B_ration/1000;
		}
		else
		{
			B_gain = gain_default;
			G_gain = gain_default*1000/B_ration;
			R_gain = (gain_default*R_ration) / B_ration;
		}
	}
	else
	{
		if(B_ration>=1000)
		{
                    R_gain = gain_default;
			G_gain = gain_default*1000/R_ration;
			B_gain = gain_default*B_ration/R_ration;
		}
		else
		{
			if(R_ration>=B_ration)
			{
                            B_gain = gain_default;
				G_gain = gain_default*1000/B_ration;
				R_gain = gain_default*R_ration/B_ration;
			}
			else
			{
                            R_gain = gain_default;
				G_gain = gain_default*1000/R_ration;
				B_gain = gain_default*B_ration/R_ration;
			}				
		}
	}
    if(g_AWB_Value.R_G != 0)
    {
           R_gain = g_AWB_Value.R_G;           
    }
    if(g_AWB_Value.B_G !=0)
        {
          B_gain = g_AWB_Value.B_G;      
    }
    LOG_INF("TINNO_CAMERA_HAL_DRIVER_LOG_GAIN.S5K5E8YX [R_GAIN=%d],[G_GAIN=%d],[B_GAIN=%d] \n",R_gain, G_gain, B_gain);
   //   LOG_INF("s5k5e8 G_gain = %d,R_gain = %d,B_gain = %d",G_gain,R_gain,B_gain);
	write_cmos_sensor(0x020E,G_gain);//GR ;value 16bit
	write_cmos_sensor(0x0210,R_gain);//R
	write_cmos_sensor(0x0212,B_gain);//B
	write_cmos_sensor(0x0214,G_gain);//GB

}


















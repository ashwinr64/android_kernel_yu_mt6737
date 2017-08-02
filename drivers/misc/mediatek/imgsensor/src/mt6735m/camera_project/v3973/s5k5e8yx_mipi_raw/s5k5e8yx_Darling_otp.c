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
#define I2c_write_id    0x5A

#define gain_default 0x0100
#define R_Gr_golden  304//golden module
#define B_Gr_golden  254
#define Gb_Gr_golden 512

static int R_Gr;//current module
static int B_Gr;
static int Gb_Gr;

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

int Read_S5K5E8_Darling_OTP_Data(void)
{ 
	LOG_INF("goto_Read_S5K5E8_OTP_Data");
	UINT8 info_flag = 0,awb_flag = 0;

	write_cmos_sensor_8(0x0A00,0x04); //:make initial state
	write_cmos_sensor_8(0x0A02,0x04);   //PAGE 0X04
	write_cmos_sensor_8(0x0A00,0x01);   //READ ENAB
	mdelay(5);

	awb_flag = read_cmos_sensor_8(0x0A04);
	LOG_INF("awb_flag = %d",awb_flag);
	LOG_INF("R_Gr = %d",(read_cmos_sensor_8(0x0A07) << 2) | ((read_cmos_sensor_8(0x0A0A)>>6 & 0x03)));
	LOG_INF("B_Gr = %d",(read_cmos_sensor_8(0x0A08) << 2) | ((read_cmos_sensor_8(0x0A0A)>>4 & 0x03)));
	LOG_INF("Gb_Gr = %d",(read_cmos_sensor_8(0x0A09) << 2) | ((read_cmos_sensor_8(0x0A0A)>>2 & 0x03)));
	if(((awb_flag >> 6) &0x03) == 0x01)
	{
		R_Gr = (read_cmos_sensor_8(0x0A07) << 2) | ((read_cmos_sensor_8(0x0A0A)>>6 & 0x03));
		B_Gr = (read_cmos_sensor_8(0x0A08) << 2) | ((read_cmos_sensor_8(0x0A0A)>>4 & 0x03));
		Gb_Gr = (read_cmos_sensor_8(0x0A09) << 2) | ((read_cmos_sensor_8(0x0A0A)>>2 & 0x03));
	}
	else if (((awb_flag >> 4) &0x03) == 0x01)
	{
		R_Gr = (read_cmos_sensor_8(0x0A0D) << 2) | ((read_cmos_sensor_8(0x0A10)>>6 & 0x03));
		B_Gr = (read_cmos_sensor_8(0x0A0E) << 2) | ((read_cmos_sensor_8(0x0A10)>>4 & 0x03));
		Gb_Gr = (read_cmos_sensor_8(0x0A0F) << 2) | ((read_cmos_sensor_8(0x0A10)>>2 & 0x03));  
	}
	else
	{
		R_Gr = 0;
		B_Gr = 0;
		Gb_Gr = 0; 
	}
	write_cmos_sensor_8(0x0A00,0x04); ////:make initial state
	write_cmos_sensor_8(0x0A00,0x00); //READ DISENAB
	LOG_INF("TINNO_CAMERA_HAL_DRIVER_LOG_2A_OTP_DATA.S5K5E8.R_Gr = %d,B_Gr = %d,Gb_Gr = %d\n",R_Gr,B_Gr,Gb_Gr);
	LOG_INF("TINNO_CAMERA_HAL_DRIVER_LOG_2A_OTP_DATA.S5K5E8.R_ave_golden = %d,B_ave_golden = %d,G_ave_golden = %d\n",R_Gr_golden,B_Gr_golden,Gb_Gr_golden);
	return 1;

}


int S5K5E8_Darling_update_otp_wb(void)
{
	LOG_INF("TINNO_CAMERA_HAL_DRIVER_LOG.S5K5E8_DARLING_update_gain\n");
	int R_gain =0,B_gain = 0,Gb_gain =0 ,Gr_gain = 0,Base_gain = 0;
	if((R_Gr == 0) ||(B_Gr == 0)||(Gb_Gr == 0))
	{
		return 0;
	}
	else
	{
		R_gain = (R_Gr_golden*1000) / R_Gr;
		B_gain = (B_Gr_golden*1000) / B_Gr;
		Gb_gain = (Gb_Gr_golden*1000) / Gb_Gr;
		Gr_gain = 1000;
		Base_gain = R_gain;
		if(Base_gain>B_gain) Base_gain=B_gain;
		if(Base_gain>Gb_gain) Base_gain=Gb_gain;
		if(Base_gain>Gr_gain) Base_gain=Gr_gain;
		R_gain = 0x100 * R_gain / Base_gain;
		B_gain = 0x100 * B_gain / Base_gain;
		Gb_gain = 0x100 * Gb_gain / Base_gain;
		Gr_gain = 0x100 * Gr_gain / Base_gain;

		if(Gr_gain>0x100)
		{
			write_cmos_sensor_8(0x020E,Gr_gain>>8);
			write_cmos_sensor_8(0x020F,Gr_gain&0xff);
		}
		if(R_gain>0x100)
		{
			write_cmos_sensor_8(0x0210,R_gain>>8);
			write_cmos_sensor_8(0x0211,R_gain&0xff);
		}
		if(B_gain>0x100)
		{
			write_cmos_sensor_8(0x0212,B_gain>>8);
			write_cmos_sensor_8(0x0213,B_gain&0xff);
		}
		if(Gb_gain>0x100)
		{
			write_cmos_sensor_8(0x0214,Gb_gain>>8);
			write_cmos_sensor_8(0x0215,Gb_gain&0xff);
		}
	}
	return 1;

}




















#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "ov8856mipiraw_Sensor.h"

#define PFX "ov8856_sunwin_otp"
#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)


static kal_uint16 ov8856_read_i2c(kal_uint32 addr)
{
	kal_uint16 get_byte=0;

	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, 0x20);

	return get_byte;
}

static void ov8856_write_i2c(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
	iWriteRegI2C(pu_send_cmd, 3, 0x20);
}

struct otp_struct
{
	int flag; // bit[7]: info, bit[6]:wb, bit[5]:vcm, bit[4]:lenc
	int module_integrator_id;
	int lens_id;
	int production_year;
	int production_month;
	int production_day;
	int rg_ratio;
	int bg_ratio;
	int lenc[240];
	int checksum;
}otp_info;


static int RG_Ratio_Typical= 0x131 ;  
static int BG_Ratio_Typical= 0x176 ; 

int ov8856_read_otp(void)
{
	int otp_flag, addr, temp, i;

	ov8856_write_i2c(0x0100,0x01);
	mdelay(10);
	int temp1;
	temp1 = ov8856_read_i2c(0x5001);    

	ov8856_write_i2c(0x5001, (0x00 & 0x08) | (temp1 & (~0x08)));
	// read OTP into buffer
	ov8856_write_i2c(0x3d84, 0xC0);
	ov8856_write_i2c(0x3d88, 0x70); // OTP start address
	ov8856_write_i2c(0x3d89, 0x10);
	ov8856_write_i2c(0x3d8A, 0x72); // OTP end address
	ov8856_write_i2c(0x3d8B, 0x0E);
	ov8856_write_i2c(0x3d81, 0x01); // load otp into buffer
	mdelay(10);

	// OTP base information and WB calibration data
	otp_flag = ov8856_read_i2c(0x7010);
	LOG_INF("ov8856_otp_read_test  otp_flag: 0x%x,\n", otp_flag); 
	addr = 0;

	if((otp_flag & 0xc0) == 0x40) 
	{
		addr = 0x7011; // base address of info group 1
	}
	else if((otp_flag & 0x30) == 0x10) 
	{
		addr = 0x7019; // base address of info group 2
	}
	if(addr != 0)
	{
		otp_info.flag = 0xC0; // valid info and AWB in OTP
		otp_info.module_integrator_id = ov8856_read_i2c(addr);
		otp_info.lens_id = ov8856_read_i2c( addr + 1); 
		otp_info.production_year = ov8856_read_i2c( addr + 2); 
		otp_info.production_month = ov8856_read_i2c( addr + 3); 
		otp_info.production_day = ov8856_read_i2c(addr + 4); 
		temp = ov8856_read_i2c(addr + 7);
		otp_info.rg_ratio = (ov8856_read_i2c(addr + 5)<<2) + ((temp>>6) & 0x03);
		otp_info.bg_ratio = (ov8856_read_i2c(addr + 6)<<2) + ((temp>>4) & 0x03);
	}
	else
	{
		otp_info.flag = 0x00; // not info and AWB in OTP
		otp_info.module_integrator_id = 0;
		otp_info.lens_id = 0;
		otp_info.production_year = 0;
		otp_info.production_month = 0;
		otp_info.production_day = 0;
		otp_info.rg_ratio = 0;
		otp_info.bg_ratio = 0;
	}

	// OTP Lenc Calibration
	otp_flag = ov8856_read_i2c(0x7028);
	LOG_INF("ov8856_otp_read_test lsc otp_flag: 0x%x,\n", otp_flag); 

	addr = 0;
	int checksum2=0;	

	if((otp_flag & 0xc0) == 0x40)
	{
		addr = 0x7029; //base address of Lenc Calibration group 1
	}
	else if((otp_flag & 0x30) == 0x10)
	{
		addr = 0x711a;; //base address of Lenc Calibration group 2
	}	

	if(addr != 0)
	{
		for(i=0;i<240;i++)
		{
			otp_info.lenc[i]=ov8856_read_i2c(addr + i);
			checksum2 += otp_info.lenc[i];
		}
		checksum2 = (checksum2)%255 +1;
		otp_info.checksum = ov8856_read_i2c((addr + 240));

		if(otp_info.checksum == checksum2)
		{
			otp_info.flag |= 0x10;
		}
	}
	else
	{
		for(i=0;i<240;i++)
		{
			otp_info.lenc[i]=0;
		}
	}
	for(i=0x7010;i<=0x720e;i++) 
	{
		ov8856_write_i2c(i,0); // clear OTP buffer, recommended use continuous write to accelarate
	}
	//set 0x5001[3] to \A1\B01\A1\B1
	temp1 = ov8856_read_i2c(0x5001);
	ov8856_write_i2c(0x5001, (0x08 & 0x08) | (temp1 & (~0x08)));

	return otp_info.flag;
}

int ov8856_apply_otp(void)
{
	int rg, bg, R_gain, G_gain, B_gain, Base_gain, temp, i;
	// apply OTP WB Calibration
	if (otp_info.flag & 0x40)
	{
		rg = otp_info.rg_ratio;
		bg = otp_info.bg_ratio;
		//calculate G gain
		R_gain = (RG_Ratio_Typical*1000) / rg;
		B_gain = (BG_Ratio_Typical*1000) / bg;
		G_gain = 1000;
		if (R_gain < 1000 || B_gain < 1000)
		{
			if (R_gain < B_gain)
				Base_gain = R_gain;
			else
				Base_gain = B_gain;
		}
		else
		{
			Base_gain = G_gain;
		}
		R_gain = 0x400 * R_gain / (Base_gain);
		B_gain = 0x400 * B_gain / (Base_gain);
		G_gain = 0x400 * G_gain / (Base_gain);
		// update sensor WB gain
		if (R_gain>0x400)
		{
			ov8856_write_i2c(0x5019, R_gain>>8);
			ov8856_write_i2c(0x501a, R_gain & 0x00ff);
		}
		if (G_gain>0x400)
		{
			ov8856_write_i2c(0x501b, G_gain>>8);
			ov8856_write_i2c(0x501c, G_gain & 0x00ff);
		}
		if (B_gain>0x400)
		{
			ov8856_write_i2c(0x501d, B_gain>>8);
			ov8856_write_i2c(0x501e, B_gain & 0x00ff);
		}
	}
	printk("ov8856_sunwin_otp>>>>0x%x>>>0x%x>>>0x%x\n",R_gain,B_gain,G_gain);
	// apply OTP Lenc Calibration
	if (otp_info.flag & 0x10)
	{
		temp = ov8856_read_i2c(0x5000);
		temp = 0x20 | temp;
		ov8856_write_i2c(0x5000, temp);
		for(i=0;i<240;i++)
		{
			ov8856_write_i2c(0x5900 + i, otp_info.lenc[i]);
		}
		printk("ov8856_sunwin_otp>>>>>>lsc success\n");
	}
	return otp_info.flag;
}



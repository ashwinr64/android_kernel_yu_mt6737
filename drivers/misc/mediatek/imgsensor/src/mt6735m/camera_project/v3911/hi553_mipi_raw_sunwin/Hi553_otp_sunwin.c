/*
NOTE:
The modification is appended to initialization of image sensor. 
After sensor initialization, use the function
bool otp_update_wb(unsigned short golden_rg, unsigned short golden_bg),
then the calibration of AWB will be applied. 
After finishing the OTP written, we will provide you the golden_rg and golden_bg settings.
*/
/*
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>


#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
	
#include "ov5670_Sensor.h"
#include "ov5670_Camera_Sensor_para.h"
#include "ov5670_CameraCustomized.h"
*/


#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "Hi553_mipi_raw_sunwin.h"

#define  DEBUG_PRINT	1
//#undef printk
//#define printk(fmt, args...) printk(KERN_INFO "ov5670_OTP.c: " fmt, ## args)
//#define printk(fmt, arg...) printk("[ov5670MIPIRaw_OTP_CMK]%s: " fmt "\n", __FUNCTION__ ,##arg)//LINE <> <DATE20130923> <ov5670 OTP log> wupingzhou

extern kal_uint8 hi553_read_cmos_sensor_for_otp(kal_uint16 addr);

extern void hi553_wirite_cmos_sensor_for_otp(kal_uint16 addr, kal_uint8 para);
extern void write_cmos_sensor_for_otp_16(kal_uint16 addr, kal_uint16 para);



#define OTP_DATA_ADDR         0x3D00
#define OTP_LOAD_ADDR         0x3D81

#define OTP_WB_GROUP_ADDR     0x3D05
#define OTP_WB_GROUP_SIZE     9
#define OTP_BANK_ADDR         0x3D84
#define OTP_BANK              0x3D85
#define OTP_END_ADDR          0x3D86


#define GAIN_DEFAULT_VALUE    0x0400 // 1x gain

#define RG_Ratio_Typical 	1  
#define BG_Ratio_Typical 	1  

#define OTP_MID               0x06
//#define LENS_ID              0x46

static kal_uint16 r_unit,b_unit,g_unit;

// R/G and B/G of current camera module
static unsigned char RG_MSB = 0;
static unsigned char BG_MSB = 0;
static unsigned char AWB_LSB = 0;
static float rg_ratio_unit;
static float bg_ratio_unit;
signed char check_MID_sunwin_hi553(bool bOnlyCheck);


/*******************************************************************************
* Function    :  otp_update_wb
* Description :  Update white balance settings from OTP
* Parameters  :  [in] golden_rg : R/G of golden camera module
                 [in] golden_bg : B/G of golden camera module
* Return      :  1, success; 0, fail
*******************************************************************************/	
void hi553_otp_enable(void)
{

	hi553_wirite_cmos_sensor_for_otp(0x0a02, 0x01); //Fast sleep on
	hi553_wirite_cmos_sensor_for_otp(0x0a00, 0x00); //stand by on	
	mdelay(100);
	hi553_wirite_cmos_sensor_for_otp(0x0f02, 0x00); //pll disable	
	hi553_wirite_cmos_sensor_for_otp(0x011a, 0x01); 	
	hi553_wirite_cmos_sensor_for_otp(0x011b, 0x09); 	
	hi553_wirite_cmos_sensor_for_otp(0x0d04, 0x01); 	
	hi553_wirite_cmos_sensor_for_otp(0x0d00, 0x07); 	
	hi553_wirite_cmos_sensor_for_otp(0x003f, 0x10); 	
	hi553_wirite_cmos_sensor_for_otp(0x0a00, 0x01); //stand by off	
}

void hi553_otp_disable(void)
{
	hi553_wirite_cmos_sensor_for_otp(0x0a00, 0x00); //stand by on	
	mdelay(100);
	hi553_wirite_cmos_sensor_for_otp(0x003f, 0x00); 	
	hi553_wirite_cmos_sensor_for_otp(0x0a00, 0x01); //stand by off	
}

bool otp_update_wb_sunwin_hi553(unsigned short golden_rg, unsigned short golden_bg) 
{
	
	kal_uint32 r_gain= 0x100,b_gain=0x100,g_gain=0x100;

	//r_gain = ((golden_rg/(((r_unit-64)/(g_unit-64))*0x200+0.5))*0x100);
	r_gain = (golden_rg*0x100)/r_unit;
	//b_gain = ((golden_bg/(((b_unit-64)/(g_unit-64))*0x200+0.5))*0x100);
	b_gain = (golden_bg*0x100)/b_unit;

	if(DEBUG_PRINT)
	printk("[Hi553 ]enter otp_update_wb_sunwin_hi553 r_gain=0x%x b_gain = 0x%x  \n",r_gain,b_gain);

	if((r_gain==0)||(b_gain==0))
		return 0;
	
	if(r_gain < b_gain)
	{
		if(r_gain < 0x100)
		{
			b_gain = 0x100*b_gain/r_gain;
			g_gain = 0x100*g_gain/r_gain;
			r_gain= 0x100;
		}
	}
	else
	{
		if(b_gain < 0x100)
		{
			r_gain = 0x100*r_gain/b_gain;
			g_gain = 0x100*g_gain/b_gain;
			b_gain= 0x100;
		}
	
	}
	hi553_otp_enable();
	hi553_wirite_cmos_sensor_for_otp(0x0126,(g_gain>>8)&0xff);	
	hi553_wirite_cmos_sensor_for_otp(0x0127,(g_gain)&0xff); 
	hi553_wirite_cmos_sensor_for_otp(0x0128,(g_gain>>8)&0xff);	
	hi553_wirite_cmos_sensor_for_otp(0x0129,(g_gain)&0xff); 
	
	hi553_wirite_cmos_sensor_for_otp(0x012a,(r_gain>>8)&0xff);	
	hi553_wirite_cmos_sensor_for_otp(0x012b,(r_gain)&0xff); 
	hi553_wirite_cmos_sensor_for_otp(0x012c,(b_gain>>8)&0xff);	
	hi553_wirite_cmos_sensor_for_otp(0x012d,(b_gain)&0xff); 

	hi553_otp_disable();
	return 0;
}

signed char hi553_get_chip_information(void)
{
	kal_uint8 otp_flag;
	kal_uint8 i;
	
	hi553_wirite_cmos_sensor_for_otp(0x010a,(0x0501>>8)&0xff);	
	hi553_wirite_cmos_sensor_for_otp(0x010b,(0x0501)&0xff); 
	hi553_wirite_cmos_sensor_for_otp(0x0102, 0x01); 
	otp_flag = hi553_read_cmos_sensor_for_otp(0x108);
	
	if(DEBUG_PRINT)
	printk("[Hi553 ]enter hi553_get_chip_information =0x%x \n",otp_flag);
	switch(otp_flag){
		case 1:			
			otp_flag = hi553_read_cmos_sensor_for_otp(0x108);
			break;
		case 0x13:
			hi553_wirite_cmos_sensor_for_otp(0x010a,(0x0513>>8)&0xff);	
			hi553_wirite_cmos_sensor_for_otp(0x010b,(0x0513)&0xff); 
			hi553_wirite_cmos_sensor_for_otp(0x0102, 0x01); 
			otp_flag = hi553_read_cmos_sensor_for_otp(0x108);
			break;
		case 0x37:
			hi553_wirite_cmos_sensor_for_otp(0x010a,(0x0524>>8)&0xff);	
			hi553_wirite_cmos_sensor_for_otp(0x010b,(0x0524)&0xff); 
			hi553_wirite_cmos_sensor_for_otp(0x0102, 0x01); 
			otp_flag = hi553_read_cmos_sensor_for_otp(0x108);
			break;
			
		default:
			return 0;
			break;
		}
	
	if(otp_flag == OTP_MID)
		return 1;
	else 
		return 0;
}
#define HI553_CALI_WB_DATA_LENGTH	29
signed char hi553_get_cali_wb(void)
{
	kal_uint8 otp_flag;
	kal_uint16 checksum;
	kal_uint8 i;
	kal_uint8 wb_data[HI553_CALI_WB_DATA_LENGTH];

	rg_ratio_unit = 0;
	bg_ratio_unit = 0;
	hi553_wirite_cmos_sensor_for_otp(0x010a,(0x0535>>8)&0xff);	
	hi553_wirite_cmos_sensor_for_otp(0x010b,(0x0535)&0xff); 
	hi553_wirite_cmos_sensor_for_otp(0x0102, 0x01); 
	otp_flag = hi553_read_cmos_sensor_for_otp(0x108);
	
	if(DEBUG_PRINT)
	printk("[Hi553 ]enter hi553_get_cali_wb =0x%x \n",otp_flag);
	checksum = 0;
	switch(otp_flag)
	{
		case 1:
			for(i = 0; i < HI553_CALI_WB_DATA_LENGTH; i++)
			{
				wb_data[i] = hi553_read_cmos_sensor_for_otp(0x108);
				
				if(DEBUG_PRINT)
				printk("[Hi553 ] hi553_get_cali_wb =0x%x \n",wb_data[i]);
				checksum += wb_data[i];
			}
			otp_flag = hi553_read_cmos_sensor_for_otp(0x108);
			checksum = checksum%0xff + 1;
			break;
			
		case 0x13:		
			hi553_wirite_cmos_sensor_for_otp(0x010a,(0x0554>>8)&0xff);	
			hi553_wirite_cmos_sensor_for_otp(0x010b,(0x0554)&0xff); 
			hi553_wirite_cmos_sensor_for_otp(0x0102, 0x01); 
			
			for(i = 0; i < HI553_CALI_WB_DATA_LENGTH; i++)
			{
				wb_data[i] = hi553_read_cmos_sensor_for_otp(0x108);
				
				if(DEBUG_PRINT)
				printk("[Hi553 ] hi553_get_cali_wb flag= 0x13 data =0x%x \n",wb_data[i]);
				checksum += wb_data[i];
			}
			otp_flag = hi553_read_cmos_sensor_for_otp(0x108);
			checksum = checksum%0xff + 1;
			break;

		case 0x37:		
			hi553_wirite_cmos_sensor_for_otp(0x010a,(0x0572>>8)&0xff);	
			hi553_wirite_cmos_sensor_for_otp(0x010b,(0x0572)&0xff); 
			hi553_wirite_cmos_sensor_for_otp(0x0102, 0x01); 
			
			for(i = 0; i < HI553_CALI_WB_DATA_LENGTH; i++)
			{
				wb_data[i] = hi553_read_cmos_sensor_for_otp(0x108);
				
				if(DEBUG_PRINT)
				printk("[Hi553 ] hi553_get_cali_wb flag= 0x37 data=0x%x \n",wb_data[i]);
				checksum += wb_data[i];
			}
			otp_flag = hi553_read_cmos_sensor_for_otp(0x108);
			checksum = checksum%0xff + 1;
		break;

		default:
			return 0;
			break;
		}

		
		r_unit = (wb_data[0]<<8)|(wb_data[1]);
		b_unit = (wb_data[2]<<8)|(wb_data[3]);
		g_unit = (wb_data[4]<<8)|(wb_data[5]);
		
		if(DEBUG_PRINT)
		printk("[Hi553 ] hi553_get_cali_wb r_unit=0x%x r_unit=0x%x r_unit=0x%x \n",r_unit,b_unit,g_unit);
		
		if(checksum == otp_flag)
		{
			return 1;
		}
		else
			return 0;

}
signed char check_MID_sunwin_hi553(bool bOnlyCheck)
{

		int addr, temp, i,MID_ID;
		int temp1;
		kal_uint8 result = 0;

		if(DEBUG_PRINT)
		printk("[Hi553 ]enter check_MID_sunwin_hi553g =0x%x \n",bOnlyCheck);

		
		hi553_otp_enable();
		
		//otp_flag = hi553_read_cmos_sensor_for_otp(0x0501);
		result = hi553_get_chip_information();

		if((result)&&(bOnlyCheck))
		{		
			result = hi553_get_cali_wb();
		}
		else if(result)
		{
			result = hi553_get_cali_wb();
		}
	
		hi553_otp_disable();
		

		return result;		 

		
	}



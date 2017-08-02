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
#include "s5k5e2ya_sunwinmipiraw_Sensor.h"

#define MODULE_ID_NULL			0x0000
#define MODULE_ID_s5k5e2ya_sunwin_liteon		0x0002    //xxx: sensor P/N;  yyy: module vendor
#define MODULE_ID_END			0xFFFF
#define LSC_PARAM_QTY 240
#define SENSOR_FAIL   -1
#define SENSOR_SUCCESS 0
struct otp_info_t {
	uint16_t flag;
	uint16_t module_id;
	uint16_t lens_id;
	uint16_t vcm_id;
	uint16_t vcm_driver_id;
	uint16_t year;
	uint16_t month;
	uint16_t day;
	uint16_t rg_ratio_current;
	uint16_t bg_ratio_current;
	uint16_t rg_ratio_typical;
	uint16_t bg_ratio_typical;
	uint16_t r_current;
	uint16_t g_current;
	uint16_t b_current;
	uint16_t r_typical;
	uint16_t g_typical;
	uint16_t b_typical;
	uint16_t vcm_dac_start;
	uint16_t vcm_dac_inifity;
	uint16_t vcm_dac_macro;
	uint16_t lsc_param[LSC_PARAM_QTY];
}Sunwin_S5K5E2_OTP;

#define OTP_AUTO_LOAD_LSC_s5k5e2ya_sunwin_liteon

#define RG_TYPICAL_s5k5e2ya_sunwin_liteon    0x0234
#define BG_TYPICAL_s5k5e2ya_sunwin_liteon		0x01CF
#define R_TYPICAL_s5k5e2ya_sunwin_liteon		0x0000
#define G_TYPICAL_s5k5e2ya_sunwin_liteon		0x0000
#define B_TYPICAL_s5k5e2ya_sunwin_liteon		0x0000

#define S5K5E2YA_SUNWINM_OTP_RG_MAX  0x25D  //0.78 x 1024  = 799
#define S5K5E2YA_SUNWINM_OTP_RG_MIN  0x207  //0.57 x 1024  = 584
#define S5K5E2YA_SUNWINM_OTP_BG_MAX  0x1F7  //0.68 x 1024  = 697
#define S5K5E2YA_SUNWINM_OTP_BG_MIN  0x1B4  //0.48 x 1024  = 492
#define LOG_TAG  "S5K5E2YA_SUNWIN:"
#define CAM_CAL_DEBUG
#ifdef CAM_CAL_DEBUG
#define SENSOR_PRINT(fmt,arg...)  printk(LOG_TAG "%s: " fmt "\n", __FUNCTION__ ,##arg)
#else
#define SENSOR_PRINT(fmt,arg...)
#endif
extern  void S5K5E2_SUNWIN_write_cmos_sensor(u16 addr, u32 para);
extern  kal_uint16 S5K5E2_SUNWIN_read_cmos_sensor(u32 addr);

uint32_t s5k5e2ya_sunwin_liteon_read_otp_info(struct otp_info_t *otp_info)
{
	uint32_t rtn = SENSOR_SUCCESS;
	uint16_t temp1,temp2,temp3;
	uint16_t infoMidTemp1,infoMidTemp2,infoMidTemp3,infoLensIDTemp1,infoLensIDTemp2,infoLensIDTemp3;
	uint16_t wbFlagTemp1,wbFlagTemp2,wbFlagTemp3,wbRGMsb1,wbRGMsb2,wbRGMsb3,wbRGLsb1,wbRGLsb2,wbRGLsb3;
	uint16_t wbBGMsb1,wbBGMsb2,wbBGMsb3,wbBGLsb1,wbBGLsb2,wbBGLsb3;
	//struct otp_info_t *otp_info=(struct otp_info_t *)param_ptr;
	otp_info->rg_ratio_typical=RG_TYPICAL_s5k5e2ya_sunwin_liteon;
	otp_info->bg_ratio_typical=BG_TYPICAL_s5k5e2ya_sunwin_liteon;
	otp_info->r_typical=R_TYPICAL_s5k5e2ya_sunwin_liteon;
	otp_info->g_typical=G_TYPICAL_s5k5e2ya_sunwin_liteon;
	otp_info->b_typical=B_TYPICAL_s5k5e2ya_sunwin_liteon;

	/*TODO*/
	//read lsc data. check moudule for debug
	{
		uint32_t i;
		uint32_t lsc_data_count = 0;
	    //Check lsc value is empty or valid
		S5K5E2_SUNWIN_write_cmos_sensor(0x0a00, 0x04); //make initial state
		S5K5E2_SUNWIN_write_cmos_sensor(0x0a02, 0x05); //page set
		S5K5E2_SUNWIN_write_cmos_sensor(0x0a00, 0x01); //otp enable read
		mdelay(5);
		
		temp1 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A04);
		temp2 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A05);
		temp3 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A06);
		otp_info->flag = temp1 | temp2 | temp3;
		if((otp_info->flag & 0x03) == 0x00)
		{
			SENSOR_PRINT("otp is invalid");
		}
		else if((otp_info->flag & 0x03) == 0x02)
		{
			int nStartAddr = 576;
			int nEndAddr = 936;
			S5K5E2_SUNWIN_write_cmos_sensor(0x0a00, 0x04);
			S5K5E2_SUNWIN_write_cmos_sensor(0x0a02, nStartAddr/64);
			S5K5E2_SUNWIN_write_cmos_sensor(0x0a00, 0x01);
			for(i = nStartAddr;i < nEndAddr; i++)
			{
				if(i%64 == 0)
				{
					S5K5E2_SUNWIN_write_cmos_sensor(0x0a00, 0x04);
					S5K5E2_SUNWIN_write_cmos_sensor(0x0a02, i/64);
					S5K5E2_SUNWIN_write_cmos_sensor(0x0a00, 0x01);
					mdelay(5);
					lsc_data_count += S5K5E2_SUNWIN_read_cmos_sensor(0x0A04 + i%64);	
				}
			}
			
		}
		S5K5E2_SUNWIN_write_cmos_sensor(0x0a00, 0x04); //make initial state
		S5K5E2_SUNWIN_write_cmos_sensor(0x0a00, 0x00); //disable enable read	
		if(lsc_data_count == 0) {
			SENSOR_PRINT("OTP LSC no data!\n");
			return rtn;
		}			
	}
	
    //Read module info	
	S5K5E2_SUNWIN_write_cmos_sensor(0x0a00, 0x04); //make initial state
	S5K5E2_SUNWIN_write_cmos_sensor(0x0a02, 0x05); //page 5,module info part in page 5
	S5K5E2_SUNWIN_write_cmos_sensor(0x0a00, 0x01); //otp enable read
	mdelay(5);
	
	temp1 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A0A);
	temp2 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A0B);
	temp3 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A0C);
	otp_info->flag = temp1 | temp2 | temp3;
	if(otp_info->flag == 0x00)
	{
		otp_info->module_id = 0x00;
		otp_info->lens_id = 0x00;
	}
	else if(otp_info->flag == 0x01)  // Module info group1
	{
		S5K5E2_SUNWIN_write_cmos_sensor(0x0a00, 0x04);
		S5K5E2_SUNWIN_write_cmos_sensor(0x0a02, 0x05);
		S5K5E2_SUNWIN_write_cmos_sensor(0x0a00, 0x01);
		infoMidTemp1 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A0F);
		infoLensIDTemp1 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A10);
		infoMidTemp2 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A1F);
		infoLensIDTemp2 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A20);
		infoMidTemp3 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A2F);
		infoLensIDTemp3 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A30);
		otp_info->module_id = infoMidTemp1 | infoMidTemp2 | infoMidTemp3;
		otp_info->lens_id = infoLensIDTemp1 | infoLensIDTemp2 | infoLensIDTemp3;
	}
	else if(otp_info->flag == 0x07)  //Module info group2
	{
		S5K5E2_SUNWIN_write_cmos_sensor(0x0a00, 0x04);
		S5K5E2_SUNWIN_write_cmos_sensor(0x0a02, 0x04);
		S5K5E2_SUNWIN_write_cmos_sensor(0x0a00, 0x01);
		infoMidTemp1 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A0F);
		infoLensIDTemp1 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A10);
		infoMidTemp2 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A1F);
		infoLensIDTemp2 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A20);
		infoMidTemp3 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A2F);
		infoLensIDTemp3 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A30);
		otp_info->module_id = infoMidTemp1 | infoMidTemp2 | infoMidTemp3;
		otp_info->lens_id = infoLensIDTemp1 | infoLensIDTemp2 | infoLensIDTemp3;
	}
	else
	{
		otp_info->module_id = 0xFF;
		otp_info->lens_id = 0xFF;
	}
	S5K5E2_SUNWIN_write_cmos_sensor(0x0a00, 0x04);
	S5K5E2_SUNWIN_write_cmos_sensor(0x0a00, 0x00);
	mdelay(5);
	
	//Read module awb 
	S5K5E2_SUNWIN_write_cmos_sensor(0x0a00, 0x04);
	S5K5E2_SUNWIN_write_cmos_sensor(0x0a02, 0x05);
	S5K5E2_SUNWIN_write_cmos_sensor(0x0a00, 0x01);
	mdelay(5);
	wbFlagTemp1 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A07);
	wbFlagTemp2 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A08);
	wbFlagTemp3 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A09);
	otp_info->flag = wbFlagTemp1 | wbFlagTemp2 | wbFlagTemp3;
	if(otp_info->flag == 0x00)
	{
		otp_info->rg_ratio_current = 0x00;
		otp_info->bg_ratio_current = 0x00;
	}
	else if(otp_info->flag == 0x01)  //group1
	{
		S5K5E2_SUNWIN_write_cmos_sensor(0x0a00, 0x04);
		S5K5E2_SUNWIN_write_cmos_sensor(0x0a02, 0x05);
		S5K5E2_SUNWIN_write_cmos_sensor(0x0a00, 0x01);
		wbRGMsb1 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A14);
		wbRGLsb1 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A15);
		wbBGMsb1 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A16);
		wbBGLsb1 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A17);
		wbRGMsb2 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A24);
		wbRGLsb2 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A25);
		wbBGMsb2 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A26);
		wbBGLsb2 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A27);
		wbRGMsb3 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A34);
		wbRGLsb3 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A35);
		wbBGMsb3 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A36);
		wbBGLsb3 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A37);
		otp_info->rg_ratio_current = (((wbRGMsb1 | wbRGMsb2 | wbRGMsb3) << 8) & 0xFF00) | ((wbRGLsb1 | wbRGLsb2 | wbRGLsb3) & 0xFF);
		otp_info->bg_ratio_current = (((wbBGMsb1 | wbBGMsb2 | wbBGMsb3) << 8) & 0xFF00) | ((wbBGLsb1 | wbBGLsb2 | wbBGLsb3) & 0xFF);	
	}
	else if(otp_info->flag == 0x07)  //group2
	{
		S5K5E2_SUNWIN_write_cmos_sensor(0x0a00, 0x04);
		S5K5E2_SUNWIN_write_cmos_sensor(0x0a02, 0x04);
		S5K5E2_SUNWIN_write_cmos_sensor(0x0a00, 0x01);
		wbRGMsb1 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A14);
		wbRGLsb1 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A15);
		wbBGMsb1 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A16);
		wbBGLsb1 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A17);
		wbRGMsb2 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A24);
		wbRGLsb2 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A25);
		wbBGMsb2 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A26);
		wbBGLsb2 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A27);
		wbRGMsb3 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A34);
		wbRGLsb3 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A35);
		wbBGMsb3 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A36);
		wbBGLsb3 = S5K5E2_SUNWIN_read_cmos_sensor(0x0A37);
		otp_info->rg_ratio_current = (((wbRGMsb1 | wbRGMsb2 | wbRGMsb3) << 8) & 0xFF00) | ((wbRGLsb1 | wbRGLsb2 | wbRGLsb3) & 0xFF);
		otp_info->bg_ratio_current = (((wbBGMsb1 | wbBGMsb2 | wbBGMsb3) << 8) & 0xFF00) | ((wbBGLsb1 | wbBGLsb2 | wbBGLsb3) & 0xFF);	
	}
	else
	{
		otp_info->rg_ratio_current = 0xFF;
		otp_info->bg_ratio_current = 0xFF;
	}
	S5K5E2_SUNWIN_write_cmos_sensor(0x0a00, 0x04);
	S5K5E2_SUNWIN_write_cmos_sensor(0x0a00, 0x00);
	
	/*print otp information*/
	SENSOR_PRINT("flag=0x%x",otp_info->flag);
	SENSOR_PRINT("module_id=0x%x",otp_info->module_id);
	SENSOR_PRINT("lens_id=0x%x",otp_info->lens_id);
	SENSOR_PRINT("vcm_id=0x%x",otp_info->vcm_id);
	SENSOR_PRINT("vcm_id=0x%x",otp_info->vcm_id);
	SENSOR_PRINT("vcm_driver_id=0x%x",otp_info->vcm_driver_id);
	SENSOR_PRINT("data=%d-%d-%d",otp_info->year,otp_info->month,otp_info->day);
	SENSOR_PRINT("rg_ratio_current=0x%x",otp_info->rg_ratio_current);
 	SENSOR_PRINT("bg_ratio_current=0x%x",otp_info->bg_ratio_current);
	SENSOR_PRINT("rg_ratio_typical=0x%x",otp_info->rg_ratio_typical);
	SENSOR_PRINT("bg_ratio_typical=0x%x",otp_info->bg_ratio_typical);
	SENSOR_PRINT("r_current=0x%x",otp_info->r_current);
	SENSOR_PRINT("g_current=0x%x",otp_info->g_current);
	SENSOR_PRINT("b_current=0x%x",otp_info->b_current);
	SENSOR_PRINT("r_typical=0x%x",otp_info->r_typical);
	SENSOR_PRINT("g_typical=0x%x",otp_info->g_typical);
	SENSOR_PRINT("b_typical=0x%x",otp_info->b_typical);
	SENSOR_PRINT("vcm_dac_start=0x%x",otp_info->vcm_dac_start);
	SENSOR_PRINT("vcm_dac_inifity=0x%x",otp_info->vcm_dac_inifity);
	SENSOR_PRINT("vcm_dac_macro=0x%x",otp_info->vcm_dac_macro);
	return rtn;
	
}
uint32_t s5k5e2ya_sunwin_liteon_update_awb(struct otp_info_t *otp_info)
{
	uint32_t rtn = SENSOR_SUCCESS;
	//struct otp_info_t *otp_info=(struct otp_info_t *)param_ptr;

	/*TODO*/
	uint16_t stream_value = 0;
	uint32_t g_gain=0,r_gain=0,b_gain = 0,g_gain_b ,g_gain_r;

	//calculate R,G,B gain
	if(otp_info->bg_ratio_current < otp_info->bg_ratio_typical){
		if(otp_info->rg_ratio_current< otp_info->rg_ratio_typical){
			g_gain= 0x100;
			b_gain = 0x100 * otp_info->bg_ratio_typical / otp_info->bg_ratio_current;
			r_gain = 0x100 * otp_info->rg_ratio_typical / otp_info->rg_ratio_current;
		}
		else{
	        r_gain = 0x100;
			g_gain = 0x100 * otp_info->rg_ratio_current / otp_info->rg_ratio_typical;
			b_gain = g_gain * otp_info->bg_ratio_typical / otp_info->bg_ratio_current;	        
		}
	}
	else{
		if(otp_info->rg_ratio_current < otp_info->rg_ratio_typical){
	        b_gain = 0x100;
			g_gain = 0x100 * otp_info->bg_ratio_current / otp_info->bg_ratio_typical;
			r_gain = g_gain * otp_info->rg_ratio_typical / otp_info->rg_ratio_current;
		}
		else{
	       	g_gain_b = 0x100*otp_info->bg_ratio_current / otp_info->bg_ratio_typical;
		    g_gain_r = 0x100*otp_info->rg_ratio_current / otp_info->rg_ratio_typical;
			
			if(g_gain_b > g_gain_r)	{
				b_gain = 0x100;
				g_gain = g_gain_b;
				r_gain = g_gain * otp_info->rg_ratio_typical / otp_info->rg_ratio_current;
			}
			else	{
				r_gain = 0x100;
				g_gain = g_gain_r;
				b_gain= g_gain * otp_info->bg_ratio_typical / otp_info->bg_ratio_current;
			}        
		}	
	}

	//write to register
	SENSOR_PRINT("r_Gain=0x%x\n", r_gain);	
	SENSOR_PRINT("g_Gain=0x%x\n", g_gain);	
	SENSOR_PRINT("b_Gain=0x%x\n", b_gain);	

	S5K5E2_SUNWIN_write_cmos_sensor(0x020e, (g_gain&0xff00)>>8);
	S5K5E2_SUNWIN_write_cmos_sensor(0x020f, g_gain&0xff);
	S5K5E2_SUNWIN_write_cmos_sensor(0x0210, (r_gain&0xff00)>>8);
	S5K5E2_SUNWIN_write_cmos_sensor(0x0211, r_gain&0xff);
	S5K5E2_SUNWIN_write_cmos_sensor(0x0212, (b_gain&0xff00)>>8);
	S5K5E2_SUNWIN_write_cmos_sensor(0x0213, b_gain&0xff);
	S5K5E2_SUNWIN_write_cmos_sensor(0x0214, (g_gain&0xff00)>>8);
	S5K5E2_SUNWIN_write_cmos_sensor(0x0215, g_gain&0xff);
	
	return SENSOR_SUCCESS;
}

#ifndef OTP_AUTO_LOAD_LSC_s5k5e2ya_sunwin_liteon

static uint32_t s5k5e2ya_sunwin_liteon_update_lsc(void *param_ptr)
{
	uint32_t rtn = SENSOR_SUCCESS;
	struct otp_info_t *otp_info=(struct otp_info_t *)param_ptr;

	/*TODO*/
	
	return rtn;
}

#endif
static uint32_t s5k5e2ya_sunwin_liteon_update_otp(struct otp_info_t *otp_info)
{
	uint32_t rtn = SENSOR_SUCCESS;
	//struct otp_info_t *otp_info=(struct otp_info_t *)param_ptr;

	rtn=s5k5e2ya_sunwin_liteon_update_awb(otp_info);
	if(rtn!=SENSOR_SUCCESS)
	{
		SENSOR_PRINT("OTP awb appliy error!");
		return rtn;
	}

	#ifndef OTP_AUTO_LOAD_LSC_s5k5e2ya_sunwin_liteon
	
	rtn=s5k5e2ya_sunwin_liteon_update_lsc(otp_info);
	if(rtn!=SENSOR_SUCCESS)
	{
		SENSOR_PRINT("OTP lsc appliy error!");
		return rtn;
	}
	#endif
	
	return rtn;
}

static uint32_t s5k5e2ya_sunwin_liteon_identify_otp(struct otp_info_t *otp_info)
{
	uint32_t rtn = SENSOR_SUCCESS;

	rtn=s5k5e2ya_sunwin_liteon_read_otp_info(otp_info);
	SENSOR_PRINT("rtn=%d",rtn);

	return rtn;
}


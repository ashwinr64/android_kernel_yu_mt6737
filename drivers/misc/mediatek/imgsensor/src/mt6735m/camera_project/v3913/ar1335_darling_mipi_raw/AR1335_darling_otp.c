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

#include "ar1335mipi_darling_Sensor.h"

#define PFX "AR1335_darling_otp"
#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)

#define i2c_write_id  0x6c
struct Darling_AR1335_otp_struct {
    u8 flag;
    u8 MID;
    int Year;
    u8 Month;
    u8 Day;
    u8 LID;
    int RGr_ratio;
    int BGr_ratio;
    int GrGb_ratio;
    int VCM_Infinity;
    int VCM_Macro;
    int LSC_table[115];
}AR1335_OTP;
 
#define RGr_ratio_Typical 512
#define BGr_ratio_Typical 512
#define GrGb_ratio_Typical 1024

//Darling_AR1335_OTP *AR1335_OTP;
u16 Darling_AR1335_OTP_Data[9] = {0};

static kal_uint16 AR1335_read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
	kal_uint16 tmp = 0;

    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
    iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 2, i2c_write_id);

	tmp = get_byte >> 8;
	get_byte = ((get_byte & 0x00ff) << 8) | tmp; 

    return get_byte;
}

static void AR1335_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para >> 8), (char)(para & 0xFF)};
    iWriteRegI2C(pu_send_cmd, 4, i2c_write_id);
}
bool is_darling_camera(void)
{
        u16 Val_Info = 0;
        u8 module_id;
      AR1335_write_cmos_sensor(0x301A,0x0001);//reset
    mdelay(10);
    AR1335_write_cmos_sensor(0x301A,0x0218);//disable streaming
    AR1335_write_cmos_sensor(0x304C,0x3000);//choose to only read Record Type 0x30
    AR1335_write_cmos_sensor(0x3054,0x0400);//Enable read OTP buffer registers
    AR1335_write_cmos_sensor(0x304A,0x0210);//Auto read star
    Val_Info = AR1335_read_cmos_sensor(0x3800);
    LOG_INF("Val_Info = %d",Val_Info);
    if(Val_Info== 0x4000){module_id = AR1335_read_cmos_sensor(0x3802)>>8;}
    else if(Val_Info== 0xD000){module_id = AR1335_read_cmos_sensor(0x380A)>>8;}
    else if(Val_Info== 0xF400){module_id = AR1335_read_cmos_sensor(0x3812)>>8;}   
    else if(Val_Info== 0x00){module_id = 0;}
    if(0x5 == module_id)//darling = 0x05
    {
        return 1;
    }
    return 0;
}
void Darling_AR1335_read_OTP(void)
{
    AR1335_write_cmos_sensor(0x301A,0x0001);//reset
    mdelay(10);
    AR1335_write_cmos_sensor(0x301A,0x0218);//disable streaming
    AR1335_write_cmos_sensor(0x304C,0x3081);//choose to only read Record Type 0x30
    AR1335_write_cmos_sensor(0x3054,0x0400);//Enable read OTP buffer registers
    AR1335_write_cmos_sensor(0x304A,0x0210);//Auto read star
    int i;

    u16 Val_Info = 0;
    Val_Info=AR1335_read_cmos_sensor(0x3800);//flag of info
    LOG_INF("Val_Info = 0x%x",Val_Info); 
    if(Val_Info== 0x4000)
    {
        AR1335_OTP.flag = 0x01;
        AR1335_OTP.MID = AR1335_read_cmos_sensor(0x3802)>>8;
        AR1335_OTP.LID = AR1335_read_cmos_sensor(0x3802)&0xFF;
        AR1335_OTP.Year= AR1335_read_cmos_sensor(0x3804);
        AR1335_OTP.Month= AR1335_read_cmos_sensor(0x3806)>>8;
        AR1335_OTP.Day= AR1335_read_cmos_sensor(0x3806)&0xFF;
    }
    else if(Val_Info== 0xD000)
//    else if(Val_Info== 0x40)
    {
        AR1335_OTP.flag = 0x01;
        AR1335_OTP.MID = AR1335_read_cmos_sensor(0x380A)>>8;
        AR1335_OTP.LID = AR1335_read_cmos_sensor(0x380A)&0xFF;
        AR1335_OTP.Year= AR1335_read_cmos_sensor(0x380C);
        AR1335_OTP.Month= AR1335_read_cmos_sensor(0x380E)>>8;
        AR1335_OTP.Day= AR1335_read_cmos_sensor(0x380E)&0xFF;
    }   
    else if(Val_Info== 0xF400)
    {
        AR1335_OTP.flag = 0x01;
        AR1335_OTP.MID = AR1335_read_cmos_sensor(0x3812)>>8;
        AR1335_OTP.LID = AR1335_read_cmos_sensor(0x3812)&0xFF;
        AR1335_OTP.Year= AR1335_read_cmos_sensor(0x3814);
        AR1335_OTP.Month= AR1335_read_cmos_sensor(0x3816)>>8;
        AR1335_OTP.Day= AR1335_read_cmos_sensor(0x3816)&0xFF;
    }
    else if(Val_Info== 0x00)
    {
        AR1335_OTP.flag=0x00;
        AR1335_OTP.MID =0x00;
        AR1335_OTP.Year=0x00;
        AR1335_OTP.Month=0x00;
        AR1335_OTP.Day=0x00;
        AR1335_OTP.LID = 0x00;
    }
    LOG_INF("AR1335_OTP.MID = %d",AR1335_OTP.MID);
    LOG_INF("Year = %d,Month = %d,Day = %d",AR1335_OTP.Year,AR1335_OTP.Month,AR1335_OTP.Day);
    u16 Val_AF = 0;
    u16 Val_LSC = 0;
    u16 Val_AWB = 0;
//    Val_AF=AR1335_read_cmos_sensor(0x380A);//flag of AF
//    Val_LSC=AR1335_read_cmos_sensor(0x380C);//flag of LSC
//    Val_AWB=AR1335_read_cmos_sensor(0x380E);//flag of AWB
    Val_AF=AR1335_read_cmos_sensor(0x381A);//flag of AF
    Val_LSC=AR1335_read_cmos_sensor(0x381C);//flag of LSC
    Val_AWB=AR1335_read_cmos_sensor(0x381E);//flag of AWB
    LOG_INF("Val_AF = 0x%x,Val_LSC = 0x%x,Val_AWB = 0x%x",Val_AF,Val_LSC,Val_AWB);
    if(Val_AF== 0x4000)
    {
        AR1335_write_cmos_sensor(0x304C,0x3100);//choose to only read Record Type 0x31
        AR1335_write_cmos_sensor(0x3054,0x0400);//Enable read OTP buffer registers
        AR1335_write_cmos_sensor(0x304A,0x0210);//Auto read star
        
        AR1335_OTP.VCM_Infinity = AR1335_read_cmos_sensor(0x3800);
        AR1335_OTP.VCM_Macro = AR1335_read_cmos_sensor(0x3802);
          AR1335_OTP.flag +=(1<<1);
    }
    else if(Val_AF== 0xD000)
    {
        AR1335_write_cmos_sensor(0x304C,0x3200);//choose to only read Record Type 0x32
        AR1335_write_cmos_sensor(0x3054,0x0400);//Enable read OTP buffer registers
        AR1335_write_cmos_sensor(0x304A,0x0210);//Auto read star
        
        AR1335_OTP.VCM_Infinity = AR1335_read_cmos_sensor(0x3800);
        AR1335_OTP.VCM_Macro = AR1335_read_cmos_sensor(0x3802);
        AR1335_OTP.flag +=(1<<1);
    }
    else if(Val_AF== 0xF400)
    {
        AR1335_write_cmos_sensor(0x304C,0x3300);//choose to only read Record Type 0x33
        AR1335_write_cmos_sensor(0x3054,0x0400);//Enable read OTP buffer registers
        AR1335_write_cmos_sensor(0x304A,0x0210);//Auto read star

        AR1335_OTP.VCM_Infinity = AR1335_read_cmos_sensor(0x3800);
        AR1335_OTP.VCM_Macro = AR1335_read_cmos_sensor(0x3802);
        AR1335_OTP.flag +=(1<<1);
    }
    else
    {
        AR1335_OTP.VCM_Infinity = 0x00;
        AR1335_OTP.VCM_Macro = 0x00;
    }
    LOG_INF("VCM_Infinity =%d,VCM_Macro = %d",AR1335_OTP.VCM_Infinity,AR1335_OTP.VCM_Macro);
    if(Val_AWB== 0x4000)
    {
        AR1335_write_cmos_sensor(0x304C,0x3700);//choose to only read Record Type 0x37
        AR1335_write_cmos_sensor(0x3054,0x0400);//Enable read OTP buffer registers
        AR1335_write_cmos_sensor(0x304A,0x0210);//Auto read star

        AR1335_OTP.RGr_ratio = AR1335_read_cmos_sensor(0x3800);
        AR1335_OTP.BGr_ratio = AR1335_read_cmos_sensor(0x3802);
        AR1335_OTP.GrGb_ratio =AR1335_read_cmos_sensor(0x3804);
        AR1335_OTP.flag +=(1<<2);
    }
    else if(Val_AWB== 0xD000)
    {
        AR1335_write_cmos_sensor(0x304C,0x3800);//choose to only read Record Type 0x38
        AR1335_write_cmos_sensor(0x3054,0x0400);//Enable read OTP buffer registers
        AR1335_write_cmos_sensor(0x304A,0x0210);//Auto read star

        AR1335_OTP.RGr_ratio = AR1335_read_cmos_sensor(0x3800);
        AR1335_OTP.BGr_ratio = AR1335_read_cmos_sensor(0x3802);
        AR1335_OTP.GrGb_ratio =AR1335_read_cmos_sensor(0x3804);
        AR1335_OTP.flag +=(1<<2);
    }
    else if(Val_AWB== 0xF400)
    {
        AR1335_write_cmos_sensor(0x304C,0x3900);//choose to only read Record Type 0x39
        AR1335_write_cmos_sensor(0x3054,0x0400);//Enable read OTP buffer registers
        AR1335_write_cmos_sensor(0x304A,0x0210);//Auto read star

        AR1335_OTP.RGr_ratio = AR1335_read_cmos_sensor(0x3800);
        AR1335_OTP.BGr_ratio = AR1335_read_cmos_sensor(0x3802);
        AR1335_OTP.GrGb_ratio =AR1335_read_cmos_sensor(0x3804);
        AR1335_OTP.flag +=(1<<2);
    }
    else
    {
        AR1335_OTP.RGr_ratio = 0x00;
        AR1335_OTP.BGr_ratio = 0x00;
        AR1335_OTP.GrGb_ratio =0x00;
    }
    LOG_INF("RGr_ratio = %d,BGr_ratio =%d,GrGb_ratio=%d",AR1335_OTP.RGr_ratio,AR1335_OTP.BGr_ratio,AR1335_OTP.GrGb_ratio);
#if 1
    if(Val_LSC== 0x4000)
    {
        AR1335_write_cmos_sensor(0x304C,0x3400);//choose to only read Record Type 0x34
        AR1335_write_cmos_sensor(0x3054,0x0400);//Enable read OTP buffer registers
        AR1335_write_cmos_sensor(0x304A,0x0210);//Auto read star

        for(i = 0;i<115;i++)
        {
            AR1335_OTP.LSC_table[i] = AR1335_read_cmos_sensor(0x3800+i);
        }
            AR1335_OTP.flag +=(1<<3);
   }
   else if(Val_LSC== 0xD000)
   {
        AR1335_write_cmos_sensor(0x304C,0x3500);//choose to only read Record Type 0x35
        AR1335_write_cmos_sensor(0x3054,0x0400);//Enable read OTP buffer registers
        AR1335_write_cmos_sensor(0x304A,0x0210);//Auto read star

       for(i = 0;i<115;i++)
       {
           AR1335_OTP.LSC_table[i] = AR1335_read_cmos_sensor(0x3800+i);
       }
         AR1335_OTP.flag +=(1<<3);
  }
  else if(Val_LSC== 0xF400)
  {
        AR1335_write_cmos_sensor(0x304C,0x3600);//choose to only read Record Type 0x36
        AR1335_write_cmos_sensor(0x3054,0x0400);//Enable read OTP buffer registers
        AR1335_write_cmos_sensor(0x304A,0x0210);//Auto read star
        for(i = 0;i<115;i++)
        {
             AR1335_OTP.LSC_table[i] = AR1335_read_cmos_sensor(0x3800+i);
        }
        AR1335_OTP.flag +=(1<<3);
  }
  else{
        for(i = 0;i<115;i++)
        {
        AR1335_OTP.LSC_table[i] = 0x00;
        }
    }
#endif
    LOG_INF("TINNO_CAMERA_HAL_DRIVER_LOG_2A_OTP_DATA.DARLING_AR1335.RGr_ratio = %d,BGr_ratio=%d,GrGb_ratio = %d\n",AR1335_OTP.RGr_ratio,AR1335_OTP.BGr_ratio,AR1335_OTP.GrGb_ratio);
    LOG_INF("TINNO_CAMERA_HAL_DRIVER_LOG_2A_OTP_DATA.DARLING_AR1335.RGr_ratio_Typical = %d,BGr_ratio_Typical=%d,GrGb_ratio_Typical = %d\n",RGr_ratio_Typical,BGr_ratio_Typical,GrGb_ratio_Typical);
    LOG_INF("TINNO_CAMERA_HAL_DRIVER_LOG_2A_OTP_DATA.DARLING_AR1335.VCM_Infinity =%d,VCM_Macro = %d\n",AR1335_OTP.VCM_Infinity,AR1335_OTP.VCM_Macro);
   Darling_AR1335_OTP_Data[0] = AR1335_OTP.MID;
   Darling_AR1335_OTP_Data[1] = AR1335_OTP.RGr_ratio;
   Darling_AR1335_OTP_Data[2] = AR1335_OTP.BGr_ratio;
   Darling_AR1335_OTP_Data[3] = AR1335_OTP.GrGb_ratio;
   Darling_AR1335_OTP_Data[4] = RGr_ratio_Typical;
   Darling_AR1335_OTP_Data[5] = BGr_ratio_Typical;
   Darling_AR1335_OTP_Data[6] = GrGb_ratio_Typical;
   Darling_AR1335_OTP_Data[7] = AR1335_OTP.VCM_Infinity;
   Darling_AR1335_OTP_Data[8] = AR1335_OTP.VCM_Macro; 
}

void Darling_OTP_AUTO_LOAD_LSC(void)
{
        LOG_INF("TINNO_CAMERA_HAL_DRIVER_LOG.DARLING_AR1335.AUTO_LOAD_LSC\n");
        kal_uint16 temp;
	//Select_Type(0x11);
	// enable sc
	temp = AR1335_read_cmos_sensor(0x3780);  
	AR1335_write_cmos_sensor(0x3780,temp|0x8000);  
//        Write_cmos_sensor(0x3780,0x8000); 
}
void Darling_AR1335_apply_OTP(void)
{
    if(((AR1335_OTP.flag>>3)&0x01) == 0x01)
    {
        int i=0;
        for(i=0;i<20;i++) AR1335_write_cmos_sensor(0x3600+i*2,AR1335_OTP.LSC_table[i+2]);
        for(i=0;i<20;i++) AR1335_write_cmos_sensor(0x3640+i*2,AR1335_OTP.LSC_table[i+23]);
        for(i=0;i<20;i++) AR1335_write_cmos_sensor(0x3680+i*2,AR1335_OTP.LSC_table[i+44]);
        for(i=0;i<20;i++) AR1335_write_cmos_sensor(0x36c0+i*2,AR1335_OTP.LSC_table[i+65]);
        for(i=0;i<20;i++) AR1335_write_cmos_sensor(0x3700+i*2,AR1335_OTP.LSC_table[i+86]);
        for(i=0;i<2;i++) AR1335_write_cmos_sensor(0x3782+i*2,AR1335_OTP.LSC_table[i+107]);
        for(i=0;i<4;i++) AR1335_write_cmos_sensor(0x37C0+i*2,AR1335_OTP.LSC_table[i+110]);
    }
}




















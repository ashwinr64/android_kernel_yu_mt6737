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

#include "s5k3m2mipi_Sensor.h"

#define PFX "s5k2m2_sunwin_otp"
#define S5K3M2_DEBUG
#ifdef S5K3M2_DEBUG
       // #define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)
    #define S5K3M2_DB(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)
#else
	#define S5K3M2_DB(fmt, arg...)
#endif

//S5K3M2 OTP Driver
#define OTP_GROUP_EMPTY                 0x00
#define OTP_GROUP_VALID                 0x01

#define S5K3M2_I2C_SPEED                300//300 KHz
#define S5K3M2_I2C_WRITE_ID             0x5A

kal_uint8 MID=0;
#define TRULY_ID 2

static kal_uint16 R_Gr_Ratio=0; 
static kal_uint16 B_Gr_Ratio=0;
//static kal_uint16 Gb_Gr_Ratio=0;
#define  RG_Ratio_typical  0x118		//goldSample典型值,在read_otp函数中根据custom ID进行赋值
#define  BG_Ratio_typical   0x100

#define GAIN_DEFAULT       0x0100
#define GAIN_GREEN1_ADDR   0x020E
#define GAIN_BLUE_ADDR     0x0212
#define GAIN_RED_ADDR      0x0210
#define GAIN_GREEN2_ADDR   0x0214

#define FlagOfAFC 0xA25
#define FlagOfAWB 0xA2E

static kal_uint16 otp_read(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    kdSetI2CSpeed(S5K3M2_I2C_SPEED);
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 2, S5K3M2_I2C_WRITE_ID);
    return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}
static void otp_write(kal_uint16 addr, kal_uint16 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
    kdSetI2CSpeed(S5K3M2_I2C_SPEED);  
    iWriteRegI2C(pusendcmd , 4, S5K3M2_I2C_WRITE_ID);
}

static kal_uint8 otp_read_8(kal_uint32 addr)
{
    kal_uint8 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    kdSetI2CSpeed(S5K3M2_I2C_SPEED);    
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte, 1, S5K3M2_I2C_WRITE_ID);
    return get_byte;
}

static void otp_write_8(kal_uint16 addr, kal_uint8 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
    kdSetI2CSpeed(S5K3M2_I2C_SPEED);    
    iWriteRegI2C(pusendcmd , 3, S5K3M2_I2C_WRITE_ID);
}

/*************************************************************************************************
* Function    :  start_read_otp
* Description :  before read otp , set the reading block setting  
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x15
* Return      :  0, reading block setting err
                 1, reading block setting ok 
**************************************************************************************************/
static kal_bool start_read_otp(void)
{
	kal_uint8 val = 0;
	kal_uint16 i;
    
    otp_write(0x0136, 0x1800);//External CLK 24MHz......Follow setting according to diff CLK
    otp_write(0x0304, 0x0006);
    otp_write(0x0306, 0x006E);//1120 0x0064 //1117 //0x0073
    otp_write(0x030c, 0x0004);
    otp_write(0x030e, 0x006A); //1117 //0x0064
    otp_write(0x0302, 0x0001);
    otp_write(0x0300, 0x0004);
    otp_write(0x030a, 0x0001);
    otp_write(0x0308, 0x0008);
    
    otp_write(0x0100, 0x0100);//STREAM ON ??
  //  otp_write_8(0x0100, 0x01);
    mdelay(10);
    otp_write_8(0x0A02, 0x1F);   //Set page of OTP (0 ~ 0x15) ... page 31(Dec)
    otp_write(0x0A00, 0x0100);  //Read Enable
    mdelay(2);
    return 1;
}


void stop_read_otp(void)
{
    //otp_write(0x0A00, 0x04);//make initial state
	otp_write(0x0A00, 0x00);//Disable NVM controller
}


kal_uint8 get_otp_flag(kal_uint16 addr, kal_uint8 groupIdx)
{
    kal_uint8 flag_AWB;
    
//    start_read_otp();
    flag_AWB = otp_read_8(addr);
//    stop_read_otp();
    S5K3M2_DB("flag_AWB = %d",flag_AWB);
    if(groupIdx == 0){return (flag_AWB&0xC0)>>6;}
    else if(groupIdx == 1)
    {
        return ((flag_AWB&0x30)>>4);
    }
}

kal_bool otp_lsc_update(void) //lsc
{
    
    S5K3M2_DB("TINNO_CAMERA_HAL_DRIVER_LOG.otp_lsc_update\n");
    #if 0
    otp_write(0x6028,0x2000);            
    otp_write(0x602A,0x14FA);            
    otp_write_8(0x6F12,0x0F);            
    otp_write(0x6028,0x4000);            
    otp_write(0x602A,0x306A);            
    otp_write(0x6F12,0x0068);            
    otp_write(0x602A,0x0B00);            
    otp_write_8(0x6F12,0x01);            
    otp_write(0x602A,0x3058);            
    otp_write(0x6F12,0x0900);//otp reinit
    #endif
    
    #if 1
    otp_write(0x0100,0x0000); 
    otp_write(0x6028,0x2000); 
    otp_write(0x602a,0x14fa); 
    otp_write(0x6f12,0x0f00); 
    otp_write(0x6028,0x4000); 
    otp_write(0x306a,0x0068); 
    otp_write(0x0b00,0x0100); 
    otp_write(0x3058,0x0900);
    
    otp_write(0x0100,0x0100);     
    #endif

    return 1;
}

kal_bool otp_wb_update(void)
{
    kal_uint16 R_GAIN;
    kal_uint16 B_GAIN;
    kal_uint16 G_GAIN;
    kal_uint16 G_gain_R;
    kal_uint16 G_gain_B;
    S5K3M2_DB("TINNO_CAMERA_HAL_DRIVER_LOG.S5K3M2.otp_wb_update");
   S5K3M2_DB("TINNO_CAMERA_HAL_DRIVER_LOG.S5K3M2_READ_WB_DATA R.Gr_Ratio = %d,B_Gr_Ratio =%d",R_Gr_Ratio,B_Gr_Ratio);
    printk("S5K3M2 zcw++: OTP WB Update Start: \n"); 
    if(!R_Gr_Ratio || !B_Gr_Ratio||!BG_Ratio_typical ||!RG_Ratio_typical)
    {
        printk("S5K3M2 zcw++: OTP WB ratio Data Err!\n");
        return 0;
    }
    if(B_Gr_Ratio < BG_Ratio_typical)
    {
        if(R_Gr_Ratio < RG_Ratio_typical)
        {
            G_GAIN = GAIN_DEFAULT;
            B_GAIN = GAIN_DEFAULT * BG_Ratio_typical / B_Gr_Ratio;
            R_GAIN = GAIN_DEFAULT * RG_Ratio_typical / R_Gr_Ratio;
        }
	else
        {
	     R_GAIN = GAIN_DEFAULT;
            G_GAIN = GAIN_DEFAULT * R_Gr_Ratio / RG_Ratio_typical;
            B_GAIN = G_GAIN * BG_Ratio_typical / B_Gr_Ratio;	        
        }
	}
    else
    {
        if(R_Gr_Ratio < RG_Ratio_typical)
        {
            B_GAIN = GAIN_DEFAULT;
            G_GAIN = GAIN_DEFAULT * B_Gr_Ratio / BG_Ratio_typical;
            R_GAIN = G_GAIN * RG_Ratio_typical / R_Gr_Ratio;
        }
        else
        {
            G_gain_B = GAIN_DEFAULT* B_Gr_Ratio / BG_Ratio_typical;
            G_gain_R = GAIN_DEFAULT* R_Gr_Ratio / RG_Ratio_typical;
            
            if(G_gain_B > G_gain_R)
            {
                B_GAIN = GAIN_DEFAULT;
                G_GAIN = G_gain_B;
                R_GAIN = G_GAIN * RG_Ratio_typical / R_Gr_Ratio;
            }
            else
            {
                R_GAIN = GAIN_DEFAULT;
                G_GAIN = G_gain_R;
                B_GAIN = G_GAIN * BG_Ratio_typical / B_Gr_Ratio;
            }	        
        }		
    }
    if(g_AWB_Value.R_G != 0)
    {
           R_GAIN = g_AWB_Value.R_G;           
    }
    if(g_AWB_Value.B_G !=0)
    {
          B_GAIN = g_AWB_Value.B_G;        
    }
    S5K3M2_DB("TINNO_CAMERA_HAL_DRIVER_LOG_GAIN.S5K3M2 : [R_GAIN=%d],[G_GAIN=%d],[B_GAIN=%d] \n",R_GAIN, G_GAIN, B_GAIN);
    otp_write(GAIN_RED_ADDR, R_GAIN);      
    otp_write(GAIN_BLUE_ADDR, B_GAIN);     
    otp_write(GAIN_GREEN1_ADDR, G_GAIN); //Green 1 default gain 1x     
    otp_write(GAIN_GREEN2_ADDR, G_GAIN); //Green 2 default gain 1x

    otp_write_8(0x3056, 0x01);
    printk("S5K3M2 zcw++: OTP WB Update Finished!  Write 0x3056=1 \n");    
    return 1;
}


int  S5K3M2_vcm_data[2];
int Read_S5K3m2_OTP_Data(void)
{
    int groupIdx;
   start_read_otp();
   for(groupIdx=0;groupIdx<2;groupIdx++)
    {
       if(get_otp_flag(FlagOfAWB,groupIdx) == 0x01)
       {
            R_Gr_Ratio = otp_read(0x0A2F + groupIdx*4);
            B_Gr_Ratio = otp_read(0x0A31 + groupIdx*4);
            S5K3M2_DB("TINNO_CAMERA_HAL_DRIVER_LOG_2A_OTP_DATA.S5K3M2.Gr_Ratio = %d,B_Gr_Ratio =%d",R_Gr_Ratio,B_Gr_Ratio);
            S5K3M2_DB("TINNO_CAMERA_HAL_DRIVER_LOG_2A_OTP_DATA.S5K3M2 RG_Ratio_typical = %d,BG_Ratio_typical=%d",RG_Ratio_typical,BG_Ratio_typical);
       }
       if(get_otp_flag(FlagOfAFC,groupIdx) == 0x01)
        {
              S5K3M2_vcm_data[0] = otp_read(0x0A26 + groupIdx*4);
              S5K3M2_vcm_data[1] = otp_read(0x0A28 + groupIdx*4);
              S5K3M2_DB("TINNO_CAMERA_HAL_DRIVER_LOG_2A_OTP_DATA.S5K3M2_vcm_data[0] = %d,S5K3M2_vcm_data[1] =%d",S5K3M2_vcm_data[0],S5K3M2_vcm_data[1]);
       }
   }
   stop_read_otp();
}




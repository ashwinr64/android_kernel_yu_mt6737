/*************************************************************************************************
imx135_otp.c
---------------------------------------------------------
OTP Application file From Truly for imx135
2013.01.14
---------------------------------------------------------
NOTE:
The modification is appended to initialization of image sensor.
After sensor initialization, use the function , and get the id value.
bool otp_wb_update(BYTE zone)
and
bool otp_lenc_update(BYTE zone),
then the calibration of AWB and LSC will be applied.
After finishing the OTP written, we will provide you the golden_rg and golden_bg settings.
**************************************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>


#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "cam_cal.h"
#include "cam_cal_define.h"

#include "imx135mipiraw_Sensor_sunwin.h"

//#include <linux/xlog.h>
#define PFX "imx135_otp_sunwin"
#define LOG_INF(format, args...)    pr_err(PFX "[%s] " format, __FUNCTION__, ##args)
//#include "imx135_otp.h"
#define i2c_write_id  0x34
//#define imx135_otp_sunwin_debug
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
//extern  BYTE imx135_byteread_cmos_sensor(kal_uint32 addr);
//extern kal_uint16 read_cmos_sensor(kal_uint32 addr);
//extern void write_cmos_sensor(kal_uint32 addr, kal_uint32 para);
//extern void write_cmos_sensor_16(kal_uint16 addr,kal_uint16 para);
//extern void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para);
//#define IMX135_SUNWIN_OTP(fmt,arg...)  pr_err(PFX "[%s] " format, __FUNCTION__, ##args)

static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
	kal_uint16 tmp = 0;

    char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
    iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, i2c_write_id);

	//tmp = get_byte >> 8;
	//get_byte = ((get_byte & 0x00ff) << 8) | tmp; 

    return get_byte;
}
static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)para};
    iWriteRegI2C(pu_send_cmd, 3, i2c_write_id);
}
static void write_cmos_sensor_8(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)para};
    iWriteRegI2C(pu_send_cmd, 3, i2c_write_id);
}
static void write_cmos_sensor_16(kal_uint32 addr, kal_uint32 para)
{
    char pu_send_cmd[4] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para >> 8), (char)(para & 0xFF)};
    iWriteRegI2C(pu_send_cmd, 4, i2c_write_id);
}
#if 0
BYTE imx135_byteread_cmos_sensor(kal_uint32 addr)
{
    BYTE get_byte=0;
    char puSendCmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(puSendCmd , 2, (u8*)&get_byte,1,IMX135MIPI_WRITE_ID);
    return get_byte;
}
extern  void imx135_wordwrite_cmos_sensor(u16 addr, u32 para);
extern  void imx135_bytewrite_cmos_sensor(u16 addr, u32 para);
#endif


#define USHORT             unsigned short
#define BYTE               unsigned char
#define Sleep(ms) mdelay(ms)

#define TRULY_ID           0x02

enum LENS
{
    LARGEN_LENS = 1,
    KT_LENS,
    KM_LENS,
    GENIUS_LENS,
    SUNNY_LENS,
    OTHER_LENS,
};
enum DRIVER_IC
{
    DONGWOOK = 1,
    ADI,
    ASM,
    ROHM,
    OTHER_DRIVER,
};
enum VCM
{
    TDK = 1,
    MISTUMIS,
    SIKAO,
    MWT,
    ALPS,
    OTHER_VCM,
};
#define VALID_OTP          0x01

#define GAIN_DEFAULT       0x0100
#define GAIN_GREEN1_ADDR_H   0x020E
#define GAIN_GREEN1_ADDR_L   0x020F

#define GAIN_BLUE_ADDR_H     0x0212
#define GAIN_BLUE_ADDR_L     0x0213

#define GAIN_RED_ADDR_H      0x0210
#define GAIN_RED_ADDR_L      0x0211

#define GAIN_GREEN2_ADDR_H   0x0214
#define GAIN_GREEN2_ADDR_L   0x0214


USHORT current_r;
USHORT current_gr;
USHORT current_gb;
USHORT current_b;

#define golden_r_g 628
#define golden_b_g 666
#define golden_g_g 1025
kal_uint32 r_ratio;
kal_uint32 b_ratio;

BYTE Sunwin_Module_ID;
//kal_uint32    golden_r = 0, golden_gr = 0, golden_gb = 0, golden_b = 0;
//kal_uint32    current_r = 0, current_gr = 0, current_gb = 0, current_b = 0;
/*************************************************************************************************
* Function    :  start_read_otp
* Description :  before read otp , set the reading block setting
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  0, reading block setting err
                 1, reading block setting ok
**************************************************************************************************/
static bool start_read_otp(BYTE zone)
{
LOG_INF("+");
    BYTE val = 0;
    int i;
    write_cmos_sensor_8(0x0104, 0x01);
    write_cmos_sensor(0x3B02, zone);   //PAGE
    write_cmos_sensor(0x3B00, 0x01);
    write_cmos_sensor_8(0x0104, 0x00);
    Sleep(5);
    for(i=0;i<100;i++)
    {
        val = read_cmos_sensor(0x3B01);
		LOG_INF("val = %d\n",val);
        if((val & 0x01) == 0x01)
            break;
        Sleep(2);
    }
    if(i == 100)
    {
        LOG_INF("Read Page %d Err!\n", zone); // print log
        return 0;
    }
	LOG_INF("-");
    return 1;
}


/*************************************************************************************************
* Function    :  get_otp_flag
* Description :  get otp WRITTEN_FLAG
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  [BYTE], if 0x40 , this type has valid otp data, otherwise, invalid otp data
**************************************************************************************************/
static BYTE get_otp_flag(BYTE zone)
{
    BYTE flag = 0;
    if(!start_read_otp(zone))
    {
        LOG_INF("Start read Page %d Fail!\n", zone);
        return 0;
    }
    flag = read_cmos_sensor(0x3B04);
	LOG_INF("flag = %d\n",flag);
   // flag = flag & 0xc0;
    LOG_INF("OTP Flag:0x%02x\n",flag );
    return flag;
}

/*************************************************************************************************
* Function    :  get_otp_date
* Description :  get otp date value
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
**************************************************************************************************/
static bool get_otp_date(BYTE zone)
{
    BYTE year  = 0;
    BYTE month = 0;
    BYTE day   = 0;
    if(!start_read_otp(zone))
    {
        LOG_INF("Start read Page %d Fail!\n", zone);
        return 0;
    }
    year  = read_cmos_sensor(0x3B06);
    month = read_cmos_sensor(0x3B07);
    day   = read_cmos_sensor(0x3B08);
    LOG_INF("OTP date=%02d.%02d.%02d", year,month,day);
    return 1;
}


/*************************************************************************************************
* Function    :  get_otp_module_id
* Description :  get otp MID value
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  [BYTE] 0 : OTP data fail
                 other value : module ID data , TRULY ID is 0x0001
**************************************************************************************************/
 BYTE sunwin_get_otp_module_id(void)
{
    BYTE module_id = 0;
	BYTE id_zone=0;
if(get_otp_flag(0x02)) //page 1 and page 2 for awb
		id_zone = 0x02;
	else
		id_zone = 0x01;	
    if(!start_read_otp(id_zone))
    {
        LOG_INF("OTP Start read Page %d Fail!\n", id_zone);
        return 0;
    }
    module_id = read_cmos_sensor(0x3B0a);
    LOG_INF("OTP_Module ID: 0x%02x.\n",module_id);
    return module_id;
}


/*************************************************************************************************
* Function    :  get_otp_lens_id
* Description :  get otp LENS_ID value
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  [BYTE] 0 : OTP data fail
                 other value : LENS ID data
**************************************************************************************************/
BYTE sunwin_get_otp_lens_id(void)
{
 BYTE lens_id = 0;
	BYTE id_zone=0;
if(get_otp_flag(0x02)) //page 1 and page 2 for awb
		id_zone = 0x02;
	else
		id_zone = 0x01;	
    if(!start_read_otp(id_zone))
    {
        LOG_INF("OTP Start read Page %d Fail!\n", id_zone);
        return 0;
    }
    lens_id = read_cmos_sensor(0x3B0B);
    LOG_INF("OTP_lens ID: 0x%02x.\n",lens_id);
    return lens_id;

   }


/*************************************************************************************************
* Function    :  get_otp_vcm_id
* Description :  get otp VCM_ID value
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  [BYTE] 0 : OTP data fail
                 other value : VCM ID data
**************************************************************************************************/
static BYTE get_otp_vcm_id(BYTE zone)
{
    BYTE vcm_id = 0;
    if(!start_read_otp(zone))
    {
        LOG_INF("Start read Page %d Fail!\n", zone);
        return 0;
    }
    vcm_id = read_cmos_sensor(0x3B0A);
    LOG_INF("OTP_VCM ID: 0x%02x.\n",vcm_id);
    return vcm_id;
}


/*************************************************************************************************
* Function    :  get_otp_driver_id
* Description :  get otp driver id value
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  [BYTE] 0 : OTP data fail
                 other value : driver ID data
**************************************************************************************************/
static BYTE get_otp_driver_id(BYTE zone)
{
    BYTE driver_id = 0;
    if(!start_read_otp(zone))
    {
        LOG_INF("Start read Page %d Fail!\n", zone);
        return 0;
    }
    driver_id = read_cmos_sensor(0x3B0B);
    LOG_INF("OTP_Driver ID: 0x%02x.\n",driver_id);
    return driver_id;
}

/*************************************************************************************************
* Function    :  get_light_id
* Description :  get otp environment light temperature value
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  [BYTE] 0 : OTP data fail
                        other value : driver ID data
                        BIT0:D65(6500K) EN
                        BIT1:D50(5100K) EN
                        BIT2:CWF(4000K) EN
                        BIT3:A Light(2800K) EN
**************************************************************************************************/
static BYTE get_light_id(BYTE zone)
{
    BYTE light_id = 0;
    if(!start_read_otp(zone))
    {
        LOG_INF("Start read Page %d Fail!\n", zone);
        return 0;
    }
    light_id = read_cmos_sensor(0x3B0C);
    LOG_INF("OTP_Light ID: 0x%02x.\n",light_id);
    return light_id;
}

/*************************************************************************************************
* Function    :  get_lsc_flag
* Description :  get LSC WRITTEN_FLAG
* Return      :  [BYTE], if 0x40 , this type has valid lsc data, otherwise, invalid otp data
**************************************************************************************************/
static BYTE get_lsc_flag(void)
{
    BYTE flag = 0;
    if(!start_read_otp(0x0B))
    {
        LOG_INF("Start read Page 0x0B Fail!\n");
        return 0;
    }
    flag = read_cmos_sensor(0x3B43);
    flag = flag & 0xc0;
    LOG_INF("OTP Flag:0x%02x",flag );
    return flag;
}

/*************************************************************************************************
* Function    :  otp_lenc_update
* Description :  Update lens correction
* Return      :  [bool] 0 : OTP data fail
                        1 : otp_lenc update success
**************************************************************************************************/
#if 0
bool otp_lenc_update(void)
{
    BYTE lsc_flag;
    int i, j;
    BYTE temp1, temp2;
	BYTE lsc_data[64 * 8] ={0};

    lsc_flag = get_lsc_flag();
    if(lsc_flag == 0xC0 || lsc_flag == 0x80)
    {
        LOG_INF("OTP lsc data invalid\n");
        return 0;
    }
    else if(lsc_flag == 0x00)
    {
        LOG_INF("OTP no lsc data\n");
        return 0;
    }

    for(i=0;i<8;i++)
    {
        if(!start_read_otp(0x04+i))
        {
            LOG_INF("OTP Start read Page %d Fail!\n", 0x04+i);
            return 0;
        }
        for(j=0;j<64;j++)
            lsc_data[i*64+j] = read_cmos_sensor(0x3B04+j);
    }
#ifdef DEBUG_IMX135_OTP
    for (i=0;i<504;i++)
        {
            LOG_INF("%0x  ",lsc_data[i]);
            if((i+1)%64==0)
                LOG_INF("\n");
        }
#endif

    write_cmos_sensor_8(0x0104, 0x01);
    for(i=0;i<504;i++) //LSC SIZE is 504 BYTES
        write_cmos_sensor(0x4800+i, lsc_data[i]);
    write_cmos_sensor_8(0x0104, 0x00);

    //Enable LSC
    temp1 = read_cmos_sensor(0x0700);
    temp2 = read_cmos_sensor(0x3A63);
    temp1 = temp1 | 0x01;
    temp2 = temp2 | 0x01;
    write_cmos_sensor_8(0x0104, 0x00);
    write_cmos_sensor(0x0700, temp1);
    write_cmos_sensor(0x3A63, temp2);
    write_cmos_sensor_8(0x0104, 0x00);

    LOG_INF("OTP Update lsc finished\n");

    return 1;
}
#endif

static bool otp_lenc_update(BYTE zone)
{
	LOG_INF("+");
	int val= 0;
	int i,j ,k;
	BYTE lsc_data[378] ={0}; //lsc use data
	BYTE lsc_data_mask[384] ={0}; //all data
	    BYTE temp1, temp2;
	if(!start_read_otp(zone))
    {
        LOG_INF("Start read Page %d Fail!\n", zone);
        return 0;
    }

	for( i=0;i<6;i++)
    {
    	if(start_read_otp(zone+i)){
        for( j=0;j<64;j++){
            lsc_data_mask[i*64+j] = read_cmos_sensor(0x3B04+j);
#ifdef imx135_otp_sunwin_debug
		val = i*64+j;
		LOG_INF("lsc_data_mask[%d]= %d\n",val,lsc_data_mask[i*64+j]);
#endif
    		}	
    	}	
   }
	memcpy(lsc_data,lsc_data_mask+3,sizeof(lsc_data));
#ifdef imx135_otp_sunwin_debug
	for ( k=0;k<378;k++){
	LOG_INF("lsc_data[%d]= %d\n",k,lsc_data[k]);
	}
#endif
	write_cmos_sensor_8(0x0104, 0x01);
	for(i=0;i<126;i++){  	
        write_cmos_sensor(0x4800+i*4, (lsc_data[i*3]>>4));
	 write_cmos_sensor((0x4800+i*4+1), (lsc_data[i*3+1]));
	 write_cmos_sensor((0x4800+i*4+2), (lsc_data[i*3] & 0xF));
	  write_cmos_sensor((0x4800+i*4+3), (lsc_data[i*3+2]));			  
	}
	write_cmos_sensor_8(0x0104, 0x00);

	    //Enable LSC
    temp1 = read_cmos_sensor(0x0700);
    temp2 = read_cmos_sensor(0x3A63);
    temp1 = temp1 | 0x01;
    temp2 = temp2 | 0x01;
    write_cmos_sensor_8(0x0104, 0x00);
    write_cmos_sensor(0x0700, temp1);
    write_cmos_sensor(0x3A63, temp2);
    write_cmos_sensor_8(0x0104, 0x00);

    LOG_INF("OTP Update lsc finished\n");

	return 1;
}
/*************************************************************************************************
* Function    :  wb_gain_set
* Description :  Set WB ratio to register gain setting  512x
* Parameters  :  [int] r_ratio : R ratio data compared with golden module R
                       b_ratio : B ratio data compared with golden module B
* Return      :  [bool] 0 : set wb fail
                        1 : WB set success
**************************************************************************************************/

static bool wb_gain_set(void)
{
    USHORT R_GAIN;
    USHORT B_GAIN;
    USHORT Gr_GAIN;
    USHORT Gb_GAIN;
    USHORT G_GAIN;

	u8 get_R_gain_h = 0;
	u8 get_R_gain_l = 0;
	u8 get_G_gain_h = 0;
	u8 get_G_gain_l = 0;
	u8 get_B_gain_h = 0;
	u8 get_B_gain_l = 0;
	
	
    if(!r_ratio || !b_ratio)
    {
        LOG_INF("OTP WB ratio Data Err!\n");
        return 0;
    }

    if(r_ratio >= 512 )
    {
        if(b_ratio>=512)
        {
            R_GAIN = (USHORT)(GAIN_DEFAULT * r_ratio / 512);
            G_GAIN = GAIN_DEFAULT;
            B_GAIN = (USHORT)(GAIN_DEFAULT * b_ratio / 512);
        }
        else
        {
            R_GAIN =  (USHORT)(GAIN_DEFAULT*r_ratio / b_ratio );
            G_GAIN = (USHORT)(GAIN_DEFAULT*512 / b_ratio );
            B_GAIN = GAIN_DEFAULT;
        }
    }
    else
    {
        if(b_ratio >= 512)
        {
            R_GAIN = GAIN_DEFAULT;
            G_GAIN = (USHORT)(GAIN_DEFAULT*512 /r_ratio);
            B_GAIN =  (USHORT)(GAIN_DEFAULT*b_ratio / r_ratio );
        }
        else
        {
            Gr_GAIN = (USHORT)(GAIN_DEFAULT*512/ r_ratio );
            Gb_GAIN = (USHORT)(GAIN_DEFAULT*512/b_ratio );
            if(Gr_GAIN >= Gb_GAIN)
            {
                R_GAIN = GAIN_DEFAULT;
                G_GAIN = (USHORT)(GAIN_DEFAULT *512/ r_ratio );
                B_GAIN =  (USHORT)(GAIN_DEFAULT*b_ratio / r_ratio );
            }
            else
            {
                R_GAIN =  (USHORT)(GAIN_DEFAULT*r_ratio  / b_ratio);
                G_GAIN = (USHORT)(GAIN_DEFAULT*512 / b_ratio );
                B_GAIN = GAIN_DEFAULT;
            }
        }
    }

    //R_GAIN = 0x0FFF; // For testing, use this gain the image will become red.

    LOG_INF("OTP_golden_r=%d,golden_gr=%d,golden_gb=%d,golden_b=%d \n",golden_r_g,golden_g_g,golden_g_g,golden_b_g);
    LOG_INF("OTP_current_r=%d,current_gr=%d,current_gb=%d,current_b=%d \n",current_r,current_gr,current_gb,current_b);
    LOG_INF("OTP_r_ratio=%d,b_ratio=%d \n",r_ratio,b_ratio);
    LOG_INF("R_GAIN=0x%0x,G_GAIN=0x%0x,B_GAIN=0x%0x.\n",R_GAIN,G_GAIN,B_GAIN);
#if 0
    write_cmos_sensor_8(0x0104, 0x01);
    write_cmos_sensor_8(GAIN_RED_ADDR_H, (R_GAIN>>8)&0xff);
    write_cmos_sensor_8(GAIN_RED_ADDR_L, (R_GAIN)&0xff);
    write_cmos_sensor_8(GAIN_BLUE_ADDR_H, (B_GAIN>>8)&0xff);
    write_cmos_sensor_8(GAIN_BLUE_ADDR_L, (B_GAIN)&0xff);
    write_cmos_sensor_8(GAIN_GREEN1_ADDR_H, (G_GAIN>>8)&0xff); //Green 1 default gain 1x
    write_cmos_sensor_8(GAIN_GREEN1_ADDR_L, (G_GAIN)&0xff);
    write_cmos_sensor_8(GAIN_GREEN2_ADDR_H, (G_GAIN>>8)&0xff);//Green 2 default gain 1x
    write_cmos_sensor_8(GAIN_GREEN2_ADDR_L, (G_GAIN)&0xff); //Green 2 default gain 1x
        write_cmos_sensor_8(0x0104, 0x00);
#endif
#if 1
    write_cmos_sensor_8(0x0104, 0x01);
    write_cmos_sensor_16(GAIN_RED_ADDR_H,R_GAIN);
    write_cmos_sensor_16(GAIN_BLUE_ADDR_H,B_GAIN);
    write_cmos_sensor_16(GAIN_GREEN1_ADDR_H,G_GAIN);
    write_cmos_sensor_16(GAIN_GREEN2_ADDR_H,G_GAIN);
    write_cmos_sensor_8(0x0104, 0x00);
#endif
#ifdef imx135_otp_sunwin_debug
get_R_gain_h = read_cmos_sensor(GAIN_RED_ADDR_H); 
get_R_gain_l = read_cmos_sensor(GAIN_RED_ADDR_L); 
get_G_gain_h = read_cmos_sensor(GAIN_GREEN1_ADDR_H);
get_G_gain_l = read_cmos_sensor(GAIN_GREEN1_ADDR_L);
get_B_gain_h = read_cmos_sensor(GAIN_BLUE_ADDR_H);
get_B_gain_l = read_cmos_sensor(GAIN_BLUE_ADDR_L);

LOG_INF("get_R_gain_h=0x%x,get_R_gain_l=0x%x\n",get_R_gain_h,get_R_gain_l);
LOG_INF("get_G_gain_h=0x%x,get_G_gain_l=0x%x\n",get_G_gain_h,get_G_gain_l);
LOG_INF("get_B_gain_h=0x%x,get_B_gain_l=0x%x\n",get_B_gain_h,get_B_gain_l);
#endif
    LOG_INF("OTP WB Update Finished! \n");
    return 1;
}

/*************************************************************************************************
* Function    :  get_otp_wb
* Description :  Get WB data
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
**************************************************************************************************/
static bool get_otp_wb(BYTE zone)
{
    BYTE temph = 0;
    BYTE templ = 0;
    current_r = 0, current_gr = 0, current_gb = 0, current_b = 0;

    if(!start_read_otp(zone))
    {
        LOG_INF("Start read Page %d Fail!\n", zone);
        return 0;
    }

/*    temph = read_cmos_sensor(0x3B18);
    templ = read_cmos_sensor(0x3B19);
    golden_r  = (USHORT)templ + (((USHORT)temph & 0x03) << 8);

    temph = read_cmos_sensor(0x3B1A);
    templ = read_cmos_sensor(0x3B1B);
    golden_gr  = (USHORT)templ + (((USHORT)temph & 0x03) << 8);

    temph = read_cmos_sensor(0x3B1C);
    templ = read_cmos_sensor(0x3B1D);
    golden_gb  = (USHORT)templ + (((USHORT)temph & 0x03) << 8);

    temph = read_cmos_sensor(0x3B1E);
    templ = read_cmos_sensor(0x3B1F);
    golden_b  = (USHORT)templ + (((USHORT)temph & 0x03) << 8);

    temph = read_cmos_sensor(0x3B10);
    templ = read_cmos_sensor(0x3B11);
    current_r  = (USHORT)templ + (((USHORT)temph & 0x03) << 8);

    temph = read_cmos_sensor(0x3B12);
    templ = read_cmos_sensor(0x3B13);
    current_gr  = (USHORT)templ + (((USHORT)temph & 0x03) << 8);

    temph = read_cmos_sensor(0x3B14);
    templ = read_cmos_sensor(0x3B15);
    current_gb  = (USHORT)templ + (((USHORT)temph & 0x03) << 8);

    temph = read_cmos_sensor(0x3B16);
    templ = read_cmos_sensor(0x3B17);
    current_b  = (USHORT)templ + (((USHORT)temph & 0x03) << 8);*/
	current_r = (read_cmos_sensor(0x3B13) << 8) | read_cmos_sensor(0x3B14);
	current_b = (read_cmos_sensor(0x3B15) << 8) | read_cmos_sensor(0x3B16);
	current_gr = current_gb = (read_cmos_sensor(0x3B17) << 8) | read_cmos_sensor(0x3B18);
	LOG_INF("current_r = %d,current_b =%d,current_gr=current_gb=%d",current_r,current_b,current_gr);
    return 1;
}


/*************************************************************************************************
* Function    :  otp_wb_update
* Description :  Update WB correction
* Return      :  [bool] 0 : OTP data fail
                        1 : otp_WB update success
**************************************************************************************************/
static bool otp_wb_update(BYTE zone)
{
    USHORT golden_g, current_g;


    if(!get_otp_wb(zone))  // get wb data from otp
    {
        LOG_INF("Get OTP WB data Err!\n");
        return 0;
    }

    golden_g = golden_g_g;
    current_g = (current_gr + current_gb) / 2;

    if(!golden_r_g || !golden_b_g || !golden_g_g || !current_gr || !current_r || !current_b)
    {
        LOG_INF("WB update Err !\n");
        return 0;
    }

    r_ratio = 512 * golden_r_g * current_g /( golden_g_g * current_r );
    b_ratio = 512 * golden_b_g * current_g /( golden_g_g * current_b );

    wb_gain_set();

    LOG_INF("OTP WB update finished! \n");

    return 1;
}
static BYTE _otp_awb_set = 0;
static BYTE _otp_lsc_set = 0;
static DEFINE_SPINLOCK(imx135_otp_lock);

void otp_clear_flag_sunwin(void){
	spin_lock(&imx135_otp_lock);
	_otp_awb_set = 0;
    _otp_lsc_set = 0;
	spin_unlock(&imx135_otp_lock);
}

/*************************************************************************************************
* Function    :  otp_update()
* Description :  update otp data from otp , it otp data is valid,
                 it include get ID and WB update function
* Return      :  [bool] 0 : update fail
                        1 : update success
**************************************************************************************************/
#if 0
bool otp_update(BYTE update_sensor_otp_awb, BYTE update_sensor_otp_lsc)
{
    BYTE zone = 0x01;
    BYTE FLG  = 0x00;
    BYTE MID = 0x00;
    int i;

    LOG_INF("update_sensor_otp_awb: %d, update_sensor_otp_lsc: %d _otp_awb_set %d _otp_lsc_set%d\n",
		update_sensor_otp_awb, update_sensor_otp_lsc, _otp_awb_set, _otp_lsc_set);
    if(_otp_awb_set ==1 &&_otp_lsc_set ==1)
		return 1;

    for(i=0;i<3;i++)
    {
        FLG = get_otp_flag(zone);
        if(FLG == VALID_OTP)
            break;
        else
            zone++;
    }
	LOG_INF("zone = %d\n",zone);
    if(i==3)
    {
        LOG_INF("No OTP Data or OTP data is invalid!!!\n");
        return 0;
    }

    MID =     get_otp_module_id(zone);
	LOG_INF("MID = %d\n",MID);
#ifdef DEBUG_IMX135_OTP
    get_otp_lens_id(zone);
    get_otp_vcm_id(zone);
#endif

    if(MID != TRULY_ID) //Select
    {
        LOG_INF("No Truly Module !!!!\n");
        return 0;
    }

    if(0 != update_sensor_otp_awb && _otp_awb_set == 0) {
		LOG_INF("update_sensor_otp_awb == 1");
    	spin_lock(&imx135_otp_lock);
        _otp_awb_set = 1;
        spin_unlock(&imx135_otp_lock);
        if(otp_wb_update(zone)){
	    return 0;
        }
    }


    if(0 != update_sensor_otp_lsc && _otp_lsc_set == 0)
    {
        spin_lock(&imx135_otp_lock);
        _otp_lsc_set = 1;
     	spin_unlock(&imx135_otp_lock);
        if(!otp_lenc_update())
        {
            LOG_INF("OTP Update LSC Err\n");
            return 0;
        }
    }
    return 1;
}

#endif
USHORT sunwin_af_inf;
USHORT sunwin_af_mac;
//liaoshuang add for imx135 otp
bool otp_update_sunwin(BYTE update_sensor_otp_awb, BYTE update_sensor_otp_lsc)
{
	BYTE awb_flag=0, lsc_flag =0,af_flag=0;
	BYTE awb_zone=0, lsc_zone=0,af_zone=0;
//awb data
	if(get_otp_flag(0x02)) //page 1 and page 2 for awb
		awb_zone = 0x02;
	else
		awb_zone = 0x01;	
	LOG_INF("awb_zone=%d\n",awb_zone);
	if(0 != update_sensor_otp_awb && _otp_awb_set == 0) {
	LOG_INF("update_sensor_otp_awb == 1");
    		spin_lock(&imx135_otp_lock);
        _otp_awb_set = 1;
        spin_unlock(&imx135_otp_lock);
        if(!otp_wb_update(awb_zone)){
	    return 0;
        }
    }
//	Sunwin_Module_ID = get_otp_module_id(awb_zone);
	LOG_INF("Sunwin_Module_ID = %d\n",Sunwin_Module_ID);
//lsc data
	if(get_otp_flag(0x09))//page 3-8 and page 9-14 for lsc
		lsc_zone = 0x09;
	else
		lsc_zone = 0x03;
	LOG_INF("lsc_zone=%d\n",lsc_zone);
	 if(0 != update_sensor_otp_lsc && _otp_lsc_set == 0)
    {
    		LOG_INF("update_sensor_otp_lsc == 1");
        spin_lock(&imx135_otp_lock);
        _otp_lsc_set = 1;
     	spin_unlock(&imx135_otp_lock);
        if(!otp_lenc_update(lsc_zone))
        {
            LOG_INF("OTP Update LSC Err\n");
            return 0;
        }
    }
//af data 
	if(start_read_otp(0x0f))
    {
    	if(read_cmos_sensor(0x3B14))
		sunwin_af_mac = (read_cmos_sensor(0x3b17) << 8) | (read_cmos_sensor(0x3b18));
	else 
		sunwin_af_mac = (read_cmos_sensor(0x3b07) << 8) | (read_cmos_sensor(0x3b08));
	if(read_cmos_sensor(0x3B34))
		sunwin_af_inf = (read_cmos_sensor(0x3b37) << 8) | (read_cmos_sensor(0x3b38));	
	else		
		sunwin_af_inf = (read_cmos_sensor(0x3b27) << 8) | (read_cmos_sensor(0x3b28));
	 LOG_INF("start_read_otp(0x15)-");
    }
	else{
	sunwin_af_mac = 0;
	sunwin_af_inf = 0;   
   }
	LOG_INF("sunwin_af_mac = %d,sunwin_af_inf = %d\n",sunwin_af_mac,sunwin_af_inf);
}



static DEFINE_SPINLOCK(g_CAM_CALLock); // for SMP
#define CAM_CAL_I2C_BUSNUM 0
#define CAM_CAL_DEV_MAJOR_NUMBER 226

/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_ICS_REVISION 1 //seanlin111208
/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_DRVNAME "IMX135_SUNWIN_CAM_CAL_DRV"
#define CAM_CAL_I2C_GROUP_ID 0
/*******************************************************************************
*
********************************************************************************/
/* fix warning MSG 
static unsigned short g_pu2Normal_i2c[] = {S5K4H8_DEVICE_ID , I2C_CLIENT_END};
static unsigned short g_u2Ignore = I2C_CLIENT_END;
static struct i2c_client_address_data g_stCAM_CAL_Addr_data = {
    .normal_i2c = g_pu2Normal_i2c,
    .probe = &g_u2Ignore,
    .ignore = &g_u2Ignore
}; */

static struct i2c_board_info __initdata kd_cam_cal_dev={ I2C_BOARD_INFO(CAM_CAL_DRVNAME, 0x20>>1)}; //make dummy_eeprom co-exist

//81 is used for V4L driver
static dev_t g_CAM_CALdevno = MKDEV(CAM_CAL_DEV_MAJOR_NUMBER,0);
static struct cdev * g_pCAM_CAL_CharDrv = NULL;
//static spinlock_t g_CAM_CALLock;
static struct class *CAM_CAL_class = NULL;
static atomic_t g_CAM_CALatomic;
/*******************************************************************************
*
********************************************************************************/

u8 imx135_sunwin_id_data[]= {0x10,0xff,0x00,0x40,0x88};
static int selective_read_region(u32 offset, BYTE* data,u16 i2c_id,u32 size)
{    
    printk("[IMX135_SUNWIN_CAM_CAL] selective_read_region offset =%d size %d data read = %d\n", offset,size, *data);
if(size ==2 ){
	if(offset == 7){
		memcpy((void *)data,(void *)&sunwin_af_inf,size);
	}
	
	else if(offset == 9){
		 
		memcpy((void *)data,(void *)&sunwin_af_mac,size);

	}
}
if(size == 4){
		memcpy((void *)data,(void *)&imx135_sunwin_id_data[1],size);

	}


}



/*******************************************************************************
*
********************************************************************************/
#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int CAM_CAL_Ioctl(struct inode * a_pstInode,
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
#else 
static long CAM_CAL_Ioctl(
    struct file *file, 
    unsigned int a_u4Command, 
    unsigned long a_u4Param
)
#endif
{
    int i4RetValue = 0;
    u8 * pBuff = NULL;
    u8 * pWorkingBuff = NULL;
    stCAM_CAL_INFO_STRUCT *ptempbuf;

#ifdef CAM_CALGETDLT_DEBUG
    struct timeval ktv1, ktv2;
    unsigned long TimeIntervalUS;
#endif

    if(_IOC_NONE == _IOC_DIR(a_u4Command))
    {
    }
    else
    {
        pBuff = (u8 *)kmalloc(sizeof(stCAM_CAL_INFO_STRUCT),GFP_KERNEL);

        if(NULL == pBuff)
        {
            LOG_INF("[IMX135_SUNWIN_CAM_CAL] ioctl allocate mem failed\n");
            return -ENOMEM;
        }

        if(_IOC_WRITE & _IOC_DIR(a_u4Command))
        {
            if(copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT)))
            {    //get input structure address
                kfree(pBuff);
                LOG_INF("[IMX135_SUNWIN_CAM_CAL] ioctl copy from user failed\n");
                return -EFAULT;
            }
        }
    }

    ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
    pWorkingBuff = (u8*)kmalloc(ptempbuf->u4Length,GFP_KERNEL); 
    if(NULL == pWorkingBuff)
    {
        kfree(pBuff);
        LOG_INF("[IMX135_SUNWIN_CAM_CAL] ioctl allocate mem failed\n");
        return -ENOMEM;
    }
//fix warning MSG     LOG_INF("[IMX135_SUNWIN_CAM_CAL] init Working buffer address 0x%x  command is 0x%08x\n", pWorkingBuff, a_u4Command);

 
    if(copy_from_user((u8*)pWorkingBuff ,  (u8*)ptempbuf->pu1Params, ptempbuf->u4Length))
    {
        kfree(pBuff);
        kfree(pWorkingBuff);
        LOG_INF("[IMX135_SUNWIN_CAM_CAL] ioctl copy from user failed\n");
        return -EFAULT;
    } 
    
    switch(a_u4Command)
    {


		 case CAM_CALIOC_S_WRITE:
    LOG_INF("[LOG_INF] CAM_CALIOC_S_WRITE \n");        
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif
            i4RetValue = 0;
           // i4RetValue=iWriteData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pu1Params);
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
#endif
            break;

	
      
        case CAM_CALIOC_G_READ:
            LOG_INF("[IMX135_SUNWIN_CAM_CAL] Read CMD \n");
#ifdef CAM_CALGETDLT_DEBUG            
            do_gettimeofday(&ktv1);
#endif 
            LOG_INF("[IMX135_SUNWIN_CAM_CAL] offset %d \n", ptempbuf->u4Offset);
            LOG_INF("[IMX135_SUNWIN_CAM_CAL] length %d \n", ptempbuf->u4Length);
          // LOG_INF("[IMX135_SUNWIN_CAM_CAL] Before read Working buffer address 0x%x \n", pWorkingBuff);


if(ptempbuf->u4Length == 2){

       i4RetValue = selective_read_region(ptempbuf->u4Offset, pWorkingBuff, 0x20, ptempbuf->u4Length);

}else if(ptempbuf->u4Length == 4){

	   i4RetValue = selective_read_region(ptempbuf->u4Offset, pWorkingBuff, 0x20, ptempbuf->u4Length);
 }        
		
          // LOG_INF("[IMX135_SUNWIN_CAM_CAL] After read Working buffer address 0x%x \n", pWorkingBuff);


#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            printk("Read data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif            

            break;
        default :
      	     LOG_INF("[IMX135_SUNWIN_CAM_CAL] No CMD \n");
            i4RetValue = -EPERM;
        break;
    }

    if(_IOC_READ & _IOC_DIR(a_u4Command))
    {
        //copy data to user space buffer, keep other input paremeter unchange.
        LOG_INF("[IMX135_SUNWIN_CAM_CAL] to user length %d \n", ptempbuf->u4Length);
		
  

  
		if(copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pWorkingBuff , ptempbuf->u4Length))
        {
            kfree(pBuff);
            kfree(pWorkingBuff);
            LOG_INF("[IMX135_SUNWIN_CAM_CAL] ioctl copy to user failed\n");
            return -EFAULT;
        }
    }

    kfree(pBuff);
    kfree(pWorkingBuff);
    return i4RetValue;
}


static u32 g_u4Opened = 0;
//#define
//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.
static int CAM_CAL_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    LOG_INF("[s5k4h8 CAM_CAL] CAM_CAL_Open\n");
    spin_lock(&g_CAM_CALLock);
    if(g_u4Opened)
    {
        spin_unlock(&g_CAM_CALLock);
        return -EBUSY;
    }
    else
    {
        g_u4Opened = 1;
        atomic_set(&g_CAM_CALatomic,0);
    }
    spin_unlock(&g_CAM_CALLock);

//#if defined(MT6572)
	// do nothing
//#else
    //if(TRUE != hwPowerOn(MT65XX_POWER_LDO_VCAMA, VOL_2800, "S24CS64A"))
    //{
    //    LOG_INF("[IMX135_SUNWIN_CAM_CAL] Fail to enable analog gain\n");
    //    return -EIO;
    //}
//#endif	

    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int CAM_CAL_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    spin_lock(&g_CAM_CALLock);

    g_u4Opened = 0;

    atomic_set(&g_CAM_CALatomic,0);

    spin_unlock(&g_CAM_CALLock);

    return 0;
}

static const struct file_operations g_stCAM_CAL_fops =
{
    .owner = THIS_MODULE,
    .open = CAM_CAL_Open,
    .release = CAM_CAL_Release,
    //.ioctl = CAM_CAL_Ioctl
    .unlocked_ioctl = CAM_CAL_Ioctl
};

#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1
inline static int RegisterCAM_CALCharDrv(void)
{
    struct device* CAM_CAL_device = NULL;

#if CAM_CAL_DYNAMIC_ALLOCATE_DEVNO
    if( alloc_chrdev_region(&g_CAM_CALdevno, 0, 1,CAM_CAL_DRVNAME) )
    {
        LOG_INF("[IMX135_SUNWIN_CAM_CAL] Allocate device no failed\n");

        return -EAGAIN;
    }
#else
    if( register_chrdev_region(  g_CAM_CALdevno , 1 , CAM_CAL_DRVNAME) )
    {
        LOG_INF("[IMX135_SUNWIN_CAM_CAL] Register device no failed\n");

        return -EAGAIN;
    }
#endif

    //Allocate driver
    g_pCAM_CAL_CharDrv = cdev_alloc();

    if(NULL == g_pCAM_CAL_CharDrv)
    {
        unregister_chrdev_region(g_CAM_CALdevno, 1);

        LOG_INF("[IMX135_SUNWIN_CAM_CAL] Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

    g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1))
    {
        LOG_INF("[IMX135_SUNWIN_CAM_CAL] Attatch file operation failed\n");

        unregister_chrdev_region(g_CAM_CALdevno, 1);

        return -EAGAIN;
    }
 
    CAM_CAL_class = class_create(THIS_MODULE, "IMX135_SUNWIN_CAM_CALdrv");
    if (IS_ERR(CAM_CAL_class)) {
        int ret = PTR_ERR(CAM_CAL_class);
        LOG_INF("Unable to create class, err = %d\n", ret);
        return ret;
    }
	
    CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);

    return 0;
}

inline static void UnregisterCAM_CALCharDrv(void)
{
    //Release char driver
    cdev_del(g_pCAM_CAL_CharDrv);

    unregister_chrdev_region(g_CAM_CALdevno, 1);

    device_destroy(CAM_CAL_class, g_CAM_CALdevno);
    class_destroy(CAM_CAL_class);
}

static int CAM_CAL_probe(struct platform_device *pdev)
{

    return 0;//i2c_add_driver(&CAM_CAL_i2c_driver);
}

static int CAM_CAL_remove(struct platform_device *pdev)
{
    //i2c_del_driver(&CAM_CAL_i2c_driver);
    return 0;
}

// platform structure
static struct platform_driver g_stCAM_CAL_Driver = {
    .probe		= CAM_CAL_probe,
    .remove	= CAM_CAL_remove,
    .driver		= {
        .name	= CAM_CAL_DRVNAME,
        .owner	= THIS_MODULE,
    }
};


static struct platform_device g_stCAM_CAL_Device = {
    .name = CAM_CAL_DRVNAME,
    .id = 0,
    .dev = {
    }
};

static int __init CAM_CAL_i2C_init(void)
{

   // i2c_register_board_info(CAM_CAL_I2C_BUSNUM, &kd_cam_cal_dev, 1);


	 int i4RetValue = 0;
    LOG_INF("[IMX135_SUNWIN_CAM_CAL]\n");
   //Register char driver
	i4RetValue = RegisterCAM_CALCharDrv();
    if(i4RetValue){
 	   LOG_INF(" [IMX135_SUNWIN_CAM_CAL] register char device failed!\n");
	   return i4RetValue;
	}
	LOG_INF(" [IMX135_SUNWIN_CAM_CAL] Attached!! \n");

   
    if(platform_driver_register(&g_stCAM_CAL_Driver)){
        printk("failed to register CAM_CAL driver\n");
        return -ENODEV;
    }
    if (platform_device_register(&g_stCAM_CAL_Device))
    {
        printk("failed to register CAM_CAL driver\n");
        return -ENODEV;
    }	
LOG_INF(" IMX135_SUNWIN_CAM_CAL  Attached Pass !! \n");
    return 0;
}

static void __exit CAM_CAL_i2C_exit(void)
{
	platform_driver_unregister(&g_stCAM_CAL_Driver);
}

module_init(CAM_CAL_i2C_init);
module_exit(CAM_CAL_i2C_exit);

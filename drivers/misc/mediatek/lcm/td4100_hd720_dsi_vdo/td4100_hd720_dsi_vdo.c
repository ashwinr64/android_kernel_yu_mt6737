#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
   #include <platform/mt_gpio.h>
   #include <string.h>
#elif defined(BUILD_UBOOT)
   #include <asm/arch/mt_gpio.h>
#else
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1280)

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

#ifdef BUILD_LK
#define LCM_PRINT printf
#else
#if defined(BUILD_UBOOT)
#define LCM_PRINT printf
#else
#define LCM_PRINT printk
#endif
#endif

#define LCM_DBG(fmt, arg...) \
    LCM_PRINT ("[TD4100 kernel] %s (line:%d) :" fmt "\r\n", __func__, __LINE__, ## arg)
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
#define AUXADC_LCD_ID_CHANNEL   12
#define TD4100_LCM_MIN_VOL 1250
#define TD4100_LCM_MAX_VOL 1500

static LCM_UTIL_FUNCS lcm_util;
#if 0
#define SET_RESET_PIN(v)                                    (lcm_util.set_reset_pin((v)))
#else
extern int DispTEpin_Enable(void);
extern int DispTEpin_Disable(void);
#define SET_RESET_PIN(v)    \
    if(v)                                           \
        DispTEpin_Enable(); \
    else                                           \
        DispTEpin_Disable();
#endif

extern void lcm_power_ldo(int onoff);
extern void lcm_cs_pin(int onoff);

#define SET_PWR_PIN(v)     \
	if(v)					\
		lcm_power_ldo(v);\
	else					\
		lcm_power_ldo(v);

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V3(para_tbl,size,force_update)        lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define   LCM_DSI_CMD_MODE							0

#define REGFLAG_END_OF_TABLE                                	  0xFD   // END OF REGISTERS MARKER
#define REGFLAG_DELAY                                           0xFC

struct LCM_setting_table
{
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table  lcm_deep_sleep_mode_in_setting_v2[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 50, {}},

    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
};

static struct LCM_setting_table  lcm_deep_sleep_mode_out_setting_v2[] = {
    // Display off sequence
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Sleep Mode On
    {0x29, 1, {0x00}},
    {REGFLAG_DELAY, 50, {}},
};

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;
    for(i = 0; i < count; i++)
    {
        unsigned cmd;
        cmd = table[i].cmd;
        switch (cmd)
        {
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
            case REGFLAG_END_OF_TABLE :
                break;
            default:
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }

}


static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        #else
		params->dsi.mode   = BURST_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
        #endif
	
		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Video mode setting		
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		
		params->dsi.vertical_sync_active = 126;
		params->dsi.vertical_backporch = 128;
		params->dsi.vertical_frontporch	= 127;
		params->dsi.vertical_active_line = FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active = 4;
		params->dsi.horizontal_backporch = 60; //6
		params->dsi.horizontal_frontporch = 24;//78;
		params->dsi.horizontal_active_pixel = FRAME_WIDTH;

		// Bit rate calculation
		//params->dsi.PLL_CLOCK=205;
		params->dsi.PLL_CLOCK=230;  //LINE </EGAFM-297> <change the mipi clock to reduce disturbing the wifi> <20160413> panzaoyan
		params->dsi.ssc_disable=1;
		
		//yixuhong 20150511 add esd check function
#ifndef BUILD_LK	
	params->dsi.esd_check_enable = 1; 
	params->dsi.customization_esd_check_enable = 0;//0:te esd check 1:read register
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x1C;
	
	params->dsi.lcm_esd_check_table[1].cmd = 0xFA;
	params->dsi.lcm_esd_check_table[1].count = 2;
	params->dsi.lcm_esd_check_table[1].para_list[0] = 0x00;
	params->dsi.lcm_esd_check_table[1].para_list[1] = 0x00;
#endif /*BUILD_LK*/

}

static void init_lcm_registers(void)
{
	unsigned int data_array[32];
	
	data_array[0] = 0x00350500;
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
	
	data_array[0] = 0x00290500; 
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);
	
}

static void lcm_init(void)
{
	LCM_DBG(); 
	SET_PWR_PIN(0);
	MDELAY(10);
	SET_PWR_PIN(1);
	MDELAY(10);
	
	lcm_cs_pin(1);
	MDELAY(20);
	
	SET_RESET_PIN(0);
	MDELAY(5); 
	SET_RESET_PIN(1);
	MDELAY(130);
	init_lcm_registers();
	LCM_DBG("lcm init end \n");

}

static void lcm_suspend(void)
{	
	LCM_DBG();
  
	//push_table(lcm_deep_sleep_mode_in_setting_v2, sizeof(lcm_deep_sleep_mode_in_setting_v2) / sizeof(struct LCM_setting_table), 1);
	SET_RESET_PIN(0);
	MDELAY(5); 
	SET_RESET_PIN(1);
	MDELAY(130);
    lcm_cs_pin(0);
  	MDELAY(20);
	    
}


static void lcm_resume(void)
{
	LCM_DBG();
	//push_table(lcm_deep_sleep_mode_out_setting_v2, sizeof(lcm_deep_sleep_mode_out_setting_v2) / sizeof(struct LCM_setting_table), 1);
	lcm_init();
}      

static unsigned int lcm_compare_id(void)
{
	int data[4] = {0, 0, 0, 0};
	int tmp = 0 ,rc = 0, iVoltage = 0;
	rc = IMM_GetOneChannelValue(AUXADC_LCD_ID_CHANNEL, data, &tmp);
		if(rc < 0)
		{
			LCM_DBG("read LCD_ID vol error\n");
			return 0;
		}
		else
		{
			iVoltage = (data[0]*1000) + (data[1]*10) + (data[2]);
			LCM_DBG("data[0]=%d,data[1]=%d,data[2]=%d,data[3]=%d,iVoltage=%d\n",
					data[0], data[1], data[2], data[3], iVoltage);
		if((TD4100_LCM_MIN_VOL <= iVoltage) && (iVoltage < TD4100_LCM_MAX_VOL)){
				return 1;
			}else{
				 return 0;
			}
		}
		return 0;
}



LCM_DRIVER td4100_hd720_dsi_vdo_lcm_drv = 
{
	.name		= "td4100_hd720_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
};

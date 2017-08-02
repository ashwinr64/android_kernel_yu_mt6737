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
    LCM_PRINT ("[ili9881c_boe] %s (line:%d) :" fmt "\r\n", __func__, __LINE__, ## arg)
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util;
#if 0
#define SET_RESET_PIN(v)                                    (lcm_util.set_reset_pin((v)))
#else
extern int DispTEpin_Enable(void);
extern int DispTEpin_Disable(void);
extern int get_lcm_id_status(void);
#define SET_RESET_PIN(v)    \
    if(v)                                           \
        DispTEpin_Enable(); \
    else                                           \
        DispTEpin_Disable();
#endif

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
#define AUXADC_LCD_ID_CHANNEL   12
#define BOE_LCM_MIN_VOL 0
#define BOE_LCM_MAX_VOL 120
#define LCM_ID_ILI9881	0x9881 
#define GPIO_LCD_ID (GPIO60 | 0x80000000)

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


static struct LCM_setting_table lcm_initialization_setting_chipone[] = 
{
	{0x01,1,{0x00}},
	{REGFLAG_DELAY,10,{}},
	{0xF0,2,{0x5A,0x5A}},
	{0xF1,2,{0xA5,0xA5}},
	{0xF0,2,{0xB4,0x4B}},
	{0xB6,2,{0x4E,0x4E}},
	{0xB3,22,{0x23,0x1D,0x06,0x04,0x10,0x12,0x0C,0x0E,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x00,0x00,0x00,0x22,0x1C}},
	{0xB4,22,{0x23,0x1D,0x07,0x05,0x11,0x13,0x0D,0x0F,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x00,0x00,0x00,0x22,0x1C}},
	{0xB0,12,{0x98,0x76,0x98,0x76,0x33,0x33,0x33,0x33,0x22,0x01,0x7B,0x01}},
	{0xB1,8,{0x53,0xA0,0x00,0x85,0x22,0x01,0x7B,0x01}},
	{0xBD,6,{0x4E,0x0E,0x41,0x41,0x11,0x1E}},
	{0xB7,17,{0x01,0x01,0x09,0x11,0x0D,0x15,0x19,0x0D,0x21,0x1D,0x00,0x00,0x20,0x00,0x02,0xFF,0x3C}},
	{0xB8,5,{0x24,0x01,0x30,0x34,0x53}},
	{0xB9,4,{0xA1,0x2C,0xFF,0xC4}},
	{0xBA,2,{0x27,0x63}},
	{0xC1,6,{0x16,0x16,0x04,0x0C,0x10,0x04}},
	{0xC2,2,{0x82,0x10}},
	{0xC3,3,{0x22,0x31,0x04}},
	{0xC7,5,{0x05,0x23,0x6B,0x41,0x00}},
	{0xC8,38,{0x7C,0x64,0x55,0x48,0x45,0x36,0x3C,0x27,0x40,0x3F,0x3F,0x5D,0x4C,0x53,0x46,0x43,0x35,0x21,0x06,0x7C,0x64,0x55,0x48,0x45,0x36,0x3C,0x27,0x40,0x3F,0x3F,0x5D,0x4C,0x53,0x46,0x43,0x35,0x21,0x06}},
	{0xC6,8,{0x00,0x00,0x68,0x00,0x00,0x60,0x36,0x00}},
	{0xF3,1,{0x12}},

	{0x11,1,{0x00}},
	{REGFLAG_DELAY,120,{}},
	{0x29,1,{0x00}}, 
	{REGFLAG_DELAY,20,{}},				   
	{REGFLAG_END_OF_TABLE, 0x00, {}} 
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

		//bingqian.tang date20160413 add for GGAFMA-394
		#ifdef  CONFIG_PROJECT_P6601_WIK_FR
        	params->physical_width  = 62.10; //62.10;
        	params->physical_height = 110.40; //110.40;
		#endif
		//bingqian.tang end

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
		
		params->dsi.vertical_sync_active = 6;
		params->dsi.vertical_backporch = 18;
		params->dsi.vertical_frontporch	= 10;
		params->dsi.vertical_active_line = FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active = 30;
		params->dsi.horizontal_backporch = 60;
		params->dsi.horizontal_frontporch = 60;
		params->dsi.horizontal_active_pixel = FRAME_WIDTH;

		// Bit rate calculation
		//params->dsi.PLL_CLOCK=205;
		params->dsi.PLL_CLOCK=214;  //LINE </EGAFM-297> <change the mipi clock to reduce disturbing the wifi> <20160413> panzaoyan
		params->dsi.ssc_disable=0;
		
		//yixuhong 20150511 add esd check function
#ifndef BUILD_LK	
	params->dsi.esd_check_enable = 1; 
	params->dsi.customization_esd_check_enable = 1;//0:te esd check 1:read register
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
	
	//params->dsi.lcm_esd_check_table[1].cmd = 0x0B;
//	params->dsi.lcm_esd_check_table[1].count = 1;
//	params->dsi.lcm_esd_check_table[1].para_list[0] = 0x00;
	params->dsi.lcm_esd_check_table[1].cmd = 0x0D;
	params->dsi.lcm_esd_check_table[1].count = 1;
	params->dsi.lcm_esd_check_table[1].para_list[0] = 0x00;
#endif /*BUILD_LK*/

}

static unsigned int lcm_compare_id(void);

static void lcm_init(void)
{
	LCM_DBG(); 
	SET_RESET_PIN(1);
	MDELAY(20);
#ifdef GPIO_LCM_PWR
	SET_PWR_PIN(0);
	MDELAY(20);
	SET_PWR_PIN(1);
	MDELAY(150);
#endif	
	SET_RESET_PIN(0);
	MDELAY(10); 
	SET_RESET_PIN(1);
	MDELAY(120);
	LCM_DBG("jacky debug,lcm reset end \n");

	push_table(lcm_initialization_setting_chipone, sizeof(lcm_initialization_setting_chipone) / sizeof(struct LCM_setting_table), 1);
	
	LCM_DBG("jacy debug,lcm init end \n");

	//lcm_compare_id();
}

static void lcm_suspend(void)
{	
	LCM_DBG();
    push_table(lcm_deep_sleep_mode_in_setting_v2, sizeof(lcm_deep_sleep_mode_in_setting_v2) / sizeof(struct LCM_setting_table), 1);
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(20); // 1ms

    SET_RESET_PIN(1);
    MDELAY(120);
}

static void lcm_resume(void)
{
	LCM_DBG();
	SET_RESET_PIN(0);
	MDELAY(10); 
	SET_RESET_PIN(1);
	MDELAY(120); 
	push_table(lcm_initialization_setting_chipone, sizeof(lcm_initialization_setting_chipone) / sizeof(struct LCM_setting_table), 1);
}      
static struct LCM_setting_table read_lcm_id[] = 
{
	{0xFF,3,{0x98,0x81,0x01}}
};

static unsigned int lcm_compare_id(void)
{
#if 1
	s32 lcd_hw_id = -1;
	char  buffer[3]   = {0};
	//lcd_hw_id = mt_get_gpio_in(GPIO_LCD_ID);
	lcd_hw_id = get_lcm_id_status();
	LCM_DBG("lcm_compare_id lcd_hw_id=%d \n",lcd_hw_id);
	if (1==lcd_hw_id)
	{
		return 1;
	}
	else
	{
		return 0;
	}
#else
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
		if((BOE_LCM_MIN_VOL <= iVoltage) && (iVoltage < BOE_LCM_MAX_VOL))
				return 1;
			else
				return 0;
		}
		return 0;
#endif
}


static unsigned int lcm_esd_check(void)
{
  #ifndef BUILD_LK
	char  buffer[3];
	int   array[4];

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x0A, buffer, 1);

	LCM_DBG("%s:esd buffer = 0x%x\n",__func__, buffer[0]);
	
	if(buffer[0]==0x9c)
	{
		return FALSE;
	}
	else
	{			 
		return TRUE;
	}
#else
	return FALSE;
#endif

}

static unsigned int lcm_esd_recover(void)
{
	LCM_DBG();
	lcm_init();
	return TRUE;
}


LCM_DRIVER ili9881c_v3971_hd720_dsi_vdo_boe_chipone_lcm_drv = 
{
	.name		= "ili9881c_hd720_dsi_vdo_boe_chipone",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
};

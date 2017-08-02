#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
#endif


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(854)

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xDD   // END OF REGISTERS MARKER
#define LCM_ID       (0x9806)


//#define LCM_DSI_CMD_MODE									0

#ifndef TRUE
    #define   TRUE     1
#endif
 
#ifndef FALSE
    #define   FALSE    0
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
	LCM_PRINT("[LCM_ILI9806E_FWVGA_DSI_VDO_HLT] %s (line:%d) :" fmt "\r\n", __func__, __LINE__, ## arg)

// zx  add for lcm detect ,read adc voltage
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
extern int auxadc_test(void ) ;
// #define AUXADC_LCM_VOLTAGE_CHANNEL     12
#define AUXADC_ADC_FDD_RF_PARAMS_DYNAMIC_CUSTOM_CH_CHANNEL     1
#define MIN_VOLTAGE (600)     // zx  add for lcm detect

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (480)
#define FRAME_HEIGHT (854)

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))

static unsigned int lcm_compare_id(void);
// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)										lcm_util.dsi_read_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
 
 struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] =
{

	{0xFF,0x5,{0xFF,0x98,0x06,0x04,0x01}},
	{0x08,0x1,{0x10}},

	{0x21,0x1,{0x01}},
	{0x30,0x1,{0x01}},
	{0x31,0x1,{0x00}},  //COLUMN
	{0x40,0x1,{0x15}},
	{0x41,0x1,{0x33}},
	{0x42,0x1,{0x03}},
	{0x43,0x1,{0x09}},
	{0x44,0x1,{0x09}},
	{0x46,0x1,{0x44}},
	{0x50,0x1,{0x88}},
	{0x51,0x1,{0x88}},
	{0x52,0x1,{0x00}},
	{0x53,0x1,{0x28}},
	{0x57,0x1,{0x50}},
	{0x60,0x1,{0x0a}},
	{0x61,0x1,{0x00}},
	{0x62,0x1,{0x08}},
	{0x63,0x1,{0x00}},


	{0xA0,0x1,{0x00}},  //0
	{0xA1,0x1,{0x05}},   //4
	{0xA2,0x1,{0x0f}},  //8
	{0xA3,0x1,{0x11}},  //16
	{0xA4,0x1,{0x0a}},  //24
	{0xA5,0x1,{0x19}},  //52
	{0xA6,0x1,{0x08}},  //80
	{0xA7,0x1,{0x06}},  //108
	{0xA8,0x1,{0x03}},  //147
	{0xA9,0x1,{0x08}},  //175
	{0xAA,0x1,{0x0b}},  //203
	{0xAB,0x1,{0x06}},  //231
	{0xAC,0x1,{0x0b}},  //239
	{0xAD,0x1,{0x29}},  //247
	{0xAE,0x1,{0x26}},  //251
	{0xAF,0x1,{0x00}},


	{0xC0,0x1,{0x00}},
	{0xC1,0x1,{0x05}},
	{0xC2,0x1,{0x11}},
	{0xC3,0x1,{0x11}},
	{0xC4,0x1,{0x0b}},
	{0xC5,0x1,{0x1d}},
	{0xC6,0x1,{0x0d}},
	{0xC7,0x1,{0x0b}},
	{0xC8,0x1,{0x02}},
	{0xC9,0x1,{0x06}},
	{0xCA,0x1,{0x00}},
	{0xCB,0x1,{0x05}},
	{0xCC,0x1,{0x0e}},
	{0xCD,0x1,{0x28}},
	{0xCE,0x1,{0x24}},
	{0xCF,0x1,{0x00}},
		


	{0xFF,0x5,{0xFF,0x98,0x06,0x04,0x06}},
	{0x00,0x1,{0x21}},
	{0x01,0x1,{0x09}},
	{0x02,0x1,{0x00}},
	{0x03,0x1,{0x00}},
	{0x04,0x1,{0x01}},
	{0x05,0x1,{0x01}},
	{0x06,0x1,{0x80}},
	{0x07,0x1,{0x05}},
	{0x08,0x1,{0x02}},
	{0x09,0x1,{0x80}},
	{0x0A,0x1,{0x00}},
	{0x0B,0x1,{0x00}},
	{0x0C,0x1,{0x0a}},
	{0x0D,0x1,{0x0a}},
	{0x0E,0x1,{0x00}},
	{0x0F,0x1,{0x00}},
	{0x10,0x1,{0xe0}},
	{0x11,0x1,{0xe4}},
	{0x12,0x1,{0x04}},
	{0x13,0x1,{0x00}},
	{0x14,0x1,{0x00}},
	{0x15,0x1,{0xC0}},
	{0x16,0x1,{0x08}},
	{0x17,0x1,{0x00}},
	{0x18,0x1,{0x00}},
	{0x19,0x1,{0x00}},
	{0x1A,0x1,{0x00}},
	{0x1B,0x1,{0x00}},
	{0x1C,0x1,{0x00}},
	{0x1D,0x1,{0x00}},
	{0x20,0x1,{0x01}},
	{0x21,0x1,{0x23}},
	{0x22,0x1,{0x45}},
	{0x23,0x1,{0x67}},
	{0x24,0x1,{0x01}},
	{0x25,0x1,{0x23}},
	{0x26,0x1,{0x45}},
	{0x27,0x1,{0x67}},
	{0x30,0x1,{0x01}},
	{0x31,0x1,{0x11}},
	{0x32,0x1,{0x00}},
	{0x33,0x1,{0xEE}},
	{0x34,0x1,{0xFF}},
	{0x35,0x1,{0xBB}},
	{0x36,0x1,{0xCA}},
	{0x37,0x1,{0xDD}},
	{0x38,0x1,{0xAC}},
	{0x39,0x1,{0x76}},
	{0x3A,0x1,{0x67}},
	{0x3B,0x1,{0x22}},
	{0x3C,0x1,{0x22}},
	{0x3D,0x1,{0x22}},
	{0x3E,0x1,{0x22}},
	{0x3F,0x1,{0x22}},
	{0x40,0x1,{0x22}},
	{0x52,0x1,{0x10}},
	{0x53,0x1,{0x10}},
	{0x54,0x1,{0x13}},
		

	{0xFF,0x5,{0xFF,0x98,0x06,0x04,0x07}},
	{0x17,0x1,{0x22}},
	{0x02,0x1,{0x77}},
	{0xE1,0x1,{0x79}},
	//{0x06,0x1,0x13
	//delay,0x10
	{0xB3,0x1,{0x10}},
	{0x26,0x1,{0xB2}},


	{0xFF,0x5,{0xFF,0x98,0x06,0x04,0x00}},
	{0x55,0x1,{0xB0}},
	{0x35,1,{0x00}}, // TE on
	{0x11,0x01,{0x0}},
	{REGFLAG_DELAY, 200, {}},
	{0x29,0x01,{0x0}},


		 // Note
	 // Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.
	 // Setting ending by predefined flag
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};




static struct LCM_setting_table lcm_set_window[] = {

};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
	//Normal mode on
//	{0x13, 1, {0x00}},
//	{REGFLAG_DELAY,20,{}},
    // Sleep Out
	{0x11, 1, {0x00}},
             {REGFLAG_DELAY, 150, {}},
    // Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Sleep Mode On
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 50, {}},
    
    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 20, {}},
    
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_backlight_level_setting[] = {

};

static struct LCM_setting_table lcm_compare_id_setting[] = {

	{0xD3,	3,	{0xFF, 0x83, 0x79}},
	{REGFLAG_DELAY, 10, {}}, 	

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
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


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

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
		
    params->physical_width  = 61.63;
    params->physical_height = 109.65;
    
		// enable tearing-free
		//params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;
		//params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif
	
	
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_TWO_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=256;

		// Video mode setting		
		params->dsi.intermediat_buffer_num = 2;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		params->dsi.word_count=480*3;

 //edit by Magnum 2013-7-25 , solve esd read id error
	//	 cycle_time = (4 * 1000 * div2 * div1 * pre_div * post_div)/ (fbk_sel * (fbk_div+0x01) * 26) + 
	// 1 = 
  // ui = (1000 * div2 * div1 * pre_div * post_div)/ (fbk_sel * (fbk_div+0x01) * 26 * 2) + 1;
		
		params->dsi.vertical_sync_active				= 7;
		params->dsi.vertical_backporch				= 8;
		params->dsi.vertical_frontporch				= 8;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 6;//8;
		params->dsi.horizontal_backporch				= 85;//50;
		params->dsi.horizontal_frontporch				= 76; //46;
		params->dsi.horizontal_active_pixel			= FRAME_WIDTH;
		params->dsi.compatibility_for_nvk = 0;	
		params->dsi.ssc_disable=1;
		params->dsi.ssc_range=2;
		params->dsi.PLL_CLOCK				= 215;//170;//222;		

		params->dsi.esd_check_enable = 1;
		params->dsi.customization_esd_check_enable = 1;
		params->dsi.lcm_esd_check_table[0].cmd          = 0x0A;
		params->dsi.lcm_esd_check_table[0].count        = 1;
		params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;

		/* ESD or noise interference recovery For video mode LCM only. */
		// Send TE packet to LCM in a period of n frames and check the response.
	/*	params->dsi.lcm_int_te_monitor = FALSE;
		params->dsi.lcm_int_te_period = 1;		// Unit : frames

		// Need longer FP for more opportunity to do int. TE monitor applicably.
		if(params->dsi.lcm_int_te_monitor)
			params->dsi.vertical_frontporch *= 2;
		
		// Monitor external TE (or named VSYNC) from LCM once per 2 sec. (LCM VSYNC must be wired to baseband TE pin.)
		params->dsi.lcm_ext_te_monitor = FALSE;
		// Non-continuous clock
		params->dsi.noncont_clock = TRUE;
		params->dsi.noncont_clock_period = 2;	// Unit : frames  */
}


static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(25);
    SET_RESET_PIN(1);
    MDELAY(120);
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
//return;
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
     MDELAY(60); // 60ms for ps function zx
     SET_RESET_PIN(0);
}


static void lcm_resume(void)
{
	lcm_init();
	
	//push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}


static unsigned int lcm_compare_id(void)
{
    int array[4];
    char buffer[5];
    char id_high=0;
    char id_midd=0;
    char id_low=0;
    int id=0;

    int data[4] = {0,0,0,0};
    int res = 0;
    int rawdata = 0;
    int lcm_vol = 0;


    //Do reset here
    SET_RESET_PIN(1);
    MDELAY(2);
    SET_RESET_PIN(0);
    MDELAY(25);       
    SET_RESET_PIN(1);
    MDELAY(120);   

    array[0]=0x00063902;
    array[1]=0x0698ffff;
    array[2]=0x00000104;
    dsi_set_cmdq(array, 3, 1);
    MDELAY(10);

    array[0]=0x00023700;//0x00023700;
    dsi_set_cmdq(array, 1, 1);
    //read_reg_v2(0x04, buffer, 3);//if read 0x04,should get 0x008000,that is both OK.

    read_reg_v2(0x00, buffer,1);
    id_high = buffer[0]; ///////////////////////0x98
    LCM_DBG("ILI9806_HLT:  id_high =%x\n", id_high);
    MDELAY(2);
    read_reg_v2(0x01, buffer,1);
    id_midd = buffer[0]; ///////////////////////0x06
    LCM_DBG("ILI9806_HLT:  id_midd =%x\n", id_midd);
    MDELAY(2);
    // read_reg_v2(0x02, buffer,1);
    read_reg_v2(0xDA, buffer,1);
    id_low = buffer[0]; ////////////////////////0x04
    LCM_DBG("ILI9806_HLT:  id_low =%x\n", id_low);

    //	id = id_high;
    id = (id_high << 8) | id_midd;
    LCM_DBG("ILI9806_HLT: id = %x\n", id);

#ifndef AUXADC_LCM_VOLTAGE_CHANNEL
    LCM_DBG("ILI9806_HLT: No need to check id pin voltage, id = %x(target=%x)\n", id, LCM_ID);
    return (LCM_ID == id) ? 1 : 0;
#else
    if (LCM_ID != id) {
        LCM_DBG("ILI9806_HLT: not match id, id = %x\n", id);
        return 0;        
    }

    res = IMM_GetOneChannelValue(AUXADC_LCM_VOLTAGE_CHANNEL,data,&rawdata);
    if(res < 0)
    {
        LCM_DBG("ILI9806_HLT: [adc_uboot]: get data error\n");
        return 0;
    }
    
    lcm_vol = data[0]*1000+data[1]*10;
    LCM_DBG("ILI9806_HLT: [adc_uboot]: lcm_vol= %d\n",lcm_vol);

    if ((LCM_ID == id)&&(lcm_vol > MIN_VOLTAGE)){
        LCM_DBG("ILI9806_HLT: [adc_uboot]: find lcm id = %x lcm_vol = %d\n",id,lcm_vol);
        return 1;
    }else{
        return 0;
    }
#endif
}


LCM_DRIVER ili9806e_fwvga_dsi_vdo_hlt_tn_lcm_drv = 
{
    .name			= "ili9806e_fwvga_dsi_vdo_hlt_tn",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = lcm_compare_id,	
#if (LCM_DSI_CMD_MODE)
	.set_backlight	= lcm_setbacklight,
    .update         = lcm_update,
#endif
};


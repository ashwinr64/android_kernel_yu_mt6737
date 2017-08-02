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

#define FRAME_WIDTH                                          (480)
#define FRAME_HEIGHT                                         (854)

#define REGFLAG_DELAY                                         0XFE
#define REGFLAG_END_OF_TABLE                                  0xDD   // END OF REGISTERS MARKER
#define LCM_ID       (0x7701)


//#define LCM_DSI_CMD_MODE                                    0

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
    LCM_PRINT("[LCM_ST7701_FWVGA_DSI_VDO_HZ] %s (line:%d) :" fmt "\r\n", __func__, __LINE__, ## arg)

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

#define SET_RESET_PIN(v)                                    (lcm_util.set_reset_pin((v)))

#define UDELAY(n)                                             (lcm_util.udelay(n))
#define MDELAY(n)                                             (lcm_util.mdelay(n))

static unsigned int lcm_compare_id(void);
// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)        lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                        lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                    lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                        lcm_util.dsi_read_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)                lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
 
 struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] =
{
    // SET_RESET_PIN(1);
    // MDELAY(10);
    // SET_RESET_PIN(0);
    // MDELAY(10);
    // SET_RESET_PIN(1);
    // MDELAY(120);

    {0x01,0,{0x00}},
    {REGFLAG_DELAY,120,{}},
    {0xFF,5,{0x77,0x01,0x00,0x00,0x11}},
    {0xD1,1,{0x11}},
    {0x11,0,{0x00}},
    {REGFLAG_DELAY,120,{}},
    {0xFF,5,{0x77,0x01,0x00,0x00,0x10}},
    {0xC0,2,{0xE9,0x03}},
    {0xC1,2,{0x11,0x02}},
    {0xC2,2,{0x31,0x08}},

    {0xB0,16,{0x03,0x04,0xC1,0x0C,0x13,0x08,0x05,0x0C,0x0E,0x1C,0x08,0x11,0x0A,0x10,0x11,0x19}},
    {0xB1,16,{0x00,0x03,0x07,0x0E,0x13,0x07,0x02,0x06,0x06,0x26,0x0A,0x1F,0x1F,0xD9,0x1D,0x19}},
    {0xFF,5,{0x77,0x01,0x00,0x00,0x11}},
    {0xB0,1,{0x5D}},
    {0xB1,1,{0x4A}},
    {0xB2,1,{0x07}},
    {0xB3,1,{0x80}},
    {0xB5,1,{0x47}},
    {0xB7,1,{0x8A}},
    {0xB8,1,{0x10}},
    {0xC1,1,{0x78}},
    {0xC2,1,{0x78}},
    {0xD0,1,{0x88}},
    {REGFLAG_DELAY,100,{}},
    {0xE0,3,{0x00,0x00,0x02}},
    {0xE1,11,{0x08,0x00,0x0A,0x00,0x07,0x00,0x09,0x00,0x00,0x33,0x33}},
    {0xE2,13,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
    {0xE3,4,{0x00,0x00,0x33,0x33}},
    {0xE4,2,{0x44,0x44}},
    {0xE5,16,{0x0E,0x60,0xAF,0xAF,0x10,0x60,0xAF,0xAF,0x0A,0x60,0xAF,0xAF,0x0C,0x60,0xAF,0xAF}},
    {0xE6,4,{0x00,0x00,0x33,0x33}},
    {0xE7,2,{0x44,0x44}},
    {0xE8,16,{0x0D,0x60,0xAF,0xAF,0x0F,0x60,0xAF,0xAF,0x09,0x60,0xAF,0xAF,0x0B,0x60,0xAF,0xAF}},
    {0xEB,7,{0x02,0x01,0xE4,0xE4,0x44,0x00,0x40}},
    {0xEC,2,{0x02,0x01}},
    {0xED,16,{0xAB,0x89,0x76,0x54,0x01,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x10,0x45,0x67,0x98,0xBA}},
    {0xFF,5,{0x77,0x01,0x00,0x00,0x00}},
    {REGFLAG_DELAY,10,{}},
    {0x29,0,{0x00}},
    {REGFLAG_DELAY,50,{}},
    {REGFLAG_END_OF_TABLE,0x00,{}}
};




static struct LCM_setting_table lcm_set_window[] = {

};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Normal mode on
    // {0x13, 1, {0x00}},
    // {REGFLAG_DELAY,20,{}},
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
    {0xD3,    3,    {0xFF, 0x83, 0x79}},
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
    // params->dbi.te_mode                 = LCM_DBI_TE_MODE_VSYNC_ONLY;
    params->dbi.te_mode                 = LCM_DBI_TE_MODE_DISABLED;
    //  params->dbi.te_edge_polarity        = LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
    params->dsi.mode   = CMD_MODE;
#else
    params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif


    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM                = LCM_TWO_LANE;
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
    //     cycle_time = (4 * 1000 * div2 * div1 * pre_div * post_div)/ (fbk_sel * (fbk_div+0x01) * 26) + 
    // 1 = 
    // ui = (1000 * div2 * div1 * pre_div * post_div)/ (fbk_sel * (fbk_div+0x01) * 26 * 2) + 1;

    params->dsi.vertical_sync_active                = 10;
    params->dsi.vertical_backporch                    = 20;
    params->dsi.vertical_frontporch                    = 22;
    params->dsi.vertical_active_line                = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active                = 20;
    params->dsi.horizontal_backporch                = 60;
    params->dsi.horizontal_frontporch                = 60;
    params->dsi.horizontal_active_pixel                    = FRAME_WIDTH;
    params->dsi.compatibility_for_nvk = 0;    
    params->dsi.ssc_disable=1;
    params->dsi.ssc_range=2;
    params->dsi.LANE_NUM                = LCM_TWO_LANE;
    params->dsi.PLL_CLOCK = 190;

    params->dsi.esd_check_enable = 1;
    params->dsi.customization_esd_check_enable = 1;
    params->dsi.lcm_esd_check_table[0].cmd          = 0x0A;
    params->dsi.lcm_esd_check_table[0].count        = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;

    /* ESD or noise interference recovery For video mode LCM only. */
    // Send TE packet to LCM in a period of n frames and check the response.
    /*    params->dsi.lcm_int_te_monitor = FALSE;
    params->dsi.lcm_int_te_period = 1;        // Unit : frames

    // Need longer FP for more opportunity to do int. TE monitor applicably.
    if(params->dsi.lcm_int_te_monitor)
    params->dsi.vertical_frontporch *= 2;

    // Monitor external TE (or named VSYNC) from LCM once per 2 sec. (LCM VSYNC must be wired to baseband TE pin.)
    params->dsi.lcm_ext_te_monitor = FALSE;
    // Non-continuous clock
    params->dsi.noncont_clock = TRUE;
    params->dsi.noncont_clock_period = 2;    // Unit : frames  */
}


static void lcm_init(void)
{
    LCM_DBG("ST_7701_HZ: %s\n", __func__);
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
    LCM_DBG("ST_7701_HZ: %s\n", __func__);
    //return;
    push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
    MDELAY(60); // 60ms for ps function zx
    SET_RESET_PIN(0);
}


static void lcm_resume(void)
{
    LCM_DBG("ST_7701_HZ: %s\n", __func__);
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

    array[0]=0x00023700;
    dsi_set_cmdq(array, 1, 1);

    read_reg_v2(0xA1, buffer,3);
    id_high = buffer[0];
    LCM_DBG("ST_7701_HZ:  id_high =%x\n", id_high);
    MDELAY(2);

    // read_reg_v2(0xa2, buffer,1);
    id_midd = buffer[1]; 
    LCM_DBG("ST_7701_HZ:  id_midd =%x\n", id_midd);
    MDELAY(2);

    // read_reg_v2(0xDC, buffer,1);
    id_low = buffer[2];
    LCM_DBG("ST_7701_HZ:  id_low =%x\n", id_low);

    id = (id_high << 8) | id_midd;
    LCM_DBG("ST_7701_HZ: id = %x\n", id);

#ifndef AUXADC_LCM_VOLTAGE_CHANNEL
    LCM_DBG("ST_7701_HZ: No need to check id pin voltage, id = %x(target=%x)\n", id, LCM_ID);
    return (LCM_ID == id) ? 1 : 0;
#else
    if (LCM_ID != id)
    {
        LCM_DBG("ST_7701_HZ: not match, id = %x\n", id);
        return 0;
    }

    res = IMM_GetOneChannelValue(AUXADC_LCM_VOLTAGE_CHANNEL,data,&rawdata);
    if(res < 0)
    {
        LCM_DBG("ST_7701_HZ: [adc_uboot]: get data error\n");
        return 0;
    }
    
    lcm_vol = data[0]*1000+data[1]*10;
    LCM_DBG("ST_7701_HZ: [adc_uboot]: lcm_vol= %d\n",lcm_vol);

    if ((LCM_ID == id)&&(lcm_vol > MIN_VOLTAGE))
    {
        LCM_DBG("ST_7701_HZ: [adc_uboot]: find lcm id = %x lcm_vol = %d\n",id,lcm_vol);
        return 1;
    }else
    {
        return 0;
    }
#endif
}


LCM_DRIVER st7701_fwvga_dsi_vdo_hz_tn_lcm_drv = 
{
    .name           = "st7701_fwvga_dsi_vdo_hz_tn",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,    
#if (LCM_DSI_CMD_MODE)
    .set_backlight  = lcm_setbacklight,
    .update         = lcm_update,
#endif
};


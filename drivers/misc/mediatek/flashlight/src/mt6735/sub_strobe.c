
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#ifdef CONFIG_COMPAT
#include <linux/fs.h>
#include <linux/compat.h>
#endif
#include "kd_flashlight.h"
/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */
#define TAG_NAME "[sub_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_WARN(fmt, arg...)        pr_warn(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_NOTICE(fmt, arg...)      pr_notice(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_INFO(fmt, arg...)        pr_info(TAG_NAME "%s: " fmt, __func__ , ##arg)
#define PK_TRC_FUNC(f)              pr_debug(TAG_NAME "<%s>\n", __func__)
#define PK_TRC_VERBOSE(fmt, arg...) pr_debug(TAG_NAME fmt, ##arg)
#define PK_ERROR(fmt, arg...)       pr_err(TAG_NAME "%s: " fmt, __func__ , ##arg)

#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#define PK_VER PK_TRC_VERBOSE
#define PK_ERR PK_ERROR
#else
#define PK_DBG(a, ...)
#define PK_VER(a, ...)
#define PK_ERR(a, ...)
#endif
static DEFINE_SPINLOCK(g_strobeSMPLock_sub); /* cotta-- SMP proection */


static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;

static int gDuty=0;
static int g_timeOutTimeMs=0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(g_strobeSem_sub);
#else
static DECLARE_MUTEX(g_strobeSem_sub);
#endif

#define STROBE_DEVICE_ID 0x63

#define LM3643_REG_ENABLE       	0x01
#define LM3643_REG_FLASH_LED1  0x03
#define LM3643_REG_FLASH_LED2  0x04
#define LM3643_REG_TORCH_LED1  0x05
#define LM3643_REG_TORCH_LED2  0x06
#define LM3643_REG_TIMING           0x08

static struct work_struct workTimeOut_sub;

extern struct pinctrl *flashlightpinctrl;

extern struct pinctrl_state *flashlight_mode_h;
extern struct pinctrl_state *flashlight_mode_l ;
extern struct pinctrl_state *flashlight_en_h ;
extern struct pinctrl_state *flashlight_en_l ;
extern struct pinctrl_state *flashlight_ext1_h ;
extern struct pinctrl_state *flashlight_ext1_l ;

//#define FLASH_GPIO_ENE   GPIO_CAMERA_FLASH_EN_PIN
//#define FLASH_GPIO_ENS   GPIO_CAMERA_FLASH_MODE_PIN
//#define FLASH_GPIO_ENT   GPIO_CAMERA_FLASH_EXT1_PIN
enum
{
	e_SubDutyNum = 4,
};
static int subtorch_flag=0;
static int torchDuty[e_SubDutyNum]=    {35,71,106,127};
//50,100,150,179ma
static int flashDuty[e_SubDutyNum]=     {3,8,12,14};
//200,250,300,350,400,450,500,550,600,650,700,750,800,850,900,950,1000,1100,1200,1300,1400,1500ma

/*****************************************************************************
Functions
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data);
extern int LM3643_write_reg_ext(u8 reg, u8 val);
#if 0
int setDuty_LM3643_1(int duty)
{

	if(duty<0)
		duty=0;
	else if(duty>=e_DutyNum)
		duty=e_DutyNum-1;
	m_duty1=duty;
	
	if(subtorch_flag)
		writeReg(LM3643_REG_TORCH_LED2, torchDuty[duty]);
	else{
	       writeReg(LM3643_REG_FLASH_LED2, flashDuty[duty]);
	       PK_DBG(" setDuty_LM3643_1 line=%d\n",__LINE__);
		}
	return 0;

}
#endif

static int FL_Enable(void)
{
    int temp = 0;;
    PK_DBG("FL_enable sub :g_duty=%d \n",gDuty);
    PK_DBG("FL_enable sub :subtorch_flag=%d \n",subtorch_flag);
    
   pinctrl_select_state(flashlightpinctrl, flashlight_en_h);
 //   mt_set_gpio_out(FLASH_GPIO_ENE,GPIO_OUT_ONE);
    mdelay(20);

	LM3643_write_reg_ext(LM3643_REG_ENABLE,0x00);
	LM3643_write_reg_ext(LM3643_REG_TIMING, 0x1F);
	
	//temp=readReg(LM3643_REG_ENABLE);

	PK_DBG(" FL_Enable temp=%d\n",temp);

#if 0	
	if(subtorch_flag)
		 LM3643_write_reg_ext(LM3643_REG_ENABLE, temp|0x0a);
	else	
              LM3643_write_reg_ext(LM3643_REG_ENABLE, temp|0x0e);
#endif

	if(gDuty == 0)
	{
              LM3643_write_reg_ext(LM3643_REG_ENABLE, temp|0x0a);
              LM3643_write_reg_ext(LM3643_REG_TORCH_LED2, torchDuty[gDuty]);
	}
	else
	{
              LM3643_write_reg_ext(LM3643_REG_ENABLE, temp|0x0e);
              LM3643_write_reg_ext(LM3643_REG_FLASH_LED2, flashDuty[gDuty]);	
	}

    return 0;
}

static int FL_Disable(void)
{
	pinctrl_select_state(flashlightpinctrl, flashlight_en_h);

   // mt_set_gpio_out(FLASH_GPIO_ENE,GPIO_OUT_ONE);
    mdelay(20);

    LM3643_write_reg_ext(0x01, 0x00); 
	pinctrl_select_state(flashlightpinctrl, flashlight_en_l);
	pinctrl_select_state(flashlightpinctrl, flashlight_ext1_l);

 //   mt_set_gpio_out(FLASH_GPIO_ENE,GPIO_OUT_ZERO);
    //mt_set_gpio_out(FLASH_GPIO_ENT,GPIO_OUT_ZERO);
    PK_DBG(" FL_Disable line=%d\n",__LINE__);
    return 0;
}
static int FL_dim_duty(kal_uint32 duty)
{
    PK_DBG(" FL_dim_duty line=%d\n",__LINE__);
    gDuty =  duty;
    return 0;
}
static int FL_Init(void)
{
	pinctrl_select_state(flashlightpinctrl, flashlight_mode_l);
	pinctrl_select_state(flashlightpinctrl, flashlight_en_l);
	pinctrl_select_state(flashlightpinctrl, flashlight_ext1_l);

 /*   if(mt_set_gpio_mode(FLASH_GPIO_ENT,GPIO_MODE_00))    {PK_DBG("[constant_flashlight] set gpio mode failed!! \n");  }
    if(mt_set_gpio_dir(FLASH_GPIO_ENT,GPIO_DIR_OUT))     {PK_DBG("[constant_flashlight] set gpio dir failed!! \n");   }
    if(mt_set_gpio_out(FLASH_GPIO_ENT,GPIO_OUT_ZERO))    {PK_DBG("[constant_flashlight] set gpio failed!! \n");    }

    if(mt_set_gpio_mode(FLASH_GPIO_ENE,GPIO_MODE_00))    {PK_DBG("[constant_flashlight] set gpio mode failed!! \n");  }
    if(mt_set_gpio_dir(FLASH_GPIO_ENE,GPIO_DIR_OUT))     {PK_DBG("[constant_flashlight] set gpio dir failed!! \n");   }
    if(mt_set_gpio_out(FLASH_GPIO_ENE,GPIO_OUT_ZERO))    {PK_DBG("[constant_flashlight] set gpio failed!! \n");    }

    if(mt_set_gpio_mode(FLASH_GPIO_ENS,GPIO_MODE_00))    {PK_DBG("[constant_flashlight] set gpio mode failed!! \n");   }
    if(mt_set_gpio_dir(FLASH_GPIO_ENS,GPIO_DIR_OUT))     {PK_DBG("[constant_flashlight] set gpio dir failed!! \n");    }
    if(mt_set_gpio_out(FLASH_GPIO_ENS,GPIO_OUT_ZERO))    {PK_DBG("[constant_flashlight] set gpio failed!! \n");    }
*/
    PK_DBG(" FL_Init line=%d\n",__LINE__);
    return 0;
}

static int FL_Uninit(void)
{
    FL_Disable();
    return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data)
{
    FL_Disable();
    PK_DBG("ledTimeOut_callback\n");
}

static enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&workTimeOut_sub);
    return HRTIMER_NORESTART;
}
static struct hrtimer g_timeOutTimer;
static void timerInit(void)
{
    INIT_WORK(&workTimeOut_sub, work_timeOutFunc);
    g_timeOutTimeMs=1000; //1s
    hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
    g_timeOutTimer.function=ledTimeOutCallback;
}

static int sub_strobe_ioctl(unsigned int cmd, unsigned long arg)
{
	PK_DBG("sub dummy ioctl");
    int i4RetValue = 0;
    int ior_shift;
    int iow_shift;
    int iowr_shift;
    ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
    iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
    iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
    PK_DBG("RT4505 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%ld\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg);
    switch(cmd)
    {

        case FLASH_IOC_SET_TIME_OUT_TIME_MS:
            PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %ld\n",arg);
            g_timeOutTimeMs=arg;
		PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",arg);
		if(arg == 20000 || arg == 0)
			subtorch_flag = 1;
		else 
			subtorch_flag = 0;
		PK_DBG("FLASHLIGHT_TORCH_FLAG: %d\n",subtorch_flag);
		g_timeOutTimeMs=arg;            
            break;

        case FLASH_IOC_SET_DUTY :
            PK_DBG("FLASHLIGHT_DUTY: %ld\n",arg);
            FL_dim_duty(arg);
            break;

        case FLASH_IOC_SET_STEP:
            PK_DBG("FLASH_IOC_SET_STEP: %ld\n",arg);
            break;

        case FLASH_IOC_SET_ONOFF :
            PK_DBG("FLASHLIGHT_ONOFF: %ld\n",arg);
            if(arg==1)
            {
                if(g_timeOutTimeMs!=0)
                {
                    ktime_t ktime;
                    ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
                    hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
                }
                FL_Enable();
            }
            else
            {
                FL_Disable();
                hrtimer_cancel( &g_timeOutTimer );
            }
            break;
        default :
            PK_DBG(" No such command \n");
            i4RetValue = -EPERM;
            break;
    }
    return i4RetValue;
}

static int sub_strobe_open(void *pArg)
{
    PK_DBG("sub dummy open");

    int i4RetValue = 0;
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

    if (0 == strobe_Res)
    {
        FL_Init();
        timerInit();
    }
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
    spin_lock_irq(&g_strobeSMPLock_sub);

    if(strobe_Res)
    {
        PK_ERR(" busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }
    spin_unlock_irq(&g_strobeSMPLock_sub);

    return i4RetValue;

}

static int sub_strobe_release(void *pArg)
{
    PK_DBG("sub dummy release");

    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock_sub);

        strobe_Res = 0;
        strobe_Timeus = 0;

        /* LED On Status */
        g_strobe_On = FALSE;

        spin_unlock_irq(&g_strobeSMPLock_sub);

        FL_Uninit();
    }

    return 0;
}

FLASHLIGHT_FUNCTION_STRUCT subStrobeFunc = {
	sub_strobe_open,
	sub_strobe_release,
	sub_strobe_ioctl
};


MUINT32 subStrobeInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &subStrobeFunc;
	return 0;
}

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
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
//#include "kd_camera_hw.h"
//#include <cust_gpio_usage.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
//#include <linux/xlog.h>
#include <linux/version.h>
#include "kd_camera_typedef.h"

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
#include <linux/mutex.h>
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#endif

#include <linux/i2c.h>
#include <linux/leds.h>



/******************************************************************************
 * Debug configuration
******************************************************************************/
// availible parameter
// ANDROID_LOG_ASSERT
// ANDROID_LOG_ERROR
// ANDROID_LOG_WARNING
// ANDROID_LOG_INFO
// ANDROID_LOG_DEBUG
// ANDROID_LOG_VERBOSE
#define TAG_NAME "leds_strobe.c"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)


#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
	#define PK_DBG PK_DBG_FUNC
	#define PK_VER PK_TRC_VERBOSE
	#define PK_ERR PK_ERROR
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock); /* cotta-- SMP proection */


static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;

static int g_duty = -1;
static int g_timeOutTimeMs=0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(g_strobeSem);
#else
static DECLARE_MUTEX(g_strobeSem);
#endif


//#define STROBE_DEVICE_ID 0x63
#define STROBE_DEVICE_ID 0x31


static struct work_struct workTimeOut;

//#define FLASH_GPIO_ENF GPIO12
//#define FLASH_GPIO_ENT GPIO13

#define LM3643_REG_ENABLE       	0x01
#define LM3643_REG_FLASH_LED1  0x03
#define LM3643_REG_FLASH_LED2  0x04
#define LM3643_REG_TORCH_LED1  0x05
#define LM3643_REG_TORCH_LED2  0x06
#define LM3643_REG_TIMING           0x08

//jiangwei begin
static struct i2c_client *LM3643_i2c_client;

/*****************************************************************************
Functions
*****************************************************************************/
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
static void work_timeOutFunc(struct work_struct *data);

struct LM3643_chip_data {
	struct i2c_client *client;

	/* struct led_classdev cdev_flash; */
	/* struct led_classdev cdev_torch; */
	/* struct led_classdev cdev_indicator; */

	struct LM3643_platform_data *pdata;
	struct mutex lock;

	u8 last_flag;
	u8 no_pdata;
};

#if 0
int readReg(int reg)
{
    char buf[2];
    char bufR[2];
    buf[0]=reg;
    iReadRegI2C(buf , 1, bufR,1, STROBE_DEVICE_ID);
    PK_DBG("qq reg=%x val=%x qq\n", buf[0],bufR[0]);
    return (int)bufR[0];
}
#endif

static int LM3643_read_reg(struct i2c_client *client, u8 reg)
{
	int val = 0;
	struct LM3643_chip_data *chip = i2c_get_clientdata(client);

//	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
//	mutex_unlock(&chip->lock);


	return val;
}

int readReg(int reg)
{
	int val;
	val = LM3643_read_reg(LM3643_i2c_client, reg);
	return (int)val;
}

static int LM3643_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret = 0;
	struct LM3643_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		PK_DBG("failed writing at 0x%02x\n", reg);
	return ret;
}

int writeReg(int reg, int data)
{
    char buf[2];
    buf[0]=reg;
    buf[1]=data;
	
    iWriteRegI2C(buf, 2, STROBE_DEVICE_ID);

   return 0;
}
enum
{
	e_DutyNum = 22,
};
static int isMovieMode[e_DutyNum]={1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static int torchDuty[e_DutyNum]=    {35,71,106,127,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//50,100,150,179ma
static int flashDuty[e_DutyNum]=     {3,8,12,14,16,20,25,29,33,37,42,46,50,55,59,63,67,72,76,80,84,93,101,110,118,127};
//200,250,300,350,400,450,500,550,600,650,700,750,800,850,900,950,1000,1100,1200,1300,1400,1500ma
static int m_duty1=0;
static int m_duty2=0;
static int torch_flag=0;
int flashEnable_LM3643_1(void)
{
	int temp;
	
	temp=readReg(LM3643_REG_ENABLE);
	
	if(torch_flag)
		 writeReg(LM3643_REG_ENABLE, temp|0x09);
	else
              writeReg(LM3643_REG_ENABLE, temp|0x0D);
	return 0;
}
int flashDisable_LM3643_1(void)
{
	int temp;
	temp=readReg(LM3643_REG_ENABLE);
       writeReg(LM3643_REG_ENABLE, temp&0xFE);
      return 0;
}


int setDuty_LM3643_1(int duty)
{
//jiangwei remove temp
#if 0
	if(duty<0)
		duty=0;
	else if(duty>=e_DutyNum)
		duty=e_DutyNum-1;
	m_duty1=duty;
	
	if(torch_flag)
		writeReg(LM3643_REG_TORCH_LED1, torchDuty[duty]);
	else{
	       writeReg(LM3643_REG_FLASH_LED1, flashDuty[duty]);
	       PK_DBG(" setDuty_LM3643_1 line=%d\n",__LINE__);
		}
#endif	
	return 0;
}



int flashEnable_LM3643_2(void)
{
	int temp;
	
	temp=readReg(LM3643_REG_ENABLE);
	
	if(torch_flag)
		 writeReg(LM3643_REG_ENABLE, temp|0x0A);
	else
              writeReg(LM3643_REG_ENABLE, temp|0x0E);
	return 0;
}
int flashDisable_LM3643_2(void)
{
	int temp;
	temp=readReg(LM3643_REG_ENABLE);
       writeReg(LM3643_REG_ENABLE, temp&0xFD);

	return 0;
}


int setDuty_LM3643_2(int duty)
{

	if(duty<0)
		duty=0;
	else if(duty>=e_DutyNum)
		duty=e_DutyNum-1;
	m_duty2=duty;
	
	if(torch_flag)
		writeReg(LM3643_REG_TORCH_LED2, torchDuty[duty]);
	else{
		 
	       writeReg(LM3643_REG_FLASH_LED2, flashDuty[duty]);
              PK_DBG(" setDuty_LM3643_2 line=%d\n",__LINE__);
		}
	return 0;

}

int init_LM3643(void)
{
	int err;
	err =  writeReg(LM3643_REG_ENABLE,0x00);
      err =  writeReg(LM3643_REG_TIMING, 0x1F);
	return err;
}






struct LM3643_platform_data {
	u8 torch_pin_enable;	/* 1:  TX1/TORCH pin isa hardware TORCH enable */
	u8 pam_sync_pin_enable;	/* 1:  TX2 Mode The ENVM/TX2 is a PAM Sync. on input */
	u8 thermal_comp_mode_enable;	/* 1: LEDI/NTC pin in Thermal Comparator Mode */
	u8 strobe_pin_disable;	/* 1 : STROBE Input disabled */
	u8 vout_mode_enable;	/* 1 : Voltage Out Mode enable */
};



int LM3643_write_reg_ext(u8 reg, u8 val)
{
	return	LM3643_write_reg(LM3643_i2c_client,reg,val);
}

static int LM3643_chip_init(struct LM3643_chip_data *chip)
{
	return 0;
}

static int LM3643_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct LM3643_chip_data *chip;
	struct LM3643_platform_data *pdata = client->dev.platform_data;

	int err = -1;

	printk("YCC LM3643_probe start--->.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		PK_DBG("LM3643 i2c functionality check fail.\n");
		return err;
	}

	chip = kzalloc(sizeof(struct LM3643_chip_data), GFP_KERNEL);
	chip->client = client;

	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	if (pdata == NULL) {	/* values are set to Zero. */
		PK_DBG("LM3643 Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct LM3643_platform_data), GFP_KERNEL);
		chip->pdata = pdata;
		chip->no_pdata = 1;
	}

	chip->pdata = pdata;
	if (LM3643_chip_init(chip) < 0)
		goto err_chip_init;

	LM3643_i2c_client = client;
	printk("YCC LM3643 Initializing is done\n");

	return 0;

err_chip_init:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	printk("YCC LM3643 probe is failed\n");
	return -ENODEV;
}

static int LM3643_remove(struct i2c_client *client)
{
	struct LM3643_chip_data *chip = i2c_get_clientdata(client);

	if (chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);
	return 0;
}


#define LM3643_NAME "leds-LM3643"
static const struct i2c_device_id LM3643_id[] = {
	{LM3643_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id LM3643_of_match[] = {
	{.compatible = "mediatek,strobe_main"},
	{},
};
#endif

static struct i2c_driver LM3643_i2c_driver = {
	.driver = {
		   .name = LM3643_NAME,
#ifdef CONFIG_OF
		   .of_match_table = LM3643_of_match,
#endif
		   },
	.probe = LM3643_probe,
	.remove = LM3643_remove,
	.id_table = LM3643_id,
};
static int __init LM3643_init(void)
{
	printk("YCC LM3643_init\n");
	return i2c_add_driver(&LM3643_i2c_driver);
}

static void __exit LM3643_exit(void)
{
	i2c_del_driver(&LM3643_i2c_driver);
}


module_init(LM3643_init);
module_exit(LM3643_exit);

MODULE_DESCRIPTION("Flash driver for LM3643");
MODULE_AUTHOR("pw <pengwei@mediatek.com>");
MODULE_LICENSE("GPL v2");


//jiangwei end
#ifdef CONFIG_OF
static const struct of_device_id Flashlight_use_gpio_of_match[] = {
	{.compatible = "mediatek,strobe_gpio"},
	{},
};
#endif
struct pinctrl *flashlightpinctrl = NULL;

struct pinctrl_state *flashlight_mode_h = NULL;
struct pinctrl_state *flashlight_mode_l = NULL;
struct pinctrl_state *flashlight_en_h = NULL;
struct pinctrl_state *flashlight_en_l = NULL;
struct pinctrl_state *flashlight_ext1_h = NULL;
struct pinctrl_state *flashlight_ext1_l = NULL;

static int Flashlight_use_gpio_probe(struct platform_device *pdev)
{
    int ret = 0;
    struct task_struct *keyEvent_thread = NULL;
	printk("Flashlight_use_gpio_probe enter \n");


		flashlightpinctrl = devm_pinctrl_get(&pdev->dev);
		if (IS_ERR(flashlightpinctrl)) {
				PK_DBG("IS_ERR(flashlightpinctrl) \n");
		return -1;	
		}
		flashlight_mode_l= pinctrl_lookup_state(flashlightpinctrl, "flashlightpin_cfg0");
		if (IS_ERR(flashlight_mode_l)) {
			PK_DBG("IS_ERR(flashlight_mode_l) \n");
		return -1;	 
		}

	   flashlight_mode_h = pinctrl_lookup_state(flashlightpinctrl, "flashlightpin_cfg1");
	   if (IS_ERR(flashlight_mode_h)) {
	  	PK_DBG("IS_ERR(flashlight_mode_h) \n");
	   return -1;	
	   }
	   flashlight_en_l= pinctrl_lookup_state(flashlightpinctrl, "flashlightpin_en0");
	   if (IS_ERR(flashlight_en_l)) {
	   	PK_DBG("IS_ERR(flashlight_en_l) \n");
	   return -1;	
	   }
	   flashlight_en_h= pinctrl_lookup_state(flashlightpinctrl, "flashlightpin_en1");
	   if (IS_ERR(flashlight_en_h)) {
	   	PK_DBG("IS_ERR(flashlight_en_h) \n");
	   return -1;	
	   }
	   flashlight_ext1_l= pinctrl_lookup_state(flashlightpinctrl, "flashlightpin_ext10");
	   if (IS_ERR(flashlight_ext1_l)) {
	   	PK_DBG("IS_ERR(flashlight_ext1_l) \n");
	   return -1;	
	   }
	   
	   flashlight_ext1_h= pinctrl_lookup_state(flashlightpinctrl, "flashlightpin_ext11");
	   if (IS_ERR(flashlight_ext1_h)) {
	   	PK_DBG("IS_ERR(flashlight_ext1_h) \n");
	   return -1;	
	   }

	printk("Flashlight_use_gpio_probe exit\n");

    return 0;
}


static int Flashlight_use_gpio_remove(struct platform_device *dev)	
{
	return 0;
}

static struct platform_driver Flashlight_use_gpio_driver = {
	.probe	= Flashlight_use_gpio_probe,
	.remove  = Flashlight_use_gpio_remove,
	.driver    = {
	.name       = "flashlight",
	.of_match_table = Flashlight_use_gpio_of_match,	
	},
};

static int __init Flashlight_use_gpio_init(void)
{
	printk("Flashlight_use_gpio_init\n");
	//return i2c_add_driver(&Flashlight_use_gpio_i2c_driver);
	    platform_driver_register(&Flashlight_use_gpio_driver);
}

static void __exit Flashlight_use_gpio_exit(void)
{
	printk("Flashlight_use_gpio_exit\n");
	//i2c_del_driver(&Flashlight_use_gpio_i2c_driver);
	    platform_driver_unregister(&Flashlight_use_gpio_driver);
}



module_init(Flashlight_use_gpio_init);
module_exit(Flashlight_use_gpio_exit);

MODULE_DESCRIPTION("Flash driver for GPIO flashlight");
MODULE_AUTHOR("jack <jack.kang@tinno.com>");
MODULE_LICENSE("GPL v2");
int FL_Enable(void)
{
	int temp = 0;;

	PK_DBG("YCC FL_enable :g_duty=%d \n",g_duty);
	PK_DBG("YCC FL_enable :torch_flag=%d \n",torch_flag);
	
	PK_DBG(" FL_Enable line=%d\n",__LINE__);

	pinctrl_select_state(flashlightpinctrl, flashlight_en_h);
	pinctrl_select_state(flashlightpinctrl, flashlight_mode_h);

	LM3643_write_reg(LM3643_i2c_client,LM3643_REG_ENABLE,0x00);
	LM3643_write_reg(LM3643_i2c_client,LM3643_REG_TIMING, 0x1F);

#if 0
//	temp=readReg(LM3643_REG_ENABLE);

//	PK_DBG(" FL_Enable temp=%d\n",temp);
	
	if(torch_flag||g_duty)
		 LM3643_write_reg(LM3643_i2c_client,LM3643_REG_ENABLE, temp|0x09);
	else	
              LM3643_write_reg(LM3643_i2c_client,LM3643_REG_ENABLE, temp|0x0D);
#else
	if(g_duty == 0)
	{
			 LM3643_write_reg(LM3643_i2c_client,LM3643_REG_ENABLE, temp|0x09);
			 LM3643_write_reg(LM3643_i2c_client,LM3643_REG_TORCH_LED1, torchDuty[g_duty]);
	}
	else
	{
              LM3643_write_reg(LM3643_i2c_client,LM3643_REG_ENABLE, temp|0x0D);	
              LM3643_write_reg(LM3643_i2c_client,LM3643_REG_FLASH_LED1, flashDuty[g_duty]);	
	}
#endif	

	PK_DBG("exit FL_Enable line=%d\n",__LINE__);

    return 0;
}



int FL_Disable(void)
{
	pinctrl_select_state(flashlightpinctrl, flashlight_en_l);
		pinctrl_select_state(flashlightpinctrl, flashlight_ext1_l);
    //flashDisable_LM3643_1();//jiangwei before 
    LM3643_write_reg_ext(0x01, 0x00); 
	PK_DBG(" FL_Disable line=%d\n",__LINE__);
    return 0;
}

int FL_dim_duty(kal_uint32 duty)
{

    PK_DBG(" FL_dim_duty line=%d\n",__LINE__);
    PK_DBG(" FL_dim_duty duty=%d\n",duty);

    setDuty_LM3643_1(duty);//jiangwei before

     g_duty = duty;
    return 0;
}




int FL_Init(void)
{
	pinctrl_select_state(flashlightpinctrl, flashlight_mode_l);
	pinctrl_select_state(flashlightpinctrl, flashlight_en_l);
	pinctrl_select_state(flashlightpinctrl, flashlight_ext1_l);

    PK_DBG("enter  FL_Init line=%d\n",__LINE__);

    //init_LM3643();

    //LM3643_write_reg(LM3643_i2c_client,LM3643_REG_ENABLE,0x00);
    //LM3643_write_reg(LM3643_i2c_client,LM3643_REG_TIMING, 0x1F);
    
    INIT_WORK(&workTimeOut, work_timeOutFunc);
    PK_DBG("out  FL_Init line=%d\n",__LINE__);
    return 0;
}


int FL_Uninit(void)
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
    //printk(KERN_ALERT "work handler function./n");
}



enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&workTimeOut);
    return HRTIMER_NORESTART;
}
static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
  INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs=1000; //1s
	hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	g_timeOutTimer.function=ledTimeOutCallback;

}



static int constant_flashlight_ioctl(MUINT32 cmd, MUINT32 arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	PK_DBG("LM3643 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg);
    switch(cmd)
    {

		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",arg);
			if(arg == 20000 || arg == 0)
				torch_flag = 1;
			else 
				torch_flag = 0;
			PK_DBG("FLASHLIGHT_TORCH_FLAG: %d\n",torch_flag);
			g_timeOutTimeMs=arg;
		break;


    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("FLASHLIGHT_DUTY: %d\n",arg);
    		FL_dim_duty(arg);
    		break;


    	case FLASH_IOC_SET_STEP:
    		PK_DBG("FLASH_IOC_SET_STEP: %d\n",arg);

    		break;

    	case FLASH_IOC_SET_ONOFF :
    		PK_DBG("FLASHLIGHT_ONOFF: %d\n",arg);
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
    	case FLASH_IOC_SET_REG_ADR:
    	    break;
    	case FLASH_IOC_SET_REG_VAL:
    	    break;
    	case FLASH_IOC_SET_REG:
    	    break;
    	case FLASH_IOC_GET_REG:
    	    break;



		default :
    		PK_DBG(" No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
    return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
    int i4RetValue = 0;
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res)
	{
	    FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


    if(strobe_Res)
    {
		PK_DBG(" busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }


    spin_unlock_irq(&g_strobeSMPLock);
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

    return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
    PK_DBG(" constant_flashlight_release\n");

    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock);

        strobe_Res = 0;
        strobe_Timeus = 0;

        /* LED On Status */
        g_strobe_On = FALSE;

        spin_unlock_irq(&g_strobeSMPLock);

    	FL_Uninit();
    }

    PK_DBG(" Done\n");

    return 0;

}


FLASHLIGHT_FUNCTION_STRUCT	constantFlashlightFunc=
{
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &constantFlashlightFunc;
    }
    return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

    return 0;
}

EXPORT_SYMBOL(strobe_VDIrq);



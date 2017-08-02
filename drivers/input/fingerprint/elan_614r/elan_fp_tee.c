#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/of_gpio.h>
#include <linux/sched.h>
#include <linux/wakelock.h>

#ifdef CONFIG_MTK_CLKMGR
#include "mach/mt_clkmgr.h"
#else
#include <linux/clk.h>
#endif

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#endif
#include "mt_spi.h"
#include "mt_spi_hal.h"
#include "mt_gpio.h"
#include "mach/gpio_const.h"
//#include "../../../../spi/mediatek/mt6735/mt_spi.h"
#include <linux/miscdevice.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>

//#include "../fp_drv/fp_drv.h"

#include "elan_fp_tee.h"
#include <linux/completion.h>
#include "elan_fp_tee.h"
#include <linux/platform_device.h>
#include "../fp_drv/fp_drv.h"

//#define reinit_completion(x) INIT_COMPLETION(*(x))
#define VERSION_LOG	"ELAN FINGER PRINT V1.4.4.1"

#define _ELAN_DEBUG_
#ifdef _ELAN_DEBUG_
	static int elan_debug = 1;
	#define ELAN_DEBUG(format, args ...) \
			do { \
					if (elan_debug) \
							printk(KERN_ERR "[ELAN] " format, ##args); \
			} while (0)		 
#else
         #define ELAN_DEBUG(format, args ...)
#endif

#define GPIO_FP_ID	880
#define KEY_FP_INT			KEY_POWER //KEY_WAKEUP // change by customer & framework support
#define KEY_FP_INT2			KEY_1 // change by customer & framework support
#define SPI_MAX_SPEED		3*1000*1000

struct completion cmd_done_irq;
static int factory_status = 0;
static DEFINE_MUTEX(elan_factory_mutex);
static struct fasync_struct *fasync_queue = NULL;

struct efsa120s_data  {
	int 					irq_gpio;
	int						isr;
	int 					rst_gpio;
	int						irq_is_disable;
	struct miscdevice		efsa120_dev;	/* char device for ioctl */
	//struct spi_device	*pdev;
	struct platform_device	*pdev;
	struct input_dev		*input_dev;
	spinlock_t				irq_lock;
	wait_queue_head_t		efsa_wait;
	struct wake_lock		wake_lock;
	struct regulator *reg;
	struct pinctrl *pinctrl1;
	struct pinctrl_state *pins_default;
#if 0
	struct pinctrl_state *eint_as_int, *eint_in_low, *eint_in_float, *fp_rst_low, *fp_rst_high,*pins_miso_spi,*pins_miso_pullhigh,*pins_miso_pulllow;
#else
	struct pinctrl_state *eint_as_int, *eint_in_low, *eint_in_float, *fp_rst_low, *fp_rst_high,*miso_pull_up,*miso_pull_disable;
	struct pinctrl_state *fp_enable_low, *fp_enable_high;
#endif
};

static struct efsa120s_data *elan_fp = NULL;

static void elan_spi_clk_enable(struct efsa120s_data *fp, u8 bonoff);

void efsa120s_irq_enable(void *_fp)
{
	struct efsa120s_data *fp = _fp;	
	unsigned long irqflags = 0;
	ELAN_DEBUG("IRQ Enable = %d.\n", fp->isr);
  
	spin_lock_irqsave(&fp->irq_lock, irqflags);
	if (fp->irq_is_disable) 
	{
		enable_irq(fp->isr);
		fp->irq_is_disable = 0; 
	}
	spin_unlock_irqrestore(&fp->irq_lock, irqflags);
}

void efsa120s_irq_disable(void *_fp)
{
	struct efsa120s_data *fp = _fp;
	unsigned long irqflags;
	ELAN_DEBUG("IRQ Disable = %d.\n", fp->isr);

	spin_lock_irqsave(&fp->irq_lock, irqflags);
	if (!fp->irq_is_disable)
	{
		fp->irq_is_disable = 1; 
		disable_irq_nosync(fp->isr);
	}
	spin_unlock_irqrestore(&fp->irq_lock, irqflags);
}

static ssize_t show_drv_version_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", VERSION_LOG);
}
static DEVICE_ATTR(drv_version, S_IRUGO, show_drv_version_value, NULL);

#ifdef _ELAN_DEBUG_
static ssize_t elan_debug_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	if(elan_debug){
		elan_debug=0;
	} else {
		elan_debug=1;
	}
	return sprintf(buf, "[ELAN] elan debug %d\n", elan_debug);
}
static DEVICE_ATTR(elan_debug, S_IRUGO, elan_debug_value, NULL);
#endif

static struct attribute *efsa120s_attributes[] = {
	&dev_attr_drv_version.attr,
#ifdef _ELAN_DEBUG_
	&dev_attr_elan_debug.attr,
#endif
	NULL
};

static struct attribute_group efsa120s_attr_group = {
	.attrs = efsa120s_attributes,
};

static void efsa120s_reset_output(struct efsa120s_data *fp, int level)
{
	//printk("[efsa120s]efsa120s_reset_output level = %d   ,%d,   %d\n", level,elan_fp->pinctrl1,elan_fp->fp_rst_low);

	if (level)
		pinctrl_select_state(fp->pinctrl1, fp->fp_rst_high);
	else
		pinctrl_select_state(fp->pinctrl1, fp->fp_rst_low);
}

static void efsa120s_reset(struct efsa120s_data *fp)
{
	/* Developement platform */
	efsa120s_reset_output(fp,0);
	mdelay(5);
	efsa120s_reset_output(fp,1);
	mdelay(50);
}

static int efsa120s_open(struct inode *inode, struct file *filp)
{
	struct efsa120s_data *fp = container_of(filp->private_data, struct efsa120s_data, efsa120_dev);	
	filp->private_data = fp;
	ELAN_DEBUG("%s()\n", __func__);
	return 0;
}

static int efsa120s_close(struct inode *inode, struct file *filp)
{
	ELAN_DEBUG("%s()\n", __func__);
	return 0;
}

static long efsa120s_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct efsa120s_data *fp = filp->private_data;
    int keycode;

	ELAN_DEBUG("%s() : cmd = [%04X]\n", __func__, cmd);

	switch(cmd)
	{
		case ID_IOCTL_RESET: //6
			efsa120s_reset(fp);
			ELAN_DEBUG("ID_IOCTL_RESET\n");
			break;
		case ID_IOCTL_POLL_INIT: //20
			reinit_completion(&cmd_done_irq);
			ELAN_DEBUG("ID_IOCTL_POLL_INIT\n");
			break;
		case ID_IOCTL_POLL_EXIT: //23
			complete(&cmd_done_irq);
			ELAN_DEBUG("ID_IOCTL_POLL_EXIT\n");
			break;
        case ID_IOCTL_INPUT_KEYCODE: //add input keycode by herman KEY_CAMERA = 212
			keycode =(int __user)arg;
			ELAN_DEBUG("%s() : ID_IOCTL_INPUT_KEYCODE check keycode = %d \n",__func__, keycode);
			if (!keycode) {
				ELAN_DEBUG("Keycode %d not defined, ignored.\n", (int __user)arg);
				break ;
			}
			input_report_key(fp->input_dev, keycode, 1); // Added for KEY Event
			input_sync(fp->input_dev);
			input_report_key(fp->input_dev, keycode, 0); // Added for KEY Event
			input_sync(fp->input_dev);
			break;	
			
		case ID_IOCTL_SET_KEYCODE: //add for set keycode by herman KEY_CAMERA = 212
			keycode =(int __user)arg;
			ELAN_DEBUG("%s() : ID_IOCTL_SET_KEYCODE check keycode = %d \n",__func__, keycode);
			if (!keycode) {
				ELAN_DEBUG("Keycode %d not defined, ignored.\n", (int __user)arg);
				break ;
			}
			input_set_capability(fp->input_dev, EV_KEY, keycode); 
			set_bit(keycode, fp->input_dev->keybit);	
			break;
        case ID_IOCTL_INPUT_KEYCODE_DOWN:
            keycode =(int __user)arg;
            ELAN_DEBUG("%s() : ID_IOCTL_INPUT_KEYCODE_DOWN check keycode = %d \n",__func__, keycode);
            if(!keycode) {
                ELAN_DEBUG("Keycode %d not defined, ignored.\n", (int __user)arg);
				break ;
            }
            input_report_key(fp->input_dev, keycode, 1); // Added for KEY Event
			input_sync(fp->input_dev);
            break;
        case ID_IOCTL_INPUT_KEYCODE_UP:
            keycode =(int __user)arg;
            ELAN_DEBUG("%s() : ID_IOCTL_INPUT_KEYCODE_UP check keycode = %d \n",__func__, keycode);
            if(!keycode) {
                ELAN_DEBUG("Keycode %d not defined, ignored.\n", (int __user)arg);
				break ;
            }
            input_report_key(fp->input_dev, keycode, 0); // Added for KEY Event
			input_sync(fp->input_dev);
            break;
        case ID_IOCTL_READ_FACTORY_STATUS:
            mutex_lock(&elan_factory_mutex);
            ELAN_DEBUG("READ_FACTORY_STATUS = %d", factory_status);
            mutex_unlock(&elan_factory_mutex);
            return factory_status;
            break;

        case ID_IOCTL_WRITE_FACTORY_STATUS:
            mutex_lock(&elan_factory_mutex);
            factory_status = (int __user)arg;
            ELAN_DEBUG("WRITE_FACTORY_STATUS = %d\n", factory_status);
            mutex_unlock(&elan_factory_mutex);
            break;
		case ID_IOCTL_EN_IRQ: //55
			efsa120s_irq_enable(fp);
			ELAN_DEBUG("ID_IOCTL_EN_IRQ\n");
			break;
		case ID_IOCTL_DIS_IRQ: //66
			efsa120s_irq_disable(fp);
			ELAN_DEBUG("ID_IOCTL_DIS_IRQ\n");
			break;
		case ID_IOCTL_ENABLE_SPI_CLK:
			ELAN_DEBUG("%s() : ELAN_IOC_ENABLE_SPI_CLK ======\n", __func__);
			elan_spi_clk_enable(fp, 1);
			break;
		case ID_IOCTL_DISABLE_SPI_CLK:
			ELAN_DEBUG("%s() : ID_IOCTL_DISABLE_SPI_CLK ======\n", __func__);
			elan_spi_clk_enable(fp, 0);
			break;
		default:
			ELAN_DEBUG("INVALID COMMAND\n");
			break;
	}
	return 0;
}

static unsigned int efsa120s_poll(struct file *file, poll_table *wait)
{
	struct efsa120s_data *fp = file->private_data;
	int mask=0; 
	wait_for_completion_interruptible(&cmd_done_irq);
	poll_wait(file, &fp->efsa_wait, wait);
	mask |= POLLIN | POLLRDNORM;
	return mask;
}

static int elan_fp_fasync(int fd, struct file * filp, int on)
{
	//elan_info("%s enter \n",__func__);
	return fasync_helper(fd, filp, on, &fasync_queue);
}

static const struct file_operations efsa120s_fops = {
	.owner 			= THIS_MODULE,
	.open 			= efsa120s_open,
	.unlocked_ioctl = efsa120s_ioctl,
	.poll			= efsa120s_poll,
	.release 		= efsa120s_close,
	.fasync 		= elan_fp_fasync,
};



static irqreturn_t efsa120s_irq_handler(int irq, void *_fp)
{
	struct efsa120s_data *fp = _fp;

	ELAN_DEBUG("%s()\n", __func__);
	/* input power keyevent */
	wake_lock_timeout(&fp->wake_lock,msecs_to_jiffies(3000));
	complete(&cmd_done_irq);
	if (fasync_queue){
		kill_fasync(&fasync_queue, SIGIO, POLL_IN);
	}
	return IRQ_HANDLED;
}

static int efsa120s_setup_cdev(struct efsa120s_data *fp)
{
	
	fp->efsa120_dev.minor = MISC_DYNAMIC_MINOR;
	fp->efsa120_dev.name = "elan_fp";
	fp->efsa120_dev.fops = &efsa120s_fops;
	fp->efsa120_dev.mode = S_IFREG|S_IRWXUGO; 
	if (misc_register(&fp->efsa120_dev) < 0) {
  		ELAN_DEBUG("misc_register failed!!");
		return -1;		
	}
  	else {
		ELAN_DEBUG("misc_register finished!!");		
	}
	return 0;
}

static int efsa120s_sysfs_create(struct efsa120s_data *sysfs)
{
	//struct efsa120s_data *fp = spi_get_drvdata(sysfs->pdev);
	struct efsa120s_data *fp = platform_get_drvdata(sysfs->pdev);
	int error = 0;
	
	/* Register sysfs */
	error = sysfs_create_group(&fp->pdev->dev.kobj, &efsa120s_attr_group);
	if (error) {
		dev_err(&fp->pdev->dev, "[ELAN] Failed to create sysfs attributes, err: %d\n", error);
		goto fail_un;
	}
	return 0;
fail_un:
	/* Remove sysfs */
	sysfs_remove_group(&fp->pdev->dev.kobj, &efsa120s_attr_group);
		
	return error;
}

static void elan_spi_clk_enable(struct efsa120s_data *fp, u8 bonoff)
{
#ifdef CONFIG_MTK_CLKMGR
	if (bonoff)
		enable_clock(MT_CG_PERI_SPI0, "spi");
	else
		disable_clock(MT_CG_PERI_SPI0, "spi");

#else
	/* changed after MT6797 platform */
	struct mt_spi_t *ms = NULL;
	ms = spi_master_get_devdata(fp->pdev->master);

	if (bonoff) {
		mt_spi_enable_clk(ms);
	} else {
		mt_spi_disable_clk(ms);
	}
#endif
}

static void efsa120s_gpio_as_int(struct efsa120s_data *fp)
{
	printk("[efsa120s]efsa120s_gpio_as_int\n");
	pinctrl_select_state(fp->pinctrl1, fp->eint_as_int);
}


static char efsa120s_gpio_config(struct efsa120s_data *fp)
{	
	int ret = -1;
	struct device_node *node;
	printk("[elan]:%s enter\n", __func__);
	node = of_find_compatible_node(NULL, NULL, "mediatek,fingerprint");
	if ( node)
	{
		efsa120s_gpio_as_int(fp);		
		fp->isr = irq_of_parse_and_map( node, 0);
		printk("ELAN efsa120s->irq = %d\n",  fp->isr);
		if (! fp->isr)
		{
			printk("ELAN irq_of_parse_and_map fail!!\n");
			return -1;
		}
	}
	else
	{
		printk("ELAN null irq node!!\n");
		return -1;
	}

	ret = request_irq(fp->isr, efsa120s_irq_handler, IRQF_TRIGGER_RISING | IRQF_ONESHOT, "elan_fp_irq", fp);
	if (ret) {
		ELAN_DEBUG("%s : =====EINT IRQ LINE NOT AVAILABLE  %d\n", __func__,ret);
	} else {
		ELAN_DEBUG("%s : =====set EINT finished, fp_irq=%d", __func__, fp->isr);
		//efsa120s_irq_disable(fp);
		//msleep(20);
		//efsa120s_irq_enable(fp);
		//pinctrl_select_state(fp->pinctrl1, fp->pins_miso_spi);	//enable SPI
		
		//elan_spi_clk_enable(fp, 1);
	}

	return 0;
}

#if 0
static int elan_parse_dt(struct device *dev, struct efsa120s_data *pdata)
{
	struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "mediatek,fingerprint");
	if (node) {
		int ret=0;
		printk("[fp] mt_fp_pinctrl+++++++++++++++++\n");

		pdata->fp_rst_high = pinctrl_lookup_state(pdata->pinctrl1, "fp_rst_high");
		if (IS_ERR(pdata->fp_rst_high)) {
			ret = PTR_ERR(pdata->fp_rst_high);
			dev_err(&pdata->pdev->dev, "fwq Cannot find fp pinctrl fp_rst_high!\n");
			return ret;
		}
		pdata->fp_rst_low = pinctrl_lookup_state(pdata->pinctrl1, "fp_rst_low");
		if (IS_ERR(pdata->fp_rst_low)) {
			ret = PTR_ERR(pdata->fp_rst_low);
			dev_err(&pdata->pdev->dev, "fwq Cannot find fp pinctrl fp_rst_low!\n");
			return ret;
		}
		pdata->eint_as_int = pinctrl_lookup_state(pdata->pinctrl1, "eint_as_int");
		if (IS_ERR(pdata->eint_as_int)) {
			ret = PTR_ERR(pdata->eint_as_int);
			dev_err(&pdata->pdev->dev, "fwq Cannot find fp pinctrl eint_output_high!\n");
			return ret;
		}
		pdata->pins_miso_spi = pinctrl_lookup_state(pdata->pinctrl1, "miso_spi");
		if (IS_ERR(pdata->pins_miso_spi)) {
			ret = PTR_ERR(pdata->pins_miso_spi);
			dev_err(&pdata->pdev->dev, "%s can't find fingerprint pinctrl miso_spi\n", __func__);
			return ret;
		}
		pdata->pins_miso_pullhigh = pinctrl_lookup_state(pdata->pinctrl1, "miso_pullhigh");
		if (IS_ERR(pdata->pins_miso_pullhigh)) {
			ret = PTR_ERR(pdata->pins_miso_pullhigh);
			dev_err(&pdata->pdev->dev, "%s can't find fingerprint pinctrl miso_pullhigh\n", __func__);
			return ret;
		}
		pdata->pins_miso_pulllow = pinctrl_lookup_state(pdata->pinctrl1, "miso_pulllow");
		if (IS_ERR(pdata->pins_miso_pulllow)) {
			ret = PTR_ERR(pdata->pins_miso_pulllow);
			dev_err(&pdata->pdev->dev, "%s can't find fingerprint pinctrl miso_pulllow\n", __func__);
			return ret;
		}
	}
	return 0;
}
#else

int elan_parse_dt(struct device *dev, struct efsa120s_data *pdata)
{
	struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "mediatek,fingerprint");
	if (node) {
		int ret;
		printk("[fp] mt_fp_pinctrl+++++++++++++++++\n");

		pdata->fp_rst_high = pinctrl_lookup_state(pdata->pinctrl1, "fp_rst_high");
		if (IS_ERR(pdata->fp_rst_high)) {
			ret = PTR_ERR(pdata->fp_rst_high);
			dev_err(&pdata->pdev->dev, "fwq Cannot find fp pinctrl fp_rst_high!\n");
			return ret;
		}
		pdata->fp_rst_low = pinctrl_lookup_state(pdata->pinctrl1, "fp_rst_low");
		if (IS_ERR(pdata->fp_rst_low)) {
			ret = PTR_ERR(pdata->fp_rst_low);
			dev_err(&pdata->pdev->dev, "fwq Cannot find fp pinctrl fp_rst_low!\n");
			return ret;
		}
		pdata->eint_as_int = pinctrl_lookup_state(pdata->pinctrl1, "eint_as_int");
		if (IS_ERR(pdata->eint_as_int)) {
			ret = PTR_ERR(pdata->eint_as_int);
			dev_err(&pdata->pdev->dev, "fwq Cannot find fp pinctrl eint_as_int!\n");
			return ret;
		}
		pdata->eint_in_low = pinctrl_lookup_state(pdata->pinctrl1, "eint_in_low");
		if (IS_ERR(pdata->eint_in_low)) {
			ret = PTR_ERR(pdata->eint_in_low);
			dev_err(&pdata->pdev->dev, "fwq Cannot find fp pinctrl eint_output_low!\n");
			return ret;
		}
		pdata->eint_in_float = pinctrl_lookup_state(pdata->pinctrl1, "eint_in_float");
		if (IS_ERR(pdata->eint_in_float)) {
			ret = PTR_ERR(pdata->eint_in_float);
			dev_err(&pdata->pdev->dev, "fwq Cannot find fp pinctrl eint_output_high!\n");
			return ret;
		}
		pdata->fp_enable_low = pinctrl_lookup_state(pdata->pinctrl1, "fp_enable_low");
        	if (IS_ERR(pdata->fp_enable_low)) {
        		ret = PTR_ERR(pdata->fp_enable_low);
        		dev_err(&pdata->pdev->dev, "Cannot find fp pinctrl fp_enable_low!\n");
        		return ret;
        	}
        	pdata->fp_enable_high = pinctrl_lookup_state(pdata->pinctrl1, "fp_enable_high");
        	if (IS_ERR(pdata->fp_enable_high)) {
        		ret = PTR_ERR(pdata->fp_enable_high);
        		dev_err(&pdata->pdev->dev, "Cannot find fp pinctrl fp_enable_high!\n");
        		return ret;
        	}

		return ret;
		printk("[FP] mt_fp_pinctrl----------\n");
	}else{
		
	}
	return 0;
}
#endif

void efsa120s_gpio_power(int onoff)
{
	int ret;
	printk("%s onoff = %d", __func__, onoff);
	if(onoff){
		ret = regulator_enable(elan_fp->reg);	/*enable regulator*/
		if (ret)
			printk("regulator_enable() failed!\n");
	}else{
		ret = regulator_disable(elan_fp->reg);	/*disable regulator*/
		if (ret)
			printk("regulator_disable() failed!\n");
	}
}

static int efsa120s_probe(struct platform_device *pdev)
//static int efsa120s_probe(struct spi_device *pdev)
{	
	struct efsa120s_data *fp = NULL;
	struct input_dev *input_dev = NULL;
	int err = 0;
	
	ELAN_DEBUG("=====%s() Start=====\n", __func__);
	//ELAN_DEBUG("%s GPIO_FP_ID : %d\n", VERSION_LOG, gpio_get_value(GPIO_FP_ID));

	init_completion(&cmd_done_irq);
#if 0
	err = spi_setup(pdev);
	if(err < 0)
		ELAN_DEBUG("spi_setup fail (0x%x).\n", err);
#endif	
	/* Allocate Device Data */
	#if 0
	fp = devm_kzalloc(&pdev->dev, sizeof(struct efsa120s_data), GFP_KERNEL);
	if(!fp)
		ELAN_DEBUG("alloc efsa120s data fail.\n");	
	#endif
	fp = kzalloc(sizeof(struct efsa120s_data), GFP_KERNEL);
	if(!fp)
		ELAN_DEBUG("alloc efsa120s data fail.\n");

	elan_fp = fp;
	/* Init Poll Wait */
	init_waitqueue_head(&fp->efsa_wait);

	/* Init Input Device */
	input_dev = input_allocate_device();
	if (!input_dev)
		ELAN_DEBUG("alloc input_dev fail.\n");

	fp->pdev = pdev;		
	//spi_set_drvdata(pdev, fp);
	platform_set_drvdata(pdev, fp);
	input_dev->name = "efsa120s";
	input_dev->id.bustype = BUS_SPI;
	input_dev->dev.parent = &pdev->dev;
	input_set_drvdata(input_dev, fp);	

	input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY);
	input_set_capability(input_dev, EV_KEY, KEY_FP_INT); // change by customer, send key event to framework. KEY_xxx could be changed.
	input_set_capability(input_dev, EV_KEY, KEY_FP_INT2); // change by customer, send key event to framework. KEY_xxx could be changed.

	fp->input_dev = input_dev;	
#if 1
	/* Init Sysfs */
	err = efsa120s_sysfs_create(fp);
	if(err < 0)
		ELAN_DEBUG("efsa120s sysfs fail.\n");
#endif

	/* Init Char Device */
	err = efsa120s_setup_cdev(fp);
	if(err < 0)
		ELAN_DEBUG("efsa120s setup device fail.\n");

	/* Register Input Device */
#if 0
	err = input_register_device(input_dev);
	if(err) {
		ELAN_DEBUG("Unable to register input device, error: %d!\n", err);
		goto fp_probe_fail;
	}
#endif

	/* Parse Device Tree */
	pdev->dev.of_node=of_find_compatible_node(NULL, NULL, "mediatek,fingerprint");
       //add by yinglong.tang begin
	fp->reg = regulator_get(&pdev->dev, "vfp");
	err = regulator_set_voltage(fp->reg, 2800000, 2800000);	/*set 2.8v*/
	if (err) {
		dev_err("regulator_set_voltage(%d) failed!\n", err);
		return -1;
	}
	//add by yinglong.tang end
	
	fp->pinctrl1 = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(fp->pinctrl1)) {
		err = PTR_ERR(fp->pinctrl1);
		dev_err(&pdev->dev, "fwq Cannot find fp pinctrl1!\n");
		return err;
	}

	err = elan_parse_dt(&pdev->dev, fp);	

	//add by yinglong.tang
	efsa120s_gpio_power(1);

	/* Init EFSA120S GPIO */
	err = efsa120s_gpio_config(fp);
	if(err < 0)
		ELAN_DEBUG("GPIO request fail (%d).\n", err);

	wake_lock_init(&fp->wake_lock, WAKE_LOCK_SUSPEND, "fp_wake_lock");

	/* Init IRQ FUNC */
	/*err = request_threaded_irq(fp->isr, NULL, efsa120s_irq_handler,
			IRQF_NO_SUSPEND | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			pdev->dev.driver->name, fp);*/
	/*err = request_irq(fp->isr, efsa120s_irq_handler,
			IRQF_NO_SUSPEND | IRQF_TRIGGER_RISING, 
			pdev->dev.driver->name, fp);
	if(err)
		ELAN_DEBUG("Failed to request IRQ %d.\n", err);*/

	//irq_set_irq_wake(fp->isr, 1);

	spin_lock_init(&fp->irq_lock);

        full_fp_chip_name("tee_fp");
	ELAN_DEBUG("=====%s() End=====\n", __func__);
	return 0;

fp_probe_fail:
	platform_set_drvdata(pdev, NULL);
	//spi_set_drvdata(pdev, NULL);
	input_free_device(input_dev);
	input_dev = NULL;
	kfree(fp);
	return -ENOMEM;
}

static int efsa120s_remove(struct spi_device *pdev)
{
	struct efsa120s_data *fp = spi_get_drvdata(pdev);
	
	if (fp->isr)
		free_irq(fp->isr, fp);

	gpio_free(fp->irq_gpio);
	gpio_free(fp->rst_gpio);
	
	misc_deregister(&fp->efsa120_dev);
	input_free_device(fp->input_dev);
		
	kfree(fp);

	platform_set_drvdata(pdev, NULL);
	//spi_set_drvdata(pdev, NULL);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int efsa120s_suspend(struct device *dev)
{
	ELAN_DEBUG("efsa120s suspend!\n");
	return 0;
}

static int efsa120s_resume(struct device *dev)
{
	ELAN_DEBUG("efsa120s resume!\n");
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(efsa120s_pm_ops, efsa120s_suspend, efsa120s_resume);

#ifdef CONFIG_OF
static const struct of_device_id efsa120s_of_match[] = {
	{ .compatible = "mediatek,fingerprint", },
	{ .compatible = "mediatek,elan-fp", },
	{ .compatible = "elan,elan-fp", },
	{},
};
MODULE_DEVICE_TABLE(of, efsa120s_of_match);
#endif


static const struct platform_device_id efp_id[] = {
	{"elan_fp", 0},
	{}
};
MODULE_DEVICE_TABLE(spi, efp_id);

static struct platform_driver efsa120s_driver = {//spi_driver
	.driver = {
		.name 	= "elan_fp",
		.owner = THIS_MODULE,
		.of_match_table = efsa120s_of_match,
	},
	.probe 	= efsa120s_probe,
	.remove = efsa120s_remove,
	.id_table = efp_id,
};

#if 0
static struct mt_chip_conf spi_xxxx_conf = {  
	//SPI speed
	.setuptime = 3,
	.holdtime = 3,
	.high_time = 10,
	.low_time = 10,
	.cs_idletime = 2,
	.ulthgh_thrsh = 0,
	
	//SPI mode
	.cpol = 0,
	.cpha = 0,
	
	.rx_mlsb = 1,
	.tx_mlsb = 1,
	
	.tx_endian = 0,
	.rx_endian = 0,
	
	.com_mod = 1,
	.pause = 0,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};

static struct spi_board_info efsa120s_spi_board_info[] = {
      
	[0] = {
		.modalias               = "elan_fp",
		.bus_num                = 0, // change by customer
		.chip_select            = 0, // change by customer, usually = 0.
		.max_speed_hz           = SPI_MAX_SPEED,
		.mode			= SPI_MODE_0,
		.controller_data = (void*)&spi_xxxx_conf,
	},
};
#endif

static int __init efsa120s_init(void)
{
	int status = 0;
	ELAN_DEBUG("=====%s() Start=====\n", __func__);

	status = platform_driver_register(&efsa120s_driver);
	//spi_register_board_info(efsa120s_spi_board_info, ARRAY_SIZE(efsa120s_spi_board_info));
	//status = spi_register_driver(&efsa120s_driver);
	if(status < 0)
		ELAN_DEBUG("%s FAIL !\n", __func__);

	ELAN_DEBUG("=====%s() End=====\n", __func__);

	return status;
}

static void __exit efsa120s_exist(void)
{	
	//spi_unregister_driver(&efsa120s_driver);
	platform_driver_unregister(&efsa120s_driver);
}

module_init(efsa120s_init);
module_exit(efsa120s_exist);

MODULE_AUTHOR("KennyKang <kenny.kang@emc.com.tw>");
MODULE_DESCRIPTION("ELAN SPI FingerPrint eFSA120S driver");
MODULE_VERSION(VERSION_LOG);
MODULE_LICENSE("GPL");

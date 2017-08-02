 /*Simple synchronous userspace interface to SPI devices
  *
  * Copyright (C) 2006 SWAPP
  *     Andrea Paterniani <a.paterniani@swapp-eng.it>
  * Copyright (C) 2007 David Brownell (simplification, cleanup)
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation; either version 2 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program; if not, write to the Free Software
  * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
  */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/clk.h> 
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/ktime.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/timer.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/pm_qos.h>
#include <linux/cpufreq.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include "gf_spi.h"
#include "../fp_drv/fp_drv.h"
#include <linux/poll.h> //xiaowen

#define GF_SPIDEV_NAME     "goodix,fingerprint"
/*device name after register in charater*/
#define GF_DEV_NAME            "goodix_fp"
#define	GF_INPUT_NAME	    "qwerty"	/*"goodix_fp" */
#define SPI_DEV_NAME        "spidev"

#define	CHRD_DRIVER_NAME	"goodix_fp_spi"
#define	CLASS_NAME		    "goodix_fp"
#define SPIDEV_MAJOR		225	/* assigned */
#define N_SPI_MINORS		32	/* ... up to 256 */

/*GF input keys*/
#define	GF_KEY_POWER	KEY_POWER
#define	GF_KEY_HOME		KEY_HOME
#define	GF_KEY_MENU		KEY_MENU
#define	GF_KEY_BACK		KEY_BACK
#define GF_UP_KEY		KEY_UP
#define GF_DOWN_KEY	    KEY_DOWN
#define GF_LEFT_KEY	    KEY_LEFT
#define GF_RIGHT_KEY	KEY_RIGHT
#define	GF_KEY_FORCE    KEY_F9
#define GF_APP_SWITCH	KEY_F19

/**************************debug******************************/


/*Global variables*/
/*static MODE g_mode = GF_IMAGE_MODE;*/
static DECLARE_BITMAP(minors, N_SPI_MINORS);
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
static DECLARE_WAIT_QUEUE_HEAD(gf_poll_wq);

#define GF_DATA_ADDR        (0x5202)

//value for GF_STATUS(0x0837) register
#define GF_INT_ERR_FW       (0xE0)
#define GF_INT_ERR_CFG      (0xE1)
#define GF_INT_ERR_ESD      (0xE3)
#define GF_INT_KEY          (0xB0)
#define GF_INT_KEY_LEFT     (0xB1)
#define GF_INT_KEY_RIGHT    (0xB2)
#define GF_INT_KEY_LIGHT    (0xB5)
#define GF_INT_KEY_WEIGHT   (0xB6)
#define GF_INT_IMAGE        (0xC0)
#define GF_INT_GSC          (0xC1)
#define GF_INT_HBD          (0xA0)
#define GF_INT_INVALID      (0x00)


#define SENSOR_ROW      (60)
#define SENSOR_COL      (128)
#define GF_ROW_SIZE             (2 + SENSOR_COL*2 +2)
#define GF_ROW_DATA_SIZE        (SENSOR_COL*2)
#define GF_READ_BUFF_SIZE1		(8 * GF_ROW_SIZE)
#define GF_READ_BUFF_SIZE2		(7 * GF_ROW_SIZE)
#define GF_SPI_DATA_LEN         (SENSOR_ROW * GF_ROW_SIZE)   //valid data len

#define BUF_SIZE    (GF_SPI_DATA_LEN + 1024 )

//static unsigned char gBuffer[16*1024] = {0};
//static DEFINE_MUTEX(buf_lock);

//xiaowen, add start
static int ftm_gfx_irq_state = 0;
static int ftm_gfx_irq_key = 0;
//xiaowen, add end

static struct gf_dev gf;

static struct mt_chip_conf spi_conf_mt65xx = {
	.setuptime = 15,
	.holdtime = 15,
	.high_time = 21, 
	.low_time = 21,	
	.cs_idletime = 20,
	.ulthgh_thrsh = 0,
	.cpol = 0,
	.cpha = 0,
	.rx_mlsb = 1,
	.tx_mlsb = 1,
	.tx_endian = 0,
	.rx_endian = 0,
	.com_mod = FIFO_TRANSFER,
	.pause = 0,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};

/*
static const struct dev_pm_ops gx_pm = {
	.suspend = gf_suspend_test,
	.resume = gf_resume_test
};
*/
static struct of_device_id gx_match_table[] = {
	{ .compatible = "mediatek,fingerprint", },
	{},
};


static struct spi_driver gf_driver = {
	.driver = {
		   .name = GF_DEV_NAME,
		   .owner = THIS_MODULE,
		   .bus    = &spi_bus_type,
		   .of_match_table = gx_match_table,
		   },
	.probe = gf_probe,
	.remove = gf_remove,
	.suspend = gf_suspend,
	.resume = gf_resume,
};

static struct spi_board_info spi_board_devs[] __initdata = {
	[0] = {
	    .modalias = GF_DEV_NAME,
		.bus_num = 0,
		.chip_select=0,
		.mode = SPI_MODE_0,
		.controller_data = &spi_conf_mt65xx,
	},
};

static int gf_spi_transfer(struct gf_dev *gf_dev, unsigned long arg);
static ssize_t gf_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
int gf_spi_read_bytes(struct gf_dev *gf_dev,
				u16 addr, u32 data_len, u8 *rx_buf);



/*  IRQ operations */
static void gf_enable_irq(struct gf_dev *gf_dev)
{
    FUNC_ENTRY();

#if 0
		enable_irq(gf_dev->spi->irq);
		gf_dev->irq_enabled = 1;
	
#else
	if (gf_dev->irq_enabled) {
		gf_print("IRQ has been enabled.\n");
	} else {
		enable_irq(gf_dev->irq);
		gf_dev->irq_enabled = 1;
	}
#endif
    FUNC_EXIT();
}

static void gf_disable_irq(struct gf_dev *gf_dev)
{
    FUNC_ENTRY();
#if 0
		gf_dev->irq_enabled = 0;
		disable_irq(gf_dev->spi->irq);
#else
	if (gf_dev->irq_enabled) {
		gf_dev->irq_enabled = 0;
		disable_irq(gf_dev->irq);
	} else {
		gf_print("IRQ has been disabled.\n");
	}
#endif
    FUNC_EXIT();
}

/* spi clock reference */
static long spi_clk_max_rate(struct clk *clk, unsigned long rate)
{
	long lowest_available, nearest_low, step_size, cur;
	long step_direction = -1;
	long guess = rate;
	int max_steps = 10;

	cur = clk_round_rate(clk, rate);
	if (cur == rate)
		return rate;

	/* if we got here then: cur > rate */
	lowest_available = clk_round_rate(clk, 0);
	if (lowest_available > rate)
		return -EINVAL;

	step_size = (rate - lowest_available) >> 1;
	nearest_low = lowest_available;

	while (max_steps-- && step_size) {
		guess += step_size * step_direction;
		cur = clk_round_rate(clk, guess);

		if ((cur < rate) && (cur > nearest_low))
			nearest_low = cur;
		/*
		 * if we stepped too far, then start stepping in the other
		 * direction with half the step size
		 */
		if (((cur > rate) && (step_direction > 0))
		    || ((cur < rate) && (step_direction < 0))) {
			step_direction = -step_direction;
			step_size >>= 1;
		}
	}
	return nearest_low;
}

static void spi_clock_set(struct gf_dev *gf_dev, int speed)
{
	long rate;
	int rc;

	rate = spi_clk_max_rate(gf_dev->core_clk, speed);
	if (rate < 0) {
		pr_info("%s: no match found for requested clock frequency:%d",
			__func__, speed);
		return;
	}

	rc = clk_set_rate(gf_dev->core_clk, rate);
}



void gf_spi_setup(struct gf_dev *gf_dev, int max_speed_hz)
{
    pr_info("####### %s %d \n", __func__, __LINE__);
	
        /*  gf_dev->spi->mode = SPI_MODE_0; //CPOL=CPHA=0
        gf_dev->spi->max_speed_hz = max_speed_hz;
        //gf_dev->spi->irq = gf_irq_num(gf_dev);;
        gf_dev->spi->bits_per_word = 8;
        spi_setup(gf_dev->spi);
        pr_info("####### %s %d \n", __func__, __LINE__);*/
    
    gf_dev->spi->mode = SPI_MODE_0; //CPOL=CPHA=0
    gf_dev->spi->max_speed_hz = max_speed_hz; 
    gf_dev->spi->bits_per_word = 8;
    gf_dev->spi->controller_data  = (void*)&spi_conf_mt65xx;
    spi_setup(gf_dev->spi);
}


static void gf_spi_clk_open(struct gf_dev *gf_dev)
{
	/*open core clk*/

	if ((!gf_dev->core_clk) || (!gf_dev->iface_clk))
		return;

	clk_prepare(gf_dev->core_clk);

	/*open iface clk*/
	clk_prepare(gf_dev->iface_clk);
	gf_dev->clk_enabled = 1;

}

static void gf_spi_clk_close(struct gf_dev *gf_dev)
{
	if ((!gf_dev->core_clk) || (!gf_dev->iface_clk))
		return;

	/*close iface clk*/
	clk_unprepare(gf_dev->iface_clk);

	/*open core clk*/
	clk_unprepare(gf_dev->core_clk);
	gf_dev->clk_enabled = 0;
}

static SPI_SPEED trans_spi_speed(unsigned int speed)
{
	SPI_SPEED spi_speed = SPEED_1MHZ;

	if (speed >= 4000000)
	{
		spi_speed = SPEED_4MHZ;
	}

	printk("%s , speed_in=%l, speed_out=%l",__func__, speed, spi_speed);
	return spi_speed;
}

static long gf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct gf_dev *gf_dev = &gf;
	struct gf_key gf_key = { 0 };
	unsigned int speed = 0;
	int retval = 0;
    u32 tmp = 0;//xiaowen
	char buf[512]={0};

	FUNC_ENTRY();
	if (_IOC_TYPE(cmd) != GF_IOC_MAGIC)
		return -ENODEV;

	if (_IOC_DIR(cmd) & _IOC_READ)
		retval =
		    !access_ok(VERIFY_WRITE, (void __user *)arg,
			       _IOC_SIZE(cmd));
	if ((retval == 0) && (_IOC_DIR(cmd) & _IOC_WRITE))
		retval =
		    !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (retval)
		return -EFAULT;

	switch (cmd) {
	case GF_IOC_DISABLE_IRQ:
        gf_print("%s , GF_IOC_DISABLE_IRQ",__func__);
		gf_disable_irq(gf_dev);
		break;
	case GF_IOC_ENABLE_IRQ:        
        gf_print("%s , GF_IOC_ENABLE_IRQ",__func__);
		gf_enable_irq(gf_dev);
		break;
	case GF_IOC_SETSPEED:    
        gf_print("%s , GF_IOC_SETSPEED",__func__);
		retval = __get_user(speed, (u32 __user *) arg);
		if (retval == 0) {
			if (speed > 12 * 1000 * 1000) {
				pr_info("Set speed:%d is larger than 12Mbps.\n",speed);
			} else {
				pr_info("Set speed:%d bps.\n",speed);
				gf_spi_set_mode(gf_dev->spi, trans_spi_speed(speed), 0);
			}
		} else {
			pr_info("Failed to get speed from user. retval = %d\n",retval);
		}
		break;
	case GF_IOC_RESET:        
        	gf_print("%s , GF_IOC_RESET",__func__);
		gf_hw_reset(gf_dev, 70);
		break;
	case GF_IOC_COOLBOOT:
		gf_power_on(gf_dev);
		mdelay(5);
		gf_power_on(gf_dev);
		break;
	case GF_IOC_SENDKEY:
		 ftm_gfx_irq_state = 0;	
		if (copy_from_user
		    (&gf_key, (struct gf_key *)arg, sizeof(struct gf_key))) {
                pr_info("Failed to copy data from user space,line=%d.\n", __LINE__);
			retval = -EFAULT;
			break;
		}
		printk(KERN_ERR"%s: gf_key.key = %d, gf_key.value = %d.\n",__func__,gf_key.key, gf_key.value);  
		if(gf_key.value  == 1) {                 
			ftm_gfx_irq_state = 0x1; //down.              
		} else if(gf_key.value  == 0) {     
			ftm_gfx_irq_state = 0x2; //up.              
		}	
        printk(KERN_ERR"%s: ftm_gfx_irq_state = %d.\n",__func__, ftm_gfx_irq_state);
		//send_sig(SIGUSR1, current, 1);		
		ftm_gfx_irq_key = 1;			
		wake_up(&gf_poll_wq);		   
		/*
		input_report_key(gf_dev->input, gf_key.key, gf_key.value);
		input_sync(gf_dev->input);
		*/
		break;
	case GF_IOC_CLK_READY:
		gf_spi_clk_open(gf_dev);
		break;
	case GF_IOC_CLK_UNREADY:
		gf_spi_clk_close(gf_dev);
		break;
	case GF_IOC_PM_FBCABCK:
		__put_user(gf_dev->fb_black, (u8 __user *) arg);
		break;
    case GF_IOC_SPI_TRANSFER:
        gf_spi_transfer(gf_dev, arg);
        break;
    //xiaowen add start.
	case GF_IOC_SET_MODE:
	{
		//u16 status = 0;	
		//u16 mode = 0;
		//u16 key = 0;
		u16 next_mode = 0;
		
		printk(KERN_ERR"[FTM]:set mode\n");
        
		retval = __get_user(tmp, (u32 __user*)arg);
		next_mode = (u16)tmp;
		printk(KERN_ERR"[FTM]:next mode:[%d]\n", next_mode);
		
		if(next_mode >= GFX1XM_IMAGE_MODE && next_mode <= GFX1XM_FF_MODE){
			gf_spi_write_word(gf_dev, 0x0834, next_mode);
		}
		
		//gf_spi_read_word(gf_dev, 0x0836, &status);		
		//gf_spi_read_word(gf_dev, 0x083B, &key);		
	}
	break;	
	
    case GF_IOC_FTM:
    {
        int tmp1 = 0;
        printk(KERN_ERR"%s: ftm_gfx_irq_state = %d.\n",__func__,
			ftm_gfx_irq_state);
        retval = __get_user(tmp1, (u32 __user*)arg);
        tmp = ftm_gfx_irq_state;
        if(tmp1 == 0) {
            ftm_gfx_irq_state = 0;
        }
            retval = __put_user(tmp, (__u32 __user *)arg);
    }
    
break;
//xiaowen add end.
	
	case GF_IOC_RELEASEVERSION:
	{
	    gf_print("%s , GF_IOC_RELEASEVERSION",__func__);
	    if (copy_from_user(buf, (int *)arg, sizeof(buf))) {	
		gf_print("%s , FAIL TO COPY GF_IOC_RELEASEVERSION!",__func__);
		break;
	    }
	    full_fp_chip_info(buf);
	}
	break;

	default:
		gf_print("Unsupport cmd:0x%x\n", cmd);
		break;
	}

	//FUNC_EXIT();
	return retval;
}

#ifdef CONFIG_COMPAT
static long
gf_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return gf_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif /*CONFIG_COMPAT*/

static irqreturn_t gf_irq(int irq, void *handle)
{
	struct gf_dev *gf_dev = &gf;
    FUNC_ENTRY();

#if 0
    u8 data = 0;
	int i;
	char version[20]= {0};
	
   // gf_spi_read_byte(gf_dev, 0x0837, &data);
   // gf_print(" -----------interrupt event callback!!---------- %d\n", data);


    for(i = 0; i < 1; i++) {
		gf_spi_read_bytes(gf_dev,0x0800, 6, gf_dev->gBuffer);
		memcpy(version, gf_dev->gBuffer + GF_RDATA_OFFSET, 6);	
		gf_print("interrupt: chip===>version = %x,%x,%x,%x,%x,%x \n", 
			version[0],version[1],version[2],version[3],version[4],version[5]); 
    }
#endif
	
#ifdef GF_FASYNC
	if (gf_dev->async){
		kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
	}
#endif
    //FUNC_EXIT();

	return IRQ_HANDLED;
}




#if  0
static void gf_spi_complete(void *arg)
{
    complete(arg);
}
/**********************************************************
 *Message format:
 *  write cmd   |  ADDR_H |ADDR_L  |  data stream  |
 *    1B         |   1B    |  1B    |  length       |
 *
 * read buffer length should be 1 + 1 + 1 + data_length
 ***********************************************************/
static int gf_spi_write_bytes(struct gf_dev *gf_dev,
    u16 addr, u32 data_len, u8 *tx_buf)
{
    DECLARE_COMPLETION_ONSTACK(read_done);
    struct spi_message msg;
    struct spi_transfer *xfer;
    int ret = 0;

    xfer = kzalloc(sizeof(*xfer), GFP_KERNEL);
    if( xfer == NULL){
        gf_print("No memory for command.\n");
        return -ENOMEM;
    }

    /*send gf command to device.*/
    spi_message_init(&msg);
    tx_buf[0] = GF_W;
    tx_buf[1] = (u8)((addr >> 8)&0xFF);
    tx_buf[2] = (u8)(addr & 0xFF);
    xfer[0].tx_buf = tx_buf;
    xfer[0].len = data_len + 3;
    xfer[0].delay_usecs = 5;
    spi_message_add_tail(xfer, &msg);
    
    msg.complete = gf_spi_complete;
    msg.context = &read_done;

    spin_lock_irq(&gf_dev->spi_lock);
    ret = spi_async(gf_dev->spi, &msg);
    spin_unlock_irq(&gf_dev->spi_lock);
    if(ret == 0) {
        wait_for_completion(&read_done);
        if(msg.status == 0)
            ret = msg.actual_length - GF_WDATA_OFFSET;
    }
    //pr_info("ret = %d, actual_length = %d\n", ret, msg.actual_length);
    kfree(xfer);
    if(xfer != NULL)
        xfer = NULL;

    return ret;
}

/*************************************************************
 *First message:
 *  write cmd   |  ADDR_H |ADDR_L  |
 *    1B         |   1B    |  1B    |
 *Second message:
 *  read cmd   |  data stream  |
 *    1B        |   length    |
 *
 * read buffer length should be 1 + 1 + 1 + 1 + data_length
 **************************************************************/
static int gf_spi_read_bytes(struct gf_dev *gf_dev,
    u16 addr, u32 data_len, u8 *rx_buf)
{
    DECLARE_COMPLETION_ONSTACK(write_done);
    struct spi_message msg;
    struct spi_transfer *xfer;
    int ret = 0;

    xfer = kzalloc(sizeof(*xfer)*2, GFP_KERNEL);
    if( xfer == NULL){
        gf_print("No memory for command.\n");
        return -ENOMEM;
    }

    /*send gf command to device.*/
    spi_message_init(&msg);
    rx_buf[0] = GF_W;
    rx_buf[1] = (u8)((addr >> 8)&0xFF);
    rx_buf[2] = (u8)(addr & 0xFF);
    xfer[0].tx_buf = rx_buf;
    xfer[0].len = 3;
    xfer[0].delay_usecs = 5;
    spi_message_add_tail(&xfer[0], &msg);

    /*if wanted to read data from gf.
     *Should write Read command to device
     *before read any data from device.
     */
    //memset(rx_buf, 0xff, data_len);
    spi_sync(gf_dev->spi, &msg);
    spi_message_init(&msg);
    memset(rx_buf + GF_RDATA_OFFSET, 0xFF, data_len);
    rx_buf[GF_RDATA_OFFSET - 1] = GF_R;
    xfer[1].tx_buf = &rx_buf[GF_RDATA_OFFSET - 1];

    xfer[1].rx_buf = &rx_buf[GF_RDATA_OFFSET - 1];
    xfer[1].len = data_len + 1;
    xfer[1].delay_usecs = 5;

    spi_message_add_tail(&xfer[1], &msg);

    msg.complete = gf_spi_complete;
    msg.context = &write_done;

    spin_lock_irq(&gf_dev->spi_lock);
    ret = spi_async(gf_dev->spi, &msg);
    spin_unlock_irq(&gf_dev->spi_lock);
    if(ret == 0) {
        wait_for_completion(&write_done);
        if(msg.status == 0)
            ret = msg.actual_length - 1;//GF_RDATA_OFFSET;
    }
    //pr_info("ret = %d, actual_length = %d\n", ret, msg.actual_length);
    kfree(xfer);
    if(xfer != NULL)
        xfer = NULL;
    return ret;
}

#endif


static void gf_spi_set_mode(struct spi_device *spi, SPI_SPEED speed, int flag)
{
	struct mt_chip_conf *mcc = &spi_conf_mt65xx;
	if(flag == 0) {
		mcc->com_mod = FIFO_TRANSFER;
	} else {
		mcc->com_mod = DMA_TRANSFER;
	}
	switch(speed)
	{
		case SPEED_500KHZ:
			mcc->high_time = 120;
			mcc->low_time = 120;
			break;
		case SPEED_1MHZ:
			mcc->high_time = 60;
			mcc->low_time = 60;
			break;
		case SPEED_2MHZ:
			mcc->high_time = 30;
			mcc->low_time = 30;
			break;
		case SPEED_3MHZ:
			mcc->high_time = 20;
			mcc->low_time = 20;
			break;
		case SPEED_4MHZ:
			mcc->high_time = 15;
			mcc->low_time = 15;
			break;

		case SPEED_6MHZ:
			mcc->high_time = 10;
			mcc->low_time = 10;
			break;
		case SPEED_8MHZ:
		    mcc->high_time = 8;
			mcc->low_time = 8;
			break;  
		case SPEED_KEEP:
		case SPEED_UNSUPPORTED:
			break;
	}
	if(spi_setup(spi) < 0){
		//gf_error("gf:Failed to set spi.");
	}
}


/**********************************************************
 *Message format:
 *	write cmd   |  ADDR_H |ADDR_L  |  data stream  |
 *    1B         |   1B    |  1B    |  length       |
 *
 * read buffer length should be 1 + 1 + 1 + data_length
 ***********************************************************/
int gf_spi_write_bytes(struct gf_dev *gf_dev,
				u16 addr, u32 data_len, u8 *tx_buf)
{
	struct spi_message msg;
	struct spi_transfer *xfer;
	u32 package_num = (data_len + 2*GF_WDATA_OFFSET)>>MTK_SPI_ALIGN_MASK_NUM;
	u32 reminder = (data_len + 2*GF_WDATA_OFFSET) & MTK_SPI_ALIGN_MASK;
	u8 *reminder_buf = NULL;
	u8 twice = 0;
	int ret = 0;

	FUNC_ENTRY();

	/*set spi mode.*/
	if((data_len + GF_WDATA_OFFSET) > 32) {
		gf_spi_set_mode(gf_dev->spi, SPEED_KEEP, 1); //FIFO
	} else {
		gf_spi_set_mode(gf_dev->spi, SPEED_KEEP, 0); //FIFO
	}
	if((package_num > 0) && (reminder != 0)) {
		twice = 1;
		/*copy the reminder data to temporarity buffer.*/
		reminder_buf = kzalloc(reminder + GF_WDATA_OFFSET, GFP_KERNEL);
		if(reminder_buf == NULL ) {
			gf_print("gf:No memory for exter data.");
			return -ENOMEM;
		}
		memcpy(reminder_buf + GF_WDATA_OFFSET, tx_buf + 2*GF_WDATA_OFFSET+data_len - reminder, reminder);
        gf_print("gf:w-reminder:0x%x-0x%x,0x%x", reminder_buf[GF_WDATA_OFFSET],reminder_buf[GF_WDATA_OFFSET+1],
                reminder_buf[GF_WDATA_OFFSET + 2]);
		xfer = kzalloc(sizeof(*xfer)*2, GFP_KERNEL);
	} else {
		twice = 0;
		xfer = kzalloc(sizeof(*xfer), GFP_KERNEL);
	}
	if(xfer == NULL){
		gf_print("gf:No memory for command.");
		if(reminder_buf != NULL)
			kfree(reminder_buf);
		return -ENOMEM;
	}

	gf_print("gf:write twice = %d. data_len = %d, package_num = %d, reminder = %d\n", (int)twice, (int)data_len, (int)package_num, (int)reminder);
	/*if the length is not align with 1024. Need 2 transfer at least.*/
	spi_message_init(&msg);
	tx_buf[0] = GF_W;
	tx_buf[1] = (u8)((addr >> 8)&0xFF);
	tx_buf[2] = (u8)(addr & 0xFF);
	xfer[0].tx_buf = tx_buf;
	//xfer[0].delay_usecs = 5;
	if(twice == 1) {
		xfer[0].len = package_num << MTK_SPI_ALIGN_MASK_NUM;
		spi_message_add_tail(&xfer[0], &msg);
		addr += (data_len - reminder + GF_WDATA_OFFSET);
		reminder_buf[0] = GF_W;
		reminder_buf[1] = (u8)((addr >> 8)&0xFF);
		reminder_buf[2] = (u8)(addr & 0xFF);
		xfer[1].tx_buf = reminder_buf;
		xfer[1].len = reminder + 2*GF_WDATA_OFFSET;
		//xfer[1].delay_usecs = 5;
		spi_message_add_tail(&xfer[1], &msg);
	} else {
		xfer[0].len = data_len + GF_WDATA_OFFSET;
		spi_message_add_tail(&xfer[0], &msg);
	}
	ret = spi_sync(gf_dev->spi, &msg);
	if(ret == 0) {
		if(twice == 1)
			ret = msg.actual_length - 2*GF_WDATA_OFFSET;
		else
			ret = msg.actual_length - GF_WDATA_OFFSET;
	} else 	{
		gf_print("gf:write async failed. ret = %d", ret);
	}

	if(xfer != NULL) {
		kfree(xfer);
		xfer = NULL;
	}
	if(reminder_buf != NULL) {
		kfree(reminder_buf);
		reminder_buf = NULL;
	}
	
	return ret;
}

/*************************************************************
 *First message:
 *	write cmd   |  ADDR_H |ADDR_L  |
 *    1B         |   1B    |  1B    |
 *Second message:
 *	read cmd   |  data stream  |
 *    1B        |   length    |
 *
 * read buffer length should be 1 + 1 + 1 + 1 + data_length
 **************************************************************/
int gf_spi_read_bytes(struct gf_dev *gf_dev,
				u16 addr, u32 data_len, u8 *rx_buf)
{
	struct spi_message msg;
	struct spi_transfer *xfer;
	u32 package_num = (data_len + 1 + 1)>>MTK_SPI_ALIGN_MASK_NUM;
	u32 reminder = (data_len + 1 + 1) & MTK_SPI_ALIGN_MASK;
	u8 *one_more_buff = NULL;
	u8 twice = 0;
	int ret = 0;	

	FUNC_ENTRY();
	
	one_more_buff = kzalloc(15000, GFP_KERNEL);
	if(one_more_buff == NULL ) {
		gf_print("No memory for one_more_buff.");
		return -ENOMEM;
	}
	xfer = kzalloc(sizeof(*xfer)*2, GFP_KERNEL);
	if((package_num > 0) && (reminder != 0)) {
		twice = 1;
		printk("stone package_num is %d reminder is %d\n",package_num,reminder);
	} else {
		twice = 0;
	}
	if( xfer == NULL){
		gf_print("No memory for command.");
		if(one_more_buff != NULL)
			kfree(one_more_buff);
		return -ENOMEM;
	}
	/*set spi mode.*/
	if((data_len + GF_RDATA_OFFSET) > 32) {
		gf_spi_set_mode(gf_dev->spi, SPEED_KEEP, 1); //DMA
	} else {
		gf_spi_set_mode(gf_dev->spi, SPEED_KEEP, 0); //FIFO
	}
	spi_message_init(&msg);
    /*send GF command to device.*/
	rx_buf[0] = GF_W;
	rx_buf[1] = (u8)((addr >> 8)&0xFF);
	rx_buf[2] = (u8)(addr & 0xFF);
	xfer[0].tx_buf = rx_buf;
	xfer[0].len = 3;
	spi_message_add_tail(&xfer[0], &msg);
	spi_sync(gf_dev->spi, &msg);
	spi_message_init(&msg);

	/*if wanted to read data from GF. 
	 *Should write Read command to device
	 *before read any data from device.
	 */
	//memset(rx_buf, 0xff, data_len);
	one_more_buff[0] = GF_R;
	xfer[1].tx_buf = &one_more_buff[0];
	xfer[1].rx_buf = &one_more_buff[0];	
	//read 1 additional package to ensure no even data read
	if(twice == 1)
		xfer[1].len = ((package_num+1) << MTK_SPI_ALIGN_MASK_NUM);
	else
		xfer[1].len = data_len + 1;
	spi_message_add_tail(&xfer[1], &msg);
	ret = spi_sync(gf_dev->spi, &msg);
	if(ret == 0) {
		memcpy(rx_buf + GF_RDATA_OFFSET,one_more_buff+1,data_len);
		ret = data_len;
	}else {
        gf_print("gf: read failed. ret = %d", ret);
  }

	kfree(xfer);
	if(xfer != NULL)
		xfer = NULL;
	if(one_more_buff != NULL) {
		kfree(one_more_buff);
		one_more_buff = NULL;
	}	
	//gf_debug(SPI_DEBUG,"gf:read twice = %d, data_len = %d, package_num = %d, reminder = %d\n",(int)twice, (int)data_len, (int)package_num, (int)reminder);
	gf_print("gf:data_len = %d, msg.actual_length = %d, ret = %d\n", (int)data_len, (int)msg.actual_length, ret);
	return ret;
}

static int gf_spi_read_byte(struct gf_dev *gf_dev, u16 addr, u8 *value)
{
    int status = 0;
    mutex_lock(&gf_dev->buf_lock);

    status = gf_spi_read_bytes(gf_dev, addr, 1, gf_dev->gBuffer);
    *value = gf_dev->gBuffer[GF_WDATA_OFFSET];
    mutex_unlock(&gf_dev->buf_lock);
    return status;
}
static int gf_spi_write_byte(struct gf_dev *gf_dev, u16 addr, u8 value)
{
    int status = 0;
    mutex_lock(&gf_dev->buf_lock);
    gf_dev->gBuffer[GF_WDATA_OFFSET] = value;
    status = gf_spi_write_bytes(gf_dev, addr, 1, gf_dev->gBuffer);
    mutex_unlock(&gf_dev->buf_lock);
    return status;
}

int gf_spi_read_word(struct gf_dev *gf_dev, u16 addr, u16 *value)
{
	int status = 0;
	u8 *buf = NULL;
	mutex_lock(&gf_dev->buf_lock);
	status = gf_spi_read_bytes(gf_dev, addr, 2, gf_dev->gBuffer);
	buf = gf_dev->gBuffer + GF_RDATA_OFFSET;
	*value = (u16)buf[0]<<8 | buf[1];
	mutex_unlock(&gf_dev->buf_lock);
	return status;
}

int gf_spi_write_word(struct gf_dev* gf_dev, u16 addr, u16 value)
{
	int status = 0;
    mutex_lock(&gf_dev->buf_lock);
    gf_dev->gBuffer[GF_WDATA_OFFSET] = 0x00;
    gf_dev->gBuffer[GF_WDATA_OFFSET+1] = 0x01;
    gf_dev->gBuffer[GF_WDATA_OFFSET+2] = (u8)(value>>8);
    gf_dev->gBuffer[GF_WDATA_OFFSET+3] = (u8)(value & 0x00ff);
    status = gf_spi_write_bytes(gf_dev, addr, 4, gf_dev->gBuffer);
    mutex_unlock(&gf_dev->buf_lock);

    return status;
}

/*
int gf_spi_write_word(struct gf_dev *gf_dev, unsigned short addr, unsigned short write_value)
{
    gf_dev->gBuffer[GF_WDATA_OFFSET + 0] =  0;
    gf_dev->gBuffer[GF_WDATA_OFFSET + 1] =  1;
    gf_dev->gBuffer[GF_WDATA_OFFSET + 2] =  (unsigned char) (write_value >> 8);
    gf_dev->gBuffer[GF_WDATA_OFFSET + 3] =  (unsigned char) (write_value & 0xff);
    
    return gf_spi_write_bytes(gf_dev, addr, 4, gf_dev->gBuffer);
}

int gf_spi_read_word(struct gf_dev *gf_dev, unsigned short addr, unsigned short *read_value)
{
    int ret = 0;
    
    ret = gf_spi_read_bytes(gf_dev, addr, 2, gf_dev->gBuffer);
    *read_value = gf_dev->gBuffer[GF_RDATA_OFFSET + 0] << 8;
    *read_value += gf_dev->gBuffer[GF_RDATA_OFFSET + 1];
    
    return ret;
}*/

static int gf_spi_transfer(struct gf_dev *gf_dev, unsigned long arg)
{
    struct gf_spi_transfer ioc = {0};
    FUNC_ENTRY();
    /*copy command data from user to kernel.*/
    if(copy_from_user(&ioc, (struct gf_spi_transfer*)arg, sizeof(struct gf_spi_transfer))){
        gf_print("Failed to copy command from user to kernel.line=%d\n", __LINE__);
        return -EFAULT;
    }
    if(ioc.len == 0) {
        gf_print("The request length is 0.\n");
        return -EMSGSIZE;
    }

    mutex_lock(&gf_dev->buf_lock);
    if(ioc.cmd == GF_R) {
        /*if want to read data from hardware.*/
        //pr_info("Read data from 0x%x, len = 0x%x buf = 0x%llx\n", ioc.addr, ioc.len, ioc.buf);
        gf_spi_read_bytes(gf_dev, ioc.addr, ioc.len, gf_dev->gBuffer);
        if(copy_to_user((void __user*) ioc.buf, (void *)(gf_dev->gBuffer + GF_RDATA_OFFSET), ioc.len)) {
            gf_print("Failed to copy data from kernel to user.line=%d\n",__LINE__);
            mutex_unlock(&gf_dev->buf_lock);
            return -EFAULT;
        }
    } else if (ioc.cmd == GF_W) {
        /*if want to read data from hardware.*/
        //pr_info("Write data from 0x%x, len = 0x%x\n", ioc.addr, ioc.len);

        if(copy_from_user(gf_dev->gBuffer + GF_WDATA_OFFSET, (void *)ioc.buf, ioc.len)){
            gf_print("Failed to copy data from user to kernel.line=%d\n", __LINE__);
            mutex_unlock(&gf_dev->buf_lock);
            return -EFAULT;
        }

        gf_spi_write_bytes(gf_dev, ioc.addr, ioc.len, gf_dev->gBuffer);
    } else {
        gf_print("Error command for gf.\n");
    }
    mutex_unlock(&gf_dev->buf_lock);
    FUNC_EXIT();
    return 0;
}


static int gf_open(struct inode *inode, struct file *filp)
{
	struct gf_dev *gf_dev;
	int status = -ENXIO;

	FUNC_ENTRY();
	mutex_lock(&device_list_lock);

	list_for_each_entry(gf_dev, &device_list, device_entry) {
		if (gf_dev->devt == inode->i_rdev) {
			gf_print("Found\n");
			status = 0;
			break;
		}
	}

	if (status == 0) {
		if (status == 0) {
			gf_dev->users++;
			filp->private_data = gf_dev;
			nonseekable_open(inode, filp);
			gf_print("Succeed to open device. irq = %d, users:%d\n",
			       gf_dev->spi->irq, gf_dev->users);
            if (gf_dev->users == 1){
                //gf_enable_irq(gf_dev);
            }
		}
	} else {
		gf_print("No device for minor %d\n", iminor(inode));
	}
	//gf_power_on(gf_dev);
	mutex_unlock(&device_list_lock);
	FUNC_EXIT();
	return status;
}

#ifdef GF_FASYNC
static int gf_fasync(int fd, struct file *filp, int mode)
{
	struct gf_dev *gf_dev = filp->private_data;
	int ret;

	FUNC_ENTRY();
	ret = fasync_helper(fd, filp, mode, &gf_dev->async);
	FUNC_EXIT();
	gf_print("ret = %d\n", ret);
	return ret;
}
#endif
//xiaowen, add start
static unsigned int gf_fp_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;
	
	poll_wait(file, &gf_poll_wq, wait);
	
	if(ftm_gfx_irq_key != 0){
		mask = POLLIN | POLLRDNORM;
	}
	
	ftm_gfx_irq_key = 0;
	return mask;
}
//xiaowen, add end

static int gf_release(struct inode *inode, struct file *filp)
{
	struct gf_dev *gf_dev;
	int status = 0;

	FUNC_ENTRY();
	mutex_lock(&device_list_lock);
	gf_dev = filp->private_data;
	filp->private_data = NULL;

	/*last close?? */
	gf_dev->users--;
	if (!gf_dev->users) {

		gf_print("disble_irq. irq = %d\n", gf_dev->spi->irq);
		gf_disable_irq(gf_dev);
	}
	mutex_unlock(&device_list_lock);
	FUNC_EXIT();
	return status;
}

static const struct file_operations gf_fops = {
	.owner = THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.unlocked_ioctl = gf_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = gf_compat_ioctl,
#endif /*CONFIG_COMPAT*/
	.open = gf_open,
	.release = gf_release,
	.read = gf_read,
#ifdef GF_FASYNC
	.fasync = gf_fasync,
#endif
       .poll = gf_fp_poll,
};

/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

#ifdef USE_SPI_BUS
static int gfspi_ioctl_clk_init(struct spi_device *spi, struct gf_dev *data)
{
	pr_debug("%s: enter\n", __func__);

	data->clk_enabled = 0;
	data->core_clk = clk_get(&spi->dev, "core_clk");
	if (IS_ERR_OR_NULL(data->core_clk)) {
		pr_err("%s: fail to get core_clk\n", __func__);
		return -1;
	}
	data->iface_clk = clk_get(&spi->dev, "iface_clk");
	if (IS_ERR_OR_NULL(data->iface_clk)) {
		pr_err("%s: fail to get iface_clk\n", __func__);
		clk_put(data->core_clk);
		data->core_clk = NULL;
		return -2;
	}
	return 0;
}

static int gfspi_ioctl_clk_enable(struct gf_dev *data)
{
	int err;

	pr_debug("%s: enter\n", __func__);

	if (data->clk_enabled)
		return 0;

	err = clk_prepare_enable(data->core_clk);
	if (err) {
		pr_err("%s: fail to enable core_clk\n", __func__);
		return -1;
	}

	err = clk_prepare_enable(data->iface_clk);
	if (err) {
		pr_err("%s: fail to enable iface_clk\n", __func__);
		clk_disable_unprepare(data->core_clk);
		return -2;
	}

	data->clk_enabled = 1;

	return 0;
}

static int gfspi_ioctl_clk_disable(struct gf_dev *data)
{
	pr_debug("%s: enter\n", __func__);

	if (!data->clk_enabled)
		return 0;

	clk_disable_unprepare(data->core_clk);
	clk_disable_unprepare(data->iface_clk);
	data->clk_enabled = 0;

	return 0;
}

static int gfspi_ioctl_clk_uninit(struct gf_dev *data)
{
	pr_debug("%s: enter\n", __func__);

	if (data->clk_enabled)
		gfspi_ioctl_clk_disable(data);

	if (!IS_ERR_OR_NULL(data->core_clk)) {
		clk_put(data->core_clk);
		data->core_clk = NULL;
	}

	if (!IS_ERR_OR_NULL(data->iface_clk)) {
		clk_put(data->iface_clk);
		data->iface_clk = NULL;
	}

	return 0;
}
#endif

static int goodix_fb_state_chg_callback(struct notifier_block *nb,
					unsigned long val, void *data)
{
	struct gf_dev *gf_dev;
	struct fb_event *evdata = data;
	unsigned int blank;

	if (val != FB_EARLY_EVENT_BLANK)
		return 0;
	pr_info("[info] %s go to the goodix_fb_state_chg_callback value = %d\n",
		__func__, (int)val);
	gf_dev = container_of(nb, struct gf_dev, notifier);
	if (evdata && evdata->data && val == FB_EARLY_EVENT_BLANK && gf_dev) {
		blank = *(int *)(evdata->data);
		switch (blank) {
		case FB_BLANK_POWERDOWN:
			if (gf_dev->device_available == 1) {
				gf_dev->fb_black = 1;
#ifdef GF_FASYNC
				if (gf_dev->async) {
					kill_fasync(&gf_dev->async, SIGIO,
						    POLL_IN);
				}
#endif
				/*device unavailable */
				gf_dev->device_available = 0;
			}
			break;
		case FB_BLANK_UNBLANK:
			if (gf_dev->device_available == 0) {
				gf_dev->fb_black = 0;
				/*device available */
				gf_dev->device_available = 1;
			}
			break;
		default:
			pr_info("%s defalut\n", __func__);
			break;
		}
	}
	return NOTIFY_OK;
}

static struct notifier_block goodix_noti_block = {
	.notifier_call = goodix_fb_state_chg_callback,
};

static void gf_reg_key_kernel(struct gf_dev *gf_dev)
{
	__set_bit(EV_KEY, gf_dev->input->evbit);
	__set_bit(GF_KEY_POWER, gf_dev->input->keybit);
	__set_bit(GF_KEY_HOME, gf_dev->input->keybit);
	__set_bit(GF_KEY_MENU, gf_dev->input->keybit);
	__set_bit(GF_KEY_BACK, gf_dev->input->keybit);
	__set_bit(GF_UP_KEY, gf_dev->input->keybit);
	__set_bit(GF_RIGHT_KEY, gf_dev->input->keybit);
	__set_bit(GF_LEFT_KEY, gf_dev->input->keybit);
	__set_bit(GF_DOWN_KEY, gf_dev->input->keybit);
	__set_bit(GF_KEY_FORCE, gf_dev->input->keybit);
	__set_bit(GF_APP_SWITCH, gf_dev->input->keybit);
	gf_dev->input->name = GF_INPUT_NAME;
	if (input_register_device(gf_dev->input))
		gf_print("Failed to register GF as input device.\n");
}

int gf_read_chip_id(struct gf_dev *pdev) 
{
    int ret = 0;
    unsigned short val = 0;
    char version[20] = {0};
	FUNC_ENTRY();
    printk(KERN_ERR"1goodix-chip id: 0x%x \n", val);

    gf_spi_read_word(pdev, 0x0142, &val);
    gf_print("goodix-chip id: 0x%x \n", val);
     
    printk(KERN_ERR"2goodix-chip id: 0x%x \n", val);
    if(val == 0x12a1) {
        return 0;
    }
        printk(KERN_ERR"3goodix-chip id: 0x%x \n", val);
    return -1;
/*
    for(ret = 0; ret < 3; ret++) {
		gf_spi_read_bytes(pdev,0x0800, 6, pdev->gBuffer);
		memcpy(version, pdev->gBuffer + GF_RDATA_OFFSET, 6);	
		gf_print(" chip===>version = %x,%x,%x,%x,%x,%x \n", 
			version[0],version[1],version[2],version[3],version[4],version[5]); 
    }
*/
}

static struct class *gf_class;
static int gf_probe(struct spi_device *spi)
{
	struct gf_dev *gf_dev = &gf;
	int status = -EINVAL;
	unsigned long minor;
	int ret;
	
	FUNC_ENTRY();

	INIT_LIST_HEAD(&gf_dev->device_entry);
	gf_dev->spi = spi;
	gf_dev->irq_gpio = -EINVAL;
	gf_dev->reset_gpio = -EINVAL;
	gf_dev->pwr_gpio = -EINVAL;
	gf_dev->device_available = 0;
	gf_dev->fb_black = 0;

    mutex_init(&gf_dev->buf_lock);
    spin_lock_init(&gf_dev->spi_lock);

	gf_dev->gBuffer = (unsigned char*)__get_free_pages(GFP_KERNEL, get_order(BUF_SIZE));
	if(gf_dev->gBuffer == NULL) {
		return -ENOMEM;
	}

	if (gf_parse_dts(gf_dev)) {
		gf_print("gf_parse_dts err! \n");
        goto error;
	}

	if (gf_power_init(gf_dev)) {
		gf_print("gf_power_init err! \n");
		goto error;
	}
	
	if (gf_power_on(gf_dev)) {
		gf_print("gf_power_on err! \n");
		goto error;
	}
    if (gf_read_chip_id(gf_dev)) {
		gf_print("gf_read_chip_id err! \n");
		goto error;
	}
     printk(KERN_ERR"gf_read_chip_id = %d", gf_read_chip_id(gf_dev));

        

	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(SPIDEV_MAJOR, CHRD_DRIVER_NAME, &gf_fops); //register char devices.
	if (status < 0) {
		gf_print("Failed to register char device!\n");
		goto error;
	}
	gf_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(gf_class)) {
		unregister_chrdev(SPIDEV_MAJOR, gf_driver.driver.name);
		gf_print("Failed to create class.\n");
		goto error;
	}

	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;
		gf_dev->devt = MKDEV(SPIDEV_MAJOR, minor);
		dev = device_create(gf_class, &gf_dev->spi->dev, gf_dev->devt,
				    gf_dev, GF_DEV_NAME);
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		gf_print("no minor number available!\n");
		status = -ENODEV;
	}

	if (status == 0) {
		set_bit(minor, minors);
		list_add(&gf_dev->device_entry, &device_list);
	} else {
		gf_dev->devt = 0;
	}
	mutex_unlock(&device_list_lock);

	if (status == 0) {
        spi_set_drvdata(spi, gf_dev);
		/*input device subsystem */
		gf_dev->input = input_allocate_device();
		if (gf_dev->input == NULL) {
			gf_print("Faile to allocate input device.\n");
			status = -ENOMEM;
		}

		gf_dev->notifier = goodix_noti_block;
		fb_register_client(&gf_dev->notifier);
		gf_reg_key_kernel(gf_dev);

		gf_hw_reset(gf_dev, 60);
        gf_spi_setup(gf_dev, 8*1000*1000);
		gf_spi_set_mode(gf_dev->spi, SPEED_1MHZ, 0);

        gf_gpio_as_int(gf_dev);
        if(gf_irq_num(gf_dev)) {
			gf_print("gf_irq_num err! \n");
            goto error;
		}
		
		ret = request_threaded_irq(gf_dev->spi->irq, NULL, gf_irq,
					   IRQF_TRIGGER_RISING | IRQF_ONESHOT, "gf", gf_dev);
		if (!ret) {
			enable_irq_wake(gf_dev->spi->irq);
			gf_dev->irq_enabled = 1;
			gf_disable_irq(gf_dev);
		}
		else {
			gf_print("request_threaded_irq ret: %d\n",  ret);
            goto error;
		}
	}
	else {
        goto error;
	}

	gf_dev->device_available = 1;
	gf_print("------------- succeed to gf_probe! --------------\n");
	
	full_fp_chip_name(GF_DEV_NAME);
	return status;

error:

	gf_cleanup(gf_dev);
	gf_dev->device_available = 0;
	if (gf_dev->devt != 0) {
		gf_print("Err: status = %d\n", status);
		mutex_lock(&device_list_lock);
		list_del(&gf_dev->device_entry);
		device_destroy(gf_class, gf_dev->devt);
		clear_bit(MINOR(gf_dev->devt), minors);
		mutex_unlock(&device_list_lock);
		
gfspi_probe_clk_enable_failed:
		gfspi_ioctl_clk_uninit(gf_dev);
gfspi_probe_clk_init_failed:
		if (gf_dev->input != NULL)
			input_unregister_device(gf_dev->input);
    }

    if(gf_dev->gBuffer != NULL) {
        free_pages((unsigned long)gf_dev->gBuffer, get_order(BUF_SIZE));
	}


	gf_print("~~~~ fail to gf_probe! ~~~~\n");

	return status;
}

static int gf_remove(struct spi_device *spi)
{
	struct gf_dev *gf_dev = &gf;
	FUNC_ENTRY();


	/* make sure ops on existing fds can abort cleanly */
	if (gf_dev->spi->irq)
		free_irq(gf_dev->spi->irq, gf_dev);

	if (gf_dev->input != NULL)
		input_unregister_device(gf_dev->input);
		input_free_device(gf_dev->input);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&gf_dev->device_entry);
	device_destroy(gf_class, gf_dev->devt);
	clear_bit(MINOR(gf_dev->devt), minors);

    gf_cleanup(gf_dev);
	gf_dev->device_available = 0;
    fb_unregister_client(&gf_dev->notifier);
	gfspi_ioctl_clk_uninit(gf_dev);

    if (gf_dev->users == 0) {
        if(gf_dev->gBuffer) 
            free_pages((unsigned long)gf_dev->gBuffer, get_order(BUF_SIZE));
    }
    else{
        gf_print("Not free_pages.\n");
    }

    mutex_unlock(&device_list_lock);

	FUNC_EXIT();
	return 0;
}

static int gf_suspend(struct spi_device *spi, pm_message_t mesg)
{
	gf_print("%s: enter\n", __func__);
#if 0 //defined(USE_SPI_BUS)
	struct gf_dev *gfspi_device;

	pr_debug("%s: enter\n", __func__);

	gfspi_device = spi_get_drvdata(spi);
	gfspi_ioctl_clk_disable(gfspi_device);
#endif
	gf_print("gf_suspend_test.\n");
	return 0;
}


static int gf_resume(struct spi_device *spi)
{
	pr_debug("%s: enter\n", __func__);
#if 0 //defined(USE_SPI_BUS)
	struct gf_dev *gfspi_device;

	pr_debug("%s: enter\n", __func__);

	gfspi_device = spi_get_drvdata(spi);
	gfspi_ioctl_clk_enable(gfspi_device);
#endif
	gf_print(KERN_ERR "gf_resume_test.\n");
	return 0;
}

int stop_sampling(struct gf_dev *gf_dev)
{
    unsigned short val = 0;

    gf_spi_read_word(gf_dev, 0x0840, &val);
    val &= 0xFF00;
    val |= 0x0001;
    gf_spi_write_word(gf_dev, 0x0840, val);

}

int start_sampling(struct gf_dev *gf_dev)
{
    unsigned short val = 0;

    gf_spi_read_word(gf_dev, 0x0840, &val);
    val &= 0xFF00;
    gf_spi_write_word(gf_dev, 0x0840, val);
}

int gf_read_pre_image(struct gf_dev *gf_dev)
{
    int cnt = 0;
    unsigned char status = 0;
    unsigned short r0836 = 0;
    int ret = -1;
    
    while (cnt < 100)
    {
        gf_spi_read_word(gf_dev, 0x0836, &r0836);
        status = (unsigned char)(r0836 >> 8);
        gf_print("status=0x%02x", status);

        if (status == GF_INT_IMAGE)
        {
            ret = 0;
            break;
        }
        else if (status != GF_INT_INVALID)
        {
            gf_spi_write_word(gf_dev, 0x0836, 0);
            //break;
        }

        mdelay(3);
        cnt++;
    }
    return ret;
}

int get_frame_data(struct gf_dev *gf_dev, unsigned char *pFrame, int len)
{
    int timeout = 0;
    unsigned short val = 0;
    unsigned short fifo_len = 0;
    unsigned short fifo_len_tmp = 0;
    unsigned short fifo_len_next = GF_READ_BUFF_SIZE1;
    unsigned short total_len = 0;
    const unsigned int memsize = 3 * 1024 + GF_RDATA_OFFSET;
    //unsigned char buf[3 * 1024 + GF_RDATA_OFFSET] = {0};
    unsigned char *buf = NULL;

    int ret = -1;
    
    if ((pFrame == NULL) || (len < GF_SPI_DATA_LEN)) {        
        gf_print("[%s]input error, len=%d", __func__, len);
        return -1;
    }

    buf = (unsigned char*)__get_free_pages(GFP_KERNEL, get_order(memsize));
    if (buf == NULL) {
        gf_print("[%s]get_free_pages error", __func__);
        return -1;
    }

    //1. check to see if FIFO is empty or not
    //[0x5204]&0x8000 == 0, data is valid
    timeout = 100; //100 * 1ms
    do{
        gf_spi_read_word(gf_dev, 0x5204, &val);
        if (!(val & (1 << 15)))
        {
            break;
        }
        mdelay(1);        
        timeout --;
    }while(timeout > 0);
    
    if (timeout <= 0)
    {

        free_pages((unsigned long)buf, get_order(len));
        gf_print("wait_data time out");
        return -1;
    }

    //2. notify_fw_reading_begin
    gf_spi_write_word(gf_dev, 0x0836, 0x0002);
    gf_print("fifo_reading_begin");

    //3. read data
    timeout = 100;
    do
    {
        gf_spi_read_word(gf_dev, 0x5204, &val);
        //gf_print("0x5204=0x%x", val);
        if (val & (1 << 15))
        {
            gf_print("fifo is empty, exit");
            break;
        }

        val &= 0x07ff;
        fifo_len_tmp = val * 2;

        if (fifo_len_tmp >= fifo_len_next)
        {
            fifo_len = fifo_len_tmp;
            fifo_len_next =
                    (fifo_len_next == GF_READ_BUFF_SIZE1) ?
                            GF_READ_BUFF_SIZE2 : GF_READ_BUFF_SIZE1;
            if (total_len + fifo_len <= GF_SPI_DATA_LEN)
            {

//                gf_secspi_read_fifo(gf_secdev, GF_DATA_ADDR, fifo_len - 4,
//                        buf + total_len);
//                gf_secspi_read_fifo(gf_secdev, GF_DATA_ADDR, 4,
//                        buf + total_len + fifo_len - 4);
                spi_clock_set(gf_dev, 12*1000*1000);
                gf_spi_setup(gf_dev, 12*1000*1000);

                gf_spi_read_bytes(gf_dev, GF_DATA_ADDR, fifo_len - 4, buf);
                memcpy(pFrame + total_len, buf + GF_RDATA_OFFSET, fifo_len - 4);
                total_len += fifo_len - 4;
                
                gf_spi_read_bytes(gf_dev, GF_DATA_ADDR, 4, buf);

                spi_clock_set(gf_dev, 1*1000*1000);
                gf_spi_setup(gf_dev, 1*1000*1000);

                memcpy(pFrame + total_len , buf + GF_RDATA_OFFSET, 4); 
                total_len += 4;
                
                //gf_print("fifo_read len=%d, total=%d", fifo_len, total_len);
                if (total_len == GF_SPI_DATA_LEN)
                {
                    ret = 0;
                    //ret = restructure_raw_data(buf, image->buf, total_len);
                    break;
                }
            }
            else
            {
                gf_print("error, total_len + fifo_len > GF_SPI_DATA_LEN");
            }
        }
        else
        {
            //gf_print("error, fifo_len_tmp = %d", fifo_len_tmp);
            //break;
        }

        timeout--;
        udelay(300);
    } while ((timeout > 0) && (total_len < GF_SPI_DATA_LEN));
    // end of ADC FIFO reading block

    //4. notify_fw_reading_end
    gf_spi_write_word(gf_dev, 0x0836, 0);
    gf_print("fifo_reading_end");


    free_pages((unsigned long)buf, get_order(memsize));
    return ret;
}

typedef struct
{
    unsigned short id;
    unsigned short raw[SENSOR_COL];
    unsigned short crc;
} gx_row_t;

static void endian_exchange(unsigned char* buf, int len)
{
    int i = 0;
    unsigned char buf_tmp = 0;

    for(i = 0; i < len /2; i++)
    {
        buf_tmp = buf[2 * i + 1];
        buf[2 * i + 1] = buf[2 * i];
        buf[2 * i] = buf_tmp;
    }
}

#define CRC_16_POLYNOMIALS 0x8005

unsigned short gf_calc_crc(unsigned char* pchMsg, unsigned short wDataLen)
{
    unsigned char i = 0, chChar = 0;
    unsigned short wCRC = 0xFFFF;

    while(wDataLen--)
    {
        chChar = *pchMsg;
        pchMsg++;
        wCRC ^= (((unsigned short) chChar) << 8);

        for(i = 0; i < 8; i++) {
            if (wCRC & 0x8000)
            {
                wCRC = (wCRC << 1) ^ CRC_16_POLYNOMIALS;    
            }
            else
            {
                wCRC <<= 1;
            }
        }
    }
    return wCRC;
}

static int restructure_raw_data(unsigned char *pFrame,
        unsigned char* pFpdata, int total_len)
{
    int i = 0, j = 0;
    unsigned short id = 0;
    unsigned short calc_crc = 0;
    unsigned char crc_buf[GF_ROW_DATA_SIZE] =
    { 0 };
    gx_row_t row =
    { 0 };

    if (total_len != SENSOR_ROW * GF_ROW_SIZE)
    {
        gf_print("error total_len=%d", total_len);
        return -1;
    }

    for (i = 0; i < SENSOR_ROW; i++)
    {
        memcpy(&row, pFrame + i * GF_ROW_SIZE, GF_ROW_SIZE);
        memcpy(crc_buf, row.raw, sizeof(crc_buf));
        /* Change byte order for calculating crc */
        //endian_exchange(crc_buf, GF_ROW_SIZE);
        endian_exchange((unsigned char*)&row, GF_ROW_SIZE);

        calc_crc = gf_calc_crc(crc_buf, sizeof(crc_buf));
        if (calc_crc == row.crc)
        {
            id = row.id;
            row.id = 4 * (id / 4) + (4 - id % 4) - 1;
            //gf_print("%d - > %d", id, row.id);
            for (j = 0; j < SENSOR_COL; j++)
            {
                row.raw[j] = row.raw[j] / 16;
            }
            memcpy(pFpdata + row.id * GF_ROW_DATA_SIZE,
                    (unsigned char*) row.raw, GF_ROW_DATA_SIZE);
        }
        else
        {
            gf_print( "Row %d crc error, calc_crc=%x, read_crc=%x",
                    row.id, calc_crc, row.crc);
            return -1;
        }
    }

    gf_print("crc success, read frame ok");
    return 0;

}

int gf_read_image(struct gf_dev *gf_dev, unsigned char* pImage, int len)
{
    //unsigned char buf[GF_SPI_DATA_LEN] = {0};   
    unsigned char *buf = NULL;   
    gf_print("%s\n", __func__);
 
    buf = (unsigned char*)__get_free_pages(GFP_KERNEL, get_order(GF_SPI_DATA_LEN));
    if (buf == NULL) {
        gf_print("[%s]get_free_pages error", __func__);
        return -1;
    }

    //input parameter validity check
    if ((pImage == NULL) || (len < SENSOR_ROW * SENSOR_COL * 2))
    {
        free_pages((unsigned long)buf, get_order(GF_SPI_DATA_LEN));
        gf_print("Input buffer is NULL in %s\n", __func__);
        return -1;
    }

    if (gf_read_pre_image(gf_dev) != 0)
    {
        free_pages((unsigned long)buf, get_order(GF_SPI_DATA_LEN));
        gf_print("gf_read_pre_image, timeout");
        return -1;
    }

    stop_sampling(gf_dev);

    if (get_frame_data(gf_dev, buf, GF_SPI_DATA_LEN) != 0)
    {
        free_pages((unsigned long)buf, get_order(GF_SPI_DATA_LEN));
        gf_print("gf_read_image, error");
        return -1;        
    }
    
    return restructure_raw_data(buf, pImage, GF_SPI_DATA_LEN);
        
}




/*-------------------------------------------------------------------------*/
/* Read-only message with current device setup */
static ssize_t gf_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    struct gf_dev *gf_dev = filp->private_data;
    ssize_t			status = 0;
    int ret = -1;
    const unsigned int len = SENSOR_ROW * SENSOR_COL * 2;
    unsigned char *bufImage = NULL;
    //long int t1, t2;
    FUNC_ENTRY();

    gf_print("[%s]buf=%p, count=%d",__func__, buf, count);

    if ((count > len)||(count == 0)) {
        gf_print("Max size for write buffer is %d. wanted length is %d\n", len, (unsigned int)count);
        FUNC_EXIT();
        return -EMSGSIZE;
    }

    bufImage = (unsigned char*)__get_free_pages(GFP_KERNEL, get_order(len));
    if (bufImage == NULL){
        gf_print("get_free_pages error");
    }

    gf_dev = filp->private_data;

    
    //t1 = ktime_to_us(ktime_get());
    
    mutex_lock(&gf_dev->buf_lock);
    ret = gf_read_image(gf_dev, bufImage, len);
    if(ret == 0) {
        ret = copy_to_user(buf, bufImage, len);
        if (ret == 0){
            status = len;
        }
        else{
            pr_err("copy_to_user error.");
            status = -EFAULT;
        }
    } else {
        pr_err("Failed to read data from FIFO.\n");
        status = -EFAULT;
    }
    // t2 = ktime_to_us(ktime_get());
    //gf_print("read time use: %ld\n", t2-t1);
    mutex_unlock(&gf_dev->buf_lock);			   

    free_pages((unsigned long)bufImage, get_order(len));
    return status;
}

static int __init gf_init(void)
{
	int status;
	FUNC_ENTRY();

    spi_register_board_info(spi_board_devs,ARRAY_SIZE(spi_board_devs));
    status = spi_register_driver(&gf_driver);
    if (status < 0) {
    	gf_print("Failed to register SPI driver.");
    }
    return status;
}

static void __exit gf_exit(void)
{
    FUNC_ENTRY();
	spi_unregister_driver(&gf_driver);
}




module_init(gf_init);
module_exit(gf_exit);

MODULE_AUTHOR("Jiangtao Yi, <yijiangtao@goodix.com>");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:gf-spi");

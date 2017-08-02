#include "tpd.h"
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>

#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
/* #define TIMER_DEBUG */


#include <linux/timer.h>


#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/device.h>


#include "tpd_custom_ft6x06.h"

#include <linux/module.h>

#include <linux/jiffies.h>

 extern void mt65xx_eint_unmask(unsigned int line);
 extern void mt65xx_eint_mask(unsigned int line);

#ifdef TPD_FACTORY_FW_UPDATE 
 extern int ftm_ft6x06_force_update;
#endif

#define ISP_FLASH_SIZE	0x8000 //32KB

#define FT6X06_FIRMWAIR_VERSION_D

extern void ft6x06_power_up(void);
extern void ft6x06_power_down(void);
/*==========================================*/
static tinno_ts_data *g_pts = NULL;

static int fts_i2c_read_a_byte_data(struct i2c_client *client, char reg)
{
	int ret;
	uint8_t iic_data;
	ret = i2c_smbus_read_i2c_block_data(client, reg, 1, &iic_data);
	if (iic_data < 0 || ret < 0){
		CTP_DBG("%s: i2c error, ret=%d\n", __func__, ret);
		return -1;
	}
	return (int)iic_data;
}

static inline int _lock(atomic_t *excl)
{
	if (atomic_inc_return(excl) == 1) {
		return 0;
	} else {
		atomic_dec(excl);
		return -1;
	}
}

static inline void _unlock(atomic_t *excl)
{
	atomic_dec(excl);
}

static ssize_t fts_isp_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{	
	//ret = copy_to_user(buf,&acc, sizeof(acc));
	CTP_DBG("");
	return -EIO;
}

static DECLARE_WAIT_QUEUE_HEAD(waiter_write);
static volatile	int write_flag;

 static int isp_thread(void *para)
 {
	int rc = 0;
	struct sched_param param = { .sched_priority = 4 };
	tinno_ts_data *ts = (tinno_ts_data *)para;
	sched_setscheduler(current, SCHED_RR, &param);
	set_current_state(TASK_RUNNING);	
	rc = fts_i2c_write_block(ts->isp_pBuffer, ts->isp_code_count);
	if (rc < ts->isp_code_count) {
		CTP_DBG("i2c_transfer failed(%d)", rc);
		ts->isp_code_count = -EAGAIN;
	} else{
		ts->isp_code_count = rc;
	}
	write_flag = 1;
	wake_up_interruptible(&waiter_write);
		
	return 0;
 }

static ssize_t fts_isp_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
	int rc = 0;
	tinno_ts_data *ts = file->private_data;
	void __user *start = buf;
	CTP_DBG("count = %d, offset=%d", count, (int)(*offset));
	
	if ( count > ISP_FLASH_SIZE ){
		CTP_DBG("isp code is too long.");
		return -EDOM;
	}

	if ( copy_from_user((char *)ts->isp_pBuffer, start, count) ){
		CTP_DBG("copy failed(%d)", rc);
		return -EACCES;
	}

	ts->isp_code_count = count;
	write_flag = 0;
	ts->thread_isp= kthread_run(isp_thread, ts, TPD_DEVICE"-ISP");
	if (IS_ERR(ts->thread_isp)){ 
		rc = PTR_ERR(ts->thread_isp);
		CTP_DBG(" failed to create kernel thread: %d\n", rc);
		return rc;
	} 

	//block user thread
	wait_event_interruptible(waiter_write, write_flag!=0);
	
	return ts->isp_code_count;
}

static int fts_isp_open(struct inode *inode, struct file *file)
{
	CTP_DBG("try to open isp.");

	if ( atomic_read( &g_pts->ts_sleepState ) ){
		CTP_DBG("TP is in sleep state, please try again latter.");
		return -EAGAIN;
	}

	if (_lock(&g_pts->isp_opened)){
		CTP_DBG("isp is already opened.");
		return -EBUSY;
	}
		
	file->private_data = g_pts;

	g_pts->isp_pBuffer = (uint8_t *)kmalloc(ISP_FLASH_SIZE, GFP_KERNEL);
	if ( NULL == g_pts->isp_pBuffer ){
		_unlock ( &g_pts->isp_opened );
		CTP_DBG("no memory for isp.");
		return -ENOMEM;
	}
	
//	mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	
//	ft6x06_complete_unfinished_event();
	
#ifdef CONFIG_TOUCHSCREEN_FT5X05_DISABLE_KEY_WHEN_SLIDE
		fts_6x06_key_cancel();
#endif

	wake_lock(&g_pts->wake_lock);

	CTP_DBG("isp open success.");
	return 0;
}

static int fts_isp_close(struct inode *inode, struct file *file)
{
	tinno_ts_data *ts = file->private_data;
	
	CTP_DBG("try to close isp.");
	
	if ( !atomic_read( &g_pts->isp_opened ) ){
		CTP_DBG("no opened isp.");
		return -ENODEV;
	}
	
	kfree(ts->isp_pBuffer);
	ts->isp_pBuffer = NULL;
	
	file->private_data = NULL;
	
	_unlock ( &ts->isp_opened );
	
	wake_unlock(&ts->wake_lock);
	
//	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);  
	
	CTP_DBG("close isp success!");
	return 0;
}
extern void fts_6x06_hw_reset_isp(int para);

static int upgrade_start_time = 4;
static int fts_switch_to_update(tinno_ts_data *ts)
{
	int ret = 0, i=0;
	uint8_t arrCommand[] = {0x55, 0xaa};
	
	CTP_DBG("fts_switch_to_update zx \n");
	//LINE <ft6x06> <DATE20140910> <modify for ft6336> yolo
	
#if 1	
	/*write 0xaa to register 0xfc*/
	//ret = fts_write_reg(0xFC, 0xAA);
	ret = fts_write_reg(0xBC, 0xAA); // zhangxiaofei add for ft6x06 touch panel driver
	if (ret < 0) {
		//CTP_DBG("write 0xaa to register 0xfc failed");
		CTP_DBG("write 0xaa to register 0xbc failed");
		goto err;
	}
	//LINE <ft6x06> <DATE20140910> <modify for ft6336> yolo
		msleep(10);//50

	/*write 0x55 to register 0xfc*/
	//ret = fts_write_reg(0xFC, 0x55);
	ret = fts_write_reg(0xBC, 0x55); // zhangxiaofei add for ft6x06 touch panel driver
	if (ret < 0) {
		//CTP_DBG("write 0x55 to register 0xfc failed");
		CTP_DBG("write 0x55 to register 0xbc failed");
		goto err;
	}
		//LINE <ft6x06> <DATE20140910> <modify for ft6336> yolo
    	upgrade_start_time += 2;
		msleep(upgrade_start_time); //40

#else
    //fts_6x06_hw_reset();
//LINE <ft6x06> <DATE20140910> <modify for ft6336> yolo
		upgrade_start_time += 2;
   		fts_6x06_hw_reset_isp(upgrade_start_time);
#endif 

	do{
		mutex_lock(&ts->mutex);
		ret = i2c_master_send(ts->client, (const char*)arrCommand, sizeof(arrCommand));
		mutex_unlock(&ts->mutex);
		++i;
	}while(ret < 0 && i < 5);

	ret = 0;
err:
       CTP_DBG("fts_switch_to_update ret = %d,i=%d \n",ret,i);
	return ret;
}

static int fts_mode_switch(tinno_ts_data *ts, int iMode)
{
	int ret = 0;
	
	CTP_DBG("iMode=%d \n ", iMode);
	
	if ( FTS_MODE_OPRATE == iMode ){
	}
	else if (FTS_MODE_UPDATE == iMode){
		ret = fts_switch_to_update(ts);
	}
	else if (FTS_MODE_SYSTEM == iMode){
	}
	else{
		CTP_DBG("unsupport mode %d", iMode);
	}
	return ret;
}


//BEGIN <add changing flag> <DATE20130330> <add changing flag> zhangxiaofei
int fts_ft6x06_switch_charger_status(u8 charger_flag)
{
    int ret = 0;
    u8 vl_read_charger_flag = 0;

    CTP_DBG("charger_flag =  %d\n",charger_flag);


    ret = fts_write_reg(0x8b, charger_flag);
    if (ret < 0) {
        CTP_DBG("write  charger_flag(%d) to register 0x8b failed", charger_flag);
        goto err;
    }
    msleep(50);

// read check
#if 0
    ret = fts_read_reg(0x8b, &vl_read_charger_flag);
    if (ret < 0){
        CTP_DBG("read  0x8b failed");
        goto err;
    }
    
    CTP_DBG("read  vl_read_charger_flag = %d, from register 0x8b", vl_read_charger_flag);
#endif    

err:
	return ret;
}
//END <add changing flag> <DATE20130330> <add changing flag> zhangxiaofei


static int fts_ctpm_auto_clb(void)
{
    unsigned char uc_temp[1];
    unsigned char i ;

    CTP_DBG("start auto CLB.\n");
    msleep(200);
    fts_write_reg(0, 0x40);  
    mdelay(100);   //make sure already enter factory mode
    fts_write_reg(2, 0x4);  //write command to start calibration
    mdelay(300);
    for(i=0;i<100;i++)
    {
        fts_read_reg(0,uc_temp);
        if ( ((uc_temp[0]&0x70)>>4) == 0x0)  //return to normal mode, calibration finish
        {
            break;
        }
        mdelay(200);
        CTP_DBG("waiting calibration %d\n",i);
        
    }
    CTP_DBG("calibration OK.\n");
    
    msleep(300);
    fts_write_reg(0, 0x40);  //goto factory mode
    mdelay(100);   //make sure already enter factory mode
    fts_write_reg(2, 0x5);  //store CLB result
    mdelay(300);
    fts_write_reg(0, 0x0); //return to normal mode 
    msleep(300);
    CTP_DBG("store CLB result OK.\n");
    return 0;
}

static int ft6x06_get_tp_id(tinno_ts_data *ts, int *ptp_id)
{
	int rc;
	char tp_id[2];
	*ptp_id = -1;
	
	CTP_DBG("Try to get TPID!");
	
	rc = fts_cmd_write(0x90, 0x00, 0x00, 0x00, 4);
	if (rc < 4) {
		CTP_DBG("i2c_master_send failed(%d)", rc);
		return -EIO;
	} 
	
	ts->client->addr = ts->client->addr & I2C_MASK_FLAG;
	rc = i2c_master_recv(ts->client, tp_id, 2);
	if (rc < 2) {
		CTP_DBG("i2c_master_recv failed(%d)", rc);
		return -EIO;
	} 

	*ptp_id = (( int )tp_id[0] << 8) | (( int )tp_id[1]);
			
	return 0;
}

static int ft6x06_get_fw_version(tinno_ts_data *ts)
{
	int ret;
	uint8_t fw_version;
	ret = fts_read_reg(0xA6, &fw_version);
	if (ret < 0){
		CTP_DBG("i2c error, ret=%d\n", ret);
		return -1;
	}
	CTP_DBG("fw_version=0x%X\n", fw_version);
	return (int)fw_version;
}

int ft6x06_get_ic_status(void)
{
	int ret;
	uint8_t ic_status;
	ret = fts_read_reg(0x00, &ic_status);
	if (ret < 0){
		CTP_DBG("i2c error, ret=%d\n", ret);
		return -1;
	}
	CTP_DBG("ic_status=0x%X\n", ic_status);
	return (int)0;
}
int get_fw_version_ext(void)
{
	int version = -1;
	if(g_pts)
		version = ft6x06_get_fw_version(g_pts);
	CTP_DBG("fw_version=0x%X\n", version);
	return version;
}
EXPORT_SYMBOL(get_fw_version_ext);

static int ft6x06_get_vendor_from_bootloader(tinno_ts_data *ts, uint8_t *pfw_vendor, uint8_t *pfw_version)
{
	int rc = 0, tp_id;
	uint8_t version_id = 0, buf[5];
       int resettimes = 0;
	
	CTP_DBG("Try to switch to update mode!");
 do{
	fts_mode_switch(ts, FTS_MODE_UPDATE);
	ft6x06_get_tp_id(ts, &tp_id);	
	if ( FTS_CTP_FIRWARE_ID == tp_id ){
		//rc = -EINVAL;
		//goto err;
    	    CTP_DBG("check id OK \n");
    	    break;
	}
    }while(0==tp_id&&(resettimes++)<20);
	//Get vendor ID.
	rc = fts_cmd_write(0xcd, 0x00, 0x00, 0x00, 1);
	if (rc < 1) {
		CTP_DBG("i2c_master_send failed(%d)", rc);
		goto err;
	} 
	rc = i2c_master_recv(ts->client, &version_id, 1);
	if (rc < 1) {
		CTP_DBG("i2c_master_recv failed(%d)", rc);
		goto err;
	} 
	
	//*pfw_version = version_id;
	*pfw_version = -1;//Force to update.
	CTP_DBG("bootloader version = 0x%x\n", version_id);

	/* --------- read current project setting  ---------- */
	//set read start address
	//rc = fts_cmd_write(0x03, 0x00, 0x78, 0x00, 4);  // FT6x06
	rc = fts_cmd_write(0x03, 0x00, 0x07, 0xB0, 4);  // FT6X36
	if (rc < 4) {
		CTP_DBG("i2c_master_send failed(%d)", rc);
		goto err;
	} 
	
	rc = i2c_master_recv(g_pts->client, buf, sizeof(buf));
	if (rc < 0){
		CTP_DBG("i2c_master_recv failed(%d)", rc);
		goto err;
	}

	CTP_DBG("vendor_id = 0x%x\n", buf[4]);
	
	*pfw_vendor = buf[4];
	
	CTP_DBG("Try to reset TP!");
	rc = fts_cmd_write(0x07,0x00,0x00,0x00,1);
	if (rc < 0) {
		CTP_DBG("reset failed");
		goto err;
	}
	msleep(200);
	
	return 0;
err:
	return rc;
}

int ft6x06_get_vendor_version(tinno_ts_data *ts, uint8_t *pfw_vendor, uint8_t *pfw_version)
{
	int ret;
	*pfw_version = ft6x06_get_fw_version(ts);

	ret = fts_read_reg(0xA8, pfw_vendor);
	if (ret < 0){
		CTP_DBG("i2c error, ret=%d\n", ret);
		return ret;
	}
	CTP_DBG("one fw_vendor=0x%X, fw_version=0x%X\n", *pfw_vendor, *pfw_version);
	if ( 0xA8 == *pfw_vendor || 0x00 == *pfw_vendor ){
		CTP_DBG("FW in TP has problem, get factory ID from bootloader.\n");
		ret = ft6x06_get_vendor_from_bootloader(ts, pfw_vendor, pfw_version);
		if (ret){
			CTP_DBG("ft6x06_get_vendor_from_bootloader error, ret=%d\n", ret);
			return -EFAULT;
		}
        
#if defined(CONFIG_PROJECT_V3911) || defined(CONFIG_PROJECT_V3913) // Jiangde, 0x11==YixingTP
        ret = 0xFF; // get from bl flag
#endif
	}

#if defined(CONFIG_PROJECT_V3911) || defined(CONFIG_PROJECT_V3913) // Jiangde, 0x11==YixingTP
    if (0xFF != ret                                                // if not get from bl, we will do it
        && (0x80 == *pfw_vendor || 0x11 == *pfw_vendor || 0x81 == *pfw_vendor) )
    {
        uint8_t iVendor = 0;
        uint8_t iVersion = 0;
        
        ret = ft6x06_get_vendor_from_bootloader(ts, &iVendor, &iVersion);
        CTP_DBG("HJDDbgTP, Yixing TP iVendor=0x%x, iVersion=0x%x\n", iVendor, iVersion);\

        if (*pfw_vendor != iVendor)
        {
            CTP_DBG("HJDDbgTP, use wrong fireware? *pfw_vendor=0x%x, iVendor=0x%x\n", *pfw_vendor, iVendor);\
            *pfw_vendor = iVendor;
        }
    }
#endif

	CTP_DBG("two fw_vendor=0x%X, fw_version=0x%X\n", *pfw_vendor, *pfw_version);
	return 0;
}
EXPORT_SYMBOL(ft6x06_get_vendor_version);

extern void ft6x06_tp_upgrade(const char * ftbin_buf, int buf_len);
int ft6x06_fw_upgrade_from_file(void)
{
    int ret = -1;

    printk("[ft6x06]  Entry ft6x06_fw_upgrade_from_file \n");
	
    ft6x06_tp_upgrade(ft6x06_file_fw_data,ft6x06_file_fw_data_len);

    return 0;
}

static int fts_isp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	tinno_ts_data *ts = file->private_data;
	int flag;
	int rc = 0;
	
	if ( !atomic_read( &g_pts->isp_opened ) ){
		CTP_DBG("no opened isp.");
		return -ENODEV;
	}
	
	/* check cmd */
	if(_IOC_TYPE(cmd) != FT6x06_IOCTLID)	
	{
		CTP_DBG("cmd magic type error");
		return -EINVAL;
	}

	if(_IOC_DIR(cmd) & _IOC_READ)
		rc = !access_ok(VERIFY_WRITE,(void __user*)arg, _IOC_SIZE(cmd));
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
		rc = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));
	if(rc)
	{
		CTP_DBG("cmd access_ok error");
		return -EINVAL;
	}

	switch (cmd) {
        case FT6x06_IOCTL_FW_UPDATE:
         //   printk("[ft6x06] tp upgrade: ft6x06_file_fw_data=%x, ft6x06_file_fw_data_len = %x \n",
          //         ft6x06_file_fw_data, ft6x06_file_fw_data_len);
            if((ft6x06_file_fw_data == NULL) || (ft6x06_file_fw_data_len <= 0))
            {
                return 0;
            }
            else
            {
                int ret = 0;
                ret = ft6x06_fw_upgrade_from_file();
                printk("[ft6x06]  ft6x06_fw_upgrade_from_file ret =%x ", ret);
                ft6x06_file_fw_data = NULL;
                ft6x06_file_fw_data_len = 0;
                return ret;
            }
            break;

        case FT6x06_IOCTL_TP_UPGRADE_SET_BIN_BUF:
            ft6x06_file_fw_data = (uint8_t *)arg;
            break;

        case FT6x06_IOCTL_TP_UPGRADE_SET_BIN_LEN:
            ft6x06_file_fw_data_len = (int)arg;
            break;
			
		case FT5X06_IOCTL_GET_VENDOR_VERSION:
		{
			int rc;
			uint8_t vendor = 0;
			uint8_t version = 0;
			CTP_DBG("Try to get vendor_version!");
			rc = ft6x06_get_vendor_version(ts, &vendor, &version);
			if ( rc < 0 ){
			rc = -EFAULT;
			break;
			}
			CTP_DBG("vendor version =%d, version=%d!", vendor, version);
			flag = (vendor<<8)|version;
			if(copy_to_user(argp,&flag, sizeof(int))!=0)
			{
			CTP_DBG("copy_to_user error");
			rc = -EFAULT;
			}
			break;
		}	
		
		case FT5X06_IOCTL_SET_POWER_UP:
		{
			CTP_DBG("ft6x06 set power UP!!!!!");
			ft6x06_power_up();
			break;
		}
		
		case FT5X06_IOCTL_SET_POWER_DOWN:
		{
			CTP_DBG("ft6x06 set power down!!!!!");
			ft6x06_power_down();
			break;
		}
		
        default:
            break;
    }

	return rc;
}

//BEGIN<touch panel><date20131021><tp auto update>yinhuiyong
 /* should never be called */
extern u8 fts_cmd_write5(u8 btcmd,u8 btPara1,u8 btPara2,u8 btPara3,u8 btPara4,u8 num);
void ft6x06_tp_upgrade(const char * ftbin_buf, int buf_len)
{
       unsigned char reg_val[2]={0};
	int vl_tp_ver = 0x00;

	int vl_bin_ver = 0x00;

	int rc = 0;

	int i = 0;
       int resettimes=0;

	int tp_id = 0;


	CTP_DBG("entry ft6x06_tp_upgrade \n");

	//vl_bin_ver = ftbin_buf[buf_len-2];
		vl_bin_ver = ftbin_buf[0x10A];
       CTP_DBG("entry ft6x06_tp_upgrade vl_bin_ver=%x \n",vl_bin_ver);
	vl_tp_ver = get_fw_version_ext(); 

#ifdef TPD_FACTORY_FW_UPDATE 
        CTP_DBG("bin_ver= 0x%x, current_tp_ver= 0x%x,ftm_ft6x06_force_update:%d", vl_bin_ver, vl_tp_ver,ftm_ft6x06_force_update);

        if(vl_bin_ver > vl_tp_ver || ftm_ft6x06_force_update == 1)

	#else

	if(vl_bin_ver > vl_tp_ver )
  #endif
	//if (1)
	{
		CTP_DBG(" have new ver, upgrade... \n");

		CTP_DBG("Step 1:switch mode from WORK to UPDATE...");
             do{
                
		rc = fts_mode_switch(g_pts, (int)FTS_MODE_UPDATE);
		if(rc)
		{
			CTP_DBG("switch to update mode error");
			return; //  -EIO;
		}

		msleep(10);


		CTP_DBG("Step 2:check READ-ID...\n");

		
		
		for( i = 0; i < 3; i++ )
		{
			rc = ft6x06_get_tp_id(g_pts, &tp_id);
			CTP_DBG("TPID=0x%X!rc =%d\n", tp_id,rc);
			if(rc)
			{
				CTP_DBG("Get tp ID error(%d)\n", rc);
				return ; // rc = -EIO;
			} 
			else
			{
				CTP_DBG("tp_id=0x%X\n", tp_id);
	            // 5316-->7907      5206-->7903
				if (( 0x7908 == tp_id )||(0x7918 == tp_id)||(0x791c == tp_id))
                                                     {                            
					CTP_DBG("check id OK \n");
					break;
				}
			}
		}
             }while(0==tp_id&&(resettimes++)<20);
             CTP_DBG("\n tp_id = %d, resettimes = %d ",tp_id,resettimes);
		if ( i == 3 )
		{
			CTP_DBG("\n CHECK-ID error!");
			return; // goto err_read_id;
		}
              rc = fts_cmd_write5(0x90, 0, 0, 0, 0,5);
		if (rc < 1) {
			CTP_DBG("erase failed");
			return;
		}
		CTP_DBG("Step 4:erase falsh...");

		rc = fts_cmd_write(0x61, 0, 0, 0, 1);
		if (rc < 1) {
			CTP_DBG("erase failed");
			return;
		}
		//msleep(1500);
             msleep(2000);   // FT6X36
             for(i=0; i < 200; i++)
             {
                rc = fts_cmd_write(0x6A, 0, 0, 0, 4);
                rc = i2c_master_recv(g_pts->client, reg_val, 2);
                if(0xB0 == reg_val[0] && 0x02 == reg_val[1])
                {
                    CTP_DBG("Step 4-1:erase falsh...");
                    break;
		   }
                msleep(50);   
             }
             
             
		CTP_DBG("Step 6:write firmware(FW) to ctpm flash");


		rc = fts_i2c_write_block(ftbin_buf, buf_len);

		msleep(100); 

		if ( rc != buf_len ){
			CTP_DBG("\n Error,  write rc (%d) != buf_len(%d) !!!!!!!!!", rc , buf_len);
	              msleep(5); 
			return;
		}
		else {
		CTP_DBG("write OK \n");
		}

		#if 0
		CTP_DBG("Step 9:read out checksum...");
			uint8_t check_sum;
			CTP_DBG("Try to get checksum!");
			fts_cmd_write(0xCC,0x00,0x00,0x00,1);
			ts->client->addr = ts->client->addr & I2C_MASK_FLAG;
			rc = i2c_master_recv(ts->client, &check_sum, 1);
			if (rc < 0) {
				CTP_DBG("read checksum failed");
			}
			CTP_DBG("checksum=%d!", check_sum);
			flag = check_sum;
			if(copy_to_user(argp,&flag, sizeof(int))!=0)
			{
				CTP_DBG("copy_to_user error");
				return;//rc = -EFAULT;
			}
		#endif
		
		// same as hw reset, tp comand reset sometime do not work
		#if 1
		CTP_DBG("Step 10:reset the new FW...");

		rc = fts_cmd_write(0x07,0x00,0x00,0x00,1);
		if (rc < 0) {
			CTP_DBG("reset failed");

			return;
		}

	       msleep(100);  //make sure CTP startup normally

		CTP_DBG("Step 10:update the new FW.success..");
	    #endif

		// 6x06 6x08 do not cal
		//CTP_DBG("Step 11: Calibrate the TP, please don't touch the TP before the operation is finished! ...");
		//fts_ctpm_auto_clb();
		
		fts_6x06_hw_reset();

	}
	else
	{
		CTP_DBG("tp firmware needn't upgrade. \n");
	}
	

}
//END<touch panel><date20131021><tp auto update>yinhuiyong
static const struct file_operations fts_isp_fops = {
	.owner = THIS_MODULE,
	.read = fts_isp_read,
	.write = fts_isp_write,
	.open = fts_isp_open,
	.release = fts_isp_close,
	.unlocked_ioctl = fts_isp_ioctl,
};

static struct miscdevice fts_isp_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "fts_isp",
	.fops = &fts_isp_fops,
};

 /* called when loaded into kernel */
 int fts_6x06_isp_init( tinno_ts_data *ts ) 
 {
 	int ret;
	CTP_DBG("MediaTek FT6x06 touch panel isp init\n");
	
	wake_lock_init(&ts->wake_lock, WAKE_LOCK_SUSPEND, "fts_tp_isp");
	
	ret = misc_register(&fts_isp_device);
	if (ret) {
		misc_deregister(&fts_isp_device);
		CTP_DBG(KERN_ERR "fts_isp_device device register failed (%d)\n", ret);
		goto exit_misc_device_register_failed;
	}
	
	g_pts = ts;
	return 0;
	
exit_misc_device_register_failed:
	misc_deregister(&fts_isp_device);
	fts_isp_device.minor = MISC_DYNAMIC_MINOR;
	wake_lock_destroy(&ts->wake_lock);
	return ret;
 }
 
 /* should never be called */
 void fts_6x06_isp_exit(void) 
{
	CTP_DBG("MediaTek FT6x06 touch panel isp exit\n");
	if ( g_pts ){
		misc_deregister(&fts_isp_device);
		fts_isp_device.minor = MISC_DYNAMIC_MINOR;
		wake_lock_destroy(&g_pts->wake_lock);
		g_pts = NULL;
	}
}
 

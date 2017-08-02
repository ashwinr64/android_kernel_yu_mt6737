#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/module.h>
//#include "kd_camera_hw.h"
#include "kd_camera_typedef.h"
#include "cam_cal.h"
#include "cam_cal_define.h"
#include "AR1335_eeprom.h"
//#include <asm/system.h>  // for SMP
#include <linux/dma-mapping.h>
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif


#define CAM_CALINF(fmt, arg...)    pr_debug("[%s] " fmt, __FUNCTION__, ##arg)
#define CAM_CALDB(fmt, arg...)     pr_debug("[%s] " fmt, __FUNCTION__, ##arg)
#define CAM_CALERR(fmt, arg...)    pr_err("[%s] " fmt, __FUNCTION__, ##arg)

static DEFINE_SPINLOCK(g_CAM_CALLock); // for SMP

#define CAM_CAL_DRVNAME "AR1335_CAM_CAL_DRV"  

static dev_t g_CAM_CALdevno = MKDEV(CAM_CAL_DEV_MAJOR_NUMBER,0);
static struct cdev * g_pCAM_CAL_CharDrv = NULL;
static struct class *CAM_CAL_class = NULL;
static atomic_t g_CAM_CALatomic;

const u8 ar1335_otp_version[] = {0xBC,0x36,0x2C,0x01,0x54};
extern u16 Darling_AR1335_OTP_Data[9];
extern u16 Sunwin_AR1335_OTP_Data[9];

#ifdef CONFIG_COMPAT
static int compat_put_cal_info_struct(
            COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data->u4Offset);
    err |= put_user(i, &data32->u4Offset);
    err |= get_user(i, &data->u4Length);
    err |= put_user(i, &data32->u4Length);
    /* Assume pointer is not change */
#if 1
    err |= get_user(p, &data->pu1Params);
    err |= put_user(p, &data32->pu1Params);
#endif
    return err;
}
static int compat_get_cal_info_struct(
            COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data32->u4Offset);
    err |= put_user(i, &data->u4Offset);
    err |= get_user(i, &data32->u4Length);
    err |= put_user(i, &data->u4Length);
    err |= get_user(p, &data32->pu1Params);
    err |= put_user(compat_ptr(p), &data->pu1Params);

    return err;
}

static long AR1335eeprom_Ioctl_Compat(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long ret;
    COMPAT_stCAM_CAL_INFO_STRUCT __user *data32;
    stCAM_CAL_INFO_STRUCT __user *data;
    int err;
	CAM_CALDB("[CAMERA SENSOR] AR1335_eeprom_DEVICE_ID,%p %p %x ioc size %d\n",filp->f_op ,filp->f_op->unlocked_ioctl,cmd,_IOC_SIZE(cmd) );

    if (!filp->f_op || !filp->f_op->unlocked_ioctl)
        return -ENOTTY;

    switch (cmd) {

    case COMPAT_CAM_CALIOC_G_READ:
    {
        data32 = compat_ptr(arg);
        data = compat_alloc_user_space(sizeof(*data));
        if (data == NULL)
            return -EFAULT;

        err = compat_get_cal_info_struct(data32, data);
        if (err)
            return err;

        ret = filp->f_op->unlocked_ioctl(filp, CAM_CALIOC_G_READ,(unsigned long)data);
        err = compat_put_cal_info_struct(data32, data);


        if(err != 0)
            CAM_CALERR("[CAMERA SENSOR] compat_put_acdk_sensor_getinfo_struct failed\n");
        return ret;
    }
    default:
        return -ENOIOCTLCMD;
    }
}
#endif /*CONFIG_COMPAT*/

static void Sunwin_AR1335_read_otp(u32 offset,BYTE* data,u32 size)
{       
	if((0x3B==offset) &&(1==size))
	{
		*data = 0xF;//AWBAFConfig =0xF mean,do af  AWBcalibration
             //   *data = 0xE;
    }
	else if((0x3c)==offset&&(6==size)) //awb data
	{
	    memcpy((void *)data,(void *)(&Sunwin_AR1335_OTP_Data[1]),6);//	memcpy((void *)data,(void *)&af_zhi,size);
	}
    	else if((0x42)==offset&&(6==size))
	{
		memcpy((void *)data,(void *)(&Sunwin_AR1335_OTP_Data[1+3]),6);//read_imx258_eeprom_size(0xA0,offset,data,size);
	}
       else if((0x48)==offset&&(2==size))
	{
		memcpy((void *)data,(void *)(&Sunwin_AR1335_OTP_Data[1+3+3]),2);//read_imx258_eeprom_size(0xA0,offset,data,size);
	}
       else if((0x4A)==offset&&(2==size))
	{
		memcpy((void *)data,(void *)(&Sunwin_AR1335_OTP_Data[1+3+3+1]),2);//read_imx258_eeprom_size(0xA0,offset,data,size);
	}
}
static void Darling_AR1335_read_otp(u32 offset,BYTE* data,u32 size)
{
	if((0x3B==offset) &&(1==size))
	{
		*data = 0xF;//AWBAFConfig =0xF mean,do af  AWBcalibration
		//*data = 0xE;
	}
	else if((0x3c)==offset&&(6==size)) //awb data
	{
	    memcpy((void *)data,(void *)(&Darling_AR1335_OTP_Data[1]),6);//	memcpy((void *)data,(void *)&af_zhi,size);
	}
    	else if((0x42)==offset&&(6==size))
	{
		memcpy((void *)data,(void *)(&Darling_AR1335_OTP_Data[1+3]),6);//read_imx258_eeprom_size(0xA0,offset,data,size);
	}
       else if((0x48)==offset&&(2==size))
	{
		memcpy((void *)data,(void *)(&Darling_AR1335_OTP_Data[1+3+3]),2);//read_imx258_eeprom_size(0xA0,offset,data,size);
	}
       else if((0x4A)==offset&&(2==size))
	{
		memcpy((void *)data,(void *)(&Darling_AR1335_OTP_Data[1+3+3+1]),2);//read_imx258_eeprom_size(0xA0,offset,data,size);
	}
}
static int selective_read_region(u32 offset, BYTE* data,u32 size)
{
    u8 otp_af_data[2]={0};

	if((0==offset) &&(4==size))
	{
		memcpy((void *)data,(void *)&ar1335_otp_version[offset],size);
	}
	else
	{   
              if(Sunwin_AR1335_OTP_Data[0] == 0x06)//sunwin sensor
              {
                    Sunwin_AR1335_read_otp(offset,data,size); 
              }
             else if(Darling_AR1335_OTP_Data[0] == 0x05) //darling sensor
              {
                    Darling_AR1335_read_otp(offset,data,size); 
             }
	}
   return   size;
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
    u8 * pu1Params = NULL;
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
            CAM_CALERR(" ioctl allocate mem failed\n");
            return -ENOMEM;
        }

        if(_IOC_WRITE & _IOC_DIR(a_u4Command))
        {
            if(copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT)))
            {    //get input structure address
                kfree(pBuff);
                CAM_CALERR("ioctl copy from user failed\n");
                return -EFAULT;
            }
        }
    }
    //------------------------------------------------------//
    ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
    pu1Params = (u8*)kmalloc(ptempbuf->u4Length,GFP_KERNEL);
    if(NULL == pu1Params)
    {
        kfree(pBuff);
        CAM_CALERR("ioctl allocate mem failed\n");
        return -ENOMEM;
    }
    if(copy_from_user((u8*)pu1Params ,  (u8*)ptempbuf->pu1Params, ptempbuf->u4Length))
    {
        kfree(pBuff);
        kfree(pu1Params);
        CAM_CALERR(" ioctl copy from user failed\n");
        return -EFAULT;
    }
    switch(a_u4Command)
    {
        case CAM_CALIOC_S_WRITE:
            CAM_CALDB("Write CMD \n");
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif
            i4RetValue = 0;//iWriteData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pu1Params);
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
            CAM_CALDB("Write data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif
            break;
        case CAM_CALIOC_G_READ:
            CAM_CALDB("[CAM_CAL] Read CMD \n");
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif
            i4RetValue = selective_read_region(ptempbuf->u4Offset, pu1Params, ptempbuf->u4Length);

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
            CAM_CALDB("Read data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif

            break;
        default :
      	     CAM_CALINF("[CAM_CAL] No CMD \n");
            i4RetValue = -EPERM;
        break;
    }

    if(_IOC_READ & _IOC_DIR(a_u4Command))
    {
        //copy data to user space buffer, keep other input paremeter unchange.
        if(copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pu1Params , ptempbuf->u4Length))
        {
            kfree(pBuff);
            kfree(pu1Params);
            CAM_CALERR("[CAM_CAL] ioctl copy to user failed\n");
            return -EFAULT;
        }
    }

    kfree(pBuff);
    kfree(pu1Params);
    return i4RetValue;
}


static u32 g_u4Opened = 0;
//#define
//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.
static int CAM_CAL_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    CAM_CALDB("CAM_CAL_Open\n");
    spin_lock(&g_CAM_CALLock);
    if(g_u4Opened)
    {
        spin_unlock(&g_CAM_CALLock);
		CAM_CALERR("Opened, return -EBUSY\n");
        return -EBUSY;
    }
    else
    {
        g_u4Opened = 1;
        atomic_set(&g_CAM_CALatomic,0);
    }
    spin_unlock(&g_CAM_CALLock);
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
#ifdef CONFIG_COMPAT
    .compat_ioctl = AR1335eeprom_Ioctl_Compat,
#endif
    .unlocked_ioctl = CAM_CAL_Ioctl
};

#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1

inline static int RegisterCAM_CALCharDrv(void)
{
    struct device* CAM_CAL_device = NULL;
    CAM_CALDB("RegisterCAM_CALCharDrv\n");
    
#if CAM_CAL_DYNAMIC_ALLOCATE_DEVNO
    if( alloc_chrdev_region(&g_CAM_CALdevno, 0, 1,CAM_CAL_DRVNAME) )
    {
        CAM_CALERR(" Allocate device no failed\n");

        return -EAGAIN;
    }
#else
    if( register_chrdev_region(  g_CAM_CALdevno , 1 , CAM_CAL_DRVNAME) )
    {
        CAM_CALERR(" Register device no failed\n");

        return -EAGAIN;
    }
#endif

    //Allocate driver
    g_pCAM_CAL_CharDrv = cdev_alloc();

    if(NULL == g_pCAM_CAL_CharDrv)
    {
        unregister_chrdev_region(g_CAM_CALdevno, 1);

        CAM_CALERR(" Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

    g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1))
    {
        CAM_CALERR(" Attatch file operation failed\n");

        unregister_chrdev_region(g_CAM_CALdevno, 1);

        return -EAGAIN;
    }

    CAM_CAL_class = class_create(THIS_MODULE, "AR1335_CAM_CALdrv");
    if (IS_ERR(CAM_CAL_class)) {
        int ret = PTR_ERR(CAM_CAL_class);
        CAM_CALERR("Unable to create class, err = %d\n", ret);
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

    return 0;
}

static int CAM_CAL_remove(struct platform_device *pdev)
{
    return 0;
}

// platform structure
static struct platform_driver g_stCAM_CAL_Driver = {
    .probe		= CAM_CAL_probe,
    .remove		= CAM_CAL_remove,
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

static int __init CAM_CAL_init(void)
{
    int i4RetValue = 0;
   //Register char driver
	i4RetValue = RegisterCAM_CALCharDrv();
    if(i4RetValue){
 	   CAM_CALDB(" register char device failed!\n");
	   return i4RetValue;
	}
	CAM_CALDB(" Attached!! \n");
    if(platform_driver_register(&g_stCAM_CAL_Driver)){
        CAM_CALERR("failed to register AR1335_eeprom driver\n");
        return -ENODEV;
    }
    if (platform_device_register(&g_stCAM_CAL_Device))
    {
        CAM_CALERR("failed to register AD1335_eeprom driver, 2nd time\n");
        return -ENODEV;
    }

    return 0;
}

static void __exit CAM_CAL_exit(void)
{
	platform_driver_unregister(&g_stCAM_CAL_Driver);
}

module_init(CAM_CAL_init);
module_exit(CAM_CAL_exit);

MODULE_DESCRIPTION("CAM_CAL driver");
MODULE_AUTHOR("Sean Lin <Sean.Lin@Mediatek.com>");
MODULE_LICENSE("GPL");


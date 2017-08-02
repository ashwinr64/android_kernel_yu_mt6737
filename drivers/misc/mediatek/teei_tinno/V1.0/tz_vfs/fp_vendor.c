#include "fp_vendor.h"
#include<linux/types.h>
#include "../tz_driver/include/nt_smc_call.h"
#include<linux/kernel.h>
#include <linux/mutex.h>


#define FPC_VENDOR_ID       0x01
#define ELAN_VENDOR_ID    0x02
#define GOODIX_VENDOR_ID    0x03


int fp_vendor_active = 0;
int fp_vendor = FP_VENDOR_INVALID;
static DEFINE_MUTEX(fp_vendor_lock);

int get_fp_vendor(void)
{
    uint64_t fp_vendor_id_64 = 0;
    uint32_t *p_temp = NULL;
    uint32_t fp_vendor_id_32 = 0;


    mutex_lock(&fp_vendor_lock);
    if(fp_vendor_active) {
        mutex_unlock(&fp_vendor_lock);
        return fp_vendor;
    }
#if 1
    get_t_device_id(&fp_vendor_id_32);
    p_temp = (uint32_t *)&fp_vendor_id_32;
    fp_vendor_id_32 = *p_temp;
#else
    get_t_device_id(&fp_vendor_id_64);
    printk("%s:%d fp_vendor_id_64 = 0x%llx\n", __func__, __LINE__,fp_vendor_id_64);
    p_temp = (uint32_t *)&fp_vendor_id_64;
    fp_vendor_id_32 = *p_temp;
    // fp_vendor_id_32 = (fp_vendor_id_32 >> 8) & 0xff;
#endif
    printk("%s:%d fp_vendor_id_32 = 0x%x\n", __func__, __LINE__, fp_vendor_id_32);

    switch(fp_vendor_id_32) {
        case FPC_VENDOR_ID:
            fp_vendor = FPC_VENDOR;
            break;
        case ELAN_VENDOR_ID:
            fp_vendor = ELAN_VENDOR;
            break;

        case GOODIX_VENDOR_ID:
            fp_vendor = GOODIX_VENDOR;
            break;

        default:
            fp_vendor = FP_VENDOR_INVALID;
            break;
    }
    fp_vendor_active = 1;

    //fp_vendor = FPC_VENDOR;
  //  fp_vendor = GOODIX_VENDOR;
    printk("%s:%d fp_vendor = 0x%x\n", __func__, __LINE__, fp_vendor);
    mutex_unlock(&fp_vendor_lock);
    return fp_vendor;
}

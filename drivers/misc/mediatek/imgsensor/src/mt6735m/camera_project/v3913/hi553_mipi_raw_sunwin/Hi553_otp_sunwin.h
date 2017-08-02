/*****************************************************************************
 *
 * Filename:
 * ---------
 *   catc24c16.h
 *
 * Project:
 * --------
 *   ALPS
 *
 * Description:
 * ------------
 *   Header file of CAM_CAL driver
 *
 *
 * Author:
 * -------
 *   John Wei (MTK07407)
 *
 *============================================================================*/
#ifndef __HI553_OTP_SUNWIN_H
#define __HI553_OTP_SUNWIN_H


/* CAM_CAL READ/WRITE ID */
#define Hi553_OTP_DEVICE_ID							0x50	//slave id of Hi-842

extern signed char check_MID_sunwin_hi553(bool bOnlyCheck);
extern bool otp_update_wb_sunwin_hi553(unsigned short golden_rg, unsigned short golden_bg) ;
extern bool otp_update_wb_sunwin_hi553(unsigned short golden_rg, unsigned short golden_bg) ;


#endif /* __HI553_OTP_SUNWIN_H */


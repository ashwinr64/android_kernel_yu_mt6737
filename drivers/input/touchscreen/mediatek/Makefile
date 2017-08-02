include $(srctree)/drivers/misc/mediatek/Makefile.custom
#subdir-ccflags-y += -Werror
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek
# In case the platform does NOT support this type of sensors

ifneq ($(CONFIG_TOUCHSCREEN_MTK),)
obj-y   +=  mtk_tpd.o tpd_button.o tpd_calibrate.o tpd_debug.o \
		tpd_default.o tpd_init.o tpd_misc.o tpd_setting.o
endif

obj-$(CONFIG_TOUCHSCREEN_MTK)   += tpd_common_probe.o

obj-$(CONFIG_TOUCHSCREEN_MTK)	+=  GT1151/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT1151TB)	+=  GT1151TB/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT910)	+=  GT910/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT9XXTB_HOTKNOT)	+=  GT9XXTB_hotknot/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT9XX_HOTKNOT)	+=  GT9XX_hotknot/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT9XX_HOTKNOT_PHONE)	+=  GT9XX_hotknot_phone/
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_INCELL_TD4100)	+=  synaptics_dsx_incell_td4100/
obj-$(CONFIG_TOUCHSCREEN_MTK_SYNAPTICS_I2C_RMI4)	+=  synaptics_i2c_rmi4/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT928)	+=  GT928/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT9XX_HOTKNOT_SCP)	+=  GT9XX_hotknot_scp/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT911)	+=  GT911/
obj-$(CONFIG_TOUCHSCREEN_MTK_FT5X0X)	+=  ft5x0x/
obj-$(CONFIG_TOUCHSCREEN_MTK_SYNAPTICS_3320_50) +=  synaptics_3320_50/
obj-$(CONFIG_TOUCHSCREEN_MTK_GT9271TB_HOTKNOT) +=  GT9271TB_hotknot/
obj-$(CONFIG_TOUCHSCREEN_UNIFIED_DRIVER_4)	+=  unified_driver_4/
obj-$(CONFIG_MTK_TGESTURE)	+=  tgesture/
obj-$(CONFIG_TOUCHSCREEN_MTK_FT5346) += ft5346/
obj-$(CONFIG_MTK_FT5346_V3961) += ft5346_v3961/
ifeq ($(CONFIG_TOUCHSCREEN_MTK_GT615),y)
obj-y	+=  gt615/
endif

ifeq ($(CONFIG_TINNO_FT6X06),y)
obj-y	+=  focaltech/
endif

ifeq ($(CONFIG_MTK_FT6X06),y)
obj-y	+=  ft6x06/
endif

ifeq ($(CONFIG_MTK_FT6636U),y)
obj-y	+=  ft6636u/
endif


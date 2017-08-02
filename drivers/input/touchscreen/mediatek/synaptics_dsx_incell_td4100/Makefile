# Linux driver folder
include $(srctree)/drivers/misc/mediatek/Makefile.custom

$(warning --[fyf]--ARCH_MTK_PROJECT---$(ARCH_MTK_PROJECT)----)
ccflags-y += -I$(srctree)/drivers/input/touchscreen/synaptics_dsx/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(PWD)/vendor/tinno/mdp/$(ARCH_MTK_PROJECT)/touchpanel/synaptics_dsx_incell_td4100/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
ccflags-y += -I$(srctree)/drivers/watchdog/mediatek/
ccflags-y += -I$(srctree)/drivers/watchdog/mediatek/wdt/
ccflags-y += -I$(srctree)/drivers/watchdog/mediatek/wdt/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/i2c/$(MTK_PLATFORM)/


obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_RMI_DEV) +=  synaptics_dsx_rmi_dev.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_TEST_REPORTING) +=  synaptics_dsx_test_reporting.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_I2C) +=  synaptics_dsx_i2c.o
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX_FW_UPDATE) +=  synaptics_dsx_fw_update.o


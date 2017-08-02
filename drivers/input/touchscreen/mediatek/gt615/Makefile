include $(srctree)/drivers/misc/mediatek/Makefile.custom

$(warning --[fyf]--ARCH_MTK_PROJECT---$(ARCH_MTK_PROJECT)----)

ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/gt615/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(PWD)/vendor/tinno/mdp/$(ARCH_MTK_PROJECT)/touchpanel/gt615/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/

obj-y	+=  gt9xx_driver.o
obj-y	+=  goodix_tool.o
obj-y	+=  gt9xx_update.o
obj-$(CONFIG_TP_AUTO_TEST)	+=  gt9xx_openshort.o


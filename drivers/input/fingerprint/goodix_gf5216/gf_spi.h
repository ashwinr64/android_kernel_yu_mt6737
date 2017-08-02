#ifndef __GF_SPI_H
#define __GF_SPI_H

#include <linux/types.h>
#include <linux/notifier.h>
#include <linux/pm.h>


#include "../../../spi/mediatek/mt6735/mt_spi.h"

/**********************************************************/
/**********************GF ops****************************/
#define GF_W                0xF0
#define GF_R                0xF1
#define GF_WDATA_OFFSET     (0x3)
#define GF_RDATA_OFFSET     (0x5)

typedef enum {
	GF_IMAGE_MODE = 0,
	GF_KEY_MODE,
	GF_SLEEP_MODE,
	GF_FF_MODE,
	GF_DEBUG_MODE = 0x56
}MODE;

struct gf_key {
	unsigned int key;
	int value;
};

struct gf_spi_transfer {
    unsigned char cmd;
    unsigned char reserve;
    unsigned short addr;
    unsigned int len;
    unsigned long long buf;
};

#define  GF_IOC_MAGIC         'g'
#define  GF_IOC_DISABLE_IRQ	_IO(GF_IOC_MAGIC, 0)
#define  GF_IOC_ENABLE_IRQ	_IO(GF_IOC_MAGIC, 1)
#define  GF_IOC_SETSPEED    _IOW(GF_IOC_MAGIC, 2, unsigned int)
#define  GF_IOC_RESET       _IO(GF_IOC_MAGIC, 3)
#define  GF_IOC_COOLBOOT    _IO(GF_IOC_MAGIC, 4)
#define  GF_IOC_SENDKEY    _IOW(GF_IOC_MAGIC, 5, struct gf_key)
#define  GF_IOC_CLK_READY  _IO(GF_IOC_MAGIC, 6)
#define  GF_IOC_CLK_UNREADY  _IO(GF_IOC_MAGIC, 7)
#define  GF_IOC_PM_FBCABCK  _IO(GF_IOC_MAGIC, 8)
#define  GF_IOC_SPI_TRANSFER  _IOWR(GF_IOC_MAGIC, 9, struct gf_spi_transfer)
//guomingyi add start.
#define  GF_IOC_FTM	_IOW(GF_IOC_MAGIC, 101, int)
#define  GF_IOC_SET_MODE	_IOW(GF_IOC_MAGIC, 102, int)
//guomingyi add end.
#define  GF_IOC_MAXNR    10

#define  GF_IOC_FTM	_IOW(GF_IOC_MAGIC, 101, int)
#define  GF_IOC_SET_MODE	_IOW(GF_IOC_MAGIC, 102, int)

#define  GF_IOC_RELEASEVERSION   _IOW(GF_IOC_MAGIC, 11, char *)

#define MTK_SPI_ALIGN_MASK_NUM  10
#define MTK_SPI_ALIGN_MASK  ((0x1 << MTK_SPI_ALIGN_MASK_NUM) - 1)


#define	GFX1XM_IMAGE_MODE       (0x00)
#define	GFX1XM_KEY_MODE	        (0x01)
#define GFX1XM_SLEEP_MODE       (0x02)
#define GFX1XM_FF_MODE          (0x03)
#define GFX1XM_DEBUG_MODE       (0x56)

/**************************debug******************************/
#define DEFAULT_DEBUG   (0)
#define SUSPEND_DEBUG   (1)
#define SPI_DEBUG       (2)
#define TIME_DEBUG      (3)
#define FLOW_DEBUG      (4)

#define GF_DEBUG
/*#undef  GF_DEBUG*/

#ifdef  GF_DEBUG
#define gf_print(fmt, args...) do { \
					printk(KERN_ERR"gf5216_spi:" fmt, ##args);\
		} while (0)

//#define gf_print(args...) printk(KERN_ERR "gf_spi:" ##args)
#define FUNC_ENTRY()  printk(KERN_ERR "gf5216_spi:%s, entry\n", __func__)
#define FUNC_EXIT()  printk(KERN_ERR "gf5216_spi:%s, exit\n", __func__)
#else
#define gf_print(fmt, args...)
#define FUNC_ENTRY()
#define FUNC_EXIT()
#endif


typedef enum {
	SPEED_500KHZ=0,
	SPEED_1MHZ,
	SPEED_2MHZ,
	SPEED_3MHZ,
	SPEED_4MHZ,
	SPEED_6MHZ,
	SPEED_8MHZ,
	SPEED_KEEP,
	SPEED_UNSUPPORTED
}SPI_SPEED;

#define GF_VDD_MIN_UV      2800000
#define GF_VDD_MAX_UV	   2800000
#define GF_VIO_MIN_UV      1800000
#define GF_VIO_MAX_UV      1800000

#define  USE_SPI_BUS	1
#define GF_FASYNC 		1//If support fasync mechanism.
struct gf_dev {
	dev_t devt;
    spinlock_t   spi_lock;
	struct list_head device_entry;
#if defined(USE_SPI_BUS)
	struct spi_device *spi;
#elif defined(USE_PLATFORM_BUS)
	struct platform_device *spi;
#endif
	struct clk *core_clk;
	struct clk *iface_clk;

	struct input_dev *input;
	/* buffer is NULL unless this device is open (users > 0) */
	unsigned users;
	signed irq_gpio;
	signed reset_gpio;
	signed pwr_gpio;
	int irq;
	int irq_enabled;
	int clk_enabled;
#ifdef GF_FASYNC
	struct fasync_struct *async;
#endif
	struct notifier_block notifier;
	char device_available;
	char fb_black;
    unsigned char *gBuffer;
    struct mutex buf_lock;

	unsigned int isPowerOn;
    struct regulator *avdd;
    struct pinctrl *pinctrl1;
    struct pinctrl_state *pins_default;
    struct pinctrl_state *eint_as_int, *eint_in_low, *eint_in_high, *eint_in_float, 
    	*fp_rst_low, *fp_rst_high,
    	*miso_pull_up,*miso_pull_disable;

};

extern unsigned int irq_of_parse_and_map(struct device_node *node, int index);
void gf_cleanup(struct gf_dev *gf_dev);
int gf_parse_dts(struct gf_dev* pdev);
int gf_power_init(struct gf_dev* pdev);
int gf_power_on(struct gf_dev *pdev);
int gf_power_off(struct gf_dev *pdev);
int gf_irq_num(struct gf_dev *pdev);
int gf_hw_reset(struct gf_dev *pdev, unsigned int delay_ms);
void gf_gpio_as_int(struct gf_dev *pdev);

static int gf_probe(struct spi_device *spi);
static int gf_remove(struct spi_device *spi);
static int gf_suspend(struct spi_device *spi, pm_message_t mesg);
static int gf_resume(struct spi_device *spi);
static void gf_miso_pullup(struct gf_dev *pdev, int enable);
static void gf_reset_output(struct gf_dev *pdev, int level);
static void gf_irq_pulldown(struct gf_dev *pdev, int enable);
static int gf_spi_write_byte(struct gf_dev *gf_dev, u16 addr, u8 value);
static int gf_spi_read_byte(struct gf_dev *gf_dev, u16 addr, u8 *value);
int gf_spi_read_word(struct gf_dev *gf_dev, u16 addr, u16 *value);
int gf_spi_write_word(struct gf_dev* gf_dev, u16 addr, u16 value);
static void gf_spi_set_mode(struct spi_device *spi, SPI_SPEED speed, int flag);


int gf_power_on(struct gf_dev *gf_dev);
int gf_power_off(struct gf_dev *gf_dev);

int gf_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms);
int gf_irq_num(struct gf_dev *gf_dev);

#endif /*__GF_SPI_H*/

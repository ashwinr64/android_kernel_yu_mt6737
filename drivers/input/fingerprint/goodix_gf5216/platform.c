#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>

#include "gf_spi.h"

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>


/**********************************************************************************/
static DEFINE_MUTEX(gf_set_gpio_mutex);

/**********************************************************************************/

int gf_parse_dts(struct gf_dev* pdev)
{
	int ret = -1;
	
    FUNC_ENTRY();

	if(pdev->spi->dev.of_node == NULL) {
        pdev->spi->dev.of_node = of_find_compatible_node(NULL, NULL, "mediatek,fingerprint");
	}
	
    if(pdev->spi->dev.of_node) {
		pdev->pinctrl1 = devm_pinctrl_get(&pdev->spi->dev);
		if (IS_ERR(pdev->pinctrl1)) {
			ret = PTR_ERR(pdev->pinctrl1);
			gf_print(" Cannot find fp pinctrl1!\n");
			return ret;
		}

    	pdev->fp_rst_high = pinctrl_lookup_state(pdev->pinctrl1, "fp_rst_high");
    	if (IS_ERR(pdev->fp_rst_high)) {
    		ret = PTR_ERR(pdev->fp_rst_high);
    		dev_err(&pdev->spi->dev, " Cannot find fp pinctrl fp_rst_high!\n");
    		return ret;
    	}
    	pdev->fp_rst_low = pinctrl_lookup_state(pdev->pinctrl1, "fp_rst_low");
    	if (IS_ERR(pdev->fp_rst_low)) {
    		ret = PTR_ERR(pdev->fp_rst_low);
    		dev_err(&pdev->spi->dev, " Cannot find fp pinctrl fp_rst_low!\n");
    		return ret;
    	}
    	pdev->eint_as_int = pinctrl_lookup_state(pdev->pinctrl1, "eint_as_int");
    	if (IS_ERR(pdev->eint_as_int)) {
    		ret = PTR_ERR(pdev->eint_as_int);
    		dev_err(&pdev->spi->dev, " Cannot find fp pinctrl eint_as_int!\n");
    		return ret;
    	}
    	pdev->eint_in_low = pinctrl_lookup_state(pdev->pinctrl1, "eint_in_low");
    	if (IS_ERR(pdev->eint_in_low)) {
    		ret = PTR_ERR(pdev->eint_in_low);
    		dev_err(&pdev->spi->dev, " Cannot find fp pinctrl eint_output_low!\n");
    		return ret;
    	}
    	pdev->eint_in_high= pinctrl_lookup_state(pdev->pinctrl1, "eint_in_high");
    	if (IS_ERR(pdev->eint_in_high)) {
    		ret = PTR_ERR(pdev->eint_in_high);
    		dev_err(&pdev->spi->dev, " Cannot find fp pinctrl eint_in_high!\n");
    		return ret;
    	}
    
    	pdev->eint_in_float = pinctrl_lookup_state(pdev->pinctrl1, "eint_in_float");
    	if (IS_ERR(pdev->eint_in_float)) {
    		ret = PTR_ERR(pdev->eint_in_float);
    		dev_err(&pdev->spi->dev, " Cannot find fp pinctrl eint_output_high!\n");
    		return ret;
    	}
    		pdev->miso_pull_up = pinctrl_lookup_state(pdev->pinctrl1, "miso_pull_up");
    	if (IS_ERR(pdev->miso_pull_up)) {
    		ret = PTR_ERR(pdev->miso_pull_up);
    		dev_err(&pdev->spi->dev, " Cannot find fp pinctrl miso_pull_up!\n");
    		return ret;
    	}
    	pdev->miso_pull_disable = pinctrl_lookup_state(pdev->pinctrl1, "miso_pull_disable");
    	if (IS_ERR(pdev->miso_pull_disable)) {
    		ret = PTR_ERR(pdev->miso_pull_disable);
    		dev_err(&pdev->spi->dev, " Cannot find fp pinctrl miso_pull_disable!\n");
    		return ret;
    	}
    }
    else {
       gf_print(" pdev->spi->dev.of_node is null!\n");
       return ret;
    }
    return 0;
}

static int gf_power_ctl(struct gf_dev* pdev, int on) 
{
    int rc = 0;

	FUNC_ENTRY();
    if((on && !pdev->isPowerOn) || (!on && pdev->isPowerOn)) {
		if(on) 
		{
			gf_reset_output(pdev,0);
			gf_irq_pulldown(pdev,0);
			
			rc = regulator_enable(pdev->avdd); 
			if (rc) { 
				gf_print("regulator_enable() failed!\n"); 
				return rc;
			}
			udelay(400);
			
			gf_reset_output(pdev,1);
			udelay(100);
			
			gf_irq_pulldown(pdev,1);
			msleep(20);
			
			gf_irq_pulldown(pdev,0);
			msleep(10);
			
			gf_irq_pulldown(pdev,-1);
			msleep(1);
			gf_reset_output(pdev,1);
			msleep(60);
			pdev->isPowerOn = 1;
		} 
		else 
		{
			gf_irq_pulldown(pdev,1);
			gf_reset_output(pdev,0);
			msleep(10);
			rc = regulator_disable(pdev->avdd);	
			if (rc) {
				gf_print("regulator_disable() failed!\n");
				return rc;
			}
			msleep(50);
			pdev->isPowerOn = 0;
		}
    }
    else {
    	gf_print("Ignore %d to %d\n", on, pdev->isPowerOn);
    }
    return rc;
}

void gf_cleanup(struct gf_dev * pdev) { }

int gf_power_init(struct gf_dev* pdev) 
{
    int ret = 0;

    FUNC_ENTRY();
    pdev->avdd = regulator_get(&pdev->spi->dev, "vfp");
    if(IS_ERR(pdev->avdd)) {
    	ret = PTR_ERR(pdev->avdd);
    	gf_print("Regulator get failed vdd ret=%d\n", ret);
    	return ret;
    }
	
    ret = regulator_set_voltage(pdev->avdd, GF_VDD_MIN_UV, GF_VDD_MAX_UV);
    if (ret) {
    	gf_print("regulator_set_voltage(%d) failed!\n", ret);
    	goto reg_vdd_put;
    }
	return 0;

reg_vdd_put:
	
	regulator_put(pdev->avdd);
	return ret;
}


int gf_power_on(struct gf_dev *pdev)
{
	int rc;

    FUNC_ENTRY();
	
	rc = gf_power_ctl(pdev, 1);
    msleep(10);
    return rc;
}

int gf_power_off(struct gf_dev *pdev)
{
   FUNC_ENTRY();
   
   return gf_power_ctl(pdev, 0);
}


int gf_hw_reset(struct gf_dev *pdev, unsigned int delay_ms)
{	
    gf_miso_pullup(pdev, 1);
    gf_reset_output(pdev, 0);
    msleep(5); 
    gf_reset_output(pdev, 1);
    if(delay_ms){
        msleep(delay_ms);
    }
    
    gf_miso_pullup(pdev, 0);
    return 0;
}

int gf_irq_num(struct gf_dev *pdev)
{
    int ret;
	u32 ints[2] = {0, 0};
    struct device_node *node = pdev->spi->dev.of_node;
	
	FUNC_ENTRY();
	
	if(node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_request(ints[0], "fingerprint-irq");
		gpio_set_debounce(ints[0], ints[1]);
		gf_print("gpio_set_debounce ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

		pdev->spi->irq = irq_of_parse_and_map(node, 0);

		gf_print(" %s: irq num = %d\n", __func__, pdev->spi->irq);
		if(! pdev->spi->irq) {
			gf_print("%s: gf_irq_num fail!!\n",  __func__);
			return -1;
		}
	}
	else {
		gf_print("%s: null node!!\n",__func__);
		return -1;
	}
    return 0;
}


void gf_gpio_as_int(struct gf_dev *pdev)
{
	mutex_lock(&gf_set_gpio_mutex);
	pinctrl_select_state(pdev->pinctrl1, pdev->eint_as_int);
	mutex_unlock(&gf_set_gpio_mutex);
}

static void gf_reset_output(struct gf_dev *pdev, int level)
{
	mutex_lock(&gf_set_gpio_mutex);
	//gf_print("%s: level = %d\n", __func__,level);

	if (level)
		pinctrl_select_state(pdev->pinctrl1, pdev->fp_rst_high);
	else
		pinctrl_select_state(pdev->pinctrl1, pdev->fp_rst_low);
	mutex_unlock(&gf_set_gpio_mutex);
}

static void gf_irq_pulldown(struct gf_dev *pdev, int enable)
{
	mutex_lock(&gf_set_gpio_mutex);
	//gf_print("%s: enable = %d\n", __func__,enable);
	
	if (enable == 1) {
		pinctrl_select_state(pdev->pinctrl1, pdev->eint_in_low);
	}
	else if(enable == 0) {
		pinctrl_select_state(pdev->pinctrl1, pdev->eint_in_high);
	}
	else if(enable == -1) {
		pinctrl_select_state(pdev->pinctrl1, pdev->eint_in_float);
	}
	mutex_unlock(&gf_set_gpio_mutex);
}

static void gf_miso_pullup(struct gf_dev *pdev, int enable)
{
	mutex_lock(&gf_set_gpio_mutex);
	//gf_print("%s: enable = %d\n", __func__,enable);

	if (enable)
		pinctrl_select_state(pdev->pinctrl1, pdev->miso_pull_up);
	else
		pinctrl_select_state(pdev->pinctrl1, pdev->miso_pull_disable);
	mutex_unlock(&gf_set_gpio_mutex);
}



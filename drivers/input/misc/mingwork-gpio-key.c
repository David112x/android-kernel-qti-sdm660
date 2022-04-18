/* Copyright (c) 2012-2017, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>

#include <linux/interrupt.h>

#include <linux/kernel.h>
#include <linux/err.h>

#include <linux/input.h>

#include <linux/gpio.h>
#include <linux/of_gpio.h>

#define DRV_NAME	"mw-gpio-key"
#define DRV_VERSION	"1.00"


struct input_dev *virt_key_dev;

static int led_pin;	// default the led_pin value is 0(white color)
static int led_pin_volume;
static int led_pin_mute_red;
static int led_pin_mute_white;
static int invert_value = 0;	// default white color

void mw_revert_led_status(void)
{
#if 0
	invert_value = !invert_value;	// toggle the led status
	printk("sdssgl %s.%d,pin status is %d.\n",__func__,__LINE__,invert_value);

	gpio_direction_output(led_pin,invert_value);
	gpio_direction_output(led_pin_volume,!invert_value);
	gpio_direction_output(led_pin_mute_white,!invert_value);
	gpio_direction_output(led_pin_mute_red,invert_value);
#endif

}
EXPORT_SYMBOL(mw_revert_led_status);

int mw_get_led_status(void)
{
    return invert_value;
}
EXPORT_SYMBOL(mw_get_led_status);

void mw_set_led_status(int status)
{
	invert_value = status;	// toggle the led status
	printk("sdssgl %s.%d,pin status is %d.\n",__func__,__LINE__,invert_value);

	gpio_direction_output(led_pin,invert_value);
	gpio_direction_output(led_pin_volume,!invert_value);
	gpio_direction_output(led_pin_mute_white,!invert_value);
	gpio_direction_output(led_pin_mute_red,invert_value);

}
EXPORT_SYMBOL(mw_set_led_status);

static irqreturn_t mw_gpio_key_irq_handler(int irq, void *data)
{

	//if(invert_value)
	//{
		input_report_key(virt_key_dev,KEY_MICMUTE,1);
		input_sync(virt_key_dev);
		input_report_key(virt_key_dev,KEY_MICMUTE,0);
		input_sync(virt_key_dev);
	//}else{
	//	input_report_key(virt_key_dev,KEY_MICMUTE,1);
	//	input_sync(virt_key_dev);
	//	input_report_key(virt_key_dev,KEY_MICMUTE,0);
	//	input_sync(virt_key_dev);
		
	//}

    mw_revert_led_status();

	return IRQ_HANDLED;
}

static int gpio_key_probe(struct platform_device *pdev)
{

	struct device_node *np = pdev->dev.of_node;
	int irq_pin;	
	int ret = 0;
	int error = -1;

	irq_pin = of_get_named_gpio(np,"mw,irq-pin",0);
	printk("sdssgl %s.%d,irq_pin is %d.\n",__func__,__LINE__,irq_pin);

	if ((!gpio_is_valid(irq_pin))){
		ret = -EINVAL;
		dev_err(&pdev->dev,"sdssgl %s(%d),irq_pin is invalid,ret is %d.\n",__func__,__LINE__,ret);
		// return ret;
	}else{
		irq_pin = gpio_to_irq(irq_pin);
		if(irq_pin < 0){
			dev_err(&pdev->dev,"sdssgl %s: unable to get gpio_to_irq for touch irq gpio [%d]\n",__func__,irq_pin);

		}

	}

	led_pin = of_get_named_gpio(np,"mw,led-pin",0);
	printk("sdssgl %s.%d,led_pin is %d.\n",__func__,__LINE__,led_pin);

	if ((!gpio_is_valid(led_pin))){
		ret = -EINVAL;
		dev_err(&pdev->dev,"sdssgl %s(%d),led_pin is invalid,ret is %d.\n",__func__,__LINE__,ret);
		// return ret;
	}else{
		error = gpio_request(led_pin,"mw_led_pin");
		if(error){
			dev_err(&pdev->dev,"sdssgl %s: unable to request led pin [%d]\n",__func__,led_pin);

		}

	}

	led_pin_volume = of_get_named_gpio(np,"mw,led-pin-volume",0);
	printk("sdssgl %s.%d,led_pin_volume is %d.\n",__func__,__LINE__,led_pin_volume);

	if ((!gpio_is_valid(led_pin_volume))){
		ret = -EINVAL;
		dev_err(&pdev->dev,"sdssgl %s(%d),led_pin_volume is invalid,ret is %d.\n",__func__,__LINE__,ret);
		// return ret;
	}else{
		error = gpio_request(led_pin_volume,"mw_led_pin_volume");
		if(error){
			dev_err(&pdev->dev,"sdssgl %s: unable to request led pin [%d]\n",__func__,led_pin_volume);

		}
	}

	led_pin_mute_white = of_get_named_gpio(np,"mw,led-pin-mute-white",0);
	printk("sdssgl %s.%d,led_pin_mute_white is %d.\n",__func__,__LINE__,led_pin_mute_white);

	if ((!gpio_is_valid(led_pin_mute_white))){
		ret = -EINVAL;
		dev_err(&pdev->dev,"sdssgl %s(%d),led_pin_mute_white is invalid,ret is %d.\n",__func__,__LINE__,ret);
		// return ret;
	}else{
		error = gpio_request(led_pin_mute_white,"mw_led_pin_mute_white");
		if(error){
			dev_err(&pdev->dev,"sdssgl %s: unable to request led pin [%d]\n",__func__,led_pin_mute_white);

		}


	}

	led_pin_mute_red = of_get_named_gpio(np,"mw,led-pin-mute-red",0);
	printk("sdssgl %s.%d,led_pin_mute_red is %d.\n",__func__,__LINE__,led_pin_mute_red);

	if ((!gpio_is_valid(led_pin_mute_red))){
		ret = -EINVAL;
		dev_err(&pdev->dev,"sdssgl %s(%d),led_pin_mute_red is invalid,ret is %d.\n",__func__,__LINE__,ret);
		// return ret;
	}else{
		error = gpio_request(led_pin_mute_red,"mw_led_pin_mute_red");
		if(error){
			dev_err(&pdev->dev,"sdssgl %s: unable to request led pin [%d]\n",__func__,led_pin_mute_red);

		}

	}

    /* defaut is white color */
	gpio_direction_output(led_pin_volume,0);
	gpio_direction_output(led_pin_mute_white,1);
	gpio_direction_output(led_pin_mute_red,0);

	ret = request_irq(irq_pin,mw_gpio_key_irq_handler,IRQF_TRIGGER_RISING,pdev->name,NULL);
	if(ret != 0)
	{
		dev_err(&pdev->dev,"sdssgl request_irq_failed.\n");
	}

	virt_key_dev = input_allocate_device();
	if(virt_key_dev == NULL){
		printk("sdssgl Can not allocate memory for virt key input device.\n");
		ret = ENOMEM;
	}

	virt_key_dev->name = "virt_mute/unmute_key";
	virt_key_dev->dev.parent = &pdev->dev;
	set_bit(EV_KEY,virt_key_dev->evbit);
	set_bit(KEY_MICMUTE,virt_key_dev->keybit);
	set_bit(KEY_MICMUTE,virt_key_dev->keybit);
		
	ret = input_register_device(virt_key_dev);
	if(ret < 0){
		printk("sdssgl Can not register virt key input device.\n");
	}

	printk("sdssgl %s.%d,the end.\n",__func__,__LINE__);
	return 0;

}

static int gpio_key_remove(struct platform_device *pdev)
{

	return 0;
}

static const struct of_device_id mw_gpio_key_dt_match[] = {
	{.compatible = "mw,gpio-key"},
	{},
};
MODULE_DEVICE_TABLE(of, mw_gpio_key_dt_match);

static struct platform_driver gpio_key_driver = {
	.driver = {
		.name = "mw-gpio-key",
		.owner = THIS_MODULE,
		.of_match_table = mw_gpio_key_dt_match,
	},
	.probe = gpio_key_probe,
	.remove = gpio_key_remove,
};

static int __init gpio_key_init(void)
{
	int rc;

	printk("sdssgl %s.%d,driver name is %s, version is %s.\n",__func__,__LINE__,DRV_NAME,DRV_VERSION);
	rc = platform_driver_register(&gpio_key_driver);
	if (rc) {
		pr_err("%s: Failed to register mw-gpio-key driver\n",
			__func__);
		return rc;
	}

	return 0;
}

static void __exit gpio_key_exit(void)
{
	platform_driver_unregister(&gpio_key_driver);
}

module_init(gpio_key_init);
module_exit(gpio_key_exit);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("GuoliangHan <han.guoliang@mingwork.com>");
MODULE_DESCRIPTION("MingWork gpio key driver");

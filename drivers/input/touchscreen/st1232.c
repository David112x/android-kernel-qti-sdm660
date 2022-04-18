/*
 * ST1232 Touchscreen Controller Driver
 *
 * Copyright (C) 2010 Renesas Solutions Corp.
 *	Tony SIM <chinyeow.sim.xt@renesas.com>
 *
 * Using code from:
 *  - android.git.kernel.org: projects/kernel/common.git: synaptics_i2c_rmi.c
 *	Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pm_qos.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/platform_data/st1232_pdata.h>

#define ST1232_TS_NAME	"st1232-ts"

#define MIN_X		0x00
#define MIN_Y		0x00
#define MAX_X		0x437	/* (1080 - 1) */
#define MAX_Y		0x77f	/* (1920 - 1) */
#define MAX_AREA	0xff
#define MAX_FINGERS	5

struct st1232_ts_finger {
	u16 x;
	u16 y;
	u8 t;
	bool is_valid;
};

struct st1232_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct st1232_ts_finger finger[MAX_FINGERS];
	struct dev_pm_qos_request low_latency_req;
	int reset_gpio;
};


static int st1232_ts_read_id(struct st1232_ts_data *ts)
{
	struct st1232_ts_finger *finger = ts->finger;
	struct i2c_client *client = ts->client;
	struct i2c_msg msg[2];
	int error;
	u8 start_reg;
	u8 buf[8];

	/* read touchscreen data from ST1232 */
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &start_reg;
	start_reg = 0x1;

	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = sizeof(buf);
	msg[1].buf = buf;

	error = i2c_transfer(client->adapter, msg, 2);
	if (error < 0)
		return error;

	pr_notice("ldx st1232:buf[0]:%x buf[1]:%d\n", buf[0], buf[1]);
	return 0;
}
static int st1232_ts_read_data(struct st1232_ts_data *ts)
{
	struct st1232_ts_finger *finger = ts->finger;
	struct i2c_client *client = ts->client;
	struct i2c_msg msg[2];
	int error;
	u8 start_reg;
	u8 buf[20];

	/* read touchscreen data from ST1232 */
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &start_reg;
	start_reg = 0x12;

	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = sizeof(buf);
	msg[1].buf = buf;

	error = i2c_transfer(client->adapter, msg, 2);
	if (error < 0)
		return error;

	/* get "valid" bits */
	finger[0].is_valid = buf[0] >> 7;
	finger[1].is_valid = buf[4] >> 7;
	finger[2].is_valid = buf[8] >> 7;
	finger[3].is_valid = buf[12] >> 7;
	finger[4].is_valid = buf[16] >> 7;
	/* get xy coordinate */
	if (finger[0].is_valid) {
		finger[0].x = ((buf[0] & 0x0070) << 4) | buf[1];
		finger[0].y = ((buf[0] & 0x0007) << 8) | buf[2];
		finger[0].t = buf[6];
	}

	if (finger[1].is_valid) {
		finger[1].x = ((buf[4] & 0x0070) << 4) | buf[5];
		finger[1].y = ((buf[4] & 0x0007) << 8) | buf[6];
		finger[1].t = buf[7];
	}
	
		if (finger[2].is_valid) {
		finger[2].x = ((buf[8] & 0x0070) << 4) | buf[9];
		finger[2].y = ((buf[8] & 0x0007) << 8) | buf[10];
		finger[2].t = 100;//buf[8];
	}
	
		if (finger[3].is_valid) {
		finger[3].x = ((buf[12] & 0x0070) << 4) | buf[13];
		finger[3].y = ((buf[12] & 0x0007) << 8) | buf[14];
		finger[3].t = 100;//buf[9];
	}
	
		if (finger[4].is_valid) {
		finger[4].x = ((buf[16] & 0x0070) << 4) | buf[17];
		finger[4].y = ((buf[16] & 0x0007) << 8) | buf[18];
		finger[4].t = 100;//buf[10];
	}

	return 0;
}
#define SWAP_XY
static irqreturn_t st1232_ts_irq_handler(int irq, void *dev_id)
{
	struct st1232_ts_data *ts = dev_id;
	struct st1232_ts_finger *finger = ts->finger;
	struct input_dev *input_dev = ts->input_dev;
	int count = 0;
	int i, ret;
	int temp;

	ret = st1232_ts_read_data(ts);
	if (ret < 0)
		goto end;

	/* multi touch protocol */
	for (i = 0; i < MAX_FINGERS; i++) {
		if (!finger[i].is_valid)
			continue;

		//pr_notice("ldx tp coordinate: finger[%d], x[%d],y[%d]\n",i, finger[i].x, finger[i].y);
		#ifdef SWAP_XY
		temp = finger[i].y;
		finger[i].y = finger[i].x;//*1080/1920;
		finger[i].x = (1079 - temp);//*1920/1080;

		//pr_notice("ldx after tp coordinate: finger[%d], x[%d],y[%d]\n",i, finger[i].x, finger[i].y);
		#endif
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, finger[i].t);
		input_report_abs(input_dev, ABS_MT_POSITION_X, finger[i].x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, finger[i].y);
		input_mt_sync(input_dev);
		
		count++;
	}

	/* SYN_MT_REPORT only if no contact */
	if (!count) {
		input_mt_sync(input_dev);
		if (ts->low_latency_req.dev) {
			dev_pm_qos_remove_request(&ts->low_latency_req);
			ts->low_latency_req.dev = NULL;
		}
	} else if (!ts->low_latency_req.dev) {
		/* First contact, request 100 us latency. */
		dev_pm_qos_add_ancestor_request(&ts->client->dev,
						&ts->low_latency_req,
						DEV_PM_QOS_RESUME_LATENCY, 100);
	}

	/* SYN_REPORT */
	input_sync(input_dev);

end:
	return IRQ_HANDLED;
}

static void st1232_ts_power(struct st1232_ts_data *ts, bool poweron)
{
	if (gpio_is_valid(ts->reset_gpio))
		gpio_direction_output(ts->reset_gpio, poweron);
}

static int st1232_ts_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct st1232_ts_data *ts;
	struct st1232_pdata *pdata;// = dev_get_platdata(&client->dev);
	struct input_dev *input_dev;
	struct device_node *np = client->dev.of_node;
	int error = 0;
	int irqn = 0;
	const char *desc = "st1232_touch";

	//pr_notice("ldx debug %s line:%d\n", __func__, __LINE__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "need I2C_FUNC_I2C\n");
		return -EIO;
	}
	//pr_notice("ldx debug %s line:%d\n", __func__, __LINE__);
	pdata = devm_kzalloc(&client->dev,
		sizeof(struct st1232_pdata), GFP_KERNEL);

	pdata->reset_gpio = of_get_named_gpio(np, "st,reset-gpio", 0);
	if ((!gpio_is_valid(pdata->reset_gpio)))
		return -EINVAL;
	//pr_notice("ldx debug %s line:%d\n", __func__, __LINE__);
	pdata->irq_gpio = of_get_named_gpio(np, "st,irq-gpio", 0);
	if ((!gpio_is_valid(pdata->irq_gpio)))
		return -EINVAL;
	//pr_notice("ldx debug %s line:%d\n", __func__, __LINE__);
	if (gpio_is_valid(pdata->reset_gpio)) {
		error = gpio_request(pdata->reset_gpio, "touch_reset_gpio");
		if (error) {
			dev_err(&client->dev, "%s: unable to request touch reset gpio [%d]\n",
				__func__, pdata->irq_gpio);
		//	goto err_en_gpio;
		}

		gpio_direction_output(pdata->reset_gpio, 1);
		mdelay(1);
		gpio_direction_output(pdata->reset_gpio, 0);
		mdelay(5);

		gpio_direction_output(pdata->reset_gpio, 1);
	}
	//pr_notice("ldx debug %s line:%d\n", __func__, __LINE__);
	if (gpio_is_valid(pdata->irq_gpio)) {
		/*error = gpio_request(pdata->irq_gpio, "touch_irq_gpio");
		if (error) {
			dev_err(&client->dev, "%s: unable to request touch irq gpio [%d]\n",
				__func__, pdata->irq_gpio);
		//	goto err_en_gpio;
		}
		error = gpio_direction_input(pdata->irq_gpio);
		if (error) {
			dev_err(&client->dev,
			"%s: unable to set direction for nfc irq gpio [%d]\n",
				__func__,
				pdata->irq_gpio);
			//goto err_irq_gpio;
		}*/

		irqn = gpio_to_irq(pdata->irq_gpio);
		if (irqn < 0) {
			error = irqn;
			dev_err(&client->dev,
			"%s: unable to get gpio_to_irq for touch irq gpio [%d]\n",
				__func__,
				pdata->irq_gpio);
			//goto err_irq_gpio;
		}
		client->irq = irqn;
	} else {
		dev_err(&client->dev, "%s: irq gpio not provided\n", __func__);
	}

	//pr_notice("ldx debug %s line:%d\n", __func__, __LINE__);
	if (!client->irq) {
		dev_err(&client->dev, "no IRQ?\n");
		return -EINVAL;
	}

	error = devm_gpio_request_one(&client->dev, pdata->irq_gpio,
		GPIOF_IN, desc);
	if (error < 0) {
		pr_err("failed to request gpio %d, error %d\n",
			client->irq, error);
	}

	//pr_notice("ldx debug %s line:%d\n", __func__, __LINE__);
	ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	//pr_notice("ldx debug %s line:%d\n", __func__, __LINE__);
	input_dev = devm_input_allocate_device(&client->dev);
	if (!input_dev)
		return -ENOMEM;

	//pr_notice("ldx debug %s line:%d\n", __func__, __LINE__);
	ts->client = client;
	ts->input_dev = input_dev;

	if (pdata)
		ts->reset_gpio = pdata->reset_gpio;
	else
		ts->reset_gpio = -ENODEV;

	/*if (gpio_is_valid(ts->reset_gpio)) {
		error = devm_gpio_request(&client->dev, ts->reset_gpio, NULL);
		if (error) {
			dev_err(&client->dev,
				"Unable to request GPIO pin %d.\n",
				ts->reset_gpio);
				return error;
		}
	}*/
	//pr_notice("ldx debug %s line:%d reset_gpio:%d irq_gpio:%d\n", __func__, __LINE__, pdata->reset_gpio, pdata->irq_gpio);
	st1232_ts_power(ts, true);

	
	//while(1)
	//{

	st1232_ts_read_id(ts);
	mdelay(20);
	//}
	input_dev->name = "st1232-touchscreen";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);

	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, MIN_X, MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, MIN_Y, MAX_Y, 0, 0);

	error = devm_request_threaded_irq(&client->dev, client->irq,
					  NULL, st1232_ts_irq_handler,
					  IRQF_ONESHOT| IRQF_TRIGGER_RISING,
					  client->name, ts);
	if (error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		return error;
	}
	pr_notice("ldx debug %s line:%d\n", __func__, __LINE__);
	error = input_register_device(ts->input_dev);
	if (error) {
		dev_err(&client->dev, "Unable to register %s input device\n",
			input_dev->name);
		return error;
	}
	pr_notice("ldx debug %s line:%d\n", __func__, __LINE__);
	i2c_set_clientdata(client, ts);
	device_init_wakeup(&client->dev, 1);

	return 0;
}

static int st1232_ts_remove(struct i2c_client *client)
{
	struct st1232_ts_data *ts = i2c_get_clientdata(client);

	st1232_ts_power(ts, false);

	return 0;
}

static int __maybe_unused st1232_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1232_ts_data *ts = i2c_get_clientdata(client);

	if (device_may_wakeup(&client->dev)) {
		enable_irq_wake(client->irq);
	} else {
		disable_irq(client->irq);
		st1232_ts_power(ts, false);
	}

	return 0;
}

static int __maybe_unused st1232_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct st1232_ts_data *ts = i2c_get_clientdata(client);

	if (device_may_wakeup(&client->dev)) {
		disable_irq_wake(client->irq);
	} else {
		st1232_ts_power(ts, true);
		enable_irq(client->irq);
	}

	return 0;
}

static SIMPLE_DEV_PM_OPS(st1232_ts_pm_ops,
			 st1232_ts_suspend, st1232_ts_resume);

static const struct i2c_device_id st1232_ts_id[] = {
	{ ST1232_TS_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, st1232_ts_id);

#ifdef CONFIG_OF
static const struct of_device_id st1232_ts_dt_ids[] = {
	{ .compatible = "sitronix,st1232", },
	{ }
};
MODULE_DEVICE_TABLE(of, st1232_ts_dt_ids);
#endif

static struct i2c_driver st1232_ts_driver = {
	.probe		= st1232_ts_probe,
	.remove		= st1232_ts_remove,
	.id_table	= st1232_ts_id,
	.driver = {
		.name	= ST1232_TS_NAME,
		.of_match_table = of_match_ptr(st1232_ts_dt_ids),
		.pm	= &st1232_ts_pm_ops,
	},
};

module_i2c_driver(st1232_ts_driver);

MODULE_AUTHOR("Tony SIM <chinyeow.sim.xt@renesas.com>");
MODULE_DESCRIPTION("SITRONIX ST1232 Touchscreen Controller Driver");
MODULE_LICENSE("GPL");

#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>

unsigned int g_charger_state = 0 ;
unsigned int    xmos_state = 1;
unsigned int    xmos_pwdn_state = 1;//0 means enabled

//factory
#ifdef CONFIG_WIND_FACTORY_CYCLE_TEST
unsigned int    mode_numer = 0;
unsigned int    factory_value = 0;
#endif 


#define AGING_POWER_TEST_PROC_FOLDER "gpio_set"
#define AGING_POWER_TEST_PROC_CHARGING_CHARGESTATE "usb3.0_enable"
#define AGING_POWER_TEST_PROC_CHARGING_XMOS "xmos_set"
#define AGING_POWER_TEST_PROC_CHARGING_XMOS_PWDN "xmos_down"
//factory
#ifdef CONFIG_WIND_FACTORY_CYCLE_TEST
#define AGING_POWER_TEST_PROC_FACTORY_GPIO_SET "factory_gpio_set"
#define AGING_POWER_TEST_PROC_FACTORY_GPIO_GET "factory_gpio_get"
#define AGING_POWER_TEST_PROC_FACTORY_VALUE "factory_value"
#endif

#define FIVE_V_VBUS_GPIO		29
#define CC1_HOST_GPIO		32
#define HUB_SWITCH_GPIO		50
#define VDP3_ENABLE_GPIO	28
#define XMOS_CONNECT_USB_GPIO 2
#define XMOS_AMP_PDN_GPIO 51

#define LCD_KEY_LED_VOLUME_WHITE_GPIO 68
#define LCD_KEY_LED_MUTE_WHITE_GPIO 69
#define LCD_KEY_LED_MUTE_RED_GPIO 21

static struct proc_dir_entry *aging_power_test_proc_dir = NULL;
static struct proc_dir_entry *proc_Charging_ChargeState_file = NULL;
static struct proc_dir_entry *proc_xmos_set_file = NULL;
static struct proc_dir_entry *proc_xmos_pwdn_file = NULL;


static int Charging_ChargeState_show(struct seq_file *m, void *v)
{
	
	 seq_printf(m, "%u\n", g_charger_state);
	 return 0;
}
static int Charging_ChargeState_read(struct inode *inode, struct file *file)
{
	return single_open(file, Charging_ChargeState_show, NULL);
}

static ssize_t Charging_ChargeState_write(struct file *file, const char __user *buffer,
			     size_t count, loff_t *ppos)
{
	unsigned long temp = 0; 
	int err = kstrtoul_from_user(buffer, count, 0, &temp);
	if (err)
		return err;
	g_charger_state = (unsigned int)temp;
	printk("usb3.0_enable:%d",g_charger_state);

	if(1){
    gpio_direction_output(VDP3_ENABLE_GPIO,!g_charger_state);//step2: hub reset
	
	gpio_direction_output(FIVE_V_VBUS_GPIO,g_charger_state);//step2: vbus
	mdelay(100);
    gpio_direction_output(CC1_HOST_GPIO,g_charger_state);//step2: cc1
    //mdelay(100); 
	gpio_direction_output(HUB_SWITCH_GPIO,g_charger_state);//step2: data
	//mdelay(100);
	}
	else{
		gpio_direction_output(VDP3_ENABLE_GPIO,!g_charger_state);//step2: hub reset
		
		
		mdelay(100);
		gpio_direction_output(CC1_HOST_GPIO,g_charger_state);//step2: cc1
		//mdelay(100); 
		gpio_direction_output(HUB_SWITCH_GPIO,g_charger_state);//step2: data
        mdelay(100);
		gpio_direction_output(FIVE_V_VBUS_GPIO,g_charger_state);//step2: vbus
	 

		}
	
	
	

	return count;
}

static const struct file_operations proc_Charging_ChargeState_file_ops = {
	.open		= Charging_ChargeState_read,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.write 		= Charging_ChargeState_write,
	.release	= single_release,
};

//xmos_set write open
static int xmos_set_show(struct seq_file *m, void *v)
{
	
	 seq_printf(m, "%u\n", xmos_state);
	 return 0;
}
static int xmos_set_read(struct inode *inode, struct file *file)
{
	return single_open(file, xmos_set_show, NULL);
}
static ssize_t xmos_set_write(struct file *file, const char __user *buffer,
			     size_t count, loff_t *ppos)
{
	unsigned long temp = 0; 
	int err = kstrtoul_from_user(buffer, count, 0, &temp);
	if (err)
		return err;
	xmos_state = (unsigned int)temp;
	printk("wind xmos set");
	gpio_direction_output(XMOS_CONNECT_USB_GPIO,xmos_state);
	//gpio_set_value(XMOS_CONNECT_USB_GPIO,xmos_state);
	
	return count;
}
//xmos
static const struct file_operations proc_xmos_set_file_ops = {
	.open		= xmos_set_read,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.write 		= xmos_set_write,
	.release	= single_release,
};


//xmos pwdn open
static int xmos_pwdn_show(struct seq_file *m, void *v)
{
	
	 seq_printf(m, "%u\n", xmos_pwdn_state);
	 return 0;
}
static int xmos_pwdn_read(struct inode *inode, struct file *file)
{
	return single_open(file, xmos_pwdn_show, NULL);
}
static ssize_t xmos_pwdn_write(struct file *file, const char __user *buffer,
			     size_t count, loff_t *ppos)
{
	unsigned long temp = 0; 
	int err = kstrtoul_from_user(buffer, count, 0, &temp);
	if (err)
		return err;
	xmos_pwdn_state = (unsigned int)temp;
	printk("wind xmos pwdn");
	gpio_direction_output(XMOS_AMP_PDN_GPIO,xmos_pwdn_state);
	//gpio_set_value(XMOS_AMP_PDN_GPIO,xmos_pwdn_state);
	
	return count;
}
//xmos
static const struct file_operations proc_xmos_pwdn_file_ops = {
	.open		= xmos_pwdn_read,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.write 		= xmos_pwdn_write,
	.release	= single_release,
};
static int aging_power_test_proc_init(void)
{
	aging_power_test_proc_dir = proc_mkdir(AGING_POWER_TEST_PROC_FOLDER, NULL);
	if (aging_power_test_proc_dir == NULL)
	{
		pr_err(" wind %s: aging_power_test_proc_dir file create failed!\n", __func__);
		return -ENOMEM;
	}
	//usb3.0
	proc_Charging_ChargeState_file = proc_create(AGING_POWER_TEST_PROC_CHARGING_CHARGESTATE, (S_IRUGO | S_IWUSR), 
		aging_power_test_proc_dir, &proc_Charging_ChargeState_file_ops);
	if(proc_Charging_ChargeState_file == NULL)
	{
		pr_err("wind  %s: proc_Charging_ChargeState_file file create failed!\n", __func__);
		remove_proc_entry( AGING_POWER_TEST_PROC_CHARGING_CHARGESTATE, aging_power_test_proc_dir );
		return -ENOMEM;
	}

	//xmos
	proc_xmos_set_file = proc_create(AGING_POWER_TEST_PROC_CHARGING_XMOS, (S_IRUGO | S_IWUSR), 
		aging_power_test_proc_dir, &proc_xmos_set_file_ops);
	if(proc_xmos_set_file == NULL)
	{
		pr_err("wind  %s: proc_xmos_set_file file create failed!\n", __func__);
		remove_proc_entry( AGING_POWER_TEST_PROC_CHARGING_XMOS, aging_power_test_proc_dir );
		return -ENOMEM;
	}

	//xmos power down
	proc_xmos_pwdn_file = proc_create(AGING_POWER_TEST_PROC_CHARGING_XMOS_PWDN, (S_IRUGO | S_IWUSR), 
		aging_power_test_proc_dir, &proc_xmos_pwdn_file_ops);
	if(proc_xmos_pwdn_file == NULL)
	{
		pr_err("wind  %s: proc_xmos_pwdn_file file create failed!\n", __func__);
		remove_proc_entry( AGING_POWER_TEST_PROC_CHARGING_XMOS_PWDN, aging_power_test_proc_dir );
		return -ENOMEM;
	}
	pr_err(" wind %s\n", __func__);
	return 0 ;
}

static int usb_temp_check_probe(struct platform_device *dev)
{
	aging_power_test_proc_init();
	
	gpio_direction_output(XMOS_CONNECT_USB_GPIO,1);//connect xmos to android,not type c
	gpio_direction_output(XMOS_AMP_PDN_GPIO,1);//pwdn xmos to avoid pop sound when bring up

	//gpio_direction_output(LCD_KEY_LED_VOLUME_WHITE_GPIO,1);//connect xmos to android,not type c
	//gpio_direction_output(LCD_KEY_LED_MUTE_WHITE_GPIO,0);//connect xmos to android,not type c
	//gpio_direction_output(LCD_KEY_LED_MUTE_RED_GPIO,1);//connect xmos to android,not type c	
	mdelay(100);

	printk("minglog:enter usb_temp_check_probe \n");
	return 0;
}

static int usb_temp_check_remove(struct platform_device *dev)
{
	 
	return 0;
}




struct of_device_id usb_temp_of_match[] = {
	{ .compatible = "qcom, usb_tem_det-eint" },	
	{},
};

static struct platform_driver usb_temp_check_driver = {
	.probe = usb_temp_check_probe,
	.remove =usb_temp_check_remove,
	.shutdown = NULL,
	.suspend = NULL,
	.resume = NULL,
	.driver = {
		   .name = "usb_temp_check",
		   	.of_match_table = usb_temp_of_match,
		   },
};



static int  usb_temp_check_init(void)
{
    int ret2;
	ret2 = platform_driver_register(&usb_temp_check_driver);
	printk("ldx usb_temp_check_init\n");
	if(ret2){
		printk("Unable to usb_temp_check_driver register(%d)\n",ret2);
		return ret2;
	}
	return 0;
}

static void __exit usb_temp_check_exit(void)
{
	platform_driver_unregister(&usb_temp_check_driver);
}


late_initcall(usb_temp_check_init);
module_exit(usb_temp_check_exit);
MODULE_AUTHOR("lvwenkang");
MODULE_DESCRIPTION("Usb_temp_check Driver");
MODULE_LICENSE("GPL");

//addde by  liulinsheng@wind-mobi 20171018  for usb_temp_check end

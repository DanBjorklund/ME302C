/*
 * ASUS switch button diver
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/io.h>
#include <asm/intel-mid.h>
#include <linux/wakelock.h>

#define DRIVER_VERSION		"0.0.0"

#define DRIVER_NAME		"switch"

struct switch_button_str {
	struct input_dev *input;
	struct work_struct switch_work;
	struct workqueue_struct *switch_wq;
	struct attribute_group attrs;
	struct wake_lock wake_lock;
	int switch_gpio;
	int lcd_change;
	int irq;
};

static struct switch_button_str *switch_dev;

static ssize_t switch_button_status(struct device *dev,struct device_attribute *attr, char *buf);

static DEVICE_ATTR(switch_status, (S_IWUSR|S_IRUGO), switch_button_status, NULL);

static struct attribute *switch_attr[] = {
	&dev_attr_switch_status.attr,
	NULL
};

static ssize_t switch_button_status(struct device *dev,struct device_attribute *attr, char *buf)
{
	int gpio_value;
	int status;
	
	gpio_value = gpio_get_value(switch_dev->switch_gpio);
	
	printk("[swith] %s:touch status=%d. \n", __func__, gpio_value);
        
	if(gpio_value > 0)
	    status = 1;
	else
	    status = 0;

	return sprintf(buf, "%d\n", status);
}

static void switch_work_func(struct work_struct *work)
{
	if (gpio_get_value(switch_dev->switch_gpio) > 0)
	{
		input_report_key(switch_dev->input, KEY_POWER, false);
	}
	else
	{
		input_report_key(switch_dev->input, KEY_POWER, true);
		
		if(!switch_dev->lcd_change)
		{
			wake_lock(&switch_dev->wake_lock);
			switch_dev->lcd_change = 1;
			printk("[switch] %s switch to windows \n", __func__);
		}
		else
		{
			wake_unlock(&switch_dev->wake_lock);
			switch_dev->lcd_change = 0;
			printk("[switch] %s switch to android \n", __func__);
		}

	}
	
	input_sync(switch_dev->input);
	
	return;
}

static irqreturn_t switch_interrupt_handler(int irq, void *dev_id)
{
	printk("[switch] %s ++ \n", __func__);
	
//	queue_delayed_work(switch_dev->switch_wq, &switch_dev->switch_work, 0);
	queue_work(switch_dev->switch_wq, &switch_dev->switch_work);
	return IRQ_HANDLED;
}

static int switch_probe(struct platform_device *pdev)
{
	int err;
	
	printk("[switch]: %s \n", __func__);
	
	switch_dev = kzalloc(sizeof (struct switch_button_str), GFP_KERNEL);
	if (!switch_dev) {
		printk(KERN_ERR "[switch] %s: switch_dev allocate switch_button_str failed\n", __func__);
		err = -ENOMEM;
		goto err_probe_alloc_data_failed;
	}
	
    switch_dev->switch_wq = create_singlethread_workqueue("switch_wq");
    if (!switch_dev->switch_wq) {
        printk(KERN_ERR "[switch] %s: create workqueue failed\n", __func__);
        err = -ENOMEM;
        goto err_probe_create_wq_failed;
    }
		
	INIT_WORK(&switch_dev->switch_work, switch_work_func);
	wake_lock_init(&switch_dev->wake_lock, WAKE_LOCK_SUSPEND, "switch_wake");
//	INIT_DELAYED_WORK_DEFERRABLE(&switch_dev->switch_work, switch_function);

	switch_dev->switch_gpio = get_gpio_by_name("switch");
	
	err = gpio_request(switch_dev->switch_gpio,"switch_button");
	if(err < 0)
	{
		printk(KERN_ERR "Failed to request GPIO%d (switch_button) error=%d\n", switch_dev->switch_gpio, err);
		goto err_probe_gpio_request_failed;
	}

	err = gpio_direction_input(switch_dev->switch_gpio);
	if (err)
	{
		printk(KERN_ERR "Failed to set gpio input failed, error=%d\n", err);
		goto err_probe_gpio_direction_input_failed;
	}
	
	switch_dev->irq = gpio_to_irq(switch_dev->switch_gpio);
	
	printk("[switch] %s: irq = %d, gpio = %d \n", __func__, switch_dev->irq, switch_dev->switch_gpio );
	
	err = request_irq(switch_dev->irq,switch_interrupt_handler,
			IRQF_SHARED|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, "switch_button_irq", switch_dev);
	if (err < 0) {
		printk("[switch] %s: Could not register for switch button interrupt, irq = %d, err = %d \n", __func__,  switch_dev->irq, err );
	    err = -EIO;
	    goto err_probe_request_irq_failed;
	}

	enable_irq_wake(switch_dev->irq);
	
	switch_dev->input = input_allocate_device();
	if (!switch_dev->input)
	{
		printk(KERN_ERR "[switch] %s: input allocate device failed!! \n", __func__);
		err = -ENOMEM;
		goto err_probe_input_allocate_device_failed;
	}
	
	switch_dev->input->name = "switch-button";
	switch_dev->input->dev.parent = &pdev->dev;
	
	input_set_capability(switch_dev->input, EV_KEY, KEY_POWER);
	
	
	err = input_register_device(switch_dev->input);
	if (err) {
		printk(KERN_ERR "[switch] %s: input device register failed, %d\n", __func__, err);
		goto err_probe_input_register_device_failed;
	}	
	
	switch_dev->attrs.attrs = switch_attr;
	err = sysfs_create_group(&pdev->dev.kobj, &switch_dev->attrs);
//	err = sysfs_create_group(&pdev->dev.kobj,&hall_sensor_group); 
	if (err) {
		printk("[swith] %s: Not able to create the sysfs. \n", __func__);
		goto err_probe_sysfs_create_group_failed;
	}
	
	if (gpio_get_value(switch_dev->switch_gpio) > 0)
	{
		switch_dev->lcd_change = 0; //0: android, 1: windows
	}
	else
	{
		switch_dev->lcd_change = 1; //0: android, 1: windows
	}
		
	printk("[swith]: switch button driver ver.%s \n",DRIVER_VERSION);
	
	return 0;
	
err_probe_sysfs_create_group_failed:
err_probe_input_register_device_failed:
	if (switch_dev->input)
		input_free_device(switch_dev->input);
err_probe_input_allocate_device_failed:
	disable_irq(switch_dev->irq);
err_probe_request_irq_failed:
	free_irq(switch_dev->irq, switch_dev);
err_probe_gpio_direction_input_failed:
	gpio_free(switch_dev->switch_gpio);
err_probe_gpio_request_failed:
	if (switch_dev->switch_wq)
		destroy_workqueue(switch_dev->switch_wq);
err_probe_create_wq_failed:
	kfree(switch_dev);
err_probe_alloc_data_failed:
	
	return err;

}

static const struct platform_device_id switch_table[] = {
	{DRIVER_NAME, 1},
};

static struct platform_driver switch_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe	= switch_probe,
//	.remove	= switch_remove,
	.id_table = switch_table,
};

static int __init switch_init(void)
{
	int ret;
	
	printk("[switch]: %s \n", __func__);
	
	ret = platform_driver_register(&switch_driver);
	return ret;
}

static void __exit switch_exit(void)
{
	printk("[switch]: %s \n", __func__);
	
	platform_driver_unregister(&switch_driver);
}

module_init(switch_init);
module_exit(switch_exit);

MODULE_AUTHOR("Joe_CH_Chen@asus.com");
MODULE_DESCRIPTION("asus switch button driver");
MODULE_LICENSE("GPL");

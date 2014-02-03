#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/kobject.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/async.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <asm/intel-mid.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <asm/intel_scu_pmic.h>

#define DRIVER_NAME "hall_sensor"


struct hall_sensor_str {
	struct input_dev *lid_indev;
	struct delayed_work hall_sensor_work;
	int irq;
};

static spinlock_t mHallSensorLock;
static int enable = 1;
static struct hall_sensor_str *hall_sensor_dev;
static struct workqueue_struct *hall_sensor_wq;
static int hall_sensor_gpio;
static struct kobject *hall_sensor_kobj;
static ssize_t show_action_status(struct device *dev,struct device_attribute *attr, char *buf)
{
	int gpio_value;
        int status;

	gpio_value = gpio_get_value(hall_sensor_gpio);
        
	pr_info("[%s] hall_sensor_gpio = %d gpio_value = %d ...\n", DRIVER_NAME, hall_sensor_gpio, gpio_value);
        
	if(gpio_value > 0)
	    status = 1;
	else
	    status = 0;
	
	
	return sprintf(buf, "%d\n", status);
}

static ssize_t show_hall_sensor_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", enable);
}

static ssize_t store_hall_sensor_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int request;
	sscanf(buf, "%du", &request);
	if(request==enable){
		return count;
	}
	else {
		unsigned long flags;
		spin_lock_irqsave(&mHallSensorLock, flags);
		if (enable==0){
			enable_irq(hall_sensor_dev->irq);
			enable=1;
		}
		else if (enable==1){		
			disable_irq(hall_sensor_dev->irq);
			enable=0;
		}
		spin_unlock_irqrestore(&mHallSensorLock, flags);
	}
	return count;
}


static SENSOR_DEVICE_ATTR_2(action_status, S_IRUGO, show_action_status, NULL, 0, 0);
static SENSOR_DEVICE_ATTR_2(activity, S_IRUGO|S_IWUSR,show_hall_sensor_enable, store_hall_sensor_enable, 0, 0);

static struct attribute *hall_sensor_attrs[] = {
	&sensor_dev_attr_action_status.dev_attr.attr,
	&sensor_dev_attr_activity.dev_attr.attr,
        NULL
};

static struct attribute_group hall_sensor_group = {
	.name = "hall_sensor",
        .attrs = hall_sensor_attrs
};

static void lid_set_input_params(struct input_dev *dev)
{
       set_bit(EV_SW, dev->evbit);
       set_bit(SW_LID, dev->swbit);
}
static int lid_input_device_create(void)
{
       int err = 0;

       hall_sensor_dev->lid_indev = input_allocate_device();     
       if(!hall_sensor_dev->lid_indev){
          pr_info("[%s] lid_indev allocation fails\n", DRIVER_NAME);
	  err = -ENOMEM;
	  goto exit;
       }

       hall_sensor_dev->lid_indev->name = "lid_input";
       hall_sensor_dev->lid_indev->phys= "/dev/input/lid_indev";
       hall_sensor_dev->lid_indev->dev.parent= NULL;

       lid_set_input_params(hall_sensor_dev->lid_indev);
       err = input_register_device(hall_sensor_dev->lid_indev);
       if (err) {
          pr_info("[%s] input registration fails\n", DRIVER_NAME);
	  err = -1;
	  goto exit_input_free;
       }
  
       return 0;
exit_input_free:
       input_free_device(hall_sensor_dev->lid_indev);
       hall_sensor_dev->lid_indev = NULL;
exit:
       return err;
}

static void lid_report_function(struct work_struct *dat)
{
	
	int status = 0;

	if (hall_sensor_dev->lid_indev==NULL){
	   pr_info("[%s] LID input device doesn't exist\n", DRIVER_NAME);
	   return;
	}

	msleep(50);
	if (gpio_get_value(hall_sensor_gpio) > 0)
		status = 1;
	else
		status = 0;

	input_report_switch(hall_sensor_dev->lid_indev, SW_LID, !status);
	input_sync(hall_sensor_dev->lid_indev);
	pr_info("[%s] SW_LID report value = %d\n", DRIVER_NAME,!status);



}

static irqreturn_t hall_sensor_interrupt_handler(int irq, void *dev_id)
{
	pr_info("[%s] hall_sensor_interrupt = %d\n", DRIVER_NAME,hall_sensor_dev->irq);
	queue_delayed_work(hall_sensor_wq, &hall_sensor_dev->hall_sensor_work, 0);
	return IRQ_HANDLED;
}

static int set_irq_hall_sensor(void)
{
	int rc = 0 ;

	hall_sensor_dev->irq = gpio_to_irq(hall_sensor_gpio);
	pr_info("[%s] hall_sensor irq = %d\n", DRIVER_NAME,hall_sensor_dev->irq);
	rc = request_irq(hall_sensor_dev->irq,hall_sensor_interrupt_handler,
			IRQF_SHARED|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING,"hall_sensor_irq",hall_sensor_dev);
	if (rc<0) {
	    pr_info("[%s] Could not register for hall sensor interrupt, irq = %d, rc = %d\n", DRIVER_NAME,hall_sensor_dev->irq,rc);
	    rc = -EIO;
	    goto err_gpio_request_irq_fail ;
	}

	enable_irq_wake(hall_sensor_dev->irq);

	return 0;

err_gpio_request_irq_fail:
	return rc;
}


static int __init hall_sensor_init(void)
{	


	int ret;

	//set gpio
        hall_sensor_gpio = get_gpio_by_name("1V8_O_LID#");
        if (!gpio_is_valid(hall_sensor_gpio))
	{
		 pr_info("[%s] GPIO for hall sensor does not exist.\n", DRIVER_NAME);
		 return -1;
	}
	gpio_request(hall_sensor_gpio,"hall_sensor_gpio");
	gpio_direction_input(hall_sensor_gpio);

	//set file node
        hall_sensor_kobj = kobject_create_and_add("hall_sensor_kobject", kernel_kobj);
	if (!hall_sensor_kobj)
		return -ENOMEM;
        ret = sysfs_create_group(hall_sensor_kobj, &hall_sensor_group);
        if (ret)
                kobject_put(hall_sensor_kobj);

        spin_lock_init(&mHallSensorLock);

	hall_sensor_dev = kzalloc(sizeof (struct hall_sensor_str), GFP_KERNEL);
	if (!hall_sensor_dev) {
	    pr_info("[%s] Memory allocation fails for hall sensor\n", DRIVER_NAME);
	    ret = -ENOMEM;
	    goto fail_for_hall_sensor;
	}

	hall_sensor_dev->lid_indev = NULL;

	ret = lid_input_device_create();
	if (ret < 0)
	    goto fail_for_hall_sensor;
        
	hall_sensor_wq = create_singlethread_workqueue("hall_sensor_wq");
	INIT_DELAYED_WORK_DEFERRABLE(&hall_sensor_dev->hall_sensor_work, lid_report_function);

	ret = set_irq_hall_sensor();
	if (ret < 0)
	    goto fail_for_irq_hall_sensor;

        queue_delayed_work(hall_sensor_wq, &hall_sensor_dev->hall_sensor_work, 0);
	return 0;
		
		
fail_for_irq_hall_sensor:
        input_free_device(hall_sensor_dev->lid_indev);
	hall_sensor_dev->lid_indev = NULL;
	kfree(hall_sensor_dev);
fail_for_hall_sensor:
	return ret;

}

static void __exit hall_sensor_exit(void)
{
	free_irq(hall_sensor_dev->irq, hall_sensor_dev);
	input_free_device(hall_sensor_dev->lid_indev);
	kfree(hall_sensor_dev);
	kobject_put(hall_sensor_kobj);
}


module_init(hall_sensor_init);
module_exit(hall_sensor_exit);


MODULE_DESCRIPTION("Intel Hall sensor Driver");
MODULE_LICENSE("GPL v2");

/* drivers/input/touchscreen/gt927.c
 * 
 * 2010 - 2013 Goodix Technology.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be a reference 
 * to you, when you are integrating the GOODiX's CTP IC into your system, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * General Public License for more details.
 * 
 * Latest Version:  1.6
 * Authors: andrew@goodix.com, meta@goodix.com
 * Release Date: 2013/03/
 * Revision record:
 *      V1.0:   
 *          first Release. By Andrew, 2012/08/31 
 *      V1.2:
 *          modify gtp_reset_guitar,slot report,tracking_id & 0x0F. By Andrew, 2012/10/15
 *      V1.4:
 *          modify gt927_update.c. By Andrew, 2012/12/12
 *      V1.6: 
 *          1. new heartbeat/esd_protect mechanism(add external watchdog)
 *          2. doze mode, sliding wakeup 
 *          3. 3 more cfg_group(GT9 Sensor_ID: 0~5) 
 *          3. config length verification
 *          4. names & comments
 *                  By Meta, 2013/03/11
 */

#include <linux/irq.h>
//#include "gt927.h" // mark by leo
#include <linux/i2c/gt927.h>	// add by leo

#if GTP_ICS_SLOT_REPORT
    #include <linux/input/mt.h>
#endif

#define CFG_GROUP_LEN(p_cfg_grp)  (sizeof(p_cfg_grp) / sizeof(p_cfg_grp[0]))

//static const char *goodix_ts_name = "Goodix Capacitive TouchScreen";//mark by leo
static const char *goodix_ts_name = "Goodix-touchscreen";//add by leo
static struct workqueue_struct *goodix_wq;
struct i2c_client * i2c_connect_client = NULL; 
u8 config[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH]
                = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};

#if GTP_HAVE_TOUCH_KEY
    static const u16 touch_key_array[] = GTP_KEY_TAB;
    #define GTP_MAX_KEY_NUM  (sizeof(touch_key_array)/sizeof(touch_key_array[0]))
#endif

static s8 gtp_i2c_test(struct i2c_client *client);
void gtp_reset_guitar(struct i2c_client *client, s32 ms);
void gtp_int_sync(s32 ms);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void goodix_ts_early_suspend(struct early_suspend *h);
static void goodix_ts_late_resume(struct early_suspend *h);
#endif
 
#if GTP_CREATE_WR_NODE
extern s32 init_wr_node(struct i2c_client*);
extern void uninit_wr_node(void);
#endif

#if GTP_AUTO_UPDATE
extern u8 gup_init_update_proc(struct goodix_ts_data *);
#endif

#if GTP_ESD_PROTECT
static struct delayed_work gtp_esd_check_work;
static struct workqueue_struct * gtp_esd_check_workqueue = NULL;
static void gtp_esd_check_func(struct work_struct *);
s32 gtp_init_ext_watchdog(struct i2c_client *client);
#endif


#if GTP_SLIDING_WAKEUP
u8 doze_enabled = 0;
#endif

//add by leo ++
static DEFINE_SPINLOCK(gt927_spin_lock);
//add by leo --

//other
s32 gtp_read_version(struct i2c_client *client, u16* version);


static ssize_t gt927_reset(struct device *dev, struct device_attribute *attr, char *buf){

	gtp_reset_guitar(touch_chip->client, 50);
	
	return sprintf(buf,"%s Finished\n", __func__);
}
static DEVICE_ATTR(gt927_rst, S_IRUGO, gt927_reset, NULL);

static ssize_t gt927_work_trigger(struct device *dev, struct device_attribute *attr, char *buf){

	printk("////////////////////		%s:[%d]: Start \n", __func__, __LINE__); // add by leo
   	queue_work(goodix_wq, &touch_chip->work);
	printk("////////////////////		%s:[%d]: End \n", __func__, __LINE__); // add by leo
	
	return sprintf(buf,"%s Finished\n", __func__);
}
static DEVICE_ATTR(gt927_work, S_IRUGO, gt927_work_trigger, NULL);

static ssize_t gt927_get_fw_version(struct device *dev, struct device_attribute *attr, char *buf){

	u16 version_info;
	int err=0;

	err = gtp_read_version(touch_chip->client, &version_info);
	if (err < 0)
	{
	    GTP_ERROR("Read version failed.");
	}
	return sprintf(buf,"gtp_read_version (%d)\n", err);
}
static DEVICE_ATTR(tp_fw_version, S_IRUGO, gt927_get_fw_version, NULL);

static ssize_t gt927_InterruptPin_show(struct device *dev, struct device_attribute *attr, char *buf){

	int intr_value=0, err=0;

	err = gpio_direction_input(touch_chip->intr_gpio);
	if (err){
		printk("[gt927] %s: Failed to set (GoodixTouch-interrupt) input direction, error=%d\n", __func__, err);
		gpio_free(touch_chip->intr_gpio);
	}

	intr_value = gpio_get_value(GTP_INT_PORT);

	return sprintf(buf,"Get GPIO INT value = %d\n", intr_value);
}
static ssize_t gt927_InterruptPin_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count){

	int intr_value=0,err=0,i=0;

	sscanf (buf, "%d", &intr_value);
	
	err = gpio_direction_output(touch_chip->intr_gpio,0);
	if (err){
		printk("[gt927] %s: Failed to set (GoodixTouch-interrupt) output direction, error=%d\n", __func__, err);
		gpio_free(touch_chip->intr_gpio);
	}

	for(i=0;i<20;i++){
		gpio_set_value(GTP_INT_PORT, 1);
		msleep(500);
		gpio_set_value(GTP_INT_PORT, 0);
		msleep(500);
	}

	gpio_set_value(GTP_INT_PORT, intr_value);
	
	printk("////////////////////		%s: Set GPIO INT (%d)\n", __func__, intr_value);
	return count;
}
static DEVICE_ATTR(gt927_intpin, S_IWUSR | S_IRUGO , gt927_InterruptPin_show, gt927_InterruptPin_store);


static ssize_t gt927_ResetPin_show(struct device *dev, struct device_attribute *attr, char *buf){

	int rst_value=0;

	rst_value = gpio_get_value(GTP_RST_PORT);

	return sprintf(buf,"Get GPIO RST value = %d\n", rst_value);
}
static ssize_t gt927_ResetPin_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count){

	int rst_value=0;

	sscanf (buf, "%d", &rst_value);
	gpio_set_value(GTP_RST_PORT, rst_value);
	
	printk("////////////////////		%s: Set GPIO RST (%d)\n", __func__, rst_value);
	return count;
}
static DEVICE_ATTR(gt927_rstpin, S_IWUSR | S_IRUGO , gt927_ResetPin_show, gt927_ResetPin_store);

static struct attribute *gt927_attributes[] = {
	&dev_attr_gt927_rstpin.attr,
	&dev_attr_gt927_intpin.attr,
	&dev_attr_tp_fw_version.attr,
	&dev_attr_gt927_work.attr,
	&dev_attr_gt927_rst.attr,
	NULL
};

static const struct attribute_group gt927_attr_group = {
	.attrs = gt927_attributes,
};
/*******************************************************
Function:
    Read data from the i2c slave device.
Input:
    client:     i2c device.
    buf[0~1]:   read start address.
    buf[2~len-1]:   read data buffer.
    len:    GTP_ADDR_LENGTH + read bytes count
Output:
    numbers of i2c_msgs to transfer: 
      2: succeed, otherwise: failed
*********************************************************/
s32 gtp_i2c_read(struct i2c_client *client, u8 *buf, s32 len)
{
    struct i2c_msg msgs[2];
    s32 ret=-1;
    s32 retries = 0;

    GTP_DEBUG_FUNC();

    msgs[0].flags = !I2C_M_RD;
    msgs[0].addr  = client->addr;
    msgs[0].len   = GTP_ADDR_LENGTH;
    msgs[0].buf   = &buf[0];

    msgs[1].flags = I2C_M_RD;
    msgs[1].addr  = client->addr;
    msgs[1].len   = len - GTP_ADDR_LENGTH;
    msgs[1].buf   = &buf[GTP_ADDR_LENGTH];

    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, msgs, 2);
        if(ret == 2)break;
        retries++;
    }
    if((retries >= 5))
    {
    #if GTP_SLIDING_WAKEUP
        // reset chip would quit doze mode
        if (doze_enabled)
        {
            return ret;
        }
    #endif
        GTP_DEBUG("I2C communication timeout, resetting chip...");
        gtp_reset_guitar(client, 10);
    }
    return ret;
}

/*******************************************************
Function:
    Write data to the i2c slave device.
Input:
    client:     i2c device.
    buf[0~1]:   write start address.
    buf[2~len-1]:   data buffer
    len:    GTP_ADDR_LENGTH + write bytes count
Output:
    numbers of i2c_msgs to transfer: 
        1: succeed, otherwise: failed
*********************************************************/
s32 gtp_i2c_write(struct i2c_client *client,u8 *buf,s32 len)
{
    struct i2c_msg msg;
    s32 ret = -1;
    s32 retries = 0;

    GTP_DEBUG_FUNC();

    msg.flags = !I2C_M_RD;
    msg.addr  = client->addr;
    msg.len   = len;
    msg.buf   = buf;

    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, &msg, 1);
        if (ret == 1)break;
        retries++;
    }
    if((retries >= 5))
    {
    #if GTP_SLIDING_WAKEUP
        if (doze_enabled)
        {
            return ret;
        }
    #endif
        GTP_DEBUG("I2C communication timeout, resetting chip...");
        gtp_reset_guitar(client, 10);
    }
    return ret;
}


/*******************************************************
Function:
    Send config.
Input:
    client: i2c device.
Output:
    result of i2c write operation. 
        1: succeed, otherwise: failed
*********************************************************/
s32 gtp_send_cfg(struct i2c_client *client)
{
    s32 ret = 2;

    printk("////////////////////		%s:[%d]: Start \n", __func__, __LINE__); // add by leo
#if GTP_DRIVER_SEND_CFG
    s32 retry = 0;

    for (retry = 0; retry < 5; retry++)
    {
        ret = gtp_i2c_write(client, config , GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);
        if (ret > 0)
        {
            break;
        }
    }
#endif

    return ret;
}

/*******************************************************
Function:
    Disable irq function
Input:
    ts: goodix i2c_client private data
Output:
    None.
*********************************************************/
void gtp_irq_disable(struct goodix_ts_data *ts)
{
    unsigned long irqflags;

    GTP_DEBUG_FUNC();

    spin_lock_irqsave(&touch_chip->irq_lock, irqflags);
    if (!touch_chip->irq_is_disable)
    {
        touch_chip->irq_is_disable = 1; 
        disable_irq_nosync(touch_chip->client->irq);
    }
    spin_unlock_irqrestore(&touch_chip->irq_lock, irqflags);
}

/*******************************************************
Function:
    Enable irq function
Input:
    ts: goodix i2c_client private data
Output:
    None.
*********************************************************/
void gtp_irq_enable(struct goodix_ts_data *ts)
{
    unsigned long irqflags = 0;
	
    GTP_DEBUG_FUNC();
    
    spin_lock_irqsave(&touch_chip->irq_lock, irqflags);
    if (touch_chip->irq_is_disable) 
    {
        enable_irq(touch_chip->client->irq);
        touch_chip->irq_is_disable = 0; 
    }
    spin_unlock_irqrestore(&touch_chip->irq_lock, irqflags);
}


/*******************************************************
Function:
    Report touch point event 
Input:
    ts: goodix i2c_client private data
    id: trackId
    x:  input x coordinate
    y:  input y coordinate
    w:  input pressure
Output:
    None.
*********************************************************/
static void gtp_touch_down(struct goodix_ts_data* ts,s32 id,s32 x,s32 y,s32 w)
{
#if GTP_CHANGE_X2Y
    GTP_SWAP(x, y);
#endif

#if GTP_ICS_SLOT_REPORT
    input_mt_slot(touch_chip->input_dev, id);
    input_report_abs(touch_chip->input_dev, ABS_MT_TRACKING_ID, id);
    input_report_abs(touch_chip->input_dev, ABS_MT_POSITION_X, x);
    input_report_abs(touch_chip->input_dev, ABS_MT_POSITION_Y, y);
    input_report_abs(touch_chip->input_dev, ABS_MT_TOUCH_MAJOR, w);
    input_report_abs(touch_chip->input_dev, ABS_MT_WIDTH_MAJOR, w);
#else
    input_report_abs(touch_chip->input_dev, ABS_MT_POSITION_X, x);
    input_report_abs(touch_chip->input_dev, ABS_MT_POSITION_Y, y);
    input_report_abs(touch_chip->input_dev, ABS_MT_TOUCH_MAJOR, w);
    input_report_abs(touch_chip->input_dev, ABS_MT_WIDTH_MAJOR, w);
    input_report_abs(touch_chip->input_dev, ABS_MT_TRACKING_ID, id);
    input_mt_sync(touch_chip->input_dev);
#endif

    GTP_DEBUG("ID:%d, X:%d, Y:%d, W:%d", id, x, y, w);
}

/*******************************************************
Function:
    Report touch release event
Input:
    ts: goodix i2c_client private data
Output:
    None.
*********************************************************/
static void gtp_touch_up(struct goodix_ts_data* ts, s32 id)
{
#if GTP_ICS_SLOT_REPORT
    input_mt_slot(touch_chip->input_dev, id);
    input_report_abs(touch_chip->input_dev, ABS_MT_TRACKING_ID, -1);
    GTP_DEBUG("Touch id[%2d] release!", id);
#else
    input_report_abs(touch_chip->input_dev, ABS_MT_TOUCH_MAJOR, 0);
    input_report_abs(touch_chip->input_dev, ABS_MT_WIDTH_MAJOR, 0);
    input_mt_sync(touch_chip->input_dev);
#endif
}


/*******************************************************
Function:
    Goodix touchscreen work function
Input:
    work: work struct of goodix_workqueue
Output:
    None.
*********************************************************/
static void goodix_ts_work_func(struct work_struct *work)
{
    u8  end_cmd[3] = {GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF, 0};
    u8  point_data[2 + 1 + 8 * GTP_MAX_TOUCH + 1]={GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF};
    u8  touch_num = 0;
    u8  finger = 0;
    static u16 pre_touch = 0;
    static u8 pre_key = 0;
    u8  key_value = 0;
    u8* coor_data = NULL;
    s32 input_x = 0;
    s32 input_y = 0;
    s32 input_w = 0;
    s32 id = 0;
    s32 i  = 0;
    s32 ret = -1;
    //struct goodix_ts_data *ts = NULL; //mark by leo

#if GTP_SLIDING_WAKEUP
    u8 buf[3] = {0x81, 0x4B};
#endif

    GTP_DEBUG_FUNC();

    touch_chip = container_of(work, struct goodix_ts_data, work);
    if (touch_chip->enter_update)
    {
        return;
    }

#if GTP_SLIDING_WAKEUP
    if (doze_enabled)
    {               
        ret = gtp_i2c_read(i2c_connect_client, buf, 3);
        GTP_DEBUG("0x814B = 0x%02X", buf[2]);
        if (ret > 0)
        {               
            if (buf[2] == 0xAA)
            {
                GTP_INFO("Sliding To Light up the screen!");
                input_report_key(touch_chip->input_dev, KEY_POWER, 1);
                input_report_key(touch_chip->input_dev, KEY_POWER, 0);
                doze_enabled = 0;
            }
        }
        goto exit_work_func;
    }
#endif

    ret = gtp_i2c_read(touch_chip->client, point_data, 12);
    if (ret < 0)
    {
        GTP_ERROR("I2C transfer error. errno:%d\n ", ret);
        goto exit_work_func;
    }

    finger = point_data[GTP_ADDR_LENGTH];    
    if((finger & 0x80) == 0)
    {
        goto exit_work_func;
    }

    touch_num = finger & 0x0f;
    if (touch_num > GTP_MAX_TOUCH)
    {
        goto exit_work_func;
    }

    if (touch_num > 1)
    {
        u8 buf[8 * GTP_MAX_TOUCH] = {(GTP_READ_COOR_ADDR + 10) >> 8, (GTP_READ_COOR_ADDR + 10) & 0xff};

        ret = gtp_i2c_read(touch_chip->client, buf, 2 + 8 * (touch_num - 1)); 
        memcpy(&point_data[12], &buf[2], 8 * (touch_num - 1));
    }

#if GTP_HAVE_TOUCH_KEY
    key_value = point_data[3 + 8 * touch_num];
    
    if(key_value || pre_key)
    {
        for (i = 0; i < GTP_MAX_KEY_NUM; i++)
        {
            input_report_key(touch_chip->input_dev, touch_key_array[i], key_value & (0x01<<i));   
        }
        touch_num = 0;
        pre_touch = 0;
    }
#endif
    pre_key = key_value;

    GTP_DEBUG("pre_touch:%02x, finger:%02x.", pre_touch, finger);

#if GTP_ICS_SLOT_REPORT
    if (pre_touch || touch_num)
    {
        s32 pos = 0;
        u16 touch_index = 0;

        coor_data = &point_data[3];
        if(touch_num)
        {
            id = coor_data[pos] & 0x0F;
            touch_index |= (0x01<<id);
        }

        GTP_DEBUG("id = %d,touch_index = 0x%x, pre_touch = 0x%x\n",id, touch_index,pre_touch);
        for (i = 0; i < GTP_MAX_TOUCH; i++)
        {
            if (touch_index & (0x01<<i))
            {
                input_x  = coor_data[pos + 1] | coor_data[pos + 2] << 8;
                input_y  = coor_data[pos + 3] | coor_data[pos + 4] << 8;
                input_w  = coor_data[pos + 5] | coor_data[pos + 6] << 8;

                gtp_touch_down(touch_chip, id, input_x, input_y, input_w);
                pre_touch |= 0x01 << i;

                pos += 8;
                id = coor_data[pos] & 0x0F;
                touch_index |= (0x01<<id);
            }
            else
            {
                gtp_touch_up(touch_chip, i);
                pre_touch &= ~(0x01 << i);
            }
        }
    }

#else
    if (touch_num)
    {
        for (i = 0; i < touch_num; i++)
        {
            coor_data = &point_data[i * 8 + 3];

            id = coor_data[0] & 0x0F;
            input_x  = coor_data[1] | coor_data[2] << 8;
            input_y  = coor_data[3] | coor_data[4] << 8;
            input_w  = coor_data[5] | coor_data[6] << 8;

            gtp_touch_down(touch_chip, id, input_x, input_y, input_w);
        }
    }
    else if (pre_touch)
    {
        GTP_DEBUG("Touch Released!");
        gtp_touch_up(touch_chip, 0);
    }

    pre_touch = touch_num;
    input_report_key(touch_chip->input_dev, BTN_TOUCH, (touch_num || key_value));
#endif

    input_sync(touch_chip->input_dev);

exit_work_func:
    if(!touch_chip->gtp_rawdiff_mode)
    {
        ret = gtp_i2c_write(touch_chip->client, end_cmd, 3);
        if (ret < 0)
        {
            GTP_INFO("I2C write end_cmd error!");
        }
    }
    if (touch_chip->use_irq)
    {
        gtp_irq_enable(touch_chip);
    }
}

/*******************************************************
Function:
    Timer interrupt service routine for polling mode.
Input:
    timer: timer struct pointer
Output:
    Timer work mode. 
        HRTIMER_NORESTART: no restart mode
*********************************************************/
static enum hrtimer_restart goodix_ts_timer_handler(struct hrtimer *timer)
{
    struct goodix_ts_data *ts = container_of(timer, struct goodix_ts_data, timer);

    printk("////////////////////		%s:[%d]: Start \n", __func__, __LINE__); // add by leo
    GTP_DEBUG_FUNC();

    queue_work(goodix_wq, &ts->work);
    hrtimer_start(&ts->timer, ktime_set(0, (GTP_POLL_TIME+6)*1000000), HRTIMER_MODE_REL);
    return HRTIMER_NORESTART;
}

/*******************************************************
Function:
    External interrupt service routine for interrupt mode.
Input:
    irq:  interrupt number.
    dev_id: private data pointer
Output:
    Handle Result.
        IRQ_HANDLED: interrupt handled successfully
*********************************************************/
static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
    //struct goodix_ts_data *ts = dev_id; //mark by leo

    GTP_DEBUG_FUNC();
 
    gtp_irq_disable(touch_chip);

    queue_work(goodix_wq, &touch_chip->work);
    
    return IRQ_HANDLED;
}
/*******************************************************
Function:
    Synchronization.
Input:
    ms: synchronization time in millionsecond.
Output:
    None.
*******************************************************/
void gtp_int_sync(s32 ms)
{
    printk("////////////////////		%s:[%d]: Start \n", __func__, __LINE__); // add by leo
    //GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
    msleep(ms);
    //GTP_GPIO_AS_INT(GTP_INT_PORT);
}

/*******************************************************
Function:
    Reset chip.
Input:
    ms: reset time in millionsecond
Output:
    None.
*******************************************************/
void gtp_reset_guitar(struct i2c_client *client, s32 ms)
{
       printk("////////////////////		%s:[%d]: Start \n", __func__, __LINE__); // add by leo
	GTP_DEBUG_FUNC();

	disable_irq(touch_chip->irq);

    	gpio_set_value(touch_chip->rst_gpio, 0); // begin select I2C slave addr

	msleep(ms);                        // T2: > 10ms
	// HIGH: 0x28/0x29, LOW: 0xBA/0xBB
//	gpio_set_value(touch_chip->intr_gpio, 0);	 //  1:slave addr=0x14 ; 0:slave addr=0x5d

    	msleep(2);                          // T3: > 100us
	gpio_set_value(touch_chip->rst_gpio, 1);

    	msleep(6);                          // T4: > 5ms
//	gpio_set_value(touch_chip->intr_gpio, 0); // end select I2C slave addr

	gtp_int_sync(50);

// add by leo for testtest ++
	gtp_send_cfg(touch_chip->client);
       msleep(100);
// add by leo for testtest --

	enable_irq(touch_chip->irq);
    
#if GTP_ESD_PROTECT
    	gtp_init_ext_watchdog(client);
#endif
}

#if GTP_SLIDING_WAKEUP
/*******************************************************
Function:
    Enter doze mode for sliding wakeup.
Input:
    ts: goodix tp private data
Output:
    1: succeed, otherwise failed
*******************************************************/
static s8 gtp_enter_doze(struct goodix_ts_data *ts)
{
    s8 ret = -1;
    s8 retry = 0;
    u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 8};

    printk("////////////////////		%s:[%d]: Start \n", __func__, __LINE__); // add by leo
    GTP_DEBUG_FUNC();

    gtp_irq_disable(touch_chip);
    gtp_reset_guitar(touch_chip->client, 20);
    msleep(50);         // wait for INT port transferred into FLOATING INPUT STATUS

    GTP_DEBUG("entering doze mode...");
    while(retry++ < 5)
    {
        ret = gtp_i2c_write(touch_chip->client, i2c_control_buf, 3);
        if (ret > 0)
        {
            doze_enabled = 1;
            GTP_INFO("GTP has been working in doze mode!");
            gtp_irq_enable(touch_chip);
            return ret;
        }
        msleep(10);
    }
    GTP_ERROR("GTP send doze cmd failed.");
    gtp_irq_enable(touch_chip);
    return ret;
}
#else 
/*******************************************************
Function:
    Enter sleep mode.
Input:
    ts: private data.
Output:
    Executive outcomes.
       1: succeed, otherwise failed.
*******************************************************/
static s8 gtp_enter_sleep(struct goodix_ts_data * ts)
{
    s8 ret = -1;
    s8 retry = 0;
    u8 i2c_control_buf[3] = {(u8)(GTP_REG_SLEEP >> 8), (u8)GTP_REG_SLEEP, 5};

    printk("////////////////////		%s:[%d]: Start \n", __func__, __LINE__); // add by leo
    GTP_DEBUG_FUNC();
    
    //GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
    msleep(5);
    while(retry++ < 5)
    {
        ret = gtp_i2c_write(touch_chip->client, i2c_control_buf, 3);
        if (ret > 0)
        {
            GTP_INFO("GTP enter sleep!");
            
            return ret;
        }
        msleep(10);
    }
    GTP_ERROR("GTP send sleep cmd failed.");
    return ret;
}
#endif 
/*******************************************************
Function:
    Wakeup from sleep.
Input:
    ts: private data.
Output:
    Executive outcomes.
        >0: succeed, otherwise: failed.
*******************************************************/
static s8 gtp_wakeup_sleep(struct goodix_ts_data * ts)
{
    u8 retry = 0;
    s8 ret = -1;

    printk("////////////////////		%s:[%d]: Start \n", __func__, __LINE__); // add by leo
    GTP_DEBUG_FUNC();
    
#if GTP_POWER_CTRL_SLEEP
    while(retry++ < 5)
    {
        gtp_reset_guitar(touch_chip->client, 20);
        
    #if GTP_DRIVER_SEND_CFG
        ret = gtp_send_cfg(touch_chip->client);
        if (ret > 0)
        {
            GTP_INFO("Wakeup sleep send config success.");
            return ret;
        }
    #else
        GTP_INFO("GTP Wakeup!");
        return 1;
    #endif
    }
#else
    while(retry++ < 10)
    {
        //GTP_GPIO_OUTPUT(GTP_INT_PORT, 1);
        msleep(5);
        
        ret = gtp_i2c_test(touch_chip->client);
        if (ret > 0)
        {
            GTP_INFO("GTP wakeup sleep.");
            
            gtp_int_sync(25);
            msleep(20);
        #if GTP_SLIDING_WAKEUP
            gtp_init_ext_watchdog(touch_chip->client);
        #endif
            return ret;
        }
        gtp_reset_guitar(touch_chip->client, 20);
    }
#endif

    GTP_ERROR("GTP wakeup sleep failed.");
    return ret;
}

/*******************************************************
Function:
    Initialize gtp.
Input:
    ts: goodix private data
Output:
    Executive outcomes.
        0: succeed, otherwise: failed
*******************************************************/
static s32 gtp_init_panel(struct goodix_ts_data *ts)
{
    s32 ret = -1;

    printk("////////////////////		%s:[%d]: Start \n", __func__, __LINE__); // add by leo

#if GTP_DRIVER_SEND_CFG
    s32 i;
    u8 check_sum = 0;
    u8 opr_buf[16];
    u8 sensor_id = 0;

    u8 cfg_info_group1[] = CTP_CFG_GROUP1;
    u8 cfg_info_group2[] = CTP_CFG_GROUP2;
    u8 cfg_info_group3[] = CTP_CFG_GROUP3;
    u8 cfg_info_group4[] = CTP_CFG_GROUP4;
    u8 cfg_info_group5[] = CTP_CFG_GROUP5;
    u8 cfg_info_group6[] = CTP_CFG_GROUP6;
    u8 *send_cfg_buf[] = {cfg_info_group1, cfg_info_group2, cfg_info_group3,
                        cfg_info_group4, cfg_info_group5, cfg_info_group6};
    u8 cfg_info_len[] = { CFG_GROUP_LEN(cfg_info_group1), 
                          CFG_GROUP_LEN(cfg_info_group2),
                          CFG_GROUP_LEN(cfg_info_group3),
                          CFG_GROUP_LEN(cfg_info_group4), 
                          CFG_GROUP_LEN(cfg_info_group5),
                          CFG_GROUP_LEN(cfg_info_group6)};

    GTP_DEBUG("Config Groups\' Lengths: %d, %d, %d, %d, %d, %d", 
        cfg_info_len[0], cfg_info_len[1], cfg_info_len[2], cfg_info_len[3],
        cfg_info_len[4], cfg_info_len[5]);


    if ((!cfg_info_len[1]) && (!cfg_info_len[2]) && 
        (!cfg_info_len[3]) && (!cfg_info_len[4]) && 
        (!cfg_info_len[5]))
    {
        sensor_id = 0; 
    }
    else
    {
        opr_buf[0] = (u8)(GTP_REG_SENSOR_ID >> 8);
        opr_buf[1] = (u8)(GTP_REG_SENSOR_ID & 0xff);
        ret = gtp_i2c_read(touch_chip->client, opr_buf, 3);
        if (ret < 0)
        {
            GTP_ERROR("Failed to read Sensor_ID, using DEFAULT config!");
            sensor_id = 0;
            if (cfg_info_len[0] != 0)
            {
                send_cfg_buf[0][0] = 0x00;      // RESET Config Version
            }
        }
        else
        {
            sensor_id = opr_buf[2] & 0x07;
        }
    }
    GTP_DEBUG("Sensor_ID: %d", sensor_id);
    
    touch_chip->gtp_cfg_len = cfg_info_len[sensor_id];
    
    if (touch_chip->gtp_cfg_len == 0)
    {
        GTP_ERROR("Sensor_ID(%d) matches with NULL CONFIG GROUP!NO Config Send! You need to check you header file CFG_GROUP section!", sensor_id);
        return -1;
    }
    
    memset(&config[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);
    memcpy(&config[GTP_ADDR_LENGTH], send_cfg_buf[sensor_id], touch_chip->gtp_cfg_len);

#if GTP_CUSTOM_CFG
    config[RESOLUTION_LOC]     = (u8)GTP_MAX_WIDTH;
    config[RESOLUTION_LOC + 1] = (u8)(GTP_MAX_WIDTH>>8);
    config[RESOLUTION_LOC + 2] = (u8)GTP_MAX_HEIGHT;
    config[RESOLUTION_LOC + 3] = (u8)(GTP_MAX_HEIGHT>>8);
    
    if (GTP_INT_TRIGGER == 0)  //RISING
    {
        config[TRIGGER_LOC] &= 0xfe; 
    }
    else if (GTP_INT_TRIGGER == 1)  //FALLING
    {
        config[TRIGGER_LOC] |= 0x01;
    }
#endif  // GTP_CUSTOM_CFG
    
    check_sum = 0;
    for (i = GTP_ADDR_LENGTH; i < touch_chip->gtp_cfg_len; i++)
    {
        check_sum += config[i];
    }
    config[touch_chip->gtp_cfg_len] = (~check_sum) + 1;
    
#else // DRIVER NOT SEND CONFIG
    touch_chip->gtp_cfg_len = GTP_CONFIG_MAX_LENGTH;
    ret = gtp_i2c_read(touch_chip->client, config, touch_chip->gtp_cfg_len + GTP_ADDR_LENGTH);
    if (ret < 0)
    {
        GTP_ERROR("Read Config Failed, Using DEFAULT Resolution & INT Trigger!");
        touch_chip->abs_x_max = GTP_MAX_WIDTH;
        touch_chip->abs_y_max = GTP_MAX_HEIGHT;
        touch_chip->int_trigger_type = GTP_INT_TRIGGER;
    }
#endif // GTP_DRIVER_SEND_CFG

    GTP_DEBUG_FUNC();
    if ((touch_chip->abs_x_max == 0) && (touch_chip->abs_y_max == 0))
    {
        touch_chip->abs_x_max = (config[RESOLUTION_LOC + 1] << 8) + config[RESOLUTION_LOC];
        touch_chip->abs_y_max = (config[RESOLUTION_LOC + 3] << 8) + config[RESOLUTION_LOC + 2];
        touch_chip->int_trigger_type = (config[TRIGGER_LOC]) & 0x03; 
    }
    ret = gtp_send_cfg(touch_chip->client);
    if (ret < 0)
    {
        GTP_ERROR("Send config error.");
    }
    GTP_DEBUG("X_MAX = %d, Y_MAX = %d, TRIGGER = 0x%02x",
        touch_chip->abs_x_max,touch_chip->abs_y_max,touch_chip->int_trigger_type);


    msleep(10);
    return 0;
}

/*******************************************************
Function:
    Read chip version.
Input:
    client:  i2c device
    version: buffer to keep ic firmware version
Output:
    read operation return.
        2: succeed, otherwise: failed
*******************************************************/
s32 gtp_read_version(struct i2c_client *client, u16* version)
{
    s32 ret = -1;
    u8 buf[8] = {GTP_REG_VERSION >> 8, GTP_REG_VERSION & 0xff};

    printk("////////////////////		%s:[%d]: Start \n", __func__, __LINE__); // add by leo
    GTP_DEBUG_FUNC();

    ret = gtp_i2c_read(client, buf, sizeof(buf));
    if (ret < 0)
    {
        GTP_ERROR("GTP read version failed");
        return ret;
    }

    if (version)
    {
        *version = (buf[7] << 8) | buf[6];
    }
    
    if (buf[5] == 0x00)
    {
        GTP_INFO("IC Version: %c%c%c_%02x%02x", buf[2], buf[3], buf[4], buf[7], buf[6]);
    }
    else
    {
        GTP_INFO("IC Version: %c%c%c%c_%02x%02x", buf[2], buf[3], buf[4], buf[5], buf[7], buf[6]);
    }
    return ret;
}

/*******************************************************
Function:
    I2c test Function.
Input:
    client:i2c client.
Output:
    Executive outcomes.
        2: succeed, otherwise failed.
*******************************************************/
static s8 gtp_i2c_test(struct i2c_client *client)
{
    u8 test[3] = {GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff};
    u8 retry = 0;
    s8 ret = -1;

    printk("////////////////////		%s:[%d]: Start \n", __func__, __LINE__); // add by leo
    GTP_DEBUG_FUNC();
  
    while(retry++ < 5)
    {
        ret = gtp_i2c_read(client, test, 3);
        if (ret > 0)
        {
            return ret;
        }
        GTP_ERROR("GTP i2c test failed time %d.",retry);
        msleep(10);
    }
    return ret;
}

/*******************************************************
Function:
    Request gpio(INT & RST) ports.
Input:
    ts: private data.
Output:
    Executive outcomes.
        >= 0: succeed, < 0: failed
*******************************************************/
static s8 gtp_request_io_port(struct goodix_ts_data *ts)
{
	int err=0;

	printk("////////////////////		%s:[%d]: Start \n", __func__, __LINE__); // add by leo

//add by leo for 1.8v ++
	//Elan POWER pin
	err = gpio_request(176, "ElanTouch-power");
	if (err < 0)
		printk("[gt927] %s: Failed to request GPIO%d (ElanTouch-power) error=%d\n", __func__, 176, err);

	err = gpio_direction_output(176, 1);
	if (err){
		printk("[gt927] %s: Failed to set power direction, error=%d\n", __func__, err);
		gpio_free(176);
	}
//add by leo for 1.8v --
	
	touch_chip->intr_gpio=GTP_INT_PORT;
	touch_chip->rst_gpio=GTP_RST_PORT;

	//init INTERRUPT pin
	err = gpio_request(touch_chip->intr_gpio, "GoodixTouch-irq");
	if(err < 0)
		printk("[gt927] %s: Failed to request GPIO%d (GoodixTouch-interrupt), error=%d", __func__, touch_chip->intr_gpio, err);

	err = gpio_direction_input(touch_chip->intr_gpio);
	if (err){
		printk("[gt927] %s: Failed to set (GoodixTouch-interrupt) input direction, error=%d", __func__, err);
		gpio_free(touch_chip->intr_gpio);
	}

	touch_chip->irq = gpio_to_irq(touch_chip->intr_gpio);
	printk("[gt927] %s: irq=%d\n", __func__, touch_chip->irq);

	//init RESET pin
	err = gpio_request(touch_chip->rst_gpio, "GoodixTouch-reset");
	if (err < 0)
		printk("[gt927] %s: Failed to request GPIO%d (ElanTouch-reset) error=%d", __func__, touch_chip->rst_gpio, err);

	err = gpio_direction_output(touch_chip->rst_gpio, 0);
	if (err){
		printk("[gt927] %s: Failed to set (GoodixTouch-reset) output direction, error=%d\n", __func__, err);
		gpio_free(touch_chip->rst_gpio);
	}

	 gtp_reset_guitar(ts->client, 20);
	
    	return err;
}

/*******************************************************
Function:
    Request interrupt.
Input:
    ts: private data.
Output:
    Executive outcomes.
        0: succeed, -1: failed.
*******************************************************/
static s8 gtp_request_irq(struct goodix_ts_data *ts)
{
    s32 ret = -1;
    const u8 irq_table[] = GTP_IRQ_TAB;

    printk("////////////////////		%s:[%d]: Start \n", __func__, __LINE__); // add by leo
    GTP_DEBUG("INT trigger type:%x", touch_chip->int_trigger_type);
/*
    ret  = request_irq(touch_chip->client->irq, 
                       goodix_ts_irq_handler,
                       irq_table[touch_chip->int_trigger_type],
                       touch_chip->client->name,
                       touch_chip);
*/	
    printk("////////////////////		%s:[%d]: request_irq (touch_chip->irq=%d) \n", __func__, __LINE__,touch_chip->irq); // add by leo
    ret  = request_irq(touch_chip->irq, goodix_ts_irq_handler, IRQ_TYPE_EDGE_FALLING, touch_chip->client->name, touch_chip);	// add by leo
    if (ret)
    {
        GTP_ERROR("Request IRQ failed!ERRNO:%d.", ret);
        GTP_GPIO_AS_INPUT(GTP_INT_PORT);
        GTP_GPIO_FREE(GTP_INT_PORT);

        hrtimer_init(&touch_chip->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        touch_chip->timer.function = goodix_ts_timer_handler;
        hrtimer_start(&touch_chip->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
        return -1;
    }
    else 
    {
        gtp_irq_disable(touch_chip);
        touch_chip->use_irq = 1;
        return 0;
    }
}

/*******************************************************
Function:
    Request input device Function.
Input:
    ts:private data.
Output:
    Executive outcomes.
        0: succeed, otherwise: failed.
*******************************************************/
static s8 gtp_request_input_dev(struct goodix_ts_data *ts)
{
    s8 ret = -1;
    s8 phys[32];
#if GTP_HAVE_TOUCH_KEY
    u8 index = 0;
#endif

    printk("////////////////////		%s:[%d]: Start \n", __func__, __LINE__); // add by leo
    GTP_DEBUG_FUNC();
  
    touch_chip->input_dev = input_allocate_device();
    if (touch_chip->input_dev == NULL)
    {
        GTP_ERROR("Failed to allocate input device.");
        return -ENOMEM;
    }

    touch_chip->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
#if GTP_ICS_SLOT_REPORT
    __set_bit(INPUT_PROP_DIRECT, touch_chip->input_dev->propbit);
    input_mt_init_slots(touch_chip->input_dev, 255);
#else
    touch_chip->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif

#if GTP_HAVE_TOUCH_KEY
    for (index = 0; index < GTP_MAX_KEY_NUM; index++)
    {
        input_set_capability(touch_chip->input_dev,EV_KEY,touch_key_array[index]);  
    }
#endif

#if GTP_SLIDING_WAKEUP
    input_set_capability(touch_chip->input_dev, EV_KEY, KEY_POWER);
#endif 

#if GTP_CHANGE_X2Y
    GTP_SWAP(touch_chip->abs_x_max, touch_chip->abs_y_max);
#endif

    input_set_abs_params(touch_chip->input_dev, ABS_MT_POSITION_X, 0, touch_chip->abs_x_max, 0, 0);
    input_set_abs_params(touch_chip->input_dev, ABS_MT_POSITION_Y, 0, touch_chip->abs_y_max, 0, 0);
    input_set_abs_params(touch_chip->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(touch_chip->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);  
    input_set_abs_params(touch_chip->input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

    sprintf(phys, "input/ts");
    touch_chip->input_dev->name = goodix_ts_name;
    touch_chip->input_dev->phys = phys;
    touch_chip->input_dev->id.bustype = BUS_I2C;
    touch_chip->input_dev->id.vendor = 0xDEAD;
    touch_chip->input_dev->id.product = 0xBEEF;
    touch_chip->input_dev->id.version = 10427;
    
    ret = input_register_device(touch_chip->input_dev);
    if (ret)
    {
        GTP_ERROR("Register %s input device failed", touch_chip->input_dev->name);
        return -ENODEV;
    }
    
#ifdef CONFIG_HAS_EARLYSUSPEND
    touch_chip->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    touch_chip->early_suspend.suspend = goodix_ts_early_suspend;
    touch_chip->early_suspend.resume = goodix_ts_late_resume;
    register_early_suspend(&touch_chip->early_suspend);
#endif

    return 0;
}

/*******************************************************
Function:
    I2c probe.
Input:
    client: i2c device struct.
    id: device id.
Output:
    Executive outcomes. 
        0: succeed.
*******************************************************/
static int goodix_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    s32 ret = -1;
    //struct goodix_ts_data *ts;
    u16 version_info;

    printk("////////////////////		%s:[%d]: Probe Start \n", __func__, __LINE__); // add by leo

    GTP_DEBUG_FUNC();
    
    //do NOT remove these logs
    GTP_INFO("GTP Driver Version: %s", GTP_DRIVER_VERSION);
    GTP_INFO("GTP Driver Built@%s, %s", __TIME__, __DATE__);
    GTP_INFO("GTP I2C Address: 0x%02x", client->addr);

    i2c_connect_client = client;
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
    {
        GTP_ERROR("I2C check functionality failed.");
        return -ENODEV;
    }
    touch_chip = kzalloc(sizeof(*touch_chip), GFP_KERNEL);
    if (touch_chip == NULL)
    {
        GTP_ERROR("Alloc GFP_KERNEL memory failed.");
        return -ENOMEM;
    }
   
    memset(touch_chip, 0, sizeof(*touch_chip));
    INIT_WORK(&touch_chip->work, goodix_ts_work_func);
    touch_chip->client = client;
    i2c_set_clientdata(client, touch_chip);
    //ts->irq_lock = SPIN_LOCK_UNLOCKED; //mark by leo
    touch_chip->irq_lock = gt927_spin_lock; // add by leo
    touch_chip->gtp_rawdiff_mode = 0;

//add by leo for attribute ++
	ret = sysfs_create_group(&client->dev.kobj, &gt927_attr_group);
	if (ret){
		printk("////////////////////		%s:[%d]: Register sysfs Failed (%d) \n", __func__, __LINE__, ret);
	}
//add by leo for attribute --

    printk("////////////////////		%s:[%d]: Request IO \n", __func__, __LINE__); // add by leo

    ret = gtp_request_io_port(touch_chip);
    if (ret < 0)
    {
        GTP_ERROR("GTP request IO port failed.");
        kfree(touch_chip);
        return ret;
    }

    printk("////////////////////		%s:[%d]: I2C Test \n", __func__, __LINE__); // add by leo

    ret = gtp_i2c_test(client);
    if (ret < 0)
    {
        GTP_ERROR("I2C communication ERROR!");
    }

#if GTP_AUTO_UPDATE
    ret = gup_init_update_proc(touch_chip);
    if (ret < 0)
    {
        GTP_ERROR("Create update thread error.");
    }
#endif
    
    ret = gtp_init_panel(touch_chip);
    if (ret < 0)
    {
        GTP_ERROR("GTP init panel failed.");
    }

    printk("////////////////////		%s:[%d]: Input Dev Register \n", __func__, __LINE__); // add by leo

    ret = gtp_request_input_dev(touch_chip);
    if (ret < 0)
    {
        GTP_ERROR("GTP request input dev failed");
    }
    
    ret = gtp_request_irq(touch_chip); 
    if (ret < 0)
    {
        GTP_INFO("GTP works in polling mode.");
    }
    else
    {
        GTP_INFO("GTP works in interrupt mode.");
    }

    ret = gtp_read_version(client, &version_info);
    if (ret < 0)
    {
        GTP_ERROR("Read version failed.");
    }
    spin_lock_init(&touch_chip->irq_lock);
    //ts->irq_lock = SPIN_LOCK_UNLOCKED; // mark by leo
    touch_chip->irq_lock = gt927_spin_lock; // add by leo

    printk("////////////////////		%s:[%d]: Enable IRQ \n", __func__, __LINE__); // add by leo

//add by leo for testtest ++
    gtp_reset_guitar(touch_chip->client, 150);
//    gtp_send_cfg(touch_chip->client);
//    msleep(100);
//add by leo for testtest --

    gtp_irq_enable(touch_chip);	

#if GTP_CREATE_WR_NODE
    init_wr_node(client);
#endif

#if GTP_ESD_PROTECT
    INIT_DELAYED_WORK(&gtp_esd_check_work, gtp_esd_check_func);
    gtp_esd_check_workqueue = create_workqueue("gtp_esd_check");
    queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, GTP_ESD_CHECK_CIRCLE); 
#endif

    printk("////////////////////		%s:[%d]: Probe Finished \n", __func__, __LINE__); // add by leo
    return 0;
}


/*******************************************************
Function:
    Goodix touchscreen driver release function.
Input:
    client: i2c device struct.
Output:
    Executive outcomes. 0---succeed.
*******************************************************/
static int goodix_ts_remove(struct i2c_client *client)
{
    //struct goodix_ts_data *ts = i2c_get_clientdata(client);
    
    GTP_DEBUG_FUNC();
    
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&touch_chip->early_suspend);
#endif

#if GTP_CREATE_WR_NODE
    uninit_wr_node();
#endif

#if GTP_ESD_PROTECT
    destroy_workqueue(gtp_esd_check_workqueue);
#endif


    if (touch_chip) 
    {
        if (touch_chip->use_irq)
        {
            GTP_GPIO_AS_INPUT(GTP_INT_PORT);
            GTP_GPIO_FREE(GTP_INT_PORT);
            free_irq(client->irq, touch_chip);
        }
        else
        {
            hrtimer_cancel(&touch_chip->timer);
        }
    }   
    
    GTP_INFO("GTP driver removing...");
    i2c_set_clientdata(client, NULL);
    input_unregister_device(touch_chip->input_dev);
    kfree(touch_chip);

    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
/*******************************************************
Function:
    Early suspend function.
Input:
    h: early_suspend struct.
Output:
    None.
*******************************************************/
static void goodix_ts_early_suspend(struct early_suspend *h)
{
    //struct goodix_ts_data *ts;
    s8 ret = -1;    
    touch_chip = container_of(h, struct goodix_ts_data, early_suspend);
    
    GTP_DEBUG_FUNC();

#if GTP_ESD_PROTECT
    touch_chip->gtp_is_suspend = 1;
    cancel_delayed_work_sync(&gtp_esd_check_work);
#endif

#if GTP_SLIDING_WAKEUP
    ret = gtp_enter_doze(touch_chip);
#else
    if (touch_chip->use_irq)
    {
        gtp_irq_disable(touch_chip);
    }
    else
    {
        hrtimer_cancel(&touch_chip->timer);
    }
    ret = gtp_enter_sleep(touch_chip);
#endif 
    if (ret < 0)
    {
        GTP_ERROR("GTP early suspend failed.");
    }
    // to avoid waking up while not sleeping, delay 48 + 10ms to ensure reliability    
    msleep(58);   
}

/*******************************************************
Function:
    Late resume function.
Input:
    h: early_suspend struct.
Output:
    None.
*******************************************************/
static void goodix_ts_late_resume(struct early_suspend *h)
{
    //struct goodix_ts_data *ts;
    s8 ret = -1;
    touch_chip = container_of(h, struct goodix_ts_data, early_suspend);
    
    GTP_DEBUG_FUNC();
    
    ret = gtp_wakeup_sleep(touch_chip);

#if GTP_SLIDING_WAKEUP
    doze_enabled = 0;
#endif

    if (ret < 0)
    {
        GTP_ERROR("GTP later resume failed.");
    }

    if (touch_chip->use_irq)
    {
        gtp_irq_enable(touch_chip);
    }
    else
    {
        hrtimer_start(&touch_chip->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
    }

#if GTP_ESD_PROTECT
    touch_chip->gtp_is_suspend = 0;
    queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, GTP_ESD_CHECK_CIRCLE);
#endif
}
#endif

#if GTP_ESD_PROTECT
/*******************************************************
Function:
    Initialize external watchdog for esd protect
Input:
    client:  i2c device.
Output:
    result of i2c write operation. 
        1: succeed, otherwise: failed
*********************************************************/
s32 gtp_init_ext_watchdog(struct i2c_client *client)
{
    u8 opr_buffer[4] = {0x80, 0x40, 0xAA, 0xAA};
    GTP_DEBUG("Init external watchdog...");
    return gtp_i2c_write(client, opr_buffer, 4);
}

/*******************************************************
Function:
    Esd protect function.
    Added external watchdog by meta, 2013/03/07
Input:
    work: delayed work
Output:
    None.
*******************************************************/
static void gtp_esd_check_func(struct work_struct *work)
{
    s32 i;
    s32 ret = -1;
    //struct goodix_ts_data *ts = NULL;
    u8 test[4] = {0x80, 0x40};

    printk("////////////////////		%s:[%d]: Start \n", __func__, __LINE__); // add by leo
    GTP_DEBUG_FUNC();

    touch_chip = i2c_get_clientdata(i2c_connect_client);

    if (touch_chip->gtp_is_suspend || touch_chip->enter_update)
    {
        return;
    }
    
    for (i = 0; i < 3; i++)
    {
        ret = gtp_i2c_read(touch_chip->client, test, 4);
        
        GTP_DEBUG("0x8040 = 0x%02X, 0x8041 = 0x%02X", test[2], test[3]);
        if ((ret < 0))
        {
            // IC works abnormally..
            continue;
        }
        else
        { 
            if ((test[2] == 0xAA) || (test[3] != 0xAA))
            {
                // IC works abnormally..
                i = 3;
                break;  
            }
            else 
            {
                // IC works normally, Write 0x8040 0xAA
                test[2] = 0xAA; 
                gtp_i2c_write(touch_chip->client, test, 3);
                break;
            }
        }
    }
    if (i >= 3)
    {
        GTP_ERROR("IC Working ABNORMALLY, Resetting Guitar...");
        gtp_reset_guitar(touch_chip->client, 50);
    }

    if(!touch_chip->gtp_is_suspend)
    {
        queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work, GTP_ESD_CHECK_CIRCLE);
    }

    return;
}
#endif

static const struct i2c_device_id goodix_ts_id[] = {
    { GTP_I2C_NAME, 0 },
    { }
};

static struct i2c_driver goodix_ts_driver = {
    .probe      = goodix_ts_probe,
    .remove     = goodix_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend    = goodix_ts_early_suspend,
    .resume     = goodix_ts_late_resume,
#endif
    .id_table   = goodix_ts_id,
    .driver = {
        .name     = GTP_I2C_NAME,
        .owner    = THIS_MODULE,
    },
};

/*******************************************************    
Function:
    Driver Install function.
Input:
    None.
Output:
    Executive Outcomes. 0---succeed.
********************************************************/
static int __devinit goodix_ts_init(void)
{
    s32 ret;

    printk("////////////////////		%s:[%d]: Init Start \n", __func__, __LINE__); // add by leo

    GTP_DEBUG_FUNC();   
    GTP_INFO("GTP driver installing...");
    goodix_wq = create_singlethread_workqueue("goodix_wq");
    if (!goodix_wq)
    {
        GTP_ERROR("Creat workqueue failed.");
        return -ENOMEM;
    }
    ret = i2c_add_driver(&goodix_ts_driver);
    return ret; 
}

/*******************************************************    
Function:
    Driver uninstall function.
Input:
    None.
Output:
    Executive Outcomes. 0---succeed.
********************************************************/
static void __exit goodix_ts_exit(void)
{
    GTP_DEBUG_FUNC();
    GTP_INFO("GTP driver exited.");
    i2c_del_driver(&goodix_ts_driver);
    if (goodix_wq)
    {
        destroy_workqueue(goodix_wq);
    }
}

late_initcall(goodix_ts_init);
module_exit(goodix_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL");

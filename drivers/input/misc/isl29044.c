/* Isl29044 ps do not work,als data too low */
/******************************************************************************
 * isl29044.h - Linux kernel module for Intersil isl29044 ambient light sensor
 *				and proximity sensor
 *
 * Copyright 2008-2012 Intersil Inc..
 *
 * DESCRIPTION:
 *	- This is the linux driver for isl29044.
 *		Kernel version 3.0.8
 *
 * modification history
 * --------------------
 * v1.0   2010/04/06, Shouxian Chen(Simon Chen) create this file
 * v1.1   2012/06/05, Shouxian Chen(Simon Chen) modified for Android 4.0 and
 *			linux 3.0.8
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 ******************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/idr.h>
#include <linux/fs.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <asm/io.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/spinlock.h>
#include <asm/uaccess.h>
#include <linux/workqueue.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif
#include <misc/app_info.h>

#define ISL29044_WHO_AM_I_REG 0X00		/* device ID register address*/
#define ISL29044_WHO_AM_I_VALUE 0X2E	/* device ID value */
#define ISL29044_ADDR	0x44
#define	DEVICE_NAME		"isl29044"
#define	DRIVER_VERSION	"1.1"

#define ALS_EN_MSK		(1 << 0)
#define PS_EN_MSK		(1 << 1)

#define PS_POLL_TIME	100	 	/* unit is ms */

#define ISL29044_MAX_PROX_VALUE  255    /*  default prox max value */
#define ISL29044_MIN_PROX_VALUE  0
#define ISL29044_PROX_REG_FARAWAY 0X03  /* far away interrupt register */
#define ISL29044_PROX_REG_CLOSE  0x04   /* close interrupt register */
#define ISL29044_PROX_REG_DATA   0X08   /* prox data register */
#define ISL29044_PROX_REPORT_FARAWAY 1   /* when far away interrupt happened report this value */
#define ISL29044_PROX_REPORT_CLOSE   0   /* when close interrupt happened report this value */

#ifdef CONFIG_HUAWEI_KERNEL
#define TP_GLASS_TRANSMITTANCE	(10)	 	/* 10% for default */
#define MAX_PROX_THRESHOLD		220 	/* 220 for default */
#define MIN_PROX_THRESHOLD		210 	/* 210 for default */

/*use this to contrl the al sensor debug message*/
static int als_debug_mask;
module_param_named(als_debug, als_debug_mask, int,
		S_IRUGO | S_IWUSR | S_IWGRP);

#define ALS_DEBUG_MASK(x...) do {\
	if (als_debug_mask) \
		printk(KERN_DEBUG x);\
	} while (0)
	
/*use this to contrl the pr sensor debug message*/
static int ps_debug_mask;
module_param_named(ps_debug, ps_debug_mask, int,
		S_IRUGO | S_IWUSR | S_IWGRP);

#define PS_DEBUG_MASK(x...) do {\
	if (ps_debug_mask) \
		printk(KERN_DEBUG x);\
	} while (0)
#endif

/*use this to contrl the pr sensor error message*/
static int ps_error_mask = 1;
module_param_named(ps_error, ps_error_mask, int,
		S_IRUGO | S_IWUSR | S_IWGRP);

#define PS_ERROR_MASK(x...) do {\
	if (ps_error_mask) \
		printk(KERN_ERR x);\
	} while (0)

/* Each client has this additional data */
struct isl29044_data_t {
	struct i2c_client* client;
	u8 als_pwr_status;
	u8 ps_pwr_status;
	u8 ps_led_drv_cur;	/* led driver current, 0: 110mA, 1: 220mA */
	u8 als_range;		/* als range, 0: 125 Lux, 1: 2000Lux */
	u8 als_mode;		/* als mode, 0: Visible light, 1: IR light */
	u8 ps_lt;			/* ps low limit */
	u8 ps_ht;			/* ps high limit */
	int poll_delay;		/* poll delay set by hal */
	u8 show_als_raw;	/* show als raw data flag, used for debug */
	u8 show_ps_raw;	/* show als raw data flag, used for debug */
	struct timer_list als_timer;	/* als poll timer */
	struct timer_list ps_timer;	/* ps poll timer */
	spinlock_t als_timer_lock;
	spinlock_t ps_timer_lock;
	struct work_struct als_work;
	struct work_struct ps_work;
	struct input_dev *als_input_dev;
	struct input_dev *ps_input_dev;
	int last_ps;
	u8 als_range_using;		/* the als range using now */
	u8 als_pwr_before_suspend;
	u8 ps_pwr_before_suspend;
};

/* Do not scan isl29028 automatic */
static const unsigned short normal_i2c[] = {ISL29044_ADDR, I2C_CLIENT_END };

/* data struct for isl29044 device */
struct isl29044_data_t	isl29044_data = {
	.client = NULL,
	.als_pwr_status = 0,
	.ps_pwr_status = 0,
	.als_input_dev = NULL,
	.ps_input_dev = NULL
};

/* prox self adjust parameter */
static u32 pwnd_value = 120;
static u32 pwave_value = 30;
static u32 min_proximity_value = 71;

static void do_als_timer(unsigned long arg)
{
	struct isl29044_data_t *dev_dat;

	dev_dat = (struct isl29044_data_t *)arg;

#ifdef CONFIG_HUAWEI_KERNEL
	ALS_DEBUG_MASK("%s:als_pwr_status = %d \n",__func__,dev_dat->als_pwr_status);
#endif
	/* timer handler is atomic context, so don't need sinp_lock() */
	//spin_lock(&dev_dat->als_timer);
	if(dev_dat->als_pwr_status == 0)
	{
		//spin_unlock(&dev_dat->als_timer);
		return ;
	}
	//spin_unlock(&dev_dat->als_timer);

	/* start a work queue, I cannot do i2c oepration in timer context for
	   this context is atomic and i2c function maybe sleep. */
	schedule_work(&dev_dat->als_work);
}

static void do_ps_timer(unsigned long arg)
{
	struct isl29044_data_t *dev_dat;

	dev_dat = (struct isl29044_data_t *)arg;

#ifdef CONFIG_HUAWEI_KERNEL
	PS_DEBUG_MASK("%s:ps_pwr_status = %d \n",__func__,dev_dat->ps_pwr_status);
#endif
	if(dev_dat->ps_pwr_status == 0)
	{
		return ;
	}

	/* start a work queue, I cannot do i2c oepration in timer context for
	   this context is atomic and i2c function maybe sleep. */
	schedule_work(&dev_dat->ps_work);
}

static void do_als_work(struct work_struct *work)
{
	struct isl29044_data_t *dev_dat;
	int ret;
	static int als_dat;
	u8 show_raw_dat;
	int lux;
	u8 als_range;

	dev_dat = container_of(work, struct isl29044_data_t, als_work);

	spin_lock(&dev_dat->ps_timer_lock);
	show_raw_dat = dev_dat->show_als_raw;
	spin_unlock(&dev_dat->ps_timer_lock);

	als_range = dev_dat->als_range_using;
#ifdef CONFIG_HUAWEI_KERNEL
	ALS_DEBUG_MASK("%s: als_range_using = %d \n",__func__,als_range);
#endif

	ret = i2c_smbus_read_byte_data(dev_dat->client, 0x09);
#ifdef CONFIG_HUAWEI_KERNEL
	ALS_DEBUG_MASK("%s: als_data_lo = %d \n",__func__,ret);
#endif
	if(ret < 0) goto err_rd;
	als_dat = (u8)ret;

	ret = i2c_smbus_read_byte_data(dev_dat->client, 0x0a);
#ifdef CONFIG_HUAWEI_KERNEL
	ALS_DEBUG_MASK("%s: als_data_hi = %d \n",__func__,ret);
#endif
	if(ret < 0) goto err_rd;
	als_dat = als_dat + ( ((u8)ret & 0x0f) << 8 );
	if(als_range)
	{
#ifdef CONFIG_HUAWEI_KERNEL
		lux = ((als_dat * 2000) / 4096) * (TP_GLASS_TRANSMITTANCE);
#else
		lux = (als_dat * 2000) / 4096;
#endif		
	}
	else
	{
		lux = (als_dat * 125) / 4096;
	}
#ifdef CONFIG_HUAWEI_KERNEL
	ALS_DEBUG_MASK("%s: als_raw_data = %d, als_lux = %d\n",__func__,als_dat, lux);
#endif

	input_report_abs(dev_dat->als_input_dev, ABS_MISC, lux);
	input_sync(dev_dat->als_input_dev);
	if(show_raw_dat)
	{
		printk(KERN_INFO "now als raw data is = %d, LUX = %d\n", als_dat, lux);
	}

	/* restart timer */
	spin_lock(&dev_dat->als_timer_lock);
	if(dev_dat->als_pwr_status == 0)
	{
		spin_unlock(&dev_dat->als_timer_lock);
		return ;
	}
	dev_dat->als_timer.expires = jiffies + (HZ * dev_dat->poll_delay) / 1000;
	spin_unlock(&dev_dat->als_timer_lock);
	add_timer(&dev_dat->als_timer);

	return ;

err_rd:
	printk(KERN_ERR "Read ps sensor error, ret = %d\n", ret);
	return ;
}

/* prox self sdjust function */
static int prox_report_data(struct work_struct *work)
{
    int ret = -EINVAL;                    /* use to get return value */
    unsigned close_value_buff = 0;      /* close value buffer, avert close value overbrim */
    s32 close_value = 0;                /* close interrupt value */
    s32 far_away_value = 0;             /* far away interrupt value */
    s32 pdata = 0;                       /* prox data */
    
	if (NULL == work)
	{
		PS_ERROR_MASK("%s %d:error:input work = NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	
    pdata = i2c_smbus_read_byte_data(isl29044_data.client, ISL29044_PROX_REG_DATA);
    if (pdata < 0)                      /* read prox data failed */
    {
        PS_ERROR_MASK("%s %d:read prox data failed. pdata = 0x%02x\n", __func__, __LINE__, pdata);
        pdata = 200;
    }

    if (pdata + pwave_value < min_proximity_value)          /* self adjust */
    {
        min_proximity_value = pdata + pwave_value;
        close_value_buff = min_proximity_value + pwnd_value;/* reset close value */
        if (close_value_buff > ISL29044_MAX_PROX_VALUE)
        {
            close_value_buff = ISL29044_MAX_PROX_VALUE;
            min_proximity_value = ISL29044_MAX_PROX_VALUE - pwnd_value;
        }

        isl29044_data.ps_ht = close_value_buff;
        isl29044_data.ps_lt = min_proximity_value;

        /* write new upper and lower to register */
        ret = i2c_smbus_write_byte_data(isl29044_data.client, 
                                            ISL29044_PROX_REG_FARAWAY, isl29044_data.ps_lt);
        if (ret < 0)                                            /* write far away register error */
        {
            PS_ERROR_MASK("%s %d:ISL29044_PROX_REG_FARAWAY write error ret = %d\n", 
                            __func__, __LINE__, ret);
            return ret;
        }
        ret = i2c_smbus_write_byte_data(isl29044_data.client, 
                                            ISL29044_PROX_REG_CLOSE, isl29044_data.ps_ht);
        if (ret < 0)                                            /* write close register error */
        {
            PS_ERROR_MASK("%s %d:ISL29044_PROX_REG_CLOSE write error ret = %d\n", 
                            __func__, __LINE__, ret);
            return ret;
        }
        
    }

    far_away_value = i2c_smbus_read_byte_data(isl29044_data.client, ISL29044_PROX_REG_FARAWAY);
    if (far_away_value < 0)/* far away value read error */
    {
        PS_ERROR_MASK("%s %d:far_away_value read error ret = %d\n", 
                          __func__, __LINE__, far_away_value);
        return far_away_value;
    }
    
    close_value = i2c_smbus_read_byte_data(isl29044_data.client, ISL29044_PROX_REG_CLOSE);
    if (close_value < 0)/* close value read error */
    {
        PS_ERROR_MASK("%s %d:close read error ret = %d\n", __func__, __LINE__, close_value);
        return close_value;
    }
  
    if (pdata < far_away_value)/* far away interrupt */
    {
        /* set far away value 0 and set close register the nomal value */
        ret = i2c_smbus_write_byte_data(isl29044_data.client, 
                                        ISL29044_PROX_REG_FARAWAY, ISL29044_MIN_PROX_VALUE);
        if (ret < 0)/* write far away interrupt register failed */
        {
            PS_ERROR_MASK("%s %d:write register 0x%02x error!\n", __func__, __LINE__,
                                        ISL29044_PROX_REG_FARAWAY);
        }
        
        ret = i2c_smbus_write_byte_data(isl29044_data.client, 
                                        ISL29044_PROX_REG_CLOSE, isl29044_data.ps_ht);
        if (ret < 0)/* write close interrupt register failed */
        {
            PS_ERROR_MASK("%s %d:write register 0x%02x error!\n", __func__, __LINE__,
                                        ISL29044_PROX_REG_CLOSE);
        }
        
        input_report_abs(isl29044_data.ps_input_dev, ABS_DISTANCE, ISL29044_PROX_REPORT_FARAWAY);
    	input_sync(isl29044_data.ps_input_dev);
     }
	 
     if (pdata > close_value)/* close interrupt */
     {
        
        ret = i2c_smbus_write_byte_data(isl29044_data.client, 
                                            ISL29044_PROX_REG_FARAWAY, isl29044_data.ps_lt);
        if (ret < 0)/* write far away interrupt register failed */
        {
            PS_ERROR_MASK("%s %d:write register 0x%02x error!\n", __func__, __LINE__,
                                            ISL29044_PROX_REG_FARAWAY);
        }
        ret = i2c_smbus_write_byte_data(isl29044_data.client, 
                                            ISL29044_PROX_REG_CLOSE, ISL29044_MAX_PROX_VALUE);
        if (ret < 0)/* write close interrupt register failed */
        {
            PS_ERROR_MASK("%s %d:write register 0x%02x error!\n", __func__, __LINE__,
                                            ISL29044_PROX_REG_CLOSE);
        }
        
        input_report_abs(isl29044_data.ps_input_dev, ABS_DISTANCE, ISL29044_PROX_REPORT_CLOSE);
		input_sync(isl29044_data.ps_input_dev);
    }

    return 0;
}

static void do_ps_work(struct work_struct *work)
{
	struct isl29044_data_t *dev_dat;
	int last_ps;
	int ret;
	u8 show_raw_dat;

	dev_dat = container_of(work, struct isl29044_data_t, ps_work);

	spin_lock(&dev_dat->ps_timer_lock);
	show_raw_dat = dev_dat->show_ps_raw;
	spin_unlock(&dev_dat->ps_timer_lock);

	ret = i2c_smbus_read_byte_data(dev_dat->client, 0x02);
	if(ret < 0) goto err_rd;

	last_ps = dev_dat->last_ps;
	dev_dat->last_ps = (ret & 0x80) ? 0 : 1;

#ifdef CONFIG_HUAWEI_KERNEL
	/* 0 for far;1 for near */
	PS_DEBUG_MASK("%s:last_ps_state = %d,now_ps_state = %d \n",__func__,last_ps,dev_dat->last_ps);
#endif
	
	if(show_raw_dat)
	{
		ret = i2c_smbus_read_byte_data(dev_dat->client, 0x08);
		if(ret < 0) goto err_rd;
		printk(KERN_INFO "ps raw data = %d\n", ret);
	}
	
#ifdef CONFIG_HUAWEI_KERNEL
	ret = i2c_smbus_read_byte_data(dev_dat->client, 0x08);
	if(ret < 0){
		goto err_rd;
	}
	PS_DEBUG_MASK("%s:ps_raw_data = %d\n",__func__,ret);//show ps raw data 
#endif

	/* delete the code which check if interrupt register is being change */
	/* this check well bring hardware and software not synchronization*/
	ret = prox_report_data(work);
	if (ret < 0)
	{
		PS_ERROR_MASK("%s %d:prox report data error! ret = %d \n", __func__, __LINE__, ret);	
	}

	/* restart timer */
	spin_lock(&dev_dat->ps_timer_lock);
	if(dev_dat->ps_pwr_status == 0)
	{
		spin_unlock(&dev_dat->ps_timer_lock);
		return ;
	}
	dev_dat->ps_timer.expires = jiffies + (HZ * PS_POLL_TIME) / 1000;
	spin_unlock(&dev_dat->ps_timer_lock);
	add_timer(&dev_dat->ps_timer);

	return ;

err_rd:
	printk(KERN_ERR "Read als sensor error, ret = %d\n", ret);
	return ;
}

/* enable to run als */
static int set_sensor_reg(struct isl29044_data_t *dev_dat)
{
	u8 reg_dat[5];
	int i, ret;

	reg_dat[2] = 0x22;
	reg_dat[3] = dev_dat->ps_lt;
	reg_dat[4] = dev_dat->ps_ht;

	reg_dat[1] = 0x50;	/* set ps sleep time to 50ms */
	spin_lock(&dev_dat->als_timer_lock);
	if(dev_dat->als_pwr_status) reg_dat[1] |= 0x04;
	spin_unlock(&dev_dat->als_timer_lock);

	spin_lock(&dev_dat->ps_timer_lock);
	if(dev_dat->ps_pwr_status) reg_dat[1] |= 0x80;
	spin_unlock(&dev_dat->ps_timer_lock);

	if(dev_dat->als_mode) reg_dat[1] |= 0x01;
	if(dev_dat->als_range) reg_dat[1] |= 0x02;
	if(dev_dat->ps_led_drv_cur) reg_dat[1] |= 0x08;

	for(i = 2 ; i <= 4; i++)
	{
		ret = i2c_smbus_write_byte_data(dev_dat->client, i, reg_dat[i]);
		if(ret < 0) return ret;
	}

	ret = i2c_smbus_write_byte_data(dev_dat->client, 0x01, reg_dat[1]);
	if(ret < 0) return ret;

	return 0;
}

/* set power status */
static int set_als_pwr_st(u8 state, struct isl29044_data_t *dat)
{
	int ret = 0;

	if(state)
	{
		spin_lock(&dat->als_timer_lock);
		if(dat->als_pwr_status)
		{
			spin_unlock(&dat->als_timer_lock);
			return ret;
		}
		dat->als_pwr_status = 1;
		spin_unlock(&dat->als_timer_lock);
		ret = set_sensor_reg(dat);
		if(ret < 0)
		{
			printk(KERN_ERR "set light sensor reg error, ret = %d\n", ret);
			return ret;
		}

		/* start timer */
		dat->als_timer.function = &do_als_timer;
		dat->als_timer.data = (unsigned long)dat;
		spin_lock(&dat->als_timer_lock);
		dat->als_timer.expires = jiffies + (HZ * dat->poll_delay) / 1000;
		spin_unlock(&dat->als_timer_lock);

		dat->als_range_using = dat->als_range;
		add_timer(&dat->als_timer);
	}
	else
	{
		spin_lock(&dat->als_timer_lock);
		if(dat->als_pwr_status == 0)
		{
			spin_unlock(&dat->als_timer_lock);
			return ret;
		}
		dat->als_pwr_status = 0;
		spin_unlock(&dat->als_timer_lock);
		ret = set_sensor_reg(dat);

		/* delete timer */
		del_timer_sync(&dat->als_timer);
	}

	return ret;
}

static int set_ps_pwr_st(u8 state, struct isl29044_data_t *dat)
{
	int ret = 0;

	if(state)
	{
		spin_lock(&dat->ps_timer_lock);
		if(dat->ps_pwr_status)
		{
			spin_unlock(&dat->ps_timer_lock);
			return ret;
		}
		dat->ps_pwr_status = 1;
		spin_unlock(&dat->ps_timer_lock);

		dat->last_ps = -1;
		ret = set_sensor_reg(dat);
		if(ret < 0)
		{
			printk(KERN_ERR "set proximity sensor reg error, ret = %d\n", ret);
			return ret;
		}
		/*when enable, report a faraway first to avoid mmi can't test porx*/
		input_report_abs(isl29044_data.ps_input_dev, ABS_DISTANCE, ISL29044_PROX_REPORT_FARAWAY);
		/* start timer */
		dat->ps_timer.function = &do_ps_timer;
		dat->ps_timer.data = (unsigned long)dat;
		dat->ps_timer.expires = jiffies + (HZ * PS_POLL_TIME) / 1000;
		add_timer(&dat->ps_timer);
	}
	else
	{
		spin_lock(&dat->ps_timer_lock);
		if(dat->ps_pwr_status == 0)
		{
			spin_unlock(&dat->ps_timer_lock);
			return ret;
		}
		dat->ps_pwr_status = 0;
		spin_unlock(&dat->ps_timer_lock);

		ret = set_sensor_reg(dat);

		/* delete timer */
		del_timer_sync(&dat->ps_timer);
	}

	return ret;
}

/* device attribute */
/* enable als attribute */
static ssize_t show_enable_als_sensor(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct isl29044_data_t *dat;
	u8 pwr_status;

	dat = (struct isl29044_data_t *)dev->platform_data;
	spin_lock(&dat->als_timer_lock);
	pwr_status = dat->als_pwr_status;
	spin_unlock(&dat->als_timer_lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", pwr_status);
}
static ssize_t store_enable_als_sensor(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct isl29044_data_t *dat;
	ssize_t ret;
	unsigned long val;

	dat = (struct isl29044_data_t *)dev->platform_data;

	val = simple_strtoul(buf, NULL, 10);
	ret = set_als_pwr_st(val, dat);

	if(ret == 0) ret = count;
	return ret;
}
static DEVICE_ATTR(enable_als_sensor, S_IWUGO|S_IRUGO, show_enable_als_sensor,
	store_enable_als_sensor);

/* enable ps attribute */
static ssize_t show_enable_ps_sensor(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct isl29044_data_t *dat;
	u8 pwr_status;

	dat = (struct isl29044_data_t *)dev->platform_data;
	spin_lock(&dat->ps_timer_lock);
	pwr_status = dat->ps_pwr_status;
	spin_unlock(&dat->ps_timer_lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", pwr_status);
}
static ssize_t store_enable_ps_sensor(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct isl29044_data_t *dat;
	ssize_t ret;
	unsigned long val;

	dat = (struct isl29044_data_t *)dev->platform_data;

	val = simple_strtoul(buf, NULL, 10);
	ret = set_ps_pwr_st(val, dat);

	if(ret == 0) ret = count;
	return ret;
}
static DEVICE_ATTR(enable_ps_sensor, S_IWUGO|S_IRUGO, show_enable_ps_sensor,
	store_enable_ps_sensor);

/* ps led driver current attribute */
static ssize_t show_ps_led_drv(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct isl29044_data_t *dat;

	dat = (struct isl29044_data_t *)dev->platform_data;
	return snprintf(buf, PAGE_SIZE, "%d\n", dat->ps_led_drv_cur);
}
static ssize_t store_ps_led_drv(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct isl29044_data_t *dat;
	int val;

	if(sscanf(buf, "%d", &val) != 1)
	{
		return -EINVAL;
	}

	dat = (struct isl29044_data_t *)dev->platform_data;
	if(val) dat->ps_led_drv_cur = 1;
	else dat->ps_led_drv_cur = 0;

	return count;
}
static DEVICE_ATTR(ps_led_driver_current, S_IWUGO|S_IRUGO, show_ps_led_drv,
	store_ps_led_drv);

/* als range attribute */
static ssize_t show_als_range(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct isl29044_data_t *dat;
	u8 range;

	dat = (struct isl29044_data_t *)dev->platform_data;
	spin_lock(&dat->als_timer_lock);
	range = dat->als_range;
	spin_unlock(&dat->als_timer_lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", range);
}
static ssize_t store_als_range(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct isl29044_data_t *dat;
	int val;

	if(sscanf(buf, "%d", &val) != 1)
	{
		return -EINVAL;
	}

	dat = (struct isl29044_data_t *)dev->platform_data;

	spin_lock(&dat->als_timer_lock);
	if(val) dat->als_range = 1;
	else dat->als_range = 0;
	spin_unlock(&dat->als_timer_lock);

	return count;
}
static DEVICE_ATTR(als_range, S_IWUGO|S_IRUGO, show_als_range, store_als_range);

/* als mode attribute */
static ssize_t show_als_mode(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct isl29044_data_t *dat;

	dat = (struct isl29044_data_t *)dev->platform_data;
	return snprintf(buf, PAGE_SIZE, "%d\n", dat->als_mode);
}
static ssize_t store_als_mode(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct isl29044_data_t *dat;
	int val;

	if(sscanf(buf, "%d", &val) != 1)
	{
		return -EINVAL;
	}

	dat = (struct isl29044_data_t *)dev->platform_data;
	if(val) dat->als_mode = 1;
	else dat->als_mode = 0;

	return count;
}
static DEVICE_ATTR(als_mode, S_IWUGO|S_IRUGO, show_als_mode, store_als_mode);

/* ps limit range attribute */
static ssize_t show_ps_limit(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct isl29044_data_t *dat;

	dat = (struct isl29044_data_t *)dev->platform_data;
	return snprintf(buf, PAGE_SIZE, "%d %d\n", dat->ps_lt, dat->ps_ht);
}
static ssize_t store_ps_limit(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct isl29044_data_t *dat;
	int lt, ht;

	if(sscanf(buf, "%d %d", &lt, &ht) != 2)
	{
		return -EINVAL;
	}

	dat = (struct isl29044_data_t *)dev->platform_data;

	if(lt > 255) dat->ps_lt = 255;
	else if(lt < 0) dat->ps_lt = 0;
	else  dat->ps_lt = lt;

	if(ht > 255) dat->ps_ht = 255;
	else if(ht < 0) dat->ps_ht = 0;
	else  dat->ps_ht = ht;

	return count;
}
static DEVICE_ATTR(ps_limit, S_IWUGO|S_IRUGO, show_ps_limit, store_ps_limit);

/* poll delay attribute */
static ssize_t show_poll_delay (struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct isl29044_data_t *dat;
	int delay;

	dat = (struct isl29044_data_t *)dev->platform_data;
	spin_lock(&dat->als_timer_lock);
	delay = dat->poll_delay;
	spin_unlock(&dat->als_timer_lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", delay);
}
static ssize_t store_poll_delay (struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct isl29044_data_t *dat;
	unsigned long ns;
	int delay;

	if(sscanf(buf, "%ld", &ns) != 1)
	{
		return -EINVAL;
	}
	//printk("ns=%ld\n",ns);
	delay = ns / 1000 / 1000;

	dat = (struct isl29044_data_t *)dev->platform_data;

	spin_lock(&dat->als_timer_lock);
	if(delay  < 120) dat->poll_delay = 120;
	else if(delay > 65535) dat->poll_delay = 65535;
	else dat->poll_delay = delay;
	spin_unlock(&dat->als_timer_lock);

	return count;
}
static DEVICE_ATTR(poll_delay, S_IWUGO|S_IRUGO, show_poll_delay,
	store_poll_delay);

/* show als raw data attribute */
static ssize_t show_als_show_raw (struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct isl29044_data_t *dat;
	u8 flag;

	dat = (struct isl29044_data_t *)dev->platform_data;
	spin_lock(&dat->als_timer_lock);
	flag = dat->show_als_raw;
	spin_unlock(&dat->als_timer_lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", flag);
}
static ssize_t store_als_show_raw (struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct isl29044_data_t *dat;
	int flag;

	if(sscanf(buf, "%d", &flag) != 1)
	{
		return -EINVAL;
	}

	dat = (struct isl29044_data_t *)dev->platform_data;

	spin_lock(&dat->als_timer_lock);
	if(flag == 0) dat->show_als_raw = 0;
	else dat->show_als_raw = 1;
	spin_unlock(&dat->als_timer_lock);

	return count;
}
static DEVICE_ATTR(als_show_raw, S_IWUGO|S_IRUGO, show_als_show_raw,
	store_als_show_raw);


/* show ps raw data attribute */
static ssize_t show_ps_show_raw (struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	struct isl29044_data_t *dat;
	u8 flag;

	dat = (struct isl29044_data_t *)dev->platform_data;
	spin_lock(&dat->als_timer_lock);
	
    dat->show_ps_raw = i2c_smbus_read_byte_data(dat->client, ISL29044_PROX_REG_DATA);
	flag = dat->show_ps_raw;
	spin_unlock(&dat->als_timer_lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", flag);
}
static ssize_t store_ps_show_raw (struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct isl29044_data_t *dat;
	int flag;

	if(sscanf(buf, "%d", &flag) != 1)
	{
		return -EINVAL;
	}

	dat = (struct isl29044_data_t *)dev->platform_data;

	spin_lock(&dat->als_timer_lock);
	if(flag == 0) dat->show_ps_raw = 0;
	else dat->show_ps_raw = 1;
	spin_unlock(&dat->als_timer_lock);

	return count;
}
static DEVICE_ATTR(ps_show_raw, S_IWUGO|S_IRUGO, show_ps_show_raw,
	store_ps_show_raw);

static struct attribute *als_attr[] = {
	&dev_attr_enable_als_sensor.attr,
	&dev_attr_als_range.attr,
	&dev_attr_als_mode.attr,
	&dev_attr_poll_delay.attr,
	&dev_attr_als_show_raw.attr,
	NULL
};

static struct attribute_group als_attr_grp = {
//	.name = "light sensor",
	.attrs = als_attr
};

static struct attribute *ps_attr[] = {
	&dev_attr_enable_ps_sensor.attr,
	&dev_attr_ps_led_driver_current.attr,
	&dev_attr_ps_limit.attr,
	&dev_attr_ps_show_raw.attr,
	NULL
};

static struct attribute_group ps_attr_grp = {
//	.name = "proximity sensor",
	.attrs = ps_attr
};


/* initial and register a input device for sensor */
static int init_input_dev(struct isl29044_data_t *dev_dat)
{
	int err;
	struct input_dev *als_dev;
	struct input_dev *ps_dev;

	als_dev = input_allocate_device();
	if (!als_dev)
	{
		return -ENOMEM;
	}

	ps_dev = input_allocate_device();
	if (!ps_dev)
	{
		err = -ENOMEM;
		goto err_free_als;
	}

	als_dev->name = "lightsensor-level";
	als_dev->id.bustype = BUS_I2C;
	als_dev->id.vendor  = 0x0001;
	als_dev->id.product = 0x0001;
	als_dev->id.version = 0x0100;
	als_dev->evbit[0] = BIT_MASK(EV_ABS);
	als_dev->absbit[BIT_WORD(ABS_MISC)] |= BIT_MASK(ABS_MISC);
	als_dev->dev.platform_data = &isl29044_data;
	input_set_abs_params(als_dev, ABS_MISC, 0, 2000, 0, 0);

	ps_dev->name = "proximity";
	ps_dev->id.bustype = BUS_I2C;
	ps_dev->id.vendor  = 0x0001;
	ps_dev->id.product = 0x0002;
	ps_dev->id.version = 0x0100;
	ps_dev->evbit[0] = BIT_MASK(EV_ABS);
	ps_dev->absbit[BIT_WORD(ABS_DISTANCE)] |= BIT_MASK(ABS_DISTANCE);
	ps_dev->dev.platform_data = &isl29044_data;
	input_set_abs_params(ps_dev, ABS_DISTANCE, 0, 1, 0, 0);

	err = input_register_device(als_dev);
	if (err)
	{
		goto err_free_ps;
	}

	err = input_register_device(ps_dev);
	if (err)
	{
		goto err_free_ps;
	}

#if 0
	/* register device attribute */
	err = device_create_file(&als_dev->dev, &dev_attr_enable_als_sensor);
	if (err) goto err_free_ps;

	err = device_create_file(&ps_dev->dev, &dev_attr_enable_ps_sensor);
	if (err) goto err_rm_als_en_attr;

	err = device_create_file(&ps_dev->dev, &dev_attr_ps_led_driver_current);
	if (err) goto err_rm_ps_en_attr;

	err = device_create_file(&als_dev->dev, &dev_attr_als_range);
	if (err) goto err_rm_ps_led_attr;

	err = device_create_file(&als_dev->dev, &dev_attr_als_mode);
	if (err) goto err_rm_als_range_attr;

	err = device_create_file(&ps_dev->dev, &dev_attr_ps_limit);
	if (err) goto err_rm_als_mode_attr;

	err = device_create_file(&als_dev->dev, &dev_attr_poll_delay);
	if (err) goto err_rm_ps_limit_attr;

	err = device_create_file(&als_dev->dev, &dev_attr_als_show_raw);
	if (err) goto err_rm_poll_delay_attr;

	err = device_create_file(&ps_dev->dev, &dev_attr_ps_show_raw);
	if (err) goto err_rm_als_show_raw_attr;
#endif

	err = sysfs_create_group(&als_dev->dev.kobj, &als_attr_grp);
	if (err) {
		dev_err(&als_dev->dev, "isl29044: device create als file failed\n");
		goto err_free_ps;
	}

	err = sysfs_create_group(&ps_dev->dev.kobj, &ps_attr_grp);
	if (err) {
		dev_err(&ps_dev->dev, "isl29044: device create ps file failed\n");
		goto err_free_ps;
	}

	dev_dat->als_input_dev = als_dev;
	dev_dat->ps_input_dev = ps_dev;

	return 0;

#if 0
err_rm_als_show_raw_attr:
	device_remove_file(&als_dev->dev, &dev_attr_als_show_raw);
err_rm_poll_delay_attr:
	device_remove_file(&als_dev->dev, &dev_attr_poll_delay);
err_rm_ps_limit_attr:
	device_remove_file(&ps_dev->dev, &dev_attr_ps_limit);
err_rm_als_mode_attr:
	device_remove_file(&als_dev->dev, &dev_attr_als_mode);
err_rm_als_range_attr:
	device_remove_file(&als_dev->dev, &dev_attr_als_range);
err_rm_ps_led_attr:
	device_remove_file(&ps_dev->dev, &dev_attr_ps_led_driver_current);
err_rm_ps_en_attr:
	device_remove_file(&ps_dev->dev, &dev_attr_enable_ps_sensor);
err_rm_als_en_attr:
	device_remove_file(&als_dev->dev, &dev_attr_enable_als_sensor);
#endif

err_free_ps:
	input_free_device(ps_dev);
err_free_als:
	input_free_device(als_dev);

	return err;
}

struct sensor_regulator {
	struct regulator *vreg;
	const char *name;
	u32	min_uV;
	u32	max_uV;
};

struct sensor_regulator isl29044_acc_vreg[] = {
	{NULL, "vdd", 1700000, 3600000},
	{NULL, "vddio", 1700000, 3600000},
};

static int isl29044_acc_config_regulator(struct isl29044_data_t *acc, bool on)
{
	int rc = 0, i;
	int num_reg = sizeof(isl29044_acc_vreg) / sizeof(struct sensor_regulator);

	if (on) {
		for (i = 0; i < num_reg; i++) {
			isl29044_acc_vreg[i].vreg =
				regulator_get(&acc->client->dev,
				isl29044_acc_vreg[i].name);
			if (IS_ERR(isl29044_acc_vreg[i].vreg)) {
				rc = PTR_ERR(isl29044_acc_vreg[i].vreg);
				pr_err("%s:regulator get failed rc=%d\n",
								__func__, rc);
				isl29044_acc_vreg[i].vreg = NULL;
				goto error_vdd;
			}

			if (regulator_count_voltages(
				isl29044_acc_vreg[i].vreg) > 0) {
				rc = regulator_set_voltage(
					isl29044_acc_vreg[i].vreg,
					isl29044_acc_vreg[i].min_uV,
					isl29044_acc_vreg[i].max_uV);
				if (rc) {
					pr_err("%s: set voltage failed rc=%d\n",
					__func__, rc);
					regulator_put(isl29044_acc_vreg[i].vreg);
					isl29044_acc_vreg[i].vreg = NULL;
					goto error_vdd;
				}
			}

			rc = regulator_enable(isl29044_acc_vreg[i].vreg);
			if (rc) {
				pr_err("%s: regulator_enable failed rc =%d\n",
					__func__, rc);
				if (regulator_count_voltages(
					isl29044_acc_vreg[i].vreg) > 0) {
					regulator_set_voltage(
						isl29044_acc_vreg[i].vreg, 0,
						isl29044_acc_vreg[i].max_uV);
				}
				regulator_put(isl29044_acc_vreg[i].vreg);
				isl29044_acc_vreg[i].vreg = NULL;
				goto error_vdd;
			}
		}
		return rc;
	} else {
		i = num_reg;
	}

error_vdd:
	while (--i >= 0) {
		if (!IS_ERR_OR_NULL(isl29044_acc_vreg[i].vreg)) {
			if (regulator_count_voltages(
			isl29044_acc_vreg[i].vreg) > 0) {
				regulator_set_voltage(isl29044_acc_vreg[i].vreg,
						0, isl29044_acc_vreg[i].max_uV);
			}
			regulator_disable(isl29044_acc_vreg[i].vreg);
			regulator_put(isl29044_acc_vreg[i].vreg);
			isl29044_acc_vreg[i].vreg = NULL;
		}
	}
	return rc;
}

/* Return 0 if detection is successful, -ENODEV otherwise */
static int isl29044_detect(struct i2c_client *client,
	struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;

	printk(KERN_DEBUG "In isl29044_detect()\n");
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WRITE_BYTE_DATA
				     | I2C_FUNC_SMBUS_READ_BYTE))
	{
		printk("I2c adapter don't support ISL29044\n");
		return -ENODEV;
	}

	/* probe that if isl29044 is at the i2 address */
	if (i2c_smbus_xfer(adapter, client->addr, 0,I2C_SMBUS_WRITE,
		0,I2C_SMBUS_QUICK,NULL) < 0)
		{
		printk("I2c smbus don't support ISL29044\n");
		return -ENODEV;
}
	strlcpy(info->type, "isl29044", I2C_NAME_SIZE);
	printk("%s is found at i2c device address %d\n",
		info->type, client->addr);

	return 0;
}

/**
 * isl29044_get_device_id - get the device's board id
 * @dev_dat: Handle to device data struct
 * 
 * read device id register and comput device id, returning device id 
 * or a error number
 */

static s32 isl29044_get_device_id(struct isl29044_data_t *dev_dat)
{
	s32 ret = 0;
	s32 who_am_i = -EINVAL;
	
	if (NULL == dev_dat)
	{
		return -EINVAL;
	}
	
	ret = i2c_smbus_read_byte_data(dev_dat->client, ISL29044_WHO_AM_I_REG);
	if (ret < 0)
	{
		dev_err(&(dev_dat->client->dev),"%s %d:Read WHO AM I failed!\n", __func__, __LINE__);
		return ret;	
	}

	who_am_i = ret >> 2;/* from second byte to seventh byte are used to storage the board id  */
	who_am_i &= 0x0000003F;
	dev_info(&(dev_dat->client->dev),"%s %d:device id:%d\n",__func__, __LINE__, who_am_i);
	
	return who_am_i;
}
/* ******************************************************************
    Name:
    Description:this function get data from dtsi file 
                and initialization the  variable
    input:dev
    output:none
    return:success return 0, failed return error code
* ***************************************************************** */
static int isl29044_parse_dt(struct device *dev)
{
    int ret = -EINVAL;
    u32 temp_value = 0;
    struct device_node *isl29044_node = dev->of_node;

	if (NULL == dev)
	{
		PS_ERROR_MASK("%s %d:input dev = NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

    /* get min_proximity_value initialization value */
    ret = of_property_read_u32(isl29044_node, "intersil,min_proximity_value", &temp_value);
	if (ret != 0) 
    {
		PS_ERROR_MASK("%s %d:Unable to read min_proximity_value\n", __func__, __LINE__);
		return ret;
	} 
    else 
    {
		min_proximity_value = temp_value;
	}

    ret = of_property_read_u32(isl29044_node, "intersil,pwnd_value", &temp_value);
    if (ret < 0)
    {
        PS_ERROR_MASK("%s %d:Unable to read pwnd_value\n", __func__, __LINE__);
        return ret;
    } 
    else 
    {
        pwnd_value = temp_value;
    }

    ret = of_property_read_u32(isl29044_node, "intersil,pwave_value", &temp_value);
    if (ret < 0)
    {
        PS_ERROR_MASK("%s %d:Unable to read pwave_value\n", __func__, __LINE__);
        return ret;    
    }
    else   
    {
        pwave_value = temp_value;
    }

    ret = of_property_read_u32(isl29044_node, "intersil,interrupt_upper", &temp_value);
    if (ret < 0)
    {
        PS_ERROR_MASK("%s %d:Unable to read interrupt_upper\n", __func__, __LINE__);
        return ret;    
    }
    else   
    {
        isl29044_data.ps_ht = temp_value;
    }

    ret = of_property_read_u32(isl29044_node, "intersil,interrupt_lower", &temp_value);
    if (ret < 0)
    {
        PS_ERROR_MASK("%s %d:Unable to read interrupt_lower\n", __func__, __LINE__);
        return ret;    
    }
    else   
    {
        isl29044_data.ps_lt = temp_value;
    }
	
    return 0;
}

/* isl29044 probed */
static int isl29044_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int err, i;
	u8 reg_dat[8];
    int device_id = -EINVAL;
    
	printk(KERN_DEBUG "In isl29044_probe()\n");

	/* initial device data struct */
	isl29044_data.client = client;
	isl29044_data.als_pwr_status = 0;
	isl29044_data.ps_pwr_status = 0;
	isl29044_data.ps_led_drv_cur = 0;
	isl29044_data.als_range = 1;
	isl29044_data.als_mode = 0;
#ifdef CONFIG_HUAWEI_KERNEL
	isl29044_data.ps_lt = MIN_PROX_THRESHOLD;
	isl29044_data.ps_ht = MAX_PROX_THRESHOLD;
#else
	isl29044_data.ps_lt = 120;
	isl29044_data.ps_ht = 150;
#endif
	isl29044_data.poll_delay = 200;
	isl29044_data.show_als_raw = 0;
	isl29044_data.show_ps_raw = 0;
    err = isl29044_parse_dt(&(client->dev));
    if (err < 0)
    {
         PS_ERROR_MASK("%s %d:isl29044_parse_dt failed!\n", __func__, __LINE__);
         return err;
    }

	spin_lock_init(&isl29044_data.als_timer_lock);
	spin_lock_init(&isl29044_data.ps_timer_lock);
	INIT_WORK(&isl29044_data.als_work, &do_als_work);
	INIT_WORK(&isl29044_data.ps_work, &do_ps_work);
	init_timer(&isl29044_data.als_timer);
	init_timer(&isl29044_data.ps_timer);

	i2c_set_clientdata(client,&isl29044_data);
	isl29044_acc_config_regulator(&isl29044_data, true);
	
	device_id = isl29044_get_device_id(&isl29044_data);
	if (device_id != ISL29044_WHO_AM_I_VALUE)
	{
		dev_err(&client->dev, "%s %d:driver and device not match driver:%d,device:%d\n",
                        __func__, __LINE__, ISL29044_WHO_AM_I_VALUE, device_id); 
		return device_id;
	}
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
    set_hw_dev_flag(DEV_I2C_APS);
    set_hw_dev_flag(DEV_I2C_L_SENSOR);
#endif
	dev_info(&client->dev,"%s %d:devices and driver matched!\n", __func__, __LINE__);
	err = app_info_set("LP-Sensor", DEVICE_NAME);
   	if (err < 0)/*faile to add app_info*/
	{
		dev_err(&client->dev,"%s %d:faile to add app_info\n", __func__, __LINE__);
	}
	/* initial isl29044 */
	err = set_sensor_reg(&isl29044_data);
	if(err < 0) return err;

	/* initial als interrupt limit to low = 0, high = 4095, so als cannot
	   trigger a interrupt. We use ps interrupt only */
	reg_dat[5] = 0x00;
	reg_dat[6] = 0xf0;
	reg_dat[7] = 0xff;
	for(i = 5; i <= 7; i++)
	{
		err = i2c_smbus_write_byte_data(client, i, reg_dat[i]);
		if(err < 0) return err;
	}
    
	/* Add input device register here */
	err = init_input_dev(&isl29044_data);
	if(err < 0)
	{
		return err;
	}

	return err;
}

static int isl29044_remove(struct i2c_client *client)
{
	struct input_dev *als_dev;
	struct input_dev *ps_dev;

	printk(KERN_INFO "%s at address %d is removed\n",client->name,client->addr);

	/* clean the isl29044 data struct when isl29044 device remove */
	isl29044_data.client = NULL;
	isl29044_data.als_pwr_status = 0;
	isl29044_data.ps_pwr_status = 0;

	als_dev = isl29044_data.als_input_dev;
	ps_dev = isl29044_data.ps_input_dev;

#if 0
	device_remove_file(&ps_dev->dev, &dev_attr_ps_show_raw);
	device_remove_file(&als_dev->dev, &dev_attr_als_show_raw);
	device_remove_file(&als_dev->dev, &dev_attr_poll_delay);
	device_remove_file(&ps_dev->dev, &dev_attr_ps_limit);
	device_remove_file(&als_dev->dev, &dev_attr_als_mode);
	device_remove_file(&als_dev->dev, &dev_attr_als_range);
	device_remove_file(&ps_dev->dev, &dev_attr_ps_led_driver_current);
	device_remove_file(&als_dev->dev, &dev_attr_enable_als_sensor);
	device_remove_file(&ps_dev->dev, &dev_attr_enable_ps_sensor);
#endif

	sysfs_remove_group(&als_dev->dev.kobj, &als_attr_grp);
	sysfs_remove_group(&ps_dev->dev.kobj, &ps_attr_grp);
	isl29044_acc_config_regulator(&isl29044_data, false);
	input_unregister_device(als_dev);
	input_unregister_device(ps_dev);

	isl29044_data.als_input_dev = NULL;
	isl29044_data.ps_input_dev = NULL;

	return 0;
}

#ifdef CONFIG_PM
/* if define power manager, define suspend and resume function */
static int isl29044_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct isl29044_data_t *dat;
	int ret;

	dat = i2c_get_clientdata(client);

	spin_lock(&dat->als_timer_lock);
	dat->als_pwr_before_suspend = dat->als_pwr_status;
	spin_unlock(&dat->als_timer_lock);
	ret = set_als_pwr_st(0, dat);
	if(ret < 0) return ret;

	spin_lock(&dat->ps_timer_lock);
	dat->ps_pwr_before_suspend = dat->ps_pwr_status;
	spin_unlock(&dat->ps_timer_lock);
	ret = set_ps_pwr_st(0, dat);
	if(ret < 0) return ret;

	return 0;
}

static int isl29044_resume(struct i2c_client *client)
{
	struct isl29044_data_t *dat;
	int ret;

	dat = i2c_get_clientdata(client);

	ret = set_als_pwr_st(dat->als_pwr_before_suspend, dat);
	if(ret < 0) return ret;

	ret = set_ps_pwr_st(dat->ps_pwr_before_suspend, dat);
	if(ret < 0) return ret;

	return 0;
}
#else
#define	isl29044_suspend 	NULL
#define isl29044_resume		NULL
#endif		/*ifdef CONFIG_PM end*/

static const struct i2c_device_id isl29044_id[] = {
	{ "isl29044", 0 },
	{ }
};

#ifdef CONFIG_OF
static struct of_device_id intersil_match_table[] = {
            { .compatible = "intersil,isl29044",},
            { },
    };
#else
#define intersil_match_table NULL
#endif

static struct i2c_driver isl29044_driver = {
	.driver = {
		.name	= "isl29044",
		.of_match_table = intersil_match_table,
	},
	.probe			= isl29044_probe,
	.remove			= isl29044_remove,
	.id_table		= isl29044_id,
	.detect			= isl29044_detect,
	.address_list	= normal_i2c,
	.suspend		= isl29044_suspend,
	.resume			= isl29044_resume
};

struct i2c_client *isl29044_client;

static int __init isl29044_init(void)
{
	int ret;

	/* register the i2c driver for isl29044 */
	ret = i2c_add_driver(&isl29044_driver);
	if(ret < 0) printk(KERN_ERR "Add isl29044 driver error, ret = %d\n", ret);
	printk(KERN_DEBUG "init isl29044 module\n");

	return ret;
}

static void __exit isl29044_exit(void)
{
	printk(KERN_DEBUG "exit isl29044 module\n");
	i2c_del_driver(&isl29044_driver);
}


MODULE_AUTHOR("Chen Shouxian");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("isl29044 ambient light sensor driver");
MODULE_VERSION(DRIVER_VERSION);

module_init(isl29044_init);
module_exit(isl29044_exit);

/* drivers/i2c/chips/akm09911.c - akm09911 and akm8962 compass driver
 *
 * Copyright (C) 2007-2008 HTC Corporation.
 * Author: Hou-Kun Chen <houkun.chen@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
 * Revised by AKM 2010/11/15
 *
 */
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/akm09911.h>
#include <linux/earlysuspend.h>
#include <mach/vreg.h>
#include <linux/sensors.h>
//#include <hsad/config_interface.h>
#include <linux/gpio_event.h>
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif
#include <misc/app_info.h>

//#define AKM09911_DEBUG		1
#define AKM09911_DEBUG_MSG	1
#define AKM09911_DEBUG_FUNC	0
#define AKM09911_DEBUG_DATA	0
#define MAX_FAILURE_COUNT	3
#define AKM09911_RETRY_COUNT 3
#define AKM09911_DEFAULT_DELAY	100
/*move it to hardware_self_adapt.h*/

#if AKM09911_DEBUG_MSG
#define AKMDBG(format, ...)	\
		printk(KERN_INFO "AKM09911 " format "\n", ## __VA_ARGS__)
#else
#define AKMDBG(format, ...)
#endif

#if AKM09911_DEBUG_FUNC
#define AKMFUNC(func) \
		printk(KERN_INFO "AKM09911 " func " is called\n")
#else
#define AKMFUNC(func)
#endif

/* add log control code */
static int akm09911_debug_mask = 1;
module_param_named(akm09911_debug_mask, akm09911_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);
#define AKM09911_ERROR(x...) do {\
    if (akm09911_debug_mask >= 1) \
        printk(KERN_ERR x);\
    } while (0)

#define AKM09911_INFO(x...) do {\
    if (akm09911_debug_mask >= 2) \
        printk(KERN_ERR x);\
    } while (0)

#define AKM09911_DEBUG(x...) do {\
    if (akm09911_debug_mask >= 3) \
        printk(KERN_ERR x);\
    } while (0)

/* workqueue for compass queue_work */
static struct workqueue_struct *compass_wq;

static struct i2c_client *this_client;

struct akm09911_data {
	struct input_dev *input_dev;
	struct work_struct work;
	struct early_suspend akm_early_suspend;
};

extern struct input_dev *sensor_dev;

/* Addresses to scan -- protected by sense_data_mutex */
static char sense_data[SENSOR_DATA_SIZE];
static struct mutex sense_data_mutex;
static DECLARE_WAIT_QUEUE_HEAD(data_ready_wq);
static DECLARE_WAIT_QUEUE_HEAD(open_wq);

static atomic_t data_ready;
static atomic_t open_count;
static atomic_t open_flag;
static atomic_t reserve_open_flag;

static atomic_t m_flag;
static atomic_t a_flag;
static atomic_t mv_flag;

/*save the value of auto-calibration*/
static short calibration_value=0;

static struct akm09911_data *this_akm = NULL;

static int failure_count;

static short akmd_delay = AKM09911_DEFAULT_DELAY;

//static atomic_t suspend_flag = ATOMIC_INIT(0);

/*delete one line*/

static int AKI2C_RxData(char *rxData, int length)
{
	uint8_t loop_i;
	struct i2c_msg msgs[] = {
		{
			.addr = this_client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = this_client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};
#if AKM09911_DEBUG_DATA
	int i;
	char addr = rxData[0];
#endif
#ifdef AKM09911_DEBUG
	/* Caller should check parameter validity.*/
	if ((rxData == NULL) || (length < 1)) {
		return -EINVAL;
	}
#endif
    /* if read faild, it will try again, 
     * the max try time is #AKM09911_RETRY_COUNT */
	for (loop_i = 0; loop_i < AKM09911_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(this_client->adapter, msgs, 2) > 0) {
			break;
		}
        
		mdelay(10);
	}

	if (loop_i >= AKM09911_RETRY_COUNT) {
		AKM09911_ERROR("%s retry over %d\n", __func__, AKM09911_RETRY_COUNT);
		return -EIO;
	}
#if AKM09911_DEBUG_DATA
	AKM09911_ERROR("%s:RxData: len=%02x, addr=%02x\n  data=",__func__, length, addr);
	for (i = 0; i < length; i++) {
		AKM09911_ERROR(" %02x", rxData[i]);
	}
    printk(KERN_INFO "\n");
#endif
	return 0;
}

static int AKI2C_TxData(char *txData, int length)
{
	uint8_t loop_i;
	struct i2c_msg msg[] = {
		{
			.addr = this_client->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};
#if AKM09911_DEBUG_DATA
	int i;
#endif
#ifdef AKM09911_DEBUG
	/* Caller should check parameter validity.*/
	if ((txData == NULL) || (length < 2)) {
		return -EINVAL;
	}
#endif
	for (loop_i = 0; loop_i < AKM09911_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(this_client->adapter, msg, 1) > 0) {
			break;
		}
		mdelay(10);
	}

	if (loop_i >= AKM09911_RETRY_COUNT) {
		AKM09911_ERROR("%s retry over %d\n", __func__, AKM09911_RETRY_COUNT);
		return -EIO;
	}
#if AKM09911_DEBUG_DATA
	AKM09911_INFO("TxData: len=%02x, addr=%02x\n  data=", length, txData[0]);
	for (i = 0; i < (length-1); i++) {
		AKM09911_INFO(" %02x", txData[i + 1]);
	}
	AKM09911_INFO("\n");
#endif
	return 0;
}

static int AKECS_SetMode_SngMeasure(void)
{
	char buffer[2];

	atomic_set(&data_ready, 0);

	/* Set measure mode */
	buffer[0] = AK09911_REG_CNTL2;
	buffer[1] = AK09911_MODE_SNG_MEASURE;  

	/* Set data */
	return AKI2C_TxData(buffer, 2);
}

static int AKECS_SetMode_SelfTest(void)
{
	char buffer[2];

	/* Set measure mode */
	buffer[0] = AK09911_REG_CNTL2;
	buffer[1] = AK09911_MODE_SELF_TEST; 
	/* Set data */
	return AKI2C_TxData(buffer, 2);
}

static int AKECS_SetMode_FUSEAccess(void)
{
	char buffer[2];

	/* Set measure mode */
	buffer[0] = AK09911_REG_CNTL2;
	buffer[1] = AK09911_MODE_FUSE_ACCESS;
	/* Set data */
	return AKI2C_TxData(buffer, 2);
}

static int AKECS_SetMode_PowerDown(void)
{
	char buffer[2];

	/* Set powerdown mode */
	buffer[0] = AK09911_REG_CNTL2;
	buffer[1] = AK09911_MODE_POWERDOWN;
	/* Set data */
	return AKI2C_TxData(buffer, 2);
}

static int AKECS_SetMode(char mode)
{
	int ret;

	switch (mode) {
	case AK09911_MODE_SNG_MEASURE:
		ret = AKECS_SetMode_SngMeasure();
		break;
	case AK09911_MODE_SELF_TEST:
		ret = AKECS_SetMode_SelfTest();
		break;
	case AK09911_MODE_FUSE_ACCESS:
		ret = AKECS_SetMode_FUSEAccess();
		break;
	case AK09911_MODE_POWERDOWN:
		ret = AKECS_SetMode_PowerDown();

		break;
	default:
		AKM09911_DEBUG("%s: Unknown mode(%d)", __func__, mode);
		return -EINVAL;
	}

	/* wait at least 100us after changing mode */
		udelay(100);

	return ret;
}

static int AKECS_CheckDevice(void)
{
	char buffer[2];
	int ret;

	/* Set measure mode */
	buffer[0] = AK09911_REG_WIA1;

	/* Read data */
	ret = AKI2C_RxData(buffer, 2);
	if (ret < 0) {
        AKM09911_ERROR("%s %d:I2c connect error!\n", __func__, __LINE__);
		return ret;
	}
    
	/* Check read data */
	if ((buffer[0] != AK09911_WIA1_VALUE)||(buffer[1] != AK09911_WIA2_VALUE)) {
        AKM09911_ERROR("%s %d:Check read data failed\n", __func__, __LINE__);
		return -ENXIO;
	}

	return 0;
}

static int AKECS_CheckChipName(void)
{
	char buffer[2];
	int ret;

	/* use WIA1 WIA2 to check AK09911*/
	buffer[0] = AK09911_REG_WIA1;
	/* Read data */
	ret = AKI2C_RxData(buffer, 2);
	if (ret < 0) {
		return ret;
	}
	/* modify AKM chip name for chip identify */
	if ((buffer[0] == AK09911_WIA1_VALUE)&&(buffer[1] == AK09911_WIA2_VALUE))
	{
		AKM09911_INFO("%s %d:My compass is AKM09911\n", __func__, __LINE__);
	}
	else 
	{
		AKM09911_ERROR("%s %d:It is not AKM0911C\n", __func__, __LINE__);
		return -ENXIO;
	}

	return ret;
}

static int AKECS_GetData_Poll(	uint8_t *rbuf,	int size)
{
	char buffer[SENSOR_DATA_SIZE];
	int err;

	/* Read status */
	buffer[0] = AK09911_REG_ST1;
	err = AKI2C_RxData( buffer, 1);
	if (err < 0) {
		AKM09911_ERROR("%s %d:AKI2C_RxData failed.", __func__, __LINE__);
		return err;
	}

	/* Check ST bit */
	if (!(AKM_DRDY_IS_HIGH(buffer[0])))
		return -EAGAIN;

	/* Read rest data */
	buffer[1] = AK09911_REG_ST1 + 1;
	err = AKI2C_RxData(&(buffer[1]), SENSOR_DATA_SIZE-1);
	if (err < 0) {
		AKM09911_ERROR("%s %d:AKI2C_RxData failed.", __func__, __LINE__);
		return err;
	}

	mutex_lock(&sense_data_mutex);
	memcpy(rbuf, buffer, size);
	atomic_set(&data_ready, 0);
	mutex_unlock(&sense_data_mutex);
	failure_count = 0;
	return 0;
}

static void AKECS_SetYPR(int *rbuf)
{
	struct akm09911_data *data = i2c_get_clientdata(this_client);

	AKM09911_DEBUG("AKM09911 %s:\n", __func__);
	AKM09911_DEBUG("  yaw =%6d, pitch =%6d, roll =%6d\n",
			rbuf[0], rbuf[1], rbuf[2]);
	AKM09911_DEBUG("  tmp =%6d, m_stat =%6d, g_stat =%6d\n",
			rbuf[3], rbuf[4], rbuf[5]);
	AKM09911_DEBUG("  Acceleration[LSB]: %6d,%6d,%6d\n",
			rbuf[6], rbuf[7], rbuf[8]);
	AKM09911_DEBUG("  Geomagnetism[LSB]: %6d,%6d,%6d\n",
			rbuf[9], rbuf[10], rbuf[11]);

	/* Report magnetic sensor information */
	if (atomic_read(&m_flag)) 
	{
		input_report_abs(data->input_dev, ABS_RX, rbuf[0]);
		input_report_abs(data->input_dev, ABS_RY, rbuf[1]);
		input_report_abs(data->input_dev, ABS_RZ, rbuf[2]);
		input_report_abs(data->input_dev, ABS_RUDDER, rbuf[4]);
		/* Gyroscope sensor */
		input_report_abs(data->input_dev, ABS_HAT2X, rbuf[12]);
		input_report_abs(data->input_dev, ABS_HAT2Y, rbuf[13]);
		input_report_abs(data->input_dev, ABS_HAT3X, rbuf[14]);
		input_report_abs(data->input_dev, ABS_HAT3Y, rbuf[5]);//FIX TO LEVEL 3
		/* Rotation Vector */
		input_report_abs(data->input_dev, ABS_TILT_X, rbuf[15]);
		input_report_abs(data->input_dev, ABS_TILT_Y, rbuf[16]);
		input_report_abs(data->input_dev, ABS_TOOL_WIDTH, rbuf[17]);
		input_report_abs(data->input_dev, ABS_VOLUME, rbuf[18]);

		/*LinearAcc*/
		input_report_abs(data->input_dev, ABS_DISTANCE, rbuf[19]);
		input_report_abs(data->input_dev, ABS_PRESSURE, rbuf[20]);
		input_report_abs(data->input_dev, ABS_MISC, rbuf[21]);

		/*Gravity*/
		input_report_abs(data->input_dev, ABS_GAS, rbuf[22]);
		input_report_abs(data->input_dev, ABS_WHEEL, rbuf[23]);
		input_report_abs(data->input_dev, ABS_THROTTLE, rbuf[24]);
	}
	/* acc sensor data neednt be reported by compass,so we don't handle it*/

	/* Report magnetic vector information */
	if (atomic_read(&mv_flag)) 
	{
		input_report_abs(data->input_dev, ABS_HAT0X, rbuf[9]);
		input_report_abs(data->input_dev, ABS_HAT0Y, rbuf[10]);
		input_report_abs(data->input_dev, ABS_BRAKE, rbuf[11]);
	}

	input_sync(data->input_dev);
}
static int AKECS_GetOpenStatus(void)
{
	wait_event_interruptible(open_wq, (atomic_read(&open_flag) != 0));
	return atomic_read(&open_flag);
}

static int AKECS_GetCloseStatus(void)
{
	wait_event_interruptible(open_wq, (atomic_read(&open_flag) <= 0));
	return atomic_read(&open_flag);
}

static void AKECS_CloseDone(void)
{
	atomic_set(&m_flag, 1);
	atomic_set(&a_flag, 1);
	atomic_set(&mv_flag, 1);
}

/***** akm_aot functions ***************************************/
static int akm_aot_open(struct inode *inode, struct file *file)
{
	int ret = -1;
	/*
	*int atomic_cmpxchg(atomic_t *v, int old, int new)
	*{
	*	int ret;
	*	unsigned long flags;
	*	spin_lock_irqsave(ATOMIC_HASH(v), flags);
	*
	*	ret = v->counter;
	*	if (likely(ret == old))
	*		v->counter = new;
	*
	*	spin_unlock_irqrestore(ATOMIC_HASH(v), flags);
	*	return ret;
	*}
	*/

	AKM09911_INFO("%s %d:akm_aot_open", __func__, __LINE__);
	if (atomic_cmpxchg(&open_count, 0, 1) == 0) {
		if (atomic_cmpxchg(&open_flag, 0, 1) == 0) {
			atomic_set(&reserve_open_flag, 1);
			wake_up(&open_wq);
			ret = 0;
		}
	}
	return ret;
}

static int akm_aot_release(struct inode *inode, struct file *file)
{
	AKM09911_INFO("%s %d:akm_aot_release", __func__, __LINE__);
	atomic_set(&reserve_open_flag, 0);
	atomic_set(&open_flag, 0);
	atomic_set(&open_count, 0);
	wake_up(&open_wq);
	return 0;
}

static long
akm_aot_ioctl(struct file *file,
			  unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	short flag;

	switch (cmd) {
	case ECS_IOCTL_APP_SET_MFLAG:
	case ECS_IOCTL_APP_SET_AFLAG:
	case ECS_IOCTL_APP_SET_MVFLAG:
		if (copy_from_user(&flag, argp, sizeof(flag))) {
			return -EFAULT;
		}
		if (flag < 0 || flag > 1) {
			return -EINVAL;
		}
		break;
	case ECS_IOCTL_APP_SET_DELAY:
		if (copy_from_user(&flag, argp, sizeof(flag))) {
			return -EFAULT;
		}
		break;
	default:
		break;
	}

	/*get the value of auto-calibration*/
	switch (cmd) {
	case ECS_IOCTL_APP_SET_MFLAG:
		atomic_set(&m_flag, flag);
		AKM09911_DEBUG("%s %d:MFLAG is set to %d", __func__, __LINE__, flag);
		break;
	case ECS_IOCTL_APP_GET_MFLAG:
		flag = atomic_read(&m_flag);
		break;
	case ECS_IOCTL_APP_SET_AFLAG:
		atomic_set(&a_flag, flag);
		AKM09911_DEBUG("%s %d:AFLAG is set to %d", __func__, __LINE__, flag);
		break;
	case ECS_IOCTL_APP_GET_AFLAG:
		flag = atomic_read(&a_flag);
		break;
	case ECS_IOCTL_APP_SET_MVFLAG:
		atomic_set(&mv_flag, flag);
		AKM09911_DEBUG("%s %d:MVFLAG is set to %d", __func__, __LINE__, flag);
		break;
	case ECS_IOCTL_APP_GET_MVFLAG:
		flag = atomic_read(&mv_flag);
		break;
	case ECS_IOCTL_APP_SET_DELAY:
		akmd_delay = flag;
		AKM09911_DEBUG("%s %d:Delay is set to %d", __func__, __LINE__, flag);
		break;
	case ECS_IOCTL_APP_GET_DELAY:
		flag = akmd_delay;
		break;
	case ECS_IOCTL_APP_GET_DEVID:
	    break;
	case ECS_IOCTL_APP_GET_CAL:
		flag=calibration_value;
		break;
	default:
		return -ENOTTY;
	}

	switch (cmd) {
	case ECS_IOCTL_APP_GET_MFLAG:
	case ECS_IOCTL_APP_GET_AFLAG:
	case ECS_IOCTL_APP_GET_MVFLAG:
	case ECS_IOCTL_APP_GET_DELAY:
	case ECS_IOCTL_APP_GET_CAL:
		if (copy_to_user(argp, &flag, sizeof(flag))) {
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_APP_GET_DEVID:
		
		if (copy_to_user(argp, AKM09911_I2C_NAME, strlen(AKM09911_I2C_NAME)+1))
			return -EFAULT;
		break;
	default:
		break;
	}

	return 0;
}

/***** akmd functions ********************************************/
static int akmd_open(struct inode *inode, struct file *file)
{
	AKM09911_DEBUG("%s %d:akmd_open", __func__, __LINE__);
	return nonseekable_open(inode, file);
}

static int akmd_release(struct inode *inode, struct file *file)
{
	AKM09911_DEBUG("akmd_release");
	AKECS_CloseDone();
	return 0;
}
#define AKM_YPR_DATA_SIZE		25  //19
static long
akmd_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg)
{
	void __user *argp = (void __user *)arg;

	/* NOTE: In this function the size of "char" should be 1-byte. */
	char sData[SENSOR_DATA_SIZE];/* for GETDATA */
	char rwbuf[RWBUF_SIZE];		/* for READ/WRITE */
	int32_t ypr_buf[AKM_YPR_DATA_SIZE];		/* for SET_YPR */
	char mode;			/* for SET_MODE*/
	//int value[12];	/* for SET_YPR */
	short delay;		/* for GET_DELAY */
	int status;			/* for OPEN/CLOSE_STATUS */
	int ret = -1;		/* Return value. */
	int slide = 0;
	/*AKMDBG("%s (0x%08X).", __func__, cmd);*/

	/*set the value of auto-calibration*/
	switch (cmd) {
	case ECS_IOCTL_WRITE:
	case ECS_IOCTL_READ:
		if (argp == NULL) {
			AKMDBG("invalid argument.");
			return -EINVAL;
		}
		if (copy_from_user(&rwbuf, argp, sizeof(rwbuf))) {
			AKMDBG("copy_from_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_SET_MODE:
		if (argp == NULL) {
			AKMDBG("invalid argument.");
			return -EINVAL;
		}
		if (copy_from_user(&mode, argp, sizeof(mode))) {
			AKMDBG("copy_from_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_SET_YPR:
		if (argp == NULL) {
			AKMDBG("invalid argument.");
			return -EINVAL;
		}
		if (copy_from_user(&ypr_buf, argp, sizeof(ypr_buf))) {
			AKMDBG("copy_from_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_SET_CAL:
		if (argp == NULL) {
			AKMDBG("invalid argument.");
			return -EINVAL;
		}
		if (copy_from_user(&calibration_value, argp, sizeof(calibration_value))) {
			AKMDBG("copy_from_user failed.");
			return -EFAULT;
		}
		printk(KERN_INFO "ECS_IOCTL_SET_CAL   calibration_value=%d\n",calibration_value);
		break;
	/*add ioctl cmd*/
	case ECS_IOCTL_APP_GET_SLIDE:
		if (argp == NULL) {
			AKMDBG("invalid argument.");
			return -EINVAL;
		}
		break;
	default:
		break;
	}

	switch (cmd) {
	case ECS_IOCTL_WRITE:
		AKMFUNC("IOCTL_WRITE");
		if ((rwbuf[0] < 2) || (rwbuf[0] > (RWBUF_SIZE-1))) {
			AKMDBG("invalid argument.");
			return -EINVAL;
		}
		ret = AKI2C_TxData(&rwbuf[1], rwbuf[0]);
		if (ret < 0) {
			return ret;
		}
		break;
	case ECS_IOCTL_READ:
		AKMFUNC("IOCTL_READ");
		if ((rwbuf[0] < 1) || (rwbuf[0] > (RWBUF_SIZE-1))) {
			AKMDBG("invalid argument.");
			return -EINVAL;
		}
		ret = AKI2C_RxData(&rwbuf[1], rwbuf[0]);
		if (ret < 0) {
			return ret;
		}
		break;
	case ECS_IOCTL_SET_MODE:
		AKMFUNC("IOCTL_SET_MODE");
		ret = AKECS_SetMode(mode);
		if (ret < 0) {
			return ret;
		}
		break;
	case ECS_IOCTL_GETDATA:
		AKMFUNC("IOCTL_GET_DATA");
		ret = AKECS_GetData_Poll(sData, SENSOR_DATA_SIZE); //  AK09911 can only use polling mode 
		if (ret < 0) {
			return ret;
		}
		break;
	case ECS_IOCTL_SET_YPR:
		AKECS_SetYPR(ypr_buf);
		break;
	case ECS_IOCTL_GET_OPEN_STATUS:
		AKMFUNC("IOCTL_GET_OPEN_STATUS");
		status = AKECS_GetOpenStatus();
		AKMDBG("AKECS_GetOpenStatus returned (%d)", status);
		break;
	case ECS_IOCTL_GET_CLOSE_STATUS:
		AKMFUNC("IOCTL_GET_CLOSE_STATUS");
		status = AKECS_GetCloseStatus();
		AKMDBG("AKECS_GetCloseStatus returned (%d)", status);
		break;
	case ECS_IOCTL_GET_DELAY:
		AKMFUNC("IOCTL_GET_DELAY");
		delay = akmd_delay;
		break;
	/*add ioctl cmd*/
	case ECS_IOCTL_APP_GET_SLIDE:  
	//	slide = get_slide_pressed();
		AKMFUNC("GET_SLIDE \n");
		break;
	case ECS_IOCTL_SET_CAL:
		break;
	default:
		return -ENOTTY;
	}

	switch (cmd) {
	case ECS_IOCTL_READ:
		if (copy_to_user(argp, &rwbuf, rwbuf[0]+1)) {
			AKMDBG("copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GETDATA:
		if (copy_to_user(argp, &sData, sizeof(sData))) {
			AKMDBG("copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_OPEN_STATUS:
	case ECS_IOCTL_GET_CLOSE_STATUS:
		if (copy_to_user(argp, &status, sizeof(status))) {
			AKMDBG("copy_to_user failed.");
			return -EFAULT;
		}
		break;
	case ECS_IOCTL_GET_DELAY:
		if (copy_to_user(argp, &delay, sizeof(delay))) {
			AKMDBG("copy_to_user failed.");
			return -EFAULT;
		}
		break;
	/*add ioctl cmd*/
	case ECS_IOCTL_APP_GET_SLIDE:  
		if (copy_to_user(argp, &slide,sizeof(slide))) {
			AKMDBG("copy_to_user failed.");
			return -EFAULT;
		}
		break;
	default:
		break;
	}

	return 0;
}

static void akm09911_work_func(struct work_struct *work)
{
	char buffer[SENSOR_DATA_SIZE];
	int ret;

	memset(buffer, 0, SENSOR_DATA_SIZE);
	buffer[0] = AK09911_REG_ST1;
	ret = AKI2C_RxData(buffer, SENSOR_DATA_SIZE);
	if (ret < 0) {
		printk(KERN_ERR "AKM09911 akm09911_work_func: I2C failed\n");
		goto WORK_FUNC_END;
	}
    
	/* Check ST bit */
	if ((buffer[0] & 0x01) != 0x01) {
		printk(KERN_ERR "AKM09911 akm09911_work_func: ST is not set\n");
		goto WORK_FUNC_END;
	}

	mutex_lock(&sense_data_mutex);
	memcpy(sense_data, buffer, SENSOR_DATA_SIZE);
	atomic_set(&data_ready, 1);
	wake_up(&data_ready_wq);
	mutex_unlock(&sense_data_mutex);

WORK_FUNC_END:
	enable_irq(this_client->irq);

	AKMFUNC("akm09911_work_func");
}

#if 0
static irqreturn_t akm09911_interrupt(int irq, void *dev_id)
{
	struct akm09911_data *data = dev_id;
	AKMFUNC("akm09911_interrupt");
	disable_irq_nosync(this_client->irq);
/* use compass_wq instead of system_wq */
	queue_work(compass_wq, &data->work);
	return IRQ_HANDLED;
}

static void akm09911_early_suspend(struct early_suspend *handler)
{
	AKMFUNC("akm09911_early_suspend");
	atomic_set(&suspend_flag, 1);
	atomic_set(&reserve_open_flag, atomic_read(&open_flag));
	atomic_set(&open_flag, 0);
	wake_up(&open_wq);
//	disable_irq(this_client->irq);
	AKMDBG("suspended with flag=%d",atomic_read(&reserve_open_flag));
}

static void akm09911_early_resume(struct early_suspend *handler)
{
	AKMFUNC("akm09911_early_resume");
//	enable_irq(this_client->irq);
	atomic_set(&suspend_flag, 0);
	atomic_set(&open_flag, atomic_read(&reserve_open_flag));
	wake_up(&open_wq);
	AKMDBG("resumed with flag=%d",atomic_read(&reserve_open_flag));
}
#endif

/*********************************************/
static struct file_operations akmd_fops = {
	.owner = THIS_MODULE,
	.open = akmd_open,
	.release = akmd_release,
	.unlocked_ioctl = akmd_ioctl,
};

static struct file_operations akm_aot_fops = {
	.owner = THIS_MODULE,
	.open = akm_aot_open,
	.release = akm_aot_release,
	.unlocked_ioctl = akm_aot_ioctl,
};

static struct miscdevice akmd_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "akm09911_dev",
	.fops = &akmd_fops,
};

static struct miscdevice akm_aot_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "compass_aot",
	.fops = &akm_aot_fops,
};

/*modify Int_gpio for akm09911 */
/*Modifying reset pin configuration to set as direction output */
static int akm09911_config_gpio_pin(struct akm09911_platform_data *pdata)
{
	int err, ret;

	/*RST*/
	err = gpio_request(pdata->rst_gpio, "akm09911_rst");
	if (err)
	{
		pr_err("%s, %d: gpio_request failed for akm09911_rst\n", __func__, __LINE__);
		return err;
	}

	err = gpio_tlmm_config(GPIO_CFG(pdata->rst_gpio,0,
							GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if(err < 0) 
	{
		gpio_free(pdata->rst_gpio);
		pr_err("%s: Fail set gpio as OUTPUT,PULL_UP=%d\n",__func__,pdata->rst_gpio);
		return err;
	}
	gpio_direction_output(pdata->rst_gpio, 0);
	mdelay(1);
	gpio_direction_output(pdata->rst_gpio, 1);
	ret = gpio_get_value(pdata->rst_gpio);
	printk(KERN_INFO "Reset = %d at %s \n",ret,__func__);
	return 0;
}
static int akm09911_parse_dt(struct device *dev, struct akm09911_platform_data *pdata) 
{
    
    struct device_node *np = dev->of_node;
    u32 temp_val = 0;
    int rc = 0;
    
    /* init reset gpio */
    rc = of_property_read_u32(np, "akm,rst_gpio", &temp_val);
    if (rc)
    {
        printk(KERN_ERR "Unable to read rst_gpio\n");
        return rc;
    }
    else
    {
        pdata->rst_gpio = (int)temp_val;
    }
    
    /* get chip pisition */
    rc = of_property_read_u32(np, "akm,chip_pisition", &temp_val);
    if (rc)
    {
        printk(KERN_ERR "Unable to read chip_pisition\n");
        return rc;
    }
    else
    {
        pdata->chip_position = (int)temp_val;
    }

    rc = of_property_read_u32(np, "akm,virtual_gyro_flag", &temp_val);
    if (rc)
    {
        printk(KERN_ERR "Unable to read virtual_gyro_flag\n");
        return rc;
    }
    else
    {
        pdata->virtual_gyro_flag = (int)temp_val;
    }

    return 0;
}

/*********************************************/
int akm09911_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct akm09911_data *akm = NULL;
	int err = 0;
	//int gpio_config;
	int ret = -1;
	struct akm09911_platform_data *pdata = NULL;

    AKM09911_ERROR("%s %d:Compass akm09911 probe begin.", __FUNCTION__, __LINE__);
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		AKM09911_ERROR("AKM09911 akm09911_probe: check_functionality failed.\n");
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

    /* if we use DTS to create device, we get init data from dts file */
    /* if not, we get init data from client->dev.platform_data */
    if (client->dev.of_node) 
    {
        pdata = kzalloc(sizeof(struct akm09911_platform_data), GFP_KERNEL);
        if (NULL == pdata)
        {
            AKM09911_ERROR("%s %d:Kzalloc failed!\n", __FUNCTION__, __LINE__);
            ret = -ENOMEM;
            goto exit_alloc_data_failed;
        }
        
        memset(pdata, 0, sizeof(struct akm09911_platform_data));
        ret = akm09911_parse_dt(&client->dev, pdata);
        if (ret)
        {
            AKM09911_ERROR("%s %d:Init platform data failed\n", __FUNCTION__, __LINE__);
            goto exit_init_platform_data;
        }
    }
    else
    {
        if (client->dev.platform_data)
        {
            pdata = (struct akm09911_platform_data *)client->dev.platform_data;
        }
        else
        {
            AKM09911_ERROR("%s %d:Init platform data failed\n", __FUNCTION__, __LINE__);
            goto exit_alloc_data_failed;
        }
    }

    akm09911_config_gpio_pin(pdata);
	/* Allocate memory for driver data */
	akm = kzalloc(sizeof(struct akm09911_data), GFP_KERNEL);
	if (!akm) {
		AKM09911_ERROR("AKM09911 akm09911_probe: memory allocation failed.\n");
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

    this_akm = akm;
	INIT_WORK(&akm->work, akm09911_work_func);
	i2c_set_clientdata(client, akm);

	this_client = client;

	/* Check connection */
	err = AKECS_CheckDevice();
	if (err < 0) {
		AKM09911_ERROR("AKM09911 akm09911_probe: set power down mode error\n");
		goto exit_check_dev_id;
	}

	err = AKECS_CheckChipName();
	if (err < 0) {
		AKM09911_ERROR("AKM09911 akm09911_probe: fail check chip id\n");
		goto exit_check_dev_id;
	}

	/* Declare input device */
	akm->input_dev = sensor_dev;
	if (akm->input_dev == NULL) {
		err = -ENOMEM;
		AKM09911_ERROR("akm09911_probe: Failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	/* we neednt input_allocate_device() */
	
	/* Setup input device */
	set_bit(EV_ABS, akm->input_dev->evbit);
	/* yaw (0, 360) */
	input_set_abs_params(akm->input_dev, ABS_RX, 0, 23040, 3, 3);
	/* pitch (-180, 180) */
	input_set_abs_params(akm->input_dev, ABS_RY, -11520, 11520, 3, 3);
	/* roll (-90, 90) */
	input_set_abs_params(akm->input_dev, ABS_RZ, -5760, 5760, 3, 3);
	/* temparature Not using*/
	/* 
	input_set_abs_params(akm->input_dev, ABS_THROTTLE, -30, 85, 0, 0);
	 */
	/* status of magnetic sensor */
	input_set_abs_params(akm->input_dev, ABS_RUDDER, -32768, 3, 0, 0);
	/* status of acceleration sensor */
	//input_set_abs_params(akm->input_dev, ABS_WHEEL, -32768, 3, 3, 3);
	/* x-axis of raw magnetic vector (-8188, 8188) */
	input_set_abs_params(akm->input_dev, ABS_HAT0X, -32768, 32767, 3, 3);
	/* y-axis of raw magnetic vector (-8188, 8188) */
	input_set_abs_params(akm->input_dev, ABS_HAT0Y, -32768, 32767, 3, 3);
	/* z-axis of raw magnetic vector (-8188, 8188) */
	input_set_abs_params(akm->input_dev, ABS_BRAKE, -32768, 32767, 3, 3);

	/* Gyroscope 2000 deg/sec =34.90 rad/sec in Q16 format,Q16 means it has enlarge 65536 times*/
	input_set_abs_params(akm->input_dev,ABS_HAT2X,-2287638, 2287638, 3, 3);
	input_set_abs_params(akm->input_dev,ABS_HAT2Y,-2287638, 2287638, 3, 3);
	input_set_abs_params(akm->input_dev,ABS_HAT3X,-2287638, 2287638, 3, 3);
	/* status of PseudoGyro sensor */
	input_set_abs_params(akm->input_dev,ABS_HAT3Y,0, 3, 0, 0);

	/* Rotation Vector [-1,+1] in Q16 format */
	input_set_abs_params(akm->input_dev, ABS_TILT_X,-65536, 65536, 3, 3);
	input_set_abs_params(akm->input_dev, ABS_TILT_Y,-65536, 65536, 3, 3);
	input_set_abs_params(akm->input_dev, ABS_VOLUME,-65536, 65536, 3, 3);
	input_set_abs_params(akm->input_dev, ABS_TOOL_WIDTH,-65536, 65536, 3, 3);

	/* Gravity (2G = 19.61 m/s2 in Q16) */
	input_set_abs_params(akm->input_dev, ABS_GAS, -1285377, 1285377, 3, 3);
	input_set_abs_params(akm->input_dev, ABS_WHEEL, -1285377, 1285377, 3, 3);
	input_set_abs_params(akm->input_dev, ABS_THROTTLE, -1285377, 1285377, 3, 3);

	/* Linear acceleration (16G = 156.9 m/s2 in Q16) */
	input_set_abs_params(akm->input_dev, ABS_DISTANCE, -10283017, 10283017, 3, 3);
	input_set_abs_params(akm->input_dev, ABS_PRESSURE, -10283017, 10283017, 3, 3);
	input_set_abs_params(akm->input_dev, ABS_MISC, -10283017, 10283017, 3, 3);
	/* Set name */
	//akm->input_dev->name = "compass";
	/* Register input device,if there is no G-sensor to register this struct*/
	/*err = input_register_device(akm->input_dev);
	if (err) {
		printk(KERN_ERR "AKM09911 akm09911_probe: "
				"Unable to register input device\n");
		goto exit_input_register_fail;
	}*/
	err = misc_register(&akmd_device);
	if (err) {
		AKM09911_ERROR("AKM09911 akm09911_probe:akmd_device register failed\n");
		goto exit_akmd_device_register_failed;
	}

	err = misc_register(&akm_aot_device);
	if (err) {
		AKM09911_ERROR("AKM09911 akm09911_probe:akm_aot_device register failed\n");
		goto exit_akm_aot_device_register_failed;
	}

    /* create compass workqueue */
	compass_wq = create_singlethread_workqueue("compass_wq");

	if (!compass_wq) 
	{
		err = -ENOMEM;
		AKM09911_ERROR("%s, line %d: create_singlethread_workqueue fail!\n", __func__, __LINE__);
		goto exit_akm_create_workqueue_failed;
	}

	mutex_init(&sense_data_mutex);

	init_waitqueue_head(&data_ready_wq);
	init_waitqueue_head(&open_wq);

	/* As default, report all information */
	/*modify the default values*/
	atomic_set(&m_flag, 0);
	atomic_set(&a_flag, 0);
	atomic_set(&mv_flag, 0);
#ifdef CONFIG_HAS_EARLYSUSPEND
	akm->akm_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	akm->akm_early_suspend.suspend = akm09911_early_suspend;
	akm->akm_early_suspend.resume = akm09911_early_resume;
	register_early_suspend(&akm->akm_early_suspend);
#endif
	err = set_sensor_input(AKM, akm->input_dev->dev.kobj.name);
	if (err) {
		AKM09911_ERROR("%s set_sensor_input failed\n", __func__);
		goto exit_akm_aot_device_register_failed;
	}
	/*if(virtual_gyro_is_supported())
		set_sensors_list(MGY_SENSOR);*/
	/*set_sensors_list(M_SENSOR);*/
	#ifdef CONFIG_HUAWEI_HW_DEV_DCT
	/* detect current device successful, set the flag as present */
	set_hw_dev_flag(DEV_I2C_COMPASS);
	#endif

    err = app_info_set("M-Sensor", "AKM09911");
    
	if (err) 
	{
		AKM09911_ERROR("%s, line %d:set compass AKM8963 app_info error\n", __func__, __LINE__);
	}	

    if(1 == pdata->virtual_gyro_flag)
	{
		err = app_info_set("V-Gyro-Sensor", "1");
	}
	else
	{
		err = app_info_set("V-Gyro-Sensor", "0");
	}
    
	if (err) 
	{
		pr_info("%s, line %d:set AKM V-Gyro app_info error\n", __func__, __LINE__);
	}
    
	AKM09911_ERROR("%s %d:Compass akm09911 is successfully probed.", __FUNCTION__, __LINE__);
	return 0;

/*add compass exception process*/
exit_akm_create_workqueue_failed:
exit_akm_aot_device_register_failed:
	misc_deregister(&akmd_device);
exit_akmd_device_register_failed:
	//input_unregister_device(akm->input_dev);
/*exit_input_register_fail:
	input_free_device(akm->input_dev);*/
exit_input_dev_alloc_failed:
//	free_irq(client->irq, akm);
//exit_request_irq:
exit_check_dev_id:
//exit_check_platform_data:
exit_init_platform_data:
	kfree(akm);
exit_alloc_data_failed:
exit_check_functionality_failed:

	return err;
}

static int akm09911_remove(struct i2c_client *client)
{
	struct akm09911_data *akm = i2c_get_clientdata(client);
	AKMFUNC("akm09911_remove");
	unregister_early_suspend(&akm->akm_early_suspend);
	misc_deregister(&akm_aot_device);
	misc_deregister(&akmd_device);
	input_unregister_device(akm->input_dev);
	free_irq(client->irq, akm);
	kfree(akm);
	AKMDBG("successfully removed.");
	return 0;
}

static struct of_device_id akm09911_i2c_of_match[] = {
        { .compatible = "AKM,akm09911",},
        { },
};

static const struct i2c_device_id akm09911_id[] = {
	{AKM09911_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver akm09911_driver = {
	.probe      = akm09911_probe,
	.remove     = akm09911_remove,
	.id_table   = akm09911_id,
	.driver = {
	    .owner = THIS_MODULE,
		.name = AKM09911_I2C_NAME,
		.of_match_table = akm09911_i2c_of_match,
	},
};

static int __init akm09911_init(void)
{
    int ret = i2c_add_driver(&akm09911_driver);
    printk(KERN_ERR "AKM09911 init, ret = %d\n", ret);
	return ret;
}

static void __exit akm09911_exit(void)
{
	
	i2c_del_driver(&akm09911_driver);
/* destory compass workqueue */
	if (compass_wq)
	{
		destroy_workqueue(compass_wq);
	}
}

late_initcall(akm09911_init);
module_exit(akm09911_exit);

MODULE_DESCRIPTION("AKM09911 compass driver");
MODULE_LICENSE("GPL");

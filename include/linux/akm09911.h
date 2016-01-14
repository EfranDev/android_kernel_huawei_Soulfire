/*
 * Definitions for akm09911 compass chip.
 */
#ifndef AKM09911_H
#define AKM09911_H

#include <linux/ioctl.h>
#include <linux/module.h>
#define AKM09911_I2C_NAME "akm09911"

/*! \name AK09911 operation mode
 \anchor AK09911_Mode
 Defines an operation mode of the AK09911.*/
/*! @{*/
#define AK09911_MODE_SNG_MEASURE	0x01
#define	AK09911_MODE_SELF_TEST		0x10
#define	AK09911_MODE_FUSE_ACCESS	0x1F
#define	AK09911_MODE_POWERDOWN		0x00

/*! @}*/

#define SENSOR_DATA_SIZE		9
#define RWBUF_SIZE				16


/*! \name AK09911 register address
\anchor AK09911_REG
Defines a register address of the AK09911.*/
/*! @{*/
#define AK09911_REG_WIA1			0x00
#define AK09911_REG_WIA2			0x01
#define AK09911_REG_INFO1			0x02
#define AK09911_REG_INFO2			0x03
#define AK09911_REG_ST1				0x10
#define AK09911_REG_HXL				0x11
#define AK09911_REG_HXH				0x12
#define AK09911_REG_HYL				0x13
#define AK09911_REG_HYH				0x14
#define AK09911_REG_HZL				0x15
#define AK09911_REG_HZH				0x16
#define AK09911_REG_TMPS			0x17
#define AK09911_REG_ST2				0x18
#define AK09911_REG_CNTL1			0x30
#define AK09911_REG_CNTL2			0x31
#define AK09911_REG_CNTL3			0x32
/*! @}*/

/*! \name AK09911 fuse-rom address
\anchor AK09911_FUSE
Defines a read-only address of the fuse ROM of the AK09911.*/
/*! @{*/
#define AK09911_FUSE_ASAX			0x60
#define AK09911_FUSE_ASAY			0x61
#define AK09911_FUSE_ASAZ			0x62

#define AK09911_RESET_DATA			0x01

#define AK09911_REGS_SIZE		13
#define AK09911_WIA1_VALUE		0x48
#define AK09911_WIA2_VALUE		0x05

#define AKM_DRDY_IS_HIGH(x)		((x) & 0x01)
/*! @}*/

#define AKMIO                   0xA1

/* IOCTLs for AKM library */
#define ECS_IOCTL_WRITE             _IOW(AKMIO, 0x01, char*)
#define ECS_IOCTL_READ              _IOWR(AKMIO, 0x02, char*)
#define ECS_IOCTL_RESET      	    _IO(AKMIO, 0x03)
#define ECS_IOCTL_SET_MODE          _IOW(AKMIO, 0x04, short)
#define ECS_IOCTL_GETDATA           _IOR(AKMIO, 0x05, char[SENSOR_DATA_SIZE])
#define ECS_IOCTL_SET_YPR           _IOW(AKMIO, 0x06, short[12])
#define ECS_IOCTL_GET_OPEN_STATUS   _IOR(AKMIO, 0x07, int)
#define ECS_IOCTL_GET_CLOSE_STATUS  _IOR(AKMIO, 0x08, int)
#define ECS_IOCTL_GET_DELAY         _IOR(AKMIO, 0x30, short)
#define ECS_IOCTL_GET_PROJECT_NAME  _IOR(AKMIO, 0x0D, char[64])
#define ECS_IOCTL_GET_MATRIX        _IOR(AKMIO, 0x0E, short [4][3][3])


/* IOCTLs for APPs */
#define ECS_IOCTL_APP_SET_MODE		_IOW(AKMIO, 0x10, short)/* NOT use */
#define ECS_IOCTL_APP_SET_MFLAG		_IOW(AKMIO, 0x11, short)
#define ECS_IOCTL_APP_GET_MFLAG		_IOW(AKMIO, 0x12, short)
#define ECS_IOCTL_APP_SET_AFLAG		_IOW(AKMIO, 0x13, short)
#define ECS_IOCTL_APP_GET_AFLAG		_IOR(AKMIO, 0x14, short)
#define ECS_IOCTL_APP_SET_TFLAG		_IOR(AKMIO, 0x15, short)/* NOT use */
#define ECS_IOCTL_APP_GET_TFLAG		_IOR(AKMIO, 0x16, short)/* NOT use */
#define ECS_IOCTL_APP_RESET_PEDOMETER   _IO(AKMIO, 0x17)	/* NOT use */
#define ECS_IOCTL_APP_SET_DELAY		_IOW(AKMIO, 0x18, short)
#define ECS_IOCTL_APP_GET_DELAY		ECS_IOCTL_GET_DELAY
#define ECS_IOCTL_APP_SET_MVFLAG	_IOW(AKMIO, 0x19, short)
#define ECS_IOCTL_APP_GET_MVFLAG	_IOR(AKMIO, 0x1A, short)
#define ECS_IOCTL_APP_GET_SLIDE 	_IOR(AKMIO, 0x1B, short)
#define ECS_IOCTL_SET_CAL  	_IOR(AKMIO, 0x0C, short)
#define ECS_IOCTL_APP_GET_CAL  	_IOR(AKMIO, 0x21, short)

#define ECS_IOCTL_APP_GET_DEVID 	_IOR(AKMIO, 0x1F, char[20])

/*struct akm09911_platform_data {
	char layouts[3][3];
	char project_name[64];
	int gpio_DRDY;
};*/

struct akm09911_platform_data {
	int poll_interval;
	int min_interval;

	u8 g_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
	int (*compass_power)(void);
	int rst_gpio;
	int int_gpio;
	int chip_position;
/*AKM use HAL to simulate Gyro sensor */
	int virtual_gyro_flag;


	/* set gpio_int[1,2] either to the choosen gpio pin number or to -EINVAL
	 * if leaved unconnected
	 */

};


#endif

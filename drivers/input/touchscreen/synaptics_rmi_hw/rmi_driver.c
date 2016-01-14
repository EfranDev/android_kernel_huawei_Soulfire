/*
 * Copyright (c) 2011 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 *
 * This driver adds support for generic RMI4 devices from Synpatics. It
 * implements the mandatory f01 RMI register and depends on the presence of
 * other required RMI functions.
 *
 * The RMI4 specification can be found here (URL split after files/ for
 * style reasons):
 * http://www.synaptics.com/sites/default/files/
 *           511-000136-01-Rev-E-RMI4%20Intrfacing%20Guide.pdf
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/pm.h>
#include <linux/rmi.h>
#include "rmi_driver.h"
#include <linux/input/rmi_i2c.h>

#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>

#include <linux/input.h>
#if !defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
#define DELAY_DEBUG 0
#define REGISTER_DEBUG 1

#define PDT_END_SCAN_LOCATION	0x0005
#define PDT_PROPERTIES_LOCATION 0x00EF
#define BSR_LOCATION 0x00FE
#define HAS_BSR_MASK 0x20
#define HAS_NONSTANDARD_PDT_MASK 0x40
#define RMI4_END_OF_PDT(id) ((id) == 0x00 || (id) == 0xff)
#define RMI4_MAX_PAGE 0xff
#define RMI4_PAGE_SIZE 0x100

#define RMI_DEVICE_RESET_CMD	0x01
#define DEFAULT_RESET_DELAY_MS	20


#define WRITE_FW_BLOCK            0x2
#define ERASE_ALL                 0x3
#define READ_CONFIG_BLOCK         0x5
#define WRITE_CONFIG_BLOCK        0x6
#define ERASE_CONFIG              0x7
#define ENABLE_FLASH_PROG         0xf

#define PDT_START_SCAN_LOCATION	0x00e9
#define PDT_END_SCAN_LOCATION	0x0005

#define BLK_SZ_OFF	3
#define IMG_BLK_CNT_OFF	5
#define CFG_BLK_CNT_OFF	7
#define BLK_SIZE 16
#define BLK_NUM_OFF 2


#define TP_FW_COB_FILE_NAME  "/tp/1191601.img"



#ifdef CONFIG_HAS_EARLYSUSPEND
static void rmi_driver_early_suspend(struct early_suspend *h);
static void rmi_driver_late_resume(struct early_suspend *h);
#endif

/* sysfs files for attributes for driver values. */
static ssize_t rmi_driver_hasbsr_show(struct device *dev,
				      struct device_attribute *attr, char *buf);

static ssize_t rmi_driver_bsr_show(struct device *dev,
				   struct device_attribute *attr, char *buf);

static ssize_t rmi_driver_bsr_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count);

static ssize_t rmi_driver_enabled_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf);

static ssize_t rmi_driver_enabled_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count);

static ssize_t rmi_driver_phys_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf);

static ssize_t rmi_driver_version_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf);

#if REGISTER_DEBUG
static ssize_t rmi_driver_reg_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count);
#endif

#if DELAY_DEBUG
static ssize_t rmi_delay_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf);

static ssize_t rmi_delay_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count);
#endif

static int rmi_driver_process_reset_requests(struct rmi_device *rmi_dev);

static int rmi_driver_process_config_requests(struct rmi_device *rmi_dev);

static int rmi_driver_irq_restore(struct rmi_device *rmi_dev);



static int i2c_update_firmware(struct rmi_device *rmi_dev, char *filename);

static int synaptics_download(struct rmi_device *rmi_dev,
					const unsigned char *pgm_data,u8 cmd);					
static int rmi4_read_PDT(struct rmi_device *rmi_dev);
static int rmi4_enable_program(struct rmi_device *rmi_dev);
static int rmi4_wait_attn(struct rmi_device *rmi_dev,int udelayed);
static int rmi4_program_firmware(struct rmi_device *rmi_dev,
					const unsigned char *pgm_data);
static int rmi4_check_firmware(struct rmi_device *rmi_dev,
					const unsigned char *pgm_data);
static int rmi4_program_configuration(struct rmi_device *rmi_dev,
					const unsigned char *pgm_data,u8 cmd );
static int rmi4_write_image(struct rmi_device *rmi_dev,
					unsigned char type_cmd,const unsigned char *pgm_data);
static int rmi4_disable_program(struct rmi_device *rmi_dev);
static int rmi_driver_suspend(struct device *dev);
static int rmi_driver_resume(struct device *dev);



struct rmi_function_descriptor f01_pdt;
struct rmi_function_descriptor f34_pdt;



/*move to tmi_i2c.c*/
static struct device_attribute attrs[] = {
	__ATTR(hasbsr, RMI_RO_ATTR,
	       rmi_driver_hasbsr_show, rmi_store_error),
	__ATTR(bsr, RMI_RW_ATTR,
	       rmi_driver_bsr_show, rmi_driver_bsr_store),
	__ATTR(enabled, RMI_RW_ATTR,
	       rmi_driver_enabled_show, rmi_driver_enabled_store),
	__ATTR(phys, RMI_RO_ATTR,
	       rmi_driver_phys_show, rmi_store_error),
#if REGISTER_DEBUG
	__ATTR(reg, RMI_WU_ATTR,
	       rmi_show_error, rmi_driver_reg_store),
#endif
#if DELAY_DEBUG
	__ATTR(delay, RMI_RW_ATTR,
	       rmi_delay_show, rmi_delay_store),
#endif
	__ATTR(version, RMI_RO_ATTR,
	       rmi_driver_version_show, rmi_store_error),
};
/*Need to use the function of synaptics fw update */
static int i2c_update_firmware(struct rmi_device *rmi_dev, char *filename) 
{
	char *buf;
	struct file	*filp;
	struct inode *inode = NULL;
	mm_segment_t oldfs;
	uint16_t	length;
	int ret = 0;

	/* open file */
	oldfs = get_fs();
	set_fs(KERNEL_DS);
	filp = filp_open(filename, O_RDONLY, S_IRUSR);
	if (IS_ERR(filp))
	{
        printk("%s: file %s filp_open error\n", __FUNCTION__,filename);
        set_fs(oldfs);
        return -1;
	}

	if (!filp->f_op)
	{
        printk("%s: File Operation Method Error\n", __FUNCTION__);
        filp_close(filp, NULL);
        set_fs(oldfs);
        return -1;
	}

	inode = filp->f_path.dentry->d_inode;
	if (!inode) 
	{
		printk("%s: Get inode from filp failed\n", __FUNCTION__);
		filp_close(filp, NULL);
		set_fs(oldfs);
		
		return -1;
	}

	/* file's size */
	length = i_size_read(inode->i_mapping->host);
	if (!( length > 0 && length < 62*1024 ))
	{
		printk("file size error\n");
		filp_close(filp, NULL);
		set_fs(oldfs);
		
		return -1;
	}

	/* allocation buff size */
	buf = vmalloc(length+(length%2));
	if (!buf) 
	{
		printk("alloctation memory failed\n");
		filp_close(filp, NULL);
		set_fs(oldfs);
		return -1;
	}

	/* read data */
	if (filp->f_op->read(filp, buf, length, &filp->f_pos) != length)
	{
		printk("%s: file read error\n", __FUNCTION__);
		filp_close(filp, NULL);
		set_fs(oldfs);
		vfree(buf);
		
		return -1;
	}

	ret = synaptics_download(rmi_dev,buf,WRITE_FW_BLOCK);

 	filp_close(filp, NULL);
	set_fs(oldfs);
	vfree(buf);
	
	return ret;
}

static int synaptics_download(struct rmi_device *rmi_dev,
	 								const unsigned char *pgm_data,u8 cmd)
{
	int ret;

	ret = rmi4_read_PDT(rmi_dev);
	if(ret != 0)
	{
		printk("RMI page func check error\n");
		return -1;
	}

	ret = rmi4_enable_program(rmi_dev);
	if( ret != 0)
	{
		printk("%s:%d:RMI enable program error,return...\n",__FUNCTION__,__LINE__);
		goto error;
	}

    if (cmd == WRITE_FW_BLOCK)
    {
		ret = rmi4_check_firmware(rmi_dev,pgm_data);
		if( ret != 0)
		{
			printk("%s:%d:RMI check firmware error,return...\n",__FUNCTION__,__LINE__);
			goto error;
		}
    

		ret = rmi4_program_firmware(rmi_dev, pgm_data + 0x100);
		if( ret != 0)
		{
			printk("%s:%d:RMI program firmware error,return...",__FUNCTION__,__LINE__);
			goto error;
		}
        
		ret = rmi4_program_configuration(rmi_dev, pgm_data +  0x100,cmd);
		if( ret != 0)
		{
			printk("%s:%d:RMI program configuration error,return...",__FUNCTION__,__LINE__);
			goto error;
		}
		
	}
	
	return rmi4_disable_program(rmi_dev);

error:
	rmi4_disable_program(rmi_dev);
	printk("%s:%d:error,return ....",__FUNCTION__,__LINE__);
	return -1;

}

static int rmi4_read_PDT(struct rmi_device *rmi_dev)
{
	struct pdt_entry pdt_entry;
	bool fn01found = false;
	bool fn34found = false;
	int retval = 0;
	int i;

	/* Rescan of the PDT is needed since issuing the Flash Enable cmd
	 * the device registers for Fn$01 and Fn$34 moving around because
	 * of the change from Bootloader mode to Flash Programming mode
	 * may change to a different PDT with only Fn$01 and Fn$34 that
	 * could have addresses for query, control, data, command registers
	 * that differ from the PDT scan done at device initialization. 
	 */

	/* rescan the PDT - filling in Fn01 and Fn34 addresses -
	 * this is only temporary - the device will need to be reset
	 * to return the PDT to the normal values. 
	 */

	/* mini-parse the PDT - we only have to get Fn$01 and Fn$34 and
	 * since we are Flash Programming mode we only have page 0. 
	 */
	for (i = PDT_START_SCAN_LOCATION; i >= PDT_END_SCAN_LOCATION;
			i -= sizeof(pdt_entry)) 
	{
		retval = rmi_read_block(rmi_dev, i, (u8 *)&pdt_entry,
					       sizeof(pdt_entry));
		if (retval != sizeof(pdt_entry)) 
		{
			pr_info( "%s: err frm rmi_read_block pdt "
					"entry data from PDT, "
					"error = %d.", __func__, retval);
			return retval;
		}

		if ((pdt_entry.function_number == 0x00) 
			||(pdt_entry.function_number == 0xff))
			break;

		pr_debug( "%s: Found F%.2X\n",
				__func__, pdt_entry.function_number);

		/* f01 found - just fill in the new addresses in
		 * the existing fc. 
		 */
		if (pdt_entry.function_number == 0x01) 
		{
			fn01found = true;
			f01_pdt.query_base_addr =
					pdt_entry.query_base_addr;
			f01_pdt.command_base_addr =
				  pdt_entry.command_base_addr;
			f01_pdt.control_base_addr =
				  pdt_entry.control_base_addr;
			f01_pdt.data_base_addr =
				  pdt_entry.data_base_addr;
			f01_pdt.function_number =
				  pdt_entry.function_number;
	
			if (fn34found)
				break;
		}

		/* f34 found - just fill in the new addresses in
		 * the existing fc. 
		 */
		if (pdt_entry.function_number == 0x34) 
		{
			fn34found = true;
			f34_pdt.query_base_addr =
				  pdt_entry.query_base_addr;
			f34_pdt.command_base_addr =
				  pdt_entry.command_base_addr;
			f34_pdt.control_base_addr =
				  pdt_entry.control_base_addr;
			f34_pdt.data_base_addr =
				  pdt_entry.data_base_addr;
			f34_pdt.function_number =
				  pdt_entry.function_number;
			
			if (fn01found)
				break;
		}

	}
	
	if (!fn01found || !fn34found) 
	{
		pr_info("%s: failed to find fn$01 or fn$34 trying "
				"to do rescan PDT.\n"
				, __func__);
		return -EINVAL;
	}
  
	return 0;
}

static int rmi4_enable_program(struct rmi_device *rmi_dev)
{
	unsigned char data[2];
	u16 data_base_addr;
	int ret = -1;
	unsigned char status;
	printk("RMI4 enable program...\n");

	
	/* Write the Bootloader ID key data back to the first two Block
	 * Data registers (F34_Flash_Data2.0 and F34_Flash_Data2.1). */

	ret = rmi_read_block(rmi_dev, 
			f34_pdt.query_base_addr, data,
			ARRAY_SIZE(data));

	if (ret < 0) {
		dev_err(&rmi_dev->dev, "Could not read bootloaderid from 0x%04x.\n",
			f34_pdt.query_base_addr);
		return ret;
	}

	data_base_addr = f34_pdt.data_base_addr;

	ret = rmi_write_block(rmi_dev,
				data_base_addr + BLK_NUM_OFF,
				data,
				ARRAY_SIZE(data));

	if (ret < 0) 
	{
		pr_err( "%s : Could not write bootloader id to 0x%x\n",
		       __func__, data_base_addr + BLK_NUM_OFF);
		return ret;
	}

	/* Issue the Enable flash command to the device. */
	ret = rmi_read(rmi_dev,
				data_base_addr + BLK_SIZE+ BLK_NUM_OFF,&status);
	status |= ENABLE_FLASH_PROG;
	ret = rmi_write(rmi_dev,
				data_base_addr + BLK_SIZE +
				BLK_NUM_OFF, status);
	if (ret < 0) 
	{
		pr_err("%s: Could not write command 0x%02x "
				"to 0x%04x\n", __func__, status,
				data_base_addr + BLK_SIZE +
				BLK_NUM_OFF);
		return ret;
	}		
    //printk("f34_status addr:0x%x\n",data_base_addr + instance_data->blocksize +BLK_NUM_OFF);
	ret = rmi4_wait_attn(rmi_dev,12);
	if (ret < 0) 
	{
		pr_err("%s:%d,error",__func__,__LINE__);
		return ret;
	}

	//Rescan the Page Description Table
	rmi4_read_PDT(rmi_dev);


	/*check status*/
	ret = rmi_read(rmi_dev,
			  	data_base_addr + BLK_SIZE +
			  	BLK_NUM_OFF, &status);

	if ((ret < 0) || (status != 0x80)) 
	{
		dev_err(&rmi_dev->dev, "Could not read status from 0x%x\n",
		       	data_base_addr + BLK_SIZE + BLK_NUM_OFF);
		/* failure */
		return -1;
	}
	return ret;
}
static int rmi4_wait_attn(struct rmi_device *rmi_dev,int udelayed)
{
	u16 data_base_addr = f34_pdt.data_base_addr;
	u8 status;
	int retval;
	u8 temp;
	int loop_count=0;
	

	rmi_read(rmi_dev,
			  	f01_pdt.data_base_addr , &status);
	printk("f01database:0x%x,status:0x%x",f01_pdt.data_base_addr,status);
	do
	{
		mdelay(udelayed);
		/* Read the Fn $34 status from F34_Flash_Data3 to see the previous
		 * commands status. F34_Flash_Data3 will be the address after the
		 * 2 block number registers plus blocksize Data registers.
		 *  inform user space - through a sysfs param. */
		retval = rmi_read(rmi_dev,
			  	data_base_addr + BLK_SIZE+
			  	BLK_NUM_OFF, &status);

		if (retval < 0) 
		{
			dev_err(&rmi_dev->dev, "Could not read status from 0x%x\n",
		       		data_base_addr + BLK_SIZE + BLK_NUM_OFF);
			status = 0xff;	/* failure */
		}
		/* Clear the attention assertion by reading the interrupt status register */
		rmi_read(rmi_dev,
				f01_pdt.data_base_addr + 1, 
				&temp);		
	}while(loop_count++ < 0x10 && (status != 0x80));

	/*printk("f34:0x%x,status:0x%x",data_base_addr + instance_data->blocksize +
			  	BLK_NUM_OFF,status);*/
	if(loop_count >= 0x10)
	{
		pr_err("rmi4 wait attn timeout:retval=0x%x\n",retval);
		return -1;
	}
	return 0;
}
static int rmi4_program_firmware(struct rmi_device *rmi_dev,
					const unsigned char *pgm_data)
{
	unsigned char data[2];
	u16 data_base_addr;
	int ret = -1;
	u8 status = 0;
	
	pr_info("RMI4 program UI firmware...\n");
	

	/* Write the Bootloader ID key data back to the first two Block
	 * Data registers (F34_Flash_Data2.0 and F34_Flash_Data2.1). */
	data_base_addr = f34_pdt.data_base_addr;

	ret = rmi_read_block(rmi_dev, 
			f34_pdt.query_base_addr, data,
			ARRAY_SIZE(data));

	if (ret < 0) {
		dev_err(&rmi_dev->dev, "Could not read bootloaderid from 0x%04x.\n",
			f34_pdt.query_base_addr);
		return ret;
	}
	
	ret = rmi_write_block(rmi_dev,
				data_base_addr + BLK_NUM_OFF,
				data,
				ARRAY_SIZE(data));
	if (ret < 0) 
	{
		pr_err( "%s : Could not write bootloader id to 0x%x\n",
		       __func__, data_base_addr + BLK_NUM_OFF);
		return ret;
	}
	
	rmi_read(rmi_dev,
			  	data_base_addr + BLK_SIZE +
			  	BLK_NUM_OFF, &status);
	//printk("status:%d\n",status);
	/* Issue the Erase all command to the device. */
	ret = rmi_write(rmi_dev,
				data_base_addr + BLK_SIZE +
				BLK_NUM_OFF, ERASE_ALL);

	if (ret < 0) 
	{
		pr_err("%s: Could not write command 0x%02x "
				"to 0x%04x\n", __func__,ERASE_ALL,
				data_base_addr + BLK_SIZE +
				BLK_NUM_OFF);
		return ret;
	}	

	rmi4_wait_attn(rmi_dev,300);

	/*check status*/
	ret = rmi_read(rmi_dev,
			  	data_base_addr + BLK_SIZE +
			  	BLK_NUM_OFF, &status);

	if ((ret < 0) || (status != 0x80)) 
	{
		dev_err(&rmi_dev->dev, "Could not read status from 0x%x\n",
		       	data_base_addr + BLK_SIZE + BLK_NUM_OFF);
		/* failure */
		return -1;
	}

	/*write firmware*/
	if( rmi4_write_image(rmi_dev,WRITE_FW_BLOCK,pgm_data) <0 )
	{
		pr_err("write UI firmware error!\n");
		return -1;
	}

	/*check status*/
	ret = rmi_read(rmi_dev,
			  	data_base_addr + BLK_SIZE +
			  	BLK_NUM_OFF, &status);

	if ((ret < 0) || (status != 0x80)) 
	{
		dev_err(&rmi_dev->dev, "Could not read status from 0x%x\n",
		       	data_base_addr + BLK_SIZE + BLK_NUM_OFF);
		/* failure */
		return -1;
	}
	return 0;
}

static int rmi4_check_firmware(struct rmi_device *rmi_dev,
							const unsigned char *pgm_data)
{
	unsigned short bootloader_id;
	const unsigned char *SynaFirmware;
	unsigned char m_firmwareImgVersion;
	unsigned short m_bootloader;
	unsigned char buf[2];
	int retval = 0;
  	SynaFirmware = pgm_data;
	bootloader_id = (unsigned int)SynaFirmware[4] + (unsigned int)SynaFirmware[5]*0x100;
	m_firmwareImgVersion = SynaFirmware[7];


	retval = rmi_read_block(rmi_dev,f34_pdt.query_base_addr,buf,ARRAY_SIZE(buf));
	if (retval < 0) {
		dev_err(&rmi_dev->dev, "Could not read bootloaderid from 0x%04x.\n",
			f34_pdt.query_base_addr);
		return retval;
	}
	batohs(&m_bootloader, buf);
	
	if ((m_firmwareImgVersion == 0) && (m_bootloader != bootloader_id))
	{
		return -1;
	}
	return 0;
}

static int rmi4_program_configuration(struct rmi_device *rmi_dev,
							const unsigned char *pgm_data,u8 cmd )
{
	int ret;
	unsigned short block_size;
	unsigned short ui_blocks;
	u16 data_base_addr;
	u16 query_base_addr;
	u8 status = 0;
	unsigned short bootloader_id = 0 ;
	unsigned char data[2];

	printk("\nRMI4 program Config firmware...\n");
	data_base_addr = f34_pdt.data_base_addr;
	query_base_addr = f34_pdt.query_base_addr;
	
	if (cmd == WRITE_FW_BLOCK)
	{
		
		ret = rmi_read_block(rmi_dev, query_base_addr + BLK_SZ_OFF, data,
			ARRAY_SIZE(data));
		if (ret < 0) {
			dev_err(&rmi_dev->dev, "Could not read block size from 0x%04x, "
				"error=%d.\n", query_base_addr + BLK_SZ_OFF, ret);
			return ret;
		}
		batohs(&block_size, data);

		ret = rmi_read_block(rmi_dev, query_base_addr + IMG_BLK_CNT_OFF,
			data, ARRAY_SIZE(data));

		if (ret < 0) {
			dev_err(&rmi_dev->dev, "Couldn't read image block count from 0x%x, "
				"error=%d.\n", query_base_addr + IMG_BLK_CNT_OFF,
				ret);
			return ret;
		}
		batohs(&ui_blocks, data);
		
		if(rmi4_write_image(rmi_dev, WRITE_CONFIG_BLOCK,pgm_data+ui_blocks*block_size ) < 0)
		{
			pr_err("write configure image error\n");
			return -1;
		}
	}
	else
	{
		ret = rmi_read_block(rmi_dev,query_base_addr,data,ARRAY_SIZE(data));
		if (ret < 0) {
			dev_err(&rmi_dev->dev, "Could not read bootloaderid from 0x%04x.\n",
				query_base_addr);
			return ret;
		}
		batohs(&bootloader_id, data);
		/* Write the Bootloader ID key data back to the first two Block
	 	 * Data registers (F34_Flash_Data2.0 and F34_Flash_Data2.1). */
		hstoba(data, bootloader_id);
		ret = rmi_write_block(rmi_dev,
				data_base_addr + BLK_NUM_OFF,
				data,
				ARRAY_SIZE(data));
		if (ret < 0) 
		{
			pr_err( "%s : Could not write bootloader id to 0x%x\n",
		       __func__, data_base_addr + BLK_NUM_OFF);
			return ret;
		}

		/* Issue the Erase config command to the device. */
		ret = rmi_write(rmi_dev,
				data_base_addr + BLK_SIZE +
				BLK_NUM_OFF, ERASE_CONFIG);
		if (ret < 0) 
		{
			pr_err("%s: Could not write command 0x%02x "
				"to 0x%04x\n", __func__, ERASE_CONFIG,
				data_base_addr + BLK_SIZE +
				BLK_NUM_OFF);
			return ret;
		}	

		rmi4_wait_attn(rmi_dev,300);

		/*check status*/
		ret = rmi_read(rmi_dev,
			  	data_base_addr + BLK_SIZE +
			  	BLK_NUM_OFF, &status);
		if ((ret < 0) || (status != 0x80)) 
		{
			dev_err(&rmi_dev->dev, "Could not read status from 0x%x\n",
		       	data_base_addr + BLK_SIZE + BLK_NUM_OFF);
			/* failure */
			return -1;
		}
	
		if(rmi4_write_image(rmi_dev, WRITE_CONFIG_BLOCK,pgm_data) < 0)
		{
			pr_err("write configure image error\n");
			return -1;
		}
	}
	
	/*check status*/
	ret = rmi_read(rmi_dev,
			  	data_base_addr + BLK_SIZE +
			  	BLK_NUM_OFF, &status);

	if ((ret < 0) || (status != 0x80)) 
	{
		dev_err(&rmi_dev->dev, "Could not read status from 0x%x\n",
		       	data_base_addr + BLK_SIZE + BLK_NUM_OFF);
		/* failure */
		return -1;
	}
	
	return 0;
}

static int rmi4_write_image(struct rmi_device *rmi_dev,
							unsigned char type_cmd,const unsigned char *pgm_data)
{
	unsigned short block_size;
	unsigned short img_blocks;
	unsigned short block_index;
	const unsigned char * p_data;
	int i;
	unsigned char data[2];
	unsigned short fw_block;
	unsigned short config_block;
	u16 data_base_addr;
	u16 query_base_addr;
	int ret = -1;

    unsigned char configdate[16] = {0};
    unsigned short first_block = 0;

	data_base_addr = f34_pdt.data_base_addr;
	query_base_addr = f34_pdt.query_base_addr;
	
	
    ret = rmi_read_block(rmi_dev, query_base_addr + BLK_SZ_OFF, data,
                ARRAY_SIZE(data));
    if (ret < 0) {
        dev_err(&rmi_dev->dev, "Could not read block size from 0x%04x, "
            "error=%d.\n", query_base_addr + BLK_SZ_OFF, ret);
        return ret;
    }
    batohs(&block_size, data);

    ret = rmi_read_block(rmi_dev, query_base_addr + IMG_BLK_CNT_OFF, data,
                ARRAY_SIZE(data));
		    if (ret < 0) {
		        dev_err(&rmi_dev->dev, "Could not read fw block size from 0x%04x, "
		            "error=%d.\n", query_base_addr + IMG_BLK_CNT_OFF, ret);
		        return ret;
		    }
	batohs(&fw_block, data);

	ret = rmi_read_block(rmi_dev, query_base_addr + CFG_BLK_CNT_OFF, data,
                ARRAY_SIZE(data));
		    if (ret < 0) {
		        dev_err(&rmi_dev->dev, "Could not read config block size from 0x%04x, "
		            "error=%d.\n", query_base_addr + CFG_BLK_CNT_OFF, ret);
		        return ret;
		    }
	batohs(&config_block, data);
	
	switch (type_cmd)
	{
		case WRITE_FW_BLOCK:
			/* UI Firmware block count */
			img_blocks = fw_block;
			break;
		case WRITE_CONFIG_BLOCK:
			/* read Configuration Block Count 0 */		
			img_blocks = config_block;
			break;
		default:
			pr_err("image type error\n");
			return -1;
	}

	p_data = pgm_data;
	
    // Write Block Number
	hstoba(data, first_block);
	ret = rmi_write_block(rmi_dev,
			data_base_addr,
			data,ARRAY_SIZE(data));
	if (ret < 0) 
	{
		dev_err(&rmi_dev->dev, "Could not write to 0x%x\n",data_base_addr);
		/* failure */
		return -1;
	}     
	//printk("img_blocks:%d,block_size:%d\n",img_blocks,block_size);
	for(block_index = 0; block_index < img_blocks; ++block_index)
	{

		for(i=0; i<block_size ; i++)      
		{          
		    configdate[i] = *(p_data+i);     
		}    

		ret = rmi_write_block(rmi_dev,data_base_addr + BLK_NUM_OFF,configdate,block_size);
		if (ret < 0) 
			{
				dev_err(&rmi_dev->dev, "Could not write to 0x%x\n",
		       			data_base_addr+ BLK_NUM_OFF + i);
				/* failure */
				return -1;
			}

		p_data += block_size;	
		
		// Issue Write Firmware or configuration Block command
		ret = rmi_write(rmi_dev,
				data_base_addr + BLK_SIZE +
				BLK_NUM_OFF, type_cmd);

		if (ret < 0) 
		{
			dev_err(&rmi_dev->dev, "Could not write to 0x%x\n",
		       		data_base_addr + BLK_SIZE + BLK_NUM_OFF);
			/* failure */
			return -1;
		}
		
		// Wait ATTN. Read Flash Command register and check error
		if(rmi4_wait_attn(rmi_dev,1) != 0)
		{
			return -1;
		}
	}
	return 0;
}
static int rmi4_disable_program(struct rmi_device *rmi_dev)
{
	
	unsigned char cdata; 
	unsigned int loop_count=0;
	
	printk("RMI4 disable program...\n");
	
	// Issue a reset command
	rmi_write(rmi_dev,
				f01_pdt.command_base_addr, 0x01);

	// Wait for ATTN to be asserted to see if device is in idle state
	rmi4_wait_attn(rmi_dev,20);

	// Read F01 Status flash prog, ensure the 6th bit is '0'
	do
	{
		rmi_read(rmi_dev,
			  	f01_pdt.data_base_addr
			  	, &cdata);
		udelay(2);
	} while(((cdata & 0x40) != 0) && (loop_count++ < 10));

	//Rescan the Page Description Table
	return rmi4_read_PDT(rmi_dev);
}


/* Useful helper functions for u8* */

void u8_set_bit(u8 *target, int pos)
{
	target[pos/8] |= 1<<pos%8;
}

void u8_clear_bit(u8 *target, int pos)
{
	target[pos/8] &= ~(1<<pos%8);
}

bool u8_is_set(u8 *target, int pos)
{
	return target[pos/8] & 1<<pos%8;
}

bool u8_is_any_set(u8 *target, int size)
{
	int i;
	for (i = 0; i < size; i++) {
		if (target[i])
			return true;
	}
	return false;
}

void u8_or(u8 *dest, u8 *target1, u8 *target2, int size)
{
	int i;
	for (i = 0; i < size; i++)
		dest[i] = target1[i] | target2[i];
}

void u8_and(u8 *dest, u8 *target1, u8 *target2, int size)
{
	int i;
	for (i = 0; i < size; i++)
		dest[i] = target1[i] & target2[i];
}

static bool has_bsr(struct rmi_driver_data *data)
{
	return (data->pdt_props & HAS_BSR_MASK) != 0;
}

/* Utility routine to set bits in a register. */
int rmi_set_bits(struct rmi_device *rmi_dev, unsigned short address,
		 unsigned char bits)
{
	unsigned char reg_contents;
	int retval;

	retval = rmi_read_block(rmi_dev, address, &reg_contents, 1);
	if (retval)
		return retval;
	reg_contents = reg_contents | bits;
	retval = rmi_write_block(rmi_dev, address, &reg_contents, 1);
	if (retval == 1)
		return 0;
	else if (retval == 0)
		return -EIO;
	return retval;
}
EXPORT_SYMBOL(rmi_set_bits);

/* Utility routine to clear bits in a register. */
int rmi_clear_bits(struct rmi_device *rmi_dev, unsigned short address,
		   unsigned char bits)
{
	unsigned char reg_contents;
	int retval;

	retval = rmi_read_block(rmi_dev, address, &reg_contents, 1);
	if (retval)
		return retval;
	reg_contents = reg_contents & ~bits;
	retval = rmi_write_block(rmi_dev, address, &reg_contents, 1);
	if (retval == 1)
		return 0;
	else if (retval == 0)
		return -EIO;
	return retval;
}
EXPORT_SYMBOL(rmi_clear_bits);

static void rmi_free_function_list(struct rmi_device *rmi_dev)
{
	struct rmi_function_container *entry, *n;
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);

	if (!data) {
		dev_err(&rmi_dev->dev, "WTF: No driver data in %s\n", __func__);
		return;
	}

	if (list_empty(&data->rmi_functions.list))
		return;

	list_for_each_entry_safe(entry, n, &data->rmi_functions.list, list) {
		kfree(entry->irq_mask);
		list_del(&entry->list);
	}
}

void no_op(struct device *dev)
{
	dev_dbg(dev, "REMOVING KOBJ!");
	kobject_put(&dev->kobj);
}

static int init_one_function(struct rmi_device *rmi_dev,
			     struct rmi_function_container *fc)
{
	int retval;

	if (!fc->fh) {
		struct rmi_function_handler *fh =
			rmi_get_function_handler(fc->fd.function_number);
		if (!fh) {
			dev_info(&rmi_dev->dev, "No handler for F%02X.\n",
				fc->fd.function_number);
			return 0;
		}
		fc->fh = fh;
	}

	if (!fc->fh->init)
		return 0;
	/* This memset might not be what we want to do... */
	memset(&(fc->dev), 0, sizeof(struct device));
	dev_set_name(&(fc->dev), "fn%02x", fc->fd.function_number);
	fc->dev.release = no_op;

	fc->dev.parent = &rmi_dev->dev;
	dev_info(&rmi_dev->dev, "%s: Register F%02X.\n", __func__,
			fc->fd.function_number);
	retval = device_register(&fc->dev);
	if (retval) {
		dev_err(&rmi_dev->dev, "Failed device_register for F%02X.\n",
			fc->fd.function_number);
		return retval;
	}

	retval = fc->fh->init(fc);
	if (retval < 0) {
		dev_err(&rmi_dev->dev, "Failed to initialize function F%02x\n",
			fc->fd.function_number);
		goto error_exit;
	}

	return 0;

error_exit:
	device_unregister(&fc->dev);
	return retval;
}

static void rmi_driver_fh_add(struct rmi_device *rmi_dev,
			      struct rmi_function_handler *fh)
{
	struct rmi_function_container *entry;
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);

	if (!data)
		return;
	if (fh->func == 0x01) {
		if (data->f01_container)
			data->f01_container->fh = fh;
	} else if (!list_empty(&data->rmi_functions.list)) {
		mutex_lock(&data->pdt_mutex);
		list_for_each_entry(entry, &data->rmi_functions.list, list)
			if (entry->fd.function_number == fh->func) {
				entry->fh = fh;
				init_one_function(rmi_dev, entry);
			}
		mutex_unlock(&data->pdt_mutex);
	}

}

static void rmi_driver_fh_remove(struct rmi_device *rmi_dev,
				 struct rmi_function_handler *fh)
{
	struct rmi_function_container *entry, *temp;
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);

	list_for_each_entry_safe(entry, temp, &data->rmi_functions.list,
									list) {
		if (entry->fd.function_number == fh->func) {
			if (fh->remove)
				fh->remove(entry);

			entry->fh = NULL;
			device_unregister(&entry->dev);
		}
	}
}


static int rmi_driver_process_reset_requests(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);
	struct device *dev = &rmi_dev->dev;
	struct rmi_function_container *entry;
	int error;

	/* Device control (F01) is handled before anything else. */

	if (data->f01_container->fh->reset) {
		error = data->f01_container->fh->reset(data->f01_container);
		if (error < 0) {
			dev_err(dev, "%s: f%.2x"
					" reset handler failed:"
					" %d\n", __func__,
					data->f01_container->fh->func, error);
			return error;
		}
	}

	list_for_each_entry(entry, &data->rmi_functions.list, list) {
		if (entry->fh->reset) {
			error = entry->fh->reset(entry);
			if (error < 0) {
				dev_err(dev, "%s: f%.2x"
						" reset handler failed:"
						" %d\n", __func__,
						entry->fh->func, error);
				return error;
			}
		}

	}

	return 0;
}


static int rmi_driver_process_config_requests(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);
	struct device *dev = &rmi_dev->dev;
	struct rmi_function_container *entry;
	int error;

	/* Device control (F01) is handled before anything else. */

	if (data->f01_container->fh->config) {
		error = data->f01_container->fh->config(data->f01_container);
		if (error < 0) {
			dev_err(dev, "%s: f%.2x"
					" config handler failed:"
					" %d\n", __func__,
					data->f01_container->fh->func, error);
			return error;
		}
	}

	list_for_each_entry(entry, &data->rmi_functions.list, list) {
		if (entry->fh->config) {
			error = entry->fh->config(entry);
			if (error < 0) {
				dev_err(dev, "%s: f%.2x"
						" config handler failed:"
						" %d\n", __func__,
						entry->fh->func, error);
				return error;
			}
		}
	}

	return 0;
}

static void construct_mask(u8 *mask, int num, int pos)
{
	int i;

	for (i = 0; i < num; i++)
		u8_set_bit(mask, pos+i);
}

static int process_interrupt_requests(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);
	struct device *dev = &rmi_dev->dev;
	struct rmi_function_container *entry;
	u8 irq_status[data->num_of_irq_regs];
	u8 irq_bits[data->num_of_irq_regs];
	int error;
	int i ;

    tp_log_debug("%s start \n", __func__);

	error = rmi_read_block(rmi_dev,
				data->f01_container->fd.data_base_addr + 1,
				irq_status, data->num_of_irq_regs);
	for (i = 0; i < data->num_of_irq_regs; i++) {
		pr_debug("process_interrupt_requests: irq_status[%d]:0x%02x\n",i,irq_status[i]);
    }
    if (error < 0) {
		dev_err(dev, "%s: failed to read irqs.", __func__);
		return error;
	}
	/* Device control (F01) is handled before anything else. */
	u8_and(irq_bits, irq_status, data->f01_container->irq_mask,
			data->num_of_irq_regs);
	if (u8_is_any_set(irq_bits, data->num_of_irq_regs))
		data->f01_container->fh->attention(
				data->f01_container, irq_bits);

	u8_and(irq_status, irq_status, data->current_irq_mask,
	       data->num_of_irq_regs);
	/* At this point, irq_status has all bits that are set in the
	 * interrupt status register and are enabled.
	 */

	list_for_each_entry(entry, &data->rmi_functions.list, list)
		if (entry->irq_mask && entry->fh && entry->fh->attention) {
			u8_and(irq_bits, irq_status, entry->irq_mask,
			       data->num_of_irq_regs);
			if (u8_is_any_set(irq_bits, data->num_of_irq_regs)) {
				error = entry->fh->attention(entry, irq_bits);
				if (error < 0)
					dev_err(dev, "%s: f%.2x"
						" attention handler failed:"
						" %d\n", __func__,
						entry->fh->func, error);
			}
		}

	return 0;
}

static int rmi_driver_irq_handler(struct rmi_device *rmi_dev, int irq)
{
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);

	/* Can get called before the driver is fully ready to deal with
	 * interrupts.
	 */
	if (!data || !data->f01_container || !data->f01_container->fh) {
		tp_log_debug("Not ready to handle interrupts yet!\n");
		return 0;
	}

	return process_interrupt_requests(rmi_dev);
}


static int rmi_driver_reset_handler(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);
	int error;

	/* Can get called before the driver is fully ready to deal with
	 * interrupts.
	 */
	if (!data || !data->f01_container || !data->f01_container->fh) {
		dev_warn(&rmi_dev->dev,
			 "Not ready to handle reset yet!\n");
		return 0;
	}

	error = rmi_driver_process_reset_requests(rmi_dev);
	if (error < 0)
		return error;


	error = rmi_driver_process_config_requests(rmi_dev);
	if (error < 0)
		return error;

	if (data->irq_stored) {
		error = rmi_driver_irq_restore(rmi_dev);
		if (error < 0)
			return error;
	}

	dev_err(&rmi_dev->dev, "DEBUG 5!\n");

	return 0;
}



/*
 * Construct a function's IRQ mask. This should
 * be called once and stored.
 */
static u8 *rmi_driver_irq_get_mask(struct rmi_device *rmi_dev,
				   struct rmi_function_container *fc) {
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);
	
	u8 *irq_mask = kzalloc(sizeof(u8)*data->num_of_irq_regs, GFP_KERNEL);
	if (irq_mask)
		construct_mask(irq_mask, fc->num_of_irqs, fc->irq_pos);

	return irq_mask;
}

/*
 * This pair of functions allows functions like function 54 to request to have
 * other interupts disabled until the restore function is called. Only one store
 * happens at a time.
 */
static int rmi_driver_irq_save(struct rmi_device *rmi_dev, u8 * new_ints)
{
	int retval = 0;
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);
	struct device *dev = &rmi_dev->dev;

	mutex_lock(&data->irq_mutex);
	if (!data->irq_stored) {
		/* Save current enabled interupts */
		retval = rmi_read_block(rmi_dev,
				data->f01_container->fd.control_base_addr+1,
				data->irq_mask_store, data->num_of_irq_regs);
		if (retval < 0) {
			dev_err(dev, "%s: Failed to read enabled interrupts!",
								__func__);
			goto error_unlock;
		}
		/*
		 * Disable every interupt except for function 54
		 * TODO:Will also want to not disable function 1-like functions.
		 * No need to take care of this now, since there's no good way
		 * to identify them.
		 */
		retval = rmi_write_block(rmi_dev,
				data->f01_container->fd.control_base_addr+1,
				new_ints, data->num_of_irq_regs);
		if (retval < 0) {
			dev_err(dev, "%s: Failed to change enabled interrupts!",
								__func__);
			goto error_unlock;
		}
		memcpy(data->current_irq_mask, new_ints,
					data->num_of_irq_regs * sizeof(u8));
		data->irq_stored = true;
	} else {
		retval = -ENOSPC; /* No space to store IRQs.*/
		dev_err(dev, "%s: Attempted to save values when"
						" already stored!", __func__);
	}

error_unlock:
	mutex_unlock(&data->irq_mutex);
	return retval;
}

static int rmi_driver_irq_restore(struct rmi_device *rmi_dev)
{
	int retval = 0;
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);
	struct device *dev = &rmi_dev->dev;
	mutex_lock(&data->irq_mutex);

	if (data->irq_stored) {
		retval = rmi_write_block(rmi_dev,
				data->f01_container->fd.control_base_addr+1,
				data->irq_mask_store, data->num_of_irq_regs);
		if (retval < 0) {
			dev_err(dev, "%s: Failed to write enabled interupts!",
								__func__);
			goto error_unlock;
		}
		memcpy(data->current_irq_mask, data->irq_mask_store,
					data->num_of_irq_regs * sizeof(u8));
		data->irq_stored = false;
	} else {
		retval = -EINVAL;
		dev_err(dev, "%s: Attempted to restore values when not stored!",
			__func__);
	}

error_unlock:
	mutex_unlock(&data->irq_mutex);
	return retval;
}

static int rmi_driver_fn_generic(struct rmi_device *rmi_dev,
				     struct pdt_entry *pdt_ptr,
				     int *current_irq_count,
				     u16 page_start)
{
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);
	struct rmi_function_container *fc = NULL;
	int retval = 0;
	struct device *dev = &rmi_dev->dev;
	struct rmi_device_platform_data *pdata;

	pdata = to_rmi_platform_data(rmi_dev);

	dev_info(dev, "Initializing F%02X for %s.\n", pdt_ptr->function_number,
		pdata->sensor_name);

	fc = kzalloc(sizeof(struct rmi_function_container),
			GFP_KERNEL);
	if (!fc) {
		dev_err(dev, "Failed to allocate container for F%02X.\n",
			pdt_ptr->function_number);
		retval = -ENOMEM;
		goto error_free_data;
	}

	copy_pdt_entry_to_fd(pdt_ptr, &fc->fd, page_start);

	fc->rmi_dev = rmi_dev;
	fc->num_of_irqs = pdt_ptr->interrupt_source_count;
	fc->irq_pos = *current_irq_count;
	*current_irq_count += fc->num_of_irqs;

	retval = init_one_function(rmi_dev, fc);
	if (retval < 0) {
		dev_err(dev, "Failed to initialize F%.2x\n",
			pdt_ptr->function_number);
		kfree(fc);
		goto error_free_data;
	}

	list_add_tail(&fc->list, &data->rmi_functions.list);
	return 0;

error_free_data:
	kfree(fc);
	return retval;
}

/*
 * F01 was once handled very differently from all other functions.  It is
 * now only slightly special, and as the driver is refined we expect this
 * function to go away.
 */
static int rmi_driver_fn_01_specific(struct rmi_device *rmi_dev,
				     struct pdt_entry *pdt_ptr,
				     int *current_irq_count,
				     u16 page_start)
{
	struct rmi_driver_data *data = NULL;
	struct rmi_function_container *fc = NULL;
	union f01_device_status device_status;
	int retval = 0;
	struct device *dev = &rmi_dev->dev;
	struct rmi_function_handler *fh =
		rmi_get_function_handler(0x01);
	struct rmi_device_platform_data *pdata;

	pdata = to_rmi_platform_data(rmi_dev);
	data = rmi_get_driverdata(rmi_dev);

	retval = rmi_read(rmi_dev, pdt_ptr->data_base_addr, &device_status.reg);
	if (retval) {
		dev_err(dev, "Failed to read device status.\n");
		return retval;
	}

	dev_info(dev, "Initializing F01 for %s.\n", pdata->sensor_name);
	if (!fh)
		dev_info(dev, "%s: No function handler for F01?!", __func__);

	fc = kzalloc(sizeof(struct rmi_function_container), GFP_KERNEL);
	if (!fc) {
		retval = -ENOMEM;
		return retval;
	}

	copy_pdt_entry_to_fd(pdt_ptr, &fc->fd, page_start);
	fc->num_of_irqs = pdt_ptr->interrupt_source_count;
	fc->irq_pos = *current_irq_count;
	*current_irq_count += fc->num_of_irqs;

	fc->rmi_dev        = rmi_dev;
	fc->dev.parent     = &fc->rmi_dev->dev;
	fc->fh = fh;

	dev_set_name(&(fc->dev), "fn%02x", fc->fd.function_number);
	fc->dev.release = no_op;

	dev_info(dev, "%s: Register F01.\n", __func__);
	retval = device_register(&fc->dev);
	if (retval) {
		dev_err(dev, "%s: Failed device_register for F01.\n", __func__);
		goto error_free_data;
	}

	data->f01_container = fc;
	data->f01_bootloader_mode = device_status.flash_prog;
	if (device_status.flash_prog)
		dev_warn(dev, "WARNING: RMI4 device is in bootloader mode!\n");

	return retval;

error_free_data:
	kfree(fc);
	return retval;
}

/*
 * Scan the PDT for F01 so we can force a reset before anything else
 * is done.  This forces the sensor into a known state, and also
 * forces application of any pending updates from reflashing the
 * firmware or configuration.  We have to do this before actually
 * building the PDT because the reflash might cause various registers
 * to move around.
 */
static int do_initial_reset(struct rmi_device *rmi_dev)
{
	struct pdt_entry pdt_entry;
	int page;
	struct device *dev = &rmi_dev->dev;
	bool done = false;
	int i;
	int retval;
	struct rmi_device_platform_data *pdata;
	struct rmi_driver_data *data = NULL;
	union f01_device_status device_status;

	dev_info(dev, "Initial reset.\n");
	pdata = to_rmi_platform_data(rmi_dev);
	data = rmi_get_driverdata(rmi_dev);
	
	for (page = 0; (page <= RMI4_MAX_PAGE) && !done; page++) {
		u16 page_start = RMI4_PAGE_SIZE * page;
		u16 pdt_start = page_start + PDT_START_SCAN_LOCATION;
		u16 pdt_end = page_start + PDT_END_SCAN_LOCATION;

		done = true;
		for (i = pdt_start; i >= pdt_end; i -= sizeof(pdt_entry)) {
			retval = rmi_read_block(rmi_dev, i, (u8 *)&pdt_entry,
					       sizeof(pdt_entry));
			if (retval != sizeof(pdt_entry)) {
				dev_err(dev, "Read PDT entry at 0x%04x"
					"failed, code = %d.\n", i, retval);
				return retval;
			}

			dev_info(dev, "%s: Found F%.2X on page 0x%02X\n",
				__func__, pdt_entry.function_number, page);
			
			if (RMI4_END_OF_PDT(pdt_entry.function_number))
				break;
			done = false;

			if (pdt_entry.function_number == 0x01) {
				u16 cmd_addr = page_start +
					pdt_entry.command_base_addr;
				u8 cmd_buf = RMI_DEVICE_RESET_CMD;
				retval = rmi_write_block(rmi_dev, cmd_addr,
						&cmd_buf, 1);
				if (retval < 0) {
					dev_err(dev, "Initial reset failed. "
						"Code = %d.\n", retval);
					return retval;
				}
				mdelay(pdata->reset_delay_ms);

				/*read FW status*/
				retval = rmi_read(rmi_dev, page_start + pdt_entry.data_base_addr, &device_status.reg);
				if (retval) {
					pr_info("Failed to read device status.\n");
					return retval;
				}
				data->f01_bootloader_mode = device_status.flash_prog;
				return 0;
			}
		}
	}

	dev_warn(dev, "WARNING: Failed to find F01 for initial reset.\n");
	return -ENODEV;
}

static int rmi_scan_pdt(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data;
	struct pdt_entry pdt_entry;
	int page;
	struct device *dev = &rmi_dev->dev;
	int irq_count = 0;
	bool done = false;
	int i;
	int retval;

	dev_info(dev, "Scanning PDT...\n");

	data = rmi_get_driverdata(rmi_dev);
	mutex_lock(&data->pdt_mutex);
	for (page = 0; (page <= RMI4_MAX_PAGE) && !done; page++) {
		u16 page_start = RMI4_PAGE_SIZE * page;
		u16 pdt_start = page_start + PDT_START_SCAN_LOCATION;
		u16 pdt_end = page_start + PDT_END_SCAN_LOCATION;

		done = true;
		for (i = pdt_start; i >= pdt_end; i -= sizeof(pdt_entry)) {
			retval = rmi_read_block(rmi_dev, i, (u8 *)&pdt_entry,
					       sizeof(pdt_entry));
			if (retval != sizeof(pdt_entry)) {
				dev_err(dev, "Read PDT entry at 0x%04x "
					"failed.\n", i);
				goto error_exit;
			}
			
			dev_info(dev, "%s: Found F%.2X on page 0x%02X\n",
				__func__, pdt_entry.function_number, page);
			
			if (RMI4_END_OF_PDT(pdt_entry.function_number))
				break;

			done = false;

			if (pdt_entry.function_number == 0x01)
				retval = rmi_driver_fn_01_specific(rmi_dev,
						&pdt_entry, &irq_count,
						page_start);
			else
				retval = rmi_driver_fn_generic(rmi_dev,
						&pdt_entry, &irq_count,
						page_start);

			if (retval)
				goto error_exit;
		}
		done = done | data->f01_bootloader_mode;
	}
	data->irq_count = irq_count;
	data->num_of_irq_regs = (irq_count + 7) / 8;
	dev_info(dev, "%s: Done with PDT scan.\n", __func__);
	retval = 0;

error_exit:
	mutex_unlock(&data->pdt_mutex);
	return retval;
}
#if !defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank = NULL;
	struct rmi_device *cd =
		container_of(self, struct rmi_device, fb_notif);

	struct device *dev = &cd->dev;
	
	if (evdata && evdata->data && event == FB_EVENT_BLANK && cd) {
		blank = evdata->data;
		dev_info(dev, "%s:%d %d\n",__func__,__LINE__, *blank);
		/*In case of resume we get BLANK_UNBLANK and for suspen BLANK_POWERDOWN*/
		if (*blank == FB_BLANK_UNBLANK)
			rmi_driver_resume(dev);
		else if (*blank == FB_BLANK_POWERDOWN)
			rmi_driver_suspend(dev);
	}
	return 0;
}

static void configure_sleep(struct rmi_device *rmi_dev)
{
	int retval = 0;
       struct device *dev = &rmi_dev->dev;
	   
	rmi_dev->fb_notif.notifier_call = fb_notifier_callback;

	/*register the callback to the FB*/
	retval = fb_register_client(&rmi_dev->fb_notif);
	if (retval)
		dev_info(dev,
			"Unable to register fb_notifier: %d\n", retval);
	return;
}
#endif
static int rmi_driver_probe(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = NULL;
	struct rmi_function_container *fc;
	struct rmi_device_platform_data *pdata;
	int irq = 0;
	
	int error = 0;
	struct device *dev = &rmi_dev->dev;
	int attr_count = 0;

	dev_info(dev, "%s: Starting probe.\n", __func__);
	

	pdata = to_rmi_platform_data(rmi_dev);

	data = kzalloc(sizeof(struct rmi_driver_data), GFP_KERNEL);
	if (!data) {
		dev_err(dev, "%s: Failed to allocate driver data.\n", __func__);
		return -ENOMEM;
	}
	INIT_LIST_HEAD(&data->rmi_functions.list);
	rmi_set_driverdata(rmi_dev, data);
	mutex_init(&data->pdt_mutex);

	if (!pdata->reset_delay_ms)
		pdata->reset_delay_ms = DEFAULT_RESET_DELAY_MS;
	error = do_initial_reset(rmi_dev);
	if (error)
		dev_warn(dev, "RMI initial reset failed! Soldiering on.\n");
			

    /*update fw when the fw went into bootloader mode */
    if (data->f01_bootloader_mode) 
    {
        if (NULL == rmi_dev)
        {
            tp_log_err("%s, rmi_dev is NULL\n", __func__);
        	goto err_free_data;
        }
        memset(&f01_pdt,0,sizeof(struct rmi_function_descriptor));
        memset(&f34_pdt,0,sizeof(struct rmi_function_descriptor));
        irq= gpio_to_irq(pdata->attn_gpio);
        disable_irq(irq);
        error = i2c_update_firmware(rmi_dev,TP_FW_COB_FILE_NAME);
        if (error < 0)
        {
			dev_err(dev, "syna_ic was failed to update firmware.\n");
			goto err_free_data;
	    }
        enable_irq(irq);
        tp_log_info("%s, update firmware finished!\n", __func__);
     
    }
   

	error = rmi_scan_pdt(rmi_dev);
	if (error) {
		dev_err(dev, "PDT scan for %s failed with code %d.\n",
			pdata->sensor_name, error);
		goto err_free_data;
	}

	if (!data->f01_container) {
		dev_err(dev, "missing F01 container!\n");
		error = -EINVAL;
		goto err_free_data;
	}
    
	data->f01_container->irq_mask = kzalloc(
			sizeof(u8)*data->num_of_irq_regs, GFP_KERNEL);
	if (!data->f01_container->irq_mask) {
		dev_err(dev, "Failed to allocate F01 IRQ mask.\n");
		error = -ENOMEM;
		goto err_free_data;
	}
	construct_mask(data->f01_container->irq_mask,
		       data->f01_container->num_of_irqs,
		       data->f01_container->irq_pos);
	list_for_each_entry(fc, &data->rmi_functions.list, list)
		fc->irq_mask = rmi_driver_irq_get_mask(rmi_dev, fc);
    
	error = rmi_driver_f01_init(rmi_dev);
	if (error < 0) {
		dev_err(dev, "Failed to initialize F01.\n");
		goto err_free_data;
	}

	error = rmi_read(rmi_dev, PDT_PROPERTIES_LOCATION,
			 (char *) &data->pdt_props);
	if (error < 0) {
		/* we'll print out a warning and continue since
		 * failure to get the PDT properties is not a cause to fail
		 */
		dev_warn(dev, "Could not read PDT properties from 0x%04x. "
			 "Assuming 0x00.\n", PDT_PROPERTIES_LOCATION);
	}

	dev_info(dev, "%s: Creating sysfs files.", __func__);
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		error = device_create_file(dev, &attrs[attr_count]);
		if (error < 0) {
			dev_err(dev, "%s: Failed to create sysfs file %s.\n",
				__func__, attrs[attr_count].attr.name);
			goto err_free_data;
		}
	}

/*move to tmi_i2c.c*/
	__mutex_init(&data->irq_mutex, "irq_mutex", &data->irq_key);
	data->current_irq_mask = kzalloc(sizeof(u8)*data->num_of_irq_regs,
					 GFP_KERNEL);
	if (!data->current_irq_mask) {
		dev_err(dev, "Failed to allocate current_irq_mask.\n");
		error = -ENOMEM;
		goto err_free_data;
	}
	error = rmi_read_block(rmi_dev,
				data->f01_container->fd.control_base_addr+1,
				data->current_irq_mask, data->num_of_irq_regs);
	if (error < 0) {
		dev_err(dev, "%s: Failed to read current IRQ mask.\n",
			__func__);
		goto err_free_data;
	}
	data->irq_mask_store = kzalloc(sizeof(u8)*data->num_of_irq_regs,
				       GFP_KERNEL);
	if (!data->irq_mask_store) {
		dev_err(dev, "Failed to allocate mask store.\n");
		error = -ENOMEM;
		goto err_free_data;
	}
#if !defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_FB)
       configure_sleep(rmi_dev);
#endif
#ifdef	CONFIG_PM
	data->pm_data = pdata->pm_data;
	data->pre_suspend = pdata->pre_suspend;
	data->post_resume = pdata->post_resume;

	mutex_init(&data->suspend_mutex);

#ifdef CONFIG_HAS_EARLYSUSPEND
	rmi_dev->early_suspend_handler.level =
		EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	rmi_dev->early_suspend_handler.suspend = rmi_driver_early_suspend;
	rmi_dev->early_suspend_handler.resume = rmi_driver_late_resume;
	register_early_suspend(&rmi_dev->early_suspend_handler);
#endif /* CONFIG_HAS_EARLYSUSPEND */
#endif /* CONFIG_PM */
	data->enabled = true;

	dev_info(dev, "connected RMI device manufacturer: %s product: %s\n",
		 data->manufacturer_id == 1 ? "synaptics" : "unknown",
		 data->product_id);

	

	return 0;

 err_free_data:
	rmi_free_function_list(rmi_dev);
	for (attr_count--; attr_count >= 0; attr_count--)
		device_remove_file(dev, &attrs[attr_count]);
	if (data) {
		if (data->f01_container)
			kfree(data->f01_container->irq_mask);
		kfree(data->irq_mask_store);
		kfree(data->current_irq_mask);
		kfree(data);
		rmi_set_driverdata(rmi_dev, NULL);
	}
	return error;
}

#ifdef CONFIG_PM
static int rmi_driver_suspend(struct device *dev)
{
	struct rmi_device *rmi_dev;
	struct rmi_driver_data *data;
	struct rmi_function_container *entry;
	int retval = 0;

	rmi_dev = to_rmi_device(dev);
	data = rmi_get_driverdata(rmi_dev);
	dev_info(dev, "%s: rmi suspend start.\n", __func__);
	
	mutex_lock(&data->suspend_mutex);
	if (data->suspended)
		goto exit;

#ifndef	CONFIG_HAS_EARLYSUSPEND
	if (data->pre_suspend) {
		retval = data->pre_suspend(data->pm_data);
		if (retval)
			goto exit;
	}
#endif  /* !CONFIG_HAS_EARLYSUSPEND */

	list_for_each_entry(entry, &data->rmi_functions.list, list)
		if (entry->fh && entry->fh->suspend) {
			retval = entry->fh->suspend(entry);
			if (retval < 0)
				goto exit;
		}

	if (data->f01_container && data->f01_container->fh
				&& data->f01_container->fh->suspend) {
		retval = data->f01_container->fh->suspend(data->f01_container);
		if (retval < 0)
			goto exit;
	}
	data->suspended = true;

exit:
	mutex_unlock(&data->suspend_mutex);
	return retval;
}

static int rmi_driver_resume(struct device *dev)
{
	struct rmi_device *rmi_dev;
	struct rmi_driver_data *data;
	struct rmi_function_container *entry;
	int retval = 0;

	rmi_dev = to_rmi_device(dev);
	data = rmi_get_driverdata(rmi_dev);
	dev_info(dev, "%s: rmi resume enter.\n", __func__);

	mutex_lock(&data->suspend_mutex);
	if (!data->suspended)
		goto exit;

	if (data->f01_container && data->f01_container->fh
				&& data->f01_container->fh->resume) {
		retval = data->f01_container->fh->resume(data->f01_container);
		if (retval < 0)
			goto exit;
	}

	list_for_each_entry(entry, &data->rmi_functions.list, list)
		if (entry->fh && entry->fh->resume) {
			retval = entry->fh->resume(entry);
			if (retval < 0)
				goto exit;
		}

#ifndef	CONFIG_HAS_EARLYSUSPEND
	if (data->post_resume) {
		retval = data->post_resume(data->pm_data);
		if (retval)
			goto exit;
	}
#endif  /* !CONFIG_HAS_EARLYSUSPEND */

	data->suspended = false;

exit:
	mutex_unlock(&data->suspend_mutex);
	return retval;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void rmi_driver_early_suspend(struct early_suspend *h)
{
	struct rmi_device *rmi_dev =
	    container_of(h, struct rmi_device, early_suspend_handler);
	struct rmi_driver_data *data;
	struct rmi_function_container *entry;
	int retval = 0;

	data = rmi_get_driverdata(rmi_dev);
	dev_dbg(&rmi_dev->dev, "Early suspend.\n");

	mutex_lock(&data->suspend_mutex);
	if (data->suspended)
		goto exit;

	if (data->pre_suspend) {
		retval = data->pre_suspend(data->pm_data);
		if (retval)
			goto exit;
	}

	list_for_each_entry(entry, &data->rmi_functions.list, list)
		if (entry->fh && entry->fh->early_suspend) {
			retval = entry->fh->early_suspend(entry);
			if (retval < 0)
				goto exit;
		}

	if (data->f01_container && data->f01_container->fh
				&& data->f01_container->fh->early_suspend) {
		retval = data->f01_container->fh->early_suspend(
				data->f01_container);
		if (retval < 0)
			goto exit;
	}
	data->suspended = true;

exit:
	mutex_unlock(&data->suspend_mutex);
}

static void rmi_driver_late_resume(struct early_suspend *h)
{
	struct rmi_device *rmi_dev =
	    container_of(h, struct rmi_device, early_suspend_handler);
	struct rmi_driver_data *data;
	struct rmi_function_container *entry;
	int retval = 0;

	data = rmi_get_driverdata(rmi_dev);
	dev_dbg(&rmi_dev->dev, "Late resume.\n");

	mutex_lock(&data->suspend_mutex);
	if (!data->suspended)
		goto exit;

	if (data->f01_container && data->f01_container->fh
				&& data->f01_container->fh->late_resume) {
		retval = data->f01_container->fh->late_resume(
				data->f01_container);
		if (retval < 0)
			goto exit;
	}

	list_for_each_entry(entry, &data->rmi_functions.list, list)
		if (entry->fh && entry->fh->late_resume) {
			retval = entry->fh->late_resume(entry);
			if (retval < 0)
				goto exit;
		}

	if (data->post_resume) {
		retval = data->post_resume(data->pm_data);
		if (retval)
			goto exit;
	}

	data->suspended = false;

exit:
	mutex_unlock(&data->suspend_mutex);
}
#endif /* CONFIG_HAS_EARLYSUSPEND */
#endif /* CONFIG_PM */

static int __devexit rmi_driver_remove(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);
	struct rmi_function_container *entry;
	int i;

	list_for_each_entry(entry, &data->rmi_functions.list, list)
		if (entry->fh && entry->fh->remove)
			entry->fh->remove(entry);
#if !defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_FB)
	fb_unregister_client(&rmi_dev->fb_notif);
#endif

	rmi_free_function_list(rmi_dev);
	for (i = 0; i < ARRAY_SIZE(attrs); i++)
		device_remove_file(&rmi_dev->dev, &attrs[i]);
	kfree(data->f01_container->irq_mask);
	kfree(data->irq_mask_store);
	kfree(data->current_irq_mask);
	kfree(data);

	return 0;
}

#ifdef UNIVERSAL_DEV_PM_OPS
static UNIVERSAL_DEV_PM_OPS(rmi_driver_pm, rmi_driver_suspend,
			    rmi_driver_resume, NULL);
#endif

static struct rmi_driver sensor_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "rmi-generic",
/* wakeup and sleep method
 * 1) pm
 * 2) Earlysuspend
 * select one method
 */
#ifndef CONFIG_FB
#ifndef CONFIG_HAS_EARLYSUSPEND
#ifdef UNIVERSAL_DEV_PM_OPS
		.pm = &rmi_driver_pm,
#endif
#endif
#endif
	},
	.probe = rmi_driver_probe,
	.irq_handler = rmi_driver_irq_handler,
	.reset_handler = rmi_driver_reset_handler,
	.fh_add = rmi_driver_fh_add,
	.fh_remove = rmi_driver_fh_remove,
	.get_func_irq_mask = rmi_driver_irq_get_mask,
	.store_irq_mask = rmi_driver_irq_save,
	.restore_irq_mask = rmi_driver_irq_restore,
	.remove = __devexit_p(rmi_driver_remove)
};

/* sysfs show and store fns for driver attributes */
static ssize_t rmi_driver_hasbsr_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct rmi_device *rmi_dev;
	struct rmi_driver_data *data;
	rmi_dev = to_rmi_device(dev);
	data = rmi_get_driverdata(rmi_dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", has_bsr(data));
}

static ssize_t rmi_driver_bsr_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct rmi_device *rmi_dev;
	struct rmi_driver_data *data;
	rmi_dev = to_rmi_device(dev);
	data = rmi_get_driverdata(rmi_dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", data->bsr);
}

static ssize_t rmi_driver_bsr_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int retval;
	unsigned long val;
	struct rmi_device *rmi_dev;
	struct rmi_driver_data *data;

	rmi_dev = to_rmi_device(dev);
	data = rmi_get_driverdata(rmi_dev);

	/* need to convert the string data to an actual value */
	retval = strict_strtoul(buf, 10, &val);
	if (retval < 0) {
		dev_err(dev, "Invalid value '%s' written to BSR.\n", buf);
		return -EINVAL;
	}

	retval = rmi_write(rmi_dev, BSR_LOCATION, (unsigned char)val);
	if (retval) {
		dev_err(dev, "%s : failed to write bsr %u to 0x%x\n",
			__func__, (unsigned int)val, BSR_LOCATION);
		return retval;
	}

	data->bsr = val;

	return count;
}

static void disable_sensor(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);

	rmi_dev->phys->disable_device(rmi_dev->phys);

	data->enabled = false;
}

static int enable_sensor(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);
	int retval = 0;

	retval = rmi_dev->phys->enable_device(rmi_dev->phys);
	/* non-zero means error occurred */
	if (retval)
		return retval;

	data->enabled = true;

	return 0;
}

static ssize_t rmi_driver_enabled_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct rmi_device *rmi_dev;
	struct rmi_driver_data *data;

	rmi_dev = to_rmi_device(dev);
	data = rmi_get_driverdata(rmi_dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", data->enabled);
}

static ssize_t rmi_driver_enabled_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int retval;
	int new_value;
	struct rmi_device *rmi_dev;
	struct rmi_driver_data *data;

	rmi_dev = to_rmi_device(dev);
	data = rmi_get_driverdata(rmi_dev);

	if (sysfs_streq(buf, "0"))
		new_value = false;
	else if (sysfs_streq(buf, "1"))
		new_value = true;
	else
		return -EINVAL;

	if (new_value) {
		retval = enable_sensor(rmi_dev);
		if (retval) {
			dev_err(dev, "Failed to enable sensor, code=%d.\n",
				retval);
			return -EIO;
		}
	} else {
		disable_sensor(rmi_dev);
	}

	return count;
}

#if REGISTER_DEBUG
static ssize_t rmi_driver_reg_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int retval;
	unsigned int address;
	unsigned int bytes;
	struct rmi_device *rmi_dev;
	struct rmi_driver_data *data;
	u8 readbuf[128];
	unsigned char outbuf[512];
	unsigned char *bufptr = outbuf;
	int i;

	rmi_dev = to_rmi_device(dev);
	data = rmi_get_driverdata(rmi_dev);

	retval = sscanf(buf, "%x %u", &address, &bytes);
	if (retval != 2) {
		dev_err(dev, "Invalid input (code %d) for reg store: %s",
			retval, buf);
		return -EINVAL;
	}
	if (address < 0 || address > 0xFFFF) {
		dev_err(dev, "Invalid address for reg store '%#06x'.\n",
			address);
		return -EINVAL;
	}
	if (bytes < 0 || bytes >= sizeof(readbuf) || address+bytes > 65535) {
		dev_err(dev, "Invalid byte count for reg store '%d'.\n",
			bytes);
		return -EINVAL;
	}

	retval = rmi_read_block(rmi_dev, address, readbuf, bytes);
	if (retval != bytes) {
		dev_err(dev, "Failed to read %d registers at %#06x, code %d.\n",
			bytes, address, retval);
		return retval;
	}

	dev_info(dev, "Reading %d bytes from %#06x.\n", bytes, address);
	for (i = 0; i < bytes; i++) {
		retval = snprintf(bufptr, 4, "%02X ", readbuf[i]);
		if (retval < 0) {
			dev_err(dev, "Failed to format string. Code: %d",
				retval);
			return retval;
		}
		bufptr += retval;
	}
	dev_info(dev, "%s\n", outbuf);

	return count;
}
#endif

#if DELAY_DEBUG
static ssize_t rmi_delay_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int retval;
	struct rmi_device *rmi_dev;
	struct rmi_device_platform_data *pdata;
	unsigned int new_read_delay;
	unsigned int new_write_delay;
	unsigned int new_block_delay;
	unsigned int new_pre_delay;
	unsigned int new_post_delay;

	retval = sscanf(buf, "%u %u %u %u %u", &new_read_delay,
			&new_write_delay, &new_block_delay,
			&new_pre_delay, &new_post_delay);
	if (retval != 5) {
		dev_err(dev, "Incorrect number of values provided for delay.");
		return -EINVAL;
	}
	if (new_read_delay < 0) {
		dev_err(dev, "Byte delay must be positive microseconds.\n");
		return -EINVAL;
	}
	if (new_write_delay < 0) {
		dev_err(dev, "Write delay must be positive microseconds.\n");
		return -EINVAL;
	}
	if (new_block_delay < 0) {
		dev_err(dev, "Block delay must be positive microseconds.\n");
		return -EINVAL;
	}
	if (new_pre_delay < 0) {
		dev_err(dev,
			"Pre-transfer delay must be positive microseconds.\n");
		return -EINVAL;
	}
	if (new_post_delay < 0) {
		dev_err(dev,
			"Post-transfer delay must be positive microseconds.\n");
		return -EINVAL;
	}

	rmi_dev = to_rmi_device(dev);
	pdata = rmi_dev->phys->platform_data;

	dev_info(dev, "Setting delays to %u %u %u %u %u.\n", new_read_delay,
		 new_write_delay, new_block_delay, new_pre_delay,
		 new_post_delay);
	pdata->spi_data.read_delay_us = new_read_delay;
	pdata->spi_data.write_delay_us = new_write_delay;
	pdata->spi_data.block_delay_us = new_block_delay;
	pdata->spi_data.pre_delay_us = new_pre_delay;
	pdata->spi_data.post_delay_us = new_post_delay;

	return count;
}

static ssize_t rmi_delay_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct rmi_device *rmi_dev;
	struct rmi_device_platform_data *pdata;

	rmi_dev = to_rmi_device(dev);
	pdata = rmi_dev->phys->platform_data;

	return snprintf(buf, PAGE_SIZE, "%d %d %d %d %d\n",
		pdata->spi_data.read_delay_us, pdata->spi_data.write_delay_us,
		pdata->spi_data.block_delay_us,
		pdata->spi_data.pre_delay_us, pdata->spi_data.post_delay_us);
}
#endif

static ssize_t rmi_driver_phys_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct rmi_device *rmi_dev;
	struct rmi_phys_info *info;

	rmi_dev = to_rmi_device(dev);
	info = &rmi_dev->phys->info;

	return snprintf(buf, PAGE_SIZE, "%-5s %ld %ld %ld %ld %ld %ld %ld\n",
		 info->proto ? info->proto : "unk",
		 info->tx_count, info->tx_bytes, info->tx_errs,
		 info->rx_count, info->rx_bytes, info->rx_errs,
		 info->attn_count);
}

static ssize_t rmi_driver_version_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n",
		 RMI_DRIVER_VERSION_STRING);
}

static int __init rmi_driver_init(void)
{
	return rmi_register_driver(&sensor_driver);
}

static void __exit rmi_driver_exit(void)
{
	rmi_unregister_driver(&sensor_driver);
}

module_init(rmi_driver_init);
module_exit(rmi_driver_exit);

MODULE_AUTHOR("Christopher Heiny <cheiny@synaptics.com");
MODULE_DESCRIPTION("RMI generic driver");
MODULE_LICENSE("GPL");

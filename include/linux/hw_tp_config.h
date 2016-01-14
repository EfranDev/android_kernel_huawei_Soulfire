/*
 * Copyright (c) 2012 Huawei Device Company
 *
 * This file include Touch fw and config for solving touch performance.
 * 
 * fw be named as followed:
 *
 * config be named as followed:
 *
 */
#include <linux/types.h>
#include <linux/list.h>
#include <linux/touch_platform_config.h>

#define SYN_CONFIG_SIZE 32 * 16
#define CURRENT_PR_VERSION  1191601
#define TP_ID0  127
#define TP_ID1  128

#define IC_TYPE_2202 2202
#define IC_TYPE_3200 3200
#define IC_TYPE_3207 3207

struct syn_version_config
{
	uint32_t syn_firmware_version;
	uint32_t syn_moudel_version;
	u16 syn_ic_name;
};

/* Y300 and G510 Defined separately */

typedef enum 
{
   TP_COB_ID_OFILM = 0x0000, //ID0 low   , ID1  low
   TP_COB_ID_JUNDA = 0x000f, //ID0 float , ID1  low
   TP_COB_ID_TRULY = 0x000c, //ID1 float , ID0  low
   TP_COB_ID_OTHER = 0x0003, //ID0 float , ID1  float
}hw_tp_id_cob;

typedef enum 
{
	TP_COF_ID_OFILM = 0, 
	TP_COF_ID_ECW = 1, 
	TP_COF_ID_TRULY = 2, 
	TP_COF_ID_MUTTO = 3,
	TP_COF_ID_GIS = 4, 
	TP_COF_ID_JUNDA = 5, 
	TP_COF_ID_LENSONE = 6, 
	TP_COF_ID_UNKNOW = 7,
}hw_tp_id_cof;

uint8_t *get_tp_version_config(int module_id,u16 ic_type);

uint8_t* get_tp_lockdown_config(void);
u16 *get_f54_raw_cap_uplimit(int module_id,u16 ic_type);
u16 *get_f54_raw_cap_lowlimit(int module_id,u16 ic_type);
int get_touch_resolution(struct tp_resolution_conversion *tp_resolution_type);
/*get COB tp module id*/
int get_tp_id(void);


#ifndef BUILD_LK
#include <linux/string.h>
#else
#include <string.h>
#endif 

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#elif (defined BUILD_UBOOT)
#include <asm/arch/mt6577_gpio.h>
#else
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#endif

#include "lcm_drv.h"

#ifndef BUILD_LK
static bool fgisFirst = TRUE;
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define FRAME_WIDTH  (800)
#define FRAME_HEIGHT (1280)

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------
static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#ifdef GPIO_LCM_PWR
#define GPIO_LCD_PWR_EN      GPIO_LCM_PWR
#else
#define GPIO_LCD_PWR_EN      0xFFFFFFFF
#endif


#ifdef GPIO_LCM_RST
#define GPIO_LCD_RST      GPIO_LCM_RST
#else
#define GPIO_LCD_RST      0xFFFFFFFF
#endif

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	  lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)				lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)																			lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)									lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)																				lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   						lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)   

#define   LCM_DSI_CMD_MODE	0

#define REGFLAG_DELAY             							0xAC
#define REGFLAG_END_OF_TABLE      							0xAD   // END OF REGISTERS MARKER
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------
struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};
//update initial param for IC nt35523 0.01
static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28,0,{}},
	{0x10,0,{}},
	{REGFLAG_DELAY, 120, {}}
};
static struct LCM_setting_table lcm_initialization_setting[] = {
{0xFF,4,{0xAA,0x55,0xA5,0x80}},//========== Internal setting ==========
{0x6F,2,{0x11,0x00}},
{0xF7,2,{0x20,0x00}},
{0x6F,1,{0x06}},
{0xF7,1,{0xA0}},
{0x6F,1,{0x19}},
{0xF7,1,{0x12}},
{0xF4,1,{0x03}},
// new Vcom floating
{0x6F,1,{0x08}},
{0xFA,1,{0x40}},
{0x6F,1,{0x11}},
{0xF3,1,{0x01}},
// for1, abnormal power off
{0x6F,1,{0x06}},
{0xFC,1,{0x03}},
{0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},//========== page0 relative ==========
{0xB1,2,{0x68,0x01}},
{0xB6,1,{0x08}},
{0x6F,1,{0x02}},
{0xB8,1,{0x08}},
{0xBB,2,{0x54,0x44}},
{0xBC,2,{0x05,0x05}},
{0xC7,1,{0x01}},
{0xBD,5,{0x02,0xB0,0x1E,0x1E,0x00}},
{0xC5,2,{0x01,0x07}},
{0xC8,1,{0x80}},
{0xF0,5,{0x55,0xAA,0x52,0x08,0x01}},//========== page1 relative ==========
{0xB0,2,{0x05,0x05}},
{0xB1,2,{0x05,0x05}},
//new}1,
{0xB2,2,{0x00,0x00}},
//{0xB8,2,{0x03,0x03}},
{0xBC,2,{0x90,0x01}},
{0xBD,2,{0x90,0x01}},
{0xCA,1,{0x00}},
{0xC0,1,{0x04}},
{0xBE,1,{0x29}},
{0xB3,2,{0x28,0x28}},
{0xB4,2,{0x12,0x12}},
{0xB9,2,{0x35,0x35}},
{0xBA,2,{0x25,0x25}},
{0xF0,5,{0x55,0xAA,0x52,0x08,0x02}},//========== page2 relative ==========
{0xEE,1,{0x01}},
{0xEF,4,{0x09,0x06,0x15,0x18}},
{0xB0,6,{0x00,0x00,0x00,0x24,0x00,0x55}},
{0x6F,1,{0x06}},
{0xB0,6,{0x00,0x77,0x00,0x94,0x00,0xC0}},
{0x6F,1,{0x0C}},
{0xB0,4,{0x00,0xE3,0x01,0x1A}},
{0xB1,6,{0x01,0x46,0x01,0x88,0x01,0xBC}},
{0x6F,1,{0x06}},
{0xB1,6,{0x02,0x0B,0x02,0x4B,0x02,0x4D}},
{0x6F,1,{0x0C}},
{0xB1,4,{0x02,0x88,0x02,0xC9}},
{0xB2,6,{0x02,0xF3,0x03,0x29,0x03,0x4E}},
{0x6F,1,{0x06}},
{0xB2,6,{0x03,0x7D,0x03,0x9B,0x03,0xBE}},
{0x6F,1,{0x0C}},
{0xB2,4,{0x03,0xD3,0x03,0xE9}},
{0xB3,4,{0x03,0xFB,0x03,0xFF}},
{0xF0,5,{0x55,0xAA,0x52,0x08,0x06}},//========== GOA relative ==========
{0xB0,2,{0x0B,0x2E}},
{0xB1,2,{0x2E,0x2E}},
{0xB2,2,{0x2E,0x09}},
{0xB3,2,{0x2A,0x29}},
{0xB4,2,{0x1B,0x19}},
{0xB5,2,{0x17,0x15}},
{0xB6,2,{0x13,0x11}},
{0xB7,2,{0x01,0x2E}},
{0xB8,2,{0x2E,0x2E}},
{0xB9,2,{0x2E,0x2E}},
{0xBA,2,{0x2E,0x2E}},
{0xBB,2,{0x2E,0x2E}},
{0xBC,2,{0x2E,0x00}},
{0xBD,2,{0x10,0x12}},
{0xBE,2,{0x14,0x16}},
{0xBF,2,{0x18,0x1A}},
{0xC0,2,{0x29,0x2A}},
{0xC1,2,{0x08,0x2E}},
{0xC2,2,{0x2E,0x2E}},
{0xC3,2,{0x2E,0x0A}},
{0xE5,2,{0x2E,0x2E}},
{0xC4,2,{0x0A,0x2E}},
{0xC5,2,{0x2E,0x2E}},
{0xC6,2,{0x2E,0x00}},
{0xC7,2,{0x2A,0x29}},
{0xC8,2,{0x10,0x12}},
{0xC9,2,{0x14,0x16}},
{0xCA,2,{0x18,0x1A}},
{0xCB,2,{0x08,0x2E}},
{0xCC,2,{0x2E,0x2E}},
{0xCD,2,{0x2E,0x2E}},
{0xCE,2,{0x2E,0x2E}},
{0xCF,2,{0x2E,0x2E}},
{0xD0,2,{0x2E,0x09}},
{0xD1,2,{0x1B,0x19}},
{0xD2,2,{0x17,0x15}},
{0xD3,2,{0x13,0x11}},
{0xD4,2,{0x29,0x2A}},
{0xD5,2,{0x01,0x2E}},
{0xD6,2,{0x2E,0x2E}},
{0xD7,2,{0x2E,0x0B}},
{0xE6,2,{0x2E,0x2E}},
{0xD8,5,{0x00,0x00,0x00,0x00,0x00}},
{0xD9,5,{0x00,0x00,0x00,0x00,0x00}},
{0xE7,1,{0x00}},
{0xF0,5,{0x55,0xAA,0x52,0x08,0x03}},// PAGE3 :
{0xB0,2,{0x20,0x00}},
{0xB1,2,{0x20,0x00}},
{0xB2,5,{0x05,0x00,0x00,0x00,0x00}},
{0xB6,5,{0x05,0x00,0x00,0x00,0x00}},
{0xB7,5,{0x05,0x00,0x00,0x00,0x00}},
{0xBA,5,{0x57,0x00,0x00,0x00,0x00}},
{0xBB,5,{0x57,0x00,0x00,0x00,0x00}},
{0xC0,4,{0x00,0x00,0x00,0x00}},
{0xC1,4,{0x00,0x00,0x00,0x00}},
{0xC4,1,{0x60}},
{0xC5,1,{0x40}},
{0xF0,5,{0x55,0xAA,0x52,0x08,0x05}},// PAGE5 :
{0xBD,5,{0x03,0x01,0x03,0x03,0x03}},
{0xB0,2,{0x17,0x06}},
{0xB1,2,{0x17,0x06}},
{0xB2,2,{0x17,0x06}},
{0xB3,2,{0x17,0x06}},
{0xB4,2,{0x17,0x06}},
{0xB5,2,{0x17,0x06}},
{0xB8,1,{0x00}},
{0xB9,1,{0x00}},
{0xBA,1,{0x00}},
{0xBB,1,{0x02}},
{0xBC,1,{0x00}},
{0xC0,1,{0x07}},
{0xC4,1,{0x80}},
{0xC5,1,{0xA4}},
{0xC8,2,{0x05,0x30}},
{0xC9,2,{0x01,0x31}},
{0xCC,3,{0x00,0x00,0x3C}},
{0xCD,3,{0x00,0x00,0x3C}},
{0xD1,5,{0x00,0x05,0x09,0x07,0x10}},
{0xD2,5,{0x00,0x05,0x0E,0x07,0x10}},
{0xE5,1,{0x06}},
{0xE6,1,{0x06}},
{0xE7,1,{0x06}},
{0xE8,1,{0x06}},
{0xE9,1,{0x06}},
{0xEA,1,{0x06}},
{0xED,1,{0x30}},
{0x6F,1,{0x11}},
{0xF3,1,{0x01}},//reload setting
{0x35,1,{0x00}},
	
{0x11, 0, {0x00}},// Normal Display
{REGFLAG_DELAY, 120, {}},

{0x29, 0, {0x00}},// Display ON
{REGFLAG_DELAY, 100, {}},
{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 0, {0x00}},
    {REGFLAG_DELAY, 100, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},

    // Sleep Mode On
	{0x10, 0, {0x00}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++)
    {
        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd) {

            case REGFLAG_DELAY :
                if(table[i].count <= 10)
                    MDELAY(table[i].count);
                else
                    MDELAY(table[i].count);
                break;

            case REGFLAG_END_OF_TABLE :
                break;

            default:
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;
		
    #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
    #else
		params->dsi.mode   = SYNC_EVENT_VDO_MODE;		//SYNC_EVENT_VDO_MODE;
    #endif
	
		// DSI
		/* Command mode setting */
		// Three lane or Four lane
		params->dsi.LANE_NUM								= LCM_FOUR_LANE;
		
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=256;

		// Video mode setting		
		params->dsi.intermediat_buffer_num = 0;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		params->dsi.word_count=FRAME_WIDTH*3;
		
		params->dsi.vertical_sync_active				= 6;
		params->dsi.vertical_backporch					= 3;
		params->dsi.vertical_frontporch					= 20;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active			= 6;
		params->dsi.horizontal_backporch				= 48;
		params->dsi.horizontal_frontporch				= 16;
		params->dsi.horizontal_active_pixel			= FRAME_WIDTH;
		
    	params->dsi.ssc_disable							= 1;
		params->dsi.PLL_CLOCK = 221;
		
		params->dsi.cont_clock= 0;		
		params->dsi.clk_lp_per_line_enable = 1;		
}

static void lcm_set_gpio_output(unsigned int GPIO, unsigned int output)
{
   mt_set_gpio_mode(GPIO, GPIO_MODE_00);
   mt_set_gpio_dir(GPIO, GPIO_DIR_OUT);
   mt_set_gpio_out(GPIO, (output>0)? GPIO_OUT_ONE: GPIO_OUT_ZERO);
}


static void lcm_init_power(void)
{
#ifdef BUILD_LK 
	printf("[LK/LCM] lcm_init_power() enter\n");
	
#else
	printk("[Kernel/LCM] lcm_init_power() enter\n");
	
#endif
//	lcm_set_gpio_output(GPIO_LCD_PWR_EN, GPIO_OUT_ONE);
//	MDELAY(20);
}

static void lcm_suspend_power(void)
{
#ifdef BUILD_LK 
	printf("[LK/LCM] lcm_suspend_power() enter\n");

#else
	printk("[Kernel/LCM] lcm_suspend_power() enter\n");
#endif
// 	lcm_set_gpio_output(GPIO_LCD_PWR_EN, GPIO_OUT_ZERO);
}

static void lcm_resume_power(void)
{
#ifdef BUILD_LK 
	printf("[LK/LCM] lcm_resume_power() enter\n");

#else
	printk("[Kernel/LCM] lcm_resume_power() enter\n");

#endif
//	lcm_set_gpio_output(GPIO_LCD_PWR_EN, GPIO_OUT_ONE);
//	MDELAY(20);
}

static void lcm_init(void)
{
#ifdef BUILD_LK 
	printf("[LK/LCM] lcm_init() enter\n");
	lcm_set_gpio_output(GPIO_LCD_PWR_EN, GPIO_OUT_ONE);
	MDELAY(20);
	//VDD power on ->VGP3_PMU 1.8V
	upmu_set_rg_vgp3_vosel(3);
	upmu_set_rg_vgp3_en(0x1);

	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	MDELAY(20);
	// when phone initial , config output high, enable backlight drv chip  	
#else
	printk("[Kernel/LCM] lcm_init() enter\n");
#endif
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);	
}


static void lcm_suspend(void)
{
#ifdef BUILD_LK
	printf("[LK/LCM] lcm_suspend() enter\n");
	lcm_set_gpio_output(GPIO_LCD_PWR_EN,GPIO_OUT_ZERO);
	MDELAY(20);		
	//VDD power off ->VGP3_PMU 1.8V
	upmu_set_rg_vgp3_vosel(0);
	upmu_set_rg_vgp3_en(0);	
	MDELAY(20);	
	lcm_set_gpio_output(GPIO_LCD_RST,GPIO_OUT_ZERO);
	MDELAY(20);	
#else
	printk("[Kernel/LCM] lcm_suspend() enter\n");
	lcm_set_gpio_output(GPIO_LCD_PWR_EN,GPIO_OUT_ZERO);
	MDELAY(20);
	//VDD power off ->VGP3_PMU 1.8V
	if (fgisFirst == TRUE) {
		 fgisFirst = FALSE;
		hwPowerOn(MT6323_POWER_LDO_VGP3, VOL_1800, "LCM");
	}
	hwPowerDown(MT6323_POWER_LDO_VGP3, "LCM");
	MDELAY(20);	
	lcm_set_gpio_output(GPIO_LCD_RST,GPIO_OUT_ZERO);
	MDELAY(20);
#endif
 	push_table(lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1); 
//	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_resume(void)
{
#ifdef BUILD_LK 
	printf("[LK/LCM] lcm_resume() enter\n");
	lcm_set_gpio_output(GPIO_LCD_PWR_EN, GPIO_OUT_ONE);
	MDELAY(20);	
	//VGP3_PMU 1.8V
	upmu_set_rg_vgp3_vosel(3);
	upmu_set_rg_vgp3_en(0x1);
	MDELAY(20);
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	MDELAY(20);

#else
	printk("[Kernel/LCM] lcm_resume() enter\n");
	lcm_set_gpio_output(GPIO_LCD_PWR_EN, GPIO_OUT_ONE);
	MDELAY(20);	
	hwPowerOn(MT6323_POWER_LDO_VGP3 , VOL_1800 ,"LCM");
	MDELAY(20);		
	lcm_set_gpio_output(GPIO_LCD_RST, GPIO_OUT_ONE);
	MDELAY(20);

#endif
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
//	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);	
}

#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x00290508; 				//HW bug, so need send one HS packet
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
}
#endif
static unsigned int lcm_esd_check()
{
#ifndef BUILD_LK
	   unsigned char buffer[1];
   unsigned int array[16];


   #ifdef BUILD_LK
	printf("[cabc] otm1287a: lcm_esd_check enter\n");
#else
	printk("[cabc] otm1287a: lcm_esd_check enter\n");
#endif

   array[0] = 0x00013700;// read id return two byte,version and id
   dsi_set_cmdq(array, 1, 1);
//	id = read_reg(0xF4);
   read_reg_v2(0x0A, buffer, 1);
#ifdef BUILD_LK
    printf("lcm_esd_check  0x0A = %x\n",buffer[0]);
#else
    printk("lcm_esd_check  0x0A = %x\n",buffer[0]);
#endif
   if(buffer[0] != 0x9C)
   {
      return 1;
   }

   array[0] = 0x00013700;// read id return two byte,version and id
   dsi_set_cmdq(array, 1, 1);
//	id = read_reg(0xF4);
   read_reg_v2(0x0D, buffer, 1);
#ifdef BUILD_LK
    printf("lcm_esd_check 0x0D =%x\n",buffer[0]);
#else
    printk("lcm_esd_check 0x0D =%x\n",buffer[0]);
#endif
   if(buffer[0] != 0x00)
   {
      return 1;
   }
   array[0] = 0x00013700;// read id return two byte,version and id
   dsi_set_cmdq(array, 1, 1);
//	id = read_reg(0xF4);
   read_reg_v2(0x0E, buffer, 1);
#ifdef BUILD_LK
    printf("lcm_esd_check  0x0E = %x\n",buffer[0]);
#else
    printk("lcm_esd_check  0x0E = %x\n",buffer[0]);
#endif
   if(buffer[0] != 0x80)
   {
      return 1;
   }

#ifdef BUILD_LK
	printf("[cabc] otm1287a: lcm_esd_check exit\n");
#else
	printk("[cabc] otm1287a: lcm_esd_check exit\n");
#endif

   return 0;
#endif
}
static unsigned int lcm_esd_recover()
{
	unsigned int data_array[16];

#ifdef BUILD_LK
    printf("lcm_esd_recover enter");
#else
  printk("lcm_esd_recover enter");
#endif

   lcm_init();
 	data_array[0]=0x00110500;
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(50);
	
	data_array[0]=0x00290500;
	dsi_set_cmdq(&data_array, 1, 1);

	data_array[0]= 0x00023902;
	data_array[1]= 0xFF51;
	dsi_set_cmdq(&data_array, 2, 1);
	MDELAY(10);

    return TRUE;
}

LCM_DRIVER nt35521_wxga_dsi_vdo_lcm_drv = 
{
	.name			= "nt35521_wxga_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.init_power		= lcm_init_power,
	.resume_power   = lcm_resume_power,
	.suspend_power  = lcm_suspend_power,
	.esd_check 		= lcm_esd_check,
	.esd_recover	= lcm_esd_recover,	
#if (LCM_DSI_CMD_MODE)
	.update         = lcm_update,
#endif
};

#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/switch.h>
#include <mach/param.h> // __SWITCH_SEL
#include <mach/fsa9480_i2c.h>


#if 1 // P1-KOR
#define _SAMSUNG_KIES_ACM_UMS_MODE_
#define _SAMSUNG_KIES_ACM_UMS_ADB_MODE_
#define _UART_HW_BUG_
#endif
//#define _SUPPORT_SAMSUNG_AUTOINSTALLER_

#if 1
	#define dmsg(arg,...)	printk("[USB_SWITCH] %s(%d): "arg,__FUNCTION__,__LINE__, ## __VA_ARGS__)
#else
	#define dmsg(arg,...) 	{}
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* MODEM USB_SEL Pin control */
/* 1 : PDA, 2 : MODEM */
#define SWITCH_PDA			1
#define SWITCH_MODEM		2
#ifdef _UART_HW_BUG_
#define SWITCH_UART			3	// For P1 KOR
#endif

/* switch selection */
#define USB_SEL_MASK  				(1 << 0)
#define UART_SEL_MASK				(1 << 1)
#define USB_SAMSUNG_KIES_MASK		(1 << 2)
#define USB_UMS_MASK				(1 << 3)
#define USB_MTP_MASK				(1 << 4)
#define USB_VTP_MASK				(1 << 5)
#define USB_ASKON_MASK				(1 << 6)
#define USB_ADB_MASK				(1 << 7)
#define USB_DM_MASK					(1 << 8)
#define USB_ACM_MASK				(1 << 9)
#ifdef _SUPPORT_SAMSUNG_AUTOINSTALLER_
#define USB_SAMSUNG_KIES_REAL_MASK	(1 << 10)
#endif

#define DRIVER_NAME  "usb_mass_storage"



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// drivers/switch/fsa9480_i2c.c
extern void FSA9480_Set_ID_Switch(int sel);
extern void ap_usb_power_on(int set_vaue);
extern int get_usb_cable_state(void);
extern u8 FSA9480_Get_USB_Status(void);
//extern u8 FSA9480_Get_JIG_Status(void);
extern u8 FSA9480_Get_JIG_UART_Status(void);
extern void FSA9480_Enable_CP_USB(u8 enable);
extern u8 get_switch_ID(void);
#if 1 // P1-KOR
extern void FSA9480_InitDevice(void);
#endif

// drivers/power/s5pc110_battery.c
extern unsigned int charging_mode_get(void);

// drivers/usb/gadget/s3c_otg_udc.c
extern int s3c_usb_cable(int connected);

// drivers/usb/gadget/adb_ums_acm_mtp_rndis.c
extern int usb_mtp_select(int disable);
extern int usb_switch_select(int enable);
extern int askon_switch_select(int enable);

extern void otg_phy_init(void);
extern void otg_phy_off(void);



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

extern struct device *switch_dev;
extern int askonstatus;
extern int inaskonstatus;
extern u8 MicroJigUARTOffStatus;
extern int usb_state;

int BOOTUP = 1;
EXPORT_SYMBOL(BOOTUP);
int oldusbstatus;
EXPORT_SYMBOL(oldusbstatus);
int mtp_mode_on = 0;
EXPORT_SYMBOL(mtp_mode_on);
int connectivity_switching_init_state=0;
EXPORT_SYMBOL(connectivity_switching_init_state);
int currentusbstatus = 0;//USBSTATUS_SAMSUNG_KIES;
EXPORT_SYMBOL(currentusbstatus);


static int usb_path = 0;
static int switch_sel = 0;

static u8 switchinginitvalue[12];
static u8 uart_message[6];
static u8 usb_message[5];
static int factoryresetstatus=0;

static int uart_current_owner = 1;
static int g_tethering;
static int mtpinitstatus=0;
static int askinitstatus=0;

int mtp_power_off = 0;

struct switch_dev 		indicator_dev;
struct delayed_work 	switch_init_work;



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef _SUPPORT_SAMSUNG_AUTOINSTALLER_
bool IsKiesCurrentUsbStatus(void)
{
	if( (currentusbstatus == USBSTATUS_SAMSUNG_KIES)
#ifndef _SAMSUNG_KIES_ACM_UMS_MODE_
	|| (currentusbstatus == USBSTATUS_SAMSUNG_KIES_REAL)
#endif
	) {
		return true;
	}

	return false;
}
EXPORT_SYMBOL(IsKiesCurrentUsbStatus);
#endif

void UsbIndicator(u8 state)
{
	switch_set_state(&indicator_dev, state);
}
EXPORT_SYMBOL(UsbIndicator);

// TODO : remove this
static void usb_api_set_usb_switch(USB_SWITCH_MODE usb_switch)
{
	if(usb_switch == USB_SW_CP)
	{
		//USB_SEL GPIO Set High => CP USB enable
		FSA9480_Enable_CP_USB(1);
	}
	else if(usb_switch == USB_SW_AP)
	{
		//USB_SEL GPIO Set Low => AP USB enable
		FSA9480_Enable_CP_USB(0);
	}
#ifdef _UART_HW_BUG_
	else if(usb_switch == USB_SW_AP+1/*USB_SW_UART*/)  // For P1 KOR
	{
		FSA9480_Enable_CP_USB(2);  // USB to UART
	}
#endif
}

static void Ap_Cp_Switch_Config(u16 ap_cp_mode)
{
	switch (ap_cp_mode) {
		case AP_USB_MODE:
			usb_path=1;
			usb_api_set_usb_switch(USB_SW_AP);
			break;
		case AP_UART_MODE:
			gpio_set_value(GPIO_UART_SEL, 1);
			break;
		case CP_USB_MODE:
			usb_path=2;
			usb_api_set_usb_switch(USB_SW_CP);
			break;
		case CP_UART_MODE:
			gpio_set_value(GPIO_UART_SEL, 0);			
			break;
#ifdef _UART_HW_BUG_
		case (CP_UART_MODE+1): // UART_USB_MODE:  // For P1 KOR
			usb_api_set_usb_switch(USB_SW_CP+1/*USB_SW_UART*/);
			break;
#endif
		default:
			dmsg("Ap_Cp_Switch_Config error");
	}
		
}

static void usb_switching_value_update(int value)
{
	if(value == SWITCH_PDA)
		usb_message[0] = 'A';
	else
		usb_message[0] = 'C';		

	usb_message[1] = 'P';
	usb_message[2] = 'U';
	usb_message[3] = 'S';
	usb_message[4] = 'B';

}

static void uart_switching_value_update(int value)
{
	if(value == SWITCH_PDA)
		uart_message[0] = 'A';
	else
		uart_message[0] = 'C';

	uart_message[1] = 'P';
	uart_message[2] = 'U';
	uart_message[3] = 'A';
	uart_message[4] = 'R';
	uart_message[5] = 'T';

}

static void switching_value_update(void)
{
	int index;
	
	for(index=0;index<5;index++)
		switchinginitvalue[index] = usb_message[index];

	for(index=5;index<11;index++)
		switchinginitvalue[index] = uart_message[index-5];

	switchinginitvalue[11] = '\0';

}

static ssize_t factoryreset_value_store(struct device *dev, struct device_attribute *attr,	const char *buf, size_t size)
{
	if(strncmp(buf, "FACTORYRESET", 12) == 0 || strncmp(buf, "factoryreset", 12) == 0)
		factoryresetstatus = 0xAE;

	return size;
}

static DEVICE_ATTR(FactoryResetValue, S_IRUGO |S_IWUGO | S_IRUSR | S_IWUSR, NULL, factoryreset_value_store);


/* for sysfs control (/sys/class/sec/switch/usb_sel) */
static ssize_t usb_sel_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "USB Switch : %s\n", usb_path==SWITCH_PDA?"PDA":"MODEM");
}


void usb_switch_mode(int sel)
{
	if (sel == SWITCH_PDA)
	{
		dmsg("Path = PDA\n");
		Ap_Cp_Switch_Config(AP_USB_MODE);
	}
	else if (sel == SWITCH_MODEM) 
	{
		dmsg("Path = MODEM\n");
		Ap_Cp_Switch_Config(CP_USB_MODE);
	}
#ifdef _UART_HW_BUG_
	else if (sel == SWITCH_UART)  // For P1 KOR
	{
		dmsg("Path = UART\n");
		Ap_Cp_Switch_Config(CP_USB_MODE+1/*UART_USB_MODE*/);
	}
#endif
	else
		dmsg("Invalid mode...\n");
}
EXPORT_SYMBOL(usb_switch_mode);


static void microusb_uart_status(int status)
{
	int switch_sel;
	int uart_sel;
	int usb_sel;

	if(!FSA9480_Get_JIG_UART_Status())	
		return;
	
	if (sec_get_param_value)
		sec_get_param_value(__SWITCH_SEL, &switch_sel);

	uart_sel = (switch_sel & (int)(UART_SEL_MASK)) >> 1;
	usb_sel = switch_sel & (int)(USB_SEL_MASK);

	if(status) {
		if(uart_sel)
			Ap_Cp_Switch_Config(AP_UART_MODE);	
		else
			Ap_Cp_Switch_Config(CP_UART_MODE);	
	}
	else {
		if(!usb_sel)
			Ap_Cp_Switch_Config(AP_USB_MODE);
	}
}


static ssize_t usb_sel_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	dmsg("\n");

	if (sec_get_param_value)
		sec_get_param_value(__SWITCH_SEL, &switch_sel);

	if(strncmp(buf, "PDA", 3) == 0 || strncmp(buf, "pda", 3) == 0)
	{
		usb_switch_mode(SWITCH_PDA);
		usb_switching_value_update(SWITCH_PDA);
		switch_sel |= USB_SEL_MASK;
	}

	if(strncmp(buf, "MODEM", 5) == 0 || strncmp(buf, "modem", 5) == 0)
	{
		usb_switch_mode(SWITCH_MODEM);
		usb_switching_value_update(SWITCH_MODEM);		
		switch_sel &= ~USB_SEL_MASK;
	}

#ifdef _UART_HW_BUG_
	if(strncmp(buf, "UART", 4) == 0 || strncmp(buf, "uart", 4) == 0)  // For P1 KOR
	{
		usb_switch_mode(SWITCH_UART);
	}
#endif

	switching_value_update();

	if (sec_set_param_value)
		sec_set_param_value(__SWITCH_SEL, &switch_sel);

	microusb_uart_status(0);

	return size;
}

static DEVICE_ATTR(usb_sel, S_IRUGO |S_IWUGO | S_IRUSR | S_IWUSR, usb_sel_show, usb_sel_store);


void usb_switch_state(void)
{
	int usb_sel = 0;
	dmsg("\n");

	if(!connectivity_switching_init_state)
		return;

	if (sec_get_param_value)
		sec_get_param_value(__SWITCH_SEL, &switch_sel);

	printk("\n[WJ] %s, %s, switch_sel=%d\n", __FILE__, __FUNCTION__, switch_sel);

	/* for KIES_REAL state */
	/*
	if (switch_sel & USB_SAMSUNG_KIES_MASK) {
		if( currentusbstatus == USBSTATUS_ADB )
		{
			printk("[USB] %s, currentusbstatus is USBSTATUS_ADB\n", __func__);
		}
		else
		{
			printk("[USB] %s, call usb_switch_select for KIES\n", __func__);
			usb_switch_select(USBSTATUS_SAMSUNG_KIES);
		}
	}
	else if(switch_sel & USB_ASKON_MASK)
	{
		if( currentusbstatus == USBSTATUS_ADB )
		{
			printk("[USB] %s, currentusbstatus is USBSTATUS_ADB\n", __func__);
		}
		else
		{
			printk("[USB] %s, call usb_switch_select for ASKON\n", __func__);
			usb_switch_select(USBSTATUS_ASKON);
		}
	}
	*/
	

	usb_sel = switch_sel & (int)(USB_SEL_MASK);
	
	if(usb_sel) {
		usb_switch_mode(SWITCH_PDA);
		usb_switching_value_update(SWITCH_PDA);
	}
	else {
		usb_switch_mode(SWITCH_MODEM);
		usb_switching_value_update(SWITCH_MODEM);
	}
}

void uart_insert_switch_state(void)
{
	int usb_sel = 0;

	if(!connectivity_switching_init_state)
		return;

	if (sec_get_param_value)
		sec_get_param_value(__SWITCH_SEL, &switch_sel);

	usb_sel = switch_sel & (int)(USB_SEL_MASK);

	if(!usb_sel)
		Ap_Cp_Switch_Config(AP_USB_MODE);
}

void uart_remove_switch_state(void)
{
	int usb_sel = 0;

	if(!connectivity_switching_init_state)
		return;

	if (sec_get_param_value)
		sec_get_param_value(__SWITCH_SEL, &switch_sel);

	usb_sel = switch_sel & (int)(USB_SEL_MASK);

	if(usb_sel)
		Ap_Cp_Switch_Config(AP_USB_MODE);
	else
		Ap_Cp_Switch_Config(CP_USB_MODE);

}


/**********************************************************************
*    Name         : usb_state_show()
*    Description : for sysfs control (/sys/class/sec/switch/usb_state)
*                        return usb state using fsa9480's device1 and device2 register
*                        this function is used only when NPS want to check the usb cable's state.
*    Parameter   :
*                       
*                       
*    Return        : USB cable state's string
*                        USB_STATE_CONFIGURED is returned if usb cable is connected
***********************************************************************/
static ssize_t usb_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int cable_state = get_usb_cable_state();

	return sprintf(buf, "%s\n", (cable_state & (CRB_JIG_USB<<8 | CRA_USB<<0 ))?"USB_STATE_CONFIGURED":"USB_STATE_NOTCONFIGURED");

} 


/**********************************************************************
*    Name         : usb_state_store()
*    Description : for sysfs control (/sys/class/sec/switch/usb_state)
*                        noting to do.
*    Parameter   :
*                       
*                       
*    Return        : None
*
***********************************************************************/
static ssize_t usb_state_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	dmsg("\n");
	return 0;
}

/*sysfs for usb cable's state.*/
static DEVICE_ATTR(usb_state, S_IRUGO |S_IWUGO | S_IRUSR | S_IWUSR, usb_state_show, usb_state_store);


static ssize_t uart_switch_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
	if (uart_current_owner)
		return sprintf(buf, "[UART Switch] Current UART owner = PDA \n");
	else			
		return sprintf(buf, "[UART Switch] Current UART owner = MODEM \n");
}

static ssize_t uart_switch_store(struct device *dev, struct device_attribute *attr,	const char *buf, size_t size)
{	

	if (sec_get_param_value)
		sec_get_param_value(__SWITCH_SEL, &switch_sel);

	if (strncmp(buf, "PDA", 3) == 0 || strncmp(buf, "pda", 3) == 0)	{		
		Ap_Cp_Switch_Config(AP_UART_MODE);
		uart_switching_value_update(SWITCH_PDA);
		uart_current_owner = 1;		
		switch_sel |= UART_SEL_MASK;
		printk("[UART Switch] Path : PDA\n");	
	}	

	if (strncmp(buf, "MODEM", 5) == 0 || strncmp(buf, "modem", 5) == 0) {		
		Ap_Cp_Switch_Config(CP_UART_MODE);
		uart_switching_value_update(SWITCH_MODEM);
		uart_current_owner = 0;		
		switch_sel &= ~UART_SEL_MASK;
		printk("[UART Switch] Path : MODEM\n");	
	}

	switching_value_update();	

	if (sec_set_param_value)
		sec_set_param_value(__SWITCH_SEL, &switch_sel);

	return size;
}

static DEVICE_ATTR(uart_sel, S_IRUGO |S_IWUGO | S_IRUSR | S_IWUSR, uart_switch_show, uart_switch_store);

static int UsbMenuSelStore(int sel)
{	
	int switch_sel, ret;

	if (sec_get_param_value)
		sec_get_param_value(__SWITCH_SEL, &switch_sel);	

	if(sel == 0){
		switch_sel &= ~(int)USB_UMS_MASK;
		switch_sel &= ~(int)USB_MTP_MASK;
		switch_sel &= ~(int)USB_VTP_MASK;
		switch_sel &= ~(int)USB_ASKON_MASK;
#ifdef _SUPPORT_SAMSUNG_AUTOINSTALLER_
		switch_sel &= ~(int)USB_SAMSUNG_KIES_REAL_MASK;
#endif
		switch_sel |= (int)USB_SAMSUNG_KIES_MASK;	
	}
	else if(sel == 1){
		switch_sel &= ~(int)USB_UMS_MASK;
		switch_sel &= ~(int)USB_SAMSUNG_KIES_MASK;
		switch_sel &= ~(int)USB_VTP_MASK;
		switch_sel &= ~(int)USB_ASKON_MASK;
#ifdef _SUPPORT_SAMSUNG_AUTOINSTALLER_
		switch_sel &= ~(int)USB_SAMSUNG_KIES_REAL_MASK;
#endif
		switch_sel |= (int)USB_MTP_MASK;		
	}
	else if(sel == 2){
		switch_sel &= ~(int)USB_SAMSUNG_KIES_MASK;
		switch_sel &= ~(int)USB_MTP_MASK;
		switch_sel &= ~(int)USB_VTP_MASK;
		switch_sel &= ~(int)USB_ASKON_MASK;
#ifdef _SUPPORT_SAMSUNG_AUTOINSTALLER_
		switch_sel &= ~(int)USB_SAMSUNG_KIES_REAL_MASK;
#endif
		switch_sel |= (int)USB_UMS_MASK;
	}
	else if(sel == 3){
		switch_sel &= ~(int)USB_UMS_MASK;
		switch_sel &= ~(int)USB_MTP_MASK;
		switch_sel &= ~(int)USB_SAMSUNG_KIES_MASK;
		switch_sel &= ~(int)USB_ASKON_MASK;
#ifdef _SUPPORT_SAMSUNG_AUTOINSTALLER_
		switch_sel &= ~(int)USB_SAMSUNG_KIES_REAL_MASK;
#endif
		switch_sel |= (int)USB_VTP_MASK;	
	}
	else if(sel == 4){
		switch_sel &= ~(int)USB_UMS_MASK;
		switch_sel &= ~(int)USB_MTP_MASK;
		switch_sel &= ~(int)USB_VTP_MASK;
		switch_sel &= ~(int)USB_SAMSUNG_KIES_MASK;
#ifdef _SUPPORT_SAMSUNG_AUTOINSTALLER_
		switch_sel &= ~(int)USB_SAMSUNG_KIES_REAL_MASK;	
#endif
		switch_sel |= (int)USB_ASKON_MASK;	
	}
#ifdef _SUPPORT_SAMSUNG_AUTOINSTALLER_
	// KIES_REAL
	else if(sel == 5){
		switch_sel &= ~(int)USB_UMS_MASK;
		switch_sel &= ~(int)USB_MTP_MASK;
		switch_sel &= ~(int)USB_VTP_MASK;
		switch_sel &= ~(int)USB_ASKON_MASK;		
		switch_sel &= ~(int)USB_SAMSUNG_KIES_MASK;	
		switch_sel |= (int)USB_SAMSUNG_KIES_REAL_MASK;	
	}
#endif

	if (sec_set_param_value)
		sec_set_param_value(__SWITCH_SEL, &switch_sel);


	printk("\n[WJ] %s, %s, switch_sel=%d\n", __FILE__, __FUNCTION__, switch_sel);

	// returns current USB Mode setting...
	ret = switch_sel;
	ret &= ~(UART_SEL_MASK|USB_SEL_MASK);

	dmsg("ret = 0x%x\n", ret);

	return ret;
}

static void PathSelStore(int sel)
{	
	int switch_sel;

	if (sec_get_param_value)
		sec_get_param_value(__SWITCH_SEL, &switch_sel);	

	if(sel == AP_USB_MODE){
		switch_sel |= USB_SEL_MASK;	
	}
	else if(sel == CP_USB_MODE){
		switch_sel &= ~USB_SEL_MASK;		
	}
	else if(sel == AP_UART_MODE){
		switch_sel |= UART_SEL_MASK;
	}
	else if(sel == CP_UART_MODE){
		switch_sel &= ~UART_SEL_MASK;	
	}

	if (sec_set_param_value)
		sec_set_param_value(__SWITCH_SEL, &switch_sel);
}


static ssize_t UsbMenuSel_switch_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
	if (currentusbstatus == USBSTATUS_UMS) 
		return sprintf(buf, "[UsbMenuSel] UMS\n");

#ifdef _SUPPORT_SAMSUNG_AUTOINSTALLER_
	else if (currentusbstatus == USBSTATUS_SAMSUNG_KIES) 
		return sprintf(buf, "[UsbMenuSel] UMS_CDFS\n");

	else if (currentusbstatus == USBSTATUS_SAMSUNG_KIES_REAL) 
		return sprintf(buf, "[UsbMenuSel] ACM_MTP\n");
#else
#ifdef _SAMSUNG_KIES_ACM_UMS_MODE_
	else if (currentusbstatus == USBSTATUS_SAMSUNG_KIES) 
		return sprintf(buf, "[UsbMenuSel] ACM_ADB_UMS\n");
#else
	else if (currentusbstatus == USBSTATUS_SAMSUNG_KIES)
		return sprintf(buf, "[UsbMenuSel] ACM_MTP\n");
#endif
#endif

	else if (currentusbstatus == USBSTATUS_MTPONLY) 
		return sprintf(buf, "[UsbMenuSel] MTP\n");

	else if (currentusbstatus == USBSTATUS_ASKON) 
		return sprintf(buf, "[UsbMenuSel] ASK\n");

	else if (currentusbstatus == USBSTATUS_VTP) 
		return sprintf(buf, "[UsbMenuSel] VTP\n");

	else if (currentusbstatus == USBSTATUS_ADB) 
		return sprintf(buf, "[UsbMenuSel] ACM_ADB_UMS\n");

	return 0;
}


static ssize_t UsbMenuSel_switch_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	dmsg("buf=%s\n", buf);

#ifdef _SAMSUNG_KIES_ACM_UMS_ADB_MODE_
	askonstatus = 0;
#endif

#ifdef _SUPPORT_SAMSUNG_AUTOINSTALLER_
	if (strncmp(buf, "KIES_REAL", 9) == 0)
	{
		//UsbMenuSelStore(0);		
		usb_switch_select(USBSTATUS_SAMSUNG_KIES_REAL);
	}
	else
#endif
	if (strncmp(buf, "KIES", 4) == 0)
	{
		UsbMenuSelStore(0);
		usb_switch_select(USBSTATUS_SAMSUNG_KIES);
	}

	if (strncmp(buf, "MTP", 3) == 0)
	{
		UsbMenuSelStore(1);					
		usb_switch_select(USBSTATUS_MTPONLY);
	}

	if (strncmp(buf, "UMS", 3) == 0)
	{
		UsbMenuSelStore(2);							
		usb_switch_select(USBSTATUS_UMS);
	}

	if (strncmp(buf, "VTP", 3) == 0)
	{
		UsbMenuSelStore(3);							
		usb_switch_select(USBSTATUS_VTP);
	}

	if (strncmp(buf, "ASKON", 5) == 0)
	{
		UsbMenuSelStore(4);
		usb_switch_select(USBSTATUS_ASKON);
#ifdef _SAMSUNG_KIES_ACM_UMS_ADB_MODE_
		askonstatus = 1;
#endif
	}

	return size;
}

static DEVICE_ATTR(UsbMenuSel, S_IRUGO |S_IWUGO | S_IRUSR | S_IWUSR, UsbMenuSel_switch_show, UsbMenuSel_switch_store);


static ssize_t AskOnStatus_switch_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if(inaskonstatus)
		return sprintf(buf, "%s\n", "Blocking");
	else
		return sprintf(buf, "%s\n", "NonBlocking");
}


static ssize_t AskOnStatus_switch_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{		
	return size;
}

static DEVICE_ATTR(AskOnStatus, S_IRUGO |S_IWUGO | S_IRUSR | S_IWUSR, AskOnStatus_switch_show, AskOnStatus_switch_store);


static ssize_t AskOnMenuSel_switch_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
	return sprintf(buf, "[AskOnMenuSel] Port test ready!! \n");
}

static ssize_t AskOnMenuSel_switch_store(struct device *dev, struct device_attribute *attr,	const char *buf, size_t size)
{		
	if (strncmp(buf, "KIES", 4) == 0)
	{
		askon_switch_select(USBSTATUS_SAMSUNG_KIES);
	}

	if (strncmp(buf, "MTP", 3) == 0)
	{
		askon_switch_select(USBSTATUS_MTPONLY);
	}

	if (strncmp(buf, "UMS", 3) == 0)
	{
		askon_switch_select(USBSTATUS_UMS);
	}

	if (strncmp(buf, "VTP", 3) == 0)
	{
		askon_switch_select(USBSTATUS_VTP);
	}

	return size;
}

static DEVICE_ATTR(AskOnMenuSel, S_IRUGO |S_IWUGO | S_IRUSR | S_IWUSR, AskOnMenuSel_switch_show, AskOnMenuSel_switch_store);


static ssize_t Mtp_switch_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
	return sprintf(buf, "[Mtp] MtpDeviceOn \n");
}

static ssize_t Mtp_switch_store(struct device *dev, struct device_attribute *attr,	const char *buf, size_t size)
{		
	if (strncmp(buf, "Mtp", 3) == 0)
	{
		if(mtp_mode_on)
		{
			printk("[Mtp_switch_store]AP USB power on. \n");
#ifdef VODA
			askon_switch_select(USBSTATUS_SAMSUNG_KIES);
#endif
			ap_usb_power_on(1);
		}
	}
	else if (strncmp(buf, "OFF", 3) == 0)
	{
		printk("[Mtp_switch_store]AP USB power off. \n");
		usb_state = 0;
		usb_mtp_select(1);
	}
	return size;
}

static DEVICE_ATTR(Mtp, S_IRUGO |S_IWUGO | S_IRUSR | S_IWUSR, Mtp_switch_show, Mtp_switch_store);

static ssize_t tethering_switch_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (g_tethering)
		return sprintf(buf, "1\n");
	else			
		return sprintf(buf, "0\n");
}

static ssize_t tethering_switch_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int usbstatus;

	usbstatus = FSA9480_Get_USB_Status();

	dmsg("usbstatus = 0x%x, currentusbstatus = 0x%x\n", usbstatus, currentusbstatus);

	if (strncmp(buf, "1", 1) == 0)
	{
		dmsg("tethering On\n");

		g_tethering = 1;
		usb_switch_select(USBSTATUS_VTP);
		UsbIndicator(0);
	}
	else if (strncmp(buf, "0", 1) == 0)
	{
		dmsg("tethering Off\n");

		g_tethering = 0;
		usb_switch_select(oldusbstatus);
		if(usbstatus)
			UsbIndicator(1);
	}

	return size;
}

static DEVICE_ATTR(tethering, S_IRUGO |S_IWUGO | S_IRUSR | S_IWUSR, tethering_switch_show, tethering_switch_store);

static ssize_t MtpInitStatusSel_switch_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
	if(mtpinitstatus == 2)
		return sprintf(buf, "%s\n", "START");
	else
		return sprintf(buf, "%s\n", "STOP");
}

static ssize_t MtpInitStatusSel_switch_store(struct device *dev, struct device_attribute *attr,	const char *buf, size_t size)
{
	mtpinitstatus = mtpinitstatus + 1;

	return size;
}

static DEVICE_ATTR(MtpInitStatusSel, S_IRUGO |S_IWUGO | S_IRUSR | S_IWUSR, MtpInitStatusSel_switch_show, MtpInitStatusSel_switch_store);


static ssize_t AskInitStatusSel_switch_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
	if(askinitstatus == 2)
		return sprintf(buf, "%s\n", "START");
	else
		return sprintf(buf, "%s\n", "STOP");
}

static ssize_t AskInitStatusSel_switch_store(struct device *dev, struct device_attribute *attr,	const char *buf, size_t size)
{
	askinitstatus = askinitstatus + 1;

	return size;
}

static DEVICE_ATTR(AskInitStatusSel, S_IRUGO |S_IWUGO | S_IRUSR | S_IWUSR, AskInitStatusSel_switch_show, AskInitStatusSel_switch_store);


static ssize_t get_SwitchingInitValue(struct device *dev, struct device_attribute *attr,	char *buf)
{	
	return snprintf(buf, 12, "%s\n", switchinginitvalue);
}

static DEVICE_ATTR(SwitchingInitValue, S_IRUGO, get_SwitchingInitValue, NULL);


static ssize_t ID_switch_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	u8 data = get_switch_ID();

	return sprintf(buf, "[ SWITCH ] %s: MANUALSW2(0x%x) \n", __func__, data);
}

static ssize_t ID_switch_store(struct device *dev, struct device_attribute *attr,	const char *buf, size_t size)
{
	if (strncmp(buf, "OPEN", 4) == 0)
	{
		FSA9480_Set_ID_Switch(0);
	}
	else if (strncmp(buf, "VIDEO", 5) == 0)
	{
		FSA9480_Set_ID_Switch(1);
	}
	else if (strncmp(buf, "BYPASS", 6) == 0)
	{
		FSA9480_Set_ID_Switch(2);
	}

	return size;
}

static DEVICE_ATTR(IDSwitch, S_IRUGO |S_IWUGO | S_IRUSR | S_IWUSR, ID_switch_show, ID_switch_store);

// TODO : remove this.
int  FSA9480_PMIC_CP_USB(void)
{
	int usb_sel = 0;

	if (sec_get_param_value)
		sec_get_param_value(__SWITCH_SEL, &switch_sel);

	usb_sel = switch_sel & (int)(USB_SEL_MASK);

	return usb_sel;
}

static void connectivity_switching_init(struct work_struct *ignored)
{
	int usb_sel, uart_sel, samsung_kies_sel, ums_sel, mtp_sel, vtp_sel, askon_sel;
	int lpm_mode_check = charging_mode_get();
	switch_sel = 0;

	dmsg("\n");

	if (sec_get_param_value) {
		sec_get_param_value(__SWITCH_SEL, &switch_sel);
		cancel_delayed_work(&switch_init_work);
	}
	else {
		schedule_delayed_work(&switch_init_work, msecs_to_jiffies(100));		
		return;
	}

	if(BOOTUP) {
		BOOTUP = 0; 
		otg_phy_init(); //USB Power on after boot up.
	}

	usb_sel = switch_sel & (int)(USB_SEL_MASK);
	uart_sel = (switch_sel & (int)(UART_SEL_MASK)) >> 1;
	samsung_kies_sel = (switch_sel & (int)(USB_SAMSUNG_KIES_MASK)) >> 2;
	ums_sel = (switch_sel & (int)(USB_UMS_MASK)) >> 3;
	mtp_sel = (switch_sel & (int)(USB_MTP_MASK)) >> 4;
	vtp_sel = (switch_sel & (int)(USB_VTP_MASK)) >> 5;
	askon_sel = (switch_sel & (int)(USB_ASKON_MASK)) >> 6;

	printk("\n[WJ] %s, %s, switch_sel=%d\n", __FILE__, __FUNCTION__, switch_sel);

	if( samsung_kies_sel ) currentusbstatus = USBSTATUS_SAMSUNG_KIES;
	else if(ums_sel) currentusbstatus = USBSTATUS_UMS;
	else if(mtp_sel) currentusbstatus = USBSTATUS_MTPONLY;
	else if(askon_sel) currentusbstatus = USBSTATUS_ASKON;

	if((switch_sel == 0x1) || (factoryresetstatus == 0xAE)) {
		PathSelStore(AP_USB_MODE);
		Ap_Cp_Switch_Config(AP_USB_MODE);	
		usb_switching_value_update(SWITCH_PDA);

		PathSelStore(CP_UART_MODE);
		Ap_Cp_Switch_Config(CP_UART_MODE);	
		uart_switching_value_update(SWITCH_MODEM);
	}
	else {
		if(usb_sel) {
			Ap_Cp_Switch_Config(AP_USB_MODE);	
			usb_switching_value_update(SWITCH_PDA);
		}
		else {
			if(MicroJigUARTOffStatus) {
				Ap_Cp_Switch_Config(AP_USB_MODE);
			}
			else {
				Ap_Cp_Switch_Config(CP_USB_MODE);	
				usb_switching_value_update(SWITCH_MODEM);
			}
		}
	
		if(uart_sel) {
			Ap_Cp_Switch_Config(AP_UART_MODE);	
			uart_switching_value_update(SWITCH_PDA);
		}
		else {
			Ap_Cp_Switch_Config(CP_UART_MODE);	
			uart_switching_value_update(SWITCH_MODEM);
		}
	}

/*Turn off usb power when LPM mode*/
	if(lpm_mode_check)
		otg_phy_off();
			
	switching_value_update();

	if((switch_sel == 1) || (factoryresetstatus == 0xAE)) {
		usb_switch_select(USBSTATUS_SAMSUNG_KIES);
#ifndef _SAMSUNG_KIES_ACM_UMS_MODE_
		mtp_mode_on = 1;
		ap_usb_power_on(0);
#endif
		UsbMenuSelStore(0);	
	}
	else {
		if(usb_sel) {
				if(samsung_kies_sel) {
					usb_switch_select(USBSTATUS_SAMSUNG_KIES);
					/*USB Power off till MTP Appl launching*/
					//mtp_mode_on = 1;
					//ap_usb_power_on(0);
				}
				else if(mtp_sel) {
					usb_switch_select(USBSTATUS_MTPONLY);
					/*USB Power off till MTP Appl launching*/
					mtp_mode_on = 1;
					ap_usb_power_on(0);
				}
				else if(ums_sel) {
					usb_switch_select(USBSTATUS_UMS);
				}
			else if(vtp_sel) {
				usb_switch_select(USBSTATUS_VTP);
			}
				else if(askon_sel) {
					usb_switch_select(USBSTATUS_ASKON);
			}
		}
	}

#if 0 // P1-KOR
	if(!FSA9480_Get_USB_Status()) {
		s3c_usb_cable(1);
		mdelay(5);
		s3c_usb_cable(0);
	} else {
		s3c_usb_cable(1);
		indicator_dev.state = 1;
	}
#endif

	dmsg("switch_sel : 0x%x\n", switch_sel);
	microusb_uart_status(1);

	connectivity_switching_init_state=1;

#if 1 // P1-KOR
	FSA9480_InitDevice();
#endif
}


static ssize_t print_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", DRIVER_NAME);
}

static ssize_t print_switch_state(struct switch_dev *sdev, char *buf)
{
	int usbstatus;

	if( mtp_power_off == 1 )
	{
		dmsg("USB power off for MTP\n");
		mtp_power_off = 0;
		return sprintf(buf, "%s\n", "RemoveOffline");
	}

	usbstatus = FSA9480_Get_USB_Status();

	dmsg("usbstatus = 0x%x, currentusbstatus = 0x%x\n", usbstatus, currentusbstatus);

	switch(currentusbstatus)
	{
	case USBSTATUS_UMS:
	case USBSTATUS_ADB:
#ifdef _SAMSUNG_KIES_ACM_UMS_MODE_
	case USBSTATUS_SAMSUNG_KIES:
#endif
		if(usbstatus) {
			return sprintf(buf, "%s\n", "ums online");
		} else {
			return sprintf(buf, "%s\n", "ums offline");
		}
		break;

	case USBSTATUS_VTP:
		return sprintf(buf, "%s\n", "RemoveOffline");
		break;

	default:
 		if(usbstatus) {
			return sprintf(buf, "%s\n", "InsertOffline");
		} else {
			return sprintf(buf, "%s\n", "RemoveOffline");
		}
		break;
	}
}

static int user_switch_create_files(void)
{
	if (device_create_file(switch_dev, &dev_attr_uart_sel) < 0)
		dmsg("Failed to create device file(%s)!\n", dev_attr_uart_sel.attr.name);

	if (device_create_file(switch_dev, &dev_attr_usb_sel) < 0)
		dmsg("Failed to create device file(%s)!\n", dev_attr_usb_sel.attr.name);

	if (device_create_file(switch_dev, &dev_attr_usb_state) < 0)
		dmsg("Failed to create device file(%s)!\n", dev_attr_usb_state.attr.name);

#if 0
	if (device_create_file(switch_dev, &dev_attr_DMport) < 0)
		dmsg("Failed to create device file(%s)!\n", dev_attr_DMport.attr.name);

	if (device_create_file(switch_dev, &dev_attr_DMlog) < 0)
		dmsg("Failed to create device file(%s)!\n", dev_attr_DMlog.attr.name);
#endif	

	if (device_create_file(switch_dev, &dev_attr_UsbMenuSel) < 0)
		dmsg("Failed to create device file(%s)!\n", dev_attr_UsbMenuSel.attr.name);

	if (device_create_file(switch_dev, &dev_attr_AskOnMenuSel) < 0)
		dmsg("Failed to create device file(%s)!\n", dev_attr_AskOnMenuSel.attr.name);

	if (device_create_file(switch_dev, &dev_attr_Mtp) < 0)
		dmsg("Failed to create device file(%s)!\n", dev_attr_Mtp.attr.name);

	if (device_create_file(switch_dev, &dev_attr_SwitchingInitValue) < 0)
		dmsg("Failed to create device file(%s)!\n", dev_attr_SwitchingInitValue.attr.name);		

	if (device_create_file(switch_dev, &dev_attr_FactoryResetValue) < 0)
		dmsg("Failed to create device file(%s)!\n", dev_attr_FactoryResetValue.attr.name);		

	if (device_create_file(switch_dev, &dev_attr_AskOnStatus) < 0)
		dmsg("Failed to create device file(%s)!\n", dev_attr_AskOnStatus.attr.name);			

	if (device_create_file(switch_dev, &dev_attr_MtpInitStatusSel) < 0)
		dmsg("Failed to create device file(%s)!\n", dev_attr_MtpInitStatusSel.attr.name);			

	if (device_create_file(switch_dev, &dev_attr_AskInitStatusSel) < 0)
		dmsg("Failed to create device file(%s)!\n", dev_attr_AskInitStatusSel.attr.name);			

	if (device_create_file(switch_dev, &dev_attr_IDSwitch) < 0)
		dmsg("Failed to create device file(%s)!\n", dev_attr_IDSwitch.attr.name);

	if (device_create_file(switch_dev, &dev_attr_tethering) < 0)
		dmsg("Failed to create device file(%s)!\n", dev_attr_tethering.attr.name);

	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if 1
int user_switch_init(void)
{
	dmsg("user switch init\n");
	user_switch_create_files();

	indicator_dev.name = DRIVER_NAME;
	indicator_dev.print_name = print_switch_name;
	indicator_dev.print_state = print_switch_state;
	switch_dev_register(&indicator_dev);

	INIT_DELAYED_WORK(&switch_init_work, connectivity_switching_init);
	schedule_delayed_work(&switch_init_work, msecs_to_jiffies(200));

	return 0;
}

void user_switch_exit(void)
{
}
#else
static int __init user_switch_init(void)
{
	user_switch_create_files();

	indicator_dev.name = DRIVER_NAME;
	indicator_dev.print_name = print_switch_name;
	indicator_dev.print_state = print_switch_state;
	switch_dev_register(&indicator_dev);

	INIT_DELAYED_WORK(&switch_init_work, connectivity_switching_init);
	schedule_delayed_work(&switch_init_work, msecs_to_jiffies(200));
}

static void __exit user_switch_exit(void)
{
}

module_init(user_switch_init);
module_exit(user_switch_exit);

MODULE_AUTHOR("Samsung eletronics");
MODULE_DESCRIPTION("user switch driver");
MODULE_LICENSE("GPL");
#endif

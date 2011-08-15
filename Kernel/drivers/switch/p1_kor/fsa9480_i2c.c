#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <plat/gpio-cfg.h>
#include <mach/max8998_function.h>
#include <mach/fsa9480_i2c.h>

#if 1 // P1-KOR
#define _UART_HW_BUG_
#define _JIG_USB_HW_WORKAROUND_
#endif

//////////////////////////////////////////////////////////////////////////
extern void uart_remove_switch_state(void);
extern void uart_insert_switch_state(void);
extern void UsbIndicator(u8 state);
extern int user_switch_init(void);
extern void usb_switch_state(void);
extern void askon_gadget_disconnect(void);
extern int s3c_usb_cable(int connected);
#ifdef _JIG_USB_HW_WORKAROUND_
extern unsigned char maxim_lpm_chg_status(void);
#endif

static int fsa9480_read(u8 reg, u8 *data);

int askonstatus;
EXPORT_SYMBOL(askonstatus);
int inaskonstatus;
EXPORT_SYMBOL(inaskonstatus);
int uUSB_check_finished = 0;
EXPORT_SYMBOL(uUSB_check_finished);

extern unsigned int HWREV;
extern int mtp_mode_on;

#define log_usb_disable 	0
#define log_usb_enable 		1
#define log_usb_active		2

static u8 MicroUSBStatus=0;
static u8 MicroJigUSBOnStatus=0;
static u8 MicroJigUSBOffStatus=0;
u8 MicroJigUARTOffStatus=0;
u8 MicroTAstatus=0;


struct i2c_driver fsa9480_i2c_driver;
static struct i2c_client *fsa9480_i2c_client = NULL;

struct fsa9480_state {
	struct i2c_client *client;
};

static struct i2c_device_id fsa9480_id[] = {
	{"fsa9480", 0},
	{}
};

static u8 fsa9480_device1 = 0, fsa9480_device2 = 0, fsa9480_adc = 0;
int usb_state = 0;

static wait_queue_head_t usb_detect_waitq;
static struct workqueue_struct *fsa9480_workqueue;
static struct work_struct fsa9480_work;

extern int usb_mtp_select(int disable);
extern int usb_switch_select(int enable);
extern int askon_switch_select(int enable);
extern unsigned int charging_mode_get(void);

int samsung_kies_mtp_mode_flag;
int check_reg = 0;

static int microusb_usbpath = -1;

FSA9480_DEV_TY1_TYPE FSA9480_Get_DEV_TYP1(void)
{
	return fsa9480_device1;
}
EXPORT_SYMBOL(FSA9480_Get_DEV_TYP1);


u8 FSA9480_Get_JIG_Status(void)
{
#ifdef _JIG_USB_HW_WORKAROUND_
	u8 device2;

	fsa9480_read(REGISTER_DEVICETYPE2, &device2);
//	printk("[FSA9480] %s: dev2(0x%x)\n", __func__, device2);
	
	if((device2 & (FSA9480_DEV_TY2_JIG_USB_ON | FSA9480_DEV_TY2_JIG_USB_OFF))
	&& (HWREV >= 13)) // for P1-KOR (Above Rev11)
		return 1;
	else
		return 0;
#else
	if(MicroJigUSBOnStatus | MicroJigUSBOffStatus | MicroJigUARTOffStatus)
		return 1;
	else
		return 0;
#endif
}
EXPORT_SYMBOL(FSA9480_Get_JIG_Status);


u8 FSA9480_Get_FPM_Status(void)
{
	if(fsa9480_adc == RID_FM_BOOT_ON_UART)
		return 1;
	else
		return 0;
}
EXPORT_SYMBOL(FSA9480_Get_FPM_Status);


u8 FSA9480_Get_USB_Status(void)
{
#ifdef _JIG_USB_HW_WORKAROUND_
	u8 device1, device2;
		
	if(microusb_usbpath > 0)
		return 0;

	fsa9480_read(REGISTER_DEVICETYPE1, &device1);
	fsa9480_read(REGISTER_DEVICETYPE2, &device2);
	
	if( (device1 == FSA9480_DEV_TY1_USB) ||
		(FSA9480_Get_JIG_Status() && maxim_lpm_chg_status()) )
		return 1;
	else
		return 0;
#else
	if(microusb_usbpath > 0)
		return 0;

	if( MicroUSBStatus | MicroJigUSBOnStatus | MicroJigUSBOffStatus )
		return 1;
	else
		return 0;
#endif
}
EXPORT_SYMBOL(FSA9480_Get_USB_Status);


u8 FSA9480_Get_TA_Status(void)
{
	if(MicroTAstatus)
		return 1;
	else
		return 0;
}
EXPORT_SYMBOL(FSA9480_Get_TA_Status);


u8 FSA9480_Get_JIG_UART_Status(void)
{
	if(MicroJigUARTOffStatus)
		return 1;
	else
		return 0;
}
EXPORT_SYMBOL(FSA9480_Get_JIG_UART_Status);


int get_usb_cable_state(void)
{
	return usb_state;
}


static int fsa9480_read(u8 reg, u8 *data)
{
	struct i2c_client *client = fsa9480_i2c_client;
	int ret;

	if(client == NULL)
		return -ENODEV;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0)
		return -EIO;

	*data = ret & 0xff;
	return 0;
}

static int fsa9480_write(u8 reg, u8 data)
{
	struct i2c_client *client = fsa9480_i2c_client;

	if(client == NULL)
		return -ENODEV;

	return i2c_smbus_write_byte_data(client, reg, data);
}


u8 get_switch_ID(void)
{
	u8 data = 0;

	fsa9480_read( REGISTER_MANUALSW2, &data);

	return data;
}

void ap_usb_power_on(int set_vaue)
{
 	byte reg_value=0;
	byte reg_address=0x0D;

	if(set_vaue){
		Get_MAX8998_PM_ADDR(reg_address, &reg_value, 1); // read 0x0D register
		reg_value = reg_value | (0x1 << 7);
		Set_MAX8998_PM_ADDR(reg_address,&reg_value,1);
		printk("[ap_usb_power_on] AP USB Power ON, askon:%d, mtp:%d\n", askonstatus, mtp_mode_on);
		if(mtp_mode_on == 1) {
			samsung_kies_mtp_mode_flag = 1;
			printk("************ [ap_usb_power_on] samsung_kies_mtp_mode_flag:%d, mtp:%d\n", samsung_kies_mtp_mode_flag, mtp_mode_on);
		}
		else {
			samsung_kies_mtp_mode_flag = 0;
			printk("!!!!!!!!!!! [ap_usb_power_on] AP samsung_kies_mtp_mode_flag%d, mtp:%d\n", samsung_kies_mtp_mode_flag, mtp_mode_on);
		}
	}
	else {
		Get_MAX8998_PM_ADDR(reg_address, &reg_value, 1); // read 0x0D register
		reg_value = reg_value & ~(0x1 << 7);
		Set_MAX8998_PM_ADDR(reg_address,&reg_value,1);
		printk("[ap_usb_power_on] AP USB Power OFF, askon:%d, mtp:%d\n", askonstatus, mtp_mode_on);
	}
}


void FSA9480_ChangePathToAudio(u8 enable)
{
	u8 manualsw1;

	if(enable)
	{
		mdelay(10);
		fsa9480_write( REGISTER_MANUALSW1, 0x48);			

		mdelay(10);
		fsa9480_write( REGISTER_CONTROL, 0x1A);

		fsa9480_read( REGISTER_MANUALSW1, &manualsw1);
		printk("Fsa9480 ManualSW1 = 0x%x\n",manualsw1);
	}
	else
	{
		mdelay(10);
		fsa9480_write( REGISTER_CONTROL, 0x1E);	
	}
}
EXPORT_SYMBOL(FSA9480_ChangePathToAudio);


void FSA9480_Set_ID_Switch(int sel)
{
	u8 data = 0;
	
	printk("[FSA9480] %s: select (%d)\n", __func__, sel);

	fsa9480_read(REGISTER_MANUALSW2, &data);
	data &= ~(0x3);

	switch(sel)
	{
		case 1:  // ID connected to video
			data |= 0x1;
			break;
		case 2:  // ID connected to bypass port
			data |= 0x2;
			break;
		case 0:  // Open all switch
		default:
			break;
	}

	fsa9480_write(REGISTER_MANUALSW2, data);

	fsa9480_read(REGISTER_MANUALSW2, &data);
	printk("[FSA9480] %s: MANUALSW2 (0x%x)\n", __func__, data);
}


void FSA9480_Enable_CP_USB(u8 enable)
{
	byte reg_value=0;
	byte reg_address=0x0D;

	microusb_usbpath = enable;

#ifdef _UART_HW_BUG_
	if(enable == 2)  // Set USB path to UART (For P1 KOR)
	{
		mdelay(10);
		fsa9480_write(REGISTER_MANUALSW1, 0x6C);  // Set path to RXD/TXD

		mdelay(10);
		fsa9480_write(REGISTER_CONTROL, 0x1A);

		fsa9480_read(REGISTER_MANUALSW1, &reg_value);
		printk("[FSA9480] %s: ManualSW1 = 0x%x\n", __func__, reg_value);
	}
	else
#endif
	if(enable == 1)
	{
		printk("[FSA9480_Enable_CP_USB] Enable CP USB\n");
		mdelay(10);
		Get_MAX8998_PM_ADDR(reg_address, &reg_value, 1); // read 0x0D register
		check_reg = reg_value;
		reg_value = ((0x2<<5)|reg_value);
		check_reg = reg_value;
		Set_MAX8998_PM_ADDR(reg_address,&reg_value,1);
		check_reg = reg_value;
			
		mdelay(10);
		fsa9480_write(REGISTER_MANUALSW1, 0x90);	

		mdelay(10);
		fsa9480_write(REGISTER_CONTROL, 0x1A);	
	}
	else
	{
		printk("[FSA9480_Enable_AP_USB] Enable AP USB\n");
		Get_MAX8998_PM_ADDR(reg_address, &reg_value, 1); // read 0x0D register
		reg_value = ((0x2<<6)|reg_value);
		Set_MAX8998_PM_ADDR(reg_address,&reg_value,1);
	
		if(askonstatus||mtp_mode_on)
			ap_usb_power_on(0);
		else
			ap_usb_power_on(1);

		mdelay(10);
		fsa9480_write(REGISTER_CONTROL, 0x1E);
	}
}


static void FSA9480_ProcessDevice(u8 dev1, u8 dev2, u8 attach)
{
	DEBUG_FSA9480("[FSA9480] %s (dev1 : 0x%x, dev2 : 0x%x)\n", __func__, dev1, dev2);

	if(dev1)
	{
		switch(dev1)
		{
			case FSA9480_DEV_TY1_AUD_TY1:
//				DEBUG_FSA9480("Audio Type1 ");
				if(attach & FSA9480_INT1_ATTACH)
					DEBUG_FSA9480("FSA9480_DEV_TY1_AUD_TY1 --- ATTACH\n");
				else
					DEBUG_FSA9480("FSA9480_DEV_TY1_AUD_TY1 --- DETACH\n");
				break;

			case FSA9480_DEV_TY1_AUD_TY2:
				DEBUG_FSA9480("Audio Type2 ");
				break;

			case FSA9480_DEV_TY1_USB:
//				DEBUG_FSA9480("USB attach or detach: %d\n",attach);
				if(attach & FSA9480_INT1_ATTACH)
				{
					DEBUG_FSA9480("FSA9480_DEV_TY1_USB --- ATTACH\n");
					MicroUSBStatus = 1;
					usb_switch_state();

					if(!askonstatus)
						UsbIndicator(1);
					else
						inaskonstatus = 0;			

					uUSB_check_finished = 1;  // finished
				}
				else if(attach & FSA9480_INT1_DETACH)
				{	
					DEBUG_FSA9480("FSA9480_DEV_TY1_USB --- DETACH\n");
					MicroUSBStatus = 0;
					UsbIndicator(0);
					askon_gadget_disconnect();

					uUSB_check_finished = 0;  // finished
				}
				break;

			case FSA9480_DEV_TY1_UART:
				DEBUG_FSA9480("UART\n");
				break;

			case FSA9480_DEV_TY1_CAR_KIT:
				DEBUG_FSA9480("Carkit\n");
				break;

			case FSA9480_DEV_TY1_USB_CHG:
				DEBUG_FSA9480("USB\n");
				break;

			case FSA9480_DEV_TY1_DED_CHG:
				{
					if(attach & FSA9480_INT1_ATTACH)
					{
						DEBUG_FSA9480("Dedicated Charger ATTACH\n");
						uUSB_check_finished = 1;  // finished
						//A9480_ChangePathToAudio(TRUE);
					}					
					else if(attach & FSA9480_INT1_DETACH)
					{				
						DEBUG_FSA9480("Dedicated Charger DETACH\n");
						uUSB_check_finished = 0;  // finished
						//A9480_ChangePathToAudio(FALSE);
					}
				}
				break;

			case FSA9480_DEV_TY1_USB_OTG:
				DEBUG_FSA9480("USB OTG\n");
				break;

			default:
				DEBUG_FSA9480("Unknown device\n");
				break;
		}

	}

	if(dev2)
	{
		switch(dev2)
		{
			case FSA9480_DEV_TY2_JIG_USB_ON:
//				DEBUG_FSA9480("JIG USB ON attach or detach: %d\n",attach);

#ifdef _JIG_USB_HW_WORKAROUND_
				if(HWREV >= 13) {  // To support JIG_USB_ON (Above Rev11)
					if(maxim_lpm_chg_status())
					{
						DEBUG_FSA9480("FSA9480_DEV_TY2_JIG_USB_ON --- ATTACH (VBUS)\n");
						MicroJigUSBOnStatus = 1;
						usb_switch_state();

						if(!askonstatus)
							UsbIndicator(1);
						else
							inaskonstatus = 0;
	
						uUSB_check_finished = 1;  // finished
					}
					else
					{
						DEBUG_FSA9480("FSA9480_DEV_TY2_JIG_USB_ON --- DETACH (VBUS)\n");
						MicroJigUSBOnStatus = 0;
						inaskonstatus = 0;
						UsbIndicator(0);
						askon_gadget_disconnect();
	
						uUSB_check_finished = 0;  // finished
					}
				}
				else
#endif
				{
					if(attach & FSA9480_INT1_ATTACH)
					{
						DEBUG_FSA9480("FSA9480_DEV_TY2_JIG_USB_ON --- ATTACH\n");
						MicroJigUSBOnStatus = 1;
						usb_switch_state();

						if(!askonstatus)
							UsbIndicator(1);
						else
							inaskonstatus = 0;
					}
					else if(attach & FSA9480_INT1_DETACH)
					{
						DEBUG_FSA9480("FSA9480_DEV_TY2_JIG_USB_ON --- DETACH\n");
						MicroJigUSBOnStatus = 0;
						inaskonstatus = 0;
						UsbIndicator(0);
						askon_gadget_disconnect();				
					}
				}
				break;

			case FSA9480_DEV_TY2_JIG_USB_OFF:

#ifdef _JIG_USB_HW_WORKAROUND_
				if(HWREV >= 13) {  // To support JIG_USB_OFF (Above Rev11)
					if(maxim_lpm_chg_status())
					{
						DEBUG_FSA9480("FSA9480_DEV_TY2_JIG_USB_OFF --- ATTACH (VBUS)\n");
						MicroJigUSBOnStatus = 1;
						usb_switch_state();

						if(!askonstatus)
							UsbIndicator(1);
						else
							inaskonstatus = 0;
	
						uUSB_check_finished = 1;  // finished
					}
					else
					{
						DEBUG_FSA9480("FSA9480_DEV_TY2_JIG_USB_OFF --- DETACH (VBUS)\n");
						MicroJigUSBOnStatus = 0;
						inaskonstatus = 0;
						UsbIndicator(0);
						askon_gadget_disconnect();
	
						uUSB_check_finished = 0;  // finished
					}
				}
				else
#endif
				{
					if(attach & FSA9480_INT1_ATTACH)
					{
						DEBUG_FSA9480("FSA9480_DEV_TY2_JIG_USB_OFF --- ATTACH\n");
						MicroJigUSBOffStatus = 1;
						usb_switch_state();

						if(!askonstatus)
							UsbIndicator(1);
						else
							inaskonstatus = 0;	
					}
					else if(attach & FSA9480_INT1_DETACH)
					{
						DEBUG_FSA9480("FSA9480_DEV_TY2_JIG_USB_OFF --- DETACH\n");
						MicroJigUSBOffStatus = 0;
						inaskonstatus = 0;
						UsbIndicator(0);
						askon_gadget_disconnect();					
					}
				}
				break;

			case FSA9480_DEV_TY2_JIG_UART_ON:
				if(attach & FSA9480_INT1_ATTACH)
				{
					DEBUG_FSA9480("FSA9480_DEV_TY2_JIG_UART_ON --- ATTACH\n");
				}
				else
				{
					DEBUG_FSA9480("FSA9480_DEV_TY2_JIG_UART_ON --- DETACH\n");
				}
				//DEBUG_FSA9480("JIG UART ON\n");
				break;

			case FSA9480_DEV_TY2_JIG_UART_OFF:
				if(attach & FSA9480_INT1_ATTACH)
				{
					DEBUG_FSA9480("FSA9480_DEV_TY2_JIG_UART_OFF --- ATTACH\n");
					MicroJigUARTOffStatus = 1;
					uart_insert_switch_state();
				}
				else
				{
					DEBUG_FSA9480("FSA9480_DEV_TY2_JIG_UART_OFF --- DETACH\n");
					MicroJigUARTOffStatus = 0;
					uart_remove_switch_state();
				}
				//DEBUG_FSA9480("JIT UART OFF\n");
				break;

			case FSA9480_DEV_TY2_PDD:
				DEBUG_FSA9480("PPD\n");
				break;

			case FSA9480_DEV_TY2_TTY:
				DEBUG_FSA9480("TTY\n");
				break;

			case FSA9480_DEV_TY2_AV:
				//DEBUG_FSA9480("AudioVideo\n");
				if(attach & FSA9480_INT1_ATTACH)
					DEBUG_FSA9480("FSA9480_DEV_TY2_AV --- ATTACH\n");
				else
					DEBUG_FSA9480("FSA9480_DEV_TY2_AV --- DETACH\n");
				break;

			default:
				DEBUG_FSA9480("Unknown device\n");
				break;
		}
	}

}


static void FSA9480_ReadIntRegister(struct work_struct * work)
{
	u8 interrupt1, interrupt2, device1, device2, temp;

	DEBUG_FSA9480("[FSA9480] %s\n", __func__);

	fsa9480_read(REGISTER_INTERRUPT1, &interrupt1);
	msleep(5);

	fsa9480_read(REGISTER_INTERRUPT2, &interrupt2);
	msleep(5);

	fsa9480_read(REGISTER_DEVICETYPE1, &device1);
	msleep(5);

	fsa9480_read(REGISTER_DEVICETYPE2, &device2);

	if(microusb_usbpath > 0) // if CP USB
	{
		usb_state = 0;

		fsa9480_write(REGISTER_CONTROL, 0x1A);
		mdelay(10);

		fsa9480_write(REGISTER_MANUALSW1, 0x90);
		mdelay(10);

		fsa9480_write(REGISTER_INTERRUPTMASK1, 0xFC);
	}
	else
	{
	usb_state = (device2 << 8) | (device1 << 0);

	if((interrupt1 & FSA9480_INT1_ATTACH)
#ifdef _JIG_USB_HW_WORKAROUND_
	|| (FSA9480_Get_JIG_Status() && maxim_lpm_chg_status())
#endif
	)
	{
		fsa9480_device1 = device1;
		fsa9480_device2 = device2;

		if(fsa9480_device1 != FSA9480_DEV_TY1_DED_CHG) {
				s3c_usb_cable(USB_CABLE_ATTACHED);
		}

		if(fsa9480_device1 & FSA9480_DEV_TY1_CAR_KIT)
		{
			msleep(5);
			fsa9480_write(REGISTER_CARKITSTATUS, 0x02);

			msleep(5);
			fsa9480_read(REGISTER_CARKITINT1, &temp);
		}
	}

	msleep(5);

	fsa9480_write(REGISTER_CONTROL, 0x1E);
	fsa9480_write(REGISTER_INTERRUPTMASK1, 0xFC);

	FSA9480_ProcessDevice(fsa9480_device1, fsa9480_device2, interrupt1);

	if((interrupt1 & FSA9480_INT1_DETACH)
#ifdef _JIG_USB_HW_WORKAROUND_
	|| (FSA9480_Get_JIG_Status() && !maxim_lpm_chg_status())
#endif
	)
	{
		if(fsa9480_device1 != FSA9480_DEV_TY1_DED_CHG) {
				s3c_usb_cable(USB_CABLE_DETACHED);
		}

		fsa9480_device1 = 0;
		fsa9480_device2 = 0;
	}
	}
	
	enable_irq(IRQ_FSA9480_INTB);
}


static irqreturn_t fsa9480_interrupt(int irq, void *ptr)
{
	printk("%s\n", __func__);

	disable_irq_nosync(IRQ_FSA9480_INTB);

	uUSB_check_finished = 0;  // reset

	queue_work(fsa9480_workqueue, &fsa9480_work);

	return IRQ_HANDLED;
}


#ifdef _JIG_USB_HW_WORKAROUND_
void fsa9480_fake_interrupt(void)
{
	printk("%s\n", __func__);
	disable_irq_nosync(IRQ_FSA9480_INTB);
	
	uUSB_check_finished = 0;  // reset

	queue_work(fsa9480_workqueue, &fsa9480_work);
}
EXPORT_SYMBOL(fsa9480_fake_interrupt);
#endif

#ifdef CONFIG_USB_S3C_OTG_HOST
#define PMIC_IRQ		IRQ_EINT7
void fsa9480_enable_interrupt(int enable)
{
	DEBUG_FSA9480("[FSA9480] enable_interrupt(%d)\n", enable);

	if(enable)
   	{
		enable_irq(IRQ_FSA9480_INTB);
		enable_irq(PMIC_IRQ);
	}
	else
	{
		disable_irq_nosync(PMIC_IRQ);
		disable_irq_nosync(IRQ_FSA9480_INTB);
	}
}
EXPORT_SYMBOL(fsa9480_enable_interrupt);
#endif

static void fsa9480_interrupt_init(void)
{
	s3c_gpio_cfgpin(GPIO_JACK_nINT, S3C_GPIO_SFN(GPIO_JACK_nINT_AF));
	s3c_gpio_setpull(GPIO_JACK_nINT, S3C_GPIO_PULL_NONE);
	set_irq_type(IRQ_FSA9480_INTB, IRQ_TYPE_EDGE_FALLING);

	if (request_irq(IRQ_FSA9480_INTB, fsa9480_interrupt, IRQF_DISABLED, "FSA9480 Detected", NULL))
	{
		DEBUG_FSA9480("[FSA9480]fail to register IRQ[%d] for FSA9480 USB Switch \n", IRQ_FSA9480_INTB);
	}
}


static void fsa9480_chip_init(void)
{
	u8 data = 0;

//	iks.kim : fsa9480 reset blocked temporarily (charger detection problem)
//	fsa9480_write(HIDDEN_REGISTER_MANUAL_OVERRDES1, 0x01); //RESET

	mdelay(10);

	fsa9480_read(REGISTER_DEVICEID, &data);

	mdelay(10);

	fsa9480_write(REGISTER_CONTROL, 0x1E);

	mdelay(10);

	// Unmask attach/detach interrupt
	fsa9480_write(REGISTER_INTERRUPTMASK1, 0xFC);
	fsa9480_write(REGISTER_INTERRUPTMASK2, 0xFF);

	mdelay(10);

	// Mask CARKIT all interrupt
	fsa9480_write(REGISTER_CARKITMASK1, 0xFF);
	fsa9480_write(REGISTER_CARKITMASK2, 0xFF);

	mdelay(10);

	fsa9480_read(REGISTER_DEVICETYPE1, &fsa9480_device1);

	mdelay(10);

	fsa9480_read(REGISTER_DEVICETYPE2, &fsa9480_device2);
}


#if 1 // P1-KOR
void FSA9480_InitDevice(void)
{
#define I2C_READ_RETRY_MAX 2
	int ret = 0;
	int read_retry;
	u8 device1;
	u8 device2;
	u8 interrupt1;
	u8 interrupt2;
	u8 attach = FSA9480_INT1_ATTACH;

	for(read_retry = 0; read_retry < I2C_READ_RETRY_MAX; read_retry++) {
		device1 = 0;
		if(fsa9480_read(REGISTER_DEVICETYPE1, &device1) == 0) {
			break;
		}
	}

	for(read_retry = 0; read_retry < I2C_READ_RETRY_MAX; read_retry++) {
		device2 = 0;
		if(fsa9480_read(REGISTER_DEVICETYPE2, &device2) == 0) {
			break;
		}
	}

	if(microusb_usbpath > 0) // if CP USB
	{
		s3c_usb_cable(USB_CABLE_DETACHED);

		usb_state = 0;

		fsa9480_write(REGISTER_CONTROL, 0x1A);
		mdelay(10);

		fsa9480_write(REGISTER_MANUALSW1, 0x90);
		mdelay(10);
	}
	else
	{
	if(device1 && (device1 != FSA9480_DEV_TY1_DED_CHG)) {
		s3c_usb_cable(USB_CABLE_ATTACHED);
	}

	if(device1 == 0 && device2 == 0) {
		device1 = FSA9480_DEV_TY1_USB;
		attach = FSA9480_INT1_DETACH;
		s3c_usb_cable(USB_CABLE_DETACHED);
	}

	usb_state = (device2 << 8) | (device1 << 0);

	fsa9480_device1 = device1;
	fsa9480_device2 = device2;

		fsa9480_write(REGISTER_CONTROL, 0x1E);

	FSA9480_ProcessDevice(fsa9480_device1, fsa9480_device2, attach);
	}

	fsa9480_write(REGISTER_INTERRUPTMASK1, 0xFC);

	// clear interrupt
	fsa9480_read(REGISTER_INTERRUPT1, &interrupt1);
	msleep(5);

	fsa9480_read(REGISTER_INTERRUPT2, &interrupt2);
	msleep(5);

	fsa9480_interrupt_init();
}
EXPORT_SYMBOL(FSA9480_InitDevice);
#endif


static int fsa9480_codec_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct fsa9480_state *state;
	struct device *dev = &client->dev;
	u8 pData;

	DEBUG_FSA9480("[FSA9480] %s\n", __func__);

	s3c_gpio_cfgpin(GPIO_USB_SW_SCL, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_USB_SW_SCL, S3C_GPIO_PULL_NONE);

	s3c_gpio_cfgpin(GPIO_USB_SW_SDA, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_USB_SW_SDA, S3C_GPIO_PULL_NONE);

	s3c_gpio_cfgpin(GPIO_UART_SEL, S3C_GPIO_OUTPUT );
	s3c_gpio_setpull(GPIO_UART_SEL, S3C_GPIO_PULL_NONE);

	user_switch_init();

	init_waitqueue_head(&usb_detect_waitq); 
	INIT_WORK(&fsa9480_work, FSA9480_ReadIntRegister);
	fsa9480_workqueue = create_singlethread_workqueue("fsa9480_workqueue");

	state = kzalloc(sizeof(struct fsa9480_state), GFP_KERNEL);
	if(!state) {
		dev_err(dev, "%s: failed to create fsa9480_state\n", __func__);
		return -ENOMEM;
	}

	state->client = client;
	fsa9480_i2c_client = client;

	i2c_set_clientdata(client, state);
	if(!fsa9480_i2c_client)
	{
		dev_err(dev, "%s: failed to create fsa9480_i2c_client\n", __func__);
		return -ENODEV;
	}

	/*clear interrupt mask register*/
	fsa9480_read(REGISTER_CONTROL, &pData);
	fsa9480_write(REGISTER_CONTROL, pData & ~INT_MASK);

#if 0 // P1-KOR
	fsa9480_interrupt_init();
#endif

	fsa9480_chip_init();

	return 0;
}


static int __devexit fsa9480_remove(struct i2c_client *client)
{
	struct fsa9480_state *state = i2c_get_clientdata(client);
	kfree(state);
	return 0;
}


struct i2c_driver fsa9480_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "fsa9480",
	},
	.id_table	= fsa9480_id,
	.probe	= fsa9480_codec_probe,
	.remove	= __devexit_p(fsa9480_remove),
	.command = NULL,
};


static int __init fsa9480_driver_init(void)
{
	int ret;
	DEBUG_FSA9480("%s\n", __func__);

	if((ret = i2c_add_driver(&fsa9480_i2c_driver)))
		pr_err("%s: Can't add fsa9480 i2c driver\n", __func__);

	return ret;
}


static void __exit fsa9480_driver_exit(void)
{
	DEBUG_FSA9480("%s\n", __func__);
	i2c_del_driver(&fsa9480_i2c_driver);
}

subsys_initcall(fsa9480_driver_init);
module_exit(fsa9480_driver_exit);

MODULE_AUTHOR("Samsung eletronics");
MODULE_DESCRIPTION("fsa9480 driver");
MODULE_LICENSE("GPL");

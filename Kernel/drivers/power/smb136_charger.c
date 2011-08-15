 
/* Slave address */
#define SMB136_SLAVE_ADDR	0x9A

/* SMB136 Registers. */
#define SMB_ChargeCurrent		0x00
#define SMB_InputCurrentLimit	0x01
#define SMB_FloatVoltage		0x02
#define SMB_ControlA			0x03
#define SMB_ControlB			0x04
#define SMB_PinControl		0x05
#define SMB_OTGControl		0x06
#define SMB_Fault				0x07
#define SMB_Temperature		0x08
#define SMB_SafetyTimer		0x09
#define SMB_VSYS				0x0A
#define SMB_I2CAddr			0x0B

#define SMB_IRQreset			0x30
#define SMB_CommandA		0x31
#define SMB_StatusA			0x32
#define SMB_StatusB			0x33
#define SMB_StatusC			0x34
#define SMB_StatusD			0x35
#define SMB_StatusE			0x36
#define SMB_StatusF			0x37
#define SMB_StatusG			0x38
#define SMB_StatusH			0x39
#define SMB_DeviceID			0x3B
#define SMB_CommandB		0x3C

/* SMB_StatusC register bit. */
#define SMB_USB			1
#define SMB_CHARGER		0
#define Compelete			1
#define Busy				0
#define InputCurrent275	0xE
#define InputCurrent500	0xF
#define InputCurrent700	0x0
#define InputCurrent800	0x1
#define InputCurrent900	0x2
#define InputCurrent1000	0x3
#define InputCurrent1100	0x4
#define InputCurrent1200	0x5
#define InputCurrent1300	0x6
#define InputCurrent1400	0x7


#define DEVICE_TA			1
#define DEVICE_USB		2

static struct i2c_driver smb136_i2c_driver;
static struct i2c_client *smb136_i2c_client = NULL;

int charger_i2c_init = 0;


struct smb136_state {
	struct i2c_client *client;
};

static struct i2c_device_id smb136_id[] = {
	{"smb136", 0},
	{}
};


static int smb136_i2c_read(struct i2c_client *client, u8 reg, u8 *data)
{
	int ret;

	if(!client)
		return -ENODEV;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0)
		return -EIO;

	*data = ret & 0xff;
	return 0;

}


static int smb136_i2c_write(struct i2c_client *client, u8 reg, u8 data)
{
	if(!client)
		return -ENODEV;

	return i2c_smbus_write_byte_data(client, reg, data);
}


u32 smb136_is_charging_active(void)
{
	u8 data=0;
	smb136_i2c_read(smb136_i2c_client, 0x39, &data);		
	printk("SMB136 addr : 0x39 data : 0x%02x\n",data);

	if(data & 0x01)
		return 1;
	else
		return 0;
}


u32 smb136_is_fullcharging(void)
{
	u8 data=0;
	smb136_i2c_read(smb136_i2c_client, 0x36, &data);		
	printk("SMB136 addr : 0x36 data : 0x%02x\n",data);
	
	if ((data & 0x08) == 0x08)	// hanapark_P1 : return if charge error (2010-10-02)
		return 0;

	if(data & 0x40)	// Charge current < Termination Current
		return 1;
	else
		return 0;
}


u32 smb136_is_already_fullcharged(void)
{
	u8 data=0;
	smb136_i2c_read(smb136_i2c_client, 0x36, &data);		
	printk("SMB136 addr : 0x36 data : 0x%02x\n",data);

	if ((data & 0x08) == 0x08)	// hanapark_P1 : return if charge error (2010-10-02)
		return 0;

	if((data & 0xc0) == 0xc0)	// At least one charge cycle terminated, Charge current < Termination Current
		return 1;
	else 
		return 0;
}

void smb136_test_read(void)
{
	u8 data;
	u32 addr;

	if(!charger_i2c_init) {
		printk("%s : smb136 charger IC i2c is not initialized!!\n", __func__);
		return ;
	}

	for(addr=0;addr<0x0c;addr++)
	{
		smb136_i2c_read(smb136_i2c_client, addr, &data);		
		printk("SMB136 addr : 0x%02x data : 0x%02x\n", addr,data);
	}

	for(addr=0x31;addr<0x3D;addr++)
	{
		smb136_i2c_read(smb136_i2c_client, addr, &data);		
		printk("SMB136 addr : 0x%02x data : 0x%02x\n", addr,data);
	}

}

void smb136_charging(int source)
{
	u8 data;

	if(!charger_i2c_init) {
		printk("%s : smb136 charger IC i2c is not initialized!!\n", __func__);
		return ;
	}

	if(source ==DEVICE_TA)
	{
		//1. HC mode
		data= 0x8c;	

		smb136_i2c_write(smb136_i2c_client,SMB_CommandA, data);
		udelay(10);


		// 2. Change USB5/1/HC Control from Pin to I2C
		smb136_i2c_write(smb136_i2c_client, SMB_PinControl,0x8);
		udelay(10);

		smb136_i2c_write(smb136_i2c_client,SMB_CommandA, 0x8c);
		udelay(10);

		//3. Set charge current to 1500mA
#ifdef CONFIG_TARGET_LOCALE_VZW
		data = 0xf2; //terminal Current 100mA 
#else
		data = 0xf4;
#endif
		
		smb136_i2c_write(smb136_i2c_client,SMB_ChargeCurrent, data);
		udelay(10);
	}
	else if(source ==DEVICE_USB)
	{
		// 1. USBIN 500mA mode 
		data= 0x88;	

		smb136_i2c_write(smb136_i2c_client,SMB_CommandA, data);
		udelay(10);

		// 2. Change USB5/1/HC Control from Pin to I2C
		smb136_i2c_write(smb136_i2c_client, SMB_PinControl,0x8);
		udelay(10);

		smb136_i2c_write(smb136_i2c_client,SMB_CommandA, 0x88);
		udelay(10);

		// 3. Set charge current to 500mA
#ifdef CONFIG_TARGET_LOCALE_VZW
		data = 0x12;
#else
		data = 0x14;
#endif
		smb136_i2c_write(smb136_i2c_client,SMB_ChargeCurrent, data);
		udelay(10);
	}

	// 3. Disable Automatic Input Current Limit
	data=	0xe6;
	smb136_i2c_write(smb136_i2c_client,SMB_InputCurrentLimit, data);
	udelay(10);

	//4. Automatic Recharge Disabed 
	data = 0x8c;
	smb136_i2c_write(smb136_i2c_client,SMB_ControlA, data);
	udelay(10);

	//5. Safty timer Disabled
	data = 0x28;
	smb136_i2c_write(smb136_i2c_client,SMB_ControlB, data);
	udelay(10);

	//6. Disable USB D+/D- Detection
	data=0x28;
	smb136_i2c_write(smb136_i2c_client,SMB_OTGControl, data);
	udelay(10);

	//7. Set Output Polarity for STAT
	data=0xCA;
	smb136_i2c_write(smb136_i2c_client, SMB_FloatVoltage, data);
	udelay(10);

	//8. Re-load Enable
	data=0x4b;
	smb136_i2c_write(smb136_i2c_client, SMB_SafetyTimer, data);
	udelay(10);

}


void smb136_set_otg_mode(int enable)
{
	struct i2c_client *client = smb136_i2c_client;
	u8 data;

	printk("[SMB136] %s (enable : %d)\n", __func__, enable);

	if(!charger_i2c_init) {
		printk("%s : smb136 charger IC i2c is not initialized!!\n", __func__);
		return ;
	}

	if(enable)  // Enable OTG Mode
	{
		// 1. Set OTG Mode (Clear Bit5 of Pin Control)
		smb136_i2c_read(client, SMB_PinControl, &data);
		data &= ~(0x1 << 5);
		smb136_i2c_write(client, SMB_PinControl, data);
		udelay(10);

		// 2. Enable OTG Mode (Set Bit1 of Command Register A)
		smb136_i2c_read(client, SMB_CommandA, &data);
		data |= (0x1 << 1);
		smb136_i2c_write(client, SMB_CommandA, data);
		udelay(10);
	}
	else  // Re-init charger IC
	{
		// 1. Allow volatile writes to 00~09h, USB 500mA Mode, USB5/1 Mode
		data = 0x88;
		smb136_i2c_write(client, SMB_CommandA, data);
		udelay(10);

		// 2. Change USB5/1/HC Control from Pin to I2C
		data = 0x08;
		smb136_i2c_write(client, SMB_PinControl, data);
		udelay(10);

		// 3. Allow volatile writes to 00~09h, USB 500mA Mode, USB5/1 Mode
		data = 0x88;
		smb136_i2c_write(client, SMB_CommandA, data);
		udelay(10);

		// 4. Disable Automatic Input Current Limit
		data = 0xe6;
		smb136_i2c_write(client, SMB_InputCurrentLimit, data);
		udelay(10);

		//5. Fast Charge Current set 500mA
#ifdef CONFIG_TARGET_LOCALE_VZW
		data = 0xf2;
#else 
		data = 0xf4;
#endif
		smb136_i2c_write(client, SMB_ChargeCurrent, data);
		udelay(10);

		//6. Automatic Recharge Disabed
		data = 0x8c;
		smb136_i2c_write(client, SMB_ControlA, data);
		udelay(10);

		//7. Safty timer Disabled
		data = 0x28;
		smb136_i2c_write(client, SMB_ControlB, data);
		udelay(10);

		//8. Disable USB D+/D- Detection
		data = 0x28;
		smb136_i2c_write(client, SMB_OTGControl, data);
		udelay(10);

		//9. Set Output Polarity for STAT
		data = 0xca;
		smb136_i2c_write(client, SMB_FloatVoltage, data);
		udelay(10);

		//10. Re-load Enable
		data = 0x4b;
		smb136_i2c_write(client, SMB_SafetyTimer, data);
		udelay(10);
	}

	smb136_test_read();
}
EXPORT_SYMBOL(smb136_set_otg_mode);


static int smb136_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct smb136_state *state;

	state = kzalloc(sizeof(struct smb136_state), GFP_KERNEL);
	if (state == NULL) {		
		printk("failed to allocate memory \n");
		return -ENOMEM;
	}
	
	state->client = client;
	i2c_set_clientdata(client, state);
	
	/* rest of the initialisation goes here. */
	
	printk("Smb136 charger attach success!!!\n");

	smb136_i2c_client = client;

	charger_i2c_init = 1;

	smb136_test_read();
	
	return 0;

}



static int __devexit smb136_remove(struct i2c_client *client)
{
	struct smb136_state *state = i2c_get_clientdata(client);
	kfree(state);
	return 0;
}


static struct i2c_driver smb136_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "smb136",
	},
	.id_table	= smb136_id,
	.probe	= smb136_i2c_probe,
	.remove	= __devexit_p(smb136_remove),
	.command = NULL,
};



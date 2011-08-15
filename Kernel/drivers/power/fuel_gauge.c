/* Fuel_guage.c */
#include <linux/rtc.h>

#define MAX17042
#define TEMPERATURE_FROM_MAX17042

/* Slave address */
#define MAX17042_SLAVE_ADDR	0x6D

/* Register address */
#define STATUS_REG				0x00
#define VALRT_THRESHOLD_REG	0x01
#define TALRT_THRESHOLD_REG	0x02
#define SALRT_THRESHOLD_REG	0x03
#define REMCAP_REP_REG		0x05
#define SOCREP_REG			0x06
#define TEMPERATURE_REG		0x08
#define VCELL_REG				0x09
#define CURRENT_REG			0x0A
#define AVG_CURRENT_REG		0x0B
#define SOCMIX_REG			0x0D
#define SOCAV_REG				0x0E
#define REMCAP_MIX_REG		0x0F
#define FULLCAP_REG			0x10
#define RFAST_REG				0x15
#define DESIGNCAP_REG			0x18
#define AVR_VCELL_REG			0x19
#define CONFIG_REG			0x1D
#define REMCAP_AV_REG		0x1F
#define FullCAP_NOM_REG		0x23
#define MISCCFG_REG			0x2B
#define RCOMP_REG				0x38
#define FSTAT_REG				0x3D
#define dQacc_REG				0x45
#define dPacc_REG				0x46
#define OCV_REG				0xEE
#define VFOCV_REG				0xFB
#define VFSOC_REG				0xFF


// SDI Battery Data
#define SDI_Capacity			0x1F40  // 4000mAh
#define SDI_VFCapacity		0x29AC  // 5334mAh

// ATL Battery Data
#define ATL_Capacity		0x1FBE  // 4063mAh
#define ATL_VFCapacity		0x2A54  // 5418mAh

const int atl_temp_table[][2] = {
	{-51311,		-200},
	{-50431,		-190},
	{-49551,		-180},
	{-47791,		-170},
	{-46911,		-160},
	{-46031,		-150},
	{-45151,		-140},
	{-43391,		-130},
	{-42511,		-120},
	{-41631,		-110},
	{-40751,		-100},
	{-39871,		-90},
	{-38111,		-80},
	{-35471,		-70},
	{-34591,		-60},
	{-33711,		-50},
	{-31951,		-40},
	{-31071,		-30},
	{-29311,		-20},
	{-28451,		-10},
	{-27551,		0},
	{-26651,		10},
	{463,		20},
	{1838,		30},
	{3250,		40},
	{4702,		50},
	{6152,		60},
	{7468,		70},
	{8822,		80},
	{10140,		90},
	{11546,		100},
	{12826,		110},
	{14058,		120},
	{15370,		130},
	{16405,		140},
	{17717,		150},
	{18780,		160},
	{20000,		170},
	{21031,		180},
	{22136,		190},
	{23156,		200},
	{24198,		210},
	{25027,		220},
	{26124,		230},
	{26920,		240},
	{27608,		250},
	{28421,		260},
	{29218,		270},
	{30074,		280},
	{30838,		290},
	{31343,		300},
	{32074,		310},
	{32604,		320},
	{33370,		330},
	{33889,		340},
	{34713,		350},
	{35276,		360},
	{35842,		370},
	{36354,		380},
	{36873,		390},
	{37417,		400},
	{37920,		410},
	{38405,		420},
	{38826,		430},
	{39276,		440},
	{39729,		450},
	{40172,		460},
	{40510,		470},
	{40916,		480},
	{41312,		490},
	{41670,		500},
	{42042,		510},
	{42354,		520},
	{42655,		530},
	{42931,		540},
	{43270,		550},
	{43562,		560},
	{43854,		570},
	{44074,		580},
	{44405,		590},
	{44577,		600},
	{44811,		610},
	{45183,		620},
	{45370,		630},
	{45542,		640},
	{45635,		650},
	{45947,		660},
	{46136,		670},
	{46308,		680},
	{46514,		690},
	{46655,		700},
};

// For low battery compensation.
#ifdef CONFIG_TARGET_LOCALE_KOR
// SDI type Offset
#define SDI_Range4_1_Offset		3310
#define SDI_Range4_3_Offset		3400
#define SDI_Range3_1_Offset		3441
#define SDI_Range3_3_Offset		3444
#define SDI_Range2_1_Offset		3450
#define SDI_Range2_3_Offset		3534
#define SDI_Range1_1_Offset		3451
#define SDI_Range1_3_Offset		3531

#define SDI_Range4_1_Slope		0
#define SDI_Range4_3_Slope		0
#define SDI_Range3_1_Slope		97
#define SDI_Range3_3_Slope		27
#define SDI_Range2_1_Slope		96
#define SDI_Range2_3_Slope		134
#define SDI_Range1_1_Slope		0
#define SDI_Range1_3_Slope		0

// ATL type threshold
#define ATL_Range5_1_Offset		3252
#define ATL_Range5_3_Offset		3268
#define ATL_Range4_1_Offset		3292
#define ATL_Range4_3_Offset		3285
#define ATL_Range3_1_Offset		3295
#define ATL_Range3_3_Offset		3318
#define ATL_Range2_1_Offset		3325
#define ATL_Range2_3_Offset		3346
#define ATL_Range1_1_Offset		3320
#define ATL_Range1_3_Offset		3337

#define ATL_Range5_1_Slope		0
#define ATL_Range5_3_Slope		0
#define ATL_Range4_1_Slope		30  // 0.03
#define ATL_Range4_3_Slope		667  // 0.00667
#define ATL_Range3_1_Slope		20
#define ATL_Range3_3_Slope		40
#define ATL_Range2_1_Slope		60
#define ATL_Range2_3_Slope		76
#define ATL_Range1_1_Slope		0
#define ATL_Range1_3_Slope		0

#else
// SDI type Offset
#define SDI_Range4_1_Offset		3320
#define SDI_Range4_3_Offset		3410
#define SDI_Range3_1_Offset		3451
#define SDI_Range3_3_Offset		3454
#define SDI_Range2_1_Offset		3461
#define SDI_Range2_3_Offset		3544
#define SDI_Range1_1_Offset		3456
#define SDI_Range1_3_Offset		3536

#define SDI_Range4_1_Slope		0
#define SDI_Range4_3_Slope		0
#define SDI_Range3_1_Slope		97
#define SDI_Range3_3_Slope		27
#define SDI_Range2_1_Slope		96
#define SDI_Range2_3_Slope		134
#define SDI_Range1_1_Slope		0
#define SDI_Range1_3_Slope		0

// ATL type threshold
#define ATL_Range5_1_Offset		3277
#define ATL_Range5_3_Offset		3293
#define ATL_Range4_1_Offset		3312
#define ATL_Range4_3_Offset		3305
#define ATL_Range3_1_Offset		3310
#define ATL_Range3_3_Offset		3333
#define ATL_Range2_1_Offset		3335
#define ATL_Range2_3_Offset		3356
#define ATL_Range1_1_Offset		3325
#define ATL_Range1_3_Offset		3342

#define ATL_Range5_1_Slope		0
#define ATL_Range5_3_Slope		0
#define ATL_Range4_1_Slope		30  // 0.03
#define ATL_Range4_3_Slope		667  // 0.00667
#define ATL_Range3_1_Slope		20
#define ATL_Range3_3_Slope		40
#define ATL_Range2_1_Slope		60
#define ATL_Range2_3_Slope		76
#define ATL_Range1_1_Slope		0
#define ATL_Range1_3_Slope		0

#endif


typedef enum {
	POSITIVE = 0,
	NEGATIVE = 1
} sign_type_t;

typedef enum {
	UNKNOWN_TYPE = 0,
	SDI_BATTERY_TYPE,
	ATL_BATTERY_TYPE
} battery_type_t;


static int pr_cnt = 0;
static u32 battery_type = UNKNOWN_TYPE;
static u32 PrevFullCap = 0;
static u32 PrevVfFullCap = 0;
static u32 FirstFullChargedCAP = 0;
static u16 Capacity = 0;
static u16 VFCapacity = 0;
spinlock_t fg_lock;

int fuel_guage_init = 0;
EXPORT_SYMBOL(fuel_guage_init);

extern unsigned char maxim_lpm_chg_status(void);
extern u8 FSA9480_Get_JIG_Status(void);

void fg_periodic_read(void);
void fg_read_model_data(void);


static struct i2c_driver fg_i2c_driver;
static struct i2c_client *fg_i2c_client = NULL;


struct fg_state{
	struct i2c_client	*client;	
};
struct fg_state *fg_state;


static int fg_i2c_read(struct i2c_client *client, u8 reg, u8 *data, u8 length)
{
	u16 value;

#if 0  // not used.
	value = swab16(i2c_smbus_read_word_data(client, (u8)reg));

	if (value < 0)
		pr_err("%s: Failed to fg_i2c_read\n", __func__);

	*data = (value &0xff00) >>8;
	*(data+1) = value & 0x00ff;
#endif
		value = i2c_smbus_read_i2c_block_data(client, (u8)reg, length, data);
		if (value < 0)
			pr_err("%s: Failed to fg_i2c_read\n", __func__);
	
	return 0;
}

static int fg_i2c_write(struct i2c_client *client, u8 reg, u8 *data, u8 length)
{
	u16 value;	
		
	value = i2c_smbus_write_i2c_block_data(client, (u8)reg, length, data);
	if (value < 0)
		pr_err("%s: Failed to fg_i2c_read\n", __func__);
	
	return 0;
}


// Simple function of read/write
int fg_read_register(u8 addr)
{
	struct i2c_client *client = fg_i2c_client;
	u8 data[2];

	if (fg_i2c_read(client, addr, data, (u8)2) < 0) {
		pr_err("%s: Failed to read addr(0x%x)\n", __func__, addr);
		return -1;
	}

	return ( (data[1] << 8) | data[0] );
}


int fg_write_register(u8 addr, u16 w_data)
{
	struct i2c_client *client = fg_i2c_client;
	u8 data[2];

	data[0] = w_data & 0xFF;
	data[1] = (w_data >> 8);

	if (fg_i2c_write(client, addr, data, (u8)2) < 0) {
		pr_err("%s: Failed to write addr(0x%x)\n", __func__, addr);
		return -1;
	}

	return 0;
}


int fg_read_16register(u8 addr, u16* r_data)
{
	struct i2c_client *client = fg_i2c_client;
	u8 data[32];
	int i = 0;

	if (fg_i2c_read(client, addr, data, (u8)32) < 0) {
		pr_err("%s: Failed to read addr(0x%x)\n", __func__, addr);
		return -1;
	}

	for(i=0; i<16; i++) {
		r_data[i] = (data[2*i + 1] << 8) | data[2*i];
//		printk("%s - addr(0x%02x), r_data(0x%04x)\n", __func__, (addr + i), (data[2*i + 1] << 8) | data[2*i]);
	}
	
	return 0;
}


int fg_write_16register(u8 addr, u16* w_data)
{
	struct i2c_client *client = fg_i2c_client;
	u8 data[32];
	u32 i;
	
	for(i=0; i<16; i++) {
		data[2*i] = w_data[i] & 0xFF;
		data[2*i + 1] = (w_data[i] >> 8);
//		printk("%s - addr(0x%02x), w_data(0x%04x)\n", __func__, (addr + i), (data[2*i + 1] << 8) | data[2*i]);
	}
	
	if (fg_i2c_write(client, addr, data, (u8)32) < 0) {
		pr_err("%s: Failed to read addr(0x%x)\n", __func__, addr);
		return -1;
	}

	return 0;
}


void fg_write_and_verify_register(u8 addr, u16 w_data)
{
	u16 r_data;
	u8 retry_cnt = 2;

retry_write:
	fg_write_register(addr, w_data);
	r_data = fg_read_register(addr);

	if(r_data != w_data) {
		pr_err("%s: verification failed (addr : 0x%x, w_data : 0x%x, r_data : 0x%x)\n", __func__, addr, w_data, r_data);

		if(retry_cnt--)
			goto retry_write;
	}
}


int fg_read_vcell(void)
{
	struct i2c_client *client = fg_i2c_client;
	u8 data[2];
	u32 vcell = 0;
	u16 w_data;
	u32 temp;
	u32 temp2;

	if(!fuel_guage_init) {
		printk("%s : fuel guage IC is not initialized!!\n", __func__);
		return -1;
	}

	if (fg_i2c_read(client, VCELL_REG, data, (u8)2) < 0) {
		pr_err("%s: Failed to read VCELL\n", __func__);
		return -1;
	}

	w_data = (data[1]<<8) | data[0];

	temp = (w_data & 0xFFF) * 78125;
	vcell = temp / 1000000;

	temp = ((w_data & 0xF000) >> 4) * 78125;
	temp2 = temp / 1000000;
	vcell += (temp2 << 4);

	if(!(pr_cnt % 30))
		printk("%s : VCELL(%d), data(0x%04x)\n", __func__, vcell, (data[1]<<8) | data[0]);

	return vcell;
}


int fg_read_vfocv(void)
{
	struct i2c_client *client = fg_i2c_client;
	u8 data[2];
	u32 vfocv = 0;
	u16 w_data;
	u32 temp;
	u32 temp2;

	if(!fuel_guage_init) {
		printk("%s : fuel guage IC is not initialized!!\n", __func__);
		return -1;
	}

	if (fg_i2c_read(client, VFOCV_REG, data, (u8)2) < 0) {
		pr_err("%s: Failed to read VFOCV\n", __func__);
		return -1;
	}

	w_data = (data[1]<<8) | data[0];

	temp = (w_data & 0xFFF) * 78125;
	vfocv = temp / 1000000;

	temp = ((w_data & 0xF000) >> 4) * 78125;
	temp2 = temp / 1000000;
	vfocv += (temp2 << 4);

	return vfocv;
}


s32 fg_read_temp(void)
{
	struct i2c_client *client = fg_i2c_client;
	u8 data[2];
	s32 temper = 0;
	s32 trim1 = 15385;
	s32 trim2 = 22308;

	if(!fuel_guage_init) {
		printk("%s : fuel guage IC is not initialized!!\n", __func__);
		return -1;
	}

	if (fg_i2c_read(client, TEMPERATURE_REG, data, (u8)2) < 0) {
		pr_err("%s: Failed to read SOCREP\n", __func__);
		return -1;
	}


	if(data[1]&(0x1 << 7)) //Negative
	{
		temper = ((~(data[1]))&0xFF)+1;
		temper *= (-1000);
	}
	else
	{
		temper = data[1] & 0x7f;
		temper *= 1000;
		temper += data[0] * 39 / 10;
		if(temper > 47000)
			temper = temper * trim1/10000 - trim2;
	}
		
	return temper;
}


s32 fg_read_temp2(void)
{
	struct i2c_client *client = fg_i2c_client;
	u8 data[2];
	s32 temper = 0;
	s32 trim1_1 = 122;
	s32 trim1_2 = 8950;
	s32 trim2_1 = 200;
	s32 trim2_2 = 51000;

	if(!fuel_guage_init) {
		printk("%s : fuel guage IC is not initialized!!\n", __func__);
		return -1;
	}

	if (fg_i2c_read(client, TEMPERATURE_REG, data, (u8)2) < 0) {
		pr_err("%s: Failed to read SOCREP\n", __func__);
		return -1;
	}


	if(data[1]&(0x1 << 7)) //Negative
	{
		temper = ((~(data[1]))&0xFF)+1;
		temper *= (-1000);
	}
	else
	{
		temper = data[1] & 0x7f;
		temper *= 1000;
		temper += data[0] * 39 / 10;
		if(temper >= 47000 && temper <60000)
			temper = temper * trim1_1/100 - trim1_2;
		else if(temper >=60000)
			temper = temper * trim2_1/100 - trim2_2;
	}

	return temper;
}


s32 fg_read_temp3(void)
{
	struct i2c_client *client = fg_i2c_client;
	u8 data[2];
	s32 temper = 0;
	s32 trim1_1 = 257;
	s32 trim1_2 = 126785;
	s32 trim2_1 = 88;
	s32 trim2_2 = 24911;
	int array_size = 0;
	int table_temp = 0;
	int i = 0;

	if(!fuel_guage_init) {
		printk("%s : fuel guage IC is not initialized!!\n", __func__);
		return -1;
	}

	if (fg_i2c_read(client, TEMPERATURE_REG, data, (u8)2) < 0) {
		pr_err("%s: Failed to read SOCREP\n", __func__);
		return -1;
	}

	if(data[1]&(0x1 << 7)) //Negative
	{
		temper = ((~(data[1]))&0xFF)+1;
		temper *= (-1000);
		temper = temper * trim2_1/100 - trim2_2;
	}
	else
	{
		temper = data[1] & 0x7f;
		temper *= 1000;
		temper += data[0] * 39 / 10;

//		if(temper >=60000)
//			temper = temper * trim1_1/100 - trim1_2;
//		else
//			temper = temper * trim2_1/100 - trim2_2;
	}

//	return temper;

	array_size = ARRAY_SIZE(atl_temp_table);
	for (i = 0; i < (array_size - 1); i++) {
		if (i == 0) {
			if (temper <= atl_temp_table[0][0]) {
				table_temp = atl_temp_table[0][1];
				break;
			} else if (temper >= atl_temp_table[array_size-1][0]) {
				table_temp = atl_temp_table[array_size-1][1];
				break;
			}
		}

		if (atl_temp_table[i][0] < temper &&
				atl_temp_table[i+1][0] >= temper) {
			table_temp = atl_temp_table[i+1][1];
		}
	}

//	printk("ATL temp : fuel_temp(%d), table_temp(%d)\n", temper, table_temp);

	return (table_temp * 100);
}


s32 fg_read_batt_temp(void)
{
	if(battery_type == SDI_BATTERY_TYPE)
		return fg_read_temp2();
	else if(battery_type == ATL_BATTERY_TYPE)
		return fg_read_temp3();
	else
		return (s32)30000;
}


int fg_read_soc(void)
{
	struct i2c_client *client = fg_i2c_client;
	u8 data[2];
	u32 soc = 0;
	u32 temp = 0;
	u8 data2[2];
	u32 remcap = 0;
	u32 fullcap = 0;

	if(!fuel_guage_init) {
		printk("%s : fuel guage IC is not initialized!!\n", __func__);
		return -1;
	}

	if (fg_i2c_read(client, SOCREP_REG, data, (u8)2) < 0) {
		pr_err("%s: Failed to read SOCREP\n", __func__);
		return -1;
	}
#if 0  // Not Used
	if (fg_i2c_read(client, SOCMIX_REG, data, (u8)2) < 0) {
		pr_err("%s: Failed to read SOCMIX\n", __func__);
		return -1;
	}
	if (fg_i2c_read(client, SOCAV_REG, data, (u8)2) < 0) {
		pr_err("%s: Failed to read SOCAV\n", __func__);
		return -1;
	}
#endif
	if (fg_i2c_read(client, REMCAP_REP_REG, data2, (u8)2) < 0) {
		pr_err("%s: Failed to read REMCAP_REP_REG\n", __func__);
		return -1;
	}

	temp = data[0] * 39 / 1000;

	soc = data[1];
//	if(temp >= 5)  // over 0.5 %
//		soc += 1;

	remcap = ((data2[1]<<8) | data2[0]) /2;
	fullcap = fg_read_register(FULLCAP_REG);

	if(!(pr_cnt % 30)) {
		printk("%s : SOC(%d), data(0x%04x)\n", __func__, soc, (data[1]<<8) | data[0]);
		printk("%s : FullCAP(%d), data(0x%04x)\n", __func__, (fullcap/2), (u16)fullcap);
		printk("%s : RemCAP(%d), data(0x%04x)\n", __func__, remcap, (data2[1]<<8) | data2[0]);
	}

	return soc;
}


int fg_read_vfsoc(void)
{
	struct i2c_client *client = fg_i2c_client;
	u8 data[2];
	u32 vfsoc = 0;
	u32 temp = 0;

	if(!fuel_guage_init) {
		printk("%s : fuel guage IC is not initialized!!\n", __func__);
		return -1;
	}

	if (fg_i2c_read(client, VFSOC_REG, data, (u8)2) < 0) {
		pr_err("%s: Failed to read VFSOC\n", __func__);
		return -1;
	}

	temp = data[0] * 39 / 1000;

	vfsoc = data[1];
//	if(temp >= 5)  // over 0.5 %
//		vfsoc += 1;

//	if(!(pr_cnt % 30))
//		printk("%s : VfSOC(%d), data(0x%04x)\n", __func__, vfsoc, (data[1]<<8) | data[0]);

	return vfsoc;
}


int fg_read_current(void)
{
	struct i2c_client *client = fg_i2c_client;
	u8 data1[2], data2[2];
	u32 temp, sign;
	s32 i_current = 0;
	s32 avg_current = 0;

	if(!fuel_guage_init) {
		printk("%s : fuel guage IC is not initialized!!\n", __func__);
		return -1;
	}

	if (fg_i2c_read(client, CURRENT_REG, data1, (u8)2) < 0) {
		pr_err("%s: Failed to read CURRENT\n", __func__);
		return -1;
	}

	if (fg_i2c_read(client, AVG_CURRENT_REG, data2, (u8)2) < 0) {
		pr_err("%s: Failed to read AVERAGE CURRENT\n", __func__);
		return -1;
	}

	temp = ((data1[1]<<8) | data1[0]) & 0xFFFF;
	if(temp & (0x1 << 15))
		{
		sign = NEGATIVE;
		temp = (~(temp) & 0xFFFF) + 1;
		}
	else
		sign = POSITIVE;
//	printk("%s : temp(0x%08x), data1(0x%04x)\n", __func__, temp, (data1[1]<<8) | data1[0]);

	temp = temp * 15625;
	i_current = temp / 100000;
	if(sign)
		i_current *= -1;

	temp = ((data2[1]<<8) | data2[0]) & 0xFFFF;
	if(temp & (0x1 << 15))
		{
		sign = NEGATIVE;
		temp = (~(temp) & 0xFFFF) + 1;
		}
	else
		sign = POSITIVE;
//	printk("%s : temp(0x%08x), data2(0x%04x)\n", __func__, temp, (data2[1]<<8) | data2[0]);

	temp = temp * 15625;
	avg_current = temp / 100000;
	if(sign)
		avg_current *= -1;

	if(!(pr_cnt++ % 30))
	{
		printk("%s : CURRENT(%dmA), AVG_CURRENT(%dmA)\n", __func__, i_current, avg_current);
		pr_cnt = 1;

		fg_periodic_read();  // Read max17042's all registers every 1 minute.
	}

	return i_current;
}


int fg_read_avg_current(void)
{
	struct i2c_client *client = fg_i2c_client;
	u8  data2[2];
	u32 temp, sign;
	s32 avg_current = 0;

	if(!fuel_guage_init) {
		printk("%s : fuel guage IC is not initialized!!\n", __func__);
		return -1;
	}
	
	if (fg_i2c_read(client, AVG_CURRENT_REG, data2, (u8)2) < 0) {
		pr_err("%s: Failed to read AVERAGE CURRENT\n", __func__);
		return -1;
	}


	temp = ((data2[1]<<8) | data2[0]) & 0xFFFF;
	if(temp & (0x1 << 15))
	{
		sign = NEGATIVE;
		temp = (~(temp) & 0xFFFF) + 1;
	}
	else
		sign = POSITIVE;

	temp = temp * 15625;
	avg_current = temp / 100000;
	
	if(sign)
		avg_current *= -1;

	return avg_current;
}


int fg_reset_soc(void)
{
	struct i2c_client *client = fg_i2c_client;
	u8 data[2];
	s32 ret = 0;

	printk("%s : Before quick-start - VfOCV(%d), VfSOC(%d), RepSOC(%d)\n",
					__func__, fg_read_vfocv(), fg_read_vfsoc(), fg_read_soc());

	if(maxim_lpm_chg_status()) {
		printk("%s : Return by DCIN input (TA or USB)\n", __func__);
		return 0;
	}

	if(!FSA9480_Get_JIG_Status()) {
		printk("%s : Return by No JIG_ON signal\n", __func__);
		return 0;
	}

	if(!fuel_guage_init) {
		printk("%s : fuel guage IC is not initialized!!\n", __func__);
		return -1;
	}
	
	// cycle 0
	fg_write_register(0x17, (u16)(0x0));
	
	if (fg_i2c_read(client, MISCCFG_REG, data, (u8)2) < 0) {
		pr_err("%s: Failed to read MiscCFG\n", __func__);
		return -1;
	}

	data[1] |= (0x1 << 2);  // Set bit10 makes quick start

	if (fg_i2c_write(client, MISCCFG_REG, data, (u8)2) < 0) {
		pr_err("%s: Failed to write MiscCFG\n", __func__);
		return -1;
	}

	msleep(250);
#if defined(CONFIG_TARGET_LOCALE_NTT)
	msleep(250);
#endif /* CONFIG_TARGET_LOCALE_NTT */

	fg_write_register(0x10, Capacity);  // FullCAP

	msleep(500);

	printk("%s : After quick-start - VfOCV(%d), VfSOC(%d), RepSOC(%d)\n",
					__func__, fg_read_vfocv(), fg_read_vfsoc(), fg_read_soc());

	// cycle 160
	fg_write_register(0x17, (u16)(0x00a0));

	return ret;
}


//#define __TEST_FULLCAP_INVALID_CASE__
int fg_reset_capacity(int sel)
{
//	struct i2c_client *client = fg_i2c_client;
	s32 ret = 0;
#ifdef __TEST_FULLCAP_INVALID_CASE__
	u16 cap = 800;
	u16 data[16];
	int i = 0;

	memset(data, 0x0, sizeof(data));  // all zero
#endif

	if(!fuel_guage_init) {
		printk("%s : fuel guage IC is not initialized!!\n", __func__);
		return -1;
	}

#ifndef __TEST_FULLCAP_INVALID_CASE__
	ret = fg_write_register(DESIGNCAP_REG, VFCapacity-1);  // DesignCAP
#else
//	fg_write_register(REMCAP_MIX_REG, cap);
//	fg_write_register(REMCAP_AV_REG, cap);
//	fg_write_register(REMCAP_REP_REG, cap);
//	fg_write_register(FULLCAP_REG, cap);

	switch(sel) {
	case 1:  // corrupt by unlock model
		fg_write_and_verify_register(0x62, 0x0059);
		fg_write_and_verify_register(0x63, 0x00C4);
		break;

	case 2:  // corrupt VFOCV
		fg_write_and_verify_register(VFOCV_REG, 0x0000);  // bad VFOCV value
		break;

	case 3:
		fg_write_and_verify_register(0x62, 0x0059);  // first unlock model
		fg_write_and_verify_register(0x63, 0x00C4);
		fg_write_16register(0x80, data);
		fg_write_and_verify_register(0x62, 0x0000);  // lock model
		fg_write_and_verify_register(0x63, 0x0000);
		break;

	case 4:
		fg_write_and_verify_register(0x62, 0x0059);  // first unlock model
		fg_write_and_verify_register(0x63, 0x00C4);
		fg_write_16register(0x80, data);
		fg_write_and_verify_register(0x62, 0x0000);  // lock model
		fg_write_and_verify_register(0x63, 0x0000);
		fg_write_and_verify_register(VFOCV_REG, 0x0000);  // bad VFOCV value
		break;
	}
#endif

	return ret;
}


int fg_check_chip_state(void)
{
	u32 vcell, soc;

	vcell = fg_read_vcell();
	soc = fg_read_soc();

	printk("%s : vcell(%d), soc(%d)\n", __func__, vcell, soc);
	
	// if read operation fails, then it's not alive status
	if( (vcell < 0) || (soc < 0) )
		return 0;
	else
		return 1;
}


extern int soc_restart_flag;
int fg_adjust_capacity(void)
{
	struct i2c_client *client = fg_i2c_client;
	u8 data[2];
	s32 ret = 0;

	if(!fuel_guage_init) {
		printk("%s : fuel guage IC is not initialized!!\n", __func__);
		return -1;
	}

	data[0] = 0;
	data[1] = 0;
	
	// 1. Write RemCapREP(05h)=0;
	if (fg_i2c_write(client, REMCAP_REP_REG, data, (u8)2) < 0) {
		pr_err("%s: Failed to write RemCap_REP\n", __func__);
		return -1;
	}
#if 0  // Not used (Recommendation from MAXIM)
	// 2. Write RemCapMIX(0Fh)=0;
	if (fg_i2c_write(client, REMCAP_MIX_REG, data, (u8)2) < 0) {
		pr_err("%s: Failed to write RemCap_MIX\n", __func__);
		return -1;
	}

	// 3. Write RemCapAV(1Fh)=0;
	if (fg_i2c_write(client, REMCAP_AV_REG, data, (u8)2) < 0) {
		pr_err("%s: Failed to write RemCap_AV\n", __func__);
		return -1;
	}

	//4. Write RepSOC(06h)=0;
	if (fg_i2c_write(client, SOCREP_REG, data, (u8)2) < 0) {
		pr_err("%s: Failed to write SOC_REP\n", __func__);
		return -1;
	}

	//5. Write MixSOC(0Dh)=0;
	if (fg_i2c_write(client, SOCMIX_REG, data, (u8)2) < 0) {
		pr_err("%s: Failed to write SOC_MIX\n", __func__);
		return -1;
	}

	//6. Write SOCAV(0Eh)=Table_SOC;
	if (fg_i2c_write(client, SOCAV_REG, data, (u8)2) < 0) {
		pr_err("%s: Failed to write SOC_AV\n", __func__);
		return -1;
	}
#endif
	msleep(200);

	printk("%s : After adjust - RepSOC(%d)\n", __func__, fg_read_soc());

	soc_restart_flag = 1;  // Set flag

	return ret;
}


extern int low_batt_comp_flag;
void fg_low_batt_compensation(u32 level)
{
	u16 read_val = 0;
	u32 temp = 0;
	u16 tempVal=0;

	printk("%s : Adjust SOCrep to %d!!\n", __func__, level);

	//1) RemCapREP (05h) = FullCap(10h) x 0.034 (or 0.014)
	read_val = fg_read_register(0x10);
	temp = read_val * (level*10 + 4) / 1000;
	fg_write_register(0x05, (u16)temp);

	//2) RemCapMix(0Fh) = RemCapREP
//	fg_write_register(0x0f, (u16)temp);

	//3) RemCapAV(1Fh) = RemCapREP; 
//	fg_write_register(0x1f, (u16)temp);

	//4) RepSOC (06h) = 3.4% or 1.4%
	tempVal=(u16)((level << 8) | 0x67);  // 103(0x67) * 0.0039 = 0.4%
	fg_write_register(0x06, tempVal);

	//5) MixSOC (0Dh) = RepSOC
	fg_write_register(0x0D, tempVal);

	//6) AVSOC (0Eh) = RepSOC; 
	fg_write_register(0x0E, tempVal);	

	low_batt_comp_flag = 1;  // Set flag
	
}


void fg_test_read(void)
{
	u8 data[2], reg;
	struct i2c_client *client = fg_i2c_client;

	for(reg=0; reg<0x40; reg++)
	{
		if(reg != 0x0C && reg != 0x13 && reg != 0x15 && reg != 0x20 && reg != 0x22
			&& reg != 0x26 && reg != 0x28 && reg != 0x30 && reg != 0x31 && reg != 0x34
			&& reg != 0x35 && reg != 0x3C && reg != 0x3E)
		{
			fg_i2c_read(client, reg, data, (u8)2);
 			printk("%s - addr(0x%02x), data(0x%04x)\n", __func__, reg, (data[1]<<8) | data[0]);
		}
	}

	printk("%s - addr(0x%02x), data(0x%04x)\n", __func__, (u8)VFOCV_REG, (u16)fg_read_register((u8)VFOCV_REG));
	printk("%s - addr(0x%02x), data(0x%04x)\n", __func__, (u8)VFSOC_REG, (u16)fg_read_register((u8)VFSOC_REG));
}


void fg_periodic_read(void)
{
//	struct i2c_client *client = fg_i2c_client;
	u8 data[2], reg;
	struct timespec ts;
	struct rtc_time tm;

	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);

	printk("[MAX17042] %d/%d/%d %02d:%02d,",
		tm.tm_mday, tm.tm_mon + 1, tm.tm_year + 2000, tm.tm_hour, tm.tm_min);

	for(reg = 0; reg < 0x50; reg++)
		printk("%04xh,", fg_read_register(reg));

	for(reg = 0xe0; reg < 0x100; reg++)
	{
		if(reg==0xff) {
			printk("%04xh\n", fg_read_register(reg));
			break;
		}
		else
			printk("%04xh,", fg_read_register(reg));
	}

}


void fg_read_model_data(void)
{
	struct i2c_client *client = fg_i2c_client;
	u16 data0[16], data1[16], data2[16];
	int i = 0;

	printk("[FG_Model] ");

	// Unlock model access
	fg_write_register(0x62, 0x0059);  // Unlock Model Access
	fg_write_register(0x63, 0x00C4);

	// Read model data
	fg_read_16register(0x80, data0);
	fg_read_16register(0x90, data1);
	fg_read_16register(0xa0, data2);

	// Print model data
	for(i = 0; i < 16; i++)
		printk("0x%04x, ", data0[i]);

	for(i = 0; i < 16; i++)
		printk("0x%04x, ", data1[i]);

	for(i = 0; i < 16; i++) {
		if(i==15)
			printk("0x%04x", data2[i]);
		else
			printk("0x%04x, ", data2[i]);
	}

relock:
	// Lock model access
	fg_write_register(0x62, 0x0000);  // Lock Model Access
	fg_write_register(0x63, 0x0000);

	// Read model data again
	fg_read_16register(0x80, data0);
	fg_read_16register(0x90, data1);
	fg_read_16register(0xa0, data2);

	for(i=0; i<16; i++) {
		if( data0[i] || data1[i] || data2[i]) {
			printk("%s : data is non-zero, lock again!!\n", __func__);
			goto relock;
		}
	}
	
}


void fg_test_print(void)
{
	struct i2c_client *client = fg_i2c_client;
	u8 data[2];
	u32 average_vcell = 0;
	u16 w_data;
	u32 temp;
	u32 temp2;
	u16 reg_data;

	if(!fuel_guage_init) {
		printk("%s : fuel guage IC is not initialized!!\n", __func__);
		return ;
	}

	if (fg_i2c_read(client, AVR_VCELL_REG, data, (u8)2) < 0) {
		pr_err("%s: Failed to read VCELL\n", __func__);
		return ;
	}

	w_data = (data[1]<<8) | data[0];

	temp = (w_data & 0xFFF) * 78125;
	average_vcell = temp / 1000000;

	temp = ((w_data & 0xF000) >> 4) * 78125;
	temp2 = temp / 1000000;
	average_vcell += (temp2 << 4);

	printk("%s : AVG_VCELL(%d), data(0x%04x)\n", __func__, average_vcell, (data[1]<<8) | data[0]);

	reg_data = fg_read_register(FULLCAP_REG);
	printk("%s : FULLCAP(%d), data(0x%04x)\n", __func__, reg_data/2, reg_data);

	reg_data = fg_read_register(REMCAP_MIX_REG);
	printk("%s : REMCAP_MIX(%d), data(0x%04x)\n", __func__, reg_data/2, reg_data);

	reg_data = fg_read_register(REMCAP_AV_REG);
	printk("%s : REMCAP_AV(%d), data(0x%04x)\n", __func__, reg_data/2, reg_data);

}


#ifdef MAX17042
int fg_alert_init(void)
{
	struct i2c_client *client = fg_i2c_client;
	u8 misccgf_data[2];
	u8 salrt_data[2];	
	u8 config_data[2];	
	u8 valrt_data[2];
	u8 talrt_data[2];
	u16 read_data = 0;

	if(!fuel_guage_init) {
		printk("%s : fuel guage IC is not initialized!!\n", __func__);
		return -1;
	}

	// Using RepSOC
	if (fg_i2c_read(client, MISCCFG_REG, misccgf_data, (u8)2) < 0) {
		pr_err("%s: Failed to read MISCCFG_REG\n", __func__);
		return -1;
	}
	misccgf_data[0] = misccgf_data[0] & ~(0x03);	
	
	if(fg_i2c_write(client, MISCCFG_REG, misccgf_data, (u8)2))
	{
		pr_info("%s: Failed to write MISCCFG_REG\n", __func__);
		return -1;
	}

	// SALRT Threshold setting
	salrt_data[1]=0xff;
	salrt_data[0]=0x01; //1%
	if(fg_i2c_write(client, SALRT_THRESHOLD_REG, salrt_data, (u8)2))
	{
		pr_info("%s: Failed to write SALRT_THRESHOLD_REG\n", __func__);
		return -1;	
	}
	
rewrite_valrt:
	// Reset VALRT Threshold setting (disable)
	valrt_data[1] = 0xFF;
	valrt_data[0] = 0x00;
	if(fg_i2c_write(client, VALRT_THRESHOLD_REG, valrt_data, (u8)2))
	{
		pr_info("%s: Failed to write VALRT_THRESHOLD_REG\n", __func__);
		return -1;
	}

	read_data = fg_read_register((u8)VALRT_THRESHOLD_REG);
	if(read_data != 0xff00) {
		printk(KERN_ERR "%s : VALRT_THRESHOLD_REG is not valid (0x%x)\n", __func__, read_data);
//		goto rewrite_valrt;
	}

rewrite_talrt:
	// Reset TALRT Threshold setting (disable)
	talrt_data[1] = 0x7F;
	talrt_data[0] = 0x80;
	if(fg_i2c_write(client, TALRT_THRESHOLD_REG, talrt_data, (u8)2))
	{
		pr_info("%s: Failed to write TALRT_THRESHOLD_REG\n", __func__);
		return -1;
	}

	read_data = fg_read_register((u8)TALRT_THRESHOLD_REG);
	if(read_data != 0x7f80) {
		printk(KERN_ERR "%s : TALRT_THRESHOLD_REG is not valid (0x%x)\n", __func__, read_data);
//		goto rewrite_talrt;
	}

	mdelay(100);
	
	// Enable SOC alerts
	if (fg_i2c_read(client, CONFIG_REG, config_data, (u8)2) < 0) {
		pr_err("%s: Failed to read CONFIG_REG\n", __func__);
		return -1;
	}
	config_data[0] = config_data[0] | (0x1 << 2);	
	
	if(fg_i2c_write(client, CONFIG_REG, config_data, (u8)2))
	{
		pr_info("%s: Failed to write CONFIG_REG\n", __func__);
		return -1;
	}
		
	return 1;	
}


int fg_check_status_reg(void)
{
	struct i2c_client *client = fg_i2c_client;
	u8 status_data[2];
	int ret = 0;
	
	if(!fuel_guage_init) {
		printk("%s : fuel guage IC is not initialized!!\n", __func__);
		return -1;
	}

	// 1. Check Smn was generatedread
	if (fg_i2c_read(client, STATUS_REG, status_data, (u8)2) < 0) {
		pr_err("%s: Failed to read STATUS_REG\n", __func__);
		return -1;
	}
	printk("%s - addr(0x00), data(0x%04x)\n", __func__, (status_data[1]<<8) | status_data[0]);

	if(status_data[1] & (0x1 << 2))
		ret = 1;

	// 2. clear Status reg
	status_data[1] = 0;
	if(fg_i2c_write(client, STATUS_REG, status_data, (u8)2))
	{
		pr_info("%s: Failed to write STATUS_REG\n", __func__);
		return -1;
	}
	
	return ret;
}


int fg_check_battery_present(void)
{
	struct i2c_client *client = fg_i2c_client;
	u8 status_data[2];
	int ret = 1;
	
	if(!fuel_guage_init) {
		printk("%s : fuel guage IC is not initialized!!\n", __func__);
		return -1;
	}

	// 1. Check Bst bit
	if (fg_i2c_read(client, STATUS_REG, status_data, (u8)2) < 0) {
		pr_err("%s: Failed to read STATUS_REG\n", __func__);
		return -1;
	}

	if(status_data[0] & (0x1 << 3))
	{
		printk("%s - addr(0x01), data(0x%04x)\n", __func__, (status_data[1]<<8) | status_data[0]);
		printk("%s : battery is absent!!\n", __func__);
		ret = 0;
	}

	return ret;
}


void fg_fullcharged_compensation(u32 is_recharging, u32 pre_update)
{
	static u16 NewFullCap_data = 0;
	
	if(!fuel_guage_init) {
		printk("%s : fuel guage IC is not initialized!!\n", __func__);
		return;
	}

	printk("%s : is_recharging(%d), pre_update(%d)\n", __func__, is_recharging, pre_update);

	NewFullCap_data = fg_read_register(FULLCAP_REG);

	if(NewFullCap_data > (Capacity * 110 / 100))
	{
		printk("%s : [Case 1] PrevFullCap = 0x%04x, NewFullCap = 0x%04x\n",
			__func__, PrevFullCap, NewFullCap_data);

		NewFullCap_data = (Capacity * 110) / 100;

//		fg_write_register(REMCAP_MIX_REG, (u16)(NewFullCap_data));
		fg_write_register(REMCAP_REP_REG, (u16)(NewFullCap_data));
		fg_write_register(FULLCAP_REG, (u16)(NewFullCap_data));
	}
	else if(NewFullCap_data < (Capacity * 70 / 100))
	{
		printk("%s : [Case 5] PrevFullCap = 0x%04x, NewFullCap = 0x%04x\n",
			__func__, PrevFullCap, NewFullCap_data);

		NewFullCap_data = (Capacity * 70) / 100;

//		fg_write_register(REMCAP_MIX_REG, (u16)(NewFullCap_data));
		fg_write_register(REMCAP_REP_REG, (u16)(NewFullCap_data));
		fg_write_register(FULLCAP_REG, (u16)(NewFullCap_data));
	}
	else
	{
		if(NewFullCap_data > (PrevFullCap * 105 / 100))
		{
			printk("%s : [Case 2] PrevFullCap = 0x%04x, NewFullCap = 0x%04x\n",
				__func__, PrevFullCap, NewFullCap_data);

			NewFullCap_data = (PrevFullCap * 105) / 100;

//			fg_write_register(REMCAP_MIX_REG, (u16)(NewFullCap_data));
			fg_write_register(REMCAP_REP_REG, (u16)(NewFullCap_data));
			fg_write_register(FULLCAP_REG, (u16)(NewFullCap_data));
		}
		else if(NewFullCap_data < (PrevFullCap * 90 / 100))
		{
			printk("%s : [Case 3] PrevFullCap = 0x%04x, NewFullCap = 0x%04x\n",
				__func__, PrevFullCap, NewFullCap_data);
		
			NewFullCap_data = (PrevFullCap * 90) / 100;

//			fg_write_register(REMCAP_MIX_REG, (u16)(NewFullCap_data));
			fg_write_register(REMCAP_REP_REG, (u16)(NewFullCap_data));
			fg_write_register(FULLCAP_REG, (u16)(NewFullCap_data));
		}
		else
		{
			printk("%s : [Case 4] PrevFullCap = 0x%04x, NewFullCap = 0x%04x\n",
				__func__, PrevFullCap, NewFullCap_data);

			// Do nothing...
		}
	}

	// In case of recharging, re-write FirstFullChargedCAP to FullCAP, RemCAP_REP.
	if(!is_recharging)
		FirstFullChargedCAP = NewFullCap_data;
	else {
		printk("%s : [Case 6] FirstFullCap = 0x%04x, NewFullCap = 0x%04x\n",
			__func__, FirstFullChargedCAP, NewFullCap_data);

		fg_write_register(REMCAP_REP_REG, (u16)(FirstFullChargedCAP));
		fg_write_register(FULLCAP_REG, (u16)(FirstFullChargedCAP));
	}

	//4. Write RepSOC(06h)=100%;
	fg_write_register(SOCREP_REG, (u16)(0x64 << 8));

	//5. Write MixSOC(0Dh)=100%;
	fg_write_register(SOCMIX_REG, (u16)(0x64 << 8));

	//6. Write AVSOC(0Eh)=100%;
	fg_write_register(SOCAV_REG, (u16)(0x64 << 8));

	if(!pre_update)  // if pre_update case, skip updating PrevFullCAP value.
		PrevFullCap = fg_read_register(FULLCAP_REG);

	printk("%s : (A) FullCap = 0x%04x, RemCap = 0x%04x\n", __func__,
		fg_read_register(FULLCAP_REG), fg_read_register(REMCAP_REP_REG));

	fg_periodic_read();

}


void fg_check_vf_fullcap_range(void)
{
	static u16 NewVfFullCap = 0;
	u16 print_flag = 1;
	
	if(!fuel_guage_init) {
		printk("%s : fuel guage IC is not initialized!!\n", __func__);
		return ;
	}

	NewVfFullCap = fg_read_register(FullCAP_NOM_REG);

	if(NewVfFullCap > (VFCapacity * 110 / 100))
	{
		printk("%s : [Case 1] PrevVfFullCap = 0x%04x, NewVfFullCap = 0x%04x\n",
			__func__, PrevVfFullCap, NewVfFullCap);

		NewVfFullCap = (VFCapacity * 110) / 100;

		fg_write_register(dQacc_REG, (u16)(NewVfFullCap / 4));
		fg_write_register(dPacc_REG, (u16)0x3200);
	}
	else if(NewVfFullCap < (VFCapacity * 70 / 100))
	{
		printk("%s : [Case 5] PrevVfFullCap = 0x%04x, NewVfFullCap = 0x%04x\n",
			__func__, PrevVfFullCap, NewVfFullCap);

		NewVfFullCap = (VFCapacity * 70) / 100;

		fg_write_register(dQacc_REG, (u16)(NewVfFullCap / 4));
		fg_write_register(dPacc_REG, (u16)0x3200);
	}
	else
	{
		if(NewVfFullCap > (PrevVfFullCap * 105 / 100))
		{
			printk("%s : [Case 2] PrevVfFullCap = 0x%04x, NewVfFullCap = 0x%04x\n",
				__func__, PrevVfFullCap, NewVfFullCap);

			NewVfFullCap = (PrevVfFullCap * 105) / 100;

			fg_write_register(dQacc_REG, (u16)(NewVfFullCap / 4));
			fg_write_register(dPacc_REG, (u16)0x3200);
		}
		else if(NewVfFullCap < (PrevVfFullCap * 90 / 100))
		{
			printk("%s : [Case 3] PrevVfFullCap = 0x%04x, NewVfFullCap = 0x%04x\n",
				__func__, PrevVfFullCap, NewVfFullCap);
		
			NewVfFullCap = (PrevVfFullCap * 90) / 100;
		
			fg_write_register(dQacc_REG, (u16)(NewVfFullCap / 4));
			fg_write_register(dPacc_REG, (u16)0x3200);
		}
		else
		{
			printk("%s : [Case 4] PrevVfFullCap = 0x%04x, NewVfFullCap = 0x%04x\n",
				__func__, PrevVfFullCap, NewVfFullCap);

			// Do nothing...
			print_flag = 0;
		}
	}

	PrevVfFullCap = fg_read_register(FullCAP_NOM_REG);

	if(print_flag)
		printk("%s : VfFullCap(0x%04x), dQacc(0x%04x), dPacc(0x%04x)\n", __func__,
			fg_read_register(FullCAP_NOM_REG), fg_read_register(dQacc_REG), fg_read_register(dPacc_REG));

}


static u32 prevVfSOC = 0;
static u32 prevRepSOC = 0;
static u32 prevRemCap = 0;
static u32 prevMixCap = 0;
static u32 prevFullCapacity= 0;
static u32 prevVFCapacity= 0;
static u32 prevVfOCV = 0;

int fg_check_cap_corruption(void)
{
	u32 VfSOC = fg_read_vfsoc();
	u32 RepSOC = fg_read_soc();
	u32 MixCap = fg_read_register(REMCAP_MIX_REG);
	u32 VfOCV = fg_read_register(VFOCV_REG);
	u32 RemCap = fg_read_register(REMCAP_REP_REG);
	u32 FullCapacity= fg_read_register(FULLCAP_REG);
	u32 VfFullCapacity = fg_read_register(FullCAP_NOM_REG);
	u32 temp, temp2, newVfOCV, pr_vfocv;
	unsigned long flag = 0;
	int ret = 0;

	if(!fuel_guage_init) {
		printk("%s : fuel guage IC is not initialized!!\n", __func__);
		return 0;
	}

	// If usgin Jig, then skip checking.
	if(FSA9480_Get_JIG_Status()) {
		printk("%s : Return by Using Jig(%d)\n", __func__, FSA9480_Get_JIG_Status());
		return 0;  // it's ok
	}

	// Check full charge learning case.
	if( ((VfSOC >= 70) && ((RemCap >= (FullCapacity * 995 / 1000)) && (RemCap <= (FullCapacity * 1005 / 1000))))
		|| low_batt_comp_flag || soc_restart_flag)
	{
		printk("%s : RemCap(%d), FullCap(%d), SOC(%d), low_batt_comp_flag(%d), soc_restart_flag(%d)\n",
			__func__, (RemCap/2), (FullCapacity/2), RepSOC, low_batt_comp_flag, soc_restart_flag);
		prevRepSOC = RepSOC;
		prevRemCap = RemCap;
		prevFullCapacity= FullCapacity;
		if(soc_restart_flag)  // reset flag
			soc_restart_flag = 0;

		ret = 1;   // recover case
}

	// ocv calculation for print
	temp = (VfOCV & 0xFFF) * 78125;
	pr_vfocv = temp / 1000000;
	
	temp = ((VfOCV & 0xF000) >> 4) * 78125;
	temp2 = temp / 1000000;
	pr_vfocv += (temp2 << 4);

	printk("%s : VfSOC(%d), RepSOC(%d), MixCap(%d), VfOCV(0x%04x, %d)\n",
		__func__, VfSOC, RepSOC, (MixCap/2), VfOCV, pr_vfocv);

	if( ( ((VfSOC+5) < prevVfSOC) || (VfSOC > (prevVfSOC+5)) )
		|| ( ((RepSOC+5) < prevRepSOC) || (RepSOC > (prevRepSOC+5)) )
		|| ( ((MixCap+530) < prevMixCap) || (MixCap > (prevMixCap+530)) ) )  // MixCap differ is greater than 265mAh
	{
		fg_periodic_read();

		printk("[FG_Recovery] (B) VfSOC(%d), prevVfSOC(%d), RepSOC(%d), prevRepSOC(%d), MixCap(%d), prevMixCap(%d)\n",
			VfSOC, prevVfSOC, RepSOC, prevRepSOC, (MixCap/2), (prevMixCap/2));

		spin_lock_irqsave(&fg_lock, flag);

		fg_write_and_verify_register(REMCAP_MIX_REG , prevMixCap);  // MixCap
		fg_write_register(VFOCV_REG, prevVfOCV);
		mdelay(200);

		fg_write_and_verify_register(REMCAP_REP_REG, prevRemCap);  // RemCAP
		VfSOC = fg_read_register(VFSOC_REG);  // read VFSOC
		fg_write_register(0x60, 0x0080);	 // Enable Write Access to VFSOC0
		fg_write_and_verify_register(0x48, VfSOC);  // VFSOC0
		fg_write_register(0x60, 0x0000); 	 // Disable Write Access to VFSOC0

		fg_write_and_verify_register(0x45, (prevVFCapacity / 4));  // dQ_acc
		fg_write_and_verify_register(0x46, 0x3200);  // dP_acc
		fg_write_and_verify_register(FULLCAP_REG, prevFullCapacity);
		fg_write_and_verify_register(FullCAP_NOM_REG, prevVFCapacity);  // FullCAPNom

		spin_unlock_irqrestore(&fg_lock, flag);

		msleep(200);

//		prevVfSOC = fg_read_vfsoc();
//		prevRepSOC = fg_read_soc();
//		prevRemCap = fg_read_register(REMCAP_REP_REG);
//		prevMixCap = fg_read_register(REMCAP_MIX_REG);
//		prevFullCapacity= fg_read_register(FULLCAP_REG);
//		prevVFCapacity = fg_read_register(FullCAP_NOM_REG);
//		prevVfOCV = fg_read_register(VFOCV_REG);

		// ocv calculation for print
		newVfOCV = fg_read_register(VFOCV_REG);
		temp = (newVfOCV & 0xFFF) * 78125;
		pr_vfocv = temp / 1000000;

		temp = ((newVfOCV & 0xF000) >> 4) * 78125;
		temp2 = temp / 1000000;
		pr_vfocv += (temp2 << 4);

		printk("[FG_Recovery] (A) newVfSOC(%d), newRepSOC(%d), newMixCap(%d), newVfOCV(0x%04x, %d)\n",
			fg_read_vfsoc(), fg_read_soc(), (fg_read_register(REMCAP_MIX_REG)/2), newVfOCV, pr_vfocv);
		
		fg_periodic_read();

		ret = 1;
	}
	else {
		prevVfSOC = VfSOC;
		prevRepSOC = RepSOC;
		prevRemCap = RemCap;
		prevMixCap = MixCap;
		prevFullCapacity= FullCapacity;
		prevVFCapacity = VfFullCapacity;
		prevVfOCV = VfOCV;
	}

	return ret;
}

#endif


void fg_set_battery_type(void)
{
	u16 data = 0;
	u8 type_str[10];

	data = fg_read_register(DESIGNCAP_REG);

#ifndef CONFIG_TARGET_LOCALE_USAGSM
	if((data == SDI_VFCapacity) || (data == SDI_VFCapacity-1))
		battery_type = SDI_BATTERY_TYPE;
	else if((data == ATL_VFCapacity) || (data == ATL_VFCapacity-1))
		battery_type = ATL_BATTERY_TYPE;
#else
		battery_type = SDI_BATTERY_TYPE;
#endif

	if(battery_type == SDI_BATTERY_TYPE)
		sprintf(type_str, "SDI");
	else if(battery_type == ATL_BATTERY_TYPE)
		sprintf(type_str, "ATL");
	else
		sprintf(type_str, "Unknown");

	printk("%s : DesignCAP(0x%04x), Battery type(%s)\n", __func__, data, type_str);

	switch(battery_type) {
	case ATL_BATTERY_TYPE:
		Capacity = ATL_Capacity;
		VFCapacity = ATL_VFCapacity;
		break;

	case SDI_BATTERY_TYPE:
	default:
		Capacity = SDI_Capacity;
		VFCapacity = SDI_VFCapacity;
		break;
	}

}


void fuel_gauge_rcomp(void)
{
#ifndef CONFIG_MACH_S5PC110_P1
	struct i2c_client *client = fg_i2c_client;
	u8 rst_cmd[2];
	s32 ret = 0;

	if(!fuel_guage_init) {
		printk("%s : fuel guage IC is not initialized!!\n", __func__);
		return ;
	}
#if 0
	rst_cmd[0] = 0xa0;
	rst_cmd[1] = 0x00;

	ret = fg_i2c_write(client, RCOMP_REG, rst_cmd, (u8)2);
	if (ret)
		pr_info("%s: failed fuel_gauge_rcomp(%d)\n", __func__, ret);
	
	msleep(500);
#endif
#endif
}


int fg_read_rcomp(void)
{
#ifndef CONFIG_MACH_S5PC110_P1
	struct i2c_client *client = fg_i2c_client;
	u8 data[2];
	u16 rcomp = 0;

	if(!fuel_guage_init) {
		printk("%s : fuel guage IC is not initialized!!\n", __func__);
		return -1;
	}
#if 0
	if (fg_i2c_read(client, RCOMP_REG, data, (u8)2) < 0) {
		pr_err("%s: Failed to read RCOMP\n", __func__);
		return -1;
	}

	rcomp = (data[1]<<8) | data[0];
#endif
	return rcomp;
#else
	return 0;
#endif
}


static int fg_i2c_remove(struct i2c_client *client)
{
	struct fg_state *fg = i2c_get_clientdata(client);

	kfree(fg);
	return 0;
}

static int fg_i2c_probe(struct i2c_client *client,  const struct i2c_device_id *id)
{
	struct fg_state *fg;
	unsigned long flag = 0;

	fg = kzalloc(sizeof(struct fg_state), GFP_KERNEL);
	if (fg == NULL) {		
		printk("failed to allocate memory \n");
		return -ENOMEM;
	}
	
	fg->client = client;
	i2c_set_clientdata(client, fg);
	
	/* rest of the initialisation goes here. */
	
	printk("Fuel guage attach success!!!\n");

	fg_i2c_client = client;

	fuel_guage_init = 1;

	fg_set_battery_type();

	// Init parameters to prevent wrong compensation.
	PrevFullCap = fg_read_register(FULLCAP_REG);
	PrevVfFullCap = fg_read_register(FullCAP_NOM_REG);
	FirstFullChargedCAP = PrevFullCap;  // Init FullCAP of first full charging.

	prevVfSOC = fg_read_vfsoc();
	prevRepSOC = fg_read_soc();
	prevRemCap = fg_read_register(REMCAP_REP_REG);
	prevMixCap = fg_read_register(REMCAP_MIX_REG);
	prevVfOCV = fg_read_register(VFOCV_REG);
	prevFullCapacity = PrevFullCap;
	prevVFCapacity = PrevVfFullCap;

	spin_lock_init(&fg_lock);

	spin_lock_irqsave(&fg_lock, flag);
	fg_read_model_data();
	spin_unlock_irqrestore(&fg_lock, flag);
	
	return 0;
}


static const struct i2c_device_id fg_device_id[] = {
	{"fuelgauge", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, fg_device_id);


static struct i2c_driver fg_i2c_driver = {
	.driver = {
		.name = "fuelgauge",
		.owner = THIS_MODULE,
	},
	.probe	= fg_i2c_probe,
	.remove	= fg_i2c_remove,
	.id_table	= fg_device_id,
};


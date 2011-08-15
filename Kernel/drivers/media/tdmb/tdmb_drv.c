/* drviers/media/tdmb/tdmb_drv.c
 *
 * TDMB_DRV for Linux
 *
 * klaatu, Copyright (c) 2009 Samsung Eclectronics
 * 			http://www.samsung.com/
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fcntl.h>

#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

#include <mach/gpio.h>
//#include <mach/mux.h>
#include <asm/io.h>

#include <linux/fs.h>
#include <asm/uaccess.h>

#include <linux/time.h>
#include <linux/timer.h>
#include <linux/i2c/twl4030.h>

#include <plat/gpio-cfg.h>
#include <mach/regs-gpio.h>
//#include <plat/gpio-bank-h3.h>

#include "tdmb.h"

#ifdef CONFIG_TDMB_GDM7024
#include "gdm_headers.h"
#include "tdmb_drv_gct.h"
#endif


#ifdef CONFIG_TDMB_T3700
#include "INC_INCLUDES.h"
#endif

// --------------------------------
#define TDMB_DEBUG

#ifdef TDMB_DEBUG
#define DPRINTK(x...) printk("TDMB " x)
#else /* TDMB_DEBUG */
#define DPRINTK(x...)  /* null */
#endif /* TDMB_DEBUG */

static DEFINE_MUTEX(tdmb_int_mutex);

tdmb_type g_TDMBGlobal; 

// --------------------------------
// definition


#ifdef CONFIG_TDMB_T3700
#define TDMB_ID_0x80        0x80
#endif

//// GPIO //////////////
//#define GPIO_TDMB_EN	OMAP_GPIO_TDMB_EN
//#define GPIO_TDMB_RST	OMAP_GPIO_TDMB_RST
//#define GPIO_TDMB_INT	OMAP_GPIO_TDMB_INT
#define GPIO_TDMB_RST GPIO_TDMB_RST_N

//#define IRQ_TDMB_INT	gpio_to_irq(GPIO_TDMB_INT)
#define IRQ_TDMB_INT	IRQ_EINT12   //IRQ_EINT10

#define GPIO_LEVEL_LOW   0 
#define GPIO_LEVEL_HIGH  1
////////////////////////
// TODO
extern unsigned int *ts_head;
extern unsigned int *ts_tail;
extern char *ts_buffer;
extern unsigned int ts_size;

#define FALSE 0
#define TRUE 1

extern void tdmb_work_function(void);
static struct workqueue_struct *tdmb_workqueue=0;

extern void spi_open();
// --------------------------------

DECLARE_WORK(tdmb_work, tdmb_work_function);


int IsTDMBPowerOn(void) 
{ 
    return g_TDMBGlobal.b_isTDMB_Enable; 
} 

static irqreturn_t TDMB_irq_handler(int irq, void *dev_id)
{
	int ret = 0;

	if(!tdmb_workqueue)
	{
		DPRINTK("tdmb_workqueue doesn't exist!\n");
		tdmb_workqueue=create_singlethread_workqueue("ktdmbd");

		if(!tdmb_workqueue)
		{
			DPRINTK("failed to create_workqueue\n");
			return TDMB_ERROR;
		}
	}

	ret=queue_work(tdmb_workqueue, &tdmb_work);
	if(ret==0)
	{
		DPRINTK("failed in queue_work\n");
	}
/*    
	else
	{
		DPRINTK("suceeded in queue_work\n");
	}
*/
	return IRQ_HANDLED;
}

unsigned int TDMB_drv_Init(void)
{
	unsigned int dRet = TDMB_ERROR;
   //define tdmb workqueue
	DPRINTK("Work Queue Init ! \r\n");

	tdmb_workqueue = create_singlethread_workqueue("ktdmbd");

	if(!tdmb_workqueue)
	{
		DPRINTK("failed create_singlethread_workqueue\n");
		return TDMB_ERROR;
	}

	set_irq_type(IRQ_TDMB_INT,IRQ_TYPE_EDGE_FALLING); // TODO
	
	dRet = request_irq(IRQ_TDMB_INT, TDMB_irq_handler, IRQF_DISABLED, "TDMB", NULL);
	if(dRet < 0)
	{
		DPRINTK("request_irq failed !! \r\n");
		return TDMB_ERROR;
	}
	return TDMB_SUCCESS;
}

void TDMB_drv_PowerOn(void)// power on
{

#if defined(CONFIG_TDMB_T3700)
    // TDMB Reset
    gpio_request(GPIO_TDMB_EN,"tdmb" );
	gpio_direction_output(GPIO_TDMB_EN, GPIO_LEVEL_HIGH);
    msleep(10);
    DPRINTK("TDMB_drv_PowerOn !! 1\r\n");
    gpio_request(GPIO_TDMB_RST,"tdmb" );
    gpio_direction_output(GPIO_TDMB_RST, GPIO_LEVEL_LOW);
    msleep(2);
    DPRINTK("TDMB_drv_PowerOn !! 7\r\n");
    gpio_set_value(GPIO_TDMB_RST, GPIO_LEVEL_HIGH);    
    msleep(10);
    DPRINTK("TDMB_drv_PowerOn !! 5\r\n");
#elif defined(CONFIG_TDMB_GDM7024)
    gpio_request(GPIO_TDMB_EN,"tdmb" );
    gpio_direction_output(GPIO_TDMB_EN, GPIO_LEVEL_LOW);
	gpio_set_value(GPIO_TDMB_EN, GPIO_LEVEL_HIGH);
	msleep(10);
	//VMMC2 - klaatu tdmb
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, LDO_1_8V, TWL4030_VMMC2_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, (DEV_GRP_P1 << 5), TWL4030_VMMC2_DEV_GRP);
	msleep(100);
#endif	
	DPRINTK("TDMB_drv_PowerOn !! 3\r\n");
}

void TDMB_drv_Enable()
{
	// initialize spi interface
	spi_open();
#ifdef CONFIG_TDMB_GDM7024
	// define gdm hal functions
	gdm_hal_init_func();
	// gct binary download
	TDMB_drv_gct_download();
#endif
}

int TDMB_PowerOn(void)
{
#ifdef CONFIG_TDMB_T3700
	DPRINTK("call TDMB_PowerOn ! \n");
// 	TDMB_drv_Init();
	TDMB_drv_PowerOn();// power on by gpio
	DPRINTK("call TDMB_PowerOn ! 34\n");
    spi_open();
 	TDMB_drv_Init();
    DPRINTK("call TDMB_PowerOn ! 35\n");

    if(INC_SUCCESS != INTERFACE_INIT(TDMB_ID_0x80))
    {
        printk("[TDMB] tdmb power on failed\n");
        TDMB_PowerOff();
        return FALSE;
    }
    else
    {
    	g_TDMBGlobal.b_isTDMB_Enable = 1; 
    	printk("[TDMB] tdmb power on success\n");
    	return TRUE;
	}
/////////////////////////////////////////////
#if 0
    INC_UINT32 KOREnsembleNormalFreq[MAX_KOREABAND_NORMAL_CHANNEL] = 
    {
    	181280,183008,184736,205280,207008,
    	208736
    };
    int ScanLoop, nDataCnt;
    INC_CHANNEL_INFO* pDb;
    for(ScanLoop = 0; ScanLoop < MAX_KOREABAND_NORMAL_CHANNEL; ScanLoop++)
    {
        if(INTERFACE_SCAN(TDMB_ID_0x80, KOREnsembleNormalFreq[ScanLoop]) == INC_SUCCESS)
        {
           for(nDataCnt = 0 ; nDataCnt< INTERFACE_GETDMB_CNT(); nDataCnt++){
                pDb = INTERFACE_GETDB_DMB(nDataCnt);
                DPRINTK("DMB :: %s  ID %d \n", pDb->aucLabel, pDb->ucSubChID);
            }
           for(nDataCnt = 0 ; nDataCnt< INTERFACE_GETDAB_CNT(); nDataCnt++){
                pDb = INTERFACE_GETDB_DAB(nDataCnt);
                DPRINTK("DAB :: %s ID %d \n", pDb->aucLabel, pDb->ucSubChID);
           }
        }
    }



    ////////////////////////// CHANNEL START ////////////////////////////////
    INC_CHANNEL_INFO stDB;
    stDB.ulRFFreq = 208736;
    stDB.ucSubChID = 0 ;
    stDB.ucServiceType = 0x18;
    if(INTERFACE_START(TDMB_ID_0x80, &stDB) == INC_SUCCESS)
    {
        DPRINTK("INTERFACE_START GOOD \n");
    }
#endif
#if 0
////////////////////////////////  Read/Write Test code   //////////////////////////////// 
  int nLoop; 
  unsigned short usStatus; 

  for(nLoop =0; nLoop < 0x100; nLoop++) 
  { 
        INC_CMD_WRITE(0x80, APB_MPI_BASE + 0x1, nLoop); 
        usStatus = INC_CMD_READ(0x80, APB_MPI_BASE + 0x1); 
        if(usStatus  != nLoop) 
        { 
            // TODO ?�기??메시지 코드 ?�어주세??...
DPRINTK("test : usStatus 0x%x\n",usStatus); 
        } 
        else
{
DPRINTK("Good : usStatus 0x%x\n",usStatus);
}
  } 


//////////////////////////////  Register Burst Test code //////////////////////////////  
  #define TEST_BUFF        4 
//  int nLoop;
  usStatus = 0xAAAA; 
  unsigned short ausBuff[4]; 

  for(nLoop =0; nLoop < TEST_BUFF; nLoop++) 
  { 
        INC_CMD_WRITE(0x80, APB_MPI_BASE + 0x1 + nLoop, usStatus ); 
        usStatus = ~usStatus; 
  }
  INC_CMD_READ_BURST(0x80, APB_MPI_BASE + 0x1 , (unsigned char*)ausBuff, TEST_BUFF*2); 
  for(nLoop =0; nLoop < TEST_BUFF; nLoop++)
  { 
        if(ausBuff[nLoop] != usStatus) 
        { 
            // TODO ?�기??메시지 코드 ?�어주세??... 
DPRINTK("test2 : ausBuff 0x%x, 0x%x\n", ausBuff[nLoop],usStatus);
        } 
        usStatus = ~usStatus; 
  }
#endif
/////////////////////////////////////////////
#endif
   
#ifdef CONFIG_TDMB_GDM7024
	TDMB_drv_Init();
	TDMB_drv_PowerOn();// power on by gpio
	TDMB_drv_Enable();

	DPRINTK("call TDMB_PowerOn ! \n");

	// you have to figure out whether TDMB PowerOn really succeeded or not.
	g_TDMBGlobal.b_isTDMB_Enable = 1; 
	return TRUE;
#endif
}

void TDMB_PowerOff(void)
{
	DPRINTK("call TDMB_PowerOff ! \n");
	g_TDMBGlobal.b_isTDMB_Enable = 0; 
#ifdef CONFIG_TDMB_T3700
    INC_STOP(TDMB_ID_0x80);
#endif
#ifdef CONFIG_TDMB_GDM7024
	// give command to stop TDMB
	gdm_dab_stop(DAB_0_PATH); // you must decide parameter, say , SPI_0_PATH
	msleep(10);
#endif

	// disable ISR
	free_irq(IRQ_TDMB_INT, NULL);

	// flush workqueue
	flush_workqueue(tdmb_workqueue);
	destroy_workqueue(tdmb_workqueue);
    tdmb_workqueue = 0;

	ts_size = 0;
    
#ifdef CONFIG_TDMB_T3700
    gpio_set_value(GPIO_TDMB_RST, GPIO_LEVEL_LOW);    
	msleep(10);
    gpio_set_value(GPIO_TDMB_EN, GPIO_LEVEL_LOW);
	msleep(10);
#endif

#ifdef CONFIG_TDMB_GDM7024
	// power down by controling gpio
	gpio_set_value(GPIO_TDMB_EN, GPIO_LEVEL_LOW);
	msleep(10);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0, TWL4030_VMMC2_DEV_GRP);
#endif	
}
/*
void TDMB_PowerOff(void)
{
	DPRINTK("call TDMB_PowerOff ! \n");
	g_TDMBGlobal.b_isTDMB_Enable = 0; 
#ifdef CONFIG_TDMB_T3700
    gpio_set_value(GPIO_TDMB_EN, GPIO_LEVEL_LOW);
#endif
#ifdef CONFIG_TDMB_GDM7024
	// give command to stop TDMB
	gdm_dab_stop(DAB_0_PATH); // you must decide parameter, say , SPI_0_PATH
	msleep(10);
#endif

	// flush workqueue
	flush_workqueue(tdmb_workqueue);
	destroy_workqueue(tdmb_workqueue);

	// disable ISR
	free_irq(IRQ_TDMB_INT, NULL);

	ts_size = 0;

#ifdef CONFIG_TDMB_GDM7024
	// power down by controling gpio
	gpio_set_value(GPIO_TDMB_EN, GPIO_LEVEL_LOW);
	msleep(10);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0, TWL4030_VMMC2_DEV_GRP);
#endif	
}
*/

int TDMB_AddDataToRing(unsigned char* pData, unsigned long dwDataSize)
{
	int ret = 0;
	unsigned int size;
	unsigned int head;
	unsigned int tail;
	unsigned int dist;
	unsigned int temp_size;

//	DPRINTK("%s  \n", __FUNCTION__);

    if ( ts_size == 0 )
    {
        return ret;
    }

	size = dwDataSize;
	head = *ts_head;
	tail = *ts_tail;

//	DPRINTK("entering head:%d,tail:%d size:%d,ps_size:%d\n",head,tail,size,ts_size);

	//
	if(size > ts_size )
	{
		DPRINTK(" Error - size too large \n");
	}
	else
	{
		if( head >= tail )
			dist = head-tail;
		else
			dist = ts_size+head-tail;

//		DPRINTK("dist: %x\n", dist);

		if( (ts_size-dist) < size)
		{
			DPRINTK(" too small space is left in Ring Buffer!!\n");
		}
		else
		{
			if( head+size <= ts_size )
			{
				memcpy( (ts_buffer+head), (char*)pData, size);

				head += size;
				if(head == ts_size)
					head = 0;
			}
			else
			{
				temp_size = ts_size-head;
				temp_size = (temp_size/DMB_TS_SIZE)*DMB_TS_SIZE;

				if(temp_size>0)
				{
					memcpy( (ts_buffer+head), (char*)pData, temp_size);
				}
				memcpy( ts_buffer, (char*)(pData+temp_size), size-temp_size);
				head = size-temp_size;
			}
//			DPRINTK("< data > %x, %x, %x, %x \n",
//						*(ts_buffer+ *ts_head),
//						*(ts_buffer+ *ts_head +1),
//						*(ts_buffer+ *ts_head +2),
//						*(ts_buffer+ *ts_head +3) );

//			DPRINTK("exiting - head : %d\n",head);
			*ts_head = head;
		}
	}

    return ret; 
}

#ifdef TDMB_FROM_FILE // ########################################################################################

#define TDMB_FROM_FILE_BIN "/system/usr/tdmb/Test.TS"
#define TIME_STEP	(10*HZ/100) // 0.1sec

struct file *fp_tff = NULL;
struct timer_list tff_timer;

void TDMB_drv_FromFile_Open(void);
void tfftimer_exit(void);
void tff_work_function(void);
void tfftimer_registertimer( struct timer_list *timer, unsigned long timeover);

DECLARE_WORK(tdmb_work_fromfile, tff_work_function);

void TDMB_PowerOn_FromFile(void)
{
	TDMB_drv_FromFile_Open();
}

void TDMB_PowerOff_FromFile(void)
{
	g_TDMBGlobal.b_isTDMB_Enable = 0; 

	DPRINTK("%s\n",__FUNCTION__);
	tfftimer_exit();
}
int tff_read_from_file_to_dest(char *dest, int size)
{
	int ret=0; 
	int temp_size;
	int data_received=0;
	mm_segment_t oldfs;

	temp_size= size;

	oldfs = get_fs();
	set_fs(KERNEL_DS);
	do{
		temp_size -= ret;
		ret = fp_tff->f_op->read(fp_tff, dest, temp_size, &fp_tff->f_pos);
		DPRINTK("---> file read [ret:%d] [f_pos:%d] \n", ret, fp_tff->f_pos);
		if(ret < temp_size)
		{
			DPRINTK(" file from the first \n");
			fp_tff->f_op->llseek(fp_tff, 0, SEEK_SET);
		}
		data_received += ret;
	}while(ret < temp_size);
	
	set_fs(oldfs);

	return ret;
}

void tff_work_function(void)
{
	int ret = 0;
	mm_segment_t oldfs;
	unsigned int size;
	unsigned int head;
	unsigned int tail;
	unsigned int dist;
	unsigned int temp_size;

	DPRINTK("%s  \n", __FUNCTION__);

	size = DMB_TS_SIZE*40;
	head = *ts_head;
	tail = *ts_tail;

	DPRINTK("entering head:%d,tail:%d size:%d,ps_size:%d\n",head,tail,size,ts_size);

	//
	if(size > ts_size )
	{
		DPRINTK(" Error - size too large \n");
	}
	else
	{
		if( head >= tail )
			dist = head-tail;
		else
			dist = ts_size+head-tail;

		DPRINTK("dist: %x\n", dist);

		if( (ts_size-dist) < size)
		{
			DPRINTK(" too small space is left in Ring Buffer!!\n");
		}
		else
		{

			//
			if( head+size <= ts_size )
			{
				ret=tff_read_from_file_to_dest( (ts_buffer+head), size);

				head += ret;
				if(head == ts_size)
					head = 0;
			}
			else
			{
				temp_size = ts_size-head;
				temp_size = (temp_size/DMB_TS_SIZE)*DMB_TS_SIZE;

				if(temp_size>0)
				{
					ret=tff_read_from_file_to_dest( (ts_buffer+head), temp_size );
					temp_size=ret;
				}

				ret=tff_read_from_file_to_dest( ts_buffer, size-temp_size);

				head = size-temp_size;
			}

			DPRINTK("< data > %x, %x, %x, %x \n",
						*(ts_buffer+ *ts_head),
						*(ts_buffer+ *ts_head +1),
						*(ts_buffer+ *ts_head +2),
						*(ts_buffer+ *ts_head +3) );

			DPRINTK("exiting - head : %d\n",head);
			*ts_head = head;
		}
	}

	tfftimer_registertimer(&tff_timer, TIME_STEP);
}

void tfftimer_timeover(void)
{
	int ret = 0;
	mm_segment_t oldfs;

	DPRINTK("%s\n",__FUNCTION__);

	ret=queue_work(tdmb_workqueue, &tdmb_work_fromfile);
	if(ret==0)
	{
		DPRINTK("failed in queue_work\n");
	}
	else
	{
		DPRINTK("suceeded in queue_work\n");
	}

	//tfftimer_registertimer(&tff_timer, TIME_STEP);
}

void tfftimer_registertimer(struct timer_list *timer, unsigned long timeover)
{
	DPRINTK("%s\n",__FUNCTION__);
	init_timer(timer);
	timer->expires = get_jiffies_64() + timeover;
	timer->function = tfftimer_timeover;
	add_timer(timer);
}

int tfftimer_init(void)
{
	DPRINTK("%s\n",__FUNCTION__);
	tfftimer_registertimer(&tff_timer, TIME_STEP);

	return 0;
}

void tfftimer_exit(void)
{
	DPRINTK("%s\n",__FUNCTION__);
	del_timer(&tff_timer);
}

void TDMB_drv_FromFile_Open(void)
{
	int ret = 0;
	mm_segment_t oldfs;

	DPRINTK("%s\n",__FUNCTION__);
	DPRINTK(" ##############################\n" );

	// open file
	fp_tff = filp_open(TDMB_FROM_FILE_BIN, O_RDONLY, 0);
	DPRINTK("fp_tff : %x\n", fp_tff);

    //fp_tff->f_pos = 3750600 - (3750600%188 + 3750600/188 - 188*10); // for test of boundary cond.

	// work queue init
	if(!tdmb_workqueue)
	{
		DPRINTK("tdmb_workqueue doesn't exist!\n");
		tdmb_workqueue=create_singlethread_workqueue("ktdmbd");

		if(!tdmb_workqueue)
		{
			DPRINTK("failed to create_workqueue\n");
			return TDMB_ERROR;
		}
	}

	// timer init
	tfftimer_init();
}

#endif /* TDMB_FROM_FILE  */ // ###########################################################

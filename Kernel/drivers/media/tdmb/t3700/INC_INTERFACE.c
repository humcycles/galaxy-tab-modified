#include "INC_INCLUDES.h"

#include "tdmb.h"

#define MSCBuff_Size 1024
#define TS_PACKET_SIZE 188
int bfirst=1;
unsigned char MSCBuff[MSCBuff_Size];
unsigned char TSBuff[INC_INTERRUPT_SIZE * 2];
int TSBuffpos = 0;
int MSCBuffpos = 0;
int mp2len = 0;
static const int bitRateTable[2][16] = { {0, 32, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 384, 0},  /* MPEG1 for id=1*/ 
                                                           {0,  8,  16, 24, 32, 40, 48,  56,   64,   80,  96,  112, 128, 144, 160, 0} };/* MPEG2 for id=0 */

/************************************************************************/
/* Operating Chip set : T3700                                           */
/* Software version   : version 1.10                                    */
/* Software Update    : 2009.01.08                                      */
/************************************************************************/

ST_SUBCH_INFO		g_stDmbInfo;
ST_SUBCH_INFO		g_stDabInfo;
ST_SUBCH_INFO		g_stDataInfo;
ST_SUBCH_INFO		g_stFIDCInfo;
ENSEMBLE_BAND 		m_ucRfBand 		= KOREA_BAND_ENABLE;


INC_CHANNEL_INFO    g_currChInfo; //tdmb
INC_UINT8           g_IsFactoryMode = 0;

/*********************************************************************************/
/*  RF Band Select						                                         */
/*																				 */
/*  INC_UINT8 m_ucRfBand = KOREA_BAND_ENABLE,									 */
/*						   BANDIII_ENABLE,										 */
/*						   LBAND_ENABLE,										 */
/*						   CHINA_ENABLE,										 */
/*						   EXTERNAL_ENABLE,										 */
/*********************************************************************************/
CTRL_MODE 			m_ucCommandMode 	= INC_SPI_CTRL;
ST_TRANSMISSION		m_ucTransMode		= TRANSMISSION_MODE1;
UPLOAD_MODE_INFO	m_ucUploadMode 		= STREAM_UPLOAD_SPI;
CLOCK_SPEED			m_ucClockSpeed 		= INC_OUTPUT_CLOCK_4096;
INC_ACTIVE_MODE		m_ucMPI_CS_Active 	= INC_ACTIVE_LOW;
INC_ACTIVE_MODE		m_ucMPI_CLK_Active 	= INC_ACTIVE_HIGH;
INC_UINT16			m_unIntCtrl			= (INC_INTERRUPT_POLARITY_HIGH | \
										   INC_INTERRUPT_PULSE | \
										   INC_INTERRUPT_AUTOCLEAR_ENABLE | \
										   (INC_INTERRUPT_PULSE_COUNT & INC_INTERRUPT_PULSE_COUNT_MASK));


/*********************************************************************************/
/* PLL_MODE			m_ucPLL_Mode												 */
/*					T3700  Input Clock Setting  								 */
/*********************************************************************************/
PLL_MODE			m_ucPLL_Mode		= INPUT_CLOCK_24576KHZ;


/*********************************************************************************/
/* INC_DPD_MODE		m_ucDPD_Mode												 */
/*					T3700  Power Saving mode setting							 */
/*********************************************************************************/
INC_DPD_MODE		m_ucDPD_Mode		= INC_DPD_ON;

struct spi_device *spi_dmb;

/*********************************************************************************/
/*  MPI Chip Select and Clock Setup Part                                         */
/*																				 */
/*  INC_UINT8 m_ucCommandMode = INC_I2C_CTRL, INC_SPI_CTRL, INC_EBI_CTRL		 */
/*																				 */
/*  INC_UINT8 m_ucUploadMode = STREAM_UPLOAD_MASTER_SERIAL,						 */
/*							   STREAM_UPLOAD_SLAVE_PARALLEL,					 */
/*							   STREAM_UPLOAD_TS,								 */
/*							   STREAM_UPLOAD_SPI,								 */
/*																				 */
/*  INC_UINT8 m_ucClockSpeed = INC_OUTPUT_CLOCK_4096,							 */
/*							   INC_OUTPUT_CLOCK_2048,							 */
/*							   INC_OUTPUT_CLOCK_1024,							 */
/*********************************************************************************/


void INC_DELAY(INC_UINT16 uiDelay)
{
    msleep(uiDelay);
}

INC_UINT16 INC_I2C_READ(INC_UINT8 ucI2CID, INC_UINT16 uiAddr)
{
    INC_UINT16 uiReadData = 0;
    return uiReadData;
}

INC_UINT8 INC_I2C_WRITE(INC_UINT8 ucI2CID, INC_UINT16 uiAddr, INC_UINT16 uiData)
{
    return INC_SUCCESS ;
}

INC_UINT8 INC_I2C_READ_BURST(INC_UINT8 ucI2CID,  INC_UINT16 uiAddr, INC_UINT8* pData, INC_UINT16 nSize)
{
    return INC_SUCCESS ;
}

INC_UINT8 INC_EBI_WRITE(INC_UINT8 ucI2CID, INC_UINT16 uiAddr, INC_UINT16 uiData)
{
	INC_UINT16 uiCMD = INC_REGISTER_CTRL(SPI_REGWRITE_CMD) | 1;
	INC_UINT16 uiNewAddr = (ucI2CID == TDMB_I2C_ID82) ? (uiAddr | 0x8000) : uiAddr;

	*(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS = uiNewAddr >> 8;
	*(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS = uiNewAddr & 0xff;
	*(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS = uiCMD >> 8;
	*(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS = uiCMD & 0xff;

	*(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS = (uiData >> 8) & 0xff;
	*(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS =  uiData & 0xff;

    return INC_ERROR;
}

INC_UINT16 INC_EBI_READ(INC_UINT8 ucI2CID, INC_UINT16 uiAddr)
{
    INC_UINT16 uiRcvData = 0;
	INC_UINT16 uiCMD = INC_REGISTER_CTRL(SPI_REGREAD_CMD) | 1;
	INC_UINT16 uiNewAddr = (ucI2CID == TDMB_I2C_ID82) ? (uiAddr | 0x8000) : uiAddr;

	*(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS = uiNewAddr >> 8;
	*(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS = uiNewAddr & 0xff;
	*(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS = uiCMD >> 8;
	*(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS = uiCMD & 0xff;
	
	uiRcvData  = (*(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS  & 0xff) << 8;
	uiRcvData |= (*(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS & 0xff);
	
    return uiRcvData;
}

INC_UINT8 INC_EBI_READ_BURST(INC_UINT8 ucI2CID,  INC_UINT16 uiAddr, INC_UINT8* pData, INC_UINT16 nSize)
{
	INC_UINT16 uiLoop, nIndex = 0, anLength[2], uiCMD;
	INC_UINT16 uiNewAddr = (ucI2CID == TDMB_I2C_ID82) ? (uiAddr | 0x8000) : uiAddr;
	
	if(nSize > INC_MPI_MAX_BUFF) return INC_ERROR;
    memset((INC_INT8*)anLength, 0, sizeof(anLength));

	if(nSize > 0xfff) {
		anLength[nIndex++] = 0xfff;
		anLength[nIndex++] = nSize - 0xfff;
    }
	else anLength[nIndex++] = nSize;

	for(uiLoop = 0; uiLoop < nIndex; uiLoop++){

		uiCMD = INC_REGISTER_CTRL(SPI_MEMREAD_CMD) | (anLength[nIndex] & 0xFFF);
		
		*(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS = uiNewAddr >> 8;
		*(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS = uiNewAddr & 0xff;
		*(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS = uiCMD >> 8;
		*(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS = uiCMD & 0xff;
		
        for(uiLoop = 0 ; uiLoop < anLength[nIndex]; uiLoop++){
			*pData++ = *(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS & 0xff;
        }
    }

	return INC_SUCCESS;
}

INC_UINT16 INC_SPI_REG_READ(INC_UINT8 ucI2CID, INC_UINT16 uiAddr)
{
    INC_UINT16 uiRcvData = 0;
    INC_UINT8 acRxBuff[2];
	INC_UINT16 uiCMD = INC_REGISTER_CTRL(SPI_REGREAD_CMD) | 1;
    INC_UINT8 auiBuff[4];
    INC_UINT8 cCnt = 0;
	INC_UINT16 uiNewAddr = (ucI2CID == TDMB_I2C_ID82) ? (uiAddr | 0x8000) : uiAddr;

	auiBuff[cCnt++] = uiNewAddr >> 8;
	auiBuff[cCnt++] = uiNewAddr & 0xff;
    auiBuff[cCnt++] = uiCMD >> 8;
    auiBuff[cCnt++] = uiCMD & 0xff;

	struct spi_message              msg;
	struct spi_transfer             transfer[2];
	unsigned char                   status;
	
	memset( &msg, 0, sizeof( msg ) );
	memset( transfer, 0, sizeof( transfer ) );
	spi_message_init( &msg );

	msg.spi=spi_dmb;

	transfer[0].tx_buf = (u8 *) auiBuff;
	transfer[0].rx_buf = (u8 *) NULL;
	transfer[0].len = 4;
	transfer[0].bits_per_word = 8;
	transfer[0].delay_usecs = 0;
	spi_message_add_tail( &(transfer[0]), &msg );

	transfer[1].tx_buf = (u8 *) NULL;
	transfer[1].rx_buf = (u8 *) acRxBuff;
	transfer[1].len = 2;
	transfer[1].bits_per_word = 8;
	transfer[1].delay_usecs = 0;
	spi_message_add_tail( &(transfer[1]), &msg );

	status = spi_sync(spi_dmb, &msg);
	uiRcvData = (INC_UINT16)(acRxBuff[0] << 8)|(INC_UINT16)acRxBuff[1];

    //TODO SPI Read code here...
    return uiRcvData;
}

INC_UINT8 INC_SPI_REG_WRITE(INC_UINT8 ucI2CID, INC_UINT16 uiAddr, INC_UINT16 uiData)
{
	INC_UINT16 uiCMD = INC_REGISTER_CTRL(SPI_REGWRITE_CMD) | 1;
    INC_UINT8 auiBuff[6];
    INC_UINT8 cCnt = 0;
	INC_UINT16 uiNewAddr = (ucI2CID == TDMB_I2C_ID82) ? (uiAddr | 0x8000) : uiAddr;
	struct spi_message              msg;
	struct spi_transfer             transfer;
	unsigned char                   status;

	auiBuff[cCnt++] = uiNewAddr >> 8;
	auiBuff[cCnt++] = uiNewAddr & 0xff;
    auiBuff[cCnt++] = uiCMD >> 8;
    auiBuff[cCnt++] = uiCMD & 0xff;
    auiBuff[cCnt++] = uiData >> 8;
    auiBuff[cCnt++] = uiData & 0xff;

	memset( &msg, 0, sizeof( msg ) );
	memset( &transfer, 0, sizeof( transfer ) );
	spi_message_init( &msg );

	msg.spi=spi_dmb;

	transfer.tx_buf = (u8 *) auiBuff;
	transfer.rx_buf = NULL;
	transfer.len = 6;
	transfer.bits_per_word = 8;
	transfer.delay_usecs = 0;
	spi_message_add_tail( &transfer, &msg );

	status = spi_sync(spi_dmb, &msg);
	
    //TODO SPI SDO Send code here...

    return INC_SUCCESS;
}

INC_UINT8 INC_SPI_READ_BURST(INC_UINT8 ucI2CID, INC_UINT16 uiAddr, INC_UINT8* pBuff, INC_UINT16 wSize)
{
	INC_UINT16 uiLoop, nIndex = 0,  uiCMD;
    INC_UINT8 auiBuff[4];
	INC_UINT16 uiNewAddr = (ucI2CID == TDMB_I2C_ID82) ? (uiAddr | 0x8000) : uiAddr;

	struct spi_message              msg;
	struct spi_transfer             transfer[2];
	unsigned char                   status;
	

	auiBuff[0] = uiNewAddr >> 8;
	auiBuff[1] = uiNewAddr & 0xff;
	uiCMD = INC_REGISTER_CTRL(SPI_MEMREAD_CMD) | (wSize & 0xFFF);
    auiBuff[2] = uiCMD >> 8;
    auiBuff[3] = uiCMD & 0xff;
	
   	memset( &msg, 0, sizeof( msg ) );
	memset( transfer, 0, sizeof( transfer ) );
	spi_message_init( &msg );

	msg.spi=spi_dmb;

	transfer[0].tx_buf = (u8 *) auiBuff;
	transfer[0].rx_buf = (u8 *)NULL;
	transfer[0].len = 4;
	transfer[0].bits_per_word = 8;
	transfer[0].delay_usecs = 0;
	spi_message_add_tail( &(transfer[0]), &msg );
	
	transfer[1].tx_buf = (u8 *) NULL;
	transfer[1].rx_buf = (u8 *)pBuff;
	transfer[1].len = wSize;
	transfer[1].bits_per_word = 8;
	transfer[1].delay_usecs = 0;
	spi_message_add_tail( &(transfer[1]), &msg );
	status = spi_sync(spi_dmb, &msg);

    return INC_SUCCESS;
}

INC_UINT8 INTERFACE_DBINIT(void)
{
	memset(&g_stDmbInfo,	0, sizeof(ST_SUBCH_INFO));
	memset(&g_stDabInfo,	0, sizeof(ST_SUBCH_INFO));
	memset(&g_stDataInfo,	0, sizeof(ST_SUBCH_INFO));
	memset(&g_stFIDCInfo,	0, sizeof(ST_SUBCH_INFO));
	return INC_SUCCESS;
}

// 초기 전원 입력시 호출
INC_UINT8 INTERFACE_INIT(INC_UINT8 ucI2CID)
{
    return INC_INIT(ucI2CID);
}

// 에러 발생시 에러코드 읽기
INC_ERROR_INFO INTERFACE_ERROR_STATUS(INC_UINT8 ucI2CID)
{
    ST_BBPINFO* pInfo;
    pInfo = INC_GET_STRINFO(ucI2CID);
    return pInfo->nBbpStatus;
}

/*********************************************************************************/
/* 단일 채널 선택하여 시작하기....                                               */  
/* pChInfo->ucServiceType, pChInfo->ucSubChID, pChInfo->ulRFFreq 는              */
/* 반드시 넘겨주어야 한다.                                                       */
/* DMB채널 선택시 pChInfo->ucServiceType = 0x18                                  */
/* DAB, DATA채널 선택시 pChInfo->ucServiceType = 0으로 설정을 해야함.            */
/*********************************************************************************/
INC_UINT8 INTERFACE_START(INC_UINT8 ucI2CID, ST_SUBCH_INFO* pChInfo)
{
    g_currChInfo = pChInfo->astSubChInfo[0]; // for dab
    bfirst = 1;
    TSBuffpos = 0;
    MSCBuffpos = 0;
    mp2len = 0;
#ifdef INC_RS_DEC_ENABLE
		INC_RS_DECODER_INIT();
#endif
    g_IsFactoryMode = 0;
	return INC_CHANNEL_START(ucI2CID, pChInfo, SIMPLE_FIC_ENABLE);
}

INC_UINT8 INTERFACE_START_TEST(INC_UINT8 ucI2CID, ST_SUBCH_INFO* pChInfo)
{
    g_currChInfo = pChInfo->astSubChInfo[0]; // for dab
    bfirst = 1;
    TSBuffpos = 0;
    MSCBuffpos = 0;
    mp2len = 0;
#ifdef INC_RS_DEC_ENABLE
		INC_RS_DECODER_INIT();
#endif
    g_IsFactoryMode = 1;
	return INC_CHANNEL_START(ucI2CID, pChInfo, SIMPLE_FIC_DISABLE);
}
/*********************************************************************************/
/* 스캔시  호출한다.                                                             */
/* 주파수 값은 받드시넘겨주어야 한다.                                            */
/* Band를 변경하여 스캔시는 m_ucRfBand를 변경하여야 한다.                        */
/* 주파수 값은 받드시넘겨주어야 한다.                                            */
/*********************************************************************************/
INC_UINT8 INTERFACE_SCAN(INC_UINT8 ucI2CID, INC_UINT32 ulFreq)
{
	INTERFACE_DBINIT();
	if(INC_ENSEMBLE_SCAN(ucI2CID, ulFreq) == INC_ERROR) return INC_ERROR;

	INC_DB_UPDATE(ulFreq, &g_stDmbInfo, &g_stDabInfo, &g_stDataInfo, &g_stFIDCInfo);
	
    return INC_SUCCESS;
}

/*********************************************************************************/
/* 단일채널 스캔이 완료되면 DMB채널 개수를 리턴한다.                             */
/*********************************************************************************/
INC_UINT16 INTERFACE_GETDMB_CNT(void)
{
    return (INC_UINT16)g_stDmbInfo.nSetCnt;
}

/*********************************************************************************/
/* 단일채널 스캔이 완료되면 DAB채널 개수를 리턴한다.                             */
/*********************************************************************************/
INC_UINT16 INTERFACE_GETDAB_CNT(void)
{
    return (INC_UINT16)g_stDabInfo.nSetCnt;
}

/*********************************************************************************/
/* 단일채널 스캔이 완료되면 DATA채널 개수를 리턴한다.                            */
/*********************************************************************************/
INC_UINT16 INTERFACE_GETDATA_CNT(void)
{
    return (INC_UINT16)g_stDataInfo.nSetCnt;
}

/*********************************************************************************/
/* 단일채널 스캔이 완료되면 FIDC채널 개수를 리턴한다.                            */
/*********************************************************************************/
INC_UINT16 INTERFACE_GETFIDC_CNT(void)
{
    return (INC_UINT16)g_stFIDCInfo.nSetCnt;
}

/*********************************************************************************/
/* 단일채널 스캔이 완료되면 Ensemble lable을 리턴한다.                           */
/*********************************************************************************/
INC_UINT8* INTERFACE_GETENSEMBLE_LABEL(INC_UINT8 ucI2CID)
{
	ST_FICDB_LIST*		pList;
	pList = INC_GET_FICDB_LIST();
	return pList->aucEnsembleName;
}

/*********************************************************************************/
/* DMB 채널 정보를 리턴한다.                                                     */
/*********************************************************************************/
INC_CHANNEL_INFO* INTERFACE_GETDB_DMB(INC_INT16 uiPos)
{
    if(uiPos >= MAX_SUBCH_SIZE) return INC_NULL;
    if(uiPos >= g_stDmbInfo.nSetCnt) return INC_NULL;
    return &g_stDmbInfo.astSubChInfo[uiPos];
}

/*********************************************************************************/
/* DAB 채널 정보를 리턴한다.                                                     */
/*********************************************************************************/
INC_CHANNEL_INFO* INTERFACE_GETDB_DAB(INC_INT16 uiPos)
{
    if(uiPos >= MAX_SUBCH_SIZE) return INC_NULL;
    if(uiPos >= g_stDabInfo.nSetCnt) return INC_NULL;
    return &g_stDabInfo.astSubChInfo[uiPos];
}

/*********************************************************************************/
/* DATA 채널 정보를 리턴한다.                                                    */
/*********************************************************************************/
INC_CHANNEL_INFO* INTERFACE_GETDB_DATA(INC_INT16 uiPos)
{
    if(uiPos >= MAX_SUBCH_SIZE) return INC_NULL;
    if(uiPos >= g_stDataInfo.nSetCnt) return INC_NULL;
    return &g_stDataInfo.astSubChInfo[uiPos];
}

/*********************************************************************************/
/* FIDC 채널 정보를 리턴한다.                                                    */
/*********************************************************************************/
INC_CHANNEL_INFO* INTERFACE_GETDB_FIDC(INC_INT16 uiPos)
{
    if(uiPos >= MAX_SUBCH_SIZE) return INC_NULL;
    if(uiPos >= g_stFIDCInfo.nSetCnt) return INC_NULL;
    return &g_stFIDCInfo.astSubChInfo[uiPos];
}


// 시청 중 FIC 정보 변경되었는지를 체크
INC_UINT8 INTERFACE_RECONFIG(INC_UINT8 ucI2CID)
{
	return INC_FIC_RECONFIGURATION_HW_CHECK(ucI2CID);
}

// 앙상블 이 정상적으로 호출되고 난 후 500ms마다 호출..
INC_UINT8 INTERFACE_STATUS_CHECK(INC_UINT8 ucI2CID)
{
    return INC_STATUS_CHECK(ucI2CID);
}

INC_UINT16 INTERFACE_GET_CER(INC_UINT8 ucI2CID)
{
    return INC_GET_CER(ucI2CID);
}

INC_UINT8 INTERFACE_GET_SNR(INC_UINT8 ucI2CID)
{
    return INC_GET_SNR(ucI2CID);
}

INC_DOUBLE32 INTERFACE_GET_POSTBER(INC_UINT8 ucI2CID)
{
    return INC_GET_POSTBER(ucI2CID);
}

INC_DOUBLE32 INTERFACE_GET_PREBER(INC_UINT8 ucI2CID)
{
    return INC_GET_PREBER(ucI2CID);
}

// 스캔 및 채널전환시 강제 중지시 호출...
void INTERFACE_USER_STOP(INC_UINT8 ucI2CID)
{
    ST_BBPINFO* pInfo;
    pInfo = INC_GET_STRINFO(ucI2CID);
    pInfo->ucStop = 1;
}

// 인터럽트 인에이블...
void INTERFACE_INT_ENABLE(INC_UINT8 ucI2CID, INC_UINT16 unSet)
{
	INC_SET_INTERRUPT(ucI2CID, unSet);
}


// 인터럽스 클리어
void INTERFACE_INT_CLEAR(INC_UINT8 ucI2CID, INC_UINT16 unClr)
{
	INC_CLEAR_INTERRUPT(ucI2CID, unClr);
}



// 인터럽트 서비스 루틴... // SPI Slave Mode or MPI Slave Mode
INC_UINT8 INTERFACE_ISR(INC_UINT8 ucI2CID, INC_UINT8* pBuff)
{
	INC_UINT16 unLoop;
	
	if(m_ucUploadMode == STREAM_UPLOAD_SPI){
		INC_SPI_READ_BURST(ucI2CID, APB_STREAM_BASE, pBuff, INC_INTERRUPT_SIZE);
	}
	if(m_ucUploadMode == STREAM_UPLOAD_SLAVE_PARALLEL)
	{
		for(unLoop = 0; unLoop < INC_INTERRUPT_SIZE; unLoop++){
			pBuff[unLoop] = *(volatile INC_UINT8*)STREAM_PARALLEL_ADDRESS & 0xff;
		}
	}

	if((m_unIntCtrl & INC_INTERRUPT_LEVEL) && (!(m_unIntCtrl & INC_INTERRUPT_AUTOCLEAR_ENABLE)))
		INTERFACE_INT_CLEAR(ucI2CID, INC_MPI_INTERRUPT_ENABLE);

	
	return INC_SUCCESS;
}


INC_UINT8  g_acStreamBuff[INC_INTERRUPT_SIZE+188];
extern int TDMB_AddDataToRing(unsigned char* pData, unsigned long dwDataSize);
extern unsigned char _tdmb_make_result(unsigned char byCmd, unsigned short byDataLength, unsigned char* pbyData);
int __AddDataMSC(unsigned char* pData, unsigned long dwDataSize, int SubChID);
int __AddDataTS(unsigned char* pData, unsigned long dwDataSize);
void tdmb_work_function(void)
{
    INC_UINT16 ulRemainLength = INC_INTERRUPT_SIZE;
    INC_UINT16 unIndex = 0;
    INC_UINT16 unSPISize= 0xFFF;
    
    memset(g_acStreamBuff, 0, sizeof(g_acStreamBuff));

    while(ulRemainLength)
    {
        if(ulRemainLength >= unSPISize){

            INC_SPI_READ_BURST(0x80, APB_STREAM_BASE, &g_acStreamBuff[unIndex*unSPISize], unSPISize);
            unIndex++;
            ulRemainLength -= unSPISize;
            
        }
        else
        {
            INC_SPI_READ_BURST(0x80, APB_STREAM_BASE, &g_acStreamBuff[unIndex*unSPISize], ulRemainLength);
            ulRemainLength = 0;
        }
    }

    if ( g_currChInfo.ucServiceType == 0x18 )
    {
     //TDMB_AddDataToRing(g_acStreamBuff, INC_INTERRUPT_SIZE);
        __AddDataTS(g_acStreamBuff, INC_INTERRUPT_SIZE);
    }
    else
    {
        INC_UINT16 i;
        INC_UINT16 maxi = INC_INTERRUPT_SIZE/188;
        unsigned char* pData = (unsigned char*)g_acStreamBuff;
        for ( i = 0 ; i < maxi ; i++ )
        {
            __AddDataMSC(pData, 188, g_currChInfo.ucSubChID);
            pData += 188;
        }
    }

    return;
}

static int tdmbspi_probe(struct spi_device *spi)
{
	//	struct tdmb_spi *test;

	spi_dmb = spi;

	printk("tdmbspi_probe() \n");

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	spi_setup(spi);

	return 0;
}


static int __devexit tdmbspi_remove(struct spi_device *spi)
{
	return 0;
}

static struct spi_driver tdmbspi_driver = {
	.driver = {
		.name	= "tdmbspi",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= tdmbspi_probe,
	.remove		= __devexit_p(tdmbspi_remove),
};

int tdmbspi_init(void)
{
	printk("Register SPI driver for tdmb\n");

	return spi_register_driver(&tdmbspi_driver);
}

void tdmbspi_exit(void)
{
	spi_unregister_driver(&tdmbspi_driver);
}

void spi_open()
{
	printk("spi_open()!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
}

void spi_close()
{
	printk("spi_close()!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
}

#ifdef USER_APPLICATION_TYPE
static int __Tdmb_SetVisualRadioInfo(SubChInfoType* subChInfo)
{
    int i = 0, j = 0;

    subChInfo->bVisualRadio = 0;
    
    /* UA Type - TV, Visual Radio : 0x009 */
    for(i = 0; i < subChInfo->NumberofUserAppl; i++)
    {
        if(subChInfo->UserApplType[i] == 0x009)
        {
            for(j = 0; j < subChInfo->UserApplLength[i]; j++)
            {
                /* Video Service Object Profile ID - Profile1(H.264, ER-BSAC) : 0x01, Profile2(H.264, HE-AAC v2) : 0x02 */
                /* Main Object Type ID - TV : 0x01, Visual Radio : 0x02 */
            
                if((subChInfo->UserApplData[j][0] == 0x01)
                    && (subChInfo->UserApplData[j][1] == 0x02))
                {
                    subChInfo->bVisualRadio = 1;
                    return 1;
                }
            }
        }
    }
    return 0;
}
#endif

int UpdateEnsembleInfo(EnsembleInfoType* ensembleInfo, unsigned long freq)
{
    int i, j, k, l;
    int nSubChIdx = 0;
    int nCnt;
    INC_CHANNEL_INFO* pINC_SubChInfo;
    const char * ensembleName = NULL;

    if (INTERFACE_GETDMB_CNT() + INTERFACE_GETDAB_CNT() > 0)
    {
        ensembleInfo->TotalSubChNumber = 0;
        ensembleName = (char *)INTERFACE_GETENSEMBLE_LABEL(TDMB_I2C_ID80); //TDMB_T3700

        if (ensembleName)
        {
            strncpy((char *)ensembleInfo->EnsembleLabelCharField, (char *)ensembleName, ENSEMBLE_LABEL_SIZE_MAX);
        }
        ensembleInfo->EnsembleFrequency = freq;

        for ( i=0 ; i<2 ; i++ )
        {
            nCnt = (i==0)?INTERFACE_GETDMB_CNT():INTERFACE_GETDAB_CNT();

            for ( j=0 ; j<nCnt ; j++, nSubChIdx++ )
            {
                pINC_SubChInfo = (i==0)?INTERFACE_GETDB_DMB(j):INTERFACE_GETDB_DAB(j);
                ensembleInfo->SubChInfo[nSubChIdx].SubChID      = pINC_SubChInfo->ucSubChID;
                ensembleInfo->SubChInfo[nSubChIdx].StartAddress = pINC_SubChInfo->uiStarAddr;
                ensembleInfo->SubChInfo[nSubChIdx].TMId         = pINC_SubChInfo->uiTmID;
                ensembleInfo->SubChInfo[nSubChIdx].Type         = pINC_SubChInfo->ucServiceType;
                ensembleInfo->SubChInfo[nSubChIdx].ServiceID    = pINC_SubChInfo->ulServiceID;
                memcpy(ensembleInfo->SubChInfo[nSubChIdx].ServiceLabel, pINC_SubChInfo->aucLabel, SERVICE_LABEL_SIZE_MAX);

#ifdef USER_APPLICATION_TYPE
                ensembleInfo->SubChInfo[nSubChIdx].NumberofUserAppl = pINC_SubChInfo->stUsrApp.ucUAppCount;
               
                for(k = 0; k < ensembleInfo->SubChInfo[nSubChIdx].NumberofUserAppl; k++)
                {
                    ensembleInfo->SubChInfo[nSubChIdx].UserApplType[k]   = pINC_SubChInfo->stUsrApp.astUserApp[k].unUserAppType;
                    ensembleInfo->SubChInfo[nSubChIdx].UserApplLength[k] = pINC_SubChInfo->stUsrApp.astUserApp[k].ucDataLength;
                    
                    for(l = 0; l < ensembleInfo->SubChInfo[nSubChIdx].UserApplLength[k]; l++)
                    {
                        ensembleInfo->SubChInfo[nSubChIdx].UserApplData[k][l] = pINC_SubChInfo->stUsrApp.astUserApp[k].aucData[l]; 
                    }
                }
                __Tdmb_SetVisualRadioInfo(&ensembleInfo->SubChInfo[nSubChIdx]);   
#endif                
            }
        }
    }

    ensembleInfo->TotalSubChNumber = nSubChIdx;

    return nSubChIdx;
}

int __AddDataTS(unsigned char* pData, unsigned long dwDataSize)
{
    int j = 0;
    int maxi = 0;
    if(bfirst)
    {
        printk("!!!!!!!!!!!!! first sync dwDataSize = %ld !!!!!!!!!!!!\n", dwDataSize);

        for(j=0;j<dwDataSize;j++)
        {
            if(pData[j]==0x47)
            {                
                printk("!!!!!!!!!!!!! first sync j = %d !!!!!!!!!!!!\n",j);
                _tdmb_make_result(DMB_TS_PACKET_RESYNC, sizeof(int), &j);
                maxi = (dwDataSize - j) / TS_PACKET_SIZE;
                TSBuffpos = (dwDataSize - j) % TS_PACKET_SIZE;
                TDMB_AddDataToRing(&pData[j], maxi * TS_PACKET_SIZE);
                if(TSBuffpos > 0)
                    memcpy(TSBuff, &pData[j + maxi * TS_PACKET_SIZE], TSBuffpos);
                bfirst = 0;
                return 0;
            }
        }
    }
    else
    {
        maxi = (dwDataSize) / TS_PACKET_SIZE;

        if(TSBuffpos > 0)
        {
            if(pData[TS_PACKET_SIZE - TSBuffpos] != 0x47)
            {
                printk("!!!!!!!!!!!!! error 0x%x,0x%x!!!!!!!!!!!!\n",pData[TS_PACKET_SIZE - TSBuffpos], pData[TS_PACKET_SIZE - TSBuffpos + 1]);                    
                memset(TSBuff, 0, INC_INTERRUPT_SIZE * 2);
                TSBuffpos =0;
                bfirst =1;
                return -1;
            }

            memcpy(&TSBuff[TSBuffpos], pData, TS_PACKET_SIZE-TSBuffpos);
            TDMB_AddDataToRing(TSBuff, TS_PACKET_SIZE);
            TDMB_AddDataToRing(&pData[TS_PACKET_SIZE - TSBuffpos], dwDataSize - TS_PACKET_SIZE);
            memcpy(TSBuff, &pData[dwDataSize-TSBuffpos], TSBuffpos);
        }
        else
        {
            if(pData[0] != 0x47)
            {
                printk("!!!!!!!!!!!!! error 0x%x,0x%x!!!!!!!!!!!!\n",pData[0], pData[1]);                    
                memset(TSBuff, 0, INC_INTERRUPT_SIZE * 2);
                TSBuffpos =0;
                bfirst =1;
                return -1;
            }

            TDMB_AddDataToRing(pData, dwDataSize);
        }
    }
    return 0;
}                       
int MP2_GetPacketLength(unsigned char *pkt)
{
    int id, layer_index, bitrate_index, fs_index, samplerate,protection;
    int bitrate, length;
                                              
    id = (pkt[1]>>3) &0x01; /* 1: ISO/IEC 11172-3, 0:ISO/IEC 13818-3 */
    layer_index = (pkt[1]>>1)&0x03; /* 2 */
    protection = pkt[1]&0x1;
    if (protection != 0)
    {
        //QTV_F("protection_bit is *NOT* 0");
    }
    bitrate_index = (pkt[2]>>4);
    fs_index = (pkt[2]>>2)&0x3; /* 1 */

    if(pkt[0]==0xff && (pkt[1]>>4)==0xf) /* sync word check */
    {
        if ( (bitrate_index > 0 && bitrate_index < 15) && (layer_index==2) && (fs_index ==1) )
        {
            if (id ==1 && layer_index ==2) /* Fs==48 KHz*/
            {
                bitrate=1000*bitRateTable[0][bitrate_index];
                samplerate=48000;
            }
            else if (id==0 && layer_index ==2) /* Fs=24 KHz */
            {
                bitrate=1000*bitRateTable[1][bitrate_index];
                samplerate=24000;
            }
            else
                return -1;
        }
        else
            return -1;
    }
    else
        return -1;

    if ( (pkt[2]&0x02)!=0) /* padding bit */
    {
        return -1;            
    }
    
    length = (144*bitrate)/(samplerate);
    
    return length;
}

int __AddDataMSC(unsigned char* pData, unsigned long dwDataSize, int SubChID)
{
    int j,readpos =0;
    unsigned char pOutAddr[188];
    static int first=1;
    int remainbyte = 0;
    if(bfirst)
    {
        for(j=0;j<dwDataSize-4;j++)
        {
            if(pData[j]==0xFF && (pData[j+1]>>4==0xF))
            {                
                mp2len = MP2_GetPacketLength(&pData[j]);
                printk("!!!!!!!!!!!!! first sync mp2len= %d !!!!!!!!!!!!\n",mp2len);
                if(mp2len <=0 || mp2len >MSCBuff_Size )
                    return -1;
                memcpy(MSCBuff, &pData[j], dwDataSize-j);
                MSCBuffpos = dwDataSize-j;
                bfirst = 0;
                first =1;
                return 0;
            }
        }
    }
    else
    {
        if(mp2len <=0 || mp2len >MSCBuff_Size )
        {
            MSCBuffpos =0;
            bfirst =1;
            return -1;
        }   

        remainbyte = dwDataSize;
        if(mp2len-MSCBuffpos>=dwDataSize)
        {
            memcpy(MSCBuff+MSCBuffpos, pData,  dwDataSize);
            MSCBuffpos += dwDataSize;
            remainbyte = 0;
        }
        else if(mp2len-MSCBuffpos>0)
        {
            memcpy(MSCBuff+MSCBuffpos, pData,  mp2len-MSCBuffpos);
            remainbyte = dwDataSize - (mp2len -MSCBuffpos);
            MSCBuffpos = mp2len;
        }

        if(MSCBuffpos==mp2len)
        {
            while(MSCBuffpos>readpos)
            {
                //memset(pOutAddr, 0, 188);
                if(first)
                {
                    pOutAddr[0]=0xDF;
                    pOutAddr[1]=0xDF;
                    pOutAddr[2]= (SubChID<<2);
                    pOutAddr[2] |=(((MSCBuffpos>>3)>>8)&0x03);
                    pOutAddr[3] = (MSCBuffpos>>3)&0xFF;
                    if(!(MSCBuff[0]==0xFF && (MSCBuff[1]>>4==0xF)))
                    {
                        printk("!!!!!!!!!!!!! error 0x%x,0x%x!!!!!!!!!!!!\n",MSCBuff[0], MSCBuff[1]);                    
                        memset(MSCBuff, 0, MSCBuff_Size);
                        MSCBuffpos =0;
                        bfirst =1;
                        return -1;
                    }
                    memcpy(pOutAddr+4, MSCBuff, 184);
                    readpos = 184;
                    first =0;
                }
                else
                {
                    pOutAddr[0]=0xDF;
                    pOutAddr[1]=0xD0;
                    if(MSCBuffpos-readpos>=184)
                    {
                        memcpy(pOutAddr+4, MSCBuff+readpos, 184);
                        readpos +=184;
                    }
                    else
                    {
                        memcpy(pOutAddr+4, MSCBuff+readpos, MSCBuffpos-readpos);
                        readpos +=(MSCBuffpos-readpos);
                    }

                }
                TDMB_AddDataToRing(pOutAddr, 188);
            }
            first =1;
            MSCBuffpos =0;
            if(remainbyte>0)
            {
                memcpy(MSCBuff, pData+dwDataSize-remainbyte, remainbyte);
                MSCBuffpos = remainbyte;                
            }
        }
        else if(MSCBuffpos > mp2len)
        {
            printk("!!!!!!!!!!!!!Error MSCBuffpos=%d, mp2len =%d!!!!!!!!!!!!\n",MSCBuffpos, mp2len);                    
            memset(MSCBuff, 0, MSCBuff_Size);
            MSCBuffpos =0;
            bfirst =1;
            return -1;
        }
    }
    return 0;
}



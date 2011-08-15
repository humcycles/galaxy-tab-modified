#include "INC_INCLUDES.h"

#ifdef INC_FIFO_SOURCE_ENABLE

#define INC_GET_SUBCHANNEL_SIZE(X, Y) (((((X)<<8) | (Y)) & 0x3FF) * 2)
#define INC_GET_SUBCHANNEL_ID(X) (((X) >> 2) & 0x3F)
ST_FIFO g_astChFifo[MAX_CHANNEL_FIFO];
INC_UINT8 g_acCheckBuff[INC_HEADER_CHECK_BUFF];

INC_UINT8 INC_QFIFO_INIT(PST_FIFO pFF, INC_UINT32 ulDepth)
{
	if(pFF == INC_NULL) return INC_ERROR;

	pFF->ulFront = pFF->ulRear = 0;

	if(ulDepth == 0 || ulDepth >= INC_FIFO_DEPTH) pFF->ulDepth = INC_FIFO_DEPTH + 1;
	else pFF->ulDepth = ulDepth + 1;
	return INC_SUCCESS;
}

INC_UINT32 INC_QFIFO_FREE_SIZE(PST_FIFO pFF)
{
	if(pFF == INC_NULL) return INC_ERROR;

	return (pFF->ulFront >= pFF->ulRear) ?
		((pFF->ulRear + pFF->ulDepth) - pFF->ulFront) - 1 : (pFF->ulRear - pFF->ulFront) - 1;
}

INC_UINT32 INC_QFIFO_GET_SIZE(PST_FIFO pFF)
{
	if(pFF == INC_NULL) return INC_ERROR;

	return (pFF->ulFront >= pFF->ulRear) ?
		(pFF->ulFront - pFF->ulRear) : (pFF->ulFront + pFF->ulDepth - pFF->ulRear);
}

INC_UINT8 INC_QFIFO_AT(PST_FIFO pFF, INC_UINT8* pData, INC_UINT32 ulSize)
{
	INC_UINT32 ulLoop, ulOldRear;

	if(pFF == INC_NULL || pData == INC_NULL || ulSize > INC_QFIFO_GET_SIZE(pFF)) 
		return INC_ERROR;

	ulOldRear = pFF->ulRear;
	for(ulLoop = 0 ; ulLoop < ulSize; ulLoop++)	{
		pData[ulLoop] = pFF->acBuff[ulOldRear++];
		ulOldRear %= pFF->ulDepth;
	}
	return INC_SUCCESS;
}

INC_UINT8 INC_QFIFO_ADD(PST_FIFO pFF, INC_UINT8* pData, INC_UINT32 ulSize)
{
	INC_UINT32 ulLoop;

	if(pFF == INC_NULL || pData == INC_NULL || ulSize > INC_QFIFO_FREE_SIZE(pFF)) 
		return INC_ERROR;

	for(ulLoop = 0 ; ulLoop < ulSize; ulLoop++)	{
		pFF->acBuff[pFF->ulFront++] = pData[ulLoop];
		pFF->ulFront %= pFF->ulDepth;
	}
	return INC_SUCCESS;
}

INC_UINT8 INC_QFIFO_BRING(PST_FIFO pFF, INC_UINT8* pData, INC_UINT32 ulSize)
{
	INC_UINT32 ulLoop;

	if(pFF == INC_NULL || pData == INC_NULL || ulSize > INC_QFIFO_GET_SIZE(pFF)) 
		return INC_ERROR;

	for(ulLoop = 0 ; ulLoop < ulSize; ulLoop++)	{
		pData[ulLoop] = pFF->acBuff[pFF->ulRear++];
		pFF->ulRear %= pFF->ulDepth;
	}
	return INC_SUCCESS;
}

void INC_MULTI_SORT_INIT(void)
{
	INC_UINT32 ulLoop;
	ST_FIFO* pFifo;

	for(ulLoop = 0 ; ulLoop < MAX_CHANNEL_FIFO; ulLoop++){
		pFifo = INC_GET_CHANNEL_FIFO((MULTI_CHANNEL_INFO)ulLoop);
		INC_QFIFO_INIT(pFifo, 0);
		pFifo->unSubChID = INC_SUB_CHANNEL_ID_MASK;
	}
}

ST_FIFO* INC_GET_CHANNEL_FIFO(MULTI_CHANNEL_INFO ucIndex)
{
	if(ucIndex >= MAX_CHANNEL_FIFO) return INC_NULL;
	return &g_astChFifo[ucIndex];
}

ST_HEADER_INFO INC_HEADER_CHECK(ST_FIFO* pMainFifo)
{
	INC_UINT32 ulLoop,  ulFrame, ulIndex, ulTotalLength, ulSubChTSize;
	INC_UINT16 aunChSize[MAX_CHANNEL_FIFO-1];

	ulFrame = INC_QFIFO_GET_SIZE(pMainFifo) / MAX_HEADER_SIZE;

	for(ulIndex = 0; ulIndex < (ulFrame-1); ulIndex++){

		INC_QFIFO_AT(pMainFifo, g_acCheckBuff, MAX_HEADER_SIZE);
		for(ulLoop = 0 ; ulLoop < (MAX_HEADER_SIZE-1); ulLoop++)
		{
			if(g_acCheckBuff[ulLoop] == HEADER_ID_0x33 && g_acCheckBuff[ulLoop+1] == HEADER_ID_0x00)
			{
				if(ulLoop) INC_QFIFO_BRING(pMainFifo, g_acCheckBuff, ulLoop);

				INC_QFIFO_AT(pMainFifo, g_acCheckBuff, MAX_HEADER_SIZE);

				ulTotalLength = (INC_UINT16)(g_acCheckBuff[4] << 8) | g_acCheckBuff[5];
				ulTotalLength = (ulTotalLength & 0x8000) ? (ulTotalLength << 1) : ulTotalLength;

				aunChSize[0] = INC_GET_SUBCHANNEL_SIZE(g_acCheckBuff[0x8], g_acCheckBuff[0x9]);
				aunChSize[1] = INC_GET_SUBCHANNEL_SIZE(g_acCheckBuff[0xA], g_acCheckBuff[0xB]);
				aunChSize[2] = INC_GET_SUBCHANNEL_SIZE(g_acCheckBuff[0xC], g_acCheckBuff[0xD]);
				aunChSize[3] = INC_GET_SUBCHANNEL_SIZE(g_acCheckBuff[0xE], g_acCheckBuff[0xF]);

				ulSubChTSize = aunChSize[0] + aunChSize[1] + aunChSize[2] + aunChSize[3] + MAX_HEADER_SIZE;

				if(ulSubChTSize != ulTotalLength) {
					INC_QFIFO_INIT(pMainFifo, 0);
					return INC_HEADER_NOT_SEARACH;
				}

				if(INC_QFIFO_GET_SIZE(pMainFifo) < (ulTotalLength + MAX_HEADER_SIZE))
					return INC_HEADER_SIZE_ERROR;

				INC_QFIFO_AT(pMainFifo, g_acCheckBuff, ulTotalLength + MAX_HEADER_SIZE);

				if(g_acCheckBuff[ulTotalLength] == HEADER_ID_0x33 && g_acCheckBuff[ulTotalLength+1] == HEADER_ID_0x00)
				{
					return INC_HEADER_GOOD;
				}
				else{
					INC_QFIFO_INIT(pMainFifo, 0);
					return INC_HEADER_NOT_SEARACH;
				}
			}
		}

		if(g_acCheckBuff[ulLoop] == HEADER_ID_0x33) INC_QFIFO_BRING(pMainFifo, g_acCheckBuff, MAX_HEADER_SIZE-1);
		else INC_QFIFO_BRING(pMainFifo, g_acCheckBuff, MAX_HEADER_SIZE);
	}

	return INC_HEADER_NOT_SEARACH;
}

INC_UINT8 INC_MULTI_FIFO_PROCESS(INC_UINT8* pData, INC_UINT32 ulSize)
{
	INC_UINT8  cIndex, bIsData = INC_ERROR;
	INC_UINT8 acBuff[INC_CIF_MAX_SIZE];

	INC_UINT16 aunChSize[MAX_CHANNEL_FIFO-1], unTotalSize;
	INC_UINT16 aunSubChID[MAX_CHANNEL_FIFO-1];

	ST_FIFO* pFifo;
	ST_FIFO* pMainFifo;

	pMainFifo = INC_GET_CHANNEL_FIFO(MAIN_INPUT_DATA);
	if(INC_QFIFO_ADD(pMainFifo, pData, ulSize) != INC_SUCCESS)
	{
		INC_QFIFO_INIT(pMainFifo, 0);
		return INC_ERROR;
	}

	while(1){

		if(INC_QFIFO_GET_SIZE(pMainFifo) < MAX_HEADER_SIZE) return bIsData;
		if(INC_HEADER_CHECK(pMainFifo) != INC_HEADER_GOOD){
			return bIsData;
		}

		INC_QFIFO_AT(pMainFifo, acBuff, MAX_HEADER_SIZE);

		unTotalSize = (INC_UINT16)(acBuff[4] << 8) | acBuff[5];
		unTotalSize = (unTotalSize & 0x8000) ? (unTotalSize << 1) : unTotalSize;

		if(unTotalSize > INC_QFIFO_GET_SIZE(pMainFifo))
			return bIsData;

		//각각의 채널별 크기를 구하고...
		memset(aunChSize, 0 , sizeof(aunChSize));

		aunChSize[0] = INC_GET_SUBCHANNEL_SIZE(acBuff[0x8], acBuff[0x9]);
		aunChSize[1] = INC_GET_SUBCHANNEL_SIZE(acBuff[0xA], acBuff[0xB]);
		aunChSize[2] = INC_GET_SUBCHANNEL_SIZE(acBuff[0xC], acBuff[0xD]);
		aunChSize[3] = INC_GET_SUBCHANNEL_SIZE(acBuff[0xE], acBuff[0xF]);

		aunSubChID[0] = INC_GET_SUBCHANNEL_ID(acBuff[0x8]);
		aunSubChID[1] = INC_GET_SUBCHANNEL_ID(acBuff[0xA]);
		aunSubChID[2] = INC_GET_SUBCHANNEL_ID(acBuff[0xC]);
		aunSubChID[3] = INC_GET_SUBCHANNEL_ID(acBuff[0xE]);

		INC_QFIFO_BRING(pMainFifo, acBuff, MAX_HEADER_SIZE);

		for(cIndex = 0; cIndex < (MAX_CHANNEL_FIFO-1); cIndex++){
			if(!aunChSize[cIndex]) continue;

			pFifo = INC_GET_CHANNEL_FIFO((MULTI_CHANNEL_INFO)(cIndex+1));

			if(pFifo->unSubChID == INC_SUB_CHANNEL_ID_MASK)
				pFifo->unSubChID = aunSubChID[cIndex];

			while(aunChSize[cIndex] > INC_CIF_MAX_SIZE)
			{
				INC_QFIFO_BRING(pMainFifo, acBuff, (INC_UINT32)INC_CIF_MAX_SIZE);
				INC_QFIFO_ADD(pFifo, acBuff, (INC_UINT32)INC_CIF_MAX_SIZE);
				aunChSize[cIndex] -= INC_CIF_MAX_SIZE;
			}

			INC_QFIFO_BRING(pMainFifo, acBuff, (INC_UINT32)aunChSize[cIndex]);
			INC_QFIFO_ADD(pFifo, acBuff, (INC_UINT32)aunChSize[cIndex]);
		}

		bIsData = INC_SUCCESS;
	}

	return INC_SUCCESS;
}




#endif


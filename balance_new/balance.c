/*************************************************************************************************************
 * @file    balance.c
 * @biref   main and initial task of balance operation
 * @details when charge end, mark the cells that need to be balanced, then run balance cmd in discharge mode
 * @par     history
 * <table>
 * <tr><th>Author    <th>Date        <th>Version   <th>ChangeLogs  </tr>
 * <tr><td>shenxd    <td>2019-04-08  <td>V1.0      <td>First issue </tr>
 * <tr><td>shenxd    <td>2020-09-02  <td>V1.1      <td>
 * <tr><td>fd        <td>2021-08-05  <td>V1.2      <td>rebuild     </tr>
 * -# remove many dependencies of pro_confgi.h
 * -# use const varibles to replace macro define
 * -# add bmu count to the maximum: 15
 * </table>
 **************************************************************************************************************/

/**************************************************************************************************************
 *                    Include
 * ************************************************************************************************************/
#include "comm_def.h"
#include "Global_define.h"
#include "can_driver.h"
#include "fault_record.h"
#include "nvm_mgr.h"
#include "bmu.h"
#include "soc.h"
#include "current.h"
#include "key_state.h"
#include "data_record.h"
#include "lib.h"
#include "nvm_mgr.h"
#include "DCCharge_Management.h"
#include "app_chg.h"
#include "soh.h"
#include "cell_balance.h"
/***************************************************************
 *                  Macro declaration
 * *************************************************************/
#define     BALANCE_ON                   (1u)
#define     BALANCE_OFF                  (0u)
#define     BALANCE_VOLT_DIFF            (150u)     /* mV */


#define     BALANCE_BYTE_COUNT           (6u)                    /*balance byte number in message*/
#define     BALANCE_CHECK_TIME           (10u)
#define     BALANCE_FAIL_COMFIR_COUNT    (5u)                    /*confirm time for balance fault*/
#define     MAX_CONTINUETIME             (30u)                   /*Unit: min, scale: 1u*/
#define     BALANCE_CONTINUE_AS          ((INT32U)PRO_CELL_CAPACITY_DESIGN * RteGetTotalSohValue() * 6u * 6u / 10000u)  /*SOC 0.1% CAP*/
#if PRO_CELL_CAPACITY_DESIGN > 790
#error "BALANCE_CONTINUE_AS overflow!!"
#endif
#define     BALANCE_MINUS(A,B)           ((A) > (B) ? (A) - (B) : 0u)

#define     BALANCE_CONTINUE_PRECESION   (5u)                    /*Unit: Amin; 100mA *  60s * 80% */
#define     BALANCE_SET_GAP              (1u)                    /*period of calcuate balance flag unit : s*/
#define     BALANCE_MINUTE               (60u)
#define     BALANCE_TIME_MAX             (30u * 60u)            /*Max balancing time 20h*/
#define     BALANCE_CHGTIME_MAX          (8u * 60u)             /*Max balancing time of charging mode*/
#define     BALANCE_SOC_MAX              ((INT16U)((INT32U)BALANCE_CONTINUE_PRECESION * BALANCE_TIME_MAX / BALANCE_CONTINUE_AS) & 0xFFFFu)  /*Unit: 0.1%SOC*/
#define     BALANCE_SOC_MAX_CHG          ((INT16U)((INT32U)BALANCE_CONTINUE_PRECESION * BALANCE_CHGTIME_MAX / BALANCE_CONTINUE_AS) & 0xFFFFu)  /*Unit: 0.1%SOC*/

#define     BALANCE_BYTE                 ((PRO_CELL_VOLT_SERIAL * PRO_CELL_PARELL_NUM + 7u) / 8u)

#define     BALANCE_CHGVOLTLVL0          (3450u)               /*Unit: mV*/
#define     BALANCE_CHGVOLTLVL1          (3500u)               /*Unit: mV*/
#define     BALANCE_STOPMINVOLT          (3200u)               /*Unit: mV*/
#define     BALANCE_STOPMAXVOLT          (3600u)               /*Unit: mV*/

#define     BALANCE_CHGVOLTPERSOCLVL1    (6u)                  /*Unit: mV/0.1%SOC @ 3500mV*/
#define     BALANCE_CHGVOLTPERSOCLVL2    (10u)                 /*Unit: mV/0.1%SOC @ 3650mV*/

#define     BALANCE_CHGDELTVOLT0         (20u)                 /*Unit: mV*/
#define     BALANCE_CHGDELTVOLT1         (40u)                 /*Unit: mV*/
#define     BALANCE_CHGDELTVOLT2         (140u)                /*Unit: mV*/
#define     BALANCE_SOC_NORMAL           (4u)                  /*Unit: %, scale:0.1*/

#define     BALANCE_LOWSOC_NORMAL        (15u)                  /*Unit: %, scale:0.1*/

#define     BALANCE_DYNAMIC_CURR         ((INT16U)((INT32U)PRO_CELL_CAPACITY_DESIGN * RteGetTotalSohValue() / 300u))      /*1/3C, Unit: A, scale: 0.1*/
#define     BALANCE_DYNAMIC_TEMPDIFF     (15u)
#define     BALANCE_DYNAMIC_TEMPMIN      ((TEMPERATURE_OFFSET + 20u) * TEMPERATURE_FACTOR)

#define     BALANCE_LOWSOC_TEMPDIFF      (10u)
#define     BALANCE_LOWSOC_TEMPMIN       ((TEMPERATURE_OFFSET - 5u) * TEMPERATURE_FACTOR)


typedef enum
{
    DCH = 0,
    CHG
} Direction;

typedef struct
{
    INT8U uTempDif;
    INT8U uThreshold;
}TEMPDIF_Thread;

typedef struct
{
    INT16U uBalanceTimeMin     : 15u;
    INT16U uSOCBalanceSetFlag  : 1u;
}BALANCENVM_Data;

/***************************************************************
 *                 Varibels declaration
 * *************************************************************/

#pragma DATA_SEG CALRAM_BAL
static  volatile INT8U  gucarrBalanceFlag[PRO_BMU_COUNT_MAX][BALANCE_BYTE_COUNT];
#pragma DATA_SEG DEFAULT

#pragma DATA_SEG CALRAM_BALTIME
static BALANCENVM_Data gstContinueTime;
#pragma DATA_SEG DEFAULT

#pragma DATA_SEG __GPAGE_SEG PAGED_RAM
static INT8U guContinueBalanceFlag;	 		/*indicate in continue balance mode*/
static INT8U gucBalancestatus;       		/*indicate balance status*/
static INT16U guBalanceCellNum;
static INT8U  gucBalancePreCondMet;
static INT8U gucarrBalanceFBStatus[BALANCE_BYTE];
static INT16U guBalancingSOC;              /*Unit: %, scale : 0.1*/
#pragma DATA_SEG DEFAULT 

static INT16U guBalancePrecisionTimerS;

static INT8U gparrBmuBalanceData[PRO_BMU_COUNT_MAX][BALANCE_BYTE_COUNT];
static INT8U gucBalanceOffVCANCmd;

static INT8U gucBalancingType;             /*0u: No balancing, 1: SOC-Amend balancing; 2:Static Voltage balancing; 3:Charge finished balancing;4:chargind dynamic balancing;5:discharge dynamic balancing*/
/***************************************************************
 *          Global function declaration
 * *************************************************************/
INT8U RteGetBalanceActiveSt(void);
void AlgInitBalance(void);
void AlgTaskBalance(void);
void ApiClearBalanceFlag(void);
INT16U RteGetRemainContinueTime(void);
void ApiSetChgBalanceFlag(void);
void ApiSetLowSOCBalanceFlag(INT8U ucSOCAmemdFlag);
INT16U RteGetBalanceCellNum(void);
INT8U RteGetBalancePreCondMet(void);
INT8U RteGetCellBalanceByteStatus(INT8U ucByteIndex);
INT8U RteGetCellBalanceCmd(INT8U ucBmuId, INT8U ucByte);
/***************************************************************
 *          Local function declaration
 * *************************************************************/
static void sendBalanceCmd(INT8U);
static void checkBalanceErrStatus(void);
static void checkBalanceFbStatus(void);
static void normalBalance(Direction);
static void continueBalance(void);
static INT16U getThreshold(Direction);
static INT8U checkVoltVaild(void);

/***************************************************************
 *          constant declaration
 * *************************************************************/

const TEMPDIF_Thread ChgThreshold[] =
{
      { 0u,20u},
      { 5u,25u},
      {10u,30u},
      {15u,35u},
      {20u,40u},
      {30u,45u},
};

const TEMPDIF_Thread DchThreshold[] =
{
      { 0u,25u},
      { 5u,30u},
      {10u,35u},
      {15u,40u},
      {20u,45u},
      {30u,50u},
};

/**************************************************
 * @brief  Intial function of balance module, must call once when initalize the project
 * @@param none
 * @retrun none
 *************************************************/
void AlgInitBalance(void)
{
    INT8U ucBmuId;
    INT8U ucBalanceByte;
    
	sendBalanceCmd(0u);
	(void)Stb_S_SetTimer(&guBalancePrecisionTimerS);

	guContinueBalanceFlag = BALANCE_OFF;
	guBalanceCellNum = 0xFFFFu;
	gucBalancingType = 0u;

	if(gstContinueTime.uBalanceTimeMin > BALANCE_TIME_MAX)
	{
		gstContinueTime.uBalanceTimeMin = 0u;
		gstContinueTime.uSOCBalanceSetFlag = 0u;
	}
    else
    {
        /*not in balance do nothing*/
    }

    /*check if need to operate balance*/
    for(ucBmuId = 0u ; ucBmuId < glbucSysBmuCount ; ucBmuId += 1u)
    {
        for(ucBalanceByte = 0u ; ucBalanceByte < BALANCE_BYTE_COUNT ; ucBalanceByte += 1u)
        {
            if(gstContinueTime.uBalanceTimeMin)
            {
				if(gucarrBalanceFlag[ucBmuId][ucBalanceByte])
				{
					guContinueBalanceFlag = BALANCE_ON;
					sendBalanceCmd(1u);
					break;
				}
				else
				{
					/*do nothing*/
				}
            }
			else
			{
				sendBalanceCmd(0u);
			}
        }

        if(BALANCE_ON == guContinueBalanceFlag)
        {
            break;
        }
        else
        {
            /*do nothing*/
        }
    }

    if(BALANCE_OFF == guContinueBalanceFlag)
    {
        gstContinueTime.uBalanceTimeMin = 0u;		
		gstContinueTime.uSOCBalanceSetFlag = 0u;
    }
    else
    {
        /*continue balance*/
    }
	gucBalancePreCondMet = 0u;
	gucBalanceOffVCANCmd = 0u;

}

/**************************************************
 * @brief       fulfil the balance command output buffer
 * @@param[in]
 * -# 1 send balance cmd by the calculation
 * -# 0 send balance cmd all zeros
 * @retrun none
 *************************************************/
static void sendBalanceCmd(INT8U ucCmd)
{
    INT8U ucBmuId;
    INT8U ucBalanceByte;

    if(ucCmd)
    {
        for(ucBmuId = 0u ; ucBmuId < glbucSysBmuCount ; ucBmuId += 1u)
        {
            for(ucBalanceByte = 0u ; ucBalanceByte < BALANCE_BYTE_COUNT ; ucBalanceByte += 1u)
            {
                gparrBmuBalanceData[ucBmuId][ucBalanceByte] = gucarrBalanceFlag[ucBmuId][ucBalanceByte];				
            }			
			(void)EcuPutCellBalanceCmd(ucBmuId, gparrBmuBalanceData[ucBmuId]);
        }
    }
    else
    {
        for(ucBmuId = 0u ; ucBmuId < glbucSysBmuCount ; ucBmuId += 1u)
        {
            for(ucBalanceByte = 0u ; ucBalanceByte < BALANCE_BYTE_COUNT ; ucBalanceByte += 1u)
            {
                 gparrBmuBalanceData[ucBmuId][ucBalanceByte] = 0u;
            }			
			(void)EcuPutCellBalanceCmd(ucBmuId, gparrBmuBalanceData[ucBmuId]);
        }		
		gucBalancingType = 0u;
    }
}

/****************************************
 * @brief: check balance error status
 * @return: none
 * **************************************/
static void checkBalanceFbStatus(void)
{
    INT8U ucBmuId;
	INT8U ucCellIdx;
	INT8U ucCellNo;
	INT8U ucBalancedCellNo;

	ucCellNo = 0u;	
	ucBalancedCellNo = 0u;
    for(ucBmuId = 0u ; ucBmuId < glbucSysBmuCount; ucBmuId += 1u)
    {		
		for(ucCellIdx = 0u ; ucCellIdx < glbucSysBmuCellConfig[ucBmuId]; ucCellIdx += 1u)
		{
			if((ucCellIdx >> 3u) >= BALANCE_BYTE_COUNT)
			{
				break;
			}
			else
			{
				if(ApiGetBalanceSwitchStatus(ucBmuId)->szBalance[ucCellIdx >> 3u] & (1u << (INT8U)(ucCellIdx & 0x7u)))
				{
					gucarrBalanceFBStatus[ucCellNo >> 3u] |= (((INT8U)BALANCE_ON) << (INT8U)(ucCellNo & 0x7u));
					ucBalancedCellNo += 1u;
				}
				else
				{
					gucarrBalanceFBStatus[ucCellNo >> 3u] &= (INT8U)(~(((INT8U)BALANCE_ON) << (INT8U)(ucCellNo & 0x7u)));
				}
				ucCellNo += 1u;
			}
		}
    }
	guBalanceCellNum = ucBalancedCellNo;

	if(ucBalancedCellNo)
	{
		gucBalancestatus = BALANCE_ON;
	}
	else
	{
		gucBalancestatus = BALANCE_OFF;
	}
	
}

/****************************************
 * @brief: check balance error status
 * @return: none
 * **************************************/
static void checkBalanceErrStatus(void)
{
    INT8U ucBmuId;
    INT8U ucBalanceByte;
    INT8U ucRet;
    static INT8U ucCount = 0u;

    ucRet = 0u;

    for(ucBmuId = 0u ; ucBmuId < glbucSysBmuCount; ucBmuId += 1u)
    {

        for(ucBalanceByte = 0u ; ucBalanceByte < BALANCE_BYTE_COUNT; ucBalanceByte += 1u)
        {
            if(ApiGetBalanceErrorStatus(ucBmuId)->szBalance[ucBalanceByte])
            {
               ucRet = 1u;
               break;
            }
            else
            {
                /*do nothing*/
            }
        }

        if(ucRet)
        {
            break;
        }
        else
        {
            /*continue check next bmu*/
        }
    }

    if(ucRet)
    {
        if(ucCount < BALANCE_FAIL_COMFIR_COUNT)
        {
            ucCount += 1u;
        }
        else
        {
            ApiPutBalanceErr(1u);
        }
    }
    else
    {
        ucCount = 0u;
        ApiPutBalanceErr(0u);
    }
}


/**************************************************
 * @brief  check if Threshold Table is valid
 * @@param[in]
 * -# 1 send balance cmd by the calculation
 * -# 0 send balance cmd all zeros
 * @retrun none
 *************************************************/
INT8U CheckThreshold()
{
    INT8U ucAns = TRUE;
    INT8U ucCurIdx;

    for(ucCurIdx = 0;ucCurIdx < sizeof(ChgThreshold) / sizeof(ChgThreshold[0]) - 1u;ucCurIdx += 1u)
    {
        if(ChgThreshold[ucCurIdx].uTempDif >= ChgThreshold[ucCurIdx + 1u].uTempDif)
        {
            ucAns = FALSE;
            break;
        }
        else
        {
            /*keep value*/
        }

        if(ChgThreshold[ucCurIdx].uThreshold >= ChgThreshold[ucCurIdx + 1u].uThreshold)
        {
            ucAns = FALSE;
            break;
        }
        else
        {
            /*keep value*/
        }
    }

    if((TRUE == ucAns) && (sizeof(DchThreshold) / sizeof(DchThreshold[0]) >= 1u))
    {
        for(ucCurIdx = 0;ucCurIdx < sizeof(DchThreshold) / sizeof(DchThreshold[0]) - 1u;ucCurIdx += 1u)
        {
            if(DchThreshold[ucCurIdx].uTempDif >= DchThreshold[ucCurIdx + 1u].uTempDif)
            {
                ucAns = FALSE;
                break;
            }
            else
            {
                /*keep value*/
            }

            if(DchThreshold[ucCurIdx].uThreshold >= DchThreshold[ucCurIdx + 1u].uThreshold)
            {
                ucAns = FALSE;
                break;
            }
            else
            {
                /*keep value*/
            }
        }
    }
    else
    {
        ucAns = FALSE;
    }

    return ucAns;
}


/****************************************
 * @brief:
 * @return: none
 * **************************************/
static void balancingExecute(void)
{	
    INT8U ucBmuId;
    INT8U ucBalanceByte;
	
    if((1u == gucBalanceOffVCANCmd)
        || (RteGetSlaverCommErr())
        || (FALSE == checkVoltVaild())
        || (RteGetBalanceErr())
        || (RteGetFireErr())
        || (0u == RteGetVoltSmpCmplFlag())
        || (0u == RteGetTempSmpCmplFlag())
        || (RteGetBMUOverTmp_Err())
        || (RteGetCellTempSensor_SevErr())
        || (RteGetCellTempAbnormalSv_Err())
        || (RteGetCellTempOpenCircuit_SevErr())
        || (RteGetTotalCellTempMaxMax() > ((TEMPERATURE_OFFSET + 50u) * TEMPERATURE_FACTOR))
        || (RteGetTotalCellTempMinMin() < ((TEMPERATURE_OFFSET - 10u) * TEMPERATURE_FACTOR))                
	    || (RteGetTotalCellVoltMaxMax() > BALANCE_STOPMAXVOLT)
        || (RteGetTotalCellVoltMinMin() < BALANCE_STOPMINVOLT))
    {
        sendBalanceCmd(0u);   /*slave communication lost not working*/
		gucBalancePreCondMet = 0u;
		ApiPutBalCmd(1u);
    }
    else
    {
		gucBalancePreCondMet = 1u;

		/*check if need to operate balance*/
		if(gstContinueTime.uBalanceTimeMin)
        {
			guContinueBalanceFlag = BALANCE_OFF;
			
			for(ucBmuId = 0u ; ucBmuId < glbucSysBmuCount ; ucBmuId += 1u)
			{
				for(ucBalanceByte = 0u ; ucBalanceByte < BALANCE_BYTE_COUNT ; ucBalanceByte += 1u)
				{
					if(gucarrBalanceFlag[ucBmuId][ucBalanceByte])
					{
						guContinueBalanceFlag = BALANCE_ON;
						break;
					}
					else
					{
						/*do nothing*/
					}
				}
			
				if(BALANCE_ON == guContinueBalanceFlag)
				{
					break;
				}
				else
				{
					/*do nothing*/
				}
			}

			if(BALANCE_OFF == guContinueBalanceFlag)
			{
				gstContinueTime.uBalanceTimeMin = 0u;		
				gstContinueTime.uSOCBalanceSetFlag = 0u;
			}
			else
			{
				/*continue balance*/
			}
        }
		else
		{
			guContinueBalanceFlag = BALANCE_OFF;
		}

		if(BALANCE_OFF == guContinueBalanceFlag)
        {
			if(DCCHG_STEP_CHARGING == RteGetFcgStep() || ACCHG_STEP_CHARGING == RteGetACChgStep())
	        {
	            if(CURR_ZERO > RteGetTotalCurr100msMean())
	            {
	                normalBalance(CHG);
	            }
	            else
	            {
	                normalBalance(DCH);
	            }
	        }
	        else
	        {				
				normalBalance(DCH);
	        }
        }
        else
        {
			continueBalance();
        }
		
		ApiPutBalCmd(1u);
    }
}

/****************************************
 * @brief: clear balance flag, then the cmd will send all zeros
 * @return: none
 * **************************************/
void ApiClearBalanceFlag(void)
{
    INT8U ucBmuId;
    INT8U ucBalanceByte;

    for(ucBmuId = 0 ; ucBmuId < glbucSysBmuCount ; ucBmuId += 1u)
    {
        for(ucBalanceByte = 0 ; ucBalanceByte < BALANCE_BYTE_COUNT ; ucBalanceByte += 1u)
        {
            gucarrBalanceFlag[ucBmuId][ucBalanceByte] = BALANCE_OFF;
        }
    }
    gstContinueTime.uBalanceTimeMin = 0;	
	gstContinueTime.uSOCBalanceSetFlag = 0u;
}

/****************************************
 * @brief: main task of balance, called in 100 ms task
 * @return: none
 * **************************************/
void AlgTaskBalance(void)
{
    static INT8U ucRuntime = 0;

	EcuUpdataCellBalanceStatus();

	switch (ucRuntime)
	{
		case 0u:
			checkBalanceErrStatus();
		break;
		case 1u:
			checkBalanceFbStatus();
		break;
		case 2u:
		    balancingExecute();
		break;
		default:
		break;
	}

	ucRuntime += 1u ;
	if(ucRuntime >= BALANCE_CHECK_TIME)
	{
		ucRuntime = 0u;
	}
	else
	{
	 /*keep the loop value*/
	}
}

/**********************************
 * @brief: FindToBeBalancedCellNo, called by Balancing set function
 * @return: none
 ***********************************/
void FindToBeBalancedCellNo(INT16U ucCellVoltDelta)
{
    INT8U ucBmuId;
    INT8U ucBmuLinkId;
    INT8U ucCellIdx;
    INT8U ucPackNum;
    INT16U uIndex;

	for(ucPackNum = 0u; ucPackNum < glbucSysPackParallel; ucPackNum += 1u)
	{				
		uIndex = 0u;
		/*notice: check each branch, but i	is not the bmu id*/
		for(ucBmuId = (glbucSysBmuCount / glbucSysPackParallel)  * ucPackNum ; ucBmuId < (glbucSysBmuCount / glbucSysPackParallel) * (ucPackNum + 1u) ; ucBmuId += 1u)
		{
			/*notice: here not check the index out of range because this has done in selfcheck*/
			ucBmuLinkId = glbucarrBmuIndex[ucBmuId];
		
			for(ucCellIdx = 0u ; ucCellIdx < glbucSysBmuCellConfig[ucBmuLinkId]; ucCellIdx += 1u)
			{
				if((ucCellIdx >> 3u) >= BALANCE_BYTE_COUNT)
				{
					break;
				}
				else
				{
					/*valid continue*/
				}
		
				if(uIndex >= PRO_CELL_VOLT_SERIAL_MAX)
				{
					break;
				}
				else
				{
					/*valid continue*/
				}
		
				if((INT32U)RteGetCellVoltage(ucPackNum, uIndex) > ((INT32U)RteGetTotalCellVoltMinMin() + ucCellVoltDelta))
				{
					gucarrBalanceFlag[ucBmuLinkId][ucCellIdx >> 3u] |= (((INT8U)BALANCE_ON) << (INT8U)(ucCellIdx & 0x7u));
				}
				else
				{
					gucarrBalanceFlag[ucBmuLinkId][ucCellIdx >> 3u] &= (INT8U)(~(((INT8U)BALANCE_ON) << (INT8U)(ucCellIdx & 0x7u)));
				}
		
				uIndex += 1u;
			}
		}
	}
	
}

/**********************************
 * @brief: ApiSetChgBalanceFlag, called by charge management function at the charging stoping phase
 * @return: none
 ***********************************/
void ApiSetChgBalanceFlag(void)
{
	INT16U uBalanceDeltVolt;
	INT8U ucVoltPerMillSOC;

	if((gstContinueTime.uSOCBalanceSetFlag && (gstContinueTime.uBalanceTimeMin >= 120u))
		|| ((RteGetTotalCellTempMinMin() <= BALANCE_DYNAMIC_TEMPMIN) && ((RteGetTotalCellTempDiff() > BALANCE_LOWSOC_TEMPDIFF))))
	{
		/*Don't cover the value when SOC Balancing time has been set*/
	}
	else
	{		
		if(RteGetTotalCellVoltMaxMax() >= BALANCE_CHGVOLTLVL1)
		{
			/*Caclulate delt volt which indicate balancing is needed*/
			if(RteGetTotalCellVoltMaxMax() >= PRO_CHG_CELL_VOLT_MAX)
			{
				uBalanceDeltVolt = BALANCE_CHGDELTVOLT2;
			}
			else
			{
				uBalanceDeltVolt = BALANCE_CHGDELTVOLT1 + (INT16U)((INT32U)(RteGetTotalCellVoltMaxMax() - BALANCE_CHGVOLTLVL1) * (BALANCE_CHGDELTVOLT2 - BALANCE_CHGDELTVOLT1) / (PRO_CHG_CELL_VOLT_MAX - BALANCE_CHGVOLTLVL1) & 0xFFFFu);
			}
			
			/*Check if balancing is needed*/
			if(RteGetTotalCellVoltDiff() > uBalanceDeltVolt)
			{
				/*Calculate volt speed per soc*/
				ucVoltPerMillSOC = BALANCE_CHGVOLTPERSOCLVL1 + (RteGetTotalCellVoltMaxMax() - BALANCE_CHGVOLTLVL1) * 4u / 150u;
				
				/*Calculate balancing SOC that needed without considering balancing time limit*/
				guBalancingSOC = ((RteGetTotalCellVoltDiff() - uBalanceDeltVolt) + ucVoltPerMillSOC / 2) / ucVoltPerMillSOC + BALANCE_SOC_NORMAL;
		
				if(guBalancingSOC > BALANCE_SOC_MAX_CHG)
				{
					guBalancingSOC = BALANCE_SOC_MAX_CHG;			
					gstContinueTime.uBalanceTimeMin = BALANCE_CHGTIME_MAX;
				}
				else if(guBalancingSOC < BALANCE_SOC_NORMAL)
				{
					guBalancingSOC = BALANCE_SOC_NORMAL;
					gstContinueTime.uBalanceTimeMin = (INT16U)(BALANCE_CONTINUE_AS * guBalancingSOC / BALANCE_CONTINUE_PRECESION & 0xFFFFu);
				}
				else
				{
					guBalancingSOC = BALANCE_SOC_NORMAL;
					gstContinueTime.uBalanceTimeMin = (INT16U)(BALANCE_CONTINUE_AS * guBalancingSOC / BALANCE_CONTINUE_PRECESION & 0xFFFFu);				
				}
				
				gstContinueTime.uSOCBalanceSetFlag = 0u;
				
				FindToBeBalancedCellNo(uBalanceDeltVolt);
			}
			else
			{
				guBalancingSOC = 0u;
				ApiClearBalanceFlag();			
			}
		}
		else 
		{
			/*Caclulate delt volt which indicate balancing is needed*/
			if(RteGetTotalCellVoltMaxMax() < BALANCE_CHGVOLTLVL0)
			{
				uBalanceDeltVolt = BALANCE_CHGDELTVOLT0;
			}
			else
			{
				uBalanceDeltVolt = BALANCE_CHGDELTVOLT0 + (INT16U)((INT32U)(RteGetTotalCellVoltMaxMax() - BALANCE_CHGVOLTLVL0) * (BALANCE_CHGDELTVOLT1 - BALANCE_CHGDELTVOLT0) / (BALANCE_CHGVOLTLVL1 - BALANCE_CHGVOLTLVL0) & 0xFFFFu);
			}
			
			/*Check if balancing is needed*/
			if(RteGetTotalCellVoltDiff() > uBalanceDeltVolt)
			{
				FindToBeBalancedCellNo(uBalanceDeltVolt);
				
				guBalancingSOC = BALANCE_SOC_NORMAL;
				gstContinueTime.uBalanceTimeMin = (INT16U)(BALANCE_CONTINUE_AS * guBalancingSOC / BALANCE_CONTINUE_PRECESION & 0xFFFFu);				
				gstContinueTime.uSOCBalanceSetFlag = 0u;
			}
			else
			{
				guBalancingSOC = 0u;
				ApiClearBalanceFlag();			
			}			
		}
	}
}

/*******************************************************
 * ApiSetLowSOCBalanceFlag, called by SOC static amend function when static condition was met
 * *****************************************************/

void ApiSetLowSOCBalanceFlag(INT8U ucSOCAmemdFlag)
{
	INT16U uBalanceDeltVolt;
	INT16U uBalancingSOCCellVolt;
	INT16U uTobeBalancedCellSOC;

	if((RteGetTotalCellTempDiff() < BALANCE_LOWSOC_TEMPDIFF) && (RteGetTotalCellTempMinMin() >= BALANCE_LOWSOC_TEMPMIN))
	{
		if(ucSOCAmemdFlag)
		{
			guBalancingSOC = RteGetTotalSocMax() - RteGetTotalSocMin();

			if(guBalancingSOC < BALANCE_LOWSOC_NORMAL)
			{
				guBalancingSOC = 0u;
			}
			else
			{
				guBalancingSOC = BALANCE_LOWSOC_NORMAL;
			}
			
			uTobeBalancedCellSOC = RteGetTotalSocMin() + guBalancingSOC;

			if(guBalancingSOC)
			{
				if(ucSOCAmemdFlag & 4u) /*OCV Amend*/
				{
					if(RteGetTotalCellTempMinMin() >= 35u + TEMPERATURE_ZERO)
					{
						uBalancingSOCCellVolt = Lookup1D16table(uTobeBalancedCellSOC, AMEND_OCV_FORM_DEMESION, guarrOcvSoc, guarrOcvForm[6]);
					}
					else if(RteGetTotalCellTempMinMin() >= 25u + TEMPERATURE_ZERO)
					{
						uBalancingSOCCellVolt = Lookup1D16table(uTobeBalancedCellSOC, AMEND_OCV_FORM_DEMESION, guarrOcvSoc, guarrOcvForm[5]);
					}
					else if(RteGetTotalCellTempMinMin() >= 10u + TEMPERATURE_ZERO)
					{
						uBalancingSOCCellVolt = Lookup1D16table(uTobeBalancedCellSOC, AMEND_OCV_FORM_DEMESION, guarrOcvSoc, guarrOcvForm[4]);
					}
					else if(RteGetTotalCellTempMinMin() >= 5u + TEMPERATURE_ZERO)
					{
						uBalancingSOCCellVolt = Lookup1D16table(uTobeBalancedCellSOC, AMEND_OCV_FORM_DEMESION, guarrOcvSoc, guarrOcvForm[3]);
					}
					else if(RteGetTotalCellTempMinMin() >= 0u + TEMPERATURE_ZERO)
					{
						uBalancingSOCCellVolt = Lookup1D16table(uTobeBalancedCellSOC, AMEND_OCV_FORM_DEMESION, guarrOcvSoc, guarrOcvForm[2]);
					}
					else if(RteGetTotalCellTempMinMin() >= TEMPERATURE_ZERO - 5u)
					{
						uBalancingSOCCellVolt = Lookup1D16table(uTobeBalancedCellSOC, AMEND_OCV_FORM_DEMESION, guarrOcvSoc, guarrOcvForm[1]);
					}
					else
					{
						uBalancingSOCCellVolt = RteGetTotalCellVoltMinMin();
					}
					
					if(uBalancingSOCCellVolt > RteGetTotalCellVoltMinMin())
					{
						uBalanceDeltVolt =	uBalancingSOCCellVolt - RteGetTotalCellVoltMinMin();
					}
					else
					{
						uBalanceDeltVolt =	0u;
					}
				}
				else if(ucSOCAmemdFlag & 2u) /*5 min quasistatic amend*/
				{
					if(RteGetTotalCellTempMinMin() >= 45u + TEMPERATURE_ZERO)
					{
						uBalancingSOCCellVolt = Lookup1D16table(uTobeBalancedCellSOC, AMEND_OCV_FORM_DEMESION, guarrQuasiStaticOcvSoc, guarrQuasiStatic5MinOcvForm[0]);
					}
					else if(RteGetTotalCellTempMinMin() >= 25u + TEMPERATURE_ZERO)
					{
						uBalancingSOCCellVolt = Lookup1D16table(uTobeBalancedCellSOC, AMEND_OCV_FORM_DEMESION, guarrQuasiStaticOcvSoc, guarrQuasiStatic5MinOcvForm[1]);
					}
					else if(RteGetTotalCellTempMinMin() >= 10u + TEMPERATURE_ZERO)
					{
						uBalancingSOCCellVolt = Lookup1D16table(uTobeBalancedCellSOC, AMEND_OCV_FORM_DEMESION, guarrQuasiStaticOcvSoc, guarrQuasiStatic5MinOcvForm[2]);
					}
					else if(RteGetTotalCellTempMinMin() >= 0u + TEMPERATURE_ZERO)
					{
						uBalancingSOCCellVolt = Lookup1D16table(uTobeBalancedCellSOC, AMEND_OCV_FORM_DEMESION, guarrQuasiStaticOcvSoc, guarrQuasiStatic5MinOcvForm[3]);
					}
					else
					{
						uBalancingSOCCellVolt = RteGetTotalCellVoltMinMin();
					}
					
					if(uBalancingSOCCellVolt > RteGetTotalCellVoltMinMin())
					{
						uBalanceDeltVolt =	uBalancingSOCCellVolt - RteGetTotalCellVoltMinMin();
					}
					else
					{
						uBalanceDeltVolt =	0u;
					}
				}
				else if(ucSOCAmemdFlag & 1u)  /*1 min quasistatic amend*/
				{
					if(RteGetTotalCellTempMinMin() >= 45u + TEMPERATURE_ZERO)
					{
						uBalancingSOCCellVolt = Lookup1D16table(uTobeBalancedCellSOC, AMEND_OCV_FORM_DEMESION, guarrQuasiStaticOcvSoc, guarrQuasiStatic1MinOcvForm[0]);
					}
					else if(RteGetTotalCellTempMinMin() >= 25u + TEMPERATURE_ZERO)
					{
						uBalancingSOCCellVolt = Lookup1D16table(uTobeBalancedCellSOC, AMEND_OCV_FORM_DEMESION, guarrQuasiStaticOcvSoc, guarrQuasiStatic1MinOcvForm[1]);
					}
					else if(RteGetTotalCellTempMinMin() >= 10u + TEMPERATURE_ZERO)
					{
						uBalancingSOCCellVolt = Lookup1D16table(uTobeBalancedCellSOC, AMEND_OCV_FORM_DEMESION, guarrQuasiStaticOcvSoc, guarrQuasiStatic1MinOcvForm[2]);
					}
					else if(RteGetTotalCellTempMinMin() >= 0u + TEMPERATURE_ZERO)
					{
						uBalancingSOCCellVolt = Lookup1D16table(uTobeBalancedCellSOC, AMEND_OCV_FORM_DEMESION, guarrQuasiStaticOcvSoc, guarrQuasiStatic1MinOcvForm[3]);
					}
					else
					{
						uBalancingSOCCellVolt = RteGetTotalCellVoltMinMin();
					}
					
					if(uBalancingSOCCellVolt > RteGetTotalCellVoltMinMin())
					{
						uBalanceDeltVolt =	uBalancingSOCCellVolt - RteGetTotalCellVoltMinMin();
					}
					else
					{
						uBalanceDeltVolt =	0u;
					}
				}
				else
				{
					/*Wrong amend*/
				}
				
				if(uBalanceDeltVolt)
				{
					if(guBalancingSOC >= BALANCE_SOC_MAX)
					{
						gstContinueTime.uBalanceTimeMin = BALANCE_TIME_MAX;				
					}
					else
					{
						gstContinueTime.uBalanceTimeMin = (INT16U)((BALANCE_CONTINUE_AS * guBalancingSOC + BALANCE_CONTINUE_PRECESION / 2) / BALANCE_CONTINUE_PRECESION & 0xFFFFu);				
					}
					gstContinueTime.uSOCBalanceSetFlag = 1u;
					
					FindToBeBalancedCellNo(uBalanceDeltVolt);	
				}
				else
				{
					
				}
			}
			else
			{
				/*Cell Uniformity is good, no need to start balance, No need to */
				guBalancingSOC = 0u;
				ApiClearBalanceFlag();			
			}
		}
		else /*When SOC not amended*/
		{
			if(gstContinueTime.uBalanceTimeMin >= 120u)
			{
				/*Don't reset the balance time*/
			}
			else
			{
				if(RteGetTotalCellTempMinMin() > BALANCE_DYNAMIC_TEMPMIN)
				{
					uBalanceDeltVolt = 10u;
				}
				else
				{
					uBalanceDeltVolt = 10u + RteGetTotalCellTempDiff();
				}
				
				if(RteGetTotalCellVoltDiff() > uBalanceDeltVolt)
				{					
					gstContinueTime.uBalanceTimeMin = BALANCE_CHGTIME_MAX;	
					gstContinueTime.uSOCBalanceSetFlag = 0u;
					FindToBeBalancedCellNo(uBalanceDeltVolt);	
				}
				else
				{
					/*Cell Uniformity is good, no need to start balance, No need to */
				}
			}
		}
	}
	else
	{
		/*Don't calculate the balancing time*/
	}
}

/*******************************************************
 * continue operation balance
 * *****************************************************/
static void continueBalance(void)
{

    if(BALANCE_ON == guContinueBalanceFlag)
    {
        if(0u == gstContinueTime.uBalanceTimeMin)
        {
            guContinueBalanceFlag = BALANCE_OFF;
			ApiClearBalanceFlag();
            sendBalanceCmd(0u);
        }
        else
        {
            if(Stb_S_Timeout(guBalancePrecisionTimerS,BALANCE_MINUTE))
            {
                gstContinueTime.uBalanceTimeMin -= 1u;
                (void)Stb_S_SetTimer(&guBalancePrecisionTimerS);
            }
            else
            {
                /*do nothing*/
            }
            sendBalanceCmd(1u);

			if(gstContinueTime.uSOCBalanceSetFlag)
			{
				gucBalancingType = 1u;
			}
			else
			{
				gucBalancingType = 3u;
			}
        }
    }
    else
    {

    }

}

/*************************************************************
 *balance operation
 *************************************************************/
static void normalBalance(Direction input)
{
    INT8U ucBmuId;
    INT8U ucBmuLinkId;
    INT8U ucCellIdx;
    INT8U ucPackNum;
    INT16U uIndex;
	INT8U ucBalancingOn;

	ucBalancingOn = 0u;
	
	if((RteGetTotalCellTempMinMin() > BALANCE_DYNAMIC_TEMPMIN)
        || (RteGetTotalCellTempDiff() < BALANCE_DYNAMIC_TEMPDIFF))
	{
		for(ucPackNum = 0u; ucPackNum < glbucSysPackParallel; ucPackNum += 1u)
		{
			if(DCH == input)
			{
				if((RteGetStmCurrMean(ucPackNum) > CURR_ZERO + BALANCE_DYNAMIC_CURR)
					|| (RteGetStmCurrMean(ucPackNum) < CURR_ZERO))
				{					
					sendBalanceCmd(0u);
					continue;
				}
				else
				{

				}
			}
			else
			{
				if(RteGetStmCurrMean(ucPackNum) < CURR_ZERO - BALANCE_DYNAMIC_CURR)
				{					
					sendBalanceCmd(0u);
					continue;
				}
				else
				{

				}
			}
		
			uIndex = 0u;
		
			/*notice: check each branch, but i	is not the bmu id*/
			for(ucBmuId = (glbucSysBmuCount / glbucSysPackParallel)  * ucPackNum ; ucBmuId < (glbucSysBmuCount / glbucSysPackParallel) * (ucPackNum + 1u) ; ucBmuId += 1u)
			{
		
				/*notice: here not check the index out of range because this has done in selfcheck*/
				ucBmuLinkId = glbucarrBmuIndex[ucBmuId];
		
				for(ucCellIdx = 0u ; ucCellIdx < glbucSysBmuCellConfig[ucBmuLinkId]; ucCellIdx += 1u)
				{
					if((ucCellIdx >> 3u) >= BALANCE_BYTE_COUNT)
					{
						break;
					}
					else
					{
						/*valid continue*/
					}
		
					if(uIndex >= PRO_CELL_VOLT_SERIAL_MAX)
					{
						break;
					}
					else
					{
						/*valid continue*/
					}
		
					if((INT32U)RteGetCellVoltage(ucPackNum, uIndex) > (((INT32U)RteGetCellVoltMin(ucPackNum,0u) + getThreshold(input))))
					{
						gparrBmuBalanceData[ucBmuLinkId][ucCellIdx >> 3u] |= (((INT8U)BALANCE_ON) << (INT8U)(ucCellIdx & 0x7u));
						ucBalancingOn = 1u;
					}
					else
					{
						gparrBmuBalanceData[ucBmuLinkId][ucCellIdx >> 3u] &= (INT8U)(~(((INT8U)BALANCE_ON) << (INT8U)(ucCellIdx & 0x7u)));
					}
		
					uIndex += 1u;
				}
				
				(void)EcuPutCellBalanceCmd(ucBmuId, gparrBmuBalanceData[ucBmuId]);
			}
		}

		if(ucBalancingOn)
		{
			if(DCH == input)
			{
				gucBalancingType = 5u;
			}
			else
			{
				gucBalancingType = 4u;
			}
		}
		else
		{
			gucBalancingType = 0u;
		}
    }
	else
	{
		sendBalanceCmd(0u);
	}
	
}


static INT16U getThreshold(Direction input)
{
    INT16U uAns;
    INT8U ucThresholdIdx;
    
    if(DCH == input)
    {
        if(RteGetTotalCellTempDiff() <= DchThreshold[0u].uTempDif)
        {
            uAns = DchThreshold[0u].uThreshold;
        }
        else
        {
            for(ucThresholdIdx = 0; ucThresholdIdx < BALANCE_MINUS(sizeof(DchThreshold) / sizeof(DchThreshold[0]),1u); ucThresholdIdx += 1u)
            {
                if(RteGetTotalCellTempDiff() <= DchThreshold[ucThresholdIdx + 1u].uTempDif)
                {
                    uAns = DchThreshold[ucThresholdIdx].uThreshold + (RteGetTotalCellTempDiff() - DchThreshold[ucThresholdIdx].uTempDif) * (DchThreshold[ucThresholdIdx + 1u].uThreshold - DchThreshold[ucThresholdIdx].uThreshold) / (DchThreshold[ucThresholdIdx + 1u].uTempDif - DchThreshold[ucThresholdIdx].uTempDif);
                    break;
                }
                else
                {
                    /*continue search*/
                }
            }

            if(sizeof(DchThreshold) / sizeof(DchThreshold[0]) == ucThresholdIdx + 1u)
            {
                uAns = DchThreshold[ucThresholdIdx].uThreshold;
            }
            else
            {
                /*keep value*/
            }
        }
    }
    else if(CHG == input)
    {
        if(RteGetTotalCellTempDiff() <= ChgThreshold[0u].uTempDif)
        {
            uAns = ChgThreshold[0u].uThreshold;
        }
        else
        {
            for(ucThresholdIdx = 0; ucThresholdIdx < BALANCE_MINUS(sizeof(ChgThreshold) / sizeof(ChgThreshold[0]),1u); ucThresholdIdx += 1u)
            {
                if(RteGetTotalCellTempDiff() <= ChgThreshold[ucThresholdIdx + 1].uTempDif)
                {
                    uAns = ChgThreshold[ucThresholdIdx].uThreshold + (RteGetTotalCellTempDiff() - ChgThreshold[ucThresholdIdx].uTempDif) * (ChgThreshold[ucThresholdIdx + 1u].uThreshold - ChgThreshold[ucThresholdIdx].uThreshold) / (ChgThreshold[ucThresholdIdx + 1u].uTempDif - ChgThreshold[ucThresholdIdx].uTempDif);
                    break;
                }
                else
                {
                    /*continue search*/
                }
            }

            if(sizeof(ChgThreshold) / sizeof(ChgThreshold[0]) == ucThresholdIdx + 1u)
            {
                uAns = ChgThreshold[ucThresholdIdx].uThreshold;
            }
            else
            {
                /*keep value*/
            }
        }
    }
    else
    {
        uAns = PRO_CELLVOLT_REASONABLE_MAX;                                      /*invail input close all balance flag*/
    }

    return uAns;
}


static INT8U checkVoltVaild(void)
{
    INT8U ucAns;

    ucAns = TRUE;

    if((RteGetCellVoltOpenCircuit_Err())
	    || (RteGetCellVoltSensor_Err())
        || (RteGetTotalCellVoltMinMin() < PRO_CELLVOLT_REASONABLE_MIN)        
	    || (RteGetTotalCellVoltMaxMax() > PRO_CELLVOLT_REASONABLE_MAX))
    {
        ucAns = FALSE;
    }
    else
    {
        /*keep value*/
    }

    return ucAns;
}

/**************************************************
 * @brief: get the balance status
 * @return: if one cell is in balacne , the resuls is true
 * @retval: 1, if true, 0, false
 * ************************************************/
INT8U RteGetBalanceActiveSt(void)
{
    return gucBalancestatus;
}

INT16U RteGetBalanceCellNum(void)
{
	return guBalanceCellNum;
}

INT16U RteGetRemainContinueTime(void)
{
    return gstContinueTime.uBalanceTimeMin;
}

INT8U RteGetBalancePreCondMet(void)
{
    return gucBalancePreCondMet;
}

INT8U RteGetCellBalanceByteStatus(INT8U ucByteIndex)
{
	if(ucByteIndex >= BALANCE_BYTE)
	{
		return 0u;
	}
	else
	{
		return gucarrBalanceFBStatus[ucByteIndex];
	}
}

INT8U RteGetCellBalanceCmd(INT8U ucBmuId, INT8U ucByte)
{
	if(ucBmuId >= glbucSysBmuCount || ucByte >= BALANCE_BYTE_COUNT)
	{
		return 0u;
	}
	else
	{
		return gparrBmuBalanceData[ucBmuId][ucByte];
	}
}

void ApiPutVCANBalanceOffCmd(INT8U ucdata)
{
	gucBalanceOffVCANCmd = ucdata;
}

INT8U RteGetBalancingType(void)
{
	return gucBalancingType;
}


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
#include "CDD_NVM.h"
#include "bmu.h"
#include "soc.h"
#include "CurrDet.h"
#include "Cdd_PwrMgr.h"
#include "data_record.h"
#include "lib.h"
#include "CDD_NVM.h"
#include "DCCharge_Management.h"
#include "app_chg.h"
#include "soh.h"

/***************************************************************
 *                  Macro declaration
 * *************************************************************/
#define     BALANCE_ON                   (1u)
#define     BALANCE_OFF                  (0u)
#define     BALANCE_BYTE_COUNT           (7u)                    /*balance byte number in message*/
#define     BALANCE_CHECK_TIME           (10u)
#define     BALANCE_FAIL_COMFIR_COUNT    (5u)                    /*confirm time for balance fault*/
#define     MAX_CONTINUETIME             (30u)                   /*Unit: min, scale: 1u*/
#define     BALANCE_CONTINUE_AS          ((uint32)PRO_CELL_CAPACITY * RteGetTotalSohValue() * 6u * 6u / 10 / 1000u) /*0.1% capacity, unit: As*/
#if PRO_CELL_CAPACITY > 790
#error "BALANCE_CONTINUE_AS overflow!!"
#endif
#define     BALANCE_CONTINUE_PRECESION   (5u)                    /*Unit: As; 100mA *  60s * 80% */
#define     BALANCE_CONTINUE_SOCCONSUME  (5u)                    /*0.5%*/
#define     BALANCE_SET_GAP              (1u)                    /*period of calcuate balance flag unit : s*/
#define     BALANCE_CONTINUE_MAX         (BALANCE_CONTINUE_AS * BALANCE_CONTINUE_SOCCONSUME / BALANCE_CONTINUE_PRECESION) /*todo*/
#define     BALANCE_MINUTE               (60u)
#define     BALANCE_MINUS(A,B)           ((A) > (B) ? (A) - (B) : 0u)
#define     BALANCE_BMU_CELL_COUNT       (56u)                   /*max 56 cells in one bmu*/


#define     BALANCE_CHGVOLTLVL0          (3450u)                 /*Cell-voltage threshold level 0, unit: mV*/
#define     BALANCE_CHGVOLTLVL1          (3500u)                 /*Cell-voltage threshold level 1, unit: mV*/
#define     BALANCE_CHGVOLTPERSOCLVL1    (6u)                    /*Voltage change per 0.1% SOC at 3500 mV, unit: mV*/
#define     BALANCE_CHGVOLTPERSOCLVL2    (10u)                   /*Voltage change per 0.1% SOC at 3650 mV, unit: mV*/
#define     BALANCE_CHGDELTVOLT0         (20u)                   /*Full-charge balance delta-voltage at level 0, unit: mV*/
#define     BALANCE_CHGDELTVOLT1         (40u)                   /*Full-charge balance delta-voltage at level 1, unit: mV*/
#define     BALANCE_CHGDELTVOLT2         (140u)                  /*Full-charge balance delta-voltage at charge-voltage limit, unit: mV*/
#define     BALANCE_SOC_NORMAL           (4u)                    /*Min full-charge balance SOC estimate, scale: 0.1%*/
#define     BALANCE_DYNAMIC_TEMPMIN      ((TEMPERATURE_OFFSET + 20u) * TEMPERATURE_FACTOR)
#define     BALANCE_LOWSOC_TEMPDIFF      (10u)                   /*Do not overwrite an existing balance setting above this temperature difference*/

typedef enum
{
    DCH = 0,
    CHG
} Direction;

typedef struct
{
    uint8 uTempDif;
    uint8 uThreshold;
}TEMPDIF_Thread;

/***************************************************************
 *                 Varibels declaration
 * *************************************************************/
static volatile uint8 __attribute__ ((section(".CalRAM_BAL"))) gucarrBalanceFlag[PRO_BMU_COUNT_MAX][BALANCE_BYTE_COUNT];
static volatile uint16 __attribute__ ((section(".CalRAM_BALTIME"))) guContinueTime;

static uint8 guContinueBalanceFlag;	 /*indicate in continue balance mode*/
static uint8 gucBalancestatus;       /*indicate balance status*/
static uint16 guBalancePrecisionTimerS;
/***************************************************************
 *          Global function declaration
 * *************************************************************/
uint8 RteGetBalanceActiveSt(void);
void AlgInitBalance(void);
void AlgTaskBalance(void);
void ApiClearBalanceFlag(void);
uint16 RteGetRemainContinueTime(void);
void ApiSetBalanceFlag(void);
void ApiSetForceBalance(void);
void ApiClearForceBalance(void);
uint8 CheckThreshold(void);

/***************************************************************
 *          Local function declaration
 * *************************************************************/
static void sendBalanceCmd(uint8);
static void checkBalanceErrStatus(void);
static void checkBalanceFbStatus(void);
static void normalBalance(Direction);
static void continueBalance(void);
static uint16 getThreshold(Direction);
static uint8 checkVoltVaild(void);

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

static uint8* const gparrBmuBalanceData[PRO_BMU_COUNT_MAX][BALANCE_BYTE_COUNT] =
{
    {
        &(MBCU_BMU01_Balance_Ctrl_Message._c[0]),
        &(MBCU_BMU01_Balance_Ctrl_Message._c[1u]),
        &(MBCU_BMU01_Balance_Ctrl_Message._c[2u]),
        &(MBCU_BMU01_Balance_Ctrl_Message._c[3u]),
        &(MBCU_BMU01_Balance_Ctrl_Message._c[4u]),
        &(MBCU_BMU01_Balance_Ctrl_Message._c[5u]),
        &(MBCU_BMU01_Balance_Ctrl_Message._c[6u]),
    },
    {
        &(MBCU_BMU02_Balance_Ctrl_Message._c[0]),
        &(MBCU_BMU02_Balance_Ctrl_Message._c[1u]),
        &(MBCU_BMU02_Balance_Ctrl_Message._c[2u]),
        &(MBCU_BMU02_Balance_Ctrl_Message._c[3u]),
        &(MBCU_BMU02_Balance_Ctrl_Message._c[4u]),
        &(MBCU_BMU02_Balance_Ctrl_Message._c[5u]),
        &(MBCU_BMU02_Balance_Ctrl_Message._c[6u]),
    },
    {
        &(MBCU_BMU03_Balance_Ctrl_Message._c[0]),
        &(MBCU_BMU03_Balance_Ctrl_Message._c[1u]),
        &(MBCU_BMU03_Balance_Ctrl_Message._c[2u]),
        &(MBCU_BMU03_Balance_Ctrl_Message._c[3u]),
        &(MBCU_BMU03_Balance_Ctrl_Message._c[4u]),
        &(MBCU_BMU03_Balance_Ctrl_Message._c[5u]),
        &(MBCU_BMU03_Balance_Ctrl_Message._c[6u]),
    },
    {
        &(MBCU_BMU04_Balance_Ctrl_Message._c[0]),
        &(MBCU_BMU04_Balance_Ctrl_Message._c[1u]),
        &(MBCU_BMU04_Balance_Ctrl_Message._c[2u]),
        &(MBCU_BMU04_Balance_Ctrl_Message._c[3u]),
        &(MBCU_BMU04_Balance_Ctrl_Message._c[4u]),
        &(MBCU_BMU04_Balance_Ctrl_Message._c[5u]),
        &(MBCU_BMU04_Balance_Ctrl_Message._c[6u]),
    },
    {
        &(MBCU_BMU05_Balance_Ctrl_Message._c[0]),
        &(MBCU_BMU05_Balance_Ctrl_Message._c[1u]),
        &(MBCU_BMU05_Balance_Ctrl_Message._c[2u]),
        &(MBCU_BMU05_Balance_Ctrl_Message._c[3u]),
        &(MBCU_BMU05_Balance_Ctrl_Message._c[4u]),
        &(MBCU_BMU05_Balance_Ctrl_Message._c[5u]),
        &(MBCU_BMU05_Balance_Ctrl_Message._c[6u]),
    },
    {
        &(MBCU_BMU06_Balance_Ctrl_Message._c[0]),
        &(MBCU_BMU06_Balance_Ctrl_Message._c[1u]),
        &(MBCU_BMU06_Balance_Ctrl_Message._c[2u]),
        &(MBCU_BMU06_Balance_Ctrl_Message._c[3u]),
        &(MBCU_BMU06_Balance_Ctrl_Message._c[4u]),
        &(MBCU_BMU06_Balance_Ctrl_Message._c[5u]),
        &(MBCU_BMU06_Balance_Ctrl_Message._c[6u]),
    },
    {
        &(MBCU_BMU07_Balance_Ctrl_Message._c[0]),
        &(MBCU_BMU07_Balance_Ctrl_Message._c[1u]),
        &(MBCU_BMU07_Balance_Ctrl_Message._c[2u]),
        &(MBCU_BMU07_Balance_Ctrl_Message._c[3u]),
        &(MBCU_BMU07_Balance_Ctrl_Message._c[4u]),
        &(MBCU_BMU07_Balance_Ctrl_Message._c[5u]),
        &(MBCU_BMU07_Balance_Ctrl_Message._c[6u]),
    },
    {
        &(MBCU_BMU08_Balance_Ctrl_Message._c[0]),
        &(MBCU_BMU08_Balance_Ctrl_Message._c[1u]),
        &(MBCU_BMU08_Balance_Ctrl_Message._c[2u]),
        &(MBCU_BMU08_Balance_Ctrl_Message._c[3u]),
        &(MBCU_BMU08_Balance_Ctrl_Message._c[4u]),
        &(MBCU_BMU08_Balance_Ctrl_Message._c[5u]),
        &(MBCU_BMU08_Balance_Ctrl_Message._c[6u]),
    },
    {
        &(MBCU_BMU09_Balance_Ctrl_Message._c[0]),
        &(MBCU_BMU09_Balance_Ctrl_Message._c[1u]),
        &(MBCU_BMU09_Balance_Ctrl_Message._c[2u]),
        &(MBCU_BMU09_Balance_Ctrl_Message._c[3u]),
        &(MBCU_BMU09_Balance_Ctrl_Message._c[4u]),
        &(MBCU_BMU09_Balance_Ctrl_Message._c[5u]),
        &(MBCU_BMU09_Balance_Ctrl_Message._c[6u]),
    },
    {
        &(MBCU_BMU0A_Balance_Ctrl_Message._c[0]),
        &(MBCU_BMU0A_Balance_Ctrl_Message._c[1u]),
        &(MBCU_BMU0A_Balance_Ctrl_Message._c[2u]),
        &(MBCU_BMU0A_Balance_Ctrl_Message._c[3u]),
        &(MBCU_BMU0A_Balance_Ctrl_Message._c[4u]),
        &(MBCU_BMU0A_Balance_Ctrl_Message._c[5u]),
        &(MBCU_BMU0A_Balance_Ctrl_Message._c[6u]),
    },
    {
        &(MBCU_BMU0B_Balance_Ctrl_Message._c[0]),
        &(MBCU_BMU0B_Balance_Ctrl_Message._c[1u]),
        &(MBCU_BMU0B_Balance_Ctrl_Message._c[2u]),
        &(MBCU_BMU0B_Balance_Ctrl_Message._c[3u]),
        &(MBCU_BMU0B_Balance_Ctrl_Message._c[4u]),
        &(MBCU_BMU0B_Balance_Ctrl_Message._c[5u]),
        &(MBCU_BMU0B_Balance_Ctrl_Message._c[6u]),
    },
    {
        &(MBCU_BMU0C_Balance_Ctrl_Message._c[0]),
        &(MBCU_BMU0C_Balance_Ctrl_Message._c[1u]),
        &(MBCU_BMU0C_Balance_Ctrl_Message._c[2u]),
        &(MBCU_BMU0C_Balance_Ctrl_Message._c[3u]),
        &(MBCU_BMU0C_Balance_Ctrl_Message._c[4u]),
        &(MBCU_BMU0C_Balance_Ctrl_Message._c[5u]),
        &(MBCU_BMU0C_Balance_Ctrl_Message._c[6u]),
    },
    {
        &(MBCU_BMU0D_Balance_Ctrl_Message._c[0]),
        &(MBCU_BMU0D_Balance_Ctrl_Message._c[1u]),
        &(MBCU_BMU0D_Balance_Ctrl_Message._c[2u]),
        &(MBCU_BMU0D_Balance_Ctrl_Message._c[3u]),
        &(MBCU_BMU0D_Balance_Ctrl_Message._c[4u]),
        &(MBCU_BMU0D_Balance_Ctrl_Message._c[5u]),
        &(MBCU_BMU0D_Balance_Ctrl_Message._c[6u]),
    },
    {
        &(MBCU_BMU0E_Balance_Ctrl_Message._c[0]),
        &(MBCU_BMU0E_Balance_Ctrl_Message._c[1u]),
        &(MBCU_BMU0E_Balance_Ctrl_Message._c[2u]),
        &(MBCU_BMU0E_Balance_Ctrl_Message._c[3u]),
        &(MBCU_BMU0E_Balance_Ctrl_Message._c[4u]),
        &(MBCU_BMU0E_Balance_Ctrl_Message._c[5u]),
        &(MBCU_BMU0E_Balance_Ctrl_Message._c[6u]),
    },
    {
        &(MBCU_BMU0F_Balance_Ctrl_Message._c[0]),
        &(MBCU_BMU0F_Balance_Ctrl_Message._c[1u]),
        &(MBCU_BMU0F_Balance_Ctrl_Message._c[2u]),
        &(MBCU_BMU0F_Balance_Ctrl_Message._c[3u]),
        &(MBCU_BMU0F_Balance_Ctrl_Message._c[4u]),
        &(MBCU_BMU0F_Balance_Ctrl_Message._c[5u]),
        &(MBCU_BMU0F_Balance_Ctrl_Message._c[6u]),
    },
};

static uint8* const gparrBmuBalanceErrStatus[PRO_BMU_COUNT_MAX][BALANCE_BYTE_COUNT] =
{
    {
        &(BMU01_Balance_Status_Msg._c[0]),
        &(BMU01_Balance_Status_Msg._c[1u]),
        &(BMU01_Balance_Status_Msg._c[2u]),
        &(BMU01_Balance_Status_Msg._c[3u]),
        &(BMU01_Balance_Status_Msg._c[4u]),
        &(BMU01_Balance_Status_Msg._c[5u]),
        &(BMU01_Balance_Status_Msg._c[6u]),
    },
    {
        &(BMU02_Balance_Status_Msg._c[0]),
        &(BMU02_Balance_Status_Msg._c[1u]),
        &(BMU02_Balance_Status_Msg._c[2u]),
        &(BMU02_Balance_Status_Msg._c[3u]),
        &(BMU02_Balance_Status_Msg._c[4u]),
        &(BMU02_Balance_Status_Msg._c[5u]),
        &(BMU02_Balance_Status_Msg._c[6u]),
    },
    {
        &(BMU03_Balance_Status_Msg._c[0]),
        &(BMU03_Balance_Status_Msg._c[1u]),
        &(BMU03_Balance_Status_Msg._c[2u]),
        &(BMU03_Balance_Status_Msg._c[3u]),
        &(BMU03_Balance_Status_Msg._c[4u]),
        &(BMU03_Balance_Status_Msg._c[5u]),
        &(BMU03_Balance_Status_Msg._c[6u]),
    },
    {
        &(BMU04_Balance_Status_Msg._c[0]),
        &(BMU04_Balance_Status_Msg._c[1u]),
        &(BMU04_Balance_Status_Msg._c[2u]),
        &(BMU04_Balance_Status_Msg._c[3u]),
        &(BMU04_Balance_Status_Msg._c[4u]),
        &(BMU04_Balance_Status_Msg._c[5u]),
        &(BMU04_Balance_Status_Msg._c[6u]),
    },
    {
        &(BMU05_Balance_Status_Msg._c[0]),
        &(BMU05_Balance_Status_Msg._c[1u]),
        &(BMU05_Balance_Status_Msg._c[2u]),
        &(BMU05_Balance_Status_Msg._c[3u]),
        &(BMU05_Balance_Status_Msg._c[4u]),
        &(BMU05_Balance_Status_Msg._c[5u]),
        &(BMU05_Balance_Status_Msg._c[6u]),
    },
    {
        &(BMU06_Balance_Status_Msg._c[0]),
        &(BMU06_Balance_Status_Msg._c[1u]),
        &(BMU06_Balance_Status_Msg._c[2u]),
        &(BMU06_Balance_Status_Msg._c[3u]),
        &(BMU06_Balance_Status_Msg._c[4u]),
        &(BMU06_Balance_Status_Msg._c[5u]),
        &(BMU06_Balance_Status_Msg._c[6u]),
    },
    {
        &(BMU07_Balance_Status_Msg._c[0]),
        &(BMU07_Balance_Status_Msg._c[1u]),
        &(BMU07_Balance_Status_Msg._c[2u]),
        &(BMU07_Balance_Status_Msg._c[3u]),
        &(BMU07_Balance_Status_Msg._c[4u]),
        &(BMU07_Balance_Status_Msg._c[5u]),
        &(BMU07_Balance_Status_Msg._c[6u]),
    },
    {
        &(BMU08_Balance_Status_Msg._c[0]),
        &(BMU08_Balance_Status_Msg._c[1u]),
        &(BMU08_Balance_Status_Msg._c[2u]),
        &(BMU08_Balance_Status_Msg._c[3u]),
        &(BMU08_Balance_Status_Msg._c[4u]),
        &(BMU08_Balance_Status_Msg._c[5u]),
        &(BMU08_Balance_Status_Msg._c[6u]),
    },
    {
        &(BMU09_Balance_Status_Msg._c[0]),
        &(BMU09_Balance_Status_Msg._c[1u]),
        &(BMU09_Balance_Status_Msg._c[2u]),
        &(BMU09_Balance_Status_Msg._c[3u]),
        &(BMU09_Balance_Status_Msg._c[4u]),
        &(BMU09_Balance_Status_Msg._c[5u]),
        &(BMU09_Balance_Status_Msg._c[6u]),
    },
    {
        &(BMU0A_Balance_Status_Msg._c[0]),
        &(BMU0A_Balance_Status_Msg._c[1u]),
        &(BMU0A_Balance_Status_Msg._c[2u]),
        &(BMU0A_Balance_Status_Msg._c[3u]),
        &(BMU0A_Balance_Status_Msg._c[4u]),
        &(BMU0A_Balance_Status_Msg._c[5u]),
        &(BMU0A_Balance_Status_Msg._c[6u]),
    },
    {
        &(BMU0B_Balance_Status_Msg._c[0]),
        &(BMU0B_Balance_Status_Msg._c[1u]),
        &(BMU0B_Balance_Status_Msg._c[2u]),
        &(BMU0B_Balance_Status_Msg._c[3u]),
        &(BMU0B_Balance_Status_Msg._c[4u]),
        &(BMU0B_Balance_Status_Msg._c[5u]),
        &(BMU0B_Balance_Status_Msg._c[6u]),
    },
    {
        &(BMU0C_Balance_Status_Msg._c[0]),
        &(BMU0C_Balance_Status_Msg._c[1u]),
        &(BMU0C_Balance_Status_Msg._c[2u]),
        &(BMU0C_Balance_Status_Msg._c[3u]),
        &(BMU0C_Balance_Status_Msg._c[4u]),
        &(BMU0C_Balance_Status_Msg._c[5u]),
        &(BMU0C_Balance_Status_Msg._c[6u]),
    },
    {
        &(BMU0D_Balance_Status_Msg._c[0]),
        &(BMU0D_Balance_Status_Msg._c[1u]),
        &(BMU0D_Balance_Status_Msg._c[2u]),
        &(BMU0D_Balance_Status_Msg._c[3u]),
        &(BMU0D_Balance_Status_Msg._c[4u]),
        &(BMU0D_Balance_Status_Msg._c[5u]),
        &(BMU0D_Balance_Status_Msg._c[6u]),
    },
    {
        &(BMU0E_Balance_Status_Msg._c[0]),
        &(BMU0E_Balance_Status_Msg._c[1u]),
        &(BMU0E_Balance_Status_Msg._c[2u]),
        &(BMU0E_Balance_Status_Msg._c[3u]),
        &(BMU0E_Balance_Status_Msg._c[4u]),
        &(BMU0E_Balance_Status_Msg._c[5u]),
        &(BMU0E_Balance_Status_Msg._c[6u]),
    },
    {
        &(BMU0F_Balance_Status_Msg._c[0]),
        &(BMU0F_Balance_Status_Msg._c[1u]),
        &(BMU0F_Balance_Status_Msg._c[2u]),
        &(BMU0F_Balance_Status_Msg._c[3u]),
        &(BMU0F_Balance_Status_Msg._c[4u]),
        &(BMU0F_Balance_Status_Msg._c[5u]),
        &(BMU0F_Balance_Status_Msg._c[6u]),
    },
};

static uint8* const gparrBmuBalanceFeedback[PRO_BMU_COUNT_MAX][BALANCE_BYTE_COUNT] =
{
    {
        &(BMU01_Balance_Feedback_Msg._c[0]),
        &(BMU01_Balance_Feedback_Msg._c[1u]),
        &(BMU01_Balance_Feedback_Msg._c[2u]),
        &(BMU01_Balance_Feedback_Msg._c[3u]),
        &(BMU01_Balance_Feedback_Msg._c[4u]),
        &(BMU01_Balance_Feedback_Msg._c[5u]),
        &(BMU01_Balance_Feedback_Msg._c[6u]),
    },
    {
        &(BMU02_Balance_Feedback_Msg._c[0]),
        &(BMU02_Balance_Feedback_Msg._c[1u]),
        &(BMU02_Balance_Feedback_Msg._c[2u]),
        &(BMU02_Balance_Feedback_Msg._c[3u]),
        &(BMU02_Balance_Feedback_Msg._c[4u]),
        &(BMU02_Balance_Feedback_Msg._c[5u]),
        &(BMU02_Balance_Feedback_Msg._c[6u]),
    },
    {
        &(BMU03_Balance_Feedback_Msg._c[0]),
        &(BMU03_Balance_Feedback_Msg._c[1u]),
        &(BMU03_Balance_Feedback_Msg._c[2u]),
        &(BMU03_Balance_Feedback_Msg._c[3u]),
        &(BMU03_Balance_Feedback_Msg._c[4u]),
        &(BMU03_Balance_Feedback_Msg._c[5u]),
        &(BMU03_Balance_Feedback_Msg._c[6u]),
    },
    {
        &(BMU04_Balance_Feedback_Msg._c[0]),
        &(BMU04_Balance_Feedback_Msg._c[1u]),
        &(BMU04_Balance_Feedback_Msg._c[2u]),
        &(BMU04_Balance_Feedback_Msg._c[3u]),
        &(BMU04_Balance_Feedback_Msg._c[4u]),
        &(BMU04_Balance_Feedback_Msg._c[5u]),
        &(BMU04_Balance_Feedback_Msg._c[6u]),
    },
    {
        &(BMU05_Balance_Feedback_Msg._c[0]),
        &(BMU05_Balance_Feedback_Msg._c[1u]),
        &(BMU05_Balance_Feedback_Msg._c[2u]),
        &(BMU05_Balance_Feedback_Msg._c[3u]),
        &(BMU05_Balance_Feedback_Msg._c[4u]),
        &(BMU05_Balance_Feedback_Msg._c[5u]),
        &(BMU05_Balance_Feedback_Msg._c[6u]),
    },
    {
        &(BMU06_Balance_Feedback_Msg._c[0]),
        &(BMU06_Balance_Feedback_Msg._c[1u]),
        &(BMU06_Balance_Feedback_Msg._c[2u]),
        &(BMU06_Balance_Feedback_Msg._c[3u]),
        &(BMU06_Balance_Feedback_Msg._c[4u]),
        &(BMU06_Balance_Feedback_Msg._c[5u]),
        &(BMU06_Balance_Feedback_Msg._c[6u]),
    },
    {
        &(BMU07_Balance_Feedback_Msg._c[0]),
        &(BMU07_Balance_Feedback_Msg._c[1u]),
        &(BMU07_Balance_Feedback_Msg._c[2u]),
        &(BMU07_Balance_Feedback_Msg._c[3u]),
        &(BMU07_Balance_Feedback_Msg._c[4u]),
        &(BMU07_Balance_Feedback_Msg._c[5u]),
        &(BMU07_Balance_Feedback_Msg._c[6u]),
    },
    {
        &(BMU08_Balance_Feedback_Msg._c[0]),
        &(BMU08_Balance_Feedback_Msg._c[1u]),
        &(BMU08_Balance_Feedback_Msg._c[2u]),
        &(BMU08_Balance_Feedback_Msg._c[3u]),
        &(BMU08_Balance_Feedback_Msg._c[4u]),
        &(BMU08_Balance_Feedback_Msg._c[5u]),
        &(BMU08_Balance_Feedback_Msg._c[6u]),
    },
    {
        &(BMU09_Balance_Feedback_Msg._c[0]),
        &(BMU09_Balance_Feedback_Msg._c[1u]),
        &(BMU09_Balance_Feedback_Msg._c[2u]),
        &(BMU09_Balance_Feedback_Msg._c[3u]),
        &(BMU09_Balance_Feedback_Msg._c[4u]),
        &(BMU09_Balance_Feedback_Msg._c[5u]),
        &(BMU09_Balance_Feedback_Msg._c[6u]),
    },
    {
        &(BMU0A_Balance_Feedback_Msg._c[0]),
        &(BMU0A_Balance_Feedback_Msg._c[1u]),
        &(BMU0A_Balance_Feedback_Msg._c[2u]),
        &(BMU0A_Balance_Feedback_Msg._c[3u]),
        &(BMU0A_Balance_Feedback_Msg._c[4u]),
        &(BMU0A_Balance_Feedback_Msg._c[5u]),
        &(BMU0A_Balance_Feedback_Msg._c[6u]),
    },
    {
        &(BMU0B_Balance_Feedback_Msg._c[0]),
        &(BMU0B_Balance_Feedback_Msg._c[1u]),
        &(BMU0B_Balance_Feedback_Msg._c[2u]),
        &(BMU0B_Balance_Feedback_Msg._c[3u]),
        &(BMU0B_Balance_Feedback_Msg._c[4u]),
        &(BMU0B_Balance_Feedback_Msg._c[5u]),
        &(BMU0B_Balance_Feedback_Msg._c[6u]),
    },
    {
        &(BMU0C_Balance_Feedback_Msg._c[0]),
        &(BMU0C_Balance_Feedback_Msg._c[1u]),
        &(BMU0C_Balance_Feedback_Msg._c[2u]),
        &(BMU0C_Balance_Feedback_Msg._c[3u]),
        &(BMU0C_Balance_Feedback_Msg._c[4u]),
        &(BMU0C_Balance_Feedback_Msg._c[5u]),
        &(BMU0C_Balance_Feedback_Msg._c[6u]),
    },
    {
        &(BMU0D_Balance_Feedback_Msg._c[0]),
        &(BMU0D_Balance_Feedback_Msg._c[1u]),
        &(BMU0D_Balance_Feedback_Msg._c[2u]),
        &(BMU0D_Balance_Feedback_Msg._c[3u]),
        &(BMU0D_Balance_Feedback_Msg._c[4u]),
        &(BMU0D_Balance_Feedback_Msg._c[5u]),
        &(BMU0D_Balance_Feedback_Msg._c[6u]),
    },
    {
        &(BMU0E_Balance_Feedback_Msg._c[0]),
        &(BMU0E_Balance_Feedback_Msg._c[1u]),
        &(BMU0E_Balance_Feedback_Msg._c[2u]),
        &(BMU0E_Balance_Feedback_Msg._c[3u]),
        &(BMU0E_Balance_Feedback_Msg._c[4u]),
        &(BMU0E_Balance_Feedback_Msg._c[5u]),
        &(BMU0E_Balance_Feedback_Msg._c[6u]),
    },
    {
        &(BMU0F_Balance_Feedback_Msg._c[0]),
        &(BMU0F_Balance_Feedback_Msg._c[1u]),
        &(BMU0F_Balance_Feedback_Msg._c[2u]),
        &(BMU0F_Balance_Feedback_Msg._c[3u]),
        &(BMU0F_Balance_Feedback_Msg._c[4u]),
        &(BMU0F_Balance_Feedback_Msg._c[5u]),
        &(BMU0F_Balance_Feedback_Msg._c[6u]),
    },
};

/**************************************************
 * @brief  Intial function of balance module, must call once when initalize the project
 * @@param none
 * @retrun none
 *************************************************/
void AlgInitBalance(void)
{
    uint8 ucBmuId;
    uint8 ucBalanceByte;
    
	sendBalanceCmd(0u);
	(void)Stb_S_SetTimer(&guBalancePrecisionTimerS);

	guContinueBalanceFlag = BALANCE_OFF;

    /*check if need to operate balance*/
    for(ucBmuId = 0u ; ucBmuId < glbucSysBmuCount ; ucBmuId += 1u)
    {
        for(ucBalanceByte = 0u ; ucBalanceByte < BALANCE_BYTE_COUNT ; ucBalanceByte += 1u)
        {
            /*found, the exit*/
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

        if(BALANCE_ON == guContinueBalanceFlag)
        {
            break;
        }
        else
        {
            /*do nothing*/
        }
    }

    guContinueTime = (guContinueTime > BALANCE_CONTINUE_MAX) ? 0u : guContinueTime;

    if(guContinueTime > 0)
    {
        guContinueTime = (uint16)(guContinueTime - 1u);       /*reduce one minute*/
    }
    else
    {
        /*not in balance do nothing*/
    }

    if(BALANCE_OFF == guContinueBalanceFlag)
    {
        guContinueTime = 0u;
    }
    else
    {
        /*continue balance*/
    }
}

/**************************************************
 * @brief       fulfil the balance command output buffer
 * @@param[in]
 * -# 1 send balance cmd by the calculation
 * -# 0 send balance cmd all zeros
 * @retrun none
 *************************************************/
static void sendBalanceCmd(uint8 ucCmd)
{
    uint8 ucBmuId;
    uint8 ucBalanceByte;

    if(ucCmd)
    {
        for(ucBmuId = 0u ; ucBmuId < glbucSysBmuCount ; ucBmuId += 1u)
        {
            for(ucBalanceByte = 0u ; ucBalanceByte < BALANCE_BYTE_COUNT ; ucBalanceByte += 1u)
            {
                *(gparrBmuBalanceData[ucBmuId][ucBalanceByte]) = gucarrBalanceFlag[ucBmuId][ucBalanceByte];
            }
        }
    }
    else
    {
        for(ucBmuId = 0u ; ucBmuId < glbucSysBmuCount ; ucBmuId += 1u)
        {
            for(ucBalanceByte = 0u ; ucBalanceByte < BALANCE_BYTE_COUNT ; ucBalanceByte += 1u)
            {
                 *(gparrBmuBalanceData[ucBmuId][ucBalanceByte]) = 0u;
            }
        }
    }
}

/****************************************
 * @brief: check balance error status
 * @return: none
 * **************************************/
static void checkBalanceFbStatus(void)
{
    uint8 ucBmuId;
    uint8 ucBalanceByte;

    gucBalancestatus = BALANCE_OFF;

    for(ucBmuId = 0u ; ucBmuId < glbucSysBmuCount; ucBmuId += 1u)
    {
        for(ucBalanceByte = 0u ; ucBalanceByte < BALANCE_BYTE_COUNT ; ucBalanceByte += 1u)
        {
            if(*(gparrBmuBalanceFeedback[ucBmuId][ucBalanceByte]))
            {
                gucBalancestatus = BALANCE_ON;
                break;
            }
            else
            {
                /*do nothing*/
            }
        }

        if(BALANCE_ON == gucBalancestatus)
        {
            break;
        }
        else
        {
            /*do nothing*/
        }
    }
}

/****************************************
 * @brief: check balance error status
 * @return: none
 * **************************************/
static void checkBalanceErrStatus(void)
{
    uint8 ucBmuId;
    uint8 ucBalanceByte;
    uint8 ucRet;
    static uint8 ucCount = 0u;

    ucRet = 0u;

    for(ucBmuId = 0u ; ucBmuId < glbucSysBmuCount; ucBmuId += 1u)
    {

        for(ucBalanceByte = 0u ; ucBalanceByte < BALANCE_BYTE_COUNT; ucBalanceByte += 1u)
        {
            if(*(gparrBmuBalanceErrStatus[ucBmuId][ucBalanceByte]))
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
//            ApiPutBalanceErr(1u);
        }
    }
    else
    {
        ucCount = 0u;
//        ApiPutBalanceErr(0u);
    }
}


/**************************************************
 * @brief  check if Threshold Table is valid
 * @@param[in]
 * -# 1 send balance cmd by the calculation
 * -# 0 send balance cmd all zeros
 * @retrun none
 *************************************************/
uint8 CheckThreshold(void)
{
    uint8 ucAns = TRUE;
    uint8 ucCurIdx;

    if(sizeof(ChgThreshold) / sizeof(ChgThreshold[0]) < 1u)
    {
        ucAns = FALSE;
    }
    else
    {
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
    if((SYS_ERR_LEVEL_NORMAL != RteGetSlaverCommErr())
//    	|| SYS_ERR_LEVEL_NORMAL != RteGetBmuSpiCommErr()
//		|| SYS_ERR_LEVEL_NORMAL != RteGetBalanceCircuitErr()
//		|| SYS_ERR_LEVEL_NORMAL != RteGetBalanceErr()
        || (FALSE == checkVoltVaild()))
    {
        sendBalanceCmd(0u);   /*slave community lost not working*/
        (void)Stb_S_SetTimer(&guBalancePrecisionTimerS);
    }
    else
    {
        if(RteGetDCChgIsChargingStep() || ACCHG_STEP_CHARGING == RteGetACChgStep())
        {
            if(CURR_ZERO > RteGetTotalCurrentValue())
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

            if(BALANCE_OFF == guContinueBalanceFlag)
            {
                normalBalance(DCH);
            }
            else
            {
                /*Do nothing in normalBalance*/
            }

            continueBalance();
        }
    }
}

/****************************************
 * @brief: clear balance flag, then the cmd will send all zeros
 * @return: none
 * **************************************/
void ApiClearBalanceFlag(void)
{
    uint8 ucBmuId;
    uint8 ucBalanceByte;

    for(ucBmuId = 0 ; ucBmuId < glbucSysBmuCount ; ucBmuId += 1u)
    {
        for(ucBalanceByte = 0 ; ucBalanceByte < BALANCE_BYTE_COUNT ; ucBalanceByte += 1u)
        {
            gucarrBalanceFlag[ucBmuId][ucBalanceByte] = BALANCE_OFF;
        }
    }
    guContinueTime = 0;
}

/****************************************
 * @brief: main task of balance, called in 100 ms task
 * @return: none
 * **************************************/
void AlgTaskBalance(void)
{
    static uint8 ucRuntime = 0;

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
 * @brief: main task of balance, called in 10 ms task
 * @return: none
 ***********************************/
void ApiSetBalanceFlag(void)
{
    uint8 ucBmuId;
    uint8 ucBmuLinkId;
    uint8 ucCellIdx;
    uint8 ucPackNum;
    uint16 uIndex;
    uint8 ucNeedBalance;
    uint16 uBalanceDeltVolt;
    uint16 uBalancingSoc;
    uint8 ucVoltPerMillSoc;
    uint16 uCellVoltMax;
    uint16 uCellVoltDiff;
    uint16 uCellVoltMin;

    ucNeedBalance = FALSE;

    if((RteGetTotalCellTempMinMin() <= BALANCE_DYNAMIC_TEMPMIN) && (RteGetTotalCellTempDiff() > BALANCE_LOWSOC_TEMPDIFF)
        || (guContinueBalanceFlag == BALANCE_ON) && (guContinueTime > 120u))
    {
        /*do not overwrite the existing balance setting*/
        return;
    }


    uCellVoltMax = RteGetTotalCellVoltMaxMax();
    uCellVoltDiff = RteGetTotalCellVoltDiff();
    uCellVoltMin = RteGetTotalCellVoltMinMin();

    // maxmax >= 3500
    if(uCellVoltMax >= BALANCE_CHGVOLTLVL1)
    {
        // maxmax >= 3650
        if(uCellVoltMax >= PRO_CHG_CELL_VOLT_MAX)
        {
            // Δv = 140
            uBalanceDeltVolt = BALANCE_CHGDELTVOLT2;
        }
        // 3650 > maxmax >= 3500
        else
        {
            // Δv = 40 ~ 140
            uBalanceDeltVolt = BALANCE_CHGDELTVOLT1 + (uint16)(((uint32)(uCellVoltMax - BALANCE_CHGVOLTLVL1) * (BALANCE_CHGDELTVOLT2 - BALANCE_CHGDELTVOLT1) / (PRO_CHG_CELL_VOLT_MAX - BALANCE_CHGVOLTLVL1)) & 0xFFFFu);
        }
        // diff > Δv
        if(uCellVoltDiff > uBalanceDeltVolt)
        {

            if(uCellVoltMax >= PRO_CHG_CELL_VOLT_MAX)
            {
                ucVoltPerMillSoc = BALANCE_CHGVOLTPERSOCLVL2;
            }
            else
            {
                ucVoltPerMillSoc = (uint8)(BALANCE_CHGVOLTPERSOCLVL1 + ((uCellVoltMax - BALANCE_CHGVOLTLVL1) * (BALANCE_CHGVOLTPERSOCLVL2 - BALANCE_CHGVOLTPERSOCLVL1) / (PRO_CHG_CELL_VOLT_MAX - BALANCE_CHGVOLTLVL1)));
            }

            /*
            3650mv 时 uBalanceDeltVolt 为 140mv
            因此，uCellVoltDiff 为 190mv时

            电芯极差        需要均衡多少容量(单位0.1%)
            uCellVoltDiff  bal_cap(0.1%)
            190mv           0.5%
            180mv           0.4%
            170mv           0.3%
            160mv           0.2%
            141mv~150mv     0.1%

            */
            uBalancingSoc = (uint16)(((uint32)(uCellVoltDiff - uBalanceDeltVolt) + (uint32)ucVoltPerMillSoc- 1u)/ (uint32)ucVoltPerMillSoc);

            if(uBalancingSoc >= BALANCE_CONTINUE_SOCCONSUME)
            {
                guContinueTime = (uint16)(BALANCE_CONTINUE_MAX > 0xFFFFu ? 0xFFFFu: BALANCE_CONTINUE_MAX);
            }
            else
            {
                guContinueTime = (uint16)(((uint32)BALANCE_CONTINUE_AS * uBalancingSoc / BALANCE_CONTINUE_PRECESION));
            }
        }
        else
        {
            ApiClearBalanceFlag();
            return;
        }
    }
    // maxmax < 3500
    else
    {
        // maxmax < 3450
        if(uCellVoltMax < BALANCE_CHGVOLTLVL0)
        {
            // Δv = 20 mv
            uBalanceDeltVolt = BALANCE_CHGDELTVOLT0;
        }
        // 3450 <= maxmax < 3500
        else
        {
            // Δv = 20 ~ 40 mv
            uBalanceDeltVolt = BALANCE_CHGDELTVOLT0 + (uint16)(((uint32)(uCellVoltMax - BALANCE_CHGVOLTLVL0) * (BALANCE_CHGDELTVOLT1 - BALANCE_CHGDELTVOLT0) / (BALANCE_CHGVOLTLVL1 - BALANCE_CHGVOLTLVL0)) & 0xFFFFu);
        }

        if(uCellVoltDiff > uBalanceDeltVolt)
        {
            // 0.4% cap
            guContinueTime = (uint16)((BALANCE_CONTINUE_AS * BALANCE_SOC_NORMAL / BALANCE_CONTINUE_PRECESION) & 0xFFFFu);
        }
        else
        {
            ApiClearBalanceFlag();
            return;
        }
    }

    for(ucPackNum = 0u; ucPackNum < glbucSysPackParallel; ucPackNum += 1u)
    {
        uIndex = 0u;
        for(ucBmuId = (glbucSysBmuCount / glbucSysPackParallel) * ucPackNum; ucBmuId < (glbucSysBmuCount / glbucSysPackParallel) * (ucPackNum + 1u); ucBmuId += 1u)
        {
            ucBmuLinkId = glbucarrBmuIndex[ucBmuId];

            for(ucCellIdx = 0u; ucCellIdx < glbucSysBmuCellConfig[ucBmuLinkId]; ucCellIdx += 1u)
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

                if((uint32)RteGetCellVoltage(ucPackNum, uIndex) > ((uint32)uCellVoltMin + uBalanceDeltVolt))
                {
                    gucarrBalanceFlag[ucBmuLinkId][ucCellIdx >> 3u] |= (uint8)(((uint8)BALANCE_ON) << (ucCellIdx & 0x7u));
                    ucNeedBalance = TRUE;
                }
                else
                {
                    gucarrBalanceFlag[ucBmuLinkId][ucCellIdx >> 3u] &= (uint8)(~(((uint8)BALANCE_ON) << (ucCellIdx & 0x7u)));
                }

                uIndex += 1u;
            }
        }
    }

    if(TRUE == ucNeedBalance)
    {
        (void)ApiNvmWriteBalanceData();
        (void)Stb_S_SetTimer(&guBalancePrecisionTimerS);
        guContinueBalanceFlag = BALANCE_ON;
    }
    else
    {
        /*No valid cell passed the final selection; do not keep stale time.*/
        ApiClearBalanceFlag();
    }
}

/*********************************************************************
* @brief    Interface of set force balance flag
* @param    none
* @retval   none
*********************************************************************/
void ApiSetForceBalance(void)
{
    uint8 ucPackNum;
    uint8 ucTemp;
    uint8 ucBmudIdx;
    uint8 ucCellIdx;
    uint16 uVoltTemp = 0u;
    uint16 uVoltMax = 0u;
    uint16 uVoltMin = 0u;
    uint32 uTempValue;
    uint8 ucNeedBalance;

    ucNeedBalance = FALSE;

    if ((RteGetTotalCurrentValue() < 5u * CURR_FACTOR + CURR_ZERO) &&
            (RteGetTotalCurrentValue() > CURR_ZERO - 5u * CURR_FACTOR))
    {
        ApiClearBalanceFlag();
        for (ucPackNum = 0u; ucPackNum < PRO_PACK_PARELL_NUM; ucPackNum += 1u)
        {
            uVoltMax = RteGetCellVoltMax(ucPackNum, 0u);
            uVoltMin = RteGetCellVoltMin(ucPackNum, 0u);

            /*reuse of uVoltTemp*/
            uVoltTemp = uVoltMax > uVoltMin ? uVoltMin + 10u : uVoltMax + 10u;

            ucTemp = (uint8) (ucPackNum * PRO_BMU_COUNT / PRO_PACK_PARELL_NUM);

            for (ucBmudIdx = ucTemp; ucBmudIdx < ucTemp + PRO_BMU_COUNT / PRO_PACK_PARELL_NUM; ucBmudIdx += 1u)
            {
                for (ucCellIdx = 0u; ucCellIdx < glbucSysBmuCellConfig[ucBmudIdx]; ucCellIdx += 1u)
                {
                    if (ucCellIdx >= BALANCE_BMU_CELL_COUNT)
                    {
                        break;
                    }

                    uTempValue = RteGetCellVoltage(ucPackNum, ucCellIdx);
                    uTempValue &= 0x0000FFFFul;
                    /*uVoltTemp is the lowewst voltage that need to be balanced*/
                    if (uTempValue > uVoltTemp)
                    {
                        gucarrBalanceFlag[ucBmudIdx][ucCellIdx >> 3u] |= (uint8)(((uint8)BALANCE_ON) << (ucCellIdx & 0x7u));
                        ucNeedBalance = TRUE;
                    }
                    else
                    {
                        gucarrBalanceFlag[ucBmudIdx][ucCellIdx >> 3u] &= (uint8)(~(((uint8)BALANCE_ON) << (ucCellIdx & 0x7u)));
                    }
                }
            }
        }

        if(TRUE == ucNeedBalance)
        {
            guContinueTime = (uint16)(BALANCE_CONTINUE_MAX > 0xFFFFu ? 0xFFFFu : BALANCE_CONTINUE_MAX);
            (void)Stb_S_SetTimer(&guBalancePrecisionTimerS);
            (void)ApiNvmWriteBalanceData();
            guContinueBalanceFlag = BALANCE_ON;
        }
        else
        {
            /*donot need continue balance*/
        }
    }
}

void ApiClearForceBalance(void)
{
    ApiClearBalanceFlag();
}

/*******************************************************
 * continue operation balance
 * *****************************************************/
static void continueBalance(void)
{

    if(BALANCE_ON == guContinueBalanceFlag)
    {
        if( 0u == guContinueTime
            || (RteGetTotalCellVoltMinMin() <= PRO_DCHG_CELL_VOLT_MIN))
        {
            guContinueBalanceFlag = BALANCE_OFF;
            guContinueTime = 0;
            ApiClearBalanceFlag();
            sendBalanceCmd(0u);
        }
        else
        {
            if(Stb_S_Timeout(guBalancePrecisionTimerS,BALANCE_MINUTE))
            {
                guContinueTime = (uint16)(guContinueTime - 1u);
                (void)Stb_S_SetTimer(&guBalancePrecisionTimerS);
            }
            else
            {
                /*do nothing*/
            }
            sendBalanceCmd(1u);
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
    uint8 ucBmuId;
    uint8 ucBmuLinkId;
    uint8 ucCellIdx;
    uint8 ucPackNum;
    uint16 uIndex;

	
	for(ucPackNum = 0u; ucPackNum < glbucSysPackParallel; ucPackNum += 1u)
	{
        if(DCH == input && RteGetStmCurrMean(ucPackNum) < CURR_ZERO)
        {
            sendBalanceCmd(0u);
            continue;
        }
        else
        {
            /*do nothing*/
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

				if((uint32)RteGetCellVoltage(ucPackNum, uIndex) > (((uint32)RteGetCellVoltMin(ucPackNum,0u) + getThreshold(input))))
				{
				    *(gparrBmuBalanceData[ucBmuLinkId][ucCellIdx >> 3u]) |= (uint8)(((uint8)BALANCE_ON) << (ucCellIdx & 0x7u));
				}
				else
				{
				    *(gparrBmuBalanceData[ucBmuLinkId][ucCellIdx >> 3u]) &= (uint8)(~(((uint8)BALANCE_ON) << (ucCellIdx & 0x7u)));
				}
	
				uIndex += 1u;
			}
		}
	}
}


static uint16 getThreshold(Direction input)
{
    uint16 uAns = 0u;
    uint8 ucThresholdIdx = 0u;
    
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
                    uAns =(uint16)( DchThreshold[ucThresholdIdx].uThreshold + (RteGetTotalCellTempDiff() - DchThreshold[ucThresholdIdx].uTempDif) * (DchThreshold[ucThresholdIdx + 1u].uThreshold - DchThreshold[ucThresholdIdx].uThreshold) / (DchThreshold[ucThresholdIdx + 1u].uTempDif - DchThreshold[ucThresholdIdx].uTempDif));
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
                    uAns = (uint16)(ChgThreshold[ucThresholdIdx].uThreshold + (RteGetTotalCellTempDiff() - ChgThreshold[ucThresholdIdx].uTempDif) * (ChgThreshold[ucThresholdIdx + 1u].uThreshold - ChgThreshold[ucThresholdIdx].uThreshold) / (ChgThreshold[ucThresholdIdx + 1u].uTempDif - ChgThreshold[ucThresholdIdx].uTempDif));
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


static uint8 checkVoltVaild(void)
{
    uint8 ucAns;

    ucAns = TRUE;

    if((RteGetTotalCellVoltMinMin() > PRO_CELLVOLT_REASONABLE_MAX)
        || (RteGetTotalCellVoltMinMin() < PRO_CELLVOLT_REASONABLE_MIN)
        || (RteGetTotalCellVoltMinMin() < glbucarrErrThreshold[ERR_NUM_CELL_VOLT_LOW].FaultThreshold[POWER_LIMP_INDEX]))
    {
        ucAns = FALSE;
    }
    else
    {
        /*keep value*/
    }

    if(TRUE == ucAns)
    {
        if((RteGetTotalCellVoltMaxMax() > PRO_CELLVOLT_REASONABLE_MAX)
             || (RteGetTotalCellVoltMaxMax() < PRO_CELLVOLT_REASONABLE_MIN))
        {
            ucAns = FALSE;
        }
        else
        {
            /*keep value*/
        }
    }
    else
    {
        /*already get ans do nothing*/
    }
    return ucAns;
}

/**************************************************
 * @brief: get the balance status
 * @return: if one cell is in balacne , the resuls is true
 * @retval: 1, if true, 0, false
 * ************************************************/
uint8 RteGetBalanceActiveSt(void)
{
    return gucBalancestatus;
}


uint16 RteGetRemainContinueTime(void)
{
    return guContinueTime;
}


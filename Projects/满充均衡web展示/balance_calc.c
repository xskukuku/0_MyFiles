#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

// 类型定义
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;

// 配置参数
uint16 PRO_CHG_CELL_VOLT_MAX = 3650u;
uint16 PRO_CELL_CAPACITY = 340u;
uint16 SOH_VALUE = 1000u;  // SOH = 100%, scale 0.1, 所以是 1000
uint16 BALANCE_CHGVOLTLVL0 = 3450u;
uint16 BALANCE_CHGVOLTLVL1 = 3500u;
uint8 BALANCE_CHGVOLTPERSOCLVL1 = 6u;
uint8 BALANCE_CHGVOLTPERSOCLVL2 = 10u;
uint16 BALANCE_CHGDELTVOLT0 = 20u;
uint16 BALANCE_CHGDELTVOLT1 = 40u;
uint16 BALANCE_CHGDELTVOLT2 = 140u;
uint16 BALANCE_SOC_MIN = 2u;
uint16 BALANCE_SOC_NORMAL = 4u;
uint16 BALANCE_SOC_MAX = 7u;
uint16 BALANCE_TIME_MAX = 1920u;

// 全局变量
uint16 guContinueTime = 0;

// 输入参数
uint16 gTotalCellVoltMaxMax = 3450u;
uint16 gTotalCellVoltDiff = 20u;

// 输出结果
uint16 gBalanceDeltVolt = 0;
uint16 gBalancingSoc = 0;
uint8 gVoltPerMillSoc = 0;
uint8 gBalanceCleared = 0;

// calculateBalanceTime函数
static uint16 calculateBalanceTime(uint16 uBalanceSoc)
{
    uint32 ulBalanceTime;
    uint32 BALANCE_CONTINUE_AS;
    uint32 BALANCE_CONTINUE_PRECESION = 5u;

    // BALANCE_CONTINUE_AS = PRO_CELL_CAPACITY * RteGetTotalSohValue() * 6 * 6 / 10 / 1000
    // PRO_CELL_CAPACITY = 340 Ah
    // RteGetTotalSohValue() = 1000 (表示100%, scale 0.1)
    // 6 * 6 = 36 (转换系数)
    // 结果单位: As (安培·秒)
    // 340 * 1000 * 36 / 10 / 1000 = 1224 As
    BALANCE_CONTINUE_AS = ((uint32)PRO_CELL_CAPACITY * (uint32)SOH_VALUE * 6u * 6u / 10u / 1000u);

    // guContinueTime = (BALANCE_CONTINUE_AS * uBalanceSoc + BALANCE_CONTINUE_PRECESION/2) / BALANCE_CONTINUE_PRECESION
    // BALANCE_CONTINUE_PRECESION = 5 As (100mA * 60s * 80%)
    // 单位: 分钟
    ulBalanceTime = ((uint32)BALANCE_CONTINUE_AS * (uint32)uBalanceSoc + ((uint32)BALANCE_CONTINUE_PRECESION / 2u)) / (uint32)BALANCE_CONTINUE_PRECESION;

    if(ulBalanceTime > (uint32)BALANCE_TIME_MAX)
    {
        ulBalanceTime = (uint32)BALANCE_TIME_MAX;
    }

    if(ulBalanceTime > 0xFFFFu)
    {
        ulBalanceTime = 0xFFFFu;
    }

    return (uint16)ulBalanceTime;
}

// ApiSetBalanceFlag核心逻辑
void ApiSetBalanceFlag(void)
{
    uint16 uBalanceDeltVolt;
    uint16 uBalancingSoc;
    uint16 uCellVoltMax;
    uint16 uCellVoltDiff;
    uint8 ucVoltPerMillSoc;

    gBalanceCleared = 0;
    uCellVoltMax  = gTotalCellVoltMaxMax;
    uCellVoltDiff = gTotalCellVoltDiff;

    // 计算uBalanceDeltVolt
    if(uCellVoltMax < BALANCE_CHGVOLTLVL0)/* MaxMax < 3450mv */
    {
        uBalanceDeltVolt = BALANCE_CHGDELTVOLT0;
    }
    else if(uCellVoltMax >= PRO_CHG_CELL_VOLT_MAX) /* MaxMax >= 3650mv */
    {
        uBalanceDeltVolt = BALANCE_CHGDELTVOLT2;
    }
    else if(uCellVoltMax < BALANCE_CHGVOLTLVL1)/* 3450mv <= MaxMax < 3500mv */
    {
        uBalanceDeltVolt = BALANCE_CHGDELTVOLT0 + (uint16)(((uint32)(uCellVoltMax - BALANCE_CHGVOLTLVL0) * (BALANCE_CHGDELTVOLT1 - BALANCE_CHGDELTVOLT0) / (BALANCE_CHGVOLTLVL1 - BALANCE_CHGVOLTLVL0)) & 0xFFFFu);
    }
    else/* 3500mv <= MaxMax < 3650mv */
    {
        uBalanceDeltVolt = BALANCE_CHGDELTVOLT1 + (uint16)(((uint32)(uCellVoltMax - BALANCE_CHGVOLTLVL1) * (BALANCE_CHGDELTVOLT2 - BALANCE_CHGDELTVOLT1) / (PRO_CHG_CELL_VOLT_MAX - BALANCE_CHGVOLTLVL1)) & 0xFFFFu);
    }

    // 检查是否需要清除均衡标志
    if(uCellVoltDiff <= uBalanceDeltVolt)
    {
        // ApiClearBalanceFlag
        gBalanceCleared = 1;
        gBalanceDeltVolt = uBalanceDeltVolt;
        gBalancingSoc = 0;
        gVoltPerMillSoc = 0;
        guContinueTime = 0;
        return;
    }
    
    // 计算其他参数
    if(uCellVoltMax >= BALANCE_CHGVOLTLVL1)/* MaxMax >=  3500*/
    {
        if(uCellVoltMax >= PRO_CHG_CELL_VOLT_MAX)/* MaxMax >=  3650*/
        {
            ucVoltPerMillSoc = BALANCE_CHGVOLTPERSOCLVL2;
        }
        else/* 3500mv <= MaxMax < 3650mv */
        {
            ucVoltPerMillSoc = (uint8)(BALANCE_CHGVOLTPERSOCLVL1 + ((uCellVoltMax - BALANCE_CHGVOLTLVL1) * (BALANCE_CHGVOLTPERSOCLVL2 - BALANCE_CHGVOLTPERSOCLVL1) / (PRO_CHG_CELL_VOLT_MAX - BALANCE_CHGVOLTLVL1)));
        }
        uBalancingSoc = (uint16)((uint32)(uCellVoltDiff - uBalanceDeltVolt) + ucVoltPerMillSoc / 2u) / ucVoltPerMillSoc + BALANCE_SOC_NORMAL;
        guContinueTime = calculateBalanceTime((uBalancingSoc < BALANCE_SOC_MAX) ? uBalancingSoc : BALANCE_SOC_MAX);
        
        gBalanceDeltVolt = uBalanceDeltVolt;
        gBalancingSoc = (uBalancingSoc < BALANCE_SOC_MAX) ? uBalancingSoc : BALANCE_SOC_MAX;
        gVoltPerMillSoc = ucVoltPerMillSoc;
    }
    else/* MaxMax <  3500mv*/
    {
        /* Low voltage region: use minimum SOC */
        guContinueTime = calculateBalanceTime(BALANCE_SOC_MIN);
        
        gBalanceDeltVolt = uBalanceDeltVolt;
        gBalancingSoc = BALANCE_SOC_MIN;
        gVoltPerMillSoc = 0;
    }
}

int main(int argc, char *argv[])
{
    if(argc < 3)
    {
        printf("Usage: %s <voltMax> <voltDiff>\n", argv[0]);
        return 1;
    }

    gTotalCellVoltMaxMax = (uint16)atoi(argv[1]);
    gTotalCellVoltDiff = (uint16)atoi(argv[2]);

    // 调用核心函数
    ApiSetBalanceFlag();

    // 计算小时数（四舍五入）
    uint16 hours = (guContinueTime + 30) / 60;

    // 输出JSON格式结果
    printf("{\n");
    printf("  \"uBalanceDeltVolt\": %u,\n", gBalanceDeltVolt);
    printf("  \"guContinueTime\": %u,\n", guContinueTime);
    printf("  \"guContinueTimeHours\": %u,\n", hours);
    printf("  \"uBalancingSoc\": %u,\n", gBalancingSoc);
    printf("  \"ucVoltPerMillSoc\": %u,\n", gVoltPerMillSoc);
    printf("  \"isCleared\": %u\n", gBalanceCleared);
    printf("}\n");

    return 0;
}

/*************************************************************************************************************
 * @file    balance.h
 * @biref   main and initial task of balance operation
 * @details when charge end, mark the cells that need to be balanced, then run balance cmd in discharge mode
 * @par     history
 * <table>
 * <tr><th>Author    <th>Date        <th>Version   <th>ChangeLogs  </tr>
 * <tr><td>shenxd    <td>2019-04-08  <td>V1.0      <td>First issue </tr>
 * <tr><td>shenxd    <td>2020-09-02  <td>V1.1      <td>
 * -# remove many dependencies of pro_confgi.h
 * -# use const varibles to replace macro define
 * -# add bmu count to the maximum: 15
 * </table>
 **************************************************************************************************************/

#ifndef BALANCE_H_
#define BALANCE_H_

#include "comm_inc.h"

extern void AlgInitBalance(void);

extern void AlgTaskBalance(void);

extern void ApiClearBalanceFlag(void);

extern void ApiSetChgBalanceFlag(void);
extern void ApiSetLowSOCBalanceFlag(INT8U ucSOCAmemdFlag);

extern INT8U RteGetBalanceActiveSt(void);
extern INT16U RteGetBalanceCellNum(void);
extern INT8U RteGetBalancePreCondMet(void);
extern INT8U RteGetCellBalanceByteStatus(INT8U ucByteIndex);
extern INT8U RteGetCellBalanceCmd(INT8U ucBmuId, INT8U ucByte);
extern void ApiPutVCANBalanceOffCmd(INT8U ucdata);
extern INT16U RteGetRemainContinueTime(void);
extern INT8U RteGetBalancingType(void);

#endif /*BALANCE_H_*/

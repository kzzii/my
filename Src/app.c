/*
 * app.c
 *
 *  Created on: 2022. 11. 8.
 *      Author: NB70
 */
#include<stdio.h>

#include "main.h"
#include "app.h"
#include "cell.h"
#include "param.h"
#include "stm32g4xx_it.h"
/*--------------------------------------------------------------------------*/
typedef struct _app_t {
	uint8_t responseDataType;
	uint16_t turnLedToCurrentValue[4];
	uint16_t currentLimit[4];
} app_t;
static app_t self;
/*--------------------------------------------------------------------------*/
static uint16_t cellout_store[CELL_LEN] = {0};

static void setParam (
		hostData_t *hostData,
		void (*setFunc)(uint8_t, uint16_t)
		) {
	uint8_t i, j = 0;
	uint16_t tempConvValue;
		for (i = cell_1; i < cell_num; i++) {
			tempConvValue = (uint16_t)hostData->data[j++] << 8;
			tempConvValue |= (uint16_t)hostData->data[j++];

			cellout_store[i]=tempConvValue;

			setFunc(i, tempConvValue);
		}
}

static void setParam_2 (
		hostData_t *hostData,
		uint8_t par_n,
		void (*setFunc)(uint8_t, uint8_t, uint16_t)
		) {
	uint8_t i, j = 0;
	uint16_t tempConvValue;
	for (i = cell_1; i < cell_num; i++) {
		tempConvValue = (uint16_t)hostData->data[j++] << 8;
		tempConvValue |= (uint16_t)hostData->data[j++];
		setFunc(i, par_n, tempConvValue);
	}
}


hostData_t host_getData(void) {
	hostData_t res = {0, {0}};
	newDataFlag = false;

	res.cmd = recvData[0];
	uint8_t i;
	for (i = 0; i < 8; i++) res.data[i] = recvData[i +1];

	return res;
}

void host_sendDataSet(uint8_t index, uint8_t data) {
	aTxBuffer[index] = data;
}

void currentLimitControl(void) {
	uint8_t i;
	bool limitFlag = false;
	for (i = cell_1; i < cell_num; i++) {
		if (Cell_current(i) >= self.currentLimit[i]) {
			Cell_setCellOutRawValue(i, 0);
			limitFlag = true;
		}
	}
	if (limitFlag == true) Cell_updateCellOut();
}


void balancingLedControl(void) {

	if (Cell_current(cell_1) < self.turnLedToCurrentValue[cell_1]){ BALANCING_CELL1_OFF();}
	else{ BALANCING_CELL1_ON();}

	if (Cell_current(cell_2) < self.turnLedToCurrentValue[cell_2]){ BALANCING_CELL2_OFF();}
	else{ BALANCING_CELL2_ON();}

	if (Cell_current(cell_3) < self.turnLedToCurrentValue[cell_3]){ BALANCING_CELL3_OFF();}
	else{ BALANCING_CELL3_ON();}

	if (Cell_current(cell_4) < self.turnLedToCurrentValue[cell_4]){ BALANCING_CELL4_OFF();}
	else{ BALANCING_CELL4_ON();}
}

void commandProcessing(void) {
	newDataFlag = false;

	uint8_t i, j = 0;
	uint16_t tempConvValue;
	hostData_t hostData = host_getData();
	switch(hostData.cmd) {
		case cmd_setDac:
			setParam(&hostData, Cell_setCellOutRawValue);
			Cell_cellOutCalibration();
	        Cell_updateCellOut();
	        break;
	        /////////////////////////////////////////////////////////
		case cmd_setScale_m:
			setParam(&hostData, Cell_setParam_CellOutMultiply);
			break;
		case cmd_setScale_d:
			setParam(&hostData, Cell_setParam_CellOutDivide);
			break;
		case cmd_setOffset_p:
			setParam(&hostData, Cell_setParam_CellOutPlus);
			break;
		case cmd_setOffset_m:
			setParam(&hostData, Cell_setParam_CellOutMinus);
			break;

	    	/////////////////////////////////////////////////////////
		case cmd_flashWrite:
			Cell_writeParam();
			break;
		case cmd_reset:
			cell_init();
			break;
		case cmd_dummy:
			break;
		case cmd_selectResponse:
			self.responseDataType = hostData.data[1];
			break;
	    	/////////////////////////////////////////////////////////
		case cmd_voltageScale_m:
			setParam(&hostData, Cell_setParam_voltageMultiply);
			break;
		case cmd_voltageScale_d:
			setParam(&hostData, Cell_setParam_voltageDivide);
			break;
		case cmd_voltageOffset_p:
			setParam(&hostData, Cell_setParam_voltagePlus);
			break;
		case cmd_voltageOffset_m:
			setParam(&hostData, Cell_setParam_voltageMinus);
			break;

			/////////////////////////////////////////////////////////
		case cmd_currentP1_1_m:
			setParam_2(&hostData, 0, Cell_setParam_currentP1Multiply);
			break;
		case cmd_currentP1_1_d:
			setParam_2(&hostData, 0, Cell_setParam_currentP1Divide);
			break;
		case cmd_currentP2_1_m:
			setParam_2(&hostData, 0, Cell_setParam_currentP2Multiply);
			break;
		case cmd_currentP2_1_d:
			setParam_2(&hostData, 0, Cell_setParam_currentP2Divide);
			break;
		case cmd_currentP3_1_p:
			setParam_2(&hostData, 0, Cell_setParam_currentP3Plus);
			break;
		case cmd_currentP3_1_m:
			setParam_2(&hostData, 0, Cell_setParam_currentP3Minus);
			break;

			/////////////////////////////////////////////////////////
		case cmd_turnLedToCurrentValue:
			for (i = cell_1; i < cell_num; i++) {
				tempConvValue = (uint16_t)hostData.data[j++] << 8;
				tempConvValue |= (uint16_t)hostData.data[j++];
				Cell_setTurnLedToCurrentValue(i, tempConvValue);
				self.turnLedToCurrentValue[i] = tempConvValue;
			}
			break;
		case cmd_currentLimit:
			for (i = cell_1; i < cell_num; i++) {
				tempConvValue = (uint16_t)hostData.data[j++] << 8;
				tempConvValue |= (uint16_t)hostData.data[j++];
				Cell_setCurrentLimit(i, tempConvValue);
				self.currentLimit[i] = tempConvValue;
			}
			break;
			/////////////////////////////////////////////////////////

		case cmd_currentP1_2_m:
			setParam_2(&hostData, 1, Cell_setParam_currentP1Multiply);
			break;
		case cmd_currentP1_2_d:
			setParam_2(&hostData, 1, Cell_setParam_currentP1Divide);
			break;
		case cmd_currentP2_2_m:
			setParam_2(&hostData, 1, Cell_setParam_currentP2Multiply);
			break;
		case cmd_currentP2_2_d:
			setParam_2(&hostData, 1, Cell_setParam_currentP2Divide);
			break;
		case cmd_currentP3_2_p:
			setParam_2(&hostData, 1, Cell_setParam_currentP3Plus);
			break;
		case cmd_currentP3_2_m:
			setParam_2(&hostData, 1, Cell_setParam_currentP3Minus);
			break;

		case cmd_currentP1_3_m:
			setParam_2(&hostData, 2, Cell_setParam_currentP1Multiply);
			break;
		case cmd_currentP1_3_d:
			setParam_2(&hostData, 2, Cell_setParam_currentP1Divide);
			break;
		case cmd_currentP2_3_m:
			setParam_2(&hostData, 2, Cell_setParam_currentP2Multiply);
			break;
		case cmd_currentP2_3_d:
			setParam_2(&hostData, 2, Cell_setParam_currentP2Divide);
			break;
		case cmd_currentP3_3_p:
			setParam_2(&hostData, 2, Cell_setParam_currentP3Plus);
			break;
		case cmd_currentP3_3_m:
			setParam_2(&hostData, 2, Cell_setParam_currentP3Minus);
			break;

		case cmd_currentP1_4_m:
			setParam_2(&hostData, 3, Cell_setParam_currentP1Multiply);
			break;
		case cmd_currentP1_4_d:
			setParam_2(&hostData, 3, Cell_setParam_currentP1Divide);
			break;
		case cmd_currentP2_4_m:
			setParam_2(&hostData, 3, Cell_setParam_currentP2Multiply);
			break;
		case cmd_currentP2_4_d:
			setParam_2(&hostData, 3, Cell_setParam_currentP2Divide);
			break;
		case cmd_currentP3_4_p:
			setParam_2(&hostData, 3, Cell_setParam_currentP3Plus);
			break;
		case cmd_currentP3_4_m:
			setParam_2(&hostData, 3, Cell_setParam_currentP3Minus);
			break;

			/////////////////////////////////////////////////////////
		case cmd_current_cut_point_1:
			setParam_2(&hostData, 0, Cell_setParam_currentCutPoint);
			break;
		case cmd_current_cut_point_2:
			setParam_2(&hostData, 1, Cell_setParam_currentCutPoint);
			break;
		case cmd_current_cut_point_3:
			setParam_2(&hostData, 2, Cell_setParam_currentCutPoint);
			break;
			/////////////////////////////////////////////////////////

		case cmd_flash_lock:		//not use (only infineon)
			break;

		default:
			break;

	}
}

static void setResponse(uint16_t (*getFunc)(uint8_t)) {
	uint8_t i, j = 0;
	uint16_t tempConvValue;
	for (i = cell_1; i < cell_num; i++) {
		tempConvValue = getFunc(i);
		aTxBuffer[j++] = (uint8_t)(tempConvValue >> 8);
		aTxBuffer[j++] = (uint8_t)(tempConvValue);
	}
}

static void setResponse_2(uint8_t par_n, uint16_t (*getFunc)(uint8_t, uint8_t)) {
	uint8_t i, j = 0;
	uint16_t tempConvValue;
	for (i = cell_1; i < cell_num; i++) {
		tempConvValue = getFunc(i, par_n);
		aTxBuffer[j++] = (uint8_t)(tempConvValue >> 8);
		aTxBuffer[j++] = (uint8_t)(tempConvValue);
	}
}

void hostToSendDataSet(void) {
	uint8_t i, j = 0;
	uint16_t tempConvValue;

	Cell_updateMeasure();

	switch(self.responseDataType) {
		case responseDataType_current:
			if (Cell_currentUpdated() == false) break;
			Cell_setFalseCurrentUpdated();
			setResponse(Cell_current);

			break;
		case responseDataType_voltage:
			if (Cell_voltageUpdated() == false) break;
			Cell_setFalseVoltageUpdated();
			setResponse(Cell_voltage);

			break;
		case responseDataType_oldBalacing:
			if (Cell_currentUpdated() == false) break;
			Cell_setFalseCurrentUpdated();
			for (i = cell_1; i < cell_num; i++) {
				tempConvValue = Cell_current(i);
				if (tempConvValue < self.turnLedToCurrentValue[i]) continue;
				j |= (1 << i);
			}
			host_sendDataSet(0, ~(j << 4));
			for (i = 1; i < 8; i++) host_sendDataSet(i, 0xFF);
			break;
			///////////////////////////////////////////////////////////////
			setResponse(Cell_param_CellOutMultiply);
			break;
		case responseDataType_cellOutScale_m:
			setResponse(Cell_param_CellOutMultiply);
			break;

		case responseDataType_cellOutScale_d:
			setResponse(Cell_param_CellOutDivide);
			break;
		case responseDataType_cellOutOffset_p:
			setResponse(Cell_param_CellOutPlus);
			break;
		case responseDataType_cellOutOffset_m:
			setResponse(Cell_param_CellOutMinus);
			break;

			///////////////////////////////////////////////////////////////
		case responseDataType_voltageScale_m:
			setResponse(Cell_param_voltageMultiply);
			break;
		case responseDataType_voltageScale_d:
			setResponse(Cell_param_voltageDivide);
			break;
		case responseDataType_voltageOffset_p:
			setResponse(Cell_param_voltagePlus);
			break;
		case responseDataType_voltageOffset_m:
			setResponse(Cell_param_voltageMinus);
			break;

			///////////////////////////////////////////////////////////////

		case responseDataType_currentP1_1_m:
			setResponse_2(0, Cell_param_currentP1Multiply);
			break;
		case responseDataType_currentP1_1_d:
			setResponse_2(0, Cell_param_currentP1Divide);
			break;
		case responseDataType_currentP2_1_m:
			setResponse_2(0, Cell_param_currentP2Multiply);
			break;
		case responseDataType_currentP2_1_d:
			setResponse_2(0, Cell_param_currentP2Divide);
			break;
		case responseDataType_currentP3_1_p:
			setResponse_2(0, Cell_param_currentP3Plus);
			break;
		case responseDataType_currentP3_1_m:
			setResponse_2(0, Cell_param_currentP3Minus);
			break;

			///////////////////////////////////////////////////////////////
		case responseDataType_cellOut:
			setResponse(Cell_cellOutRawValue);
			break;
		case responseDataType_turnLedToCurrentValue:
			setResponse(Cell_turnLedToCurrentValue);
			break;
		case responseDataType_currentLimit:
			setResponse(Cell_currentLimit);
			break;
		case responseDataType_adcState:		//not use
//			for (i = cell_1; i < cell_num; i++) {
//				tempConvValue = adc_state(i) + 0xa0;
//				host_sendDataSet(j++, (uint8_t)(tempConvValue >> 8));
//				host_sendDataSet(j++, (uint8_t)(tempConvValue));
//			}
			break;

			///////////////////////////////////////////////////////////////

		case responseDataType_currentP1_2_m:
			setResponse_2(1, Cell_param_currentP1Multiply);
			break;
		case responseDataType_currentP1_2_d:
			setResponse_2(1, Cell_param_currentP1Divide);
			break;
		case responseDataType_currentP2_2_m:
			setResponse_2(1, Cell_param_currentP2Multiply);
			break;
		case responseDataType_currentP2_2_d:
			setResponse_2(1, Cell_param_currentP2Divide);
			break;
		case responseDataType_currentP3_2_p:
			setResponse_2(1, Cell_param_currentP3Plus);
			break;
		case responseDataType_currentP3_2_m:
			setResponse_2(1, Cell_param_currentP3Minus);
			break;

		case responseDataType_currentP1_3_m:
			setResponse_2(2, Cell_param_currentP1Multiply);
			break;
		case responseDataType_currentP1_3_d:
			setResponse_2(2, Cell_param_currentP1Divide);
			break;
		case responseDataType_currentP2_3_m:
			setResponse_2(2, Cell_param_currentP2Multiply);
			break;
		case responseDataType_currentP2_3_d:
			setResponse_2(2, Cell_param_currentP2Divide);
			break;
		case responseDataType_currentP3_3_p:
			setResponse_2(2, Cell_param_currentP3Plus);
			break;
		case responseDataType_currentP3_3_m:
			setResponse_2(2, Cell_param_currentP3Minus);
			break;

		case responseDataType_currentP1_4_m:
			setResponse_2(3, Cell_param_currentP1Multiply);
			break;
		case responseDataType_currentP1_4_d:
			setResponse_2(3, Cell_param_currentP1Divide);
			break;
		case responseDataType_currentP2_4_m:
			setResponse_2(3, Cell_param_currentP2Multiply);
			break;
		case responseDataType_currentP2_4_d:
			setResponse_2(3, Cell_param_currentP2Divide);
			break;
		case responseDataType_currentP3_4_p:
			setResponse_2(3, Cell_param_currentP3Plus);
			break;
		case responseDataType_currentP3_4_m:
			setResponse_2(3, Cell_param_currentP3Minus);
			break;

			///////////////////////////////////////////////////////////////
		case responseDataType_current_cut_point_1:
			setResponse_2(0, Cell_param_currentCutPoint);
			break;
		case responseDataType_current_cut_point_2:
			setResponse_2(1, Cell_param_currentCutPoint);
			break;
		case responseDataType_current_cut_point_3:
			setResponse_2(2, Cell_param_currentCutPoint);
			break;
			///////////////////////////////////////////////////////////////

		case responseDataType_flash_lock:		//not use (only infineon)

			break;

		default:
			break;

	}
}

static void chip_lock(void) {
    FLASH_OBProgramInitTypeDef read_ob = {
        .WRPArea = -1,
        .PCROPConfig = -1,
        .SecBank = -1,
    };
    HAL_FLASHEx_OBGetConfig(&read_ob);

    if (read_ob.RDPLevel == OB_RDP_LEVEL_1) return;

    assert(HAL_FLASH_Unlock() == HAL_OK);
    assert(HAL_FLASH_OB_Unlock() == HAL_OK);
    assert(FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE) == HAL_OK);

    FLASH_OBProgramInitTypeDef write_ob = {
        .OptionType = OPTIONBYTE_RDP,
        .RDPLevel = OB_RDP_LEVEL_1,
    };
    assert(HAL_FLASHEx_OBProgram(&write_ob) == HAL_OK);
    assert(HAL_FLASH_OB_Launch() == HAL_OK);

    assert(HAL_FLASH_OB_Lock() == HAL_OK);
    assert(HAL_FLASH_Lock() == HAL_OK);
}

void app_init(void) {
	/* USER CODE END 0 */

	/* External variables --------------------------------------------------------*/

	/* USER CODE BEGIN EV */

	cell_init();

//    LL_IWDG_ReloadCounter(IWDG);
//    chip_lock();

//    LL_IWDG_ReloadCounter(IWDG);

	self.responseDataType = responseDataType_oldBalacing;

	uint8_t i;
	for(i = cell_1; i < cell_num; i++) {
		self.currentLimit[i] = Cell_currentLimit(i);
		self.turnLedToCurrentValue[i] = Cell_turnLedToCurrentValue(i);
	}

}

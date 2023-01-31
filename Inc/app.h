/*
 * app.h
 *
 *  Created on: 2022. 11. 8.
 *      Author: NB70
 */

#ifndef APP_H_
#define APP_H_

enum cmd_list {
	cmd_setDac,	// 0
	cmd_setScale_m,
	cmd_setScale_d,
	cmd_setOffset_p,
	cmd_setOffset_m,
	// 4

	cmd_flashWrite,
	cmd_adcReadOn,
	cmd_adcReadOff,
	cmd_reset,
	cmd_dummy,
	cmd_selectResponse,		// 10

	cmd_voltageScale_m,
	cmd_voltageScale_d,
	cmd_voltageOffset_p,
	cmd_voltageOffset_m,	// 14

	cmd_currentP1_1_m,
	cmd_currentP1_1_d,
	cmd_currentP2_1_m,
	cmd_currentP2_1_d,
	cmd_currentP3_1_p,
	cmd_currentP3_1_m,		// 20

	cmd_turnLedToCurrentValue,
	cmd_currentLimit,		// 22

	cmd_currentP1_2_m,
	cmd_currentP1_2_d,
	cmd_currentP2_2_m,
	cmd_currentP2_2_d,
	cmd_currentP3_2_p,
	cmd_currentP3_2_m,		// 28

	cmd_currentP1_3_m,
	cmd_currentP1_3_d,
	cmd_currentP2_3_m,
	cmd_currentP2_3_d,
	cmd_currentP3_3_p,
	cmd_currentP3_3_m,		// 34

	cmd_currentP1_4_m,
	cmd_currentP1_4_d,
	cmd_currentP2_4_m,
	cmd_currentP2_4_d,
	cmd_currentP3_4_p,
	cmd_currentP3_4_m,		// 40

	cmd_current_cut_point_1,
	cmd_current_cut_point_2,
	cmd_current_cut_point_3,	// 43

	cmd_flash_lock,			// 44
} ;

enum {
	responseDataType_current,
	responseDataType_voltage,
	responseDataType_oldBalacing,
	// 2

	responseDataType_cellOutScale_m,
	responseDataType_cellOutScale_d,
	responseDataType_cellOutOffset_p,
	responseDataType_cellOutOffset_m,
	// 6

	responseDataType_voltageScale_m,
	responseDataType_voltageScale_d,
	responseDataType_voltageOffset_p,
	responseDataType_voltageOffset_m,
	// 10

	responseDataType_currentP1_1_m,
	responseDataType_currentP1_1_d,
	responseDataType_currentP2_1_m,
	responseDataType_currentP2_1_d,
	responseDataType_currentP3_1_p,
	responseDataType_currentP3_1_m,
	// 16

	responseDataType_cellOut,
	responseDataType_turnLedToCurrentValue,
	responseDataType_currentLimit,
	// 19

	responseDataType_adcState,
	// 20

	responseDataType_currentP1_2_m,
	responseDataType_currentP1_2_d,
	responseDataType_currentP2_2_m,
	responseDataType_currentP2_2_d,
	responseDataType_currentP3_2_p,
	responseDataType_currentP3_2_m,
	// 26

	responseDataType_currentP1_3_m,
	responseDataType_currentP1_3_d,
	responseDataType_currentP2_3_m,
	responseDataType_currentP2_3_d,
	responseDataType_currentP3_3_p,
	responseDataType_currentP3_3_m,
	// 32

	responseDataType_currentP1_4_m,
	responseDataType_currentP1_4_d,
	responseDataType_currentP2_4_m,
	responseDataType_currentP2_4_d,
	responseDataType_currentP3_4_p,
	responseDataType_currentP3_4_m,
	// 38

	responseDataType_current_cut_point_1,
	responseDataType_current_cut_point_2,
	responseDataType_current_cut_point_3,
	// 41

	responseDataType_flash_lock,
	// 42
};

typedef struct _hostData_t {
	uint8_t cmd;
	uint8_t data[8];
} hostData_t;

hostData_t host_getData(void);

//extern bool responseFlag;

extern void currentLimitControl(void);
extern void balancingLedControl(void);

extern void commandProcessing(void);
extern void hostToSendDataSet();

extern void app_init(void);

#endif /* APP_H_ */

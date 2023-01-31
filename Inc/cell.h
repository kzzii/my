/*
 * cell.h
 *
 *  Created on: 2022. 11. 8.
 *      Author: NB70
 */

#ifndef CELL_H_
#define CELL_H_


/*----------------------------------------------------------------------------*/

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

enum { cell_1 = 0, cell_2 = 1, cell_3 = 2, cell_4 = 3, cell_num = 4, };
enum { cell_adc_voltage = 0, cell_adc_current = 1,};


/*----------------------------------------------------------------------------*/

#define CELL1 0
#define CELL2 1
#define CELL3 2
#define CELL4 3

//#define CELL_LEN 2
#define CELL_LEN 4

#define FLAOT_TO_U16_SACLE 10000.0f

#define DAC_CELL1_CELLOUT 0
#define DAC_CELL2_CELLOUT 1

#define ADC_CELL1_CURRENT 0
#define ADC_CELL2_CURRENT 1
#define ADC_CELL1_VOLTAGE 2
#define ADC_CELL2_VOLTAGE 3

#define ADC_SUM_BUF_LEN 4
#define ADC_SUM_BUF_MASK 0x3

#define DAC_CELLOUT_SCALE (0xffff / 5.0f)
#define ADC_CURRENT_SCALE (2.5f / 0xffff)
#define ADC_VOLTAGE_SCALE (5.0f / 0xffff)

#define CELLOUT_U16_MAX 50000

/*----------------------------------------------------------------------------*/

extern void cell_init(void);
extern void cell_irq_100us_timer(void);
extern void cell_irq_timeout(void);
extern void cell_irq_spi(void);

void  Cell_setCellOutRawValue(uint8_t, uint16_t);
void Cell_updateCellOut(void);

extern void setDefaultParameter(void);

extern uint8_t updateState;

extern uint16_t Cell_cellOutRawValue(uint8_t index);

extern void Cell_cellOutCalibration(void);

extern void Cell_setParam_CellOutMultiply(uint8_t index, uint16_t value);
extern void Cell_setParam_CellOutDivide(uint8_t index, uint16_t value);
extern void Cell_setParam_CellOutPlus(uint8_t index, uint16_t value);
extern void Cell_setParam_CellOutMinus(uint8_t index, uint16_t value);

extern uint16_t Cell_param_CellOutMultiply(uint8_t index);
extern uint16_t Cell_param_CellOutDivide(  uint8_t index);
extern uint16_t Cell_param_CellOutPlus(    uint8_t index);
extern uint16_t Cell_param_CellOutMinus(   uint8_t index);
/////////////////////////////////

extern void Cell_setFalseCurrentUpdated(void);
extern void Cell_setFalseVoltageUpdated(void);

extern uint16_t Cell_current(uint8_t index);
extern uint16_t Cell_voltage(uint8_t index);

extern void Cell_updateMeasure(void);
extern bool Cell_currentUpdated(void);
extern bool Cell_voltageUpdated(void);

//////////////////////////////////////////
extern void  Cell_setParam_voltageMultiply(uint8_t index, uint16_t value);
extern void  Cell_setParam_voltageDivide(  uint8_t index, uint16_t value);
extern void  Cell_setParam_voltagePlus(    uint8_t index, uint16_t value);
extern void  Cell_setParam_voltageMinus(   uint8_t index, uint16_t value);

extern uint16_t Cell_param_voltageMultiply(uint8_t index);
extern uint16_t Cell_param_voltageDivide(  uint8_t index);
extern uint16_t Cell_param_voltagePlus(    uint8_t index);
extern uint16_t Cell_param_voltageMinus(   uint8_t index);

extern void Cell_writeParam(void);

////////////////////////////////////////

extern void  Cell_setParam_currentP1Multiply(uint8_t cell_n, uint8_t par_n, uint16_t value);
extern void  Cell_setParam_currentP1Divide(  uint8_t cell_n, uint8_t par_n, uint16_t value);
extern void  Cell_setParam_currentP2Multiply(uint8_t cell_n, uint8_t par_n, uint16_t value);
extern void  Cell_setParam_currentP2Divide(  uint8_t cell_n, uint8_t par_n, uint16_t value);
extern void  Cell_setParam_currentP3Plus(    uint8_t cell_n, uint8_t par_n, uint16_t value);
extern void  Cell_setParam_currentP3Minus(   uint8_t cell_n, uint8_t par_n, uint16_t value);

extern uint16_t Cell_param_currentP1Multiply(uint8_t cell_n, uint8_t par_n);
extern uint16_t Cell_param_currentP1Divide(  uint8_t cell_n, uint8_t par_n);
extern uint16_t Cell_param_currentP2Multiply(uint8_t cell_n, uint8_t par_n);
extern uint16_t Cell_param_currentP2Divide(  uint8_t cell_n, uint8_t par_n);
extern uint16_t Cell_param_currentP3Plus(    uint8_t cell_n, uint8_t par_n);
extern uint16_t Cell_param_currentP3Minus(   uint8_t cell_n, uint8_t par_n);

/////////////////////////////
extern void Cell_setTurnLedToCurrentValue(uint8_t index, uint16_t value);
extern uint16_t Cell_turnLedToCurrentValue(uint8_t index);
extern void  Cell_setCurrentLimit(uint8_t index, uint16_t value);
extern uint16_t Cell_currentLimit(uint8_t index);

extern void Cell_setParam_currentCutPoint(uint8_t cell_n, uint8_t index, uint16_t value);
extern uint16_t Cell_param_currentCutPoint(uint8_t cell_n, uint8_t index);


#endif /* CELL_H_ */

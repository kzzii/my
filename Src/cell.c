/*
 * cell.c
 *
 *  Created on: 2022. 11. 8.
 *      Author: NB70
 */
#include "cell.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "main.h"
#include "param.h"

static bool phase = false;
static bool spi_swap;

static size_t prev_adc_count;
/*----------------------------------------------------------------------------*/

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


static int32_t sum_current_value[CELL_LEN][ADC_SUM_BUF_LEN];
static int32_t sum_voltage_value[CELL_LEN][ADC_SUM_BUF_LEN];

static size_t sum_len[ADC_SUM_BUF_LEN];
static size_t sum_index;

static uint16_t dac_cellout_value[CELL_LEN];
static uint16_t adc_current_value[CELL_LEN];
static uint16_t adc_voltage_value[CELL_LEN];

static size_t adc_sample_len;
static uint32_t adc_update_count;

enum {
	update_none,
	update_current,
	update_voltage,
	update_do,
};

enum {
	measurementMode_unknown,
	measurementMode_currentOnly,
	measurementMode_all,
};

/*--------------------------------------------------------------------------*/
typedef struct _Cell_t {
	uint16_t cellOutValue[4];
	uint16_t currentValue[4];
	uint16_t voltageValue[4];

	uint16_t cellOut_scale_m[4];
	uint16_t cellOut_scale_d[4];
	uint16_t cellOut_offset_p[4];
	uint16_t cellOut_offset_m[4];

	uint16_t voltage_scale_m[4];
	uint16_t voltage_scale_d[4];
	uint16_t voltage_offset_p[4];
	uint16_t voltage_offset_m[4];

	uint16_t current_p1_m[4][4];
	uint16_t current_p1_d[4][4];
	uint16_t current_p2_m[4][4];
	uint16_t current_p2_d[4][4];
	uint16_t current_p3_p[4][4];
	uint16_t current_p3_m[4][4];

	uint16_t current_cut_point[4][3];

	uint16_t turnLedToCurrentValue[4];
	uint16_t currentLimit[4];

	uint8_t updateState;
	uint8_t updateTarget;

	bool currentValueUpdated;
	bool voltageValueUpdated;

	uint8_t measurementMode;

} Cell_t;
static Cell_t self;

/*--------------------------------------------------------------------------*/
void setDefaultParameter(void) {
	uint8_t i;
	uint16_t j = 0;
	for (i = 0; i < 4; i++) {
		self.cellOut_scale_m[i] = 1000;
		self.cellOut_scale_d[i] = 1000;
		self.cellOut_offset_p[i] = 0;
		self.cellOut_offset_m[i] = 0;

		self.voltage_scale_m[i] = 1000;
		self.voltage_scale_d[i] = 1000;
		self.voltage_offset_p[i] = 0;
		self.voltage_offset_m[i] = 0;

		self.current_p1_m[i][0] = 1000;
		self.current_p1_d[i][0] = 1000;
		self.current_p2_m[i][0] = 0;
		self.current_p2_d[i][0] = 0;
		self.current_p3_p[i][0] = 0;
		self.current_p3_m[i][0] = 0;

		self.turnLedToCurrentValue[i] = 2620;
		self.currentLimit[i] = 13100;

		self.current_p1_m[i][1] = 1000;
		self.current_p1_d[i][1] = 1000;
		self.current_p2_m[i][1] = 0;
		self.current_p2_d[i][1] = 0;
		self.current_p3_p[i][1] = 0;
		self.current_p3_m[i][1] = 0;

		self.current_p1_m[i][2] = 1000;
		self.current_p1_d[i][2] = 1000;
		self.current_p2_m[i][2] = 0;
		self.current_p2_d[i][2] = 0;
		self.current_p3_p[i][2] = 0;
		self.current_p3_m[i][2] = 0;

		self.current_p1_m[i][3] = 1000;
		self.current_p1_d[i][3] = 1000;
		self.current_p2_m[i][3] = 0;
		self.current_p2_d[i][3] = 0;
		self.current_p3_p[i][3] = 0;
		self.current_p3_m[i][3] = 0;

		self.current_cut_point[i][0] = 0xffff;
		self.current_cut_point[i][1] = 0xffff;
		self.current_cut_point[i][2] = 0xffff;

		calData_setValue(j++, self.cellOut_scale_m[i]);
		calData_setValue(j++, self.cellOut_scale_d[i]);
		calData_setValue(j++, self.cellOut_offset_p[i]);
		calData_setValue(j++, self.cellOut_offset_m[i]);

		calData_setValue(j++, self.voltage_scale_m[i]);
		calData_setValue(j++, self.voltage_scale_d[i]);
		calData_setValue(j++, self.voltage_offset_p[i]);
		calData_setValue(j++, self.voltage_offset_m[i]);

		calData_setValue(j++, self.current_p1_m[i][0]);
		calData_setValue(j++, self.current_p1_d[i][0]);
		calData_setValue(j++, self.current_p2_m[i][0]);
		calData_setValue(j++, self.current_p2_d[i][0]);
		calData_setValue(j++, self.current_p3_p[i][0]);
		calData_setValue(j++, self.current_p3_m[i][0]);

		calData_setValue(j++, self.turnLedToCurrentValue[i]);
		//calData_setValue(j++, self.currentLimit[i]);

		calData_setValue(j++, self.current_p1_m[i][1]);
		calData_setValue(j++, self.current_p1_d[i][1]);
		calData_setValue(j++, self.current_p2_m[i][1]);
		calData_setValue(j++, self.current_p2_d[i][1]);
		calData_setValue(j++, self.current_p3_p[i][1]);
		calData_setValue(j++, self.current_p3_m[i][1]);

		calData_setValue(j++, self.current_p1_m[i][2]);
		calData_setValue(j++, self.current_p1_d[i][2]);
		calData_setValue(j++, self.current_p2_m[i][2]);
		calData_setValue(j++, self.current_p2_d[i][2]);
		calData_setValue(j++, self.current_p3_p[i][2]);
		calData_setValue(j++, self.current_p3_m[i][2]);

		calData_setValue(j++, self.current_p1_m[i][3]);
		calData_setValue(j++, self.current_p1_d[i][3]);
		calData_setValue(j++, self.current_p2_m[i][3]);
		calData_setValue(j++, self.current_p2_d[i][3]);
		calData_setValue(j++, self.current_p3_p[i][3]);
		calData_setValue(j++, self.current_p3_m[i][3]);

		calData_setValue(j++, self.current_cut_point[i][0]);
		calData_setValue(j++, self.current_cut_point[i][1]);
		calData_setValue(j++, self.current_cut_point[i][2]);
	}
}

static void loadParameter(void) {
	param_load();


	uint8_t i;
	uint16_t j = 0;
	for (i = 0; i < 4; i++) {
		self.cellOut_scale_m[i]  = calData_value(j++);
		self.cellOut_scale_d[i]  = calData_value(j++);
		self.cellOut_offset_p[i] = calData_value(j++);
		self.cellOut_offset_m[i] = calData_value(j++);

		self.voltage_scale_m[i]  = calData_value(j++);
		self.voltage_scale_d[i]  = calData_value(j++);
		self.voltage_offset_p[i] = calData_value(j++);
		self.voltage_offset_m[i] = calData_value(j++);

		self.current_p1_m[i][0] = calData_value(j++);
		self.current_p1_d[i][0] = calData_value(j++);
		self.current_p2_m[i][0] = calData_value(j++);
		self.current_p2_d[i][0] = calData_value(j++);
		self.current_p3_p[i][0] = calData_value(j++);
		self.current_p3_m[i][0] = calData_value(j++);

		self.turnLedToCurrentValue[i] = calData_value(j++);
		//self.currentLimit[i] = calData_value(j++);

		self.current_p1_m[i][1] = calData_value(j++);
		self.current_p1_d[i][1] = calData_value(j++);
		self.current_p2_m[i][1] = calData_value(j++);
		self.current_p2_d[i][1] = calData_value(j++);
		self.current_p3_p[i][1] = calData_value(j++);
		self.current_p3_m[i][1] = calData_value(j++);

		self.current_p1_m[i][2] = calData_value(j++);
		self.current_p1_d[i][2] = calData_value(j++);
		self.current_p2_m[i][2] = calData_value(j++);
		self.current_p2_d[i][2] = calData_value(j++);
		self.current_p3_p[i][2] = calData_value(j++);
		self.current_p3_m[i][2] = calData_value(j++);

		self.current_p1_m[i][3] = calData_value(j++);
		self.current_p1_d[i][3] = calData_value(j++);
		self.current_p2_m[i][3] = calData_value(j++);
		self.current_p2_d[i][3] = calData_value(j++);
		self.current_p3_p[i][3] = calData_value(j++);
		self.current_p3_m[i][3] = calData_value(j++);

		self.current_cut_point[i][0] = calData_value(j++);
		self.current_cut_point[i][1] = calData_value(j++);
		self.current_cut_point[i][2] = calData_value(j++);
	}
}

/*----------------------------------------------------------------------------*/
static void adc_sum(void) {

	if(spi_swap){
		sum_current_value[CELL1][sum_index] += adc_current_value[CELL1];
		sum_voltage_value[CELL1][sum_index] += adc_voltage_value[CELL1];

		sum_current_value[CELL3][sum_index] += adc_current_value[CELL3];
		sum_voltage_value[CELL3][sum_index] += adc_voltage_value[CELL3];

		spi_swap = false;
	}
    else{
    	sum_current_value[CELL2][sum_index] += adc_current_value[CELL2];
    	sum_voltage_value[CELL2][sum_index] += adc_voltage_value[CELL2];

    	sum_current_value[CELL4][sum_index] += adc_current_value[CELL4];
    	sum_voltage_value[CELL4][sum_index] += adc_voltage_value[CELL4];

    	spi_swap = true;
    	sum_len[sum_index]++;
    }


    if (sum_len[sum_index] < adc_sample_len) return;

    sum_index = (sum_index + 1) & ADC_SUM_BUF_MASK;

    for(uint8_t i=0;i<CELL_LEN;i++){
        sum_current_value[i][sum_index] = 0;
        sum_voltage_value[i][sum_index] = 0;
    }

    sum_len[sum_index] = 0;
    adc_update_count++;
}

static size_t get_sum_index(void) {
    size_t prev_sum_idx = (sum_index - 1) & ADC_SUM_BUF_MASK;
    return prev_sum_idx;
}

static uint16_t get_adc_current_mean_index(const uint8_t ch,
                                        const size_t sum_idx) {
    int32_t sum = sum_current_value[ch][sum_idx];
    size_t len = sum_len[sum_idx];
    uint16_t mean = (uint16_t)(sum / len);
    return mean;
}

static uint16_t get_adc_voltage_mean_index(const uint8_t ch,
                                        const size_t sum_idx) {
    int32_t sum = sum_voltage_value[ch][sum_idx];
    size_t len = sum_len[sum_idx];
    uint16_t mean = (uint16_t)(sum / len);
    return mean;
}


static void spi_phase1(void) {			// chip select control
	if(spi_swap){
		HW_SPI_CELL1_En();
		HW_SPI_CELL3_En();

		HW_SPI1_CS1_LOW();
		HW_SPI2_CS1_LOW();
	}
	else{
		HW_SPI_CELL2_En();
		HW_SPI_CELL4_En();

		HW_SPI1_CS2_LOW();
		HW_SPI2_CS2_LOW();
	}

    uint16_t curr_t = TIM6->CNT;

    TIM6->ARR = curr_t + 9;

    LL_TIM_ClearFlag_UPDATE(TIM6);
    LL_TIM_EnableIT_UPDATE(TIM6);

    LL_TIM_DisableIT_UPDATE(TIM7);
}

static void spi_phase2(void) {			// DAC set
    LL_SPI_TransmitData16(SPI1, 0x00);
    LL_SPI_TransmitData16(SPI2, 0x00);

    if(spi_swap){
    	LL_SPI_TransmitData16(SPI1, dac_cellout_value[CELL1]);
    	LL_SPI_TransmitData16(SPI2, dac_cellout_value[CELL3]);
    }
    else{
    	LL_SPI_TransmitData16(SPI1, dac_cellout_value[CELL2]);
    	LL_SPI_TransmitData16(SPI2, dac_cellout_value[CELL4]);
    }

    LL_TIM_DisableIT_UPDATE(TIM6);
    LL_SPI_EnableIT_RXNE(SPI2);

    phase = true;
}

static void spi_phase3(void) {			//ADC receive
	if(spi_swap){
		adc_current_value[CELL1] = LL_SPI_ReceiveData16(SPI1);
		adc_current_value[CELL3] = LL_SPI_ReceiveData16(SPI2);
	}
	else{
		adc_current_value[CELL2] = LL_SPI_ReceiveData16(SPI1);
		adc_current_value[CELL4] = LL_SPI_ReceiveData16(SPI2);
	}

    phase = false;
}

static void spi_phase4(void) {			// CS high ADC sum
    if(spi_swap){
		adc_voltage_value[CELL1] = LL_SPI_ReceiveData16(SPI1);
    	adc_voltage_value[CELL3] = LL_SPI_ReceiveData16(SPI2);

    	HW_SPI1_CS1_HIGH();
		HW_SPI2_CS1_HIGH();

    }
    else{
		adc_voltage_value[CELL2] = LL_SPI_ReceiveData16(SPI1);
        adc_voltage_value[CELL4] = LL_SPI_ReceiveData16(SPI2);

    	HW_SPI1_CS2_HIGH();
        HW_SPI2_CS2_HIGH();

    }

	adc_sum();

    LL_TIM_EnableIT_UPDATE(TIM7);
    LL_SPI_DisableIT_RXNE(SPI2);



}
static void setCellOutToZero(void) {
	uint8_t i;
	for (i = 0; i < cell_num; i++) {
		Cell_setCellOutRawValue(i, 0x0000);
		self.cellOutValue[i] = 0;
		self.currentValue[i] = 0;
		self.voltageValue[i] = 0;
	}
}

void cell_init(void) {
	size_t i, j;
    for (i = 0; i < CELL_LEN; i++) {
        for (j = 0; j < ADC_SUM_BUF_LEN; j++) {
            sum_current_value[i][j] = 0;
            sum_voltage_value[i][j] = 0;
        }
        dac_cellout_value[i] = 0;
        adc_current_value[i] = 0;
        adc_voltage_value[i] = 0;
    }
    for (j = 0; j < ADC_SUM_BUF_LEN; j++) {
        sum_len[j] = 0;
    }
	setCellOutToZero();

    setDefaultParameter();

	loadParameter();

    sum_index = 0;
    phase = false;
    spi_swap = true;
    HW_SPI_CELL1_En()

    adc_sample_len = 10;
    adc_update_count = 0;

    prev_adc_count = 0;

	self.updateState = update_none;
	self.updateTarget = update_none;

	self.currentValueUpdated = false;
	self.voltageValueUpdated = false;

	self.measurementMode = measurementMode_all;

    LL_TIM_ClearFlag_UPDATE(TIM7);
    LL_TIM_EnableIT_UPDATE(TIM7);
    LL_TIM_EnableCounter(TIM7);

    LL_TIM_EnableCounter(TIM6);
    LL_SPI_Enable(SPI1);
    LL_SPI_Enable(SPI2);

	/* Enable RXNE  Interrupt             */
	LL_SPI_EnableIT_RXNE(SPI3);
}

void cell_irq_100us_timer(void) {
    LL_TIM_ClearFlag_UPDATE(TIM7);
    spi_phase1();
}

void cell_irq_timeout(void) {
    LL_TIM_ClearFlag_UPDATE(TIM6);
    spi_phase2();
}

void cell_irq_spi(void) {
    if (phase) {
        spi_phase3();
    } else {
        spi_phase4();
    }
}


void dac_setRawValue(uint8_t index, uint16_t value) {
	dac_cellout_value[index] = value;
}

void Cell_updateCellOut(void) {
	uint8_t i;
	for (i = 0; i < cell_num; i++) dac_setRawValue(i,self.cellOutValue[i]);
}


void  Cell_setCellOutRawValue(uint8_t index, uint16_t value) {
	self.cellOutValue[index] = value;
}
////////////////////////////////////

uint16_t Cell_cellOutRawValue(uint8_t index) {
	return self.cellOutValue[index];
}

static void calibration(uint16_t *targetValue,
		uint16_t *scale_m, uint16_t *scale_d,
		uint16_t *offset_p, uint16_t *offset_m) {
	uint8_t i;
	int32_t tempValue;
	for (i = 0; i < cell_num; i++) {
		tempValue = (int32_t)targetValue[i];
		if (scale_m[i] != scale_d[i]) {
			tempValue *= (int32_t)scale_m[i];
			tempValue /= (int32_t)scale_d[i];
		}
		if (offset_p[i] - offset_m[i]) {
			tempValue += offset_p[i];
			tempValue -= offset_m[i];
		}

		if (tempValue > 0xffff) tempValue = 0xffff;
		else if (tempValue < 0) tempValue = 0;

		targetValue[i] = (uint16_t)tempValue;
	}
}

//static uint16_t calibration_single(uint8_t idx) {
//	int32_t tempValue;
//
//	tempValue = (int32_t)self.voltageValue[idx];
//	if (self.voltage_scale_m[idx] != self.voltage_scale_d[idx]) {
//		tempValue *= (int32_t)self.voltage_scale_m[idx];
//		tempValue /= (int32_t)self.voltage_scale_d[idx];
//	}
//	if (self.voltage_offset_p[idx] - self.voltage_offset_m[idx]) {
//		tempValue += self.voltage_offset_p[idx];
//		tempValue -= self.voltage_offset_m[idx];
//	}
//
//	if (tempValue > 0xffff) tempValue = 0xffff;
//	else if (tempValue < 0) tempValue = 0;
//
//	return (uint16_t)tempValue;
//
//}



void Cell_voltageCalibration(void) {
#if 0
	uint8_t i;
	int32_t tempCellValue;
	for (i = 0; i < cell_num; i++) {
		tempCellValue = (int32_t)self.voltageValue[i];
		tempCellValue *= (int32_t)self.voltage_scale_m[i];
		tempCellValue /= (int32_t)self.voltage_scale_d[i];
		tempCellValue += self.voltage_offset_p[i];
		tempCellValue -= self.voltage_offset_m[i];

		if (tempCellValue > 0xffff) tempCellValue = 0xffff;
		else if (tempCellValue < 0) tempCellValue = 0;

		self.voltageValue[i] = (uint16_t)tempCellValue;
	}
#endif
	calibration(self.voltageValue,
			self.voltage_scale_m, self.voltage_scale_d,
			self.voltage_offset_p, self.voltage_offset_m);
}



static void currentCalc(uint8_t cell_n, uint8_t par_n) {
	int32_t tempValue;
	int32_t sum;
	sum = 0;
	tempValue = (int32_t)self.currentValue[cell_n];
	if (self.current_p1_m[cell_n][par_n] != self.current_p1_d[cell_n][par_n]) {
		tempValue *= (int32_t)self.current_p1_m[cell_n][par_n];
		tempValue /= (int32_t)self.current_p1_d[cell_n][par_n];
	}
	sum += tempValue;

	if (self.current_p2_m[cell_n][par_n] != self.current_p2_d[cell_n][par_n]) {
		tempValue = (int32_t)self.cellOutValue[cell_n];
		tempValue *= (int32_t)self.current_p2_m[cell_n][par_n];
		tempValue /= (int32_t)self.current_p2_d[cell_n][par_n];
	} else {
		tempValue = 0;
	}
	sum -= tempValue;

	if (self.current_p3_p[cell_n][par_n] - self.current_p3_m[cell_n][par_n]) {
		sum += self.current_p3_p[cell_n][par_n];
		sum -= self.current_p3_m[cell_n][par_n];
	}

	if (sum > 0xffff) sum = 0xffff;
	else if (sum < 0) sum = 0;

	self.currentValue[cell_n] = (uint16_t)sum;
}

void Cell_currentCalibration(void) {
	uint8_t i;
	for (i = 0; i < cell_num; i++) {
		if (self.current_cut_point[i][0] >= self.currentValue[i]) currentCalc(i, 0);
		else if (self.current_cut_point[i][1] >= self.currentValue[i]) currentCalc(i, 1);
		else if (self.current_cut_point[i][2] >= self.currentValue[i]) currentCalc(i, 2);
		else currentCalc(i, 3);
	}
}

void Cell_currentCalibration_single(uint8_t idx) {
	if (self.current_cut_point[idx][0] >= self.currentValue[idx]) currentCalc(idx, 0);
	else if (self.current_cut_point[idx][1] >= self.currentValue[idx]) currentCalc(idx, 1);
	else if (self.current_cut_point[idx][2] >= self.currentValue[idx]) currentCalc(idx, 2);
	else currentCalc(idx, 3);
}

////////////////////////////////////////////////
void Cell_cellOutCalibration(void) {
	calibration(self.cellOutValue,
			self.cellOut_scale_m, self.cellOut_scale_d,
			self.cellOut_offset_p, self.cellOut_offset_m);
}

void  Cell_setParam_CellOutMultiply(uint8_t index, uint16_t value) {
	self.cellOut_scale_m[index] = value;
}
void  Cell_setParam_CellOutDivide(uint8_t index, uint16_t value) {
	self.cellOut_scale_d[index] = value;
}
void  Cell_setParam_CellOutPlus(uint8_t index, uint16_t value) {
	self.cellOut_offset_p[index] = value;
}
void  Cell_setParam_CellOutMinus(uint8_t index, uint16_t value) {
	self.cellOut_offset_m[index] = value;
}

uint16_t Cell_param_CellOutMultiply(uint8_t index) {
	return self.cellOut_scale_m[index];
}
uint16_t Cell_param_CellOutDivide(  uint8_t index) {
	return self.cellOut_scale_d[index];
}
uint16_t Cell_param_CellOutPlus(    uint8_t index) {
	return self.cellOut_offset_p[index];
}
uint16_t Cell_param_CellOutMinus(   uint8_t index) {
	return self.cellOut_offset_m[index];
}

void Cell_writeParam(void) {
	uint8_t i;
	uint16_t j = 0;
	for (i = 0; i < 4; i++) {
		calData_setValue(j++, self.cellOut_scale_m[i]);
		calData_setValue(j++, self.cellOut_scale_d[i]);
		calData_setValue(j++, self.cellOut_offset_p[i]);
		calData_setValue(j++, self.cellOut_offset_m[i]);
		//4

		calData_setValue(j++, self.voltage_scale_m[i]);
		calData_setValue(j++, self.voltage_scale_d[i]);
		calData_setValue(j++, self.voltage_offset_p[i]);
		calData_setValue(j++, self.voltage_offset_m[i]);
		// 8

		calData_setValue(j++, self.current_p1_m[i][0]);
		calData_setValue(j++, self.current_p1_d[i][0]);
		calData_setValue(j++, self.current_p2_m[i][0]);
		calData_setValue(j++, self.current_p2_d[i][0]);
		calData_setValue(j++, self.current_p3_p[i][0]);
		calData_setValue(j++, self.current_p3_m[i][0]);
		// 14

		calData_setValue(j++, self.turnLedToCurrentValue[i]);
		//calData_setValue(j++, self.currentLimit[i]);
		// 15

		calData_setValue(j++, self.current_p1_m[i][1]);
		calData_setValue(j++, self.current_p1_d[i][1]);
		calData_setValue(j++, self.current_p2_m[i][1]);
		calData_setValue(j++, self.current_p2_d[i][1]);
		calData_setValue(j++, self.current_p3_p[i][1]);
		calData_setValue(j++, self.current_p3_m[i][1]);
		// 21

		calData_setValue(j++, self.current_p1_m[i][2]);
		calData_setValue(j++, self.current_p1_d[i][2]);
		calData_setValue(j++, self.current_p2_m[i][2]);
		calData_setValue(j++, self.current_p2_d[i][2]);
		calData_setValue(j++, self.current_p3_p[i][2]);
		calData_setValue(j++, self.current_p3_m[i][2]);
		// 27

		calData_setValue(j++, self.current_p1_m[i][3]);
		calData_setValue(j++, self.current_p1_d[i][3]);
		calData_setValue(j++, self.current_p2_m[i][3]);
		calData_setValue(j++, self.current_p2_d[i][3]);
		calData_setValue(j++, self.current_p3_p[i][3]);
		calData_setValue(j++, self.current_p3_m[i][3]);
		// 33

		calData_setValue(j++, self.current_cut_point[i][0]);
		calData_setValue(j++, self.current_cut_point[i][1]);
		calData_setValue(j++, self.current_cut_point[i][2]);
		// 36
	}
	param_save();
}

/////////////////////////////////////////////////////

void adc_curr_value(void) {
	size_t sum_idx = get_sum_index();
	uint8_t i;

	for (i = 0; i < cell_num; i++){
		self.currentValue[i] = get_adc_current_mean_index(i,sum_idx);
	}

	Cell_currentCalibration();

}

void adc_vol_value(void) {
    size_t sum_idx = get_sum_index();
	uint8_t i;

	for (i = 0; i < cell_num; i++){
		self.voltageValue[i] = get_adc_voltage_mean_index(i,sum_idx);
	}

	Cell_voltageCalibration();
}

void Cell_setFalseCurrentUpdated(void) {
	self.currentValueUpdated = false;
}

void Cell_setFalseVoltageUpdated(void) {
	self.voltageValueUpdated = false;
}

uint16_t Cell_current(uint8_t index) {
	return self.currentValue[index];
}

uint16_t Cell_voltage(uint8_t index) {
	return self.voltageValue[index];
}

size_t cell_adc_update_count(void) {
    return adc_update_count;
}

static void setUpdateTarget(uint8_t target) {
	if (self.updateState !=  update_none) return;

	self.updateState = update_do;
	self.updateTarget = target;

	switch(self.updateTarget) {
		case update_voltage:
//			adc_setChannel(cell_adc_voltage);
			break;
		case update_current:
//			adc_setChannel(cell_adc_current);
			break;
		default:
			break;
	}
}


void updateMeasure(void) {
    size_t adc_count = cell_adc_update_count();
    if (adc_count == prev_adc_count) return;
    prev_adc_count = adc_count;

//    uint8_t i;
	switch(self.updateTarget) {
		case update_current:
			self.currentValueUpdated = true;
			adc_curr_value();
			break;
		case update_voltage:
			self.voltageValueUpdated = true;
			adc_vol_value();
			break;
		default:
			break;
	}

	self.updateState = update_none;

	switch(self.measurementMode) {
		case measurementMode_currentOnly:
			setUpdateTarget(update_current);
			break;
		case measurementMode_all:
			if (self.updateTarget == update_voltage) setUpdateTarget(update_current);
			else setUpdateTarget(update_voltage);
			break;
		default:
			break;
	}

}

void Cell_updateMeasure(void) {
	updateMeasure();
}

bool Cell_currentUpdated(void) {
	return self.currentValueUpdated;
}

bool Cell_voltageUpdated(void) {
	if (self.measurementMode != measurementMode_all) {
		self.measurementMode = measurementMode_all;
	}
	return self.voltageValueUpdated;
}


///////////////////////////////////////////////////

void  Cell_setParam_voltageMultiply(uint8_t index, uint16_t value) {
	self.voltage_scale_m[index] = value;
}
void  Cell_setParam_voltageDivide(  uint8_t index, uint16_t value) {
	self.voltage_scale_d[index] = value;
}
void  Cell_setParam_voltagePlus(    uint8_t index, uint16_t value) {
	self.voltage_offset_p[index] = value;
}
void  Cell_setParam_voltageMinus(   uint8_t index, uint16_t value) {
	self.voltage_offset_m[index] = value;
}

uint16_t Cell_param_voltageMultiply(uint8_t index) {
	return self.voltage_scale_m[index];
}
uint16_t Cell_param_voltageDivide(uint8_t index) {
	return self.voltage_scale_d[index];
}
uint16_t Cell_param_voltagePlus(uint8_t index) {
	return self.voltage_offset_p[index];
}
uint16_t Cell_param_voltageMinus(uint8_t index) {
	return self.voltage_offset_m[index];
}

/////////////////////////////////////////////////


void  Cell_setParam_currentP1Multiply(uint8_t cell_n, uint8_t par_n, uint16_t value) {
	self.current_p1_m[cell_n][par_n] = value;
}
void  Cell_setParam_currentP1Divide(  uint8_t cell_n, uint8_t par_n, uint16_t value) {
	self.current_p1_d[cell_n][par_n] = value;
}
void  Cell_setParam_currentP2Multiply(uint8_t cell_n, uint8_t par_n, uint16_t value) {
	self.current_p2_m[cell_n][par_n] = value;
}
void  Cell_setParam_currentP2Divide(  uint8_t cell_n, uint8_t par_n, uint16_t value) {
	self.current_p2_d[cell_n][par_n] = value;
}
void  Cell_setParam_currentP3Plus(    uint8_t cell_n, uint8_t par_n, uint16_t value) {
	self.current_p3_p[cell_n][par_n] = value;
}
void  Cell_setParam_currentP3Minus(   uint8_t cell_n, uint8_t par_n, uint16_t value) {
	self.current_p3_m[cell_n][par_n] = value;
}

uint16_t Cell_param_currentP1Multiply(uint8_t cell_n, uint8_t par_n) {
	return self.current_p1_m[cell_n][par_n];
}
uint16_t Cell_param_currentP1Divide(  uint8_t cell_n, uint8_t par_n) {
	return self.current_p1_d[cell_n][par_n];
}
uint16_t Cell_param_currentP2Multiply(uint8_t cell_n, uint8_t par_n) {
	return self.current_p2_m[cell_n][par_n];
}
uint16_t Cell_param_currentP2Divide(  uint8_t cell_n, uint8_t par_n) {
	return self.current_p2_d[cell_n][par_n];
}
uint16_t Cell_param_currentP3Plus(    uint8_t cell_n, uint8_t par_n) {
	return self.current_p3_p[cell_n][par_n];
}
uint16_t Cell_param_currentP3Minus(   uint8_t cell_n, uint8_t par_n) {
	return self.current_p3_m[cell_n][par_n];
}


/*--------------------------------------------------------------------------*/
void  Cell_setTurnLedToCurrentValue(uint8_t index, uint16_t value) {
	self.turnLedToCurrentValue[index] = value;
}
uint16_t Cell_turnLedToCurrentValue(uint8_t index) {
	return self.turnLedToCurrentValue[index];
}

void  Cell_setCurrentLimit(uint8_t index, uint16_t value) {
	self.currentLimit[index] = value;
}
uint16_t Cell_currentLimit(uint8_t index) {
	return self.currentLimit[index];
}

/*--------------------------------------------------------------------------*/

void Cell_setParam_currentCutPoint(uint8_t cell_n, uint8_t index, uint16_t value) {
	self.current_cut_point[cell_n][index] = value;
}
uint16_t Cell_param_currentCutPoint(uint8_t cell_n, uint8_t index) {
	return self.current_cut_point[cell_n][index];
}



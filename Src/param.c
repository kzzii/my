#include "param.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "main.h"
#include "param.h"
#include "cell.h"
#include "stm32g4xx_hal_flash.h"

/*----------------------------------------------------------------------------*/

//#define PARAM_FLASH_ADDR 0x0807f800		//ADDR_FLASH_PAGE_255
#define PARAM_FLASH_PAGE 127
//end address : PARAM_FLASH_PAGE + FLASH_PAGE_SIZE-1
#define PARAM_VER 1
#define PARAM_INDEX_LEN 144
/*----------------------------------------------------------------------------*/

typedef struct {
    uint32_t crc;
    uint32_t param_ver;
    uint32_t param[PARAM_INDEX_LEN];
} flash_data_t;

static uint32_t param_ram[PARAM_INDEX_LEN];

/*----------------------------------------------------------------------------*/

static bool param_exist_flag = false;
static bool init_done = false;

/*----------------------------------------------------------------------------*/
//flash_data_t test_load;
static bool load(void) {
    flash_data_t flash_data = *(flash_data_t *)(uintptr_t)BANK2_FLASH_PAGE_127;
//    memcpy(test_load.param, &flash_data.param, sizeof(uint32_t) * PARAM_INDEX_LEN);

    if (flash_data.param_ver != PARAM_VER) {
    	setDefaultParameter();
        return false;
    }

    uint32_t crc;
    size_t i;

    LL_CRC_ResetCRCCalculationUnit(CRC);
	for (i = 0; i < sizeof(uint32_t) * PARAM_INDEX_LEN; i++) {
		LL_CRC_FeedData8(CRC, *((uint8_t *)&flash_data.param + i));
	}

    crc = LL_CRC_ReadData32(CRC);

    if (flash_data.crc != crc) {
    	setDefaultParameter();
        return false;
    }

    memcpy(param_ram, &flash_data.param, sizeof(uint32_t) * PARAM_INDEX_LEN);
    return true;
}

static void erase(void) {
    FLASH_EraseInitTypeDef erase_init = {0};
    uint32_t page_error = 0;

    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Banks = FLASH_BANK_2;
    erase_init.Page = PARAM_FLASH_PAGE;
    erase_init.NbPages = 1;

    assert(HAL_FLASH_Unlock() == HAL_OK);

    assert(HAL_FLASHEx_Erase(&erase_init, &page_error) == HAL_OK);

    assert(HAL_FLASH_Lock() == HAL_OK);
}

static void save(void) {
    FLASH_EraseInitTypeDef erase_init = {0};
    uint32_t page_error = 0;
    size_t i;
    size_t size;
    uint64_t data;
    flash_data_t flash_data = {0};
    const size_t flash_data_size = sizeof(flash_data_t);
    uint8_t *src_addr;
    uint32_t dst_addr;

    memcpy(flash_data.param, param_ram, sizeof(uint32_t) * PARAM_INDEX_LEN);
    flash_data.param_ver = PARAM_VER;

    LL_CRC_ResetCRCCalculationUnit(CRC);
	for (i = 0; i < sizeof(uint32_t) * PARAM_INDEX_LEN; i++) {
		LL_CRC_FeedData8(CRC, *((uint8_t *)&flash_data.param + i));
	}
    flash_data.crc = LL_CRC_ReadData32(CRC);

    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Banks = FLASH_BANK_2;
    erase_init.Page = PARAM_FLASH_PAGE;
    erase_init.NbPages = 1;

    assert(HAL_FLASH_Unlock() == HAL_OK);

    assert(HAL_FLASHEx_Erase(&erase_init, &page_error) == HAL_OK);

    for (i = 0; i < flash_data_size; i += 8) {
        size = flash_data_size - i;
        if (size > 8) size = 8;

        src_addr = (uint8_t *)&flash_data + i;
        dst_addr = BANK2_FLASH_PAGE_127 + i;

        data = 0;
        memcpy(&data, (uint8_t *)src_addr, size);
        assert(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, dst_addr,
                                 data) == HAL_OK);
    }

    assert(HAL_FLASH_Lock() == HAL_OK);
}

/*----------------------------------------------------------------------------*/

void param_init(void) {
    param_load();
    init_done = true;
}

bool param_exist(void) {
    assert(init_done);
    return param_exist_flag;
}


void calData_setValue(uint16_t index, uint16_t value) {
	param_ram[index] = (uint32_t)value;
}

uint16_t calData_value(uint16_t index) {
	return (uint16_t)param_ram[index];
}

//uint32_t param_get_param(const size_t index) {
//    assert(init_done);
//    return param_ram[index];
//}
//
//void param_set_param(const size_t index, uint32_t value) {
//    param_ram[index] = value;
//}

//float param_get_param_float(const size_t index) {
//    assert(init_done);
//    float par;
//    memcpy(&par, &param_ram[index], sizeof(float));
//    return par;
//}

void param_load(void) {
    param_exist_flag = load();
}

void param_save(void) {
//    assert(init_done);
    save();
}

void param_erase(void) {
    erase();
}

void param_load_default(void) {
	setDefaultParameter();
}

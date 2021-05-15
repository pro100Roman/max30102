/**
 * @file        max30102.h
 *
 * @date        27 March 2021
 * @author      Serhii
 * @brief
 */

#ifndef MAX30102_H
#define MAX30102_H

#include <stdint.h>

#include "max30102_regTypes.h"
#include "max30102_regMap.h"
#include "max30102_registers.h"

#warning delete
//#include "i2c.h"
#include "stm32l0xx_hal.h"

typedef enum {
    STATUS_OK = 0,
    STATUS_FAILED
} oprStatus_t;

#define MAX30102_ADDR                       (0x57 << 1)
#define MAX30102_SENSE_BUFF_SIZE            30u

typedef struct {
    uint8_t head;
    uint8_t tail;
    uint32_t red[MAX30102_SENSE_BUFF_SIZE];
    uint32_t iRed[MAX30102_SENSE_BUFF_SIZE];

    int32_t spo2;
    int8_t spo2Valid;
    int32_t heartRate;
    int8_t heartRateValid;
} max30102_data_t;

typedef struct {
    void*               _interfaceHandle;
    uint8_t             (*_interfaceTransmite)(I2C_HandleTypeDef*, uint16_t , uint8_t*, uint16_t , uint32_t);
    uint8_t             (*_interfaceReceive)  (I2C_HandleTypeDef*, uint16_t , uint8_t*, uint16_t , uint32_t);
    void                (*_delayMs)(uint32_t);

    uint8_t             addr;
    max30102_data_t     data;

} max30102_t;

#define max30102_ctor(_name, _ih, _itx, _irx, _delay) max30102_t _name = { \
                ._interfaceHandle       = (void*)&_ih,                          \
                ._interfaceTransmite    = &_itx,                                \
                ._interfaceReceive      = &_irx,                                \
                ._delayMs               = &_delay,                              \
                .addr                   = MAX30102_ADDR,                        \
}

uint8_t max30102_init(max30102_t *me);
uint8_t max30102_calHeartRateAndOxygen(max30102_t *me);

#endif /* MAX30102_H */

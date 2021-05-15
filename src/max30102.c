/**
 * @file        max30102.c
 *
 * @date        27 March 2021
 * @author      Serhii
 * @brief
 */

#include <stdbool.h>
#include "max30102.h"
#warning delete
#include "main.h"

#define MAX30102_INTERFACE_TIMEOUT              100u    //mS
#define MAX30102_I2C_MAX_BUFF_SIZE              100u
#define MAX30102_LEDS_CHANNEL_DATA_SIZE         6u      //bytes
#define MAX30102_SAMPLE_MEM_BANK                32u     //samples
#define MAX30102_FIFO_RESET                     0x00u
#define MAX30102_FIFO_FULL_COUNTER              0x0Fu
#define MAX30102_FIFO_CFG_FIFO_ROL_LOVER_EN     0x00u
#define MAX30102_LED_PULSE_AMPLITUDE            0x24u
#define MAX30102_PILOT_PULSE_AMPLITUDE          0x7fu
#define MAX30102_ASSERT(expr)                   assert_param(expr)
#define MAX30102_SAMPL_FREQ                     25u     // hz
#define MAX30102_SAMPL_TIME                     4u      // seconds
#define MAX30102_BUFF_SIZE                      (MAX30102_SAMPL_FREQ * MAX30102_SAMPL_TIME)
#define MAX30102_BUFF_SIZE_MA4                  (MAX30102_BUFF_SIZE - 4u)
#define MAX30102_THRESHOLD_MIN                  30u
#define MAX30102_THRESHOLD_MAX                  60u
#define min(x,y) ((x) < (y) ? (x) : (y))

static uint8_t max30102_powerOn(max30102_t *me);
static uint8_t max30102_powerOff(max30102_t *me);
static uint8_t max30102_reset(max30102_t *me);
static uint8_t max30102_intrEnable(max30102_t *me);
static uint8_t max30102_getFifoReadPointer(max30102_t *me, uint8_t *ptrFifo);
static uint8_t max30102_getFifoWritePointer(max30102_t *me, uint8_t *ptrFifo);
static uint8_t max30102_setFifoWritePointer(max30102_t *me, uint8_t ptrFifo);
static uint8_t max30102_setFifoReadPointer(max30102_t *me, uint8_t ptrFifo);
static uint8_t max30102_setFifoOvrCounter(max30102_t *me, uint8_t count);
static uint8_t max30102_readFifoData(max30102_t *me, uint32_t *pRedBuff, uint32_t *pIrBuff);
static uint8_t max30102_resetFifo(max30102_t *me);
static uint8_t max30102_fifoConfig(max30102_t *me, uint8_t fifoFull, uint8_t rollover, uint8_t sampleAveraging);
static uint8_t max30102_modeConfig(max30102_t *me, uint8_t mode);
static uint8_t max30102_spo2Config(max30102_t *me, uint8_t ledPower, uint8_t sampleRate, uint8_t adcRange);
static uint8_t max30102_setLedPulseAmplitude(max30102_t *me, uint8_t ledPulseAmplitude, uint8_t pilotPulseAmplitude);
static uint8_t max30102_readRegs(max30102_t *me, uint8_t reg, uint8_t *data, uint8_t count);
static uint8_t max30102_writeRegs(max30102_t *me, uint8_t reg, const uint8_t *data, uint8_t count);
static uint8_t readByteToReg(max30102_t *me, uint8_t reg_addr, uint8_t *data);
static uint8_t writeByteToReg(max30102_t *me, uint8_t reg_addr, uint8_t data);
static uint8_t writeByteToRegWithCheck(max30102_t *me, uint8_t reg_addr, uint8_t data);
static uint8_t max30102_getNewFifoData(max30102_t *me);
static uint8_t max30102_heartRateAndOxygenSaturation(max30102_t *me, uint32_t *pRedBuff,
        uint32_t *pIrBuff, const uint32_t buffLength);
static void max30102_findPeaks(int32_t *pLocs, int32_t *nNpks, int32_t *pIr, int32_t size,
        int32_t minHeight, int32_t minDistance, int32_t maxNum );
static void max30102_peaksAboveMinHeight(int32_t *pLocs, int32_t *nNpks, int32_t *pIr,
        int32_t size, int32_t minHeight );
static void max30102_removeClosePeaks(int32_t *pLocs, int32_t *pnNpks, int32_t *pIr, int32_t minDistance);
static void max30102_sortIndicesDescend(int32_t  *pIr, int32_t *pIndx, int32_t size);
static void max30102_sortAscend(int32_t *pIr, int32_t size);

uint8_t max30102_init(max30102_t *me) {
    uint8_t res = 0;

    res |= max30102_powerOn(me);
    res |= max30102_reset(me);
    res |= max30102_intrEnable(me);

    res |= max30102_resetFifo(me);

    res |= max30102_fifoConfig(me, MAX30102_FIFO_FULL_COUNTER, MAX30102_FIFO_CFG_FIFO_ROL_LOVER_EN,
            MAX30102_FIFO_CFG_REG_SMP_AVE_NO_4);
    res |= max30102_modeConfig(me, MAX30102_MODE_CFG_REG_MODE_HR_SPO2);
    res |= max30102_spo2Config(me, MAX30102_SPO2_CFG_REG_LED_PW_ADC_18_BITS,
            MAX30102_SPO2_CFG_REG_SPO2_SR_SMP_100, MAX30102_SPO2_CFG_REG_SPO2_ADC_RGE_4096);

    res |= max30102_setLedPulseAmplitude(me, MAX30102_LED_PULSE_AMPLITUDE, MAX30102_PILOT_PULSE_AMPLITUDE);

    MAX30102_ASSERT(res);
    return res;
}

static uint8_t max30102_powerOn(max30102_t *me) {
    uint8_t temp;
    uint8_t res = 0;

    res |= readByteToReg(me, MAX30102_MODE_CFG_REG, &temp);
    temp &= ~MAX30102_MODE_CFG_REG_SHDN;
    res |= writeByteToReg(me, MAX30102_MODE_CFG_REG, temp);
    return res;
}

static uint8_t max30102_powerOff(max30102_t *me) {
    uint8_t temp;
    uint8_t res = 0;

    res |= readByteToReg(me, MAX30102_MODE_CFG_REG, &temp);
    temp |= MAX30102_MODE_CFG_REG_SHDN;
    res |= writeByteToReg(me, MAX30102_MODE_CFG_REG, temp);
    return res;
}

static uint8_t max30102_reset(max30102_t *me) {
    uint8_t temp;
    uint8_t res = 0;

    res |= readByteToReg(me, MAX30102_MODE_CFG_REG, &temp);
    temp |= MAX30102_MODE_CFG_REG_RESET;
    res |= writeByteToReg(me, MAX30102_MODE_CFG_REG, temp);

    do {
        me->_delayMs(1);
        res |= readByteToReg(me, MAX30102_MODE_CFG_REG, &temp);
    } while((temp & MAX30102_MODE_CFG_REG_RESET) != 0);
    return res;
}

static uint8_t max30102_intrEnable(max30102_t *me) {
    uint8_t res = 0;
    uint8_t temp;

    temp = MAX30102_INT_EN1_REG_A_FULL_EN | MAX30102_INT_EN1_REG_PPG_RDY_EN;
    res |= writeByteToRegWithCheck(me, MAX30102_INT_EN1_REG, temp);

    temp = 0;
    res |= writeByteToRegWithCheck(me, MAX30102_INT_EN2_REG, temp);
    return res;
}
static uint8_t max30102_getFifoWritePointer(max30102_t *me, uint8_t *ptrFifo) {
    return readByteToReg(me, MAX30102_FIFO_WR_PTR_REG, ptrFifo);
}

static uint8_t max30102_getFifoReadPointer(max30102_t *me, uint8_t *ptrFifo) {
    return readByteToReg(me, MAX30102_FIFO_RD_PTR_REG, ptrFifo);
}

static uint8_t max30102_setFifoWritePointer(max30102_t *me, uint8_t ptrFifo) {
    uint8_t temp = ptrFifo & MAX30102_FIFO_WR_PTR_REG_MASK;
    return writeByteToRegWithCheck(me, MAX30102_FIFO_WR_PTR_REG, temp);
}

static uint8_t max30102_setFifoReadPointer(max30102_t *me, uint8_t ptrFifo) {
    uint8_t temp = ptrFifo & MAX30102_FIFO_RD_PTR_REG_MASK;
    return writeByteToRegWithCheck(me, MAX30102_FIFO_RD_PTR_REG, temp);
}

static uint8_t max30102_setFifoOvrCounter(max30102_t *me, uint8_t count) {
    uint8_t temp = count & MAX30102_OVR_CTR_REG_MASK;
    return writeByteToRegWithCheck(me, MAX30102_OVR_CTR_REG, temp);
}

static uint8_t max30102_readFifoData(max30102_t *me, uint32_t *pRedBuff, uint32_t *pIrBuff) {
    uint8_t res = 0;
    uint8_t dataTemp[MAX30102_LEDS_CHANNEL_DATA_SIZE];

    res |= max30102_readRegs(me, MAX30102_FIFO_DATA_REG, dataTemp, MAX30102_LEDS_CHANNEL_DATA_SIZE);

    *pRedBuff = (((uint32_t)dataTemp[0]) << 16
            | ((uint32_t)dataTemp[1]) << 8 | dataTemp[2]) & 0x3ffff;

    *pIrBuff = (((uint32_t)dataTemp[3]) << 16
            | ((uint32_t)dataTemp[4]) << 8 | dataTemp[5]) & 0x3ffff;

    return res;
}

static uint8_t max30102_resetFifo(max30102_t *me) {
    uint8_t res = 0;

    me->data.head = 0;
    me->data.tail = 0;

    res |= max30102_setFifoWritePointer(me, MAX30102_FIFO_RESET);
    res |= max30102_setFifoOvrCounter(me, MAX30102_FIFO_RESET);
    res |= max30102_setFifoReadPointer(me, MAX30102_FIFO_RESET);
    return res;
}

static uint8_t max30102_fifoConfig(max30102_t *me, uint8_t fifoFull, uint8_t rollover, uint8_t sampleAveraging) {
    uint8_t temp;

    fifoFull &= MAX30102_FIFO_CFG_REG_FIFO_A_FULL_MASK;

    rollover <<= MAX30102_FIFO_CFG_REG_FIFO_ROL_LOVER_SHIFT;
    rollover &= MAX30102_FIFO_CFG_REG_FIFO_ROL_LOVER_MASK;

    sampleAveraging <<= MAX30102_FIFO_CFG_REG_SMP_AVE_SHIFT;
    sampleAveraging &= MAX30102_FIFO_CFG_REG_SMP_AVE_LOVER_MASK;

    temp = fifoFull | rollover | sampleAveraging;
    return writeByteToRegWithCheck(me, MAX30102_FIFO_CFG_REG, temp);
}

static uint8_t max30102_modeConfig(max30102_t *me, uint8_t mode) {
    uint8_t temp = mode & MAX30102_MODE_CFG_REG_MODE_MASK;
    return writeByteToRegWithCheck(me, MAX30102_MODE_CFG_REG, temp);
}

static uint8_t max30102_spo2Config(max30102_t *me, uint8_t ledPower, uint8_t sampleRate, uint8_t adcRange) {
    uint8_t temp;

    ledPower &= MAX30102_SPO2_CFG_REG_LED_PW_MASK;
    sampleRate <<= MAX30102_SPO2_CFG_REG_SPO2_SR_SHIFT;
    sampleRate &= MAX30102_SPO2_CFG_REG_SPO2_SR_MASK;
    adcRange <<= MAX30102_SPO2_CFG_REG_SPO2_ADC_RGE_SHIFT;
    adcRange &= MAX30102_SPO2_CFG_REG_SPO2_ADC_RGE_MASK;

    temp = ledPower | sampleRate | adcRange;
    return writeByteToRegWithCheck(me, MAX30102_SPO2_CFG_REG, temp);
}

static uint8_t max30102_setLedPulseAmplitude(max30102_t *me, uint8_t ledPulseAmplitude, uint8_t pilotPulseAmplitude) {
    uint8_t res = 0;

    res |= writeByteToRegWithCheck(me, MAX30102_LED1_PULSE_AMP_REG, ledPulseAmplitude);
    res |= writeByteToRegWithCheck(me, MAX30102_LED2_PULSE_AMP_REG, ledPulseAmplitude);
    res |= writeByteToRegWithCheck(me, MAX30102_PROX_MODE_LED_PULSE_AMP_REG, pilotPulseAmplitude);
    return res;
}

static uint8_t readByteToReg(max30102_t *me, uint8_t reg_addr, uint8_t *data) {
    return max30102_readRegs(me, reg_addr, data, 1);
}

static uint8_t writeByteToReg(max30102_t *me, uint8_t reg_addr, uint8_t data) {
    return max30102_writeRegs(me, reg_addr, &data, 1);
}

static uint8_t writeByteToRegWithCheck(max30102_t *me, uint8_t reg_addr, uint8_t data) {
    uint8_t ret;
    uint8_t readData;

    ret = max30102_writeRegs(me, reg_addr, &data, 1);
    if (ret != MAX30102_OK) {
        return ret;
    }

    me->_delayMs(1);

    ret = max30102_readRegs(me, reg_addr, &readData, 1);
    if (ret != MAX30102_OK) {
        return ret;
    }

    if(readData != data)
    {
        ret = MAX30102_REG_CONT_ERROR;
    }

    return ret;
}

static uint8_t max30102_readRegs(max30102_t *me, uint8_t reg, uint8_t *data, uint8_t count) {
    uint8_t ret;
    uint8_t buff[1];

    buff[0] = reg;

    ret = me->_interfaceTransmite(me->_interfaceHandle, me->addr, buff, 1, MAX30102_INTERFACE_TIMEOUT);

    if (ret != STATUS_OK) {
        return MAX30102_I2C_FAIL_ERROR;
    }

    ret = me->_interfaceReceive(me->_interfaceHandle, me->addr, data, count, MAX30102_INTERFACE_TIMEOUT);
    if (ret != STATUS_OK) {
        return MAX30102_I2C_FAIL_ERROR;
    }

    return MAX30102_OK;
}

static uint8_t max30102_writeRegs(max30102_t *me, uint8_t reg, const uint8_t *data, uint8_t count) {
    uint8_t ret;
    uint8_t count_with_reg = count + 1;
    uint8_t buff[MAX30102_I2C_MAX_BUFF_SIZE];

    buff[0] = reg;

    for (uint8_t i = 1; i < count_with_reg; i++) {
        buff[i] = data[i - 1];
    }

    ret = me->_interfaceTransmite(me->_interfaceHandle, me->addr, buff, count_with_reg, MAX30102_INTERFACE_TIMEOUT);
    if (ret != STATUS_OK) {
        return MAX30102_I2C_FAIL_ERROR;
    }

    return MAX30102_OK;
}

/***
 *
 * Calculation of the heart rate and the oxygen
 *
 ***/

uint8_t max30102_calHeartRateAndOxygen(max30102_t *me) {
    uint8_t res = 0;
    int8_t numberOfSamples = 0;
    uint32_t irBuff[MAX30102_BUFF_SIZE];
    uint32_t redBuff[MAX30102_BUFF_SIZE];

    for (uint8_t i = 0 ; i < MAX30102_BUFF_SIZE;) {
        res |= max30102_getNewFifoData(me);

        numberOfSamples = me->data.head - me->data.tail;

        if (numberOfSamples < 0) {
            numberOfSamples += MAX30102_SENSE_BUFF_SIZE;
        }

        while(numberOfSamples > 0) {
            me->data.tail++;
            me->data.tail %= MAX30102_SENSE_BUFF_SIZE;

            redBuff[i] = me->data.red[me->data.tail];
            irBuff[i] = me->data.iRed[me->data.tail];
            i++;
            numberOfSamples--;
            if(i == MAX30102_BUFF_SIZE) break;
        }
    }

    res |= max30102_heartRateAndOxygenSaturation(me, redBuff, irBuff, MAX30102_BUFF_SIZE);

    return res;
}

static uint8_t max30102_getNewFifoData(max30102_t *me) {
    uint8_t res = 0;
    uint8_t readPointer = 0;
    uint8_t writePointer = 0;
    int32_t numberOfSamples = 0;

    res |= max30102_getFifoReadPointer(me, &readPointer);
    res |= max30102_getFifoWritePointer(me, &writePointer);

    if (readPointer != writePointer) {
        numberOfSamples = writePointer - readPointer;
        if (numberOfSamples < 0) {
            numberOfSamples += MAX30102_SAMPLE_MEM_BANK;
        }

        while (numberOfSamples > 0) {
            me->data.head++;
            me->data.head %= MAX30102_SENSE_BUFF_SIZE;
            max30102_readFifoData(me, &me->data.red[me->data.head], &me->data.iRed[me->data.head]);
            numberOfSamples--;
        }
    }

    return res;
}

static uint8_t max30102_heartRateAndOxygenSaturation(max30102_t *me, uint32_t *pRedBuff,
        uint32_t *pIrBuff, const uint32_t buffLength) {

    static int32_t tempIr[MAX30102_BUFF_SIZE];
    static int32_t tempRed[MAX30102_BUFF_SIZE];
    uint32_t irMean = 0;
    int32_t threshold1 = 0;
    int32_t irValleyLocs[15] = {0};
    int32_t nNpks;
    int32_t peakIntervalSum;
    int32_t exactIrValleyLocsCount;
    int32_t ratioAverage = 0;
    int32_t ratioCount = 0;

    int32_t middleIdx;

    int32_t redAc, irAc;
    int32_t redDcMax, irDcMax;
    int32_t redDcMaxIdx = 0;
    int32_t irDcMaxIdx = 0;
    int32_t ratio[5];
    int32_t nume, denom ;

    /* Calculates DC mean and subtract DC from ir */
    for (int i = 0; i < buffLength; i++) {
        irMean += pIrBuff[i] ;
    }
    irMean = irMean/buffLength ;

    /* Remove DC and invert signal so that we can use peak detector as valley detector */
    for (int i = 0; i < buffLength; i++) {
        tempIr[i] = irMean - pIrBuff[i];
    }

    /* 4 pt Moving Average */
    for (int i = 0; i < MAX30102_BUFF_SIZE_MA4; i++) {
        tempIr[i]=(tempIr[i] + tempIr[i+1] + tempIr[i+2]+ tempIr[i+3])/(int)4;
    }
    /* Calculate threshold */
    for (int i = 0; i < MAX30102_BUFF_SIZE_MA4; i++) {
      threshold1 +=  tempIr[i];
    }
    threshold1 =  threshold1 / MAX30102_BUFF_SIZE_MA4;
    if (threshold1 < MAX30102_THRESHOLD_MIN) {
        threshold1 = MAX30102_THRESHOLD_MIN;
    }
    else if (threshold1 > MAX30102_THRESHOLD_MAX) {
        threshold1 = MAX30102_THRESHOLD_MAX;
    }

    /* Signal was flipped, use peak detector as valley detector */
    max30102_findPeaks(irValleyLocs, &nNpks, tempIr, MAX30102_BUFF_SIZE_MA4, threshold1, 4, 15 );

    peakIntervalSum =0;
    if (nNpks >= 2) {
        for (int i = 1; i < nNpks; i++) {
            peakIntervalSum += (irValleyLocs[i] -irValleyLocs[i-1]);
        }
        peakIntervalSum = peakIntervalSum/(nNpks-1);
        me->data.heartRate = (int32_t)((MAX30102_SAMPL_FREQ*60) / peakIntervalSum);
        me->data.heartRateValid = 1;
    }
    else {
        /* Unable to calculate because # of peaks are too small */
        me->data.heartRate = -999;
        me->data.heartRateValid  = 0;
        return MAX30102_CAL_ERROR;
    }

    /* Load raw value again for SPO2 calculation : RED(=y) and IR(=X) */
    for (int i = 0; i < buffLength ; i++) {
        tempIr[i] = pIrBuff[i];
        tempRed[i] = pRedBuff[i];
    }

    /* Find precise min near irValleyLocs */
    exactIrValleyLocsCount = nNpks;

    /* Using exact_ir_valley_locs, find ir-red DC and ir-red AC for SPO2 calibration ratio
     * finding AC/DC maximum of raw */
    for (int i = 0; i < 5; i++) {
        ratio[i] = 0;
    }
    for (int i = 0; i < exactIrValleyLocsCount; i++) {
        if (irValleyLocs[i] > MAX30102_BUFF_SIZE ) {
            /* Do not use SPO2 since valley loc is out of range */
            me->data.spo2 = -999;
            me->data.spo2Valid = 0;
            return MAX30102_CAL_ERROR;
        }
    }
    /* Find max between two valley locations
     * and use ratio betwen AC compoent of Ir & Red and DC compoent of Ir & Red for SPO2 */
    for (int i = 0; i < exactIrValleyLocsCount-1; i++) {
        redDcMax= -16777216 ;
        irDcMax= -16777216;

        if (irValleyLocs[i+1] - irValleyLocs[i] > 3) {
            for (int k = irValleyLocs[i]; k < irValleyLocs[i+1]; k++) {
                if (tempIr[k] > irDcMax) {
                    irDcMax = tempIr[k];
                    irDcMaxIdx = k;
                }
                if (tempRed[k] > redDcMax) {
                    redDcMax = tempRed[k];
                    redDcMaxIdx = k;
                }
            }

            redAc = (tempRed[irValleyLocs[i+1]] - tempRed[irValleyLocs[i]]) * (redDcMaxIdx - irValleyLocs[i]);
            redAc = tempRed[irValleyLocs[i]] + redAc / (irValleyLocs[i+1] - irValleyLocs[i]);
            redAc = tempRed[redDcMaxIdx] - redAc;

            irAc = (tempIr[irValleyLocs[i+1]] - tempIr[irValleyLocs[i]]) * (irDcMaxIdx -irValleyLocs[i]);
            irAc =  tempIr[irValleyLocs[i]] + irAc / (irValleyLocs[i+1] - irValleyLocs[i]);
            irAc =  tempIr[redDcMaxIdx] - irAc;

            /* Prepare X100 to preserve floating value */
            nume = (redAc * irDcMax) >> 7 ;
            denom = (irAc *redDcMax) >> 7;

            if (denom > 0  && ratioCount < 5 &&  nume != 0) {
                ratio[ratioCount] = (nume * 100) / denom ;
                ratioCount++;
            }
        }
    }
    /* Choose median value since PPG signal may varies from beat to beat */
    max30102_sortAscend(ratio, ratioCount);
    middleIdx = ratioCount/2;

    if (middleIdx > 1) {
        ratioAverage = (ratio[middleIdx-1] + ratio[middleIdx])/2;
    }
    else {
        ratioAverage = ratio[middleIdx ];
    }

    if (ratioAverage > 2 && ratioAverage < 184) {
        //me->data.spo2 = uch_spo2_table[ratioAverage];
        me->data.spo2Valid = 1;
    }
    else {
        me->data.spo2 = -999;
        me->data.spo2Valid = 0;
        return MAX30102_CAL_ERROR;
    }

    return MAX30102_OK;
}

static void max30102_findPeaks(int32_t *pLocs, int32_t *nNpks,  int32_t *pIr, int32_t size,
        int32_t minHeight, int32_t minDistance, int32_t maxNum ) {

    max30102_peaksAboveMinHeight(pLocs, nNpks, pIr, size, minHeight);
    max30102_removeClosePeaks(pLocs, nNpks, pIr, minDistance);
    *nNpks = min(*nNpks, maxNum );
}

static void max30102_peaksAboveMinHeight(int32_t *pLocs, int32_t *nNpks, int32_t  *pIr,
        int32_t size, int32_t minHeight ) {

    int32_t i = 1;
    int32_t width;

    *nNpks = 0;

    while (i < (size - 1)) {
        /* Find left edge of potential peaks */
        if (pIr[i] > minHeight && pIr[i] > pIr[i-1]) {
            width = 1;

            /* Find flat peaks */
            while (i+width < size && pIr[i] == pIr[i+width]) {
                width++;
            }

            /* Find right edge of peaks */
            if (pIr[i] > pIr[i+width] && (*nNpks) < 15 ) {
                pLocs[(*nNpks)++] = i;

                /* For flat peaks, peak location is left edge */
                i += width+1;
            }
            else {
                i += width;
            }
        }
          else {
            i++;
          }
    }
}

static void max30102_removeClosePeaks(int32_t *pLocs, int32_t *pnNpks, int32_t *pIr, int32_t minDistance) {
    int32_t oldNpks;
    int32_t dist;

    /* Order peaks from large to small */
    max30102_sortIndicesDescend(pIr, pLocs, *pnNpks);

    for (int i = -1; i < *pnNpks; i++) {
        oldNpks = *pnNpks;
        *pnNpks = i+1;

        for (int j = i+1; j < oldNpks; j++) {
            /* lag-zero peak of autocorr is at index -1 */
            dist =  pLocs[j] - ( i == -1 ? -1 : pLocs[i] );
            if (dist > minDistance || dist < -minDistance) {
                pLocs[(*pnNpks)++] = pLocs[j];
            }
        }
    }

    /* Resort indices int32_to ascending order */
    max30102_sortAscend(pLocs, *pnNpks);
}

static void max30102_sortIndicesDescend(int32_t  *pIr, int32_t *pIndx, int32_t size) {
    int32_t j, n_temp;

    for (int i = 1; i < size; i++) {
        n_temp = pIndx[i];

        for (j = i; j > 0 && pIr[n_temp] > pIr[pIndx[j-1]]; j--) {
            pIndx[j] = pIndx[j-1];
        }
        pIndx[j] = n_temp;
    }
}

static void max30102_sortAscend(int32_t *pIr, int32_t size) {
    int32_t j, n_temp;

    for (int i = 1; i < size; i++) {
        n_temp = pIr[i];

        for (j = i; j > 0 && n_temp < pIr[j-1]; j--) {
            pIr[j] = pIr[j-1];
        }
        pIr[j] = n_temp;
    }
}

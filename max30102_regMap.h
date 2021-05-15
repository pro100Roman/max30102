/**
 * @file        max30102_regMap.h
 *
 * @date        27 March 2021
 * @author      Serhii
 * @brief
 */

#ifndef MAX30102_REGMAP_H
#define MAX30102_REGMAP_H

// Status Registers
#define MAX30102_INT_STS1_REG                   0x00u
#define MAX30102_INT_STS2_REG                   0x01u
#define MAX30102_INT_EN1_REG                    0x02u
#define MAX30102_INT_EN2_REG                    0x03u

// FIFO Registers
#define MAX30102_FIFO_WR_PTR_REG                0x04u
#define MAX30102_OVR_CTR_REG                    0x05u
#define MAX30102_FIFO_RD_PTR_REG                0x06u
#define MAX30102_FIFO_DATA_REG                  0x07u

// Configuration Registers
#define MAX30102_FIFO_CFG_REG                   0x08u
#define MAX30102_MODE_CFG_REG                   0x09u
#define MAX30102_SPO2_CFG_REG                   0x0Au
#define MAX30102_LED1_PULSE_AMP_REG             0x0Cu
#define MAX30102_LED2_PULSE_AMP_REG             0x0Du
#define MAX30102_PROX_MODE_LED_PULSE_AMP_REG    0x10u
#define MAX30102_MULTI_LED_MODE_CNTR1_REG       0x11u
#define MAX30102_MULTI_LED_MODE_CNTR2_REG       0x12u

// Die Temperature Registers
#define MAX30102_DIE_TMP_INT_REG                0x1Fu
#define MAX30102_DIE_TMP_FRAC_REG               0x20u
#define MAX30102_DIE_TMP_CFG_REG                0x21u

// Proximity Function Registers
#define MAX30102_PROX_INT_THRESH_REG            0x30u

// Part ID Registers
#define MAX30102_REVISION_ID_REG                0xFEu
#define MAX30102_PART_ID_REG                    0xFFu

#endif /* MAX30102_REGMAP_H */

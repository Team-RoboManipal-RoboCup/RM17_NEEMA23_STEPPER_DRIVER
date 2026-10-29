/**
 * @file    as5600.h
 * @brief   AS5600 12-bit Magnetic Rotary Encoder Driver for STM32F103
 *          Pinout: SCL on PB6, SDA on PB7.
 *          Register-level / robust bit-bang I2C with bus recovery.
 */

#ifndef AS5600_H
#define AS5600_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* ==================== HARDWARE CONFIGURATION ==================== */
#define AS5600_I2C_PORT         GPIOB
#define AS5600_SCL_PIN          GPIO_PIN_6    /* PB6: I2C SCL */
#define AS5600_SDA_PIN          GPIO_PIN_7    /* PB7: I2C SDA */

/* AS5600 7-bit I2C Address */
#define AS5600_I2C_ADDR_7BIT    0x36
#define AS5600_I2C_ADDR_WRITE   (AS5600_I2C_ADDR_7BIT << 1)       /* 0x6C */
#define AS5600_I2C_ADDR_READ    ((AS5600_I2C_ADDR_7BIT << 1) | 1) /* 0x6D */

/* AS5600 Registers */
#define AS5600_REG_STATUS       0x0B
#define AS5600_REG_RAW_ANGLE_H  0x0C          /* Raw Angle [11:8] */
#define AS5600_REG_RAW_ANGLE_L  0x0D          /* Raw Angle [7:0]  */
#define AS5600_REG_ANGLE_H      0x0E          /* Filtered Angle [11:8] */
#define AS5600_REG_ANGLE_L      0x0F          /* Filtered Angle [7:0]  */
#define AS5600_REG_AGC          0x1A
#define AS5600_REG_MAGNITUDE_H  0x1B
#define AS5600_REG_MAGNITUDE_L  0x1C

/* Status Register Bitfields */
#define AS5600_STATUS_MD        (1U << 5)     /* Magnet Detected */
#define AS5600_STATUS_ML        (1U << 4)     /* Magnet Too Weak (Too far) */
#define AS5600_STATUS_MH        (1U << 3)     /* Magnet Too Strong (Too close) */

/* Return Status Codes */
#define AS5600_OK               0
#define AS5600_ERR_NACK         1
#define AS5600_ERR_TIMEOUT      2
#define AS5600_ERR_BUS          3

/* ==================== PUBLIC API ==================== */

/**
 * @brief  Initializes GPIO PB6 (SCL) and PB7 (SDA) as Open-Drain outputs,
 *         executes a 9-clock bus unstick sequence, and checks communication.
 * @retval AS5600_OK if sensor detected, or error code.
 */
uint8_t AS5600_Init(void);

/**
 * @brief  Reads 12-bit RAW ANGLE from registers 0x0C (MSB) and 0x0D (LSB).
 *         Combined as ((uint16_t)msb << 8) | lsb.
 * @param  raw_angle_out: Pointer to store the 12-bit value (0 to 4095).
 * @retval AS5600_OK on success, or error code.
 */
uint8_t AS5600_ReadRawAngle(uint16_t *raw_angle_out);

/**
 * @brief  Reads AS5600 STATUS register (0x0B).
 * @param  status_out: Pointer to store status byte (MD, ML, MH bits).
 * @retval AS5600_OK on success, or error code.
 */
uint8_t AS5600_ReadStatus(uint8_t *status_out);

/**
 * @brief  Checks whether a magnet is currently detected.
 * @retval 1 if magnet detected, 0 otherwise.
 */
uint8_t AS5600_IsMagnetDetected(void);

#endif /* AS5600_H */

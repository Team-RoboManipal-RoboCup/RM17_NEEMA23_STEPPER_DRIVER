/**
 * @file    as5600.c
 * @brief   AS5600 12-bit Magnetic Rotary Encoder Driver for STM32F103
 *          Pinout: SCL on PB6, SDA on PB7.
 *          Robust, lockup-free bit-bang I2C with bus clearing and clock stretching support.
 */

#include "as5600.h"
#include "stepper_config.h"

/* ==================== LOW-LEVEL I2C BIT-BANG MACROS ==================== */
/* Open-Drain GPIO mode:
 * Writing BSRR (HIGH) lets the line float HIGH via external/internal pull-up.
 * Writing BRR  (LOW)  actively drives the line LOW (0V).
 * Reading IDR returns the physical logic state on the pin.
 */
static inline void i2c_scl_release(void) {
    AS5600_I2C_PORT->BSRR = AS5600_SCL_PIN;
}

static inline void i2c_scl_low(void) {
    AS5600_I2C_PORT->BRR = AS5600_SCL_PIN;
}

static inline void i2c_sda_release(void) {
    AS5600_I2C_PORT->BSRR = AS5600_SDA_PIN;
}

static inline void i2c_sda_low(void) {
    AS5600_I2C_PORT->BRR = AS5600_SDA_PIN;
}

static inline uint8_t i2c_sda_read(void) {
    return (AS5600_I2C_PORT->IDR & AS5600_SDA_PIN) ? 1U : 0U;
}

static inline uint8_t i2c_scl_read(void) {
    return (AS5600_I2C_PORT->IDR & AS5600_SCL_PIN) ? 1U : 0U;
}

/* Half-cycle delay (~2.5 µs for ~200 kHz I2C clock @ 72MHz) */
static inline void i2c_delay(void) {
    delay_us(2);
}

/* SCL release with clock-stretching timeout */
static uint8_t i2c_scl_float_wait(void) {
    i2c_scl_release();
    uint32_t timeout = 50; /* 50 µs max stretch timeout */
    while (!i2c_scl_read()) {
        delay_us(1);
        if (--timeout == 0) {
            return AS5600_ERR_TIMEOUT;
        }
    }
    return AS5600_OK;
}

/* ==================== I2C PROTOCOL PRIMITIVES ==================== */

static void i2c_start(void) {
    i2c_sda_release();
    i2c_scl_release();
    i2c_delay();
    i2c_sda_low();
    i2c_delay();
    i2c_scl_low();
    i2c_delay();
}

static void i2c_stop(void) {
    i2c_sda_low();
    i2c_delay();
    i2c_scl_release();
    i2c_delay();
    i2c_sda_release();
    i2c_delay();
}

/**
 * @brief  Writes 8 bits over I2C and reads ACK from slave.
 * @retval 0 if ACK received (success), 1 if NACK.
 */
static uint8_t i2c_write_byte(uint8_t byte) {
    for (uint8_t i = 0; i < 8; i++) {
        if (byte & 0x80) {
            i2c_sda_release();
        } else {
            i2c_sda_low();
        }
        byte <<= 1;
        i2c_delay();

        if (i2c_scl_float_wait() != AS5600_OK) {
            return 1; /* Timeout / error */
        }
        i2c_delay();
        i2c_scl_low();
    }

    /* Read ACK */
    i2c_sda_release();
    i2c_delay();
    if (i2c_scl_float_wait() != AS5600_OK) {
        return 1;
    }
    i2c_delay();
    uint8_t ack = i2c_sda_read(); /* 0 = ACK, 1 = NACK */
    i2c_scl_low();
    i2c_delay();

    return ack;
}

/**
 * @brief  Reads 8 bits from slave and sends ACK or NACK.
 * @param  ack: 1 to send ACK (more bytes to follow), 0 to send NACK (last byte).
 * @retval Read byte.
 */
static uint8_t i2c_read_byte(uint8_t ack) {
    uint8_t byte = 0;
    i2c_sda_release();

    for (uint8_t i = 0; i < 8; i++) {
        byte <<= 1;
        i2c_delay();
        if (i2c_scl_float_wait() != AS5600_OK) {
            return 0;
        }
        i2c_delay();
        if (i2c_sda_read()) {
            byte |= 1U;
        }
        i2c_scl_low();
    }

    /* Send ACK or NACK */
    if (ack) {
        i2c_sda_low(); /* ACK */
    } else {
        i2c_sda_release(); /* NACK */
    }
    i2c_delay();
    i2c_scl_release();
    i2c_delay();
    i2c_scl_low();
    i2c_sda_release();
    i2c_delay();

    return byte;
}

/* ==================== BUS CLEAR / UNSTICK ROUTINE ==================== */
/**
 * @brief  If MCU was reset in the middle of an I2C read, the slave might be
 *         holding SDA LOW. Clocking SCL 9 times releases the slave.
 */
static void i2c_bus_clear(void) {
    i2c_sda_release();
    i2c_scl_release();
    delay_us(10);

    for (uint8_t i = 0; i < 9; i++) {
        i2c_scl_low();
        delay_us(5);
        i2c_scl_release();
        delay_us(5);
        if (i2c_sda_read()) {
            break; /* SDA released */
        }
    }
    i2c_stop();
}

/* ==================== PUBLIC API IMPLEMENTATION ==================== */

uint8_t AS5600_Init(void) {
    /* 1. Enable GPIOB Clock */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 2. Configure PB6 (SCL) and PB7 (SDA) as Open-Drain Output with Pull-Up */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = AS5600_SCL_PIN | AS5600_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(AS5600_I2C_PORT, &GPIO_InitStruct);

    /* 3. Release both lines */
    i2c_sda_release();
    i2c_scl_release();
    delay_us(20);

    /* 4. Clear bus in case slave was left in mid-byte */
    i2c_bus_clear();

    /* 5. Verify sensor responds to address */
    i2c_start();
    uint8_t nack = i2c_write_byte(AS5600_I2C_ADDR_WRITE);
    i2c_stop();

    if (nack) {
        return AS5600_ERR_NACK;
    }

    return AS5600_OK;
}

uint8_t AS5600_ReadRawAngle(uint16_t *raw_angle_out) {
    if (!raw_angle_out) {
        return AS5600_ERR_BUS;
    }

    /* 1. Start I2C and send Address (Write) */
    i2c_start();
    if (i2c_write_byte(AS5600_I2C_ADDR_WRITE) != 0) {
        i2c_stop();
        return AS5600_ERR_NACK;
    }

    /* 2. Send Register Pointer 0x0C (Raw Angle High Byte) */
    if (i2c_write_byte(AS5600_REG_RAW_ANGLE_H) != 0) {
        i2c_stop();
        return AS5600_ERR_NACK;
    }

    /* 3. Repeated Start and send Address (Read) */
    i2c_start();
    if (i2c_write_byte(AS5600_I2C_ADDR_READ) != 0) {
        i2c_stop();
        return AS5600_ERR_NACK;
    }

    /* 4. Read High Byte from 0x0C (send ACK to auto-increment to 0x0D) */
    uint8_t msb = i2c_read_byte(1);

    /* 5. Read Low Byte from 0x0D (send NACK as last byte) */
    uint8_t lsb = i2c_read_byte(0);

    /* 6. Stop Condition */
    i2c_stop();

    /* Combine as ((uint16_t)msb << 8) | lsb. Mask msb to 4 bits [11:8] */
    *raw_angle_out = (((uint16_t)(msb & 0x0FU) << 8) | (uint16_t)lsb);

    return AS5600_OK;
}

uint8_t AS5600_ReadStatus(uint8_t *status_out) {
    if (!status_out) {
        return AS5600_ERR_BUS;
    }

    i2c_start();
    if (i2c_write_byte(AS5600_I2C_ADDR_WRITE) != 0) {
        i2c_stop();
        return AS5600_ERR_NACK;
    }

    if (i2c_write_byte(AS5600_REG_STATUS) != 0) {
        i2c_stop();
        return AS5600_ERR_NACK;
    }

    i2c_start();
    if (i2c_write_byte(AS5600_I2C_ADDR_READ) != 0) {
        i2c_stop();
        return AS5600_ERR_NACK;
    }

    *status_out = i2c_read_byte(0); /* NACK */
    i2c_stop();

    return AS5600_OK;
}

uint8_t AS5600_IsMagnetDetected(void) {
    uint8_t status = 0;
    if (AS5600_ReadStatus(&status) == AS5600_OK) {
        return (status & AS5600_STATUS_MD) ? 1U : 0U;
    }
    return 0;
}

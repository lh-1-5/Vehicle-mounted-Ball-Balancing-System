#include "at24c02.h"

#include "delay.h"
#include "ti_msp_dl_config.h"

/* 100 kHz-class software I2C timing.  Keeping the bus slow makes first-board
 * bring-up tolerant of module wiring and pull-up variations. */
#define AT24_I2C_DELAY_US         (5u)
#define AT24_ACK_POLL_RETRIES     (1200u) /* maximum about 6 ms */

/*
 * PA26/PA30 keep their digital input buffers enabled. I2C open drain is
 * emulated by enabling the low output for 0 and disabling the output for 1.
 * The EEPROM module supplies the bus pull-up resistors.
 */
static void at24_delay(void)
{
    delay_us(AT24_I2C_DELAY_US);
}

static void at24_scl_low(void)
{
    DL_GPIO_clearPins(GPIO_AT24_PORT, GPIO_AT24_SCL_PIN);
    DL_GPIO_enableOutput(GPIO_AT24_PORT, GPIO_AT24_SCL_PIN);
}

static void at24_scl_release(void)
{
    DL_GPIO_disableOutput(GPIO_AT24_PORT, GPIO_AT24_SCL_PIN);
}

static void at24_sda_low(void)
{
    DL_GPIO_clearPins(GPIO_AT24_PORT, GPIO_AT24_SDA_PIN);
    DL_GPIO_enableOutput(GPIO_AT24_PORT, GPIO_AT24_SDA_PIN);
}

static void at24_sda_release(void)
{
    DL_GPIO_disableOutput(GPIO_AT24_PORT, GPIO_AT24_SDA_PIN);
}

static uint8_t at24_sda_read(void)
{
    return (DL_GPIO_readPins(GPIO_AT24_PORT, GPIO_AT24_SDA_PIN) != 0u);
}

static AT24C02_Status at24_scl_high(void)
{
    uint16_t timeout = 100u;

    at24_scl_release();
    while ((DL_GPIO_readPins(GPIO_AT24_PORT, GPIO_AT24_SCL_PIN) == 0u)) {
        if (--timeout == 0u) {
            return AT24C02_ERR_TIMEOUT;
        }
        at24_delay();
    }
    return AT24C02_OK;
}

static AT24C02_Status at24_start(void)
{
    AT24C02_Status status;

    at24_sda_release();
    status = at24_scl_high();
    if (status != AT24C02_OK) {
        return status;
    }
    at24_delay();
    at24_sda_low();
    at24_delay();
    at24_scl_low();
    at24_delay();
    return AT24C02_OK;
}

static AT24C02_Status at24_stop(void)
{
    AT24C02_Status status;

    at24_sda_low();
    at24_delay();
    status = at24_scl_high();
    if (status != AT24C02_OK) {
        return status;
    }
    at24_delay();
    at24_sda_release();
    at24_delay();
    return AT24C02_OK;
}

static AT24C02_Status at24_write_bit(uint8_t bit)
{
    AT24C02_Status status;

    if (bit != 0u) {
        at24_sda_release();
    } else {
        at24_sda_low();
    }
    at24_delay();
    status = at24_scl_high();
    if (status != AT24C02_OK) {
        return status;
    }
    at24_delay();
    at24_scl_low();
    at24_delay();
    return AT24C02_OK;
}

static AT24C02_Status at24_read_bit(uint8_t *bit)
{
    AT24C02_Status status;

    at24_sda_release();
    at24_delay();
    status = at24_scl_high();
    if (status != AT24C02_OK) {
        return status;
    }
    at24_delay();
    *bit = at24_sda_read();
    at24_scl_low();
    at24_delay();
    return AT24C02_OK;
}

static AT24C02_Status at24_write_byte(uint8_t byte)
{
    AT24C02_Status status;
    uint8_t bit;

    for (uint8_t i = 0u; i < 8u; ++i) {
        status = at24_write_bit((uint8_t)(byte & 0x80u));
        if (status != AT24C02_OK) {
            return status;
        }
        byte <<= 1;
    }

    status = at24_read_bit(&bit);
    if (status != AT24C02_OK) {
        return status;
    }
    return (bit == 0u) ? AT24C02_OK : AT24C02_ERR_NACK;
}

static AT24C02_Status at24_read_byte(uint8_t *byte, uint8_t nack)
{
    AT24C02_Status status;
    uint8_t bit;
    uint8_t value = 0u;

    for (uint8_t i = 0u; i < 8u; ++i) {
        status = at24_read_bit(&bit);
        if (status != AT24C02_OK) {
            return status;
        }
        value = (uint8_t)((value << 1) | bit);
    }

    status = at24_write_bit(nack);
    if (status == AT24C02_OK) {
        *byte = value;
    }
    return status;
}

static AT24C02_Status at24_wait_write_complete(void)
{
    AT24C02_Status status;

    for (uint16_t i = 0u; i < AT24_ACK_POLL_RETRIES; ++i) {
        status = at24_start();
        if (status != AT24C02_OK) {
            return status;
        }
        status = at24_write_byte((uint8_t)(AT24C02_I2C_ADDRESS << 1));
        (void)at24_stop();
        if (status == AT24C02_OK) {
            return AT24C02_OK;
        }
        if (status != AT24C02_ERR_NACK) {
            return status;
        }
    }
    return AT24C02_ERR_TIMEOUT;
}

static AT24C02_Status at24_write_page(uint8_t address, const uint8_t *data,
                                       uint8_t length)
{
    AT24C02_Status status;

    status = at24_start();
    if (status == AT24C02_OK) {
        status = at24_write_byte((uint8_t)(AT24C02_I2C_ADDRESS << 1));
    }
    if (status == AT24C02_OK) {
        status = at24_write_byte(address);
    }
    for (uint8_t i = 0u; (i < length) && (status == AT24C02_OK); ++i) {
        status = at24_write_byte(data[i]);
    }
    (void)at24_stop();

    return (status == AT24C02_OK) ? at24_wait_write_complete() : status;
}

void AT24C02_Init(void)
{
    /* Keep INENA set so DIN reports the externally pulled-up bus level. */
    DL_GPIO_initDigitalInput(GPIO_AT24_SCL_IOMUX);
    DL_GPIO_initDigitalInput(GPIO_AT24_SDA_IOMUX);

    /* A low output latch makes a later enableOutput() an immediate low drive. */
    DL_GPIO_clearPins(GPIO_AT24_PORT, GPIO_AT24_SCL_PIN | GPIO_AT24_SDA_PIN);
    DL_GPIO_disableOutput(GPIO_AT24_PORT,
                          GPIO_AT24_SCL_PIN | GPIO_AT24_SDA_PIN);
}

AT24C02_Status AT24C02_WriteByte(uint8_t address, uint8_t data)
{
    return at24_write_page(address, &data, 1u);
}

AT24C02_Status AT24C02_ReadByte(uint8_t address, uint8_t *data)
{
    return AT24C02_Read(address, data, 1u);
}

AT24C02_Status AT24C02_Write(uint8_t address, const uint8_t *data,
                              uint16_t length)
{
    AT24C02_Status status;

    if ((data == 0) || (length == 0u) ||
        ((uint16_t)address + length > AT24C02_SIZE_BYTES)) {
        return AT24C02_ERR_ARG;
    }

    while (length != 0u) {
        uint8_t page_left = (uint8_t)(AT24C02_PAGE_SIZE_BYTES -
                            (address % AT24C02_PAGE_SIZE_BYTES));
        uint8_t count = (length < page_left) ? (uint8_t)length : page_left;

        status = at24_write_page(address, data, count);
        if (status != AT24C02_OK) {
            return status;
        }
        address = (uint8_t)(address + count);
        data += count;
        length -= count;
    }
    return AT24C02_OK;
}

AT24C02_Status AT24C02_Read(uint8_t address, uint8_t *data, uint16_t length)
{
    AT24C02_Status status;

    if ((data == 0) || (length == 0u) ||
        ((uint16_t)address + length > AT24C02_SIZE_BYTES)) {
        return AT24C02_ERR_ARG;
    }

    status = at24_start();
    if (status == AT24C02_OK) {
        status = at24_write_byte((uint8_t)(AT24C02_I2C_ADDRESS << 1));
    }
    if (status == AT24C02_OK) {
        status = at24_write_byte(address);
    }
    if (status == AT24C02_OK) {
        status = at24_start(); /* repeated START */
    }
    if (status == AT24C02_OK) {
        status = at24_write_byte((uint8_t)((AT24C02_I2C_ADDRESS << 1) | 1u));
    }
    for (uint16_t i = 0u; (i < length) && (status == AT24C02_OK); ++i) {
        status = at24_read_byte(&data[i], (uint8_t)(i + 1u == length));
    }
    (void)at24_stop();
    return status;
}

#ifndef AT24C02_H
#define AT24C02_H

#include <stdint.h>

/* AT24C02: 256 bytes, 8-byte page write, 7-bit I2C address 0x50 when
 * A0/A1/A2 are tied low. */
#define AT24C02_I2C_ADDRESS       (0x50u)
#define AT24C02_SIZE_BYTES        (256u)
#define AT24C02_PAGE_SIZE_BYTES   (8u)

typedef enum {
    AT24C02_OK = 0,
    AT24C02_ERR_ARG,
    AT24C02_ERR_NACK,
    AT24C02_ERR_TIMEOUT
} AT24C02_Status;

/* Call after SYSCFG_DL_init(). PA26=SCL and PA30=SDA. */
void AT24C02_Init(void);

AT24C02_Status AT24C02_WriteByte(uint8_t address, uint8_t data);
AT24C02_Status AT24C02_ReadByte(uint8_t address, uint8_t *data);

/* Buffer writes are split at the EEPROM's 8-byte page boundaries. */
AT24C02_Status AT24C02_Write(uint8_t address, const uint8_t *data,
                              uint16_t length);
AT24C02_Status AT24C02_Read(uint8_t address, uint8_t *data,
                             uint16_t length);

#endif /* AT24C02_H */

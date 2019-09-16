#ifndef _I2CM_H_
#define _I2CM_H_

#include <stdint.h>


// Процедура инициализации i2c ESP32
void i2cm_init();
// Процедура деинициализации i2c ESP32
void i2cm_deinit();
// Функция чтения массива байт по i2c ESP32
int i2cm_read(uint8_t slaveAddr, uint8_t startAddress, uint8_t *data, uint16_t nBytes);
// Функция записи массива байт по i2c ESP32
int i2cm_write(uint8_t slaveAddr, uint8_t startAddress, uint8_t *data, uint16_t nBytes);

#endif

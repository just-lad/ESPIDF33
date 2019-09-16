#include "i2cm.h"
#include <stdio.h>
#include "driver/i2c.h"



#define PIN_SCL          	22               	/*!< gpio number for I2C master clock */
#define PIN_SDA          	21               	/*!< gpio number for I2C master data  */
#define I2C_NUM             I2C_NUM_1        	/*!< I2C port number for master dev */
#define I2C_TX_BUF_DISABLE  0                	/*!< I2C master do not need buffer */
#define I2C_RX_BUF_DISABLE  0                	/*!< I2C master do not need buffer */
#define I2C_FREQ_HZ         1000000	// 400000   /*!< I2C master clock frequency */

#define WRITE_BIT           I2C_MASTER_WRITE 	/*!< I2C master write */
#define READ_BIT            I2C_MASTER_READ  	/*!< I2C master read */
#define ACK_CHECK_EN        1              		/*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS       0              		/*!< I2C master will not check ack from slave */
#define ACK_VAL             (i2c_ack_type_t) 0  /*!< I2C ack value */
#define NACK_VAL            (i2c_ack_type_t) 1  /*!< I2C nack value */


//==============================================================================
// Процедура инициализации i2c ESP32
//==============================================================================
void i2cm_init()
{
    int i2c_master_port = I2C_NUM_1;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = PIN_SDA;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = PIN_SCL;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, I2C_RX_BUF_DISABLE, I2C_TX_BUF_DISABLE, 0);
}
//==============================================================================


//==============================================================================
// Процедура деинициализации i2c ESP32
//==============================================================================
void i2cm_deinit()
{
    int i2c_master_port = I2C_NUM_1;
    i2c_driver_delete(i2c_master_port);
}
//==============================================================================


//==============================================================================
// Функция чтения массива байт по i2c ESP32
//==============================================================================
int i2cm_read(uint8_t slaveAddr, uint8_t startAddress, uint8_t *data, uint16_t nBytes)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Сначала выбираем внутренний адрес сенсора
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, slaveAddr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, startAddress, ACK_CHECK_EN);

    // Теперь читаем из сенсора
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, slaveAddr << 1 | READ_BIT, ACK_CHECK_EN);
    if (nBytes > 1)
        i2c_master_read(cmd, data, nBytes - 1, ACK_VAL);
    i2c_master_read_byte(cmd, data + nBytes - 1, NACK_VAL);
    i2c_master_stop(cmd);

    int ret = i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}
//==============================================================================


//==============================================================================
// Функция записи массива байт по i2c ESP32
//==============================================================================
int i2cm_write(uint8_t slaveAddr, uint8_t startAddress, uint8_t *data, uint16_t nBytes)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Сначала выбираем внутренний адрес сенсора
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slaveAddr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, startAddress, ACK_CHECK_EN);

    // Пишем данные
    i2c_master_write(cmd, data, nBytes, ACK_VAL);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}
//==============================================================================

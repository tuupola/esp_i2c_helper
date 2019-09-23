/*

Copyright (c) 2019 Mika Tuupola

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#ifndef _I2C_HAL_MASTER_H
#define _I2C_HAL_MASTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <esp_err.h>

typedef esp_err_t i2c_hal_err_t;

#define I2C_HAL_MASTER_NUM          (1)
#define I2C_HAL_MASTER_SDA          (21)
#define I2C_HAL_MASTER_SCL          (22)
#define I2C_HAL_MASTER_FREQ_HZ      (1000000)
#define I2C_HAL_MASTER_RX_BUF_LEN   (0)
#define I2C_HAL_MASTER_TX_BUF_LEN   (0)

i2c_hal_err_t i2c_hal_master_init();
i2c_hal_err_t i2c_hal_master_read(uint8_t address, uint8_t reg, uint8_t *buffer, uint16_t size);
i2c_hal_err_t i2c_hal_master_write(uint8_t address, uint8_t reg, uint8_t *buffer, uint16_t size);
i2c_hal_err_t i2c_hal_master_close();
//i2c_hal_err_t i2c_hal_master_ioctl(int16_t command, void *buffer);

#ifdef __cplusplus
}
#endif
#endif

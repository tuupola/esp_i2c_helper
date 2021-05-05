/*

SPDX-License-Identifier: MIT

MIT License

Copyright (c) 2021 Rop Gonggrijp. Based on esp_i2c_helper by Mika Tuupola.

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

#include <stdint.h>
#include <stddef.h>

#include <esp_log.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <driver/i2c.h>

#include "sdkconfig.h"

#include "i2c_manager.h"

static const char* TAG = I2C_TAG;

static SemaphoreHandle_t I2C_FN(_local_mutex)[2] = { NULL, NULL };
static SemaphoreHandle_t* I2C_FN(_mutex) = &I2C_FN(_local_mutex)[0];

static const uint8_t ACK_CHECK_EN = 1;

#if defined (CONFIG_I2C_MANAGER_0_ENABLED)
	#if defined (CONFIG_I2C_MANAGER_0_PULLUPS)
		#define I2C_MANAGER_0_PULLUPS 	true
	#else
		#define I2C_MANAGER_0_PULLUPS 	false
	#endif

	#define I2C_MANAGER_0_TIMEOUT 		CONFIG_I2C_MANAGER_0_TIMEOUT / portTICK_RATE_MS
	#define I2C_MANAGER_0_LOCK_TIMEOUT	CONFIG_I2C_MANAGER_0_LOCK_TIMEOUT / portTICK_RATE_MS
#endif


#if defined (CONFIG_I2C_MANAGER_1_ENABLED)
	#if defined (CONFIG_I2C_MANAGER_1_PULLUPS)
		#define I2C_MANAGER_1_PULLUPS 	true
	#else
		#define I2C_MANAGER_1_PULLUPS 	false
	#endif

	#define I2C_MANAGER_1_TIMEOUT 		CONFIG_I2C_MANAGER_1_TIMEOUT / portTICK_RATE_MS
	#define I2C_MANAGER_1_LOCK_TIMEOUT	CONFIG_I2C_MANAGER_1_LOCK_TIMEOUT / portTICK_RATE_MS
#endif

esp_err_t I2C_FN(_init)(i2c_port_t port) {

	esp_err_t ret = ESP_OK;

	if (I2C_FN(_mutex)[port] == 0) {

		ESP_LOGI(TAG, "Starting I2C master at port %d.", (int)port);

		I2C_FN(_mutex)[port] = xSemaphoreCreateMutex();

		i2c_config_t conf;

		if (port == I2C_NUM_0) {
			#if defined (CONFIG_I2C_MANAGER_0_ENABLED)
				conf.sda_io_num = CONFIG_I2C_MANAGER_0_SDA;
				conf.scl_io_num = CONFIG_I2C_MANAGER_0_SCL;
				conf.sda_pullup_en = I2C_MANAGER_0_PULLUPS ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
				conf.scl_pullup_en = conf.sda_pullup_en;
				conf.master.clk_speed = CONFIG_I2C_MANAGER_0_FREQ_HZ;
			#else
				ESP_LOGE(TAG, "I2C port 0 not configured.");
				ESP_ERROR_CHECK(ESP_FAIL);
			#endif
		} else if (port == I2C_NUM_1) {
			#if defined (CONFIG_I2C_MANAGER_1_ENABLED)
				conf.sda_io_num = CONFIG_I2C_MANAGER_1_SDA;
				conf.scl_io_num = CONFIG_I2C_MANAGER_1_SCL;
				conf.sda_pullup_en = I2C_MANAGER_1_PULLUPS ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
				conf.scl_pullup_en = conf.sda_pullup_en;
				conf.master.clk_speed = CONFIG_I2C_MANAGER_1_FREQ_HZ;
			#else
				ESP_LOGE(TAG, "I2C port 1 not configured.");
				ESP_ERROR_CHECK(ESP_FAIL);
			#endif
		} else {
			ESP_LOGE(TAG, "Invalid port.");
			ESP_ERROR_CHECK(ESP_FAIL);
		}

		conf.mode = I2C_MODE_MASTER;

//		bool pullups = (port == I2C_NUM_0) ? I2C_MANAGER_0_PULLUPS : I2C_MANAGER_1_PULLUPS;

		ret = i2c_param_config(port, &conf);
		ret |= i2c_driver_install(port, conf.mode, 0, 0, 0);

		if (ret != ESP_OK) {
			ESP_LOGE(TAG, "Failed to initialise I2C port %d.", (int)port);
			ESP_LOGW(TAG, "If it was already open, we'll use it with whatever settings were used "
			              "to open it. See I2C Manager README for details.");
		} else {
			ESP_LOGI(TAG, "Initialised port %d (SDA: %d, SCL: %d, speed: %d Hz.)",
					 port, conf.sda_io_num, conf.scl_io_num, conf.master.clk_speed);
//					 pullups ? ", internal pullups" : "");
		}

	}

    return ret;
}

esp_err_t I2C_FN(_read)(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t *buffer, uint16_t size) {

    esp_err_t result;

    // May seem weird, but init starts with a check if it's needed, no need for that check twice.
	I2C_FN(_init)(port);

	if (reg) {
	   	ESP_LOGD(TAG, "Reading addr 0x%02x reg 0x%02x port %d", addr, reg, port);
	} else {
		ESP_LOGD(TAG, "Reading addr CONFIG_I2C_MANAGER_1_ENABLED0x%02x port %d", addr, port);
	}

	TickType_t timeout;
	#if defined (CONFIG_I2C_MANAGER_0_ENABLED)
		if (port == I2C_NUM_0) {
			timeout = (CONFIG_I2C_MANAGER_0_TIMEOUT) / portTICK_RATE_MS;
		}
	#endif
	#if defined (CONFIG_I2C_MANAGER_1_ENABLED)
		if (port == I2C_NUM_1) {
			timeout = (CONFIG_I2C_MANAGER_1_TIMEOUT) / portTICK_RATE_MS;
		}
	#endif

	if (I2C_FN(_lock)((int)port) == ESP_OK) {
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		if (reg) {
			/* When reading specific register set the addr pointer first. */
			i2c_master_start(cmd);
			i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
			i2c_master_write(cmd, &reg, 1, ACK_CHECK_EN);
		}
		/* Read size bytes from the current pointer. */
		i2c_master_start(cmd);
		i2c_master_write_byte(
			cmd,
			(addr << 1) | I2C_MASTER_READ,
			ACK_CHECK_EN
		);
		if (size > 1) {
			i2c_master_read(cmd, buffer, size - 1, I2C_MASTER_ACK);
		}
		i2c_master_read_byte(cmd, buffer + size - 1, I2C_MASTER_NACK);
		i2c_master_stop(cmd);
		result = i2c_master_cmd_begin(port, cmd, timeout);
		i2c_cmd_link_delete(cmd);
		I2C_FN(_unlock)((int)port);
	} else {
		ESP_LOGE(TAG, "Lock could not be obtained for port %d.", (int)port);
		return ESP_ERR_TIMEOUT;
	}

    if (result != ESP_OK) {
    	ESP_LOGW(TAG, "Error reading addr 0x%02x reg 0x%02x port %d", addr, reg, port);
    }

	ESP_LOG_BUFFER_HEX_LEVEL(TAG, buffer, size, ESP_LOG_DEBUG);

    return result;
}

esp_err_t I2C_FN(_write)(i2c_port_t port, uint8_t addr, uint8_t reg, const uint8_t *buffer, uint16_t size) {

    esp_err_t result;

    // May seem weird, but init starts with a check if it's needed, no need for that check twice.
	I2C_FN(_init)(port);

    ESP_LOGD(TAG, "Writing addr 0x%02x reg 0x%02x port %d", addr, reg, port);

	TickType_t timeout;
	#if defined (CONFIG_I2C_MANAGER_0_ENABLED)
		if (port == I2C_NUM_0) {
			timeout = (CONFIG_I2C_MANAGER_0_TIMEOUT) / portTICK_RATE_MS;
		}
	#endif
	#if defined (CONFIG_I2C_MANAGER_1_ENABLED)
		if (port == I2C_NUM_1) {
			timeout = (CONFIG_I2C_MANAGER_1_TIMEOUT) / portTICK_RATE_MS;
		}
	#endif

	if (I2C_FN(_lock)((int)port) == ESP_OK) {
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		ESP_LOG_BUFFER_HEX_LEVEL(TAG, buffer, size, ESP_LOG_DEBUG);
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
		i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
		i2c_master_write(cmd, (uint8_t *)buffer, size, ACK_CHECK_EN);
		i2c_master_stop(cmd);
		result = i2c_master_cmd_begin( port, cmd, timeout);
		i2c_cmd_link_delete(cmd);
		I2C_FN(_unlock)((int)port);
	} else {
		ESP_LOGE(TAG, "Lock could not be obtained for port %d.", (int)port);
		return ESP_ERR_TIMEOUT;
	}

    if (result != ESP_OK) {
    	ESP_LOGW(TAG, "Error writing addr 0x%02x reg 0x%02x port %d", addr, reg, port);
    }

    return result;
}

esp_err_t I2C_FN(_close)(i2c_port_t port) {
    vSemaphoreDelete(I2C_FN(_mutex)[port]);
    I2C_FN(_mutex)[port] = NULL;
    ESP_LOGI(TAG, "Closing I2C master at port %d", port);
    return i2c_driver_delete(port);
}

esp_err_t I2C_FN(_lock)(i2c_port_t port) {
	ESP_LOGD(TAG, "Mutex lock set for %d.", (int)port);

	TickType_t timeout;
	#if defined (CONFIG_I2C_MANAGER_0_ENABLED)
		if (port == I2C_NUM_0) {
			timeout = (CONFIG_I2C_MANAGER_0_LOCK_TIMEOUT) / portTICK_RATE_MS;
		}
	#endif
	#if defined (CONFIG_I2C_MANAGER_1_ENABLED)
		if (port == I2C_NUM_1) {
			timeout = (CONFIG_I2C_MANAGER_1_LOCK_TIMEOUT) / portTICK_RATE_MS;
		}
	#endif

	if (xSemaphoreTake(I2C_FN(_mutex)[port], timeout) == pdTRUE) {
		return ESP_OK;
	} else {
		ESP_LOGE(TAG, "Removing stale mutex lock from port %d.", (int)port);
		I2C_FN(_force_unlock)(port);
		return (xSemaphoreTake(I2C_FN(_mutex)[port], timeout) == pdTRUE ? ESP_OK : ESP_FAIL);
	}
}

esp_err_t I2C_FN(_unlock)(i2c_port_t port) {
	ESP_LOGD(TAG, "Mutex lock removed for %d.", (int)port);
	return (xSemaphoreGive(I2C_FN(_mutex)[port]) == pdTRUE) ? ESP_OK : ESP_FAIL;
}

esp_err_t I2C_FN(_force_unlock)(i2c_port_t port) {
	if (I2C_FN(_mutex)[port]) {
		vSemaphoreDelete(I2C_FN(_mutex)[port]);
	}
	I2C_FN(_mutex)[port] = xSemaphoreCreateMutex();
	return ESP_OK;
}



#ifdef I2C_OEM

    void I2C_FN(_locking)(void* leader) {
        if (leader) {
            ESP_LOGI(TAG, "Now following I2C Manager for locking");
            I2C_FN(_mutex) = (SemaphoreHandle_t*)leader;
		}
    }

#else

    void* i2c_manager_locking() {
        return (void*)i2c_manager_mutex;
    }

    int32_t i2c_hal_read(void *handle, uint8_t address, uint8_t reg, uint8_t *buffer, uint16_t size) {
        return i2c_manager_read(*(i2c_port_t*)handle, address, reg, buffer, size);
    }

    int32_t i2c_hal_write(void *handle, uint8_t address, uint8_t reg, const uint8_t *buffer, uint16_t size) {
        return i2c_manager_write(*(i2c_port_t*)handle, address, reg, buffer, size);
    }

    static i2c_port_t port_zero = I2C_NUM_0;
    static i2c_port_t port_one = I2C_NUM_1;

    static i2c_hal_t _i2c_hal[2] = {
        {&i2c_hal_read, &i2c_hal_write, &port_zero},
        {&i2c_hal_read, &i2c_hal_write, &port_one}
    };

    void* i2c_hal(i2c_port_t port) {
        return (void*)&_i2c_hal[port];
    }

#endif

/*********************************************************************************
   Original author: Alexandr Pochtovy<alex.mail.prime@gmail.com>

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

 * 	INA219.c
 *  Created on: Jan 27, 2022
 */

#include "INA219.h"

static const uint8_t INA219_REG_LEN = 2;
static const uint32_t  INA219_MaxCurrentmA = 3200;//maximum curent in mA
static const uint32_t  INA219_ShuntResistancemOmh = 100;//shunt resistance in milliOmh
#define INA219_CalibrationVal				(uint32_t)0x08000000 / ((uint32_t)(INA219_MaxCurrentmA * INA219_ShuntResistancemOmh) / 10)
#define INA219_Current_LSB_mkA			(uint32_t)(INA219_MaxCurrentmA * 1000 / 0x8000)
#define INA219_Power_LSB_mkV				(uint32_t)(20 * INA219_Current_LSB_mkA)

static inline CONCAT_BYTES(uint8_t msb, uint8_t lsb) {
    return (uint16_t)(((uint16_t)msb << 8) | (uint16_t)lsb);
}

//Init & setup	=============================================================================================
uint8_t INA219_Init(I2C_Connection *_i2c, INA219_t *dev) {
	if (_i2c->status == PORT_FREE) {
		_i2c->addr = dev->addr;
		_i2c->len = INA219_REG_LEN;
		_i2c->mode = I2C_MODE_WRITE;
		uint8_t dt[INA219_REG_LEN];
		switch (dev->step) {
		case 0:
			dev->status = INIT;
			_i2c->reg = INA219_REG_CONFIG;
			uint16_t cfg = 	INA219_CFG_MODE_SHBV_CONTINUOUS 	|	// shunt and bus voltage continuous	defaulth
									INA219_CFG_SADC_12BIT_128S 					|	// 128 x 12-bit shunt samples averaged together
									INA219_CFG_BADC_12BIT_128S 					|	// 128 x 12-bit bus samples averaged together
									INA219_CFG_GAIN_8_320MV 						|	// Gain 8, 320mV Range defaulth
									INA219_CFG_BVRANGE_16V;							// 0-16V Range
			dt[0] = (uint8_t)cfg;
			dt[1] = (uint8_t)(cfg >> 8);
			PutMulti(&_i2c->buffer, dt, INA219_REG_LEN);
			dev->step = 1;
			break;
		case 1:
			dev->raw.calibration = INA219_CalibrationVal;
			_i2c->reg = INA219_REG_CALIBRATION;
			dt[0] = (uint8_t)dev->raw.calibration;
			dt[1] = (uint8_t)(dev->raw.calibration >> 8);
			PutMulti(&_i2c->buffer, dt, INA219_REG_LEN);
			dev->step = 2;
			break;
		case 2:
			dev->status = OK;
			dev->step = 0;
			return 1;
			break;
		default:
			dev->step = 0;
			break;
		}
		I2C_Start_IRQ(_i2c);
	}
	return 0;
}

uint8_t INA219_GetRawData(I2C_Connection *_i2c, INA219_t *dev) {
	if ((_i2c->status == PORT_FREE) && (dev->status == OK)) {
		uint8_t dt[INA219_REG_LEN];
		_i2c->addr = dev->addr;
		_i2c->len = INA219_REG_LEN;
		_i2c->mode = I2C_MODE_READ;
		switch (dev->step) {
		case 0://select reg
			switch (_i2c->reg) {
			case INA219_REG_SHUNTVOLTAGE://shunt voltage
				_i2c->reg = INA219_REG_BUSVOLTAGE;//read shunt voltage
				break;
			case INA219_REG_BUSVOLTAGE://read shunt voltage
				_i2c->reg = INA219_REG_POWER;//read power
				break;
			case INA219_REG_POWER://read power
				_i2c->reg = INA219_REG_CURRENT;//read current
				break;
			case INA219_REG_CURRENT://current
				_i2c->reg = INA219_REG_SHUNTVOLTAGE;//read voltage
				break;
			default:
				_i2c->reg = INA219_REG_SHUNTVOLTAGE;
				break;
			}
			dev->step = 1;
			break;
		case 1:
			GetMulti(&_i2c->buffer, dt, INA219_REG_LEN);
			switch (_i2c->reg) {
				case INA219_REG_SHUNTVOLTAGE:
					dev->raw.shuntV = (int16_t)CONCAT_BYTES(dt[0], dt[1]);
					break;
				case INA219_REG_BUSVOLTAGE://read shunt voltage
					dev->raw.voltage = (uint16_t)CONCAT_BYTES(dt[0], dt[1]);
					break;
				case INA219_REG_POWER://read power
					dev->raw.power = (uint16_t)CONCAT_BYTES(dt[0], dt[1]);
					break;
				case INA219_REG_CURRENT://current
					dev->raw.current = (uint16_t)CONCAT_BYTES(dt[0], dt[1]);
					return 1;
					break;
				default:
					break;
			}
			dev->step = 0;
			break;
		default:
			dev->step = 0;
			break;
		}
		I2C_Start_IRQ(_i2c);
	}
	return 0;
}
//Get & conversion raw data	=================================================================================
uint16_t INA219_GetVoltageInt(INA219_t *dev, uint16_t divider) {
	return (uint16_t)(((uint32_t)(dev->raw.voltage >> 3  ) * 4) / divider);
}

float INA219_GetVoltageFloat(INA219_t *dev, uint16_t divider) {
	return ((float)((uint32_t)(dev->raw.voltage >> 3) * 4)) / divider;
}

uint16_t INA219_GetCurrentInt(INA219_t *dev, uint16_t divider) {
	return (uint16_t)(((uint32_t)dev->raw.current * INA219_Current_LSB_mkA) / divider);
}

float INA219_GetCurrentFloat(INA219_t *dev, uint16_t divider) {
	return ((float)((uint32_t)dev->raw.current * INA219_Current_LSB_mkA)) / divider;
}

uint16_t INA219_GetPowerInt(INA219_t *dev, uint16_t divider) {
	return (uint16_t)(((uint32_t)dev->raw.power * INA219_Power_LSB_mkV) / divider);
}

float INA219_GetPowerFloat(INA219_t *dev, uint16_t divider) {
	return (float)(((uint32_t)dev->raw.power * INA219_Power_LSB_mkV)) / divider;
}

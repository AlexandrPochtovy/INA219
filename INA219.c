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
//Init & setup	=============================================================================================
INA_Connect_Status INA219_Init(I2C_Connection *_i2c, INA219_dev *dev, uint8_t *pbuffer) {
	if (_i2c->i2cStatus == I2C_Bus_Free) {
		_i2c->addr = dev->addr;
		_i2c->len = INA219_REG_LEN;
		_i2c->mode = I2C_MODE_WRITE;
		_i2c->rxtxp = pbuffer;
		switch (dev->step) {
		case 0:
			dev->status = INA_Init;
			_i2c->reg = INA219_REG_CONFIG;
			uint16_t cfg = 	INA219_CFG_MODE_SHBV_CONTINUOUS 	|	// shunt and bus voltage continuous	defaulth
									INA219_CFG_SADC_12BIT_128S 					|	// 128 x 12-bit shunt samples averaged together
									INA219_CFG_BADC_12BIT_128S 					|	// 128 x 12-bit bus samples averaged together
									INA219_CFG_GAIN_8_320MV 						|	// Gain 8, 320mV Range defaulth
									INA219_CFG_BVRANGE_16V;							// 0-16V Range
			_i2c->rxtxp[0] = (uint8_t)cfg;
			_i2c->rxtxp[1] = (uint8_t)(cfg >> 8);
			dev->step = 1;
			break;
		case 1:
			dev->raw.calibration = INA219_CalibrationVal;
			_i2c->reg = INA219_REG_CALIBRATION;
			_i2c->rxtxp[0] = (uint8_t)dev->raw.calibration;
			_i2c->rxtxp[1] = (uint8_t)(dev->raw.calibration >> 8);
			dev->step = 2;
			break;
		case 2:
			dev->status = INA_OK;
			dev->step = 0;
			return INA_Complite;
			break;
		default:
			dev->step = 0;
			break;
		}
		I2C_Start_IRQ(_i2c);
	}
	return INA_Processing;
}

INA_Connect_Status INA219_GetRawData(I2C_Connection *_i2c, INA219_dev *dev, uint8_t *pbuffer) {
	if ((_i2c->i2cStatus == I2C_Bus_Free) && (dev->status == INA_OK)) {
		_i2c->addr = dev->addr;
		_i2c->len = INA219_REG_LEN;
		_i2c->mode = I2C_MODE_READ;
		_i2c->rxtxp = pbuffer;
		switch (dev->step) {
		case 0://select reg
			switch (dev->reg) {
			case INA219_REG_SHUNTVOLTAGE://shunt voltage
				dev->reg = INA219_REG_BUSVOLTAGE;//read shunt voltage
				_i2c->reg = INA219_REG_BUSVOLTAGE;//read shunt voltage
				break;
			case INA219_REG_BUSVOLTAGE://read shunt voltage
				dev->reg = INA219_REG_POWER;//read power
				_i2c->reg = INA219_REG_POWER;//read power
				break;
			case INA219_REG_POWER://read power
				dev->reg = INA219_REG_CURRENT;//read current
				_i2c->reg = INA219_REG_CURRENT;//read current
				break;
			case INA219_REG_CURRENT://current
				dev->reg = INA219_REG_SHUNTVOLTAGE;//read voltage
				_i2c->reg = INA219_REG_SHUNTVOLTAGE;//read voltage
				break;
			default:
				dev->reg = INA219_REG_SHUNTVOLTAGE;
				_i2c->reg = INA219_REG_SHUNTVOLTAGE;
				break;
			}
			dev->step = 1;
			break;

		case 1:
			switch (dev->reg) {
			case INA219_REG_SHUNTVOLTAGE://shunt voltage
					dev->raw.shuntV = INA219_CONCAT_BYTES(_i2c->rxtxp[0], _i2c->rxtxp[1]);;
					break;
			case INA219_REG_BUSVOLTAGE://read shunt voltage
				dev->raw.voltage = INA219_CONCAT_BYTES(_i2c->rxtxp[0], _i2c->rxtxp[1]);
				break;
			case INA219_REG_POWER://read power
				dev->raw.power = INA219_CONCAT_BYTES(_i2c->rxtxp[0], _i2c->rxtxp[1]);
				break;
			case INA219_REG_CURRENT://current
				dev->raw.current = INA219_CONCAT_BYTES(_i2c->rxtxp[0], _i2c->rxtxp[1]);
				break;
			default:
				break;
			}
			dev->step = 0;
			return INA_Complite;
			break;
		default:
			dev->step = 0;
			break;
		}
		I2C_Start_IRQ(_i2c);
	}
	return INA_Processing;
}
//Get & conversion raw data	=================================================================================
uint16_t INA219_GetVoltageInt(INA219_dev *dev, uint16_t divider) {
	return (uint16_t)(((uint32_t)(dev->raw.voltage >> 3  ) * 4) / divider);
}

float INA219_GetVoltageFloat(INA219_dev *dev, uint16_t divider) {
	return ((float)((uint32_t)(dev->raw.voltage >> 3) * 4)) / divider;
}

uint16_t INA219_GetCurrentInt(INA219_dev *dev, uint16_t divider) {
	return (uint16_t)(((uint32_t)dev->raw.current * INA219_Current_LSB_mkA) / divider);
}

float INA219_GetCurrentFloat(INA219_dev *dev, uint16_t divider) {
	return ((float)((uint32_t)dev->raw.current * INA219_Current_LSB_mkA)) / divider;
}

uint16_t INA219_GetPowerInt(INA219_dev *dev, uint16_t divider) {
	return (uint16_t)(((uint32_t)dev->raw.power * INA219_Power_LSB_mkV) / divider);
}

float INA219_GetPowerFloat(INA219_dev *dev, uint16_t divider) {
	return (float)(((uint32_t)dev->raw.power * INA219_Power_LSB_mkV)) / divider;
}

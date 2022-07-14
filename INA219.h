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

#ifndef INC_INA219_H_
#define INC_INA219_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "INA219_Register.h"
#include "I2C/MyI2C.h"
//============================================================================================
enum INA219_ADDRESS {
	INA219_ADDR = 0x88
};

typedef struct INA219_RawData_t {
	int16_t shuntV;
	uint16_t voltage;
	uint16_t current;
	uint16_t power;
	uint16_t calibration;
} INA219_RawData;

typedef struct INA219 {
		const uint8_t addr;
		uint8_t step;
		Device_status_t status;
		INA219_RawData raw;
} INA219_t;
//Init & setup	=============================================================================================
uint8_t INA219_Init(I2C_Connection *_i2c, INA219_t *dev);
uint8_t INA219_GetRawData(I2C_Connection *_i2c, INA219_t *dev);
//Get & conversion raw data	=================================================================================
uint16_t INA219_GetVoltageInt(INA219_t *dev, uint16_t divider);
float INA219_GetVoltageFloat(INA219_t *dev, uint16_t divider);
uint16_t INA219_GetCurrentInt(INA219_t *dev, uint16_t divider);
float INA219_GetCurrentFloat(INA219_t *dev, uint16_t divider);
uint16_t INA219_GetPowerInt(INA219_t *dev, uint16_t divider);
float INA219_GetPowerFloat(INA219_t *dev, uint16_t divider);

#ifdef __cplusplus
}
#endif

#endif /* INC_INA219_H_ */

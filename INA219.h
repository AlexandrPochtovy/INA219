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
#include "I2C_Master/MyI2C.h"
//============================================================================================
#define INA219_CONCAT_BYTES(msb, lsb)	(((uint16_t)msb << 8) | (uint16_t)lsb)

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

/* состояние процесса обмена данными с устройством как с отдельным элементом сети
 * 	применяется для отображения состояния процесса работы с устройством для главного кода
 */
typedef enum INA_status_t {//состояние устройства
	INA_Init,		//устройство не настроено
	INA_OK,		//устройство готово к опросу
	INA_Faulth	//устройство неисправно
} INA_status;

/*	состояние обмена данными с устройством, использовать для завершения функции работы с устройством */
typedef enum INA_Connect_Status_t {
	INA_Processing, //выполняется работа с устройством: обмен данными, обработка результатов
	INA_Complite	//работа с устройством завершена, данные считаны/записаны корректно
} INA_Connect_Status;

typedef struct INA219_dev_t {
		uint8_t addr;
		uint8_t reg;
		uint8_t step;
		INA_status status;
		INA219_RawData raw;
} INA219_dev;
//Init & setup	=============================================================================================
INA_Connect_Status INA219_Init(I2C_Connection *_i2c, INA219_dev *dev, uint8_t *pbuffer);
INA_Connect_Status INA219_GetRawData(I2C_Connection *_i2c, INA219_dev *dev, uint8_t *pbuffer);
//Get & conversion raw data	=================================================================================
uint16_t INA219_GetVoltageInt(INA219_dev *dev, uint16_t divider);
float INA219_GetVoltageFloat(INA219_dev *dev, uint16_t divider);
uint16_t INA219_GetCurrentInt(INA219_dev *dev, uint16_t divider);
float INA219_GetCurrentFloat(INA219_dev *dev, uint16_t divider);
uint16_t INA219_GetPowerInt(INA219_dev *dev, uint16_t divider);
float INA219_GetPowerFloat(INA219_dev *dev, uint16_t divider);

#ifdef __cplusplus
}
#endif

#endif /* INC_INA219_H_ */

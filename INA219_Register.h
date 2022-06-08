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

 * 	INA219_Register.h
 *  Created on: Jan 27, 2022
 */

#ifndef INA219_REGISTER_H_
#define INA219_REGISTER_H_

#ifdef __cplusplus
extern "C" {
#endif

//	Registers	======================================================
enum INA219_REG{
	INA219_REG_CONFIG				= 0x00,
	INA219_REG_SHUNTVOLTAGE	= 0x01,
	INA219_REG_BUSVOLTAGE		= 0x02,
	INA219_REG_POWER				= 0x03,
	INA219_REG_CURRENT			= 0x04,
	INA219_REG_CALIBRATION		= 0x05
};
//	MASK	====================================================
//bus voltage select
enum NA219_CFG_BVRANGE {
	INA219_CFG_BVRANGE_16V = 0x0000,	// 0-16V Range
	INA219_CFG_BVRANGE_32V = 0x2000	// 0-32V Range defaulth
};
//gain config
enum INA219_CFG_GAIN {
	INA219_CFG_GAIN_1_40MV	 	= 0x0000,	// Gain 1, 40mV Range
	INA219_CFG_GAIN_2_80MV	 	= 0x0800,	// Gain 2, 80mV Range
	INA219_CFG_GAIN_4_160MV 	= 0x1000,	// Gain 4, 160mV Range
	INA219_CFG_GAIN_8_320MV 	= 0x1800	// Gain 8, 320mV Range defaulth
};
//BUS VOLTAGE resolution and samples select
enum INA219_CFG_BADC {
	INA219_CFG_BADC_9BIT_1S		= 0x0000,  // 9-bit bus res = 0..511
  	INA219_CFG_BADC_10BIT_1S		= 0x0080, // 10-bit bus res = 0..1023
  	INA219_CFG_BADC_11BIT_1S		= 0x0100, // 11-bit bus res = 0..2047
  	INA219_CFG_BADC_12BIT_1S		= 0x0180, // 12-bit bus res = 0..4097 defaulth
  	INA219_CFG_BADC_12BIT_2S		= 0x0480, // 2 x 12-bit bus samples averaged together
  	INA219_CFG_BADC_12BIT_4S		= 0x0500, // 4 x 12-bit bus samples averaged together
  	INA219_CFG_BADC_12BIT_8S		= 0x0580, // 8 x 12-bit bus samples averaged together
  	INA219_CFG_BADC_12BIT_16S	= 0x0600, // 16 x 12-bit bus samples averaged together
  	INA219_CFG_BADC_12BIT_32S	= 0x0680, // 32 x 12-bit bus samples averaged together
  	INA219_CFG_BADC_12BIT_64S	= 0x0700, // 64 x 12-bit bus samples averaged together
  	INA219_CFG_BADC_12BIT_128S	= 0x0780 // 128 x 12-bit bus samples averaged together
};
//SHUNT VOLTAGE resolution and samples select
enum INA219_CFG_SADC {
	INA219_CFG_SADC_9BIT_1S			= 0x0000,  // 9-bit shunt res = 0..511
  	INA219_CFG_SADC_10BIT_1S		= 0x0008, // 10-bit shunt res = 0..1023
  	INA219_CFG_SADC_11BIT_1S		= 0x0010, // 11-bit shunt res = 0..2047
  	INA219_CFG_SADC_12BIT_1S		= 0x0018, // 12-bit shunt res = 0..4097 defaulth
  	INA219_CFG_SADC_12BIT_2S		= 0x0048, // 2 x 12-bit shunt samples averaged together
  	INA219_CFG_SADC_12BIT_4S		= 0x0050, // 4 x 12-bit shunt samples averaged together
  	INA219_CFG_SADC_12BIT_8S		= 0x0058, // 8 x 12-bit shunt samples averaged together
  	INA219_CFG_SADC_12BIT_16S		= 0x0060, // 16 x 12-bit shunt samples averaged together
  	INA219_CFG_SADC_12BIT_32S		= 0x0068, // 32 x 12-bit shunt samples averaged together
  	INA219_CFG_SADC_12BIT_64S		= 0x0070, // 64 x 12-bit shunt samples averaged together
  	INA219_CFG_SADC_12BIT_128S	= 0x0078, // 128 x 12-bit shunt samples averaged together
};
//mode select
enum INA219_CFG_MODE {
	INA219_CFG_MODE_POWERDOWN			= 0x0000,	// power down
  	INA219_CFG_MODE_SH_TRIGGERED			= 0x0001,	// only shunt voltage triggered
  	INA219_CFG_MODE_BV_TRIGGERED			= 0x0002,	// only bus voltage triggered
  	INA219_CFG_MODE_SHBV_TRIGGERED		= 0x0003, 	// shunt and bus voltage triggered
  	INA219_CFG_MODE_ADCOFF						= 0x0004, 	// ADC off
  	INA219_CFG_MODE_SH_CONTINUOUS		= 0x0005, 	// shunt voltage continuous
  	INA219_CFG_MODE_BV_CONTINUOUS		= 0x0006, 	// bus voltage continuous
  	INA219_CFG_MODE_SHBV_CONTINUOUS 	= 0x0007,	// shunt and bus voltage continuous	defaulth
   INA219_CFG_FULL_RESET 					 					= 0x8000
};

#ifdef __cplusplus
}
#endif

#endif /* INA219_REGISTER_H_ */

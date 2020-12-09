#ifndef TMC5130_IO_H_
#define TMC5130_IO_H_

#include "stm32f4xx_hal.h"

void TMC5130_IO_Init(TMC5130TypeDef *tmc5130);
void TMC5130_Write_Register(TMC5130TypeDef *handle, uint8_t address, int32_t value);
int32_t TMC5130_Read_Register(TMC5130TypeDef *handle, uint8_t address);

#endif /* TMC5130_IO_H_ */

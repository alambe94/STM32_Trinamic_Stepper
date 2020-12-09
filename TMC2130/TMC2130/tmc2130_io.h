#ifndef TMC2130_IO_H_
#define TMC2130_IO_H_

#include "stm32f4xx_hal.h"

void TMC2130_IO_Init(TMC2130TypeDef *tmc2130);
void TMC2130_Write_Register(TMC2130TypeDef *handle, uint8_t address, int32_t value);
int32_t TMC2130_Read_Register(TMC2130TypeDef *handle, uint8_t address);

#endif /* TMC2130_IO_H_ */

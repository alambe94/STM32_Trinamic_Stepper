
#include "spi.h"
#include "Macros.h"
#include "tmc2130.h"
#include "tmc2130_io.h"
#include "TMC2130_Constants.h"
#include "RegisterAccess.h"

// Initialize a TMC2130 IC.
void TMC2130_IO_Init(TMC2130TypeDef *tmc2130)
{
    // spi configured in cube @see spi.c
	// gpio configured in cube @see gpio.c

    uint8_t address;
    for (size_t i = 0; i < TMC2130_REGISTER_COUNT; i++)
    {
        tmc2130->registerAccess[i] = tmc2130_defaultRegisterAccess[i];

        // Write to the shadow
        address = TMC_ADDRESS(i);
        tmc2130->shadowRegister[address] = TMC2130_Read_Register(tmc2130, address);
    }
}


void TMC2130_Write_Register(TMC2130TypeDef *tmc2130, uint8_t address, int32_t value)
{
    uint8_t temp;
    address += TMC2130_WRITE_BIT;

    HAL_GPIO_WritePin(tmc2130->CS_Port, tmc2130->CS_Pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&hspi1, &address, 1, 100);

    temp = ((value >> 24UL) & 0xFF);
    HAL_SPI_Transmit(&hspi1, &temp, 1, 100);

    temp = ((value >> 16UL) & 0xFF);
    HAL_SPI_Transmit(&hspi1, &temp, 1, 100);

    temp = ((value >> 8UL) & 0xFF);
    HAL_SPI_Transmit(&hspi1, &temp, 1, 100);

    temp = ((value >> 0UL) & 0xFF);
    HAL_SPI_Transmit(&hspi1, &temp, 1, 100);

    HAL_GPIO_WritePin(tmc2130->CS_Port, tmc2130->CS_Pin, GPIO_PIN_SET);

    // Write to the shadow register and mark the register dirty
    address = TMC_ADDRESS(address);
    tmc2130->shadowRegister[address] = value;
    tmc2130->registerAccess[address] |= TMC_ACCESS_DIRTY;
}

// Read an integer from the given address
int32_t TMC2130_Read_Register(TMC2130TypeDef *tmc2130, uint8_t address)
{
    uint8_t temp = 0x00;
    address = TMC_ADDRESS(address);
    int32_t val = 0;

    // register not readable -> shadow register copy
    if (!TMC_IS_READABLE(tmc2130->registerAccess[address]))
        return tmc2130->shadowRegister[address];

    HAL_GPIO_WritePin(tmc2130->CS_Port, tmc2130->CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &address, 1, 100);
    HAL_SPI_Transmit(&hspi1, &temp, 1, 100);
    HAL_SPI_Transmit(&hspi1, &temp, 1, 100);
    HAL_SPI_Transmit(&hspi1, &temp, 1, 100);
    HAL_SPI_Transmit(&hspi1, &temp, 1, 100);
    HAL_GPIO_WritePin(tmc2130->CS_Port, tmc2130->CS_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(tmc2130->CS_Port, tmc2130->CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &address, 1, 100);
    HAL_SPI_Receive(&hspi1, &temp, 1, 100);
    val = temp << 24;
    HAL_SPI_Receive(&hspi1, &temp, 1, 100);
    val |= temp << 16;
    HAL_SPI_Receive(&hspi1, &temp, 1, 100);
    val |= temp << 8;
    HAL_SPI_Receive(&hspi1, &temp, 1, 100);
    val |= temp << 0;
    HAL_GPIO_WritePin(tmc2130->CS_Port, tmc2130->CS_Pin, GPIO_PIN_SET);

    return val;
}


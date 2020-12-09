#include <stdlib.h>

#include "tmc5130.h"
#include "tmc5130_io.h"
#include "delay_us.h"
#include "usart.h"
#include "main.h"

TMC5130TypeDef Motor_X;

void App_Main(void)
{
	int32_t sts = 0;

	Motor_X.CS_Port = SPI_CS_X_GPIO_Port;
	Motor_X.CS_Pin = SPI_CS_X_Pin;

	TMC5130_IO_Init(&Motor_X);

	TMC5130_Set_Chopper_TOff(&Motor_X, 4);
	TMC5130_Set_Chopper_Blank_Time(&Motor_X, 0b10);
	TMC5130_Set_I_MAX(&Motor_X, 5);
	TMC5130_Set_I_STBY(&Motor_X, 3);
	TMC5130_Set_Microstep_Resolution(&Motor_X, 128);
	TMC5130_Set_SE_Speed_THRESH(&Motor_X, 0xFFFFF);
	TMC5130_Set_SE_HSpeed_THRESH(&Motor_X, 0);
	TMC5130_Set_SE_I_MIN(&Motor_X, 5);
	TMC5130_Set_SE_HYS(&Motor_X, 2);
	TMC5130_Set_SE_I_DWN_Step(&Motor_X, 0b01);

	TMC5130_Set_ACC_A1(&Motor_X, 0x3E8);
	TMC5130_Set_ACC_MAX(&Motor_X, 0x1F4);

	TMC5130_Set_VEL_Start(&Motor_X, 100);
	TMC5130_Set_VEL_V1(&Motor_X, 0x0);
	TMC5130_Set_VEL_MAX(&Motor_X, 0x186A0);
	TMC5130_Set_VEL_Stop(&Motor_X, 0xA);

	TMC5130_Set_DEC_MAX(&Motor_X, 0x2BC);
	TMC5130_Set_DEC_D1(&Motor_X, 0x578);

	TMC5130_Set_stallGuard2_THRESH(&Motor_X, 12);

	TMC5130_Run(&Motor_X, 1000000);

	UNUSED(sts);
}

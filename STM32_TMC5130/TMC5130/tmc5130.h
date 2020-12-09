#ifndef TMC_5130_H_
#define TMC_5130_H_

#include "Macros.h"
#include "stm32f4xx_hal.h"
#include "RegisterAccess.h"
#include "TMC5130_Constants.h"

// Helper macros
#define TMC5130_FIELD_READ(tdef, address, mask, shift) \
	FIELD_GET(TMC5130_Read_Register(tdef, address), mask, shift)

#define TMC5130_FIELD_WRITE(tdef, address, mask, shift, value) \
	(TMC5130_Write_Register(tdef, address, FIELD_SET(TMC5130_Read_Register(tdef, address), mask, shift, value)))

// Typedefs
typedef struct
{
	GPIO_TypeDef *CS_Port;
	uint16_t CS_Pin;

	int32_t shadowRegister[TMC5130_REGISTER_COUNT];
	uint8_t registerAccess[TMC5130_REGISTER_COUNT];

} TMC5130TypeDef;


// Register access permissions:
//   0x00: none (reserved)
//   0x01: read
//   0x02: write
//   0x03: read/write
//   0x13: read/write, separate functions/values for reading or writing
//   0x21: read, flag register (read to clear)
//   0x42: write, has hardware presets on reset
static const uint8_t tmc5130_defaultRegisterAccess[TMC5130_REGISTER_COUNT] =
{
//  0     1     2     3     4     5     6     7     8     9     A     B     C     D     E     F
	0x03, 0x21, 0x01, 0x02, 0x13, 0x02, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, // 0x00 - 0x0F
	0x02, 0x02, 0x01, 0x02, 0x02, 0x02, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, // 0x10 - 0x1F
	0x03, 0x03, 0x01, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, ____, 0x02, 0x02, 0x02, 0x03, ____, ____, // 0x20 - 0x2F
	____, ____, ____, 0x02, 0x03, 0x21, 0x01, ____, 0x03, 0x03, 0x02, 0x21, 0x01, ____, ____, ____, // 0x30 - 0x3F
	____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, // 0x40 - 0x4F
	____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, // 0x50 - 0x5F
	0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x01, 0x01, 0x03, 0x02, 0x02, 0x01, // 0x60 - 0x6F
	0x42, 0x01, 0x02, 0x01, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____, ____  // 0x70 - 0x7F
};

void TMC5130_Run(TMC5130TypeDef *handle, int32_t velocity);
void TMC5130_stop(TMC5130TypeDef *handle);
void TMC5130_Move_To(TMC5130TypeDef *handle, int32_t position, uint32_t velocityMax);
void TMC5130_Move(TMC5130TypeDef *handle, int32_t ticks, uint32_t velocityMax);
int32_t TMC5130_Get_Target_Position(TMC5130TypeDef *handle);
void TMC5130_Set_Target_Position(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_Actual_Position(TMC5130TypeDef *handle);
void TMC5130_Set_Actual_Position(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_Target_Speed(TMC5130TypeDef *handle);
void TMC5130_Set_Target_Speed(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_Actual_Speed(TMC5130TypeDef *handle);
int32_t TMC5130_Get_VEL_MAX(TMC5130TypeDef *handle);
void TMC5130_Set_VEL_MAX(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_ACC_MAX(TMC5130TypeDef *handle);
void TMC5130_Set_ACC_MAX(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_I_MAX(TMC5130TypeDef *handle);
void TMC5130_Set_I_MAX(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_I_STBY(TMC5130TypeDef *handle);
void TMC5130_Set_I_STBY(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_Position_Flag(TMC5130TypeDef *handle);
int32_t TMC5130_Get_VREF(TMC5130TypeDef *handle);
void TMC5130_Set_VREF(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_Right_Endstop(TMC5130TypeDef *handle);
int32_t TMC5130_Get_Left_Endstop(TMC5130TypeDef *handle);
int32_t TMC5130_Get_Auto_Right_Stop(TMC5130TypeDef *handle);
void TMC5130_Set_Auto_Right_Stop(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_Auto_Left_Stop(TMC5130TypeDef *handle);
void TMC5130_Set_Auto_Left_Stop(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_SW_MODE(TMC5130TypeDef *handle);
void TMC5130_Set_SW_MODE(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_ACC_A1(TMC5130TypeDef *handle);
void TMC5130_Set_ACC_A1(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_VEL_V1(TMC5130TypeDef *handle);
void TMC5130_Set_VEL_V1(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_DEC_MAX(TMC5130TypeDef *handle);
void TMC5130_Set_DEC_MAX(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_DEC_D1(TMC5130TypeDef *handle);
void TMC5130_Set_DEC_D1(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_VEL_Start(TMC5130TypeDef *handle);
void TMC5130_Set_VEL_Start(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_VEL_Stop(TMC5130TypeDef *handle);
void TMC5130_Set_VEL_Stop(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_Wait_Time_Ramp_Down(TMC5130TypeDef *handle);
void TMC5130_Set_Wait_Time_Ramp_Down(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_SE_HSpeed_THRESH(TMC5130TypeDef *handle);
void TMC5130_Set_SE_HSpeed_THRESH(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_MIN_Speed_dcStep(TMC5130TypeDef *handle);
void TMC5130_Set_MIN_Speed_dcStep(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_HS_Chopper_Mode(TMC5130TypeDef *handle);
void TMC5130_Set_HS_Chopper_Mode(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_FS_Mode(TMC5130TypeDef *handle);
void TMC5130_Set_FS_Mode(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_Measured_Speed(TMC5130TypeDef *handle);
int32_t TMC5130_Get_Ramp_Type(TMC5130TypeDef *handle);
void TMC5130_Set_Ramp_Type(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_Analog_I_Scale(TMC5130TypeDef *handle);
void TMC5130_Set_Analog_I_Scale(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_Internal_RSense(TMC5130TypeDef *handle);
void TMC5130_Set_Internal_RSense(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_Microstep_Resolution(TMC5130TypeDef *handle);
void TMC5130_Set_Microstep_Resolution(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_Chopper_Blank_Time(TMC5130TypeDef *handle);
void TMC5130_Set_Chopper_Blank_Time(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_CONST_Toff_Mode(TMC5130TypeDef *handle);
void TMC5130_Set_CONST_Toff_Mode(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_FD_COMP(TMC5130TypeDef *handle);
void TMC5130_Set_FD_COMP(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_Chopper_HYS_FD_Time(TMC5130TypeDef *handle);
void TMC5130_Set_Chopper_HYS_FD_Time(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_Chopper_HYS_Sine_Offset(TMC5130TypeDef *handle);
void TMC5130_Set_Chopper_HYS_Sine_Offset(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_Chopper_TOff(TMC5130TypeDef *handle);
void TMC5130_Set_Chopper_TOff(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_SE_I_MIN(TMC5130TypeDef *handle);
void TMC5130_Set_SE_I_MIN(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_SE_I_DWN_Step(TMC5130TypeDef *handle);
void TMC5130_Set_SE_I_DWN_Step(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_SE_HYS(TMC5130TypeDef *handle);
void TMC5130_Set_SE_HYS(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_SE_I_UP_Step(TMC5130TypeDef *handle);
void TMC5130_Set_SE_I_UP_Step(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_SE_HYS_Start(TMC5130TypeDef *handle);
void TMC5130_Set_SE_HYS_Start(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_stallGuard2_Filter(TMC5130TypeDef *handle);
void TMC5130_Set_stallGuard2_Filter(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_stallGuard2_THRESH(TMC5130TypeDef *handle);
void TMC5130_Set_stallGuard2_THRESH(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_VSense(TMC5130TypeDef *handle);
void TMC5130_Set_VSense(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_SE_Actual_I(TMC5130TypeDef *handle);
int32_t TMC5130_Get_SE_VEL_Stall(TMC5130TypeDef *handle);
void TMC5130_Set_SE_VEL_Stall(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_SE_Speed_THRESH(TMC5130TypeDef *handle);
void TMC5130_Set_SE_Speed_THRESH(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_Random_TOff_Mode(TMC5130TypeDef *handle);
void TMC5130_Set_Random_TOff_Mode(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_Chopper_SYNC(TMC5130TypeDef *handle);
void TMC5130_Set_Chopper_SYNC(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_PWM_THRESH_Speed(TMC5130TypeDef *handle);
void TMC5130_Set_PWM_THRESH_Speed(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_PWM_Gradient(TMC5130TypeDef *handle);
void TMC5130_Set_PWM_Gradient(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_PWM_Amplitude(TMC5130TypeDef *handle);
void TMC5130_Set_PWM_Amplitude(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_PWM_FREQ(TMC5130TypeDef *handle);
void TMC5130_Set_PWM_FREQ(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_PWM_Auto_Scale(TMC5130TypeDef *handle);
void TMC5130_Set_PWM_Auto_Scale(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_Freewheeling_Mode(TMC5130TypeDef *handle);
void TMC5130_Set_Freewheeling_Mode(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_Load_Value(TMC5130TypeDef *handle);
int32_t TMC5130_Get_Encoder_Position(TMC5130TypeDef *handle);
void TMC5130_Set_Encoder_Position(TMC5130TypeDef *handle, int32_t value);
int32_t TMC5130_Get_Encoder_Resolution(TMC5130TypeDef *handle);
void TMC5130_Set_Encoder_Resolution(TMC5130TypeDef *handle, int32_t value);

#endif /* TMC_5130_H_ */

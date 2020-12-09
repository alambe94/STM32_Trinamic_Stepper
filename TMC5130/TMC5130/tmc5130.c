#include "stdlib.h"
#include "tmc5130.h"
#include "tmc5130_io.h"
#include "TMC5130_Fields.h"
#include "TMC5130_Register.h"
#include "TMC5130_Constants.h"

#define VREF_FULLSCALE 2714 // mV
static uint32_t vmax_position;
static uint16_t vref; // mV

// Rotate with a given velocity (to the right)
void TMC5130_Run(TMC5130TypeDef *handle, int32_t velocity)
{
	// Set absolute velocity
	TMC5130_Write_Register(handle, TMC5130_VMAX, abs(velocity));
	// Set direction
	TMC5130_Write_Register(handle, TMC5130_RAMPMODE, (velocity >= 0) ? TMC5130_MODE_VELPOS : TMC5130_MODE_VELNEG);
}

// Stop moving
void TMC5130_stop(TMC5130TypeDef *handle)
{
	TMC5130_Run(handle, 0);
}

// Move to a specified position with a given velocity
void TMC5130_Move_To(TMC5130TypeDef *handle, int32_t position, uint32_t velocityMax)
{
	TMC5130_Write_Register(handle, TMC5130_RAMPMODE, TMC5130_MODE_POSITION);

	// VMAX also holds the target velocity in velocity mode.
	// Re-write the position mode maximum velocity here.
	TMC5130_Write_Register(handle, TMC5130_VMAX, velocityMax);

	TMC5130_Write_Register(handle, TMC5130_XTARGET, position);
}

// Move by a given amount with a given velocity
// This function will write the absolute target position to *ticks
void TMC5130_Move(TMC5130TypeDef *handle, int32_t ticks, uint32_t velocityMax)
{
	// determine actual position and add numbers of ticks to move
	ticks += TMC5130_Read_Register(handle, TMC5130_XACTUAL);

	TMC5130_Move_To(handle, ticks, velocityMax);
}

// Target position
int32_t TMC5130_Get_Target_Position(TMC5130TypeDef *handle)
{
	return TMC5130_Read_Register(handle, TMC5130_XTARGET);
}

void TMC5130_Set_Target_Position(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_Write_Register(handle, TMC5130_XTARGET, value);
}

// Actual position
int32_t TMC5130_Get_Actual_Position(TMC5130TypeDef *handle)
{
	return TMC5130_Read_Register(handle, TMC5130_XACTUAL);
}

void TMC5130_Set_Actual_Position(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_Write_Register(handle, TMC5130_XACTUAL, value);
}

// Target speed
int32_t TMC5130_Get_Target_Speed(TMC5130TypeDef *handle)
{
	return TMC5130_Read_Register(handle, TMC5130_VMAX);
}

void TMC5130_Set_Target_Speed(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_Write_Register(handle, TMC5130_VMAX, value);
}

// Actual speed
int32_t TMC5130_Get_Actual_Speed(TMC5130TypeDef *handle)
{
	int32_t tmp = TMC5130_Read_Register(handle, TMC5130_VACTUAL);
	return CAST_Sn_TO_S32(tmp, 24);
}

// Maximum speed
int32_t TMC5130_Get_VEL_MAX(TMC5130TypeDef *handle)
{
	return vmax_position;
}

void TMC5130_Set_VEL_MAX(TMC5130TypeDef *handle, int32_t value)
{
	vmax_position = abs(value);
	if (TMC5130_Read_Register(handle, TMC5130_RAMPMODE) == TMC5130_MODE_POSITION)
		TMC5130_Write_Register(handle, TMC5130_VMAX, vmax_position);
}

// Maximum acceleration
int32_t TMC5130_Get_ACC_MAX(TMC5130TypeDef *handle)
{
	return TMC5130_Read_Register(handle, TMC5130_AMAX);
}

void TMC5130_Set_ACC_MAX(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_Write_Register(handle, TMC5130_AMAX, value);
}

// Maximum current
int32_t TMC5130_Get_I_MAX(TMC5130TypeDef *handle)
{
	return TMC5130_FIELD_READ(handle, TMC5130_IHOLD_IRUN, TMC5130_IRUN_MASK, TMC5130_IRUN_SHIFT);
}

void TMC5130_Set_I_MAX(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_FIELD_WRITE(handle, TMC5130_IHOLD_IRUN, TMC5130_IRUN_MASK, TMC5130_IRUN_SHIFT, value);
}

// Standby current
int32_t TMC5130_Get_I_STBY(TMC5130TypeDef *handle)
{
	return TMC5130_FIELD_READ(handle, TMC5130_IHOLD_IRUN, TMC5130_IHOLD_MASK, TMC5130_IHOLD_SHIFT);
}

void TMC5130_Set_I_STBY(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_FIELD_WRITE(handle, TMC5130_IHOLD_IRUN, TMC5130_IHOLD_MASK, TMC5130_IHOLD_SHIFT, value);
}

// Position reached flag
int32_t TMC5130_Get_Position_Flag(TMC5130TypeDef *handle)
{
	return (TMC5130_Read_Register(handle, TMC5130_RAMPSTAT) & TMC5130_RS_POSREACHED) ? 1 : 0;
}

// VREF
int32_t TMC5130_Get_VREF(TMC5130TypeDef *handle)
{
	return vref;
}

void TMC5130_Set_VREF(TMC5130TypeDef *handle, int32_t value)
{
	if ((uint32_t)value < VREF_FULLSCALE)
	{
		vref = value;
		//Timer.setDuty(TIMER_CHANNEL_1, vref * TIMER_MAX / VREF_FULLSCALE);
	}
	else
	{
		//errors |= TMC_ERROR_VALUE;
	}
}

// Right endstop
int32_t TMC5130_Get_Right_Endstop(TMC5130TypeDef *handle)
{
	return (TMC5130_Read_Register(handle, TMC5130_RAMPSTAT) & TMC5130_RS_STOPR) ? 0 : 1;
}

// Left endstop
int32_t TMC5130_Get_Left_Endstop(TMC5130TypeDef *handle)
{
	return (TMC5130_Read_Register(handle, TMC5130_RAMPSTAT) & TMC5130_RS_STOPL) ? 0 : 1;
}

// Automatic right stop
int32_t TMC5130_Get_Auto_Right_Stop(TMC5130TypeDef *handle)
{
	return (TMC5130_Read_Register(handle, TMC5130_SWMODE) & TMC5130_SW_STOPR_ENABLE) ? 1 : 0;
}

void TMC5130_Set_Auto_Right_Stop(TMC5130TypeDef *handle, int32_t value)
{
	int32_t buffer = TMC5130_Read_Register(handle, TMC5130_SWMODE);
	if (value == 0)
		TMC5130_Write_Register(handle, TMC5130_SWMODE, buffer | TMC5130_SW_STOPR_ENABLE);
	else
		TMC5130_Write_Register(handle, TMC5130_SWMODE, buffer & ~TMC5130_SW_STOPR_ENABLE);
}

// Automatic left stop
int32_t TMC5130_Get_Auto_Left_Stop(TMC5130TypeDef *handle)
{
	return (TMC5130_Read_Register(handle, TMC5130_SWMODE) & TMC5130_SW_STOPL_ENABLE) ? 1 : 0;
}

void TMC5130_Set_Auto_Left_Stop(TMC5130TypeDef *handle, int32_t value)
{
	int32_t buffer = TMC5130_Read_Register(handle, TMC5130_SWMODE);
	if (value == 0)
		TMC5130_Write_Register(handle, TMC5130_SWMODE, buffer | TMC5130_SW_STOPL_ENABLE);
	else
		TMC5130_Write_Register(handle, TMC5130_SWMODE, buffer & ~TMC5130_SW_STOPL_ENABLE);
}

// SW_MODE Register
int32_t TMC5130_Get_SW_MODE(TMC5130TypeDef *handle)
{
	return TMC5130_Read_Register(handle, TMC5130_SWMODE);
}

void TMC5130_Set_SW_MODE(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_Write_Register(handle, TMC5130_SWMODE, value);
}

// Acceleration A1
int32_t TMC5130_Get_ACC_A1(TMC5130TypeDef *handle)
{
	return TMC5130_Read_Register(handle, TMC5130_A1);
}

void TMC5130_Set_ACC_A1(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_Write_Register(handle, TMC5130_A1, value);
}

// Velocity V1
int32_t TMC5130_Get_VEL_V1(TMC5130TypeDef *handle)
{
	return TMC5130_Read_Register(handle, TMC5130_V1);
}

void TMC5130_Set_VEL_V1(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_Write_Register(handle, TMC5130_V1, value);
}

// Maximum Deceleration
int32_t TMC5130_Get_DEC_MAX(TMC5130TypeDef *handle)
{
	return TMC5130_Read_Register(handle, TMC5130_DMAX);
}

void TMC5130_Set_DEC_MAX(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_Write_Register(handle, TMC5130_DMAX, value);
}

// Deceleration D1
int32_t TMC5130_Get_DEC_D1(TMC5130TypeDef *handle)
{
	return TMC5130_Read_Register(handle, TMC5130_D1);
}

void TMC5130_Set_DEC_D1(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_Write_Register(handle, TMC5130_D1, value);
}

// Velocity VSTART
int32_t TMC5130_Get_VEL_Start(TMC5130TypeDef *handle)
{
	return TMC5130_Read_Register(handle, TMC5130_VSTART);
}

void TMC5130_Set_VEL_Start(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_Write_Register(handle, TMC5130_VSTART, value);
}

// Velocity VSTOP
int32_t TMC5130_Get_VEL_Stop(TMC5130TypeDef *handle)
{
	return TMC5130_Read_Register(handle, TMC5130_VSTOP);
}

void TMC5130_Set_VEL_Stop(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_Write_Register(handle, TMC5130_VSTOP, value);
}

// Waiting time after ramp down
int32_t TMC5130_Get_Wait_Time_Ramp_Down(TMC5130TypeDef *handle)
{
	return TMC5130_Read_Register(handle, TMC5130_TZEROWAIT);
}

void TMC5130_Set_Wait_Time_Ramp_Down(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_Write_Register(handle, TMC5130_TZEROWAIT, value);
}

// Speed threshold for high speed mode
int32_t TMC5130_Get_SE_HSpeed_THRESH(TMC5130TypeDef *handle)
{
	int32_t buffer = TMC5130_Read_Register(handle, TMC5130_THIGH);
	return MIN(0xFFFFF, (1 << 24) / ((buffer) ? buffer : 1));
}

void TMC5130_Set_SE_HSpeed_THRESH(TMC5130TypeDef *handle, int32_t value)
{
	int32_t tmp = MIN(0xFFFFF, (1 << 24) / ((value) ? value : 1));
	TMC5130_Write_Register(handle, TMC5130_THIGH, tmp);
}

// Minimum speed for switching to dcStep
int32_t TMC5130_Get_MIN_Speed_dcStep(TMC5130TypeDef *handle)
{
	return TMC5130_Read_Register(handle, TMC5130_VDCMIN);
}

void TMC5130_Set_MIN_Speed_dcStep(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_Write_Register(handle, TMC5130_VDCMIN, value);
}

// High speed chopper mode
int32_t TMC5130_Get_HS_Chopper_Mode(TMC5130TypeDef *handle)
{
	return TMC5130_FIELD_READ(handle, TMC5130_CHOPCONF, TMC5130_VHIGHCHM_MASK, TMC5130_VHIGHCHM_SHIFT);
}

void TMC5130_Set_HS_Chopper_Mode(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_FIELD_WRITE(handle, TMC5130_CHOPCONF, TMC5130_VHIGHCHM_MASK, TMC5130_VHIGHCHM_SHIFT, value);
}

// High speed fullstep mode
int32_t TMC5130_Get_FS_Mode(TMC5130TypeDef *handle)
{
	return TMC5130_FIELD_READ(handle, TMC5130_CHOPCONF, TMC5130_VHIGHFS_MASK, TMC5130_VHIGHFS_SHIFT);
}

void TMC5130_Set_FS_Mode(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_FIELD_WRITE(handle, TMC5130_CHOPCONF, TMC5130_VHIGHFS_MASK, TMC5130_VHIGHFS_SHIFT, value);
}

// Measured Speed
int32_t TMC5130_Get_Measured_Speed(TMC5130TypeDef *handle)
{
	//return handle->velocity;
	return 0;
}

// par::RampType
int32_t TMC5130_Get_Ramp_Type(TMC5130TypeDef *handle)
{
	return TMC5130_FIELD_READ(handle, TMC5130_RAMPMODE, TMC5130_RAMPMODE_MASK, TMC5130_RAMPMODE_SHIFT);
}

void TMC5130_Set_Ramp_Type(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_FIELD_WRITE(handle, TMC5130_RAMPMODE, TMC5130_RAMPMODE_MASK, TMC5130_RAMPMODE_SHIFT, value);
}

// Analog I Scale
int32_t TMC5130_Get_Analog_I_Scale(TMC5130TypeDef *handle)
{
	return TMC5130_FIELD_READ(handle, TMC5130_GCONF, TMC5130_I_SCALE_ANALOG_MASK, TMC5130_I_SCALE_ANALOG_SHIFT);
}

void TMC5130_Set_Analog_I_Scale(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_FIELD_WRITE(handle, TMC5130_GCONF, TMC5130_I_SCALE_ANALOG_MASK, TMC5130_I_SCALE_ANALOG_SHIFT, value);
}

// Internal RSense
int32_t TMC5130_Get_Internal_RSense(TMC5130TypeDef *handle)
{
	return TMC5130_FIELD_READ(handle, TMC5130_GCONF, TMC5130_INTERNAL_RSENSE_MASK, TMC5130_INTERNAL_RSENSE_SHIFT);
}

void TMC5130_Set_Internal_RSense(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_FIELD_WRITE(handle, TMC5130_GCONF, TMC5130_INTERNAL_RSENSE_MASK, TMC5130_INTERNAL_RSENSE_SHIFT, value);
}

// Microstep Resolution
int32_t TMC5130_Get_Microstep_Resolution(TMC5130TypeDef *handle)
{
	return 256 >> TMC5130_FIELD_READ(handle, TMC5130_CHOPCONF, TMC5130_MRES_MASK, TMC5130_MRES_SHIFT);
}

void TMC5130_Set_Microstep_Resolution(TMC5130TypeDef *handle, int32_t value)
{
	switch (value)
	{
	case 1:
		value = 8;

	case 2:
		value = 7;

	case 4:
		value = 6;

	case 8:
		value = 5;

	case 16:
		value = 4;

	case 32:
		value = 3;

	case 64:
		value = 2;

	case 128:
		value = 1;

	case 256:
		value = 0;

	default:
		value = -1;
	}

	if (value != -1)
	{
		TMC5130_FIELD_WRITE(handle, TMC5130_CHOPCONF, TMC5130_MRES_MASK, TMC5130_MRES_SHIFT, value);
	}
	else
	{
		//errors |= TMC_ERROR_VALUE;
	}
}

// Chopper blank time
int32_t TMC5130_Get_Chopper_Blank_Time(TMC5130TypeDef *handle)
{
	return TMC5130_FIELD_READ(handle, TMC5130_CHOPCONF, TMC5130_TBL_MASK, TMC5130_TBL_SHIFT);
}

void TMC5130_Set_Chopper_Blank_Time(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_FIELD_WRITE(handle, TMC5130_CHOPCONF, TMC5130_TBL_MASK, TMC5130_TBL_SHIFT, value);
}

// Constant TOff Mode
int32_t TMC5130_Get_CONST_Toff_Mode(TMC5130TypeDef *handle)
{
	return TMC5130_FIELD_READ(handle, TMC5130_CHOPCONF, TMC5130_CHM_MASK, TMC5130_CHM_SHIFT);
}

void TMC5130_Set_CONST_Toff_Mode(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_FIELD_WRITE(handle, TMC5130_CHOPCONF, TMC5130_CHM_MASK, TMC5130_CHM_SHIFT, value);
}

// Disable fast decay comparator
int32_t TMC5130_Get_FD_COMP(TMC5130TypeDef *handle)
{
	return TMC5130_FIELD_READ(handle, TMC5130_CHOPCONF, TMC5130_DISFDCC_MASK, TMC5130_DISFDCC_SHIFT);
}

void TMC5130_Set_FD_COMP(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_FIELD_WRITE(handle, TMC5130_CHOPCONF, TMC5130_DISFDCC_MASK, TMC5130_DISFDCC_SHIFT, value);
}

// Chopper hysteresis end / fast decay time
int32_t TMC5130_Get_Chopper_HYS_FD_Time(TMC5130TypeDef *handle)
{
	int32_t buffer = TMC5130_Read_Register(handle, TMC5130_CHOPCONF);

	if (buffer & (1 << 14))
	{
		return TMC5130_FIELD_READ(handle, TMC5130_CHOPCONF, TMC5130_HEND_MASK, TMC5130_HEND_SHIFT);
	}
	else
	{
		return ((buffer >> 4) & 0x07) | (buffer & (1 << 11)) ? (1 << 3) : 0;
	}
}

void TMC5130_Set_Chopper_HYS_FD_Time(TMC5130TypeDef *handle, int32_t value)
{
	int32_t buffer = TMC5130_Read_Register(handle, TMC5130_CHOPCONF);

	if (buffer & (1 << 14))
	{
		TMC5130_FIELD_WRITE(handle, TMC5130_CHOPCONF, TMC5130_HEND_MASK, TMC5130_HEND_SHIFT, value);
	}
	else
	{
		if (value & (1 << 3))
			buffer |= (0x01 << 11);
		else
			buffer &= ~(0x01 << 11);

		buffer &= ~(0x07 << 4);
		buffer |= (value & 0x0F) << 4;

		TMC5130_Write_Register(handle, TMC5130_CHOPCONF, buffer);
	}
}

// Chopper hysteresis start / sine wave offset
int32_t TMC5130_Get_Chopper_HYS_Sine_Offset(TMC5130TypeDef *handle)
{
	int32_t buffer = TMC5130_Read_Register(handle, TMC5130_CHOPCONF);

	if (buffer & (1 << 14))
	{
		return TMC5130_FIELD_READ(handle, TMC5130_CHOPCONF, TMC5130_HSTRT_MASK, TMC5130_HSTRT_SHIFT);
	}
	else
	{
		return ((buffer >> 7) & 0x0F) | (buffer & (1 << 11)) ? 1 << 3 : 0;
	}
}

void TMC5130_Set_Chopper_HYS_Sine_Offset(TMC5130TypeDef *handle, int32_t value)
{
	if (TMC5130_Read_Register(handle, TMC5130_CHOPCONF) & (1 << 14))
	{
		TMC5130_FIELD_WRITE(handle, TMC5130_CHOPCONF, TMC5130_HSTRT_MASK, TMC5130_HSTRT_SHIFT, value);
	}
	else
	{
		TMC5130_FIELD_WRITE(handle, TMC5130_CHOPCONF, TMC5130_OFFSET_MASK, TMC5130_OFFSET_SHIFT, value);
	}
}

// Chopper off time
int32_t TMC5130_Get_Chopper_TOff(TMC5130TypeDef *handle)
{
	return TMC5130_FIELD_READ(handle, TMC5130_CHOPCONF, TMC5130_TOFF_MASK, TMC5130_TOFF_SHIFT);
}

void TMC5130_Set_Chopper_TOff(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_FIELD_WRITE(handle, TMC5130_CHOPCONF, TMC5130_TOFF_MASK, TMC5130_TOFF_SHIFT, value);
}

// smartEnergy current minimum (SEIMIN)
int32_t TMC5130_Get_SE_I_MIN(TMC5130TypeDef *handle)
{
	return TMC5130_FIELD_READ(handle, TMC5130_COOLCONF, TMC5130_SEIMIN_MASK, TMC5130_SEIMIN_SHIFT);
}

void TMC5130_Set_SE_I_MIN(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_FIELD_WRITE(handle, TMC5130_COOLCONF, TMC5130_SEIMIN_MASK, TMC5130_SEIMIN_SHIFT, value);
}

// smartEnergy current down step
int32_t TMC5130_Get_SE_I_DWN_Step(TMC5130TypeDef *handle)
{
	return TMC5130_FIELD_READ(handle, TMC5130_COOLCONF, TMC5130_SEDN_MASK, TMC5130_SEDN_SHIFT);
}

void TMC5130_Set_SE_I_DWN_Step(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_FIELD_WRITE(handle, TMC5130_COOLCONF, TMC5130_SEDN_MASK, TMC5130_SEDN_SHIFT, value);
}

// smartEnergy hysteresis
int32_t TMC5130_Get_SE_HYS(TMC5130TypeDef *handle)
{
	return TMC5130_FIELD_READ(handle, TMC5130_COOLCONF, TMC5130_SEMAX_MASK, TMC5130_SEMAX_SHIFT);
}

void TMC5130_Set_SE_HYS(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_FIELD_WRITE(handle, TMC5130_COOLCONF, TMC5130_SEMAX_MASK, TMC5130_SEMAX_SHIFT, value);
}

// smartEnergy current up step
int32_t TMC5130_Get_SE_I_UP_Step(TMC5130TypeDef *handle)
{
	return TMC5130_FIELD_READ(handle, TMC5130_COOLCONF, TMC5130_SEUP_MASK, TMC5130_SEUP_SHIFT);
}

void TMC5130_Set_SE_I_UP_Step(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_FIELD_WRITE(handle, TMC5130_COOLCONF, TMC5130_SEUP_MASK, TMC5130_SEUP_SHIFT, value);
}

// smartEnergy hysteresis start
int32_t TMC5130_Get_SE_HYS_Start(TMC5130TypeDef *handle)
{
	return TMC5130_FIELD_READ(handle, TMC5130_COOLCONF, TMC5130_SEMIN_MASK, TMC5130_SEMIN_SHIFT);
}

void TMC5130_Set_SE_HYS_Start(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_FIELD_WRITE(handle, TMC5130_COOLCONF, TMC5130_SEMIN_MASK, TMC5130_SEMIN_SHIFT, value);
}

// stallGuard2 filter enable
int32_t TMC5130_Get_stallGuard2_Filter(TMC5130TypeDef *handle)
{
	return TMC5130_FIELD_READ(handle, TMC5130_COOLCONF, TMC5130_SFILT_MASK, TMC5130_SFILT_SHIFT);
}

void TMC5130_Set_stallGuard2_Filter(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_FIELD_WRITE(handle, TMC5130_COOLCONF, TMC5130_SFILT_MASK, TMC5130_SFILT_SHIFT, value);
}

// stallGuard2 threshold
int32_t TMC5130_Get_stallGuard2_THRESH(TMC5130TypeDef *handle)
{
	int32_t tmp = TMC5130_FIELD_READ(handle, TMC5130_COOLCONF, TMC5130_SGT_MASK, TMC5130_SGT_SHIFT);
	return CAST_Sn_TO_S32(tmp, 7);
}

void TMC5130_Set_stallGuard2_THRESH(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_FIELD_WRITE(handle, TMC5130_COOLCONF, TMC5130_SGT_MASK, TMC5130_SGT_SHIFT, value);
}

// VSense
int32_t TMC5130_Get_VSense(TMC5130TypeDef *handle)
{
	return TMC5130_FIELD_READ(handle, TMC5130_CHOPCONF, TMC5130_VSENSE_MASK, TMC5130_VSENSE_SHIFT);
}

void TMC5130_Set_VSense(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_FIELD_WRITE(handle, TMC5130_CHOPCONF, TMC5130_VSENSE_MASK, TMC5130_VSENSE_SHIFT, value);
}

// smartEnergy actual current
int32_t TMC5130_Get_SE_Actual_I(TMC5130TypeDef *handle)
{
	return TMC5130_FIELD_READ(handle, TMC5130_DRVSTATUS, TMC5130_CS_ACTUAL_MASK, TMC5130_CS_ACTUAL_SHIFT);
}

// smartEnergy stall velocity
// this function sort of doubles with 182 but is necessary to allow cross chip compliance
int32_t TMC5130_Get_SE_VEL_Stall(TMC5130TypeDef *handle)
{
	if (TMC5130_FIELD_READ(handle, TMC5130_SWMODE, TMC5130_SG_STOP_MASK, TMC5130_SG_STOP_SHIFT))
	{
		int32_t buffer = TMC5130_Read_Register(handle, TMC5130_TCOOLTHRS);
		return MIN(0xFFFFF, (1 << 24) / ((buffer) ? buffer : 1));
	}
	else
		return 0;
}

void TMC5130_Set_SE_VEL_Stall(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_FIELD_WRITE(handle, TMC5130_SWMODE, TMC5130_SG_STOP_MASK, TMC5130_SG_STOP_SHIFT, (value) ? 1 : 0);
	int32_t tmp = MIN(0xFFFFF, (1 << 24) / ((value) ? value : 1));
	TMC5130_Write_Register(handle, TMC5130_TCOOLTHRS, tmp);
}

// smartEnergy threshold speed
int32_t TMC5130_Get_SE_Speed_THRESH(TMC5130TypeDef *handle)
{
	int32_t buffer = TMC5130_Read_Register(handle, TMC5130_TCOOLTHRS);
	return MIN(0xFFFFF, (1 << 24) / ((buffer) ? buffer : 1));
}

void TMC5130_Set_SE_Speed_THRESH(TMC5130TypeDef *handle, int32_t value)
{
	int32_t buffer = MIN(0xFFFFF, (1 << 24) / ((value) ? value : 1));
	TMC5130_Write_Register(handle, TMC5130_TCOOLTHRS, buffer);
}

// Random TOff mode
int32_t TMC5130_Get_Random_TOff_Mode(TMC5130TypeDef *handle)
{
	return TMC5130_FIELD_READ(handle, TMC5130_CHOPCONF, TMC5130_RNDTF_MASK, TMC5130_RNDTF_SHIFT);
}

void TMC5130_Set_Random_TOff_Mode(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_FIELD_WRITE(handle, TMC5130_CHOPCONF, TMC5130_RNDTF_MASK, TMC5130_RNDTF_SHIFT, value);
}

// Chopper synchronization
int32_t TMC5130_Get_Chopper_SYNC(TMC5130TypeDef *handle)
{
	return TMC5130_FIELD_READ(handle, TMC5130_CHOPCONF, TMC5130_SYNC_MASK, TMC5130_SYNC_SHIFT);
}

void TMC5130_Set_Chopper_SYNC(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_FIELD_WRITE(handle, TMC5130_CHOPCONF, TMC5130_SYNC_MASK, TMC5130_SYNC_SHIFT, value);
}

// PWM threshold speed
int32_t TMC5130_Get_PWM_THRESH_Speed(TMC5130TypeDef *handle)
{
	int32_t buffer = TMC5130_Read_Register(handle, TMC5130_TPWMTHRS);
	return MIN(0xFFFFF, (1 << 24) / ((buffer) ? buffer : 1));
}

void TMC5130_Set_PWM_THRESH_Speed(TMC5130TypeDef *handle, int32_t value)
{
	int32_t tmp = MIN(0xFFFFF, (1 << 24) / ((value) ? value : 1));
	TMC5130_Write_Register(handle, TMC5130_TPWMTHRS, tmp);
}

// PWM gradient
int32_t TMC5130_Get_PWM_Gradient(TMC5130TypeDef *handle)
{
	return TMC5130_FIELD_READ(handle, TMC5130_PWMCONF, TMC5130_PWM_GRAD_MASK, TMC5130_PWM_GRAD_SHIFT);
}

void TMC5130_Set_PWM_Gradient(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_FIELD_WRITE(handle, TMC5130_PWMCONF, TMC5130_PWM_GRAD_MASK, TMC5130_PWM_GRAD_SHIFT, value);
	TMC5130_FIELD_WRITE(handle, TMC5130_GCONF, TMC5130_EN_PWM_MODE_MASK, TMC5130_EN_PWM_MODE_SHIFT, (value) ? 1 : 0);
}

// PWM amplitude
int32_t TMC5130_Get_PWM_Amplitude(TMC5130TypeDef *handle)
{
	return TMC5130_FIELD_READ(handle, TMC5130_PWMCONF, TMC5130_PWM_AMPL_MASK, TMC5130_PWM_AMPL_SHIFT);
}

void TMC5130_Set_PWM_Amplitude(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_FIELD_WRITE(handle, TMC5130_PWMCONF, TMC5130_PWM_AMPL_MASK, TMC5130_PWM_AMPL_SHIFT, value);
}

// PWM frequency
int32_t TMC5130_Get_PWM_FREQ(TMC5130TypeDef *handle)
{
	return TMC5130_FIELD_READ(handle, TMC5130_PWMCONF, TMC5130_PWM_FREQ_MASK, TMC5130_PWM_FREQ_SHIFT);
}

void TMC5130_Set_PWM_FREQ(TMC5130TypeDef *handle, int32_t value)
{
	if (value >= 0 && value < 4)
	{
		TMC5130_FIELD_WRITE(handle, TMC5130_PWMCONF, TMC5130_PWM_FREQ_MASK, TMC5130_PWM_FREQ_SHIFT, value);
	}
	else
	{
		//errors |= TMC_ERROR_VALUE;
	}
}

// PWM autoscale
int32_t TMC5130_Get_PWM_Auto_Scale(TMC5130TypeDef *handle)
{
	return TMC5130_FIELD_READ(handle, TMC5130_PWMCONF, TMC5130_PWM_AUTOSCALE_MASK, TMC5130_PWM_AUTOSCALE_SHIFT);
}

void TMC5130_Set_PWM_Auto_Scale(TMC5130TypeDef *handle, int32_t value)
{
	if (value >= 0 && value < 2)
	{
		TMC5130_FIELD_WRITE(handle, TMC5130_PWMCONF, TMC5130_PWM_AUTOSCALE_MASK, TMC5130_PWM_AUTOSCALE_SHIFT, value);
	}
	else
	{
		//errors |= TMC_ERROR_VALUE;
	}
}

// Freewheeling mode
int32_t TMC5130_Get_Freewheeling_Mode(TMC5130TypeDef *handle)
{
	return TMC5130_FIELD_READ(handle, TMC5130_PWMCONF, TMC5130_FREEWHEEL_MASK, TMC5130_FREEWHEEL_SHIFT);
}

void TMC5130_Set_Freewheeling_Mode(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_FIELD_WRITE(handle, TMC5130_PWMCONF, TMC5130_FREEWHEEL_MASK, TMC5130_FREEWHEEL_SHIFT, value);
}

// Load value
int32_t TMC5130_Get_Load_Value(TMC5130TypeDef *handle)
{
	return TMC5130_FIELD_READ(handle, TMC5130_DRVSTATUS, TMC5130_SG_RESULT_MASK, TMC5130_SG_RESULT_SHIFT);
}

// Encoder position
int32_t TMC5130_Get_Encoder_Position(TMC5130TypeDef *handle)
{
	return TMC5130_Read_Register(handle, TMC5130_XENC);
}

void TMC5130_Set_Encoder_Position(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_Write_Register(handle, TMC5130_XENC, value);
}

// Encoder Resolution
int32_t TMC5130_Get_Encoder_Resolution(TMC5130TypeDef *handle)
{
	return TMC5130_Read_Register(handle, TMC5130_ENC_CONST);
}

void TMC5130_Set_Encoder_Resolution(TMC5130TypeDef *handle, int32_t value)
{
	TMC5130_Write_Register(handle, TMC5130_ENC_CONST, value);
}

#include "tmc2130.h"
#include "tmc2130_io.h"
#include "TMC2130_Mask_Shift.h"
#include "TMC2130_Register.h"
#include "TMC2130_Constants.h"
#include "delay_us.h"

void TMC2130_Set_Max_Current(TMC2130TypeDef *handle, int32_t value)
{
    TMC2130_FIELD_UPDATE(handle, TMC2130_IHOLD_IRUN, TMC2130_IRUN_MASK, TMC2130_IRUN_SHIFT, value);
}
int32_t TMC2130_Get_Max_Current(TMC2130TypeDef *handle)
{
    return TMC2130_FIELD_READ(handle, TMC2130_IHOLD_IRUN, TMC2130_IRUN_MASK, TMC2130_IRUN_SHIFT);
}

void TMC2130_Set_Standby_Current(TMC2130TypeDef *handle, int32_t value)
{
    TMC2130_FIELD_UPDATE(handle, TMC2130_IHOLD_IRUN, TMC2130_IHOLD_MASK, TMC2130_IHOLD_SHIFT, value);
}
int32_t TMC2130_Get_Standby_Current(TMC2130TypeDef *handle)
{
    return TMC2130_FIELD_READ(handle, TMC2130_IHOLD_IRUN, TMC2130_IHOLD_MASK, TMC2130_IHOLD_SHIFT);
}

void TMC2130_Set_Power_Down(TMC2130TypeDef *handle, int32_t value)
{
    TMC2130_Write_Register(handle, TMC2130_TPOWERDOWN, value);
}

// Speed threshold for high speed mode
void TMC2130_Set_THIGH(TMC2130TypeDef *handle, int32_t value)
{
    value = MIN(0xFFFFF, (1 << 24) / ((value) ? value : 1));
    TMC2130_Write_Register(handle, TMC2130_THIGH, value);
}
int32_t TMC2130_Get_THIGH(TMC2130TypeDef *handle)
{
    int32_t tempValue;
    tempValue = TMC2130_Read_Register(handle, TMC2130_THIGH);
    return MIN(0xFFFFF, (1 << 24) / ((tempValue) ? tempValue : 1));
}

// Minimum speed for switching to dcStep
void TMC2130_Set_MIN_Speed_dcStep(TMC2130TypeDef *handle, int32_t value)
{
    TMC2130_Write_Register(handle, TMC2130_VDCMIN, value);
}
int32_t TMC2130_Get_MIN_Speed_dcStep(TMC2130TypeDef *handle)
{
    return TMC2130_Read_Register(handle, TMC2130_VDCMIN);
}

// High speed fullstep mode
void TMC2130_Set_High_Speed_Full_Step_Mode(TMC2130TypeDef *handle,
                                           int32_t value)
{
    TMC2130_FIELD_UPDATE(handle, TMC2130_CHOPCONF, TMC2130_VHIGHFS_MASK, TMC2130_VHIGHFS_SHIFT, value);
}
int32_t TMC2130_Get_High_Speed_Full_Step_Mode(TMC2130TypeDef *handle)
{
    return TMC2130_FIELD_READ(handle, TMC2130_CHOPCONF, TMC2130_VHIGHFS_MASK, TMC2130_VHIGHFS_SHIFT);
}

// High speed chopper mode
void TMC2130_Set_High_Speed_Chopper_Mode(TMC2130TypeDef *handle, int32_t value)
{
    TMC2130_FIELD_UPDATE(handle, TMC2130_CHOPCONF, TMC2130_VHIGHCHM_MASK, TMC2130_VHIGHCHM_SHIFT, value);
}
int32_t TMC2130_Get_High_Speed_Chopper_Mode(TMC2130TypeDef *handle)
{
    return TMC2130_FIELD_READ(handle, TMC2130_CHOPCONF, TMC2130_VHIGHCHM_MASK, TMC2130_VHIGHCHM_SHIFT);
}

// Internal RSense
void TMC2130_Set_Internal_RSense(TMC2130TypeDef *handle, int32_t value)
{
    TMC2130_FIELD_UPDATE(handle, TMC2130_GCONF, TMC2130_INTERNAL_RSENSE_MASK, TMC2130_INTERNAL_RSENSE_SHIFT, value);
}
int32_t TMC2130_Get_Internal_RSense(TMC2130TypeDef *handle)
{
    return TMC2130_FIELD_READ(handle, TMC2130_GCONF, TMC2130_INTERNAL_RSENSE_MASK, TMC2130_INTERNAL_RSENSE_SHIFT);
}

// I_scale_analog
void TMC2130_Set_I_Scale_Analog(TMC2130TypeDef *handle, int32_t value)
{
    TMC2130_FIELD_UPDATE(handle, TMC2130_GCONF, TMC2130_I_SCALE_ANALOG_MASK, TMC2130_I_SCALE_ANALOG_SHIFT, value);
}

int32_t TMC2130_Get_I_Scale_Analog(TMC2130TypeDef *handle)
{
    return TMC2130_FIELD_READ(handle, TMC2130_GCONF, TMC2130_I_SCALE_ANALOG_MASK, TMC2130_I_SCALE_ANALOG_SHIFT);
}

// Microstep Resolution
void TMC2130_Set_Microstep(TMC2130TypeDef *handle, int32_t value)
{
    switch (value)
    {
    case 1:
        value = 8;
        break;
    case 2:
        value = 7;
        break;
    case 4:
        value = 6;
        break;
    case 8:
        value = 5;
        break;
    case 16:
        value = 4;
        break;
    case 32:
        value = 3;
        break;
    case 64:
        value = 2;
        break;
    case 128:
        value = 1;
        break;
    case 256:
        value = 0;
        break;
    default:
        value = -1;
        break;
    }

    if (value != -1)
    {
        TMC2130_FIELD_UPDATE(handle, TMC2130_CHOPCONF, TMC2130_MRES_MASK, TMC2130_MRES_SHIFT, value);
    }
}
int32_t TMC2130_Get_Microstep(TMC2130TypeDef *handle)
{
    return 256 >> TMC2130_FIELD_READ(handle, TMC2130_CHOPCONF, TMC2130_MRES_MASK, TMC2130_MRES_SHIFT);
}

// Chopper blank time
void TMC2130_Set_Chopper_Blank_Time(TMC2130TypeDef *handle, int32_t value)
{
    TMC2130_FIELD_UPDATE(handle, TMC2130_CHOPCONF, TMC2130_TBL_MASK, TMC2130_TBL_SHIFT, value);
}
int32_t TMC2130_Get_Chopper_Blank_Time(TMC2130TypeDef *handle)
{
    return TMC2130_FIELD_READ(handle, TMC2130_CHOPCONF, TMC2130_TBL_MASK, TMC2130_TBL_SHIFT);
}

// Constant TOff Mode
void TMC2130_Set_Constant_Toff_Mode(TMC2130TypeDef *handle, int32_t value)
{
    TMC2130_FIELD_UPDATE(handle, TMC2130_CHOPCONF, TMC2130_CHM_MASK, TMC2130_CHM_SHIFT, value);
}
int32_t TMC2130_Get_Constant_Toff_Mode(TMC2130TypeDef *handle)
{
    return TMC2130_FIELD_READ(handle, TMC2130_CHOPCONF, TMC2130_CHM_MASK, TMC2130_CHM_SHIFT);
}

// Disable fast decay comparator
void TMC2130_Set_Fast_Decay_Comparator(TMC2130TypeDef *handle, int32_t value)
{
    TMC2130_FIELD_UPDATE(handle, TMC2130_CHOPCONF, TMC2130_DISFDCC_MASK, TMC2130_DISFDCC_SHIFT, value);
}
int32_t TMC2130_Get_Fast_Decay_Comparator(TMC2130TypeDef *handle)
{
    return TMC2130_FIELD_READ(handle, TMC2130_CHOPCONF, TMC2130_DISFDCC_MASK, TMC2130_DISFDCC_SHIFT);
}

// Chopper hysteresis end / fast decay time
void TMC2130_Set_Chopper_Hysteresis_Time(TMC2130TypeDef *handle, int32_t value)
{
    int32_t tempValue;
    (void)tempValue;
    if (TMC2130_Read_Register(handle, TMC2130_CHOPCONF) & (1 << 14))
    {
        TMC2130_FIELD_UPDATE(handle, TMC2130_CHOPCONF, TMC2130_HEND_MASK, TMC2130_HEND_SHIFT, value);
    }
    else
    {
        tempValue = TMC2130_Read_Register(handle, TMC2130_CHOPCONF);

        TMC2130_FIELD_UPDATE(handle, TMC2130_CHOPCONF, TMC2130_TFD_3_MASK, TMC2130_TFD_3_SHIFT, (value & (1 << 3)) ? 1 : 0);
        TMC2130_FIELD_UPDATE(handle, TMC2130_CHOPCONF, TMC2130_TFD_ALL_MASK, TMC2130_TFD_ALL_SHIFT, value);
    }
}
int32_t TMC2130_Get_Chopper_Hysteresis_Time(TMC2130TypeDef *handle)
{
    int32_t tempValue;
    int32_t retValue;
    if (TMC2130_Read_Register(handle, TMC2130_CHOPCONF) & (1 << 14))
    {
        retValue = TMC2130_FIELD_READ(handle, TMC2130_CHOPCONF, TMC2130_HEND_MASK, TMC2130_HEND_SHIFT);
    }
    else
    {
        tempValue = TMC2130_Read_Register(handle, TMC2130_CHOPCONF);
        retValue = (TMC2130_Read_Register(handle, TMC2130_CHOPCONF) >> 4) & 0x07;
        if (tempValue & (1 << 11))
        {
            retValue |= 1 << 3;
        }
    }

    return retValue;
}

// Chopper hysteresis start / sine wave offset
void TMC2130_Set_Chopper_Hysteresis_Offset(TMC2130TypeDef *handle,
                                           int32_t value)
{
    if (TMC2130_Read_Register(handle, TMC2130_CHOPCONF) & (1 << 14))
    {
        TMC2130_FIELD_UPDATE(handle, TMC2130_CHOPCONF, TMC2130_HSTRT_MASK, TMC2130_HSTRT_SHIFT, value);
    }
    else
    {
        TMC2130_FIELD_UPDATE(handle, TMC2130_CHOPCONF, TMC2130_OFFSET_MASK, TMC2130_OFFSET_SHIFT, value);
    }
}
int32_t TMC2130_Get_Chopper_Hysteresis_Offset(TMC2130TypeDef *handle)
{
    int32_t tempValue;
    int32_t retValue;
    if (TMC2130_Read_Register(handle, TMC2130_CHOPCONF) & (1 << 14))
    {
        retValue = TMC2130_FIELD_READ(handle, TMC2130_CHOPCONF, TMC2130_HSTRT_MASK, TMC2130_HSTRT_SHIFT);
    }
    else
    {
        tempValue = TMC2130_Read_Register(handle, TMC2130_CHOPCONF);
        retValue = (TMC2130_Read_Register(handle, TMC2130_CHOPCONF) >> 7) & 0x0F;
        if (tempValue & (1 << 11))
        {
            retValue |= 1 << 3;
        }
    }

    return retValue;
}

// Chopper off time
void TMC2130_Set_Chopper_Off_Time(TMC2130TypeDef *handle, int32_t value)
{
    TMC2130_FIELD_UPDATE(handle, TMC2130_CHOPCONF, TMC2130_TOFF_MASK, TMC2130_TOFF_SHIFT, value);
}
int32_t TMC2130_Get_Chopper_Off_Time(TMC2130TypeDef *handle)
{
    return TMC2130_FIELD_READ(handle, TMC2130_CHOPCONF, TMC2130_TOFF_MASK, TMC2130_TOFF_SHIFT);
}

// smartEnergy current minimum (SEIMIN)
void TMC2130_Set_SEMIN_I(TMC2130TypeDef *handle, int32_t value)
{
    TMC2130_FIELD_UPDATE(handle, TMC2130_COOLCONF, TMC2130_SEIMIN_MASK, TMC2130_SEIMIN_SHIFT, value);
}
int32_t TMC2130_Get_SEMIN_I(TMC2130TypeDef *handle)
{
    return TMC2130_FIELD_READ(handle, TMC2130_COOLCONF, TMC2130_SEIMIN_MASK, TMC2130_SEIMIN_SHIFT);
}

// smartEnergy current down step
void TMC2130_Set_SEDN_I(TMC2130TypeDef *handle, int32_t value)
{
    TMC2130_FIELD_UPDATE(handle, TMC2130_COOLCONF, TMC2130_SEDN_MASK, TMC2130_SEDN_SHIFT, value);
}
int32_t TMC2130_Get_SEDN_I(TMC2130TypeDef *handle)
{
    return TMC2130_FIELD_READ(handle, TMC2130_COOLCONF, TMC2130_SEDN_MASK, TMC2130_SEDN_SHIFT);
}

// smartEnergy hysteresis
void TMC2130_Set_SEMAX(TMC2130TypeDef *handle, int32_t value)
{
    TMC2130_FIELD_UPDATE(handle, TMC2130_COOLCONF, TMC2130_SEMAX_MASK, TMC2130_SEMAX_SHIFT, value);
}
int32_t TMC2130_Get_SEMAX(TMC2130TypeDef *handle)
{
    return TMC2130_FIELD_READ(handle, TMC2130_COOLCONF, TMC2130_SEMAX_MASK, TMC2130_SEMAX_SHIFT);
}

// smartEnergy current up step
void TMC2130_Set_SEUP_I(TMC2130TypeDef *handle, int32_t value)
{
    TMC2130_FIELD_UPDATE(handle, TMC2130_COOLCONF, TMC2130_SEUP_MASK, TMC2130_SEUP_SHIFT, value);
}
int32_t TMC2130_Get_SEUP_I(TMC2130TypeDef *handle)
{
    return TMC2130_FIELD_READ(handle, TMC2130_COOLCONF, TMC2130_SEUP_MASK, TMC2130_SEUP_SHIFT);
}

// smartEnergy hysteresis start
void TMC2130_Set_SEMIN(TMC2130TypeDef *handle, int32_t value)
{
    TMC2130_FIELD_UPDATE(handle, TMC2130_COOLCONF, TMC2130_SEMIN_MASK, TMC2130_SEMIN_SHIFT, value);
}
int32_t TMC2130_Get_SEMIN(TMC2130TypeDef *handle)
{
    return TMC2130_FIELD_READ(handle, TMC2130_COOLCONF, TMC2130_SEMIN_MASK, TMC2130_SEMIN_SHIFT);
}

// stallGuard2 filter enable
void TMC2130_Set_Stall_Flter(TMC2130TypeDef *handle, int32_t value)
{
    TMC2130_FIELD_UPDATE(handle, TMC2130_COOLCONF, TMC2130_SFILT_MASK, TMC2130_SFILT_SHIFT, value);
}
int32_t TMC2130_Get_Stall_Flter(TMC2130TypeDef *handle)
{
    return TMC2130_FIELD_READ(handle, TMC2130_COOLCONF, TMC2130_SFILT_MASK, TMC2130_SFILT_SHIFT);
}

// stallGuard2 threshold
void TMC2130_Set_Stall_Threshold(TMC2130TypeDef *handle, int32_t value)
{
    TMC2130_FIELD_UPDATE(handle, TMC2130_COOLCONF, TMC2130_SGT_MASK, TMC2130_SGT_SHIFT, value);
}
int32_t TMC2130_Get_Stall_Threshold(TMC2130TypeDef *handle)
{
    int32_t retvalue;
    retvalue = TMC2130_FIELD_READ(handle, TMC2130_COOLCONF, TMC2130_SGT_MASK, TMC2130_SGT_SHIFT);
    retvalue = CAST_Sn_TO_S32(retvalue, 7);
    return retvalue;
}

// VSense
void TMC2130_Set_Vsense(TMC2130TypeDef *handle, int32_t value)
{
    TMC2130_FIELD_UPDATE(handle, TMC2130_CHOPCONF, TMC2130_VSENSE_MASK, TMC2130_VSENSE_SHIFT, value);
}
int32_t TMC2130_Get_Vsense(TMC2130TypeDef *handle)
{
    return TMC2130_FIELD_READ(handle, TMC2130_CHOPCONF, TMC2130_VSENSE_MASK, TMC2130_VSENSE_SHIFT);
}

// smartEnergy actual current
int32_t TMC2130_Get_SE_Actual_Current(TMC2130TypeDef *handle)
{
    return TMC2130_FIELD_READ(handle, TMC2130_DRV_STATUS, TMC2130_CS_ACTUAL_MASK, TMC2130_CS_ACTUAL_SHIFT);
}

// smartEnergy threshold speed
void TMC2130_Set_TCOOLTHRS(TMC2130TypeDef *handle, int32_t value)
{
    value = MIN(0xFFFFF, (1 << 24) / ((value) ? value : 1));
    TMC2130_Write_Register(handle, TMC2130_TCOOLTHRS, value);
}
int32_t TMC2130_Get_TCOOLTHRS(TMC2130TypeDef *handle)
{
    int32_t retvalue;
    int32_t tempValue = TMC2130_Read_Register(handle, TMC2130_TCOOLTHRS);
    retvalue = MIN(0xFFFFF, (1 << 24) / ((tempValue) ? tempValue : 1));
    return retvalue;
}

// Random TOff mode
void TMC2130_Set_Random_Toff_Mode(TMC2130TypeDef *handle, int32_t value)
{
    TMC2130_FIELD_UPDATE(handle, TMC2130_CHOPCONF, TMC2130_RNDTF_MASK, TMC2130_RNDTF_SHIFT, value);
}
int32_t TMC2130_Get_Random_Toff_Mode(TMC2130TypeDef *handle)
{
    return TMC2130_FIELD_READ(handle, TMC2130_CHOPCONF, TMC2130_RNDTF_MASK, TMC2130_RNDTF_SHIFT);
}

// Chopper synchronization
void TMC2130_Set_Chopper_Sync(TMC2130TypeDef *handle, int32_t value)
{
    TMC2130_FIELD_UPDATE(handle, TMC2130_CHOPCONF, TMC2130_SYNC_MASK, TMC2130_SYNC_SHIFT, value);
}
int32_t TMC2130_Get_Chopper_Sync(TMC2130TypeDef *handle)
{
    return TMC2130_FIELD_READ(handle, TMC2130_CHOPCONF, TMC2130_SYNC_MASK, TMC2130_SYNC_SHIFT);
}

// PWM threshold speed
void TMC2130_Set_PWM_Threshold(TMC2130TypeDef *handle, int32_t value)
{
    value = MIN(0xFFFFF, (1 << 24) / ((value) ? value : 1));
    TMC2130_Write_Register(handle, TMC2130_TPWMTHRS, value);
}
int32_t TMC2130_Get_PWM_Threshold(TMC2130TypeDef *handle)
{
    int32_t retvalue;
    int32_t tempValue = TMC2130_Read_Register(handle, TMC2130_TPWMTHRS);
    retvalue = MIN(0xFFFFF, (1 << 24) / ((tempValue) ? tempValue : 1));
    return retvalue;
}

// PWM gradient
void TMC2130_Set_PWM_Gradient(TMC2130TypeDef *handle, int32_t value)
{
    // Set gradient
    TMC2130_FIELD_UPDATE(handle, TMC2130_PWMCONF, TMC2130_PWM_GRAD_MASK, TMC2130_PWM_GRAD_SHIFT, value);

    // Enable/disable stealthChop accordingly
    TMC2130_FIELD_UPDATE(handle, TMC2130_GCONF, TMC2130_EN_PWM_MODE_MASK, TMC2130_EN_PWM_MODE_SHIFT, (value) ? 1 : 0);
}
int32_t TMC2130_Get_PWM_Gradient(TMC2130TypeDef *handle)
{
    return TMC2130_FIELD_READ(handle, TMC2130_PWMCONF, TMC2130_PWM_GRAD_MASK, TMC2130_PWM_GRAD_SHIFT);
}

// PWM amplitude
void TMC2130_Set_PWM_Amplitude(TMC2130TypeDef *handle, int32_t value)
{
    TMC2130_FIELD_UPDATE(handle, TMC2130_PWMCONF, TMC2130_PWM_AMPL_MASK, TMC2130_PWM_AMPL_SHIFT, value);
}
int32_t TMC2130_Get_PWM_Amplitude(TMC2130TypeDef *handle)
{
    return TMC2130_FIELD_READ(handle, TMC2130_PWMCONF, TMC2130_PWM_AMPL_MASK, TMC2130_PWM_AMPL_SHIFT);
}

// PWM frequency
void TMC2130_Set_PWM_Frequency(TMC2130TypeDef *handle, int32_t value)
{
    if (value >= 0 && value < 4)
    {
        TMC2130_FIELD_UPDATE(handle, TMC2130_PWMCONF, TMC2130_PWM_FREQ_MASK, TMC2130_PWM_FREQ_SHIFT, value);
    }
}
int32_t TMC2130_Get_PWM_Frequency(TMC2130TypeDef *handle)
{
    return TMC2130_FIELD_READ(handle, TMC2130_PWMCONF, TMC2130_PWM_FREQ_MASK, TMC2130_PWM_FREQ_SHIFT);
}

// PWM autoscale
void TMC2130_Set_PWM_Autoscale(TMC2130TypeDef *handle, int32_t value)
{
    TMC2130_FIELD_UPDATE(handle, TMC2130_PWMCONF,
                         TMC2130_PWM_AUTOSCALE_MASK, TMC2130_PWM_AUTOSCALE_SHIFT,
                         (value) ? 1 : 0);
}
int32_t TMC2130_Get_PWM_Autoscale(TMC2130TypeDef *handle)
{
    return TMC2130_FIELD_READ(handle, TMC2130_PWMCONF, TMC2130_PWM_AUTOSCALE_MASK, TMC2130_PWM_AUTOSCALE_SHIFT);
}

// Freewheeling mode
void TMC2130_Set_Freewheeling_Mode(TMC2130TypeDef *handle, int32_t value)
{
    TMC2130_FIELD_UPDATE(handle, TMC2130_PWMCONF, TMC2130_FREEWHEEL_MASK, TMC2130_FREEWHEEL_SHIFT, value);
}
int32_t TMC2130_Get_Freewheeling_Mode(TMC2130TypeDef *handle)
{
    return TMC2130_FIELD_READ(handle, TMC2130_PWMCONF, TMC2130_FREEWHEEL_MASK, TMC2130_FREEWHEEL_SHIFT);
}

// Load value
int32_t TMC2130_Get_Load_Value(TMC2130TypeDef *handle)
{
    return TMC2130_FIELD_READ(handle, TMC2130_DRV_STATUS, TMC2130_SG_RESULT_MASK, TMC2130_SG_RESULT_SHIFT);
}

// set diag0 attribute
void TMC2130_Set_DIAG0_Attribute(TMC2130TypeDef *handle, DIAG_Attribute_t attr, uint8_t enable)
{
    uint32_t mask = 0xFFFFFFFF;
    uint32_t shift = 0;
    switch (attr)
    {
    case DIAG_ERROR:
        mask = TMC2130_DIAG0_ERROR_ONLY_WITH_SD_MODE1_MASK;
        shift = TMC2130_DIAG0_ERROR_ONLY_WITH_SD_MODE1_SHIFT;
        break;
    case DIAG_OTPW:
        mask = TMC2130_DIAG0_OTPW_ONLY_WITH_SD_MODE1_MASK;
        shift = TMC2130_DIAG0_OTPW_ONLY_WITH_SD_MODE1_SHIFT;
        break;
    case DIAG_STALL:
        mask = TMC2130_DIAG0_STALL_MASK;
        shift = TMC2130_DIAG0_STALL_SHIFT;
        break;
    case DIAG_PUSHPULL:
        mask = TMC2130_DIAG0_INT_PUSHPULL_MASK;
        shift = TMC2130_DIAG0_INT_PUSHPULL_SHIFT;
        break;

    default:
        break;
    }

    TMC2130_FIELD_UPDATE(handle, TMC2130_GCONF, mask, shift, enable);
}

// set diag1 attribute
void TMC2130_Set_DIAG1_Attribute(TMC2130TypeDef *handle, DIAG_Attribute_t attr, uint8_t enable)
{
    uint32_t mask = 0xFFFFFFFF;
    uint32_t shift = 0;
    switch (attr)
    {
    case DIAG_STALL:
        mask = TMC2130_DIAG1_STALL_MASK;
        shift = TMC2130_DIAG1_STALL_SHIFT;
        break;
    case DIAG_INDEX:
        mask = TMC2130_DIAG1_INDEX_SHIFT;
        shift = TMC2130_DIAG1_ONSTATE_MASK;
        break;
    case DIAG_ON_STATE:
        mask = TMC2130_DIAG1_ONSTATE_MASK;
        shift = TMC2130_DIAG1_ONSTATE_SHIFT;
        break;
    case DIAG_STEPS_SKIPPED:
        mask = TMC2130_DIAG1_STEPS_SKIPPED_MASK;
        shift = TMC2130_DIAG1_STEPS_SKIPPED_SHIFT;
        break;
    case DIAG_PUSHPULL:
        mask = TMC2130_DIAG1_POSCOMP_PUSHPULL_MASK;
        shift = TMC2130_DIAG1_POSCOMP_PUSHPULL_SHIFT;
        break;

    default:
        break;
    }

    TMC2130_FIELD_UPDATE(handle, TMC2130_GCONF, mask, shift, enable);
}

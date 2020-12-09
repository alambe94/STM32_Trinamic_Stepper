#ifndef TMC2130_STEP_GENERATOR_H_
#define TMC2130_STEP_GENERATOR_H_

#include "API_Header.h"
#include "LinearRamp1.h"
#include "tmc2130.h"

#define STEPDIR_FREQUENCY (1 << 17)
#define STEPDIR_MAX_VELOCITY STEPDIR_FREQUENCY // Limit: 1 Step per interrupt (2^17 Hz) -> 2^17 pps
#define STEPDIR_MAX_ACCELERATION 2147418111    // Limit: Highest value above accumulator digits (0xFFFE0000).
// Any value above would lead to acceleration overflow whenever the accumulator digits overflow

#define STEPDIR_DEFAULT_ACCELERATION 100000
#define STEPDIR_DEFAULT_VELOCITY STEPDIR_MAX_VELOCITY

typedef enum
{
    STEPDIR_INTERNAL = 0,
    STEPDIR_EXTERNAL = 1
} StepDirMode; // Has to be set explicitly here because IDE relies on this number.

typedef enum
{
    STOP_NORMAL,
    STOP_EMERGENCY,
    STOP_STALL
} StepDirStop;

typedef enum
{
    SYNC_IDLE,               // Sync mechanism not running
    SYNC_SNAPSHOT_REQUESTED, // Main code saved the new acceleration and is waiting for the interrupt to save the velocity and apply the acceleration (atomically, from the StepDir generator perspective).
    SYNC_SNAPSHOT_SAVED,     // Interrupt saved the velocity
    SYNC_UPDATE_DATA         // Main code calculated an accelerationSteps difference which the interrupt needs to apply.
} StepDirSync;

// StepDir status bits
#define STATUS_EMERGENCY_STOP 0x01    // Halting condition - Emergency Off
#define STATUS_NO_STEP_PIN 0x02       // Halting condition - No pin set for Step output
#define STATUS_NO_DIR_PIN 0x04        // Halting condition - No pin set for Direction output
#define STATUS_STALLED 0x08           // Halting condition - Stall detected (while Stallguard is enabled)
#define STATUS_TARGET_REACHED 0x10    // Position mode status - target reached
#define STATUS_STALLGUARD_ACTIVE 0x20 // Stallguard status - Velocity threshold reached, Stallguard enabled
#define STATUS_MODE 0x40              // 0: Positioning mode, 1: Velocity mode

typedef struct
{ // Generic parameters
    uint8_t targetReached;
    uint8_t haltingCondition;
    // StallGuard
    bool stallGuardActive;
    int32_t minVelocityForStall;
    // Acceleration updating sync mechanism (see acceleration setter for details)
    StepDirSync syncFlag; // Synchronisation flag between main code & interrupt
    // Snapshot data
    // Interrupt -> main code
    int32_t oldVelocity;
    int32_t oldVelAccu;
    // Main code -> interrupt
    uint32_t newAcceleration;
    int32_t stepDifference;
    uint32_t frequency;

    TMC_LinearRamp ramp;
    StepDirMode mode;
} StepGeneraorTypedef;

typedef struct TMC2130_Controller_t
{
    TMC2130TypeDef *Motor;
    StepGeneraorTypedef Step_Generator;

} TMC2130_Controller_t;

void TMC_Add(TMC2130_Controller_t *handle);
void TMC_TIM_Enable(uint8_t enable);
void TMC_Enable_Driver(TMC2130_Controller_t *handle, uint8_t enable);
void TMC_Rotate(TMC2130_Controller_t *handle, int32_t velocity);
void TMC_Goto(TMC2130_Controller_t *handle, int32_t position);
void TMC_Move(TMC2130_Controller_t *handle, int32_t steps);
void TMC_Loop(TMC2130_Controller_t *handle);
void TMC_Stop(TMC2130_Controller_t *handle, StepDirStop stopType);
uint8_t TMC_Get_Status(TMC2130_Controller_t *handle);
uint8_t TMC_Target_Reached(TMC2130_Controller_t *handle);
void TMC_Enable_Stall(TMC2130_Controller_t *handle, int32_t minVelocity);
void TMC_Disable_Stall(TMC2130_Controller_t *handle);
void TMC_Set_Actual_Position(TMC2130_Controller_t *handle, int32_t actualPosition);
void TMC_Set_Acceleration(TMC2130_Controller_t *handle, uint32_t acceleration);
void TMC_Set_MAX_velocity(TMC2130_Controller_t *handle, int32_t velocityMax);
void TMC_Set_Frequency(TMC2130_Controller_t *handle, uint32_t frequency);
void TMC_Set_Mode(TMC2130_Controller_t *handle, StepDirMode mode);
void TMC_Set_Precision(TMC2130_Controller_t *handle, uint32_t precision);
int32_t TMC_Get_Actual_Position(TMC2130_Controller_t *handle);
int32_t TMC_Get_Target_Position(TMC2130_Controller_t *handle);
int32_t TMC_Get_Actual_Velocity(TMC2130_Controller_t *handle);
int32_t TMC_Get_Target_Velocity(TMC2130_Controller_t *handle);
uint32_t TMC_Get_Acceleration(TMC2130_Controller_t *handle);
int32_t TMC_Get_Max_Velocity(TMC2130_Controller_t *handle);
StepDirMode TMC_Get_Mode(TMC2130_Controller_t *handle);
uint32_t TMC_Get_Frequency(TMC2130_Controller_t *handle);
uint32_t TMC_Get_Precision(TMC2130_Controller_t *handle);
int32_t TMC_Get_MAX_Acceleration(TMC2130_Controller_t *handle);
int32_t TMC_Get_Measured_Speed(TMC2130_Controller_t *handle);

#endif /* TMC2130_STEP_GENERATOR_H_ */

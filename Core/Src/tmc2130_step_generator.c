/*
 * tmc2130_step_generator.c
 *
 * This is a basic implementation of a StepDir generator, capable of generating
 * velocity (reaching a target velocity with linear acceleration) and position
 * (reach a target position with linear acc-/decceleration and a maximum velocity)
 * ramps.
 *
 * ***** HOW IT WORKS *****
 *
 * General:
 *   A high frequency (2^17 Hz) Interrupt calculates acceleration, velocity and
 *   position. Position and velocity are calculated with 17 binary decimal places
 *   of precision.
 *
 * Velocity mode:
 *   In velocity mode, the generator will accelerate towards a target velocity.
 *   Acceleration and velocity can be changed at any point of the ramp. The position
 *   is tracked and resettable (useful for setting a reference point via a switch).
 *
 * Position mode:
 *   In position mode, a linearly accelerated ramp is used to reach the position.
 *   Parameters for the ramp are acceleration and maximum velocity. The generator
 *   will always increase the velocity towards the maximum velocity until the
 *   remaining distance to the target is required for the deceleration ramp.
 *   Acceleration, maximum velocity and target position can all be changed during
 *   the ramp. Note that decreasing acceleration or changing target position may
 *   lead to overshooting the target. In that case the generator will start a new
 *   ramp to the target, while always staying within the bounds of acceleration and
 *   velocity.
 *
 *   Due to imprecision in the deceleration distance calculations, a small tolerance
 *   window is used, where the motor will set the velocity to zero if the velocity is
 *   small enough and the position is reached (V_STOP). If the position is barely
 *   missed (HOMING_DISTANCE) and the velocity is zero, the generator will home in
 *   towards the target position at a low velocity (V_STOP).
 *   Changing the actual position value is not possible while in position mode
 *   the generator is not idle (target position reached, velocity zero).
 *   Changing the acceleration to zero is not possible in position mode.
 *   Acceleration value changes require a recalculation of the braking distance.
 *   This can result in more frequent near-misses of the target position, which the
 *   generator will compensate with starting new ramps or homing in (see above).
 *
 *   If overshooting the target by any step is not permitted, it is recommended to
 *   drive to the target without changing parameters during the ramp. Alternatively,
 *   driving to a point shortly before the actual target point and then starting
 *   another small ramp allows for parameter changes during the first ramp, only
 *   requiring a small distance drive with 'locked' parameters.
 *
 *   Overshooting from calculation errors is mostly limited to single digit
 *   position differences. Decreasing acceleration or moving the target position
 *   towards the actual position might result in bigger misses of the target.
 *
 * StallGuard:
 *   The StepDir generator supports the StallGuard feature, either by a input pin
 *   signal or with external monitoring. The function periodicJob() will check,
 *   whether the velocity is above the set StallGuard threshold velocity, set a
 *   status flag (usable for external StallGuard monitoring) and - if present -
 *   check the input pin for indicated stalls.
 *   Make sure that periodicJob() is called frequently to allow quick stall
 *   detection. The function can also be called by an interrupt to guarantee
 *   quick detection [1]. The interrupt should have a lower priority than the
 *   Step-Generator interrupt.
 *
 *   When using external monitoring, for example by checking a chip register,
 *   you can use the STATUS_STALLGUARD_ACTIVE bit of getStatus() to see if
 *   StallGuard is active. In case of a stall, calling stop(STOP_STALL) will
 *   trigger the stall mechanism, shutting down the generator without loosing
 *   further steps.
 *   Clearing a stall condition is done by setting the stall velocity threshold
 *   to any value.
 *   Position mode will start a new ramp towards the target after a stall.
 *
 * Emergency Stop:
 *   The stop function implements an emergency stop. This will result in the
 *   channel immediately stopping any movements. No parameters are updated to
 *   allow for diagnostics. The only way to clear the emergency stop event is
 *   to init() the StepDir generator again [2].
 *
 * ***** LIMITATIONS  *****
 *
 * The frequency of the StepDir generator is limited by the processor. On the
 * Landungsbrücke, the worst case of two motors/channels (TMC2041) is able to
 * still run at 2^17 Hz. Since the bulk of the calculation is per-motor/channel,
 * using a chip with only one motor/channel would allow a frequency of 2^18 Hz.
 * (Note that quite a few calculations have to divide by the frequency, so
 *  choosing a power of two simplifies those to right-shifts.)
 *
 * The limit on Step pulses is one generated pulse per interrupt.
 * The maximum velocity therefore is equal to the interrupt frequency:
 *   Max Velocity: 2^17 pps = 131072 pps
 *
 * Each tick the acceleration value gets added to the velocity accumulator
 * variable (uint32_t). The upper 15 digits are added to the velocity, the lower
 * 17 digits are kept in the accumulator between ticks. The maximum
 * acceleration will result in the upper 15 digits being 1 each tick, increasing
 * the velocity by 32767 (0x7FFF) per tick. The accumulator digits stay unchanged,
 * otherwise the overflow of the lower 17 accumulator digits into the upper 15
 * digits would cause the uint32_t to overflow, loosing an acceleration tick.
 * In other words: The upper 15 digits are 1, the lower 17 digits are 0:
 *   Max Acceleration: 0xFFFE0000 = 4294836224 pps^2
 *
 * A change from lowest to highest (or vice-versa) velocity would take 9 ticks at
 * maximum acceleration:
 *   ceil( (VMAX- (-VMAX)) / AMAX) = ceil(8,000244) = 9
 *
 * ***** Side notes ******
 * [1]: Technically periodicJob() is not interrupt-safe, since it updates the
 *      haltingCondition bitfield. Read-Modify-Write cycles of the main code
 *      could result in the changes of interrupts to the bitfield to be lost.
 *      In practice, the interrupt should just rewrite the stall bit on the next
 *      check though due to the nature of StallGuard.
 * [2]: Emergency stop does not have a graceful recovery method by design.
 *      Clearing the emergency stop via init() will result in all channels
 *      being reset.
 */

#include "stm32f4xx_hal.h"
#include "tmc2130_step_generator.h"
#include "TMC2130.h"

// Reset value for stallguard threshold. Since Stallguard is motor/application-specific we can't choose a good value here,
// so this value is rather randomly chosen. Leaving it at zero means stall detection turned off.
#define STALLGUARD_THRESHOLD 0

typedef struct TMC2130_Motor_t
    {
	TMC2130TypeDef Motor;
	ConfigurationTypeDef Motor_Config;
	StepGeneraorTypedef Step_Generator;
	TMC_LinearRamp Ramp_Calaculator;
	StepDirMode Mode;

	GPIO_TypeDef *Step_Port;
	uint16_t Step_Pin;

	GPIO_TypeDef *Dir_Port;
	uint16_t Dir_Pin;

    } TMC2130_Motor_t;

#define MAX_TMC2130 3

TMC2130_Motor_t *TMC2130_List[MAX_TMC2130];
uint8_t TMC2130_List_Count = 0;

void TMC2130_Add(TMC2130_Motor_t *handle)
    {
    if (TMC2130_List_Count < MAX_TMC2130)
	{
	TMC2130_List[TMC2130_List_Count++] = handle;
	}

    }

int32_t calculateStepDifference(int32_t velocity, uint32_t oldAccel,
	uint32_t newAccel);

void TMC2130_Step_TIM_ISR()
    {

    TMC2130_Motor_t *handle = NULL;

    for (uint8_t i = 0; i < TMC2130_List_Count; i++)
	{

	handle = TMC2130_List[TMC2130_List_Count];

	// If any halting condition is present, abort immediately
	if (handle->Step_Generator.haltingCondition)
	    ;

	// Reset step output (falling edge of last pulse)

	// Check StallGuard pin if one is registered

	// Compute ramp
	int32_t dx = tmc_ramp_linear_compute(&handle->Ramp_Calaculator);

	// Step
	if (dx == 0) // No change in position -> skip step generation
	    goto skipStep;

	// Direction
	//*((dx > 0) ? 1:0;

	// Set step output (rising edge of step pulse)

	skipStep:
	// Synchronised Acceleration update
	switch (handle->Step_Generator.syncFlag)
	    {
	case SYNC_SNAPSHOT_REQUESTED:
	    // Apply the new acceleration
	    tmc_ramp_linear_set_acceleration(&handle->Ramp_Calaculator,
		    handle->Step_Generator.newAcceleration);
	    // Save a snapshot of the velocity
	    handle->Step_Generator.oldVelocity =
		    tmc_ramp_linear_get_rampVelocity(&handle->Ramp_Calaculator);

	    handle->Step_Generator.syncFlag = SYNC_SNAPSHOT_SAVED;
	    break;
	case SYNC_UPDATE_DATA:
	    handle->Ramp_Calaculator.accelerationSteps +=
		    handle->Step_Generator.stepDifference;
	    handle->Step_Generator.syncFlag = SYNC_IDLE;
	    break;
	default:
	    break;
	    }
	}
    }

void TMC2130_Rotate(TMC2130_Motor_t *handle, int32_t velocity)
    {

    // Set the rampmode first - other way around might cause issues
    tmc_ramp_linear_set_mode(&handle->Ramp_Calaculator,
	    TMC_RAMP_LINEAR_MODE_VELOCITY);
    switch (handle->Mode)
	{
    case STEPDIR_INTERNAL:
	tmc_ramp_linear_set_targetVelocity(&handle->Ramp_Calaculator,
		MIN(STEPDIR_MAX_VELOCITY, velocity));
	break;
    case STEPDIR_EXTERNAL:
    default:
	tmc_ramp_linear_set_targetVelocity(&handle->Ramp_Calaculator, velocity);
	break;
	}
    }

void TMC2130_Goto(TMC2130_Motor_t *handle, int position)
    {
    tmc_ramp_linear_set_mode(&handle->Ramp_Calaculator,
	    TMC_RAMP_LINEAR_MODE_POSITION);
    tmc_ramp_linear_set_targetPosition(&handle->Ramp_Calaculator, position);
    }

void TMC2130_Move(TMC2130_Motor_t *handle, uint8_t motor, int32_t ticks)
    {
    // determine actual position and add numbers of ticks to move
    ticks += TMC2130_Get_Actual_Position();
    return TMC2130_Goto(motor, ticks);
    }

void TMC2130_Loop(TMC2130_Motor_t *handle, int position)
    {

    // Check stallguard velocity threshold
    if ((handle->Step_Generator.stallGuardThreshold != 0)
	    && (abs(tmc_ramp_linear_get_rampVelocity(&handle->Ramp_Calaculator))
		    >= handle->Step_Generator.stallGuardThreshold))
	{
	handle->Step_Generator.stallGuardActive = true;
	}
    else
	{
	handle->Step_Generator.stallGuardActive = false;
	}

    }

void TMC2130_Stop(TMC2130_Motor_t *handle, StepDirStop stopType)
    {
    switch (stopType)
	{
    case STOP_NORMAL:
	tmc_ramp_linear_set_targetVelocity(&handle->Ramp_Calaculator, 0);
	tmc_ramp_linear_set_mode(&handle->Ramp_Calaculator,
		TMC_RAMP_LINEAR_MODE_VELOCITY);
	break;
    case STOP_EMERGENCY:
	handle->Step_Generator.haltingCondition |= STATUS_EMERGENCY_STOP;
	break;
    case STOP_STALL:
	handle->Step_Generator.haltingCondition |= STATUS_STALLED;
	tmc_ramp_linear_set_rampVelocity(&handle->Ramp_Calaculator, 0);
	handle->Step_Generator.ramp.accumulatorVelocity = 0;
	tmc_ramp_linear_set_targetVelocity(&handle->Ramp_Calaculator, 0);
	handle->Step_Generator.ramp.accelerationSteps = 0;
	break;
	}
    }

uint8_t TMC2130_Get_Status(TMC2130_Motor_t *handle)
    {

    uint8_t status = handle->Step_Generator.haltingCondition;

    status |=
	    (handle->Step_Generator.targetReached) ? STATUS_TARGET_REACHED : 0;
    status |=
	    (handle->Step_Generator.stallGuardActive) ?
		    STATUS_STALLGUARD_ACTIVE : 0;
    status |=
	    (tmc_ramp_linear_get_mode(&handle->Ramp_Calaculator)
		    == TMC_RAMP_LINEAR_MODE_VELOCITY) ? STATUS_MODE : 0;

    return status;
    }

void TMC2130_stallGuard(TMC2130_Motor_t *handle, bool stall)
    {
    if (handle->Step_Generator.stallGuardActive && stall)
	{
	TMC2130_Stop(handle, STOP_STALL);
	}
    }

// ===== Setters =====
// The setters are responsible to access their respective variables while keeping the ramp generation stable

// Set actual and target position (Not during an active position ramp)
void TMC2130_Set_Actual_Position(TMC2130_Motor_t *handle, int actualPosition)
    {

    if (tmc_ramp_linear_get_mode(&handle->Ramp_Calaculator)
	    == TMC_RAMP_LINEAR_MODE_POSITION)
	{
	// In position mode: If we're not idle -> abort
//		if ((handle->Step_Generator.actualVelocity != 0) ||
//		   (handle->Step_Generator.actualPosition != handle->Step_Generator.targetPosition))
//		{
//			return;
//		}

	// todo CHECK 2: Use a haltingCondition to prevent movement instead of VMAX? (LH)
	// Temporarity set VMAX to 0 to prevent movement between setting actualPosition and targetPosition
//		uint32_t tmp = handle->Step_Generator.velocityMax;
//		handle->Step_Generator.velocityMax = 0;

	// Also update target position to prevent movement
	tmc_ramp_linear_set_targetPosition(&handle->Ramp_Calaculator,
		actualPosition);
	tmc_ramp_linear_set_rampPosition(&handle->Ramp_Calaculator,
		actualPosition);

	// Restore VMAX
//		handle->Step_Generator.velocityMax = tmp;
	}
    else
	{
	// In velocity mode the position is not relevant so we can just update it without precautions
	tmc_ramp_linear_set_rampPosition(&handle->Ramp_Calaculator,
		actualPosition);
	}
    }

void TMC2130_Set_Acceleration(TMC2130_Motor_t *handle, uint32_t acceleration)
    {

    if (tmc_ramp_linear_get_mode(&handle->Ramp_Calaculator)
	    == TMC_RAMP_LINEAR_MODE_VELOCITY)
	{	// Velocity mode does not require any special actions
	tmc_ramp_linear_set_acceleration(&handle->Ramp_Calaculator,
		acceleration);
	return;
	}

    // Position mode does not allow acceleration 0
    if (acceleration == 0)
	return;

    tmc_ramp_linear_set_acceleration(&handle->Ramp_Calaculator, acceleration);

    // Store the old acceleration
    uint32_t oldAcceleration = tmc_ramp_linear_get_acceleration(
	    &handle->Ramp_Calaculator);

    // If the channel is not halted we need to synchronise with the interrupt
    if (handle->Step_Generator.haltingCondition == 0)
	{
	// Sync mechanism: store the new acceleration value and request
	// a snapshot from the interrupt
	handle->Step_Generator.newAcceleration = acceleration;
	handle->Step_Generator.syncFlag = SYNC_SNAPSHOT_REQUESTED;
	// Wait for the flag update from the interrupt.
	while (ACCESS_ONCE(handle->Step_Generator.syncFlag)
		!= SYNC_SNAPSHOT_SAVED)
	    ; // todo CHECK 2: Timeout to prevent deadlock? (LH) #1
	}
    else
	{ // Channel is halted -> access data directly without sync mechanism
	  //handle->Step_Generator.acceleration = acceleration;
	tmc_ramp_linear_set_acceleration(&handle->Ramp_Calaculator,
		acceleration);
	handle->Step_Generator.oldVelocity = tmc_ramp_linear_get_rampVelocity(
		&handle->Ramp_Calaculator);
	}

    int32_t stepDifference = calculateStepDifference(
	    handle->Step_Generator.oldVelocity, oldAcceleration, acceleration);

    if (handle->Step_Generator.haltingCondition == 0)
	{
	handle->Step_Generator.stepDifference = stepDifference;
	handle->Step_Generator.syncFlag = SYNC_UPDATE_DATA;

	// Wait for interrupt to set flag to SYNC_IDLE
	while (ACCESS_ONCE(handle->Step_Generator.syncFlag) != SYNC_IDLE)
	    ; // todo CHECK 2: Timeout to prevent deadlock? (LH) #2
	}
    else
	{ // Channel is halted -> access data directly without sync mechanism
	handle->Step_Generator.ramp.accelerationSteps += stepDifference;
	}
    }

void TMC2130_Set_MAX_velocity(TMC2130_Motor_t *handle, int velocityMax)
    {
    tmc_ramp_linear_set_maxVelocity(&handle->Ramp_Calaculator, velocityMax);
    }

// Set the velocity threshold for active StallGuard. Also reset the stall flag
void TMC2130_Set_Stallth(TMC2130_Motor_t *handle, int stallGuardThreshold)
    {
    handle->Step_Generator.stallGuardThreshold = stallGuardThreshold;
    handle->Step_Generator.haltingCondition &= ~STATUS_STALLED;
    }

void TMC2130_Set_Frequency(TMC2130_Motor_t *handle, uint32_t frequency)
    {
    handle->Step_Generator.frequency = frequency;
    }

void TMC2130_Set_Mode(TMC2130_Motor_t *handle, StepDirMode mode)
    {
    handle->Mode = mode;

    if (mode == STEPDIR_INTERNAL)
	{
	TMC2130_Set_Frequency(handle, STEPDIR_FREQUENCY);
	}
    }

void TMC2130_Set_Precision(TMC2130_Motor_t *handle, uint32_t precision)
    {
    tmc_ramp_linear_set_precision(&handle->Ramp_Calaculator, precision);
    }

// ===== Getters =====
int TMC2130_Get_Actual_Position(TMC2130_Motor_t *handle)
    {
    return tmc_ramp_linear_get_rampPosition(&handle->Ramp_Calaculator);
    }

int TMC2130_Get_Target_Position(TMC2130_Motor_t *handle)
    {
    return tmc_ramp_linear_get_targetPosition(&handle->Ramp_Calaculator);
    }

int TMC2130_Get_Actual_Velocity(TMC2130_Motor_t *handle)
    {
    return tmc_ramp_linear_get_rampVelocity(&handle->Ramp_Calaculator);
    }

int TMC2130_Get_Target_Velocity(TMC2130_Motor_t *handle)
    {
    return tmc_ramp_linear_get_targetVelocity(&handle->Ramp_Calaculator);
    }

uint32_t TMC2130_Get_Acceleration(TMC2130_Motor_t *handle)
    {
    return tmc_ramp_linear_get_acceleration(&handle->Ramp_Calaculator);
    }

int TMC2130_Get_Max_Velocity(TMC2130_Motor_t *handle)
    {
    return tmc_ramp_linear_get_maxVelocity(&handle->Ramp_Calaculator);
    }

int TMC2130_Get_Stallth(TMC2130_Motor_t *handle)
    {
    return handle->Step_Generator.stallGuardThreshold;
    }

StepDirMode TMC2130_Get_Mode(TMC2130_Motor_t *handle)
    {
    return handle->Mode;
    }

uint32_t TMC2130_Get_Frequency(TMC2130_Motor_t *handle)
    {
    return handle->Step_Generator.frequency;
    }

uint32_t TMC2130_Get_Precision(TMC2130_Motor_t *handle)
    {
    return tmc_ramp_linear_get_precision(&handle->Ramp_Calaculator);
    }

int32_t TMC2130_Get_MAX_Acceleration(TMC2130_Motor_t *handle)
    {
    if (handle->Mode == STEPDIR_INTERNAL)
	return STEPDIR_MAX_ACCELERATION;

    // STEPDIR_EXTERNAL -> no limitation from this generator
    return s32_MAX ;
    }

// ===================

void TMC2130_Init(TMC2130_Motor_t *handle, uint32_t precision)
    {
    if (precision == 0)
	{
	// Use default precision
	precision = STEPDIR_FREQUENCY;
	}

    handle->Step_Generator.oldVelAccu = 0;
    handle->Step_Generator.oldVelocity = 0;
    handle->Step_Generator.newAcceleration = 0;

    // Set the no-pin halting conditions before changing the pins
    // to avoid a race condition with the interrupt
    handle->Step_Generator.haltingCondition = STATUS_NO_STEP_PIN
	    | STATUS_NO_DIR_PIN;
    handle->Step_Generator.stallGuardPin = 0x00;
    handle->Step_Generator.stepPin = 0x00;
    handle->Step_Generator.dirPin = 0x00;

    handle->Step_Generator.stallGuardThreshold = STALLGUARD_THRESHOLD;

    handle->Step_Generator.mode = STEPDIR_INTERNAL;
    handle->Step_Generator.frequency = precision;

    tmc_ramp_linear_init(&handle->Step_Generator.ramp);
    tmc_ramp_linear_set_precision(&handle->Step_Generator.ramp, precision);
    tmc_ramp_linear_set_maxVelocity(&handle->Step_Generator.ramp,
    STEPDIR_DEFAULT_VELOCITY);
    tmc_ramp_linear_set_acceleration(&handle->Step_Generator.ramp,
    STEPDIR_DEFAULT_ACCELERATION);

    // Chip-specific hardware peripheral initialisation
    // configure tim

    }

// ===== Helper function =====
/* The required calculation to do is the difference of the required
 * amount of steps to reach a given velocity, using two different
 * given accelerations with an evenly accelerated ramp.
 *
 * Calculation:
 *   v1: Start velocity
 *   v2: Target velocity
 *   a:  Acceleration
 *   t:  Time required to reach the target velocity
 *   s:  distance traveled while accelerating
 *
 *   t = (v2 - v1) / a
 * The distance can be calculated with the average velocity v_avrg:
 *   v_avrg = (v2 + v1) / 2
 *   s = v_avrg * t
 *     = (v2 + v1) / 2 * (v2 - v1) / a
 *     = (v2 + v1) * (v2 - v1) / (2*a)
 *     = (v2^2 - v1^2) / (2*a)
 *
 * Our calculation assumes that the starting velocity v1 is zero:
 *   v1 := 0
 *   s = (v2^2 - 0^2) / (2*a)
 *     = v2^2 / (2*a)
 *
 * Calculating velocities with an accumulator results in the velocity
 * being equal or up to 1 below the theoretical velocity (distributed evenly).
 * To reduce the maximum error in the result of the step calculation,
 * the velocity will be increased by 0.5, so that the velocity error
 * will be within [0.5, -0.5).
 *   s = (v+0.5)^2 / (2*a)
 * Change to using integer math:
 *   s = ((v+0.5)*2/2)^2 / (2*a)
 *     = ((v*2 + 1)/2)^2 / (2*a)
 *     = (v*2 + 1)^2 / 2^2 / (2*a)
 *     = (v*2 + 1)^2 / (8a)
 *
 * The result we need is the difference s2 - s1, using a2 and a1 respectively.
 * Only changing the acceleration allows us to reuse most of the calculation:
 * We define
 *   x := (v*2 + 1)^2 / 8
 * so that
 *   s = x/a
 *
 * Variables <=> Formula:
 *   oldAccel: a2
 *   newAccel: a1
 *   velocity: v
 *   oldSteps: s1
 *   newSteps: s2
 */
int32_t calculateStepDifference(int32_t velocity, uint32_t oldAccel,
	uint32_t newAccel)
    {
    int64_t tmp = velocity;
    tmp = tmp * 2 + 1;
    tmp = (tmp * tmp) / 4;
    tmp = tmp / 2;
    uint32_t oldSteps = tmp / oldAccel;
    uint32_t newSteps = tmp / newAccel;

    return newSteps - oldSteps;
    }

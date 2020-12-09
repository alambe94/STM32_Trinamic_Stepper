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
#include "tmc2130.h"
#include "TMC2130_Register.h"
#include "TMC2130_Mask_Shift.h"
#include "tmc2130_io.h"

#define MAX_TMC2130 3
TMC2130_Controller_t *TMC2130_List[MAX_TMC2130];
uint8_t TMC2130_List_Count = 0;

// configure tim
// configure in cube to generate 131Khz interrupt. see tim.c
// gpio configured in cube see gpio.c
// 84000000/2^17 tim arr or tim clk/STEPDIR_FREQUENCY
// add all the instances of motors before enabling tim isr, enable tim manually
void TMC_Add(TMC2130_Controller_t *handle)
{
	if (TMC2130_List_Count < MAX_TMC2130)
	{
		handle->Step_Generator.oldVelAccu = 0;
		handle->Step_Generator.oldVelocity = 0;
		handle->Step_Generator.newAcceleration = 0;
		handle->Step_Generator.mode = STEPDIR_INTERNAL;
		handle->Step_Generator.frequency = STEPDIR_FREQUENCY;
		handle->Step_Generator.haltingCondition = 0x00;

		tmc_ramp_linear_init(&handle->Step_Generator.ramp);
		tmc_ramp_linear_set_precision(&handle->Step_Generator.ramp, STEPDIR_FREQUENCY);
		tmc_ramp_linear_set_maxVelocity(&handle->Step_Generator.ramp, STEPDIR_MAX_VELOCITY);
		tmc_ramp_linear_set_acceleration(&handle->Step_Generator.ramp, STEPDIR_MAX_ACCELERATION);

		TMC2130_IO_Init(handle->Motor);

		TMC2130_List[TMC2130_List_Count++] = handle;
	}
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

extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim10;
void TMC_TIM_Enable(uint8_t enable)
{
	if (enable)
	{
		HAL_TIM_Base_Start_IT(&htim11);
		HAL_TIM_Base_Start_IT(&htim10);
	}
	else
	{
		HAL_TIM_Base_Stop_IT(&htim11);
		HAL_TIM_Base_Stop_IT(&htim10);
	}
}

// tim isr 131Khz
// called from HAL_TIM_PeriodElapsedCallback in stm32f4xx_it.c
void TMC_Main_ISR()
{

	TMC2130_Controller_t *handle = NULL;

	uint8_t active_motors = 0;

	for (uint8_t i = 0; i < TMC2130_List_Count; i++)
	{

		handle = TMC2130_List[i];

		// If any halting condition is present, abort immediately
		if (handle->Step_Generator.haltingCondition)
			continue;

		// at least one motor is active
		active_motors++;

		// Reset step output (falling edge of last pulse)
		HAL_GPIO_WritePin(handle->Motor->Step_Port, handle->Motor->Step_Pin,
						  GPIO_PIN_RESET);

		// Compute ramp
		int32_t dx = tmc_ramp_linear_compute(&handle->Step_Generator.ramp);

		// Step
		if (dx == 0) // No change in position -> skip step generation
			goto skipStep;

		// Direction
		if (dx > 0)
		{
			HAL_GPIO_WritePin(handle->Motor->DIR_Port, handle->Motor->DIR_Pin, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(handle->Motor->DIR_Port, handle->Motor->DIR_Pin, GPIO_PIN_SET);
		}

		// Set step output (rising edge of step pulse)
		HAL_GPIO_WritePin(handle->Motor->Step_Port, handle->Motor->Step_Pin, GPIO_PIN_SET);

	skipStep:
		// Synchronised Acceleration update
		switch (handle->Step_Generator.syncFlag)
		{
		case SYNC_SNAPSHOT_REQUESTED:
			// Apply the new acceleration
			tmc_ramp_linear_set_acceleration(&handle->Step_Generator.ramp, handle->Step_Generator.newAcceleration);
			// Save a snapshot of the velocity
			handle->Step_Generator.oldVelocity = tmc_ramp_linear_get_rampVelocity(&handle->Step_Generator.ramp);
			handle->Step_Generator.syncFlag = SYNC_SNAPSHOT_SAVED;
			break;
		case SYNC_UPDATE_DATA:
			handle->Step_Generator.ramp.accelerationSteps +=
				handle->Step_Generator.stepDifference;
			handle->Step_Generator.syncFlag = SYNC_IDLE;
			break;
		default:
			break;
		}
	}

	// zero motor active disable isr
	if (!active_motors)
	{
		//HAL_TIM_Base_Stop_IT(&htim11);
	}
}

// tim isr 100hz
// called from HAL_TIM_PeriodElapsedCallback in stm32f4xx_it.c
void TMC_LOOP_ISR()
{
	TMC2130_Controller_t *handle = NULL;

	for (uint8_t i = 0; i < TMC2130_List_Count; i++)
	{
		handle = TMC2130_List[i];

		TMC_Loop(handle);
	}
}

void TMC_Enable_Driver(TMC2130_Controller_t *handle, uint8_t enable)
{
	if (enable)
	{
		HAL_GPIO_WritePin(handle->Motor->Enable_Port, handle->Motor->Enable_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(handle->Motor->Enable_Port, handle->Motor->Enable_Pin, GPIO_PIN_SET);
	}
}

void TMC_Rotate(TMC2130_Controller_t *handle, int32_t velocity)
{
	handle->Step_Generator.haltingCondition = 0;

	// Set the rampmode first - other way around might cause issues
	tmc_ramp_linear_set_mode(&handle->Step_Generator.ramp, TMC_RAMP_LINEAR_MODE_VELOCITY);
	switch (handle->Step_Generator.mode)
	{
	case STEPDIR_INTERNAL:
		tmc_ramp_linear_set_targetVelocity(&handle->Step_Generator.ramp,
										   MIN(STEPDIR_MAX_VELOCITY, velocity));
		break;

	case STEPDIR_EXTERNAL:
	default:
		tmc_ramp_linear_set_targetVelocity(&handle->Step_Generator.ramp,
										   velocity);
		break;
	}
}

void TMC_Goto(TMC2130_Controller_t *handle, int32_t position)
{
	handle->Step_Generator.haltingCondition = 0;

	tmc_ramp_linear_set_mode(&handle->Step_Generator.ramp, TMC_RAMP_LINEAR_MODE_POSITION);
	tmc_ramp_linear_set_targetPosition(&handle->Step_Generator.ramp, position);
}

void TMC_Move(TMC2130_Controller_t *handle, int32_t steps)
{
	// determine actual position and add numbers of ticks to move
	steps += TMC_Get_Actual_Position(handle);
	TMC_Goto(handle, steps);
}

void TMC_Loop(TMC2130_Controller_t *handle)
{
	// Check stallGuard from spi
	if (TMC2130_Read_Register(handle->Motor, TMC2130_DRV_STATUS) & TMC2130_STALLGUARD_MASK)
	{
		if (handle->Step_Generator.stallGuardActive &&
			(abs(tmc_ramp_linear_get_rampVelocity(&handle->Step_Generator.ramp)) >= handle->Step_Generator.minVelocityForStall))
		{
			TMC_Stop(handle, STOP_STALL);
		}
	}

	// Check stallGuard from diag pin
	/*
    if (HAL_GPIO_ReadPin(handle->Motor->DIAG0_Port, handle->Motor->DIAG0_Pin) == GPIO_PIN_RESET)
	{
	if (handle->Step_Generator.stallGuardActive
		&& (abs(
			tmc_ramp_linear_get_rampVelocity(
				&handle->Step_Generator.ramp))
			>= handle->Step_Generator.minVelocityForStall))
	    {
	    TMC_Stop(handle, STOP_STALL);
	    }
	}
*/
}

void TMC_Stop(TMC2130_Controller_t *handle, StepDirStop stopType)
{
	switch (stopType)
	{
	case STOP_NORMAL:
		tmc_ramp_linear_set_targetVelocity(&handle->Step_Generator.ramp, 0);
		tmc_ramp_linear_set_mode(&handle->Step_Generator.ramp, TMC_RAMP_LINEAR_MODE_VELOCITY);
		break;

	case STOP_EMERGENCY:
		handle->Step_Generator.haltingCondition |= STATUS_EMERGENCY_STOP;
		break;

	case STOP_STALL:
		handle->Step_Generator.haltingCondition |= STATUS_STALLED;
		tmc_ramp_linear_set_rampVelocity(&handle->Step_Generator.ramp, 0);
		handle->Step_Generator.ramp.accumulatorVelocity = 0;
		tmc_ramp_linear_set_targetVelocity(&handle->Step_Generator.ramp, 0);
		handle->Step_Generator.ramp.accelerationSteps = 0;
		break;
	}
}

uint8_t TMC_Get_Status(TMC2130_Controller_t *handle)
{
	uint8_t status = handle->Step_Generator.haltingCondition;

	handle->Step_Generator.targetReached = 0;

	if (TMC_Target_Reached(handle))
	{
		handle->Step_Generator.targetReached = 1;
	}

	status |= (handle->Step_Generator.targetReached) ? STATUS_TARGET_REACHED : 0;
	status |= (handle->Step_Generator.stallGuardActive) ? STATUS_STALLGUARD_ACTIVE : 0;
	status |= (tmc_ramp_linear_get_mode(&handle->Step_Generator.ramp) == TMC_RAMP_LINEAR_MODE_VELOCITY) ? STATUS_MODE : 0;

	return status;
}

uint8_t TMC_Target_Reached(TMC2130_Controller_t *handle)
{
	uint8_t xreturn = 0;

	if (tmc_ramp_linear_get_mode(&handle->Step_Generator.ramp) == TMC_RAMP_LINEAR_MODE_POSITION)
	{
		if (tmc_ramp_linear_get_rampPosition(&handle->Step_Generator.ramp) == tmc_ramp_linear_get_targetPosition(&handle->Step_Generator.ramp))
		{
			xreturn = 1;
		}
	}
	else
	{
		if (tmc_ramp_linear_get_rampVelocity(&handle->Step_Generator.ramp) == tmc_ramp_linear_get_targetVelocity(&handle->Step_Generator.ramp))
		{
			xreturn = 1;
		}
	}

	return xreturn;
}

void TMC_Enable_Stall(TMC2130_Controller_t *handle, int32_t minVelocity)
{
	handle->Step_Generator.stallGuardActive = 1;
	handle->Step_Generator.minVelocityForStall = minVelocity;
}

void TMC_Disable_Stall(TMC2130_Controller_t *handle)
{
	handle->Step_Generator.stallGuardActive = 0;
}

// ===== Setters =====
// The setters are responsible to access their respective variables while keeping the ramp generation stable

// Set actual and target position (Not during an active position ramp)
void TMC_Set_Actual_Position(TMC2130_Controller_t *handle, int32_t actualPosition)
{
	if (tmc_ramp_linear_get_mode(&handle->Step_Generator.ramp) == TMC_RAMP_LINEAR_MODE_POSITION)
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
		tmc_ramp_linear_set_targetPosition(&handle->Step_Generator.ramp, actualPosition);
		tmc_ramp_linear_set_rampPosition(&handle->Step_Generator.ramp, actualPosition);

		// Restore VMAX
		//		handle->Step_Generator.velocityMax = tmp;
	}
	else
	{
		// In velocity mode the position is not relevant so we can just update it without precautions
		tmc_ramp_linear_set_rampPosition(&handle->Step_Generator.ramp, actualPosition);
	}
}

void TMC_Set_Acceleration(TMC2130_Controller_t *handle, uint32_t acceleration)
{
	if (tmc_ramp_linear_get_mode(&handle->Step_Generator.ramp) == TMC_RAMP_LINEAR_MODE_VELOCITY)
	{ // Velocity mode does not require any special actions
		tmc_ramp_linear_set_acceleration(&handle->Step_Generator.ramp, acceleration);
		return;
	}

	// Position mode does not allow acceleration 0
	if (acceleration == 0)
		return;

	tmc_ramp_linear_set_acceleration(&handle->Step_Generator.ramp, acceleration);

	// Store the old acceleration
	uint32_t oldAcceleration = tmc_ramp_linear_get_acceleration(&handle->Step_Generator.ramp);

	// If the channel is not halted we need to synchronise with the interrupt
	if (handle->Step_Generator.haltingCondition == 0)
	{
		// Sync mechanism: store the new acceleration value and request
		// a snapshot from the interrupt
		handle->Step_Generator.newAcceleration = acceleration;
		handle->Step_Generator.syncFlag = SYNC_SNAPSHOT_REQUESTED;
		// Wait for the flag update from the interrupt.
		while (ACCESS_ONCE(handle->Step_Generator.syncFlag) != SYNC_SNAPSHOT_SAVED)
			; // todo CHECK 2: Timeout to prevent deadlock? (LH) #1
	}
	else
	{ // Channel is halted -> access data directly without sync mechanism
		//handle->Step_Generator.acceleration = acceleration;
		tmc_ramp_linear_set_acceleration(&handle->Step_Generator.ramp, acceleration);
		handle->Step_Generator.oldVelocity = tmc_ramp_linear_get_rampVelocity(&handle->Step_Generator.ramp);
	}

	int32_t stepDifference = calculateStepDifference(handle->Step_Generator.oldVelocity, oldAcceleration, acceleration);

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

void TMC_Set_MAX_velocity(TMC2130_Controller_t *handle, int32_t velocityMax)
{
	tmc_ramp_linear_set_maxVelocity(&handle->Step_Generator.ramp, velocityMax);
}

void TMC_Set_Frequency(TMC2130_Controller_t *handle, uint32_t frequency)
{
	handle->Step_Generator.frequency = frequency;
}

void TMC_Set_Mode(TMC2130_Controller_t *handle, StepDirMode mode)
{
	handle->Step_Generator.mode = mode;

	if (mode == STEPDIR_INTERNAL)
	{
		TMC_Set_Frequency(handle, STEPDIR_FREQUENCY);
	}
}

void TMC_Set_Precision(TMC2130_Controller_t *handle, uint32_t precision)
{
	tmc_ramp_linear_set_precision(&handle->Step_Generator.ramp, precision);
}

// ===== Getters =====
int32_t TMC_Get_Actual_Position(TMC2130_Controller_t *handle)
{
	return tmc_ramp_linear_get_rampPosition(&handle->Step_Generator.ramp);
}

int32_t TMC_Get_Target_Position(TMC2130_Controller_t *handle)
{
	return tmc_ramp_linear_get_targetPosition(&handle->Step_Generator.ramp);
}

int32_t TMC_Get_Actual_Velocity(TMC2130_Controller_t *handle)
{
	return tmc_ramp_linear_get_rampVelocity(&handle->Step_Generator.ramp);
}

int32_t TMC_Get_Target_Velocity(TMC2130_Controller_t *handle)
{
	return tmc_ramp_linear_get_targetVelocity(&handle->Step_Generator.ramp);
}

uint32_t TMC_Get_Acceleration(TMC2130_Controller_t *handle)
{
	return tmc_ramp_linear_get_acceleration(&handle->Step_Generator.ramp);
}

int32_t TMC_Get_Max_Velocity(TMC2130_Controller_t *handle)
{
	return tmc_ramp_linear_get_maxVelocity(&handle->Step_Generator.ramp);
}

StepDirMode TMC_Get_Mode(TMC2130_Controller_t *handle)
{
	return handle->Step_Generator.mode;
}

uint32_t TMC_Get_Frequency(TMC2130_Controller_t *handle)
{
	return handle->Step_Generator.frequency;
}

uint32_t TMC_Get_Precision(TMC2130_Controller_t *handle)
{
	return tmc_ramp_linear_get_precision(&handle->Step_Generator.ramp);
}

int32_t TMC_Get_MAX_Acceleration(TMC2130_Controller_t *handle)
{
	if (handle->Step_Generator.mode == STEPDIR_INTERNAL)
		return STEPDIR_MAX_ACCELERATION;

	// STEPDIR_EXTERNAL -> no limitation from this generator
	return s32_MAX;
}

// Measured Speed
int32_t TMC_Get_Measured_Speed(TMC2130_Controller_t *handle)
{
	int32_t tempValue = (int32_t)(((int64_t)TMC_Get_Frequency(handle) * (int64_t)122) / (int64_t)TMC2130_FIELD_READ(handle->Motor, TMC2130_TSTEP,
																													TMC2130_TSTEP_MASK, TMC2130_TSTEP_SHIFT));
	return (abs(tempValue) < 20) ? 0 : tempValue;
}

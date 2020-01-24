/*
 * tmc2130_step_generator.h
 *
 *  Created on: Jan 24, 2020
 *      Author: medprime
 */

#ifndef TMC2130_STEP_GENERATOR_H_
#define TMC2130_STEP_GENERATOR_H_

#include "tmc/helpers/API_Header.h"
#include "tmc/ramp/LinearRamp1.h"

#define STEPDIR_FREQUENCY         (1 << 17)
#define STEPDIR_MAX_VELOCITY      STEPDIR_FREQUENCY // Limit: 1 Step per interrupt (2^17 Hz) -> 2^17 pps
#define STEPDIR_MAX_ACCELERATION  2147418111        // Limit: Highest value above accumulator digits (0xFFFE0000).
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
    SYNC_IDLE,                // Sync mechanism not running
    SYNC_SNAPSHOT_REQUESTED, // Main code saved the new acceleration and is waiting for the interrupt to save the velocity and apply the acceleration (atomically, from the StepDir generator perspective).
    SYNC_SNAPSHOT_SAVED,      // Interrupt saved the velocity
    SYNC_UPDATE_DATA // Main code calculated an accelerationSteps difference which the interrupt needs to apply.
    } StepDirSync;

// StepDir status bits
#define STATUS_EMERGENCY_STOP     0x01  // Halting condition - Emergency Off
#define STATUS_NO_STEP_PIN        0x02  // Halting condition - No pin set for Step output
#define STATUS_NO_DIR_PIN         0x04  // Halting condition - No pin set for Direction output
#define STATUS_STALLED            0x08  // Halting condition - Stall detected (while Stallguard is enabled)
#define STATUS_TARGET_REACHED     0x10  // Position mode status - target reached
#define STATUS_STALLGUARD_ACTIVE  0x20  // Stallguard status - Velocity threshold reached, Stallguard enabled
#define STATUS_MODE               0x40  // 0: Positioning mode, 1: Velocity mode

typedef struct
    {	// Generic parameters
	uint8_t targetReached;
	uint8_t haltingCondition;
	// StallGuard
	bool stallGuardActive;
	int stallGuardThreshold;
	uint16_t stallGuardPin;
	// StepDir Pins
	uint16_t stepPin;
	uint16_t dirPin;
	// Acceleration updating sync mechanism (see acceleration setter for details)
	StepDirSync syncFlag; // Synchronisation flag between main code & interrupt
	// Snapshot data
	// Interrupt -> main code
	int32_t oldVelocity;
	int32_t oldVelAccu;
	// Main code -> interrupt
	uint32_t newAcceleration;
	int32_t stepDifference;
	StepDirMode mode;
	uint32_t frequency;

	TMC_LinearRamp ramp;
    } StepGeneraorTypedef;

#endif /* TMC2130_STEP_GENERATOR_H_ */

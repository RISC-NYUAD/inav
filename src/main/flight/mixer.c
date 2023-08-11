/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/pwm_output.h"
#include "drivers/pwm_mapping.h"
#include "drivers/time.h"

#include "fc/fc_core.h"
#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "fc/controlrate_profile.h"
#include "fc/settings.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/servos.h"

#include "sensors/gyro.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"
#include "navigation/navigation_pos_estimator_private.h"

#include "rx/rx.h"

#include "sensors/battery.h"

FASTRAM int16_t motor[MAX_SUPPORTED_MOTORS];
FASTRAM int16_t motor_disarmed[MAX_SUPPORTED_MOTORS];
static float motorMixRange;
static float mixerScale = 1.0f;
static EXTENDED_FASTRAM motorMixer_t currentMixer[MAX_SUPPORTED_MOTORS];
static EXTENDED_FASTRAM uint8_t motorCount = 0;
EXTENDED_FASTRAM int mixerThrottleCommand;
//EXTENDED_FASTRAM int mixerFxCommand;
//EXTENDED_FASTRAM int mixerFyCommand;
static EXTENDED_FASTRAM int throttleIdleValue = 0;
static EXTENDED_FASTRAM int motorValueWhenStopped = 0;
static reversibleMotorsThrottleState_e reversibleMotorsThrottleState = MOTOR_DIRECTION_FORWARD;
static EXTENDED_FASTRAM int throttleDeadbandLow = 0;
static EXTENDED_FASTRAM int throttleDeadbandHigh = 0;
static EXTENDED_FASTRAM int throttleRangeMin = 0;
static EXTENDED_FASTRAM int throttleRangeMax = 0;
static EXTENDED_FASTRAM int8_t motorYawMultiplier = 1;

static float J_z = 0.0f;
static EXTENDED_FASTRAM mixerController_t controller_struct;
static bool angle_resetted = true;

/*
// TODO: add correct mixing table for OMNICOPTER
static const motorMixer_t mixerOmnicopter[] = {
  {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f},            // Motor 1
  {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f},            // Motor 2
  {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f},            // Motor 3
  {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f},            // Motor 4
  {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f},            // Motor 5
  {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f},            // Motor 6
  {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f},            // Motor 7
  {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f},            // Motor 8
};
*/
typedef struct{
  float angle_P ;
  float Kp ;
  float Kd ;
  float Ki ;
  float J ;
  float prev_verr ;
  float verr_dot ;

//  angle_struct_s() : angle_P(11.0), Kp(20.0), Kd(10.0), Ki(0.1), J(0.0), prev_verr(0.0), verr_dot(0.0) { }

}AngleStruct_t;

static EXTENDED_FASTRAM AngleStruct_t angleState[FLIGHT_DYNAMICS_INDEX_COUNT];



int motorZeroCommand = 0;

PG_REGISTER_WITH_RESET_TEMPLATE(reversibleMotorsConfig_t, reversibleMotorsConfig, PG_REVERSIBLE_MOTORS_CONFIG, 0);

PG_RESET_TEMPLATE(reversibleMotorsConfig_t, reversibleMotorsConfig,
    .deadband_low = SETTING_3D_DEADBAND_LOW_DEFAULT,
    .deadband_high = SETTING_3D_DEADBAND_HIGH_DEFAULT,
    .neutral = SETTING_3D_NEUTRAL_DEFAULT
);

PG_REGISTER_WITH_RESET_TEMPLATE(mixerConfig_t, mixerConfig, PG_MIXER_CONFIG, 5);

PG_RESET_TEMPLATE(mixerConfig_t, mixerConfig,
    .motorDirectionInverted = SETTING_MOTOR_DIRECTION_INVERTED_DEFAULT,
    .platformType = SETTING_PLATFORM_TYPE_DEFAULT,
    .hasFlaps = SETTING_HAS_FLAPS_DEFAULT,
    .appliedMixerPreset = SETTING_MODEL_PREVIEW_TYPE_DEFAULT, //This flag is not available in CLI and used by Configurator only
    .outputMode = SETTING_OUTPUT_MODE_DEFAULT,
);

PG_REGISTER_WITH_RESET_TEMPLATE(motorConfig_t, motorConfig, PG_MOTOR_CONFIG, 9);

PG_RESET_TEMPLATE(motorConfig_t, motorConfig,
    .motorPwmProtocol = SETTING_MOTOR_PWM_PROTOCOL_DEFAULT,
    .motorPwmRate = SETTING_MOTOR_PWM_RATE_DEFAULT,
    .maxthrottle = SETTING_MAX_THROTTLE_DEFAULT,
    .mincommand = SETTING_MIN_COMMAND_DEFAULT,
    .motorPoleCount = SETTING_MOTOR_POLES_DEFAULT,            // Most brushless motors that we use are 14 poles
);

PG_REGISTER_ARRAY(motorMixer_t, MAX_SUPPORTED_MOTORS, primaryMotorMixer, PG_MOTOR_MIXER, 0);

#define CRASH_OVER_AFTER_CRASH_FLIP_STICK_MIN 0.15f

int getThrottleIdleValue(void)
{
    if (!throttleIdleValue) {
        throttleIdleValue = motorConfig()->mincommand + (((motorConfig()->maxthrottle - motorConfig()->mincommand) / 100.0f) * currentBatteryProfile->motor.throttleIdle);
    }

    return throttleIdleValue;
}

static void computeMotorCount(void)
{
    motorCount = 0;
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        // check if done
        if (primaryMotorMixer(i)->throttle == 0.0f) {
            break;
        }
        motorCount++;
    }
}

uint8_t getMotorCount(void) {
    return motorCount;
}

float getMotorMixRange(void)
{
    return motorMixRange;
}

bool mixerIsOutputSaturated(void)
{
    return motorMixRange >= 1.0f;
}

void mixerUpdateStateFlags(void)
{
    DISABLE_STATE(FIXED_WING_LEGACY);
    DISABLE_STATE(MULTIROTOR);
    DISABLE_STATE(ROVER);
    DISABLE_STATE(BOAT);
    DISABLE_STATE(AIRPLANE);
    DISABLE_STATE(MOVE_FORWARD_ONLY);

    if (mixerConfig()->platformType == PLATFORM_AIRPLANE) {
        ENABLE_STATE(FIXED_WING_LEGACY);
        ENABLE_STATE(AIRPLANE);
        ENABLE_STATE(ALTITUDE_CONTROL);
        ENABLE_STATE(MOVE_FORWARD_ONLY);
    } if (mixerConfig()->platformType == PLATFORM_ROVER) {
        ENABLE_STATE(ROVER);
        ENABLE_STATE(FIXED_WING_LEGACY);
        ENABLE_STATE(MOVE_FORWARD_ONLY);
    } if (mixerConfig()->platformType == PLATFORM_BOAT) {
        ENABLE_STATE(BOAT);
        ENABLE_STATE(FIXED_WING_LEGACY);
        ENABLE_STATE(MOVE_FORWARD_ONLY);
    } else if (mixerConfig()->platformType == PLATFORM_MULTIROTOR) {
        ENABLE_STATE(MULTIROTOR);
        ENABLE_STATE(ALTITUDE_CONTROL);
    } else if (mixerConfig()->platformType == PLATFORM_TRICOPTER) {
        ENABLE_STATE(MULTIROTOR);
        ENABLE_STATE(ALTITUDE_CONTROL);
    } else if (mixerConfig()->platformType == PLATFORM_HELICOPTER) {
        ENABLE_STATE(MULTIROTOR);
        ENABLE_STATE(ALTITUDE_CONTROL);
    }

    if (mixerConfig()->hasFlaps) {
        ENABLE_STATE(FLAPERON_AVAILABLE);
    } else {
        DISABLE_STATE(FLAPERON_AVAILABLE);
    }
}

void nullMotorRateLimiting(const float dT)
{
    UNUSED(dT);
}

void mixerInit(void)
{
	//J(0.0), prev_verr(0.0), verr_dot(0.0)
	angleState[0].angle_P = 11.0;
	angleState[1].angle_P = 11.0;
	angleState[0].Kp = 20.0;
	angleState[1].Kp = 20.0;
	angleState[0].Kd = 10.0;
	angleState[1].Kd = 10.0;
	angleState[0].Ki = 0.1;
	angleState[1].Ki = 0.1;
	angleState[2].angle_P = 2.5;
	angleState[2].Kp = 5.0;
	angleState[2].Kd = 3.0;
	angleState[2].Ki = 1.2;	

    computeMotorCount();
    loadPrimaryMotorMixer();
    // in 3D mode, mixer gain has to be halved
    if (feature(FEATURE_REVERSIBLE_MOTORS)) {
        mixerScale = 0.5f;
    }

    throttleDeadbandLow = PWM_RANGE_MIDDLE - rcControlsConfig()->mid_throttle_deadband;
    throttleDeadbandHigh = PWM_RANGE_MIDDLE + rcControlsConfig()->mid_throttle_deadband;

    mixerResetDisarmedMotors();

    if (mixerConfig()->motorDirectionInverted) {
        motorYawMultiplier = -1;
    } else {
        motorYawMultiplier = 1;
    }
}

void mixerResetDisarmedMotors(void)
{

    if (feature(FEATURE_REVERSIBLE_MOTORS)) {
        motorZeroCommand = reversibleMotorsConfig()->neutral;
        throttleRangeMin = throttleDeadbandHigh;
        throttleRangeMax = motorConfig()->maxthrottle;
    } else {
        motorZeroCommand = motorConfig()->mincommand;
        throttleRangeMin = getThrottleIdleValue();
        throttleRangeMax = motorConfig()->maxthrottle;
    }

    reversibleMotorsThrottleState = MOTOR_DIRECTION_FORWARD;

    if (feature(FEATURE_MOTOR_STOP)) {
        motorValueWhenStopped = motorZeroCommand;
    } else {
        motorValueWhenStopped = getThrottleIdleValue();
    }

    // set disarmed motor values
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        motor_disarmed[i] = motorZeroCommand;
    }
}

#ifdef USE_DSHOT
static uint16_t handleOutputScaling(
    int16_t input,          // Input value from the mixer
    int16_t stopThreshold,  // Threshold value to check if motor should be rotating or not
    int16_t onStopValue,    // Value sent to the ESC when min rotation is required - on motor_stop it is STOP command, without motor_stop it's a value that keeps rotation
    int16_t inputScaleMin,  // Input range - min value
    int16_t inputScaleMax,  // Input range - max value
    int16_t outputScaleMin, // Output range - min value
    int16_t outputScaleMax, // Output range - max value
    bool moveForward        // If motor should be rotating FORWARD or BACKWARD
)
{
    int value;
    if (moveForward && input < stopThreshold) {
        //Send motor stop command
        value = onStopValue;
    }
    else if (!moveForward && input > stopThreshold) {
        //Send motor stop command
        value = onStopValue;
    }
    else {
        //Scale input to protocol output values
        value = scaleRangef(input, inputScaleMin, inputScaleMax, outputScaleMin, outputScaleMax);
        value = constrain(value, outputScaleMin, outputScaleMax);
    }
    return value;
}
static void applyTurtleModeToMotors(void) {

    if (ARMING_FLAG(ARMED)) {
        const float flipPowerFactor = ((float)currentBatteryProfile->motor.turtleModePowerFactor)/100.0f;
        const float stickDeflectionPitchAbs = ABS(((float) rcCommand[PITCH]) / 500.0f);
        const float stickDeflectionRollAbs = ABS(((float) rcCommand[ROLL]) / 500.0f);
        const float stickDeflectionYawAbs = ABS(((float) rcCommand[YAW]) / 500.0f);
        //deflection stick position

        const float stickDeflectionPitchExpo =
                flipPowerFactor * stickDeflectionPitchAbs + power3(stickDeflectionPitchAbs) * (1 - flipPowerFactor);
        const float stickDeflectionRollExpo =
                flipPowerFactor * stickDeflectionRollAbs + power3(stickDeflectionRollAbs) * (1 - flipPowerFactor);
        const float stickDeflectionYawExpo =
                flipPowerFactor * stickDeflectionYawAbs + power3(stickDeflectionYawAbs) * (1 - flipPowerFactor);

        float signPitch = rcCommand[PITCH] < 0 ? 1 : -1;
        float signRoll = rcCommand[ROLL] < 0 ? 1 : -1;
        float signYaw = (float)((rcCommand[YAW] < 0 ? 1 : -1) * (mixerConfig()->motorDirectionInverted ? 1 : -1));

        float stickDeflectionLength = calc_length_pythagorean_2D(stickDeflectionPitchAbs, stickDeflectionRollAbs);
        float stickDeflectionExpoLength = calc_length_pythagorean_2D(stickDeflectionPitchExpo, stickDeflectionRollExpo);

        if (stickDeflectionYawAbs > MAX(stickDeflectionPitchAbs, stickDeflectionRollAbs)) {
            // If yaw is the dominant, disable pitch and roll
            stickDeflectionLength = stickDeflectionYawAbs;
            stickDeflectionExpoLength = stickDeflectionYawExpo;
            signRoll = 0;
            signPitch = 0;
        } else {
            // If pitch/roll dominant, disable yaw
            signYaw = 0;
        }

        const float cosPhi = (stickDeflectionLength > 0) ? (stickDeflectionPitchAbs + stickDeflectionRollAbs) /
                                                           (fast_fsqrtf(2.0f) * stickDeflectionLength) : 0;
        const float cosThreshold = fast_fsqrtf(3.0f) / 2.0f; // cos(PI/6.0f)

        if (cosPhi < cosThreshold) {
            // Enforce either roll or pitch exclusively, if not on diagonal
            if (stickDeflectionRollAbs > stickDeflectionPitchAbs) {
                signPitch = 0;
            } else {
                signRoll = 0;
            }
        }

        // Apply a reasonable amount of stick deadband
        const float crashFlipStickMinExpo =
                flipPowerFactor * CRASH_OVER_AFTER_CRASH_FLIP_STICK_MIN + power3(CRASH_OVER_AFTER_CRASH_FLIP_STICK_MIN) * (1 - flipPowerFactor);
        const float flipStickRange = 1.0f - crashFlipStickMinExpo;
        const float flipPower = MAX(0.0f, stickDeflectionExpoLength - crashFlipStickMinExpo) / flipStickRange;

        for (int i = 0; i < motorCount; ++i) {

            float motorOutputNormalised =
                    signPitch * currentMixer[i].pitch +
                    signRoll * currentMixer[i].roll +
                    signYaw * currentMixer[i].yaw;

            if (motorOutputNormalised < 0) {
                motorOutputNormalised = 0;
            }

            motorOutputNormalised = MIN(1.0f, flipPower * motorOutputNormalised);

            motor[i] = (int16_t)scaleRangef(motorOutputNormalised, 0, 1, motorConfig()->mincommand, motorConfig()->maxthrottle);
        }
    } else {
        // Disarmed mode
        stopMotors();
    }
}
#endif

void FAST_CODE writeMotors(void)
{    
//	printf("%d\n",motorCount);

    for (int i = 0; i < motorCount; i++) {
        uint16_t motorValue;

#ifdef USE_DSHOT
        // If we use DSHOT we need to convert motorValue to DSHOT ranges
        if (isMotorProtocolDigital()) {

            if (feature(FEATURE_REVERSIBLE_MOTORS)) {
                if (reversibleMotorsThrottleState == MOTOR_DIRECTION_FORWARD) {
                    motorValue = handleOutputScaling(
                        motor[i],
                        throttleRangeMin,
                        DSHOT_DISARM_COMMAND,
                        throttleRangeMin,
                        throttleRangeMax,
                        DSHOT_3D_DEADBAND_HIGH,
                        DSHOT_MAX_THROTTLE,
                        true
                    );
                } else {
                    motorValue = handleOutputScaling(
                        motor[i],
                        throttleRangeMax,
                        DSHOT_DISARM_COMMAND,
                        throttleRangeMin,
                        throttleRangeMax,
                        DSHOT_MIN_THROTTLE,
                        DSHOT_3D_DEADBAND_LOW,
                        false
                    );
                }
            }
            else {
                motorValue = handleOutputScaling(
                    motor[i],
                    throttleIdleValue,
                    DSHOT_DISARM_COMMAND,
                    motorConfig()->mincommand,
                    motorConfig()->maxthrottle,
                    DSHOT_MIN_THROTTLE,
                    DSHOT_MAX_THROTTLE,
                    true
                );
            }
        }
        else {
            if (feature(FEATURE_REVERSIBLE_MOTORS)) {
                if (reversibleMotorsThrottleState == MOTOR_DIRECTION_FORWARD) {
                    motorValue = handleOutputScaling(
                        motor[i],
                        throttleRangeMin,
                        motor[i],
                        throttleRangeMin,
                        throttleRangeMax,
                        reversibleMotorsConfig()->deadband_high,
                        motorConfig()->maxthrottle,
                        true
                    );
                } else {
                    motorValue = handleOutputScaling(
                        motor[i],
                        throttleRangeMax,
                        motor[i],
                        throttleRangeMin,
                        throttleRangeMax,
                        motorConfig()->mincommand,
                        reversibleMotorsConfig()->deadband_low,
                        false
                    );
                }
            } else {
                motorValue = motor[i];
            }

        }
#else
        // We don't define USE_DSHOT
        motorValue = motor[i];
#endif
		//printf("%d\n",motorValue);
        pwmWriteMotor(i, motorValue);
    }
}

void writeAllMotors(int16_t mc)
{
    // Sends commands to all motors
    for (int i = 0; i < motorCount; i++) {
        motor[i] = mc;
    }
    writeMotors();
}

void stopMotors(void)
{
    writeAllMotors(feature(FEATURE_REVERSIBLE_MOTORS) ? reversibleMotorsConfig()->neutral : motorConfig()->mincommand);

    delay(50); // give the timers and ESCs a chance to react.
}

void stopPwmAllMotors(void)
{

    pwmShutdownPulsesForAllMotors(motorCount);

}

static int getReversibleMotorsThrottleDeadband(void)
{
    int directionValue;

    if (reversibleMotorsThrottleState == MOTOR_DIRECTION_BACKWARD) {
        directionValue = reversibleMotorsConfig()->deadband_low;
    } else {
        directionValue = reversibleMotorsConfig()->deadband_high;
    }

    return feature(FEATURE_MOTOR_STOP) ? reversibleMotorsConfig()->neutral : directionValue;
}

void FAST_CODE mixTable(float dT)
{
#ifdef USE_DSHOT
    if (FLIGHT_MODE(TURTLE_MODE)) {
        applyTurtleModeToMotors();
        return;
    }
#endif
#ifdef PLATFORM_IS_OMNICOPTER

//	printf("Py:%.2f, Vy:%.2f\n",py,vy);
//    int16_t input[3];   // RPY, range [-500:+500]
//    input[ROLL] = axisPID[ROLL];
//    input[PITCH] = axisPID[PITCH];
//    input[YAW] = axisPID[YAW];
	float py = navGetCurrentActualPositionAndVelocity()->pos.y; //cm
	float vy = navGetCurrentActualPositionAndVelocity()->vel.y; //cmps
	float px = navGetCurrentActualPositionAndVelocity()->pos.x;
	float vx = navGetCurrentActualPositionAndVelocity()->vel.x;
	float pz = navGetCurrentActualPositionAndVelocity()->pos.z;
	float vz = navGetCurrentActualPositionAndVelocity()->vel.z;

    int16_t rpyMix[MAX_SUPPORTED_MOTORS];
    int16_t rpyMixMax = 0; // assumption: symetrical about zero.
    int16_t rpyMixMin = 0;
    int16_t rpyMixRange = rpyMixMax - rpyMixMin;
    int16_t throttleRange;
    int16_t throttleMin, throttleMax;


	int16_t real_rll = attitude.values.roll;
	int16_t real_pit = attitude.values.pitch;
	int16_t real_yaw = attitude.values.yaw; //in decidegrees i.e. *0.1 turns it into degrees
	controller_struct.phi = (float) real_rll*0.1*0.01745333 ;
	controller_struct.theta = (float) real_pit*0.1*0.01745333 ;
	float yaw_0_2pi = (float) real_yaw*0.1*0.01745333 ;
	controller_struct.psi = atan2_approx(sin_approx(yaw_0_2pi), cos_approx(yaw_0_2pi)); //make it -pi, pi
	controller_struct.phi_d = gyro.gyroADCf[0]*0.01745333;
	controller_struct.theta_d = gyro.gyroADCf[1]*0.01745333;
	controller_struct.psi_d = -gyro.gyroADCf[2]*0.01745333; //NOTE: (-) here since it appears to be NED
//	printf("Yaw: %.3f, Yaw_dot: %.3f\n",controller_struct.psi,controller_struct.psi_d);

	if(isNavHoldPositionActive()){
		J_z += dT * (pz - 300.0); //posControl.desiredState.pos.z
		if(J_z > 2000.0) J_z = 2000.0;
		if(J_z < -2000.0) J_z = -2000.0;
		float alt_ctrl = 1300.0 -0.3*(pz - 300.0) - 0.08*(vz - 0.0) - 0.05*J_z ;
		mixerThrottleCommand = (int) alt_ctrl;
//		printf("%.2f, %.2f, %.2f\n", px, py, pz);
	}else{
		mixerThrottleCommand = rcCommand[THROTTLE];
		//printf("%d\n",mixerThrottleCommand);
	}
	float roll_cmd = 0.0;
	float pitch_cmd = 0.0;
	if(isNavHoldPositionActive()){
		float x_ctrl = -0.2*(px - posControl.desiredState.pos.x) - 0.09*vx ;
		float y_ctrl = -0.2*(py - posControl.desiredState.pos.y) - 0.09*vy ;
		float x_body_ctrl = posControl.actualState.cosYaw * x_ctrl - posControl.actualState.sinYaw * y_ctrl ;
		float y_body_ctrl = -posControl.actualState.sinYaw * x_ctrl - posControl.actualState.cosYaw * y_ctrl ;
		float roll_cmd = x_body_ctrl * 1.0 ;
		float pitch_cmd = y_body_ctrl * 1.0 ;
		rcCommand[FD_PITCH] = pidAngleToRcCommand(pitch_cmd, pidProfile()->max_angle_inclination[FD_PITCH]);
		rcCommand[FD_ROLL] = pidAngleToRcCommand(roll_cmd, pidProfile()->max_angle_inclination[FD_ROLL]);
		OVERRIDE_OMNI[FD_ROLL] = roll_cmd;
		OVERRIDE_OMNI[FD_PITCH] = pitch_cmd;
		//printf("%d,%d\n",OVERRIDE_OMNI[FD_ROLL],OVERRIDE_OMNI[FD_PITCH]);
		//printf("%.2f,%.2f\n",posControl.desiredState.pos.x,posControl.desiredState.pos.y);
	}
	float phiTarget = 0.001 * OVERRIDE_OMNI[FD_ROLL];
	float thetaTarget = 0.001 * OVERRIDE_OMNI[FD_PITCH];
	//float psiTarget = getFlightAxisAngleOverride(2, computePidLevelTarget(2));
	printf("%.2f, %.2f\n", phiTarget, thetaTarget);

	float error = controller_struct.phi - phiTarget ; 
	float v_des = -angleState[0].angle_P * error ;
	float v_err = controller_struct.phi_d - v_des ;
	angleState[0].J += dT * v_err ;
	float v_err_dot = (v_err - angleState[0].prev_verr)/dT ;
	angleState[0].verr_dot = 0.2*v_err_dot + 0.8*angleState[0].verr_dot ;
	angleState[0].prev_verr = v_err;
	float tx = -angleState[0].Kp * v_err - angleState[0].Kd * angleState[0].verr_dot - angleState[0].Ki * angleState[0].J ;
	tx *= 0.04365;

	error = controller_struct.theta - thetaTarget ; 
	v_des = -angleState[1].angle_P * error ;
	v_err = controller_struct.theta_d - v_des ;
	angleState[1].J += dT * v_err ;
	v_err_dot = (v_err - angleState[1].prev_verr)/dT ;
	angleState[1].verr_dot = 0.2*v_err_dot + 0.8*angleState[1].verr_dot ;
	angleState[1].prev_verr = v_err;
	float ty = -angleState[1].Kp * v_err - angleState[1].Kd * angleState[1].verr_dot - angleState[1].Ki * angleState[1].J ;
	ty *= 0.062;

	error = controller_struct.psi - 0.0 ;
	float yaw_rate_des = -angleState[2].angle_P * atan2_approx(sin_approx(error),cos_approx(error)) ;
	error = controller_struct.psi_d - yaw_rate_des;
	angleState[2].J += dT * error ;
	v_err = (error - angleState[2].prev_verr)/dT ;
	angleState[2].prev_verr = error ;
	float tz = -angleState[2].Kp * error - angleState[2].Kd * v_err - angleState[2].Ki * angleState[2].J ;
	tz *= 0.105;
	if(fabsf(angleState[2].verr_dot - tz) > 10.0){
		tz *= 0.01;
	}
	angleState[2].verr_dot = 0.05*tz + 0.95*angleState[2].verr_dot ; 
	tz = angleState[2].verr_dot;


//    mixerThrottleCommand = rcCommand[THROTTLE];
	float fx = 0.0;
	float fy = 0.0;
	float fz = (float) (mixerThrottleCommand/70.0) ;
//	float tx = (float) (input[ROLL]/1000.0) ;
//	float ty = (float) (input[PITCH]/1000.0) ;
//	float tz = (float) (input[YAW]/500.0) ;
	//printf("%.2f,%.3f,%.3f,%.3f\n", fz,tx,ty,tz);

	float forces[8];
	forces[0] = 0.05672595*fx + 0.30532735*fy + 0.22103447*fz + 1.34366265*tx - 0.20123229*ty + 0.10340595*tz;
	forces[1] = -0.29740887*fx + 0.24432859*fy + 0.17201345*fz - 1.23561926*tx - 0.20164879*ty - 0.3404224*tz;
	forces[2] = -0.22981984*fx - 0.29660767*fy + 0.00229046*fz + 0.07218196*tx + 0.05310532*ty - 0.44562788*tz;
	forces[3] = 0.01294391*fx + 0.23981251*fy - 0.34150929*fz + 0.29266017*tx + 1.00092644*ty + 0.3116706*tz;
	forces[4] = 0.50699376*fx + 0.00542995*fy + 0.02709579*fz + 1.1140566*tx + 0.49020019*ty - 0.55076316*tz;
	forces[5] = -0.02700161*fx - 0.06731981*fy - 0.35838813*fz - 0.38171035*tx - 1.13501878*ty + 0.15742413*tz;
	forces[6] = 0.16147557*fx - 0.18154778*fy + 0.17071623*fz - 1.58165043*tx - 0.40588238*ty + 0.40624333*tz;
	forces[7] = -0.18915626*fx - 0.25156428*fy + 0.10476156*fz + 0.35410129*tx + 0.39684149*ty + 0.34967509*tz;
	float min_f = 0.0;
	for(int i = 0 ; i < 8 ; i++){
	  if(forces[i] < min_f){
		min_f = forces[i];
	  }
	}
	float addition = fabsf(min_f) * 1.04;
	float max_f = 0.0;
	for(int i = 0 ; i < 8 ; i++){
		forces[i] += addition;
		if(forces[i] > max_f){
			max_f = forces[i];
		}
	}
	if(max_f > 13.2){
		for(int i = 0 ; i < 8 ; i++){
			forces[i] *= (13.2/max_f);
		}
	}
	float normalized_forces[8];
	for(int i = 0 ; i < 8 ; i++){
		normalized_forces[i] = fast_fsqrtf(forces[i])*0.24055 ;  //should now be in 0-1, the constant is 1/sqrt(thr_const) * RPS_2_RPM / MAX_RPM
	}


    throttleRangeMin = throttleIdleValue;
    throttleRangeMax = motorConfig()->maxthrottle;
    #define THROTTLE_CLIPPING_FACTOR    0.33f
    if (ARMING_FLAG(ARMED)) {
		if(angle_resetted){
			angle_resetted = false; //when we arm, if the angle_reset is true, make it false. This will allow to reset again when we disarm
		}
        const motorStatus_e currentMotorStatus = getMotorStatus();
        for (int i = 0; i < motorCount; i++) {
            motor[i] = (int16_t) (normalized_forces[i] * (throttleRangeMax-throttleRangeMin) + throttleRangeMin) ;
			//printf("%.2f, %d, %d, %d\n",normalized_forces[i], motor[i], throttleRangeMin, throttleRangeMax);
			
            if (failsafeIsActive()) {
                motor[i] = constrain(motor[i], motorConfig()->mincommand, motorConfig()->maxthrottle);
            } else {
                motor[i] = constrain(motor[i], throttleRangeMin, throttleRangeMax);
            }

            // Motor stop handling
            if (currentMotorStatus != MOTOR_RUNNING) {
                motor[i] = motorValueWhenStopped;
            }
        }
    } else {
		if(!angle_resetted){//when we disarm, if the angle_reset is false, reset and make it true.
			for(int i = 0 ; i < 3 ; i++){
				angleState[i].verr_dot = 0.0;
				angleState[i].prev_verr = 0.0;
				angleState[i].J = 0.0;
			}
			angle_resetted = true;
		}
        for (int i = 0; i < motorCount; i++) {
            motor[i] = motor_disarmed[i];
        }
    }//printf("\n");

#else
    int16_t input[3];   // RPY, range [-500:+500]
    // Allow direct stick input to motors in passthrough mode on airplanes
    if (STATE(FIXED_WING_LEGACY) && FLIGHT_MODE(MANUAL_MODE)) {
        // Direct passthru from RX
        input[ROLL] = rcCommand[ROLL];
        input[PITCH] = rcCommand[PITCH];
        input[YAW] = rcCommand[YAW];
    }
    else {
        input[ROLL] = axisPID[ROLL];
        input[PITCH] = axisPID[PITCH];
        input[YAW] = axisPID[YAW];
    }

    // Initial mixer concept by bdoiron74 reused and optimized for Air Mode
    int16_t rpyMix[MAX_SUPPORTED_MOTORS];
    int16_t rpyMixMax = 0; // assumption: symetrical about zero.
    int16_t rpyMixMin = 0;

    // motors for non-servo mixes
    for (int i = 0; i < motorCount; i++) {
        rpyMix[i] =
            (input[PITCH] * currentMixer[i].pitch +
            input[ROLL] * currentMixer[i].roll +
            -motorYawMultiplier * input[YAW] * currentMixer[i].yaw) * mixerScale;

        if (rpyMix[i] > rpyMixMax) rpyMixMax = rpyMix[i];
        if (rpyMix[i] < rpyMixMin) rpyMixMin = rpyMix[i];
    }

    int16_t rpyMixRange = rpyMixMax - rpyMixMin;
    int16_t throttleRange;
    int16_t throttleMin, throttleMax;
    // Find min and max throttle based on condition.
#ifdef USE_PROGRAMMING_FRAMEWORK
    if (LOGIC_CONDITION_GLOBAL_FLAG(LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_THROTTLE)) {
        throttleRangeMin = throttleIdleValue;
        throttleRangeMax = motorConfig()->maxthrottle;
        mixerThrottleCommand = constrain(logicConditionValuesByType[LOGIC_CONDITION_OVERRIDE_THROTTLE], throttleRangeMin, throttleRangeMax);
    } else
#endif
    if (feature(FEATURE_REVERSIBLE_MOTORS)) {

        if (rcCommand[THROTTLE] >= (throttleDeadbandHigh) || STATE(SET_REVERSIBLE_MOTORS_FORWARD)) {
            /*
             * Throttle is above deadband, FORWARD direction
             */
            reversibleMotorsThrottleState = MOTOR_DIRECTION_FORWARD;
            throttleRangeMax = motorConfig()->maxthrottle;
            throttleRangeMin = throttleDeadbandHigh;
            DISABLE_STATE(SET_REVERSIBLE_MOTORS_FORWARD);
        } else if (rcCommand[THROTTLE] <= throttleDeadbandLow) {
            /*
             * Throttle is below deadband, BACKWARD direction
             */
            reversibleMotorsThrottleState = MOTOR_DIRECTION_BACKWARD;
            throttleRangeMax = throttleDeadbandLow;
            throttleRangeMin = motorConfig()->mincommand;
        }


        motorValueWhenStopped = getReversibleMotorsThrottleDeadband();
        mixerThrottleCommand = constrain(rcCommand[THROTTLE], throttleRangeMin, throttleRangeMax);

#ifdef USE_DSHOT
        if(isMotorProtocolDigital() && feature(FEATURE_REVERSIBLE_MOTORS) && reversibleMotorsThrottleState == MOTOR_DIRECTION_BACKWARD) {
            /*
             * We need to start the throttle output from stick input to start in the middle of the stick at the low and.
             * Without this, it's starting at the high side.
             */
            int throttleDistanceToMax = throttleRangeMax - rcCommand[THROTTLE];
            mixerThrottleCommand = throttleRangeMin + throttleDistanceToMax;
        }
#endif
    } else {
        mixerThrottleCommand = rcCommand[THROTTLE];
        throttleRangeMin = throttleIdleValue;
        throttleRangeMax = motorConfig()->maxthrottle;

        // Throttle scaling to limit max throttle when battery is full
    #ifdef USE_PROGRAMMING_FRAMEWORK
        mixerThrottleCommand = ((mixerThrottleCommand - throttleRangeMin) * getThrottleScale(currentBatteryProfile->motor.throttleScale)) + throttleRangeMin;
    #else
        mixerThrottleCommand = ((mixerThrottleCommand - throttleRangeMin) * currentBatteryProfile->motor.throttleScale) + throttleRangeMin;
    #endif
        // Throttle compensation based on battery voltage
        if (feature(FEATURE_THR_VBAT_COMP) && isAmperageConfigured() && feature(FEATURE_VBAT)) {
            mixerThrottleCommand = MIN(throttleRangeMin + (mixerThrottleCommand - throttleRangeMin) * calculateThrottleCompensationFactor(), throttleRangeMax);
        }
    }

    throttleMin = throttleRangeMin;
    throttleMax = throttleRangeMax;

    throttleRange = throttleMax - throttleMin;

    #define THROTTLE_CLIPPING_FACTOR    0.33f
    motorMixRange = (float)rpyMixRange / (float)throttleRange;
    if (motorMixRange > 1.0f) {
        for (int i = 0; i < motorCount; i++) {
            rpyMix[i] /= motorMixRange;
        }

        // Allow some clipping on edges to soften correction response
        throttleMin = throttleMin + (throttleRange / 2) - (throttleRange * THROTTLE_CLIPPING_FACTOR / 2);
        throttleMax = throttleMin + (throttleRange / 2) + (throttleRange * THROTTLE_CLIPPING_FACTOR / 2);
    } else {
        throttleMin = MIN(throttleMin + (rpyMixRange / 2), throttleMin + (throttleRange / 2) - (throttleRange * THROTTLE_CLIPPING_FACTOR / 2));
        throttleMax = MAX(throttleMax - (rpyMixRange / 2), throttleMin + (throttleRange / 2) + (throttleRange * THROTTLE_CLIPPING_FACTOR / 2));
    }

    // Now add in the desired throttle, but keep in a range that doesn't clip adjusted
    // roll/pitch/yaw. This could move throttle down, but also up for those low throttle flips.
    if (ARMING_FLAG(ARMED)) {
        const motorStatus_e currentMotorStatus = getMotorStatus();
        for (int i = 0; i < motorCount; i++) {
            motor[i] = rpyMix[i] + constrain(mixerThrottleCommand * currentMixer[i].throttle, throttleMin, throttleMax);
			
            if (failsafeIsActive()) {
                motor[i] = constrain(motor[i], motorConfig()->mincommand, motorConfig()->maxthrottle);
            } else {
                motor[i] = constrain(motor[i], throttleRangeMin, throttleRangeMax);
            }

            // Motor stop handling
            if (currentMotorStatus != MOTOR_RUNNING) {
                motor[i] = motorValueWhenStopped;
            }
#ifdef USE_DEV_TOOLS
            if (systemConfig()->groundTestMode) {
                motor[i] = motorZeroCommand;
            }
#endif
//			printf("%d:%d\n",i,motor[i]);
        }
    } else {
        for (int i = 0; i < motorCount; i++) {
            motor[i] = motor_disarmed[i];
        }
    }
#endif
}

int16_t getThrottlePercent(bool useScaled)
{
    int16_t thr = constrain(rcCommand[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX);
    const int idleThrottle = getThrottleIdleValue();
    
    if (useScaled) {
       thr = (thr - idleThrottle) * 100 / (motorConfig()->maxthrottle - idleThrottle);
    } else {
        thr = (rxGetChannelValue(THROTTLE) - PWM_RANGE_MIN) * 100 / (PWM_RANGE_MAX - PWM_RANGE_MIN);
    }
    return thr;
}

motorStatus_e getMotorStatus(void)
{
    if (failsafeRequiresMotorStop()) {
        return MOTOR_STOPPED_AUTO;
    }

    if (!failsafeIsActive() && STATE(NAV_MOTOR_STOP_OR_IDLE)) {
        return MOTOR_STOPPED_AUTO;
    }

    const bool fixedWingOrAirmodeNotActive = STATE(FIXED_WING_LEGACY) || !STATE(AIRMODE_ACTIVE);

    if (throttleStickIsLow() && fixedWingOrAirmodeNotActive) {
        if ((navConfig()->general.flags.nav_overrides_motor_stop == NOMS_OFF_ALWAYS) && failsafeIsActive()) {
            // If we are in failsafe and user was holding stick low before it was triggered and nav_overrides_motor_stop is set to OFF_ALWAYS
            // and either on a plane or on a quad with inactive airmode - stop motor
            return MOTOR_STOPPED_USER;

        } else if (!failsafeIsActive()) {
            // If user is holding stick low, we are not in failsafe and either on a plane or on a quad with inactive
            // airmode - we need to check if we are allowing navigation to override MOTOR_STOP

            switch (navConfig()->general.flags.nav_overrides_motor_stop) {
                case NOMS_ALL_NAV:
                    return navigationInAutomaticThrottleMode() ? MOTOR_RUNNING : MOTOR_STOPPED_USER;

                case NOMS_AUTO_ONLY:
                    return navigationIsFlyingAutonomousMode() ? MOTOR_RUNNING : MOTOR_STOPPED_USER;

                case NOMS_OFF:
                default:
                    return MOTOR_STOPPED_USER;
            }
        }
    }

    return MOTOR_RUNNING;
}

void loadPrimaryMotorMixer(void) {
    for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        currentMixer[i] = *primaryMotorMixer(i);
    }
}

bool areMotorsRunning(void)
{
    if (ARMING_FLAG(ARMED)) {
        return true;
    } else {
        for (int i = 0; i < motorCount; i++) {
            if (motor_disarmed[i] != motorZeroCommand) {
                return true;
            }
        }
    }

    return false;
}

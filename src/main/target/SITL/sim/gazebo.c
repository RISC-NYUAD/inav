/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <pthread.h>
#include <math.h>

#include "platform.h"

#include "target.h"
#include "target/SITL/udplink.h"
#include "target/SITL/sim/gazebo.h"
#include "target/SITL/dyad.h"
#include "target/SITL/sim/realFlight.h"
#include "target/SITL/sim/simple_soap_client.h"
#include "target/SITL/sim/xplane.h"
#include "target/SITL/sim/simHelper.h"
#include "fc/runtime_config.h"
#include "drivers/time.h"
#include "drivers/accgyro/accgyro_fake.h"
#include "drivers/barometer/barometer_fake.h"
#include "sensors/battery_sensor_fake.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "drivers/pitotmeter/pitotmeter_fake.h"
#include "drivers/compass/compass_fake.h"
#include "drivers/rangefinder/rangefinder_virtual.h"
#include "io/rangefinder.h"
#include "common/utils.h"
#include "common/maths.h"
#include "flight/mixer.h"
#include "flight/servos.h"
#include "flight/imu.h"
#include "io/gps.h"
#include "rx/rx.h"
#include "rx/sim.h"

#define GZ_PORT 18083
#define GZ_MAX_CHANNEL_COUNT 12
// RealFlight scenerys doesn't represent real landscapes, so fake some nice coords: Area 51 ;)
#define FAKE_LAT 37.277127f
#define FAKE_LON -115.799669f

static char simulator_ip[32] = "127.0.0.1";

#define PORT_PWM_RAW    9001    // Out
#define PORT_PWM        9002    // Out
#define PORT_STATE      9003    // In
#define PORT_RC         9004    // In


static fdm_packet fdmPkt;
static rc_packet rcPkt;
static servo_packet pwmPkt;
static servo_packet_raw pwmRawPkt;

static bool rc_received = false;
static bool fdm_received = false;

static bool workerRunning = true;
static pthread_t tcpWorker, udpWorker, udpWorkerRC;
static udpLink_t stateLink, pwmLink, pwmRawLink, rcLink;
static uint8_t pwmMapping[GZ_MAX_PWM_OUTS];
static uint8_t mappingCount;

static bool isInitalised = false;
static bool useImu = false;

typedef struct 
{
    double m_channelValues[GZ_MAX_PWM_OUTS];
    double m_currentPhysicsSpeedMultiplier;
    double m_currentPhysicsTime_SEC;
    double m_airspeed_MPS;
    double m_altitudeASL_MTR;
    double m_altitudeAGL_MTR;
    double m_groundspeed_MPS;
    double m_pitchRate_DEGpSEC;
    double m_rollRate_DEGpSEC;
    double m_yawRate_DEGpSEC;
    double m_azimuth_DEG;
    double m_inclination_DEG;
    double m_roll_DEG;
    double m_orientationQuaternion_X;
    double m_orientationQuaternion_Y;
    double m_orientationQuaternion_Z;
    double m_orientationQuaternion_W;
    double m_aircraftPositionX_MTR;
    double m_aircraftPositionY_MTR;
    double m_velocityWorldU_MPS;
    double m_velocityWorldV_MPS;
    double m_velocityWorldW_MPS;
    double m_velocityBodyU_MPS;
    double m_velocityBodyV_MPS;
    double m_velocityBodyW_MPS;
    double m_accelerationWorldAX_MPS2;
    double m_accelerationWorldAY_MPS2;
    double m_accelerationWorldAZ_MPS2;
    double m_accelerationBodyAX_MPS2;
    double m_accelerationBodyAY_MPS2;
    double m_accelerationBodyAZ_MPS2;
    double m_windX_MPS;
    double m_windY_MPS;
    double m_windZ_MPSPS;
    double m_propRPM;
    double m_heliMainRotorRPM;
    double m_batteryVoltage_VOLTS;
    double m_batteryCurrentDraw_AMPS;
    double m_batteryRemainingCapacity_MAH;
    double m_fuelRemaining_OZ;
    bool m_isLocked;
    bool m_hasLostComponents;
    bool m_anEngineIsRunning;
    bool m_isTouchingGround;
    bool m_flightAxisControllerIsActive;
    char* m_currentAircraftStatus;
    bool m_resetButtonHasBeenPressed;
} rfValues_t;

rfValues_t rfValues; 

/*
static bool getChannelValues(const char* response, uint16_t* channelValues)
{
    if (!response) {
        return false;
    }
    
    const char* channelValueTag = "<m-channelValues-0to1 xsi:type=\"SOAP-ENC:Array\" SOAP-ENC:arrayType=\"xsd:double[12]\">";
    char* pos = strstr(response, channelValueTag);
    if (!pos){
        return false;
    }

    pos += strlen(channelValueTag);
    for (size_t i = 0; i < RF_MAX_CHANNEL_COUNT; i++) {
        char* end = strstr(pos, "</");
        if (!end) {
            return false;
        }

        channelValues[i] = FLOAT_0_1_TO_PWM((float)atof(pos + 6));
        pos = end + 7;   
    }

    return true;
}
*/

static void fakeCoords(double posX, double posY, double distanceX, double distanceY, double *lat, double *lon)
{
    double m = 1 / (2 * (double)M_PIf / 360 * EARTH_RADIUS) / 1000;
    *lat = (double)(posX + (distanceX * m));
    *lon = (double)(posY + (distanceY * m) / cos(posX * ((double)M_PIf / 180)));
} 

static double convertAzimuth(double azimuth)
{
    if (azimuth < 0) {
        azimuth += 360;
    }
    return 360 - fmod(azimuth + 90, 360.0f);
}

/*
static void exchangeData(void)
{
    double servoValues[RF_MAX_PWM_OUTS] = { 0 };    
    for (int i = 0; i < mappingCount; i++) {
        if (pwmMapping[i] & 0x80){ // Motor
            servoValues[i] = PWM_TO_FLOAT_0_1(motor[pwmMapping[i] & 0x7f]);
        } else { 
            servoValues[i] = PWM_TO_FLOAT_0_1(servo[pwmMapping[i]]);
        }
    }

    startRequest("ExchangeData", "<ExchangeData><pControlInputs><m-selectedChannels>%u</m-selectedChannels><m-channelValues-0to1><item>%.4f</item><item>%.4f</item><item>%.4f</item><item>%.4f</item><item>%.4f</item><item>%.4f</item><item>%.4f</item><item>%.4f</item><item>%.4f</item><item>%.4f</item><item>%.4f</item><item>%.4f</item></m-channelValues-0to1></pControlInputs></ExchangeData>",
        0xFFF, servoValues[0], servoValues[1], servoValues[2], servoValues[3], servoValues[4], servoValues[5], servoValues[6], servoValues[7], servoValues[8], servoValues[9], servoValues[10], servoValues[11]);
    char* response = endRequest();

    //rfValues.m_currentPhysicsTime_SEC = getDoubleFromResponse(response, "m-currentPhysicsTime-SEC");
    //rfValues.m_currentPhysicsSpeedMultiplier = getDoubleFromResponse(response, "m-currentPhysicsSpeedMultiplier");
    rfValues.m_airspeed_MPS = getDoubleFromResponse(response, "m-airspeed-MPS");
    rfValues.m_altitudeASL_MTR = getDoubleFromResponse(response, "m-altitudeASL-MTR");
    rfValues.m_altitudeAGL_MTR = getDoubleFromResponse(response, "m-altitudeAGL-MTR");
    rfValues.m_groundspeed_MPS = getDoubleFromResponse(response, "m-groundspeed-MPS");
    rfValues.m_pitchRate_DEGpSEC = getDoubleFromResponse(response, "m-pitchRate-DEGpSEC");
    rfValues.m_rollRate_DEGpSEC = getDoubleFromResponse(response, "m-rollRate-DEGpSEC");
    rfValues.m_yawRate_DEGpSEC = getDoubleFromResponse(response, "m-yawRate-DEGpSEC");
    rfValues.m_azimuth_DEG = getDoubleFromResponse(response, "m-azimuth-DEG");
    rfValues.m_inclination_DEG = getDoubleFromResponse(response, "m-inclination-DEG");
    rfValues.m_roll_DEG = getDoubleFromResponse(response, "m-roll-DEG");
    //rfValues.m_orientationQuaternion_X = getDoubleFromResponse(response, "m-orientationQuaternion-X");
    //rfValues.m_orientationQuaternion_Y = getDoubleFromResponse(response, "m-orientationQuaternion-Y");
    //rfValues.m_orientationQuaternion_Z = getDoubleFromResponse(response, "m-orientationQuaternion-Z");
    //rfValues.m_orientationQuaternion_W = getDoubleFromResponse(response, "m-orientationQuaternion-W");
    rfValues.m_aircraftPositionX_MTR = getDoubleFromResponse(response, "m-aircraftPositionX-MTR");
    rfValues.m_aircraftPositionY_MTR = getDoubleFromResponse(response, "m-aircraftPositionY-MTR");
    //rfValues.m_velocityWorldU_MPS = getDoubleFromResponse(response, "m-velocityWorldU-MPS");
    //rfValues.m_velocityWorldV_MPS = getDoubleFromResponse(response, "m-velocityWorldV-MPS");
    //rfValues.m_velocityWorldW_MPS = getDoubleFromResponse(response, "m-velocityWorldW-MPS");
    //rfValues.m_velocityBodyU_MPS = getDoubleFromResponse(response, "m-velocityBodyU-MPS");
    //rfValues.m_velocityBodyV_MPS = getDoubleFromResponse(response, "mm-velocityBodyV-MPS");
    //rfValues.m_velocityBodyW_MPS = getDoubleFromResponse(response, "m-velocityBodyW-MPS");
    //rfValues.m_accelerationWorldAX_MPS2 = getDoubleFromResponse(response, "m-accelerationWorldAX-MPS2");
    //rfValues.m_accelerationWorldAY_MPS2 = getDoubleFromResponse(response, "m-accelerationWorldAY-MPS2");
    //rfValues.m_accelerationWorldAZ_MPS2 = getDoubleFromResponse(response, "m-accelerationWorldAZ-MPS2");
    rfValues.m_accelerationBodyAX_MPS2 = getDoubleFromResponse(response, "m-accelerationBodyAX-MPS2");
    rfValues.m_accelerationBodyAY_MPS2 = getDoubleFromResponse(response, "m-accelerationBodyAY-MPS2");
    rfValues.m_accelerationBodyAZ_MPS2 = getDoubleFromResponse(response, "m-accelerationBodyAZ-MPS2");
    //rfValues.m_windX_MPS = getDoubleFromResponse(response, "m-windX-MPS");
    //rfValues.m_windY_MPS = getDoubleFromResponse(response, "m-windY-MPS");
    //rfValues.m_windZ_MPSPS = getDoubleFromResponse(response, "m-windZ-MPS");
    //rfValues.m_propRPM = getDoubleFromResponse(response, "m-propRPM");
    //rfValues.m_heliMainRotorRPM = getDoubleFromResponse(response, "m-heliMainRotorRPM");
    rfValues.m_batteryVoltage_VOLTS = getDoubleFromResponse(response, "m-batteryVoltage-VOLTS");
    rfValues.m_batteryCurrentDraw_AMPS = getDoubleFromResponse(response, "m-batteryCurrentDraw-AMPS");
    //rfValues.m_batteryRemainingCapacity_MAH = getDoubleFromResponse(response, "m-batteryRemainingCapacity-MAH");
    //rfValues.m_fuelRemaining_OZ = getDoubleFromResponse(response, "m-fuelRemaining-OZ");
    //rfValues.m_isLocked = getBoolFromResponse(response, "m-isLocked");
    //rfValues.m_hasLostComponents = getBoolFromResponse(response, "m-hasLostComponents");
    //rfValues.m_anEngineIsRunning = getBoolFromResponse(response, "m-anEngineIsRunning");
    //rfValues.m_isTouchingGround = getBoolFromResponse(response, "m-isTouchingGround");
    //rfValues.m_flightAxisControllerIsActive= getBoolFromResponse(response, "m-flightAxisControllerIsActive");
    rfValues.m_currentAircraftStatus = getStringFromResponse(response, "m-currentAircraftStatus");

    
    uint16_t channelValues[RF_MAX_CHANNEL_COUNT];
    getChannelValues(response, channelValues);
    rxSimSetChannelValue(channelValues, RF_MAX_CHANNEL_COUNT);
    
    double lat, lon;
    fakeCoords(FAKE_LAT, FAKE_LON, rfValues.m_aircraftPositionX_MTR, -rfValues.m_aircraftPositionY_MTR, &lat, &lon);
    
    int16_t course = (int16_t)round(convertAzimuth(rfValues.m_azimuth_DEG) * 10);
    int32_t altitude = (int32_t)round(rfValues.m_altitudeASL_MTR * 100);
    gpsFakeSet(
        GPS_FIX_3D,
        16,
        (int32_t)round(lat * 10000000),
        (int32_t)round(lon * 10000000),
        altitude,
        (int16_t)round(rfValues.m_groundspeed_MPS * 100),
        course,
        0, 
        0,
        0,
        0
    );

    int32_t altitudeOverGround = (int32_t)round(rfValues.m_altitudeAGL_MTR * 100);
    if (altitudeOverGround > 0 && altitudeOverGround <= RANGEFINDER_VIRTUAL_MAX_RANGE_CM) {
        fakeRangefindersSetData(altitudeOverGround);
    } else {
        fakeRangefindersSetData(-1);
    }

    const int16_t roll_inav = (int16_t)round(rfValues.m_roll_DEG * 10);
    const int16_t pitch_inav = (int16_t)round(-rfValues.m_inclination_DEG * 10);
    const int16_t yaw_inav = course;
    if (!useImu) {
        imuSetAttitudeRPY(roll_inav, pitch_inav, yaw_inav);
        imuUpdateAttitude(micros());
    }

    // RealFlights acc data is weird if the aircraft has not yet taken off. Fake 1G in horizontale position
    int16_t accX = 0;
    int16_t accY = 0;
    int16_t accZ = 0;
    if (rfValues.m_currentAircraftStatus && strncmp(rfValues.m_currentAircraftStatus, "CAS-WAITINGTOLAUNCH", strlen(rfValues.m_currentAircraftStatus)) == 0) {
        accX = 0;
        accY = 0;
        accZ = (int16_t)(GRAVITY_MSS * 1000.0f);
    } else {
         accX = constrainToInt16(rfValues.m_accelerationBodyAX_MPS2 * 1000);
         accY = constrainToInt16(-rfValues.m_accelerationBodyAY_MPS2 * 1000);
         accZ = constrainToInt16(-rfValues.m_accelerationBodyAZ_MPS2 * 1000);
    }

    fakeAccSet(accX, accY, accZ);

    fakeGyroSet(
        constrainToInt16(rfValues.m_rollRate_DEGpSEC * (double)16.0),
        constrainToInt16(-rfValues.m_pitchRate_DEGpSEC * (double)16.0),
        constrainToInt16(rfValues.m_yawRate_DEGpSEC * (double)16.0)
    );

    fakeBaroSet(altitudeToPressure(altitude), DEGREES_TO_CENTIDEGREES(21));
    fakePitotSetAirspeed(rfValues.m_airspeed_MPS * 100);

    fakeBattSensorSetVbat((uint16_t)round(rfValues.m_batteryVoltage_VOLTS * 100));
    fakeBattSensorSetAmperage((uint16_t)round(rfValues.m_batteryCurrentDraw_AMPS * 100)); 

    fpQuaternion_t quat;
    fpVector3_t north;
    north.x = 1.0f;
    north.y = 0;
    north.z = 0;
    computeQuaternionFromRPY(&quat, roll_inav, pitch_inav, yaw_inav);
    transformVectorEarthToBody(&north, &quat);
    fakeMagSet(
        constrainToInt16(north.x * 16000.0f),
        constrainToInt16(north.y * 16000.0f),
        constrainToInt16(north.z * 16000.0f)
    );
}
*/

//=============================BELOW==============================//
void updateState(const fdm_packet* pkt)
{
/*
    static double last_timestamp = 0; // in seconds
    static uint64_t last_realtime = 0; // in uS
    static struct timespec last_ts; // last packet

    struct timespec now_ts;
    clock_gettime(CLOCK_MONOTONIC, &now_ts);

    const uint64_t realtime_now = micros64_real();
    if (realtime_now > last_realtime + 500*1e3) { // 500ms timeout
        last_timestamp = pkt->timestamp;
        last_realtime = realtime_now;
        sendMotorUpdate();
        return;
    }

    const double deltaSim = pkt->timestamp - last_timestamp;  // in seconds
    if (deltaSim < 0) { // don't use old packet
        return;
    }

    int16_t x,y,z;
    x = constrain(-pkt->imu_linear_acceleration_xyz[0] * ACC_SCALE, -32767, 32767);
    y = constrain(-pkt->imu_linear_acceleration_xyz[1] * ACC_SCALE, -32767, 32767);
    z = constrain(-pkt->imu_linear_acceleration_xyz[2] * ACC_SCALE, -32767, 32767);
    fakeAccSet(fakeAccDev, x, y, z);
//    printf("[acc]%lf,%lf,%lf\n", pkt->imu_linear_acceleration_xyz[0], pkt->imu_linear_acceleration_xyz[1], pkt->imu_linear_acceleration_xyz[2]);

    x = constrain(pkt->imu_angular_velocity_rpy[0] * GYRO_SCALE * RAD2DEG, -32767, 32767);
    y = constrain(-pkt->imu_angular_velocity_rpy[1] * GYRO_SCALE * RAD2DEG, -32767, 32767);
    z = constrain(-pkt->imu_angular_velocity_rpy[2] * GYRO_SCALE * RAD2DEG, -32767, 32767);
    fakeGyroSet(fakeGyroDev, x, y, z);
//    printf("[gyr]%lf,%lf,%lf\n", pkt->imu_angular_velocity_rpy[0], pkt->imu_angular_velocity_rpy[1], pkt->imu_angular_velocity_rpy[2]);

    // temperature in 0.01 C = 25 deg
    fakeBaroSet(pkt->pressure, 2500);
#if !defined(USE_IMU_CALC)
#if defined(SET_IMU_FROM_EULER)
    // set from Euler
    double qw = pkt->imu_orientation_quat[0];
    double qx = pkt->imu_orientation_quat[1];
    double qy = pkt->imu_orientation_quat[2];
    double qz = pkt->imu_orientation_quat[3];
    double ysqr = qy * qy;
    double xf, yf, zf;

    // roll (x-axis rotation)
    double t0 = +2.0 * (qw * qx + qy * qz);
    double t1 = +1.0 - 2.0 * (qx * qx + ysqr);
    xf = atan2(t0, t1) * RAD2DEG;

    // pitch (y-axis rotation)
    double t2 = +2.0 * (qw * qy - qz * qx);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    yf = asin(t2) * RAD2DEG; // from wiki

    // yaw (z-axis rotation)
    double t3 = +2.0 * (qw * qz + qx * qy);
    double t4 = +1.0 - 2.0 * (ysqr + qz * qz);
    zf = atan2(t3, t4) * RAD2DEG;
    imuSetAttitudeRPY(xf, -yf, zf); // yes! pitch was inverted!!
#else
    imuSetAttitudeQuat(pkt->imu_orientation_quat[0], pkt->imu_orientation_quat[1], pkt->imu_orientation_quat[2], pkt->imu_orientation_quat[3]);
#endif
#endif

#if defined(SIMULATOR_IMU_SYNC)
    imuSetHasNewData(deltaSim*1e6);
    imuUpdateAttitude(micros());
#endif


    if (deltaSim < 0.02 && deltaSim > 0) { // simulator should run faster than 50Hz
//        simRate = simRate * 0.5 + (1e6 * deltaSim / (realtime_now - last_realtime)) * 0.5;
        struct timespec out_ts;
        timeval_sub(&out_ts, &now_ts, &last_ts);
        simRate = deltaSim / (out_ts.tv_sec + 1e-9*out_ts.tv_nsec);
    }
//    printf("simRate = %lf, millis64 = %lu, millis64_real = %lu, deltaSim = %lf\n", simRate, millis64(), millis64_real(), deltaSim*1e6);

    last_timestamp = pkt->timestamp;
    last_realtime = micros64_real();

    last_ts.tv_sec = now_ts.tv_sec;
    last_ts.tv_nsec = now_ts.tv_nsec;

    pthread_mutex_unlock(&updateLock); // can send PWM output now

#if defined(SIMULATOR_GYROPID_SYNC)
    pthread_mutex_unlock(&mainLoopLock); // can run main loop
#endif
*/
	if(!isInitalised){
		isInitalised = true;
	}

}


static void* tcpThread(void* data)
{
    UNUSED(data);

    dyad_init();
    dyad_setTickInterval(0.2f);
    dyad_setUpdateTimeout(0.5f);

    while (workerRunning) {
        dyad_update();
    }

    dyad_shutdown();
    printf("tcpThread end!!\n");
    return NULL;
}

static void* udpThread(void* data)
{
    UNUSED(data);
    int n = 0;

    while (workerRunning) {
        n = udpRecv(&stateLink, &fdmPkt, sizeof(fdm_packet), 100);
        if (n == sizeof(fdm_packet)) {
            if (!fdm_received) {
                printf("[SITL] new fdm %d t:%f from %s:%d\n", n, fdmPkt.timestamp, inet_ntoa(stateLink.recv.sin_addr), stateLink.recv.sin_port);
                fdm_received = true;
            }
            updateState(&fdmPkt);
        }
    }

    printf("udpThread end!!\n");
    return NULL;
}

static float readRCSITL(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t channel)
{
    UNUSED(rxRuntimeConfig);
    return rcPkt.channels[channel];
}

static uint8_t rxRCFrameStatus(rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxRuntimeConfig);
    return RX_FRAME_COMPLETE;
}

static void *udpRCThread(void *data)
{
    UNUSED(data);
    int n = 0;

    while (workerRunning) {
        n = udpRecv(&rcLink, &rcPkt, sizeof(rc_packet), 100);
        if (n == sizeof(rc_packet)) {
            if (!rc_received) {
                printf("[SITL] new rc %d: t:%f AETR: %d %d %d %d AUX1-4: %d %d %d %d\n", n, rcPkt.timestamp,
                    rcPkt.channels[0], rcPkt.channels[1],rcPkt.channels[2],rcPkt.channels[3],
                    rcPkt.channels[4], rcPkt.channels[5],rcPkt.channels[6],rcPkt.channels[7]);
                rc_received = true;
            }
        }
    }

    printf("udpRCThread end!!\n");
    return NULL;
}


bool simGazeboInit(char* ip, uint8_t* mapping, uint8_t mapCount, bool imu)
{
	int ret;

    memcpy(pwmMapping, mapping, mapCount);
    mappingCount = mapCount;
    useImu = imu;

    ret = pthread_create(&tcpWorker, NULL, tcpThread, NULL);
    if (ret != 0) {
        printf("Create tcpWorker error!\n");
        exit(1);
    }

    ret = udpInit(&pwmLink, simulator_ip, PORT_PWM, false);
    printf("[SITL] init PwmOut UDP link to gazebo %s:%d...%d\n", simulator_ip, PORT_PWM, ret);

    ret = udpInit(&pwmRawLink, simulator_ip, PORT_PWM_RAW, false);
    printf("[SITL] init PwmOut UDP link to RF9 %s:%d...%d\n", simulator_ip, PORT_PWM_RAW, ret);

    ret = udpInit(&stateLink, NULL, PORT_STATE, true);
    printf("[SITL] start UDP server @%d...%d\n", PORT_STATE, ret);

    ret = udpInit(&rcLink, NULL, PORT_RC, true);
    printf("[SITL] start UDP server for RC input @%d...%d\n", PORT_RC, ret);

//    ret = udpInit(&stateLink, NULL, 9003, true);
//    printf("start UDP server...%d\n", ret);

    ret = pthread_create(&udpWorker, NULL, udpThread, NULL);
    if (ret != 0) {
        printf("Create udpWorker error!\n");
        exit(1);
    }

    ret = pthread_create(&udpWorkerRC, NULL, udpRCThread, NULL);
    if (ret != 0) {
        printf("Create udpRCThread error!\n");
        exit(1);
    }

    // Wait until the connection is established, the interface has been initialised 
    // and the first valid packet has been received to avoid problems with the startup calibration.   
    while (!isInitalised) {
        delay(250);
    }

    return true;
}

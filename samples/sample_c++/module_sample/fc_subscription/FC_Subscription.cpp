/**
 ********************************************************************
 * @file    test_perception.cpp
 * @brief
 *
 * @copyright (c) 2021 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
    #include <stdexcept>
    #include "FC_Subscription.hpp"
    #include "dji_logger.h"
    #include <utils/util_misc.h>
    #include "dji_error.h"
    #include <stdint.h>
    #include <stddef.h>
    #include <stdio.h>

/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/


/* Private values -------------------------------------------------------------*/
  

/* Private functions declaration ---------------------------------------------*/
static T_DjiReturnCode Dji_FcSubscriptionReceiveQuaternionCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp);
static T_DjiReturnCode Dji_FcSubscriptionReceiveAccelerationGroundCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp);
static T_DjiReturnCode Dji_FcSubscriptionReceiveAccelerationBodyCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp);
static T_DjiReturnCode Dji_FcSubscriptionReceiveAccelerationRawCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp);
static T_DjiReturnCode Dji_FcSubscriptionReceiveVelocityCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp);
static T_DjiReturnCode Dji_FcSubscriptionReceiveAngularRateFusionedCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp);
static T_DjiReturnCode Dji_FcSubscriptionReceiveAngularRateRawCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp);
static T_DjiReturnCode Dji_FcSubscriptionReceiveAltitudeFusedCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp);
static T_DjiReturnCode Dji_FcSubscriptionReceiveAltitudeBarometerCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp);
static T_DjiReturnCode Dji_FcSubscriptionReceiveAltitudeOfHomePointCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp);
static T_DjiReturnCode Dji_FcSubscriptionReceiveHeightFusionCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp);
static T_DjiReturnCode Dji_FcSubscriptionReceiveHeightRelativeCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp);
static T_DjiReturnCode Dji_FcSubscriptionReceivePositionFusedCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp);
static T_DjiReturnCode Dji_FcSubscriptionReceiveCompassCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp);
static T_DjiReturnCode Dji_FcSubscriptionReceiveFlightStatusCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp);
static T_DjiReturnCode Dji_FcSubscriptionReceivePositionVOCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp);
static T_DjiReturnCode Dji_FcSubscriptionReceiveImuAttiNaviDataWithTimestampCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp);

/*
+ Quaternion DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION
+ AccelerationGround DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_GROUND
+ AccelerationBody DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_BODY
+ AccelerationRaw DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_RAW  
+ Velocity DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY
+ AngularRateFusioned DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_FUSIONED
+ AngularRateRaw DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_RAW
+ AltitudeFused DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED
+ AltitudeBarometer DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_BAROMETER
+ AltitudeOfHomePoint DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT
+ HeightFusion DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION
+ HeightRelative DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_RELATIVE
+ PositionFused DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED
- GpsDate DJI_FC_SUBSCRIPTION_TOPIC_GPS_DATE
- GpsTime DJI_FC_SUBSCRIPTION_TOPIC_GPS_TIME
- GpsPosition DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION
- GpsVelocity DJI_FC_SUBSCRIPTION_TOPIC_GPS_VELOCITY
- GpsDetails DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS
- GpsSignalLevel DJI_FC_SUBSCRIPTION_TOPIC_GPS_SIGNAL_LEVEL
- RtkPosition DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION
- RtkVelocity DJI_FC_SUBSCRIPTION_TOPIC_RTK_VELOCITY
- RtkYaw DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW
- RtkPositionInfo DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO
- RtkYawInfo DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW_INFO
+ Compass DJI_FC_SUBSCRIPTION_TOPIC_COMPASS
- RC DJI_FC_SUBSCRIPTION_TOPIC_RC
- GimbalAngles DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES
- GimbalStatus DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_STATUS
+ FlightStatus DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT
- Displaymode DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE
- Landinggear DJI_FC_SUBSCRIPTION_TOPIC_STATUS_LANDINGGEAR
- MotorStartError DJI_FC_SUBSCRIPTION_TOPIC_STATUS_MOTOR_START_ERROR
- WholeBatteryInfo DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_INFO
- ControlDevice DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE
- HardSync DJI_FC_SUBSCRIPTION_TOPIC_HARD_SYNC
- GpsControlLevel DJI_FC_SUBSCRIPTION_TOPIC_GPS_CONTROL_LEVEL
- RCWithFlagData DJI_FC_SUBSCRIPTION_TOPIC_RC_WITH_FLAG_DATA
- EscData DJI_FC_SUBSCRIPTION_TOPIC_ESC_DATA
- RTKConnectStatus DJI_FC_SUBSCRIPTION_TOPIC_RTK_CONNECT_STATUS
- GimbalControlMode DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_CONTROL_MODE
- FlightAnomaly DJI_FC_SUBSCRIPTION_TOPIC_FLIGHT_ANOMALY
+ PositionVO DJI_FC_SUBSCRIPTION_TOPIC_POSITION_VO
- AvoidData DJI_FC_SUBSCRIPTION_TOPIC_AVOID_DATA
- HomePointSetStatus DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_SET_STATUS
- HomePointInfo DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_INFO
- ThreeGimbalData DJI_FC_SUBSCRIPTION_TOPIC_THREE_GIMBAL_DATA 
- SingleBatteryInfoIndex1 DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX1
- SingleBatteryInfoIndex2 DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX2
+ ImuAttiNaviDataWithTimestamp DJI_FC_SUBSCRIPTION_TOPIC_IMU_ATTI_NAVI_DATA_WITH_TIMESTAMP
- TotalNumber DJI_FC_SUBSCRIPTION_TOPIC_TOTAL_NUMBER
*/

/* Exported functions definition ---------------------------------------------*/

FC_Subscription::FC_Subscription(){

    T_DjiReturnCode djiStat;
    T_DjiOsalHandler *osalHandler = NULL;

    osalHandler = DjiPlatform_GetOsalHandler();
    djiStat = DjiFcSubscription_Init();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("init data subscription module error.");
        throw ("Perception init failed");
    }
}

FC_Subscription::~FC_Subscription()
{
    T_DjiReturnCode djiStat;
    djiStat = DjiFcSubscription_DeInit();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Deinit fc subscription error.");
        perror("Perception deinit failed");
    }
}

T_DjiReturnCode FC_Subscription::SubscribeTopicQuaternian(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe)
{
    T_DjiReturnCode djiStat;

    if(subscribe_unsubscribe){
        djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
            Dji_FcSubscriptionReceiveQuaternionCallback);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Subscribe topic quaternion error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
    else{
        djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("UnSubscribe topic quaternion error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
}

T_DjiReturnCode FC_Subscription::SubscribeTopicAccelerationGround(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe)
{
    T_DjiReturnCode djiStat;

    if(subscribe_unsubscribe){
        djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_GROUND, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
            Dji_FcSubscriptionReceiveAccelerationGroundCallback);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Subscribe topic AccelerationGround error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
    else{
        djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_GROUND);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("UnSubscribe topic AccelerationGround error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
}


T_DjiReturnCode FC_Subscription::SubscribeTopicAccelerationBody(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe)
{
    T_DjiReturnCode djiStat;

    if(subscribe_unsubscribe){
        djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_BODY, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
            Dji_FcSubscriptionReceiveAccelerationBodyCallback);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Subscribe topic AccelerationBody error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
    else{
        djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_BODY);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("UnSubscribe topic AccelerationBody error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
}

T_DjiReturnCode FC_Subscription::SubscribeTopicAccelerationRaw(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe)
{
    T_DjiReturnCode djiStat;

    if(subscribe_unsubscribe){
        djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_RAW, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
            Dji_FcSubscriptionReceiveAccelerationRawCallback);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Subscribe topic AccelerationRaw error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
    else{
        djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_RAW);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("UnSubscribe topic AccelerationRaw error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
}

T_DjiReturnCode FC_Subscription::SubscribeTopicVelocity(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe)
{
    T_DjiReturnCode djiStat;

    if(subscribe_unsubscribe){
        djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
            Dji_FcSubscriptionReceiveVelocityCallback);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Subscribe topic Velocity error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
    else{
        djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("UnSubscribe topic Velocity error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
}

T_DjiReturnCode FC_Subscription::SubscribeTopicAngularRateFusioned(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe)
{
    T_DjiReturnCode djiStat;

    if(subscribe_unsubscribe){
        djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_FUSIONED, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
            Dji_FcSubscriptionReceiveAngularRateFusionedCallback);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Subscribe topic AngularRateFusioned error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
    else{
        djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_FUSIONED);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("UnSubscribe topic AngularRateFusioned error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
}

T_DjiReturnCode FC_Subscription::SubscribeTopicAngularRateRaw(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe)
{
    T_DjiReturnCode djiStat;

    if(subscribe_unsubscribe){
        djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_RAW, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
            Dji_FcSubscriptionReceiveAngularRateRawCallback);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Subscribe topic AngularRateRaw error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
    else{
        djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_RAW);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("UnSubscribe topic AngularRateRaw error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
}

T_DjiReturnCode FC_Subscription::SubscribeTopicAltitudeFused(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe)
{
    T_DjiReturnCode djiStat;

    if(subscribe_unsubscribe){
        djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
            Dji_FcSubscriptionReceiveAltitudeFusedCallback);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Subscribe topic AltitudeFused error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
    else{
        djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("UnSubscribe topic AltitudeFused error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
} 

T_DjiReturnCode FC_Subscription::SubscribeTopicAltitudeBarometer(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe)
{
    T_DjiReturnCode djiStat;

    if(subscribe_unsubscribe){
        djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_BAROMETER, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
            Dji_FcSubscriptionReceiveAltitudeBarometerCallback);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Subscribe topic AltitudeBarometer error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
    else{
        djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_BAROMETER);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("UnSubscribe topic AltitudeBarometer error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
}

T_DjiReturnCode FC_Subscription::SubscribeTopicAltitudeOfHomePoint(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe)
{
    T_DjiReturnCode djiStat;

    if(subscribe_unsubscribe){
        djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
            Dji_FcSubscriptionReceiveAltitudeOfHomePointCallback);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Subscribe topic AltitudeOfHomePoint error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
    else{
        djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("UnSubscribe topic AltitudeOfHomePoint error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
}

T_DjiReturnCode FC_Subscription::SubscribeTopicHeightFusion(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe)
{
    T_DjiReturnCode djiStat;

    if(subscribe_unsubscribe){
        djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
            Dji_FcSubscriptionReceiveHeightFusionCallback);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Subscribe topic HeightFusion error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
    else{
        djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("UnSubscribe topic HeightFusion error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
}

/*!
 * @brief Relative height above ground of aircraft topic name. Please refer to
 * ::T_DjiFcSubscriptionHeightRelative for information about data structure.
 * @details This data is a fusion output from aircraft. The height is a direct estimate of the closest large object
 * below the aircraft's ultrasonic sensors.
 * @warning This topic does not come with a 'valid' flag - so if the aircraft is too far from an object for the
 * ultrasonic sensors/VO to provide any meaningful data, the values will latch and there is no way for user to
 * determine if the data is valid or not. Please use with caution.
 * @datastruct \ref T_DjiFcSubscriptionHeightRelative
 */
T_DjiReturnCode FC_Subscription::SubscribeTopicHeightRelative(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe)
{
    T_DjiReturnCode djiStat;

    if(subscribe_unsubscribe){
        djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_RELATIVE, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
            Dji_FcSubscriptionReceiveHeightRelativeCallback);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Subscribe topic HeightRelative error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
    else{
        djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_RELATIVE);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("UnSubscribe topic HeightRelative error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
}

T_DjiReturnCode FC_Subscription::SubscribeTopicPositionFused(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe)
{
    T_DjiReturnCode djiStat;

    if(subscribe_unsubscribe){
        djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
            Dji_FcSubscriptionReceivePositionFusedCallback);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Subscribe topic PositionFused error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
    else{
        djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("UnSubscribe topic PositionFused error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
}

//T_DjiReturnCode SubscribeTopicGpsDate(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

//T_DjiReturnCode SubscribeTopicGpsTime(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

//T_DjiReturnCode SubscribeTopicGpsPosition(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

//T_DjiReturnCode SubscribeTopicGpsVelocity(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

//T_DjiReturnCode SubscribeTopicGpsDetails(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

//T_DjiReturnCode SubscribeTopicGpsSignalLevel(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

//T_DjiReturnCode SubscribeTopicRtkPosition(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

//T_DjiReturnCode SubscribeTopicRtkVelocity(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

//T_DjiReturnCode SubscribeTopicRtkYaw(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

//T_DjiReturnCode SubscribeTopicRtkPositionInfo(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

//T_DjiReturnCode SubscribeTopicRtkYawInfo(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

T_DjiReturnCode FC_Subscription::SubscribeTopicCompass(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe)
{
    T_DjiReturnCode djiStat;

    if(subscribe_unsubscribe){
        djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_COMPASS, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
            Dji_FcSubscriptionReceiveCompassCallback);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Subscribe topic Compass error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
    else{
        djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_COMPASS);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("UnSubscribe topic Compass error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
}

//T_DjiReturnCode SubscribeTopicRC(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

//T_DjiReturnCode SubscribeTopicGimbalAngles(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

//T_DjiReturnCode SubscribeTopicGimbalStatus(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

T_DjiReturnCode FC_Subscription::SubscribeTopicFlightStatus(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe)
{
    T_DjiReturnCode djiStat;

    if(subscribe_unsubscribe){
        djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
            Dji_FcSubscriptionReceiveFlightStatusCallback);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Subscribe topic FlightStatus error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
    else{
        djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("UnSubscribe topic FlightStatus error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
}

//T_DjiReturnCode SubscribeTopicDisplaymode(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

//T_DjiReturnCode SubscribeTopicLandinggear(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

//T_DjiReturnCode SubscribeTopicMotorStartError(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

//T_DjiReturnCode SubscribeTopicWholeBatteryInfo(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

//T_DjiReturnCode SubscribeTopicControlDevice(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

//T_DjiReturnCode SubscribeTopicHardSync(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

//T_DjiReturnCode SubscribeTopicGpsControlLevel(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

//T_DjiReturnCode SubscribeTopicRCWithFlagData(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

//T_DjiReturnCode SubscribeTopicEscData(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

//T_DjiReturnCode SubscribeTopicRTKConnectStatus(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

//T_DjiReturnCode SubscribeTopicGimbalControlMode(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

//T_DjiReturnCode SubscribeTopicFlightAnomaly(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

T_DjiReturnCode FC_Subscription::SubscribeTopicPositionVO(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe)
{
    T_DjiReturnCode djiStat;

    if(subscribe_unsubscribe){
        djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_VO, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
            Dji_FcSubscriptionReceivePositionVOCallback);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Subscribe topic PositionVO error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
    else{
        djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_VO);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("UnSubscribe topic PositionVO error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
}

//T_DjiReturnCode SubscribeTopicAvoidData(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

//(T_DjiReturnCode SubscribeTopicHomePointSetStatus(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

//T_DjiReturnCode SubscribeTopicHomePointInfo(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

//T_DjiReturnCode SubscribeTopicThreeGimbalData(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

//T_DjiReturnCode SubscribeTopicSingleBatteryInfoIndex1(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);
                                                                                  
//T_DjiReturnCode SubscribeTopicSingleBatteryInfoIndex2(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

/*!
 * @brief Please refer to ::T_DjiFcSubscriptionImuAttiNaviDataWithTimestamp for information about data structure.
 * @datastruct \ref T_DjiFcSubscriptionImuAttiNaviDataWithTimestamp
 */
T_DjiReturnCode FC_Subscription::SubscribeTopicImuAttiNaviDataWithTimestamp(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe)
{
    T_DjiReturnCode djiStat;

    if(subscribe_unsubscribe){
        djiStat = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_IMU_ATTI_NAVI_DATA_WITH_TIMESTAMP, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
            Dji_FcSubscriptionReceiveImuAttiNaviDataWithTimestampCallback);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Subscribe topic ImuAttiNaviDataWithTimestamp error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
    else{
        djiStat = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_IMU_ATTI_NAVI_DATA_WITH_TIMESTAMP);
        if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("UnSubscribe topic ImuAttiNaviDataWithTimestamp error.");
            return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
    }
}

//T_DjiReturnCode SubscribeTopicTotalNumber(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);


/* Private functions definition-----------------------------------------------*/

#ifndef __CC_ARM
#pragma GCC diagnostic pop
#endif

static T_DjiReturnCode Dji_FcSubscriptionReceiveQuaternionCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionQuaternion *quaternion = (T_DjiFcSubscriptionQuaternion *) data;
    fcSubscriptionData.Quaternion=(*quaternion);
    fcSubscriptionData.QuaternionTimestamp=(*timestamp);
    
    USER_UTIL_UNUSED(dataSize);
    
    // dji_f64_t pitch, yaw, roll;

    //pitch = (dji_f64_t) asinf(-2 * quaternion->q1 * quaternion->q3 + 2 * quaternion->q0 * quaternion->q2) * 57.3;
    //roll = (dji_f64_t) atan2f(2 * quaternion->q2 * quaternion->q3 + 2 * quaternion->q0 * quaternion->q1,
    //                         -2 * quaternion->q1 * quaternion->q1 - 2 * quaternion->q2 * quaternion->q2 + 1) * 57.3;
    //yaw = (dji_f64_t) atan2f(2 * quaternion->q1 * quaternion->q2 + 2 * quaternion->q0 * quaternion->q3,
    //                         -2 * quaternion->q2 * quaternion->q2 - 2 * quaternion->q3 * quaternion->q3 + 1) *
    //      57.3;

    //if (s_userFcSubscriptionDataShow == true) {
    //    if (s_userFcSubscriptionDataCnt++ % DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ == 0) {
    //        USER_LOG_INFO("receive quaternion data.");
    //        USER_LOG_INFO("timestamp: millisecond %u microsecond %u.", timestamp->millisecond,
    //                      timestamp->microsecond);
    //        USER_LOG_INFO("quaternion: %f %f %f %f.", quaternion->q0, quaternion->q1, quaternion->q2,
    //                      quaternion->q3);

    //        USER_LOG_INFO("euler angles: pitch = %.2f roll = %.2f yaw = %.2f.\r\n", pitch, roll, yaw);
    //        DjiTest_WidgetLogAppend("pitch = %.2f roll = %.2f yaw = %.2f.", pitch, roll, yaw);
    //    }
    //}
     
    return DjiErrorCode::DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode Dji_FcSubscriptionReceiveAccelerationGroundCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionAccelerationGround *AccelerationGround = (T_DjiFcSubscriptionAccelerationGround *) data;
    fcSubscriptionData.AccelerationGround=(*AccelerationGround);
    fcSubscriptionData.AccelerationGroundTimestamp=(*timestamp);
    
    USER_UTIL_UNUSED(dataSize);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode Dji_FcSubscriptionReceiveAccelerationBodyCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionAccelerationBody *AccelerationBody = (T_DjiFcSubscriptionAccelerationBody *) data;
    fcSubscriptionData.AccelerationBody=(*AccelerationBody);
    fcSubscriptionData.AccelerationBodyTimestamp=(*timestamp);
    
    USER_UTIL_UNUSED(dataSize);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode Dji_FcSubscriptionReceiveAccelerationRawCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionAccelerationRaw *AccelerationRaw = (T_DjiFcSubscriptionAccelerationRaw *) data;
    fcSubscriptionData.AccelerationRaw=(*AccelerationRaw);
    fcSubscriptionData.AccelerationRawTimestamp=(*timestamp);
    
    USER_UTIL_UNUSED(dataSize);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
static T_DjiReturnCode Dji_FcSubscriptionReceiveVelocityCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionVelocity *Velocity = (T_DjiFcSubscriptionVelocity *) data;
    fcSubscriptionData.Velocity=(*Velocity);
    fcSubscriptionData.VelocityTimestamp=(*timestamp);
    
    USER_UTIL_UNUSED(dataSize);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
static T_DjiReturnCode Dji_FcSubscriptionReceiveAngularRateFusionedCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionAngularRateFusioned *AngularRateFusioned = (T_DjiFcSubscriptionAngularRateFusioned *) data;
    fcSubscriptionData.AngularRateFusioned=(*AngularRateFusioned);
    fcSubscriptionData.AngularRateFusionedTimestamp=(*timestamp);
    
    USER_UTIL_UNUSED(dataSize);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
static T_DjiReturnCode Dji_FcSubscriptionReceiveAngularRateRawCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionAngularRateRaw *AngularRateRaw = (T_DjiFcSubscriptionAngularRateRaw *) data;
    fcSubscriptionData.AngularRateRaw=(*AngularRateRaw);
    fcSubscriptionData.AngularRateRawTimestamp=(*timestamp);
    
    USER_UTIL_UNUSED(dataSize);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
static T_DjiReturnCode Dji_FcSubscriptionReceiveAltitudeFusedCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionAltitudeFused *AltitudeFused = (T_DjiFcSubscriptionAltitudeFused *) data;
    fcSubscriptionData.AltitudeFused=(*AltitudeFused);
    fcSubscriptionData.AltitudeFusedTimestamp=(*timestamp);
    
    USER_UTIL_UNUSED(dataSize);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
static T_DjiReturnCode Dji_FcSubscriptionReceiveAltitudeBarometerCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionAltitudeBarometer *AltitudeBarometer = (T_DjiFcSubscriptionAltitudeBarometer *) data;
    fcSubscriptionData.AltitudeBarometer=(*AltitudeBarometer);
    fcSubscriptionData.AltitudeBarometerTimestamp=(*timestamp);
    
    USER_UTIL_UNUSED(dataSize);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
static T_DjiReturnCode Dji_FcSubscriptionReceiveAltitudeOfHomePointCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionAltitudeOfHomePoint *AltitudeOfHomePoint = (T_DjiFcSubscriptionAltitudeOfHomePoint *) data;
    fcSubscriptionData.AltitudeOfHomePoint=(*AltitudeOfHomePoint);
    fcSubscriptionData.AltitudeOfHomePointTimestamp=(*timestamp);
    
    USER_UTIL_UNUSED(dataSize);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
static T_DjiReturnCode Dji_FcSubscriptionReceiveHeightFusionCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionHeightFusion *HeightFusion = (T_DjiFcSubscriptionHeightFusion *) data;
    fcSubscriptionData.HeightFusion=(*HeightFusion);
    fcSubscriptionData.HeightFusionTimestamp=(*timestamp);
    
    USER_UTIL_UNUSED(dataSize);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
static T_DjiReturnCode Dji_FcSubscriptionReceiveHeightRelativeCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionHeightRelative *HeightRelative = (T_DjiFcSubscriptionHeightRelative *) data;
    fcSubscriptionData.HeightRelative=(*HeightRelative);
    fcSubscriptionData.HeightRelativeTimestamp=(*timestamp);
    
    USER_UTIL_UNUSED(dataSize);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
static T_DjiReturnCode Dji_FcSubscriptionReceivePositionFusedCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionPositionFused *PositionFused = (T_DjiFcSubscriptionPositionFused *) data;
    fcSubscriptionData.PositionFused=(*PositionFused);
    fcSubscriptionData.PositionFusedTimestamp=(*timestamp);
    
    USER_UTIL_UNUSED(dataSize);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
static T_DjiReturnCode Dji_FcSubscriptionReceiveCompassCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionCompass *Compass = (T_DjiFcSubscriptionCompass *) data;
    fcSubscriptionData.Compass=(*Compass);
    fcSubscriptionData.CompassTimestamp=(*timestamp);
    
    USER_UTIL_UNUSED(dataSize);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
static T_DjiReturnCode Dji_FcSubscriptionReceiveFlightStatusCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionFlightStatus *FlightStatus = (T_DjiFcSubscriptionFlightStatus *) data;
    fcSubscriptionData.FlightStatus=(*FlightStatus);
    fcSubscriptionData.FlightStatusTimestamp=(*timestamp);
    
    USER_UTIL_UNUSED(dataSize);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
static T_DjiReturnCode Dji_FcSubscriptionReceivePositionVOCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionPositionVO *PositionVO = (T_DjiFcSubscriptionPositionVO *) data;
    fcSubscriptionData.PositionVO=(*PositionVO);
    fcSubscriptionData.PositionVOTimestamp=(*timestamp);
    
    USER_UTIL_UNUSED(dataSize);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
static T_DjiReturnCode Dji_FcSubscriptionReceiveImuAttiNaviDataWithTimestampCallback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    T_DjiFcSubscriptionImuAttiNaviDataWithTimestamp *ImuAttiNaviDataWithTimestamp = (T_DjiFcSubscriptionImuAttiNaviDataWithTimestamp *) data;
    fcSubscriptionData.ImuAttiNaviDataWithTimestamp=(*ImuAttiNaviDataWithTimestamp);
    fcSubscriptionData.ImuAttiNaviDataWithTimestampTimestamp=(*timestamp);
    
    USER_UTIL_UNUSED(dataSize);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}


/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/



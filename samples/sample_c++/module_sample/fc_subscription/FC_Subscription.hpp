/**
 ********************************************************************
 * @file    FC_Subscription.hpp
 * @brief   This is the header file for "FC_Subscription.hpp", defining the structure and
 * (exported) function prototypes.
 *
 * @author Metin Ozkan (meozkan@gmail.com)
 *

 *
 *********************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef FC_SUBSCRIPTION_H
#define FC_SUBSCRIPTION_H

/* Includes ------------------------------------------------------------------*/
#include "dji_typedef.h"
#include "dji_fc_subscription.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/
typedef struct FC_DATA {
  ///dji_f32_t q0; /*!< w, rad (when converted to a rotation matrix or Euler angles). */
  ///dji_f32_t q1; /*!< x, rad (when converted to a rotation matrix or Euler angles). */
  ///dji_f32_t q2; /*!< y, rad (when converted to a rotation matrix or Euler angles). */
  ///dji_f32_t q3; /*!< z, rad (when converted to a rotation matrix or Euler angles). */
  T_DjiFcSubscriptionQuaternion Quaternion;
  ///uint32_t millisecond;
  ///uint32_t microsecond;
  T_DjiDataTimestamp QuaternionTimestamp;
  ///T_DjiVector3f
  T_DjiFcSubscriptionAccelerationGround AccelerationGround;
  T_DjiDataTimestamp AccelerationGroundTimestamp;
  ///T_DjiVector3f
  T_DjiFcSubscriptionAccelerationBody AccelerationBody;
  T_DjiDataTimestamp AccelerationBodyTimestamp;
  ///T_DjiVector3f
  T_DjiFcSubscriptionAccelerationRaw AccelerationRaw;
  T_DjiDataTimestamp AccelerationRawTimestamp;
  ////*! Velocity of aircraft. */
  ///T_DjiVector3f data;
  ////*! Health state of aircraft velocity data. It can be any value of ::E_DjiFcSubscriptionDataHealthFlag. */
  ///uint8_t health: 1;
  ////*! Reserved. */
  ///uint8_t reserve: 7;
  T_DjiFcSubscriptionVelocity Velocity;
  T_DjiDataTimestamp VelocityTimestamp;
  ///T_DjiVector3f
  T_DjiFcSubscriptionAngularRateFusioned AngularRateFusioned;
  T_DjiDataTimestamp AngularRateFusionedTimestamp;
  ///T_DjiVector3f
  T_DjiFcSubscriptionAngularRateRaw AngularRateRaw;
  T_DjiDataTimestamp AngularRateRawTimestamp;
  ///dji_f32_t
  T_DjiFcSubscriptionAltitudeFused AltitudeFused;
  T_DjiDataTimestamp AltitudeFusedTimestamp;
  ///dji_f32_t
  T_DjiFcSubscriptionAltitudeBarometer AltitudeBarometer;
  T_DjiDataTimestamp AltitudeBarometerTimestamp;
  ///dji_f32_t
  T_DjiFcSubscriptionAltitudeOfHomePoint AltitudeOfHomePoint;
  T_DjiDataTimestamp AltitudeOfHomePointTimestamp;
  ///dji_f32_t
  T_DjiFcSubscriptionHeightFusion HeightFusion;
  T_DjiDataTimestamp HeightFusionTimestamp;
  ///dji_f32_t
  T_DjiFcSubscriptionHeightRelative HeightRelative;
  T_DjiDataTimestamp HeightRelativeTimestamp;
  ///dji_f64_t longitude; /*!< Longitude, unit: rad. */
  ///dji_f64_t latitude; /*!< Latitude, unit: rad. */
  ///dji_f32_t altitude; /*!< Altitude, WGS 84 reference ellipsoid, unit: m. */
  ///uint16_t visibleSatelliteNumber; /*!< Number of visible satellites. */
  T_DjiFcSubscriptionPositionFused PositionFused;
  T_DjiDataTimestamp PositionFusedTimestamp;
  ///int16_t x;
  ///int16_t y;
  ///int16_t z;
  T_DjiFcSubscriptionCompass Compass;
  T_DjiDataTimestamp CompassTimestamp;
  ///Flight status information topic data structure. 
  ///It can be any value of ::E_DjiFcSubscriptionFlightStatus.
  T_DjiFcSubscriptionFlightStatus FlightStatus;
  T_DjiDataTimestamp FlightStatusTimestamp;
  ///dji_f32_t x;              /*!< North (best effort), unit: m */
  ///dji_f32_t y;              /*!< East (best effort),  unit: m */
  ///dji_f32_t z;              /*!< Down,  unit: m */
  ///uint8_t xHealth: 1;
  ///uint8_t yHealth: 1;
  ///uint8_t zHealth: 1;
  ///uint8_t reserved: 5;
  T_DjiFcSubscriptionPositionVO PositionVO;
  T_DjiDataTimestamp PositionVOTimestamp;
  ///uint16_t version;
  ///uint16_t flag;
  ///dji_f32_t pn_x;
  ///dji_f32_t pn_y;
  ///dji_f32_t pn_z;
  ///dji_f32_t vn_x;
  ///dji_f32_t vn_y;
  ///dji_f32_t vn_z;
  ///dji_f32_t an_x;
  ///dji_f32_t an_y;
  ///dji_f32_t an_z;
  ///dji_f32_t q[4];
  ///uint16_t resv;
  ///uint16_t cnt;
  ///uint32_t timestamp;
  T_DjiFcSubscriptionImuAttiNaviDataWithTimestamp ImuAttiNaviDataWithTimestamp;
  T_DjiDataTimestamp ImuAttiNaviDataWithTimestampTimestamp;
} FC_SubscriptionData;


using namespace std;

class FC_Subscription {
public:
    FC_Subscription();
    ~FC_Subscription();

    

    FC_SubscriptionData getSubscriptionData();


     /*!
     * @brief Quaternion of aircraft topic name. Quaternion topic provides aircraft body frame (FRD) to ground frame
     * (NED) rotation. Please refer to ::T_DjiFcSubscriptionQuaternion for information about data structure.
     * @details The DJI quaternion follows Hamilton convention (q0 = w, q1 = x, q2 = y, q3 = z).
     * | Angle        | Unit | Accuracy   | Notes                                           |
       |--------------|------|------------|-------------------------------------------------|
       | pitch, roll  | deg  | <1         | in NON-AHRS mode                                |
       | yaw          | deg  | <3         | in well-calibrated compass with fine aligned    |
       | yaw with rtk | deg  | around 1.2 | in RTK heading fixed mode with 1 meter baseline |
     * @datastruct \ref T_DjiFcSubscriptionQuaternion
     */
    T_DjiReturnCode SubscribeTopicQuaternian(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);
    
     /*!
     * @brief Provides aircraft's acceleration w.r.t a ground-fixed \b NEU frame @ up to 200Hz
     * @warning Please note that this data is not in a conventional right-handed frame of reference.
     * @details This is a fusion output from the flight control system. The output is in a right-handed NED frame, but the
     * sign of the Z-axis acceleration is flipped before publishing to this topic. So if you are looking to get acceleration
     * in an NED frame, simply flip the sign of the z-axis value. Beyond that, you can convert using rotations to
     * any right-handed frame of reference.
     * @units m/s<SUP>2</SUP>
     * @datastruct \ref T_DjiFcSubscriptionAccelerationGround
     */
    T_DjiReturnCode SubscribeTopicAccelerationGround(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief Provides aircraft's acceleration w.r.t a body-fixed \b FRU frame @ up to 200Hz
     * @warning Please note that this data is not in a conventional right-handed frame of reference.
     * @details This is a fusion output from the flight control system.
     * @units m/s<SUP>2</SUP>
     * @datastruct \ref T_DjiFcSubscriptionAccelerationBody
     */
    T_DjiReturnCode SubscribeTopicAccelerationBody(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

     /*!
     * @brief Provides aircraft's acceleration in an IMU-centered, body-fixed \b FRD frame @ up to 400Hz
     * @details This is a filtered output from the IMU on board the flight control system.
     * @sensors IMU
     * @units m/s<SUP>2</SUP>
     * @datastruct \ref T_DjiFcSubscriptionAccelerationRaw
     */
    T_DjiReturnCode SubscribeTopicAccelerationRaw(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief Velocity of aircraft topic name. Velocity topic provides aircraft's velocity in a ground-fixed NEU frame.
     * Please refer to ::T_DjiFcSubscriptionVelocity for information about data structure.
     * @warning Please note that this data is not in a conventional right-handed frame of reference.
     * @details This velocity data is a fusion output from the aircraft. Original output is in a right-handed NED frame, but the
     * sign of the Z-axis velocity is flipped before publishing to this topic. So if you are looking to get velocity
     * in an NED frame, simply flip the sign of the z-axis value. Beyond that, you can convert using rotations to
     * any right-handed frame of reference.
     * | Axis     | Unit | Accuracy                                                                                    |
       |----------|------|---------------------------------------------------------------------------------------------|
       | vgx, vgy | m/s  | Around 5cm/s for GNSS navigation. Around 3cm/s with VO at 1 meter height                    |
       | vgz      | m/s  | 10cm/s only with barometer in steady air. 3cm/s with VO at 1 meter height with 8cm baseline |
     * @datastruct \ref T_DjiFcSubscriptionVelocity
     */
    T_DjiReturnCode SubscribeTopicVelocity(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief Provides aircraft's angular velocity in a ground-fixed \b NED frame @ up to 200Hz
     * @details This is a fusion output from the flight control system.
     * @units rad/s
     * @datastruct \ref T_DjiFcSubscriptionAngularRateFusioned
     */
    T_DjiReturnCode SubscribeTopicAngularRateFusioned(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief Provides aircraft's angular velocity in an IMU-centered, body-fixed \b FRD frame @ up to 400Hz
     * @details This is a filtered output from the IMU on board the flight control system.
     * @sensors IMU
     * @units rad/s
     * @datastruct \ref T_DjiFcSubscriptionAngularRateRaw
     */
    T_DjiReturnCode SubscribeTopicAngularRateRaw(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);
   
    /*!
     * @brief Fused altitude of aircraft topic name. Fused altitude topic provides aircraft's fused altitude from sea
     * level. Please refer to ::T_DjiFcSubscriptionAltitudeFused for information about data structure.
     * @units m
     * @datastruct \ref T_DjiFcSubscriptionAltitudeFused
     */
    T_DjiReturnCode SubscribeTopicAltitudeFused(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true); 

    /*!
     * @brief Provides aircraft's pressure altitude from sea level using the ICAO model @ up to 200Hz
     * @details
     * This is a filetered output from the barometer without any further fusion.
     *
     * The ICAO model gives an MSL altitude of 1013.25mBar at 15&deg; C and a temperature lapse rate of -6.5&deg; C
     * per 1000m. In your case, it may be possible that the take off altitude of the aircraft is recording a higher pressure
     * than 1013.25mBar. Let's take an example - a weather station shows that SFO (San Francisco International Airport) had
     * recently recorded a pressure of 1027.1mBar. SFO is 4m above MSL, yet, if you calculate the Pressure Altitude using
     * the ICAO model, it relates to -114m. You can use an online calculator to similarly calculate the Pressure Altitude
     * in your area.
     *
     * Another factor that may affect your altitude reading is manufacturing differences in the barometer - it is not
     * uncommon to have a variation of &plusmn;30m readings at the same physical location with two different aircraft. For a given
     * aircraft, these readings will be consistent, so you will need to calibrate the offset of your system if your code
     * relies on the accuracy of the absolute value of altitude.
     * @sensors GPS, Barometer, IMU
     * @units m
     * @datastruct \ref T_DjiFcSubscriptionAltitudeBarometer
     */
    T_DjiReturnCode SubscribeTopicAltitudeBarometer(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);
    
    /*!
     * @brief Provides the altitude from sea level when the aircraft last took off.
     * @details
     * This is a fusion output from the flight control system, and also uses the ICAO model.
     *
     * The ICAO model gives an MSL altitude of 1013.25mBar at 15&deg; C and a temperature lapse rate of -6.5&deg; C
     * per 1000m. In your case, it may be possible that the take off altitude of the aircraft is recording a higher pressure
     * than 1013.25mBar. Let's take an example - a weather station shows that SFO (San Francisco International Airport) had
     * recently recorded a pressure of 1027.1mBar. SFO is 4m above MSL, yet, if you calculate the Pressure Altitude using
     * the ICAO model, it relates to -114m. You can use an online calculator to similarly calculate the Pressure Altitude
     * in your area.
     *
     * Another factor that may affect your altitude reading is manufacturing differences in the barometer - it is not
     * uncommon to have a variation of &plusmn;30m readings at the same physical location with two different aircraft. For a given
     * aircraft, these readings will be consistent, so you will need to calibrate the offset of your system if your code
     * relies on the accuracy of the absolute value of altitude.
     *
     * @note This value is updated each time the drone takes off.
     *
     * @units m
     * @datastruct \ref T_DjiFcSubscriptionAltitudeOfHomePoint
     */
    T_DjiReturnCode SubscribeTopicAltitudeOfHomePoint(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief Provides the relative height above ground at up to 100Hz.
     * @details
     * This is a fusion output from the flight control system. The height is a direct estimate of the closest large object below the aircraft's ultrasonic sensors.
     * A large object is something that covers the ultrasonic sensor for an extended duration of time.
     *
     * @warning This topic does not come with a 'valid' flag - so if the aircraft is too far from an object for the
     * ultrasonic sensors/VO to provide any meaningful data, the values will latch and there is no way for user code to
     * determine if the data is valid or not. Use with caution.
     * @sensors Visual Odometry, Ultrasonic
     * @units m
     * @datastruct \ref T_DjiFcSubscriptionHeightFusion
     */
    T_DjiReturnCode SubscribeTopicHeightFusion(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

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
    T_DjiReturnCode SubscribeTopicHeightRelative(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * Fused position of aircraft topic name. Please refer to ::T_DjiFcSubscriptionPositionFused for information
     * about data structure.
     * @warning Please note that if GPS signal is weak (low visibleSatelliteNumber, see below), the
     * latitude/longitude values won't be updated but the altitude might still be. There is currently no way to know if
     * the lat/lon update is healthy.
     * @details The most important component of this topic is the T_DjiFcSubscriptionPositionFused::visibleSatelliteNumber.
     * Use this to track your GPS satellite coverage and build some heuristics for when you might expect to lose GPS updates.
     *   | Axis | Unit | Position Sensor | Accuracy                                         |
         |------|------|-----------------|--------------------------------------------------|
         | x, y | m    | GPS             | <3m with open sky without multipath              |
         | z    | m    | GPS             | <5m with open sky without multipath              |
     * @datastruct \ref T_DjiFcSubscriptionPositionFused
     */
    T_DjiReturnCode SubscribeTopicPositionFused(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief GPS date topic name. Please refer to ::T_DjiFcSubscriptionGpsDate for information about data structure.
     * @datastruct \ref T_DjiFcSubscriptionGpsDate
     */
    T_DjiReturnCode SubscribeTopicGpsDate(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief GPS time topic name. Please refer to ::T_DjiFcSubscriptionGpsTime for information about data structure.
     * @datastruct \ref T_DjiFcSubscriptionGpsTime
     */
    T_DjiReturnCode SubscribeTopicGpsTime(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief GPS position topic name. Please refer to ::T_DjiFcSubscriptionGpsPosition for information about data structure.
     * @details
     *   | Axis | Accuracy                                         |
         |------|--------------------------------------------------|
         | x, y | <3m with open sky without multipath              |
         | z    | <5m with open sky without multipath              |
     * @datastruct \ref T_DjiFcSubscriptionGpsPosition
     */
    T_DjiReturnCode SubscribeTopicGpsPosition(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief GPS velocity topic name. Please refer to ::T_DjiFcSubscriptionGpsVelocity for information about data structure.
     * @datastruct \ref T_DjiFcSubscriptionGpsVelocity
     */
    T_DjiReturnCode SubscribeTopicGpsVelocity(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief GPS details topic name. GPS details topic provides GPS state and other detail information. Please refer
     * to ::T_DjiFcSubscriptionGpsDetail for information about data structure.
     * @datastruct \ref T_DjiFcSubscriptionGpsDetails
     */
    T_DjiReturnCode SubscribeTopicGpsDetails(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief GPS signal level topic name. This topic provides a measure of the quality of GPS signal. Please refer to
     * ::T_DjiFcSubscriptionGpsSignalLevel for information about data structure.
     * @datastruct \ref T_DjiFcSubscriptionGpsSignalLevel
     */
    T_DjiReturnCode SubscribeTopicGpsSignalLevel(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief RTK position topic name. Please refer to ::T_DjiFcSubscriptionRtkPosition for information about data structure.
     * @details
     *   | Axis | Accuracy                                         |
         |------|--------------------------------------------------|
         | x, y | ~2cm with fine alignment and fix condition       |
         | z    | ~3cm with fine alignment and fix condition       |
     * @datastruct \ref T_DjiFcSubscriptionRtkPosition
     */
    T_DjiReturnCode SubscribeTopicRtkPosition(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief RTK velocity topic name. Please refer to ::T_DjiFcSubscriptionRtkVelocity for information about data structure.
     * @datastruct \ref T_DjiFcSubscriptionRtkVelocity
     */
    T_DjiReturnCode SubscribeTopicRtkVelocity(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief RTK yaw topic name. Please refer to ::T_DjiFcSubscriptionRtkYaw for information about data structure.
     * @details The RTK yaw will provide the vector from ANT1 to ANT2 as configured in DJI Assistant 2. This
     * means that the value of RTK yaw will be 90deg offset from the yaw of the aircraft.
     * @datastruct \ref T_DjiFcSubscriptionRtkYaw
     */
    T_DjiReturnCode SubscribeTopicRtkYaw(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief RTK position information topic name. RTK position information topic provides a state of RTK position
     * solution. Please refer to ::T_DjiFcSubscriptionRtkPositionInfo for information about data structure.
     * @datastruct \ref T_DjiFcSubscriptionRtkPositionInfo
     */
    T_DjiReturnCode SubscribeTopicRtkPositionInfo(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief RTK yaw topic name. RTK yaw information topic provides a state of RTK yaw solution. Please refer to
     * ::T_DjiFcSubscriptionRtkYawInfo for information about data structure.
     * @datastruct \ref T_DjiFcSubscriptionRtkYawInfo
     */
    T_DjiReturnCode SubscribeTopicRtkYawInfo(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief Provides aircraft's magnetometer reading, fused with IMU and GPS @ up to 100Hz
     * @details This reading is the magnetic field recorded by the magnetometer in x,y,z axis, calibrated such that
     * 1000 < |m| < 2000, and fused with IMU and GPS for robustness
     * @sensors Magnetometer, IMU, GPS
     * @units N/A
     * @datastruct \ref T_DjiFcSubscriptionCompass
     */
    T_DjiReturnCode SubscribeTopicCompass(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief Provides remote controller stick inputs @ up to 100Hz
     * @details This topic will give you:
     * - Stick inputs (R,P,Y,Thr)
     * - Mode switch (P/A/F)
     * - Landing gear switch (Up/Down)
     *
     * @datastruct \ref T_DjiFcSubscriptionRC
     * @also \ref TOPIC_RC_WITH_FLAG_DATA
     */
    T_DjiReturnCode SubscribeTopicRC(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief Provides gimbal pitch, roll, yaw @ up to 50Hz
     * @details
     * The reference frame for gimbal angles is a NED frame attached to the gimbal.
     * This topic uses a data structure, Vector3f, that is too generic for the topic. The order of angles is :
     * |Data Structure Element| Meaning|
     * |----------------------|--------|
     * |Vector3f.x            |pitch   |
     * |Vector3f.y            |roll    |
     * |Vector3f.z            |yaw     |
     *
     * @perf
     * 0.1 deg accuracy in all axes
     *
     * @sensors Gimbal Encoder, IMU, Magnetometer
     * @units deg
     * @datastruct \ref T_DjiFcSubscriptionGimbalAngles
     * @also \ref TOPIC_GIMBAL_STATUS, \ref TOPIC_GIMBAL_CONTROL_MODE
     */
    T_DjiReturnCode SubscribeTopicGimbalAngles(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief Provides gimbal status and error codes @ up to 50Hz
     * @details Please see the \ref GimbalStatus struct for the details on what data you can receive.
     *
     * @datastruct \ref T_DjiFcSubscriptionGimbalStatus
     * @also \ref TOPIC_GIMBAL_ANGLES, \ref TOPIC_GIMBAL_CONTROL_MODE
     */
    T_DjiReturnCode SubscribeTopicGimbalStatus(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief Flight status topic name. Please refer to ::T_DjiFcSubscriptionFlightStatus for information about data structure.
     * @datastruct \ref T_DjiFcSubscriptionFlightStatus
     */
    T_DjiReturnCode SubscribeTopicFlightStatus(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief Provides a granular state representation for various tasks/flight modes @ up to 50Hz
     * @details Typically, use this topic together with \ref TOPIC_STATUS_FLIGHT to get a
     * better understanding of the overall status of the aircraft.
     *
     * @datastruct \ref T_DjiFcSubscriptionDisplaymode
     * @also \ref TOPIC_STATUS_FLIGHT
     */
    T_DjiReturnCode SubscribeTopicDisplaymode(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief Provides status for the landing gear state @ up to 50Hz
     *
     * @datastruct \ref T_DjiFcSubscriptionLandinggear
     */
    T_DjiReturnCode SubscribeTopicLandinggear(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief If motors failed to start, this topic provides reasons why. Available @ up to 50Hz
     * @datastruct \ref T_DjiFcSubscriptionMotorStartError
     * \note These enumerations show up in the ErrorCode class because they can also be returned as acknowledgements
     * for APIs that start the motors, such as \ref Control::takeoff "Takeoff" or \ref Control::armMotors "Arm"
     */
    T_DjiReturnCode SubscribeTopicMotorStartError(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief Battery information topic name. Please refer to ::T_DjiFcSubscriptionWholeBatteryInfo for information about data structure.
     * @datastruct \ref T_DjiFcSubscriptionWholeBatteryInfo
     */
    T_DjiReturnCode SubscribeTopicWholeBatteryInfo(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief Provides states of the aircraft related to SDK/RC control
     * @details The following information is available in this topic:
     * |Data Structure Element| Meaning|
     * |----------------------|--------|
     * |controlMode           |The modes in which the aircraft is being controlled (control loops being applied on horizontal, vertical and yaw axes of the aircraft)|
     * |deviceStatus          |Which device is controlling the motion of the aircraft: RC (Manual control), MSDK (Missions kicked off through mobile), OSDK (Missions kicked off through onboard/ low-level flight control)    |
     * |flightStatus          |Has the OSDK been granted control authority? Since MSDK and RC have precedence, it is possible that deviceStatus shows RC or MSDK actually controlling the aircraft but this value is 1.     |
     * |vrcStatus             |Deprecated|
     * @datastruct \ref T_DjiFcSubscriptionControlDevice
     */
    T_DjiReturnCode SubscribeTopicControlDevice(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief Provides IMU and quaternion data time-synced with a hardware clock signal @ up to 400Hz.
     * @details This is the only data which can be synchronized with external software or hardware systems. If you want to
     * fuse an external sensor's data with the aircraft's IMU, this data along with a hardware trigger from the A3/N3's
     * expansion ports is how you would do it. You can see detailed documentation on how this process works in the [Hardware
     * Sync Guide](https://developer.dji.com/onboard-sdk/documentation/guides/component-guide-hardware-sync.html).
     * @sensors IMU, sensor fusion output
     * @units
     * |Data Structure Element| Units|
     * |----------------------|--------|
     * |Timestamp |2.5ms, 1ns (See \ref SyncTimestamp)|
     * |Quaternion |rad (after converting to rotation matrix)|
     * |Acceleration |g|
     * |Gyroscope |rad/sec|
     * @datastruct \ref T_DjiFcSubscriptionHardSync
     */
    T_DjiReturnCode SubscribeTopicHardSync(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief Provides a measure of the quality of GPS signal, with a mechanism for guarding against unset homepoint @ up to 50Hz
     * @details The level varies from 0 to 5, with 0 being the worst and 5 the best GPS signal. The key difference between
     * this and TOPIC_GPS_SIGNAL_LEVEL is that this topic always returns 0 if the homepoint is not set. Once the home point is
     * set, the behavior is exactly the same as TOPIC_GPS_SIGNAL_LEVEL.
     * @sensors GPS
     * @datastruct \ref T_DjiFcSubscriptionGpsControlLevel
     * @also \ref TOPIC_GPS_SIGNAL_LEVEL
     */
    T_DjiReturnCode SubscribeTopicGpsControlLevel(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief Provides normalized remote controller stick input data, along with connection status @ up to 50Hz
     * @note This topic was added in August 2018. Your aircraft may require a FW update to enable this feature.
     * @details This topic will give you:
     * - Stick inputs (R,P,Y,Thr)
     * - Mode switch (P/A/F)
     * - Landing gear switch (Up/Down)
     * - Connection status for air system, ground system and MSDK apps. The connection status also includes a
     * logicConnected element, which will change to false if either the air system or the ground system radios
     * are disconnected for >3s.
     * - Deadzones near the center of the stick positions are also handled in this topic.
     *
     * @datastruct \ref T_DjiFcSubscriptionRCWithFlagData
     * @also \ref TOPIC_RC
     */
    T_DjiReturnCode SubscribeTopicRCWithFlagData(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief Provides raw data from the ESCs @ up to 50Hz
     * @note This topic was added in August 2018. Your aircraft may require a FW update to enable this feature.
     * @details This topic supports reporting data for up to 8 ESCs; note that only DJI Intelligent ESCs are supported
     * for this reporting feature. Use this topic to get data on elements close to the hardware - e.g. motor speeds,
     * ESC current and voltage, error flags at the ESC level etc.
     * @datastruct \ref T_DjiFcSubscriptionEscData
     */
    T_DjiReturnCode SubscribeTopicEscData(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief Provides RTK connection status @ up to 50Hz
     * @note This topic was added in August 2018. Your aircraft may require a FW update to enable this feature.
     * @details This topic will update in real time whether the RTK GPS system is connected or not; typical uses
     * include app-level logic to switch between GPS and RTK sources of positioning based on this flag.
     * @datastruct \ref T_DjiFcSubscriptionRTKConnectStatus
     */
    T_DjiReturnCode SubscribeTopicRTKConnectStatus(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief Provides the mode in which the gimbal will interpret control commands @ up to 50Hz
     * @note This topic was added in August 2018. Your aircraft may require a FW update to enable this feature.
     * @details This topic will report the current control mode which can be set in the
     * DJI Go app, MSDK apps, or through Onboard SDK gimbal control APIs (see \ref Gimbal::AngleData "AngleData" struct
     * for more information)
     * @datastruct \ref T_DjiFcSubscriptionGimbalControlMode
     */
    T_DjiReturnCode SubscribeTopicGimbalControlMode(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief Provides a number of flags which report different errors the aircraft may encounter in flight @ up to 50Hz
     * @note This topic was added in August 2018. Your aircraft may require a FW update to enable this feature.
     * @warning Most of the errors reported by this topic are cases where immediate action is required; you can use these
     * as a baseline for implementing safety-related error-handling routines.
     * @datastruct \ref T_DjiFcSubscriptionFlightAnomaly
     */
    T_DjiReturnCode SubscribeTopicFlightAnomaly(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief Provides aircraft's position in a Cartesian frame @ up to 50Hz, without the need for GPS
     * @warning This topic does not follow a standard co-ordinate convention. Please read the details below for usage.
     * @details This is the only topic which can provide positioning information without having a GPS fix; though this
     * can be a big enabler please note the caveats of using this topic:
     * - The topic will use an origin that does not have a global reference, and is not published to the SDK.
     * - The topic uses a combination of VO and compass heading to identify the X-Y axes of its reference frame. This means
     * that if your compass performance is not good in an environment, there is no guarantee the X-Y axes will point to
     * North and East.
     * - The actual directions of the X-Y axes are currently not published to the SDK.
     * - If during a flight the compass performance were to change dramatically, the orientation of the X-Y axes may change
     * to re-align with North-East. The aircraft's position in X and Y may exhibit discontinuities in these cases.
     * - The reference frame is referred to as the Navigation Frame - Cartesian X,Y axes aligned with N,E directions on a best-effort
     * basis, and Z aligned to D (down) direction.
     * - A health flag for each axis provides some granularity on whether this data is valid or not.
     *
     * The key takeaway from these details is that this topic provides a best-effort attempt at providing position
     * information in the absence of absolute references (GPS, compass etc.), without guarantees of consistency if
     * environmental conditions change. So if your application is confined to a stable environment, or if you will
     * have GPS and compass available at all times, this topic can still provide useful data that cannot be otherwise
     * had. If using for control, make sure to have guards checking for the continuity of data.
     *
     * @note Since this topic relies on visual features and/or GPS, if your environment does not provide any of these
     * sources of data, the quality of this topic will reduce significantly. VO data quality will reduce if you are too high
     * above the ground. Make sure that the Vision Positioning System is enabled in DJI Go 4 before using this topic
     * (by default it is enabled).
     * @sensors IMU, VO, GPS(if available), RTK (if available), ultrasonic, magnetometer, barometer
     * @units m
     * @datastruct \ref T_DjiFcSubscriptionPositionVO
     */
    T_DjiReturnCode SubscribeTopicPositionVO(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief Provides obstacle info around the vehicle @ up to 100Hz
     * @datastruct \ref T_DjiFcSubscriptionAvoidData
     */
    T_DjiReturnCode SubscribeTopicAvoidData(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief Provides status of whether the home point was set or not
     * @datastruct \ref T_DjiFcSubscriptionHomePointSetStatus
     */
    T_DjiReturnCode SubscribeTopicHomePointSetStatus(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief Provides homepoint information, the valid of the home point infomation can ref to the
     * topic DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_SET_STATUS
     * @datastruct \ref T_DjiFcSubscriptionHomePointInfo
     */
    T_DjiReturnCode SubscribeTopicHomePointInfo(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief Provides three gimbal information, used for M300
     * @datastruct \ref T_DjiFcSubscriptionThreeGimbalData
     */
    T_DjiReturnCode SubscribeTopicThreeGimbalData(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief Battery information topic name. Please refer to ::T_DjiFcSubscriptionSingleBatteryInfo for information about data structure.
     * @datastruct \ref T_DjiFcSubscriptionSingleBatteryInfo
     */
    T_DjiReturnCode SubscribeTopicSingleBatteryInfoIndex1(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);

    /*!
     * @brief Battery information topic name. Please refer to ::T_DjiFcSubscriptionSingleBatteryInfo for information about data structure.
     * @datastruct \ref T_DjiFcSubscriptionSingleBatteryInfo
     */
    T_DjiReturnCode SubscribeTopicSingleBatteryInfoIndex2(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);
    
    /*!
     * @brief Please refer to ::T_DjiFcSubscriptionImuAttiNaviDataWithTimestamp for information about data structure.
     * @datastruct \ref T_DjiFcSubscriptionImuAttiNaviDataWithTimestamp
     */
    T_DjiReturnCode SubscribeTopicImuAttiNaviDataWithTimestamp(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);
    

    /*! Total number of topics that can be subscribed. */
    T_DjiReturnCode SubscribeTopicTotalNumber(DjiReceiveDataOfTopicCallback callback, bool subscribe_unsubscribe=true);


private:
};

/* Exported functions --------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif // TEST_PERCEPTION_H
/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/

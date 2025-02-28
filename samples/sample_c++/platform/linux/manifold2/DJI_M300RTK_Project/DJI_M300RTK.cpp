/**
 ********************************************************************
 * @file    application.cpp
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
#include "DJI_M300RTK.hpp"
#include "dji_sdk_app_info.h"
#include <dji_platform.h>
#include <dji_logger.h>
#include <dji_core.h>
#include <dji_aircraft_info.h>
#include <csignal>
#include "dji_sdk_config.h"
#include <sys/select.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <fstream>
#include <mutex>

#include "../common/osal/osal.h"
#include "../common/osal/osal_fs.h"
#include "../common/osal/osal_socket.h"
#include "../manifold2/hal/hal_usb_bulk.h"
#include "../manifold2/hal/hal_uart.h"
#include "../manifold2/hal/hal_network.h"

#include "utils/dji_config_manager.h"
//#include <gimbal_emu/test_payload_gimbal_emu.h>
//#include <camera_emu/test_payload_cam_emu_media.h>
//#include <camera_emu/test_payload_cam_emu_base.h>
#include "widget/test_widget.h"
#include "widget/test_widget_speaker.h"
#include <power_management/test_power_management.h>
#include "data_transmission/test_data_transmission.h"
#include "fc_subscription/test_fc_subscription.h"
#include <fc_subscription/FC_Subscription.hpp>
#include <cstddef>
#include <cstdio>
#include <chrono>
#include <unistd.h>
#include <stdio.h>

/* Private constants ---------------------------------------------------------*/
#define DJI_LOG_PATH                    "Logs/DJI"
#define DJI_LOG_INDEX_FILE_NAME         "Logs/index"
#define DJI_LOG_FOLDER_NAME             "Logs"
#define DJI_LOG_PATH_MAX_SIZE           (128)
#define DJI_LOG_FOLDER_NAME_MAX_SIZE    (32)
#define DJI_SYSTEM_CMD_STR_MAX_SIZE     (64)
#define DJI_LOG_MAX_COUNT               (10)

#define USER_UTIL_UNUSED(x)                                 ((x) = (x))
#define USER_UTIL_MIN(a, b)                                 (((a) < (b)) ? (a) : (b))
#define USER_UTIL_MAX(a, b)                                 (((a) > (b)) ? (a) : (b))

#define DJI_USE_SDK_CONFIG_BY_JSON      (0)

/* Private types -------------------------------------------------------------*/
static FILE *s_djiLogFile;
static FILE *s_djiLogFileCnt;

std::timed_mutex MutexForCopyData;
bool DJI_M300RTK::isStop=false;
bool DJI_M300RTK::isStart=false;

/* Private values -------------------------------------------------------------*/


/* Private functions declaration ---------------------------------------------*/
static void DjiUser_NormalExitHandler(int signalNum);
static T_DjiReturnCode DjiTest_HighPowerApplyPinInit();
static T_DjiReturnCode DjiTest_WriteHighPowerApplyPin(E_DjiPowerManagementPinState pinState);

/* Exported functions definition ---------------------------------------------*/
DJI_M300RTK::DJI_M300RTK(const std::string _fileName)
    : fileName(_fileName), pollPeriodSpan((unsigned)200u)
{

    ///DjiUser_SetupEnvironment
    T_DjiReturnCode returnCode;
    T_DjiOsalHandler osalHandler = {0};
    T_DjiHalUartHandler uartHandler = {0};
    T_DjiHalUsbBulkHandler usbBulkHandler = {0};
    T_DjiLoggerConsole printConsole;
    T_DjiLoggerConsole localRecordConsole;
    T_DjiFileSystemHandler fileSystemHandler = {0};
    T_DjiSocketHandler socketHandler{0};
    T_DjiHalNetworkHandler networkHandler = {0};
    T_DjiUserLinkConfig linkConfig;

    networkHandler.NetworkInit = HalNetWork_Init;
    networkHandler.NetworkDeInit = HalNetWork_DeInit;
    networkHandler.NetworkGetDeviceInfo = HalNetWork_GetDeviceInfo;

    socketHandler.Socket = Osal_Socket;
    socketHandler.Bind = Osal_Bind;
    socketHandler.Close = Osal_Close;
    socketHandler.UdpSendData = Osal_UdpSendData;
    socketHandler.UdpRecvData = Osal_UdpRecvData;
    socketHandler.TcpListen = Osal_TcpListen;
    socketHandler.TcpAccept = Osal_TcpAccept;
    socketHandler.TcpConnect = Osal_TcpConnect;
    socketHandler.TcpSendData = Osal_TcpSendData;
    socketHandler.TcpRecvData = Osal_TcpRecvData;

    osalHandler.TaskCreate = Osal_TaskCreate;
    osalHandler.TaskDestroy = Osal_TaskDestroy;
    osalHandler.TaskSleepMs = Osal_TaskSleepMs;
    osalHandler.MutexCreate = Osal_MutexCreate;
    osalHandler.MutexDestroy = Osal_MutexDestroy;
    osalHandler.MutexLock = Osal_MutexLock;
    osalHandler.MutexUnlock = Osal_MutexUnlock;
    osalHandler.SemaphoreCreate = Osal_SemaphoreCreate;
    osalHandler.SemaphoreDestroy = Osal_SemaphoreDestroy;
    osalHandler.SemaphoreWait = Osal_SemaphoreWait;
    osalHandler.SemaphoreTimedWait = Osal_SemaphoreTimedWait;
    osalHandler.SemaphorePost = Osal_SemaphorePost;
    osalHandler.Malloc = Osal_Malloc;
    osalHandler.Free = Osal_Free;
    osalHandler.GetTimeMs = Osal_GetTimeMs;
    osalHandler.GetTimeUs = Osal_GetTimeUs;
    osalHandler.GetRandomNum = Osal_GetRandomNum;

    printConsole.func = DjiUser_PrintConsole;
    printConsole.consoleLevel = DJI_LOGGER_CONSOLE_LOG_LEVEL_INFO;
    printConsole.isSupportColor = true;

    localRecordConsole.consoleLevel = DJI_LOGGER_CONSOLE_LOG_LEVEL_DEBUG;
    localRecordConsole.func = DjiUser_LocalWrite;
    localRecordConsole.isSupportColor = false;

    uartHandler.UartInit = HalUart_Init;
    uartHandler.UartDeInit = HalUart_DeInit;
    uartHandler.UartWriteData = HalUart_WriteData;
    uartHandler.UartReadData = HalUart_ReadData;
    uartHandler.UartGetStatus = HalUart_GetStatus;

    usbBulkHandler.UsbBulkInit = HalUsbBulk_Init;
    usbBulkHandler.UsbBulkDeInit = HalUsbBulk_DeInit;
    usbBulkHandler.UsbBulkWriteData = HalUsbBulk_WriteData;
    usbBulkHandler.UsbBulkReadData = HalUsbBulk_ReadData;
    usbBulkHandler.UsbBulkGetDeviceInfo = HalUsbBulk_GetDeviceInfo;

    fileSystemHandler.FileOpen = Osal_FileOpen,
    fileSystemHandler.FileClose = Osal_FileClose,
    fileSystemHandler.FileWrite = Osal_FileWrite,
    fileSystemHandler.FileRead = Osal_FileRead,
    fileSystemHandler.FileSync = Osal_FileSync,
    fileSystemHandler.FileSeek = Osal_FileSeek,
    fileSystemHandler.DirOpen = Osal_DirOpen,
    fileSystemHandler.DirClose = Osal_DirClose,
    fileSystemHandler.DirRead = Osal_DirRead,
    fileSystemHandler.Mkdir = Osal_Mkdir,
    fileSystemHandler.Unlink = Osal_Unlink,
    fileSystemHandler.Rename = Osal_Rename,
    fileSystemHandler.Stat = Osal_Stat,

    returnCode = DjiPlatform_RegOsalHandler(&osalHandler);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("Register osal handler error.");
    }

    returnCode = DjiPlatform_RegHalUartHandler(&uartHandler);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("Register hal uart handler error.");
    }

    #if DJI_USE_SDK_CONFIG_BY_JSON
        if (argc > 1) {
            DjiUserConfigManager_LoadConfiguration(argv[1]);
        } else {
            DjiUserConfigManager_LoadConfiguration(nullptr);
        }

        DjiUserConfigManager_GetLinkConfig(&linkConfig);
        if (linkConfig.type == DJI_USER_LINK_CONFIG_USE_UART_AND_NETWORK_DEVICE) {
            returnCode = DjiPlatform_RegHalNetworkHandler(&networkHandler);
            if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                throw std::runtime_error("Register hal network handler error");
            }
        } else if (linkConfig.type == DJI_USER_LINK_CONFIG_USE_UART_AND_USB_BULK_DEVICE) {
            returnCode = DjiPlatform_RegHalUsbBulkHandler(&usbBulkHandler);
            if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                throw std::runtime_error("Register hal usb bulk handler error.");
            }
        } else {
            /*!< Attention: Only use uart hardware connection. */
        }
    #else
        #if (CONFIG_HARDWARE_CONNECTION == DJI_USE_UART_AND_USB_BULK_DEVICE)
            returnCode = DjiPlatform_RegHalUsbBulkHandler(&usbBulkHandler);
            if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                throw std::runtime_error("Register hal usb bulk handler error.");
            }
        #elif (CONFIG_HARDWARE_CONNECTION == DJI_USE_UART_AND_NETWORK_DEVICE)
            returnCode = DjiPlatform_RegHalNetworkHandler(&networkHandler);
            if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                throw std::runtime_error("Register hal network handler error");
            }
        #elif (CONFIG_HARDWARE_CONNECTION == DJI_USE_ONLY_UART)
            /*!< Attention: Only use uart hardware connection.
            */
        #endif
    #endif

    //Attention: if you want to use camera stream view function, please uncomment it.
    returnCode = DjiPlatform_RegSocketHandler(&socketHandler);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("register osal socket handler error");
    }

    returnCode = DjiPlatform_RegFileSystemHandler(&fileSystemHandler);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("Register osal filesystem handler error.");
    }

    if (DjiUser_LocalWriteFsInit(DJI_LOG_PATH) != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("File system init error.");
    }

    returnCode = DjiLogger_AddConsole(&printConsole);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("Add printf console error.");
    }

    returnCode = DjiLogger_AddConsole(&localRecordConsole);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("Add printf console error.");
    }
     
    ///DjiUser_ApplicationStart();
    T_DjiUserInfo userInfo;
    T_DjiAircraftInfoBaseInfo aircraftInfoBaseInfo;
    T_DjiFirmwareVersion firmwareVersion = {
        .majorVersion = 1,
        .minorVersion = 0,
        .modifyVersion = 0,
        .debugVersion = 0,
    };
    #if DJI_USE_SDK_CONFIG_BY_JSON
        DjiUserConfigManager_GetAppInfo(&userInfo);
    #else
        returnCode = DjiUser_FillInUserInfo(&userInfo);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            throw std::runtime_error("Fill user info error, please check user info config.");
    }
    #endif


    returnCode = DjiCore_Init(&userInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        sleep(1);
        throw std::runtime_error("Core init error.");
    }

    //if (aircraftInfoBaseInfo.mountPosition != DJI_MOUNT_POSITION_EXTENSION_PORT
    //    && DJI_MOUNT_POSITION_EXTENSION_LITE_PORT != aircraftInfoBaseInfo.mountPosition) {
    //    throw std::runtime_error("Please run this sample on extension port.");
    //}

    returnCode = DjiCore_SetAlias("PSDK_APPALIAS");
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("Set alias error.");
    }

    returnCode = DjiCore_SetFirmwareVersion(firmwareVersion);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("Set firmware version error.");
    }
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("Set serial number error");
    }

    #ifdef CONFIG_MODULE_SAMPLE_CAMERA_EMU_ON
        returnCode = DjiTest_CameraEmuBaseStartService();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("camera emu common init error");
        }
    #endif

    #ifdef CONFIG_MODULE_SAMPLE_CAMERA_MEDIA_ON
        returnCode = DjiTest_CameraEmuMediaStartService();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("camera emu media init error");
        }
    #endif


    #ifdef CONFIG_MODULE_SAMPLE_FC_SUBSCRIPTION_ON
        returnCode = DjiTest_FcSubscriptionStartService();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("data subscription sample init error\n");
        }
    #endif

    #ifdef CONFIG_MODULE_SAMPLE_GIMBAL_EMU_ON
        returnCode = DjiTest_GimbalStartService();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("psdk gimbal init error");
        }
    #endif

    #ifdef CONFIG_MODULE_SAMPLE_WIDGET_ON
        returnCode = DjiTest_WidgetStartService();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("widget sample init error");
        }
    #endif

    #ifdef CONFIG_MODULE_SAMPLE_WIDGET_SPEAKER_ON
        returnCode = DjiTest_WidgetSpeakerStartService();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("widget speaker test init error");
        }
    #endif

    #ifdef CONFIG_MODULE_SAMPLE_POWER_MANAGEMENT_ON
        T_DjiTest_ApplyHighPowerHandler applyHighPowerHandler = {
            .pinInit = DjiTest_HighPowerApplyPinInit,
            .pinWrite = DjiTest_WriteHighPowerApplyPin,
        };

        returnCode = DjiTest_RegApplyHighPowerHandler(&applyHighPowerHandler);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("regsiter apply high power handler error");
        }

        returnCode = DjiTest_PowerManagementStartService();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("power management init error");
        }
    #endif

    #ifdef CONFIG_MODULE_SAMPLE_DATA_TRANSMISSION_ON
        returnCode = DjiTest_DataTransmissionStartService();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("widget sample init error");
        }
    #endif


    returnCode = DjiCore_ApplicationStart();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        throw std::runtime_error("Start sdk application error.");
    }
    USER_LOG_INFO("Application start.");
    
    Osal_TaskSleepMs(3000);

    fcSubscription=new FC_Subscription();

    //fcSubscription->getSubscriptionData(pFCSubscriptionData);
}

DJI_M300RTK::~DJI_M300RTK()
= default;

/* Private functions definition-----------------------------------------------*/

T_DjiReturnCode DJI_M300RTK::connect()
{
    T_DjiReturnCode djiStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    
    USER_LOG_INFO("Fc subscription start");

    USER_LOG_INFO("Init fc subscription module");
    djiStat = DjiFcSubscription_Init();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("--->init data subscription module error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    USER_LOG_INFO("--->Subscribe the topics of quaternion");
    fcSubscription->SubscribeTopicQuaternian(NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    USER_LOG_INFO("--->Subscribe the topics of acceleration ground");
    fcSubscription->SubscribeTopicAccelerationGround(NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic acceleration ground error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    USER_LOG_INFO("--->Subscribe the topics of acceleration body");
    fcSubscription->SubscribeTopicAccelerationBody(NULL);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic acceleration body.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    USER_LOG_INFO("--->Subscribe the topics of acceleration raw");
    fcSubscription->SubscribeTopicAccelerationRaw(NULL);
      if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic acceleration raw.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    USER_LOG_INFO("--->Subscribe the topics of velocity");
    fcSubscription->SubscribeTopicVelocity(NULL);
      if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic velocity.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    USER_LOG_INFO("--->Subscribe the topics of angular rate fusion");
    fcSubscription->SubscribeTopicAngularRateFusioned(NULL);
      if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic angular rate fusion.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    USER_LOG_INFO("--->Subscribe the topics of angular rate raw");
    fcSubscription->SubscribeTopicAngularRateRaw(NULL);
      if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic angular rate raw.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    USER_LOG_INFO("--->Subscribe the topics of altitude fused");
    fcSubscription->SubscribeTopicAltitudeFused(NULL);
      if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic altitute fused.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    USER_LOG_INFO("--->Subscribe the topics of altitude barometer");
    fcSubscription->SubscribeTopicAltitudeBarometer(NULL);
      if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic altitute barometer.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    USER_LOG_INFO("--->Subscribe the topics of altitude of home point");
    fcSubscription->SubscribeTopicAltitudeOfHomePoint(NULL);
      if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic altitute of home point.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    USER_LOG_INFO("--->Subscribe the topics of height fusion");
    fcSubscription->SubscribeTopicHeightFusion(NULL);
      if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic height fusion.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    USER_LOG_INFO("--->Subscribe the topics of height relative");
    fcSubscription->SubscribeTopicHeightRelative(NULL);
      if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic height relative.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    USER_LOG_INFO("--->Subscribe the topics of position fused");
    fcSubscription->SubscribeTopicPositionFused(NULL);
      if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic position fused.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    USER_LOG_INFO("--->Subscribe the topics of flight status");
    fcSubscription->SubscribeTopicFlightStatus(NULL);
      if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic flight status.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    USER_LOG_INFO("--->Subscribe the topics of compass");
    fcSubscription->SubscribeTopicCompass(NULL);
      if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic compass.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    USER_LOG_INFO("--->Subscribe the topics of positionVO");
    fcSubscription->SubscribeTopicPositionVO(NULL);
      if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic positionVO.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    USER_LOG_INFO("--->Subscribe the topics of ImuAttiNaviDataWithTime");
    fcSubscription->SubscribeTopicImuAttiNaviDataWithTimestamp(NULL);
      if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic ImuAttiNaviDataWithTime.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }  

     //-------------------------
    //Configure the signal
    fd_set readfds;
    struct sigaction sa;

    sa.sa_handler = DJI_M300RTK::SignalHandler;     /* Establish signal handler */
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGUSR1, &sa, NULL);


    //Open File for records
    fileRecorder=new std::ofstream(fileName); 
    if (!fileRecorder->is_open()){
        throw std::runtime_error("Could not open file for writing: " + fileName);
        return false;
    }
    else{
        (*fileRecorder) << "systemtime_ms, " 
                        << "Quaternion (w,z,y,z), " 
                        << "QuaternionTimestamp(millisec,microsec), " 
                        << "AccelerationGround(x,y,z), "
                        << "AccelerationGroundTimestamp(millisec,microsec), "
                        << "AccelerationBody(x,y,z), "
                        << "AccelerationBodyTimestamp(millisec,microsec), " 
                        << "AccelerationRaw(x,y,z), "
                        << "AccelerationRawTimestamp(millisec,microsec), " 
                        << "Velocity(x,y,z), "
                        << "VelocityTimestamp(millisec,microsec), "
                        << "AngularRateFusioned(x,y,z), "
                        << "AngularRateFusionedTimestamp(millisec,microsec), "
                        << "AngularRateRaw(x,y,z), "
                        << "AngularRateRawTimestamp(millisec,microsec), "
                        << "AltitudeFused, "
                        << "AltitudeFusedTimestamp(millisec,microsec), "
                        << "AltitudeBarometer, "
                        << "AltitudeBarometerTimerstamp(millisec,microsec), "
                        << "AltitudeOfHomePoint, "
                        << "AltitudeOfHomePointTimerstamp(millisec,microsec), "
                        << "HeightFusion, " 
                        << "HeightFusionTimerstamp(millisec,microsec), "
                        << "HeightRelative, "
                        << "HeightRelativeTimerstamp(millisec,microsec), "
                        << "PositionFused (longitude, latitude, altitude), "
                        << "PositionFusedTimerstamp(millisec,microsec), "
                        << "Compass(x,y,z), "
                        << "CompassTimerstamp(millisec,microsec), "
                        << "FlightStatus, "
                        << "FlightStatusTimerstamp(millisec,microsec), "
                        << "PositionVO (x,y,z), "
                        << "PositionVOTimerstamp(millisec,microsec), "
                        << "ImuAttiNaviDataWithTimestamp (pn_x, pn_y, pn_z, vn_x, vn_y, vn_z, an_x, an_y, an_z, q1, q2, q3, q4, resv, cnt), "
                        << "ImuAttiNaviDataWithTimestampTimerstamp(millisec,microsec)"
                        <<'\n';

    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DJI_M300RTK::run()
{
    int64_t prevtimePeriodForTimeStamp=0;

    // Now let's block SIGUSR1
    sigset_t sigset;
    sigemptyset(&sigset);
    sigaddset(&sigset, SIGUSR1);
    sigprocmask(SIG_BLOCK, &sigset, NULL);

    // SIGUSR1 is now blocked and pending -- this call to sigwait will return
    // immediately
    int sig;
    int result = sigwait(&sigset, &sig);
    if(result == 0)
        printf("sigwait got signal: %d\n", sig);

    isStart=true;

    // acquire a single snapshot
    while(!isStop) {
        if(isStart){
            // make sure that capture is implemented periodically
            auto lastSnapTime = std::chrono::steady_clock::now();

            if (MutexForCopyData.try_lock_for(std::chrono::milliseconds(5))){
                FCSubscriptionDataTmp=fcSubscription->getSubscriptionData();
                MutexForCopyData.unlock();
            }

            //cout<<"QUA1:"<<pFCSubscriptionData->Quaternion.q0<<endl;
            //cout<<"QUA1:"<<FCSubscriptionDataTmp.Quaternion.q0<<endl;

            //printf("QUA1: %lf \n",pFCSubscriptionData->Quaternion.q0);
            //printf("QUA2: %lf \n",FCSubscriptionDataTmp.Quaternion.q0);

            if (fileRecorder->is_open()){

                // Get the current time from the system clock
                auto now = std::chrono::system_clock::now();
                // Convert the current time to time since epoch
                auto duration = now.time_since_epoch();
                // Convert duration to milliseconds
                auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
                
                (*fileRecorder) << milliseconds <<", " //Record system time ms
                //"Quaternion (w,x,y,z), " 
                << FCSubscriptionDataTmp.Quaternion.q0 <<", "   
                << FCSubscriptionDataTmp.Quaternion.q1 <<", "
                << FCSubscriptionDataTmp.Quaternion.q2 <<", " 
                << FCSubscriptionDataTmp.Quaternion.q3 <<", " 
                //"QuaternionTimestamp(millisec,microsec), " 
                << FCSubscriptionDataTmp.QuaternionTimestamp.millisecond << ", "
                << FCSubscriptionDataTmp.QuaternionTimestamp.microsecond << ", "
                // "AccelerationGround(x,y,z), "
                << FCSubscriptionDataTmp.AccelerationGround.x<<", "
                << FCSubscriptionDataTmp.AccelerationGround.y<<", "
                << FCSubscriptionDataTmp.AccelerationGround.z<<", "     
                // "AccelerationGroundTimestamp(millisec,microsec), "
                << FCSubscriptionDataTmp.AccelerationGroundTimestamp.millisecond << ", "
                << FCSubscriptionDataTmp.AccelerationGroundTimestamp.microsecond << ", "
                // "AccelerationBody(x,y,z), "
                << FCSubscriptionDataTmp.AccelerationBody.x<<", "
                << FCSubscriptionDataTmp.AccelerationBody.y<<", "
                << FCSubscriptionDataTmp.AccelerationBody.z<<", "
                // "AccelerationBodyTimestamp(millisec,microsec), " 
                << FCSubscriptionDataTmp.AccelerationBodyTimestamp.millisecond << ", "
                << FCSubscriptionDataTmp.AccelerationBodyTimestamp.microsecond << ", "        
                // "AccelerationRaw(x,y,z), "
                << FCSubscriptionDataTmp.AccelerationRaw.x<<", "
                << FCSubscriptionDataTmp.AccelerationRaw.y<<", "
                << FCSubscriptionDataTmp.AccelerationRaw.z<<", "
                // "AccelerationRawTimestamp(millisec,microsec), " 
                << FCSubscriptionDataTmp.AccelerationRawTimestamp.millisecond << ", "
                << FCSubscriptionDataTmp.AccelerationRawTimestamp.microsecond << ", " 
                // "Velocity(x,y,z), "
                << FCSubscriptionDataTmp.Velocity.data.x<<", "
                << FCSubscriptionDataTmp.Velocity.data.y<<", "
                << FCSubscriptionDataTmp.Velocity.data.z<<", "
                // "VelocityTimestamp(millisec,microsec), "
                << FCSubscriptionDataTmp.VelocityTimestamp.millisecond << ", "
                << FCSubscriptionDataTmp.VelocityTimestamp.microsecond << ", " 
                // "AngularRateFusioned(x,y,z), "
                << FCSubscriptionDataTmp.AngularRateFusioned.x<<", "
                << FCSubscriptionDataTmp.AngularRateFusioned.y<<", "
                << FCSubscriptionDataTmp.AngularRateFusioned.z<<", "
                // "AngularRateFusionedTimestamp(millisec,microsec), "
                << FCSubscriptionDataTmp.AngularRateFusionedTimestamp.millisecond << ", "
                << FCSubscriptionDataTmp.AngularRateFusionedTimestamp.microsecond << ", " 
                // "AngularRateRaw(x,y,z), "
                << FCSubscriptionDataTmp.AngularRateRaw.x<<", "
                << FCSubscriptionDataTmp.AngularRateRaw.y<<", "
                << FCSubscriptionDataTmp.AngularRateRaw.z<<", "
                // "AngularRateRawTimestamp(millisec,microsec), "
                << FCSubscriptionDataTmp.AngularRateRawTimestamp.millisecond << ", "
                << FCSubscriptionDataTmp.AngularRateRawTimestamp.microsecond << ", " 
                // "AltitudeFused, "
                << FCSubscriptionDataTmp.AltitudeFused<<", "
                // "AltitudeFusedTimestamp(millisec,microsec), "
                << FCSubscriptionDataTmp.AltitudeFusedTimestamp.millisecond << ", "
                << FCSubscriptionDataTmp.AltitudeFusedTimestamp.microsecond << ", " 
                // "AltitudeBarometer, "
                << FCSubscriptionDataTmp.AltitudeBarometer<<", "
                // "AltitudeBarometerTimerstamp(millisec,microsec), "
                << FCSubscriptionDataTmp.AltitudeBarometerTimestamp.millisecond << ", "
                << FCSubscriptionDataTmp.AltitudeBarometerTimestamp.microsecond << ", " 
                // "AltitudeOfHomePoint, "
                << FCSubscriptionDataTmp.AltitudeOfHomePoint<<", "
                // "AltitudeOfHomePointTimerstamp(millisec,microsec), "
                << FCSubscriptionDataTmp.AltitudeOfHomePointTimestamp.millisecond << ", "
                << FCSubscriptionDataTmp.AltitudeOfHomePointTimestamp.microsecond << ", " 
                // "HeightFusion, "
                << FCSubscriptionDataTmp.HeightFusion<<", "
                // "HeightFusionTimerstamp(millisec,microsec), "
                << FCSubscriptionDataTmp.HeightFusionTimestamp.millisecond << ", "
                << FCSubscriptionDataTmp.HeightFusionTimestamp.microsecond << ", " 
                // "HeightRelative, "
                << FCSubscriptionDataTmp.HeightRelative<<", "
                // "HeightRelativeTimerstamp(millisec,microsec), "
                << FCSubscriptionDataTmp.HeightRelativeTimestamp.millisecond << ", "
                << FCSubscriptionDataTmp.HeightRelativeTimestamp.microsecond << ", " 
                // "PositionFused (longitude, latitude, altitude, NumVisiableServer), "
                << FCSubscriptionDataTmp.PositionFused.longitude<<", "
                << FCSubscriptionDataTmp.PositionFused.latitude<<", "
                << FCSubscriptionDataTmp.PositionFused.altitude<<", "
                // "PositionFusedTimerstamp(millisec,microsec), "
                << FCSubscriptionDataTmp.PositionFusedTimestamp.millisecond << ", "
                << FCSubscriptionDataTmp.PositionFusedTimestamp.microsecond << ", " 
                // "Compass(x,y,z), "
                << FCSubscriptionDataTmp.Compass.x <<", "
                << FCSubscriptionDataTmp.Compass.y <<", "
                << FCSubscriptionDataTmp.Compass.z <<", "
                // "CompassTimerstamp(millisec,microsec), "
                << FCSubscriptionDataTmp.CompassTimestamp.millisecond << ", "
                << FCSubscriptionDataTmp.CompassTimestamp.microsecond << ", " 
                // "FlightStatus, "
                << FCSubscriptionDataTmp.FlightStatus <<", "
                // "FlightStatusTimerstamp(millisec,microsec), "
                << FCSubscriptionDataTmp.FlightStatusTimestamp.millisecond << ", "
                << FCSubscriptionDataTmp.FlightStatusTimestamp.microsecond << ", " 
                // "PositionVO (x,y,z), "
                << FCSubscriptionDataTmp.PositionVO.x <<", "
                << FCSubscriptionDataTmp.PositionVO.y <<", "
                << FCSubscriptionDataTmp.PositionVO.z <<", "
                // "PositionVOTimerstamp(millisec,microsec), "
                << FCSubscriptionDataTmp.PositionVOTimestamp.millisecond << ", "
                << FCSubscriptionDataTmp.PositionVOTimestamp.microsecond << ", " 
                // "ImuAttiNaviDataWithTimestamp (pn_x, pn_y, pn_z, vn_x, vn_y, vn_z, an_x, an_y, an_z, q1, q2, q3, q4, resv,cnt  ), "
                << FCSubscriptionDataTmp.ImuAttiNaviDataWithTimestamp.pn_x <<", "
                << FCSubscriptionDataTmp.ImuAttiNaviDataWithTimestamp.pn_y <<", "
                << FCSubscriptionDataTmp.ImuAttiNaviDataWithTimestamp.pn_z <<", "
                << FCSubscriptionDataTmp.ImuAttiNaviDataWithTimestamp.vn_x <<", "
                << FCSubscriptionDataTmp.ImuAttiNaviDataWithTimestamp.vn_y <<", "
                << FCSubscriptionDataTmp.ImuAttiNaviDataWithTimestamp.vn_z <<", "
                << FCSubscriptionDataTmp.ImuAttiNaviDataWithTimestamp.an_x <<", "
                << FCSubscriptionDataTmp.ImuAttiNaviDataWithTimestamp.an_y <<", "
                << FCSubscriptionDataTmp.ImuAttiNaviDataWithTimestamp.an_z <<", "
                << FCSubscriptionDataTmp.ImuAttiNaviDataWithTimestamp.q[0] <<", "
                << FCSubscriptionDataTmp.ImuAttiNaviDataWithTimestamp.q[1] <<", "
                << FCSubscriptionDataTmp.ImuAttiNaviDataWithTimestamp.q[2] <<", "
                << FCSubscriptionDataTmp.ImuAttiNaviDataWithTimestamp.q[3] <<", "
                << FCSubscriptionDataTmp.ImuAttiNaviDataWithTimestamp.resv <<", "
                << FCSubscriptionDataTmp.ImuAttiNaviDataWithTimestamp.cnt <<", "
                // "ImuAttiNaviDataWithTimestampTimerstamp(millisec,microsec), "
                << FCSubscriptionDataTmp.ImuAttiNaviDataWithTimestampTimestamp.millisecond << ", "
                << FCSubscriptionDataTmp.ImuAttiNaviDataWithTimestampTimestamp.microsecond 
                <<'\n';
        
            }

            //Wait for keeping the period
            const auto timeSinceLastSnap = std::chrono::steady_clock::now() - lastSnapTime;

            if (timeSinceLastSnap < pollPeriodSpan){
                auto timeToWait = pollPeriodSpan - timeSinceLastSnap;
                std::this_thread::sleep_for(timeToWait);
            }            
        } //if(isStart){
    } //while(!isStop) {

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DJI_M300RTK::disConnect()
{
    T_DjiReturnCode djiStat;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

    USER_LOG_INFO("--->Unsubscribe the topics");
    djiStat = fcSubscription->SubscribeTopicQuaternian(NULL,false);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic quaternion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    else
        USER_LOG_INFO("-->UnSubscribe the topic quaternion...");

    djiStat = fcSubscription->SubscribeTopicAccelerationGround(NULL,false);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic AccelerationGround error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    else
        USER_LOG_INFO("-->UnSubscribe the topic AccelerationGround...");



    djiStat = fcSubscription->SubscribeTopicAccelerationBody(NULL,false);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic AccelerationBody error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
     else
        USER_LOG_INFO("-->UnSubscribe the topic AccelerationBody...");

    djiStat = fcSubscription->SubscribeTopicAccelerationRaw(NULL,false);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic AccelerationRaw error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
     else
        USER_LOG_INFO("-->UnSubscribe the topic AccelerationRaw...");

    djiStat = fcSubscription->SubscribeTopicVelocity(NULL,false);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic Velocity error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
     else
        USER_LOG_INFO("-->UnSubscribe the topic Velocity...");

    djiStat = fcSubscription->SubscribeTopicAngularRateFusioned(NULL,false);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic AngularRateFusioned error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
     else
        USER_LOG_INFO("-->UnSubscribe the topic AngularRateFusioned...");

    djiStat = fcSubscription->SubscribeTopicAngularRateRaw(NULL,false);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic AngularRateRaw error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
     else
        USER_LOG_INFO("-->UnSubscribe the topic AngularRateRaw...");

    djiStat = fcSubscription->SubscribeTopicAltitudeFused(NULL,false);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic AltitudeFused error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
     else
        USER_LOG_INFO("-->UnSubscribe the topic AltitudeFused...");

    djiStat = fcSubscription->SubscribeTopicAltitudeBarometer(NULL,false);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic AltitudeBarometer error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
     else
        USER_LOG_INFO("-->UnSubscribe the topic AltitudeBarometer...");

    djiStat = fcSubscription->SubscribeTopicAltitudeOfHomePoint(NULL,false);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic AltitudeOfHomePoint error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
     else
        USER_LOG_INFO("-->UnSubscribe the topic AltitudeOfHomePoint...");

    djiStat = fcSubscription->SubscribeTopicHeightFusion(NULL,false);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic HeightFusion error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
     else
        USER_LOG_INFO("-->UnSubscribe the topic HeightFusion...");

    djiStat = fcSubscription->SubscribeTopicHeightRelative(NULL,false);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic HeightRelative error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } 
     else
        USER_LOG_INFO("-->UnSubscribe the topic HeightRelative..."); 
    
    djiStat = fcSubscription->SubscribeTopicPositionFused(NULL,false);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic PositionFused error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
     else
        USER_LOG_INFO("-->UnSubscribe the topic PositionFused...");  

    djiStat = fcSubscription->SubscribeTopicFlightStatus(NULL,false);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic FlightStatus error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
     else
        USER_LOG_INFO("-->UnSubscribe the topic FlightStatus...");  

    djiStat = fcSubscription->SubscribeTopicCompass(NULL,false);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic Compass error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
     else
        USER_LOG_INFO("-->UnSubscribe the topic Compass..."); 

    djiStat = fcSubscription->SubscribeTopicPositionVO(NULL,false);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic PositionVO error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } 
     else
        USER_LOG_INFO("-->UnSubscribe the topic PositionVO...");

    djiStat = fcSubscription->SubscribeTopicImuAttiNaviDataWithTimestamp(NULL,false);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("UnSubscribe topic ImuAttiNaviDataWithTimestamp error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    } 
     else
        USER_LOG_INFO("-->UnSubscribe the topic ImuAttiNaviDataWithTimestamp...");

    USER_LOG_INFO("--> Deinit fc subscription module");

    djiStat = DjiFcSubscription_DeInit();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Deinit fc subscription error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

}

void DJI_M300RTK::SignalHandler(int sig)
{
    if(sig==SIGINT){
        std::cout<<"SIGINT: Exiting..."<<std::endl;
        isStop=true;
    }    
    if(sig==SIGUSR1){
        std::cout<<"SIGUSR1: Recording..."<<std::endl;
        isStart=true;
    }
}

T_DjiReturnCode DJI_M300RTK::DjiUser_PrintConsole(const uint8_t *data, uint16_t dataLen)
{
    printf("%s", data);

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DJI_M300RTK::DjiUser_LocalWrite(const uint8_t *data, uint16_t dataLen)
{
    int32_t realLen;

    if (s_djiLogFile == nullptr) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    realLen = fwrite(data, 1, dataLen, s_djiLogFile);
    fflush(s_djiLogFile);
    if (realLen == dataLen) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    } else {
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
}

T_DjiReturnCode DJI_M300RTK::DjiUser_FillInUserInfo(T_DjiUserInfo *userInfo)
{
    memset(userInfo->appName, 0, sizeof(userInfo->appName));
    memset(userInfo->appId, 0, sizeof(userInfo->appId));
    memset(userInfo->appKey, 0, sizeof(userInfo->appKey));
    memset(userInfo->appLicense, 0, sizeof(userInfo->appLicense));
    memset(userInfo->developerAccount, 0, sizeof(userInfo->developerAccount));
    memset(userInfo->baudRate, 0, sizeof(userInfo->baudRate));

    if (strlen(USER_APP_NAME) >= sizeof(userInfo->appName) ||
        strlen(USER_APP_ID) > sizeof(userInfo->appId) ||
        strlen(USER_APP_KEY) > sizeof(userInfo->appKey) ||
        strlen(USER_APP_LICENSE) > sizeof(userInfo->appLicense) ||
        strlen(USER_DEVELOPER_ACCOUNT) >= sizeof(userInfo->developerAccount) ||
        strlen(USER_BAUD_RATE) > sizeof(userInfo->baudRate)) {
        USER_LOG_ERROR("Length of user information string is beyond limit. Please check.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    if (!strcmp(USER_APP_NAME, "your_app_name") ||
        !strcmp(USER_APP_ID, "your_app_id") ||
        !strcmp(USER_APP_KEY, "your_app_key") ||
        !strcmp(USER_BAUD_RATE, "your_app_license") ||
        !strcmp(USER_DEVELOPER_ACCOUNT, "your_developer_account") ||
        !strcmp(USER_BAUD_RATE, "your_baud_rate")) {
        USER_LOG_ERROR(
            "Please fill in correct user information to 'samples/sample_c++/platform/linux/manifold2/application/dji_sdk_app_info.h' file.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    strncpy(userInfo->appName, USER_APP_NAME, sizeof(userInfo->appName) - 1);
    memcpy(userInfo->appId, USER_APP_ID, USER_UTIL_MIN(sizeof(userInfo->appId), strlen(USER_APP_ID)));
    memcpy(userInfo->appKey, USER_APP_KEY, USER_UTIL_MIN(sizeof(userInfo->appKey), strlen(USER_APP_KEY)));
    memcpy(userInfo->appLicense, USER_APP_LICENSE,
           USER_UTIL_MIN(sizeof(userInfo->appLicense), strlen(USER_APP_LICENSE)));
    memcpy(userInfo->baudRate, USER_BAUD_RATE, USER_UTIL_MIN(sizeof(userInfo->baudRate), strlen(USER_BAUD_RATE)));
    strncpy(userInfo->developerAccount, USER_DEVELOPER_ACCOUNT, sizeof(userInfo->developerAccount) - 1);

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DJI_M300RTK::DjiUser_LocalWriteFsInit(const char *path)
{
    T_DjiReturnCode djiReturnCode = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    char filePath[DJI_LOG_PATH_MAX_SIZE];
    char systemCmd[DJI_SYSTEM_CMD_STR_MAX_SIZE];
    char folderName[DJI_LOG_FOLDER_NAME_MAX_SIZE];
    time_t currentTime = time(nullptr);
    struct tm *localTime = localtime(&currentTime);
    uint16_t logFileIndex = 0;
    uint16_t currentLogFileIndex;
    uint8_t ret;

    if (localTime == nullptr) {
        printf("Get local time error.\r\n");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    if (access(DJI_LOG_FOLDER_NAME, F_OK) != 0) {
        sprintf(folderName, "mkdir %s", DJI_LOG_FOLDER_NAME);
        ret = system(folderName);
        if (ret != 0) {
            printf("Create new log folder error, ret:%d.\r\n", ret);
            return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
        }
    }

    s_djiLogFileCnt = fopen(DJI_LOG_INDEX_FILE_NAME, "rb+");
    if (s_djiLogFileCnt == nullptr) {
        s_djiLogFileCnt = fopen(DJI_LOG_INDEX_FILE_NAME, "wb+");
        if (s_djiLogFileCnt == nullptr) {
            return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
        }
    } else {
        ret = fseek(s_djiLogFileCnt, 0, SEEK_SET);
        if (ret != 0) {
            printf("Seek log count file error, ret: %d, errno: %d.\r\n", ret, errno);
            return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
        }

        ret = fread((uint16_t *) &logFileIndex, 1, sizeof(uint16_t), s_djiLogFileCnt);
        if (ret != sizeof(uint16_t)) {
            printf("Read log file index error.\r\n");
        }
    }

    currentLogFileIndex = logFileIndex;
    logFileIndex++;

    ret = fseek(s_djiLogFileCnt, 0, SEEK_SET);
    if (ret != 0) {
        printf("Seek log file error, ret: %d, errno: %d.\r\n", ret, errno);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    ret = fwrite((uint16_t *) &logFileIndex, 1, sizeof(uint16_t), s_djiLogFileCnt);
    if (ret != sizeof(uint16_t)) {
        printf("Write log file index error.\r\n");
        fclose(s_djiLogFileCnt);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    fclose(s_djiLogFileCnt);

    sprintf(filePath, "%s_%04d_%04d%02d%02d_%02d-%02d-%02d.log", path, currentLogFileIndex,
            localTime->tm_year + 1900, localTime->tm_mon + 1, localTime->tm_mday,
            localTime->tm_hour, localTime->tm_min, localTime->tm_sec);

    s_djiLogFile = fopen(filePath, "wb+");
    if (s_djiLogFile == nullptr) {
        USER_LOG_ERROR("Open filepath time error.");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    if (logFileIndex >= DJI_LOG_MAX_COUNT) {
        sprintf(systemCmd, "rm -rf %s_%04d*.log", path, currentLogFileIndex - DJI_LOG_MAX_COUNT);
        ret = system(systemCmd);
        if (ret != 0) {
            printf("Remove file error, ret:%d.\r\n", ret);
            return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
        }
    }

    sprintf(systemCmd, "ln -sfrv %s " DJI_LOG_FOLDER_NAME "/latest.log", filePath);
    system(systemCmd);
    return djiReturnCode;
}

static T_DjiReturnCode DjiTest_HighPowerApplyPinInit()
{
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode DjiTest_WriteHighPowerApplyPin(E_DjiPowerManagementPinState pinState)
{
    //attention: please pull up the HWPR pin state by hardware.
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/

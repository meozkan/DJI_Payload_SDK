/**
 ********************************************************************
 * @file    main.cpp
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
//#include <liveview/test_liveview_entry.hpp>
//#include <perception/test_perception_entry.hpp>
//#include <flight_control/test_flight_control.h>
//#include <gimbal/test_gimbal_entry.hpp>
//#include "application.hpp"
#include "fc_subscription/test_fc_subscription.h"
//#include <gimbal_emu/test_payload_gimbal_emu.h>
//#include <camera_emu/test_payload_cam_emu_media.h>
//#include <camera_emu/test_payload_cam_emu_base.h>
#include <dji_logger.h>
//#include "widget/test_widget.h"
//#include "widget/test_widget_speaker.h"
//#include <power_management/test_power_management.h>
#include "data_transmission/test_data_transmission.h"
//#include <flight_controller/test_flight_controller_entry.h>
//#include <positioning/test_positioning.h>
//#include <hms_manager/hms_manager_entry.h>
//#include "camera_manager/test_camera_manager_entry.h"
 #include <DJI_M300RTK.hpp>
 #include <iostream>

/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private values -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/

/* Exported functions definition ---------------------------------------------*/

DJI_M300RTK *dji_M300rtk;

int main(int argc, char **argv)
{

    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    T_DjiReturnCode returnCode;  
    try{
        dji_M300rtk = new DJI_M300RTK();
    }
    catch(const runtime_error& error)
    {
        std::cout<<"DJI_M300RTK Constractor error: "<<error.what()<<std::endl;

    }
    std::cout<<"Connecting..."<<std::endl;
    if(DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS!=dji_M300rtk->connect()){
        std::cout<<"not connect... Exiting..."<<std::endl;
        return 0;
    } 
    std::cout<<"Connected..."<<std::endl;


    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
    std::cout<<"Running..."<<std::endl;
    if(DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS!=dji_M300rtk->run()){
        std::cout<<"not stop run correctly... Exiting..."<<std::endl;
        std::cout<<"disconnecting..."<<std::endl;
        if(DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS!=dji_M300rtk->disConnect()){
            std::cout<<"not diconnect... Exiting..."<<std::endl;
            return 0;
        }
        return 0;
    } 
    std::cout<<"Stopped..."<<std::endl;

    std::cout<<"disconnecting..."<<std::endl;
    if(DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS!=dji_M300rtk->disConnect()){
        std::cout<<"not diconnect... Exiting..."<<std::endl;
        return 0;
    }

    std::cout<<"dji_M300rtk process has finished....Exiting..."<<std::endl;

    return 0; 

}

/* Private functions definition-----------------------------------------------*/

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/

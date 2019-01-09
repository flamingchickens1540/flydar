/*
 *  RPLIDAR
 *  Ultra Simple Data Grabber Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2018 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdio.h>
#include <stdlib.h>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

using namespace rp::standalone::rplidar;

bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;


    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

#include <signal.h>
bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

struct Polar2D
{
    double th;
    double r;
};

struct Point2D
{
    double x;
    double y;
};

Point2D polarToCartesian(Polar2D p) {
    return Point2D{
        p.r*cos(p.th),
        p.r*sin(p.th)
    };
}

int main(int argc, const char * argv[]) {
    const char * opt_com_path = NULL;
    _u32         baudrateArray[2] = {115200, 256000};
    _u32         opt_com_baudrate = 0;
    u_result     op_result;

    bool useArgcBaudrate = false;

    printf("Ultra simple LIDAR data grabber for RPLIDAR.\n"
           "Version: "RPLIDAR_SDK_VERSION"\n");

    // read serial port from the command line...
    if (argc>1) opt_com_path = argv[1]; // or set to a fixed value: e.g. "com3" 

    // read baud rate from the command line if specified...
    if (argc>2)
    {
        opt_com_baudrate = strtoul(argv[2], NULL, 10);
        useArgcBaudrate = true;
    }

    if (!opt_com_path) {
#ifdef _WIN32
        // use default com port
        opt_com_path = "\\\\.\\com3";
#else
        opt_com_path = "/dev/ttyUSB0";
#endif
    }

    // create the driver instance
	RPlidarDriver * drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }
    
    rplidar_response_device_info_t devinfo;
    bool connectSuccess = false;
    // make connection...
    if(useArgcBaudrate)
    {
        if(!drv)
            drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
        if (IS_OK(drv->connect(opt_com_path, opt_com_baudrate)))
        {
            op_result = drv->getDeviceInfo(devinfo);

            if (IS_OK(op_result)) 
            {
                connectSuccess = true;
            }
            else
            {
                delete drv;
                drv = NULL;
            }
        }
    }
    else
    {
        size_t baudRateArraySize = (sizeof(baudrateArray))/ (sizeof(baudrateArray[0]));
        for(size_t i = 0; i < baudRateArraySize; ++i)
        {
            if(!drv)
                drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
            if(IS_OK(drv->connect(opt_com_path, baudrateArray[i])))
            {
                op_result = drv->getDeviceInfo(devinfo);

                if (IS_OK(op_result)) 
                {
                    connectSuccess = true;
                    break;
                }
                else
                {
                    delete drv;
                    drv = NULL;
                }
            }
        }
    }
    if (!connectSuccess) {
        
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
            , opt_com_path);
        RPlidarDriver::DisposeDriver(drv);
        drv = NULL;
        return 0;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);



    // check health...
    if (!checkRPLIDARHealth(drv)) {
        RPlidarDriver::DisposeDriver(drv);
        drv = NULL;
        return 0;
    }

//    auto inst = nt::NetworkTableInstance::GetDefault();
    nt::NetworkTable::SetClientMode();
    nt::NetworkTable::SetIPAddress("10.15.40.2");

    auto limeTable = nt::NetworkTable::GetTable("limelight");
    auto tx00 = limeTable->GetEntry("tx00");
    auto tx11 = limeTable->GetEntry("tx11");

    auto smartDashboard = nt::NetworkTable::GetTable("SmartDashboard");
    auto goal_x = smartDashboard->GetEntry("--------goal_position_x");
    auto goal_y = smartDashboard->GetEntry("--------goal_position_y");
    auto goal_theta = smartDashboard->GetEntry("--------goal_orientation_z");

    bool sentFlag = false;

    signal(SIGINT, ctrlc);
    
    drv->startMotor();
    // start scan...
    drv->startScan(0,1);

    // fetech result and print it out...
    while (1) {
        rplidar_response_measurement_node_t nodes[8192];
        size_t   count = _countof(nodes);

        op_result = drv->grabScanData(nodes, count);

        if (IS_OK(op_result)) {
            float thetaA = NULL;
            float distanceA = NULL;
            float thetaB = NULL;
            float distanceB = NULL;

            double tx00val = tx00.GetDouble(99);
            double tx11val = tx11.GetDouble(99);
            double leftAngle = std::fmin(tx00val, tx11val);
            double rightAngle = std::fmax(tx00val, tx11val);

            drv->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count ; ++pos) {
                float theta = (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f - 270.0f;
                float distance = nodes[pos].distance_q2 / 4.0f / 1000.0f;
                if (theta > -30.0f && theta < 30.0f && distance > 0.0f) {
                    printf("%s theta: %03.2f Dist: %08.2f Q: %d ",
                           (nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ? "S " : "  ",
                           theta,
                           distance,
                           nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
                    for (int i = 0; i < (distance)*30.0f; ++i) {
                        printf("-");
                    }
                    if (fabs(theta+2.5f-leftAngle) < 1.0f) {
                        distanceB = distance;
                        thetaB = theta;
                        printf("|");
                    }
                    if (fabs(theta-0.0f-rightAngle) < 1.0f) {
                        distanceA = distance;
                        thetaA = theta;
                        printf("+");
                    }
                    printf("\n");
                }
            }

            if (distanceA && distanceB && tx00val < 99 && tx11val < 99 ) {
                Point2D pointA = polarToCartesian(Polar2D{M_PI/180*thetaA, distanceA});
                Point2D pointB = polarToCartesian(Polar2D{M_PI/180*thetaB, distanceB});

                printf("Point A: (%08.2f, %08.2f)\n", pointA.x, pointA.y);
                printf("Point B: (%08.2f, %08.2f)\n", pointB.x, pointB.y);

                double slope = (pointA.y - pointB.y) / (pointA.x - pointB.x);
                double x = 1*((slope*pointA.x + pointA.y)/(tan(M_PI/180*(leftAngle-rightAngle)/2)-slope)+0.1461);
                double y = x*(tan(M_PI/180*(leftAngle+rightAngle)/2));
                double angle = atan(-1/slope);
                double off = 0.3;
                double x_off = x+off*cos(angle);
                double y_off = y+off*sin(angle);
//                printf("Slope1: %08.2f Slope2: %08.2f\n", slope, (leftAngle+rightAngle)/2); 0.48m
                printf("Point C: (%08.3f, %08.3f)\n", x, y);
                printf("Point D: (%08.3f, %08.3f)\n", x_off, y_off);
                printf("Slope: %08.3f\n", angle);

                if (!sentFlag) {
                    goal_x.SetDouble(-x_off);
                    goal_y.SetDouble(-y_off);
                    goal_theta.SetDouble(angle);
                    sentFlag = true;
                }

//                RPlidarDriver::DisposeDriver(drv);
//                drv = NULL;
//                return 0;


            }
        }

        if (ctrl_c_pressed){ 
            break;
        }
    }

    drv->stop();
    drv->stopMotor();
    // done!
//on_finished:
    RPlidarDriver::DisposeDriver(drv);
    drv = NULL;
    return 0;
}


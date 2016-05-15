/*
 *  Copyright (c) BirdsEyeView Aerobotics, LLC, 2016.
 *
 *  This program is free software: you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 3 as published
 *  by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License version 3 for more details.
 *
 *  You should have received a copy of the GNU General Public License version
 *  3 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __BEV_Gimbal_H__
#define __BEV_Gimbal_H__

#include "BEV_Device.h"

#include <AP_GPS.h>
#include <AP_AHRS.h>
#include <AP_InertialNav.h>
#include <AP_HAL_PX4.h>

#define ENABLED                 1
#define DISABLED                0

#define BEV_GIMBAL_DEBUGGING ENABLED
#define BEV_GIMBAL_HFDEBUGGING DISABLED

class BEV_Gimbal : public BEV_Device
{
public:
    //Constructor
    BEV_Gimbal(BEV_PayloadCommunicator &p, int32_t payloadId);
    void set_ahrs_inav(AP_AHRS const* ahrs, AP_InertialNav const* inav) {_ahrs = ahrs; _inav = inav;}
    void update();
    void process_message(const bev_payload_struct & s);
    void center();
    void request_min_max();
    void point_here(int32_t lat, int32_t lng, int32_t alt);
    void set_pitch_yaw_speed(int16_t pitch_rate, int16_t yaw_rate);
    void set_test_angles(int16_t pitch, int16_t yaw);

private:
    //message handlers
    void _process_min_max_message(const bev_payload_struct & s);

    //members
    AP_AHRS const*                  _ahrs;
    AP_InertialNav const*           _inav;

    //gimbal limits
    int16_t                         _tilt_min;
    int16_t                         _tilt_max;
    int16_t                         _pan_min;
    int16_t                         _pan_max;

    void _push_regular_message();

};

#endif //__BEV_GIMBAL_H

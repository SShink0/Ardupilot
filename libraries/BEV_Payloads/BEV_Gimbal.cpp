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

#include <AP_HAL.h>
#include "BEV_Gimbal.h"

extern const AP_HAL::HAL& hal;

BEV_Gimbal::BEV_Gimbal(BEV_PayloadCommunicator &p, int32_t payloadId) :
            BEV_Device(p, payloadId),
            _ahrs(nullptr),
            _inav(nullptr)
{
    //set min / max angular displacements in case they're not read from the device
    _pan_min  =-16000;
    _pan_max  = 16000;
    _tilt_min = -9000;
    _tilt_max =     0;
}

void BEV_Gimbal::update()
{
    //decimate so calls are only at 4hz
    static uint32_t last_update_time = 0;
    if(hal.scheduler->millis() - last_update_time < 250) {
        return;
    }
    last_update_time = hal.scheduler->millis();

    //push updated attitude to px4io
    _push_regular_message();

#if BEV_GIMBAL_HFDEBUGGING == ENABLED
    hal.console->println("BEV_Gimbal::update() - pushing regular message");
#endif //BEV_GIMBAL_DEBUGGING ENABLED
}

void BEV_Gimbal::process_message(const bev_payload_struct& s)
{
    switch(s.message_id) {
    case 6:
        _process_min_max_message(s);
        break;
    default:
        break;
    }
}

void BEV_Gimbal::_process_min_max_message(const bev_payload_struct & s)
{
#if BEV_GIMBAL_DEBUGGING == ENABLED
    hal.console->println("BEV_GIMBAL::process_min_max_message()");
#endif //BEV_GIMBAL_DEBUGGING

    _tilt_min = s.d1;
    _tilt_max = s.d2;
    _pan_min  = s.d3;
    _pan_max  = s.d4;

#if BEV_GIMBAL_DEBUGGING == ENABLED
    hal.console->printf_P(PSTR("BEV_GIMBAL:: tilt_min:%d tilt_max:%d pan_min:%d. pan_max:%d\n"),_tilt_min, _tilt_max, _pan_min, _pan_max);
#endif //BEV_GIMBAL_DEBUGGING
}

void BEV_Gimbal::center()
{
#if BEV_GIMBAL_DEBUGGING == ENABLED
    hal.console->println("BEV_GIMBAL::Center()");
#endif //BEV_GIMBAL_DEBUGGING

    //BEV only if gimbal is connected. Otherewise wasting uORB bandwidth
    if(!_attached) {
        return;
    }

    bev_payload_struct s = BEV_PAYLOAD_STRUCT_DEFAULT;
    s.payload_id = _payloadId;
    s.message_id = 1;
    s.d1 = 1;
    _payloadCommunicator.push_important_message(s);
}

void BEV_Gimbal::point_here(int32_t lat, int32_t lng, int32_t alt)
{
#if BEV_GIMBAL_DEBUGGING == ENABLED
    hal.console->printf_P(PSTR("BEV_GIMBAL::point here (%3.2f, %3.2f, %3.2f)\n"), lat*1e-7, lng*1e-7, alt*1e-2);
#endif //BEV_GIMBAL_DEBUGGING

    //BEV only if gimbal is connected. Otherewise wasting uORB bandwidth
    if(!_attached) {
        return;
    }

    //check to make sure it's not command to center
    if(!lat && !lng && !alt) {
        this->center();
        return;
    }

    //relay command to px4io
    bev_payload_struct s = BEV_PAYLOAD_STRUCT_DEFAULT;
    s.payload_id = _payloadId;
    s.message_id = 3;
    s.d1 = lat;
    s.d2 = lng;
    s.d3 = alt;
    _payloadCommunicator.push_important_message(s);
}

void BEV_Gimbal::set_pitch_yaw_speed(int16_t pitch_rate, int16_t yaw_rate)
{
    //BEV only if gimbal is connected. Otherewise wasting uORB bandwidth
    if(!_attached) {
        return;
    }

    //relay command to px4io
    bev_payload_struct s = BEV_PAYLOAD_STRUCT_DEFAULT;
    s.payload_id = _payloadId;
    s.message_id = 4;
    s.d1 = pitch_rate;
    s.d2 = yaw_rate;
    _payloadCommunicator.push_unimportant_message(s);
}

void BEV_Gimbal::set_test_angles(int16_t pitch, int16_t yaw)
{
    //BEV only if gimbal is connected. Otherewise wasting uORB bandwidth
    if(!_attached) {
        return;
    }

    //relay command to px4io
    bev_payload_struct s = BEV_PAYLOAD_STRUCT_DEFAULT;
    s.payload_id = _payloadId;
    s.message_id = 5;
    s.d1 = pitch;
    s.d2 = yaw;
    _payloadCommunicator.push_important_message(s);
}

void BEV_Gimbal::request_min_max()
{
    bev_payload_struct s = BEV_PAYLOAD_STRUCT_DEFAULT;
    s.payload_id = _payloadId;
    s.message_id = 6;
    _payloadCommunicator.push_important_message(s);
}

void BEV_Gimbal::_push_regular_message()
{
    if( (_ahrs == nullptr) || (_inav == nullptr)) {
        return;
    }

    //BEV only if gimbal is connected. Otherewise wasting uORB bandwidth
    if(!_attached) {
        return;
    }

    bev_payload_struct s = BEV_PAYLOAD_STRUCT_DEFAULT;
    s.payload_id = _payloadId;
    s.message_id = 2;
    s.d1 = _inav->get_latitude();
    s.d2 = _inav->get_longitude();
    s.d3 = _inav->get_altitude();
    s.d4 = _ahrs->yaw_sensor << 8 | (uint8_t)_ahrs->get_gps().status();

    _payloadCommunicator.push_unimportant_message(s);
}

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 * AP_KDECAN.cpp
 *
 *      Author: Francisco Ferreira and Tom Pittenger
 */

#include "AP_KDECAN.h"

#if AP_KDECAN_ENABLED
#include <stdio.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>    // for MIN,MAX

extern const AP_HAL::HAL& hal;

// table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_KDECAN::var_info[] = {

    // @Param: NPOLE
    // @DisplayName: Number of motor poles
    // @Description: Sets the number of motor poles to calculate the correct RPM value
    AP_GROUPINFO("NPOLE", 1, AP_KDECAN, _num_poles, DEFAULT_NUM_POLES),

    AP_GROUPEND
};

AP_KDECAN::AP_KDECAN()
{
    AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_KDECAN must be singleton");
    }
#endif
    _singleton = this;
}

void AP_KDECAN::init()
{
    if (_driver != nullptr) {
        // only allow one instance
        return;
    }

    AP_CANManager::Driver_Type protocol = AP_CANManager::Driver_Type_None;

    for (uint8_t i = 0; i < HAL_NUM_CAN_IFACES; i++) {
#if HAL_CANMANAGER_ENABLED
        protocol = AP::can().get_driver_type(i);
#elif defined(HAL_BUILD_AP_PERIPH)
        protocol = CANSensor::get_driver_type(i);
#endif
        if (protocol == AP_CANManager::Driver_Type_KDECAN) {
            _driver = new AP_KDECAN_Driver();
            return;
        }
    }
}

void AP_KDECAN::update()
{
    if (_driver == nullptr) {
        return;
    }
    _driver->update((uint8_t)_num_poles.get());
}

AP_KDECAN_Driver::AP_KDECAN_Driver() : CANSensor("KDECAN")
{
    register_driver(AP_CANManager::Driver_Type_KDECAN);

    // start thread for receiving and sending CAN frames
    hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_KDECAN_Driver::loop, void), "kdecan", 4096, AP_HAL::Scheduler::PRIORITY_CAN, 0);
}

// parse inbound frames
void AP_KDECAN_Driver::handle_frame(AP_HAL::CANFrame &frame)
{
    if (!frame.isExtended()) {
        return;
    }

    const frame_id_t id { .value = frame.id & AP_HAL::CANFrame::MaskExtID };

    // if (id.object_address != TELEMETRY_OBJ_ADDR) {
    //     GCS_SEND_TEXT(MAV_SEVERITY_DEBUG,"KDECAN: rx id:%d, src:%d, dest:%d, len:%d", (int)id.object_address, (int)id.source_id, (int)id.destination_id, (int)frame.dlc);
    // }

    // check if frame is valid: directed at autopilot, doesn't come from broadcast and ESC was detected before
    switch (id.object_address) {
        case ESC_INFO_OBJ_ADDR:
            if (frame.dlc == 5 &&
                id.destination_id == AUTOPILOT_NODE_ID &&
                id.source_id >= ESC_NODE_ID_FIRST &&
                id.source_id < (KDECAN_MAX_NUM_ESCS + ESC_NODE_ID_FIRST))
            {
                const uint16_t bitmask = (1U << (id.source_id - ESC_NODE_ID_FIRST));

                if ((bitmask & _init.detected_bitmask) != bitmask) {
                    _init.detected_bitmask |= bitmask;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO,"KDECAN: Found ESC id %u", id.source_id);
                }
            }
        break;

#if HAL_WITH_ESC_TELEM
        case TELEMETRY_OBJ_ADDR:
            if (id.destination_id == AUTOPILOT_NODE_ID &&
                id.source_id != BROADCAST_NODE_ID &&
                (1U << (id.source_id - ESC_NODE_ID_FIRST) & _init.detected_bitmask) &&
                frame.dlc == 8)
            {
                const uint8_t idx = id.source_id - ESC_NODE_ID_FIRST;
                const uint8_t num_poles = _telemetry.num_poles > 0 ? _telemetry.num_poles : DEFAULT_NUM_POLES;
                update_rpm(idx, uint16_t(uint16_t(frame.data[4] << 8 | frame.data[5]) * 60UL * 2 / num_poles));

                TelemetryData t {
                    .temperature_cdeg = int16_t(frame.data[6] * 100),
                    .voltage = float(uint16_t(frame.data[0] << 8 | frame.data[1])) * 0.01f,
                    .current = float(uint16_t(frame.data[2] << 8 | frame.data[3])) * 0.01f,
                };
                update_telem_data(idx, t,
                    AP_ESC_Telem_Backend::TelemetryType::CURRENT |
                    AP_ESC_Telem_Backend::TelemetryType::VOLTAGE |
                    AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE);
            }
            break;
#endif // HAL_WITH_ESC_TELEM

        case UPDATE_NODE_ID_OBJ_ADDR:
            // reply from setting new node ID
            _init.detected_bitmask |= 1U << (id.source_id - ESC_NODE_ID_FIRST);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "KDECAN: Found ESC id %u", id.source_id);
            break;
    }
}

void AP_KDECAN_Driver::update(const uint8_t num_poles)
{
    if (!hal.util->get_soft_armed() || _init.detected_bitmask == 0) {
        return;
    }

#if HAL_WITH_ESC_TELEM
    _telemetry.num_poles = num_poles;
#endif
    
    uint16_t pwm[KDECAN_MAX_NUM_ESCS] {};

    uint8_t index = 0;
    for (uint16_t i = 0; i < ARRAY_SIZE(pwm); i++) {
        if ((_init.detected_bitmask & (1UL<<index)) == 0) {
            continue;
        }
        if (SRV_Channels::channel_function(i) > SRV_Channel::Aux_servo_function_t::k_none) {
            pwm[index++] = SRV_Channels::srv_channel(i)->get_output_pwm();
        }
    }

    {
        // queue the PWMs for loop()
        WITH_SEMAPHORE(_output.sem);
        memcpy(&_output.pwm, &pwm, sizeof(_output.pwm));
        _output.is_new = true;
    }

#if AP_KDECAN_USE_EVENTS
    if (_output.thread_ctx != nullptr) {
        // trigger the thread to wake up immediately
        chEvtSignal(_output.thread_ctx, 1);
    }
#endif
}

void AP_KDECAN_Driver::loop()
{
    uint16_t pwm[KDECAN_MAX_NUM_ESCS] {};

#if AP_KDECAN_USE_EVENTS
    _output.thread_ctx = chThdGetSelfX();
#endif

    uint8_t broadcast_esc_info_boot_spam_count = 3;
    uint32_t broadcast_esc_info_next_interval_ms = 100; // spam a few at boot at 5Hz

    while (true) {
#if AP_KDECAN_USE_EVENTS
        // sleep until we get new data, but also wake up at 400Hz to send the old data again
        chEvtWaitAnyTimeout(ALL_EVENTS, chTimeUS2I(2500));
 #else
        hal.scheduler->delay_microseconds(2500); // 400Hz
#endif

        const uint32_t now_ms = AP_HAL::millis();

        // This should run at 400Hz
        {
            WITH_SEMAPHORE(_output.sem);
            if (_output.is_new) {
                _output.last_new_ms = now_ms;
                _output.is_new = false;
                memcpy(&pwm, &_output.pwm, sizeof(pwm));

            } else if (_output.last_new_ms && now_ms - _output.last_new_ms > 1000) {
                // if we haven't gotten any PWM updates for a bit, zero it
                // out so we don't just keep sending the same values forever
                memset(&pwm, 0, sizeof(pwm));
                _output.last_new_ms = 0;
            }
        }

        uint8_t index = 0;
        uint8_t retry = 0;

        while (index < KDECAN_MAX_NUM_ESCS) {
            if ((_init.detected_bitmask & (1 << index)) == 0) {
                // we're not sending this index so skip it
                index++;
            } else if (send_packet_uint16(SET_PWM_OBJ_ADDR, (index + ESC_NODE_ID_FIRST), 1, pwm[index]) || retry++ >= 10) {
                // sent successfully or we've retried too many times, move on to the next
                index++;
                retry = 0;
            } else {
                // send failed, likely due to CAN TX buffer full. Delay a tiny bit and try again but only a few times
                hal.scheduler->delay_microseconds(10);
            }
        } // while index

#if HAL_WITH_ESC_TELEM
        // broadcast as request-telemetry msg to everyone
        if (_init.detected_bitmask != 0 && now_ms - _telemetry.timer_ms >= TELEMETRY_INTERVAL_MS) {
            if (send_packet(TELEMETRY_OBJ_ADDR, BROADCAST_NODE_ID, 10)) {
                _telemetry.timer_ms = now_ms;
            }
        }
#endif // HAL_WITH_ESC_TELEM

        if ((_init.detected_bitmask == 0 || broadcast_esc_info_boot_spam_count > 0) && (now_ms - _init.detected_bitmask_ms >= broadcast_esc_info_next_interval_ms)) {
            // broadcast an "anyone there?" quick at boot but then 1Hz forever until we see at least 1 esc respond
            if (broadcast_esc_info_boot_spam_count > 0) {
                broadcast_esc_info_boot_spam_count--;
            } else {
                broadcast_esc_info_next_interval_ms = 1000;
            }

            if (send_packet(ESC_INFO_OBJ_ADDR, BROADCAST_NODE_ID, 100)) {
                _init.detected_bitmask_ms = now_ms;
            }
        }

    } // while true
}

bool AP_KDECAN_Driver::send_packet_uint16(const uint8_t address, const uint8_t dest_id, const uint32_t timeout_ms, const uint16_t data)
{
    const uint16_t data_be16 = htobe16(data);
    return send_packet(address, dest_id, timeout_ms, (uint8_t*)&data_be16, 2);
}

bool AP_KDECAN_Driver::send_packet(const uint8_t address, const uint8_t dest_id, const uint32_t timeout_ms, const uint8_t *data, const uint8_t data_len)
{
    // broadcast telemetry request frame
    frame_id_t id = { { .object_address = address,
                        .destination_id = dest_id,
                        .source_id = AUTOPILOT_NODE_ID,
                        .priority = 0,
                        .unused = 0 } };

    AP_HAL::CANFrame frame = AP_HAL::CANFrame((id.value | AP_HAL::CANFrame::FlagEFF), data, data_len, false);

    const uint64_t timeout_us = uint64_t(timeout_ms) * 1000UL;
    return write_frame(frame, timeout_us);
}

bool AP_KDECAN_Driver::pre_arm_check(char* reason, const uint8_t reason_len)
{
    uint16_t configured_servo_motors_count = 0;
    for (uint16_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        if (SRV_Channel::is_motor(SRV_Channels::channel_function(i))) {
            configured_servo_motors_count++;
        }
    }

    const uint8_t num_present_escs = __builtin_popcount(_init.detected_bitmask);

    if (configured_servo_motors_count != num_present_escs) {
        snprintf(reason, reason_len, "ESC count error: Srv:%u, Detected:%u", (unsigned)configured_servo_motors_count, (unsigned)num_present_escs);
        return false;
    }

    return true;
}

// singleton instance
AP_KDECAN *AP_KDECAN::_singleton;

namespace AP {
AP_KDECAN *kdecan()
{
    return AP_KDECAN::get_singleton();
}
};

#endif // AP_KDECAN_ENABLED


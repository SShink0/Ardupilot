/*
  Implementation details for transfering waypoint information using
  the MISSION_ITEM protocol to and from a GCS.

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

#include "GCS_config.h"
#include <AP_Mission/AP_Mission_config.h>

#if HAL_GCS_ENABLED && AP_MISSION_ENABLED

#include "MissionItemProtocol_Waypoints.h"

#include <AP_Logger/AP_Logger.h>
#include <AP_Mission/AP_Mission.h>

#include "GCS.h"

MAV_MISSION_RESULT MissionItemProtocol_Waypoints::append_item(const mavlink_mission_item_int_t &mission_item_int)
{
    // sanity check for DO_JUMP command
    AP_Mission::Mission_Command cmd {};

    const MAV_MISSION_RESULT res = AP_Mission::mavlink_int_to_mission_cmd(mission_item_int, cmd);
    if (res != MAV_MISSION_ACCEPTED) {
        return res;
    }

    if (cmd.id == MAV_CMD_DO_JUMP) {
        if ((cmd.content.jump.target >= item_count() && cmd.content.jump.target > request_last) || cmd.content.jump.target == 0) {
            return MAV_MISSION_ERROR;
        }
    }

    if (!mission.add_cmd(cmd)) {
        return MAV_MISSION_ERROR;
    }
    return MAV_MISSION_ACCEPTED;
}

bool MissionItemProtocol_Waypoints::clear_all_items()
{
    return mission.clear();
}

MAV_MISSION_RESULT MissionItemProtocol_Waypoints::complete(const GCS_MAVLINK &_link)
{
    _link.send_text(MAV_SEVERITY_INFO, "Flight plan received");
    AP::logger().Write_EntireMission();
    invalidate_checksum();
    return MAV_MISSION_ACCEPTED;
}

MAV_MISSION_RESULT MissionItemProtocol_Waypoints::get_item(const GCS_MAVLINK &_link,
                                                           const mavlink_message_t &msg,
                                                           const mavlink_mission_request_int_t &packet,
                                                           mavlink_mission_item_int_t &ret_packet)
{
    if (packet.seq != 0 && // always allow HOME to be read
        packet.seq >= mission.num_commands()) {
        // try to educate the GCS on the actual size of the mission:
        const mavlink_channel_t chan = _link.get_chan();
        if (HAVE_PAYLOAD_SPACE(chan, MISSION_COUNT)) {
            uint32_t _opaque_id = 0;
            IGNORE_RETURN(opaque_id(_opaque_id));
            mavlink_msg_mission_count_send(chan,
                                           msg.sysid,
                                           msg.compid,
                                           mission.num_commands(),
                                           MAV_MISSION_TYPE_MISSION,
                                           _opaque_id);
        }
        return MAV_MISSION_ERROR;
    }

    AP_Mission::Mission_Command cmd;

    // retrieve mission from eeprom
    if (!mission.read_cmd_from_storage(packet.seq, cmd)) {
        return MAV_MISSION_ERROR;
    }

    if (!AP_Mission::mission_cmd_to_mavlink_int(cmd, ret_packet)) {
        return MAV_MISSION_ERROR;
    }

    // set packet's current field to 1 if this is the command being executed
    if (cmd.id == (uint16_t)mission.get_current_nav_cmd().index) {
        ret_packet.current = 1;
    } else {
        ret_packet.current = 0;
    }

    // set auto continue to 1
    ret_packet.autocontinue = 1;     // 1 (true), 0 (false)

    return MAV_MISSION_ACCEPTED;
}

uint16_t MissionItemProtocol_Waypoints::item_count() const {
    return mission.num_commands();
}

uint16_t MissionItemProtocol_Waypoints::max_items() const {
    return mission.num_commands_max();
}

MAV_MISSION_RESULT MissionItemProtocol_Waypoints::replace_item(const mavlink_mission_item_int_t &mission_item_int)
{
    AP_Mission::Mission_Command cmd {};

    const MAV_MISSION_RESULT res = AP_Mission::mavlink_int_to_mission_cmd(mission_item_int, cmd);
    if (res != MAV_MISSION_ACCEPTED) {
        return res;
    }

    // sanity check for DO_JUMP command
    if (cmd.id == MAV_CMD_DO_JUMP) {
        if ((cmd.content.jump.target >= item_count() && cmd.content.jump.target > request_last) || cmd.content.jump.target == 0) {
            return MAV_MISSION_ERROR;
        }
    }
    if (!mission.replace_cmd(cmd.index, cmd)) {
        return MAV_MISSION_ERROR;
    }
    return MAV_MISSION_ACCEPTED;
}

void MissionItemProtocol_Waypoints::timeout()
{
    link->send_text(MAV_SEVERITY_WARNING, "Mission upload timeout");
}

void MissionItemProtocol_Waypoints::truncate(const mavlink_mission_count_t &packet)
{
    // new mission arriving, truncate mission to be the same length
    mission.truncate(packet.count);
}

// returns a unique ID for this mission
bool MissionItemProtocol_Waypoints::opaque_id(uint32_t &checksum) const
{
    WITH_SEMAPHORE(mission.get_semaphore());
    switch (checksum_state.state) {
    case ChecksumState::READY:
        if (mission.last_change_time_ms() != checksum_state.mission_change_time_ms) {
            return false;
        }
        checksum = checksum_state.checksum;
        // can't use zero as the field is an extension field in mavlink2:
        if (checksum == 0) {
            checksum = UINT32_MAX;
        }
        return true;
    case ChecksumState::CALCULATING:
    case ChecksumState::ERROR:
        return false;
    }
    return false;
}

void MissionItemProtocol_Waypoints::invalidate_checksum()
{
    WITH_SEMAPHORE(mission.get_semaphore());

    checksum_state.state = ChecksumState::CALCULATING;
    checksum_state.checksum = 0;
    checksum_state.current_waypoint = 1;
    checksum_state.count = mission.num_commands();
    checksum_state.mission_change_time_ms = mission.last_change_time_ms();
}

void MissionItemProtocol_Waypoints::update_checksum()
{
    // update the checksum if required:

    WITH_SEMAPHORE(mission.get_semaphore());

    const uint32_t mission_last_change_time_ms = mission.last_change_time_ms();

    if (mission_last_change_time_ms == checksum_state.last_calculate_time_ms) {
        return;
    }

    // decide whether we need to start calculating the checksum from
    // the start; we may be partially through the calculation and need
    // to start again
    bool do_initialisation = false;
    switch (checksum_state.state) {
    case ChecksumState::READY:
        do_initialisation = true;
        break;
    case ChecksumState::CALCULATING:
        if (checksum_state.mission_change_time_ms != mission_last_change_time_ms) {
            // mission changed part-way through our calculations
            do_initialisation = true;
        }
        break;
    case ChecksumState::ERROR:
        do_initialisation = true;
        break;
    }

    if (do_initialisation) {
        invalidate_checksum();
    }

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - mission.last_change_time_ms() < 500) {
        // don't start to calculate unless the mission's been
        // unchanged for a while.
        return;
    }

    // AP: Took 2.178000ms to checksum 373 points (5.839142ms/1000 points
    for (uint16_t count = 0;
         count<16 && checksum_state.current_waypoint<checksum_state.count;
         count++, checksum_state.current_waypoint++) {
        AP_Mission::Mission_Command cmd;
        if (!mission.read_cmd_from_storage(checksum_state.current_waypoint, cmd)) {
            checksum_state.state = ChecksumState::ERROR;
            return;
        }
        mavlink_mission_item_int_t ret_packet;
        if (!AP_Mission::mission_cmd_to_mavlink_int(cmd, ret_packet)) {
            checksum_state.state = ChecksumState::ERROR;
            return;
        }
#define ADD_TO_CHECKSUM(field) checksum_state.checksum = crc_crc32(checksum_state.checksum, (uint8_t*)&ret_packet.field, sizeof(ret_packet.field));
        ADD_TO_CHECKSUM(frame);
        ADD_TO_CHECKSUM(command);
        ADD_TO_CHECKSUM(autocontinue);
        ADD_TO_CHECKSUM(param1);
        ADD_TO_CHECKSUM(param2);
        ADD_TO_CHECKSUM(param3);
        ADD_TO_CHECKSUM(param4);
        ADD_TO_CHECKSUM(x);
        ADD_TO_CHECKSUM(y);
        ADD_TO_CHECKSUM(z);
#undef ADD_TO_CHECKSUM
    }

    if (checksum_state.current_waypoint<checksum_state.count) {
        return;
    }

    checksum_state.state = ChecksumState::READY;
    checksum_state.last_calculate_time_ms = mission_last_change_time_ms;

    // poke the parent class to send the deferred ack, if any:
    deferred_mission_ack.opaque_id = checksum_state.checksum;
    if (deferred_mission_ack.opaque_id == 0) {
        deferred_mission_ack.opaque_id = -1;
    }
}

void MissionItemProtocol_Waypoints::update()
{
    update_checksum();
    MissionItemProtocol::update();
}

#endif  // HAL_GCS_ENABLED && AP_MISSION_ENABLED


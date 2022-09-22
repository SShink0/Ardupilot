/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: @rhythmize
 */
#pragma once

#include <string>
#include <AP_HAL/AP_HAL.h>
#include <l8w8jwt/decode.h>

class AP_AerobridgeTrustedFlight
{
public:
    AP_AerobridgeTrustedFlight();

    static AP_AerobridgeTrustedFlight *get_singleton()
    {
        return _singleton;
    }

    bool is_valid();

private:

    bool read_file(std::string &filename, std::string &filecontent);
    bool verify_token(std::string &key, std::string &token);
    std::string get_filepath(std::string &filename);

    struct l8w8jwt_decoding_params params;
    static AP_AerobridgeTrustedFlight *_singleton;
    std::string public_key_path;
    std::string token_path;
    std::string issuer = "https://id.openskies.sh/";
    std::string public_key_filename = "trusted_flight/auth_server_public_key.pem";
    std::string token_filename = "trusted_flight/aerobridge_trusted_flight.jwt.json";
    std::string basepath = HAL_BOARD_STORAGE_DIRECTORY;
};

namespace AP
{
AP_AerobridgeTrustedFlight &aerobridge_trusted_flight();
};

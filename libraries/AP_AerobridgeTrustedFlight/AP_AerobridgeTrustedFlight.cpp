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
#include <AP_Filesystem/AP_Filesystem.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_AerobridgeTrustedFlight.h"

extern const AP_HAL::HAL& hal;

AP_AerobridgeTrustedFlight::AP_AerobridgeTrustedFlight()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("Too many AerobridgeTrustedFlight modules");
        return;
    }

    public_key_path = get_filepath(public_key_filename);
    token_path = get_filepath(token_filename);

    l8w8jwt_decoding_params_init(&params);

    _singleton = this;
}

bool AP_AerobridgeTrustedFlight::is_valid()
{
    std::string public_key;
    std::string token;

    return read_file(public_key_path, public_key) && read_file(token_path, token) && verify_token(public_key, token);
}

bool AP_AerobridgeTrustedFlight::read_file(std::string &filename, std::string &filecontent)
{
    FileData *filedata = AP::FS().load_file(get_filepath(filename).c_str());

    if (filedata == nullptr) {
        hal.console->printf("Cannot load file %s\n", filename.c_str());
        return false;
    }
    if (filedata->length <= 0) {
        hal.console->printf("Empty file %s\n", filename.c_str());
        return false;
    }
    filecontent.assign(reinterpret_cast<char *>(const_cast<uint8_t *>(filedata->data)), filedata->length);
    delete filedata;
    return true;
}

bool AP_AerobridgeTrustedFlight::verify_token(std::string &key, std::string &token)
{

    params.alg = L8W8JWT_ALG_RS256;

    params.jwt = const_cast<char*>(token.c_str());
    params.jwt_length = token.size();

    params.verification_key = reinterpret_cast<unsigned char *>(const_cast<char *>(key.c_str()));
    params.verification_key_length = key.size();

    params.validate_iss = const_cast<char *>(issuer.c_str());
    params.validate_iss_length = issuer.size();

    params.validate_exp = 1;
    params.exp_tolerance_seconds = 60;

    enum l8w8jwt_validation_result validation_result;
    struct l8w8jwt_claim claims;
    struct l8w8jwt_claim* ref = &claims;
    size_t claims_count;

    int decode_result = l8w8jwt_decode(&params, &validation_result, &ref, &claims_count);

    if (decode_result == L8W8JWT_DECODE_FAILED_INVALID_TOKEN_FORMAT) {
        hal.console->printf("Aerobridge Trusted Flight: Token format invalid\n");
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Token verification failed\n");
        return false;
    }
    if (decode_result == L8W8JWT_KEY_PARSE_FAILURE) {
        hal.console->printf("Aerobridge Trusted Flight: Public Key format invalid\n");
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Token verification failed\n");
        return false;
    }
    if (validation_result == L8W8JWT_ISS_FAILURE) {
        hal.console->printf("Aerobridge Trusted Flight: Invalid issuer\n");
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Token verification failed\n");
        return false;
    }
    if (validation_result == L8W8JWT_EXP_FAILURE) {
        hal.console->printf("Aerobridge Trusted Flight: Token expired\n");
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Token verification failed\n");
        return false;
    }

    if (decode_result != L8W8JWT_SUCCESS || validation_result != L8W8JWT_VALID) {
        hal.console->printf("Aerobridge Trusted Flight: Failed: decode_result: %d, validation_result: %d\n", decode_result, validation_result);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Token verification failed\n");
        return false;
    }
    hal.console->printf("Aerobridge Trusted Flight: Token verification successful\n");
    return true;
}

std::string AP_AerobridgeTrustedFlight::get_filepath(std::string &filename)
{
    std::string filepath = basepath + "/" + filename;
    return filepath;
}

AP_AerobridgeTrustedFlight *AP_AerobridgeTrustedFlight::_singleton;

namespace AP
{
AP_AerobridgeTrustedFlight &aerobridge_trusted_flight()
{
    return *AP_AerobridgeTrustedFlight::get_singleton();
}
};

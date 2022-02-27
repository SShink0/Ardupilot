#pragma once

#include <string>
#include <AP_HAL/AP_HAL.h>
#include <l8w8jwt/decode.h>

class AP_AerobridgeGuardian {
public:
    AP_AerobridgeGuardian();

    static AP_AerobridgeGuardian *get_singleton() {
        return _singleton;
    }

    bool is_valid();

private:

    bool read_file(std::string &filename, std::string &filecontent);
    bool verify_token(std::string &key, std::string &token);
    std::string get_filepath(std::string &filename);

    struct l8w8jwt_decoding_params params;    
    static AP_AerobridgeGuardian *_singleton;
    std::string public_key_path;
    std::string token_path;
    std::string issuer = "auth0";
    std::string public_key_filename = "RSAKey.pem";
    std::string token_filename = "token";
    std::string basepath = HAL_BOARD_STORAGE_DIRECTORY;
};

namespace AP {
    AP_AerobridgeGuardian &aerobridge_guardian();
};

#include <AP_Filesystem/AP_Filesystem.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_AerobridgeGuardian.h"

extern const AP_HAL::HAL& hal;

AP_AerobridgeGuardian::AP_AerobridgeGuardian(){
    if (_singleton != nullptr) {
        AP_HAL::panic("Too many AerobridgeGuardian modules");
        return;
    }

    public_key_path = get_filepath(public_key_filename);
    token_path = get_filepath(token_filename);
    
    l8w8jwt_decoding_params_init(&params);

    _singleton = this;
}

bool AP_AerobridgeGuardian::is_valid(){
    hal.console->printf(">>>> checking validity\n");
    gcs().send_text(MAV_SEVERITY_CRITICAL, ">>>> checking validity\n");

    std::string public_key;
    std::string token;

    return read_file(public_key_path, public_key) && read_file(token_path, token) && verify_token(public_key, token);
}

bool AP_AerobridgeGuardian::read_file(std::string &filename, std::string &filecontent) {
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
/*
struct l8w8jwt_decoding_params
{
    int alg;
    
    char* jwt;
    size_t jwt_length;

    unsigned char* verification_key;
    size_t verification_key_length;
    
    char* validate_iss;
    size_t validate_iss_length;
    
    char* validate_sub;
    size_t validate_sub_length;
    
    char* validate_aud;
    size_t validate_aud_length;
    
    char* validate_jti;
    size_t validate_jti_length;
    
    int validate_exp;
    int validate_nbf;
    int validate_iat;
    uint8_t exp_tolerance_seconds;
    uint8_t nbf_tolerance_seconds;
    uint8_t iat_tolerance_seconds;
    
    char* validate_typ;
    size_t validate_typ_length;
};
*/
bool AP_AerobridgeGuardian::verify_token(std::string &key, std::string &token) {

    hal.console->printf(">>> verifying token\n");
    params.alg = L8W8JWT_ALG_RS256;

    params.jwt = const_cast<char*>(token.c_str());
    params.jwt_length = token.size();

    params.verification_key = reinterpret_cast<unsigned char*>(const_cast<char*>(key.c_str()));
    params.verification_key_length = key.size();

    enum l8w8jwt_validation_result validation_result;
    struct l8w8jwt_claim claims;
    struct l8w8jwt_claim* ref = &claims;
    size_t claims_count;
    
    int decode_result = l8w8jwt_decode(&params, &validation_result, &ref, &claims_count);

    printf("###############################################\n");
    printf(">>> Decode result: %d | Validation result: %d\n", decode_result, validation_result);
    printf(">>> Claims count: %lu\n", claims_count);
    for (struct l8w8jwt_claim* claim = ref; claim < ref + claims_count; ++claim)
    {
        printf("%s -> %s\n", claim->key, claim->value);
    }
    printf("###############################################\n");
    
    return decode_result == L8W8JWT_SUCCESS && validation_result == L8W8JWT_VALID;
}

std::string AP_AerobridgeGuardian::get_filepath(std::string &filename) {
    std::string filepath = basepath + "/" + filename; 
    return filepath;
}

AP_AerobridgeGuardian *AP_AerobridgeGuardian::_singleton;

namespace AP {
    AP_AerobridgeGuardian &aerobridge_guardian() {
        return *AP_AerobridgeGuardian::get_singleton();
    }
};

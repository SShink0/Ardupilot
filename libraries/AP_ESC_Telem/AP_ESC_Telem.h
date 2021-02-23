#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_ESC_Telem_Backend.h"

#if HAL_WITH_ESC_TELEM

#define ESC_MAX_BACKENDS 6
#define ESC_TELEM_MAX_ESCS   12
#define ESC_TELEM_DATA_TIMEOUT_MS 5000

class AP_ESC_Telem {
public:
    friend class AP_ESC_Telem_Backend;

    AP_ESC_Telem();

    /* Do not allow copies */
    AP_ESC_Telem(const AP_ESC_Telem &other) = delete;
    AP_ESC_Telem &operator=(const AP_ESC_Telem&) = delete;

    static AP_ESC_Telem *get_singleton();

    // get an individual ESC's rpm if available, returns true on success
    bool get_rpm(uint8_t esc_index, float& rpm) const;

    // get an individual ESC's temperature in degrees if available, returns true on success
    bool get_temperature(uint8_t esc_index, int16_t& temp) const;

    // get an individual motor's temperature in degrees if available, returns true on success
    bool get_motor_temperature(uint8_t esc_index, int16_t& temp) const;

    // get an individual ESC's current in centi-amps if available, returns true on success
    bool get_current_ca(uint8_t esc_index, uint16_t& amps_ca) const;

    // get an individual ESC's usage time in seconds if available, returns true on success
    bool get_usage_seconds(uint8_t esc_index, uint32_t& usage_sec) const;

    // get an individual ESC's voltage in centi-volts if available, returns true on success
    bool get_voltage_cv(uint8_t esc_index, uint16_t& volts_cv) const;

    // get an individual ESC's consumption in mah if available, returns true on success
    bool get_consumption_mah(uint8_t esc_index, uint16_t& consumption) const;

    // return the average motor frequency in Hz for dynamic filtering
    float get_average_motor_frequency_hz() const;

    // return all of the motor frequencies in Hz for dynamic filtering
    uint8_t get_motor_frequencies_hz(uint8_t nfreqs, float* freqs) const;

    // get the number of valid ESCs
    uint8_t get_num_active_escs() const;

    // return the last time telemetry data was received in ms for the given ESC or 0 if never
    uint32_t get_last_telem_data_ms(uint8_t esc_index) const {
        if (esc_index > ESC_TELEM_MAX_ESCS) return 0;
        return _telem_data[esc_index].last_update_ms;
    }

    // send telemetry data to mavlink
    void send_esc_telemetry_mavlink(uint8_t mav_chan);

    // load backend drivers
    bool add_backend(AP_ESC_Telem_Backend *backend);

    // udpate at 10Hz to log telemetry
    void update();

    void init();

private:
    // callback to update the rpm in the frontend, should be called by the driver when new data is available
    void update_rpm(uint8_t esc_index, uint16_t new_rpm, float error_rate);
    // callback to update the data in the frontend, should be called by the driver when new data is available
    void update_telem_data(uint8_t esc_index, const AP_ESC_Telem_Backend::TelemetryData& new_data, uint8_t data_mask);

    AP_ESC_Telem_Backend* _backends[ESC_MAX_BACKENDS];

    uint8_t _backend_count;
    bool _initialised;

    // rpm data
    volatile AP_ESC_Telem_Backend::RpmData _rpm_data[ESC_TELEM_MAX_ESCS];
    // telemetry data
    volatile AP_ESC_Telem_Backend::TelemetryData _telem_data[ESC_TELEM_MAX_ESCS];

    uint32_t _last_telem_log_ms[ESC_TELEM_MAX_ESCS];

    static AP_ESC_Telem *_singleton;
};

namespace AP {
    AP_ESC_Telem &esc_telem();
};

#endif

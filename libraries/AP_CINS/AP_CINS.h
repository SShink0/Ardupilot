/*
  CINS state estimator for AP_AHRS
 */

#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_Math/LieGroups.h>
#include <AP_Common/Location.h>

class AP_CINS {
public:
    void init(void);
    void update();

    Vector3f get_accel() const {
        return state.accel.tofloat();
    }
    Vector3f get_gyro() const {
        return state.gyro.tofloat();
    }
    Quaternion get_quat() const {
        QuaternionF quat;
        quat.from_rotation_matrix(state.XHat.rot());
        return quat.tofloat();
    }
    Location get_location() const {
        Location loc = state.origin;
        loc.offset(state.XHat.pos().x, state.XHat.pos().y);
        loc.alt -= state.XHat.pos().z * 100;
        return loc;
    }
    Vector3f get_velocity() const {
        return state.XHat.vel().tofloat();
    }
    bool healthy(void) const {
        return state.have_origin;
    }
    bool get_origin(Location &loc) {
        loc = state.origin;
        return state.have_origin;
    }

private:
    void update_imu(const Vector3F &gyro_rads, const Vector3F &accel_mss, const ftype dt);
    void update_gps(const Vector3F &pos, const Vector3F &vel, const ftype gps_dt);
    void update_states_gps(const Vector3F &pos, const Vector3F &vel, const ftype dt);
    void update_states_gps_cts(const Vector3F &pos, const Vector3F &vel, const ftype dt);
    void update_attitude_from_compass();
    bool init_yaw(void);
    bool get_compass_yaw(ftype &yaw_rad, ftype &dt);
    bool get_compass_vector(Vector3F &mag_vec, Vector3F &mag_ref, ftype &dt);

    Vector3F compute_bias_update_gps(const SIM23& Gamma, const Vector3F& pos_tru, const Vector3F& vel_tru, const ftype& dt);
    Vector3F compute_bias_update_compass(const Vector3F& mag_tru, const Vector3F& mag_ref, const ftype& dt);
    void compute_bias_update_imu(const SIM23& Gamma);
    void saturate_bias(Vector3F& bias_correction, const ftype& dt) const;

    struct {
        Vector3F accel;
        Vector3F gyro;
        Location origin;
        bool have_origin;

        //XHat and ZHat for CINS
        Gal3F XHat; // State estimate containing attitude, velocity, position
        SIM23 ZHat; // Auxiliary state for estimator maths

        // Gyro Bias
        Vector3F gyro_bias;
        struct {
            Matrix3F rot;
            Matrix3F vel;
            Matrix3F pos;
        } bias_gain_mat;
    } state;

    uint32_t last_gps_update_ms;
    bool done_yaw_init;
    uint32_t last_mag_us;
};

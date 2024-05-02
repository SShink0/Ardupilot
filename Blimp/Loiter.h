#pragma once

class Vector4b
{
public:
    bool    x;
    bool    y;
    bool    z;
    bool    yaw;

    constexpr Vector4b()
        : x(0)
        , y(0)
        , z(0)
        , yaw(0) {}

    constexpr Vector4b(const bool x0, const bool y0, const bool z0, const bool yaw0)
        : x(x0)
        , y(y0)
        , z(z0)
        , yaw(yaw0) {}

    Vector4b operator &&(const Vector4b &v)
    {
        Vector4b temp{x && v.x, y && v.y, z && v.z, yaw && v.yaw};
        return temp;
    }

    Vector4b operator ||(const Vector4b &v)
    {
        Vector4b temp{x || v.x, y || v.y, z || v.z, yaw || v.yaw};
        return temp;
    }

};



class Loiter
{
public:
    friend class Blimp;
    friend class Fins;

    float scaler_x;
    float scaler_y;
    float scaler_z;
    float scaler_yaw;

    //constructor
    Loiter(uint16_t loop_rate)
    {
        scaler_x = 1;
        scaler_y = 1;
        scaler_z = 1;
        scaler_yaw = 1;
        AP_Param::setup_object_defaults(this, var_info);
    };

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo        var_info[];

    // Vel & pos PIDs
    //p, i, d, ff, imax, filt_T_hz, filt_E_hz, filt_D_hz
    AC_PID pid_vel_x{3, 0.2, 0, 0, 0.2, 0, 0, 0};
    AC_PID pid_vel_y{3, 0.2, 0, 0, 0.2, 0, 0, 0};
    AC_PID pid_vel_z{7, 1.5, 0, 0, 1, 0, 0, 0};
    AC_PID pid_vel_yaw{3, 0.4, 0, 0, 0.2, 0, 0, 0};

    AC_PID pid_pos_x{1, 0.05, 0, 0, 0.1, 0, 0, 0};
    AC_PID pid_pos_y{1, 0.05, 0, 0, 0.1, 0, 0, 0};
    AC_PID pid_pos_z{0.7, 0,   0, 0, 0, 0, 0, 0};
    AC_PID pid_pos_yaw{1.2, 0.5, 0, 0, 2, 0, 0, 0};

    AP_Float            rp_damp_lim;
    AP_Float            rp_damp_off;
    AP_Float            rp_damp_amt;
    AP_Float            rp_damp_lim2;
    AP_Float            rp_damp_amt2;
    AP_Float            rp_damp_off2;
    AP_Int16            rp_damp_msk;
    AP_Float            scaler_spd;
    AP_Float            pos_lag;
    AP_Float            bat_mult;
    AP_Float            bat_off;

    //Run Loiter controller with target position and yaw in global frame. Expects to be called at loop rate.
    void run(Vector3f& target_pos, float& target_yaw, Vector4b axes_disabled);
    //Run Loiter controller with target velocity and yaw velocity in global frame. Expects to be called at loop rate.
    void run_vel(Vector3f& target_vel, float& target_vel_yaw, Vector4b axes_disabled, bool log);
};

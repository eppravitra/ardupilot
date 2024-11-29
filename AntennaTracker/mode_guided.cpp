#include "mode.h"

#include "Tracker.h"
#include <GCS_MAVLink/GCS.h>

void ModeGuided::update()
{
    static uint32_t last_debug;
    const uint32_t now = AP_HAL::millis();
    float target_roll, target_pitch, target_yaw;
    _target_att.to_euler(target_roll, target_pitch, target_yaw);
    if (now - last_debug > 5000) {
        last_debug = now;
        gcs().send_text(MAV_SEVERITY_INFO, "target_yaw=%f target_pitch=%f", degrees(target_yaw), degrees(target_pitch));
    }

    float yaw_error,bf_pitch_cmd;

    update_command(target_yaw,target_pitch,&yaw_error,&bf_pitch_cmd);

    // update actuators
    tracker.update_yaw_cr_servo(degrees(yaw_error)*100.0f);
    tracker.update_pitch_pos_cmd_servo(degrees(bf_pitch_cmd));

    if (now - last_debug > 500) {
        last_debug = now;
        gcs().send_text(MAV_SEVERITY_INFO, "yaw_error=%f", degrees(yaw_error));
    }

    //logging
    Parameters &g = tracker.g;
    g.pidYaw2Srv.set_target_rate(degrees(target_yaw));
    g.pidYaw2Srv.set_actual_rate(degrees(AP::ahrs().get_yaw()));
    g.pidPitch2Srv.set_target_rate(degrees(bf_pitch_cmd));
    g.pidPitch2Srv.set_actual_rate(degrees(AP::ahrs().get_pitch()));

    /*   
    calc_angle_error(degrees(target_pitch)*100, degrees(target_yaw)*100, false);
    float bf_pitch;
    float bf_yaw;
    convert_ef_to_bf(target_pitch, target_yaw, bf_pitch, bf_yaw);
    tracker.update_pitch_servo(bf_pitch);
    tracker.update_yaw_servo(bf_yaw);
*/
}

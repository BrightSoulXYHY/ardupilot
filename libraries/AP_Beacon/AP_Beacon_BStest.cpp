#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_Beacon_BStest.h"
#include <ctype.h>
#include <stdio.h>

AP_Beacon_BStest::AP_Beacon_BStest(AP_Beacon &frontend) : AP_Beacon_Backend(frontend)
{
    _last_request_setting_ms = AP_HAL::millis();
}

bool AP_Beacon_BStest::healthy() { return true; }

void AP_Beacon_BStest::update(void)
{
    // set_beacon_distance(0, 3.14159f);
    if (AP_HAL::millis() - _last_request_setting_ms > 2000)
    {
        _last_request_setting_ms = AP_HAL::millis();
        _dis += 2.0f;
        _angel += 3.0f;
    }
}

void AP_Beacon_BStest::get_data(float &dis, float &angel)
{
    dis=_dis;
    angel=_angel;
}
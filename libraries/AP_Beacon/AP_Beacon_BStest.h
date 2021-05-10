#pragma once

#include "AP_Beacon_Backend.h"


class AP_Beacon_BStest : public AP_Beacon_Backend
{

public:
    // constructor
    AP_Beacon_BStest(AP_Beacon &frontend);

    // return true if sensor is basically healthy (we are receiving data)
    bool healthy() override;

    // update
    void update() override;

    void get_data(float &dis, float &angel) override;

    float _dis = 0.0f, _angel = 0.0f;
    uint32_t _last_request_setting_ms;
};


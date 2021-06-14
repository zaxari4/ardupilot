#pragma once

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_Serial.h"
#include "AP_RangeFinder_Params.h"

class AP_RangeFinder_FM24 : public AP_RangeFinder_Backend_Serial
{

public:

    using AP_RangeFinder_Backend_Serial::AP_RangeFinder_Backend_Serial;

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_RADAR;
    }

    uint32_t initial_baudrate(uint8_t serial_instance) const override {
        return 57600;
    }

    uint16_t rx_bufsize() const override { return 128; }
    uint16_t tx_bufsize() const override { return 128; }

private:

    bool get_reading(uint16_t &reading_cm) override;

    uint8_t _linebuf[8];
    uint8_t _linebuf_len;
};

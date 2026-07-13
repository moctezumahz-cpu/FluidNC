// Copyright (c) 2025 - MCHTec
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

/*
	Corner detection for plasma THC anti-dive.
	
	Monitors the actual feedrate against the programmed rate.
	When actual speed drops below threshold% of programmed, Corner pin activates.
	This tells the THC to stop height tracking (anti-dive) during corners/circles.
*/

#include "../Pin.h"
#include "../Configuration/Configurable.h"

class CornerConfig : public Configuration::Configurable {
public:
    CornerConfig() = default;

    void group(Configuration::HandlerBase& handler) override {
        handler.item("pin", _pin);
        handler.item("threshold_pct", _threshold_pct, 50, 90);
    }

    void init() {
        if (!_pin.undefined()) {
            _pin.setAttr(Pin::Attr::Output);
            _pin.off();
        }
    }

    // Called each time prep_buffer() generates a segment with updated speed
    // actual_speed_mmmin: current composite feedrate from the motion
    // programmed_rate_mmmin: programmed feedrate (F word)
    void check_speed(float actual_speed_mmmin, float programmed_rate_mmmin) {
        if (_pin.undefined()) return;
        if (programmed_rate_mmmin < 0.001f) return;  // rapid or invalid

        bool corner = (actual_speed_mmmin / programmed_rate_mmmin) < (_threshold_pct / 100.0f);
        _pin.write(corner);
    }

    void deinit() {
        if (!_pin.undefined()) {
            _pin.off();
            _pin.setAttr(Pin::Attr::Input);
        }
    }

    bool has_pin() const { return !_pin.undefined(); }

protected:
    Pin _pin;
    int _threshold_pct = 70;  // 70% default
};

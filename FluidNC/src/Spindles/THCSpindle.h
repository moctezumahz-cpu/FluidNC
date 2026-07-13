// Copyright (c) 2025 - MCHTec
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

/*
    THC Spindle — Torch Height Controller interface for plasma cutting.
    
    M3 → START pin HIGH, espera READY del THC (obligatorio)
    M5 → START pin LOW
    
    También maneja el corner detection para anti-dive:
    - corner_pin: gpio.38 (salida, HIGH cuando detecta esquina)
    - threshold_corner: 70 (porcentaje de feedrate para considerar esquina)
    
    ready_pin es obligatorio. El timeout es solo safety net.
*/

#include "Spindle.h"
#include "../Configuration/GenericFactory.h"

namespace Spindles {
    class THC : public Spindle {
    public:
        THC() = default;

        THC(const THC&) = delete;
        THC(THC&&)      = delete;
        THC& operator=(const THC&) = delete;
        THC& operator=(THC&&) = delete;

        void init() override;
        void setState(SpindleState state, SpindleSpeed speed) override;
        void setSpeedfromISR(uint32_t dev_speed) override;
        void config_message() override;

        // Corner detection para anti-dive
        void corner_check(float actual_speed, float programmed_rate);

        void group(Configuration::HandlerBase& handler) override {
            handler.item("start_pin", _start_pin);
            handler.item("ready_pin", _ready_pin);
            handler.item("pierce_timeout_ms", _pierce_timeout_ms, 500, 30000);
            handler.item("corner_pin", _corner_pin);
            handler.item("threshold_corner", _threshold_corner, 50, 90);
            Spindle::group(handler);
        }

        const char* name() const override { return "THC"; }

        virtual ~THC() {}

    protected:
        Pin _start_pin;          // START signal to THC
        Pin _ready_pin;          // READY signal from THC
        int _pierce_timeout_ms = 5000;
        Pin _corner_pin;         // Corner detection output
        int _threshold_corner = 70;
    };
}

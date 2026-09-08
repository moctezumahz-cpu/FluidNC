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

    Manejo de errores:
    - M3 sin aire (presostato): interlock — START no se enciende, alarma (air_lost)
    - M3 sin READY tras pierce_timeout_ms: alarma (ready_timeout)
    - READY lost durante corte (arc loss) con debounce anti-EMI arc_loss_debounce_ms
    - error_pin: entrada desde THC, HIGH = error
    - poll() se llama desde Stepper::prep_buffer() para monitoreo periódico
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
        void corner_check(float actual_speed, float programmed_rate) override;

        // Poll periódico: monitoreo READY + errores RS-485/THC
        void poll() override;

        // Trigger de error: apaga todo y dispara alarm
        void trigger_error();

        // Estado para telemetria del bridge ($THC/Status, ER 4/5/6/7)
        bool get_ready_lost()       const { return _ready_lost; }
        bool get_air_lost()         const { return _air_lost; }        // ER 6: baja presion de aire
        bool get_ready_timeout()    const { return _ready_timeout; }   // ER 7: M3 sin READY (sin arco)
        bool get_error_pin_active() const { return !_error_pin.undefined() && _error_pin.read() == Pin::On; }

        void group(Configuration::HandlerBase& handler) override {
            handler.item("start_pin", _start_pin);
            handler.item("ready_pin", _ready_pin);
            handler.item("pierce_timeout_ms", _pierce_timeout_ms, 500, 30000);
            handler.item("arc_loss_debounce_ms", _arc_loss_debounce_ms, 50, 5000);
            handler.item("corner_pin", _corner_pin);
            handler.item("threshold_corner", _threshold_corner, 50, 90);
            handler.item("error_pin", _error_pin);
            Spindle::group(handler);
        }

        const char* name() const override { return "THC"; }

        virtual ~THC() {}

    protected:
        Pin _start_pin;          // START signal to THC
        Pin _ready_pin;          // READY signal from THC
        int _pierce_timeout_ms = 5000;
        int _arc_loss_debounce_ms = 200;  // READY debe permanecer Off este tiempo para alarmar (anti-EMI)
        Pin _corner_pin;         // Corner detection output
        int _threshold_corner = 70;
        Pin _error_pin;          // Error input from THC (opcional)

        // Flags de error (volatile por acceso desde distintos contextos)
        volatile bool _ready_lost       = false;
        volatile bool _air_lost         = false;  // baja presion de aire (interlock M3 / durante corte)
        volatile bool _ready_timeout    = false;  // M3 expiró sin READY (sin arco)
        volatile bool _error_triggered  = false;
        bool _ready_was_high            = false;  // READY se afirmó alguna vez
        uint32_t _arc_loss_timer        = 0;      // millis() de inicio del debounce de arc loss (0 = no corriendo)
    };
}

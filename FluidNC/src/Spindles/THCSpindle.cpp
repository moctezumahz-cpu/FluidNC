#include "THCSpindle.h"

#include "../System.h"
#include "../Report.h"
#include "../Logging.h"
#include "../Machine/MachineConfig.h"
#include "../Protocol.h"

namespace Spindles {

    void THC::init() {
        log_warn("THC: init() called — spindle is active");
        if (_start_pin.undefined()) {
            log_error("THC: start_pin required — spindle NOT initialized");
            return;
        }
        if (_ready_pin.undefined()) {
            log_error("THC: ready_pin required — THC always provides READY");
            return;
        }
        log_warn("THC: pins OK — start=" << _start_pin.name() << " ready=" << _ready_pin.name());

        _start_pin.setAttr(Pin::Attr::Output);
        _start_pin.off();

        _ready_pin.setAttr(Pin::Attr::Input);

        if (!_corner_pin.undefined()) {
            _corner_pin.setAttr(Pin::Attr::Output);
            _corner_pin.off();
            log_warn("THC: corner pin configured — " << _corner_pin.name());
        }

        if (_speeds.size() == 0) {
            linearSpeeds(300, 100.0f);  // S = volts
        }
        setupSpeeds(1);
        config_message();
    }

    void THC::config_message() {
        log_info("THC Spindle Start:" << _start_pin.name() << " Ready:" << _ready_pin.name()
                                      << " Timeout:" << _pierce_timeout_ms << "ms");
    }

    void THC::setState(SpindleState state, SpindleSpeed speed) {
        if (sys.abort) return;

        // Forward S value as Vsetpoint to THC bridge
        if (config->_thc) {
            config->_thc->set_vsetpoint((int)speed);
        }

        if (state == SpindleState::Disable) {
            // M5
            if (!_corner_pin.undefined()) {
                _corner_pin.off();
            }
            if (config->_thc) {
                config->_thc->set_start(false);
            }
            _start_pin.off();
            return;
        }

        // M3: START al THC + notify bridge
        if (config->_thc) {
            config->_thc->set_start(true);
        }
        _start_pin.on();

        // Esperar READY del THC con timeout
        // Si el pin READY (gpio.39) se pone HIGH, sale al instante
        int remaining = _pierce_timeout_ms;
        while (remaining > 0) {
            if (sys.abort) {
                _start_pin.off();
                return;
            }
            if (_ready_pin.read() == Pin::On) {
                log_info("THC: READY after " << (_pierce_timeout_ms - remaining) << "ms");
                break;
            }
            protocol_execute_realtime();
            vTaskDelay(pdMS_TO_TICKS(10));
            remaining -= 10;
        }
        if (remaining <= 0) {
            log_warn("THC: READY timeout " << _pierce_timeout_ms << "ms — apagando START");
            _start_pin.off();
        }
    }

    // Corner detection para anti-dive
    void THC::corner_check(float actual_speed, float programmed_rate) {
        if (_corner_pin.undefined()) return;
        // Sentinel: -1 = force OFF
        if (actual_speed < 0) {
            _corner_pin.off();
            return;
        }
        if (programmed_rate < 0.001f) return;
        bool corner = (actual_speed / programmed_rate) < (_threshold_corner / 100.0f);
        _corner_pin.write(corner);
    }

    void IRAM_ATTR THC::setSpeedfromISR(uint32_t dev_speed) {
        // No-op: START/READY son manejados en setState
    }

    // NOTA: Debe estar fuera del namespace anónimo para que LTO no lo elimine
    __attribute__((used))
    SpindleFactory::InstanceBuilder<THC> thc_registration("THCSpindle");

    // Forzar linker a incluir este objeto (autoregistro del factory)
    // Variable volátil referenciada desde MachineConfig
    volatile int _thc_spindle_linked = 0;
    void __force_thcspindle_link() {
        _thc_spindle_linked = 1;
    }
}

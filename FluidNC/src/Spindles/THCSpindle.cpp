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

        if (!_error_pin.undefined()) {
            _error_pin.setAttr(Pin::Attr::Input);
            log_warn("THC: error pin configured — " << _error_pin.name());
        }

        // Resetear flags de error
        _ready_lost       = false;
        _error_triggered  = false;
        _ready_was_high   = false;
        _current_state    = SpindleState::Unknown;

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
            // M5: apagar todo
            if (!_corner_pin.undefined()) {
                _corner_pin.off();
            }
            if (config->_thc) {
                config->_thc->set_start(false);
            }
            _start_pin.off();
            _current_state   = SpindleState::Disable;
            _ready_lost      = false;
            _error_triggered = false;
            _ready_was_high  = false;
            return;
        }

        // M3: si ya estamos en Cw (M3 duplicado), hacer M5 implícito y luego M3 de nuevo
        if (_current_state == SpindleState::Cw) {
            log_warn("THC: M3 duplicado — haciendo M5+M3 implícito");
            // M5 implícito
            if (!_corner_pin.undefined()) {
                _corner_pin.off();
            }
            if (config->_thc) {
                config->_thc->set_start(false);
            }
            _start_pin.off();
            // Seguir con M3 abajo (sin tocar flags aún)
        }

        // Reset flags para nuevo M3
        _ready_lost       = false;
        _error_triggered  = false;
        _ready_was_high   = false;

        // M3: START al THC + notify bridge
        if (config->_thc) {
            config->_thc->set_start(true);
        }
        _start_pin.on();

        // Esperar READY del THC con timeout
        int remaining = _pierce_timeout_ms;
        while (remaining > 0) {
            if (sys.abort) {
                _start_pin.off();
                _current_state = SpindleState::Disable;
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
            log_warn("THC: READY timeout " << _pierce_timeout_ms << "ms — apagando START, queda esperando");
            _start_pin.off();
            // El spindle queda "activo" pero sin START — se queda en loop de espera
            _current_state = SpindleState::Cw;
            return;
        }

        // READY recibido — entramos en modo corte
        _ready_was_high  = true;
        _current_state   = SpindleState::Cw;
    }

    // ── Poll periódico: monitoreo durante el corte ──
    void THC::poll() {
        // Solo monitorear si estamos en modo corte
        if (_current_state != SpindleState::Cw) return;
        if (_error_triggered) return;  // Ya en error

        // 1. READY lost durante corte
        if (_ready_was_high && _ready_pin.read() == Pin::Off) {
            log_error("THC: READY lost during cut — plasma extinguished");
            _ready_lost = true;
            trigger_error();
            return;
        }

        // 2. Error pin desde THC (error_pin = HIGH = error)
        if (!_error_pin.undefined() && _error_pin.read() == Pin::On) {
            log_error("THC: error pin asserted by THC");
            trigger_error();
            return;
        }

        // 3. Error desde RS-485 (campo Err del THC)
        if (config->_thc) {
            if (config->_thc->get_error()) {
                log_error("THC: error reported by THC-MCH (Err=1)");
                trigger_error();
                return;
            }
            if (config->_thc->comm_lost()) {
                log_error("THC: RS-485 communication lost (2s timeout)");
                trigger_error();
                return;
            }
        }
    }

    // ── Trigger de error: apaga todo y dispara alarm ──
    void THC::trigger_error() {
        if (_error_triggered) return;  // Solo una vez
        _error_triggered = true;

        // START off
        _start_pin.off();

        // Corner off
        if (!_corner_pin.undefined()) {
            _corner_pin.off();
        }

        // Bridge: start=false
        if (config->_thc) {
            config->_thc->set_start(false);
        }

        // Disparar alarm del sistema FluidNC
        rtAlarm = ExecAlarm::SpindleControl;
    }

    // Corner detection para anti-dive
    void THC::corner_check(float actual_speed, float programmed_rate) {
        if (_corner_pin.undefined()) return;
        // Sentinel: -1 = force OFF
        if (actual_speed < 0) {
            _corner_pin.off();
            return;
        }
        // Si hay error, corner siempre OFF
        if (_error_triggered || _ready_lost) {
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

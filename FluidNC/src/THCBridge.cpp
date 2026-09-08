#include "THCBridge.h"
#include "Logging.h"
#include "Machine/MachineConfig.h"
#include "Spindles/THCSpindle.h"  // get_ready_lost()/get_error_pin_active() para error_code()

#include <HardwareSerial.h>   // Serial, Serial1, Serial2
#include <string.h>

// Map uart_num to HardwareSerial reference (0=USB CDC, not used for RS-485)
static HardwareSerial& uart_for_num(int n) {
    switch (n) {
        case 1: return Serial1;
        case 2: return Serial2;
        default: return Serial1;
    }
}

THCBridge::THCBridge() {}

THCBridge::~THCBridge() {
    deinit();
}

void THCBridge::init() {
    if (_txd_pin.undefined() || _rxd_pin.undefined()) {
        log_info("THC Bridge: no pins configured, disabled");
        return;
    }

    auto& ser = uart_for_num(_uart_num);
    ser.begin(_baud, SERIAL_8N1, _rxd_pin.getNative(Pin::Capabilities::Input), _txd_pin.getNative(Pin::Capabilities::Output));

    if (!_re_pin.undefined()) {
        _re_pin.setAttr(Pin::Attr::Output);
        _re_pin.off();  // receive mode
    }

    if (!_air_pin.undefined()) {
        _air_pin.setAttr(Pin::Attr::Input);
        log_info("THC Bridge: air_pin " << _air_pin.name() << " (presostato, HIGH = presion OK)");
    }

    if (_corner) {
        _corner->init();
    }

    _running = true;
    xTaskCreatePinnedToCore(
        task_loop, "thc_bridge", 4096, this, 5, &_task, 1
    );

    log_info("THC Bridge: UART" << _uart_num << " @" << _baud << " RS-485");
}

void THCBridge::deinit() {
    _running = false;
    if (_task) {
        vTaskDelete(_task);
        _task = nullptr;
    }
    auto& ser = uart_for_num(_uart_num);
    ser.end();
    if (!_re_pin.undefined()) {
        _re_pin.setAttr(Pin::Attr::Input);
    }
}

void THCBridge::set_vsetpoint(int volts) {
    _params.vsetpoint = constrain(volts, 50, 250);
}

void THCBridge::set_ihs(int steps) {
    _params.ihs = constrain(steps, 0, 19900);
}

void THCBridge::set_start_delay(int ms) {
    _params.start_delay = constrain(ms, 0, 5000);
}

void THCBridge::set_start(bool on) {
    _params.start_virtual = on;
}

// ── Telemetria de estado ($THC/Status) ──

bool THCBridge::air_ok() const {
    if (_air_pin.undefined()) return true;  // sin pin configurado -> no bloquea, OK
    return _air_pin.read() == Pin::On;      // HIGH = presion de aire OK
}

int THCBridge::error_code() const {
    // 0=sin error, 1=comm lost RS-485, 2=colision, 3=bit error del THC,
    // 4=pin ERROR del spindle, 5=READY perdido (spindle)
    if (comm_lost()) return 1;
    if (get_collision()) return 2;
    if (get_error()) return 3;
    if (_spindle && _spindle->get_error_pin_active()) return 4;
    if (_spindle && _spindle->get_ready_lost()) return 5;
    return 0;
}

// ── RS-485 half-duplex helpers ──

void THCBridge::rs485_write(const uint8_t* data, size_t len) {
    if (!_re_pin.undefined()) {
        _re_pin.on();  // transmit mode
        delayMicroseconds(300);
    }
    auto& ser = uart_for_num(_uart_num);
    ser.write(data, len);
    ser.flush();
    if (!_re_pin.undefined()) {
        delayMicroseconds(300);
        _re_pin.off();  // receive mode
    }
}

String THCBridge::rs485_read_line() {
    auto& ser = uart_for_num(_uart_num);
    String line;
    int timeout = 50;  // ~5 chars at 19200
    while (timeout-- > 0) {
        if (ser.available()) {
            char c = ser.read();
            if (c == '\n') return line;
            if (c != '\r') line += c;
        } else {
            delay(1);
        }
    }
    return line;
}

// ── Frame protocol ──

void THCBridge::send_frame() {
    // Build 19-value frame matching THC-MCH HD protocol
    char buf[128];
    snprintf(buf, sizeof(buf),
        "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,0,%d,%d,%d,%d,%d,%d,%d,%d",
        _params.vsetpoint,
        _params.start_delay,
        _params.ihs,
        _params.speed_thc,
        _params.speed_probe,
        _params.speed_min,
        _params.probe_sense ? 1 : 0,
        _params.arranca_plasma ? 1 : 0,
        _params.transfer ? 1 : 0,
        _params.return_up ? 1 : 0,
        _params.thc_enable ? 1 : 0,
        _params.invert_probe ? 1 : 0,
        _params.invert_ready ? 1 : 0,
        _params.invert_probe_enable ? 1 : 0,
        _params.invert_dir ? 1 : 0,
        _params.up_virtual ? 1 : 0,
        _params.down_virtual ? 1 : 0,
        _params.start_virtual ? 1 : 0
    );

    rs485_write((uint8_t*)buf, strlen(buf));
    rs485_write((uint8_t*)"\n", 1);
}

void THCBridge::parse_telemetry(const char* line) {
    // Format: V:140,VsP:140,Sd:5000,IHS:2500,ES:0,Str:0,Po:0,...
    // Parse key:value pairs
    const char* p = line;
    bool any_key_matched = false;  // Para detectar trama válida

    while (*p) {
        // Skip to next key
        while (*p && !((*p >= 'A' && *p <= 'Z') || (*p >= 'a' && *p <= 'z'))) p++;
        if (!*p) break;

        const char* key_start = p;
        while (*p && *p != ':') p++;
        if (!*p) break;

        // Extract key
        char key[8] = {0};
        int klen = p - key_start;
        if (klen > 7) klen = 7;
        strncpy(key, key_start, klen);

        p++; // skip ':'

        int ival = atoi(p);

        // Skip to next value separator
        while (*p && *p != ',' && *p != ';') p++;
        if (*p == ',') p++;

        // Map key to telemetry field
        if      (strcmp(key, "V") == 0)    { _tel.voltage = ival; any_key_matched = true; }
        else if (strcmp(key, "VsP") == 0)  _tel.setpoint = ival;
        else if (strcmp(key, "ES") == 0)   _tel.sequence_state = ival;
        else if (strcmp(key, "Po") == 0)   _tel.position = ival;
        else if (strcmp(key, "Str") == 0)  _tel.start = ival;
        else if (strcmp(key, "Up") == 0)   _tel.up = ival;
        else if (strcmp(key, "Dwn") == 0)  _tel.down = ival;
        else if (strcmp(key, "Rdy") == 0)  { _tel.ready = ival; any_key_matched = true; }
        else if (strcmp(key, "Prb") == 0)  _tel.probe = ival;
        else if (strcmp(key, "Col") == 0)  _tel.collision = ival;
        else if (strcmp(key, "Cor") == 0)  _tel.corner = ival;
        else if (strcmp(key, "Lim") == 0)  _tel.lim_alto = ival;
        else if (strcmp(key, "Err") == 0)  { _tel.error = ival; any_key_matched = true; }
        else if (strcmp(key, "Diag") == 0) _tel.diag = ival;
        else if (strcmp(key, "IHS") == 0)  _tel.ihs = ival;
        else if (strcmp(key, "Sd") == 0)   _tel.start_delay = ival;

        // Arc established: sequence state >= 5 (THC tracking)
        _tel.arc_established = (_tel.sequence_state >= 5 && _tel.sequence_state < 6);
    }

    // Si se parseó al menos una clave crítica → trama válida → reset timeout
    if (any_key_matched) {
        _comm_timeout = 0;
        _comm_lost    = false;
    }
}

// ── FreeRTOS task ──

void THCBridge::task_loop(void* arg) {
    THCBridge* bridge = (THCBridge*)arg;
    while (bridge->_running) {
        bridge->send_frame();
        String line = bridge->rs485_read_line();
        if (line.length() > 0) {
            bridge->parse_telemetry(line.c_str());
        }

        // RS-485 timeout: si pasan ~2s sin trama válida, marcar pérdida
        bridge->_comm_timeout++;
        if (bridge->_comm_timeout > 20) {  // 20 ticks * 100ms = 2s
            bridge->_comm_lost = true;
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // 10 fps
    }
    vTaskDelete(NULL);
}

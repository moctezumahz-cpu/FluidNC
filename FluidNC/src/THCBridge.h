// Copyright (c) 2025 - MCHTec
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

/*
	THC RS-485 Bridge
	
	Communication loop between FluidNC and THC-MCH HD via RS-485.
	Sends config frame every 100ms, receives telemetry.
	
	Config YAML:
	  thc:
	    uart:
	      txd_pin: gpio.17
	      rxd_pin: gpio.18
	      re_pin: gpio.21    # RE/DE for RS-485 half-duplex
	    baud: 19200
	
	Access telemetry from GCode/Macros via M100 or status report.
*/

#include "Pin.h"
#include "Configuration/Configurable.h"
#include "Machine/CornerConfig.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class THCBridge : public Configuration::Configurable {
public:
    THCBridge();
    ~THCBridge();

    void group(Configuration::HandlerBase& handler) override {
        handler.item("txd_pin", _txd_pin);
        handler.item("rxd_pin", _rxd_pin);
        handler.item("re_pin", _re_pin);
        handler.item("baud", _baud, 2400, 115200);
        handler.item("uart_num", _uart_num, 1, 2);
        handler.section("corner", _corner);
    }

    void init();
    void deinit();

    // Set parameters from spindle GCode
    void set_vsetpoint(int volts);   // from S word
    void set_ihs(int steps);
    void set_start_delay(int ms);
    void set_start(bool on);         // M3/M5 toggle

    // Telemetry readout
    int  get_voltage()    const { return _tel.voltage; }
    int  get_position()   const { return _tel.position; }
    int  get_state()      const { return _tel.sequence_state; }
    bool get_ready()      const { return _tel.ready; }
    bool get_corner()     const { return _tel.corner; }
    bool get_collision()  const { return _tel.collision; }
    bool get_arc()        const { return _tel.arc_established; }
    bool get_error()      const { return _tel.error; }
    bool comm_lost()      const { return _comm_lost; }
    void reset_comm()           { _comm_timeout = 0; _comm_lost = false; }

    // Corner detection (called from Stepper::prep_buffer)
    CornerConfig* corner_config() { return _corner; }
    void          corner_check(float actual_speed, float programmed_rate) {
        if (_corner) _corner->check_speed(actual_speed, programmed_rate);
    }

protected:
    Pin _txd_pin;
    Pin _rxd_pin;
    Pin _re_pin;
    int _baud = 19200;
    int _uart_num = 1;  // 0=Serial, 1=Serial1, 2=Serial2

    CornerConfig* _corner = nullptr;

    TaskHandle_t _task = nullptr;
    bool _running = false;

    // RS-485 communication timeout
    volatile int  _comm_timeout = 0;   // ticks sin trama válida (1 tick = ~100ms)
    volatile bool _comm_lost    = false;

    // Telemetry store (read by task, read from main)
    struct {
        int  voltage          = 0;
        int  setpoint         = 140;
        int  position         = 0;
        int  sequence_state   = 0;
        bool start            = false;
        bool up               = false;
        bool down             = false;
        bool ready            = false;
        bool probe            = false;
        bool collision        = false;
        bool corner           = false;
        bool lim_alto         = false;
        bool error            = false;
        bool diag             = false;
        int  ihs              = 2500;
        int  start_delay      = 5000;
        bool arc_established  = false;
    } _tel;

    // Parameters to send (written from main, read from task)
    struct {
        int  vsetpoint    = 140;
        int  start_delay  = 5000;
        int  ihs          = 2500;
        int  speed_thc    = 2000;
        int  speed_probe  = 2000;
        int  speed_min    = 2000;
        bool probe_sense  = true;
        bool arranca_plasma = true;
        bool transfer     = true;
        bool return_up    = true;
        bool thc_enable   = true;
        bool invert_probe = false;
        bool invert_ready = false;
        bool invert_probe_enable = false;
        bool invert_dir   = false;
        bool up_virtual   = false;
        bool down_virtual = false;
        bool start_virtual = false;
    } _params;

    void send_frame();
    void parse_telemetry(const char* line);
    void rs485_write(const uint8_t* data, size_t len);
    String rs485_read_line();

    static void task_loop(void* arg);
};

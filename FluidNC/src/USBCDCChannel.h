// Copyright (c) 2026 - MCH
// Console channel over the ESP32-S3 native USB (USB-Serial/JTAG CDC).
//
// Motivo (03-oct-2026): la consola del fork vivia en UART0 = gpio43/44, y esos
// dos GPIO son justamente las redes RX/TX del RS-485 del ArcPilot en la tarjeta
// MCH-FLUID-PLASMA R3. Con la consola en el USB nativo, gpio43/44 quedan libres
// para el RS-485 y la app (PlasmaCore) puede hablar por /dev/ttyACMx.

#pragma once

#include "Channel.h"
#include "lineedit.h"

class USBCDCChannel : public Channel {
private:
    Lineedit* _lineedit;

public:
    USBCDCChannel(bool addCR = true);

    void init();  // arranca la CDC y se registra en allChannels

    // Print (salida)
    size_t write(uint8_t c) override;
    size_t write(const uint8_t* buf, size_t len) override;

    // Stream (entrada)
    int peek(void) override;
    int available(void) override;
    int read() override;

    // Channel
    int      rx_buffer_available() override;
    void     flushRx() override;
    bool     realtimeOkay(char c) override;
    bool     lineComplete(char* line, char c) override;
    Channel* pollLine(char* line) override;
};

extern USBCDCChannel USBConsole;

// Copyright (c) 2026 - MCH
// Consola por el USB nativo del ESP32-S3 (ver USBCDCChannel.h).

#include "USBCDCChannel.h"
#include "Serial.h"  // allChannels

#include <Arduino.h>

USBCDCChannel USBConsole(true);

USBCDCChannel::USBCDCChannel(bool addCR) : Channel("usb", addCR) {
    _lineedit = new Lineedit(this, _line, Channel::maxLine - 1);
}

void USBCDCChannel::init() {
    Serial.begin();
    allChannels.registration(this);
}

size_t USBCDCChannel::write(uint8_t c) {
    // Si nadie tiene el puerto abierto, HWCDC se queda esperando: mejor tirar el byte
    if (!(bool)Serial) {
        return 1;
    }
    return Serial.write(c);
}

size_t USBCDCChannel::write(const uint8_t* buffer, size_t length) {
    if (!(bool)Serial) {
        return length;
    }
    if (_addCR) {
        size_t rem      = length;
        char   lastchar = '\0';
        size_t j        = 0;
        while (rem) {
            const int bufsize = 80;
            uint8_t   modbuf[bufsize];
            size_t    k = 0;
            while (rem && k < (bufsize - 1)) {
                char c = buffer[j++];
                if (c == '\n' && lastchar != '\r') {
                    modbuf[k++] = '\r';
                }
                lastchar    = c;
                modbuf[k++] = c;
                --rem;
            }
            Serial.write(modbuf, k);
        }
        return length;
    }
    return Serial.write(buffer, length);
}

int USBCDCChannel::available() {
    return Serial.available();
}

int USBCDCChannel::peek() {
    return Serial.peek();
}

int USBCDCChannel::read() {
    return Serial.read();
}

int USBCDCChannel::rx_buffer_available() {
    return Serial.availableForWrite();
}

void USBCDCChannel::flushRx() {
    while (Serial.available()) {
        Serial.read();
    }
    Channel::flushRx();
}

bool USBCDCChannel::realtimeOkay(char c) {
    return _lineedit->realtime(c);
}

bool USBCDCChannel::lineComplete(char* line, char c) {
    if (_lineedit->step(c)) {
        _linelen        = _lineedit->finish();
        _line[_linelen] = '\0';
        strcpy(line, _line);
        _linelen = 0;
        return true;
    }
    return false;
}

Channel* USBCDCChannel::pollLine(char* line) {
    return Channel::pollLine(line);
}

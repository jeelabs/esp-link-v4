// Esp-link-v4 Serial Bridge
// Copyright (C) 2018 by Throsten von Eicken

// The SerialBridge library provides a full-duplex transparent bridge between TCP connections and a
// uart. It allows an arbitrary number of TCP connections (resources permitting...) to all receive
// characters coming in on the uart, as well as to send characters to the uart.
//
// This library is written for the esp8266 and uses the ESPAsyncTCP library. It is fixed to use the
// esp8266's only full uart (uart0/Serial).
//
// The TCP-to-Uart path uses TCP ack back-pressure to limit the amount of buffering required. The
// ESPAsyncTCP library calls handleData with a packet at a time and the SerialBridge only ACKs the
// characters as they are stuffed into the uart and buffers the remainder of the packet. As a
// result, the buffer space is a max of 4 MSS, i.e. 4*536 or 4*1460 bytes depending on the LwIP
// configuration chosen.
//
// The Uart-to-TCP path uses only the buffer in the uart driver. When there are characters to
// transmit it determines the max number that can be sent on all TCP connections, reads those from
// the buffer, and then transmits them.
//
// The main limitation of the SerialBridge is buffer size, which is constrained by the esp8266
// memory. On the TCP-to-Uart path thanks to the TCP back pressure the amount of buffering required
// is bounded. However, due to the limitations of the LwIP library it can easily lead to hiccups,
// i.e., stop&go type of flow. On the Uart-to-TCP path there is no reasonable buffer bound and it is
// easy for Wifi packet loss and other network or receiver hiccups to cause characters to be lost
// due to buffer overflow. Implementing uart flow-control could solve this...

#ifndef SerialBridge_h
#define SerialBridge_h

#include <stdlib.h>
#include "ESPAsyncTCP.h"

// SbrClient holds the state we need for one TCP client.
struct SbrClient;

struct SerialBridge {
    SerialBridge() : _overrun(false), _disabled(false), _debug(0) {}

    // begin operation of the serial bridge, the default rxBufSz provides 173ms of buffering at
    // 115200 baud
    void begin(uint16_t port=2323, uint32_t baudrate=115200, uint32_t rxBufSz=2000);
    // loop must be called from the arduino loop function to perform background tasks
    void loop();
    // debug printf function used for info/debug messages
    void debug(void dbgPrintf(const char*, ...));
    // disable turns the serial bridge off temporarily, e.g. to use the uart for something else
    void disable() { _disabled = true; }
    // enable re-enables after a disable
    void enable() { _disabled = false; }

    // private

    void handleNewClient(AsyncClient* client);
    void recvUartCheck();
    void recvTCPCheck();
    void gc();

    std::vector<SbrClient*> _clients; // a list to hold all clients
    bool _overrun; // state used to only warn once per overrun event
    bool _disabled;
    void (*_debug)(const char*, ...);
};

#endif // SerialBridge_h

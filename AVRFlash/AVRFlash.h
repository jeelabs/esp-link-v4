// Copyright (c) 2015-2018 by Thorsten von Eicken

// Protocol used : https://github.com/Optiboot/optiboot/wiki/HowOptibootWorks

#ifndef AVRFlash_h
#define AVRFlash_h

#include <ESPAsyncWebServer.h>
#include <Ticker.h>
#include "HexRecord.h"

#define RESP_SZ 64

enum AVRProgStates {     // overall programming states
  stateInit = 0,             // initial delay
  stateSync,                 // waiting to hear back
  stateGetSig,               // reading device signature
  stateGetVersLo,            // reading optiboot version, low bits
  stateGetVersHi,            // reading optiboot version, high bits
  stateIdle,                 // idle, waiting to program page
  stateProg,                 // programming...
};

struct AVRFlash : HexRecord {
    // the constructor allocates the memory necessary for the flashing operation. An AVRFlash object
    // should only be used once and a new one allocated to perform the next flash operation.
    AVRFlash(HardwareSerial &uart, uint8_t resetPin, int baudrate=115200) :
        HexRecord(128),
        _progState(stateInit),
        _stateStart(0),
        _baudCnt(0),
        _ackWait(false),
        _optibootVers(0),
        _baudrate(0),
        _confBaud(baudrate),
        _uart(uart),
        _resetPin(resetPin),
        _doneCB(0),
        _doneCBArg(0),
        _responseLen(0)
    {
        _responseBuf[0] = 0;
    }

    ~AVRFlash() {
        _timer.detach();
    }

    // sync initiates the flashing operation by starting the AVR reset and sync operations.
    // The AVR will then be kept in sync for some time expecting the data to arrive.
    void sync();

    // finish indicates that there is no more data coming and provides a callback that should be called
    // when the flashing operation has completed or errored.
    template< typename ARG >
    void finish(void (*doneCB)(ARG), ARG cbArg) {
        if (hasError()) {
            (*doneCB)(cbArg);
            return;
        }

        _doneCB = (void(*)(void*))doneCB;
        _doneCBArg = (void *)cbArg;
    }

    void abort() {
        _doneCB = 0; // just in case
        _timer.detach();
        resetAVR(); // avoid leaving it in some weird state
    }

    // private

    Ticker _timer;         // timer to keep things moving

    AVRProgStates _progState; // programming state
    uint32_t _stateStart;  // when we started the current _progState
    short _baudCnt;        // counter for sync attempts at different baud rates
    bool _ackWait;         // expecting an ACK
    uint16_t _optibootVers;
    uint32_t _baudrate;    // baud rate at which we're programming
    uint32_t _confBaud;    // baud rate configured/requested

    HardwareSerial &_uart;
    uint8_t _resetPin;

    void (*_doneCB)(void*);     // callback to be made when programming completes or errors
    void *_doneCBArg;

    char _responseBuf[RESP_SZ]; // buffer to accumulate responses from optiboot
    short _responseLen;    // amount accumulated so far

    void setBaudrate(uint32_t);
    void checkFinish();
    void resetAVR();
    void timerCB();
    void armTimer(uint32_t ms);
    void nextBaud();
    void fetchUart();
    bool parseResponse();
    void processAcks();
    bool checkSyncAck();
    bool programPage(FlashPage&);

};

#endif

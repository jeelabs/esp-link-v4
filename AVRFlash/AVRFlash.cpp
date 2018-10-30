// Copyright (c) 2015 by Thorsten von Eicken, see LICENSE.txt in the esp-link repo

// Some code moved to esp-link/pgmshared.c to avoid code duplication.
// Those changes are Copyright (c) 2017 by Danny Backx.

// Protocol used : https://github.com/Optiboot/optiboot/wiki/HowOptibootWorks

#include <Arduino.h>
#include "stk500.h"
#include "AVRFlash.h"

#define CB_INTERVAL      5   // check uart every N milliseconds
#define INIT_DELAY     150   // wait this many millisecs before sending anything
#define BAUD_INTERVAL  600   // interval after which we change baud rate
#define PGM_TIMEOUT  20000   // timeout after sync is achieved, in milliseconds
#define PGM_INTERVAL   200   // send sync at this interval in ms when in programming mode
#define ATTEMPTS         8   // number of attempts total to make

#define DBG(fmt, ...) _debug(PSTR(fmt), __VA_ARGS__)

static char* progStates[] = { "init", "sync", "sig", "ver0", "ver1", "idle", "prog" };

// HardwareSerial doesn't expose a way to set the baudrate, so we need a hack while this gets sorted
// out...
void AVRFlash::setBaudrate(uint32_t rate) {
    if (rate < 1200) return;
    _baudrate = rate;
    USD(0) = (ESP8266_CLOCK / rate);
}

// sync initiates the flashing operation by starting the AVR reset and sync operations.
void AVRFlash::sync() {
    DBG("AVRFlash::sync @br=%d\n", _confBaud);
    // TODO: check that there are not two programming requests at the same time
    // TODO: lock uart from serial bridge

    // check that we know the reset pin, else error out with that
    if (_resetPin < 0) {
        strcpy(_errMessage, "No reset pin defined");
        return;
    }

    // issue reset and start timer to get sync response
    setBaudrate(_confBaud);
    resetAVR();
    _progState = stateInit;
    armTimer(INIT_DELAY);
}


#if 0
    // calculate some stats
    float dt = ((system_get_time() - _startTime)/1000)/1000.0; // in seconds
    sprintf(_errMessage, "Success. %d bytes at %d baud in %d.%ds, %dB/s %d%% efficient",
            _pgmDone, _baudrate, (int)dt, (int)(dt*10)%10, (int)(_pgmDone/dt),
            (int)(100.0*(10.0*_pgmDone/_baudrate)/dt));
#endif

// checkFinish should be called when there is an error to trigger the _doneCB, if it has been
// registered. If it hasn't yet then when finish() will be called it will call the CB immediately
// itself since hasError() will be true.
void AVRFlash::checkFinish() {
    if (_doneCB) {
        (*_doneCB)(_doneCBArg);
    } else if (!hasError()) {
        // checkFinish should only be called when there is an error
        strcpy(_errMessage, "Unknown error, oops!");
    }
}

void AVRFlash::resetAVR() {
    pinMode(_resetPin, OUTPUT);
    digitalWrite(_resetPin, 0);
    delayMicroseconds(100);
    digitalWrite(_resetPin, 1);
}

static void _timerCB(AVRFlash*_this) { _this->timerCB(); }

// arm the one-shot timer so we move to the next state
void AVRFlash::armTimer(uint32_t ms) {
    _timer.once_ms<AVRFlash*>((uint32_t)ms, &_timerCB, this);
    //_timer.once_ms(ms, std::bind(&AVRFlash::timerCB, this));
}

static const int baudrates[] = { 0, 9600, 57600, 115200 };

// move to next baud rate
void AVRFlash::nextBaud() {
    setBaudrate(_baudCnt%4 == 0 ? _confBaud : baudrates[_baudCnt%4]);
    _baudCnt++;
    DBG("changing to %ld baud\n", _baudrate);
}

void AVRFlash::fetchUart() {
    int ch;
    while (_responseLen < RESP_SZ-1 && (ch=_uart.read()) >= 0) {
        _responseBuf[_responseLen++] = ch;
    }
    _responseBuf[_responseLen] = 0;
}

void AVRFlash::processAcks() {
    while (_responseLen >= 2 && _responseBuf[0] == STK_INSYNC && _responseBuf[1] == STK_OK) {
        if (_ackWait) _ackWait = false;
        memmove(_responseBuf, _responseBuf+2, _responseLen-2);
        _responseLen -= 2;
    }
}

// timer callback that drives pretty much everything else
void AVRFlash::timerCB() {
    switch (_progState) {
    case stateInit: // initial delay expired, send sync chars
        _uart.write(STK_GET_SYNC);
        _uart.write(CRC_EOP);
        _progState = stateSync;
        _stateStart = millis();
        armTimer(CB_INTERVAL);
        return;
    case stateSync: // waiting to get an ACK response to sync request
        if (checkSyncAck()) {
            // got ack, send request to get signature
            _uart.write(STK_READ_SIGN);
            _uart.write(CRC_EOP);
            _progState = stateGetSig;
            _stateStart = millis();
            _startTime = _stateStart;
            armTimer(CB_INTERVAL);
            DBG("got sync, sending read-sig \n", 0);
            return;
        }
        if (millis()-_stateStart < BAUD_INTERVAL-INIT_DELAY) {
            // need to keep waiting...
            armTimer(CB_INTERVAL);
            return;
        }
        if (_baudCnt > ATTEMPTS) {
            // we're doomed, give up
            sprintf(_errMessage, "sync abandoned after %d attempts", _baudCnt);
            DBG("%s\n", _errMessage);
            checkFinish();
            return;
        }
        // time to switch baud rate and issue a reset
        DBG("no sync response @%d baud\n", _baudrate);
        nextBaud();
        resetAVR();
        _progState = stateInit;
        armTimer(INIT_DELAY);
        return;
    case stateIdle: // we need to send the next programming command if we can
        fetchUart();
        processAcks();
        if (_lastPage != 0) {
            // we have a page we can flash!
            FlashPage *fp = _lastPage->next;
            if (_lastPage == fp) {
                _lastPage = 0;
                if (_resume != 0) (*_resume)(_resumeArg);
            } else {
                _lastPage->next = fp->next;
            }
            programPage(*fp);
            delete fp;
            _progState = stateProg;
            _stateStart = millis();
            armTimer(CB_INTERVAL);
            return;
        }
        if (_doneCB) {
            // we got the final callback info, this means no more data, we're done!
            // tell optiboot to reboot into the sketch
            _uart.write(STK_LEAVE_PROGMODE);
            _uart.write(CRC_EOP);
            // perform the callback
            (*_doneCB)(_doneCBArg);
            return;
        }
        if (millis()-_startTime > PGM_TIMEOUT) {
            strcpy(_errMessage, "programming time-out");
            DBG("%s\n", _errMessage);
            checkFinish();
            return;
        }
        if (millis()-_stateStart > PGM_INTERVAL) {
            _uart.write(STK_GET_SYNC);
            _uart.write(CRC_EOP);
            _ackWait=true; // we now expect an ACK
            _stateStart = millis();
        }
        armTimer(CB_INTERVAL);
        return;
    case stateProg: // we're programming and we're waiting for the ack
        fetchUart();
        processAcks();
        if (!_ackWait) {
            DBG("Programmed page\n", 0);
            _progState = stateIdle;
            _stateStart = millis();
            _uart.write(STK_GET_SYNC);
            _uart.write(CRC_EOP);
            _ackWait=true; // we now expect an ACK
            armTimer(CB_INTERVAL);
            return;
        }
        if (millis()-_stateStart > PGM_INTERVAL) {
            strcpy(_errMessage, "no response to page programming command");
            DBG("%s\n", _errMessage);
            checkFinish();
            return;
        }
        armTimer(CB_INTERVAL);
        return;
    default: // we're trying to get some info from optiboot so we need to check whether it responded
        if (parseResponse()) {
            _stateStart = millis();
            armTimer(CB_INTERVAL);
            return;
        }
        if (hasError()) {
            DBG("%s\n", _errMessage);
            checkFinish();
            return;
        }
        if (millis()-_stateStart > PGM_INTERVAL) {
            sprintf(_errMessage, "no response in state %s(%d) @%d baud\n",
                    progStates[_progState], _progState, _baudrate);
            DBG("%s\n", _errMessage);
            checkFinish();
            return;
        }
        armTimer(CB_INTERVAL);
        return;
    }
}

#if 0
static void print_buff(char *msg, char *buf, short length) {
  DBG("OB GOT %s %d:", msg, length);
  for (int i=0; i<length; i++) DBG(" %x", buf[i]);
  DBG("\n");
}
#endif

// checkSyncAck looks for an ack at the end of the buffer, assuming that the running sketch may have
// spewed a bunch of chars before the reset.
bool AVRFlash::checkSyncAck() {
    fetchUart();
    // look for STK_INSYNC+STK_OK at end of buffer
    if (_responseLen > 0 && _responseBuf[_responseLen-1] == STK_INSYNC) {
        // missing STK_OK after STK_INSYNC, shift stuff out and try again
        _responseBuf[0] = STK_INSYNC;
        _responseLen = 1;
    } else if (_responseLen > 1 && _responseBuf[_responseLen-2] == STK_INSYNC &&
            _responseBuf[_responseLen-1] == STK_OK) {
        // got sync response
        _responseLen = 0; // ignore anything that may have accumulated
        return true;
    } else {
        // nothing useful, keep at most half the buffer for error message purposes
        if (_responseLen > RESP_SZ/2) {
            memcpy(_responseBuf, _responseBuf+_responseLen-RESP_SZ/2, RESP_SZ/2);
            _responseLen = RESP_SZ/2;
            _responseBuf[_responseLen] = 0; // string terminator
        }
    }
    return false;
}

// programPage starts the programming of a page by sending the address and the data.
bool AVRFlash::programPage(FlashPage &fp) {
    if (fp.len > _pageSz) {
        strcpy(_errMessage, "Internal error: FlashPage too long");
        return false;
    }
    DBG("Programming %d@0x%x\n", fp.len, fp.addr);

    // send address to optiboot (little endian format)
    _uart.write(STK_LOAD_ADDRESS);
    uint16_t addr = fp.addr >> 1; // word address
    _uart.write(addr & 0xff);
    _uart.write(addr >> 8);
    _uart.write(CRC_EOP);

    // wait a brief amt to get an ack
    _ackWait=true;
    uint32_t t0 = millis();
    while (_ackWait) {
        if (millis()-t0 > 2) {
            strcpy(_errMessage,"flashing failed in load address");
            return false;
        }
        fetchUart();
        processAcks();
    }

    // send page length (big-endian format, go figure...)
    _uart.write(STK_PROG_PAGE);
    _uart.write(fp.len>>8);
    _uart.write(fp.len&0xff);
    _uart.write('F'); // we're writing flash

    // send page content
    _uart.write(fp.data, fp.len);
    _uart.write(CRC_EOP);
    _ackWait=true;
    return true;
}

bool AVRFlash::parseResponse() {
    fetchUart();
    switch (_progState) {
    case stateGetSig: // expecting signature
        processAcks();
        if (_responseLen < 5) return false;
        if (_responseBuf[0] == STK_INSYNC && _responseBuf[4] == STK_OK &&
            _responseBuf[1] == 0x1e && _responseBuf[2] == 0x95 && _responseBuf[3] == 0x0f) {
                // right on... ask for optiboot version
                //DBG("Got signature!\n", 0);
                _uart.write(STK_GET_PARAMETER);
                _uart.write(0x82);
                _uart.write(CRC_EOP);
                _progState = stateGetVersLo;
                _responseLen = 0;
                return true;
        }
        sprintf(_errMessage, "bad programmer signature: 0x%02x 0x%02x 0x%02x\n",
                _responseBuf[1], _responseBuf[2], _responseBuf[3]);
        return false;
    case stateGetVersLo: // expecting version
        if (_responseLen < 3) return false;
        if (_responseBuf[0] == STK_INSYNC && _responseBuf[2] == STK_OK) {
            //DBG("Got vers lo %d\n", _responseBuf[1]);
            _optibootVers = _responseBuf[1];
            _uart.write(STK_GET_PARAMETER);
            _uart.write(0x81);
            _uart.write(CRC_EOP);
            _progState = stateGetVersHi;
            _responseLen = 0;
            return true;
        }
        strcpy(_errMessage, "did not get optiboot version low");
        return false;
    case stateGetVersHi: // expecting version
        if (_responseLen < 3) return false;
        if (_responseBuf[0] == STK_INSYNC && _responseBuf[2] == STK_OK) {
            //DBG("Got vers hi %d\n", _responseBuf[1]);
            _optibootVers |= _responseBuf[1]<<8;
            _progState = stateIdle;
            _responseLen = 0;
            return true;
        }
        strcpy(_errMessage, "did not get optiboot version high");
        return false;
    }
    return false;
}

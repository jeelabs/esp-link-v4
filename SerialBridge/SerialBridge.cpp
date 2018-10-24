// Esp-link-v4 Serial Bridge
// Copyright (C) 2018 by Throsten von Eicken

#include "Arduino.h"
#include "SerialBridge.h"

// INFO is used to print infrequent informational messages, e.g. when a client connects/disconnects
#define INFO(...) _sbr_debug(__VA_ARGS__)
// DBG prints more frequent debug messages
#define DBG(...)
// TRC prints short and cryptic trace messages that allow the exaqct flow of data to be traced. The
// messages are short so they don't alter the timing of the code much.
#define TRC(...)

#define SERIAL_BRIDGE_PORT Serial

static void(*_sbr_debug)(const char*, ...) = 0;

// SbrClient holds the state we need for one TCP client.
struct SbrClient {
    SerialBridge *sbr;
    AsyncClient *client;    // handle to ESPAsyncTCP client
    uint8_t     *rxBuf;     // malloc'ed buffer with received characters
    uint16_t    rxBufSize;  // size of buffer in bytes
    uint16_t    rxBufNext;  // next char in buffer to send to UART

    void rxBufToUart(int writable);
    void handleError(int8_t error);
    void handleData(void *data, size_t len);
    void handleDisconnect();
    void handleTimeout(uint32_t time);
};

// rxBufToUart writes chars from an rx buffer to the uart.
void SbrClient::rxBufToUart(int writable) {
    int w = rxBufSize - rxBufNext;
    if (w > writable) w = writable;
    DBG(PSTR("[SERIAL_BRIDGE] writing %d buf->uart\n"), w);
    TRC(PSTR("wr<%d>"), w);
    int n = SERIAL_BRIDGE_PORT.write(rxBuf+rxBufNext, w);
    rxBufNext += n;
    if (rxBufNext == rxBufSize) {
        DBG(PSTR("[SERIAL_BRIDGE] free 0x%x, cli %x\n"), rxBuf, sbr_cli);
        free(rxBuf);
        rxBuf = 0;
        if (client) // null if connection is already closed
            client->ack(rxBufSize);
    }
}

// client socket event handlers

// handleError just prints a message and it is expected that ESPAsyncTCP also calls the
// handleDisconnect callback.
void SbrClient::handleError(int8_t error) {
    INFO(PSTR("[SERIAL_BRIDGE] conn err client %s: %s\n"),
        client->remoteIP().toString().c_str(),
        client->errorToString(error));
}

// handleData receives a packet coming in on a TCP connection. It tries to stuff some characters
// into the uart and has to buffer the rest. Only the characters stuffed into the uart are acked.
void SbrClient::handleData(void *data, size_t len) {
        DBG(PSTR("[SERIAL_BRIDGE] rcv client %s: %d bytes\n"),
                client->remoteIP().toString().c_str(), len);
        size_t writable = SERIAL_BRIDGE_PORT.availableForWrite();
        TRC(PSTR("rx<%d/%d/%d>"), len, rxBuf ? rxBufSize - rxBufNext : 0, writable);
        // if we have buffered chars take this opportunity to stuff some into the uart
        if (writable > 0 && rxBuf != 0) {
            rxBufToUart(writable);
            writable = SERIAL_BRIDGE_PORT.availableForWrite();
        }
        // if we can write all to uart then we're done
        if (!rxBuf && writable > len) {
            TRC(PSTR("wr{%d}"), len);
            DBG(PSTR("[SERIAL_BRIDGE] writing all %d to uart\n"), len);
            SERIAL_BRIDGE_PORT.write((uint8_t*)data, len);
            return;
        }
        // write what we can
        if (!rxBuf && writable > 0) {
            client->ackLater();
            TRC(PSTR("wr[%d]"), writable);
            DBG(PSTR("[SERIAL_BRIDGE] writing %d to uart\n"), writable);
            SERIAL_BRIDGE_PORT.write((uint8_t *)data, writable);
            client->ack(writable);
        } else {
            writable = 0;
            client->ackLater();
        }
        // buffer what we couldn't write
        if (!rxBuf) {
            DBG(PSTR("[SERIAL_BRIDGE] new buffer %d\n"), len-writable);
            rxBufSize = len-writable;
            rxBuf = (uint8_t *)calloc(1, rxBufSize);
            if (rxBuf != 0) {
                DBG(PSTR("[SERIAL_BRIDGE] calloc 0x%x, cli %x\n"), rxBuf, this);
                rxBufNext = 0;
                memcpy(rxBuf, (uint8_t*)data+writable, rxBufSize);
            } else {
                INFO(PSTR("[SERIAL_BRIDGE] calloc failed\n"));
            }
        } else {
            DBG(PSTR("[SERIAL_BRIDGE] append buffer %d\n"), len-writable);
            int sz = rxBufSize + len-writable;
            rxBuf = (uint8_t *)realloc(rxBuf, sz);
            if (rxBuf != 0) {
                DBG(PSTR("[SERIAL_BRIDGE] realloc 0x%x, cli %x\n"), rxBuf, this);
                memcpy(rxBuf+rxBufSize, (uint8_t*)data+writable, len-writable);
                rxBufSize = sz;
            } else {
                INFO(PSTR("[SERIAL_BRIDGE] realloc failed\n"));
            }
        }
}

// handleDisconnect receives notification that a connection has been terminated. It cannot fully
// clean up the client struct because data may be pending in the buffer.
void SbrClient::handleDisconnect() {
        INFO(PSTR("[SERIAL_BRIDGE] client %s disconnect\n"),
            client->remoteIP().toString().c_str());
        client = 0;
}

// handleTimeout just prints a message for now 'cause it's not clear what needs to be done, e.g.,
// does it have to actively close the (obviously broken) connection?
void SbrClient::handleTimeout(uint32_t time) {
        INFO(PSTR("[SERIAL_BRIDGE] client %s TCP timeout\n"),
            client->remoteIP().toString().c_str());
}

// server socket event handlers

static void _sbrHandleData(void* arg, AsyncClient* c, void *data, size_t len) {
    ((SbrClient*)arg)->handleData(data, len);
}
static void _sbrHandleError(void* arg, AsyncClient* c, int8_t error) {
    ((SbrClient*)arg)->handleError(error);
}
static void _sbrHandleDisconnect(void* arg, AsyncClient* c) {
    ((SbrClient*)arg)->handleDisconnect();
}
static void _sbrHandleTimeout(void* arg, AsyncClient* c, uint32_t time) {
    ((SbrClient*)arg)->handleTimeout(time);
}
static void _sbrHandleNewClient(void* arg, AsyncClient* client) {
    ((SerialBridge*)arg)->handleNewClient(client);
}

// handleNewClient is called for a new connection and allocates a client descriptor.
void SerialBridge::handleNewClient(AsyncClient* client) {
    INFO(PSTR("[SERIAL_BRIDGE] connect from %s\n"),
        client->remoteIP().toString().c_str());

    // add to list
    SbrClient *sbr_cli = (SbrClient*)calloc(1, sizeof(SbrClient));
    sbr_cli->sbr = this;
    sbr_cli->client = client;
    _clients.push_back(sbr_cli);

    // register callbacks
    client->onData(&_sbrHandleData, sbr_cli);
    client->onError(&_sbrHandleError, sbr_cli);
    client->onDisconnect(&_sbrHandleDisconnect, sbr_cli);
    client->onTimeout(&_sbrHandleTimeout, sbr_cli);
}

// periodic functions that keep things moving in the arduino loop()

// recvUartCheck checks whether something arrived on the uart and tries to send it out on
// connected clients. It only pulls out of the uart receive buffer what it can send to all clients.
// The assumption here is that the interrupt handler's buffer is sufficient.
void SerialBridge::recvUartCheck() {
    // check that we have connected clients and there's something to send
    if (SERIAL_BRIDGE_PORT.peek() < 0) {
        SERIAL_BRIDGE_PORT.hasOverrun(); // clear flag in uart driver
        _overrun = false;
        return;
    }
    if (_clients.empty()) {
        // no client connected, drop incoming chars on the floor
        while (SERIAL_BRIDGE_PORT.read() != -1) ;
        SERIAL_BRIDGE_PORT.hasOverrun(); // clear flag in uart driver
        return;
    }
    // warn about input overrun
    if (!_overrun && SERIAL_BRIDGE_PORT.hasOverrun()) {
        _overrun = true;
        INFO(PSTR("[SERIAL_BRIDGE] uart input overrun\n"));
    }
    // we always send the same to all clients: determine minimum we can send to all
    size_t min_sendable = SERIAL_BRIDGE_PORT.available();
    size_t avail = min_sendable;
    for (SbrClient* cli : _clients) {
        if (!cli->client) continue; // already closed
        if (cli->client->space() < min_sendable) {
            min_sendable = cli->client->space();
        }
    }
    if (min_sendable <= 0) { TRC(PSTR("tx{0/%d}"), avail); return; }
    // read from serial into buffer
    char buf[min_sendable];
    for (size_t i=0; i<min_sendable; i++) {
        buf[i] = SERIAL_BRIDGE_PORT.read();
    }
    // send buffer to each client
    for (SbrClient* cli : _clients) {
        if (!cli->client) continue; // already closed
        size_t n = cli->client->add(buf, min_sendable, 0);
        if (n != min_sendable) { // should never occur..
            INFO(PSTR("[SERIAL_BRIDGE] err client %s: will=%d sendable=%d\n"),
                cli->client->remoteIP().toString().c_str(), n, min_sendable);
        } else {
            TRC(PSTR("tx<%d/%d>"), n, avail);
            DBG(PSTR("[SERIAL_BRIDGE] sent %d bytes to cli %x\n"), n, cli);
        }
        if (!cli->client->send()) INFO(PSTR("[SERIAL_BRIDGE] send failed\n"));
    }
}

// recvTCPCheck handles data that got received but couldn't be stuffed into the uart.
void SerialBridge::recvTCPCheck() {
    for (SbrClient* cli : _clients) {
        if (cli->rxBuf == 0) continue;
        //if (cli->client && cli->client->space() == 0) continue; // HACK!
        int writable = SERIAL_BRIDGE_PORT.availableForWrite();
        if (writable <= 0) continue;
        // looks like we have something that we can write to the UART, so do it...
        cli->rxBufToUart(writable);
    }
}

// gc garbage collects client descriptors that have no connection and no buffer
void SerialBridge::gc() {
    for (auto cli = _clients.begin(); cli != _clients.end(); ) {
        if ((*cli)->client || (*cli)->rxBuf) {
            cli++; // client connected or buffer still has data
        } else {
            cli = _clients.erase(cli); // no connection and no buffer: garbage collect
        }
    }
}

// loop must be called from the arduino loop function to perform background tasks
void SerialBridge::loop() {
    if (!_clients.empty()) TRC("{");
    recvUartCheck();
    recvTCPCheck();
    gc();
    if (!_clients.empty()) TRC("}");
}

void SerialBridge::debug(void dbgPrintf(const char*, ...)) {
    _sbr_debug = dbgPrintf;
}

void SerialBridge::begin(uint16_t port, uint32_t baudrate, uint32_t rxBufSz) {
    // init port
    SERIAL_BRIDGE_PORT.setRxBufferSize(rxBufSz);
    SERIAL_BRIDGE_PORT.begin(baudrate);

    _clients.clear();
    AsyncServer* server = new AsyncServer(port);
    server->onClient(&_sbrHandleNewClient, this);
    server->begin();
    INFO(PSTR("[SERIAL_BRIDGE] listening on port %d, baud rate %d\n"), port, baudrate);
}

// Copyright (c) 2015-2018 by Thorsten von Eicken, see LICENSE.txt in the esp-link repo
// Copyright (c) 2017 by Danny Backx

#ifndef _PGM_DATA_H_
#define _PGM_DATA_H_

#define ERR_MAX 128

// FlashPage describes a page of flash to be programmed.
struct FlashPage {
    FlashPage *next;
    uint16_t len;
    uint16_t pad;
    uint32_t addr;
    uint8_t data[0];
};

// structure used to remember request details from one callback to the next
struct HexRecord {
    HexRecord(uint32_t pageSize) :
        _pageLen(0),
        _address(0),
        _lastPage(0),
        _stop(0),
        _stopArg(0),
        _resume(0),
        _resumeArg(0),
        _pageSz(pageSize),
        _pgmDone(0),
        _startTime(0),
        _eof(0),
        _mega(false)
    {
        _errMessage[0] = 0;
        _pageBuf = (uint8_t*)calloc(1, pageSize+pageSize/2);
        _saved = (uint8_t*)calloc(1, 128); // need space for string terminator
        if (!_pageBuf || !_saved) {
            strcpy(_errMessage, "Out of memory");
        }
    }

    ~HexRecord() {
        if (_pageBuf) free(_pageBuf);
        if (_saved) free(_saved);
    }

    // write a buffer of hex records to flash. This really just parses the hex records and
    // places the info/data into FlashPage structs. If stop and resume are non-null,
    // it calls stop(cbArg) while parsing when it enqueues a page and it arranges for resume(cbArg)
    // to be called when the last page is dequeued.
    // It returns the number of bytes written (really: it returns len on success and 0 on failure).
    template<typename ARG>
    uint32_t write(uint8_t *data, size_t len, void (*stop)(ARG), void (*resume)(ARG), ARG cbArg) {
        if (hasError()) return 0;
        _stop = (void(*)(void*))stop;
        _stopArg = (void*)cbArg;
        _resume = (void(*)(void*))resume;
        _resumeArg = (void*)cbArg;
        return _write(data, len);
    }

    // hasError returns true if an error occurred
    bool hasError() { return _errMessage[0] != 0; }
    // getError() returns an error description as a string (or null if no error)
    char *getError() { return _errMessage; }

    // private

    uint8_t *_saved;            // buffer for saved incomplete hex records
    // current page being accumulated
    uint8_t *_pageBuf;          // buffer for received data to be sent to AVR
    uint16_t _pageLen;          // number of bytes in pageBuf
    uint32_t _address;          // address to write next page to
    // queue of pages to be programmed
    FlashPage *_lastPage;       // pointer to last page, _lastPage->next points to first page
    void (*_stop)(void*);       // callback to stop input into write()
    void *_stopArg;
    void (*_resume)(void*);     // callback to resume input into write()
    void *_resumeArg;

    uint16_t _pageSz;           // size of flash page to be programmed at a time
    uint32_t _pgmDone;          // number of bytes programmed
    uint32_t _startTime;        // time of program POST request
    bool _eof;                  // got EOF record
    //uint32_t _segment;          // for extended segment addressing, added to the address field

    char _errMessage[ERR_MAX];  // error message

    // STK500v2 variables
    bool _mega;                 // whether to use the Mega (STK500v2) protocol
    //int _hardwareVersion, _firmwareVersionMajor, _firmwareVersionMinor, _vTarget;
    //uint8_t _signature[3];
    //uint8_t  _lfuse, _hfuse, _efuse;

    // methods
    static bool checkHex(uint8_t *buf, short len);
    static uint32_t getHexValue(uint8_t *buf, short len);
    static void appendPretty(uint8_t *buf, int max, uint8_t *raw, int rawLen);
    static bool verifyChecksum(uint8_t *buf, short len);
    uint32_t _write(uint8_t *data, size_t len);
    bool processRecord(uint8_t *buf, short len);
    void addPage();

    // debug sets the printf function used for info/debug messages
    void debug(void dbgPrintf(const char*, ...)) { _debug = dbgPrintf; }
    void (*_debug)(const char*, ...);
};

#endif

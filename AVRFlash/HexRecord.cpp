// Copyright (c) 2015 by Thorsten von Eicken, see LICENSE.txt in the esp-link repo
// Copyright (c) 2016-2017 by Danny Backx

#include <Arduino.h>
#include "HexRecord.h"

#define DBG(fmt, ...) _debug(PSTR(fmt), __VA_ARGS__)

// hasError returns true if an error occurred
bool HexRecord::hasError() { return _errMessage[0] != 0; }
// getError() returns an error description as a string (or null if no error)
char *HexRecord::getError() { return hasError() ? _errMessage : 0; }

// verify that N chars are hex characters
bool HexRecord::checkHex(uint8_t *buf, short len) {
  while (len--) {
    char c = *buf++;
    if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f'))
      continue;
    return false;
  }
  return true;
}

// get hex value of some hex characters
uint32_t HexRecord::getHexValue(uint8_t *buf, short len) {
  uint32_t v = 0;
  while (len--) {
    v = (v<<4) | (uint32_t)(*buf & 0xf);
    if (*buf > '9') v += 9;
    buf++;
  }
  return v;
}

// verify checksum
bool HexRecord::verifyChecksum(uint8_t *buf, short len) {
  uint8_t sum = 0;
  while (len >= 2) {
    sum += (uint8_t)getHexValue(buf, 2);
    buf += 2;
    len -= 2;
  }
  return sum == 0;
}

// append one string to another but visually escape non-printing characters in the appended
// string using \x00 hex notation, max is the max chars in the concatenated string.
void HexRecord::appendPretty(uint8_t *buf, int max, uint8_t *raw, int rawLen) {
  int off = strlen((char*)buf);
  max -= off + 1; // for null termination
  for (int i=0; i<max && i<rawLen; i++) {
    unsigned char c = raw[i++];
    if (c >= ' ' && c <= '~') {
      buf[off++] = c;
    } else if (c == '\n') {
      buf[off++] = '\\';
      buf[off++] = 'n';
    } else if (c == '\r') {
      buf[off++] = '\\';
      buf[off++] = 'r';
    } else {
      buf[off++] = '\\';
      buf[off++] = 'x';
      buf[off++] = '0'+(unsigned char)((c>>4)+((c>>4)>9?7:0));
      buf[off++] = '0'+(unsigned char)((c&0xf)+((c&0xf)>9?7:0));
    }
  }
  buf[off] = 0;
}

// write accepts data in the form of hex records to be flashed to the uC. It accumulates the
// input into the '_saved' buffer from where it processes hex records. The hex records are then
// enqueued for programming when the uC is ready.
uint32_t HexRecord::_write(uint8_t *data, size_t len) {
    //DBG("HexRecord::write %d bytes\n", len);
    bool qEmpty = _lastPage == 0;
    uint32_t ret = len; // need to return len on success

    // iterate through the data received, parse HEX records, and enqueue them
    while (len > 0) {
        // first fill-up the saved buffer
        short saveLen = strlen((char*)_saved);
        if (saveLen < _pageSz) {
            short cpy = _pageSz-saveLen;
            if (cpy > len) cpy = len;
            memcpy(_saved+saveLen, data, cpy);
            saveLen += cpy;
            data += cpy;
            len -= cpy;
            _saved[saveLen] = 0; // string terminator
            //DBG("cp %d buff->saved\n", cpy);
        }

        // process HEX records
        while (saveLen >= 11) { // 11 is minimal record length
            // skip any CR/LF
            short skip = 0;
            while (skip < saveLen && (_saved[skip] == '\n' || _saved[skip] == '\r'))
                skip++;
            if (skip > 0) {
                // shift out cr/lf (keep terminating \0)
                memmove(_saved, _saved+skip, saveLen+1-skip);
                saveLen -= skip;
                if (saveLen < 11) break;
                //DBG("skip %d cr/lf\n", skip);
            }

            // inspect whether we have a proper record start
            if (_saved[0] != ':') {
                DBG("found non-: start\n", 0);
                sprintf(_errMessage, "Expected start of record in POST data, got %c", _saved[0]);
                return 0;
            }

            if (!checkHex(_saved+1, 2)) return 0; // error... checkHex sets _errMessage
            uint8_t recLen = getHexValue(_saved+1, 2);
            //DBG("record %d\n", recLen);

            // process record
            if (saveLen < 11+recLen*2) break; // need more data to fill-in first...
            if (!processRecord(_saved, 11+recLen*2)) return 0; // processRecord sets _errMessage
            if (qEmpty && _lastPage != 0) (*_stop)(_stopArg); // added first page, tell input to stop

            // shift record out of _saved buffer
            short shift = 11+recLen*2;
            memmove(_saved, _saved+shift, saveLen+1-shift);
            saveLen -= shift;
            //DBG("OB %d byte record\n", shift);
        }
    }
    return ret;
}

// addPage appends a page to the list of pages to be programmed
void HexRecord::addPage() {
    DBG("HexRecord::addPage(@0x%x, %d bytes)\n", _address, _pageLen);
    FlashPage *fp = (FlashPage*)calloc(1, 3*4+_pageLen);
    if (fp == 0) {
        strcpy(_errMessage, "out of memory");
        return;
    }

    fp->len = _pageLen;
    fp->addr = _address;
    memcpy(fp->data, _pageBuf, _pageLen);
    // add to end of page list
    if (_lastPage) {
        fp->next = _lastPage->next;
        _lastPage->next = fp;
    } else {
        fp->next = fp;
    }
    _lastPage = fp;

    _pageLen = 0;
}

// processRecord parses a hex record and typically appends to a flashPage
// assumes that the records starts with ':' & hex length
bool HexRecord::processRecord(uint8_t *buf, short len) {
    buf++; len--; // skip leading ':'

    // check we have all hex chars
    if (!checkHex(buf, len)) {
        sprintf(_errMessage, "Invalid hex character found");
        return false;
    }

    // verify checksum
    if (!verifyChecksum(buf, len)) {
        buf[len>32?32:len] = 0; // avoid _errMessage buffer overflow
        sprintf(_errMessage, "Invalid checksum for record %s", buf);
        return false;
    }

    // dispatch based on record type
    uint8_t type = getHexValue(buf+6, 2);
    switch (type) {
    case 0x00: { // Intel HEX data record
        //DBG("REC data %ld pglen=%d\n", getHexValue(buf, 2), _pageLen);
        uint32_t addr = getHexValue(buf+2, 4);
        // check whether this is disjoint from data we have accumulated
        if (_pageLen > 0 && addr != ((_address+_pageLen)&0xffff)) {
           addPage();
        }
        // set address, unless we're adding to the end (_addPage call may have changed pageLen)
        if (_pageLen == 0) {
           _address = (_address & 0xffff0000) | addr;
        }
        // append record
        uint16_t recLen = getHexValue(buf, 2);
        for (uint16_t i=0; i<recLen; i++)
           _pageBuf[_pageLen++] = getHexValue(buf+8+2*i, 2);
        // add page, if we have a full page
        if (_pageLen >= _pageSz) addPage();
        break; }
    case 0x01: // Intel HEX EOF record
        // add any remaining partial page
        if (_pageLen > 0) addPage();
        _eof = true;
        break;
    case 0x04: // Intel HEX address record
        DBG("HexRecord::processRecord: address 0x%x\n", getHexValue(buf+8, 4) << 16);
        // add any remaining partial page
        if (_pageLen > 0) addPage();
        _address = getHexValue(buf+8, 4) << 16;
        break;
    case 0x05: // Intel HEX start address (MDK-ARM only)
        // ignore, there's no way to tell optiboot that...
        break;
    case 0x02: // Intel HEX extended segment address record
        // Depending on the case, just ignoring this record could solve the problem
        // _segment = getHexValue(buf+8, 4) << 4;
        //DBG("segment 0x%08X\n", _segment);
        return true;
    default:
        // DBG("OB bad record type\n");
        sprintf(_errMessage, "Invalid/unknown record type: 0x%02x, packet %s", type, buf);
        return false;
    }
    return true;
}

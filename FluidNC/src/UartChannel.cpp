// Copyright (c) 2023 -  Mitch Bradley
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "UartChannel.h"
#include "Driver/Console.h"
#include "Machine/MachineConfig.h"  // config
#include "Serial.h"                 // allChannels
#include "Report.h"                 // report_realtime_status

UartChannel::UartChannel(objnum_t num, bool addCR) : Channel("uart_channel", num, addCR) {
    _lineedit = new Lineedit(this, _line, Channel::maxLine - 1);
    _active   = false;
}

void UartChannel::init() {
    auto uart = config->_uarts[_uart_num];
    if (!uart) {
        log_error(name() << ": missing uart" << _uart_num);
    } else if (!uart->configured()) {
        log_error(name() << ": uart" << _uart_num << " failed configuration");
    } else {
        init(uart);
    }
    // _active means "this port has shown signs of life", and is otherwise set
    // only by receiving.  With no rx pin nothing ever will, so start active.
    // The uart can be null here if it is missing from the config; dereferencing
    // it unconditionally used to crash during startup.
    if (!uart || uart->_rxd_pin.undefined()) {
        _active = true;
    }
    setReportInterval(_report_interval_ms);
}
void UartChannel::init(Uart* uart) {
    if (!uart || !uart->configured()) {
        log_error(name() << ": cannot initialize with unconfigured UART");
        return;
    }
    _uart = uart;
    // A serial port with a report interval is a pendant or a sender, and it
    // needs reports to keep coming while the machine is idle - that is the
    // whole point of asking for an interval.  Browsers, which get an interval
    // too, keep the on-change-only behaviour.
    _report_when_idle = true;
    allChannels.registration(this);
    if (_report_interval_ms) {
        log_info(name() << " created at report interval: " << _report_interval_ms);
    } else {
        log_info(name() << " created");
    }
    sendGreeting();
    if (_uart_num) {
        getExpanderId();
    }
}

void UartChannel::sendGreeting() {
    // Tell the channel listener that FluidNC has restarted.
    // The initial newline clears out any garbage characters that might have
    // resulted from the UART initialization and turn-on
    print("\n");
    out("RST", "MSG:");
    _last_greeting_ms = millis();
}

// How often to repeat the greeting while the port has not answered.
static const uint32_t greeting_repeat_ms = 1000;

void UartChannel::handle() {
    // A device that powers up alongside FluidNC - a pendant, typically - is
    // often not listening yet when init() sends the greeting, and misses it.
    // FluidNC then says nothing more: autoReport() only sends a time-based
    // report while the machine is moving, so an idle machine produces no
    // traffic at all.  The device waits for data before introducing itself,
    // FluidNC waits to be spoken to, and the standoff lasts until something
    // unrelated happens to emit a log line.  That is the "long delay after
    // powerup, and only once it received any data" behaviour.
    //
    // Repeating the greeting until the port answers costs a few bytes a second
    // on a dedicated UART and lets a late starter synchronise promptly.  It
    // also covers a pendant plugged in after FluidNC has booted.
    // Gate on having received a whole command line, not on _active.  _active is
    // set by any received byte, and line noise at power-up - or anything
    // getExpanderId() hands to the queue - sets it before the pendant has said
    // a word, which would switch the greeting off precisely when it is needed.
    if (_peer_spoke || !_report_interval_ms) {
        return;
    }
    if ((millis() - _last_greeting_ms) < greeting_repeat_ms) {
        return;
    }
    sendGreeting();

    // Send a real status report as well.  The banner alone only says FluidNC
    // restarted; a pendant waiting to see live data has something to work with
    // straight away.  This is confined to a port that has a report interval
    // configured and has not answered yet, so no other channel is affected.
    report_realtime_status(*this);
}

// An IO expander answers the ID query immediately.  Waiting longer than this
// only delays startup, and the old loop had no overall bound at all: it ran for
// as long as anything kept arriving.
static const uint32_t expander_id_timeout_ms = 100;

void UartChannel::getExpanderId() {
    out("ID", "EXP:");

    char     buf[128];
    uint32_t start = millis();
    while ((millis() - start) < expander_id_timeout_ms) {
        // sizeof(buf) - 1 leaves room for the terminator.  Reading a full 128
        // bytes used to write buf[128], one past the end.
        size_t len = _uart->timedReadBytes(buf, sizeof(buf) - 1, 10);
        if (!len) {
            continue;
        }
        buf[len] = '\0';
        if (strncmp(buf, "(EXP,", 5) == 0) {
            auto pos = strrchr(buf, ')');
            if (pos) {
                *pos = '\0';
            }
            print("ok\n");
            log_info("IO Expander " << &buf[5]);
            return;
        }

        // Not an expander, so this belongs to whatever is actually on the port -
        // a pendant, typically, and very likely its opening message.  These reads
        // bypass the channel, so anything dropped here is invisible to
        // Channel::pollLine() and never sets _active.  Discarding it left the
        // channel inactive and silent until the device happened to transmit
        // again, which is why a pendant could take a long time to come up, or
        // never come up at all.  Hand it to the normal input path instead;
        // realtime characters are recognised when the bytes are dequeued.
        for (size_t i = 0; i < len; i++) {
            queue_push(static_cast<uint8_t>(buf[i]));
        }
        _active = true;
    }
}

size_t UartChannel::write(uint8_t c) {
    return _uart->write(c);
}

size_t UartChannel::write(const uint8_t* buffer, size_t length) {
    // Replace \n with \r\n
    if (_addCR) {
        size_t rem      = length;
        char   lastchar = '\0';
        size_t j        = 0;
        while (rem) {
            const int bufsize = 80;
            uint8_t   modbuf[bufsize];
            // bufsize-1 in case the last character is \n
            size_t k = 0;
            while (rem && k < (bufsize - 1)) {
                char c = buffer[j++];
                if (c == '\n' && lastchar != '\r') {
                    modbuf[k++] = '\r';
                }
                lastchar    = c;
                modbuf[k++] = c;
                --rem;
            }
            _uart->write(modbuf, k);
        }
        return length;
    } else {
        return _uart->write(buffer, length);
    }
}

int UartChannel::available() {
    return _uart->available();
}

int UartChannel::peek() {
    return _uart->peek();
}

int UartChannel::rx_buffer_available() {
    return _uart->rx_buffer_available();
}

bool UartChannel::realtimeOkay(char c) {
    return _lineedit->realtime(c);
}

bool UartChannel::lineComplete(char* line, char c) {
    if (_lineedit->step(c)) {
        // A whole line arrived, so there is a real peer on this port.
        _peer_spoke = true;
        _linelen        = _lineedit->finish();
        _line[_linelen] = '\0';
        strcpy(line, _line);
        _linelen = 0;
        return true;
    }
    return false;
}

int UartChannel::read() {
    auto c = _uart->read();
    if (c == 0x11) {
        // 0x11 is XON.  If we receive that, it is a request to use software flow control
        // 0 0 means use default values from uart.cpp
        _uart->setSwFlowControl(true, 0, 0);
        return -1;
    }
    return c;
}

void UartChannel::flushRx() {
    _uart->flushRx();
    Channel::flushRx();
}

size_t UartChannel::timedReadBytes(char* buffer, size_t length, TickType_t timeout) {
    size_t remlen = length;

    // It is likely that _queue will be empty because timedReadBytes() is only
    // used in situations where the UART is not receiving GCode commands
    // and Grbl realtime characters.
    uint8_t queued = 0;
    while (remlen && try_pop_queued_byte(queued)) {
        *buffer++ = queued;
        --remlen;
    }

    auto thislen = _uart->timedReadBytes(buffer, remlen, timeout);
    remlen -= thislen;

    return length - remlen;
}

void UartChannel::out(const std::string& s, const char* tag) {
    log_stream(*this, "[" << tag << s);
}

void UartChannel::out_acked(const std::string& s, const char* tag) {
    log_stream(*this, "[" << tag << s);
}

void UartChannel::beginJSON(const char* json_tag) {
    //    out_acked(json_tag, "JSONBEGIN:");
}
void UartChannel::endJSON(const char* json_tag) {
    //    out_acked(json_tag, "JSONEND:");
}

void UartChannel::registerEvent(pinnum_t pinnum, InputPin* obj) {
    Channel::registerEvent(pinnum, obj);  // Establish the handler function first
    _uart->registerInputPin(pinnum, obj);
}

bool UartChannel::setAttr(pinnum_t index, bool* value, const std::string& attrString) {
    out(attrString, "EXP:");
    _ackwait = 1;
    for (size_t i = 0; i < 75; i++) {
        pollLine(nullptr);
        if (_ackwait < 1) {
            return _ackwait == 0;
        }
        delay_ms(1);
    }
    _ackwait = 0;
    log_error("IO Expander is unresponsive");
    return false;
}

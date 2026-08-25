// Copyright (c) 2022 Mitch Bradley
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "FluidPath.h"
#include "Driver/sdspi.h"
#include "Config.h"
#include "Error.h"
#include "Machine/MachineConfig.h"
#include "FluidError.hpp"
#include "HashFS.h"
#include "string_util.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "Arduino.h"

Volume SD { "sd" };
Volume LocalFS { "localfs" };

// Locking rules for the SD mount state:
//
//   sd_cache_mutex protects cached_mount.  sd_mount_mutex protects
//   sd_refcnt and the sd_mount()/sd_unmount() calls.  When both are
//   needed, sd_cache_mutex is always taken first, since the FluidPath
//   constructor holds it while constructing an SDMountState, whose
//   constructor takes sd_mount_mutex.  ~SDMountState() takes only
//   sd_mount_mutex, so nothing acquires the two in the opposite order.
static SemaphoreHandle_t sd_cache_mutex = xSemaphoreCreateMutex();
static SemaphoreHandle_t sd_mount_mutex = xSemaphoreCreateMutex();

// The mount state of the most recent FluidPath, so that concurrent
// paths on the SD card share a single mount.
static std::weak_ptr<SDMountState> cached_mount;

// The number of live SDMountState objects.  The card stays mounted while
// this is nonzero.  This has to be counted explicitly rather than read off
// the shared_ptr use count, because a use count reaching zero and the
// matching ~SDMountState() body running are two separate events.  Another
// task can construct a new SDMountState in between, and without this
// counter the still-pending destructor would then unmount the card that
// the new owner considers mounted.  cached_mount would go on handing that
// state to every later FluidPath, so the card would never be remounted and
// all SD access would fail until the next reboot.
static uint32_t sd_refcnt = 0;

namespace {
    // RAII so a throw while mounting cannot leak a mutex.
    class LockGuard {
        SemaphoreHandle_t _mutex;

    public:
        explicit LockGuard(SemaphoreHandle_t mutex) : _mutex(mutex) { xSemaphoreTake(_mutex, portMAX_DELAY); }
        ~LockGuard() { xSemaphoreGive(_mutex); }
        LockGuard(const LockGuard&)            = delete;
        LockGuard& operator=(const LockGuard&) = delete;
    };
}

// SDMountState manages SD card mount/unmount lifecycle
// It is instantiated as a shared_ptr that is automatically
// reference-counted.  The first instance to be constructed mounts
// the SD card and the last one to be destroyed unmounts it.
SDMountState::SDMountState() {
    LockGuard guard(sd_mount_mutex);
    if (sd_refcnt == 0) {
        auto ec = sd_mount();
        if (ec) {
            throw stdfs::filesystem_error { "Failed to mount SD card", ec };
        }
    }
    ++sd_refcnt;
}

SDMountState::~SDMountState() {
    LockGuard guard(sd_mount_mutex);
    if (sd_refcnt && --sd_refcnt == 0) {
        sd_unmount();
    }
}

const std::string FluidPath::canonPath(std::string_view filename, const Volume& defaultFs) {
    std::string ret;
    //    log_debug("fn " << filename << " fs " << defaultFs.name);
    if (filename.empty()) {
        ret = defaultFs.prefix;
        return ret;
    }

    // A std::filesystem::path with a trailing slash (except for just "/")
    // is considered to be a path with an empty final component, not a
    // final directory component.  That causes problems when trying to
    // determine the file type, so we remove trailing slases.
    while (filename.length() > 1 && filename[filename.length() - 1] == '/') {
        filename.remove_suffix(1);
    }

    if (filename[0] == '/') {
        auto        pos = filename.find('/', 1);
        std::string fsname;
        std::string tail;
        if (pos != std::string::npos) {
            fsname = filename.substr(1, pos - 1);
            tail   = filename.substr(pos);
        } else {
            fsname = filename.substr(1);
            tail   = "";
        }
        //            log_debug("FS " << fsname << " tail " << tail << " fn " << filename);
        if (string_util::equal_ignore_case(fsname, LocalFS.name) || string_util::equal_ignore_case(fsname, "spiffs") ||
            string_util::equal_ignore_case(fsname, "littlefs")) {
            ret = LocalFS.prefix;
            ret += tail;
            return ret;
        }
        if (string_util::equal_ignore_case(fsname, SD.name)) {
            ret = SD.prefix;
            ret += tail;
            return ret;
        }
        ret = defaultFs.prefix;
        ret += filename;
        return ret;
    }

    // The pathname did not begin with /
    ret = defaultFs.prefix;
    ret += "/";
    ret += filename;
    return ret;
}

FluidPath::FluidPath(const std::string_view name, const Volume& fs, std::error_code* ecptr) : stdfs::path(canonPath(name, fs)) {
    auto mount = *(++begin());  // Use the path iterator to get the first component
    _isSD      = mount == "sd";

    if (_isSD) {
        if (!config->_sdCard->config_ok) {
            std::error_code ec = FluidError::SDNotConfigured;
            if (ecptr) {
                *ecptr = ec;
                return;
            }
            throw stdfs::filesystem_error { "SD card is inaccessible", name, ec };
        }
        try {
            // Hold sd_cache_mutex across the test and the construction, so that
            // two tasks cannot both find cached_mount expired and race to mount.
            LockGuard guard(sd_cache_mutex);
            // Try to reuse existing mount state if another FluidPath still owns it
            if (auto cached = cached_mount.lock()) {
                _sd_mount_state = cached;
            } else {
                _sd_mount_state = std::make_shared<SDMountState>();
                cached_mount    = _sd_mount_state;
            }
        } catch (const stdfs::filesystem_error& ex) {
            if (ecptr) {
                *ecptr = ex.code();
                return;
            }
            throw stdfs::filesystem_error { "SD card is inaccessible", name, ex.code() };
        }
    }
}

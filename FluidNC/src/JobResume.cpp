// Copyright (c) 2026 - Gandalf van Schnaufenberg
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "JobResume.h"

#include "Job.h"
#include "Planner.h"
#include "GCode.h"
#include "System.h"
#include "Logging.h"
#include "FluidPath.h"
#include "Machine/MachineConfig.h"

#include <cstdio>
#include <cstring>
#include <Arduino.h>  // millis()

namespace JobResume {
    uint32_t interval_ms = 10000;

    namespace {
        // Two slots written alternately.  A power cut during a write can only
        // damage the slot being written; the other still holds the previous
        // good record, so there is always something to come back to.  This is
        // why the record is not simply rewritten in place.
        const char* slot_path[2] = { "/sd/.fnc_resume0", "/sd/.fnc_resume1" };

        constexpr uint32_t kMagic   = 0x464e4352;  // "FNCR"
        constexpr uint16_t kVersion = 1;

        struct __attribute__((packed)) Record {
            uint32_t magic;
            uint16_t version;
            uint16_t axes;
            uint32_t seq;  // higher wins
            uint64_t offset;
            int32_t  line;
            uint32_t file_size;
            char     path[128];
            float    mpos[MAX_N_AXIS];
            float    coord_offset[MAX_N_AXIS];
            uint8_t  coord_select;
            float    feed_rate;
            float    spindle_speed;
            uint8_t  spindle;
            uint8_t  coolant;
            uint8_t  units;
            uint8_t  distance;
            uint32_t tool;
            uint32_t crc;  // over every byte above
        };

        uint32_t crc32(const uint8_t* data, size_t len) {
            uint32_t crc = 0xffffffff;
            for (size_t i = 0; i < len; ++i) {
                crc ^= data[i];
                for (int b = 0; b < 8; ++b) {
                    crc = (crc >> 1) ^ (0xedb88320u & (-(int32_t)(crc & 1)));
                }
            }
            return ~crc;
        }

        uint32_t record_crc(const Record& r) { return crc32(reinterpret_cast<const uint8_t*>(&r), offsetof(Record, crc)); }

        uint32_t s_seq          = 0;
        uint32_t s_last_write   = 0;
        bool     s_have_written = false;

        bool load_slot(int slot, Record& r) {
            FILE* fd = fopen(slot_path[slot], "rb");
            if (!fd) {
                return false;
            }
            size_t got = fread(&r, 1, sizeof r, fd);
            fclose(fd);
            return got == sizeof r && r.magic == kMagic && r.version == kVersion && r.crc == record_crc(r);
        }

        // Writing opens a second descriptor while the job file holds the first.
        // sd_mount() allows enough for both plus a WebUI request; see the note
        // where max_files is chosen.
        bool store(const Record& r, int slot) {
            FILE* fd = fopen(slot_path[slot], "wb");
            if (!fd) {
                return false;
            }
            bool ok = fwrite(&r, 1, sizeof r, fd) == sizeof r;
            ok      = (fflush(fd) == 0) && ok;
            ok      = (fclose(fd) == 0) && ok;
            return ok;
        }
    }

    void poll() {
        if (interval_ms == 0) {
            return;
        }

        Channel* job = Job::channel();
        if (!job) {
            return;  // nothing running
        }

        uint32_t now = millis();
        if (s_have_written && (now - s_last_write) < interval_ms) {
            return;
        }

        // The block the machine is executing now, which is what the recorded
        // position describes.  With the planner empty there is nothing in
        // flight worth a checkpoint - the previous one still stands.
        plan_block_t* block = plan_get_current_block();
        if (!block) {
            return;
        }

        // Continue the sequence already on the card rather than restarting at
        // 1.  s_seq is zero after a reboot, and a power cut is exactly when a
        // reboot happens: without this, the first checkpoints of the next job
        // would be numbered below the stale ones still stored, and read() would
        // hand back the *old* job as the newer record.
        if (!s_have_written) {
            for (int slot = 0; slot < 2; ++slot) {
                Record prev;
                if (load_slot(slot, prev) && prev.seq > s_seq) {
                    s_seq = prev.seq;
                }
            }
        }

        Record r;
        memset(&r, 0, sizeof r);
        r.magic   = kMagic;
        r.version = kVersion;
        r.axes    = Machine::Axes::_numberAxis;
        r.seq     = ++s_seq;
        r.offset  = block->file_offset;
        r.line    = block->line_number;

        r.file_size = job->size();
        strncpy(r.path, job->name(), sizeof r.path - 1);

        float* mpos = get_mpos();
        for (size_t i = 0; i < MAX_N_AXIS; ++i) {
            r.mpos[i]         = mpos[i];
            r.coord_offset[i] = gc_state.coord_offset[i];
        }
        r.coord_select  = static_cast<uint8_t>(gc_state.modal.coord_select);
        r.feed_rate     = gc_state.feed_rate;
        r.spindle_speed = gc_state.spindle_speed;
        r.spindle       = static_cast<uint8_t>(gc_state.modal.spindle);
        // CoolantState is a two-bit field, not an integer; pack it by hand.
        r.coolant       = (gc_state.modal.coolant.Mist ? 1 : 0) | (gc_state.modal.coolant.Flood ? 2 : 0);
        r.units         = static_cast<uint8_t>(gc_state.modal.units);
        r.distance      = static_cast<uint8_t>(gc_state.modal.distance);
        r.tool          = gc_state.selected_tool;
        r.crc           = record_crc(r);

        // Alternate slots so the previous good record always survives.
        if (store(r, s_seq & 1)) {
            s_last_write   = now;
            s_have_written = true;
        } else {
            // Do not retry every pass; a card that cannot be written now is
            // unlikely to recover within a few milliseconds, and hammering it
            // would slow the job down for nothing.
            s_last_write   = now;
            s_have_written = true;
            log_warn("Resume checkpoint could not be written");
        }
    }

    bool read(Checkpoint& out) {
        Record best;
        bool   found = false;
        for (int slot = 0; slot < 2; ++slot) {
            Record r;
            if (load_slot(slot, r) && (!found || r.seq > best.seq)) {
                best  = r;
                found = true;
            }
        }
        if (!found) {
            return false;
        }

        out.path      = best.path;
        out.offset    = static_cast<size_t>(best.offset);
        out.line      = best.line;
        out.file_size = best.file_size;
        for (size_t i = 0; i < MAX_N_AXIS; ++i) {
            out.mpos[i]         = best.mpos[i];
            out.coord_offset[i] = best.coord_offset[i];
        }
        out.coord_select  = best.coord_select;
        out.feed_rate     = best.feed_rate;
        out.spindle_speed = best.spindle_speed;
        out.spindle       = best.spindle;
        out.coolant       = best.coolant;
        out.units         = best.units;
        out.distance      = best.distance;
        out.tool          = best.tool;
        return true;
    }

    void clear() {
        for (int slot = 0; slot < 2; ++slot) {
            remove(slot_path[slot]);
        }
        s_have_written = false;
        s_seq          = 0;
    }
}

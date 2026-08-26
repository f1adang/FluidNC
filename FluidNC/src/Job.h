// Copyright (c) 2024 - Mitch Bradley
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Channel.h"
#include <vector>

class JobSource {
private:
    Channel*                     _channel;
    std::map<std::string, float> _local_params;

public:
    JobSource(Channel* channel) : _channel(channel) {}
    bool get_param(const std::string& name, float& value) {
        auto it = _local_params.find(name);
        if (it == _local_params.end()) {
            return false;
        }
        value = it->second;
        return true;
    }
    bool set_param(const std::string& name, float value) {
        _local_params[name] = value;
        return true;
    }
    bool param_exists(const std::string& name) { return _local_params.count(name) != 0; }

    // Expose local parameters for enumeration
    const std::map<std::string, float>& local_params() const { return _local_params; }

    void   save() { _channel->save(); }
    void   restore() { _channel->restore(); }
    size_t position() { return _channel->position(); }
    void   set_position(size_t pos) { _channel->set_position(pos); }
    size_t lineNumber() { return _channel->lineNumber(); }
    void   setLineNumber(size_t line_number) { _channel->setLineNumber(line_number); }

    Channel* channel() { return _channel; }

    ~JobSource() { delete _channel; }
};

class Job {
private:
    // The channel that launched the outermost job.  Job output - "ok",
    // error reports and the like - goes there instead of to the job's own
    // input channel.  It is private because it must be read via leader(),
    // which copes with the channel going away while the job runs.
    static Channel* _leader;

    static void pop();
    static void release_leader();

public:
    // The channel to which job output should be sent, or nullptr if there
    // is none.  A job outlives its leader: a WebSocket leader disappears as
    // soon as WiFi connectivity is lost, and that must not stop the job.
    static Channel* leader();

    static bool active();

    static void       save();
    static void       restore();
    static void       nest(Channel* in_channel, Channel* out_channel);
    static void       unnest();
    static void       abort();
    static JobSource* source();

    static bool     get_param(const std::string& name, float& value);
    static bool     set_param(const std::string& name, float value);
    static bool     param_exists(const std::string& name);
    static Channel* channel();

    // Expose access to jobs stack for listing local parameters
    static const std::vector<JobSource*>& jobs_stack();
};

void list_local_params(Channel& out);

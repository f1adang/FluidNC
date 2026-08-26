// Copyright (c) 2024 - Mitch Bradley
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Job.h"
#include <map>
#include <vector>

std::vector<JobSource*> job;

Channel* Job::_leader = nullptr;

// Job output goes to the channel that launched the job, not to the job's own
// input channel.  That leader channel can die while the job is still running -
// a WebSocket disappears as soon as WiFi connectivity is lost - so we hold a
// processing reference to it for the duration of the job.  The reference keeps
// the channel object alive, hence the leader pointer valid, until we are done
// with it; writes to a channel that is closing are discarded by the channel
// itself.  Without the reference the channel would be deleted out from under
// the job, and the resulting use of freed memory would crash the controller
// mid-job.
void Job::release_leader() {
    if (_leader) {
        _leader->release_processing_ref();
        _leader = nullptr;
    }
}

Channel* Job::leader() {
    if (_leader && _leader->is_closing()) {
        // Stop reporting to a channel that has gone away, but keep the job
        // running.  Losing e.g. the WebUI connection is not a reason to
        // abandon a job that is already underway.
        log_info("Job output channel " << _leader->name() << " closed; the job continues");
        release_leader();
    }
    return _leader;
}

bool Job::active() {
    return !job.empty();
}

JobSource* Job::source() {
    return job.empty() ? nullptr : job.back();
}

// save() and restore() are use to close/reopen an SD file atop the job stack
// before trying to open a nested SD file.  The reason for that is because
// the number of simultaneously-open SD files is limited to conserve RAM.
void Job::save() {
    if (active()) {
        job.back()->save();
    }
}
void Job::restore() {
    if (active()) {
        job.back()->restore();
    }
}
void Job::nest(Channel* in_channel, Channel* out_channel) {
    auto source = new JobSource(in_channel);
    // At info level so that the console records when a job starts, giving the
    // "job sent" or "Job aborted" line at the other end something to pair with.
    log_info("Job started: " << in_channel->name() << (job.empty() ? "" : " (nested)"));
    if (out_channel && job.empty()) {
        release_leader();
        // If the reference cannot be taken, the channel is already on its way
        // out, so the job runs without a leader rather than with a stale one.
        if (out_channel->try_acquire_processing_ref()) {
            _leader = out_channel;
        }
    }
    job.push_back(source);
}
void Job::pop() {
    auto source = job.back();
    job.pop_back();
    delete source;
    if (!active()) {
        release_leader();
    }
}
void Job::unnest() {
    if (active()) {
        pop();
        restore();
    }
}

void Job::abort() {
    // Kill all active jobs
    while (active()) {
        pop();
    }
}

bool Job::get_param(const std::string& name, float& value) {
    return job.back()->get_param(name, value);
}
bool Job::set_param(const std::string& name, float value) {
    return job.back()->set_param(name, value);
}
bool Job::param_exists(const std::string& name) {
    return job.back()->param_exists(name);
}
Channel* Job::channel() {
    return job.back()->channel();
}

const std::vector<JobSource*>& Job::jobs_stack() {
    return job;
}

void list_local_params(Channel& out) {
    const auto& job_stack = Job::jobs_stack();
    if (job_stack.empty()) {
        log_info_to(out, "No active jobs - no local parameters");
        return;
    }

    int depth = 0;
    for (auto source : job_stack) {
        const auto& local_params = source->local_params();
        if (local_params.empty()) {
            log_info_to(out, "Job depth " << depth << " - No local parameters");
        } else {
            log_info_to(out, "Job depth " << depth << " - Local Parameters");
            for (const auto& param : local_params) {
                // Format: parameter_name = value
                log_info_to(out, param.first << " = " << param.second);
            }
        }
        depth++;
    }
}

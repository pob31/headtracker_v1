/* htk::Transport — the byte pipe under the parser (PROTOCOL.md §1.1).
 *
 * The dongle enumerates as a USB CDC-ACM virtual serial port, so the transport
 * is deliberately dumb: it moves bytes and knows nothing about framing. That
 * keeps the replay backends (a file, an in-memory buffer) behind the identical
 * interface, which is what makes the whole stack testable with no hardware.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace htk {

class Transport {
public:
    virtual ~Transport() = default;

    /* Opens `target`. On failure returns false and sets `err` to a
     * human-readable reason (shown to users, so name the thing that failed). */
    virtual bool open(const std::string &target, std::string &err) = 0;
    virtual void close() = 0;
    virtual bool is_open() const = 0;

    /* Returns the number of bytes read, 0 on timeout (NOT an error), or -1 if
     * the port died. MUST return as soon as ANY bytes are available: batching
     * reads into fixed-size chunks is exactly the mistake that adds
     * milliseconds to a budget measured in single digits. */
    virtual int read(uint8_t *buf, size_t cap, int timeout_ms) = 0;

    virtual bool write(const uint8_t *data, size_t len) = 0;
};

/* Real serial port. `target` is a platform path: "COM7" or "\\\\.\\COM7" on
 * Windows, "/dev/ttyACM0" or "/dev/cu.usbmodem1401" elsewhere. */
std::unique_ptr<Transport> make_serial();

/* Replays a captured byte stream — an htgen dump — as though it were a
 * dongle. `bytes_per_second <= 0` replays as fast as the reader asks (useful
 * in tests); a positive value paces it so rate and staleness logic behave
 * realistically. `target` "-" reads standard input. Writes are discarded.
 * A file that runs out reports EOF as a closed port, which is what a consumer
 * would see if the dongle were unplugged. */
std::unique_ptr<Transport> make_file(double bytes_per_second = 0.0, bool loop = false);

/* Wraps any transport and records every byte READ to `record_path` (the
 * device->host stream only — exactly what --replay consumes later). The file
 * is opened on open(), flushed per read so a killed process still leaves a
 * complete capture. */
std::unique_ptr<Transport> make_tee(std::unique_ptr<Transport> inner,
                                    const std::string &record_path);

/* In-memory transport for tests: `feed()` appends bytes for the reader to
 * consume, `written()` returns everything the client has sent. */
class MemTransport : public Transport {
public:
    bool open(const std::string &target, std::string &err) override;
    void close() override;
    bool is_open() const override;
    int read(uint8_t *buf, size_t cap, int timeout_ms) override;
    bool write(const uint8_t *data, size_t len) override;

    /* Thread-safe: tests feed from the test thread while the client's reader
     * thread consumes. */
    void feed(const uint8_t *data, size_t len);
    void feed(const std::vector<uint8_t> &v);
    std::vector<uint8_t> written() const;
    void clear_written();

    /* Make the next read() report a dead port, as an unplug would. */
    void fail();

private:
    struct Impl;
    std::shared_ptr<Impl> impl_;
};

/* ---- Port enumeration and dongle discovery (PROTOCOL.md §1.7) ---- */

struct PortInfo {
    std::string path;        /* what to hand to Transport::open() */
    std::string description; /* friendly name / by-id link, may be empty */
    std::string hwid;        /* platform hardware id, may be empty */
};

/* Every serial port the OS currently reports. Ports that are known to be
 * expensive or dangerous to probe (Bluetooth SPP, which can block for seconds
 * on open) are excluded rather than returned. */
std::vector<PortInfo> enumerate_ports();

/* True when a port's description/hwid suggests it IS the dongle, used to probe
 * likely candidates before unrelated hardware. */
bool looks_like_dongle(const PortInfo &p);

/* Opens each candidate, sends HELLO and waits for a compatible HELLO_RESP from
 * a device that identifies as a dongle. Returns the winning path, or an empty
 * string. Ports that look like a dongle are tried first. */
std::string find_dongle(int probe_timeout_ms = 300);

/* Probe one specific port. Exposed so a UI can explain why an explicitly
 * configured port did not work. */
bool probe_port(const std::string &path, int probe_timeout_ms, std::string &err);

} // namespace htk

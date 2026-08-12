/* Transport backends: serial (Win32 / POSIX), file replay, in-memory.
 *
 * Deliberately ONE translation unit with internal platform guards rather than
 * transport_win32.cpp + transport_posix.cpp: consumers that build through
 * Projucer compile every listed file on every exporter, and per-file platform
 * exclusion there is awkward and easy to get wrong. A self-guarding TU keeps
 * the consumer's file list flat and makes the CMake and Projucer builds
 * behave identically.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#if defined(_MSC_VER) && !defined(_CRT_SECURE_NO_WARNINGS)
#define _CRT_SECURE_NO_WARNINGS /* std::fopen on a caller-supplied path */
#endif

#include "headtracker/transport.hpp"

#include "headtracker/commands.hpp"
#include "headtracker/parser.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <climits>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <deque>
#include <mutex>
#include <thread>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <setupapi.h>
#include <fcntl.h>
#include <io.h>
#if defined(_MSC_VER)
/* Declare the dependency here rather than relying on every consumer's build
 * system to add it. Projucer-based consumers in particular have no reliable
 * place to put it, and a missing setupapi is a link error a long way from its
 * cause. The CMake target links it too, for MinGW and for static analysis. */
#pragma comment(lib, "setupapi.lib")
#pragma comment(lib, "advapi32.lib")
#endif
#else
#include <dirent.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace htk {
namespace {

uint64_t now_ms()
{
    using namespace std::chrono;
    return (uint64_t) duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

} // namespace

/* ===================================================================== */
/* MemTransport                                                          */
/* ===================================================================== */

struct MemTransport::Impl {
    mutable std::mutex mu;
    std::deque<uint8_t> in;
    std::vector<uint8_t> out;
    bool open = false;
    bool failed = false;
};

bool MemTransport::open(const std::string &, std::string &)
{
    if (!impl_) {
        impl_ = std::make_shared<Impl>();
    }
    std::lock_guard<std::mutex> lk(impl_->mu);
    impl_->open = true;
    impl_->failed = false;
    return true;
}

void MemTransport::close()
{
    if (!impl_) {
        return;
    }
    std::lock_guard<std::mutex> lk(impl_->mu);
    impl_->open = false;
}

bool MemTransport::is_open() const
{
    if (!impl_) {
        return false;
    }
    std::lock_guard<std::mutex> lk(impl_->mu);
    return impl_->open;
}

int MemTransport::read(uint8_t *buf, size_t cap, int timeout_ms)
{
    if (!impl_) {
        return -1;
    }
    const uint64_t deadline = now_ms() + (uint64_t) (timeout_ms > 0 ? timeout_ms : 0);
    for (;;) {
        {
            std::lock_guard<std::mutex> lk(impl_->mu);
            if (impl_->failed || !impl_->open) {
                return -1;
            }
            if (!impl_->in.empty()) {
                const size_t n = (std::min)(cap, impl_->in.size());
                for (size_t i = 0; i < n; ++i) {
                    buf[i] = impl_->in[i];
                }
                impl_->in.erase(impl_->in.begin(), impl_->in.begin() + (ptrdiff_t) n);
                return (int) n;
            }
        }
        if (now_ms() >= deadline) {
            return 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

bool MemTransport::write(const uint8_t *data, size_t len)
{
    if (!impl_) {
        return false;
    }
    std::lock_guard<std::mutex> lk(impl_->mu);
    if (!impl_->open) {
        return false;
    }
    impl_->out.insert(impl_->out.end(), data, data + len);
    return true;
}

void MemTransport::feed(const uint8_t *data, size_t len)
{
    if (!impl_) {
        impl_ = std::make_shared<Impl>();
    }
    std::lock_guard<std::mutex> lk(impl_->mu);
    impl_->in.insert(impl_->in.end(), data, data + len);
}

void MemTransport::feed(const std::vector<uint8_t> &v)
{
    if (!v.empty()) {
        feed(v.data(), v.size());
    }
}

std::vector<uint8_t> MemTransport::written() const
{
    if (!impl_) {
        return {};
    }
    std::lock_guard<std::mutex> lk(impl_->mu);
    return impl_->out;
}

void MemTransport::clear_written()
{
    if (!impl_) {
        return;
    }
    std::lock_guard<std::mutex> lk(impl_->mu);
    impl_->out.clear();
}

void MemTransport::fail()
{
    if (!impl_) {
        impl_ = std::make_shared<Impl>();
    }
    std::lock_guard<std::mutex> lk(impl_->mu);
    impl_->failed = true;
}

/* ===================================================================== */
/* FileTransport — replay an htgen capture as though it were a dongle     */
/* ===================================================================== */

namespace {

class FileTransport : public Transport {
public:
    FileTransport(double bps, bool loop) : bps_(bps), loop_(loop) {}

    ~FileTransport() override { close(); }

    bool open(const std::string &target, std::string &err) override
    {
        close();
        if (target == "-") {
            f_ = stdin;
            owns_ = false;
#if defined(_WIN32)
            /* stdin must be binary or 0x1A truncates the stream and CRLF
             * translation corrupts COBS-encoded bytes. */
            _setmode(_fileno(stdin), _O_BINARY);
#endif
        } else {
            f_ = std::fopen(target.c_str(), "rb");
            owns_ = true;
            if (f_ == nullptr) {
                err = "cannot open replay file '" + target + "'";
                return false;
            }
        }
        path_ = target;
        start_ms_ = now_ms();
        delivered_ = 0;
        eof_ = false;
        return true;
    }

    void close() override
    {
        if (f_ != nullptr && owns_) {
            std::fclose(f_);
        }
        f_ = nullptr;
        owns_ = false;
    }

    bool is_open() const override { return f_ != nullptr && !eof_; }

    int read(uint8_t *buf, size_t cap, int timeout_ms) override
    {
        if (f_ == nullptr) {
            return -1;
        }

        /* Pace to the nominal byte rate so a consumer's rate and staleness
         * logic sees something like real timing rather than the whole capture
         * in one gulp. */
        if (bps_ > 0.0) {
            const uint64_t elapsed = now_ms() - start_ms_;
            const double allowed = bps_ * (double) elapsed / 1000.0;
            if ((double) delivered_ >= allowed) {
                const double deficit = (double) delivered_ - allowed;
                int wait = (int) (deficit * 1000.0 / bps_) + 1;
                if (timeout_ms >= 0 && wait > timeout_ms) {
                    wait = timeout_ms;
                }
                if (wait > 0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(wait));
                }
                return 0;
            }
            size_t budget = (size_t) (allowed - (double) delivered_);
            if (budget == 0) {
                budget = 1; /* always make forward progress */
            }
            cap = (std::min)(cap, budget);
        }

        const size_t n = std::fread(buf, 1, cap, f_);
        if (n == 0) {
            if (loop_ && owns_ && std::fseek(f_, 0, SEEK_SET) == 0) {
                /* The seq/t_us discontinuity at the seam is useful test
                 * material, not a defect: it is what a tracker reboot looks
                 * like on the wire. */
                start_ms_ = now_ms();
                delivered_ = 0;
                return 0;
            }
            eof_ = true;
            return -1; /* capture exhausted == the device went away */
        }
        delivered_ += n;
        return (int) n;
    }

    bool write(const uint8_t *, size_t) override { return true; } /* discarded */

private:
    std::FILE *f_ = nullptr;
    bool owns_ = false;
    std::string path_;
    double bps_ = 0.0;
    bool loop_ = false;
    uint64_t start_ms_ = 0;
    uint64_t delivered_ = 0;
    bool eof_ = false;
};

} // namespace

std::unique_ptr<Transport> make_file(double bytes_per_second, bool loop)
{
    return std::unique_ptr<Transport>(new FileTransport(bytes_per_second, loop));
}

/* ===================================================================== */
/* SerialPort                                                            */
/* ===================================================================== */

namespace {

#if defined(_WIN32)

class SerialPort : public Transport {
public:
    ~SerialPort() override { close(); }

    bool open(const std::string &target, std::string &err) override
    {
        close();

        /* The \\.\ prefix is mandatory for COM10 and above, and harmless
         * below it. */
        std::string path = target;
        if (path.compare(0, 4, "\\\\.\\") != 0) {
            path = "\\\\.\\" + path;
        }

        h_ = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                         OPEN_EXISTING, 0, nullptr);
        if (h_ == INVALID_HANDLE_VALUE) {
            h_ = nullptr;
            err = "cannot open " + target + " (error " + std::to_string((unsigned long) GetLastError()) + ")";
            return false;
        }

        SetupComm(h_, 16384, 4096);
        PurgeComm(h_, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);

        DCB dcb;
        std::memset(&dcb, 0, sizeof(dcb));
        dcb.DCBlength = sizeof(dcb);
        if (!GetCommState(h_, &dcb)) {
            err = "GetCommState failed on " + target;
            close();
            return false;
        }
        /* CDC-ACM ignores line coding (it is a USB pipe, not a UART) but the
         * settings must still be legal or SetCommState refuses them. */
        dcb.BaudRate = CBR_115200;
        dcb.ByteSize = 8;
        dcb.Parity = NOPARITY;
        dcb.StopBits = ONESTOPBIT;
        dcb.fBinary = TRUE;
        dcb.fDtrControl = DTR_CONTROL_ENABLE;
        dcb.fRtsControl = RTS_CONTROL_ENABLE;
        dcb.fOutxCtsFlow = FALSE;
        dcb.fOutxDsrFlow = FALSE;
        dcb.fDsrSensitivity = FALSE;
        dcb.fOutX = FALSE;
        dcb.fInX = FALSE;
        dcb.fAbortOnError = FALSE;
        if (!SetCommState(h_, &dcb)) {
            err = "SetCommState failed on " + target;
            close();
            return false;
        }

        set_timeouts(20);
        return true;
    }

    void close() override
    {
        if (h_ != nullptr) {
            CloseHandle(h_);
            h_ = nullptr;
        }
    }

    bool is_open() const override { return h_ != nullptr; }

    int read(uint8_t *buf, size_t cap, int timeout_ms) override
    {
        if (h_ == nullptr) {
            return -1;
        }
        if (timeout_ms != timeout_set_) {
            set_timeouts(timeout_ms);
        }

        DWORD got = 0;
        if (!ReadFile(h_, buf, (DWORD) cap, &got, nullptr)) {
            const DWORD e = GetLastError();
            /* A vanished device surfaces as one of these rather than a clean
             * zero-byte read. */
            if (e == ERROR_OPERATION_ABORTED || e == ERROR_ACCESS_DENIED ||
                e == ERROR_BAD_COMMAND || e == ERROR_DEVICE_REMOVED ||
                e == ERROR_INVALID_HANDLE || e == ERROR_GEN_FAILURE) {
                return -1;
            }
            COMSTAT st;
            DWORD errs = 0;
            ClearCommError(h_, &errs, &st);
            return 0;
        }
        return (int) got;
    }

    bool write(const uint8_t *data, size_t len) override
    {
        if (h_ == nullptr) {
            return false;
        }
        size_t sent = 0;
        while (sent < len) {
            DWORD wrote = 0;
            if (!WriteFile(h_, data + sent, (DWORD) (len - sent), &wrote, nullptr)) {
                return false;
            }
            if (wrote == 0) {
                return false;
            }
            sent += wrote;
        }
        return true;
    }

private:
    void set_timeouts(int timeout_ms)
    {
        /* The documented "return immediately with whatever is buffered,
         * otherwise wait up to ReadTotalTimeoutConstant for the FIRST byte"
         * recipe, and the reason this class exists as more than a CreateFile
         * wrapper: a naive configuration adds 1-15 ms of buffering to every
         * read, which alone would blow the whole motion-to-USB budget.
         * Note the common MAXDWORD/0/0 variant returns instantly ALWAYS,
         * turning the reader into a spin loop -- do not "simplify" to it. */
        COMMTIMEOUTS t;
        std::memset(&t, 0, sizeof(t));
        t.ReadIntervalTimeout = MAXDWORD;
        t.ReadTotalTimeoutMultiplier = MAXDWORD;
        t.ReadTotalTimeoutConstant = (DWORD) (timeout_ms > 0 ? timeout_ms : 0);
        t.WriteTotalTimeoutConstant = 500;
        t.WriteTotalTimeoutMultiplier = 0;
        SetCommTimeouts(h_, &t);
        timeout_set_ = timeout_ms;
    }

    HANDLE h_ = nullptr;
    int timeout_set_ = -1;
};

#else /* POSIX */

class SerialPort : public Transport {
public:
    ~SerialPort() override { close(); }

    bool open(const std::string &target, std::string &err) override
    {
        close();

        /* O_NONBLOCK on open so a port without carrier does not block there;
         * poll() then owns all the timing, which keeps read() semantics
         * identical to the Windows backend. */
        fd_ = ::open(target.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            err = "cannot open " + target + " (" + std::strerror(errno) + ")";
            return false;
        }

#if defined(TIOCEXCL)
        ::ioctl(fd_, TIOCEXCL); /* advisory exclusivity; best effort */
#endif

        struct termios tio;
        if (tcgetattr(fd_, &tio) != 0) {
            err = "tcgetattr failed on " + target;
            close();
            return false;
        }
        cfmakeraw(&tio);
        tio.c_cflag |= (CLOCAL | CREAD);
        tio.c_cflag &= ~(unsigned) CSTOPB;
#if defined(CRTSCTS)
        tio.c_cflag &= ~(unsigned) CRTSCTS;
#endif
        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 0;
        cfsetispeed(&tio, B115200);
        cfsetospeed(&tio, B115200);
        if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
            err = "tcsetattr failed on " + target;
            close();
            return false;
        }
        tcflush(fd_, TCIOFLUSH);
        return true;
    }

    void close() override
    {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    bool is_open() const override { return fd_ >= 0; }

    int read(uint8_t *buf, size_t cap, int timeout_ms) override
    {
        if (fd_ < 0) {
            return -1;
        }

        struct pollfd p;
        p.fd = fd_;
        p.events = POLLIN;
        p.revents = 0;

        const int pr = ::poll(&p, 1, timeout_ms);
        if (pr < 0) {
            return (errno == EINTR) ? 0 : -1;
        }
        if (pr == 0) {
            return 0;
        }
        if ((p.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
            return -1;
        }

        const ssize_t n = ::read(fd_, buf, cap);
        if (n < 0) {
            return (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) ? 0 : -1;
        }
        if (n == 0) {
            return -1; /* CDC device unplugged */
        }
        return (int) n;
    }

    bool write(const uint8_t *data, size_t len) override
    {
        if (fd_ < 0) {
            return false;
        }
        size_t sent = 0;
        while (sent < len) {
            const ssize_t n = ::write(fd_, data + sent, len - sent);
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                    continue;
                }
                return false;
            }
            sent += (size_t) n;
        }
        return true;
    }

private:
    int fd_ = -1;
};

#endif

} // namespace

std::unique_ptr<Transport> make_serial()
{
    return std::unique_ptr<Transport>(new SerialPort());
}

/* ===================================================================== */
/* Enumeration                                                           */
/* ===================================================================== */

namespace {

bool contains_ci(const std::string &hay, const char *needle)
{
    std::string a = hay, b = needle;
    std::transform(a.begin(), a.end(), a.begin(), [](unsigned char c) { return (char) ::tolower(c); });
    std::transform(b.begin(), b.end(), b.begin(), [](unsigned char c) { return (char) ::tolower(c); });
    return a.find(b) != std::string::npos;
}

} // namespace

bool looks_like_dongle(const PortInfo &p)
{
    /* The dongle firmware sets CONFIG_USB_DEVICE_PRODUCT but pins no VID/PID,
     * so the product string is the only hint available. If a PID is ever
     * pinned, match it here first and this becomes deterministic. */
    return contains_ci(p.description, "headtracker") || contains_ci(p.hwid, "headtracker");
}

#if defined(_WIN32)

std::vector<PortInfo> enumerate_ports()
{
    std::vector<PortInfo> out;

    static const GUID kPortsClass = {
        0x4d36e978, 0xe325, 0x11ce, { 0xbf, 0xc1, 0x08, 0x00, 0x2b, 0xe1, 0x03, 0x18 }
    };

    HDEVINFO set = SetupDiGetClassDevsA(&kPortsClass, nullptr, nullptr, DIGCF_PRESENT);
    if (set == INVALID_HANDLE_VALUE) {
        return out;
    }

    SP_DEVINFO_DATA dev;
    std::memset(&dev, 0, sizeof(dev));
    dev.cbSize = sizeof(dev);

    for (DWORD i = 0; SetupDiEnumDeviceInfo(set, i, &dev); ++i) {
        char portName[64] = { 0 };
        HKEY key = SetupDiOpenDevRegKey(set, &dev, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
        if (key != INVALID_HANDLE_VALUE) {
            DWORD type = 0, size = sizeof(portName);
            RegQueryValueExA(key, "PortName", nullptr, &type, (LPBYTE) portName, &size);
            RegCloseKey(key);
        }
        if (portName[0] == 0) {
            continue;
        }

        char friendly[256] = { 0 };
        SetupDiGetDeviceRegistryPropertyA(set, &dev, SPDRP_FRIENDLYNAME, nullptr,
                                          (PBYTE) friendly, sizeof(friendly), nullptr);
        char hwid[256] = { 0 };
        SetupDiGetDeviceRegistryPropertyA(set, &dev, SPDRP_HARDWAREID, nullptr,
                                          (PBYTE) hwid, sizeof(hwid), nullptr);

        PortInfo p;
        p.path = portName;
        p.description = friendly;
        p.hwid = hwid;

        /* Bluetooth SPP ports can block for many seconds on open, which would
         * stall discovery on ordinary user machines. Never probe them. */
        if (contains_ci(p.hwid, "BTHENUM") || contains_ci(p.description, "bluetooth")) {
            continue;
        }

        out.push_back(p);
    }

    SetupDiDestroyDeviceInfoList(set);
    return out;
}

#else

std::vector<PortInfo> enumerate_ports()
{
    std::vector<PortInfo> out;

    auto scan = [&out](const char *dir, const char *prefix, bool resolve_link) {
        DIR *d = ::opendir(dir);
        if (d == nullptr) {
            return;
        }
        while (struct dirent *e = ::readdir(d)) {
            const std::string name = e->d_name;
            if (name == "." || name == "..") {
                continue;
            }
            if (prefix != nullptr && name.compare(0, std::strlen(prefix), prefix) != 0) {
                continue;
            }
            const std::string full = std::string(dir) + "/" + name;

            PortInfo p;
            p.description = name;
            if (resolve_link) {
                char buf[PATH_MAX];
                const ssize_t n = ::readlink(full.c_str(), buf, sizeof(buf) - 1);
                if (n > 0) {
                    buf[n] = 0;
                    std::string t(buf);
                    /* by-id entries are relative links like ../../ttyACM0 */
                    const size_t slash = t.find_last_of('/');
                    p.path = "/dev/" + (slash == std::string::npos ? t : t.substr(slash + 1));
                } else {
                    p.path = full;
                }
                p.hwid = name; /* by-id name carries vendor/product/serial */
            } else {
                p.path = full;
            }
            out.push_back(p);
        }
        ::closedir(d);
    };

    /* /dev/serial/by-id is the good case on Linux: stable across reboots and
     * the USB product string is right there in the name, so the dongle can be
     * recognised without probing anything. */
    scan("/dev/serial/by-id", nullptr, true);

    if (out.empty()) {
        scan("/dev", "ttyACM", false);
    }
    /* macOS: callout devices only. Never /dev/tty.* -- those block on DCD. */
    scan("/dev", "cu.usbmodem", false);

    return out;
}

#endif

/* ===================================================================== */
/* Discovery                                                             */
/* ===================================================================== */

bool probe_port(const std::string &path, int probe_timeout_ms, std::string &err)
{
    auto t = make_serial();
    if (!t->open(path, err)) {
        return false;
    }

    Parser parser;
    bool ok = false;
    bool answered = false;
    parser.on_hello_resp = [&](const htk_hello_resp &h) {
        answered = true;
        ok = compatible(h) && h.device == 1;
    };

    const auto hello = encode_hello();
    const uint64_t deadline = now_ms() + (uint64_t) probe_timeout_ms;
    uint64_t next_hello = 0;
    int sent = 0;

    uint8_t buf[256];
    while (now_ms() < deadline && !answered) {
        /* Re-send rather than trusting one write: a freshly enumerated CDC
         * device can drop the first bytes while its host-side pipe settles. */
        if (now_ms() >= next_hello && sent < 3) {
            if (!t->write(hello.data(), hello.size())) {
                err = "write failed on " + path;
                t->close();
                return false;
            }
            next_hello = now_ms() + 100;
            ++sent;
        }

        const int n = t->read(buf, sizeof(buf), 20);
        if (n < 0) {
            err = "read failed on " + path;
            t->close();
            return false;
        }
        if (n > 0) {
            parser.feed(buf, (size_t) n);
        }
    }

    t->close();

    if (!answered) {
        err = "no HELLO_RESP from " + path;
    } else if (!ok) {
        err = "device on " + path + " is not a compatible dongle";
    }
    return ok;
}

std::string find_dongle(int probe_timeout_ms)
{
    auto ports = enumerate_ports();

    /* Likely candidates first, so the common case costs one probe and an
     * unrelated modem or 3D printer is only poked when nothing else matched. */
    std::stable_partition(ports.begin(), ports.end(),
                          [](const PortInfo &p) { return looks_like_dongle(p); });

    for (const auto &p : ports) {
        std::string err;
        if (probe_port(p.path, probe_timeout_ms, err)) {
            return p.path;
        }
    }
    return {};
}

} // namespace htk

#include <iostream>
#include <vector>
#include <array>
#include <chrono>
#include <iomanip>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <optional>

namespace dp {

struct SBus2 {
    std::array<float, 16> ch;
    bool failsafe;
    bool framelost;
    bool ch18;
    bool flag;
};

class sbus {
public:
    sbus(const std::string& port_name, int baudrate = B115200)
        : port_name_(port_name), baudrate_(baudrate), fd_(-1)
    {}

    bool open_port() {
        fd_ = ::open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            std::cerr << "ポートを開けませんでした: " << port_name_ << std::endl;
            return false;
        }
        struct termios tty;
        memset(&tty, 0, sizeof tty);
        if (tcgetattr(fd_, &tty) != 0) {
            std::cerr << "tcgetattr error" << std::endl;
            ::close(fd_);
            fd_ = -1;
            return false;
        }
        cfsetospeed(&tty, baudrate_);
        cfsetispeed(&tty, baudrate_);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 1;
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            std::cerr << "tcsetattr error" << std::endl;
            ::close(fd_);
            fd_ = -1;
            return false;
        }
        return true;
    }

    void close_port() {
        if (fd_ >= 0) ::close(fd_);
        fd_ = -1;
    }

    std::optional<SBus2> read_frame() {
        std::vector<uint8_t> buffer(1024);
        std::vector<uint8_t> frame_buffer;
        int bytes_read = ::read(fd_, buffer.data(), buffer.size());
        if (bytes_read <= 0) return std::nullopt;

        for (int i = 0; i < bytes_read; ++i) {
            uint8_t byte = buffer[i];
            if (byte == 0x0F) {
                if (!frame_buffer.empty()) {
                    auto sbus_opt = parse_frame(frame_buffer);
                    frame_buffer.clear();
                    if (sbus_opt) return sbus_opt;
                }
            }
            frame_buffer.push_back(byte);
        }
        return std::nullopt;
    }

    static void dump_frame(const SBus2& sbus) {
        for (size_t i = 0; i < sbus.ch.size(); ++i)
            std::cout << "ch" << std::setw(2) << std::setfill('0') << i << ": " << std::showpos << std::fixed << std::setprecision(4) << sbus.ch[i] << std::noshowpos << std::endl;
        std::cout << "FS:" << sbus.failsafe << " FL:" << sbus.framelost << " CH18:" << sbus.ch18 << " FLG:" << sbus.flag << std::endl;
    }

private:
    std::string port_name_;
    int baudrate_;
    int fd_;

    static uint8_t calculate_checksum(const uint8_t* data, size_t len) {
        uint8_t sum = 0;
        for (size_t i = 0; i < len; ++i) sum ^= data[i];
        return sum;
    }

    static std::optional<SBus2> parse_frame(const std::vector<uint8_t>& frame) {
        if (frame.size() < 35) {
            std::cerr << "frame is too short len:" << frame.size() << std::endl;
            return std::nullopt;
        }
        uint8_t sum = frame[34];
        uint8_t calc_sum = calculate_checksum(&frame[1], 33);
        if (sum != calc_sum) {
            std::cerr << "checksum mismatch: frame:" << std::hex << (int)sum << " vs calc:" << (int)calc_sum << std::dec << std::endl;
            return std::nullopt;
        }
        SBus2 sbus;
        for (size_t idx = 0; idx < 16; ++idx) {
            size_t offset = 1 + idx * 2;
            uint16_t value = (frame[offset] << 8) | frame[offset + 1];
            float dvalue = (static_cast<int32_t>(value) - 1024) / 850.0f;
            if (dvalue > 0.99f) dvalue = 1.0f;
            else if (dvalue < -0.99f) dvalue = -1.0f;
            sbus.ch[idx] = dvalue;
        }
        uint8_t b23 = frame[33];
        sbus.failsafe  = (b23 & 0b00001000) != 0;
        sbus.framelost = (b23 & 0b00000100) != 0;
        sbus.ch18      = (b23 & 0b00000010) != 0;
        sbus.flag      = (b23 & 0b00000001) != 0;
        return sbus;
    }
};

} // namespace dp

// 使用例
int main() {
    dp::sbus sbus_reader("/dev/ttyUSB0");
    if (!sbus_reader.open_port()) return 1;

    int count = 0;
    while (true) {
        auto sbus_opt = sbus_reader.read_frame();
        if (sbus_opt) {
            dp::sbus::dump_frame(*sbus_opt);
            ++count;
        }
        usleep(10000);
    }
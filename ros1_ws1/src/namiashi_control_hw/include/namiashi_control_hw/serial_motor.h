#pragma once

#include <optional>
#include <string>
#include <vector>
#include <asio.hpp>

namespace dp {

class lkmotor {
public:
    lkmotor(const std::string& port, unsigned int baudrate = 1000000);

    void connect(uint8_t id);

    void servo(uint8_t id, bool on);
    void set_0deg(uint8_t id);
    std::optional<double> get_pos(uint8_t id);
    void send_mdeg(uint8_t id, double mdeg);
    void send_mdeg(uint8_t id, double mdeg, double dps);

    void set_current_pos(uint8_t id, double expected_pos);

private:
    void send_cmd_impl(uint8_t id, uint8_t cmd, const std::vector<uint8_t>& data);

    asio::io_context io_;
    asio::serial_port ser_;

    static uint8_t checksum(const std::vector<uint8_t>& datas);
    void swrite(const std::vector<uint8_t>& bdata);
};

} // namespace dp
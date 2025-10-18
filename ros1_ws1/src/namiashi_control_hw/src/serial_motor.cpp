#include "namiashi_control_hw/serial_motor.h"
#include <cstdio>
#include <iomanip>
#include <thread>
#include <chrono>
#include <algorithm>
#include <optional>
#include <sys/ioctl.h>
#include <termios.h>

namespace dp {

lkmotor::lkmotor(const std::string& port, unsigned int baudrate)
    : io_(), ser_(io_, port)
{
  ser_.set_option(asio::serial_port_base::baud_rate(baudrate));
  ser_.set_option(asio::serial_port_base::character_size(8));
  ser_.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
  ser_.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
}

void lkmotor::connect(uint8_t id) {
  std::vector<uint8_t> head = {0x3E, 0x10, id, 0x00};
  head.push_back(checksum(head));
  swrite(head);
}

void lkmotor::send_cmd_impl(uint8_t id, uint8_t cmd, const std::vector<uint8_t>& data) {
  std::vector<uint8_t> head = {0x3E, cmd, id, static_cast<uint8_t>(data.size())};
  head.push_back(checksum(head));
  std::vector<uint8_t> msg = head;
  if (data.size()) {
    msg.insert(msg.end(), data.begin(), data.end());
    msg.push_back(checksum(data));
  }
  swrite(msg);
}

void lkmotor::servo(uint8_t id, bool on) {
  auto flg = on ? 0x88 : 0x80;
  send_cmd_impl(id, flg, {});
}

void lkmotor::set_0deg(uint8_t id) {
  int retry_count = 0;
  const int max_retries = 5;
  bool success = false;
  while (retry_count < max_retries) {
    set_current_pos(id, 0.0);
    auto pos_opt = get_pos(id);
    if (pos_opt.has_value()) {
      double pos = pos_opt.value();
      if (std::abs(pos) <= 1.0) {
        success = true;
        break;
      } else {
        printf("[WARN] set_0deg: Position after set_current_pos is out of 1.0 deg range: %.3f (retry %d)\n", pos, retry_count + 1);
      }
    } else {
      printf("[WARN] set_0deg: Failed to get position after set_current_pos (retry %d)\n", retry_count + 1);
    }
    ++retry_count;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (!success) {
    printf("[ERROR] set_0deg: Could not set position within 1.0 deg after %d retries\n", max_retries);
  }
  // send_cmd_impl(id, 0x93, {});
}

std::optional<double> lkmotor::get_pos(uint8_t id) {
  // シリアルのリードバッファをクリア
  int fd = ser_.native_handle();
  tcflush(fd, TCIFLUSH);

  send_cmd_impl(id, 0x92, {});
  std::vector<uint8_t> rep_head = {0x3E, 0x92, id, 0x08};
  rep_head.push_back(checksum(rep_head));

  std::vector<uint8_t> rep(32);
  size_t n = ser_.read_some(asio::buffer(rep));
  rep.resize(n);

  auto it = std::search(rep.begin(), rep.end(), rep_head.begin(), rep_head.end());
  if (it == rep.end())
    return std::nullopt;
    
  size_t idx = std::distance(rep.begin(), it) + rep_head.size();
  if (idx + 8 > rep.size())
    return std::nullopt;

  std::vector<uint8_t> data(rep.begin() + idx, rep.begin() + idx + 8);
  int64_t val = 0;
  for (size_t i = 0; i < 8; ++i)
    val |= ((int64_t)data[i]) << (8 * i);

  return val / 1000;
}

void lkmotor::send_mdeg(uint8_t id, double mdeg) {
  auto deg = static_cast<int64_t>(mdeg * 1000.0f);
  send_cmd_impl(id, 0xA3, {
    static_cast<uint8_t>((deg >> 0) & 0xFF),
    static_cast<uint8_t>((deg >> 8) & 0xFF),
    static_cast<uint8_t>((deg >> 16) & 0xFF),
    static_cast<uint8_t>((deg >> 24) & 0xFF),
    static_cast<uint8_t>((deg >> 32) & 0xFF),
    static_cast<uint8_t>((deg >> 40) & 0xFF),
    static_cast<uint8_t>((deg >> 48) & 0xFF),
    static_cast<uint8_t>((deg >> 56) & 0xFF),
  });
}

void lkmotor::send_mdeg(uint8_t id, double mdeg, double dps) {
  auto deg = static_cast<int64_t>(mdeg * 1000.0f);
  auto spd = static_cast<int32_t>(dps * 1000.0f);
  send_cmd_impl(id, 0xA4, {
    static_cast<uint8_t>((deg >> 0) & 0xFF),
    static_cast<uint8_t>((deg >> 8) & 0xFF),
    static_cast<uint8_t>((deg >> 16) & 0xFF),
    static_cast<uint8_t>((deg >> 24) & 0xFF),
    static_cast<uint8_t>((deg >> 32) & 0xFF),
    static_cast<uint8_t>((deg >> 40) & 0xFF),
    static_cast<uint8_t>((deg >> 48) & 0xFF),
    static_cast<uint8_t>((deg >> 56) & 0xFF),
    static_cast<uint8_t>((spd >> 0) & 0xFF),
    static_cast<uint8_t>((spd >> 8) & 0xFF),
    static_cast<uint8_t>((spd >> 16) & 0xFF),
    static_cast<uint8_t>((spd >> 24) & 0xFF),
  });
}

uint8_t lkmotor::checksum(const std::vector<uint8_t>& datas) {
  uint8_t chksum = 0x00;
  for (auto d : datas) {
      chksum = (chksum + d) & 0xFF;
  }
  return chksum;
}

void lkmotor::set_current_pos(uint8_t id, double expected_pos) {
  auto mdeg = static_cast<int64_t>(expected_pos * 100.0f);
  send_cmd_impl(id, 0x95, {
    static_cast<uint8_t>((mdeg >> 0) & 0xFF),
    static_cast<uint8_t>((mdeg >> 8) & 0xFF),
    static_cast<uint8_t>((mdeg >> 16) & 0xFF),
    static_cast<uint8_t>((mdeg >> 24) & 0xFF),
  });
}


void lkmotor::swrite(const std::vector<uint8_t>& bdata) {
  #if 0
  printf("out:");
  for (auto byte : bdata) {
      printf(" %02x", byte);
  }
  printf("\n");
  #endif
  asio::write(ser_, asio::buffer(bdata));
}

} // namespace dp
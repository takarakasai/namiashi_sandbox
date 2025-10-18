#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h> // SBUS信号例

// for sbus
#include <iostream>
#include <vector>
#include <array>
#include <chrono>
#include <iomanip>
#include <asio.hpp>
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
    sbus(const std::string& port_name, unsigned int baudrate = 115200)
        : io_(), ser_(io_)
    {
        ser_.open(port_name);
        ser_.set_option(asio::serial_port_base::baud_rate(baudrate));
        ser_.set_option(asio::serial_port_base::character_size(8));
        ser_.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
        ser_.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
        ser_.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));
    }

    std::optional<SBus2> read_frame() {
        std::vector<uint8_t> buffer(1024);
        size_t bytes_read = ser_.read_some(asio::buffer(buffer));
        if (bytes_read == 0) return std::nullopt;

        for (size_t i = 0; i < bytes_read; ++i) {
            uint8_t byte = buffer[i];
            if (byte == 0x0F) {
                if (!frame_buffer_.empty()) {
                    auto sbus_opt = parse_frame(frame_buffer_);
                    frame_buffer_.clear();
                    if (sbus_opt) {
                        return sbus_opt;
                    }
                }
            }
            frame_buffer_.push_back(byte);
        }
        return std::nullopt;
    }

    static void dump_frame(const SBus2& sbus) {
        for (size_t i = 0; i < sbus.ch.size(); ++i)
            std::cout << "ch" << std::setw(2) << std::setfill('0') << i << ": " << std::showpos << std::fixed << std::setprecision(4) << sbus.ch[i] << std::noshowpos << std::endl;
        std::cout << "FS:" << sbus.failsafe << " FL:" << sbus.framelost << " CH18:" << sbus.ch18 << " FLG:" << sbus.flag << std::endl;
    }

private:
    asio::io_context io_;
    asio::serial_port ser_;

    std::vector<uint8_t> frame_buffer_;

    static uint8_t calculate_checksum(const uint8_t* data, size_t len) {
        uint8_t sum = 0;
        for (size_t i = 0; i < len; ++i) sum ^= data[i];
        return sum;
    }

    static std::optional<SBus2> parse_frame(const std::vector<uint8_t>& frame) {
        if (frame.size() < 35) {
            #if defined(DP_SBUS_DEBUG)
            std::cerr << "frame is too short len:" << frame.size() << std::endl;
            #endif
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

class TeleopSbus
{
public:
    TeleopSbus(ros::NodeHandle& nh)
    {
        pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        sub_ = nh.subscribe("sbus", 1, &TeleopSbus::sbusCallback, this);
    }

    void sbusCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        // 例: msg->data[0] = 前後, msg->data[1] = 左右
        geometry_msgs::Twist twist;
        if (msg->data.size() >= 2) {
            twist.linear.x  = msg->data[0]; // 前後
            twist.angular.z = msg->data[1]; // 左右
        }
        pub_.publish(twist);
    }

private:
    ros::Publisher pub_;
    ros::Subscriber sub_;
};

int main(int argc, char** argv)
{
  #if 0
    ros::init(argc, argv, "teleop_sbus_node");
    ros::NodeHandle nh;
    TeleopSbus teleop(nh);
    ros::spin();
    return 0;
  #else
    ros::init(argc, argv, "teleop_sbus_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    dp::sbus sbus_reader("/dev/ttyAMA2", 115200);

    ros::Rate rate(50); // 50Hz
    size_t count = 0;
    bool is_signal_losting = false;
    while (ros::ok()) {
        auto sbus_opt = sbus_reader.read_frame();
        if (sbus_opt) {
            if (count++ % 20 == 0) {
                sbus_reader.dump_frame(*sbus_opt);
            }
            geometry_msgs::Twist twist;
            if (sbus_opt->failsafe || sbus_opt->framelost) {
                if (!is_signal_losting) {
                    ROS_WARN("Signal Lost");
                    is_signal_losting = true;
                }
                twist.linear.x = 0.0;
                twist.linear.y = 0.0;
                twist.linear.z = 0.0;
                twist.angular.z = 0.0;
            } else {
                if (is_signal_losting) {
                    ROS_WARN("Signal Active");
                    is_signal_losting = false;
                }
                // ch[1] = 前後, ch[3] = 左右 (right-hand coords)
                // clip関数で値を制限
                auto clip = [](float val, float min_thresh, float max_thresh) {
                    if (min_thresh < val && val < max_thresh) return 0.0f;
                    return val;
                };
                twist.linear.x  = clip(sbus_opt->ch[1], -0.2f, 0.2f); // F(+) B(-)
                twist.linear.y  = clip(sbus_opt->ch[0], -0.2f, 0.2f); // L(+) R(-)
                twist.linear.z  = clip(sbus_opt->ch[5], -0.5f, 0.5f); // U(+) D(-)
                twist.angular.z = clip(sbus_opt->ch[3], -0.2f, 0.2f); // R(+) L(-)
            }
            pub.publish(twist);
        } else {
        }
        ros::spinOnce();
        rate.sleep();
    }
    // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return 0;
  #endif
}

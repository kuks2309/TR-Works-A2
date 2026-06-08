// TOF USB-serial driver: reads the ESP32-C3 (VL53L0X) over /dev/ttyACM0 and
// republishes the range as sensor_msgs/Range on /tof/range.
//
// The board (ToF2CAN, ESP32-C3) exposes a native USB CDC serial port and streams
// one CSV line per measurement at 115200 baud:  "<mm>,<status>"  (~10 Hz),
// e.g. "495,2". This driver parses the leading integer (millimetres).
//
// GOTCHA: the ESP32-C3 native USB-CDC only starts streaming once DTR is asserted
// by the host. `cat`/`stty` do not raise DTR (so the port looks silent); this
// driver raises DTR explicitly after opening. The driver is transport-only; the
// detection pipeline consumes /tof/range and does not care how it is produced.
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <regex>
#include <string>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

using namespace std::chrono_literals;

namespace openarmx_tof_driver
{

class TofSerialDriver : public rclcpp::Node
{
public:
  TofSerialDriver() : rclcpp::Node("tof_serial_driver")
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyACM0");
    baud_ = declare_parameter<int>("baud", 115200);
    frame_id_ = declare_parameter<std::string>("frame_id", "tof_link");
    field_of_view_ = declare_parameter<double>("field_of_view", 0.44);  // VL53L0X ~25deg
    min_range_ = declare_parameter<double>("min_range", 0.03);
    max_range_ = declare_parameter<double>("max_range", 2.0);
    // Lines matching this regex yield the range; group 1 is millimeters.
    // Real firmware streams "<mm>,<status>" (e.g. "495,2").
    pattern_ = declare_parameter<std::string>("pattern", R"((\d+)\s*,\s*\d+)");

    pub_ = create_publisher<sensor_msgs::msg::Range>("/tof/range", 10);
    re_ = std::regex(pattern_);

    if (!openPort()) {
      RCLCPP_ERROR(get_logger(), "could not open %s; will retry", port_.c_str());
    }
    timer_ = create_wall_timer(20ms, std::bind(&TofSerialDriver::poll, this));
    RCLCPP_INFO(get_logger(), "tof_serial_driver: port=%s baud=%d -> /tof/range",
                port_.c_str(), baud_);
  }

  ~TofSerialDriver() override { if (fd_ >= 0) ::close(fd_); }

private:
  bool openPort()
  {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    fd_ = ::open(port_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;

    termios tio{};
    if (tcgetattr(fd_, &tio) != 0) { ::close(fd_); fd_ = -1; return false; }
    cfmakeraw(&tio);
    speed_t spd = B115200;
    switch (baud_) {
      case 9600:   spd = B9600;   break;
      case 57600:  spd = B57600;  break;
      case 115200: spd = B115200; break;
      case 230400: spd = B230400; break;
      default:     spd = B115200; break;
    }
    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    tcsetattr(fd_, TCSANOW, &tio);
    tcflush(fd_, TCIFLUSH);

    // ESP32-C3 native USB-CDC only streams once DTR is asserted by the host.
    // (cat/stty leave DTR low, so the port looks silent.) Raise DTR + RTS.
    int mctl = 0;
    if (ioctl(fd_, TIOCMGET, &mctl) == 0) {
      mctl |= (TIOCM_DTR | TIOCM_RTS);
      ioctl(fd_, TIOCMSET, &mctl);
    }
    return true;
  }

  void poll()
  {
    if (fd_ < 0) { openPort(); return; }
    char buf[256];
    ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n <= 0) {
      if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "serial read error: %s; reopening", strerror(errno));
        ::close(fd_); fd_ = -1;
      }
      return;
    }
    line_.append(buf, static_cast<size_t>(n));
    size_t pos;
    while ((pos = line_.find('\n')) != std::string::npos) {
      handleLine(line_.substr(0, pos));
      line_.erase(0, pos + 1);
    }
    if (line_.size() > 4096) line_.clear();  // runaway guard
  }

  void handleLine(const std::string & ln)
  {
    std::smatch m;
    if (!std::regex_search(ln, m, re_) || m.size() < 2) return;
    double mm = std::stod(m[1].str());
    double meters = mm / 1000.0;

    sensor_msgs::msg::Range msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
    msg.field_of_view = static_cast<float>(field_of_view_);
    msg.min_range = static_cast<float>(min_range_);
    msg.max_range = static_cast<float>(max_range_);
    msg.range = static_cast<float>(meters);
    pub_->publish(msg);
  }

  std::string port_, frame_id_, pattern_, line_;
  int baud_ = 115200, fd_ = -1;
  double field_of_view_, min_range_, max_range_;
  std::regex re_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace openarmx_tof_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<openarmx_tof_driver::TofSerialDriver>());
  rclcpp::shutdown();
  return 0;
}

#include "demo_aim/serial_port.hpp"
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>
namespace aim {
namespace {
std::runtime_error failure(const char *what) {
  return std::runtime_error(std::string(what) + ": " + std::strerror(errno));
}
void checkTimeout(int timeout) {
  if (timeout < 0 || timeout > 1000)
    throw std::invalid_argument("UART timeout must be 0..1000 ms");
}
} // namespace
SerialPort::SerialPort(const std::string &device, int baud) {
  speed_t speed;
  switch (baud) {
  case 115200:
    speed = B115200;
    break;
  case 460800:
    speed = B460800;
    break;
  case 921600:
    speed = B921600;
    break;
  default:
    throw std::invalid_argument("Supported baud: 115200,460800,921600");
  }
  if (device.empty() || device.find('?') != std::string::npos)
    throw std::invalid_argument("Set actual UART device path");
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0)
    throw failure("Open UART");
  try {
    termios t{};
    if (tcgetattr(fd_, &t) < 0)
      throw failure("Read UART settings");
    cfmakeraw(&t);
    t.c_cflag &= ~(CSTOPB | PARENB | CSIZE | CRTSCTS);
    t.c_cflag |= CS8 | CLOCAL | CREAD;
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 0;
    if (cfsetispeed(&t, speed) < 0 || cfsetospeed(&t, speed) < 0 ||
        tcsetattr(fd_, TCSANOW, &t) < 0)
      throw failure("Configure UART");
  } catch (...) {
    ::close(fd_);
    fd_ = -1;
    throw;
  }
}
SerialPort::~SerialPort() {
  if (fd_ >= 0)
    ::close(fd_);
}
std::vector<std::uint8_t> SerialPort::read(int timeout) {
  checkTimeout(timeout);
  pollfd p{fd_, POLLIN, 0};
  const int ready = poll(&p, 1, timeout);
  if (ready < 0) {
    if (errno == EINTR)
      return {};
    throw failure("Poll UART");
  }
  if (!ready)
    return {};
  if (p.revents & (POLLERR | POLLHUP | POLLNVAL))
    throw std::runtime_error("UART disconnected");
  std::vector<std::uint8_t> data(4096);
  const ssize_t n = ::read(fd_, data.data(), data.size());
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
      return {};
    throw failure("Read UART");
  }
  data.resize(static_cast<std::size_t>(n));
  return data;
}
void SerialPort::write(const std::vector<std::uint8_t> &packet, int timeout) {
  checkTimeout(timeout);
  if (packet.size() > 4096)
    throw std::invalid_argument("UART packet too large");
  using Clock = std::chrono::steady_clock;
  const auto deadline = Clock::now() + std::chrono::milliseconds(timeout);
  std::size_t sent = 0;
  while (sent < packet.size()) {
    const auto remaining =
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline -
                                                              Clock::now())
            .count();
    if (remaining < 0)
      throw std::runtime_error("UART write timeout");
    pollfd p{fd_, POLLOUT, 0};
    const int ready = poll(&p, 1, static_cast<int>(remaining));
    if (ready < 0) {
      if (errno == EINTR)
        continue;
      throw failure("Poll UART write");
    }
    if (!ready)
      throw std::runtime_error("UART write timeout");
    if (p.revents & (POLLERR | POLLHUP | POLLNVAL))
      throw std::runtime_error("UART disconnected");
    const ssize_t n = ::write(fd_, packet.data() + sent, packet.size() - sent);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
        continue;
      throw failure("Write UART");
    }
    sent += static_cast<std::size_t>(n);
  }
}
} // namespace aim

// 简单串口封装(POSIX termios),非阻塞读,阻塞写。
#ifndef CAR_BASE_SERIAL_PORT_HPP_
#define CAR_BASE_SERIAL_PORT_HPP_

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <string>

namespace car_base
{

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort() {close_port();}

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  // 打开串口。成功返回 true;失败时 err 填入原因。
  bool open_port(const std::string & device, int baud, std::string & err)
  {
    close_port();
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      err = std::string("open() 失败: ") + std::strerror(errno);
      return false;
    }

    struct termios tio;
    std::memset(&tio, 0, sizeof(tio));
    if (tcgetattr(fd_, &tio) != 0) {
      err = std::string("tcgetattr 失败: ") + std::strerror(errno);
      close_port();
      return false;
    }

    speed_t spd = to_speed(baud);
    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);

    // 8N1,原始模式,无流控
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag &= ~PARENB;      // 无校验
    tio.c_cflag &= ~CSTOPB;      // 1 停止位
    tio.c_cflag &= ~CRTSCTS;     // 无硬件流控
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);  // 无软件流控
    tio.c_iflag &= ~(ICRNL | INLCR | IGNCR);
    tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // 原始
    tio.c_oflag &= ~OPOST;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;         // 非阻塞(配合 O_NONBLOCK)

    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
      err = std::string("tcsetattr 失败: ") + std::strerror(errno);
      close_port();
      return false;
    }
    tcflush(fd_, TCIOFLUSH);
    return true;
  }

  bool is_open() const {return fd_ >= 0;}

  void close_port()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  // 非阻塞读,返回读到的字节数(0 表示暂无数据,-1 表示错误)
  ssize_t read_some(uint8_t * buf, size_t max_len)
  {
    if (fd_ < 0) {return -1;}
    ssize_t n = ::read(fd_, buf, max_len);
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {return 0;}
    return n;
  }

  // 写全部字节,返回是否成功写完
  bool write_all(const uint8_t * buf, size_t len)
  {
    if (fd_ < 0) {return false;}
    size_t written = 0;
    while (written < len) {
      ssize_t n = ::write(fd_, buf + written, len - written);
      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {continue;}
        return false;
      }
      written += static_cast<size_t>(n);
    }
    return true;
  }

private:
  static speed_t to_speed(int baud)
  {
    switch (baud) {
      case 9600: return B9600;
      case 19200: return B19200;
      case 38400: return B38400;
      case 57600: return B57600;
      case 115200: return B115200;
      case 230400: return B230400;
      case 460800: return B460800;
      case 921600: return B921600;
      default: return B115200;
    }
  }

  int fd_ = -1;
};

}  // namespace car_base

#endif  // CAR_BASE_SERIAL_PORT_HPP_

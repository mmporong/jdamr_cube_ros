#include "jdamr_base_driver/serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace jdamr
{

static speed_t to_speed(int baud)
{
  switch (baud) {
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return 0;
  }
}

bool SerialPort::open(const std::string & device, int baud, std::string & error)
{
  close();
  const speed_t sp = to_speed(baud);
  if (sp == 0) {
    error = "지원하지 않는 보레이트: " + std::to_string(baud);
    return false;
  }
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0) {
    error = device + " 열기 실패: " + std::strerror(errno);
    return false;
  }

  termios tio{};
  if (tcgetattr(fd_, &tio) != 0) {
    error = std::string("tcgetattr: ") + std::strerror(errno);
    close();
    return false;
  }
  cfmakeraw(&tio);           // 에코·개행 변환·시그널 전부 끔
  cfsetispeed(&tio, sp);
  cfsetospeed(&tio, sp);
  tio.c_cflag |= CLOCAL | CREAD;
  tio.c_cc[VMIN] = 0;        // VMIN=0, VTIME=1 → 최대 100ms 블록 후 복귀
  tio.c_cc[VTIME] = 1;       //   (읽기 스레드가 종료 플래그를 볼 수 있는 주기)
  if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
    error = std::string("tcsetattr: ") + std::strerror(errno);
    close();
    return false;
  }
  tcflush(fd_, TCIOFLUSH);   // 이전 세션 잔여 바이트 폐기 (재동기 부담 감소)
  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

ssize_t SerialPort::read_some(uint8_t * buf, size_t n)
{
  return ::read(fd_, buf, n);
}

bool SerialPort::write_all(const uint8_t * buf, size_t n)
{
  size_t done = 0;
  while (done < n) {
    const ssize_t w = ::write(fd_, buf + done, n - done);
    if (w < 0) {
      if (errno == EINTR) {continue;}
      return false;
    }
    done += static_cast<size_t>(w);
  }
  return true;
}

}  // namespace jdamr

// POSIX 시리얼 래퍼 — termios raw 모드, 읽기 스레드에서 블로킹 read 전용.
// VTIME=1(100ms) 로 read 가 주기적으로 깨어나 종료 플래그를 볼 수 있게 한다.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace jdamr
{

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort() {close();}
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  // 성공 시 true. baud 는 115200/230400/460800/921600 지원.
  bool open(const std::string & device, int baud, std::string & error);
  void close();
  bool is_open() const {return fd_ >= 0;}

  // 최대 n바이트 읽기. 반환: 읽은 바이트 수(0 = 타임아웃), 음수 = 오류.
  ssize_t read_some(uint8_t * buf, size_t n);
  // 전량 쓰기. 실패 시 false.
  bool write_all(const uint8_t * buf, size_t n);

private:
  int fd_{-1};
};

}  // namespace jdamr

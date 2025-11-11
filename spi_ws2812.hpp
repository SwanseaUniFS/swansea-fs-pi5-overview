#pragma once
#include <cstdint>
#include <vector>
#include <string>

class SpiWs2812 {
public:
  // Use "/dev/spidev0.1" on your Pi 5 (you have that device)
  bool open(const char* dev="/dev/spidev0.1", uint32_t speed_hz=3200000);
  void close();
  // Input buffer is GRB per LED (3 * led_count bytes)
  bool show(const uint8_t* grb, int led_count);
  std::string last_error() const { return err_; }
private:
  int fd_ = -1;
  uint32_t speed_ = 3200000;
  std::vector<uint8_t> tx_;
  std::string err_;
};

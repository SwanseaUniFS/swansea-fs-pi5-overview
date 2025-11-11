#pragma once
#include <cstdint>
#include <vector>
#include <string>

// WS2812 LED driver using Raspberry Pi SPI (spidev)
// Compatible with /dev/spidev1.0 → GPIO20 (pin 38) on SPI1 bus
// Requires: dtoverlay=spi1-3cs in /boot/config.txt
// Wiring: DIN→GPIO20, GND common, 5V power, 330 Ω resistor on DIN recommended.

class SpiWs2812 {
public:
  // Opens SPI device (default /dev/spidev1.0 at 3.2 MHz)
  bool open(const char* dev = "/dev/spidev1.0", uint32_t speed_hz = 3200000);

  // Close SPI device
  void close();

  // Send LED data (GRB order, 3 bytes per LED)
  bool show(const uint8_t* grb, int led_count);

  // Retrieve last error message
  std::string last_error() const { return err_; }

private:
  int fd_ = -1;
  uint32_t speed_ = 3200000;
  std::vector<uint8_t> tx_;
  std::string err_;
};

#include "spi_ws2812.hpp"
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

// 4-bit -> 16-bit timing LUT (1 -> 1110, 0 -> 1000)
static uint16_t NIBBLE_LUT[16];

static void init_lut() {
  static bool inited = false;
  if (inited) return;
  inited = true;
  for (int n = 0; n < 16; ++n) {
    uint16_t out = 0;
    for (int i = 3; i >= 0; --i) {
      out <<= 4;
      out |= ((n >> i) & 1) ? 0b1110 : 0b1000;
    }
    NIBBLE_LUT[n] = out;
  }
}

bool SpiWs2812::open(const char* dev, uint32_t speed_hz) {
  init_lut();

  speed_ = speed_hz;
  fd_ = ::open(dev, O_RDWR);             // O_RDWR works best on many kernels
  if (fd_ < 0) { err_ = "open spidev failed"; return false; }

  // Prefer the 8-bit mode ioctl (more widely supported)
  uint8_t mode8 = SPI_MODE_0;            // no SPI_NO_CS needed (CS not wired)
  uint8_t bpw   = 8;

  if (ioctl(fd_, SPI_IOC_WR_MODE, &mode8) < 0) {
    err_ = "set mode failed"; return false;
  }
  if (ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bpw) < 0) {
    err_ = "set bpw failed"; return false;
  }
  if (ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_) < 0) {
    err_ = "set speed failed"; return false;
  }
  return true;
}

void SpiWs2812::close() {
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

bool SpiWs2812::show(const uint8_t* grb, int led_count) {
  if (fd_ < 0) { err_ = "spidev not open"; return false; }
  if (led_count <= 0) return true;

  // Each color byte -> 32 encoded bits -> 4 bytes; 3 colors/LED -> 12 bytes/LED
  const int BYTES_PER_COLOR = 4;
  const int BYTES_PER_LED   = 3 * BYTES_PER_COLOR;

  tx_.assign(led_count * BYTES_PER_LED, 0);

  uint8_t* p = tx_.data();
  auto enc_byte = [&](uint8_t v) {
    uint16_t hi = NIBBLE_LUT[(v >> 4) & 0x0F];
    uint16_t lo = NIBBLE_LUT[v & 0x0F];
    *p++ = (uint8_t)(hi >> 8);
    *p++ = (uint8_t)(hi & 0xFF);
    *p++ = (uint8_t)(lo >> 8);
    *p++ = (uint8_t)(lo & 0xFF);
  };

  for (int i = 0; i < led_count; ++i) {
    // order: GRB
    enc_byte(grb[i*3 + 0]);  // G
    enc_byte(grb[i*3 + 1]);  // R
    enc_byte(grb[i*3 + 2]);  // B
  }

  ssize_t need = tx_.size();
  ssize_t wr = ::write(fd_, tx_.data(), need);
  if (wr != need) { err_ = "spi write short"; return false; }

  // Latch/reset: keep line low for >80 µs
  usleep(300);
  return true;
}

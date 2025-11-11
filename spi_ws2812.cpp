#include "spi_ws2812.hpp"
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

// -----------------------------------------------------------------------------
// LUT for WS2812 SPI encoding
// Each input bit is expanded into 4 SPI bits (1 → 1110, 0 → 1000)
// At 3.2 MHz this yields ~1.25 µs per WS2812 bit → compliant timing
// -----------------------------------------------------------------------------
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

// -----------------------------------------------------------------------------
// Open SPI device
// -----------------------------------------------------------------------------
bool SpiWs2812::open(const char* dev, uint32_t speed_hz) {
  init_lut();
  speed_ = speed_hz;

  fd_ = ::open(dev, O_RDWR);
  if (fd_ < 0) { err_ = "open spidev failed"; return false; }

  uint8_t mode8 = SPI_MODE_0 | SPI_NO_CS;  // keep CS inactive, MOSI only
  uint8_t bpw   = 8;

  if (ioctl(fd_, SPI_IOC_WR_MODE, &mode8) < 0) {
    err_ = "set SPI mode failed";
    ::close(fd_); fd_ = -1;
    return false;
  }
  if (ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bpw) < 0) {
    err_ = "set bits_per_word failed";
    ::close(fd_); fd_ = -1;
    return false;
  }
  if (ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_) < 0) {
    err_ = "set speed failed";
    ::close(fd_); fd_ = -1;
    return false;
  }

  return true;
}

// -----------------------------------------------------------------------------
// Close SPI device
// -----------------------------------------------------------------------------
void SpiWs2812::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

// -----------------------------------------------------------------------------
// Encode & send LED data
// -----------------------------------------------------------------------------
bool SpiWs2812::show(const uint8_t* grb, int led_count) {
  if (fd_ < 0) { err_ = "spidev not open"; return false; }
  if (led_count <= 0) return true;

  constexpr int BYTES_PER_COLOR = 4;  // 4 SPI bytes per color byte
  constexpr int BYTES_PER_LED   = 3 * BYTES_PER_COLOR;

  tx_.resize(led_count * BYTES_PER_LED);
  uint8_t* p = tx_.data();

  auto enc_byte = [&](uint8_t v) {
    uint16_t hi = NIBBLE_LUT[(v >> 4) & 0x0F];
    uint16_t lo = NIBBLE_LUT[v & 0x0F];
    *p++ = uint8_t(hi >> 8);
    *p++ = uint8_t(hi & 0xFF);
    *p++ = uint8_t(lo >> 8);
    *p++ = uint8_t(lo & 0xFF);
  };

  for (int i = 0; i < led_count; ++i) {
    // order: GRB
    enc_byte(grb[i * 3 + 0]);
    enc_byte(grb[i * 3 + 1]);
    enc_byte(grb[i * 3 + 2]);
  }

  ssize_t need = tx_.size();
  ssize_t wr = ::write(fd_, tx_.data(), need);
  if (wr != need) {
    err_ = "spi write short";
    return false;
  }

  // WS2812 latch/reset requires > 80 µs low — 300 µs is safe
  usleep(300);
  return true;
}

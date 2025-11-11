#pragma once
#include <cstdint>
#include <optional>

struct CanFrame {
  uint32_t id;
  uint8_t  dlc;
  uint8_t  data[8];
};

class SocketCan {
public:
  explicit SocketCan(const char* ifname = "can0");
  ~SocketCan();
  bool open();
  std::optional<CanFrame> read_nonblock();
private:
  int sock_ = -1;
  const char* ifname_;
};

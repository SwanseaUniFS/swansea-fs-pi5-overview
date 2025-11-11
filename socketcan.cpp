#include "socketcan.hpp"
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <cstdio>


SocketCan::SocketCan(const char* ifname) : ifname_(ifname) {}
SocketCan::~SocketCan(){ if (sock_ >= 0) close(sock_); }

bool SocketCan::open(){
  int s = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (s < 0) return false;
  ifreq ifr{}; std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", ifname_);
  if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) { close(s); return false; }
  sockaddr_can addr{}; addr.can_family = AF_CAN; addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(s, (sockaddr*)&addr, sizeof(addr)) < 0) { close(s); return false; }
  sock_ = s; return true;
}

std::optional<CanFrame> SocketCan::read_nonblock(){
  can_frame f{};
  int n = ::recv(sock_, &f, sizeof(f), MSG_DONTWAIT);
  if (n <= 0) return std::nullopt;
  CanFrame out{}; out.id = f.can_id & CAN_EFF_MASK; out.dlc = f.can_dlc;
  std::memcpy(out.data, f.data, 8);
  return out;
}

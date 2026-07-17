//test_led.cpp

#include <cstdio>
#include <cstring>
#include <cstdint>
#include <unistd.h>
#include <ws2811.h>

static constexpr int   LED_PIN = 18;
static constexpr int   LED_COUNT = 19;
static constexpr int   LED_BRIGHTNESS = 128;

static ws2811_t g_leds;

static inline uint32_t grb(uint8_t r, uint8_t g, uint8_t b){
  return (uint32_t(g) << 16) | (uint32_t(r) << 8) | uint32_t(b);
}

static inline void leds_set_rgb(int i, uint8_t r, uint8_t g, uint8_t b){
  if (i < 0 || i >= LED_COUNT) return;
  g_leds.channel[0].leds[i] = grb(r, g, b);
}

static void leds_off(){
  for (int i = 0; i < LED_COUNT; i++) leds_set_rgb(i, 0, 0, 0);
  ws2811_render(&g_leds);
}

static void turn_them(int r, int g, int b) {
  std::printf("turn them\n");
  for (int i = 0; i < LED_COUNT; i++) {
    leds_set_rgb(i, r, g, b);
  }
  ws2811_render(&g_leds);
}

int main(int argc, char *argv[]){
  std::memset(&g_leds, 0, sizeof(ws2811_t));
  g_leds.freq                  = WS2811_TARGET_FREQ;
  g_leds.dmanum                = 10;
  g_leds.channel[0].gpionum    = LED_PIN;
  g_leds.channel[0].count      = LED_COUNT;
  g_leds.channel[0].invert     = 0;
  g_leds.channel[0].brightness = LED_BRIGHTNESS;
  g_leds.channel[0].strip_type = WS2811_STRIP_GRB;

  ws2811_return_t ret = ws2811_init(&g_leds);
  if (ret != WS2811_SUCCESS){
    std::fprintf(stderr, "ws2811_init failed: %s\n", ws2811_get_return_t_str(ret));
    return 1;
  }

  bool quit = false;
  while (!quit){
    turn_them(255, 255, 255);
    sleep(1);
    turn_them(0, 0, 0);
    sleep(1);
  }

  leds_off();
  ws2811_fini(&g_leds);
  return 0;
}
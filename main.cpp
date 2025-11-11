// Raspberry Pi dash for DTAFast T8+
// LVGL + SDL2 + SocketCAN + WS2812 over SPI (spi_ws2812.hpp)
// CAN map (little-endian preferred; auto fallback to big-endian if LE=0):
//   0x2000: RPM      @ bytes 0..1 (U16 / 1)
//   0x2001: OilPres  @ bytes 6..7 (U16 / 10.0 kPa)
//   0x2002: OilTemp  @ bytes 2..3 (U16 / 10.0 °C)
//          : Voltage @ bytes 4..5 (U16 / 10.0 V)

#include <cstdio>
#include <cstdint>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <vector>
#include <SDL2/SDL.h>
#include <iostream>
#include <cstring>

extern "C" {
  #include "lvgl.h"
  #include "ui.h"
}

#include "socketcan.hpp"
#include "config.h"        // RPM_MIN, RPM_MAX, VOLTAGE_MIN, PRESSURE_MIN, TEMP_MAX
#include "spi_ws2812.hpp"  // SPI→WS2812 helper

#define DEBUG_CAN 1

// ---------- screen ----------
static constexpr int SCR_W = 800;
static constexpr int SCR_H = 480;

// ---------- CAN throttle ----------
static constexpr int MAX_CAN_PER_FRAME = 300;

// ---------- LEDs: only red flash on over-rev, otherwise OFF ----------
static constexpr int   LED_COUNT = 19;
static constexpr float LED_BRIGHTNESS_SCALE = 0.20f;
static constexpr uint8_t RED_R = uint8_t(255 * LED_BRIGHTNESS_SCALE);
static constexpr uint8_t RED_G = 0;
static constexpr uint8_t RED_B = 0;
static constexpr uint32_t FLICKER_INTERVAL_MS = 20;
static constexpr uint32_t RPM_TIMEOUT_MS      = 400;  // if no RPM frames, force LEDs off

static SpiWs2812 g_spi;
static std::vector<uint8_t> g_led_grb(LED_COUNT * 3, 0);
static bool   g_leds_shown = false;
static bool   g_flash_on   = true;
static uint32_t g_last_flash_ms    = 0;
static uint32_t g_last_rpm_framems = 0;

// ---------- SDL / LVGL ----------
static SDL_Window*   g_win = nullptr;
static SDL_Renderer* g_ren = nullptr;
static SDL_Texture*  g_tex = nullptr;

static lv_disp_draw_buf_t g_draw_buf;
static lv_color_t g_buf1[SCR_W * 160];
static lv_color_t g_buf2[SCR_W * 160];
static bool g_need_present = false;

// ---------- label helpers ----------
static char buf32[32];
static inline void txt_u16(uint16_t v, lv_obj_t* o) {
  std::snprintf(buf32, sizeof(buf32), "%u", (unsigned)v);
  lv_label_set_text(o, buf32);
}
static inline void txt_1dp(double v, lv_obj_t* o) {
  std::snprintf(buf32, sizeof(buf32), "%.1f", v);
  lv_label_set_text(o, buf32);
}
static inline void set_visible(lv_obj_t* o, bool vis) {
  if (!o) return;
  if (vis) lv_obj_clear_flag(o, LV_OBJ_FLAG_HIDDEN);
  else     lv_obj_add_flag(o,   LV_OBJ_FLAG_HIDDEN);
}

// ---------- LED helpers (GRB write order) ----------
static inline void leds_clear_all() {
  std::fill(g_led_grb.begin(), g_led_grb.end(), 0);
}
static inline void leds_show() {
  (void)g_spi.show(g_led_grb.data(), LED_COUNT);
  g_leds_shown = true;
}
static inline void leds_off() {
  if (!g_leds_shown) return;
  leds_clear_all();
  leds_show();
  g_leds_shown = false;
}

// Only flash red when rpm > RPM_MAX. Otherwise OFF. Never any other colour.
static void updateRPMLEDs_overrev_only(uint16_t rpm, uint32_t now_ms) {
  if (rpm == 0) { leds_off(); return; }

  const bool over = rpm > RPM_MAX;
  if (!over) { leds_off(); return; }

  if (now_ms - g_last_flash_ms >= FLICKER_INTERVAL_MS) {
    g_last_flash_ms = now_ms;
    g_flash_on = !g_flash_on;
  }

  if (!g_flash_on) { leds_off(); return; }

  // Fill all lit with pure RED (single channel) → no way to get white
  for (int i = 0; i < LED_COUNT; ++i) {
    g_led_grb[i*3 + 0] = 0;       // G
    g_led_grb[i*3 + 1] = RED_R;   // R
    g_led_grb[i*3 + 2] = 0;       // B
  }
  leds_show();
}

// ===================== CAN parsing =====================
static uint16_t last_rpm_raw   = 0xFFFF;
static uint16_t last_oilp_raw  = 0xFFFF;
static uint16_t last_oilt_raw  = 0xFFFF;
static uint16_t last_volt_raw  = 0xFFFF;

static uint32_t last_oilp_set_ms = 0;
static uint32_t last_oilt_set_ms = 0;
static uint32_t last_volt_set_ms = 0;

static inline uint16_t u16_le(const uint8_t *d){ return uint16_t(d[0] | (uint16_t(d[1])<<8)); }
static inline uint16_t u16_be(const uint8_t *d){ return uint16_t((uint16_t(d[0])<<8) | d[1]); }
static inline uint16_t u16_auto(const uint8_t *d, const char *name, bool *used_be) {
  uint16_t le = u16_le(d);
  if (le != 0) { if (used_be) *used_be = false; return le; }
  uint16_t be = u16_be(d);
#if DEBUG_CAN
  std::printf("[CAN] %s: LE=0 using BE=%u\n", name, (unsigned)be);
#endif
  if (used_be) *used_be = true;
  return be;
}

// 0x2000 — RPM
static void handle_2000(const CanFrame &fr) {
  bool used_be = false;
  uint16_t rpm = u16_auto(&fr.data[0], "rpm", &used_be);
  g_last_rpm_framems = SDL_GetTicks();

#if DEBUG_CAN
  std::printf("[CAN] 2000 rpm=%u%s\n", (unsigned)rpm, used_be?" (BE)":"");
#endif

  if (last_rpm_raw == 0xFFFF || std::abs(int(rpm) - int(last_rpm_raw)) >= 5) {
    last_rpm_raw = rpm;
    extern lv_obj_t *ui_erpm, *ui_erpmbar;
    txt_u16(rpm, ui_erpm);
    lv_bar_set_value(ui_erpmbar, rpm, LV_ANIM_OFF);
  }

  // Show high/low boxes (your latest behaviour)
  extern lv_obj_t *ui_erpmbackswitchup, *ui_erpmbackswitchdown;
  set_visible(ui_erpmbackswitchup,   rpm >= RPM_MAX);
  set_visible(ui_erpmbackswitchdown, rpm > 0);

  updateRPMLEDs_overrev_only(rpm, SDL_GetTicks());
}

// 0x2001 — Oil Pressure @ bytes 6..7 (kPa = /10.0) — always visible, white text, 1dp
static void handle_2001(const CanFrame &fr) {
  bool used_be = false;
  uint16_t raw = u16_auto(&fr.data[6], "oilP", &used_be);
  double kpa = raw / 10.0;

#if DEBUG_CAN
  std::printf("[CAN] 2001 oilP_raw=%u kPa=%.1f%s\n",(unsigned)raw,kpa,used_be?" (BE)":"");
#endif

  uint32_t now = SDL_GetTicks();
  if (raw != last_oilp_raw || now - last_oilp_set_ms > 250) {
    last_oilp_raw   = raw;
    last_oilp_set_ms = now;
    extern lv_obj_t *ui_eoilpressure, *ui_oilpressuredu, *ui_eoilpressureback;
    txt_1dp(kpa, ui_eoilpressure);
    lv_obj_set_style_text_color(ui_eoilpressure,  lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_color(ui_oilpressuredu, lv_color_hex(0xFFFFFF), 0);
    set_visible(ui_eoilpressureback, true);
  }
}

// 0x2002 — Oil Temp @ bytes 2..3 (°C = /10.0), Voltage @ bytes 4..5 (V = /10.0)
// Both: always visible, white text. Temp/Volt refreshed at least every 250 ms.
static void handle_2002(const CanFrame &fr) {
  uint32_t now = SDL_GetTicks();

  // Temp
  {
    bool used_be = false;
    uint16_t raw_t = u16_auto(&fr.data[2], "oilT", &used_be);
    double c = raw_t / 10.0;
#if DEBUG_CAN
    std::printf("[CAN] 2002 oilT_raw=%u C=%.1f%s\n",(unsigned)raw_t,c,used_be?" (BE)":"");
#endif
    if (raw_t != last_oilt_raw || now - last_oilt_set_ms > 250) {
      last_oilt_raw   = raw_t;
      last_oilt_set_ms = now;
      extern lv_obj_t *ui_eoiltemperature, *ui_oiltemperaturedu, *ui_eoiltemperatureback;
      txt_1dp(c, ui_eoiltemperature);
      lv_obj_set_style_text_color(ui_eoiltemperature,  lv_color_hex(0xFFFFFF), 0);
      lv_obj_set_style_text_color(ui_oiltemperaturedu, lv_color_hex(0xFFFFFF), 0);
      set_visible(ui_eoiltemperatureback, true);
    }
  }

  // Voltage
  {
    bool used_be = false;
    uint16_t raw_v = u16_auto(&fr.data[4], "volt", &used_be);
    double v = raw_v / 10.0;
#if DEBUG_CAN
    std::printf("[CAN] 2002 volt_raw=%u V=%.1f%s\n",(unsigned)raw_v,v,used_be?" (BE)":"");
#endif
    if (raw_v != last_volt_raw || now - last_volt_set_ms > 250) {
      last_volt_raw   = raw_v;
      last_volt_set_ms = now;
      extern lv_obj_t *ui_evoltage, *ui_voltagedu, *ui_evoltageback;
      txt_1dp(v, ui_evoltage);
      lv_obj_set_style_text_color(ui_evoltage,  lv_color_hex(0xFFFFFF), 0);
      lv_obj_set_style_text_color(ui_voltagedu, lv_color_hex(0xFFFFFF), 0);
      set_visible(ui_evoltageback, true);
    }
  }
}

// 0x2003 — Gear @ byte 0
static uint8_t last_gear = 0xFF;
static void handle_2003(const CanFrame &fr) {
  uint8_t g = fr.data[0];
#if DEBUG_CAN
  std::printf("[CAN] 2003 gear=%u\n", (unsigned)g);
#endif
  if (g != last_gear) {
    last_gear = g;
    extern lv_obj_t *ui_egear;
    txt_u16(g, ui_egear);
  }
}

/* --------- LVGL flush --------- */
static void sdl_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p) {
  int w = area->x2 - area->x1 + 1;
  SDL_Rect rect{ area->x1, area->y1, w, area->y2 - area->y1 + 1 };
  SDL_UpdateTexture(g_tex, &rect, color_p, w * (int)sizeof(lv_color_t));
  g_need_present = true;
  lv_disp_flush_ready(drv);
}

int main(int argc, char *argv[]) {
  // SDL
  SDL_SetHint(SDL_HINT_RENDER_VSYNC, "1");
  SDL_SetHint(SDL_HINT_RENDER_BATCHING, "1");
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
    std::fprintf(stderr, "SDL init failed: %s\n", SDL_GetError());
    return 1;
  }

  g_win = SDL_CreateWindow("Dash",
                           SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                           SCR_W, SCR_H,
                           SDL_WINDOW_SHOWN | SDL_WINDOW_BORDERLESS);
  SDL_SetWindowFullscreen(g_win, SDL_WINDOW_FULLSCREEN_DESKTOP);
  g_ren = SDL_CreateRenderer(g_win, -1, 0);
  g_tex = SDL_CreateTexture(g_ren, SDL_PIXELFORMAT_RGB565,
                            SDL_TEXTUREACCESS_STREAMING, SCR_W, SCR_H);
  SDL_SetTextureBlendMode(g_tex, SDL_BLENDMODE_NONE);

  // SPI LEDs → start dark
  if (!g_spi.open("/dev/spidev0.1", 2400000)) {
    std::fprintf(stderr, "SPI LED open failed: %s\n", g_spi.last_error().c_str());
  } else {
    leds_off();
  }

  // LVGL
  lv_init();
  lv_disp_draw_buf_init(&g_draw_buf, g_buf1, g_buf2, SCR_W * 160);
  lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res  = SCR_W;
  disp_drv.ver_res  = SCR_H;
  disp_drv.draw_buf = &g_draw_buf;
  disp_drv.flush_cb = sdl_flush;
  lv_disp_drv_register(&disp_drv);

  ui_init();

  // RPM overlay defaults
  extern lv_obj_t *ui_erpmbackswitchup, *ui_erpmbackswitchdown;
  set_visible(ui_erpmbackswitchup, false);
  set_visible(ui_erpmbackswitchdown, false);

  // CAN
  char defaultcan[100];
  if (argc > 1) {
    std::strcpy(defaultcan, argv[1]);
  } else {
    std::strcpy(defaultcan, "can0");
  }
  SocketCan can(defaultcan);
  if (!can.open()) std::fprintf(stderr, "CAN open failed (is can0 up?)\n");

  bool quit = false;
  uint32_t last_tick = SDL_GetTicks();
  g_last_rpm_framems = last_tick;

  while (!quit) {
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
      if (e.type == SDL_QUIT) quit = true;
      if (e.type == SDL_KEYDOWN &&
          (e.key.keysym.sym == SDLK_ESCAPE || e.key.keysym.sym == SDLK_q)) quit = true;
    }

    // CAN frames
    for (int i = 0; i < MAX_CAN_PER_FRAME; ++i) {
      auto fr = can.read_nonblock();
      if (!fr) break;
      const uint32_t id = fr->id & 0x1FFFFFFF;
      switch (id) {
        case 0x2000: handle_2000(*fr); break;
        case 0x2001: handle_2001(*fr); break;
        case 0x2002: handle_2002(*fr); break;
        case 0x2003: handle_2003(*fr); break;
        default: break;
      }
    }

    // LED watchdog: no RPM frames → off
    const uint32_t now = SDL_GetTicks();
    if ((now - g_last_rpm_framems) > RPM_TIMEOUT_MS) {
      leds_off();
    }

    // LVGL
    uint32_t delta = now - last_tick;
    last_tick = now;
    if (delta > 30) delta = 30;
    lv_tick_inc(delta);
    lv_task_handler();

    if (g_need_present) {
      SDL_RenderCopy(g_ren, g_tex, nullptr, nullptr);
      SDL_RenderPresent(g_ren);
      g_need_present = false;
    }
    SDL_Delay(1);
  }

  // Cleanup
  leds_off(); g_spi.close();
  SDL_DestroyTexture(g_tex); SDL_DestroyRenderer(g_ren);
  SDL_DestroyWindow(g_win); SDL_Quit();
  return 0;
}

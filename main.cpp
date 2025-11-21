// Raspberry Pi dash for DTAFast T8+
// LVGL + SDL2 + SocketCAN + WS2812 via rpi_ws281x (GPIO PWM/PCM)
// CAN map:
//   0x2000: RPM      @ 0..1 (U16 / 1)
//   0x2001: OilPres  @ 6..7 (U16 / 10.0 kPa)
//   0x2002: OilTemp  @ 2..3 (U16 / 10.0 °C) ; Voltage @ 4..5 (U16 / 10.0 V)

#include <cstdio>
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <vector>
#include <SDL2/SDL.h>
#include <cstring>
#include <iostream>   // LED test includes
#include <unistd.h>   // LED test includes (sleep/usleep)
#include <ws2811.h>   // LED test includes

extern "C" {
  #include "lvgl.h"
  #include "ui.h"
}

#include "socketcan.hpp"
#include "config.h"     // must provide RPM_MAX, etc.

// =================== Display config ===================
static constexpr int SCR_W = 800;
static constexpr int SCR_H = 480;

// =================== CAN throttle =====================
static constexpr int MAX_CAN_PER_FRAME = 300;

// =================== LED strip (ws281x) ===============
// Use rpi_ws281x (no spi_ws2812.hpp). Supported pins for WS281X: 18,12,13,19.
static constexpr int   LED_PIN = 18;                 // <-- set your GPIO here
static constexpr int   LED_COUNT = 19;
static constexpr int   LED_BRIGHTNESS = 128;         // 0..255
static constexpr uint32_t FLICKER_INTERVAL_MS = 20;  // flash cadence for ≥85%
static constexpr uint32_t RPM_TIMEOUT_MS      = 400; // blank if no RPM frame

// ws281x controller
static ws2811_t g_leds;

// GRB color builder
static inline uint32_t grb(uint8_t r, uint8_t g, uint8_t b){
  return (uint32_t(g) << 16) | (uint32_t(r) << 8) | uint32_t(b);
}

static bool     g_leds_on   = false;
static bool     g_flash_on  = true;
static uint32_t g_last_flash_ms    = 0;
static uint32_t g_last_rpm_framems = 0;

// LED helpers
static inline void leds_clear_all(){
  for (int i=0;i<LED_COUNT;++i) g_leds.channel[0].leds[i] = 0;
}
static inline void leds_show(){
  ws2811_render(&g_leds);
  g_leds_on = true;
}
static inline void leds_off(){
  if (!g_leds_on) return;
  leds_clear_all();
  ws2811_render(&g_leds);
  g_leds_on = false;
}
static inline void leds_set_rgb(int i, uint8_t r, uint8_t g, uint8_t b){
  if (i<0 || i>=LED_COUNT) return;
  g_leds.channel[0].leds[i] = grb(r,g,b); // strip_type = GRB below
}

// Progressive RPM LEDs:
// 0–50%  = GREEN
// 50–75% = AMBER
// 75–85% = RED
// ≥85%   = PURPLE and the whole lit section flashes (F1 style)
static void updateRPMLEDs_progress(uint16_t rpm, uint32_t now_ms){
  if (rpm == 0) { leds_off(); return; }

  const float max_rpm = float(RPM_MAX);
  if (max_rpm <= 0.0f) { leds_off(); return; }

  float pct = std::min(1.0f, rpm / (float)RPM_DISPLAY_MAX);      // 0..1
  int   lit = int(std::round(pct * LED_COUNT));   // LEDs to light (0..LED_COUNT)

  const bool should_flash = (pct >= 0.85);
  if (should_flash) {
    if (now_ms - g_last_flash_ms >= FLICKER_INTERVAL_MS) {
      g_last_flash_ms = now_ms;
      g_flash_on = !g_flash_on;
    }
    if (!g_flash_on) { leds_off(); return; }      // flash OFF frame
  }

  leds_clear_all();
  for (int i = 0; i < lit; ++i) {
    float pos = float(i + 1) / float(LED_COUNT);  // position 0..1 across strip
    uint8_t r=0,g=0,b=0;
    if      (pos <= 0.85){ g=0;   r=255; b=0;   } // red
    else                   { g=0; r=128;   b=128; } // PURPLE
    leds_set_rgb(i, g, r, b);
  }
  leds_show();
}

// ===================== CAN parsing =====================
static uint16_t last_rpm_raw=0xFFFF, last_oilp_raw=0xFFFF, last_oilt_raw=0xFFFF, last_volt_raw=0xFFFF;

static inline uint16_t u16_le(const uint8_t *d){ return uint16_t(d[0] | (uint16_t(d[1])<<8)); }
static inline uint16_t u16_be(const uint8_t *d){ return uint16_t((uint16_t(d[0])<<8) | d[1]); }
static inline uint16_t u16_auto(const uint8_t *d, const char *name, bool *used_be) {
  uint16_t le=u16_le(d);
  if (le != 0){ if(used_be) *used_be=false; return le; }
  uint16_t be=u16_be(d);
#if 1
  std::printf("[CAN] %s: LE=0 using BE=%u\n", name, (unsigned)be);
#endif
  if(used_be) *used_be=true;
  return be;
}

// extern UI objects
extern "C" lv_obj_t *ui_erpmbar;
extern "C" lv_obj_t *ui_erpm;
extern "C" lv_obj_t *ui_erpmbackswitchup;
extern "C" lv_obj_t *ui_erpmbackswitchdown;
extern "C" lv_obj_t *ui_eoilpressure;
extern "C" lv_obj_t *ui_oilpressuredu;
extern "C" lv_obj_t *ui_eoilpressureback;
extern "C" lv_obj_t *ui_eoiltemperature;
extern "C" lv_obj_t *ui_oiltemperaturedu;
extern "C" lv_obj_t *ui_eoiltemperatureback;
extern "C" lv_obj_t *ui_evoltage;
extern "C" lv_obj_t *ui_voltagedu;
extern "C" lv_obj_t *ui_evoltageback;
extern "C" lv_obj_t *ui_egear;


//0x2000 rpm
static void handle_2000(const CanFrame &fr){
  bool used_be=false;
  uint16_t rpm = u16_auto(&fr.data[0], "rpm", &used_be);
  

#if 1
  std::printf("[CAN] 2000 rpm=%u%s\n", (unsigned)rpm, used_be?" (BE)":"");
#endif

  if (last_rpm_raw == 0xFFFF || std::abs(int(rpm) - int(last_rpm_raw)) >= 1) {
    last_rpm_raw = rpm;
    char buf32[32];
    std::snprintf(buf32,sizeof(buf32),"%u",(unsigned)rpm);
    lv_label_set_text(ui_erpm, buf32);
    lv_bar_set_value(ui_erpmbar, rpm, LV_ANIM_OFF);
  }

  uint16_t raw_t = u16_auto(&fr.data[4], "coolT", &used_be);
  double c = raw_t;
#if 1
  std::printf("[CAN] 2000 coolt_raw=%u C=%.1f%s\n",(unsigned)raw_t,c,used_be?" (BE)":"");
#endif
  if (raw_t != last_oilt_raw){
    last_oilt_raw = raw_t;
    char buf32[32];
    std::snprintf(buf32,sizeof(buf32),"%.1f",c);
    lv_label_set_text(ui_eoiltemperature, buf32);
    lv_obj_set_style_text_color(ui_eoiltemperature,  lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_color(ui_oiltemperaturedu, lv_color_hex(0xFFFFFF), 0);
    lv_obj_add_flag(ui_eoiltemperatureback, LV_OBJ_FLAG_HIDDEN); // visible already
  }

  // Update LEDs based on CAN RPM moved to main loop
 
}

//0x2001 pressure
static void handle_2001(const CanFrame &fr){
  bool used_be=false;
  uint16_t raw = u16_auto(&fr.data[6], "oilP", &used_be);
  double kpa = raw/100.0;
#if 1
  std::printf("[CAN] 2001 oilP_raw=%u kPa=%.1f%s\n",(unsigned)raw,kpa,used_be?" (BE)":"");
#endif
  if (raw != last_oilp_raw){
    last_oilp_raw = raw;
    char buf32[32];
    std::snprintf(buf32,sizeof(buf32),"%.1f",kpa);
    lv_label_set_text(ui_eoilpressure, buf32);
    lv_obj_set_style_text_color(ui_eoilpressure,  lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_color(ui_oilpressuredu, lv_color_hex(0xFFFFFF), 0);
    lv_obj_add_flag(ui_eoilpressureback, LV_OBJ_FLAG_HIDDEN);
  }
}

//0x2002 temperature voltage
static void handle_2002(const CanFrame &fr){
  bool used_be=false;
  uint16_t raw_v = u16_auto(&fr.data[4], "volt", &used_be);
  double v = raw_v / 10.0;
#if 1
  std::printf("[CAN] 2002 volt_raw=%u V=%.1f%s\n",(unsigned)raw_v,v,used_be?" (BE)":"");
#endif
  if (raw_v != last_volt_raw){
    last_volt_raw = raw_v;
    char buf32[32];
    std::snprintf(buf32,sizeof(buf32),"%.1f",v);
    lv_label_set_text(ui_evoltage, buf32);
    lv_obj_set_style_text_color(ui_evoltage,  lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_color(ui_voltagedu, lv_color_hex(0xFFFFFF), 0);
    lv_obj_add_flag(ui_evoltageback, LV_OBJ_FLAG_HIDDEN);
  }
}

// 0x2003 — Gear
static void handle_2003(const CanFrame &fr){
  uint8_t g0 = fr.data[0];
  uint8_t g1 = fr.data[1];
  uint8_t g  = g0 ? g0 : g1;
  if (g == 0) lv_label_set_text(ui_egear, "N");
  else {
    char buf32[32];
    std::snprintf(buf32,sizeof(buf32),"%u",(unsigned)g);
    lv_label_set_text(ui_egear, buf32);
  }
}

// ---------- LVGL flush ----------
static lv_disp_draw_buf_t g_draw_buf;
static lv_color_t g_buf1[SCR_W * 160];
static lv_color_t g_buf2[SCR_W * 160];
static SDL_Window*   g_win = nullptr;
static SDL_Renderer* g_ren = nullptr;
static SDL_Texture*  g_tex = nullptr;
static bool g_need_present = false;

static void sdl_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p){
  int w = area->x2 - area->x1 + 1;
  SDL_Rect rect{ area->x1, area->y1, w, area->y2 - area->y1 + 1 };
  SDL_UpdateTexture(g_tex, &rect, color_p, w * (int)sizeof(lv_color_t));
  g_need_present = true;
  lv_disp_flush_ready(drv);
}

int main(int argc, char *argv[]){
  // ---------- ws281x init ----------
  std::memset(&g_leds, 0, sizeof(ws2811_t));
  g_leds.freq                 = WS2811_TARGET_FREQ;
  g_leds.dmanum               = 10;
  g_leds.channel[0].gpionum   = LED_PIN;         // set your pin
  g_leds.channel[0].count     = LED_COUNT;
  g_leds.channel[0].invert    = 0;
  g_leds.channel[0].brightness= LED_BRIGHTNESS;
  g_leds.channel[0].strip_type= WS2811_STRIP_GRB; // common for WS2812B
  // channel[1] left at zeros

  {
    ws2811_return_t ret = ws2811_init(&g_leds);
    if (ret != WS2811_SUCCESS){
      std::fprintf(stderr, "ws2811_init failed: %s\n", ws2811_get_return_t_str(ret));
      return 1;
    }
    leds_off(); // start blank
  }

  // ---------- SDL ----------
  SDL_SetHint(SDL_HINT_RENDER_VSYNC, "1");
  SDL_SetHint(SDL_HINT_RENDER_BATCHING, "1");
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
    std::fprintf(stderr, "SDL_Init failed\n");
    ws2811_fini(&g_leds);
    return 1;
  }

  g_win = SDL_CreateWindow("Dash", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCR_W, SCR_H, SDL_WINDOW_SHOWN | SDL_WINDOW_BORDERLESS);
  SDL_SetWindowFullscreen(g_win, SDL_WINDOW_FULLSCREEN_DESKTOP);
  g_ren = SDL_CreateRenderer(g_win, -1, 0);
  g_tex = SDL_CreateTexture(g_ren, SDL_PIXELFORMAT_RGB565, SDL_TEXTUREACCESS_STREAMING, SCR_W, SCR_H);
  SDL_SetTextureBlendMode(g_tex, SDL_BLENDMODE_NONE);

  // ---------- LVGL ----------
  lv_init();
  lv_disp_draw_buf_init(&g_draw_buf, g_buf1, g_buf2, SCR_W * 160);
  lv_disp_drv_t disp_drv; lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = SCR_W; disp_drv.ver_res = SCR_H;
  disp_drv.draw_buf = &g_draw_buf; disp_drv.flush_cb = sdl_flush;
  lv_disp_drv_register(&disp_drv);

  ui_init();
  lv_obj_add_flag(ui_erpmbackswitchup,   LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(ui_erpmbackswitchdown, LV_OBJ_FLAG_HIDDEN);

  // ---------- CAN ----------
  char ifname[64];
  if (argc > 1) std::strncpy(ifname, argv[1], sizeof(ifname)-1), ifname[sizeof(ifname)-1]=0;
  else std::strcpy(ifname, "can0");
  SocketCan can(ifname);
  can.open();

  bool quit=false;
  uint32_t last_tick=SDL_GetTicks();
  g_last_rpm_framems = last_tick;

  // ---------- Main loop ----------
  while(!quit){
    SDL_Event e;
    while(SDL_PollEvent(&e)){
      if(e.type==SDL_QUIT) quit=true;
      if(e.type==SDL_KEYDOWN && (e.key.keysym.sym==SDLK_ESCAPE || e.key.keysym.sym==SDLK_q)) quit=true;
    }

    for(int i=0;i<MAX_CAN_PER_FRAME;++i){
      auto fr = can.read_nonblock();
      if(!fr) break;
      uint32_t id = fr->id & 0x1FFFFFFF;
      switch(id){
        case 0x2000: handle_2000(*fr); break;
        case 0x2001: handle_2001(*fr); break;
        case 0x2002: handle_2002(*fr); break;
        case 0x2003: handle_2003(*fr); break;
        default: break;
      }

    }
    g_last_rpm_framems = SDL_GetTicks();
    updateRPMLEDs_progress(last_rpm_raw, SDL_GetTicks());

    // LED watchdog: blank strip if no RPM frames recently
    uint32_t now = SDL_GetTicks();
    // if ((now - g_last_rpm_framems) > RPM_TIMEOUT_MS) {
    //   leds_off();
    // }

    // LVGL tick/handler
    uint32_t delta = now - last_tick; last_tick = now;
    if (delta > 30) delta = 30;
    lv_tick_inc(delta);
    lv_task_handler();

    if (g_need_present){
      SDL_RenderCopy(g_ren, g_tex, nullptr, nullptr);
      SDL_RenderPresent(g_ren);
      g_need_present = false;
    }
    SDL_Delay(1);
  }

  // ---------- Shutdown ----------
  leds_off();
  ws2811_fini(&g_leds);

  SDL_DestroyTexture(g_tex); SDL_DestroyRenderer(g_ren);
  SDL_DestroyWindow(g_win); SDL_Quit();
  return 0;
}

#include <cstdio>
#include <cstdint>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <vector>
#include <SDL2/SDL.h>

extern "C" {
  #include "lvgl.h"   
  #include "ui.h"     
}

#include "socketcan.hpp"   
#include "config.h"        
#include "spi_ws2812.hpp"  

// ---------------- WS2812 over SPI ----------------
static SpiWs2812 g_spi;
static std::vector<uint8_t> g_led_grb;   
static constexpr int   LED_COUNT           = 19;
static constexpr float LED_BRIGHTNESS_SCALE= 0.20f; // 20% brightness

static constexpr uint32_t LED_COLOR_GREEN  = 0x00FF00;
static constexpr uint32_t LED_COLOR_YELLOW = 0xFFFF00;
static constexpr uint32_t LED_COLOR_RED    = 0xFF0000;
static constexpr uint32_t LED_COLOR_PURPLE = 0xB400FF;
static constexpr uint32_t LED_COLOR_OFF    = 0x000000;
static constexpr uint32_t FLICKER_INTERVAL_MS = 20;

static uint16_t g_last_led_rpm_raw = 0xFFFF;
static uint32_t g_last_flash_ms = 0;
static bool     g_flash_on = true;

// ---------------- screen / LVGL ----------------
static constexpr int SCR_W = 800;
static constexpr int SCR_H = 480;
static constexpr int MAX_CAN_PER_FRAME = 300;

static SDL_Window*   g_win = nullptr;
static SDL_Renderer* g_ren = nullptr;
static SDL_Texture*  g_tex = nullptr;

static lv_disp_draw_buf_t g_draw_buf;
static lv_color_t g_buf1[SCR_W * 160];
static lv_color_t g_buf2[SCR_W * 160];
static bool g_need_present = false;

static char buf32[32];
static inline void txt_u16(uint16_t v, lv_obj_t* o){ lv_snprintf(buf32,sizeof(buf32),"%u",(unsigned)v); lv_label_set_text(o,buf32); }
static inline void txt_f(double v, lv_obj_t* o,int p=1){ char f[8]; std::snprintf(f,sizeof(f),"%%.%df",p); lv_snprintf(buf32,sizeof(buf32),f,v); lv_label_set_text(o,buf32); }
static inline void set_visible(lv_obj_t* o, bool vis){ if(!o) return; if(vis) lv_obj_clear_flag(o,LV_OBJ_FLAG_HIDDEN); else lv_obj_add_flag(o,LV_OBJ_FLAG_HIDDEN); }

// ---------------- LED helpers ----------------
static inline void leds_set(int idx, uint32_t rgb){
  if (idx < 0 || idx >= LED_COUNT) return;
  uint8_t r = (rgb>>16)&0xFF, g = (rgb>>8)&0xFF, b = rgb&0xFF;
  r = (uint8_t)(r * LED_BRIGHTNESS_SCALE);
  g = (uint8_t)(g * LED_BRIGHTNESS_SCALE);
  b = (uint8_t)(b * LED_BRIGHTNESS_SCALE);
  g_led_grb[idx*3+0] = g;
  g_led_grb[idx*3+1] = r;
  g_led_grb[idx*3+2] = b;
}
static inline void leds_clear_all(){ std::fill(g_led_grb.begin(), g_led_grb.end(), 0); }
static inline void leds_show(){ (void)g_spi.show(g_led_grb.data(), LED_COUNT); }

static void led_solid_white_test(){
  for(int i=0;i<LED_COUNT;i++) leds_set(i,0xFFFFFF);
  leds_show(); SDL_Delay(300);
  leds_clear_all(); leds_show();
}

static void updateRPMLEDs(uint16_t rpm_raw, uint32_t now_ms){
  bool over_rev = rpm_raw > RPM_MAX;
  if (over_rev){
    if (now_ms - g_last_flash_ms >= FLICKER_INTERVAL_MS){ g_last_flash_ms = now_ms; g_flash_on = !g_flash_on; }
  } else g_flash_on = true;

  const double min_rpm = 0.0;
  double max_rpm = RPM_MAX * 1.2; if (max_rpm <= min_rpm + 1) max_rpm = min_rpm + 1;

  const double clamped = std::min(std::max((double)rpm_raw, min_rpm), max_rpm);
  int numOn = (int)std::lround((clamped - min_rpm)/(max_rpm - min_rpm)*LED_COUNT);
  if (numOn < 0) numOn = 0; if (numOn > LED_COUNT) numOn = LED_COUNT;

  int r_end = (int)std::lround((RPM_MAX - min_rpm)/(max_rpm - min_rpm)*LED_COUNT);
  if (r_end < LED_COUNT/2) r_end = LED_COUNT/2; if (r_end > LED_COUNT) r_end = LED_COUNT;

  leds_clear_all();
  if (!over_rev || (over_rev && g_flash_on)){
    for(int i=0;i<numOn;i++){
      uint32_t c = LED_COLOR_OFF;
      if      (i < (int)(LED_COUNT*0.25)) c = LED_COLOR_GREEN;
      else if (i < (int)(LED_COUNT*0.50)) c = LED_COLOR_YELLOW;
      else if (i < r_end)                 c = LED_COLOR_RED;
      else                                c = LED_COLOR_PURPLE;
      leds_set(i,c);
    }
  }
  leds_show();
}

// ---------------- values cache ----------------
static uint16_t last_speed_raw = 0xFFFF;   
static uint16_t last_rpm_raw   = 0xFFFF;  
static uint16_t last_volt_raw  = 0xFFFF;   
static uint16_t last_oilp_raw  = 0xFFFF;   
static uint16_t last_oilt_raw  = 0xFFFF;   
static uint8_t  last_gear      = 0xFF;

static inline uint16_t be16(const uint8_t* d){ return (uint16_t)d[0]<<8 | d[1]; }

static void on_2000(const CanFrame& rx){
  uint16_t rpm   = be16(&rx.data[0]);  
  uint16_t speed = be16(&rx.data[2]); 
  if (last_rpm_raw == 0xFFFF || std::abs((int)rpm - (int)last_rpm_raw) >= 5){
    last_rpm_raw = rpm;
    extern lv_obj_t *ui_erpm, *ui_erpmbar;
    txt_u16(rpm, ui_erpm);
    lv_bar_set_value(ui_erpmbar, rpm, LV_ANIM_OFF);
  }
  if (speed != last_speed_raw){
    last_speed_raw = speed;
    extern lv_obj_t *ui_espeed, *ui_espeedarc;
    uint16_t sp = speed / 10; // back to km/h
    txt_u16(sp, ui_espeed);
    lv_arc_set_value(ui_espeedarc, sp);
  }
  extern lv_obj_t *ui_erpmbackswitchup, *ui_erpmbackswitchdown;
  bool in_low_band  = rpm > 0;          // green on when >0
  bool in_high_band = rpm <= RPM_MAX;   // red ON while <= max
  set_visible(ui_erpmbackswitchdown, in_low_band);
  set_visible(ui_erpmbackswitchup,   in_high_band);

  updateRPMLEDs(rpm, SDL_GetTicks());
}

static void on_2001(const CanFrame& rx){
  uint16_t v10   = be16(&rx.data[0]); 
  uint16_t p10   = be16(&rx.data[2]); 
  uint16_t t10K  = be16(&rx.data[4]); 

  if (v10 != last_volt_raw){
    last_volt_raw = v10;
    double v = v10 / 10.0;
    extern lv_obj_t *ui_evoltage, *ui_voltagedu, *ui_evoltageback;
    txt_f(v, ui_evoltage, 1);
    bool low = v < VOLTAGE_MIN;
    lv_obj_set_style_text_color(ui_evoltage,  lv_color_hex(low ? 0xFFFFFF : 0x555555), 0);
    lv_obj_set_style_text_color(ui_voltagedu, lv_color_hex(low ? 0xFFFFFF : 0x555555), 0);
    set_visible(ui_evoltageback, low);
  }

  if (p10 != last_oilp_raw){
    last_oilp_raw = p10;
    double p = p10 / 10.0 - 0.0; 
    extern lv_obj_t *ui_eoilpressure, *ui_oilpressuredu, *ui_eoilpressureback;
    txt_f(p, ui_eoilpressure, 1);
    bool low = p < PRESSURE_MIN;
    lv_obj_set_style_text_color(ui_eoilpressure,  lv_color_hex(low ? 0xFFFFFF : 0x555555), 0);
    lv_obj_set_style_text_color(ui_oilpressuredu, lv_color_hex(low ? 0xFFFFFF : 0x555555), 0);
    set_visible(ui_eoilpressureback, low);
  }

  if (t10K != last_oilt_raw){
    last_oilt_raw = t10K;
    double tC = (t10K / 10.0) - 273.15;
    extern lv_obj_t *ui_eoiltemperature, *ui_oiltemperaturedu, *ui_eoiltemperatureback;
    txt_f(tC, ui_eoiltemperature, 1);
    bool high = tC > TEMP_MAX;
    lv_obj_set_style_text_color(ui_eoiltemperature,  lv_color_hex(high ? 0xFFFFFF : 0x555555), 0);
    lv_obj_set_style_text_color(ui_oiltemperaturedu, lv_color_hex(high ? 0xFFFFFF : 0x555555), 0);
    set_visible(ui_eoiltemperatureback, high);
  }
}

static void on_2002(const CanFrame& rx){
  uint8_t gear = rx.data[0];
  if (gear != last_gear){
    last_gear = gear;
    extern lv_obj_t *ui_egear;
    txt_u16(gear, ui_egear);
  }
  bool mil = ((rx.data[7] >> 7) & 1) != 0;
  extern lv_obj_t *ui_enginedu, *ui_eengineback;
  lv_obj_set_style_text_color(ui_enginedu, lv_color_hex(mil ? 0xFFFFFF : 0x555555), 0);
  set_visible(ui_eengineback, mil);
}

// --------- LVGL v8 flush --------- 
static void sdl_flush(lv_disp_drv_t *drv, const lv_area_t *a, lv_color_t *color_p){
  int w = a->x2 - a->x1 + 1;
  SDL_Rect rect{ a->x1, a->y1, w, a->y2 - a->y1 + 1 };
  SDL_UpdateTexture(g_tex, &rect, color_p, w * (int)sizeof(lv_color_t));
  g_need_present = true;
  lv_disp_flush_ready(drv);
}

int main(){
  // SDL / window
  SDL_SetHint(SDL_HINT_RENDER_VSYNC, "1");
  SDL_SetHint(SDL_HINT_RENDER_BATCHING, "1");
  if (SDL_Init(SDL_INIT_VIDEO|SDL_INIT_TIMER) != 0){ std::fprintf(stderr,"SDL init failed: %s\n",SDL_GetError()); return 1; }

  g_win = SDL_CreateWindow("Dash", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                           SCR_W, SCR_H, SDL_WINDOW_SHOWN | SDL_WINDOW_BORDERLESS);
  SDL_SetWindowFullscreen(g_win, SDL_WINDOW_FULLSCREEN_DESKTOP);
  g_ren = SDL_CreateRenderer(g_win,-1,0);
  g_tex = SDL_CreateTexture(g_ren, SDL_PIXELFORMAT_RGB565, SDL_TEXTUREACCESS_STREAMING, SCR_W, SCR_H);
  SDL_SetTextureBlendMode(g_tex, SDL_BLENDMODE_NONE);

  // LEDs
  g_led_grb.assign(LED_COUNT*3, 0);
  if (!g_spi.open("/dev/spidev0.1", 2400000)){
    std::fprintf(stderr,"SPI LED open failed: %s\n", g_spi.last_error().c_str());
  } else {
    led_solid_white_test();
  }

  // LVGL
  lv_init();
  lv_disp_draw_buf_init(&g_draw_buf, g_buf1, g_buf2, SCR_W*160);
  lv_disp_drv_t disp_drv; lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = SCR_W; disp_drv.ver_res = SCR_H;
  disp_drv.draw_buf = &g_draw_buf; disp_drv.flush_cb = sdl_flush;
  lv_disp_drv_register(&disp_drv);

  ui_init();
  extern lv_obj_t *ui_erpmbackswitchup, *ui_erpmbackswitchdown;
  set_visible(ui_erpmbackswitchup, false);
  set_visible(ui_erpmbackswitchdown, false);

  // CAN
  SocketCan can("can0");
  if (!can.open()) std::fprintf(stderr,"CAN open failed (is can0 up?)\n");

  bool quit=false; uint32_t last_tick=SDL_GetTicks();
  while(!quit){
    SDL_Event e; while(SDL_PollEvent(&e)){ if(e.type==SDL_QUIT) quit=true; if(e.type==SDL_KEYDOWN && (e.key.keysym.sym==SDLK_ESCAPE||e.key.keysym.sym==SDLK_q)) quit=true; }

    for(int i=0;i<MAX_CAN_PER_FRAME;i++){
      auto f = can.read_nonblock(); if(!f) break;
      switch(f->id){
        case 0x2000: on_2000(*f); break;
        case 0x2001: on_2001(*f); break;
        case 0x2002: on_2002(*f); break;
        default: break;
      }
    }

    uint32_t now=SDL_GetTicks(), dt=now-last_tick; last_tick=now; if(dt>30) dt=30;
    lv_tick_inc(dt); lv_task_handler();
    if(g_need_present){ SDL_RenderCopy(g_ren,g_tex,nullptr,nullptr); SDL_RenderPresent(g_ren); g_need_present=false; }
    SDL_Delay(1);
  }

  leds_clear_all(); leds_show(); g_spi.close();
  SDL_DestroyTexture(g_tex); SDL_DestroyRenderer(g_ren); SDL_DestroyWindow(g_win); SDL_Quit();
  return 0;
}

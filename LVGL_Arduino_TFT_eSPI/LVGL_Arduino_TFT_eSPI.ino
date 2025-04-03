/*Using LVGL with Arduino requires some extra steps:
 *Be sure to read the docs here: https://docs.lvgl.io/master/integration/framework/arduino.html  */

#include <lvgl.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include "lv_tft_espi.hpp"

// Touchscreen pins
// https://github.com/espressif/arduino-esp32/blob/master/variants/jczn_2432s028r/pins_arduino.h
#define XPT2046_IRQ   CYD_TP_IRQ
#define XPT2046_MOSI  CYD_TP_MOSI
#define XPT2046_MISO  CYD_TP_MISO
#define XPT2046_CLK   CYD_TP_CLK
#define XPT2046_CS    CYD_TP_CS

// Touchscreen SPI bus
// https://randomnerdtutorials.com/cheap-yellow-display-esp32-2432s028r/
#ifdef  USE_HSPI_PORT
#define XPT2046_SPI   VSPI
#else
#define XPT2046_SPI   HSPI
#endif
SPIClass touchscreenSPI = SPIClass(XPT2046_SPI);
XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);

/*To use the built-in examples and demos of LVGL uncomment the includes below respectively.
 *You also need to copy `lvgl/examples` to `lvgl/src/examples`. Similarly for the demos `lvgl/demos` to `lvgl/src/demos`.
 *Note that the `lv_examples` library is for LVGL v7 and you shouldn't install it for this version (since LVGL v8)
 *as the examples and demos are now part of the main LVGL library. */

//#include <examples/lv_examples.h>
#include <demos/lv_demos.h>

/*Set to your screen resolution and rotation*/
#define TFT_HOR_RES   TFT_WIDTH   // 240
#define TFT_VER_RES   TFT_HEIGHT  // 320
#define TFT_ROTATION  LV_DISPLAY_ROTATION_0 // LV_DISPLAY_ROTATION_{0|90|180|270}

/*LVGL draw into this buffer, 1/10 screen size usually works well. The size is in bytes*/
#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 10 * (LV_COLOR_DEPTH / 8))
uint32_t draw_buf[DRAW_BUF_SIZE / 4];

#if LV_USE_LOG != 0
void my_print(lv_log_level_t level, const char* buf) {
  LV_UNUSED(level);
  Serial.println(buf);
  Serial.flush();
}
#endif

/*Read the touchpad*/
void my_touchpad_read(lv_indev_t* indev, lv_indev_data_t* data) {
  bool touched = touchscreen.touched();
  if (!touched) {
    data->state = LV_INDEV_STATE_RELEASED;
  } else {
    // Get Touchscreen points
    TS_Point p = touchscreen.getPoint();

    // Calibrate Touchscreen points with map function to the correct width and height
    data->point.x = map(p.x, 200, 3700, 0, TFT_HOR_RES - 1);
    data->point.y = map(p.y, 240, 3800, 0, TFT_VER_RES - 1);
    data->state = LV_INDEV_STATE_PRESSED;

    Serial.printf("x:%d, y:%d\n", data->point.x, data->point.y);
  }
}

/*use Arduinos millis() as tick source*/
static uint32_t my_tick(void) {
  return millis();
}

void setup() {
  Serial.begin(115200);
  while (millis() < 1000);

  // Start and initialize the SPI for the touchscreen
  touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  touchscreen.begin(touchscreenSPI);
  touchscreen.setRotation(TFT_ROTATION & 0x01 ? 2 : 0);  // Adjust rotation to match LVGL orientation

  lv_init();

  /*Set a tick source so that LVGL will know how much time elapsed. */
  lv_tick_set_cb(my_tick);

  /* register print function for debugging */
#if LV_USE_LOG != 0
  lv_log_register_print_cb(my_print);
#endif

  lv_display_t* disp;
  disp = lv_tft_espi_create(TFT_HOR_RES, TFT_VER_RES, draw_buf, sizeof(draw_buf));
  lv_display_set_rotation(disp, TFT_ROTATION);

  /*Initialize the input device driver*/
  lv_indev_t* indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER); /*Touchpad should have POINTER type*/
  lv_indev_set_read_cb(indev, my_touchpad_read);

#if defined(LV_DEMOS_H)

  lv_demo_widgets();
  //lv_demo_music();

#elif defined(LV_EXAMPLES_H)

  lv_example_checkbox_1();
  //lv_example_checkbox_2();

#endif

  Serial.println("Setup done");
  Serial.println(USER_SETUP_INFO);
}

void loop() {
  lv_timer_handler(); /* let the GUI do its work */
}
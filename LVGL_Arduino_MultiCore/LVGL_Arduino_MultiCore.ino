/*Using LVGL with Arduino requires some extra steps:
 *Be sure to read the docs here: https://docs.lvgl.io/master/integration/framework/arduino.html  */

// IN lv_conf.h
//  DO NOT FORGET TO SET 'LV_USE_TFT_ESPI' TO 0
//  DO NOT FORGET TO SET 'LV_USE_ILI9341' and/or 'LV_USE_ST7789'
#include <lvgl.h>

#define USE_LGFX_AUTODETECT true

#if USE_LGFX_AUTODETECT
#define LGFX_AUTODETECT
#include <LovyanGFX.h>
#else
// false: Panel driver: ILI9341 (micro-USB x 1 type)
// true : Panel driver: ST7789  (micro-USB x 1 + USB-C x 1 type)
#define DISPLAY_CYD_2USB true
#include "LGFX_ESP32_2432S028R_CYD.hpp"
#endif  // USE_LGFX_AUTODETECT

static LGFX tft;

#define DRAW_BUF_N_BUFS     2 // single (1) or double (2)
#define DRAW_BUF_N_DIVS     3 // 2 - 10

/*To use the built-in examples and demos of LVGL uncomment the includes below respectively.
 *You also need to copy `lvgl/examples` to `lvgl/src/examples`. Similarly for the demos `lvgl/demos` to `lvgl/src/demos`.
 *Note that the `lv_examples` library is for LVGL v7 and you shouldn't install it for this version (since LVGL v8)
 *as the examples and demos are now part of the main LVGL library. */

//#include <examples/lv_examples.h>
#include <demos/lv_demos.h>

/*Set to your screen resolution and rotation*/
#define TFT_HOR_RES   240 // Portrait orientation default width
#define TFT_VER_RES   320 // Portrait orientation default height
#define TFT_ROTATION  LV_DISPLAY_ROTATION_270 // LV_DISPLAY_ROTATION_{0|90|180|270}

/*LVGL draw into this buffer, 1/10 screen size usually works well. The size is in bytes*/
#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / DRAW_BUF_N_DIVS * (LV_COLOR_DEPTH / 8))
static uint8_t* draw_buf[2] = { NULL, };

#if LV_USE_LOG != 0
static void my_print(lv_log_level_t level, const char *buf) {
  LV_UNUSED(level);
  Serial.println(buf);
  Serial.flush();
}
#endif

//----------------------------------------------------------------------
// Configuration for flush task
//----------------------------------------------------------------------
#define FLUSH_TASK_STACK_SZ 2048
#define FLUSH_TASK_PRIORITY 1
#define FLUSH_TASK_CORE     0

typedef struct {
  int32_t x, y, w, h;
  uint8_t *px_map;
} MessageQueue_t;

// Message queues and semaphores for handshaking
static TaskHandle_t taskHandle;
static QueueHandle_t queHandle;
static SemaphoreHandle_t semHandle;

inline __attribute__((always_inline))
static void flush_draw_buf(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t *px_map) {
  tft.setAddrWindow(x, y, w, h);
  tft.pushPixelsDMA((lgfx::rgb565_t *)px_map, w * h);  // { startWrite(); writePixelsDMA(data, len); endWrite(); }
}

static void flush_task(void *pvParameters) {
  MessageQueue_t queue;
  while (1) {
    if (xQueueReceive(queHandle, &queue, portMAX_DELAY) != pdTRUE) {
      Serial.println("unable to receive queue.");
      continue;
    }

    flush_draw_buf(queue.x, queue.y, queue.w, queue.h, queue.px_map);

    if (xSemaphoreGive(semHandle) != pdTRUE) {
      Serial.println("unable to give semaphore.");
    }
  }
}

static void flush_wait(lv_display_t *disp) {
  if (xSemaphoreTake(semHandle, portMAX_DELAY) != pdTRUE) {
    Serial.println("unable to take semaphore.");
  }

#if DRAW_BUF_N_BUFS == 1
  /*Call it to tell LVGL you are ready*/
  lv_display_flush_ready(disp);
#endif
}

static bool flush_init(void) {
  semHandle = xSemaphoreCreateBinary();
  queHandle = xQueueCreate(1, sizeof(MessageQueue_t));
  if (queHandle == NULL || semHandle == NULL) {
    Serial.println("unable to create queue or semaphore.");
    return false;
  }

  // Set up sender task in core 1 and start immediately
  bool ret = xTaskCreatePinnedToCore(
      flush_task, "flush_task",
      FLUSH_TASK_STACK_SZ,  // The stack size
      NULL,                 // Pass reference to a variable describing the task number
      FLUSH_TASK_PRIORITY,  // priority
      &taskHandle,          // Pass reference to task handle
      FLUSH_TASK_CORE
  );

  return (ret == pdPASS);
}

/* LVGL calls it when a rendered image needs to copied to the display*/
static void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
  MessageQueue_t queue = {
    .x = area->x1,
    .y = area->y1,
    .w = lv_area_get_width(area),
    .h = lv_area_get_height(area),
    .px_map = px_map
  };
  if (xQueueSend(queHandle, &queue, portMAX_DELAY) != pdTRUE) {
    Serial.println("unable to send queue.");
  }
}

/*Read the touchpad*/
static void my_touchpad_read(lv_indev_t *indev, lv_indev_data_t *data) {
  uint16_t x, y;
  bool touched = tft.getTouch(&x, &y);
  if (!touched) {
    data->state = LV_INDEV_STATE_RELEASED;
  } else {
    data->state = LV_INDEV_STATE_PRESSED;

    switch (tft.getRotation()) {
      case LV_DISPLAY_ROTATION_0:
        data->point.x = x;
        data->point.y = y;
        break;
      case LV_DISPLAY_ROTATION_90:
        data->point.x = y;
        data->point.y = TFT_VER_RES - x;
        break;
      case LV_DISPLAY_ROTATION_180:
        data->point.x = TFT_HOR_RES - x;
        data->point.y = TFT_VER_RES - y;
        break;
      case LV_DISPLAY_ROTATION_270:
        data->point.x = TFT_HOR_RES - y;
        data->point.y = x;
        break;
    }

    Serial.printf("x: %d (%d), y: %d (%d)\n", data->point.x, x, data->point.y, y);
  }
}

static void resolution_changed_event_cb(lv_event_t *e) {
  lv_display_t *disp = (lv_display_t *)lv_event_get_target(e);
  lv_display_rotation_t rot = lv_display_get_rotation(disp);

  /* handle rotation */
  switch (rot) {
    case LV_DISPLAY_ROTATION_0:
      tft.setRotation(0); /* Portrait orientation */
      break;
    case LV_DISPLAY_ROTATION_90:
      tft.setRotation(1); /* Landscape orientation */
      break;
    case LV_DISPLAY_ROTATION_180:
      tft.setRotation(2); /* Portrait orientation, flipped */
      break;
    case LV_DISPLAY_ROTATION_270:
      tft.setRotation(3); /* Landscape orientation, flipped */
      break;
  }
}

// Calibrate touch when enabled (optional)
static void calibrate_touch(uint16_t cal[8]) {
  // Draw guide text on the screen.
  tft.setTextDatum(textdatum_t::middle_center);
  tft.drawString("touch the arrow marker.", tft.width() >> 1, tft.height() >> 1);
  tft.setTextDatum(textdatum_t::top_left);

  // You will need to calibrate by touching the four corners of the screen.
  uint16_t fg = TFT_WHITE;
  uint16_t bg = TFT_BLACK;
  if (tft.isEPD()) {  // Electronic Paper Display
    std::swap(fg, bg);
  }

  tft.calibrateTouch(cal, fg, bg, std::max(tft.width(), tft.height()) >> 3);

  Serial.print("\nuint16_t cal[8] = { ");
  for (int i = 0; i < 8; i++) {
    Serial.printf("%d%s", cal[i], (i < 7 ? ", " : " };\n"));
  }
  Serial.print("tft.setTouchCalibrate(cal);\n");
}

static void tft_init(void) {
  tft.init();
  tft.initDMA();
  tft.setColorDepth(16);  // Set to 16-bit RGB565

  if (tft.touch()) {
    if (true) {
      const uint16_t cal[8] = {
        240,   // x_min
        3700,  // y_min
        240,   // x_min
        200,   // y_max
        3800,  // x_max
        3700,  // y_min
        3800,  // x_max
        200    // y_max
      };
      tft.setTouchCalibrate((uint16_t*)cal);
    } else {
      uint16_t cal[8];
      calibrate_touch(cal);
      tft.setTouchCalibrate(cal);
    }
  } else {
    Serial.println("Touch device not found.");
  }
}

/*use Arduinos millis() as tick source*/
static uint32_t my_tick(void) {
  return millis();
}

void setup() {
  Serial.begin(115200);
  while (millis() < 1000);

  tft_init();
  lv_init();

  if (!flush_init()) {
    Serial.println("unable to initialize flush task.");
    while (1) delay(1000);
  }

  /*Set a tick source so that LVGL will know how much time elapsed. */
  lv_tick_set_cb(my_tick);

  /* register print function for debugging */
#if LV_USE_LOG != 0
  lv_log_register_print_cb(my_print);
#endif

  lv_display_t *disp = lv_display_create(TFT_HOR_RES, TFT_VER_RES);
  lv_display_add_event_cb(disp, resolution_changed_event_cb, LV_EVENT_RESOLUTION_CHANGED, NULL);
  lv_display_set_rotation(disp, (lv_display_rotation_t)TFT_ROTATION);
  lv_display_set_flush_cb(disp, my_disp_flush);
  lv_display_set_flush_wait_cb(disp, flush_wait);

  for (int i = 0; i < DRAW_BUF_N_BUFS; i++) {
    draw_buf[i] = (uint8_t*)heap_caps_malloc(DRAW_BUF_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    Serial.printf("buf[%d]: 0x%x%s", i, draw_buf[i], (DRAW_BUF_N_BUFS == 1 || i == 1 ? "\n" : ", "));
  }

  lv_display_set_buffers(disp, draw_buf[0], draw_buf[1], DRAW_BUF_SIZE, LV_DISPLAY_RENDER_MODE_PARTIAL);

  /*Initialize the input device driver*/
  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER); /*Touchpad should have POINTER type*/
  lv_indev_set_read_cb(indev, my_touchpad_read);

#if defined(LV_DEMOS_H)

  //lv_demo_widgets();
  lv_demo_music();

#elif defined(LV_EXAMPLES_H)

  lv_example_arc_1();
  //lv_example_checkbox_1();

#endif

  Serial.println("Setup done");
}

void loop() {
  lv_timer_handler(); /* let the GUI do its work */
}
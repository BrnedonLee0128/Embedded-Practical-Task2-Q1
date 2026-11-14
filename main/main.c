#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"

/* ---------- Config ---------- */
#define PIN_LED1   GPIO_NUM_2   // Red    - Task A
#define PIN_LED2   GPIO_NUM_3   // Green  - Task B
#define PIN_LED3   GPIO_NUM_4   // Yellow - Task C
#define PIN_SDA    8
#define PIN_SCL    9
#define OLED_ADDR  0x3C
#define OLED_W     128
#define OLED_H     32
#define SCALE      2
#define TASK_PRIO  1
#define BLINKS     5

#define FORCE_YIELD  0          // 1 = taskYIELD() every 15 ms (fallback)
#define SLICE_US     15000

#define MAXSW  200
#define MAXEV  100

/* ---------- OLED (SSD1306 128x32) ---------- */
static i2c_master_dev_handle_t oled;
static uint8_t fb[OLED_W * OLED_H / 8];

typedef struct { char c; uint8_t d[5]; } glyph_t;
static const glyph_t FONT[] = {
    {' ',{0x00,0x00,0x00,0x00,0x00}},
    {'A',{0x7E,0x11,0x11,0x11,0x7E}}, {'B',{0x7F,0x49,0x49,0x49,0x36}},
    {'C',{0x3E,0x41,0x41,0x41,0x22}}, {'D',{0x7F,0x41,0x41,0x22,0x1C}},
    {'E',{0x7F,0x49,0x49,0x49,0x41}}, {'G',{0x3E,0x41,0x49,0x49,0x7A}},
    {'I',{0x00,0x41,0x7F,0x41,0x00}}, {'K',{0x7F,0x08,0x14,0x22,0x41}},
    {'L',{0x7F,0x40,0x40,0x40,0x40}}, {'N',{0x7F,0x04,0x08,0x10,0x7F}},
    {'O',{0x3E,0x41,0x41,0x41,0x3E}}, {'R',{0x7F,0x09,0x19,0x29,0x46}},
    {'S',{0x46,0x49,0x49,0x49,0x31}}, {'T',{0x01,0x01,0x7F,0x01,0x01}},
    {'U',{0x3F,0x40,0x40,0x40,0x3F}},
};

static const uint8_t *glyph_lookup(char c) {
    for (size_t i = 0; i < sizeof(FONT) / sizeof(FONT[0]); i++)
        if (FONT[i].c == c) return FONT[i].d;
    return FONT[0].d;
}

static void oled_init(i2c_master_bus_handle_t bus) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = OLED_ADDR,
        .scl_speed_hz    = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &oled));

    static const uint8_t init_seq[] = {
        0x00,
        0xAE, 0xD5, 0x80, 0xA8, 0x1F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x02,
        0x81, 0x8F, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6,
        0x2E, 0xAF,
    };
    ESP_ERROR_CHECK(i2c_master_transmit(oled, init_seq, sizeof(init_seq), 100));
}

static void oled_flush(void) {
    static const uint8_t window[] = { 0x00, 0x21, 0, OLED_W - 1, 0x22, 0, (OLED_H / 8) - 1 };
    i2c_master_transmit(oled, window, sizeof(window), 100);
    uint8_t buf[OLED_W + 1];
    buf[0] = 0x40;
    for (int pg = 0; pg < OLED_H / 8; pg++) {
        memcpy(&buf[1], &fb[pg * OLED_W], OLED_W);
        i2c_master_transmit(oled, buf, sizeof(buf), 100);
    }
}

static void fb_pixel(int x, int y) {
    if (x < 0 || x >= OLED_W || y < 0 || y >= OLED_H) return;
    fb[x + (y / 8) * OLED_W] |= (1 << (y % 8));
}

static void draw_char(int x, int y, char c) {
    const uint8_t *g = glyph_lookup(c);
    for (int col = 0; col < 5; col++)
        for (int row = 0; row < 7; row++)
            if (g[col] & (1 << row))
                for (int dx = 0; dx < SCALE; dx++)
                    for (int dy = 0; dy < SCALE; dy++)
                        fb_pixel(x + col * SCALE + dx, y + row * SCALE + dy);
}

static void draw_text(int x, int y, const char *s) {
    for (; *s; s++, x += 6 * SCALE) draw_char(x, y, *s);
}

static void oled_show(const char *l1, const char *l2) {
    memset(fb, 0, sizeof(fb));
    draw_text(0, 1, l1);
    draw_text(0, 17, l2);
    oled_flush();
}

/* ---------- Beacon tasks ---------- */
typedef struct {
    const char *name, *line1, *line2;
    gpio_num_t  led;
    uint32_t    half_ms;
    int64_t     last;
    int         n;
    int64_t     out[MAXSW], in[MAXSW];
} beacon_t;

static beacon_t beacons[] = {
    { .name = "Task A", .line1 = "TASK A", .line2 = "RUNNING", .led = PIN_LED1, .half_ms = 200 },
    { .name = "Task B", .line1 = "TASK B", .line2 = "RUNNING", .led = PIN_LED2, .half_ms = 300 },
    { .name = "Task C", .line1 = "TASK C", .line2 = "RUNNING", .led = PIN_LED3, .half_ms = 100 },
};

typedef struct { int64_t t; const char *name; const char *what; int blink; } ev_t;
static ev_t evs[MAXEV];
static int ev_n = 0;
static portMUX_TYPE ev_mux = portMUX_INITIALIZER_UNLOCKED;

static SemaphoreHandle_t oled_mtx;
static int tasks_done = 0;

static void ev_add(const beacon_t *b, const char *what, int blink) {
    int64_t t = esp_timer_get_time();
    portENTER_CRITICAL(&ev_mux);
    if (ev_n < MAXEV) evs[ev_n++] = (ev_t){t, b->name, what, blink};
    portEXIT_CRITICAL(&ev_mux);
}

// Busy-wait on timestamp. Task stays Ready -> time-sliced.
// Records every gap > 2 ms = task was switched out.
static void wait_ms(beacon_t *b, uint32_t ms) {
    int64_t end = esp_timer_get_time() + (int64_t)ms * 1000;
    int64_t now;
#if FORCE_YIELD
    int64_t slice_start = esp_timer_get_time();
#endif
    b->last = 0;
    while ((now = esp_timer_get_time()) < end) {
        if (b->last && (now - b->last) > 2000 && b->n < MAXSW) {
            b->out[b->n] = b->last;
            b->in[b->n]  = now;
            b->n++;
#if FORCE_YIELD
            slice_start = now;
#endif
        }
        b->last = now;
#if FORCE_YIELD
        if (now - slice_start >= SLICE_US) {
            taskYIELD();
            slice_start = esp_timer_get_time();
        }
#endif
    }
}

static void dump_all(void) {
    printf("==== EVENTS ====\n");
    for (int i = 0; i < ev_n; i++)
        printf("[%8lld ms] %s blink %d %s\n",
               evs[i].t / 1000, evs[i].name, evs[i].blink, evs[i].what);

    printf("==== SWITCHES (task preempted: ran / waited) ====\n");
    for (int t = 0; t < 3; t++) {
        beacon_t *b = &beacons[t];
        for (int k = 1; k < b->n; k++)
            printf("%s out@%lld ms  back@%lld ms  waited=%lld ms  ran=%lld ms\n",
                   b->name, b->out[k] / 1000, b->in[k] / 1000,
                   (b->in[k] - b->out[k]) / 1000,
                   (b->out[k] - b->in[k - 1]) / 1000);
    }
    printf("==== END ====\n");
}

static void beacon_task(void *pv) {
    beacon_t *b = (beacon_t *)pv;

    // OLED: show this task at start
    xSemaphoreTake(oled_mtx, portMAX_DELAY);
    oled_show(b->line1, b->line2);
    xSemaphoreGive(oled_mtx);

    for (int i = 1; i <= BLINKS; i++) {
        gpio_set_level(b->led, 1);
        ev_add(b, "ON", i);
        wait_ms(b, b->half_ms);

        gpio_set_level(b->led, 0);
        ev_add(b, "OFF", i);
        wait_ms(b, b->half_ms);
    }

    gpio_set_level(b->led, 0);
    ev_add(b, "DONE", BLINKS);

    xSemaphoreTake(oled_mtx, portMAX_DELAY);
    int d = ++tasks_done;
    if (d == 3) {
        oled_show("ALL TASKS", "DONE");
        dump_all();
    }
    xSemaphoreGive(oled_mtx);

    vTaskDelete(NULL);
}

void app_main(void) {
    // LEDs
    for (int i = 0; i < 3; i++) {
        gpio_reset_pin(beacons[i].led);
        gpio_set_direction(beacons[i].led, GPIO_MODE_OUTPUT);
        gpio_set_level(beacons[i].led, 0);
    }

    // I2C + OLED
    i2c_master_bus_config_t bc = {
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = PIN_SDA,
        .scl_io_num        = PIN_SCL,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bc, &bus));
    oled_init(bus);
    oled_show("STARTING", "");

    oled_mtx = xSemaphoreCreateMutex();

    vTaskDelay(pdMS_TO_TICKS(3000));   // let terminal reconnect

    // Same priority -> round-robin
    xTaskCreate(beacon_task, "Task A", 4096, &beacons[0], TASK_PRIO, NULL);
    xTaskCreate(beacon_task, "Task B", 4096, &beacons[1], TASK_PRIO, NULL);
    xTaskCreate(beacon_task, "Task C", 4096, &beacons[2], TASK_PRIO, NULL);
}
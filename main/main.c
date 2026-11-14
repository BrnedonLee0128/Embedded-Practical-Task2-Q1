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
    const char *name;
    const char *line1;
    const char *line2;
    gpio_num_t  led;
    uint32_t    half_ms;
} beacon_t;

static beacon_t beacons[] = {
    {"Task A", "TASK A", "RUNNING", PIN_LED1, 200},
    {"Task B", "TASK B", "RUNNING", PIN_LED2, 300},
    {"Task C", "TASK C", "RUNNING", PIN_LED3, 100},
};

static SemaphoreHandle_t oled_mtx;
static const beacon_t *last_shown = NULL;
static int tasks_done = 0;

// Busy-wait on timestamp. Task stays Ready -> time-sliced.
static void wait_ms(uint32_t ms) {
    int64_t end = esp_timer_get_time() + (int64_t)ms * 1000;
    while (esp_timer_get_time() < end) { }
}

static void beacon_task(void *pv) {
    const beacon_t *b = (const beacon_t *)pv;

    for (int i = 1; i <= BLINKS; i++) {
        gpio_set_level(b->led, 1);

        xSemaphoreTake(oled_mtx, portMAX_DELAY);
        if (last_shown != b) {
            last_shown = b;
            oled_show(b->line1, b->line2);
        }
        xSemaphoreGive(oled_mtx);

        printf("[%8lld ms] %s blink %d ON\n", esp_timer_get_time() / 1000, b->name, i);
        wait_ms(b->half_ms);

        gpio_set_level(b->led, 0);
        printf("[%8lld ms] %s blink %d OFF\n", esp_timer_get_time() / 1000, b->name, i);
        wait_ms(b->half_ms);
    }

    gpio_set_level(b->led, 0);

    xSemaphoreTake(oled_mtx, portMAX_DELAY);
    if (++tasks_done == 3) oled_show("ALL TASKS", "DONE");
    xSemaphoreGive(oled_mtx);

    printf("[%8lld ms] %s finished, deleting\n", esp_timer_get_time() / 1000, b->name);
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
    vTaskDelay(pdMS_TO_TICKS(3000));   // let terminal reconnect with Tera Term 
    oled_mtx = xSemaphoreCreateMutex();

    // Same priority -> round-robin
    xTaskCreate(beacon_task, "Task A", 4096, &beacons[0], TASK_PRIO, NULL);
    xTaskCreate(beacon_task, "Task B", 4096, &beacons[1], TASK_PRIO, NULL);
    xTaskCreate(beacon_task, "Task C", 4096, &beacons[2], TASK_PRIO, NULL);
}